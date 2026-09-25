# SPDX-License-Identifier: Apache-2.0
"""
Robust sensor-origin translation from a majority world-static radar scene.

No ROS, pose integration, SDK variance interpretation, or radiometric weighting.
Covariance is an experimental model with explicit floors, not calibrated error.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class FitConfig:
    doppler_sign: int = 1  # +1: input positive receding; -1: positive approaching.
    residual_threshold: float = 0.20
    min_inliers: int = 8
    min_inlier_fraction: float = 0.60
    max_condition: float = 30.0
    ransac_trials: int = 128
    noise_floor: float = 0.05  # radial m/s, independently of near-zero fit residuals
    velocity_std_floor: float = 0.10  # m/s, added to every estimated axis
    max_velocity_std: float = 1.0
    max_speed: float = 15.0

    def __post_init__(self):
        if type(self.doppler_sign) is not int or self.doppler_sign not in (-1, 1):
            raise ValueError('doppler_sign must be +1 (receding) or -1 (approaching)')
        if type(self.min_inliers) is not int or not 4 <= self.min_inliers <= 4096:
            raise ValueError('min_inliers must be an integer within 4..4096')
        if type(self.ransac_trials) is not int or not 1 <= self.ransac_trials <= 1024:
            raise ValueError('ransac_trials must be an integer within 1..1024')
        positive = (self.residual_threshold, self.noise_floor, self.velocity_std_floor,
                    self.max_velocity_std, self.max_speed, self.max_condition)
        if not all(math.isfinite(x) and x > 0 for x in positive):
            raise ValueError('Fit scales and limits must be finite and positive')
        if not .5 < self.min_inlier_fraction <= 1:
            raise ValueError('min_inlier_fraction must be within (0.5, 1]')
        if self.max_condition <= 1 or self.max_velocity_std <= self.velocity_std_floor:
            raise ValueError('Invalid geometry or uncertainty limit')


@dataclass
class FitResult:
    valid: bool
    reason: str
    velocity: np.ndarray | None = None
    covariance: np.ndarray | None = None
    inliers: np.ndarray | None = None
    residuals: np.ndarray | None = None
    condition: float | None = None
    rmse: float | None = None


def condition(matrix):
    if matrix.shape[0] < 3:
        return float('inf')
    singular = np.linalg.svd(matrix, compute_uv=False)
    return float(singular[0] / singular[-1]) if singular[-1] > 1e-9 else float('inf')


def huber_weights(residuals, residual_threshold):
    """Bounded IRLS weights in (0.05, 1]; the Huber knee is half the inlier gate."""
    absolute = np.abs(residuals)
    return np.clip(residual_threshold * .5 / np.maximum(absolute, 1e-12), .05, 1)


def fit_velocity(xyz, radial_speed, config=FitConfig()):
    """
    Fit d = -u.v in 3D. Unknown vertical motion is never forced to zero.

    RANSAC finds a majority consensus; bounded Huber reweighting refines it.
    A valid result still depends on correct Doppler sign and a static majority.
    Residuals are in the positive-receding convention after sign adaptation.
    """
    xyz = np.asarray(xyz, dtype=float)
    speeds = np.asarray(radial_speed, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or speeds.shape != (len(xyz),):
        return FitResult(False, 'invalid_shape')
    if len(xyz) < config.min_inliers:
        return FitResult(False, 'insufficient_points')
    if len(xyz) > 4096:
        return FitResult(False, 'too_many_points')
    if not np.isfinite(xyz).all() or not np.isfinite(speeds).all():
        return FitResult(False, 'nonfinite_measurements')
    ranges = np.linalg.norm(xyz, axis=1)
    if np.any(ranges <= 1e-6) or not np.isfinite(ranges).all():
        return FitResult(False, 'invalid_bearing')
    matrix = xyz / ranges[:, None]
    doppler = config.doppler_sign * speeds
    if condition(matrix) > config.max_condition:
        return FitResult(False, 'unobservable_geometry')
    rng = np.random.default_rng(0)  # Reproducible comparisons on the same frame.
    best, best_score = None, (-1, -float('inf'))
    for _ in range(config.ransac_trials):
        sample = rng.choice(len(matrix), 3, replace=False)
        if condition(matrix[sample]) > config.max_condition:
            continue
        candidate = np.linalg.lstsq(matrix[sample], -doppler[sample], rcond=None)[0]
        residual = np.abs(doppler + matrix @ candidate)
        mask = residual <= config.residual_threshold
        score = (int(mask.sum()), -float(np.sum(np.minimum(
            residual / config.residual_threshold, 1.0) ** 2)))
        if score > best_score:
            best, best_score = mask, score
    needed = max(config.min_inliers, math.ceil(config.min_inlier_fraction * len(matrix)))
    if best is None or best.sum() < needed:
        return FitResult(False, 'insufficient_consensus')
    if condition(matrix[best]) > config.max_condition:
        return FitResult(False, 'unobservable_consensus')
    # Refine a fixed consensus; then reclassify every input with the final model.
    a, b = matrix[best], -doppler[best]
    velocity = np.linalg.lstsq(a, b, rcond=None)[0]
    for _ in range(5):
        root_weights = np.sqrt(huber_weights(a @ velocity - b, config.residual_threshold))
        refined = np.linalg.lstsq(a * root_weights[:, None], b * root_weights, rcond=None)[0]
        change = np.linalg.norm(refined - velocity)
        velocity = refined
        if change < 1e-8:
            break
    residual = doppler + matrix @ velocity
    mask = np.abs(residual) <= config.residual_threshold
    if mask.sum() < needed:
        return FitResult(False, 'insufficient_refined_consensus')
    final_condition = condition(matrix[mask])
    if final_condition > config.max_condition:
        return FitResult(False, 'unobservable_refined_consensus')
    if not np.isfinite(velocity).all() or np.linalg.norm(velocity) > config.max_speed:
        return FitResult(False, 'speed_limit')
    # Use only points supporting both fitting and final classification. Weights
    # <= 1 prevent a large radiometric value from inventing extra information.
    # Recompute the Huber weights at the returned velocity so the information
    # matrix and the residual variance describe the same weighted model:
    # Var(r_i) = sigma^2 / w_i, sigma^2 = sum(w r^2) / (n - 3).
    weights = huber_weights(a @ velocity - b, config.residual_threshold)
    support = mask[best]
    information = a[support].T @ (weights[support, None] * a[support])
    if condition(a[support] * np.sqrt(weights[support, None])) > config.max_condition:
        return FitResult(False, 'unobservable_weighted_geometry')
    supported = residual[best][support]
    sigma2 = max(config.noise_floor ** 2, float(
        np.sum(weights[support] * supported ** 2) / max(1, int(support.sum()) - 3)))
    covariance = sigma2 * np.linalg.inv(information)
    covariance += np.eye(3) * config.velocity_std_floor ** 2
    if (not np.isfinite(covariance).all()
            or np.max(np.linalg.eigvalsh(covariance)) > config.max_velocity_std ** 2):
        return FitResult(False, 'uncertainty_limit')
    return FitResult(True, 'valid', velocity, covariance, mask, residual, final_condition,
                     float(np.sqrt(np.mean(residual[mask] ** 2))))
