# SPDX-License-Identifier: Apache-2.0
import numpy as np
import pytest

from smartmicro_processing.doppler import fit_velocity, FitConfig


def scene():
    rng = np.random.default_rng(12)
    az = rng.uniform(-1.2, 1.2, 90)
    el = rng.uniform(-.35, .35, 90)
    xyz = np.column_stack((np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)))
    truth = np.array([1.2, -.4, .25])
    return xyz, -(xyz @ truth), truth


def test_recovers_3d_translation_with_noise_and_dynamic_outliers():
    xyz, speed, truth = scene()
    speed += np.random.default_rng(3).normal(0, .015, len(speed))
    speed[::4] += 3
    result = fit_velocity(xyz * 8, speed)
    assert result.valid, result.reason
    np.testing.assert_allclose(result.velocity, truth, atol=.04)
    assert not result.inliers[::4].any()
    assert result.inliers.sum() == len(speed) - len(speed[::4])
    assert np.linalg.eigvalsh(result.covariance).min() >= .10**2
    np.testing.assert_allclose(result.covariance, result.covariance.T, atol=1e-12)


def test_sign_adapter_and_permutation():
    xyz, speed, truth = scene()
    result = fit_velocity(xyz[::-1], -speed[::-1], FitConfig(doppler_sign=-1))
    assert result.valid
    np.testing.assert_allclose(result.velocity, truth, atol=1e-10)


def test_zero_doppler_keeps_landmarks_and_positive_uncertainty():
    xyz, speed, _ = scene()
    result = fit_velocity(xyz, speed * 0)
    assert result.valid and result.inliers.all()
    np.testing.assert_allclose(result.velocity, 0, atol=1e-12)
    assert np.linalg.eigvalsh(result.covariance).min() > .01


def test_unobservable_vertical_component_rejects_instead_of_claiming_zero():
    az = np.linspace(-1, 1, 30)
    xyz = np.column_stack((np.cos(az), np.sin(az), np.zeros(30)))
    assert fit_velocity(xyz, -xyz[:, 0]).reason == 'unobservable_geometry'


def test_bearing_diversity_only_in_outliers_is_rejected():
    az = np.linspace(-1, 1, 80)
    xyz = np.column_stack((np.cos(az), np.sin(az), np.zeros(80)))
    extra, _, _ = scene()
    xyz = np.concatenate([xyz, extra[:10]])
    speed = -xyz[:, 0]
    speed[-10:] += np.arange(1, 11) * 3
    result = fit_velocity(xyz, speed)
    assert not result.valid


def test_no_majority_consensus():
    xyz, speed, _ = scene()
    speed += np.random.default_rng(4).uniform(-8, 8, len(speed))
    result = fit_velocity(xyz, speed)
    assert not result.valid
    assert result.reason == 'insufficient_consensus'


@pytest.mark.parametrize('failure', ['sparse', 'nan', 'zero_range', 'shape', 'speed',
                                     'covariance'])
def test_invalid_measurements_and_implausible_fit(failure):
    xyz, speed, _ = scene()
    config = FitConfig()
    if failure == 'sparse':
        xyz, speed = xyz[:3], speed[:3]
    elif failure == 'nan':
        speed[0] = np.nan
    elif failure == 'zero_range':
        xyz[0] = 0
    elif failure == 'shape':
        speed = speed[:-1]
    elif failure == 'speed':
        speed *= 30
    elif failure == 'covariance':
        config = FitConfig(max_velocity_std=.100001)
    assert not fit_velocity(xyz, speed, config).valid


@pytest.mark.parametrize('options', [{'doppler_sign': 0}, {'min_inlier_fraction': .5},
                                     {'noise_floor': float('nan')}, {'min_inliers': 3},
                                     {'ransac_trials': 0}, {'max_condition': 1}])
def test_invalid_config(options):
    with pytest.raises(ValueError):
        FitConfig(**options)


def test_covariance_uses_weighted_residual_variance_at_returned_velocity():
    xyz, speed, _ = scene()
    # Residuals beyond the 0.1 m/s Huber knee get weights < 1 but stay inliers.
    speed += np.random.default_rng(7).uniform(-.12, .12, len(speed))
    config = FitConfig()
    result = fit_velocity(xyz, speed, config)
    assert result.valid and result.inliers.all()
    residual = speed + xyz @ result.velocity
    weights = np.clip(config.residual_threshold * .5 / np.abs(residual), .05, 1)
    assert weights.min() < 1  # The weighted and unweighted variances differ here.
    sigma2 = np.sum(weights * residual ** 2) / (len(residual) - 3)
    assert sigma2 > config.noise_floor ** 2
    information = xyz.T @ (weights[:, None] * xyz)
    expected = sigma2 * np.linalg.inv(information) + np.eye(3) * config.velocity_std_floor ** 2
    np.testing.assert_allclose(result.covariance, expected, rtol=1e-9, atol=1e-15)
    unweighted = np.sum(residual ** 2) / (len(residual) - 3)
    assert sigma2 < unweighted
