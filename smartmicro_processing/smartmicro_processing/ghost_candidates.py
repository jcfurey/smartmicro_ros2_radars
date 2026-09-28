# SPDX-License-Identifier: Apache-2.0
"""
Experimental current-scan copy evidence; not wired into production rejection.

Matching signed speeds and directions narrows the hypothesis to aligned copies.
Two distinct nearby support points provide corroboration, not proof of a path:
independent aligned objects can still match. Other multipath geometries are
deliberately left unresolved. No static point alone causes rejection here.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CopyEvidenceConfig:
    """Existing geometric scales, frozen before evaluation on new scenes."""

    range_gap: float = 1.5
    speed_tolerance: float = 0.25
    direction_deg: float = 4.0
    support_radius: float = 0.6

    def __post_init__(self):
        scales = (self.range_gap, self.speed_tolerance, self.direction_deg,
                  self.support_radius)
        if not all(math.isfinite(x) and x > 0 for x in scales):
            raise ValueError('Copy evidence scales must be finite and positive')
        if self.direction_deg > 180:
            raise ValueError('Direction gate cannot exceed 180 degrees')


def copy_evidence(mover_xyz, mover_speed, config=CopyEvidenceConfig()):
    """
    Return signed-pair, aligned-pair and corroborated-pair ablation masks.

    A corroborated detection needs two distinct nearer XYZ positions, within
    support_radius of each other, each satisfying its signed-speed/direction
    gate. All three variants use one scan only. Duplicate coordinates cannot
    manufacture the second supporting return. Nonfinite and zero-range rows
    provide no evidence; input order and Doppler convention do not matter.
    """
    xyz = np.asarray(mover_xyz, dtype=float).reshape(-1, 3)
    speed = np.asarray(mover_speed, dtype=float).reshape(-1)
    if len(xyz) != len(speed):
        raise ValueError('mover_xyz and mover_speed lengths differ')
    ranges = np.linalg.norm(xyz, axis=1)
    valid = np.isfinite(xyz).all(axis=1) & np.isfinite(speed) & (ranges > 0)
    unit = np.zeros_like(xyz)
    np.divide(xyz, ranges[:, None], out=unit, where=valid[:, None])
    farther = ranges[:, None] - ranges[None, :] > config.range_gap
    same_sign = ((speed[:, None] > 0) & (speed[None, :] > 0)
                 | (speed[:, None] < 0) & (speed[None, :] < 0))
    matched = (valid[:, None] & valid[None, :] & farther & same_sign
               & (np.abs(speed[:, None] - speed[None, :]) < config.speed_tolerance))
    signed = matched.any(axis=1)
    cosine = np.clip(unit @ unit.T, -1., 1.)
    matched &= cosine > math.cos(math.radians(config.direction_deg))
    aligned = matched.any(axis=1)
    # Distances between the supporting points, not between support and ghost.
    squared = np.sum((xyz[:, None, :] - xyz[None, :, :]) ** 2, axis=2)
    nearby = (squared > 0) & (squared <= config.support_radius ** 2)
    supported = np.zeros(len(xyz), dtype=bool)
    for index in np.flatnonzero(matched.sum(axis=1) >= 2):
        candidates = np.flatnonzero(matched[index])
        supported[index] = nearby[np.ix_(candidates, candidates)].any()
    return {'signed_pair': signed, 'aligned_pair': aligned, 'supported_copy': supported}
