# SPDX-License-Identifier: Apache-2.0
"""Single-scan multipath-ghost rejection for Doppler movers."""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class GhostConfig:
    """
    Geometric rules measured on a UMRR-96 walk-through (see ACCUMULATION.md).

    A mover is a ghost when a nearer mover with a similar ego-compensated speed
    exists (a bounce path repeats the target's Doppler at a longer range), or
    when a static return at the same bearing is nearer (the mover is behind it).
    Both rules assume a mover is not directly behind another one: two people
    at the same speed and bearing, range_gap apart, lose the farther one.
    """

    range_gap: float = 1.5  # m a ghost must lie beyond the nearer return
    speed_tolerance: float = 0.25  # m/s, |compensated speed| match for a bounce copy
    wall_azimuth_deg: float = 4.0  # bearing match for the behind-static-return rule

    def __post_init__(self):
        values = (self.range_gap, self.speed_tolerance, self.wall_azimuth_deg)
        if not all(math.isfinite(x) and x > 0 for x in values):
            raise ValueError('Ghost gates must be finite and positive')


def ghost_mask(mover_xyz, mover_speed, static_xyz, config=GhostConfig()):
    """Return True for movers classified as multipath ghosts."""
    mover_xyz = np.asarray(mover_xyz, dtype=float).reshape(-1, 3)
    speed = np.abs(np.asarray(mover_speed, dtype=float).reshape(-1))
    static_xyz = np.asarray(static_xyz, dtype=float).reshape(-1, 3)
    if len(mover_xyz) != len(speed):
        raise ValueError('mover_xyz and mover_speed lengths differ')
    ranges = np.linalg.norm(mover_xyz, axis=1)
    farther = ranges[:, None] - ranges[None, :] > config.range_gap
    same_speed = np.abs(speed[:, None] - speed[None, :]) < config.speed_tolerance
    # A second-order bounce (radar-target-wall-target-radar) roughly doubles the speed.
    double_speed = np.abs(speed[:, None] - 2 * speed[None, :]) < 2 * config.speed_tolerance
    ghosts = np.any(farther & (same_speed | double_speed), axis=1)
    if len(static_xyz):
        azimuth = np.arctan2(mover_xyz[:, 1], mover_xyz[:, 0])
        static_azimuth = np.arctan2(static_xyz[:, 1], static_xyz[:, 0])
        bearing = np.abs(np.angle(np.exp(1j * (azimuth[:, None] - static_azimuth[None, :]))))
        behind = ranges[:, None] > np.linalg.norm(static_xyz, axis=1)[None, :] + config.range_gap
        ghosts |= np.any((bearing < math.radians(config.wall_azimuth_deg)) & behind, axis=1)
    return ghosts
