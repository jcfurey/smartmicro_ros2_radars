# SPDX-License-Identifier: Apache-2.0
"""Obstacle evidence for a Nav2 ObstacleLayer (marking only) from classified radar returns."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .accumulation import transform_measurements


@dataclass
class ObstacleConfig:
    persistence_hits: int = 3  # a static return is kept when its cell was hit in >= k ...
    persistence_window: int = 5  # ... of the last n scans (the current scan included)
    range_bin: float = 0.5  # m, polar persistence cell
    azimuth_bin_deg: float = 3.0
    safety_range: float = 1.0  # m, non-ghost returns nearer than this pass immediately
    track_radius: float = 0.8  # m, moving returns this close to a confirmed track pass
    obstacle_height: float = 0.3  # m, output z; negative keeps the measured z
    neighbourhood: int = 0  # cells counted around each cell: 0 = own cell, 1 = 3x3
    include_tracks: bool = True  # add each confirmed track position as a point
    shadow_gap: float = 1.5  # m; <= 0 disables the track-shadow rule
    shadow_half_angle_deg: float = 15.0  # track-shadow bearing half-width; 180 = any bearing

    def __post_init__(self):
        if not 1 <= self.persistence_hits <= self.persistence_window <= 64:
            raise ValueError('Need 1 <= persistence_hits <= persistence_window <= 64')
        values = (self.range_bin, self.azimuth_bin_deg, self.safety_range, self.track_radius)
        if not all(math.isfinite(v) and v > 0 for v in values) or not math.isfinite(
                self.obstacle_height):
            raise ValueError('Obstacle scales must be finite and positive')
        if not (math.isfinite(self.shadow_half_angle_deg)
                and 0 < self.shadow_half_angle_deg <= 180):
            raise ValueError('shadow_half_angle_deg must be within (0, 180]')


class PersistenceFilter:
    """
    k-of-n polar-cell persistence; a cell counts its 3x3 neighbourhood.

    Cells are in the input frame: on a moving platform the window must stay
    short enough that motion during it is below one cell.
    """

    def __init__(self, config):
        self.config = config
        self.history = deque(maxlen=config.persistence_window)

    def keys(self, xyz):
        c = self.config
        r = np.linalg.norm(xyz[:, :2], axis=1)
        az = np.arctan2(xyz[:, 1], xyz[:, 0])
        return np.stack([np.floor(r / c.range_bin),
                         np.floor(az / math.radians(c.azimuth_bin_deg))], 1).astype(int)

    def step(self, xyz):
        """Record this scan's cells and return a mask of persistent returns."""
        xyz = np.asarray(xyz, float).reshape(-1, 3)
        keys = self.keys(xyz) if len(xyz) else np.empty((0, 2), int)
        n = range(-self.config.neighbourhood, self.config.neighbourhood + 1)
        dilated = {(a + i, b + j) for a, b in map(tuple, keys) for i in n for j in n}
        self.history.append(dilated)
        counts = [sum((a, b) in scan for scan in self.history) for a, b in keys]
        return np.array(counts, int) >= self.config.persistence_hits

    def reset(self):
        self.history.clear()


def near_tracks(xyz, track_xy, radius):
    """Mask of points within ``radius`` (planar) of any confirmed track position."""
    xyz = np.asarray(xyz, float).reshape(-1, 3)
    track_xy = np.asarray(track_xy, float).reshape(-1, 2)
    if not len(xyz) or not len(track_xy):
        return np.zeros(len(xyz), bool)
    return np.min(np.linalg.norm(xyz[:, None, :2] - track_xy[None], axis=2), axis=1) < radius


def obstacle_points(static_xyz, persistent, mover_xyz, mover_ghost, track_xy, config,
                    novel=None, flatten=True, keep_movers=False):
    """
    Select obstacle evidence and return an (N, 3) array.

    Static returns pass when persistent (or inside safety_range). Moving
    returns pass when not ghosts and either near a confirmed track or inside
    safety_range. Ghost-classified returns never pass. With ``novel`` (a mask
    of static returns outside the learned background), a novel static return
    within ``shadow_half_angle_deg`` of a confirmed track's bearing and more
    than ``shadow_gap`` beyond that track's range is treated as the object's
    multipath and dropped while the track exists. A half-angle of 180 degrees
    reproduces the earlier rule: beyond the nearest track at any bearing.
    Track positions get z = 0 (the sensor's height). ``flatten=False`` keeps
    the measured z for a later transform (see ``to_frame``). ``keep_movers``
    passes every non-ghost mover, for scans whose track positions are unknown.
    """
    static_xyz = np.asarray(static_xyz, float).reshape(-1, 3)
    mover_xyz = np.asarray(mover_xyz, float).reshape(-1, 3)
    track_xy = np.asarray(track_xy, float).reshape(-1, 2)
    near_static = np.linalg.norm(static_xyz[:, :2], axis=1) < config.safety_range
    keep = np.asarray(persistent, bool) | near_static
    if novel is not None and config.shadow_gap > 0 and len(track_xy) and len(static_xyz):
        ranges = np.linalg.norm(static_xyz[:, :2], axis=1)
        offset = (np.arctan2(static_xyz[:, 1], static_xyz[:, 0])[:, None]
                  - np.arctan2(track_xy[:, 1], track_xy[:, 0])[None])
        bearing = np.abs(np.angle(np.exp(1j * offset)))
        behind = ranges[:, None] > np.linalg.norm(track_xy, axis=1)[None] + config.shadow_gap
        shadowed = np.any(behind & (bearing <= math.radians(config.shadow_half_angle_deg)), 1)
        keep &= ~(np.asarray(novel, bool) & shadowed)
    keep_static = static_xyz[keep]
    ok = ~np.asarray(mover_ghost, bool)
    near_mover = np.linalg.norm(mover_xyz[:, :2], axis=1) < config.safety_range
    on_track = (np.ones(len(mover_xyz), bool) if keep_movers
                else near_tracks(mover_xyz, track_xy, config.track_radius))
    parts = [keep_static, mover_xyz[ok & (on_track | near_mover)]]
    if config.include_tracks and len(track_xy):
        parts.append(np.c_[track_xy, np.zeros(len(track_xy))])
    points = np.vstack(parts)
    if flatten and config.obstacle_height >= 0 and len(points):
        points[:, 2] = config.obstacle_height
    return points


def to_frame(points, translation, quaternion, height):
    """
    Transform sensor-frame points by a target-from-sensor pose (XYZW rotation).

    Afterwards z is set to ``height`` in the target frame; a negative height
    keeps the transformed z. Raises ValueError for an invalid transform.
    """
    points = np.asarray(points, float).reshape(-1, 3)
    result = transform_measurements(np.c_[points, np.zeros((len(points), 2))],
                                    translation, quaternion)[:, :3]
    if height >= 0:
        result[:, 2] = height
    return result
