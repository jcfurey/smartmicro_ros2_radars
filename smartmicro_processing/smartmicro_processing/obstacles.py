# SPDX-License-Identifier: Apache-2.0
"""Obstacle evidence for a Nav2 ObstacleLayer (marking only) from classified radar returns."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


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

    def __post_init__(self):
        if not 1 <= self.persistence_hits <= self.persistence_window <= 64:
            raise ValueError('Need 1 <= persistence_hits <= persistence_window <= 64')
        values = (self.range_bin, self.azimuth_bin_deg, self.safety_range, self.track_radius)
        if not all(math.isfinite(v) and v > 0 for v in values) or not math.isfinite(
                self.obstacle_height):
            raise ValueError('Obstacle scales must be finite and positive')


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


def obstacle_points(static_xyz, persistent, mover_xyz, mover_ghost, track_xy, config,
                    novel=None):
    """
    Select obstacle evidence and return an (N, 3) array.

    Static returns pass when persistent (or inside safety_range). Moving
    returns pass when not ghosts and either near a confirmed track or inside
    safety_range. Ghost-classified returns never pass. With ``novel`` (a mask
    of static returns outside the learned background), novel static returns
    more than ``shadow_gap`` beyond the nearest confirmed track are treated as
    that object's multipath and dropped while the track exists.
    """
    static_xyz = np.asarray(static_xyz, float).reshape(-1, 3)
    mover_xyz = np.asarray(mover_xyz, float).reshape(-1, 3)
    track_xy = np.asarray(track_xy, float).reshape(-1, 2)
    near_static = np.linalg.norm(static_xyz[:, :2], axis=1) < config.safety_range
    keep = np.asarray(persistent, bool) | near_static
    if novel is not None and config.shadow_gap > 0 and len(track_xy) and len(static_xyz):
        nearest_track = np.min(np.linalg.norm(track_xy, axis=1))
        ranges = np.linalg.norm(static_xyz[:, :2], axis=1)
        keep &= ~(np.asarray(novel, bool) & (ranges > nearest_track + config.shadow_gap))
    keep_static = static_xyz[keep]
    ok = ~np.asarray(mover_ghost, bool)
    near_mover = np.linalg.norm(mover_xyz[:, :2], axis=1) < config.safety_range
    if len(track_xy) and len(mover_xyz):
        on_track = np.min(np.linalg.norm(
            mover_xyz[:, None, :2] - track_xy[None], axis=2), axis=1) < config.track_radius
    else:
        on_track = np.zeros(len(mover_xyz), bool)
    parts = [keep_static, mover_xyz[ok & (on_track | near_mover)]]
    if config.include_tracks and len(track_xy):
        parts.append(np.c_[track_xy, np.zeros(len(track_xy))])
    points = np.vstack(parts)
    if config.obstacle_height >= 0 and len(points):
        points[:, 2] = config.obstacle_height
    return points
