# SPDX-License-Identifier: Apache-2.0
"""Optional post-detection gates for mapping and radial-motion views."""

from collections import deque
import math

import numpy as np

FIELDS = ('x', 'y', 'z', 'range', 'azimuth_angle', 'snr', 'radial_speed')
RANGE_GATE = .5  # m, |range - prior range|
ANGLE_GATE = math.radians(3)  # rad, circular azimuth difference
HISTORY_SECONDS = .5
# Hash cells are slightly wider than the gates, so any pair inside both gates
# lies in the same or an adjacent cell despite rounding.
RANGE_CELL = RANGE_GATE * 1.001
ANGLE_CELL = ANGLE_GATE * 1.001
ANGLE_CELLS = int(2 * math.pi // ANGLE_CELL)
MAX_RANGE_CELL = int(1e9 // RANGE_CELL)  # Monotone clamp keeps adjacency; bounds int64 ids.
BRUTE_FORCE_PAIRS = 65536


def columns(points):
    """Read the gate fields as float64 arrays from a structured array or mappings."""
    if isinstance(points, np.ndarray) and points.dtype.names:
        return [np.asarray(points[name], dtype=float).reshape(-1) for name in FIELDS]
    return [np.array([float(point[name]) for point in points], dtype=float) for name in FIELDS]


def gate(query_range, query_angle, prior_range, prior_angle):
    """
    Elementwise ``|dr| <= 0.5 and |remainder(da, 2*pi)| <= 3 deg``.

    Both angles are within [-pi, pi], so the IEEE remainder of their difference
    is the difference itself or, beyond pi, the difference minus 2*pi with the
    same sign: exactly what ``math.remainder`` returns, bit for bit.
    """
    difference = query_angle - prior_angle
    wrapped = np.where(np.abs(difference) > math.pi,
                       difference - np.copysign(2 * math.pi, difference), difference)
    return (np.abs(query_range - prior_range) <= RANGE_GATE) & (np.abs(wrapped) <= ANGLE_GATE)


def _cells(ranges, angles):
    range_cells = np.minimum(np.floor(ranges / RANGE_CELL), MAX_RANGE_CELL).astype(np.int64)
    angle_cells = np.minimum(np.floor((angles + math.pi) / ANGLE_CELL),
                             ANGLE_CELLS - 1).astype(np.int64)
    return range_cells, angle_cells


def confirmed(query_range, query_angle, prior_range, prior_angle):
    """
    Return, per query, whether any prior detection lies within both gates.

    Small inputs compare every pair; larger inputs look only in the 3x3
    neighbouring (range, azimuth) hash cells, with azimuth wrapping at +/-pi.
    """
    matched = np.zeros(len(query_range), dtype=bool)
    if not len(query_range) or not len(prior_range):
        return matched
    if len(query_range) * len(prior_range) <= BRUTE_FORCE_PAIRS:
        return gate(query_range[:, None], query_angle[:, None],
                    prior_range[None, :], prior_angle[None, :]).any(axis=1)
    prior_rc, prior_ac = _cells(prior_range, prior_angle)
    prior_ids = prior_rc * ANGLE_CELLS + prior_ac
    order = np.argsort(prior_ids, kind='stable')
    sorted_ids = prior_ids[order]
    query_rc, query_ac = _cells(query_range, query_angle)
    queries = np.arange(len(query_range))
    for range_offset in (-1, 0, 1):
        for angle_offset in (-1, 0, 1):
            cell_range = query_rc + range_offset
            ids = cell_range * ANGLE_CELLS + (query_ac + angle_offset) % ANGLE_CELLS
            low = np.searchsorted(sorted_ids, ids, 'left')
            counts = np.where((cell_range >= 0) & ~matched,
                              np.searchsorted(sorted_ids, ids, 'right') - low, 0)
            total = int(counts.sum())
            if not total:
                continue
            query_index = np.repeat(queries, counts)
            starts = np.repeat(low - (np.cumsum(counts) - counts), counts)
            prior_index = order[starts + np.arange(total)]
            hits = gate(query_range[query_index], query_angle[query_index],
                        prior_range[prior_index], prior_angle[prior_index])
            matched[query_index[hits]] = True
    return matched


class DetectionFilter:
    """Select original target indices; never smooth or synthesize coordinates."""

    MODES = ('off', 'quality', 'mapping', 'moving')

    def __init__(self, mode='off', min_snr_db=6.0, min_abs_speed=.25):
        if mode not in self.MODES:
            raise ValueError('filter_mode must be off, quality, mapping, or moving')
        if not math.isfinite(min_snr_db) or not -20 <= min_snr_db <= 80:
            raise ValueError('filter_min_snr_db must be within -20..80 dB')
        if not math.isfinite(min_abs_speed) or not 0 <= min_abs_speed <= 30:
            raise ValueError('filter_min_abs_speed must be within 0..30 m/s')
        self.mode, self.min_snr_db, self.min_abs_speed = mode, min_snr_db, min_abs_speed
        # (stamp, (candidate ranges, candidate azimuths)) of the previous two scans.
        self.history = deque(maxlen=2)
        self.last_stamp = None

    def select(self, points, stamp):
        """Return (selected original point indices, statistics)."""
        stats = {'mode': self.mode, 'input': len(points), 'accepted': 0,
                 'rejected_quality': 0, 'rejected_motion': 0, 'rejected_temporal': 0,
                 'min_snr_db': self.min_snr_db, 'min_abs_speed': self.min_abs_speed}
        if self.mode == 'off':
            stats['accepted'] = len(points)
            return list(range(len(points))), stats
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.history.clear()
        self.last_stamp = stamp
        while self.history and stamp - self.history[0][0] > HISTORY_SECONDS:
            self.history.popleft()
        if len(points):
            x, y, z, radius, angle, snr, speed = columns(points)
            with np.errstate(invalid='ignore'):
                quality = (np.isfinite(np.column_stack((x, y, z, radius, angle, snr, speed)))
                           .all(axis=1) & (radius >= 0) & (np.abs(angle) <= math.pi)
                           & (snr >= self.min_snr_db))
                moving = np.abs(speed) >= self.min_abs_speed
        else:
            radius = angle = np.empty(0)
            quality = moving = np.zeros(0, dtype=bool)
        candidate = quality & moving if self.mode == 'moving' else quality
        stats['rejected_quality'] = int((~quality).sum())
        stats['rejected_motion'] = int((quality & ~candidate).sum())
        accepted = candidate.copy()
        # Two of three scans: one nearby candidate in either prior scan is sufficient.
        # Same-scan duplicates cannot confirm each other. This assumes a still radar.
        if self.mode == 'mapping':
            prior_range = np.concatenate([ranges for _, (ranges, _) in self.history] + [[]])
            prior_angle = np.concatenate([angles for _, (_, angles) in self.history] + [[]])
            accepted[candidate] = confirmed(radius[candidate], angle[candidate],
                                            prior_range, prior_angle)
            stats['rejected_temporal'] = int((candidate & ~accepted).sum())
        self.history.append((stamp, (radius[candidate], angle[candidate])))
        selected = np.flatnonzero(accepted).tolist()
        stats['accepted'] = len(selected)
        return selected, stats
