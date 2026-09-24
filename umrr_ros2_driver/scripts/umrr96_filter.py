# SPDX-License-Identifier: Apache-2.0
"""Optional post-detection gates for mapping and radial-motion views."""

from collections import deque
import math


class DetectionFilter:
    MODES = ('off', 'mapping', 'moving')

    def __init__(self, mode='off', min_snr_db=6.0, min_abs_speed=.25):
        if mode not in self.MODES:
            raise ValueError('filter_mode must be off, mapping, or moving')
        if not math.isfinite(min_snr_db) or not -20 <= min_snr_db <= 80:
            raise ValueError('filter_min_snr_db must be within -20..80 dB')
        if not math.isfinite(min_abs_speed) or not 0 <= min_abs_speed <= 30:
            raise ValueError('filter_min_abs_speed must be within 0..30 m/s')
        self.mode, self.min_snr_db, self.min_abs_speed = mode, min_snr_db, min_abs_speed
        self.history = deque(maxlen=2)
        self.last_stamp = None

    def select(self, points, stamp):
        """Select original point indices; never smooth or synthesize coordinates."""
        stats = dict(mode=self.mode, input=len(points), accepted=0,
                     rejected_quality=0, rejected_motion=0, rejected_temporal=0,
                     min_snr_db=self.min_snr_db, min_abs_speed=self.min_abs_speed)
        if self.mode == 'off':
            stats['accepted'] = len(points)
            return list(range(len(points))), stats
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.history.clear()
        self.last_stamp = stamp
        while self.history and stamp - self.history[0][0] > .5:
            self.history.popleft()
        candidates, accepted = [], []
        for index, point in enumerate(points):
            x, y, z, radius, angle, snr, speed = (float(point[name]) for name in (
                'x', 'y', 'z', 'range', 'azimuth_angle', 'snr', 'radial_speed'))
            if (not all(math.isfinite(v) for v in (x, y, z, radius, angle, snr, speed))
                    or radius < 0 or abs(angle) > math.pi or snr < self.min_snr_db):
                stats['rejected_quality'] += 1
                continue
            if self.mode == 'moving' and abs(speed) < self.min_abs_speed:
                stats['rejected_motion'] += 1
                continue
            candidates.append((radius, angle))
            # Two of three scans: one nearby candidate in either prior scan is sufficient.
            # Same-scan duplicates cannot confirm each other. This assumes a still radar.
            if self.mode == 'mapping' and not any(
                abs(radius - old_range) <= .5
                and abs(math.remainder(angle - old_angle, 2 * math.pi)) <= math.radians(3)
                for _, prior in self.history for old_range, old_angle in prior
            ):
                stats['rejected_temporal'] += 1
                continue
            accepted.append(index)
        self.history.append((stamp, candidates))
        stats['accepted'] = len(accepted)
        return accepted, stats
