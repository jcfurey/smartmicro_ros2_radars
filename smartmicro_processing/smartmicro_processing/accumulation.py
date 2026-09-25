# SPDX-License-Identifier: Apache-2.0
"""Bounded temporal radar evidence, with at most one vote per voxel per scan."""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class AccumulationConfig:
    window_seconds: float = .5
    voxel_size: float = .25
    min_support_scans: int = 2
    max_scans: int = 32
    max_observations: int = 8192

    def __post_init__(self):
        if not math.isfinite(self.window_seconds) or not .1 <= self.window_seconds <= 5:
            raise ValueError('window_seconds must be within 0.1..5 seconds')
        if not math.isfinite(self.voxel_size) or not .05 <= self.voxel_size <= 5:
            raise ValueError('voxel_size must be within 0.05..5 metres')
        for name, value, upper in (('max_scans', self.max_scans, 256),
                                   ('max_observations', self.max_observations, 131072)):
            if type(value) is not int or not 1 <= value <= upper:
                raise ValueError(f'{name} must be an integer within 1..{upper}')
        if (type(self.min_support_scans) is not int
                or not 1 <= self.min_support_scans <= self.max_scans):
            raise ValueError('min_support_scans must be within 1..max_scans')


def transform_measurements(values, translation, quaternion):
    """Apply target-from-sensor translation and XYZW rotation to XYZ only.

    Other columns retain their sensor-measurement meaning. The returned points
    are transformed detections, never interpolated surfaces or voxel centres.
    """
    values = np.asarray(values, dtype=float)
    translation = np.asarray(translation, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if (values.ndim != 2 or values.shape[1] != 5 or translation.shape != (3,)
            or quaternion.shape != (4,) or not np.isfinite(values).all()
            or not np.isfinite(translation).all() or not np.isfinite(quaternion).all()):
        raise ValueError('Invalid transform or measurements')
    norm = np.linalg.norm(quaternion)
    if abs(norm - 1) > 1e-3:
        raise ValueError('Transform rotation must be a unit quaternion')
    x, y, z, w = quaternion / norm
    rotation = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    result = values.copy()
    result[:, :3] = values[:, :3] @ rotation.T + translation
    if (not np.isfinite(result).all()
            or np.any(np.abs(result[:, :3]) > np.finfo(np.float32).max)):
        raise ValueError('Transform coordinates cannot be represented in the output cloud')
    return result


def pose_discontinuity(previous, current, translation_limit, rotation_limit):
    """Heuristic step check; TF has no pose covariance or reset identifier."""
    old_t, old_q = previous
    new_t, new_q = current
    distance = np.linalg.norm(np.asarray(new_t) - np.asarray(old_t))
    cosine = abs(float(np.dot(old_q, new_q))) / (np.linalg.norm(old_q) * np.linalg.norm(new_q))
    angle = 2 * math.acos(float(np.clip(cosine, 0, 1)))
    return distance > translation_limit or angle > rotation_limit


class TemporalEvidence:
    def __init__(self, config=AccumulationConfig()):
        self.config = config
        self.frames = deque()
        self.observations = 0
        self.last_stamp = None
        self.now = None
        self.capacity_drops = 0

    def clear(self):
        self.frames.clear()
        self.observations = 0
        self.last_stamp = self.now = None

    def _remove_oldest(self, capacity=False):
        _, values = self.frames.popleft()
        self.observations -= len(values)
        if capacity:
            self.capacity_drops += len(values)

    def expire(self, now_ns):
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError('Clock must be nonnegative integer nanoseconds')
        if self.now is not None and now_ns < self.now:
            self.clear()
        self.now = now_ns
        cutoff = now_ns - round(self.config.window_seconds * 1e9)
        while self.frames and self.frames[0][0] <= cutoff:
            self._remove_oldest()

    def add(self, values, indices, stamp_ns, now_ns):
        """Accept a chronological scan already transformed at its own timestamp."""
        self.expire(now_ns)
        if type(stamp_ns) is not int or stamp_ns <= 0:
            raise ValueError('Scan timestamp must be positive integer nanoseconds')
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise ValueError('Scan timestamps must increase strictly')
        if stamp_ns <= now_ns - round(self.config.window_seconds * 1e9):
            return False
        values, indices = np.asarray(values, dtype=float), np.asarray(indices)
        if (values.ndim != 2 or values.shape[1] != 5 or indices.shape != (len(values),)
                or len(values) > 4096 or not np.isfinite(values).all()
                or np.any(np.abs(values[:, :3]) > np.finfo(np.float32).max)
                or not np.issubdtype(indices.dtype, np.integer) or np.any(indices < 0)):
            raise ValueError('Invalid evidence observations')
        cells = {}
        for value, index in zip(values, indices):
            key = tuple(math.floor(float(x) / self.config.voxel_size) for x in value[:3])
            # Deterministic representative: first measured point in this scan's
            # cell. Stronger SNR/RCS does not get extra votes or weight.
            if key not in cells:
                cells[key] = (value.copy(), int(index))
        if len(cells) > self.config.max_observations:
            self.capacity_drops += len(cells) - self.config.max_observations
            cells = dict(list(cells.items())[:self.config.max_observations])
        self.last_stamp = stamp_ns
        if cells:
            self.frames.append((stamp_ns, cells))
            self.observations += len(cells)
        while (len(self.frames) > self.config.max_scans
               or self.observations > self.config.max_observations):
            self._remove_oldest(capacity=True)
        return True

    def snapshot(self, now_ns):
        """Return one newest measured representative per cell plus support/age."""
        self.expire(now_ns)
        cells = {}
        for stamp, observations in self.frames:
            for key, (value, index) in observations.items():
                previous = cells.get(key)
                support = 1 if previous is None else previous['support_scans'] + 1
                first = stamp if previous is None else previous['first_stamp_ns']
                cells[key] = dict(value=value, source_index=index, source_stamp_ns=stamp,
                                  first_stamp_ns=first, support_scans=support,
                                  age_seconds=max(0, now_ns - stamp) * 1e-9,
                                  span_seconds=(stamp - first) * 1e-9)
        return [cells[key] for key in sorted(cells)]
