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


def pose_step(previous, current):
    """Return translation distance (m) and rotation angle (rad) between two poses."""
    old_t, old_q = previous
    new_t, new_q = current
    distance = float(np.linalg.norm(np.asarray(new_t) - np.asarray(old_t)))
    cosine = abs(float(np.dot(old_q, new_q))) / (np.linalg.norm(old_q) * np.linalg.norm(new_q))
    return distance, 2 * math.acos(float(np.clip(cosine, 0, 1)))


def pose_step_limits(translation_step, rotation_step, dt_seconds=0.0,
                     max_speed=0.0, max_angular_speed=0.0):
    """Discontinuity thresholds for scans ``dt_seconds`` apart.

    A fixed per-scan step would reject ordinary motion above ``step * rate``
    and after dropped scans, so the allowance grows with elapsed time.
    """
    dt = max(0.0, float(dt_seconds))
    return (translation_step + max_speed * dt,
            min(math.pi, rotation_step + max_angular_speed * dt))


def pose_discontinuity(previous, current, translation_limit, rotation_limit):
    """Heuristic step check; TF has no pose covariance or reset identifier."""
    distance, angle = pose_step(previous, current)
    return distance > translation_limit or angle > rotation_limit


class EvidenceFrame:
    """One scan's voxel representatives in first-measured order."""

    __slots__ = ('keys', 'values', 'indices')

    def __init__(self, keys, values, indices):
        self.keys, self.values, self.indices = keys, values, indices

    def __len__(self):
        return len(self.indices)


SNAPSHOT_DTYPE = np.dtype([
    ('value', '<f8', (5,)), ('source_index', '<i8'), ('source_stamp_ns', '<i8'),
    ('first_stamp_ns', '<i8'), ('support_scans', '<i8'), ('age_seconds', '<f8'),
    ('span_seconds', '<f8'),
])


def voxel_keys(xyz, voxel_size):
    """Exact integer-valued voxel keys; +0.0 folds -0.0 into the same cell as 0.0."""
    return np.floor(np.asarray(xyz, dtype=float) / voxel_size) + 0.0


def _group_starts(sorted_keys):
    starts = np.ones(len(sorted_keys), dtype=bool)
    if len(sorted_keys) > 1:
        starts[1:] = np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)
    return starts


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
        _, frame = self.frames.popleft()
        self.observations -= len(frame)
        if capacity:
            self.capacity_drops += len(frame)

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
        keys = voxel_keys(values[:, :3], self.config.voxel_size)
        # Deterministic representative: first measured point in this scan's
        # cell. Stronger SNR/RCS does not get extra votes or weight. lexsort
        # is stable, so each group's first element is its first measurement.
        order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
        first = np.sort(order[_group_starts(keys[order])])
        if len(first) > self.config.max_observations:
            self.capacity_drops += len(first) - self.config.max_observations
            first = first[:self.config.max_observations]
        self.last_stamp = stamp_ns
        if len(first):
            self.frames.append((stamp_ns, EvidenceFrame(
                keys[first], values[first].copy(), indices[first].astype(np.int64))))
            self.observations += len(first)
        while (len(self.frames) > self.config.max_scans
               or self.observations > self.config.max_observations):
            self._remove_oldest(capacity=True)
        return True

    def snapshot_array(self, now_ns):
        """Return one newest representative per cell, sorted by cell, as a record array."""
        self.expire(now_ns)
        if not self.frames:
            return np.empty(0, dtype=SNAPSHOT_DTYPE)
        frames = [frame for _, frame in self.frames]
        keys = np.concatenate([frame.keys for frame in frames])
        stamps = np.concatenate([np.full(len(frame), stamp, dtype=np.int64)
                                 for stamp, frame in self.frames])
        order = np.lexsort((stamps, keys[:, 2], keys[:, 1], keys[:, 0]))
        starts = np.flatnonzero(_group_starts(keys[order]))
        ends = np.append(starts[1:], len(order)) - 1
        newest, oldest = order[ends], order[starts]
        result = np.empty(len(starts), dtype=SNAPSHOT_DTYPE)
        result['value'] = np.concatenate([frame.values for frame in frames])[newest]
        result['source_index'] = np.concatenate([frame.indices for frame in frames])[newest]
        result['source_stamp_ns'] = stamps[newest]
        result['first_stamp_ns'] = stamps[oldest]
        result['support_scans'] = ends - starts + 1
        result['age_seconds'] = np.maximum(0, now_ns - stamps[newest]) * 1e-9
        result['span_seconds'] = (stamps[newest] - stamps[oldest]) * 1e-9
        return result

    def snapshot(self, now_ns):
        """List-of-dicts form of :meth:`snapshot_array` for analysis scripts and tests."""
        return [dict(value=record['value'].copy(), source_index=int(record['source_index']),
                     source_stamp_ns=int(record['source_stamp_ns']),
                     first_stamp_ns=int(record['first_stamp_ns']),
                     support_scans=int(record['support_scans']),
                     age_seconds=float(record['age_seconds']),
                     span_seconds=float(record['span_seconds']))
                for record in self.snapshot_array(now_ns)]
