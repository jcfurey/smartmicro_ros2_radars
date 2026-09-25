# SPDX-License-Identifier: Apache-2.0
"""Byte equivalence of the vectorized cloud/evidence code with the original loops (P4/P5)."""
from collections import deque
from copy import deepcopy
import math

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from smartmicro_processing.accumulation import AccumulationConfig, TemporalEvidence
from smartmicro_processing.cloud import subset_cloud
from smartmicro_processing.evidence_cloud import DTYPE, evidence_cloud
from std_msgs.msg import Header


def reference_subset_cloud(cloud, indices):
    """Original point-by-point implementation (before P4)."""
    indices = np.asarray(indices, dtype=np.int64)
    data = bytes(cloud.data)
    selected = bytearray()
    for index in indices:
        start = int(index // cloud.width * cloud.row_step + index % cloud.width * cloud.point_step)
        selected.extend(data[start:start + cloud.point_step])
    return PointCloud2(header=deepcopy(cloud.header), height=1, width=len(indices),
                       fields=deepcopy(cloud.fields), is_bigendian=cloud.is_bigendian,
                       point_step=cloud.point_step, row_step=len(indices) * cloud.point_step,
                       data=bytes(selected), is_dense=False)


class ReferenceEvidence:
    """Original dict-per-cell implementation (before P5), minus input validation."""

    def __init__(self, config):
        self.config = config
        self.frames = deque()
        self.observations = 0
        self.last_stamp = self.now = None
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
        if self.now is not None and now_ns < self.now:
            self.clear()
        self.now = now_ns
        cutoff = now_ns - round(self.config.window_seconds * 1e9)
        while self.frames and self.frames[0][0] <= cutoff:
            self._remove_oldest()

    def add(self, values, indices, stamp_ns, now_ns):
        self.expire(now_ns)
        if self.last_stamp is not None and stamp_ns <= self.last_stamp:
            raise ValueError('Scan timestamps must increase strictly')
        if stamp_ns <= now_ns - round(self.config.window_seconds * 1e9):
            return False
        cells = {}
        for value, index in zip(values, indices):
            key = tuple(math.floor(float(x) / self.config.voxel_size) for x in value[:3])
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
        self.expire(now_ns)
        cells = {}
        for stamp, observations in self.frames:
            for key, (value, index) in observations.items():
                previous = cells.get(key)
                support = 1 if previous is None else previous['support_scans'] + 1
                first = stamp if previous is None else previous['first_stamp_ns']
                cells[key] = {'value': value, 'source_index': index, 'source_stamp_ns': stamp,
                              'first_stamp_ns': first, 'support_scans': support,
                              'age_seconds': max(0, now_ns - stamp) * 1e-9,
                              'span_seconds': (stamp - first) * 1e-9}
        return [cells[key] for key in sorted(cells)]


def reference_evidence_cloud(samples, header):
    data = np.empty(len(samples), dtype=DTYPE)
    for index, sample in enumerate(samples):
        stamp = sample['source_stamp_ns']
        data[index] = (*sample['value'], sample['age_seconds'], sample['span_seconds'],
                       sample['support_scans'], sample['source_index'],
                       stamp // 1_000_000_000, stamp % 1_000_000_000)
    return data.tobytes()


def random_cloud(rng, width, height, point_step, row_padding, big_endian):
    fields = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'radial_speed', 'snr'))]
    row_step = width * point_step + row_padding
    data = rng.integers(0, 256, height * row_step, dtype=np.uint8).tobytes()
    return PointCloud2(header=Header(frame_id='umrr96'), height=height, width=width,
                       fields=fields, is_bigendian=big_endian, point_step=point_step,
                       row_step=row_step, data=data, is_dense=False)


@pytest.mark.parametrize('seed', range(12))
def test_subset_cloud_matches_point_loop(seed):
    rng = np.random.default_rng(seed)
    width, height = int(rng.integers(1, 64)), int(rng.integers(1, 5))
    cloud = random_cloud(rng, width, height, int(rng.integers(20, 48)),
                         int(rng.integers(0, 9)), bool(seed % 2))
    count = width * height
    for indices in (rng.integers(0, count, int(rng.integers(0, 3 * count))),
                    np.arange(count), np.arange(count)[::-1], []):
        expected = reference_subset_cloud(cloud, indices)
        actual = subset_cloud(cloud, indices)
        assert bytes(actual.data) == bytes(expected.data)
        assert (actual.width, actual.height, actual.row_step, actual.point_step) == (
            expected.width, expected.height, expected.row_step, expected.point_step)
        assert actual.fields == expected.fields and actual.header == expected.header
        assert actual.is_bigendian == expected.is_bigendian and not actual.is_dense


def test_subset_cloud_accepts_list_data_and_rejects_invalid_indices():
    cloud = random_cloud(np.random.default_rng(1), 4, 1, 20, 0, False)
    listed = deepcopy(cloud)
    listed.data = list(bytes(cloud.data))
    expected = bytes(reference_subset_cloud(cloud, [2, 0]).data)
    assert bytes(subset_cloud(listed, [2, 0]).data) == expected
    for invalid in ([-1], [4], [[0]]):
        with pytest.raises(ValueError):
            subset_cloud(cloud, invalid)


def random_scan(rng, voxel_size):
    count = int(rng.integers(0, 300))
    # Quantized coordinates give many same-voxel duplicates, boundaries and -0.0.
    xyz = rng.integers(-40, 40, (count, 3)) * (voxel_size / 4)
    xyz[rng.random((count, 3)) < .05] = -0.0
    xyz += rng.normal(0, 1e-3, xyz.shape) * (rng.random((count, 3)) < .5)
    values = np.column_stack((xyz, rng.normal(0, 3, count), rng.uniform(5, 60, count)))
    return values, rng.permutation(4096)[:count]


@pytest.mark.parametrize('seed', range(10))
def test_temporal_evidence_and_cloud_match_dict_implementation(seed):
    rng = np.random.default_rng(100 + seed)
    config = AccumulationConfig(window_seconds=float(rng.choice([.3, .5, 1.0])),
                                voxel_size=float(rng.choice([.1, .25, .5])),
                                max_scans=int(rng.choice([3, 8, 32])),
                                max_observations=int(rng.choice([50, 400, 8192])),
                                min_support_scans=2)
    reference, actual = ReferenceEvidence(config), TemporalEvidence(config)
    stamp = 20_000_000_000 + int(rng.integers(0, 10**9))
    header = Header(frame_id='odom')
    for step in range(40):
        stamp += int(rng.integers(1, 250_000_000))
        now = stamp + int(rng.integers(0, 50_000_000))
        if step == 25:  # Backward clock jump clears both histories.
            stamp -= 5_000_000_000
            now = stamp
        values, indices = random_scan(rng, config.voxel_size)
        accepted = reference.add(values, indices, stamp, now)
        assert actual.add(values, indices, stamp, now) == accepted
        assert (reference.observations, reference.capacity_drops, len(reference.frames)) == (
            actual.observations, actual.capacity_drops, len(actual.frames))
        expected = reference.snapshot(now)
        records = actual.snapshot_array(now)
        assert reference_evidence_cloud(expected, header) == bytes(
            evidence_cloud(records, header).data)
        confirmed = [s for s in expected if s['support_scans'] >= config.min_support_scans]
        assert reference_evidence_cloud(confirmed, header) == bytes(evidence_cloud(
            records[records['support_scans'] >= config.min_support_scans], header).data)
        listed = actual.snapshot(now)
        assert len(listed) == len(expected)
        for old, new in zip(expected, listed):
            np.testing.assert_array_equal(old['value'], new['value'])
            assert {k: v for k, v in old.items() if k != 'value'} == {
                k: v for k, v in new.items() if k != 'value'}
        # The list form still packs identically.
        assert bytes(evidence_cloud(listed, header).data) == reference_evidence_cloud(
            expected, header)
