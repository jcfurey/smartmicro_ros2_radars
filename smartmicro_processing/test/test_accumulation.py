# SPDX-License-Identifier: Apache-2.0
import math

import numpy as np
import pytest
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import read_points
from smartmicro_processing.accumulation import (AccumulationConfig, TemporalEvidence,
                                                transform_measurements)
from smartmicro_processing.evidence_cloud import evidence_cloud
from std_msgs.msg import Header


def add(evidence, values, stamp, now=None):
    values = np.asarray(values, dtype=float).reshape(-1, 5)
    return evidence.add(values, np.arange(len(values)), stamp, stamp if now is None else now)


def test_transform_axes_and_source_radiometry():
    values = np.array([[1., 0, 0, -.7, 42.]])
    actual = transform_measurements(values, [.3, -.2, .4], [0, 0, math.sqrt(.5), math.sqrt(.5)])
    np.testing.assert_allclose(actual, [[.3, .8, .4, -.7, 42.]], atol=1e-12)
    np.testing.assert_allclose(values, [[1, 0, 0, -.7, 42]])


def test_translation_and_turn_compensate_known_landmark_without_filling_surfaces():
    world = np.array([4.3, 2.2, .6])
    compensated, uncompensated = TemporalEvidence(), TemporalEvidence()
    observations = [([4.3, 2.2, .6], [0, 0, 0], [0, 0, 0, 1]),
                    ([3.3, 2.2, .6], [1, 0, 0], [0, 0, 0, 1]),
                    ([1.2, -3.3, .6], [1, 1, 0], [0, 0, math.sqrt(.5), math.sqrt(.5)])]
    for index, (xyz, translation, quaternion) in enumerate(observations):
        stamp = 1_000_000_000 + index * 100_000_000
        value = np.array([[*xyz, -.2, 30.]])
        add(compensated, transform_measurements(value, translation, quaternion), stamp)
        add(uncompensated, value, stamp)
    result = compensated.snapshot(stamp)
    assert len(result) == 1 and result[0]['support_scans'] == 3
    np.testing.assert_allclose(result[0]['value'][:3], world, atol=1e-12)
    assert len(uncompensated.snapshot(stamp)) == 3


def test_same_scan_duplicates_have_one_vote_and_newest_real_point_is_representative():
    evidence = TemporalEvidence()
    add(evidence, [[1.1, .1, .1, 0, 20]]*100, 1_000_000_000)
    first = evidence.snapshot(1_000_000_000)
    assert len(first) == 1 and first[0]['support_scans'] == 1
    add(evidence, [[1.2, .2, .2, -.1, 9999]], 1_100_000_000)
    sample = evidence.snapshot(1_200_000_000)[0]
    assert sample['support_scans'] == 2
    np.testing.assert_allclose(sample['value'], [1.2, .2, .2, -.1, 9999])
    assert sample['age_seconds'] == pytest.approx(.1)
    assert sample['span_seconds'] == pytest.approx(.1)


def test_expiration_recomputes_support_and_removes_last_observation_at_deadline():
    evidence = TemporalEvidence()
    for stamp in (1_000_000_000, 1_200_000_000):
        add(evidence, [[1, 0, 0, 0, 30]], stamp)
    assert evidence.snapshot(1_499_999_999)[0]['support_scans'] == 2
    remaining = evidence.snapshot(1_500_000_000)[0]
    assert remaining['support_scans'] == 1 and remaining['span_seconds'] == 0
    assert evidence.snapshot(1_700_000_000) == []
    assert evidence.observations == 0


def test_capacity_limits_and_negative_voxel_boundaries():
    evidence = TemporalEvidence(AccumulationConfig(max_scans=2, max_observations=3))
    add(evidence, [[-.01, 0, 0, 0, 30], [.01, 0, 0, 0, 30]], 1_000_000_000)
    assert len(evidence.snapshot(1_000_000_000)) == 2
    add(evidence, [[1, 0, 0, 0, 30], [2, 0, 0, 0, 30]], 1_100_000_000)
    assert evidence.observations == 2 and evidence.capacity_drops == 2
    add(evidence, [[3, 0, 0, 0, 30]], 1_200_000_000)
    assert evidence.observations == 3
    add(evidence, [[4, 0, 0, 0, 30]], 1_300_000_000)
    assert len(evidence.frames) == 2 and evidence.observations == 2
    assert evidence.capacity_drops == 4


def test_oversized_scan_is_bounded_and_reports_lost_representatives():
    evidence = TemporalEvidence(AccumulationConfig(max_observations=2))
    add(evidence, [[x, 0, 0, 0, 30] for x in range(4)], 1_000_000_000)
    assert evidence.observations == 2 and evidence.capacity_drops == 2


def test_duplicate_stamp_rejected_and_clock_rewind_clears_old_epoch():
    evidence = TemporalEvidence()
    values = [[1, 0, 0, 0, 30]]
    add(evidence, values, 10_000_000_000)
    with pytest.raises(ValueError, match='increase strictly'):
        add(evidence, values, 10_000_000_000)
    assert evidence.snapshot(1_000_000_000) == []
    add(evidence, values, 1_100_000_000)
    assert evidence.snapshot(1_100_000_000)[0]['support_scans'] == 1


def test_expired_delayed_scan_does_not_renew_lifetime():
    evidence = TemporalEvidence()
    assert not add(evidence, [[1, 0, 0, 0, 30]], 1_000_000_000, 1_500_000_000)
    assert evidence.snapshot(1_500_000_000) == []


def test_evidence_cloud_has_explicit_source_metadata_and_empty_layout():
    evidence = TemporalEvidence()
    values = np.array([[4.3, 2.2, .6, -.4, 35.]])
    evidence.add(values, np.array([7]), 1_123_456_789, 1_123_456_789)
    header = Header(frame_id='odom')
    message = evidence_cloud(evidence.snapshot(1_223_456_789), header)
    assert all(f.datatype == PointField.FLOAT32 for f in message.fields[:3])
    point = read_points(message)[0]
    assert point['source_point_index'] == 7
    assert point['source_stamp_sec'] == 1 and point['source_stamp_nanosec'] == 123_456_789
    assert point['age_seconds'] == pytest.approx(.1)
    assert point['source_radial_speed'] == -.4 and point['source_snr'] == 35
    np.testing.assert_allclose([point['x'], point['y'], point['z']], values[0, :3])
    empty = evidence_cloud([], header)
    assert not len(read_points(empty)) and empty.fields == message.fields


@pytest.mark.parametrize('options', [{'window_seconds': 0}, {'window_seconds': float('nan')},
                                     {'voxel_size': .001}, {'max_scans': 0},
                                     {'min_support_scans': 33}, {'max_observations': 2.5}])
def test_invalid_config(options):
    with pytest.raises(ValueError):
        AccumulationConfig(**options)


@pytest.mark.parametrize('translation,rotation', [([0, 0, 0], [0, 0, 0, 0]),
                                                  ([float('nan'), 0, 0], [0, 0, 0, 1])])
def test_invalid_transform_rejected(translation, rotation):
    with pytest.raises(ValueError):
        transform_measurements([[1, 0, 0, 0, 30]], translation, rotation)
