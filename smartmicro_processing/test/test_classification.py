# SPDX-License-Identifier: Apache-2.0
"""Audit provenance, overlapping reasons, and current-scan visualization contracts."""
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points
from smartmicro_processing.classification import classified_cloud, classify, cleared_audit
from smartmicro_processing.cloud import GateConfig, select_measurements
from smartmicro_processing.doppler import FitResult
from smartmicro_processing.ghosts import ghost_reasons, GhostReason
from std_msgs.msg import Header
from umrr_ros2_msgs.msg import DetectionAudit as Audit


def test_every_raw_index_has_a_decision_and_overlapping_reasons_survive():
    values = np.array([[np.nan, 0, 0, 0, 0], [121, 0, 0, 0, 0], [2, 0, 0, 0, 0],
                       [3, 1, 0, 0, 30], [4, 2, 0, .5, 30], [5, 1, 0, .5, 30],
                       [6, 2, 0, 1, 30], [7, 1, 0, .5, 30]])
    indices, stats = select_measurements(values)
    fit = FitResult(True, 'valid', inliers=np.array([True, False, False, False, False]))
    reasons = np.array([0, GhostReason.SAME_SPEED, GhostReason.DOUBLE_SPEED,
                        GhostReason.SAME_SPEED | GhostReason.BEHIND_STATIC], dtype=np.uint8)
    header = Header(frame_id='umrr96')
    header.stamp.sec = 10
    audit = classify(header, values, GateConfig(), fit, indices, reasons, [4])
    assert list(audit.source_index) == list(range(8))
    assert list(audit.classification) == [Audit.REJECTED] * 3 + [
        Audit.STATIC, Audit.MOVING, Audit.SUSPECTED_GHOST, Audit.SUSPECTED_GHOST,
        Audit.SUSPECTED_GHOST]
    assert list(audit.reason_flags) == [
        Audit.NONFINITE, Audit.OUT_OF_RANGE, Audit.LOW_SNR, 0, 0, Audit.GHOST_SAME_SPEED,
        Audit.GHOST_DOUBLE_SPEED, Audit.GHOST_SAME_SPEED | Audit.GHOST_BEHIND_STATIC]
    assert list(audit.track_associated) == [False] * 4 + [True] + [False] * 3
    assert audit.fit_valid and audit.event == Audit.SCAN and stats['accepted'] == 5
    cloud = classified_cloud(header, values, audit)
    points = read_points(cloud, skip_nans=False)
    # Nonfinite XYZ cannot be rendered; its audit entry still exists. Quality
    # rejects with finite XYZ remain visible, never silently disappear.
    assert points['source_index'].tolist() == list(range(1, 8))
    np.testing.assert_array_equal(points['x'], values[1:, 0].astype(np.float32))
    assert points['reason_flags'].tolist() == list(audit.reason_flags)[1:]
    assert points['rgb'].tolist() == [
        0x888888, 0x888888, 0xEBEBEB, 0xFF3C28, 0x5A8CFF, 0x5A8CFF, 0x5A8CFF]
    header.frame_id = 'changed'
    assert cloud.header.frame_id == audit.header.frame_id == 'umrr96'
    assert cloud.header.stamp.sec == 10


def test_failed_fit_retains_current_measurements_and_clear_is_not_an_empty_scan():
    header = Header(frame_id='umrr96')
    values = np.array([[2., 0, 0, 0, 30], [3, 0, 0, 0, 30]])
    indices, _ = select_measurements(values)
    audit = classify(header, values, GateConfig(), FitResult(False, 'insufficient_points'),
                     indices, np.empty(0, np.uint8), [])
    assert list(audit.classification) == [Audit.UNCLASSIFIED] * 2
    assert list(audit.reason_flags) == [Audit.FIT_REJECTED] * 2
    points = read_points(classified_cloud(header, values, audit))
    assert points['rgb'].tolist() == [0xFFC94A] * 2
    empty = np.empty((0, 5))
    empty_audit = classify(header, empty, GateConfig(), FitResult(False, 'insufficient_points'),
                           np.empty(0, int), np.empty(0, np.uint8), [])
    clear = cleared_audit(header, 'input_stale')
    assert empty_audit.event == Audit.SCAN and clear.event == Audit.CLEAR
    for message in (empty_audit, clear):
        cloud = classified_cloud(header, empty, message)
        assert cloud.width == 0 and len(cloud.data) == 0
        assert 'rgb' in [f.name for f in cloud.fields]
        assert not len(message.source_index)


def test_nonfinite_speed_is_audited_but_finite_position_is_visible():
    values = np.array([[2., 0, 0, np.nan, 30], [1e100, 0, 0, 0, 30]])
    indices, _ = select_measurements(values)
    audit = classify(Header(), values, GateConfig(), FitResult(False, 'insufficient_points'),
                     indices, np.empty(0, np.uint8), [])
    assert list(audit.reason_flags) == [Audit.NONFINITE, Audit.OUT_OF_RANGE]
    # Float64 positions beyond display float32 range remain audited, not rendered as inf.
    cloud = classified_cloud(Header(), values, audit)
    assert read_points(cloud)['source_index'].tolist() == [0]


def test_ghost_reason_overlap_and_independent_mover_counterexample_are_explained():
    movers = [[2, 0, 0], [5, 0, 0]]
    reasons = ghost_reasons(movers, [.1, .1], [[3, 0, 0]])
    assert reasons.tolist() == [
        0, int(GhostReason.SAME_SPEED | GhostReason.DOUBLE_SPEED | GhostReason.BEHIND_STATIC)]
    # Preserve, expose, and do not silently "fix" the established heuristic:
    # opposite-speed direct movers at different bearings can still trigger it.
    independent = [[2, 0, 0], [2.5, 5 * np.sin(np.pi / 3), 0]]
    reasons = ghost_reasons(independent, [.5, -.5], [])
    assert reasons.tolist() == [0, int(GhostReason.SAME_SPEED)]
