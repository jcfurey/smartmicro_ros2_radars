# SPDX-License-Identifier: Apache-2.0
import numpy as np
import pytest

from smartmicro_processing.tracker import cluster, MovingObjectTracker, TrackerConfig

DT = 0.055


def radial(position, velocity):
    return float(np.dot(position, velocity) / np.linalg.norm(position))


def test_cluster_links_neighbours():
    labels = cluster(np.array([[0, 0], [0.4, 0], [0.8, 0], [5, 5]], float), 0.6)
    assert labels[0] == labels[1] == labels[2] != labels[3]


def test_track_confirms_and_estimates_velocity():
    tracker = MovingObjectTracker()
    velocity = np.array([0.0, 0.8])
    for k in range(60):
        position = np.array([2.0, -1.0]) + velocity * k * DT
        tracks = tracker.step(k * DT, [[*position, 0]], [radial(position, velocity)])
    assert len(tracks) == 1
    np.testing.assert_allclose(tracks[0].x[2:], velocity, atol=0.15)


def test_single_scan_blip_is_not_confirmed():
    tracker = MovingObjectTracker()
    assert tracker.step(0.0, [[3, 0, 0]], [0.5]) == []
    for k in range(1, 20):
        assert tracker.step(k * DT, [], []) == []


def test_farther_same_speed_track_is_ghost():
    tracker = MovingObjectTracker()
    for k in range(40):
        near = np.array([1.5 + 0.02 * k, 0.0])
        far = np.array([6.0 + 0.02 * k, 1.0])
        tracks = tracker.step(k * DT, [[*near, 0], [*far, 0]], [0.36, 0.36])
    assert [round(float(np.linalg.norm(t.x[:2]))) for t in tracks] == [2]


def test_standing_object_held_on_novel_static_support_not_on_background():
    tracker = MovingObjectTracker(TrackerConfig(background_time_constant=5.0))
    wall = [[4.0, y, 0.0] for y in (-1.0, 0.0, 1.0)]
    k = 0
    for k in range(200):  # learn the wall as background
        tracker.step(k * DT, [], [], wall)
    velocity = np.array([0.8, 0.0])
    for j in range(40):  # walk toward x = 2.3
        k += 1
        position = np.array([0.7, 0.0]) + velocity * j * DT
        tracker.step(k * DT, [[*position, 0]], [radial(position, velocity)], wall)
    stop = position
    for j in range(40):  # stand still for 2.2 s: only zero-Doppler detections remain
        k += 1
        tracks = tracker.step(k * DT, [], [], wall + [[*stop, 0]])
    assert len(tracks) == 1
    for j in range(40):  # person leaves; the wall alone must not keep the track
        k += 1
        tracks = tracker.step(k * DT, [], [], wall)
    assert tracks == []


def test_background_is_learned_after_warmup_not_time_constant():
    tracker = MovingObjectTracker(TrackerConfig(background_time_constant=30.0,
                                                background_warmup=3.0))
    wall = [[4.0, 0.0, 0.0]]
    k = 0
    while k * DT < 2.5:
        tracker.step(k * DT, [], [], wall)
        assert tracker.static_novel is None  # unlearned: callers must not drop returns
        k += 1
    while k * DT < 3.5:
        tracker.step(k * DT, [], [], wall)
        k += 1
    # Hit in every scan: background after the warm-up, not after ~21 s (0.7 time constants).
    assert tracker.background.ready and tracker.static_novel.tolist() == [False]
    tracker.step(k * DT, [], [], wall + [[2.0, 1.0, 0.0]])
    assert tracker.static_novel.tolist() == [False, True]


def test_moving_sensor_resets_background():
    tracker = MovingObjectTracker()
    wall = [[4.0, 0.0, 0.0]]
    for k in range(100):
        tracker.step(k * DT, [], [], wall)
    assert tracker.background.ready
    tracker.step(100 * DT, [], [], wall, sensor_moving=True)
    assert not tracker.background.ready and tracker.static_novel is None


def confirmed_tracker(scans=20):
    tracker = MovingObjectTracker()
    velocity = np.array([0.0, 0.8])
    for k in range(scans):
        position = np.array([2.0, -1.0]) + velocity * k * DT
        tracks = tracker.step(k * DT, [[*position, 0]], [radial(position, velocity)])
    assert len(tracks) == 1
    return tracker, position, scans * DT


def test_coast_records_a_miss_without_learning_background():
    tracker, _, t = confirmed_tracker()
    weight = tracker.background.weight
    tracks = tracker.coast(t)
    assert len(tracks) == 1 and tracks[0].history[-1] is False
    assert tracker.background.weight == weight and tracker.static_novel is None


def test_data_gap_longer_than_max_coast_drops_tracks():
    tracker, position, t = confirmed_tracker()
    old_id = tracker.tracks[0].track_id
    # A detection near the extrapolated position after a 2 s gap starts a new tentative track.
    assert tracker.step(t + 2.0, [[*position, 0]], [0.5]) == []
    assert [tr.track_id for tr in tracker.tracks] != [old_id]


def test_invalid_config():
    with pytest.raises(ValueError):
        TrackerConfig(confirm_hits=6, confirm_window=5)


def test_evidence_confirms_consistent_motion_in_six_scans_without_more_points():
    tracker = MovingObjectTracker(TrackerConfig(evidence_confirmation=True))
    for k in range(6):
        position = [2. + .5 * k * DT, 0., 0.]
        result = tracker.step(k * DT, [position] * 10, [.5] * 10)
        assert bool(result) == (k == 5)
    assert len(tracker.tracks[0].confirmation_evidence) == 6


def test_evidence_does_not_accelerate_inconsistent_motion_and_preserves_fallback():
    tracker = MovingObjectTracker(TrackerConfig(evidence_confirmation=True))
    for k in range(8):
        result = tracker.step(k * DT, [[2. + .2 * (-1) ** k, 0., 0.]], [.8 * (-1) ** k])
        if k < 7:
            assert not result
    assert result


def test_misses_reduce_confirmation_evidence_and_cannot_confirm():
    tracker = MovingObjectTracker(TrackerConfig(evidence_confirmation=True))
    for k in range(5):
        assert not tracker.step(k * DT, [[2. + .5 * k * DT, 0., 0.]], [.5])
    prior = sum(tracker.tracks[0].confirmation_evidence)
    assert not tracker.coast(5 * DT)
    assert sum(tracker.tracks[0].confirmation_evidence) == pytest.approx(prior - 1.)
    assert not tracker.coast(6 * DT)
    assert not tracker.step(7 * DT, [[2. + .5 * 7 * DT, 0., 0.]], [.5])


def test_evidence_resets_after_a_data_gap():
    tracker = MovingObjectTracker(TrackerConfig(evidence_confirmation=True))
    for k in range(5):
        tracker.step(k * DT, [[2. + .5 * k * DT, 0., 0.]], [.5])
    assert not tracker.step(2., [[3., 0., 0.]], [.5])
    assert tracker.tracks[0].confirmation_evidence == [1.]
