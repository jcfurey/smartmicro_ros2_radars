# SPDX-License-Identifier: Apache-2.0
from itertools import product

import numpy as np
import pytest

from smartmicro_processing.tracker import cluster, MovingObjectTracker, Track, TrackerConfig

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


@pytest.mark.parametrize('evidence,standing,joint', product((False, True), repeat=3))
def test_options_compose_across_confirmation_stop_and_departure(evidence, standing, joint):
    tracker = MovingObjectTracker(TrackerConfig(
        evidence_confirmation=evidence, standing_support=standing, joint_association=joint))
    wall = [[4., 0., 0.]]
    for k in range(100):
        tracker.step(k * DT, [], [], wall)
    for k in range(100, 108):
        stop = [1. + .5 * (k - 100) * DT, 0., 0.]
        visible = tracker.step(k * DT, [stop], [.5], wall)
        assert bool(visible) == (k >= (105 if evidence else 107))
    for k in range(108, 348):
        visible = tracker.step(k * DT, [], [], wall + [stop])
        if standing:
            assert len(visible) == 1 and visible[0].last_support == k * DT
    assert bool(visible) == standing
    for k in range(348, 373):
        visible = tracker.step(k * DT, [], [], wall)
    assert not visible


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


def stopped_tracker():
    tracker = MovingObjectTracker(TrackerConfig(standing_support=True))
    wall = [[4., 0., 0.]]
    for k in range(100):
        tracker.step(k * DT, [], [], wall)
    for k in range(100, 140):
        stop = [1. + .5 * (k - 100) * DT, 0., 0.]
        tracker.step(k * DT, [stop], [.5], wall)
    assert tracker.tracks[0].standing_background is not None
    return tracker, stop, wall, 140


def test_current_support_keeps_standing_person_beyond_legacy_hold_then_expires():
    tracker, stop, wall, start = stopped_tracker()
    for k in range(start, start + 220):
        visible = tracker.step(k * DT, [], [], wall + [stop])
        assert len(visible) == 1
        assert visible[0].last_support == k * DT
    # A learned wall cannot maintain the departed person's track.
    for k in range(start + 220, start + 245):
        visible = tracker.step(k * DT, [], [], wall)
    assert not visible


def test_standing_support_has_an_absolute_time_limit():
    tracker, stop, wall, start = stopped_tracker()
    tracker.config.standing_hold = 7.
    for k in range(start, start + 155):
        visible = tracker.step(k * DT, [], [], wall + [stop])
    assert not visible


def test_static_support_cannot_drift_away_from_stop_anchor():
    tracker, stop, wall, start = stopped_tracker()
    for k in range(start, start + 20):
        tracker.step(k * DT, [], [], wall + [stop])
    anchor = tracker.tracks[0].standing_anchor.copy()
    for j in range(100):
        k = start + 20 + j
        visible = tracker.step(k * DT, [], [], wall + [[stop[0], .025 * j, 0.]])
    assert not visible
    assert np.isfinite(anchor).all()


def test_sensor_motion_invalidates_the_saved_background():
    tracker, stop, wall, start = stopped_tracker()
    tracker.step(start * DT, [], [], wall + [stop], sensor_moving=True)
    assert tracker.tracks[0].standing_background is None
    assert tracker.tracks[0].standing_anchor is None


def test_unknown_static_returns_do_not_extend_standing_support():
    tracker, stop, wall, start = stopped_tracker()
    for k in range(start, start + 100):
        tracker.step(k * DT, [], [], wall + [stop])
    for k in range(start + 100, start + 125):
        visible = tracker.coast(k * DT)
    assert not visible


def test_verified_standing_track_accepts_intermittent_current_support():
    tracker, stop, wall, start = stopped_tracker()
    for k in range(start, start + 20):
        tracker.step(k * DT, [], [], wall + [stop])
    assert tracker.tracks[0].standing_verified
    for k in range(start + 20, start + 240):
        support = wall + ([stop] if k % 8 == 0 else [])
        visible = tracker.step(k * DT, [], [], support)
        assert len(visible) == 1
        if k % 8 == 0:
            assert visible[0].last_support == k * DT


def test_reappearing_preexisting_nearby_wall_cannot_take_over_a_standing_track():
    tracker = MovingObjectTracker(TrackerConfig(standing_support=True))
    wall = [[2.6, 0., 0.]]
    for k in range(100):
        tracker.step(k * DT, [], [], wall)
    for k in range(100, 141):
        stop = [1.1 + .5 * (k - 100) * DT, 0., 0.]
        tracker.step(k * DT, [stop], [.5], wall)
    for k in range(141, 381):
        assert tracker.step(k * DT, [], [], [stop])
    for k in range(381, 406):
        visible = tracker.step(k * DT, [], [], wall)
    assert not visible


def test_a_static_point_cannot_support_two_standing_tracks():
    tracker = MovingObjectTracker(TrackerConfig(standing_support=True))
    for ident, y in [(1, -.1), (2, .1)]:
        tracker.tracks.append(Track(ident, np.array([2., y, 0., 0.]), np.eye(4),
                                    0., 0., 0., confirmed=True, standing_background=frozenset()))
    selected = tracker.standing_observations(.5, np.array([[2., .05]]), False)
    assert len(selected[1]) == 0 and len(selected[2]) == 1


def association_track(track_id, x, velocity=(0., 0.)):
    return Track(track_id, np.r_[x, velocity], np.diag([.04, .04, 1., 1.]),
                 0., 0., 0., [True] * 10, confirmed=True, hits=10)


def test_joint_assignment_preserves_both_feasible_updates():
    tracker = MovingObjectTracker(TrackerConfig(joint_association=True))
    tracker.tracks = [association_track(1, [2., 0.]), association_track(2, [3., 0.])]
    tracker.next_id = 3
    tracker.step(DT, [[2.4, 0., 0.], [1.4, 0., 0.]], [-.5, -.5])
    assert len(tracker.tracks) == 2
    assert all(t.last_moving == DT for t in tracker.tracks)


def test_signed_velocity_breaks_an_ambiguous_position_match():
    tracker = MovingObjectTracker(
        TrackerConfig(joint_association=True, association_uncertainty=True))
    tracks = [association_track(1, [3., -.2], (.5, 0.)),
              association_track(2, [3., .2], (-.5, 0.))]
    for t in tracks:
        t.P[2:, 2:] *= .01
    # Positions slightly favor the opposite identity; Doppler favors continuity.
    obs = [(np.array([3., .1]), .5, 1), (np.array([3., -.1]), -.5, 1)]
    assert tracker.associate(tracks, obs) == {1: 0, 2: 1}
    tracker.config.association_doppler = False
    assert tracker.associate(tracks, obs) == {1: 1, 2: 0}


def test_joint_assignment_allows_misses_and_never_reuses_an_observation():
    tracker = MovingObjectTracker(TrackerConfig(joint_association=True))
    tracks = [association_track(1, [2., 0.]), association_track(2, [2.1, 0.])]
    assert len(tracker.associate(tracks, [(np.array([2.05, 0.]), 0., 1)])) == 1
    assert tracker.associate(tracks, [(np.array([10., 0.]), 0., 1)]) == {}
    assert tracker.associate(tracks, []) == {}
    assert tracker.associate([], [(np.array([2., 0.]), 0., 1)]) == {}


def test_joint_assignment_preserves_confirmed_priority_and_observation_permutation():
    tracker = MovingObjectTracker(TrackerConfig(joint_association=True))
    tracks = [association_track(1, [2., 0.]), association_track(2, [3., 0.])]
    obs = [(np.array([2.4, 0.]), 0., 1), (np.array([1.4, 0.]), 0., 1)]
    before = tracker.associate(tracks, obs)
    after = tracker.associate(tracks[::-1], obs[::-1])
    assert before == {k: 1-v for k, v in after.items()}
    tracks[1].confirmed = False
    tracks[1].x[:2] = [2.2, 0.]
    assert tracker.associate(tracks, [(np.array([2.2, 0.]), 0., 1)]) == {1: 0}
