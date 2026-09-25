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


def test_invalid_config():
    with pytest.raises(ValueError):
        TrackerConfig(confirm_hits=6, confirm_window=5)
