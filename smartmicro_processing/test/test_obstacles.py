# SPDX-License-Identifier: Apache-2.0
import numpy as np
import pytest

from smartmicro_processing.obstacles import (obstacle_points, ObstacleConfig, PersistenceFilter,
                                             to_frame)


def test_persistence_needs_k_of_n():
    f = PersistenceFilter(ObstacleConfig(persistence_hits=3, persistence_window=5))
    wall, blip = [5.0, 0.0, 0.0], [8.0, 2.0, 0.0]
    assert not f.step([wall, blip]).any()
    assert not f.step([wall]).any()
    assert f.step([wall]).tolist() == [True]
    assert f.step([blip]).tolist() == [False]


def test_selection_rules():
    c = ObstacleConfig(obstacle_height=0.3, include_tracks=True)
    static = np.array([[5.0, 0, 0], [0.5, 0, 0], [9.0, 1, 0]])
    persistent = np.array([True, False, True])
    novel = np.array([False, False, True])  # the far one appeared with the track
    movers = np.array([[2.0, 0.1, 0], [6.0, 0, 0], [2.1, 3.0, 0]])
    ghosts = np.array([False, True, False])
    points = obstacle_points(static, persistent, movers, ghosts, [[2.0, 0.0]], c, novel)
    kept = {tuple(np.round(p[:2], 1)) for p in points}
    # persistent wall, near-range return, on-track mover, the track itself
    assert kept == {(5.0, 0.0), (0.5, 0.0), (2.0, 0.1), (2.0, 0.0)}
    assert np.all(points[:, 2] == 0.3)


def kept_xy(points):
    return {tuple(np.round(p[:2], 1)) for p in points}


@pytest.mark.parametrize('half_angle,second_person_kept', [(15.0, True), (180.0, False)])
def test_shadow_rule_drops_only_returns_behind_a_track(half_angle, second_person_kept):
    # A track at +45 deg (2.8 m); a standing second person at -31 deg (5.8 m) is novel.
    c = ObstacleConfig(shadow_half_angle_deg=half_angle, include_tracks=False)
    static = np.array([[5.0, -3.0, 0], [4.5, 4.4, 0], [3.0, 3.0, 0]])
    points = obstacle_points(static, [True] * 3, np.empty((0, 3)), [], [[2.0, 2.0]], c,
                             novel=[True] * 3)
    # Behind the track at its bearing (> 2.8 + 1.5 m) is multipath either way; a
    # novel return just behind the track (within shadow_gap) is kept.
    expected = {(3.0, 3.0)} | ({(5.0, -3.0)} if second_person_kept else set())
    assert kept_xy(points) == expected


def test_shadow_rule_uses_each_tracks_own_range():
    c = ObstacleConfig(include_tracks=False)
    tracks = [[1.0, 0.0], [0.0, 4.0]]  # near track ahead, far track to the left
    static = np.array([[0.2, 6.0, 0], [0.0, 5.0, 0], [3.0, 0.1, 0]])
    points = obstacle_points(static, [True] * 3, np.empty((0, 3)), [], tracks, c,
                             novel=[True] * 3)
    # (0.2, 6) is behind the far track; (0, 5) is within its gap although far beyond
    # the near one; (3, 0.1) is behind the near track.
    assert kept_xy(points) == {(0.0, 5.0)}


def test_any_bearing_half_angle_reproduces_the_nearest_track_rule():
    rng = np.random.default_rng(3)
    c = ObstacleConfig(shadow_half_angle_deg=180.0, include_tracks=False)
    for _ in range(50):
        static = np.c_[rng.uniform(-10, 10, (40, 2)), np.zeros(40)]
        tracks = rng.uniform(-5, 5, (rng.integers(1, 4), 2))
        novel = rng.random(40) < .7
        points = obstacle_points(static, np.ones(40, bool), np.empty((0, 3)), [], tracks, c,
                                 novel)
        nearest = np.min(np.linalg.norm(tracks, axis=1))
        legacy = ~(novel & (np.linalg.norm(static[:, :2], axis=1) > nearest + c.shadow_gap))
        np.testing.assert_array_equal(points[:, :2], static[legacy, :2])


def test_measured_height_kept_when_negative():
    c = ObstacleConfig(obstacle_height=-1.0, include_tracks=False)
    points = obstacle_points([[3.0, 0, 0.7]], [True], np.empty((0, 3)), [], [], c)
    assert points[:, 2].tolist() == [0.7]


def test_to_frame_flattens_after_the_mount_transform():
    half = np.radians(5.0) / 2  # pitched 5 deg down, 0.5 m up
    mount = ([0.0, 0.0, 0.5], [0.0, np.sin(half), 0.0, np.cos(half)])
    points = to_frame([[10.0, 0.0, 0.0]], *mount, -1.0)
    np.testing.assert_allclose(points, [[10 * np.cos(2 * half), 0, .5 - 10 * np.sin(2 * half)]])
    assert points[0, 2] < 0  # below any min_obstacle_height without flattening
    assert to_frame([[10.0, 0.0, 0.0]], *mount, 0.3)[0, 2] == 0.3
    assert to_frame(np.empty((0, 3)), *mount, 0.3).shape == (0, 3)
    with pytest.raises(ValueError):
        to_frame([[1.0, 0.0, 0.0]], [0, 0, 0], [0, 0, 0, 0], 0.3)


def test_invalid_config():
    with pytest.raises(ValueError):
        ObstacleConfig(persistence_hits=6, persistence_window=5)
    for half_angle in (0.0, -5.0, 180.5, float('nan')):
        with pytest.raises(ValueError):
            ObstacleConfig(shadow_half_angle_deg=half_angle)
