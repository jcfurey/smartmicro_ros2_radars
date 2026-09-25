# SPDX-License-Identifier: Apache-2.0
import numpy as np
import pytest

from smartmicro_processing.obstacles import obstacle_points, ObstacleConfig, PersistenceFilter


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


def test_measured_height_kept_when_negative():
    c = ObstacleConfig(obstacle_height=-1.0, include_tracks=False)
    points = obstacle_points([[3.0, 0, 0.7]], [True], np.empty((0, 3)), [], [], c)
    assert points[:, 2].tolist() == [0.7]


def test_invalid_config():
    with pytest.raises(ValueError):
        ObstacleConfig(persistence_hits=6, persistence_window=5)
