# SPDX-License-Identifier: Apache-2.0
import math

import numpy as np
import pytest

from smartmicro_processing.ghosts import ghost_mask, ghost_rejection_mask, GhostConfig


def polar(r, az_deg):
    a = math.radians(az_deg)
    return [r * math.cos(a), r * math.sin(a), 0.0]


def test_farther_same_speed_copy_is_ghost():
    movers = [polar(2.0, 0), polar(2.3, 5), polar(6.0, -30)]
    ghosts = ghost_mask(movers, [0.5, -0.45, 0.55], [])
    assert ghosts.tolist() == [False, False, True]


def test_different_speed_far_mover_is_kept():
    ghosts = ghost_mask([polar(2.0, 0), polar(6.0, -30)], [0.5, 1.5], [])
    assert ghosts.tolist() == [False, False]


def test_mover_behind_static_return_is_ghost():
    static = [polar(3.0, 20.0)]
    ghosts = ghost_mask([polar(5.0, 22.0), polar(5.0, 40.0), polar(2.0, 20.0)],
                        [1.0, 1.0, 3.0], static)
    assert ghosts.tolist() == [True, False, False]


def test_bearing_wraps_at_pi():
    ghosts = ghost_mask([polar(5.0, 179.0)], [1.0], [polar(2.0, -179.0)])
    assert ghosts.tolist() == [True]


def test_empty_inputs():
    assert ghost_mask(np.empty((0, 3)), [], np.empty((0, 3))).shape == (0,)


def test_invalid_config():
    with pytest.raises(ValueError):
        GhostConfig(range_gap=0)


def test_static_only_policy_preserves_all_other_rejection_triggers():
    reasons = np.arange(8, dtype=np.uint8)
    assert ghost_rejection_mask(reasons).tolist() == [False] + [True] * 7
    assert ghost_rejection_mask(reasons, reject_static_only=False).tolist() == [
        False, True, True, True, False, True, True, True]
    np.testing.assert_array_equal(reasons, np.arange(8, dtype=np.uint8))
    with pytest.raises(ValueError):
        ghost_rejection_mask(reasons, reject_static_only='false')


def test_static_only_advisory_does_not_disable_speed_copy_rejection():
    assert not ghost_mask([[6, 0, 0]], [1.], [[3, 0, 0]], reject_static_only=False)[0]
    assert ghost_mask([[2, 0, 0], [6, 0, 0]], [1., 1.], [[3, 0, 0]],
                      reject_static_only=False).tolist() == [False, True]
