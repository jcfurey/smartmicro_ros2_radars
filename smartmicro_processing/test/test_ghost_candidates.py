# SPDX-License-Identifier: Apache-2.0
import math

import numpy as np
import pytest

from smartmicro_processing.ghost_candidates import copy_evidence, CopyEvidenceConfig


def polar(r, az, el=0):
    a, e = math.radians(az), math.radians(el)
    return [r * math.cos(e) * math.cos(a), r * math.cos(e) * math.sin(a), r * math.sin(e)]


def test_labeled_opposite_direction_counterexample_is_retained():
    result = copy_evidence([polar(8.990, -12.279), polar(11.524, 13.232)], [1.051, -1.184])
    assert not any(mask.any() for mask in result.values())


def test_same_speed_other_bearing_requires_more_than_speed():
    result = copy_evidence([polar(3, -30), polar(3.2, -29), polar(8, 30)], [.5, .5, .5])
    assert result['signed_pair'].tolist() == [False, False, True]
    assert not result['aligned_pair'].any()
    assert not result['supported_copy'].any()


def test_two_spatially_consistent_returns_support_aligned_copy():
    result = copy_evidence([polar(3, 0), polar(3.2, 1), polar(8, 1)], [.5, .55, .51])
    assert result['supported_copy'].tolist() == [False, False, True]


def test_one_return_or_duplicates_cannot_manufacture_corroboration():
    for near in ([polar(3, 0)], [polar(3, 0)] * 5):
        result = copy_evidence(near + [polar(8, 1)], [.5] * (len(near) + 1))
        assert result['aligned_pair'][-1]
        assert not result['supported_copy'].any()


def test_scattered_nearer_points_do_not_form_one_support_group():
    result = copy_evidence([polar(3, 0), polar(5, 0), polar(8, 0)], [.5] * 3)
    assert result['aligned_pair'][-1]
    assert not result['supported_copy'].any()


def test_full_direction_includes_elevation_and_wraps_bearing():
    wrapped = copy_evidence([polar(3, -179), polar(3.2, -179), polar(8, 179)], [.5] * 3)
    assert wrapped['supported_copy'][-1]
    high = copy_evidence([polar(3, 0), polar(3.2, 0), polar(8, 0, 30)], [.5] * 3)
    assert not high['aligned_pair'].any()


def test_input_order_and_global_doppler_sign_do_not_change_evidence():
    xyz = np.array([polar(3, 0), polar(3.2, 1), polar(8, 1), polar(9, -40)])
    speed = np.array([.5, .55, .51, -.5])
    order = np.array([2, 0, 3, 1])
    original = copy_evidence(xyz, speed)
    permuted = copy_evidence(xyz[order], -speed[order])
    for key in original:
        np.testing.assert_array_equal(permuted[key], original[key][order])


def test_invalid_and_stationary_measurements_are_not_evidence():
    xyz = [[np.nan, 0, 0], [0, 0, 0], [3, 0, 0], [3.2, 0, 0], [8, 0, 0]]
    with np.errstate(invalid='ignore'):
        result = copy_evidence(xyz, [.5, .5, 0, 0, .5])
    assert not any(mask.any() for mask in result.values())
    assert not copy_evidence([], [])['supported_copy'].size
    with pytest.raises(ValueError):
        copy_evidence([[1, 0, 0]], [])
    with pytest.raises(ValueError):
        CopyEvidenceConfig(direction_deg=181)
    with pytest.raises(ValueError):
        CopyEvidenceConfig(speed_tolerance=float('nan'))


def test_aligned_independent_objects_remain_an_explicit_limitation():
    # Measurement-only evidence cannot distinguish this from a reflected copy.
    result = copy_evidence([polar(3, 0), polar(3.2, 1), polar(8, 0)], [.5] * 3)
    assert result['supported_copy'][-1]
