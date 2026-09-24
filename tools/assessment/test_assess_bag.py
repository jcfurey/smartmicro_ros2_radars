# SPDX-License-Identifier: Apache-2.0
"""Numerical checks for the exploratory Doppler audit, not sensor validation."""
import unittest

import numpy as np

from assess_bag import distribution, geometry_condition, planar_fit


class FitTests(unittest.TestCase):
    def test_recovers_translation_with_dynamic_outliers(self):
        azimuth = np.linspace(-1.1, 1.1, 40)
        directions = np.column_stack((np.cos(azimuth), np.sin(azimuth), np.zeros(40)))
        truth = np.array([1.2, -.4])
        doppler = -directions[:, :2] @ truth
        doppler[::5] += 3
        result = planar_fit(directions, doppler, np.random.default_rng(0))
        np.testing.assert_allclose([result['vx'], result['vy']], truth, atol=1e-10)
        self.assertAlmostEqual(result['consensus_fraction'], .8)
        self.assertLess(result['consensus_rmse'], 1e-10)

    def test_rejects_unobservable_lateral_velocity(self):
        directions = np.tile([1., 0., 0.], (20, 1))
        self.assertTrue(np.isinf(geometry_condition(directions[:, :2])))
        self.assertIsNone(planar_fit(directions, np.ones(20), np.random.default_rng(0)))

    def test_rejects_missing_and_nonfinite_inputs(self):
        self.assertIsNone(planar_fit(np.empty((0, 3)), [], np.random.default_rng(0)))
        self.assertIsNone(planar_fit(np.full((20, 3), np.nan), np.ones(20),
                                     np.random.default_rng(0)))

    def test_nonfinite_is_reported_unavailable(self):
        result = distribution([np.nan, np.inf])
        self.assertEqual(result['count'], 2)
        self.assertEqual(result['finite'], 0)
        self.assertIsNone(result['median'])


if __name__ == '__main__':
    unittest.main()
