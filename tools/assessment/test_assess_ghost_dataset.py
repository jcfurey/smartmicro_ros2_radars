# SPDX-License-Identifier: Apache-2.0
"""Protect label semantics and independence of sensor/frame/ground truth."""
import unittest

import numpy as np

from assess_ghost_dataset import assess_rows, classify_rows, decode_labels, GhostConfig


DTYPE = [('frame', 'i4'), ('sensor', 'S6'), ('r_sc', 'f8'), ('phi_sc', 'f8'),
         ('vr_sc', 'f8'), ('label_id', 'i4'), ('group', '?')]


class GhostDatasetTest(unittest.TestCase):
    def test_documented_examples_and_ambiguous_order(self):
        labels = [1111, 1011, 2111, 1112, 1124, 2126, 2132, 2100, 2000,
                  1113, -1112, -3011, 0, -1, -2, 1211, 9999, 1121, 1119]
        expected = ['direct'] * 3 + ['specific_ghost'] * 4 + ['generic_ghost'] * 2
        expected += ['direct_ambiguous', 'uncertain', 'uncertain', 'background',
                     'ignore', 'noise', 'invalid', 'invalid', 'invalid', 'invalid']
        self.assertEqual(decode_labels(labels, np.zeros(len(labels), bool)).tolist(), expected)

    def test_group_excluded_and_shape_checked(self):
        self.assertEqual(decode_labels([1111], [True]).tolist(), ['group'])
        with self.assertRaises(ValueError):
            decode_labels([1111, 1112], [False])

    def test_unspecified_order_with_known_type(self):
        self.assertEqual(decode_labels([1110, 1120, 2110], [False] * 3).tolist(),
                         ['generic_ghost'] * 3)

    def test_sensor_and_frame_are_independent(self):
        rows = np.array([(0, b'left', 3, 0, 1, 1111, False),
                         (0, b'right', 8, 0, 1, 1111, False),
                         (1, b'left', 8, 0, 1, 1111, False)], dtype=DTYPE)
        _, _, reasons, count = classify_rows(rows, GhostConfig(), .05, .15, 120)
        self.assertEqual(reasons.tolist(), [0, 0, 0])
        self.assertEqual(count, 3)

    def test_unlabeled_point_can_trigger_rule_without_label_leakage(self):
        rows = np.array([(0, b'left', 3, 0, 1, 0, False),
                         (0, b'left', 8, 1, 1, 1111, False)], dtype=DTYPE)
        before = classify_rows(rows, GhostConfig(), .05, .15, 120)[2]
        self.assertEqual(before.tolist(), [0, 1])
        rows['label_id'] = [1124, -1]
        rows['group'] = True
        after = classify_rows(rows, GhostConfig(), .05, .15, 120)[2]
        np.testing.assert_array_equal(before, after)

    def test_invalid_measurements_do_not_trigger_and_boundary_is_static(self):
        rows = np.array([(0, b'left', .1, 0, 1, 1111, False),
                         (0, b'left', 3, np.nan, 1, 1111, False),
                         (0, b'left', 3, 0, .05, 0, False),
                         (0, b'left', 8, 0, 1, 1112, False)], dtype=DTYPE)
        valid, moving, reasons, _ = classify_rows(rows, GhostConfig(), .05, .15, 120)
        self.assertEqual(valid.tolist(), [False, False, True, True])
        self.assertEqual(moving.tolist(), [False, False, False, True])
        self.assertEqual(reasons.tolist(), [0, 0, 0, 4])

    def test_scoring_denominators_and_overlapping_rules(self):
        rows = np.array([(0, b'left', 2, 0, 0, 0, False),
                         (0, b'left', 3, 0, 1, 1111, False),
                         (0, b'left', 8, 0, 1, 1112, False),
                         (0, b'left', 9, 0, 1, 1113, False),
                         (0, b'left', 10, 0, 1, -1112, False),
                         (0, b'left', 11, 0, 1, 1100, False)], dtype=DTYPE)
        report, _ = assess_rows(rows, GhostConfig())
        metrics = report['variants']['all']
        self.assertEqual(metrics['direct_total'], 1)
        self.assertEqual(metrics['direct_retention'], 1.)
        self.assertEqual(metrics['specific_ghost_total'], 1)
        self.assertEqual(metrics['specific_ghost_rejected'], 1)
        self.assertEqual(metrics['generic_ghost_rejection'], 1.)
        self.assertEqual(report['categories_moving']['direct_ambiguous'], 1)
        self.assertEqual(report['variants']['none']['specific_ghost_rejection'], 0.)

    def test_empty_data_has_null_rates(self):
        report, _ = assess_rows(np.empty(0, dtype=DTYPE), GhostConfig())
        self.assertEqual(report['sensor_frames'], 0)
        self.assertIsNone(report['variants']['all']['direct_retention'])


if __name__ == '__main__':
    unittest.main()
