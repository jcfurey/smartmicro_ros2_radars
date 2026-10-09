# SPDX-License-Identifier: Apache-2.0
"""Frame parsing of the custom CAN sender; needs neither python3-can nor a display."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest

# The tool imports python-can and Tk at module level; parse_frame uses neither.
for missing in ('can', 'tkinter'):
    if importlib.util.find_spec(missing) is None:
        stub = types.ModuleType(missing)
        stub.Button = stub.Checkbutton = stub.Entry = stub.IntVar = None
        stub.Label = stub.Text = None
        sys.modules[missing] = stub

_SPEC = importlib.util.spec_from_file_location(
    'custom_can_sender', Path(__file__).resolve().parents[1] / 'custom_can_sender.py')
sender = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(sender)


class ParseFrameTests(unittest.TestCase):
    """Payload bytes keep their positions; only trailing bytes may be empty."""

    def test_full_and_short_frames(self):
        self.assertEqual(sender.parse_frame('123', ['01', '02', '03', '04', 'a', 'B', 'c', 'FF']),
                         (0x123, False, [1, 2, 3, 4, 0xA, 0xB, 0xC, 0xFF]))
        self.assertEqual(sender.parse_frame('7ff', ['01', '02', '', '', '', '', '', '']),
                         (0x7FF, False, [1, 2]))
        self.assertEqual(sender.parse_frame('800', [''] * 8), (0x800, True, []))

    def test_gap_before_a_filled_byte_is_rejected(self):
        # C60: a blank middle byte used to be dropped, shifting 03 to index 1 (DLC 2).
        for payload in (['01', '', '03'], ['', '02'], ['01', ' ', '03', '', '']):
            with self.assertRaises(ValueError, msg=payload):
                sender.parse_frame('123', payload)

    def test_invalid_values(self):
        for can_id, payload in (('', ['01']), ('20000000', ['01']), ('123', ['100']),
                                ('123', ['-1']), ('123', ['zz'])):
            with self.assertRaises(ValueError, msg=(can_id, payload)):
                sender.parse_frame(can_id, payload)


if __name__ == '__main__':
    unittest.main()
