#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Apply/check the audited Smart Access Automotive 3.13.0 F32 header repair."""

import argparse
from pathlib import Path


ORIGINAL = '''        uint64_t *rawIntPointer = reinterpret_cast<uint64_t *>(&tmp);
        result = *rawIntPointer;
        result &= 0xFFFFFFFF;'''
FIXED = '''        // Local repair: read exactly the four bytes of the F32 value.
        uint32_t bits{};
        static_assert(sizeof(bits) == sizeof(tmp), "F32 must occupy four bytes");
        std::memcpy(&bits, &tmp, sizeof(bits));
        result = bits;'''


def repair(source):
    """Be idempotent, and refuse unfamiliar code instead of guessing a patch."""
    if source.count(FIXED) == 1 and '#include <cstring>' in source and ORIGINAL not in source:
        return source
    if source.count(ORIGINAL) != 1 or '#include <string>' not in source or FIXED in source:
        raise ValueError('Unrecognized SDK Instruction.h; review its F32 conversion before building')
    source = source.replace(ORIGINAL, FIXED)
    if '#include <cstring>' not in source:
        source = source.replace('#include <string>', '#include <string>\n#include <cstring>')
    return source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Verify without modifying the SDK')
    parser.add_argument('sdk', type=Path, help='Extracted smartmicro directory')
    args = parser.parse_args()
    header = args.sdk / 'include/Instruction.h'
    source = header.read_text()
    fixed = repair(source)
    if args.check and source != fixed:
        parser.error('SDK F32 repair is required: run tools/patch_smart_access.py without --check')
    if source != fixed:
        header.write_text(fixed)
        print(f'Applied four-byte F32 conversion repair to {header}')
    else:
        print('SDK F32 conversion repair verified')


if __name__ == '__main__':
    main()
