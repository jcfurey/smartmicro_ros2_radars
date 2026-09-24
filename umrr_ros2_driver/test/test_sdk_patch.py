# SPDX-License-Identifier: Apache-2.0
"""Check extraction repair idempotence and fail-closed handling of unfamiliar SDKs."""
import importlib.util
from pathlib import Path
import pytest

path = Path(__file__).resolve().parents[2] / 'tools/patch_smart_access.py'
spec = importlib.util.spec_from_file_location('sdk_patch', path)
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)


def test_repair_is_idempotent():
    source = '#include <string>\n' + patch.ORIGINAL
    repaired = patch.repair(source)
    assert patch.ORIGINAL not in repaired
    assert patch.FIXED in repaired
    assert repaired.count('#include <cstring>') == 1
    assert patch.repair(repaired) == repaired


def test_unknown_or_duplicate_code_is_rejected():
    for source in ('unfamiliar SDK', '#include <string>\n' + patch.ORIGINAL * 2,
                   '#include <string>\n#include <cstring>\n' + patch.FIXED * 2):
        with pytest.raises(ValueError, match='Unrecognized'):
            patch.repair(source)
