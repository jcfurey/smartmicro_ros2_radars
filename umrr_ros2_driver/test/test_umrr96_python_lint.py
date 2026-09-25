# SPDX-License-Identifier: Apache-2.0
"""flake8 and pep257 for the driver's Python package, scripts and their tests."""

from pathlib import Path

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257_main

ROOT = Path(__file__).resolve().parents[1]
PATHS = [str(ROOT / path) for path in (
    'umrr_ros2_driver_py', 'scripts', 'test/test_umrr96_views.py',
    'test/test_umrr96_measure.py', 'test/test_umrr96_python_lint.py')]


def test_flake8():
    code, errors = main_with_errors(argv=PATHS)
    assert code == 0, f'{len(errors)} flake8 errors:\n' + '\n'.join(errors)


def test_pep257():
    assert pep257_main(argv=PATHS) == 0, 'pep257 errors; see the output above'
