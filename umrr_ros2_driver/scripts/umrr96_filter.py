# SPDX-License-Identifier: Apache-2.0
"""Compatibility shim for scripts that put this directory on ``sys.path``."""

try:
    from umrr_ros2_driver_py.detection_filter import DetectionFilter  # noqa: F401
except ImportError:  # Unsourced source tree: use the package next to this directory.
    import pathlib
    import sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
    from umrr_ros2_driver_py.detection_filter import DetectionFilter  # noqa: F401
