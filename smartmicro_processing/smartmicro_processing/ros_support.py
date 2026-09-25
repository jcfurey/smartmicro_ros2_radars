# SPDX-License-Identifier: Apache-2.0
"""Parameter declaration and diagnostics rate limiting shared by the processing nodes."""
import math
import time

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor


def as_float(name, value):
    """Coerce an INTEGER or DOUBLE parameter value; YAML ``120`` means ``120.0``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a number, got {value!r}')
    return float(value)


def declare(node, name, default, description, low=None, high=None, step=None,
            read_only=True):
    """
    Declare a documented parameter and return its (coerced) value.

    Floating-point parameters use dynamic typing so an integer override such as
    ``max_range: 120`` or ``publish_hz:=5`` is accepted and converted, instead of
    raising ``InvalidParameterTypeException`` at startup. The optional range is
    published in the descriptor and enforced by rclpy for values of the declared
    type; callers still validate combined constraints.
    """
    descriptor = ParameterDescriptor(description=description, read_only=read_only)
    if isinstance(default, float):
        descriptor.dynamic_typing = True
        if low is not None and high is not None:
            descriptor.floating_point_range = [FloatingPointRange(
                from_value=float(low), to_value=float(high), step=0.0)]
            descriptor.additional_constraints = f'Number within [{low:g}, {high:g}]'
        value = as_float(name, node.declare_parameter(name, default, descriptor).value)
        if low is not None and high is not None and not (
                math.isfinite(value) and low <= value <= high):
            raise ValueError(f'{name} must be within {low:g}..{high:g}, got {value:g}')
        return value
    if isinstance(default, int) and not isinstance(default, bool):
        if low is not None and high is not None:
            descriptor.integer_range = [IntegerRange(
                from_value=int(low), to_value=int(high), step=int(step or 0))]
    return node.declare_parameter(name, default, descriptor).value


class DiagnosticsRateLimiter:
    """
    Allow a diagnostics message on a key change, otherwise at most once per period.

    ``diagnostic_updater``'s Python ``Updater`` is not used: it renames statuses
    to ``<node>: <task>`` (breaking the documented status names) and runs on the
    ROS clock, so it stops while a bag's ``/clock`` is paused. This uses the
    steady clock, like the input watchdogs.
    """

    def __init__(self, period=1.0):
        self.period = period
        self.last_key = None
        self.last_time = None

    def due(self, key, force=False):
        now = time.monotonic()
        if (force or key != self.last_key or self.last_time is None
                or now - self.last_time >= self.period):
            self.last_key, self.last_time = key, now
            return True
        return False
