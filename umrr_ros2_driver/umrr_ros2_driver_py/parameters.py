# SPDX-License-Identifier: Apache-2.0
"""Documented parameter declaration that accepts integers for floating-point values."""

import math

from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor


def as_float(name, value):
    """Coerce an INTEGER or DOUBLE parameter value; YAML ``20`` means ``20.0``."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a number, got {value!r}')
    return float(value)


def declare(node, name, default, description, low=None, high=None, read_only=True):
    """
    Declare a parameter with a description and optional range; return its value.

    Floating-point parameters use dynamic typing and are coerced, so an integer
    override such as ``max_range: 20`` or ``publish_hz:=5`` does not raise
    ``InvalidParameterTypeException``. Runtime-writable ones must validate and
    coerce in their set-parameters callbacks as well.
    """
    descriptor = ParameterDescriptor(description=description, read_only=read_only)
    ranged = low is not None and high is not None
    if isinstance(default, float):
        descriptor.dynamic_typing = True
        if ranged:
            descriptor.floating_point_range = [FloatingPointRange(
                from_value=float(low), to_value=float(high), step=0.0)]
            descriptor.additional_constraints = f'Number within [{low:g}, {high:g}]'
        value = as_float(name, node.declare_parameter(name, default, descriptor).value)
        if ranged and not (math.isfinite(value) and low <= value <= high):
            raise ValueError(f'{name} must be within {low:g}..{high:g}, got {value:g}')
        return value
    if isinstance(default, int) and not isinstance(default, bool) and ranged:
        descriptor.integer_range = [IntegerRange(from_value=int(low), to_value=int(high),
                                                 step=0)]
    return node.declare_parameter(name, default, descriptor).value
