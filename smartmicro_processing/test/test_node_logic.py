# SPDX-License-Identifier: Apache-2.0
"""In-process node checks: parameters, empty clouds, pose steps, stamps and diagnostics."""
import math
import time

import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from smartmicro_processing.accumulation import pose_discontinuity, pose_step_limits
from smartmicro_processing.accumulation_node import RadarAccumulation
from smartmicro_processing.cloud import empty_cloud
from smartmicro_processing.node import RadarProcessing
from smartmicro_processing.ros_support import as_float, DiagnosticsRateLimiter
from std_msgs.msg import Header

FIELDS = [PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, name in enumerate(('x', 'y', 'z', 'radial_speed', 'snr'))]


@pytest.fixture
def ros():
    def start(*overrides):
        args = ['--ros-args']
        for override in overrides:
            args += ['-p', override]
        rclpy.init(args=args)
    yield start
    rclpy.try_shutdown()


def stamp(ns):
    return Time(nanoseconds=ns).to_msg()


def cloud(ns, points, frame='umrr96'):
    return create_cloud(Header(frame_id=frame, stamp=stamp(ns)), FIELDS,
                        np.asarray(points, dtype=np.float32).reshape(-1, 5))


class Recorder:
    """Publisher stand-in with one subscriber."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


def test_integer_overrides_are_accepted_for_float_parameters(ros):
    ros('max_range:=120', 'max_input_age:=1', 'noise_floor:=1', 'min_snr_db:=-5')
    node = RadarProcessing()
    try:
        assert node.gate_config.max_range == 120.0 and type(node.gate_config.max_range) is float
        assert node.max_age == 1.0 and node.fit_config.noise_floor == 1.0
        assert node.gate_config.min_snr_db == -5.0
        descriptor = node.describe_parameter('max_range')
        assert descriptor.description and descriptor.floating_point_range[0].to_value == 300
        assert node.describe_parameter('doppler_sign').integer_range[0].step == 2
    finally:
        node.destroy_node()


def test_accumulation_integer_rate_and_non_numeric_rejection(ros):
    ros('publish_hz:=5', 'voxel_size:=1', 'mode:=stationary_preview')
    node = RadarAccumulation()
    try:
        assert node.timer.timer_period_ns == 200_000_000
        assert node.config.voxel_size == 1.0
    finally:
        node.destroy_node()
    with pytest.raises(ValueError):
        as_float('publish_hz', 'fast')
    with pytest.raises(ValueError):
        as_float('publish_hz', True)


def test_pose_step_limit_scales_with_scan_spacing():
    still = ([0, 0, 0], [0, 0, 0, 1])
    moved = ([3.0, 0, 0], [0, 0, 0, 1])
    # 3 m in 0.1 s exceeds 1 m + 15 m/s * 0.1 s = 2.5 m: a discontinuity...
    assert pose_discontinuity(still, moved, *pose_step_limits(1.0, .5, .1, 15.0, 2.0))
    # ...but 3 m in 0.2 s (one dropped scan at 10 Hz) is within 1 m + 3 m.
    assert not pose_discontinuity(still, moved, *pose_step_limits(1.0, .5, .2, 15.0, 2.0))
    assert pose_step_limits(1.0, .5, 10.0, 15.0, 2.0)[1] == math.pi
    assert pose_step_limits(1.0, .5, -1.0, 15.0, 2.0) == (1.0, .5)


def test_accumulation_fast_motion_keeps_history_and_jump_warns(ros):
    ros('mode:=stationary_preview')
    node = RadarAccumulation()
    try:
        node.last_pose, node.last_transformed_stamp = ([0, 0, 0], [0, 0, 0, 1]), 1_000_000_000
        node.evidence.add(np.array([[1., 0, 0, 0, 30]]), np.array([0]), 1_000_000_000,
                          1_000_000_000)
        # 1.2 m at 12 m/s over 0.1 s used to exceed the fixed 1 m step every scan.
        node.check_pose_step(([1.2, 0, 0], [0, 0, 0, 1]), 1_100_000_000)
        assert node.pose_resets == 0 and node.evidence.observations == 1
        node.check_pose_step(([30.0, 0, 0], [0, 0, 0, 1]), 1_100_000_000)
        assert node.pose_resets == 1 and node.evidence.observations == 0
    finally:
        node.destroy_node()


def test_accumulation_ignores_empty_clouds_for_stamps_and_watchdog(ros):
    ros('mode:=stationary_preview')
    node = RadarAccumulation()
    try:
        now = node.get_clock().now().nanoseconds
        # An empty clear stamped later (e.g. the processing node's now()) ...
        node.receive(empty_cloud(Header(frame_id='umrr96', stamp=stamp(now + 40_000_000))))
        assert node.last_stamp is None and node.last_receipt is None
        assert node.empty_inputs == 1 and node.dropped['nonmonotonic_stamp'] == 0
        # ... must not make the next, older-stamped real scan non-monotonic.
        node.receive(cloud(now - 10_000_000, [[2, 0, 0, 0, 30]]))
        assert node.processed == 1 and node.dropped['nonmonotonic_stamp'] == 0
        receipt = node.last_receipt
        node.receive(empty_cloud(Header(frame_id='umrr96', stamp=stamp(now))))
        assert node.last_receipt == receipt  # Empty clouds do not hide upstream staleness.
    finally:
        node.destroy_node()


def test_accumulation_output_stamp_is_newest_contributing_scan(ros):
    ros('mode:=stationary_preview')
    node = RadarAccumulation()
    recorder, confirmed = Recorder(), Recorder()
    try:
        node.evidence_pub, node.confirmed_pub = recorder, confirmed
        node.published_data = {recorder: None, confirmed: None}
        node.empty_heartbeat = {publisher: DiagnosticsRateLimiter()
                                for publisher in (recorder, confirmed)}
        now = node.get_clock().now().nanoseconds
        first, second = now - 60_000_000, now - 30_000_000
        node.receive(cloud(first, [[2, 0, 0, 0, 30]]))
        node.receive(cloud(second, [[5, 0, 0, 0, 30]]))
        node.publish()
        header = recorder.messages[-1].header
        assert Time.from_msg(header.stamp).nanoseconds == second
        assert recorder.messages[-1].width == 2 and confirmed.messages[-1].width == 0
        # Unchanged empty outputs are not republished every cycle.
        count = len(confirmed.messages)
        node.publish()
        node.publish()
        assert len(confirmed.messages) == count
    finally:
        node.destroy_node()


def test_processing_clears_once_with_last_accepted_stamp(ros):
    ros()
    node = RadarProcessing()
    outputs = {name: Recorder() for name in node.cloud_publishers}
    diagnostics = Recorder()
    try:
        node.cloud_publishers = outputs
        node.diagnostics_pub = diagnostics
        now = node.get_clock().now().nanoseconds
        accepted = now - 20_000_000
        node.receive(cloud(accepted, [[2, 1, 0, 0, 30]] * 3))
        assert outputs['quality_targets'].messages[-1].width == 3
        for _ in range(3):
            node.receive(cloud(now, [[2, 1, 0, 0, 30]], frame='other'))
        assert node.rejected_inputs == 3
        clears = [m for m in outputs['quality_targets'].messages if m.width == 0]
        assert len(clears) == 1  # Cleared on the transition only, not per rejection.
        assert Time.from_msg(clears[0].header.stamp).nanoseconds == accepted
        # State changes publish immediately; repeats are rate limited.
        states = [{v.key: v.value for v in m.status[0].values}['state']
                  for m in diagnostics.messages]
        assert states == ['insufficient_points', 'unexpected_frame']
        node.diagnostics_limiter.last_time = time.monotonic() - 2
        node.watchdog()
        assert len(diagnostics.messages) == 3
    finally:
        node.destroy_node()
