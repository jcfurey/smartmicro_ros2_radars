# SPDX-License-Identifier: Apache-2.0
"""In-process node checks: parameters, empty clouds, pose steps, stamps and diagnostics."""
from itertools import product
import math
from pathlib import Path
import time
from types import SimpleNamespace

from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from smartmicro_processing import (accumulation_node, node as processing_node, radar_tracks,
                                   ros_support)
from smartmicro_processing.accumulation import pose_discontinuity, pose_step_limits
from smartmicro_processing.accumulation_node import RadarAccumulation
from smartmicro_processing.cloud import empty_cloud
from smartmicro_processing.node import RadarProcessing, TRACK_FIELDS
from smartmicro_processing.ros_support import as_float, DiagnosticsRateLimiter
from std_msgs.msg import Header
from umrr_ros2_msgs.msg import DetectionAudit
import yaml

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
    """Publisher stand-in with ``subscribers`` subscribers (one by default)."""

    def __init__(self, subscribers=1):
        self.messages = []
        self.subscribers = subscribers

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscribers


@pytest.mark.parametrize('node_type,name', [(RadarProcessing, 'umrr96_processing.yaml'),
                                            (RadarAccumulation, 'umrr96_accumulation.yaml')])
def test_python_defaults_equal_the_shipped_yaml(ros, node_type, name):
    # The audit CLIs use the Python defaults: they must replay the deployed settings.
    with open(Path(__file__).parents[1] / 'config' / name) as stream:
        (section,) = yaml.safe_load(stream).values()
    ros()
    node = node_type()
    try:
        for key, value in section['ros__parameters'].items():
            assert node.has_parameter(key), key
            default = node.get_parameter(key).value
            assert type(default) is type(value) and default == value, (key, default, value)
    finally:
        node.destroy_node()


@pytest.mark.parametrize('node_type,topic', [
    (RadarProcessing, '/umrr96_processing/obstacles'),
    (RadarAccumulation, '/umrr96_accumulation/accumulated_targets')])
def test_publishers_are_reliable_by_default_and_overridable(ros, node_type, topic):
    # Best-effort outputs left a default (reliable) subscriber with nothing but an
    # incompatible-QoS warning, and qos_overrides were ignored.
    ros()
    node = node_type()
    try:
        publishers = [p for p in node.publishers if p.topic_name != '/parameter_events']
        assert topic in [p.topic_name for p in publishers] and len(publishers) >= 3
        for publisher in publishers:
            qos = publisher.qos_profile
            assert qos.reliability == QoSReliabilityPolicy.RELIABLE, publisher.topic_name
            assert qos.durability == QoSDurabilityPolicy.VOLATILE
            assert qos.history == QoSHistoryPolicy.KEEP_LAST and qos.depth >= 5
            prefix = f'qos_overrides.{publisher.topic_name}.publisher.'
            for policy in ('reliability', 'history', 'depth'):
                assert node.has_parameter(prefix + policy), prefix + policy
    finally:
        node.destroy_node()
    rclpy.try_shutdown()
    ros(f'qos_overrides.{topic}.publisher.reliability:=best_effort',
        f'qos_overrides.{topic}.publisher.depth:=2')
    node = node_type()
    try:
        by_topic = {p.topic_name: p.qos_profile for p in node.publishers}
        assert by_topic[topic].reliability == QoSReliabilityPolicy.BEST_EFFORT
        assert by_topic[topic].depth == 2
        others = [qos for name, qos in by_topic.items()
                  if name not in (topic, '/parameter_events')]
        assert all(qos.reliability == QoSReliabilityPolicy.RELIABLE for qos in others)
    finally:
        node.destroy_node()


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


@pytest.mark.parametrize('evidence,standing,joint', product((False, True), repeat=3))
def test_tracker_options_use_only_current_scan_positions(ros, evidence, standing, joint):
    options = {'evidence_confirmation': evidence, 'standing_support': standing,
               'joint_association': joint}
    ros(*(key + ':=' + str(value).lower() for key, value in options.items()))
    node = RadarProcessing()
    try:
        for key, value in options.items():
            assert getattr(node.tracker.config, key) == value
        node.diagnostics_pub = Recorder()
        node.cloud_publishers = {name: Recorder() for name in node.cloud_publishers}
        node.audit_pub = Recorder()
        static = np.array([[3, 0, 0], [2, 1, 0], [2, -1, 0], [2, 0, 1], [2, 0, -1],
                           [3, 1, 1], [3, -1, -1], [2, 1, -1], [2, -1, 1]])
        rows = np.column_stack((static, np.zeros(9), np.full(9, 30)))
        start = node.get_clock().now().nanoseconds
        for k in range(6 if evidence else 8):
            now = start + k * 55_000_000
            node.now_ns = lambda: now
            node.receive(cloud(now, np.vstack((rows, [1. + .5*k*.055, 0., 0., .5, 30.]))))
        assert node.stats['tracks'] == 1
        assert node.cloud_publishers['tracked_targets'].messages[-1].width == 1
        now += 55_000_000
        node.receive(cloud(now, rows))
        assert node.cloud_publishers['classified_targets'].messages[-1].width == 9
        assert node.cloud_publishers['tracked_targets'].messages[-1].width == 0
        diagnostics = {v.key: v.value
                       for v in node.diagnostics_pub.messages[-1].status[0].values}
        for key, value in options.items():
            assert diagnostics[key] == str(value)
    finally:
        node.destroy_node()


def test_shadow_half_angle_parameter_reaches_the_obstacle_filter(ros):
    ros()
    node = RadarProcessing()
    try:
        assert node.obstacle_config.shadow_half_angle_deg == 15.0
        descriptor = node.describe_parameter('shadow_half_angle_deg')
        assert descriptor.read_only and descriptor.floating_point_range[0].to_value == 180
    finally:
        node.destroy_node()
    rclpy.try_shutdown()
    ros('shadow_half_angle_deg:=180')
    node = RadarProcessing()
    try:
        assert node.obstacle_config.shadow_half_angle_deg == 180.0
    finally:
        node.destroy_node()
    rclpy.try_shutdown()
    ros('shadow_half_angle_deg:=0')
    with pytest.raises(ValueError):
        RadarProcessing()


@pytest.mark.parametrize('node_type', [RadarProcessing, RadarAccumulation])
def test_hardware_id_defaults_to_the_frame_and_can_name_the_radar(ros, node_type):
    ros()
    node = node_type()
    try:
        assert node.hardware_id == 'umrr96'
    finally:
        node.destroy_node()
    rclpy.try_shutdown()
    ros('hardware_id:=umrr96_v1_2_2@192.168.11.11')
    node = node_type()
    try:
        node.diagnostics_pub = Recorder()
        if node_type is RadarProcessing:
            node.publish_diagnostics()
        else:
            node.publish_diagnostics(
                DiagnosticStatus.OK, 0, 0, node.get_clock().now().nanoseconds)
        status = node.diagnostics_pub.messages[-1].status[0]
        assert status.hardware_id == 'umrr96_v1_2_2@192.168.11.11'
    finally:
        node.destroy_node()


def test_track_ids_are_exact_uint32():
    field = next(f for f in TRACK_FIELDS if f.name == 'track_id')
    assert field.datatype == PointField.UINT32 and field.offset == 24
    rows = [(1., 2., 0., .1, .2, .3, ident, 1.) for ident in (2 ** 24, 2 ** 24 + 1, 2 ** 32 - 1)]
    message = create_cloud(Header(frame_id='umrr96'), TRACK_FIELDS, rows)
    assert message.point_step == 32
    assert read_points(message)['track_id'].tolist() == [2 ** 24, 2 ** 24 + 1, 2 ** 32 - 1]


def test_standing_support_parameters_reach_the_tracker(ros):
    ros('standing_support:=true', 'standing_hold:=30')
    node = RadarProcessing()
    try:
        assert node.tracker.config.standing_support
        assert node.tracker.config.standing_hold == 30.
        assert node.describe_parameter('standing_support').read_only
    finally:
        node.destroy_node()


def test_ghost_parameters_set_the_track_level_ghost_rule(ros):
    # The tracker kept its own 1.5 m / 0.25 m/s copy whatever the YAML said.
    ros()
    node = RadarProcessing()
    try:
        assert node.tracker.config.ghost_range_gap == node.ghost_config.range_gap == 1.5
        assert node.tracker.config.ghost_speed_tolerance == node.ghost_config.speed_tolerance
    finally:
        node.destroy_node()
    rclpy.try_shutdown()
    ros('range_gap:=2.5', 'speed_tolerance:=0.4')
    node = RadarProcessing()
    try:
        assert node.ghost_config.range_gap == 2.5 and node.ghost_config.speed_tolerance == .4
        assert node.tracker.config.ghost_range_gap == 2.5
        assert node.tracker.config.ghost_speed_tolerance == .4
        assert node.tracker.config is node.tracker_config
    finally:
        node.destroy_node()


def test_association_parameters_reach_the_tracker(ros):
    ros('joint_association:=true', 'association_uncertainty:=false', 'association_doppler:=true')
    node = RadarProcessing()
    try:
        assert node.tracker.config.joint_association
        assert not node.tracker.config.association_uncertainty
        assert node.tracker.config.association_doppler
        assert node.describe_parameter('joint_association').read_only
    finally:
        node.destroy_node()


@pytest.mark.parametrize('reject_static', [True, False])
def test_static_only_policy_agrees_across_clouds_audit_and_diagnostics(ros, reject_static):
    ros('reject_static_only:=' + str(reject_static).lower())
    node = RadarProcessing()
    try:
        node.cloud_publishers = {name: Recorder() for name in node.cloud_publishers}
        node.audit_pub = Recorder()
        static = np.array([[3, 0, 0], [2, 1, 0], [2, -1, 0], [2, 0, 1], [2, 0, -1],
                           [3, 1, 1], [3, -1, -1], [2, 1, -1], [2, -1, 1]])
        points = np.vstack((np.column_stack((static, np.zeros(9), np.full(9, 30))),
                            [6, 0, 0, 1, 30]))
        now = node.get_clock().now().nanoseconds
        message = cloud(now, points)
        original = bytes(message.data)
        node.receive(message)
        assert node.state == 'valid'
        assert bytes(message.data) == original
        assert node.cloud_publishers['moving_targets'].messages[-1].width == int(not reject_static)
        assert node.cloud_publishers['moving_ghosts'].messages[-1].width == int(reject_static)
        audit = node.audit_pub.messages[-1]
        assert audit.header == message.header
        assert audit.source_index[-1] == 9
        assert audit.reason_flags[-1] == DetectionAudit.GHOST_BEHIND_STATIC
        expected = DetectionAudit.SUSPECTED_GHOST if reject_static else DetectionAudit.MOVING
        assert audit.classification[-1] == expected
        assert node.stats['static_only_advisory'] == int(not reject_static)
        assert node.stats['reject_static_only'] == reject_static
        node.receive(cloud(now + 1_000_000, points[:-1]))
        assert node.cloud_publishers['moving_targets'].messages[-1].width == 0
        assert node.cloud_publishers['classified_targets'].messages[-1].width == 9
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
        # Empty outputs keep the evidence schema, so field-name readers never fail.
        assert confirmed.messages[-1].fields == recorder.messages[-1].fields
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
    audits = Recorder()
    try:
        node.cloud_publishers = outputs
        node.diagnostics_pub = diagnostics
        node.audit_pub = audits
        now = node.get_clock().now().nanoseconds
        accepted = now - 20_000_000
        node.receive(cloud(accepted, [[2, 1, 0, 0, 30]] * 3))
        assert outputs['quality_targets'].messages[-1].width == 3
        assert audits.messages[-1].event == DetectionAudit.SCAN
        for _ in range(3):
            node.receive(cloud(now, [[2, 1, 0, 0, 30]], frame='other'))
        assert node.rejected_inputs == 3
        clears = [m for m in outputs['quality_targets'].messages if m.width == 0]
        assert len(clears) == 1  # Cleared on the transition only, not per rejection.
        assert Time.from_msg(clears[0].header.stamp).nanoseconds == accepted
        assert len(audits.messages) == 2
        assert audits.messages[-1].event == DetectionAudit.CLEAR
        assert audits.messages[-1].status == 'unexpected_frame'
        assert audits.messages[-1].header == clears[0].header
        # State changes publish immediately; repeats are rate limited.
        states = [{v.key: v.value for v in m.status[0].values}['state']
                  for m in diagnostics.messages]
        assert states == ['insufficient_points', 'unexpected_frame']
        node.diagnostics_limiter.last_time = time.monotonic() - 2
        node.watchdog()
        assert len(diagnostics.messages) == 3
    finally:
        node.destroy_node()


def test_processing_clears_keep_each_output_schema(ros):
    ros()
    node = RadarProcessing()
    outputs = {name: Recorder() for name in node.cloud_publishers}
    try:
        node.cloud_publishers = outputs
        node.track_pub, node.obstacle_pub = Recorder(), Recorder()
        node.audit_pub, node.diagnostics_pub = Recorder(), Recorder()
        fields = FIELDS + [PointField(name='rcs', offset=20, datatype=PointField.FLOAT32,
                                      count=1)]
        now = node.get_clock().now().nanoseconds
        node.receive(create_cloud(Header(frame_id='umrr96', stamp=stamp(now - 20_000_000)),
                                  fields, np.array([[2, 1, 0, 0, 30, 5]] * 3, np.float32)))
        node.receive(cloud(now, [[2, 1, 0, 0, 30]], frame='other'))  # rejected: clears
        tracks, obstacles = node.track_pub.messages, node.obstacle_pub.messages
        assert [m.width for m in tracks] == [0, 0] and len(obstacles) == 2
        assert tracks[-1].fields == tracks[0].fields == TRACK_FIELDS
        assert obstacles[-1].fields == obstacles[0].fields
        assert [f.name for f in obstacles[-1].fields] == ['x', 'y', 'z']
        assert not len(read_points(tracks[-1], field_names=['x', 'y', 'vx', 'track_id']))
        for name, recorder in outputs.items():
            messages = recorder.messages
            if name != 'classified_targets':
                assert messages[-1].width == 0 and messages[-1].fields == fields
                assert messages[-1].point_step == 24
                assert messages[0].fields == fields and messages[0].point_step == 24
    finally:
        node.destroy_node()


@pytest.fixture
def steady(monkeypatch):
    """Replace the steady clock used by the nodes and the diagnostics rate limiter."""
    clock = SimpleNamespace(now=1000.0)
    fake = SimpleNamespace(monotonic=lambda: clock.now)
    for module in (processing_node, accumulation_node, ros_support):
        monkeypatch.setattr(module, 'time', fake)
    return clock


def diagnostic_states(recorder):
    return [{v.key: v.value for v in m.status[0].values}['state'] for m in recorder.messages]


def test_processing_rejections_keep_their_state_without_flooding_diagnostics(ros, steady):
    ros()
    node = recorded_node()
    try:
        ros_time = node.get_clock().now().nanoseconds
        node.now_ns = lambda: ros_time
        node.receive(moving_sensor_cloud(ros_time - 20_000_000, np.zeros(3)))
        assert diagnostic_states(node.diagnostics_pub) == ['valid']
        # 3 s of wrong-frame scans at 18 Hz while the 10 Hz watchdog keeps running:
        # the accepted stamp ages past max_input_age but input is not silent.
        for tick in range(1, 541):
            steady.now += 1 / 180
            ros_time += 1_000_000_000 // 180
            if tick % 10 == 0:
                node.receive(cloud(ros_time, [[2, 1, 0, 0, 30]], frame='other'))
            if tick % 18 == 0:
                node.watchdog()
        states = diagnostic_states(node.diagnostics_pub)
        assert set(states[1:]) == {'unexpected_frame'} and node.state == 'unexpected_frame'
        assert len(states) <= 1 + 1 + 3  # the change, then at most one per second
        steady.now += 1.0  # input silent: the watchdog now reports staleness
        node.watchdog()
        assert node.state == 'input_stale'
    finally:
        node.destroy_node()


def test_accumulation_rejections_keep_their_state_without_flooding_diagnostics(ros, steady):
    ros('mode:=stationary_preview')
    node = RadarAccumulation()
    evidence, confirmed = Recorder(), Recorder()
    try:
        node.evidence_pub, node.confirmed_pub = evidence, confirmed
        node.published_data = {evidence: None, confirmed: None}
        node.empty_heartbeat = {publisher: DiagnosticsRateLimiter()
                                for publisher in (evidence, confirmed)}
        node.diagnostics_pub = Recorder()
        ros_time = node.get_clock().now().nanoseconds
        node.now_ns = lambda: ros_time
        node.receive(cloud(ros_time - 10_000_000, [[2, 0, 0, 0, 30]]))
        node.publish()
        assert diagnostic_states(node.diagnostics_pub) == ['stationary_preview']
        for tick in range(1, 541):
            steady.now += 1 / 180
            ros_time += 1_000_000_000 // 180
            if tick % 10 == 0:
                node.receive(cloud(ros_time, [[2, 0, 0, 0, 30]], frame='other'))
            if tick % 18 == 0:
                node.publish()
        states = diagnostic_states(node.diagnostics_pub)
        assert set(states[1:]) == {'unexpected_frame'} and node.state == 'unexpected_frame'
        assert len(states) <= 1 + 1 + 3
        assert not node.evidence.frames  # history still expires without accepted scans
        steady.now += 1.0
        node.publish()
        assert node.state == 'input_stale'
    finally:
        node.destroy_node()


def moving_sensor_cloud(ns, velocity):
    """Generate a static scene seen from a moving sensor (positive-receding Doppler)."""
    points = []
    for azimuth in np.radians([-50, -35, -20, -8, 5, 15, 28, 40, 55, 65]):
        for z in (-0.6, 0.0, 0.6):
            xyz = np.array([4 * math.cos(azimuth), 4 * math.sin(azimuth), z])
            points.append([*xyz, -xyz @ velocity / np.linalg.norm(xyz), 30])
    return cloud(ns, points)


def test_processing_keeps_tracker_across_rejects_and_coasts_on_failed_fit(ros):
    ros()
    node = RadarProcessing()
    tracks = Recorder()
    try:
        node.track_pub = tracks
        tracker = node.tracker
        for t in (0.0, 5.0):
            tracker.background.update(t, np.array([[4.0, 0.0]]))
        now = node.get_clock().now().nanoseconds
        node.receive(cloud(now - 20_000_000, [[2, 1, 0, 0, 30]] * 3))  # too few: failed fit
        assert node.state == 'insufficient_points' and len(tracks.messages) == 1
        node.receive(cloud(now, [[2, 1, 0, 0, 30]], frame='other'))  # rejected
        # One rejected scan clears the outputs but keeps the learned background.
        assert node.tracker is tracker and tracker.background.ready
    finally:
        node.destroy_node()


def test_processing_moving_sensor_disables_background(ros):
    ros()
    node = RadarProcessing()
    try:
        for t in (0.0, 5.0):
            node.tracker.background.update(t, np.array([[4.0, 0.0]]))
        now = node.get_clock().now().nanoseconds
        for k in range(3):
            node.receive(moving_sensor_cloud(now - (30 - 10 * k) * 1_000_000,
                                             np.array([0.5, 0.0, 0.0])))
            assert node.state == 'valid'
        assert node.sensor_moving and node.stats['sensor_moving']
        assert not node.tracker.background.ready and node.tracker.static_novel is None
    finally:
        node.destroy_node()


def radar_mount(pitch_deg=5.0, height=.5):
    """Return base_link <- umrr96: 0.2 m forward, ``height`` up, pitched down."""
    mount = TransformStamped()
    mount.header.frame_id, mount.child_frame_id = 'base_link', 'umrr96'
    mount.transform.translation.x, mount.transform.translation.z = .2, height
    half = math.radians(pitch_deg) / 2
    mount.transform.rotation.y, mount.transform.rotation.w = math.sin(half), math.cos(half)
    return mount


def recorded_node():
    node = RadarProcessing()
    node.cloud_publishers = {name: Recorder() for name in node.cloud_publishers}
    for name in ('track_pub', 'obstacle_pub', 'audit_pub', 'diagnostics_pub'):
        setattr(node, name, Recorder())
    return node


def test_subset_clouds_are_built_only_with_subscribers(ros):
    # All eight subset/display clouds used to be built and published on every scan.
    ros()
    node = recorded_node()
    try:
        subscribed = {'doppler_inliers', 'classified_targets'}
        node.cloud_publishers = {name: Recorder(int(name in subscribed))
                                 for name in node.cloud_publishers}
        node.velocity_pub = Recorder()
        outputs = node.cloud_publishers
        now = node.get_clock().now().nanoseconds
        for k in range(3):
            node.receive(moving_sensor_cloud(now - (40 - 10 * k) * 1_000_000, np.zeros(3)))
        assert {name for name, r in outputs.items() if r.messages} == subscribed
        assert [m.width for m in outputs['doppler_inliers'].messages] == [30] * 3
        assert outputs['classified_targets'].messages[-1].width == 30
        assert node.stats['displayed'] == 30  # counted without building the display
        for name in ('obstacle_pub', 'track_pub', 'audit_pub', 'velocity_pub'):
            assert len(getattr(node, name).messages) == 3, name
        node.receive(cloud(now, [[2, 1, 0, 0, 30]], frame='other'))  # rejected: clears
        assert {name for name, r in outputs.items() if r.messages} == subscribed
        for name in subscribed:
            assert len(outputs[name].messages) == 4 and outputs[name].messages[-1].width == 0
        assert node.obstacle_pub.messages[-1].width == 0
        assert node.audit_pub.messages[-1].event == DetectionAudit.CLEAR
        # A late subscriber gets the next scan; the next clear reaches it too.
        outputs['moving_targets'].subscribers = 1
        node.receive(moving_sensor_cloud(now - 5_000_000, np.zeros(3)))
        assert [m.width for m in outputs['moving_targets'].messages] == [0]
        outputs['doppler_inliers'].subscribers = 0
        node.receive(cloud(now, [[2, 1, 0, 0, 30]], frame='other'))
        assert [m.width for m in outputs['moving_targets'].messages] == [0, 0]
        # Unsubscribed since its last scan, but it carried data: cleared as well.
        assert [m.width for m in outputs['doppler_inliers'].messages][-2:] == [30, 0]
        assert not outputs['quality_targets'].messages
    finally:
        node.destroy_node()


@pytest.mark.parametrize('frame', ['', 'base_link'])
def test_obstacles_are_flattened_in_the_obstacle_frame(ros, frame):
    # A radar 0.5 m up pitched 5 deg down: flattening in the sensor frame put returns
    # beyond ~8 m below Nav2's min_obstacle_height in the robot's frame.
    ros(*(['obstacle_frame:=' + frame] if frame else []))  # default: the input frame
    node = recorded_node()
    try:
        if frame:
            node.tf_buffer.set_transform_static(radar_mount(), 'test')
        now = node.get_clock().now().nanoseconds
        for k in range(3):  # static persistence needs 3 of 5 scans
            message = moving_sensor_cloud(now - (30 - 10 * k) * 1_000_000, np.zeros(3))
            node.receive(message)
        obstacles = node.obstacle_pub.messages[-1]
        assert obstacles.header.stamp == message.header.stamp
        assert obstacles.header.frame_id == (frame or 'umrr96')
        out = read_points(obstacles)
        assert len(out) == 30 and np.all(out['z'] == np.float32(.3))
        source = read_points(message)
        pitch = math.radians(5.0) if frame else 0.
        expected_x = (np.cos(pitch) * source['x'] + np.sin(pitch) * source['z']
                      + (.2 if frame else 0.))
        np.testing.assert_allclose(np.sort(out['x']), np.sort(expected_x), atol=1e-5)
        np.testing.assert_allclose(np.sort(out['y']), np.sort(source['y']), atol=1e-5)
        assert node.stats['obstacles_published'] and node.obstacle_tf_failures == 0
    finally:
        node.destroy_node()


def test_obstacles_are_withheld_without_a_transform(ros):
    ros('obstacle_frame:=base_link')
    node = recorded_node()
    try:
        now = node.get_clock().now().nanoseconds
        node.receive(moving_sensor_cloud(now - 30_000_000, np.zeros(3)))
        # No silent identity fallback: nothing is published, the miss is counted.
        assert node.state == 'valid' and not node.obstacle_pub.messages
        assert node.track_pub.messages and node.cloud_publishers['quality_targets'].messages
        assert node.obstacle_tf_failures == 1 and not node.stats['obstacles_published']
        status = node.diagnostics_pub.messages[-1].status[0]
        values = {v.key: v.value for v in status.values}
        assert status.level == DiagnosticStatus.WARN and 'obstacles withheld' in status.message
        assert values['obstacle_tf_failures'] == '1' and values['obstacle_frame'] == 'base_link'
        node.tf_buffer.set_transform_static(radar_mount(), 'test')
        node.receive(moving_sensor_cloud(now - 20_000_000, np.zeros(3)))
        assert node.obstacle_pub.messages[-1].header.frame_id == 'base_link'
        assert node.diagnostics_pub.messages[-1].status[0].level == DiagnosticStatus.OK
        node.tf_buffer.set_transform_static(radar_mount(pitch_deg=10.), 'test')
        node.receive(moving_sensor_cloud(now - 10_000_000, np.zeros(3)))
        assert node.obstacle_tf_changes == 1  # not rigidly attached: warned and counted
        node.receive(cloud(now, [[2, 1, 0, 0, 30]], frame='other'))  # clear
        clear = node.obstacle_pub.messages[-1]
        assert clear.width == 0 and clear.header.frame_id == 'base_link'
    finally:
        node.destroy_node()


@pytest.mark.parametrize('gap,failed_fits,reset', [
    (.3, False, False), (8., False, True), (2., True, True)])
def test_processing_restarts_background_after_a_gap_between_valid_fits(ros, gap, failed_fits,
                                                                       reset):
    # The platform may have moved during a data gap or a run of failed fits; the old
    # scene used to stay "background" and the shadow rule dropped real new structure.
    ros()
    node = RadarProcessing()
    try:
        node.cloud_publishers = {name: Recorder() for name in node.cloud_publishers}
        for name in ('track_pub', 'obstacle_pub', 'audit_pub', 'diagnostics_pub'):
            setattr(node, name, Recorder())
        start = node.get_clock().now().nanoseconds

        def send(t, points=None):
            ns = start + round(t * 1e9)
            node.now_ns = lambda: ns + 5_000_000
            node.receive(moving_sensor_cloud(ns, np.zeros(3)) if points is None
                         else cloud(ns, points))

        t = 0.
        while t < 3.5:  # stationary: background learned after the 3 s warm-up
            send(t)
            t += .055
        assert node.tracker.background.ready and node.tracker.static_novel is not None
        end = t + gap
        while failed_fits and t < end:
            send(t, [[2, 1, 0, 0, 30]] * 3)
            assert node.state == 'insufficient_points'
            t += .055
        send(max(t, end))
        assert node.state == 'valid'
        assert node.tracker.background.ready != reset
        assert (node.tracker.static_novel is None) == reset
        assert node.background_gap_resets == int(reset)
    finally:
        node.destroy_node()


LANDMARKS = np.array([[14 * math.cos(a), 14 * math.sin(a), z]
                      for a in np.radians([-50, -35, -20, -8, 5, 15, 28, 40, 55, 65])
                      for z in (-.8, 0., .8)])
WALL = np.array([[x, 2., z] for x in np.arange(-5, 25, .25) for z in (-.6, 0., .6)])


def drive(node, sensor_velocity, person, person_velocity, scans, scene=LANDMARKS, leave=None):
    """
    Feed a world-fixed scene and a walking person (two returns) seen from a moving radar.

    Doppler is positive receding, relative to the radar. Returns per-scan rows of
    (true person xy in the sensor frame or None, published tracks, obstacle xy).
    """
    for name in ('track_pub', 'obstacle_pub', 'audit_pub', 'diagnostics_pub', 'velocity_pub'):
        setattr(node, name, Recorder())
    if node.radar_tracks_pub is not None:
        node.radar_tracks_pub = Recorder()
    node.cloud_publishers = {name: Recorder() for name in node.cloud_publishers}
    sensor_velocity = np.asarray(sensor_velocity, float)
    start = node.get_clock().now().nanoseconds
    rows = []
    for k in range(scans):
        t = k * .055
        ns = start + round(t * 1e9)
        node.now_ns = lambda ns=ns: ns + 5_000_000
        relative = scene - sensor_velocity * t
        relative = relative[relative[:, 0] > .3]
        bearing = relative / np.linalg.norm(relative, axis=1)[:, None]
        points = np.column_stack((relative, bearing @ -sensor_velocity,
                                  np.full(len(relative), 30)))
        truth = None
        if leave is None or k < leave:
            truth = np.asarray(person, float) + (np.asarray(person_velocity) - sensor_velocity) * t
            for offset in ((0, 0, 0), (.2, .1, 0)):
                xyz = truth + offset
                speed = xyz @ (np.asarray(person_velocity) - sensor_velocity) / np.linalg.norm(xyz)
                points = np.vstack((points, [*xyz, speed, 30]))
        node.receive(cloud(ns, points))
        assert node.state == 'valid'
        tracks = read_points(node.track_pub.messages[-1])
        obstacles = read_points(node.obstacle_pub.messages[-1])
        rows.append((None if truth is None else truth[:2], tracks,
                     np.column_stack((obstacles['x'], obstacles['y']))))
    return rows


@pytest.mark.parametrize('person,person_velocity,relative_vx', [
    ([10., 0., 0.], [-1., 0., 0.], -2.),  # approach: closing at 2 m/s
    ([4., .3, 0.], [1., 0., 0.], 0.)])  # follower: fixed in the sensor frame
def test_moving_sensor_tracks_people_in_the_sensor_frame(ros, person, person_velocity,
                                                         relative_vx):
    # Fed ego-compensated speeds, the EKF lagged 1.3-1.6 m behind the sensor-frame
    # position, split the track and dropped the person from obstacles.
    ros()
    node = RadarProcessing()
    try:
        rows = drive(node, [1., 0., 0.], person, person_velocity, 70)
        assert node.sensor_moving
        for truth, tracks, obstacles in rows[10:]:
            if np.linalg.norm(truth) < node.obstacle_config.safety_range:
                continue
            assert len(tracks) == 1
            assert math.hypot(tracks['x'][0] - truth[0], tracks['y'][0] - truth[1]) < .3
            assert tracks['vx'][0] == pytest.approx(relative_vx, abs=.2)
            assert np.min(np.linalg.norm(obstacles - truth, axis=1)) < .3
        assert node.tracker.next_id == 2  # one identity throughout
    finally:
        node.destroy_node()


def test_moving_sensor_wall_does_not_hold_a_departed_track(ros):
    ros()
    node = RadarProcessing()
    try:
        rows = drive(node, [1., 0., 0.], [7., 1.9, 0.], [-.5, 0., 0.], 140,
                     np.vstack((LANDMARKS, WALL)), leave=20)
        assert len(rows[19][1]) == 1  # confirmed before leaving
        alive = [k for k, (_, tracks, _) in enumerate(rows) if k >= 20 and len(tracks)]
        # At most max_coast, not static_hold (5 s) on novel wall returns.
        assert (max(alive) - 19) * .055 <= node.tracker_config.max_coast + 1e-9
    finally:
        node.destroy_node()


def test_radar_track_fields_uuid_and_covariance():
    pytest.importorskip('radar_msgs.msg')
    from radar_msgs.msg import RadarTrack
    P = np.array([[.04, .01, .002, 0.], [.01, .09, 0., .003],
                  [.002, 0., .5, .07], [0., .003, .07, .6]])
    track = SimpleNamespace(track_id=7, x=np.array([3., -1., .4, .2]), P=P)
    header = Header(frame_id='umrr96', stamp=stamp(5))
    message = radar_tracks.radar_tracks(header, [track], '/front/umrr96_processing', z=.5)
    (item,) = message.tracks
    assert message.header == header and item.classification == RadarTrack.DYNAMIC
    assert (item.position.x, item.position.y, item.position.z) == (3., -1., .5)
    assert (item.velocity.x, item.velocity.y, item.velocity.z) == (.4, .2, 0.)
    assert (item.acceleration.x, item.acceleration.y, item.acceleration.z) == (0., 0., 0.)
    assert (item.size.x, item.size.y, item.size.z) == (0., 0., 0.)  # placeholder
    big = radar_tracks.UNOBSERVED_VARIANCE
    np.testing.assert_allclose(item.position_covariance, [.04, .01, 0, .09, 0, big], rtol=1e-6)
    np.testing.assert_allclose(item.velocity_covariance, [.5, .07, 0, .6, 0, big], rtol=1e-6)
    for unknown in (item.acceleration_covariance, item.size_covariance):
        np.testing.assert_allclose(unknown, [big, 0, 0, big, 0, big])
    same = radar_tracks.track_uuid('/front/umrr96_processing', 7)
    assert bytes(item.uuid.uuid) == same.bytes and same.version == 5
    assert radar_tracks.track_uuid('/rear/umrr96_processing', 7) != same
    assert radar_tracks.track_uuid('/front/umrr96_processing', 8) != same


def test_radar_tracks_mirror_tracked_objects(ros):
    pytest.importorskip('radar_msgs.msg')
    ros()
    node = RadarProcessing()
    try:
        assert node.radar_tracks_pub is not None
        drive(node, [0., 0., 0.], [6., 1., 0.], [-.8, .3, 0.], 40)
        typed, clouds = node.radar_tracks_pub.messages, node.track_pub.messages
        assert len(typed) == len(clouds) == 40
        uuids = set()
        for message, cloud_message in zip(typed, clouds):
            rows = read_points(cloud_message)
            assert message.header == cloud_message.header
            assert len(message.tracks) == len(rows)
            for item, row in zip(message.tracks, rows):
                assert (item.position.x, item.position.y) == pytest.approx(
                    (row['x'], row['y']), abs=1e-5)
                assert item.position.z == row['z'] == 0.
                assert (item.velocity.x, item.velocity.y) == pytest.approx(
                    (row['vx'], row['vy']), abs=1e-5)
                assert bytes(item.uuid.uuid) == radar_tracks.track_uuid(
                    '/umrr96_processing', row['track_id']).bytes
                uuids.add(bytes(item.uuid.uuid))
        assert len(uuids) == 1 and len(typed[-1].tracks) == 1  # one stable identity
        (track,) = [t for t in node.tracker.tracks if t.confirmed]
        np.testing.assert_allclose(typed[-1].tracks[0].position_covariance[[0, 1, 3]],
                                   track.P[[0, 0, 1], [0, 1, 1]], rtol=1e-6)
        np.testing.assert_allclose(typed[-1].tracks[0].velocity_covariance[[0, 1, 3]],
                                   track.P[[2, 2, 3], [2, 3, 3]], rtol=1e-6)
        node.receive(cloud(node.last_stamp + 1, [[2, 1, 0, 0, 30]], frame='other'))  # clear
        assert typed[-1].tracks == [] and typed[-1].header.frame_id == 'umrr96'
    finally:
        node.destroy_node()


def test_typed_tracks_are_skipped_without_radar_msgs(ros, monkeypatch):
    monkeypatch.setattr(radar_tracks, 'RadarTracks', None)
    ros()
    node = recorded_node()
    try:
        assert node.radar_tracks_pub is None
        assert not any(p.topic_name.endswith('/tracks') for p in node.publishers)
        now = node.get_clock().now().nanoseconds
        node.receive(moving_sensor_cloud(now - 20_000_000, np.zeros(3)))
        assert node.state == 'valid' and node.track_pub.messages
        values = {v.key: v.value for v in node.diagnostics_pub.messages[-1].status[0].values}
        assert values['typed_tracks'] == 'False'
    finally:
        node.destroy_node()


def test_track_ids_continue_across_a_clock_reset(ros):
    ros()
    node = RadarProcessing()
    try:
        node.tracker.next_id = 42
        node.last_now = node.get_clock().now().nanoseconds + 10 ** 12
        node.now_ns()  # the ROS clock went backwards
        assert node.state == 'clock_reset' and node.tracker.next_id == 42
        assert not node.tracker.tracks
    finally:
        node.destroy_node()


def planar_rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0],
                     [0, 0, 1]])


def odom_transform(ns, position, yaw):
    """Return odom <- umrr96 at ``ns``: the radar at ``position`` heading ``yaw``."""
    message = TransformStamped()
    message.header.frame_id, message.child_frame_id = 'odom', 'umrr96'
    message.header.stamp = stamp(ns)
    translation = message.transform.translation
    translation.x, translation.y, translation.z = (float(v) for v in position)
    message.transform.rotation.z = math.sin(yaw / 2)
    message.transform.rotation.w = math.cos(yaw / 2)
    return message


# Landmarks all around the start pose; only those in front of the radar are returned.
RING = np.array([[14 * math.cos(a), 14 * math.sin(a), z]
                 for a in np.radians(np.arange(0, 360, 12)) for z in (-.8, 0., .8)])


def drive_odom(node, person, person_velocity, scans, speed=1., yaw=.5, yaw_rate=0.,
               start_xy=(2., -1.), missing=()):
    """
    Feed a world-fixed scene and a walking person seen from a radar driving in odom.

    The radar starts at ``start_xy`` (0.5 m up) heading ``yaw`` and drives at
    ``speed`` along its heading, turning at ``yaw_rate``; odom <- umrr96 is in
    TF at every scan stamp except ``missing`` scans. ``person`` and
    ``person_velocity`` are given in the radar's start frame. Doppler is
    positive receding, relative to the radar. Returns per-scan rows of (true
    person xy in odom, true velocity in odom, published tracks, obstacle xy in
    the radar frame, true person xy in the radar frame).
    """
    for name in ('track_pub', 'obstacle_pub', 'audit_pub', 'diagnostics_pub', 'velocity_pub'):
        setattr(node, name, Recorder())
    node.cloud_publishers = {name: Recorder() for name in node.cloud_publishers}
    start = np.array([*start_xy, .5])
    initial = planar_rotation(yaw)
    scene = RING @ initial.T + start
    walker = initial @ np.asarray(person, float) + start
    walker_velocity = initial @ np.asarray(person_velocity, float)
    begin = node.get_clock().now().nanoseconds
    rows = []
    for k in range(scans):
        t = k * .055
        ns = begin + round(t * 1e9)
        node.now_ns = lambda ns=ns: ns + 5_000_000
        heading = yaw + yaw_rate * t
        if yaw_rate:
            position = start + speed / yaw_rate * np.array([
                math.sin(heading) - math.sin(yaw), math.cos(yaw) - math.cos(heading), 0.])
        else:
            position = start + speed * t * initial[:, 0]
        velocity = speed * planar_rotation(heading)[:, 0]
        rotation = planar_rotation(heading)  # odom <- umrr96
        if k not in missing:
            node.tf_buffer.set_transform(odom_transform(ns, position, heading), 'test')
        local = (scene - position) @ rotation
        local = local[local[:, 0] > .3]
        bearing = local / np.linalg.norm(local, axis=1)[:, None]
        points = np.column_stack((local, bearing @ (rotation.T @ -velocity),
                                  np.full(len(local), 30)))
        truth = walker + walker_velocity * t
        for offset in ((0, 0, 0), (.2, .1, 0)):
            xyz = (truth + offset - position) @ rotation
            relative = rotation.T @ (walker_velocity - velocity)
            points = np.vstack((points, [*xyz, xyz @ relative / np.linalg.norm(xyz), 30]))
        node.receive(cloud(ns, points))
        assert node.state == 'valid'
        tracks = read_points(node.track_pub.messages[-1])
        obstacles = read_points(node.obstacle_pub.messages[-1])
        rows.append((truth[:2], walker_velocity[:2], tracks,
                     np.column_stack((obstacles['x'], obstacles['y'])),
                     ((truth - position) @ rotation)[:2]))
    return rows


@pytest.mark.parametrize('person,person_velocity,yaw_rate', [
    ([10., 0., 0.], [-1., 0., 0.], 0.),  # approach: closing at 2 m/s
    ([4., .3, 0.], [1., 0., 0.], 0.),  # follower: fixed in the sensor frame
    ([8., 1., 0.], [-.6, -.4, 0.], .15)])  # turning radar, crossing walker
def test_tracking_frame_gives_ground_tracks_on_a_moving_radar(ros, person, person_velocity,
                                                              yaw_rate):
    ros('tracking_frame:=odom')
    node = RadarProcessing()
    try:
        rows = drive_odom(node, person, person_velocity, 70, yaw_rate=yaw_rate)
        assert node.sensor_moving and node.tracking_tf_failures == 0
        assert node.track_pub.messages[-1].header.frame_id == 'odom'
        assert node.obstacle_pub.messages[-1].header.frame_id == 'umrr96'
        for truth, velocity, tracks, obstacles, local in rows[10:]:
            assert len(tracks) == 1
            assert math.hypot(tracks['x'][0] - truth[0], tracks['y'][0] - truth[1]) < .3
            assert tracks['z'][0] == pytest.approx(.5)  # the radar's height in odom
            assert (tracks['vx'][0], tracks['vy'][0]) == pytest.approx(tuple(velocity), abs=.2)
            if np.linalg.norm(local) >= node.obstacle_config.safety_range:
                assert np.min(np.linalg.norm(obstacles - local, axis=1)) < .3
        assert node.tracker.next_id == 2  # one identity throughout
        node.receive(cloud(node.last_stamp + 1, [[2, 1, 0, 0, 30]], frame='other'))  # clear
        assert node.track_pub.messages[-1].header.frame_id == 'odom'
        assert node.track_pub.messages[-1].width == 0
    finally:
        node.destroy_node()


def test_tracking_frame_coasts_and_counts_scans_without_transform(ros):
    ros('tracking_frame:=odom', 'tf_wait_seconds:=0')
    node = RadarProcessing()
    try:
        missing = range(30, 34)
        rows = drive_odom(node, [10., 0., 0.], [-1., 0., 0.], 50, missing=missing)
        assert node.tracking_tf_failures == len(missing)
        for k, (truth, velocity, tracks, obstacles, local) in enumerate(rows[10:], 10):
            assert len(tracks) == 1  # coasted through the gap, same identity
            error = math.hypot(tracks['x'][0] - truth[0], tracks['y'][0] - truth[1])
            assert error < (.4 if k in missing else .3)
            if k in missing:  # track position unknown in the radar frame: movers pass
                assert np.min(np.linalg.norm(obstacles - local, axis=1)) < .3
        assert node.tracker.next_id == 2
        statuses = [m.status[0] for m in node.diagnostics_pub.messages]
        coasting = [s for s in statuses if 'tracker coasting' in s.message]
        assert coasting and all(s.level == DiagnosticStatus.WARN for s in coasting)
        values = {v.key: v.value for v in statuses[-1].values}
        assert values['tracking_tf_failures'] == '4' and values['tracking_frame'] == 'odom'
        assert values['track_velocity'] == 'ground' and values['tracking_tf_error'] == 'None'
        assert statuses[-1].level == DiagnosticStatus.OK
    finally:
        node.destroy_node()


def test_tracking_frame_scans_wait_for_their_transform_without_blocking(ros, steady):
    ros('tracking_frame:=odom')  # tf_wait_seconds 0.1
    node = recorded_node()
    try:
        now = node.get_clock().now().nanoseconds
        node.now_ns = lambda: now
        first, second = now - 20_000_000, now - 10_000_000
        node.receive(moving_sensor_cloud(first, np.zeros(3)))
        node.receive(moving_sensor_cloud(second, np.zeros(3)))
        assert len(node.pending) == 2 and not node.track_pub.messages  # waiting, not blocked
        node.tf_buffer.set_transform(odom_transform(first, (1., 2., .5), .3), 'test')
        node.process_pending()  # the timer: the first scan's TF has arrived
        assert len(node.pending) == 1 and len(node.track_pub.messages) == 1
        assert node.track_pub.messages[0].header.frame_id == 'odom'
        assert node.tracking_tf_failures == 0 and node.stats['tracking_tf_available']
        steady.now += .05
        node.process_pending()
        assert len(node.pending) == 1  # still within tf_wait_seconds
        steady.now += .1
        node.process_pending()  # expired: processed without TF, the tracker coasts
        assert not node.pending and len(node.track_pub.messages) == 2
        assert node.tracking_tf_failures == 1 and not node.stats['tracking_tf_available']
        assert 'ExtrapolationException' in node.tracking_tf_error
        stamps = [Time.from_msg(m.header.stamp).nanoseconds for m in node.obstacle_pub.messages]
        assert stamps == [first, second]  # in order, at their own stamps
    finally:
        node.destroy_node()


@pytest.mark.parametrize('overrides', [
    ('tracking_frame:=odom', 'standing_support:=true'),
    ('tracking_frame:=umrr96',), ('tracking_frame:=/odom',)])
def test_tracking_frame_rejects_invalid_combinations(ros, overrides):
    ros(*overrides)
    with pytest.raises(ValueError):
        RadarProcessing()
