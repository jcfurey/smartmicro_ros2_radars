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
from rclpy.time import Time
from sensor_msgs.msg import PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from smartmicro_processing import accumulation_node, node as processing_node, ros_support
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
    """Publisher stand-in with one subscriber."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


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
