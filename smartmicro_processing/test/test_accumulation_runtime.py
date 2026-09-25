# SPDX-License-Identifier: Apache-2.0
"""Installed ROS launch: delayed/exact TF, missing poses, resets and preview separation."""
import math
import os
import signal
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from std_msgs.msg import Header
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster


@pytest.mark.parametrize('mode', ['pose_compensated', 'stationary_preview'])
def test_installed_accumulator(tmp_path, mode):
    rclpy.init()
    node = rclpy.create_node('accumulation_test_controller')
    child = None
    log_path = tmp_path / 'launch.log'
    params_path = tmp_path / 'params.yaml'
    params_path.write_text('/**:\n  ros__parameters:\n'
                           '    max_pose_translation_step: 3.0\n'
                           '    max_pose_rotation_step: 2.0\n')
    publisher = node.create_publisher(PointCloud2, '/test_radar/inliers', qos_profile_sensor_data)
    clocks = node.create_publisher(Clock, '/clock', 10)
    broadcaster = TransformBroadcaster(node) if mode == 'pose_compensated' else None
    prefix = '/umrr96_accumulation/' + ('stationary_preview/' if broadcaster is None else '')
    clouds, confirmed, states = [], [], []
    node.create_subscription(PointCloud2, prefix + 'accumulated_targets', clouds.append,
                             qos_profile_sensor_data)
    node.create_subscription(PointCloud2, prefix + 'confirmed_targets', confirmed.append,
                             qos_profile_sensor_data)

    def diagnostic(msg):
        for status in msg.status:
            if status.name == '/umrr96_accumulation/evidence':
                states.append({v.key: v.value for v in status.values})

    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 10)

    def wait_for(predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert child.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(log_path.read_text() + '\n' + str(states[-3:]))

    def clock(seconds):
        stamp = Time(nanoseconds=round(seconds * 1e9)).to_msg()
        clocks.publish(Clock(clock=stamp))
        # Different DDS topics do not guarantee /clock is handled before a cloud.
        deadline = time.monotonic() + .02
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
        return stamp

    def transform(seconds, translation, yaw=0):
        msg = TransformStamped(header=Header(frame_id='test_odom',
                               stamp=Time(nanoseconds=round(seconds*1e9)).to_msg()),
                               child_frame_id='test_radar')
        t = msg.transform.translation
        t.x, t.y, t.z = map(float, translation)
        msg.transform.rotation.z, msg.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        broadcaster.sendTransform(msg)

    fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'radial_speed', 'snr',
                                     'false_alarm_probability'))]

    def scan(seconds, xyz):
        msg = create_cloud(Header(frame_id='test_radar', stamp=clock(seconds)), fields,
                           np.array([[*xyz, 0, 30, np.nan]]*20, dtype=np.float32))
        publisher.publish(msg)
        return msg

    def latest():
        return read_points(clouds[-1]) if clouds else []

    try:
        with log_path.open('w') as log:
            child = subprocess.Popen([
                'ros2', 'launch', 'smartmicro_processing', 'umrr96_accumulation.launch.py',
                'mode:=' + mode, 'params_file:=' + str(params_path),
                'input_topic:=/test_radar/inliers', 'expected_frame_id:=test_radar',
                'fixed_frame:=test_odom', 'use_sim_time:=true',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for(lambda: publisher.get_subscription_count() == 1
                     and clocks.get_subscription_count() >= 1 and clouds and states
                     and (broadcaster is None or node.count_subscribers('/tf') >= 1), 12)
            if broadcaster:
                clock(100)
                transform(99.9, [0, 0, 0])
                scan(100, [3.3, 2.2, .6])
                wait_for(lambda: states[-1]['state'] == 'waiting_for_tf')
                assert all(c.width == 0 for c in clouds)  # Never fall back to latest TF.
                # Bracketing poses give translation [1,0,0] at the scan stamp.
                transform(100.1, [2, 0, 0])
                wait_for(lambda: len(latest()) == 1)
                np.testing.assert_allclose([latest()[0][n] for n in ('x', 'y', 'z')],
                                           [4.3, 2.2, .6], atol=1e-6)
                scan(100.1, [2.3, 2.2, .6])
                wait_for(lambda: len(latest()) == 1 and latest()[0]['support_scans'] == 2)
                transform(100.2, [1, 1, 0], math.pi/2)
                last_scan = scan(100.2, [1.2, -3.3, .6])
                wait_for(lambda: len(latest()) == 1 and latest()[0]['support_scans'] == 3)
                np.testing.assert_allclose([latest()[0][n] for n in ('x', 'y', 'z')],
                                           [4.3, 2.2, .6], atol=1e-6)
                assert clouds[-1].header.frame_id == 'test_odom'
                assert states[-1]['motion_compensated'] == 'True'
                assert node.count_publishers('/tf') == 1  # Only this test's broadcaster.
            else:
                scan(100, [4.3, 2.2, .6])
                wait_for(lambda: len(latest()) == 1)
                assert latest()[0]['support_scans'] == 1
                last_scan = scan(100.1, [4.4, 2.2, .6])
                wait_for(lambda: len(latest()) == 1 and latest()[0]['support_scans'] == 2)
                assert latest()[0]['x'] == pytest.approx(4.4)
                assert clouds[-1].header.frame_id == 'test_radar'
                assert states[-1]['motion_compensated'] == 'False'
                assert node.count_publishers('/tf') == 0
                assert node.count_publishers('/umrr96_accumulation/accumulated_targets') == 0
            wait_for(lambda: confirmed and confirmed[-1].width == 1)
            before = len(states)
            publisher.publish(last_scan)
            wait_for(lambda: any(s.get('dropped_nonmonotonic_stamp') == '1'
                                 for s in states[before:]))
            assert latest()[0]['support_scans'] in (2, 3)
            if broadcaster:
                transform(100.3, [10, 0, 0])
                scan(100.3, [10.3, .2, .2])
                wait_for(lambda: states[-1]['pose_resets'] == '1')
                wait_for(lambda: len(latest()) == 1 and latest()[0]['x'] > 20)
                assert latest()[0]['support_scans'] == 1  # Old pose epoch removed.
                scan(100.4, [1, 1, 1])  # No pose at this time, only older transforms.
                wait_for(lambda: states[-1].get('dropped_tf_timeout') == '1')
                assert latest()[0]['x'] > 20
                clock(100.9)
                wait_for(lambda: clouds[-1].width == 0)
            # Reset service removes evidence immediately and starts a fresh history.
            reset = node.create_client(Empty, '/umrr96_accumulation/reset')
            wait_for(reset.service_is_ready)
            future = reset.call_async(Empty.Request())
            wait_for(future.done)
            wait_for(lambda: states[-1]['manual_resets'] == '1' and clouds[-1].width == 0)
            clock(10)
            wait_for(lambda: states[-1]['clock_resets'] == '1')
            if broadcaster:
                transform(10, [0, 0, 0])
            scan(10, [2.2, .2, .2])
            wait_for(lambda: len(latest()) == 1)
            assert latest()[0]['support_scans'] == 1
            # Freeze /clock and input: wall watchdog still expires both clouds.
            wait_for(lambda: states[-1]['state'] == 'input_stale')
            wait_for(lambda: clouds[-1].width == 0 and confirmed[-1].width == 0)
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
    assert 'Traceback' not in log_path.read_text(), log_path.read_text()
