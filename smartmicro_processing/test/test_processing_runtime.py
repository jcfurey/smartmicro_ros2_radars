# SPDX-License-Identifier: Apache-2.0
"""Real installed launch in isolated DDS; no connection to radar control services."""
import os
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud, read_points
from smartmicro_processing.experiment_launch import EXPERIMENTS
from std_msgs.msg import Header
from umrr_ros2_msgs.msg import DetectionAudit
import yaml


def velocity_std_floor():
    path = os.path.join(get_package_share_directory('smartmicro_processing'),
                        'config', 'umrr96_processing.yaml')
    with open(path) as stream:
        config = yaml.safe_load(stream)
    return config['/**/umrr96_processing']['ros__parameters']['velocity_std_floor']


@pytest.mark.parametrize('sim_time', [False, True])
def test_installed_processing_handles_motion_invalid_frames_disconnect_and_clock_reset(
        tmp_path, sim_time):
    rclpy.init()
    node = rclpy.create_node('processing_test_controller')
    child = None
    log_path = tmp_path / 'launch.log'
    outputs = {name: [] for name in ('quality_targets', 'doppler_inliers',
                                     'doppler_outliers', 'unclassified_targets',
                                     'classified_targets')}
    velocities, statuses, audits = [], [], []
    publisher = node.create_publisher(PointCloud2, '/test_radar/targets', qos_profile_sensor_data)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    # A default (reliable) subscriber must receive too, not only sensor-data ones.
    # Subset clouds are built only with subscribers: wait until each one is matched.
    subset_subscriptions = [node.create_subscription(
        PointCloud2, '/umrr96_processing/' + name, messages.append,
        10 if name == 'classified_targets' else qos_profile_sensor_data)
        for name, messages in outputs.items()]
    node.create_subscription(TwistWithCovarianceStamped,
                             '/umrr96_processing/experimental_velocity',
                             velocities.append, 10)
    node.create_subscription(DetectionAudit, '/umrr96_processing/detection_audit',
                             audits.append, qos_profile_sensor_data)

    def diagnostic(message):
        for status in message.status:
            if status.name == '/umrr96_processing/doppler':
                statuses.append({entry.key: entry.value for entry in status.values})

    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, 10)

    def wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert child.poll() is None, log_path.read_text()
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
        raise AssertionError(log_path.read_text() + '\nRecent diagnostics: ' + str(statuses[-3:]))

    simulated = 100.0

    def set_clock(seconds):
        msg = Clock()
        msg.clock.sec = int(seconds)
        msg.clock.nanosec = round((seconds - int(seconds)) * 1e9)
        clock_pub.publish(msg)
        deadline = time.monotonic() + .05
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.005)
        return msg.clock

    rng = np.random.default_rng(12)
    az, el = rng.uniform(-1.2, 1.2, 90), rng.uniform(-.35, .35, 90)
    directions = np.column_stack((np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)))
    truth = np.array([1.2, -.4, .25])
    speed = -directions @ truth
    speed[::5] += 3
    fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'radial_speed', 'snr',
                                        'false_alarm_probability'))]

    def cloud():
        nonlocal simulated
        simulated += .02
        stamp = set_clock(simulated) if sim_time else node.get_clock().now().to_msg()
        return create_cloud(Header(frame_id='test_sensor', stamp=stamp), fields,
                            np.column_stack((directions * 8, speed, np.full(90, 30),
                                             np.full(90, np.nan))).astype(np.float32))

    def send_expect(message, state):
        start = len(statuses)
        publisher.publish(message)
        wait_for(lambda: any(s['state'] == state for s in statuses[start:]))

    try:
        # The ordinary run checks shipped defaults. Replay checks YAML inheritance,
        # explicit enable and explicit disable through the installed launch file.
        experiment_args = []
        expected_options = dict.fromkeys(EXPERIMENTS, False)
        if sim_time:
            config_path = tmp_path / 'processing.yaml'
            with open(os.path.join(get_package_share_directory('smartmicro_processing'),
                                   'config', 'umrr96_processing.yaml')) as stream:
                config = yaml.safe_load(stream)
            config['/**/umrr96_processing']['ros__parameters'].update(
                standing_support=True, joint_association=True)
            config_path.write_text(yaml.safe_dump(config))
            experiment_args = ['params_file:=' + str(config_path),
                               'evidence_confirmation:=true', 'joint_association:=false']
            expected_options.update(evidence_confirmation=True, standing_support=True)
        with log_path.open('w') as log:
            child = subprocess.Popen([
                'ros2', 'launch', 'smartmicro_processing', 'umrr96_processing.launch.py',
                'input_topic:=/test_radar/targets', 'expected_frame_id:=test_sensor',
                'use_sim_time:=' + str(sim_time).lower(),
                *experiment_args,
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for(lambda: publisher.get_subscription_count() == 1
                     and node.count_publishers('/umrr96_processing/experimental_velocity') == 1
                     and all(s.get_publisher_count() for s in subset_subscriptions)
                     and (not sim_time or clock_pub.get_subscription_count() >= 1), 12)
            first = cloud()
            send_expect(first, 'valid')
            wait_for(lambda: velocities and audits
                     and all(messages for messages in outputs.values()))
            velocity = velocities[-1]
            assert velocity.header == first.header
            actual = velocity.twist.twist.linear
            np.testing.assert_allclose([actual.x, actual.y, actual.z], truth, atol=1e-5)
            cov = np.asarray(velocity.twist.covariance).reshape(6, 6)
            assert np.linalg.eigvalsh(cov[:3, :3]).min() >= .999 * velocity_std_floor() ** 2
            np.testing.assert_allclose(cov[3:, 3:], np.eye(3)*1e6)
            assert outputs['quality_targets'][-1].data == first.data
            assert outputs['doppler_inliers'][-1].width == 72
            assert outputs['doppler_outliers'][-1].width == 18
            assert outputs['unclassified_targets'][-1].width == 0
            first_audit = audits[-1]
            assert first_audit.header == first.header and first_audit.event == DetectionAudit.SCAN
            assert list(first_audit.source_index) == list(range(90))
            assert list(first_audit.classification).count(DetectionAudit.STATIC) == 72
            assert list(first_audit.classification).count(DetectionAudit.MOVING) == 18
            assert not any(first_audit.track_associated)
            shown = read_points(outputs['classified_targets'][-1])
            np.testing.assert_array_equal(shown['x'], read_points(first)['x'])
            assert statuses[-1]['calibrated'] == 'False'
            for key, value in expected_options.items():
                assert statuses[-1][key] == str(value)
            count = len(velocities)
            send_expect(first, 'nonmonotonic_stamp')
            malformed = cloud()
            malformed.fields = malformed.fields[1:]
            send_expect(malformed, 'invalid_cloud')
            wrong_frame = cloud()
            wrong_frame.header.frame_id = 'unrelated_sensor'
            send_expect(wrong_frame, 'unexpected_frame')
            old = cloud()
            old.header.stamp.sec -= 2
            send_expect(old, 'stale_stamp')
            future = cloud()
            future.header.stamp.sec += 2
            send_expect(future, 'future_stamp')
            flat = cloud()
            flat.data = create_cloud(flat.header, fields, np.column_stack((
                np.ones(90)*8, np.zeros((90, 2)), -np.ones(90), np.ones(90)*30,
                np.full(90, np.nan))).astype(np.float32)).data
            send_expect(flat, 'unobservable_geometry')
            wait_for(lambda: outputs['unclassified_targets'][-1].width == 90)
            wait_for(lambda: audits[-1].header == flat.header
                     and outputs['classified_targets'][-1].header == flat.header)
            assert list(audits[-1].classification) == [DetectionAudit.UNCLASSIFIED] * 90
            assert list(audits[-1].reason_flags) == [DetectionAudit.FIT_REJECTED] * 90
            # Latest scan replaces the earlier positions, including on failed fits.
            shown = read_points(outputs['classified_targets'][-1])
            assert len(shown) == 90 and np.all(shown['x'] == 8) and np.all(shown['y'] == 0)
            assert outputs['doppler_inliers'][-1].width == 0
            assert outputs['doppler_outliers'][-1].width == 0
            assert len(velocities) == count
            # Wall-clock watchdog must work even when /clock has stopped advancing.
            wait_for(lambda: statuses[-1]['state'] == 'input_stale')
            wait_for(lambda: all(messages[-1].width == 0 for messages in outputs.values()))
            wait_for(lambda: audits[-1].event == DetectionAudit.CLEAR)
            assert audits[-1].header == flat.header and audits[-1].status == 'input_stale'
            assert not len(audits[-1].source_index)
            assert len(velocities) == count
            if sim_time:
                simulated = 10.0  # Backward bag clock jump must permit a new epoch.
            recovery = cloud()
            send_expect(recovery, 'valid')
            wait_for(lambda: len(velocities) == count + 1)
            assert velocities[-1].header == recovery.header
    finally:
        if child is not None and child.poll() is None:
            # Let launch forward SIGINT once; signalling its whole process group
            # also interrupts the Python node a second time during teardown.
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
    assert 'Traceback' not in log_path.read_text(), log_path.read_text()
