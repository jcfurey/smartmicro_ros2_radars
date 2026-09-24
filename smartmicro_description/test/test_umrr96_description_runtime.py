# SPDX-License-Identifier: Apache-2.0
"""Exercise the installed launch, latched description and static TF in isolation."""
import math
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


@pytest.mark.parametrize('sim_time', ['false', 'true'])
def test_late_subscriber_gets_model_and_correct_transform_without_joint_states(tmp_path, sim_time):
    rclpy.init()
    node = rclpy.create_node('umrr96_description_test_' + sim_time)
    log_path = tmp_path / 'launch.log'
    child = None
    try:
        with log_path.open('w') as log:
            child = subprocess.Popen([
                'ros2', 'launch', 'smartmicro_description', 'umrr96_description.launch.py',
                'namespace:=description_test', 'sensor_name:=test_radar',
                'frame_id:=test_radar_data', 'measurement_xyz:=0.01 -0.02 0.03',
                'measurement_rpy:=0 0 1.5707963267948966', 'use_sim_time:=' + sim_time,
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            topic = '/description_test/test_radar/robot_description'
            deadline = time.monotonic() + 12
            while node.count_publishers(topic) == 0 and time.monotonic() < deadline:
                assert child.poll() is None, log_path.read_text()
                rclpy.spin_once(node, timeout_sec=.05)
            assert node.count_publishers(topic) == 1, log_path.read_text()
            # Deliberately attach after startup; both topics must be latched.
            wait_until = time.monotonic() + .3
            while time.monotonic() < wait_until:
                rclpy.spin_once(node, timeout_sec=.05)
            models = []
            node.create_subscription(String, topic, models.append,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if models and buffer.can_transform('test_radar_link', 'test_radar_data', Time()):
                    break
            assert models, log_path.read_text()
            root = ET.fromstring(models[-1].data)
            assert {link.get('name') for link in root.findall('link')} == {
                'test_radar_link', 'test_radar_data'}
            transform = buffer.lookup_transform('test_radar_link', 'test_radar_data', Time())
            t, q = transform.transform.translation, transform.transform.rotation
            assert [t.x, t.y, t.z] == pytest.approx([.01, -.02, .03])
            assert [q.x, q.y, q.z, q.w] == pytest.approx([0, 0, math.sqrt(.5), math.sqrt(.5)])
            # A +X data point must land at +Y of the housing after a 90-degree yaw.
            point = [t.x + 1 - 2*(q.y*q.y + q.z*q.z),
                     t.y + 2*(q.x*q.y + q.z*q.w),
                     t.z + 2*(q.x*q.z - q.y*q.w)]
            assert point == pytest.approx([.01, .98, .03])
            assert node.count_publishers('/joint_states') == 0
            listener.unregister()
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
