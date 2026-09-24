#!/usr/bin/env python3
"""Check read/tuning validation, timeouts and cleanup without a connected radar.

Run after building and sourcing the workspace:
    ROS_DOMAIN_ID=174 python3 path/to/test/readback_smoke.py
"""

import json
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from umrr_ros2_msgs.srv import GetMode, GetStatus, SetMode


def main():
    """Use a silent UDP peer to exercise the installed node's failure paths."""
    executable = Path(get_package_prefix('umrr_ros2_driver')) / (
        'lib/umrr_ros2_driver/smartmicro_radar_readback_node')
    before = set(Path('/tmp').glob('smartmicro-readback-*'))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
        peer.bind(('127.0.0.1', 0))
        sensor_port = peer.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as available:
            available.bind(('127.0.0.1', 0))
            host_port = available.getsockname()[1]
        command = [
            str(executable), '--ros-args',
            '-r', '__node:=umrr96_readback_smoke_server',
            '-r', 'smart_radar/get_radar_mode:=umrr96_readback_smoke/mode',
            '-r', 'smart_radar/get_radar_status:=umrr96_readback_smoke/status',
            '-r', 'smart_radar/set_radar_mode:=umrr96_readback_smoke/set',
            '-p', 'sensor_id:=230739', '-p', 'interface_name:=lo',
            '-p', 'host_ip:=127.0.0.1', '-p', 'sensor_ip:=127.0.0.1',
            '-p', f'host_port:={host_port}', '-p', f'sensor_port:={sensor_port}',
            '-p', 'timeout_ms:=250',
        ]
        with tempfile.TemporaryFile(mode='w+') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            rclpy.init()
            node = rclpy.create_node('umrr96_readback_smoke_client')
            mode = node.create_client(GetMode, '/umrr96_readback_smoke/mode')
            status = node.create_client(GetStatus, '/umrr96_readback_smoke/status')
            setter = node.create_client(SetMode, '/umrr96_readback_smoke/set')

            def call(client, request, expected_error):
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=4)
                assert future.done(), 'ROS service did not finish'
                result = json.loads(future.result().res)
                assert result['success'] is False, result
                assert expected_error in result['error'], result
                return result

            try:
                assert mode.wait_for_service(timeout_sec=10), 'Mode service unavailable'
                assert status.wait_for_service(timeout_sec=5), 'Status service unavailable'
                assert setter.wait_for_service(timeout_sec=5), 'Set service unavailable'
                normal = dict(sensor_id=230739, section_name='auto_interface_0dim',
                              params=['frequency_sweep_idx'], param_types=[3])
                cases = [
                    ({'sensor_id': 1}, 'Sensor ID'),
                    ({'section_name': 'wrong_section'}, 'section'),
                    ({'params': [], 'param_types': []}, '1–10'),
                    ({'param_types': []}, 'equal number'),
                    ({'params': ['frequency_sweep_idx'] * 11,
                      'param_types': [3] * 11}, '1–10'),
                    ({'params': ['frequency_sweep_idx'] * 2,
                      'param_types': [3] * 2}, 'unique'),
                    ({'param_types': [4]}, 'data type'),
                    ({'params': ['nonexistent_parameter']}, 'Unknown'),
                    ({'param_types': [0]}, 'incorrectly typed'),
                ]
                for changes, error in cases:
                    call(mode, GetMode.Request(**dict(normal, **changes)), error)
                call(status, GetStatus.Request(
                    sensor_id=230739, section_name='auto_interface',
                    statuses=['sw_version_major'], status_types=[0]), 'incorrectly typed')
                tuning = dict(sensor_id=230739, section_name='auto_interface_0dim',
                              params=['frequency_sweep_idx'], values=['2'], value_types=[3])
                for changes, error in (
                    ({'sensor_id': 1}, 'sensor ID'),
                    ({'section_name': 'wrong_section'}, 'section'),
                    ({'values': []}, 'equal lengths'),
                    ({'params': ['ip_source_address']}, 'Unsupported'),
                    ({'value_types': [0]}, 'Unsupported'),
                    ({'values': ['3']}, 'Invalid tuning value'),
                    ({'values': ['-1']}, 'Invalid tuning value'),
                    ({'values': ['2junk']}, 'Invalid tuning value'),
                    ({'values': ['']}, 'Invalid tuning value'),
                    ({'params': ['frequency_sweep_idx', 'tx_antenna_idx'],
                      'values': ['2', '3'], 'value_types': [3, 3]}, 'Invalid tuning value'),
                ):
                    call(setter, SetMode.Request(**dict(tuning, **changes)), error)
                peer.settimeout(.1)
                try:
                    peer.recvfrom(65535)
                except TimeoutError:
                    pass
                else:
                    raise AssertionError('Rejected request sent a packet to the sensor')
                # Repeated timeouts must not strand batches or block the next call.
                for client, request in (
                    (mode, GetMode.Request(**normal)),
                    (status, GetStatus.Request(
                        sensor_id=230739, section_name='auto_interface',
                        statuses=['sw_version_major'], status_types=[1])),
                    (setter, SetMode.Request(**tuning)),
                ):
                    started = time.monotonic()
                    call(client, request, 'Timed out')
                    assert .20 <= time.monotonic() - started < 3
                peer.settimeout(1)
                packet, _ = peer.recvfrom(65535)
                assert packet, 'No SDK read request reached the silent peer'
            finally:
                node.destroy_node()
                rclpy.shutdown()
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    raise AssertionError('Readback node did not stop')
                finally:
                    log.seek(0)
                    print(log.read())
            assert process.returncode == 0, process.returncode
    assert set(Path('/tmp').glob('smartmicro-readback-*')) == before, 'SDK config leaked'
    print('PASS: read/write validation, repeated timeouts, clean shutdown and config cleanup')


if __name__ == '__main__':
    main()
