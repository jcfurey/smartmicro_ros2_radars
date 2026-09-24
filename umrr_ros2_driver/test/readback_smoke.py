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
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.parameter import Parameter
from rcl_interfaces.srv import DescribeParameters, SetParametersAtomically
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
            diagnostics = []
            node.create_subscription(DiagnosticArray, '/diagnostics',
                                     lambda msg: diagnostics.extend(msg.status), 10)
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
                describe = node.create_client(DescribeParameters,
                    '/umrr96_readback_smoke_server/describe_parameters')
                assert describe.wait_for_service(timeout_sec=5)
                future = describe.call_async(DescribeParameters.Request(names=[
                    'sensor_id', 'host_port', 'sensor_port', 'host_ip', 'sensor_ip',
                    'interface_name', 'timeout_ms']))
                rclpy.spin_until_future_complete(node, future, timeout_sec=4)
                assert future.done() and all(d.read_only for d in future.result().descriptors)
                parameter_setter = node.create_client(SetParametersAtomically,
                    '/umrr96_readback_smoke_server/set_parameters_atomically')
                assert parameter_setter.wait_for_service(timeout_sec=5)
                future = parameter_setter.call_async(SetParametersAtomically.Request(parameters=[
                    Parameter('host_port', value=host_port + 1).to_parameter_msg()]))
                rclpy.spin_until_future_complete(node, future, timeout_sec=4)
                assert future.done() and not future.result().result.successful
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
                deadline = time.monotonic() + 4
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                    matched = [s for s in diagnostics if s.name.endswith('Control requests') and
                               dict((v.key, v.value) for v in s.values).get('timeouts') == '3']
                    if matched:
                        break
                assert matched and matched[-1].level == DiagnosticStatus.WARN, diagnostics
                values = {v.key: v.value for v in matched[-1].values}
                assert int(values['invalid_requests']) == 20, values
                assert int(values['failed_requests']) == 3, values
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


def test_readback():
    main()


def test_invalid_startup_parameters():
    """Reject out-of-range values before creating an SDK socket, without leaking config."""
    executable = Path(get_package_prefix('umrr_ros2_driver')) / (
        'lib/umrr_ros2_driver/smartmicro_radar_readback_node')
    before = set(Path(tempfile.gettempdir()).glob('smartmicro-readback-*'))
    for parameter in ('sensor_id:=-1', 'sensor_id:=4294967296', 'host_port:=0',
                      'sensor_port:=65536', 'timeout_ms:=30001'):
        result = subprocess.run([str(executable), '--ros-args', '-p', 'sensor_id:=230739',
                                 '-p', parameter], capture_output=True, text=True, timeout=8)
        assert result.returncode == 1, result.stdout + result.stderr
    assert set(Path(tempfile.gettempdir()).glob('smartmicro-readback-*')) == before


if __name__ == '__main__':
    main()
