#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise real SDK reception on loopback; no connected sensor or Docker required."""

import json
import os
from pathlib import Path
import signal
import struct
import socket
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import DescribeParameters, SetParametersAtomically
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from umrr_ros2_msgs.msg import PortTargetHeader, RadarTiming, Umrr96RawQuality
from umrr_ros2_msgs.srv import FirmwareDownload, SetMode
import yaml


def unused_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def test_driver_runtime():
    prefix = Path(get_package_prefix('umrr_ros2_driver'))
    executable = prefix / 'lib/umrr_ros2_driver/smartmicro_radar_node_exe'
    config = Path(get_package_share_directory('umrr_ros2_driver')) / 'config'
    originals = {p: p.read_bytes() for p in config.glob('*.json')}
    repo = Path(__file__).resolve().parents[2]
    processes, logs = [], []
    rclpy.init()
    node = rclpy.create_node('driver_runtime_test')
    clouds, headers, timing, statuses = [], [], [], []
    quality = []
    topic = '/driver_runtime/a/smart_radar/'
    node.create_subscription(PointCloud2, topic + 'port_targets_0', clouds.append, 10)
    node.create_subscription(PortTargetHeader, topic + 'port_targetheader_0', headers.append, 10)
    node.create_subscription(RadarTiming, topic + 'timing_0', timing.append, 10)
    node.create_subscription(Umrr96RawQuality, topic + 'umrr96_raw_quality_0', quality.append, 10)
    node.create_subscription(DiagnosticArray, '/diagnostics',
                             lambda msg: statuses.extend(msg.status), 10)

    def wait(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError('Timed out waiting for test condition')

    def call(client, request):
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(request)
        wait(future.done)
        return future.result()

    def stop(process):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise AssertionError('Node failed to shut down cleanly')
        assert process.returncode == 0

    try:
        with tempfile.TemporaryDirectory(prefix='umrr-runtime-test-') as directory:
            run = Path(directory)

            def launch(command, **env):
                log = tempfile.TemporaryFile(mode='w+')
                logs.append(log)
                process = subprocess.Popen(command, env=dict(os.environ, TMPDIR=str(run), **env),
                                           stdout=log, stderr=subprocess.STDOUT)
                processes.append(process)
                return process

            ports = set()
            while len(ports) < 3:
                ports.add(unused_port())
            port_a, port_b, peer_port = ports
            parameters = dict(
                master_data_serial_type='port_based', master_inst_serial_type='port_based',
                adapters={'adapter_0': dict(hw_type='eth', hw_dev_id=4, hw_iface_name='lo',
                                           hw_ip_address='127.0.0.1', port=port_a)},
                sensors={'sensor_0': dict(
                    link_type='eth', pub_type='target', model='umrr96_v1_2_2', dev_id=4,
                    id=200, frame_id='umrr96_test', history_size=10, ip='127.0.0.1',
                    port=peer_port, inst_type='port_based', data_type='port_based',
                    uifname='umrr96_t153_automotive', uifmajorv=1, uifminorv=2, uifpatchv=2)},
                **{'diagnostics.stale_timeout': .5})
            driver_processes = []
            for name, port in (('a', port_a), ('b', port_b)):
                parameters['adapters']['adapter_0']['port'] = port
                params = run / f'{name}.yaml'
                params.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
                driver_processes.append(launch([
                    str(executable), '--ros-args', '-r', f'__ns:=/driver_runtime/{name}',
                    '-r', f'__node:=runtime_{name}', '--params-file', str(params)]))
            wait(lambda: node.count_publishers(topic + 'port_targets_0') and
                 node.count_publishers('/driver_runtime/b/smart_radar/port_targets_0'))
            dirs = list(run.glob('smartmicro-data-*'))
            assert len(dirs) == 2, dirs
            assert {json.loads((p / 'hw_inventory.json').read_text())['hwItems'][0]['port']
                    for p in dirs} == {port_a, port_b}
            for path in dirs:
                assert json.loads((path / 'smart_access_config.json').read_text())[
                    'config_path'] == str(path)
            assert all(p.read_bytes() == value for p, value in originals.items())

            descriptions = node.create_client(
                DescribeParameters, '/driver_runtime/a/runtime_a/describe_parameters')
            names = ['adapters.adapter_0.port', 'sensors.sensor_0.id',
                     'sensors.sensor_0.history_size', 'diagnostics.stale_timeout']
            assert all(d.read_only for d in call(
                descriptions, DescribeParameters.Request(names=names)).descriptors)
            setter = node.create_client(
                SetParametersAtomically, '/driver_runtime/a/runtime_a/set_parameters_atomically')
            result = call(setter, SetParametersAtomically.Request(parameters=[
                Parameter('adapters.adapter_0.port', value=port_b).to_parameter_msg()]))
            assert not result.result.successful

            # Service validation happens before anything reaches the SDK (C5, C6).
            set_mode = node.create_client(SetMode, topic + 'set_radar_mode')
            for sensor_id, value, value_type, expected in (
                    (0, '1', 3, 'Sensor ID is invalid'),
                    (200, '12abc', 1, 'not a decimal uint32'),
                    (200, '-1', 1, 'not a decimal uint32'),
                    (200, '5000000000', 1, 'out of range'),
                    (200, 'nan', 0, 'not a finite float32'),
                    (200, '256', 3, 'out of range')):
                response = call(set_mode, SetMode.Request(
                    section_name='auto_interface_0dim', sensor_id=sensor_id,
                    params=['frequency_sweep_idx'], values=[value], value_types=[value_type]))
                assert expected in response.res, (value, response.res)
            # Firmware download replies are deferred to a worker thread (C4).
            download = node.create_client(FirmwareDownload, topic + 'firmware_download')
            response = call(download, FirmwareDownload.Request(sensor_id=0, file_path='/none'))
            assert 'invalid' in response.res, response.res
            response = call(download, FirmwareDownload.Request(
                sensor_id=200, file_path=str(run / 'missing.bin')))
            assert 'could not open update image' in response.res, response.res
            wait(lambda: any(s.name.endswith('Target stream 0') and
                             s.level == DiagnosticStatus.STALE for s in statuses))

            sim = run / 'sender'
            sim.mkdir()
            for filename in ('com_lib_config.json', 'hw_inventory.json', 'routing_table.json'):
                data = json.loads((repo / 'simulator/config_umrr96' / filename).read_text())
                if filename == 'com_lib_config.json':
                    data.update(shared_lib_path=str(prefix / 'lib'), config_path=str(sim),
                                user_interface_patch_v=2)
                elif filename == 'hw_inventory.json':
                    data['hwItems'][0].update(iface_name='lo', ip_address='127.0.0.1',
                                              port=peer_port)
                else:
                    data['clients'][0].update(ip='127.0.0.1', port=port_a)
                (sim / filename).write_text(json.dumps(data))
            # Port 2.1 uses a 24-byte network-order generic header and a
            # little-endian 8-byte list header followed by 56-byte target records.
            fixture = bytearray((repo / 'simulator/targetlist_port_v2_1_0.bin').read_bytes())
            struct.pack_into('<fHH', fixture, 24, .1, 17, 0x1234)
            for index in range(17):
                struct.pack_into('<10fIffH', fixture, 32 + index * 56,
                    1.0 + index, .5, .1, .2, .01 + index, .02 + index,
                    .03 + index, .04 + index, 2.0, .25, 0x123400 + index, 40.0, 10.0, index + 100)
            fixture_path = run / 'known_quality_port.bin'
            fixture_path.write_bytes(fixture)
            started_ns = node.get_clock().now().nanoseconds
            sender = launch([os.environ['SMARTMICRO_TEST_SENDER'],
                             str(fixture_path)],
                            SMART_ACCESS_CFG_FILE_PATH=str(sim / 'com_lib_config.json'))
            wait(lambda: len(clouds) >= 5 and len(timing) >= 5 and len(headers) >= 5
                 and len(quality) >= 5)
            matched = 0
            for cloud in clouds:
                stamp = cloud.header.stamp
                ros_ns = stamp.sec * 10**9 + stamp.nanosec
                assert started_ns <= ros_ns <= node.get_clock().now().nanoseconds
                found = [t for t in timing if t.header == cloud.header]
                if not found:
                    continue  # Independent DDS topics may be delivered in different orders.
                matched += 1
                raw = found[0]
                assert raw.timestamp_source == RadarTiming.ROS_RECEIVE_TIME
                assert raw.stream == RadarTiming.TARGETS and raw.sensor_id == 200
                assert raw.device_timestamp_us * 1000 != ros_ns
                assert any(h.header == cloud.header for h in headers)
                assert cloud.point_step == 72 and cloud.width == 17
                metadata = next(h for h in headers if h.header == cloud.header)
                assert metadata.acquisition_setup_valid and metadata.acquisition_setup == 0x1234
                raw_values = next((q for q in quality if q.header == cloud.header), None)
                if raw_values is None:
                    continue
                assert raw_values.sensor_id == 200
                assert raw_values.semantics == Umrr96RawQuality.SEMANTICS_UNVERIFIED
                assert list(raw_values.false_alarm_probability_raw) == [.25] * 17
                assert list(raw_values.flags_raw) == list(range(0x123400, 0x123400 + 17))
                records = point_cloud2.read_points(cloud)
                for index, record in enumerate(records):
                    for field, base in (('variance_range', .01), ('variance_speed', .02),
                                        ('variance_azimuth_angle', .03),
                                        ('variance_elevation_angle', .04)):
                        expected = struct.unpack('<f', struct.pack('<f', base + index))[0]
                        assert float(record[field]) == expected, (field, index, record[field])
                    assert int(record['peak_idx']) == index + 100
            wait(lambda: any(s.name == 'runtime_a: UDP adapter 0' and
                             {v.key: v.value for v in s.values}.get('kernel_counters_available') == 'True'
                             for s in statuses))
            assert matched >= 3
            # The fixture deliberately repeats its original counter. ROS stamps still advance.
            assert len({t.device_timestamp_us for t in timing}) == 1
            assert len({(c.header.stamp.sec, c.header.stamp.nanosec) for c in clouds}) >= 5
            wait(lambda: any(s.name == 'runtime_a: Target stream 0' and
                             s.level == DiagnosticStatus.WARN and
                             int(dict((v.key, v.value) for v in s.values).get(
                                 'timestamp_repeats', '0')) > 0 for s in statuses))
            stop(sender)
            statuses.clear()
            wait(lambda: any(s.name == 'runtime_a: Target stream 0' and
                             s.level == DiagnosticStatus.STALE for s in statuses))
            restarted_sender = launch([os.environ['SMARTMICRO_TEST_SENDER'], str(fixture_path)],
                SMART_ACCESS_CFG_FILE_PATH=str(sim / 'com_lib_config.json'))
            previous_count = len(clouds)
            wait(lambda: len(clouds) >= previous_count + 3)
            stop(driver_processes[0])
            stop(restarted_sender)
            assert len(list(run.glob('smartmicro-data-*'))) == 1
            stop(driver_processes[1])
            assert not list(run.glob('smartmicro-data-*'))
            assert all(p.read_bytes() == value for p, value in originals.items())
    finally:
        try:
            for process in reversed(processes):
                if process.poll() is None:
                    stop(process)
        finally:
            for log in logs:
                log.seek(0)
                print(log.read())
                log.close()
            node.destroy_node()
            rclpy.shutdown()
