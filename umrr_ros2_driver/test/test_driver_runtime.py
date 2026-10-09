#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Exercise real SDK reception on loopback; no connected sensor or Docker required."""

import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pytest
from rcl_interfaces.srv import DescribeParameters, SetParametersAtomically
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from umrr_ros2_msgs.msg import PortTargetHeader, RadarTiming, Umrr96RawQuality
from umrr_ros2_msgs.srv import FirmwareDownload, GetMode, GetStatus, SetMode
import yaml

try:  # Optional: the driver publishes RadarScan only when built with radar_msgs.
    from radar_msgs.msg import RadarScan
except ImportError:
    RadarScan = None


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
    scans = []
    if RadarScan is not None:
        node.create_subscription(RadarScan, topic + 'radar_scan_0', scans.append, 10)

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
            parameters = dict(  # noqa: C408 (keyword form mirrors the parameter file)
                master_data_serial_type='port_based', master_inst_serial_type='port_based',
                adapters={'adapter_0': dict(  # noqa: C408
                    hw_type='eth', hw_dev_id=4, hw_iface_name='lo',
                    hw_ip_address='127.0.0.1', port=port_a)},
                sensors={'sensor_0': dict(  # noqa: C408
                    link_type='eth', pub_type='target', model='umrr96_v1_2_2', dev_id=4,
                    id=200, frame_id='umrr96_test', history_size=10, ip='127.0.0.1',
                    port=peer_port, inst_type='port_based', data_type='port_based',
                    uifname='umrr96_t153_automotive', uifmajorv=1, uifminorv=2, uifpatchv=2)},
                instruction_timeout_ms=300, **{'diagnostics.stale_timeout': .5})
            driver_processes = []
            for name, port in (('a', port_a), ('b', port_b)):
                parameters['adapters']['adapter_0']['port'] = port
                parameters['publish_radar_scan'] = RadarScan is not None and name == 'a'
                if name == 'b':  # Standard QoS override parameters (REP 2003 consumers).
                    parameters['qos_overrides'] = {
                        '/driver_runtime/b/smart_radar/port_targets_0': {
                            'publisher': {'reliability': 'best_effort', 'depth': 3}}}
                    # An explicit value wins over the namespaced default (C57).
                    parameters['diagnostic_updater'] = {'use_fqn': False}
                    # An unset user interface is taken from the model (C64).
                    for key in ('uifname', 'uifmajorv', 'uifminorv', 'uifpatchv'):
                        del parameters['sensors']['sensor_0'][key]
                params = run / f'{name}.yaml'
                params.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
                driver_processes.append(launch([
                    str(executable), '--ros-args', '-r', f'__ns:=/driver_runtime/{name}',
                    '-r', f'__node:=runtime_{name}', '--params-file', str(params)]))
            # Namespaced nodes report fully qualified status names by default (C57).
            status_a = '/driver_runtime/a/runtime_a: '
            wait(lambda: node.count_publishers(topic + 'port_targets_0') and
                 node.count_publishers('/driver_runtime/b/smart_radar/port_targets_0'))
            qos_a = node.get_publishers_info_by_topic(topic + 'port_targets_0')[0].qos_profile
            qos_b = node.get_publishers_info_by_topic(
                '/driver_runtime/b/smart_radar/port_targets_0')[0].qos_profile
            assert qos_a.reliability == ReliabilityPolicy.RELIABLE and qos_a.depth == 10
            assert qos_b.reliability == ReliabilityPolicy.BEST_EFFORT and qos_b.depth == 3
            dirs = list(run.glob('smartmicro-data-*'))
            assert len(dirs) == 2, dirs
            assert {json.loads((p / 'hw_inventory.json').read_text())['hwItems'][0]['port']
                    for p in dirs} == {port_a, port_b}
            for path in dirs:
                client = json.loads((path / 'routing_table.json').read_text())['clients'][0]
                assert [client['user_interface_name'], client['user_interface_major_v'],
                        client['user_interface_minor_v'], client['user_interface_patch_v']] == [
                    'umrr96_t153_automotive', 1, 2, 2], client
                sdk_config = json.loads((path / 'smart_access_config.json').read_text())
                assert sdk_config['config_path'] == str(path)
                assert sdk_config['shared_lib_path'] == str(path / 'sdk-lib')
                assert (path / 'sdk-lib' / 'libsmart_access.so').exists()
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
            # Control requests reach the SDK and reply when the sensor answers. Nothing
            # listens on the sensor port yet, so both concurrent reads time out; neither
            # blocks the executor. A name or type the UIF rejects fails before sending.
            get_mode = node.create_client(GetMode, topic + 'get_radar_mode')
            get_status = node.create_client(GetStatus, topic + 'get_radar_status')
            assert get_mode.wait_for_service(timeout_sec=10)
            assert get_status.wait_for_service(timeout_sec=10)
            started = time.monotonic()
            pending = [
                get_mode.call_async(GetMode.Request(
                    section_name='auto_interface_0dim', sensor_id=200,
                    params=['output_control_target_list_can'], param_types=[3])),
                get_status.call_async(GetStatus.Request(
                    section_name='auto_interface', sensor_id=200,
                    statuses=['sw_version_major'], status_types=[1]))]
            wait(lambda: all(f.done() for f in pending))
            elapsed = time.monotonic() - started
            for future in pending:
                reply = json.loads(future.result().res)
                assert reply['sensor_id'] == 200 and reply['success'] is False, reply
                assert 'Timed out after 300 ms' in reply['error'], reply
            assert .25 < elapsed < 2.5, elapsed
            response = call(get_status, GetStatus.Request(
                section_name='auto_interface', sensor_id=200,
                statuses=['sw_version_major'], status_types=[0]))
            assert 'Failed to add instruction' in response.res, response.res
            response = call(get_mode, GetMode.Request(
                section_name='auto_interface_0dim', sensor_id=200, params=[], param_types=[]))
            assert 'non-empty' in response.res, response.res
            wait(lambda: any(
                s.name == status_a + 'SDK callbacks' and s.level == DiagnosticStatus.OK and
                int({v.key: v.value for v in s.values}.get('instruction_timeouts', '0')) >= 2
                for s in statuses))
            assert any(s.name == 'runtime_b: Target stream 0' for s in statuses)
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
                    # The SDK aborts on library paths of 160+ characters; use a short alias.
                    sdk_alias = run / 'sdk-lib'
                    sdk_alias.symlink_to(prefix / 'lib' / 'umrr_ros2_driver')
                    data.update(shared_lib_path=str(sdk_alias), config_path=str(sim),
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
                struct.pack_into(
                    '<10fIffH', fixture, 32 + index * 56,
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
            if RadarScan is not None:
                wait(lambda: scans)
                scan = scans[-1]
                cloud = next((c for c in clouds if c.header == scan.header), None)
                assert len(scan.returns) == 17
                if cloud is not None:
                    ranges = [float(r['range']) for r in point_cloud2.read_points(cloud)]
                    assert [r.range for r in scan.returns] == ranges
            wait(lambda: any(
                s.name == status_a + 'UDP adapter 0' and
                {v.key: v.value for v in s.values}.get('kernel_counters_available') == 'True'
                for s in statuses))
            assert matched >= 3
            # The fixture deliberately repeats its original counter. ROS stamps still advance.
            assert len({t.device_timestamp_us for t in timing}) == 1
            assert len({(c.header.stamp.sec, c.header.stamp.nanosec) for c in clouds}) >= 5
            wait(lambda: any(s.name == status_a + 'Target stream 0' and
                             s.level == DiagnosticStatus.WARN and
                             int({v.key: v.value for v in s.values}.get(
                                 'timestamp_repeats', '0')) > 0 for s in statuses))
            stop(sender)
            statuses.clear()
            wait(lambda: any(s.name == status_a + 'Target stream 0' and
                             s.level == DiagnosticStatus.STALE for s in statuses))
            restarted_sender = launch(
                [os.environ['SMARTMICRO_TEST_SENDER'], str(fixture_path)],
                SMART_ACCESS_CFG_FILE_PATH=str(sim / 'com_lib_config.json'))
            previous_count = len(clouds)
            wait(lambda: len(clouds) >= previous_count + 3)
            stop(restarted_sender)
            # A cycle without targets: an empty cloud and an empty RadarScan (C53).
            empty = bytearray(fixture[:32])
            struct.pack_into('>I', empty, 16, len(empty))  # Generic header port size.
            struct.pack_into('<H', empty, 28, 0)  # Number of targets.
            empty_path = run / 'empty_port.bin'
            empty_path.write_bytes(empty)
            empty_sender = launch([os.environ['SMARTMICRO_TEST_SENDER'], str(empty_path)],
                                  SMART_ACCESS_CFG_FILE_PATH=str(sim / 'com_lib_config.json'))
            wait(lambda: any(c.width == 0 for c in clouds[previous_count:]))
            if RadarScan is not None:
                scan_count = len(scans)
                wait(lambda: any(not s.returns for s in scans[scan_count:]))
                empty_scan = next(s for s in scans[scan_count:] if not s.returns)
                assert empty_scan.header.frame_id == 'umrr96_test'
            assert driver_processes[0].poll() is None
            stop(driver_processes[0])
            stop(empty_sender)
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


def test_radar_scan_requires_radar_msgs():
    """Without radar_msgs, enabling the RadarScan output fails at startup."""
    if RadarScan is not None:
        return
    prefix = Path(get_package_prefix('umrr_ros2_driver'))
    with tempfile.TemporaryDirectory(prefix='umrr-radar-scan-test-') as directory:
        params = Path(directory) / 'params.yaml'
        params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
            'publish_radar_scan': True,
            'adapters': {'adapter_0': {'hw_type': 'eth', 'hw_dev_id': 4,
                                       'hw_iface_name': 'lo', 'port': unused_port()}},
            'sensors': {'sensor_0': {
                'link_type': 'eth', 'pub_type': 'target', 'model': 'umrr96_v1_2_2',
                'dev_id': 4, 'id': 200, 'ip': '127.0.0.1', 'port': unused_port()}}}}}))
        result = subprocess.run(
            [str(prefix / 'lib/umrr_ros2_driver/smartmicro_radar_node_exe'),
             '--ros-args', '--params-file', str(params)],
            capture_output=True, text=True, timeout=20,
            env=dict(os.environ, TMPDIR=directory))
        assert result.returncode != 0
        assert 'built without radar_msgs' in result.stdout + result.stderr
        assert not list(Path(directory).glob('smartmicro-data-*'))


def _startup_failure(parameters, directory):
    params = Path(directory) / 'params.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    prefix = Path(get_package_prefix('umrr_ros2_driver'))
    result = subprocess.run(
        [str(prefix / 'lib/umrr_ros2_driver/smartmicro_radar_node_exe'),
         '--ros-args', '--params-file', str(params)],
        capture_output=True, text=True, timeout=20,
        env=dict(os.environ, TMPDIR=directory))
    assert result.returncode != 0
    assert not list(Path(directory).glob('smartmicro-data-*'))
    return result.stdout + result.stderr


def _eth_sensor(**overrides):
    sensor = {'link_type': 'eth', 'pub_type': 'target', 'model': 'umrr96_v1_2_2',
              'dev_id': 4, 'id': 200, 'ip': '127.0.0.1', 'port': 55555}
    sensor.update(overrides)
    return sensor


@pytest.mark.parametrize('sensors, extra, message', [
    ({'sensor_0': _eth_sensor(ip='192.168.11.1l')}, {}, "sensor_0.ip must be the sensor's IPv4"),
    ({'sensor_0': _eth_sensor(), 'sensor_1': _eth_sensor()}, {},
     'sensor_1.id duplicates sensors.sensor_0'),
    ({'sensor_0': _eth_sensor(frame_id='/umrr')}, {}, "frame_id must not start with '/'"),
    ({'sensor_0': _eth_sensor(link_type='can', model='umrr96_can_v1_2_2')}, {},
     "does not match the 'eth' adapter"),
    ({'sensor_0': _eth_sensor()}, {'instruction_timeout_ms': 50}, 'instruction_timeout_ms'),
    ({'sensor_0': _eth_sensor(frame_id='radar'),
      'sensor_1': _eth_sensor(id=201, frame_id='radar')}, {},
     "sensor_1.frame_id 'radar' duplicates sensors.sensor_0"),
    ({'sensor_0': _eth_sensor(uifname='umrra4_automotive', uifmajorv=1, uifminorv=6,
                              uifpatchv=0)}, {},
     "sensor_0.uifname 'umrra4_automotive' does not match model 'umrr96_v1_2_2'"),
    ({'sensor_0': _eth_sensor(uifname='umrr96_t153_automotive', uifmajorv=1, uifminorv=2,
                              uifpatchv=1)}, {},
     "sensor_0.uifpatchv 1 does not match model 'umrr96_v1_2_2' (expects 2)"),
])
def test_invalid_startup_configuration_is_rejected(sensors, extra, message):
    """Configurations that would silently deliver nothing fail at startup instead."""
    with tempfile.TemporaryDirectory(prefix='umrr-startup-test-') as directory:
        output = _startup_failure(dict(
            adapters={'adapter_0': {'hw_type': 'eth', 'hw_dev_id': 4, 'hw_iface_name': 'lo',
                                    'port': unused_port()}},
            sensors=sensors, **extra), directory)
        assert message in output, output


def _shipped_sensors():
    """Yield the file name and sensors of every shipped driver parameter file."""
    for path in sorted((Path(__file__).resolve().parents[1] / 'param').rglob('*.yaml')):
        for section in yaml.safe_load(path.read_text()).values():
            if not isinstance(section, dict):
                continue  # model_uif_catalogue.yaml
            sensors = (section.get('ros__parameters') or {}).get('sensors')
            if isinstance(sensors, dict):
                yield path.name, sensors


def test_shipped_parameter_files_match_the_model_catalogue():
    """Every shipped sensor names its model's interface (C64) and has its own frame (C69)."""
    catalogue = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / 'param/model_uif_catalogue.yaml').read_text())
    interfaces = {entry['model']: entry['uifname']
                  for entry in catalogue['entries_can'] + catalogue['entries_port']}
    checked = 0
    for name, sensors in _shipped_sensors():
        frames = [sensor['frame_id'] for sensor in sensors.values()]
        assert len(set(frames)) == len(frames), (name, frames)
        for key, sensor in sensors.items():
            version = [int(part) for part in sensor['model'].rsplit('_v', 1)[1].split('_')]
            assert sensor['uifname'] == interfaces[sensor['model']], (name, key)
            assert [sensor['uifmajorv'], sensor['uifminorv'], sensor['uifpatchv']] == version, (
                name, key)
            checked += 1
    assert checked >= 10, checked


def test_status_per_configured_stream():
    """Object streams get their own status, only where configured (C62)."""
    executable = Path(get_package_prefix('umrr_ros2_driver')) / (
        'lib/umrr_ros2_driver/smartmicro_radar_node_exe')
    rclpy.init()
    node = rclpy.create_node('driver_streams_test')
    statuses = {}
    node.create_subscription(DiagnosticArray, '/diagnostics', lambda msg: statuses.update(
        {s.name: s for s in msg.status}), 10)
    serialization = {'inst_type': 'port_based', 'data_type': 'port_based'}
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='umrr-streams-test-') as directory:
            params = Path(directory) / 'params.yaml'
            params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
                'master_data_serial_type': 'port_based', 'master_inst_serial_type': 'port_based',
                'adapters': {'adapter_0': {
                    'hw_type': 'eth', 'hw_dev_id': 4, 'hw_iface_name': 'lo',
                    'hw_ip_address': '127.0.0.1', 'port': unused_port()}},
                'sensors': {
                    'sensor_0': _eth_sensor(pub_type='mse', model='umrra4_mse_v3_0_0',
                                            frame_id='umrr_0', port=unused_port(),
                                            **serialization),
                    # The SDK fails to initialize with two clients on one address.
                    'sensor_1': _eth_sensor(id=201, frame_id='umrr_1', ip='127.0.0.2',
                                            port=unused_port(), **serialization)}}}}))
            process = subprocess.Popen(
                [str(executable), '--ros-args', '-r', '__ns:=/driver_streams',
                 '-r', '__node:=streams', '--params-file', str(params)],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                env=dict(os.environ, TMPDIR=directory))
            prefix = '/driver_streams/streams: '
            expected = [prefix + name for name in (
                'Object stream 0', 'SDK callbacks', 'Target stream 0', 'Target stream 1',
                'UDP adapter 0')]
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and process.poll() is None and not all(
                    name in statuses and statuses[name].message != 'Node starting up'
                    for name in expected):
                rclpy.spin_once(node, timeout_sec=.1)
            assert process.poll() is None, 'driver exited'
            assert sorted(name for name in statuses if name.startswith(prefix)) == expected
            objects = statuses[prefix + 'Object stream 0']
            assert objects.level == DiagnosticStatus.STALE, objects
            assert objects.message == 'Waiting for objects', objects
            assert objects.hardware_id == 'umrra4_mse_v3_0_0@127.0.0.1', objects
            assert statuses[prefix + 'Target stream 1'].message == 'Waiting for targets'
    finally:
        output = ''
        if process is not None:
            process.send_signal(signal.SIGINT)
            try:
                output, _ = process.communicate(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                output, _ = process.communicate()
            print(output)
        node.destroy_node()
        rclpy.shutdown()
    assert process.returncode == 0, output
