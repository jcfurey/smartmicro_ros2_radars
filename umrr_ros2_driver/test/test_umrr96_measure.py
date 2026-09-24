# SPDX-License-Identifier: Apache-2.0
"""Verify experiment restoration after partial writes and failures, without hardware."""

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time
import pytest
import rclpy
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from umrr_ros2_msgs.msg import RadarTiming
from umrr_ros2_msgs.srv import GetMode, GetStatus

source = Path(__file__).resolve().parents[1] / 'scripts/umrr96_measure.py'
spec = importlib.util.spec_from_file_location('umrr96_measure', source)
measure = importlib.util.module_from_spec(spec)
spec.loader.exec_module(measure)


def test_capture_statistics_are_json_serializable_and_do_not_invent_missing_quality():
    data = measure.Measurements()
    fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('range', 'snr', 'radial_speed'))]
    data.cloud(point_cloud2.create_cloud(Header(), fields, [(2, 30, -1), (8, 10, 2)]))
    data.timing(RadarTiming(device_timestamp_us=1000000, stream=RadarTiming.TARGETS))
    data.timing(RadarTiming(device_timestamp_us=1055000, stream=RadarTiming.TARGETS))
    report = data.result(1)
    json.dumps(report, allow_nan=False)
    assert report['detections_under_5m_per_frame']['mean'] == 1
    assert report['device_interval_seconds']['mean'] == .055
    assert report['quality']['variance_range']['mean'] is None
    assert report['raw_quality_frames'] == 0
    assert len(measure.PARAMETERS) == 29 and len(measure.STATUSES) == 14


class FakeControl:
    def __init__(self, manual=0, fail=None):
        self.values = dict(prf_selector_manual=manual, prf_manual_value_idx=2,
                           frequency_sweep_idx=2)
        self.original = self.values.copy()
        self.calls = 0
        self.fail = fail

    def read(self, names=None):
        return {n: self.values[n] for n in (names or self.values)}

    def write(self, values):
        self.calls += 1
        self.values.update(values)
        if self.calls == self.fail:
            raise RuntimeError('Response lost after sensor applied write')


@pytest.mark.parametrize('manual', (0, 1))
@pytest.mark.parametrize('fail', (None, 1, 3, 8))
def test_prf_restore_after_success_or_partially_applied_write(manual, fail):
    control = FakeControl(manual, fail)
    report = dict(parameters_before=control.read(), windows=[])
    saves = []
    def run():
        measure.prf_trials(control, lambda: {'frames': 100}, report,
                           lambda: saves.append(report['restoration']))
    if fail is None:
        run()
        assert len(report['windows']) == 7
    else:
        with pytest.raises(RuntimeError, match='Response lost'):
            run()
    assert control.values == control.original
    assert report['restoration'] == 'verified'
    assert saves[0] == 'pending' and saves[-1] == 'verified'


def test_interrupt_and_missing_data_restore_profile():
    for error in (KeyboardInterrupt(), RuntimeError('capture failed')):
        control = FakeControl()
        report = dict(parameters_before=control.read(), windows=[])
        def interrupt():
            raise error
        with pytest.raises(type(error)):
            measure.prf_trials(control, interrupt, report, lambda: None)
        assert report['restoration'] == 'verified'
        assert control.values == control.original
    with pytest.raises(RuntimeError, match='No target frames'):
        measure.prf_trials(control, lambda: {'frames': 0}, report, lambda: None)
    assert control.values == control.original


def test_concurrent_change_is_reported_and_unrelated_values_are_preserved():
    control = FakeControl()
    report = dict(parameters_before=control.read(), windows=[])
    def changed():
        control.values['frequency_sweep_idx'] = 1
        return {'frames': 20}
    with pytest.raises(RuntimeError, match='Settings changed'):
        measure.prf_trials(control, changed, report, lambda: None)
    assert control.values['frequency_sweep_idx'] == 1
    assert all(control.values[n] == control.original[n] for n in measure.PRF_NAMES)


def test_failed_restoration_is_never_reported_as_verified():
    control = FakeControl()
    report = dict(parameters_before=control.read(), windows=[])
    def fail():
        control.write = lambda values: (_ for _ in ()).throw(RuntimeError('Disconnected'))
        raise RuntimeError('Lost connection')
    with pytest.raises(RuntimeError, match='restoration could not be verified'):
        measure.prf_trials(control, fail, report, lambda: None)
    assert report['restoration'] == 'failed'
    assert len(report['restoration_errors']) == 3


def test_read_only_cli_captures_profile_and_raw_data(tmp_path):
    rclpy.init()
    node = rclpy.create_node('measure_cli_fixture')
    def reply(request, response):
        names = request.params if hasattr(request, 'params') else request.statuses
        response.res = json.dumps(dict(success=True, sensor_id=request.sensor_id,
            values={n: dict(response_type=1, value=0) for n in names}))
        return response
    node.create_service(GetMode, '/measure_fixture/get_radar_mode', reply)
    node.create_service(GetStatus, '/measure_fixture/get_radar_status', reply)
    publisher = node.create_publisher(measure.PointCloud2, '/measure_fixture/port_targets_0', 10)
    fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('range', 'snr', 'radial_speed'))]
    def publish():
        publisher.publish(point_cloud2.create_cloud(
            Header(stamp=node.get_clock().now().to_msg()), fields, [(2, 20, 0), (8, 10, -1)]))
    node.create_timer(.05, publish)
    output = tmp_path / 'measurement.json'
    process = subprocess.Popen([sys.executable, str(source), '--seconds', '1',
                                '--control-prefix', '/measure_fixture',
                                '--topic-prefix', '/measure_fixture',
                                '--scene', 'moving', '--output', str(output)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + 12
        while process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert process.poll() == 0, process.communicate(timeout=1)[0] if process.poll() is not None else 'CLI timeout'
        report = json.loads(output.read_text())
        assert report['success'] and report['restoration'] == 'not_needed'
        assert len(report['parameters_before']) == 29 and len(report['statuses']) == 14
        assert report['parameters_after'] == report['parameters_before']
        assert report['scene'] == 'moving' and not report['prf_trials']
        window = report['windows'][0]
        assert window['frames'] >= 10
        assert window['detections_per_frame']['mean'] == 2
        assert window['detections_under_5m_per_frame']['mean'] == 1
    finally:
        if process.poll() is None:
            process.kill()
        process.communicate(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
