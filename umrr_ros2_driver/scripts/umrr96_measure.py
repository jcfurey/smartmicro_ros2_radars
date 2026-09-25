#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Read a complete UMRR-96 profile and measure detections; optionally trial PRFs."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from umrr_ros2_msgs.msg import PortTargetHeader, RadarTiming, Umrr96RawQuality
from umrr_ros2_msgs.srv import GetMode, GetStatus, SetMode


# Names/types from Smart Access UIF 1.2.2 auto_interface{,_0dim}. These are
# read requests, not a claim that every firmware implements optional outputs.
PARAMETERS = dict.fromkeys((
    'tx_antenna_idx', 'center_frequency_idx', 'frequency_sweep_idx', 'range_toggle_mode',
    'prf_selector_manual', 'prf_set_selector', 'prf_manual_value_idx',
    'output_control_target_list_can', 'output_control_object_list_can',
    'output_control_target_list_eth', 'output_control_object_list_eth',
    'sync_mode', 'sync_slave_identifier', 'sync_group_identifier',
    'sync_nof_devices_1st_group', 'sync_nof_devices_2nd_group', 'sync_interface',
    'time_sync_mode', 'time_sync_nof_devices'), 3)
PARAMETERS.update({f'tv_{bound}_speed_sweep_idx_{sweep}': 0
                   for sweep in range(3) for bound in ('min', 'max')})
PARAMETERS.update(dict.fromkeys(('ip_source_address', 'subnet_mask', 'ip_dest_address'), 1))
PARAMETERS['ip_dest_port'] = 2
STATUSES = dict.fromkeys((
    'auto_interface_version_major', 'auto_interface_version_minor',
    'product_serial', 'product_gen', 'product_mod_high', 'product_mod_low', 'product_rev',
    'systime_low_dword', 'systime_high_dword'), 0)
STATUSES.update(dict.fromkeys((
    'sw_generation', 'sw_version_major', 'sw_version_minor', 'sw_version_patch',
    'customer_id'), 1))
PRF_NAMES = ('prf_selector_manual', 'prf_manual_value_idx')


def distribution(values):
    finite = [float(v) for v in values if math.isfinite(v)]
    return {'count': len(values), 'finite': len(finite),
            'min': min(finite) if finite else None, 'max': max(finite) if finite else None,
            'mean': float(np.mean(finite)) if finite else None,
            'median': float(np.median(finite)) if finite else None}


class Measurements:
    """Bounded window statistics from raw detections, independent of host filtering."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.counts, self.near, self.arrivals, self.timestamps = [], [], [], []
        self.ranges, self.snr, self.speed = [], [], []
        self.quality = {n: [] for n in (
            'variance_range', 'variance_speed', 'variance_azimuth_angle',
            'variance_elevation_angle', 'false_alarm_probability')}
        self.flags, self.acquisition, self.cycles = Counter(), Counter(), []
        self.raw_pfa, self.raw_flags = [], Counter()
        self.raw_quality_frames = 0
        self.layout = []

    def cloud(self, msg):
        points = point_cloud2.read_points(msg)
        self.arrivals.append(time.monotonic())
        self.counts.append(len(points))
        self.layout = [f.name for f in msg.fields]
        self.near.append(sum(math.isfinite(float(p['range'])) and 0 <= float(p['range']) < 5
                             for p in points))
        for p in points:
            self.ranges.append(float(p['range']))
            self.snr.append(float(p['snr']))
            self.speed.append(float(p['radial_speed']))
            for name, values in self.quality.items():
                if name in self.layout:
                    values.append(float(p[name]))
            if 'flags' in self.layout:
                self.flags[int(p['flags'])] += 1

    def header(self, msg):
        self.cycles.append(msg.cycle_time)
        if msg.acquisition_setup_valid:
            self.acquisition[msg.acquisition_setup] += 1

    def timing(self, msg):
        if msg.stream == RadarTiming.TARGETS:
            self.timestamps.append(msg.device_timestamp_us)

    def raw_quality(self, msg):
        self.raw_quality_frames += 1
        self.raw_pfa.extend(msg.false_alarm_probability_raw)
        self.raw_flags.update(msg.flags_raw)

    def result(self, seconds):
        intervals = np.diff(self.timestamps) / 1e6
        return {'duration_seconds': seconds, 'frames': len(self.counts),
                'observed_hz': (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])
                if len(self.arrivals) > 1 else 0,
                'detections_per_frame': distribution(self.counts),
                'detections_under_5m_per_frame': distribution(self.near),
                'range_m': distribution(self.ranges), 'snr_db': distribution(self.snr),
                'radial_speed_mps': distribution(self.speed),
                'device_interval_seconds': distribution(intervals),
                'device_timestamp_nonincreasing': sum(int(v <= 0) for v in intervals),
                'sensor_cycle_seconds': distribution(self.cycles),
                'acquisition_setup_counts': dict(self.acquisition),
                'quality': {n: distribution(v) for n, v in self.quality.items()},
                'flags_counts': dict(self.flags), 'fields': self.layout,
                'raw_quality_frames': self.raw_quality_frames,
                'raw_pfa': distribution(self.raw_pfa), 'raw_flags_counts': dict(self.raw_flags),
                'raw_quality_semantics': 'unverified'}


class Control:
    """Read/write radar parameters through the driver's mode services."""

    def __init__(self, node, sensor, prefix):
        self.node, self.sensor = node, sensor
        self.getter = node.create_client(GetMode, prefix + '/get_radar_mode')
        self.status = node.create_client(GetStatus, prefix + '/get_radar_status')
        self.setter = node.create_client(SetMode, prefix + '/set_radar_mode')

    def call(self, client, request, names):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'Unavailable service: {client.srv_name}')
        future = client.call_async(request)
        # The control node supports up to 30 seconds sensor response timeout.
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=35)
        if not future.done():
            client.remove_pending_request(future)
            raise RuntimeError('ROS control timeout; sensor state may have changed')
        result = json.loads(future.result().res)
        if not result.get('success') or result.get('sensor_id') != self.sensor:
            raise RuntimeError(f'Control request failed: {result}')
        values = result.get('values', {})
        if any(values.get(n, {}).get('response_type') != 1 for n in names):
            raise RuntimeError(f'Incomplete sensor response: {result}')
        values = {n: values[n]['value'] for n in names}
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values.values()):
            raise RuntimeError(f'Non-finite or non-numeric sensor response: {values}')
        return values

    def read(self, names=None, status=False):
        definitions = STATUSES if status else PARAMETERS
        names = list(definitions if names is None else names)
        values = {}
        for offset in range(0, len(names), 10):
            batch = names[offset:offset + 10]
            common = {'sensor_id': self.sensor,
                      'section_name': 'auto_interface' if status else 'auto_interface_0dim'}
            if status:
                request = GetStatus.Request(**common, statuses=batch,
                                            status_types=[definitions[n] for n in batch])
            else:
                request = GetMode.Request(**common, params=batch,
                                          param_types=[definitions[n] for n in batch])
            values.update(self.call(self.status if status else self.getter, request, batch))
        return values

    def write(self, values):
        names = list(values)
        request = SetMode.Request(
            sensor_id=self.sensor, section_name='auto_interface_0dim', params=names,
            values=[str(values[n]) for n in names], value_types=[PARAMETERS[n] for n in names])
        self.call(self.setter, request, names)
        actual = self.read(names)
        if any(not math.isclose(actual[n], values[n], rel_tol=1e-6, abs_tol=1e-6) for n in names):
            raise RuntimeError(f'Write readback mismatch: expected {values}, received {actual}')


def prf_trials(control, observe, report, save):
    """Interleave automatic baselines with manual indices; restore even after a failed write."""
    original = report['parameters_before']
    report['restore_values'] = {n: original[n] for n in PRF_NAMES}
    report['restoration'] = 'pending'
    save()
    try:
        for label, index in [('automatic', None), ('manual_0', 0), ('automatic', None),
                             ('manual_1', 1), ('automatic', None), ('manual_2', 2),
                             ('automatic', None)]:
            control.write({'prf_selector_manual': 0})
            if index is not None:
                control.write({'prf_manual_value_idx': index})
                control.write({'prf_selector_manual': 1})
            before = control.read()
            if any(before[n] != original[n] for n in original if n not in PRF_NAMES):
                raise RuntimeError('Other sensor settings changed; stopping the PRF comparison')
            observation = observe()
            after = control.read()
            report['windows'].append(dict(label=label, settings=before, **observation))
            save()
            if after != before:
                raise RuntimeError('Settings changed during the measurement window')
            if not observation['frames']:
                raise RuntimeError('No target frames received; stopping the PRF comparison')
    finally:
        report['restoration'] = 'restoring'
        errors = []
        # Attempt all restoration steps, including after a timeout that may have
        # applied on the radar. Do not overwrite unrelated settings.
        for values in ({'prf_selector_manual': 0},
                       {'prf_manual_value_idx': original['prf_manual_value_idx']},
                       {'prf_selector_manual': original['prf_selector_manual']}):
            try:
                control.write(values)
            except Exception as error:
                errors.append(str(error))
        try:
            restored = control.read(PRF_NAMES)
            report['restoration_readback'] = restored
            if restored != report['restore_values']:
                errors.append('Final PRF values do not match the saved profile')
        except Exception as error:
            errors.append(str(error))
        report['restoration'] = 'failed' if errors else 'verified'
        report['restoration_errors'] = errors
        save()
        if errors:
            raise RuntimeError(f'PRF restoration could not be verified: {errors}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-id', type=lambda v: int(v, 0), default=230739)
    parser.add_argument('--control-prefix', default='/smart_radar')
    parser.add_argument('--topic-prefix', default='/smart_radar')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--scene', choices=('stationary', 'moving', 'unknown'), default='unknown')
    parser.add_argument('--prf-trials', action='store_true',
                        help='Change volatile PRF settings and restore them')
    parser.add_argument('--output', type=Path, required=True,
                        help='New JSON report; will not overwrite an existing file')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('--seconds must be within 1..120')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        stream.write('{}\n')
    report = {'started_at': datetime.now(timezone.utc).isoformat(), 'sensor_id': args.sensor_id,
              'scene': args.scene, 'prf_trials': args.prf_trials, 'windows': [],
              'restoration': 'not_needed'}

    def save():
        temporary = args.output.with_suffix(args.output.suffix + '.tmp')
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
        temporary.replace(args.output)

    # Keep the ROS context usable in finally when an experiment is interrupted.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('umrr96_measure')
    measurements = Measurements()
    prefix = args.topic_prefix.rstrip('/')
    for message_type, suffix, callback in (
            (PointCloud2, '/port_targets_0', measurements.cloud),
            (PortTargetHeader, '/port_targetheader_0', measurements.header),
            (RadarTiming, '/timing_0', measurements.timing),
            (Umrr96RawQuality, '/umrr96_raw_quality_0', measurements.raw_quality)):
        node.create_subscription(message_type, prefix + suffix, callback,
                                 qos_profile_sensor_data)
    control = Control(node, args.sensor_id, args.control_prefix.rstrip('/'))

    def interrupt(signum, frame):
        if report['restoration'] == 'restoring':
            return
        # A second signal must not interrupt restoration.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def observe():
        end = time.monotonic() + 2  # Drain pre-change frames and allow settings to settle.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
        measurements.reset()
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
        result = measurements.result(time.monotonic() - start)
        summary = ('frames', 'observed_hz', 'detections_per_frame')
        print(json.dumps({k: result[k] for k in summary}), flush=True)
        return result

    try:
        report['statuses'] = control.read(status=True)
        report['parameters_before'] = control.read()
        report['windows'].append(dict(label='initial', **observe()))
        save()
        if not report['windows'][0]['frames']:
            raise RuntimeError('No target frames received')
        if args.prf_trials:
            prf_trials(control, observe, report, save)
        report['parameters_after'] = control.read()
        report['success'] = True
    except (Exception, KeyboardInterrupt) as error:
        report['success'] = False
        report['error'] = str(error) or type(error).__name__
        raise
    finally:
        save()
        node.destroy_node()
        rclpy.shutdown()
        print(f'Report: {args.output}; restoration: {report["restoration"]}', flush=True)


if __name__ == '__main__':
    main()
