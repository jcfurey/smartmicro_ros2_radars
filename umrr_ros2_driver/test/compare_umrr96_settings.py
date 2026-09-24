#!/usr/bin/env python3
"""Compare temporary UMRR-96 settings and restore the starting values.

Requires umrr96_live.launch.py. This hardware experiment sends parameter writes;
it never sends EEPROM-save or reset commands. Keep the scene still for comparison.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import statistics
import struct
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from umrr_ros2_msgs.msg import PortTargetHeader
from umrr_ros2_msgs.srv import GetMode, SetMode


PARAMETERS = ['frequency_sweep_idx', 'range_toggle_mode', 'tx_antenna_idx',
              'output_control_target_list_can']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-id', type=int, default=230739)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--baseline-seconds', type=float, default=10)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seconds < 5 or args.baseline_seconds < 5:
        parser.error('Measurement windows must be at least five seconds')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report = {'started_at': datetime.now(timezone.utc).isoformat(),
              'sensor_id': args.sensor_id, 'persisted': False, 'restored': False,
              'trials': [], 'writes': []}
    rclpy.init()
    node = rclpy.create_node('umrr96_setting_comparison')
    # Restore through ROS before shutting down the context on Ctrl-C or SIGTERM.
    def interrupted(signum, frame):
        raise KeyboardInterrupt(f'Signal {signum}; restoring sensor settings')
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    getter = node.create_client(GetMode, '/smart_radar/get_radar_mode')
    setter = node.create_client(SetMode, '/smart_radar/set_radar_mode')
    frames, headers = [], []
    collecting = False

    def receive_cloud(cloud):
        if not collecting:
            return
        offsets = {field.name: field.offset for field in cloud.fields}
        fmt = '>f' if cloud.is_bigendian else '<f'
        ranges, snrs, azimuths = [], [], []
        for row in range(cloud.height):
            for column in range(cloud.width):
                base = row * cloud.row_step + column * cloud.point_step
                distance = struct.unpack_from(fmt, cloud.data, base + offsets['range'])[0]
                if math.isfinite(distance):
                    ranges.append(distance)
                snrs.append(struct.unpack_from(fmt, cloud.data, base + offsets['snr'])[0])
                azimuths.append(struct.unpack_from(
                    fmt, cloud.data, base + offsets['azimuth_angle'])[0])
        frames.append({'time': time.monotonic(), 'count': cloud.width * cloud.height,
                       'near1': sum(0 <= value < 1 for value in ranges),
                       'near3': sum(0 <= value < 3 for value in ranges),
                       'near5': sum(0 <= value < 5 for value in ranges),
                       'ranges': ranges, 'snrs': snrs, 'azimuths': azimuths})

    def receive_header(header):
        if collecting:
            headers.append(header)

    node.create_subscription(PointCloud2, '/smart_radar/port_targets_0',
                             receive_cloud, qos_profile_sensor_data)
    node.create_subscription(PortTargetHeader, '/smart_radar/port_targetheader_0',
                             receive_header, qos_profile_sensor_data)

    def save():
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=max(0, min(.1, deadline - time.monotonic())))

    def call(client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError('ROS control service timed out')
        result = json.loads(future.result().res)
        if not result['success']:
            raise RuntimeError(json.dumps(result))
        return result

    def read():
        response = call(getter, GetMode.Request(
            sensor_id=args.sensor_id, section_name='auto_interface_0dim',
            params=PARAMETERS, param_types=[3] * len(PARAMETERS)))
        return {name: response['values'][name]['value'] for name in PARAMETERS}

    def apply(settings):
        current = read()
        changes = {name: value for name, value in settings.items() if current[name] != value}
        if changes:
            event = {'requested': changes, 'at': datetime.now(timezone.utc).isoformat()}
            report['writes'].append(event)
            save()
            try:
                event['response'] = call(setter, SetMode.Request(
                    sensor_id=args.sensor_id, section_name='auto_interface_0dim',
                    params=list(changes), values=[str(value) for value in changes.values()],
                    value_types=[3] * len(changes)))
            except Exception as error:
                event['error'] = str(error)
                save()
                raise
            spin(2)
        actual = read()
        if any(actual[name] != value for name, value in settings.items()):
            raise RuntimeError(f'Readback mismatch: expected {settings}, got {actual}')
        return actual

    def measure(label, settings, seconds):
        nonlocal collecting
        print(f'Measuring {label} for {seconds:g}s: {settings}', flush=True)
        frames.clear()
        headers.clear()
        collecting = True
        try:
            spin(seconds)
        finally:
            collecting = False
        if len(frames) < 2 or not headers:
            raise RuntimeError(f'{label}: live target stream missing')
        all_ranges = [value for frame in frames for value in frame['ranges']]
        snrs = [value for frame in frames for value in frame['snrs'] if math.isfinite(value)]
        azimuths = [value for frame in frames for value in frame['azimuths']
                    if math.isfinite(value)]
        result = {
            'label': label, 'settings': settings, 'seconds': seconds, 'frames': len(frames),
            'cloud_hz': (len(frames) - 1) / (frames[-1]['time'] - frames[0]['time']),
            'mean_targets': statistics.mean(frame['count'] for frame in frames),
            'min_targets': min(frame['count'] for frame in frames),
            'max_targets': max(frame['count'] for frame in frames),
            'mean_targets_under_1m': statistics.mean(frame['near1'] for frame in frames),
            'mean_targets_under_3m': statistics.mean(frame['near3'] for frame in frames),
            'mean_targets_under_5m': statistics.mean(frame['near5'] for frame in frames),
            'minimum_range_m': min(all_ranges) if all_ranges else None,
            'maximum_range_m': max(all_ranges) if all_ranges else None,
            'median_snr': statistics.median(snrs) if snrs else None,
            'azimuth_limits_rad': [min(azimuths), max(azimuths)] if azimuths else None,
            'mean_sensor_cycle_time': statistics.mean(header.cycle_time for header in headers),
        }
        # The UMRR-96 callback does not populate the generic ROS header's antenna
        # and sweep indices. Their zero defaults are not sensor feedback.
        report['trials'].append(result)
        save()
        print(json.dumps(result), flush=True)
        return result

    baseline = None
    try:
        assert getter.wait_for_service(timeout_sec=10), 'Read service unavailable'
        assert setter.wait_for_service(timeout_sec=10), 'Tuning service unavailable'
        baseline = report['baseline_settings'] = read()
        save()
        spin(2)
        measure('baseline_start', baseline, args.seconds)
        for label, changes in (
            ('can_targets_disabled', {'output_control_target_list_can': 0}),
            ('antenna_1', {'tx_antenna_idx': 1}),
            ('antenna_2', {'tx_antenna_idx': 2}),
            ('sweep_512mhz', {'frequency_sweep_idx': 1}),
        ):
            try:
                settings = apply(dict(baseline, **changes))
                measure(label, settings, args.seconds)
            except Exception as error:
                report['trials'].append({'label': label, 'error': str(error)})
                print(f'{label} failed: {error}', flush=True)
                save()
            finally:
                restored = apply(baseline)
            measure(f'baseline_after_{label}', restored, args.baseline_seconds)
    finally:
        if baseline is not None:
            try:
                report['final_readback'] = apply(baseline)
                report['restored'] = True
                print('Starting sensor settings restored and verified.', flush=True)
            except Exception as error:
                report['restore_error'] = str(error)
                print(f'RESTORATION FAILED: {error}', flush=True)
        report['finished_at'] = datetime.now(timezone.utc).isoformat()
        save()
        node.destroy_node()
        rclpy.shutdown()
    if not report['restored']:
        raise RuntimeError('Sensor baseline was not restored; inspect the report')


if __name__ == '__main__':
    main()
