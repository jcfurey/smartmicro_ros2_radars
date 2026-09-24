#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Offline UMRR-96 evidence audit. No ROS node, sensor writes, or pose publication.

Source ROS and this driver's build before running. The planar Doppler fit is
an exploratory consistency check, not calibrated ego velocity or ground truth.
"""

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[2] / 'umrr_ros2_driver' / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from umrr96_filter import DetectionFilter  # noqa: E402


def distribution(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    result = {'count': int(values.size), 'finite': int(finite.size)}
    for label, percentile in [('min', 0), ('p05', 5), ('median', 50), ('p95', 95), ('max', 100)]:
        result[label] = float(np.percentile(finite, percentile)) if finite.size else None
    result['mean'] = float(np.mean(finite)) if finite.size else None
    return result


def geometry_condition(matrix):
    """Condition of the direction matrix, not its squared normal matrix."""
    if len(matrix) < matrix.shape[1] or not np.isfinite(matrix).all():
        return float('inf')
    singular = np.linalg.svd(matrix, compute_uv=False)
    return float(singular[0] / singular[-1]) if singular[-1] > 1e-9 else float('inf')


def planar_fit(directions, doppler, rng, threshold=.30, trials=64):
    """Assume positive receding Doppler and zero radar vertical velocity.

    RANSAC rejects gross residuals, followed by unweighted least squares.
    Values/thresholds are analysis assumptions, not calibrated sensor claims.
    """
    matrix = np.asarray(directions, dtype=float)[:, :2]
    doppler = np.asarray(doppler, dtype=float)
    if (len(matrix) < 6 or not np.isfinite(doppler).all()
            or geometry_condition(matrix) > 100):
        return None
    best = None
    best_score = (-1, -float('inf'))
    for _ in range(trials):
        sample = rng.choice(len(matrix), 2, replace=False)
        if geometry_condition(matrix[sample]) > 100:
            continue
        velocity = np.linalg.lstsq(matrix[sample], -doppler[sample], rcond=None)[0]
        residual = np.abs(doppler + matrix @ velocity)
        inliers = residual <= threshold
        score = (int(inliers.sum()), -float(np.median(residual[inliers])))
        if score > best_score:
            best, best_score = inliers, score
    if best is None or best.sum() < 6 or geometry_condition(matrix[best]) > 100:
        return None
    velocity = np.linalg.lstsq(matrix[best], -doppler[best], rcond=None)[0]
    residual = doppler + matrix @ velocity
    return {'vx': float(velocity[0]), 'vy': float(velocity[1]),
            'consensus_fraction': float(np.mean(best)),
            'consensus_rmse': float(np.sqrt(np.mean(residual[best] ** 2))),
            'all_points_rmse': float(np.sqrt(np.mean(residual ** 2))),
            'consensus_condition': geometry_condition(matrix[best])}


def stamp_key(message):
    stamp = message.header.stamp
    return (message.header.frame_id, stamp.sec * 1_000_000_000 + stamp.nanosec)


def assess(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py.point_cloud2 import read_points

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    topics = Counter()
    raw, headers, timing, quality, filtered = {}, {}, {}, {}, {}
    diagnostic_history = defaultdict(list)
    states, cycles, setup, duplicate_keys = [], [], Counter(), Counter()
    ordered, receipts, layout = [], [], []
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        message = deserialize_message(data, types[topic])
        topics[topic] += 1
        target = None
        if topic == '/smart_radar/port_targets_0':
            target = raw
            ordered.append(stamp_key(message))
            receipts.append(receipt)
            layout = [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype,
                       'count': f.count} for f in message.fields]
        elif topic == '/smart_radar/filtered_targets_0':
            target = filtered
        elif topic == '/smart_radar/port_targetheader_0':
            target = headers
            cycles.append(message.cycle_time)
            if message.acquisition_setup_valid:
                setup[message.acquisition_setup] += 1
        elif topic == '/smart_radar/timing_0' and message.stream == 0:
            target = timing
        elif topic == '/smart_radar/umrr96_raw_quality_0':
            target = quality
        elif topic == '/smart_radar/filter_status':
            states.append(json.loads(message.data))
        elif topic == '/diagnostics':
            for status in message.status:
                level = status.level
                diagnostic_history[status.name].append({
                    'level': level[0] if isinstance(level, bytes) else int(level),
                    'message': status.message,
                    'values': {v.key: v.value for v in status.values}})
        if target is not None:
            key = stamp_key(message)
            duplicate_keys[topic] += int(key in target)
            target[key] = message
    if not raw:
        raise ValueError('No raw UMRR-96 clouds in the bag')
    if any(duplicate_keys.values()):
        raise ValueError(f'Duplicate frame/stamp keys need separate analysis: {duplicate_keys}')

    fields = defaultdict(list)
    counts, all_field_finite, core_finite = [], 0, 0
    planar_condition, spatial_condition, fits = [], [], []
    errors = defaultdict(list)
    filters = {mode: DetectionFilter(mode) for mode in ('off', 'quality', 'mapping', 'moving')}
    filter_counts = defaultdict(list)
    filter_stats = {mode: Counter() for mode in filters}
    snr_thresholds = {value: 0 for value in (0, 6, 12, 20, 30, 40, 50)}
    raw_pfa, raw_flags = [], Counter()
    matched, mismatches = Counter(), Counter()
    rng = np.random.default_rng(0)
    for key in ordered:
        message = raw[key]
        points = read_points(message, skip_nans=False).reshape(-1)
        count = len(points)
        counts.append(count)
        for name in points.dtype.names:
            fields[name].extend(points[name].tolist())
        xyz = np.column_stack([points[n] for n in ('x', 'y', 'z')]).astype(float)
        core = np.column_stack([points[n] for n in (
            'x', 'y', 'z', 'range', 'azimuth_angle', 'elevation_angle', 'radial_speed', 'snr')])
        core_finite += int(np.isfinite(core).all(axis=1).sum())
        all_field_finite += len(read_points(message, skip_nans=True))
        if count:
            errors['range_vs_xyz_m'].extend(np.abs(np.linalg.norm(xyz, axis=1) - points['range']))
            errors['snr_vs_power_minus_noise_db'].extend(np.abs(
                points['snr'] - (points['power'] - points['noise'])))
        for threshold in snr_thresholds:
            snr_thresholds[threshold] += int(np.sum(points['snr'] >= threshold))
        for mode, selector in filters.items():
            selected, stats = selector.select(points, key[1] / 1e9)
            filter_counts[mode].append(len(selected))
            filter_stats[mode].update({k: stats[k] for k in (
                'rejected_quality', 'rejected_motion', 'rejected_temporal')})
        valid, _ = DetectionFilter('quality').select(points, key[1] / 1e9)
        valid = [i for i in valid if np.isfinite(points['elevation_angle'][i])
                 and np.linalg.norm(xyz[i]) > 0]
        direction = xyz[valid] / np.linalg.norm(xyz[valid], axis=1)[:, None]
        planar_condition.append(geometry_condition(direction[:, :2]))
        spatial_condition.append(geometry_condition(direction))
        fit = planar_fit(direction, points['radial_speed'][valid], rng)
        if fit:
            fits.append(fit)
        if key in headers:
            matched['headers'] += 1
            mismatches['header_target_count'] += headers[key].number_of_targets != count
        if key in timing:
            matched['timing'] += 1
        if key in quality:
            matched['raw_quality'] += 1
            q = quality[key]
            mismatches['quality_array_length'] += (
                len(q.flags_raw) != count or len(q.false_alarm_probability_raw) != count)
            raw_pfa.extend(q.false_alarm_probability_raw)
            raw_flags.update(q.flags_raw)
        if key in filtered:
            matched['filtered'] += 1
            # Valid for this capture only when the reported filter retains all points.
            mismatches['filtered_bytes_differ'] += (
                message.data != filtered[key].data or message.width != filtered[key].width)

    stamps = np.array([k[1] for k in ordered], dtype=np.int64)
    span = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0
    device = [(key[1], timing[key].device_timestamp_us) for key in ordered if key in timing]
    deltas = np.diff(np.asarray(device, dtype=np.int64), axis=0)
    device_delta = deltas[:, 1] / 1e6 if len(deltas) else []
    delay_change = deltas[:, 0] / 1e9 - deltas[:, 1] / 1e6 if len(deltas) else []
    data_count = sum(counts)
    diagnostic_summary = {}
    for name, records in diagnostic_history.items():
        diagnostic_summary[name] = {'samples': len(records),
                                    'levels': dict(Counter(r['level'] for r in records)),
                                    'first': records[0], 'last': records[-1]}
    return {
        'bag': str(bag),
        'source_files': {p.name: {'bytes': p.stat().st_size,
                                'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                         for p in sorted(bag.iterdir()) if p.is_file()},
        'topics': dict(topics), 'cloud_layout': layout,
        'start_utc': datetime.fromtimestamp(stamps[0] / 1e9, timezone.utc).isoformat(),
        'end_utc': datetime.fromtimestamp(stamps[-1] / 1e9, timezone.utc).isoformat(),
        'raw_frames': len(raw), 'detections': data_count, 'span_seconds': span,
        'hz': (len(raw) - 1) / span if span > 0 else None,
        'detections_per_frame': distribution(counts),
        'all_core_fields_finite_detections': core_finite,
        'all_fields_skip_nans_true_retained': all_field_finite,
        'fields': {name: distribution(values) for name, values in fields.items()},
        'invariants': {name: distribution(values) for name, values in errors.items()},
        'metadata_matched_to_cloud': dict(matched), 'mismatches': dict(mismatches),
        'ros_stamp_nonincreasing_intervals': int(np.sum(np.diff(stamps) <= 0)),
        'device_nonincreasing_intervals': int(np.sum(np.asarray(device_delta) <= 0)),
        'receive_intervals_seconds': distribution(np.diff(stamps) / 1e9),
        'device_intervals_seconds': distribution(device_delta),
        'relative_delay_change_seconds': distribution(delay_change),
        'recorder_minus_cloud_stamp_seconds': distribution(
            (np.asarray(receipts, dtype=np.int64) - stamps) / 1e9),
        'reported_cycle_seconds': distribution(cycles), 'acquisition_setup_raw': dict(setup),
        'pfa_raw': distribution(raw_pfa), 'flags_raw': dict(raw_flags),
        'snr_threshold_retention': {str(t): n / data_count if data_count else None
                                    for t, n in snr_thresholds.items()},
        'existing_filters_offline': {mode: {
            'retained_fraction': sum(values) / data_count if data_count else None,
            'per_frame': distribution(values), 'rejections': dict(filter_stats[mode])}
            for mode, values in filter_counts.items()},
        'geometry_quality_gated': {'condition_U_xy': distribution(planar_condition),
                                  'condition_U_xyz': distribution(spatial_condition)},
        'exploratory_planar_doppler_fit': {
            'assumptions': ['positive receding sign UNVERIFIED', 'radar vertical velocity zero',
                            'dominant stationary scene', 'not ground truth or validated odometry'],
            'ransac_seed': 0, 'ransac_trials': 64, 'residual_threshold_mps': .30,
            'minimum_consensus': 6, 'maximum_condition_U_xy': 100,
            'successful_frames': len(fits),
            'distributions': {name: distribution([fit[name] for fit in fits])
                              for name in fits[0]} if fits else {}},
        'filter_status_first': states[0] if states else None,
        'filter_status_last': states[-1] if states else None,
        'diagnostics': diagnostic_summary,
        'limitations': ['No labeled targets or independent pose reference supplied',
                        'Capture was not a controlled stationary or motion experiment',
                        'No detection precision/recall or pose accuracy can be inferred',
                        'Delay variation and recorder delay are not acquisition latency',
                        'RCS/variance/Pfa/flag calibration is not established']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; choose a new report path')
    report = assess(args.bag)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: report[k] for k in ('raw_frames', 'detections', 'hz')}, indent=2))
    print(f'Report: {args.output}')


if __name__ == '__main__':
    main()
