# SPDX-License-Identifier: Apache-2.0
"""Conditional sensor-frame density comparison on a bag without calibrated poses."""
import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

from .accumulation import AccumulationConfig, TemporalEvidence
from .audit import summary
from .cloud import GateConfig, measurements, select_measurements
from .doppler import FitConfig, fit_velocity


def audit(bag, topic, windows, voxel_size):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    if {t.name: t.type for t in reader.get_all_topics_and_types()}.get(topic) != 'sensor_msgs/msg/PointCloud2':
        raise ValueError(f'No PointCloud2 topic {topic}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    evidence = [TemporalEvidence(AccumulationConfig(window_seconds=w, voxel_size=voxel_size))
                for w in windows]
    distributions = [dict(accumulated=[], confirmed=[], gain=[]) for _ in windows]
    fit_config, gate_config = FitConfig(), GateConfig()
    counts, frames = Counter(), Counter()
    instantaneous, points = [], []
    first = last = None
    while reader.has_next():
        _, data, _ = reader.read_next()
        cloud = deserialize_message(data, PointCloud2)
        stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
        if stamp <= 0 or (last is not None and stamp <= last):
            raise ValueError('Audit requires strictly increasing, positive timestamps')
        first = stamp if first is None else first
        last = stamp
        frames[cloud.header.frame_id] += 1
        if len(frames) != 1:
            raise ValueError('Sensor-frame audit requires a single consistent input frame')
        values = measurements(cloud)
        indices, stats = select_measurements(values, gate_config)
        fit = fit_velocity(values[indices, :3], values[indices, 3], fit_config)
        counts[fit.reason] += 1
        counts['input_points'] += stats['input']
        accepted = indices[fit.inliers] if fit.valid else np.empty(0, dtype=int)
        counts['doppler_inlier_points'] += len(accepted)
        cells_now = len({tuple(np.floor(v[:3] / voxel_size)) for v in values[accepted]})
        instantaneous.append(cells_now)
        points.append(len(accepted))
        for history, dist in zip(evidence, distributions):
            history.add(values[accepted], accepted, stamp, stamp)
            samples = history.snapshot(stamp)
            dist['accumulated'].append(len(samples))
            dist['confirmed'].append(sum(s['support_scans'] >= history.config.min_support_scans
                                         for s in samples))
            if cells_now:
                dist['gain'].append(len(samples) / cells_now)
    if last is None:
        raise ValueError('No scans read')

    def describe(data):
        return dict(mean=float(np.mean(data)) if len(data) else None, **summary(data))

    trials = []
    for history, dist in zip(evidence, distributions):
        expired = history.snapshot(last + round(history.config.window_seconds * 1e9))
        trials.append(dict(config=asdict(history.config),
                           accumulated_voxels=describe(dist['accumulated']),
                           confirmed_voxels=describe(dist['confirmed']),
                           accumulated_to_instantaneous_voxel_ratio=describe(dist['gain']),
                           capacity_drops=history.capacity_drops,
                           empty_after_last_scan_plus_window=not expired))
    sources = []
    for path in sorted(bag.rglob('*')) if bag.is_dir() else [bag]:
        if path.is_file() and path.suffix in ('.mcap', '.db3', '.yaml'):
            with path.open('rb') as stream:
                sources.append(dict(file=str(path), sha256=hashlib.file_digest(stream, 'sha256').hexdigest()))
    return dict(generated_at=datetime.now(timezone.utc).isoformat(),
                mode='stationary_preview', sensor_motion_compensated=False,
                stationary_assumption_verified_from_recording=False,
                independent_reference_available=False, sources=sources, input_topic=topic,
                frames=dict(frames), counts=dict(counts), duration_seconds=(last-first)*1e-9,
                fit_config=asdict(fit_config), gate_config=asdict(gate_config),
                instantaneous_inlier_points=describe(points), instantaneous_voxels=describe(instantaneous),
                windows=trials, interpretation='Sensor-frame comparison under a stationary-sensor '
                'assumption. Counts include noisy cell-boundary crossings and repeated evidence; '
                'they do not establish independent surface coverage, physical resolution or mapping accuracy.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--topic', default='/smart_radar/port_targets_0')
    parser.add_argument('--stationary-preview', action='store_true', required=True,
                        help='Explicitly accept an uncompensated sensor-frame diagnostic')
    parser.add_argument('--windows', nargs='+', type=float, default=[.3, .5, 1.0])
    parser.add_argument('--voxel-size', type=float, default=.25)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output exists; choose a new path')
    result = audit(args.bag, args.topic, args.windows, args.voxel_size)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(scans=result['frames'], instantaneous=result['instantaneous_voxels'],
                          windows=[dict(seconds=w['config']['window_seconds'],
                                        mean_voxels=w['accumulated_voxels']['mean'],
                                        mean_confirmed=w['confirmed_voxels']['mean'])
                                   for w in result['windows']])))
