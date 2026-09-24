# SPDX-License-Identifier: Apache-2.0
"""Replay recorded raw targets through the same adapter and fitter as the ROS node."""
import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .cloud import GateConfig, measurements, select_measurements
from .doppler import FitConfig, fit_velocity


def summary(values):
    return {name: float(np.percentile(values, percentile)) if len(values) else None
            for name, percentile in [('min', 0), ('median', 50), ('p95', 95), ('max', 100)]}


def audit(bag, topic, fit_config, gate_config):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topics.get(topic) != 'sensor_msgs/msg/PointCloud2':
        raise ValueError(f'No PointCloud2 topic {topic}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    reasons, frames, totals = Counter(), Counter(), Counter()
    speeds, conditions, rmses, covariances, compute_ms = [], [], [], [], []
    while reader.has_next():
        _, data, _ = reader.read_next()
        cloud = deserialize_message(data, PointCloud2)
        start = time.perf_counter()
        frames[cloud.header.frame_id] += 1
        values = measurements(cloud)
        indices, stats = select_measurements(values, gate_config)
        totals.update(stats)
        selected = values[indices]
        fit = fit_velocity(selected[:, :3], selected[:, 3], fit_config)
        compute_ms.append(1000 * (time.perf_counter() - start))
        reasons[fit.reason] += 1
        if fit.valid:
            totals['inliers'] += int(fit.inliers.sum())
            totals['outliers'] += int((~fit.inliers).sum())
            speeds.append(float(np.linalg.norm(fit.velocity)))
            conditions.append(fit.condition)
            rmses.append(fit.rmse)
            covariances.append(float(np.sqrt(np.max(np.linalg.eigvalsh(fit.covariance)))))
        else:
            totals['unclassified'] += len(indices)
    if not frames:
        raise ValueError('No scans read')
    sources = []
    for path in sorted(bag.rglob('*')) if bag.is_dir() else [bag]:
        if path.is_file() and path.suffix in ('.mcap', '.db3', '.yaml'):
            with path.open('rb') as source:
                sources.append(dict(file=str(path), bytes=path.stat().st_size,
                                    sha256=hashlib.file_digest(source, 'sha256').hexdigest()))
    return dict(generated_at=datetime.now(timezone.utc).isoformat(), topic=topic, sources=sources,
                frames=dict(frames), scans=sum(frames.values()), reasons=dict(reasons),
                counts=dict(totals), fit_config=asdict(fit_config), gate_config=asdict(gate_config),
                fitted_speed_mps=summary(speeds), direction_condition=summary(conditions),
                residual_rmse_mps=summary(rmses), model_max_velocity_std_mps=summary(covariances),
                processing_ms=summary(compute_ms), independent_reference_available=False,
                interpretation='Numerical replay only; fit residuals are not velocity accuracy. '
                               'Doppler sign, timing, extrinsics and covariance are uncalibrated. '
                               'Historical replay does not exercise live freshness checks.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--topic', default='/smart_radar/port_targets_0')
    parser.add_argument('--doppler-sign', type=int, choices=(-1, 1), default=1)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; use a new report path')
    result = audit(args.bag, args.topic, FitConfig(doppler_sign=args.doppler_sign), GateConfig())
    with args.output.open('x') as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps(dict(scans=result['scans'], reasons=result['reasons'], counts=result['counts'])))


if __name__ == '__main__':
    main()
