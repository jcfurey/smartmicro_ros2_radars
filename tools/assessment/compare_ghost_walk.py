# SPDX-License-Identifier: Apache-2.0
"""Replay point criteria and the unchanged tracker on the documented UMRR walk.

Room-range proxy labels are used only while the person was documented present.
No live node, sensor command, point history display, or hardware tuning occurs.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import yaml

from compare_ghost_criteria import (
    RULES, REPO, compare_frame, counts, add_counts, with_rates, percentiles, sha256_file,
    GhostConfig, CopyEvidenceConfig)
from smartmicro_processing.cloud import measurements, select_measurements, GateConfig
from smartmicro_processing.doppler import fit_velocity, FitConfig
from smartmicro_processing.tracker import MovingObjectTracker, TrackerConfig

PHASES = {'walking': (6, 40), 'standing': (42, 50), 'swaying': (50, 56)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a new output path')
    config_path = REPO / 'smartmicro_processing/config/umrr96_processing.yaml'
    params = yaml.safe_load(config_path.read_text())['/**/umrr96_processing']['ros__parameters']

    def config(kind):
        return kind(**{key: params[key] for key in asdict(kind()) if key in params})

    fit_config, gate_config, tracker_config = config(FitConfig), config(GateConfig), config(TrackerConfig)
    legacy_config, candidate_config = config(GhostConfig), CopyEvidenceConfig()
    trackers = {rule: MovingObjectTracker(tracker_config) for rule in RULES}
    totals = {phase: {rule: Counter() for rule in RULES} for phase in (*PHASES, 'all_scored')}
    tracks = {phase: {rule: Counter() for rule in RULES} for phase in PHASES}
    reasons, stats = Counter(), Counter()
    elapsed = {'legacy': [], 'candidate_three_ablations': []}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/smart_radar/port_targets_0']))
    first_stamp = previous_stamp = None
    fast_scans = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        cloud = deserialize_message(data, PointCloud2)
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        if previous_stamp is not None and stamp <= previous_stamp:
            raise ValueError('Nonmonotonic input stamps')
        previous_stamp = stamp
        first_stamp = stamp if first_stamp is None else first_stamp
        values = measurements(cloud)
        indices, _ = select_measurements(values, gate_config)
        selected = values[indices]
        fit = fit_velocity(selected[:, :3], selected[:, 3], fit_config)
        stats['scans'] += 1
        stats['raw_points'] += len(values)
        reasons[fit.reason] += 1
        phase = next((p for p, (lo, hi) in PHASES.items() if lo <= stamp - first_stamp < hi), None)
        if fit.valid:
            fast_scans = fast_scans + 1 if np.linalg.norm(fit.velocity) > params['sensor_moving_speed'] else 0
            xyz = selected[~fit.inliers, :3]
            speed = fit.residuals[~fit.inliers]
            static = selected[fit.inliers, :3]
            masks, timing = compare_frame(xyz, speed, static, legacy_config, candidate_config)
            for key in elapsed:
                elapsed[key].append(timing[key])
            # No labels above this line, and none go to the tracker below.
            ranges = np.linalg.norm(xyz, axis=1)
            labels = np.full(len(xyz), 'unknown', dtype='U24')
            labels[ranges < 3.] = 'direct'
            labels[ranges > 3.4] = 'specific_ghost'
            for rule, rejected in masks.items():
                confirmed = trackers[rule].step(stamp, xyz[~rejected], speed[~rejected], static,
                                                sensor_moving=fast_scans >= 3)
                if phase:
                    metrics = counts(labels, rejected, np.ones(len(xyz), dtype=bool))
                    for cohort in (phase, 'all_scored'):
                        add_counts(totals[cohort][rule], metrics)
                    tracks[phase][rule]['scans'] += 1
                    tracks[phase][rule]['person_present'] += int(any(
                        np.linalg.norm(t.x[:2]) < 3. for t in confirmed))
                    tracks[phase][rule]['far_track_present'] += int(any(
                        np.linalg.norm(t.x[:2]) > 3.4 for t in confirmed))
        else:
            for rule, tracker in trackers.items():
                confirmed = tracker.coast(stamp)
                if phase:
                    tracks[phase][rule]['scans'] += 1
                    tracks[phase][rule]['person_present'] += int(any(
                        np.linalg.norm(t.x[:2]) < 3. for t in confirmed))
                    tracks[phase][rule]['far_track_present'] += int(any(
                        np.linalg.norm(t.x[:2]) > 3.4 for t in confirmed))
    if not stats['scans']:
        raise ValueError('No raw target scans')
    report = {
        'scope': 'Recorded UMRR point/track computation, no DDS, freshness or obstacle-layer evaluation',
        'labels': 'Room proxy: direct <3m; ghost >3.4m, only documented person-present phases; not independent annotation',
        'phases_s': PHASES, 'stats': dict(stats), 'fit_reasons': dict(reasons),
        'fit_config': asdict(fit_config), 'gate_config': asdict(gate_config),
        'tracker_config': asdict(tracker_config), 'legacy_config': asdict(legacy_config),
        'candidate_config': asdict(candidate_config),
        'point_metrics': {p: {rule: with_rates(v) for rule, v in rules.items()}
                          for p, rules in totals.items()},
        'unchanged_tracker_counts': {p: {rule: dict(v) for rule, v in rules.items()}
                                    for p, rules in tracks.items()},
        'timing_ms': {key: percentiles(values) for key, values in elapsed.items()},
        'source_sha256': {str(p): sha256_file(p) for p in [
            *sorted(args.bag.glob('*.mcap')), Path(__file__), config_path,
            Path(__file__).with_name('compare_ghost_criteria.py'),
            REPO / 'smartmicro_processing/smartmicro_processing/ghost_candidates.py',
            REPO / 'smartmicro_processing/smartmicro_processing/ghosts.py',
            REPO / 'smartmicro_processing/smartmicro_processing/tracker.py']},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report['point_metrics']['all_scored'], indent=2))


if __name__ == '__main__':
    main()
