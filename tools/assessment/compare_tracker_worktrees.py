# SPDX-License-Identifier: Apache-2.0
"""Compare isolated tracker worktrees on identical saved UMRR measurements."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
import yaml

from assess_ghost_dataset import REPO, sha256_file
from smartmicro_processing.cloud import measurements, select_measurements, GateConfig
from smartmicro_processing.doppler import fit_velocity, FitConfig
from smartmicro_processing.ghosts import ghost_reasons, ghost_rejection_mask, GhostConfig
from smartmicro_processing.obstacles import near_tracks

PHASES = {'walking': (6, 40), 'standing': (42, 50), 'swaying': (50, 56),
          'absent': (64, 80)}


def load_tracker(name, repo, overrides=None):
    path = repo / 'smartmicro_processing/smartmicro_processing/tracker.py'
    spec = importlib.util.spec_from_file_location('experiment_' + name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    config_path = repo / 'smartmicro_processing/config/umrr96_processing.yaml'
    params = yaml.safe_load(config_path.read_text())['/**/umrr96_processing']['ros__parameters']
    config = module.TrackerConfig(**{k: params[k] for k in asdict(module.TrackerConfig()) if k in params})
    for key, value in (overrides or {}).items():
        setattr(config, key, value)
    config.__post_init__()
    return module.MovingObjectTracker(config), {'repository': str(repo), 'config': asdict(config),
                                              'source_sha256': sha256_file(path),
                                              'yaml_sha256': sha256_file(config_path)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--worktrees', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--check-disabled', action='store_true')
    parser.add_argument('--unlabeled', action='store_true', help='Disable room phases for other recordings')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a fresh output')
    names = ('baseline', 'confirmation', 'standing', 'association', 'long_static_hold_control',
             'association_position_only_control', 'association_innovation_control')
    if args.check_disabled:
        names += ('confirmation_disabled', 'standing_disabled', 'association_disabled')
    trackers, provenance = {}, {}
    for name in names:
        repo = (REPO if name in ('baseline', 'long_static_hold_control') else args.worktrees /
                ('smartmicro-' + name.split('_')[0]))
        override = ({'static_hold': 30.} if name == 'long_static_hold_control' else
                    {'association_uncertainty': True, 'association_doppler': False}
                    if name == 'association_position_only_control' else
                    {'association_uncertainty': True, 'association_doppler': True}
                    if name == 'association_innovation_control' else
                    {'evidence_confirmation': False} if name == 'confirmation_disabled' else
                    {'standing_support': False} if name == 'standing_disabled' else
                    {'joint_association': False} if name == 'association_disabled' else None)
        trackers[name], provenance[name] = load_tracker(name, repo, override)
    params_path = REPO / 'smartmicro_processing/config/umrr96_processing.yaml'
    params = yaml.safe_load(params_path.read_text())['/**/umrr96_processing']['ros__parameters']

    def config(kind):
        return kind(**{k: params[k] for k in asdict(kind()) if k in params})

    fit_config, gate, ghosts = config(FitConfig), config(GateConfig), config(GhostConfig)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/smart_radar/port_targets_0']))
    tallies = {n: {p: Counter() for p in PHASES} for n in names}
    times = {n: [] for n in names}
    hashes = {n: hashlib.sha256() for n in names}
    total = Counter()
    first = previous = None
    fast_scans = 0
    for n in names:
        tallies[n]['all_unlabeled'] = Counter()
    while reader.has_next():
        _, data, _ = reader.read_next()
        cloud = deserialize_message(data, PointCloud2)
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        if previous is not None and stamp <= previous:
            raise ValueError('Nonmonotonic input stamps')
        first = stamp if first is None else first
        previous = stamp
        phase = (None if args.unlabeled else
                 next((p for p, (lo, hi) in PHASES.items() if lo <= stamp-first < hi), None))
        values = measurements(cloud)
        indices, _ = select_measurements(values, gate)
        selected = values[indices]
        fit = fit_velocity(selected[:, :3], selected[:, 3], fit_config)
        total.update(scans=1, raw_points=len(values), valid_fits=int(fit.valid))
        if fit.valid:
            fast_scans = fast_scans + 1 if np.linalg.norm(fit.velocity) > params['sensor_moving_speed'] else 0
            xyz, speed = selected[~fit.inliers, :3], fit.residuals[~fit.inliers]
            static = selected[fit.inliers, :3]
            rejected = ghost_rejection_mask(ghost_reasons(xyz, speed, static, ghosts),
                                             reject_static_only=params['reject_static_only'])
            kept, retained_speed = xyz[~rejected], speed[~rejected]
        else:
            kept = np.empty((0, 3))
        for name, tracker in trackers.items():
            start = time.perf_counter_ns()
            visible = (tracker.step(stamp, kept, retained_speed, static, sensor_moving=fast_scans >= 3)
                       if fit.valid else tracker.coast(stamp))
            times[name].append((time.perf_counter_ns()-start)/1e6)
            serial = [[t.track_id, t.x.tolist(), t.P.tolist(), t.last_moving,
                       t.last_support, t.confirmed, t.ghost] for t in tracker.tracks]
            hashes[name].update(json.dumps(serial, allow_nan=False).encode())
            for track in tracker.tracks:
                if not np.isfinite(track.x).all() or not np.isfinite(track.P).all():
                    raise ValueError('Nonfinite state: ' + name)
            count = tallies[name]['all_unlabeled']
            count.update(scans=1, visible_track_instances=len(visible))
            if phase:
                count = tallies[name][phase]
                count['scans'] += 1
                count['any_track_present'] += bool(visible)
                count['near_track_present'] += any(np.linalg.norm(t.x[:2]) < 3. for t in visible)
                count['far_track_present'] += any(np.linalg.norm(t.x[:2]) > 3.4 for t in visible)
                supported = [t for t in visible if t.last_support == stamp]
                count['current_supported_near_track_present'] += any(np.linalg.norm(t.x[:2]) < 3. for t in supported)
                if phase != 'absent':
                    on_track = near_tracks(kept, [t.x[:2] for t in visible], params['track_radius'])
                    ranges = np.linalg.norm(kept, axis=1)
                    count['direct_proxy_retained_on_track'] += int((on_track & (ranges < 3.)).sum())
                    count['ghost_proxy_retained_on_track'] += int((on_track & (ranges > 3.4)).sum())
    if not total['scans']:
        raise ValueError('No target scans')
    if args.check_disabled:
        for name in ('confirmation_disabled', 'standing_disabled', 'association_disabled'):
            if hashes[name].hexdigest() != hashes['baseline'].hexdigest():
                raise ValueError('Disabled experiment changed tracker states: ' + name)
    report = {'scope': 'Full independent tracker rollouts on identical point-stage outputs; current-scan display not accumulated.',
              'limits': 'Single previously examined room recording, range proxies rather than independent annotations; no DDS, acquisition/render latency or obstacle evaluation.',
              'absent_phase_limit': 'Interior of historical 62-82 s absence interval, trimmed to 64-80 s; no camera annotation. Report all tracks, not identity accuracy.',
              'phases_s': {} if args.unlabeled else PHASES, 'totals': dict(total),
              'disabled_state_equivalence_checked': args.check_disabled,
              'point_config': {'fit': asdict(fit_config), 'gate': asdict(gate), 'ghost': asdict(ghosts)},
              'variants': {n: {**provenance[n], 'phases': {p: dict(c) for p, c in tallies[n].items()},
                               'state_digest': hashes[n].hexdigest(),
                               'tracker_ms': {key: float(np.percentile(times[n], q)) for key, q in [('p50',50),('p95',95),('p99',99),('max',100)]}}
                           for n in names},
              'input_sha256': {str(p): sha256_file(p) for p in sorted(args.bag.glob('*.mcap'))},
              'harness_sha256': sha256_file(Path(__file__))}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    for n, v in report['variants'].items():
        print(n, json.dumps(v['phases']))


if __name__ == '__main__':
    main()
