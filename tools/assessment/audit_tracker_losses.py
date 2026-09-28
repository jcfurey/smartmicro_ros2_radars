# SPDX-License-Identifier: Apache-2.0
"""Locate tracker bottlenecks without changing the current point/track decisions.

The joint assignment is a one-step, maximum-cardinality diagnostic within the
existing distance gate and confirmed-first priority. It is not a replacement
tracker or a trajectory-accuracy score. Run on the documented UMRR walk only.
"""
import argparse
from collections import Counter
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from scipy.optimize import linear_sum_assignment
import yaml

from assess_ghost_dataset import REPO, sha256_file
from sensor_msgs.msg import PointCloud2
from smartmicro_processing.cloud import measurements, select_measurements, GateConfig
from smartmicro_processing.doppler import fit_velocity, FitConfig
from smartmicro_processing.ghosts import ghost_reasons, ghost_rejection_mask, GhostConfig
from smartmicro_processing.obstacles import near_tracks
from smartmicro_processing.tracker import MovingObjectTracker, Track, TrackerConfig

PHASES = {'walking': (6, 40), 'standing': (42, 50), 'swaying': (50, 56)}


def assignments(tracks, observations, gate, joint=False):
    """Keep confirmed-first priority; compare greedy with joint assignment per tier."""
    used, result = set(), {}
    order = sorted(tracks, key=lambda t: (not t.confirmed, t.track_id))
    if not joint:
        for t in order:
            best, distance = None, gate
            for i, (position, _, _) in enumerate(observations):
                d = np.linalg.norm(position - t.x[:2])
                if i not in used and d < distance:
                    best, distance = i, d
            if best is not None:
                used.add(best)
                result[t.track_id] = best
        return result
    for confirmed in (True, False):
        tier = [t for t in order if t.confirmed == confirmed]
        available = [i for i in range(len(observations)) if i not in used]
        if not tier or not available:
            continue
        distances = np.array([[np.linalg.norm(observations[i][0] - t.x[:2])
                               for i in available] for t in tier])
        miss = len(tier) + 1.
        costs = np.full((len(tier), len(available) + len(tier)), miss)
        costs[:, :len(available)] = np.where(distances < gate, distances / gate, miss ** 3)
        rows, columns = linear_sum_assignment(costs)
        for r, c in zip(rows, columns):
            if c < len(available):
                assert distances[r, c] < gate
                result[tier[r].track_id] = available[c]
                used.add(available[c])
    return result


class ObservedTracker(MovingObjectTracker):
    """Record association opportunities while running the unchanged tracker."""

    def measurements(self, xyz, speed):
        result = super().measurements(xyz, speed)
        self.greedy = assignments(self.tracks, result, self.config.gate)
        self.joint = assignments(self.tracks, result, self.config.gate, joint=True)
        self.expected_updates = {k: result[v][0].copy() for k, v in self.greedy.items()}
        self.actual_updates = {}
        self.association_tracks = len(self.tracks)
        self.cluster_count = len(result)
        return result

    def update(self, track, position, radial_speed):
        self.actual_updates[track.track_id] = position.copy()
        super().update(track, position, radial_speed)

    def verify(self):
        assert self.actual_updates.keys() == self.expected_updates.keys()
        for key in self.actual_updates:
            np.testing.assert_array_equal(self.actual_updates[key], self.expected_updates[key])


def seeded_track(track_id, xy, speed=0.0):
    return Track(track_id, np.r_[xy, 0., 0.], np.eye(4) * .04, 0., 0., 0.,
                 [True] * 10, confirmed=True, hits=10, first_stamp=-1., radial_speed=speed)


def counterexamples():
    tracker = ObservedTracker()
    tracker.tracks = [seeded_track(1, [2., 0.]), seeded_track(2, [3., 0.])]
    tracker.next_id = 3
    tracker.step(.055, [[2.4, 0., 0.], [1.4, 0., 0.]], [-.5, -.5])
    tracker.verify()
    assert len(tracker.greedy) == 1 and len(tracker.joint) == 2
    association = {'predicted_xy': [[2., 0.], [3., 0.]],
                   'measurement_xy': [[2.4, 0.], [1.4, 0.]],
                   'gate_m': tracker.config.gate,
                   'greedy': tracker.greedy, 'joint': tracker.joint,
                   'extra_tentative_tracks_after_actual_step': sum(not t.confirmed for t in tracker.tracks)}
    tracker = MovingObjectTracker()
    near = seeded_track(1, [2., 0.], .5)
    far = seeded_track(2, [4., 4.], -.5)
    tracker.tracks = [near, far]
    assert tracker.is_ghost(far)
    ghost = {'near_xy': near.x[:2].tolist(), 'far_xy': far.x[:2].tolist(),
             'radial_speeds': [.5, -.5], 'bearing_separation_deg': 45.,
             'confirmed_far_track_hidden': True,
             'scope': 'Track-stage counterexample with supplied detections, not end-to-end retained data.'}
    merged = tracker.measurements([[2., -.2, 0.], [2., .2, 0.]], [.5, -.5])
    assert len(merged) == 1 and merged[0][1] == 0.
    return {'greedy_assignment': association, 'opposite_motion_ghost': ghost,
            'cluster_cancellation': {'two_measurements_apart_m': .4,
                                     'input_radial_speeds': [.5, -.5],
                                     'cluster_count': 1, 'mean_radial_speed': 0.},
            'limit': 'Synthetic isolated mechanisms, not measured multi-person accuracy.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a fresh output path')
    config_path = REPO / 'smartmicro_processing/config/umrr96_processing.yaml'
    params = yaml.safe_load(config_path.read_text())['/**/umrr96_processing']['ros__parameters']

    def config(kind):
        return kind(**{k: params[k] for k in asdict(kind()) if k in params})

    gate, fit_config, ghosts, track_config = [config(k) for k in
                                           (GateConfig, FitConfig, GhostConfig, TrackerConfig)]
    tracker = ObservedTracker(track_config)
    confirmation_trackers = {
        'current': tracker,
        'six_of_eight': MovingObjectTracker(replace(track_config, confirm_hits=6, confirm_window=8)),
        'four_of_five': MovingObjectTracker(replace(track_config, confirm_hits=4, confirm_window=5)),
    }
    confirmation_counts = {key: {p: Counter() for p in PHASES} for key in confirmation_trackers}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/smart_radar/port_targets_0']))
    counts = {phase: Counter() for phase in (*PHASES, 'whole_bag_unlabeled')}
    first = previous = None
    fast_scans = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        cloud = deserialize_message(data, PointCloud2)
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        if previous is not None and stamp <= previous:
            raise ValueError('Nonmonotonic input stamps')
        previous = stamp
        first = stamp if first is None else first
        phase = next((p for p, (lo, hi) in PHASES.items() if lo <= stamp-first < hi), None)
        values = measurements(cloud)
        indices, _ = select_measurements(values, gate)
        selected = values[indices]
        fit = fit_velocity(selected[:, :3], selected[:, 3], fit_config)
        stats = Counter(scans=1, raw_points=len(values), quality_points=len(indices))
        if fit.valid:
            fast_scans = fast_scans + 1 if np.linalg.norm(fit.velocity) > params['sensor_moving_speed'] else 0
            xyz, speed = selected[~fit.inliers, :3], fit.residuals[~fit.inliers]
            static = selected[fit.inliers, :3]
            flags = ghost_reasons(xyz, speed, static, ghosts)
            rejected = ghost_rejection_mask(flags, reject_static_only=params['reject_static_only'])
            kept = xyz[~rejected]
            visible = tracker.step(stamp, kept, speed[~rejected], static, sensor_moving=fast_scans >= 3)
            stats.update(movers=len(xyz), point_ghosts=int(rejected.sum()), retained_movers=len(kept))
            if phase:
                ranges = np.linalg.norm(xyz, axis=1)
                for label, chosen in [('direct_proxy', ranges < 3.), ('ghost_proxy', ranges > 3.4)]:
                    stats[label + '_movers'] += int(chosen.sum())
                    stats[label + '_point_rejected'] += int((chosen & rejected).sum())
                    for point in xyz[chosen & ~rejected]:
                        nearby = [t for t in tracker.tracks if
                                  np.linalg.norm(point[:2] - t.x[:2]) < params['track_radius']]
                        category = ('visible_confirmed' if any(t.confirmed and not t.ghost for t in nearby)
                                    else 'ghost_hidden' if any(t.confirmed and t.ghost for t in nearby)
                                    else 'tentative' if nearby else 'no_nearby_track')
                        stats[label + '_retained_' + category] += 1
        else:
            stats['failed_fits'] += 1
            kept = np.empty((0, 3))
            visible = tracker.coast(stamp)
        tracker.verify()
        for key, alternative in confirmation_trackers.items():
            if key == 'current':
                confirmed = visible
            elif fit.valid:
                confirmed = alternative.step(stamp, kept, speed[~rejected], static,
                                             sensor_moving=fast_scans >= 3)
            else:
                confirmed = alternative.coast(stamp)
            if phase:
                tally = confirmation_counts[key][phase]
                tally['scans'] += 1
                tally['person_proxy_track_present'] += any(np.linalg.norm(t.x[:2]) < 3. for t in confirmed)
                tally['far_track_present'] += any(np.linalg.norm(t.x[:2]) > 3.4 for t in confirmed)
                on_track = near_tracks(kept, [t.x[:2] for t in confirmed], params['track_radius'])
                ranges = np.linalg.norm(kept, axis=1)
                tally['direct_proxy_retained_on_track'] += int((on_track & (ranges < 3.)).sum())
                tally['ghost_proxy_retained_on_track'] += int((on_track & (ranges > 3.4)).sum())
        stats['greedy_hits'] += len(tracker.greedy)
        stats['joint_one_step_hits'] += len(tracker.joint)
        stats['frames_with_multiple_existing_tracks'] += tracker.association_tracks > 1
        stats['frames_with_joint_extra_hit'] += len(tracker.joint) > len(tracker.greedy)
        stats['frames_with_different_assignment'] += tracker.joint != tracker.greedy
        for t in tracker.tracks:
            if t.confirmed and t.ghost:
                stats['hidden_track_instances'] += 1
                stats['hidden_track_instances_with_current_moving_hit'] += t.last_moving == stamp
        if phase:
            stats['person_proxy_track_present'] += any(np.linalg.norm(t.x[:2]) < 3. for t in visible)
            stats['far_track_present'] += any(np.linalg.norm(t.x[:2]) > 3.4 for t in visible)
            counts[phase].update(stats)
        # Exclude proxy labels from whole-bag numbers outside the scored phases.
        counts['whole_bag_unlabeled'].update({k: v for k, v in stats.items()
                                             if not ('proxy' in k or k == 'far_track_present')})
    if first is None:
        raise ValueError('No raw target scans')
    sources = [Path(__file__), config_path, *sorted(args.bag.glob('*.mcap')),
               *[REPO / 'smartmicro_processing/smartmicro_processing' / name
                 for name in ('tracker.py', 'ghosts.py', 'doppler.py', 'cloud.py', 'obstacles.py')]]
    report = {'scope': 'Unmodified tracker, current YAML point policy; no live sensor, DDS or obstacle evaluation.',
              'limits': 'Room-range proxies only in documented present phases; joint matches are one-step possibilities, not evaluated trajectories or identities.',
              'phases_s': PHASES, 'counts': {k: dict(v) for k, v in counts.items()},
              'confirmation_exploration': {
                  'protocol': 'Exploratory shorter windows on the same opened bag; all other settings, inputs and point decisions fixed; no new holdout.',
                  'variants': {key: {'confirm_hits': tr.config.confirm_hits,
                                     'confirm_window': tr.config.confirm_window,
                                     'phases': {p: dict(v) for p, v in confirmation_counts[key].items()}}
                               for key, tr in confirmation_trackers.items()}},
              'config': {'gate': asdict(gate), 'fit': asdict(fit_config), 'ghost': asdict(ghosts),
                         'tracker': asdict(track_config), 'reject_static_only': params['reject_static_only'],
                         'track_radius': params['track_radius'], 'sensor_moving_speed': params['sensor_moving_speed']},
              'counterexamples': counterexamples(),
              'source_sha256': {str(p): sha256_file(p) for p in sources}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report['counts'], indent=2))
    print(json.dumps(report['confirmation_exploration'], indent=2))


if __name__ == '__main__':
    main()
