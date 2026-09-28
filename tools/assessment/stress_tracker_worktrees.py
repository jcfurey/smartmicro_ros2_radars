# SPDX-License-Identifier: Apache-2.0
"""Controlled tracker-stage tests; synthetic truth is used only for scoring."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment

from compare_tracker_worktrees import load_tracker, REPO, sha256_file

NAMES = ('baseline', 'confirmation', 'standing', 'association', 'long_static_hold_control')


def make(name, roots):
    repo = REPO if name in ('baseline', 'long_static_hold_control') else roots / ('smartmicro-' + name)
    return load_tracker(name, repo, {'static_hold': 30.} if name == 'long_static_hold_control' else None)


def score_tracks(visible, truth, radius=.8):
    if not visible:
        return {}, 0
    distances = np.array([[np.linalg.norm(t.x[:2] - xy) for t in visible] for xy in truth])
    costs = np.full((len(truth), len(visible) + len(truth)), len(truth) + 1.)
    costs[:, :len(visible)] = np.where(distances < radius, distances / radius, 100.)
    rows, cols = linear_sum_assignment(costs)
    matches = {int(r): visible[c].track_id for r, c in zip(rows, cols)
               if c < len(visible) and distances[r, c] < radius}
    return matches, len(visible) - len(matches)


def crossing(roots):
    aggregate = {name: Counter() for name in NAMES}
    per_seed = []
    for seed in range(20):
        rng = np.random.default_rng(1000 + seed)
        trackers = {n: make(n, roots)[0] for n in NAMES}
        totals = {n: Counter() for n in NAMES}
        previous_ids = {n: {} for n in NAMES}
        for frame in range(200):
            stamp = frame * .055
            truth = np.array([[3., -1.6 + .4 * stamp], [3.9, 1.6 - .4 * stamp]])
            velocity = np.array([[0., .4], [0., -.4]])
            position = truth + rng.normal(0., .18, (2, 2))
            speed = np.sum(truth * velocity, axis=1) / np.linalg.norm(truth, axis=1)
            speed += rng.normal(0., .05, 2)
            selected = np.flatnonzero(rng.random(2) >= .1)
            rng.shuffle(selected)
            xyz = np.column_stack((position[selected], np.zeros(len(selected))))
            for name, tr in trackers.items():
                visible = tr.step(stamp, xyz, speed[selected])
                if stamp < 1.:
                    continue
                matches, false = score_tracks(visible, truth)
                totals[name].update(scored_frames=1, object_opportunities=2,
                                    matched_objects=len(matches), unmatched_track_instances=false)
                for obj, track_id in matches.items():
                    if obj in previous_ids[name] and previous_ids[name][obj] != track_id:
                        totals[name]['id_changes'] += 1
                    previous_ids[name][obj] = track_id
        for name in NAMES:
            aggregate[name].update(totals[name])
        per_seed.append({'seed': 1000 + seed, 'variants': {n: dict(v) for n, v in totals.items()}})
    return {'config': {'seeds': [1000, 1019], 'frames_per_seed': 200, 'dt_s': .055,
                       'position_std_m': .18, 'radial_speed_std_m_s': .05, 'drop_probability': .1,
                       'score_after_s': 1., 'score_radius_m': .8},
            'aggregate': {n: dict(v) for n, v in aggregate.items()}, 'per_seed': per_seed}


def standing(roots):
    report = {}
    for occluded in (False, True):
        values = {}
        for name in NAMES:
            tr, _ = make(name, roots)
            wall = [[2.6, 0., 0.]]
            dt = .055
            for k in range(100):
                tr.step(k * dt, [], [], wall)
            for k in range(100, 141):
                stop = [1.1 + .5 * (k - 100) * dt, 0., 0.]
                tr.step(k * dt, [stop], [.5], wall)
            count = Counter()
            for k in range(141, 381):
                visible = tr.step(k * dt, [], [], ([stop] if occluded else wall + [stop]))
                near = [t for t in visible if np.linalg.norm(t.x[:2] - stop[:2]) < .5]
                count['standing_scans'] += 1
                count['standing_covered'] += bool(near)
                count['standing_supported_now'] += any(t.last_support == k * dt for t in near)
            for k in range(381, 481):
                visible = tr.step(k * dt, [], [], wall)
                count['departure_scans'] += 1
                count['departure_track_present'] += bool(visible)
                if (k - 381) * dt > tr.config.max_coast:
                    count['departure_after_coast_scans'] += 1
                    count['departure_after_coast_track_present'] += bool(visible)
            values[name] = dict(count)
        report['occluded_wall' if occluded else 'visible_wall'] = values
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worktrees', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a fresh output')
    report = {'scope': 'Synthetic tracker-only measurements, no point rejection, ego fit or sensor model.',
              'limits': 'Controlled mechanisms, not a forecast of physical crossing performance. ID changes include reinitialization after gaps. No parameter search over seeds.',
              'sources': {n: make(n, args.worktrees)[1] for n in NAMES},
              'crossing': crossing(args.worktrees), 'standing': standing(args.worktrees),
              'harness_sha256': sha256_file(Path(__file__))}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'crossing': report['crossing']['aggregate'], 'standing': report['standing']}, indent=2))


if __name__ == '__main__':
    main()
