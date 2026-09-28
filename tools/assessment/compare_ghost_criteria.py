# SPDX-License-Identifier: Apache-2.0
"""Compare frozen ghost criteria on a pilot or previously unused source scenes."""
import argparse
from dataclasses import asdict
import io
import json
from pathlib import Path
import time
import zipfile

import h5py
import numpy as np

from assess_ghost_dataset import REPO, decode_labels, sha256_file
from smartmicro_processing.ghosts import ghost_reasons, ghost_rejection_mask, GhostConfig
from smartmicro_processing.ghost_candidates import copy_evidence, CopyEvidenceConfig

RULES = ('none', 'legacy', 'no_static_only', 'signed_pair', 'aligned_pair', 'supported_copy')
DEVELOPMENT_SCENES = (11, 13, 21)


def counts(labels, rejected, eligible):
    result = {}
    for label in ('direct', 'specific_ghost', 'generic_ghost'):
        selected = eligible & (labels == label)
        result[label + '_total'] = int(selected.sum())
        result[label + '_rejected'] = int((selected & rejected).sum())
    return result


def add_counts(total, values):
    for key, value in values.items():
        total[key] = total.get(key, 0) + value


def with_rates(values):
    result = dict(values)
    for label in ('direct', 'specific_ghost', 'generic_ghost'):
        n = values[label + '_total']
        rejected = values[label + '_rejected']
        key = label + ('_retention' if label == 'direct' else '_rejection')
        result[key] = ((n - rejected if label == 'direct' else rejected) / n if n else None)
    return result


def compare_frame(xyz, speed, static, legacy_config, candidate_config):
    start = time.perf_counter_ns()
    reasons = ghost_reasons(xyz, speed, static, legacy_config)
    legacy = reasons != 0
    middle = time.perf_counter_ns()
    candidates = copy_evidence(xyz, speed, candidate_config)
    end = time.perf_counter_ns()
    masks = {'none': np.zeros(len(xyz), dtype=bool), 'legacy': legacy,
             'no_static_only': ghost_rejection_mask(reasons, reject_static_only=False),
             **candidates}
    return masks, {'legacy': (middle - start) / 1e6,
                   'candidate_three_ablations': (end - middle) / 1e6}


def evaluate(rows, legacy_config, candidate_config):
    # Same fixed-sensor adaptation as the committed pilot, before any labels.
    r, az, vr = (rows[k] for k in ('r_sc', 'phi_sc', 'vr_sc'))
    valid = (np.isfinite(r) & np.isfinite(az) & np.isfinite(vr) & (r >= .15) & (r <= 120))
    moving = valid & (np.abs(vr) > .05)
    masks = {rule: np.zeros(len(rows), dtype=bool) for rule in RULES}
    elapsed = {'legacy': [], 'candidate_three_ablations': []}
    sensor_frames = 0
    for sensor in np.unique(rows['sensor']):
        indices = np.flatnonzero(rows['sensor'] == sensor)
        indices = indices[np.argsort(rows['frame'][indices], kind='stable')]
        breaks = np.flatnonzero(np.diff(rows['frame'][indices])) + 1
        for group in np.split(indices, breaks):
            if not len(group):
                continue
            sensor_frames += 1
            selected = group[valid[group]]
            xyz = np.column_stack((r[selected] * np.cos(az[selected]),
                                   r[selected] * np.sin(az[selected]), np.zeros(len(selected))))
            movers = moving[selected]
            rejected, timing = compare_frame(xyz[movers], vr[selected[movers]], xyz[~movers],
                                            legacy_config, candidate_config)
            for rule in RULES:
                masks[rule][selected[movers]] = rejected[rule]
            for key in elapsed:
                elapsed[key].append(timing[key])
    labels = decode_labels(rows['label_id'], rows['group'])
    by_cohort = {}
    for cohort, eligible in [('moving', moving), ('all_valid', valid)]:
        by_cohort[cohort] = {rule: counts(labels, masks[rule], eligible) for rule in RULES}
    return {'rows': len(rows), 'sensor_frames': sensor_frames,
            'cohorts': by_cohort}, elapsed


def choose_sources(root, phase):
    if phase == 'pilot':
        return json.loads((root / 'sample-manifest.json').read_text())
    selected = []
    for archive in ('original.zip', 'virtual.zip'):
        index = json.loads((root / (archive + '.index.json')).read_text())
        for scene in range(1, 22):
            if scene in DEVELOPMENT_SCENES:
                continue
            members = [x for x in index if f'scenario-{scene:02d}_' in x['name']
                       and x['name'].endswith('.h5')]
            if not members:
                raise ValueError(f'Missing source scene: {scene}, {archive}')
            groups = [[x for x in members if f'_{kind}_' in x['name']]
                      for kind in ('cycl', 'ped')] if archive == 'original.zip' else [members]
            for group in groups:
                if not group:
                    continue  # Scene 09 has no original pedestrian recording.
                member = min(group, key=lambda x: (x['compressed_bytes'], x['name']))
                selected.append({**member, 'archive': archive,
                                 'selection': 'Smallest available member per original class or virtual scene'})
    return selected


def percentiles(values):
    return {name: float(np.percentile(values, q)) if values else None
            for name, q in [('p50', 50), ('p95', 95), ('p99', 99), ('max', 100)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', type=Path, required=True)
    parser.add_argument('--phase', choices=['pilot', 'held_out'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan_path = args.output.with_suffix('.selection.json')
    if args.output.exists() or plan_path.exists():
        parser.error('Choose new output and selection paths')
    sources = choose_sources(args.dataset_root, args.phase)
    legacy_config, candidate_config = GhostConfig(), CopyEvidenceConfig()
    report = {
        'phase': args.phase, 'excluded_development_scenes': DEVELOPMENT_SCENES,
        'selection_limit': 'The release has no original pedestrian sequence for scene 09',
        'scope': 'Current-scan point stage only; no SNR gate, ego fit, tracker or accumulation',
        'adaptation': {'range_m': [.15, 120], 'stationary_speed_max_m_s': .05},
        'legacy_config': asdict(legacy_config), 'candidate_config': asdict(candidate_config),
        'source_sha256': {str(p.relative_to(REPO)): sha256_file(p) for p in (
            Path(__file__), REPO / 'smartmicro_processing/smartmicro_processing/ghost_candidates.py',
            REPO / 'smartmicro_processing/smartmicro_processing/ghosts.py',
            Path(__file__).with_name('assess_ghost_dataset.py'))},
        'files': [], 'aggregates': {}, 'timing_ms': {},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with plan_path.open('x') as stream:
        json.dump({'phase': args.phase, 'candidate_config': asdict(candidate_config),
                   'sources': sources}, stream, indent=2)
        stream.write('\n')
    totals, times = {}, {'legacy': [], 'candidate_three_ablations': []}
    for i, item in enumerate(sources):
        if args.phase == 'pilot':
            path = Path(item['local_path'])
            if sha256_file(path) != item['sha256']:
                raise ValueError('Sample SHA256 mismatch')
            with h5py.File(path, 'r') as data:
                rows = data['radar'][:]
        else:
            # Read only one member into memory; no additional raw data on workspace disk.
            with zipfile.ZipFile(args.dataset_root / 'archives' / item['archive']) as archive:
                info = archive.getinfo(item['name'])
                if info.file_size != item['bytes'] or f'{info.CRC:08x}' != item['crc32']:
                    raise ValueError('Archive member differs from verified index')
                with io.BytesIO(archive.read(item['name'])) as member:
                    with h5py.File(member, 'r') as data:
                        rows = data['radar'][:]
        result, timing = evaluate(rows, legacy_config, candidate_config)
        scene = int(item['name'].split('scenario-')[1][:2])
        kind = 'cycl' if '_cycl_' in item['name'] else 'ped' if '_ped_' in item['name'] else 'mixed'
        for group in (item['archive'], f'{item["archive"]}/scene-{scene:02d}',
                      f'{item["archive"]}/{kind}'):
            aggregate = totals.setdefault(group, {})
            for cohort, rules in result['cohorts'].items():
                for rule, values in rules.items():
                    add_counts(aggregate.setdefault(cohort, {}).setdefault(rule, {}), values)
        for key in times:
            times[key].extend(timing[key])
        report['files'].append({
            'source': {k: v for k, v in item.items() if k != 'local_path'},
            'rows': result['rows'], 'sensor_frames': result['sensor_frames'],
            'cohorts': {c: {rule: with_rates(v) for rule, v in rules.items()}
                        for c, rules in result['cohorts'].items()}})
        print(f'{i+1}/{len(sources)} {item["name"]}: {result["rows"]} rows', flush=True)
    report['aggregates'] = {group: {c: {rule: with_rates(v) for rule, v in rules.items()}
                                  for c, rules in cohorts.items()}
                            for group, cohorts in totals.items()}
    report['timing_ms'] = {key: percentiles(values) for key, values in times.items()}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
