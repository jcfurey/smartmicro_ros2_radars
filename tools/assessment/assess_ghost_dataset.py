# SPDX-License-Identifier: Apache-2.0
"""Offline, single-scan test of our ghost rules on Radar Ghost Dataset v1.1.

This adapts only the point ghost stage: fixed sensor, polar 2D measurements,
no amplitude/SNR substitution, no ego fit, tracking, accumulation, or ROS.
Ground-truth fields are read only after predictions have been generated.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / 'smartmicro_processing'))
from smartmicro_processing.ghosts import GhostConfig, GhostReason, ghost_reasons


VARIANTS = {
    'none': 0,
    'same_speed': int(GhostReason.SAME_SPEED),
    'double_speed': int(GhostReason.DOUBLE_SPEED),
    'behind_static': int(GhostReason.BEHIND_STATIC),
    'all': 7,
}
BUCKETS = ('direct', 'specific_ghost', 'generic_ghost', 'direct_ambiguous',
           'uncertain', 'background', 'ignore', 'noise', 'group', 'invalid')


def decode_labels(labels, groups):
    """Conservative binary scoring; preserve excluded categories explicitly.

    Order is a bit mask: 3 can include direct, 6 cannot include direct.
    Order 0 is documented multipath, reported separately because its order
    is unspecified. Negative annotations never enter confident scores.
    """
    labels = np.asarray(labels, dtype=np.int64)
    groups = np.asarray(groups, dtype=bool)
    if labels.shape != groups.shape or labels.ndim != 1:
        raise ValueError('Labels and groups must be equal-length vectors')
    c = labels // 1000
    m = labels // 100 % 10
    t = labels // 10 % 10
    o = labels % 10
    valid = ((c >= 1) & (c <= 5) & np.isin(m, [0, 1])
             & np.isin(t, [0, 1, 2, 3]) & np.isin(o, [0, 1, 2, 3, 4, 6]))
    result = np.full(labels.shape, 'invalid', dtype='U24')
    result[valid & (t == 1) & (o == 1)] = 'direct'
    result[valid & np.isin(o, [2, 4, 6])] = 'specific_ghost'
    result[valid & (o == 0)] = 'generic_ghost'
    result[valid & (o == 3)] = 'direct_ambiguous'
    result[labels <= -1000] = 'uncertain'
    for value, name in [(0, 'background'), (-1, 'ignore'), (-2, 'noise')]:
        result[labels == value] = name
    result[groups] = 'group'
    return result


def classify_rows(rows, config, residual_threshold, min_range, max_range):
    """Keep each sensor/frame independent; annotations cannot affect decisions."""
    required = {'frame', 'sensor', 'r_sc', 'phi_sc', 'vr_sc'}
    if not required.issubset(rows.dtype.names or ()):
        raise ValueError(f'Missing measurement fields: {required - set(rows.dtype.names or ())}')
    radius, azimuth, speed = (rows[k] for k in ('r_sc', 'phi_sc', 'vr_sc'))
    valid = (np.isfinite(radius) & np.isfinite(azimuth) & np.isfinite(speed)
             & (radius >= min_range) & (radius <= max_range))
    moving = valid & (np.abs(speed) > residual_threshold)
    reasons = np.zeros(len(rows), dtype=np.uint8)
    sensor_frames = 0
    for sensor in np.unique(rows['sensor']):
        sensor_indices = np.flatnonzero(rows['sensor'] == sensor)
        order = sensor_indices[np.argsort(rows['frame'][sensor_indices], kind='stable')]
        breaks = np.flatnonzero(np.diff(rows['frame'][order])) + 1
        for indices in np.split(order, breaks):
            if not len(indices):
                continue
            sensor_frames += 1
            selected = indices[valid[indices]]
            xyz = np.column_stack((radius[selected] * np.cos(azimuth[selected]),
                                   radius[selected] * np.sin(azimuth[selected]),
                                   np.zeros(len(selected))))
            mover = moving[selected]
            reasons[selected[mover]] = ghost_reasons(
                xyz[mover], speed[selected[mover]], xyz[~mover], config)
    return valid, moving, reasons, sensor_frames


def score(buckets, reasons, selected):
    counts = {}
    for variant, bitmask in VARIANTS.items():
        rejected = (reasons & bitmask) != 0
        metrics = {}
        for label in ('direct', 'specific_ghost', 'generic_ghost'):
            eligible = selected & (buckets == label)
            metrics[label + '_total'] = int(np.count_nonzero(eligible))
            metrics[label + '_rejected'] = int(np.count_nonzero(eligible & rejected))
        counts[variant] = metrics
    return counts


def rates(counts):
    result = {}
    for variant, metrics in counts.items():
        result[variant] = dict(metrics)
        for label in ('direct', 'specific_ghost', 'generic_ghost'):
            total, rejected = metrics[label + '_total'], metrics[label + '_rejected']
            name = 'direct_retention' if label == 'direct' else label + '_rejection'
            result[variant][name] = ((total - rejected if label == 'direct' else rejected)
                                     / total if total else None)
    return result


def label_counts(buckets, selected):
    return {b: int(np.count_nonzero(selected & (buckets == b))) for b in BUCKETS}


def assess_rows(rows, config, residual_threshold=.05, min_range=.15, max_range=120.):
    valid, moving, reasons, sensor_frames = classify_rows(
        rows, config, residual_threshold, min_range, max_range)
    buckets = decode_labels(rows['label_id'], rows['group'])
    scores = score(buckets, reasons, moving)
    all_rows = np.ones(len(rows), dtype=bool)
    report = {
        'rows': len(rows), 'sensor_frames': sensor_frames,
        'valid_rows': int(valid.sum()), 'moving_rows': int(moving.sum()),
        'labels': dict(sorted(Counter(map(str, rows['label_id'])).items())),
        'categories_all': label_counts(buckets, all_rows),
        'categories_invalid_measurement_or_range': label_counts(buckets, ~valid),
        'categories_stationary': label_counts(buckets, valid & ~moving),
        'categories_moving': label_counts(buckets, moving),
        'moving_reason_masks': {str(k): int(np.count_nonzero(moving & (reasons == k)))
                                for k in range(8)},
        'variants': rates(scores),
        'by_sensor': {}, 'all_rules_by_range_m': {}, 'all_rules_by_abs_bearing_deg': {},
    }
    for sensor in np.unique(rows['sensor']):
        report['by_sensor'][sensor.decode()] = rates(score(
            buckets, reasons, moving & (rows['sensor'] == sensor)))
    abs_bearing = np.abs(np.rad2deg(np.angle(np.exp(1j * rows['phi_sc']))))
    for key, values, edges in [
            ('all_rules_by_range_m', rows['r_sc'], [0, 5, 10, 20, 120.000001]),
            ('all_rules_by_abs_bearing_deg', abs_bearing, [0, 30, 60, 90, 180.000001])]:
        for lower, upper in zip(edges, edges[1:]):
            selected = moving & (values >= lower) & (values < upper)
            report[key][f'{lower:g}-{upper:g}'] = rates(score(buckets, reasons, selected))['all']
    return report, scores


def sha256_file(path):
    with open(path, 'rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True,
                        help='Selected members, local_path and expected sha256 for each H5')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; choose a new path')
    import h5py
    import yaml

    config_path = REPO / 'smartmicro_processing/config/umrr96_processing.yaml'
    params = yaml.safe_load(config_path.read_text())['/**/umrr96_processing']['ros__parameters']
    config = GhostConfig(**{k: params[k] for k in asdict(GhostConfig())})
    adaptation = {k: params[k] for k in ('residual_threshold', 'min_range', 'max_range')}
    manifest = json.loads(args.manifest.read_text())
    seen = set()
    aggregates = {}
    files = []
    for item in manifest:
        identity = (item['archive'], item['name'])
        if identity in seen:
            raise ValueError(f'Duplicate member: {identity}')
        seen.add(identity)
        if item['archive'] not in ('original.zip', 'virtual.zip'):
            raise ValueError('Expected original.zip or virtual.zip')
        path = Path(item['local_path'])
        digest = sha256_file(path)
        if digest != item['sha256']:
            raise ValueError(f'SHA256 mismatch: {path}')
        with h5py.File(path, 'r') as data:
            rows = data['radar'][:]
        report, scores = assess_rows(rows, config, **adaptation)
        provenance = {k: v for k, v in item.items() if k != 'local_path'}
        files.append({'source': provenance, 'radar_fields': list(rows.dtype.names), **report})
        aggregate = aggregates.setdefault(item['archive'], {
            v: {k: 0 for k in metrics} for v, metrics in scores.items()})
        for variant, metrics in scores.items():
            for key, value in metrics.items():
                aggregate[variant][key] += value
        print(item['name'], report['rows'], 'rows', flush=True)
    report = {
        'scope': 'Exploratory 2D point-ghost-stage transfer test; not full pipeline or paper reproduction',
        'dataset': 'https://zenodo.org/records/6676246',
        'prediction_policy': 'All finite in-range points, each sensor/frame separately; zero ego speed; no SNR gate',
        'scoring_policy': 'Moving points only; confident direct vs specified-order ghosts; generic ghosts separate; groups/uncertain excluded',
        'versions': {'python': sys.version.split()[0], 'numpy': np.__version__, 'h5py': h5py.__version__},
        'source_sha256': {str(p.relative_to(REPO)): sha256_file(p) for p in (
            Path(__file__), config_path, REPO / 'smartmicro_processing/smartmicro_processing/ghosts.py')},
        'config': {**asdict(config), **adaptation},
        'aggregates': {key: rates(value) for key, value in aggregates.items()},
        'files': files,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
