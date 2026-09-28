# SPDX-License-Identifier: Apache-2.0
"""Compare the launch and Python fallback motion gates, without optimizing them."""
import argparse
import json
from pathlib import Path

import h5py
import yaml

from assess_ghost_dataset import (
    REPO, classify_rows, decode_labels, score, rates, GhostConfig, sha256_file)
from smartmicro_processing.doppler import FitConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a new output path')
    config_path = REPO / 'smartmicro_processing/config/umrr96_processing.yaml'
    params = yaml.safe_load(config_path.read_text())['/**/umrr96_processing']['ros__parameters']
    config = GhostConfig(**{key: params[key] for key in vars(GhostConfig())})
    thresholds = [params['residual_threshold'], FitConfig().residual_threshold]
    manifest = json.loads(args.manifest.read_text())
    identities = [(item['archive'], item['name']) for item in manifest]
    if len(set(identities)) != len(identities):
        raise ValueError('Duplicate source members')
    report = {
        'thresholds_m_s': thresholds,
        'cohorts': 'moving and all finite in-range labeled points; all_valid denominator stays fixed',
        'config': {**vars(config), 'min_range': params['min_range'], 'max_range': params['max_range']},
        'source_sha256': {str(p.relative_to(REPO)): sha256_file(p) for p in (
            Path(__file__), Path(__file__).with_name('assess_ghost_dataset.py'), config_path,
            REPO / 'smartmicro_processing/smartmicro_processing/ghosts.py',
            REPO / 'smartmicro_processing/smartmicro_processing/doppler.py')},
        'members': [{k: v for k, v in item.items() if k != 'local_path'} for item in manifest],
        'aggregates': {},
    }
    counts = {str(threshold): {} for threshold in thresholds}
    for item in manifest:
        if item['archive'] not in ('original.zip', 'virtual.zip'):
            raise ValueError('Unknown archive type')
        if sha256_file(item['local_path']) != item['sha256']:
            raise ValueError('Input checksum mismatch')
        with h5py.File(item['local_path'], 'r') as data:
            rows = data['radar'][:]
        for threshold in thresholds:
            valid, moving, reasons, _ = classify_rows(
                rows, config, threshold, params['min_range'], params['max_range'])
            labels = decode_labels(rows['label_id'], rows['group'])
            for name, selected in [('moving', moving), ('all_valid', valid)]:
                metrics = score(labels, reasons, selected)['all']
                aggregate = counts[str(threshold)].setdefault(
                    item['archive'] + '/' + name, {key: 0 for key in metrics})
                for key, value in metrics.items():
                    aggregate[key] += value
    report['aggregates'] = {
        threshold: {key: rates({'all': metrics})['all'] for key, metrics in cohorts.items()}
        for threshold, cohorts in counts.items()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')


if __name__ == '__main__':
    main()
