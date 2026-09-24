#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Plot the historical settings trial and the separate, fresh filter audit."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assessment', required=True, type=Path)
    parser.add_argument('--tuning', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    assessment = json.loads(args.assessment.read_text())
    trials = {trial['label']: trial for trial in json.loads(args.tuning.read_text())['trials']}
    baseline, faster = trials['baseline_start'], trials['can_targets_disabled']
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False,
                         'axes.spines.right': False, 'font.family': 'DejaVu Sans'})
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.2))
    colors = ['#526879', '#117C83']
    for ax, field, title, units in [
            (axes[0], 'cloud_hz', 'Earlier trial: update rate', 'Scans / second'),
            (axes[1], 'mean_targets', 'Earlier trial: scan density', 'Detections / scan')]:
        values = [baseline[field], faster[field]]
        bars = ax.bar(['CAN output on', 'CAN output off'], values, color=colors, width=.58)
        ax.bar_label(bars, fmt='%.2f', padding=4)
        ax.set_ylim(0, max(values) * 1.28)
        ax.set_ylabel(units)
        ax.set_title(title, loc='left', fontweight='bold')
        ax.set_axisbelow(True)
        ax.grid(axis='y', alpha=.2)
    gates = assessment['snr_threshold_retention']
    axes[2].plot([float(k) for k in gates], [100*v for v in gates.values()],
                 color='#117C83', marker='o', linewidth=2)
    axes[2].set(xlabel='Minimum reported SNR (dB)', ylabel='Detections retained (%)',
                ylim=(-2, 108), xlim=(-1, 51))
    axes[2].set_title('Fresh capture: SNR filtering', loc='left', fontweight='bold')
    axes[2].grid(alpha=.2)
    axes[2].annotate('6 dB retains 100%', (6, 100), xytext=(9, 75),
                     arrowprops={'arrowstyle': '->', 'color': '#526879'})
    fig.suptitle('UMRR-96: faster output, unchanged detections per scan',
                 x=.055, ha='left', fontsize=15, fontweight='bold')
    fig.text(.055, .05,
             '2026-09-24 | Historical settings trial and a separate 808-scan capture. '
             'Retention is not detection accuracy.', fontsize=9, color='#526879')
    fig.tight_layout(rect=(.02, .10, 1, .92))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)


if __name__ == '__main__':
    main()
