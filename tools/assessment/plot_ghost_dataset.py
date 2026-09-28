# SPDX-License-Identifier: Apache-2.0
"""Plot point retention/rejection separately for originals and overlays."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('report', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Choose a new output path')
    data = json.loads(args.report.read_text())
    variants = ['none', 'same_speed', 'double_speed', 'behind_static', 'all']
    labels = ['No suppression', 'Same speed', 'Double speed', 'Behind static', 'All rules']
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5), sharey=True, layout='constrained')
    for ax, archive, title in zip(axes, ['original.zip', 'virtual.zip'],
                                 ['Original recordings', 'Constructed overlays']):
        metrics = data['aggregates'][archive]
        y = np.arange(len(variants))
        for offset, field, label, color in [
                (-.18, 'direct_retention', 'Direct returns retained', '#147d92'),
                (.18, 'specific_ghost_rejection', 'Ghost returns rejected', '#ba6718')]:
            values = [metrics[v][field] * 100 for v in variants]
            bars = ax.barh(y + offset, values, .33, label=label, color=color)
            ax.bar_label(bars, labels=[f'{v:.1f}%' for v in values], padding=3, fontsize=9)
        ax.set_yticks(y, labels)
        ax.set_xlim(0, 116)
        ax.set_xticks([0, 25, 50, 75, 100])
        ax.set_xlabel('Percent of eligible moving detections')
        ax.set_title(f'{title}\n{metrics["all"]["direct_total"]:,} direct / '
                     f'{metrics["all"]["specific_ghost_total"]:,} ghosts', fontsize=11)
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='x', alpha=.15)
        ax.set_axisbelow(True)
    axes[0].invert_yaxis()
    axes[0].legend(loc='lower left', bbox_to_anchor=(0, -.3), frameon=False)
    fig.suptitle('Existing ghost rules lose many direct returns on this dataset sample\n'
                 'Six sequences · two sensors evaluated separately · no SNR gate or ego fit',
                 fontsize=13)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)


if __name__ == '__main__':
    main()
