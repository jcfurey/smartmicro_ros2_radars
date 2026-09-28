# Opt-in tracker integration — 2026-09-28

The confirmation, standing-support and joint-association branches are merged
into `cam_wip`. All three options default **off**, in both `TrackerConfig` and
the installed YAML. The original branch reports remain historical records of
the individual experiments with their options enabled.

Both processing and tracking launches expose `evidence_confirmation`,
`standing_support` and `joint_association`. Each accepts `true`, `false`, or
`config` (inherit YAML). Explicit launch choices override YAML; omitted choices
preserve it. The node reports the settings in diagnostics and makes them
read-only after startup. See [usage](../smartmicro_processing/README.md#opt-in-tracker-experiments).

`standing_hold` defaults to 30 seconds and matters only with standing support.
`association_uncertainty` remains false: its covariance-based gating regressed
in the original experiment. `association_doppler` remains true as a latent
preference, used only when both joint association and uncertainty are enabled.
Distance-only joint assignment uses SciPy's solver; the dependency is declared.

## Compatibility and combinations

The replay uses a frozen copy of tracker code/YAML from **`0349cc3`** and the
original experiment commits: confirmation `5969945`, standing `160a1da`, and
association `d9a264f`. All variants receive the same point-stage outputs.
Every combination of the three booleans is replayed from a cold start over
**3,436 scans / 98,443 raw points**, spanning the walk and two quiet recordings.

At every scan, the default merged tracker exactly matches the pre-merge
tracker's existing runtime state. Each individual option also exactly matches
its corresponding original branch. Comparison includes every reference `Track`
field, background occupancy/weight/timestamps, next ID, last step and static
novelty mask. New fields absent from a reference class are excluded; the report
lists the compared fields. This is stronger than the original worktree study's
smaller state digest. All combinations retain finite state and covariance.

Bits below are confirmation, standing support, and joint association, in that
order. Values are scans with a near-range person-proxy track:

| Enabled bits | Walking / 618 | Standing / 145 | Swaying / 108 |
| --- | ---: | ---: | ---: |
| 000 (default) | 591 | 88 | 63 |
| 001 | 591 | 88 | 63 |
| 010 | 591 | 145 | 108 |
| 011 | 591 | 145 | 108 |
| 100 | 593 | 88 | 64 |
| 101 | 593 | 88 | 64 |
| 110 | 593 | 145 | 108 |
| 111 | 593 | 145 | 108 |

All combinations have far-range tracks in 10/618 walking scans, none while
standing/swaying, no tracks in the 290-scan interior absence interval, and no
confirmed tracks in the 1,591 quiet scans. Full standing coverage includes
bounded coasting: with standing support, 98/145 standing scans have a current
supporting return. These are the previously examined room recordings and range
proxies, not independent annotations or a general performance guarantee.
Joint distance assignment adds no improvement on this recording.

## Package checks

The isolated processing build passes, along with **153 pytest cases** and
package lint. Tests include all eight combinations through confirmation,
standing and departure; all eight through the node's current-scan point clouds;
the original assignment counterexample; and installed-launch tests exercising
default-off behavior, inherited YAML, explicit enable, and explicit disable.
Four initial lint findings were corrected and the lint check passed on rerun.

The current RViz preset retains zero point decay. Internal track coasting
retains `max_coast=1.0`; extended standing support renews only from current
returns. No sensor tuning, live radar trial or obstacle-layer evaluation was
performed. The isolated ROS tests used localhost test domains 178/177.

The [machine-readable summary](umrr96-opt-in-integration-20260928.json) includes
input/report/source hashes and exact comparison pairs. Detailed replay evidence
is under workspace `results/umrr96-opt-in-integration-20260928/`; the frozen
pre-merge sources are retained there. See the
[reproduction command](../tools/assessment/README.md#merged-option-validation).
