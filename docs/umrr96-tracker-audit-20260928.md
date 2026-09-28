# What to improve after point rejection — 2026-09-28

The next useful target is track confirmation based on the strength of the
evidence, followed by joint association for multiple people. The saved UMRR walk
shows more retained person-proxy points near tentative tracks than removed by
point rejection. Simply confirming sooner also admits more ghosts, however.
This follow-up adds a reproducible offline diagnostic; it changes no runtime
filter, tracker setting or RViz persistence.

## Where the recorded detections go

The audit replays the same 1,845 scans / 60,251 points as the
[criteria comparison](umrr96-rejection-criteria-20260928.md), with the current
YAML, existing point policy and unchanged tracker. All raw points pass the
required-field, range and 6 dB SNR gates in this recording. Lowering the host SNR
gate therefore cannot recover more of these recorded points. This says nothing
about detections already removed inside the sensor.

The documented person-present phases use the same approximate room labels:
movers below 3 m are direct proxies, and those beyond 3.4 m are ghost proxies.
Of 1,278 direct-proxy movers, point rejection removes 54 and retains 1,224.
Those retained detections divide as follows:

| Current point's proximity to tracks | Direct proxy | Ghost proxy |
|---|---:|---:|
| Visible confirmed track within 0.8 m | 1,015 | 3 |
| Confirmed but ghost-hidden track only | 0 | 23 |
| Tentative tracks only | **199** | **143** |
| No nearby track | 10 | 0 |

These categories follow the existing proximity-based `tracked_targets` contract;
they do not establish object identities or actual cluster ownership. Retained
points remain visible in the current-detection display even while tentative.
The direct-proxy deficit here concerns tracked association and classification,
not missing raw measurements.

## Shorter confirmation windows: measured tradeoff

An exploratory replay varies only the M-of-N confirmation counts, with a cold
tracker for each variant and identical point decisions. It uses this already
examined recording, not an independent evaluation set.

| Confirmation | Walking person-track presence | Walking far-track presence | Swaying person-track presence | Swaying far-track presence |
|---|---:|---:|---:|---:|
| Current: 8 of 10 | 591/618 (95.6%) | 10/618 (1.6%) | 63/108 (58.3%) | 0/108 (0.0%) |
| 6 of 8 | 595/618 (96.3%) | 20/618 (3.2%) | 65/108 (60.2%) | 13/108 (12.0%) |
| 4 of 5 | 607/618 (98.2%) | 28/618 (4.5%) | 67/108 (62.0%) | 14/108 (13.0%) |

Standing coverage stays 88/145 (60.7%) for every setting, with no far tracks.
There are no Doppler movers in that standing interval, so changing mover
confirmation cannot solve that loss. A separate standing-support experiment
needs to distinguish a person from the learned background.

The proposed next experiment is a confirmation score using prediction residuals,
measurement uncertainty, gaps and ghost evidence, instead of counting every hit
equally. Start by measuring whether these features distinguish the 199 and 143
tentative-point cohorts; then evaluate a frozen rule on separate recordings.
The current audit does not demonstrate that such a score will succeed.

## Reproduced multi-target weaknesses

The tool also exercises three isolated synthetic counterexamples against the
actual tracker:

- With predicted positions `(2,0)` and `(3,0)`, observations `(2.4,0)` and
  `(1.4,0)`, and the current 1.2 m gate, greedy association updates one existing
  track and creates a new tentative track. A joint assignment updates both
  existing tracks within the same gate.
- Confirmed tracks at `(2,0)` and `(4,4)` with signed radial speeds `+0.5` and
  `-0.5 m/s` trigger the farther track's ghost rule despite 45° bearing
  separation and opposite motion signs. This isolates the track stage; the
  current point filter could already have suppressed the farther input.
- Two points 0.4 m apart with radial speeds `+0.5` and `-0.5 m/s` merge into
  one zero-speed cluster under position-only clustering. This motivates testing
  motion compatibility, while checking that limb motion does not fragment one
  person into several tracks.

On the real walk, joint assignment offers one extra one-step match, with 17
changed assignments across 1,845 scans. Those alternatives are calculated on
the baseline's predicted states, without feeding them back into tracking.
They are opportunities, not measured identity or trajectory improvements.
The diagnostic keeps confirmed-first priority and maximizes matched detections
within each priority tier before minimizing distance. It is not a calibrated
probabilistic association model.

[Pegoraro and Rossi, sections IV-C–D](https://arxiv.org/html/2105.11368)
provide a stronger next association experiment: position uncertainty informed
by polar measurement geometry, innovation-based likelihoods, and global
one-to-one assignment. Borrowing those components does not require adopting
their close-track deletion rule, which would be problematic for our crossing
case. Our use of signed Doppler in association would be a separate extension
requiring validation.

For rejection itself, test explicit reflector/path consistency when wall
geometry is sufficiently supported. The
[MPET-GLMB measurement model](https://arxiv.org/html/2602.03464v2)
jointly represents targets, reflectors and propagation paths. That is a useful
modeling direction for steel-wall multipath; our current sparse wall returns
may require an independently measured layout to test it. Repeating a heuristic
over several scans alone does not make it physically correct.

All proposed temporal evidence can label current measurements while RViz
continues to show only current-scan XYZ with zero decay. None of this creates
additional native wall detections.

## Evidence and reproduction

[The audit JSON](umrr96-tracker-audit-20260928.json) contains phase counts,
configurations, counterexamples and source hashes. The full local report is
`results/umrr96-tracker-audit-20260928/walk-final.json`. During replay, the
diagnostic's greedy assignments matched the actual tracker updates in every
scan. The single-tier joint solver also matched exhaustive cardinality/distance
optima on 100 seeded small graphs. Baseline phase totals agree with the preceding
criteria replay. These checks validate the diagnostic, not multi-person accuracy.

Use the [assessment commands](../tools/assessment/README.md#tracker-loss-audit).
The radar can remain powered off. ROS is used to deserialize the saved bag;
the tool publishes nothing and does not evaluate DDS, live latency or the
obstacle layer. Production processing sources and defaults are unchanged by
this follow-up.
