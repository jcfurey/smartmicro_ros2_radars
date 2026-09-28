# Three tracker worktrees — 2026-09-28

**Standing support is the strongest candidate for the next physical trial.**
It maintains the person track throughout the recorded standing and swaying
intervals, without adding far tracks or tracks in the scored absence interval.
Confirmation gives a small improvement. Joint distance association fixes a
controlled assignment failure, but the wider tests show no accuracy gain yet.

All worktrees branch from **`3aee7a7`**, which checkpoints the preceding criteria
and tracker audits. Each has its own implementation, active experimental YAML,
build/install tree, tests and evidence. The main `cam_wip` processing defaults
remain the shared baseline; none of the experiment branches is merged there.
The radar remained powered off, and no hardware commands were sent.

| Worktree under workspace `worktrees/` | Branch | Commit | Report |
|---|---|---|---|
| `smartmicro-confirmation` | `experiment/umrr96-confirmation` | `5969945` | [Confirmation](../../../worktrees/smartmicro-confirmation/docs/experiment-confirmation.md) |
| `smartmicro-standing` | `experiment/umrr96-standing` | `160a1da` | [Standing support](../../../worktrees/smartmicro-standing/docs/experiment-standing.md) |
| `smartmicro-association` | `experiment/umrr96-association` | `d9a264f` | [Joint association](../../../worktrees/smartmicro-association/docs/experiment-association.md) |

## Same-recording comparison

Every tracker gets identical measurements after the baseline quality, Doppler
fit and point-ghost rules. Each runs independently from a cold start, including
its own background learning and feedback through future track states. The
1,845-scan walk contains 60,251 raw points. Person/ghost scores use the same
approximate room-range proxies as the preceding audit, not independent labels.

| Active variant | Walking person track | Standing person track | Swaying person track | Walking far track |
|---|---:|---:|---:|---:|
| Baseline | 591/618 (95.6%) | 88/145 (60.7%) | 63/108 (58.3%) | 10/618 (1.6%) |
| Confirmation | **593/618 (96.0%)** | 88/145 (60.7%) | **64/108 (59.3%)** | 10/618 (1.6%) |
| Standing support | 591/618 (95.6%) | **145/145 (100%)** | **108/108 (100%)** | 10/618 (1.6%) |
| Joint distance association | 591/618 (95.6%) | 88/145 (60.7%) | 63/108 (58.3%) | 10/618 (1.6%) |

All four have zero far-track presence in standing/swaying and zero tracks in
the 290-scan absence interval. The absence check uses 64–80 seconds, inside
the historical 62–82 second interval, avoiding the entering/leaving edges; it
has no independent camera annotation. Two additional quiet captures contribute
1,591 scans / 38,192 points, with no confirmed tracks in any variant.

Standing's 100% track coverage includes bounded coasting: 98/145 standing scans
have a current supporting return, compared with 53/145 for baseline. This does
not create native measurements. RViz still displays only current-scan XYZ with
zero point-cloud decay. Internal track coasting retains its existing one-second
limit, and extended standing support has a 30-second absolute cap.

## What the implementations change

**Confirmation:** an early path accepts six sufficiently consistent hits,
scored from position and signed-speed prediction errors. Misses reduce the
score, and point count cannot multiply a scan's vote. The existing 8-of-10 rule
remains the fallback. The improvement is small enough to warrant another
capture before adoption.

**Standing:** a confirmed mover records the warmed background before stopping.
Current returns outside that saved background can support an anchored stopped
track, with exclusive point ownership. Initial standing support needs three
hits in five scans; subsequent intermittent support can renew it until the
coast timeout or absolute cap. Sensor movement invalidates the saved background.
The first prototype repeatedly required a dense support burst and covered only
89/145 standing scans. The recorded trace exposed this unnecessary second
gate, which was removed after support was established. This revision is an
exploratory change on the same recording, not independent confirmation.

**Association:** joint distance matching uses the existing gate and confirmed
priority, maximizing matched tracks before minimizing distance. It fixes the
two-track case where greedy matching consumes the only available observation
for the second track. A separate innovation-cost mode remains disabled: using
the uncalibrated EKF uncertainty and signed Doppler reduced swaying coverage to
0/108 and produced far tracks in 32/108 scans. Position-only innovation gating
also lost three walking scans. The final active mode adds no new innovation gate.

## Controls and controlled cases

Merely raising legacy `static_hold` from 5 to 30 seconds also reaches full
standing/swaying coverage in this bag. It is therefore a necessary control,
not evidence that the extra background mechanism caused this recording's gain.

The mechanism distinguishes itself in the controlled long-stop case: both
baseline and timeout-only control absorb the stationary return into their
learned background and cover 69/240 scans. Saved-background support covers
240/240, each with a current supporting return. After departure it coasts for
18 scans, then disappears. No track survives beyond the existing coast limit,
including when a nearby known wall reappears after occlusion. Tests also check
anchor drift, intermittent returns, absolute age and exclusive support.

Twenty fixed-seed noisy two-person crossings produce identical aggregate
scores for the active variants: 7,237/7,240 object opportunities matched, zero
ID changes and zero unmatched tracks. These deliberately controlled tracker
inputs omit the point-filter and sensor models. They demonstrate no broad
association improvement; the separate assignment counterexample demonstrates
the specific failure that joint matching repairs. Physical crossings and a
standing/exit sequence near walls remain necessary evaluations.

## Validation and artifacts

All three isolated builds and package checks pass:

| Worktree | Processing pytest cases | ROS test domains |
|---|---:|---|
| Confirmation | 124 | 181 / 182 |
| Standing | 128 | 183 / 184 |
| Association | 124 | 185 / 186 |

The counts include shared regression cases, not 376 independent new tests.
Package lint also passes. With each feature disabled, its recorded digest
matches baseline across all **3,436 scans / 98,443 raw points**. The digest covers
track IDs, estimated state/covariance, support times, confirmation and ghost
flags; it does not hash every private internal variable. The confirmation node
test also checks that the next scan's cloud contains no old tracked point.

Recorded tracker-only computation p95 is below 0.7 ms for all active variants
on this host. The comparison runs variants sequentially and some other jobs
were active, so these timings do not establish relative speedups or live latency.
They exclude the fitter, DDS, sensor acquisition and RViz rendering. The obstacle
layer was not evaluated.

The [comparison evidence](umrr96-worktree-comparison-20260928.json) records branch
commits, configurations, source/report hashes, counts, controls and checks.
Detailed local reports are under `results/umrr96-three-worktrees-20260928/`.
Standing-source line wrapping after the recorded runs preserves an identical
Python AST; the evaluated source snapshot is retained, and the final synthetic
run hashes the current source directly.

Use the [reproduction commands](../tools/assessment/README.md#three-worktree-comparison).
Each build/install is under `.colcon/umrr96-exp-<name>/`. For future live trials,
select only one processing overlay and publisher at a time. None of these
branches has been enabled against the physical radar.
