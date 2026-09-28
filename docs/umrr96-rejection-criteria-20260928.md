# Rejection criteria comparison — 2026-09-28

The recorded UMRR-96 walk supports a small, optional tradeoff: keep a mover when
its only ghost evidence is a nearer static return. This rescues nine additional
person-proxy detections while admitting 25 additional ghost-proxy detections,
with unchanged confirmed far-track presence in this recording. Stronger
signed-speed, direction and multiple-return requirements transfer poorly to the
sparse UMRR data. None justifies replacing the live default.

The preceding paper/dataset study was committed as **`c22535e`**. This follow-up
adds the optional `reject_static_only` policy, defaulting to **true**, plus offline
comparison tools. The radar remained powered off. Raw datasets remain on
`/run/media/jcfurey/Bulk-Storage/RadarGhostDataset/v1.1`.

## Criteria and implementation

All variants inspect one current scan. The existing rules reject a farther
mover for a same-absolute-speed match, a doubled-absolute-speed match, or a
nearer static return at similar bearing. Speed matches have no bearing/sign
requirement; independent movers can therefore trigger them. A static detection
alone does not establish an opaque wall. The
[paper study](umrr96-ghost-dataset-study-20260928.md#what-the-full-text-paper-reading-changes)
also explains why doubled radial speed is not a general bounce-order model.

| Report key | Rejection evidence |
|---|---|
| `none` | No point-stage ghost rejection; tracker rules still apply in bag replay |
| `legacy` | Any of the three existing triggers |
| `no_static_only` | Same/double-speed triggers; static-only evidence is advisory |
| `signed_pair` | Farther by >1.5 m; same Doppler sign and signed-speed difference <0.25 m/s |
| `aligned_pair` | Signed pair plus full 3D direction difference <4° |
| `supported_copy` | Aligned pair supported by two distinct nearer XYZ positions within 0.6 m of each other |

The last three candidates replace all existing triggers for their offline
comparison; they do not additionally apply behind-static or double-speed rules.
Their scales came from existing geometric settings and were frozen before the
larger scene comparison. Duplicate XYZ cannot supply a second supporting return.
Even multiple aligned returns cannot distinguish an independent same-speed
object from a copy in every case. These candidates are not wired into the node.

The optional runtime policy separates evidence from the rejection decision.
With `reject_static_only: false`, retained movers still have the
`GHOST_BEHIND_STATIC` audit flag, while classification, subset clouds and color
agree that they were retained. Other overlapping ghost triggers still reject.
Diagnostics expose the active policy and `static_only_advisory` count. See the
[configuration instructions](../smartmicro_processing/README.md#comparing-static-only-rejection).

## Current radar: recorded walk

The replay reads 1,845 scans / 60,251 raw detections from the September 24 walk.
It uses the current quality gates, 3D Doppler fitter and a separate cold instance
of the unchanged tracker for each policy. Scoring is limited to the documented
walking (6–40 s), standing (42–50 s) and swaying (50–56 s) phases. Room-range
proxies call movers below 3 m direct and beyond 3.4 m ghosts. These are not
independent annotations, and intermediate ranges are unscored.

| Point policy | Direct proxy retained | Ghost proxy rejected | Walking scans with a far track |
|---|---:|---:|---:|
| None | 100.0% | 0.0% | 16.3% |
| Existing | 95.8% | 93.7% | 1.6% |
| Static-only advisory | **96.5%** | **92.8%** | **1.6%** |
| Signed pair | 97.4% | 72.8% | 21.7% |
| Aligned pair | 99.9% | 11.6% | 18.9% |
| Supported copy | 100.0% | 0.4% | 16.7% |

Point denominators are **1,278 direct / 2,682 ghost proxies**. Existing rejection
removes 54 direct and 2,513 ghost points; static-only advisory removes 45 and
2,488. Both produce a confirmed person track in **591/618 walking scans** and a
far track in **10/618**. This is unchanged presence, not proof that trajectories
or identities agree. Person-track presence while standing is 88/145 and while
swaying 63/108 for every variant. No standing movers enter the point score;
that score does not measure standing-person detection.

The stricter copy criteria barely reject the sparse sensor's ghosts. Their
greater direct retention does not compensate for the much higher far-track
presence. The tracker also retains its own same/double-absolute-speed ghost
rule; changing the point policy does not fix independent-mover suppression
there. Tracking interactions explain why point retention alone cannot predict
track quality. Obstacle-layer effects were not evaluated.

## Wider dataset comparison

After the six-recording pilot in scenes 11, 13 and 21, the comparison selected
the smallest compressed original pedestrian and cyclist recording available in
each other scene, plus its smallest virtual recording. Scene 09 has no original
pedestrian sequence. Selection was written before evaluation and used filenames
and sizes, not annotations: **35 originals + 18 overlays in 18 scenes**, totaling
**10,923,422 rows / 26,232 sensor/frame groups**.

The three copy candidates were fixed before these scenes were opened.
`no_static_only` was added afterward as an **exploratory follow-up** on those
same files; it has no new independent holdout. These short/small selections do
not represent every recording. Originals and virtual overlays remain separate;
overlays are constructed combinations, not physical crossing experiments.

| Point policy | Original direct retained | Original ghosts rejected | Overlay direct retained | Overlay ghosts rejected |
|---|---:|---:|---:|---:|
| None | 100.0% | 0.0% | 100.0% | 0.0% |
| Existing | 43.4% | 89.0% | 45.1% | 95.9% |
| Static-only advisory | 69.4% | 54.1% | 60.5% | 80.1% |
| Signed pair | 81.2% | 44.3% | 82.4% | 56.7% |
| Aligned pair | 96.7% | 23.6% | 95.8% | 28.1% |
| Supported copy | 99.0% | 17.2% | 97.1% | 22.2% |

Moving-label denominators are **185,634 direct / 36,851 specified ghosts** for
originals and **9,868 / 1,319** for overlays. The adapter uses finite range
0.15–120 m, zero ego velocity and `abs(vr)>0.05 m/s`, with 2D XYZ and no SNR gate,
ego fit or tracker. Amplitude is not substituted for SNR. Confident label masks,
generic ghosts and exclusions follow the
[baseline scoring contract](umrr96-ghost-dataset-study-20260928.md#adapter-and-scoring-contract).
Fixed all-valid cohorts and class breakdowns are retained in the evidence.

Static-only advisory substantially reduces direct suppression but also loses
much of the original dataset's ghost rejection. Rates are detection-weighted,
with correlated frames/points and no independent-sample confidence intervals.
The dataset is much denser than our radar; these percentages are not UMRR
accuracy estimates or steel-wall recall measurements.

## Verification and next work

The subsequent [tracker audit](umrr96-tracker-audit-20260928.md) locates losses
after point rejection, tests shorter confirmation windows and reproduces
multi-target association/ghost-rule counterexamples. It keeps runtime defaults
unchanged and identifies evidence-based confirmation as a separate experiment.

The build and package checks passed **119 processing pytest cases** plus lint.
New cases cover independent movers, duplicate support, direction/elevation,
overlapping flags, both node policies, audit/color agreement, byte preservation
and clearing on the next scan. A default-policy publication-sink replay matched
all existing outputs over **474 scans / 8,689 detections**, with zero output
mismatches and zero current-scan provenance errors. It bypasses historical
freshness checks and does not measure DDS or live latency. RViz decay remains
zero; the changes add no historical display points.

The [evidence JSON](umrr96-rejection-criteria-20260928.json) records counts,
configurations, source members/CRCs, report hashes and validation. Full reports
remain under workspace `results/umrr96-rejection-20260928/`. Evaluated source
snapshots preserve the wider run's hashes; later docstring/comment formatting
was checked for identical Python ASTs after removing docstrings. Final pilot
and UMRR replay hashes match the current evaluated code. Use the
[reproduction commands](../tools/assessment/README.md#comparing-rejection-criteria).

The next useful rejection experiment should use continued track support and
supported reflector geometry before deleting ambiguous detections. First obtain
a physical UMRR capture with two independent people at differing ranges,
bearings and motion signs; retain a standing phase and mark reflector locations.
That can test the tracker’s separate magnitude-only rule and whether a temporal
hypothesis preserves people without adding ghost tracks. Internal history can
inform today's decision while every displayed XYZ still comes from today's scan.
