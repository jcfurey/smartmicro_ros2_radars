# UMRR-96 ingest and filtering research, 2026-09-28

Scope: the current Type 153 radar, serial 230739, last-read firmware 5.2.2,
Smart Access Automotive 3.13.0 / UIF 1.2.2, and this fork at `6a0e9a6` with
the session's uncommitted changes. The operator selected radar-only research,
reported a fixed radar with possible moving people, and rejected one-second
point persistence. The operator announced radar shutdown during the research;
subsequent work used saved data and source code. No sensor settings or runtime filtering
were changed for this investigation.

The active Ethernet decoder already publishes all documented per-target
getters. The most useful next work is to make filter decisions traceable and
preserve ambiguous current-scan detections, then improve multi-object filtering
and association. Additional instantaneous measurements require a documented
sensor output or firmware capability; increasing ROS history does not supply
them.

Follow-up implementation: the filter audit and current-scan view are now built
and tested offline; see [validation below](#filter-audit-implementation-follow-up).
The existing filtering heuristics remain unchanged.

The [additional paper review](umrr96-paper-shortlist-20260928.md) expands this
research with sparse tracking, 2026 multipath/temporal methods, wall-reconstruction
data requirements and a public labeled ghost benchmark. It ranks the next
experiments against this sensor's available fields and measured scan rate.

## Exactly which SDK fields reach ROS

This inventory compares the installed UIF headers with
[`stream_codecs.hpp`](../umrr_ros2_driver/include/umrr_ros2_driver/stream_codecs.hpp),
[`smartmicro_radar_node.cpp`](../umrr_ros2_driver/src/smartmicro_radar_node.cpp),
and the message definitions. It describes this fork's current UMRR-96 path,
not every sensor model or the upstream driver's historical behavior.

| UMRR-96 `ComTargetList` getter | Published destination |
| --- | --- |
| `GetRange()` | Raw cloud `range`; also used to derive XYZ |
| `GetSpeedRadial()` | `radial_speed` |
| `GetAzimuthAngle()` / `GetElevationAngle()` | `azimuth_angle` / `elevation_angle`; also used for XYZ |
| `GetPower()` / `GetNoise()` | `power` / `noise`; `snr` is their difference |
| `GetRCS()` | `rcs`, copied without a unit conversion |
| `GetVarianceRange()` / `GetVarianceSpeed()` | `variance_range` / `variance_speed` |
| `GetVarianceAzimuthAngle()` / `GetVarianceElevationAngle()` | `variance_azimuth_angle` / `variance_elevation_angle` |
| `GetPeakIdx()` | `peak_idx`; not an object ID |
| `GetFalseAlarmProbability()` / `GetFlags()` | `Umrr96RawQuality.false_alarm_probability_raw` / `flags_raw` |
| Target-list `GetCycleTime()` / `GetNumberOfTargets()` | `PortTargetHeader.cycle_time` / `number_of_targets` |
| Target-list `GetAcquisitionSetup()` | `PortTargetHeader.acquisition_setup`, with `acquisition_setup_valid=true` |
| Port `GetTimestamp()` | `RadarTiming.device_timestamp_us`; ROS header stamp remains host receive time |
| Port ID, major/minor version, size, endianness, index, header major/minor version | Corresponding `PortTargetHeader` fields |

All **14 per-target getters**, **3 target-list header getters**, and **9 generic
port header getters** have destinations. Pfa and flags are published only when
the raw-quality topic has a subscriber. The cloud's corresponding fields retain
their compatibility sentinels; the side topic explicitly marks the raw values'
semantics unverified. Four variances and peak index were recovered by earlier
work, not newly exposed by this research.

The separate `ComTargetBaseList`/CAN conversion preserves its seven target
getters and six target-list header getters, including cycle count and fractional
timestamp. It does **not** forward eight generic port metadata getters: port ID,
major/minor version, size, endianness, index, and header major/minor version.
Its generic timestamp is forwarded through `RadarTiming`. This omission does
not affect the active Ethernet `ComTargetList` stream or add detections if fixed.

Several generic ROS header fields are zero on UMRR-96 because this UIF has no
corresponding getter: separate per-scan antenna/sweep/centre-frequency/PRF,
unambiguous speed, and acquisition-start/base fields. Configuration readback
and the opaque acquisition word are available; interpreting those zeros as
measured values would be incorrect.

Primary local definitions: [target getters](../../../references/smartmicro_umrr96/smart_access/include/umrr96_t153_automotive_v1_2_2/comtargetlist/Target.h),
[target header](../../../references/smartmicro_umrr96/smart_access/include/umrr96_t153_automotive_v1_2_2/comtargetlist/TargetListHeader.h),
[port header](../../../references/smartmicro_umrr96/smart_access/include/umrr96_t153_automotive_v1_2_2/comtargetlist/PortHeader.h),
[raw-quality message](../umrr_ros2_msgs/msg/Umrr96RawQuality.msg).

## Additional input streams and their limits

| Candidate | Evidence and practical value |
| --- | --- |
| Ethernet full target list | Already the primary input, with the quality fields above |
| Compact/CAN target list | Another representation with fewer quality fields; no evidence of additional independent detections. Earlier CAN-output-off measurements raised Ethernet delivery from 8.33 to 18.18 Hz |
| Generic SDK port observer / packet capture | Useful for inventorying transmitted port IDs, decoder comparisons and replay. Receiving generic bytes does not enable an undocumented sensor output |
| Native object list / collision-avoidance grid | Datasheet describes optional firmware products. Current object-output controls have min=max=0, and the UMRR-96 stream interface has only target-list callbacks |
| ADC, complex antenna samples, range-Doppler cube, actual peak list | No corresponding stream in the audited UMRR-96 interface. A peak index is only a reference; it does not contain the peak's spectrum |
| Firmware detection threshold or clutter suppression | No general CFAR/SNR threshold or static-clutter suppression control in the 29-parameter UIF definition |

The [vendor stream interface](../../../references/smartmicro_umrr96/smart_access/include/umrr96_t153_automotive_v1_2_2/DataStreamServiceIface.h)
has full and compact target callbacks. The common `DataServicesIface` can receive
generic ports, but an observer should first be exercised in an isolated replay
process: this is a singleton SDK and callback coexistence is not established.
The [earlier SDK audit](sdk-audit.md) covers that ownership/lifetime constraint.

The [Type 153 datasheet](https://www.smartmicro.com/fileadmin/media/Downloads/Automotive_Radar/Sensor_Data_Sheets_76-81GHz/UMRR_Automotive_Type_153_Data_Sheet.pdf)
describes the optional firmware outputs. The [upstream compatibility table](https://github.com/smartmicro/smartmicro_ros2_radars#umrr-radars-and-smart-access-api-version)
pairs UIF 1.2.2 with firmware 5.2.4; our last readback was 5.2.2. No public evidence
reviewed here establishes that an upgrade would unlock denser output on this unit.

## New observational sample

A passive recording was started before the operator announced shutdown. The
recorder did not finish within its SIGINT grace period and was force-stopped
(exit 137). Its original MCAP was preserved, a copy was reindexed successfully,
and complete readable records were analyzed. This is a **recovered partial
capture**, not a clean end-to-end loss test. The recoverable raw span was
26.015 seconds; comparisons below use 473 frames present in every processing
topic. Capture boundaries and unmatched messages are excluded from those joins.

| Measurement | Result |
| --- | ---: |
| Raw frames / detections in recoverable sample | 474 / 8,689 |
| Raw scan rate | 18.1815 Hz |
| Raw detections per scan | 18.33 mean; 12–25 range |
| SDK header target-count mismatches | 0 in 474 matched headers |
| Views filtered-cloud byte differences | 0 in 474 matched clouds |
| Timing / raw-quality messages matched to raw | 473 / 473 |
| Raw-quality array-length mismatches | 0 |
| Kernel UDP drops | 0 throughout 26 diagnostic samples |
| Raw receive interval p95 / maximum | 55.128 / 55.176 ms |
| Processing duration p95 / maximum | 11.234 / 11.490 ms, 25 diagnostic samples |
| Static-cloud recorder arrival minus raw receive stamp, p95 | 10.505 ms |
| Core numeric fields finite | 8,689 / 8,689 detections |
| All-field `read_points(..., skip_nans=True)` retention | **0 / 8,689** |

The timestamps measure host delivery and computation, not acquisition-to-display
latency. Zero kernel drops do not establish zero network, SDK-assembly or DDS
loss. The regular received intervals and measured processing times do not
suggest a large processing backlog in this sample.

| Stage, on the same 473 scans | Total points | Points at 6 ≤ range < 10 m |
| --- | ---: | ---: |
| Raw | 8,675 | 1,371 |
| Quality gates | 8,675 | 1,371 |
| Doppler inliers | 8,631 | 1,334 |
| Doppler outliers | 44 | 37 |
| Retained movers | 26 | 22 |
| Suspected moving ghosts | 18 | 15 |
| Track-associated measured points | 0 | 0 |
| Unclassified after failed fit | 0 | 0 |

All emitted point records in these subsets matched records from their same raw
scan; the partition counts agreed. About 99.5% of raw points reached the static
layer. The sample is unlabeled: the 18 rejected movers are **suspected** ghosts,
and zero track-associated points is not evidence of missing people. These counts
cannot be used as detection precision/recall or compared across scenes as an
antenna improvement.

All four variance fields were finite and positive. Raw Pfa and flags were zero
for all 8,667 detections with a quality side message. Acquisition setup was 49.
Minimum SNR was 16.918 dB, above the 6 dB quality gate. Raising SNR to 30 dB
would retain only 64.2% of this sample, without evidence of preferential ghost
removal. Calibration/validity of RCS, variances, Pfa and flags remains unresolved.

Artifacts are under workspace `results/umrr96-ingest-research-20260928/`:
original `live/`, reindexed `recovered/`, `assessment.json`, `pipeline.json`,
and `check_pipeline.py`. The existing
[`assess_bag.py`](../tools/assessment/assess_bag.py) produced the field statistics;
the new offline script compares pipeline subsets and reproduces the synthetic
cases below. The [compact evidence report](umrr96-ingest-filtering-evidence-20260928.json)
retains source hashes and the principal measurements in the repository.

## Filtering opportunities compatible with current-scan display

**Use only required fields for validity checks.** The all-field NaN trap above
comes from the intentionally unavailable cloud Pfa field. Consumers needing
XYZ, speed and SNR should select those fields and validate them, rather than
drop a point for unavailable optional metadata. Our current processing adapter
already does this correctly:

```python
points = read_points(cloud,
                     field_names=('x', 'y', 'z', 'radial_speed', 'snr'),
                     skip_nans=True)
```

**Make rejection explainable and reversible.** Existing stage topics preserve
point bytes, but do not carry per-point rejection reasons and original raw
indices. Add a diagnostic classification alongside each raw scan: source index,
stage/reason, and whether a measurement is current, ambiguous or associated
with a track. Join the quality sidecar by stamp/frame and raw index, never by
position or `peak_idx`. Keep uncertain returns visible as current measurements;
a heuristic label should not become a calibrated confidence probability.

**Address the two-mover counterexample before stronger ghost filtering.**
[`ghosts.py`](../smartmicro_processing/smartmicro_processing/ghosts.py) rejects
a farther mover matching another mover's absolute speed, or approximately twice
it, without requiring a related bearing. The track-level rule in
[`tracker.py`](../smartmicro_processing/smartmicro_processing/tracker.py) has
the same issue. Offline synthetic inputs with direct returns at 2 m/0° and
5 m/60° gave:

| Near/far radial speed (m/s) | Current ghost mask: near, far |
| --- | --- |
| +0.5 / +0.5 | false, **true** |
| +0.5 / −0.5 | false, **true** |
| +0.5 / +1.0 | false, **true** |
| +0.5 / +2.0 | false, false |

These are constructed counterexamples, not a measured live false-negative
rate. Similar speed is insufficient proof of a reflection path. A candidate
replacement should retain independent motion hypotheses and require additional
geometry/trajectory evidence before suppression. Merely adding a bearing or
sign gate is not established as sufficient: earlier one-person replay showed
that legitimate multipath can have another bearing or opposite Doppler sign.

**Use Doppler in grouping and association.** Current clustering is spatial
single linkage with a 0.6 m radius. Two points 0.2 m apart are grouped even when
their speeds are +0.5 and −0.5 m/s. The association is greedy nearest-neighbor,
with fixed position/radial-speed noise. A useful experiment is scaled XY/range
and signed-Doppler grouping, singleton retention, and joint position/velocity
association. Gates must account for radial projection and limb motion;
matching equal Doppler is not itself an object identity test.

**Exploit the explicitly fixed installation as an optional mode.** If the
operator guarantees the radar remains fixed, zero sensor velocity is known.
An explicit fixed-radar mode could classify radial motion without estimating
ego velocity from every sparse scene. Compare it against the existing RANSAC
fit under few-static-return and many-moving-return cases. Do not infer that the
radar is fixed merely from near-zero Doppler; pure rotation is a counterexample.

**Calibrate uncertainty before weighting by it.** Preserve the reported
variances, establish their units and validity with smartmicro, then compare
their predicted spread against repeated isolated-reflector measurements at
several ranges and bearings. Propagate polar uncertainty into Cartesian gates
with the coordinate-transform Jacobian and retain an empirically justified
noise floor. A range-dependent gate is a candidate for improving association;
it does not create angular resolution or additional measured points.

Temporal state can inform the classification of the **current** scan without
republishing historical points. Any new display path should keep each displayed
measurement tied to the current input stamp. The saved static layer has zero
decay, while moving layers still have 0.5 s decay; track state also coasts for
up to 1 s. Those existing mechanisms differ from the rejected one-second static
history, but should be made explicit in a future instantaneous-view preset.
No display settings were changed by this research.

## Relevant primary research

| Source | Method and applicability to this radar |
| --- | --- |
| [Chamseddine et al., 2021, ghost classification](https://www.dfki.de/fileadmin/user_upload/import/11198_Chamseddine2021Ghost.pdf) | Point-cloud neural classification using radar features; lidar supplies training labels. The inference concept fits target-level data, but requires suitable labels and validation on this sensor; it is not a pretrained solution for our room |
| [Liu et al., 2025, trajectory-guided ghost suppression](https://doi.org/10.3390/s25113377) | Combines point features with past trajectory features and classifies current detections. Uses a 30-frame trajectory buffer and different radar hardware. Useful architectural precedent for internal temporal evidence without drawing old points; startup behavior, latency and generalization still need measurement |
| [Fatseas et al., PointNet++/DBSCAN optimization](https://ris.utwente.nl/ws/portalfiles/portal/479252327/Optimizing_PointNet_and_DBSCAN_for_Object_Detection_in_Automotive_Radar_Point_Clouds.pdf) | Uses radar-specific features and scaled XY/velocity clustering. Supports testing Doppler-aware grouping; their class-specific optimization on RadarScenes does not supply parameters for our unlabeled UMRR-96 data |
| [Zheng et al., 2024, multipath detection](https://arxiv.org/abs/2309.13585) | Uses transmit/receive angular structure to distinguish direct and indirect paths. We cannot implement that signal-level test from the present target list, which lacks complex per-antenna samples and separate departure-angle observations |

These papers motivate experiments. Their accuracy figures are not transferred
to this hardware or used as expected performance.

## Recommended implementation and validation order

1. **Add a filter audit and current-scan classification view.** Account for raw,
   quality, static, moving, ambiguous, suspected-ghost and track-associated
   points with reasons and raw indices. Preserve raw bytes and expose failed-fit
   returns. Verify counts/partitions and that displayed points belong to the
   current scan; measure p95/p99 processing and output age separately.
2. **Replace unconditional ghost suppression in an offline branch.** Evaluate
   both point and track rules on synthetic counterexamples and held-out
   recordings of two independent movers, opposite directions, crossing,
   standing and wall reflections. Report true-return retention versus ghost
   retention by range/bearing, time to first detection and track fragmentation.
   Keep the existing single-person bags as regression cases, not the test set
   used to choose every parameter.
3. **Compare fixed-radar classification and Doppler-aware association.** Test
   these independently so gains have an identifiable cause. Preserve isolated
   detections and measure the missed-return cost of confirmation. Use temporal
   evidence internally while maintaining the requested current-scan output.
4. **Complete the vendor field/output contract.** Ask about variance/RCS units,
   Pfa/flags validity and definitions, acquisition setup/timestamp encoding,
   target-list caps, available detection/clutter controls, and supported
   peak-list/raw/native-object outputs on serial 230739. Include the outstanding
   antenna/interleaving questions. No vendor message was sent.

Larger receive queues, arbitrary SNR increases, point upsampling, and enabling
the compact stream do not have evidence here as ways to improve measured wall
coverage. Packet-level replay and per-stage counters remain useful for auditing
future changes, especially SDK assembly and DDS loss that kernel counters alone
cannot detect.

## Filter-audit implementation follow-up

After the operator asked to continue, implemented the first recommendation:
[`DetectionAudit`](../umrr_ros2_msgs/msg/DetectionAudit.msg) accounts for each raw
row with its classification, rejection flags and measured-point track
association. The separate `classified_targets` cloud renders current positions,
including drawable quality rejects, suspected ghosts and failed-fit returns.
Raw and existing subset clouds retain their fields and bytes. Stale/rejected
input and clock resets clear the display and emit a distinct audit `CLEAR`
event, rather than inventing an empty observation.

The tracking launch now defaults to
[`umrr96_classified.rviz`](../smartmicro_processing/rviz/umrr96_classified.rviz).
Its one enabled point cloud has zero decay; every other point-cloud display also
has zero decay, and track estimates are disabled. This does not add scan history
or predicted positions to the measurement display. See the
[color legend and inspection instructions](../smartmicro_processing/README.md).
No sensor commands or live-session restart were performed after shutdown.

Validation:

- Built both interface and processing packages; all 105 processing pytest cases
  and both packages' configured lint checks pass. ROS integration cases cover
  normal/simulated time, failed fits, invalid input, disconnect clearing and
  backward-clock recovery in isolated loopback DDS domains.
- The installed RViz RGB8 transformer decoded all six display colors from a
  serialized generated cloud. Preset checks confirm one enabled measurement
  cloud, zero decay and disabled track estimates. This is not a live GUI review.
- Replayed the same 474 saved scans through the prior `6a0e9a6` implementations
  and the new code with identical configuration and fresh tracker state. All
  seven existing subset clouds, tracked objects and obstacles matched exactly
  in headers, schema and record bytes. All 8,689 raw points received audit
  entries and current-scan display positions; no provenance errors were found.
- The classifications were 8,645 static, 26 retained moving and 18 suspected
  ghosts, all 18 triggering the behind-static rule. These are algorithm outputs,
  not labeled accuracy measurements.
- Offline callback compute p95 changed from 9.20 to 9.93 ms and p99 from 10.09
  to 10.54 ms in this comparison. Publication sinks replaced DDS, so these are
  neither end-to-end latency nor output-age measurements. The recovered partial
  capture and unlabeled scene also limit what this replay establishes.

The [replay report](umrr96-classification-replay-20260928.json) includes counts,
timings and source hashes. Harnesses and the RGB compatibility probe are retained
under workspace `results/umrr96-classification-20260928/`. The next filtering
experiment remains multi-mover ghost-rule evaluation; this implementation makes
those decisions visible without claiming to have corrected them.

## Live follow-up after power-up

The operator reported the radar was back. Its Ethernet neighbor was reachable,
and the previous tracking session had exited, so the updated tracking launch
started on domain 96. The normal startup reapplied and verified volatile CAN
target output off; RViz read back short sweep 2, antenna 0 and firmware 5.2.2.
RViz loaded the classified preset and subscribed to `classified_targets`.

The [live report](umrr96-classification-live-20260928.json) covers 20 seconds:
364 raw scans received, 363 complete raw/audit/display joins, 18.18 Hz and 23.25
mean detections per matched scan. All 8,439 joined points had matching original
indices, current positions, classification/reason fields and RGB colors, with
no validation errors or clear events. All were classified static, so this
sample does not exercise moving/ghost classes. All 20 sampled UDP diagnostics
reported zero kernel drops; this does not prove absence of loss elsewhere.

Receive-stamp-to-observer delay for the display cloud was p95 11.19 ms and p99
12.62 ms. These include processing and DDS delivery to the verification node,
but exclude sensor acquisition latency and RViz render latency. The RViz window
was mapped and its subscriber was present; a desktop screenshot attempt returned
black, so no visual-quality claim is based on that capture. Launch logs and the
full verification report are in workspace
`results/umrr96-classification-live-20260928/`.
