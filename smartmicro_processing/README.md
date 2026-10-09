# Experimental UMRR-96 processing

This package provides a detection adapter, robust **3D radar-origin translation
velocity** and [bounded temporal evidence](ACCUMULATION.md). It builds independently of the proprietary
SDK and runs alongside the existing driver. It subscribes to raw targets and
never sends sensor commands. The existing driver, raw/filtered topics, TF and
state estimator retain their roles.

The [accumulation guide](ACCUMULATION.md) covers timestamped pose compensation,
the explicitly stationary live preview, per-cell support/age, expiry and RViz.
The commands and velocity contract below describe the first Doppler iteration.

Build and run from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
colcon --log-base .colcon/umrr96-processing/log build \
  --base-paths src/smartmicro_ros2_radars/smartmicro_processing \
    src/smartmicro_ros2_radars/umrr_ros2_msgs \
  --build-base .colcon/umrr96-processing/build \
  --install-base .colcon/umrr96-processing/install \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source .colcon/umrr96-processing/install/local_setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch smartmicro_processing umrr96_processing.launch.py
```

To start the driver (with readback, views and the sensor URDF), this node and
the current-scan RViz view in one go — closing RViz stops everything:

```bash
ros2 launch smartmicro_processing umrr96_tracking.launch.py
# rviz:=false for headless; publish_description:=false if a robot URDF owns the TF;
# driver_params:=... / processing_params:=... for other hosts or tuning.
```

The default view is now `rviz/umrr96_classified.rviz`: one cloud containing only
the latest scan, colored by the current filter decisions. All point-cloud decay
times in that preset are zero. Track estimates and the older stage displays are
available but disabled. The previous view remains selectable with
`rviz_config:=/absolute/path/to/umrr96_moving.rviz`.

| Current-detection color | Meaning |
| --- | --- |
| White | Static-scene Doppler inlier |
| Pink | Retained Doppler outlier, not associated with a confirmed track |
| Red | Retained measured point associated with a confirmed track |
| Blue | Suspected ghost rejected by an existing heuristic |
| Amber | Quality return whose scan's Doppler fit failed |
| Gray | Rejected by a required-field, range or SNR gate; position is still drawable |

These are algorithm labels, not ground truth. Blue points remain visible because
the current ghost rules can also reject independent movers. Every shown position
comes from the current raw scan; no historical points or predicted track positions
are inserted. A failed fit still shows its quality returns in amber. Positions
that are nonfinite or cannot be represented as float32 cannot be drawn but retain
their audit entries. The display is an XYZ/color adapter; raw and existing subset
clouds retain their original records and fields.

The default input is the relative name `smart_radar/port_targets_0`
(`/smart_radar/port_targets_0` without a namespace) with frame `umrr96`. Launch
arguments `namespace`, `input_topic`, `expected_frame_id`, `params_file` and
`use_sim_time` support other sources, several radars and bag replay; the launch
file remaps the relative input name to `input_topic`, and the `input_topic`
parameter still works in existing parameter files. The YAML is keyed
`/**/umrr96_processing`, so it applies in any namespace. Configuration is
validated and read-only after startup; every parameter carries a description
and range (`ros2 param describe`), and floating-point parameters accept integer
values such as `max_range: 120`. Copy [the YAML](config/umrr96_processing.yaml) to compare settings
in separate runs. Run one publisher for these output names at a time.

### Opt-in tracker experiments

These independent startup options default to **false** in Python and the shipped
YAML. They can be combined; `/diagnostics` reports the selected settings.

| Parameter / launch argument | When enabled |
| --- | --- |
| `evidence_confirmation` | Six sufficiently consistent position/Doppler hits can confirm early; the existing 8-of-10 fallback remains. |
| `standing_support` | Current static returns outside a saved background can support an anchored stopped mover, up to `standing_hold` (30 s). Requires a warmed background in a fixed scene frame; sensor movement resets it. |
| `joint_association` | Match tracks to measurements jointly by distance, one-to-one, with confirmed-track priority and the existing distance gate. |

For example, enable standing support with the live view:

```bash
ros2 launch smartmicro_processing umrr96_tracking.launch.py standing_support:=true
```

Or enable all three for a processing-only run:

```bash
ros2 launch smartmicro_processing umrr96_processing.launch.py \
  evidence_confirmation:=true standing_support:=true joint_association:=true
```

Both launch files accept each switch as `true`, `false`, or `config` (the launch
default). `config` inherits the parameter file; an explicit boolean overrides
it. The same ROS parameters work with `ros2 run ... --ros-args -p name:=true`.
They are read-only after startup, so restart processing to change them.

These options affect track confirmation, support and assignment; they do not
create extra radar detections. The current-scan RViz cloud still has zero point
decay. Standing support needs current returns to renew a track; internal
coasting remains bounded by the existing `max_coast` (1 s).

`association_uncertainty` is a separate, default-off research submode requiring
`joint_association`. It **regressed on the recorded walk**; leave it false for
the distance-assignment experiment. `association_doppler` (default true) has an
effect only when both association switches are enabled. SciPy supplies the
assignment solver and is declared as a package dependency.

See the [original experiment comparison](../docs/umrr96-worktree-comparison-20260928.md)
and [merged-option validation](../docs/umrr96-opt-in-integration-20260928.md).

| Output under `/umrr96_processing/` | Meaning |
| --- | --- |
| `detection_audit` | `umrr_ros2_msgs/DetectionAudit`: one decision per original point, raw indices, quality/ghost reason flags, and current track association |
| `classified_targets` | Current finite positions, RGB colors and audit fields for RViz; includes drawable quality rejects, suspected ghosts and failed-fit returns |
| `quality_targets` | Finite XYZ/Doppler/SNR, range and modest SNR gates; keeps both static and moving returns |
| `doppler_inliers` | Quality targets compatible with the fitted static-scene Doppler model |
| `doppler_outliers` | Quality targets outside that model's residual gate; not automatically moving objects |
| `moving_targets` | Doppler outliers that pass single-scan multipath-ghost rejection (`range_gap`, `speed_tolerance`, `wall_azimuth_deg`, `reject_static_only`) |
| `moving_ghosts` | Doppler outliers rejected by the active ghost policy. By default, any same-absolute-speed, double-absolute-speed or behind-static trigger rejects. Speed-copy rules ignore bearing and Doppler sign, so two independent movers more than `range_gap` apart in range can suppress the farther one. See the [criteria comparison](../docs/umrr96-rejection-criteria-20260928.md) for recorded tradeoffs. |
| `tracked_targets` | `moving_targets` within `track_radius` of a confirmed track: the ghost-resistant moving-object cloud (lags a new object by the ~0.4 s confirmation) |
| `tracked_objects` | Confirmed moving-object tracks: x, y, z, vx, vy, speed, track_id, age (sensor frame). vx, vy and speed are velocity **relative to the radar** in sensor axes; they equal ground velocity only while the radar is stationary |
| `track_markers` | RViz markers for the tracks (built only with subscribers); labels and arrows show the same radar-relative velocity |
| `obstacles` | Nav2 marking evidence: persistent static returns, ghost-filtered movers on tracks, track positions, non-ghost returns within `safety_range`; z flattened to `obstacle_height`. Novel static returns more than `shadow_gap` beyond a confirmed track and within `shadow_half_angle_deg` (15°) of its bearing are dropped as its multipath, only once the background is learned (`background_warmup`) and never while the sensor moves (see [track shadow](#track-shadow-rule)) |
| `unclassified_targets` | Quality targets when the velocity fit is rejected |
| `experimental_velocity` | `TwistWithCovarianceStamped` at the input stamp/frame, published only for accepted numerical fits |

`/diagnostics` includes `/umrr96_processing/doppler`, rejection reasons, counts,
condition, residual RMSE, computation time, last velocity age and
`calibrated=False`, `sensor_moving`, `background_ready` and `background_gap_resets`. An OK diagnostic means numerical checks passed, not measured
accuracy. Inspect the clouds in RViz using PointCloud2 displays, sensor-data QoS
(Best Effort), and fixed frame `umrr96`. No additional TF publisher is required.

### Track shadow rule

A novel static return is dropped from `obstacles` as a confirmed track's
multipath only when it lies within `shadow_half_angle_deg` of that track's
bearing and more than `shadow_gap` beyond that track's range. The earlier rule
dropped every novel static return beyond the nearest track at any bearing, so
a second, standing person elsewhere in the field of view vanished while
someone else was tracked. `shadow_half_angle_deg: 180` restores that rule. Its
ghost suppression was measured on the `walk` capture (far ghost points
1.97 → 0.14 per scan, [filtering](../docs/umrr96-filtering-20260924.md)); the
effect of the 15° default on ghosts is unmeasured because that recording is no
longer available. Synthetic checks cover the geometry only.

### Tracking on a moving radar

The tracker runs in the radar frame. Its constant-velocity EKF is updated with
each mover cluster's measured Doppler (sign-adapted: radial speed relative to
the radar), because sensor-frame positions move at the object's velocity minus
the radar's. The ego-compensated residual (`uᵀ v_object`) only separates movers
from the static scene and feeds the same-speed ghost rules (`moving_ghosts` and
the track-level rule). On a stationary radar the two agree to within the fit's
velocity noise; in synthetic stationary scenes the change left track identities
unchanged and moved track states by at most 5 mm.

The background also restarts its warm-up when the input-stamp gap between valid
fits exceeds `stale_timeout` (a data gap or a run of failed fits leaves sensor
motion unknown); `background_gap_resets` counts these restarts.

While the radar is moving (`sensor_moving`), the background is reset and every
static return would be novel, so confirmed tracks get no zero-Doppler
(`static_hold`) support: a track whose mover stops or leaves ends after
`max_coast`. A person standing still in front of a moving radar is not tracked;
they reach `obstacles` only through static persistence or `safety_range`.

### Inspecting a filter decision

Select a point in the current-detections RViz cloud to inspect `source_index`,
`classification`, `reason_flags` and `track_associated`. The typed audit is also
available without RViz:

```bash
ros2 topic echo /umrr96_processing/detection_audit --once
ros2 interface show umrr_ros2_msgs/msg/DetectionAudit
```

Match the audit to the raw cloud by **header stamp and frame ID**. `source_index`
is the original row-major point index, and all audit arrays have that raw scan's
point count, including quality rejects. Use the same raw index to access the
driver's `umrr96_raw_quality_0` sidecar; a filtered index or `peak_idx` is not a
substitute. The display's source indices skip undrawable positions but continue
to refer to the original raw rows.

Quality reasons use the existing first-failure order: nonfinite required fields,
range, then SNR. A failed fit marks remaining points `UNCLASSIFIED/FIT_REJECTED`.
Ghost reasons identify the existing same-absolute-speed, doubled-absolute-speed,
and behind-static-return rules; multiple bits can be set. Diagnostic ghost-reason
counts can therefore overlap. Track association is a separate boolean and does
not claim a unique object identity. The underlying quality, Doppler, ghost,
tracker and obstacle decisions are unchanged by this audit feature.

### Comparing static-only rejection

The experimental `reject_static_only` parameter defaults to `true`, preserving
the established rejection policy. With `false`, a nearer static return alone
leaves a mover in `moving_targets`; same-speed or double-speed evidence still
rejects it. A single static detection does not establish an opaque wall.

To compare this option, copy the **complete** [parameter YAML](config/umrr96_processing.yaml),
change `reject_static_only: true` to `reject_static_only: false` in that copy,
and select it at startup:

```bash
ros2 launch smartmicro_processing umrr96_processing.launch.py \
  params_file:=/absolute/path/to/umrr96_processing_comparison.yaml
```

The parameter is read-only during a run. Retained points still carry
`GHOST_BEHIND_STATIC` in their audit, but their classification is `MOVING` and
their display color reflects retention. Consumers must use `classification`
for the decision, rather than testing `reason_flags != 0`. Diagnostics report
the active `reject_static_only` policy and the count `static_only_advisory`;
`ghost_behind_static` includes both advisory and rejected hypotheses.

In the [recorded comparison](../docs/umrr96-rejection-criteria-20260928.md), this
option retained nine additional person-proxy points and admitted 25 additional
ghost-proxy points; confirmed far-track presence was unchanged. The public
dataset showed a much larger loss of ghost suppression. This is an optional
tradeoff, not a validated replacement default. The tracker retains its separate
absolute-speed ghost rule. Stronger signed-speed, direction and two-return
support candidates remain offline experiments because they missed most ghosts
in the sparse UMRR recording. All variants use current-scan points; no point
history or RViz decay is added.

The criteria changes passed 119 processing pytest cases and package lint.
A separate default-policy replay preserved all existing outputs and current-scan
provenance over 474 scans / 8,689 detections. These offline checks do not establish
live latency, crossing-person accuracy or obstacle-layer safety.

### Scan lifecycle and measurement contract

An actual empty scan has `event=SCAN` and empty arrays. After previously published
data is cleared due to stale/rejected input or a clock reset, the audit sends one
`event=CLEAR` with empty arrays, the last accepted scan's stamp and the clearing
cause in `status`. It is a display invalidation, not another sensor observation.
Malformed or freshness-rejected inputs do not receive per-point assignments;
their whole-scan reason remains in diagnostics. The colored cloud clears on the
same watchdog/rejection transitions as existing outputs.

The September 28 implementation passed 105 processing pytest cases and package
lint. An [offline comparison](../docs/umrr96-classification-replay-20260928.json)
matched all existing subset, tracked-object and obstacle outputs over 474 saved
scans, accounting for all 8,689 current detections. The installed RViz RGB8
transformer decoded every display color correctly. The radar was powered down;
this initial check was offline. After power-up, a
[20-second live check](../docs/umrr96-classification-live-20260928.json) matched
363 scans and all 8,439 detections to their audit entries and display positions
at 18.18 Hz. RViz subscribed to the new cloud; receive-stamp-to-observer p95 was
11.19 ms. The sample contained only static classifications and does not establish
moving-object accuracy, acquisition latency or RViz render latency.

Only the five required fields affect input eligibility. An optional unknown
`false_alarm_probability=NaN` does not reject a valid detection. Selected point
records retain all source bytes, field definitions, timestamps and frame names;
output clouds are flattened and row padding is omitted. Both byte orders and
padded input rows are handled without mutating the source. Unknown quality,
SDK variances, flags and RCS are preserved, not treated as calibrated weights.
Range gates use the norm of XYZ in metres; no absolute-Doppler gate removes
zero-speed landmarks. Empty/invalid scans remain observable through diagnostics.

For a world-static return with unit bearing `u`, the model is `d = -uᵀ v` with
**positive-receding** Doppler. `doppler_sign=1` assumes this is the source
convention; `-1` adapts positive-approaching input. Physical sign verification
is still required. RANSAC finds a majority consensus, then Huber-weighted least
squares refines it. Final residuals classify every quality target. The defaults
require eight inliers, at least 60% consensus and direction-matrix condition
at most 30. Scans with inadequate 3D bearing diversity, excessive fitted speed
or model uncertainty are rejected. There is no assumption of zero vertical
velocity, and no integration to a position or orientation estimate.

The 0.20 m/s residual threshold, 0.05 m/s Doppler noise floor and 0.10 m/s
velocity floor are experimental settings, not manufacturer accuracy claims.
Linear covariance uses the weighted bearing geometry and the larger of the
weighted residual variance, `Σ wᵢrᵢ² / (n − 3)` with the Huber weights evaluated
at the returned velocity, and the noise floor, plus the velocity-floor variance
on all three axes. It has **not** been calibrated for angular errors, timing, multipath,
correlated detections or model-selection bias. Angular velocity is unobserved:
its values are zero placeholders with variance `1e6`. A dominant moving object
or coherent multipath can satisfy the model and produce a wrong accepted fit.
Small residuals and apparent consensus cannot establish truth.

Freshness checks reject unexpected frames, malformed layouts, repeated/backward
stamps, stamps older than 0.5 s and stamps over 0.05 s in the future. A wall-clock
watchdog clears the output clouds and reports stale input after 0.5 s without data,
including when simulation time pauses. Clouds are cleared once, on the transition
from published data, with the last accepted input stamp rather than a newer
`now()`, so a downstream monotonic-stamp check still accepts the next scan.
Rejections are logged as throttled warnings. `/diagnostics` is published
immediately on a state change and otherwise at most once per
`diagnostics_period` (1 s). Invalid estimates publish **no twist**;
downstream consumers must enforce their own timestamp timeout and must not reuse
the last twist indefinitely. A backward ROS clock jump clears the timestamp
epoch so bag replay can recover. No zero-velocity replacement or pose/TF is
published on failures. Clouds retain the driver's receive-time stamps; no
unmeasured acquisition-time correction is applied.

This velocity is at the radar measurement origin in radar coordinates. Future
body-frame fusion needs the measured rotation, lever-arm correction during turns,
verified Doppler sign, acquisition-time alignment and empirical covariance
coverage. The standalone [sensor description](../smartmicro_description/README.md)
does not supply those calibrations. Keep the experimental topic separate from the
navigation estimator until controlled motion has been compared against an
independent reference.

Run the synthetic and ROS integration checks:

```bash
colcon --log-base .colcon/umrr96-processing/log test \
  --base-paths src/smartmicro_ros2_radars/smartmicro_processing \
    src/smartmicro_ros2_radars/umrr_ros2_msgs \
  --build-base .colcon/umrr96-processing/build \
  --install-base .colcon/umrr96-processing/install \
  --packages-select smartmicro_processing --event-handlers console_direct+
colcon test-result --test-result-base .colcon/umrr96-processing/build --verbose
```

Replay the recorded raw scans through the same adapter and fitter, without
starting a ROS node or replaying control topics:

```bash
ros2 run smartmicro_processing umrr96_processing_audit \
  results/umrr96-investigation-20260924/live \
  --output /tmp/umrr96-processing-replay.json
```

The audit uses the documented default gates/fit (or `--doppler-sign -1`), records
configuration and input hashes, and refuses to overwrite an existing report.
Historical bags bypass the live freshness checks. Fit residuals are reported
separately from accuracy; the current bag has no reference trajectory.

On 2026-09-24, 29 pytest cases passed: 27 adapter/numerical cases and two installed
ROS launch cases covering normal and simulation time, invalid frames/stamps,
disconnection and recovery after a backward clock jump. The
[recorded replay](../docs/umrr96-processing-replay-20260924.json) processed all 808
scans and retained all 21,451 returns as Doppler inliers. The
[10-second live check](../docs/umrr96-processing-live-20260924.json) matched all
181 input scans to the four output clouds at 18.10 Hz, verified byte-preserving
partitions and received 181 finite velocity estimates. Median processing time
was 7.51 ms; p95 was 8.78 ms. Receive-stamp age excludes unknown sensor latency.
These quiet-motion samples do not test physical dynamic-outlier rejection or
velocity accuracy. Known-motion and moving-return rejection are exercised by
synthetic tests only.

During that September 24 check, the standalone processing launch ran beside the
original driver; its PID, log and overlay were recorded in
`/tmp/umrr96-processing-live/session.json`. Use the existing publisher for
inspection only after verifying it is still active; the launch command above is
for starting a new session. The original radar driver and sensor settings were
not changed during that live check.

The second iteration adds 19 accumulation checks, bringing the package total to
48 pytest cases. Its separate [live and replay results](ACCUMULATION.md) quantify
recent-cell density and source provenance. The next experiments are controlled
forward/reverse and lateral motion to verify Doppler and velocity, measured
mounting/time calibration with IMU and geometric odometry, and physical evaluation
of pose compensation and radiometric weighting. These iterations do not increase
native per-scan detections or establish mapping/navigation accuracy. See the
[investigation and validation plan](../docs/umrr96-navigation-investigation.md).
