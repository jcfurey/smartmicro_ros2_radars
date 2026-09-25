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
  --build-base .colcon/umrr96-processing/build \
  --install-base .colcon/umrr96-processing/install \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source .colcon/umrr96-processing/install/local_setup.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch smartmicro_processing umrr96_processing.launch.py
```

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

| Output under `/umrr96_processing/` | Meaning |
| --- | --- |
| `quality_targets` | Finite XYZ/Doppler/SNR, range and modest SNR gates; keeps both static and moving returns |
| `doppler_inliers` | Quality targets compatible with the fitted static-scene Doppler model |
| `doppler_outliers` | Quality targets outside that model's residual gate; not automatically moving objects |
| `moving_targets` | Doppler outliers that pass single-scan multipath-ghost rejection (`range_gap`, `speed_tolerance`, `wall_azimuth_deg`) |
| `moving_ghosts` | Doppler outliers rejected as ghosts: a nearer same-speed mover, or a nearer static return at the same bearing. On a 2026-09-24 walk-through (stationary sensor, one person, 3 m room) the rules removed 85% of ghosts and kept 97% of real returns; about 23% of `moving_targets` remained ghosts. Two movers at the same speed and bearing lose the farther one. |
| `tracked_objects` | Confirmed moving-object tracks: x, y, z, vx, vy, speed, track_id, age (sensor frame) |
| `track_markers` | RViz markers for the tracks (built only with subscribers) |
| `obstacles` | Nav2 marking evidence: persistent static returns, ghost-filtered movers on tracks, track positions, non-ghost returns within `safety_range`; z flattened to `obstacle_height` |
| `unclassified_targets` | Quality targets when the velocity fit is rejected |
| `experimental_velocity` | `TwistWithCovarianceStamped` at the input stamp/frame, published only for accepted numerical fits |

`/diagnostics` includes `/umrr96_processing/doppler`, rejection reasons, counts,
condition, residual RMSE, computation time, last velocity age and
`calibrated=False`. An OK diagnostic means numerical checks passed, not measured
accuracy. Inspect the clouds in RViz using PointCloud2 displays, sensor-data QoS
(Best Effort), and fixed frame `umrr96`. No additional TF publisher is required.

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
watchdog clears the four clouds and reports stale input after 0.5 s without data,
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

The standalone processing launch remains running beside the original driver;
its PID, log and overlay are recorded in
`/tmp/umrr96-processing-live/session.json`. Use the existing publisher for
inspection; the launch command above is for starting a new session. The original
radar driver and sensor settings were not changed during this live check.

The second iteration adds 19 accumulation checks, bringing the package total to
48 pytest cases. Its separate [live and replay results](ACCUMULATION.md) quantify
recent-cell density and source provenance. The next experiments are controlled
forward/reverse and lateral motion to verify Doppler and velocity, measured
mounting/time calibration with IMU and geometric odometry, and physical evaluation
of pose compensation and radiometric weighting. These iterations do not increase
native per-scan detections or establish mapping/navigation accuracy. See the
[investigation and validation plan](../docs/umrr96-navigation-investigation.md).
