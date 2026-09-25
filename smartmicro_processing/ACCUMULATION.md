# Temporal radar evidence and stationary preview

The second processing iteration accumulates **measured returns** over a short,
bounded window. One representative per 3D voxel carries a scan-support count,
age and source timestamp/index. It provides denser recent evidence for evaluating
radar's contribution alongside lidar/camera/IMU. The voxel size is a data reduction
setting, not a claim of radar resolution or map accuracy.

Build/source `smartmicro_processing` using the [package instructions](README.md).
For real timestamped `odom -> umrr96` transforms, launch:

```bash
ros2 launch smartmicro_processing umrr96_accumulation.launch.py \
  fixed_frame:=odom
```

The default input is `/umrr96_processing/doppler_inliers`; start the Doppler node
first if it is not already running. The accumulator transforms **each scan at
that scan's exact header timestamp**. It waits briefly for delayed TF, with bounded
pending memory and no blocking wait inside a callback. Missing/extrapolated TF
drops the affected scan after the timeout. There is no latest-transform or
identity fallback. With only the standalone sensor URDF, the default output stays
empty until a real pose chain exists. No TF or odometry is published by this node.

For a sensor physically held stationary, an explicitly separate preview needs
no odometry. Do not run another instance with the same node name if it is already up:

```bash
ros2 launch smartmicro_processing umrr96_accumulation.launch.py \
  mode:=stationary_preview rviz:=true
```

This publishes in `umrr96`, labels diagnostics `stationary_preview` and sets
`motion_compensated=False`. Keep the sensor still. If it moves, the accumulated
scene smears; stop/reset the preview or use calibrated pose compensation. This
mode creates no world frame. The preview viewer displays accumulated points
coloured by scan support, current inliers in white, the sensor model and an
optional age-coloured confirmed cloud. The data window handles expiration;
RViz's own point-cloud decay is zero so it does not accumulate a second history.
Closing this viewer leaves the independent radar and processing nodes running.

For a preview already running, open only its viewer:

```bash
rviz2 -d src/smartmicro_ros2_radars/smartmicro_processing/rviz/umrr96_accumulation.rviz
```

Output names are deliberately separate:

| Mode | Accumulated cloud | Confirmed cloud | Frame |
| --- | --- | --- | --- |
| Pose compensated | `/umrr96_accumulation/accumulated_targets` | `/umrr96_accumulation/confirmed_targets` | Configured fixed frame, default `odom` |
| Stationary preview | `/umrr96_accumulation/stationary_preview/accumulated_targets` | `/umrr96_accumulation/stationary_preview/confirmed_targets` | Input sensor frame, default `umrr96` |

Both outputs use sensor-data QoS (Best Effort). `node_name` can distinguish
concurrent comparisons; `input_topic`, `expected_frame_id`, `fixed_frame`,
`params_file` and `use_sim_time` are launch arguments. The supplied RViz view is
for stationary preview and opens with `rviz:=true` in that mode. Configure a
PointCloud2 display in the actual fixed frame when testing pose compensation.

Within each scan, duplicate points in a voxel contribute **one vote**. The first
measured point in that voxel represents the scan; the newest scan's representative
is published. There is no averaging into a synthetic point or expansion into a
surface. Votes count distinct scans within the window and are still temporally
correlated. A confirmed cell has support from at least two scans by default.
The accumulated cloud exposes first-scan evidence immediately for comparison;
confirmation is not imposed on the existing raw or quality obstacle streams.

The derived cloud schema is:

| Fields | Meaning |
| --- | --- |
| `x`, `y`, `z` | Newest representative's transformed coordinates, float32 metres in the output frame |
| `source_radial_speed`, `source_snr` | Original scalar measurements in the source sensor convention, float64; not a world-frame velocity or calibrated confidence |
| `age_seconds` | Output header stamp minus representative's source stamp, floored at zero within the allowed future tolerance |
| `span_seconds` | Time between oldest and newest supporting scans still retained in this cell |
| `support_scans` | Number of supporting scans currently retained |
| `source_stamp_sec`, `source_stamp_nanosec`, `source_point_index` | Identify the representative in the input topic's original scan, before finite-point selection |

XYZ uses float32 for compatibility with
[RViz's XYZ point-cloud transformer](https://github.com/ros2/rviz/blob/rolling/rviz_default_plugins/src/rviz_default_plugins/displays/pointcloud/transformers/xyz_pc_transformer.cpp).
Use a nearby local odometry origin to limit float32 coordinate rounding.
Internal transformations and voxel selection use double precision. Unsupported
or nonfinite coordinates are rejected. Original optional RCS/variance/flag values
remain available by source stamp/index in the input/raw recording; this derived
cloud does not assign physical semantics to them.

The [default parameters](config/umrr96_accumulation.yaml) retain 0.5 s of history
in 0.25 m cells, at most 32 scans and 8,192 per-scan voxel representatives.
Pending TF is limited to 32 scans and 0.2 s. Input stamps older than 0.5 s or more
than 0.05 s ahead are rejected, as are wrong frames and repeated/backward stamps.
Expired observations cannot be renewed by delayed TF. Capacity eviction removes
oldest scans; an oversized incoming scan retains its first bounded set of cells.
All capacity losses are counted in diagnostics. Raise limits only after measuring
memory, computation and output age on the intended scene.

Every output cycle expires observations by ROS timestamp. A steady-clock watchdog
also clears all evidence and pending input after 0.5 s without an accepted scan,
including when `/clock` stops. Empty inlier scans cannot renew old observations.
Backward ROS-clock jumps clear history, pending input and the TF cache. A per-scan
pose step over 1 m or 0.5 rad clears accumulated history before storing the new
scan. These configurable step limits are discontinuity heuristics, not calibrated
motion bounds; smaller coordinate resets and bad poses can evade them. TF carries
no pose covariance or reset identifier. The pose owner should request an explicit
history reset on relocalization, calibration or coordinate-system changes:

```bash
ros2 service call /umrr96_accumulation/reset std_srvs/srv/Empty '{}'
```

The service also clears pending input and cached poses. `/diagnostics` reports
`/umrr96_accumulation/evidence`: mode, compensation status, pending/processed
scans, rejection reasons, retained observations, cell/support counts and resets.
An OK state means scans were transformed and accumulated; `calibrated=False`
remains explicit. Empty cells convey **unknown**, and these counts are not
occupancy probabilities. No free-space ray clearing or Nav2 costmap modification
is implemented. A future Nav2 layer must own explicit evidence expiry in its
costs; publishing an empty cloud alone does not erase already marked costmap cells.

Run the tests with the package's documented `colcon test` command. They include
known translated/rotated landmark geometry, same-scan duplicate suppression,
expiry/confirmation, memory limits, exact-time TF interpolation, unavailable
future TF, delayed TF, pose/clock resets, preview topic separation and pause-time
staleness. Synthetic transforms stay inside an isolated ROS domain.

The existing recording contains no calibrated poses. A conditional stationary
comparison can evaluate cell count/support and lifetime across window sizes:

```bash
ros2 run smartmicro_processing umrr96_accumulation_audit \
  results/umrr96-investigation-20260924/live --stationary-preview \
  --windows 0.3 0.5 1.0 --output /tmp/umrr96-accumulation-replay.json
```

The audit calls the same quality adapter, Doppler fitter and accumulation core,
records its parameters and source hashes, and refuses an existing output file.
It never fabricates a trajectory. Cell-count gains include noise-driven boundary
crossings and repeated observations; they are not an independent measurement of
new surface coverage or accuracy. Compare windows at matched false-obstacle rates
once surveyed targets, calibrated time/extrinsics and reference poses are available.

Moving/tangential targets and coherent multipath can pass the Doppler gate.
Confirmation and TTL limit their persistence but do not prove static-world truth.
Per-point acquisition timing, pose/angle uncertainty propagation, radiometric
calibration, empirical motion accuracy and navigation benefit remain follow-up
experiments in the [validation plan](../docs/umrr96-navigation-investigation.md).

Verification completed on September 24, 2026 (local time). All **48 pytest cases**
in the processing package pass, including 19 new accumulation cases. The
[808-scan replay](../docs/umrr96-accumulation-replay.json) contains an average of
26.55 instantaneous 0.25 m cells. Under the explicit stationary-preview assumption:

| Window | Mean recent cells | Mean cells supported by ≥2 scans |
| --- | ---: | ---: |
| 0.3 s | 68.03 | 30.48 |
| 0.5 s | 93.60 | 42.04 |
| 1.0 s | 143.45 | 65.49 |

All three histories are empty at the last scan plus their window, with no capacity
losses. The old bag does not independently establish sensor motion or geometric
accuracy; keep these counts separate from physically validated coverage.

The user confirmed the sensor was stationary for the new
[live check](../docs/umrr96-accumulation-live.json), starting at 2026-09-25 00:03 UTC.
Across 182 input frames and 100 preview snapshots, a 0.5 s window produced **50.66
mean recent cells and 33.84 confirmed cells**, versus 22.81 instantaneous cells.
All 5,066 published representatives matched their source coordinates, Doppler and
SNR exactly; source ages and 20,490 support votes were independently checked.
A temporary pose-compensated node produced 102 empty clouds, zero transformed
scans and explicit TF timeouts because the radar had no odometry chain. It
published no TF and was stopped after the check. No synthetic pose was introduced
into the live ROS domain.

The stationary preview and its RViz viewer remain running. Session PID, overlay
and log are in `/tmp/umrr96-accumulation-live/session.json`. The existing driver,
description and Doppler nodes continued running throughout the check. Stop or
reset this preview before moving the sensor, and use measured poses for a moving
platform. The live and recorded scenes have different counts; this is not an
interleaved hardware or filter-accuracy comparison.
