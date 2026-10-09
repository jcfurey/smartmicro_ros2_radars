# Output surface review (2026-10-09)

Scope: every topic and service the fork publishes at `cam_wip` `a1cb03a`, read
from code, against REP 103/105/2003, `radar_msgs`, Nav2's costmap layers and
`robot_localization`. Defects found on the way are in the
[code review](review-2026-09-24.md#review-2026-10-09) (C49–C70, S24–S29, M5–M6);
this page records the inventory and the planned output changes (O1–O14).
Upstream `master` was still `3dc5fcc` on this date.

## Inventory

Driver and views names are relative (`<ns>/smart_radar/...`); processing and
accumulation names are private (`<ns>/<node>/...`). Stamps: **rx** = ROS time in
the SDK callback (not acquisition time), **in** = copied from the input cloud.

| Topic (no namespace) | Producer | Type | Frame / stamp | QoS | When |
| --- | --- | --- | --- | --- | --- |
| `smart_radar/port_targets_N` | driver | PointCloud2, 72 B | `frame_id` / rx | reliable, depth `history_size`, overridable | every scan, 18.2 Hz (8.3 Hz with CAN output on) |
| `smart_radar/port_targetheader_N`, `timing_N` | driver | `PortTargetHeader`, `RadarTiming` | same | same | every scan |
| `smart_radar/umrr96_raw_quality_N` | driver | `Umrr96RawQuality` | cloud header | same | UMRR-96 Ethernet, with subscribers |
| `smart_radar/radar_scan_N` | driver | `radar_msgs/RadarScan` | cloud header | same | radar_msgs build + `publish_radar_scan` + subscribers |
| `smart_radar/{port,can}_objects_N` | driver | PointCloud2, 48 B | `frame_id` / rx | same | `pub_type: mse` models (not UMRR-96) |
| `smart_radar/filtered_targets_0`, `filter_status` | views | PointCloud2 subset, JSON `String` | in | best effort d5; status reliable transient-local | every scan |
| `smart_radar/fan_image`, `fan_targets`, `density_grid`, `density_cells`, `fan_guides` | views | Image rgb8 960×640, PointCloud2, Marker(Array) | sensor or `grid_frame_id` | reliable d1 | 10 Hz with subscribers; guides latched |
| `umrr96_processing/{quality_targets, doppler_inliers, doppler_outliers, unclassified_targets, moving_targets, moving_ghosts, tracked_targets}` | processing | PointCloud2, driver records | in | best effort d5 | every accepted scan |
| `umrr96_processing/classified_targets`, `detection_audit` | processing | PointCloud2 (xyz, rgb, audit), `DetectionAudit` | in | best effort | every accepted scan |
| `umrr96_processing/tracked_objects` | processing | PointCloud2: x, y, z, vx, vy, speed, track_id, age | in | best effort | every accepted scan |
| `umrr96_processing/obstacles` | processing | PointCloud2: x, y, z | in | best effort | every accepted scan |
| `umrr96_processing/experimental_velocity` | processing | `TwistWithCovarianceStamped` | in | reliable d10 | valid fits only |
| `umrr96_accumulation/.../{accumulated,confirmed}_targets` | accumulation | PointCloud2 with provenance fields | `odom` or sensor / newest scan | best effort d5 | 10 Hz with subscribers |
| `/diagnostics` | all | `DiagnosticArray` | — | — | about 1 Hz |

Seven RViz configurations ship across three packages, and the three
"what should a robot use" candidates (`filtered_targets_0`, `obstacles`,
`confirmed_targets`) are each recommended somewhere. Everything except the
accumulator is in the sensor frame and stamped at receive time.

## Problems for consumers

- **No contract.** About 25 topics from four processes; the views outputs sit
  under `smart_radar/` and look like driver topics but vanish with the views
  process. Eight processing subsets overlap `classified_targets` +
  `detection_audit`, and are built on every scan without subscribers.
- **Typed data in custom clouds.** Tracks are a PointCloud2 with no covariance
  (the EKF has one), no size, no tentative/confirmed status; MSE objects are the
  same (S22). `radar_msgs/RadarScan` exists but radar_msgs is not a declared
  dependency, so a rosdep install never builds it.
- **Sensor frame and receive time.** Tracks, obstacles and the twist are in
  `umrr96` at receive time (S21). Nav2, track consumers and `robot_localization`
  all look up TF at the stamp: at 1 rad/s, 100 ms of unmodelled latency is about
  1 m of lateral error at 10 m.
- **QoS.** Derived outputs are best effort: a default (reliable) subscriber gets
  nothing and only an incompatible-QoS warning. The Python nodes do not accept
  `qos_overrides` (the driver does since S20).
- **Diagnostics.** One device appears under three hardware IDs
  (`umrr96_v1_2_2@192.168.11.11`, `umrr96@192.168.11.11`, `umrr96`) and two naming
  styles, so the aggregator cannot group them. No rate/age monitor on the outputs
  a robot depends on; sensor fault reports never reach `/diagnostics` (S23).
- **Nav2.** `obstacles` only marks; ObstacleLayer users need another sensor to
  clear it, and there is no complete STVL example with decay.
- **Ego velocity.** The twist is at the radar origin in radar axes; angular terms
  are placeholders (variance 1e6) and vz is weakly observed (unreliable
  elevation), but its covariance does not say so. Radar-inertial estimators
  (e.g. DELIRIOM's planned radar term) need raw per-point Doppler with a verified
  sign and acquisition time more than a pre-fitted twist.
- **Operator.** `fan_image` is about 18 MB/s raw; `filter_status` is JSON at scan
  rate; RViz configs use absolute names (C39); `umrr96_grid.rviz` enables a
  RobotModel the default launch does not publish.

## Live observations (2026-10-09)

From the stationary live session and its 2,206-scan static recording
([evidence](umrr96-review-live-20261009.json)):

- **Twist covariance is the floor, not the data (O5).** Reported σ is about
  0.050 m/s on each axis (`velocity_std_floor`), while the scan-to-scan spread of
  the fit is 1.0, 1.0 and 4.8 mm/s (x, y, z): about 50× conservative in x/y, and
  z is about five times noisier than x/y. A per-axis, data-driven covariance
  needs a moving recording to calibrate.
- **Obstacle persistence works as designed.** It keeps 5–23 (mean 14) of about
  25 detections per scan: 94.5% of detections in persistent structure cells and
  37% of flicker detections, which are 69% of all detections in this room.
  Per-scan consumers (e.g. Nav2's collision monitor) still see the flicker
  that passes; an ObstacleLayer keeps marks until a clearing source clears them.
- **Latency (O6).** Receive-stamp age is 0.8 ms and processing adds about 10 ms
  (p95 11.5 ms); the sensor-internal latency before the SDK callback remains
  unknown.
- **Diagnostics (O7).** One device appeared as `umrr96_v1_2_2@192.168.11.11`
  (data node), `umrr96@192.168.11.11` (readback) and `umrr96` (processing).
  Fixed the same day: all three use `umrr96_v1_2_2@<ip>` (processing via its
  new `hardware_id` parameter); the adapter stays `udp@192.168.11.17:55555`.
- **RadarScan (O9)** works live at 18.18 Hz when built against radar_msgs; it is
  still not a declared dependency.

## Planned changes

Status: `planned`, `in progress`, `done`, `deferred`. Quick wins first.

| ID | Change | Notes / related | Status |
| --- | --- | --- | --- |
| O1 | **Robot contract** in `umrr_ros2_driver/doc/interfaces.md`: four integration topics — raw `port_targets_N`, `obstacles`, typed tracks, ego velocity — with producer, frame, stamp, QoS, rate and intended consumer. Move the views outputs to the views node's private namespace (keep `smart_radar/filtered_targets_0` as a deprecated alias); put audit/debug outputs under `debug/`. | Overlaps ros2-improvements item 1 | planned |
| O2 | **Reliable derived outputs:** KEEP_LAST(5) reliable publishers with `QoSOverridingOptions.with_default_policies()` in the Python nodes; subscriptions stay sensor-data QoS. | ros2-improvements item 4 | planned |
| O3 | **Typed tracks:** `radar_msgs/RadarTracks` from the tracker — `uuid` from `track_id`, position, velocity, upper-triangular covariances from `Track.P`, placeholder size, DYNAMIC classification. Same type for MSE objects (`object_id`, `speed_absolute·(cos, sin)(heading)`, `length`, `object_class`). Declare radar_msgs and unique_identifier_msgs. | S22; after O4 for ground velocities | planned |
| O4 | **Fixed-frame tracking:** tracker, persistence and background in `odom` via TF at each scan's stamp, giving ground velocities and a background that survives sensor motion. | C34 remainder; builds on C49, C51 | planned |
| O5 | **Fusable ego velocity:** planar (vx, vy) option or vz variance from elevation quality; documented `twist0_config`/rejection threshold for robot_localization; optional `base_link` output with lever-arm correction from an IMU yaw rate. | Needs measured `base_link → umrr96` | planned |
| O6 | **Timing:** `stamp_offset_s` latency parameter recorded in `RadarTiming` (quick); acquisition-time stamps from `acquisition_start` when the time base is SMS/PTP; latency calibration by Doppler-vs-IMU cross-correlation. | S21 | planned |
| O7 | **Diagnostics:** one hardware ID (`model@ip`) in every node; frequency + stamp-age monitors on `port_targets_N` and `obstacles`; fault reports as diagnostics once `criticality` semantics are known. | S23 | in progress (hardware IDs unified 2026-10-09) |
| O8 | **Nav2:** complete STVL example with decay as the primary radar-only configuration; investigate a radar layer that expires its own marks. | Navigation investigation | planned |
| O9 | **Driver cloud conveniences:** optional `intensity` alias (power or rcs) for PCL/LIO tools; set `is_dense` from actual XYZ validity; RadarScan on by default once the Doppler sign is verified (O14). | | planned |
| O10 | **Namespacing:** relative names in RViz configs, configurable panel endpoints, readback services under their own names (`umrr96_tuning/...`) instead of moving the driver's to `data_receiver/`. | C39, S26 | planned |
| O11 | **Launch split:** headless `robot.launch.py` (driver, readback, processing) and `viz.launch.py` (views, RViz, description); neutral parameter defaults with the bench file opt-in. | S19, M6 | planned |
| O12 | **Operator views:** one RViz configuration per launch; `fan_image` through image_transport and redrawn only on new scans; typed filter status. | P9, ros2-improvements items 2–3 | planned |
| O13 | **Processing topic consolidation:** subsets built only with subscribers; keep obstacles, tracks and velocity as consumer topics; drop the redundant `source_index`. | | planned |
| O14 | **Doppler sign verification** on hardware with a target at known velocity. Prerequisite for O3, O5 and O9's RadarScan default. | interfaces.md `radial_speed` | planned |

The UMRR-96 recordings used by earlier evaluations (`results/umrr96-*`) are not
on disk any more. New recordings (static, walk, two people, moving platform with
odometry) are needed to re-score processing defaults after O4.
