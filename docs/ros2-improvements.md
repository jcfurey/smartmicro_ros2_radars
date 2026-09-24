# ROS 2 driver improvements

Review date: 2026-09-24. Target: ROS 2 Lyrical, UMRR-96 Type 153 firmware
5.2.2, Ethernet data and a separate Ethernet control process.

## Evidence and priorities

A six-second live sample received 50 clouds, averaging 27.94 detections each.
Cloud timestamps were approximately 652 seconds while the receiving ROS clock
was approximately 1.79 billion seconds: the original driver exposed the radar's
device counter as if it were ROS time, preventing correct TF lookup and fusion.

The same sample measured approximately 0.33% of one CPU core for data reception,
0.17% for control, and 19.25% for the Python views process. These are short local
observations, not a performance benchmark. A 960 × 640 RGB image at 10 Hz has an
uncompressed payload of 18.4 MB/s. Optimize visualization before the low-cost
receive path unless longer measurements identify a different bottleneck.

The original data node also rewrote installed SDK JSON configuration. With a
symlink install this changed the source checkout and let concurrent processes
interfere. The control node already used a private temporary directory.

## First implementation batch

- [x] Use ROS receive time for outgoing message headers. Preserve the unmodified
  device microsecond counter separately and explicitly identify the timestamp
  source. Receive time is not synchronized acquisition time.
- [x] Give each data/control process its own SDK configuration directory, built
  from read-only templates and removed during normal destruction.
- [x] Mark startup-only connection/configuration parameters read-only, validate
  identifiers, ports and queue sizes, and document when a restart is required.
- [x] Publish standard diagnostics for stream activity, observed frequency,
  device timestamp behavior and control request failures/timeouts.
- [x] Add focused offline regression checks, build on Lyrical, then verify live
  timestamps, readback and diagnostics without changing sensor settings.

Preserve raw detections, selectable host filters, existing RViz layout, and the
separate data/control processes required by the vendor SDK. Avoid changing
hardware configuration as part of this batch.

## Follow-up work

The sensor-output priorities are tracked separately in
[UMRR-96 output improvements](umrr96-output-priorities.md), with implementation,
validation results and remaining hardware/vendor dependencies.

1. **Namespaces and deployment:** replace absolute view/panel topic and service
   names with relative names or configurable endpoints; make RViz optional;
   test two radars and headless launch.
2. **Visualization:** cache unchanged images, avoid building outputs without
   subscribers, avoid copying clouds with filtering disabled, reserve point
   buffers, and add image_transport for remote compressed viewing.
3. **Typed interfaces:** replace JSON filter status and JSON control results
   gradually with typed messages while retaining compatibility where needed.
4. **QoS:** expose supported overrides and test raw/filtered topics with RViz and
   rosbag. Choose reliability/depth deliberately; best effort is not mandatory
   for every publisher.
5. **Lifecycle and shutdown:** callback draining now replaces the shutdown delay,
   and initialization uses the SDK's synchronous result without a fixed sleep.
   SDK singleton ownership and retained callback code still prevent a claim of
   safe component unloading/reloading or lifecycle support.
6. **Regression automation:** extend the new colcon suite with disconnect/reconnect,
   complete launch/panel namespace coverage and a read-only installation fixture.
   Processing tests and concurrent data-process isolation are now covered; legacy
   Docker/network tests are opt-in separately.
7. **Data quality:** the [SDK audit](sdk-audit.md) verified four nonzero variance
   fields, peak indices and acquisition setup in captured UMRR-96 packets.
   Those are now published with provenance; confirm variance units/calibration and the
   meaning of zero flags/false-alarm probability before using them as confidence.
   For moving platforms, transform detections into a fixed frame before
   accumulating the density grid.
8. **SDK instruction correctness — completed:** extraction now applies the
   audited F32 conversion repair. CMake checks it, and ASan/UBSan tests exercise
   float bit patterns and unchanged integer conversions. No shared SDK binaries
   or physical sensor settings were changed.
9. **Cloud dependency simplification — completed:** two explicit builders replace
   the external wrapper and its submodule, retaining 72-byte target and 48-byte
   object strides. All 66 target/object callbacks reserve their list size and
   borrow the SDK list without copying it. Layout/byte tests cover every field,
   empty clouds, NaNs, integer limits and zero padding. See the
   [wrapper audit](point-cloud-wrapper-audit.md). This is
   principally a maintenance improvement, not a measured receive-path bottleneck.

## Validation record

The first batch adds `RadarTiming` on `smart_radar/timing_N` without changing the
existing cloud fields or custom header schemas. Matching messages share a ROS
receive stamp; raw device time remains a separate unsigned microsecond value.
It also fixes adapter/sensor counting when every configured array slot is used.
See [bringup behavior and diagnostics](umrr96-bringup.md#timestamps-runtime-configuration-and-diagnostics).

Build and focused checks from the workspace root:

```bash
source /opt/ros/lyrical/setup.bash
source install/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=6 colcon build \
  --base-paths src/smartmicro_ros2_radars \
  --packages-up-to umrr_ros2_driver smart_rviz_plugin --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --base-paths src/smartmicro_ros2_radars \
  --packages-select umrr_ros2_driver smart_rviz_plugin \
  --ctest-args -R 'test_runtime|test_point_cloud_builder|test_sdk_float|test_sdk_patch|test_driver_runtime|test_readback|test_umrr96_views|panel_loading_smoke|umrr96_panel_smoke' \
  --output-on-failure
```

The driver tests use UDP loopback and ROS domain 178 by default, selectable at
configure time with `SMARTMICRO_TEST_DOMAIN_ID`. They require the SDK but no
physical radar or Docker. Legacy network tests require
`-DSMARTMICRO_NETWORK_TESTS=ON` explicitly.

Result after wrapper removal on this Lyrical host: **34 focused tests passed** (32 driver/processing
tests and two RViz tests), with zero errors, failures or skips.

- C++ tests check independent temporary configuration, exception cleanup, stream
  silence/recovery and repeated/backward/zero device counters. New checks cover
  callback draining/late entries/exceptions and sanitized F32 bit conversions.
- Extraction-repair tests verify idempotence and rejection of unfamiliar code.
- Six cloud-builder tests pin both published schemas and serialized bytes,
  including integer signedness, NaN payloads, empty clouds and padding. A clean
  build succeeds without the wrapper package. Recorded old/new ROS replay
  produces identical data in all 18 target fields across 953 detections.
- SDK replay tests run two namespaced data processes on separate sockets, verify
  independent configuration, unchanged installed templates, clean shutdown and
  cleanup, read-only parameters, and advancing ROS stamps despite a repeated
  fixture timestamp. Target diagnostics transition from waiting to timestamp
  warning and then stale after replay stops. The updated fixture verifies all four
  reported variances, peak index and acquisition setup exactly, then shuts down
  the driver while replay traffic is still arriving.
- Readback tests check 20 malformed read/tuning requests, three timeouts,
  diagnostic counters, rejected parameter changes and invalid startup ranges.
- The existing 14 processing/image/filter tests and two RViz panel tests pass.
- A post-fix ROS replay of the earlier sensor capture preserved all four variance
  fields and peak indices exactly across 30 frames and 953 detections, with
  acquisition setup 49 throughout and normal shutdown/configuration cleanup.
- Before the SDK fixes, a six-second physical-sensor sample matched all 50 clouds with their custom
  headers and timing messages, preserving all 18 point fields. Subscriber receipt
  was 0.27–0.78 ms after the ROS stamp (mean 0.52 ms). This measures local ROS
  delivery, **not sensor acquisition latency**. The raw counter was about
  2014.8 seconds while the ROS header was in the host clock epoch.
- Both diagnostic tasks reported `OK`; the target task observed approximately
  8–9 Hz with no timestamp anomalies. All four tuning reads succeeded; sweep 2,
  range switching 0, antenna 0 and CAN targets 1 were unchanged. The Moving
  returns host filter was retained. The image topic continued publishing.

Known limits: only UMRR-96 Ethernet is verified on hardware; other model/CAN
callbacks use the shared timestamp path but still need hardware validation.
Receive time is not synchronized acquisition time. Callback gates now protect
owner state, but SDK singletons and shared-library unloading remain unresolved.
Control diagnostics may be delayed
by the configured synchronous request timeout. Abrupt termination may leave a
private temporary directory. The vendor float conversion warnings are resolved;
deprecated calls in unrelated RViz panels remain. After restarting the SDK fixes,
a live check verified the new quality fields across 1,458 detections. During the
subsequent wrapper removal, the live driver and RViz remain on that existing
build; the validated replacement is in `.colcon/umrr96-no-wrapper/install`.

## References

- [ROS clock and time design](https://design.ros2.org/articles/clock_and_time.html)
- [Managed node lifecycle](https://design.ros2.org/articles/node_lifecycle.html)
- [diagnostic_updater](https://index.ros.org/p/diagnostic_updater/)
- [ParameterDescriptor](https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/msg/ParameterDescriptor.msg)
- [image_transport / image_common](https://github.com/ros-perception/image_common)
- [ROS coordinate and unit conventions, REP 103](https://www.ros.org/reps/rep-0103.html)
