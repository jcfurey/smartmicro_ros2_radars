# umrr_ros2_driver interfaces

Reference for the data node (`smartmicro_radar_node_exe`, component
`smartmicro::drivers::radar::SmartmicroRadarNode`) and the UMRR-96 readback
node (`smartmicro_radar_readback_node`, component
`smartmicro::drivers::radar::ReadbackNode`). Units come from the vendor
user-interface (UIF) serialization files under
`smartmicro/user_interfaces/*/serialization`; where the vendor does not declare
a unit, this page says so rather than guessing.

## Topics, services and namespaces

Topic and service names are relative and keep their historical form, with a
`smart_radar/` prefix and the sensor index `N` (`sensors.sensor_N`):

| Name | Type |
|------|------|
| `smart_radar/port_targets_N`, `smart_radar/can_targets_N` | `sensor_msgs/PointCloud2` |
| `smart_radar/port_objects_N`, `smart_radar/can_objects_N` | `sensor_msgs/PointCloud2` (`pub_type: mse`) |
| `smart_radar/port_targetheader_N`, `smart_radar/can_targetheader_N` | `PortTargetHeader`, `CanTargetHeader` |
| `smart_radar/port_objectheader_N`, `smart_radar/can_objectheader_N` | `PortObjectHeader`, `CanObjectHeader` |
| `smart_radar/port_faultreport_N` | `PortFaultReportsMsg` (models with fault reports) |
| `smart_radar/timing_N` | `RadarTiming` |
| `smart_radar/umrr96_raw_quality_N` | `Umrr96RawQuality` (UMRR-96 Ethernet) |
| `smart_radar/set_radar_mode`, `get_radar_mode`, `get_radar_status`, `send_command`, `set_ip_address`, `firmware_download` | data node services; `umrr96_live.launch.py` moves the first three (see [service routing](#service-routing)) |

### Optional `radar_msgs/RadarScan`

When the package is built with [radar_msgs](https://index.ros.org/p/radar_msgs/)
available (`apt install ros-lyrical-radar-msgs`; CMake option
`SMARTMICRO_WITH_RADAR_MSGS`, default ON, detects it) and the startup parameter
`publish_radar_scan: true` is set, every target cloud is also published as
`radar_msgs/RadarScan` on `smart_radar/radar_scan_N`: same header and detection
order, `range` [m], `azimuth`/`elevation` [rad], `doppler_velocity` = the SDK
radial speed [m/s] without sign conversion (see `radial_speed` below) and
`amplitude` = power [dB]. A cycle without targets gives an empty scan (before
2026-10-09 such a cycle was undefined behaviour and aborted builds with
`_GLIBCXX_ASSERTIONS`). The scan is only built while it has subscribers.
Setting the parameter on a build without radar_msgs fails at startup. radar_msgs
is not a declared package dependency, so rosdep does not install it.

To run several radars or place one under a robot namespace, set the node
namespace instead of renaming topics: `ros2 run ... --ros-args -r __ns:=/front`
or the `namespace` launch argument. Everything then appears under
`/front/smart_radar/...`. The parameter files use `/**/smart_radar:` keys so they
apply in any namespace. Individual topics can still be remapped, for example
`-r smart_radar/port_targets_0:=points`.

This and the next paragraphs describe the data node's services.
`firmware_download` replies when the sensor reports a final status (minutes);
the update runs on a worker thread, so diagnostics and the other services keep
running meanwhile. Service requests are validated completely before anything is
sent: unknown sensor ids (including 0), non-numeric, negative, out-of-range or
non-finite values, empty name lists and names or types the sensor's interface
does not define are rejected with an error string starting `Error:` (or, for the
older wording, `Failed to ...`).

`set_radar_mode`, `get_radar_mode`, `get_radar_status`, `send_command` and
`set_ip_address` reply when the sensor answers, without blocking the executor
(deferred responses). The reply text (`res`, `res_ip`) is the same JSON the
readback node returns:

```json
{"sensor_id": 100, "section": "auto_interface_0dim", "success": true,
 "values": {"frequency_sweep_idx": {"response_type": 1, "value": 2}}}
```

`success` is true only if the sensor accepted every instruction; a rejected
instruction has an `error` naming the SDK response type (for example "value above
maximum") instead of a `value`. Without a reply within the startup parameter
`instruction_timeout_ms` (default 3000, 100..60000) the reply is
`success: false` with a "Timed out ..." error; a timed-out write may still have
been applied. A successful `set_ip_address` adds a `note` to restart the radar.
Before 2026-09-30 these services replied at once with "Success: Request sent"
and only logged the sensor's answer. Every SDK instruction batch is released
after its reply or timeout (previously none were).

### Service routing

Which node owns the root service names depends on the launch file:

| Launch | `smart_radar/{set_radar_mode,get_radar_mode,get_radar_status}` | `smart_radar/data_receiver/{...}` | `smart_radar/{send_command,set_ip_address,firmware_download}` |
|--------|------------------|------------------|------------------|
| `radar.launch.py`, or the data executable alone | data node | — | data node |
| `umrr96_live.launch.py`, and `smartmicro_processing`'s `umrr96_tracking.launch.py` (includes it) | readback node | data node (remapped) | data node |

The readback node (`smart_radar_readback`) differs from the data node behind the
same names:

- It handles one UMRR-96 Type 153 (interface 1.2.2): requests whose `sensor_id`
  is not its startup parameter `sensor_id` are rejected ("Sensor ID does not
  match this readback node"), so the data node's IDs (for example 100 in
  `radar.params.template.yaml`) do not work there.
- It is synchronous: the service callback waits for the sensor reply for up to
  `timeout_ms` (default 2000 ms), and calls are handled one at a time.
- `get_radar_mode` reads section `auto_interface_0dim` and `get_radar_status`
  section `auto_interface` only, 1–10 unique names per request.
- `set_radar_mode` writes section `auto_interface_0dim` only, and only the
  UMRR-96 tuning whitelist: `uint8` (`value_types: 3`) `frequency_sweep_idx`
  0–2, `range_toggle_mode` 0–3, `tx_antenna_idx` 0–2,
  `output_control_target_list_can` 0–1, `prf_selector_manual` 0–1,
  `prf_set_selector` 0, `prf_manual_value_idx` 0–2, and `float32`
  (`value_types: 0`) `tv_min_speed_sweep_idx_{0,1,2}` /
  `tv_max_speed_sweep_idx_{0,1,2}` within ±150, always as a min/max pair with
  min ≤ max. The whole request is validated before anything is sent.
- It has no `send_command`, `set_ip_address` or `firmware_download`; those stay
  with the data node.

Replies use the JSON above. With the bench parameter file:

```bash
ros2 service call /smart_radar/get_radar_mode umrr_ros2_msgs/srv/GetMode \
  '{sensor_id: 230739, section_name: auto_interface_0dim, params: [frequency_sweep_idx], param_types: [3]}'
```

On the UMRR-96 the data node's port-based instructions get no sensor reply: the
same read through `smart_radar/data_receiver/get_radar_mode` returns
`success: false` with "Timed out after 3000 ms" (live, 2026-10-09). The data
node's `send_command` and `set_ip_address` send instruction batches through the
same client, so treat a timeout there as "unknown", not as "not applied". See
[UMRR-96 bring-up](../../docs/umrr96-bringup.md#read-parameters-and-status)
for why the readback node exists.

### QoS

Data publishers are reliable, volatile, `KEEP_LAST` with depth
`sensors.sensor_N.history_size`: reliable publishers match both reliable and
best-effort (`SensorDataQoS`) subscribers. Reliability, history and depth of each
data topic can be overridden with the standard startup parameters
`qos_overrides.<fully qualified topic>.publisher.<policy>`:

```yaml
/**/smart_radar:
  ros__parameters:
    qos_overrides:
      /smart_radar/port_targets_0:     # include the namespace, e.g. /front/smart_radar/...
        publisher:
          reliability: best_effort
          depth: 3
```

### Startup validation

Besides link type, model and publish type, the node refuses to start when an
Ethernet sensor's `ip` or an adapter's `hw_ip_address` is not an IPv4 address,
two sensors share an `id`, a `frame_id` (two devices in one TF frame; since
2026-10-09, and the shipped files now use `umrr_0`, `umrr_1`, ...) or two adapters a
`hw_dev_id`, a sensor's `link_type`
differs from its adapter's `hw_type`, or a `frame_id` starts with `/` (tf2
rejects such frames). Each of these previously started a node that published
nothing for that sensor.

It also names the parameter for configurations that the SDK refused only with
"Communication Service initialization failed" (since 2026-10-09): two Ethernet
sensors with the same `ip` (the SDK identifies Ethernet sensors by address, so a
different `port` does not help), a sensor `inst_type` or `data_type` other than
`port_based` or `can_based` (empty included; the shipped CAN example now sets
`can_based`), a second `eth` adapter (SDK 3.13.0: "Only one ETH iface is
allowed"; several Ethernet sensors share one adapter) and an adapter `hw_type`
other than `eth` or `can`. `master_data_serial_type` and
`master_inst_serial_type` may stay empty, which the SDK accepts with its own
default (loopback reception was unaffected), but any other value than
`port_based` or `can_based` is rejected as a typo. Use `port_based` for Ethernet
sensors: on loopback an Ethernet sensor with `inst_type: can_based` started but
never received data (not rejected; unverified on hardware).

`uifname`, `uifmajorv`, `uifminorv` and `uifpatchv` must name the user interface
of the sensor's `model` (see `param/model_uif_catalogue.yaml`; the version is the
model suffix, e.g. `umrr96_v1_2_2` → `umrr96_t153_automotive` 1.2.2). A different
name or version is rejected with the parameter named; before 2026-10-09 the node
started but delivered no data for that sensor. Leaving all four unset (empty name, zero versions) takes them from the
model.

### Diagnostics

Both nodes publish `/diagnostics` at about 1 Hz through `diagnostic_updater`.
Status names are `<node name>: <status>`; a node with a namespace uses its fully
qualified name (`/front/smart_radar: Target stream 0`), so two radars never report
under the same name. This is the default of the parameter
`diagnostic_updater.use_fqn` whenever the node namespace is not `/`, however the
node is started (`ros2 run ... -r __ns:=/front`, a launch file or a component
container); setting the parameter explicitly overrides it. Before 2026-10-09 only
the launch files set it.

Each configured data stream has its own liveness status: `Target stream N` for
every sensor and `Object stream N` for sensors with `pub_type: mse` (object lists
are subscribed to only then). A stream is `STALE` before its first list ("Waiting
for ...") and after `diagnostics.stale_timeout` without one, `WARN` after a device
timestamp anomaly (repeated, backwards or zero) and `OK` otherwise. Before
2026-10-09 only target lists were tracked: an MSE sensor whose object stream stopped
still reported `OK`.

The device timestamp behind these checks and `RadarTiming.device_timestamp_us` is
the generic port header timestamp for Ethernet lists. For CAN lists the SDK fills
the port header with its own host receive time (`system_clock` µs, written when it
assembles the port; no CAN signal maps to it), so CAN target lists use the
acquisition time of their list header instead: `TimeStamp` [s] +
`AcqTimeStampFraction` [s] (also published raw in `CanTargetHeader.time_stamp` and
`acq_ts_fraction`). CAN object list headers carry no time: their
`device_timestamp_us` is 0, the object stream reports
`device_timestamp_available: false` and only its liveness is checked. Before
2026-10-09 CAN lists reported the SDK receive time as device time. Header stamps
remain ROS receive time throughout.

The readback node's `Control requests` status counts requests, timeouts and
rejections and reports the startup CAN target-output write as
`startup_can_target_output` (`pending`, `writing (attempt N)`, `verifying (attempt
N)`, `retrying: <error>`, `on (confirmed)`/`off (confirmed)` or `superseded by a
set_radar_mode request`) with `startup_can_target_attempts`. The startup write and
its readback run without blocking the executor, so control requests and the
diagnostics are served while the sensor is unreachable.

`UDP adapter N` reports the kernel's drop counter of the Ethernet adapter's socket
(`/proc/net/udp`). Only the process's own sockets are considered, matched by local
port and, when `hw_ip_address` is set, by local address; the status is `WARN`
("unavailable or ambiguous") only if more than one socket still matches. The SDK
(3.13.0) accepts one Ethernet adapter per process: a second one fails SDK
initialization ("Only one ETH iface is allowed").

## Launch files

`umrr96_live.launch.py` (driver, readback, views, RViz):
`params_file` (default: the cam-ripper bench file, which hard-codes a host
interface and addresses; supply your own elsewhere), `namespace` (default
empty), `use_sim_time` (`false`), `rviz` (`true`; closing RViz ends the launch,
use `false` headless), `targets_topic` (views input, default
`smart_radar/port_targets_0`), `view`, `rviz_config`, `publish_description`
(`false`; includes `smartmicro_description` in its own launch scope, without its
viewer), `description_frame_id`, `description_sensor_name`. The readback node
takes over the root mode/status service names; see
[service routing](#service-routing).

`radar.launch.py` (driver only): `params_file` (default
`param/radar.params.template.yaml`), `namespace`, `node_name` (`smart_radar`),
`use_sim_time`. All services are the data node's.

## Components and processes

| Component | Executable |
|-----------|------------|
| `smartmicro::drivers::radar::SmartmicroRadarNode` | `smartmicro_radar_node_exe` |
| `smartmicro::drivers::radar::ReadbackNode` | `smartmicro_radar_readback_node` |

The Smart Access SDK is a process-wide singleton. Load at most one of these
components per process (container), and never the data node together with the
readback node.

The SDK reads the path of its configuration only from the environment variable
`SMART_ACCESS_CFG_FILE_PATH` (its `Init()` takes no path), and glibc `setenv` is not
safe against a concurrent `getenv` in another thread. The standalone executables
create the private configuration directory and set the variable before
`rclcpp::init`, when only the main thread (and LTTng-UST listener threads started
by library constructors) exist. A component constructed in a container sets it in
its constructor while the container's middleware and executor threads already
run, as before 2026-10-09 in every case; prefer the executables where that risk
matters.

## Install layout

The vendor libraries (`libsmart_access.so`, `libcom_lib.so`, `libosal.so` and the
user-interface libraries) are installed in `lib/umrr_ros2_driver/`, not in the
shared `lib/`, and are found through the node libraries' RPATH
(`$ORIGIN/umrr_ros2_driver`). Anything that pointed at `<prefix>/lib/libsmart_access.so`
or used `<prefix>/lib` as the SDK `shared_lib_path` must use
`<prefix>/lib/umrr_ros2_driver`.

The SDK aborts the process ("buffer overflow detected") when `shared_lib_path`
has 160 or more characters. The nodes therefore give the SDK a short alias
(`<private runtime dir>/sdk-lib`) instead of the install path; keep `TMPDIR`
short.

`smart_access_config.json` and `config_path.hpp` are generated in the build tree
and installed (`share/umrr_ros2_driver/config/`,
`include/umrr_ros2_driver/umrr_ros2_driver/`); the build no longer writes into
the install prefix at configure time. Installed JSON files are templates only:
each process copies them into a private temporary directory. Only
`lib-linux-x86_64-gcc_9` SDK binaries exist; configuring on another processor
fails with a message unless `-DSMARTMICRO_LIB_DIR=` names vendor libraries for it.

## Point clouds

All clouds are `sensor_msgs/PointCloud2`, little-endian, `height = 1`,
`is_dense = false`, `frame_id` = `sensors.sensor_N.frame_id`, stamped with ROS
receive time (see `RadarTiming`). Values a model does not provide are NaN for
float fields and the maximum value for unsigned integer fields.

### Targets: `smart_radar/port_targets_N`, `smart_radar/can_targets_N` (72-byte stride)

| Field | Type | Unit / meaning |
|-------|------|----------------|
| `x`, `y`, `z` | float32 | m, sensor frame (x forward, y left, z up), computed from range and angles |
| `radial_speed` | float32 | m/s, the SDK `SpeedRadial` value unchanged. **Sign:** the vendor documentation does not state it. The fork's processing assumes positive = receding (`doppler_sign` in `smartmicro_processing`); verify with a target moving at a known velocity before relying on it. |
| `power` | float32 | dB (port `Power`; CAN `SignalLevel`) |
| `rcs` | float32 | Radar cross-section [m²]. Port UIFs declare `_m_sq`; CAN UIFs declare `_dB` (dBsm), which the driver converts with 10^(dBsm/10) (before 2026-09-30 CAN values were published in dBsm). |
| `noise` | float32 | dB |
| `snr` | float32 | dB, `power - noise` |
| `azimuth_angle`, `elevation_angle` | float32 | rad |
| `range` | float32 | m |
| `variance_range`, `variance_speed`, `variance_azimuth_angle`, `variance_elevation_angle` | float32 | Variances as reported; units are not declared by the UIF (presumably m², (m/s)², rad², rad²). NaN where unavailable. |
| `false_alarm_probability` | float32 | Probability as reported. NaN for UMRR-96 (raw values in `umrr96_raw_quality_N`). |
| `flags` | uint32 | Vendor bit field; UINT32_MAX where unavailable. |
| `peak_idx` | uint16 | Index into the vendor peak list; UINT16_MAX where unavailable. |

### Objects: `smart_radar/port_objects_N`, `smart_radar/can_objects_N` (48-byte stride)

| Field | Type | Unit / meaning |
|-------|------|----------------|
| `x`, `y`, `z` | float32 | m |
| `speed_absolute` | float32 | m/s |
| `heading` | float32 | rad, [-π, π]. CAN interfaces deliver degrees (`HeadingDeg`); the driver converts them to radians. |
| `length` | float32 | m |
| `mileage` | float32 | As reported (unit not declared); NaN on CAN |
| `quality` | float32 | Unitless quality indicator |
| `acceleration` | float32 | m/s² |
| `object_id` | int16 | Object id. The CAN SDK delivers `uint16`; ids ≥ 32768 appear negative. Reinterpret as `uint16` (`id & 0xFFFF`) if your sensor can produce such ids. The type is kept for schema compatibility. |
| `idle_cycles`, `spline_idx` | uint16 | Counters/indices; UINT16_MAX on CAN |
| `object_class` | uint8 | Vendor class code; UINT8_MAX on CAN |
| `status` | uint16 | Vendor status (e.g. 0 = object, 2 = guardrail); UINT16_MAX on CAN |

## Custom messages

`umrr_ros2_msgs` headers carry unit comments. Two fields were misspelled
upstream; both spellings are now published with identical values:

| Deprecated | Use instead |
|------------|-------------|
| `PortTargetHeader.umambiguous_speed` | `PortTargetHeader.unambiguous_speed` |
| `PortFaultReport.occurence_count` | `PortFaultReport.occurrence_count` |

`SetMode`/`GetMode` requests define `TYPE_FLOAT32=0`, `TYPE_UINT32=1`,
`TYPE_UINT16=2`, `TYPE_UINT8=3`; `GetStatus` defines `TYPE_UINT32=0`,
`TYPE_UINT16=1`, `TYPE_UINT8=2`, `TYPE_INT32=3` (the codes differ).
