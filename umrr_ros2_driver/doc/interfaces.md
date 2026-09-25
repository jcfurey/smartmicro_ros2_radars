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
| `smart_radar/set_radar_mode`, `get_radar_mode`, `get_radar_status`, `send_command`, `set_ip_address`, `firmware_download` | services |

To run several radars or place one under a robot namespace, set the node
namespace instead of renaming topics: `ros2 run ... --ros-args -r __ns:=/front`
or the `namespace` launch argument. Everything then appears under
`/front/smart_radar/...`. The parameter files use `/**/smart_radar:` keys so they
apply in any namespace. Individual topics can still be remapped, for example
`-r smart_radar/port_targets_0:=points`.

`firmware_download` replies when the sensor reports a final status (minutes);
the update runs on a worker thread, so diagnostics and the other services keep
running meanwhile. Service requests are validated completely before anything is
sent: unknown sensor ids (including 0), non-numeric, negative, out-of-range or
non-finite values are rejected with an error string.

## Launch files

`umrr96_live.launch.py` (driver, readback, views, RViz):
`params_file` (default: the cam-ripper bench file, which hard-codes a host
interface and addresses; supply your own elsewhere), `namespace` (default
empty), `use_sim_time` (`false`), `rviz` (`true`; closing RViz ends the launch,
use `false` headless), `targets_topic` (views input, default
`smart_radar/port_targets_0`), `view`, `rviz_config`, `publish_description`,
`description_frame_id`.

`radar.launch.py` (driver only): `params_file` (default
`param/radar.params.template.yaml`), `namespace`, `node_name` (`smart_radar`),
`use_sim_time`.

## Components and processes

| Component | Executable |
|-----------|------------|
| `smartmicro::drivers::radar::SmartmicroRadarNode` | `smartmicro_radar_node_exe` |
| `smartmicro::drivers::radar::ReadbackNode` | `smartmicro_radar_readback_node` |

The Smart Access SDK is a process-wide singleton. Load at most one of these
components per process (container), and never the data node together with the
readback node.

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
| `rcs` | float32 | Radar cross-section as reported: port UIFs declare `_m_sq` (m²), CAN UIFs declare `_dB`. Not converted. |
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
