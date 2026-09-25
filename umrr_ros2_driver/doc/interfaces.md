# umrr_ros2_driver interfaces

Reference for the data node (`smartmicro_radar_node_exe`, component
`smartmicro::drivers::radar::SmartmicroRadarNode`) and the UMRR-96 readback
node (`smartmicro_radar_readback_node`, component
`smartmicro::drivers::radar::ReadbackNode`). Units come from the vendor
user-interface (UIF) serialization files under
`smartmicro/user_interfaces/*/serialization`; where the vendor does not declare
a unit, this page says so rather than guessing.

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
