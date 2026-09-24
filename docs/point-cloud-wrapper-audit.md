# Point cloud wrapper dependency audit

Investigated on 2026-09-24 against driver commit `65d9f3b`, on the Lyrical
x86_64 host. The live driver, readback, views and RViz remained running. The
initial audit changed no production code, installed binaries or sensor settings.
The subsequent removal is described below.

## Recommendation

The external `point_cloud_msg_wrapper` dependency can be removed.
Keep two small, explicit cloud builders for the existing target and object
layouts, using `sensor_msgs::msg::PointCloud2`, `PointField` and the standard
ROS modifier for allocation. Preserve every published field and its offset.
This is primarily a dependency and maintainability improvement; the measured
wrapper cost is small at this radar's output rate.

The wrapper has a useful purpose: it derives message fields from C++ members,
offers typed append/access operations and checks field compatibility. Our driver
uses only a small part of that API, and already has two fixed, known schemas.
Removing the dependency means owning those schemas and their regression tests.

## Usage before removal

- The pinned dependency is version 1.0.7 at `0a62affd736fc39744467ff3a68d7e1f6c3d1bb5`
  (commit dated 2022-01-20). This describes our checkout, not current upstream
  maintenance status.
- Only `smartmicro_radar_node.cpp` includes or directly uses the wrapper. The
  driver has 52 target-cloud builders and 14 object-cloud builders. Every one
  constructs an empty cloud and calls `push_back()`; UMRR-96 Ethernet also calls
  `reserve()`. No wrapper views, read iterators or existing-cloud validation are used.
- The dependency is header-only. The built driver does not link a wrapper shared
  library. It introduces no ROS nodes, topics, SDK decoder, PCL dependency or
  middleware transport stage. Its append operation still copies a point into
  the ROS message buffer.
- The driver carries 26 custom field-generator macros, two generator tuples and
  point equality operators to satisfy the wrapper's API.
- Repository declarations are in `umrr_ros2_driver/package.xml`, the driver's
  `CMakeLists.txt`, `.gitmodules` and the `third_party` submodule. Build/readme
  instructions also describe it. No other workspace package directly declares
  a dependency on this wrapper in its package manifest or CMakeLists.

## Preserve the published binary layout

The wrapper derives field offsets and stride from C++ struct layout. Those
values have become part of the published message contract on this host.

| Layout | Current fields | Current point stride | Naive packed ROS setter stride |
| --- | ---: | ---: | ---: |
| `RadarPoint` | 18 | 72 bytes | 70 bytes |
| `ObjectPoint` | 14 | 48 bytes | 45 bytes |

For targets, 16 float32 values occupy offsets 0–63, `flags` is uint32 at 64,
and `peak_idx` is uint16 at 68. Bytes 70–71 are padding.

For objects, nine float32 values occupy offsets 0–35. `object_id` is int16 at
36, `idle_cycles` uint16 at 38, `spline_idx` uint16 at 40, `object_class` uint8
at 42, and `status` uint16 at 44. Byte 43 and bytes 46–47 are padding. Packing
the fields naively moves `status` to offset 43 as well as changing the stride.

The installed Lyrical `sensor_msgs/impl/point_cloud2_iterator.hpp` confirms that
`setPointCloud2Fields()` packs fields sequentially. The
[official ROS 2 implementation listing](https://docs.ros.org/en/ros2_packages/kilted/api/sensor_msgs/generated/program_listing_file_include_sensor_msgs_impl_point_cloud2_iterator.hpp.html)
also shows this behavior. `setPointCloud2FieldsByString("xyz")` adds its own
padding and does not describe these custom radar schemas.

A replacement should set explicit fields/offsets and strides, allocate the data
buffer once per message, and write each field using fixed-size copies or correctly
typed ROS iterators. Keep height 1, the existing frame/stamp, correct row size and
endianness, and `is_dense=false` because unavailable attributes use NaNs.
Zero padding explicitly instead of copying unspecified C++ padding. Assertions
and tests should pin field sizes, integer signedness and offsets. Avoid treating
the byte vector as an array of constructed C++ point objects.

## Coverage check limitation found

The wrapper's new-cloud constructor tries to detect omitted field generators by
constructing a byte mask, interpreting it as a point and comparing it with a
default point. This relies on the point's equality operator checking every member.

Our `ObjectPoint::operator==` checks its nine floating-point members but ignores
`object_id`, `idle_cycles`, `spline_idx`, `object_class` and `status`. A standalone
probe removed all five corresponding generators: construction still succeeded.
All five generators are present in the production driver, so this demonstrates
a gap in future-change detection, not missing fields in current publications.

There are also typed reinterpretations of byte storage in the wrapper's append
and validation paths. The proposed writer avoids them with field-sized copies.
No sanitizer failure was observed in this audit; a clean sanitizer run does not
establish portability of all object-lifetime/alignment assumptions.

## Standalone checks

The probe extracted the actual point structs and generator definitions from the
driver, then compared them with an independently defined schema and a writer
using only standard ROS message types plus `sensor_msgs::PointCloud2Modifier`.

- Target and object clouds with 0, 1, 30 and 1,024 points all matched metadata
  and every declared field byte. The fixture included negative zero, a NaN payload
  and integer sentinels. Unspecified struct padding was excluded from comparisons;
  the proposed writer's padding is zero-initialized.
- Both normal and ASan/UBSan builds passed. No radar packets or ROS sockets are
  used by the probe, and it does not rebuild or modify the live installation.
- The standard packed setter produced the 70/45-byte strides and object status
  offset 43 described above.
- An optimized local loop measured about **1.22 microseconds per fresh 30-point
  target cloud**, including wrapper construction, reserve, append and destruction
  over 20,000 repetitions. This synthetic measurement excludes SDK decoding,
  publication and visualization. It does not establish a replacement speedup.

Local artifacts are in `/tmp/umrr96-wrapper-audit/`: `current_point_types.hpp`,
`compare.cpp`, `results.json` and `sanitized-results.json`. The prototype is an
investigation artifact; it is not yet integrated into the driver.

## Removal scope and acceptance checks

1. Add the two explicit schemas/builders and pin all metadata and declared field
   bytes in tests, including empty clouds, NaNs, integer limits and padding.
2. Replace the two modifier aliases and adapt the 66 callback sites as needed;
   keep all model/CAN data conversions and sentinels intact. A small append API
   can keep callback changes mechanical, while reserving known list sizes.
3. Remove the wrapper includes/macros/equality workaround and dependency
   declarations, then remove the submodule and update build instructions.
4. Build in a clean isolated prefix with the wrapper absent to prove there is no
   accidental dependency on an old installed package. Run existing driver replay,
   processing/image and RViz checks against it; compare recorded sensor packets.
5. Restart the live session only when ready to deploy the validated change.

## Implementation

[`point_cloud_builder.hpp`](../umrr_ros2_driver/include/umrr_ros2_driver/point_cloud_builder.hpp)
now owns the two fixed schemas and their value structs. `RadarCloudBuilder` and
`ObjectCloudBuilder` use the standard ROS modifier for allocation and encode each
scalar explicitly in little-endian order. The message bytes do not depend on C++
struct padding or alignment. Padding is zero-filled, NaN payloads are preserved,
and oversized rows are rejected before allocation or uint32 metadata overflow.

All 66 target/object callbacks use these builders and reserve the decoded list
size. They bind the SDK list by const reference, avoiding a copied vector of
shared pointers. The old macros, generator tuples and equality workaround are
gone, as are the package/CMake dependencies and the Git submodule. Target
conversions, timestamps, topics, QoS and message schemas remain unchanged.

Six dedicated ASan/UBSan tests pin both schemas and their serialized bytes using
independent little-endian fixtures. They cover empty clouds, 1/30/1,024 records,
all float fields, signed object IDs, integer limits, NaNs/infinities/subnormals,
padding and reserve behavior. Neither production nor test code includes the old
wrapper.

Validation after removal: a clean build of all three smartmicro packages passed
in `.colcon/umrr96-no-wrapper`, with only that install and `/opt/ros/lyrical`
available as ROS prefixes. The wrapper package is absent from the active ament
index; `CMAKE_DISABLE_FIND_PACKAGE_point_cloud_msg_wrapper=ON` also rejects any
accidental future required lookup. All **34 focused tests** passed, including
the six new serialization tests and the existing replay, processing and RViz tests.

Old and new driver executables independently replayed the same captured UDP data
on loopback. Loaded library paths were checked to ensure that the two processes
actually used different builds. Across 30 frames and 953 detections, all 18
published field bytes matched exactly, as did cloud/header metadata apart from
the intentionally different ROS receive stamps. Both processes shut down cleanly.
Local comparison artifacts are in `/tmp/umrr96-wrapper-removal/`.

The live session continues using its existing build so it can remain uninterrupted.
The validated replacement install is at
`.colcon/umrr96-no-wrapper/install` under the workspace root. Source
`/opt/ros/lyrical/setup.bash` followed by that install's `local_setup.bash` in a
fresh shell for the next launch, or rebuild the regular workspace after stopping
the live session. Do not launch a second receiver on the sensor's live UDP port.

Historical sources are available at
[`0a62aff` wrapper implementation](https://gitlab.com/ApexAI/point_cloud_msg_wrapper/-/blob/0a62affd736fc39744467ff3a68d7e1f6c3d1bb5/point_cloud_msg_wrapper/include/point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp)
and its
[design notes](https://gitlab.com/ApexAI/point_cloud_msg_wrapper/-/blob/0a62affd736fc39744467ff3a68d7e1f6c3d1bb5/point_cloud_msg_wrapper/design/point_cloud_msg_wrapper-design.md).
