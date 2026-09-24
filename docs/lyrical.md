# Building on ROS 2 Lyrical

The driver, messages, point-cloud wrapper and all five RViz panels build on the
tested host: Ubuntu 26.04.1 x86_64, ROS 2 Lyrical, GCC 15.2, CMake 4.2.3 and
Qt 6.10.2. Smart Access Automotive 3.13.0 uses its supplied
`lib-linux-x86_64-gcc_9` libraries; the directory can be overridden with
`-DSMARTMICRO_LIB_DIR=...` for another SDK platform.

## Setup and build

Run from the ROS workspace root. The SDK setup only needs to run once; it is
already installed on this host. The point-cloud wrapper is a pinned upstream
submodule because it was absent from this host's Lyrical package index.

```bash
source /opt/ros/lyrical/setup.bash
git -C src/smartmicro_ros2_radars submodule update --init --recursive
cd src/smartmicro_ros2_radars
./smart_extract.sh
cd ../..
rosdep install --from-paths src/smartmicro_ros2_radars --ignore-src --rosdistro lyrical -y
CMAKE_BUILD_PARALLEL_LEVEL=6 colcon build \
  --base-paths src/smartmicro_ros2_radars \
  --packages-up-to umrr_ros2_driver smart_rviz_plugin \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 pkg executables umrr_ros2_driver
```

The executable is `smartmicro_radar_node_exe`. The fork's former root
`COLCON_IGNORE` marker has been removed. The SDK and build products remain
ignored by Git. Its upstream download script handles SDK installation; no SDK
libraries are vendored into tracked source.

CMake now uses the system `nlohmann-json3-dev` package (3.11 or newer), avoiding
a network download during configuration. The RViz library selects the Qt major
version exported by RViz, uses imported CMake targets supported by Lyrical,
accepts the current `cv_bridge.hpp` header, and moves the service future into
its worker. Closing the firmware panel also stops its executor even if the
worker has not started spinning yet.

## Verification

Enable and run the headless panel regression separately from the upstream
network integration suite:

```bash
CMAKE_BUILD_PARALLEL_LEVEL=6 colcon build \
  --base-paths src/smartmicro_ros2_radars \
  --packages-select smart_rviz_plugin --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/local_setup.bash
ROS_DOMAIN_ID=174 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  colcon test --packages-select smart_rviz_plugin \
  --event-handlers console_direct+
colcon test-result --test-result-base build/smart_rviz_plugin --verbose
```

The test uses Qt's offscreen platform, loads every registered smartmicro panel,
and immediately destroys each instance. A timeout detects shutdown hangs. It
needs permission to open local ROS sockets, but no display server or radar.

On 2026-09-24, a separate UMRR-96 v1.2.2 loopback replay used the repository's
native simulator and `targetlist_port_v2_1_0.bin`. It received three
`PointCloud2` messages with 17 targets each, verified all 18 field names and the
72-byte point layout, discovered the status service, and stopped the driver
with exit code 0. Simulator paths and configuration were adapted in `/tmp`;
the driver's config files were restored afterward. This validates local SDK
loading and the Ethernet target-output path; it is not a physical sensor test.

The existing driver launch test expects the multi-sensor Docker network from
`docker-compose.yml`. It was not run as part of this host build. The upstream
wrapper's legacy test CMake also predates Lyrical, so the initial dependency
build above disables its tests. Firmware download and hardware parameter writes
were not exercised. GCC 15 warns about an out-of-bounds float-to-integer read
in the supplied SDK `Instruction.h`; that vendor header has not been patched.

## Sensor configuration

The shipped default launch file uses an example for a different radar and
host network interface. Before connecting the UMRR-96, prepare a YAML file
using [the sensor examples](../umrr_ros2_driver/param/example/radar.sensor.example.yaml)
and the [model catalogue](../umrr_ros2_driver/param/model_uif_catalogue.yaml).
For interface 1.2.2, select `umrr96_v1_2_2` for Ethernet or
`umrr96_can_v1_2_2` for CAN, with `uifname: umrr96_t153_automotive` and
version fields `1`, `2`, `2`. Confirm that the physical sensor's firmware
matches, then supply its actual IP/port or CAN adapter configuration.

With a YAML file whose node key is `/smart_radar`:

```bash
ros2 run umrr_ros2_driver smartmicro_radar_node_exe \
  --ros-args -r __node:=smart_radar --params-file /path/to/radar.yaml
```
