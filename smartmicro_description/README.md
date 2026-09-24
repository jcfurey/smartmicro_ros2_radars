# smartmicro sensor descriptions

The UMRR-96 Type 153 model provides a reusable Xacro macro, a standalone URDF,
a ROS 2 `robot_state_publisher` launch and a close-up RViz configuration. It
builds independently of the Smart Access SDK and radar driver.

For the current standalone sensor, the complete tree is:

```text
umrr96_link   housing reference, standalone root
└── umrr96    measurement frame, matching PointCloud2.header.frame_id
```

Both frames use +X forward out of the front face, +Y left and +Z up. This agrees
with the driver's Cartesian conversion of range/azimuth/elevation. There is no
camera optical-frame rotation. The body frame is at the centre of the nominal
housing envelope; its upright orientation follows the physical TOP label.

The default housing-to-measurement transform is identity **by convention**, not
a measured antenna phase centre. The local documentation does not locate the
measurement origin precisely. `measurement_xyz` and `measurement_rpy` expose
that calibration independently of a future robot mounting transform. Axes/signs
must also be verified with a physical target before fusion.

The standalone model creates no `base_link`, `odom` or `map`. It supplies sensor
frames, not the changing pose of a handheld sensor. World-fixed accumulation and
Doppler compensation still need measured mounting extrinsics and real odometry.

The primitive geometry follows the **dimension drawing on p. 8** of the
[local 2020-11-11 datasheet](../../../references/smartmicro_umrr96/pdfs/umrr96_type153_datasheet_2020-11-11.pdf).
That front view is 97 mm across and 76 mm high. The performance table labels its
97×76×17.7 mm values H/W/D; the model follows the actual dimension drawing's
orientation. The nominal housing visual is 17.7 mm deep. Its coarse collision
box uses the drawing's larger 22.45 mm depth and 103.8 mm width including the
connector protrusion, centred 3.4 mm toward +Y. In a front view the connector is
on the observer's right, which is +Y in this convention.

This is a bounding-envelope model, not manufacturer CAD: mounting tabs, holes,
ribs, fasteners and the connector's detailed profile are simplified. Front/top
colour indicators and the connector visual's 18 mm height are illustrative.
The collision envelope covers these visuals; include the cable, bracket and
measured clearances in a mounted robot's collision model. No inertial tensor or
centre of mass is invented from the datasheet's ≤153 g mass limit. This is not
yet a physics simulation model or radar simulator.

Build from the workspace root without rebuilding the live driver:

```bash
source /opt/ros/lyrical/setup.bash
colcon --log-base .colcon/umrr96-description/log build \
  --base-paths src/smartmicro_ros2_radars/smartmicro_description \
  --build-base .colcon/umrr96-description/build \
  --install-base .colcon/umrr96-description/install \
  --symlink-install --cmake-args -DBUILD_TESTING=ON
source .colcon/umrr96-description/install/local_setup.bash
ros2 launch smartmicro_description umrr96_description.launch.py
```

This starts only the description publisher; the existing radar can keep running.
Use the same ROS domain and middleware as the radar. The current live session
uses `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Add `rviz:=true` for a close-up view.
For the existing RViz window, add/enable RobotModel using the **transient-local**
description topic `/umrr96/robot_description`. Zoom in to see the 97 mm housing
within the much larger detection scene. The optional viewer also subscribes to
the raw detection cloud; zoom out to see returns.

If the publisher is already running, open just the viewer without starting a
second TF publisher (from the workspace root, with the same ROS environment):

```bash
rviz2 -d src/smartmicro_ros2_radars/smartmicro_description/rviz/umrr96_description.rviz
```

The publisher name is `/umrr96_state_publisher`; description and `/tf_static`
remain available to late subscribers without a joint-state publisher. With
`namespace:=sensors`, the description topic becomes
`/sensors/umrr96/robot_description`; TF frame names stay unchanged. Choose unique
`sensor_name` and `frame_id` for additional sensors. `frame_id` must match the
driver's cloud header. `use_sim_time:=true` is supported for bag replay.

Export a plain URDF for other tools:

```bash
xacro src/smartmicro_ros2_radars/smartmicro_description/urdf/umrr96.urdf.xacro \
  -o /tmp/umrr96.urdf
check_urdf /tmp/umrr96.urdf
```

Once mounted, include the macro in the actual robot description, using measured
values rather than adding a second state publisher for the same sensor:

```xml
<xacro:include filename="$(find smartmicro_description)/urdf/umrr96_macro.urdf.xacro"/>
<xacro:umrr96_sensor name="umrr96" frame_id="umrr96" parent="${radar_parent}"
    xyz="${radar_mount_xyz}" rpy="${radar_mount_rpy}"
    measurement_xyz="${radar_measurement_xyz}" measurement_rpy="${radar_measurement_rpy}"/>
```

The supplied parent link must already exist. XYZ is metres; RPY is radians in
URDF fixed-axis roll/pitch/yaw order. `xyz/rpy` locates the housing in the parent;
`measurement_xyz/rpy` locates the data frame in the housing. The intended mounted
chain is `odom -> base_link -> ... -> umrr96_link -> umrr96`, with odometry owning
the changing transform and the robot state publisher owning the fixed joints.
Stop the standalone publisher when the full robot model owns these joints.

Tests expand and parse the real Xacro with `check_urdf`, verify the mechanical
envelope, compose two radars, reject malformed launch transforms, and launch the
installed publisher in an isolated ROS domain. The runtime test checks a
nonzero measurement offset and 90° yaw, late description/TF subscriptions and
simulation time without a joint-state publisher. Run:

```bash
colcon --log-base .colcon/umrr96-description/log test \
  --build-base .colcon/umrr96-description/build \
  --install-base .colcon/umrr96-description/install --packages-select smartmicro_description \
  --event-handlers console_direct+
colcon test-result --test-result-base .colcon/umrr96-description/build --verbose
```

On 2026-09-24 the package built on Lyrical and all ten pytest cases passed
(eight description checks and two runtime cases). A standalone publisher was
then started alongside the existing physical radar. The
[live verification](../docs/umrr96-description-verification.json) records the
incoming `umrr96` cloud frame, successful timestamped lookups into `umrr96_link`,
and a single sensor description/TF publisher. The headless publisher remains
running; its launch PID and log are recorded in
`/tmp/umrr96-description-live/session.json`. The existing radar session was not
restarted. These checks validate the description and TF contract, not the
unmeasured physical mounting or radar measurement-origin calibration.
