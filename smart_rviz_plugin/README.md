# smart_rviz_plugin

ROS2 RViz panel plugin suite for Smartmicro radar operations.

## What this package provides

This package provides RViz panels to support day-to-day radar workflows:
1. Smart Recorder (CSV target/object recording)
2. Smart Command Configurator (services: command/param/status operations)
3. Smart Firmware Download (firmware transfer workflow)
4. Smart Status (target/object header monitoring)
5. Smart Fault Reports (fault report monitoring)
6. UMRR-96 Configuration (readback, temporary tuning, live measurement rate)

## Build

From workspace root:

```bash
colcon build --packages-select smart_rviz_plugin
source install/setup.bash
```

## Load plugins in RViz2

1. Start RViz2.
2. Open Panels menu.
3. Add the desired Smart plugins from the smart_rviz_plugin library.

## Plugin summary

### UMRR-96 Configuration

Included in `ros2 launch umrr_ros2_driver umrr96_live.launch.py`, or add
`smart_rviz_plugin/UMRR-96 Configuration` from RViz's Panels menu.

- Reads the four basic tuning settings and firmware version from the radar.
- Shows target count, received update rate, and sensor cycle time from
  `/smart_radar/port_targetheader_0`.
- Stages range, range switching, antenna index, and CAN target-output changes.
- **Apply changes** sends only changed fields, then verifies them through readback.
- **Short-range Ethernet preset** stages sweep 2, range switching off, antenna 0,
  and CAN target output off. Click Apply to send the preset.
- **Starting values** stages the first settings read during this panel session.
  Click Apply to restore them. Closing the panel does not restore settings.
- **Advanced sensor controls…** opens PRF selection/index and the minimum/maximum
  velocity gates for long, medium and short sweeps. It reads before writing to
  detect concurrent changes, orders PRF switching, sends velocity bounds as
  float pairs and verifies the result. Rejected/timed-out sequences stop and
  show fresh readback, including partial changes. Its own **Starting values**
  button stages restoration of the first advanced profile read for that sensor.
- **Filtering and density history** selects Off, Quality only, Stable mapping, or Moving returns,
  with adjustable minimum SNR, radial speed, and density decay (0.1–30 s).
  **Apply view settings** changes the host view parameters atomically. Filter
  changes clear accumulated history; decay-only edits preserve existing hits
  and apply the new decay rate from that moment onward. It sends no sensor
  commands. Current mode and accepted/rejected counts are displayed separately
  from the staged values. The raw target topic stays available for comparison.

Density decay defaults to 2 s: each cell's hit weight falls to about 37% over
that period without new hits. It affects the accumulated density grid; the
fan image remains the latest scan. Older view nodes leave this control disabled
until updated, while filtering remains available.

Quality only applies SNR/validity gates without stationary-radar assumptions.
Stable mapping requires a stationary radar; Moving returns uses sensor-relative
radial speed. The [measurement CLI](../docs/umrr96-output-priorities.md) provides
repeatable PRF comparisons with interleaved baselines and automatic restoration.

Opening the panel or loading an RViz configuration never writes sensor settings.
Only the sensor ID is saved in the RViz file; parameter values are read afresh.
There is no EEPROM-save or reset action. Service calls are asynchronous and have
bounded timeouts, so missing replies do not block the interface. This panel uses
the UMRR-96 control process and `umrr96_views` launched by `umrr96_live.launch.py`.
See the [bringup notes](../docs/umrr96-bringup.md) for filter behavior and the
separate RGB **Radar fan image** display included in the saved RViz configurations.

### Smart Recorder
- Captures target/object data and exports CSV.
- The topic list refreshes every second, so topics that appear after RViz starts
  (for example with `umrr96_live.launch.py`) are offered. Only the selected
  topic is subscribed.
- **Recording limit** (default 1,000,000 rows, saved in the RViz config) bounds
  memory: recording stops at the limit and offers Save/Discard.

### Smart Command Configurator
- Sends commands and mode/config related service calls.
- Sensor IDs and integer values accept decimal or `0x` hexadecimal; values are
  checked against the selected type before sending, and parse errors are shown
  in the response area. Command values are sent as float32 unchanged.
- Requests are asynchronous with a 10 s deadline; a missing reply never blocks RViz.
- The per-sensor instruction tables come from the Smart Access SDK
  (`smart_extract.sh` unpacks it into `umrr_ros2_driver/smartmicro`). The build
  installs them to `share/smart_rviz_plugin/user_interfaces`; rebuild the
  plugin after extracting the SDK. `SMART_USER_INTERFACES_DIR` overrides the
  location. A missing table is reported in the panel with the paths searched.

### Smart Firmware Download
- Sends firmware download request to selected sensor.
- The request is asynchronous (6 min deadline); the panel stays responsive and
  can be closed while a download is pending.

### Smart Status
- Displays live target/object header status topics (refreshed every second,
  subscribed only when selected).

### Smart Fault Reports
- Displays live fault report topics per sensor. When the selected topic
  disappears the panel unsubscribes and clears its tables.

### Custom CAN sender
`custom_can_sender.py` is a stand-alone Tk tool (needs `python3-can` and
`python3-tk`) that sends one CAN frame, or one per second with **Loop**, on a
SocketCAN interface. IDs above `0x7FF` are sent as 29-bit extended frames.

## Troubleshooting

1. If plugin is not visible in RViz, verify package is built and sourced.
2. If service-based panels do not respond, verify corresponding service servers are running.
