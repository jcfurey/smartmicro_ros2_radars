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

- Reads the four supported tuning settings and firmware version from the radar.
- Shows target count, received update rate, and sensor cycle time from
  `/smart_radar/port_targetheader_0`.
- Stages range, range switching, antenna index, and CAN target-output changes.
- **Apply changes** sends only changed fields, then verifies them through readback.
- **Short-range Ethernet preset** stages sweep 2, range switching off, antenna 0,
  and CAN target output off. Click Apply to send the preset.
- **Starting values** stages the first settings read during this panel session.
  Click Apply to restore them. Closing the panel does not restore settings.

Opening the panel or loading an RViz configuration never writes sensor settings.
Only the sensor ID is saved in the RViz file; parameter values are read afresh.
There is no EEPROM-save or reset action. Service calls are asynchronous and have
bounded timeouts, so missing replies do not block the interface. This panel uses
the UMRR-96 control process launched by `umrr96_live.launch.py`.

### Smart Recorder
- Captures target/object data and exports CSV.

### Smart Command Configurator
- Sends commands and mode/config related service calls.

### Smart Firmware Download
- Sends firmware download request to selected sensor.

### Smart Status
- Displays live target/object header status topics.

### Smart Fault Reports
- Displays live fault report topics per sensor.

## Troubleshooting

1. If plugin is not visible in RViz, verify package is built and sourced.
2. If service-based panels do not respond, verify corresponding service servers are running.
