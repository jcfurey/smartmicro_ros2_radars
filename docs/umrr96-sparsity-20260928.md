# UMRR-96 sparsity investigation, 2026-09-28

Sensor 230739, firmware 5.2.2. The operator reports steel walls approximately
8 m away at an oblique angle. The radar is fixed; people may move. Comparisons
therefore include scene variation and do not establish calibrated wall coverage.
The operator prefers the current-scan view and rejected a one-second point
history because the points lingered.

## Host filtering

A 20-second capture produced 167 matched scans at 8.33 Hz before the usual
CAN-output-off startup setting was reapplied. There were 3,489 raw detections,
3,489 quality detections and 3,474 static Doppler inliers. At 6–10 m, 588 of
589 raw detections reached the static display. The host filters were not the
cause of sparse raw returns in that capture.

The normal live view subsequently ran at 18.18 Hz with CAN target output off.
The baseline was about 19.6 raw detections per scan. Short sweep 2, antenna 0,
automatic PRF, range switching off and speed validation limits ±20 m/s were
read back. No EEPROM-save, reset or firmware-update commands were sent.

## PRF comparison

The measurement CLI now accepts `--range-band MIN MAX`. It reports raw
detections per scan and SNR within `[MIN, MAX)` metres, includes zero-hit scans
in the mean, and preserves the overall measurements. For this scene:

```bash
ros2 run umrr_ros2_driver umrr96_measure --prf-trials --scene moving \
  --seconds 20 --range-band 6 10 --output /tmp/prf-trials.json
```

The first run had 2,909 scans across eight windows (initial, automatic,
manual 0, automatic, manual 1, automatic, manual 2, automatic). A repeat with
ten-second windows had 1,454 scans. Each manual setting is compared below with
the mean of the automatic windows immediately before and after it.

| PRF index | First run: total detections | Repeat: total detections | First run: 6–10 m | Repeat: 6–10 m |
| --- | ---: | ---: | ---: | ---: |
| 0 | +0.5% | +4.9% | −0.9% | +3.4% |
| 1 | +2.4% | −0.9% | +0.4% | −11.7% |
| 2 | −1.8% | −0.5% | +12.1% | +3.4% |

The automatic 6–10 m baseline itself varied from 3.36 to 5.80 detections per
scan in the first run. No PRF setting showed a substantial, repeatable gain
that justified replacing automatic selection in this moving scene. Both
experiments restored the entire starting profile, verified by readback.

Workspace artifacts: `results/umrr96-sparsity-20260928/baseline.json`,
`prf-trials.json` and `prf-repeat.json`.

## Antenna and sweep comparison

The same range-band measurement was applied to antennas 1 and 2 and medium-range
sweep 1, with 15-second windows and a return to baseline between each trial.
There were 1,910 scans. Baseline remained antenna 0, short sweep 2 and automatic
PRF. The complete original profile was restored and verified afterward.

| Setting | Mean raw detections/scan | Within 5 m/scan | At 6–10 m/scan |
| --- | ---: | ---: | ---: |
| Baseline before antenna 1 | 20.27 | 6.47 | 5.32 |
| Antenna 1 | 20.06 | 7.57 | 5.01 |
| Baseline before antenna 2 | 19.13 | 6.05 | 5.10 |
| Antenna 2 | 27.11 | 8.13 | 3.11 |
| Baseline before sweep 1 | 19.92 | 5.95 | 5.14 |
| Medium-range sweep 1 | 20.95 | 3.06 | 3.75 |
| Final baseline | 20.54 | 6.75 | 5.11 |

Antenna 2 yielded about 39% more detections overall than its neighboring
baselines, with about 39% fewer in the 6–10 m band. It is a potential alternative
for overall scene density, but is not an improvement in both metrics. Antenna 1
did not materially change total density. Sweep 1 extended the observed maximum
range from about 19.2 m to 48.1 m while reducing nearby detections. These are
scene-dependent detection counts; they do not identify real walls or distinguish
additional reflectors from multipath.

The experiment script, original profile, per-window measurements and verified
restoration are retained in
`results/umrr96-sparsity-20260928/antenna_sweep_trials.py` and `antenna-sweep.json`.

## Multiple-antenna operation

The operator asked whether multiple antennas can be used at once. The available
documentation does not establish a supported simultaneous or automatic
alternating mode for this sensor and firmware.

The [Type 153 datasheet, pages 5–6](https://www.smartmicro.com/fileadmin/media/Downloads/Automotive_Radar/Sensor_Data_Sheets_76-81GHz/UMRR_Automotive_Type_153_Data_Sheet.pdf)
describes selectable straight and squint beams. Its wording about simultaneous
or independent selection appears in the context of choosing range and beam;
it is insufficient to establish concurrent transmission from multiple beams.
The bundled [UIF v1.2.2 parameter definition](../umrr_ros2_driver/smartmicro/user_interfaces/UserInterfaceUmrr96_t153_automotiveV1.2.2/instructions/params/auto_interface_0dim.param)
defines `tx_antenna_idx` as one index in `0`–`2`, default `0`. It provides no
index-to-beam mapping, documented combined value, antenna bitmask or automatic
antenna switching control. `range_toggle_mode` alternates range modes;
automatic PRF changes pulse repetition frequency. Neither documents antenna
switching. The labels “antenna 0/1/2” above therefore refer to selector values,
not a verified mapping to individual physical transmit antennas.

Sequential selection worked in the measured trials, with two seconds allowed
to settle. That does not validate switching every scan. Parameter readback
alone cannot identify which acquisition produced an in-flight target list:
the UMRR-96 generic ROS antenna field is not populated, and the vendor describes
`AcquisitionSetup` without documenting its encoding. Firmware-controlled beam
interleaving would be a candidate for combining coverage if supported, but its
transition behavior and per-beam update rate need confirmation.

Questions for smartmicro are the meaning of indices `0`–`2` on firmware 5.2.2,
whether simultaneous or interleaved beams are supported, and the documented
switch timing and acquisition identification. No combined-mode or rapid
switching commands were tried. The restored baseline remains selector `0`,
with the current-scan display and no added persistence.

## Rejected display experiment

A one-second history in 0.25 m voxels, requiring support from two scans,
increased displayed static points from 19.6 to 50.4 on average. At 6–10 m it
increased the displayed count from 3.73 to 12.74. All 7,453 representatives
whose source scans were captured matched their original XYZ exactly; maximum
age was 0.943 s and minimum support was two scans. Another 106 representatives
referenced scans before capture began.

These are retained observations, including noisy voxel-boundary crossings,
not additional instantaneous sensor detections or proof of wall shape. The
operator disliked the visible persistence. The history integration was removed,
its node stopped, and RViz returned to the current Doppler-inlier scan with
zero static-cloud decay. No history-related repository changes remain.

## Software verification

The measurement tests cover range-band endpoints, empty scans, nonfinite ranges,
invalid bounds, reset behavior, CLI forwarding and restoration after partial
writes/interruption. All 21 measurement tests pass. The updated Python files
pass flake8 and pep257; the driver build passes.
