# UMRR-96 offline assessment

## Radar Ghost Dataset

The [2026-09-28 dataset study](../../docs/umrr96-ghost-dataset-study-20260928.md)
evaluates the existing single-scan point ghost rules on a six-sequence sample of
Radar Ghost Dataset v1.1. Data is on Bulk-Storage, outside Git. This is a 2D,
fixed-radar stage adapter; it does not run our 3D ego fit, SNR gate or tracker.
The script reads current range, residual and ghost settings from the launch
YAML and records their values and source hashes. Labels only affect scoring.

Dependencies are Python, NumPy, h5py and PyYAML; Matplotlib is needed for the
plot. The existing research environment is
`/tmp/umrr96-ghost-research-venv/bin/python`. To recreate it, use a virtual
environment and install `numpy h5py pyyaml matplotlib`. From the repository:

```bash
python3 tools/assessment/assess_ghost_dataset.py \
  --manifest /run/media/jcfurey/Bulk-Storage/RadarGhostDataset/v1.1/sample-manifest.json \
  --output /tmp/ghost-dataset-repeat.json
python3 -m unittest discover -s tools/assessment -p 'test_assess_ghost_dataset.py' -v
MPLCONFIGDIR=/tmp/umrr96-matplotlib python3 tools/assessment/plot_ghost_dataset.py \
  /tmp/ghost-dataset-repeat.json /tmp/ghost-dataset-repeat.png
```

Choose fresh output paths. Each H5 must match the manifest SHA256 before being
evaluated; source member names and URLs remain in the output. Original and
virtual archives are aggregated separately. Reported retention/rejection applies
only to eligible moving detections with confident labels. Unspecified-order
ghosts, stationary labels and exclusions have separate counts. The full archive
MD5 verification record is `archives-verified.json` beside the manifest; partial
downloads are not verified archives. The report is not a sensor accuracy claim.

To reproduce the sensitivity comparison between the current YAML motion gate
and the Python fallback, with all other gates unchanged:

```bash
python3 tools/assessment/assess_ghost_thresholds.py \
  --manifest /run/media/jcfurey/Bulk-Storage/RadarGhostDataset/v1.1/sample-manifest.json \
  --output /tmp/ghost-threshold-repeat.json
```

It reports both moving-only scores and scores over a fixed set of all valid
labeled detections, so changing the motion gate does not silently change the
denominator in that second comparison. Neither threshold is optimized here.

## Recorded UMRR-96 bags

These tools inspect recorded detections and reproduce the
[2026-09-24 investigation](../../docs/umrr96-navigation-investigation.md).
They do not publish poses, call sensor services, tune hardware, or establish
navigation accuracy. The numerical checks are independent of ROS; bag reading
requires ROS 2, `rosbag2_py`, `sensor_msgs_py`, NumPy, and the driver's messages.

From the workspace root, reproduce the audit using a **new** output path:

```bash
source /opt/ros/lyrical/setup.bash
source .colcon/umrr96-no-wrapper/install/local_setup.bash
python3 src/smartmicro_ros2_radars/tools/assessment/assess_bag.py \
  results/umrr96-investigation-20260924/live \
  --output /tmp/umrr96-assessment-repeat.json
python3 -m unittest discover \
  -s src/smartmicro_ros2_radars/tools/assessment -p 'test_*.py' -v
```

The script expects sensor 0 under `/smart_radar`, with the current message
definitions. Source another compatible build if the isolated build is absent.
For this short research capture it loads scans into memory. It reports raw field
statistics, byte preservation, timing, metadata agreement, replay of existing
host filter modes, and direction-matrix conditioning. It refuses duplicate
frame/stamp keys instead of silently conflating scans. Field distributions
explicitly count unavailable values. SHA-256 hashes identify bag source files.

The exploratory planar RANSAC fit uses a 0.30 m/s residual threshold, 64 trials,
seed 0, at least six consensus points, and direction-matrix condition ≤100.
It assumes zero radar vertical velocity, a dominant stationary scene, and
positive-receding Doppler. Those assumptions are **unverified on the live unit**.
The report exposes these limits; a small residual is not a velocity error metric.
No SDK variance, raw Pfa, flags, or RCS value is treated as calibrated confidence.
Filter replay uses the current source defaults: 6 dB and 0.25 m/s. Its initial
temporal history is empty, independently of the running views node.

The original approximately 4.7 MB MCAP and the failed full-profile read are in
`results/umrr96-investigation-20260924/`, outside Git under the workspace's existing
ignore rule. The two compact JSON reports in `docs/` preserve derived evidence
and live graph/readback context. Back up the MCAP separately if preserving the
underlying experiment beyond this workspace is required.

To collect another observational sample, use a **new** bag directory and the
ROS domain/middleware of the running driver. These were the live settings:

```bash
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOG_DIR=/tmp/umrr96-assessment-roslog
timeout --signal=INT --kill-after=10s 45s ros2 bag record \
  --disable-keyboard-controls --node-name umrr96_investigation_recorder \
  -o /tmp/umrr96-new-observation -s mcap --topics \
  /smart_radar/port_targets_0 /smart_radar/filtered_targets_0 \
  /smart_radar/port_targetheader_0 /smart_radar/timing_0 \
  /smart_radar/umrr96_raw_quality_0 /smart_radar/filter_status \
  /diagnostics /tf /tf_static
```

`timeout` returns 124 for the scheduled stop; verify rosbag metadata and clean
recorder shutdown. Add actual IMU, lidar/camera, pose-reference and odometry
topics for a motion experiment. Naming `/tf` does not create a TF publisher.
Optional read-only profile measurement uses the existing `umrr96_measure.py`
without `--prf-trials`. A failed control read can abort that tool before its
measurement window, so keep passive recording independent of it.

Reproduce the figure:

```bash
MPLCONFIGDIR=/tmp/umrr96-matplotlib python3 \
  src/smartmicro_ros2_radars/tools/assessment/plot_assessment.py \
  --assessment src/smartmicro_ros2_radars/docs/umrr96-assessment-20260924.json \
  --tuning src/smartmicro_ros2_radars/docs/umrr96_38553_tuning.json \
  --output /tmp/umrr96-assessment.png
```
