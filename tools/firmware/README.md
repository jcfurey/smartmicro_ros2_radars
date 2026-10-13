# UMRR-96 bench bootloader tools

These are the helpers used to install the original vendor 5.2.4 image on
UMRR-96 Type 153 serial **230739**, through **VN1630A CH1 / can0**, CAN network
ID 0 at 500 kbit/s. They are intentionally specific to that unit and the
5.2.2 → 5.2.4 procedure. The installed radar now runs 5.2.4; an ordinary update
attempt refuses a current application version other than 5.2.2.

The original SDK handles the download protocol, encryption and flash operation.
No vendor library is modified. The runner requires the original XML SHA-256
`a1b98c9149868d3b3dbb853bdb6b4d2d222bb0390562f3e29d713ca6f6cc7f61`.
Firmware images, packet captures, recovered firmware and exported analysis stay
under the repository's Git-ignored `results/` directory. The Ghidra installation
and saved project are stored outside the workspace; their current locations
are documented in [the analysis notes](../../docs/umrr96-firmware-update-20261013.md#ghidra-workspace-cleanup).

Run from this repository root, after extracting the Smart Access SDK:

```bash
python3 tools/firmware/build.py --work-dir results/umrr96-can-update-20261013
```

This only compiles the helpers and prepares their configurations. It sends no
radar traffic. `--sdk-dir` and `--library-dir` override SDK/build locations.
The original vendor XML must be supplied separately in the working directory
as `UMRR-96_Type_153_Firmware_v5.2.4.xml`.

For read-only CAN identity/configuration capture, create an output directory and run:

```bash
mkdir -p results/umrr96-can-update-20261013/readcheck-new
SMART_ACCESS_CFG_FILE_PATH="$PWD/results/umrr96-can-update-20261013/config/smart_access_config.json" \
  results/umrr96-can-update-20261013/can_maintenance \
  results/umrr96-can-update-20261013/read-definitions.json \
  results/umrr96-can-update-20261013/readcheck-new snapshot
```

An authorized update uses:

```bash
python3 tools/firmware/run_can_update.py --work-dir results/umrr96-can-update-20261013
```

Before arming, `can0` must be up at 500 kbit/s in normal mode, with
`sudo ip link set dev can0 txqueuelen 4096`. Configure bitrate while the host
interface is down. The updater first snapshots identity/settings, then waits for
the bootloader. Only after its `ARMED` message should the operator cycle the
application-mode radar once. Once transfer starts, leave power connected. The
vendor state machine supplies protocol timeouts; the helper adds no outer abort.

The recorded first-block timeout can be retried while the same radar remains
powered and emits idle bootloader echoes:

```bash
python3 tools/firmware/run_can_update.py \
  --work-dir results/umrr96-can-update-20261013 \
  --retry-bootloader results/umrr96-can-update-20261013/attempt-20261013T172756Z
```

That mode validates the previous manifest, original image digest, serial and
first-block timeout, then requires three fresh matching idle echoes. The earlier
snapshot is a reference, not a new application readback. This is the recovery
path used on 2026-10-13, not a general recovery recipe for other failures.

Every attempt records raw CAN traffic, SDK progress/terminal status and interface
statistics. SDK byte progress is per segment. Success also requires application
readback of 5.2.4. `port_readback` separately checks all 29 parameters and 14
statuses through unmodified Ethernet port-format instructions, using the
`port-readback-config` prepared by the build script.

The successful evidence is in
`results/umrr96-can-update-20261013/attempt-20261013T173635Z/verification.json`.
The compact, versioned [verification result](results-2026-10-13.json) records the
image digest, all segment lengths, unchanged settings and post-update checks.
See [the update and analysis](../../docs/umrr96-firmware-update-20261013.md).

The subsequent detection-tuning experiments are recorded separately in
[the control map](detection-controls-2026-10-13.json),
[scalar results](density-results-2026-10-13.json),
[floor results](floor-results-2026-10-13.json) and
[noise-estimator results](noise-estimator-results-2026-10-13.json).
All tested values were restored; antenna changes are excluded by the operator.
The private serializer sources, raw CAN captures and exact instruction
transcripts remain inside the repository's ignored `results/` directory.
