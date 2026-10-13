# UMRR-96 firmware 5.2.4 update and analysis (2026-10-13)

Sensor: UMRR-96 Type 153, serial 230739 (client ID `0x00038553`) at
192.168.11.11, updated **5.2.2 → 5.2.4** through VN1630A CH1 (`can0`). The
original vendor SDK completed all nine segments after increasing the host CAN
transmit queue. Both CAN and Ethernet `port_based` readback confirm 5.2.4 and
all 29 saved parameters unchanged. Driver: `cam_wip` with
`firmware_update_timeout_s` (C74); no driver runtime or vendor library changes
were needed for the CAN update. Raw Ethernet artifacts are in
`results/umrr96-firmware-20261013/`; CAN update evidence is in
`results/umrr96-can-update-20261013/`, outside git.

All radar result directories now live inside this repository under Git-ignored
`results/`. The previous workspace-level paths are compatibility symlinks;
historical manifests retain their original paths and digests. Reusable source,
build/configuration scripts and a compact verification result are tracked under
[`tools/firmware/`](../tools/firmware/README.md), including
[`results-2026-10-13.json`](../tools/firmware/results-2026-10-13.json).

## Firmware bundle

`UMRR-96 Type 153 - Firmware_Bundle_V5.2.4.7z` (SHA-256 `dfc4af07…de2ebe`), dated
2022-05-12, obtained by the operator. It holds the firmware as smartmicro's XML
download format (`UMRR-96_Type_153_Firmware_v5.2.4.xml`: one software pack,
`umrr9600_radar_icc_cc1`, nine encrypted blocks, 817,704 bytes; every block
matched its declared length and MD5), the vendor changelog, Customer Interface
V13, Ethernet Interface V07, Radar Synchronization V01, the CAN specification
(`.dbc`, `.csdx`) and a DriveRecorder3 desktop file. The vendor documents are
proprietary and are not committed here; this page records only facts needed for
the driver.

5.2.2 → 5.2.4 per the changelog: Ethernet behaviour under heavy load was fixed,
and the instruction port's sequence counter was changed, which the vendor states
is required for communication with Smart Access. BIOS (1.1.0), bootloader
(8.0.2), CAN specification and manuals are unchanged. The vendor's update tool is
DriveRecorder3 3.2 (CAN and Ethernet), which the operator does not have.

## Attempts

The Smart Access update service accepts XML images (user manual, software update
service). Before writing anything, a read-only snapshot recorded all 29
parameters and 14 statuses. The three Ethernet attempts below each first read
the sensor through the same driver process.

| Attempt | Driver instruction format | Read before the update | Update result |
| --- | --- | --- | --- |
| 1 (08:48) | `can_based` (as the readback node) | answered in 0.78 s | SDK `STOPPED_BY_ERROR_TIMEOUT` after 62 s, 0 bytes downloaded |
| 2 (08:56) | `can_based`, under packet capture | answered | same |
| 3 (08:58) | `port_based` (bench default), under packet capture | driver timed out after 3 s | same |

After each attempt the sensor answered normally, reported 5.2.2, and the five
mode parameters read back as in the snapshot.

### CAN bootloader attempt

Customer Interface V13, page 18, specifies that CAN software download requires
the bootloader phase immediately after startup, recognizable by CAN ID `0x320`
echoes. VN1630A hardware CH1 maps to Linux `can0` (`dev_port=0`), at 500 kbit/s.
A 12-second application capture before maintenance decoded 218 complete scans
and 6,588 detections at 18.18 Hz without CAN errors or dropped frames.

After operator authorization, a separate maintenance helper using the original
SDK saved all 29 parameters and 14 statuses, checked serial 230739 and firmware
5.2.2, and waited for the bootloader. The operator cycled power once after the
helper was armed. The capture records update-mode and transfer-mode handshakes,
then 28 data frames (`0x3F4`) for the first block. Hundreds of SDK CAN writes
failed, and the SDK ended with `STOPPED_BY_ERROR_TIMEOUT`, segment 1, zero
acknowledged downloaded bytes after 32.93 seconds. This does not establish that
flash contents were unchanged. The radar continued to emit idle bootloader
echoes and was kept powered until the successful retry.

The host used `txqueuelen=10`; the custom VN1630A driver permits 16 outstanding
frames. The SDK's OSAL calls `write()` once per frame without retrying a failed
write. Increasing only the host queue to `txqueuelen=4096` eliminated SDK write
failures in the retry. That retry used the same hash-checked original XML,
the previous identity/configuration snapshot, and three fresh matching idle
bootloader echoes as its start gate, without another operator power cycle.
The SDK completed all nine segments in **122.82 seconds** with `READY_SUCCESS`.
Its downloaded-byte counter is per segment; the final value 5,000 is the last
segment's length, not the total image size of 817,704 bytes.

Application readback then confirmed serial 230739, firmware **5.2.4**, and all
29 parameters unchanged. A separate read-only Ethernet process using the
unmodified SDK's `port_based` serializer successfully read all 14 statuses and
29 parameters, matching CAN configuration readback. The instruction timeout
observed with 5.2.2 is therefore resolved on this unit after the update (C77).

A subsequent 12-second passive CAN capture contained 218 scan headers at
18.1814 Hz and 7,498 complete target records. All counters advanced by one;
217 scans were complete, with only the final scan cut by the capture boundary.
There were no error frames, receive drops or receive errors during that capture.
The update capture itself contains controller-state transitions and 16 additional
bus errors around the post-transfer application restart/readback; the controller
returned to `ERROR-ACTIVE` with TX/RX error counters zero. SDK write failures
during the successful retry: zero.

Artifacts are in `results/umrr96-can-update-20261013/`, including the initial CAN
attempt `attempt-20261013T172756Z` and successful retry
`attempt-20261013T173635Z/verification.json`; the original XML SHA-256 is
`a1b98c9149868d3b3dbb853bdb6b4d2d222bb0390562f3e29d713ca6f6cc7f61`.

For this adapter/SDK combination, set the host queue before arming the updater:

```bash
sudo ip link set dev can0 up type can bitrate 500000
sudo ip link set dev can0 txqueuelen 4096
```

The first command requires an interface that is currently down. Changing only
`txqueuelen` on an already configured, running interface does not reset the radar.

## What the earlier Ethernet capture shows

UDP between host and sensor was captured with tcpdump (`host 192.168.11.11` on
`enp68s0f0`; Ubuntu's AppArmor profile for tcpdump refuses signals, even from
root, so give tcpdump its own limit such as `-G 300 -W 1` rather than relying on
`timeout` or `kill`) and decoded against the
smartmicro transport header (start `0x7E`, application protocol type, flags, then
optional fields; port protocol payloads start with the port identifier).

- **The update never starts.** In both captured attempts the SDK sent exactly one
  packet: a CAN-format message over Ethernet (application type 4) with CAN ID
  `0x3F2` and data `00 00 00 06 00 00 00 00`, the update start request. The
  sensor never answered it, and the SDK timed out about 62 s later without
  sending image data. The request is the same whichever instruction format the
  driver is configured for.
- **Port-format instructions are answered but rejected.** The `port_based` read
  (port 46) got a port-46 reply 65 ms later, yet Smart Access 3.13.0 reported a
  timeout. This replaces the bring-up note that port-format instructions "did not
  receive replies": 5.2.2 replies, and the SDK cannot correlate its sequence
  counter. The offline analysis below reproduces this failure and isolates the
  counter's byte order.
- **CAN-format instructions work end to end**: a request (application type 4) was
  answered in 27 ms and decoded.
- **The sensor's own log reports cycle overruns.** The sensor sends a text log
  (application type 6) from UDP source port 1234 to the host's data port. With
  CAN target output on (the power-up default; 8.33 Hz), the 6.5-minute capture
  holds 2,234 "Cycle timing exceeds its limit of 120 ms" errors, as many
  "Cycletime exceeded - DATASTREAMS" errors, and 6,695 `FATAL` fault reports
  (ModID 16, FGroup 1, FCode 106), although the target list still arrived every
  120 ms. Whether this persists with CAN target output off (18.18 Hz) is not yet
  measured (C76).

## Offline SDK and firmware analysis

REA 6.4.0 with Ghidra 12.1.4 inspected the installed Smart Access binaries.
An isolated SDK probe then replayed the captured instruction reply over
`127.0.0.1`, with fresh processes and private configuration. This follow-up
sent no traffic to the radar and changed no vendor libraries or driver runtime
code. Evidence, reproduction helpers, native analysis snapshots and firmware
artifacts are in the repository's `results/umrr96-rea-20261013/`, outside git.

### Instruction timeout reproduced

Capture packets 2049 and 2054 contain the request and successful reply for
`auto_interface.sw_version_patch` (section 2012, instruction 5). The request's
port body declares little endian; the reply declares big endian. Both contain
sequence-counter bytes `01 00 00 00` at port offsets 28–31:

| Packet | Declared body byte order | Counter decoded by SDK |
| --- | --- | --- |
| Request | Little endian | 1 |
| Original reply | Big endian | 16,777,216 |
| Reply with counter bytes `00 00 00 01` | Big endian | 1 |

The original reply produced no callback before the probe's 3-second deadline.
Changing only UDP payload offsets 46 and 49 to correct the counter produced
one `SUCCESS` callback after 66.2 ms, with U16 value 2. Both probes generated
instruction records matching the captured request. The reply has no payload
CRC, so the correction did not require changing other payload bytes.

The decompiled path explains the result:

- Base UIF `instruction_fix_endianness` at ELF address `0x50d50` swaps the
  counter at port offset `0x1c` when converting a big-endian body to host order.
- Base UIF `InstructionPortSerializer::deserializePort` at `0x3ca90` copies
  that counter into `InstructionBuffer` offset `0x18`.
- ComLib `DataServices::SetInstructionBuffer` at `0x40500` registers the
  response callback by client ID and sequence; the receive path at `0x40c20`
  looks up the same pair and drops an unmatched reply.

These are ELF virtual addresses; REA's Ghidra imports of these shared libraries
add image base `0x100000`. The source digests and Evidence IDs are retained in
`comlib-evidence.json` and `base-uif-evidence.json`. The packet facts and
experiment are in `instruction-capture.json` and
`instruction-loopback-comparison.json`.

This confirms the cause of the observed 5.2.2 instruction timeout. Keep the
existing CAN-format control process for that firmware; the result does not
establish a supported SDK compatibility patch for arbitrary replies or writes.
It also does not explain the unanswered firmware-update start request.

### Update initiation confirmed separately

ComLib's command builder at ELF address `0x72e70` constructs CAN ID `0x3F2`,
DLC 8, with command 0 producing `00 00 00 06 00 00 00 00`. This matches both
captured update starts. Correcting instruction reply correlation cannot by
itself supply the missing update-mode response. The subsequent CAN capture
confirms that this command is accepted during the bootloader phase;
application-mode Ethernet acceptance remains unverified.

### Firmware plaintext and initial decompilation

The SDK also contains XTEA routines and a mode table that allow offline recovery
of all nine encrypted firmware blocks. Mode 3 uses XTEA with 32 cycles and
big-endian block words. Re-encryption reproduces all 817,704 original ciphertext
bytes; coherent diagnostics, source filenames, flash pointers and decoded
startup code provide separate evidence that the recovered plaintext is useful.
The reproduction helper reads the required table from the original, hash-checked
SDK at runtime; no key values or recovered firmware are committed here.

Ghidra successfully decoded the firmware with
`PowerPC:BE:64:VLE-32addr`, using each XML block's declared load address and
startup-derived small-data register bases. The refined analysis identified
2,547 candidate functions and exported pseudocode for 22 selected functions,
including instruction dispatch, parameter validation, status reads and target
list initialization. This is an initial static analysis, not complete source
recovery; the exact processor chip and runtime RAM contents remain unconfirmed.

REA's raw-image adapter does not admit PowerPC, so firmware analysis used Ghidra
headless directly after the SDK investigation through REA. The saved project,
import/export scripts, decrypted-block hashes and selected assembly/pseudocode
remain in the results directory. Only supplied flash blocks were mapped; missing
flash ranges were not treated as recovered bytes.

Two target-list functions provide concrete driver-facing evidence:

- `0x0103dfc0` constructs the acquisition-setup word with validity bit 0,
  antenna bits 1–2, sweep bits 3–4 and centre-frequency bits 5–6, corroborating
  the bundle manual. It clears the word when the setup cannot be matched.
- `0x0103e17c` assembles 56-byte output records from 52-byte internal detections
  and writes constant zero at record offset 36. The SDK's `TargetImpl::Parse`
  at ELF address `0x41ef0` maps that wire offset to
  `GetFalseAlarmProbability()`: **Pfa is a zero placeholder in this 5.2.4 output
  path**, rather than a computed per-target confidence. The assembly, getter
  mapping and provenance are retained in `target-quality-layout-check.json`.

The recovered image is **5.2.4**, matching the physical sensor after the CAN
update; the initial recorded readback was **5.2.2**. Firmware-internal tables and processing stages do not
establish additional controls or calibrated quality-field semantics for that
sensor. Keep the existing public-interface and validity rules until each proposed
driver change has matching protocol evidence and a focused replay or hardware
check.

## Outcome and remaining checks

The attempted Smart Access update path did not start an update on this unit
from 5.2.2 over Ethernet: its start request was not answered, so nothing was
written in those Ethernet attempts (C75). The CAN bootloader procedure completed
the update and preserved the saved settings. Ethernet port-format readback now
works on 5.2.4 (C77). Application-mode Ethernet firmware updating remains
unverified; the successful CAN procedure does not establish that path.

The existing separate readback process remains in the launch files. Combining
control with the data process needs a driver-level check, despite the successful
standalone SDK reads. Cycle-overrun logging (C76) also needs a new Ethernet
capture before claiming it is resolved.

For an application-mode sensor, arm the CAN updater before the single planned
power cycle needed to enter the vendor's bootloader phase. Once a transfer has
started, keep power connected; a partial transfer cannot be assumed harmless.
