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

### Internal detection controls found in a further decompile pass

The absence of threshold controls in the public SDK does not establish their
absence inside the firmware. A further read-only saved-project analysis found
section **3014**, registered by the `peak_det.c` initialization path. Its
descriptor is at `0x010bb66c`, with 16 parameter rows at `0x010bb4a8` and one
array dimension containing one element (index 0).

| Internal parameter | Type | Binary default | Declared bounds | Confirmed consumer |
| --- | --- | --- | --- | --- |
| 3014 / 0 | U16 | 1 | 1–3 | `0x0108e960` selects one of the three noise-threshold routines |
| 3014 / 101 | F32 | 1.0 | approximately 0.01–9.0 | The same function scales this value by 128 and supplies it to noise/peak processing |

These defaults were recovered from flash. The subsequent live tests below
confirmed the current values and external access. The threshold scalar does
not establish a command to disable all detection filtering or export
ADC/spectrum data.

A fixed GET-only CAN probe first verified public reads, then received a raw
reply for internal section 3016 / parameter 1 with value 2 (matching the sweep).
Following that internal reply, both the probe and live SDK ROS decoder exited
with SIGSEGV; the exact crash location was not traced. Section 3014 was not
queried. The CAN driver was restored and verified receiving 56 raw clouds and
55 fan clouds in three seconds. Raw CAN capture and MCAP recording continued
through the decoder outage, then were finalized when the operator powered the
radar off. No parameter writes were sent.

Detailed source hashes, rows, decompilation and remaining questions are retained
in `results/umrr96-rea-20261013/density-findings.json` and its adjacent exports.
That initial failure required a response path that avoids unknown sections in
the SDK decoder.

### Live internal-control comparison after power-up

The follow-up used the original SDK only on localhost to serialize fixed
requests. A separate Python SocketCAN process sent those frames on `can0` and
validated the raw replies, with all SDK hardware receivers stopped. Reads
verified serial 230739, firmware patch 4, section 3014 / 0 = **1**, and
3014 / 101 = **1.0**. The wire success status is 0, mapped by the SDK to success
1. The version-5 response CRC covers the three body frames, excludes the header,
and uses CRC16-CCITT with initial value `0xffff`, stored little-endian.

Two bounded comparisons temporarily wrote only section 3014 / 101. Every SET
returned success and was followed by a GET confirming the exact F32 bits. Each
measurement covered eight seconds after a one-second settling interval; the
original value was restored between candidates and at the end of each run.

| Run | Threshold sequence | Median detections per scan, in sequence |
| --- | --- | --- |
| First | 1.0 → 0.9 → 1.0 → 0.75 → 1.0 | 29 → 30 → 31 → 31 → 32 |
| Second | 1.0 → 0.5 → 1.0 → 0.25 → 1.0 | 31 → 31 → 29 → 23 → 30 |

Across **1,459 measured scans**, no tested reduction produced a clear density
increase over the surrounding baseline windows. The 0.25 window had fewer
reported detections. These are short sequential observations of the current
scene, not a controlled target experiment. The output noise field also changed,
so this control should not be treated as a pure export filter.

Capture checks found no CAN errors or interface drops, consecutive scan
counters, and approximately 18 Hz acquisition. All measurement scans were
complete except the last scan in each raw capture, where recording stopped
during its target frames. Both final GETs confirmed **1.0 (`0x3f800000`)**.
All 29 public parameters matched the post-update snapshot; no save-to-flash,
reboot or firmware-update command was sent during these comparisons.

A further static check explains why lowering 101 need not lower every final
threshold. In method 1, `0x0108e5d4` converts **3014 / 102** to an unsigned
integer; `0x010969bc` uses it to build a separate absolute floor. Parameter
**3014 / 103**, internally clamped to at least 1.0, can impose an additional
maximum-spectrum floor above 1.5. The final threshold takes the maximum of the
noise branch times a range factor and that floor. Final readbacks were **102 =
2.0** and **103 = 0.0**; neither was written. Local-maximum selection still
follows threshold generation. Which stage limits density in this scene remains
unresolved.

The compact [live result](../tools/firmware/density-results-2026-10-13.json)
records restoration, window statistics and capture hashes. Full raw CAN,
instruction transcripts, offline serializer sources and analysis are retained
under `results/umrr96-rea-20261013/density-comparison-20261013T203926Z/` and
`density-comparison-20261013T204103Z/`, alongside the earlier decompile exports.
External read/write access to this internal scalar is now established; a
configuration that yields substantially more points, or delegates all radar
filtering to ROS, is not established by these tests.

### Separate floor and combined controls

The next live pass confirmed external read/write access to **3014 / 102**.
Its F32 metadata bounds are approximately 0.00001–35000, signature
`0x1bf8c337`, and initial value 2.0. The processing builder truncates this
parameter to an unsigned integer. Focused read-only exports of `0x010b7630`
and `0x010b2724` confirmed the floor/float-truncation helpers: setting 102 to
0.5 produces integer zero and a zero absolute-floor term, while the independent
noise-estimate threshold remains active.

The measurements held method 1 and the additional floor control 103 = 0.0.
Each window again covered eight seconds after settling, with exact SET/GET
readback and restoration between candidates.

| Test | (floor 102, scalar 101) sequence | Median detections per scan |
| --- | --- | --- |
| Reduced floor | (2, 1) → (1, 1) → (2, 1) → (1, 1) → (2, 1) | 29 → 30 → 31 → 30 → 32 |
| Zero absolute-floor term | (2, 1) → (0.5, 1) → (2, 1) → (0.5, 1) → (2, 1) | 31 → 31 → 31 → 30 → 30 |
| Combined reduction | (2, 1) → (0.5, 0.5) → (2, 1) → (0.5, 0.5) → (2, 1) | 29 → 31 → 30 → 32 → 30 |

The floor-only tests produced no clear density gain. The combined test yielded
approximately 5–6% more detections by mean than each window's neighboring
baseline means. This is a small observed improvement in the current scene,
not a demonstrated substantial increase or an all-detections mode.

All **2,188 measured scans** were complete, with consecutive scan counters,
approximately 18 Hz acquisition and zero CAN errors/drops. Final independent
reads verified **102 = 2.0** and **101 = 1.0**, plus method 1 and 103 = 0.0.
All 29 public parameters matched the preceding snapshot. No save-to-flash,
reset or update command was sent. See the compact
[floor result](../tools/firmware/floor-results-2026-10-13.json); raw captures,
serializer sources, instruction transcripts and analysis remain under
`results/umrr96-rea-20261013/floor-comparison-*` and
`floor-scalar-comparison-*`.

### Remaining SDK and internal tuning leads

A subsequent GET-only snapshot successfully read **all 16 section-3014
parameters**, confirming their current values rather than assuming the flash
defaults. The remaining useful leads are behavioral descriptions inferred from
actual consumers, not recovered vendor parameter names:

| Control | Current value | Established behavior |
| --- | --- | --- |
| 3014 / 0 | 1 | Selects three distinct noise-estimation algorithms; subsequent live comparisons are recorded below |
| 3014 / 3 and 4 | 64 and 25 | Denominator and primary numerator for the selected noise-order rank |
| 3014 / 6 | 4 | Method-1 range sampling interval, with interpolation between noise estimates |
| 3014 / 8 and 9 | 50 and 100 | Mean-noise range window used by methods 2 and 3 |

The rank settings are coupled: ID5 must be less than ID4, and ID4 + ID5 must
not exceed ID3. All 16 parameters are readable over CAN. The follow-up
comparisons below additionally establish SET access for method 0 and primary
rank 4, alongside scalar 101 and floor 102.

Two apparent extra filters are already unrestrictive: ID1 = 0 excludes no
spectrum-edge bins, and ID11 = 1024 does not reject spectral-bin-zero peaks
within the reviewed ≤200-range-gate path. **ID11 is not a candidate-count
limit.** The actual local-maximum worker at `0x010952d4` stops appending when
the descriptor-defined peak buffer is full. Its initializer at `0x01092444`
checks `20 * capacity + 0x24` bytes. The numeric capacity and live saturation
remain unresolved; this is a possible explanation for non-monotonic final
point counts, not an observed overflow.

The user has excluded antenna changes from further experiments. The public
SDK's 29 named parameters also offer manual PRF indices 0–2. The current
1536 MHz short-range waveform is already the highest-bandwidth option in the
shipped UIF. Widening the ±20 m/s validation interval does not explain missing
near-stationary returns. The shared manual's `angular_separation` setting is
documented for Type 132, not this Type 153. There is no named filter-off control
in the delivered Type-153 interface.

IDs 10 and 104 remain unmapped; in particular, the reviewed range-factor
interpolation helper does **not** establish that ID104 controls that factor.
See [the control map](../tools/firmware/detection-controls-2026-10-13.json) for
current values, confidence, constraints and remaining experiments. These final
reads sent no writes; method 1, scalar 1.0 and floor 2.0 remain restored.

### Noise-algorithm and rank comparisons with antenna settings fixed

The next authorized pass explicitly excluded antenna changes. Fixed SDK
requests were again serialized on localhost and sent through the independent
raw-CAN path. The first comparison wrote only **3014 / 0**, confirming that
methods 2 and 3 are externally writable. The second wrote only **3014 / 4**,
confirming primary-rank values 16 and 11 while denominator 3 = 64 and secondary
numerator 5 = 10 stayed fixed. Both candidates satisfy the coupled validation
constraints. Each experiment restored its original value between candidates.

| Test | Setting sequence | Median detections per scan |
| --- | --- | --- |
| Noise algorithm | 1 → 2 → 1 → 3 → 1 | 31 → 30 → 32 → 32 → 34 |
| Primary rank, denominator 64 | 25 → 16 → 25 → 11 → 25 | 30 → 33 → 32 → 30 → 30 |
| (rank, scalar, floor) | (25, 1, 2) → (16, 0.5, 0.5) → (25, 1, 2) → (16, 0.5, 0.5) → (25, 1, 2) | 31 → 31 → 29 → 32 → 31 |

Neither alternative algorithm showed a clear density gain. Rank 16 showed a
small increase within the variation seen in the surrounding baseline windows;
rank 11 showed no gain. The combined change produced approximately 4.1% and
7.3% more detections by mean than the respective neighboring baseline means.
This remains a modest scene-specific change and does not establish an
improvement over the earlier scalar/floor combination alone.

All **2,189 measured scans** were complete with consecutive counters and zero
CAN errors/drops. Every acquisition-setup word remained **49**: valid, antenna
0, sweep 2, center-frequency index 1. Auditing the instruction transcripts
confirmed **zero antenna writes**: every SET was in section 3014 and restricted
to IDs 0, 4, 101 and 102. Final independent reads matched all 16 original
internal values, and all 29 public parameters matched the preceding snapshot.
Method 1, rank 25, scalar 1.0 and floor 2.0 are restored; no save, reset or
firmware-update command was sent.

See the compact
[noise-estimator results](../tools/firmware/noise-estimator-results-2026-10-13.json)
for window statistics, exact writes, acquisition checks and capture hashes.
Full evidence remains under `results/umrr96-rea-20261013/method-comparison-*`,
`rank-comparison-*` and `rank-combined-comparison-*`.

A bounded capacity follow-up confirmed that the peak limit comes from an
input-port descriptor at packet offset `0x20`, rather than a demonstrated SDK
parameter. Its numeric value and saturation remain unresolved; see
`results/umrr96-rea-20261013/capacity-followup.json`. Range-noise sampling
control 3014 / 6 remains untested live. Antenna changes remain excluded from
future experiments.

### Verified update outcome

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

## Ghidra workspace cleanup

The Ghidra installation and saved database were relocated outside the workspace
on 2026-10-13. Analysis exports, source hashes, firmware inputs, CAN captures and
tuning results remain in this repository's ignored `results/` directory.

| Item | Current local path |
| --- | --- |
| Ghidra 12.1.4 installation | `/home/jcfurey/.local/opt/ghidra_12.1.4_PUBLIC` |
| Saved project directory | `/home/jcfurey/GhidraProjects/umrr96_v524` |
| Project name | `umrr96_v524` |
| REA launcher | `/home/jcfurey/.local/bin/rea-radar` |

The launcher and retained `rea-session.mjs` now use the relocated installation.
The empty workspace `.local-tools/` directory was removed and is ignored to
prevent future tool installations appearing as repository content. All saved
project-file hashes and the installation inventory were verified after the
move. The relocation manifest is
`results/umrr96-rea-20261013/ghidra-workspace-cleanup.json`; historical logs keep
the paths that were valid when those records were created.
