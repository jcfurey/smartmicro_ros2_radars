# UMRR-96 firmware 5.2.4 update attempt (2026-10-13)

Sensor: UMRR-96 Type 153, serial 230739 (client ID `0x00038553`) at
192.168.11.11, firmware **5.2.2**. Driver: `cam_wip` with `firmware_update_timeout_s`
(C74). Outcome: **not updated**; the sensor stayed on 5.2.2 with every setting
unchanged. Raw artifacts (pre-flash snapshot, node logs, packet capture, decoder)
are in the workspace's `results/umrr96-firmware-20261013/`, outside git.

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
parameters and 14 statuses, and each attempt first read the sensor through the
same driver process.

| Attempt | Driver instruction format | Read before the update | Update result |
| --- | --- | --- | --- |
| 1 (08:48) | `can_based` (as the readback node) | answered in 0.78 s | SDK `STOPPED_BY_ERROR_TIMEOUT` after 62 s, 0 bytes downloaded |
| 2 (08:56) | `can_based`, under packet capture | answered | same |
| 3 (08:58) | `port_based` (bench default), under packet capture | driver timed out after 3 s | same |

After each attempt the sensor answered normally, reported 5.2.2, and the five
mode parameters read back as in the snapshot.

## What the capture shows

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
  receive replies": 5.2.2 replies, and the SDK discards the reply, consistent with
  the instruction-port sequence-counter change in 5.2.4.
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

## Conclusion and next steps

The Smart Access update path cannot update this unit from 5.2.2 over Ethernet:
its start request is not answered, so nothing is written (C75). Options:

1. Update over a CAN bus. The start request is a CAN message, and early firmware
   was updated over CAN only. Needs a SocketCAN adapter on the sensor's CAN
   pins and a CAN adapter in the driver configuration; try it the same way
   (snapshot, read, then update).
2. Ask smartmicro support (automotive) for the supported path from 5.2.2 to
   5.2.4 without DriveRecorder3, citing the unanswered `0x3F2` start request, the
   rejected port-46 replies and the logged cycle overruns.

Do not power-cycle the sensor during any future attempt; a transfer that has
started cannot be assumed recoverable without vendor confirmation.
