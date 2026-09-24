# Isolated SDK audit tools

These tools inspect the locally extracted Smart Access SDK without rebuilding or
restarting the ROS driver. See [the findings](../../docs/sdk-audit.md).

Run from the `smartmicro_ros2_radars` repository root. The packet replay script
always binds/sends to `127.0.0.1`, uses ephemeral UDP ports and private configuration,
and never sends sensor instructions. It accepts ordinary Ethernet pcaps with
microsecond timestamps and complete, unfragmented IPv4 UDP datagrams from
`192.168.11.11:55555` to port 55555. SDK fragmentation inside UDP payloads is
preserved. Truncated frames and IP fragments are skipped; captures requiring IP
reassembly need another reader.

```bash
smartmicro_sdk="$PWD/umrr_ros2_driver/smartmicro"
smartmicro_audit_dir=/tmp/umrr96-sdk-audit
mkdir -p "$smartmicro_audit_dir"
g++ -std=c++17 -O2 -pthread -I "$smartmicro_sdk/include" \
  tools/sdk_audit/decode_probe.cpp \
  -L "$smartmicro_sdk/lib-linux-x86_64-gcc_9" \
  -Wl,-rpath,"$smartmicro_sdk/lib-linux-x86_64-gcc_9" \
  -lsmart_access -lcom_lib -losal -lumrr96_t153_automotivev1.2.2_user_interface \
  -o "$smartmicro_audit_dir/decode_probe"
python3 tools/sdk_audit/replay_capture.py \
  /tmp/umrr96-readback/control.pcap \
  "$smartmicro_audit_dir/decode_probe" "$smartmicro_audit_dir/decoded.json"
```

The capture is a local bench artifact, not included in Git. The observer records
up to 30 frames and every available target getter, plus acquisition setup and
device-monitor state before/after replay. JSON objects with `error` or `nonfinite`
indicate unsupported or invalid getter results. Replay feeds the first 4.5 seconds
of eligible data at its original timing. Build dependencies are a C++17 compiler,
the extracted SDK and nlohmann-json headers; the runner uses only Python's standard
library and no ROS domain.

The F32 defect reproducer needs no SDK initialization, shared libraries or socket:

```bash
g++ -std=c++17 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I "$smartmicro_sdk/include" tools/sdk_audit/float_probe.cpp \
  -o "$smartmicro_audit_dir/float_probe"
ASAN_OPTIONS=detect_leaks=0 "$smartmicro_audit_dir/float_probe"
```

With the original vendor header, this fails with an ASan stack buffer overread
in `InstructionBase<float>::GetConvertValue`. With the extraction repair now
applied by `tools/patch_smart_access.py`, it prints `1 3fc00000` without sanitizer
errors. The ordinary `test_sdk_float` regression adds ASan/UBSan coverage for
negative values, signed zero, subnormals, infinities, NaN payloads, read requests
and unchanged integer conversions. `test_sdk_patch` checks repair idempotence
and rejection of unfamiliar or duplicated original code.

`results-2026-09-24.json` retains aggregate observations and the source capture hash;
the full decoded target lists remain in the local output file.
