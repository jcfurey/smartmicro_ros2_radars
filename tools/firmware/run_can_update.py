#!/usr/bin/env python3
"""Run the operator-approved original XML update with capture and evidence."""
from datetime import datetime, timezone
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys

EXPECTED = 'a1b98c9149868d3b3dbb853bdb6b4d2d222bb0390562f3e29d713ca6f6cc7f61'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--retry-bootloader', type=Path)
    args = parser.parse_args()
    ROOT = args.work_dir.resolve()
    IMAGE = ROOT / 'UMRR-96_Type_153_Firmware_v5.2.4.xml'
    if hashlib.sha256(IMAGE.read_bytes()).hexdigest() != EXPECTED:
        raise RuntimeError('Original vendor XML digest mismatch')
    state = json.loads(subprocess.check_output(
        ['ip', '-json', '-details', '-statistics', 'link', 'show', 'can0'], text=True))[0]
    can = state['linkinfo']['info_data']
    if state['operstate'] != 'UP' or can['bittiming']['bitrate'] != 500000:
        raise RuntimeError('can0 must already be UP at 500 kbit/s')
    if 'LISTEN-ONLY' in can.get('ctrlmode', []):
        raise RuntimeError('CAN update needs normal mode, not listen-only')
    if state.get('txqlen', 0) < 4096:
        raise RuntimeError('can0 txqueuelen must be at least 4096 for SDK firmware bursts')
    previous = None
    if args.retry_bootloader:
        previous = args.retry_bootloader.resolve()
        if previous.parent != ROOT or not previous.name.startswith('attempt-'):
            raise RuntimeError('Recovery must reference a recorded attempt in this maintenance directory')
        manifest = json.loads((previous / 'manifest.json').read_text())
        terminal = json.loads((previous / 'update-terminal.json').read_text())
        snapshot = json.loads((previous / 'snapshot-before.json').read_text())
        if (manifest['xml_sha256'] != EXPECTED or manifest['interface'] != 'can0' or
                terminal['status'] != 'STOPPED_BY_ERROR_TIMEOUT' or
                terminal['downloaded_bytes'] != 0 or terminal['segment'] != 1 or
                snapshot['statuses']['product_serial'] != 230739):
            raise RuntimeError('Recovery evidence does not match the first-block timeout on this sensor')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    attempt = ROOT / ('attempt-' + stamp)
    attempt.mkdir()
    (ROOT / 'latest-attempt.txt').write_text(str(attempt) + '\n')
    (attempt / 'can0-before.json').write_text(json.dumps(state, indent=2) + '\n')
    (attempt / 'manifest.json').write_text(json.dumps({
        'authorization': 'User resumed the previously held-off CAN firmware update',
        'xml': str(IMAGE), 'xml_sha256': EXPECTED,
        'interface': 'can0', 'adapter_channel': 'VN1630A CH1',
        'start_gate': ('Three fresh matching idle bootloader echoes; verified previous snapshot'
                       if previous else 'Fresh 0x320 bootloader echo after a complete current sensor snapshot'),
        'previous_attempt': str(previous) if previous else None,
        'previous_snapshot_sha256': (hashlib.sha256((previous / 'snapshot-before.json').read_bytes()).hexdigest()
                                     if previous else None),
    }, indent=2) + '\n')
    config = ROOT / 'config/smart_access_config.json'
    command = [str(ROOT / 'can_maintenance'), str(ROOT / 'read-definitions.json'),
               str(attempt), 'retry-bootloader' if previous else 'update', str(IMAGE)]
    if previous:
        command.append(str(previous / 'snapshot-before.json'))
    print('ATTEMPT ' + str(attempt), flush=True)
    with (attempt / 'can-frames.log').open('wb') as frames, \
         (attempt / 'capture.stderr.log').open('wb') as capture_errors, \
         (attempt / 'console.log').open('w', buffering=1) as log:
        capture = subprocess.Popen(['candump', '-L', '-d', '-r', '1048576', 'can0,0:0,#FFFFFFFF'],
                                   stdout=frames, stderr=capture_errors)
        process = None
        try:
            env = dict(os.environ, SMART_ACCESS_CFG_FILE_PATH=str(config))
            process = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1)
            for line in process.stdout:
                log.write(line)
                print(line.rstrip(), flush=True)
            code = process.wait()
            (attempt / 'process-result.json').write_text(json.dumps({'exit_code': code}, indent=2) + '\n')
            print('MAINTENANCE_EXIT ' + str(code), flush=True)
            return code
        finally:
            # Never terminate the maintenance process during a transfer.
            # This owned capture can stop after the SDK process has exited.
            if process is None or process.poll() is not None:
                if capture.poll() is None:
                    capture.send_signal(signal.SIGINT)
                    capture.wait(timeout=10)
                after = subprocess.check_output(
                    ['ip', '-json', '-details', '-statistics', 'link', 'show', 'can0'], text=True)
                (attempt / 'can0-after.json').write_text(after)


if __name__ == '__main__':
    sys.exit(main())
