#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Replay a captured radar stream into an isolated SDK observer on loopback only."""
import argparse
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import time


def packets(path):
    with path.open('rb') as capture:
        header = capture.read(24)
        endian = {b'\xd4\xc3\xb2\xa1': '<', b'\xa1\xb2\xc3\xd4': '>'}.get(header[:4])
        if endian is None or struct.unpack(endian + 'I', header[20:24])[0] != 1:
            raise ValueError('Expected microsecond-resolution Ethernet pcap')
        while record := capture.read(16):
            sec, usec, size, original = struct.unpack(endian + 'IIII', record)
            frame = capture.read(size)
            if size != original or len(frame) < 42 or frame[12:14] != b'\x08\x00':
                continue
            ip = frame[14:]
            if ip[9] != 17 or ip[12:16] != socket.inet_aton('192.168.11.11'):
                continue
            # Reject IP fragments; SDK fragmentation inside complete UDP datagrams is retained.
            if struct.unpack('!H', ip[6:8])[0] & 0x3fff:
                continue
            udp = ip[(ip[0] & 15) * 4:]
            source, destination, length, checksum = struct.unpack('!HHHH', udp[:8])
            if source == 55555 and destination == 55555 and len(udp) >= length:
                yield sec + usec * 1e-6, udp[8:length]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('probe', type=Path)
    parser.add_argument('report', type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[2]
    sdk = repo / 'umrr_ros2_driver/smartmicro'
    with tempfile.TemporaryDirectory(prefix='umrr96-sdk-audit-') as directory:
        config = Path(directory)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as available:
            available.bind(('127.0.0.1', 0))
            receiver_port = available.getsockname()[1]
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            sender.bind(('127.0.0.1', 0))
            for name, data in {
                'smart_access_config.json': dict(
                    name='Offline SDK audit', version='1.0.0', client_id=1, role='master',
                    shared_lib_path=str(sdk / 'lib-linux-x86_64-gcc_9'), config_path=str(config),
                    user_interface_name='base', user_interface_major_v=1,
                    user_interface_minor_v=0, user_interface_patch_v=2,
                    instruction_serialization_type='port_based', data_serialization_type='port_based',
                    alive=False, download_path=''),
                'hw_inventory.json': dict(name='Loopback only', version='1.1.0', hwItems=[dict(
                    type='eth', dev_id=1, iface_name='lo', ip_address='127.0.0.1', port=receiver_port)]),
                'routing_table.json': dict(name='Offline peer', version='1.0.0', clients=[dict(
                    client_id=230739, link_type='eth', dev_id=1, ip='127.0.0.1',
                    port=sender.getsockname()[1], instruction_serialization_type='port_based',
                    data_serialization_type='port_based', user_interface_name='umrr96_t153_automotive',
                    user_interface_major_v=1, user_interface_minor_v=2, user_interface_patch_v=2)])
            }.items():
                (config / name).write_text(json.dumps(data))
            with tempfile.TemporaryFile(mode='w+') as log:
                process = subprocess.Popen([str(args.probe.resolve()), str(args.report.resolve())],
                    env=dict(os.environ, SMART_ACCESS_CFG_FILE_PATH=str(config / 'smart_access_config.json')),
                    stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline:
                        log.seek(0)
                        if 'READY' in log.read():
                            break
                        if process.poll() is not None:
                            raise RuntimeError('Probe exited during initialization')
                        time.sleep(.05)
                    else:
                        raise RuntimeError('Probe did not become ready')
                    origin = None
                    started = time.monotonic()
                    sent = 0
                    for timestamp, payload in packets(args.capture):
                        if origin is None:
                            origin = timestamp
                        offset = timestamp - origin
                        if offset > 4.5:
                            break
                        time.sleep(max(0, started + offset - time.monotonic()))
                        sender.sendto(payload, ('127.0.0.1', receiver_port))
                        sent += 1
                    assert process.wait(timeout=15) == 0
                    report = json.loads(args.report.read_text())
                    report['udp_datagrams_replayed'] = sent
                    args.report.write_text(json.dumps(report, indent=2) + '\n')
                    print(json.dumps(dict(frames=len(report['frames']),
                        points=sum(len(f['targets']) for f in report['frames']),
                        connected_before=report['connected_before_replay'],
                        connected_after=report['connected_after_replay'],
                        first_frame={k: v for k, v in report['frames'][0].items() if k != 'targets'},
                        first_point=report['frames'][0]['targets'][0]), indent=2))
                finally:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                    if process.returncode:
                        log.seek(0)
                        print(log.read())


if __name__ == '__main__':
    main()
