#!/usr/bin/env python3
"""Build the bench maintenance helpers and private SDK configurations; no I/O to radar."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess


SOURCE = Path(__file__).resolve().parent
REPO = SOURCE.parents[1]


def write(path, data):
    path.write_text(json.dumps(data, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--work-dir', type=Path, required=True)
    parser.add_argument('--sdk-dir', type=Path, default=REPO / 'umrr_ros2_driver/smartmicro')
    parser.add_argument('--library-dir', type=Path)
    args = parser.parse_args()
    work = args.work_dir.resolve()
    sdk = args.sdk_dir.resolve()
    library = (args.library_dir or sdk / 'lib-linux-x86_64-gcc_9').resolve()
    if not (sdk / 'include/CommunicationServicesIface.h').is_file():
        raise RuntimeError('Extract the Smart Access SDK before building maintenance helpers')
    work.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(SOURCE / 'read-definitions.json', work / 'read-definitions.json')
    for ethernet, folder in [(False, 'config'), (True, 'port-readback-config')]:
        directory = work / folder
        directory.mkdir(exist_ok=True)
        serializer = 'port_based' if ethernet else 'can_based'
        master = dict(name='UMRR96 bench maintenance', version='1.0.0', client_id=1,
                      role='master', shared_lib_path=str(library), config_path=str(directory),
                      user_interface_name='base', user_interface_major_v=1,
                      user_interface_minor_v=0, user_interface_patch_v=2,
                      instruction_serialization_type=serializer,
                      data_serialization_type=serializer, alive=False, download_path='')
        adapter = (dict(type='eth', dev_id=1, iface_name='enp68s0f0',
                        ip_address='192.168.11.17', port=55558) if ethernet else
                   dict(type='can', dev_id=1, iface_name='can0', baudrate=500000))
        peer = dict(client_id=230739, link_type='eth' if ethernet else 'can', dev_id=1,
                    instruction_serialization_type=serializer, data_serialization_type=serializer,
                    user_interface_name='umrr96_t153_automotive', user_interface_major_v=1,
                    user_interface_minor_v=2, user_interface_patch_v=2)
        peer.update(dict(ip='192.168.11.11', port=55555) if ethernet else dict(can_network_id=0))
        write(directory / 'smart_access_config.json', master)
        write(directory / 'hw_inventory.json', dict(name='Bench adapter', version='1.1.0', hwItems=[adapter]))
        write(directory / 'routing_table.json', dict(name='Bench radar', version='1.0.0', clients=[peer]))
    for name in ['can_maintenance', 'port_readback']:
        command = ['g++', '-std=c++17', '-O2', '-pthread', '-I', str(sdk / 'include'),
                   str(SOURCE / (name + '.cpp')), '-L', str(library),
                   '-Wl,-rpath,' + str(library), '-lsmart_access', '-lcom_lib', '-losal',
                   '-lumrr96_t153_automotivev1.2.2_user_interface', '-o', str(work / name)]
        subprocess.run(command, check=True)
    print('Built helpers and private configurations in ' + str(work))


if __name__ == '__main__':
    main()
