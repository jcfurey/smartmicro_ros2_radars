#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")"

smart_pack=SmartAccessAutomotive_3_13_0.tar.gz
URL_smartbinaries=https://www.smartmicro.com/fileadmin/media/Downloads/Automotive_Radar/Software/${smart_pack}
# sha256 of ${smart_pack} as published (checked 2026-09-24). Override with
# SMART_ACCESS_SHA256 only after verifying a re-published archive yourself.
SMART_ACCESS_SHA256=${SMART_ACCESS_SHA256:-fcddb3a8234e9b8963cae00f4ed6398c3375e91d1000e8310aa20a7f9cd79447}

cat << EOF

The following clause is explicit for the Smart Access release.

*********************************************************************************

This software is licensed under the Apache 2.0 License

Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The Software is provided "as is", without warranty of any kind, express or implied, including but not limited to the warranties of merchantability, fitness for a particular
purpose and noninfringement. In no event shall the authors or copyright holders be liable for any claim, damages or other liability, whether in an action of contract, tort or
otherwise, arising from, out of or in connection with the software or the use or other dealings in the Software.

*********************************************************************************
EOF
echo
echo -n "Do you accept the agreement you just read? (yes/no)"
echo ""
REPLY=""
read -r REPLY || true
echo ""
case "$REPLY" in
    yes)
    echo "You have accepted the agreement."
    ;;
    *)
    echo "Agreement not accepted." >&2
    exit 1
esac
echo

download_dir=$(mktemp -d)
function cleanup {
    rm -rf "$download_dir"
}
trap cleanup EXIT

function getSmartaccessBinaries {
    # Always download afresh into a private directory: a resumed (-c) or stale
    # file from an earlier attempt could otherwise be extracted.
    wget -O "$download_dir/$smart_pack" "$URL_smartbinaries"
    echo "verifying ${smart_pack}"
    if ! echo "${SMART_ACCESS_SHA256}  $download_dir/$smart_pack" | sha256sum -c -; then
        echo "Checksum mismatch for ${smart_pack}; refusing to extract." >&2
        exit 1
    fi
    echo "extracting smart access"
    tar xfz "$download_dir/$smart_pack" --strip-components=1 -C umrr_ros2_driver/smartmicro/
}

getSmartaccessBinaries
python3 tools/patch_smart_access.py umrr_ros2_driver/smartmicro
