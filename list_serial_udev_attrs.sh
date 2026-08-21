#!/bin/bash
set -euo pipefail

for dev in /dev/ttyACM* /dev/ttyUSB* /dev/ttyTHS*; do
    [[ -e "${dev}" ]] || continue

    echo
    echo "== ${dev} =="
    udevadm info -q property -n "${dev}" \
        | grep -E '^(DEVNAME|ID_BUS|ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_SERIAL_SHORT|ID_PATH|ID_USB_INTERFACE_NUM)=' \
        || true
done
