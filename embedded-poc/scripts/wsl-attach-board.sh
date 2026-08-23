#!/usr/bin/env bash
# Re-attach the ESP32-S3 dev board to WSL after it re-enumerates.
#
# WHY THIS EXISTS
#
# CoreS3 flashes over the ESP32-S3's *native* USB-Serial-JTAG, so the
# USB device is the chip. Every chip reset — including the one
# `espflash` performs after writing, and any press of the board's
# button — tears that device down and brings it back as a new one.
# Under WSL the new device is not attached to the Linux side until
# someone runs `usbipd attach` again, so from inside WSL the board
# simply vanishes: `/dev/ttyACM0` is gone and every flash fails with
# "Serial port not found".
#
# That is the real cost behind `flash-monitor.sh`'s single-process
# design, and it was not written down anywhere — which is how a
# session ended up asking for "just press reset" a dozen times without
# accounting for what each one costs. A reset is not free here.
#
# USAGE
#
#   embedded-poc/scripts/wsl-attach-board.sh            # wait, then attach
#   embedded-poc/scripts/wsl-attach-board.sh 60         # give up after 60 s
#
# Exits 0 once /dev/ttyACM0 is present, 1 on timeout. Safe to run when
# the board is already attached — it says so and returns.
#
# Needs `usbipd-win` on the Windows side. Attaching may require an
# elevated Windows prompt the first time a device is bound; if this
# reports "not shared", run `usbipd bind --busid <BUSID>` there once.
set -euo pipefail

TIMEOUT=${1:-45}
USBIPD='/mnt/c/Program Files/usbipd-win/usbipd.exe'

if [[ ! -x "$USBIPD" ]]; then
    echo "[wsl-attach] usbipd.exe not found at $USBIPD — is usbipd-win installed?" >&2
    exit 1
fi

if [[ -e /dev/ttyACM0 ]]; then
    echo "[wsl-attach] /dev/ttyACM0 already present; nothing to do."
    exit 0
fi

# The board shows up on the Windows side a moment after it
# re-enumerates; poll for it rather than failing on the first look.
deadline=$(( SECONDS + TIMEOUT ))
busid=""
while (( SECONDS < deadline )); do
    # Only the "Connected:" block carries BUSIDs — the "Persisted:"
    # block below it lists GUIDs of devices that are *not* present, and
    # matching those yields an id `attach` will reject.
    busid=$("$USBIPD" list 2>/dev/null \
        | sed -n '/^Connected:/,/^$/p' \
        | grep -iE 'ESP32-S3|JTAG/serial debug unit' \
        | grep -oE '^[0-9]+-[0-9]+' \
        | head -1) || true
    [[ -n "$busid" ]] && break
    sleep 1
done

if [[ -z "$busid" ]]; then
    echo "[wsl-attach] no ESP32-S3 on the Windows side after ${TIMEOUT}s." >&2
    echo "[wsl-attach]   The board is powered off, or its USB cable is out. This script" >&2
    echo "[wsl-attach]   cannot help with either." >&2
    exit 1
fi

echo "[wsl-attach] attaching busid $busid"
"$USBIPD" attach --wsl --busid "$busid" || true

# udev needs a moment to create the node.
for _ in $(seq 1 20); do
    if [[ -e /dev/ttyACM0 ]]; then
        echo "[wsl-attach] /dev/ttyACM0 is up."
        exit 0
    fi
    sleep 0.5
done

echo "[wsl-attach] attached busid $busid but /dev/ttyACM0 never appeared." >&2
exit 1
