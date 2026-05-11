#!/usr/bin/env bash
# Standardized flash + serial-capture for ESP32-S3 / ESP32 boards.
#
# We use `espflash flash --monitor` as a SINGLE process under `script(1)`,
# which gives it a pty so the interactive monitor half works while we
# still get a transcript file.
#
# Why not split into two processes (`espflash flash` + `espflash monitor`)?
# On S3 USB-OTG boards every fresh `espflash monitor` attach issues a
# reset (DTR pulse on the USB CDC virtual port) that drops the chip
# back into DOWNLOAD mode (rst:0x15 USB_UART_CHIP_RESET, boot:0x2
# DOWNLOAD), so the just-flashed app never starts. None of
# `--before {default-reset,no-reset,usb-reset,no-reset-no-sync}` fixed
# this reliably. The combined `--monitor` mode skips that second reset.
#
# Re-flashing the SAME ELF prints "Segment … has not changed, skipping
# write" and finishes in ~5 s. That is NOT a successful flash — touch a
# source file or change a `log::info!` line to force a real rewrite,
# and expect ~15-25 s for a real factory-partition write.
#
# Usage:
#   embedded-poc/scripts/flash-monitor.sh <ELF> <LOG_FILE> [DURATION_SEC] [PORT] [PARTITIONS]
#
# Defaults:
#   DURATION_SEC = 90
#   PORT         = /dev/ttyACM0
#   PARTITIONS   = ./partitions.csv
#
# Requires `source ~/export-esp.sh` to have been run in the parent shell.

set -euo pipefail

ELF=${1:?usage: flash-monitor.sh <ELF> <LOG_FILE> [DURATION] [PORT] [PARTITIONS]}
LOG=${2:?usage: flash-monitor.sh <ELF> <LOG_FILE> [DURATION] [PORT] [PARTITIONS]}
DURATION=${3:-90}
PORT=${4:-/dev/ttyACM0}
PARTITIONS=${5:-./partitions.csv}

if [[ ! -f "$ELF" ]]; then
    echo "ELF not found: $ELF" >&2
    exit 1
fi
if [[ ! -f "$PARTITIONS" ]]; then
    echo "Partitions CSV not found: $PARTITIONS" >&2
    exit 1
fi

mkdir -p "$(dirname "$LOG")"
: > "$LOG"

echo "[flash-monitor] running 'espflash flash --monitor' under pty for ${DURATION}s → $LOG"

# `script -qfc CMD FILE`     : run CMD under pty, append transcript to FILE.
# `timeout --foreground`      : SIGTERM cleanly via the pty, not the parent.
# stdin from /dev/null        : monitor will not block on input.
timeout --foreground "$DURATION" \
    script -qfc \
        "espflash flash --monitor --port '$PORT' --partition-table '$PARTITIONS' '$ELF'" \
        "$LOG" \
        </dev/null \
    || true

# Strip CR injected by the pty so logs are pure LF.
sed -i 's/\r$//' "$LOG"

echo "[flash-monitor] done. $(wc -l <"$LOG") lines captured."
