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
# **`espflash` writes a 4 MB flash-size into the APP image header when
# `--flash-size` isn't passed**, regardless of what it correctly
# auto-detects from the connected chip for its own log output (issue
# #306, 2026-08-16). The 2nd-stage bootloader reads that header field
# — not the physically-detected size — to validate the partition
# table at boot, so any factory partition genuinely needing more than
# 4 MB boots into `E (NN) boot: Failed to verify partition table` /
# `flash_parts: partition N invalid ... exceeds flash chip size
# 0x400000` forever, even after a full `espflash erase-flash`. Every
# app flashed through this script before that issue happened to fit
# under 4 MB total (factory + littlefs etc.), so this was silently
# wrong the whole time it just never mattered — and it recurred for
# real on a CoreS3 fst4-ddc-bench flash (2026-08-21): board-info
# correctly reports 16 MB, but the app header still said 4 MB and the
# board boot-looped on the same partition-table-verification error.
#
# **Fixed at the root, not per-board**: when the caller doesn't pass
# `FLASH_SIZE` explicitly, this script now runs `espflash board-info`
# first and parses its own "Flash size: NNMB" line to build the
# `--flash-size` flag automatically — the same value the connected
# chip already reports, just also handed to the part of espflash that
# was silently ignoring it. Falls back to the old (unset) behaviour
# with a warning if `board-info` fails or its output doesn't parse, so
# a flaky port doesn't turn this into a harder failure than before.
# Pass `FLASH_SIZE` explicitly to override the detected value.
#
# Usage:
#   embedded-poc/scripts/flash-monitor.sh <ELF> <LOG_FILE> [DURATION_SEC] [PORT] [PARTITIONS] [FLASH_SIZE]
#
# Defaults:
#   DURATION_SEC = 90
#   PORT         = /dev/ttyACM0
#   PARTITIONS   = ./partitions.csv
#   FLASH_SIZE   = (unset — auto-detected from the connected chip via `espflash board-info`)
#
# Requires `source ~/export-esp.sh` to have been run in the parent shell.

set -euo pipefail

ELF=${1:?usage: flash-monitor.sh <ELF> <LOG_FILE> [DURATION] [PORT] [PARTITIONS] [FLASH_SIZE]}
LOG=${2:?usage: flash-monitor.sh <ELF> <LOG_FILE> [DURATION] [PORT] [PARTITIONS] [FLASH_SIZE]}
DURATION=${3:-90}
PORT=${4:-/dev/ttyACM0}
PARTITIONS=${5:-./partitions.csv}
FLASH_SIZE=${6:-}

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

if [[ -z "$FLASH_SIZE" ]]; then
    # `board-info`'s own line looks like "Flash size:        16MB" —
    # take the last field, lower-case it (espflash's `--flash-size`
    # wants e.g. `16mb`, not `16MB`). `|| true`: don't let a flaky
    # port turn detection failure into a hard script failure under
    # `set -e`; DETECTED simply stays empty and the fallback below
    # warns and proceeds exactly as before this fix existed.
    DETECTED=$(espflash board-info --port "$PORT" 2>/dev/null \
        | grep -i '^Flash size' \
        | awk '{print tolower($NF)}') || true
    if [[ -n "$DETECTED" ]]; then
        FLASH_SIZE="$DETECTED"
        echo "[flash-monitor] auto-detected flash size: $FLASH_SIZE"
    else
        echo "[flash-monitor] WARNING: could not auto-detect flash size from 'espflash board-info' — falling back to espflash's own default (currently 4mb regardless of chip). Pass FLASH_SIZE explicitly if this board's factory partition exceeds 4 MB." >&2
    fi
fi

FLASH_SIZE_FLAG=""
if [[ -n "$FLASH_SIZE" ]]; then
    FLASH_SIZE_FLAG="--flash-size $FLASH_SIZE"
fi

echo "[flash-monitor] running 'espflash flash --monitor' under pty for ${DURATION}s → $LOG${FLASH_SIZE_FLAG:+ (--flash-size $FLASH_SIZE)}"

# `script -qfc CMD FILE`     : run CMD under pty, append transcript to FILE.
# `timeout --foreground`      : SIGTERM cleanly via the pty, not the parent.
# stdin from /dev/null        : monitor will not block on input.
timeout --foreground "$DURATION" \
    script -qfc \
        "espflash flash --monitor --port '$PORT' --partition-table '$PARTITIONS' $FLASH_SIZE_FLAG '$ELF'" \
        "$LOG" \
        </dev/null \
    || true

# Strip CR injected by the pty so logs are pure LF.
sed -i 's/\r$//' "$LOG"

echo "[flash-monitor] done. $(wc -l <"$LOG") lines captured."
