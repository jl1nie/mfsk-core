#!/usr/bin/env bash
# Flash + capture, with the five failure modes that cost a whole
# session on 2026-09-01 handled once instead of rediscovered each time.
#
#   capture.sh <elf> <log-basename> <seconds> [marker-regex]
#
# `flash-monitor.sh` does the flashing and knows the espflash quirks
# (single process for flash+monitor, --flash-size from board-info). This
# wrapper is about everything *around* that call:
#
# 1. **The port must be free.** A previous capture's espflash still
#    holds /dev/ttyACM0 for its whole window, and the new one then dies
#    with "Failed to open serial port" — or worse, is skipped by a
#    guard and looks like it ran. Waits, with a timeout, and checks with
#    `pgrep -x espflash` (`pgrep -f 'espflash flash'` matches the
#    waiting shell's own command line — that mistake cost a 6-minute
#    hang).
# 2. **The board must actually be attached to WSL.** `usbipd list` can
#    say `Attached` while `lsusb` shows nothing and /dev/ttyACM0 is a
#    stale node; espflash then reports "Error while connecting". Re-runs
#    `usbipd attach`, and if that cannot be fixed from here (device gone
#    from Windows, or unbound and needing an admin `usbipd bind`) says
#    exactly which physical step is needed rather than retrying.
# 3. **Log files are never overwritten.** A rerun that reuses a name
#    destroys the measurement it was meant to compare against; that
#    happened. `-2`, `-3`, … are appended.
# 4. **A capture that produced nothing is a failure, not a result.**
#    With a marker regex, exits non-zero when the marker never appeared,
#    so a caller cannot mistake an empty log for a measurement.
# 5. **The app may take the USB console.** `apps::ft4`/`fst4`/`wspr`
#    install the USB host driver through the display panel, which
#    detaches USB-Serial-JTAG: the *running image* then cannot be
#    re-flashed without a manual download-mode entry. Detected up front
#    by name so the message says "hold RST ~2 s" instead of leaving the
#    caller to guess.
set -uo pipefail

ELF=${1:?usage: capture.sh <elf> <log-basename> <seconds> [marker-regex]}
LOGBASE=${2:?}
SECS=${3:-120}
MARKER=${4:-}

USBIPD=${USBIPD:-/mnt/c/Program Files/usbipd-win/usbipd.exe}
VIDPID=${VIDPID:-303a:1001}
PORT=${PORT:-/dev/ttyACM0}
export FLASH_SIZE=${FLASH_SIZE:-16mb}

here=$(cd "$(dirname "$0")" && pwd)

say() { printf '[capture] %s\n' "$*"; }
die() { printf '[capture] %s\n' "$*" >&2; exit 1; }

# ── 1. port free ────────────────────────────────────────────────────
waited=0
while pgrep -x espflash >/dev/null; do
    if [ "$waited" -eq 0 ]; then
        say "another espflash is running — waiting for it to finish"
    fi
    sleep 5
    waited=$((waited + 5))
    [ "$waited" -ge 400 ] && die "port still busy after ${waited}s; stop the other capture first"
done

# ── 2. board attached to WSL, not just to Windows ───────────────────
attach_if_needed() {
    lsusb 2>/dev/null | grep -qi "${VIDPID/:/:}" && return 0
    [ -x "$USBIPD" ] || return 1
    local row
    row=$("$USBIPD" list 2>/dev/null | grep -i "$VIDPID" | head -1)
    [ -n "$row" ] || return 2          # gone from Windows too
    local busid state
    busid=$(awk '{print $1}' <<<"$row")
    state=$(awk '{print $NF}' <<<"$row")
    case "$state" in
        # `Attached` while lsusb sees nothing is the stale case: detach
        # first, or the re-attach is refused as already done.
        Attached|接続済み) "$USBIPD" detach --busid "$busid" >/dev/null 2>&1; sleep 2 ;;
        "Not shared"|共有されていません) return 3 ;;
    esac
    "$USBIPD" attach --wsl --busid "$busid" >/dev/null 2>&1
    sleep 3
    lsusb 2>/dev/null | grep -qi "$VIDPID"
}

if ! attach_if_needed; then
    case $? in
      2) die "the board is not enumerated on Windows at all.
   Unplug USB, hold the CoreS3 power button ~6 s to power it down,
   plug it back in, then press the power button to switch it on." ;;
      3) die "the board is present but not shared. In an ADMIN PowerShell:
   usbipd bind --busid <busid>   (see: usbipd list)" ;;
      *) die "could not attach the board to WSL; check 'usbipd list' and the cable." ;;
    esac
fi
[ -e "$PORT" ] || die "$PORT missing even after attach"

# ── 3. a fresh log name ─────────────────────────────────────────────
log="$LOGBASE"
n=2
while [ -e "$log" ]; do
    log="${LOGBASE%.log}-$n.log"
    n=$((n + 1))
done
[ "$log" = "$LOGBASE" ] || say "log exists; writing $log instead"

# ── 4. flash + capture ──────────────────────────────────────────────
say "flashing $(basename "$ELF") → $log (${SECS}s)"
"$here/flash-monitor.sh" "$ELF" "$log" "$SECS"
rc=$?

# ── 5. did it produce anything? ─────────────────────────────────────
if grep -qa "waiting for download" "$log" 2>/dev/null; then
    die "the board is parked in DOWNLOAD mode. Press RST (short) and rerun.
   If the running image installs the USB host driver (the ft4/fst4/wspr
   *apps*, not the benches), it detaches the console: hold RST ~2 s to
   enter download mode first."
fi
if grep -qa "Error while connecting" "$log" 2>/dev/null; then
    die "espflash could not reach the board — see the two notes above; a
   short RST press is the usual fix."
fi

if [ -n "$MARKER" ]; then
    if grep -qa "$MARKER" "$log"; then
        say "marker '$MARKER' seen — capture usable ($(wc -l <"$log") lines)"
    else
        die "marker '$MARKER' never appeared in $log — the capture ran but
   produced nothing to measure (window too short, or the app did not get
   that far)."
    fi
fi
exit $rc
