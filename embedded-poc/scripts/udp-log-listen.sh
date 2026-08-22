#!/usr/bin/env bash
# Capture the UDP log datagrams the CoreS3 / M5StickS3 apps fan out.
#
# Usage:
#   embedded-poc/scripts/udp-log-listen.sh [PORT] [LOG_FILE]
#
# Defaults:
#   PORT     = 9999
#   LOG_FILE = /tmp/mfsk-udp.log
#
# Each datagram renders as one line, prefixed with arrival time and
# sender. Newline ordering follows datagram arrival, which matches send
# order on a quiet LAN (no in-flight reordering).
#
# WHY THIS IS PYTHON AND NOT `nc -lu`
#
# It was `nc -lu "$PORT" | tee`, and on Ubuntu's OpenBSD netcat
# (1.226) that receives **nothing** — verified 2026-08-22 against a
# board that a five-line Python listener was reading happily at the
# same moment, on the same port, from the same sender. Neither
# `nc -lu 9999` nor `nc -ulp 9999` produced a byte.
#
# That is not a footnote. This script is the *replacement console*:
# installing the USB host driver detaches USB-Serial-JTAG, so from
# that moment it is the only way to see what the device is doing. A
# listener that silently receives nothing, during a session where the
# device has also gone quiet on serial, is indistinguishable from a
# device that crashed — and the session ends with no result. See
# `docs/reference/EMBEDDED.md`'s "Live UAC bring-up" for where this
# sits in the procedure (issue #163).
set -euo pipefail

PORT="${1:-9999}"
LOG_FILE="${2:-/tmp/mfsk-udp.log}"

mkdir -p "$(dirname "$LOG_FILE")"
echo "[udp-log-listen] listening on 0.0.0.0:$PORT → $LOG_FILE  (Ctrl+C to stop)"

exec python3 -u -c '
import socket, sys, time

port = int(sys.argv[1])
path = sys.argv[2]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# 0.0.0.0 so a device configured to send to the subnet broadcast is
# heard as well as one sending unicast.
sock.bind(("0.0.0.0", port))

with open(path, "a", buffering=1) as log:
    while True:
        data, addr = sock.recvfrom(65535)
        text = data.decode("utf-8", "replace").rstrip("\n")
        for line in text.split("\n"):
            out = "%s %s %s" % (time.strftime("%H:%M:%S"), addr[0], line)
            print(out)
            log.write(out + "\n")
' "$PORT" "$LOG_FILE"
