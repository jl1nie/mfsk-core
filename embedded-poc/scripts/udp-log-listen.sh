#!/usr/bin/env bash
# Capture m5stack-s3-app's WiFi UDP log datagrams (Phase 0.6).
#
# Usage:
#   embedded-poc/scripts/udp-log-listen.sh [PORT] [LOG_FILE]
#
# Defaults:
#   PORT     = 9999
#   LOG_FILE = /tmp/m5s3-udp.log
#
# Each datagram from the device renders as one line. Newline ordering
# follows datagram arrival, which matches send order on a quiet LAN
# (no in-flight reordering).

set -euo pipefail

PORT="${1:-9999}"
LOG_FILE="${2:-/tmp/m5s3-udp.log}"

mkdir -p "$(dirname "$LOG_FILE")"

echo "[udp-log-listen] listening on :$PORT → $LOG_FILE  (Ctrl+C to stop)"
nc -lu "$PORT" | tee "$LOG_FILE"
