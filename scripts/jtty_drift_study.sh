#!/usr/bin/env bash
# D6 (docs/notes/JTTY_UPSTREAM.md): how does a JTTY receiver cope with a
# frequency that drifts across the frame, as a satellite's Doppler shift does?
#
# Usage:
#   scripts/jtty_drift_study.sh [WSJT-X-dir] [work-dir]
#
# Synthesises `CQ K1ABC CQ` (one frame, centred on 1500 Hz, drifting linearly
# by 0..80 Hz/s, SNR -8 and -12 dB in 2500 Hz, deterministic noise, 20 files per
# cell) with this crate's transmitter, decodes the files with this crate's
# receiver (the `drift_study` test) and with WSJT-X's rjtty, and prints the two
# recalls side by side. Upstream has no drift term in its transmitter or its
# receiver; the point of the comparison is to tell "our port is weaker" from
# "the receiver is".
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
WORK="${2:-$REPO_ROOT/target/jtty_drift}"

"$SCRIPT_DIR/build_jttysim.sh" "$WSJTX_DIR" >/dev/null
RJTTY="$REPO_ROOT/target/jttysim/build/rjtty"
rm -rf "$WORK"; mkdir -p "$WORK"

OURS="$WORK/ours.txt"
( cd "$REPO_ROOT" && JTTY_DRIFT_OUT="$WORK/wav" MFSK_REQUIRE_CORPUS=1 \
    cargo test -p mfsk-core --release --no-default-features \
      --features jtty,std,fft-rustfft,parallel --test jtty_rx drift_study \
      -- --ignored --nocapture 2>&1 ) | grep '^DRIFT' > "$OURS"

printf '%-6s %-12s  %-8s %-8s\n' "SNR dB" "drift Hz/s" "ours" "rjtty"
cd "$WORK/wav"
# lines look like: DRIFT snr=-8 drift=12 ours=20/20
while read -r _ snr_kv drift_kv ours_kv; do
  snr="${snr_kv#snr=}"; drift="${drift_kv#drift=}"; ours="${ours_kv#ours=}"
  ok=0
  for f in "$(printf 'drift_d%02d_s%02d_' "$drift" "${snr#-}")"*.wav; do
    # capture first: `grep -q` exiting early would SIGPIPE rjtty and, under
    # pipefail, turn a match into a failure
    out="$("$RJTTY" 4.6 0 384 1500 50 "$f")"
    if grep -q 'CQ K1ABC CQ$' <<<"$out"; then ok=$((ok+1)); fi
  done
  printf '%-6s %-12s  %-8s %-8s\n' "$snr" "$drift" "$ours" "$ok/20"
done < "$OURS"
