#!/usr/bin/env bash
# Generate jt9sim WAV files for a JT9 AWGN SNR sweep.
#
# Usage:
#   scripts/gen_jt9_sweep_wavs.sh [jt9sim-path] [out-dir]
#
# Defaults:
#   jt9sim-path  target/jt9sim/jt9sim
#   out-dir      embedded-poc/assets/jt9_sweep/
#
# Run scripts/build_jt9sim.sh first if the binary doesn't exist.
#
# File naming:  jt9_awgn_<snr_tag>_<trial>.wav
#   snr_tag:  m20 = -20 dB, p05 = +5 dB
#   trial:    01..TRIALS
#
# jt9sim's CLI is positional (no getopt-style flags, unlike jt65sim),
# so negative SNR values pass through directly with no backslash
# escaping needed:
#   jt9sim "message" fspan nsigs minutes SNR nfiles
# f0 is hardcoded to 1400 Hz inside jt9sim.f90 (no CLI override).
#
# Existing files are skipped (safe to re-run after widening the grid).
# Jobs run in parallel (JOBS env var, default: nproc).
# Trials per cell: TRIALS env var, default 20. Raising it does not
# extend an existing corpus: a cell is skipped only when every trial is
# already there, so an incomplete one is regenerated whole (the
# simulator is invoked once for all TRIALS) and the files already
# present are overwritten with a fresh set of realisations. A trial
# index therefore names a different signal afterwards. Generate into a
# separate out-dir (2nd positional arg) rather than in place.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

JT9SIM="${1:-$REPO_ROOT/target/jt9sim/jt9sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/jt9_sweep}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"

# run_cell cd's into a tmpdir before invoking jt9sim, so both paths
# must be absolute regardless of how the caller passed them in.
[[ -x "$JT9SIM" ]] && JT9SIM="$(cd "$(dirname "$JT9SIM")" && pwd)/$(basename "$JT9SIM")"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

if [[ ! -x "$JT9SIM" ]]; then
  echo "error: jt9sim not found at $JT9SIM" >&2
  echo "Run scripts/build_jt9sim.sh first." >&2
  exit 1
fi

MSG="CQ JL1NIE PM95"
FSPAN=200
TRIALS="${TRIALS:-20}"

# WSJT-X's published JT9 AWGN threshold (2500 Hz ref BW) is around
# -25 to -26 dB; grid extends a few dB past it on both sides so the
# 50% crossing is observed, not censored at the grid edge (mirrors
# the FST4 #146 lesson and the JT65 sweep in tests/jt65_sweep.rs).
SNRS="10 5 0 -5 -10 -15 -18 -20 -22 -24 -25 -26 -27 -28 -30"

snr_tag() {
  local snr=$1
  local rounded
  rounded="$(printf '%.0f' "$snr")"
  if (( rounded < 0 )); then
    printf "m%02d" "$(( -rounded ))"
  else
    printf "p%02d" "$rounded"
  fi
}

run_cell() {
  local snr=$1
  local tag; tag="$(snr_tag "$snr")"

  local missing=0
  for T in $(seq 1 "$TRIALS"); do
    local dest="$OUT_DIR/jt9_awgn_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  JT9  SNR=%4s dB  generating %d files ...\n" "$snr" "$TRIALS"

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' EXIT

  (
    cd "$tmpd"
    "$JT9SIM" "$MSG" "$FSPAN" 1 1 "$snr" "$TRIALS" >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(printf '000000_%04d.wav' "$((T - 1))")"
      dest="$OUT_DIR/jt9_awgn_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell snr_tag
export JT9SIM OUT_DIR TRIALS MSG FSPAN

echo "Generating JT9 AWGN sweep corpus: $(echo $SNRS | wc -w) SNR points, TRIALS=$TRIALS, JOBS=$JOBS"
echo "Output: $OUT_DIR"
echo ""

active=0
pids=()
for SNR in $SNRS; do
  run_cell "$SNR" &
  pids+=($!)
  (( active++ )) || true
  if (( active >= JOBS )); then
    wait "${pids[0]}"
    pids=("${pids[@]:1}")
    (( active-- )) || true
  fi
done
for pid in "${pids[@]}"; do
  wait "$pid"
done

echo ""
echo "Done. Assets: $OUT_DIR"
echo "  $(ls "$OUT_DIR" | wc -l) files total"
echo ""
echo "Run the sweep test with:"
echo "  cargo test --test jt9_sweep --release --features jt9,fft-rustfft,uvpacket \\"
echo "    -- --ignored --nocapture"
