#!/usr/bin/env bash
# Generate ft8sim WAV files for SNR sweep + fading benchmark.
#
# Usage:
#   scripts/gen_ft8_sweep_wavs.sh [ft8sim-path] [out-dir]
#
# File naming:  ft8_<channel>_<snr>_<trial>.wav
#   channel:  awgn | ccir_good | ccir_moderate | ccir_poor
#   snr:      m05 = -5 dB, m24 = -24 dB, etc.
#   trial:    01..TRIALS
#
# Run build_ft8sim.sh first if the binary doesn't exist.
# Existing files are skipped (safe to re-run after widening the grid).
# Jobs run in parallel (JOBS env var, default: nproc).
#
# Mirrors scripts/gen_ft4_sweep_wavs.sh (see docs/notes/FT8_BENCHMARK.md).
# FT8 has no sub-modes, so there's one SNR grid instead of one per period.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FT8SIM="${1:-$REPO_ROOT/target/ft8sim/ft8sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/ft8_sweep}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"

if [[ ! -x "$FT8SIM" ]]; then
  echo "error: ft8sim not found at $FT8SIM" >&2
  echo "Run scripts/build_ft8sim.sh first." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

MSG="CQ JL1NIE PM95"
F0=1500
DT=0.0
TRIALS=20

# Published WSJT-X AWGN threshold (2500 Hz ref BW) is ~-20 to -21 dB (vs
# FT4's -17.5 dB — FT8's longer 15 s slot / deeper FEC interleaving buys
# back sensitivity FT4 trades away for throughput). Grid extends a few dB
# past it on both sides so the 50% crossing is observed rather than
# censored at the grid edge (see the FST4 #146 lesson in
# docs/notes/FST4_BENCHMARK.md section 3).
SNRS="-5 -10 -15 -17 -18 -19 -20 -21 -22 -23 -24 -25 -26"

CHANNELS=(
  "awgn          0.0  0.0"
  "ccir_good     0.1  0.5"
  "ccir_moderate 0.5  1.0"
  "ccir_poor     1.0  2.0"
)

snr_tag() {
  local snr=$1
  if (( snr < 0 )); then
    printf "m%02d" "$(( -snr ))"
  else
    printf "p%02d" "$snr"
  fi
}

# One worker function per (chan, snr) cell — each gets its own tmpdir.
run_cell() {
  local chan=$1 fdop=$2 del=$3 snr=$4

  local tag; tag="$(snr_tag "$snr")"

  # Skip if all trials already exist.
  local missing=0
  for T in $(seq 1 "$TRIALS"); do
    local dest="$OUT_DIR/ft8_${chan}_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  FT8  %-14s  SNR=%4d dB  generating %d files ...\n" \
    "$chan" "$snr" "$TRIALS"

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' EXIT

  (
    cd "$tmpd"
    "$FT8SIM" "$MSG" "$F0" "$DT" "$fdop" "$del" "$TRIALS" "$snr" \
      >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(printf '000000_%06d.wav' "$T")"
      dest="$OUT_DIR/ft8_${chan}_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell snr_tag
export FT8SIM OUT_DIR TRIALS MSG F0 DT

# Build the full list of (chan fdop del snr) tuples, then fan out.
CELLS=()
for CHAN_SPEC in "${CHANNELS[@]}"; do
  read -r CHAN FDOP DEL <<< "$CHAN_SPEC"
  for SNR in $SNRS; do
    CELLS+=("$CHAN $FDOP $DEL $SNR")
  done
done

echo "Generating FT8 sweep corpus: ${#CELLS[@]} cells, TRIALS=$TRIALS, JOBS=$JOBS"
echo "Output: $OUT_DIR"
echo ""

# Run cells in parallel using a simple job-pool (no GNU parallel needed).
active=0
pids=()
for CELL in "${CELLS[@]}"; do
  read -r chan fdop del snr <<< "$CELL"
  run_cell "$chan" "$fdop" "$del" "$snr" &
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
echo "  MFSK_FT8_SWEEP_DIR=$OUT_DIR \\"
echo "    cargo test --test ft8_sweep --release --features ft8,fft-rustfft,parallel,uvpacket \\"
echo "    -- --ignored --nocapture"
