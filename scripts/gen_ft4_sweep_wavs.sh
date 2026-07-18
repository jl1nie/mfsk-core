#!/usr/bin/env bash
# Generate ft4sim WAV files for SNR sweep + fading benchmark.
#
# Usage:
#   scripts/gen_ft4_sweep_wavs.sh [ft4sim-path] [out-dir]
#
# File naming:  ft4_<channel>_<snr>_<trial>.wav
#   channel:  awgn | ccir_good | ccir_moderate | ccir_poor
#   snr:      m05 = -5 dB, m24 = -24 dB, etc.
#   trial:    01..TRIALS
#
# Run build_ft4sim.sh first if the binary doesn't exist.
# Existing files are skipped (safe to re-run after widening the grid).
# Jobs run in parallel (JOBS env var, default: nproc).
#
# Mirrors scripts/gen_fst4_sweep_wavs.sh (see docs/notes/FT4_BENCHMARK.md).
# FT4 has no sub-modes, so there's one SNR grid instead of one per period.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FT4SIM="${1:-$REPO_ROOT/target/ft4sim/ft4sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/ft4_sweep}"
JOBS="${JOBS:-$(nproc)}"

if [[ ! -x "$FT4SIM" ]]; then
  echo "error: ft4sim not found at $FT4SIM" >&2
  echo "Run scripts/build_ft4sim.sh first." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

MSG="CQ JL1NIE PM95"
F0=1500
DT=0.0
TRIALS=20

# Published WSJT-X AWGN threshold (2500 Hz ref BW) is ~-17.5 dB (vs FT8's
# -21 dB — FT4 trades sensitivity for the shorter 7.5 s slot / less FEC
# interleaving). Grid extends a few dB past it on both sides so the 50%
# crossing is observed rather than censored at the grid edge (see the
# FST4 #146 lesson in docs/notes/FST4_BENCHMARK.md section 3).
SNRS="-5 -10 -13 -14 -15 -16 -17 -18 -19 -20 -21 -22 -23"

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
    local dest="$OUT_DIR/ft4_${chan}_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  FT4  %-14s  SNR=%4d dB  generating %d files ...\n" \
    "$chan" "$snr" "$TRIALS"

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' RETURN

  (
    cd "$tmpd"
    "$FT4SIM" "$MSG" "$F0" "$DT" "$fdop" "$del" "$TRIALS" "$snr" \
      >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(printf '000000_%06d.wav' "$T")"
      dest="$OUT_DIR/ft4_${chan}_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell snr_tag
export FT4SIM OUT_DIR TRIALS MSG F0 DT

# Build the full list of (chan fdop del snr) tuples, then fan out.
CELLS=()
for CHAN_SPEC in "${CHANNELS[@]}"; do
  read -r CHAN FDOP DEL <<< "$CHAN_SPEC"
  for SNR in $SNRS; do
    CELLS+=("$CHAN $FDOP $DEL $SNR")
  done
done

echo "Generating FT4 sweep corpus: ${#CELLS[@]} cells, TRIALS=$TRIALS, JOBS=$JOBS"
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
wait

echo ""
echo "Done. Assets: $OUT_DIR"
echo "  $(ls "$OUT_DIR" | wc -l) files total"
echo ""
echo "Run the sweep test with:"
echo "  MFSK_FT4_SWEEP_DIR=$OUT_DIR \\"
echo "    cargo test --test ft4_sweep --release --features ft4,fft-rustfft,parallel,uvpacket \\"
echo "    -- --ignored --nocapture"
