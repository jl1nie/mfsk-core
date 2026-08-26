#!/usr/bin/env bash
# Generate fst4sim WAV files for SNR sweep + fading benchmark.
#
# Usage:
#   scripts/gen_fst4_sweep_wavs.sh [fst4sim-path] [out-dir]
#
# File naming:  fst4_<nsec>_<channel>_<snr>_<trial>.wav
#   channel:  awgn | ccir_good | ccir_moderate | ccir_poor
#   snr:      m05 = -5 dB, m24 = -24 dB, etc.
#   trial:    01..TRIALS
#
# Run build_fst4sim.sh first if the binary doesn't exist.
# Existing files are skipped (safe to re-run after adding modes/conditions).
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

FST4SIM="${1:-$REPO_ROOT/target/fst4sim/fst4sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/fst4_sweep}"
JOBS="${JOBS:-$(nproc)}"

if [[ ! -x "$FST4SIM" ]]; then
  echo "error: fst4sim not found at $FST4SIM" >&2
  echo "Run scripts/build_fst4sim.sh first." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

MSG="CQ JL1NIE PM95"
F0=1500
DT=0.0
TRIALS="${TRIALS:-20}"

declare -A MODE_SNRS
MODE_SNRS[15]="-5 -10 -14 -15 -16 -17 -18 -19 -20 -21 -22 -23"
MODE_SNRS[30]="-5 -10 -15 -18 -19 -20 -21 -22 -23 -24 -25 -26"
MODE_SNRS[60]="-5 -10 -15 -20 -22 -23 -24 -25 -26 -27 -28 -29 -30"
MODE_SNRS[120]="-5 -10 -15 -20 -24 -26 -27 -28 -29 -30 -31 -32 -33"
MODE_SNRS[300]="-5 -10 -15 -20 -24 -27 -30 -31 -32 -33 -34 -35 -36 -37"

CHANNELS=(
  "awgn          0.0  0.0"
  "ccir_good     0.1  0.5"
  "ccir_moderate 0.5  1.0"
  "ccir_poor     1.0  2.0"
)

fst4sim_outname() {
  local nsec=$1 trial=$2
  if (( nsec <= 30 )); then
    printf "000000_%06d.wav" "$trial"
  else
    printf "000000_%04d.wav" "$trial"
  fi
}

snr_tag() {
  local snr=$1
  if (( snr < 0 )); then
    printf "m%02d" "$(( -snr ))"
  else
    printf "p%02d" "$snr"
  fi
}

# One worker function per (nsec, chan, snr) cell — each gets its own tmpdir.
run_cell() {
  local nsec=$1 chan=$2 fdop=$3 del=$4 snr=$5

  local tag; tag="$(snr_tag "$snr")"

  # Skip if all trials already exist.
  local missing=0
  for T in $(seq 1 "$TRIALS"); do
    local dest="$OUT_DIR/fst4_${nsec}_${chan}_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  FST4-%3d  %-14s  SNR=%4d dB  generating %d files ...\n" \
    "$nsec" "$chan" "$snr" "$TRIALS"

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' RETURN

  (
    cd "$tmpd"
    "$FST4SIM" "$MSG" "$nsec" "$F0" "$DT" "$fdop" "$del" "$TRIALS" "$snr" F \
      >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(fst4sim_outname "$nsec" "$T")"
      dest="$OUT_DIR/fst4_${nsec}_${chan}_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell fst4sim_outname snr_tag
export FST4SIM OUT_DIR TRIALS MSG F0 DT

# Build the full list of (nsec chan fdop del snr) tuples, then fan out.
CELLS=()
for NSEC in 15 30 60 120 300; do
  for CHAN_SPEC in "${CHANNELS[@]}"; do
    read -r CHAN FDOP DEL <<< "$CHAN_SPEC"
    for SNR in ${MODE_SNRS[$NSEC]}; do
      CELLS+=("$NSEC $CHAN $FDOP $DEL $SNR")
    done
  done
done

echo "Generating FST4 sweep corpus: ${#CELLS[@]} cells, TRIALS=$TRIALS, JOBS=$JOBS"
echo "Output: $OUT_DIR"
echo ""

# Run cells in parallel using a simple job-pool (no GNU parallel needed).
active=0
pids=()
for CELL in "${CELLS[@]}"; do
  read -r nsec chan fdop del snr <<< "$CELL"
  run_cell "$nsec" "$chan" "$fdop" "$del" "$snr" &
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
echo "  MFSK_FST4_SWEEP_DIR=$OUT_DIR \\"
echo "    cargo test --test fst4_sweep --release --features fst4,fft-rustfft,parallel \\"
echo "    -- --ignored --nocapture"
