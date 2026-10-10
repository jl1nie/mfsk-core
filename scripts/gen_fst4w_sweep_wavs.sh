#!/usr/bin/env bash
# Generate fst4sim WAV files for the FST4W SNR sweep + fading benchmark (#649).
#
# FST4W is fst4sim's last argument, W=T: the encoder is asked for the 50-bit
# WSPR-type message (`genfst4`, iwspr=1). Periods 120 and 300 s only: a 900 or
# 1800 s corpus would be tens of GB (a 900 s file is 21.6 MB).
#
# Usage:
#   scripts/gen_fst4w_sweep_wavs.sh [fst4sim-path] [out-dir]
#
# File naming:  fst4w_<nsec>_<channel>_<snr>_<trial>.wav
#   channel:  awgn | ccir_good | ccir_moderate | ccir_poor
#   snr:      m05 = -5 dB, m24 = -24 dB, etc.
#   trial:    01..TRIALS
#
# Run build_fst4sim.sh first if the binary doesn't exist.
# Existing files are skipped (safe to re-run after adding modes/conditions).
# Jobs run in parallel (JOBS env var, default: nproc).
# Trials per cell: TRIALS env var, default 20. Raising it regenerates
# any cell that is not already complete — the simulator is invoked once
# for all TRIALS and the files already present are overwritten.
#
# That overwrite is safe: fst4sim is deterministic and its realisations
# do not depend on the requested trial count, so trials 1..N come back
# byte-identical and every existing index still names the same signal.
# Verified 2026-09-08 by regenerating four cells at TRIALS=100 and
# md5-comparing the first 20 against the stored corpus (80/80 identical)
# — see docs/notes/FST4_BENCHMARK.md, "Trap 1". Raising TRIALS is
# therefore how you extend a corpus; the header used to say the
# opposite.
#
# Across *machines* the guarantee is only as good as the fst4sim build
# being the same one, which nothing here checks.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FST4SIM="${1:-$REPO_ROOT/target/fst4sim/fst4sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/fst4w_sweep}"
JOBS="${JOBS:-$(nproc)}"

if [[ ! -x "$FST4SIM" ]]; then
  echo "error: fst4sim not found at $FST4SIM" >&2
  echo "Run scripts/build_fst4sim.sh first." >&2
  exit 1
fi

# run_cell cd's into a tmpdir before invoking the simulator, so both
# paths must be absolute regardless of how the caller passed them in.
[[ -x "$FST4SIM" ]] && FST4SIM="$(cd "$(dirname "$FST4SIM")" && pwd)/$(basename "$FST4SIM")"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

MSG="JL1NIE PM95 37"
F0=1500
DT=0.0
TRIALS="${TRIALS:-20}"

declare -A MODE_SNRS
MODE_SNRS[120]="-5 -15 -24 -27 -29 -30 -31 -32 -33 -34 -35"
MODE_SNRS[300]="-5 -20 -28 -30 -32 -33 -34 -35 -36 -37 -38"

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
    local dest="$OUT_DIR/fst4w_${nsec}_${chan}_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  FST4W-%3d %-14s  SNR=%4d dB  generating %d files ...\n" \
    "$nsec" "$chan" "$snr" "$TRIALS"

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' RETURN

  (
    cd "$tmpd"
    "$FST4SIM" "$MSG" "$nsec" "$F0" "$DT" "$fdop" "$del" "$TRIALS" "$snr" T \
      >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(fst4sim_outname "$nsec" "$T")"
      dest="$OUT_DIR/fst4w_${nsec}_${chan}_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell fst4sim_outname snr_tag
export FST4SIM OUT_DIR TRIALS MSG F0 DT

# Build the full list of (nsec chan fdop del snr) tuples, then fan out.
CELLS=()
for NSEC in 120 300; do
  for CHAN_SPEC in "${CHANNELS[@]}"; do
    read -r CHAN FDOP DEL <<< "$CHAN_SPEC"
    for SNR in ${MODE_SNRS[$NSEC]}; do
      CELLS+=("$NSEC $CHAN $FDOP $DEL $SNR")
    done
  done
done

# Refuse an unseeded simulator, and an out-dir whose WAVs came from another
# seed or binary: scripts/lib/corpus-stamp.sh.
source "$SCRIPT_DIR/lib/corpus-stamp.sh"
corpus_preflight "$OUT_DIR" "$FST4SIM" deterministic

echo "Generating FST4W sweep corpus: ${#CELLS[@]} cells, TRIALS=$TRIALS, JOBS=$JOBS"
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

corpus_stamp "$OUT_DIR" "$FST4SIM" deterministic "$0"
echo ""
echo "Done. Assets: $OUT_DIR"
echo "  $(ls "$OUT_DIR" | wc -l) files total"
echo ""
echo "Run the sweep test with:"
echo "  MFSK_FST4W_SWEEP_DIR=$OUT_DIR \\"
echo "    cargo test --test fst4w_sweep --release --features fst4w,fft-rustfft,parallel \\"
echo "    -- --ignored --nocapture"
