#!/usr/bin/env bash
# Generate fst4sim WAV files for SNR sweep + fading benchmark.
#
# Usage:
#   scripts/gen_fst4_sweep_wavs.sh [fst4sim-path] [out-dir]
#
# File naming:  fst4_<nsec>_<channel>_<snr>_<trial>.wav
#   channel:  awgn | hf_quiet | hf_typical | hf_disturbed
#   snr:      m05 = -5 dB, m24 = -24 dB, etc.
#   trial:    01..TRIALS
#
# Run build_fst4sim.sh first if the binary doesn't exist.
# Existing files are skipped (safe to re-run after adding modes/conditions).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FST4SIM="${1:-$REPO_ROOT/target/fst4sim/fst4sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/fst4_sweep}"

if [[ ! -x "$FST4SIM" ]]; then
  echo "error: fst4sim not found at $FST4SIM" >&2
  echo "Run scripts/build_fst4sim.sh first." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

MSG="CQ JL1NIE PM95"
F0=1500
DT=0.0
TRIALS=10   # files per (mode, channel, snr) — raise to 20 for thorough sweep

# SNR levels per mode (dB, all negative — add more as needed)
declare -A MODE_SNRS
MODE_SNRS[15]="-5 -10 -15 -17 -20"
MODE_SNRS[30]="-5 -10 -15 -20 -22"
MODE_SNRS[60]="-5 -10 -15 -20 -24"
MODE_SNRS[120]="-5 -10 -15 -20 -24 -26"
MODE_SNRS[300]="-5 -10 -15 -20 -24 -27 -30"

# Channel conditions: name fdop del
# fdop/del values follow ITU-R (CCIR) Watterson HF channel models.
CHANNELS=(
  "awgn          0.0  0.0"
  "ccir_good     0.1  0.5"
  "ccir_moderate 0.5  1.0"
  "ccir_poor     1.0  2.0"
)

# WAV filename written by fst4sim (depends on nsec)
fst4sim_outname() {
  local nsec=$1 trial=$2
  if (( nsec <= 30 )); then
    printf "000000_%06d.wav" "$trial"
  else
    printf "000000_%04d.wav" "$trial"
  fi
}

# SNR → tag: -5 → m05, -24 → m24, 5 → p05
snr_tag() {
  local snr=$1
  if (( snr < 0 )); then
    printf "m%02d" "$(( -snr ))"
  else
    printf "p%02d" "$snr"
  fi
}

TMPDIR_WORK="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_WORK"' EXIT

total=0; skipped=0; generated=0

for NSEC in 15 30 60 120 300; do
  SNRS="${MODE_SNRS[$NSEC]}"
  for CHAN_SPEC in "${CHANNELS[@]}"; do
    read -r CHAN FDOP DEL <<< "$CHAN_SPEC"
    for SNR in $SNRS; do
      TAG="$(snr_tag "$SNR")"

      # Generate all TRIALS files if any are missing
      missing=0
      for T in $(seq 1 "$TRIALS"); do
        DEST="$OUT_DIR/fst4_${NSEC}_${CHAN}_${TAG}_$(printf '%02d' "$T").wav"
        [[ -f "$DEST" ]] || (( missing++ )) || true
      done

      (( total += TRIALS ))
      if (( missing == 0 )); then
        (( skipped += TRIALS )) || true
        continue
      fi

      printf "  FST4-%3d  %-14s  SNR=%4d dB  generating %d files ...\n" \
        "$NSEC" "$CHAN" "$SNR" "$TRIALS"

      (
        cd "$TMPDIR_WORK"
        rm -f 000000_*.wav
        "$FST4SIM" "$MSG" "$NSEC" "$F0" "$DT" "$FDOP" "$DEL" "$TRIALS" "$SNR" F \
          >/dev/null
        for T in $(seq 1 "$TRIALS"); do
          SRC="$(fst4sim_outname "$NSEC" "$T")"
          DEST="$OUT_DIR/fst4_${NSEC}_${CHAN}_${TAG}_$(printf '%02d' "$T").wav"
          [[ -f "$SRC" ]] && mv "$SRC" "$DEST"
        done
      )
      (( generated += TRIALS )) || true
    done
  done
done

echo ""
echo "Done. Total=$total  Generated=$generated  Skipped(existing)=$skipped"
echo "Assets: $OUT_DIR"
echo ""
echo "Run the sweep test with:"
echo "  MFSK_FST4_SWEEP_DIR=$OUT_DIR \\"
echo "    cargo test --test fst4_sweep --features fst4,fft-rustfft \\"
echo "    -- --ignored --nocapture"
