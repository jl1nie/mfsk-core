#!/usr/bin/env bash
# Generate fst4sim synthetic WAV files for each FST4 sub-mode.
#
# Usage:
#   scripts/gen_fst4_sim_wavs.sh [fst4sim-path] [out-dir]
#
# Defaults:
#   fst4sim-path  target/fst4sim/fst4sim
#   out-dir       embedded-poc/assets/fst4_sim/
#
# Run build_fst4sim.sh first if the binary doesn't exist.
#
# Each sub-mode gets one clean AWGN file (fdop=0 del=0) at SNR=-5 dB
# at f0=1500 Hz. Filename pattern written by fst4sim:
#   nsec<=30  ->  000000_000001.wav
#   nsec>=60  ->  000000_0001.wav
#
# The Rust integration test reads these from MFSK_FST4_SIM_ASSETS_DIR
# (defaults to embedded-poc/assets/fst4_sim/).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FST4SIM="${1:-$REPO_ROOT/target/fst4sim/fst4sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/fst4_sim}"

if [[ ! -x "$FST4SIM" ]]; then
  echo "error: fst4sim not found at $FST4SIM" >&2
  echo "Run scripts/build_fst4sim.sh first." >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

# Message sent by fst4sim in all cases.
MSG="CQ JL1NIE PM95"
F0=1500          # centre frequency Hz
DT=0.0           # zero time offset
FDOP=0.0         # no Watterson fading
DEL=0.0          # no multipath delay
NFILES=1         # one file per mode
SNR=-5           # dB, well above FST4 sensitivity floor

declare -A MODES=(
  [15]="000000_000001.wav"
  [30]="000000_000001.wav"
  [60]="000000_0001.wav"
  [120]="000000_0001.wav"
  [300]="000000_0001.wav"
)

TMPDIR_WORK="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_WORK"' EXIT

for NSEC in 15 30 60 120 300; do
  OUTFILE="${MODES[$NSEC]}"
  DEST="$OUT_DIR/fst4_${NSEC}_sim.wav"

  echo -n "  FST4-${NSEC}: generating ..."
  (
    cd "$TMPDIR_WORK"
    "$FST4SIM" "$MSG" "$NSEC" "$F0" "$DT" "$FDOP" "$DEL" "$NFILES" "$SNR" F \
      >/dev/null
    mv "$OUTFILE" "$DEST"
  )
  echo " -> $(basename "$DEST")  ($(du -h "$DEST" | cut -f1))"
done

echo ""
echo "Done. Assets in: $OUT_DIR"
echo ""
echo "Run tests with:"
echo "  MFSK_FST4_SIM_ASSETS_DIR=$OUT_DIR \\"
echo "    cargo test --test fst4_sim_roundtrip --features fst4,fft-rustfft -- --nocapture"
