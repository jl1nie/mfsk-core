#!/usr/bin/env bash
# Generate wsprsim WAV files for a WSPR AWGN SNR sweep.
#
# Usage:
#   scripts/gen_wspr_sweep_wavs.sh [wsprsim-path] [out-dir]
#
# Defaults:
#   wsprsim-path  target/wsprsim/wsprsim
#   out-dir       embedded-poc/assets/wspr_sweep/
#
# Run scripts/build_wsprsim.sh first if the binary doesn't exist.
#
# File naming:  wspr_awgn_<snr_tag>_<trial>.wav
#   snr_tag:  m28 = -28 dB, p05 = +5 dB
#   trial:    01..TRIALS
#
# wsprsim's CLI (the *.wav-capable Fortran build; see
# build_wsprsim.sh) is positional, no getopt flags:
#   wsprsim "message" f0 DT fspread delay nwav nfiles snr
# fspread=0 delay=0 -> clean AWGN, no Watterson fading. f0=0 puts the
# carrier at the 1500 Hz center all other *sim tools in this repo use.
# nwav=1 selects the *.wav output branch (matching every other *sim
# tool's format; the alternative *.c2 branch is complex-baseband IQ
# that mfsk-core's WAV decode path can't consume).
#
# Existing files are skipped (safe to re-run after widening the grid).
# Jobs run in parallel (JOBS env var, default: nproc).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WSPRSIM="${1:-$REPO_ROOT/target/wsprsim/wsprsim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/wspr_sweep}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"

# run_cell cd's into a tmpdir before invoking wsprsim, so both paths
# must be absolute regardless of how the caller passed them in.
[[ -x "$WSPRSIM" ]] && WSPRSIM="$(cd "$(dirname "$WSPRSIM")" && pwd)/$(basename "$WSPRSIM")"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

if [[ ! -x "$WSPRSIM" ]]; then
  echo "error: wsprsim not found at $WSPRSIM" >&2
  echo "Run scripts/build_wsprsim.sh first." >&2
  exit 1
fi

# WSPR message: "call grid dBm" -- no CQ form, unlike FT8/JT65/JT9/Q65.
MSG="JL1NIE PM95 37"
TRIALS=20

# WSJT-X's published WSPR sensitivity floor is around -28 to -33 dB in
# a 2500 Hz reference bandwidth (K1JT's WSPR papers cite decodes down
# to about -31 dB S/N with strong FEC gain); grid centres on that
# range so the 50% crossing is observed, not censored at the grid
# edge (mirroring the FST4 #146 / JT65 grid-centering lesson).
SNRS="0 -10 -15 -20 -24 -26 -27 -28 -29 -30 -31 -32 -34"

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
    local dest="$OUT_DIR/wspr_awgn_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  WSPR  SNR=%6s dB  generating %d files ...\n" "$snr" "$TRIALS"

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' EXIT

  (
    cd "$tmpd"
    "$WSPRSIM" "$MSG" 0 0.0 0.0 0.0 1 "$TRIALS" "$snr" >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(printf '000000_%04d.wav' "$T")"
      dest="$OUT_DIR/wspr_awgn_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell snr_tag
export WSPRSIM OUT_DIR TRIALS MSG

echo "Generating WSPR AWGN sweep corpus: $(echo $SNRS | wc -w) SNR points, TRIALS=$TRIALS, JOBS=$JOBS"
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
echo "  cargo test --test wspr_sweep --release --features wspr,fft-rustfft \\"
echo "    -- --ignored --nocapture"
