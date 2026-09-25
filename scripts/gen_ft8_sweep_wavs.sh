#!/usr/bin/env bash
# Generate ft8sim WAV files for SNR sweep + fading benchmark.
#
# Usage:
#   scripts/gen_ft8_sweep_wavs.sh [ft8sim-path] [out-dir]
#
# File naming:  ft8_<channel>_<snr>_<trial>.wav
#   channel:  awgn | ccir_good | ccir_moderate | ccir_poor        (FT8_CHANNEL_SET=ccir, the default)
#             itu_lq itu_lm itu_ld itu_mq itu_mm itu_md itu_hq itu_hm itu_hd  (FT8_CHANNEL_SET=itu)
#   snr:      m05 = -5 dB, m24 = -24 dB, etc.
#   trial:    01..TRIALS
#
# TWO CHANNEL SETS, DELIBERATELY NOT INTERCHANGEABLE. The `ccir_*` set is what
# `sweep-baseline.json` and every "vs jt9" number in docs/notes/ were measured
# on, and it needs an ft8sim built from the WSJT-X tree the corpus was made
# with (`2b9d654`). In February 2024 WSJT-X corrected the Watterson simulator
# (f21f37ad0 "Correct the definition of fspread", 7ce6e29a7 "same spreading
# function as ITU-R F.1487"): for the same fspread argument a newer tree gives a
# WIDER Doppler spectrum. So `ccir_*` regenerated from a b4f9a43-or-later tree is
# a different, harsher channel under the same name (AWGN files still match byte
# for byte, which hides it), and the old `ccir_*` are milder than the ITU
# channels their numbers suggest. The `itu_*` set is generated with the
# corrected simulator and named differently on purpose; it needs ft8sim from
# `v3.0.0` or later (the ITU channel codes were added in 3.x):
#   itu_lq/lm/ld = low latitude quiet/moderate/disturbed  (0.5 Hz/0.5 ms, 1.5/2.0, 10/6)
#   itu_mq/mm/md = mid latitude                            (0.1/0.5, 0.5/1.0, 1.0/2.0)
#   itu_hq/hm/hd = high latitude                           (0.5/1.0, 10/3, 30/7)
# The default output directory follows the set (`ft8_sweep`, `ft8_itu_sweep`).
#
# Run build_ft8sim.sh first if the binary doesn't exist.
# Existing files are skipped (safe to re-run after widening the grid).
# Jobs run in parallel (JOBS env var, default: nproc).
# Trials per cell: TRIALS env var, default 20. Raising it does not
# extend an existing corpus: a cell is skipped only when every trial is
# already there, so an incomplete one is regenerated whole (the
# simulator is invoked once for all TRIALS, drawing the whole cell in
# one sequence) so every trial index in it gets a different signal than
# before. Regenerating a cell at the *same* TRIALS is reproducible: this
# generator is seeded from MFSK_SIM_SEED, default 1 -- see
# scripts/sim_sgran_stub.c. Generate into a separate out-dir (2nd
# positional arg) rather than in place.
#
# Mirrors scripts/gen_ft4_sweep_wavs.sh (see docs/notes/FT8_BENCHMARK.md).
# FT8 has no sub-modes, so there's one SNR grid instead of one per period.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

FT8_CHANNEL_SET="${FT8_CHANNEL_SET:-ccir}"
case "$FT8_CHANNEL_SET" in
  ccir) DEFAULT_DIR_NAME=ft8_sweep ;;
  itu)  DEFAULT_DIR_NAME=ft8_itu_sweep ;;
  *) echo "error: FT8_CHANNEL_SET=$FT8_CHANNEL_SET: expected ccir or itu" >&2; exit 1 ;;
esac

FT8SIM="${1:-$REPO_ROOT/target/ft8sim/ft8sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/$DEFAULT_DIR_NAME}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"

if [[ ! -x "$FT8SIM" ]]; then
  echo "error: ft8sim not found at $FT8SIM" >&2
  echo "Run scripts/build_ft8sim.sh first." >&2
  exit 1
fi

# run_cell cd's into a tmpdir before invoking the simulator, so both
# paths must be absolute regardless of how the caller passed them in.
[[ -x "$FT8SIM" ]] && FT8SIM="$(cd "$(dirname "$FT8SIM")" && pwd)/$(basename "$FT8SIM")"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

MSG="CQ JL1NIE PM95"
F0=1500
DT=0.0
TRIALS="${TRIALS:-20}"

# Published WSJT-X AWGN threshold (2500 Hz ref BW) is ~-20 to -21 dB (vs
# FT4's -17.5 dB — FT8's longer 15 s slot / deeper FEC interleaving buys
# back sensitivity FT4 trades away for throughput). Grid extends a few dB
# past it on both sides so the 50% crossing is observed rather than
# censored at the grid edge (see the FST4 #146 lesson in
# docs/notes/FST4_BENCHMARK.md section 3).
SNRS="-5 -10 -15 -17 -18 -19 -20 -21 -22 -23 -24 -25 -26"

if [[ "$FT8_CHANNEL_SET" == itu ]]; then
  # Field 2 is the ITU code ft8sim takes in place of the Doppler spread; field
  # 3 is a placeholder (ft8sim still requires the argument, and ignores it).
  # SNR grid: the quiet and moderate channels cross 50 % near -20 dB, but the
  # disturbed ones (itu_ld, itu_hm, itu_hd: 10-30 Hz of Doppler spread, wider
  # than FT8's 6.25 Hz tone spacing) are still at 0-50 % at -5 dB, so the grid
  # reaches +10 dB and is denser through -8..-16.
  SNRS="10 5 0 -5 -8 -10 -12 -14 -16 -18 -19 -20 -21 -22 -23 -24 -26"
  CHANNELS=(
    "itu_lq LQ 1.0"
    "itu_lm LM 1.0"
    "itu_ld LD 1.0"
    "itu_mq MQ 1.0"
    "itu_mm MM 1.0"
    "itu_md MD 1.0"
    "itu_hq HQ 1.0"
    "itu_hm HM 1.0"
    "itu_hd HD 1.0"
  )
  # An ft8sim without the ITU codes fails on the code argument at run time,
  # per cell, deep inside the job pool. Say so up front instead.
  probe="$(mktemp -d)"
  if ! ( cd "$probe" && "$FT8SIM" "$MSG" "$F0" "$DT" MM 1.0 1 -20 >/dev/null 2>&1 ); then
    rm -rf "$probe"
    echo "error: $FT8SIM does not accept ITU channel codes (MM ...)." >&2
    echo "  FT8_CHANNEL_SET=itu needs an ft8sim built from WSJT-X v3.0.0 or later:" >&2
    echo "  scripts/build_ft8sim.sh <wsjtx-3.x-tree> <out-dir>" >&2
    exit 1
  fi
  rm -rf "$probe"
else
  CHANNELS=(
    "awgn          0.0  0.0"
    "ccir_good     0.1  0.5"
    "ccir_moderate 0.5  1.0"
    "ccir_poor     1.0  2.0"
  )
fi

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
