#!/usr/bin/env bash
# Generate jt65sim WAV files for a JT65A AWGN SNR sweep.
#
# Usage:
#   scripts/gen_jt65_sweep_wavs.sh [jt65sim-path] [out-dir]
#
# File naming:  jt65_awgn_<snr>_<trial>.wav
#   snr:      m05 = -5 dB, p05 = +5 dB (real, applied SNR — see the
#             sentinel note below)
#   trial:    01..TRIALS
#
# jt65sim has no standalone from-source build script in this repo (unlike
# ft8sim/ft4sim/fst4sim — jt65sim links the full wsjt_fort/wsjt_cxx
# libraries per WSJT-X's CMakeLists.txt, not a small standalone subset).
# Build it via the full WSJT-X CMake build instead:
#   cmake --build ~/wsjtx-build --target jt65sim -j"$(nproc)"
# then point this script at the resulting binary (default guess below).
#
# ## jt65sim SNR=0 sentinel — do not pass "0" as a grid value
#
# `jt65sim.f90` special-cases `snrdb == 0.0`: it does NOT mean "0 dB",
# it silently substitutes a built-in weak-signal default (~-19 to -21 dB
# depending on sub-mode). Passing a literal 0 to `-s` produces a signal
# nowhere near 0 dB even though jt65sim's own console readout still
# prints "0.0" in that case (it's echoing the requested value, not the
# applied one) — this cost a full debugging detour before being traced
# to `jt65sim.f90:167-170`. This script's grid must never include the
# literal integer/float 0; use a small non-zero value (e.g. 0.01) if a
# near-0 dB point is wanted.
#
# Run existing files are skipped (safe to re-run after widening the grid).
# Jobs run in parallel (JOBS env var, default: nproc).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

JT65SIM="${1:-$HOME/wsjtx-build/jt65sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/jt65_sweep}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"

if [[ ! -x "$JT65SIM" ]]; then
  echo "error: jt65sim not found at $JT65SIM" >&2
  echo "Build it with: cmake --build ~/wsjtx-build --target jt65sim -j\$(nproc)" >&2
  echo "(requires a configured WSJT-X CMake build tree; see CLAUDE.md)" >&2
  exit 1
fi

mkdir -p "$OUT_DIR"

MSG="CQ JL1NIE PM95"
F0=1500
TRIALS=20

# JT65A hard-decision (no kvasd) sensitivity floor is roughly -15 to -20 dB
# in this reference bandwidth (see docs/notes/ — grid centres on the steep
# part of that curve so the 50% crossing is observed, not censored at the
# grid edge, mirroring the FST4 #146 lesson).
SNRS="10 5 0.01 -5 -10 -12 -14 -15 -16 -17 -18 -19 -20 -22 -25"

snr_tag() {
  local snr=$1
  # snr may be fractional (e.g. 0.01); format as an integer-rounded tag.
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
    local dest="$OUT_DIR/jt65_awgn_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  JT65  SNR=%6s dB  generating %d files ...\n" "$snr" "$TRIALS"

  # jt65sim's own arg parser requires negative values escaped with a
  # literal leading backslash (its help text: "Use \ (\\ on *nix shells)
  # to escape -ve arguments") — a bare "-20" is rejected as "option -s
  # requires an argument".
  local snr_arg="$snr"
  if [[ "$snr" == -* ]]; then
    snr_arg="\\$snr"
  fi

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' EXIT

  (
    cd "$tmpd"
    "$JT65SIM" -M "$MSG" -n 1 -s "$snr_arg" -f "$TRIALS" -F "$F0" >/dev/null
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(printf '000000_%04d.wav' "$T")"
      dest="$OUT_DIR/jt65_awgn_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell snr_tag
export JT65SIM OUT_DIR TRIALS MSG F0

echo "Generating JT65 AWGN sweep corpus: $(echo $SNRS | wc -w) SNR points, TRIALS=$TRIALS, JOBS=$JOBS"
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
echo "  cargo test --test jt65_sweep --release --features jt65,fft-rustfft \\"
echo "    -- --ignored --nocapture"
