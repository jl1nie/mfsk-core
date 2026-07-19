#!/usr/bin/env bash
# Generate q65sim WAV files for an AWGN SNR sweep across all six
# sub-mode ZSTs this crate actually wires (Q65a30, Q65a60, Q65b60,
# Q65c60, Q65d60, Q65e60). WSJT-X's Q65 also supports 15/120/300 s
# periods and other (period, sub-mode) combinations, but this crate
# doesn't implement those, so this sweep intentionally covers only
# the six that exist -- mirroring the scoping call made for the
# JT65/JT9 sweeps (validate what's shipped, not a hypothetical
# superset).
#
# Usage:
#   scripts/gen_q65_sweep_wavs.sh [q65sim-path] [out-dir]
#
# q65sim has a real CMakeLists.txt target (unlike jt9sim), so build it
# via the full WSJT-X CMake build:
#   cmake --build ~/wsjtx-build --target q65sim -j"$(nproc)"
#
# File naming:  q65_<submode>_awgn_<snr_tag>_<trial>.wav
#   submode:  a30 | a60 | b60 | c60 | d60 | e60
#   snr_tag:  m27 = -27 dB, p05 = +5 dB
#   trial:    01..TRIALS
#
# Per-submode SNR grids are centred on q65params.f90's analytical
# threshold formula (`-27 + 10*log10(7200/nsps)`, which depends only
# on T/R period, not sub-mode letter -- under pure AWGN the wider tone
# spacing of B/C/D/E doesn't change matched-filter sensitivity, only
# Doppler/fading tolerance): -24 dB for the 30 s period (a30), -27 dB
# for all five 60 s sub-modes (a60/b60/c60/d60/e60).
#
# Existing files are skipped (safe to re-run after widening the grid).
# Jobs run in parallel (JOBS env var, default: nproc).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

Q65SIM="${1:-$HOME/wsjtx-build/q65sim}"
OUT_DIR="${2:-$REPO_ROOT/embedded-poc/assets/q65_sweep}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 1)}"

[[ -x "$Q65SIM" ]] && Q65SIM="$(cd "$(dirname "$Q65SIM")" && pwd)/$(basename "$Q65SIM")"
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

if [[ ! -x "$Q65SIM" ]]; then
  echo "error: q65sim not found at $Q65SIM" >&2
  echo "Build it with: cmake --build ~/wsjtx-build --target q65sim -j\$(nproc)" >&2
  exit 1
fi

MSG="CQ JL1NIE PM95"
F0=1500
TRIALS=15

# submode:period:letter:threshold_dB
CONFIGS=(
  "a30:30:A:-24"
  "a60:60:A:-27"
  "b60:60:B:-27"
  "c60:60:C:-27"
  "d60:60:D:-27"
  "e60:60:E:-27"
)

# Offsets (dB) relative to each config's analytical threshold.
OFFSETS="9 6 3 2 1 0 -1 -2 -3 -5 -7"

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
  local submode=$1 period=$2 letter=$3 snr=$4
  local tag; tag="$(snr_tag "$snr")"

  local missing=0
  for T in $(seq 1 "$TRIALS"); do
    local dest="$OUT_DIR/q65_${submode}_awgn_${tag}_$(printf '%02d' "$T").wav"
    [[ -f "$dest" ]] || (( missing++ )) || true
  done
  if (( missing == 0 )); then
    return 0
  fi

  printf "  Q65-%-3s SNR=%4s dB  generating %d files ...\n" "$submode" "$snr" "$TRIALS"

  local tmpd; tmpd="$(mktemp -d)"
  trap 'rm -rf "$tmpd"' EXIT

  (
    cd "$tmpd"
    # q65sim "msg" A-E freq fDop DT f1 Stp TRp Nsig Nfile SNR
    # fDop=0 DT=0 f1=0 Stp=0 -> clean AWGN, no Doppler/drift/tracking.
    "$Q65SIM" "$MSG" "$letter" "$F0" 0.0 0.0 0.0 0 "$period" 1 "$TRIALS" "$snr" >/dev/null
    # q65sim's filename index increments by (period/30) per file, not
    # by 1 (q65sim.f90: istart=(ifile*period*2)-(period*2), fname
    # index = istart/60 = (ifile-1)*period/30) -- confirmed by direct
    # inspection since this isn't documented in the -h text.
    local step=$(( period / 30 ))
    for T in $(seq 1 "$TRIALS"); do
      local src dest
      src="$(printf '000000_%04d.wav' "$(( (T - 1) * step ))")"
      dest="$OUT_DIR/q65_${submode}_awgn_${tag}_$(printf '%02d' "$T").wav"
      [[ -f "$src" ]] && mv "$src" "$dest"
    done
  )
}

export -f run_cell snr_tag
export Q65SIM OUT_DIR TRIALS MSG F0

TOTAL_CELLS=$(( ${#CONFIGS[@]} * $(echo $OFFSETS | wc -w) ))
echo "Generating Q65 AWGN sweep corpus: ${#CONFIGS[@]} sub-modes x $(echo $OFFSETS | wc -w) SNR points = $TOTAL_CELLS cells, TRIALS=$TRIALS, JOBS=$JOBS"
echo "Output: $OUT_DIR"
echo ""

active=0
pids=()
for CONFIG in "${CONFIGS[@]}"; do
  IFS=':' read -r submode period letter threshold <<< "$CONFIG"
  for OFFSET in $OFFSETS; do
    snr=$(( threshold + OFFSET ))
    run_cell "$submode" "$period" "$letter" "$snr" &
    pids+=($!)
    (( active++ )) || true
    if (( active >= JOBS )); then
      wait "${pids[0]}"
      pids=("${pids[@]:1}")
      (( active-- )) || true
    fi
  done
done
for pid in "${pids[@]}"; do
  wait "$pid"
done

echo ""
echo "Done. Assets: $OUT_DIR"
echo "  $(ls "$OUT_DIR" | wc -l) files total"
echo ""
echo "Run the sweep test with:"
echo "  cargo test --test q65_sim_sweep --release --features q65,fft-rustfft,uvpacket \\"
echo "    -- --ignored --nocapture"
