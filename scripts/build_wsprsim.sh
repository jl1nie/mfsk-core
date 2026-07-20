#!/usr/bin/env bash
# Build wsprsim (WAV-output flavor) from WSJT-X Fortran sources.
#
# Usage:
#   scripts/build_wsprsim.sh [WSJT-X-dir] [out-dir]
#
# Defaults:
#   WSJT-X-dir  ../WSJT-X  (sibling of this repo)
#   out-dir     target/wsprsim/
#
# Requires: gfortran, gcc
#   Ubuntu:  sudo apt-get install gfortran
#
# WSJT-X ships *two* WSPR simulators. CMakeLists.txt only wires the C
# one (lib/wsprd/wsprsim.c via `add_executable(wsprsim ...)`), which
# writes *.c2 complex-baseband files (375 Sa/s IQ) meant to feed
# `wsprd` -- not something mfsk-core's WAV-based decode path can
# consume directly. The *other* one, lib/wsprd/wsprsimf.f90 (Fortran,
# no CMake target at all -- same "orphaned" situation as jt9sim), has
# a WAV-output branch (`nwav=1`) that writes real 12 kHz PCM16 audio
# via wspr_wav.f90, matching every other *sim tool's output format.
# This script builds that one, picking out its actual dependency
# closure the same way build_jt9sim.sh does for jt9sim.
#
# One wrinkle: wsprsimf.f90 calls `watterson(...)` on its *.c2 (nwav=0)
# branch, which this build never exercises -- but Fortran external
# calls are resolved at link time regardless of which runtime branch
# executes, so the symbol must exist. The real watterson.f90 pulls in
# four2a/FFTW; rather than drag that in for an unused code path, this
# script compiles a local no-op stub (watterson_stub.f90, written
# alongside this script) with a matching signature.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/target/wsprsim}"

if [[ -z "$WSJTX_DIR" || ! -d "$WSJTX_DIR" ]]; then
  echo "error: WSJT-X source tree not found. Pass its path as the first argument." >&2
  echo "  $0 /path/to/WSJT-X" >&2
  exit 1
fi

if ! command -v gfortran &>/dev/null; then
  echo "error: gfortran not found. Install with:" >&2
  echo "  sudo apt-get install gfortran" >&2
  exit 1
fi

LIB="$WSJTX_DIR/lib"
mkdir -p "$OUT_DIR"
BUILD="$OUT_DIR/build"
mkdir -p "$BUILD"

echo "Building wsprsim from $WSJTX_DIR ..."
echo "Output dir: $OUT_DIR"

cd "$BUILD"

FFLAGS=(-O2 -Wall -Wno-unused-variable -Wno-unused-dummy-argument -I"$LIB" -I"$LIB/wsprd")

echo "  [1/14] wavhdr.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/wavhdr.f90"

echo "  [2/14] deg2grid.f90 + grid2deg.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/deg2grid.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/grid2deg.f90"

echo "  [3/14] fmtmsg.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fmtmsg.f90"

echo "  [4/14] packjt.f90 (module: packcall/packgrid/pack50/packpfx)"
gfortran "${FFLAGS[@]}" -c "$LIB/packjt.f90"

echo "  [5/14] hashing.f90 + hash.f90 + nhash.c"
gfortran "${FFLAGS[@]}" -c "$LIB/hashing.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/hash.f90"
gcc -O2 -c "$LIB/wsprd/nhash.c"

echo "  [6/14] wqencode.f90 (WSPR source encoding)"
gfortran "${FFLAGS[@]}" -c "$LIB/wqencode.f90"

echo "  [7/14] encode232.f90 (includes conv232.f90 via -I)"
gfortran "${FFLAGS[@]}" -c "$LIB/encode232.f90"

echo "  [8/14] inter_wspr.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/inter_wspr.f90"

echo "  [9/14] genwspr.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/genwspr.f90"

echo "  [10/14] wspr_wav.f90 (channel symbols -> 12 kHz iwave)"
gfortran "${FFLAGS[@]}" -c "$LIB/wsprd/wspr_wav.f90"

echo "  [11/14] gran.c + sgran.c + init_random_seed.{f90,c}"
gcc -O2 -c "$LIB/gran.c"
gcc -O2 -c "$LIB/sgran.c"
gfortran "${FFLAGS[@]}" -c "$LIB/init_random_seed.f90" -o init_random_seed_f90.o
gcc -O2 -c "$LIB/init_random_seed.c" -o init_random_seed_c.o

echo "  [12/14] watterson_stub.f90 (unused nwav=0 path, avoids FFTW dep)"
gfortran "${FFLAGS[@]}" -c "$SCRIPT_DIR/wsprsim_watterson_stub.f90" -o watterson_stub.o

echo "  [13/14] link wsprsim"
gfortran "${FFLAGS[@]}" \
  wavhdr.o deg2grid.o grid2deg.o fmtmsg.o packjt.o \
  hashing.o hash.o nhash.o wqencode.o encode232.o inter_wspr.o \
  genwspr.o wspr_wav.o gran.o sgran.o \
  init_random_seed_f90.o init_random_seed_c.o watterson_stub.o \
  "$LIB/wsprd/wsprsimf.f90" \
  -o "$OUT_DIR/wsprsim" -lm

echo "  [14/14] done"
echo ""
echo "Built: $OUT_DIR/wsprsim"
echo "Usage: wsprsim \"message\" f0 DT fspread delay nwav nfiles snr"
echo "  message   e.g. \"JL1NIE PM95 37\" (call grid dBm -- WSPR has no CQ form)"
echo "  f0        freq offset from 1500 Hz center (Hz)"
echo "  DT        time offset from nominal (s)"
echo "  fspread   Watterson frequency spread (Hz); 0 for clean AWGN"
echo "  delay     Watterson delay (ms); 0 for clean AWGN"
echo "  nwav      1 for *.wav output (0 would be *.c2, unsupported by this build's stub)"
echo "  nfiles    number of files"
echo "  snr       SNR_2500 (dB, 2500 Hz reference bandwidth)"
echo "  e.g. wsprsim \"JL1NIE PM95 37\" 0 0.0 0.0 0.0 1 1 -28"
echo "Run 'scripts/gen_wspr_sweep_wavs.sh' to generate test WAVs."
