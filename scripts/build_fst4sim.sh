#!/usr/bin/env bash
# Build fst4sim from WSJT-X Fortran sources.
#
# Usage:
#   scripts/build_fst4sim.sh [WSJT-X-dir] [out-dir]
#
# Defaults:
#   WSJT-X-dir  ../../../WSJT-X  (sibling of this repo's parent)
#   out-dir     target/fst4sim/
#
# Requires: gfortran, gcc
#   Ubuntu:  sudo apt-get install gfortran
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/target/fst4sim}"

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

echo "Building fst4sim from $WSJTX_DIR ..."
echo "Output dir: $OUT_DIR"

# Compilation happens in $BUILD so .mod files land there.
# The -I flags point to source dirs that contain fst4_params.f90
# (included via `include 'fst4_params.f90'` in genfst4.f90 / fst4sim.f90).
cd "$BUILD"

FFLAGS=(-O2 -Wall -Wno-unused-variable -Wno-unused-dummy-argument
        -I"$LIB/fst4" -I"$LIB")

echo "  [1/14] wavhdr.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/wavhdr.f90"

echo "  [2/14] prog_args.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/prog_args.f90"

echo "  [3/14] crc.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/crc.f90"

echo "  [4a/15] packjt.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/packjt.f90"

echo "  [4b/15] 77bit/packjt77.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/77bit/packjt77.f90"

echo "  [5a] deg2grid.f90 + grid2deg.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/deg2grid.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/grid2deg.f90"

echo "  [5b] fmtmsg.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fmtmsg.f90"

echo "  [5c] chkcall.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/chkcall.f90"

echo "  [5d] ft2/gfsk_pulse.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft2/gfsk_pulse.f90"

echo "  [5f] fst4/get_crc24.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fst4/get_crc24.f90"

# ldpc_240_*_generator.f90 are data-only fragments included by encode*.f90
# via `include`; do NOT compile them as standalone units.
echo "  [6/11] fst4/encode240_101.f90"
gfortran "${FFLAGS[@]}" -I"$LIB/fst4" -c "$LIB/fst4/encode240_101.f90"

echo "  [7/11] fst4/encode240_74.f90"
gfortran "${FFLAGS[@]}" -I"$LIB/fst4" -c "$LIB/fst4/encode240_74.f90"

echo "  [8/11] fst4/gen_fst4wave.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fst4/gen_fst4wave.f90"

echo "  [9/11] fst4/genfst4.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fst4/genfst4.f90"

echo "  [10/11] fftw3mod.f90 + four2a.f90 + ft8/watterson.f90 + fst4/lorentzian_fading.f90"
gfortran "${FFLAGS[@]}" -I/usr/include -c "$LIB/fftw3mod.f90"
gfortran "${FFLAGS[@]}" -I/usr/include -c "$LIB/four2a.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft8/watterson.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fst4/lorentzian_fading.f90"

echo "  [11/11] gran.c"
gcc -O2 -c "$LIB/gran.c"

echo "  [link] fst4sim"
gfortran "${FFLAGS[@]}" \
  wavhdr.o prog_args.o crc.o \
  deg2grid.o grid2deg.o fmtmsg.o chkcall.o \
  packjt.o packjt77.o \
  gfsk_pulse.o fftw3mod.o four2a.o \
  get_crc24.o encode240_101.o encode240_74.o \
  gen_fst4wave.o genfst4.o watterson.o lorentzian_fading.o gran.o \
  "$LIB/fst4/fst4sim.f90" \
  -o "$OUT_DIR/fst4sim" -lfftw3f -lm

echo ""
echo "Built: $OUT_DIR/fst4sim"
echo "Run 'scripts/gen_fst4_sim_wavs.sh' to generate test WAVs."
