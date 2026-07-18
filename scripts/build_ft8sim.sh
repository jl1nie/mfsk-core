#!/usr/bin/env bash
# Build ft8sim from WSJT-X Fortran sources.
#
# Usage:
#   scripts/build_ft8sim.sh [WSJT-X-dir] [out-dir]
#
# Defaults:
#   WSJT-X-dir  ../WSJT-X  (sibling of this repo)
#   out-dir     target/ft8sim/
#
# Requires: gfortran, gcc, g++, libboost-dev
#   Ubuntu:  sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
#
# Mirrors scripts/build_ft4sim.sh (see docs/notes/FT4_BENCHMARK.md for the
# methodology this feeds into) — same shared lib subtree (FT8 and FT4 share
# LDPC(174,91), CRC-14, sgran RNG seeding), swapping ft4/genft4.f90 +
# ft4/gen_ft4wave.f90 for ft8/genft8.f90 + ft8/gen_ft8wave.f90. The FT8
# variant of gen_*wave additionally needs timer_module.f90 (a no-op stub
# by default, only wired to something real when WSJT-X's UI calls
# `timer => <callback>`).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/target/ft8sim}"

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

echo "Building ft8sim from $WSJTX_DIR ..."
echo "Output dir: $OUT_DIR"

# Compilation happens in $BUILD so .mod files land there.
# The -I flags point to source dirs that contain ft8_params.f90
# (included via `include 'ft8_params.f90'` in genft8.f90 / ft8sim.f90).
cd "$BUILD"

FFLAGS=(-O2 -Wall -Wno-unused-variable -Wno-unused-dummy-argument
        -I"$LIB/ft8" -I"$LIB")

echo "  [1/14] wavhdr.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/wavhdr.f90"

echo "  [2/14] prog_args.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/prog_args.f90"

echo "  [3/14] crc.f90 + crc14.cpp (boost) + sgran.c + init_random_seed.c"
gfortran "${FFLAGS[@]}" -c "$LIB/crc.f90"
g++ -O2 -c "$LIB/crc14.cpp" -o crc14.o
gcc -O2 -I"$LIB" -c "$LIB/sgran.c"
gcc -O2 -I"$LIB" -c "$LIB/init_random_seed.c"

echo "  [4a/14] packjt.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/packjt.f90"

echo "  [4b/14] 77bit/packjt77.f90"
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

echo "  [5e] timer_module.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/timer_module.f90"

# ldpc_174_91_c_generator.f90 is a data-only fragment included by
# encode174_91.f90 via `include`; do NOT compile it as a standalone unit.
echo "  [6/14] ft8/encode174_91.f90"
gfortran "${FFLAGS[@]}" -I"$LIB/ft8" -c "$LIB/ft8/encode174_91.f90"

echo "  [7/14] ft8/gen_ft8wave.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft8/gen_ft8wave.f90"

echo "  [8/14] ft8/genft8.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft8/genft8.f90"

echo "  [9/14] fftw3mod.f90 + four2a.f90 + ft8/watterson.f90"
gfortran "${FFLAGS[@]}" -I/usr/include -c "$LIB/fftw3mod.f90"
gfortran "${FFLAGS[@]}" -I/usr/include -c "$LIB/four2a.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft8/watterson.f90"

echo "  [10/14] gran.c"
gcc -O2 -c "$LIB/gran.c"

echo "  [link] ft8sim"
gfortran "${FFLAGS[@]}" \
  wavhdr.o prog_args.o crc.o crc14.o sgran.o init_random_seed.o \
  deg2grid.o grid2deg.o fmtmsg.o chkcall.o \
  packjt.o packjt77.o \
  gfsk_pulse.o timer_module.o fftw3mod.o four2a.o \
  encode174_91.o gen_ft8wave.o genft8.o watterson.o gran.o \
  "$LIB/ft8/ft8sim.f90" \
  -o "$OUT_DIR/ft8sim" -lfftw3f -lm -lstdc++

echo ""
echo "Built: $OUT_DIR/ft8sim"
echo "Run 'scripts/gen_ft8_sweep_wavs.sh' to generate the SNR sweep WAV corpus."
