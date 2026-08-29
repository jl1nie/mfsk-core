#!/usr/bin/env bash
# Build ft4sim from WSJT-X Fortran sources.
#
# Usage:
#   scripts/build_ft4sim.sh [WSJT-X-dir] [out-dir]
#
# Defaults:
#   WSJT-X-dir  ../WSJT-X  (sibling of this repo)
#   out-dir     target/ft4sim/
#
# Requires: gfortran, gcc
#   Ubuntu:  sudo apt-get install gfortran
#
# Mirrors scripts/build_fst4sim.sh (see docs/notes/FST4_BENCHMARK.md for the
# methodology this feeds into) — same shared lib subtree, swapping the
# fst4-specific compile units for ft4/genft4.f90 + ft4/gen_ft4wave.f90 +
# ft8/encode174_91.f90 (FT4 reuses FT8's LDPC(174,91) code).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/target/ft4sim}"

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

echo "Building ft4sim from $WSJTX_DIR ..."
echo "Output dir: $OUT_DIR"

# Compilation happens in $BUILD so .mod files land there.
# The -I flags point to source dirs that contain ft4_params.f90
# (included via `include 'ft4_params.f90'` in genft4.f90 / ft4sim.f90).
cd "$BUILD"

FFLAGS=(-O2 -Wall -Wno-unused-variable -Wno-unused-dummy-argument
        -I"$LIB/ft4" -I"$LIB")

echo "  [1/13] wavhdr.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/wavhdr.f90"

echo "  [2/13] prog_args.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/prog_args.f90"

echo "  [3/13] crc.f90 + crc14.cpp (boost) + sgran.c + init_random_seed.c"
gfortran "${FFLAGS[@]}" -c "$LIB/crc.f90"
g++ -O2 -c "$LIB/crc14.cpp" -o crc14.o
gcc -O2 -I"$LIB" -c "$LIB/sgran.c"
gcc -O2 -I"$LIB" -c "$LIB/init_random_seed.c"

echo "  [4a/13] packjt.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/packjt.f90"

echo "  [4b/13] 77bit/packjt77.f90"
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

# ldpc_174_91_c_generator.f90 is a data-only fragment included by
# encode174_91.f90 via `include`; do NOT compile it as a standalone unit.
echo "  [6/13] ft8/encode174_91.f90"
gfortran "${FFLAGS[@]}" -I"$LIB/ft8" -c "$LIB/ft8/encode174_91.f90"

echo "  [7/13] ft4/gen_ft4wave.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft4/gen_ft4wave.f90"

echo "  [8/13] ft4/genft4.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft4/genft4.f90"

echo "  [9/13] fftw3mod.f90 + four2a.f90 + ft8/watterson.f90"
gfortran "${FFLAGS[@]}" -I/usr/include -c "$LIB/fftw3mod.f90"
gfortran "${FFLAGS[@]}" -I/usr/include -c "$LIB/four2a.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/ft8/watterson.f90"

echo "  [10/13] gran.c"
gcc -O2 -c "$LIB/gran.c"

echo "  [link] ft4sim"
gfortran "${FFLAGS[@]}" \
  wavhdr.o prog_args.o crc.o crc14.o sgran.o init_random_seed.o \
  deg2grid.o grid2deg.o fmtmsg.o chkcall.o \
  packjt.o packjt77.o \
  gfsk_pulse.o fftw3mod.o four2a.o \
  encode174_91.o gen_ft4wave.o genft4.o watterson.o gran.o \
  "$LIB/ft4/ft4sim.f90" \
  -o "$OUT_DIR/ft4sim" -lfftw3f -lm -lstdc++

# `ft4sim_mult` is the *multi-signal* sibling: it reads a `messages.txt`
# of (SNR, DT, frequency, message) rows and lays several signals into
# one slot, each at its own SNR and a **random DT in +/-0.5 s**. Same
# objects, one more link step.
#
# Why it is worth building: `ft4sim` puts exactly one signal in a slot
# at DT=0.0, which makes it blind to every question about a *crowded*
# band -- candidate counts, ranking depth, phantom decodes -- and to
# timing spread (see `docs/notes/FT4_BENCHMARK.md` sections 18 and 21.2,
# where both gaps bit).
echo "  [link] ft4sim_mult"
gfortran "${FFLAGS[@]}" \
  wavhdr.o prog_args.o crc.o crc14.o sgran.o init_random_seed.o \
  deg2grid.o grid2deg.o fmtmsg.o chkcall.o \
  packjt.o packjt77.o \
  gfsk_pulse.o fftw3mod.o four2a.o \
  encode174_91.o gen_ft4wave.o genft4.o watterson.o gran.o \
  "$LIB/ft4/ft4sim_mult.f90" \
  -o "$OUT_DIR/ft4sim_mult" -lfftw3f -lm -lstdc++

echo ""
echo "Built: $OUT_DIR/ft4sim"
echo "Built: $OUT_DIR/ft4sim_mult"
echo "Run 'scripts/gen_ft4_sweep_wavs.sh' to generate the SNR sweep WAV corpus,"
echo "or 'scripts/gen_ft4_mult_wavs.sh' for the crowded-band corpus."
