#!/usr/bin/env bash
# Build jt9sim from WSJT-X Fortran sources.
#
# Usage:
#   scripts/build_jt9sim.sh [WSJT-X-dir] [out-dir]
#
# Defaults:
#   WSJT-X-dir  ../WSJT-X  (sibling of this repo)
#   out-dir     target/jt9sim/
#
# Requires: gfortran, gcc
#   Ubuntu:  sudo apt-get install gfortran
#
# Unlike ft8sim/ft4sim/fst4sim/jt65sim, jt9sim has no CMakeLists.txt
# target at all in WSJT-X's own build (not even linked against the
# full wsjt_fort library) -- it's built here the same way
# build_fst4sim.sh assembles a minimal standalone subset, picking out
# just jt9sim.f90's actual dependency closure (gen9 -> packjt/entail/
# encode232/interleave9/graycode, plus jt9fano/fano232 for jt9sim's
# own internal self-verify decode step).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
OUT_DIR="${2:-$REPO_ROOT/target/jt9sim}"

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

echo "Building jt9sim from $WSJTX_DIR ..."
echo "Output dir: $OUT_DIR"

cd "$BUILD"

FFLAGS=(-O2 -Wall -Wno-unused-variable -Wno-unused-dummy-argument -I"$LIB")

echo "  [1/12] wavhdr.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/wavhdr.f90"

echo "  [2/12] deg2grid.f90 + grid2deg.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/deg2grid.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/grid2deg.f90"

echo "  [3/12] fmtmsg.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fmtmsg.f90"

echo "  [4/12] chkcall.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/chkcall.f90"

echo "  [5/12] packjt.f90 (module: packmsg/unpackmsg/packbits/unpackbits/...)"
gfortran "${FFLAGS[@]}" -c "$LIB/packjt.f90"

echo "  [6/12] entail.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/entail.f90"

echo "  [7/12] encode232.f90 (includes conv232.f90 via -I)"
gfortran "${FFLAGS[@]}" -c "$LIB/encode232.f90"

echo "  [8/12] interleave9.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/interleave9.f90"

echo "  [9/12] graycode.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/graycode.f90"

echo "  [10/12] gen9.f90 (includes jt9sync.f90 via -I)"
gfortran "${FFLAGS[@]}" -c "$LIB/gen9.f90"

echo "  [11/12] fano232.f90 + jt9fano.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/fano232.f90"
gfortran "${FFLAGS[@]}" -c "$LIB/jt9fano.f90"

echo "  [12/12] gran.c + sgran.c + init_random_seed.{f90,c} + igray.c"
# Two distinct symbols needed: jt9sim.f90 calls the Fortran
# subroutine directly (gfortran mangles it to init_random_seed_),
# while sgran.c's sgran_() wrapper calls the plain C function
# init_random_seed() (no trailing underscore) -- both source files
# must be compiled, under different object names.
gcc -O2 -c "$LIB/gran.c"
gcc -O2 -c "$LIB/sgran.c"
gfortran "${FFLAGS[@]}" -c "$LIB/init_random_seed.f90" -o init_random_seed_f90.o
gcc -O2 -c "$LIB/init_random_seed.c" -o init_random_seed_c.o
gcc -O2 -c "$LIB/igray.c"

echo "  [link] jt9sim"
gfortran "${FFLAGS[@]}" \
  wavhdr.o deg2grid.o grid2deg.o fmtmsg.o chkcall.o packjt.o \
  entail.o encode232.o interleave9.o graycode.o gen9.o \
  fano232.o jt9fano.o gran.o sgran.o init_random_seed_f90.o \
  init_random_seed_c.o igray.o \
  "$LIB/jt9sim.f90" \
  -o "$OUT_DIR/jt9sim" -lm

echo ""
echo "Built: $OUT_DIR/jt9sim"
echo "Usage: jt9sim \"message\" fspan nsigs minutes SNR nfiles"
echo "  e.g. jt9sim \"CQ JL1NIE PM95\" 200 1 1 -15 1"
echo "Run 'scripts/gen_jt9_sweep_wavs.sh' to generate test WAVs."
