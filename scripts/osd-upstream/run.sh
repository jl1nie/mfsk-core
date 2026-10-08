#!/bin/bash
# Runs WSJT-X's own OSD on recorded inputs and writes its answers: the oracle
# behind `mfsk-core/tests/osd_upstream_fixture.rs` (#417).
#
#   scripts/osd-upstream/run.sh <wsjtx-lib-dir> <174|240> <in.bin> <out.bin> [reps]
#
# <wsjtx-lib-dir> is the `lib/` of a WSJT-X checkout (v3.2.0-rc1 was used).
# Built with the flags a WSJT-X release build gives Fortran (CMake's Release
# `-O3` plus `-funroll-loops -fno-f2c`, CMake/CompilerFlags.cmake), so the
# printed `us/call` is upstream's speed on this machine. `reps` repeats the run
# for timing (default 1); the outputs are from the first.
#
# in.bin  (little-endian): i32 count, then per record i32 ndeep, i8 apmask[N],
#                          f32 llr[N]
# out.bin                : per record i32 nhardmin, f32 dmin, i8 cw[N]
#                          (nhardmin < 0: the winner failed the CRC)
#
# To check the committed fixture against upstream:
#   scripts/osd-upstream/run.sh ~/src/WSJT-X/lib 174 \
#       mfsk-core/tests/fixtures/osd_upstream/osd174_91_in.bin /tmp/o.bin
#   cmp /tmp/o.bin mfsk-core/tests/fixtures/osd_upstream/osd174_91_out.bin
set -e
L=$1; M=$2; IN=$(realpath "$3"); OUT=$(realpath -m "$4"); REPS=${5:-1}
HERE=$(cd "$(dirname "$0")" && pwd)
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT; cd "$W"
FLAGS="-O3 -funroll-loops -fno-f2c -fno-range-check"
if [ "$M" = 174 ]; then
  gfortran $FLAGS -I"$L/ft8" -o osd "$L/crc.f90" "$L/indexx.f90" "$L/ft8/get_crc14.f90" \
    "$L/ft8/encode174_91_nocrc.f90" "$L/ft8/osd174_91.f90" "$HERE/driver174.f90" 2>/dev/null
else
  gfortran $FLAGS -I"$L/fst4" -o osd "$L/crc.f90" "$L/indexx.f90" "$L/fst4/get_crc24.f90" \
    "$L/fst4/encode240_101.f90" "$L/fst4/osd240_101.f90" "$HERE/driver240.f90" 2>/dev/null
fi
./osd "$IN" "$OUT" "$REPS"
