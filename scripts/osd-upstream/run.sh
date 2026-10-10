#!/bin/bash
# Runs WSJT-X's own OSD on recorded inputs and writes its answers: the oracle
# behind `mfsk-core/tests/osd_upstream_fixture.rs` (#417).
#
#   scripts/osd-upstream/run.sh <wsjtx-lib-dir> <174|240|74> <in.bin> <out.bin> [reps]
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
# Mode 74 is `decode240_74_owned` (FST4W, v3.3.0-beta1), which is not an OSD
# alone but BP + `fastosd240_74`; export the tag first, never the working tree:
#   git -C ~/src/WSJT-X archive v3.3.0-beta1 lib | tar -x -C "$T"  # then "$T/lib"
# in.bin  (74): i32 count, then per record i32 Keff, i32 maxosd, i32 norder,
#               i8 apmask[240], f32 llr[240]
# out.bin (74): per record i32 ntype, i32 nharderror, f32 dmin, i8 cw[240],
#               i8 message74[74]  (ntype 0 / nharderror -1: no decode)
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
elif [ "$M" = 74 ]; then
  gfortran $FLAGS -I"$L/fst4" -o osd "$L/crc.f90" "$L/indexx.f90" "$L/platanh.f90" \
    "$L/fst4/get_crc24.f90" "$L/fst4/encode240_74.f90" "$L/fst4/fst4_osd_workspace.f90" \
    "$L/fst4/fastosd240_74.f90" "$L/fst4/decode240_74.f90" "$HERE/driver240_74.f90" 2>/dev/null
else
  gfortran $FLAGS -I"$L/fst4" -o osd "$L/crc.f90" "$L/indexx.f90" "$L/fst4/get_crc24.f90" \
    "$L/fst4/encode240_101.f90" "$L/fst4/osd240_101.f90" "$HERE/driver240.f90" 2>/dev/null
fi
./osd "$IN" "$OUT" "$REPS"
