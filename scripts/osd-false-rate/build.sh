#!/bin/bash
# False-accept rate of WSJT-X's own decode174_91 / decode240_101 on iid Gaussian LLRs (#452, #456).
#
#   scripts/osd-false-rate/build.sh <wsjtx-lib-dir> <174|240> <draws-per-process> [norder]
#
# <wsjtx-lib-dir> is the `lib/` of a WSJT-X checkout (v3.2.0-rc1 was used).
# Runs 12 seeds in parallel, each printing `PROGRESS` (every 1000 draws for 174, every 10 for
# 240) to out_<seed>.txt in a temporary directory (printed at the end), and a
# `FALSERATE` line when done. The counts are
# results with a valid CRC (nharderror > 0, as decode174_91's caller sees them).
#
# Cost: decode174_91 (maxosd=2, norder=2) is ~2.5 ms a draw; decode240_101 at
# norder=2 is fast enough, at norder=3 it took over 30 s a draw here (gfortran -O2),
# so 240/norder 3 was never measured. Pick draws so that the expected hits
# (rate x draws) are a handful, not a thousand: 5.8e-5 for FT8 upstream needs
# ~50 000 draws for 3 hits.
set -e
L=$1; M=$2; N=${3:-2000}; NORD=${4:-2}
HERE=$(cd "$(dirname "$0")" && pwd)
W=$(mktemp -d); cd "$W"
if [ "$M" = 174 ]; then
  grep -v "use crc\|iso_c_binding" "$L/ft8/encode174_91_nocrc.f90" > enc.f90
  gfortran -O2 -I"$L/ft8" -o fr "$HERE/main174.f90" "$L/ft8/decode174_91.f90" "$L/ft8/osd174_91.f90" \
    "$L/ft8/get_crc14.f90" enc.f90 "$L/platanh.f90" "$L/indexx.f90"
else
  grep -v "use crc\|iso_c_binding" "$L/fst4/encode240_101.f90" > enc.f90
  gfortran -O2 -I"$L/fst4" -o fr "$HERE/main240.f90" "$L/fst4/decode240_101.f90" "$L/fst4/osd240_101.f90" \
    "$L/fst4/get_crc24.f90" enc.f90 "$L/platanh.f90" "$L/indexx.f90"
fi
for s in 11 12 13 14 15 16 17 18 19 20 21 22; do
  if [ "$M" = 174 ]; then ./fr "$N" "$s" > "out_$s.txt" & else ./fr "$N" "$s" "$NORD" > "out_$s.txt" & fi
done
wait
tail -qn1 out_*.txt
echo "outputs in $W"
