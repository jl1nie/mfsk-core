#!/usr/bin/env bash
# Run ft8-compare on WAV files one at a time, with our-side traces (needs ft8-trace-probe.patch applied for the traces).
#
#   run_ours.sh <ft8-compare binary> <out-dir> <wav>...
#
# <out-dir>/rtrace/<stem>.csv      file,decoder,message,freq_hz,dt_s,snr_db,ms   (our decoder only)
# <out-dir>/rtrace/<stem>.rtrace   MFSK_FT8_TRACE (stages, candidates, ladder outcome, LLRs); empty without the patch
# Env passed through: FT8_COMPARE_ONLY=mfsk|sic3 (default here: mfsk), FT8_COMPARE_STRATEGIES=1, MFSK_FT8_TRACE_CS_F / _SYNC_F (Hz),
# MFSK_FT8_DUMP_RESID=<dir>. Single thread (RAYON_NUM_THREADS=1).
set -u
[ $# -ge 3 ] || { sed -n '2,11p' "$0"; exit 2; }
bin=$(realpath "$1"); out=$(realpath -m "$2"); shift 2
mkdir -p "$out/rtrace"
for wav in "$@"; do
  stem=$(basename "$wav" .wav); rm -f "$out/rtrace/$stem.rtrace"
  MFSK_FT8_TRACE="$out/rtrace/$stem.rtrace" MFSK_FT8_TRACE_LLR=${MFSK_FT8_TRACE_LLR:-1} FT8_COMPARE_ONLY=${FT8_COMPARE_ONLY:-mfsk} \
    RAYON_NUM_THREADS=1 "$bin" "$out/rtrace/$stem.csv" "$wav" >/dev/null 2>&1
done
