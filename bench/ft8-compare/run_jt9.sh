#!/usr/bin/env bash
# Run WSJT-X's jt9 on WAV files, one file per scratch directory (jt9 writes decoded.txt, timer.out, ... into its cwd).
#
#   run_jt9.sh <jt9> <out-dir> <wav>...
#
# <out-dir>/jt9_<stem>.out        what jt9 printed (the reference decodes)
# <out-dir>/jt9_wall.txt          "<stem> <ms>": jt9's own timer.out total for the file
# With a JT9_TRACE build (scripts/build_jt9_upstream.sh), also
# <out-dir>/trace/<stem>.trace    ft8_trace.dat: stages, candidates, LLRs, cs, sync2d (env JT9_TRACE_CS_F / JT9_TRACE_SYNC_F, Hz)
# <out-dir>/resid/<stem>/resid_<nzhsym>_<pass>.bin   float32 residual after each search pass
# jt9 -8 -d 3 -L 200 -H 3000 is task `ft8/t1` of scripts/upstream_tasks.json. One OpenMP thread, so times are single-threaded.
set -u
[ $# -ge 3 ] || { sed -n '2,12p' "$0"; exit 2; }
jt9=$(realpath "$1"); out=$(realpath -m "$2"); shift 2
mkdir -p "$out/trace"
for wav in "$@"; do
  stem=$(basename "$wav" .wav); w=$(mktemp -d)
  cp "$wav" "$w/000000_0000.wav"
  (
    cd "$w" || exit 1
    OMP_NUM_THREADS=1 "$jt9" -8 -d 3 -L 200 -H 3000 000000_0000.wav > "$out/jt9_$stem.out" 2>/dev/null
    t=$(awk '$1=="jt9"{print $2*1000; exit}' timer.out 2>/dev/null)
    echo "$stem ${t:-NA}" >> "$out/jt9_wall.txt"
    [ -f ft8_trace.dat ] && cp ft8_trace.dat "$out/trace/$stem.trace"
    if ls resid_*.bin >/dev/null 2>&1; then mkdir -p "$out/resid/$stem" && cp resid_*.bin "$out/resid/$stem/"; fi
  )
  rm -rf "$w"
done
