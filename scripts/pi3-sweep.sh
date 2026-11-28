#!/usr/bin/env bash
# The Pi 3B study's sweep (docs/notes/PI3_JT9_GAP.md): every configuration over the corpus, on 1 and on 4 threads of this machine.
#   scripts/pi3-sweep.sh <corpus-dir> <out-dir> [configs...]
# <corpus-dir> holds slot.wav (12 kHz mono) with slot.ref beside it; `a_*` are the 22 recordings (jt9's 402), `b_*` the busy slots.
# Output: <out-dir>/<config>.t<threads>.txt (the example's S/D/T lines); `scripts/pi3-sweep.py <out-dir>` prints the table.
set -euo pipefail
corpus=${1:?corpus dir}; out=${2:?out dir}; shift 2
configs=("$@")
[ ${#configs[@]} -gt 0 ] || configs=(fast:off normal:off normal:cq normal:full deep:off deep:cq deep:full cells:p0_window cells:p0_late cells:sic2osd cells:sic3osd hybrid:0:2:late hybrid:10:2:late hybrid:30:2:late hybrid:100:2:late hybrid2:10:0:2:x hybrid2:10:10:2:known hybrid2:30:30:2:known)
mkdir -p "$out"
cd "$(dirname "$0")/.."
cargo build -p mfsk-core --release --features full,fixed-point,internal-testing --example pi3_sweep >&2
bin=target/release/examples/pi3_sweep
for c in "${configs[@]}"; do
  for t in 1 4; do
    f="$out/${c/:/_}.t$t.txt"
    [ -s "$f" ] && continue
    echo "== $c on $t thread(s)" >&2
    RAYON_NUM_THREADS=$t taskset -c "0-$((t-1))" "$bin" "$c" "$corpus"/*.wav > "$f"
  done
done
