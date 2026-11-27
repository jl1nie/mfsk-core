#!/usr/bin/env bash
# Host instructions a BP pass costs in the cells flow: the BP instructions of one `icap-cands` run (callgrind) divided by the passes `BpStats` counted in the same run.
# The three vendored recordings, 30 candidates, OSD off, no AP pass (the board's build); at the `d` floor 16 (the flow's) and 0 (the whole staircase). Needs valgrind.
# Usage: scripts/bp-instr-per-pass.sh        (design note section 27, CHANGELOG: 76 000 before the flat edge tables, 37 600 after, on x86)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
W="$(mktemp -d)"; trap 'rm -rf "$W"' EXIT
(cd "$ROOT/hosttest/icap-app" && CARGO_TARGET_DIR="$W/target" CARGO_PROFILE_RELEASE_DEBUG=1 \
  cargo build --release --no-default-features --features fixed-point --bin icap-cands 2>&1 | grep -E "^error" -A5 || true)
BIN="$W/target/release/icap-cands"
for n in qso3_busy qso1 qso2; do ln -s "$ROOT/embedded-poc/assets/$n.wav" "$W/$n.wav"; : > "$W/$n.ref"; done
for d in 16 0; do
  ti=0; tp=0
  for n in qso3_busy qso1 qso2; do
    p=$(D_MIN_SCORE=$d "$BIN" "$W/$n.wav" 2>/dev/null | awk -F'\t' '{p+=$11} END{print p}')
    D_MIN_SCORE=$d valgrind --tool=callgrind --callgrind-out-file="$W/cg.out" "$BIN" "$W/$n.wav" >/dev/null 2>&1
    i=$(callgrind_annotate --inclusive=yes --threshold=100 "$W/cg.out" 2>/dev/null | grep -E "bp_decode_generic_nms_with_scratch" | grep -E "^\s*[0-9,]+ \(" | head -1 | awk '{gsub(",","",$1); print $1}')
    echo "d floor $d $n: BP $i instructions, $p passes -> $((i / p)) a pass"
    ti=$((ti + i)); tp=$((tp + p))
  done
  echo "d floor $d all: $((ti / tp)) instructions a pass ($ti / $tp)"
done
