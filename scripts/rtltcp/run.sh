#!/bin/bash
# Replay one recording to one client and wait for both (#686).
# usage: scripts/rtltcp/run.sh RECORDING.bin OUT_DIR NAME PORT CLIENT [client args...]
# e.g.:  scripts/rtltcp/run.sh rec1.bin /tmp/rr back2 5106 target/release/skimmer --rate 1024000 --ch FT8@7074000
# The client is given --server 127.0.0.1:PORT --protocol rtl_tcp before its own arguments. It connects 0.3 s
# before the stream's first sample (the replay aligns that to the recording's phase modulo 15 s), because a
# client reads the greeting within 10 s.
set -eu
rec=$1; out=$2; name=$3; port=$4; client=$5; shift 5
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$out"
python3 -I "$here/replay.py" "$rec" "$port" > "$out/replay-$name.log" 2>&1 &
rp=$!
until grep -q "^W0" "$out/replay-$name.log"; do sleep 0.2; done
W0=$(awk '/^W0/{print $2}' "$out/replay-$name.log")
while [ "$(date +%s%N)" -lt $((W0 - 300000000)) ]; do sleep 0.05; done
span=$(awk '/^listening/{print int($6)+30}' "$out/replay-$name.log")
timeout "$span" "$client" --server "127.0.0.1:$port" --protocol rtl_tcp "$@" > "$out/out-$name.log" 2>&1 || true
kill "$rp" 2>/dev/null || true
wait "$rp" 2>/dev/null || true
