#!/usr/bin/env bash
# P3 (docs/notes/JTTY_UPSTREAM.md): how does this crate's multi-signal decoding —
# subtraction, retro re-sweep, message assembly, with the candidates of a pass
# decoded against one residual (D4) — compare with WSJT-X's rjtty, which decodes
# them one after another?
#
# Usage:
#   scripts/jtty_multi_study.sh [WSJT-X-dir] [work-dir] [N] [easy|hard]
#
# N random two-station recordings (default 100), seeded: station A `CQ K1ABC CQ`
# at 1460-1540 Hz, SNR -6..0 dB; station B `CQ W9XYZ CQ` 25-120 Hz away, SNR
# -14..-6 dB, its start within 1.5 s of A's. Both are generated noiseless by sjtty,
# scaled to their SNR (2500 Hz) against one noise-only recording and added. The
# `hard`: B only 12-50 Hz from A, starting within 0.3 s of it, at -18..-10 dB —
# the case the schedule difference matters in. The same files go to rjtty and to this crate; the table says how many of the 2N
# messages each recovered, and where they differ.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
WSJTX_DIR="${1:-$(cd "$REPO_ROOT/../WSJT-X" 2>/dev/null && pwd || echo "")}"
WORK="${2:-$REPO_ROOT/target/jtty_multi}"
N="${3:-100}"
MODE="${4:-easy}"

"$SCRIPT_DIR/build_jttysim.sh" "$WSJTX_DIR" >/dev/null
TOOLS="$REPO_ROOT/target/jttysim/build"
rm -rf "$WORK"; mkdir -p "$WORK/wav" "$WORK/gen"

( cd "$WORK/gen" && "$TOOLS/sjtty" "AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AAAAA AA" 1500 0.3 AW 0 384 1 -99 > /dev/null && mv 000000_000001.wav noise.wav )

python3 - "$TOOLS/sjtty" "$WORK" "$N" "$MODE" <<'PY'
import sys, os, subprocess, random, wave, array, math, shutil
sjtty, work, n, mode = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
rnd = random.Random(20260926)
def read(p):
    w = wave.open(p); a = array.array('h'); a.frombytes(w.readframes(w.getnframes())); return a
noise = read(f"{work}/gen/noise.wav")
def clean(msg, f0, dt, tag):
    d = f"{work}/gen/{tag}"; os.makedirs(d, exist_ok=True)
    subprocess.run([sjtty, msg, f"{f0:.1f}", f"{dt:.2f}", "AW", "0", "384", "1", "91"], cwd=d, stdout=subprocess.DEVNULL, check=True)
    return read(f"{d}/000000_000001.wav")
rows = []
for i in range(1, n + 1):
    fa = rnd.uniform(1460, 1540); dta = rnd.uniform(0.3, 1.5)
    hard = mode == 'hard'
    df = (rnd.uniform(12, 50) if hard else rnd.uniform(25, 120)) * rnd.choice((-1, 1)); fb = fa + df
    dtb = max(0.1, dta + rnd.uniform(-0.3, 0.3) if hard else dta + rnd.uniform(-1.5, 1.5))
    sa = rnd.uniform(-6, 0); sb = rnd.uniform(-18, -10) if hard else rnd.uniform(-14, -6)
    a = clean("CQ K1ABC CQ", fa, dta, "a"); b = clean("CQ W9XYZ CQ", fb, dtb, "b")
    ga = math.sqrt(2 * 4167.0 * 10 ** (sa / 10)) / 32766.9; gb = math.sqrt(2 * 4167.0 * 10 ** (sb / 10)) / 32766.9
    m = max(len(a), len(b))
    mix = array.array('h', [0]) * m
    for k in range(m):
        x = noise[k] + (ga * a[k] if k < len(a) else 0) + (gb * b[k] if k < len(b) else 0)
        mix[k] = max(-32768, min(32767, int(round(x))))
    name = f"multi_{i:03d}.wav"
    w = wave.open(f"{work}/wav/{name}", "wb"); w.setnchannels(1); w.setsampwidth(2); w.setframerate(12000)
    w.writeframes(mix.tobytes()); w.close()
    rows.append(f"{name}\t{fa:.1f}\t{dta:.2f}\t{sa:.1f}\t{fb:.1f}\t{dtb:.2f}\t{sb:.1f}")
open(f"{work}/scenarios.tsv", "w").write("\n".join(rows) + "\n")
PY

# ours
( cd "$REPO_ROOT" && JTTY_DIAG_DIR="$WORK/wav" MFSK_REQUIRE_CORPUS=1 \
    cargo test -p mfsk-core --release --no-default-features \
      --features jtty,std,fft-rustfft,parallel --test jtty_rx diag_messages \
      -- --ignored --nocapture 2>&1 ) | grep '^MSGS' > "$WORK/ours.txt"

# rjtty at ndebug 0 prints only frames that were not absorbed as repeats (ndebug 1
# also prints those, which would credit it with decodes the GUI never shows).
# Each line is `freq text-so-far`; keep the distinct texts that are not a prefix of
# another (a message's earlier frames are its own prefixes).
( cd "$WORK/wav" && for f in multi_*.wav; do
    texts="$("$TOOLS/rjtty" 4.6 0 384 1500 50 "$f" | awk 'NF>=2 {$1=""; sub(/^ +/,""); print}' | python3 -c '
import sys
t = []
for l in sys.stdin:
    l = l.rstrip("\n")
    if l and l not in t: t.append(l)
print("|".join(x for x in t if not any(y != x and y.startswith(x) for y in t)))')"
    echo "MSGS $f $texts"
  done ) > "$WORK/rjtty.txt"

python3 - "$WORK" <<'PY'
import sys
work = sys.argv[1]
def load(p):
    d = {}
    for l in open(p):
        parts = l.rstrip("\n").split(" ", 2)
        d[parts[1]] = set(t for t in (parts[2].split("|") if len(parts) > 2 else []) if t)
    return d
ours, up = load(f"{work}/ours.txt"), load(f"{work}/rjtty.txt")
sc = {l.split("\t")[0]: l.rstrip().split("\t") for l in open(f"{work}/scenarios.tsv")}
want = ("CQ K1ABC CQ", "CQ W9XYZ CQ")
tot = {"ours": [0, 0], "rjtty": [0, 0]}; only_ours = only_up = 0
print(f"{'file':<14}{'A dB':>6}{'B dB':>6}{'df Hz':>7}{'dt s':>6}  ours(A B)  rjtty(A B)")
for f in sorted(sc):
    o = [w in ours.get(f, ()) for w in want]; r = [w in up.get(f, ()) for w in want]
    for k in (0, 1):
        tot["ours"][k] += o[k]; tot["rjtty"][k] += r[k]
        only_ours += o[k] and not r[k]; only_up += r[k] and not o[k]
    if o != r:
        s = sc[f]
        print(f"{f:<14}{s[3]:>6}{s[6]:>6}{float(s[4])-float(s[1]):>7.0f}{float(s[5])-float(s[2]):>6.2f}  {int(o[0])} {int(o[1])}        {int(r[0])} {int(r[1])}")
n = len(sc)
print(f"\nrecovered of {n} per station (A strong, B weak):  ours {tot['ours'][0]} / {tot['ours'][1]}   rjtty {tot['rjtty'][0]} / {tot['rjtty'][1]}")
print(f"messages only ours: {only_ours}    only rjtty: {only_up}")
PY
