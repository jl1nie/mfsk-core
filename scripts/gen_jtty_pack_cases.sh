#!/usr/bin/env bash
# Golden cases for the JTTY text packer: a deterministic set of messages, each
# run through WSJT-X's own `pack_jtty` (v3.2.0-rc1, via jtty_pack_oracle).
#
#   scripts/gen_jtty_pack_cases.sh [tools-dir] [out-file]
#
#   tools-dir  target/jttysim/build (scripts/build_jttysim.sh builds it)
#   out-file   embedded-poc/assets/golden/jtty/pack_cases.tsv
#
# The cases are curated ones (every atom kind, every profile difference, the
# edges of the grammar) plus seeded random compositions of a vocabulary made of
# the tokens the packer treats specially: standard and non-standard calls,
# grids, numbers around the 17-bit limit, location tokens, ARRL sections, class
# tokens, control phrases, `599`, and words that are plain text. Seeded, so the
# file is reproducible byte for byte.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TOOLS="${1:-$REPO_ROOT/target/jttysim/build}"
OUT="${2:-$REPO_ROOT/embedded-poc/assets/golden/jtty/pack_cases.tsv}"
ORACLE="$TOOLS/jtty_pack_oracle"
[ -x "$ORACLE" ] || { echo "no $ORACLE — run scripts/build_jttysim.sh first" >&2; exit 1; }

python3 - "$ORACLE" "$OUT" <<'PY'
import random, subprocess, sys

oracle, out = sys.argv[1:3]
rnd = random.Random(477)

calls = ["K1ABC", "JA1XYZ", "W9XYZ", "DL1ABC", "3DA0RS", "VK3NV", "9A1A", "N0CALL", "HB9XYZ",
         "K1ABC/P", "PJ4/K1ABC", "K1ABCDEF", "QA1BC", "ja1xyz", "K1ABC/QRP"]
grids = ["FN42", "PM95", "AA00", "RR99", "SS00", "fn42", "IO91", "FN4", "FN421"]
numbers = ["5", "05", "005", "599", "59", "73", "100", "1234", "131071", "131072", "0", "1234567",
           "007", "12", "40", "90", "1999"]
locs = ["CA", "TX", "ON", "CT", "NY", "VE", "PA", "MA", "K", "ABCD", "5B", "A1", "09", "0A", "ZZ"]
sections = ["EMA", "WMA", "ORG", "NLI", "DX", "BOGUS", "ENY", "MB", "NT"]
classes = ["1A", "3A", "33A", "10F", "2G", "4B", "32A", "1F", "0A", "12C"]
controls = ["AGN?", "CALL?", "AGN CALL", "NR?", "AGN NR", "EXCH?", "STATE?", "SECTION?", "ZONE?",
            "GRID?", "RPRT?", "QSL TU", "TU", "QRZ?", "QSO B4", "WAIT", "NIL?", "OK?"]
words = ["CQ", "TU", "NOW", "DE", "73", "AGN", "HELLO", "WORLD", "RIG", "ANT", "WX", "SUNNY", "QTH",
         "NAME", "BT", "K", "R", "RST", "ES", "FB", "OM", "VY", "PSE", "QRZ", "the", "quick", "brown",
         "fox", "~", "#", "{}", "é", "|", "\\", "@", "^", "*", "a.b", "1/2", "5NN"]
tokens = calls + grids + numbers + locs + sections + classes + controls + words

def rand_word():
    n = rnd.randint(1, 8)
    alpha = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 +-./?!\"#$%,&*()_'=[]{}<>|:;abcxyz@^\\`~"
    return "".join(rnd.choice(alpha) for _ in range(n))

cases = []
def add(msg):
    if len(msg) <= 80:
        for profile in (0, 1, 2):
            cases.append((profile, msg))

# curated
for c in controls + calls + grids + numbers + locs + classes:
    add(c)
for c in ["CQ K1ABC CQ", "TU K1ABC CQ", "K1ABC TU", "K1ABC AGN?", "TU NOW K1ABC", "  cq  k1abc   cq  ",
          "599 5", "599 05", "599 005", "599 5 599 12", "599 CA", "599 TX 599 ON", "599 05 CA", "599 FN42",
          "599 131071", "599 131072", "599 0", "599", "599 599", "5NN", "3A EMA", "1F DX", "33A EMA",
          "599 1234", "599 12 34", "FN42 599 5 CA", "CQ CQ CQ K1ABC K1ABC", "", " ", "   ",
          "RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!", "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG",
          "K1ABC DE JA1XYZ 599 5 CA K", "CQ CQ CQ DE K1ABC K1ABC K1ABC PSE K",
          "A" * 80, "AB " * 26, "1 " * 40, "TU " * 26, "599 5 " * 13, "  A  ~ B  "]:
    add(c)

# random compositions
for _ in range(1500):
    k = rnd.randint(1, 9)
    parts = [rnd.choice(tokens) if rnd.random() < 0.9 else rand_word() for _ in range(k)]
    if rnd.random() < 0.3:
        parts.insert(rnd.randint(0, len(parts)), "599")
    seps = [" " * rnd.choice([1, 1, 1, 1, 2, 3]) for _ in parts]
    msg = "".join(p + s for p, s in zip(parts, seps))
    if rnd.random() < 0.1:
        msg = " " + msg
    msg = msg[:80]
    if rnd.random() < 0.5:
        for profile in (rnd.randint(0, 2),):
            cases.append((profile, msg))
    else:
        add(msg)

# random strings
for _ in range(300):
    n = rnd.randint(1, 80)
    msg = "".join(rnd.choice("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 ") for _ in range(n))
    cases.append((rnd.randint(0, 2), msg))

# a tab or a newline would break the format, and a non-ASCII byte would be the byte-vs-character
# difference documented in pack.rs; keep those two out of the *file* but not out of the curated set above
cases = [(p, m) for p, m in cases if "\t" not in m and "\n" not in m and all(ord(c) < 128 for c in m)]
seen, uniq = set(), []
for c in cases:
    if c not in seen:
        seen.add(c); uniq.append(c)

inp = "".join(f"{p}\t{m}\n" for p, m in uniq)
res = subprocess.run([oracle], input=inp, capture_output=True, text=True, check=True)
lines = res.stdout.splitlines()
assert len(lines) == len(uniq), (len(lines), len(uniq))
counts = {}
for l in lines:
    f = l.split("\t")
    counts[f[2]] = counts.get(f[2], 0) + 1
with open(out, "w") as fh:
    fh.write("# WSJT-X v3.2.0-rc1 pack_jtty over scripts/gen_jtty_pack_cases.sh's messages\n")
    fh.write("# profile (0 unknown, 1 Field Day, 2 RTTY Roundup)\tmessage\tnframes (-1 rejected)\tframes: 34-bit payloads, 9 hex digits each\n")
    fh.write(res.stdout)
print(f"{len(lines)} cases; nframes histogram: " + " ".join(f"{k}:{counts[k]}" for k in sorted(counts, key=int)))
PY
