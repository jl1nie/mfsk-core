#!/usr/bin/env bash
# Golden cases for FST4W transmit encoding: a deterministic list of messages, each
# run through WSJT-X's own genfst4 with iwspr=1 (scripts/fst4w/genfst4w_oracle.f90,
# built from the v3.3.0-beta1 tag by scripts/fst4w/build_oracle.sh).
#
#   scripts/fst4w/gen_tones_cases.sh [oracle] [out-file]
#
#   oracle    target/fst4w-oracle/genfst4w_oracle
#   out-file  embedded-poc/assets/golden/fst4w/tones_beta1.tsv
#
# The cases are curated ones (every WSPR message type, the edges of each field's
# grammar, rejections) plus seeded random compositions of the tokens the packer
# treats specially. Seeded, so the file is reproducible byte for byte.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ORACLE="${1:-$REPO_ROOT/target/fst4w-oracle/genfst4w_oracle}"
OUT="${2:-$REPO_ROOT/embedded-poc/assets/golden/fst4w/tones_beta1.tsv}"
[ -x "$ORACLE" ] || { echo "no $ORACLE - run scripts/fst4w/build_oracle.sh first" >&2; exit 1; }
COMMIT="$(cat "$(dirname "$ORACLE")/exported-commit" 2>/dev/null || echo unknown)"

python3 -I - "$ORACLE" "$OUT" "$COMMIT" <<'PY'
import random, subprocess, sys

oracle, out, commit = sys.argv[1:4]
rnd = random.Random(649)

calls = ["K1ABC", "JA1XYZ", "W9XYZ", "DL1ABC", "VK3NV", "9A1A", "N0CALL", "HB9XYZ", "G4ABC", "3Y0Z",
         "A1B", "K9AN", "JL1NIE", "3DA0RS", "KH6ABC", "QA1BC", "K1ABCD", "W1ABCDE", "ja1xyz", "k1abc",
         "AA1AAA", "1A2BC", "A22A", "R1ABC", "KH1/KH7Z", "PJ4/K1ABC", "K1ABC/P", "K1ABC/QRP", "K1ABC/R"]
grids4 = ["FN42", "PM95", "AA00", "RR99", "SS00", "IO91", "fn42", "FN4", "FN421", "RA00", "AR99", "0042"]
grids6 = ["FN42AB", "FN42XX", "PM95sr", "IO91WM", "FN42YA", "FN42aa", "FN42A", "FN42AB1"]
dbms = [str(i) for i in range(0, 62)] + ["05", "007", "-1", "3.5", "abc", "1e1", "+10", "100"]
prefixes = ["PJ4", "VP9", "KH6", "3DA", "A", "AB", "K", "0K", "ZZZ", "W1AW", "OH", "9A", "ZS6"]
suffixes = ["P", "R", "QRP", "1", "12", "123", "0K", "A1", "999", "ABCD", "7"]

msgs = []
def add(m): msgs.append(m)

for c in calls[:20]:
    for g in grids4[:6]:
        add(f"{c} {g} {rnd.choice([0,3,7,10,13,17,20,23,27,30,33,37,40,43,47,50,53,57,60])}")
for d in dbms:
    add(f"K1ABC FN42 {d}")
for g in grids4 + grids6:
    add(f"K1ABC {g} 37")
    add(f"<K1ABC> {g}")
for p in prefixes:
    for base in ["K1ABC", "JA1XYZ", "G4AB", "N0CALL"]:
        add(f"{p}/{base} {rnd.choice([0,10,37,60])}")
for s in suffixes:
    for base in ["K1ABC", "JA1XYZ", "W9XYZ"]:
        add(f"{base}/{s} {rnd.choice([0,10,37,60])}")
for t in ["<K1ABC> FN42AB", "<PJ4/K1ABC> FN42AB", "<KH1/KH7Z> IO91ab", "<...> FN42", "<AB> FN42", "<ABC> FN42",
          "<K1ABC FN42", "K1ABC> FN42", "<<K1ABC>> FN42", "<K1ABC> FN42 37", "<K1ABC>FN42", "<K1ABC-1> FN42AB",
          "<VERYLONGCALL1> FN42", "<JA1XYZ/QRP> PM95", "<3DA0RS> KG33", "<k1abc> fn42ab", "<  > FN42",
          "<K1ABC> AA00AA", "<K1ABC> RR99XX", "<K1ABC> RR99XY", "<K1ABC/1234> FN42"]:
    add(t)
for t in ["CQ K1ABC FN42", "hello", "73", "K1ABC", "K1ABC FN42", "FN42 37", "K1ABC FN42 37 38",
          "  K1ABC  FN42   37  ", "K1ABC\tFN42 37", "CQ POTA K1ABC FN42", "K1ABC RR73", "K1ABC W9XYZ -10",
          "TNX 73 GL", "0123456789ABCDEF01", "A" * 40, "K1ABC FN42 37 " + "x" * 30, "K1ABC FN42 3" + "7" * 40]:
    add(t)

toks = calls + grids4 + grids6 + dbms + [p + "/" + c for p in prefixes[:6] for c in calls[:4]] + \
       [c + "/" + s for c in calls[:4] for s in suffixes[:6]] + \
       ["<" + c + ">" for c in calls] + ["CQ", "DE", "QRZ", "73", "RR73", "<...>", "TEST"]
for _ in range(900):
    n = rnd.choice([2, 2, 3, 3, 3, 4])
    m = " ".join(rnd.choice(toks) for _ in range(n))
    if rnd.random() < 0.1:
        m = m.lower()
    if rnd.random() < 0.05:
        m = "  " + m + "  "
    add(m)

seen, uniq = set(), []
for m in msgs:
    if m not in seen and m.strip():
        seen.add(m); uniq.append(m)

res = subprocess.run([oracle], input="\n".join(uniq) + "\n", capture_output=True, text=True, check=True)
lines = res.stdout.splitlines()
assert len(lines) == len(uniq), (len(lines), len(uniq))
with open(out, "w") as f:
    f.write(f"# WSJT-X v3.3.0-beta1 ({commit}) genfst4 with iwspr=1 over scripts/fst4w/gen_tones_cases.sh's messages\n")
    f.write("# message\tmsgsent ('*** bad message ***' = rejected)\tiwspr out\ti3.n3 (pack77 on the same options)\t74 bits (50 payload + CRC-24)\t160 tones\n")
    for l in lines:
        f.write(l + "\n")
print(f"{len(lines)} cases -> {out}")
PY
