#!/usr/bin/env bash
# Golden cases for msg::wspr::pack_message: a deterministic list of messages, each run through
# WSJT-X's own wqencode + wqdecode (scripts/wspr/wspr_pack_oracle.f90, built from the v3.2.0-rc1 tag
# by scripts/wspr/build_oracle.sh).
#
#   scripts/wspr/gen_cases.sh [oracle-dir] [out-file]
#
#   oracle-dir  target/wspr-oracle
#   out-file    mfsk-core/tests/fixtures/wspr_pack_rc1.tsv
#
# Columns: message (as given; \t and \\ escaped), status, ntype, 50 bits, what wqdecode prints.
# status: ok      wqencode packed it (both oracle builds agree)
#         reject  wqencode returned without writing anything
#         abort   a Fortran runtime error (unreadable power, bounds violation)
#         undefined  the two builds disagree: wqencode reads memory it never wrote
# Seeded, so the file is reproducible byte for byte from the same oracle.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ORDIR="${1:-$REPO_ROOT/target/wspr-oracle}"
OUT="${2:-$REPO_ROOT/mfsk-core/tests/fixtures/wspr_pack_rc1.tsv}"
[ -x "$ORDIR/wspr_pack_oracle" ] || { echo "no oracle in $ORDIR - run scripts/wspr/build_oracle.sh first" >&2; exit 1; }
COMMIT="$(cat "$ORDIR/exported-commit")"
python3 -I - "$ORDIR" "$OUT" "$COMMIT" <<'PY'
import itertools, os, random, subprocess, sys, tempfile

ordir, out, commit = sys.argv[1:4]
rnd = random.Random(2026)
cases = []
def add(m):
    if m not in cases:
        cases.append(m)

# ---- type 1 -------------------------------------------------------------
for p in range(0, 61):
    add(f"K1ABC FN42 {p}")
for p in ["-5", "-1", "61", "62", "99", "100", "1000", "2147483647", "2147483648", "-2147483648",
          "-2147483649", "abc", "", "3.5", "1e1", "+37", "+0", "-0", "37,", "37;", "37/", "37 x",
          "37x", "3*5", "0*5", "3*", "3*,", "2*+7", "+2*7", ",", "/", ",5", " 37", "  7", "0x10",
          "'37'", "T", "37\t", "\t37", "4 5", "37,5"]:
    add(f"K1ABC FN42 {p}")
    add(f"K1ABC FN42{p}")
for c in ["K1ABC", "JA1ABC", "K9AN", "W1AW", "G4ABC", "VK3NV", "3DA0XY", "3D0XY", "3XA1AB", "3X1ABC",
          "k1abc", "Ja1abc", "K1abc", "CQ", "QRZ", "DE", "A1", "AB1", "A1B", "K1ABCD", "K1ABCDE",
          "ABCDE", "K1", "K", "1A2B", "9Z9ZZZ", "K1AB1", "K1A-C", "K1.BC", "KA1", "QQ1QQQ", "A0AAA"]:
    add(f"{c} FN42 37")
    add(f"{c} IO91 0")
grids = ["AA00", "AA99", "RR00", "RR99", "JJ00", "FN42", "PM95", "IO91", "QF22", "JN58", "RO", "RRR",
         "73", "", "0", "7", "49", "50", "-50", "-51", "R12", "R-31", "R-0", "-0", "-00", "-1", "-01",
         "-09", "-30", "-31", "-32", "R-01", "R-30", "R-1", "R49", "R50", "+5", "SS01", "AA0", "A0AA",
         "12", "1 2", "-,", "R,", "R/", "fn42", "Fn42", "FN4", "FN", "F", "KA00", "LA99", "R0", "R00",
         "AAxx", "A1", "ZZ99", "RS99", "99", "49 ", "RO ", "73 ", "RRR", "RR", "R", "-", "R-", "3*", "2*5"]
for g in grids:
    add(f"K1ABC {g} 37")
    add(f"K1ABC {g}  37")
    add(f"K1ABC {g}     37")
for f1 in "ABCDEFGHIJKLMNOPQR":
    for f2 in "ABCDEFGHIJKLMNOPQR":
        for d in ("00", "99"):
            add(f"K1ABC {f1}{f2}{d} 10")
for _ in range(120):
    g = rnd.choice("ABCDEFGHIJKLMNOPQR") + rnd.choice("ABCDEFGHIJKLMNOPQR") + str(rnd.randrange(100)).zfill(2)
    add(f"K1ABC {g} {rnd.randrange(0, 61)}")
for s in ["", " ", "K1ABC", "K1ABC ", "K1ABC FN42", "K1ABC  FN42 37", " K1ABC FN42 37", "K1ABC FN42 37 EXTRA",
          "K1ABC FN42 37 EXTRAEXTRAEXTRAEXTRA", "K1ABC FN42 3 7", "CQ K1ABC FN42", "ONE", "K1ABC FN42 37 ",
          "K1ABC FN42 37/", "K1ABC FN42 3/7", "K1ABC 37 /", "K1ABC FN42 37 <", "K1ABC <FN42> 37",
          "/K1ABC 37", "K1ABC/ 37", "/ 37", "<", ">", "<>", "< > FN42LX 37", "K1ABC FN42 37>"]:
    add(s)

# ---- type 2 -------------------------------------------------------------
calls = ["K1ABC", "G4ABC", "JA1ABC", "W1AW", "K9AN", "PA3XYZ", "VK3NV", "AB1"]
pfx = ["PJ4", "VP9", "A", "AB", "ABC", "ABCD", "3A", "3DA", "9A9", "K", "KH6", "1", "12", "123", "A1B2", "pj4", "P-4", "P.4"]
for pf in pfx:
    for c in calls[:4]:
        add(f"{pf}/{c} 37")
for c in calls:
    for sf in ["7", "P", "A", "Z", "0", "9", "M", "QRP", "10", "99", "00", "26", "35", "AB", "A1", "1A", "-", "p", "5", "/", "7X", "100"]:
        add(f"{c}/{sf} 37")
for p in list(range(0, 61)):
    add(f"PJ4/K1ABC {p}")
    add(f"K1ABC/7 {p}")
for p in ["-5", "61", "99", "abc", "", "3.5", "1e1", ",", "3*5", "37,"]:
    add(f"PJ4/K1ABC {p}")
    add(f"K1ABC/7 {p}")
for s in ["PJ4/K1ABC FN42 37", "PJ4/K1ABC  37", " PJ4/K1ABC 37", "PJ4/K1ABC 37 EXTRA", "PJ4/ 37", "/K1ABC 37", "PJ4/K1 37",
          "PJ4/K1ABCDEF 37", "PJ4/K1ABC7 37", "P/ 37", "K/7 37", "AB/ 37", "ABCDEFGHIJ/K 37", "ABCDEFGHIJK/L 37",
          "ABCDEFGHI/KL 37", "ABCDEFGHIJ/KL 37", "ABCDEFGHIJKL/M 37", "ABCDEFGHIJKLM/N 37", "A/B 37", "A/BC 37",
          "A/BCD 37", "K1ABC/ABC/D 37", "PJ4//K1ABC 37", "///", "K1ABC 37 PJ4/", "K1ABC FN42 3/7 5",
          "K1ABC/7", "PJ4/K1ABC", "PJ4/K1ABC ", "CQ/K1ABC 37", "QRZ/K1ABC 37", "DE/K1ABC 37", "K1ABC/DE 37"]:
    add(s)

# ---- type 3 -------------------------------------------------------------
for p in list(range(0, 61)):
    add(f"<K1ABC> FN42LX {p}")
for p in ["-5", "61", "99", "abc", "", "3.5", ",", "3*5", "37,"]:
    add(f"<K1ABC> FN42LX {p}")
for c in ["K1ABC", "A", "AB", "JA1ABC", "PJ4/K1ABC", "K1ABC/P", "ABCDEFGHIJKL", "ABCDEFGHIJKLM", "ABCDEFGHIJKLMNO",
          "k1abc", "Ja1abc", "1", "-", "W1AW", "G4ABC", "K1ABC/7", "PJ4/K1ABCDE"]:
    add(f"<{c}> FN42LX 37")
for g in ["FN42LX", "AA00AA", "RR99XX", "fn42lx", "FN42", "FN4", "F", "FN42LX1", "FN42LXX", "AA00", "IO91WM",
          "PM95TL", "JN58TD", "RR99", "SS99AA", "FN42lx", "FN4LX", "N42LXF", "1N42LX", "FN42L", "FN42LXYZ12",
          "F12345", "ZZ99ZZ", "--42LX", "A", "AB", "ABC"]:
    add(f"<K1ABC> {g} 37")
    add(f"<K1ABC> {g} 3")
for s in ["<K1ABC>  FN42LX 37", "<K1ABC> FN42LX  37", "<K1ABC>  FN42LX  37", "<K1ABC> FN42LX   37", "<K1ABC>FN42LX 37",
          "<K1ABC", "<K1ABC FN42LX 37", "K1ABC> FN42LX 37", "<K1ABC>", "<K1ABC> ", "<K1ABC> FN42LX", "<K1ABC> FN42LX ",
          "<> FN42LX 37", "< K1ABC> FN42LX 37", "<K1 ABC> FN42LX 37", "<K1ABC > FN42LX 37", " <K1ABC> FN42LX 37",
          "<K1ABC> FN42LX 37 EXTRA", "<K1ABC> <FN42LX> 37", "<K1ABC><FN42LX> 37", "<<K1ABC>> FN42LX 37",
          "<K1ABC>> FN42LX 37", "<A>BBBBBBBBBBBBBBBBBBB", "<A>BBBBBBBBBBBBBBBBBBBBBBBBB", "<K1ABC> FN42LX/ 37",
          "<K1ABC> FN42LX 3/7", "<K1ABC>/ FN42LX 37", "<K1ABC> /FN42LX 37", "<K1ABC> FN42LX 37>",
          "K1ABC <FN42LX> 37", "K1ABC FN42LX 37", "K1ABC FN42LX 37<", "PJ4/K1ABC <37>"]:
    add(s)

# ---- seeded compositions ------------------------------------------------
toks = ["K1ABC", "PJ4/K1ABC", "K1ABC/7", "<K1ABC>", "FN42", "FN42LX", "37", "38", "0", "60", "73", "RRR", "RO",
        "-05", "R-05", "CQ", "QRZ", "DE", "<>", "/", "<", ">", "", "A", "k1abc", "99", "3.5", "ONE", "JA1ABC", "PM95"]
for _ in range(500):
    n = rnd.choice([1, 2, 3, 3, 3, 4, 5])
    add((" " * rnd.choice([1, 1, 1, 2])).join(rnd.choice(toks) for _ in range(n)))

def run(exe, msg):
    with tempfile.NamedTemporaryFile("wb", suffix=".txt", delete=False) as f:
        f.write(msg.encode("latin-1") + b"\n")
        name = f.name
    try:
        r = subprocess.run([exe, name], capture_output=True, timeout=20)
    finally:
        os.unlink(name)
    if r.returncode != 0:
        return "abort"
    return r.stdout.decode("latin-1").rstrip("\n")

def esc(m):
    return m.replace("\\", "\\\\").replace("\t", "\\t")

rows = []
counts = {}
for m in cases:
    a = run(os.path.join(ordir, "wspr_pack_oracle"), m)
    b = run(os.path.join(ordir, "wspr_pack_oracle_b"), m)
    if a == "abort" and b == "abort":
        row = (m, "abort", "0", "0" * 50, "")
    elif a != b:
        row = (m, "undefined", "0", "0" * 50, "")
    else:
        f = a.split("\t")
        assert len(f) == 4, (m, a)
        row = (m, f[0], f[1], f[2], f[3])
    counts[row[1]] = counts.get(row[1], 0) + 1
    rows.append(row)

with open(out, "w") as o:
    o.write(f"# pack_message against WSJT-X's own wqencode + wqdecode (lib/wqencode.f90, lib/wqdecode.f90) of v3.2.0-rc1 ({commit}).\n")
    o.write("# Made by scripts/wspr/gen_cases.sh from scripts/wspr/wspr_pack_oracle.f90, built with scripts/wspr/build_oracle.sh;\n")
    o.write(f"# regenerating it with the same oracle reproduces it byte for byte. {len(rows)} cases: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())) + ".\n")
    o.write("# Columns: message (\\t and \\\\ escaped), status (ok | reject | abort | undefined), ntype, 50 bits, what wqdecode prints (type 3 call unresolved: <...>).\n")
    o.write("# abort = Fortran runtime error; undefined = two builds with different uninitialised-memory fill disagree (upstream reads memory it never wrote).\n")
    for r in rows:
        o.write("\t".join((esc(r[0]),) + r[1:]) + "\n")
print(len(rows), counts)
PY
