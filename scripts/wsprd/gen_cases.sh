#!/usr/bin/env bash
# Golden cases for msg::wspr::unpack and the WSPR decoder's message text: a deterministic list of
# 50-bit payloads, each run through WSJT-X's own `unpk_` (lib/wsprd/wsprd_utils.c, v3.2.0-rc1;
# scripts/wsprd/wsprd_unpack_oracle.c, built by scripts/wsprd/build_oracle.sh).
#
#   scripts/wsprd/gen_cases.sh [oracle-dir] [out-file]
#
#   oracle-dir  target/wsprd-unpack-oracle
#   out-file    mfsk-core/tests/fixtures/wsprd_unpack_rc1.tsv
#
# Needs mfsk-core/tests/fixtures/wspr_pack_rc1.tsv (scripts/wspr/): its `ok` rows are payloads.
# Columns: 50 bits, seed call (the hash table holds it; `-` = empty table), rc (unpk_'s return,
# 1 = wsprd prints nothing), text (wsprd's message field; `-` when rc is 1).
# Seeded, so the file is reproducible byte for byte from the same oracle.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ORDIR="${1:-$REPO_ROOT/target/wsprd-unpack-oracle}"
OUT="${2:-$REPO_ROOT/mfsk-core/tests/fixtures/wsprd_unpack_rc1.tsv}"
PACK="$REPO_ROOT/mfsk-core/tests/fixtures/wspr_pack_rc1.tsv"
[ -x "$ORDIR/wsprd_unpack_oracle" ] || { echo "no oracle in $ORDIR - run scripts/wsprd/build_oracle.sh first" >&2; exit 1; }
COMMIT="$(cat "$ORDIR/exported-commit")"
python3 -I - "$ORDIR" "$OUT" "$COMMIT" "$PACK" <<'PY'
import random, subprocess, sys

ordir, out, commit, pack = sys.argv[1:5]
rnd = random.Random(2026)
cases = []   # (bits, seed)
seen = set()
def add(bits, seed="-"):
    assert len(bits) == 50
    if (bits, seed) not in seen:
        seen.add((bits, seed))
        cases.append((bits, seed))

def bits_of(n1, n2):
    return format(n1, "028b") + format(n2, "022b")

def n1_of(c0, c1, d, l3, l4, l5):
    # unpackcall's mixed radix: 37 * 36 * 10 * 27 * 27 * 27
    return ((((c0 * 36 + c1) * 10 + d) * 27 + l3) * 27 + l4) * 27 + l5

C37 = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ "
def n1_text(s6):
    """n1 of a 6-character string laid out as the callsign slot (blank allowed)."""
    v = [C37.index(ch) for ch in s6]
    assert v[1] < 36 and v[2] < 10
    return n1_of(v[0], v[1], v[2], v[3] - 10 if v[3] >= 10 else 26, v[4] - 10 if v[4] >= 10 else 26,
                 v[5] - 10 if v[5] >= 10 else 26)

# ---- 1. every `ok` row of the wqencode fixture (+ the hash seeded for type 3) --------------
t3 = {}   # hash -> call, for the crafted type 3 rows below
for line in open(pack):
    if line.startswith("#"):
        continue
    f = line.rstrip("\n").split("\t")
    msg, status, ntype, bits = f[0], f[1], int(f[2]), f[3]
    if status != "ok":
        continue
    add(bits)
    if ntype < 0:
        call = msg[1:msg.index(">")]
        # Not an empty call (it hashes unmasked, wsprd never stores it) and not one past 10 characters:
        # the table holds decoded calls only, PPP/CCCCCC at most, and "<...>" then no longer fits.
        if call and len(call) <= 10:
            add(bits, call)
        t3[int(bits[28:], 2) >> 7] = call

# ---- 2. type 1: every power, ntype around the allowed set, grids ------------------------------
for (c0, c1, d, ls) in [(20, 1, 1, (0, 1, 2)), (19, 10, 1, (0, 26, 26)), (36, 20, 1, (26, 26, 26)),
                        (36, 36, 0, (26, 26, 26)), (35, 35, 9, (25, 25, 25)), (0, 0, 0, (0, 0, 0)),
                        (20, 1, 1, (26, 1, 2)), (20, 1, 1, (0, 26, 2)), (36, 20, 1, (0, 1, 26)),
                        (1, 36, 3, (1, 2, 3)), (13, 1, 1, (0, 21, 6))]:
    n1 = n1_of(c0, c1, d, *ls)
    for g in [0, 1, 100, 179, 180, 5000, 16000, 32399]:
        for ntype in list(range(0, 63)):
            add(bits_of(n1, g * 128 + ntype + 64))

# ---- 3. type 2: prefix and suffix forms (n3 = n2/128 + 32768*(nadd-1)) --------------------------
def n37(s):
    v = 0
    for ch in s.ljust(3):
        v = v * 37 + C37.index(ch)
    return v
pfx = ["PJ4", "VP9", "3D0", "3DA", "KH6", "W", "K", "1", "A", "AB", "9", "99", "ZZZ", "000", "A1", "1A",
       "Z9Z", " ", "A B", "OE", "9A", "KG4", "R", "RR", "DL", "4X", "9M2", "ZS", "T7", "C6A"]
for s6 in [(20, 1, 1, (0, 1, 2)), (19, 10, 1, (0, 26, 26)), (36, 20, 1, (26, 26, 26)),
           (20, 1, 1, (26, 1, 2)), (13, 1, 1, (0, 21, 6))]:
    n1 = n1_of(*s6[:3], *s6[3])
    for p in pfx:
        v = n37(p)
        for pw in (0, 3, 7, 10, 13, 17, 20, 30, 37, 40, 47, 50, 57, 60):
            for nadd in (1, 2, 3):
                nu = (pw + nadd) % 10
                if nadd > 2 and pw % 10 == 7:
                    continue
                n3 = v + 32768 * (nadd - 1)
                # lo15 = n3 & 32767, and nadd from ntype's last digit
                add(bits_of(n1, ((n3 & 32767) << 7) + (pw + nadd) + 64))
    # raw prefix field, including the >= 32400 half unpackgrid refuses and suffix values 60000..60125
    for lo in [0, 1, 36, 37, 1368, 1369, 50652 - 32768, 32399, 32400, 32767]:
        for ntype in (1, 2, 4, 5, 6, 8, 9, 11, 14, 18, 21, 31, 34, 48, 61, 62):
            add(bits_of(n1, (lo << 7) + ntype + 64))
for suffix_n in list(range(0, 126)):
    # 60000 + nc: n3 - 32768*(nadd-1) = (60000 + nc) - 32768*(nadd-1); nadd 2 -> 60000+nc-32768
    for nadd in (2,):
        n3 = 60000 + suffix_n
        lo = n3 - 32768 * (nadd - 1)
        if 0 <= lo < 32768:
            for pw in (0, 3, 7, 10, 23, 37, 60):
                add(bits_of(n1_of(20, 1, 1, 0, 1, 2), (lo << 7) + pw + nadd + 64))
for lo in range(0, 32768, 53):
    for nadd, pw in ((2, 10), (2, 37), (1, 0), (3, 23)):
        add(bits_of(n1_of(20, 1, 1, 0, 1, 2), (lo << 7) + pw + nadd + 64))

# ---- 4. type 3: hash, six-character grid in the callsign slot, all powers -------------------
grids6 = ["FN42LX", "JO33AB", "AA00AA", "RR99XX", "PM95SO", "IO91WM", "A000AA", "JO33", "JO33AB",
          "ZZ99ZZ", "FN4ZLX", "F342LX", "FNA2LX", "FN42LX", "FN42  ", "FN42L ", "FN42LZ", "FN4200",
          "AB12CD", "A1B2C3", "1A23BC", "      ", "FN    ", "F     ", "K1ABC "]
hashes = sorted(t3.items())[:6] + [(0x1234, None), (0x7FFF & 32399, None), (0, None)]
def pseudo_call_n1(g6):
    g6 = g6.ljust(6)
    call6 = g6[1:] + g6[0]                   # wqencode.f90:46: callsign = grid6(2:6)//grid6(1:1)
    if not call6[2].isdigit():
        call6 = " " + call6[:5]              # packcall's digit-in-slot-2 shift
    if not call6[2].isdigit() or call6[1] == " " or any(ch.isdigit() for ch in call6[3:]):
        return None
    return n1_text(call6)
for h, call in hashes:
    for g6 in grids6:
        n1 = pseudo_call_n1(g6)
        if n1 is None:
            continue
        for ndbm in (0, 3, 7, 10, 13, 17, 20, 27, 30, 37, 40, 47, 50, 57, 60, 1, 2, 4, 5, 8, 9, 11, 22, 33, 61, 62, 63):
            ntype = -(ndbm + 1)
            if ntype < -64:
                continue
            n2 = (h << 7) + ntype + 64
            add(bits_of(n1, n2))
            if call:
                add(bits_of(n1, n2), call)
        # ntype = -64 (ndbm 63) and the boundary just above it
        for nt in (-64, -63, -2, -1):
            add(bits_of(n1, (h << 7) + nt + 64))
            if call:
                add(bits_of(n1, (h << 7) + nt + 64), call)
# hashes >= 32400 (unpackgrid refuses) and the seed with a long / compound call
for h in (32399, 32400, 32767):
    add(bits_of(pseudo_call_n1("FN42LX"), (h << 7) + 64 - 28), "K1ABC")
# a seed that is a Type 2 compound call, as wsprd stores it
for call in ["PJ4/K1ABC", "K1ABC/7", "K1ABC/12", "3D0/K1ABC", "ABC/K1ABCD"]:
    add(bits_of(pseudo_call_n1("FN42LX"), (t3.get(0, 0) << 7) + 64 - 28), call)

# ---- 5. random payloads over the whole space -------------------------------------------------
for _ in range(2500):
    add(bits_of(rnd.randrange(262177560), rnd.randrange(1 << 22)))
for _ in range(500):
    add(bits_of(rnd.randrange(1 << 28), rnd.randrange(1 << 22)))
for _ in range(600):   # valid n1, grid < 32400, plausible ntype: where the interesting forms live
    ntype = rnd.choice(list(range(0, 63)) + list(range(-64, 0)))
    add(bits_of(rnd.randrange(262177560), (rnd.randrange(32400) << 7) + ntype + 64))
# ---- run the oracle -------------------------------------------------------------------------
inp = "".join(f"{b}\t{s}\n" for b, s in cases)
res = subprocess.run([f"{ordir}/wsprd_unpack_oracle"], input=inp, capture_output=True, text=True, check=True)
lines = res.stdout.split("\n")[:-1]
assert len(lines) == len(cases), (len(lines), len(cases))
with open(out, "w") as fh:
    fh.write(f"# msg::wspr::unpack / wspr decode text against WSJT-X's own `unpk_` (lib/wsprd/wsprd_utils.c) of v3.2.0-rc1 ({commit}).\n")
    fh.write("# Made by scripts/wsprd/gen_cases.sh from scripts/wsprd/wsprd_unpack_oracle.c, built with scripts/wsprd/build_oracle.sh;\n")
    fh.write("# regenerating it with the same oracle reproduces it byte for byte (lib/wsprd/ is identical in v3.3.0-beta2).\n")
    nprint = sum(1 for l in lines if l.startswith("0\t"))
    fh.write(f"# {len(cases)} cases: printed {nprint}, not printed {len(cases) - nprint}.\n")
    fh.write("# Columns: 50 bits, seed call (in wsprd's hash table; - = empty), rc (unpk_'s return = wsprd's noprint), text (the message field wsprd prints; - when rc is 1).\n")
    for (b, s), l in zip(cases, lines):
        fh.write(f"{b}\t{s}\t{l}\n")
print(f"wrote {out}: {len(cases)} cases")
PY
