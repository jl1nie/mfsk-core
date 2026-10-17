import random
import subprocess
import sys

oracle, out, commit = sys.argv[1:4]
rnd = random.Random(2026)

msgs = []


def add(m):
    if m not in msgs:
        msgs.append(m)


calls = ["K1ABC", "W9XYZ", "JA1ABC", "K9AN", "G4ABC", "VK3NV", "3Y0Z", "3DA0RS", "3XA0BC", "KA1ABC",
         "A1B", "R1ABC", "1A2BC", "9A1A", "LU1ABC", "W1ABCD", "K1ABCD", "OE21FTDMC", "E9ZZ", "CQ", "DE",
         "QRZ", "PJ4/K1ABC", "K1ABC/P", "K1ABC/QRP", "HB9/DL1ABC", "VE3/K1ABC", "DL1ABC/MM", "KH1/KH7Z",
         "JA/K1ABC", "K1ABC/VE3", "K1ABC/9", "K1ABC/A", "K1ABC/Z", "XX9/K1ABC", "AB/CD", "A/B", "P", "5", "Z",
         "ZS8/K1ABC", "VK9X/K1ABC", "K1ABC/1234", "ABCDE/K1ABC", "ABCD/K1ABC", "K1ABC/ABC", "K1ABC/AB"]
grids = ["FN42", "PM95", "JN58", "AA00", "RR99", "RR59", "RR50", "QR75", "-15", "-01", "-30", "-31", "R-05",
         "R-30", "R-31", "RO", "RRR", "73", "+05", "+00", "0", "5", "49", "50", "-35", "-50", "-51", "R+05",
         "R05", "R-50", "R49", "R50", "OOO", "fn42", "FN42AB", "FN4", "FN", "F", "KA50", "LA50", "XX99", "AA9",
         "0123", "-5", "R-5", "+5", "R+5"]

# 1. standard and CQ/QRZ/DE forms
for c in calls[:20]:
    for d in calls[:20]:
        if rnd.random() < 0.25:
            add(f"{c} {d} FN42")
for c in calls:
    for g in grids:
        if rnd.random() < 0.35:
            add(f"CQ {c} {g}")
        if rnd.random() < 0.2:
            add(f"K1ABC {c} {g}")
            add(f"{c} W9XYZ {g}")
for c in calls:
    for pre in ["CQ", "QRZ", "DE"]:
        add(f"{pre} {c}")
        add(f"{pre} {c} FN42")
for g in grids:
    add(f"K1ABC W9XYZ {g}")
    add(f"CQ K1ABC {g}")
    add(f"DE K1ABC {g}")
    add(f"QRZ K1ABC {g}")
    add(f"PJ4/K1ABC W9XYZ {g}")
    add(f"CQ PJ4/K1ABC {g}")
    add(f"CQ K1ABC/P {g}")
    add(f"CQ K1ABC/QRP {g}")
    add(f"DE K1ABC/QRP {g}")
    add(f"KH1/KH7Z W9XYZ {g}")

# 2. CQ variants
for t in ["CQ DX K1ABC FN42", "CQ DX K1ABC", "CQ NA K1ABC FN42", "CQ EU K1ABC", "CQ AB K1ABC FN42",
          "CQ ABC K1ABC FN42", "CQ A K1ABC", "CQ 1 K1ABC FN42", "CQ 5 K1ABC FN42", "CQ 12 K1ABC",
          "CQ 123 K1ABC FN42", "CQ 000 K1ABC FN42", "CQ 007 K1ABC", "CQ 010 K1ABC", "CQ 999 K1ABC FN42",
          "CQ 1234 K1ABC", "CQ 12A K1ABC", "CQ K1ABC", "CQ  K1ABC", "cq k1abc fn42", "CQ DE K1ABC",
          "CQ DX", "CQ NA", "CQ TEST K1ABC FN42", "CQ POTA K1ABC FN42", "CQ DX PJ4/K1ABC", "CQ NA PJ4/K1ABC",
          "CQ 5 PJ4/K1ABC", "CQ DX K1ABC/P", "CQ 123 K1ABC/P", "QRZ DX K1ABC", "CQ K1ABC R FN42",
          "CQ CQ CQ", "K1ABC CQ", "K1ABC DE", "K1ABC QRZ FN42", "DE CQ", "CQ9DX K1ABC FN42", "E9AB K1ABC",
          "E9AB K1ABC FN42", "E9A K1ABC", "CQ 9 K1ABC FN42"]:
    add(t)

# 3. words after the grid, blanks, case
for t in ["K1ABC W9XYZ FN42 EXTRA", "K1ABC W9XYZ FN42 73", "K1ABC W9XYZ OOO", "K1ABC W9XYZ FN42 OOO",
          "  K1ABC W9XYZ FN42", " K1ABC W9XYZ FN42", "K1ABC  W9XYZ   FN42", "K1ABC W9XYZ", "K1ABC", "K1ABC ",
          "k1abc w9xyz fn42", "K1ABC w9xyz Fn42", "K1ABC W9XYZ FN42 ", "K1ABC W9XYZ FN4", "K1ABC W9XYZ FN42AB"]:
    add(t)

# 4. free text of every length, alphabet and beyond it
base = "HELLO WORLD 12345 ABCDEFGH"
for n in range(0, 27):
    add(base[:n])
for t in ["TNX 73 GL", "73", "RRR", "RO", "TEST", "A", "1", "-", "+", ".", "/", "?", "  ", "HELLO", "hello",
          "Hello World", "HELLO, WORLD", "HELLO! WORLD", "A+B-C.D/E?", "A+B-C.D/E?F", "A+B-C.D/E?FG",
          "0123456789ABC", "0123456789ABCD", "0123456789ABCDEFGHIJKL", "0123456789ABCDEFGHIJKLM",
          "ZZZZZZZZZZZZZ", "?????????????", "             X", "X             ", "A   B   C", "#*@%&", "A#B*C@D",
          "HI HI HI", "TNX QSO 73 DE K1ABC", "CQ CQ CQ DE K1ABC K1ABC", "K1ABC W9XYZ FN42 K1ABC W9XYZ FN",
          "~~~~", "[]{}", "A\"B", "TEXT WITH 22 CHARS ABC", "TEXT WITH 23 CHARS ABCD", "TEXT WITH 24 CHARS ABCDE",
          "K1ABC/P", "PJ4/K1ABC", "PJ4/K1ABC W9XYZ", "K1ABC W9XYZ/P", "K1ABC/P W9XYZ", "K1ABC/P W9XYZ/P",
          "PJ4/K1ABC PJ4/W9XYZ", "A/B C/D", "// //", "K1ABC/ W9XYZ", "/K1ABC W9XYZ", "K1ABC //", "K1ABC /W9XYZ"]:
    add(t)

# 5. random compositions
words = ["CQ", "DE", "QRZ", "K1ABC", "W9XYZ", "PJ4/K1ABC", "K1ABC/P", "FN42", "-15", "R-05", "RRR", "73", "RO",
         "HELLO", "DX", "NA", "123", "OOO", "JA1ABC", "VK3NV", "3DA0RS", "3XA0BC", "ZS8/K1ABC", "K1ABC/QRP", "+05"]
for _ in range(400):
    add(" ".join(rnd.choice(words) for _ in range(rnd.randint(1, 5))))

# 6. every 3rd-token shape: all 1-4 character strings over a small alphabet
alpha = list("-+R05 9A1.,/*E")
toks = set()
for n in range(1, 5):
    for _ in range(300):
        toks.add("".join(rnd.choice(alpha) for _ in range(n)))
for t in sorted(toks):
    if t.strip():
        add(f"K1ABC W9XYZ {t}")
        if len(msgs) % 3 == 0:
            add(f"CQ K1ABC {t}")

# U rows: word patterns
words_rows = []


def w(nc1, nc2, ng):
    d = [
        (nc1 >> 22) & 63, (nc1 >> 16) & 63, (nc1 >> 10) & 63, (nc1 >> 4) & 63,
        4 * (nc1 & 15) + ((nc2 >> 26) & 3), (nc2 >> 20) & 63, (nc2 >> 14) & 63, (nc2 >> 8) & 63,
        (nc2 >> 2) & 63, 16 * (nc2 & 3) + ((ng >> 12) & 15), (ng >> 6) & 63, ng & 63]
    words_rows.append(d)


NBASE = 37 * 36 * 10 * 27 ** 3
k1abc = 0
for nc1 in [0, 1, NBASE - 1, NBASE, NBASE + 1, NBASE + 2, NBASE + 3, NBASE + 4, NBASE + 1002, NBASE + 1003,
            262178563, 262178564, 263000000, 264002071, 264002072, 265825580, 265825581, 267649089, 267649090,
            267698374, 267698375, 267747659, 267747660, 267796944, 267796945, 267796946, 268435455,
            rnd.randrange(1 << 28)]:
    for nc2 in [0, 12345678, NBASE - 1, NBASE, NBASE + 5, 267796945, 268435455]:
        for ng in [0, 22632, 32399, 32400, 32401, 32431, 32432, 32461, 32462, 32463, 32464, 32465, 32767, 32768,
                   40000, 65535, 179, 180, 15258, 5, 17999, 100, 9, 1]:
            if rnd.random() < 0.3:
                w(nc1, nc2, ng)
# every ng in the location range, with a normal pair of calls
base1 = 0x2C0A7B1
for ng in range(0, 32400, 7):
    if rnd.random() < 0.05:
        w(base1, 4000000, ng)
for ng in range(32390, 32800):
    w(5000000, 6000000, ng)
# grids that read as locations (nlat >= 85): RR59.. and the whole k range
for fl in range(18):
    for sl in range(10):
        for sla in (5, 9):
            ng = (179 - 10 * fl - sl) * 180 + 10 * 17 + sla
            w(5000000, 6000000, ng)
            w(NBASE + 1002 + 1 + 5, 6000000, ng)
for _ in range(500):
    d = [rnd.randrange(64) for _ in range(12)]
    words_rows.append(d)
for _ in range(300):
    d = [rnd.randrange(64) for _ in range(12)]
    d[9] |= 8  # free-text bit
    words_rows.append(d)
# free text with the odd/even bits and out-of-range 42-ary digits
for _ in range(100):
    nc1 = rnd.randrange(1 << 28)
    nc2 = rnd.randrange(1 << 28)
    w(nc1, nc2, 32768 + rnd.randrange(32768))

lines = []
for m in msgs:
    lines.append("P\t" + m)
for d in words_rows:
    lines.append("U\t" + " ".join(map(str, d)))
res = subprocess.run([oracle], input="\n".join(lines) + "\n", capture_output=True, text=True, check=True)
rows = res.stdout.split("\n")
if rows and rows[-1] == "":
    rows.pop()
assert len(rows) == len(lines), (len(rows), len(lines))

with open(out, "w") as f:
    f.write("# msg::jt72 pack_message / unpack against WSJT-X's own packmsg / unpackmsg (lib/packjt.f90) of v3.2.0-rc1 (%s).\n" % commit)
    f.write("# Made by scripts/jt72/gen_cases.sh (driver scripts/jt72/jt72_oracle.f90, built with scripts/jt72/build_oracle.sh);\n")
    f.write("# regenerating it with the same oracle reproduces it byte for byte. %d P rows (message -> itype, words, text) and\n" % len(msgs))
    f.write("# %d U rows (twelve words -> text). Columns: kind, message-or-words, itype ('-' for U), twelve words, text.\n" % len(words_rows))
    f.write("# The message column of a P row is the input as given (the first 22 characters count, as for packmsg's character*22);\n")
    f.write("# in the text column a NUL prints as the two characters \\0.\n")
    for line, row in zip(lines, rows):
        kind, rest = line.split("\t", 1)
        f.write(kind + "\t" + rest + "\t" + row.split("\t", 1)[1] + "\n")
print("wrote", out, len(msgs), "P rows,", len(words_rows), "U rows")
