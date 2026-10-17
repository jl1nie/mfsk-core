#!/usr/bin/env bash
# Golden cases for msg::wsjt77::pack77_message: a deterministic list of messages, each run through
# WSJT-X's own pack77 (scripts/pack77/pack77_oracle.f90, built from the v3.2.0-rc1 tag by
# scripts/pack77/build_oracle.sh).
#
#   scripts/pack77/gen_cases.sh [oracle] [out-file] [WSJT-X-dir]
#
#   oracle      target/pack77-oracle/pack77_oracle
#   out-file    mfsk-core/tests/fixtures/pack77_rc1.tsv
#   WSJT-X-dir  ../WSJT-X (for lib/77bit/messages.txt of the same tag, read through `git show`)
#
# The cases are rc1's own lib/77bit/messages.txt (filtered the way encode77.f90 filters it), curated
# cases for every candidate of pack77_encode_core (0.1 DXpedition, 0.3/0.4 Field Day, 0.5 telemetry,
# 0.6 WSPR forms, 1/2, 3, 4, 5, free text), the edges of each field's grammar, malformed messages,
# and seeded random compositions of the tokens the packer treats specially. Seeded, so the file is
# reproducible byte for byte from the same oracle.
#
# Rows with prefer_wspr_50bit set are kept for the cases of up to three words, the only ones that
# option can touch.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ORACLE="${1:-$REPO_ROOT/target/pack77-oracle/pack77_oracle}"
OUT="${2:-$REPO_ROOT/mfsk-core/tests/fixtures/pack77_rc1.tsv}"
WSJTX="${3:-$REPO_ROOT/../WSJT-X}"
TAG="${PACK77_TAG:-v3.2.0-rc1}"
[ -x "$ORACLE" ] || { echo "no $ORACLE - run scripts/pack77/build_oracle.sh first" >&2; exit 1; }
COMMIT="$(cat "$(dirname "$ORACLE")/exported-commit" 2>/dev/null || echo unknown)"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
git -C "$WSJTX" show "$TAG:lib/77bit/messages.txt" > "$TMP/messages.txt"

python3 -I - "$ORACLE" "$OUT" "$COMMIT" "$TAG" "$TMP" <<'PY'
import random, subprocess, sys

oracle, out, commit, tag, tmp = sys.argv[1:6]
rnd = random.Random(2026)

cases = []
def add(m):
    if m not in cases:
        cases.append(m)

# 1. messages.txt, the way encode77.f90 reads it: stop at '$', skip blanks, 'N.' headings and '---'
for raw in open(tmp + "/messages.txt", encoding="ascii").read().split("\n"):
    m = raw.lstrip(" ")
    if m.startswith("$"):
        break
    if not m or m[1:2] == "." or m[2:3] == "." or m.startswith("---"):
        continue
    add(m)
n_messages_txt = len(cases)

# 2. curated
calls = ["K1ABC", "W9XYZ", "JA1ABC", "3Y0Z", "G4ABC", "PA9XYZ", "KA1ABC", "A1B", "R1ABC", "QA1BC", "1A2BC",
         "9A1A", "VK3NV", "LU1ABC", "K1ABC/R", "K1ABC/P", "W9XYZ/P", "W9XYZ/R", "PJ4/K1ABC", "KH1/KH7Z",
         "K1ABC/QRP", "YW18FIFA", "3DA0RS", "W1ABCD", "K1ABCD", "K1ABC/VE3", "OE21FTDMC", "HB9/DL1ABC"]
for t in ["", " ", "   CQ K1ABC FN42", "cq k1abc fn42", " CQ K1ABC FN42", "CQ  K1ABC   FN42  ", "K1ABC W9XYZ", "K1ABC W9XYZ ",
          "K1ABC", "CQ", "DE", "QRZ", "CQ K1ABC", "DE K1ABC FN42", "QRZ K1ABC FN42", "DE K1ABC RRR", "QRZ K1ABC 73",
          "CQ DX K1ABC FN42", "CQ POTA K1ABC FN42", "CQ TEST K1ABC FN42", "CQ 123 K1ABC FN42", "CQ 12 K1ABC FN42",
          "CQ 1234 K1ABC FN42", "CQ ABCDE K1ABC FN42", "CQ NA K1ABC", "CQ K1ABC R FN42", "CQ K1ABC -10", "CQ K1ABC RRR",
          "CQ K1ABC RR73", "CQ K1ABC 73", "CQ K1ABC/P FN42", "CQ K1ABC/R FN42", "CQ K1ABC/P", "CQ_DX K1ABC FN42",
          "CQ_POTA K1ABC", "CQ <K1ABC> FN42", "CQ <K1ABC>", "CQ K1ABC FN42 73", "CQ SOTA W9XYZ/P FN42",
          "CQ DX PJ4/K1ABC", "CQ DX YW18FIFA", "CQ 123 PJ4/K1ABC"]:
    add(t)
for a in ["K1ABC", "K1ABC/R", "K1ABC/P", "<K1ABC>", "<PJ4/K1ABC>", "<...>", "PJ4/K1ABC", "3DA0RS", "QA1BC", "W1ABCD"]:
    for b in ["W9XYZ", "W9XYZ/R", "W9XYZ/P", "<W9XYZ>", "<...>", "KH1/KH7Z", "YW18FIFA", "<YW18FIFA>"]:
        for tail in ["", "FN42", "R FN42", "RR73", "RRR", "73", "-11", "R-09", "+05", "R+15", "-50", "-51", "+50", "+51", "R-30"]:
            if rnd.random() < 0.18:
                add(" ".join(x for x in [a, b, tail] if x))
for r in list(range(-52, 53, 3)) + [-35, -34, -33, -32, -31, -30, 49, 50, 51, 0]:
    add("K1ABC W9XYZ %+03d" % r)
    add("W9XYZ K1ABC R%+03d" % r)
for t in ["K1ABC W9XYZ -1", "K1ABC W9XYZ +5", "K1ABC W9XYZ -123", "K1ABC W9XYZ R-1", "K1ABC W9XYZ R 73", "K1ABC W9XYZ R", "K1ABC W9XYZ RR",
          "K1ABC W9XYZ R73", "K1ABC W9XYZ fn42", "K1ABC W9XYZ FN4", "K1ABC W9XYZ FN421", "K1ABC W9XYZ SS00", "K1ABC W9XYZ RR99", "K1ABC W9XYZ AA00",
          "K1ABC/R W9XYZ/P FN42", "K1ABC/P W9XYZ/R FN42", "K1ABC/P W9XYZ/P FN42", "K1ABC/R W9XYZ/R FN42", "K1ABC/ W9XYZ", "/R K1ABC W9XYZ",
          "K1ABC/RR W9XYZ", "K1ABC/P/R W9XYZ", "<K1ABC>/P W9XYZ", "<K1ABC/P> W9XYZ FN42", "<K1ABC> W9XYZ/P FN42", "<K1ABC> <W9XYZ> FN42",
          "<K1ABC> <W9XYZ>", "<K1ABC", "K1ABC>", "<> W9XYZ", "<K> W9XYZ", "<K1> W9XYZ", "<K1A> W9XYZ", "<<K1ABC>> W9XYZ", "<K1ABC> W9XYZ RR73",
          "W9XYZ <K1ABC> RR73", "<ABCDEFGHIJK> W9XYZ -11", "<ABCDEFGHIJKL> W9XYZ -11", "<K1ABC-1> W9XYZ -11", "<K1ABC/1> W9XYZ", "<k1abc> w9xyz -11",
          "k1abc w9xyz -11", "K1ABC  W9XYZ   -11", "AB1 CD1 -11", "ABC DEF GHI", "TNX BOB 73 GL"]:
    add(t)

# DXpedition 0.1
for t in ["K1ABC RR73; W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; W9XYZ <KH1/KH7Z> +00", "K1ABC RR73; W9XYZ <KH1/KH7Z> -00",
          "K1ABC RR73; W9XYZ <KH1/KH7Z> -30", "K1ABC RR73; W9XYZ <KH1/KH7Z> -31", "K1ABC RR73; W9XYZ <KH1/KH7Z> -09",
          "K1ABC RR73; W9XYZ <KH1/KH7Z> +32", "K1ABC RR73; W9XYZ <KH1/KH7Z> +33", "K1ABC RR73; W9XYZ <KH1/KH7Z> +31",
          "K1ABC RR73; W9XYZ <KH1/KH7Z> 08", "K1ABC RR73; W9XYZ <KH1/KH7Z> -8", "K1ABC RR73; W9XYZ <KH1/KH7Z> -008",
          "K1ABC RR73; W9XYZ KH1/KH7Z -08", "K1ABC RR73; W9XYZ <...> -08", "K1ABC RR73; W9XYZ <KH> -08", "K1ABC RR73; W9XYZ <KH7Z> -08",
          "K1ABC RR73 W9XYZ <KH1/KH7Z> -08", "K1ABC RR73;W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; W9XYZ <KH1/KH7Z>", "K1ABC RR73; W9XYZ <KH1/KH7Z> -08 X",
          "k1abc rr73; w9xyz <kh1/kh7z> -08", "K1ABC/P RR73; W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; W9XYZ/P <KH1/KH7Z> -08",
          "PJ4/K1ABC RR73; W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; PJ4/K1ABC <KH1/KH7Z> -08", "KH1/KH7Z RR73; PJ4/K1ABC <W9XYZ> +20",
          "<K1ABC> RR73; W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; <W9XYZ> <KH1/KH7Z> -08", "CQ RR73; W9XYZ <KH1/KH7Z> -08",
          "K1ABC RR73; CQ <KH1/KH7Z> -08", "AB RR73; W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; W9XYZ <KH1/KH7Z/AB> -08",
          "ABCDEFGHIJK RR73; ABCDEFGHIJK <ABCDEFGHIJK> -08", "ABCDEFGHI RR73; ABCDEFGHI <ABCDEFGHIJK> -08",
          "ABCDEFGHIJKL RR73; W9XYZ <KH1/KH7Z> -08", "K1ABC RR73; W9XYZ <KH1/KH7Z> -30", "W9XYZ RR73; K1ABC <PJ4/K1ABC> +14",
          "K1ABC RR73; W9XYZ <K1ABC> -08", "K1ABC RR73; K1ABC <K1ABC> -08", "K1ABC RR73; W9XYZ <W9XYZ> -08"]:
    add(t)

# Field Day 0.3 / 0.4
secs = ["AB","AK","AL","AR","AZ","BC","CO","CT","DE","EB","EMA","ENY","EPA","EWA","GA","GH","IA","ID","IL","IN","KS","KY","LA","LAX",
        "NS","MB","MDC","ME","MI","MN","MO","MS","MT","NC","ND","NE","NFL","NH","NL","NLI","NM","NNJ","NNY","TER","NTX","NV","OH","OK",
        "ONE","ONN","ONS","OR","ORG","PAC","PR","QC","RI","SB","SC","SCV","SD","SDG","SF","SFL","SJV","SK","SNJ","STX","SV","TN","UT",
        "VA","VI","VT","WCF","WI","WMA","WNY","WPA","WTX","WV","WWA","WY","DX","PE","NB"]
for s in secs:
    n = rnd.choice([1, 2, 6, 10, 16, 17, 25, 32])
    cl = rnd.choice("ABCDEF")
    r = rnd.choice(["", "R "])
    add("WA9XYZ KA1ABC %s%d%s %s" % (r, n, cl, s))
for t in ["WA9XYZ KA1ABC R 16A EMA", "WA9XYZ KA1ABC R 32A EMA", "WA9XYZ KA1ABC R 33A EMA", "WA9XYZ KA1ABC R 0A EMA", "WA9XYZ KA1ABC R 00A EMA",
          "WA9XYZ KA1ABC R 01A EMA", "WA9XYZ KA1ABC R 1A EMA", "WA9XYZ KA1ABC R 1F EMA", "WA9XYZ KA1ABC R 1G EMA", "WA9XYZ KA1ABC R 1a EMA",
          "WA9XYZ KA1ABC R 1A EMAX", "WA9XYZ KA1ABC R 1A EM", "WA9XYZ KA1ABC R 1A XYZ", "WA9XYZ KA1ABC R 1A ema", "WA9XYZ KA1ABC 1A EMA",
          "WA9XYZ KA1ABC RR 1A EMA", "WA9XYZ KA1ABC Q 1A EMA", "WA9XYZ KA1ABC R A EMA", "WA9XYZ KA1ABC R 1 EMA", "WA9XYZ KA1ABC R 100A EMA",
          "WA9XYZ KA1ABC 6A", "K1ABC W9XYZ 6A WI", "K1ABC/P W9XYZ 3A EMA", "K1ABC W9XYZ/P 3A EMA", "<K1ABC> W9XYZ 3A EMA", "K1ABC W9XYZ R 3A EMA X",
          "PJ4/K1ABC W9XYZ R 3A EMA", "PJ4/K1ABC YW18FIFA R 3A EMA", "ABCDEFGHIJK ABCDEFGHI 12A EMA", "ABCDEFGHIJK ABCDEFGHIJK R 12A EMA",
          "ABCDEFGHI ABCDEFGHI R 12A EMA", "AB W9XYZ 3A EMA", "CQ FD K1ABC FN42", "CQ FD K1ABC", "WA9XYZ KA1ABC R 16A EMA 73",
          "WA9XYZ KA1ABC R 9A EMA", "WA9XYZ KA1ABC 10A DX", "WA9XYZ KA1ABC R 32F PE", "WA9XYZ KA1ABC 17B NB", "QRZ KA1ABC 3A EMA", "DE KA1ABC 3A EMA"]:
    add(t)

# Telemetry 0.5
for t in ["1", "A", "F", "0", "00", "123456789ABCDEF012", "7FFFFFFFFFFFFFFFFF", "800000000000000000", "7FFFFF123456789ABC", "FFFFFF123456789ABC",
          "123456789ABCDEF0123", "123456789ABCDEF01", "123456789ABCDEF0", "123456789ABCDE", "123456789ABCD", "123456789ABC", "1234567",
          "abcdef", "ABCDEF", "abcdefabcdefabcdef", "G", "12345G", "123456789ABCDEFG", "1234 5678", "DEAD", "BEEF", "CAFE", "FACE", "BAD", "ACE", "ADD",
          "DEC", "FEED", "BEAD", "FED", "FACADE", "DECADE", "0000000000000000A", "000000000000000000", "0000000000000000000", "1 2", " 1", "1 ",
          "ABCDEF ABCDEF", "123456789ABCDEF 0", "FFFFFFFFFFFFFFFFFF", "7FFFFFFFFFFFFFFFFE", "400000", "7FFFFF", "800000", "FFFFFF"]:
    add(t)

# EU VHF 5
for t in ["<PA9XYZ> <G4ABC/P> 590003 IO91NP", "<G4ABC/P> <PA9XYZ> R 570007 JO22DB", "<W3CCX> <K1JT> 590001 FN20QI", "<W3CCX/QRP> <K1JT/QRO> 590001 FN20QI",
          "<W3CCX> <K1JT> 590000 FN20QI", "<W3CCX> <K1JT> 592047 FN20QI", "<W3CCX> <K1JT> 592048 FN20QI", "<W3CCX> <K1JT> 520001 FN20QI",
          "<W3CCX> <K1JT> 510001 FN20QI", "<W3CCX> <K1JT> 600001 FN20QI", "<W3CCX> <K1JT> 59001 FN20QI", "<W3CCX> <K1JT> 5900001 FN20QI",
          "<W3CCX> <K1JT> 590001 FN20", "<W3CCX> <K1JT> 590001 FN20Q", "<W3CCX> <K1JT> 590001 FN20QIA", "<W3CCX> <K1JT> 590001 FN20YA",
          "<W3CCX> <K1JT> 590001 FN20XX", "<W3CCX> <K1JT> 590001 SS20QI", "<W3CCX> <K1JT> 590001 fn20qi", "<W3CCX> <K1JT> R 590001 FN20QI",
          "<W3CCX> <K1JT> Q 590001 FN20QI", "<W3CCX> K1JT 590001 FN20QI", "W3CCX <K1JT> 590001 FN20QI", "W3CCX K1JT 590001 FN20QI",
          "<...> <K1JT> 590001 FN20QI", "<W3CCX> <...> 590001 FN20QI", "<...> <...> 590001 FN20QI", "<W3CCX> <K1JT> 590001 FN20QI 73",
          "<AB> <K1JT> 590001 FN20QI", "<ABC> <K1JT> 590001 FN20QI", "<ABCDEFGHIJK> <ABCDEFGHIJK> R 590001 FN20QI", "<W3CCX> <W3CCX> 590001 FN20QI",
          "<W3CCX> <K1JT> 530100 AA00AA", "<W3CCX> <K1JT> R 580100 RR99XX", "<W3CCX> <K1JT> 599999 FN20QI", "<W3CCX> <K1JT> 592047 JJ00AA",
          "<W3CCX> <K1JT> 520001 PM95SR", "<W3CCX> <K1JT> R 5900031 FN20QI", "<W3CCX> <K1JT> 590003 IO91NP", "<PA3XYZ> <G4ABC/P> R 590003 IO91NP"]:
    add(t)

# Type 3 RTTY
mults = ["AL","CA","MA","NWT","PEI","ZL","X01","X50","X99","DC","ON","VT","NJ","NY","TX"]
for m in mults:
    add("K1ABC W9XYZ %d%s9 %s" % (rnd.choice([5]), rnd.choice("23456789"), m))
for t in ["K1ABC W9XYZ 579 0001", "K1ABC W9XYZ 579 7999", "K1ABC W9XYZ 579 8000", "K1ABC W9XYZ 579 0000", "K1ABC W9XYZ 579 8001",
          "K1ABC W9XYZ 579 001", "K1ABC W9XYZ 579 00013", "K1ABC W9XYZ 579 13", "K1ABC W9XYZ R 579 0013", "TU; K1ABC W9XYZ 579 0013",
          "TU; K1ABC W9XYZ R 579 MA", "TU K1ABC W9XYZ R 579 MA", "TU; TU; K1ABC W9XYZ 579 MA", "K1ABC W9XYZ 519 MA", "K1ABC W9XYZ 529 MA",
          "K1ABC W9XYZ 599 MA", "K1ABC W9XYZ 5A9 MA", "K1ABC W9XYZ 570 MA", "K1ABC W9XYZ 57 MA", "K1ABC W9XYZ 5799 MA", "K1ABC W9XYZ 579 ma",
          "K1ABC W9XYZ 579 MAA", "K1ABC W9XYZ 579 M", "K1ABC W9XYZ 579 Y01", "K1ABC W9XYZ 579 X00", "K1ABC W9XYZ 579 X100", "K1ABC W9XYZ 579",
          "K1ABC W9XYZ R 579", "K1ABC W9XYZ R R 579 MA", "K1ABC W9XYZ 579 MA 73", "TU; KA0DEF K1ABC R 569 MA", "TU; G3AAA K1ABC R 559 MA",
          "<K1ABC> W9XYZ 579 MA", "K1ABC <W9XYZ> 579 MA", "<K1ABC> <W9XYZ> 579 MA", "<...> W9XYZ 579 MA", "PJ4/K1ABC W9XYZ 579 MA",
          "PJ4/K1ABC YW18FIFA R 579 0013", "TU; PJ4/K1ABC YW18FIFA R 579 0013", "TU; ABCDEFGHIJK ABCDEFGHIJK R 579 0013", "ABCDEFGHIJK ABCDEFGHIJK R 579 0013",
          "TU; ABCDEFGHIJK ABCDEFGHI R 579 0013", "ABCDEFGHIJK ABCDEFGHI 579 0013", "TU; <ABCDEFGHIJK> ABCDEFGHI 579 0013", "K1ABC/P W9XYZ 579 MA",
          "K1ABC/R W9XYZ/P R 579 MA", "CQ K1ABC 579 MA", "DE K1ABC 579 MA", "CQ TEST K1ABC FN42", "TU; DE K1ABC 579 MA", "k1abc w9xyz r 579 ma",
          "tu; k1abc w9xyz r 579 ma", "TU; K1ABC W9XYZ R 579 0013 X"]:
    add(t)

# Type 4
for t in ["CQ PJ4/K1ABC", "PJ4/K1ABC <W9XYZ>", "<W9XYZ> PJ4/K1ABC", "W9XYZ <PJ4/K1ABC> -11", "<PJ4/K1ABC> W9XYZ R-09", "<W9XYZ> PJ4/K1ABC RRR",
          "PJ4/K1ABC <W9XYZ> 73", "<W9XYZ> PJ4/K1ABC RR73", "<W9XYZ> PJ4/K1ABC 73", "<W9XYZ> PJ4/K1ABC -11", "<W9XYZ> PJ4/K1ABC FN42", "<W9XYZ> PJ4/K1ABC R",
          "<W9XYZ> YW18FIFA", "<YW18FIFA> W9XYZ -11", "W9XYZ <YW18FIFA> R-09", "YW18FIFA <W9XYZ> RRR", "<W9XYZ> YW18FIFA 73", "CQ YW18FIFA", "CQ OE21FTDMC",
          "<YW18FIFA> KA1ABC", "KA1ABC <YW18FIFA> -11", "<YW18FIFA> KA1ABC R-17", "<KA1ABC> YW18FIFA RR73", "<YW18FIFA> KA1ABC 73", "<PJ4/K1ABC> FK52UD",
          "<K1ABC/W4> FK52UD", "CQ K1ABC/W4", "CQ K1ABC/P", "CQ <PJ4/K1ABC>", "CQ PJ4/K1ABC 73", "CQ DX PJ4/K1ABC", "<W9XYZ> <PJ4/K1ABC>", "<...> PJ4/K1ABC",
          "PJ4/K1ABC <...>", "<W9XYZ> ABCDEFGHIJK", "<W9XYZ> ABCDEFGHIJKL", "<W9XYZ> ABCDE", "<W9XYZ> ABCD", "<W9XYZ> A/B", "<W9XYZ> AB/CD/EF", "<W9XYZ> /ABCDEF",
          "<W9XYZ> ABCDEF/", "CQ ABCD", "CQ ABCDE", "CQ ABCDEFGHIJK", "CQ ABCDEFGHIJKL", "CQ 12345", "CQ /ABCDE", "<AB> PJ4/K1ABC", "<ABC> PJ4/K1ABC",
          "<W9XYZ> pj4/k1abc", "<w9xyz> pj4/k1abc rr73", "<W9XYZ> PJ4/K1ABC 73 X", "<W9XYZ> PJ4/K1ABC RRR 73", "K1ABC <PJ4/K1ABC>", "<K1ABC> W9XYZ/QRP", "W9XYZ/QRP <K1ABC>",
          "<K1ABC> KH1/KH7Z", "KH1/KH7Z <K1ABC> RR73", "<PJ4/K1ABC> <KH1/KH7Z>", "<K1ABC> 3DA0RS", "3DA0RS <K1ABC> 73", "<K1ABC> W1ABCD", "W1ABCD <K1ABC>",
          "<K1ABC> QA1BC", "<K1ABC> A1B", "<K1ABC> 1A2BC", "CQ 3DA0RS", "CQ 1A2BC", "CQ QA1BC", "CQ W1ABCD", "CQ HB9/DL1ABC", "<K1ABC> HB9/DL1ABC"]:
    add(t)

# WSPR 0.6 forms
dbms = [str(i) for i in range(0, 62)] + ["05", "007", "-1", "3.5", "abc", "1e1", "+10", "100"]
for d in dbms:
    add("K1ABC FN42 " + d)
for g in ["FN42", "PM95", "AA00", "RR99", "SS00", "IO91", "fn42", "FN4", "FN421", "RA00", "AR99", "0042"]:
    add("W9XYZ %s 37" % g)
for c in ["K1ABC", "JA1XYZ", "W9XYZ", "3Y0Z", "A1B", "QA1BC", "K1ABCD", "3DA0RS", "K1ABC/P", "PJ4/K1ABC", "<K1ABC>"]:
    add(c + " FN42 37")
for t in ["PJ4/K1ABC 37", "K1ABC/VE3 37", "KA1ABC/VEX 37", "K1ABC/1 37", "K1ABC/12 37", "K1ABC/123 37", "K1ABC/1234 37", "K1ABC/P 37", "K1ABC/R 37",
          "K1ABC/QRP 37", "VP9/K1ABC 37", "3DA/K1ABC 37", "A/K1ABC 37", "AB/K1ABC 37", "0K/K1ABC 37", "ZZZ/K1ABC 37", "ABCD/K1ABC 37", "/K1ABC 37", "K1ABC/ 37",
          "PJ4/K1ABC 7", "PJ4/K1ABC 70", "PJ4/K1ABC 03", "PJ4/K1ABC 60", "PJ4/K1ABC 61", "PJ4/K1ABC 5", "PJ4/K1ABC 10", "PJ4/K1ABC 33", "PJ4/K1ABC 100", "PJ4/K1ABC",
          "PJ4/K1ABC/P 37", "PJ4/ABCDEF 37", "PJ4/K1ABCDE 37", "OK/K1ABC 37", "K1ABC/ABC 37", "K1ABC/AB1 37", "K1ABC/A1B 37", "K1ABC/1AB 37", "K1ABC/111 37",
          "<PJ4/K1ABC> FK52UD", "<K1ABC/W4> FK52UD", "<K1ABC> FN42", "<K1ABC> FN42AB", "<K1ABC> FN42XX", "<K1ABC> FN42YA", "<K1ABC> FN42aa", "<K1ABC> FN42A",
          "<K1ABC> FN42AB1", "<K1ABC> FN4", "<K1ABC> fn42ab", "<K1ABCDEFGHI> FN42AB", "<K1ABCDEFGHIJ> FN42AB", "<K1> FN42AB", "<K1A> FN42AB", "<...> FN42AB",
          "K1ABC FN42AB", "<K1ABC>FN42AB", "<PJ4/K1ABC> FN42", "<PJ4/K1ABC> PM95SR", "<PJ4/K1ABC> AA00AA", "<PJ4/K1ABC> RR99XX",
          "K1ABC FN42 37 X", "FN42 K1ABC 37", "K1ABC 37 FN42", "37 K1ABC"]:
    add(t)

# free text edges
alphabet = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ+-./?"
for n in range(0, 17):
    add("".join(rnd.choice(alphabet.strip()) for _ in range(n)))
for t in ["TNX BOB 73 GL", "73 DE K1ABC", "TNX 73 GL", "HELLO WORLD", "HELLO WORLD 12", "HELLO WORLD 123", "HELLO WORLD 1234", "ABCDEFGHIJKLM", "ABCDEFGHIJKLMN",
          "A B C D E F G", "A B C D E F G H", "THIS IS FAR TOO LONG TO SEND", "+-./?", "0123456789+-.", "0123456789+-./", "lowercase text", "Mixed Case 73",
          "NO COMMA, HERE", "SEMI;COLON", "COLON:", "AT@SIGN", "HASH#", "STAR*", "UNDER_SCORE", "TILDE~", "QUOTE'", "PERCENT%", "AMP&", "PAREN(X)", "EQ=", "BANG!", "$DX",
          "$DXCALL", "$DX K1ABC", "TNX\tBOB", "TAB\tTAB", "A" * 14, "A" * 13, "A" * 12, "A" * 20, "A" * 37, "A" * 38, "A" * 40, "A" * 60,
          "73 73 73 73 73 73 73 73 73 73 73 73 73 73 73 73 73 73 73 73", "1 2 3 4 5 6 7 8 9 A B C D E F G H I J K L",
          "1 2 3 4 5 6 7 8 9 A B C D E F G H I J", "1 2 3 4 5 6 7 8 9 A B C D", "A B C D E F G H I J K L M N O P Q R S",
          "A B C D E F G H I J K L M N O P Q R S T", "ABCDEFGHIJKLMN 1", "ABCDEFGHIJKLMN", "1234567890123456", "123456789012345678", "1234567890123456789",
          "CQ 123456789ABCDEF", "CQ ABCDEFGHIJKLMNOP", "DE ABCDEFGHIJKLMNOP", "QRZ 123456789ABCDEF", "CQ ABCDEFGHIJKLMNOPQRSTUVWXYZ", "K1ABC W9XYZ FN42 73",
          "K1ABC W9XYZ FN42 73 GL", "THE QUICK BROWN FOX JUMPS", "THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG TWICE"]:
    add(t)

# seeded random compositions
tok = ["CQ", "DE", "QRZ", "TU;", "R", "RR73", "RR73;", "RRR", "73", "FN42", "FN42AB", "IO91NP", "-11", "+05", "R-09", "579", "0013", "MA", "EMA", "6A",
       "16A", "32B", "590003", "<K1ABC>", "<PJ4/K1ABC>", "<...>", "K1ABC", "W9XYZ", "G4ABC/P", "K1ABC/R", "PJ4/K1ABC", "YW18FIFA", "37", "DX", "TEST", "POTA", "123",
       "-08", "ABC", "HELLO", "a1bc", "7FFF", "DEAD"]
for _ in range(280):
    n = rnd.choice([1, 2, 2, 3, 3, 3, 4, 4, 5, 5, 6])
    add(" ".join(rnd.choice(tok) for _ in range(n)))

with open(tmp + "/cases.txt", "w", encoding="ascii", newline="\n") as f:
    for m in cases:
        assert len(m) <= 80 and "\n" not in m
        f.write(m + "\n")

def run(prefer):
    args = [oracle, tmp + "/cases.txt"] + (["prefer"] if prefer else [])
    res = subprocess.run(args, check=True, capture_output=True, text=True).stdout
    lines = res.split("\n")
    if lines and lines[-1] == "":
        lines.pop()
    assert len(lines) == len(cases), (len(lines), len(cases))
    return lines

plain = run(False)
pref = run(True)

nwords = lambda m: len(m.split())
rows = []
for m, a, b in zip(cases, plain, pref):
    rows.append("0\t" + a)
    if nwords(m) <= 3:
        rows.append("1\t" + b)

with open(out, "w", encoding="ascii", newline="\n") as f:
    f.write("# pack77_message against WSJT-X's own pack77 (lib/77bit/packjt77.f90) of %s (%s).\n" % (tag, commit))
    f.write("# Made by scripts/pack77/gen_cases.sh from scripts/pack77/pack77_oracle.f90, built with scripts/pack77/build_oracle.sh;\n")
    f.write("# regenerating it with the same oracle reproduces it byte for byte. %d cases (the first %d are rc1's lib/77bit/messages.txt),\n" % (len(cases), n_messages_txt))
    f.write("# %d rows: every case with prefer_wspr_50bit unset, and the cases of up to three words again with it set.\n" % len(rows))
    f.write("# Columns: prefer_wspr_50bit, message (as given, the first 37 characters count), status, i3.n3, 77 bits, what unpack77 prints for\n")
    f.write("# them, the calls pack77 committed to the hash table (oldest first). Status 'ok' or PACK77_STATUS_* (packjt77.f90:66-71) in lower case;\n")
    f.write("# for a status other than ok the last three columns are '-'. Each message was packed from an empty hash table.\n")
    for r in rows:
        f.write(r + "\n")
print("wrote %s: %d cases, %d rows" % (out, len(cases), len(rows)))
PY
