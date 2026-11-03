#!/usr/bin/env python3 -I
"""Score ft8-compare's CSV and jt9's output against a corpus's truth.csv (the transmitted messages).

usage: score_truth.py <truth.csv> <ft8-compare out.csv> <dir with <name>.txt of jt9> <file prefix e.g. ft8_dense_>

A truth signal is HIT when the decoder printed its message (whitespace/case-insensitive, jt9's
` ?` / ` aN` marks stripped) within FTOL Hz. DT is not compared: rust-ft8 reports DT shifted by
its `window_start_offset`. An EXTRA is a decoded message that is not any truth message of the file.
Per set: recall = hits / signals sent, extras summed over files; and recall split by SNR band.
"""
import csv, re, sys, collections, os
FTOL = 5.0
truth_p, csv_p, jdir, prefix = sys.argv[1:5]
norm = lambda m: re.sub(r"\s+", " ", m.strip()).upper()
tail = re.compile(r"(\s+\?)?(\s+a\d+)?\s*$")
jl = re.compile(r"^\d{6}\s+(-?\d+)\s+(-?[\d.]+)\s+(\d+)\s+~\s+(.*?)\s*$")

truth = collections.defaultdict(list)
for r in csv.DictReader(open(truth_p)):
    truth[r["file"]].append((norm(r["msg"]), float(r["f0"]), float(r["snr"])))
dec = collections.defaultdict(lambda: collections.defaultdict(list))
for r in csv.DictReader(open(csv_p)):
    if r["message"]:
        dec[r["decoder"]][r["file"]].append((norm(r["message"]), float(r["freq_hz"])))
for f in truth:
    for l in open(os.path.join(jdir, f[:-4] + ".txt"), errors="replace"):
        m = jl.match(l)
        if m:
            dec["jt9"][f].append((norm(tail.sub("", m[4])), float(m[3])))

BANDS = [(-99, -18), (-18, -12), (-12, -6), (-6, 0), (0, 99)]
names = ["jt9", "mfsk", "mfsk-sic3", "mfsk-single", "rf8-default", "rf8-deep"]
names = [n for n in names if n in dec]
agg = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0, 0]))   # (set,dec) -> band|all -> [truth,hit,extra]
for f, sigs in truth.items():
    s = f[len(prefix):].rsplit("_", 1)[0]
    for d in names:
        got = dec[d][f]
        tmsgs = {m for m, _, _ in sigs}
        for m, f0, snr in sigs:
            hit = any(g == m and abs(gf - f0) <= FTOL for g, gf in got)
            for key in ("all", next(b for b in BANDS if b[0] <= snr < b[1])):
                agg[(s, d)][key][0] += 1; agg[(s, d)][key][1] += hit
        agg[(s, d)]["all"][2] += len({g for g, _ in got if g not in tmsgs})
sets = sorted({s for s, _ in agg}, key=lambda x: ["base", "snr", "gap", "dt", "real"].index(x) if x in ["base", "snr", "gap", "dt", "real"] else 9)
print(f"{'set':6} {'decoder':12} {'signals':>7} {'recall':>7} {'extra':>5} | recall by SNR band: " + " ".join(f"{b[0]:>4}..{b[1]:<3}" for b in BANDS))
for s in sets:
    for d in names:
        a = agg[(s, d)]
        t, h, e = a["all"]
        bands = " ".join((f"{100*a[b][1]/a[b][0]:6.1f}%({a[b][0]:>3})" if a[b][0] else "      -     ") for b in BANDS)
        print(f"{s:6} {d:12} {t:7} {100*h/t:6.1f}% {e:5} | {bands}")
    print()
