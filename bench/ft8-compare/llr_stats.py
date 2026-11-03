#!/usr/bin/env python3 -I
"""LLR agreement between jt9 and mfsk-core over every candidate both refined to the same place.

usage: llr_stats.py <dir> [stem ...]     (needs trace/ and rtrace/ from a run with MFSK_FT8_TRACE_LLR=1)
A pair = same stage (nz41/A, nz50/C), same imetric, f within 0.3 Hz and dt within 0.011 s. Per
(stage, round) and per LLR set a..e: how many pairs, median and mean sign disagreements of 174,
and the share of pairs whose signs agree on every bit.
"""
import sys, os, glob, collections, statistics as st
d = sys.argv[1]
stems = sys.argv[2:] or [os.path.basename(p)[:-6] for p in sorted(glob.glob(f"{d}/trace/*.trace"))]
res = collections.defaultdict(lambda: collections.defaultdict(list))   # (stage, round) -> set -> [disagreements]
npairs = collections.Counter()
for n in stems:
    jt = []; cur = None; nz = 0; op = 0
    for l in open(f"{d}/trace/{n}.trace"):
        t = l.split()
        if not t: continue
        if t[0] == "PASS": nz, op = int(t[1]), int(t[2])
        elif t[0] == "LLR":
            k = int(t[5]); v = [x > 0 for x in map(float, t[6:6 + 174])]
            if k == 1: cur = dict(nz=nz, op=op, imetric=int(t[2]), f=float(t[3]), dt=float(t[4]), sets=[])
            cur["sets"].append(v)
            if k == 5: jt.append(cur)
    ours = []; cur = None; rnd = 0
    for l in open(f"{d}/rtrace/{n}.rtrace"):
        if l.startswith("CAND "):
            rnd = int(l.split("round=")[1].split()[0])
        elif l.startswith("LLR "):
            h, _, vals = l.partition(" set="); kv = dict(x.split("=") for x in h.split()[2:]); st_ = h.split()[1]
            k, _, rest = vals.partition(" ")
            if k == "1": cur = dict(stage=st_, round=rnd, imetric=int(kv["imetric"]), f=float(kv["f"]), dt=float(kv["dt"]), sets=[])
            cur["sets"].append([x > 0 for x in map(float, rest.split())])
            if k == "5": ours.append(cur)
    by = collections.defaultdict(list)
    for o in ours: by[(o["stage"], o["imetric"])].append(o)
    for j in jt:
        stage = "A" if j["nz"] == 41 else "C"
        for o in by[(stage, j["imetric"])]:
            if abs(o["f"] - j["f"]) <= 0.3 and abs(o["dt"] - j["dt"]) <= 0.011 and o["round"] == j["op"] - 1:
                key = (stage, o["round"]); npairs[key] += 1
                for k in range(5):
                    res[key][k].append(sum(a != b for a, b in zip(j["sets"][k], o["sets"][k])))
                break
print(f"{'stage/round':12} {'pairs':>5} | sign disagreements of 174: median (mean), share of pairs with 0")
for key in sorted(res):
    cells = []
    for k in range(5):
        v = res[key][k]
        cells.append(f"{'abcde'[k]}: {st.median(v):4.0f} ({st.mean(v):5.1f}) {100*sum(1 for x in v if x==0)/len(v):3.0f}%")
    print(f"{key[0]}{key[1]:<11} {npairs[key]:5d} | " + "  ".join(cells))
