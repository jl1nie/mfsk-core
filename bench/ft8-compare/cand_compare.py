#!/usr/bin/env python3 -I
"""Compare the coarse candidate lists of jt9's sync8 and mfsk-core's coarse_sync on IDENTICAL audio:
the first search of the first stage (jt9 nz41 pass 1, ours A round 0; nothing subtracted yet).

usage: cand_compare.py <dir> [stem ...]    (<dir>/trace = jt9, <dir>/rtrace = ours)
A pair = coarse f within FW Hz and dt within DW s. Reports counts, overlap, and what the unmatched
candidates on each side look like (rank in its own list, score).
"""
import sys, os, re, glob, statistics as st
FW, DW = 1.6, 0.11
d = sys.argv[1]
stems = sys.argv[2:] or [os.path.basename(p)[:-6] for p in sorted(glob.glob(f"{d}/trace/*.trace"))]
T = dict(nj=0, no=0, both=0, jonly=0, oonly=0)
ranks_j, ranks_o, scores_j, scores_o, ratio = [], [], [], [], []
per = []
for n in stems:
    j, nz, op = [], 0, 0
    for l in open(f"{d}/trace/{n}.trace"):
        t = l.split()
        if not t: continue
        if t[0] == "PASS": nz, op = int(t[1]), int(t[2])
        elif t[0] == "CAND" and nz == 41 and op == 1: j.append((float(t[5]), float(t[6]), float(t[4])))   # f, dt, sync
    o, rnd = [], None
    for l in open(f"{d}/rtrace/{n}.rtrace"):
        if l.startswith("CAND A round=0 "):
            kv = dict(re.findall(r"(\w+)=(\S+)", l)); o.append((float(kv["f"]), float(kv["dt"]), float(kv["score"])))
    # one-to-one greedy match on distance
    used, pairs = set(), []
    for a, (f, dt, s) in enumerate(j):
        best = None
        for b, (g, dg, sg) in enumerate(o):
            if b in used or abs(f - g) > FW or abs(dt - dg) > DW: continue
            dist = abs(f - g) / FW + abs(dt - dg) / DW
            if best is None or dist < best[0]: best = (dist, b)
        if best: used.add(best[1]); pairs.append((a, best[1]))
    pj = {a for a, _ in pairs}; po = {b for _, b in pairs}
    T["nj"] += len(j); T["no"] += len(o); T["both"] += len(pairs)
    T["jonly"] += len(j) - len(pairs); T["oonly"] += len(o) - len(pairs)
    for a, b in pairs: ratio.append(o[b][2] / j[a][2])
    for a in range(len(j)):
        if a not in pj: ranks_j.append(a / max(len(j), 1)); scores_j.append(j[a][2])
    for b in range(len(o)):
        if b not in po: ranks_o.append(b / max(len(o), 1)); scores_o.append(o[b][2])
    per.append((n, len(j), len(o), len(pairs)))
print(f"{'file':16} {'jt9':>5} {'ours':>5} {'matched':>7}")
for r in per: print(f"{r[0]:16} {r[1]:5d} {r[2]:5d} {r[3]:7d}")
print(f"\nTOTAL jt9 {T['nj']}  ours {T['no']}  matched {T['both']}  jt9-only {T['jonly']}  ours-only {T['oonly']}")
if ratio: print(f"matched pairs: score(ours)/sync(jt9) median {st.median(ratio):.3f}  5..95%: {sorted(ratio)[int(len(ratio)*.05)]:.3f}..{sorted(ratio)[int(len(ratio)*.95)]:.3f}")
f = lambda v: f"median {st.median(v):.2f}, mean {st.mean(v):.2f}, max {max(v):.2f}" if v else "-"
print(f"jt9-only: score {f(scores_j)}; relative rank in its list (0 = best, 1 = worst) {f(ranks_j)}")
print(f"ours-only: score {f(scores_o)}; relative rank in its list {f(ranks_o)}")
