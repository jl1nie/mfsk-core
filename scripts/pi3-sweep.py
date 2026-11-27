#!/usr/bin/env python3
"""Table of the Pi 3B study (docs/notes/PI3_JT9_GAP.md) from `scripts/pi3-sweep.sh`'s output directory.

  scripts/pi3-sweep.py <out-dir> <corpus-dir>

Per configuration: decodes, share of the references (a_* = jt9's 402 on the 22 recordings, b_* = the busy slots' truth, all = 1 338), extras (decodes not in the references),
wall time a slot on 1 and on 4 threads of this machine and the speed-up, plus the hybrid rows (union of two configurations, the times added).
"""
import collections, glob, os, sys

out, corpus = sys.argv[1], sys.argv[2]

def norm(s): return " ".join(s.split())

refs = {}
for p in glob.glob(os.path.join(corpus, "*.ref")):
    refs[os.path.basename(p)[:-4]] = {norm(l) for l in open(p) if l.strip()}
n_ref = collections.Counter()
for s, r in refs.items(): n_ref[s[0]] += len(r); n_ref["all"] += len(r)

def load(path):
    dec = collections.defaultdict(set); ms = {}
    for l in open(path):
        k = l.split(" ", 1)
        if k[0] == "S":
            _, slot, n, h, t = l.split(); ms[slot] = float(t)
        elif k[0] == "D":
            _, slot, msg = l.rstrip("\n").split(" ", 2); dec[slot].add(msg)
    return dec, ms

def score(dec):
    hit = collections.Counter(); ex = 0; tot = 0
    for slot, msgs in dec.items():
        for m in msgs:
            tot += 1
            if m in refs.get(slot, ()): hit[slot[0]] += 1; hit["all"] += 1
            else: ex += 1
    return tot, hit, ex

rows = {}
for f in sorted(glob.glob(os.path.join(out, "*.t1.txt"))):
    cfg = os.path.basename(f)[:-len(".t1.txt")]
    d1, m1 = load(f)
    f4 = f.replace(".t1.", ".t4.")
    d4, m4 = load(f4) if os.path.exists(f4) else ({}, {})
    if d4 and d4 != d1:
        print(f"FAIL: {cfg}: the decodes on 4 threads differ from 1 thread", file=sys.stderr); sys.exit(1)
    rows[cfg] = (d1, sum(m1.values()), sum(m4.values()) if m4 else None, max(m1.values()) if m1 else 0)

def line(name, dec, t1, t4, worst):
    tot, hit, ex = score(dec)
    n = len(refs)
    sp = f"{t1 / t4:4.2f}x" if t4 else "  - "
    print(f"{name:<26}{tot:6d} {hit['a']:4d}/{n_ref['a']} {100*hit['a']/n_ref['a']:5.1f}% {hit['b']:4d}/{n_ref['b']} {100*hit['b']/n_ref['b']:5.1f}% {hit['all']:5d} {100*hit['all']/n_ref['all']:5.1f}% {ex:5d}  {t1/n:7.0f} {(t4/n if t4 else 0):7.0f} {sp} {worst:7.0f}")

print(f"{'configuration':<26}{'dec':>6} {'22 rec (jt9)':>16} {'38 busy':>16} {'all 60':>13} {'extra':>5}  {'ms/slot 1T':>10} {'4T':>7} {'speed':>5} {'worst1T':>7}")
for cfg, (d, t1, t4, worst) in rows.items():
    line(cfg, d, t1, t4, worst)
# hybrids: the cells flow first, the faithful pipeline after: the union, the times added
for a, b in (("cells_sic3osd", "deep_full"), ("cells_sic3osd", "normal_full"), ("cells_p0_late", "deep_full")):
    if a in rows and b in rows:
        u = collections.defaultdict(set)
        for cfg in (a, b):
            for s, m in rows[cfg][0].items(): u[s] |= m
        t1 = rows[a][1] + rows[b][1]
        t4 = (rows[a][2] + rows[b][2]) if rows[a][2] and rows[b][2] else None
        line(f"{a}+{b}", u, t1, t4, rows[a][3] + rows[b][3])
