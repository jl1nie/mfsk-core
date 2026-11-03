#!/usr/bin/env python3 -I
"""False-decode statistics on the truth-labelled busy-band corpus (ft8_busy_sweep), per configuration.

usage: phantom_stats.py <truth.csv> <phantom dir> [cfg ...]     (<phantom dir>/<cfg>/<stem>.csv + .rtrace)
A decode is TRUE when its message is one transmitted in that file (text; the noise-only files have none),
else FALSE (a phantom). For every decode the DEC event that produced it gives the stage, nsync, hard
errors and method; false decodes are listed with those, and the share of false among all decodes is
tabulated by method and hard-error bin: the empirical odds that a decode with those features is a phantom.
"""
import sys, os, re, csv, glob, collections
truth_p, d = sys.argv[1:3]
cfgs = sys.argv[3:] or ["head", "tail", "tail_dt"]
norm = lambda m: re.sub(r"\s+", " ", m.strip()).upper()
truth = collections.defaultdict(set); nsig = collections.Counter()
for r in csv.DictReader(open(truth_p)):
    truth[r["file"][:-4]].add(norm(r["msg"])); nsig[r["file"][:-4].split("_")[2]] += 1
def klass(method):
    m = method.strip("Some()")
    return "AP" if m.startswith("Ap") else "OSD" if m.startswith("Osd") else "BP" if m.startswith("Bp") else m
def hbin(h): return "<=12" if h <= 12 else "13-18" if h <= 18 else "19-24" if h <= 24 else "25+"
allstats = {}
for cfg in cfgs:
    rows = []; hits = collections.Counter(); sets = collections.Counter()
    for cp in sorted(glob.glob(f"{d}/{cfg}/*.csv")):
        stem = os.path.basename(cp)[:-4]; st = stem.split("_")[2]; sets[st] += 1
        dec = {norm(r["message"]) for r in csv.DictReader(open(cp)) if r["decoder"] == "mfsk" and r["message"]}
        ev = {}; cur = None
        for l in open(f"{d}/{cfg}/{stem}.rtrace"):
            h, _, r = l.rstrip("\n").partition(" ")
            if h == "DEC":
                body, _, msg = r.partition(" msg="); kv = dict(re.findall(r"(\w+)=(\S+)", body))
                ev.setdefault(norm(msg), dict(stage=body.split()[0], nsync=int(kv["nsync"]), hard=int(kv["hard"]), method=klass(kv["method"])))
        for m in dec:
            e = ev.get(m, dict(stage="?", nsync=-1, hard=-1, method="?"))
            ok = m in truth[stem]
            rows.append((ok, stem, m, e)); hits[st] += ok
    allstats[cfg] = (rows, hits, sets)
print(f"{'config':8s} " + " ".join(f"{s:>14s}" for s in sorted(nsig)) + "   | false decodes (all sets) | true decodes")
for cfg in cfgs:
    rows, hits, sets = allstats[cfg]
    fa = collections.Counter(r[1].split("_")[2] for r in rows if not r[0])
    print(f"{cfg:8s} " + " ".join(f"{hits[s]:4d}/{nsig[s]:<4d} +{fa[s]:<3d}" for s in sorted(nsig)) + f"   | {sum(fa.values()):4d} | {sum(r[0] for r in rows)}")
print("\n(true hits / signals sent, + false decodes per set)\n")
for cfg in cfgs:
    rows = allstats[cfg][0]
    print(f"== {cfg}: the false decodes")
    for ok, stem, m, e in rows:
        if not ok: print(f"   {stem:22s} {m:28s} {e['stage']} {e['method']:3s} hard={e['hard']:2d} nsync={e['nsync']:2d}")
tot = collections.defaultdict(lambda: [0, 0])
for cfg in cfgs:
    for ok, stem, m, e in allstats[cfg][0]:
        t = tot[(cfg, e["method"], hbin(e["hard"]))]; t[0] += 1; t[1] += (not ok)
print("\nshare of false decodes among all decodes, by method and hard-error bin (false/all):")
print(f"  {'cfg':8s} {'method':6s} " + " ".join(f"{b:>10s}" for b in ("<=12", "13-18", "19-24", "25+")))
for cfg in cfgs:
    for meth in ("BP", "OSD", "AP"):
        print(f"  {cfg:8s} {meth:6s} " + " ".join(f"{tot[(cfg,meth,b)][1]:4d}/{tot[(cfg,meth,b)][0]:<5d}" for b in ("<=12", "13-18", "19-24", "25+")))
