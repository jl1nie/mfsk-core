#!/usr/bin/env python3 -I
"""Compare the LLR vectors jt9's ft8b and mfsk-core's ladder see for the same candidate.

usage: llr_compare.py <dir> <file-stem> <message>
  <dir>/trace/<stem>.trace   JT9_TRACE build (LLR lines)       <dir>/rtrace/<stem>.rtrace  MFSK_FT8_TRACE + MFSK_FT8_TRACE_LLR
For the jt9 DEC of <message> (its LLR block is the five LLR lines before it), find our LLR blocks
at the same stage (nz41 = A, nz50 = C), same imetric, refined f within 0.6 Hz and dt within 0.03 s,
and print per set a..e: Pearson r, std ratio ours/jt9, sign disagreements (of 174).
"""
import re, sys, math
d, stem, msg = sys.argv[1:4]
norm = lambda m: re.sub(r"\s+", " ", m.strip()).upper()

def pearson(a, b):
    ma, mb = sum(a) / len(a), sum(b) / len(b)
    sa = math.sqrt(sum((x - ma) ** 2 for x in a)); sb = math.sqrt(sum((y - mb) ** 2 for y in b))
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / (sa * sb) if sa and sb else float("nan")

jt, block = None, []
for l in open(f"{d}/trace/{stem}.trace"):
    t = l.split()
    if not t: continue
    if t[0] == "LLR":
        k = int(t[5]); v = [float(x) for x in t[6:6 + 174]]
        if k == 1: block = []; meta = (int(t[1]), int(t[2]), float(t[3]), float(t[4]))
        block.append(v)
    elif t[0] == "DEC" and norm(" ".join(t[11:])) == norm(msg) and len(block) == 5 and jt is None:
        jt = dict(meta=meta, sets=block, ll=(int(t[3]), int(t[4]), int(t[5]), int(t[6])))
if not jt: sys.exit("jt9 DEC not found")
nz, imet, f1, dt1 = jt["meta"]
print(f"jt9: nz{nz} imetric={imet} f={f1:.3f} dt={dt1:.4f}  decoded at llr set {jt['ll'][0]}, aptype {jt['ll'][1]}, ntype {jt['ll'][2]}, herr {jt['ll'][3]}")
stage = "A" if nz == 41 else "C"
cur = {}; found = []
for l in open(f"{d}/rtrace/{stem}.rtrace"):
    if not l.startswith("LLR "): continue
    h, _, vals = l.partition(" set=")
    kv = dict(re.findall(r"(\w+)=(\S+)", h)); st = h.split()[1]
    k, _, rest = vals.partition(" ")
    if st != stage or int(kv["imetric"]) != imet: continue
    f, dt = float(kv["f"]), float(kv["dt"])
    if abs(f - f1) > 0.6 or abs(dt - dt1) > 0.03: continue
    cur.setdefault((f, dt, st), {})[int(k)] = [float(x) for x in rest.split()]
if not cur: sys.exit("no matching candidate of ours (same stage/imetric/position)")
for (f, dt, st), sets in cur.items():
    print(f"ours {st}: f={f:.3f} dt={dt:.4f}")
    for k in range(1, 6):
        a, b = jt["sets"][k - 1], sets[k]
        dis = sum(1 for x, y in zip(a, b) if (x > 0) != (y > 0))
        sa = math.sqrt(sum(x * x for x in a) / 174); sb = math.sqrt(sum(y * y for y in b) / 174)
        print(f"   set {'abcde'[k-1]}: r={pearson(a, b):.3f}  rms jt9 {sa:.2f} ours {sb:.2f}  sign disagreements {dis}/174  max|diff| {max(abs(x-y) for x,y in zip(a,b)):.2f}")
