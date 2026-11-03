#!/usr/bin/env python3 -I
"""Where, in the 174 bits, do mfsk-core's LLR sets differ from jt9's? (A round 0, identical audio.)

usage: llr_bitpos.py <dir>      per LLR set b, c, e (and a, d for reference): the mean |difference| per bit
index, summed into groups, and the bit indices that carry most of the sign disagreements.
"""
import sys, os, glob, collections
d = sys.argv[1]
mean_abs = {k: [0.0] * 174 for k in range(5)}; flips = {k: collections.Counter() for k in range(5)}; n = 0
for tp in sorted(glob.glob(f"{d}/trace/*.trace")):
    stem = os.path.basename(tp)[:-6]; jt = []; cur = None
    for l in open(tp):
        t = l.split()
        if not t: continue
        if t[0] == "LLR":
            k = int(t[5]); v = [float(x) for x in t[6:6 + 174]]
            if k == 1: cur = dict(nz=int(t[1]), imetric=int(t[2]), f=float(t[3]), dt=float(t[4]), sets=[])
            cur["sets"].append(v)
            if k == 5: jt.append(cur)
    ours = []; cur = None; rnd = 0
    for l in open(f"{d}/rtrace/{stem}.rtrace"):
        if l.startswith("CAND "): rnd = int(l.split("round=")[1].split()[0])
        elif l.startswith("LLR "):
            h, _, vals = l.partition(" set="); kv = dict(x.split("=") for x in h.split()[2:]); st = h.split()[1]
            k, _, rest = vals.partition(" ")
            if k == "1": cur = dict(stage=st, round=rnd, imetric=int(kv["imetric"]), f=float(kv["f"]), dt=float(kv["dt"]), sets=[])
            cur["sets"].append([float(x) for x in rest.split()])
            if k == "5": ours.append(cur)
    for j in jt:
        if j["nz"] != 41 or j["imetric"] != 1: continue
        for o in ours:
            if o["stage"] == "A" and o["round"] == 0 and o["imetric"] == 1 and abs(o["f"] - j["f"]) <= 0.3 and abs(o["dt"] - j["dt"]) <= 0.011:
                n += 1
                for k in range(5):
                    for i in range(174):
                        mean_abs[k][i] += abs(j["sets"][k][i] - o["sets"][k][i])
                        if (j["sets"][k][i] > 0) != (o["sets"][k][i] > 0): flips[k][i + 1] += 1
                break
print(f"{n} candidate pairs")
for k in (0, 3, 1, 2, 4):
    tot = sum(flips[k].values())
    top = ", ".join(f"{b}:{c}" for b, c in flips[k].most_common(12))
    m = [x / n for x in mean_abs[k]]
    # mean |diff| for the bits that sit in the last group of each half vs the rest
    edge = [i for i in range(174) if (i + 1) in set(range(82, 88)) | set(range(169, 175))]
    rest = [i for i in range(174) if i not in set(edge)]
    print(f"set {'abcde'[k]}: sign flips {tot} total; mean|diff| edge bits (82-87,169-174) {sum(m[i] for i in edge)/len(edge):.3f}, other bits {sum(m[i] for i in rest)/len(rest):.3f}; top flipped bit idx(count): {top}")
