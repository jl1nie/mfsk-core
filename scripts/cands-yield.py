#!/usr/bin/env python3
"""Yield of the cells flow's candidates: what each score / hard-sync / rank bucket decodes, and what a floor would save and lose.
Input: the TSV of `icap-cands` (hosttest/icap-app), one row per candidate. Usage: scripts/cands-yield.py probe.tsv"""
import sys
R = []
for l in open(sys.argv[1]):
    r = l.rstrip("\n").split("\t")
    R.append(dict(slot=r[0], i=int(r[1]), sc=int(r[4]), hs=int(r[5]), dec=int(r[6]), tr=int(r[7]),
                  calls=int(r[9]) if len(r) > 10 else 0, passes=int(r[10]) if len(r) > 10 else 0))
N = len(R); T = sum(r["tr"] for r in R); D = sum(r["dec"] for r in R)
print(f"candidates tried {N}, decoded {D}, in the reference {T}, decoded but not in the reference {D - T}")
P = sum(r["passes"] for r in R)
if P:
    f = [r for r in R if not r["dec"]]
    print(f"BP calls {sum(r['calls'] for r in R)}, message-update passes {P} (measured: `BpStats`); {100 * sum(r['passes'] for r in f) / P:.1f}% of the passes are spent on candidates that do not decode")
def table(title, key, buckets):
    print(f"\n{title}: tried, in the reference, yield")
    for lo, hi in buckets:
        s = [r for r in R if lo <= r[key] <= hi]; t = sum(r["tr"] for r in s)
        ps = sum(r["passes"] for r in s)
        print(f"  {key} {lo:>3}-{hi:<3}: tried {len(s):4d}  true {t:4d}  yield {100 * t / max(1, len(s)):5.1f}%" + (f"  passes {ps:6d} ({100 * ps / P:4.1f}%)  per true decode {ps / t if t else float('inf'):6.1f}" if P else ""))
table("by score", "sc", [(8, 8), (9, 9), (10, 10), (11, 12), (13, 15), (16, 20), (21, 30), (31, 127)])
table("by hard sync", "hs", [(0, 6), (7, 8), (9, 10), (11, 12), (13, 15), (16, 18), (19, 21)])
table("by rank (the list is ordered by hard sync)", "i", [(0, 4), (5, 9), (10, 14), (15, 19), (20, 24), (25, 29)])
print("\nskip a candidate with score < S: tries saved, decodes in the reference lost")
for S in (9, 10, 11, 12, 13, 15):
    s = [r for r in R if r["sc"] < S]; ps = sum(r["passes"] for r in s)
    print(f"  S={S:>2}: saved {len(s):4d} ({100 * len(s) / N:4.1f}%), lost {sum(r['tr'] for r in s):3d} of {T}" + (f"; passes saved {ps} ({100 * ps / P:4.1f}%)" if P else ""))
print("\nskip a candidate with hard sync < H")
for H in (7, 8, 9, 10, 11, 12):
    s = [r for r in R if r["hs"] < H]; ps = sum(r["passes"] for r in s)
    print(f"  H={H:>2}: saved {len(s):4d} ({100 * len(s) / N:4.1f}%), lost {sum(r['tr'] for r in s):3d}" + (f"; passes saved {ps} ({100 * ps / P:4.1f}%)" if P else ""))
