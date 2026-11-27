#!/usr/bin/env python3
"""Yield of the cells flow's candidates: what each score / hard-sync / rank bucket decodes, and what a floor would save and lose.
Input: the TSV of `icap-cands` (hosttest/icap-app), one row per candidate.
Usage: scripts/cands-yield.py probe.tsv [--by-corpus]
`--by-corpus` repeats the report per corpus, the corpus being the slot name's prefix up to the first "_"
(a_ / b_: the real recordings, busy / dense / sweep / itu: the generated tier-C sets)."""
import collections
import sys


def load(path):
    R = []
    for l in open(path):
        r = l.rstrip("\n").split("\t")
        R.append(dict(slot=r[0], i=int(r[1]), sc=int(r[4]), hs=int(r[5]), dec=int(r[6]), tr=int(r[7]),
                      calls=int(r[9]) if len(r) > 10 else 0, passes=int(r[10]) if len(r) > 10 else 0))
    return R


def report(R, title, full=True):
    N = len(R)
    T = sum(r["tr"] for r in R)
    D = sum(r["dec"] for r in R)
    P = sum(r["passes"] for r in R)
    slots = len({r["slot"] for r in R})
    print(f"\n=== {title}: {slots} slots, {N} candidates tried, decoded {D}, in the reference {T}, decoded but not in the reference {D - T}")
    if P:
        f = [r for r in R if not r["dec"]]
        print(f"BP calls {sum(r['calls'] for r in R)}, message-update passes {P}; {100 * sum(r['passes'] for r in f) / P:.1f}% of the passes are spent on candidates that do not decode")

    def table(name, key, buckets):
        print(f"\nby {name}: tried, in the reference, yield, passes")
        for lo, hi in buckets:
            s = [r for r in R if lo <= r[key] <= hi]
            t = sum(r["tr"] for r in s)
            ps = sum(r["passes"] for r in s)
            print(f"  {key} {lo:>3}-{hi:<3}: tried {len(s):6d}  true {t:6d}  yield {100 * t / max(1, len(s)):5.1f}%"
                  + (f"  passes {100 * ps / P:4.1f}%" if P else ""))

    if full:
        table("score", "sc", [(8, 8), (9, 9), (10, 10), (11, 12), (13, 15), (16, 20), (21, 30), (31, 127)])
        table("hard sync", "hs", [(0, 6), (7, 8), (9, 10), (11, 12), (13, 15), (16, 18), (19, 21)])
        table("rank (the list is ordered by hard sync)", "i", [(0, 4), (5, 9), (10, 14), (15, 19), (20, 24), (25, 29)])
    print("\nskip a candidate with score < S: tries saved, decodes in the reference lost")
    for S in (9, 10, 11, 12, 13, 15):
        s = [r for r in R if r["sc"] < S]
        ps = sum(r["passes"] for r in s)
        print(f"  S={S:>2}: saved {len(s):6d} ({100 * len(s) / N:4.1f}%), lost {sum(r['tr'] for r in s):5d} of {T}"
              + (f"; passes saved {100 * ps / P:4.1f}%" if P else ""))
    print("skip a candidate with hard sync < H")
    for H in (7, 8, 9, 10, 11, 12):
        s = [r for r in R if r["hs"] < H]
        ps = sum(r["passes"] for r in s)
        print(f"  H={H:>2}: saved {len(s):6d} ({100 * len(s) / N:4.1f}%), lost {sum(r['tr'] for r in s):5d}"
              + (f"; passes saved {100 * ps / P:4.1f}%" if P else ""))


if __name__ == "__main__":
    R = load(sys.argv[1])
    report(R, "all")
    if "--by-corpus" in sys.argv:
        by = collections.defaultdict(list)
        for r in R:
            by[r["slot"].split("_")[0]].append(r)
        for k in sorted(by):
            report(by[k], k, full=False)
