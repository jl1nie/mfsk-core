#!/usr/bin/env python3
"""Early abort of belief propagation, simulated on recorded traces: abort a BP call at pass k when the unsatisfied parity checks (ncheck) are still above T.
Input: the TRACE lines of `icap-cands` built with scripts/probe/bp-trace.patch (a LOCAL probe, not committed code: `git apply scripts/probe/bp-trace.patch`
in the repo, build icap-cands (`--no-default-features --features fixed-point`), run it over the slots of scripts/cands-corpus.py, split into big2/part_*.tsv; then
`git checkout` the two files). One TRACE line per BP call, before its candidate's summary line.
Counted: passes saved on failing calls (orig - k), and, pessimistically, a decode lost when the call that decoded is aborted (another variant may rescue it).
Design note section 27."""
import glob,collections,sys
rules=[(k,T) for k in (4,6,8,10,12,15,20) for T in (12,16,20,24,28,32)]
tot=collections.defaultdict(lambda:collections.Counter())
def corp(s): return s.split("_")[0]
ncand=0; mism=0
for fn in glob.glob("big2/part_*.tsv"):
    pend=[]
    for l in open(fn):
        if l.startswith("TRACE\t"):
            p=l.rstrip("\n").split("\t"); pend.append((list(map(int,p[4].split(","))), p[3]=="1")); continue
        r=l.rstrip("\n").split("\t")
        calls=int(r[9]); c=corp(r[0]); true=int(r[7])
        if calls!=len(pend): mism+=1
        
        ncand+=1
        pend_now=pend; pend=[]
        t=tot[c]
        t['true']+=true
        origp=0
        for tr,ok in pend_now:
            n=len(tr); orig=n-1 if (ok or n<31) else n
            origp+=orig
        t['passes']+=origp
        for (k,T) in rules:
            sav=0; lost=0
            for tr,ok in pend_now:
                n=len(tr); orig=n-1 if (ok or n<31) else n
                # abort at iteration k if the call is still running there and ncheck > T
                if k<=n-1 and tr[k]>T and not (ok and k>=n-1):
                    if ok: lost=1
                    else: sav+=orig-k
                    if ok: break
            t[('sav',k,T)]+=sav
            if lost and true: t[('lost',k,T)]+=1
print("candidates",ncand,"call-count mismatches",mism)
print("baseline passes from the traces:",sum(t["passes"] for t in tot.values()),"(BpStats total was 7448386)")
allc=collections.Counter()
for c,t in tot.items():
    for k,v in t.items(): allc[k]+=v
tot['ALL']=allc
order=['a','b','busy','dense','sweep','itu','ALL']
print("rule: abort a BP call at pass k if the unsatisfied checks > T   ->  passes saved % / true decodes lost")
print(f"{'k,T':8s}"+"".join(f"{c:>14s}" for c in order))
print(f"{'(true)':8s}"+"".join(f"{tot[c]['true']:>14d}" for c in order))
for (k,T) in rules:
    print(f"{k:>3},{T:<4d}"+"".join(f"{100*tot[c][('sav',k,T)]/max(1,tot[c]['passes']):6.1f}%/{tot[c][('lost',k,T)]:<5d}" for c in order))
