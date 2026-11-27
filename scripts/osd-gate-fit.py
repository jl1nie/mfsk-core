#!/usr/bin/env python3
"""Which OSD attempts are worth running: a gate (or a per-slot cap K) for the cells flow's OSD fallback, fitted on the attempts of 4 780 slots.
Input: the `A` rows of `icap-cands` (OSD=1 OSD_SEEDS=ad) built with `scripts/probe/osd-attempts.patch` (a local probe, not in the tree): one row per `try_fallback` call with
|llra| statistics, the hard sync, the BP's unsatisfied checks, whether OSD returned and whether the candidate is in the reference. The generated sets train, the 60 real slots judge.
Usage: scripts/osd-gate-fit.py attempts.tsv   (needs numpy; design note section 28)"""
import numpy as np, sys
rows=[l.rstrip("\n").split("\t") for l in open(sys.argv[1])]
corp=np.array([r[1].split("_")[0] for r in rows])
f=np.array([[float(x) for x in r[4:8]] for r in rows])          # mean,std,max,mean/std of |llra|
q=np.array([float(r[8]) for r in rows]); bp=np.array([[float(x) for x in r[9:13]] for r in rows])
suc=np.array([int(r[13]) for r in rows]); truth=np.array([int(r[16]) for r in rows]); nbp=np.array([int(r[15]) for r in rows])
score=np.array([float(r[17]) for r in rows]); hs=np.array([float(r[18]) for r in rows])
# d's counts are only valid when a second BP call was logged
bp[nbp<2,2]=bp[nbp<2,0]; bp[nbp<2,3]=bp[nbp<2,1]
y=((suc==1)&(truth==1)).astype(int); extra=((suc==1)&(truth==0))
real=np.isin(corp,["a","b"]); syn=~real
print(f"OSD attempts {len(y)} (real {real.sum()}, generated {syn.sum()}); successes true {y.sum()} (real {y[real].sum()}), not in reference {extra.sum()} (real {extra[real].sum()})")
def fit(X,name):
    mu=X[syn].mean(0); sd=X[syn].std(0)+1e-9; Z=np.column_stack([(X-mu)/sd,np.ones(len(X))]); w=np.zeros(Z.shape[1])
    pos=y[syn].mean(); wt=np.where(y[syn]==1,0.5/pos,0.5/(1-pos))   # balanced weights: the positives are 0.5 %
    for _ in range(4000):
        p=1/(1+np.exp(-(Z[syn]@w))); w-=0.5*(Z[syn].T@((p-y[syn])*wt)/wt.sum()+1e-4*w)
    z=Z@w
    print(f"\n== {name}")
    for keep in (1.0,0.995,0.99,0.98,0.95):
        zs=np.sort(z[syn&(y==1)]); thr=zs[int(np.floor((1-keep)*len(zs)))] if keep<1 else zs[0]
        run=z>=thr
        print(f"  keep {keep*100:5.1f}% of generated successes: OSD attempts run {100*run[syn].mean():5.1f}% of generated; real: attempts run {100*run[real].mean():5.1f}%, successes kept {int(y[real&run].sum())}/{int(y[real].sum())}, extras kept {int(extra[real&run].sum())}/{int(extra[real].sum())}")
    return z
fit(np.column_stack([f,q]),"llra stats + q")
fit(np.column_stack([f,q,score,hs]),"llra stats + q + search score + hard sync")
fit(np.column_stack([f,q,score,hs,bp]),"... + BP checks left (min and last of variant a and d)")
fit(np.column_stack([bp,q]),"BP checks + q only")

# ---- top-K per slot: run OSD only on the K best-scored attempts of a slot
slot=np.array([r[1] for r in rows])
def topk(z,name,Ks=(2,3,4,6,8,10,14)):
    print(f"\n== top-K per slot by {name}")
    for K in Ks:
        keep=np.zeros(len(z),bool)
        for s in np.unique(slot):
            idx=np.where(slot==s)[0]
            if len(idx)<=K: keep[idx]=True; continue
            keep[idx[np.argsort(-z[idx])[:K]]]=True
        print(f"  K={K:2d}: real attempts run {keep[real].sum():4d}/{real.sum()} ({100*keep[real].mean():4.1f}%), successes kept {int(y[real&keep].sum())}/{int(y[real].sum())}, extras {int(extra[real&keep].sum())}/{int(extra[real].sum())} | generated: run {100*keep[syn].mean():4.1f}%, kept {int(y[syn&keep].sum())}/{int(y[syn].sum())}")
z1=fit(np.column_stack([f,q,score,hs]),"(refit) llra stats + q + score + hs")
topk(z1,"that model")
topk(score,"search score")
topk(q,"hard sync q")
topk(f[:,3],"mean/std of |llra|")
print("\nattempts per slot on the real slots: mean %.1f"%(real.sum()/60))
