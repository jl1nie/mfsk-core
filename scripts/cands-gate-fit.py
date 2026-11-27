#!/usr/bin/env python3
"""Fit the gate before BP for a flow *with OSD* (and judge the OSD-off gate on OSD-on data): the 6-input logistic model of `cands-gate-analysis.py`, labels = candidates that decode
(BP or OSD) and are in the reference, cost = BP passes and OSD runs in host instructions. The generated sets train, the 60 real slots judge.
Input: the TSV of `icap-cands` run with OSD=1 FEATS=1 over the slots of scripts/cands-corpus.py, built with the local probe scripts/probe/gate-features.patch (columns: ... calls, passes,
osd_runs, six features). Usage: scripts/cands-gate-fit.py osdfeat_all.tsv   (needs numpy)"""
import sys
import numpy as np

BP_PASS_M = 0.0376   # M host instructions a BP pass (scripts/bp-instr-per-pass.sh: 37 600)
OSD_RUN_M = 9.2      # M host instructions an OSD run (callgrind of the cells flow with OSD on, 3 recordings: 8.7-9.4)
rows = []
for l in open(sys.argv[1]):
    r = l.rstrip("\n").split("\t")
    if len(r) < 13 or not r[12]:
        continue
    rows.append((r[0].split("_")[0], int(r[7]), int(r[10]), int(r[11]), [float(x) for x in r[12].split(",")]))
corp = np.array([r[0] for r in rows]); tr = np.array([r[1] for r in rows]); passes = np.array([r[2] for r in rows], float)
osd = np.array([r[3] for r in rows], float); F = np.array([r[4] for r in rows])
cost = passes * BP_PASS_M + osd * OSD_RUN_M
real = np.isin(corp, ["a", "b"]); syn = ~real
print(f"candidates that reached BP/OSD {len(rows)}; decodes in the reference {tr.sum()} (real {tr[real].sum()}); BP passes {int(passes.sum())}, OSD runs {int(osd.sum())}; "
      f"cost {cost.sum():.0f} M instructions (real {cost[real].sum():.0f}: BP {passes[real].sum()*BP_PASS_M:.0f} + OSD {osd[real].sum()*OSD_RUN_M:.0f})")
X = np.column_stack([F[:, 0], F[:, 1], F[:, 2], F[:, 3], F[:, 4], F[:, 5]])
mu = X[syn].mean(0); sd = X[syn].std(0) + 1e-9
Z = np.column_stack([(X - mu) / sd, np.ones(len(X))]); w = np.zeros(Z.shape[1])
for _ in range(6000):
    p = 1 / (1 + np.exp(-(Z[syn] @ w))); w -= 0.5 * (Z[syn].T @ (p - tr[syn]) / syn.sum() + 1e-4 * w)
coef = w[:-1] / sd; bias = w[-1] - float((w[:-1] * mu / sd).sum())
z = X @ coef + bias; zmin = z[syn & (tr == 1)].min()
print("coef:", [float(f"{c:.6g}") for c in coef], "bias:", float(f"{bias:.6g}"), "lowest logit of a decode in the generated sets:", float(f"{zmin:.4f}"))
def rep(name, skip):
    c = lambda m: 100 * cost[skip & m].sum() / cost[m].sum()
    print(f"  {name:34s} real: lost {int(tr[skip & real].sum()):3d} of {int(tr[real].sum())}, BP passes saved {100*passes[skip & real].sum()/passes[real].sum():5.1f}%, OSD runs saved "
          f"{100*osd[skip & real].sum()/osd[real].sum():5.1f}%, cost saved {c(real):5.1f}% | generated: lost {int(tr[skip & syn].sum())}, cost saved {c(syn):5.1f}%")
for margin in (0.0, 0.5, 1.0, 2.0):
    rep(f"OSD-on fit, margin {margin} (thr {zmin - margin:+.3f})", z < zmin - margin)
# the OSD-off gate (CELLS_GATE) applied to the same OSD-on data
off_coef = np.array([-3.08844, -4.3783, -0.467705, -0.143884, 0.0699579, 0.670358]); off_bias = 6.18511
zo = X @ off_coef + off_bias
rep("OSD-off gate (CELLS_GATE) on OSD-on data", zo < -5.6599 - 0.5)
