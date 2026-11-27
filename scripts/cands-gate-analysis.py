"""A pre-BP gate from the statistics of llra: can the cells flow tell a candidate that will not decode before it runs BP? (design note section 27)
Input: feat_all.tsv = the `icap-cands` TSV (scripts/cands-corpus.py slots) built with scripts/probe/llr-features.patch (a LOCAL probe: `git apply` it, build icap-cands,
run it over the slots, `git checkout` the files): the 12th column is the first BP attempt's llra statistics (mean, std, median, p10, p90, share under half the mean,
share over twice the mean, max of |llr|). Needs numpy. Trains a small logistic model and judges it: the generated sets train, the 60 real slots judge (and
leave-one-family-out); the threshold is the lowest score of any true decode of the training sets (so none of them is lost)."""
import numpy as np, collections, zlib
names=["mean","std","med","p10","p90","small","big","max"]
rows=[]
for l in open("feat_all.tsv"):
    r=l.rstrip("\n").split("\t")
    if len(r)<12 or not r[11]: continue
    f=list(map(float,r[11].split(",")))
    rows.append((r[0],int(r[4]),int(r[5]),int(r[7]),int(r[10]),f))
slot=[r[0] for r in rows]
corp=np.array([s.split("_")[0] for s in slot])
sc=np.array([r[1] for r in rows],float); hs=np.array([r[2] for r in rows],float)
tr=np.array([r[3] for r in rows]); passes=np.array([r[4] for r in rows],float)
F=np.array([r[5] for r in rows],float)
N=len(rows); print("candidates that reach BP:",N,"true decodes among them:",tr.sum(),"passes",int(passes.sum()))
split=np.array([zlib.crc32(s.encode())%2 for s in slot])   # 0 = train, 1 = test (by slot)
real=np.isin(corp,['a','b']); syn=~real
def report(name,skip,note=""):
    lost=int(tr[skip].sum()); sv=passes[skip].sum()
    out=f"  {name:34s} skips {skip.sum():6d} ({100*skip.mean():4.1f}%), lost {lost:3d} of {tr.sum()}, passes saved {100*sv/passes.sum():5.1f}% (real {100*passes[skip&real].sum()/passes[real].sum():5.1f}%, busy {100*passes[skip&(corp=='busy')].sum()/passes[corp=='busy'].sum():5.1f}%, dense {100*passes[skip&(corp=='dense')].sum()/passes[corp=='dense'].sum():5.1f}%)"
    print(out+note)
print("\n== single features: threshold at which no true decode is lost (and at <=0.1% lost)")
for j,n in enumerate(names):
    x=F[:,j]
    # skip if x < t  (low feature = weak) ; find the largest t with 0 lost / <=7 lost
    for allowed in (0,7):
        ts=np.sort(x[tr==1]); t=ts[allowed] if allowed<len(ts) else ts[-1]
        skip=x<t if True else None
        report(f"{n} < {t:.3f} (<= {allowed} lost)",skip)
print("\n== a logistic score (features + score + hard sync), trained on even slots, judged on odd slots")
X=np.column_stack([F,sc,hs,F[:,0]/np.maximum(F[:,1],1e-3)]); mu=X.mean(0); sd=X.std(0)+1e-9; Z=(X-mu)/sd; Z=np.column_stack([Z,np.ones(N)])
w=np.zeros(Z.shape[1]); trn=split==0
for it in range(3000):
    p=1/(1+np.exp(-(Z[trn]@w))); g=Z[trn].T@(p-tr[trn])/trn.sum()+1e-4*w; w-=0.5*g
p_all=1/(1+np.exp(-(Z@w)))
def auc(y,s):
    o=np.argsort(s); r=np.empty(len(s)); r[o]=np.arange(len(s)); n1=y.sum(); n0=len(y)-n1
    return (r[y==1].sum()-n1*(n1-1)/2)/(n1*n0)
tst=split==1
print(f"  AUC train {auc(tr[trn],p_all[trn]):.4f}   test {auc(tr[tst],p_all[tst]):.4f}")
for nm,m in (("train",trn),("test",tst)):
    ts=np.sort(p_all[m&(tr==1)])
    for allowed in (0,3):
        t=ts[min(allowed,len(ts)-1)]
        skip=m&(p_all<t); 
        lost=int(tr[skip].sum()); sv=passes[skip].sum()/passes[m].sum()
        print(f"  {nm}: skip p<{t:.5f} (<= {allowed} lost of {int(tr[m].sum())} in this half): skips {skip.sum()/m.sum()*100:4.1f}% of candidates, lost {lost}, passes saved {100*sv:5.1f}%")
# threshold chosen on train (0 lost), applied to test
t=np.sort(p_all[trn&(tr==1)])[0]; skip=tst&(p_all<t)
print(f"  threshold from train (0 lost) applied to test: lost {int(tr[skip].sum())} of {int(tr[tst].sum())}, passes saved {100*passes[skip].sum()/passes[tst].sum():.1f}% of the test half; real {100*passes[skip&real].sum()/passes[tst&real].sum():.1f}%")
print("  weights:",dict(zip(names+["score","hs","mean/std","bias"],np.round(w,2))))
print("\n== for comparison, the candidate rules that lose none (same candidates)")
report("hs < 8",hs<8); report("score<10 and hs<10",(sc<10)&(hs<10)); report("score<12 and hs<9",(sc<12)&(hs<9))

print("\n\n===== generalisation: train on the generated sets only, judge on the 60 real slots =====")
def fit(Xm,trainmask):
    mu=Xm[trainmask].mean(0); sd=Xm[trainmask].std(0)+1e-9
    Zm=np.column_stack([(Xm-mu)/sd,np.ones(len(Xm))]); w=np.zeros(Zm.shape[1])
    for it in range(4000):
        p=1/(1+np.exp(-(Zm[trainmask]@w))); w-=0.5*(Zm[trainmask].T@(p-tr[trainmask])/trainmask.sum()+1e-4*w)
    return 1/(1+np.exp(-(Zm@w)))
models={"M0: score, hs":np.column_stack([sc,hs]),"M1: + LLR features":np.column_stack([F,sc,hs,F[:,0]/np.maximum(F[:,1],1e-3)])}
for name,Xm in models.items():
    p=fit(Xm,syn)
    thr=np.sort(p[syn&(tr==1)])[0]                    # 0 lost among the generated sets' true decodes
    sk=real&(p<thr)
    thr_r=np.sort(p[real&(tr==1)])[0]                 # oracle: 0 lost on the real slots themselves
    sk_r=real&(p<thr_r)
    print(f"  {name:22s} AUC real {auc(tr[real],p[real]):.4f} | threshold from the generated sets (0 lost there): on real lost {int(tr[sk].sum())} of {int(tr[real].sum())}, skips {100*sk.sum()/real.sum():4.1f}% of the candidates, passes saved {100*passes[sk].sum()/passes[real].sum():5.1f}%"
          f" | oracle threshold (0 lost on real): passes saved {100*passes[sk_r].sum()/passes[real].sum():5.1f}%")
# per real corpus, with the M1 generated-set threshold
p=fit(models["M1: + LLR features"],syn); thr=np.sort(p[syn&(tr==1)])[0]
for c in ('a','b'):
    m=corp==c; sk=m&(p<thr)
    print(f"  M1 on {c}: lost {int(tr[sk].sum())} of {int(tr[m].sum())}, passes saved {100*passes[sk].sum()/passes[m].sum():.1f}%")

print("\n\n===== cheap features only (no sort): mean, std, max, mean/std, score, hs =====")
cheap=np.column_stack([F[:,0],F[:,1],F[:,7],F[:,0]/np.maximum(F[:,1],1e-3),sc,hs])
for name,Xm in (("cheap (6 inputs)",cheap),("cheap, no hs",np.column_stack([F[:,0],F[:,1],F[:,7],F[:,0]/np.maximum(F[:,1],1e-3),sc])),("std, max, score, hs",np.column_stack([F[:,1],F[:,7],sc,hs]))):
    p=fit(Xm,syn); thr=np.sort(p[syn&(tr==1)])[0]; sk=real&(p<thr)
    print(f"  {name:22s} AUC real {auc(tr[real],p[real]):.4f} | generated-set threshold on real: lost {int(tr[sk].sum())} of {int(tr[real].sum())}, passes saved {100*passes[sk].sum()/passes[real].sum():5.1f}% (a {100*passes[sk&(corp=='a')].sum()/passes[corp=='a'].sum():.1f}%, b {100*passes[sk&(corp=='b')].sum()/passes[corp=='b'].sum():.1f}%)")
# leave one generated family out: train on the others, judge on that family (zero-loss threshold from the training families)
print("\n  leave-one-family-out (cheap model), generated families:")
Xm=cheap
for fam in ('busy','dense','sweep','itu'):
    trm=syn&(corp!=fam); tem=corp==fam
    p=fit(Xm,trm); thr=np.sort(p[trm&(tr==1)])[0]; sk=tem&(p<thr)
    print(f"    held out {fam:6s}: lost {int(tr[sk].sum()):3d} of {int(tr[tem].sum()):5d}, passes saved {100*passes[sk].sum()/passes[tem].sum():5.1f}%")

print("\n\n===== the coefficients to ship: the cheap model on the raw scale, trained on the generated sets only =====")
# z = bias + sum(coef_i * x_i) on the raw features x = [mean, std, max, mean/std, score, hs]; the standardisation is folded into the coefficients
Xc = cheap
mu = Xc[syn].mean(0); sd = Xc[syn].std(0) + 1e-9
Zc = np.column_stack([(Xc - mu) / sd, np.ones(len(Xc))]); wv = np.zeros(Zc.shape[1])
for it in range(6000):
    pp = 1 / (1 + np.exp(-(Zc[syn] @ wv))); wv -= 0.5 * (Zc[syn].T @ (pp - tr[syn]) / syn.sum() + 1e-4 * wv)
coef = wv[:-1] / sd
bias = wv[-1] - float((wv[:-1] * mu / sd).sum())
z = Xc @ coef + bias
zmin = z[syn & (tr == 1)].min()
print("names :", ["mean|llra|", "std|llra|", "max|llra|", "mean/std", "score", "hard sync"])
print("coef  :", [float(f"{c:.6g}") for c in coef])
print("bias  :", float(f"{bias:.6g}"), " lowest logit of a true decode in the generated sets:", float(f"{zmin:.4f}"))
for margin in (0.0, 0.5, 1.0):
    thr = zmin - margin
    sk = z < thr
    print(f"  threshold {thr:+.4f} (margin {margin}): lost on generated {int(tr[sk & syn].sum())}, on real {int(tr[sk & real].sum())} of {int(tr[real].sum())}; passes saved: generated {100*passes[sk & syn].sum()/passes[syn].sum():.1f}%, real {100*passes[sk & real].sum()/passes[real].sum():.1f}%")
