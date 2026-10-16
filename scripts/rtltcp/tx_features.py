# usage: python3 scripts/rtltcp/tx_features.py CORPUS_DIR ROWS_DIR JOB MODE DIAL [SNR_MIN]   (needs numpy)
# Joins truth.json (CORPUS_DIR/seedN) with the rows of JOB (ROWS_DIR/JOB.seedN.json, eval_synth.py output) per
# transmission: loss inside its frame (samples lost, holes, reset, peak deviation from a straight line), time since
# the last reset, decoded or not. Prints, for steady-state transmissions (>40 s from start/reset, no reset inside)
# at SNR >= SNR_MIN, the hit rate by the "mismatch" = samples lost in the frame minus what the client's uniform
# stretch assumes (1 - rate/1 024 000 of the frame), in ms. RTLTCP_EVAL_PLAN.md section 11.
# TXF_END=S (environment): ignore transmissions that end after S seconds (section 12: the trace's loss-free tail).
import json, os, sys
import numpy as np
def load(corpus,rd,name,seed,mode,dial):
    T=json.load(open(f"{corpus}/seed{seed}/truth.json")); R=json.load(open(f"{rd}/{name}.seed{seed}.json"))
    t0=T['stream']['start_utc_ns']/1e9; RATE=1024000
    L=T['loss']; st=np.array([l['start_sample'] for l in L]); ln=np.array([l['len'] for l in L]); kind=[l['kind'] for l in L]
    cum=np.concatenate([[0],np.cumsum(ln)])
    def lost_before(s):  # lost samples with source index < s
        i=np.searchsorted(st,s,side='right'); tot=cum[i-1] if i>0 else 0
        # partial overlap of event i-1
        if i>0:
            e=st[i-1]+ln[i-1]
            if s<e: tot=cum[i-1]-(e-s)
        return tot
    resets=[l['start_sample']/RATE for l in L if l['kind']=='reset']
    dur={'FT8':12.64,'FT4':5.04}[mode]
    skip=R['skip_s']; end=min(R['end_s'], float(os.environ.get('TXF_END', 1e9)))
    rows={}
    for r in R['rows']:
        if int(r['dial'])==dial and r['mode']==mode: rows[(r['text'],round(r['utc']/1e9))]=r
    out=[]
    for t in T['transmissions']:
        if t['dial_hz']!=dial or t['mode']!=mode: continue
        k=(t['text'],round(t['slot_utc_s']))
        a=t['slot_utc_s']-t0+0.5+t['dt_s']; b=a+dur
        s0=int(a*RATE); s1=int(b*RATE)
        i0=np.searchsorted(st,s0,side='left'); i1=np.searchsorted(st,s1,side='right')
        evs=[(st[j],ln[j],kind[j]) for j in range(max(i0-1,0),i1) if st[j]+ln[j]>s0 and st[j]<s1]
        lost=lost_before(s1)-lost_before(s0)
        # deviation from local linear (detrended) of delivered-time vs source-time, sampled each 10 ms
        xs=np.arange(s0,s1,RATE//100); y=np.array([lost_before(x) for x in xs],float)-lost_before(s0)
        x=(xs-s0)/RATE; p=np.polyfit(x,y,1); dev=(y-np.polyval(p,x))/RATE*1000
        # deviation vs the global-rate stretch (client): r_glob = total lost / total
        rg=T['stats']['lost_samples']/T['stats']['source_samples']
        devg=(y-rg*(xs-s0))/RATE*1000
        rr=min([abs(a-x_) for x_ in resets]+[1e9])
        out.append(dict(mode=mode,snr=t['snr_db'],slot=t['slot_utc_s']-t0,par=int(round((t['slot_utc_s']-t0)/ {'FT8':15,'FT4':7.5}[mode]))%2,
          off15=(t['slot_utc_s']-t0)%15,dt=t['dt_s'],start=a,hit=k in rows,r=rows.get(k),
          nholes=sum(1 for e in evs if e[2]=='hole'),reset=any(e[2]=='reset' for e in evs),lost=lost/RATE*1000,
          devpp=float(dev.max()-dev.min()),devmax=float(np.abs(dev).max()),devg=float(np.abs(devg).max()),
          since_reset=min([a-x_ for x_ in resets if x_<=a]+[1e9]),
          inwin=(a>=skip and b<=end)))
    return [o for o in out if o['inwin']]
def main():
    corpus, rd, name, mode, dial = sys.argv[1:6]
    snr_min = float(sys.argv[6]) if len(sys.argv) > 6 else -6
    dur = {"FT8": 12.64, "FT4": 5.04}[mode]
    P = []
    for seed in (1, 2, 3):
        d = 1 - json.load(open(f"{rd}/{name}.seed{seed}.json"))["rate"] / 1024000
        for x in load(corpus, rd, name, seed, mode, int(dial)):
            if x["snr"] >= snr_min and not x["reset"] and x["since_reset"] > 40 and x["start"] > 40:
                P.append((x["lost"] - d * dur * 1000, x["hit"]))
    P = np.array(P)
    print(f"n {len(P)}  hit {P[:,1].mean():.3f}  mismatch std {P[:,0].std():.1f} ms")
    edges = np.quantile(P[:, 0], np.linspace(0, 1, 9))
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (P[:, 0] >= lo) & (P[:, 0] <= hi)
        print(f"  mismatch {lo:7.1f}..{hi:7.1f} ms: hit {P[m,1].mean():.2f} (n{m.sum()})")


if __name__ == "__main__":
    main()
