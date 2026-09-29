#!/usr/bin/env python3
# Design numbers for docs/notes/IQ_CHANNELIZER.md (#534): Direct cost from the
# code's own filter rules, PFB parameters per rate, prototype and back-end
# filters (Kaiser, verified), totals. Needs numpy + scipy.
import numpy as np, math
from scipy import signal
# Guaranteed 120 dB, designed with the 3 dB margin mfsk_core::iq uses.
ATTEN = 123
from math import gcd

# ---------- 1. Direct (IqToAudio) cost per channel, from the code's own rules ----------
PASS, STOP = 2800.0, 3200.0
def taps_for(tr): return max(15, math.ceil(5.5/tr)) | 1
def plan(fs):
    dmax = max(1, fs//24000)
    for d in range(dmax, 0, -1):
        num = 12000*d; g = gcd(num, fs); l, m = num//g, fs//g
        if l <= 2048: return d, l, m
def stage_factors(d):
    pr=[]; p=2
    while d>1:
        while d%p==0: pr.append(p); d//=p
        p+=1
        if p*p>d and d>1: pr.append(d); break
    pr.sort(reverse=True); st=[]
    for x in pr:
        for i,s in enumerate(st):
            if s*x<=10: st[i]*=x; break
        else: st.append(x)
    return st
def kaiser_n(A, tr):
    n = math.ceil((A-7.95)/(2.285*2*math.pi*tr)) + 1
    return max(3, n) | 1
def direct_cost(fs):
    """mfsk_core::iq::IqToAudio since the Kaiser rework: NCO at Fs, Kaiser
    stages (pass 3.2 k, stop out-3.2 k), L/M to 12 k complex (stop 8.8 k),
    sharp filter at 12 k (pass 2.8 k, stop 3.2 k), all at ATTEN."""
    d,l,m = plan(fs); rate=fs; parts=[]
    parts.append(("NCO @Fs", fs*6))
    for f in stage_factors(d):
        out=rate/f; t=kaiser_n(ATTEN,(out-2*STOP)/rate)
        parts.append((f"FIR /{f} @{rate/1e3:.0f}k ({t} taps)", out*t*4)); rate=out
    if not (l==1 and m==1):
        nt=kaiser_n(ATTEN,(12000-2*STOP)/(rate*l)); nt=math.ceil((nt-1)/(2*m))*2*m+1
        parts.append((f"L/M {l}/{m} ({nt} taps)", 12000*math.ceil(nt/l)*4))
    t=kaiser_n(ATTEN,(STOP-PASS)/12000); parts.append((f"sharp @12k ({t} taps)", 12000*t*4))
    return parts

print("== Direct, flops per second per channel ==")
for fs in (192000, 768000, 2400000):
    p=direct_cost(fs); tot=sum(v for _,v in p)
    print(f"Fs {fs}: total {tot/1e6:.1f} MFLOP/s")
    for n,v in p: print(f"    {n:32s} {v/1e6:6.1f}  ({100*v/tot:4.1f}%)")

# ---------- 2. PFB parameters per rate ----------
def pfb_params(fs):
    best=None
    for M in range(2, 4097, 2):
        S=fs/M; R=2*S
        if not (20000 <= S <= 32000): continue
        if (2*fs) % M: continue
        R=2*fs//M
        score=(R%12000!=0, abs(S-24000))
        if best is None or score<best[0]: best=(score,M,S,R)
    return best
print("\n== PFB parameters (2x oversampled) ==")
print(f"{'Fs':>9} {'M':>5} {'spacing':>9} {'out rate':>9} {'R/12k':>7} {'Bp(≥)':>8} {'Bs(≤)':>8}")
for fs in (96000,192000,250000,384000,768000,912000,1024000,2048000,2400000,2500000,3200000,6000000,10000000):
    b=pfb_params(fs)
    if not b: print(f"{fs:>9}  none"); continue
    _,M,S,R=b; Bp=S/2+3200+300; Bs=R-Bp
    ratio = f"{R//12000}" if R%12000==0 else f"{R/12000:.3f}"
    print(f"{fs:>9} {M:>5} {S:>9.0f} {R:>9} {ratio:>7} {Bp:>8.0f} {Bs:>8.0f}")

# ---------- 3. Prototype design and verification (768k, M=32) ----------
def kaiser_proto(fs, M, Bp, Bs, atten):
    ntaps, beta = signal.kaiserord(atten, (Bs-Bp)/(fs/2))
    K = math.ceil((ntaps-1)/M)+1          # L' = M(K-1)+1 >= ntaps
    Lp = M*(K-1)+1
    h = signal.firwin(Lp, (Bp+Bs)/2, window=('kaiser', beta), fs=fs)
    return h, K, Lp, beta
def resp(h, fs, f):
    w=2*np.pi*np.asarray(f)/fs; n=np.arange(len(h))
    return np.abs(np.exp(-1j*np.outer(w,n))@h)
print("\n== Prototype, verified ==")
for fs in (192000, 768000, 2400000):
    _,M,S,R=pfb_params(fs); Bp=S/2+3500; Bs=R-Bp
    h,K,Lp,beta=kaiser_proto(fs,M,Bp,Bs,ATTEN)
    fpass=np.linspace(0,Bp,400); fstop=np.linspace(Bs,fs/2,4000)
    rp=20*np.log10(resp(h,fs,fpass)); rs=20*np.log10(resp(h,fs,fstop).max())
    print(f"Fs {fs} M={M} K={K} L'={Lp} beta={beta:.2f}: pass ripple {rp.max()-rp.min():.4f} dB, worst stop {rs:.1f} dB, group delay {(Lp-1)//2} in = {(Lp-1)//2/(M//2):.0f} hops")

# ---------- 4. Back end (at R=48k): decimate to 12k complex, sharp filter, shift up ----------
print("\n== Back end filters, verified ==")
R=48000
nt,beta=signal.kaiserord(ATTEN,(8800-3200)/(R/2)); nt|=1
hd=signal.firwin(nt,(3200+8800)/2,window=('kaiser',beta),fs=R)
rp=20*np.log10(resp(hd,R,np.linspace(0,3200,200))); rs=20*np.log10(resp(hd,R,np.linspace(8800,24000,2000)).max())
print(f"decimate 48k->12k: {nt} taps, ripple {rp.max()-rp.min():.4f} dB, stop {rs:.1f} dB")
nt2,beta2=signal.kaiserord(ATTEN,(3200-2800)/(12000/2)); nt2|=1
hs=signal.firwin(nt2,3000,window=('kaiser',beta2),fs=12000)
rp=20*np.log10(resp(hs,12000,np.linspace(0,2800,200))); rs=20*np.log10(resp(hs,12000,np.linspace(3200,6000,2000)).max())
print(f"sharp @12k complex: {nt2} taps, ripple {rp.max()-rp.min():.4f} dB, stop {rs:.1f} dB")
# per-channel flops at R=48k
be = R*6 + 12000*nt*4 + 12000*nt2*4 + 12000*2
print(f"back end per channel: {be/1e6:.1f} MFLOP/s (NCO {R*6/1e6:.2f}, decim {12000*nt*4/1e6:.1f}, sharp {12000*nt2*4/1e6:.1f})")

# ---------- 5. PFB fixed cost, and totals vs Direct ----------
print("\n== Totals, MFLOP/s (Direct measured: 0.93% of a core per channel at 768k, 2.42% at 2.4M) ==")
for fs in (768000, 2400000):
    _,M,S,R=pfb_params(fs); Bp=S/2+3500; Bs=R-Bp
    h,K,Lp,beta=kaiser_proto(fs,M,Bp,Bs,ATTEN)
    hop=M//2
    fft = 5*M*math.log2(M) if (M & (M-1))==0 else 5*M*math.log2(M)*1.5
    pfb = fs/hop*(M*K*4 + fft)
    Rr=2*fs//M
    be_rate = Rr*6 + 12000*nt*4*(Rr/48000) + 12000*nt2*4 + 12000*2
    d = sum(v for _,v in direct_cost(fs))
    print(f"Fs {fs}: PFB fixed {pfb/1e6:.0f}, back end/ch {be_rate/1e6:.1f}, Direct/ch {d/1e6:.1f}")
    for n in (1,2,3,4,8,32,128):
        print(f"    {n:>4} ch: PFB {(pfb+n*be_rate)/1e6:8.0f}   Direct {n*d/1e6:8.0f}   ratio {n*d/(pfb+n*be_rate):5.2f}x")
