#!/usr/bin/env python3
# Whole-chain response of the PFB channel (docs/notes/IQ_CHANNELIZER.md, #534):
# in-window flatness over every window position, and the worst interferer that
# lands in the window after every fold. Needs numpy + scipy.
import numpy as np, math
from scipy import signal
# Guaranteed 120 dB, designed with the 3 dB margin mfsk_core::iq uses.
ATTEN = 123
fs, M = 768000, 32
S = fs/M; R = 2*S
Bp, Bs = S/2+3500, R-(S/2+3500)
nt,beta = signal.kaiserord(ATTEN,(Bs-Bp)/(fs/2)); K=math.ceil((nt-1)/M)+1; Lp=M*(K-1)+1
hp = signal.firwin(Lp,(Bp+Bs)/2,window=('kaiser',beta),fs=fs)
nd,bd = signal.kaiserord(ATTEN,(8800-3200)/(R/2)); nd|=1
hd = signal.firwin(nd,(3200+8800)/2,window=('kaiser',bd),fs=R)
ns,bs = signal.kaiserord(ATTEN,(400)/(6000)); ns|=1
hs = signal.firwin(ns,3000,window=('kaiser',bs),fs=12000)
def H(h, rate, f):
    f=np.atleast_1d(f); n=np.arange(len(h)); w=2*np.pi*f/rate
    return np.abs(np.exp(-1j*np.outer(w,n))@h)
def wrap(f, r): return (f + r/2) % r - r/2
def chain(x, r):
    """Tone at x Hz from the window centre, window centre r Hz from its sub-band centre."""
    f1 = x + r                      # relative to sub-band centre, at Fs
    g1 = H(hp, fs, f1)
    f2 = wrap(wrap(f1, R) - r, R)   # after decimation to R and the residual NCO
    g2 = H(hd, R, f2)
    f3 = wrap(f2, 12000)            # after /4 to 12 kHz
    g3 = H(hs, 12000, f3)
    return g1*g2*g3, f3
# In-window gain across window positions, including across the sub-band edge.
rs = np.linspace(-S/2, S/2, 97)
g = np.array([chain(np.array([0.0, -2800.0, 2800.0]), r)[0] for r in rs])
print(f"in-window gain over r in [-S/2,S/2]: min {20*np.log10(g.min()):.4f} dB max {20*np.log10(g.max()):.4f} dB")
# Worst leakage: interferers outside the window (|x| >= 3200), anywhere in ±Fs/2,
# landing inside the used window (|f3| <= 2800) -- over every window position.
worst=-400; where=None
xs = np.concatenate([np.arange(-fs/2, -3200, 3.7), np.arange(3200, fs/2, 3.7)])
for r in np.linspace(-S/2, S/2, 25):
    lvl, f3 = chain(xs, r)
    m = np.abs(f3) <= 2800
    if m.any():
        i = np.argmax(np.where(m, lvl, 0)); v = 20*np.log10(max(lvl[i],1e-30))
        if v > worst: worst=v; where=(r, xs[i], f3[i])
print(f"worst leakage into the window: {worst:.1f} dB (window {where[0]:.0f} Hz off sub-band centre, interferer {where[1]:.0f} Hz from window centre, lands at {where[2]:.0f} Hz)")
# Near-in: interferers 3.2k..6k from the window centre (the sharp filter's job).
lvl, f3 = chain(np.concatenate([np.arange(-6000,-3200,1.3), np.arange(3200,6000,1.3)]), 0.0)
print(f"near-in (3.2-6 kHz off centre) worst output: {20*np.log10(lvl.max()):.1f} dB")
