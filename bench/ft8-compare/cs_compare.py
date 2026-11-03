#!/usr/bin/env python3 -I
"""Compare jt9's complex symbol spectra `cs(0:7,79)` with mfsk-core's for one candidate.

usage: cs_compare.py <jt9 trace with CSHDR/CS> <ours rtrace with CSHDR/CS> <jt9 nz> <jt9 imetric> <our stage> <our imetric> [<skip jt9 blocks> <skip our blocks>]
Both dumps are of the candidate at the same refined frequency. Reports, over the 79 symbols x 8 tones:
  - magnitude: correlation and the scale (ours/jt9) of |cs|
  - phase of the per-element ratio ours/jt9, weighted by magnitude: its mean, its slope along the symbol
    index k (a frequency/time offset), its slope along the tone index (a filter's group delay / a timing
    offset), and the residual rms after removing those
  - the same restricted to the dominant (strongest) tone of each symbol
"""
import sys, math, cmath
def read(path, pred):
    blocks, cur, hdr = [], None, None
    for l in open(path):
        t = l.split()
        if not t: continue
        if t[0] == "CSHDR":
            cur = []; blocks.append((l.strip(), cur))
        elif t[0] == "CS" and cur is not None:
            v = [float(x) for x in t[2:]]
            cur.append([complex(v[2*i], v[2*i+1]) for i in range(8)])
    return [(h, b) for h, b in blocks if len(b) == 79 and pred(h)]
jt_nz, jt_im, st, our_im = sys.argv[3:7]
jt = read(sys.argv[1], lambda h: h.split()[1] == jt_nz and h.split()[2] == jt_im)
ours = read(sys.argv[2], lambda h: h.split()[1] == st and f"imetric={our_im}" in h)
print("jt9 :", jt[0][0]); print("ours:", ours[0][0])
a, b = jt[0][1], ours[0][1]
mag_a = [abs(a[k][t]) for k in range(79) for t in range(8)]
mag_b = [abs(b[k][t]) for k in range(79) for t in range(8)]
ma, mb = sum(mag_a)/len(mag_a), sum(mag_b)/len(mag_b)
cov = sum((x-ma)*(y-mb) for x, y in zip(mag_a, mag_b))
r = cov / math.sqrt(sum((x-ma)**2 for x in mag_a) * sum((y-mb)**2 for y in mag_b))
scale = sum(y*x for x, y in zip(mag_a, mag_b)) / sum(x*x for x in mag_a)
print(f"|cs| correlation {r:.4f}, scale ours/jt9 {scale:.3f}")
def fit(pts):
    """weighted least squares phase = c + sk*k + st*t over (k, t, phase, weight), phases unwrapped by the caller's reference."""
    import itertools
    sw = sum(w for *_, w in pts)
    mk = sum(k*w for k, t, p, w in pts)/sw; mt = sum(t*w for k, t, p, w in pts)/sw; mp = sum(p*w for k, t, p, w in pts)/sw
    skk = sum(w*(k-mk)**2 for k, t, p, w in pts); stt = sum(w*(t-mt)**2 for k, t, p, w in pts); skt = sum(w*(k-mk)*(t-mt) for k, t, p, w in pts)
    skp = sum(w*(k-mk)*(p-mp) for k, t, p, w in pts); stp = sum(w*(t-mt)*(p-mp) for k, t, p, w in pts)
    det = skk*stt - skt*skt
    sk = (skp*stt - stp*skt)/det if det else 0.0; stv = (stp*skk - skp*skt)/det if det else 0.0
    c = mp - sk*mk - stv*mt
    res = math.sqrt(sum(w*(p-(c+sk*k+stv*t))**2 for k, t, p, w in pts)/sw)
    return c, sk, stv, res
def pts(sel):
    out = []
    for k in range(79):
        row = a[k]; dom = max(range(8), key=lambda t: abs(row[t]))
        for t in range(8):
            if sel == "dom" and t != dom: continue
            z = b[k][t] * row[t].conjugate()
            if abs(z) > 0: out.append((k, t, cmath.phase(z), abs(z)))
    return out
for name, sel in (("all elements, magnitude-weighted", "all"), ("dominant tone of each symbol", "dom")):
    p = pts(sel)
    # reference phase: weighted mean direction, then unwrap about it
    ref = cmath.phase(sum(cmath.rect(w, ph) for _, _, ph, w in p))
    p = [(k, t, (ph - ref + math.pi) % (2*math.pi) - math.pi, w) for k, t, ph, w in p]
    c, sk, stv, res = fit(p)
    raw = math.sqrt(sum(w*ph*ph for _, _, ph, w in p)/sum(w for *_, w in p))
    print(f"{name}: mean phase {math.degrees(ref):+7.2f} deg; slope per symbol {math.degrees(sk):+.3f} deg; slope per tone {math.degrees(stv):+.3f} deg; rms spread {math.degrees(raw):.2f} -> {math.degrees(res):.2f} deg after removing slopes")
