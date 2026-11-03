#!/usr/bin/env python3
"""How deeply does each decoder subtract a signal it has just decoded?  (jt9's subtractft8 vs ours)

usage: resid_compare.py <wav> <dir> <jt9 dump name> <our dump name> <jt9 pass> <stage> <round>
  <dir>/jt9/<jt9 dump>      resid_41_1.bin  float32, 180000 samples (JT9_TRACE build)
  <dir>/jt9/ft8_trace.dat   <dir>/ours/<our dump> resid_A_0.bin int16, 180000 (MFSK_FT8_DUMP_RESID)
  <dir>/ours/ours.rtrace    (MFSK_FT8_TRACE)
Needs numpy (run in normal mode from a directory with no stray modules; reads only data).

For every message both decoders made in that search: the power in the signal's band (f .. f+50 Hz)
over its time span (DT .. DT+12.64 s, clipped to the audio the stage has), original, and what is left
after the search, in dB below the original.  Then where the two residuals differ most.
"""
import sys, re, wave
import numpy as np

wav, d, jname, oname, jpass, stage, rnd = sys.argv[1:8]
jpass, rnd = int(jpass), int(rnd)
FS = 12000
w = wave.open(wav); orig = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)
A = np.fromfile(f"{d}/jt9/{jname}", dtype="<f4").astype(np.float64)
B = np.fromfile(f"{d}/ours/{oname}", dtype="<i2").astype(np.float64)
n = len(A)
orig = orig[:n]
limit = 141696 if stage == "A" else n          # stage A has audio only up to 11.808 s

norm = lambda m: re.sub(r"\s+", " ", m.strip()).upper()
jt, nz, op = {}, 0, 0
want_nz = 41 if stage == "A" else 50
for l in open(f"{d}/jt9/ft8_trace.dat"):
    t = l.split()
    if not t: continue
    if t[0] == "PASS": nz, op = int(t[1]), int(t[2])
    elif t[0] == "DEC" and nz == want_nz and op == jpass:
        jt.setdefault(norm(" ".join(t[11:])), (float(t[9]), float(t[10])))
ours, cur_round = {}, 0
for l in open(f"{d}/ours/ours.rtrace"):
    if l.startswith("CAND "): cur_round = int(l.split("round=")[1].split()[0])
    elif l.startswith("DEC ") and l.split()[1] == stage and cur_round == rnd:
        h, _, m = l.rstrip().partition(" msg="); kv = dict(re.findall(r"(\w+)=(\S+)", h))
        ours.setdefault(norm(m), (float(kv["f"]), float(kv["dt"])))

def band_power(x, f0, t0, t1):
    seg = x[int(t0 * FS):int(t1 * FS)]
    if len(seg) < 4096: return None
    X = np.fft.rfft(seg * np.hanning(len(seg)))
    fr = np.fft.rfftfreq(len(seg), 1 / FS)
    sel = (fr >= f0 - 2.0) & (fr <= f0 + 52.0)
    return float((np.abs(X[sel]) ** 2).sum())

print(f"decoded in this search: jt9 {len(jt)}, ours {len(ours)}, both {len(set(jt) & set(ours))}")
print(f"{'message':24s} {'f':>7s} {'dt':>6s} | {'orig':>9s} | {'jt9 left':>9s} {'ours left':>9s} | residual power (dB re original; lower = deeper)")
rows = []
for m in sorted(set(jt) & set(ours)):
    f, dt = jt[m]
    t0 = max(dt + 0.5, 0.0); t1 = min(dt + 0.5 + 12.64, limit / FS)
    po = band_power(orig, f, t0, t1); pa = band_power(A, f, t0, t1); pb = band_power(B, f, t0, t1)
    if None in (po, pa, pb) or po <= 0: continue
    ja, ob = 10 * np.log10(pa / po), 10 * np.log10(pb / po)
    rows.append((m, f, dt, 10 * np.log10(po), ja, ob))
for m, f, dt, po, ja, ob in rows:
    print(f"{m:24s} {f:7.1f} {dt:+6.2f} | {po:7.1f} dB | {ja:8.1f}  {ob:8.1f}   | ours is {'DEEPER' if ob < ja - 0.5 else 'SHALLOWER' if ob > ja + 0.5 else 'same'} by {ja - ob:+.1f} dB")
if rows:
    dj = np.array([r[4] for r in rows]); do = np.array([r[5] for r in rows])
    print(f"\nmean residual: jt9 {dj.mean():.1f} dB, ours {do.mean():.1f} dB;  ours deeper in {(do < dj - 0.5).sum()}, shallower in {(do > dj + 0.5).sum()}, same in {(abs(do - dj) <= 0.5).sum()} of {len(rows)}")

# where do the two residuals differ?  energy of (jt9 - ours) in 50 Hz bands over the usable part
diff = (A - B)[:limit]
X = np.fft.rfft(diff * np.hanning(len(diff))); fr = np.fft.rfftfreq(len(diff), 1 / FS)
P = np.abs(X) ** 2
Xa = np.fft.rfft(A[:limit] * np.hanning(limit)); Pa = np.abs(Xa) ** 2
bands = []
for lo in range(200, 3000, 50):
    sel = (fr >= lo) & (fr < lo + 50)
    bands.append((lo, P[sel].sum(), Pa[sel].sum()))
tot = sum(b[1] for b in bands)
print(f"\nenergy of (jt9 residual - our residual), 200-3000 Hz: top 50 Hz bands (share of the total; relative to the jt9 residual in the same band)")
for lo, p, pa in sorted(bands, key=lambda b: -b[1])[:8]:
    print(f"  {lo:5d}-{lo+50:<5d} Hz  {100*p/tot:5.1f} %   difference is {10*np.log10(p/pa):+6.1f} dB re the jt9 residual there")
print("decoded signals' bands in this search (jt9):", ", ".join(f"{int(f)}" for _, (f, _) in sorted(jt.items(), key=lambda kv: kv[1][0])))
