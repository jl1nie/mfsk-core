#!/usr/bin/env python3
"""Remake one signal of the dense corpus alone: same frequency, DT, SNR, and the same noise draw
as in its file, none of the other signals. Tells crowding (alone it decodes) from the decoder's
own limit on that signal (alone it still fails).

usage: solo.py <ft8sim> <truth.csv> <out-dir> <file>:<message> ... [--boost DB]
Writes <out-dir>/solo_<file stem>__<n>.wav. --boost raises the signal by DB (the noise is kept).
"""
import csv, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import gen_ft8_busy_wavs as busy
import gen_ft8_dense_wavs as dense

ft8sim, truth, out = Path(sys.argv[1]).resolve(), sys.argv[2], Path(sys.argv[3])
args = sys.argv[4:]
boost = 0.0
if "--boost" in args:
    i = args.index("--boost"); boost = float(args[i + 1]); del args[i:i + 2]
out.mkdir(parents=True, exist_ok=True)
rows = list(csv.DictReader(open(truth)))
for a in args:
    f, msg = a.split(":", 1)
    r = next(x for x in rows if x["file"] == f and x["msg"] == msg)
    k = int(f.rsplit("_", 1)[1][:-4])
    sent, u = busy.unit_signal(ft8sim, r["msg"], float(r["f0"]), 0.0)
    snr = float(r["snr"]) + boost
    total = busy.SIG_SCALE * 10.0 ** (0.05 * snr) * dense.shifted(u, float(r["dt"]))
    total += np.random.default_rng([1, k, 9]).standard_normal(busy.NMAX)     # the file's own noise
    busy.write_wav(out / f"solo_{f[:-4]}__{abs(hash(msg)) % 1000}.wav", np.clip(np.rint(busy.GAIN * total), -32768, 32767))
    print(f"{f[:-4]} {msg!r} f0={r['f0']} dt={r['dt']} snr={snr:+.1f}")
