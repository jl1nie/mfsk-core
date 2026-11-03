#!/usr/bin/env python3
"""Generate the FT8 "dense band" corpus: busy-band files whose crowding is drawn from
what real recordings look like, one factor at a time.

Why it exists. `ft8_busy` (gen_ft8_busy_wavs.py) has SNR -24..-6 dB, signals at least
12 Hz apart and DT -0.5..+1.5 s. On the 22 real FT8 recordings of ft8_lib (13 WebSDR,
9 baseline) the signals `jt9 -8 -d 3` decodes have SNR -25..+25 dB (median -2, 59 % above
-6), 32 of 379 adjacent pairs closer than 12 Hz (17 closer than 6.25 Hz) and DT -1.5..+2.0 s
(5th..95th percentile -0.8..+1.6). On those recordings mfsk-core finds 383 of the 401
messages jt9 does, though on `ft8_busy` it matches jt9. This corpus has the three things
`ft8_busy` leaves out, each alone and then all together, with the transmitted messages
known, so a gap can be attributed by measurement and not by guess.

Sets (40 files each; every set differs from `base` in the stated factor only):
    base     20 signals, SNR U(-24,-6) dB, >= 12 Hz apart, DT U(-0.5,+1.5)   (control: `busy20`)
    snr      SNR drawn from the real recordings' histogram (-25..+25 dB)
    gap      10 % of the signals sit 0.5..12 Hz from an earlier one (real: 32/379 = 8 %)
    dt       DT U(-1.0,+2.0) s; a signal is shifted linearly and cut at the file's ends
    real     all three, and the number of signals per file drawn from the real counts (1..32)

Output: <out-dir>/ft8_dense_<set>_<NN>.wav and <out-dir>/truth.csv (file,msg,f0,dt,snr),
the same columns as `ft8_busy`, so scripts/score-jt9-busy.py reads it. Needs numpy and an
ft8sim (3.x); every waveform is ft8sim's, SNR 99, rescaled as gen_ft8_busy_wavs.py does,
and the SNR calibration is that script's. Deterministic: `--seed` seeds everything.

    scripts/gen_ft8_dense_wavs.py <ft8sim> <out-dir> [--seed 1] [--only SET ...]
"""
import argparse
import csv
import datetime
import hashlib
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_ft8_busy_wavs as busy  # noqa: E402  (waveforms, messages, wav i/o, calibration)

FS, NMAX, GAIN, SIG_SCALE = busy.FS, busy.NMAX, busy.GAIN, busy.SIG_SCALE

# From the 22 recordings, jt9 -8 -d 3 (docs/notes/FT8_REAL_COMPARE.md): lower edge of a 5 dB bin
# -> number of decodes in it. Uniform inside a bin.
REAL_SNR_BINS = {-25: 11, -20: 41, -15: 52, -10: 59, -5: 62, 0: 56, 5: 64, 10: 29, 15: 17, 20: 9, 25: 1}
REAL_COUNTS = [1, 6, 2, 5, 5, 20, 19, 17, 17, 19, 17, 26, 21, 19, 23, 15, 28, 30, 30, 32, 26, 23]

BASE_SNR = (-24.0, -6.0)
BASE_DT = (-0.5, 1.5)
WIDE_DT = (-1.0, 2.0)
CLOSE_FRACTION = 0.10
CLOSE_RANGE_HZ = (0.5, 12.0)
F_RANGE = busy.F_RANGE
N_BASE = 20

# name -> (snr, gap, dt, n_from_real)
SETS = {
    "base": (False, False, False, False),
    "snr": (True, False, False, False),
    "gap": (False, True, False, False),
    "dt": (False, False, True, False),
    "real": (True, True, True, True),
}
N_FILES = 40


def draw_snr(rng, real):
    if not real:
        return round(float(rng.uniform(*BASE_SNR)), 1)
    lo = list(REAL_SNR_BINS)
    w = np.array([REAL_SNR_BINS[b] for b in lo], dtype=float)
    b = lo[rng.choice(len(lo), p=w / w.sum())]
    return round(float(rng.uniform(b, b + 5)), 1)


def draw_frequency(rng, taken, close):
    """A frequency in F_RANGE: >= 12 Hz from every taken one, or (close) 0.5..12 Hz from one."""
    if close and taken:
        for _ in range(100):
            base = taken[rng.integers(len(taken))]
            f = round(base + rng.choice([-1, 1]) * rng.uniform(*CLOSE_RANGE_HZ), 1)
            if F_RANGE[0] <= f <= F_RANGE[1] and f not in taken:
                return f
    while True:
        f = float(np.round(rng.uniform(*F_RANGE), 1))
        if all(abs(f - g) >= busy.MIN_SPACING_HZ for g in taken):
            return f


def shifted(u, dt):
    """`u` is ft8sim's waveform at DT 0; move it by dt seconds, zero-filled, cut at the ends."""
    k = int(round(dt * FS))
    out = np.zeros(NMAX)
    if k >= 0:
        out[k:] = u[: NMAX - k]
    else:
        out[: NMAX + k] = u[-k:]
    return out


def write_stamp(out_dir, ft8sim, seed):
    root = Path(__file__).resolve().parent.parent
    git = lambda *a: subprocess.run(["git", "-C", str(root), *a], capture_output=True, text=True)
    dirty = 0 if git("diff", "--quiet", "HEAD", "--", "scripts").returncode == 0 else 1
    (out_dir / ".corpus-stamp").write_text(
        f"# Written by {Path(__file__).name}. Do not hand-edit.\n"
        f"generator={Path(__file__).name}\nrng=seeded\nseed={seed}\nsimulator={ft8sim}\n"
        f"simulator_sha256={hashlib.sha256(ft8sim.read_bytes()).hexdigest()}\n"
        f"commit={git('rev-parse', 'HEAD').stdout.strip() or 'unknown'}\nscripts_dirty={dirty}\n"
        f"date={datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')}\n"
        f"host={platform.node()}\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ft8sim", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--only", nargs="*", choices=sorted(SETS))
    args = ap.parse_args()
    ft8sim = args.ft8sim.resolve()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for name, (r_snr, r_gap, r_dt, r_n) in SETS.items():
        if args.only and name not in args.only:
            continue
        for k in range(1, N_FILES + 1):
            # Paired design: every set uses the same (seed, k) and one stream per kind of draw,
            # so file k of `snr` is file k of `base` with only its SNRs redrawn (same messages,
            # frequencies, DTs), and likewise `dt`. `gap` moves the frequencies of the signals
            # it makes close; `real` changes everything and the count.
            r_f, r_s, r_d, r_m, r_n_ = (np.random.default_rng([args.seed, k, j]) for j in range(5))
            rng = r_n_
            fname = f"ft8_dense_{name}_{k:02d}.wav"
            n = int(REAL_COUNTS[r_n_.integers(len(REAL_COUNTS))]) if r_n else N_BASE
            total = np.zeros(NMAX)
            taken = []
            for _ in range(n):
                close = r_gap and r_f.random() < CLOSE_FRACTION
                f0 = draw_frequency(r_f, taken, close)
                taken.append(f0)
                dt = round(float(r_d.uniform(*(WIDE_DT if r_dt else BASE_DT))), 2)
                snr = draw_snr(r_s, r_snr)
                sent, u = busy.unit_signal(ft8sim, busy.message(r_m), f0, 0.0)
                total += SIG_SCALE * 10.0 ** (0.05 * snr) * shifted(u, dt)
                rows.append((fname, sent, f0, dt, snr))
            total += np.random.default_rng([args.seed, k, 9]).standard_normal(NMAX)
            w = np.rint(GAIN * total)
            if np.abs(w).max() > 32767:
                print(f"warning: {fname} clips", file=sys.stderr)
            busy.write_wav(args.out_dir / fname, np.clip(w, -32768, 32767))
        print(f"  {name}: {N_FILES} files", file=sys.stderr)

    path = args.out_dir / "truth.csv"
    kept = []
    if path.exists():
        with path.open(newline="") as fh:
            kept = [r for r in csv.reader(fh)][1:]
        sets_done = {n for n in SETS if not args.only or n in args.only}
        kept = [r for r in kept if not any(r[0].startswith(f"ft8_dense_{n}_") for n in sets_done)]
    with path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["file", "msg", "f0", "dt", "snr"])
        w.writerows(kept)
        w.writerows(rows)
    write_stamp(args.out_dir, ft8sim, args.seed)
    print(f"wrote {len(list(args.out_dir.glob('*.wav')))} wavs, {len(kept) + len(rows)} truth rows -> {args.out_dir}", file=sys.stderr)


if __name__ == "__main__":
    main()
