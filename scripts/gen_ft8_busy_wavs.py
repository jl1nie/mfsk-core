#!/usr/bin/env python3
"""Generate the FT8 "busy band" corpus: several signals per file, scattered in
time and frequency, plus files with noise only.

Why it exists. The AWGN/CCIR sweep corpora hold ONE signal at DT 0 and never a
file without one, so they cannot show what crowding, a time offset or an empty
band do to a decoder: a candidate-list cap, the lag window of the coarse sync,
an acceptance gate that admits garbage. `ft8sim` writes one signal per file, so
this script asks it for each signal *without noise* (SNR 99 makes it skip the
noise and peak-normalise), rescales each to the SNR it should have, adds them up
and adds one realisation of white Gaussian noise. Nothing here re-implements FT8:
every waveform comes from ft8sim.

    scripts/gen_ft8_busy_wavs.py <ft8sim> <out-dir> [--seed 1] [--only SET ...]

Sets (files, signals per file, what each measures):
    dt1      100 x  1   one signal at -16 dB, DT scattered over -0.5..+1.5 s
                        (no crowding, no sensitivity limit: a miss is a time-window
                        problem)
    busy10    40 x 10   signals scattered over 200-2700 Hz, DT -0.5..+1.5 s,
    busy20    40 x 20   SNR -24..-6 dB, all different messages
    busy40    40 x 40
    noise    200 x  0   noise only: every decode is unexpected

Output: <out-dir>/ft8_busy_<set>_<NN>.wav and <out-dir>/truth.csv with one row
per transmitted signal: file,msg,f0,dt,snr. Noise-only files have no rows.

Signals are NOT faded: fading is what the ft8_itu corpus is for. DT is limited
to -0.5..+1.5 s because ft8sim shifts the waveform circularly inside its 15 s
buffer, so a start earlier than 0 s or an end later than 15 s would wrap round.

SNR is ft8sim's: SNR in 2500 Hz, signal power over the whole transmission,
noise standard deviation 1 per sample before the x100 gain. The calibration is
checked, not assumed: a single signal made here must cross 50 % where the same
signal made by ft8sim itself does (see docs/notes/BENCHMARKS.md).

Deterministic: `--seed` (default 1) seeds every random choice and the noise.
Needs numpy and an ft8sim; SNR 99 works on any ft8sim, ITU codes are not used.
"""
import argparse
import csv
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

import numpy as np

FS = 12000
NMAX = 15 * FS
GAIN = 100.0
BANDWIDTH_RATIO = 2500.0 / (FS / 2.0)          # ft8sim: 2500 / (fs/2)
SIG_SCALE = np.sqrt(2.0 * BANDWIDTH_RATIO)     # amplitude of a 0 dB signal, ft8sim's `sig`

# name -> (files, signals per file)
SETS = {
    "dt1": (100, 1),
    "busy10": (40, 10),
    "busy20": (40, 20),
    "busy40": (40, 40),
    "noise": (200, 0),
}
DT_RANGE = (-0.5, 1.5)
F_RANGE = (200.0, 2700.0)
SNR_RANGE = (-24.0, -6.0)
DT1_SNR = -16.0
MIN_SPACING_HZ = 12.0          # closer than this and neither signal is decodable

LETTERS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
DIGITS = "0123456789"
PREFIXES = ["K", "W", "N", "AA", "KB", "JA", "JH", "JR", "DL", "DK", "G", "F", "I", "EA", "PA", "OH", "SM", "VK", "ZL", "UA", "PY", "LU", "BG", "HL"]


def callsign(rng):
    pre = PREFIXES[rng.integers(len(PREFIXES))]
    suf = "".join(LETTERS[i] for i in rng.integers(26, size=rng.integers(1, 4)))
    return f"{pre}{DIGITS[rng.integers(10)]}{suf}"


def grid(rng):
    return LETTERS[rng.integers(18)] + LETTERS[rng.integers(18)] + DIGITS[rng.integers(10)] + DIGITS[rng.integers(10)]


def message(rng):
    """A plausible standard message. ft8sim reports what it actually packed and
    that text, not this one, is the truth."""
    kind = rng.integers(6)
    a, b = callsign(rng), callsign(rng)
    if kind == 0:
        return f"CQ {a} {grid(rng)}"
    if kind == 1:
        return f"{a} {b} {grid(rng)}"
    if kind == 2:
        return f"{a} {b} {rng.integers(-24, 10):+03d}"
    if kind == 3:
        return f"{a} {b} R{rng.integers(-24, 10):+03d}"
    if kind == 4:
        return f"{a} {b} RR73"
    return f"{a} {b} 73"


def read_wav(path):
    with wave.open(str(path), "rb") as w:
        assert w.getframerate() == FS and w.getnchannels() == 1 and w.getsampwidth() == 2
        return np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)


def write_wav(path, samples):
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(FS)
        w.writeframes(samples.astype("<i2").tobytes())


def unit_signal(ft8sim, msg, f0, dt):
    """(truth message, waveform with the amplitude ft8sim's imag(c0) has: rms 1/sqrt(2)).

    ft8sim at SNR 99 writes no noise and rescales the peak to 32766.9, so the
    absolute scale is lost; a constant-envelope GFSK signal has a known rms
    (1/sqrt(2) for imag of a unit phasor), which restores it. The ramps at
    either end are left out of the rms."""
    with tempfile.TemporaryDirectory() as tmp:
        out = subprocess.run(
            [str(ft8sim), msg, f"{f0:.3f}", f"{dt:.3f}", "0.0", "0.0", "1", "99"],
            cwd=tmp, capture_output=True, text=True, check=True,
        ).stdout
        x = read_wav(Path(tmp) / "000000_000001.wav")
    sent = None
    for line in out.splitlines():
        if line.startswith("Decoded message:"):
            sent = line[len("Decoded message:"):].split("i3.n3")[0].strip()
    if not sent:
        raise RuntimeError(f"ft8sim gave no 'Decoded message' for {msg!r}:\n{out}")
    nz = np.nonzero(np.abs(x) > 0.01 * np.abs(x).max())[0]
    core = x[nz[0] + 1500 : nz[-1] - 1500]
    rms = np.sqrt(np.mean(core**2))
    return sent, x * (np.sqrt(0.5) / rms)


def draw_frequencies(rng, n):
    """n frequencies in F_RANGE, at least MIN_SPACING_HZ apart."""
    fs = []
    while len(fs) < n:
        f = float(np.round(rng.uniform(*F_RANGE), 1))
        if all(abs(f - g) >= MIN_SPACING_HZ for g in fs):
            fs.append(f)
    return fs


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ft8sim", type=Path)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--only", nargs="*", choices=sorted(SETS), help="generate just these sets")
    args = ap.parse_args()

    ft8sim = args.ft8sim.resolve()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    truth_rows = []
    for si, (name, (n_files, n_sig)) in enumerate(SETS.items()):
        if args.only and name not in args.only:
            continue
        for k in range(1, n_files + 1):
            # one generator per file: adding files or sets never changes an existing one
            rng = np.random.default_rng([args.seed, si, k])
            fname = f"ft8_busy_{name}_{k:02d}.wav"
            total = np.zeros(NMAX)
            freqs = draw_frequencies(rng, n_sig)
            for f0 in freqs:
                dt = round(float(rng.uniform(*DT_RANGE)), 2)
                snr = DT1_SNR if name == "dt1" else round(float(rng.uniform(*SNR_RANGE)), 1)
                sent, u = unit_signal(ft8sim, message(rng), f0, dt)
                total += SIG_SCALE * 10.0 ** (0.05 * snr) * u
                truth_rows.append((fname, sent, f0, dt, snr))
            total += rng.standard_normal(NMAX)          # unit variance, as ft8sim's gran()
            wave_i16 = np.rint(GAIN * total)
            if np.abs(wave_i16).max() > 32767:
                print(f"warning: {fname} clips", file=sys.stderr)
            write_wav(args.out_dir / fname, np.clip(wave_i16, -32768, 32767))
        print(f"  {name}: {n_files} files x {n_sig} signals", file=sys.stderr)

    # truth.csv covers every set present, so a partial run (--only) merges into the
    # existing file rather than replacing it
    path = args.out_dir / "truth.csv"
    existing = []
    if path.exists():
        with path.open(newline="") as fh:
            existing = [r for r in csv.reader(fh)][1:]
    regenerated = {r[0] for r in truth_rows}
    sets_done = {n for n in SETS if not args.only or n in args.only}
    kept = [r for r in existing if not any(r[0].startswith(f"ft8_busy_{n}_") for n in sets_done)]
    with path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["file", "msg", "f0", "dt", "snr"])
        w.writerows(kept)
        w.writerows(truth_rows)
    print(f"wrote {len(list(args.out_dir.glob('*.wav')))} wavs, {len(kept) + len(truth_rows)} truth rows -> {args.out_dir}", file=sys.stderr)


if __name__ == "__main__":
    main()
