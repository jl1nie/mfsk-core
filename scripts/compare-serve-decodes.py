#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Compare what two builds of `mfsk-serve` decode, over every recording under embedded-poc/assets.

    cargo build -p mfsk-serve --release          # at the old commit; copy target/release/mfsk-serve to old
    cargo build -p mfsk-serve --release          # at the new commit
    python3 scripts/compare-serve-decodes.py OLD_BINARY NEW_BINARY

For `fast`, `normal` and `deep`, every 12 kHz mono `.wav` is decoded as FT8 by both; the script prints
(1) whether the set of decoded messages is the same, how many rows moved in `freq_hz`, `dt_sec` and
`snr_db` and by how much, and the rows that appeared or went; and (2) for the 22 recordings
`tests/fixtures/ft8_lib_jt9_beta6.tsv` holds `jt9`'s decodes of, the SNR error against `jt9` (mean, SD,
mean |d|, max |d|) for each build, at `deep`. It is how the numbers in the #762 entry of
`mfsk-core/tests/decode_snapshot.rs` and the CHANGELOG were produced. A message or a frequency that moves
is a result; an SNR that moves in the third decimal is not, which is why (2) is there.
"""
import glob
import json
import os
import statistics
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(ROOT, "embedded-poc/assets")
JT9 = os.path.join(ROOT, "mfsk-core/tests/fixtures/ft8_lib_jt9_beta6.tsv")


def decode(binary, path, depth):
    """{text: row} for one recording, or None when the file is not decodable as 12 kHz mono."""
    out = subprocess.run([binary, "decode", path, "--mode", "FT8", "--depth", depth],
                         capture_output=True, text=True, env={**os.environ, "RAYON_NUM_THREADS": "4"})
    rows = {}
    for line in out.stdout.splitlines():
        row = json.loads(line)
        if row["type"] == "error":
            return None
        if row["type"] == "row":
            rows[row["text"]] = row
    return rows


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    old, new = sys.argv[1:]
    files = sorted(glob.glob(os.path.join(ASSETS, "*.wav")))
    runs = same = rows = 0
    lost, gained, moved = [], [], {"freq_hz": 0, "dt_sec": 0}
    worst = {"freq_hz": 0.0, "dt_sec": 0.0, "snr_db": 0.0}
    snr_moves = []
    for depth in ("fast", "normal", "deep"):
        for path in files:
            a, b = decode(old, path, depth), decode(new, path, depth)
            if a is None or b is None:
                continue
            runs += 1
            rows += len(a)
            gone, came = set(a) - set(b), set(b) - set(a)
            same += not gone and not came
            lost += [(depth, os.path.basename(path), t) for t in sorted(gone)]
            gained += [(depth, os.path.basename(path), t) for t in sorted(came)]
            for text in set(a) & set(b):
                for col, tol in (("freq_hz", 1e-3), ("dt_sec", 1e-4)):
                    d = abs(a[text][col] - b[text][col])
                    worst[col] = max(worst[col], d)
                    moved[col] += d > tol
                d = abs(a[text]["snr_db"] - b[text]["snr_db"])
                worst["snr_db"] = max(worst["snr_db"], d)
                snr_moves.append((d, depth, os.path.basename(path), text, a[text]["snr_db"], b[text]["snr_db"]))
    print(f"{runs} decodes, {rows} rows: message sets equal in {same}; lost {len(lost)}, gained {len(gained)}")
    for item in lost:
        print("  lost  ", item)
    for item in gained:
        print("  gained", item)
    print(f"freq_hz moved on {moved['freq_hz']} rows (max {worst['freq_hz']:.3g}), "
          f"dt_sec on {moved['dt_sec']} (max {worst['dt_sec']:.3g})")
    n = len(snr_moves)
    print("snr_db moved: " + ", ".join(f"> {t} dB on {sum(1 for m in snr_moves if m[0] > t)}"
                                       for t in (0.0, 0.01, 0.05, 0.1, 0.2)) + f" of {n} rows; max {worst['snr_db']:.3f}")
    for m in sorted(snr_moves, reverse=True)[:5]:
        print(f"  {m[0]:.3f} dB  {m[1]:6} {m[2]:22} {m[3]!r}  {m[4]:.2f} -> {m[5]:.2f}")

    # against jt9, on the recordings the table covers
    reference = {}
    for line in open(JT9):
        if line.startswith("#") or not line.strip():
            continue
        f = line.rstrip("\n").split("\t")
        reference.setdefault(f[0], {})[f[4].strip()] = float(f[3])
    print("SNR error against jt9, deep:")
    for name, binary in (("old", old), ("new", new)):
        errors = []
        for recording, wanted in reference.items():
            path = os.path.join(ASSETS, recording + ".wav")
            got = decode(binary, path, "deep") if os.path.exists(path) else None
            for text, snr in (wanted if got else {}).items():
                if text in got:
                    errors.append(got[text]["snr_db"] - snr)
        print(f"  {name}: {len(errors)} rows, mean {statistics.mean(errors):+.3f} dB, SD {statistics.pstdev(errors):.3f}, "
              f"mean|d| {statistics.mean(abs(e) for e in errors):.3f}, max|d| {max(abs(e) for e in errors):.2f}")


if __name__ == "__main__":
    main()
