#!/usr/bin/env python3
"""Score real `jt9` output over a tier-C sweep corpus and print 50%-recall crossings.

`docs/notes/BENCHMARKS.md`, "Reading a crossing: pair it against `jt9`", says to
run WSJT-X's own binary over the same corpus and score it with the sweep's own
criteria. This is that scorer, so the comparison does not depend on someone's
one-off script.

A file is a hit when jt9 printed the injected message (CQ JL1NIE PM95) within
5 Hz of 1500 Hz and |dt| <= 0.6 s -- the criteria `tests/ft8_sweep.rs` and
`tests/ft4_sweep.rs` use. jt9 appends " ?" (low confidence) and " aN" (AP type)
to the message field; both are stripped before comparing, or a correct AP
decode is scored as a miss. The crossing itself comes from `crossing_snr()` in
`sweep-regression-check.py`, so the interpolation is identical to the one that
produced `sweep-baseline.json`.

Layout it reads (one directory per jt9 build and depth):

    <root>/<label>/<mode>_d<depth>/<wav-stem>.txt        mode: -8 (FT8) or -5 (FT4)

which is what this loop writes:

    for d in 1 2 3; do
      out=<root>/<label>/-8_d$d; mkdir -p "$out"
      for f in embedded-poc/assets/ft8_sweep/ft8_*.wav; do
        ( cd "$(mktemp -d)" && <jt9> -8 -d $d -a . -t . "$f" > "$out/$(basename "$f" .wav).txt" )
      done
    done

Usage:
    scripts/score-jt9-sweep.py ft8 <root> <label> [<label2>]
    scripts/score-jt9-sweep.py ft4 <root> old new

With two labels it also prints the second minus the first (negative = more
sensitive). Only the sweep's own corpus is meaningful: the numbers are relative
to the files they were generated from (see "Reading a crossing").
"""
import collections
import glob
import importlib.util
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("sweep_regression_check", os.path.join(HERE, "sweep-regression-check.py"))
_chk = importlib.util.module_from_spec(_spec)
_saved_argv, sys.argv = sys.argv, ["sweep-regression-check.py"]
_spec.loader.exec_module(_chk)
sys.argv = _saved_argv

MSG, F0, FREQ_TOL_HZ, DT_TOL_S = "CQ JL1NIE PM95", 1500.0, 5.0, 0.6
MODES = {"ft8": "-8", "ft4": "-5"}
CHANNELS = ["awgn", "ccir_good", "ccir_moderate", "ccir_poor"]

_LINE = re.compile(r"\s*\d+\s+-?\d+\s+(-?[\d.]+)\s+(\d+)\s+[~+]\s+(.*?)\s*$")
_TAIL = re.compile(r"(\s+\?)?(\s+a\d+)?\s*$")


def is_hit(path):
    with open(path) as fh:
        for line in fh:
            m = _LINE.match(line)
            if not m or "DecodeFinished" in line:
                continue
            dt, freq, text = float(m.group(1)), float(m.group(2)), _TAIL.sub("", m.group(3))
            if text == MSG and abs(freq - F0) <= FREQ_TOL_HZ and abs(dt) <= DT_TOL_S:
                return True
    return False


def crossing(root, label, proto, channel, depth):
    cells = collections.defaultdict(lambda: [0, 0])
    pattern = f"{root}/{label}/{MODES[proto]}_d{depth}/{proto}_{channel}_m*_*.txt"
    for path in glob.glob(pattern):
        snr = -int(re.search(r"_m(\d+)_", path).group(1))
        cells[snr][1] += 1
        cells[snr][0] += is_hit(path)
    value, note = _chk.crossing_snr({k: tuple(v) for k, v in cells.items()})
    return value, note, sum(c[1] for c in cells.values())


def main():
    if len(sys.argv) not in (4, 5) or sys.argv[1] not in MODES:
        sys.exit(__doc__)
    proto, root, labels = sys.argv[1], sys.argv[2], sys.argv[3:]
    print(f"{proto.upper()} (jt9 {MODES[proto]}), 50%-recall crossing, dB")
    print(f"{'channel':<15}" + "".join(f"{'-d' + str(d):>{14 * len(labels) + (9 if len(labels) == 2 else 0)}}" for d in (1, 2, 3)))
    for channel in CHANNELS:
        row = f"{channel:<15}"
        for depth in (1, 2, 3):
            vals = []
            for label in labels:
                v, _note, n = crossing(root, label, proto, channel, depth)
                vals.append(v)
                if n == 0:
                    sys.exit(f"no files for {label} {proto} {channel} -d{depth} under {root}")
            cell = "  ".join("n/a" if v is None else f"{v:+.2f}" for v in vals)
            if len(vals) == 2 and None not in vals:
                cell += f"  ({vals[1] - vals[0]:+.2f})"
            row += f"{cell:>{14 * len(labels) + (9 if len(labels) == 2 else 0)}}"
        print(row)


if __name__ == "__main__":
    main()
