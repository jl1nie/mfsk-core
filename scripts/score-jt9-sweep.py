#!/usr/bin/env python3
"""Score real `jt9` output over a tier-C sweep corpus and print 50%-recall crossings.

`docs/notes/BENCHMARKS.md`, "Reading a crossing: pair it against `jt9`", says to
run WSJT-X's own binary over the same corpus and score it with the sweep's own
criteria. This is that scorer, so the comparison does not depend on someone's
one-off script.

A file is a hit when jt9 printed the injected message (CQ JL1NIE PM95) within
5 Hz of 1500 Hz and |dt| <= 0.6 s -- the criteria `tests/ft8_sweep.rs`,
`tests/ft4_sweep.rs` and `tests/fst4_sweep.rs` use. jt9 appends " ?" (low confidence) and " aN" (AP type)
to the message field; both are stripped before comparing, or a correct AP
decode is scored as a miss. The crossing itself comes from `crossing_snr()` in
`sweep-regression-check.py`, so the interpolation is identical to the one that
produced `sweep-baseline.json`.

Layout it reads (one directory per jt9 build and depth):

    <root>/<label>/<mode>_d<depth>/<wav-stem>.txt        mode: -8 (FT8), -5 (FT4) or -7 (FST4)

which is what this loop writes:

    for d in 1 2 3; do
      out=<root>/<label>/-8_d$d; mkdir -p "$out"
      for f in embedded-poc/assets/ft8_sweep/ft8_*.wav; do
        ( cd "$(mktemp -d)" && <jt9> -8 -d $d -a . -t . "$f" > "$out/$(basename "$f" .wav).txt" )
      done
    done

FST4 needs the period as well (`-p <seconds>`, the <nsec> in the file name), and
prints one row per sub-mode and channel, keyed like `sweep-baseline.json`
(`fst4/60/ccir_poor`):

    for f in embedded-poc/assets/fst4_sweep/fst4_*.wav; do
      n=$(basename "$f" | cut -d_ -f2)
      ( cd "$(mktemp -d)" && <jt9> -7 -p $n -d 3 -f 1500 -F 100 -a . -t . "$f" \
          > "<root>/<label>/-7_d3/$(basename "$f" .wav).txt" )
    done

Usage:
    scripts/score-jt9-sweep.py ft8 <root> <label> [<label2>]
    scripts/score-jt9-sweep.py ft4 <root> old new
    scripts/score-jt9-sweep.py fst4 <root> <label> [<label2>]
    scripts/score-jt9-sweep.py csv <ft8|ft4|fst4> <root> <label> <depth> <out.csv>

`csv` writes one row per file in the per-trial format the sweep tests dump
(`mode,channel,snr_db,trial,pass,extra`; mode is `-` for FT8/FT4), so jt9 can be
paired file by file against a sweep run of this crate. Paired counts resolve
a small shift that two 20-trial crossings cannot.

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
MODES = {"ft8": "-8", "ft4": "-5", "fst4": "-7"}
CHANNELS = ["awgn", "ccir_good", "ccir_moderate", "ccir_poor"]

_LINE = re.compile(r"\s*\d+\s+-?\d+\s+(-?[\d.]+)\s+(\d+)\s+[~+`]\s+(.*?)\s*$")
_TAIL = re.compile(r"(\s+\?)?(\s+a\d+)?\s*$")


def decodes(path):
    """[(dt, freq, message)] jt9 printed, with its ' ?' / ' aN' markers stripped."""
    out = []
    with open(path) as fh:
        for line in fh:
            m = _LINE.match(line)
            if m and "DecodeFinished" not in line:
                out.append((float(m.group(1)), float(m.group(2)), _TAIL.sub("", m.group(3))))
    return out


def is_hit(path):
    return any(t == MSG and abs(f - F0) <= FREQ_TOL_HZ and abs(dt) <= DT_TOL_S for dt, f, t in decodes(path))


def crossing(root, label, proto, channel, depth, nsec=None):
    cells = collections.defaultdict(lambda: [0, 0])
    stem = f"{proto}_{nsec}_{channel}" if nsec else f"{proto}_{channel}"
    pattern = f"{root}/{label}/{MODES[proto]}_d{depth}/{stem}_m*_*.txt"
    for path in glob.glob(pattern):
        snr = -int(re.search(r"_m(\d+)_", path).group(1))
        cells[snr][1] += 1
        cells[snr][0] += is_hit(path)
    if not cells:
        return None, "no files", 0
    value, note = _chk.crossing_snr({k: tuple(v) for k, v in cells.items()})
    return value, note, sum(c[1] for c in cells.values())


def dump_csv(proto, root, label, depth, out):
    """Per-trial rows in the sweep tests' CSV format; `extra` = distinct other messages."""
    name = re.compile(rf"{proto}_(?:(\d+)_)?(awgn|ccir_\w+?)_([mp]\d+)_(\d+)\.txt$")
    with open(out, "w") as fh:
        fh.write("mode,channel,snr_db,trial,pass,extra\n")
        for path in sorted(glob.glob(f"{root}/{label}/{MODES[proto]}_d{depth}/{proto}_*.txt")):
            m = name.search(path)
            if not m:
                continue
            nsec, channel, tag, trial = m.groups()
            snr = -int(tag[1:]) if tag[0] == "m" else int(tag[1:])
            others = {t for _dt, _f, t in decodes(path) if t != MSG}
            fh.write(f"{nsec or '-'},{channel},{snr},{int(trial)},{int(is_hit(path))},{len(others)}\n")


def sub_modes(root, label, depth):
    """FST4 periods present on disk, from `fst4_<nsec>_...` file names."""
    found = {re.match(r".*/fst4_(\d+)_", p).group(1) for p in glob.glob(f"{root}/{label}/-7_d{depth}/fst4_*.txt")}
    return sorted(found, key=int)


def main():
    if len(sys.argv) == 7 and sys.argv[1] == "csv" and sys.argv[2] in MODES:
        dump_csv(*sys.argv[2:6], sys.argv[6])
        return
    if len(sys.argv) not in (4, 5) or sys.argv[1] not in MODES:
        sys.exit(__doc__)
    proto, root, labels = sys.argv[1], sys.argv[2], sys.argv[3:]
    print(f"{proto.upper()} (jt9 {MODES[proto]}), 50%-recall crossing, dB")
    width = 14 * len(labels) + (9 if len(labels) == 2 else 0)
    print(f"{'channel':<20}" + "".join(f"{'-d' + str(d):>{width}}" for d in (1, 2, 3)))
    for nsec in (sub_modes(root, labels[0], 3) if proto == "fst4" else [None]):
        for channel in CHANNELS:
            row = f"{(nsec + '/' if nsec else '') + channel:<20}"
            for depth in (1, 2, 3):
                vals = []
                for label in labels:
                    v, _note, n = crossing(root, label, proto, channel, depth, nsec)
                    vals.append(v)
                    if n == 0 and depth == 3:
                        sys.exit(f"no files for {label} {proto} {nsec or ''} {channel} -d{depth} under {root}")
                cell = "  ".join("n/a" if v is None else f"{v:+.2f}" for v in vals)
                if len(vals) == 2 and None not in vals:
                    cell += f"  ({vals[1] - vals[0]:+.2f})"
                row += f"{cell:>{width}}"
            print(row)


if __name__ == "__main__":
    main()
