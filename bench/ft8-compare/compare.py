#!/usr/bin/env python3 -I
"""Pair the decodes of mfsk-core, rust-ft8 and jt9 file by file.

usage: compare.py <out.csv from ft8-compare> <dir with jt9_<name>.out and jt9_wall.txt> <wav dir with <name>.txt>

Two references, reported side by side:
  jt9   the decodes of `jt9 -8 -d 3 -L 200 -H 3000` (WSJT-X v3.2.0-rc1) on the same file
  txt   the per-file text fixtures shipped with ft8_lib (rust-ft8's own ground truth)
A decode "matches" a reference entry when the message text is equal (whitespace-insensitive,
case-insensitive) and the frequency is within FTOL Hz. Each reference entry is used once.
"""
import csv, re, sys, os, collections

FTOL = 10.0
JT9 = re.compile(r"^\d{6}\s+(-?\d+)\s+(-?[\d.]+)\s+(\d+)\s+~\s+(.*?)\s*$")
TXT = re.compile(r"^(?:\d{6})?\s*(-?\d+)\s+(-?[\d.]+)\s+(\d+)\s+~\s+(.*?)\s*$")
TAIL = re.compile(r"(\s+\?)?(\s+a\d+)?\s*$")


def norm(m):
    return re.sub(r"\s+", " ", m.strip()).upper()


def read_ref(path, rx):
    out = []
    for line in open(path, errors="replace"):
        m = rx.match(line)
        if m:
            out.append((norm(TAIL.sub("", m[4])), float(m[3]), float(m[2]), int(m[1])))
    return out


def pair(dec, ref):
    """dec, ref: lists of (msg, freq, ...). Returns (matched dec idx set, matched ref idx set)."""
    used, md = set(), set()
    for i, d in enumerate(dec):
        for j, r in enumerate(ref):
            if j not in used and d[0] == r[0] and abs(d[1] - r[1]) <= FTOL:
                used.add(j); md.add(i); break
    return md, used


def main():
    csv_path, jdir, wdir = sys.argv[1:4]
    by = collections.defaultdict(lambda: collections.defaultdict(list))
    ms = collections.defaultdict(dict)
    for r in csv.DictReader(open(csv_path)):
        ms[r["decoder"]][r["file"]] = float(r["ms"])
        if r["message"]:
            by[r["decoder"]][r["file"]].append((norm(r["message"]), float(r["freq_hz"]), float(r["dt_s"]), int(float(r["snr_db"]))))
    files = sorted({f for d in by.values() for f in d} | set(ms["mfsk"]))
    wall = {l.split()[0]: float(l.split()[1]) for l in open(os.path.join(jdir, "jt9_wall.txt"))}
    decs = ["jt9", "mfsk"] + (["mfsk-sic3", "mfsk-single"] if "mfsk-sic3" in by else []) + ["rf8-default", "rf8-deep"]
    tot = {d: collections.Counter() for d in decs}
    rows = []
    for f in files:
        n = f[:-4]
        jt9 = read_ref(os.path.join(jdir, f"jt9_{n}.out"), JT9)
        txt = read_ref(os.path.join(wdir, f"{n}.txt"), TXT)
        by["jt9"][f] = jt9
        row = [n, len(jt9), len(txt)]
        for d in decs:
            dec = by[d][f]
            for refname, ref in (("jt9", jt9), ("txt", txt)):
                md, mr = pair(dec, ref)
                tot[d][refname + "_match"] += len(mr)
                tot[d][refname + "_extra"] += len(dec) - len(md)   # decodes not in the reference
                tot[d][refname + "_ref"] += len(ref)
            tot[d]["n"] += len(dec)
            row += [len(dec), pair(dec, jt9)[1].__len__(), len(dec) - len(pair(dec, jt9)[0])]
        rows.append(row)
    print("per file: name, jt9 n, txt n | for each of jt9/mfsk/rf8-default/rf8-deep: n, matched-vs-jt9, not-in-jt9")
    for r in rows:
        print(*r)
    print()
    print(f"{'decoder':12} {'n':>5} | vs jt9: {'recall':>14} {'not in jt9':>10} | vs txt: {'recall':>14} {'not in txt':>10} | {'ms/file':>8}")
    for d in decs:
        t = tot[d]
        tm = (sum(wall.values()) / len(wall)) if d == "jt9" else sum(ms[d].values()) / len(ms[d])
        print(f"{d:12} {t['n']:>5} |        {t['jt9_match']:>4}/{t['jt9_ref']:<4}={100*t['jt9_match']/max(t['jt9_ref'],1):5.1f}% {t['jt9_extra']:>10} |        {t['txt_match']:>4}/{t['txt_ref']:<4}={100*t['txt_match']/max(t['txt_ref'],1):5.1f}% {t['txt_extra']:>10} | {tm:>8.0f}")
    # decodes the other side lacks, for inspection
    out = os.path.join(jdir, "diff.txt")
    with open(out, "w") as fh:
        for f in files:
            n = f[:-4]
            jt9 = by["jt9"][f]
            for d in decs[1:]:
                md, mr = pair(by[d][f], jt9)
                for i, x in enumerate(by[d][f]):
                    if i not in md:
                        fh.write(f"{n}\t{d}\tNOT-IN-JT9\t{x[1]:.0f}\tdt{x[2]:+.1f}\t{x[3]}\t{x[0]}\n")
                for j, x in enumerate(jt9):
                    if j not in mr:
                        fh.write(f"{n}\t{d}\tJT9-ONLY\t{x[1]:.0f}\tdt{x[2]:+.1f}\t{x[3]}\t{x[0]}\n")
    print("\ndetail:", out)


main()
