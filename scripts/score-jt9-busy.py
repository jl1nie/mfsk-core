"""Score jt9 output on the busy corpus against truth.csv.
usage: score_busy.py <corpus-dir> <results-root> <label> <mode e.g. -8> <depth>
A truth signal is HIT when jt9 printed its message (markers ' ?' / ' aN' stripped) within 5 Hz and 0.5 s;
an EXTRA is a decoded message that is not any truth message of that file (by text)."""
import csv, collections, glob, os, re, sys
corpus, root, label, mode, depth = sys.argv[1:6]
truth = collections.defaultdict(list)
for r in csv.DictReader(open(f"{corpus}/truth.csv")):
    truth[r["file"].removesuffix(".wav")].append((r["msg"], float(r["f0"]), float(r["dt"])))
line = re.compile(r"\s*\d+\s+-?\d+\s+(-?[\d.]+)\s+(\d+)\s+[~+]\s+(.*?)\s*$")
tail = re.compile(r"(\s+\?)?(\s+a\d+)?\s*$")
agg = collections.defaultdict(lambda: [0, 0, 0, 0])   # set -> [files, truth, hits, extra]
for path in sorted(glob.glob(f"{root}/{label}/{mode}_d{depth}/ft8_busy_*.txt")):
    stem = os.path.basename(path)[:-4]
    sname = stem.split("_")[2]
    t = truth.get(stem, [])
    dec = []
    for l in open(path):
        m = line.match(l)
        if m and "DecodeFinished" not in l:
            dec.append((tail.sub("", m.group(3)), float(m.group(2)), float(m.group(1))))
    hits = sum(any(dm == tm and abs(df - tf) <= 5 and abs(dd - td) <= 0.5 for dm, df, dd in dec) for tm, tf, td in t)
    tmsgs = {tm for tm, _, _ in t}
    extra = len({dm for dm, _, _ in dec if dm not in tmsgs})
    a = agg[sname]
    a[0] += 1; a[1] += len(t); a[2] += hits; a[3] += extra
print(f"{label} {mode} -d{depth}")
for s, (n, tt, h, e) in sorted(agg.items()):
    rec = f"{100*h/tt:5.1f}%" if tt else "   n/a"
    print(f"  {s:8s} files={n:3d} truth={tt:4d} hits={h:4d} recall={rec}  extra={e}")
