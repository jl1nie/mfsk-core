"""Report of the staircase variants over the slot set (design note section 27): true decodes and BP passes per corpus, and the slots decoded on the SNR ladders.
Input: var/<variant>.tsv, the `icap-cands` TSVs of one run per VARIANT (base nore nod a lowa dlow16 dlow21) built with scripts/probe/staircase-variants.patch
(a LOCAL probe, not committed code: `git apply` it, build icap-cands, run `VARIANT=<v> icap-cands <slots>`), D below pointing at that directory."""
import collections, re, sys
V = ["base", "nore", "nod", "a", "lowa", "dlow16", "dlow21"]
D = "/tmp/claude-1000/-home-minoru-src-mfsk-core/e27bf878-e039-4b3a-bc5f-f1e9bca3c1cc/scratchpad/var"

def load(v):
    slots = collections.defaultdict(lambda: dict(true=0, passes=0, calls=0, anytrue=0, decoded=0))
    for l in open(f"{D}/{v}.tsv"):
        r = l.rstrip("\n").split("\t")
        s = slots[r[0]]
        s["true"] += int(r[7]); s["decoded"] += int(r[6]); s["calls"] += int(r[9]); s["passes"] += int(r[10])
        s["anytrue"] = s["anytrue"] or int(r[7])
    return slots

data = {v: load(v) for v in V}
names = list(data["base"].keys())

def corp(n): return n.split("_")[0]

print("== totals per corpus: true decodes / BP passes  (variant vs base)")
cs = ["a", "b", "busy", "dense", "sweep", "itu"]
print(f"{'':8s}" + "".join(f"{c:>20s}" for c in cs) + f"{'ALL':>20s}")
for v in V:
    row = []
    tt = tp = 0
    for c in cs:
        t = sum(data[v][n]["true"] for n in names if corp(n) == c)
        p = sum(data[v][n]["passes"] for n in names if corp(n) == c)
        tt += t; tp += p
        row.append(f"{t:7d}/{p // 1000:6d}k")
    print(f"{v:8s}" + "".join(f"{x:>20s}" for x in row) + f"{tt:7d}/{tp // 1000:6d}k")
bp = sum(data["base"][n]["passes"] for n in names)
print("\npasses vs base: " + ", ".join(f"{v} {100 * sum(data[v][n]['passes'] for n in names) / bp:.1f}%" for v in V))

# SNR ladders: slot decoded (any true candidate) per channel / SNR
def group(name):
    m = re.match(r"(sweep|itu)_ft8_(awgn|ccir_good|ccir_moderate|ccir_poor|itu_\w+?)_(m|p)(\d+)_\d+$", name)
    if not m: return None
    return (m[1] + " " + m[2], (-1 if m[3] == "m" else 1) * int(m[4]))
res = collections.defaultdict(lambda: collections.defaultdict(int))
n_in = collections.Counter()
for n in names:
    g = group(n)
    if not g: continue
    n_in[g] += 1
    for v in V: res[g][v] += data[v][n]["anytrue"]
print("\n== SNR ladders: slots decoded (of 20) per channel and SNR; rows where a variant differs from base")
chans = sorted({g[0] for g in res})
tot = {v: 0 for v in V}
for ch in chans:
    snrs = sorted({g[1] for g in res if g[0] == ch})
    lines = []
    for sn in snrs:
        r = res[(ch, sn)]
        for v in V: tot[v] += r[v]
        if any(r[v] != r["base"] for v in V):
            lines.append(f"  {sn:+4d}  " + "  ".join(f"{r[v]:5d}" for v in V))
    if lines:
        print(f"\n{ch}\n  snr   " + "  ".join(f"{v:>5s}" for v in V))
        print("\n".join(lines))
print("\nslots decoded over all ladders: " + ", ".join(f"{v} {tot[v]}" for v in V))
