#!/usr/bin/env python3
"""Build the slot directory `icap-cands` reads: each slot a .wav (symlink) with its .ref (one reference message per line).
Usage: scripts/cands-corpus.py OUT_DIR [FT8_LIB_CHECKOUT]
  a_*      the 22 vendored recordings, references = jt9 beta6 (mfsk-core/tests/fixtures/ft8_lib_jt9_beta6.tsv)
  b_NN     ft8_lib's test/wav/20m_busy (kgoba/ft8_lib 9fec6ca3, MIT; needs the checkout), references = its .txt lists
  busy_*, dense_*   the generated tier-C sets with truth.csv (embedded-poc/assets, gitignored; scripts/gen_ft8_*_wavs.*)
  sweep_*, itu_*    the generated single-signal SNR ladders, reference "CQ JL1NIE PM95"
Then:  icap-cands OUT_DIR/*.wav > probe.tsv ; scripts/cands-yield.py probe.tsv --by-corpus   (design note section 27)"""
import collections, csv, os, re, shutil, sys
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
A = f"{ROOT}/embedded-poc/assets"
out = sys.argv[1]; os.makedirs(out, exist_ok=True)
n = collections.Counter()
def put(src, name, msgs, link=True):
    dst = f"{out}/{name}.wav"
    if not os.path.exists(dst):
        (os.symlink if link else shutil.copy)(os.path.abspath(src), dst)
    open(f"{out}/{name}.ref", "w").write("\n".join(sorted(msgs)) + "\n"); n[name.split("_")[0]] += 1
ref = collections.defaultdict(set)
for l in open(f"{ROOT}/mfsk-core/tests/fixtures/ft8_lib_jt9_beta6.tsv"):
    if l.startswith("#") or not l.strip(): continue
    p = l.rstrip("\n").split("\t"); ref[p[0]].add(" ".join(p[4].split()))
for k, v in ref.items():
    if os.path.exists(f"{A}/{k}.wav"): put(f"{A}/{k}.wav", f"a_{k}", v)
if len(sys.argv) > 2:
    for i in range(1, 39):
        b = f"{sys.argv[2]}/test/wav/20m_busy/test_{i:02d}"
        msgs = {" ".join(m.group(1).split()) for l in open(b + ".txt") if (m := re.match(r"\s*\d+\s+-?\d+\s+[-\d.]+\s+\d+\s+~\s+(.*)", l))}
        put(b + ".wav", f"b_{i:02d}", msgs)
for d in ("ft8_busy_sweep", "ft8_dense_sweep"):
    if not os.path.isdir(f"{A}/{d}"): continue
    truth = collections.defaultdict(set)
    for r in csv.DictReader(open(f"{A}/{d}/truth.csv")): truth[r["file"]].add(" ".join(r["msg"].split()))
    for f in sorted(os.listdir(f"{A}/{d}")):
        if f.endswith(".wav"): put(f"{A}/{d}/{f}", f"{d.split('_')[1]}_{f[:-4]}", truth.get(f, set()))
for d in ("ft8_sweep", "ft8_itu_sweep"):
    if not os.path.isdir(f"{A}/{d}"): continue
    for f in sorted(os.listdir(f"{A}/{d}")):
        if f.endswith(".wav"): put(f"{A}/{d}/{f}", f"{d.split('_')[1]}_{f[:-4]}", {"CQ JL1NIE PM95"})
print(dict(n))
