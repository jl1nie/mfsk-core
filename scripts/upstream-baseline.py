#!/usr/bin/env python3
"""The upstream baseline: this crate against WSJT-X on the same task and the same files.

`sweep-baseline.json` compares this crate with its own past. That cannot show a
decoder that was slower or weaker than WSJT-X from the start, which is how Q65
went 2.9-4.5x slower than `jt9` unnoticed until #552. This compares against
upstream, per trial, on the tier-C corpora that already exist. It adds no corpus
and nothing to CI.

A task is defined once, in `scripts/upstream_tasks.json`: the upstream command
line and the crate request that does the same job. The upstream half runs once
per upstream release or corpus, and its per-trial outcome is committed under
`docs/notes/upstream/`. The crate half is the sweep test run with the task's
`sweep_env`.

    scripts/upstream-baseline.py generate ft8/t1       # run upstream, write docs/notes/upstream/ft8_t1.csv
    scripts/upstream-baseline.py compare  ft8/t1 <crate.csv>
    scripts/upstream-baseline.py time     ft8/t1 [--per-cell 5]
    scripts/upstream-baseline.py run      ft8/t1 <csv-dir>   # crate sweep of the task, compare, time

`run` is what `scripts/run-sensitivity-sweeps.sh` calls for every task of a
protocol it sweeps. It costs one extra sweep of that corpus (about 30 s for FT8)
and the timing (about 40 s).

`compare` pairs every trial. A group is flagged `!!` when this crate misses
significantly more of the files upstream decodes than the other way round
(exact McNemar, p < 0.05). Pairing is what makes the existing 20-trial cells
enough: two 20-trial crossings cannot resolve 0.3 dB, but 260 paired trials
can. The crossing delta is printed as a summary. Unexpected decodes are flagged
when the crate's exceed upstream's by more than `--extra-tol`.

`time` runs both sides one file at a time, single-threaded, on a few files per
channel: the lowest-SNR cell, effectively noise, and the cell nearest
upstream's crossing. Upstream's time is `timer.out`'s total, which leaves out
process start, with FFTW wisdom warm. It prints the ratio.
"""
import argparse
import collections
import csv
import glob
import hashlib
import importlib.util
import json
import math
import multiprocessing
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def _load(name, file):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, file))
    mod = importlib.util.module_from_spec(spec)
    saved, sys.argv = sys.argv, [file]
    spec.loader.exec_module(mod)
    sys.argv = saved
    return mod


_score = _load("score_jt9_sweep", "score-jt9-sweep.py")  # jt9 output parsing and hit criteria
_chk = _load("sweep_regression_check", "sweep-regression-check.py")  # crossing_snr


def tasks():
    with open(os.path.join(HERE, "upstream_tasks.json")) as fh:
        return {k: v for k, v in json.load(fh).items() if not k.startswith("_")}


def out_path(task):
    return os.path.join(ROOT, "docs", "notes", "upstream", task.replace("/", "_") + ".csv")


def upstream_binary(t):
    return os.path.join(ROOT, "target", "upstream", f"build-{t['upstream_tag']}", t["upstream_bin"])


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def corpus_dir(t):
    return os.path.join(ROOT, "embedded-poc", "assets", t["corpus"])


def corpus_stamp(t):
    path = os.path.join(corpus_dir(t), ".corpus-stamp")
    if not os.path.exists(path):
        sys.exit(f"{path}: missing -- regenerate the corpus (scripts/lib/corpus-stamp.sh)")
    with open(path) as fh:
        return dict(l.rstrip("\n").split("=", 1) for l in fh if "=" in l and not l.startswith("#"))


def corpus_files(t):
    rx = re.compile(t["file_re"])
    out = []
    for p in sorted(glob.glob(os.path.join(corpus_dir(t), "*.wav"))):
        m = rx.match(os.path.basename(p))
        if m:
            tag = m["tag"]
            snr = -int(tag[1:]) if tag[0] == "m" else int(tag[1:])
            out.append((p, m["channel"], snr, int(m["trial"])))
    return out


# ── upstream runs ──────────────────────────────────────────────────────────

def _run_one(args):
    binary, uargs, wav = args
    with tempfile.TemporaryDirectory() as d:
        shutil.copy(wav, os.path.join(d, "000000_0000.wav"))
        r = subprocess.run([binary, *uargs, "000000_0000.wav"], cwd=d, capture_output=True, text=True)
        txt = os.path.join(d, "out.txt")
        with open(txt, "w") as fh:
            fh.write(r.stdout)
        hit = _score.is_hit(txt)
        others = {m for _dt, _f, m in _score.decodes(txt) if m != _score.MSG}
    return hit, len(others)


def cmd_generate(task, jobs):
    t = tasks()[task]
    binary = upstream_binary(t)
    if not os.path.exists(binary):
        sys.exit(f"{binary}: missing -- run scripts/build_jt9_upstream.sh")
    files = corpus_files(t)
    stamp = corpus_stamp(t)
    with multiprocessing.Pool(jobs) as pool:
        res = pool.map(_run_one, [(binary, t["upstream_args"], f[0]) for f in files], chunksize=4)
    path = out_path(task)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        fh.write(f"# task={task}\n")
        fh.write(f"# upstream={t['upstream_bin']} {t['upstream_tag']} sha256={sha256(binary)}\n")
        fh.write(f"# upstream_args={' '.join(t['upstream_args'])}\n")
        fh.write(f"# corpus={t['corpus']} seed={stamp.get('seed')} simulator_sha256={stamp.get('simulator_sha256')}"
                 f" stamp_commit={stamp.get('commit')}\n")
        fh.write(f"# written by scripts/upstream-baseline.py generate; machine={platform.processor() or platform.machine()}\n")
        fh.write("channel,snr_db,trial,pass,extra\n")
        for (p, ch, snr, trial), (hit, extra) in zip(files, res):
            fh.write(f"{ch},{snr},{trial},{int(hit)},{extra}\n")
    print(f"wrote {path}: {len(files)} trials, {sum(r[0] for r in res)} decoded")


# ── compare ────────────────────────────────────────────────────────────────

def read_rows(path):
    rows = {}
    with open(path) as fh:
        lines = [l for l in fh if not l.startswith("#")]
    for r in csv.DictReader(lines):
        key = (r.get("mode", "-") if r.get("mode") not in (None, "") else "-", r["channel"], int(r["snr_db"]), int(r["trial"]))
        rows[key[1:]] = (int(r["pass"]), int(r.get("extra") or 0))
    return rows


def header(path):
    out = {}
    with open(path) as fh:
        for l in fh:
            if not l.startswith("#"):
                break
            body = l[1:].strip()
            if body.startswith("upstream_args="):
                out["upstream_args"] = body.split("=", 1)[1]
                continue
            for part in body.split():
                if "=" in part:
                    k, v = part.split("=", 1)
                    out[k] = v
    return out


def mcnemar_p(worse, better):
    n = worse + better
    if n == 0:
        return 1.0
    k = min(worse, better)
    return min(1.0, 2 * sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n)


def crossing(rows, keys):
    cells = collections.defaultdict(lambda: [0, 0])
    for k in keys:
        cells[k[1]][0] += rows[k][0]
        cells[k[1]][1] += 1
    v, _note = _chk.crossing_snr({s: tuple(c) for s, c in cells.items()})
    return v


def cmd_compare(task, crate_csv, alpha, extra_tol):
    t = tasks()[task]
    up_path = out_path(task)
    if not os.path.exists(up_path):
        sys.exit(f"{up_path}: missing -- run `generate {task}` first")
    h = header(up_path)
    stamp = corpus_stamp(t)
    if h.get("seed") != stamp.get("seed") or h.get("simulator_sha256") != stamp.get("simulator_sha256"):
        sys.exit(f"{up_path} was generated on another corpus (seed {h.get('seed')}, simulator "
                 f"{h.get('simulator_sha256', '?')[:12]}) than the one here -- regenerate it")
    up, me = read_rows(up_path), read_rows(crate_csv)
    shared = sorted(set(up) & set(me))
    if not shared:
        sys.exit("no trials in common")
    print(f"{task}: {t['task']}")
    print(f"upstream: {h.get('upstream')} {h.get('upstream_args', '')}".rstrip())
    print(f"{'group':16} {'trials':>6} {'both':>5} {'up only':>8} {'crate only':>10} {'p':>6}"
          f" {'x up':>7} {'x crate':>8} {'delta':>6} {'extra up/crate':>15}")
    flagged = False
    for ch in sorted({k[0] for k in shared}):
        keys = [k for k in shared if k[0] == ch]
        both = sum(up[k][0] and me[k][0] for k in keys)
        up_only = sum(up[k][0] and not me[k][0] for k in keys)
        me_only = sum(me[k][0] and not up[k][0] for k in keys)
        p = mcnemar_p(up_only, me_only)
        xu, xm = crossing(up, keys), crossing(me, keys)
        eu, em = sum(up[k][1] for k in keys), sum(me[k][1] for k in keys)
        bad = (p < alpha and up_only > me_only) or em > eu + extra_tol
        flagged |= bad
        fmt = lambda v: f"{v:7.2f}" if v is not None else "      -"
        delta = f"{xm - xu:+6.2f}" if xu is not None and xm is not None else "     -"
        print(f"{ch:16} {len(keys):6} {both:5} {up_only:8} {me_only:10} {p:6.3f} {fmt(xu)} {fmt(xm):>8} {delta}"
              f" {eu:7}/{em:<7}{'  !!' if bad else ''}")
    print("\n!! = significantly more upstream-only decodes than crate-only (McNemar), or more unexpected"
          f" decodes than upstream + {extra_tol}." if flagged else "\nno group behind upstream.")
    return 1 if flagged else 0


# ── time ───────────────────────────────────────────────────────────────────

def cmd_time(task, per_cell):
    t = tasks()[task]
    up = read_rows(out_path(task))
    files = corpus_files(t)
    by_cell = collections.defaultdict(list)
    for f in files:
        by_cell[(f[1], f[2])].append(f)
    pick = {"low": [], "crossing": []}
    for ch in sorted({f[1] for f in files}):
        snrs = sorted(s for c, s in by_cell if c == ch)
        keys = [k for k in up if k[0] == ch]
        xu = crossing(up, keys) if keys else None
        near = min(snrs, key=lambda s: abs(s - xu)) if xu is not None else snrs[len(snrs) // 2]
        pick["low"] += sorted(by_cell[(ch, snrs[0])], key=lambda f: f[3])[:per_cell]
        pick["crossing"] += sorted(by_cell[(ch, near)], key=lambda f: f[3])[:per_cell]

    binary = upstream_binary(t)
    wd = tempfile.mkdtemp()
    shutil.copy(pick["crossing"][0][0], os.path.join(wd, "000000_0000.wav"))
    subprocess.run([binary, *t["upstream_args"], "000000_0000.wav"], cwd=wd, capture_output=True)  # warm FFTW wisdom
    exe = crate_test_binary(t)
    print(f"{task}: ms per file, one file at a time, single-threaded")
    for kind, fs in pick.items():
        up_ms = []
        for f in fs:
            shutil.copy(f[0], os.path.join(wd, "000000_0000.wav"))
            subprocess.run([binary, *t["upstream_args"], "000000_0000.wav"], cwd=wd, capture_output=True)
            with open(os.path.join(wd, "timer.out")) as fh:
                total = next(float(l.split()[1]) for l in fh if l.split()[:1] == [t["upstream_bin"]])
            up_ms.append(total * 1000)
        sub = tempfile.mkdtemp()
        for f in fs:
            os.symlink(f[0], os.path.join(sub, os.path.basename(f[0])))
        csv_out = os.path.join(sub, "rows.csv")
        env = dict(os.environ, RAYON_NUM_THREADS="1", **t["sweep_env"],
                   **{t["sweep_dir_env"]: sub, t["sweep_csv_env"]: csv_out})
        t0 = time.perf_counter()
        r = subprocess.run([exe, t["sweep_filter"], "--ignored", "--exact", "--nocapture"],
                           env=env, capture_output=True, text=True)
        crate_ms = (time.perf_counter() - t0) * 1000 / len(fs)
        rows = len(read_rows(csv_out)) if os.path.exists(csv_out) else 0
        if "1 passed" not in r.stdout or rows != len(fs):
            sys.exit(f"the crate run decoded {rows} of {len(fs)} files:\n{r.stdout[-1500:]}{r.stderr[-1500:]}")
        shutil.rmtree(sub)
        u = sum(up_ms) / len(up_ms)
        print(f"  {kind:9} {len(fs):3} files  upstream {u:7.1f}  crate {crate_ms:7.1f}  ratio {crate_ms / u:5.2f}")
    shutil.rmtree(wd)


def cmd_run(task, csv_dir, alpha, extra_tol, per_cell):
    t = tasks()[task]
    os.makedirs(csv_dir, exist_ok=True)
    out = os.path.join(csv_dir, task.replace("/", "_") + ".csv")
    exe = crate_test_binary(t)
    if os.path.exists(out):
        os.remove(out)  # the sweep tests append
    # The test resolves its corpus from CARGO_MANIFEST_DIR when the dir env is
    # unset, which only `cargo test` provides; run directly, it would find no
    # files and pass having done nothing. Name the corpus, and count the rows.
    env = dict(os.environ, **t["sweep_env"], **{t["sweep_csv_env"]: out, t["sweep_dir_env"]: corpus_dir(t)})
    r = subprocess.run([exe, t["sweep_filter"], "--ignored", "--exact"], env=env, capture_output=True, text=True)
    rows = len(read_rows(out)) if os.path.exists(out) else 0
    if "1 passed" not in r.stdout or rows != len(corpus_files(t)):
        sys.exit(f"the crate sweep for {task} wrote {rows} of {len(corpus_files(t))} trials:\n"
                 f"{r.stdout[-1500:]}{r.stderr[-1500:]}")
    status = cmd_compare(task, out, alpha, extra_tol)
    print()
    cmd_time(task, per_cell)
    return status


def crate_test_binary(t):
    r = subprocess.run(
        ["cargo", "test", "--release", "-p", "mfsk-core", "--features", "full,internal-testing",
         "--test", t["sweep_test"], "--no-run", "--message-format=json"],
        cwd=ROOT, capture_output=True, text=True)
    for line in r.stdout.splitlines():
        try:
            j = json.loads(line)
        except ValueError:
            continue
        if j.get("executable") and os.path.basename(j["executable"]).startswith(t["sweep_test"] + "-"):
            return j["executable"]
    sys.exit(f"could not build {t['sweep_test']}:\n{r.stderr[-2000:]}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("generate")
    g.add_argument("task")
    g.add_argument("--jobs", type=int, default=os.cpu_count())
    c = sub.add_parser("compare")
    c.add_argument("task")
    c.add_argument("crate_csv")
    c.add_argument("--alpha", type=float, default=0.05)
    c.add_argument("--extra-tol", type=int, default=3)
    tm = sub.add_parser("time")
    tm.add_argument("task")
    tm.add_argument("--per-cell", type=int, default=5)
    rn = sub.add_parser("run")
    rn.add_argument("task")
    rn.add_argument("csv_dir")
    rn.add_argument("--alpha", type=float, default=0.05)
    rn.add_argument("--extra-tol", type=int, default=3)
    rn.add_argument("--per-cell", type=int, default=5)
    a = ap.parse_args()
    if a.task not in tasks():
        sys.exit(f"unknown task {a.task}; known: {', '.join(tasks())}")
    if a.cmd == "generate":
        cmd_generate(a.task, a.jobs)
    elif a.cmd == "compare":
        sys.exit(cmd_compare(a.task, a.crate_csv, a.alpha, a.extra_tol))
    elif a.cmd == "run":
        sys.exit(cmd_run(a.task, a.csv_dir, a.alpha, a.extra_tol, a.per_cell))
    else:
        cmd_time(a.task, a.per_cell)


if __name__ == "__main__":
    main()
