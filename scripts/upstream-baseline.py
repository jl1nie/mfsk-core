#!/usr/bin/env python3
"""The upstream baseline: this crate against WSJT-X on the same task and the same files.

`sweep-baseline.json` compares this crate with its own past. That cannot show a
decoder that was slower or weaker than WSJT-X from the start: Q65 went
2.9-4.5x slower than `jt9` unnoticed until #552. This compares against upstream,
per trial, on the tier-C corpora that already exist. It adds no corpus and
nothing to CI.

A task is defined once, in `scripts/upstream_tasks.json`: the upstream command
line and the crate request that does the same job. The upstream half runs once
per upstream release or corpus, and its per-trial outcome is committed under
`docs/notes/upstream/`. The crate half is the sweep test run with the task's
`sweep_env`.

    scripts/upstream-baseline.py generate ft8/t1       # run upstream, write docs/notes/upstream/ft8_t1.csv
    scripts/upstream-baseline.py compare  ft8/t1 <crate.csv>
    scripts/upstream-baseline.py time     ft8/t1 [--per-cell 2]
    scripts/upstream-baseline.py run      ft8/t1 <csv-dir>   # crate sweep of the task, compare, time
    scripts/upstream-baseline.py tasks                        # list them

`run` is what `scripts/run-sensitivity-sweeps.sh` calls for every task of a
protocol it sweeps: one extra sweep of that corpus, narrowed like the runner's
own for FST4, and the timing.

`compare` pairs every trial. A group is flagged `!!` when this crate misses
significantly more of the files upstream decodes than the other way round
(exact McNemar, p < 0.05). Pairing is what makes the existing 20-trial cells
enough: two 20-trial crossings cannot resolve 0.3 dB, but 260 paired trials
can. The crossing delta is printed as a summary. Unexpected decodes are flagged
when the crate's exceed upstream's by more than `--extra-tol`, for tasks whose
crate CSV counts them (FT8, FT4, FST4).

`time` runs both sides one file at a time, single-threaded, on a few files per
group: the lowest-SNR cell, effectively noise, and the cell nearest
upstream's crossing. For `jt9` the time is `timer.out`'s total, which leaves
out process start, with FFTW wisdom warm. `wsprd`'s timer resolves only
10 ms, so its time is the process wall clock. It prints the ratio.
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
UPSTREAM_TAG = "v3.2.0-rc1"


def _load(name, file):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, file))
    mod = importlib.util.module_from_spec(spec)
    saved, sys.argv = sys.argv, [file]
    spec.loader.exec_module(mod)
    sys.argv = saved
    return mod


_chk = _load("sweep_regression_check", "sweep-regression-check.py")  # crossing_snr


def tasks():
    with open(os.path.join(HERE, "upstream_tasks.json")) as fh:
        return {k: v for k, v in json.load(fh).items() if not k.startswith("_")}


def out_path(task):
    return os.path.join(ROOT, "docs", "notes", "upstream", task.replace("/", "_") + ".csv")


def upstream_binary(t):
    return os.path.join(ROOT, "target", "upstream", f"build-{UPSTREAM_TAG}", t["upstream_bin"])


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
    """[(path, group-dict, snr, trial)] -- group from the file name's named groups."""
    rx = re.compile(t["file_re"])
    out = []
    for p in sorted(glob.glob(os.path.join(corpus_dir(t), "*.wav"))):
        m = rx.match(os.path.basename(p))
        if m:
            tag = m["tag"]
            snr = -int(tag[1:]) if tag[0] == "m" else int(tag[1:])
            out.append((p, {g: m[g] for g in t["groups"]}, snr, int(m["trial"])))
    return out


def upstream_args(t, group):
    sub = dict(group)
    if "submode" in group:  # q65: a60 -> period 60, letter A
        sub["period"], sub["letter"] = group["submode"][1:], group["submode"][0].upper()
    return [a.format(**sub) for a in t["upstream_args"]]


# ── upstream output ────────────────────────────────────────────────────────

_JT9 = re.compile(r"\s*\d+\s+-?\d+\s+(-?[\d.]+)\s+(\d+)\s+\S\s+(.*?)\s*$")
_TAIL = re.compile(r"(\s+\?)?(\s+a\d+)?(\s+(q\d))?\s*$")
_WSPRD = re.compile(r"\s*\S+\s+-?\d+\s+(-?[\d.]+)\s+([\d.]+)\s+-?\d+\s+(.*?)\s*$")


def parse(stdout, fmt):
    """[(dt, freq_hz, message, qtype)] from one upstream run."""
    out = []
    for line in stdout.splitlines():
        if "DecodeFinished" in line:
            continue
        if fmt == "wsprd":
            m = _WSPRD.match(line)
            if m:
                out.append((float(m[1]), float(m[2]) * 1e6, m[3], None))
        else:
            m = _JT9.match(line)
            if m:
                tail = _TAIL.search(m[3])
                out.append((float(m[1]), float(m[2]), _TAIL.sub("", m[3]), tail[4] if tail else None))
    return out


def score(t, decodes, qtypes=None):
    s = t["score"]
    hit = any(
        msg == s["msg"]
        and abs(f - s["freq"]) <= s["freq_tol"]
        and (s["dt_tol"] is None or abs(dt) <= s["dt_tol"])
        and (qtypes is None or q in qtypes)
        for dt, f, msg, q in decodes
    )
    return hit, len({msg for _dt, _f, msg, _q in decodes if msg != s["msg"]})


def run_upstream(binary, args, wav, cwd):
    shutil.copy(wav, os.path.join(cwd, "000000_0000.wav"))
    t0 = time.perf_counter()
    r = subprocess.run([binary, *args, "000000_0000.wav"], cwd=cwd, capture_output=True, text=True)
    return r.stdout, time.perf_counter() - t0


def _gen_one(a):
    task, wav, group = a
    t = tasks()[task]
    with tempfile.TemporaryDirectory() as d:
        stdout, _ = run_upstream(upstream_binary(t), upstream_args(t, group), wav, d)
    dec = parse(stdout, t["score"].get("format", "jt9"))
    if "modes" in t:
        return {m: score(t, dec, set(q)) for m, q in t["modes"].items()}
    return {None: score(t, dec)}


def cmd_generate(task, jobs):
    t = tasks()[task]
    binary = upstream_binary(t)
    if not os.path.exists(binary):
        sys.exit(f"{binary}: missing -- run scripts/build_jt9_upstream.sh")
    files = corpus_files(t)
    stamp = corpus_stamp(t)
    with multiprocessing.Pool(jobs) as pool:
        res = pool.map(_gen_one, [(task, f[0], f[1]) for f in files], chunksize=2)
    path = out_path(task)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cols = t["groups"] + (["mode"] if "modes" in t else [])
    with open(path, "w") as fh:
        fh.write(f"# task={task}\n")
        fh.write(f"# upstream={t['upstream_bin']} {UPSTREAM_TAG} sha256={sha256(binary)}\n")
        fh.write(f"# upstream_args={' '.join(t['upstream_args'])}\n")
        fh.write(f"# corpus={t['corpus']} seed={stamp.get('seed')} simulator_sha256={stamp.get('simulator_sha256')}"
                 f" stamp_commit={stamp.get('commit')}\n")
        fh.write(f"# written by scripts/upstream-baseline.py generate; machine={platform.machine()}\n")
        fh.write(",".join(cols + ["snr_db", "trial", "pass", "extra"]) + "\n")
        for (p, g, snr, trial), r in zip(files, res):
            for mode, (hit, extra) in r.items():
                key = [g[c] for c in t["groups"]] + ([mode] if mode else [])
                fh.write(",".join(key + [str(snr), str(trial), str(int(hit)), str(extra)]) + "\n")
    print(f"wrote {path}: {len(files)} files")


# ── compare ────────────────────────────────────────────────────────────────

def read_rows(path, t, crate=False):
    """{(group..., [mode,] snr, trial): (pass, extra or None)}. The crate's FST4 CSV
    names the period `mode`, which is the task's `mode` group too."""
    with open(path) as fh:
        lines = [l for l in fh if not l.startswith("#")]
    rows = {}
    for r in csv.DictReader(lines):
        key = tuple(r[c] for c in t["groups"])
        if "modes" in t:
            key += (r["mode"],)
        extra = int(r["extra"]) if r.get("extra") not in (None, "") else None
        rows[key + (int(r["snr_db"]), int(r["trial"]))] = (int(r["pass"]), extra)
    if crate and "crate_modes" in t:  # Q65: this crate's cq = its plain scan, then its CQ scan
        combined = {}
        for k in rows:
            *g, _mode, snr, trial = k
            for name, parts in t["crate_modes"].items():
                hit = any(rows.get(tuple(g) + (p, snr, trial), (0, None))[0] for p in parts)
                combined[tuple(g) + (name, snr, trial)] = (int(hit), None)
        rows = combined
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
        cells[k[-2]][0] += rows[k][0]
        cells[k[-2]][1] += 1
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
    up, me = read_rows(up_path, t), read_rows(crate_csv, t, crate=True)
    shared = sorted(set(up) & set(me))
    if not shared:
        sys.exit("no trials in common")
    print(f"{task}: {t['task']}")
    print(f"upstream: {h.get('upstream')} {h.get('upstream_args', '')}".rstrip())
    print(f"{'group':22} {'trials':>6} {'both':>5} {'up only':>8} {'crate only':>10} {'p':>6}"
          f" {'x up':>7} {'x crate':>8} {'delta':>6} {'extra up/crate':>15}")
    flagged = False
    for g in sorted({k[:-2] for k in shared}, key=lambda g: [int(x) if x.isdigit() else x for x in g]):
        keys = [k for k in shared if k[:-2] == g]
        both = sum(up[k][0] and me[k][0] for k in keys)
        up_only = sum(up[k][0] and not me[k][0] for k in keys)
        me_only = sum(me[k][0] and not up[k][0] for k in keys)
        p = mcnemar_p(up_only, me_only)
        xu, xm = crossing(up, keys), crossing(me, keys)
        counted = all(me[k][1] is not None for k in keys)
        eu = sum(up[k][1] or 0 for k in keys)
        em = sum(me[k][1] or 0 for k in keys) if counted else None
        bad = (p < alpha and up_only > me_only) or (em is not None and em > eu + extra_tol)
        flagged |= bad
        fmt = lambda v: f"{v:7.2f}" if v is not None else "      -"
        delta = f"{xm - xu:+6.2f}" if xu is not None and xm is not None else "     -"
        ex = f"{eu:7}/{em:<7}" if em is not None else f"{eu:7}/{'-':<7}"
        print(f"{'/'.join(g):22} {len(keys):6} {both:5} {up_only:8} {me_only:10} {p:6.3f} {fmt(xu)} {fmt(xm):>8}"
              f" {delta} {ex}{'  !!' if bad else ''}")
    print("\n!! = significantly more upstream-only decodes than crate-only (McNemar), or more unexpected"
          f" decodes than upstream + {extra_tol}." if flagged else "\nno group behind upstream.")
    return 1 if flagged else 0


# ── the crate's sweep ──────────────────────────────────────────────────────

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


def rows_per_file(t):
    return len(t["modes"]) if "modes" in t else 1


def run_crate(t, exe, corpus, out, extra_env=None):
    """Run the task's sweep over `corpus` into `out`. The test resolves its corpus
    from CARGO_MANIFEST_DIR when the dir env is unset, which only `cargo test`
    provides; run directly it would find nothing and pass having done nothing,
    so the dir is always named and the caller counts the rows."""
    if os.path.exists(out):
        os.remove(out)  # the sweep tests append
    env = dict(os.environ, **t["sweep_env"], **(extra_env or {}),
               **{t["sweep_csv_env"]: out, t["sweep_dir_env"]: corpus})
    r = subprocess.run([exe, t["sweep_filter"], "--ignored", "--exact", "--nocapture"],
                       env=env, capture_output=True, text=True)
    if "1 passed" not in r.stdout:
        sys.exit(f"the crate sweep failed:\n{r.stdout[-1500:]}{r.stderr[-1500:]}")


def narrow_windows(kind):
    """{mode: (lo, hi)} from sweep-narrow-plan.py, as run-sensitivity-sweeps.sh narrows FST4."""
    r = subprocess.run([sys.executable, os.path.join(HERE, "sweep-narrow-plan.py")], capture_output=True, text=True)
    out = {}
    for line in r.stdout.splitlines():
        p = line.split("\t")
        if len(p) == 4 and p[0] == kind:
            out[p[1]] = (p[2], p[3])
    return out


def cmd_run(task, csv_dir, alpha, extra_tol, per_cell):
    t = tasks()[task]
    os.makedirs(csv_dir, exist_ok=True)
    out = os.path.join(csv_dir, task.replace("/", "_") + ".csv")
    exe = crate_test_binary(t)
    files = corpus_files(t)
    if t.get("narrow") == "fst4":
        # The same windows as the runner's own FST4 sweep, so the task costs what that does.
        wins = narrow_windows("fst4")
        merged, expected = [], 0
        for mode in sorted({f[1]["mode"] for f in files}, key=int):
            env = {"MFSK_FST4_SWEEP_MODES": mode}
            lo, hi = wins.get(mode, (None, None))
            if lo is not None:
                env.update(MFSK_FST4_SWEEP_SNR_MIN=lo, MFSK_FST4_SWEEP_SNR_MAX=hi)
            expected += sum(1 for f in files if f[1]["mode"] == mode
                            and (lo is None or int(lo) <= f[2] <= int(hi)))
            part = out + f".{mode}"
            run_crate(t, exe, corpus_dir(t), part, env)
            with open(part) as fh:
                lines = fh.readlines()
            merged += lines if not merged else lines[1:]
            os.remove(part)
        with open(out, "w") as fh:
            fh.writelines(merged)
    else:
        run_crate(t, exe, corpus_dir(t), out)
        expected = len(files)
    n = len(read_rows(out, t))
    if n != expected * rows_per_file(t):
        sys.exit(f"the crate sweep for {task} wrote {n} rows, expected {expected * rows_per_file(t)}")
    status = cmd_compare(task, out, alpha, extra_tol)
    print()
    cmd_time(task, per_cell)
    return status


# ── time ───────────────────────────────────────────────────────────────────

def cmd_time(task, per_cell):
    t = tasks()[task]
    per_cell = t.get("time_per_cell", per_cell)
    up = read_rows(out_path(task), t)
    files = corpus_files(t)
    gkey = lambda g: tuple(g[c] for c in t["groups"])
    by_cell = collections.defaultdict(list)
    for f in files:
        by_cell[(gkey(f[1]), f[2])].append(f)
    pick = {"low": [], "crossing": []}
    for g in sorted({gkey(f[1]) for f in files}):
        snrs = sorted(s for gg, s in by_cell if gg == g)
        keys = [k for k in up if tuple(k[:len(g)]) == g and ("modes" not in t or k[len(g)] == "cq")]
        xu = crossing(up, keys) if keys else None
        near = min(snrs, key=lambda s: abs(s - xu)) if xu is not None else snrs[len(snrs) // 2]
        pick["low"] += sorted(by_cell[(g, snrs[0])], key=lambda f: f[3])[:per_cell]
        pick["crossing"] += sorted(by_cell[(g, near)], key=lambda f: f[3])[:per_cell]

    binary = upstream_binary(t)
    wd = tempfile.mkdtemp()
    run_upstream(binary, upstream_args(t, pick["crossing"][0][1]), pick["crossing"][0][0], wd)  # warm wisdom
    exe = crate_test_binary(t)
    print(f"{task}: ms per file, one file at a time, single-threaded")
    for kind, fs in pick.items():
        up_ms = []
        for f in fs:
            _stdout, wall = run_upstream(binary, upstream_args(t, f[1]), f[0], wd)
            if t["upstream_bin"] == "jt9":
                with open(os.path.join(wd, "timer.out")) as fh:
                    wall = next(float(l.split()[1]) for l in fh if l.split()[:1] == ["jt9"])
            up_ms.append(wall * 1000)
        sub = tempfile.mkdtemp()
        for f in fs:
            os.symlink(f[0], os.path.join(sub, os.path.basename(f[0])))
        csv_out = os.path.join(sub, "rows.csv")
        extra = {"RAYON_NUM_THREADS": "1"}
        if t.get("narrow") == "fst4":
            extra["MFSK_FST4_SWEEP_MODES"] = ",".join(sorted({f[1]["mode"] for f in fs}, key=int))
        t0 = time.perf_counter()
        run_crate(t, exe, sub, csv_out, extra)
        crate_ms = (time.perf_counter() - t0) * 1000 / len(fs)
        n = len(read_rows(csv_out, t))
        if n != len(fs) * rows_per_file(t):
            sys.exit(f"the timed crate run wrote {n} rows for {len(fs)} files")
        shutil.rmtree(sub)
        u = sum(up_ms) / len(up_ms)
        print(f"  {kind:9} {len(fs):3} files  upstream {u:8.1f}  crate {crate_ms:8.1f}  ratio {crate_ms / u:5.2f}")
    shutil.rmtree(wd)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("tasks")
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
    tm.add_argument("--per-cell", type=int, default=2)
    rn = sub.add_parser("run")
    rn.add_argument("task")
    rn.add_argument("csv_dir")
    rn.add_argument("--alpha", type=float, default=0.05)
    rn.add_argument("--extra-tol", type=int, default=3)
    rn.add_argument("--per-cell", type=int, default=2)
    a = ap.parse_args()
    if a.cmd == "tasks":
        for k, v in tasks().items():
            print(f"{k:10} {v['task']}")
        return
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
