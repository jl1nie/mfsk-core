#!/usr/bin/env python3
"""Diff a tier-C sensitivity sweep run against the last recorded baseline.

Replaces the "spawn N agents to eyeball a 2600-line log against
docs/notes/BENCHMARKS.md" step from the 2026-08-14 sweep session with an
automated pass: read the per-trial CSVs the sweep tests can emit,
interpolate each channel's 50%-recall crossing SNR, and print how far it
moved from the stored baseline.

Input format (one CSV per sweep test, written via the tests' own
MFSK_*_SWEEP_CSV / MFSK_*_SWEEP_SUMMARY_CSV env vars — see
scripts/run-sensitivity-sweeps.sh, which sets these automatically and
calls this script at the end of a run):

    <dim1>,<dim2>,...,snr_db,trial,pass[,extra]

Any number of leading "dimension" columns (mode, channel, submode, ap
mode, ...) is allowed — everything before the fixed `snr_db,trial,pass`
triple is treated as the group key. One row per trial; `pass` is 0/1.

The optional trailing `extra` column is the **precision** half: how many
distinct decoded messages were *not* the one signal the file was generated
with. Every sweep WAV holds one injected transmission plus noise, so each
of those is a CRC-valid payload out of noise. (Before 2026-09 tier C
recorded recall only; precision lived in tier B's real recordings alone.)
CSVs without the column still load, and simply have no precision report.

A second, "aggregate" shape serves corpora that have no SNR ladder to
interpolate, such as the busy-band corpus (several signals per file):

    <dim1>,...,trial,truth,hits,extra

One row per file: `truth` signals were sent, `hits` decoded, `extra` were
unexpected. Groups are compared by total recall (hits/truth) and total
`extra`; the baseline keeps them under `_meta.aggregates`. A group is
flagged when recall falls by AGG_RECALL_DROP (one point) or when `extra`
rises by the precision rule.

Baseline file format (docs/notes/sweep-baseline.json): a flat
`{"<protocol>[/<dim>...]": crossing_db}` map, plus one `_meta` key
carrying per-protocol provenance — when each protocol's numbers were
measured, at which commit, on which machine, over how many trials — and
`_meta.precision`, `{group: {snr_db: [extra, trials]}}`, the unexpected
decodes per SNR cell. Precision is compared over the SNR cells present in
both the run and the baseline, so a narrowed re-run is still comparable.
`--update-baseline` refreshes `_meta` for the protocols in the run and
leaves the rest untouched, so a partial re-sweep can't backdate-launder
groups it never ran. Nothing reads the crossings out of `_meta`; a
consumer that only wants a number can keep ignoring `_`-prefixed keys.

Usage:
    scripts/sweep-regression-check.py CSV [CSV ...]
    scripts/sweep-regression-check.py --update-baseline CSV [CSV ...]
    scripts/sweep-regression-check.py --baseline path.json CSV [CSV ...]

This prints a report and always exits 0 — like the sweep tests
themselves, a moved threshold is a judgement call for a human, not a
boolean pass/fail (see run-sensitivity-sweeps.sh's "WHAT IT DOES NOT
DO"). Pass --strict to exit 1 when any group moved beyond --threshold
dB, or gained unexpected decodes beyond the precision rule below, for a
caller that wants a hard gate anyway.

Precision rule: a group is flagged when, over the shared SNR cells, it has
at least PRECISION_MIN_INCREASE more unexpected decodes than the baseline
AND at least PRECISION_RATIO times as many. A handful of extras moves with
any recall change, so a flat count would cry wolf; one file's worth of
noise decodes must not flag, while a doubled phantom rate must.
"""
import argparse
import csv as csv_mod
import datetime
import json
import platform
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BASELINE = REPO_ROOT / "docs" / "notes" / "sweep-baseline.json"

# The baseline file is `{group: crossing_db}` plus one `_meta` key.
# Anything `_`-prefixed is provenance, never a group — the group
# namespace is `<protocol>[/<dim>...]` and a protocol directory can't
# start with an underscore.
META_KEY = "_meta"

# Precision regression rule — see the module docstring.
PRECISION_MIN_INCREASE = 3
PRECISION_RATIO = 1.5

# Aggregate corpora: an absolute drop in total recall that is flagged.
AGG_RECALL_DROP = 0.01


def protocol_of(label: str) -> str:
    """`fst4/60/awgn` -> `fst4`. The granularity both consumers work at:
    run-sensitivity-sweeps.sh takes protocol names as arguments, and
    release-status.sh decides what to re-sweep per `mfsk-core/src/<p>`.
    Recording provenance per group instead would be 54 entries saying
    the same date, and per *file* would be one date that a partial
    refresh silently makes wrong for the 40 groups it didn't touch."""
    return label.split("/", 1)[0]


def machine_id() -> str:
    """Which box produced the numbers. A 50%-crossing is a decoder
    property and shouldn't move between machines, but `parallel` means
    the sweeps are rayon-scheduled and a differing core count has
    already been mistaken for a regression once (BENCHMARKS.md records
    the machine for the same reason)."""
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.platform()


def git_commit() -> str | None:
    """Short SHA the run was measured at, `-dirty` when *tracked* files
    differed from HEAD — a baseline measured on an uncommitted decoder
    edit is not reproducible and should say so.

    Untracked files deliberately do not count. They cannot change what
    was built, and letting them count made every stamp read `-dirty` for
    anyone with a scratch directory in the tree (2026-09-23: an
    unrelated `embedded-poc/m5stack-core2/` did exactly that), which
    turns the marker into noise precisely when it needs to be trusted."""
    def run(*argv):
        return subprocess.run(
            argv, cwd=REPO_ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()
    try:
        sha = run("git", "rev-parse", "--short", "HEAD")
    except (OSError, subprocess.CalledProcessError):
        return None
    try:
        if run("git", "status", "--porcelain", "--untracked-files=no"):
            sha += "-dirty"
    except (OSError, subprocess.CalledProcessError):
        pass
    return sha


def protocol_name_from_path(path: Path) -> str:
    # wspr.csv -> wspr, fst4_awgn_run3.csv -> fst4 (first underscore-free
    # token isn't assumed; caller can pass an explicit name via NAME=path).
    return path.stem


def load_cells(path: Path):
    """Returns {group_key_tuple: {snr_db: [n_hits, n_trials]}}.

    Kept as its own function because other scripts import it; the
    precision column is read by `load_cells_and_extras`."""
    return load_cells_and_extras(path)[0]


def load_cells_and_extras(path: Path):
    """Returns `(cells, extras)`.

    `cells` is `{group_key_tuple: {snr_db: [n_hits, n_trials]}}`.
    `extras` is `{group_key_tuple: {snr_db: [n_extra, n_trials]}}`, empty
    when the CSV has no `extra` column (an older CSV, or a suite that does
    not count unexpected decodes)."""
    cells = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    extras = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    with path.open(newline="") as f:
        reader = csv_mod.reader(f)
        header = next(reader, None)
        if header is None:
            return cells, extras
        if "snr_db" not in header:
            raise ValueError(f"{path}: no snr_db column in header {header}")
        i = header.index("snr_db")
        if header[i : i + 3] != ["snr_db", "trial", "pass"]:
            raise ValueError(
                f"{path}: expected snr_db,trial,pass[,extra], got {header}"
            )
        trailing = header[i + 3 :]
        if trailing not in ([], ["extra"]):
            raise ValueError(f"{path}: unexpected trailing columns {trailing}")
        has_extra = trailing == ["extra"]
        for row in reader:
            if not row:
                continue
            key = tuple(row[:i])
            snr = int(row[i])
            passed = row[i + 2] == "1"
            cell = cells[key][snr]
            cell[1] += 1
            if passed:
                cell[0] += 1
            if has_extra:
                e = extras[key][snr]
                e[1] += 1
                e[0] += int(row[i + 3])
    return cells, extras


def crossing_snr(snr_hits_trials: dict, target=0.5):
    """Linear-interpolated SNR (dB) where recall first crosses `target`,
    scanning ascending SNR. Returns (crossing_or_None, note)."""
    points = sorted(
        (snr, h / t if t else 0.0, t) for snr, (h, t) in snr_hits_trials.items()
    )
    if not points:
        return None, "no data"
    if points[0][1] >= target:
        return None, f"already >={target:.0%} at lowest sampled SNR ({points[0][0]} dB)"
    if points[-1][1] < target:
        return None, f"never reaches {target:.0%} (max {points[-1][1]:.0%} at {points[-1][0]} dB)"
    prev = points[0]
    for cur in points[1:]:
        if prev[1] < target <= cur[1]:
            snr_lo, r_lo, _ = prev
            snr_hi, r_hi, _ = cur
            if r_hi == r_lo:
                return float(snr_lo), None
            frac = (target - r_lo) / (r_hi - r_lo)
            return snr_lo + frac * (snr_hi - snr_lo), None
        prev = cur
    return None, "no monotonic crossing found"  # defensive; shouldn't hit given checks above


def is_aggregate_csv(path: Path) -> bool:
    """`...,trial,truth,hits,extra` rather than `...,snr_db,trial,pass[,extra]`."""
    with path.open(newline="") as f:
        header = next(csv_mod.reader(f), None) or []
    return "truth" in header and "hits" in header and "snr_db" not in header


def load_aggregates(path: Path):
    """Returns `{group_key_tuple: {"files": n, "truth": T, "hits": H, "extra": E}}`.

    The group key is every column before `trial`."""
    out = defaultdict(lambda: {"files": 0, "truth": 0, "hits": 0, "extra": 0})
    with path.open(newline="") as f:
        reader = csv_mod.reader(f)
        header = next(reader, None)
        if header is None:
            return out
        try:
            i = header.index("trial")
            ti, hi, ei = header.index("truth"), header.index("hits"), header.index("extra")
        except ValueError:
            raise ValueError(f"{path}: expected ...,trial,truth,hits,extra, got {header}")
        for row in reader:
            if not row:
                continue
            g = out[tuple(row[:i])]
            g["files"] += 1
            g["truth"] += int(row[ti])
            g["hits"] += int(row[hi])
            g["extra"] += int(row[ei])
    return out


def compare_aggregate(cur: dict, base: dict):
    """`(recall_cur, recall_base, extra_flagged, recall_flagged)`. Recall is
    None for a group that sent nothing (a noise-only set)."""
    def recall(g):
        return g["hits"] / g["truth"] if g["truth"] else None
    rc, rb = recall(cur), recall(base)
    recall_flagged = rc is not None and rb is not None and (rb - rc) >= AGG_RECALL_DROP
    extra_flagged = (cur["extra"] - base["extra"]) >= PRECISION_MIN_INCREASE and cur["extra"] >= PRECISION_RATIO * base["extra"]
    return rc, rb, extra_flagged, recall_flagged


def compare_precision(current: dict, baseline: dict):
    """`current` / `baseline`: `{snr_db (int or str): [n_extra, n_trials]}`.

    Compares only the SNR cells present in both. Returns
    `(cur_extras, base_extras, common_trials, flagged)`; `flagged` follows
    PRECISION_MIN_INCREASE / PRECISION_RATIO. With no shared cell it returns
    `(None, None, 0, False)` — nothing to compare, and that is not a pass."""
    cur = {str(k): v for k, v in current.items()}
    base = {str(k): v for k, v in baseline.items()}
    common = sorted(set(cur) & set(base), key=int)
    if not common:
        return None, None, 0, False
    c = sum(cur[k][0] for k in common)
    b = sum(base[k][0] for k in common)
    n = sum(cur[k][1] for k in common)
    flagged = (c - b) >= PRECISION_MIN_INCREASE and c >= PRECISION_RATIO * b
    return c, b, n, flagged


def dump_baseline(out: dict) -> str:
    """`json.dumps(indent=2)`, except each `_meta.precision` group is one line.

    Plain `indent=2` puts every `[extra, trials]` pair on four lines, which
    turned the precision block into ~1,450 lines for 32 groups; one line per
    group keeps it reviewable in a diff."""
    prec = out.get(META_KEY, {}).get("precision")
    if not prec:
        return json.dumps(out, indent=2) + "\n"
    markers = {label: f"@@precision-{i}@@" for i, label in enumerate(prec)}
    shell = json.loads(json.dumps(out))
    shell[META_KEY]["precision"] = markers
    text = json.dumps(shell, indent=2)
    for label, marker in markers.items():
        text = text.replace(f'"{marker}"', json.dumps(prec[label], separators=(", ", ": ")))
    return text + "\n"


def group_label(proto: str, key: tuple) -> str:
    return "/".join((proto,) + key) if key else proto


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csvs", nargs="+", help="CSV files, optionally NAME=path.csv to override the protocol name")
    ap.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE, help=f"baseline JSON (default: {DEFAULT_BASELINE})")
    ap.add_argument("--update-baseline", action="store_true", help="write this run's crossings into the baseline file after reporting")
    ap.add_argument("--threshold", type=float, default=0.5, help="dB move to flag (default 0.5, matches CLAUDE.md's release-tag guidance)")
    ap.add_argument("--strict", action="store_true", help="exit 1 if any group moved beyond --threshold (default: always exit 0, this is advisory)")
    ap.add_argument("--keep", action="append", default=[], metavar="GROUP",
                    help="with --update-baseline: leave this group's recorded crossing alone (repeatable). "
                         "For a move you have not explained yet: the rest of the run is still recorded, "
                         "and the protocol's provenance lists the kept groups so the date is not read as covering them")
    ap.add_argument("--machine", default=None, help="override the recorded machine string (default: /proc/cpuinfo model name)")
    args = ap.parse_args()

    raw = {}
    if args.baseline.exists():
        raw = json.loads(args.baseline.read_text())
    meta = raw.get(META_KEY, {})
    baseline = {k: v for k, v in raw.items() if not k.startswith("_")}

    current = {}
    current_precision = {}  # label -> {snr_db: [extra, trials]}
    current_aggregates = {}  # label -> {"files", "truth", "hits", "extra"}
    rows = []  # (label, crossing_or_None, note)
    for spec in args.csvs:
        if "=" in spec and not spec.startswith("/") and not Path(spec).exists():
            name, path_s = spec.split("=", 1)
            path = Path(path_s)
        else:
            path = Path(spec)
            name = protocol_name_from_path(path)
        if not path.exists():
            print(f"warning: {path} not found, skipping", file=sys.stderr)
            continue
        if is_aggregate_csv(path):
            for key, agg in sorted(load_aggregates(path).items()):
                current_aggregates[group_label(name, key)] = agg
            continue
        cells, extras = load_cells_and_extras(path)
        for key, snr_map in sorted(cells.items()):
            label = group_label(name, key)
            crossing, note = crossing_snr(snr_map)
            trials_total = sum(t for _, t in snr_map.values())
            rows.append((label, crossing, note, trials_total))
            if crossing is not None:
                current[label] = crossing
            if key in extras:
                current_precision[label] = {
                    snr: list(v) for snr, v in sorted(extras[key].items())
                }

    if not rows and not current_aggregates:
        print("no cells found in any input CSV — nothing to report.")
        return 0

    # Print the baseline's own provenance before the diff. Without it a
    # "+0.00 dB" line says the number matches but not what it matches —
    # which is exactly the question that sent a reader to `git log` on
    # the JSON, and from there to guessing.
    per_proto = meta.get("protocols", {})
    seen_protos = sorted({protocol_of(r[0]) for r in rows} | {protocol_of(g) for g in current_aggregates})
    if per_proto:
        print("comparing against:")
        for proto in seen_protos:
            m = per_proto.get(proto)
            if m is None:
                print(f"  {proto:<12} no recorded baseline — first run for this protocol")
                continue
            bits = [m.get("date", "date unknown")]
            if m.get("commit"):
                bits.append(m["commit"])
            if m.get("trials"):
                bits.append(f"{m['trials']} trials")
            print(f"  {proto:<12} {', '.join(bits)}")
        print()

    width = max((len(r[0]) for r in rows), default=8)
    if rows:
        print(f"{'group':<{width}}  {'crossing':>10}  {'baseline':>10}  {'delta':>8}  trials")
        print("-" * (width + 46))
    any_flagged = False
    any_new = False
    for label, crossing, note, n in sorted(rows):
        base = baseline.get(label)
        cur_s = f"{crossing:+.2f} dB" if crossing is not None else "—"
        base_s = f"{base:+.2f} dB" if base is not None else "(none)"
        if crossing is None:
            delta_s = note or ""
        elif base is None:
            delta_s = "NEW"
            any_new = True
        else:
            delta = crossing - base
            flag = "!!" if abs(delta) >= args.threshold else ""
            delta_s = f"{delta:+.2f} dB {flag}".strip()
            if flag:
                any_flagged = True
        print(f"{label:<{width}}  {cur_s:>10}  {base_s:>10}  {delta_s:>8}  {n}")

    # Precision: unexpected decodes, compared over the SNR cells shared with
    # the baseline. Independent of the recall crossing above — a group can
    # hold its crossing and still start manufacturing phantoms.
    precision_base = meta.get("precision", {})
    any_precision_flagged = False
    any_precision_new = False
    if current_precision:
        print()
        print("precision (unexpected decodes, over SNR cells shared with the baseline)")
        pw = max(len(label) for label in current_precision)
        print(f"{'group':<{pw}}  {'extras':>7}  {'baseline':>8}  {'delta':>7}  trials")
        print("-" * (pw + 36))
        for label in sorted(current_precision):
            cur_cells = current_precision[label]
            if label not in precision_base:
                total = sum(v[0] for v in cur_cells.values())
                n = sum(v[1] for v in cur_cells.values())
                print(f"{label:<{pw}}  {total:>7}  {'(none)':>8}  {'NEW':>7}  {n}")
                any_precision_new = True
                continue
            c, b, n, flagged = compare_precision(cur_cells, precision_base[label])
            if c is None:
                print(f"{label:<{pw}}  {'—':>7}  {'—':>8}  {'no shared SNR cell':>7}")
                continue
            mark = " !!" if flagged else ""
            any_precision_flagged = any_precision_flagged or flagged
            print(f"{label:<{pw}}  {c:>7}  {b:>8}  {c - b:>+7}{mark}  {n}")

    # Aggregate corpora (busy band): total recall and total unexpected decodes.
    agg_base = meta.get("aggregates", {})
    any_agg_flagged = False
    any_agg_new = False
    if current_aggregates:
        print()
        print("aggregate corpora (recall = hits/truth over all files; extras = unexpected decodes)")
        aw = max(len(label) for label in current_aggregates)
        print(f"{'group':<{aw}}  {'files':>5}  {'truth':>6}  {'recall':>7}  {'baseline':>8}  {'extras':>6}  {'baseline':>8}")
        print("-" * (aw + 56))
        for label in sorted(current_aggregates):
            g = current_aggregates[label]
            rec_s = f"{100 * g['hits'] / g['truth']:.1f}%" if g["truth"] else "-"
            b = agg_base.get(label)
            if b is None:
                any_agg_new = True
                print(f"{label:<{aw}}  {g['files']:>5}  {g['truth']:>6}  {rec_s:>7}  {'(none)':>8}  {g['extra']:>6}  {'NEW':>8}")
                continue
            rc, rb, extra_flagged, recall_flagged = compare_aggregate(g, b)
            rb_s = f"{100 * rb:.1f}%" if rb is not None else "-"
            marks = ("!! recall" if recall_flagged else "") + (" !! extras" if extra_flagged else "")
            any_agg_flagged = any_agg_flagged or recall_flagged or extra_flagged
            print(f"{label:<{aw}}  {g['files']:>5}  {g['truth']:>6}  {rec_s:>7}  {rb_s:>8}  {g['extra']:>6}  {b['extra']:>8} {marks}")

    print()
    if any_agg_flagged:
        print(f"⚠ an aggregate group lost >= {100 * AGG_RECALL_DROP:.0f} point of recall, or gained unexpected decodes (!! above).")
    if any_agg_new:
        print("  aggregate groups marked NEW have no baseline yet — add with --update-baseline.")
    if any_precision_flagged:
        print(f"⚠ unexpected decodes rose by >= {PRECISION_MIN_INCREASE} and >= {PRECISION_RATIO}x the baseline")
        print("  in one or more groups (!! above). Decide whether the recall gain that came with them")
        print("  is worth it, or whether an acceptance gate is too loose.")
    if any_precision_new:
        print("  precision groups marked NEW have no baseline yet — add with --update-baseline.")
    if any_flagged:
        print(f"⚠ one or more groups moved >= {args.threshold} dB from baseline — see CLAUDE.md's")
        print("  release-tag guidance ('a move worse than ~0.5 dB is worth explaining').")
    if any_new:
        print("  groups marked NEW have no baseline entry yet — add with --update-baseline")
        print("  once you've confirmed the number is correct, not just new.")
    if not any_flagged and not any_new and not any_precision_flagged and not any_agg_flagged:
        print("no groups moved beyond threshold.")

    if args.update_baseline:
        kept = sorted(g for g in args.keep if g in baseline)
        unknown = sorted(set(args.keep) - set(kept))
        if unknown:
            print(f"warning: --keep names no recorded group: {', '.join(unknown)}", file=sys.stderr)
        for g in kept:
            current.pop(g, None)
        baseline.update(current)

        # Provenance is refreshed for the protocols this run actually
        # measured and left alone for every other — a partial refresh
        # (`run-sensitivity-sweeps.sh ft4 fst4`) must not stamp today's
        # date onto the 40 groups it never ran.
        trials_by_proto = defaultdict(int)
        groups_by_proto = defaultdict(int)
        for label, _crossing, _note, n in rows:
            if label in current:
                trials_by_proto[protocol_of(label)] += n
                groups_by_proto[protocol_of(label)] += 1
        stamp = {
            "date": datetime.date.today().isoformat(),
            "commit": git_commit(),
            "machine": args.machine or machine_id(),
        }
        protocols = dict(meta.get("protocols", {}))
        for proto, n_trials in trials_by_proto.items():
            protocols[proto] = {
                **stamp,
                "trials": n_trials,
                "groups": groups_by_proto[proto],
            }
            kept_here = [g for g in kept if protocol_of(g) == proto]
            if kept_here:
                protocols[proto]["kept_crossings"] = kept_here
        # Precision cells replace, group by group, only for the groups this
        # run measured (same partial-refresh rule as the provenance above).
        # A group that ran without an `extra` column keeps its old entry.
        precision = dict(meta.get("precision", {}))
        for label, cells_by_snr in current_precision.items():
            precision[label] = {
                str(snr): v for snr, v in sorted(cells_by_snr.items())
            }
        meta = {
            **meta,
            "schema": 1,
            "protocols": dict(sorted(protocols.items())),
        }
        if precision:
            meta["precision"] = dict(sorted(precision.items()))
        if current_aggregates:
            aggregates = dict(meta.get("aggregates", {}))
            aggregates.update({label: dict(g) for label, g in current_aggregates.items()})
            meta["aggregates"] = dict(sorted(aggregates.items()))
            by_proto = defaultdict(list)
            for label, g in current_aggregates.items():
                by_proto[protocol_of(label)].append(g)
            protos = dict(meta["protocols"])
            for proto, gs in by_proto.items():
                protos[proto] = {**stamp, "trials": sum(g["files"] for g in gs), "groups": len(gs)}
            meta["protocols"] = dict(sorted(protos.items()))

        out = {META_KEY: meta}
        out.update(sorted(baseline.items()))
        args.baseline.parent.mkdir(parents=True, exist_ok=True)
        args.baseline.write_text(dump_baseline(out))
        stamped = ", ".join(sorted(set(trials_by_proto) | {protocol_of(l) for l in current_aggregates})) or "nothing"
        print(f"\nbaseline updated: {args.baseline} (provenance stamped for {stamped})")

    if args.strict and (any_flagged or any_precision_flagged or any_agg_flagged):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
