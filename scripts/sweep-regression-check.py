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

    <dim1>,<dim2>,...,snr_db,trial,pass

Any number of leading "dimension" columns (mode, channel, submode, ap
mode, ...) is allowed — everything before the fixed trailing
`snr_db,trial,pass` triple is treated as the group key. One row per
trial; `pass` is 0/1.

Baseline file format (docs/notes/sweep-baseline.json): a flat
`{"<protocol>[/<dim>...]": crossing_db}` map, plus one `_meta` key
carrying per-protocol provenance — when each protocol's numbers were
measured, at which commit, on which machine, over how many trials.
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
dB, for a caller that wants a hard gate anyway.
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
    """Short SHA the run was measured at, `-dirty` when the tree had
    uncommitted changes — a baseline measured on an uncommitted decoder
    edit is not reproducible and should say so."""
    def run(*argv):
        return subprocess.run(
            argv, cwd=REPO_ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()
    try:
        sha = run("git", "rev-parse", "--short", "HEAD")
    except (OSError, subprocess.CalledProcessError):
        return None
    try:
        if run("git", "status", "--porcelain"):
            sha += "-dirty"
    except (OSError, subprocess.CalledProcessError):
        pass
    return sha


def protocol_name_from_path(path: Path) -> str:
    # wspr.csv -> wspr, fst4_awgn_run3.csv -> fst4 (first underscore-free
    # token isn't assumed; caller can pass an explicit name via NAME=path).
    return path.stem


def load_cells(path: Path):
    """Returns {group_key_tuple: {snr_db: [n_hits, n_trials]}}."""
    cells = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    with path.open(newline="") as f:
        reader = csv_mod.reader(f)
        header = next(reader, None)
        if header is None:
            return cells
        if header[-3:] != ["snr_db", "trial", "pass"]:
            raise ValueError(
                f"{path}: expected header ending in snr_db,trial,pass, got {header}"
            )
        ndims = len(header) - 3
        for row in reader:
            if not row:
                continue
            key = tuple(row[:ndims])
            snr = int(row[ndims])
            passed = row[ndims + 2] == "1"
            cell = cells[key][snr]
            cell[1] += 1
            if passed:
                cell[0] += 1
    return cells


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


def group_label(proto: str, key: tuple) -> str:
    return "/".join((proto,) + key) if key else proto


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csvs", nargs="+", help="CSV files, optionally NAME=path.csv to override the protocol name")
    ap.add_argument("--baseline", type=Path, default=DEFAULT_BASELINE, help=f"baseline JSON (default: {DEFAULT_BASELINE})")
    ap.add_argument("--update-baseline", action="store_true", help="write this run's crossings into the baseline file after reporting")
    ap.add_argument("--threshold", type=float, default=0.5, help="dB move to flag (default 0.5, matches CLAUDE.md's release-tag guidance)")
    ap.add_argument("--strict", action="store_true", help="exit 1 if any group moved beyond --threshold (default: always exit 0, this is advisory)")
    ap.add_argument("--machine", default=None, help="override the recorded machine string (default: /proc/cpuinfo model name)")
    args = ap.parse_args()

    raw = {}
    if args.baseline.exists():
        raw = json.loads(args.baseline.read_text())
    meta = raw.get(META_KEY, {})
    baseline = {k: v for k, v in raw.items() if not k.startswith("_")}

    current = {}
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
        cells = load_cells(path)
        for key, snr_map in sorted(cells.items()):
            label = group_label(name, key)
            crossing, note = crossing_snr(snr_map)
            trials_total = sum(t for _, t in snr_map.values())
            rows.append((label, crossing, note, trials_total))
            if crossing is not None:
                current[label] = crossing

    if not rows:
        print("no cells found in any input CSV — nothing to report.")
        return 0

    # Print the baseline's own provenance before the diff. Without it a
    # "+0.00 dB" line says the number matches but not what it matches —
    # which is exactly the question that sent a reader to `git log` on
    # the JSON, and from there to guessing.
    per_proto = meta.get("protocols", {})
    seen_protos = sorted({protocol_of(r[0]) for r in rows})
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

    width = max(len(r[0]) for r in rows)
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

    print()
    if any_flagged:
        print(f"⚠ one or more groups moved >= {args.threshold} dB from baseline — see CLAUDE.md's")
        print("  release-tag guidance ('a move worse than ~0.5 dB is worth explaining').")
    if any_new:
        print("  groups marked NEW have no baseline entry yet — add with --update-baseline")
        print("  once you've confirmed the number is correct, not just new.")
    if not any_flagged and not any_new:
        print("no groups moved beyond threshold.")

    if args.update_baseline:
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
        meta = {
            **meta,
            "schema": 1,
            "protocols": dict(sorted(protocols.items())),
        }

        out = {META_KEY: meta}
        out.update(sorted(baseline.items()))
        args.baseline.parent.mkdir(parents=True, exist_ok=True)
        args.baseline.write_text(json.dumps(out, indent=2) + "\n")
        stamped = ", ".join(sorted(trials_by_proto)) or "nothing"
        print(f"\nbaseline updated: {args.baseline} (provenance stamped for {stamped})")

    if args.strict and any_flagged:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
