#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Summarise how far a machine's `decode_snapshot` output is from the fixtures (#579).

    MFSK_SNAPSHOT_REPORT=1 MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core \
        --features full,internal-testing --release --test decode_snapshot \
        -- --nocapture 2> snap.txt
    python3 scripts/snapshot-diff-stats.py snap.txt [more.txt ...]

Reads the `SNAPDIFF <case>` blocks the test prints (any other lines are ignored, so
the raw cargo output works, and so do the files under docs/notes/snapshot_platform/).
Per case: whether the message set and order match, and the largest absolute and
relative difference of each float column (as f32 from the fixture's hex bits) and
of `hard_errors`. Columns: 1 freq_hz, 2 dt_sec, 3 snr_db, 4 sync_score (frame
families; the other families have columns 1-3 only), then hard_errors and pass.
"""
import collections
import pathlib
import re
import struct
import sys

FIXTURES = pathlib.Path(__file__).resolve().parent.parent / "mfsk-core/tests/fixtures/decode_snapshot"
NAMES = {1: "freq_hz", 2: "dt_sec", 3: "snr_db", 4: "sync_score"}


def f32(h):
    return struct.unpack(">f", bytes.fromhex(h))[0]


def blocks(path):
    lines = pathlib.Path(path).read_text().split("\n")
    out, i = {}, 0
    while i < len(lines):
        m = re.match(r"SNAPDIFF (\S+)$", lines[i])
        if m and i + 1 < len(lines) and lines[i + 1] == "--- fixture":
            j, want, got = i + 2, [], []
            while lines[j] != "--- now":
                want.append(lines[j])
                j += 1
            j += 1
            while lines[j] != "SNAPEND":
                got.append(lines[j])
                j += 1
            out[m.group(1)] = (want, got)  # a case printed twice (request_shapes, via_decoder) is the same text
            i = j
        else:
            i += 1
    return out


def main(paths):
    for path in paths:
        seen = blocks(path)
        total = len(list(FIXTURES.glob("*.txt")))
        print(f"== {path}: {len(seen)} of {total} fixtures differ")
        worst = collections.defaultdict(lambda: [0.0, 0.0])
        for case, (want, got) in sorted(seen.items()):
            if [r.split("\t")[0] for r in want] != [r.split("\t")[0] for r in got]:
                print(f"  {case:26s} MESSAGES DIFFER ({len(want)} rows / {len(got)} rows)")
                continue
            cols, he, pass_diff = {}, 0, False
            for a, b in zip(want, got):
                x, y = a.split("\t"), b.split("\t")
                for c in range(1, 5 if len(x) == 7 else 4):
                    if x[c] != y[c]:
                        d = abs(f32(x[c]) - f32(y[c]))
                        r = d / max(abs(f32(x[c])), 1e-30)
                        cols[c] = (max(cols.get(c, (0, 0))[0], d), max(cols.get(c, (0, 0))[1], r))
                if len(x) == 7:
                    he = max(he, abs(int(x[5]) - int(y[5])))
                    pass_diff |= x[6] != y[6]
            parts = [f"{NAMES[c]} abs {v[0]:.3g} rel {v[1]:.2g}" for c, v in sorted(cols.items())]
            if he:
                parts.append(f"hard_errors +-{he}")
            if pass_diff:
                parts.append("PASS DIFFERS")
            print(f"  {case:26s} {'; '.join(parts)}")
            for c, v in cols.items():
                worst[NAMES[c]][0] = max(worst[NAMES[c]][0], v[0])
                worst[NAMES[c]][1] = max(worst[NAMES[c]][1], v[1])
            worst["hard_errors"][0] = max(worst["hard_errors"][0], he)
        print("  worst over all cases: " + "; ".join(f"{k} abs {v[0]:.3g} rel {v[1]:.2g}" for k, v in sorted(worst.items())))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
