#!/usr/bin/env python3
"""Print narrow SNR re-run windows for FST4/FT8/FT4, derived from
docs/notes/sweep-baseline.json's recorded 50%-crossing points.

Applies the same principle docs/notes/FST4_BENCHMARK.md section 7
("Re-verifying a fix — don't re-run the whole grid") already documents
for manual use, but automated from the baseline instead of a human
remembering the last-measured crossing: once you know roughly where a
mode/channel's threshold sits, sweeping the already-known 0%/100%
plateaus on either side buys nothing. FST4-300 in particular pays for
this every SNR point — it's the slowest sub-mode by far — so this is
where the win is biggest; FT8/FT4 are already fast, this just keeps
their release-sweep time in line as their corpora grow too.

Unlike the manual version, this never *drops* a channel — a fix that
only breaks fading and not AWGN would go unnoticed if we did, and FST4
(unlike Q65) has no separate fading-only regression test to catch that
independently (see the two protocols' own doc comments on the point).
It only narrows *SNR range*, and only for modes present in the
baseline — anything absent (new sub-mode, first run) gets no override
here, so the caller falls back to that mode's full generated grid.

Emits one TSV line per group:
    fst4<TAB><mode><TAB><snr_min><TAB><snr_max>
    ft8<TAB><TAB><snr_min><TAB><snr_max>
    ft4<TAB><TAB><snr_min><TAB><snr_max>

Consumed by scripts/run-sensitivity-sweeps.sh. A protocol/mode with no
line printed means "no baseline yet — run the full grid".
"""
import json
import sys
from collections import defaultdict
from pathlib import Path

BASELINE = Path(__file__).resolve().parent.parent / "docs" / "notes" / "sweep-baseline.json"
FST4_MODES = ["15", "30", "60", "120", "300"]


def main():
    margin = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    if not BASELINE.exists():
        return 0
    data = json.loads(BASELINE.read_text())

    fst4_by_mode = defaultdict(list)
    ft8_crossings = []
    ft4_crossings = []
    for label, crossing in data.items():
        # `_`-prefixed keys are the file's `_meta` provenance block, not
        # a group (see sweep-regression-check.py). None of the branches
        # below would match it anyway, but relying on that is one
        # renamed protocol away from a TypeError on a dict.
        if label.startswith("_"):
            continue
        parts = label.split("/")
        if parts[0] == "fst4" and len(parts) == 3:
            fst4_by_mode[parts[1]].append(crossing)
        elif parts[0] == "ft8" and len(parts) == 2:
            ft8_crossings.append(crossing)
        elif parts[0] == "ft4" and len(parts) == 2:
            ft4_crossings.append(crossing)

    for mode in FST4_MODES:
        crossings = fst4_by_mode.get(mode)
        if not crossings:
            continue
        lo, hi = min(crossings) - margin, max(crossings) + margin
        print(f"fst4\t{mode}\t{lo:.0f}\t{hi:.0f}")

    for proto, crossings in (("ft8", ft8_crossings), ("ft4", ft4_crossings)):
        if not crossings:
            continue
        lo, hi = min(crossings) - margin, max(crossings) + margin
        print(f"{proto}\t\t{lo:.0f}\t{hi:.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
