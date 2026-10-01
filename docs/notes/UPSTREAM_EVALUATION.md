# Evaluation sheet: this crate against WSJT-X, trial by trial

This is the working record behind the headline table in
[`BENCHMARKS.md`](BENCHMARKS.md) ("Against WSJT-X on the same task"). It is
for checking and for finding regressions, not for reading as a benchmark:
a reader wants the verdict and the speed, which `BENCHMARKS.md` gives, and
this sheet holds the counts and tests the verdicts rest on.

How it is made is in [`TIER_C_MANUAL.md`](TIER_C_MANUAL.md) §2 and §4.1.
In short: a *task* (band, what the operator knows, depth) is defined once in
`scripts/upstream_tasks.json` with the upstream command line and the crate
request side by side. Upstream's outcome on every file of the tier-C corpus
is committed under [`upstream/`](upstream/). A crate sweep is then paired
with it file by file.

## How to read it

- **The verdict is the paired test.** For each group (channel, sub-mode),
  the trials one side decoded and the other did not are counted, and an
  exact McNemar test says whether the difference is more than chance
  (p < 0.05). "Behind" and "ahead" below mean exactly that.
- **The 50 % crossing is a summary, not the test.** It is interpolated
  between SNR cells and carries the corpus draw's error (up to ~1.1 dB on a
  fading channel, `TIER_C_MANUAL.md` §4.1.1), common to both decoders. It is
  listed as a range, for orientation.
- **Unexpected decodes** are messages other than the one injected. Only
  FT8, FT4, FST4 and JTTY count them on the crate side; elsewhere the crate
  column is "not counted".
- **Time** is the mean per file, one file at a time, single-threaded, on the
  same files: the lowest-SNR cell ("noise") and the cell nearest upstream's
  crossing. Upstream's time is `jt9`'s own `timer.out` total where it has
  one, so process start is not counted.

Regenerate: `scripts/run-sensitivity-sweeps.sh <protocol>` prints this for
every task of the protocols it sweeps; `scripts/upstream-baseline.py run
<task> <dir>` does one task.

## Accuracy, 2026-10-01

Ryzen 9 9900X. `main` at `2e56279b` (FT8, FT4, FST4, Q65, JTTY) and
`88bb1bf0` (WSPR, JT9, JT65). Upstream: WSJT-X `v3.2.0-rc1`
(`scripts/build_jt9_upstream.sh`; `rjtty` from `gen_jtty_sweep_wavs.sh`).

| task | upstream | groups | trials | both | upstream only | crate only | groups behind | groups ahead | crossing, crate − upstream | unexpected decodes, upstream / crate |
|---|---|---|---|---|---|---|---|---|---|---|
| `ft8/t1` | `jt9 -8 -d 3` | 4 | 1 040 | 590 | 5 | 18 | 0 | 0 | −0.10 … −0.23 dB | 0 / 1 |
| `ft4/t1` | `jt9 -5 -d 3` | 4 | 1 040 | 526 | 23 | 30 | 0 | 0 | +0.27 … −0.64 dB | 0 / 0 |
| `fst4/t1` | `jt9 -7 -d 3` | 20 | 3 360 | 1 786 | 65 | 98 | 0 | 1 | +0.50 … −0.79 dB | 7 / 5 |
| `q65/t1` | `jt9 -3 -d 1 -f 1500 -F 20` | 20 | 2 640 | 1 021 | 8 | 147 | 0 | 9 | +0.33 … −2.31 dB | 0 / not counted |
| `jt9/t1` | `jt9 -9 -d 3` | 1 | 300 | 240 | 6 | 3 | 0 | 0 | +0.09 dB | 0 / not counted |
| `jtty/t1` | `rjtty 4.6 0 384 1500 50` | 2 | 360 | 160 | 0 | 0 | 0 | 0 | 0.00 dB | 0 / 0 |
| `wspr/t1` | `wsprd` | 1 | 260 | 213 | 5 | 4 | 0 | 0 | +0.11 dB | 1 / not counted |
| `jt65/t1` | `jt9 -6 -d 3` | 1 | 300 | 280 | 15 | 0 | **1** | 0 | upstream never falls below 50 % on this grid | 9 / not counted |

Groups ahead: FST4-15 CCIR moderate; Q65 A-15 plain, and B-, C-, D- and E-60
both plain and with the CQ hint. JT65's gap is known and the protocol is
legacy; it is recorded, not chased.

## Time, 2026-10-01

Crate's time as a fraction of upstream's on the same files.

| task | files a kind | noise | crossing |
|---|---|---|---|
| `ft8/t1` | 20 | 0.43 (272 / 639 ms) | 0.49 (310 / 635 ms) |
| `ft4/t1` | 20 | 0.16 (2.4 / 15.4 ms) | 0.35 (7.7 / 22.2 ms) |
| `fst4/t1` | 20 | 0.41 (100 / 242 ms) | 0.53 (150 / 280 ms) |
| `q65/t1` | 20 | 0.28 (58 / 205 ms) | 0.26 (43 / 167 ms) |
| `jt9/t1` | 10 | 0.17 (12.5 / 73.2 ms) | 0.18 (16.2 / 88.3 ms) |
| `jtty/t1` | 20 | 0.11 (14 / 127 ms) | 0.14 (19 / 132 ms) |
| `wspr/t1` | 10 | 0.67 (51 / 77 ms) | 1.0 (166 / 169 ms) |
| `jt65/t1` | 10 | 0.25 (182 / 741 ms) | 0.06 (48 / 804 ms) |

Fewer than ten files a kind is too few: `wspr/t1` read anywhere from 0.60 to
1.60 on two. Every task now times at least ten.

## What moved these numbers

The first run of this comparison (2026-10-01, morning) found FT4 and FST4
behind on accuracy and 4–6× slower, and Q65 short of twice upstream's speed.
In every case the cause was a place the crate had drifted from upstream.

- **FT4 (#553).** The SIC path dropped the blind CQ AP rung, relaxed
  `sync_min` in later rounds and ran every round. Was 4 of 4 groups behind
  and 6.0× / 4.1× upstream's time.
- **FST4 (#554).** The AP rung capped hard errors at FT8's 36, which
  `fst4_decode.f90` does not do. Every frame traced that upstream decoded and
  the crate did not came from upstream's CQ AP pass, with 40–58 hard errors
  (`fst4_decodes.dat`). The candidate search was the generic Costas search,
  letting 50 candidates through on every file where `get_candidates_fst4`
  yields 2–6; ported as `engine::fst4_coarse`. Was 15 of 20 groups behind
  and 4.1× / 3.9×; unexpected decodes fell from 19 to 5.
- **Q65 (#555, #556).** A hinted scan tried only the hint, so the task ran
  two scans; upstream's `ipass` loop tries no-AP first in the same pass.
  Candidates inside an already-decoded signal's band were not skipped
  (`q65_decode.f90:375-377`): 50 BP calls and ~200 ms on a strong file. BP
  kernels specialised to `M = 64`; `maxiters` upstream's 40; sync smoothing
  only where read. Was 0.77× / 0.69×.
- **WSPR (#557).** At about `wsprd`'s speed, which is where a faithful port
  lands: 70–74 % of both is Fano on the same candidates and cap, and the
  loop is branch-bound. Structure-of-arrays nodes were 17 % slower and
  unchecked indexing gained nothing. The look found the pooled Fano scratch
  never resetting the root's `encstate` (`fano.c:127` does); fixed, with no
  change to the crossing.

The FT8 per-channel detail as first measured is in the git history of
`BENCHMARKS.md` (2026-10-01).
