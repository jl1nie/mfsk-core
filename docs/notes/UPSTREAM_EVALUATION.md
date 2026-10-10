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

## Accuracy, 2026-10-10 (every task)

Ryzen 9 9900X. `main` at `787e0cbd`, all nine protocols re-swept on 2026-10-10 after the WSJT-X v3.3.0-beta1 ports
(#642-#646: Q65 `jz` / caller list, JTTY packing / SNR / time, packer fixes); every 50 %-crossing, every unexpected-decode
count and every paired count below is identical to the sweeps of 2026-10-09 (FT8, FT4, FST4; `2cd38b8e`, after the OSD
rewrite, #613), 2026-10-04 (WSPR, JT65, JT9, Q65; `dbdf244c`) and 2026-10-01 (JTTY; `2e56279b`), so the tables stand.
Upstream: WSJT-X `v3.2.0-rc1`
(`scripts/build_jt9_upstream.sh`; `rjtty` from `gen_jtty_sweep_wavs.sh`).

| task | upstream | groups | trials | both | upstream only | crate only | groups behind | groups ahead | crossing, crate − upstream | unexpected decodes, upstream / crate |
|---|---|---|---|---|---|---|---|---|---|---|
| `ft8/t1` | `jt9 -8 -d 3` | 4 | 1 040 | 576 | 19 | 16 | 0 | 0 | +0.33 … −0.24 dB | 0 / 0 |
| `ft4/t1` | `jt9 -5 -d 3` | 4 | 1 040 | 528 | 21 | 27 | 0 | 0 | +0.17 … −0.50 dB | 0 / 0 |
| `fst4/t1` | `jt9 -7 -d 3` | 20 | 3 360 | 1 782 | 69 | 99 | 0 | 1 | +0.50 … −0.79 dB | 7 / 4 |
| `q65/t1` | `jt9 -3 -d 1 -f 1500 -F 20` | 20 | 2 640 | 1 021 | 8 | 147 | 0 | 9 | +0.33 … −2.31 dB | 0 / not counted |
| `jt9/t1` | `jt9 -9 -d 3` | 1 | 300 | 240 | 6 | 3 | 0 | 0 | +0.09 dB | 0 / not counted |
| `jtty/t1` | `rjtty 4.6 0 384 1500 50` | 2 | 360 | 160 | 0 | 0 | 0 | 0 | 0.00 dB | 0 / 0 |
| `wspr/t1` | `wsprd` | 1 | 260 | 217 | 1 | 0 | 0 | 0 | +0.04 dB | 1 / not counted |
| `fst4w/t1` | `jt9 -W -d 3 -f 1500 -F 100` (`v3.3.0-beta1`) | 8 | 1 760 | 927 | 20 | 16 | 0 | 0 | +0.17 … −0.17 dB | 0 / 0 |
| `jt65/t1` | `jt9 -6 -d 3` | 1 | 300 | 280 | 15 | 0 | **1** | 0 | upstream never falls below 50 % on this grid | 9 / not counted |

Groups ahead: FST4-15 CCIR moderate; Q65 A-15 plain, and B-, C-, D- and E-60
both plain and with the CQ hint. JT65's gap is known and the protocol is
legacy; it is recorded, not chased (15 upstream-only against 0, as on
2026-10-01).

The 2026-10-09 run followed #613 (one OSD, line for line with `osd174_91.f90` /
`osd240_101.f90`). Against the 2026-10-04 / 2026-10-01 rows it moved FT8 by one
decode in either direction, FT4 by two fewer upstream-only and three fewer
crate-only, and FST4 by four more upstream-only and one more crate-only; no
group changed its verdict, and every 50 % crossing of the crate's own sweep is
within 0.2 dB of its baseline (`sweep-baseline.json`).

FT8 moved between the 2026-10-04 run and the one before it: upstream-only 5 → 20 and crate-only 18 → 15 in
the pooled counts, no group significant (the largest is 8 against 4, p = 0.39),
and the crossing is level in every channel. The 0.12 task asked for the CQ hint
as `ApHint::with_call1("CQ")`; the 0.13 task takes it from `Depth` and `ap`
as `jt9` does, and the old hint on the new API reproduces the 0.12 crossings. WSPR
is now 217 of 260 with one upstream-only and none crate-only (it was 5 and 4):
after the `wsprd` port of this release (`WSPR_UPSTREAM.md`).

## Time, 2026-10-09 (WSPR, JT65, JT9, Q65: 2026-10-04; JTTY: 2026-10-01)

Crate's time as a fraction of upstream's on the same files.

| task | files a kind | noise | crossing |
|---|---|---|---|
| `ft8/t1` | 20 | 0.20 (125 / 638 ms) | 0.21 (134 / 636 ms) |
| `ft4/t1` | 20 | 0.15 (2.3 / 15.8 ms) | 0.30 (6.9 / 22.8 ms) |
| `fst4/t1` | 20 | 0.19 (47 / 241 ms) | 0.22 (61 / 278 ms) |
| `q65/t1` | 20 | 0.29 (60 / 211 ms) | 0.26 (46 / 174 ms) |
| `jt9/t1` | 10 | 0.19 (15.1 / 77.9 ms) | 0.19 (17.9 / 96.2 ms) |
| `jtty/t1` | 20 | 0.11 (14 / 127 ms) | 0.14 (19 / 132 ms) |
| `wspr/t1` | 10 | 0.24 (19 / 79 ms) | 1.22 (208 / 170 ms) |
| `fst4w/t1` | 8 | 0.28 (29 / 103 ms) | 0.30 (53 / 176 ms) |
| `jt65/t1` | 10 | 0.25 (193 / 784 ms) | 0.06 (53 / 855 ms) |

Fewer than ten files a kind is too few: `wspr/t1` read anywhere from 0.60 to
1.60 on two. Every task now times at least ten. WSPR's crossing column is
the one cell where the crate is slower than upstream (1.22 against 1.0 on
2026-10-01); ten files, and this task has read 0.60 to 1.60 on two, so it is
recorded and not read as a regression. The FT8 ratio
fell from 0.43/0.49 to 0.25/0.26 with the move to `Decoder<P>`; the cause was
not isolated. It fell again, to 0.20/0.21 for FT8 and from 0.41/0.53 to
0.19/0.22 for FST4, after #613. The OSD rewrite accounts for part of that: a
single-thread A/B of `main` before and after it gave `qso3_busy` at `Depth::Deep`
570 → 504 ms and the FST4-60 golden 951 → 597 ms, and one OSD call on FT8's
LLRs 57 → 24 µs (upstream's own Fortran: 198 µs). How much of the rest came from
the other commits since 2026-10-01 was not isolated.

FST4W (2026-10-11, #649, #670) first read 1.77 at the crossing cell (311 / 175 ms, same machine, same files). Timing the
stages put 99 % of the decode in `fastosd240_74`; the per-pattern work (a 32-row partial syndrome rebuilt from 66 bytes,
and `nextpat74` over a 0/1 vector) became a `u32` syndrome xor over the pattern's at most four set bits and a position
list (311 → 127 ms); the candidate codeword of a passing pattern, which was 6.1 M of 17.6 M patterns on 20 files and the
rest of the time, became the codeword of `m0` xor four packed rows, its distance summed over the differing bits and
abandoned at `dmin` (127 → 53 ms). Bit-identical: the 1 208 upstream FEC records and all 1 760 sweep trials are unchanged. The only task whose upstream
is `v3.3.0-beta1`, not `rc1`: rc1's Keff 50 matched blank known-call entries. Eight files a kind here.

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
