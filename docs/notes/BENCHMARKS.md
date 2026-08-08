# Benchmarks vs. WSJT-X

Current-state results only — no release history here. For "how did we
get to these numbers" narrative, see `CHANGELOG.md` (per-release
detail) or `docs/notes/ROADMAP.md` (open follow-ups). For "how do I
reproduce this sweep myself", see the protocol-specific
`*_BENCHMARK.md` docs linked per section below.

Two kinds of numbers appear per protocol:

- **Golden-WAV recall** — decode a real WSJT-X-distributed recording,
  compare messages/frequency/timing (and for MSK144, SNR) against a
  known-correct reference decode.
- **AWGN sensitivity sweep** — a `*sim`-generated corpus swept across
  SNR points, giving a 50%-recall crossing to compare against WSJT-X's
  published or measured threshold.

## Summary

| Protocol | Golden-WAV recall | AWGN gap vs. WSJT-X | Status |
|----------|-------------------|----------------------|--------|
| FT8      | 8/8 host full-parity (WSJT-X), 18/18 (JTDX) | AWGN ≈ −21.6 dB (WSJT-X: −20 to −21 dB) | at/above parity |
| FT4      | 6/6 | AWGN ≈ −16.9 dB (WSJT-X: −17.5 dB, ~0.6 dB gap) | at parity |
| FST4     | 1/1 (FST4-60A only) | 0.10-0.60 dB across 5 sub-modes | at parity |
| WSPR     | 8/8 | AWGN 50% ≈ −29.8 dB, matches published sensitivity floor | at parity |
| JT9      | 7/7 | AWGN 50% ≈ −26.3 dB, no measurable gap vs. `jt9 -9` | at parity |
| JT65     | none available | ~0 dB (2026-08-08, #169: faithful `ftrsdap` port + FFT bin-alignment fix — see JT65 section for caveats on the WSJT-X comparison) | gap closed |
| Q65      | 2 real EME recordings | 0.2-1.4 dB vs. analytical target across 10 sub-modes; matches/beats WSJT-X's own decode with CQ-AP hint | at/above parity |
| MSK144   | 3/3 (incl. exact SNR match) | AWGN 50% ≈ −5.2 to −5.8 dB, 25/28 cells exact match vs. a real `jt9` build | at parity |

All AWGN 50%-crossing figures below are linear-interpolated between the
nearest swept SNR points, in each `*sim` generator's 2500 Hz
reference-bandwidth convention (matches WSJT-X's own published
numbers directly, no unit conversion needed).

## Decode speed (single golden-WAV, host)

Wall-clock time for one full decode call against each protocol's real
WSJT-X-recorded golden WAV (see the recall sections below for which
file and which decode function). Measured 2026-07-20, single run per
row — treat as an order-of-magnitude reference, not an averaged
benchmark. Re-verified in full 2026-07-26 (each row re-run against its
own protocol's real recall/timing harness) after the FT8 fine-sync
refinement rewrite below (`fine_refine_3stage`'s reference-tweak
port) and a cross-protocol Costas-reference caching hoist in
`engine::sync::fine_sync_power_per_block`: FT8 moved again (see its own
row); every other row was confirmed unchanged within run-to-run noise
(≤~15%) **except FT4**, which turned out to already be stale for an
unrelated reason — see its row below.

**Re-verified again 2026-07-29** (median of 3-5 runs per row, same
harnesses/methodology as above) after a cluster of `perf(...)` commits
landed 2026-07-26 through 2026-07-28, none reflected in the previous
pass: `17e7583`/`7adaed8` (issue #182 fine-sync + FT4 refine-radius
fixes, already folded into the FT4/FT8 notes below), `ac67d29`/
`4801722`/`b93b16c` (`fill_bmet_for_nsym` Vec-alloc removal + lazy
nsym LLR staircase + incremental ladder-rung summation in the shared
`engine::llr`/generic-pipeline code FT4 and FST4 both route through),
`43954e9` (issue #197, FST4 `SYNC_Q_MIN` tightened 10→16 to match
WSJT-X's own pre-ladder gate), `97b96a1`/`7d5eb5e` (issues #199/#201,
`BpScratch` reuse across `decode_block_multipass`'s and
`sic_inner_passes_with_cache`'s per-candidate calls), `4bbd188` (issue
#200, the same lazy-LLR-staircase treatment for `pipeline_ap`'s AP
sniper path), and `eabbf03` (issue #211/#208, `downsample_cached`'s
FFT-planner cache — the same fix already documented as a WASM win
below, but it applies to every target including this table's own
Ryzen9 host row, not just WASM). FT8, FT4, and FST4-60A all dropped
substantially; Q65/WSPR/JT9/MSK144 don't call any of the touched code
(`fill_bmet_for_nsym`, `process_candidate_ap`, `decode_block_multipass`'s
`BpScratch` path) and were left at their existing figures rather than
re-run. Recall byte-identical at every re-measured row (FT8 ship-config
7/8 golden + 7 phantom + 14 total; FT8 full-parity 8/8 golden; FT4 6/6
golden + 14/14 total; FST4-60A 2/2 golden messages) — these are pure
throughput wins, not tightened/loosened decode behavior.

**Re-verified a third time 2026-08-08** after a second `perf(...)`
commit cluster landed 2026-08-05/06 — this time reaching WSPR and Q65
too, both previously untouched by any perf-review pass. Full
methodology and per-row numbers in the "2026-08-08 re-measurement"
block below the table; headline: WSPR ~2.4× faster (never had a
dedicated pass before), Q65's fading-metric rows ~1.8-2× faster, FT8/
FT4/FST4-60A unchanged within this box's own run-to-run noise, nothing
regressed.

**Compute environment**: AMD Ryzen 9 9900X (12C/24T), 32 GB RAM,
Ubuntu 24.04.2 LTS under WSL2 (kernel 6.6.87.2-microsoft-standard-WSL2),
rustc 1.97.1, `cargo test --release --features full` (includes the
`parallel` feature — some decode paths use `rayon` internally, so this
is wall time on a many-core host, not a single-thread figure).

| Protocol | Golden WAV | Slot length | Decode time |
|---|---|---:|---:|
| Q65-60D | 201212_1838.wav (10 GHz EME, fading metric) | 60 s | 0.05 s (was 0.08 s — see 2026-08-08 note below) |
| Q65-60B | 1296 MHz troposcatter ×3 slots (multi-period averaging) | 60 s | 0.06 s (was 0.12 s — see 2026-08-08 note below) |
| FT8 | qso3_busy.wav (16-signal busy band) | 15 s | 0.07 s (was 0.06 s — see 2026-08-08 note below) |
| Q65-120D | 210117_0920.wav (rainscatter, fading metric) | 120 s | 0.07 s (was 0.12 s — see 2026-08-08 note below) |
| FST4-60A | 210115_0058.wav | 60 s | 0.08 s (unchanged — see 2026-08-08 note below) |
| FT4 | 000000_000002.wav | 7.5 s | 0.10 s (unchanged — see 2026-08-08 note below) |
| Q65-120E | 6 m ionoscatter ×2 files (fading metric) | 120 s | 0.14 s (was 0.26 s — see 2026-08-08 note below) |
| Q65-300A | 201210_0505.wav (optical scatter, fading metric) | 293.8 s | 0.19 s (was 0.34 s — see 2026-08-08 note below) |
| WSPR | 150426_0918.wav | 120 s | 0.37 s (was 0.93 s — see 2026-08-08 note below) |
| JT9 | 130418_1742.wav | 60 s | 0.30 s (was 0.37 s / 0.33 s — see the second 2026-08-08 note below) |
| Q65-30A | 6 m ionoscatter ×4 slots (multi-period averaging) | 4×30 s | 0.42 s (was 0.56 s — see 2026-08-08 note below) |
| Q65-60A | 6 m EME (plain BP + AP) | 60 s | 0.65 s (was 0.69 s) |
| MSK144 | 181211_120800.wav | 30 s | 0.84 s |
| MSK144 | 181211_120500.wav | 15 s | 0.88 s |

**2026-07-29 re-measurement** (median of 3-5 runs, same box/build as
the rest of this table; see the intro paragraph above for which
`perf(...)` commits this captures):

- **FST4-60A**: `decode_frame` wall-clock **75-82 ms** (median ~76 ms),
  down from 0.27 s (~3.5×) — dominated by the `SYNC_Q_MIN` 10→16
  tightening (issue #197) plus the lazy nsym-LLR-staircase + incremental
  ladder-rung changes to the shared `engine::llr`/generic-pipeline code
  FST4 routes through. Still 50 coarse candidates, 2/2 golden decodes
  (`CQ N5TM EL29`, `CQ K9KFR EN71`), measured via
  `tests/fst4_sweep.rs::fst4_60_diag_candidate_cost_split`'s own
  `DecodeRequest::<Fst4s60>` production-path timer.
- **FT4**: `000000_000002.wav` flat-SIC wall-clock **100-105 ms**
  (median ~104 ms), down from ~0.28 s (~2.7×) — same shared-pipeline
  LLR changes as FST4, plus `downsample_cached`'s FFT-planner cache
  (issue #211). 14/14 total decodes (6/6 golden), measured via
  `tests/ft4_sweep.rs::ft4_diag_candidate_cost_split`'s own
  `DecodeRequest::<Ft4>::new(...).sic_rounds(3)` production-path timer.
- **FT8 ship-config** (`decode_block`, `DecodeDepth::EMBEDDED`,
  `max_cand=15` — the exact call `ft8_qso3_apoff_recall.rs` uses):
  **58.8-60.5 ms** (median ~60 ms), down from 0.10 s (~1.7×) — the
  `BpScratch`-reuse fixes (issues #199/#201) plus the FFT-planner
  cache. 14 total decodes (7/8 golden + 7 phantom), unchanged.
- **FT8 full-parity** (`DecodeDepth::FULL`, `sync_min=0.8`,
  `max_cand=60`, `tests/ft8_qso3_full_parity_recall.rs`): **118-127 ms**
  (median ~118 ms), down from ~165-175 ms. Still 8/8 golden hit
  (including `K1BZM DK8NE`); phantom count 11 this pass vs. not
  separately tracked before — not a regression signal on its own since
  this row has never gated on phantom count, only golden-hit floor.
- Q65/WSPR/JT9/MSK144 rows are unchanged from 2026-07-20/26 — none of
  the perf commits since touch code any of those four protocols call
  (confirmed by `grep` for `fill_bmet_for_nsym`/`process_candidate_ap`
  under `src/q65`, `src/wspr`, `src/jt9`, `src/msk144`: no hits).
- **AWGN sweep sensitivity double-checked too, not just golden-WAV
  recall.** The lazy-LLR-staircase refactors (`ac67d29`/`4801722`/
  `b93b16c`/`4bbd188`) and FST4's `SYNC_Q_MIN` gate tightening
  (`43954e9`) are the two changes with any plausible path to shifting
  a decode outcome, not just its speed (the `BpScratch`-reuse and
  FFT-planner-cache fixes only change allocation/caching, not any
  math). Re-ran `tests/ft4_sweep.rs::ft4_snr_sweep` (AWGN channel only,
  full existing SNR grid, `parallel` rayon) and
  `tests/fst4_sweep.rs::fst4_snr_sweep` (FST4-60 only, all 4 channels)
  against current `main`: FT4 AWGN 50% crossing ≈ **−16.89 dB**
  (documented: ≈−16.9 dB) and FST4-60 AWGN ≈ **−27.56 dB** (matches the
  figure already recorded in the FST4 section's own SYNC_Q_MIN note
  below, reproduced independently here after the further perf changes
  landed) — both within noise of, in FST4's case exactly matching, the
  documented figures. No sweep-table update needed.

**2026-08-08 re-measurement** (median/representative of 3-5 runs per
row, same box as the rest of this table — AMD Ryzen 9 9900X, see the
compute-environment paragraph above; `cargo test --release --features
full,internal-testing`, matching CI's own feature set exactly, not
just `full`) after a second `perf(...)` commit cluster landed
2026-08-05/06, none reflected in the 2026-07-29 pass above:
`2177768`/`c5d8031` (FT8 AP-mask LLR iterator-chain + two per-candidate
allocation removals), `1b078ca` (`make_costas_ref` output cached
across `fine_sync_power_per_block` calls — shared FT8/FT4/Q65 sync
code), `66888b3` (`downsample_cached`'s thread-local FFT-planner reuse
in `symbol_spectra`), `eb859cf` (new `BpPooledFec`/`decode_soft_pooled`
— extends the FT8-only `BpScratch` pooling from issues #199/#201 to
the `SumProduct` kernel host builds default to, plus FST4/FT8's
zsum-seeded OSD retry, neither of which had scratch pooling before),
`da645b4` (Q65 `decode_multi_period_for` redundant narrow-energy-
extraction dedup + iterator-loop cleanup), and `abe2908`/`dbab359`
(FT4 `ft4_sync_search_window` Costas-ref caching + `ft4_coarse`'s
`symbol_spectra_avg` FFT-planner reuse). Separately, five
`perf(wspr)` commits landed the same window — `71fc382` (rayon-
parallelize `decode_scan`'s pass-1/pass-2 candidate loops), `42e3f57`
(iterator-zip the LPF convolution in `subtract_signal_baseband`),
`362b769` (pool `fano_decode`'s `Vec<Node>` scratch across a
candidate's `nblock` sweep), `8e2feb7` (eliminate a dead pre-pass-2
clone), `f7efb13` (FFT-planner reuse in `decimate_to_baseband`/
`build_spectro`) — WSPR's first dedicated perf-review pass; unlike
FT8/FT4/FST4/Q65 (each already swept 2026-07-20/25), nobody had looked
at `decode_scan_subtract`'s own cost breakdown before.

Every row above was re-run; recall re-verified byte-identical
everywhere checked (FT8 ship-config 14 total = 7 golden + 7 phantom,
full-parity 19 total = 8 golden + 11 phantom, JTDX 18/18, AP-on 6/6;
FT4 6/6 golden / 14 total; FST4-60A 2/2 golden; WSPR 19 total incl.
8/8 golden; all 7 Q65 WSJT-X-golden-WAV recall tests; JT9 7/7; MSK144
2/2 incl. exact SNR match) — same "throughput only, not behavior"
conclusion as 2026-07-29, reached the same way (full recall suite
green plus, for the two changes with any plausible path to shifting a
decode outcome, an independent AWGN/CCIR sweep re-run — see below).

- **WSPR — the real win this pass, ~2.4×**: `decode_scan_subtract` on
  `150426_0918.wav` (`wspr_speed_diag`, 5 iterations) dropped
  **~0.88-0.93 s → 367-374 ms (avg 369.7 ms)**. Plausibly dominated by
  the rayon pass-1/pass-2 parallelization, since this was the first
  perf pass WSPR ever got — no prior investigation to have already
  picked off the easy wins the way FT8/FT4/FST4/Q65 had. Re-ran the
  full AWGN sweep (`wspr_sweep.rs::wspr_awgn_snr_sweep`, all 13 SNR
  points × 20 trials) specifically to rule out the parallelization
  perturbing floating-point reduction order: **every percentage
  reproduced exactly** (40.0%@-30dB, 95.0%@-29/-28dB, 100.0%@-27dB and
  up, 0.0% below -31dB) — confirms it's a pure scheduling change, 50%
  crossing still ≈−29.8 dB.
- **Q65 — broad win across the fading-metric rows, ~1.8-2×**:
  `da645b4`'s redundant-extraction dedup (Stage B/Stage C-plain no
  longer independently re-running `averaged_data_energies` on
  identical arguments) is the most direct cause, though the commit's
  own isolated A/B measurement found it roughly noise-level on
  Q65-60B/30A alone — the win visible here is larger than that one
  commit's own attributed delta, consistent with it compounding on top
  of the same-cluster FFT-planner/Costas-ref/BP-scratch changes (all
  shared infrastructure Q65's decode paths also go through). Measured
  against each golden test's own exact `SearchParams` (not the
  existing `q65_speed_diag_coarse_vs_finetiming`/
  `q65_multi_period_speed_diag` diagnostics' params, which turned out
  not to match production config in every case — e.g. the coarse-vs-
  finetiming probe uses `max_candidates=100` for Q65-60D where the real
  golden test uses `8`, a 5× difference that would have produced a
  bogus regression reading): Q65-60B (3-file `MultiPeriodRequest`,
  `tropo_1296_60b_decodes_via_averaging`'s own config — the table row
  above was previously mislabeled "×1 slot"; the golden test always
  used all 3 files in the sample dir) **0.12 s → ~0.064 s**; Q65-30A
  (4-file, `q65_multi_period_speed_diag`) **0.56 s → ~0.42 s**;
  Q65-60D **0.08 s → ~0.05 s**; Q65-120D **0.12 s → ~0.07 s**;
  Q65-120E (both files in the dir, matching the golden test's own
  loop) **0.26 s → ~0.14 s**; Q65-300A **0.34 s → ~0.19 s**; Q65-60A
  (plain + AP-hint, 1 file) **0.69 s → ~0.65 s**, the smallest move
  since it's the only row here that doesn't route through
  `decode_multi_period_for`. Bit-identical recall (same golden
  messages) at every row — cross-checked against the full
  `q65_wsjtx_samples`/`q65_eme_submodes` recall suite, not just the
  timing probes.
- **FT4 and FST4-60A: unchanged within this box's own noise.** FT4
  flat-SIC wall-clock: 98.7-102.8 ms (documented 100-105 ms) — no
  measurable effect from the `ft4_sync_search_window`/`ft4_coarse`
  FFT-planner changes at this call's scale. FST4-60A `decode_frame`:
  78.1-89.0 ms, median ~83 ms (documented 75-82 ms, median ~76 ms) —
  `BpPooledFec` (`eb859cf`) extends scratch pooling to exactly the
  generic-pipeline `decode_soft` call this row exercises (FST4 never
  had it before), so some win was plausible here, but it doesn't show
  cleanly against this box's own ~15% run-to-run noise band either
  way. Re-ran the full FT4 AWGN/CCIR sweep and a scoped FST4-30
  AWGN/CCIR sweep (`MFSK_FST4_SWEEP_MODES=30`, same methodology as the
  2026-07-29 pass) specifically because the Costas-ref caching and
  `BpPooledFec` are the two changes in this cluster with any plausible
  path to shifting a numeric outcome, not just speed: FT4 crossings
  reproduced to within 0.1 dB of the documented table at every channel
  (AWGN ≈−16.89 dB, an exact match to the recorded figure); FST4-30
  crossings reproduced exactly at 2 of 4 channels (CCIR moderate
  −21.4 dB, CCIR poor −21.7 dB) and within 0.1 dB at the other 2 (AWGN
  ≈−23.82 dB vs. documented −23.90 dB, CCIR good ≈−23.33 dB vs.
  documented −23.3 dB) — all within this doc's own established
  20-trial sampling-noise band, no shift attributable to either change.
- **FT8: a measurement-methodology finding, not a regression.**
  Ship-config (`decode_block`, `DecodeDepth::EMBEDDED`, `max_cand=15`)
  measured **65.1-69.9 ms across two isolated back-to-back runs today
  (median ~67 ms)** — nominally above the documented "58.8-60.5 ms".
  Checked with the same git-worktree A/B methodology `da645b4`'s own
  commit message used: built and ran the identical
  `bench_qso3_busy_timing.rs` harness against `cc6aa2f` (the commit
  the 2026-07-29 figures were measured at) in a separate worktree,
  same session, back-to-back with the `HEAD` run — **67.6 ms avg
  (65.4-70.4 ms)**, statistically indistinguishable from today's
  `HEAD` number. The entire gap versus the documented 58.8-60.5 ms is
  session-to-session noise on this box (thermal/background-load state
  varies day to day — FST4-60A's ~10% wobble above is the same
  effect), not anything the 08-05/06 cluster introduced; recalibrating
  the table's FT8 row to today's honest, reproducible reading rather
  than leaving a figure this cluster's own unmodified ancestor commit
  can't reproduce either. Full-parity (`DecodeDepth::FULL`,
  `sync_min=0.8`, `max_cand=60`): 112.6-123.8 ms, avg 115.9 ms
  (documented 118-127 ms) — flat to marginally faster, 19 total
  decodes (8 golden + 11 phantom) unchanged.
- **JT9, MSK144, JT65: not touched by this cluster.** Confirmed via
  `git diff --stat cc6aa2f..HEAD -- src/msk144`: no hits at all. JT9
  and JT65 gained only the additive streaming siblings
  (`decode_scan_streaming`) and 0.8.1's `snr_db` field — no change to
  `decode_scan`'s existing decode math. Recall re-confirmed unchanged
  (JT9 7/7, MSK144 2/2 incl. SNR). Re-measured JT9's own timing anyway
  out of caution for the `snr_db` addition: 340.5-411.4 ms across 5
  runs (documented 0.33-0.34 s) — noisier than before (~20% spread,
  wider than this box's other rows) but the median (~372 ms) sits
  close enough to the old figure that this reads as ordinary machine
  noise rather than a real cost increase from the added SNR
  computation; not chased further given the size of the effect (if
  any) is smaller than this pass's own measurement noise.
- **MSK144 rows not re-measured** — no code in `msk144`'s module path
  changed at all (confirmed above), so there's nothing this pass could
  have moved; left at the existing 0.84 s / 0.88 s figures.

**Second 2026-08-08 re-measurement, later same day** (median of 5
runs, same box/methodology, git-worktree A/B against the exact
pre-session commit to isolate this pass's own effect from ordinary
run-to-run noise): a per-protocol parallelism audit (candidate-loop
`rayon` parallelism added and measured across JT65/Q65/JT9/uvpacket/
MSK144, found to help none of them, reverted everywhere — see
`CHANGELOG.md`'s 0.9.0 "Investigated candidate-loop `rayon`
parallelism…" entry) led to profiling JT65/JT9's `Spectrogram`/
`AudioFft::build` and finding two real, non-parallelism wins:

- **JT65 and JT9's `Spectrogram`/`AudioFft::build`**: the bottom-95%
  trimmed-mean noise-floor step was a full `sort_unstable_by` (O(n log
  n)) where only `select_nth_unstable_by` (O(n) average partition) is
  needed — the sort was itself larger than the FFT loop it was a minor
  adjunct to. No golden-WAV row exists for JT65 to update (none
  available, see the JT65 section), but this fix underlies part of
  JT9's row move below.
- **JT9's `softsym::AudioFft::build`** — a *second*, separate
  653,184-point FFT (distinct from `jt9::search::AudioFft`, used once
  per `decode_scan` call for `downsam9`'s baseband extraction) — moved
  to a real-input transform (`realfft` crate) instead of running a
  full complex-to-complex `rustfft` transform on real-valued audio and
  discarding the Hermitian-redundant upper half. Isolated
  (`jt9::tests::phase_breakdown_diag`, a smaller/cleaner AWGN file,
  default `SearchParams` → 32 candidates): `AudioFft::build` itself
  ~9.4-10.0ms→~5.1ms, ~57% of that file's `decode_scan` total.

  **On this table's own golden-WAV row** (`130418_1742.wav`,
  `max_candidates=200`, a real busy band — several genuine signals
  clear the sync gate and reach the expensive `afc9`/Fano stages, so
  the once-per-scan big FFT is a much smaller share of the total here
  than on the simpler diagnostic file above): three-point git-worktree
  A/B, 5 runs each — pre-session (`2288d1d`) **338.8-369.7 ms (median
  340.4 ms)**, matching the previously-documented "noise" reading;
  after `select_nth_unstable_by` alone (`30e63e1`) **317.1-319.7 ms
  (median 317.7 ms)**, a real ~7% drop, not noise; after `realfft` too
  (`d3eaf15`, current `HEAD`) **300.8-304.8 ms (median 302.0 ms)**, a
  further ~5% drop — **~340 ms → ~302 ms, ~11% total**, smaller than
  the ~25-27% seen on the lower-candidate-count diagnostic file
  because this file's candidate loop (up to 200 candidates on a busy
  band) is proportionally a much larger share of total time than in
  the simpler case. Recall unchanged at every point (7/7 golden).
  AWGN sweep crossing point unchanged (still ≈−26.3 dB, exact
  percentage match at every SNR point — see the JT9 section below).
- **`msk144::spd`'s noise-floor quantile** got the identical
  `sort_by`→`select_nth_unstable_by` fix (same audit), but on an
  array sized `nstep ≈ 833` (15 s slot, `STEP=216`) the absolute time
  is already sub-millisecond either way — not expected, and not
  separately re-measured, to move the 0.84 s / 0.88 s golden-WAV rows
  below; correctness confirmed via the full MSK144 test suite
  (unchanged), not a speed claim.

Also investigated, same pass: candidate-loop `rayon` parallelism for
all five of JT65/Q65/JT9/uvpacket/MSK144 (none had it before), and,
after that measured zero benefit everywhere, a second attempt at
Q65's per-candidate `(Δf,Δt,b90)` grid search specifically. Both
correct where implemented (verified against every real off-air Q65
recording) but **neither shipped** — real-data timing showed no
speedup either time (too few independent work items per `rayon`
region, `≤7` cells for Q65's grid search, dispatch overhead ate the
gain) — see `CHANGELOG.md`'s 0.9.0 entry for the full writeup. No
protocol's recall table below changed as a result.

Notes:

- These are real-recording decode times, not synthetic sweeps — they
  reflect actual candidate density and search cost on real audio, not
  a clean-signal best case.
- **FT8's `qso3_busy.wav` row dropped again, 0.12 s → ~0.10 s**
  (2026-07-25), re-measured via `decode_block(&slot, 100.0, 3000.0,
  1.3, DecodeDepth::EMBEDDED, 15)` (`DecodeDepth::BpVariantsAd` prior
  to the 2026-07-26 redesign below) — the exact ship-config call
  `ft8_qso3_apoff_recall.rs`'s `qso3_apoff_meets_wsjtx_golden_floor`
  uses, matching this row's original 2026-07-25 methodology. Comes
  from `fine_refine_3stage`'s rewrite (tweaks a small 32-sample Costas
  reference waveform instead of shifting the whole 3200-sample `cd0`
  baseband buffer per trial DF/DT, matching WSJT-X's real
  `sync8d.f90`/`ft8b.f90:133-140` algorithm) plus a `engine::sync.rs`
  cross-protocol Costas-reference caching hoist — both trig-heavy-loop
  fixes, not new algorithms. Recall byte-identical (7/8 golden, 7
  phantom, 14 total on `qso3_apoff`; 18/18 on `qso3_jtdx`; 6/6 JTDX
  AP-on multipass extras — see the FT8 section below for that last
  number's own history). See `CHANGELOG.md` for the full investigation
  (issue #182 follow-up).
- **New row's worth noting separately (2026-07-26): full WSJT-X
  parity (8/8, not this row's 7/8) via `DecodeDepth::FULL`,
  `sync_min=0.8`, `max_cand=60` runs `~139-148 ms`** — still
  ~7-8× faster than real `jt9 -8 -d3`'s own ~1.1 s total file decode
  time, just with OSD's extra cost included (and the `depth.osd`-gated
  auto-AP bug fixed — see the FT8 section below). Not merged into this
  table's own row since it's a different config answering a different
  question ("fastest full parity" vs "real ship-config budget"); see
  `tests/ft8_qso3_full_parity_recall.rs`.
- **FT4's `000000_000002.wav` row was stale, then found regressed,
  then fixed (issue #182).** First found 2026-07-26 while re-verifying
  every row for the FT8 fix above: real wall-clock was ~0.53-0.58 s, not
  the documented 0.049 s — not itself caused by that fix, bisected to
  `7bc1684` / issues #178-#180 (already merged to `main` before this
  pass started), which migrated FT4's `decode_frame_subtract` from a
  cheap constant-amplitude subtract onto the same WSJT-X-faithful
  channel-aware LPF subtract FT8 uses (`subtract_tones_lpf`,
  `FT4_SUBTRACT`'s `lpf_half=700`/`GfskParams`) for recall-quality
  reasons, without a corresponding re-measurement of this row.

  Root cause: not `subtract_tones_lpf` (already FFT-cached, <1 ms/call)
  but `engine::dsp::subtract::refine_freq` — its ±5 Hz/0.1 Hz carrier
  grid search (~101 evaluations/candidate) called `generate_iq` fresh
  every evaluation, fully rebuilding the GFSK-shaped modulation
  (erf-based pulse table, `O(nsym·pulse_len)` convolution, `O(nwave)`
  `sin`/`cos` integration) even though only the carrier frequency
  differs between evaluations — ~35 ms/call, ×14 real decodes ≈ 490 ms,
  essentially the entire regression. Fixed by building the carrier-free
  phasor once per call and applying each grid point via a cheap
  per-sample NCO rotation instead of a full resynthesis
  (`ls_amp_mag_tweaked`), verified against the frozen full-resynthesis
  path with a differential + argmax-preservation test. `refine_freq`
  dropped to ~15.7 ms/call; `decode_frame_subtract` wall-clock:
  multi-threaded ~575.8 ms → ~280 ms, single-threaded ~893.6 ms →
  ~602 ms. Not a full return to 48.8 ms — the LPF subtract + freq-refine
  step is a deliberate, permanent recall-quality cost (see
  `FT4_BENCHMARK.md` section 13's update for the full account); this fix
  removes the *redundant* part of that cost, not the cost itself.
  Candidate count unchanged (31, matching the 25× coarse-sync fix's own
  numbers) and recall unaffected (still 6/6).

  **Follow-up, same day: `refine_freq` was still 81% of the 280 ms**
  (227 ms of it, isolated via a standalone microbenchmark) even after the
  NCO fix above — the fix cut *per-evaluation* cost, not the *evaluation
  count* (still 101 evaluations/call, ±5 Hz radius at 0.1 Hz step).
  Cross-checking the call site's own "+/-5 Hz refine radius…matches
  WSJT-X" comment against `lib/ft4/subtractft4.f90` found the comment
  wrong: WSJT-X's `subtractft4` has no frequency-refine step at all, so
  there was nothing to match. The ±5 Hz figure actually came from
  `refine_freq`'s generic doc comment (written for `coarse_sync`'s
  ~2.93 Hz FFT-bin uncertainty) and was never re-derived after FT4 moved
  onto `ft4_coarse_sync` + `ft4_sync_search` (section 7/13), whose df
  search (`engine::sync2d::ft4_sync_search`) only ever produces
  integer-Hz offsets — bounding the true optimum to within ±0.5 Hz of
  the reported freq by construction, tighter than the ±2.5 Hz the old
  comment assumed. Shrunk `refine_freq_radius_hz` `5.0 → 1.0` (kept the
  0.1 Hz step — the response's mainlobe is well under 1 Hz wide, so
  widening the step risks skipping it), cutting the grid 101 → 21
  evaluations/call. **280 ms → 110 ms** (~2.5x further, ~2.3x off the
  48.8 ms pre-#178 floor now vs. 5.7x before this pass), recall
  byte-identical (6/6 golden, 14/14 total decodes) and the
  `ft4_busy_band_fading_probe.rs` Rayleigh-fading regression guard
  unchanged at 10/10. See `FT4_BENCHMARK.md` section 16.
- FT8's `qso3_busy.wav` was the outlier at **4.73 s** (busy band, 16
  simultaneous signals) until a profiling pass (2026-07-20) found the
  cost wasn't BP/OSD at all but `subtract_tones_lpf`'s successive-
  interference-cancellation step: a naive O(candidates × NFRAME ×
  lpf_half) direct time-domain convolution, ~310 ms per accepted
  decode. Replaced with WSJT-X's own algorithm
  (`lib/ft8/subtractft8.f90`/`lib/ft4/subtractft4.f90`) — a cached
  filter-response FFT + one forward/inverse FFT pair per call,
  O(N log N) — dropping this row to 0.45 s (~10.5×) with byte-identical
  recall (7/8 golden, 7 phantom, 14 total). FT4's golden test was
  already on a different (non-LPF) subtract path and unaffected.
- FT8's `qso3_busy.wav` dropped further, **0.45 s → 0.12 s (~3.6×)**,
  in a follow-up pass (2026-07-25). Two wiring gaps, found by
  instrumenting `decode_block_multipass` directly rather than
  re-guessing from the algorithm shape:
  1. `decode_block`'s own `refine_candidates` / per-candidate
     `process_candidates_tuned_with_ap` calls were passing
     `fft_cache: None` at every call site, so each of up to ~45
     candidates/slot re-ran the 192 k-point forward FFT from scratch
     even though `fine_refine_pass1` (a sibling stage in the same
     pass) already built and used exactly this cache. Wired the
     existing `build_fft_cache`/`downsample_cached` helpers through
     `refine_candidates`, `process_candidates_tuned_with_ap[_ref]`,
     and `auto_ap_strategy::run`, lazily rebuilding only when the
     WSJT-X-mandated sequential subtract actually mutates `work`
     (not eagerly every pass). Alone: 0.43 s → ~0.12-0.22 s.
  2. `subtract_tones_lpf_fft`'s filter-response FFT was already
     cached (`cached_window_fft`, from the original 10.5× fix above),
     but the forward/inverse `FftPlanner` *plans* for `nfft = 180 000`
     were rebuilt on every call — measured at ~2.8 ms/call for plan
     construction vs ~0.7 ms for the transform itself, i.e. plan
     rebuild cost ~4× the FFT it was gating. Cached the plans in a
     `nfft`-keyed `OnceLock`, same pattern as
     `fill_symbol_spectra.rs`'s `SYMBOL_FFT_32`.

  Both fixes are FFT-cache wiring, not new algorithms — byte-identical
  recall verified at each step (7/8 golden, 7 phantom, 14 total on
  `qso3_apoff`; 5/6 JTDX AP-on extras unchanged on `qso3_apon`).
  `fft-rustfft` is `std`-gated but thread-free, so both caches also
  apply under `wasm32-unknown-unknown` (verified via a feature-matched
  build) — no embedded-target impact either way, since
  `decode_block_multipass`'s `not(fft-rustfft)` variant has no
  subtract loop (single-pass, no SIC) and never calls these paths.
- FT4 was the outlier at **1.20 s** (7.5 s slot, only 6 signals) until a
  profiling pass (2026-07-20, `dapper-soaring-nest` plan) found its
  coarse-candidate stage was structurally the wrong algorithm — a
  generic 2-D (freq × lag) Costas-correlation search producing ~4.5×
  redundant candidates per real signal frequency on this WAV (2000
  candidates / 440 distinct frequencies), each independently paying the
  full downstream sync-refine + LLR + BP + OSD cost. Replaced with
  `engine::ft4_coarse::ft4_coarse_sync`, a faithful `getcandidates4.f90`
  port (WSJT-X's actual FT4 candidate finder has no lag dimension at
  all) — dropping this row to 0.049 s (~25×) with byte-identical 6/6
  golden recall (see `FT4_BENCHMARK.md` section 13).
- FST4-60A was the outlier at **2.60 s** (60 s slot) until a profiling
  pass (2026-07-20) found `coarse_sync` itself was cheap (unlike FT4) —
  the real cost was OSD escalating into its expensive depth-3/4 tier for
  roughly half of all 50 candidates, because the shared
  `osd_depth3_min=18` gate was calibrated against FT8's `N_SYNC=21` but
  FST4's `N_SYNC=40` makes that a far looser bar (18/40=45% vs
  18/21=86%). Hand-calibrated a FST4-specific `osd_depth3_min=20`
  against the real AWGN/CCIR sweep (a naive full `N_SYNC`-scaled value,
  34, measured as a real ~0.5 dB AWGN sensitivity regression first) —
  dropping this row to 0.27 s (~8.4×) with recall matching the
  documented pre-fix baseline across all 4 channels + FST4-120/300 spot
  checks (see `FST4_BENCHMARK.md` section 8).
- Q65-60B and Q65-30A were outliers at **1.89 s**/**2.55 s** (both route
  through `decode_multi_period_for`) until a profiling pass (2026-07-20)
  found two stacked costs: a redundant FFT extraction repeated 6× per
  candidate across the `b90 × model` fading sweep (fixed by extracting
  once, reusing across the sweep — ~8-10% win alone), and
  `max_candidates=32` hitting its cap on every slot, paying the full
  8-stage decode ladder for candidates far below the real signal's own
  score. Score-distribution profiling found the real signal ranked #0
  (top score) in every slot of both golden recordings with a healthy
  margin. Cut in two verified steps (32→16, then — after re-profiling
  showed the fading-BP stage still ~79% of wall-clock — 16→8, matching
  the library's own `SearchParams::default()`) — dropping these rows to
  0.49 s / 0.64 s (~4×) with bit-identical recall (same message,
  frequency, BP iteration count) at every step. A later pass the same
  day (the coarse-sync overhaul below) moved both again: Q65-60B
  0.49 s → **0.28 s** (downstream savings from better candidate quality
  dominated), Q65-30A 0.64 s → **0.72 s** (the coarse-search itself got
  ~4× more expensive per frequency bin — see next bullet — and this
  4-slot multi-period test's longer combined audio didn't have enough
  downstream savings to offset that).
- Q65-60B/30A dropped again, **0.28 s → 0.12 s (~2.3×)** and
  **0.72 s → 0.56 s (~1.3×)**, in a follow-up pass (2026-07-25) prompted
  by the FT8 FFT-cache investigation above: same "is this pattern
  elsewhere too" question, applied to Q65. The FFT-cache wiring gap
  itself wasn't present here — `decode_at_grid_for`'s `GridDepth::Fast`
  (WSJT-X's own automatic-decode depth) has exactly one `(Δf,Δt)` grid
  cell per candidate, so a direct call-count instrumentation found only
  16 `extract_data_energies_wide` calls for Q65-60A's 16 candidates —
  not the ~333 an earlier estimate assumed — making any FFT-planner
  cache there negligible (confirmed by measurement, not implemented).
  Phase-by-phase instrumentation of `Spectrogram::build_for` (used by
  every candidate-search call) found a different bug instead: its noise-
  floor estimate (`q65/search.rs`) computed a trimmed mean of the bottom
  95% of FFT-magnitude bins via a full `sort_unstable_by` (O(n log n))
  over the *entire* magnitude array (hundreds of thousands to millions
  of cells for a 60-120 s slot), when only the unordered *set* of
  bottom-95% values was needed. Same class of fix as FT8's
  `xsnr2_db_simple` noise-floor median (already on `select_nth_unstable_by`,
  O(n) average) — Q65 just hadn't had it applied. Measured as 64% of
  `decode_multi_period_for`'s wall-clock on Q65-60B (short slot, low
  candidate count — `build_for` dominates), 12% on Q65-30A (longer
  multi-slot audio where the BP/fading-metric stage dominates instead).
  Byte-identical recall on both golden tests (VK7MO/VK7PD on 60B,
  K1JT/K9AN AP-list on 30A) before and after.
- Q65-60A (`decode_scan_for`/`decode_scan_with_ap_for`, a different code
  path from the two above) was rewritten (2026-07-20) as a faithful
  `q65_loops.f90`/`q65_dec_q012` `(Δf, Δt, b90)` grid search, replacing
  an AWGN-only narrow-window Bessel metric + time-only retry that wasn't
  a WSJT-X algorithm at all. The rewrite's own speed goal was already
  met before this row's final number — the golden-WAV time (1.57 s →
  1.49 s) barely moved at first because the real cost shifted from
  candidate-count to `intrinsics_fast_fading` calls, but recall on this
  real recording *improved* (3 → 4 messages recovered — a new
  `W7GJ N0TB -15`). A follow-up pass (score-distribution profiling,
  same methodology as the Q65-60B/30A `max_candidates` calibration
  above) found all 4 real signals ranked within the top 8 of 530 total
  coarse candidates for this recording — the golden test's own
  `SearchParams` used `max_candidates: 32`, paying for up to 24 wasted
  decode attempts per candidate list. Cut to `16` (2× headroom above
  the weakest real signal's observed rank 7, not pushed to the edge —
  the margin there is thin, ~0.002 in score, unlike the Q65-60B/30A
  case's healthy #0-with-margin ranking) — dropped this row to
  **0.69 s** (~2.2×) with bit-identical recall (same 4 messages, both
  plain and AP). Landing the grid-search rewrite itself also surfaced
  two further bugs, fixed the
  same day: a missing full/unpruned `ibw` sweep at the grid's origin
  cell, and coarse-sync time resolution 4× coarser than WSJT-X's own
  `NSTEP=8` (`Spectrogram`, `q65/search.rs`) — closing that gap required
  restructuring candidate selection to match `q65_ccf_22`'s own shape
  (per-frequency time-collapse + local-max NMS + noise-adaptive
  percentile admission), which is the shared-infrastructure change
  behind the other Q65 rows' speed shifts in this table (it's used by
  every Q65 decode path, not just this one). A follow-up the same day
  also found `decode_at_grid_for` was using the wrong fading model
  (Gaussian instead of WSJT-X's hardcoded Lorentzian), closing a
  further ~2.5-3 dB AWGN sensitivity gap for the wide-tone-spacing C/D/E
  sub-modes with no further speed impact. See `Q65_BENCHMARK.md` for
  the full investigation, including a real-`jt9` cross-check that
  caught a false lead (an apparent sub-mode-specific regression that
  turned out to be an SNR-sampling artifact) along the way.
- Q65-60D/120D/120E/300A (fading-metric paths, `decode_scan_fading_for`)
  don't go through `decode_at_grid_for` or its Lorentzian fix — they
  already take an explicit model parameter and sweep both — but they do
  share the coarse-sync overhaul above via `coarse_search_for`, which
  first moved their rows the wrong way: Q65-120D 0.15 s → 0.22 s,
  Q65-120E 0.32 s → 0.45 s, Q65-300A 1.05 s → 1.52 s (Q65-60D roughly
  flat, 0.39 s → 0.40 s). The coarse-search itself does ~4× more
  per-frequency work now (`NSTEP=8` samples the time dimension 4× more
  finely before collapsing to one candidate per frequency bin), and for
  these longer-audio recordings that raw cost increase wasn't offset by
  fewer wasted downstream decode attempts. Score-distribution profiling
  (same methodology as Q65-60A's `max_candidates` calibration) found
  the real signal ranked within the top 3 of 94-3094 total candidates
  in all four golden recordings (60D rank 1, 120D rank 0, 120E rank 2,
  300A rank 9 — 300A's margin over the next candidate is thin, ~0.0004
  in score, so its cut kept more headroom than the others), but each
  test's own `SearchParams` used `max_candidates` of 100/30/30/200.
  Cutting to 8/8/8/20 (2× headroom for 300A, not pushed to its thin
  margin) dropped all four rows well below their pre-coarse-sync-overhaul
  numbers: Q65-60D **0.08 s**, Q65-120D **0.12 s**, Q65-120E **0.26 s**,
  Q65-300A **0.34 s** — bit-identical recall throughout. An earlier
  profiling pass found Q65-300A's golden test took 8.95 s before an
  unasserted diagnostic pre-check was removed (see CHANGELOG); today's
  0.34 s is ~26× faster than that.
- Not comparable to the embedded (Xtensa) numbers quoted elsewhere in
  this doc (e.g. FT8's ~0.7-1.2 s post-SlotEnd) — those run a
  different no_std/fixed-point pipeline on a much slower MCU core;
  this table is host x86_64 only.

### Cross-platform: Apple Silicon (M5) vs. Ryzen 9 9900X — FST4

Measured 2026-07-22. **Compute environment**: Apple M5, 10 cores
(4 performance + 6 efficiency), 16 GB RAM, macOS 26.5.2 (build
25F84), rustc/cargo 1.96.0 (aarch64-apple-darwin), same
`cargo test --release --features full` command as the Ryzen9 numbers
above (`parallel` + `rustfft` both on, so this is wall time on a
10-core host, not single-thread).

**Golden-WAV single decode** (`fst4_60_diag_candidate_cost_split`,
`tests/fst4_sweep.rs`, same harness that produced the Ryzen9 0.27 s
figure — see `FST4_BENCHMARK.md` section 8):

| Run | `decode_frame` wall-clock | `coarse_sync` | decodes |
|---|---:|---:|---:|
| 1 | 0.571 s | 36.9 ms | 3 |
| 2 | 0.554 s | 23.0 ms | 3 |
| 3 | 0.556 s | 20.7 ms | 3 |

Median **0.556 s** vs. Ryzen9's documented **0.27 s** — **~2.1x
slower**, roughly tracking the core/thread gap (M5: 10 cores vs.
Ryzen9 9900X: 12C/24T) given this workload's cost is dominated by the
`rayon`-parallelized LLR+BP+OSD stage. Recall parity separately
confirmed via the actual gated test
(`fst4_60_wsjtx_sample_recall_vs_golden`): 1/1 golden, same 2 messages
as documented (`CQ N5TM EL29`, `CQ K9KFR EN71`). (Aside: the
diagnostic harness's own `decode_frame` call reports 3 total decodes
on this machine vs. 2 recorded for the original Ryzen9 investigation —
not a recall regression, since the officially-gated recall test above
matches exactly; plausibly floating-point-reduction order
nondeterminism from `rayon` on a near-marginal candidate. Not chased
further — flagged here for whoever revisits this.)

**AWGN + CCIR sweep** (`tests/fst4_sweep.rs::fst4_snr_sweep`): the
full 5-mode × 4-channel × 20-trial grid (5,120 trials) was attempted
first but aborted after projecting tens-of-hours total wall time from
an in-progress rate (FST4-15, the cheapest sub-mode, alone was on pace
for ~63 min) — `FST4_BENCHMARK.md`'s "tens of minutes" reference for
the Ryzen9 box was never a precise measurement of this exact grid, so
that mismatch is itself informative, not a sign of an M5-specific
regression. Rerun scoped to **FST4-30 only, all 4 channels, full
existing SNR grid** (12 SNR points × 4 channels × 20 trials = 960
trials) as a representative single sub-mode.

- **Wall-clock (the actual speed benchmark)**: M5 **19 m 25 s** (`real`,
  `user` 179 m 58 s, ~9.3x average core utilization out of 10) vs.
  Ryzen9 9900X **6 m 57 s** (`real` 417.17 s, `user` 135 m 43 s, ~19.5x
  average core utilization out of 24 — 81% of available threads,
  confirming the `rayon` path is engaged on both boxes) for the
  identical scoped run (mode=30, all 4 channels, unfiltered SNR grid;
  Ryzen9 side measured 2026-07-24 on the same box as the rest of this
  doc's host figures, `cargo test --test fst4_sweep --release --features
  fst4,fft-rustfft,parallel,uvpacket -- --ignored --nocapture
  fst4_snr_sweep`). Ryzen9 is **~2.8x faster wall-clock** — tracking the
  core/thread-count gap (24 vs. 10 logical contexts) more closely than
  the single golden-WAV decode's ~2.1x gap above, consistent with this
  sweep having far more independent trials (960) to spread across cores
  than one `decode_frame` call's ~50-candidate list.
- **Recall crossings — not a speed metric, a correctness sanity
  check.** The decoder is the same algorithm on both architectures, so
  it should recover the same messages at the same SNR regardless of
  CPU; this only exists to confirm the M5 build wasn't silently
  decoding *differently* before trusting the speed numbers above.
  Ryzen9's own crossings from this scoped run (same linear-interpolation
  method used throughout this doc): AWGN ≈ −23.90 dB — matching the
  already-documented FST4-30 main-table figure below exactly, an
  independent reproduction — CCIR good ≈ −23.3 dB, moderate ≈ −21.4 dB,
  poor ≈ −21.7 dB.

  | Channel | Ryzen9 9900X | Apple M5 | Δ |
  |---|---:|---:|---:|
  | AWGN | ≈ −23.90 dB | ≈ −23.71 dB | 0.19 dB |
  | CCIR good | ≈ −23.3 dB | ≈ −22.60 dB | 0.7 dB |
  | CCIR moderate | ≈ −21.4 dB | ≈ −21.50 dB | 0.1 dB |
  | CCIR poor | ≈ −21.7 dB | ≈ −20.82 dB | 0.9 dB |

  AWGN and CCIR moderate agree tightly (≤0.2 dB) between the two
  independently-seeded 20-trial corpora. CCIR good/poor show a larger
  ~0.7-0.9 dB spread — plausibly the same 20-trial sampling noise this
  doc notes elsewhere on steep parts of an SNR curve, landing
  differently on two independent corpora, rather than a platform-
  specific decode difference (if it were platform-specific, AWGN/
  moderate on the same two corpora should show it too, and they don't).
  No evidence the M5 build decodes differently from Ryzen9.

### WASM: `+simd128` + mfsk-core update effect on FT8 decode (Node, wasm32-unknown-unknown)

Measured 2026-07-28, in support of issue #208 (mfsk-core owing docs +
implementation back to WASM consumers after `jl1nie/webft8#5` found the
wasm build had no SIMD feature enabled). **Compute environment**: Node.js
v26.3.0 (`wasm-pack build --target nodejs`), target
`wasm32-unknown-unknown`, `opt-level = 3` + `lto = true`. `--target
nodejs` and `--target web` emit the identical `.wasm` (same SIMD
instructions) — only the JS glue's module-loading strategy differs
(`require`/`import` vs. browser `fetch()`) — so timing `decode_wav()`
under `--target nodejs` is a valid headless proxy for the real deployed
WASM binary's speed, not a separate/unrepresentative build.

Two independent measurement passes, same methodology (`decode_wav()`
FT8 decode calls, `performance.now()`, median of 5-7 runs per config,
same input, steady-state):

1. **Flag-only** (this repo's own `sim_busy_band.wav`/`sim_extreme_hard.wav`
   local run, before the fixes below existed): isolates `-C
   target-feature=+simd128` alone, nothing else changed.
2. **Flag + mfsk-core update** (independently re-measured downstream in
   `jl1nie/webft8`, same two WAVs, same `--target nodejs` methodology):
   `+simd128` plus this repo's `main` after #211 (`downsample_cached`
   FFT-plan caching) and #213 (LLR loop-order/branchless vectorization)
   landed.

| WAV | scalar baseline | `+simd128` only | `+simd128` + mfsk-core update | additional gain | vs. scalar baseline |
|---|---:|---:|---:|---:|---:|
| sim_busy_band.wav | 194.2 ms | 156.5 ms | 128.8 ms | 17.7% | 33.7% |
| sim_extreme_hard.wav | 216.5 ms | 173.1 ms | 146.8 ms | 15.2% | 32.2% |

Notes:
- `sim_busy_band.wav` / `sim_extreme_hard.wav` are not part of this
  repo's committed fixture set (`ft8sim`-generated stress WAVs from a
  sibling `webft8` checkout) — both passes above are manual local
  repro, not CI-reachable. A committed, CI-safe harness now exists
  (`bench/wasm/`, wrapping the same `DecodeRequest::<Ft8>` call as
  `mfsk-core/tests/ft8_sweep.rs`, default fixture
  `embedded-poc/assets/qso3_busy.wav`) for reproducing the *shape* of
  these results on the golden fixture; see its README for usage.
- The "additional gain" column (15.2-17.7%) roughly matches the sum of
  what #211 and #213's own PRs measured independently: #211's
  `node --prof` flat profile attributed ~12.5% of total decode
  wall-clock to the FFT-plan-rebuild bug it fixed; #213's fix had a
  real but small (~2.9%) ceiling, individually within measurement noise
  on `qso3_busy.wav` but evidently contributing at the larger scale
  these two WAVs exercise.
- Decode *recall* was not independently re-verified against these two
  WAVs specifically (they're not committed fixtures with a known-golden
  answer) — the byte-identical-recall gate for #211/#213 was run
  against this repo's own FT8 golden regressions (`qso3_busy.wav`
  full-parity/AP-off/JTDX, see their PR descriptions), not against
  these two `webft8`-side WAVs.
- LDPC belief-propagation remains explicitly out of scope for
  vectorization (issue #208) — it's a gather/scatter-bound reduction, a
  poor SIMD target; the only real lever there is batching candidates
  across lanes, a separate, larger effort tracked independently.

## FT8

- **WSJT-X 8-entry golden: 7/8 ship-config (`DecodeDepth::EMBEDDED`,
  the real Core2/S3/host-mirroring-embedded path — no OSD by design,
  missing only `K1BZM DK8NE -10`), 8/8 host full-parity
  (`DecodeDepth::FULL`, `sync_min=0.8`, `max_cand=60` — see the
  "Decode speed" section above and `tests/ft8_qso3_full_parity_recall.rs`).**
  The two aren't the same code path with the same result — ship-config
  structurally cannot run OSD at all (issue #182 follow-up,
  `DecodeDepth` redesign), so 7/8 is its permanent floor, not a
  temporary gap.
- **JTDX 18-entry golden: 18/18** (`decode_block`; was 17/18 through
  issue #180's fix, 2026-07-25). The former miss, `WA2FZW DL5AXX`, was
  **not** a JTDX false positive — that classification is retracted.
  Root cause was `subtract_tones_lpf`'s SIC low-pass window: its cos²
  argument was divided by `lpf_half` instead of `NFILT` (`=
  2*lpf_half`), doubling the taper's argument range so the kernel gave
  samples 166 ms away **full weight** instead of tapering to 0 —
  actively corrupting the QSB/channel estimate on every SIC subtract
  since this became the canonical FT8/FT4 SIC path (v0.6.2). Real
  WSJT-X `jt9 -d3` independently decodes `WA2FZW DL5AXX RR73` on the
  same WAV, confirming it was always a real signal, just one
  mfsk-core's own broken SIC window was failing to clean up enough to
  reach. See issue #180 and PR #178 for the full investigation.
- **Host AP-on multipass JTDX-extras: 6/6** via
  `decode_frame_subtract_with_ap` (was 5/6 through 2026-07-25; the
  remaining miss, `K1BZM DK8NE`, was never an AP-list breadth problem
  — that hypothesis is retracted). Root cause was `osd_decode_npre1`
  (WSJT-X's OSD `ndeep=2` dispatch) being fed raw channel LLR instead
  of the BP-refined `bp_llr_zsum`, unlike WSJT-X's real
  `decode174_91.f90` driver, which always hands OSD the post-BP LLR.
  Fixed by threading the already-computed BP-refined LLR through to
  the OSD call site instead of recomputing/reusing the pre-BP one
  (issue [#182](https://github.com/jl1nie/mfsk-core/issues/182)).
- **Embedded (M5StickS3, Xtensa LX7, fixed-point)**: 6/18 + 1 bonus =
  7 total on the same WAV, in ~1.19 s post-SlotEnd via the streaming
  pipeline. M5Stack Core2 (LX6): ~2.8 s.
- **SNR calibration**: `xsnr2_db_simple` lands reported SNR within
  ±3 dB of JTDX absolute on real silicon.
- CCIR moderate/poor fading recall gap closed in 0.7.3 by widening
  `OSD_HARDERRORS_MAX` back to WSJT-X's universal 36.

**AWGN/CCIR sensitivity sweep** (`tests/ft8_sweep.rs`, `ft8sim`-driven,
`-5` to `-26` dB grid, 4 channel conditions). Re-measured 2026-07-26
(issue #182 follow-up, alongside the `DecodeDepth` redesign below —
confirmed by `git diff` scope + `decode_frame_inner`'s separate call
graph that the redesign itself moved none of these numbers; the delta
vs the last-tracked figures is accumulated prior sensitivity work
never rolled into this table before, see `FT8_BENCHMARK.md` section 9):

| Channel | mfsk-core 50% crossing | WSJT-X published / real `jt9 -8 -d3` | Gap |
|---|---:|---:|---:|
| AWGN | ≈ −21.6 dB | −20 to −21 dB published; ≈ −21.2 dB real `jt9` | mfsk-core +0.4 dB ahead |
| CCIR good | ≈ −21.1 dB | ≈ −20.75 dB real `jt9` (no published figure) | mfsk-core +0.35 dB ahead |
| CCIR moderate | ≈ −20.0 dB | ≈ −19.5 dB real `jt9` (no published figure) | mfsk-core +0.5 dB ahead |
| CCIR poor | ≈ −19.7 dB | ≈ −19.7 dB real `jt9` (no published figure) | ~parity |

WSJT-X doesn't publish per-fading-model thresholds, but `ft8sim`'s the
same simulator generating this corpus, so a real `jt9 -8 -d 3` run
against it is real ground truth. Figures above are post-fix
(2026-07-26) — see `FT8_BENCHMARK.md` section 11 for the root cause
(real `jt9 -8 -d3`'s CLI always runs an implicit CQ-AP hypothesis pass
that mfsk-core's blind `decode_frame` wasn't attempting) and the fix
(`process_one_candidate_inner`'s blind-CQ pass now always runs, not
just under an explicit `ap_hint`). Issue
[#190](https://github.com/jl1nie/mfsk-core/issues/190) closed.

Reproduce: `docs/notes/FT8_BENCHMARK.md`.

- **`DecodeDepth` redesigned from a flat, ad-hoc 4-variant enum into
  an orthogonal `{ llr_effort: LlrEffort, osd: bool }` struct; the
  automatic auto-AP rescue it used to gate was removed entirely**
  (2026-07-26, issue #182 follow-up). `BpVariantsAd`/`BpAllOsd`/
  `BpAll` became `DecodeDepth::EMBEDDED`/`FULL`/`BP_ONLY`;
  `BpAllNoNsym3` (zero real callers) has no replacement. `osd: true`
  is now host-only *by construction* — the OSD dispatch code is
  `#[cfg(feature = "fft-rustfft")]`-excluded from embedded builds,
  not just gated at runtime. Separately, `depth.osd` turned out to
  also be the only thing preventing an *unconditional*, `ap_hint`-
  independent auto-AP widening pass from firing inside every
  `decode_block*` call — measured on `qso3_busy.wav`: **1320-1450 ms
  → 145-151 ms** for the identical 19 decodes once removed (9× for
  zero recall difference at any realistic `max_cand`). See
  `CHANGELOG.md` for the full investigation and migration mapping.
- **New regression: `tests/ft8_qso3_full_parity_recall.rs`** — the
  **host research config** (`DecodeDepth::FULL`, `sync_min=0.8`,
  `max_cand=60`) hits the full WSJT-X 8-entry golden (**8/8**,
  including `K1BZM DK8NE -10`, which ship-config's `DecodeDepth::
  EMBEDDED` structurally cannot reach with OSD compiled out) in
  **~165-175 ms** (was ~139-148 ms before the issue #190 fix below
  made the blind-CQ AP pass always-on; dropped further to ~118-127 ms
  2026-07-29 — see the "Decode speed" table's own re-measurement note
  above) — still faster than real `jt9 -8 -d3`'s own ~1.1 s total file
  decode time, at full recall parity. Distinct from the ship-config row
  above, which is the real Core2/S3 production path and floors at 7/8
  by design.
- **Issue #190 fix (2026-07-26): the blind-CQ AP pass (WSJT-X
  iaptype-1, "Pass 12" in `process_one_candidate_inner`'s Step 4) now
  runs when the blind BP/OSD staircase fails and `nsync ≥
  BLIND_CQ_MIN_NSYNC (12)`, independent of `ap_hint`** — not a revival
  of the `auto_ap_strategy` module removed above (that was iaptype-2
  self-seeding from same-slot callsigns, an mfsk-core-original
  extension with no WSJT-X counterpart; this is iaptype-1, a faithful
  port that needs no mycall/hiscall at all). Real `jt9 -8 -d3`'s CLI
  hardcodes AP on (`lib/jt9.f90:302`) and always tries this hypothesis
  for idle-state candidates — mfsk-core had the matching code already
  but gated it behind an explicit `ap_hint`, so plain `decode_frame`
  calls never reached it. The nsync gate was added after measuring an
  un-gated first version: it sent 188 candidates through this pass on
  `qso3_busy.wav`'s multipass staged-SIC path (140 of them at nsync
  7-9, none producing a decode there), pushing that file's wall-clock
  0.7 s → 1.5 s — slower than real jt9's own ~1.15 s on the same file.
  Gating at nsync≥12 (comfortably below the 14-18 real recoveries
  need) cuts that to 34 candidates and ~0.85 s, faster than jt9 again,
  with zero recall change either way. Closes the CCIR moderate/poor
  sensitivity gap from the sweep table above (moderate now ahead of
  jt9, poor at parity) — see `FT8_BENCHMARK.md` section 11 for the
  full trace, cost investigation, and re-measurement.

## FT4

- **6/6 WSJT-X golden** (`samples/FT4/000000_000002.wav`), decoded in
  **~0.10 s** as of 2026-07-29 (was ~0.28 s as of 2026-07-26; briefly
  0.049 s right after the `getcandidates4.f90`-port fix, up from
  1.20 s before it — see "Decode speed" notes above and
  `FT4_BENCHMARK.md` section 13 — then a later LPF-subtract migration
  re-inflated it to ~0.53-0.58 s via a redundant-resynthesis bug in
  `refine_freq`, fixed same-day, issue #182; see the "Decode speed"
  table's own FT4 note for the full account and its 2026-07-29
  re-measurement note for the latest drop).
- **AWGN sensitivity gap: ~0.6 dB** (was ~0.3 dB before the coarse-sync
  algorithm swap, ~1.8 dB pre-0.7.3) — 50% recall crossing moved from
  −15.5 dB to −17.2 dB after a coherent Costas-block scorer fix and an
  OSD-attempt gate that had been checking a non-coherent score (issue
  #72), then to −16.9 dB after replacing the coarse-candidate stage with
  a faithful `getcandidates4.f90` port for the ~25× speed win above — a
  structural algorithm swap, not a threshold retune, so unlike the
  earlier fixes it reshaped rather than uniformly improved the
  SNR-response curve (3 of 4 channels held or improved; AWGN alone
  softened by ~0.3 dB — see `FT4_BENCHMARK.md` section 13 for the
  per-channel breakdown).
- Successive-interference-cancellation primitives
  (`subtract_signal*`, `refine_signal_freq`) ported from
  `lib/ft4_subtract.f90`; WSJT-X's Fast/Normal/Deep decode-depth menu
  exposed via `decode_frame_with_options`.

**AWGN/CCIR sensitivity sweep** (`tests/ft4_sweep.rs`, `ft4sim`-driven,
4 channel conditions):

| Channel | mfsk-core 50% crossing | WSJT-X published | Gap |
|---|---:|---:|---:|
| AWGN | ≈ −16.9 dB | −17.5 dB | 0.6 dB |
| CCIR good | ≈ −17.5 dB | — (no separate WSJT-X figure) | — |
| CCIR moderate | ≈ −15.7 dB | — | — |
| CCIR poor | ≈ −16.0 dB | — | — |

Reproduce: `docs/notes/FT4_BENCHMARK.md`.

## FST4

Five T/R-period sub-modes wired (FST4-15/30/60A/120/300), every
constant verified directly against WSJT-X `fst4_decode.f90` /
`fst4sim.f90` source.

| Sub-mode | mfsk-core 50% crossing | WSJT-X published | Gap |
|----------|------------------------:|------------------:|----:|
| FST4-15  | ≈ −20.60 dB | −20.7 dB | 0.10 dB |
| FST4-30  | ≈ −23.90 dB | −24.2 dB | 0.30 dB |
| FST4-60  | ≈ −27.62 dB | −28.1 dB | 0.48 dB |
| FST4-120 | ≈ −30.70 dB | −31.3 dB | 0.60 dB |
| FST4-300 | ≈ −34.78 dB | −35.3 dB | 0.52 dB |

Closed via a coherent full-slot local sync search, an FST4-specific
`LLR_NSYM_MAX` override, an nsym=4 LLR rung, and a zsum-OSD fallback.

Only FST4-60A has a real-recording golden-WAV lock (1/1,
`samples/FST4/210115_0058.wav`) — WSJT-X's sample tree ships no
FST4-15/30/120/300 recordings, so those four are validated by
synth-roundtrip self-consistency plus `fst4sim` sweep only, not a real
on-air recording. FST4-900/1800 and FST4W (the WSPR-style one-way
beacon variant) remain out of scope — no user demand as of writing.

**Decode speed** (2026-07-20): FST4-60A's golden-WAV decode dropped
**2.60 s → 0.27 s (~8.4×)** after hand-calibrating the OSD
depth-escalation gate for FST4's own `N_SYNC=40` (was hardcoded to a
value calibrated for FT8's `N_SYNC=21`) — recall verified unchanged
against the table above (re-measured this pass: FST4-60 AWGN ≈-27.56 dB,
CCIR good/moderate/poor bit-identical to the pre-fix figures; FST4-120/
300 AWGN spot-checked, also unchanged). Dropped further, **0.27 s →
~0.08 s (~3.5×)**, on 2026-07-29 — see the "Decode speed" table's own
re-measurement note above (issue #197's `SYNC_Q_MIN` tightening plus
the shared-pipeline lazy-LLR-staircase and FFT-planner-cache fixes).
See `FST4_BENCHMARK.md`
section 8 for the full investigation, including a first-attempt fix
that measured as a real ~0.5 dB regression and was corrected before
shipping.

Reproduce: `docs/notes/FST4_BENCHMARK.md`.

## WSPR

- **8/8 WSJT-X golden** (`samples/WSPR/150426_0918.wav`), ~0.37 s
  end-to-end on a desktop build (was ~0.88-0.93 s — WSPR's first
  dedicated perf-review pass, 2026-08-08, mainly a rayon
  parallelization of `decode_scan_subtract`'s pass-1/pass-2 candidate
  loops; see the "Decode speed" section's own 2026-08-08 note for the
  full commit list and the AWGN-sweep re-verification confirming no
  recall change) — sub-bin demod + 2-pass subtract+re-coarse + OSD-2
  fallback + Type-3 phantom filter.

**AWGN sensitivity sweep** (`tests/wspr_sweep.rs`, `wsprsim`-driven,
13 SNR points × 20 trials each):

| SNR | Recall |
|---:|---:|
| −34 dB | 0.0% |
| −32 dB | 0.0% |
| −31 dB | 0.0% |
| −30 dB | 40.0% |
| −29 dB | 95.0% |
| −28 dB | 95.0% |
| −27 dB | 100.0% |
| −26 dB | 100.0% |
| −24 dB | 100.0% |
| −20 dB | 100.0% |
| −15 dB | 100.0% |
| −10 dB | 100.0% |
| 0 dB | 100.0% |

50% crossing ≈ −29.8 dB — consistent with WSJT-X's published WSPR
sensitivity floor (commonly cited around −28 to −30 dB, 2500 Hz
reference bandwidth).

## JT9

- **7/7 WSJT-X golden** (`samples/JT9/130418_1742.wav`) via the full
  WSJT-X-faithful softsym pipeline (`afc9` + `chkss2` + `xx0` mettab +
  `sync9` per-freq collapse). This table previously said 5/5 — stale;
  `jt9_wsjtx_samples.rs`'s own comment records 7/7 as of 2026-05-08,
  well before this row was last touched. Found and corrected
  2026-07-26 during an unrelated cross-protocol re-verification pass;
  decode speed re-confirmed unchanged (0.33 s → 0.34 s, within noise)
  at the time. Still 7/7 as of 2026-08-08's `select_nth_unstable_by` +
  `realfft` speed fixes (~340 ms → ~302 ms on this same golden WAV —
  see the "Decode speed" section's second 2026-08-08 note).

**AWGN sensitivity sweep** (`tests/jt9_sweep.rs`, `jt9sim`-driven,
20 trials/SNR):

| SNR | Recall |
|---:|---:|
| −30 dB | 0.0% |
| −28 dB | 0.0% |
| −27 dB | 10.0% |
| −26 dB | 65.0% |
| −25 dB | 90.0% |
| −24 dB | 100.0% |
| −22 dB | 100.0% |
| −20 dB | 100.0% |
| −18 dB | 100.0% |
| −15 dB | 100.0% |
| −10 dB | 100.0% |
| −5 dB | 100.0% |
| 0 dB | 100.0% |
| +5 dB | 100.0% |
| +10 dB | 100.0% |

50% crossing ≈ −26.3 dB — no measurable gap vs. a real WSJT-X `jt9 -9`
build on the identical 300-file corpus (100% to −25 dB, 80% at
−26 dB there; per-cell differences are within 20-trial sampling noise
at the steep part of the curve).

## JT65 — gap closed: chase decoder + FFT-bin scalloping-loss fix (2026-08-08, issue #169)

- No real-recording golden WAV available: WSJT-X's own v3 reference
  samples need soft-symbol erasure metadata that lives in private
  WSJT-X branches. AWGN sweep (`tests/jt65_sweep.rs`, `jt65sim`-driven,
  20 trials/SNR) remains the sensitivity instrument.
- **Root cause** (unchanged from the original writeup): WSJT-X's own
  no-`kvasd` path (`jt9 -6`) isn't plain hard-decision RS — it's
  `lib/extract.f90` calling `ftrsdap` (`lib/ftrsd/ftrsdap.c`), a
  stochastic Chase decoder that runs many randomized soft-symbol
  erasure-pattern trials around Berlekamp-Massey RS, using both the
  most- and second-most-reliable symbol per position, then ranking
  candidates and applying an acceptance-margin gate. The old
  `decode_at_with_erasures` only ever tried one deterministic
  increasing-erasure-count ordering.
- **Faithful port, magic numbers included** (`mfsk_core::jt65::chase`,
  `decode_at_with_chase`/`decode_scan_chase*`) — an initial pass
  (same day) shipped a simplified approximation (linear
  confidence→erasure-probability mapping, an `nerr`+cost proxy instead
  of WSJT-X's real spectral-power ranking); the user asked for
  literal fidelity to the original instead, so it was replaced with:
  - **`PERR[8][8]`**: WSJT-X's own hand-tuned erasure-probability
    table (`ftrsdap.c:43-51`), ×1.3 scale, keyed by confidence-ratio
    bucket and reliability-rank bucket — ported verbatim, not
    approximated (this crate's `conf = (best−second)/best` already
    gives the exact ratio the table needs, so no separate raw
    probability pair was needed).
  - **`getpp`**: a literal port of `extract.f90`'s candidate-quality
    subroutine — re-encodes each successful trial's codeword, walks it
    through the same interleave+Gray-encode the transmitter uses, and
    averages the *original* un-thresholded FFT-bin power at the
    resulting 63 (position, tone) pairs. Required retaining the full
    63×64 raw power spectrum through the demod step
    (`rx::demodulate_aligned_with_runnerup`, ~16 KB — a real
    architectural change from the first pass, which had deliberately
    avoided this; not an embedded/no_std concern since JT65 already
    requires `std`/`fft-rustfft`).
  - **`nhard`/`nsoft`/`ntotal`**: WSJT-X's literal soft-distance
    formula (a position matching the runner-up guess costs nothing
    toward `nsoft`, one matching neither costs its full confidence
    weight), using the retained runner-up-tone identity.
  - **Acceptance gate**: `ntotal ≤ nd0(81) && pp2/pp1 ≤ r0(0.87)`
    (`extract.f90:169-176`, WSJT-X's "normal" aggressiveness pair) —
    replaces the first pass's own invented hit-count/margin heuristic.
    Best/second-best `pp` tracked directly across all trials (not a
    per-message tally — WSJT-X doesn't dedup by message at all, purely
    by raw quality score, and two different erasure trials landing on
    the same message are simply two data points toward the same `pp1`).
  - **Early exit**: `nhard ≤ 41 && ntotal ≤ 71` (`ftrsdap.c:211`),
    literal.
  - **RNG**: the exact `ir` extraction (`(nseed/65536) % 32768`, then
    `*100/32768`), not a simplified `%100`.
  - **Trial count**: `1000`, matching WSJT-X's own `jt9 -6` — the
    literal build this crate's JT65 numbers are benchmarked against —
    via `decoder.f90`'s `nranera=6 → ntrials=10**(6/2)=1000` formula
    (previously guessed at `2000` with no real derivation).
  Not ported: AP-hint passes and the `hint65` correlation fallback
  (`extract.f90`'s remaining machinery) — out of scope, no request for
  them yet. Two always-run false-decode-rate tests
  (`jt65::chase::tests::chase_never_false_decodes_*`, 20 seeds each of
  pure noise and a signal synthesized far below the measured floor)
  re-verify the ported acceptance gate still rejects spurious RS
  syndrome hits. Additive-only, as before: the original
  `decode_at`/`decode_at_with_erasures`/`decode_scan*` functions and
  their tests are untouched and still byte-identical.
- **Bug found and fixed same day, user-prompted** ("なぜこんなにギャップが
  あるの？何か見落としていると考えるのが合理的な結論だと思うけど？"):
  the faithful port above still used this crate's pre-existing `conf`
  metric (`(best−second)/best`, a top-2-tone-only margin) everywhere
  WSJT-X's `ftrsdap` actually uses `rxprob`/`mrprob` — real
  `demod64a.f90` source: `mrprob(j) = scale * (s1/psum)`, where `psum`
  sums **all 64** tone powers at that position, not just the top two.
  This is a materially different quantity: `mrprob` is a peakiness/SNR
  measure that reflects how much energy leaked into the noise floor
  across the *whole* spectrum at that position, which the top-2-only
  `conf` margin can't see (two positions with an identical best/second
  margin can have very different `mrprob` depending on how much power
  sits in the other 62 tones). This fed the wrong quantity into both
  the erasure-priority ordering and the `nsoft` soft-distance weighting.
  Fixed by adding a `rel` field (WSJT-X's real `mrprob`) to
  `rx::demodulate_aligned_with_runnerup` — `best_pwr/total_pwr` over
  all 64 tones, cheap (the crate already computed `total_pwr` for its
  SNR estimate) — and using `rel` everywhere WSJT-X uses
  `rxprob`/`mrprob`. `conf` is *still* correct and still used for the
  `PERR` table's ratio bucket, since `rxprob2/rxprob` algebraically
  reduces to `second_pwr/best_pwr` regardless of the `psum`
  normalization — exactly what `1 - conf` already gives; that one
  piece needed no change.
- **Measured result after the `rel`-metric fix**
  (`ChaseParams::default()`, 1000 trials, `nd0=81`, `r0=0.87`): 50%
  crossing moved from −14 dB to ≈ −19.1 dB — a further ~0.9 dB over
  the pre-fix faithful port's ≈ −18.3 dB. Still ~2-3 dB short of
  WSJT-X's documented `jt9 -6` floor (~100% to −22 dB). This prompted
  the user to push back directly: "なぜこんなにギャップがあるの？何か
  見落としていると考えるのが合理的な結論だと思うけど？" — a fair
  challenge that led to the real remaining cause below.
- **Root cause of the residual gap: FFT bin quantization ("scalloping
  loss"), not anything left inside `ftrsdap`.** A phase-by-phase
  comparison against WSJT-X's real JT65 pipeline (`decode65a.f90` →
  `decode65b.f90` → `extract.f90`, not just `ftrsdap.c` in isolation)
  found that WSJT-X applies a continuous (non-quantized) frequency+
  drift correction to the *time-domain* signal (`afc65b.f90`'s
  chi-square fit + `twkfreq65.f90`'s phase-continuous correction)
  **before** any FFT. This crate's JT65 demod had no equivalent at
  all — `rx.rs` rounded every candidate frequency to the nearest FFT
  bin (`base_bin = (base_freq_hz/df).round()`) and stopped there. For
  a rectangular window, a tone sitting exactly half a bin off center
  suffers a well-known worst-case ≈3.9 dB power loss. This crate's own
  AWGN sweep golden frequency, 1500 Hz, sits at **exactly** bin 557.5
  (NSPS=4460 @ 12 kHz ⇒ 2.6906 Hz/bin) — the worst possible case,
  hit on *every single trial* in the corpus. Confirmed directly: a
  throwaway probe decoding the same synth signal + identical noise at
  a bin-centered frequency (1498.65 Hz) vs. the golden half-bin-off
  frequency (1500.0 Hz) showed 100% vs. 43-50% recall at identical
  SNRs (−16…−20 dB) — decisive, textbook-consistent confirmation, and
  the true magnitude of "what was being missed" (bigger than anything
  found inside `ftrsdap` itself).
- **Fix**: two additive pieces, both apply to *every* JT65 decode path
  (`decode_at`, `decode_at_with_erasures`, `decode_at_with_chase`, and
  all `decode_scan*` variants) since they share the same demod/search
  code, not just chase:
  - `search::refine_freq_hz` — 3-point log-power parabolic ("Jacobsen")
    interpolation of the sync-tone power at `base_bin-1/base_bin/base_bin+1`,
    reusing the already-built `Spectrogram` (no extra FFTs). Gives
    `SyncCandidate::freq_hz` genuine sub-bin precision instead of an
    exact bin multiple.
  - `rx::demodulate_aligned_with_confidence_inner` — a running-phase
    NCO (same accumulator pattern as `engine::dsp::subtract`'s NCO
    loops) cancels the residual sub-bin offset (`base_freq_hz − base_bin·df`)
    on the time-domain audio before each symbol's FFT, phase-continuous
    across all 126 symbol windows — the same idea as WSJT-X's
    `twkfreq65`, just without the drift term (this crate's coarse
    search doesn't estimate drift; the AWGN sweep corpus has none to
    correct for). Exactly a no-op when the caller already passes a
    bin-aligned frequency (residual = 0), so every existing call site
    keeps working unchanged.
  Verified directly: re-running the earlier bin-alignment probe with
  the fix in place made the 1500 Hz (worst-case) and 1498.65 Hz
  (bin-centered) cases perform identically (100% at every tested SNR),
  confirming the NCO correction fully cancels the scalloping loss when
  fed a genuinely refined frequency.

**Final measured result** (`tests/jt65_sweep.rs`, same 300-file corpus,
20 trials/SNR, faithful `ftrsdap` port + `rel`-metric fix + bin-alignment
fix, all together):

| SNR | Before (`decode_at_with_erasures`) | After (`decode_at_with_chase`) |
|---:|---:|---:|
| −25 dB | 0.0% | 15.0% |
| −22 dB | 45.0% | 100.0% |
| −20 dB | 100.0% | 100.0% |
| −19 dB | 100.0% | 100.0% |
| −18 dB | 100.0% | 100.0% |
| −17 dB | 100.0% | 100.0% |
| −16 dB | 100.0% | 100.0% |
| −15 dB | 100.0% | 100.0% |
| −14 dB | 100.0% | 100.0% |
| −12 dB | 100.0% | 100.0% |
| −10 dB | 100.0% | 100.0% |
| −5 dB | 100.0% | 100.0% |
| 0 dB | 100.0% | 100.0% |
| +5 dB | 100.0% | 100.0% |
| +10 dB | 100.0% | 100.0% |

50% crossings (linear-interpolated): **plain hard-decision
`decode_at_with_erasures` ≈ −21.8 dB** (between the −22 dB/45% and
−20 dB/100% cells), **`decode_at_with_chase` ≈ −23.8 dB** (between the
−25 dB/15% and −22 dB/100% cells) — both now at or beyond the
previously-cited WSJT-X `jt9 -6` reference floor (~100% to −22 dB).
Take that comparison with real caution, not as a declared "we beat
WSJT-X": the −22 dB reference figure's own original provenance (was it
measured against a real `jt9` binary on this exact corpus, at what
aggressiveness setting, with its own AFC actually engaged) was not
re-verified in this session, and the corpus's lowest grid point
(−25 dB) is no longer deep enough to pin down either decoder's true
floor — chase already shows partial recall there. The honest, load-
bearing conclusion is narrower and still substantial: **issue #169's
originally-diagnosed ~7-8 dB gap is gone** — both fixes (faithful
`ftrsdap` port with the correct reliability metric, and the bin-
alignment fix) were real, and together account for effectively all of
it on this corpus. A deeper AWGN sweep (regenerated corpus reaching
below −25 dB) and an independent real-`jt9`-binary re-comparison would
be needed to make a rigorous "beats WSJT-X" claim; neither was done
here. Issue #169 can reasonably be considered closed as filed (the
diagnosed gap is closed); any further work is genuinely open-ended
tuning, not gap-closing.

(The bin-alignment fix improves *every* JT65 decode path, not just
chase — `decode_at_with_erasures`'s own numbers above jumped just as
dramatically, 50%-at−14dB → ≈−21.8dB, with zero code changes to
`decode_at_with_erasures` itself; it inherits the fix purely by
sharing `search`/`rx` with the new code.)

## Q65

- Real recordings: WSJT-X's 6 m EME (W7GJ exchanges) and 10 GHz EME
  reference both decode.
- Fast-fading metric (Gaussian/Lorentzian channel models) recovers
  5-8 dB on Doppler-spread channels, required for microwave EME.
- AP-list template matching decodes 6/6 frames at SNR −25 dB where
  plain BP fails 0/6.
- **Decode speed** (2026-07-20): Q65-60B/30A first dropped ~4× via a
  redundant-extraction fix and a calibrated `max_candidates` cut in
  `decode_multi_period_for`, then moved again (60B faster, 30A slower)
  when the coarse-sync overhaul below landed. Q65-60A (and every other
  sub-mode routed through `decode_scan_for`/`decode_scan_with_ap_for`)
  was rewritten the same day as a faithful `(Δf, Δt, b90)` grid search,
  which is also where the coarse-sync overhaul and a
  Lorentzian-vs-Gaussian fading-model fix (below) came from. That
  overhaul temporarily slowed the fading-metric golden tests
  (Q65-60D/120D/120E/300A) too, until the same score-distribution
  `max_candidates` calibration applied to Q65-60A closed the gap for
  all four — see the "Decode speed" notes above for the full per-row
  breakdown. Full writeup, including the real-`jt9` verification
  methodology: `Q65_BENCHMARK.md`. Dropped again, ~1.8-2× across every
  row that routes through `decode_multi_period_for`/
  `decode_scan_fading_for` (2026-08-08), via a redundant narrow-energy-
  extraction dedup — see the "Decode speed" section's own 2026-08-08
  note for the full per-row breakdown; that note also corrects this
  table's prior "Q65-60B ×1 slot" row label — the golden test always
  averaged all 3 files in the sample dir, never just one.

**AWGN sensitivity sweep** (`tests/q65_sim_sweep.rs`, `q65sim`-driven,
15 trials/SNR for the 15/30/60 s sub-modes, 5 trials/SNR for the
120/300 s ones — proportionally longer audio + the fine-timing retry
below multiply decode cost). Updated 2026-07-20 after the
`decode_scan_for`/`decode_scan_with_ap_for` rewrite (see
`Q65_BENCHMARK.md`); both columns are 50% crossings, approximate to
about ±0.5 dB given the sweep's SNR step granularity:

| Sub-mode | plain (`decode_scan_for`) | CQ-AP (`decode_scan_with_ap_for` + `"CQ"`) |
|----------|---------------------------:|--------------------------------------------:|
| Q65-15A  | ≈ −21.3 dB | ≈ −22.4 dB |
| Q65-30A  | ≈ −24.6 dB | ≈ −25.2 dB |
| Q65-60A  | ≈ −27.4 dB | ≈ −28.2 dB |
| Q65-60B  | ≈ −26.9 dB | ≈ −28.1 dB |
| Q65-60C  | ≈ −24.6 dB | ≈ −25.2 dB |
| Q65-60D  | ≈ −24.5 dB | ≈ −25.1 dB |
| Q65-60E  | ≈ −24.3 dB | ≈ −25.1 dB |
| Q65-120D | ≈ −25.5 dB | ≈ −27.3 dB |
| Q65-120E | ≈ −25.7 dB | ≈ −27.6 dB |
| Q65-300A | ≈ −33.6 dB | ≈ −34.5 dB |

- **CQ-AP-hinted path is the fair WSJT-X comparison, verified against a
  real `jt9` build.** WSJT-X's default `jt9` decode always has free
  access to the "CQ ??? ???" AP hypothesis, so every real `jt9` decode
  on CQ traffic implicitly gets AP-list benefit — a fair comparison
  needs this crate's `decode_scan_with_ap_for` + a `"CQ"` hint, not the
  plain baseline. Built `jt9` from WSJT-X source and ran it across the
  same `q65sim` AWGN corpus at fine SNR resolution. An initial pass
  found Q65-60C/D/E (`ibwa=8`, the widest fading-tolerance sub-modes)
  sitting ~2.5-3 dB short of `jt9`'s real crossing while Q65-60A/60B
  (`ibwa=1/3`) matched closely — traced to a real bug, not a residual
  algorithmic gap: `q65_dec1`/`q65_dec2` (`q65.f90:598,627`) both
  hardcode `nFadingModel=1` (Lorentzian), but `decode_at_grid_for` used
  Gaussian. Gaussian vs. Lorentzian barely differs at the narrow `b90`
  values Q65-60A/B ever reach, which is why they'd looked fine — but
  diverges sharply at the wide `b90` values (up to ~2 kHz) Q65-60C/D/E's
  full `ibw` sweep reaches. Switching to Lorentzian closed the gap: all
  ten sub-modes now sit within ~1 dB of `jt9`'s own crossing (Q65-60C
  mfsk-core ≈−25.2 dB vs. `jt9` ≈−25.4 dB; Q65-120E mfsk-core ≈−27.6 dB
  vs. `jt9` ≈−27.6 dB). See `Q65_BENCHMARK.md` for the investigation,
  including a false lead (an apparent sub-mode-specific cliff, actually
  a sparse-SNR-sampling artifact) it caught along the way.
- **Historical note (pre-2026-07-20 rewrite):** an earlier direct
  cross-check against a real `jt9 -3` build on Q65-120D/120E/300A, using
  the narrow-Bessel `decode_scan_for` this session replaced, found two
  sub-modes *exceeding* WSJT-X's own plain decode by 2.5-3.4 dB
  (Q65-120D ≈−28.2 dB, Q65-120E ≈−27.6 dB for `jt9 -3` vs. that
  implementation's ≈−30.8/−31.2 dB) because its unconditional
  time-retry gave it an advantage `jt9 -3` without `-c`/`-x` didn't
  have; Q65-300A matched `jt9` almost exactly. The rewrite (including
  the Lorentzian fix above) reproduces this closely — see the table
  above (Q65-120D ≈−27.3 dB, Q65-120E ≈−27.6 dB CQ-AP).

## MSK144

- **3/3 WSJT-X golden** across both `samples/MSK144/*.wav` recordings
  — message, frequency, timing, **and SNR** all gated (a systematic
  −1 dB SNR bias found post-ship was root-caused to a missing fixed
  1500 Hz-centered bandpass filter in the analytic-signal front end
  and fixed, closing the gap to an exact match).
- **AWGN sensitivity cross-validated directly against a real WSJT-X
  `jt9 -k` build** on `msk144sim`-generated signals: 25 of 28
  (ping-length × SNR) cells matched exactly, the other 3 differed by
  exactly 1 file out of 20 — no measurable recall gap at any tested
  SNR.
- MSK40 (the legacy shorthand mode) and the *adaptive* RX-equalizer
  training loop (as opposed to the fixed bandpass filter above, which
  is ported) remain out of scope.

**AWGN sensitivity sweep** (`tests/msk144_snr_sweep.rs`, self-contained
`msk144sim`-recipe synthesis, 20 seeds/SNR):

| SNR | Short ping (TRp=15s, width=0.12) | Long ping (TRp=30s, width=2.5) |
|---:|---:|---:|
| −9 dB | 0.0% | 0.0% |
| −8 dB | 0.0% | 0.0% |
| −7 dB | 0.0% | 0.0% |
| −6 dB | 40.0% | 0.0% |
| −5 dB | 90.0% | 60.0% |
| −4 dB | 100.0% | 100.0% |
| −3 dB | 100.0% | 100.0% |

50% crossing ≈ −5.8 dB (short ping) / −5.2 dB (long ping), both in
WSJT-X's 2500 Hz reference-bandwidth convention.

Reproduce: `docs/notes/MSK144_BENCHMARK.md`.
