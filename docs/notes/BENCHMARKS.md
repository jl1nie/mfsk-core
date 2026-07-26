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
| FT8      | 8/8 host full-parity (WSJT-X), 18/18 (JTDX) | AWGN ≈ −21.4 dB (WSJT-X: −20 to −21 dB) | at/above parity |
| FT4      | 6/6 | AWGN ≈ −16.9 dB (WSJT-X: −17.5 dB, ~0.6 dB gap) | at parity |
| FST4     | 1/1 (FST4-60A only) | 0.10-0.60 dB across 5 sub-modes | at parity |
| WSPR     | 8/8 | AWGN 50% ≈ −29.8 dB, matches published sensitivity floor | at parity |
| JT9      | 7/7 | AWGN 50% ≈ −26.3 dB, no measurable gap vs. `jt9 -9` | at parity |
| JT65     | none available | **~7-8 dB** at deep SNR | known gap, deprioritized |
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
`core::sync::fine_sync_power_per_block`: FT8 moved again (see its own
row); every other row was confirmed unchanged within run-to-run noise
(≤~15%) **except FT4**, which turned out to already be stale for an
unrelated reason — see its row below.

**Compute environment**: AMD Ryzen 9 9900X (12C/24T), 32 GB RAM,
Ubuntu 24.04.2 LTS under WSL2 (kernel 6.6.87.2-microsoft-standard-WSL2),
rustc 1.97.1, `cargo test --release --features full` (includes the
`parallel` feature — some decode paths use `rayon` internally, so this
is wall time on a many-core host, not a single-thread figure).

| Protocol | Golden WAV | Slot length | Decode time |
|---|---|---:|---:|
| FT4 | 000000_000002.wav | 7.5 s | 0.11 s (was 0.049 s pre-#178-180, briefly regressed to ~0.53-0.58 s then ~0.28 s — see note) |
| Q65-60D | 201212_1838.wav (10 GHz EME, fading metric) | 60 s | 0.08 s |
| FT8 | qso3_busy.wav (16-signal busy band) | 15 s | 0.10 s |
| Q65-60B | 1296 MHz troposcatter ×1 slot (multi-period averaging) | 60 s | 0.12 s |
| Q65-120D | 210117_0920.wav (rainscatter, fading metric) | 120 s | 0.12 s |
| Q65-120E | 6 m ionoscatter (fading metric) | 120 s | 0.26 s |
| FST4-60A | 210115_0058.wav | 60 s | 0.27 s |
| JT9 | 130418_1742.wav | 60 s | 0.33 s |
| Q65-300A | 201210_0505.wav (optical scatter, fading metric) | 293.8 s | 0.34 s |
| Q65-30A | 6 m ionoscatter ×4 slots (multi-period averaging) | 4×30 s | 0.56 s |
| Q65-60A | 6 m EME (plain BP + AP) | 60 s | 0.69 s |
| MSK144 | 181211_120800.wav | 30 s | 0.84 s |
| MSK144 | 181211_120500.wav | 15 s | 0.88 s |
| WSPR | 150426_0918.wav | 120 s | 0.93 s |

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
  `sync8d.f90`/`ft8b.f90:133-140` algorithm) plus a `core::sync.rs`
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
  but `core::dsp::subtract::refine_freq` — its ±5 Hz/0.1 Hz carrier
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
  search (`core::sync2d::ft4_sync_search`) only ever produces
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
  `core::ft4_coarse::ft4_coarse_sync`, a faithful `getcandidates4.f90`
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

| Channel | mfsk-core 50% crossing | WSJT-X published | Gap |
|---|---:|---:|---:|
| AWGN | ≈ −21.4 dB | −20 to −21 dB | within range |
| CCIR good | ≈ −20.8 dB | — (no separate WSJT-X figure) | — |
| CCIR moderate | ≈ −18.9 dB | — | — |
| CCIR poor | ≈ −19.0 dB | — | — |

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
  **~139-148 ms** — ~7-8× faster than real `jt9 -8 -d3`'s own ~1.1 s
  total file decode time, at full recall parity. Distinct from the
  `0.10 s` ship-config row below, which is the real Core2/S3
  production path and floors at 7/8 by design.

## FT4

- **6/6 WSJT-X golden** (`samples/FT4/000000_000002.wav`), decoded in
  **~0.28 s** as of 2026-07-26 (briefly 0.049 s right after the
  `getcandidates4.f90`-port fix, up from 1.20 s before it — see
  "Decode speed" notes above and `FT4_BENCHMARK.md` section 13 — then a
  later LPF-subtract migration re-inflated it to ~0.53-0.58 s via a
  redundant-resynthesis bug in `refine_freq`, fixed same-day, issue
  #182; see the "Decode speed" table's own FT4 note for the full
  account).
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
300 AWGN spot-checked, also unchanged). See `FST4_BENCHMARK.md`
section 8 for the full investigation, including a first-attempt fix
that measured as a real ~0.5 dB regression and was corrected before
shipping.

Reproduce: `docs/notes/FST4_BENCHMARK.md`.

## WSPR

- **8/8 WSJT-X golden** (`samples/WSPR/150426_0918.wav`), ~0.88 s
  end-to-end on a desktop build — sub-bin demod + 2-pass
  subtract+re-coarse + OSD-2 fallback + Type-3 phantom filter.

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
  decode speed re-confirmed unchanged (0.33 s → 0.34 s, within noise).

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

## JT65 — known gap, deliberately not closed

- No real-recording golden WAV available: WSJT-X's own v3 reference
  samples need soft-symbol erasure metadata that lives in private
  WSJT-X branches.
- This crate's hard-decision `decode_at`/`decode_at_with_erasures` hits
  50% recall around **−14 dB** and near-zero below −19 dB, while
  WSJT-X's own no-`kvasd` path (`jt9 -6`) holds ~100% down to −22 dB on
  the identical corpus — a real **~7-8 dB gap**.
- **Root cause**: `jt9 -6` isn't plain hard-decision RS either —
  `lib/extract.f90` calls `ftrsdap` (`lib/ftrsd/ftrsdap.c`), a
  stochastic Chase decoder that runs many randomized soft-symbol
  erasure-pattern trials around Berlekamp-Massey RS, using both the
  most- and second-most-reliable symbol per position. This crate only
  tries a single deterministic increasing-erasure-count ordering — a
  materially weaker algorithm.
- **Deliberately deprioritized** ([#169](https://github.com/jl1nie/mfsk-core/issues/169)):
  Q65 already covers JT65's deep-SNR narrowband use case across a
  wider sensitivity range (10 wired sub-modes), and JT65 on-air
  traffic has largely migrated to it. Porting `ftrsdap` is a
  substantial, JT65-specific algorithmic port with no other payoff —
  not planned unless there's an actual request for deeper JT65 recall.

**AWGN sensitivity sweep** (`tests/jt65_sweep.rs`, `jt65sim`-driven,
20 trials/SNR):

| SNR | Recall |
|---:|---:|
| −25 dB | 0.0% |
| −22 dB | 0.0% |
| −20 dB | 0.0% |
| −19 dB | 0.0% |
| −18 dB | 5.0% |
| −17 dB | 15.0% |
| −16 dB | 30.0% |
| −15 dB | 25.0% |
| −14 dB | 50.0% |
| −12 dB | 45.0% |
| −10 dB | 60.0% |
| −5 dB | 80.0% |
| 0 dB | 100.0% |
| +5 dB | 100.0% |
| +10 dB | 100.0% |

(The −15 dB / −12 dB dips relative to their neighbors are 20-trial
sampling noise, not a non-monotonic decoder — visible on the steep
part of every sweep in this doc at this trial count.)

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
  methodology: `Q65_BENCHMARK.md`.

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
