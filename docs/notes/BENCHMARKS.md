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
| FT8      | 7/8 (WSJT-X), 17/18 (JTDX) | AWGN ≈ −20.8 dB (WSJT-X: −20 to −21 dB) | at parity |
| FT4      | 6/6 | AWGN ≈ −16.9 dB (WSJT-X: −17.5 dB, ~0.6 dB gap) | at parity |
| FST4     | 1/1 (FST4-60A only) | 0.10-0.60 dB across 5 sub-modes | at parity |
| WSPR     | 8/8 | AWGN 50% ≈ −29.8 dB, matches published sensitivity floor | at parity |
| JT9      | 5/5 | AWGN 50% ≈ −26.3 dB, no measurable gap vs. `jt9 -9` | at parity |
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
benchmark.

**Compute environment**: AMD Ryzen 9 9900X (12C/24T), 32 GB RAM,
Ubuntu 24.04.2 LTS under WSL2 (kernel 6.6.87.2-microsoft-standard-WSL2),
rustc 1.97.1, `cargo test --release --features full` (includes the
`parallel` feature — some decode paths use `rayon` internally, so this
is wall time on a many-core host, not a single-thread figure).

| Protocol | Golden WAV | Slot length | Decode time |
|---|---|---:|---:|
| FT4 | 000000_000002.wav | 7.5 s | 0.049 s |
| Q65-60D | 201212_1838.wav (10 GHz EME, fading metric) | 60 s | 0.08 s |
| Q65-120D | 210117_0920.wav (rainscatter, fading metric) | 120 s | 0.12 s |
| Q65-120E | 6 m ionoscatter (fading metric) | 120 s | 0.26 s |
| FST4-60A | 210115_0058.wav | 60 s | 0.27 s |
| Q65-60B | 1296 MHz troposcatter ×1 slot (multi-period averaging) | 60 s | 0.28 s |
| JT9 | 130418_1742.wav | 60 s | 0.33 s |
| Q65-300A | 201210_0505.wav (optical scatter, fading metric) | 293.8 s | 0.34 s |
| FT8 | qso3_busy.wav (16-signal busy band) | 15 s | 0.45 s |
| Q65-60A | 6 m EME (plain BP + AP) | 60 s | 0.69 s |
| Q65-30A | 6 m ionoscatter ×4 slots (multi-period averaging) | 4×30 s | 0.72 s |
| MSK144 | 181211_120800.wav | 30 s | 0.84 s |
| MSK144 | 181211_120500.wav | 15 s | 0.88 s |
| WSPR | 150426_0918.wav | 120 s | 0.93 s |

Notes:

- These are real-recording decode times, not synthetic sweeps — they
  reflect actual candidate density and search cost on real audio, not
  a clean-signal best case.
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

## FT8

- **WSJT-X 8-entry golden: 7/8** (`decode_frame_with_ap` host,
  `decode_block` embedded — same result both paths).
- **JTDX 18-entry golden: 17/18** (`decode_block`). The one miss,
  `WA2FZW DL5AXX`, is classified a likely JTDX false positive —
  `coarse_sync` candidates exist at its claimed frequency but no AP
  context recovers a message.
- **Host AP-on multipass JTDX-extras: 5/6** via
  `decode_frame_subtract_with_ap`. The remaining miss (`K1BZM DK8NE`,
  fixed AP context `mycall=K1JT`/`hiscall=HA0DU`) needs a wider
  AP-list / callsign hash table.
- **Embedded (M5StickS3, Xtensa LX7, fixed-point)**: 6/18 + 1 bonus =
  7 total on the same WAV, in ~1.19 s post-SlotEnd via the streaming
  pipeline. M5Stack Core2 (LX6): ~2.8 s.
- **SNR calibration**: `xsnr2_db_simple` lands reported SNR within
  ±3 dB of JTDX absolute on real silicon.
- CCIR moderate/poor fading recall gap closed in 0.7.3 by widening
  `OSD_HARDERRORS_MAX` back to WSJT-X's universal 36.

**AWGN/CCIR sensitivity sweep** (`tests/ft8_sweep.rs`, `ft8sim`-driven,
`-5` to `-26` dB grid, 4 channel conditions):

| Channel | mfsk-core 50% crossing | WSJT-X published | Gap |
|---|---:|---:|---:|
| AWGN | ≈ −20.8 dB | −20 to −21 dB | within range |
| CCIR good | ≈ −20.6 dB | — (no separate WSJT-X figure) | — |
| CCIR moderate | ≈ −18.6 dB | — | — |
| CCIR poor | ≈ −18.5 dB | — | — |

Reproduce: `docs/notes/FT8_BENCHMARK.md`.

## FT4

- **6/6 WSJT-X golden** (`samples/FT4/000000_000002.wav`), decoded in
  **0.049 s** (was 1.20 s — see "Decode speed" notes above and
  `FT4_BENCHMARK.md` section 13).
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

- **5/5 WSJT-X golden** (`samples/JT9/130418_1742.wav`) via the full
  WSJT-X-faithful softsym pipeline (`afc9` + `chkss2` + `xx0` mettab +
  `sync9` per-freq collapse).

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
