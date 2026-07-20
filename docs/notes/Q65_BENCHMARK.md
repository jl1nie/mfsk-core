# Q65 decode-speed investigation

New investigation (2026-07-20), same "diagnose against WSJT-X reference,
profile, fix, verify no regression" methodology as
`FT4_BENCHMARK.md`/`FST4_BENCHMARK.md`'s speed sections. Prompted by
`BENCHMARKS.md`'s "Decode speed" table showing Q65-60A (1.57 s),
Q65-60B (1.89 s), Q65-30A (2.55 s) far slower than Q65-120D (0.15 s) /
Q65-60D (0.39 s) despite similar or shorter audio — counter-intuitive,
since 120D/60D use the fast-fading metric (presumably more expensive
per candidate) while 60A/60B/30A were assumed to be doing cheaper
"plain BP".

## Scope note

This investigation covers two of the three reported-slow sub-modes:
**Q65-60B and Q65-30A** (both routed through `decode_multi_period_for`).
**Q65-60A** (routed through `decode_scan_for`/`decode_scan_with_ap_for`)
has a different, larger root cause — see "Q65-60A: deferred" below —
and was deliberately left for a follow-up pass rather than bundled into
this one, given the size and blast radius (`decode_scan_for` is also
the baseline for `q65_sim_sweep.rs`'s AWGN sensitivity sweep across all
10 Q65 sub-modes).

## Root cause 1: redundant FFT extraction in the b90×model sweep

`decode_multi_period_for`'s per-candidate stage C tries the fast-fading
metric across `b90 ∈ {3, 8, 15} × model ∈ {Gaussian, Lorentzian}` (6
combinations). Each combination called `decode_averaged_fading_for`,
which internally called `averaged_data_energies_wide` — an FFT-based
extraction *and* multi-slot averaging step — from scratch, even though
extraction depends only on `(audio_slots, start_sample, base_freq_hz)`,
never on `b90`/`model`. All 6 combinations were re-doing bit-identical
extraction work.

**Fix**: split `decode_averaged_fading_for` into `averaged_data_energies_wide`
(unchanged, already existed) + a new `decode_fading_with_energies`
that takes pre-extracted energies. `decode_multi_period_for` now calls
`averaged_data_energies_wide` once per candidate and reuses the result
across all 6 sweep combinations. Pure refactor — same math, same
results, verified bit-identical (same messages, same frequencies, same
BP iteration counts) on both golden tests.

**Measured** (`q65_multi_period_speed_diag`, `tests/q65_wsjtx_samples.rs`):
Q65-60B 947.2 ms → 849.7 ms, Q65-30A 1654.1 ms → 1536.6 ms (~8-10%) —
real but modest; extraction was not the dominant cost.

## Root cause 2: candidate count, not per-candidate cost

Profiling (`q65_multi_period_candidate_count_diag`) found
`max_candidates=32` (the golden tests' own `SearchParams`, wider than
the library's own `SearchParams::default()` of `8`) hit and capped at
**every single slot** — all 4 ionoscatter slots for Q65-30A. Each of
those 32 candidates pays for the full decode ladder (AP-list + 6-way
fading sweep + plain Bessel fallback = up to 8 decode attempts) before
being accepted or rejected — the same "candidate explosion" pattern
FT4's coarse-sync investigation found, just in a different function.

**Calibration, not a guess** (`q65_candidate_score_calibration_diag`):
raised `max_candidates` to effectively unbounded (100 000) to see the
full score-sorted candidate list. Q65-30A's real signal (confirmed
golden message "K1JT K9AN R-16") ranked **#0 — the single highest-scoring
candidate — in all 4 slots**, with a healthy margin over the next tier
(e.g. slot 0: real signal scores 0.90-0.97, next distinct signal cluster
at 0.86 and below). This is real off-air data, not synthetic, so the
margin is meaningful evidence, not a guarantee for every future
recording — first cut `max_candidates` 32→16 (still 2× the library's own
default of 8), verified bit-identical recall, then re-profiled with
temporary per-stage timing (`Q65_MPF_DEBUG_TIMING` env-gated
instrumentation, added and reverted within this pass): the fast-fading
BP stage (up to 6 attempts/candidate) was still ~79% of total
wall-clock even after the extraction fix, confirming candidate *count*
— not per-candidate cost — remained the dominant lever. Given the same
#0-with-margin evidence, cut further to `max_candidates=8`, matching
`SearchParams::default()` exactly rather than an arbitrary number —
verified bit-identical recall again (same message, freq, dt, and — for
Q65-30A — the exact same BP iteration count, 37) before settling there.

**Measured**: golden-message recall bit-identical at every step (same
message, frequency, dt, BP iteration count throughout the 32→16→8
progression, confirming the identical candidate was found and decoded
each time).

**Combined result** (both fixes, real golden-test wall-clock, matching
`BENCHMARKS.md`'s methodology):

| Sub-mode | Before | After | Speedup |
|---|---:|---:|---:|
| Q65-60B | 1.89 s | 0.49 s | ~3.9× |
| Q65-30A | 2.55 s | 0.64 s | ~4.0× |

Full non-ignored suite (922 tests) and `-D clippy::perf -D warnings`
green throughout. `q65_sim_sweep.rs` (the AWGN sensitivity sweep) does
not call `decode_multi_period_for` at all, so this change has zero
blast radius there.

## Q65-60A: `(Δf, Δt, b90)` grid-search rewrite (2026-07-20)

Profiling (`q65_speed_diag_coarse_vs_finetiming`) found coarse search
itself cheap (~10-13 ms) for both Q65-60A and Q65-60D; the cost gap is
per-candidate: Q65-60D (fading path) ~3.7 ms/candidate vs Q65-60A
(plain path) ~24 ms/candidate. Root cause, confirmed against WSJT-X's
actual reference (`lib/qra/q65/q65_loops.f90`): WSJT-X has **no**
separate "plain BP" code path for Q65 at all — `q65_dec2` always calls
the fast-fading intrinsics (`q65_intrinsics_ff`), swept over a
submode-specific `b90` range (`ibwa` table in `q65_decode.f90:168-178`:
A=1, B=3, C/D/E=8, `ibwb=min(15,ibwa+6)`), combined with a
distance-pruned `(Δf, Δt, b90)` grid whose depth is gated by `ndepth`
(shallow/`Fast` = single-shot no retry even at low depth; the full 5×5
grid is explicitly commented `"Use 'Deep' for manual Q65 decodes"`,
i.e. never the automatic per-slot default).

Our `decode_scan_for`/`decode_scan_with_ap_for` diverged from this into
a **narrow-window, AWGN-only Bessel metric** wrapped in a **time-only**
±3-step retry (`decode_at_with_fine_timing_for`) — missing the
frequency-offset dimension, missing the b90 sweep entirely, and paying
up to 7 full decode attempts per candidate regardless of how far that
candidate is from a real signal.

### The fix: `decode_at_grid_for`

`decode_at_with_fine_timing_for` now calls `decode_at_grid_for`, a new
WSJT-X-faithful `(Δf, Δt, b90)` grid search (`src/q65/rx.rs`) reusing
`extract_data_energies_wide` + `intrinsics_fast_fading` + `Q65Codec`,
gated by a `GridDepth` enum mirroring `ndepth`'s bit field
(`Fast`/`Normal`/`Deep` ↔ `iand(ndepth,3)` 0-1/2/3). Only `Fast` is
wired in — WSJT-X's own automatic per-slot decode always runs at Fast
(confirmed independently: `jt9`'s CLI default is `-d 1`, which maps to
the same `iand(ndepth,3)!=2,3` branch).

Landing this surfaced two real, WSJT-X-faithful bugs the naive port
missed:

1. **Missing the full, unpruned `ibw` sweep at the origin cell.**
   `q65_loops.f90`'s `(Δf,Δt,b90)` grid — the only thing a literal
   reading of that file describes — prunes `ibw` by
   `ndist=ndf²+ndt²+(ibw-ibw0)²≤maxdist` at *every* cell including
   `(Δf,Δt)=(0,0)`, which for `Fast` (`maxdist=4`) excludes `ibw` more
   than 2 away from `ibw0=(ibwa+ibwb)/2` — for wide-`ibwa` sub-modes
   (C/D/E, `ibwa=8..14`, `ibw0=11`) this silently drops `ibw∈{8,9,14}`,
   including the *lowest* (least-fading-tolerant, best-for-pure-AWGN)
   end. But `q65_loops` is WSJT-X's **fallback** stage — `q65_dec0`
   calls `q65_dec_q012` (`lib/qra/q65/q65.f90:381`) *first*, at the
   sync-detected position with **no** `Δf`/`Δt` retry but the **full,
   unpruned** `ibwa..ibwb` sweep, and only falls through to
   `q65_loops` if that fails. `decode_at_grid_for` now matches this:
   the ibw loop is unpruned when `(Δf,Δt)=(0,0)`, pruned by `maxdist`
   otherwise (also added the retry-only `b90>345` cap from
   `q65_loops.f90:73`, absent from `q65_dec_q012`).
2. **Coarse-sync time resolution 4× coarser than WSJT-X's own.**
   `Spectrogram::build_for` (`src/q65/search.rs`) stepped the sync
   spectrogram at `nsps/2` — half a symbol — against WSJT-X's own
   `NSTEP=8` (`lib/qra/q65/q65.f90:3`, "Number of time bins per symbol
   in s1, s1a, s1b"). Widening to `nsps/8` alone regressed a real
   off-air multi-signal recording (`ionoscatter_6m_120e_decodes_with_
   fading_metric` stopped finding `KB7IJ N0AN`) — the finer resolution
   let a single true peak's neighbourhood flood the
   `max_candidates`-truncated list with near-duplicate rows, crowding
   out a distinct weaker signal. Fixed by restructuring
   `coarse_search_on_spec_for` to match `q65_ccf_22`'s own shape
   (`lib/qra/q65/q65.f90:506-574`): collapse over time *first* per
   frequency bin (keep only the best-scoring row, like `ccf2(i)=max
   over lag,idrift`), apply frequency-domain local-max suppression
   (`±mode_q65` bins, i.e. `±bins_per_tone`), and admit candidates via
   a **noise-adaptive** threshold — 50th/84th percentile of the
   per-frequency score curve → `rms`, admit if `(score-ave)/rms≥6.0`,
   OR'd with the pre-existing fixed `score_threshold` so a clean/strong
   signal is never rejected by a percentile baseline distorted by
   something atypical (e.g. a hard on/off edge in a synthetic test
   buffer). This is additive engineering on top of a literal
   `q65_ccf_22` read, not itself directly from the Fortran, but every
   individual piece (time-collapse, frequency NMS, percentile
   admission) mirrors a specific line range there.

### Verification: real `jt9`, and a false lead it caught

Per this project's "diagnose against WSJT-X reference" discipline, the
above wasn't shipped on code-reading alone. Built `jt9` from WSJT-X
source (`~/wsjtx-build/jt9`) and ran it (`-3 -p <period> -b <letter>
-d 1`, matching `GridDepth::Fast`) across the same `q65sim` AWGN
corpus used by `q65_sim_sweep.rs`, generating extra fine-grained SNR
trial points (1 dB steps) to resolve crossings precisely.

First pass (coarse SNR sampling) showed what looked like a
sub-mode-specific catastrophic bug: Q65-60C/60E appeared to fail
*completely* down to -34 dB and then jump straight to ~90%+ at -21 dB
— a step function, not the smooth sigmoid every other sub-mode showed
— while Q65-60D looked comparatively fine. Tracing this (candidate
dumps, per-decode debug instrumentation, a controlled clean-signal
test) ruled out every mechanism it was tempting to blame: `es_no`
metric (confirmed a *fixed per-codec constant* in the real C source,
not a bug), the coarse-search restructure above (candidate dump showed
it finds the correct `(freq, start_sample)` with an near-identical
score for both a "broken" and a "working" sub-mode), window-index
overflow in `intrinsics_fast_fading` (checked the exact bin arithmetic
against `GAUSS_LEN`'s real max of 65 — nowhere close to overflowing
even the narrowest sub-mode's window), and a structural extraction bug
(a controlled *clean*, zero-noise synthetic signal decoded correctly
through the identical grid-search code path for every sub-mode tested).
**The "cliff" turned out to be the same sparse-SNR-sampling artifact
this investigation had already found once this session** (in the
`q65_sim_sweep.rs` corpus's own fixed offset grid) — the existing
1 dB-step corpus simply didn't have trial points in the 3 dB gap where
the *new* crossing landed for the wide-`ibwa` sub-modes, so the true
(smooth) curve was invisible. Generating trials at the missing SNR
points confirmed Q65-60C/60D/60E all behave consistently and smoothly.

With that resolved, the real (smaller, coherent) picture: sub-modes
with `ibwa=1` or `3` (A, B) now track real `jt9` closely — e.g.
Q65-60B's CQ-AP crossing matches `jt9`'s own to within the sweep's own
±0.5 dB granularity. Sub-modes with `ibwa=8` (C, D, E, plus
Q65-120D/120E which share the same letters) sat a real ~2.5-3 dB behind
`jt9`.

Two hypotheses for this residual gap were checked against the actual
`decoder.f90`/`q65.f90` source and **ruled out**: (1) WSJT-X's
`q65_dec_q3` full-AP-list stage (`q65_ccf_85`, an 85-symbol
cross-correlation) running before `q65_dec_q012` — traced the call
chain (`decoder.f90:210` hardcodes `nqd=1` on every decode, which
looked promising, but `q65_set_list.f90:14` bails with `ncw=0`
immediately when `hiscall` is blank, which it is for a bare `jt9 -3`
invocation with no `-x`, so this path never actually activates for the
comparison being made); (2) `q65_ccf_22`'s drift dimension
(`max_drift`) — this is a caller-supplied argument
(`decoder.f90:214`/`q65_decode.f90:34`, ultimately a GUI/CLI option),
not something `jt9 -3` enables by default either.

**The real cause**: `q65_dec1` and `q65_dec2` (`q65.f90:598`, `:627`
— the two entry points both `q65_dec_q012` and `q65_loops` call) each
hardcode `nFadingModel=1` locally, i.e. **every real Q65 decode always
uses the Lorentzian fading model**, never Gaussian.
`decode_at_grid_for` used `FadingModel::Gaussian`. Gaussian and
Lorentzian barely differ at the narrow `b90` values Q65-60A/B's `ibw`
sweep ever reaches (`ibwa=1..7`, max `b90≈44 Hz`) — which is exactly
why those sub-modes had looked fine — but diverge sharply at the wide
`b90` values Q65-60C/D/E's full sweep reaches (`ibwa=8..14`, up to
`b90≈2 kHz`). Switching to `FadingModel::Lorentzian` closed the gap:
all ten sub-modes now sit within ~1 dB of `jt9`'s own crossing (e.g.
Q65-60C mfsk-core ≈−25.2 dB CQ-AP vs. `jt9` ≈−25.4 dB; Q65-120E
mfsk-core ≈−27.6 dB vs. `jt9`'s previously-documented ≈−27.6 dB almost
exactly). See `BENCHMARKS.md`'s Q65 section for the full table.

### Outcome

- Golden-WAV Q65-60A decode: 1.57 s → 1.49 s (modest — the cost moved
  from candidate-count to `intrinsics_fast_fading` calls, so the
  rewrite's speed goal doesn't show up as a dramatic number here) —
  but real-recording recall *improved*: `eme_6m_sample_yields_decode_
  with_ap` now recovers 4 messages instead of 3 (`W7GJ N0TB -15`,
  previously missed).
- Every real off-air Q65 golden test (`q65_wsjtx_samples.rs`, including
  the ionoscatter/tropo/rainscatter/EME/optical-scatter recordings)
  passes; full non-ignored suite (948 tests) and
  `cargo clippy --workspace --all-targets --features full -- -D
  warnings` clean throughout.
- AWGN sensitivity: see `BENCHMARKS.md`'s Q65 section for the full
  before/after/jt9 table.
