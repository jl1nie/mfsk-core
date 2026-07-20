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

## Q65-60A: deferred

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

A faithful fix means replacing `decode_scan_for`/
`decode_scan_with_ap_for`'s implementation with a new
WSJT-X-faithful `(Δf, Δt, b90)` grid search (reusing
`extract_data_energies_wide` + `intrinsics_fast_fading` + `Q65Codec`,
gated by a `GridDepth`-style parameter mirroring `ndepth`). This is a
larger, separately-scoped change: `decode_scan_for` is also the
baseline for `q65_sim_sweep.rs`'s AWGN sensitivity sweep across all 10
Q65 sub-modes' "plain" 50%-crossing figures (`BENCHMARKS.md`), so
landing it requires re-verifying all 10, not just Q65-60A's own golden
test — comparable in scope to a separate investigation. Tracked as a
follow-up, not attempted in this pass.
