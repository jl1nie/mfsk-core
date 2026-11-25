# Q65 streaming: what is left to make the shipped code a designed whole

Mode: design. Status: **parked** (2026-11-25), to be taken after the FT8 / FT4 work.

The streamed Q65 pieces are committed and tested against their slice / whole-spectrogram twins, but they were built stage
by stage (2a-2c) without a named user: the one downstream decoder looked at (a no_std Q65-60D decoder with its own
single-sweep accumulator and metric bank) needs only `FadingRow`, not these. They stay, and are to be finished as a unit.

| piece | commit | state |
|---|---|---|
| `FadingRow` (per-row metric block) | e0ac38ef | reviewed, mutation-checked, CI pending at the time of writing |
| `FastFadingStream` (on `FadingRow`) | 72f6fea4 + e0ac38ef | reviewed, CI passed |
| `q65::decode_candidate_streamed` | ee6da979, 72f6fea4 | reviewed, CI passed |
| `q65::search::coarse_search_streamed` | bc46e7a6 | CI passed; **independent review not yet acted on** (one was started and not read) |

## To do

1. **Act on the review of bc46e7a6** (unanchored `reviewer`; findings below, none fixed yet).
2. **One design note** for the three streamed entry points: what is guaranteed (equality with the slice / whole-spectrogram
   result, bit-equal where claimed), what is not (the (df, dt) grid, chirp passes, AP list, second sync: not streamed),
   the API layers (`FadingRow` -> `FastFadingStream` -> one candidate -> coarse search), and that the memory figures are
   derived, not measured on a target.
3. **Test gaps**: mutation check of `coarse_search_streamed` (band edges, drift columns, percentile admission);
   fixed-point run; audio shorter than one frame and bands at the spectrum edge; the sweep count against the band count.
4. **Not here**: on-device time and memory (the downstream owner measures), a single-sweep accumulator or b90 bank in this
   crate (the downstream decoder already has its own; revisit only when a user in this repository needs one).

## Findings of the review of bc46e7a6 (read, not yet acted on; each was checked by the reviewer by reading, not re-measured)

1. **Scope not stated** (medium-high): for sub-modes B-E the scan runs a second sync on the `nsmo`-smoothed spectrogram
   (`scan_with`, `smoothed_for_sync`); `coarse_search_streamed` reproduces only the first, unsmoothed list. State it in the
   doc comment and both LIBRARY twins (§2.1), or smooth a band widened by `nsmo + max_drift` in place. The real-recording
   test runs Q65-60D only.
2. **Memory floor understated** (medium): "FFT buffer is the floor, 330 KB at 300A" omits `raw`, the `Window` cache,
   `curve` / `rows` / `drifts` and the FFT plan: about 650 KB before the plan. The same in `ICAP_EMBEDDED_ANALYSIS.md:197`
   ("about 170 KB at 16 bands for 30A" is `power` only).
3. **Stale sentences** contradicted by the commit: "the coarse search ... is not streamed" in LIBRARY.md, LIBRARY.ja.md
   and CHANGELOG (the 2b entry), each above the paragraph saying the opposite.
4. **"47 candidates" overstated**: one candidate counted three times for 10 GHz, and 44 for 6 m is from one file; the
   several-candidate percentile case rests on one recording.
5. **Speed of the default path unmeasured**: `coarse_search_drift_on_spec_for` now goes through the `SyncColumns` trait
   without `#[inline]`; `engine/search.rs` records +25 % from exactly that omission. Measure A/B back-to-back (worktree).
6. `DECODE_CANDIDATE.md` now says both "passed on CI ubuntu" and, below it, "not verified: other platforms ... in CI".
7. Figures hold for 30A only ("793 FFTs a sweep": 713 at 120 s, 687 at 300 s); the drift margin read is about `max_drift/2`,
   the margin kept is `max_drift` (up to 50 % more memory; tests cannot tell the two apart).
8. Test gaps: the `fb_top` cut and `hi.min(n_freq-1)` clamp never bind; the sweep-count bound (20..=40 for 28 columns) is loose;
   audio between one frame and 85 symbols untested.
9. API: `max_drift: u32` and `bands: usize` swap silently with integer literals; no helper from a byte budget to `bands`;
   `z + max_drift_bins` overflows on 32-bit `usize` for huge `max_drift` (debug panic).

Not checked by the reviewer: verify.sh log, the ten mutations, timing, fixed-point / `fft-extern`, embedded targets.
