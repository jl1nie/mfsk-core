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

1. **Act on the review of bc46e7a6** (unanchored `reviewer`, given the commit and CLAUDE.md, not a summary).
2. **One design note** for the three streamed entry points: what is guaranteed (equality with the slice / whole-spectrogram
   result, bit-equal where claimed), what is not (the (df, dt) grid, chirp passes, AP list, second sync: not streamed),
   the API layers (`FadingRow` -> `FastFadingStream` -> one candidate -> coarse search), and that the memory figures are
   derived, not measured on a target.
3. **Test gaps**: mutation check of `coarse_search_streamed` (band edges, drift columns, percentile admission);
   fixed-point run; audio shorter than one frame and bands at the spectrum edge; the sweep count against the band count.
4. **Not here**: on-device time and memory (the downstream owner measures), a single-sweep accumulator or b90 bank in this
   crate (the downstream decoder already has its own; revisit only when a user in this repository needs one).
