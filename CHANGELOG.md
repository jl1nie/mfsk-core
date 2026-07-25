# Changelog

## 0.8.0 — JT65 decode-chain bug fix (#24) + JT9 AWGN SNR sweep + Q65-15A/120D/120E/300A + fine-timing sensitivity fix + CQ-AP-hint parity note (#171) + BASIS removal (#162, breaking FFI change)

### Added

- **`ft8::decode::decode_frame_subtract_staged` / `_with_ap`** (issue
  #180) — WSJT-X-faithful checkpoint SIC for FT8, ported from
  instrumented `jt9.f90`/`ft8_decode.f90` ground truth rather than the
  issue's own initial paraphrase. `jt9`'s disk-file FT8 decode is not a
  single pass over the 15 s slot: it decodes progressively larger audio
  *prefixes* at three checkpoints (0..141_696 / 0..162_432 / 0..172_800
  samples), carrying decoded signals forward and subtracting them from
  the residual *before* the final, hardest checkpoint's candidates are
  attempted — something the pre-existing flat 3-pass
  `decode_frame_subtract` structurally cannot reproduce regardless of
  threshold tuning, since nothing is ever subtracted before the
  residual a marginal candidate is found in gets assembled. Verified
  no regression vs the flat pass on all 3 real reference WAVs
  (`tests/ft8_qso3_staged_sic_check.rs`, `191111_110130.wav`,
  `191111_110200.wav`) — identical decode sets on each. The specific
  case that motivated #180 (`CQ DX DL8YHR JO41`, ~-17 dB on
  `qso3_busy.wav`) still does not decode even with the correct
  checkpoint architecture in place (checkpoint A finds 11 signals
  early, including its closest neighbour `W1FC` only ~35 Hz away,
  correctly subtracted before checkpoint C runs).

  **Root-caused, not just reproduced.** Re-ran real `jt9 -8 -d3`
  (rebuilt from the local `WSJT-X` checkout, `build_jt9/jt9`, on the
  byte-identical `samples/FT8/210703_133430.wav`) to get fresh ground
  truth instead of relying on the issue's prior transcript: at
  DL8YHR's true coordinates jt9's own per-Costas-block breakdown is
  `is1=2 is2=7 is3=6` → `nsync=15`. A new diagnostic
  (`ft8::decode::tests::issue_180_dl8yhr_staged_checkpoint_c_probe`,
  using `decode_frame_subtract_staged_with_ap_debug_residual` to
  reproduce the same per-block count against mfsk-core's *actual*
  production checkpoint-C residual) found:
  - **Raw, unsubtracted audio**: `is1=0 is2=3 is3=3` → `nsync=6`.
  - **Staged checkpoint-C residual** (all 11 early signals cleanly
    subtracted): `is1=1 is2=4 is3=4` → `nsync=9`.
  - A ±15 ms (±3-sample) `dt` sweep around the ground truth only ever
    moved the total between 8 and 10 — the off-by-one dt-convention
    hypothesis (#180 "Bug 1") explains at most ~1 of the 6-point
    shortfall, not the bulk of it.

  Two conclusions from the aggregate counts: (1) subtraction/SIC
  quality **is** doing real work (+3 points, raw→residual) — the
  staged architecture isn't wasted effort; (2) the *remaining* 6-point
  gap barely moves under a ±15 ms timing sweep (nsync stayed in
  8..10), ruling out the off-by-one dt-convention hypothesis (#180
  "Bug 1") as the dominant cause.

  **Symbol-level dump pinpoints the actual mechanism.** Re-instrumented
  `ft8b.f90` to also emit its raw 8-tone `s8` magnitudes for Costas
  blocks 2 and 3 (only block 1 was previously dumped), rebuilt `jt9`,
  and diffed symbol-by-symbol against mfsk-core's own tone magnitudes
  at the identical `(f1, dt)`, on both raw and staged-residual audio.
  Finding: at symbol positions **t=5 and t=6 of every 7-symbol Costas
  block**, mfsk-core computes a large spurious energy spike at **tone
  0** (DL8YHR's own base carrier frequency, 2606.25 Hz) that swamps
  the true, weak Costas tone — e.g. block 2 t=5: mfsk-core residual
  tone0=9.7 vs true tone6=0.2 (raw audio: tone0=42.0 vs tone6=2.7).
  Critically, **jt9's own real-signal data shows this same tone-0
  artifact only in block 1** (where it explains jt9's own weak
  `is1=2`) — its block-2/3 t=5,6 values have the *true* Costas tone
  clearly dominant (e.g. block 2 t=5: tone6=2641 vs tone0=1140; t=6:
  tone5=3570 vs tone0=1584). mfsk-core reproduces the artifact
  correctly where it's real (block 1) but *also* manufactures it where
  the real signal has none (blocks 2 and 3) — a structural,
  position-periodic pattern (always t=5,6 of a block, at three
  absolute times spread across the ~13 s message), not diffuse
  numeric imprecision.

  **Raw baseband (`cd0`) comparison localises the mechanism further —
  and revises the diagnosis.** Re-instrumented `ft8b.f90` a second
  time to dump its raw complex `cd0` (200 sps downsampled baseband)
  samples over the exact 64-sample window spanning block-1's t=5/t=6
  (Fortran index 268..331, i.e. `ibest+128 .. ibest+191`), rebuilt
  `jt9`, and diffed those against mfsk-core's own `cd0` at the
  identical physical samples (0-indexed `n` == Fortran's 1-indexed
  `n+1` — confirmed aligned, not an indexing bug: values track the
  same sign and shape throughout, ruling out a real off-by-one at the
  `cd0` level). The magnitudes are not just noisier — mfsk-core's
  `|cd0|` runs **2-4× (6-12 dB) larger than jt9's own post-subtraction
  `|cd0|` at the same samples**, most pronounced exactly over Fortran
  idx 280-327 (e.g. idx 285: jt9 `|cd0|`=205 vs mfsk-core 632; idx
  291: jt9 121 vs mfsk-core 597). That's not a tone-detection/FFT
  computation bug — it means **mfsk-core's residual genuinely has
  more uncancelled signal energy left in it than jt9's residual does**,
  at the same point in time and frequency, even though both have
  subtracted an overlapping set of interferers by this point.

  **Isolated (a) [subtraction quality] from (b) [a missing
  interferer], and ruled out (b).** Re-instrumented `ft8_decode.f90` a
  third time to dump `f1_save`/`xdt_save`/`allmessages(1:ndec_early)`
  — jt9's *actual* list of the 13 signals subtracted before the
  nzhsym=50 pass — rather than inferring it from stdout decode-print
  order. jt9's 13 match mfsk-core's 18-message final set on 12 calls,
  but include a 13th, `WA2FZW DL5AXX RR73` @ 2545.88 Hz (only 60 Hz
  from DL8YHR's own 2606.25 Hz carrier) that **mfsk-core never decodes
  anywhere** in this investigation (not in either checkpoint's results,
  not in the flat-pass baseline) — a real, plausible candidate for
  hypothesis (b): an unsubtracted nearby signal leaking into DL8YHR's
  band. Tested directly: manually built a `DecodeResult` from jt9's own
  reported `WA2FZW` coordinates and subtracted it from the staged
  residual with the same `subtract_signal_lpf` call production uses.
  Result: **no change** — `is1=1 is2=4 is3=4, nsync=9`, identical to
  before the subtract. Hypothesis (b) is ruled out for this specific
  signal (its own contribution isn't what's showing up as excess `cd0`
  energy near DL8YHR — either the reconstructed/refined waveform
  doesn't fit what's actually there, or the energy comes from
  elsewhere entirely). That left hypothesis (a) — `subtract_tones_lpf`
  vs `subtractft8.f90`'s actual per-signal cancellation depth on the
  *already-subtracted* candidates — as the standing lead.

  **Confirmed definitively: hypothesis (a), 100% subtraction quality,
  zero mfsk-core tone-detection bug.** The decisive test: swap the
  residual, keep mfsk-core's algorithm untouched. Re-instrumented
  `ft8_decode.f90` a fourth time to dump jt9's actual `dd` array —
  raw i16, taken right after jt9's real SIC (its 13 subtracted
  signals) and right before its own `nzhsym=50` `sync8`/`ft8b` search
  — to `/tmp/jt9_post_sic_dd.raw` (180,000 samples, sanity-checked:
  RMS 115, ±577 range, consistent with a heavily-subtracted residual).
  Loaded that exact buffer into mfsk-core and ran its *unmodified*
  `symbol_spectra_direct`/`sync_quality` at DL8YHR's coordinates:
  **`is1=2 is2=7 is3=6` → `nsync=15`** — bit-for-bit the same
  per-block breakdown jt9 itself reports on this buffer. Then ran the
  full BP/OSD chain (`compute_llr` → `bp_decode`/`osd_decode_*`) on
  the same residual: **decodes `"CQ DX DL8YHR JO41"` outright.**
  mfsk-core's downsample → per-symbol tone extraction → LLR → BP/OSD
  pipeline has no bug at all with respect to this signal — given the
  same clean residual WSJT-X itself decodes from, it decodes it too,
  byte-for-byte matching sync counts. The *entire* gap between
  mfsk-core's own decode (`nsync=9`, no decode) and jt9's (`nsync=15`,
  decodes) traces to one place: **mfsk-core's own SIC subtraction
  leaves more residual interference behind than `subtractft8.f90`
  does**, on the exact same set of already-decoded interfering
  signals. The next concrete, scoped step for #180 is a direct
  `subtract_tones_lpf` vs `subtractft8.f90` per-signal suppression-dB
  comparison (GFSK waveform reconstruction fidelity, LPF width/shape,
  QSB/fading-envelope tracking) — not sync-quality computation, not
  pipeline architecture, not a missing candidate. All diagnostics
  (per-block breakdown, `cd0` dump, WA2FZW confirmation, residual-swap
  confirmation) kept in
  `ft8::decode::tests::issue_180_dl8yhr_staged_checkpoint_c_probe`.
  As of the follow-up entry below, `decode_frame_subtract_with_ap`
  (and therefore `decode_frame_subtract`) delegates to this staged
  checkpoint SIC by default; the pre-#180 flat 3-pass behaviour is
  still available via `decode_frame_subtract_flat_with_ap` for callers
  that specifically want it.
  **Cost**: checkpoint A runs its own full 3-sub-pass search on top of
  checkpoint C's, so this is strictly more decode work than the flat
  pass. Measured (release, `--features full`, 10 reps, host, real
  reference WAVs): 1.31–1.87× the flat pass's wall-clock
  (`191111_110130.wav` 231ms→303ms, `191111_110200.wav`
  249ms→460ms, `qso3_busy.wav` 388ms→611ms).

- **`core::dsp::subtract::subtract_tones_lpf_refine_dt` / FT8's
  `subtract_signal_lpf_refine_dt`** (issue #180 follow-up) — closes
  part of the "next concrete step" left open above: a direct
  `subtract_tones_lpf` vs `subtractft8.f90` comparison found the two
  algorithms structurally identical (GFSK waveform synthesis, cos²
  LPF kernel, end-correction — all line-for-line matches), *except*
  for one missing feature: `subtractft8.f90` takes an `lrefinedt`
  argument, and `ft8_decode.f90`'s two early-decode-and-subtract
  checkpoints (lines 132, 162 — exactly the calls
  `decode_frame_subtract_staged` above is modelling) pass
  `lrefinedt=.true.`. That path (`subtractft8.f90`'s internal `sqf`/
  `peakup`) re-searches `dt` by ±90 samples (±7.5 ms @ 12 kHz) around
  the candidate's own value, scoring each trial by the residual power
  left in the signal's own tone band after subtraction, and commits
  the subtraction at the parabolic-fit minimum — rather than trusting
  the early candidate's own (still-coarse) `dt` outright. mfsk-core's
  staged-checkpoint SIC had no equivalent: every early subtract used
  the plain, non-refining `subtract_signal_lpf`. Ported `sqf`/`peakup`
  faithfully (`core::dsp::subtract::fft_lpf::subtract_tones_lpf_refine_dt_fft`)
  and wired it into `decode_frame_subtract_staged_with_ap_inner`'s
  checkpoint-B and checkpoint-C deferred subtracts (the flat 3-pass
  `sic_inner_passes`'s single confirmed-decode subtract is unchanged —
  matches `ft8b.f90:474`'s `lrefinedt=.false.`).
  **Effect on the DL8YHR case**: checkpoint-C's residual `sync_quality`
  at WSJT-X's own ground-truth coordinates improved from `nsync=9` to
  `nsync=10` (jt9 itself gets `nsync=15` on this signal) — a real,
  measured reduction in leftover interference from the same 13
  already-subtracted signals, though not yet enough on its own to
  flip this specific -17 dB decode (`any_hit` still `false` in
  `issue_180_dl8yhr_staged_checkpoint_c_probe`). No regression on any
  of the 18 golden `qso3_busy.wav` decodes or any other reference WAV
  (`ft8_qso3_staged_sic_check.rs`, `ft8_qso3_subtract_fix_check.rs`,
  `ft8_qso3_apoff_recall.rs`, `ft8_qso3_apon_recall.rs`, full
  workspace `cargo test --release --features full`: 100% pass). New
  regression test:
  `core::dsp::subtract::refine_dt_tests::refine_dt_beats_plain_subtract_when_dt_is_off`.
  Remaining gap (candidates per the prior comment: LPF width/shape,
  QSB/fading-envelope tracking, or f32 vs Fortran-single-precision
  accumulation differences over the 151_680-sample reference) is still
  open — logged here rather than pursued further in this pass.

- **`core::dsp::subtract::fft_lpf::normalized_kernel` LPF window shape
  bug — root cause of the DL8YHR gap, issue #180 closed.** Auditing
  the "LPF width/shape" item left open above (per the entry directly
  above this one) against `subtractft8.f90`/`subtractft4.f90` found the
  actual bug: `window(j) = cos(pi*j/NFILT)**2` in WSJT-X (`NFILT =
  2*lpf_half`) had been ported as `cos((j - lpf_half) * pi / lpf_half)²`
  — dividing by `lpf_half` instead of `NFILT` (`2*lpf_half`), doubling
  the cosine's argument range from `[-pi/2, pi/2]` to `[-pi, pi]`.
  `cos²` is monotonic (a proper single taper, 1 at the centre → 0 at
  the edges) only over the first range; over the doubled range it dips
  to 0 at the quarter points and rises back to **1 — full weight — at
  the true edges** (`lpf_half` samples / 166 ms away from the centre
  sample, for FT8's `lpf_half=2000`). Verified numerically: at
  offset=2000, shipped kernel=1.000 vs correct=0.000; at offset=1000,
  shipped=0.000 vs correct=0.500. So every `subtract_tones_lpf` /
  `subtract_signal_lpf` call since this became the canonical FT8/FT4
  SIC entry point (v0.6.2) had been running a badly-misshapen "lowpass"
  that gave 166-ms-stale channel samples as much weight as the current
  one — actively corrupting the complex-amplitude/QSB estimate instead
  of smoothing it. Root cause of the residual DL8YHR gap the two
  entries above this one were chasing (`refine_signal_freq`'s dt-search
  only got `nsync` from 9 to 10 on top of this broken window).
  One-line-formula fix, shared by both the FFT-cached path
  (`fft_lpf::normalized_kernel`, feeds both `cached_window_fft` and
  `end_correction`) and the `no_std` direct-convolution fallback
  (`subtract_tones_lpf_direct`) — fixing both fixes FT4 subtract too
  (`subtractft4.f90` uses the identical window formula, confirmed by
  inspection; FT4's own recall floor is unaffected — see regression
  results below). **Result: `CQ DX DL8YHR JO41` now decodes**
  (`decode_frame_subtract_staged`'s `qso3_busy.wav` golden-set test,
  `staged_sic_matches_flat_pass_golden_floor`: still 18/18 golden hits,
  0 phantoms in that set, unique-decode count 19→20, `has_dl8yhr` flips
  `false`→`true`) — issue #180 is closed. Full workspace `cargo test
  --release --features full` (all 70 test binaries): 100% pass, no regression on
  any golden decode set (FT8 or FT4).

- **`decode_frame_subtract_with_ap` now delegates to the staged
  checkpoint SIC by default** (issue #180 follow-up) — the flat 3-pass
  behaviour is a strict recall subset (verified: identical golden-set
  hits on every reference WAV, plus `CQ DX DL8YHR JO41` on
  `qso3_busy.wav`, which the flat loop structurally cannot reach) at
  1.3-1.9× the flat pass's wall-clock, still comfortably inside FT8's
  15 s slot budget on host. The old flat behaviour remains available
  as its own entry point, `decode_frame_subtract_flat_with_ap`, for
  callers that specifically want the cheaper non-staged path.
  **Fixed a stack-overflow regression this introduced**: the staged
  path's own two fallback branches (buffer shorter than checkpoint A;
  checkpoint A finds nothing) used to fall back to
  `decode_frame_subtract_with_ap` — which, after this change, calls
  right back into the staged path, recursing indefinitely. Both now
  fall back to `decode_frame_subtract_flat_with_ap` instead. Caught by
  the full-workspace regression run
  (`decode_frame_subtract_with_ap_silence_shape`, a short-buffer test)
  before release.

- **`decode_frame_subtract_with_auto_ap`** (`fft-rustfft` only) —
  `decode_frame_subtract_with_ap` plus a final rescue pass: harvest
  caller callsigns from the blind decodes and retry `coarse_sync`
  candidates that didn't decode blind, using each harvested callsign
  as an AP `mycall` hint. Recovers signals too weak for blind BP/OSD
  but not too weak to sync — e.g. `K1BZM DK8NE -10` (-19 dB) on
  `qso3_busy.wav`. **This is a genuine mfsk-core-original extension,
  not a port of any real jt9/WSJT-X mechanism** — checked directly
  against `ft8apset.f90`'s actual source: real WSJT-X only applies AP
  using the *operator-configured* `mycall`/`hiscall` and disables AP
  entirely the moment `mycall` isn't set (`if(len(trim(mycall12)).lt.3)
  return`, the subroutine's first line). A real `jt9 -8 -d 3` run with
  no `-c`/`-x` has AP fully disabled; its own `K1BZM DK8NE -10` decode
  is blind (`iaptype=0`) — a real, separate, still-open blind-decode
  sensitivity gap this function does not explain or close, it just
  reaches the same message through a different, AP-assisted route.
  Cost-optimized from a naive per-(candidate, callsign) sweep (~5.7s
  on `qso3_busy.wav`) via three measured steps: (1) a presync
  `sync_quality` gate before the callsign loop (66→8 candidates), (2)
  a blind-decode precheck sharing the same refined-candidate cache,
  (3) `DecodeDepth::BpAll` instead of `BpAllOsd` for the AP retries
  (OSD is redundant once AP narrows the search) — cut to ~1.25-1.3s
  single-threaded, ~0.9s on 24 cores via `rayon` (the per-callsign
  loop is embarrassingly parallel, no shared mutable state). The
  parallel speedup is real, independent task-level parallelism — not a
  substitute for closing the blind-decode gap above, and single-
  threaded this function alone still costs more wall-clock than real
  `jt9 -8 -d 3`'s entire file decode (~1.1s), since it does provably
  more search than jt9 needs for this signal (nothing — its own blind
  pass already finds it).

- **OSD fallback gate loosened from `q >= 12` to `q > 6`**
  (`decode_block/osd_strategy.rs`, issue #180 follow-up) — the old
  gate was a mfsk-core-specific deviation with no `ft8b.f90`
  counterpart, which only bails (`nsync <= 6`) before attempting any
  decode at all. Root-caused directly: `K1BZM DK8NE -10` sits at
  `q=11` on mfsk-core's own SIC residual — verified an *exact*
  per-block match to real jt9's own residual at the same coordinates
  (`is1=1 is2=7 is3=3`, confirmed via a locally-instrumented jt9
  rebuild) — yet never reached the OSD fallback at all under the old
  gate. Full regression suite showed zero change from loosening it —
  nothing was relying on the tighter gate to stay green.
  **Not sufficient on its own**: even past this gate, the real OSD
  dispatch for `q=11` (`osd_decode_npre1`, ndeep=2) still fails to
  decode this candidate on any LLR variant, where WSJT-X's real
  `osd174_91.f90` ndeep=2 succeeds (`hard_errors=18`). Verified this
  isn't a SIC/LLR-input problem: mfsk-core's own residual matches
  jt9's own residual not just on sync quality but on all 58
  data-symbol tone decisions (0/58 argmax disagreements) and on the
  actual LLR reliability ordering OSD consumes (top-91-most-reliable
  overlap 90-91/91 across all 4 LLR variants, zero hard-decision
  disagreements within the shared basis). Filed as a genuine OSD
  algorithm fidelity gap — issue #182, open.

### Changed

- **Q65-60B/30A `decode_multi_period_for` sped up ~4×**
  (`q65/rx.rs`, `tests/q65_wsjtx_samples.rs`) — two stacked fixes: (1)
  the fast-fading `b90 × model` sweep (6 combinations) was redundantly
  re-extracting FFT-based energies from scratch for each combination
  instead of extracting once and reusing; (2) `max_candidates=32` hit
  its cap on every slot, paying the full 8-stage decode ladder for
  candidates far below the real signal's own score (confirmed #0-ranked
  in every slot of both golden recordings via score-distribution
  profiling) — cut in two verified steps (32→16, then — after
  re-profiling showed the fading-BP stage still ~79% of wall-clock even
  after fix (1) — 16→8, matching `SearchParams::default()`). Golden-WAV
  wall-clock: Q65-60B 1.89s → 0.49s, Q65-30A 2.55s → 0.64s,
  bit-identical recall at every step.
- **Q65-60A/`decode_scan_for` family rewritten as a faithful `(Δf, Δt,
  b90)` grid search** (`q65/rx.rs::decode_at_grid_for`) — WSJT-X's
  `q65_loops.f90` has no separate "plain BP" path at all, always
  sweeping the fast-fading metric over a submode-specific b90 range in
  an `ndepth`-gated grid; our previous port diverged into a
  narrow-window AWGN-only Bessel metric with a time-only retry. Landing
  the faithful port surfaced three further bugs found via a real-`jt9`
  cross-check: a missing full/unpruned `ibw` sweep at the origin cell
  (WSJT-X's primary `q65_dec_q012` decode stage, which the pruned
  `q65_loops` fallback alone doesn't cover — worst for wide-`ibwa`
  C/D/E sub-modes); coarse-sync time resolution 4× coarser than
  WSJT-X's own `NSTEP=8` (`q65/search.rs::Spectrogram`, fixed alongside
  a `q65_ccf_22`-style restructure — per-frequency time-collapse +
  local-max NMS + noise-adaptive percentile admission — needed to avoid
  regressing a real off-air multi-signal recording); and a wrong fading
  model — `q65_dec1`/`q65_dec2` hardcode Lorentzian, not Gaussian, but
  the port used Gaussian, invisible for narrow-`b90` A/B sub-modes and
  costing wide-`b90` C/D/E sub-modes ~2.5-3 dB. With all three fixed,
  all ten sub-modes sit within ~1 dB of real `jt9`'s own crossing. Real
  off-air golden-test recall also *improved* (6 m EME sample: 3 → 4
  messages recovered). A follow-up `max_candidates` calibration
  (`tests/q65_wsjtx_samples.rs`, score-distribution profiling — same
  methodology as the Q65-60B/30A cut above) found all 4 real signals in
  the 6 m EME golden recording ranked within the top 8 of 530 coarse
  candidates; cut the test's `max_candidates` 32→16 (2× headroom,
  deliberately not pushed to the edge given a thin ~0.002 score margin
  at the weakest signal's rank), dropping golden-WAV time 1.49s→0.69s
  (~2.2×) with bit-identical recall. The same calibration then extended
  to the four fading-metric golden tests (`decode_scan_fading_for`,
  which don't go through `decode_at_grid_for` but share the coarse-sync
  overhaul's `coarse_search_for` and had gotten slower from it): real
  signals ranked within the top 3 of 94-3094 candidates in all four
  recordings (Q65-300A's margin over the next candidate is thin,
  ~0.0004, so its cut kept more headroom than the others). Cutting
  each test's `max_candidates` (100/30/30/200 -> 8/8/8/20) dropped
  Q65-60D 0.39s->0.08s, Q65-120D 0.15s->0.12s, Q65-120E 0.32s->0.26s,
  Q65-300A 1.05s->0.34s, bit-identical recall throughout. See
  `docs/notes/Q65_BENCHMARK.md`.
- **FST4-60A OSD depth-escalation gate hand-calibrated for its own
  `N_SYNC=40`** (`core/pipeline.rs`) — the shared `osd_depth3_min=18`
  gate was calibrated against FT8's `N_SYNC=21`; FST4's larger sync
  sequence made 18 a far looser bar, escalating roughly half of all
  candidates into the expensive OSD depth-3/4 tier regardless of signal
  quality. Golden-WAV decode time dropped **2.60 s → 0.27 s (~8.4×)**,
  recall verified unchanged (4-channel AWGN/CCIR sweep matching the
  documented pre-fix baseline almost exactly, FST4-120/300 spot-checked).
  A first attempt (reusing FT4's exact `N_SYNC`-scaled formula) measured
  as a real ~0.5 dB AWGN sensitivity regression and was corrected before
  shipping — see `docs/notes/FST4_BENCHMARK.md` section 8.
- **FT4 coarse-candidate stage replaced with a faithful
  `getcandidates4.f90` port** (`core::ft4_coarse::ft4_coarse_sync`) —
  the generic Costas-lag coarse_sync FT4 previously shared with
  FT8/FST4 was a structurally different algorithm from what WSJT-X
  actually does for FT4 (a 2-D freq×lag correlation search vs. WSJT-X's
  frequency-only periodogram), producing ~4.5× redundant candidates
  per real signal on busy recordings. Golden-WAV decode time dropped
  **1.20 s → 0.049 s (~25×)** with byte-identical 6/6 recall; AWGN 50%
  sensitivity crossing moved −17.2 dB → −16.9 dB (structural algorithm
  swap reshaped rather than uniformly improved the SNR curve — 3 of 4
  channels held or improved). See `docs/notes/FT4_BENCHMARK.md` section
  13.
- **Q65-60B/30A `decode_multi_period_for` sped up further, ~2.3×/~1.3×**
  (`q65/search.rs::Spectrogram::build_for`) — follow-up to a separate
  FFT-cache wiring investigation on FT8's `decode_block` (same "does
  this pattern exist elsewhere" question, applied to Q65). That
  specific pattern wasn't present here: `decode_at_grid_for`'s
  `GridDepth::Fast` has exactly one `(Δf,Δt)` grid cell per candidate
  (confirmed via direct call-count instrumentation — 16 calls for
  Q65-60A's 16 candidates, not the ~333 an earlier estimate assumed),
  so no redundant FFT-planner construction existed to cache. Phase-by-
  phase profiling of `decode_multi_period_for` found a different bug
  instead: `Spectrogram::build_for`'s noise-floor estimate (run once per
  candidate-search call) computed a trimmed mean of the bottom 95% of
  FFT-magnitude bins via a full `sort_unstable_by` (O(n log n)) over the
  entire magnitude array (hundreds of thousands to millions of cells for
  a 60-120 s slot) when only the unordered *set* of bottom-95% values
  was needed. Same class of fix already applied to FT8's
  `xsnr2_db_simple` noise-floor median — swapped to
  `select_nth_unstable_by` (O(n) average partition), Q65 just hadn't had
  it applied. Measured as 64% of wall-clock on Q65-60B (short slot, low
  candidate count) and 12% on Q65-30A (longer multi-slot audio where the
  BP/fading-metric stage dominates instead). Golden-WAV decode time:
  Q65-60B **0.28 s → 0.12 s**, Q65-30A **0.72 s → 0.56 s**, byte-
  identical recall on both (VK7MO/VK7PD on 60B, K1JT/K9AN AP-list on
  30A).

### Removed

- **Legacy BASIS (Q15 sin/cos dot-product) per-symbol DFT fill path**
  (#162) — dead weight since the 0.6.4 Goertzel migration (Phase
  1.7.7-Stick) made Goertzel the sole production fill on every
  embedded target, but the old path's functions and scratch
  parameters were never actually deleted. Removed
  `fill_symbol_spectra_into`/`fill_symbol_spectra_into_generic`,
  `BASIS_SCRATCH_LEN`, `symbol_spectra_direct_into`, the
  `mfsk_core::core::dotprod` module (`dot_q15_i32` + the
  `mfsk_core_dot_q15_i32` extern symbol contract), and its esp-dsp
  ASM bridge in `embedded-shared::esp_dsp_fft`
  (`mfsk_core_dot_q15_i32`, `dsps_dotprod_s16_ae32`,
  `ESP_DSP_DOT_SHIFT`/`set_dot_shift`). `fill_symbol_spectra_generic`
  (the plain rotator-based DFT) is now unconditional — it no longer
  needs a separate `fixed-point`-gated BASIS-backed overload.
- **Breaking FFI change**: `mfsk_ft8_decode_i16`'s C ABI
  (`mfsk-ffi-ft8`) dropped its `basis_re`/`basis_im` `int16_t*`
  scratch parameters — they were dead weight once Goertzel took
  over, and the crate's `mfsk_ft8_basis_scratch_len()` sizing
  function is gone with them. **C callers must drop both arguments
  from their call site** when upgrading past 0.7.x. This forces the
  minor version bump for this release (0.7.4 → 0.8.0) ahead of the
  otherwise-patch-level content below.
- `basis_re`/`basis_im` parameters dropped from every Rust function
  in the call chain down to that FFI boundary:
  `decode_block_into[_tuned]`, `refine_candidates_into`,
  `process_candidates_into[_tuned]`,
  `process_candidates_into_with_cs_scratch[_tuned]` (all
  `mfsk-core`), and `dual_core::init`/`pass2_split`/`stage3_split`/
  `run_speculative_slot` (`embedded-shared`) — all embedded app
  crates (`m5stack-s3-app`, `m5stack-core2-app`, `m5stack-cores3-app`)
  updated to match; none of them were actually using the scratch for
  anything by this point (Goertzel needs none), so this is a pure
  signature simplification with no behavioral change on any target.
- `mfsk-core/tests/ft8_goertzel_vs_basis.rs` deleted — its entire
  purpose was A/B-validating BASIS against Goertzel during the
  Phase 1.7.7-Stick migration, which is moot once BASIS no longer
  exists to compare against.

### Fixed

- **JT65 total decode failure, root-caused and fixed** (#24). Every
  `jt65sim`-generated (or otherwise WSJT-X-compatible) JT65 signal
  failed to decode regardless of SNR, even when `search::coarse_search`
  landed candidates within ~1 Hz / a few symbols of ground truth.
  Root cause: `jt65::interleave::interleave`/`deinterleave` (the 7×9
  transpose WSJT-X's `interleave63.f90` implements) had their
  permutations swapped relative to WSJT-X's TX/RX convention
  (`gen65.f90`/`jt65sim.f90` call `interleave63(sent, idir=1)` at TX;
  `extract.f90` calls `interleave63(mrsym, idir=-1)` at RX). The swap
  was internally self-consistent — TX and RX remained exact mutual
  inverses of each other — so every self-roundtrip test passed while
  real/independent-reference signals decoded to garbage RS codewords.
  Structurally identical to the JT9 encoder bug (#19): a decode path
  only ever cross-checked against this crate's own encoder, never an
  independent reference.
- New `tests/jt65_sweep.rs` (`#[ignore]`d, mirrors `ft8_sweep.rs`) +
  `scripts/gen_jt65_sweep_wavs.sh` build a `jt65sim`-generated AWGN
  SNR-sweep corpus and characterise the post-fix recall curve: 100%
  recall down to 0 dB, 50% around -14 dB, near-zero below -19 dB (all
  in `jt65sim`'s 2500 Hz reference bandwidth convention).
- Found and documented, but did **not** port in this fix: WSJT-X's
  own hard-decision-only path (`jt9 -6`, no `kvasd`) holds ~100%
  recall down to -22 dB on the same corpus — a further ~7-8 dB
  sensitivity gap. Root cause traced to `lib/ftrsd/ftrsdap.c`, a
  stochastic Chase decoder (randomized multi-trial soft-symbol
  erasure-pattern search around Berlekamp-Massey RS) that WSJT-X runs
  even without `kvasd` — a materially different (and more involved)
  algorithm than this crate's single-pass confidence-ordered erasure
  decode (`decode_at_with_erasures`). Tracked as a follow-up (#169);
  see the sweep test's doc comment for the full provenance.
- **Q65 AWGN sensitivity gap, root-caused and substantially
  narrowed** (#171). The initial sweep found a real, reproducible
  ~2-3 dB gap vs. WSJT-X's own plain-BP decode (`jt9 -3 -p 30 -b A`)
  for Q65-30A specifically (50% recall crossing ~-24 dB vs. WSJT-X's
  ~-26 to -27 dB), confirmed directly on a single failing file (not a
  batch-script artifact). The entire FEC/BP stack was verified
  byte-for-byte correct against `WSJT-X/lib/qra/q65/{qracodes.c,
  npfwht.c,pdmath.c}` first (all 10 code tables, the WHT, and every
  `pdmath` primitive — diffed programmatically, zero discrepancies),
  which pointed the search upstream: `coarse_search_for`'s reported
  best `(start_sample, freq_hz)` is measurably imprecise at low SNR
  — off by up to ~1/5 of a symbol period — and neither
  `decode_scan_for` nor `decode_at_for` ever refined that alignment
  before attempting decode. WSJT-X's `q65_loops` never trusts its own
  coarse alignment either — it always runs a local fine-timing
  (`idt`) retry loop before the real decode attempt. Added
  `decode_at_with_fine_timing_for` (tries the reported alignment
  first, then a symmetric ±3-step retry grid in units of `nsps/16`)
  to `decode_scan_inner`, the shared implementation behind every
  scan-level Q65 entry point. Full 990-file sweep re-run: all six
  wired sub-modes improved by roughly 1-2 dB at their threshold
  region (e.g. Q65-60A 20%→100% at -27 dB; Q65-30A 40%→93% at -24 dB,
  0%→33% at -25 dB).
- **Remaining residual gap traced to a comparison-methodology
  difference, not a further implementation bug.** Even after the
  fine-timing fix, a gap persisted at the deep end (e.g. Q65-30A ~0%
  at -26 dB vs. WSJT-X's ~40%). Traced this to WSJT-X's Q65 decode
  chain always having access to the free "CQ ??? ???" AP hypothesis
  (`aptype=1` in `extract.f90`'s AP table — needs no user-supplied
  callsign), so *every* real `jt9` decode attempt implicitly gets
  some AP-list benefit on CQ-calling signals, whether or not the user
  configured `-c`/`-x`. `decode_scan_for` (this crate's genuinely
  blind baseline) has no equivalent for-free hypothesis by design —
  the four decoder strategies stay deliberately separate
  (`docs/reference/LIBRARY.md` §3). Re-measured with
  `decode_scan_with_ap_for` + a `"CQ"` hint (now also reported by
  `tests/q65_sim_sweep.rs` as a second column) and it closes the gap
  almost exactly across all six sub-modes: Q65-30A -26 dB 0%→40%
  (matches WSJT-X's reported 40%), -25 dB 33%→93% (WSJT-X: 87%),
  -24 dB 93%→100% (WSJT-X: 100%); Q65-60A -29 dB 7%→53%, -28 dB
  47%→93%; Q65-60B/C/D/E show the same ~40-50 point jump at their
  respective thresholds. **Usage note, not a code change**:
  applications wanting WSJT-X-equivalent behavior for CQ traffic
  should call the AP-hinted path with at least a `"CQ"` hint rather
  than the plain one — matching what WSJT-X's own decoder always
  does internally. Issue #171 left open with this as the closing
  analysis (no further action expected; re-open if a real remaining
  gap is found with matched AP context on both sides).

### Added

- **Q65-15A**: a new sub-mode ZST (`Q65a15`, 15 s T/R period, ×1 tone
  spacing = 6.667 Hz), the fastest wired Q65 mode — one line via the
  existing `q65_submode!` macro, following the established
  15/30/60-s-period axis `q65params.f90` already defines. Wired
  through the FFI (`MfskQ65SubMode::A15`, appended after `E60` rather
  than inserted before `A30` to keep the `#[repr(C)]` enum's existing
  discriminant values stable) and the `PROTOCOLS` registry
  (`"Q65-15A"`). No real off-air recording exists for this period in
  WSJT-X's sample tree (same situation as Q65-60C/60E already
  documented), so no golden-WAV test is possible — covered instead by
  a dedicated `tests/q65_a15_roundtrip.rs` (synth + aligned/offset
  scan recovery) and folded into the `q65sim`-based AWGN sweep below
  (50% crossing ≈ -21 dB, matching `q65params.f90`'s analytical
  formula for the 15 s period).
- **Q65-120D, Q65-120E, Q65-300A**: three longer-period sub-modes
  (`Q65d120`/`Q65e120`/`Q65a300`), chosen because WSJT-X's own user
  guide (`doc/user_guide/en/protocols.adoc`) and sample tree document
  real, specific use cases for exactly these three: Q65-120D (10 GHz
  rainscatter/troposcatter, backed by 14 files in WSJT-X's own
  `UnitTests.txt` regression corpus), Q65-120E (6 m ionoscatter),
  and Q65-300A (optical/laser scatter — the deepest wired Q65
  sub-mode, ~-34 dB AWGN threshold, matching the published table
  value almost exactly). Unlike Q65-15A, these three **do** have
  golden-WAV tests: `tests/q65_wsjtx_samples.rs` gained
  `rainscatter_10ghz_120d_decodes_with_fading_metric`,
  `ionoscatter_6m_120e_decodes_with_fading_metric`, and
  `optical_scatter_300a_decodes_with_fading_metric` — golden messages
  ("VK3WE VK7MO QE37", "KB7IJ N0AN 73", "VK7MO VK7PD QE38")
  independently confirmed via `jt9 -3 -p {120,300} -b {D,E,A}` first.
  All three need the fast-fading metric to decode (plain BP fails,
  same shape as the existing Q65-60D EME test) — for Q65-300A this
  holds even though it's stable-path scatter rather than classic
  Doppler-spread EME, suggesting the fading metric's robustness helps
  generally at threshold-adjacent SNR, not only under true multipath.
  Now **10 wired Q65 sub-modes total**; docs (`LIBRARY.md`/`.ja.md`),
  `tests/protocol_invariants.rs`, FFI (`MfskQ65SubMode::{D120,E120,A300}`,
  discriminants 7-9), and the `q65sim` AWGN sweep all updated to match
  (the 120/300 s configs use 5 trials instead of 15 — their WAVs are
  proportionally larger and the #171 fine-timing retry multiplies
  decode cost further).
- **Direct WSJT-X cross-check for Q65-120D/120E/300A**: ran `jt9 -3
  -p {120,300} -b {D,E,A}` (no `-c`/`-x`) over the identical 165-file
  sweep corpus per sub-mode used above. Result: **no regression, and
  two sub-modes exceed WSJT-X's own plain decode**. Q65-300A's curve
  is statistically identical to `jt9`'s at every tested SNR point
  (both cross 50% at ≈-35 dB). Q65-120D and Q65-120E's `decode_scan_for`
  50% crossings (≈-30.7 dB, ≈-31.0 dB) are **2.5-3.4 dB better** than
  `jt9 -3`'s own plain-decode crossings (≈-28.2 dB, ≈-27.6 dB) —
  consistent across 4+ SNR points each, not sampling noise. Likely
  explanation (not fully confirmed): the #171 fine-timing retry tries
  a fixed ±3-step grid per coarse candidate regardless of T/R period,
  which may end up relatively more thorough than WSJT-X's own
  `q65_loops.f90` `idt`/`idf` retry granularity at these slower-baud,
  longer-period sub-modes specifically.
- New `scripts/gen_q65_sweep_wavs.sh` + `tests/q65_sim_sweep.rs`
  (`#[ignore]`d): a `q65sim`-generated AWGN SNR sweep covering every
  sub-mode ZST this crate actually wires (`Q65a15`, `Q65a30`,
  `Q65a60`, `Q65b60`, `Q65c60`, `Q65d60`, `Q65e60`, `Q65d120`,
  `Q65e120`, `Q65a300` — WSJT-X's Q65 also has other (period,
  sub-mode) combinations this crate doesn't implement, so the sweep
  intentionally covers only what's shipped). `q65sim` has a real
  CMakeLists.txt target (unlike `jt9sim`), so no new build script was
  needed — build via `cmake --build ~/wsjtx-build --target q65sim`.
  Below period=30 s, `q65sim` uses a completely different filename
  format (`000000_MMSS.wav`, not the sequential index used at ≥30 s)
  — `gen_q65_sweep_wavs.sh` special-cases this for Q65-15A.
- **New `tests/q65_wsjtx_samples.rs::tropo_1296_60b_decodes_via_averaging`**:
  Q65-60B was the only wired sub-mode with a real off-air recording
  already vendored (`WSJT-X/samples/Q65/60B_1296_Troposcatter/`) but
  no corresponding test. Single-slot decode fails on this dataset (as
  expected, same as the existing 10 GHz EME test's plain path); the
  multi-period EMA-on-spectrogram path (`decode_multi_period_for`,
  same mechanism the existing ionoscatter test uses) recovers the
  golden message "VK7MO VK7PD QE38" cleanly.
- Q65-60C and Q65-60E have no real off-air recording anywhere in
  WSJT-X's sample tree, so no golden-WAV test is possible for those
  two sub-modes (same situation JT65 was already in before this
  session — no real recording exists to validate against).
- New `scripts/build_jt9sim.sh` + `scripts/gen_jt9_sweep_wavs.sh` +
  `tests/jt9_sweep.rs` (`#[ignore]`d, mirrors `tests/jt65_sweep.rs`):
  a `jt9sim`-generated AWGN SNR sweep for JT9. Unlike `ft8sim`/
  `ft4sim`/`fst4sim`/`jt65sim`, `jt9sim` has no CMakeLists.txt target
  in WSJT-X at all — `build_jt9sim.sh` assembles its actual dependency
  closure (`gen9` → `packjt`/`entail`/`encode232`/`interleave9`/
  `graycode`, plus `jt9fano`/`fano232` for jt9sim's own internal
  self-verify step) as a standalone binary from source.
- Result: `decode_scan_default` holds **100% recall down to -24 dB**,
  crossing 50% around -26 dB — closely tracking WSJT-X's own `jt9 -9`
  on the identical 300-file corpus (100% to -25 dB, 80% at -26 dB; the
  per-cell differences are within 20-trial sampling noise at the
  steep part of the curve, not a systematic gap). Confirms JT9 has
  **no JT65-style hidden sensitivity gap** — the three remaining
  misses in the real-recording golden test
  (`tests/jt9_wsjtx_samples.rs`) are congestion/wrong-codeword-lock
  issues specific to that busy recording, not a general AWGN
  weakness.
- Also re-confirms the #19 encoder fix (`pack_grid4_plain`/
  `unpack_grid`) against a second, independently-built reference
  encoder: a fresh `jt9sim` signal ("CQ JL1NIE PM95" @ 1400 Hz)
  decodes cleanly, matching WSJT-X's own `jt9 -9` output exactly.

### Fixed

- **Q65 AWGN sensitivity gap, root-caused and substantially
  narrowed** (#171). The initial sweep found a real, reproducible
  ~2-3 dB gap vs. WSJT-X's own plain-BP decode (`jt9 -3 -p 30 -b A`)
  for Q65-30A specifically (50% recall crossing ~-24 dB vs. WSJT-X's
  ~-26 to -27 dB), confirmed directly on a single failing file (not a
  batch-script artifact). The entire FEC/BP stack was verified
  byte-for-byte correct against `WSJT-X/lib/qra/q65/{qracodes.c,
  npfwht.c,pdmath.c}` first (all 10 code tables, the WHT, and every
  `pdmath` primitive — diffed programmatically, zero discrepancies),
  which pointed the search upstream: `coarse_search_for`'s reported
  best `(start_sample, freq_hz)` is measurably imprecise at low SNR
  — off by up to ~1/5 of a symbol period — and neither
  `decode_scan_for` nor `decode_at_for` ever refined that alignment
  before attempting decode. WSJT-X's `q65_loops` never trusts its own
  coarse alignment either — it always runs a local fine-timing
  (`idt`) retry loop before the real decode attempt. Added
  `decode_at_with_fine_timing_for` (tries the reported alignment
  first, then a symmetric ±3-step retry grid in units of `nsps/16`)
  to `decode_scan_inner`, the shared implementation behind every
  scan-level Q65 entry point. Full 990-file sweep re-run: all six
  wired sub-modes improved by roughly 1-2 dB at their threshold
  region (e.g. Q65-60A 20%→100% at -27 dB; Q65-30A 40%→93% at -24 dB,
  0%→33% at -25 dB).
- **Remaining residual gap traced to a comparison-methodology
  difference, not a further implementation bug.** Even after the
  fine-timing fix, a gap persisted at the deep end (e.g. Q65-30A ~0%
  at -26 dB vs. WSJT-X's ~40%). Traced this to WSJT-X's Q65 decode
  chain always having access to the free "CQ ??? ???" AP hypothesis
  (`aptype=1` in `extract.f90`'s AP table — needs no user-supplied
  callsign), so *every* real `jt9` decode attempt implicitly gets
  some AP-list benefit on CQ-calling signals, whether or not the user
  configured `-c`/`-x`. `decode_scan_for` (this crate's genuinely
  blind baseline) has no equivalent for-free hypothesis by design —
  the four decoder strategies stay deliberately separate
  (`docs/reference/LIBRARY.md` §3). Re-measured with
  `decode_scan_with_ap_for` + a `"CQ"` hint (now also reported by
  `tests/q65_sim_sweep.rs` as a second column) and it closes the gap
  almost exactly across all six sub-modes: Q65-30A -26 dB 0%→40%
  (matches WSJT-X's reported 40%), -25 dB 33%→93% (WSJT-X: 87%),
  -24 dB 93%→100% (WSJT-X: 100%); Q65-60A -29 dB 7%→53%, -28 dB
  47%→93%; Q65-60B/C/D/E show the same ~40-50 point jump at their
  respective thresholds. **Usage note, not a code change**:
  applications wanting WSJT-X-equivalent behavior for CQ traffic
  should call the AP-hinted path with at least a `"CQ"` hint rather
  than the plain one — matching what WSJT-X's own decoder always
  does internally. Issue #171 left open with this as the closing
  analysis (no further action expected; re-open if a real remaining
  gap is found with matched AP context on both sides).
- **WSPR AWGN SNR sweep test infra** (`tests/wspr_sweep.rs`,
  `scripts/gen_wspr_sweep_wavs.sh`), closing the one gap left where
  WSJT-X ships a simulator but this crate had no simulator-driven
  objective benchmark — every other supported protocol
  (FT8/FT4/JT9/JT65/Q65/MSK144/FST4) already had a `*sim`-based AWGN
  sweep; WSPR's only prior validation was a single real-world WAV's
  fixed-SNR golden recall (`wspr_wsjtx_samples.rs`). WSJT-X's
  CMakeLists.txt only wires the C `wsprsim` (`lib/wsprd/wsprsim.c`),
  which writes `.c2` complex-baseband files for `wsprd`, not WAV audio
  this crate's decode path consumes — the WAV-capable simulator is a
  second, orphaned Fortran program (`lib/wsprd/wsprsimf.f90`, no
  CMake target, same situation as `jt9sim`) that this crate now
  builds standalone via `scripts/build_wsprsim.sh` (picking out its
  actual dependency closure, plus a local no-op `watterson` stub to
  avoid pulling in FFTW for a code path — the `.c2` branch — this
  build never exercises). 13-point AWGN sweep, 20 trials/point:
  100% recall from 0 dB down to -27 dB, 95% at -28/-29 dB, 40% at
  -30 dB, 0% at -31 dB and below — consistent with WSJT-X's published
  WSPR sensitivity floor.

### Tests

- **`mfsk-ffi` Rust-level ABI test coverage for the non-Q65
  protocols** (`mfsk-ffi/tests/wsjt_ffi.rs`). Previously only Q65 had
  a `cargo test`-driven FFI test (`q65_ffi.rs`); FT8/FT4/WSPR/JT9/
  JT65/FST4-60A were exercised solely by the C++ smoke driver
  (`examples/cpp_smoke`). Added encode/decode round-trips for all
  six (mirroring the C++ driver's known-good test vectors), an
  `mfsk_decode_i16` case (the f32 path was the only one covered
  before), and NULL/bad-input negative-path tests
  (`mfsk_decode_f32` with a null decoder/samples pointer,
  `mfsk_encode_ft8` with an unpackable callsign, freeing null
  pointers). Also fixed a CI gap found while adding this: the `ffi`
  job built `mfsk-ffi` and ran the C++ driver but never ran
  `cargo test -p mfsk-ffi` at all, so `q65_ffi.rs` itself was not
  actually executing in CI; `.github/workflows/ci.yml` now runs it
  (FST4's slow slot decode stays behind the same
  `RUN_FST4_ROUNDTRIP` gate the C++ driver already used).
- **`mfsk-ffi-ft8` wired into the same `ffi` CI job** — previously
  had zero CI coverage of any kind (not even a build step), despite
  shipping a `tests/streaming.rs` suite (7 tests, ring-buffer /
  resample / chunk-boundary coverage for the `mfsk_ft8_stream_*`
  API) that only ever ran locally. Added `cargo build`/`cargo test -p
  mfsk-ffi-ft8` steps using the crate's default `host` feature only
  — the `embedded-fixed-point` feature builds `no_std` and has no
  `cargo test` harness to run on a CI runner, so the
  xtensa-esp32/esp32s3-espidf cross-compiles stay build-only-verified
  in `release.yml` as before (they can't be executed without real
  hardware anyway).

### Changed

- **`mfsk-ffi`'s version unstuck from a stale `0.1.0`** — it had
  never been bumped since the crate's initial commit despite
  gaining Q65 sub-modes and other features release after release.
  Introduced `[workspace.package].version` in the root `Cargo.toml`
  as the single source of truth; `mfsk-core`, `mfsk-ffi`, and
  `mfsk-ffi-ft8` now all declare `version.workspace = true` instead
  of a hand-copied literal, so a release version bump is one edit
  instead of three manually-synced ones (the exact class of bug that
  let `mfsk-ffi` drift). `release.yml`'s tag-vs-Cargo.toml gate
  (`verify-tag` job) updated to read the workspace version from the
  root `Cargo.toml` instead of `mfsk-core/Cargo.toml` directly, since
  the latter no longer carries a literal version string.
- **`mfsk-ffi` release-build policy decided: desktop/mobile stays a
  single all-decoder library, and it's now distributed the same way
  as `mfsk-ffi-ft8`.** Confirmed design split — `mfsk-ffi-ft8` stays
  the embedded-only FT8 slice, `mfsk-ffi` stays the desktop/mobile
  superset covering all seven WSJT modes (Kotlin/Android's binary
  footprint isn't a concern here, unlike the embedded no_std targets;
  Android release binaries deliberately deferred — see below) — and
  closed the distribution gap this implied for the desktop target:
  `release.yml` gained `build-ffi-desktop-host` (linux-x86_64:
  `libmfsk.{so,a}` + `mfsk.h`), gating `publish` alongside the
  existing `mfsk-ffi-ft8` build jobs so a broken build blocks the
  release instead of leaving it half-published. `mfsk-ffi/README.md`,
  root `README.md`, and `docs/reference/LIBRARY.md` §8-9 updated to
  describe the new prebuilt-binary distribution instead of "clone and
  build". Android (arm64 via NDK cross-build) was drafted in the same
  pass — a working `build-ffi-desktop-android` job was written and
  locally verified for target/toolchain-name correctness — but
  deliberately left out of this release-build pass; Kotlin/Android
  consumers keep building `mfsk-ffi` locally with cargo-ndk per
  `mfsk-ffi/examples/kotlin_jni/README.md` for now.
- **`examples/kotlin_jni/` scaffold renamed to match the
  `wsjt-ffi` → `mfsk-ffi` crate rename it had missed** — found while
  drafting the Android release build above. `Wsjt.kt` /
  `wsjt_jni.c` / package `io.github.rsft8n` still referenced the
  pre-rename crate name (`cargo build -p wsjt-ffi`, `libwsjt.so`,
  `WsjtMessageList`, …), none of which exist anymore; the scaffold's
  own documented build commands would not have run. Renamed to
  `Mfsk.kt` / `mfsk_jni.c` / package `io.github.mfskcore` throughout,
  matching what `docs/reference/LIBRARY.md` §9 already (aspirationally)
  described. Also expanded `Mfsk.Protocol` from `FT8`/`FT4` only to
  all seven `MfskProtocol` values, matching `mfsk-ffi`'s actual
  all-decoder scope.

### Fixed

- **FT8 `decode_block` multi-pass decode ~10.5× faster on busy-band
  recordings, WSJT-X-faithful subtract algorithm.** A decode-speed
  investigation (prompted by `qso3_busy.wav` benchmarking 4.73 s —
  suspiciously slow for a 15 s slot on host hardware) profiled
  `decode_block`'s three-pass successive-interference-cancellation
  loop phase by phase. Coarse-sync, fine-refine, and per-candidate
  BP/OSD were all sub-5 ms each; the entire cost (~310 ms per
  accepted decode, ~13 calls on this recording) was
  `core::dsp::subtract::subtract_tones_lpf` — a naive O(candidates ×
  NFRAME × lpf_half) direct time-domain convolution for the
  channel-tracking LPF (≈608 M MACs/call at FT8's `lpf_half=2000`).
  Checked WSJT-X's own `lib/ft8/subtractft8.f90` /
  `lib/ft4/subtractft4.f90`: both use a *cached* filter-response FFT
  (built once, `first`/`save` in the Fortran) plus one forward +
  inverse FFT of the full slot per call — O(N log N), not O(N×M).
  Ported that algorithm exactly (including FT8's `endcorrection`
  edge-response boost, which `subtractft4.f90` omits and the port
  now matches per-protocol via a new `endcorrection: bool` parameter
  on `subtract_tones_lpf`) behind the existing `fft-rustfft` (host)
  feature; the direct-convolution version is kept as the `no_std`
  fallback, unchanged. No embedded target is affected either way —
  `decode_block`'s embedded (`not(fft-rustfft)`) variant of
  `decode_block_multipass` has no subtract loop at all (single-pass,
  no SIC), confirmed by grepping `embedded-shared`'s actual decode
  pipeline before assuming otherwise.
  `ft8_qso3_apoff_recall.rs`'s `qso3_apoff_meets_wsjtx_golden_floor`:
  **4.73 s → 0.45 s**, byte-identical recall (7/8 golden, 7 phantom,
  14 total). Also caught and fixed a second, independent staleness
  bug found during the same investigation: this test's comment
  claimed to match "the Core2 / S3 production path" while passing
  `DecodeDepth::BpAll`; the actual embedded ship config is
  `DecodeDepth::BpVariantsAd` (introduced later, skips the two most
  expensive BP LLR variants for failed candidates) — updated the
  test to match, verified identical recall either way. FT4's golden
  test is on a different (non-LPF, constant-amplitude) subtract path
  in `core::pipeline` and was unaffected by this change.
- **FT8 `decode_block` a further ~3.6× faster on busy-band recordings
  — two FFT-cache wiring gaps closed.** Follow-up profiling of
  `decode_block_multipass` (post the 10.5× subtract fix above) found
  its cost had moved, not disappeared: (1) `refine_candidates` and
  the per-candidate `process_candidates_tuned_with_ap[_ref]` calls
  were all passing `fft_cache: None`, so each of up to ~45
  candidates/slot re-ran the 192 k-point forward FFT from scratch
  even though the sibling `fine_refine_pass1` stage in the same pass
  already built and reused exactly this cache — a wiring gap, not a
  missing algorithm. Threaded the existing
  `build_fft_cache`/`downsample_cached` helpers through
  `refine_candidates`, `process_candidates_tuned_with_ap[_ref]`, and
  `auto_ap_strategy::run`, rebuilding the cache lazily only when the
  WSJT-X-mandated sequential subtract actually mutates the working
  buffer (not eagerly every pass) — alone dropped
  `qso3_apoff_meets_wsjtx_golden_floor` 0.43 s → ~0.12–0.22 s. (2)
  `subtract_tones_lpf_fft`'s filter-response FFT was already cached
  (`cached_window_fft`, from the fix above), but its forward/inverse
  `FftPlanner` *plans* for `nfft = 180 000` were rebuilt on every
  call — measured at ~2.8 ms/call for plan construction vs ~0.7 ms
  for the transform itself, i.e. plan-rebuild cost ~4× the FFT it
  was gating. Cached the plans in an `nfft`-keyed `OnceLock`, the
  same pattern `fill_symbol_spectra.rs`'s `SYMBOL_FFT_32` already
  used for the per-symbol 32-pt FFT. Combined: **0.43 s → 0.12 s
  (~3.6×)**, byte-identical recall (7/8 golden, 7 phantom, 14 total
  on `qso3_apoff`; 5/6 JTDX AP-on extras unchanged on `qso3_apon`).
  Both fixes are cache wiring, not new algorithms, so risk is low;
  `fft-rustfft` is `std`-gated but thread-free, so both caches also
  apply under `wasm32-unknown-unknown` (build-verified). No embedded
  impact either way, since `decode_block_multipass`'s
  `not(fft-rustfft)` variant has no subtract loop (single-pass, no
  SIC) and never calls these paths.

## 0.7.4 — MSK144 decode (#25)

### Added

- **MSK144 meteor-scatter mode**, ported from WSJT-X across the full
  pipeline: LDPC(128, 90) + CRC-13 FEC (a 4th `LdpcParams` impl
  reusing the crate's generic BP/OSD engine, `fec::ldpc_128_90`),
  MSK/OQPSK matched-filter DSP (`core::dsp::msk`), the joint
  CFO/timing burst-scan sync search (`msk144::sync`,
  `msk144::spd::detect_burst_candidates`), per-frame decode
  (`msk144::frame_decode`, reusing the existing `msg::wsjt77`
  `pack77`/`unpack77` payload — no bespoke message codec needed),
  and a top-level sliding-window driver (`msk144::decode::decode_slot`).
  MSK144's continuous-phase modulation and transient-burst timing
  don't fit the static-slot `core::pipeline` model every other
  protocol in this crate shares, so it gets its own driver by design,
  the same shape as WSPR's.
- Cross-validated with an independent reference synthesizer (simple
  binary-FSK audio + FFT-based Hilbert transform, mirroring WSJT-X's
  own `msk144sim.f90`) so RX correctness isn't only checked against
  this crate's own TX code.
- **Golden-WAV regression: 3 / 3** against both WSJT-X
  `samples/MSK144/*.wav` recordings
  (`tests/msk144_wsjtx_samples.rs`), matching WSJT-X's reported
  SNR/frequency/decode-time within a few Hz / exact / ~1 dB, with no
  threshold tuning needed on the first real-signal attempt.
- New `msk144` feature flag, now part of `full`.
- Out of scope (matches the original scoping in issue #25): MSK40
  (the legacy shorthand mode) and RX-equalizer training
  (`msk144signalquality.f90`-equivalent adaptive phase/amplitude
  correction across decodes).

### Fixed

- **MSK144 SNR systematic -1 dB bias, root-caused and closed** (#156).
  WSJT-X's `analytic()` always applies a fixed 1500 Hz-centered
  raised-cosine bandpass filter before computing the `pmax`/`pnoise`
  SNR ratio (`analytic.f90`'s `h(i)`); the initial port's
  `analytic_signal()` was a bare Hilbert transform with no
  frequency-selective filtering, inflating the noise floor and
  depressing every reported SNR by a consistent amount. Porting the
  filter closes the gap to an exact match on all 3 golden decodes
  (was +7/+4/+6 dB, now +8/+5/+7 dB, matching WSJT-X bit-for-bit);
  `tests/msk144_wsjtx_samples.rs` now asserts SNR (±1 dB) instead of
  only printing it.
- **AWGN sensitivity cross-validated against a real WSJT-X `jt9 -k`
  build** on `msk144sim`-generated synthetic signals: 25 of 28
  (ping-length × SNR) cells matched exactly, the other 3 differed by
  exactly 1 file out of 20 — no measurable recall gap at any tested
  SNR. New `tests/msk144_snr_sweep.rs` (`#[ignore]`'d characterization
  sweep, no WSJT-X build dependency, wired into CI's "catchall
  characterization" tier) reproduces the same synthetic-signal recipe
  as an ongoing regression signal.

### Docs

- **`docs/notes/MSK144_BENCHMARK.md` + `.ja.md`**, matching
  `FT4_BENCHMARK.md`/`FST4_BENCHMARK.md`'s convention: how to
  reproduce the `jt9`-based verification above (prerequisites,
  building `jt9`/`msk144sim`/Hamlib from source — Hamlib's
  `integration` branch referenced by `WSJT-X/INSTALL` no longer
  exists, use `master` — `msk144sim`'s verified SNR convention, and
  the measured baseline). Explicitly notes this is a one-time
  verification: `tests/msk144_snr_sweep.rs` itself needs no WSJT-X
  checkout.

## 0.7.3 — FT4 AWGN + FT8 CCIR fading sensitivity close-out (#151, #152, #153)

### Fixed

- **FT4 AWGN sensitivity gap vs WSJT-X narrowed from ~1.8 dB to ~0.3 dB**
  (`docs/notes/FT4_BENCHMARK.md` sections 8-12, PR #151), from two real,
  WSJT-X-source-verified fixes plus one confirmed-null-effect bug fixed
  anyway:
  - **Coherent Costas-block scorer**: `ft4_sync_search`'s scorer was
    non-coherent within each 4-symbol block where WSJT-X's `sync4d.f90`
    does one coherent 4-symbol correlation per block. Switched to the
    coherent scorer already built for `fst4_sync_search`. ~+1.0 dB AWGN.
  - **OSD-attempt gate checked the wrong score**: gated on coarse_sync's
    non-coherent `cand.score` instead of the coherent score, silently
    skipping OSD on 13/17 near-crossing candidates that cleared WSJT-X's
    own `syncmin=1.2`. WSJT-X's FT4 decoder has no such gate at all;
    bypassed `osd_score_min` for FT4 (same precedent as FST4's #146
    bypass), kept `osd_max_errors` as the false-accept safety net.
    ~+0.5 dB AWGN.
  - OSD depth-3/4 escalation gate scale mismatch (`nsync>=18`,
    calibrated against FT8's `N_SYNC=21` but FT4's `N_SYNC=16` is
    mathematically unreachable) — real bug, measured null effect on
    AWGN recall, fixed anyway at zero cost.
  - A candidate WSJT-X-literal 3-segment Δt-search retry was
    implemented, measured, and correctly retracted after discovering
    the diagnostic had omitted the real `hard_errors>=osd_max_errors`
    gate (10/17 apparent rescues → 0/17 once corrected) — reverted to
    avoid 3x per-candidate decode cost for zero benefit.
  - Net: AWGN 50% crossing -15.5 dB → **-17.2 dB**; CCIR fading channels
    improved +0.25 to +1.1 dB depending on channel.
- **FT8 CCIR moderate/poor fading recall gap closed** (`docs/notes/FT8_BENCHMARK.md`,
  PR #152), via a new `ft8sim`-based AWGN/CCIR sensitivity sweep
  (mirroring the FT4/FST4 `ft4sim`/`fst4sim` benchmarks). Root cause:
  `OSD_HARDERRORS_MAX = 22` (deliberately tightened in 0.6.3 to filter
  3 `qso3_busy.wav` candidates judged CRC-luck phantoms) was discarding
  genuine golden decodes under CCIR fading — widened back to WSJT-X's
  universal 36. Monotonic improvement across the full grid (AWGN
  ≈-20.4→-20.8 dB, CCIR good ≈-20.0→-20.6 dB, CCIR moderate
  ≈-18.3→-18.6 dB, CCIR poor ≈-18.2→-18.5 dB), no regressions.
  `qso3_busy.wav` hard-assertion tests unchanged (golden/phantom counts
  identical — the widening touched a disjoint candidate set); JTDX
  18-entry recall 13/18 → 17/18.
- Resolves issue #150 (JTDX-18 ground-truth question) as a side effect:
  the OSD widening above independently reproduces 3 of the previously
  JTDX-only candidates (`N1API F2VX`, `N1API HA6FQ`, `CQ EA2BFM`) —
  two independent decoders converging on the same CRC-14-protected
  message text is strong evidence they're real, not phantoms. A
  follow-up probe (PR #153) confirmed `K1BZM DK8NE` is likewise
  genuine (recoverable with AP context `mycall=K1BZM`, matching how
  WSJT-X's own golden-8 reaches it) and found `coarse_sync` candidates
  exist at WA2FZW DL5AXX's claimed frequency yet no AP context
  recovers the message — the one JTDX-18 entry still classified as a
  likely false positive.

## 0.7.2 — FST4 sensitivity close-out (#146) + docs restructure (#147)

### Fixed

- **FST4 residual AWGN sensitivity gap vs WSJT-X narrowed further, from
  0.7.1's 0.5-1.3 dB per-mode spread to a common ≈0.3 dB across all
  five sub-modes** (issue #146 close-out), from two follow-up fixes on
  top of 0.7.1's coherent full-slot sync:
  - **nsym=4 LLR rung** (`ModulationParams::LLR_NSYM_MID`, FST4-only,
    `None` elsewhere — zero effect on FT8/FT4): WSJT-X's
    `get_fst4_bitmetrics.f90` builds a 1/2/4/8-symbol coherent-depth
    ladder, but `LlrSet`'s four fixed slots only ever computed
    `{1, 2, 8}` plus the bit-normalised variant — nsym=4 was silently
    skipped. Measured across the full 5-mode AWGN sweep: FST4-30
    +0.3 dB, FST4-300 +0.16 dB, FST4-15/60/120 unchanged. Real, but
    small — not the source of the residual gap.
  - **zsum-OSD fallback** in `Ldpc240_101::decode_soft` (the actual
    lever): WSJT-X's `decode240_101.f90` doesn't feed its OSD fallback
    the raw channel LLR — it feeds `zsum`, the running sum of BP's
    variable-node soft estimate across just the first two iterations
    (`zsave(:,iter)` for `iter∈{1,2}`, `maxosd=2`). Added
    `bp_llr_zsum` and wired it as a second, additive OSD attempt —
    only reached if the existing raw-LLR OSD attempt already failed,
    so it can only add recoveries, never regress one. On FST4-120
    near-threshold AWGN trials: raw-LLR-OSD alone recovers 37/106,
    zsum-OSD alone recovers 59/106, the union recovers 62/106, zero
    regressions. `Ldpc174_91` (FT8/FT4) untouched.
- Final full 5-sub-mode re-sweep (`tests/fst4_sweep.rs`, `fst4sim`
  AWGN corpus, 20 trials/SNR point, bootstrap SE from 4000 per-trial
  resamples except FST4-120 which reused its earlier targeted-run
  crossing), 50% recall vs. WSJT-X's published thresholds:

  | Sub-mode | Measured | WSJT-X official | Gap | Bootstrap SE |
  |----------|---------:|-----------------:|----:|----:|
  | FST4-15  | −20.60 dB | −20.7 dB | 0.10 dB | 0.17 dB |
  | FST4-30  | −23.90 dB | −24.2 dB | 0.30 dB | 0.19 dB |
  | FST4-60  | −27.62 dB | −28.1 dB | 0.48 dB | 0.22 dB |
  | FST4-120 | −30.70 dB | −31.3 dB | 0.60 dB | ~0.3-0.4 dB (analytical) |
  | FST4-300 | −34.78 dB | −35.3 dB | 0.52 dB | 0.20 dB |

  Inverse-variance-weighted chi-square against a common-gap null
  (H0: all five gaps are draws from one true residual, differing only
  by per-crossing measurement noise): weighted mean gap ≈ 0.34 dB,
  χ²=3.85, df=4, **p≈0.43** — not significant. 0.7.1's "FST4-60/120
  lag behind 30/300" read was sampling noise, not a real per-mode
  deficit; one common ~0.3 dB residual now fits all five sub-modes.
  Fading-channel crossings, first full measurement under the
  zsum-OSD fix:

  | Sub-mode | awgn | ccir_good | ccir_moderate | ccir_poor |
  |---|---:|---:|---:|---:|
  | FST4-15  | −20.60 | −20.20 | −18.44 | −18.25 |
  | FST4-30  | −23.90 | −23.33 | −20.86 | −21.67 |
  | FST4-60  | −27.62 | −25.78 | −25.43 | −24.50 |
  | FST4-120 | −30.70 | −29.29 | −27.63 | −26.50 |
  | FST4-300 | −34.78 | −33.10 | −30.22 | −25.80 |

  No regressions anywhere versus the pre-fix state.
- Resolved the -23.7-vs-24.2 dB provenance question raised against
  0.7.1: the -23.7 dB figure in an earlier commit was an ad hoc
  target, not a WSJT-X reference value. The primary source
  (Franke/Somerville/Taylor, "Quick-Start Guide to FST4 and FST4W",
  wsjt.sourceforge.io/FST4_Quick_Start.pdf) confirms -24.2 dB for
  FST4-30 under the same 2500 Hz / AWGN-simulation methodology this
  repo's sweep uses — genuinely like-for-like. README/CHANGELOG
  already cited -24.2 dB; no numeric correction needed.

Issue #146 is closed. Issue #148 (blind-paired FST4-120x2 multi-period
soft combining) is a separate, unimplemented research follow-up; a
sync-vs-decode failure-population diagnostic test landed in support of
that investigation but changes no decode behaviour.

### Docs

- **CHANGELOG.md split** (issue #147): pre-0.6.0 history (0.1.0
  through 0.5.12) moved verbatim to
  [`docs/historical/CHANGELOG-0.x.md`](docs/historical/CHANGELOG-0.x.md)
  so the live changelog stays skimmable; full detail preserved, just
  relocated.
- **`docs/` split into `docs/reference/` (reader-facing: `LIBRARY.md`,
  `EMBEDDED.md`, `MANUAL_M5STICKS3.md`, `UVPACKET.md` + `.ja.md`
  mirrors) and `docs/notes/` (maintainer-facing engineering journals:
  `ROADMAP.md`, `PHASE_D_PIE_SIMD.md`, `FST4_BENCHMARK.md` +
  `.ja.md`)** (issue #147), so browsing `docs/` directly no longer
  mixes polished external-facing reference material with
  progress-log-style working notes. `docs/historical/` and
  `docs/assets/` unchanged. No behaviour change; every cross-link in
  README, CHANGELOG, CLAUDE.md files, rustdoc comments, and the moved
  docs themselves updated to match.

## 0.7.1 — FST4 AWGN sensitivity fixes + README/docs discoverability pass (#144, #146)

### Fixed

- **FST4 AWGN sensitivity gap vs WSJT-X closed from ~2.4-3.1 dB to
  ~0.3 dB** (issue #146), from three compounding fixes:
  - `LLR_NSYM_MAX` for FST4 was unset and silently inherited FT8's
    calibrated depth (3); WSJT-X's own `get_fst4_bitmetrics.f90` uses
    a 1/2/4/8-symbol correlation ladder. Override added: `= 8`.
  - `DecodeStrictness::osd_score_min()` / `osd_max_errors()` — FT8-
    calibrated OSD pre-filter and hard-error ceiling — were blocking
    or rejecting real, CRC-24-verified FST4 candidates near
    threshold. Both bypassed for FST4 specifically; FST4 now trusts
    CRC-24 alone, matching WSJT-X's own acceptance test
    (`fst4_decode.f90:570`).
  - New `core::sync2d::fst4_sync_search` — a coherent, full-slot
    two-stage local (Δf, Δt) search matching WSJT-X's
    `fst4_sync_search` / `sync_fst4` (`fst4_decode.f90:657-925`).
    The coarse pass now covers the *entire* T/R slot (±1.5 s) instead
    of a narrow local window, avoiding noise-peak lock-in, and scores
    each 8-symbol Costas block with a single phase-continuous
    coherent inner product (amplitude `|z|`, matching WSJT-X's
    `csync1`) instead of a non-coherent power-sum — ~3 dB better
    sync-score SNR discrimination in the ~5,000-cell coarse grid.
- Measured (`tests/fst4_sweep.rs`, `fst4sim`-generated AWGN corpus,
  20 trials/SNR point, full 5-sub-mode re-run against the final
  0.7.1 code — see `docs/notes/FST4_BENCHMARK.md`), 50% recall crossing
  (linear interpolation between adjacent grid points) vs. WSJT-X's
  published thresholds:

  | Sub-mode | Measured | WSJT-X official | Gap |
  |----------|---------:|-----------------:|----:|
  | FST4-15  | ≈ −20.2 dB | −20.7 dB | 0.5 dB |
  | FST4-30  | ≈ −23.4 dB | −24.2 dB | 0.8 dB |
  | FST4-60  | ≈ −27.0 dB | −28.1 dB | 1.1 dB |
  | FST4-120 | ≈ −30.0 dB | −31.3 dB | 1.3 dB |
  | FST4-300 | ≈ −34.4 dB | −35.3 dB | 0.9 dB |

  All five sub-modes improved substantially from the pre-0.7.1
  baseline (2.3-3.1 dB gap across the board); the residual gap is
  not perfectly flat across sub-modes as the earlier FST4-30-only
  spot check suggested — FST4-60/120 sit ~0.3-0.5 dB behind
  FST4-15/30/300. Not chased further in this release; a plausible
  remaining contributor is the still-missing FST4 2D frequency
  refine (`sync2d_refine` measured as a regression when tried for
  FST4, see below) — worth revisiting with real off-air data rather
  than this bin-centered synthetic sweep.

### Added

- `core::sync::coarse_sync::<P>` gained an FST4-only stage-1
  candidate-detection augmentation: a bin also enters the candidate
  list if it clears a full-slot non-coherent 4-tone power check
  (modelled on WSJT-X's `get_candidates_fst4`), alongside the
  existing short-time Costas-grid threshold. Gated on
  `P::ID == ProtocolId::Fst4`, zero effect on FT8/FT4. Measured as a
  no-op on the narrow single-signal AWGN sweep (the golden candidate
  was never at risk of being dropped there) but is a real coverage
  improvement for busy/wideband scans with many co-channel
  candidates.
- `core::sync2d::sync2d_refine` — FT4's `sync4d_refine` generalised
  (radius/step now scale with the protocol's own `TONE_SPACING_HZ` /
  `ds_spb` instead of FT4-hardcoded absolutes), verified
  bit-identical to the prior FT4 constants via the FT4 golden-WAV
  lock. FT4 only — an attempt to also enable it for FST4 measured as
  a net *regression* near threshold and was left off (see inline
  notes in `core::pipeline.rs`).
- Runnable `docs.rs` examples: FT4 decode round-trip (`ft4` module
  docs), a standalone TX-only encoder example with no FFT/`std`
  dependency (`ft8::wave_gen` module docs), and a `no_std` + `alloc`
  usage example (crate root docs) — previously only FT8 had a
  runnable round-trip doctest.
- `docs/reference/LIBRARY.md` / `.ja.md` §4 updated with the `sync2d` module
  (both `sync2d_refine` and `fst4_sync_search`), the FST4
  `coarse_sync` augmentation, and the `LLR_NSYM_MAX` /
  `DecodeStrictness` per-protocol calibration notes above — this
  section had gone stale relative to the FST4 work.

### Docs

- `README.md` reordered so a first-time visitor hits the overview,
  supported-protocol table, and a copy-pasteable Quick Start example
  before any design discussion.
- New DSP-pipeline Architecture diagram (Audio → FFT → Candidate
  Search → Sync → Demodulation → FEC → Decoded Message), kept
  separate from the existing trait-stack diagram (now under Design
  Philosophy).
- Design Philosophy expanded with an explicit "why a `Protocol`
  trait" section (shared vs. protocol-specific, zero-cost
  monomorphisation) and the "Why Rust" rationale.
- Added a Performance summary (now including the FST4 sensitivity
  table above), a Comparison-with-WSJT-X table, and an FAQ.
- Detailed reference material (attribution, modules, FFI, contributing,
  full status/recall tables) moved below License into a "Reference"
  section so the top of the file stays short.

## 0.7.0 — FST4 all sub-modes + coarse_sync parallelisation (#23, #139)

### Added

- **FST4-15, FST4-30, FST4-120, FST4-300** sub-modes (`Fst4s15`, `Fst4s30`,
  `Fst4s120`, `Fst4s300`) via the `fst4_submode!` macro.  All 5 FST4 periods
  now decode against WSJT-X `fst4sim`-generated signals (#138).
  Per-mode constants verified against `fst4_decode.f90` / `fst4sim.f90`:

  | Sub-mode | NSPS  | NDOWN | TX_START_OFFSET_S |
  |----------|-------|-------|-------------------|
  | FST4-15  | 720   | 18    | 0.5 s             |
  | FST4-30  | 1 680 | 42    | 1.0 s             |
  | FST4-60  | 3 888 | 108   | 1.0 s             |
  | FST4-120 | 8 200 | 205   | 1.0 s             |
  | FST4-300 | 21 504| 512   | 1.0 s             |

- **Generic FST4 decode API**: `decode_frame_for::<P>(audio, cfg, freq_min,
  freq_max, sync_min, max_cand)` — one function covers all sub-modes.

- **`coarse_sync` parallelisation** under `--features parallel` (#139).
  Three serial `fi`-loops (sync2d construction, per-bin peak reduction,
  candidate NMS) replaced with rayon equivalents.  Serial path unchanged
  when the feature is absent.  Measured on a 12-core host during the FST4-300
  CCIR sweep (1 120 files): wall-clock ~3 min 44 s, CPU utilisation ≈8×.
  FST4-300 benefits most (n_freq ≈ 5 200 bins vs FT8's ≈ 464); FT8 and all
  other protocols that call `coarse_sync<P>` also gain proportionally.

- **fst4sim test infrastructure** (`scripts/`):
  - `build_fst4sim.sh` — builds WSJT-X's `fst4sim` Fortran binary (gfortran
    + FFTW3) for use as a golden signal source.
  - `gen_fst4_sim_wavs.sh` — one AWGN WAV per sub-mode at −5 dB.
  - `gen_fst4_sweep_wavs.sh` — full SNR × fading matrix (5 modes × 4 ITU-R
    Watterson channels × SNR sweep × 10 trials = 1 120 WAVs).

### Tests

- `tests/fst4_sim_roundtrip.rs` — all 5 sub-modes decode `fst4sim` −5 dB
  AWGN signals; all pass.
- `tests/fst4_sweep.rs` (`#[ignore]`) — SNR sweep benchmark with recall
  table output.  Parallelises the 10 trials per cell when `parallel` is set.

  Selected sensitivity results (50 % recall threshold, 10 trials/cell):

  | Mode     | AWGN   | ccir_good | ccir_moderate | ccir_poor |
  |----------|--------|-----------|---------------|-----------|
  | FST4-15  | −17 dB | −17 dB    | −15 dB        | −15 dB    |
  | FST4-30  | −20 dB | −20 dB    | −20 dB        | −20 dB    |
  | FST4-60  | −24 dB | −24 dB    | −20 dB        | −20 dB    |
  | FST4-120 | −26 dB | −26 dB    | −25 dB        | −24 dB    |
  | FST4-300 | −30 dB | −30 dB    | −28 dB        | −22 dB    |

  Channels: `ccir_good` = fdop 0.1 Hz / del 0.5 ms,
  `ccir_moderate` = 0.5 / 1.0, `ccir_poor` = 1.0 / 2.0 (ITU-R Watterson).

## 0.6.8 — fix FST4-60A decode (#23): wrong NSPS/NDOWN/GFSK_BT + missing message scramble

`Fst4s60`'s modulation parameters were never-revisited placeholders:
`NSPS=3840`, `NDOWN=192`, `GFSK_BT=1.0` (the old `NDOWN` comment
literally said *"Production value may differ; revisit once decoder
is wired"*). Line-walking WSJT-X's `fst4_decode.f90` /
`fst4sim.f90` / `gen_fst4wave.f90` for `ntrperiod=60` gives the real
values: `NSPS=3888`, `NDOWN=108`, `BT=2.0`. The wrong `NSPS`
accumulated timing error linearly across the 160-symbol frame, so
real audio drifted ~0.3-0.6 s off the true frame start — invisible
in the synth roundtrip test because encode and decode shared the
same wrong constants self-consistently.

Separately, FST4 never wired up the 77-bit `rvec` pre-LDPC scramble
that WSJT-X's `genfst4.f90:63` applies (same sequence FT4 uses).
CRC-24 still passed on real WSJT-X audio without it — the check is
self-referential, it just confirms LDPC decoded whatever bits were
actually sent — but every decode came out as scrambled garbage
instead of a real callsign.

### Fixed

- **`fst4/mod.rs`**: `NSPS`/`NDOWN`/`SYMBOL_DT`/`TONE_SPACING_HZ`/
  `GFSK_BT` corrected to match WSJT-X; `ModulationParams::
  INFO_SCRAMBLE_RVEC` wired to a new `FST4_RVEC` constant (same
  values as `ft4::FT4_RVEC`, duplicated so `fst4` doesn't pull in
  the `ft4` feature).
- **`fst4/decode.rs`**: `FST4_60A_DOWNSAMPLE`'s `fft1_size` /
  `fft2_size` / `tone_spacing_hz` updated for the corrected `NDOWN`.
- **`fst4/encode.rs`**: `FST4_60A_GFSK` corrected (was hardcoded
  independently of `ModulationParams` and never updated in lockstep);
  `message_to_tones` now scrambles the message before CRC-24, matching
  `genfst4.f90`.

### Tests

- `tests/fst4_wsjtx_samples.rs::fst4_60_wsjtx_sample_recall_vs_golden`
  now recovers the golden message `CQ N5TM EL29` (dB=-9) against the
  WSJT-X reference WAV, plus the same recording's second signal
  `CQ K9KFR EN71` (dB=16) as a bonus. Un-ignored (was skipping with
  "decode_frame returns 0 messages" since the test was added).
- Adds `fst4_60_diagnose_golden`, a permanent diagnostic probe
  (`jt9::gate_diag::probe_missing_goldens`-style (freq, dt) grid scan)
  for any future FST4 sync/timing regression.

### Notes

- `mfsk-ffi-ft8` bumped to 0.6.8 in lock-step (FST4 isn't in its FFI
  surface — the crate only wraps FT8 — so this is a version-number
  sync only, no behavioral change).

## Embedded Phase D — ESP32-S3 LX7 PIE SIMD acceleration (2026-05-23)

Not a crates.io release (embedded-only). All four sub-phases landed
on `main` across commits `053bd67` / `0b7978b` / `a8235e7`. See
[`docs/notes/PHASE_D_PIE_SIMD.md`](docs/notes/PHASE_D_PIE_SIMD.md) for the full
plan and [`docs/notes/ROADMAP.md`](docs/notes/ROADMAP.md) Phase D for status.

### Changed — embedded (S3 only, `aes3` feature)

- **D1** `_aes3_` FFT rebind: `dsps_fft2r_fc32_ae32_` →
  `dsps_fft2r_fc32_aes3_` and `dsps_fft2r_sc16_ae32_` →
  `dsps_fft2r_sc16_aes3_` behind the new `embedded-shared/aes3`
  feature (enabled by `m5stack-s3`, `m5stack-s3-app`,
  `m5stack-cores3-app`). sc16 requires 16-byte aligned input;
  `MixedRadix3840Sc16Fft` allocates via `alloc_zeroed` +
  `#[repr(C, align(16))]`. **Measured: S3 sequential bench
  sc16 stage1 1.98 s → 1.65 s (−17%).**
- **D2′** Goertzel hot-path: bounds-check and i16→f32 cast
  hoisted out of the 1920-iteration inner loop in
  `fill_symbol_spectra_goertzel`. Branch-free path covers ~77/79
  symbols per candidate; boundary symbols fall through to the
  original per-sample check.
- **D3** `coarse_sync` allsum sliding window + m-outer loop order
  (`0b7978b`).
- **D4** demux-mag² DC-hoist + 4× unroll on the post-FFT
  |re|²+|im|² loop; `rx_wavsim` migrated to
  `run_speculative_slot` (`a8235e7`).

### Verified on CoreS3 (2026-06-05)

Dual-core wav_sim, qso3 reference WAV, Quad PSRAM, `opt-level=1`:
**post_slotend 136〜138 ms, 7/7 decodes**. coarse_sync (167〜172 ms)
runs during audio capture and is off the critical path.

---

## 0.6.7 — fix coarse_sync wasm32 runtime panic

0.6.6 left `std::time::Instant::now()` calls in `coarse_sync` gated
only by `feature = "std"` (3 timestamps + a final diagnostic block).
Any consumer building for `wasm32-unknown-unknown` with the default
`std` feature panicked the first time stage-2 ran — `Instant::now` is
unimplemented on that target. `webft8` and any future WASM PWA
consumer were affected; embedded (esp-idf) was not because esp-idf
provides a working clock.

### Fixed

- **`coarse_sync.rs`**: the 4 timestamp reads + the `eprintln!`
  diagnostic block are now gated on `#[cfg(feature = "profile-coarse")]`
  rather than `#[cfg(feature = "std")]`. WASM builds without
  `profile-coarse` (the normal case — feature is off by default)
  compile the timing code out entirely and no longer touch
  `Instant::now`.

### Removed

- `MFSK_PROFILE_COARSE` env-var dispatch in `coarse_sync` (added in
  `#130` as a host convenience to flip profiling without rebuild, but
  never exercised by any in-tree script, test, or CI job — the same
  diagnostic is reachable via `--features profile-coarse`). Embedded
  apps already enabled the feature flag directly, so their serial-log
  output is unchanged.

### Notes

- `mfsk-ffi-ft8` bumped to 0.6.7 in lock-step (no behavioral change in
  the FFI itself — it tracks `mfsk-core` so users see matching version
  numbers in release artifacts).

## 0.6.6 — cold-start slot bootstrap from coarse_sync top-5 DT median

Embedded `m5stack-s3-app` auto-sync previously consumed only
confirmed-decode DT median, so a cold start (or post-reset) with
zero confirmed decodes left the slot mis-aligned indefinitely
("BtnA required" in the log) until the operator intervened —
defeating the mountain-top "leave the laptop at home" use case
when the band is quiet for the first slot. WebFT8 faces the same
constraint with no manual sync available at all.

### Added

- **`mfsk_core::core::sync::bootstrap_dt_median(cands, top_k)`**
  (public): DT median over the top-`K` highest-score
  `SyncCandidate`s. Empirically — on `qso3_busy` + `191111`
  reference captures — `K=5` tracks confirmed-decode DT median
  within ±70 ms, while `K=10`/`20` wash out into 200–275 ms
  misses under false-candidate noise. Gated by new integration
  test `tests/ft8_coarse_sync_bootstrap.rs` (100 ms `|Δ|`
  budget). Uses `select_nth_unstable_by` for O(N) top-K
  partition + O(K log K) winner sort.

### Changed

- `embedded-shared::dual_core::SpeculativeOut` gains
  `bootstrap_dt_med: Option<f32>`, computed in
  `run_speculative_slot` right after pass1 returns and before
  the audio-window partition. `n_pass1 == 0` slots get `None`.
- `m5stack-s3-app/decode_pipeline.rs` auto-sync gains a fourth
  branch: when `n_dec == 0 && best_n == 0 && bootstrap_dt_med
  .is_some()`, the helper's value drives the bootstrap shift
  but `best_n` stays at 0, so the first confirmed-decode slot
  reclaims the HWM via the existing path. Soft anchor only.
- `m5stack-core2-app` / `m5stack-cores3-app` / `embedded-shared
  ::apps::rx_wavsim` consumers thread `bootstrap_dt_med`
  through as `_` (no slot-drift correction needed in wav_sim
  mode).

### Fixed

- Stale comment in `m5stack-s3-app/src/audio.rs` claimed the
  consumed shift hint was "pass1 candidate DT median"; in fact
  the pre-0.6.6 producer was confirmed-decode median. Corrected.

### Notes for WebFT8

`bootstrap_dt_median` is the same helper WebFT8 calls from its
cold-start path — no platform-specific wrapping required. Pass
the raw `Vec<SyncCandidate>` from either
`decode_block::coarse_sync` or `core::sync::coarse_sync::<Ft8>`.

## 0.6.5 — crates.io surface refresh (M5StickS3 FT8 controller PoC)

Non-functional patch. Refreshes the project's discoverable surface
on crates.io and docs.rs to reflect that the embedded port is no
longer aspirational:

- `mfsk-core/Cargo.toml` `description` rewritten to lead with the
  working `embedded-poc/m5stack-s3-app` M5StickS3 FT8 controller
  (LCD UI, BLE CI-V to IC-705, acoustic mic, QSO FSM, ~1.2 s
  post-SlotEnd decode on Xtensa LX7).
- `mfsk-ffi-ft8/Cargo.toml` `description` notes M5StickS3 end-to-end
  verification.
- `mfsk-core/src/lib.rs` "Why this exists" gains a fourth bullet
  for the handheld-controller use case, with cross-links to the
  M5StickS3 source crate and `docs/reference/MANUAL_M5STICKS3.md`.
- `README.md` opens with a hero photo of the device decoding five
  real on-air FT8 callsigns from a single 15 s slot
  (`docs/assets/m5sticks3-ft8-decode.jpg`).

Zero behaviour change vs 0.6.4 — no source code modifications, no
new features, no bug fixes. Pure metadata + docs.

## 0.6.4 — Goertzel per-symbol DFT (drop BASIS 120 KB internal-DRAM scratch)

Non-breaking minor — the embedded path's per-symbol DFT moves from
the BASIS sin/cos-table dot product to a generalised Goertzel
recursion. Same **magnitude** as the DFT bin
`|Σ x[n] exp(-jωn)|` for each FT8 tone — the complex output
differs by a fixed phase rotation per the standard generalised-
Goertzel `s[N-1] - exp(-jω)·s[N-2]` extraction, but FT8 downstream
consumes `|cs|²` so the rotation is irrelevant. **Zero caller-
provided scratch**, +0.16..+0.63 dB SNR improvement (f32 Goertzel
has more precision than the pre-existing Q15 BASIS dot product).

The 120 KB internal-DRAM win unblocks downstream embedded work that
couldn't fit alongside BASIS — most immediately, M5StickS3 Qso-mode
I2S bidirectional DMA descriptor allocation (`i2s_alloc_dma_desc:
allocate DMA buffer failed` on pre-0.6.4 firmwares).

### Added

- **`fill_symbol_spectra_goertzel`** (public): generalised
  Goertzel recursion for per-symbol DFT. Output-compatible with
  `fill_symbol_spectra_into` at the `cs: &mut [[Cmplx<f32>; 8]; 79]`
  level — modulo the phase rotation noted above — so downstream
  `sync_quality` / LLR / BP / OSD consume it without changes. **Call
  signature is shorter, not identical**: the BASIS `basis_re` /
  `basis_im` scratch slice arguments go away (5 args vs 7),
  reflecting the zero-scratch property. Sample-outer / tone-inner
  loop ordering for FPU pipeline parallelism on Xtensa LX6/LX7 —
  measured S3 stage3 1.47 s (matches BASIS asm dot-product speed)
  with zero internal-DRAM scratch.

### Changed

- **`process_candidates_into_with_cs_scratch_tuned`** and
  **`refine_candidates_into`**: the `not(fft-rustfft)` (embedded)
  branch now calls `fill_symbol_spectra_goertzel` instead of
  `fill_symbol_spectra_into`. Per-cell output is mathematically
  equivalent. Function signatures unchanged — BASIS scratch args
  still accepted for API back-compat with embedded callers
  (`embedded-shared::dual_core::stage3_split` /
  `pass2_split` thread them through). The args are ignored on the
  new path; will be removed in 0.7.0.

### Fixed

- **`fixed-point` now implies `nstep-half`**. The two features were
  independent in `[features]` but always co-enabled on every
  embedded target. Decoupling silently gave host `fixed-point`
  builds NSTEP=NSPS/4 while embedded used NSPS/2 — a completely
  different time-grid that made `host decode_block_into` produce a
  different decoded set than the embedded path on busy bands (e.g.
  4 decodes vs 7 on qso3_busy.wav). Coupling fixes the host-as-
  embedded-simulator contract.
- **`Plan3840Sc16`** (`core::dsp::fft_mixed_3840_sc16`): import
  `num_traits::Float` under `#[cfg(not(feature = "std"))]` so
  `f32::round()` at the i16 clamp resolves on `no_std` builds.
  Phase 1.7.7a follow-up — fixed the `Build (ft8)` / `Build (wspr)`
  / `Build (alloc ft8 fft-extern fixed-point)` CI matrix entries
  that had regressed.

### Deprecated (will remove in 0.7.0)

- `BASIS_SCRATCH_LEN`, `fill_symbol_spectra_into`,
  `symbol_spectra_direct_into`, `fill_symbol_spectra_into_generic`,
  `mfsk_ft8_basis_scratch_len` (FFI export). Superseded by
  `fill_symbol_spectra_goertzel` + zero-scratch call sites. Kept
  through 0.6.x for downstream `mfsk-ffi-ft8` consumers; the
  `dual_core::{init, pass2_split, stage3_split}` BASIS scratch args
  drop at the same time.

The `#[deprecated]` attribute itself is not added in this release —
applying it would trip the workspace-wide `-D warnings` CI gate via
the still-extant internal callers (FFI shim, embedded apps,
regression tests, plus the BASIS-internal `fill_symbol_spectra_into
→ ..._into_generic → symbol_spectra_direct_into` chain). The
attribute lands in the same 0.7.0 commit that drops the symbols,
with the call-site removals as the gating prep work. Until then
this CHANGELOG entry and the public-API doc comments are the
authoritative deprecation notice.

### Other notes

- Bundled `mfsk-ffi-ft8` from 0.6.3 to 0.6.4 (workspace-internal
  version tracking; the crate is `publish = false` — distributed
  via GitHub Releases, not crates.io).
- Phase 1.7.7 host validation tests (`tests/ft8_goertzel_vs_basis`,
  `tests/ft8_decode_block_fixed_point_baseline`) ship in this
  release as regression guards.

## 0.6.3 — `decode_block.rs` per-stage split + WSJT-X-faithful OSD

Non-breaking patch. Two interleaved storylines:

1. **Structural** — `mfsk-core/src/ft8/decode_block.rs` (3,517 lines)
   carved into a 7-sibling submodule tree (`docs/historical/CLEANUP_2026_05.md`
   ε plan, shipped as PRs #77 / #83 / #79 / #80 / #81 / #82). Pure
   refactor, zero behaviour change.
2. **OSD WSJT-X faithfulness** — `osd_strategy::try_fallback` (the
   host BpAllOsd Step-3 hook ε.6 carved out) rewired from mfsk-core's
   pre-existing brute-force `osd_decode` (ndeep=2, 4,186 patterns) /
   `osd_decode_deep(_, 3, _)` (ndeep=3, ~121k patterns) dispatch to
   the WSJT-X-faithful `nord=1 + npre1=1` (ndeep=2) /
   `+ npre2=1, ntau=14` (ndeep=3) entries, plus the 4 missing
   `ft8b.f90:422-459` post-decode validity gates and one
   `nsync<=10 && xsnr<-24` clamp-ordering bug fix. Issue #63.

### Improved

- **OSD pattern coverage reduced by ~25-700×** without WSJT-X /
  JTDX golden regression. New `osd_decode_npre1` (ndeep=2, ~165
  post-gate patterns) replaces the brute-force `osd_decode` on the
  FT8 OSD path; new `osd_decode_npre1_npre2` (ndeep=3, npre1 + a
  16,384-bucket `ntau=14` G-column XOR hash table) replaces
  `osd_decode_deep(_, 3, _)`. Both are WSJT-X line-for-line ports
  of `osd174_91.f90` with their `ntheta=10` / `ntheta=12`
  early-reject gates intact.
- **Four missing post-OSD validity gates** ported from
  `ft8b.f90:422-459`: all-zero codeword reject, message-type
  validity (`i3 > 5 || (i3==0 && n3 > 6)`), quirky free-text reject
  (`i3==0 && n3==2`), `unpack77` success guard. Applied in the
  `decode_block_multipass` retain_mut on host f32 (fixed-point
  doesn't surface OSD-pass results).
- **`nsync<=10 && xsnr<-24` gate clamp-ordering fix**.
  `recompute_snr_xsnr2` used to pre-clamp its return to `>= -24 dB`
  before the gate could test it — the source comment "on qso3_busy
  it had no effect in any case" already flagged the dead-letter.
  Function now returns raw xsnr; caller gates first, then clamps for
  display. Matches WSJT-X bit-for-bit on the order of operations.
- **Three `qso3_busy.wav` OSD CRC-luck phantoms eliminated**
  (`N1API F2VX 73` e=30, `N1API HA6FQ -23` e=25, `CQ EA2BFM IN83`
  e=31). The npre1 / npre2 / WSJT-X post-OSD gates all failed to
  filter these because their `nsync >= 13` puts them above
  `ft8b.f90:456`'s bail-out — they would also surface from WSJT-X
  proper at the same `max_cand=60` / `q_thresh=0.8` settings.
  mfsk-core adds an OSD-pass-specific `hard_errors > 22` ceiling
  (`OSD_HARDERRORS_MAX` in `decode_block/osd_strategy.rs`) that
  matches the empirical phantom boundary — `OSD_HARDERRORS_MAX` is
  the only mfsk-core-specific deviation from WSJT-X in this
  release (WSJT-X uses 36 universally; we restrict to 22 only on
  the OSD path because BP can legitimately produce e=20 cases like
  `N1PJT HB9CQK -10`).
- **WSJT-X 8-entry recall on `qso3_busy.wav`**: 7/8 maintained.
- **JTDX 18-entry recall** on `qso3_busy.wav`: **13/18** (was 16/18
  pre-0.6.3 — the dropped 3 are exactly the phantoms above; the
  JTDX golden inadvertently included them).
- **AP-on multipass extras** on `qso3_busy.wav`: **4/6** (was 5/6
  pre-0.6.3 — `CQ EA2BFM IN83` no longer surfaces because the OSD
  gate rejects it pre-AP rescore).

### Changed (internal — non-breaking)

- **`decode_block.rs` per-stage split.** Single 3,517-line file
  carved into 7 sibling submodules. Zero behaviour change.
  External callers see the same public paths via re-exports.

  | file | lines | role |
  |---|---:|---|
  | `decode_block.rs` | 423 | parent / facade |
  | `decode_block/types.rs` | 184 | audio sample + tunables |
  | `decode_block/spectrogram.rs` | 357 | `Spectrogram` + `compute_spectrogram` |
  | `decode_block/coarse_sync.rs` | 537 | Costas search + allsum |
  | `decode_block/fill_symbol_spectra.rs` | 601 | per-symbol DFT family |
  | `decode_block/process_candidates.rs` | 1,589 | engine + facade impls |
  | `decode_block/osd_strategy.rs` | 139 | OSD dispatch (`#63` hook) |
- **OSD scaffolding shared across ndeep=2 and ndeep=3.** New
  private `osd_setup_ldpc174_91` / `osd_npre1_pass` /
  `osd_result_from_best` helpers factor the WSJT-X-faithful setup /
  enumeration / result assembly so the ndeep=3 entry reuses them
  zero-cost and ndeep≥4 (nord-2/3/4 outer loop) can land later on
  the same scaffold.

### Added (public API — additive, FT8)

- `mfsk_core::fec::ldpc::osd::osd_decode_npre1(llr) -> Option<OsdResult>`
  — WSJT-X ndeep=2 entry. Replaces internal use of `osd_decode` on
  the FT8 OSD path; `osd_decode` stays for callers that prefer
  brute-force ndeep=2.
- `mfsk_core::fec::ldpc::osd::osd_decode_npre1_npre2(llr) -> Option<OsdResult>`
  — WSJT-X ndeep=3 entry. Replaces internal use of
  `osd_decode_deep(_, 3, _)` on the FT8 OSD path.

### Hardware (embedded)

- **Issue #61 closed.** `embedded-poc/m5stack-core2/` bench crate
  retired; new `embedded-poc/m5stack-core2-app/` consumes the new
  `embedded-poc/mfsk-app-shared/` carve-out (shared with
  `m5stack-s3-app`). Three-phase sequence in PR #76: Phase 1
  carve-out, Phase 2 Core2 app crate, Phase 3 bench retirement.
  Verified on M5StickS3 (S3 LX7, 7/8 golden) and M5Stack Core2
  (LX6, 7/8 per-slot from `qso3_busy.wav` via `wav_sim`, 400 alive
  ticks, stable heap).

### Infra

- **CI `Test (features = full)` job split into 5 parallel matrix
  entries** (#85). Old single ~22 min job → `Test (default)` +
  `Test (ft8 sweeps)` + `Test (q65 sweeps)` + `Test (uvpacket
  sweeps)` + `Test (lib + ft4 / fst4 / FST4 gated)`. Wall-clock
  ~10 min on a touch-everything PR.
- **`dorny/paths-filter@v3` gate** on the sweep matrix entries
  (#85). PRs that don't touch decoder source (`src/{ft8,fec,msg,
  core}/`, `Cargo.toml`, `.github/workflows/`) skip the slow sweep
  jobs entirely — ~3 min total for refactor / doc / embedded-poc-only
  PRs. Push to main always runs every sweep (post-merge regression
  safety net).

## 0.6.2 — host pipeline matches embedded subtract + cs-source

Non-breaking patch. After v0.6.1 unified the per-candidate inner,
the host pipeline still diverged from embedded `decode_block` on
two upstream axes: the cs spectra source (host computed cs from
cd0, embedded fills from 12 kHz audio) and the SIC subtract
function (host used a constant-amplitude weighted subtract,
embedded uses WSJT-X-style channel-aware LPF subtract). Both
divergences are eliminated in 0.6.2; host now matches embedded
bit-for-bit on the cs values it feeds the unified inner.

### Improved

- **Host `decode_frame_subtract_with_ap` recall on busy bands.** On
  `qso3_busy.wav` AP-on with operator context (`mycall=K1JT`,
  `hiscall=HA0DU`):
  - **14 → 18 decodes** total.
  - **JTDX-extras coverage 1/6 → 5/6.** Surfaces CQ EA2BFM,
    KD2UGC F6GCP, K1BZM EA3CJ on top of the F5RXL CQ already
    caught in 0.6.1. The single remaining miss
    (K1BZM DK8NE -19) requires a wider AP-list / callsign hash
    table — out of scope for this patch.
- **Host single-pass `decode_frame_with_ap`** unchanged at 14/18
  (single-pass doesn't benefit from the improved subtract).
- **Embedded `decode_block`** unchanged at 16/18 (the changes touch
  only the host driver path).

### Changed (internal — non-breaking)

- `ft8::decode::process_candidate` — cs-source rewired from
  `symbol_spectra(cd0, i_start)` to `fill_symbol_spectra(audio,
  refined.freq_hz, refined.dt_sec, SyncOnly+DataOnly)`, matching
  embedded's per-symbol-region DFT directly on 12 kHz audio. cd0
  still used for `fine_refine_3stage` and `sync_cv` computation.
- `decode_frame_subtract_with_ap`,
  `decode_frame_subtract_with_known_and_ap_inner`, and
  `decode_sniper_sic` — sequential subtract switched from
  `subtract_signal_weighted` (constant amplitude × QSB-aware
  partial gain) to `subtract_signal_lpf` (WSJT-X-style channel-
  aware LPF subtract, matching `decode_block_multipass`).

### Removed (technically breaking, in practice no consumers)

- `mfsk_core::ft8::subtract::subtract_signal` — was a thin wrapper
  around `subtract_signal_weighted(audio, result, 1.0)`. No
  in-tree, embedded-poc, or mfsk-ffi-ft8 callers (verified via
  `grep`).
- `mfsk_core::ft8::subtract::subtract_signal_weighted` — replaced
  everywhere by `subtract_signal_lpf` (which doesn't accept a
  partial gain — the QSB-aware attenuation it provided is no
  longer needed since LPF subtract handles channel variation
  natively).
- `mfsk_core::ft8::llr::symbol_spectra` — removed; was only used
  by `process_candidate` which now calls
  `decode_block::fill_symbol_spectra` directly. The protocol-
  generic `core::llr::symbol_spectra::<P>` (used by FT4 / JT9 /
  uvpacket / WSPR pipelines) is untouched.
- `qsb_partial_gain` (private) — only consumer was the now-removed
  weighted subtract.

The removed `subtract_signal*` and `symbol_spectra` items had no
out-of-tree callers in the workspace; the FT8 subtract public
surface is now `subtract_signal_lpf` + `refine_signal_freq`.

### Deferred / scoped out

- **Embedded recall++** beyond 16/18 on `qso3_busy.wav`. The
  remaining 2 misses (K1BZM DK8NE -19, WA2FZW DL5AXX -15) are
  intrinsically below `decode_block::coarse_sync`'s sync-min
  threshold even at sync_min=0.5 / max_cand=200; a WSJT-X-faithful
  dual-window NMS prototype was tried during the v0.6.2 plan but
  produced identical 16/18 hit count, so it wasn't worth the code
  volume. Surfacing those entries needs either a different sync
  algorithm or a wider AP-list — both deferred.
- **Embedded fixed-point AP** support (apmag in Q3i8) — still
  tracked for 0.7.x.

### Test floor changes

- `tests/ft8_qso3_apon_recall.rs::JTDX_EXTRAS_HARD_FLOOR_MULTIPASS`:
  1 → **5** (locks in the host multipass recall jump).

## 0.6.1 — host/embedded per-candidate pipeline unification

Non-breaking patch. After v0.6.0 unified the FT8 coarse-sync path,
the *per-candidate* decode body still diverged: host's
`process_candidate` (decode.rs) ran a hand-written eager-LLR + BP +
OSD + AP staircase; embedded's `process_candidates_with`
(decode_block.rs) ran a lazy-LLR staircase with tighter sync gates.
On busy bands (e.g. `qso3_busy.wav`), embedded `decode_block` caught
16/18 of the JTDX golden but host `decode_frame_with_ap` only 14/18.

This patch extracts a shared `process_one_candidate_inner` from
`process_candidates_with` and routes the host pipeline through it.
Both paths now run the *exact same* LLR / BP / OSD / AP staircase
on the per-candidate cs spectra. The remaining 14-vs-16 gap on this
WAV is upstream of the inner — the cs-source itself differs (host
`core::llr::symbol_spectra(cd0, i_start)` vs embedded
`fill_symbol_spectra(audio, freq, dt, mask)`) and is tracked as a
separate refactor.

### Added (host f32 only — `#[cfg(feature = "fft-rustfft")]`)

- `decode_block_with_ap(audio, freq_min, freq_max, sync_min, depth,
  max_cand, ap_hint) -> Vec<DecodeResult>`. AP-aware variant of
  `decode_block`; mirrors its behaviour exactly when
  `ap_hint = None` and runs the full WSJT-X iaptype loop (5..12)
  per candidate when `Some(&ap)`. Preferred entry point for
  mountain-top apps that want full AP rescue from a single call.
- `decode_block_with_ap_tuned(...with bp_max_iter + strictness)` —
  same with runtime-tunable BP iterations and strictness.

Embedded fixed-point builds (`#[cfg(feature = "fixed-point")]`)
keep their existing iaptype-1-only hardcoded path; full AP for that
build is deferred — `apmag = max(|llra|)*1.01` is f32-domain and
`ApHint::build_ap` heap pressure exceeds Core2 stage-3 budget.

### Changed (internal — non-breaking)

- Embedded OSD pass IDs shifted: 4/5/6/7 → 14/15/16/17, freeing the
  WSJT-X-canonical 5..12 range for the AP iaptype loop. Pre-flight
  grep confirmed no in-tree test gates on `pass == N` for `N ≥ 4`;
  out-of-tree consumers reading `DecodeResult::pass` for OSD-decoded
  results need to read 14/15/16/17 instead of 4/5/6/7.
- Host `process_candidate` body shrinks ~250 lines → delegate to the
  unified inner. Outer prelude (downsample, fine_refine_3stage,
  symbol_spectra, nsync gate, EqMode cs choice) preserved.
- The unified inner emits one decode per candidate after running
  `unpack77` + plausibility gate (the embedded path's behaviour).
  Old host path was lenient — it returned any CRC-converged
  77-bit codeword without unpacking. The change is what tightens
  busy-band recall vs host's old behaviour, but means
  *synthesised-with-arbitrary-bits* round-trip tests need a real
  FT8 message; updated 5 affected tests to use
  `pack77("CQ", "JA1ABC", "PM95")`.

### Improved

- Host `decode_frame_with_ap` and `decode_frame_subtract_with_ap`
  now share the WSJT-X-faithful staircase with `decode_block`. The
  qso3_busy.wav 14-vs-16 gap on the JTDX golden is structurally
  closed for everything *post* coarse-sync; only the cs-source
  divergence remains.

### Internal

- New private `process_one_candidate_inner` in `decode_block.rs`
  (`pub(super)`-visible to the FT8 module). Generic over `LlrT` so
  it serves both f32 (host + embedded f32-rustfft) and Q3i8
  (embedded fixed-point) builds with bit-identical behaviour when
  `ap_hint = None`.
- `process_candidates_with` collapsed into
  `process_candidates_with_ap` (the same body now takes
  `Option<&ApHint>`); call sites passing `None` are bit-identical to
  the prior shape.

## 0.6.0 — FT8 sync consolidation, refactor audit, AP iaptype 2

**Breaking** — bundled refactor that closes #40, #46, #48, and most of
the #49 cat A / B / C audit. After v0.6.0 the FT8 sync path has a single
WSJT-X-faithful implementation (`decode_block::coarse_sync`) and the
"`ft8::sync` namespace" wrappers that drifted in #40 are gone. The user-
visible behaviour change for callers already on 0.5.12 is small —
`freq_hint: Option<f32>` becomes a no-op on FT8 wide-band paths, and the
`ft8::sync` thin-wrapper functions are removed (out-of-tree callers must
turbofish the generic).

### Removed

- `mfsk_core::ft8::sync::coarse_sync` (#46) — was already a one-line
  trampoline. Use `mfsk_core::ft8::decode_block::{compute_spectrogram,
  coarse_sync}` directly:
  ```rust
  // before (0.5.12)
  let cands = mfsk_core::ft8::sync::coarse_sync(&audio, 200.0, 2800.0, 1.3, None, 50);
  // after (0.6.0)
  let spec = mfsk_core::ft8::decode_block::compute_spectrogram(&audio, 2800.0);
  let cands = mfsk_core::ft8::decode_block::coarse_sync(&spec, 200.0, 2800.0, 1.3, 50);
  ```
- `mfsk_core::ft8::sync::{compute_spectra, fine_sync_power,
  fine_sync_power_split, refine_candidate, refine_candidate_double}`
  (#49 cat B). All five were one-line wrappers around
  `mfsk_core::core::sync::*::<Ft8>` — call the generic directly.

### Changed (behaviour, signatures unchanged)

- **`freq_hint: Option<f32>`** on every `decode_frame*` and
  `decode_sniper*` is silently ignored. `decode_block::coarse_sync` does
  not honour candidate-score promotion; sniper paths
  (`decode_sniper_ap`) constrain `freq_min`/`freq_max` ±250 Hz around
  `target_freq` so the loss is contained.
- **`symbol_spectra` / `score_costas_block` / `fine_sync_power*` /
  `refine_candidate*` take `i32` for sample-index parameters** (#40 / #45).
  The host wide-band path was casting `i_start = ((dt+0.5)*200) as
  usize`, saturating any negative-dt candidate to 0 and silently
  misaligning the symbol grid by ~1.75 symbols. Threading `i32` through
  with WSJT-X all-or-nothing per-symbol boundary checks (= the same
  behaviour `decode_block::fill_symbol_spectra_via_cd0` already had) lifts
  host AP-off recall on `qso3_busy.wav` from 5/8 → **7/8** (matches the
  embedded `decode_block`). Out-of-tree callers of these
  `core::sync` / `core::llr` / `ft4::refine_fine` functions need to update
  their callsites to pass `i32`.

### Added

- **AP iaptype 2 — mycall-only lock** (`ft8::decode::process_candidate`).
  Mirrors WSJT-X `lib/ft8/ft8b.f90` "MyCall ??? ???" pass: locks bits
  0–28 + i3=001 (~32 bits) without forcing call2 / grid. Triggered as
  a new pass (id 5) whenever `ApHint::call1` is set; surfaces follow-up
  replies whose sender isn't the call2 in operator context. JTDX uses
  this iaptype to land entries that pass 8 (mycall + dxcall) cannot
  reach. `check_result` already rejects decodes whose unpacked text
  doesn't contain the AP-locked call1, so iaptype 2 cannot leak phantoms.

### Improved

- **FT8 host wide-band recall on busy bands**. `qso3_busy.wav`:
  WSJT-X 8-entry golden 5/8 → 7/8 (CQ F5RXL IN94 -3 dB now decodes; 3
  phantoms above 2 kHz eliminated). JTDX 18-entry golden 13/18 → 16/18.
  AP-on JTDX-extras 0/6 → 1/6.

### Public API graduation (non-breaking)

- `decode_block::coarse_sync` and `compute_spectrogram` graduate from
  `#[doc(hidden)]` to officially public — they are the canonical FT8
  surface, not benchmarking aids (#48 step B).
- `decode_block::Spectrogram`, `coarse_sync_with_allsum`,
  `coarse_allsum_len`, `precompute_coarse_allsum` /
  `precompute_coarse_allsum_into`, `refine_candidates_into`,
  `process_candidates_into_with_cs_scratch` /
  `_with_cs_scratch_tuned` graduate as well — embedded consumers
  (`embedded-shared::dual_core` / M5Stack apps) and `mfsk-ffi-ft8` were
  already depending on them; the disclaimer wasn't accurate (#49 cat C).

### Internal

- **WAV-loader test consolidation** (#49 cat A). 14 near-duplicate
  RIFF/WAVE parsers across `tests/ft8_*`, `tests/q65_*`, `tests/fst4_*`,
  `tests/wspr_*`, `tests/ft4_*`, `tests/jt9_*` collapse into four
  helpers (`load_wav_i16` / `load_wav_f32` strict + `_opt` soft) in
  `tests/common/mod.rs`. Net 342 lines deleted.

### Fixed (additional)

- **Host `decode_frame_subtract_with_ap` is now WSJT-X-faithful**.
  Mirrors the structure of `decode_block::decode_block_multipass` (which
  is a faithful port of `lib/ft8/ft8b.f90:432-437`). Two changes from
  pre-v0.6.0:
  1. **Fixed `sync_min` across all 3 passes** (was sync_min × {1.0, 0.75, 0.5}).
     Progressive relaxation lets phantoms slip through later passes when
     SIC artefacts dominate the residual.
  2. **Sequential subtract within each pass** (was batch-after-pass).
     Each accepted decode immediately subtracts so the next candidate
     in the same pass sees a cleaner residual.

  Pass termination matches WSJT-X (skip pass 2 when pass 1 returned 0;
  skip pass 3 when pass 2 returned no NEW). No recall change on the
  qso3_busy.wav reference (the host's remaining 14/18 vs embedded's
  16/18 gap on that WAV is structurally downstream — see "Known
  limitation" below) but materially more faithful in shape and will
  matter on WAVs where SIC artefact phantoms differ.

### Deferred

- `decode_block_with_ap` (ROADMAP A0' / "AP-on extras parity"). The
  remaining 2-entry gap between embedded `decode_block` (16/18 of the
  JTDX 18-entry golden on `qso3_busy.wav`) and host `decode_frame*`
  (14/18) is **inside the per-candidate processing** —
  `process_candidates_tuned` (decode_block's path) and
  `process_candidate` (decode.rs's path) are separate decoder pipelines
  with different LLR / BP staircase / sync_quality gates, and the 3
  extra decodes embedded catches (KD2UGC F6GCP, CQ EA2BFM, K1BZM EA3CJ)
  pass `process_candidates_tuned`'s gates but not `process_candidate`'s.

  Unifying the two pipelines is a non-trivial refactor (the embedded
  path is fixed-point-aware, the host path uses f32 throughout) — it's
  a 0.7.0-class change. Tracked for that release. v0.6.0 ships with
  the host pipeline structurally aligned to the WSJT-X-faithful
  multipass shape (above) but with the per-candidate processing still
  on the host-specific code path.
- `Protocol::Sync` associated-type (#48 option A) — full type-system
  enforcement against future protocol-sync drift. Scoped out: the blast
  radius is every `impl Protocol` site + the embedded feature matrix,
  for benefit that's speculative for non-FT8 protocols (none have shown
  drift in 18 months post-fork). Tracked in #48.

Older releases (0.5.12 and earlier — 0.1.0 through the initial
embedded port and Q65/abstraction unification work) are archived in
[`docs/historical/CHANGELOG-0.x.md`](docs/historical/CHANGELOG-0.x.md).
