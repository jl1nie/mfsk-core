# Changelog

## 0.7.5 — JT9 AWGN SNR sweep + jt9sim build

### Added

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
