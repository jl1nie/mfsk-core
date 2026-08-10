# FST4 sensitivity benchmark — environment setup

How to reproduce the FST4 AWGN/fading SNR sweep (`tests/fst4_sweep.rs`)
from a clean checkout on any machine. This is the harness used to
measure recall vs. SNR against WSJT-X's own `fst4sim` synthetic
signals and compare it to the published sensitivity table (see
[#146](https://github.com/jl1nie/mfsk-core/issues/146) for the
investigation that motivated tightening this up).

The whole pipeline is: **WSJT-X Fortran source → `fst4sim` binary →
WAV corpus on disk → `cargo test --ignored` reads the WAVs and prints
a recall table.** Nothing here needs network access at test time —
only the one-time build step touches WSJT-X source.

## 1. Prerequisites

| Requirement | Ubuntu/Debian package | Purpose |
|---|---|---|
| `gfortran` | `gfortran` | compiles `fst4sim.f90` + supporting lib |
| `gcc` | `build-essential` | compiles `gran.c` |
| FFTW single-precision | `libfftw3-dev` | linked as `-lfftw3f` |
| WSJT-X source tree | — (see below) | provides `lib/fst4/*.f90` |

```sh
sudo apt-get install gfortran build-essential libfftw3-dev
```

**WSJT-X source.** `scripts/build_fst4sim.sh` only needs the `lib/`
subtree (no Qt build, no `cmake` step) — any WSJT-X checkout with the
`fst4/fst4sim.f90` simulator works, including the upstream tree:

```sh
git clone https://github.com/jl1nie/WSJT-X.git ../WSJT-X
```

Per the repo-root `CLAUDE.md` test-fixture-path rule, never hardcode
an absolute path to this checkout — `build_fst4sim.sh` takes it as an
explicit argument (falling back to the `../WSJT-X` sibling-of-repo-root
convention only as a default), so the setup works the same whether
your clone lives at `/home/<you>/src/WSJT-X` or anywhere else.

## 2. Build `fst4sim`

```sh
scripts/build_fst4sim.sh [/path/to/WSJT-X] [out-dir]
# defaults: WSJT-X-dir = ../WSJT-X (sibling of this repo), out-dir = target/fst4sim/
```

Produces `target/fst4sim/fst4sim`. This is a native synthesis-only
binary (no SDR/audio I/O) — generating a full 300 s FST4-300 trial
takes well under a second, so the *build* is the only slow step here
(~14 compile units, a few seconds).

Sanity-check the build:

```sh
target/fst4sim/fst4sim "CQ JL1NIE PM95" 60 1500 0.0 0.0 0.0 1 -20 F
ls 000000_*.wav   # one 60 s AWGN trial at -20 dB
```

## 3. Generate the WAV corpus

Two generator scripts cover different needs:

- **`scripts/gen_fst4_sim_wavs.sh`** — one clean AWGN file per
  sub-mode at a fixed SNR (-5 dB). Cheap smoke-test corpus for
  `MFSK_FST4_SIM_ASSETS_DIR`-based single-frame decode tests.
- **`scripts/gen_fst4_sweep_wavs.sh`** — the full sensitivity sweep:
  every sub-mode (FST4-15/30/60/120/300) × 4 channel conditions
  (`awgn`, `ccir_good`, `ccir_moderate`, `ccir_poor`) × a per-mode SNR
  grid × `TRIALS` repeats. This is what `tests/fst4_sweep.rs` reads.

```sh
scripts/gen_fst4_sweep_wavs.sh [fst4sim-path] [out-dir]
# defaults: fst4sim-path = target/fst4sim/fst4sim
#           out-dir      = embedded-poc/assets/fst4_sweep/
```

The script is **idempotent / incremental** — it only generates files
that don't already exist, so widening the SNR grid or bumping
`TRIALS` and re-running just tops up the corpus rather than
regenerating everything.

### Tuning the SNR grid — avoid measurement censoring

`MODE_SNRS` in the script sets, per mode, the list of SNR points to
generate. **The grid must extend at least 1-2 dB past the mode's
official WSJT-X threshold**, or the sweep can't observe the real 50%
recall crossing at all — if the weakest point you generate still
decodes reliably, the "reported" sensitivity you read off the sweep
is just an artifact of where the grid happened to stop, not the
decoder's actual floor. (This exact mistake produced a spurious
3-5 dB "gap" in #146 for the three slower sub-modes.) Cross-check
against the published thresholds already documented as doc-comments
in `mfsk-core/src/fst4/mod.rs`:

| Mode | WSJT-X threshold (AWGN, dB) |
|---|---|
| FST4-15  | -20.7 |
| FST4-30  | -24.2 |
| FST4-60A | -28.1 |
| FST4-120 | -31.3 |
| FST4-300 | -35.3 |

`TRIALS` controls resolution near the crossing — 10 trials only
resolves recall in 10% steps, which is too coarse to pin down a 50%
point with confidence; 20+ is a reasonable default for a serious
sweep.

## 4. Run the sweep

```sh
MFSK_FST4_SWEEP_DIR=embedded-poc/assets/fst4_sweep \
  cargo test --test fst4_sweep --release \
  --features fst4,fft-rustfft,parallel \
  -- --ignored --nocapture
```

- `--release` matters — the sweep runs coarse-sync over hundreds of
  WAVs per mode; debug builds are painfully slow.
- `--features parallel` enables the `rayon`-parallelised coarse-sync
  fi-loop (see #139) — cuts wall time substantially, especially for
  the longer periods with more frequency bins.
- Output is a plain recall table (decodes / trials per mode × channel
  × SNR cell) — no pass/fail assertions, this is a measurement tool,
  not a regression gate.
- `MFSK_FST4_SWEEP_DIR` defaults to
  `<repo>/embedded-poc/assets/fst4_sweep` (relative to
  `CARGO_MANIFEST_DIR`) if unset, so it only needs to be passed
  explicitly when using a corpus generated elsewhere.

## 5. Reading the results

For each mode, find the lowest SNR where recall is still ~50% — that
is the empirical threshold to compare against the published table
above. A few things worth checking alongside the headline number:

- Is the crossing SNR *inside* the generated grid, or pinned at the
  weakest point? If it's pinned, the grid still isn't deep enough —
  extend `MODE_SNRS` further and regenerate (step 3 is incremental,
  so this is cheap).
- Compare AWGN vs. the three CCIR fading conditions separately — a
  gap that only shows up under fading points at a different part of
  the pipeline than a gap present even in AWGN.
- If investigating a specific candidate-pipeline hypothesis (e.g.
  search-space sizing, frequency-refine precision), it's usually
  faster to reason from `core/sync.rs::SyncDims` directly (`n_freq`,
  `jz`/`n_lag`, `df`) than to infer it from recall numbers alone —
  the sweep is for confirming/measuring effect size, not for
  discovering where in the pipeline a gap originates.

## 6. Diagnose before fixing — don't guess-and-sweep

A full sweep tells you *how big* a gap is, not *where in the
pipeline* it comes from. Picking a plausible-sounding mechanism (a
formula that happens to numerically match the measured gap), coding
a fix for it, and re-running the whole sweep to check is the slow
and unreliable way to find out — a coincidental numeric match can
send a whole re-implementation cycle in the wrong direction before
the sweep re-run (which itself takes tens of minutes) tells you it
didn't move the needle.

The reliable order is:

1. **Sweep once** to find a specific failing (mode, channel, SNR)
   cell with partial recall (e.g. 2/20) — a cell that sometimes
   decodes and sometimes doesn't is far more diagnostically useful
   than one that's 0/20 or 20/20 everywhere.
2. **Pick a handful of individual trial WAVs from that cell** and
   call the pipeline stages directly in a throwaway `#[ignore]`d
   test (see `fst4_60_diag_weak_trial` in `tests/fst4_sweep.rs` for
   the pattern) — `engine::sync::coarse_sync` first to check whether
   the correct candidate is even found and with what score relative
   to `sync_min`, then `engine::pipeline::process_candidate_basic` on
   that exact candidate to see whether it's coarse-sync,
   fine-refine, LLR, or BP/OSD that actually fails.
3. **Only once the failing stage is identified**, make the code
   change and re-verify with a *targeted* subset of the sweep (next
   section) rather than the full grid.

This is exactly how the #146 investigation caught a wrong turn: a
"coarse-sync only integrates 25% of the slot" theory numerically
matched the measured ~2.6 dB gap almost exactly (`sqrt(4) = 3.01
dB`), a fix was implemented and passed regression, but the full
sweep re-run came back byte-identical — because step 2's diagnostic
(run *after* the fact) showed the correct candidate was already
being found in every trial, well above threshold. The real cause
(FST4 silently inheriting the FT8-calibrated `LLR_NSYM_MAX = 3`
instead of WSJT-X's own 8-symbol correlation depth) was only found
by tracing the failing candidate through `process_candidate_basic`
directly.

## 7. Re-verifying a fix — don't re-run the whole grid

Once you have a specific fix and already know the approximate 50%
crossing from a prior full sweep, re-running all 5 modes × 4 channels
× full SNR grid × `TRIALS` repeats to confirm a few tenths-to-a-few
dB of movement is wasted time (the full grid run takes on the order
of tens of minutes). Narrow the re-verification to what the fix
could plausibly move:

- **Channel**: AWGN only, unless the fix specifically targets fading
  robustness (equalization, Doppler tracking) — the CCIR conditions
  add proportionally more wall time than diagnostic value for a
  baseline-sensitivity fix.
- **SNR window**: only the 2-3 dB band straddling the *previously
  measured* crossing (e.g. if the old crossing was -25.3 dB, check
  -27 to -23 dB) — no need to re-sweep the already-known 0%/100%
  plateaus on either side.
- **Modes**: if the fix is mode-independent (applies identically to
  all five sub-modes, as an `LLR_NSYM_MAX`-style constant does), one
  or two modes are enough to confirm the mechanism before spending
  time on the rest — you don't need all five to re-confirm the same
  code path.
- Compute the new 50% crossing the same way as before (linear
  interpolation between adjacent SNR points) and diff directly
  against the prior crossing rather than re-deriving from scratch.

## 8. FST4-60A decode speed — OSD-escalation gate over-triggering, ~8.4× win (2026-07-20)

New investigation, same methodology as the FT4 coarse-sync/speed work
(`~/.claude/plans/dapper-soaring-nest.md`): `docs/notes/BENCHMARKS.md`'s
"Decode speed" table had FST4-60A as the single slowest golden-WAV
decode of any protocol (2.60 s for a 60 s slot). Unlike FT4, the
bottleneck turned out **not** to be coarse-candidate redundancy.

**Profiling** (`fst4_60_diag_candidate_cost_split`, `tests/fst4_sweep.rs`,
new): on the WSJT-X FST4-60 golden WAV (`210115_0058.wav`), `coarse_sync`
found only 50 candidates across 31 distinct frequencies (1.6× — nowhere
near FT4's 4.5×), and `coarse_sync` + `fst4_sync_search` together cost
under 110 ms combined. The real cost was `symbol_spectra` + LLR + BP +
OSD: **8054 ms** summed across all 50 candidates, out of a
`decode_frame` production wall-clock of 2278 ms.

**Root cause, confirmed against WSJT-X source** (`lib/fst4_decode.f90`,
`lib/fec/ldpc240_101` equivalents): two compounding factors.

1. `Ldpc240_101::decode_soft` (`fec/ldpc240_101/mod.rs:148-197`) tries
   OSD **twice** whenever BP fails and OSD is requested: once on the raw
   channel LLR, and — faithfully matching WSJT-X's `decode240_101.f90`
   `zsave` mechanism (issue #146) — again on the running BP soft-estimate
   sum. Both are genuinely needed (issue #146 measured real recall gain
   from the zsum retry), so this isn't a bug, but it doubles the cost of
   every OSD attempt.
2. `process_candidate_basic`'s OSD-escalation gates
   (`osd_attempt_min`/`osd_depth3_min`, `core/pipeline.rs`) were
   hardcoded `(12, 18)` for every protocol except FT4 (which already got
   an `N_SYNC`-scaled version, issue #72) — calibrated against FT8's
   `N_SYNC=21`. FST4's `N_SYNC=40` (5 blocks × 8-symbol Costas) makes
   `18` a *far looser* bar than FT8's own 18/21=86% (18/40=45%), so
   roughly half of all real candidates escalated into the OSD depth-3
   tier regardless of actual signal quality.
   `fst4_60_diag_osd_escalation` (new) measured this directly on the
   golden WAV: 24 of 50 candidates attempted OSD depth-2/3 (only 1
   succeeded, 3.7 s combined), 2 escalated further to depth-4 (0
   succeeded, 2.1 s combined) — on a WAV whose real signals were all
   found comfortably under either threshold.
   The comment this replaced claimed FST4's depth-escalation gate "was
   already tuned separately" (issue #146) — checked
   `docs/historical/CHANGELOG-0.6-0.7.md` for that tuning and found
   none: issue #146 tuned `bypass_osd_score_min`
   and AWGN sensitivity, never the `(12, 18)` pair, which appears to
   have been an untouched FT8 inheritance all along (the same kind of
   stale-comment trap the FT4 investigation hit more than once).

**First attempt: reuse FT4's exact scaling formula** — extend
`P::ID == Ft4`'s `((12*N_SYNC+10)/21, (18*N_SYNC+10)/21)` to FST4 too
(→ `(23, 34)` for `N_SYNC=40`). Golden-WAV `decode_frame` dropped to
197 ms (~11.5×), recall unchanged (1/1). **But a controlled A/B on
identical code** (git-stash the pipeline.rs change, rerun the exact same
`fst4_snr_sweep` AWGN cells, restore) showed a real regression: 50%
crossing moved **-27.6 dB → -27.1 dB** (~0.5 dB) — confirmed against
`docs/historical/CHANGELOG-0.6-0.7.md`'s documented 0.7.2 baseline
(`-27.62 dB`), which the
pre-fix rerun reproduced almost exactly (`-27.625 dB`), ruling out a
stale-baseline explanation. Unlike FT4's identical-looking fix (which
only ever *raised* a threshold nsync could never reach, pure upside),
FST4's `nsync` genuinely spans into the `[18, 34)` range for some real,
recoverable signals — this is a case where "the same formula, the
opposite direction" carries real recall risk, exactly as flagged before
measuring.

A follow-up per-candidate diagnostic
(`fst4_60_diag_osd_depth34_nsync_floor`, checks decoded message against
`GOLDEN_MSG`, not just CRC-24 pass) tried to find the real depth-3/4
rescue floor directly and came back empty (0 rescues in the
near-crossing AWGN region) — contradicted by the real sweep regression,
so this diagnostic's candidate selection doesn't fully match production
(left as a known gap, not chased further; the per-candidate diagnostics
in this file remain useful for cost/count profiling, just not proven
reliable for recall-floor calibration the way `ft4_diag_smax_calibration`
was for FT4). Reverted to hand-sweeping the real `fst4_snr_sweep` test
directly instead — same fallback FT4_BENCHMARK.md section 6 used.

**Calibration, against the real recall test**:

| `osd_attempt_min` | `osd_depth3_min` | AWGN 50% crossing |
|---:|---:|---:|
| 12 (unchanged) | 34 (fully scaled) | ≈ -27.3 dB |
| 12 (unchanged) | 24 | ≈ -27.44 dB |
| 12 (unchanged) | **20** | ≈ -27.56 dB |

`osd_attempt_min` stayed at the original `12` throughout (raising it
alone, in an earlier intermediate step, was the main driver of the
first attempt's 0.5 dB loss — most real near-threshold candidates need
*some* OSD attempt, just not always depth-3). `osd_depth3_min = 20`
(barely above the original `18`) reproduced the documented baseline
closely enough to call it noise: AWGN -27.56 vs -27.62 dB documented,
and a full 4-channel re-run matched almost exactly —

| Channel | Documented (pre-fix, CHANGELOG 0.7.2) | Measured (post-fix, this pass) |
|---|---:|---:|
| AWGN | -27.62 dB | -27.56 dB |
| CCIR good | -25.78 dB | -25.78 dB |
| CCIR moderate | -25.43 dB | -25.43 dB |
| CCIR poor | -24.50 dB | -24.50 dB |

— 3 of 4 channels bit-identical to the documented figure, AWGN within
0.06 dB (well inside 20-trial sampling noise). Spot-checked FST4-120
(-30.71 vs documented -30.70 dB) and FST4-300 (-34.78 vs documented
-34.78 dB, exact) at AWGN — both sub-modes share the same `N_SYNC=40`
and confirmed equally unaffected. FST4-15/30 were not independently
re-verified (time-boxed; the mechanism is `N_SYNC`-driven and identical
across all five sub-modes, so this is a reasonable but not
independently-confirmed extrapolation — revisit if a future FST4-15/30
regression surfaces).

**Final fix** (`core/pipeline.rs`): FST4 gets its own branch —
`osd_attempt_min` stays the shared default `12`; `osd_depth3_min = 20`,
a hand-calibrated value verified against the real sweep, not derived
from the `N_SYNC` ratio formula (that formula only reproduced FT8/FT4
correctly; FST4 needed direct calibration).

**Final numbers**: golden WAV `decode_frame` wall-clock
**2278 ms → 272 ms (~8.4×)**, recall unchanged (1/1, same 2 total
decodes). `llr+bp+osd` summed cost 8054 ms → 3262 ms. Full non-ignored
suite (922 tests) and `-D clippy::perf -D warnings` green throughout.

Lesson, stated plainly (same one section 7 draws from the opposite
direction): a fix that looks identical to a previously-verified one
(same formula, same code shape) can still carry different risk when the
underlying quantity behaves differently for the new protocol — FT4's
`N_SYNC` scaling only ever unlocked dead code; FST4's tightened *live*
code that some real signals depend on. Controlled A/B against the exact
current codebase (not a possibly-stale prior baseline) is what caught
it before shipping.
