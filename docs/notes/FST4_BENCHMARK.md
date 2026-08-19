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

## 9. Attribution ablation — do two rescue mechanisms overlap? (2026-08-17)

Sections 6-7 cover finding *where* a gap is and re-verifying *that* a
fix moved it. This section covers a third question that neither
answers: when you have **two** mechanisms that each rescue failing
trials, do they rescue the *same* trials or *different* ones?

That distinction decides whether an embedded escalation ladder has to
pay for both (issue #310) or can carry only the cheaper one. It arose
concretely from issue #306/#308/#311: FST4's OSD had two candidate
rescue paths — WSJT-X's `i0±1` timing-jitter retry, and a selective
fallback to the pre-#198 unpruned combinatorial OSD search.

### Counts cannot answer it; only per-trial identity can

Two mechanisms that each recover "2 of 5" are **indistinguishable from
aggregate recall numbers**. They could be

- **substitutable** — the same 2 trials, so either one alone buys the
  full benefit,
- **additive** — 2 different trials each, so the union is 4 and
  dropping either costs recall, or
- **interacting** — neither works alone and only the combination
  rescues anything.

So an ablation for this purpose must record **which trials** each
configuration decodes, as sets, and print them. A table of pass counts
is not enough, and a "both mechanisms help by about the same amount"
summary is actively misleading.

### The knobs

- **OSD search**:
  `mfsk_core::fec::ldpc240_101::fst4_osd_diag_force_old(bool)` —
  `internal-testing`-gated, a process-global `AtomicBool`. Forces
  `fst4_osd_decode` onto the old unpruned search. Reads `false` in any
  non-`internal-testing` build, so it cannot affect a release artifact.
  **Not thread-safe**: run with `--test-threads=1`.
- **Timing offsets**: not exposed on the production entry point. See
  trap 2.

### Trap 1 — hardcoded trial indices are not portable

The original finding named trials `[2, 15, 33, 45, 67]` from a
**100-trial-per-cell** corpus. A 20-trial-per-cell generation of the
same corpus has no trials 33/45/67 at all, and its trial 2 is a
*different waveform* — `gen_fst4_sweep_wavs.sh` regenerates from
`fst4sim`, it does not extend an existing set. So those indices
silently select the wrong signals, or none, in any other environment.

This is the same class of mistake as hardcoding `/home/ubuntu/...`
asset paths (see `CLAUDE.md`, "Test fixture paths"): an index into a
generated corpus is environment state, not a fixture identity.

**Derive the trial set from whatever corpus is present, and print both
the count found and the set used.** A run that silently skipped 3 of 5
target trials otherwise reads as a real 0/5 result.

### Trap 2 — the selection baseline must not already contain the mechanism

The obvious way to pick trials is to reuse the production-path
discovery (`decode_wav_fst4_60`, i.e. `DecodeRequest`) that found the
disagreement in the first place. **That is now circular.**
`DecodeRequest`'s `osd` flag defaults to `true`, and #308's `i0±1`
retry is gated on exactly that flag — so the production "npre" arm
*already includes timing diversity*. Trials selected that way are, by
construction, ones where timing has already been given its chance and
failed. The timing arm is then guaranteed to rescue nothing, and the
run produces a confident "neither mechanism works" verdict that is an
artifact of the selection, not a measurement.

This bites specifically because the mechanism under test *shipped*
between the original finding and the re-measurement. Whenever that is
true, the pre-fix baseline no longer exists in the production entry
point, and the ablation has to reconstruct it.

Two ways out, in order of preference:

1. **Measure the whole cell across the full configuration grid and
   read the sets off afterwards.** No selection step, so nothing to
   bias. This is what `fst4_60_diag_npre_timing_substitutability_ablation`
   does — 2 timing configurations × 3 OSD configurations over every
   present trial.
2. If the grid is too expensive, select using a baseline you have
   explicitly reconstructed (npre at a single `i0`), never the current
   production path.

### Why a hand-built pipeline rather than `DecodeRequest`

The production entry point **cannot express "npre with OSD but without
timing diversity"** — the two are welded together by the `depth.osd`
gate. So the ablation calls the stages directly:

```
coarse_sync → fst4_sync_search → freq_shift_cd0
            → symbol_spectra / sync_quality → compute_llr → decode_soft
```

Sync runs **once per trial**, cached, and every configuration is
evaluated against that same prepared candidate set — so any difference
between configurations is attributable to the OSD/timing axes alone
rather than to sync jitter.

Two details that are easy to get wrong here:

- `cd0` must be frequency-shifted by `refined − coarse` before
  `symbol_spectra` (`freq_shift_cd0`). Skipping it zeroes the
  correction and artificially fails the `nsync` gate for exactly the
  marginal candidates an ablation is about. This has already caused one
  implausibly-good measurement in this line of work.
- Match success against the known golden message, not "something
  decoded" — otherwise a CRC false-accept counts as a rescue.

**The hand-built path is narrower than production** (candidates filtered
to ±5 Hz of the golden, single sync pass, no SIC), so it can decode
fewer trials than `DecodeRequest` on the same corpus. Always
cross-check against `fst4_60_diag_npre_osd_bug_hunt_ccir_moderate` on
the same files before attributing a difference to the mechanisms.

### Running it

```sh
cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_npre_timing_substitutability_ablation \
    -- --ignored --nocapture --test-threads=1
```

~6 min for a 20-trial cell (6 configurations × 20 trials, plus one
sync pass each). Scoped by test name — never a blanket `-- --ignored`,
which escalates into the full tier-C campaign (see `CLAUDE.md`).

### Trap 3 — the cell must have partial baseline recall

The grid can only separate mechanisms where the `npre @{0}` baseline is
non-zero and non-saturated. On a 0/n cell every "rescue" is measured
against nothing and a mechanism that merely *widens* an existing margin
is invisible; on an n/n cell there are no failures left to rescue. This
is section 6's rule, and it applies here with extra force.

The first run of this ablation used m26 CCIR-moderate — the cell #306
and #308 both name — and it is **0/20 for this harness**.
`fst4_60_diag_npre_baseline_scan` locates the usable band cheaply (one
configuration instead of six). Measured 2026-08-17, 20-trial corpus:

| channel | m20 | m22 | m23 | m24 | m25 | m26 | m27 | m28 | m29 |
|---|---|---|---|---|---|---|---|---|---|
| AWGN | 20/20 | 20/20 | 20/20 | 20/20 | 20/20 | 19/20 | **12/20** | **3/20** | 0/20 |
| CCIR-moderate | 20/20 | 20/20 | 20/20 | **17/20** | **7/20** | 0/20 | 0/20 | 0/20 | 0/20 |

That the original finding lives in a cell this harness cannot use is
itself the quantitative version of "the hand-built path is narrower than
production": production reaches roughly 1–2 dB deeper, carrying #308's
timing retry, a wider candidate set, and no ±`FREQ_TOL_HZ` pre-filter.

### Trap 4 — compare the combination against the union, not the singles against each other

The first classifier compared the two single-mechanism sets to each
other and never compared the *combination* against their union. It
therefore labelled a cell "substitutable" (identical singles) while the
combination was in fact rescuing strictly more than either — the one
outcome that changes the answer for #310. If you write this kind of
2×2 by hand, the synergy set (`both \ (timing ∪ fallback)`) is the
column that matters most; it is also the only one the naive comparison
cannot see.

### Result, 2026-08-17 — the usable band (20-trial corpus, 4 cells)

Relative to the `npre @{0}` baseline in each cell:

| cell | baseline | timing alone | fallback alone | both | only-with-both | verdict |
|---|---:|---|---|---|---|---|
| CCIR-mod m24 | 17/20 | 19 | 15, 19 | 15, 19 | — | partial overlap |
| CCIR-mod m25 | 7/20 | 1 | — | 1, **9, 14** | **9, 14** | **synergistic** |
| AWGN m27 | 12/20 | 15 | 15 | **3**, 15, **20** | **3, 20** | **synergistic** |
| AWGN m28 | 3/20 | 5, 12, 18 | 5 | 5, 12, 18 | — | partial overlap |

**In 2 of 4 cells the combination rescues trials that neither mechanism
rescues alone.** Mechanistically that is coherent: the unpruned search
explores more patterns but only helps if the LLRs it is handed are good
enough, and an alternate `i0` is what produces better LLRs. Both
conditions have to hold at once.

**Which mechanism dominates flips between cells** — at CCIR-moderate m24
the fallback's set contains timing's; at AWGN m28 timing's strictly
contains the fallback's. There is no universal escalation order to
derive from this.

**`npre` never decodes a trial the unpruned search misses** — the
`npre wins unpruned misses` column is empty in all four cells, at both
timing settings, and `npre→unpruned` is byte-identical to plain
`unpruned` throughout. So on this corpus `npre` is a pure recall loss
bought for speed, and a "selective fallback to unpruned" is
behaviourally the same thing as always running unpruned. Worth knowing
before designing an escalation order around the distinction.

**For #310**: the ladder cannot be reduced to one mechanism without
losing the synergy trials — but the magnitude is 2–3 trials per 20 in
the crossing band, against a measured 2.5–3× cost for full timing
diversity on real hardware. That is a trade to make deliberately, not
an argument that both are mandatory.

### Superseded first reading (`fst4_60_ccir_moderate_m26`, 20 trials)

Kept because it is how traps 3 and 4 were found, not as a result to
cite.

| configuration | decoded | trials |
|---|---:|---|
| `npre @{0}` | 0/20 | — |
| `npre @{0,+1,-1}` | 0/20 | — |
| `unpruned @{0}` | 0/20 | — |
| **`unpruned @{0,+1,-1}`** | **3/20** | 1, 4, 17 |
| `npre→unpruned @{0}` | 0/20 | — |
| `npre→unpruned @{0,+1,-1}` | 3/20 | 1, 4, 17 |

**Verdict: interacting, not substitutable and not independently
additive.** Neither mechanism rescues anything on its own — timing
diversity does nothing for `npre`, and the unpruned search does nothing
at a single `i0`. Only the combination decodes, and the fallback
configuration's hits are *identical* to plain `unpruned + timing`,
i.e. `npre` contributes nothing on those three trials.

For #310 this means the ladder cannot simply pick the cheaper of the
two: on this cell the rescue is the combination, so an embedded build
that keeps `offsets = &[0]` gets no benefit from adding an unpruned-OSD
fallback either.

Every trial in *this* cell reached `osd_depth = 3`, so `npre2` was
exercised throughout it. That does **not** generalise: in the usable
band above, some trials decode on BP alone (`depth = 0`, e.g. m24 trials
7 and 17) and some stop at `depth = 2` (m28 trials 4 and 10). Whether
`npre2` is even in play is cell-dependent, so the npre1-vs-npre2
localisation question #311 raised is answered only for the deepest
cells, not in general.

### Caveats, stated because they are load-bearing

- **20 trials per cell, single-digit rescue counts.** Directional.
  Nothing here should move a production default; that needs the
  near-threshold sweep, and the cost side needs real hardware.
- **The verdict is cell-dependent, so cite the cell.** Two of four
  cells are synergistic and two are not; a single-cell run of this
  ablation can be made to say either thing. Report the band.
- **No cell reproduces #308's specific 2/5 timing win.** #308 measured
  that on a 100-trial generation, naming trials 33/45/67 that this
  corpus does not contain (trap 1). Unresolved, and **not testable
  here**: `fst4sim` is not installed, so a 100-trial generation cannot
  be built in this environment. Do not read these tables as
  contradicting #308 until the grid has run on that generation.
- One sub-mode only (FST4-60), one target message, sniper-style
  candidate pre-filtering to ±`FREQ_TOL_HZ` of the golden frequency.
- **Wall-clock is deliberately not reported.** This is recall
  attribution. Cost numbers belong on the Ryzen 9 box or real hardware
  — an Apple laptop's AC/battery state swings host wall-clock ~3×
  (`BENCHMARKS.md` records the machine for exactly this reason).

## 10. Sniper candidate cap — retained fraction, not absolute count (2026-08-17)

Issue #312, from VK3NV's follow-up on #306. `fst4_60_diag_sniper_gate_width_sweep`
sweeps the sniper path's search width **and** its `max_cand` cap over
the real FST4-60 golden (`210115_0058.wav`, two known signals 230 Hz
apart: N5TM @ 1101 Hz, K9KFR @ 1331 Hz).

```sh
cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_sniper_gate_width_sweep \
    -- --ignored --nocapture
```

### Why the `frac` column is the point

An absolute cap is **not comparable across widths**. Measured:

| width | raw population | `max_cand = 50` is… |
|---|---:|---|
| ±250 Hz | 842 | top **6 %** |
| ±100 Hz | 332 | top **15 %** |
| ±50 Hz | 165 | top **30 %** |
| ±25 Hz | 85 | top **59 %** |

One constant, a 10× spread in retention policy. This is also why the
false-survivor count *rises* as the window narrows at fixed cap
(N5TM: 21 → 29 from ±250 to ±25 Hz) — the narrow window keeps a much
larger fraction of its own smaller pool. Reading that as "narrowing
makes things worse" is the trap the column exists to prevent.

### At matched retained fraction, narrowing the window does cut false survivors

Comparing cells at ~5 % retention rather than at equal cap:

| width | cap | frac | false survivors (N5TM / K9KFR) |
|---|---:|---:|---|
| ±250 Hz | 50 | 6 % | 21 / 18 |
| ±100 Hz | 16 | 5 % | 9 / 10 |
| ±50 Hz | 8 | 5 % | 5 / 6 |
| ±25 Hz | 4 | 5 % | 1 / 3 |

Roughly **20× fewer** false survivors at the same retention policy, and
monotone in width for both targets. Same shape at ~10 % retention. So
width and cap are not interchangeable knobs: the width genuinely shrinks
the noise pool, and the fraction is the right axis on which to compare
caps. A cap derived from the width is defensible on this evidence; a
fixed constant across widths is not.

### What this sweep cannot answer, and why

**Both golden signals are strong, and the target still decodes at every
width down to `max_cand = 4`.** No recall cliff is reached anywhere in
the table, so this cannot locate where truncation starts costing weak
decodes — which is the entire question a tighter cap raises. VK3NV
flagged this scope limit when requesting the sweep; it holds. A
near-threshold AWGN/CCIR sweep has to run before any production default
moves.

Also note `decoded` counts decode *successes*, including several
candidates decoding the same signal, so it is not a recall figure. At
±100/±50/±25 Hz only one known signal is in the window, so `real ≤ 1`
by construction there; only the ±250 Hz rows contain both.

### Audit: dedup-suppressed candidates re-admitted past the cap

VK3NV's #312 follow-up flagged a reachable path in `engine/sync.rs`:
the dedup loop marks the losing near-duplicate (4 Hz / 40 ms) with
`score = 0.0`, but the `retain` right after admits anything satisfying
`stage1_pass(fi)` regardless of score. `stage1_norm` is only populated
for FST4, so it is FST4-specific.

`fst4_60_diag_dedup_zero_score_readmission` counts it directly — a
`score` of exactly `0.0` in the output can only come from that dedup
assignment, and `SyncCandidate::score` is public, so no library change
was needed.

**It fires, but not where the concern expected.** Measured over
4 widths x 6 caps on both golden targets and two near-threshold sweep
trials:

| source | width | cap | capped | zeros | zeros in reserved near-hint group |
|---|---|---:|---:|---:|---:|
| golden / K9KFR | +/-250 Hz | 50 | 50 | 3 | 3 |
| golden / K9KFR | +/-100 Hz | 50 | 50 | 3 | 3 |
| golden / K9KFR | +/-50 Hz | 50 | 50 | 3 | 3 |
| golden / K9KFR | +/-25 Hz | 50 | 50 | 6 | 3 |

Every other row is zero — including **every cap below 50**, both
near-threshold sweep trials, and N5TM entirely.

The reason is that zeros sort last within their group, so a low cap
truncates them away before they matter: `reserved =
min(near.len(), cap.div_ceil(2))` takes the top of `near` *by score*,
and `near` always held at least `reserved` non-zero candidates here.

**So the caveat lands on the opposite rows from the hypothesis.** The
low-cap rows in the tables above are unaffected, as are section 11.2's
near-threshold recall numbers. What is affected is `max_cand = 50` —
the production default — where on one of the two golden signals **3 of
50 slots (6 %) go to candidates already known to be duplicates**, all
three inside the reserved near-hint group.

**Follow-up: do the re-admitted duplicates ever decode?**
`fst4_60_diag_dedup_zero_score_recall_effect` answers it directly, over
the partial-recall band at the production `max_cand = 50`, sniper-shaped
(`freq_hint` set). Two columns: recall from the candidate list as
`coarse_sync` returns it, and recall with `score == 0.0` entries dropped
before any decode work — i.e. exactly what tightening the gate would do.

| cell | n | zeros reaching the list | zeros that decoded | recall as-is | recall filtered |
|---|---:|---:|---:|---:|---:|
| CCIR-mod m24 | 20 | 1 | **0** | 18/20 | 18/20 |
| CCIR-mod m25 | 20 | 1 | **0** | 9/20 | 9/20 |
| AWGN m27 | 20 | 4 | **0** | 15/20 | 15/20 |
| AWGN m28 | 20 | 3 | **0** | 6/20 | 6/20 |

**No re-admitted duplicate decoded anything, and filtering them changes
no cell's recall.** On this path the OR-gate re-admission buys nothing,
so tightening `retain` to `score >= sync_min && stage1_pass(fi)` would
cost no recall and would return the slots the duplicates consume.

Sample is small and the scope is narrow — 9 zero-score candidates across
80 trials, FST4-60 only, one width (+/-250 Hz), one cap, sniper-shaped
only. It does not cover the wide-band `DecodeRequest` shape (no
`freq_hint`, where a zero sorts to the bottom rather than into a
reserved slot), nor the strong-signal golden case where the 3 zeros in
the table above appeared. "Never decodes" is supported here, not
established in general.

Not fixed here: this is a measurement of the existing behaviour, and
whether the `retain` should test `score >= sync_min && stage1_pass(fi)`
(or exclude zeroed entries explicitly) is a decision for #312 with its
own recall check, since the OR-gate exists for #146 reasons.

### One pathological candidate costs ~2 s, and survives at cap = 4

An anomaly worth keeping: N5TM's ±100 Hz and ±50 Hz rows sit at
~2 100 ms for *every* cap including 4, while the same target's ±250 Hz
and ±25 Hz rows at cap 4 are 40–90 ms. K9KFR shows no such plateau.

The reading is that a single candidate in the ±100/±50 Hz window costs
~2 s on its own, ranks in the top 4 there, is crowded out of ±250 Hz's
top 4 by stronger candidates, and falls outside ±25 Hz entirely. That is
a concrete instance of the failure mode #310 exists to bound: **one
pathological false survivor consuming the expensive depth budget before
the real candidates have had their cheapest attempt.** Candidate-count
reduction alone does not fix it — the cost is per-candidate, not
per-population.

This is a within-run comparison (25× on the same invocation), so it is
not the host power-state confound that makes absolute host wall-clock
unreliable here; but the absolute milliseconds still belong on the
Ryzen 9 box before they go in a table anyone reasons from.

## 11. Ryzen 9 runbook — what needs the big machine, and why

Three jobs cannot be finished on a laptop. Two are blocked on wall-clock,
one on absolute-timing reliability (an Apple laptop's AC/battery state
swings host wall-clock ~3×, so every absolute host number in this
document that matters is supposed to come from the Ryzen 9 box —
`BENCHMARKS.md` records the machine for the same reason).

Run each with the corpora present under `embedded-poc/assets/*_sweep/`.

### 11.1 Tier-C sensitivity sweeps — the `v0.10.0` release gate

**Required before tagging** (`CLAUDE.md`, "Releases"). Completed
2026-08-19/20, all seven protocols, on the Ryzen 9 9900X (24 threads)
— folded into a larger session that also rebuilt
`scripts/run-sensitivity-sweeps.sh` into a self-diffing regression
check (`scripts/sweep-regression-check.py` +
`docs/notes/sweep-baseline.json`, see that script's own doc comment
for the design) rather than a table a human re-reads by eye each time.

```sh
scripts/run-sensitivity-sweeps.sh
```

WSPR/FT8/JT65/JT9/Q65/MSK144 needed no re-measurement — a source diff
against the commit those numbers were first taken from confirmed zero
functional change in any of their decode paths since (`wspr::ddc` and
`engine::dsp::{analytic,msk}` moved only by the `SAMPLE_RATE_HZ`
rename, #321 — value-identical). FST4 alone got a full re-run because
`fst4::rung_major`, `engine::sync`, `engine::llr` genuinely changed
this cycle (`Schedule::PhaseSplit` #317, the coarse_sync dedup fix
#316, the npre1/npre2 OSD port). Result: **every one of FST4's twenty
(mode × channel) 50%-crossings landed byte-identical to the
pre-change measurement** — consistent with both #317 and #316 being
built and verified specifically to be recall-neutral (see their
ROADMAP.md entries), now confirmed end-to-end through the real
sweep rather than only through their own targeted equivalence tests.
FST4-15's AWGN crossing (−20.70 dB) and the other four sub-modes all
land within the pre-existing 0.1-0.6 dB gap band against WSJT-X's
published figures (cross-checked against the table in section 5
above). No `BENCHMARKS.md` update needed — nothing moved.

The full FST4 group (all 5 sub-modes × 4 channels, 6500 WAVs) took
**1852-1922 s (~31 min)** end to end on this machine, both before and
after the code change — nowhere near the "8 h+ on an 8C/16T box"
back-of-envelope extrapolation this section used to cite. That
estimate was never a measurement; the real number, run twice now, is
consistent with itself.

Going forward, `run-sensitivity-sweeps.sh` narrows the SNR range
around each group's `sweep-baseline.json` crossing automatically
(`scripts/sweep-narrow-plan.py`, `MFSK_SWEEP_FULL=1` to opt out) — a
group with no baseline entry yet (new sub-mode, first run) always
falls back to the full grid the way this run did, so future FST4
sweeps should be substantially faster than the ~31 min measured here
unless the crossing itself moves.

### 11.2 Near-threshold sniper cap sweep — the open half of #312

Section 10 answered the comparability question on strong signals but
could not find a recall cliff there. This finds it:

```sh
cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_sniper_cap_near_threshold \
    -- --ignored --nocapture
```

Default grid is 4 cells × 4 widths × 6 caps. On a 20-trial corpus that
is 1 920 candidate loops — **~2 h on an M5 laptop**, measured. Narrow it
with `MFSK_CAP_SWEEP_CELLS` (`channel:snr`, comma-separated),
`MFSK_CAP_SWEEP_WIDTHS`, `MFSK_CAP_SWEEP_CAPS` rather than editing the
test.

**Preliminary, 4 of the 96 configurations** (smoke test, ccir_moderate
m25, 20 trials — enough to prove the sweep works, not enough to
conclude):

| width | cap | raw | frac | recall |
|---|---:|---:|---:|---:|
| ±250 Hz | 4 | 17 049 | 0 % | **6/20** |
| ±250 Hz | 50 | 17 049 | 6 % | **9/20** |
| ±25 Hz | 4 | 1 755 | 5 % | **6/20** |
| ±25 Hz | 50 | 1 755 | 57 % | **9/20** |

**A cliff exists** — truncating to 4 costs 3 of 9 decodes. And note what
it is *not* tracking: recall is 6/20 at cap 4 and 9/20 at cap 50 at
**both** widths, while the retained fraction at those points differs by
an order of magnitude (0 % vs 5 %, 6 % vs 57 %).

If that survives the full grid it is a direct tension with section 10's
conclusion. Section 10 says false-survivor *cost* tracks the retained
fraction, which argues for deriving the cap from the width. This says
weak-signal *recall* tracks the absolute count, which argues for a
floor on how many ranked candidates must be kept regardless of width.
Both can be true at once — they are different columns — and if they are,
the policy is "absolute floor for recall, fraction for cost", not either
alone. That is the thing worth settling on the big machine, and it is
the question VK3NV actually asked (#312: *does the cliff sit at a
similar fraction across widths?* — preliminary answer: no).

### 11.3 Resolving the #308 timing-win tension — needs `fst4sim`

Section 9's open caveat: no cell here reproduces #308's 2/5 timing
recovery, because #308 measured it on a **100-trial-per-cell**
generation naming trials 33/45/67, and this environment's corpus has 20
per cell. Not testable on the laptop — `fst4sim` is not installed there.

On a box that has WSJT-X built:

```sh
# 100 trials per cell, at least the crossing band
TRIALS=100 scripts/gen_fst4_sweep_wavs.sh

cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_npre_baseline_scan \
    -- --ignored --nocapture          # re-locate the band: it moves with n

cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_npre_timing_substitutability_ablation \
    -- --ignored --nocapture --test-threads=1
```

Re-run the baseline scan **first**: the partial-recall band is a
property of the corpus generation, and pointing the ablation at a cell
from a different generation is trap 3 all over again. Update the cell
list in the ablation's driver to whatever the scan reports.

What this settles: whether the synergy result (2 of 4 cells) holds at
n=100, and whether #308's own 2/5 reproduces once the trials it named
actually exist.

### 11.4 Cheap `i0` ranking vs exhaustive retry

Section 12's test, full 8-run grid (4 cells × 2 OSD searches):

```sh
cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_i0_cheap_rank_vs_exhaustive \
    -- --ignored --nocapture --test-threads=1
```

`--test-threads=1` is required (`fst4_osd_diag_force_old` is a
process-global toggle). Costs are counted in decode units, so unlike the
sweeps above this one's numbers are machine-independent — the reason to
run it on the Ryzen 9 is wall-clock to completion, not measurement
quality.

Read the `argmax nsync == offset 0` line **before** the verdict; see
section 12 for why.

### 11.5 What to bring back

- Tier-C tables, diffed against `v0.9.1`, for the release decision.
- The full `fst4_60_diag_sniper_cap_near_threshold` grid — specifically
  whether the cliff sits at a fixed count or a fixed fraction.
- The ablation re-run at n=100, with the band the scan reported.

`loop_ms` columns from the Ryzen 9 are worth recording in
`BENCHMARKS.md`; the laptop's are not, and are marked as relative-shape
only wherever they appear above.

## 12. Cheap `i0` ranking vs exhaustive retry — and a degenerate metric (2026-08-18)

VK3NV's #308 proposal, measured: #308 ported WSJT-X's control flow
literally (try `i0`, `i0+1`, `i0-1` at full depth, short-circuit on
success), which is 3.02× / 2.51× of the embedded candidate loop and is
why `decode_rung_major` defaults to `offsets = &[0]`. The proposal was
that cheap sync information already exists before the expensive work, so
the offsets could be **ranked** and only the best one decoded:

> Could we score `i0`, `i0+1` and `i0-1` cheaply, choose/rank the best
> timing, and run the expensive decoder only once initially?

```sh
cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_i0_cheap_rank_vs_exhaustive \
    -- --ignored --nocapture --test-threads=1
```

### Cost is counted, not timed

The metric is **full decode units** (`compute_llr` + the BP/OSD ladder
for one candidate/offset pair), not wall-clock. That makes these numbers
machine-independent and directly quotable — unlike every `loop_ms`
column elsewhere in this document, which is relative-shape-only on a
laptop.

### The confound to check — and a correction about how to check it

`i0` does not arrive unranked. It comes from `fst4_sync_search`, whose
job is to **maximise a coherent Costas amplitude** over a (Δf, Δt) grid.
`sync_quality` measures related information, so it plausibly peaks at
`i0` too — which would make `argmax nsync` degenerate to offset 0 and
`cheap-rank ≈ base` a near-tautology rather than evidence about the
proposal.

That is a real hazard and the test now **counts** `argmax nsync == 0`
explicitly, short-circuiting the verdict to `DEGENERATE RANKING` at
≥95 %.

**Correction (2026-08-18).** An earlier revision of this section claimed
the degeneracy was already demonstrated, inferring it from the `units`
column: `base` 51 vs `cheap-rank` 52 at one cell, 38 vs 38 at another.
**That inference is invalid.** Both configurations perform exactly one
`decode_at` per candidate — `base` at offset 0, `cheap-rank` at the
argmax — so their unit counts are equal *by construction*, whichever
offset the ranking picks. The counts only diverge when the two offsets
land on opposite sides of the `nsync` gate. Equal units say nothing
about whether the ranking chose differently.

The evidence in fact points the other way: the `nsync picked the winning
offset first` line reaches 2/3 and 2/2 on AWGN m28, which is impossible
if the ranking always returned offset 0. **So the degeneracy is probably
not what is happening, and the question is open** until the explicit
counter has been run.

A second metric caveat, for whoever reads that counter: agreement is
counted **per candidate**, recall **per trial**. A candidate where the
ranking picks the winning offset does not add a decode if another
candidate in the same trial already decoded at offset 0. The two columns
are not directly comparable, and a high agreement rate alongside
`cheap-rank == base` recall is not a contradiction.

### What holds regardless

Whether or not the ranking is degenerate, one thing follows from where
`i0` comes from: **a cheap proxy for "which timing will decode" is
unlikely to be another sync-strength measure**, since the refine stage
has already maximised that. If a proxy is to work, the more promising
direction is sync-orthogonal — LLR reliability statistics, or a
truncated-BP syndrome weight, rather than Costas correlation.

### Results, 20-trial corpus, all 8 runs

Recall / decode units. Measured before the `argmax` counter existed, so
the agreement column is the only proxy-quality signal available in this
table:

| cell / OSD | base | exhaustive | cheap-rank | progressive | nsync picked winner |
|---|---|---|---|---|---|
| ccir-mod m24 / npre | 17, 51u | **18**, 83u | 17, 52u | 18, 86u | 0/6 |
| ccir-mod m24 / unpruned | 19, 51u | 19, 75u | 19, 52u | 19, 77u | 0/5 |
| ccir-mod m25 / npre | 7, 38u | **8**, 96u | 7, 38u | 8, 96u | 0/1 |
| ccir-mod m25 / unpruned | — | — | 8, 38u | **10**, 89u | 1/5 |
| AWGN m27 / npre | 12, 26u | **13**, 46u | **13, 26u** | 13, 44u | 1/1 |
| AWGN m27 / unpruned | 13, 26u | **15**, 42u | 13, 26u | 15, 39u | 1/3 |
| AWGN m28 / npre | 3, 23u | **6**, 61u | 3, 23u | 6, 60u | 2/3 |
| AWGN m28 / unpruned | 4, 23u | **6**, 60u | 4, 23u | 6, 59u | 2/2 |

**`cheap-rank` matched `exhaustive` in 1 of 8 runs** (AWGN m27 / npre:
13/20 at 26 units, i.e. exhaustive's recall at base's cost) and matched
`base` in the other seven. One hit out of eight is not a working proxy,
but it is also not the flat null the first three runs suggested.

Two results hold independently of the open question above:

- **Exhaustive retry costs 1.6–2.7× base in decode units** for +1 to +3
  decodes. Measured in algorithm-invocation counts, so it transfers
  across machines, and it is the same order as the 2.5–3× wall-clock
  measured on CoreS3.
- **`progressive` never beats `exhaustive` on cost** (86 vs 83, 96 vs
  96, 44 vs 46, 39 vs 42, 60 vs 61, 59 vs 60) — at best a few percent
  either way, well inside noise for a reordering that cannot change the
  set. It does confirm the harness: recall matches `exhaustive` in every
  run, as it must.

The AWGN cells rescue more (+3 at m28) than the CCIR-moderate ones (+1),
which is the opposite of the fading-motivated framing #308 came from and
worth re-checking at n=100.

## 13. Scheduling decision — why `offsets` stays offset-major (2026-08-18)

Issue #310 proposed folding `decode_rung_major_timed`'s `offsets`
parameter into a cost-ordered priority queue, so that "cheap attempts
across all candidates first → timing alternatives → deeper correlation →
OSD/fallbacks as budget permits" rather than running the whole 5-rung
ladder at `offset = 0` before touching `offset = -1`'s cheapest rung.

**Current position: don't fold it in. Keep the offset-major structure
and add a budget gate between offsets instead.** Recorded here with the
reasoning, because it is contrary to the issue's original framing and
should not have to be re-derived from the measurement sections above.

### The three measurements this rests on

1. **Timing diversity is expensive and thin** (§12). Exhaustive
   `{0,+1,-1}` retry costs **1.6–2.7× the decode units** of `{0}` for
   **+1 to +3 decodes per 20 trials** — same order as the 2.5–3×
   wall-clock measured on CoreS3, and in invocation counts so it
   transfers across machines.

2. **It cannot be cheaply targeted** (§12). Ranking the offsets by
   `sync_quality` and decoding only the best matched exhaustive in
   **1 of 8 runs** and the single-offset baseline in the other seven.
   There is a structural reason to expect sync-derived ranking to
   struggle — `i0` already comes from `fst4_sync_search`, which
   maximises a coherent sync score — so a working proxy would have to be
   sync-orthogonal.

3. **The mechanisms interact** (§9). In 2 of 4 cells, alternate timing
   and the unpruned-OSD fallback together rescue trials that neither
   rescues alone.

### Why not fold, in order of weight

1. **It would triple the first-rung sweep.** Rung-major's actual
   contribution is the ordering-independent bound on time-to-first-decode
   (~7.4 s projected for the first rung across 41 candidates). Mixing
   offsets into the cheapest rung trades away the property the design
   exists to provide.

2. **Offset setup is not free.** A new offset rebuilds `symbol_spectra`
   and the bit metrics. Interleaving individual rungs across offsets pays
   that setup for every candidate, early, for a mechanism that pays off
   on 1–3 trials in 20. The natural scheduling unit is
   *offset setup + a bundle of rungs at that offset*, not a free
   interleave — which is what offset-major already is.

3. **The payoff cannot be aimed.** If cheap ranking worked, front-loading
   one well-chosen alternate offset would be attractive. Measurement 2
   says it doesn't, so early offset work is spent broadly rather than
   where it will land.

### What to build instead

Keep offset-major; add a **deadline check between offsets**. Run
`offset = 0`'s full ladder to completion, then spend whatever slot time
remains on additional offsets. This preserves the first-rung bound,
keeps the expensive mechanisms off the critical path, and degrades to
today's `&[0]` behaviour when the slot is tight.

**If a second pass is added, add timing and the unpruned fallback
together as one escalation step** — measurement 3 says adding only the
cheaper of the two pays real cost and collects part of the gain. This is
the one place the measurements changed the design rather than confirming
it.

Related: `npre` never decodes a trial the unpruned search misses on any
cell tested (§9), so it buys speed and never recall. There is no reason
to interleave npre *depth* with timing — only to run npre first because
it is cheap.

### What would reverse this

- A **sync-orthogonal cheap proxy** that actually predicts the winning
  offset (LLR reliability statistics, truncated-BP syndrome weight).
  That would make front-loading one targeted alternate offset cheap, and
  the cost-ordered fold becomes attractive.
- The n=100 run (§11.3) showing the synergy trials are a materially
  larger fraction than 2–3 per 20, raising the second pass's value enough
  to justify paying for it earlier.
- Per-offset setup cost measured directly — the instrumentation already
  exists in `decode_rung_major_timed`'s `clock` hook — showing setup is
  small relative to a rung, which would weaken reason 2.

All three are open. This decision is provisional on them.

## 14. Soft Costas margin vs raw `nsync` (2026-08-18)

VK3NV's #310 proposal, implemented and measured.
`engine::llr::sync_quality_generic` inspects all four tone energies per
known sync symbol and then discards everything except
`best == expected` — so `E_exp = 9.8, E_wrong = 9.5` and
`E_exp = 9.8, E_wrong = 1.2` contribute the same `+1`.

`sync_quality_soft_generic` (new, diagnostic-only, nothing in the decode
path calls it) returns the same `nsync` **and** fills a caller-provided
`[(E_expected, E_best_wrong); n_sync]`. No new spectral work —
`symbol_spectra` has already produced these values — and the extra cost
over the hard version is one comparison per tone, since the loop tracks
the max over all tones and the max over tones *other than* the expected
one in the same pass. The buffer is caller-provided rather than
allocated so it stays usable on embedded, where a per-candidate `Vec` in
the candidate loop is a shape that has caused trouble before.

```sh
cargo test -p mfsk-core --features full,internal-testing --release \
    --test fst4_sweep fst4_60_diag_soft_costas_margin_separation \
    -- --ignored --nocapture
```

### Metric

**AUC** — the fraction of (decoding, non-decoding) candidate pairs the
score orders correctly, ties counting half; 0.5 is chance. The right
shape here because #310's intended use is *ranking* candidates for
escalation budget rather than thresholding them, and because it is
insensitive to the heavy class imbalance these cells have (~15–46
decoders against ~700 non-decoders).

Population: every candidate clearing the `nsync` gate, on the
partial-recall band, `max_cand = 50`, sniper-shaped. "Decodes" means the
golden message specifically.

### Result — the proposal passes its own kill condition

| cell | candidates (dec / not) | `nsync` | mean margin | min margin | **mean logratio** |
|---|---|---:|---:|---:|---:|
| CCIR-mod m24 | 724 (46 / 678) | 0.767 | 0.784 | 0.697 | **0.786** |
| CCIR-mod m25 | 736 (18 / 718) | 0.701 | 0.779 | 0.718 | **0.800** |
| AWGN m27 | 742 (35 / 707) | 0.722 | 0.785 | 0.685 | **0.783** |
| AWGN m28 | 754 (15 / 739) | 0.701 | 0.706 | 0.593 | **0.727** |

- `mean logratio` (mean of `ln(E_exp / E_wrong)`, clamped) beats raw
  `nsync` in **all four cells**: +0.019, +0.099, +0.061, +0.026.
- `mean margin` (`(E_exp − E_wrong) / (E_exp + E_wrong)`) beats it
  clearly in three and ties in the fourth.
- `min margin` is **worse than `nsync` everywhere** — one bad symbol is
  not what distinguishes a decodable candidate.

The gains are largest where recall is lowest (m25: 0.701 → 0.800), which
is the regime the escalation budget is for. So collapsing the four-tone
observation to 40 hard bits *is* discarding usable information, and the
unbounded log-ratio form uses it better than the bounded one.

It is an improvement, not a transformation: AUC 0.78–0.80 still
mis-orders a fifth of the pairs.

### Both groups have negative margins

`mean(dec)` runs −0.067 to −0.153 and `mean(no)` −0.260 to −0.267, so
the expected tone is *not* the strongest one on average even for
candidates that go on to decode. That is unsurprising at these SNRs, and
it means the separation lives in how negative the margin is rather than
in a sign test — worth knowing before anyone reaches for a threshold.

### The scope limit that matters for #310

This population is **every gate-passing candidate**, including the ones
that decode cheaply at the first rung. #310's actual question is
narrower: among candidates that have *already failed* the cheap rung,
which deserve the expensive BP/OSD/timing budget? That is a subset with
a different class balance, and the ranking may behave differently on it.
The result above is necessary but not sufficient for adopting this as an
escalation-priority signal.

Not wired into anything. `sync_quality` and `sync_quality_generic` are
untouched and bit-identical; the soft variant asserts equality of the
`nsync` it returns against the hard one on every candidate in the test.

### The population #310 actually triages — and the result reverses

§14's population was every gate-passing candidate, which includes the
ones that decode immediately at `llra`. Those never reach the escalation
budget, so they are irrelevant to the scheduling decision — and, being
the easiest candidates in the set, they inflate the apparent benefit of
any score.

`fst4_60_diag_soft_margin_escalation_priority` restricts the population
to exactly what #310 has to triage:

- **Population**: passes the `nsync` gate *and* fails BP on `llra` at
  `offset = 0` — the cheapest rung, the one `decode_rung_major`'s latency
  invariant guarantees every candidate.
- **Positive**: goes on to decode with the full escalation budget
  (`llrb`/`llre`/`llrc`, OSD at the gate-selected depth, and `i0 ± 1`).
- **Negative**: never decodes — every unit spent on it was wasted.

| cell | post-first-rung candidates (decode) | `nsync` | mean margin | mean logratio |
|---|---|---:|---:|---:|
| CCIR-mod m24 | 220 (86) | 0.911 | 0.914 | **0.917** |
| CCIR-mod m25 | 203 (28) | **0.726** | 0.709 | 0.700 |
| AWGN m27 | 212 (66) | 0.879 | **0.905** | 0.887 |
| AWGN m28 | 194 (19) | 0.848 | **0.854** | 0.812 |
| **pooled** | **829 (199)** | **0.868** | **0.881** | 0.871 |

**Two things change relative to §14.**

**1. The population is very triageable, by any of the three scores.**
Pooled AUC 0.87–0.88, far above chance. That is good news for #310
independent of which score wins: after the first rung, the candidates
worth escalating *are* distinguishable from the ones that will never
decode.

**2. The soft margin's advantage largely evaporates.** Pooled, `mean
margin` beats raw `nsync` by **+0.013** and `mean logratio` by **+0.003**
— against §14's +0.019 to +0.099. And it is no longer consistent: `mean
margin` loses to `nsync` in CCIR-moderate m25 (0.709 vs 0.726), and
`mean logratio` loses in two of four cells. The best-performing
formulation also flips — `mean logratio` won everywhere in §14, `mean
margin` wins pooled here.

**By VK3NV's own kill condition this fails.** The condition was: *if raw
`nsync` already separates the expensive false survivors, there is no
reason to add anything else.* On the population that matters, it does —
0.868 pooled — and the soft features add ~1 point of AUC inconsistently,
for 40 extra values per candidate of state.

### The methodological point, which is the more durable result

§14's apparent gain was substantially an artifact of measuring on the
wrong population. Including candidates that decode at the first rung —
which no escalation policy ever sees — inflated the margin's advantage
by roughly 5×.

This is why VK3NV specified the `nsync`-only baseline as part of the
proposal. Without it, the pooled 0.881 for `mean margin` reads as "the
soft feature separates well" and would justify wiring it in; against the
0.868 baseline it reads as "raw `nsync` was already doing this". Same
number, opposite decision.

The implementation stays in the tree (`sync_quality_soft_generic`,
diagnostic-only, nothing calls it) because the negative result is worth
being able to re-derive, and because the per-symbol pairs are the
instrumentation any future proxy attempt would want. But on this
evidence #310's escalation ordering should use `nsync`, which it already
has, and look elsewhere for a better signal.

**Not tested**: whether a *learned* combination of the pairs beats
`nsync` — the proposal's original neural-net framing. Three hand-picked
formulations failing is weak evidence against that, not strong.

### Correction: the soft margin *does* carry information beyond `nsync`

The section above concluded the soft features add nothing. **That was
wrong**, and the way it was wrong is instructive: an unconditional AUC
comparison cannot separate "no information" from "information masked by
correlation with the baseline".

`fst4_60_diag_soft_margin_conditional_value` tests it three ways on a
larger population — 6 cells, 1 263 post-first-rung candidates (386
decode, 877 don't), 9 feature formulations instead of 3.

**1. Stratified AUC — within a fixed `nsync` value.** If a feature were
only a proxy for `nsync`, holding `nsync` constant would leave it with
nothing to say and the AUC would sit at 0.5. It doesn't:

| feature | unconditional AUC | **stratified AUC** |
|---|---:|---:|
| `nsync` (baseline) | 0.920 | — |
| mean margin | 0.932 | **0.717** |
| mean logratio | 0.926 | **0.704** |
| sum logratio | 0.926 | 0.704 |
| mean worst-10 | 0.913 | 0.689 |
| p25 margin | 0.916 | 0.638 |
| median margin | 0.917 | 0.579 |
| min margin | 0.724 | 0.542 |
| count margin>0 | 0.921 | 0.530 |
| margin variance | 0.345 | 0.412 |

Within a fixed `nsync`, the mean normalised margin still orders
decode-from-never-decode at 0.717. **VK3NV was right that collapsing the
four-tone observation to 40 hard bits discards usable information.**

(`margin variance` inverts — below 0.5 both ways — i.e. *lower*
dispersion associates with decoding. Real but not actionable here.)

**2. A fitted linear rule, and what it actually buys.** Logistic
regression over `nsync` + all nine features:

| | AUC |
|---|---:|
| `nsync` alone | 0.920 |
| in-sample fit (optimistic upper bound) | 0.935 |
| **2-fold cross-validated (honest)** | **0.934** |

So the whole feature set, optimally combined, is worth **+0.014 AUC**
over the `nsync` this crate already computes.

### The decision is unchanged, and the reason is now cost, not absence

Both possible outcomes of this experiment led to the same action, which
is worth stating because it is why the elaborate version was still worth
running:

- had the stratified AUCs been ~0.5, there is no signal → use `nsync`;
- they are ~0.7, so there is signal → **it still does not clear the cost
  bar** → use `nsync`.

`nsync` is a `u32` that is already computed on the path. The soft
alternative costs 40 × 2 f32 = 320 B of state per candidate on a target
where per-candidate allocation in the candidate loop has caused real
trouble, plus fitted weights and standardisation constants that would
have to be re-validated across sub-modes, channel conditions and corpus
generations — a tuned constant that rots. **+0.014 AUC does not buy
that.**

Recorded this way deliberately: "measured, and not adopted because the
gain is 1.4 points against a maintained model" is a much more durable
answer than "tried three formulas and they lost", and it answers the
same proposal if it comes back.

**What would change it**: a use where the ranking quality translates
into a large wall-clock saving rather than a small ordering improvement
— e.g. if the escalation budget were tight enough that the top decile
of the ranking is all that ever gets served. That is measurable from
`decode_rung_major_timed`'s existing `clock` hook and has not been done.

The non-linear/learned version the proposal originally framed is **not**
ruled out by this, but the cost argument applies to it more strongly,
not less.

### The budget curve — what ordering is actually worth

AUC is an ordering metric; #310's currency is **decodes before the slot
deadline**. Those only connect through the budget: if it covers every
candidate, ordering changes nothing; if it covers only the head of the
ranking, a small ordering gain can matter a lot. So the AUC results
above could not settle the question on their own.

`fst4_60_diag_escalation_budget_curve` measures the missing quantity —
**per-candidate escalation cost** — and simulates a deadline. Population
is the post-first-rung set (the `llra` rung at `offset = 0` is sunk: the
latency invariant spends it on everyone, so it is outside the budget).
Cost is counted in units of one (offset, stage) evaluation over
`llrb`/`llre`/`llrc` + OSD at `i0`, `i0+1`, `i0-1`, stopping at success.
6 cells, 1 263 candidates, 386 of which decode, 12 330 units total
(mean 9.8 per candidate).

| budget | arrival order | `nsync` | mean margin | **oracle** |
|---:|---:|---:|---:|---:|
| 10 % | 49 | 166 | 172 | **358** |
| 20 % | 111 | 288 | 306 | 386 |
| 30 % | 165 | 361 | 365 | 386 |
| 40 % | 187 | 377 | 382 | 386 |
| 50 % | 198 | 380 | **386** | 386 |
| 70 % | 316 | 384 | 386 | 386 |
| 100 % | 386 | 386 | 386 | 386 |

`oracle` = decoders first, cheapest decoder first. Unachievable — it
knows the answers — but it bounds what *any* ranking could buy.

**Three things fall out.**

**1. Having a priority signal at all is worth a great deal.** At a 10 %
budget, `nsync` returns 166 decodes against arrival order's 49 — 3.4×.
Whatever else is true, #310's escalation-ordering idea is sound.

**2. The soft margin's advantage stays small, but is not zero.** +6
decodes at 10 %, **+18 at 20 %** (306 vs 288, 6.3 % relative), +4 at
30 %, and it reaches the oracle's ceiling at 50 % where `nsync` is still
6 short. Consistent with the +0.014 AUC, and it does not change the cost
verdict: 18 decodes in 386 at one budget point is not worth 320 B per
candidate plus a maintained model.

**3. The interesting number is the oracle gap, and it is large.** At a
10 % budget the oracle recovers 358 of 386 (93 %) where `nsync` gets 166
(43 %). Neither score is close. That headroom exists because the oracle
sorts by cost as well as by decodability — **a cheap decoder is worth far
more per unit than an expensive one, and neither `nsync` nor the margin
knows anything about cost.**

So the useful redirection for #310 is: **the missing signal is not "will
this decode" but "will this decode cheaply".** Both scores here predict
the former. Cost is partly structural and may be cheaper to predict than
decodability — `nsync` already determines whether OSD runs at all
(`osd_attempt_min`), and OSD is the expensive stage.

### The qualifier that keeps this in proportion

FST4-60's real situation is roughly a 7 s margin against a 13.6 s
`no8_osd` candidate loop — call it a ~50 % budget. **At 50 %, `nsync`
already returns 380 of 386 (98.4 %).** The dramatic ordering effects live
at 10–20 % budgets, which would mean a deadline several times tighter
than the one this mode actually has.

So: ordering is worth having (point 1), `nsync` is enough of it at the
budget that exists, and the oracle headroom (point 3) only becomes worth
chasing if the budget gets much tighter — a shorter sub-mode, a slower
target, or a wideband candidate list far larger than the sniper-shaped
50 used here.

### Breadth-first vs depth-first — the schedule matters more than the ordering

The budget curve above used a **depth-first** escalation (each candidate
runs its ladder to success before the next starts).
`decode_rung_major` is **breadth-first** (one rung swept across every
candidate before any descends), so those ordering conclusions could not
be carried over to the design without checking.
`fst4_60_diag_budget_curve_breadth_vs_depth` runs both schedules over
the same candidates and the same per-stage outcomes.

Ladder is offset-major per §13: `llrb`/`llre`/`llrc`/OSD at `i0`
(the `llra` rung being sunk), then the full five stages at `i0+1`, then
at `i0-1`. 1 263 candidates, 386 decode, 12 330 units, longest ladder 14
stages.

| budget | depth: arrival | depth: `nsync` | depth: oracle | breadth: arrival | breadth: `nsync` | breadth: oracle |
|---:|---:|---:|---:|---:|---:|---:|
| 10 % | 49 | **166** | 358 | 35 | 35 | 35 |
| 20 % | 111 | **288** | 386 | 80 | 80 | 80 |
| 30 % | 165 | **361** | 386 | 137 | 150 | 152 |
| 40 % | 187 | 377 | 386 | 350 | 350 | 350 |
| 50 % | 198 | **380** | 386 | 350 | 350 | 350 |
| 70 % | 316 | 384 | 386 | 360 | 366 | 366 |
| 100 % | 386 | 386 | 386 | 386 | 386 | 386 |

**1. Ordering does essentially nothing under breadth-first.** All three
orderings are identical at 10 % and 20 %, and within a couple of decodes
everywhere else. The reason is mechanical: a 10 % budget buys ~1 233 of
the 1 263 stage-0 evaluations, so nearly every candidate gets the first
escalation stage regardless of the order they are visited in. **The
"re-sort survivors by `nsync`" idea is dead for rung-major** — it buys
nothing, and one less thing needs justifying.

**2. Breadth-first is markedly worse under a tight budget.** At 10 %,
depth-first with `nsync` returns 166 decodes against breadth-first's 35;
at 20 %, 288 against 80; at the realistic ~50 %, **380 against 350**.

**3. The mechanism is the OSD stage.** Breadth-first's jump from 137 to
350 between the 30 % and 40 % budget points is a stage boundary:
sweeping stages 0-3 (the offset-0 ladder, OSD included) across ~1 263
candidates costs ~5 050 units, just under the 4 932 units that 40 %
buys. Below that, breadth-first has spent its entire budget on cheap BP
stages and **has not reached OSD for anybody** — and OSD is where most
of these decodes come from. Deferring the highest-yield stage until
every candidate has had every cheaper one is exactly what breadth-first
is for, and under a deadline it is exactly what costs decodes.

### The trade-off this quantifies, and the resulting design

This is not an argument against rung-major. Rung-major exists for the
**ordering-independent bound on time-to-first-decode** — depth-first has
no such guarantee, and a bad candidate order pushes the first decode
arbitrarily late (~30 s of a 30.15 s total in the worst case measured
earlier). The two properties are in genuine tension and the numbers
above are the price:

| | time-to-first-decode | decodes at ~50 % budget |
|---|---|---|
| breadth-first (rung-major) | **bounded by one rung** | 350 / 386 |
| depth-first + `nsync` | unbounded (ordering-dependent) | **380 / 386** |

**Both are available at once**, and the phase split already recorded in
§13 is where:

- **Phase A — first rung, breadth-first, unconditional.** `llra` at
  `offset 0` across every candidate. This *is* the latency invariant:
  every candidate gets its cheapest attempt within a bounded time
  regardless of ordering. Keep exactly as-is.
- **Phase B — escalation, depth-first, ordered by `nsync`, budget-gated.**
  Once the bound has been honoured, breadth-first has no remaining
  advantage and costs 30 decodes at the realistic budget. `nsync` is free
  here — Phase A computed it for every candidate to run the gate.
- **Phase C — second pass (alternate timing *and* unpruned OSD together,
  per §9), only if budget remains.**

So the answer to "should the escalation phase re-sort by `nsync`" is
**yes, but only because Phase B should be depth-first** — under
breadth-first the sort is worthless. The two decisions are coupled and
neither is right alone.

Unmeasured: this models the escalation as sequential units and ignores
the per-offset setup cost (`symbol_spectra` + bit-metrics rebuild) that
§13's reason 2 turns on. Phase C's cost is therefore understated here.
