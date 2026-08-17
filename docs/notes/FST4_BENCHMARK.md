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
