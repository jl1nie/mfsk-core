# FT4 sensitivity benchmark — environment setup

How to reproduce the FT4 AWGN/fading SNR sweep (`tests/ft4_sweep.rs`) from
a clean checkout on any machine. This mirrors the FST4 methodology in
[`FST4_BENCHMARK.md`](FST4_BENCHMARK.md) (built for
[#146](https://github.com/jl1nie/mfsk-core/issues/146)) and feeds the
`DecodeStrictness` recalibration tracked in
[#72](https://github.com/jl1nie/mfsk-core/issues/72) — FT4 currently
hardcodes `DecodeStrictness::Normal` with threshold numbers copy-pasted
from the FT8 calibration, never re-tuned for FT4's own SNR/OSD behaviour.

The whole pipeline is: **WSJT-X Fortran source → `ft4sim` binary → WAV
corpus on disk → `cargo test --ignored` reads the WAVs and prints a
recall table.** Nothing here needs network access at test time — only
the one-time build step touches WSJT-X source.

## 1. Prerequisites

| Requirement | Ubuntu/Debian package | Purpose |
|---|---|---|
| `gfortran` | `gfortran` | compiles `ft4sim.f90` + supporting lib |
| `gcc` / `g++` | `build-essential` | compiles `gran.c`, `sgran.c`, `init_random_seed.c`, `crc14.cpp` |
| `libboost-dev` | `libboost-dev` | `crc14.cpp` uses `boost::augmented_crc` |
| FFTW single-precision | `libfftw3-dev` | linked as `-lfftw3f` |
| WSJT-X source tree | — (see below) | provides `lib/ft4/*.f90` |

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
```

**WSJT-X source.** `scripts/build_ft4sim.sh` only needs the `lib/`
subtree (no Qt build, no `cmake` step) — any WSJT-X checkout with the
`ft4/ft4sim.f90` simulator works, including the upstream tree:

```sh
git clone https://github.com/jl1nie/WSJT-X.git ../WSJT-X
```

Per the repo-root `CLAUDE.md` test-fixture-path rule, never hardcode an
absolute path to this checkout — `build_ft4sim.sh` takes it as an
explicit argument (falling back to the `../WSJT-X` sibling-of-repo-root
convention only as a default).

## 2. Build `ft4sim`

```sh
scripts/build_ft4sim.sh [/path/to/WSJT-X] [out-dir]
# defaults: WSJT-X-dir = ../WSJT-X (sibling of this repo), out-dir = target/ft4sim/
```

Produces `target/ft4sim/ft4sim`. FT4 reuses FT8's `encode174_91`
LDPC(174,91) codec (`genft4.f90` calls it directly), so this build adds
only two FT4-specific compile units (`genft4.f90`, `gen_ft4wave.f90`) on
top of the same shared lib subtree `build_fst4sim.sh` already compiles
(`wavhdr`, `packjt77`, `watterson`, `fftw3mod`/`four2a`). It additionally
needs `crc14.cpp` (Boost-based CRC-14, C++) + `sgran.c` /
`init_random_seed.c` (RNG seeding) — `fst4sim` doesn't need these because
FST4 uses CRC-24 and never calls `sgran`.

Sanity-check the build:

```sh
target/ft4sim/ft4sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav   # one 7.5 s AWGN trial at -15 dB
```

(`ft4sim`'s CLI has no T/R-period argument — unlike `fst4sim`, FT4 has
no sub-modes: `message f0 DT fdop del nfiles snr`, 7 positional args.)

## 3. Generate the WAV corpus

```sh
scripts/gen_ft4_sweep_wavs.sh [ft4sim-path] [out-dir]
# defaults: ft4sim-path = target/ft4sim/ft4sim
#           out-dir     = embedded-poc/assets/ft4_sweep/
```

Covers 4 channel conditions (`awgn`, `ccir_good`, `ccir_moderate`,
`ccir_poor`) × the `SNRS` grid × `TRIALS` repeats (default 20). Same
idempotent/incremental behaviour as `gen_fst4_sweep_wavs.sh` — widening
the grid and re-running only tops up missing cells.

The default `SNRS` grid (`-5` down to `-23` dB) straddles FT4's
published WSJT-X AWGN threshold of about **-17.5 dB** (2500 Hz ref BW;
compare FT8's -21 dB — FT4 trades sensitivity for the shorter 7.5 s slot
and less FEC interleaving). Extend the grid deeper if the measured 50%
crossing turns out weaker than expected, per the FST4 #146
grid-censoring lesson.

## 4. Run the sweep

```sh
MFSK_FT4_SWEEP_DIR=embedded-poc/assets/ft4_sweep \
  cargo test --test ft4_sweep --release \
  --features ft4,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

`uvpacket` is only required because `tests/common/channel.rs` (pulled in
via `mod common`) unconditionally imports `mfsk_core::uvpacket` —
unrelated to FT4 itself.

Output is a plain recall table (decodes / trials per channel × SNR
cell) — no pass/fail assertions, this is a measurement tool.

### Measured 2026-07-18 (this corpus/seed, `DecodeStrictness::Normal`)

50% crossing, linear-interpolated between adjacent grid points:

| Channel | ~50% crossing |
|---|---:|
| AWGN | ≈ -15.5 dB |
| CCIR good | ≈ -14.7 dB |
| CCIR moderate | ≈ -14.3 dB |
| CCIR poor | ≈ -13.7 dB |

All about 2 dB worse than the -17.5 dB published figure. Not yet
root-caused — could be `ft4sim`'s SNR convention, a genuine pipeline
gap (candidate for the same style of investigation as #146), or the
published number itself referring to a different metric. Flagged here
rather than chased further in this pass; see section 6 of
`FST4_BENCHMARK.md` for the diagnose-before-fixing order to use if this
gets picked up.

## 5. `DecodeStrictness` probe (issue #72)

`ft4_strictness_probe` (same file) drives `engine::pipeline::decode_frame`
directly with each of `Strict` / `Normal` / `Deep` across a handful of
partial-recall cells identified by the sweep above, and reports both
"golden" recall (matches the known transmitted message) and "any-msg"
recall (any CRC-passing decode, golden or not — a proxy for false-accept
inflation, since every trial's true content is known here).

```sh
MFSK_FT4_SWEEP_DIR=embedded-poc/assets/ft4_sweep \
  cargo test --test ft4_sweep --release --features ft4,fft-rustfft,parallel,uvpacket \
  ft4_strictness_probe -- --ignored --nocapture
```

**Measured 2026-07-18 finding:** the knob is *not* a no-op for FT4,
contrary to the assumption that an uncalibrated copy-paste "probably
doesn't matter." Effect size is small in AWGN but substantial under
fading — e.g. `ccir_poor` at -13 dB: Strict 9/20 → Normal 14/20 → Deep
16/20 golden recall. But `Deep` also grows "any-msg" faster than
"golden" in several cells (`ccir_moderate` -16 dB: golden 1→2, any-msg
1→4), i.e. some of Deep's extra decodes are false-accepts, not real
sensitivity gain — the same sensitivity/false-positive trade-off the
original FT8 calibration comment describes, just unverified for FT4
until now.

Net: `Normal` (today's hardcoded default) already sits at a reasonable
point — it captures most of `Strict → Deep`'s real gain in the fading
cells that matter (`ccir_poor` -13 dB captures 5 of Deep's 7-decode
gain) without `Deep`'s worst false-accept growth. The open question
this doesn't yet answer is whether FT4-*specific* threshold numbers
(rather than the FT8-copied ones) could beat `Normal` without `Deep`'s
FP cost — that needs a numeric sweep over `osd_max_errors` /
`osd_score_min` values, not just the three existing named levels.

## 6. Numeric retune (issue #72, 2026-07-18)

Traced `strictness.osd_score_min()` / `strictness.osd_max_errors()` call
sites in `engine::pipeline::process_candidate_basic`: both are gated
behind `!is_fst4`, so **FST4 bypasses these values entirely** (the #146
fix) — in practice `engine::pipeline::DecodeStrictness`'s numbers are
FT4-exclusive. This means retuning `Normal` carries zero regression
risk against FST4's separately-tuned recall, clearing the way to
actually move the numbers instead of just characterising them.

Swept by hand: edit a candidate value, rerun `ft4_strictness_probe`
against the 8 partial-recall cells, keep the change if golden recall
went up without any-msg growing faster than golden (i.e. no new
false-accepts), discard otherwise.

- `osd_score_min`: `2.2 → 1.8`. `2.0` gained 3 cells with ~0 new FPs;
  `1.8` gained 2 more cells cleanly; `1.6` gained nothing further and
  only added false-accepts — `1.8` is the ceiling.
- `osd_max_errors`: `(depth3, depth4, other) = (26, 30, 29) → (28, 30, 31)`.
  `(29, 31, 32)` gave identical results (no benefit to going further);
  `(30, 33, 33)` added 2 false-accepts with zero additional golden
  recall — `(28, 30, 31)` is the ceiling.

Net effect on the full sweep (Normal, before → after):

| Channel | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|
| AWGN | 0%→10% | 20%→20% | 85%→85% | 95%→95% | 95%→95% |
| CCIR good | 5%→10% | 20%→20% | 40%→45% | 90%→100% | 90%→90% |
| CCIR moderate | 5%→10% | 5%→5% | 30%→40% | 70%→70% | 95%→95% |
| CCIR poor | 0%→0% | 10%→15% | 20%→25% | 45%→50% | 70%→75% |

Modest, monotonic, no regressions anywhere (loosening a gate can only
hold or increase recall) — a real but incremental gain, not a dB-scale
jump. Landed in `engine::pipeline::DecodeStrictness::{osd_max_errors,
osd_score_min}`; `Strict`/`Deep` numbers untouched (still the original
unverified FT8 copy — no current caller exercises them).

## 7. Coherent full-slot Δt search (`ft4_sync_search`, 2026-07-18)

Separately from the `DecodeStrictness` tuning above, a WSJT-X
faithfulness audit of `sync4d.f90`/`ft4_decode.f90` against our FT4 sync
pipeline found that `engine::sync::coarse_sync`'s wide (±2.5 s) Δt search
is *non-coherent* (magnitude-squared spectrogram bins) while WSJT-X's
own wide Δt search (`isync=1`, ~350-450 samples/segment × 3 segments) is
*coherent* (complex Costas correlation). `tests/ft4_coherent_wide_search_diag.rs`
confirmed the consequence empirically: under CCIR fading, `coarse_sync`'s
non-coherent pick can land 0.5-2.4 s from the true peak — far outside
the old local `sync2d_refine` (±20 downsampled samples ≈ ±30 ms) — even
though the true peak's *coherent* score was consistently higher. AWGN
was not affected (non-coherent gap stayed under ~16 ms, within the old
refine's reach).

Fix: `engine::sync2d::ft4_sync_search`, a coherent full-slot Δt search
mirroring WSJT-X's `isync=1`/`isync=2` loops (±12 Hz/3 Hz coarse ×
absolute `[-344, 1012]` downsampled-sample window step 4, then ±4 Hz/1 Hz
× ±5 samples fine), replacing `sync2d_refine`/`Sync2dConfig::for_ft4` for
FT4 (FST4 already had the equivalent `fst4_sync_search` from #146).

**First attempt was a net regression** (recall dropped even in clean
AWGN) — worth recording since it's a real trap: widening the Δt search
lets *more* of a real signal's own frequency-neighbour `coarse_sync`
candidates independently reach a genuine, self-consistent Costas lock
(not noise — confirmed both `sync_quality` and a from-scratch WSJT-X
`nsync_qual`-equivalent bit-metric gate score these highly). Tried and
discarded: (a) preferring the lowest-`hard_errors` duplicate on dedup —
worse, hard_errors doesn't correlate with position accuracy; (b) the
extra `nsync_qual`-style gate — proved WSJT-X's own gate design doesn't
apply here, since WSJT-X's simpler 1-candidate-per-frequency-peak
architecture doesn't generate these near-duplicate candidates in the
first place, so its gates were never meant to arbitrate between them.

**Actual root cause**: `DecodeResult.freq_hz` was being set from the
*pre-refinement* `cand.freq_hz`, not `refined.freq_hz` — a latent bug,
harmless before because the old narrow local refine only ever corrected
candidates that started close to the truth anyway. The wide search let
far-off candidates succeed too, each still reporting its own stale
`cand.freq_hz` — a report-layer bug, not a recall one. Fixed all three
`DecodeResult` construction sites in `process_candidate_basic` to use
`refined.freq_hz`; kept the dedup-by-`sync_score` tie-break (now mostly
cosmetic, since duplicates converge on nearly the same refined position
once reported correctly).

### Measured 2026-07-18 (vs section 6's `DecodeStrictness`-only baseline)

| Channel | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|
| AWGN | 10%→20% | 20%→30% | 85%→100% | 95%→95% | 95%→100% |
| CCIR good | 10%→20% | 20%→40% | 45%→65% | 100%→100% | 90%→95% |
| CCIR moderate | 10%→10% | 5%→5% | 40%→50% | 70%→80% | 95%→95% |
| CCIR poor | 0%→0% | 15%→20% | 25%→35% | 50%→55% | 75%→80% |

50%-crossing improvement (linear-interpolated): AWGN ≈+0.2 dB, CCIR
good ≈+0.7 dB, CCIR moderate ≈+0.3 dB, CCIR poor ≈+0.25 dB. No
regressions except CCIR poor -5 dB (100%→95%, one trial — within
sampling noise for 20 trials/cell). Verified against the full
non-ignored test suite (`cargo test --release --features full`) and
`ft4_wsjtx_sample_recall_vs_golden` — all green.

## 8. `nsync>=18` OSD-depth gate — real bug, confirmed not the AWGN gap (issue #72, 2026-07-18)

Following section 6's own "diagnose before fixing" principle (and the
identical rule in this file's sibling `FST4_BENCHMARK.md` §6): before
touching any more numbers, built `ft4_diag_weak_trials`
(`tests/ft4_sweep.rs`, mirrors `fst4_diag_weak_trials`) to trace
`coarse_sync` + `process_candidate_basic` directly on individual AWGN
trials straddling the post-section-7 ≈-15.7 dB crossing, rather than
guessing at a WSJT-X-vs-ours diff and re-running the full sweep to check.

**coarse_sync isn't the bottleneck at the crossing.** Across the -17..-14
dB AWGN cells (80 trials), only 8/20 (-17 dB) and 6/20 (-16 dB) trials had
no near-golden-frequency candidate at all — the dominant failure mode
(10/20 at both -17 dB and -16 dB) was a candidate found at the exact true
freq/dt, with a `coarse_sync` score in the same range as successful
trials, that still failed inside `process_candidate_basic`.

**Found while instrumenting that path**: `sync_quality`'s FT4 output
(`nsync`) is capped at `N_SYNC = 16` (4 Costas blocks × 4 symbols each,
`ft4/mod.rs:98`) — confirmed empirically, values across ~140 measured
candidates topped out at 16 and clustered 10-16 regardless of decode
success. But `process_candidate_basic`'s OSD depth-escalation gates
(`core/pipeline.rs`) checked `nsync >= 18` for both the depth-3 rung and
the depth-4 Top-K rescue — a threshold calibrated against FT8's
`N_SYNC = 21` (12/21, 18/21), copy-pasted as literals rather than scaled
per protocol. **18 is mathematically unreachable when the ceiling is 16**:
FT4 candidates could never get anything past OSD depth-2, silently
dropping the entire depth-3/depth-4 escalation tier — the same
copy-paste-from-FT8 pattern issue #72 already named for `DecodeStrictness`,
just in a different gate.

Fixed in `core/pipeline.rs`: `osd_attempt_min`/`osd_depth3_min` now scale
by `P::N_SYNC` using the ratio FT8's original 12/21 and 18/21 imply
(→ 9/14 for FT4's `N_SYNC=16`), gated on `P::ID == Ft4` so FT8
(`N_SYNC=21` — formula reproduces 12/18 exactly, byte-identical) and FST4
(already separately tuned, issue #146) are untouched.

**Re-verified narrow (AWGN only, -17..-14 dB, real `decode_frame` via
`ft4_snr_sweep`, not the diagnostic's simplified candidate loop)**: 4/20,
6/20, 20/20, 19/20 — **byte-identical** to section 7's post-tuning
baseline (20%, 30%, 100%, 95%). `ft4_diag_weak_trials` traced individual
`nsync>=14` trials that now correctly escalate to OSD depth-3
(confirmed via temporary tracing, since removed) and still fail — these
are genuinely SNR-limited, not depth-limited. Full non-ignored suite +
`ft4_wsjtx_sample_recall_vs_golden` (6/6) green.

**Verdict**: real, worth keeping (it's a genuine protocol-scale bug, and
untested corners — CCIR fading, busy-band multi-signal — weren't ruled
out), but **confirmed not the explanation for the ~1.8 dB AWGN gap**.
Exactly the kind of result section 6 warned about: a plausible-sounding,
source-verified diff that a full re-sweep shows didn't move the needle.
The two remaining open candidates from the original audit — (1) no
WSJT-X-style early `smax`-based candidate reject before bitmetrics are
computed at all, (2) unclear whether `ft4_sync_search` replicates
WSJT-X's 3-segment Δt search with monotonicity gating — are still
un-investigated.

## 9. `ft4_sync_search` scorer was non-coherent within each Costas block (issue #72, 2026-07-18) — ~1 dB AWGN gain

Prompted by a direct question about whether the `score` semantics feeding
`ft4_sync_search` and the `osd_score_min` gate actually mean the same
thing as WSJT-X's — the right question to ask before trusting section 8's
"scorer is faithful" note, which turned out to have only checked the
*outer* structure.

`ft4_sync_search`'s doc comment claimed the scorer matched `sync4d.f90`'s
`sync = p(z1)+p(z2)+p(z3)+p(z4)` "exactly." True at the outermost level
(4 Costas blocks, magnitude-summed, non-coherent block-to-block) — but
re-reading `sync4d.f90` line-by-line (not just the final formula) shows
each `z_k` is **one coherent dot product spanning all 4 symbols of block
k** (`z1=sum(cd0(i1:i1+4*NSS-1:2)*conjg(csync2))`, a single complex
accumulator built from a reference that concatenates all 4 symbols'
waveforms back-to-back). `ft4_sync_search` instead called
`score_costas_block` — which correlates **each symbol separately**, takes
`.norm_sqr()` (power) of each, and power-sums the 4 symbols. Non-coherent
combining across N=4 samples loses ~sqrt(4) ≈ 3 dB of discrimination vs
one coherent N=4 correlation — the identical mechanism already diagnosed
and fixed for FST4 (`project_fst4_coherent_sync` memory, issue #146):
`fst4_sync_search` already carries the correct machinery
(`make_costas_ref_continuous` + `score_flat_coherent`, one flat reference
per block, correlated once, magnitude taken) — it just hadn't been wired
up for FT4's block-by-block loop.

Cross-checked against `sync8d.f90` (FT8) to confirm `score_costas_block`
itself isn't wrong: FT8's own combining really is per-symbol,
non-coherent, power-summed (`sync = sync + p(z1)+p(z2)+p(z3)` inside a
loop over each of the 7 Costas symbol positions, `p(z)` with no sqrt —
pure power) — `score_costas_block` is faithful *for FT8*, just wrong when
reused for FT4's genuinely different (per-block-coherent) combining
style. Fix scoped to `ft4_sync_search` alone (`core/sync2d.rs`): swapped
`make_costas_ref`/`score_costas_block` for
`make_costas_ref_continuous`/`score_flat_coherent`, reusing the exact
helpers `fst4_sync_search` already uses. `score_costas_block` itself,
`sync2d_refine` (shared by other protocols), and FT8's own fine-sync are
untouched.

**Measured** (`ft4_snr_sweep`, `--ignored --nocapture`, AWGN -20..-13 dB
+ all 3 CCIR channels -18..-12 dB, real `decode_frame`, not the diag
harness):

| Channel | -18 dB | -17 dB | -16 dB | -15 dB | -14 dB | -13 dB |
|---|---:|---:|---:|---:|---:|---:|
| AWGN | 0%→0% | 20%→40% | 30%→75% | 100%→100% | 95%→100% | —→100% |
| CCIR good | —→10% | 20%→45% | 40%→60% | 65%→95% | 100%→100% | 95%→95% |
| CCIR moderate | —→5% | 10%→10% | 5%→35% | 40%→65% | 70%→95% | 95%→95% |
| CCIR poor | —→0% | 0%→0% | 15%→25% | 25%→50% | 50%→75% | 75%→90% |

(AWGN -20/-19/-18 dB stayed at 0% both before and after — crossing sits
safely inside the grid, not pinned at the floor.)

50%-crossing (linear-interpolated): **AWGN -15.7→-16.7 dB (+1.0 dB)**,
CCIR good -15.6→-16.7 dB (+1.1 dB), CCIR moderate -15.0→-15.5 dB
(+0.5 dB), CCIR poor -14.25→-15.0 dB (+0.75 dB). No regressions anywhere
in the swept range. This is the single largest AWGN gain of the whole
issue #72 investigation — more than half of the ~1.8 dB gap against
WSJT-X's published -17.5 dB closed in one fix (residual gap now ≈0.8 dB).
Fading channels gained less than AWGN, consistent with coherent
combining being partially undermined by channel decorrelation over the
4-symbol block span — a plausible, not yet separately verified,
explanation.

`ft4_wsjtx_sample_recall_vs_golden`: still 6/6, and the busy WSJT-X
sample WAV now yields 13 total CRC-passing decodes (was 11) — the two
new ones have well-formed callsign/grid/report fields, consistent with
recovering genuine weak signals rather than new false-accepts. Full
non-ignored suite green (350+ tests, 0 failed).

Reinforces the section 8 lesson from the opposite direction: don't stop
verifying a "matches WSJT-X" claim at the outermost formula when the
question is specifically about numerical/scale fidelity — the bug was
one level deeper than the code comment had actually checked.

## 10. BP/OSD decode strength — mostly ruled out, one faithfulness fix, null on AWGN (issue #72, 2026-07-18)

With the sync-side fixes (sections 8-9) landed, checked whether the
residual ≈0.8 dB AWGN gap is instead a decode-strength issue (LDPC
BP/OSD weaker than WSJT-X's `decode174_91`) — the general concern
`project_wsjtx_compliance_audit.md` flagged as "BP algorithm (tanh-product
vs min-sum NMS) — the biggest general gap."

**BP algorithm: already faithful on host, concern doesn't apply here.**
`fec::ldpc::bp::FecOpts::default()` sets `bp_kind: BpKind::SumProduct`
(`core/protocol.rs:384`) — the true log-domain tanh/atanh belief
propagation (`tov = 2·atanh(−∏tanh(−toc/2))`), matching WSJT-X's
`bpdecode174_91.f90:101,107-110` algebraically. `BpKind::NormalizedMinSum`/
`OffsetMinSum` exist in the same module but are only used by the embedded
fixed-point path (`bp_decode_generic_nms`, doc'd as embedded-only in
`bp.rs`) — not exercised by any host test, including this whole AWGN
sweep. The compliance-audit concern is real for the embedded path but
doesn't explain anything measured here.

**OSD barely gets exercised.** Extended `ft4_diag_weak_trials` to print
`DecodeResult.pass` (which LLR variant / BP vs OSD rescue) and
`hard_errors`. Across 45 successful near-crossing decodes (AWGN
-18..-15 dB): 43 succeeded on plain BP (pass 0/1/2, no OSD needed), only
2 needed OSD depth-3 rescue (pass 5), and OSD depth-2/4 (pass 4, or the
Top-K path) never fired at all. `nsync` and `score` are both attempted
(`osd@9=true`) for nearly every one of the 17 failing candidates in the
same trace, and their values heavily overlap successes' — no separation
visible, consistent with genuinely SNR-limited failures rather than a
gating artifact.

**Found and fixed one real BP parameter mismatch, but it's null on AWGN.**
Checked WSJT-X's own `max_iterations` (BP iteration budget) per protocol:
`ft8b.f90:96` and `fst4/decode240_101.f90:27` both use 30 (matching our
shared `bp_max_iter: 30` hardcoded in `core/pipeline.rs`), but
`ft4_decode.f90:194` uses **40** — FT4 is the outlier, not FT8/FST4.
Scoped the fix to `P::ID == Ft4` (same pattern as sections 8-9), leaving
FT8/FST4 byte-identical. Re-verified narrow (AWGN -19..-14 dB): **byte-
identical** to section 9's post-fix baseline (40%/75%/100%/100%) — BP
convergence isn't iteration-budget-limited in this range; candidates
either converge well under 30 iterations or don't converge regardless of
budget. Kept anyway (zero cost, now byte-faithful to WSJT-X, and may
matter for fading/busy-band cases this narrow AWGN check didn't cover).

**Verdict**: BP/OSD strength is not the source of the residual AWGN gap.
The algorithm is already WSJT-X-faithful on host, OSD's marginal
contribution here is too small (~4% of successes) to be hiding a large
effect, and the one real parameter gap found doesn't move AWGN recall.
`ft4_wsjtx_sample_recall_vs_golden` still 6/6, full non-ignored suite and
`-D clippy::perf` green. The two candidates from section 8's closing
notes — WSJT-X's early `smax`-based candidate reject before bitmetrics,
and the 3-segment Δt-search structure — remain the most likely places
left to look for the remaining ≈0.8 dB.

## 11. 3-segment Δt-search retry — implemented, measured, reverted (issue #72, 2026-07-18)

WSJT-X's `ft4_decode.f90` `iseg=1,2,3` loop doesn't just *find* a position
more thoroughly than a single collapsed pass — it can attempt a **decode**
at up to 3 different Δt positions per candidate (segment 1 `[108,560]`
always tried first if `smax≥1.2`; segments 2/3 tried too, but only if
their own `smax` beats segment 1's). `ft4_sync_search`'s single collapsed
full-window pass (section 9) picks one global-best position and decodes
once — in principle vulnerable to a higher-scoring spurious position
elsewhere in the wider union window beating the true signal's own
(lower-scoring but genuine) position at low SNR.

Built `ft4_sync_search_window` (`core/sync2d.rs`, `ft4_sync_search` now a
thin wrapper over it with the full `[-344,1012]` union) to let a
diagnostic replicate WSJT-X's literal per-segment search, and
`ft4_diag_segment_retry` (`tests/ft4_sweep.rs`) to check, for every
currently-failing candidate, whether decoding at segment 1's position
alone (not the collapsed global best) would have succeeded.

**First pass over-reported dramatically**: 10/17 "rescues." Implemented
the fix in `process_candidate_basic` (FT4-only 3-segment loop replacing
the single collapsed call) and re-verified with the real `ft4_snr_sweep`
— **byte-identical** to the pre-fix baseline, 0 movement. The diagnostic
was wrong, not the theory: `try_decode_at` tried OSD unconditionally
without the `hard_errors ≥ osd_max_errors` rejection gate real OSD
results go through, so it was accepting exactly the kind of over-
corrected, low-confidence codeword that gate exists to reject. Fixed the
diagnostic to apply the same gate and check the decoded message against
`GOLDEN_MSG` (not just "CRC passed") — corrected result: **0/17
rescues**, matching the real sweep exactly. Reverted the
`process_candidate_basic` segment loop back to the single collapsed
pass (avoids 3x the search/decode cost per FT4 candidate for zero
measured benefit); kept `ft4_sync_search_window` since it's harmless,
reusable, and the diagnostic (`ft4_diag_segment_retry`) is worth keeping
for future re-checks under CCIR fading or busy-band, where a spurious
competing peak is more plausible than in clean AWGN.

Lesson, stated plainly: a diagnostic that skips a real production gate to
"give a hypothesis its best chance" can manufacture a false positive
large enough to look like the answer. Re-verifying against the *actual*
production code path (not just a simplified stand-in) — this file's
recurring theme since section 6 — caught it before any code shipped on
the strength of the flawed number.

## 12. `osd_score_min` gate on the wrong (stale) score — real fix, ~0.5 dB further AWGN gain (issue #72, 2026-07-18)

While building section 11's diagnostic, printing `cand.score` alongside
decode outcomes surfaced a separate, real issue: `cand.score` is
`coarse_sync`'s non-coherent score — a different, unrelated quantity
from the coherent score `ft4_sync_search` computes (the one section 9
fixed to be WSJT-X-faithful). `strictness.osd_score_min()` (`1.8`) was
tuned in section 6 against `cand.score`, back before the coherent score
existed as a separate quantity, and `process_candidate_basic`'s
OSD-attempt gate (`core/pipeline.rs`) still checks `cand.score`, not the
coherent one.

Measured directly (`ft4_diag_weak_trials`, 17 currently-failing
near-crossing AWGN candidates): **13/17 (76%) have `cand.score < 1.8`**
and so never even attempt OSD — plain BP already failed, and that's the
only rung they ever get. All 17 clear WSJT-X's own `syncmin=1.2` easily
(the coherent score, correctly computed) — these are unambiguously real
signals, not noise the gate is protecting against.

WSJT-X's own FT4 decoder has no OSD-attempt score gate at all —
`decode174_91` runs BP and OSD together in one call, governed by
`ndepth`/`maxosd`, never by a score check. Section 8's FST4 bypass
(`is_fst4`) already exists for the identical symptom, found during #146:
"every real candidate's coarse-sync score sat below `osd_score_min`
(blocking OSD entirely)." Extended the same bypass to FT4
(`bypass_osd_score_min = is_fst4 || P::ID == Ft4`), keeping
`osd_max_errors` (a genuine hard-error ceiling, not a stale-quantity
gate) as the false-accept safety net — unlike FST4, which bypasses both.

**Measured** (`ft4_snr_sweep`, real `decode_frame`):

| Channel | -19 | -18 | -17 | -16 | -15 | -14 | -13 |
|---|---:|---:|---:|---:|---:|---:|---:|
| AWGN | 0%→0% | 0%→15% | 40%→60% | 75%→80% | 100%→100% | 100%→100% | 100%→100% |
| CCIR good | —→0% | 10%→15% | 45%→65% | 60%→75% | 95%→95% | 100%→100% | 95%→95% |
| CCIR moderate | —→5% | 5%→15% | 10%→15% | 35%→40% | 65%→80% | 95%→100% | 95%→95% |
| CCIR poor | —→0% | 0%→0% | 0%→0% | 25%→55% | 50%→60% | 75%→85% | 90%→90% |

No regressions anywhere. 50%-crossing (linear-interpolated): **AWGN
-16.7→-17.2 dB (+0.5 dB)** — WSJT-X's published -17.5 dB gap now only
≈0.3 dB. CCIR good -16.7→-17.3 dB (+0.6 dB), CCIR moderate -15.5→-15.75 dB
(+0.25 dB), CCIR poor -15.0→-16.1 dB (+1.1 dB, the biggest single-channel
gain — the poor-fading trials evidently had the most candidates sitting
in the blocked `cand.score<1.8` band).

`ft4_wsjtx_sample_recall_vs_golden`: still 6/6, still 13 total decodes
(no new false-accepts on the real sample — the `osd_max_errors` gate
still in place is doing its job). Full non-ignored suite and
`-D clippy::perf` green.

**Cumulative issue #72 AWGN result**: -15.5 dB (pre-tuning) → -17.2 dB
(now), essentially closing the gap to WSJT-X's published -17.5 dB. Three
real, WSJT-X-source-verified fixes contributed (sections 6, 9, 12); two
plausible-looking candidates were built, measured, and correctly
discarded (sections 8, 11) rather than shipped on faith.

## 13. Coarse-candidate stage was the wrong algorithm — `getcandidates4.f90`
    faithful port (`ft4_coarse_sync`), ~25× decode-speed win (2026-07-20)

A follow-up investigation (not #72 — the `dapper-soaring-nest` plan,
prompted by a request to both improve coarse-sync precision and BP/OSD
speed) started from a different observation than sections 1-12: FT4's
golden-WAV decode was the slowest in the whole protocol suite (1.20 s
for a 6-signal, 7.5 s recording — see `BENCHMARKS.md`'s "Decode speed"
table), and the golden test needed `max_cand=2000` despite its own code
comment claiming 200 should suffice.

**Root cause, confirmed by reading `lib/ft4/getcandidates4.f90` +
`lib/ft4/ft4_baseline.f90` line-by-line**: `engine::sync::coarse_sync`
(shared with FT8/FST4/etc, used for FT4 until this section) is a 2-D
(freq × lag) Costas-array correlation search — structurally *not* what
WSJT-X does for FT4. `getcandidates4` never searches a lag/Δt dimension
at all: it's a pure frequency-domain periodogram (Nuttall-windowed FFT
over overlapping 4-symbol-wide segments, time-averaged, 15-bin
boxcar-smoothed, normalised by `ft4_baseline`'s 5-term-polynomial noise
floor, one candidate per local-max frequency peak). FT4's actual Δt
determination happens entirely later in `ft4_sync_search` (section 9),
which already does an *absolute* full-window search ignoring each
candidate's own `dt_sec`. Consequence: every one of the generic search's
up-to-8 lag-distinct candidates per frequency is functionally redundant
downstream — each independently pays the full `ft4_sync_search` + LLR +
BP + OSD cost for what should converge on the same result.

**Measured before any change** (`ft4_diag_candidate_cost_split`,
`tests/ft4_sweep.rs`, golden WAV): 2000 candidates across only 440
distinct frequencies (4.5× redundancy), and `ft4_sync_search` alone
accounted for 5.09 s of the ~6.65 s summed per-candidate cost (LLR+BP+OSD
only 1.46 s) — confirming BP/OSD itself was never the bottleneck; the
fix belongs at candidate generation, matching that `fec::ldpc::bp`
already breaks on parity/CRC convergence and OSD depth-4 essentially
never fires (section 10).

**Fix**: `engine::ft4_coarse::ft4_coarse_sync` — a faithful
`getcandidates4.f90` + `ft4_baseline.f90` port, reusing existing faithful
primitives (`engine::baseline::fit_baseline`, already unit-tested but
previously only tried — and reverted — as a mismatched normaliser for
the generic Costas-correlation score; `engine::sync::parabolic_peak`).
Lives in `engine::` (not `ft4::`) alongside `ft4_sync_search`, since
`engine` compiles regardless of which protocol features are enabled and
`engine::pipeline` needs to call it unconditionally. Wired in via a
`P::ID == Ft4` branch in `engine::pipeline::decode_frame` /
`decode_frame_subtract`, same pattern as the existing sync2d-refine
branch. One deliberate deviation from WSJT-X: candidates are score-sorted
before truncating to `max_cand` (WSJT-X's own frequency-scan-order fill
is a Fortran fixed-array convenience, not an intentional ranking).

**Measured after** (same diagnostic, golden WAV): 31 candidates across 31
distinct frequencies (redundancy eliminated — exactly 1.0/freq), summed
`ft4_sync_search` cost 75.7 ms, LLR+BP+OSD 25.8 ms. Real production
entry point (`decode_frame_subtract`, rayon-parallel, 3 subtract passes):
**1.20 s → 48.8 ms (~25×)**. `ft4_wsjtx_sample_recall_vs_golden` stayed
**6/6** (12 total decodes, was 13 — one fewer near-duplicate/marginal
accept, not a golden miss). `max_cand` in the golden test dropped
2000→100 with byte-identical recall — 100 also happens to match WSJT-X's
own `MAXCAND=100` in `ft4_decode.f90`, unplanned but reassuring symmetry.

**Update (2026-07-26): this 48.8 ms figure went stale, then was fixed
(issue #182).** Issues #178-#180 (see `FT8_BENCHMARK.md` section 8)
migrated FT4's `decode_frame_subtract` from the cheap constant-amplitude
subtract this section measured onto the same WSJT-X-faithful
channel-aware LPF subtract FT8 uses, for recall-quality reasons — but
this row was never re-measured afterward, and regressed to ~526-576 ms
(median of several runs, both `decode_frame_subtract(&audio, 100.0,
2700.0, 0.05, 100)` directly and via this file's own
`ft4_diag_candidate_cost_split` instrumentation).

Root cause (found by adding temporary `Instant` timers around the two
calls the SIC subtract loop makes per accepted candidate): not
`subtract_tones_lpf` itself (already FFT-cached from issue #180, <1 ms/
call) but `engine::dsp::subtract::refine_freq`, at ~35 ms/call × 14 real
decodes ≈ 490 ms — essentially the entire regression. `refine_freq`
grid-searches ±5 Hz at 0.1 Hz resolution (~101 evaluations) to compensate
for `ft4_coarse_sync`'s ~5.2 Hz-bin carrier estimate before subtracting;
every evaluation called `generate_iq` → `synth_complex_f32_into`, which
rebuilt the **entire** GFSK-shaped modulation from scratch each time
(erf-based Gaussian pulse table, `O(nsym·pulse_len)` per-symbol
convolution, full `O(nwave)` phase-integration + `sin`/`cos` loop) even
though only the carrier frequency differs between the 101 evaluations of
one call — the same "recompute something invariant across a search
loop" bug pattern as this file's `fine_refine_3stage` fix.

**Fix**: `generate_iq`'s carrier term is added uniformly to every sample
before phase integration, so `phi(k; f0) = phi_mod(k) + k·(2π·f0·dt)` —
any carrier is a pure linear phase ramp on top of the carrier-free
(tone-modulation-only) phase. `refine_freq` now builds the carrier-free
phasor once per call and derives each grid point via a cheap per-sample
NCO rotation + angle-addition instead of a full resynthesis
(`ls_amp_mag_tweaked`, `core/dsp/subtract.rs`), with periodic rotor
renormalization to bound f32 drift over the ~59k-sample buffer. Verified
against the frozen full-resynthesis path with a differential test
(`ls_amp_mag_tweaked_matches_full_resynthesis`, combined absolute/
relative tolerance — the LS amplitude has a comb of deep correlation
nulls a few tenths of a Hz apart, where relative error alone isn't a
meaningful metric) and an argmax-preservation test
(`refine_freq_finds_true_offgrid_carrier`).

Measured result: `refine_freq` ~35 ms/call → ~15.7 ms/call
(`RAYON_NUM_THREADS=1`, isolating the per-call cost from thread count).
`decode_frame_subtract` real production wall-clock: multi-threaded
(default rayon, matching how the original 48.8 ms figure was measured)
~575.8 ms → **~280 ms**; single-threaded ~893.6 ms → **~602 ms**. Not a
full return to 48.8 ms — the LPF subtract + frequency-refine step is a
deliberate, permanent recall-quality addition (10/10 vs 0/10 on the
Rayleigh-faded-interferer scenario noted above) that will always cost
more than the pre-#178 constant-amplitude path; this fix removes the
*redundant* part of that added cost, not the added cost itself. Candidate
count unchanged (31/31, no redundancy) and recall unaffected (still
6/6). See `BENCHMARKS.md`'s "Decode speed" table for the current
headline number.

**One real regression found and fixed, in the test suite, not
production**: `ft4_roundtrip.rs`'s two clean-signal round-trip tests
started failing. Root cause (confirmed via a temporary in-crate debug
test surveying 18 different messages, 0/18 landing near the true
carrier): those two tests search a ~400-600 Hz band tight around the one
transmitted signal — with no other content anywhere in range,
`fit_baseline`'s per-segment low-percentile noise-floor fit has nothing
but signal to fit, and its estimate comes out systematically wrong. Widening
that band to 100 Hz to 2700 Hz (freq_min, freq_max) — the same convention
every other FT4 caller in this repo already uses — fixed both tests
outright (18/18 in the survey). An initial attempt to fix this by adding
synthetic dither noise instead was tried and discarded first: it didn't
work even at ~16 dB SNR (sigma=3000), and a follow-up 18-message survey
confirmed the failure was 0/18 regardless of noise level for a narrow
band — deterministic, not noise-sensitive, so noise was never going to
fix it. This isn't a WSJT-X faithfulness gap: `getcandidates4`'s baseline
fit assumes a realistically wide working band with genuine off-signal
content, exactly WSJT-X's own real usage (~200-4910 Hz) and every other
caller here — the two round-trip tests' narrow band was the outlier,
an assumption that only happened to hold under the old Costas-correlation
search's different (baseline-free) scoring.

**AWGN/CCIR sweep, re-measured in full** (`ft4_snr_sweep`,
`--ignored --nocapture`): 50%-crossing changed from a pure
candidate-generation swap, not a threshold retune, so (unlike sections
6-12) the "loosening a gate can only hold or increase recall" invariant
doesn't strictly apply here — a structurally different algorithm can
legitimately reshape the SNR-response curve rather than uniformly
improve or hold it.

| Channel | 50% crossing (before) | 50% crossing (after) | Δ |
|---|---:|---:|---:|
| AWGN | -17.2 dB | -16.9 dB | -0.3 dB (regressed) |
| CCIR good | -17.3 dB | -17.5 dB | +0.2 dB |
| CCIR moderate | -15.75 dB | -15.7 dB | ~0 (noise) |
| CCIR poor | -16.1 dB | -16.0 dB | ~0 (noise) |

Three of four channels are equal-or-better; AWGN alone softened by
~0.3 dB (widening WSJT-X's published-gap comparison from ~0.3 dB to
~0.6 dB) — but not uniformly: AWGN -19/-18 dB actually *improved*
(0%→5%, 15%→25%) while -17 dB alone dropped (60%→45%, a 3-trial swing
out of 20), reshaping rather than uniformly shifting the curve. Given
the ~25× decode-speed win and that 3/4 channels didn't regress, this
trade was accepted rather than chased further within this pass; revisit
if the AWGN gap needs closing again specifically (candidate: whether
`ft4_coarse_sync`'s 15-bin smoothing width interacts with `sync_min`
placement differently near the AWGN crossing specifically — untested).

Full non-ignored suite (922 passed, 0 failed) and
`-D clippy::perf -D warnings` green.

## 14. Dead-code follow-up: `sync2d_refine`/`Sync2dConfig` removed (2026-07-20)

Section 7 above (and #146 for FST4) already moved both protocols off
the shared two-pass *local* refine (`engine::sync2d::sync2d_refine` /
`Sync2dConfig`, ±10-±20 downsampled-sample window around the coarse-sync
candidate) onto their own full-slot coherent searches
(`ft4_sync_search`/`fst4_sync_search`) — but the old function and its
config struct were never deleted, just left unreferenced. Grepped the
whole repo (`mfsk-core/src`, `mfsk-core/tests`, `embedded-poc`,
`mfsk-ffi*`) for actual call sites (not doc-comment mentions): zero.
Removed `sync2d_refine`, `Sync2dConfig`, and the two helpers
(`twiddle_ref`, `score_at<P>`) only reachable from it, along with the
now-unused `make_costas_ref`/`score_costas_block` imports —
162 lines. `Sync2dResult` (the shared output type both live search
functions still return) stays. Verified: full non-ignored suite green,
`-D clippy::perf -D warnings` clean, and separately checked FST4-only /
FT8-only (no FT4/FST4) feature builds still compile — this module has
no `#[cfg(feature = "ft4")]` gate of its own (same reasoning as
`ft4_coarse`: `engine` compiles unconditionally), so a stray reference
from either protocol's exclusion would have shown up there.

## 15. Phase 4 `smax` early-reject — implemented, measured, reverted (2026-07-20)

Section 13's Phase 4 was explicitly conditional: only pursue a
per-candidate early exit (WSJT-X's `ft4_decode.f90:279
if(smax.lt.1.2) cycle`) if profiling still showed LLR/BP/OSD as a
meaningful wall-clock share after the coarse-sync fix landed. Revisited
anyway on request, with the same "measure before picking a number"
discipline section 6 established.

**Calibration** (`ft4_diag_smax_calibration`, `tests/ft4_sweep.rs`, new):
ran the current production candidate generator (`ft4_coarse_sync`) +
`ft4_sync_search` across the full AWGN/CCIR near-crossing region (17
channel/SNR cells × 20 trials), recording the coherent `score` for every
candidate against whether it decoded to `GOLDEN_MSG`. 142
golden-succeeding candidates, minimum observed score **266.1**; 16858
non-golden candidates. A first read of "16418/16858 (97.4%) score below
the golden floor" looked like a strong filtering opportunity.

**Implemented**: a gate in `process_candidate_basic` (`core/pipeline.rs`)
— `if P::ID == Ft4 && score < 200.0 { return None }` — right after
`ft4_sync_search`, before `symbol_spectra`/LLR/BP/OSD. `200.0` was
chosen with a ~25% safety margin below the observed 266.1 floor, not a
guessed translation of WSJT-X's own `1.2` (different absolute scale:
ours is a magnitude-sum over 4 Costas blocks on unit-RMS-normalised
`cd0`).

**Measured**: golden WAV stayed 6/6 (12/12 decodes, byte-identical), the
full AWGN/CCIR sweep was byte-identical to the pre-gate baseline, full
suite (922 tests) and clippy clean — recall-safe as designed. But
re-checking what fraction of non-golden scores actually fell *below the
200.0 cutoff itself* (not the 266.1 golden floor the first read
compared against) gave a very different number: **91/16858 (0.5%)**.
The "97.4% below the golden floor" statistic was true but misleading —
junk-candidate scores cluster tightly in a band just *below* 266.1
(mostly in [200, 266)), not spread out far below it. There is no gap
between "safe" and "effective": widening the cutoff toward 266 to catch
more junk would erode the safety margin against the exact same
candidates that motivated keeping it small.

**Reverted.** The gate added real code and a magic-number cutoff for a
measured 0.5% reduction in per-trial candidate-processing work — at
already-single-digit-millisecond wall-clock costs post section 13, not
worth the maintenance burden. Kept `ft4_diag_smax_calibration` (now
rayon-parallelised across the (channel, SNR, trial) grid — flagged
during this pass as a general reminder to parallelise this file's
diagnostics at the file/trial level, not just leave `process_candidate_basic`'s
own internal work serial, per the existing `ft4_snr_sweep` convention
above) as a reusable reference for anyone revisiting this later with a
different corpus or a different score quantity.

Full non-ignored suite (922 passed) and `-D clippy::perf -D warnings`
green both before and after the revert.

## 16. `refine_freq`'s search radius was 5x too wide (issue #182 follow-up, 2026-07-26)

Follow-up to section 13's update: issue #182 (`FT8_BENCHMARK.md`, same-day
`fine_refine_3stage` port) fixed `refine_freq`'s dominant cost — full GFSK
resynthesis per grid point — by switching to a per-sample NCO tweak of a
once-built carrier-free reference (`ls_amp_mag_tweaked`). That dropped
`refine_freq` from ~35 ms/call to ~15.7-16.2 ms/call and
`decode_frame_subtract`'s golden-WAV wall-clock from ~530-580 ms to
~280 ms — a real fix, but still 5.7x the pre-#178 baseline of 48.8 ms, and
the user-reported "still slow" symptom this section addresses.

Re-measured with a standalone microbenchmark isolating `refine_freq` /
`subtract_tones_lpf` from the rest of `decode_frame_subtract` (14 calls
each, matching the golden WAV's real accepted-decode count):
`refine_freq` alone accounted for 227 ms of the 280 ms total (81%,
16.2 ms/call); `subtract_tones_lpf` only 13 ms (5%, already FFT-cached
per issue #180). Confirms `refine_freq`'s ±5 Hz/0.1 Hz grid (101
evaluations/call) — not `subtract_tones_lpf` — is still the bottleneck
after the NCO fix; the NCO fix cut *per-evaluation* cost, not the
*evaluation count*.

**Root cause of the evaluation count being unnecessarily large**: cross-
checked `refine_freq`'s call-site comment ("+/-5 Hz refine radius" —
claimed to match WSJT-X) against `lib/ft4/subtractft4.f90` directly.
WSJT-X's `subtractft4` has **no frequency-refine step at all** — it
synthesizes the reference at the decoded `f0` and subtracts directly, no
grid search, no `ctwk`-style tweak (that mechanism is FT8-only,
`sync8d.f90`, and operates during *sync*, not *subtract*). The call-site
comment's "matches WSJT-X" claim was simply wrong; `refine_freq` is an
mfsk-core-specific step compensating for this codebase's own coarse-sync
frequency resolution, not a port of anything WSJT-X does in the subtract
path.

The ±5 Hz figure traces to `refine_freq`'s own doc comment, written
against the *generic* `engine::sync::coarse_sync`'s ~2.93 Hz FFT-bin
resolution ("recommend 2.5 Hz to cover one bin either side"). But FT4
stopped using that generic coarse-sync path back in section 13
(`ft4_coarse_sync`, a `getcandidates4.f90` port with its own, different
bin structure) — and more directly, `r.freq_hz` going into `refine_freq`
is `process_candidate_basic`'s post-`ft4_sync_search` refined value
(section 7's own fix), not a raw coarse-sync bin. Reading
`engine::sync2d::ft4_sync_search`'s df search (`core/sync2d.rs`) line by
line: both its coarse pass (`idf` in `-12..=12` step 3) and fine pass
(`si` in `-4..=4` step 1) only ever produce **integer-Hz** `df` values —
the reported `freq_hz` is always `candidate.freq_hz + <integer>`. That
bounds the true continuous optimum to within ±0.5 Hz of the reported
value by construction — a much tighter guarantee than the generic
coarse-sync comment's ±2.5 Hz assumed, and the ±5 Hz radius was never
re-derived when FT4 moved onto this tighter-bound path.

**Fix**: `ft4/decode.rs`'s `decode_frame_subtract_with_options` call
site — `refine_freq_radius_hz` `5.0 → 1.0` (kept the existing 0.1 Hz
step, since the mainlobe of `ls_amp_mag_tweaked`'s frequency response is
narrow enough — well under 1 Hz for a ~7.5 s tone train — that widening
the *step* risks skipping over it entirely; only the *radius* was
oversized). Cuts the grid from 101 to 21 evaluations/call, a margin of
0.5 Hz still on each side of the ±0.5 Hz quantization bound above.

**Measured**: golden WAV `decode_frame_subtract` wall-clock **280.3 ms →
110.2 ms** (~2.5x), recall byte-identical (6/6 golden, 14/14 total
decodes, same messages/freq/dt as before). The section-11-referenced
busy-band Rayleigh-fading regression guard (`ft4_busy_band_fading_probe.rs
::busy_band_fading_baseline`, the exact scenario the LPF-subtract
migration in issues #177-179 was built to fix) stayed **10/10** target
recoveries. Full non-ignored suite and `-D clippy::perf -D warnings`
green. Cumulative from the pre-#178 baseline: 48.8 ms → 110.2 ms (2.3x
remaining gap) — the residual is the LPF subtract + freq-refine step's
now-irreducible cost (21 `ls_amp_mag_tweaked` evaluations + one
`subtract_tones_lpf` FFT pair per real decode), a deliberate
recall-quality addition per section 13's original update, not redundant
work.

## 17. First hardware measurement — M5Stack CoreS3 (2026-08-29)

FT4 had never been compiled for a board, let alone run on one. This
section is the first device number and the reason it does not yet fit.
Full write-up, including what had to be built to get there, is
`docs/reference/EMBEDDED.md`'s "FT4 on embedded"; this is the
benchmark-side record.

**Setup.** `ft4-bench` (`embedded-poc/m5stack-cores3-app/src/bin/`),
M5Stack CoreS3 @ 240 MHz, `opt-level = 3`, single core, no WiFi. The
31 coarse candidates `ft4_coarse_sync` finds on the in-tree WSJT-X
golden `000000_000002.wav` over 100–2700 Hz, `sync_min = 0.05`,
`max_cand = 100` — the same search
`ft4_wsjtx_sample_recall_and_precision` runs, minus its
`.sic_rounds(3)` (embedded FT8 ships a single pass, so a multi-pass
figure would not describe what a board would run). Assets baked by
`ft4_bake_golden_precomputed`. Log:
`embedded-poc/m5stack-cores3-app/logs/ft4-bench_clean_2026-08-29.log`.

**Budget**: 7.5 s slot − (0.5 s TX offset + 105 × 48 ms) = **1.96 s**.

| stage | total | per candidate | share |
|---|---:|---:|---:|
| `downsample_cached` (5120-pt inverse FFT) | 2 252 ms | 72.7 ms | 13 % |
| **`ft4_sync_search`** | **13 225 ms** | **424 ms** | **76 %** |
| LLR + BP (`DecodeDepth::EMBEDDED`) | ~1 861 ms | ~60 ms | 11 % |
| **total (production `process_candidate_basic`)** | **17 339 ms** | 559 ms | — |
| same at `DecodeDepth::FULL` | 19 684 ms | 635 ms | — |

**11 distinct decodes on device, identical to the host on the same
assets, at both depths.** So `DecodeDepth::EMBEDDED` costs no recall
here and OSD earns nothing on this file — worth knowing before anyone
proposes dropping OSD as the optimisation. (The 14/14 the golden test
asserts needs `sic_rounds(3)`; single-pass tops out at 11 on host too.)

**17 339 ms against 1 960 ms is 8.8× over**, and the excess is one
function.

**The cost is structural.** `ft4_sync_search` per candidate across all
31: **min 423 835 µs, p50 423 897 µs, max 424 684 µs** — 0.2 % spread.
That is a fixed grid, not anything candidate-dependent, and it follows
directly from section 7's own design: the search walks the absolute
`[-344, 1012]` downsampled-sample window for every candidate regardless
of its `dt_sec`, ~19 900 (Δf, Δt) cells × 4 Costas blocks × 4 symbols ×
32 samples ≈ 10.2 M complex MACs. 424 ms of that is ~10 cycles per
complex MAC — a scalar f32 loop with an on-the-fly phasor rotation.

Two levers, neither tried, both measurable against this bench:
`dsps_dotprod_f32_aes3` on the inner product, and narrowing the search
window (WSJT-X searches the full slot because it cannot assume a clock;
a UTC-anchored board can, and `wspr-fano-cap-fast` / `wspr-pass2-topn`
are the precedent for an embedded-only trade documented with the recall
it costs).

Host reference for the same 31 candidates, sequential
(`ft4_diag_candidate_cost_split`, `tests/ft4_sweep.rs`): ~89 ms, so the
device/host ratio is ~195×. For scale, FST4-60's first embedded number
(issue #306) was ~1728× before its optimisation passes.

**Not measured here**: `ft4_coarse_sync` itself (`NFFT1 = 2304` = 256 ×
9, no ESP-DSP kernel yet — the candidate list is baked instead). It is
0.3 ms of the host slot, so it does not change the conclusion.

## 18. Δt-window narrowing — what it costs in recall (2026-08-29)

§17 put `ft4_sync_search` at 76 % of an 8.8×-over budget, at a 0.2 %
per-candidate spread — a fixed grid whose cost is set by the Δt window
alone. Narrowing it is the obvious lever, and WSJT-X cannot take it
(it searches wide because it cannot assume a clock) while a
UTC-anchored receiver can. This section is the price.

**Window arithmetic.** `i0` counts downsampled samples at
`ds_rate = 12 000/NDOWN = 666.67 Hz`, and `dt = i0/ds_rate −
TX_START_OFFSET_S`, so `dt = 0` sits at `i0 = 333` and production's
`[-344, 1012]` is **±1.0 s**, not a full slot. Coarse cost is
`9 × ceil(n_i0/4)` cells plus a fixed 99-cell fine pass.

### On the real off-air golden

`ft4_diag_sync_window_recall` (`tests/ft4_wsjtx_samples.rs`),
`000000_000002.wav`, 31 coarse candidates, `DecodeDepth::EMBEDDED`.
The control asserts that the harness at the production window decodes
exactly what `process_candidate_basic` does, so the table measures the
window and not the harness. "search ms" is measured host wall-clock of
`ft4_sync_search_window` alone, summed over all 31 candidates.

| half-width | i0 window | cells | predicted | measured | decodes | first loss |
|---:|---|---:|---:|---:|---:|---|
| ±1.000 s | [-344, 1012] | 3159 | 1.00× | 76.6 ms | 11 | — |
| ±0.750 s | [-167, 833] | 2358 | 1.34× | 1.35× | 11 | — |
| **±0.500 s** | **[0, 667]** | **1602** | **1.97×** | **1.91×** | **11** | **—** |
| ±0.375 s | [83, 583] | 1233 | 2.56× | 2.40× | 10 | `N1TRK KB7RUQ RR73` (dt −0.44) |
| ±0.300 s | [133, 533] | 1008 | 3.13× | 2.92× | 9 | + `W7BOB KJ7G RR73` (dt −0.36) |
| ±0.250 s | [167, 500] | 855 | 3.69× | 3.43× | 8 | + `VE3LON K7RL R 549 WA` (dt +0.29) |
| ±0.200 s | [200, 467] | 702 | 4.50× | 4.18× | 4 | |
| ±0.150 s | [233, 433] | 558 | 5.66× | 5.24× | 3 | |
| ±0.100 s | [267, 400] | 405 | 7.80× | 7.16× | 2 | |

Measured speedup tracks the cell count, short by the fixed fine pass.
The 11 real signals span dt −0.44 … +0.30 s, and **±0.5 s is free**.

### Isolating the mechanism

`ft4_diag_dt_window_reach` (`tests/ft4_sweep.rs`) sweeps the *true* DT
against the window with nothing else varying. **This deliberately does
not use the tier-C corpus**: `gen_ft4_sweep_wavs.sh` fixes `DT=0.0`, so
every signal in it sits dead centre of every window and it would report
no loss at any width — a property of the fixture, not the decoder. A
separate `ft4sim` corpus was generated at DT ∈ [−0.5, +0.5] in 0.05 s
steps, 20 trials/cell, AWGN. (The grid stops at ±0.5 s because `ft4sim`
writes a 6.048 s file and the frame occupies `[0.5+DT, 5.54+DT]` —
beyond that the signal is truncated by the file rather than missed by
the window, and the two are not separable. The golden above covers the
wider tail with real captures.)

At −14 dB the result is a **hard cliff at the nominal window edge**:
100 % inside, 0 % outside, no soft shoulder. ±0.500 s holds 100 % across
the whole ±0.5 s grid; ±0.375 s holds to |DT| ≈ 0.35; ±0.250 s to
|DT| ≈ 0.25.

At −17 dB (near the −16.9 dB AWGN crossing) recall is 10–55 % as
expected — and **identical column-to-column wherever the DT is inside
the window**. Narrowing costs *reach*, not *sensitivity*: a signal the
narrow window can still see decodes exactly as often as the wide one
saw it.

### What this leaves

The largest lossless width on both instruments is **±0.5 s, worth
1.91×** on the dominant stage. Applied to §17's device numbers that is
13 225 ms → ~6.9 s, and the slot total 17 339 ms → ~11.1 s against a
1 960 ms budget: **5.7× over instead of 8.8×**. Necessary, not
sufficient — the remaining factor has to come from the arithmetic
inside the grid (`dsps_dotprod_f32_aes3`, untested) or from where `cd0`
lives (40 KB per candidate, currently a PSRAM `Vec`; `internal_pool`
already documents ~5–10× for exactly this move on FT8's scratch). Both
are device-side measurements, not host ones.

## 19. Two optimisations against §17's device number (2026-08-29)

§17 measured FT4 at 8.8× over its 1.96 s slot budget with
`ft4_sync_search` at 76 % of it; §18 measured what narrowing that
search's Δt window costs (nothing, to ±0.5 s). This section is the two
remaining levers, both applied and measured.

### 19.1 The inner product — `FlatRef` + `dot_f32` (host + device)

`ft4_sync_search_window` applied its frequency shift to `cd0` **inside**
the innermost sample loop, as a rotating phasor (`twid *= step`)
restarted at every `(df, i0)` cell. But `twid[n] = step^n` is indexed by
the offset *within the Costas block*, not by `i0` — so it is identical
across the ~340 `i0` positions each `df` sweeps, and rebuilding it per
cell was that many times redundant.

`fst4_sync_search` was already on the right side of this: its `FlatRef`
folds the shift into the reference once per `(block, df)`, leaving a
plain complex inner product that `dot_f32` — and therefore
`dotprod-extern`'s `dsps_dotprod_f32_aes3` on LX7 — can serve. FT4 now
uses the same machinery. This function's own comment had already
recorded the identity ("the same dot product as twiddling each sample
of `cd0` in place"); nothing new was derived.

**Not bit-identical**, deliberately: the products reassociate, and
`FlatRef::fill` evaluates the phasor per sample rather than
accumulating a recurrence — so it carries *less* rounding error.
Verified before landing:

| check | result |
|---|---|
| `ft4_wsjtx_samples` golden | 14/14 total, 6/6 golden, 0 phantoms — unchanged |
| `run-sensitivity-sweeps.sh ft4` | awgn/ccir_good/moderate/poor all **+0.00 dB** vs baseline, 160 trials each |
| golden stage counters | `nsync_fail`/`nsync_pass`/`osd_attempt`/`n_new` identical on all 3 SIC passes |
| merge gate | green |

**Host**, `ft4_diag_sync_window_recall`, 31 candidates:
`ft4_sync_search` **76.6 ms → 27.9 ms (2.75×)**. Golden 3-pass
`decode_loop`, median of 3: **36.5 ms → 26.6 ms (1.37×)** — diluted
because that path is rayon-parallel and the search is only part of it.

### 19.2 Where `cd0` lives — measured, and the hypothesis was wrong

§18 closed by suggesting `cd0`'s PSRAM residency as a third lever:
40 KB per candidate, ~13 MB of reads per candidate across the grid, and
`internal_pool`'s own doc comment recording ~5–10× for moving exactly
this kind of hot buffer into internal DRAM on FT8's `cs` scratch.

**Measured: 1.12×.** The arithmetic about byte counts was right; the
inference that they were the bottleneck was not. After §19.1 the access
pattern is a sequential `dot_f32` over 2 KB slices, which the S3's PSRAM
data cache serves well — the earlier code was compute-bound on its
per-sample complex multiply, not bandwidth-bound, and moving the buffer
could not fix what was not broken. Worth keeping (it is ~40 KB and it is
free once reserved at boot), but it is not the lever it looked like.

Note the production path would have to take this buffer through
`worker_arena` at boot, not allocate it on demand: with WiFi up the
largest free internal block on this board is 31 744 B.

### 19.3 Combined, on hardware

`ft4-bench`, CoreS3 @ 240 MHz, `opt-level = 3`, single core, same 31
candidates. Log: `logs/ft4-bench_opt_2026-08-29.log`.

| configuration | search | cumulative |
|---|---:|---:|
| §17 baseline | 13 225 ms | 1.00× |
| + `FlatRef`/`dot_f32` (§19.1) | 4 447 ms | **2.97×** |
| + `cd0` in internal DRAM (§19.2) | 3 937 ms | 3.36× |
| + ±0.5 s window (§18) | **2 492 ms** | **5.31×** |

Per-candidate spread stays flat throughout (pass 2: min 142.5 ms, p50
142.9 ms, max 145.5 ms) — still a fixed grid, just a cheaper one.

**Slot total, production path** (`process_candidate_basic`, which builds
its own PSRAM `cd0` at the full window, so it carries §19.1 only):
**17 339 ms → 8 642 ms, 2.01×**, still **11 distinct decodes matching
the host exactly** at both `DecodeDepth::EMBEDDED` and `FULL`.

Projecting all three onto the stage split gives 2 251 (downsample) +
2 492 (search) + 1 943 (LLR/BP) = **~6 686 ms against 1 960 ms — 3.4×
over, down from 8.8×.**

### 19.4 What is left

The search is no longer dominant. The projected split is downsample
34 % / search 37 % / LLR+BP 29 % — no single term to attack, and
`downsample_cached` (the 5120-pt inverse FFT plus the 92 160-bin
extraction and taper, per candidate) has become co-equal. That is the
stage a DDC front end removes outright rather than speeds up, which is
the `mfsk_core::ft4::ddc` work FST4 already has the template for
(`docs/notes/FST4_DDC_DESIGN.md`) — and it would also retire the
host-baked wideband FFT this bench still depends on.

## 20. The DDC front end — `downsample_cached` replaced, and it costs nothing (2026-08-30)

§19.4's handoff, taken: `mfsk_core::ft4::ddc` builds the per-candidate
`cd0` by mixing and filtering, so the 92 160-point forward FFT that the
`ft4-bench` assets still bake on a host has nothing left to feed.

**Host work only.** Nothing here has run on the board yet; §20.4 says
what is still missing before it can.

### 20.1 Why FT4 is the easy case

`fst4::ddc` needs a rational resampler (`FST4_DDC_DESIGN.md` §4.2)
because `NSPS = 3888 = 2⁴·3⁵` leaves a `3⁵` denominator no integer
decimation reaches. FT4's `NDOWN = 18` divides 12 kHz exactly:
`12 000/18 = 666.667 Hz` is already `SyncDims::ds_rate`, and
`ds_spb = NSPS/NDOWN = 32` is a power of two. So the whole module is two
`FirStage`s and two mixers — no `PolyphaseResampler`, no `RxGrid`, and
no change to anything downstream of `cd0`:

```text
12 kHz real i16
  → Mixer(f0 + 31.25 Hz)                   complex @ 12 kHz
  → FirStage A: 199 taps, fc 320 Hz, ÷18   complex @ 666.667 Hz
  → FirStage B: 263 taps, fc 56 Hz,  ÷1    complex @ 666.667 Hz
  → Mixer(−31.25 Hz)                       cd0, f0 at DC
```

Sample alignment needs no trimming: `FirStage` starts its counter at
`group_delay + 1`, so each stage's output 0 is centred on its own input
0 and `cd0[0]` stays aligned with `audio[0]` — which is what lets
`ft4_sync_search`'s absolute `[-344, 1012]` window mean the same thing
on both front ends.

### 20.2 The passband is a decode parameter

`downsample_cached` keeps `[f0 − 1.5·Δf, f0 + 4.5·Δf]` =
`[−31.25, +93.75] Hz` and **zeroes the rest**. That band is asymmetric
about `f0` (the tones run upward from it), so a low-pass centred on the
candidate does not reproduce it — hence the mixer pair, centring the
*band* and rotating `f0` back to DC afterwards, which keeps every tap
real.

Getting this wrong is not a cosmetic error.
`process_candidate_basic_impl` RMS-normalises `cd0` over its whole
length (WSJT-X `ft4_decode.f90:231-232`) and `compute_llr`'s `LLR_SCALE`
is calibrated against that unit-RMS input, so noise admitted outside the
reference band rescales every LLR feeding BP. Passing the full ±333 Hz
baseband would have put the RMS ~2.3× high.

Measured, as equivalent noise bandwidth — the one number the
normalisation sees, with each path's arbitrary gain divided out by its
own tone response (`ft4::ddc::tests::noise_bandwidth_matches_the_
reference_band`):

| path | noise/tone power ratio |
|---|---:|
| `downsample_cached` | 2.150e-3 |
| `ft4::ddc` | 2.161e-3 |
| difference | **+0.021 dB** |

The reference's 101-bin raised-cosine taper is 13.0 Hz at
`12 000/92 160` Hz per bin — flat to ±49.5 Hz, zero at ±62.5. Stage B is
flat to ~49 Hz and null by ~63. The match is by construction, not by
tuning.

### 20.3 Equivalence, on the golden and across the crossing

**WSJT-X golden** (`ft4_ddc_equivalence::ft4_ddc_baseband_decodes_the_
golden_like_the_fft_path`), same 31 candidates from `ft4_coarse_sync`,
everything downstream held identical:

| depth | FFT path | DDC path | max Δfreq | max Δi0 |
|---|---:|---:|---:|---:|
| `EMBEDDED` | 11 distinct | 11 distinct, same set | 1.00 Hz (1 of 11) | 0 |
| `FULL` | 11 distinct | 11 distinct, same set | 1.00 Hz (1 of 11) | 0 |

1.00 Hz is `ft4_sync_search_window`'s own grid step (`df = idf as f32`),
i.e. the smallest disagreement expressible — one candidate sits between
two equally good cells. Ten of eleven agree exactly, and the sync
position never moves at all.

**Tier-C paired sweep** (`ft4_ddc_recall_matches_the_fft_path_across_
the_crossing`, `#[ignore]`), 4 channels × 7 SNR tags × 20 trials = 560
files straddling every channel's 50% crossing. Paired: both arms decode
the same file from the same candidate list, so the noise realisation
cancels and 20 trials per cell are enough to resolve a real difference.

| | decodes of 560 |
|---|---:|
| FFT front end | 237 |
| DDC front end | **238** |
| disagreements | 5 (3 for the DDC, 2 against) |

Every disagreement is in a cell already sitting on its own crossing.
**The front-end swap costs 0.0 dB.**

### 20.4 What this does *not* yet do

- **Not measured on hardware.** The projected per-candidate cost is
  `5000 × 199` + `5120 × 263` ≈ 2.3 M complex MACs against the 2 251 ms
  / 31 candidates ≈ 72 ms that `downsample_cached` measured on the
  CoreS3 (§19.3). That is arithmetic, and §19.2 is this file's own
  reminder of what arithmetic about cost is worth before a measurement:
  the last hypothesis of that shape predicted 5–10× and delivered 1.12×.
- **No esp-dsp FIR backend.** `FirStage::push_block` exists
  (`FST4_DDC_DESIGN.md` §4.5) but nothing binds `dsps_fird_f32_aes3`
  yet, so a device run today would use the scalar path.
- **The coarse stage now has a kernel, and no device run has used
  it.** `ft4_coarse_sync`'s `NFFT1 = 2304` (= 256·9) was the last
  non-power-of-two length; `engine::dsp::fft_mixed_2304` serves it the
  same way `fft_mixed_5120` serves the per-candidate inverse (256 × 9
  Cooley-Tukey, the 9-point factor a further 3 × 3 over `fft_15::
  fft_3`), and `EspDspPlanner` wires both directions. The bench still
  reads its baked candidate list, so nothing has exercised it on
  hardware. Host cost of the stage is 0.3 ms; on the S3 the ~152
  transforms per slot are the number to measure.
- **Not wired into `decode_frame`.** Same choice `fst4::ddc` made: a
  library building block callers reach for, not a feature flag that
  silently swaps the host's front end.

## 21. The candidate budget — the search parameter was never WSJT-X's (2026-08-30)

Every stage after `ft4_coarse_sync` is per-candidate: the Δt search, the
LLR ladder, BP, OSD. The candidate count is therefore not one lever
among several, it multiplies all of them — and nothing had ever measured
it. `tests/ft4_candidate_budget.rs` does.

### 21.1 `sync_min` below 1.0 is not a threshold at all

`getcandidates4.f90` divides the smoothed spectrum by a fitted baseline
(`ft4_baseline.f90`, ported at `engine::baseline::fit_baseline`), which
puts **noise at ~1.0 by construction**. Any `sync_min` below that admits
every peak in the band. WSJT-X passes `syncmin = 1.2`
(`ft4_decode.f90:195`); this crate's bench had been passing **0.05** and
its sweep harness **0.8**.

Measured — one pass per file, since `sync_min` only filters and never
reorders:

| `sync_min` | mean candidates, 560 sweep files | golden, 14 signals | decodes (of 237) |
|---:|---:|---:|---:|
| 0.05 | 67.1 | 31 | 237 |
| 0.80 | 67.1 | 31 | 237 |
| 1.00 | 54.8 | 28 | 237 |
| 1.10 | 12.1 | 12 | 237 |
| **1.20** (upstream) | **1.6** | **12** | **237** |
| 1.30 | 1.0 | 12 | 237 |
| 1.40 | 0.8 | 12 | 235 |
| 1.50 | 0.6 | 12 | 226 |
| 1.70 | 0.4 | 11 | 174 |
| 2.00 | 0.1 | 11 | 77 |

The sweep column is 4 channels × 7 SNR tags × 20 trials straddling every
channel's 50% crossing — i.e. exactly where a threshold would bite if it
were going to. It does not, until 1.4.

So the faithful value is also **2.6× cheaper on a crowded real recording
and 42× cheaper on a sparse one, at zero measured recall cost on
either**. This is not a tuning knob to push further: the knee is at 1.4.

### 21.2 `max_cand` is bounded by the golden, and the sweep cannot see it

On the WSJT-X golden the eleven single-pass decodes come from ranks 0-11
of 31, so `max_cand = 12` reproduces all of them:

```text
rank  0   1147.7 Hz  score 1067.27   KB0VHA KA1YQC R 539 MA
rank  1   2066.6 Hz  score  104.29   VE3LON K7RL R 549 WA
...
rank 11   2412.8 Hz  score    1.55   W7BOB KJ7G RR73
```

On the sweep corpus **every** decode is at rank 0 — which is a property
of the fixture (one signal per file, so it is always the strongest peak),
not a finding about ranking. The same trap `gen_ft4_sweep_wavs.sh`'s
fixed `DT=0.0` set for the §18 window question. `max_cand` is therefore
bounded only by the golden's 12, and the honest instrument for a weak
signal in a *crowded* band does not exist yet — injecting a swept-SNR
signal into the real golden recording would build one.

### 21.3 What changed

`ft4_wsjtx_samples::bench_assets::SYNC_MIN` is now 1.2, and
`embedded-poc/assets/ft4_golden_candidates.bin` was re-baked: **31
candidates → 12**, same 11 decodes at both `DecodeDepth::EMBEDDED` and
`FULL`. Every device number in sections 17-19 was measured over 31
candidates; per-candidate figures carry over unchanged, slot totals do
not.

Projected slot, combining this with §20's DDC (which removes the
2 251 ms downsample stage outright):

```text
(2 492 search + 1 943 LLR/BP) × 12/31 ≈ 1 717 ms   vs 1 960 ms budget
```

**On paper that is inside the budget for the first time.** On paper is
the operative phrase: neither the DDC nor the smaller candidate list has
run on the board, and this file's own §19.2 is the standing reminder of
what a projection is worth before a measurement.

### 21.4 Aside: OSD does earn its keep, just not on the golden

The same pass measured both depths. On the golden, `EMBEDDED` and `FULL`
decode identically — which is where the bench's "OSD buys nothing" line
came from. On the 560 weak sweep files:

| depth | decodes of 560 |
|---|---:|
| `DecodeDepth::FULL` | 237 |
| `DecodeDepth::EMBEDDED` | 179 |

So the ship config gives up **58 decodes, a quarter of its recall**, at
the crossing. The golden's signals are simply strong enough not to need
OSD. Whether an embedded FT4 receiver should pay for OSD is now a real
question rather than a settled one, and it belongs with the budget above:
the candidate-count saving is roughly the size of the OSD cost.

## 22. Which rungs of the ladder earn their cost (2026-08-30)

§21 halved the candidate count; this section takes the other half of the
projected slot, the 1 943 ms of LLR + BP + OSD. FT4 climbs four BP rungs
— `llra` (nsym=1), `llrb` (nsym=2), `llrc` (nsym=`LLR_NSYM_MAX`=4),
`llrd` (nsym=1 bit-normalised) — and then, if `depth.osd`, re-tries all
four through OSD at depth 2 or 3.

FST4's own ablation found its `nsym=8` rung was ~99 % of the BP cost and
that `llrd` never contributed recall. Neither statement is portable:
FT4's `nsym=4` rung enumerates 4⁴ = 256 tone hypotheses per group
against FST4's 4⁸ = 65 536.

### 22.1 Measured, on weak data

`tests/ft4_llr_ladder_ablation.rs`, 560 sweep files (4 channels × 7 SNR
tags × 20 trials) straddling every channel's 50 % crossing, one shared
front end per file, candidates oracle-filtered to the golden frequency
so candidate selection cannot contaminate a rung question:

| config | decodes | vs FULL | CPU |
|---|---:|---:|---:|
| `abcd`+OSD (= `FULL`) | 235 | +0 | 1.00× |
| **`abc`+OSD** | **235** | **+0** | **0.78×** |
| `abd`+OSD | 189 | −46 | 0.87× |
| `ab`+OSD | 189 | −46 | 0.59× |
| `a`+OSD | 132 | −103 | 0.48× |
| `abcd` (= `EMBEDDED`) | 179 | −56 | 0.10× |
| **`abc`** | **179** | **+0 vs `abcd`** | **0.08×** |
| `abd` | 136 | −99 | 0.07× |
| `ab` | 136 | −99 | 0.05× |
| `a` | 79 | −156 | 0.03× |

Self-check on the harness: its `abcd`+OSD reaches 235 against the
pipeline's own 237 and its `abcd` reaches 179 exactly. The two missing
decodes are the depth-4 Top-K OSD stage the harness does not
reimplement — which incidentally measures that stage at +2 of 237.

Three findings, in order of size:

- **`llrd` contributes nothing.** 235 → 235 with OSD, 179 → 179 without,
  and 11 → 11 distinct on the real golden. It costs 22 % of the ladder
  with OSD and 20 % without. Its own LLR is free (`compute_llr_fast`
  returns it alongside `llra`); what it costs is a BP call and an OSD
  variant.
- **`llrc` is the recall rung** — dropping it costs 46 decodes with OSD
  and 43 without. The opposite of FST4's `nsym=8`, and the reason the
  FST4 result could not simply be assumed here.
- **OSD is +56 decodes for ~10× the BP-side cost.** On hardware that
  ratio is already measured: `ft4-bench` reported 10 987 ms at `FULL`
  against 8 642 ms at `EMBEDDED` over 31 candidates, i.e. ~2 345 ms of
  OSD, which at 12 candidates is ~900 ms of a 1 960 ms budget.

### 22.2 Dropping `llrd` for FT4 is a fidelity fix, not a trade

WSJT-X's FT4 decoder has no fourth *blind* variant.
`ft4_decode.f90:341-342` builds `llrd` only for `ipass > 3`, as
`llrd = llrc` with the first 29 bits overwritten by an a-priori pattern
— it is the **AP** variant. A fourth blind `llrd = scalefac*bmetd` is
FT8's shape (`ft8c.f90:192`), which the generic ladder inherited and
applied to FT4 as well. This crate's own AP path (`msg::pipeline_ap`)
uses `llr_set.llrd` for exactly WSJT-X's purpose and is untouched.

So `process_candidate_basic_impl` now skips the blind `llrd` rung when
`P::ID == Ft4`, both in the BP staircase and in OSD's variant list.
Verified: golden 11/14 single-pass and 14/14 with SIC unchanged, zero
phantoms; the ablation above; and `run-sensitivity-sweeps.sh ft4`
**+0.00 dB on all four channels** (160 trials each).

### 22.3 A stale claim, corrected

`DecodeDepth::osd`'s doc comment said OSD was "host-only", "compiled out
of non-`fft-rustfft` builds entirely", and "a permanent architectural
boundary". None of it is in the code — neither `fec::ldpc::osd` nor the
pipeline's OSD block carries an FFT-backend `cfg` — and both the FST4
(#306) and FT4 benches have run `DecodeDepth::FULL` on an ESP32-S3 and
reported its cost. OSD on embedded is a budget decision, and after §21
it is a live one: ~900 ms for 56/560 decodes at the crossing.
