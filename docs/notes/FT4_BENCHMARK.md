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
(**Superseded — §25**: on hardware that stage is 1 288 ms, and it does
change the conclusion.)

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

### On the WSJT-X golden

(§23.5 established that this file is `ft4sim_mult` output, not an
off-air capture. It still carries a DT spread — the simulator draws
each signal's DT uniformly over ±0.5 s — which is what this section
needs; it is not evidence about on-air DT distributions.)

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
  **Measured 2026-08-30 (§25): 1 288 ms, i.e. 8.5 ms per transform and
  66 % of the whole decode budget on its own.** The "0.3 ms on host so
  it cannot matter" reasoning in this bullet and in §20 was wrong —
  whatever that host figure timed, the device/host ratio for this stage
  is nothing like the ~30x the per-candidate stages show.
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

**On paper that is inside the budget for the first time.** Two
qualifications, one of which arrived the same day:

- On paper is the operative phrase: neither the DDC nor the smaller
  candidate list has run on the board, and this file's own §19.2 is the
  standing reminder of what a projection is worth before a measurement.
- **12 candidates is a quiet-band number.** §23 measures 15-18 on
  average and up to 25 in a crowded band, which puts the same projection
  **1.2-1.7× over** budget. The line above is the light-traffic case,
  not the contest case FT4 exists for.

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

  56 decodes is not a currency the rest of this file is quoted in, so
  the per-cell recall converts it. Interpolating each channel's 50 %
  crossing from the tables in §22.4:

  | channel | `FULL` | `EMBEDDED` | OSD is worth |
  |---|---:|---:|---:|
  | awgn | −16.89 dB | −16.62 dB | 0.27 dB |
  | ccir_good | −17.42 dB | −17.00 dB | 0.42 dB |
  | ccir_moderate | −15.67 dB | −14.75 dB | 0.92 dB |
  | ccir_poor | −16.00 dB | −14.33 dB | **1.67 dB** |

  The `FULL` column reproduces `sweep-baseline.json`'s stored crossings
  (−16.89 / −17.46 / −15.71 / −16.00) to within 0.04 dB from a
  completely separate harness, which is the check that the numbers
  above mean what they say. **OSD is a fading-channel stage**: a
  quarter of a dB on AWGN, most of two dB on CCIR-poor. An embedded
  FT4 receiver that drops it to fit the budget is trading roughly
  0.3-1.7 dB depending on the path, not a uniform 25 %.

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

### 22.4 Per-cell recall behind the OSD number

Recall of 20 trials per cell, same 560 files:

```text
FULL (abcd+OSD)          m14  m15  m16  m17  m18  m19  m20
awgn                      20   20   18    9    5    1    0
ccir_good                 20   20   19   15    3    0    0
ccir_moderate             19   14    8    3    3    0    0
ccir_poor                 17   11   10    0    0    0    0

EMBEDDED (abcd)          m14  m15  m16  m17  m18  m19  m20
awgn                      20   20   18    5    2    0    0
ccir_good                 20   20   15   10    0    0    0
ccir_moderate             16    8    2    2    0    0    0
ccir_poor                 12    6    3    0    0    0    0
```

### 22.3 A stale claim, corrected

`DecodeDepth::osd`'s doc comment said OSD was "host-only", "compiled out
of non-`fft-rustfft` builds entirely", and "a permanent architectural
boundary". None of it is in the code — neither `fec::ldpc::osd` nor the
pipeline's OSD block carries an FFT-backend `cfg` — and both the FST4
(#306) and FT4 benches have run `DecodeDepth::FULL` on an ESP32-S3 and
reported its cost. OSD on embedded is a budget decision, and after §21
it is a live one: ~900 ms for 56/560 decodes at the crossing.

## 23. Band occupancy — the design point, and where the budget actually sits (2026-08-30)

§21.2 closed by naming what was missing: "the honest instrument for a
weak signal in a *crowded* band does not exist yet". It does now, and it
was already in the WSJT-X tree — `lib/ft4/ft4sim_mult.f90` lays N
signals into one slot, each at its own SNR and frequency, each at a
**random DT in ±0.5 s**, and prints the ground truth.
`scripts/build_ft4sim.sh` now links it and
`scripts/gen_ft4_mult_wavs.sh` builds a corpus from it: 5 occupancies ×
50 slots × (5 / 10 / 14 / 20 / 30) signals over 300-2600 Hz = 3 950
ground-truth signals in 45 MB, generated in ~5 s, with a `manifest.tsv`
carrying every signal's SNR, DT, frequency and message. Messages come
from upstream's own `lib/ft4/messages.txt` — a real 40 m decode log.

AWGN only — the simulator has no fading, so `ft4_sweep`'s CCIR channels
stay the instrument for that.

### 23.1 What occupancy to design for

**Not a contest**, and the honest answer is that **nobody here knows
what FT4 occupancy actually is.** An ESP32 receiver is not going to work
a saturated contest band, and recall there is a trade rather than a
requirement. What matters is an ordinary busy evening on 40 m.

The one anchor available is weaker than it looks. §23.5 shows the
WSJT-X FT4 sample is a rendering of a real 7.080 MHz decode log with
**14 signals in the 300-2700 Hz search band** — but that log is dated
`190106`, its own rows say `Rx FT8`, and FT4 did not exist publicly
until WSJT-X 2.1 on 15 July 2019 (`Release_Notes.txt`). So 14 is **FT8's
40 m occupancy rendered as FT4**, which for FT4 is an upper bound rather
than a design point: FT4's user base is a fraction of FT8's, and outside
a contest its sub-band is usually sparse.

Nothing in this repo, or in the WSJT-X tree, measures real FT4 band
occupancy. The corpus therefore *brackets* it rather than targeting a
number, and the rows to read for an embedded receiver are the sparse
ones:

- **5-10 signals** — the plausible ordinary case for FT4
- **14** — FT8 density on 40 m, i.e. a pessimistic FT4 case
- **20-30** — an upper bound with no evidence that FT4 ever reaches it

Settling this needs a capture from a radio tuned to an FT4 sub-band,
which is the same missing measurement §23.5 ends on.

| signals/slot | mean candidates | max | deepest decoding rank | rank p90 |
|---:|---:|---:|---:|---:|
| 5 | 5.3 | 8 | 6 | 4 |
| 10 | 9.2 | 13 | 10 | 7 |
| **14 — the 40 m snapshot** | **12.3** | 18 | 15 | 10 |
| 20 | 14.8 | 21 | 19 | 12 |
| 30 (contest stress) | 17.3 | 23 | 22 | 14 |

**The golden's 12 candidates is the right number after all** — it is
what a 14-signal band produces, which is what the golden is. §21.3's
projection therefore stands across the whole plausible range, and only
reaches the budget line at a density there is no evidence FT4 sees:

| band | candidates | projected slot | vs 1 960 ms |
|---|---:|---:|---|
| **5 signals — plausible FT4** | **~5** | **~700 ms** | **inside, 2.8× margin** |
| **10 signals — plausible FT4** | **~9** | **~1 180 ms** | **inside, 1.7× margin** |
| 14 (FT8 density, pessimistic) | ~12 | ~1 570 ms | inside |
| 20 | ~15 | ~1 960 ms | at the line |
| 30 (no evidence it occurs) | ~17 | ~2 220 ms | 1.1× over |

The candidate count grows far more slowly than the signal count —
sub-linearly, because `sync_min = 1.2` is a threshold on a
baseline-normalised spectrum, not a cap. Doubling the band's occupancy
from 14 to 30 costs 40 % more candidates, not 110 %.

**`max_cand` is the parameter that does not survive.** Decodes reach
rank 15 at the design point and 22 under stress, so the bound §21.2
derived from the golden (12) must not be applied as a setting — at
occupancy 20 it would truncate real signals. It stays at 100.

### 23.2 What occupancy costs is interference, not sensitivity

Recall against ground truth, single pass (the shape `dual_core` runs —
no subtract path) against the production `sic_rounds(2)`:

| signals/slot | single pass | `sic_rounds(2)` | SIC is worth |
|---:|---:|---:|---:|
| 5 | 217/250 (87 %) | 241/250 (96 %) | +9 pts |
| 10 | 371/500 (74 %) | 438/500 (88 %) | +14 pts |
| **14** | **477/700 (68 %)** | **568/700 (81 %)** | **+13 pts** |
| 20 | 565/1000 (56 %) | 721/1000 (72 %) | +16 pts |
| 30 | 626/1500 (42 %) | 888/1500 (59 %) | +17 pts |

In operator terms, at the occupancies FT4 plausibly sees: a board
without a subtract path reports about **4.4 of 5** stations, or **7.4
of 10**, where a full decoder reports 4.8 and 8.8. That is **half a
station to one and a half per slot** — and at the pessimistic
FT8-density row, 9.5 of 14 against 11.3.

Which reframes the embedded subtract question. At a saturated band it
would be indispensable; at the density FT4 plausibly runs at, it buys
roughly one station per slot for a whole extra decode pass.

The mechanism is visible in the strong end of the per-SNR table, not
the weak end:

```text
                 -17  -16  -15  -14  -13  -12  -11  -10   -8   -6   -3    0   +5  +10
single pass, 14   47%  61%  63%  70%  71%  72%  71%  66%  70%  62%  61%  70%  76% 100%
sic_rounds(2), 14 56%  66%  72%  80%  86%  82%  88%  82%  88%  76%  78%  84%  98% 100%
```

A signal at +5 dB is 22 dB above FT4's own threshold and is still missed
a quarter of the time without SIC. That is not sensitivity — it is one
signal sitting inside another's 83 Hz occupied bandwidth, the same
effect the golden's 11/14 → 14/14 records on three signals instead of
hundreds.

So the embedded trade is quantified rather than assumed, and it points
the opposite way from where §23's first draft left it: **at plausible
FT4 occupancy, no subtract path costs 9-14 points of recall — about one
station per slot — for a saving of a whole decode pass.** For a
receiver whose slot budget is 1 960 ms that is a defensible trade, not
a blocker. It becomes a blocker only at a band density FT4 has not been
shown to reach.

### 23.3 Precision, measured for the first time beyond one file

**6 phantoms of 5 118 decodes (0.12 %)** across both arms and all five
occupancies, against 3 950 ground-truth signals — about one false
decode every 50 slots at the design point and every 12 at contest
stress. FT4's `max_extra: 0` budget had only ever been checked on the
single golden file; the true rate is small but not zero, which is worth
knowing before an operator sees one.

### 23.4 What this corpus still cannot see

- **No fading.** `ft4sim_mult` adds Gaussian noise only.
- **SNR floor of −17 dB** — the simulator clamps `isnr` there, which is
  FT4's own threshold, so the deep-weak tail is out of reach.
- **DT is uniform in ±0.5 s**, which sits exactly inside §18's narrowed
  search window, so this corpus cannot test that window either.
- **It is the same generator as the golden** (§23.5), so it is not an
  independent instrument — it is a way to vary one scene parameter at a
  time around a scene upstream already chose.

### 23.5 The "real off-air golden" is a simulated scene — and upstream had no other

Worth stating plainly because several comments in this tree said
otherwise: `WSJT-X/samples/FT4/000000_000002.wav` is **`ft4sim_mult`
output**, generated from `lib/ft4/messages.txt`'s `File 2` block.

Four pieces of in-tree evidence, none of which needs an outside source:

1. The filename is the simulator's own `000000_%06d.wav` pattern. Every
   *other* protocol's sample in `WSJT-X/samples/` carries a real UTC
   capture timestamp — `FT8/210703_133430.wav`,
   `JT9/130418_1742.wav`, `WSPR/150426_0918.wav`,
   `FST4+FST4W/210115_0058.wav`, `MSK144/181211_120500.wav`. FT4 is the
   only one that does not.
2. The 19 rows of `File 2` reproduce the file exactly (below).
3. Those rows are dated `190106` and say `Rx FT8` in their own text.
4. FT4 was introduced in **WSJT-X 2.1, 15 July 2019**
   (`Release_Notes.txt`) — six months *after* that log was recorded.

So the scene is a real 40 m **FT8** band snapshot, rendered as FT4
because at the time there was no FT4 traffic to record. That is not a
criticism of upstream; it is the only thing they could have done for a
brand-new mode. But it means the widely-quoted "FT4 sample recording"
is not evidence about FT4 band occupancy, and neither this repo nor the
WSJT-X tree contains any.

```text
   297 Hz   -9 dB  N1TRK N4FKH 569 VA        2300 Hz  -13 dB  AC6BW KR9A R 559 WI
   422 Hz   -9 dB  N1TRK KB7RUQ RR73         2310 Hz   -1 dB  WD9IGY KX1X 73
   520 Hz   -9 dB  W9JA PY2APK RRR           2413 Hz  -17 dB  W7BOB KJ7G RR73
   560 Hz   -8 dB  CQ RU AB5XS EM12          2560 Hz  -12 dB  CQ RU W0FRC DM79
   727 Hz  -12 dB  NZ7P WA7JAY 589 CA        2567 Hz   -7 dB  NI6G W7DRW 569 AZ
  1148 Hz  +16 dB  KB0VHA KA1YQC R 539 MA    ─── above the 300-2700 search band ───
  1640 Hz   -3 dB  CQ RU N9OY EN43           2725 Hz   +3 dB  K4SQC VE3RX RR73
  1910 Hz  -10 dB  K1JT WB4HXE 559 GA        2813 Hz   -5 dB  CQ RU W1QA FN32
  2067 Hz   +6 dB  VE3LON K7RL R 549 WA      2995 Hz   +1 dB  CQ RU WS4WW FM17
                                             3158 Hz  +14 dB  HB9BUN KG4W R 549 VA
                                             3337 Hz   -7 dB  W9TO KN3ILZ 529 PA
```

Three consequences:

- **The file holds 19 signals, not 14.** The 14 in `FT4_FULL_REFERENCE`
  are the ones inside the searched band; the other five sit above
  2700 Hz, which is why neither `jt9 -H 2700` nor this crate ever
  reports them. "14/14 parity with jt9" is parity with a reference
  decoder over a shared band, not 100 % of what is in the file.
- **Its ground truth is exact** — SNR, DT and frequency per signal, from
  the same `messages.txt`. The reported SNRs agree with the truth to
  within a couple of dB across the 14, which is an independent check on
  `pipeline::ft4_snr_db` that nobody had noticed was available.
- **Neither this repo nor upstream has a real off-air FT4 recording.**
  Every FT4 claim about "real signals" rests on a rendering of a real
  *FT8* decode log, which carries a plausible scene (occupancy, SNR
  spread, message mix at 7.080 MHz) but none of the artefacts — drift,
  splatter, LO offsets, non-flat noise — and describes a mode with far
  more users than FT4 has. Closing that needs a capture from a radio
  tuned to an FT4 sub-band. This project has one (an IC-705 already
  feeding the CoreS3 over USB audio), which makes it the cheapest
  outstanding measurement in the whole FT4 line.

## 24. Does the host simulation stand in for the board? For FT4, yes (2026-08-30)

Real off-air FT4 is hard to capture — FT4 activity is thin enough that
upstream did not have a recording either (§23.5), which is why their own
sample is simulated. So simulation is the instrument, and the question
worth answering is not "when can we get real data" but **how much of the
device does the host reproduce**.

For FT4 the answer turns out to be: all of the arithmetic.

### 24.1 `fixed-point` is a no-op for FT4

The CoreS3 build enables `mfsk-core/fixed-point` (`cargo tree -f '{f}'`
on `m5stack-cores3-app` lists it), which reads as "the board decodes
with `SpecCell = u16` quantisation, so host f32 numbers do not
transfer". It does not, for FT4.

```sh
grep -rl 'feature = "fixed-point"' mfsk-core/src --include=*.rs
```

returns six files: `ft8/decode.rs`, `ft8/decode_block.rs`,
`ft8/decode_block/{spectrogram,coarse_sync,process_candidates}.rs`, and
`engine/fft.rs` — where the single site is `default_planner_16()`, the
i16 planner that only `decode_block` consumes. **Nothing on the generic
`engine::pipeline` path is gated by it**, and FT4 (like FST4) is
entirely on that path: `ft4_coarse_sync`'s own f32 periodogram,
`downsample_cached`, `ft4_sync_search`, `symbol_spectra`, the LLR
ladder, BP and OSD.

Measured rather than only read: `run-sensitivity-sweeps.sh ft4` under
`MFSK_SWEEP_FEATURES=full,internal-testing,fixed-point` reproduces the
f32 crossings to the digit — −16.89 / −17.46 / −15.71 / −16.00 dB, 160
trials each, `+0.00 dB` on all four channels. The golden tests pass
unchanged too. That is a fact about the feature, not a coincidence.

The board therefore runs FT4 in **the same f32 arithmetic the host
does**, and every recall number in §21-23 is the board's number as well.

### 24.2 What is left between host and board

- **The FFT kernel.** Host uses rustfft; the board uses esp-dsp's
  radix-2 asm plus this crate's `fft_mixed_5120` / `fft_mixed_2304`
  wrappers. Both f32, differing only in rounding and operation order.
  The evidence that it does not matter is already on record: `ft4-bench`
  decoded **11 distinct messages, identical to host, at both
  `DecodeDepth::EMBEDDED` and `FULL`** (§17).
- **`dotprod-extern`.** `ft4_sync_search`'s inner product goes through
  `dsps_dotprod_f32_aes3` on LX7 and a scalar loop on host. Same
  reassociation argument as the `FlatRef` change in §19.1, which was
  checked against the full sweep at +0.00 dB.
- **Everything above the arithmetic** — slot timing, audio capture,
  memory pressure — which is what a device run measures and no host
  test can.

### 24.3 The FT4 evidence base, as it stands

| question | instrument | status |
|---|---|---|
| sensitivity curve, 4 channels | `ft4sim` sweep, 160 trials/channel | measured, baseline-tracked |
| Δt reach | `ft4sim` DT sweep (§18) | measured |
| band occupancy / interference / precision | `ft4sim_mult` (§23) | measured, 3 950 signals |
| numeric path host vs board | feature audit + fixed-point sweep (§24.1) | **identical for FT4** |
| FFT kernel host vs board | `ft4-bench` on the golden (§17) | identical decodes |
| slot timing on hardware | `ft4-bench` | measured end-to-end, 12 candidates + DDC (§25): **2.33x over budget** |
| real receiver artefacts | — | **no instrument, here or upstream** |
| true FT4 band occupancy | — | **no instrument, here or upstream** |

The last two rows are not work items that a better simulator closes.
They are the honest boundary of what this line can claim, and the
corpus is built to bracket them rather than to pretend they are
answered.

## 25. The whole slot on hardware — three projections, all wrong (2026-08-30)

`ft4-bench` ran an FT4 slot end to end on a CoreS3 for the first time:
`ft4_coarse_sync` computed on-device through `fft_mixed_2304`, the
per-candidate baseband built by `ft4::ddc`, no baked asset in the path
but the 90 000-sample audio. Log:
`m5stack-cores3-app/logs/ft4-bench_wholeslot_2026-08-30.log`.
240 MHz, `opt-level = 3`, single core, 12 candidates.

### 25.1 What was right

**Decodes: 11 in all five arms**, identical to host, at every depth and
both windows. The one difference against the FFT arm is `K1JT WB4HXE`
at 1910.6 Hz against 1909.6 Hz — the single 1 Hz search-grid step that
`ft4_ddc_equivalence` already pins as 1-of-11 on host.

**The coarse kernel is exact.** `compare_candidates` paired all 12
device candidates against the host-baked list with **max Δfreq 0.00 Hz
and max Δscore 0.00 %**. Two different factorisations of a 2304-point
transform — rustfft's planner on host, 256 × 9 over esp-dsp's
`dsps_fft2r_fc32_aes3_` here — and the peak selection does not move at
all. Bit-equality was never on offer; this is better than the section
that predicted "the same peaks clear `SYNC_MIN` in the same places"
had any right to expect.

**The wideband cache is genuinely gone.** The DDC arms pass an empty
`fft_cache` and decode the same 11. What `ft4_ddc_arm_never_reads_the_
wideband_cache` asserts on host holds on the board.

### 25.2 What was wrong — the budget

| arm | coarse | candidates | slot | vs 1 960 ms |
|---|---:|---:|---:|---:|
| FFT EMBEDDED | 1 288 | 2 839 | 4 127 | 2.11× |
| FFT FULL | 1 288 | 2 831 | 4 119 | 2.10× |
| DDC EMBEDDED (±1.0 s) | 1 288 | 3 856 | 5 144 | 2.62× |
| DDC FULL (±1.0 s) | 1 288 | 3 853 | 5 141 | 2.62× |
| **DDC EMBEDDED (±0.5 s, ship)** | **1 288** | **3 287** | **4 576** | **2.33×** |

§21.3 projected `(2492 + 1943) × 12/31 ≈ 1717 ms` and called it "the
first time inside the budget on paper". The measured ship configuration
is **4 576 ms — 2.7× that projection**. Three independent errors, none
of them in the arithmetic:

**1. The coarse stage was never in the projection.** It was baked, and
this file (§20) and the bench's own doc dismissed it as "0.3 ms on host,
so its absence understates the device total by roughly that times the
device/host ratio — small". It is **1 288 ms**: 28 % of the slot, and
66 % of the entire budget on its own. 152 transforms per slot at ~8.5 ms
each. Whatever the host figure was measuring, the device/host ratio for
this stage is nothing like the ~30× the per-candidate stages show, so
the extrapolation that licensed ignoring it was unfounded. This is
[`feedback_bottleneck_hypothesis_measure_first`] a third time: a stage
excluded by an argument rather than a measurement.

**2. The DDC is 2.3× *slower* than the FFT front end here, not faster.**
`candidate_baseband` costs 154 006 µs/cand against `downsample_cached`'s
66 974 — **+1 044 ms over the slot**. §20 measured the DDC's *fidelity*
(0.0 dB, same decodes) and its host cost, and the embedded case for it
was that it removes a transform the board cannot run. That case still
stands exactly as stated — it is what makes a receiver possible at all —
but it is a **feasibility** win, not a speed one, and §21.3's projection
quietly treated the 2 251 ms `downsample_cached` line as recovered. It
is not: the DDC replaces it with something more expensive. 199 + 263
scalar f32 taps over 90 000 samples per candidate is the reason, and
`dsps_fird_f32_aes3` is the obvious unexploited lever.

**3. Narrowing is worth less than §19 implied.** 1.58× on the search
alone (1 547 → 960 ms) reproduces §18-19, but the search is now 21 % of
the slot rather than most of it, so the ship arm gains only 569 ms —
1.17× on the total.

Also settled: **internal-DRAM `cd0` placement is dead**, 1.01× here
against the 1.12× of §19.2 — no longer worth the 40 KB reservation a
production mode would have to make. And **OSD is free on a clean
candidate list**: `FULL` and `EMBEDDED` differ by 8 ms and 3 ms on the
two front ends. §22 put OSD at ~2 345 ms over 31 candidates, but that
was 19 noise candidates each running the full ladder and failing into
OSD. At `sync_min = 1.2` almost every candidate is a real signal that
BP decodes, and OSD never runs. **OSD's cost scales with failures, not
with candidates** — which means the §21 "does embedded keep OSD" question
is much cheaper than the ladder ablation made it look.

### 25.3 Where the 4 576 ms actually is

| stage | ms | share |
|---|---:|---:|
| `ft4_coarse_sync` | 1 288 | 28 % |
| `candidate_baseband` (DDC) | ~1 848 | 40 % |
| `ft4_sync_search` @ ±0.5 s | ~960 | 21 % |
| LLR + BP | ~479 | 10 % |

Search and LLR/BP together are 1 439 ms and fit the 1 960 ms budget with
room. The whole overrun is the two stages that had never been measured
on hardware. Both have an untried lever — esp-dsp FIR for the DDC, and
for the coarse stage the question of why 152 transforms cost 8.5 ms each
when their 256-point inner passes are already esp-dsp's — so 2.33× is a
starting point, not a verdict. Nothing here changes §23's conclusion
that the *candidate count* is well behaved at realistic occupancy; it
changes which stages the remaining work is in.

### 25.4 It is not the PIE staging copy (2026-08-30)

The cheapest of the three suspects behind the coarse stage's 8.5 ms per
transform was that none of its inner rows reach esp-dsp's in-place PIE
path: `MixedRadix2304Fft::process` copies through `AlignedStaging`
whenever the caller's buffer is not 16-byte aligned, and
`symbol_spectra_avg` hands it a plain `vec![Complex::new(0.0, 0.0);
NFFT1]`, whose *guaranteed* alignment is `align_of::<Complex32>() = 4`.
Whether esp-idf's allocator returns 16 anyway is not a question host
code can answer.

`pie_alignment_report()` already existed for issue #260 but **only the
plain radix-2 path ever called `record_pie_path`** — the three
mixed-radix kernels, i.e. every transform FT4 and FT8 actually run on
this board, were invisible to it. Wiring them up and reading the
counters either side of each pass (log:
`ft4-bench_piealign_2026-08-30.log`):

| pass | inner rows | in-place | staged |
|---|---:|---:|---:|
| pass 0 — coarse, 2304 = 256 × 9 | 1 368 | **1 368** | **0** |
| pass 1f — `downsample_cached`, 5120 = 1024 × 5 | 60 | 0 | **60** |
| pass 1d — DDC (FIR only) | 0 | 0 | 0 |

1 368 = 152 × 9 and 60 = 12 × 5, so the counters see exactly the rows
they should; pass 1d's zero is the self-check that each pass runs one
kernel and only one.

**The hypothesis is dead for the coarse stage**: every one of its 1 368
inner rows already runs in place on the PIE assembly. The 1 288 ms is
therefore in the *wrapper*, not the kernel — the Nuttall window over
2 304 points, the strided gather/scatter of nine 256-point rows, the
2 304 twiddle multiplies, and the magnitude accumulation over 1 152
bins, all scalar Rust. Nine 256-point esp-dsp transforms are on the
order of 0.2 ms of the 8.5. **The next probe times the combine stage
directly**; there is no point optimising a kernel that is already ~2 %
of the cost.

**And it turned up a separate defect**: `fft_mixed_5120` stages **100 %**
of its rows. Its 5 120-element buffer lands 8-mod-16, so every
1 024-point row pays a copy in and a copy out. That is real, with a
known fix (align the buffer), but note what it is *not*:
`downsample_cached` is the control arm here. The shipping receiver uses
the DDC and never calls it, so fixing this does not move the budget — it
is recorded because it is true, not because it is on the path. Left
unfixed pending evidence it costs anything: 60 rows of 8 KB is ~1 MB of
copying against an 803 ms stage, so the same "it is the wrapper, not the
kernel" conclusion probably applies here too.

### 25.5 The layer split — it is the combine stage (2026-08-30)

§25.4 reached "the wrapper" by elimination, which is not a measurement,
and "the wrapper" is three different things. The mixed-radix kernels now
time them separately — whole `Fft::process`, esp-dsp's assembly inside
it, and the staging copies — so combine = process − kernel − staging,
and whatever the caller spends around the transform is its own
wall-clock minus process. Two `esp_timer_get_time` reads per inner row;
under 1 ms against 1 290. Log:
`ft4-bench_layersplit_2026-08-30.log`.

| stage | total | kernel | staging | **combine** | outside transform |
|---|---:|---:|---:|---:|---:|
| pass 0 coarse (2304) | 1 290 ms | 201 (15 %) | **0** | **853 (66 %)** | 236 (18 %) |
| pass 1f `downsample_cached` (5120) | 803 ms | 29 (3 %) | 34 (4 %) | **508 (63 %)** | 231 (28 %) |
| pass 1d DDC (FIR) | 1 848 ms | 0 | 0 | 0 | 1 848 (100 %) |

**The answer is the combine stage: 66 % of the coarse stage and 63 % of
`downsample_cached`.** That is `fft_2304_with` / `fft_5120_with`'s
Cooley-Tukey recombination — the twiddle multiplies and the strided
gather/scatter between inner rows — scalar Rust in
`mfsk_core::engine::dsp::fft_mixed_*`. 853 ms and 508 ms, 1 361 ms
across the two stages, against a 1 960 ms budget.

Its throughput is the tell: the 2304 combine is ~5.6 ms per transform
for roughly 30 k flops, i.e. **~5 Mflop/s on a 240 MHz FPU**. Nothing
about the arithmetic is expensive; the access pattern is. This is the
shape [`reference_fft_loop_interleave_trick`] already addressed once on
this silicon (−40 % on serial DSP recursions by interleaving loops to
give the FPU independent chains).

Three corrections this forces on §25.4, which guessed rather than
measured:

- **the kernel is 15 %, not "~2 %"**. 201 ms over 1 368 rows is 147 µs
  per 256-point row — reasonable once `dsps_bit_rev_fc32_ansi`, which
  is C and not assembly, is counted in it, but four times my estimate.
  Guessing the number it had just built the instrument to measure was
  not defensible.
- **the 5120 staging defect is real and negligible**: 34 ms of 803, 4 %.
  It was worth confirming and is not worth fixing — and it is on the
  control arm the shipping receiver never calls, so the honest value of
  fixing it is zero.
- **`outside transform` is 18-28 %** and was not on the suspect list at
  all: the Nuttall window plus magnitude accumulation for coarse
  (236 ms), the 92 160-bin extraction for `downsample_cached` (231 ms).

pass 1d's 100 %-outside row is the self-check: the DDC runs no
transform, so every microsecond of its 1 848 ms is FIR, and
`dsps_fird_f32_aes3` remains its untouched lever.

**Where FT4 embedded now stands.** Two levers, both measured rather than
projected: the mixed-radix combine (1 361 ms, and shared with
`fft_mixed_3840`, so FT8 on this board is paying the same tax) and the
DDC's scalar FIR (1 848 ms). Neither has been attempted.

## 26. The combine stage: two wrong theories and a 41 % win (2026-08-30)

§25.5 put 66 % of `ft4_coarse_sync`'s 1 290 ms in the Cooley-Tukey
combine — `fft_mixed_2304`'s transposes, twiddles and 9-point columns,
scalar Rust around esp-dsp's 256-point rows. This section is what
happened when that was attacked. Two of the three attempts failed, and
the order matters more than the result.

Only `embedded-shared`'s planner and the modules' own unit tests call
`fft_*_with`; the host uses rustfft directly for 2304 and 5120. So this
is embedded-only code, and every change below was additionally pinned
**bit-identical** on host before flashing — the decodes cannot move, and
they did not: 11 in every arm of every run, and `compare_candidates`
paired 12 of 12 at max Δfreq 0.00 Hz throughout.

| step | coarse | combine | ship slot | |
|---|---:|---:|---:|---|
| §25 baseline | 1 290 | 853 | 4 576 (2.33×) | |
| blocked transposes | 1 496 | 1 033 | 4 790 (2.44×) | **worse — reverted** |
| scratch hoisted out of the call | 1 165 | 774 | 4 460 (2.27×) | |
| …and 16-byte aligned | 1 138 | 777 | 4 434 (2.26×) | |
| **…in internal DRAM** | **758** | **404** | **4 055 (2.06×)** | **−41 % coarse** |

### 26.1 The blocking attempt, which was wrong

The theory: `m[n2·256 + n1]` and the column gather touch ~6 900
addresses 2 048 bytes apart per transform, and an 18 KB scratch is past
`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL` so it lives in Quad PSRAM. At
~32 bytes a line that is >200 KB of bus traffic to move 18 KB — 5.1 ms
at 40 MB/s, against a measured 5.6 ms. The arithmetic landed on the
right number.

Blocking both transposes into stack-staged 32-column strips made it
**21 % worse** (853 → 1 033 ms). The arithmetic was right about the
quantity and wrong about the mechanism: the strided access was already
being served, so the blocking bought nothing and its extra staged copy
each way was pure cost. Reverted; the surviving artefact is the
`fft_2304_with_scratch` seam it needed, and a `#[test]` pinning
bit-identity, which is what made reverting cheap.

Third time this line has produced an "arithmetic that predicted the
right magnitude for the wrong reason" — see also
`feedback_bottleneck_hypothesis_measure_first`. The lesson is
apparently not that the estimates are bad but that a matching estimate
is not evidence.

### 26.2 The allocation, which was a tenth of it

`vec![Complex32::new(0.0, 0.0); 2304]` per call is 152 mallocs and 152
zero-fills of 18 KB per slot, for a buffer step 1 fully overwrites.
Hoisting it into the planner: 1 290 → 1 165 ms.

It also *reintroduced* the §25.4 staging tax — the hoisted `Vec` landed
8-mod-16, so 29 ms of copies appeared where the per-call one had had
none. Allocating it through `AlignedStaging` instead took it back to
zero and to 1 138 ms. Worth recording as a hazard: the alignment of a
buffer is a property of how it was allocated, and moving an allocation
changes it silently.

### 26.3 It was the working set

`symbol_spectra_avg` keeps three 18 KB buffers live per transform — its
input, the twiddle table, and the scratch — about 54 KB against an S3
data cache of ~32 KB. That is why improving locality *inside* one
buffer could not help: the set does not fit, so each pass streams from
PSRAM regardless of the order it walks.

Taking one of the three out: `heap_caps_aligned_alloc(16, 18 432,
MALLOC_CAP_INTERNAL)` for the scratch, falling back to the PSRAM path
when it fails. **Combine 777 → 404 ms, coarse 1 138 → 758 ms.**
Cumulatively against §25: coarse −41 %, combine −53 %, and the ship slot
2.33× → 2.06× of budget.

This is the same effect `internal_pool` documents for FT8's `cs` scratch
and that §19.2 measured at only 1.12× for `cd0` — the difference being
what the buffer is *for*. `cd0` is streamed once per grid cell, which a
cache serves happily; this one is traversed five times per transform
with a stride, which it does not.

**The cost is 18 KB of internal DRAM**, and the receiver needs exactly
one: the shipping path is coarse + DDC, and `fft_mixed_5120` — still at
508 ms, untouched in every run above, which is what makes it the control
that shows the change was isolated — belongs to `downsample_cached` on
the control arm. With WiFi up the largest free internal block on this
board is 31 744 B, so a production FT4 mode must reserve this through
`worker_arena` at boot, not allocate it on demand.

### 26.4 Where this leaves the budget

Ship slot 4 055 ms against 1 960. Remaining, in order:

| stage | ms |
|---|---:|
| DDC (`candidate_baseband`, scalar FIR) | ~1 848 |
| `ft4_sync_search` @ ±0.5 s | ~960 |
| coarse (combine 404 + kernel 144 + window/magnitude 208) | 758 |
| LLR + BP | ~479 |

The DDC is now the largest single item and `dsps_fird_f32_aes3` is still
untried. `fft_mixed_3840` shares the structure fixed here, so FT8's
`NFFT_SPEC` path on this board is presumably paying the same PSRAM tax —
not measured, and stated as a hypothesis this time.

## 27. The DDC: the premise was wrong, and the fix was 10 % (2026-08-30)

§25.5 listed the DDC's lever as "bind esp-dsp FIR
(`dsps_fird_f32_aes3`), untried". Measuring before writing anything
showed the premise was already false: `FirStage::dot` has routed through
`dotprod::dot_f32` since issue #307, and under `dotprod-extern` that
**is** `dsps_dotprod_f32_aes3`. The FIR has been on esp-dsp the whole
time.

What it was not on is that kernel's PIE path, which needs both operands
16-byte aligned and `n % 4 == 0`. `ft4-bench`'s new `dotprod_probe`, on
internal-DRAM buffers so alignment is the only variable:

| n | aligned | offset by one f32 |
|---:|---:|---:|
| 196 | **7 936 ps/tap** | 18 104 |
| 199 | 18 082 | 18 081 |
| 200 | **7 900** | 18 073 |
| 204 | **7 869** | 18 046 |
| 260 | **7 520** | 17 751 |
| 263 | 17 740 | 17 739 |
| 264 | **7 501** | 17 736 |

2.3×, and `ft4::ddc`'s stages are **199 and 263 taps** — so no call has
ever qualified on length, whatever the alignment. Dots are ~81 ms of the
DDC's 154 ms per candidate.

**Attempt 1, rejected: stage each window into an aligned padded buffer.**
Satisfies both preconditions on every call. Measured **1 848 → 1 937 ms**
— the copy costs ~2.7 µs against ~2.0 µs of saved dot. Reverted.

**Attempt 2, kept: pad the taps, align the history, copy nothing.** Taps
zero-padded to a multiple of four (the added terms are exactly `0.0·x`),
and the history moved off `Vec<f32>` — only 4-byte aligned, so even a
four-multiple index would not have been 16-byte aligned — onto a
`repr(align(16))` store. The window's alignment still cycles with
`win_start`, so this wins on the fraction that lands right: half of stage
A (`decim = 18`), a quarter of stage B (`decim = 1`).

**1 848 → 1 659 ms, −10.2 %.** Ship slot 4 055 → 3 876 ms, 2.06× →
1.97×. Predicted ~200 ms beforehand and measured 189 — worth recording
as the estimate that held, after §26.1's did not.

Host keeps its exact arithmetic: the padded dot is behind
`dotprod-extern`, and the alignment change moves placement, not values.

A bug on the way, since the shape recurs: sizing the history to the
*padded* tap count let `win_start + pad` run one past the end at the
last legal window, because `compact` triggered on the buffer length.
Boot loop, `range end index 777 out of range for slice of length 776`.
The trigger is now an explicit cap and the allocation carries the
padding as slack beyond it.

### 27.1 Attempt 3: four pre-shifted tap tables (2026-08-30)

The other half of those dots needed the window aligned on *every* call,
not the fraction that happens to land right. `esp_dsp_dotprod`'s own
module doc had already weighed the way to do that — one tap table per
window phase, so phase `p` carries `p` leading zeros and the dot starts
at `win_start - p`, which is 4-aligned — and rejected it: **168 KB** for
FST4's wideband coarse cascade (L = 64) against ~190 KB of free internal
DRAM.

That arithmetic does not carry to FT4. These stages are 199 and 263
taps, so four phases is **about 7.4 KB for both**. The rejection was
sound for the cascade it was written about and simply does not apply
here — worth noting, because the note read as a general verdict on the
technique.

  DDC 1 659 -> **1 306 ms** (-21 %, and -29 % against the 1 848 ms §25
  measured). FIR stages 122 -> 94 ms, stage B alone 43 -> 23 ms — the
  biggest single move, since `decim = 1` meant only a quarter of its
  windows had been landing aligned. Ship slot 3 876 -> **3 516 ms,
  1.97x -> 1.79x**.

Predicted ~1 300 ms beforehand, measured 1 306.

Every added term is exactly `0.0 · x`, so only the rounding of the sum
moves; decodes stayed at 11 in every arm. Host is unaffected — the
phase tables are behind `dotprod-extern`, and a build with no backend
keeps the single reversed-tap dot it always had. Tier A+B green (82
binaries) and the `wspr-ddc-cascade` golden green.

`dsps_fird_f32_aes3` proper — which would take the whole block and own
the sliding window — remains untried, and is now a smaller prize than
it looked: the dots it would absorb are already on the fast path.

### 27.2 Where FT4 stands after the day

| stage | §25 | now |
|---|---:|---:|
| `ft4_coarse_sync` | 1 288 | 758 |
| DDC front end | 1 848 | 1 306 |
| `ft4_sync_search` @ ±0.5 s | ~960 | ~960 |
| LLR + BP | ~479 | ~479 |
| **ship slot** | **4 576 (2.33x)** | **3 516 (1.79x)** |

The two stages nobody had measured are now the two that moved. The Δt
search is untouched and is next by size.

## 28. FT8 was paying the same tax (2026-08-30)

§26 fixed `fft_mixed_2304` and noted that `fft_mixed_3840` — FT8's
`NFFT_SPEC` kernel — has the identical structure and is "presumably
paying the same PSRAM tax", explicitly flagged as an unmeasured
hypothesis. It is now measured, and as an A/B **inside one run** rather
than across two flashes, because the two entry points differ in exactly
one thing:

- `fft_3840_with` allocates `vec![…; 3840]` per call — 30 KB, PSRAM,
  and what every FT8 decode on this board has used;
- the planner's `Fft::process` hands `fft_3840_with_scratch` a hoisted
  internal-DRAM buffer.

Same inner 256-point kernel, same twiddles, same arithmetic.

```
fft_mixed_3840 x200 — per-call vec (PSRAM) 15 808 µs/xform
                    | hoisted internal      6 478 µs/xform
                    | 2.44x | max |A-B| 0e0
```

**2.44×**, and the outputs are bit-for-bit equal, which is the check
that it is a placement change and nothing else.

Two caveats, both important:

- **This is the transform, not an FT8 slot.** How much of a slot it is
  worth has not been measured — `ft4-bench` is an FT4 harness and there
  is no FT8 bench on this board. Do not quote 2.44× as an FT8 decode
  speedup.
- **30 KB of internal DRAM against a 31 744 B largest free block once
  WiFi is up.** It succeeded here because this bench brings up neither
  WiFi nor USB host. In the controller it will very likely fail and fall
  back to PSRAM — silently, apart from the log line. FT8 getting this
  win in the real app means reserving the buffer through `worker_arena`
  at boot, the way `internal_pool` already does for `cs`, not allocating
  it on demand.

## 29. The coherent scorer was 78 % off the fast path (2026-08-30)

§27 fixed `ft4::ddc`'s FIR on the strength of a micro-benchmark.
Counting the *real* `dot_f32` call sites — new counters in the
`dotprod-extern` backend, split by which precondition failed — showed
where that had and had not landed:

| call site | calls / slot | PIE | slow: alignment | slow: length |
|---|---:|---:|---:|---:|
| front ends (FIR) | 248 904 | **100 %** | 0 | 0 |
| `ft4_sync_search` (±1.0 s) | 284 688 | **22 %** | 219 456 | 0 |

The FIR work landed completely. `engine::sync2d`'s coherent scorer —
`ft4_sync_search`'s entire inner loop, and `fst4_sync_search`'s — was
78 % on the scalar path, **purely on alignment**. Its length was never
the problem: a Costas block is `nsym · ds_spb · 2` f32, 256 for FT4.

Two operands, both failing:

- `FlatRef::plain` / `swapped` were plain `Vec<f32>`, guaranteed only
  4-byte aligned;
- `c` is `cd0[s0..]` viewed as `f32`, so its byte offset is `s0 · 8` —
  16-byte aligned only when `s0` is even, and `s0` sweeps the grid.

**And here the copy that lost in §27 wins.** The FIR's staging copy was
per dot; this one would be per candidate, amortised over ~25 000 dots.
In the event it was not needed at all: aligning `FlatRef`'s buffers and
adding a **zero-led odd-phase pair** — read from `s0 - 1`, which is
aligned, against a reference whose first complex sample is zero — takes
both operands to aligned without copying anything. Two phases rather
than the FIR's four, because the unit here is a complex sample.

```
pass2 ft4_sync_search dot_f32: 284 688 calls | 284 688 PIE (100 %)
search ±1.0 s  1 593 -> 1 048 ms
search ±0.5 s    960 ->   700 ms
ship slot      3 516 -> 3 288 ms, 1.79x -> 1.67x
```

`i0_sum` is unchanged across every run, so the search is selecting the
same positions; 11 decodes in every arm.

One mistake worth keeping: the first flash still showed 5 184 calls on
the scalar path, because the odd-phase reference was sliced back to
`n·2 + 2` = 258 — not a multiple of four. `AlignedF32` had already
padded it to 260 with zeros; using the padded length is both correct
and what the precondition wants. The bound is now expressed from that
padded length rather than as `n + 1`, since how much padding it adds
depends on the parity of `n`, which differs between FT4 and FST4.

Host is untouched — the odd phase is behind `dotprod-extern` and a
build with no backend keeps the single reference it always had. Tier
A+B green (82 binaries).

**FST4 gets this for free.** `score_flat_coherent` is shared, and
`fst4_sync_search` is the FST4 wideband monitor's binding stage
(`docs/notes/FST4_BENCHMARK.md`). Not measured here.

## 30. Dual-core is worth 1.33x, not 2x (2026-08-30)

`dual_core.rs` and `wspr_dual_core.rs` are ~650 and ~715 lines. Writing
that for FT4 on the strength of "the candidate loop is embarrassingly
parallel" is the mistake §26.1 and §27 each paid for once already, so
this is a feasibility probe first: split the 12 candidates across two
pinned tasks running byte-identical code, against a single-core
reference over the same work.

```
dual-core probe — 12 candidates | 1 core 2906 ms | 2 cores 2182 ms
                | 1.33x | decodes 11 (serial 11)
```

**1.33×.** The candidate loop is parallel in structure but not in
practice, for two reasons that were visible beforehand and one that was
not:

- `esp_dsp_fft`'s `Fc32Guard` is a process-global spinlock held across
  every transform, because the esp-dsp fc32 twiddle table is a single
  global that has to be *resized* per length. The DDC and the Δt search
  use no FFT, but `symbol_spectra`'s per-symbol 32-point transforms do —
  ~19 % of the SHIP candidate time — and they serialise. A spinlock also
  means the waiting core burns cycles rather than yielding them.
- Both cores stream `cd0` (40 KB per candidate) and the slot audio from
  PSRAM, so they contend for the bus that §26.3 already showed is this
  board's real constraint.
- Candidates are not equal cost, so a 6/6 split leaves one core idle at
  the end.

Projected onto the ship slot: candidates 2 530 → ~1 902 ms, slot →
~2 660 ms, **1.67× → 1.36×**. Real, but against ~700 lines of production
machinery and a worker stack in internal DRAM — which is the same 31 744
B block the 2304 scratch (18 KB, §26.3) and FT8's 3840 scratch (30 KB,
§28) are already competing for. That is a poor trade next to what the
alignment work returned for a fraction of the code.

### 30.1 What to do instead, on the same evidence

The SHIP slot's 2 530 ms of candidate work now splits as DDC 1 339 (53 %),
Δt search 700 (28 %), LLR+BP 491 (19 %). Every `dot_f32` in it is on the
PIE path (§29), so the dots are done.

What is left in the DDC is **not** arithmetic. Stage A is 70 ms per
candidate of which the dots are ~16; the other ~54 ms is
`FirStage::push_one` called 90 000 times — two stores, a counter, and a
compaction test per input sample, for a stage that produces 4 995
outputs. That is ~648 ms per slot, 20 % of the budget, spent on
bookkeeping.

(It is not a regression from §27's rework: the same split measured 52 ms
before those changes and 54 after, while the dots went 36 → 16.)

`FirStage::push_block` exists and documents itself as the block-mode
entry point, but its body is still `push_one` in a loop, and
`CandidateDdc` never reaches it anyway — `push_i16` mixes and pushes one
sample at a time, so stage A never sees a block. Mixing in chunks and
giving `push_block` a real body (append the block to the history, then
compute outputs at their strides) is pure data movement, bit-identical
by construction, and contained to two files.

**That is the better next move**: comparable or larger than dual-core,
at a fraction of the code, and it does not spend internal DRAM that
FT8 and a future dual-core worker both need.

## 31. Block-mode FIR: 172 ms, not the 648 predicted (2026-08-30)

§30.1 pointed at `FirStage::push_one` being called 90 000 times per
candidate to produce 4 995 outputs, and put ~54 ms of stage A's 70 ms
there — ~648 ms per slot. `push_block` had documented itself as the
block entry point since `wspr::ddc` needed one, but its body was still
`push_one` in a loop and `CandidateDdc::push_i16` never called it.

Both fixed: `push_block` now bulk-appends each run with
`copy_from_slice` and emits the outputs whose windows close inside it,
and `push_i16` mixes in 1 024-sample chunks so the stages actually see
blocks.

```
DDC        1 339 -> 1 167 ms  (-13 %)
stage A       70 ->    61 ms
ship slot  3 288 -> 3 119 ms  1.67x -> 1.59x
```

**172 ms, against ~648 predicted.** The estimate mis-attributed the
gap: stage A's non-dot 54 ms was not all bookkeeping. `ddc_stage_probe`
feeds the stage two 90 000-sample `Vec<f32>` — 720 KB of PSRAM — and a
good part of that 54 ms was reading them, which no amount of batching
removes. Fourth miss of the day, and the same root cause as §26.1:
attributing a measured residual to the mechanism that happens to be in
view.

Kept regardless — it is bit-identical by construction, pinned across
seven `(ntaps, decim, margin, chunk)` shapes including the DDC's own
199/18 and 263/1, and 172 ms is 5 % of the slot for a contained change
that spends no internal DRAM.

### 31.1 Dual-core is now worth less, not more

Re-measured in the same run: **1.33x -> 1.17x**. Amdahl, exactly as it
should be — the parallel part (FIR) got cheaper while the serialised
part (`symbol_spectra`'s FFTs behind `Fc32Guard`) did not. Every
optimisation that lands on the parallel side makes the case for §30's
~700 lines worse. Treat dual-core as closed.

### 31.2 Where the budget actually stands

Ship slot 3 119 ms against 1 960: coarse 758, DDC 1 167, Δt search 700,
LLR+BP ~494.

But that is the **14-signal** scene, and §23 put realistic FT4 occupancy
at 5-10. At 197 ms per candidate:

| occupancy | candidates | slot | vs budget |
|---|---:|---:|---:|
| 5 signals | 5.3 | ~1 801 ms | **0.92x** |
| 10 signals | 9.2 | ~2 570 ms | 1.31x |
| 14 (measured) | 12.3 | 3 119 ms | 1.59x |

**FT4 now fits at the low end of its own design range**, which it did
not this morning at any point in it. The remaining gap is the top of
that range and the FT8-density pessimum.

Streaming the coarse stage during capture (§25's list, item 2) takes
758 ms out of the post-slot budget and would put **the whole realistic
range under**: 10 signals becomes ~1 812 ms, 0.92x. `symbol_spectra_avg`
is 152 independent windowed transforms over the slot, each computable
as its samples arrive, and `wspr_app` already ships this shape
(`ddc_loop` -> `DDC_READY_IDX` -> `scan_loop`). It is also the stage
that *cannot* be parallelised, since it is pure FFT behind the global
guard — so moving it off the budget is the only lever it has.

## 32. Streaming the coarse stage, and a 100 % that was luck (2026-08-30)

`symbol_spectra_avg` is ~152 windowed transforms over the slot, each
depending only on samples that have already arrived. [`Ft4SavgBuilder`]
accumulates `savg` from audio blocks so a receiver completes each row
during capture, and `ft4_coarse_sync_from_savg` is what is left to pay
afterwards. Bit-identical at any block size, pinned on host across nine
chunk sizes including ones that do not divide `NSTEP`.

```
pass0s streamed coarse — build 798 ms (overlaps capture, 7 500 ms of it)
                       + pick    6 ms (post-slot)
                       | 12 candidates, identical to whole-slot pass0
post-slot coarse cost 761 ms -> 6 ms  (754 ms leaves the budget)
```

**The total length has to be known up front**, and that is not a
convenience: `getcandidates4.f90` averages exactly `(nz − NFFT1)/NSTEP`
rows, which is *not* "every row that fits". For a 90 000-sample slot
that is 152, while row 152 would still lie entirely inside the audio. A
builder that emitted greedily would average 153 rows and produce a
different `savg`.

### 32.1 Two mistakes, both caught by the bench

**The rewrite made the non-streaming path 2.5× worse.** The first
version buffered every push and compacted with `drain` after each row —
quadratic when the caller hands over a whole slot, which is exactly what
`symbol_spectra_avg` does: 152 rows each memmoving what was left of
90 000 samples. `ft4_coarse_sync` went 758 → 1 906 ms. Rows are now read
straight out of the caller's block whenever nothing is retained, and
only a row straddling two blocks touches the history.

**And the §29 result turned out to be luck.** In the same run the
scorer's dots went from 100 % on the PIE path to **0 %**, search 1 049 →
1 789 ms — with no change to `sync2d` at all. The odd-phase reference
handles the *parity* of `s0`, but only if `cd0`'s base is 16-byte
aligned; at an 8-mod-16 base no `s0` works. `cd0` is a
`Vec<Complex<f32>>`, alignment 4, so nothing guaranteed it — §29 simply
got an aligned 40 KB block, and leaking one more 18 KB internal buffer
elsewhere in the binary shifted the heap enough to lose it.

`AlignedCd0` now aligns it explicitly, copying once per call when
needed. That costs ~7 ms per candidate (search 1 049 → 1 129 ms at
±1.0 s) and is the difference between a 2.3× win and a coin flip.
**A measured 100 % that depends on an uncontrolled allocator is not a
guarantee** — and this one had already been written up as a result.

### 32.2 The budget, with the coarse stage off it

| | whole-slot coarse | streamed coarse |
|---|---:|---:|
| post-slot coarse | 761 ms | 6 ms |
| candidates (12) | 2 431 ms | 2 431 ms |
| **slot** | **3 192 (1.63×)** | **2 437 (1.24×)** |

At 203 ms per candidate, against §23's occupancy figures:

| occupancy | candidates | slot | vs budget |
|---|---:|---:|---:|
| 5 signals | 5.3 | ~1 082 ms | **0.55×** |
| 10 signals | 9.2 | ~1 874 ms | **0.96×** |
| 14 (FT8 density) | 12.3 | ~2 503 ms | 1.28× |

**The whole of FT4's realistic occupancy range now fits**, with the
FT8-density pessimum 1.28× over. This morning the same measurement was
2.33× at the design point and over everywhere.

What remains is not a decode-speed problem: `Ft4SavgBuilder` is a
library part, and no embedded binary calls it yet. Wiring it to the UAC
capture path — the way `wspr_app` already runs `ddc_loop` →
`DDC_READY_IDX` → `scan_loop` — is what turns the 754 ms into a real
receiver's headroom.

## 33. Does the streamed coarse stage keep up with capture? (2026-08-30)

§32 moved 754 ms off the post-slot budget on the strength of a 10 %
duty cycle. The average is close to irrelevant to whether it works.
`uac::reader_thread` calls `AudioSink::push_samples` with one read's
worth of resampled audio — `dst_scratch` is sized for ~256 samples at
48 k → 12 k — **from the reader thread, holding the sink mutex**, so a
slow block delays the next `uac_host_device_read` rather than queueing
behind it. And the work is not spread evenly: rows land every
`NSTEP = 576` samples, so one block in a few pays a whole 2 304-point
transform while its neighbours pay tens of microseconds.

| block | audio/block | blocks | with a transform | min | p50 | **max** | max / budget | duty |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 128 | 10.7 ms | 704 | 152 | 4 µs | 18 µs | 5 485 µs | **51 %** | 10 % |
| **256** | **21.3 ms** | 352 | 152 | 19 µs | 38 µs | **5 508 µs** | **25 %** | 10 % |
| 512 | 42.7 ms | 176 | 152 | 39 µs | 5 393 µs | 5 659 µs | **13 %** | 10 % |

**At the UAC's actual block size the worst block is 25 % of its own
real-time budget**, and even at 128 samples it is 51 %. The builder can
be driven straight from the reader thread; it does not need a task of
its own behind a queue.

152 rows at every block size, and `savg[100]` identical to the last
digit across all three — the host test's block-independence holds on the
device as well.

Two things this does **not** establish, and neither is closable without
a radio on the bench:

- it measures the builder alone on an otherwise-idle core. In the app
  that thread also resamples, and the core carries WiFi and the USB
  host.
- whether 5.5 ms of extra latency in the reader loop costs the USB host
  driver a packet is a property of its buffering, not of this
  measurement. The margin says it should not; that is not the same as
  knowing.

(The first run of this reported 129 rows at n = 512 rather than 152 —
the "did this block do a transform" test was a fraction of the block's
own budget, and at 512 a row block lands either side of it. A fixed
threshold separates 5 400 µs from 40 µs unambiguously. Fixed, and noted
because a diagnostic that quietly miscounts is worse than none.)

## 34. A slot budget the candidate loop actually respects (2026-08-30)

§33's receiver ran every candidate the coarse stage produced and took
2 412 ms, which is fine for a monitor and not fine for a transceiver:
the 1 960 ms budget is the gap between the frame ending and the slot
ending, and a radio that has to key up does not have 2 412 ms.

`decode_slot` now takes a `budget_ms` and stops starting candidates
once it has passed, the same shape `fst4_monitor::run_candidate_loop`
uses — check before each candidate rather than predicting whether the
next one fits. **The deadline is measured from
`CapturedSlot::closed_us`, not from entry**: a decoder that started
late because it was still finishing the previous slot gets
correspondingly less time, which is the accounting that keeps a
receiver current rather than progressively behind.

**The cut takes the weakest.** `ft4_coarse_sync` returns candidates in
descending coarse score, so truncation drops the tail.

Measured, `ft4-demo` at `TX_TURNAROUND_BUDGET_MS`:

```
slot 1 — 11 of 12 candidates tried, 10 decodes in 2146 ms of 1960 ms — cut 1 weakest at score 1.55
slot 2 — 10 of 12 candidates tried,  9 decodes in 1963 ms of 1960 ms — cut 2 weakest at score 2.63
```

What it gives up on this slot is `W7BOB KJ7G RR73` at −17 dB, the
weakest of the eleven, and on the tighter slots `NZ7P WA7JAY` at −13 dB
as well. The alternation is the deadline doing its job: the demo's
pacing lands slot close 7 490 or 7 510 ms apart, and the decoder gets
whatever is left.

### 34.1 The overshoot is one candidate, and it is structural

2 146 ms against a 1 960 ms deadline is a 186 ms overrun — the
candidate that was already running when the deadline passed. Checking
before starting cannot do better than that, and one FT4 candidate is
~10 % of the whole budget, against ~1 % for an FST4 monitor candidate
of a 52 s deadline. The shape that works there is looser here.

It is still tolerable: the next slot's transmission starts at
`TX_START_OFFSET_S = 0.5`, so a 186 ms overrun leaves 314 ms. Whether
to close it by predicting from a running per-candidate mean is a real
question, and deliberately not answered yet — a prediction that is
wrong stops a candidate early, which costs a decode outright, while the
overshoot costs only margin. Measure before choosing.

### 34.2 This is the pessimum, not the design point

The golden is the 14-signal FT8-density scene (§23.5), which is why
there is anything to cut. At FT4's own 5-10 signal occupancy the coarse
stage produces 5-9 candidates and the loop finishes inside the budget
with nothing dropped — §31.2's 0.55× and 0.96×. `RX_ONLY_BUDGET_MS`
(7 500 ms) is there for a receiver that never transmits, where the only
thing an overrun costs is the next slot, and nothing needs cutting at
all.

## 35. FT8's stage split, and what the waterfall was costing (2026-08-30)

Two measurements prompted by one question: is converting i16 audio to
f32 done so esp-dsp's f32 FFT can be used, and is that a reasonable
price for a display?

### 35.1 FT8 does not convert, and stage 1 is not a display cost

`fixed-point`'s `compute_spectrogram` reads `audio[k].to_i16() as i32`
and stays integer into the sc16 kernel — the i16 path exists precisely
so the conversion does not happen. And `stage1_inc` feeds `wf_q`
alongside the coarse search, so the spectrogram serves both; the
waterfall is a free rider on it, not its reason.

First per-stage breakdown of FT8 on the board it ships on (CoreS3,
`qso3_busy.wav`, ship config, 7 decodes):

| stage | ms | share |
|---|---:|---:|
| **1 — spectrogram** | **1 403** | **51 %** |
| 2 — coarse sync | 236 | 9 % |
| pass 2 — re-rank | 166 | 6 % |
| 3 — refine + LLR/BP | 915 | 34 % |
| **total** | **2 726** | |

Every FT8 stage figure in this tree until now came from a StickS3 or
from the app's own logs.

### 35.2 The FT4 waterfall was costing 10 % of the stage it rides on

FT4's `wf_row` was a different matter, and the question landed
squarely on it. The *rows* are free — `push_with_rows` hands over
spectra the coarse stage is already computing — but the mapping on top
of them had never been measured, and "the rows are free" is not the
same claim.

| version | per row | per slot | of the coarse stage |
|---|---:|---:|---:|
| `10 * log10` per column | 623 µs | 94 ms | **10 %** |
| integer log2 (`leading_zeros`) | 384 µs | 58 ms | 6 % |
| …and the bin mapping folded into two constants | **174 µs** | **26 ms** | **3 %** |

`f32::log10` is ~620 cycles here and there were 240 per row. Replacing
it with the exponent — bit position of the MSB plus one fractional bit
for half-octave resolution, which is what FT8's `decimate_pair_to_wf`
has always done — took a third of it.

The rest was worse and less excusable: **four float divides per column
to work out which bins it covers**, 960 a row, for a mapping that
depends on nothing but constants and never changes. Folded into a
start and a step, so the loop is one add and two truncations.

The palette is 16 coarse steps over 30 dB, so quantising the level to
half-octaves (~1.5 dB) is below what it can show — the picture is
unchanged.

**The lesson is the framing, not the microseconds.** "The transforms
are free" was true and was allowed to stand in for "the waterfall is
free", which was never measured and was not true. 94 ms a slot is not
fatal, but it is a tenth of the stage it was riding on, spent on a
picture.

## 36. What `fixed-point` actually buys FT8 (2026-08-30)

Three measurements, prompted by the question "could FT8 have taken
FT4's simpler route?". FT4 runs f32 end to end on this board; FT8's
embedded build quantises the spectrogram to u16 and the LLR/BP ladder
to Q11i16, and the recorded rationale is halved PSRAM bandwidth.

### 36.1 Recall: it costs nothing

`fixed-point` implies `nstep-half`, so no measurement had separated
them. `nstep-half` is standalone, so they can be:

| build | decodes |
|---|---:|
| `full` (f32, NSTEP = NSPS/4) | 14 |
| `full,nstep-half` (f32, NSTEP = NSPS/2) | 12 |
| `full,fixed-point` | 12 |

The last two decode the **same twelve messages**. The whole 14 → 12 is
the coarse time grid; quantisation costs nothing here. See
`ft8_qso3_decode_set` and `ft8_qso3_apoff_recall`'s corrected floor.

### 36.2 Spectrogram: 1.11× for half the memory

Both builders instantiated in one binary
(`compute_spectrogram_f32_timed`), same audio, same allocator state:

```
stage 1 (spec, u16)   1 403 ms   351 KB
stage 1 f32 (A/B)     1 563 ms   702 KB   → fixed-point is 1.11x
```

The memory halves as designed. **The time does not**: 11 %, where a
bandwidth-bound stage handed half the data should approach half the
time. So stage 1 is not bandwidth-bound — it is the 184 transforms and
the windowing/magnitude/store around them.

Not a pure cell-type contrast: the f32 path windows rectangular, the
fixed-point one Hann, and the latter also scans the slot to choose its
shift. It is what each path costs.

### 36.3 BP: fixed-point is **slower**

`LlrScalar` is a trait and `BpScratch`/`bp_decode_nms_with_scratch` are
generic over it, so unlike the spectrogram both can simply be
instantiated. Same LLRs, same iteration count, same binary:

```
BP Q11i16   22 813 us/run
BP f32      19 455 us/run   → Q11i16 is 0.85x f32
```

**18 % slower.** The LX7 has an f32 FPU; doing the check-node
arithmetic in i16 does not save anything and costs the saturating
helpers. This is the same result `#198` recorded for FST4 and FT4 — on
LX7 `fixed-point` measured *slower* than f32 — now shown for FT8's own
BP, which is the reason it was never revisited there.

### 36.4 So the answer is mostly yes

| claim | measured |
|---|---|
| costs recall | **no** — identical decode set |
| avoids an i16 → f32 conversion | no conversion exists either way |
| halves the spectrogram | **yes**, 702 → 351 KB |
| speeds up the spectrogram | 1.11× |
| speeds up BP | **0.85× — it is slower** |

What `fixed-point` defends is **memory**, not speed: 351 KB of PSRAM
against 702 KB, on a board with 8 MB of it. The one place it is clearly
right is the streaming `stage1_inc`, whose whole design is an
incremental u16 spectrogram — and that path is **not** what §36.2
measured, so this does not say it is wrong there.

Nothing here proposes ripping it out. It says the simpler route was
available for more of FT8 than was assumed, and that the LLR/BP half in
particular is carrying a quantisation that costs 18 % and buys nothing
measurable.

## 37. Budget is decodes, not headroom (2026-08-31)

§31.2 and §32.2 concluded that FT4 fits its budget at realistic
occupancy, and that was allowed to stand in for "so freeing more budget
buys little". Those are different claims and the second is false.

§34's cutoff is **firing right now**:

```
slot — 11 of 12 candidates tried, 10 decodes — cut 1 at score 1.55
slot — 10 of 12 candidates tried,  9 decodes — cut 2 at score 2.63
```

Eleven decode without a deadline; nine or ten with one. And §23's
occupancy table says that is not an artefact of this recording — the
**deepest decoding rank tracks the candidate count**:

| signals/slot | mean candidates | deepest decoding rank |
|---:|---:|---:|
| 5 | 5.3 | 6 |
| 10 | 9.2 | **10** |
| 14 | 12.3 | **15** |
| 20 | 14.8 | **19** |

The tail of the candidate list is not dead weight. Truncating it costs
decodes, at every occupancy that has been measured.

So milliseconds convert to stations. At ~197 ms per candidate a 1 960 ms
budget reaches 9.9 candidates; every 197 ms saved is one more, and the
saving matters **most on a crowded band**, which is exactly when the
decodes are wanted. That inverts §36's ranking of what is worth doing.

| lever | saving | candidates |
|---|---:|---:|
| wire the streamed coarse into the receiver (built, unwired) | **754 ms** | **+3.9** |
| shared decimation (§37.1) | 278 ms | +1.4 |
| LLR/BP — 479 ms, never broken down | ? | ? |

### 37.1 The shared decimation, measured

`ft4::ddc` shares nothing between candidates: it mixes and filters the
whole 90 000-sample slot per candidate, where FT8's spectrogram and
`downsample_cached`'s wideband transform are each computed once and
read by all of them. Stage A alone is 61 ms of a candidate's 96.

One shared decimate-by-2 over the real audio, then per-candidate mixing
and a decimate-by-9 on half the samples with half the taps:

```
A now (12k, per candidate)   60 986 us  (199 taps, /18, 90 000 in -> 4 995 out)
A after /2 (6k, per cand)    30 828 us  (101 taps,  /9, 45 000 in -> 4 995 out)
shared /2 (once, complex)   167 130 us  (111 taps,  /2, 90 000 in -> 44 973 out)

today    731 ms (12 x 60)
proposed 453 ms (shared 83 + 12 x 30)
```

The per-candidate half came out exactly as predicted, which after §26.1,
§27 and §31 is worth noting rather than assuming. The shared leg is
dearer than hoped — 83 ms is 1.4 candidates — so the break-even is
about **2.7 candidates**: always a win at §23's 5-10 signal occupancy,
roughly neutral on a dead band.

Not yet built, and it is not a drop-in. The shared stage wants a
real-input `FirStage` (the current one filters I and Q, so half its
work would be wasted, and the 83 ms above assumes that half removed);
the passband becomes three stages rather than two, and §20's point that
"the passband is a decode parameter, not a filter-design free choice"
applies — the +0.021 dB equivalent-noise-bandwidth match would have to
be re-established, and `ft4_ddc_equivalence`'s 560-file 0.0 dB result
re-run, since a third stage changes the rounding and nothing will be
bit-identical.
