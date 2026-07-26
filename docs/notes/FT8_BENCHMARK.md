# FT8 sensitivity benchmark — environment setup

How to reproduce the FT8 AWGN/fading SNR sweep (`tests/ft8_sweep.rs`) from
a clean checkout. Mirrors [`FT4_BENCHMARK.md`](FT4_BENCHMARK.md) and
[`FST4_BENCHMARK.md`](FST4_BENCHMARK.md) — same `WSJT-X Fortran source →
simN sim binary → WAV corpus → cargo test --ignored` pipeline, this time
for `ft8sim`.

This is a different thing from the existing CI "ft8 characterization"
suite (`ft8_decode_block_snr_sweep` and friends, see `.github/workflows/ci.yml`)
— those sweeps synthesize signals with a homegrown LCG noise generator,
not validated against any WSJT-X-native ground truth. `ft8sim` is
WSJT-X's own Fortran simulator (same family as `fst4sim`/`ft4sim`),
so a corpus generated from it is a true Watterson-fading reference,
not a self-consistency check.

## 1. Prerequisites / 2. Build `ft8sim`

Same packages and pattern as [`FT4_BENCHMARK.md`](FT4_BENCHMARK.md)
sections 1-2 — `ft8sim` reuses the identical shared lib subtree (FT8 and
FT4 share LDPC(174,91), CRC-14, `sgran` RNG seeding):

```sh
sudo apt-get install gfortran build-essential libboost-dev libfftw3-dev
scripts/build_ft8sim.sh [/path/to/WSJT-X] [out-dir]
# defaults: WSJT-X-dir = ../WSJT-X, out-dir = target/ft8sim/
```

Sanity-check:

```sh
target/ft8sim/ft8sim "CQ JL1NIE PM95" 1500 0.0 0.0 0.0 1 -15
ls 000000_*.wav
```

(Same 7-arg CLI as `ft4sim`: `message f0 DT fdop del nfiles snr` — no
T/R-period argument, FT8 has no sub-modes either.)

## 3. Generate the WAV corpus

```sh
scripts/gen_ft8_sweep_wavs.sh [ft8sim-path] [out-dir]
# defaults: ft8sim-path = target/ft8sim/ft8sim
#           out-dir     = embedded-poc/assets/ft8_sweep/
```

Same 4 channels × grid × `TRIALS` structure. Default `SNRS` grid (`-5`
down to `-26` dB) straddles FT8's published WSJT-X AWGN threshold of
about **-20 to -21 dB** (2500 Hz ref BW).

## 4. Run the sweep

```sh
MFSK_FT8_SWEEP_DIR=embedded-poc/assets/ft8_sweep \
  cargo test --test ft8_sweep --release \
  --features ft8,fft-rustfft,parallel,uvpacket \
  -- --ignored --nocapture
```

### Measured 2026-07-18 (this corpus/seed)

50% crossing, linear-interpolated:

| Channel | ~50% crossing |
|---|---:|
| AWGN | ≈ -20.4 dB |
| CCIR good | ≈ -20.0 dB |
| CCIR moderate | ≈ -18.3 dB |
| CCIR poor | ≈ -18.2 dB |

AWGN and CCIR-good land within ~1 dB of the -20/-21 dB published figure
— a much tighter match than FT4 showed against its -17.5 dB reference
(~2 dB gap, see `FT4_BENCHMARK.md`). This is consistent with FT8's
production `decode_frame` running through `decode_block`'s WSJT-X-
faithful pipeline (post-#48 consolidation) rather than the generic
`core::pipeline` path FT4/FST4 use. CCIR moderate/poor show a larger
~2-2.5 dB gap — not yet investigated; a candidate follow-up for the
diagnose-before-fixing approach in `FST4_BENCHMARK.md` section 6 if
fading-specific sensitivity becomes a priority.

## 5. No `DecodeStrictness` probe here

Unlike the FT4/FST4 sweeps, this one doesn't include a strictness
calibration probe. FT8's production path calls `process_candidate`
(not `pub`) with its own already-calibrated `ft8::decode::DecodeStrictness`
(see the "Calibrated from real WAV bench 2026-04-07" doc comment in
`src/ft8/decode.rs`) — a different, already-tuned struct from the
uncalibrated copy in `core::pipeline` that FT4/FST4 share (issue #72).
There's no public hook to vary it from an external test. What this
sweep gives FT8 for the first time is a systematic Watterson-fading
corpus to validate that existing calibration against, rather than a new
calibration target.

## 6. The old CI "ft8 characterization" suite — removed 2026-07-18

`.github/workflows/ci.yml` used to have a push-only matrix entry named
`ft8 characterization` covering `ft8_coarse_sync_concurrent`,
`ft8_decode_block_coarse_diag`, `ft8_decode_block_depth_sweep`,
`ft8_decode_block_pass1_sweep`, `ft8_decode_block_snr_sweep`, and
`ft8_qso3_coarse_sync_probe` (~10 min on every `ft8/**` push). All six
were audited and removed along with the matrix entry:

- All six were `println!`-only diagnostics with **zero assertions** —
  the tier could never fail regardless of what the numbers showed, so
  it bought no regression protection for its wall-clock cost.
- `ft8_coarse_sync_concurrent` required the `fixed-point` feature,
  which the CI `full` feature set doesn't enable — it was already
  running zero tests before this cleanup, just silently.
- `ft8_decode_block_snr_sweep` was the one actually synthesizing its
  own AWGN with a homegrown LCG generator (no fading model at all) and
  comparing two of our own decoders against each other with no
  external ground truth — the most direct match for "verifying against
  a non-golden simulator."
- The other four used real on-air recordings (`REAL_QSO_WAVS`,
  checked into `embedded-poc/assets/`), so they weren't synthetic, but
  still had no assertions.

Actual FT8 recall regression protection was already covered elsewhere
and needed no replacement: `ft8_qso3_apoff_recall` /
`ft8_qso3_apon_recall` are hard-assertion, always-on (`default` suite,
not `#[ignore]`'d) — these are the tests that would actually catch a
recall regression.

The separate `ft8 recall` matrix tier (`ft8_decode_block_real_qso`,
`ft8_reference_suite_recall`) turned out to have the same problem —
audited and fixed the same day: `ft8_reference_suite_recall` (a
PASS1_LIMIT/max_cand embedded-tuning config sweep, informational only)
was deleted; `ft8_decode_block_real_qso` was converted to a
hard-assertion floor test (embedded ship config vs host `decode_frame`
truth on `qso1`/`qso2`/`qso3_busy` — `qso1`/`qso2` have no WSJT-X
golden and were otherwise completely untested) and is no longer
`#[ignore]`'d, so the matrix tier itself was removed — it now runs in
`default`.

## 7. CCIR moderate/poor fading gap — diagnosed and closed (issue #72 follow-up, 2026-07-18)

Followed section 4's flagged open item using the same diagnose-before-
fixing discipline `FT4_BENCHMARK.md` sections 8-12 used to close FT4's
AWGN gap, including that investigation's two hard lessons: verify a
hypothesis with a real narrow re-sweep before crediting it, and make
sure any diagnostic applies the exact gates the real decode path does
before trusting a "rescue."

**Ruled out first (wrong angle, not a valid comparison):** hypothesised
that `process_candidate`'s unused `EqMode::Local` (every production
call site hardcodes `EqMode::Off`) might help under fading, and used
`decode_sniper_eq` (`target_freq ± 250 Hz`) as a stand-in to vary it
externally, since `decode_frame`/`process_candidate` have no public
`eq_mode` hook. **Corrected after review**: sniper mode is a hardware-
roofing-filter accommodation and `EqMode::Local` is specifically tuned
for BPF edge distortion, not a general channel condition — neither is
a valid stand-in for comparing against WSJT-X or for testing whether
equalization-in-general helps fading. (For the record, the invalid
experiment did show `EqMode::Local` making CCIR recall worse in every
cell tested — consistent with a BPF-tuned equalizer misapplied to a
different distortion pattern, but not treated as evidence either way.)

**Real diagnosis**: built `ft8_diag_weak_trials` (`tests/ft8_sweep.rs`,
mirrors `ft4_diag_weak_trials`) replicating `process_candidate`'s
prefix (`coarse_sync` → `fine_refine_3stage` → `nsync` gate) directly
against the public building blocks it itself calls — `process_candidate`
and `process_one_candidate_inner` aren't `pub` outside `crate::ft8`.
Across CCIR moderate/poor losing trials near the section-4 crossings:
the correct near-golden candidate was found essentially every time,
with `fine_refine_3stage` landing on the true position and `nsync`
clearing its gate comfortably (10-21 of 21) — coarse-sync and fine-
refine are healthy. The bottleneck is downstream, in LLR/BP/OSD.

Went internal (`ft8::decode::tests::ft8_diag_internal_osd_trace`,
`src/ft8/decode.rs` — `process_candidate` is a private `fn`, reachable
from that module's own `#[cfg(test)] mod tests` via `use super::*`)
with temporary `MFSK_FT8_OSD_TRACE`-gated tracing in
`process_one_candidate_inner`/`osd_strategy::try_fallback` (since
removed) to see per-stage `hard_errors`. Found: `OSD_HARDERRORS_MAX =
22` (`decode_block/osd_strategy.rs`) — a deliberate mfsk-core deviation
from WSJT-X's universal 36, tightened to filter 3 candidates on
`qso3_busy.wav` judged CRC-luck phantoms — was discarding *golden*
decodes under CCIR fading. Multiple independent LLR variants converged
on the exact transmitted text (`"CQ JL1NIE PM95"`, known ground truth
from `ft8sim`) at `hard_errors` in the high 20s/low 30s, all rejected
by the 22 ceiling.

**Controlled experiment**: widened `OSD_HARDERRORS_MAX` to WSJT-X's own
36 and re-measured (`ft8_snr_sweep`, real `decode_frame`):

| Channel | -21 | -20 | -19 | -18 | -17 |
|---|---:|---:|---:|---:|---:|
| CCIR moderate | 0%→0% | 10%→10% | 5%→25% | 55%→85% | 65%→85% |
| CCIR poor | 0%→5% | 10%→10% | 15%→35% | 45%→65% | 65%→90% |

No regressions anywhere in the swept range. Cross-checked against the
hard-assertion real-recording tests (`ft8_qso3_apoff_recall`,
`ft8_decode_block_real_qso`): **byte-identical** — same 7/8 golden +
7 phantom = 14 total, same 4/4, 5/5, 12/14+2. The widening didn't touch
those specific candidates at all.

It touched a different set, and the result reframes the section's own
history: `ft8_qso3_jtdx_recall.rs` (JTDX's 18-entry aggressive list)
went from 13/18 to **17/18** — recovering exactly the 3 candidates
(`N1API F2VX 73` e=30, `N1API HA6FQ -23` e=25, `CQ EA2BFM IN83` e=31)
that `OSD_HARDERRORS_MAX = 22` was introduced in 0.6.3 specifically to
exclude as assumed phantoms (recall had been 16/18 before that). Our
decoder now arrives at those 3 candidates' *exact* JTDX-claimed text
independently — two separate decoders converging on the same CRC-14-
protected message is strong evidence against coincidence (~1/16384²
if genuinely random), reclassifying them as real rather than phantom.
This also materially informs issue #150 (JTDX-18 ground-truth
question): 3 of the 4-5 previously-unverified entries now have
independent corroboration. `ft8_qso3_apon_recall.rs`'s multipass extras
floor similarly went from 4/6 back to its pre-0.6.3 5/6 (same
`CQ EA2BFM` recovery, different test).

**Adopted**: `OSD_HARDERRORS_MAX` set permanently to WSJT-X's 36 (was
22), doc comments in `osd_strategy.rs` rewritten to record this history
rather than the retracted phantom rationale, `ft8_qso3_jtdx_recall.rs`'s
`MIN_JTDX_HITS` raised 13→17, `ft8_qso3_apon_recall.rs`'s
`JTDX_EXTRAS_HARD_FLOOR_MULTIPASS` raised 4→5. Full non-ignored suite
and `-D clippy::perf` green throughout.

**Full re-sweep, all 4 channels** (`ft8_snr_sweep`, real `decode_frame`,
full `-5` to `-26` dB grid, same corpus/seed as section 4) — 50%
crossing, linear-interpolated:

| Channel | Section 4 (before) | Now (after) | Δ |
|---|---:|---:|---:|
| AWGN | ≈ -20.4 dB | ≈ -20.8 dB | +0.4 dB |
| CCIR good | ≈ -20.0 dB | ≈ -20.6 dB | +0.6 dB |
| CCIR moderate | ≈ -18.3 dB | ≈ -18.6 dB | +0.3 dB |
| CCIR poor | ≈ -18.2 dB | ≈ -18.5 dB | +0.3 dB |

No regressions anywhere in the grid (0% stays 0% at the noise floor,
100% stays 100% at strong SNR — monotonic, as expected from loosening
a reject-only gate). The crossing-point shift is more modest than the
fixed-SNR-cell recall jumps earlier in this section suggest (e.g. CCIR
moderate -18 dB: 55%→85%) — the recall-vs-SNR curve is steep through
this region, so a given percentage jump maps to a smaller dB shift at
the 50% point specifically. AWGN's ≈-20.8 dB now sits inside WSJT-X's
own published -20 to -21 dB range rather than at its edge; CCIR good
gained the most of the four (+0.6 dB) despite showing the least
fading-driven OSD-ceiling pressure in section 4 — consistent with the
fix's mechanism (recovers borderline decodes generally, not fading
specifically) rather than a fading-only effect. CCIR moderate/poor's
gains (+0.3 dB each) are real but smaller than the qualitative "85%/
90%" recall figures might suggest on their own — the fixed-SNR-cell
numbers and the 50%-crossing numbers are both correct, they're just
answering different questions (recall at a specific operating point
vs. the threshold definition used for cross-mode comparison).

## 8. SIC LPF window bug — JTDX 17/18 → 18/18 closed (issue #180, 2026-07-25)

Follow-up to section 7's `OSD_HARDERRORS_MAX` widening, which took
`ft8_qso3_jtdx_recall.rs`'s JTDX-18 recall from 13/18 to 17/18 and left
exactly one miss: `WA2FZW DL5AXX RR73` @ 2545.88 Hz, at the time
classified a likely JTDX false positive (`coarse_sync` found candidates
near its frequency but nothing recovered a message).

**That classification was wrong.** Issue #180's investigation (originally
chasing a *different* miss, `CQ DX DL8YHR JO41` @ 2606.25 Hz, also on
`qso3_busy.wav`) ground-truthed against a real WSJT-X `jt9 -d3` build and
found jt9 itself decodes `WA2FZW DL5AXX RR73` on this exact WAV — real
signal, not a JTDX artifact. jt9's own disk-decode is staged/checkpointed
(not a flat single pass): it decodes progressively larger audio prefixes
at three checkpoints, subtracting each stage's confirmed decodes before
the next, harder stage runs — DL8YHR only decodes after 13 such
early-subtracted signals are removed, WA2FZW among them.

Chasing why mfsk-core's own SIC left more residual behind than
`subtractft8.f90` on the same signals (not a sync/LLR/BP/OSD bug — a
decisive test loaded jt9's own dumped post-SIC residual buffer directly
into mfsk-core's unmodified decode chain and it decoded DL8YHR outright)
led to `subtract_tones_lpf`'s LPF kernel: `normalized_kernel`'s cos²
window divided its argument by `lpf_half` instead of `NFILT` (`=
2*lpf_half`), doubling the taper's argument range from `[-pi/2, pi/2]`
to `[-pi, pi]`. `cos²` is a single monotonic taper (1 at centre → 0 at
the edges) only over the first range; over the doubled range it dips to
0 at the quarter points and **rises back to 1 — full weight — at the
true edges** (`lpf_half` samples / 166 ms away from the current sample,
for FT8's `lpf_half=2000`). Verified numerically:

| offset (samples) | shipped kernel | correct kernel |
|---|---|---|
| 0 (centre) | 1.000 | 1.000 |
| 1000 | 0.000 | 0.500 |
| 2000 (edge) | **1.000** | **0.000** |

So every `subtract_tones_lpf` call since it became the canonical
FT8/FT4 SIC path (v0.6.2) — including every number in sections 1-7 of
this document that involves the SIC-dependent `decode_frame_subtract`
family — had been running a badly-misshapen "lowpass" giving 166-ms-stale
channel samples as much weight as the current one, actively corrupting
the QSB/channel estimate instead of smoothing it. One-line-formula fix
(divide by `2*lpf_half`, not `lpf_half`); same bug and same fix applies
to FT4 (`subtractft4.f90` uses the identical window formula).

**Result**: `ft8_qso3_jtdx_recall.rs` **17/18 → 18/18** — `WA2FZW DL5AXX
RR73` now decodes, zero new phantoms, zero regressions on the WSJT-X
8-entry golden (still 7/8, `K1BZM DK8NE` remains the one gap — see
issue #182 below) or the AP-on multipass JTDX-extras floor (still
5/6, same remaining miss at the time). Full non-ignored suite green
throughout.

**Update (issue #182, 2026-07-26)**: `K1BZM DK8NE`'s gap was **not**
an AP-list breadth problem, despite the "wider AP-list" hypothesis
this section originally pointed to. Root cause was `osd_decode_npre1`
(WSJT-X's OSD `ndeep=2` dispatch for this candidate's `q=11`) being
fed raw channel LLR instead of the BP-refined `bp_llr_zsum`, unlike
WSJT-X's real `decode174_91.f90` driver, which always hands OSD the
post-BP LLR. Threading the already-computed BP-refined LLR through to
the OSD call site (instead of recomputing/reusing the pre-BP one)
closed it: the AP-on multipass JTDX-extras floor is now **6/6**, and
the ship-config `ft8_qso3_jtdx_recall.rs` 18-entry check (which also
routes through the same OSD call site) now recovers `K1BZM DK8NE`
directly, no AP hint needed. See `CHANGELOG.md` for the full
investigation.

**Scope check — only in-band-SIC scenarios should move; section 4/7's
`ft8_snr_sweep` was deliberately *not* re-run to confirm this.** That
sweep synthesizes a single target signal per trial with no co-channel
interference, so `subtract_tones_lpf` is never invoked there — the fix
has no code path to affect those numbers through, so re-running it was
judged not worth the cost. This is inference from the fix's own call
graph, not a re-measurement of section 7's table. Indirect corroboration
exists: WebFT8's independent `ft8-bench` simulator suite (`docs/bench.md`
in the WebFT8 repo, not this one) *was* fully re-run the same day, and
every scenario without in-band interference (single target + AWGN/BPF
only) reproduced byte-identical before/after, while every scenario with
in-band SIC moved — the same scope boundary this argument predicts, just
demonstrated on a different corpus/harness than `ft8sim`. If this
distinction ever matters for a specific decision, re-run section 7's
sweep directly rather than relying on this note.

**mfsk-core issue/PR**: [#180](https://github.com/jl1nie/mfsk-core/issues/180) (investigation), [#178](https://github.com/jl1nie/mfsk-core/pull/178) (fix, merged).

## 9. AWGN/CCIR sweep re-measured, `DecodeDepth` redesign confirmed to have zero sweep impact (issue #182 follow-up, 2026-07-26)

Section 4's "Measured 2026-07-18" table had never been re-run since —
section 8 explicitly reasoned about scope rather than re-measuring.
Re-run directly this pass, prompted by the `DecodeDepth` enum→struct
redesign and the unrelated `auto_ap_strategy` removal (both landed the
same day, see `CHANGELOG.md`) — to confirm neither touched this
sweep's numbers.

**Confirmed unaffected, by construction**: `ft8_snr_sweep`'s
`decode_wav_ft8` calls `decode_frame` → `decode_frame_inner`, a
separate implementation from `decode_block::decode_block_multipass`
(the only caller `auto_ap_strategy::run` ever had) — `decode_frame_inner`
never invoked it, before or after the removal. Verified by `git diff`
across the whole redesign PR: zero files outside `ft8/decode.rs`,
`ft8/decode_block/*`, FT8-only tests, and the FT8 slice of FFI/embedded
glue were touched — no other protocol's sweep code path exists to have
moved either.

**Re-measured** (`ft8_snr_sweep`, `--ignored --nocapture`, same
`-5`..`-26` dB grid / 20 trials/cell corpus as section 4):

| Channel | 2026-07-18 | 2026-07-26 | Δ |
|---|---:|---:|---:|
| AWGN | ≈ -20.4 dB | ≈ -21.4 dB | -1.0 dB (more sensitive) |
| CCIR good | ≈ -20.0 dB | ≈ -20.8 dB | -0.8 dB |
| CCIR moderate | ≈ -18.3 dB | ≈ -18.9 dB | -0.6 dB |
| CCIR poor | ≈ -18.2 dB | ≈ -19.0 dB | -0.8 dB |

All four channels moved the same direction (more negative = lower SNR
needed = better) by a consistent 0.6-1.0 dB — not the `DecodeDepth`
redesign (confirmed unaffected above), but the accumulated effect of
every FT8 sensitivity fix landed between the two dates (OSD
`bp_llr_zsum` seeding, the `OSD_HARDERRORS_MAX` widening in section 7,
the SIC LPF window fix in section 8, and others tracked in
`CHANGELOG.md`) that were never rolled up into this specific table.
Single run, not averaged across repeats — treat deltas under ~0.5 dB
as within 20-trials/cell sampling noise (see the sparse-SNR-sampling
lesson linked from `BENCHMARKS.md`); the consistent same-direction
movement across all four channels here is the signal, not any single
cell.

This table (not `BENCHMARKS.md`'s own copy, which already carried
intermediate numbers close to today's — AWGN ≈-20.8, CCIR good ≈-20.6,
CCIR moderate ≈-18.6, CCIR poor ≈-18.5) is the one to treat as stale
after this pass; `BENCHMARKS.md`'s FT8 section has been updated to
today's figures directly.

**Also added this pass**: `tests/ft8_qso3_full_parity_recall.rs`, a
new regression distinct from `ft8_qso3_apoff_recall.rs`'s ship-config
7/8 floor — asserts the **host research config**
(`DecodeDepth::FULL`, `sync_min=0.8`, `max_cand=60`, the same default
`ft8_qso3_jtdx_recall.rs` already used) hits the full WSJT-X 8-entry
golden (**8/8**, including `K1BZM DK8NE -10`, which ship-config's
`DecodeDepth::EMBEDDED` structurally cannot reach with OSD compiled
out). Measured: **~139-148 ms** (single- or multi-threaded — this
candidate count doesn't parallelise much), ~7-8× faster than real
`jt9 -8 -d3`'s own measured ~1.1 s total file decode time, at full
recall parity. This is now the tracked answer to "how fast can host
match WSJT-X exactly on this WAV" — a question this document didn't
previously have a permanent regression test for.

## 10. CCIR moderate/poor "no separate WSJT-X figure" replaced with real `jt9 -8 -d3` ground truth (2026-07-26)

Section 4's sweep table (and `BENCHMARKS.md`'s copy) always carried
"—" in the WSJT-X-published column for CCIR good/moderate/poor —
WSJT-X publishes an official AWGN threshold but not per-fading-model
numbers. But `ft8sim` (the same WSJT-X-native simulator generating
this sweep's corpus) produces real CCIR-faded WAVs, and a real `jt9`
binary was available locally (`~/wsjtx-build/jt9`, sanity-checked
first against `qso3_busy.wav` — 22/22 decodes, byte-identical to the
known-good reference from `FT8_BENCHMARK.md`'s own prior
investigations) — so there was no reason "—" had to stay a permanent
gap. Ran `jt9 -8 -d 3` directly against the full AWGN/CCIR corpus
(all 3 channels × 13 SNR points × 20 trials = 780 files, ~62 s
wall-clock at 8-way parallel; a hit = `JL1NIE` appears in jt9's
stdout, mirroring the Rust sweep's own single-known-message
methodology):

| Channel | mfsk-core (section 9) | real `jt9 -8 -d3` | Δ (jt9 − mfsk-core) |
|---|---:|---:|---:|
| AWGN | ≈ -21.4 dB | ≈ -21.2 dB | mfsk-core +0.2 dB (ahead) |
| CCIR good | ≈ -20.8 dB | ≈ -20.75 dB | ~parity |
| CCIR moderate | ≈ -18.9 dB | ≈ -19.5 dB | **jt9 +0.6 dB (gap)** |
| CCIR poor | ≈ -19.0 dB | ≈ -19.7 dB | **jt9 +0.7 dB (gap)** |

AWGN and CCIR good confirm what section 9's own framing already
assumed — at or slightly ahead of real WSJT-X. **CCIR moderate/poor
are a different story**: a real, previously undocumented ~0.6-0.7 dB
sensitivity gap under heavier Watterson fading, now backed by direct
ground truth instead of "no published figure." This isn't the
`OSD_HARDERRORS_MAX` fading-recall issue section 7 already closed
(that was about a golden-decode ceiling on `qso3_busy.wav`, a
different axis from this sweep's clean-signal-plus-fading-channel
50%-crossing measurement) — this is a genuinely new, unclosed gap.

**Not yet root-caused.** Candidate directions for a future pass,
following this document's own diagnose-before-fixing discipline
(section 6's lesson, and `FST4_BENCHMARK.md` section 6's shared
playbook): the Watterson channel model's `fdop`/`del` parameters
scale with fading severity (`ccir_moderate` = 0.5 Hz/1.0 ms, `ccir_poor`
= 1.0 Hz/2.0 ms per the module doc in `ft8_sweep.rs`), so whatever's
different between mfsk-core and real jt9 here likely interacts with
channel decorrelation specifically — a plausible first place to look
is `fine_refine_3stage`'s coherent Costas-block combining (issue #182,
section covered in `BENCHMARKS.md`), which assumes phase stability
across the reference window; heavier fading could partially undermine
that assumption in a way this document hasn't checked, similar to the
"fading channels gained less than AWGN" pattern already observed for
FT4's analogous coherent-combining fix
(`FT4_BENCHMARK.md` section 9). Not chased further in this pass —
flagged here as the next concrete lead rather than left as an
unexplained "—". Tracked as
[#190](https://github.com/jl1nie/mfsk-core/issues/190).

Raw per-cell counts, jt9 -8 -d3, 20 trials/cell:

| SNR | CCIR good | CCIR moderate | CCIR poor |
|---:|---:|---:|---:|
| -15 dB | 19/20 | 20/20 | 20/20 |
| -17 dB | 19/20 | 17/20 | 18/20 |
| -18 dB | 19/20 | 18/20 | 16/20 |
| -19 dB | 19/20 | 11/20 | 15/20 |
| -20 dB | 16/20 | 9/20 | 8/20 |
| -21 dB | 8/20 | 3/20 | 2/20 |
| -22 dB | 3/20 | 1/20 | 0/20 |
