# SNR formulas — WSJT-X subroutine → mfsk-core function

Index of what each protocol's *reported* SNR actually computes, and
how faithfully it matches the real WSJT-X subroutine that produces the
number a user sees in WSJT-X/JTDX's own decode log. Written as part of
issue #255, which found that `engine::llr::compute_snr_db` — a single
generic "adjacent-tone power ratio" heuristic — was silently standing
in for every protocol's own, usually quite different, real formula.

Every function below has its own doc comment carrying the exact WSJT-X
source file:line, a transcription of the formula, and (where verified)
real-`jt9`-build ground truth numbers. This page only indexes them —
read the linked function for the details, don't duplicate them here.

| Protocol | Real WSJT-X subroutine | mfsk-core function | Status |
|----------|------------------------|---------------------|--------|
| FT8 | `ft8b.f90`'s `xsnr2` gate/formula | `ft8::baseline::compute_baseline_spectrum` + `ft8/decode_block/process_candidates.rs`'s `compute_xsig_wsjtx`/`apply_wsjtx_xsnr2` | **Shipped**, all 5 FT8 entry points (issue #243/#253) |
| FT4 | `ft4_decode.f90:226,452-457` | `engine::pipeline::ft4_snr_db`, via `Ft4`'s `GenericPipelineProtocol::snr_db` override | **Shipped**, all 4 call sites incl. AP path (issue #255) |
| FST4 | `fst4_decode.f90:592-621` (`xsig`/`arg`/`xsnr`) + `get_candidates_fst4.f90` (baseline) | `fst4::baseline::fst4_snr_db`, via every `Fst4s*`'s `GenericPipelineProtocol::snr_db` override | **Shipped**, **all 5 sub-modes verified** against `fst4sim` + real `jt9` (issue #255 §4) |
| Q65 | `q65.f90:744-793`'s `q65_snr` (the value WSJT-X actually displays — not the `esnodb`-based one computed inside `q65_dec_q3`/`q65_dec_q012`, which is always overwritten before display) | `q65::snr::q65_snr_db` (single-slot) / `q65::snr::q65_snr_db_averaged` (`iavg=1,2` multi-period) | **Shipped**, all 7 decode paths — the 3 multi-period call sites use an EMA-averaged composite spectrum (`q65_composite_spectrum_averaged`), matching WSJT-X's own `s1a` accumulation formula (issue #255 §5, #256) |
| JT9 | `symspec2.f90:52-54` (`sig`/`t`/`snrdb`), reached via `jt9_decode.f90:148`'s `nsnr=nint(snrdb)` | `jt9::softsym::symspec2_from_ss2` | **Shipped**, real-`jt9`-verified within +0.33…+2.86 dB (mean +1.3) on `130418_1742.wav` (issue #255) |
| JT65 | `jt65_decode.f90:251,254-255` (`s2db = 10·log10(sync2) - 35`, then clamp to `[-30,-1]`) | `jt65::rx::demodulate_aligned_with_confidence_and_snr` — **own** estimator, not the generic heuristic | **Audited, formula deliberately not ported** — the existing estimator already lands within ~0.7 dB of real `jt9`; only WSJT-X's display clamp was adopted (issue #255) |
| WSPR | `wsprd.c:1058,1063` (`smspec` local SNR, `-26.3` dB WSPR→2500 Hz reference) | `wspr::coarse_baseband` (`SNR_SCALING_DB`) | **Already faithful** — a `wsprd.c` port carrying the real scaling constant, so `snr_db` has the same calibration as wsprd's own spot output. Never on the generic heuristic. Not independently re-measured against `wsprd` here. |

## Why FT8 and Q65 aren't `GenericPipelineProtocol` implementors

`GenericPipelineProtocol::snr_db` (the trait mechanism FT4/FST4 share)
only covers protocols that actually route through
`engine/pipeline.rs`'s shared per-candidate closures. FT8 has its own
bespoke, non-generic decode engine and never instantiates this
pipeline at all; Q65 likewise has its own `q65::rx` receive chain,
built around `q65::search::Spectrogram` rather than
`engine::dsp::downsample`. Both get their own protocol-owned SNR
module instead — same precedent as `SupportsSicRounds`/
`SupportsSicEarly` not being implemented for protocols whose real
WSJT-X has no equivalent mechanism.

This split turned out to matter for more than code organisation:
FST4's real formula needed two non-obvious corrections before it
matched real `jt9` output (RMS-normalisation mismatch between this
crate's shared downsample pipeline and WSJT-X's own FST4 path, plus a
downsample scale-convention mismatch — see `fst4::baseline`'s module
doc), while Q65's port landed within ~1 dB on the *first* attempt with
no such archaeology needed. The difference: Q65's own energy
extraction (`q65::rx::extract_data_energies` and friends) FFTs the raw
audio directly, with no shared downsample/RMS-normalisation pipeline
in between to introduce a mismatch — confirming that the FT4/FST4
pipeline's own normalisation step (needed for `compute_llr`'s LLR
scale calibration, issue #18) is specifically what FST4's port had to
work around.

## Verification discipline

Every entry above marked "Shipped" was checked against a real local
`jt9` build's own reported SNR (or, for FT8/FT4's earlier work,
internal-value SNRAUDIT-style probes) on real off-air WSJT-X sample
recordings — not just synthetic signals. One exception: Q65's
multi-period (`iavg=1,2`) averaging path has no `jt9` CLI equivalent
to check against — `jt9`'s batch mode has no flag driving the
GUI-only Rx-cycle-to-Rx-cycle accumulation state `iavg` depends on
(confirmed empirically: passing the same multi-slot recording to one
`jt9` invocation across several files produces independent single-slot
decode attempts, not an averaged one). That path is verified
structurally (read against `q65.f90`'s own `s1a` accumulation) and for
recall (existing golden tests still decode), not against a real
`jt9`-reported number. `/home/minoru/src/WSJT-X`
(this crate's development machine) has a buildable WSJT-X checkout
with a working `build_jt9/` tree; adding a temporary `write(0,...)`
probe line to the relevant `.f90` file and running `make jt9` rebuilds
in a few seconds, letting a specific candidate's exact intermediate
values (not just the final displayed number) be compared directly
against this crate's own. Revert the probe edit with `git checkout --
lib/<file>.f90` afterwards — it's a local development aid, never
committed to the WSJT-X tree.

## Band-limited (roofing-filtered) input

Every formula above estimates a noise floor from a *band* of the
audio, so what happens when the audio is not full-bandwidth matters.
This is the normal case for sniper mode: the premise of a
`SniperRequest` deployment is a rig whose roofing filter has already
narrowed the RF to ~500 Hz before the ADC, so the decoder never sees
2.5 kHz of anything. Estimating a noise floor "wide" is then not
merely wasteful — most of the band being averaged is the filter's own
stopband, which is quieter than the real noise floor, so the estimate
comes out low and the reported SNR comes out high.

Measured on FT4, synthetic AWGN, 8 seeds per cell, signal at 1000 Hz,
against a 4-pole Butterworth 750-1250 Hz applied to the f32 mix before
i16 quantisation (the same roofing-filter model WebFT8's harness
uses). Error = reported − injected:

| path | noise-reference band | no filter | 500 Hz filter |
|---|---|---:|---:|
| `SniperRequest` (`coarse_sync`, 40th-percentile over the search window) | ±250 Hz = **500 Hz** | +0.16 dB | **+0.56 dB** |
| `DecodeRequest` 300-2700 Hz (`ft4_coarse_sync`, polynomial baseline) | 2400 Hz | −1.54 dB | **+2.69 dB** |
| `DecodeRequest` 750-1250 Hz (same, told the real passband) | 500 Hz | — | **−3.96 dB** |

Three things follow, none of them obvious from the un-filtered
measurements alone:

1. **Sniper is already right, structurally.** `decode_sniper_ap`
   passes `search_hz = 250.0`, so `coarse_sync`'s 40th-percentile
   noise reference spans exactly 500 Hz centred on the operator's aim
   point — which is the roofing filter's passband, because that is
   what the operator tuned. The filter costs it only ~0.4 dB. Nothing
   to fix here, and the width is worth preserving deliberately rather
   than by accident.

2. **Wide-band is the path that breaks under a roofing filter**, not
   sniper — and it degrades *with* the signal (+1.9 dB at −2 dB
   injected, +3.9 dB at −14 dB), because the deflated noise floor is a
   larger share of a weaker ratio. A caller feeding band-limited audio
   to a full-width `DecodeRequest` is the misconfiguration.

3. **But narrowing `freq_min`/`freq_max` to match is not the fix** —
   it is worse (−4.0 dB). `ft4_coarse_sync` fits a *polynomial
   baseline* rather than taking a percentile, and FT4's 83.3 Hz
   occupied bandwidth is a third of a 500 Hz analysis band, so the
   signal drags its own baseline up and the reported SNR down. The
   percentile estimator sniper uses tolerates this; the polynomial one
   does not.

Consequence for issue #255's open "wide-band and sniper disagree by
~2.8 dB" question: **do not close that gap by moving sniper toward
wide-band.** Those two figures were measured under different input
conditions — wide-band un-filtered, sniper filtered. Under the same
realistic (filtered) input, sniper is the accurate path and wide-band
is the one ~2 dB out. The proposal floated there (extract
`getcandidates4.f90`'s per-bin scoring so the sniper path reports the
same value wide-band does) would have dragged the accurate number
toward the inaccurate one.

Caveat, same as the rest of this page's synthetic work: absolute
constants here carry the harness's own SNR convention, so the
trustworthy part is the *deltas between configurations*, all of which
are computed over bit-identical audio buffers.

## JT65: audited, and deliberately *not* ported

Issue #255 listed JT65 as running on `engine::llr::compute_snr_db`,
the generic adjacent-tone heuristic. **That was wrong** — checking the
code rather than the issue text, `jt65::rx::
demodulate_aligned_with_confidence_and_snr` has always had its own
estimator: a signal / non-winning-tone power ratio carrying a real
`10·log10(2500/TONE_SPACING_HZ)` bandwidth offset, i.e. already
converted into WSJT-X's 2500 Hz reference convention rather than left
as a bare ratio.

Measured against a real local `jt9 -6 -b A` build over a 283-decode
`jt65sim` corpus (`scripts/gen_jt65_sweep_wavs.sh`), error vs `jt9`'s
own displayed value:

| injected | -22 | -20 | -18 | -16 | -14 | -12 | -10 | -5 | 0 | +5 | +10 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| err vs real `jt9` | +0.65 | -0.14 | -0.18 | -0.17 | -0.22 | -0.38 | -0.64 | -1.86 | -2.29 | -3.09 | -2.64 |

Within **±0.7 dB across -22…-10 dB**, which is where JT65 operates —
better than the FST4 port achieved (1.71 dB) and comparable to Q65's
(0.6-0.8 dB). Porting the real formula would mean reproducing
`sync2 = 3.7e-4·ccfbest/sq0` (`decode65a.f90:55`, an empirical
constant on a coherent-AFC cross-correlation this crate computes
differently) to replace a number that is already accurate. Not done,
on purpose.

What *was* adopted is the display clamp: `jt65_decode.f90:254-255`
pins the reported value to `[-30, -1]`, replacing an ad-hoc
`[-24, +49]`. Both ends were wrong, in opposite ways. The `-24` floor
bound before WSJT-X's own `-30` did, truncating the weakest decodes.
And JT65 is the one protocol here whose displayed SNR **saturates by
design** — verified directly, a `jt65sim` signal injected at +10 dB
and one at +5 dB both come back `-1` from real `jt9`. The widening
error at the strong end of the table above is this crate's estimator
compressing where `jt9` simply stops counting; it is invisible to an
operator, since everything up there displays as `-1` either way.

Regression guard: `tests/jt65_sweep.rs::jt65_reported_snr_tracks_injected`
asserts the per-cell mean over -22…-10 dB (the range where `jt9`
tracks injected SNR to ~1 dB, so asserting against the injected value
is asserting against `jt9`). It skips cleanly when the gitignored
corpus is absent, like the recall sweeps beside it.

## Who is still on the generic heuristic

After issue #255, `engine::llr::compute_snr_db` is reached by exactly
two things. Neither JT65 nor WSPR is among them, despite issue #255's
own text saying otherwise — both have always had their own estimator,
and the issue's protocol list was written from the call-graph of
`engine/pipeline.rs` rather than from each protocol's actual code.

1. **`GenericPipelineProtocol::snr_db`'s default body**
   (`engine/pipeline.rs:532,548`). FT4 and FST4 both override it, and
   they are the only implementors today — so this is currently a
   default nothing uses, kept as the honest fallback for a future
   protocol joining the generic pipeline before its real formula is
   known.

2. **`ft8::decode_block::process_candidates_with_ap`**
   (`process_candidates.rs:2177`) — see below. This one is a real gap.

### FT8's `decode_block` AP path — generic heuristic, deliberately

`apply_wsjtx_xsnr2` is applied at five sites: the four host
`ft8::decode` entry points and `decode_block_multipass`.
`process_candidates_with_ap` is not one of them — it assigns `snr_db`
from `compute_snr_db` and never post-processes it. That function backs
`decode_block_with_ap` / `decode_block_with_ap_tuned` (public API) and
`process_candidates_into_with_cs_scratch_tuned*`, which is what the
ESP32 apps call.

**This is not a user-facing gap, because the embedded apps do not
display that field.** `m5stack-{s3,cores3,core2}-app`'s
`decode_pipeline.rs` calls `ft8::decode_block::xsnr2_db_simple(&spec,
r, cell_scale)` and shows *that*; `r.snr_db` appears only as the
`raw=` term in a log line. `xsnr2_db_simple` is a separate,
purpose-built estimator with its own median-window (P50) local
baseline — chosen precisely because a mean baseline is dragged upward
by the signal being measured — and it is empirically calibrated:
`mfsk-app-shared`'s `snr_norm` module documents it landing within
±3 dB of WSJT-X / JTDX across the `qso3_busy` reference, which is why
`DEFAULT_CALIBRATION_OFFSET_DB` sits at `0.0` rather than carrying a
fudge term.

So the ordering is deliberate: the embedded display path was solved
separately, and earlier, by an empirically-validated estimator, and
`r.snr_db` on this code path is effectively a legacy field no consumer
reads for display. Wiring `apply_wsjtx_xsnr2` in here would also need
`sbase`/`spec` plumbed through, and on the `fixed-point` path it
cannot work at all — quantised `Spectrogram` cells put many noise
cells at u16 zero, so `fit_baseline`'s `log10(p.max(1e-30))` yields
sbase ≈ -250 dB and xsnr2 explodes (already documented at
`decode_block_multipass`'s own `cfg(not(feature = "fixed-point"))`
guard).

**Left as-is on purpose.** If a future consumer starts reading
`DecodeResult::snr_db` off `decode_block_with_ap`, revisit — but do
not "fix" it speculatively.

## FST4: all five sub-modes verified

When `fst4_snr_db` shipped, only FST4-60 had been checked — the one
sub-mode with a real off-air recording available locally — and the
other four were left as "share the same formula/derivation but aren't
individually confirmed". Closed using the `fst4sim` corpus
(`scripts/gen_fst4_sweep_wavs.sh`).

Injected SNR is a valid reference here because a real local `jt9 -7`
build reports within ~1 dB of it on this same corpus across every
sub-mode (measured: FST4-15 m10→-10, m18→-18; FST4-30 m15→-14,
m22→-21; FST4-60 m15→-15, m25→-25; FST4-120 m20→-20, m28→-28;
FST4-300 m24→-24, m32→-32).

Mean error, 3 trials/cell over the full SNR ladder:

| sub-mode | 15 | 30 | 60 | 120 | 300 |
|---|---:|---:|---:|---:|---:|
| AWGN | -0.45 | +0.43 | **-0.01** | -0.19 | **-1.26** |
| CCIR moderate | -0.35 | -0.01 | -0.44 | +0.07 | **-1.92** |

Four of the five sit inside ±0.5 dB, including under fading. **FST4-300
carries a real, SNR-independent ~1.3 dB offset** (~1.9 dB fading) the
others don't.

That offset is *not* a wrong parameter: `nsps`, `ndown` and
`snr_calfac` were each checked against `fst4_decode.f90:182-214` and
`:597-613` and all five sub-modes match WSJT-X exactly
(800/600/430/390/340; 720/1680/3888/8200/21504;
18/42/108/205/512). The likely origin is `fst4_snr_db`'s `xsig · NDOWN`
scale correction, which was derived and confirmed on FST4-60 — note
that sub-mode reads -0.01 dB here, i.e. exactly where it was pinned.
Left as a measured, documented residual rather than absorbed into a
per-sub-mode fudge factor, which is what fitting it would amount to.

Guard: `tests/fst4_sweep.rs::fst4_reported_snr_tracks_injected_all_submodes`.

### Method note: match the message, not `results.first()`

Worth recording because it produced a convincing false finding before
it was caught. Taking `results.first()` instead of the decode carrying
the corpus message silently admits spurious low-SNR decodes. Doing so
inflated FST4-15's `max |err|` from 1.45 dB to 6.43 dB, and generated
an apparent "FST4-300 is -5 dB off under fading, worsening with SNR"
result whose per-cell numbers were all *exactly* -42.85 dB reported —
a constant-valued false decode, not a signal. The constant is what
gave it away; a noisier artifact would have been reported as real.

## FT4 sniper SNR: variance, not a level-dependent bias (issue #258)

`msg::pipeline_ap::process_candidate_ap` feeds `SnrCtx::cand_score`
from the generic `coarse_sync` Costas-correlation score, not from
`ft4_coarse_sync`'s `getcandidates4.f90` value that `ft4_snr_db`'s
`10·log10(score − 1) − 14.8` is written against. That swap was tried
and reverted (see that function's own comment). Issue #258 asked
whether the residual is worth closing, characterising it downstream as
a **level-dependent "arch"** — ~1.2 dB of curvature peaking mid-range,
on top of a +1.78 dB mean gap — and therefore not correctable by a
constant.

Re-measured here against the `ft4sim` corpus, which real `jt9 -5`
tracks to ~1 dB (injected -5/-10/-14/-17 → reported -5/-11/-14/-18).
Both paths read the **same** buffers; sniper aimed exactly, with
`EqMode::Local` and a CQ AP hint, matching the downstream request
shape. 20 trials/cell:

| | mean | **sd** | n |
|---|---:|---:|---:|
| wide-band (`DecodeRequest`) | -0.30 | **0.44** | 134 |
| sniper (`SniperRequest`) | -0.50 | **1.73** | 126 |

Two things differ from the downstream characterisation:

1. **There is essentially no systematic gap** — 0.2 dB, not 1.78 dB.
2. **What distinguishes the sniper path is scatter, not bias.** Its
   per-file standard deviation is ~4× wide-band's, and grows as the
   signal weakens (0.88 dB at -5 dB injected, 2.45 dB at -16 dB).

That also explains the reported arch. At n=8 seeds/level with
sd ≈ 1.7 dB, the standard error of each level's mean is ~0.6 dB —
precisely the amplitude of the "curvature". A smooth-looking arch
across 7 levels is what sampling noise of that size looks like when
each point is a mean of 8. The same trap is worth remembering
generally: an apparent smooth trend in per-level means says nothing
until the per-level *spread* is quoted alongside it.

The remaining +1.64 dB flat wide-band offset reported downstream does
not reproduce here either (-0.30 dB on `ft4sim`). The likely cause is
a difference between that harness's own SNR convention and `ft4sim`'s
— worth checking by running real `jt9 -5` over that harness's own
output before treating it as a decoder offset.

**Not fixed, deliberately.** The residual is specific to the sniper
path, is dominated by variance rather than a correctable bias, and the
narrower fix #258 proposes (evaluating a `getcandidates4`-scale score
at the already-refined position, purely to feed `ft4_snr_db`) would
address a bias that the pooled measurement does not show. Consumers
should treat FT4 sniper SNR as carrying roughly ±1.7 dB of per-decode
scatter — which matters if a single decode's SNR becomes an on-air
signal report, and does not if it is averaged or displayed.

## A note on this page's own error rate

Three separate claims on this page have now been wrong in the same
direction, and all three were written by reading a description instead
of the code:

- Issue #255 listed **JT65** as being on `compute_snr_db`. It never
  was — `jt65::rx` has always had its own estimator.
- This page then listed **WSPR** the same way, copied from that issue.
  Also never true — `wspr::coarse_baseband` is a `wsprd.c` port with
  the real 26.3 dB scaling.
- This page then described the **FT8 `decode_block` AP path** as an
  unfixed 3-7 dB user-facing gap. Also wrong — the embedded apps never
  display that field.

The check that would have caught all three is the same one: grep for
the field's actual readers before describing what a user sees. Prefer
that over trusting any prose, including this page's.
