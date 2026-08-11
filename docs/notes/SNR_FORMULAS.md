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
| FST4 | `fst4_decode.f90:592-621` (`xsig`/`arg`/`xsnr`) + `get_candidates_fst4.f90` (baseline) | `fst4::baseline::fst4_snr_db`, via every `Fst4s*`'s `GenericPipelineProtocol::snr_db` override | **Shipped**, FST4-60-verified (issue #255 §4) |
| Q65 | `q65.f90:744-793`'s `q65_snr` (the value WSJT-X actually displays — not the `esnodb`-based one computed inside `q65_dec_q3`/`q65_dec_q012`, which is always overwritten before display) | `q65::snr::q65_snr_db` (single-slot) / `q65::snr::q65_snr_db_averaged` (`iavg=1,2` multi-period) | **Shipped**, all 7 decode paths — the 3 multi-period call sites use an EMA-averaged composite spectrum (`q65_composite_spectrum_averaged`), matching WSJT-X's own `s1a` accumulation formula (issue #255 §5, #256) |
| JT9 | `symspec2.f90:52-54` (`sig`/`t`/`snrdb`), reached via `jt9_decode.f90:148`'s `nsnr=nint(snrdb)` | `jt9::softsym::symspec2_from_ss2` | **Shipped**, real-`jt9`-verified within +0.33…+2.86 dB (mean +1.3) on `130418_1742.wav` (issue #255) |
| JT65 / WSPR | not investigated under issue #255 | `engine::llr::compute_snr_db` (generic adjacent-tone heuristic), wired in under issue #226 | Out of scope so far — issue #226 gave these protocols *a* decode-side `snr_db` at all, not necessarily WSJT-X's exact formula. Not audited here. |

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
