// SPDX-License-Identifier: GPL-3.0-or-later
//! FT4 per-candidate down-converter: the same complex baseband
//! [`downsample_cached`] produces, built by mixing and filtering
//! instead of by a 92 160-point FFT.
//!
//! [`downsample_cached`]: crate::engine::dsp::downsample::downsample_cached
//!
//! ## Why
//!
//! `ft4::decode::FT4_DOWNSAMPLE` asks for `fft1_size = 92_160`
//! (`2^11·3^2·5`) and `fft2_size = 5_120`. The first is the blocker on
//! embedded: it is not a power of two and it is an order of magnitude
//! past `CONFIG_DSP_MAX_FFT_SIZE_8192`, so ESP-DSP cannot serve it at
//! any size setting. The first FT4 hardware measurement
//! (`docs/notes/FT4_BENCHMARK.md` §17, 2026-08-29) had to bake that
//! transform on a host and ship it to the board as an asset —
//! `embedded-poc/assets/ft4_golden_fft_cache.bin`, 737 280 bytes — which
//! measures the per-candidate work but is not a receiver.
//!
//! A DDC removes the stage rather than accelerating it: mixing the
//! candidate to DC and decimating by an integer 18 needs no transform
//! at all, and the same 2 251 ms that `downsample_cached` spent over 31
//! candidates on a CoreS3 becomes a FIR budget that scales with taps
//! instead of with `fft2_size · log(fft2_size)` plus a slot-sized
//! forward transform nobody can run.
//!
//! **FT4 is the easy case.** `fst4::ddc` needs a rational resampler
//! (`docs/notes/FST4_DDC_DESIGN.md` §4.2) because `NSPS = 3888 = 2^4·3^5`
//! leaves a `3^5` denominator no integer decimation can reach. FT4's
//! `NDOWN = 18` divides 12 kHz exactly: `12_000/18 = 666.667 Hz` is the
//! baseband rate the refine path already runs at (`SyncDims::ds_rate`),
//! and `ds_spb = NSPS/NDOWN = 32` is a power of two. So this module is
//! two [`FirStage`]s and two mixers — no [`PolyphaseResampler`], no
//! `RxGrid`, no change to anything downstream.
//!
//! [`PolyphaseResampler`]: crate::engine::dsp::polyphase::PolyphaseResampler
//!
//! ## The passband is a decode parameter, not a filter-design free choice
//!
//! `downsample_cached` keeps the bins covering
//! `[f0 − 1.5·Δf, f0 + (ntones−1+1.5)·Δf]` = `[−31.25, +93.75] Hz`
//! around the candidate, and **zeroes everything else** — the extracted
//! block is copied into an otherwise-zero `fft2_size` buffer. That band
//! is asymmetric about `f0` (the four tones run *upward* from it), so a
//! plain low-pass centred on the candidate does not reproduce it.
//!
//! Matching it matters because `process_candidate_basic_impl`
//! RMS-normalises `cd0` over its whole length — WSJT-X
//! `ft4_decode.f90:231-232` — and `compute_llr`'s `LLR_SCALE` is
//! calibrated against that unit-RMS input. Noise admitted outside the
//! reference band is noise the normalisation divides by: pass the full
//! ±333 Hz baseband and the RMS is ~2.3× the reference's, which
//! rescales every LLR feeding BP. (Issue #18 is the same failure from
//! the other direction — no normalisation at all, and the decoder
//! converged on CRC-14 false positives.)
//!
//! So the chain mixes the **band centre** (`f0 + 31.25 Hz`) to DC,
//! low-passes symmetrically at ±62.5 Hz, and rotates the output back by
//! that same 31.25 Hz so `f0` lands at DC, which is the convention
//! `downsample_cached`'s cyclic bin shift establishes and every
//! consumer (`ft4_sync_search`, `symbol_spectra`) assumes. Real taps
//! throughout; the alternative — one complex-coefficient band-pass —
//! costs twice the multiplies for the same response.
//!
//! ## Stage split
//!
//! ```text
//! 12 kHz real i16
//!   → Mixer(f0 + 31.25 Hz)                       complex @ 12 kHz
//!   → FirStage A: 199 taps, fc 320 Hz, ÷18       complex @ 666.667 Hz
//!   → FirStage B: 263 taps, fc 56 Hz,  ÷1        complex @ 666.667 Hz
//!   → Mixer(−31.25 Hz)                           cd0, f0 at DC
//! ```
//!
//! The narrow filter sits at the *lowest* rate, the same reasoning
//! `engine::dsp::ddc`'s own module doc and `wspr::ddc`'s cascade record:
//! a FIR's tap count is set by its transition width relative to the
//! sample rate, so the sharp stage is 18× cheaper here than it would be
//! before the decimation. Stage A only has to keep stage B honest —
//! after ÷18 the fold zones are `[604, 729] Hz`, `[1271, 1396] Hz`, …
//! (content at `f` aliases to `f − 666.667k`), so a Blackman stopband
//! that starts at 486 Hz covers all of them.
//!
//! Per-candidate cost, in complex MACs over a 7.5 s slot:
//! `5000 × 199` (A) `+ 5120 × 263` (B) `≈ 2.3 M`, against the ~72 ms
//! per candidate `downsample_cached`'s 5120-point inverse measured on
//! the CoreS3. Untested on hardware — that measurement is the next
//! step, and this module deliberately keeps the split as two named
//! stages so it can be re-cut without touching the callers.

use alloc::vec::Vec;

use num_complex::Complex;

use crate::engine::ModulationParams;
use crate::engine::dsp::ddc::Mixer;
use crate::engine::dsp::fir_decimate::FirStage;
use crate::ft4::Ft4;

/// Input sample rate this module is written for — the whole WSJT
/// pipeline's, and the rate `NDOWN` divides. #323: analysis-grid
/// literal, pinned against `FT4_DOWNSAMPLE::input_rate` in
/// `tests::band_matches_the_reference_downsample`.
const INPUT_RATE_HZ: f32 = 12_000.0;

/// Baseband rate: `12_000 / NDOWN`. Identical to what
/// `SyncDims::of::<Ft4>(12_000.0)` hands `ft4_sync_search` and
/// `symbol_spectra`, which is why nothing downstream needs to know a
/// DDC produced this buffer.
pub const DS_RATE_HZ: f32 = INPUT_RATE_HZ / Ft4::NDOWN as f32;

/// Output length, matching `FT4_DOWNSAMPLE::fft2_size`.
///
/// 5 120 samples is 7.68 s at [`DS_RATE_HZ`], longer than the 7.5 s
/// slot: the reference gets the extra 120 from zero-padding the audio
/// out to `fft1_size`. Reproduced here (rather than stopping at the
/// 5 000 samples real audio yields) for two reasons — `ft4_sync_search`
/// reads up to `i0 = 1012` plus a 105-symbol frame, and the RMS
/// normalisation downstream divides by `cd0.len()`, so a shorter buffer
/// would shift the scale by `sqrt(5120/5000)` against the reference for
/// no reason.
pub const CD0_LEN: usize = 5_120;

/// `leading_pad_tones` / `trailing_pad_tones` / `ntones` from
/// `FT4_DOWNSAMPLE`, restated as the band this module has to reproduce.
/// Not read through `ft4::decode` because that module needs an FFT
/// backend and this one does not; the values are pinned against it in
/// `tests::band_matches_the_reference_downsample`.
const LEADING_PAD_TONES: f32 = 1.5;
const TRAILING_PAD_TONES: f32 = 1.5;

/// Distance from `f0` to the centre of the retained band:
/// `((ntones−1+trailing) − leading)/2 · Δf` = `1.5 · 20.833` = 31.25 Hz.
/// The chain mixes *this* to DC, not `f0` — see the module doc.
pub const BAND_CENTER_OFFSET_HZ: f32 =
    ((Ft4::NTONES as f32 - 1.0 + TRAILING_PAD_TONES) - LEADING_PAD_TONES) / 2.0
        * Ft4::TONE_SPACING_HZ;

/// Half-width of the retained band: `((ntones−1+trailing) + leading)/2 · Δf`
/// = `3 · 20.833` = 62.5 Hz.
pub const BAND_HALF_WIDTH_HZ: f32 =
    ((Ft4::NTONES as f32 - 1.0 + TRAILING_PAD_TONES) + LEADING_PAD_TONES) / 2.0
        * Ft4::TONE_SPACING_HZ;

/// Stage A — loose anti-alias ahead of the ÷18.
///
/// `fc` 320 Hz with 199 Blackman taps: transition ≈ `5.5·Fs/ntaps` =
/// 332 Hz, so flat to ~154 Hz (the band needs 62.5) and into the
/// stopband by 486 Hz (the first fold zone starts at 604 Hz). Every
/// number here has slack on the side that matters and none to spare on
/// tap count — this stage runs at 12 kHz, where taps are 18× more
/// expensive than in stage B.
const STAGE_A_NTAPS: usize = 199;
const STAGE_A_FC_HZ: f32 = 320.0;

/// Stage B — the real passband, at the baseband rate.
///
/// `fc` 56 Hz with 263 taps: transition ≈ 13.9 Hz, so flat to ~49 Hz
/// and null by ~63 Hz. That is the reference's own shape, not a
/// coincidence: `downsample_cached` tapers `edge_taper_bins = 101` bins
/// of raised cosine inwards from each edge, and at `12_000/92_160 =
/// 0.13 Hz` per bin that is 13.0 Hz — flat to ±49.5 Hz, zero at
/// ±62.5 Hz.
const STAGE_B_NTAPS: usize = 263;
const STAGE_B_FC_HZ: f32 = 56.0;

/// History margin for both stages, matching `fst4::ddc`'s: the stages
/// are small enough that a generous fixed margin costs little and keeps
/// `FirStage`'s compaction well amortised.
const HIST_MARGIN: usize = 512;

/// Streaming per-candidate down-converter — one instance per candidate
/// frequency, fed the slot's 12 kHz PCM.
///
/// Sample alignment is exact and needs no trimming: [`FirStage::new`]
/// starts its counter at `group_delay + 1`, so each stage's output 0 is
/// centred on its own input 0, and a cascade of them keeps `cd0[0]`
/// aligned with `audio[0]`. That is what lets `ft4_sync_search`'s
/// absolute `[-344, 1012]` window mean the same thing here as on the
/// FFT path.
pub struct CandidateDdc {
    mixer: Mixer,
    stage_a: FirStage,
    stage_b: FirStage,
    derotate: Mixer,
}

impl CandidateDdc {
    /// `f0_hz` is the candidate frequency, in the same absolute audio
    /// space as `SyncCandidate::freq_hz` — the frequency this baseband
    /// will carry at DC.
    pub fn new(f0_hz: f32) -> Self {
        Self {
            mixer: Mixer::new(f0_hz + BAND_CENTER_OFFSET_HZ, INPUT_RATE_HZ),
            stage_a: FirStage::new(
                STAGE_A_NTAPS,
                Ft4::NDOWN as usize,
                STAGE_A_FC_HZ / INPUT_RATE_HZ,
                HIST_MARGIN,
            ),
            stage_b: FirStage::new(STAGE_B_NTAPS, 1, STAGE_B_FC_HZ / DS_RATE_HZ, HIST_MARGIN),
            // Negative centre: `Mixer` is `exp(-j2π·centre·n/Fs)`, and
            // this stage has to undo the `+BAND_CENTER_OFFSET_HZ` the
            // input mixer applied, moving `f0` from `-31.25 Hz` to DC.
            derotate: Mixer::new(-BAND_CENTER_OFFSET_HZ, DS_RATE_HZ),
        }
    }

    /// Push one real input sample, appending whatever baseband sample
    /// it completes (0 or 1 — the chain decimates by 18 overall).
    #[inline]
    pub fn push_one(&mut self, x: f32, out: &mut Vec<Complex<f32>>) {
        let (i, q) = self.mixer.mix(x);
        let Some((i, q)) = self.stage_a.push_one(i, q) else {
            return;
        };
        let Some((i, q)) = self.stage_b.push_one(i, q) else {
            return;
        };
        let (i, q) = self.derotate.mix_complex(i, q);
        out.push(Complex::new(i, q));
    }

    /// Push a block of 12 kHz PCM.
    pub fn push_i16(&mut self, audio: &[i16], out: &mut Vec<Complex<f32>>) {
        for &s in audio {
            self.push_one(s as f32, out);
        }
    }

    /// Feed zeros until `out` holds `want` samples (or `max_zeros`
    /// input samples have been pushed, whichever comes first).
    ///
    /// This is both the filter flush and the reference's zero-padding:
    /// `downsample_cached` sees the slot padded out to `fft1_size`, so
    /// its own tail carries the filters' response to that padding
    /// rather than a hard stop, and so does this one.
    pub fn flush_to(&mut self, want: usize, out: &mut Vec<Complex<f32>>) {
        let max_zeros =
            (want + 1) * Ft4::NDOWN as usize + STAGE_A_NTAPS + STAGE_B_NTAPS * Ft4::NDOWN as usize;
        let mut pushed = 0usize;
        while out.len() < want && pushed < max_zeros {
            self.push_one(0.0, out);
            pushed += 1;
        }
    }
}

/// Block helper: the whole slot in, one `cd0` out — exactly
/// [`CD0_LEN`] samples, zero-padded if `audio` is short.
///
/// Drop-in for `downsample_cached(&fft_cache, f0_hz, &FT4_DOWNSAMPLE)`
/// with two differences the caller must know about:
///
/// - **Un-normalised**, like every other stage in `engine::dsp`. The
///   RMS normalisation `process_candidate_basic_impl` and
///   `refine_candidate_position` apply is the caller's, and on this
///   path it is not optional — see the module doc.
/// - **Not bit-comparable** to the FFT path, and not intended to be: a
///   FIR passband is not a rectangular bin window with a raised-cosine
///   taper. What is reproduced is the *band*, and therefore the noise
///   power the normalisation divides by.
pub fn candidate_baseband(audio: &[i16], f0_hz: f32) -> Vec<Complex<f32>> {
    let mut ddc = CandidateDdc::new(f0_hz);
    let mut out = Vec::with_capacity(CD0_LEN);
    ddc.push_i16(audio, &mut out);
    ddc.flush_to(CD0_LEN, &mut out);
    out.resize(CD0_LEN, Complex::new(0.0, 0.0));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    fn tone(freq_hz: f32, amp: f32, n: usize) -> Vec<i16> {
        let w = 2.0 * core::f64::consts::PI * freq_hz as f64 / INPUT_RATE_HZ as f64;
        (0..n)
            .map(|k| (amp as f64 * (w * k as f64).cos()) as i16)
            .collect()
    }

    fn mean_power(cd0: &[Complex<f32>]) -> f32 {
        cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32
    }

    /// The band constants restate `FT4_DOWNSAMPLE`'s own fields; if
    /// that config is ever retuned, this fails rather than letting the
    /// two paths quietly filter different bands.
    #[test]
    #[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
    fn band_matches_the_reference_downsample() {
        let cfg = crate::ft4::decode::FT4_DOWNSAMPLE;
        assert_eq!(cfg.input_rate as f32, INPUT_RATE_HZ);
        assert_eq!(cfg.fft2_size, CD0_LEN);
        assert_eq!(cfg.ntones, Ft4::NTONES);
        assert_eq!(cfg.leading_pad_tones, LEADING_PAD_TONES);
        assert_eq!(cfg.trailing_pad_tones, TRAILING_PAD_TONES);
        assert_eq!(cfg.tone_spacing_hz, Ft4::TONE_SPACING_HZ);
        // The reference's own output rate: `input_rate · fft2/fft1`.
        let ref_rate = cfg.input_rate as f32 * cfg.fft2_size as f32 / cfg.fft1_size as f32;
        assert!(
            (ref_rate - DS_RATE_HZ).abs() < 0.01,
            "reference rate {ref_rate} != {DS_RATE_HZ}"
        );
    }

    #[test]
    fn band_geometry_is_the_asymmetric_one() {
        // [-31.25, +93.75] Hz around f0 — 1.5 tones below, 4.5 above.
        assert!((BAND_CENTER_OFFSET_HZ - 31.2495).abs() < 1e-3);
        assert!((BAND_HALF_WIDTH_HZ - 62.499).abs() < 1e-3);
        assert!((BAND_CENTER_OFFSET_HZ - BAND_HALF_WIDTH_HZ + 31.2495).abs() < 1e-2);
    }

    #[test]
    fn output_is_exactly_cd0_len() {
        let audio = tone(1500.0, 8000.0, 90_000);
        assert_eq!(candidate_baseband(&audio, 1500.0).len(), CD0_LEN);
        // Short input still yields a full-length, zero-padded buffer.
        assert_eq!(candidate_baseband(&audio[..12_000], 1500.0).len(), CD0_LEN);
    }

    /// A tone at the candidate frequency lands at DC: the baseband
    /// phase must barely rotate across the settled span.
    #[test]
    fn candidate_tone_lands_at_dc() {
        let f0 = 1500.0f32;
        let audio = tone(f0, 8000.0, 90_000);
        let cd0 = candidate_baseband(&audio, f0);

        let (a, b) = (1000usize, 4000usize);
        let ph = |k: usize| cd0[k].im.atan2(cd0[k].re);
        let mut d = ph(b) - ph(a);
        while d > core::f32::consts::PI {
            d -= 2.0 * core::f32::consts::PI;
        }
        while d < -core::f32::consts::PI {
            d += 2.0 * core::f32::consts::PI;
        }
        let hz = d / (2.0 * core::f32::consts::PI) * DS_RATE_HZ / (b - a) as f32;
        assert!(hz.abs() < 0.05, "candidate tone drifted {hz} Hz off DC");
    }

    /// Every FT4 tone — `f0` through `f0 + 3·Δf` — sits in the flat
    /// part of the passband, within 1 dB of each other. This is the
    /// property the LLR path depends on; a filter narrow enough to help
    /// the noise budget but narrow enough to tilt the tones would be a
    /// silent sensitivity loss.
    #[test]
    fn all_four_tones_pass_flat() {
        let f0 = 1500.0f32;
        let mut powers = vec![];
        for k in 0..Ft4::NTONES {
            let audio = tone(f0 + k as f32 * Ft4::TONE_SPACING_HZ, 8000.0, 90_000);
            let cd0 = candidate_baseband(&audio, f0);
            powers.push(mean_power(&cd0[1000..4000]));
        }
        let max = powers.iter().cloned().fold(f32::MIN, f32::max);
        let min = powers.iter().cloned().fold(f32::MAX, f32::min);
        let ripple_db = 10.0 * (max / min).log10();
        assert!(
            ripple_db < 1.0,
            "passband ripple {ripple_db} dB: {powers:?}"
        );
    }

    /// The reference zeroes everything outside `[-31.25, +93.75] Hz`.
    /// Check both edges: a tone 1.5 tone-spacings *below* `f0` is inside
    /// the band, one 3 tone-spacings below is not, and the same on the
    /// upper side past `f0 + 4.5·Δf`.
    #[test]
    fn out_of_band_tones_are_rejected() {
        let f0 = 1500.0f32;
        let p = |offset: f32| {
            let audio = tone(f0 + offset, 8000.0, 90_000);
            mean_power(&candidate_baseband(&audio, f0)[1000..4000])
        };
        let in_band = p(0.0);
        for offset in [-125.0f32, -100.0, 160.0, 200.0, 400.0, 700.0] {
            let rej_db = 10.0 * (in_band / p(offset)).log10();
            assert!(
                rej_db > 50.0,
                "tone at f0{offset:+} Hz only {rej_db:.1} dB down"
            );
        }
    }

    /// Equivalent noise bandwidth, this chain against the reference —
    /// the one number the RMS normalisation actually sees, and the
    /// module doc's whole passband argument.
    ///
    /// The two paths carry unrelated absolute gains (the FFT path's
    /// `1/sqrt(fft1·fft2)`, this one's filter DC gain), so a raw power
    /// ratio says nothing. ENBW divides that gain out: for a filter
    /// `H`, white noise of variance `σ²` gives output power
    /// `σ²·∫|H|²df/Fs` while a tone at the band centre gives
    /// `|H(fc)|²·A²/2`, so `P_noise/P_tone` is `ENBW/Fs` scaled by the
    /// same `σ²/(A²/2)` on both paths. Feed both paths the same noise
    /// and the same tone, and the ratio of ratios is the bandwidth
    /// ratio alone.
    #[test]
    #[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
    fn noise_bandwidth_matches_the_reference_band() {
        use crate::engine::dsp::downsample::{build_fft_cache, downsample_cached};
        use crate::ft4::decode::FT4_DOWNSAMPLE;

        let f0 = 1500.0f32;

        // Deterministic white noise — the signal-free case the
        // normalisation is calibrated on.
        let mut state = 0x1234_5678u32;
        let noise: Vec<i16> = (0..90_000)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                ((state >> 16) as i16 as f32 * 0.1) as i16
            })
            .collect();
        // Tone at the band centre, where both paths are flat.
        let probe = tone(f0 + BAND_CENTER_OFFSET_HZ, 8000.0, 90_000);

        let path_ratio = |audio_n: &[i16], audio_t: &[i16], ddc_path: bool| -> f32 {
            let (pn, pt) = if ddc_path {
                (
                    mean_power(&candidate_baseband(audio_n, f0)[1000..4000]),
                    mean_power(&candidate_baseband(audio_t, f0)[1000..4000]),
                )
            } else {
                let cn = build_fft_cache(audio_n, &FT4_DOWNSAMPLE);
                let ct = build_fft_cache(audio_t, &FT4_DOWNSAMPLE);
                (
                    mean_power(&downsample_cached(&cn, f0, &FT4_DOWNSAMPLE)[1000..4000]),
                    mean_power(&downsample_cached(&ct, f0, &FT4_DOWNSAMPLE)[1000..4000]),
                )
            };
            pn / pt
        };

        let ddc_enbw = path_ratio(&noise, &probe, true);
        let ref_enbw = path_ratio(&noise, &probe, false);
        let ratio = ddc_enbw / ref_enbw;
        let db = 10.0 * ratio.log10();
        // Measured **+0.021 dB** (DDC 2.161e-3, reference 2.150e-3) at
        // the tap counts above — the two bands integrate to the same
        // noise power to within half a percent, which is the whole
        // claim the `fc`/`ntaps` choices make. ±1 dB of slack, so a
        // deliberate retune has room to move without silently
        // rescaling every LLR; anything past that is a filter admitting
        // materially different noise than the reference, and the RMS
        // normalisation would pass it straight into `LLR_SCALE`.
        assert!(
            db.abs() < 1.0,
            "DDC noise bandwidth is {db:+.2} dB against the reference band \
             (ddc {ddc_enbw:.4e}, ref {ref_enbw:.4e})"
        );
    }
}
