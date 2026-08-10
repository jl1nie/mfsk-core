//! Spectrum baseline estimator — historical FT8 entry point.
//!
//! The polynomial-fit implementation moved to
//! [`crate::engine::baseline`] in 2026-05 (slice 1 of issue #18) so that
//! FT4's `coarse_sync` can normalise its candidate spectrum the same
//! way WSJT-X does (`ft4_baseline.f90`). The algorithm is identical
//! across FT8 / FT4 / FST4 in WSJT-X — keeping a single Rust port
//! avoids drift.
//!
//! This module is now a thin re-export plus the FT8-specific
//! [`avg_spectrum`] helper that consumes the embedded
//! [`crate::ft8::decode_block::Spectrogram`].

#![cfg(feature = "std")]

pub use crate::engine::baseline::fit_baseline;

/// Compute the average linear power per FFT bin from a `Spectrogram`.
/// `out.len()` must equal `spec.n_freq`. FT8-specific because it
/// targets the embedded `decode_block::Spectrogram` layout.
pub fn avg_spectrum(spec: &crate::ft8::decode_block::Spectrogram, out: &mut [f32]) {
    debug_assert_eq!(out.len(), spec.n_freq);
    out.fill(0.0);
    for t in 0..spec.n_time {
        for f in 0..spec.n_freq {
            #[allow(clippy::unnecessary_cast)]
            let v = spec.data[t * spec.n_freq + f] as f32;
            out[f] += v;
        }
    }
    let inv = 1.0 / spec.n_time as f32;
    for v in out.iter_mut() {
        *v *= inv;
    }
}

/// WSJT-X `get_spectrum_baseline.f90`-faithful power spectrum, purpose-
/// built for [`fit_baseline`]'s noise-floor estimate — **not**
/// [`crate::ft8::decode_block::compute_spectrogram`]'s rectangular
/// window, which is deliberately tuned so a signal's own tones don't
/// leak onto each other (see that function's doc comment) but is
/// undefended against far sidelobes bleeding in from *other*,
/// frequency-distant signals. On an isolated single signal that's
/// harmless; on a busy/crowded band (`qso3_busy.wav`, many concurrent
/// FT8 stations) it systematically inflates the "lower envelope"
/// `fit_baseline`'s percentile step reads the noise floor from,
/// pushing `xbase` up and `xsnr2` down.
///
/// WSJT-X never reuses `sync8.f90`'s own rectangular-window spectrum
/// for this — `get_spectrum_baseline.f90` builds a dedicated one with
/// a 4-term Nuttall window (`a0=0.3635819, a1=-0.4891775,
/// a2=0.1365995, a3=-0.0106411`) at 50% frame overlap, specifically
/// for its much lower far-sidelobe leakage. This is that pipeline:
/// full `NFFT_SPEC`-length frames (no zero-pad — unlike
/// `compute_spectrogram`'s NSPS-active-plus-zero-tail), `NFFT_SPEC/2`
/// hop. The window normalisation (`window/sum(window)*NSPS*2/300.0`,
/// `NSPS*2 == NFFT_SPEC`) folds in the same `1/300` linear pre-scale
/// `compute_spectrogram` applies separately, so raw `i16` samples feed
/// in directly below with no extra scale factor.
///
/// **Not averaged across frames.** `get_spectrum_baseline.f90`'s
/// `savg=savg+s(1:NH1,j)` inner loop is a *raw sum* over all `NF≈93`
/// frames — its own "Average spectrum" comment is misleading, there is
/// no `/NF` anywhere in the real subroutine. A first version of this
/// port added that missing-looking division, which is wrong: it made
/// every bin read `10*log10(93) ≈ 19.7 dB` too low (verified against a
/// real `jt9` run's own `SNRAUDIT_PROBE` instrumentation on a clean,
/// isolated synthetic signal, 2026-08-10 — real `sbase` was ~67.6 dB
/// flat across the band; the averaged version read ~49 dB, the
/// unaveraged one ~68-69 dB, within ~1-2 dB of ground truth).
///
/// **Must be paired with a matching `xsig`** computed from the WSJT-X
/// `cd0`/per-symbol-FFT pipeline (`fill_symbol_spectra`,
/// `ft8b.f90:154-161`), *not* from `compute_spectrogram`'s rectangular
/// spectrum — the `3e6`/`-27dB` calibration in `recompute_snr_xsnr2`
/// is fit to that specific pipeline pair, and the two mismatches don't
/// cancel when only one side is corrected (verified 2026-08-10).
pub fn compute_baseline_spectrum(audio: &[i16]) -> Vec<f32> {
    use crate::engine::fft::default_planner;
    use crate::ft8::decode_block::NFFT_SPEC;
    use crate::ft8::params::NMAX;
    use num_complex::Complex;

    const NST: usize = NFFT_SPEC / 2;
    let n_out = NFFT_SPEC / 2; // NH1, matches get_spectrum_baseline.f90

    let mut window = vec![0.0f32; NFFT_SPEC];
    let (a0, a1, a2, a3) = (0.3635819f32, -0.4891775f32, 0.1365995f32, -0.0106411f32);
    let n = NFFT_SPEC as f32;
    let two_pi = core::f32::consts::PI * 2.0;
    for (i, w) in window.iter_mut().enumerate() {
        let x = i as f32;
        *w = a0
            + a1 * (two_pi * x / n).cos()
            + a2 * (2.0 * two_pi * x / n).cos()
            + a3 * (3.0 * two_pi * x / n).cos();
    }
    let wsum: f32 = window.iter().sum();
    let norm = n / wsum / 300.0;
    for w in window.iter_mut() {
        *w *= norm;
    }

    let mut planner = default_planner();
    let fft = planner.plan_forward(NFFT_SPEC);
    let mut savg = vec![0.0f32; n_out];
    let mut buf = vec![Complex::new(0.0f32, 0.0f32); NFFT_SPEC];

    let n_scan = audio.len().min(NMAX);
    let mut ia = 0usize;
    while ia + NFFT_SPEC <= n_scan {
        for (k, c) in buf.iter_mut().enumerate() {
            *c = Complex::new(audio[ia + k] as f32 * window[k], 0.0);
        }
        fft.process(&mut buf);
        for i in 0..n_out {
            savg[i] += buf[i].norm_sqr();
        }
        ia += NST;
    }
    savg
}
