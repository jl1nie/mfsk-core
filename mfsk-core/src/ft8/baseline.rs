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
