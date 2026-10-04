// SPDX-License-Identifier: GPL-3.0-only
//! Subtract a decoded JT65 signal from the audio so a weaker one under it
//! can be found: a port of `lib/subtract65.f90` (WSJT-X v3.2.0-rc1).
//!
//! The measured signal is `dd(t) = a(t)·cos(2πf0t + θ(t))`; the reference
//! `cref(t) = exp(j(2πf0t + φ(t)))` follows the decoded tones. The complex
//! amplitude is the low-passed product `dd·conj(cref)`, and
//! `dd -= 2·Re(cref·cfilt)`.
//!
//! The low-pass is upstream's `cos²(πj/1600)` window, 1601 taps,
//! normalised, applied zero-phase. Upstream does it as a 564 480-point FFT
//! convolution; since `cos² = ½ + ½cos`, the same filter is three running
//! sums, so it is done in O(n) in `f64` with no FFT (this module needs none,
//! and so builds wherever JT65 does).

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::sync_pattern::JT65_NPRC;

/// `NFILT` in `subtract65.f90`.
const NFILT: usize = 1600;

/// How a signal's symbols map to samples and tones.
#[derive(Clone, Copy, Debug)]
pub(super) struct Geometry {
    /// Samples per symbol (upstream: `ns = 4458`).
    pub samples_per_symbol: usize,
    /// Tone spacing in Hz (upstream: `2.6917`).
    pub tone_spacing_hz: f64,
}

/// `subtract65.f90`'s constants: the symbol is 4096/11025 s, 4458.23
/// samples at 12 kHz, called 4458; the tone spacing 2.6917 Hz.
pub(super) const WSJTX: Geometry = Geometry {
    samples_per_symbol: 4458,
    tone_spacing_hz: 2.6917,
};

/// Subtract the signal `tones` (126 channel tones: 0 = sync, 2..=65 data),
/// whose first symbol starts at sample `start` and whose sync tone is at
/// `f0_hz`, from `dd`.
pub(super) fn subtract65(dd: &mut [f32], start: usize, f0_hz: f32, tones: &[u8; 126], g: Geometry) {
    let ns = g.samples_per_symbol;
    let nref = 126 * ns;
    let two_pi = core::f64::consts::TAU;

    // Reference phasor and the demodulated product, per sample.
    let mut cref: Vec<(f64, f64)> = Vec::with_capacity(nref);
    let mut camp: Vec<(f64, f64)> = Vec::with_capacity(nref);
    let mut phi = 0.0f64;
    for k in 0..126 {
        let f = if JT65_NPRC[k] == 1 {
            f0_hz as f64
        } else {
            f0_hz as f64 + g.tone_spacing_hz * f64::from(tones[k])
        };
        let dphi = two_pi * f / 12_000.0;
        for _ in 0..ns {
            let (s, c) = phi.sin_cos();
            cref.push((c, s));
            let i = start + cref.len() - 1;
            camp.push(match dd.get(i) {
                // dd · conj(cref)
                Some(&v) => (f64::from(v) * c, -f64::from(v) * s),
                None => (0.0, 0.0),
            });
            phi += dphi;
            if phi >= two_pi {
                phi -= two_pi;
            }
        }
    }

    // Zero-phase smoothing by the normalised cos² window:
    // w[j] = (½ + ½cos(2πj/N)) / sum, j in -N/2..=N/2.
    let half = (NFILT / 2) as i64;
    let wsum: f64 = (-half..=half)
        .map(|j| {
            (core::f64::consts::PI * j as f64 / NFILT as f64)
                .cos()
                .powi(2)
        })
        .sum();
    // Prefix sums of x, x·e^{-i2πm/N}, x·e^{+i2πm/N}.
    let n = camp.len();
    let w = two_pi / NFILT as f64;
    let mut p0 = Vec::with_capacity(n + 1);
    let mut pm = Vec::with_capacity(n + 1);
    let mut pp = Vec::with_capacity(n + 1);
    let (mut a0, mut am, mut ap) = ((0.0, 0.0), (0.0, 0.0), (0.0, 0.0));
    p0.push(a0);
    pm.push(am);
    pp.push(ap);
    for (m, &(xr, xi)) in camp.iter().enumerate() {
        let (s, c) = (w * m as f64).sin_cos();
        a0 = (a0.0 + xr, a0.1 + xi);
        // x · (c - i s)
        am = (am.0 + xr * c + xi * s, am.1 + xi * c - xr * s);
        // x · (c + i s)
        ap = (ap.0 + xr * c - xi * s, ap.1 + xi * c + xr * s);
        p0.push(a0);
        pm.push(am);
        pp.push(ap);
    }
    for i in 0..n {
        let lo = (i as i64 - half).max(0) as usize;
        let hi = ((i as i64 + half) as usize).min(n - 1) + 1;
        let b0 = (p0[hi].0 - p0[lo].0, p0[hi].1 - p0[lo].1);
        let bm = (pm[hi].0 - pm[lo].0, pm[hi].1 - pm[lo].1);
        let bp = (pp[hi].0 - pp[lo].0, pp[hi].1 - pp[lo].1);
        // Σ_j cos(wj) x[i-j] = Re-part pair: ½(e^{iwi}·Σ x[m]e^{-iwm}
        //                                   + e^{-iwi}·Σ x[m]e^{+iwm}).
        let (s, c) = (w * i as f64).sin_cos();
        let up = (bm.0 * c - bm.1 * s, bm.0 * s + bm.1 * c);
        let dn = (bp.0 * c + bp.1 * s, bp.1 * c - bp.0 * s);
        let filt = (
            (0.5 * b0.0 + 0.25 * (up.0 + dn.0)) / wsum,
            (0.5 * b0.1 + 0.25 * (up.1 + dn.1)) / wsum,
        );
        // dd -= 2·Re(cfilt · cref)
        if let Some(d) = dd.get_mut(start + i) {
            let (cr, ci) = cref[i];
            *d -= (2.0 * (filt.0 * cr - filt.1 * ci)) as f32;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jt65::tx::{encode_channel_symbols, synthesize_standard};

    fn rms(a: &[f32]) -> f32 {
        (a.iter().map(|v| v * v).sum::<f32>() / a.len() as f32).sqrt()
    }

    /// A clean signal synthesised with the same geometry cancels to a small
    /// fraction of its level. (The crate's own synthesiser uses 4460
    /// samples a symbol and 12000/4460 Hz; WSJT-X's air signals the
    /// `WSJTX` geometry.)
    #[test]
    fn a_clean_signal_subtracts_to_a_residue() {
        let freq = 1270.0;
        let mut audio =
            synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let before = rms(&audio);
        let info = crate::msg::jt72::pack_standard("CQ", "K1ABC", "FN42").unwrap();
        let tones = encode_channel_symbols(&info);
        let g = Geometry {
            samples_per_symbol: 4460,
            tone_spacing_hz: 12_000.0 / 4460.0,
        };
        subtract65(&mut audio, 0, freq, &tones, g);
        let after = rms(&audio);
        assert!(after < 0.15 * before, "residue {after} vs {before}");
    }
}
