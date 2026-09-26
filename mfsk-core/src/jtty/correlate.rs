//! Symbol correlation of the JTTY data section.
//!
//! Ported from WSJT-X `lib/jtty/jtty_payload_correlators.f90`
//! (`jtty_correlate_payload_symbols`), tag `v3.2.0-rc1`.
//!
//! For each of the 46 data symbols, the complex correlation of the received
//! baseband (frequency-shifted so the signal's lowest tone sits at 0 Hz) with
//! each of the four tone references, and the *half-symbol* energies — the two
//! half-symbol correlations combined without their phase, `√(|h₁|² + |h₂|²)` —
//! which the ladder's last rung uses when a frequency error inside the symbol
//! has smeared the full-symbol sum.

use alloc::vec::Vec;
use core::f64::consts::TAU;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::INFO_BITS;
use super::trellis::Correlations;

/// The four tone references for a symbol of `nss` samples: tone `k` is
/// `exp(+j·2π·k·n/nss)`, so one symbol carries exactly `k` turns.
pub struct ToneRefs {
    nss: usize,
    /// `[tone][sample]`, already conjugated for use in a correlation.
    conj: [Vec<Complex32>; 4],
}

impl ToneRefs {
    /// References for `nss` samples per symbol (192 at 6 kHz). `nss` must be even.
    pub fn new(nss: usize) -> Self {
        assert!(
            nss > 0 && nss.is_multiple_of(2),
            "symbol length must be even"
        );
        let conj = core::array::from_fn(|k| {
            (0..nss)
                .map(|n| {
                    let phase = TAU * (k * n) as f64 / nss as f64;
                    Complex32::new(phase.cos() as f32, -(phase.sin() as f32))
                })
                .collect()
        });
        Self { nss, conj }
    }

    /// Samples per symbol.
    pub fn nss(&self) -> usize {
        self.nss
    }

    /// The reference of tone `k`, conjugated.
    pub fn conj(&self, k: usize) -> &[Complex32] {
        &self.conj[k]
    }

    /// `Σ conj(ref_k)·x` over one symbol starting at `x[0]`, in two halves.
    fn halves(&self, k: usize, x: &[Complex32]) -> (Complex32, Complex32) {
        let half = self.nss / 2;
        let dot = |r: &[Complex32], x: &[Complex32]| {
            r.iter()
                .zip(x)
                .fold(Complex32::new(0.0, 0.0), |acc, (&r, &x)| acc + r * x)
        };
        let r = &self.conj[k];
        (
            dot(&r[..half], &x[..half]),
            dot(&r[half..], &x[half..self.nss]),
        )
    }
}

/// Correlate the 46 data symbols that begin at `payload_start` in `samples`.
///
/// Symbols that do not fit entirely in `samples` are left zero
/// (`available_symbols`).
pub fn correlate_payload(
    refs: &ToneRefs,
    samples: &[Complex32],
    payload_start: usize,
) -> (Correlations, Correlations) {
    let zero = [[Complex32::new(0.0, 0.0); 4]; INFO_BITS];
    let (mut zsym, mut zhalf) = (zero, zero);
    let nss = refs.nss;
    let available = samples.len().saturating_sub(payload_start) / nss;
    for s in 0..INFO_BITS.min(available) {
        let x = &samples[payload_start + s * nss..payload_start + (s + 1) * nss];
        for k in 0..4 {
            let (h1, h2) = refs.halves(k, x);
            zsym[s][k] = h1 + h2;
            // energy accumulated in f64, as upstream
            let e = f64::from(h1.norm_sqr()) + f64::from(h2.norm_sqr());
            zhalf[s][k] = Complex32::new(e.sqrt() as f32, 0.0);
        }
    }
    (zsym, zhalf)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jtty::{crc, tbcc};

    /// A noiseless baseband frame of pure tones (no Gaussian shaping, so the
    /// correlations are exactly one symbol of energy in the sent tone).
    fn baseband(tones: &[u8], nss: usize) -> Vec<Complex32> {
        tones
            .iter()
            .flat_map(|&t| {
                (0..nss).map(move |n| {
                    let ph = TAU * (usize::from(t) * n) as f64 / nss as f64;
                    Complex32::new(ph.cos() as f32, ph.sin() as f32)
                })
            })
            .collect()
    }

    #[test]
    fn sent_tone_collects_a_full_symbol_and_the_others_nothing() {
        let nss = 192;
        let info = crc::append(&[1u8; 34].map(|b| b));
        let tones = tbcc::encode(&info);
        let refs = ToneRefs::new(nss);
        let (zs, zh) = correlate_payload(&refs, &baseband(&tones, nss), 0);
        for (s, &t) in tones.iter().enumerate() {
            for k in 0..4 {
                let (m, h) = (zs[s][k].norm(), zh[s][k].re);
                if k == usize::from(t) {
                    assert!((m - nss as f32).abs() < 1e-2, "sym {s}: {m}");
                    // each half collects nss/2, so √(2·(nss/2)²) = nss/√2
                    assert!((h - nss as f32 / 2f32.sqrt()).abs() < 1e-2, "sym {s}: {h}");
                } else {
                    // orthogonal over the whole symbol; over a half symbol the
                    // tones are not, but the sent one still collects far more
                    assert!(m < 1e-2, "sym {s} tone {k}: {m}");
                    assert!(h < 0.7 * zh[s][usize::from(t)].re, "sym {s} tone {k}: {h}");
                }
            }
        }
    }

    #[test]
    fn symbols_that_do_not_fit_are_zero() {
        let nss = 192;
        let refs = ToneRefs::new(nss);
        let x = baseband(&[0; 10], nss);
        let (zs, _) = correlate_payload(&refs, &x, 3 * nss);
        assert!(zs[6][0].norm() > 100.0);
        assert!(
            zs[7].iter().all(|z| z.norm() == 0.0),
            "only 7 symbols remain"
        );
    }
}
