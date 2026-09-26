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

impl ToneRefs {
    /// The references for a signal whose lowest tone is at `-shift_hz`, so that correlating them
    /// with the *unshifted* baseband gives what [`crate::jtty::dsp::shift_frequency`]`(x, shift_hz)`
    /// followed by [`correlate_payload`] does, without materialising the shifted window (#499):
    /// `conj(ref_k)·exp(jφn)` is itself a tone, `exp(j(φ − 2πk/nss)n)` with `φ = 2π·shift/fs`, and
    /// the shift's phase at a symbol's first sample is one factor per symbol.
    pub fn rotated(&self, shift_hz: f32, fs: f32) -> RotatedRefs {
        let phi = f64::from(shift_hz) * TAU / f64::from(fs);
        let nss = self.nss;
        let rot = core::array::from_fn(|k| {
            let psi = phi - TAU * k as f64 / nss as f64;
            let step = Complex32::new(psi.cos() as f32, psi.sin() as f32);
            let mut w = Complex32::new(1.0, 0.0);
            (0..nss)
                .map(|_| {
                    let now = w;
                    w *= step;
                    now
                })
                .collect()
        });
        RotatedRefs { nss, phi, rot }
    }
}

/// [`ToneRefs`] rotated by a candidate's frequency: see [`ToneRefs::rotated`].
pub struct RotatedRefs {
    nss: usize,
    /// The shift, radians per sample.
    phi: f64,
    /// `[tone][sample]`.
    rot: [Vec<Complex32>; 4],
}

impl RotatedRefs {
    /// `|Σ rot_k·x|²` over one symbol starting at `x[0]`: the shift's phase drops out of a power.
    pub fn power(&self, k: usize, x: &[Complex32]) -> f32 {
        self.rot[k]
            .iter()
            .zip(x)
            .fold(Complex32::new(0.0, 0.0), |acc, (&r, &x)| acc + r * x)
            .norm_sqr()
    }

    /// [`correlate_payload`] on the unshifted `samples`.
    pub fn correlate_payload(
        &self,
        samples: &[Complex32],
        payload_start: usize,
    ) -> (Correlations, Correlations) {
        let zero = [[Complex32::new(0.0, 0.0); 4]; INFO_BITS];
        let (mut zsym, mut zhalf) = (zero, zero);
        let nss = self.nss;
        let half = nss / 2;
        let available = samples.len().saturating_sub(payload_start) / nss;
        // the shift's phase at the first sample of each symbol, `exp(jφ(i0+1))`
        let a = self.phi * (payload_start + 1) as f64;
        let mut turn = Complex32::new(a.cos() as f32, a.sin() as f32);
        let per_symbol = self.phi * nss as f64;
        let step = Complex32::new(per_symbol.cos() as f32, per_symbol.sin() as f32);
        let dot = |r: &[Complex32], x: &[Complex32]| {
            r.iter()
                .zip(x)
                .fold(Complex32::new(0.0, 0.0), |acc, (&r, &x)| acc + r * x)
        };
        for s in 0..INFO_BITS.min(available) {
            let x = &samples[payload_start + s * nss..payload_start + (s + 1) * nss];
            for k in 0..4 {
                let r = &self.rot[k];
                let (h1, h2) = (dot(&r[..half], &x[..half]), dot(&r[half..], &x[half..]));
                zsym[s][k] = (h1 + h2) * turn;
                let e = f64::from(h1.norm_sqr()) + f64::from(h2.norm_sqr());
                zhalf[s][k] = Complex32::new(e.sqrt() as f32, 0.0);
            }
            turn *= step;
        }
        (zsym, zhalf)
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

    /// Rotating the references gives what shifting the signal and correlating does, on noise, at
    /// a frequency with a large phase per sample.
    #[test]
    fn rotated_references_agree_with_shifting_the_signal() {
        use crate::jtty::dsp::shift_frequency;
        let nss = 192;
        let mut s = 0x1234_5678u64;
        let mut rnd = move || {
            s = s
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((s >> 40) as f32 / (1u64 << 24) as f32) - 0.5
        };
        let x: Vec<Complex32> = (0..14_160).map(|_| Complex32::new(rnd(), rnd())).collect();
        let refs = ToneRefs::new(nss);
        for (shift, start) in [(-1503.7f32, 2496usize), (-2810.2, 2500), (-1200.0, 3001)] {
            let mut y = vec![Complex32::new(0.0, 0.0); x.len()];
            shift_frequency(&x, &mut y, 6000.0, shift);
            let (zs, zh) = correlate_payload(&refs, &y, start);
            let rot = refs.rotated(shift, 6000.0);
            let (rs, rh) = rot.correlate_payload(&x, start);
            for s in 0..INFO_BITS {
                for k in 0..4 {
                    let scale = zs[s][k].norm().max(1.0);
                    assert!(
                        (zs[s][k] - rs[s][k]).norm() < 2e-3 * scale.max(10.0),
                        "sym {s} tone {k}: {} vs {}",
                        zs[s][k],
                        rs[s][k]
                    );
                    assert!((zh[s][k].re - rh[s][k].re).abs() < 2e-3 * zh[s][k].re.max(10.0));
                }
            }
            // and a gate power against the shifted signal's
            let g = refs
                .conj(2)
                .iter()
                .zip(&y[start..start + nss])
                .fold(Complex32::new(0.0, 0.0), |a, (&r, &v)| a + r * v)
                .norm_sqr();
            let gr = rot.power(2, &x[start..start + nss]);
            assert!((g - gr).abs() < 2e-3 * g.max(100.0), "{g} vs {gr}");
        }
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
