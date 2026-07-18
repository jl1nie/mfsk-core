//! FFT-based analytic-signal (Hilbert transform) construction.
//!
//! Converts a real-valued signal (e.g. receiver audio) to its complex
//! analytic form by zeroing the negative-frequency half of its
//! spectrum and doubling the positive half. This is how a real audio
//! buffer becomes the complex passband/baseband signal the MSK144
//! sync/matched-filter code (`crate::msk144::sync`,
//! `crate::core::dsp::msk`) operates on — mirrors WSJT-X `analytic()`
//! (called from `mskrtd.f90`), though this port lets the FFT size
//! equal the input length rather than fixing an oversized/zero-padded
//! `NFFT1` — callers needing that specific padding behaviour can
//! zero-pad their input before calling.

use alloc::vec::Vec;
use num_complex::Complex32;

use super::super::fft::default_planner;

/// Construct the analytic signal of a real-valued buffer via FFT:
/// zero the negative-frequency bins, double the positive-frequency
/// bins (DC, and Nyquist if `input.len()` is even, are left
/// unscaled), then inverse FFT and normalize.
pub fn analytic_signal(input: &[f32]) -> Vec<Complex32> {
    let n = input.len();
    let mut planner = default_planner();
    let fwd = planner.plan_forward(n);
    let inv = planner.plan_inverse(n);

    let mut x: Vec<Complex32> = input.iter().map(|&v| Complex32::new(v, 0.0)).collect();
    fwd.process(&mut x);

    let nyquist = if n.is_multiple_of(2) {
        Some(n / 2)
    } else {
        None
    };
    for (k, xv) in x.iter_mut().enumerate() {
        if k == 0 || Some(k) == nyquist {
            // DC / Nyquist: leave unscaled.
        } else if k < n.div_ceil(2) {
            *xv *= 2.0;
        } else {
            *xv = Complex32::new(0.0, 0.0);
        }
    }

    inv.process(&mut x);
    let scale = 1.0 / n as f32;
    for xv in x.iter_mut() {
        *xv *= scale;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real cosine at bin `k0` should produce an analytic signal
    /// whose real part matches the input and whose imaginary part is
    /// its Hilbert transform (a sine at the same frequency, 90 deg
    /// lagging) -- i.e. `analytic[n] ~= exp(i*2*pi*k0*n/N)` (positive
    /// frequency only, unit amplitude).
    #[test]
    fn analytic_signal_of_pure_cosine_is_a_complex_exponential() {
        let n = 256;
        let k0 = 10;
        let input: Vec<f32> = (0..n)
            .map(|i| (2.0 * core::f32::consts::PI * k0 as f32 * i as f32 / n as f32).cos())
            .collect();

        let analytic = analytic_signal(&input);
        for (i, a) in analytic.iter().enumerate() {
            let phase = 2.0 * core::f32::consts::PI * k0 as f32 * i as f32 / n as f32;
            let expected = Complex32::new(phase.cos(), phase.sin());
            assert!(
                (a - expected).norm() < 1e-3,
                "sample {i}: got {a:?}, expected {expected:?}"
            );
        }
    }
}
