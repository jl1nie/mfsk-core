//! FFT-based analytic-signal (Hilbert transform) construction.
//!
//! Converts a real-valued signal (e.g. receiver audio) to its complex
//! analytic form by applying a fixed bandpass filter, then zeroing
//! the negative-frequency half of its spectrum and doubling the
//! positive half. This is how a real audio buffer becomes the
//! complex passband/baseband signal the MSK144 sync/matched-filter
//! code (`crate::msk144::sync`, `crate::engine::dsp::msk`) operates on
//! — mirrors WSJT-X `analytic()` (called from `mskrtd.f90`), though
//! this port lets the FFT size equal the input length rather than
//! fixing an oversized/zero-padded `NFFT1` — callers needing that
//! specific padding behaviour can zero-pad their input before
//! calling. The adaptive RX-equalizer correction WSJT-X layers on top
//! of the fixed filter (`beq`/`corr(i)`, trained by
//! `msk144signalquality.f90`) is not ported — this only applies the
//! always-on fixed filter, not the session-adaptive one.

use alloc::vec::Vec;
use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::super::fft::default_planner;
use super::msk::FS_HZ;

/// Center frequency of the fixed bandpass filter (`analytic.f90`'s
/// `f=ff-1500.0`).
const FILTER_CENTER_HZ: f32 = 1500.0;
/// Full-pass half-width: `(1-beta)/(2*T)` with `T=1/2000`, `beta=0.1`.
const FILTER_PASSBAND_HALF_HZ: f32 = 900.0;
/// Raised-cosine taper half-width: `(1+beta)/(2*T)`.
const FILTER_STOPBAND_HALF_HZ: f32 = 1_100.0;
/// Raised-cosine roll-off rate: `pi*T/beta`.
const FILTER_ROLLOFF_RATE: f32 = core::f32::consts::PI / 200.0;

/// WSJT-X's fixed raised-cosine bandpass gain at `freq_hz`
/// (`analytic.f90:26-43`'s `h(i)`): unity within ±900 Hz of 1500 Hz,
/// raised-cosine taper to zero by ±1100 Hz, zero beyond.
fn bandpass_gain(freq_hz: f32) -> f32 {
    let f = (freq_hz - FILTER_CENTER_HZ).abs();
    if f <= FILTER_PASSBAND_HALF_HZ {
        1.0
    } else if f <= FILTER_STOPBAND_HALF_HZ {
        0.5 * (1.0 + (FILTER_ROLLOFF_RATE * (f - FILTER_PASSBAND_HALF_HZ)).cos())
    } else {
        0.0
    }
}

/// Construct the analytic signal of a real-valued buffer via FFT:
/// apply the fixed 1500 Hz-centered bandpass filter, zero the
/// negative-frequency bins, double the positive-frequency bins (DC,
/// and Nyquist if `input.len()` is even, are left unscaled), then
/// inverse FFT and normalize. Assumes `input` was sampled at
/// [`FS_HZ`] — the only sample rate this crate's MSK144 pipeline
/// uses.
pub fn analytic_signal(input: &[f32]) -> Vec<Complex32> {
    let n = input.len();
    let mut planner = default_planner();
    let fwd = planner.plan_forward(n);
    let inv = planner.plan_inverse(n);

    let mut x: Vec<Complex32> = input.iter().map(|&v| Complex32::new(v, 0.0)).collect();
    fwd.process(&mut x);

    let df = FS_HZ / n as f32;
    let nyquist = if n.is_multiple_of(2) {
        Some(n / 2)
    } else {
        None
    };
    for (k, xv) in x.iter_mut().enumerate() {
        if k == 0 || Some(k) == nyquist {
            // DC / Nyquist: leave unscaled (but still filtered).
            *xv *= bandpass_gain(k as f32 * df);
        } else if k < n.div_ceil(2) {
            *xv *= 2.0 * bandpass_gain(k as f32 * df);
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
    /// frequency only, unit amplitude). `k0` is chosen to land at
    /// 1500 Hz (assuming [`FS_HZ`]), the center of the fixed bandpass
    /// filter's full-pass band, so the filter doesn't attenuate it.
    #[test]
    fn analytic_signal_of_pure_cosine_is_a_complex_exponential() {
        let n = 256;
        let k0 = 32;
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

    /// A tone well outside the fixed bandpass filter's stopband edge
    /// (±1100 Hz of 1500 Hz) should be almost entirely suppressed --
    /// this is the WSJT-X-equivalent `h(i)` filter (`analytic.f90:26-43`)
    /// this port now applies.
    #[test]
    fn analytic_signal_attenuates_out_of_band_tone() {
        let n = 256;
        let k0 = 64; // freq = 64 * (12000/256) = 3000 Hz, |3000-1500| = 1500 Hz > 1100 Hz stopband edge.
        let input: Vec<f32> = (0..n)
            .map(|i| (2.0 * core::f32::consts::PI * k0 as f32 * i as f32 / n as f32).cos())
            .collect();

        let analytic = analytic_signal(&input);
        let energy: f32 = analytic.iter().map(|a| a.norm_sqr()).sum();
        assert!(energy < 1e-4, "expected near-zero energy, got {energy}");
    }

    /// A tone right at the filter's center frequency should pass
    /// through with unit gain (sanity check on [`bandpass_gain`]
    /// directly, independent of FFT bin alignment).
    #[test]
    fn bandpass_gain_is_unity_at_center_and_zero_far_outside() {
        assert!((bandpass_gain(FILTER_CENTER_HZ) - 1.0).abs() < 1e-6);
        assert!((bandpass_gain(FILTER_CENTER_HZ - 500.0) - 1.0).abs() < 1e-6);
        assert_eq!(bandpass_gain(FILTER_CENTER_HZ + 2000.0), 0.0);
        assert_eq!(bandpass_gain(0.0), 0.0);
    }
}
