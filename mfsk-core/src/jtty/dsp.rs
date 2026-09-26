//! Signal-processing helpers of the JTTY receiver.
//!
//! Ported from WSJT-X `lib/jtty/ana64a.f90`, `gen_syncwave.f90`,
//! `lib/twkfreq.f90` and `lib/db.f90`, tag `v3.2.0-rc1`. The receiver works on a
//! complex analytic signal at 6 kHz (`fsample = 6000`, 192 samples per symbol).

use alloc::vec::Vec;
use core::f64::consts::TAU;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::{NSPS, SYNC, SYNC_SYMBOLS};
use crate::engine::fft::{FftPlanner, default_planner};

/// Sample rate of the analytic signal, Hz.
pub const FS6: f32 = 6_000.0;
/// Samples per symbol at [`FS6`].
pub const NSS: usize = NSPS / 2;
/// Symbol rate, baud.
pub const BAUD: f32 = FS6 / NSS as f32;

/// Run `f` with an FFT planner. On `std` builds each thread keeps one for its
/// lifetime, so plans (twiddle tables) are built once per thread rather than once
/// per window per task — planning was a large share of a window's cost, and
/// rayon's workers live as long as the pool.
pub fn with_planner<R>(f: impl FnOnce(&mut dyn FftPlanner) -> R) -> R {
    #[cfg(feature = "std")]
    {
        use core::cell::RefCell;
        std::thread_local! {
            static PLANNER: RefCell<alloc::boxed::Box<dyn FftPlanner>> = RefCell::new(default_planner());
        }
        PLANNER.with(|p| f(p.borrow_mut().as_mut()))
    }
    #[cfg(not(feature = "std"))]
    {
        f(default_planner().as_mut())
    }
}

/// `10·log10(x)`, or −99 when `x` is at or below `1.259e-10` (`db.f90`).
pub fn db(x: f32) -> f32 {
    if x > 1.259e-10 {
        10.0 * x.log10()
    } else {
        -99.0
    }
}

/// The complex analytic signal of real 12 kHz audio, at 6 kHz (`ana64a.f90`).
///
/// The audio is zero-padded to the next power of two `nana =
/// 2^nint(log₂ n + ½)`, transformed, its negative frequencies and everything
/// above 3 kHz cleared (DC halved), and the first `nana/2` bins transformed back
/// — which is the same signal sampled at half the rate. The result has `n/2`
/// samples; `i16` full scale maps to 1.
pub fn analytic_6k(audio: &[i16]) -> Vec<Complex32> {
    let n = audio.len();
    let nana = 1usize << (((n as f64).log2() + 0.5).round() as u32);
    let nfft2 = nana / 2;
    let fac = 2.0 / (32767.0 * nana as f32);
    let mut buf: Vec<Complex32> = audio
        .iter()
        .map(|&x| Complex32::new(fac * f32::from(x), 0.0))
        .chain(core::iter::repeat(Complex32::new(0.0, 0.0)))
        .take(nana)
        .collect();
    let (fwd, inv) = with_planner(|p| (p.plan_forward(nana), p.plan_inverse(nfft2)));
    fwd.process(&mut buf);
    buf[nfft2 / 2 + 1..nfft2].fill(Complex32::new(0.0, 0.0));
    buf[0] *= 0.5;
    buf.truncate(nfft2);
    inv.process(&mut buf);
    buf.truncate(n / 2);
    buf
}

/// The analytic signal of [`analytic_6k`] by a complex FIR filter instead of a whole-window
/// transform: a Kaiser-windowed low-pass (cut-off 1 500 Hz) modulated to +1 500 Hz passes
/// 0–3 000 Hz and stops the negative frequencies, and every other output is kept. No
/// 32 768-point transform, which `esp-dsp` (at most 8 192 points) cannot do, and nothing that
/// needs more than the samples around each output, so it can run as the audio arrives (#499).
/// The edges see zeros outside the window, as the transform's zero padding does.
pub struct AnalyticFir {
    /// `h[m + M]` for `m` in `−M..=M`, scale included.
    taps: Vec<Complex32>,
}

impl AnalyticFir {
    /// `2M + 1` taps, Kaiser `beta`.
    pub fn new(half: usize, beta: f64) -> Self {
        let i0 = |x: f64| {
            // modified Bessel function of the first kind, order 0
            let (mut sum, mut term) = (1.0f64, 1.0f64);
            for k in 1..40 {
                term *= (x / 2.0) / k as f64;
                sum += term * term;
            }
            sum
        };
        let m = half as f64;
        let fc = 1500.0 / 12_000.0; // cycles a sample
        let w0 = TAU * 1500.0 / 12_000.0;
        let taps = (0..=2 * half)
            .map(|i| {
                let t = i as f64 - m;
                let sinc = if t == 0.0 {
                    2.0 * fc
                } else {
                    (TAU * fc * t).sin() / (core::f64::consts::PI * t)
                };
                let win = i0(beta * (1.0 - (t / m).powi(2)).max(0.0).sqrt()) / i0(beta);
                let g = 2.0 / 32_767.0 * sinc * win;
                Complex32::new((g * (w0 * t).cos()) as f32, (g * (w0 * t).sin()) as f32)
            })
            .collect();
        Self { taps }
    }

    /// The analytic signal of `audio` at 6 kHz: `audio.len() / 2` samples, output `k` centred on
    /// input `2k`.
    pub fn apply(&self, audio: &[i16]) -> Vec<Complex32> {
        let half = self.taps.len() / 2;
        let n = audio.len();
        (0..n / 2)
            .map(|k| {
                let c = 2 * k;
                let (lo, hi) = (c.saturating_sub(half), (c + half).min(n - 1));
                // y[c] = Σ h[m] x[c − m]
                (lo..=hi).fold(Complex32::new(0.0, 0.0), |acc, j| {
                    acc + self.taps[c + half - j] * f32::from(audio[j])
                })
            })
            .collect()
    }
}

/// The 13-symbol sync sequence as a complex baseband waveform at 6 kHz, tone 0
/// at 0 Hz, phase continuous across symbols (`gen_syncwave.f90`).
pub fn sync_wave() -> Vec<Complex32> {
    let step = |t: u8| TAU * f64::from(BAUD) * f64::from(t) / f64::from(FS6);
    (0..SYNC_SYMBOLS * NSS)
        .scan(0.0f64, |phase, i| {
            let now = *phase;
            *phase += step(SYNC[i / NSS]);
            Some(Complex32::new(now.cos() as f32, now.sin() as f32))
        })
        .collect()
}

/// Shift `c` by `shift_hz` (`twkfreq.f90` with no drift terms): sample `i` is
/// multiplied by `exp(j·2π·shift·(i+1)/fs)`, built as a running product in
/// single precision as upstream does. `out` must be at least as long as `c`.
pub fn shift_frequency(c: &[Complex32], out: &mut [Complex32], fs: f32, shift_hz: f32) {
    let dphi = shift_hz * (core::f32::consts::TAU / fs);
    let wstep = Complex32::new(dphi.cos(), dphi.sin());
    out.iter_mut()
        .zip(c)
        .scan(Complex32::new(1.0, 0.0), |w, (o, &x)| {
            *w *= wstep;
            Some((o, *w * x))
        })
        .for_each(|(o, v)| *o = v);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_wave_is_unit_amplitude_and_starts_at_phase_zero() {
        let w = sync_wave();
        assert_eq!(w.len(), 13 * NSS);
        assert!((w[0].re - 1.0).abs() < 1e-6 && w[0].im.abs() < 1e-6);
        assert!(w.iter().all(|z| (z.norm() - 1.0).abs() < 1e-5));
    }

    #[test]
    fn db_floor() {
        assert_eq!(db(0.0), -99.0);
        assert!((db(100.0) - 20.0).abs() < 1e-5);
    }

    #[test]
    fn analytic_signal_of_a_tone() {
        // A 1500 Hz tone of amplitude 0.5 full scale -> |z| ~ 0.5 at 6 kHz.
        let n = 28_320;
        let audio: Vec<i16> = (0..n)
            .map(|i| (16_383.0 * (TAU * 1500.0 * i as f64 / 12_000.0).sin()) as i16)
            .collect();
        let z = analytic_6k(&audio);
        assert_eq!(z.len(), n / 2);
        let mid = &z[2_000..12_000];
        let mean = mid.iter().map(|c| c.norm()).sum::<f32>() / mid.len() as f32;
        assert!((mean - 0.5).abs() < 0.01, "mean amplitude {mean}");
        // and it rotates at +1500 Hz: phase advance per sample = 2π·1500/6000
        let adv = (z[5_001] * z[5_000].conj()).arg();
        assert!((adv - (TAU * 1500.0 / 6000.0) as f32).abs() < 1e-3, "{adv}");
    }

    /// The FIR form agrees with the transform on a tone: amplitude, rotation and phase.
    #[test]
    fn fir_analytic_signal_matches_the_transform() {
        let n = 28_320;
        let fir = AnalyticFir::new(48, 7.0);
        for f in [400.0, 1500.0, 2600.0] {
            let audio: Vec<i16> = (0..n)
                .map(|i| (16_383.0 * (TAU * f * i as f64 / 12_000.0 + 0.3).sin()) as i16)
                .collect();
            let (a, b) = (analytic_6k(&audio), fir.apply(&audio));
            assert_eq!(a.len(), b.len());
            for k in 200..n / 2 - 200 {
                assert!(
                    (a[k] - b[k]).norm() < 0.01,
                    "{f} Hz, sample {k}: {} vs {}",
                    a[k],
                    b[k]
                );
            }
        }
        // negative frequencies are stopped: a tone's image at −f would beat against it
        let audio: Vec<i16> = (0..n)
            .map(|i| (16_383.0 * (TAU * 1500.0 * i as f64 / 12_000.0).cos()) as i16)
            .collect();
        let b = fir.apply(&audio);
        let spread = b[500..13_000]
            .iter()
            .map(|z| (z.norm() - 0.5).abs())
            .fold(0f32, f32::max);
        assert!(spread < 0.005, "{spread}");
    }

    #[test]
    fn shift_moves_a_tone_to_dc() {
        let c: Vec<Complex32> = (0..1000)
            .map(|i| {
                let p = TAU * 700.0 * i as f64 / 6000.0;
                Complex32::new(p.cos() as f32, p.sin() as f32)
            })
            .collect();
        let mut out = alloc::vec![Complex32::new(0.0, 0.0); 1000];
        shift_frequency(&c, &mut out, FS6, -700.0);
        let adv = (out[500] * out[499].conj()).arg();
        assert!(adv.abs() < 1e-3, "{adv}");
    }
}
