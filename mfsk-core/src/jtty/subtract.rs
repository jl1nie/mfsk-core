//! Subtracting a decoded frame from the receiver's analytic signal.
//!
//! Ported from WSJT-X `lib/jtty/subtract_jtty.f90`, tag `v3.2.0-rc1`, which is
//! modelled on `subtractft8.f90`. The frame is re-encoded from its *decoded*
//! (error-corrected) payload, so the reference is the transmitted waveform, not
//! what was received. Its complex gain — amplitude and phase, slowly varying
//! with fading — is estimated by low-pass filtering `c0 · conj(reference)` over
//! the frame, and `gain · reference` is taken off `c0`. `c0` is already analytic,
//! so the full complex estimate is subtracted (no real part, no factor of two).
//!
//! **The filter** is a `cos²` window two symbols wide (385 taps at 192 samples
//! per symbol), normalised to unit sum. Upstream applies it as a circular
//! convolution through a 32 768-point FFT; the transform is zero outside the
//! frame and much longer than frame plus kernel, so nothing wraps and it is the
//! same as the linear convolution. Since `cos²(πj/N) = (1 + cos(2πj/N))/2`, this
//! is done here without an FFT and in one pass: the window is a plain moving sum
//! plus a moving sum of the samples times `exp(−j2πm/N)`, both taken from
//! prefix sums (in `f64`), so a subtraction costs `O(frame)` instead of
//! `O(frame · taps)`.
//!
//! Near the ends of the frame the window is cut off by the zeros outside it, so
//! the gain there is estimated low; that is upstream's behaviour too.

use alloc::vec::Vec;
use core::f64::consts::TAU;

use num_complex::{Complex32, Complex64};
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::tx::{Synth, synth_complex};
use super::{FRAME_SYMBOLS, NSPS};

/// Samples per symbol of the analytic signal (6 kHz).
const NSS: usize = NSPS / 2;
/// Sample rate of the analytic signal, Hz.
const FS6: f32 = 6_000.0;
/// Filter width: two symbols (`nfilt = 2·nss`).
const NFILT: usize = 2 * NSS;
const HALF: usize = NFILT / 2;

/// Subtract the frame whose 59 channel tones are `tones` (13 sync + 46 data), at
/// `f1_hz` (its lowest tone) and starting `xdt_s` seconds into `c0`, from `c0`.
/// Samples of the frame that fall outside `c0` are ignored.
///
/// All in `f32`, as [`subtract_frame_f64`] is in `f64`: the reference comes from
/// [`super::tx::Synth`] (an integer carrier oscillator and a wrapped `f32` modulation
/// phase), and the `cos²` filter is three sliding sums over `f32` phasor tables — which
/// cost nothing on the host and are about 1 800 ms against a few milliseconds on an
/// Xtensa LX7, where the `f64` version's 50 000 `sin`/`cos` are software. Agrees with
/// the `f64` form to `1e-3` of the frame's amplitude per sample (tested), and decodes the
/// same recordings (`tests/jtty_rx.rs`).
pub fn subtract_frame(c0: &mut [Complex32], tones: &[u8; FRAME_SYMBOLS], f1_hz: f32, xdt_s: f32) {
    let nframe = FRAME_SYMBOLS * NSS;
    let mut cref = alloc::vec![Complex32::new(0.0, 0.0); nframe];
    Synth::<f32>::at(tones, f1_hz, 1.0, NSS, FS6).fill_complex(&mut cref);
    let nstart = (xdt_s * FS6).round() as isize;
    let len = c0.len();
    let at = |i: usize| -> Option<usize> {
        let j = nstart + i as isize;
        (j >= 0 && (j as usize) < len).then_some(j as usize)
    };

    // camp = c0 · conj(reference) over the frame; zero where c0 does not reach
    let camp: Vec<Complex32> = (0..nframe)
        .map(|i| at(i).map_or(Complex32::new(0.0, 0.0), |j| c0[j] * cref[i].conj()))
        .collect();

    // The window is w(j) = (1 + cos(2πj/N))/2 over j = −N/2..N/2 and cos = (e^{+jθ} + e^{−jθ})/2, so
    // gain(i) = ½ (Σ camp + ½ (e^{jθ_i} S⁻ + e^{−jθ_i} S⁺)) / Σw with S∓ = Σ camp[m] e^{∓j2πm/N}
    // over the window, and Σw = N/2. The phasors have period N, so one table of them serves.
    let rot: Vec<Complex32> = (0..NFILT)
        .map(|m| Complex32::from_polar(1.0, -(TAU * m as f64 / NFILT as f64) as f32))
        .collect();
    // sums over [lo, hi), recomputed outright every `RESYNC` samples so the sliding update
    // (add the sample that enters, take off the one that leaves) cannot drift
    const RESYNC: usize = 256;
    let window = |i: usize| (i.saturating_sub(HALF), (i + HALF + 1).min(nframe));
    let sums = |lo: usize, hi: usize| {
        camp[lo..hi].iter().enumerate().fold(
            (
                Complex32::new(0.0, 0.0),
                Complex32::new(0.0, 0.0),
                Complex32::new(0.0, 0.0),
            ),
            |(b, m, p), (k, &c)| {
                let r = rot[(lo + k) % NFILT];
                (b + c, m + c * r, p + c * r.conj())
            },
        )
    };
    let (mut lo, mut hi) = window(0);
    let (mut boxs, mut minus, mut plus) = sums(lo, hi);
    for i in 0..nframe {
        if i % RESYNC == 0 && i > 0 {
            let (l, h) = window(i);
            (lo, hi) = (l, h);
            (boxs, minus, plus) = sums(lo, hi);
        } else if i > 0 {
            let (l, h) = window(i);
            if h > hi {
                let r = rot[(h - 1) % NFILT];
                let c = camp[h - 1];
                boxs += c;
                minus += c * r;
                plus += c * r.conj();
            }
            if l > lo {
                let r = rot[lo % NFILT];
                let c = camp[lo];
                boxs -= c;
                minus -= c * r;
                plus -= c * r.conj();
            }
            (lo, hi) = (l, h);
        }
        let e = rot[i % NFILT].conj(); // e^{jθ_i}
        let cos_sum = (e * minus + e.conj() * plus) * 0.5;
        let gain = (boxs + cos_sum) * (0.5 / (NFILT / 2) as f32);
        if let Some(j) = at(i) {
            c0[j] -= gain * cref[i];
        }
    }
}

/// The `f64` form of [`subtract_frame`]: prefix sums in `Complex64`, a `sin`/`cos` per sample for
/// the reference and the phasors. The reference for the tests, and what `subtract_frame` was
/// before it was moved to `f32`.
#[doc(hidden)]
pub fn subtract_frame_f64(
    c0: &mut [Complex32],
    tones: &[u8; FRAME_SYMBOLS],
    f1_hz: f32,
    xdt_s: f32,
) {
    let cref = synth_complex(tones, f1_hz, NSS, FS6);
    let nframe = cref.len();
    let nstart = (xdt_s * FS6).round() as isize;
    let len = c0.len();
    let at = |i: usize| -> Option<usize> {
        let j = nstart + i as isize;
        (j >= 0 && (j as usize) < len).then_some(j as usize)
    };

    // camp = c0 · conj(reference) over the frame; zero where c0 does not reach
    let camp: Vec<Complex64> = (0..nframe)
        .map(|i| {
            at(i).map_or(Complex64::new(0.0, 0.0), |j| {
                let z = c0[j] * cref[i].conj();
                Complex64::new(f64::from(z.re), f64::from(z.im))
            })
        })
        .collect();

    // Prefix sums of camp and of camp · exp(∓j2πm/N). The window is
    // w(j) = (1 + cos(2πj/N))/2 and cos = (e^{+jθ} + e^{−jθ})/2, so both signs are
    // needed: camp is complex, and the sin part of a single exponential does not
    // cancel unless camp is constant.
    let rot =
        |m: usize, sign: f64| Complex64::from_polar(1.0, -sign * TAU * m as f64 / NFILT as f64);
    let mut plain = alloc::vec![Complex64::new(0.0, 0.0); nframe + 1];
    let mut minus = alloc::vec![Complex64::new(0.0, 0.0); nframe + 1];
    let mut plus = alloc::vec![Complex64::new(0.0, 0.0); nframe + 1];
    for (m, &c) in camp.iter().enumerate() {
        plain[m + 1] = plain[m] + c;
        minus[m + 1] = minus[m] + c * rot(m, 1.0);
        plus[m + 1] = plus[m] + c * rot(m, -1.0);
    }
    // Σ_j cos²(πj/N) over j = −N/2..N/2
    let sumw: f64 = (0..=NFILT)
        .map(|k| {
            (core::f64::consts::PI * (k as f64 - HALF as f64) / NFILT as f64)
                .cos()
                .powi(2)
        })
        .sum();

    for i in 0..nframe {
        let (lo, hi) = (i.saturating_sub(HALF), (i + HALF + 1).min(nframe));
        let box_sum = plain[hi] - plain[lo];
        let theta = TAU * i as f64 / NFILT as f64;
        // Σ camp[m] cos(2π(i−m)/N)
        let cos_sum = (Complex64::from_polar(1.0, theta) * (minus[hi] - minus[lo])
            + Complex64::from_polar(1.0, -theta) * (plus[hi] - plus[lo]))
            * 0.5;
        let gain = (box_sum + cos_sum) * 0.5 / sumw;
        if let Some(j) = at(i) {
            let g = Complex32::new(gain.re as f32, gain.im as f32);
            c0[j] -= g * cref[i];
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jtty::source::{Atom, CallAction};
    use crate::jtty::tx;

    fn frame_tones() -> [u8; FRAME_SYMBOLS] {
        let t = tx::tones(&[Atom::call(CallAction::Cq, "K1ABC")]).unwrap();
        core::array::from_fn(|i| t[i])
    }

    fn power(x: &[Complex32]) -> f64 {
        x.iter().map(|z| f64::from(z.norm_sqr())).sum::<f64>() / x.len() as f64
    }

    /// A frame of amplitude `a` and phase `phi` at `f1`, `start` samples in.
    fn signal(
        tones: &[u8; FRAME_SYMBOLS],
        f1: f32,
        a: f32,
        phi: f32,
        start: usize,
        len: usize,
    ) -> Vec<Complex32> {
        let r = synth_complex(tones, f1, NSS, FS6);
        let rot = Complex32::from_polar(a, phi);
        let mut c = alloc::vec![Complex32::new(0.0, 0.0); len];
        r.iter()
            .enumerate()
            .for_each(|(i, &x)| c[start + i] = rot * x);
        c
    }

    #[test]
    fn a_frame_is_removed_from_its_middle_almost_completely() {
        let tones = frame_tones();
        let mut c0 = signal(&tones, 1500.0, 0.3, 1.1, 600, 14_160);
        let (lo, hi) = (600 + 2 * NSS, 600 + (FRAME_SYMBOLS - 2) * NSS);
        let before = power(&c0[lo..hi]);
        subtract_frame(&mut c0, &tones, 1500.0, 600.0 / FS6);
        let after = power(&c0[lo..hi]);
        assert!(
            after < before * 1e-5,
            "residual {:.1} dB",
            10.0 * (after / before).log10()
        );
        // nothing outside the frame is touched
        assert!(c0[..600].iter().all(|z| z.norm() == 0.0));
        assert!(
            c0[600 + FRAME_SYMBOLS * NSS..]
                .iter()
                .all(|z| z.norm() == 0.0)
        );
    }

    #[test]
    fn a_slowly_fading_frame_is_still_mostly_removed() {
        let tones = frame_tones();
        let r = synth_complex(&tones, 1500.0, NSS, FS6);
        // amplitude falls 30 % and the phase turns half a radian over the frame
        let mut c0: Vec<Complex32> = (0..14_160).map(|_| Complex32::new(0.0, 0.0)).collect();
        for (i, &x) in r.iter().enumerate() {
            let t = i as f32 / r.len() as f32;
            c0[600 + i] = Complex32::from_polar(0.3 * (1.0 - 0.3 * t), 0.5 * t) * x;
        }
        let (lo, hi) = (600 + 2 * NSS, 600 + (FRAME_SYMBOLS - 2) * NSS);
        let before = power(&c0[lo..hi]);
        subtract_frame(&mut c0, &tones, 1500.0, 600.0 / FS6);
        let after = power(&c0[lo..hi]);
        assert!(
            after < before * 1e-2,
            "residual {:.1} dB",
            10.0 * (after / before).log10()
        );
    }

    #[test]
    fn a_weaker_signal_underneath_survives() {
        // strong frame at 1500 Hz, a 20 dB weaker one at 1540 Hz, overlapping in time
        let (strong, weak_tones) = (frame_tones(), {
            let t = tx::tones(&[Atom::call(CallAction::Call, "W9XYZ")]).unwrap();
            core::array::from_fn::<u8, FRAME_SYMBOLS, _>(|i| t[i])
        });
        let s = signal(&strong, 1500.0, 1.0, 0.3, 600, 14_160);
        let w = signal(&weak_tones, 1540.0, 0.1, 2.0, 900, 14_160);
        let mut c0: Vec<Complex32> = s.iter().zip(&w).map(|(a, b)| a + b).collect();
        subtract_frame(&mut c0, &strong, 1500.0, 600.0 / FS6);
        // What is left is the weak signal plus the strong one's estimation error, which
        // a signal 40 Hz away and 20 dB down cannot avoid leaking into (the filter is
        // about a tone spacing wide): the weak signal must still be there, whole.
        let (lo, hi) = (900 + 3 * NSS, 600 + (FRAME_SYMBOLS - 3) * NSS);
        let dot = c0[lo..hi]
            .iter()
            .zip(&w[lo..hi])
            .fold(Complex32::new(0.0, 0.0), |a, (x, y)| a + x * y.conj());
        let projection = dot / (power(&w[lo..hi]) as f32 * (hi - lo) as f32);
        let residual = power(&c0[lo..hi]) / power(&w[lo..hi]);
        // before: the strong frame is 100x (20 dB) the weak one in power
        assert!(
            residual < 3.0,
            "strong frame left over: {residual:.2}x the weak one"
        );
        assert!(
            projection.norm() > 0.6 && projection.norm() < 1.2,
            "weak signal scaled by {projection}"
        );
    }

    /// The prefix-sum filter against the plain convolution with the same window,
    /// on a signal whose gain varies (which a constant gain cannot tell apart).
    #[test]
    fn the_filter_is_the_cos_squared_convolution() {
        let tones = frame_tones();
        let cref = synth_complex(&tones, 1500.0, NSS, FS6);
        let nframe = cref.len();
        // a varying complex signal: noise-like, deterministic
        let mut x = 0x2545_F491_4F6C_DD1Du64;
        let mut rnd = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 11) as f64 / (1u64 << 53) as f64 - 0.5
        };
        let mut c0: Vec<Complex32> = (0..14_160)
            .map(|_| Complex32::new(rnd() as f32, rnd() as f32))
            .collect();
        let original = c0.clone();
        let start = 500usize;
        subtract_frame(&mut c0, &tones, 1500.0, start as f32 / FS6);

        // reference: gain(i) = Σ_j w(j) camp[i−j] / Σ w, camp zero outside the frame
        let camp: Vec<Complex64> = (0..nframe)
            .map(|i| {
                let z = original[start + i] * cref[i].conj();
                Complex64::new(f64::from(z.re), f64::from(z.im))
            })
            .collect();
        let w: Vec<f64> = (0..=NFILT)
            .map(|k| {
                (core::f64::consts::PI * (k as f64 - HALF as f64) / NFILT as f64)
                    .cos()
                    .powi(2)
            })
            .collect();
        let sumw: f64 = w.iter().sum();
        let mut worst = 0f64;
        for i in 0..nframe {
            let mut g = Complex64::new(0.0, 0.0);
            for (k, wk) in w.iter().enumerate() {
                let m = i as isize + HALF as isize - k as isize;
                if (0..nframe as isize).contains(&m) {
                    g += camp[m as usize] * (*wk / sumw);
                }
            }
            let want = original[start + i] - Complex32::new(g.re as f32, g.im as f32) * cref[i];
            worst = worst.max(f64::from((c0[start + i] - want).norm()));
        }
        assert!(worst < 1e-4, "worst deviation {worst}");
    }

    /// The `f32` subtraction against the `f64` one, on a varying signal and on a frame buried in one.
    #[test]
    fn the_f32_subtraction_agrees_with_the_f64_one() {
        let tones = frame_tones();
        let mut x = 0x9E37_79B9_7F4A_7C15u64;
        let mut rnd = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 11) as f64 / (1u64 << 53) as f64 - 0.5
        };
        for (f1, start, amp) in [
            (1500.0f32, 500usize, 0.0f32),
            (1230.5, 1400, 0.3),
            (1780.25, 100, 2.0),
        ] {
            let mut c0: Vec<Complex32> = (0..14_160)
                .map(|_| Complex32::new(rnd() as f32 * 0.1, rnd() as f32 * 0.1))
                .collect();
            let s = signal(&tones, f1, amp, 0.7, start, 14_160);
            c0.iter_mut().zip(&s).for_each(|(a, b)| *a += *b);
            let mut a = c0.clone();
            let mut b = c0.clone();
            subtract_frame_f64(&mut a, &tones, f1, start as f32 / FS6);
            subtract_frame(&mut b, &tones, f1, start as f32 / FS6);
            let worst = a
                .iter()
                .zip(&b)
                .map(|(p, q)| (p - q).norm())
                .fold(0.0, f32::max);
            let scale = amp.max(0.1);
            assert!(
                worst < 1e-3 * scale,
                "{f1} Hz amp {amp}: worst deviation {worst}"
            );
        }
    }

    #[test]
    fn a_frame_running_off_the_buffer_is_handled() {
        let tones = frame_tones();
        let mut c0 = signal(&tones, 1500.0, 0.3, 0.0, 0, 14_160);
        // shifted so the frame starts before the buffer and ends inside it
        c0.rotate_left(0);
        subtract_frame(&mut c0, &tones, 1500.0, -100.0 / FS6);
        subtract_frame(&mut c0, &tones, 1500.0, 13_000.0 / FS6); // runs off the end
        assert!(c0.iter().all(|z| z.re.is_finite() && z.im.is_finite()));
    }
}
