//! A phase accumulator that needs no `f64` per sample.
//!
//! The phase is a `u64` of turns (2^64 is one cycle), so adding to it is exact and wraps by itself. An
//! `f32` sum of `2 pi f / fs` drifts (the increment is off by ~3e-8 rad and the error is systematic:
//! 3e-3 rad over an FST4-15 frame, a degree over a long one, 0.04 rad over WSPR's 110 s), and an `f64`
//! one is software on an Xtensa LX7. The frequency becomes its increment once, in `f64`; a modulation
//! increment (small, an `f32` each) is rounded to turns one at a time, which does not accumulate. WSJT-X
//! v3.3.0-beta6 sums in `f64` (`waveform_kernels.f90`, `Modulator.cpp`'s `m_phi`); this agrees with it to
//! -88 dB or better (`tests/gen_wave_vs_wsjtx.rs`).

#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

/// 2^64, one turn.
const TURN: f64 = 18_446_744_073_709_551_616.0;

pub(crate) struct Nco {
    turns: u64,
    increment: u64,
}

impl Nco {
    /// Radians to turns: `2^64 / 2 pi`, exact enough in f32 (it is a scale, not a phase).
    const TURNS_PER_RAD: f32 = 18_446_744_073_709_551_616.0 / (2.0 * core::f32::consts::PI);

    /// At phase 0 with a carrier of `f_hz` at `sample_rate`.
    pub(crate) fn new(f_hz: f32, sample_rate: f32) -> Self {
        let mut nco = Self {
            turns: 0,
            increment: 0,
        };
        nco.set_carrier(f64::from(f_hz), sample_rate);
        nco
    }

    /// Change the carrier, keeping the phase (a CPFSK symbol change). The increment is the fraction of a
    /// turn per sample, `f / fs`, to 2^-64, computed in `f64` once per call. `f_hz` is an `f64` because an
    /// `f32` `f0 + tone * spacing` is off by up to 6e-5 Hz at 1500 Hz: 0.04 rad over WSPR's 110 s.
    pub(crate) fn set_carrier(&mut self, f_hz: f64, sample_rate: f32) {
        let frac = f_hz / f64::from(sample_rate);
        self.increment = ((frac - frac.floor()) * TURN) as u64;
    }

    /// The phase now, radians, from its top 32 bits: in `[0, 2 pi]` (a value within 128 of a full turn
    /// rounds to exactly `2 pi` in the `u32` to `f32` conversion; harmless to `sin` and `cos`).
    #[inline]
    pub(crate) fn phase(&self) -> f32 {
        ((self.turns >> 32) as u32) as f32 * (2.0 * core::f32::consts::PI / 4_294_967_296.0)
    }

    /// `sin` of the phase, without a libm call: the top two bits are the quadrant, the rest is the angle in
    /// the first, folded to `[0, pi/2]` and fed to the odd degree-9 minimax polynomial that WSJT-X
    /// v3.3.0-beta6 uses (`waveform_kernels.f90`, `emit_real`; unit linear coefficient).
    #[inline]
    pub(crate) fn sin(&self) -> f32 {
        sin_turns(self.turns)
    }

    /// `cos` of the phase, the same polynomial a quarter turn on (only [`Rotor`], FT8's down-converters, reads it).
    #[cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]
    #[inline]
    pub(crate) fn cos(&self) -> f32 {
        sin_turns(self.turns.wrapping_add(1 << 62))
    }

    /// Advance by one sample: the carrier and `modulation_rad`.
    #[inline]
    pub(crate) fn step(&mut self, modulation_rad: f32) {
        let m = (modulation_rad * Self::TURNS_PER_RAD) as i64 as u64;
        self.turns = self.turns.wrapping_add(self.increment).wrapping_add(m);
    }
}

/// The folded degree-9 polynomial sine of a phase in `u64` turns (see [`Nco::sin`]).
#[inline]
fn sin_turns(turns: u64) -> f32 {
    const HALF_PI_PER_UNIT: f32 = core::f32::consts::FRAC_PI_2 / 4_294_967_296.0;
    let quadrant = (turns >> 62) as u8;
    let a = (((turns << 2) >> 32) as u32) as f32 * HALF_PI_PER_UNIT;
    let x = if quadrant & 1 == 0 {
        a
    } else {
        core::f32::consts::FRAC_PI_2 - a
    };
    let x2 = x * x;
    let y = x
        + (x * x2)
            * (-1.666_665_7e-1
                + x2 * (8.333_017e-3 + x2 * (-1.980_661_5e-4 + x2 * 2.600_054_8e-6)));
    if quadrant >= 2 { -y } else { y }
}

/// `exp(-j 2 pi f n / fs)` for consecutive sample numbers `n`, with no `f64` per sample: the mixer of a down-converter.
///
/// The phase of sample `n` is `n * (f / fs)` turns in `u64`: an integer product that wraps at one turn, so it is exact for
/// whatever `n` is (negative included: the stream may start before sample 0) and can be read at any sample, which is what an
/// `f32` running phase cannot do. Every [`Rotor::REFRESH`] samples the phasor is taken exactly from that phase (the polynomial
/// sine, no libm); between them it is rotated by the per-sample step, two `f32` multiply-adds, whose rounding builds up only
/// until the next exact read. (The mixers this replaced took the exact phase in `f64`, `ft8::decode_block::cd0_fir`'s every
/// symbol, `band_bb`'s every 4096 or 64 samples: software `f64` trig on an LX7.)
#[cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]
pub(crate) struct Rotor {
    nco: Nco,
    /// `exp(-j w)`, one sample's rotation, unit length.
    step: (f32, f32),
    /// Next sample number, and what to hand out for it.
    n: i64,
    cur: (f32, f32),
    /// Samples until the next exact read.
    left: u32,
}

#[cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]
impl Rotor {
    /// Samples between exact reads of the phase: 64 rotations of an `f32` step good to ~5e-7 leave 3e-5 rad.
    pub(crate) const REFRESH: u32 = 64;

    /// A mixer at `f_hz` (`f64`: an `f32` carrier is off by up to 6e-5 Hz at 1500 Hz) whose first sample is number `n0`.
    pub(crate) fn new(f_hz: f64, sample_rate: f32, n0: i64) -> Self {
        let mut nco = Nco::new(0.0, sample_rate);
        nco.set_carrier(f_hz, sample_rate);
        let w = Nco {
            turns: nco.increment,
            increment: 0,
        };
        let (c, s) = (w.cos(), w.sin());
        let norm = (c * c + s * s).sqrt();
        Self {
            nco,
            step: (c / norm, -s / norm),
            n: n0,
            cur: (1.0, 0.0),
            left: 0,
        }
    }

    /// `exp(-j 2 pi f n / fs)` for the next `n`.
    #[inline]
    pub(crate) fn next(&mut self) -> (f32, f32) {
        if self.left == 0 {
            self.nco.turns = self.nco.increment.wrapping_mul(self.n as u64);
            self.cur = (self.nco.cos(), -self.nco.sin());
            self.left = Self::REFRESH;
        }
        let out = self.cur;
        self.cur = (
            out.0 * self.step.0 - out.1 * self.step.1,
            out.0 * self.step.1 + out.1 * self.step.0,
        );
        self.n += 1;
        self.left -= 1;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A carrier at an exact fraction of the sample rate returns to phase 0 after a whole number of
    /// cycles, however long: nothing accumulates.
    #[test]
    fn a_long_run_does_not_drift() {
        let mut nco = Nco::new(1500.0, 12_000.0); // 1/8 turn per sample
        for _ in 0..8 * 10_000_000u64 {
            nco.step(0.0);
        }
        assert_eq!(nco.phase(), 0.0);
    }

    /// The folded polynomial is a sine to ~1e-7 all round the circle, quadrant edges included.
    #[test]
    fn sin_is_a_sine() {
        let mut nco = Nco::new(1.0, 1_000_003.0); // ~1e-6 turn per sample
        let mut worst = 0f64;
        for _ in 0..1_000_003 {
            let want = (nco.turns as f64 / TURN * 2.0 * core::f64::consts::PI).sin();
            worst = worst.max((f64::from(nco.sin()) - want).abs());
            nco.step(0.0);
        }
        assert!(worst < 5e-7, "worst {worst:.2e}");
    }

    #[test]
    fn matches_an_f64_sum() {
        let (f, fs) = (1473.37f32, 12_000.0f32);
        let mut nco = Nco::new(f, fs);
        let mut phi = 0.0f64;
        let d = 2.0 * core::f64::consts::PI * f64::from(f) / f64::from(fs);
        for _ in 0..1_300_000 {
            let e = (f64::from(nco.phase()) - phi).rem_euclid(2.0 * core::f64::consts::PI);
            assert!(e.min(2.0 * core::f64::consts::PI - e) < 2e-6, "{e}");
            nco.step(0.0);
            phi = (phi + d).rem_euclid(2.0 * core::f64::consts::PI);
        }
    }

    /// The rotor is `exp(-j 2 pi f n / fs)`: against the `f64` value at each sample of a long run, starting before sample 0
    /// (negative `n`) and after it, at frequencies from near DC to near Nyquist. Its worst error is the refresh interval's
    /// worth of step rounding (3e-5), not growth with the run. A sign flipped (`exp(+j)`), a phase read off by a sample or a
    /// step that is not unit length shows at once.
    #[cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]
    #[test]
    fn the_rotor_is_the_complex_exponential() {
        for (f, fs, n0) in [
            (1234.567f64, 12_000.0f32, -70i64),
            (0.37, 200.0, 0),
            (2950.1, 3_000.0, -1),
            (5990.0, 12_000.0, 12_345),
        ] {
            let mut r = Rotor::new(f, fs, n0);
            let mut worst = 0f64;
            for k in 0..400_000i64 {
                let n = n0 + k;
                let (re, im) = r.next();
                let ph =
                    -2.0 * core::f64::consts::PI * (f * n as f64 / f64::from(fs)).rem_euclid(1.0);
                worst = worst
                    .max((f64::from(re) - ph.cos()).abs())
                    .max((f64::from(im) - ph.sin()).abs());
            }
            assert!(worst < 5e-5, "f {f} fs {fs} n0 {n0}: worst {worst:.2e}");
        }
    }

    /// `cos` is the sine a quarter turn on, to the sine's own accuracy, all round the circle.
    #[cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]
    #[test]
    fn cos_is_a_cosine() {
        let mut nco = Nco::new(1.0, 1_000_003.0);
        let mut worst = 0f64;
        for _ in 0..1_000_003 {
            let want = (nco.turns as f64 / TURN * 2.0 * core::f64::consts::PI).cos();
            worst = worst.max((f64::from(nco.cos()) - want).abs());
            nco.step(0.0);
        }
        assert!(worst < 5e-7, "worst {worst:.2e}");
    }
}
