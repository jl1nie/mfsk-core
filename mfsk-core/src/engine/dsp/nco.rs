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

    /// The phase now, radians in `[0, 2 pi)`, from its top 32 bits.
    #[inline]
    pub(crate) fn phase(&self) -> f32 {
        ((self.turns >> 32) as u32) as f32 * (2.0 * core::f32::consts::PI / 4_294_967_296.0)
    }

    /// Advance by one sample: the carrier and `modulation_rad`.
    #[inline]
    pub(crate) fn step(&mut self, modulation_rad: f32) {
        let m = (modulation_rad * Self::TURNS_PER_RAD) as i64 as u64;
        self.turns = self.turns.wrapping_add(self.increment).wrapping_add(m);
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
}
