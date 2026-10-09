//! One symbol-length FFT, reused across the symbols of a frame.
//!
//! JT9, JT65 and Q65 demodulate by running an `nsps`-point FFT over
//! each symbol window and reading tone-bin powers out of it. Each of
//! the five sites that did so (`jt9::rx`, `jt65::rx`, `q65::rx` twice,
//! `q65::snr`) planned its own `rustfft` instance, kept its own scratch
//! and working buffer, and copied the real window into it. That
//! plumbing is this type now. The per-mode reads stay in each mode,
//! because they differ in arithmetic (`norm()²` for JT9 against
//! `norm_sqr()` elsewhere) and in what they skip. A residual carrier offset,
//! when the caller's frequency is not on a bin, is taken out by
//! [`SymbolFft::mixed`] (#424).
//!
//! The FFT is planned through [`crate::engine::fft::default_planner`],
//! so it is rustfft on host and bit-identical to the direct calls it
//! replaced (#390).

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex32;

use crate::engine::fft::{Fft, default_planner};

/// A planned `len`-point forward FFT with its own working buffer.
pub struct SymbolFft {
    fft: Box<dyn Fft>,
    buf: Vec<Complex32>,
}

impl SymbolFft {
    /// Plan an `nsps`-point forward FFT.
    pub fn new(nsps: usize) -> Self {
        Self {
            fft: default_planner().plan_forward(nsps),
            buf: vec![Complex32::new(0.0, 0.0); nsps],
        }
    }

    /// Transform length.
    #[inline]
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    /// `true` when planned for zero length.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }

    /// Spectrum of the real window `audio[start..start + len()]`.
    ///
    /// # Panics
    ///
    /// Panics if the window runs past the end of `audio`.
    pub fn real(&mut self, audio: &[f32], start: usize) -> &[Complex32] {
        let n = self.buf.len();
        for (slot, &s) in self.buf.iter_mut().zip(&audio[start..start + n]) {
            *slot = Complex32::new(s, 0.0);
        }
        self.fft.process(&mut self.buf);
        &self.buf
    }

    /// Spectrum of the real window `audio[start..start + len()]` after mixing
    /// it by `exp(-j2π·residual_hz·n/Fs)`, so a carrier that sits
    /// `residual_hz` above a bin centre lands on it.
    ///
    /// Without this a rectangular-window FFT of a carrier a half bin off loses up
    /// to ~3.9 dB to scalloping (measured on JT65's and WSPR's AWGN corpora, which
    /// is why both have always corrected for it). `|FFT|²` does not depend on the
    /// phase the mixer starts a window at, so every window restarts at 0 and the
    /// caller need not carry a phase across symbols. A residual of at most
    /// `1e-6` Hz is not worth the multiplies and takes the plain path.
    ///
    /// The mixer is the crate's `engine::dsp::ddc::Mixer`, a renormalised rotating
    /// phasor; JT65 used `cos`/`sin` of a wrapped f32 phase per sample, and WSPR of
    /// `step · n` with `n` an absolute sample index (f32 resolves the phase
    /// to ~0.06 rad by `n` ≈ 700 000).
    ///
    /// # Panics
    ///
    /// Panics if the window runs past the end of `audio`.
    pub fn mixed(
        &mut self,
        audio: &[f32],
        start: usize,
        residual_hz: f32,
        sample_rate_hz: f32,
    ) -> &[Complex32] {
        if residual_hz.abs() <= 1e-6 {
            return self.real(audio, start);
        }
        let n = self.buf.len();
        let mut mixer = super::ddc::Mixer::new(residual_hz, sample_rate_hz);
        for (slot, &s) in self.buf.iter_mut().zip(&audio[start..start + n]) {
            let (re, im) = mixer.mix(s);
            *slot = Complex32::new(re, im);
        }
        self.fft.process(&mut self.buf);
        &self.buf
    }

    /// Spectrum of a window the caller writes into the buffer, e.g.
    /// through a mixer.
    pub fn with_input(&mut self, fill: impl FnOnce(&mut [Complex32])) -> &[Complex32] {
        fill(&mut self.buf);
        self.fft.process(&mut self.buf);
        &self.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 12_000.0;
    const N: usize = 1024; // df = 11.71875 Hz

    fn tone(freq_hz: f64, len: usize) -> Vec<f32> {
        (0..len)
            .map(|k| (2.0 * core::f64::consts::PI * freq_hz * k as f64 / FS as f64).cos() as f32)
            .collect()
    }

    /// No residual is the plain real transform, bit for bit.
    #[test]
    fn zero_residual_is_the_plain_transform() {
        let audio = tone(1234.5, 4 * N);
        let mut a = SymbolFft::new(N);
        let mut b = SymbolFft::new(N);
        let plain = a.real(&audio, N).to_vec();
        for r in [0.0f32, 1e-7, -1e-7] {
            let mixed = b.mixed(&audio, N, r, FS);
            assert!(
                plain
                    .iter()
                    .zip(mixed)
                    .all(|(x, y)| x.re.to_bits() == y.re.to_bits()
                        && x.im.to_bits() == y.im.to_bits()),
                "residual {r}"
            );
        }
    }

    /// A carrier half a bin above bin 100 loses ~3.9 dB there unmixed, and none
    /// once its residual is mixed out — at the start of a buffer and 700 000
    /// samples into one, where an absolute f32 phase would have lost the
    /// fraction of a cycle.
    #[test]
    fn mixing_the_residual_out_removes_the_scalloping_loss() {
        let df = FS / N as f32;
        let f = (100.5 * df) as f64;
        let audio = tone(f, 700_000 + 2 * N);
        // |X[bin]|² of a unit cosine over N samples, exactly on a bin: (N/2)².
        let ideal = (N as f32 / 2.0).powi(2);
        for start in [0usize, 700_000] {
            let mut fft = SymbolFft::new(N);
            let unmixed = fft.real(&audio, start)[100].norm_sqr();
            let mixed = fft.mixed(&audio, start, 0.5 * df, FS)[100].norm_sqr();
            let loss_db = 10.0 * (unmixed / ideal).log10();
            assert!(
                (-4.6..-3.2).contains(&loss_db),
                "start {start}: unmixed loss {loss_db:.2} dB, expected the ~3.9 dB scalloping loss"
            );
            assert!(
                mixed > 0.97 * ideal,
                "start {start}: mixed bin holds {:.4} of the ideal power",
                mixed / ideal
            );
        }
    }
}
