//! One symbol-length FFT, reused across the symbols of a frame.
//!
//! JT9, JT65 and Q65 demodulate by running an `nsps`-point FFT over
//! each symbol window and reading tone-bin powers out of it. Each of
//! the five sites that did so (`jt9::rx`, `jt65::rx`, `q65::rx` twice,
//! `q65::snr`) planned its own `rustfft` instance, kept its own scratch
//! and working buffer, and copied the real window into it. That
//! plumbing is this type now. The per-mode reads stay in each mode,
//! because they differ in arithmetic (`norm()²` for JT9 against
//! `norm_sqr()` elsewhere), in what they skip, and in JT65's
//! NCO-mixed input.
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

    /// Spectrum of a window the caller writes into the buffer, e.g.
    /// through a mixer.
    pub fn with_input(&mut self, fill: impl FnOnce(&mut [Complex32])) -> &[Complex32] {
        fill(&mut self.buf);
        self.fft.process(&mut self.buf);
        &self.buf
    }
}
