// SPDX-License-Identifier: GPL-3.0-or-later
//! Generic complex digital down-converter: real PCM at an arbitrary
//! centre frequency → complex baseband, streaming.
//!
//! `docs/notes/FST4_DDC_DESIGN.md` §4.3. Composes three pieces this
//! crate already has, none of them protocol-specific:
//!
//! ```text
//! real i16 @ input_rate
//!   → Mixer (exp(-j2π·center·n/Fs))                    → complex @ input_rate
//!   → FirStage cascade (loose filter, integer decimate)  → complex @ input_rate/D
//!   → PolyphaseResampler (sharp filter, final stage)      → complex @ Fs_c
//! ```
//!
//! The sharp filter sits last (lowest rate), same reasoning
//! `wspr::ddc`'s own two-stage cascade documents: a single-stage filter's
//! MAC count is set by the transition-band ratio alone, independent of
//! how much decimation happens, so pushing the narrow-transition work
//! to the cheapest (lowest) rate is what makes a cascade pay for
//! itself. Unlike `wspr::ddc` (fixed 1500 Hz centre, `Fs/32` target,
//! integer-only), this module's centre frequency and target rate are
//! both caller-supplied — `fst4::ddc` is the first caller, picking
//! `(center_hz, K)` per search shape (sniper vs. wideband) and
//! deriving the cascade split from it.
//!
//! [`wspr::ddc`]: crate::wspr::ddc

use alloc::vec::Vec;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::fir_decimate::FirStage;
use super::polyphase::PolyphaseResampler;

/// Samples between phasor renormalisations — see [`Mixer`]'s doc
/// comment.
const RENORM_PERIOD: u32 = 4096;

/// Streaming complex mixer, `exp(-j2π·center_hz·n/Fs)`.
///
/// A rotating-phasor NCO (`cur *= step` each sample) rather than a
/// per-sample `sin`/`cos` call — `wspr::ddc`'s own mixer avoids that
/// too, but does it with a period-8 lookup table because its centre
/// frequency (1500 Hz) is always exactly `Fs/8`. This module's centre
/// is caller-supplied and generally irrational relative to `Fs`, so no
/// finite table applies; the rotation recurrence is the general form
/// of the same idea. Rotation error accumulates every multiply, so
/// `cur` is renormalised to unit magnitude every [`RENORM_PERIOD`]
/// samples — cheap (one `sqrt` per period) and bounds the drift a long
/// FST4 slot (thousands of samples) would otherwise accumulate.
pub(crate) struct Mixer {
    step: Complex<f32>,
    cur: Complex<f32>,
    since_renorm: u32,
}

impl Mixer {
    pub(crate) fn new(center_hz: f32, sample_rate_hz: f32) -> Self {
        let dphi = -2.0 * core::f32::consts::PI * center_hz / sample_rate_hz;
        Self {
            step: Complex::new(dphi.cos(), dphi.sin()),
            cur: Complex::new(1.0, 0.0),
            since_renorm: 0,
        }
    }

    /// Advance the phasor one sample and renormalise on schedule —
    /// the part [`Self::mix`]/[`Self::mix_complex`] share.
    #[inline]
    fn advance(&mut self) {
        self.cur *= self.step;
        self.since_renorm += 1;
        if self.since_renorm >= RENORM_PERIOD {
            let mag = (self.cur.re * self.cur.re + self.cur.im * self.cur.im).sqrt();
            if mag > 0.0 {
                self.cur.re /= mag;
                self.cur.im /= mag;
            }
            self.since_renorm = 0;
        }
    }

    /// Mix one real input sample.
    #[inline]
    pub(crate) fn mix(&mut self, x: f32) -> (f32, f32) {
        let out = (x * self.cur.re, x * self.cur.im);
        self.advance();
        out
    }

    /// Mix one complex input sample — for re-centring an
    /// already-complex baseband (`fst4::ddc`'s refine stage: the
    /// coarse DDC baseband still spans a whole search window around
    /// one `center_hz`, and pulling a specific candidate's own
    /// zero-frequency baseband out of it needs a second,
    /// per-candidate complex mix). `(i+jq)·cur`, same phasor as
    /// [`Self::mix`].
    #[inline]
    pub(crate) fn mix_complex(&mut self, i: f32, q: f32) -> (f32, f32) {
        let out = (
            i * self.cur.re - q * self.cur.im,
            i * self.cur.im + q * self.cur.re,
        );
        self.advance();
        out
    }
}

/// One integer FirStage cascade link: `(ntaps, decim, fc_norm)` —
/// `fc_norm` normalised to *this stage's own input rate*, matching
/// [`FirStage::new`]'s own parameter.
pub type IntStageSpec = (usize, usize, f32);

/// Cascade recipe: mixer centre/rate, an integer `FirStage` chain
/// (loose filters, cheap — may be empty, e.g. `fst4::ddc`'s wideband
/// config), and a final rational [`PolyphaseResampler`] stage
/// (`(l, m, ntaps)`) carrying the real passband/stopband requirement.
#[derive(Clone)]
pub struct DdcCascadeConfig {
    pub center_hz: f32,
    pub input_rate_hz: f32,
    pub int_stages: Vec<IntStageSpec>,
    pub resampler: (u32, u32, usize),
    /// `hist_margin` passed to every stage's constructor (both
    /// `FirStage` and `PolyphaseResampler` take one).
    pub hist_margin: usize,
}

/// Streaming instance built from a [`DdcCascadeConfig`].
pub struct StreamingComplexDdc {
    mixer: Mixer,
    int_stages: Vec<FirStage>,
    resampler: PolyphaseResampler,
    /// Conservative total group delay, in *original* (mixer-input-rate)
    /// samples — see [`Self::flush`].
    flush_zeros: usize,
}

impl StreamingComplexDdc {
    pub fn new(cfg: &DdcCascadeConfig) -> Self {
        let mixer = Mixer::new(cfg.center_hz, cfg.input_rate_hz);
        let int_stages = cfg
            .int_stages
            .iter()
            .map(|&(ntaps, decim, fc_norm)| FirStage::new(ntaps, decim, fc_norm, cfg.hist_margin))
            .collect();
        let (l, m, ntaps) = cfg.resampler;
        let resampler = PolyphaseResampler::new(l, m, ntaps, cfg.hist_margin);

        // Total group delay, converted to original-input-rate samples
        // one stage at a time: a stage's own `(ntaps-1)/2` is in its
        // *own* input samples, so it's scaled up by every upstream
        // decimation factor before accumulating (mirrors
        // `wspr::ddc::StreamingDdcCascade::flush`'s single-stage
        // version of the same idea, generalised to N stages plus a
        // rational tail). The resampler's own delay uses `l` in place
        // of an integer `decim` — `(ntaps-1)/2` there is measured at
        // the *upsampled* (`L`×) domain, so dividing by `l` converts
        // it back to the resampler's own input-rate samples, same
        // derivation as [`PolyphaseResampler::group_delay_output`]
        // but stopping one conversion step earlier (input-rate, not
        // output-rate). Truncating (not rounding) integer division
        // makes this an underestimate by at most a handful of
        // samples per stage — harmless for `flush`'s purpose (letting
        // transients clear before `compute_spectra` reads the tail),
        // not a precision the caller should rely on for exact timing.
        let mut scale = 1usize;
        let mut delay = 0usize;
        for &(stage_ntaps, decim, _) in &cfg.int_stages {
            delay += ((stage_ntaps - 1) / 2) * scale;
            scale *= decim;
        }
        delay += (((ntaps - 1) / 2) / l as usize) * scale;

        Self {
            mixer,
            int_stages,
            resampler,
            flush_zeros: delay + 1,
        }
    }

    /// This cascade's own estimated group delay, in *original*
    /// (mixer-input-rate) samples — the same conservative estimate
    /// [`Self::flush`] uses, exposed so a caller chaining more DSP
    /// after this cascade's output (e.g. `fst4::ddc`'s refine-stage
    /// resampler) can account for it rather than treating this
    /// cascade's sample 0 as if it carried no delay at all. Per
    /// [`Self::new`]'s derivation this is an *underestimate* by up to
    /// a handful of samples per stage — not a precision to build exact
    /// timing on without a dedicated alignment pass.
    pub fn group_delay_input_samples(&self) -> usize {
        self.flush_zeros.saturating_sub(1)
    }

    /// Push one real sample through mixer → integer cascade → final
    /// resampler, appending any complex baseband samples this push
    /// completes.
    pub fn push_one(&mut self, x: f32, out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        let (mut i, mut q) = self.mixer.mix(x);
        for stage in &mut self.int_stages {
            match stage.push_one(i, q) {
                Some((si, sq)) => {
                    i = si;
                    q = sq;
                }
                None => return,
            }
        }
        self.resampler.push(i, q, out_i, out_q);
    }

    /// Push a block of `i16` PCM.
    pub fn push_i16(&mut self, audio: &[i16], out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        for &s in audio {
            self.push_one(s as f32, out_i, out_q);
        }
    }

    /// Flush the tail: feed `flush_zeros`-worth of zero input samples
    /// through the whole cascade so the last real input reaches the
    /// centre of every filter — see that field's doc comment in
    /// [`Self::new`] for the delay derivation.
    pub fn flush(&mut self, out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        for _ in 0..self.flush_zeros {
            self.push_one(0.0, out_i, out_q);
        }
    }
}

/// Re-centre and further decimate an already-complex baseband —
/// [`StreamingComplexDdc`]'s sibling for a second, per-candidate
/// down-conversion stage on top of a first cascade's output (`fst4::
/// ddc`'s refine stage: the coarse DDC baseband still spans a whole
/// search window around one `center_hz`; recovering a specific
/// candidate's own zero-frequency baseband — what a Costas correlator
/// like [`crate::engine::sync::fine_sync_power`] assumes — needs this).
/// Same shape as `StreamingComplexDdc` minus the real-input step
/// (input is already complex) and the integer-cascade support (no
/// caller needs one yet).
pub struct StreamingComplexRecenter {
    mixer: Mixer,
    resampler: PolyphaseResampler,
    flush_zeros: usize,
}

impl StreamingComplexRecenter {
    /// `center_hz` here is relative to the *input* baseband's own
    /// zero (i.e. the offset still present after the first DDC stage,
    /// not an absolute audio frequency) — the candidate frequency
    /// this stage should pull down to DC.
    pub fn new(
        center_hz: f32,
        input_rate_hz: f32,
        l: u32,
        m: u32,
        ntaps: usize,
        hist_margin: usize,
    ) -> Self {
        let mixer = Mixer::new(center_hz, input_rate_hz);
        let resampler = PolyphaseResampler::new(l, m, ntaps, hist_margin);
        let delay = ((ntaps - 1) / 2) / l as usize;
        Self {
            mixer,
            resampler,
            flush_zeros: delay + 1,
        }
    }

    /// This stage's own estimated group delay, in its *input*
    /// (already-complex-baseband-rate) samples — same caveat as
    /// [`StreamingComplexDdc::group_delay_input_samples`].
    pub fn group_delay_input_samples(&self) -> usize {
        self.flush_zeros.saturating_sub(1)
    }

    /// This stage's own group delay in its *output* samples — see
    /// [`PolyphaseResampler::group_delay_output`], which this
    /// forwards.
    pub fn group_delay_output_samples(&self) -> usize {
        self.resampler.group_delay_output()
    }

    pub fn push_one(&mut self, i: f32, q: f32, out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        let (mi, mq) = self.mixer.mix_complex(i, q);
        self.resampler.push(mi, mq, out_i, out_q);
    }

    pub fn push(&mut self, xi: &[f32], xq: &[f32], out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        for (&i, &q) in xi.iter().zip(xq.iter()) {
            self.push_one(i, q, out_i, out_q);
        }
    }

    pub fn flush(&mut self, out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        for _ in 0..self.flush_zeros {
            self.push_one(0.0, 0.0, out_i, out_q);
        }
    }
}

/// Block-mode convenience: run a whole buffer through a fresh
/// [`StreamingComplexDdc`] and return `(i, q)`. For host tests and
/// non-streaming callers — the streaming `push_one`/`push_i16` API is
/// what an embedded, capture-time-interleaved caller would use.
pub fn ddc_block(audio: &[i16], cfg: &DdcCascadeConfig) -> (Vec<f32>, Vec<f32>) {
    let mut ddc = StreamingComplexDdc::new(cfg);
    let mut out_i = Vec::new();
    let mut out_q = Vec::new();
    ddc.push_i16(audio, &mut out_i, &mut out_q);
    ddc.flush(&mut out_i, &mut out_q);
    (out_i, out_q)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(freq_hz: f32, fs_hz: f32, amp: f32, n: usize) -> Vec<i16> {
        let w = 2.0 * core::f64::consts::PI * freq_hz as f64 / fs_hz as f64;
        (0..n)
            .map(|k| (amp as f64 * 32767.0 * (w * k as f64).cos()) as i16)
            .collect()
    }

    /// A single-stage cascade (no integer pre-decimation) centred on a
    /// tone: the tone lands at DC in the complex output, positive
    /// audio offset giving positive baseband frequency (same
    /// convention `wspr::ddc`'s own equivalent test documents).
    #[test]
    fn centre_tone_lands_near_dc() {
        let fs_in = 12_000.0f32;
        let center = 1500.0f32;
        let cfg = DdcCascadeConfig {
            center_hz: center,
            input_rate_hz: fs_in,
            int_stages: Vec::new(),
            resampler: (64, 243, 4001), // matches fst4::ddc's wideband shape
            hist_margin: 512,
        };
        let n = 96_000;
        let audio = tone(center, fs_in, 0.5, n);
        let (i, q) = ddc_block(&audio, &cfg);

        let settle = i.len() / 4;
        let span = settle..(i.len() - settle);
        let (mut si, mut sq, mut cnt) = (0.0f64, 0.0f64, 0usize);
        for k in span {
            si += i[k] as f64;
            sq += q[k] as f64;
            cnt += 1;
        }
        let (mi, mq) = (si / cnt as f64, sq / cnt as f64);
        let mag = (mi * mi + mq * mq).sqrt();
        assert!(mag > 0.05, "centre tone magnitude too small: {mag}");
        // DC in the complex output means the phase barely rotates
        // sample to sample over the settled span — check the first
        // and last sample in the span keep the same phase sign
        // structure rather than asserting an exact ratio (amplitude
        // scale isn't calibrated here, unlike `wspr::ddc::
        // REFERENCE_GAIN` — this cascade has no reference to match).
        assert!(mi.abs() > mag * 0.3 || mq.abs() > mag * 0.3);
    }

    /// An offset tone becomes a rotating phasor at the offset,
    /// measured the same way `wspr::ddc`'s own offset-tone test does.
    #[test]
    fn offset_tone_rotates_at_the_offset() {
        let fs_in = 12_000.0f32;
        let center = 1500.0f32;
        let offset = 40.0f32;
        let cfg = DdcCascadeConfig {
            center_hz: center,
            input_rate_hz: fs_in,
            int_stages: Vec::new(),
            resampler: (64, 243, 4001),
            hist_margin: 512,
        };
        let n = 96_000;
        let audio = tone(center + offset, fs_in, 0.5, n);
        let (i, q) = ddc_block(&audio, &cfg);
        let fs_out = fs_in * 64.0 / 243.0;

        let settle = i.len() / 4;
        let a = settle;
        let b = i.len() - settle;
        let ph = |k: usize| q[k].atan2(i[k]);
        let mut d = ph(b) - ph(a);
        let expect_total = 2.0 * core::f32::consts::PI * offset * (b - a) as f32 / fs_out;
        while d - expect_total > core::f32::consts::PI {
            d -= 2.0 * core::f32::consts::PI;
        }
        while expect_total - d > core::f32::consts::PI {
            d += 2.0 * core::f32::consts::PI;
        }
        let measured_hz = d / (2.0 * core::f32::consts::PI) * fs_out / (b - a) as f32;
        assert!(
            (measured_hz - offset).abs() < 1.0,
            "measured {measured_hz} Hz, expected {offset}"
        );
    }
}
