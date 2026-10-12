// SPDX-License-Identifier: GPL-3.0-only
//! The last stage of [`IqToAudio`](super::IqToAudio) when a stream's real rate is not its nominal one
//! (#686): a fractional-delay resampler of the 12 kHz complex baseband by the ratio `1 + eps`.
//!
//! The front end is built for an integer nominal rate and treats every input sample as `1/Fs_nominal`
//! apart. A source that really delivers `Fs_nominal (1 + eps)` (an `rtl_tcp` server, a sound card at an
//! odd clock) makes its chain output one 12 kHz sample per `12000 (1 + eps)^-1` real seconds... the
//! other way round: chain output `m` is at real time `m / (12000 (1 + eps))`. Output `k` of the corrected
//! stream is to be at `k / 12000`, so it is the chain's output interpolated at position `k (1 + eps)`.
//!
//! **The kernel.** A Kaiser-windowed sinc cut at 6 kHz, exactly half of 12 kHz: it is zero at every
//! other integer, so at an integer position the interpolation is the sample itself, and it passes the
//! whole ±3.2 kHz of the baseband (the existing `L/M` resampler needs the same: pass 3.2 kHz, stop
//! 8.8 kHz). `P` phases of it are kept as one prototype and the phase is **interpolated linearly**
//! between neighbours, so the position can move continuously (a rate that changes while running). The
//! numbers are in [`PHASES`] and [`design`].
//!
//! The stage has no group delay: output `k` is at position `x_k` exactly; it needs `N/2` input samples
//! ahead of the position, so its output comes `N/2` samples late. A stage that starts late (a rate
//! offset set mid-stream) has no history for its first `N/2` outputs, which are interpolated from zeros:
//! a glitch of 2 ms, once.

use alloc::vec::Vec;

use super::{AUDIO_RATE_HZ, DESIGN_MARGIN_DB, REJECT_DB};
use crate::engine::dsp::fir_decimate::{design_lowpass_kaiser, kaiser_order};

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// Phases of the kernel. Linear interpolation between them leaves an error of about `(pi f / (P Fs))^2 / 2`
/// of the signal, for `f` at the edge of the band (3.2 kHz of 12 kHz): -100 dB at 128.
pub(crate) const PHASES: usize = 128;
/// Edge of the baseband the kernel passes, Hz from the centre: the front end's own stop edge.
const PASS_EDGE_HZ: f64 = 3_200.0;

/// The kernel: `taps_per_side * 2 * PHASES + 1` samples of a sinc at `PHASES` times the 12 kHz rate,
/// cut at 6 kHz, gain `PHASES` (so a phase sums to 1), and the number of input samples it spans.
fn design() -> (Vec<f32>, usize) {
    let up = (AUDIO_RATE_HZ as usize * PHASES) as f64;
    // Pass 3.2 kHz, stop where the first image of the band begins: 12 kHz - 3.2 kHz.
    let (n, beta) = kaiser_order(
        REJECT_DB + DESIGN_MARGIN_DB,
        (AUDIO_RATE_HZ as f64 - 2.0 * PASS_EDGE_HZ) / up,
    );
    // A whole even number of input samples, so the centre is a tap and the span splits in two.
    let span = (n.div_ceil(PHASES) + 1) & !1;
    let ntaps = span * PHASES + 1;
    // The cut is 6 kHz of 12 kHz: 0.5 / PHASES of the prototype's rate.
    let mut h = design_lowpass_kaiser(ntaps, 0.5 / PHASES as f64, beta);
    for t in h.iter_mut() {
        *t *= PHASES as f32;
    }
    // One more, for the linear interpolation at the last phase.
    h.push(0.0);
    (h, span)
}

/// Resample 12 kHz complex baseband by `ratio` input samples per output sample.
pub(crate) struct RateStage {
    h: Vec<f32>,
    /// Input samples the kernel spans (even).
    span: usize,
    ratio: f64,
    /// The input kept: absolute index of `hi[0]`.
    base: i64,
    hi: Vec<f32>,
    hq: Vec<f32>,
    /// Absolute input position of the next output.
    next: f64,
}

impl RateStage {
    /// A stage whose first output is at input position `start` (the number of samples that went by
    /// before it was built: it takes over from a straight-through stream).
    pub(crate) fn new(ratio: f64, start: u64) -> Self {
        let (h, span) = design();
        let half = (span / 2) as i64;
        // The kernel reaches `half` samples back from the first position: zeros.
        Self {
            h,
            span,
            ratio,
            base: start as i64 - half,
            hi: alloc::vec![0.0; span / 2],
            hq: alloc::vec![0.0; span / 2],
            next: start as f64,
        }
    }

    pub(crate) fn set_ratio(&mut self, ratio: f64) {
        self.ratio = ratio;
    }

    /// `(I, Q)` at input position `x`; `hi`/`hq` must hold the samples `floor(x) - span/2 + 1 ..=
    /// floor(x) + span/2`.
    #[inline]
    fn at(&self, x: f64) -> (f32, f32) {
        let i = x.floor();
        let u = (x - i) * PHASES as f64;
        let q = (u as usize).min(PHASES - 1);
        let w = (u - q as f64) as f32;
        let half = (self.span / 2) as i64;
        let centre = half as usize * PHASES;
        // Input samples m = i - a for a in -span/2 ..= span/2 - 1, at kernel index centre + a P + q.
        let first = i as i64 + half - self.base;
        let (mut yi, mut yq) = (0.0f32, 0.0f32);
        for s in 0..self.span {
            // a = s - span/2; m = i + span/2 - s.
            let idx = centre + s * PHASES + q - half as usize * PHASES;
            let c = self.h[idx] + w * (self.h[idx + 1] - self.h[idx]);
            let m = (first - s as i64) as usize;
            yi += c * self.hi[m];
            yq += c * self.hq[m];
        }
        (yi, yq)
    }

    /// Append the input; emit every output whose kernel is covered.
    pub(crate) fn push(&mut self, xi: &[f32], xq: &[f32], oi: &mut Vec<f32>, oq: &mut Vec<f32>) {
        self.hi.extend_from_slice(xi);
        self.hq.extend_from_slice(xq);
        let half = (self.span / 2) as i64;
        let end = self.base + self.hi.len() as i64; // one past the last input held
        // An output at x needs the sample floor(x) + half.
        while (self.next.floor() as i64) + half < end {
            let (a, b) = self.at(self.next);
            oi.push(a);
            oq.push(b);
            self.next += self.ratio;
        }
        // Keep what the next output's kernel reaches back to.
        let keep_from = self.next.floor() as i64 - half + 1;
        let drop = (keep_from - self.base).max(0) as usize;
        if drop > 0 && drop <= self.hi.len() {
            self.hi.drain(..drop);
            self.hq.drain(..drop);
            self.base += drop as i64;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::TAU;

    fn tone(n: usize, f_hz: f64) -> (Vec<f32>, Vec<f32>) {
        (0..n)
            .map(|k| {
                let ph = TAU * f_hz * k as f64 / 12_000.0;
                (ph.cos() as f32, ph.sin() as f32)
            })
            .unzip()
    }

    /// At ratio 1 and an integer start the stage is the identity (the kernel is a sinc cut at half the
    /// rate: zero at the other integers), delayed by nothing.
    #[test]
    fn ratio_one_is_the_identity() {
        let (xi, xq) = tone(4_000, 1_700.0);
        let mut st = RateStage::new(1.0, 0);
        let (mut oi, mut oq) = (Vec::new(), Vec::new());
        st.push(&xi, &xq, &mut oi, &mut oq);
        assert!(oi.len() >= 4_000 - st.span / 2 - 1, "{}", oi.len());
        let worst = (st.span..oi.len())
            .map(|k| ((oi[k] - xi[k]).abs()).max((oq[k] - xq[k]).abs()))
            .fold(0.0f32, f32::max);
        assert!(worst < 1e-4, "{worst}");
    }

    /// A tone of 12 kHz-nominal frequency f in a stream whose real rate is (1 + eps) times nominal is a
    /// tone of f (1 + eps)... no: the samples were taken at `Fs (1 + eps)`, so the tone's true frequency
    /// is `f (1 + eps)` when read at the nominal rate; resampling by `1 + eps` restores `f` at true 12 kHz.
    /// Here: a tone made at `f_true` on the stretched grid comes out at `f_true`, to a fraction of a
    /// sample of phase, over a long run, for both signs of eps and at both ends of the band.
    #[test]
    fn restores_the_true_time_axis() {
        for &eps in &[-0.0029, -0.0007, 0.0004, 0.0031] {
            for &f_true in &[-2_900.0f64, -1_300.0, 400.0, 2_900.0] {
                // Input sample m is at true time m / (12000 (1 + eps)); the tone is exp(j 2 pi f_true t).
                let n = 20_000usize;
                let (xi, xq): (Vec<f32>, Vec<f32>) = (0..n)
                    .map(|m| {
                        let t = m as f64 / (12_000.0 * (1.0 + eps));
                        let ph = TAU * f_true * t;
                        (ph.cos() as f32, ph.sin() as f32)
                    })
                    .unzip();
                let mut st = RateStage::new(1.0 + eps, 0);
                let (mut oi, mut oq) = (Vec::new(), Vec::new());
                st.push(&xi, &xq, &mut oi, &mut oq);
                // Output k must be exp(j 2 pi f_true k / 12000).
                let worst = (st.span..oi.len().min(15_000))
                    .map(|k| {
                        let ph = TAU * f_true * k as f64 / 12_000.0;
                        (oi[k] - ph.cos() as f32)
                            .abs()
                            .max((oq[k] - ph.sin() as f32).abs())
                    })
                    .fold(0.0f32, f32::max);
                assert!(worst < 2e-3, "eps {eps} f {f_true}: {worst}");
            }
        }
    }

    /// Changing the ratio while running moves the position continuously: no jump in the output.
    #[test]
    fn a_ratio_change_does_not_jump() {
        let (xi, xq) = tone(30_000, 1_000.0);
        let mut st = RateStage::new(1.0, 0);
        let (mut oi, mut oq) = (Vec::new(), Vec::new());
        for (n, chunk) in xi.chunks(1_000).enumerate() {
            if n == 10 {
                st.set_ratio(1.002);
            }
            if n == 20 {
                st.set_ratio(0.998);
            }
            st.push(
                chunk,
                &xq[n * 1_000..n * 1_000 + chunk.len()],
                &mut oi,
                &mut oq,
            );
        }
        // A 1 kHz tone at 12 kHz moves at most 0.53 rad per sample: a jump would show as a step.
        let worst = (st.span..oi.len() - 1)
            .map(|k| {
                let d = (oi[k + 1] - oi[k]).hypot(oq[k + 1] - oq[k]);
                (d - 0.5176).abs()
            })
            .fold(0.0f32, f32::max);
        assert!(worst < 5e-3, "{worst}");
    }
}
