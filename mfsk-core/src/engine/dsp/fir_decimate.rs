// SPDX-License-Identifier: GPL-3.0-or-later
//! Generic streaming real-tap FIR filter + decimate, over a complex
//! (I, Q) history.
//!
//! Protocol-agnostic building block for incremental (chunk-at-a-time)
//! down-conversion — [`wspr::ddc`](crate::wspr::ddc) is the first
//! caller (its `StreamingDdcCascade`, a two-stage decimate-by-8-then-4
//! cascade), composing this with a protocol-specific mixer and output
//! gain. Nothing here is WSPR-specific: given a Blackman-windowed-sinc
//! lowpass ([`design_lowpass`]) and a decimation factor, [`FirStage`]
//! incrementally filters and decimates whatever complex stream it's
//! fed, one sample or one stage-cascade link at a time.

use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Windowed-sinc low-pass, cutoff `fc` normalised to the sample rate,
/// Blackman window, unit DC gain.
pub fn design_lowpass(ntaps: usize, fc_norm: f32) -> Vec<f32> {
    let mut h = vec![0.0f32; ntaps];
    let m = (ntaps - 1) as f32;
    let mut sum = 0.0f32;
    for (k, tap) in h.iter_mut().enumerate() {
        let x = k as f32 - m / 2.0;
        let sinc = if x.abs() < 1e-6 {
            2.0 * fc_norm
        } else {
            (2.0 * core::f32::consts::PI * fc_norm * x).sin() / (core::f32::consts::PI * x)
        };
        // Blackman: better stopband than Hamming (~-74 dB vs -53 dB),
        // which is the half that matters here — the passband is
        // oversized on purpose.
        let w = 0.42 - 0.5 * (2.0 * core::f32::consts::PI * k as f32 / m).cos()
            + 0.08 * (4.0 * core::f32::consts::PI * k as f32 / m).cos();
        *tap = sinc * w;
        sum += *tap;
    }
    for tap in h.iter_mut() {
        *tap /= sum;
    }
    h
}

/// One real-tapped FIR-and-decimate stage over a complex (I, Q)
/// history. Owns its buffers (`Vec`) — sized by the caller's `ntaps` +
/// `hist_margin`, so a caller with a small `ntaps` (a cascade stage,
/// say) gets a small footprint automatically, with no shared-buffer
/// placement API to route through the way a single large-`ntaps`
/// caller might still want (see [`wspr::ddc::StreamingDdc`]'s own
/// `new_in`, which predates this generic extraction and keeps its own
/// caller-supplied-buffer shape for exactly that reason).
///
/// [`wspr::ddc::StreamingDdc`]: crate::wspr::ddc::StreamingDdc
pub struct FirStage {
    /// Taps in **reverse** order, so the dot product walks history
    /// forwards and both operands are sequential.
    taps_rev: Vec<f32>,
    /// Linear (not circular) history, compacted rarely. A ring costs a
    /// wrap test per tap, which defeats unrolling on the one loop that
    /// matters; see [`Self::dot`].
    hist_i: Vec<f32>,
    hist_q: Vec<f32>,
    /// Live samples in `hist_*`.
    hist_len: usize,
    /// Index where the current `ntaps` window begins, maintained
    /// incrementally rather than as `hist_len - ntaps` — same
    /// constant-folding pitfall `wspr::ddc::StreamingDdc::win_start`
    /// documents (the `esp` Xtensa fork rejects the resulting negative
    /// addressing offset for large `ntaps`); harmless to keep the same
    /// shape here even for this module's smaller stages.
    win_start: usize,
    /// Input samples until the next output.
    to_next_out: usize,
    decim: usize,
}

impl FirStage {
    /// `ntaps` must be odd (linear phase, integer group delay).
    /// `fc_norm` is the lowpass cutoff normalised to this stage's
    /// *input* sample rate. `hist_margin` sets how many input samples
    /// accumulate between `compact` calls — larger amortises
    /// the compaction copy further at the cost of a bigger buffer; see
    /// the caller for the tradeoff against a specific memory budget.
    pub fn new(ntaps: usize, decim: usize, fc_norm: f32, hist_margin: usize) -> Self {
        assert!(ntaps % 2 == 1, "ntaps must be odd for linear phase");
        let designed = design_lowpass(ntaps, fc_norm);
        let mut taps_rev = vec![0.0f32; ntaps];
        // Index loop, not `.iter().rev()` — see the field doc comment
        // on `win_start` for why (this mirrors `StreamingDdc::new_in`'s
        // own identical workaround).
        let last = ntaps - 1;
        for k in 0..ntaps {
            taps_rev[k] = designed[last - k];
        }
        let hist_cap = ntaps + hist_margin;
        let mut hist_i = vec![0.0f32; hist_cap];
        let mut hist_q = vec![0.0f32; hist_cap];
        // Pre-filled with the zeros the filter would have seen before
        // the stream started.
        hist_i[..ntaps].fill(0.0);
        hist_q[..ntaps].fill(0.0);
        let group_delay = (ntaps - 1) / 2;
        Self {
            taps_rev,
            hist_i,
            hist_q,
            hist_len: ntaps,
            win_start: 0,
            // The first output is centred on input sample 0, which the
            // filter only sees once `group_delay` more samples have
            // arrived.
            to_next_out: group_delay + 1,
            decim,
        }
    }

    pub fn ntaps(&self) -> usize {
        self.taps_rev.len()
    }

    /// This stage's group delay, in its own input samples.
    pub fn group_delay(&self) -> usize {
        (self.ntaps() - 1) / 2
    }

    /// Push one complex input sample; returns `Some((i, q))` on the
    /// samples where this stage's decimation produces an output.
    /// Un-normalised — any output gain (e.g. matching a reference
    /// implementation's amplitude scale) is the caller's job, applied
    /// once after however many stages it chains.
    pub fn push_one(&mut self, i: f32, q: f32) -> Option<(f32, f32)> {
        self.hist_i[self.hist_len] = i;
        self.hist_q[self.hist_len] = q;
        self.hist_len += 1;
        self.win_start += 1;

        let mut out = None;
        self.to_next_out -= 1;
        if self.to_next_out == 0 {
            self.to_next_out = self.decim;
            out = Some(self.dot());
        }

        if self.hist_len == self.hist_i.len() {
            self.compact();
        }
        out
    }

    /// Block-mode counterpart to [`push_one`](Self::push_one): consume
    /// `xi.len()` complex input samples at once, appending any
    /// outputs this stage completes. Behaviourally identical to
    /// calling [`push_one`](Self::push_one) once per sample — exists
    /// so callers can amortise per-call overhead over a block rather
    /// than a sample, which is what an esp-dsp-backed `FirDecimator`
    /// (`docs/notes/FST4_DDC_DESIGN.md` §4.5) needs: `dsps_fird_f32_aes3`
    /// is itself block-shaped, so a sample-at-a-time `push_one` there
    /// would pay the FFI hop per sample instead of per block.
    pub fn push_block(
        &mut self,
        xi: &[f32],
        xq: &[f32],
        out_i: &mut Vec<f32>,
        out_q: &mut Vec<f32>,
    ) {
        assert_eq!(xi.len(), xq.len(), "I/Q blocks must be the same length");
        for (&i, &q) in xi.iter().zip(xq.iter()) {
            if let Some((oi, oq)) = self.push_one(i, q) {
                out_i.push(oi);
                out_q.push(oq);
            }
        }
    }

    fn compact(&mut self) {
        // `win_start` is the same quantity `hist_len - ntaps` would
        // give, already maintained without the constant-folded
        // subtraction — see its field comment.
        let keep = self.win_start;
        let ntaps = self.ntaps();
        self.hist_i.copy_within(keep..self.hist_len, 0);
        self.hist_q.copy_within(keep..self.hist_len, 0);
        self.hist_len = ntaps;
        self.win_start = 0;
    }

    /// `Σ h_rev[k]·x[n-ntaps+k]`, over both channels.
    ///
    /// Four partial sums per channel rather than one — the FIR is an
    /// accumulation chain and Xtensa's `fadd` has ~3-4 cycle latency
    /// against 1-cycle throughput, so a single accumulator spends most
    /// of its cycles waiting on itself; independent partials fill the
    /// issue slots (same reasoning `wspr::ddc::StreamingDdc::dot`
    /// documents, which this mirrors).
    fn dot(&self) -> (f32, f32) {
        let ntaps = self.ntaps();
        let a = self.win_start;
        let hi = &self.hist_i[a..a + ntaps];
        let hq = &self.hist_q[a..a + ntaps];
        let h = &self.taps_rev[..];

        let mut ai = [0.0f32; 4];
        let mut aq = [0.0f32; 4];
        let chunks = ntaps / 4;
        for c in 0..chunks {
            let b = c * 4;
            for l in 0..4 {
                ai[l] += h[b + l] * hi[b + l];
                aq[l] += h[b + l] * hq[b + l];
            }
        }
        let mut si = ai[0] + ai[1] + ai[2] + ai[3];
        let mut sq = aq[0] + aq[1] + aq[2] + aq[3];
        // ntaps is odd, so a remainder of up to 3 is left over.
        for k in chunks * 4..ntaps {
            si += h[k] * hi[k];
            sq += h[k] * hq[k];
        }
        (si, sq)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `push_block` is defined as "the same as `push_one` in a loop" —
    /// this pins that down bit-for-bit, including across a compaction
    /// boundary (`hist_margin` small enough that a 500-sample block
    /// forces at least one `compact()`).
    #[test]
    fn push_block_matches_repeated_push_one() {
        let audio: Vec<f32> = (0..500).map(|k| (k as f32 * 0.037).sin()).collect();

        let mut one = FirStage::new(31, 4, 0.1, 32);
        let mut oi = Vec::new();
        let mut oq = Vec::new();
        for &s in &audio {
            if let Some((i, q)) = one.push_one(s, -s) {
                oi.push(i);
                oq.push(q);
            }
        }

        let mut block = FirStage::new(31, 4, 0.1, 32);
        let neg: Vec<f32> = audio.iter().map(|&s| -s).collect();
        let mut bi = Vec::new();
        let mut bq = Vec::new();
        block.push_block(&audio, &neg, &mut bi, &mut bq);

        assert_eq!(oi, bi);
        assert_eq!(oq, bq);
    }
}
