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

// Unconditional: the history buffers use it in both builds — see
// its doc comment for why alignment is not only the backend's concern.
use super::dotprod::AlignedF32;

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
    hist_i: AlignedF32,
    hist_q: AlignedF32,
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
    /// Live-region capacity, i.e. where [`compact`](Self::compact)
    /// triggers.
    ///
    /// Not `hist_i.len()`: on a backend build the dot reads `ntaps`
    /// rounded up to a multiple of four, so the last window overruns
    /// the live region by up to three samples. The buffers carry that
    /// slack past `hist_cap` — zeros, met by the zero-padded taps —
    /// and the trigger has to be the cap rather than the allocation,
    /// or `win_start + pad` walks off the end. (It did: `range end
    /// index 777 out of range for slice of length 776`, stage B's
    /// 263 taps padded to 264, on the first flash of this path.)
    hist_cap: usize,

    /// [`taps_rev`](Self::taps_rev) zero-padded to a multiple of four
    /// and 16-byte aligned, plus matching staging for the two history
    /// windows.
    ///
    /// One zero-padded, 16-byte-aligned tap table **per window phase**.
    ///
    /// `win_start` advances by `decim` per output, so the window's
    /// 16-byte residue cycles — every residue for `decim = 1`,
    /// alternating for `decim = 18` — and the backend's PIE path needs
    /// both operands aligned. Phase `p` carries `p` leading zeros, so
    /// the dot can start at `win_start - p`, which *is* 4-aligned, and
    /// still multiply tap `j` by history `win_start + j`. Every other
    /// term is exactly `0.0 · x`.
    ///
    /// This is the "four pre-shifted tap tables" `esp_dsp_dotprod`'s
    /// module doc weighs and rejects — at 168 KB for FST4's wideband
    /// coarse cascade (L = 64) against ~190 KB of free internal DRAM.
    /// That arithmetic does not carry to here: `ft4::ddc` runs 199 and
    /// 263 taps, so four phases is **about 7.4 KB for both stages**.
    /// The measurement it was weighed against — copying the window
    /// instead, "roughly a third of the gain" — was confirmed the hard
    /// way on 2026-08-30, where the copy cost more than the dot saved
    /// (`docs/notes/FT4_BENCHMARK.md` §27).
    ///
    /// Behind `dotprod-extern` because a host build has no backend to
    /// satisfy: it would carry four tables and different rounding for
    /// nothing, so it keeps the single reversed-tap dot it always had.
    #[cfg(feature = "dotprod-extern")]
    taps_phase: [AlignedF32; 4],
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
        // The dot reads `ntaps` on a host build and `ntaps` rounded up
        // to a multiple of four on a backend build, so the buffer is
        // sized for the larger. `AlignedF32::new` zero-fills, which is
        // also the zeros the filter would have seen before the stream
        // started.
        let hist_cap = ntaps + hist_margin;
        // Slack so the longest phase-shifted window — three leading
        // zeros, then `ntaps`, rounded up to a multiple of four —
        // starting at the last legal `win_start` still lies inside the
        // allocation. The window starts *below* `win_start` by the
        // phase, which never underflows since the phase is
        // `win_start % 4`.
        let hist_alloc = hist_cap + (3 + ntaps).next_multiple_of(4) - ntaps;
        let hist_i = AlignedF32::new(hist_alloc);
        let hist_q = AlignedF32::new(hist_alloc);
        let group_delay = (ntaps - 1) / 2;
        #[cfg(feature = "dotprod-extern")]
        let taps_phase = core::array::from_fn(|phase| {
            // Zeros before and after, so every added term is exactly
            // `0.0 · x` and only the rounding of the sum differs.
            let mut t = AlignedF32::new(phase + ntaps);
            t.as_mut_slice()[phase..phase + ntaps].copy_from_slice(&taps_rev);
            t
        });
        Self {
            #[cfg(feature = "dotprod-extern")]
            taps_phase,
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
            hist_cap,
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
        self.hist_i.as_mut_slice()[self.hist_len] = i;
        self.hist_q.as_mut_slice()[self.hist_len] = q;
        self.hist_len += 1;
        self.win_start += 1;

        let mut out = None;
        self.to_next_out -= 1;
        if self.to_next_out == 0 {
            self.to_next_out = self.decim;
            out = Some(self.dot());
        }

        if self.hist_len == self.hist_cap {
            self.compact();
        }
        out
    }

    /// Block-mode counterpart to [`push_one`](Self::push_one): consume
    /// `xi.len()` complex input samples at once, appending any outputs
    /// this stage completes. Behaviourally identical to calling
    /// [`push_one`](Self::push_one) once per sample, and pinned that
    /// way bit-for-bit by `push_block_matches_repeated_push_one` —
    /// the dots see the same windows in the same order, only the
    /// stores are batched.
    ///
    /// **Why it has its own body.** Until 2026-08-30 this *was*
    /// `push_one` in a loop, and `ft4::ddc` never called it at all —
    /// `CandidateDdc::push_i16` mixed and pushed one sample at a time,
    /// so stage A saw 90 000 individual pushes to produce 4 995
    /// outputs. Measured on a CoreS3, that stage cost 70 ms per
    /// candidate of which the dot products were ~16: the other ~54 ms,
    /// **about 648 ms per FT4 slot and 20 % of its decode budget**, was
    /// two bounds-checked stores, a counter and a compaction test per
    /// input sample (`docs/notes/FT4_BENCHMARK.md` §30.1).
    pub fn push_block(
        &mut self,
        xi: &[f32],
        xq: &[f32],
        out_i: &mut Vec<f32>,
        out_q: &mut Vec<f32>,
    ) {
        assert_eq!(xi.len(), xq.len(), "I/Q blocks must be the same length");
        let ntaps = self.ntaps();
        let mut k = 0usize;
        while k < xi.len() {
            // Never write past `hist_cap`; that is where `compact`
            // triggers, and the allocation's slack past it belongs to
            // the padded dot window, not to input.
            let take = (self.hist_cap - self.hist_len).min(xi.len() - k);
            debug_assert!(take > 0);

            // The point of the whole function: one bulk append instead
            // of `take` separate stores with their own bounds checks.
            let hl = self.hist_len;
            self.hist_i.as_mut_slice()[hl..hl + take].copy_from_slice(&xi[k..k + take]);
            self.hist_q.as_mut_slice()[hl..hl + take].copy_from_slice(&xq[k..k + take]);

            // Emit every output whose window closes inside this run.
            // `hist_len`/`win_start` are advanced by addition only,
            // never recomputed as `hist_len - ntaps` — see the field
            // comment on `win_start` for the Xtensa codegen reason.
            let mut consumed = 0usize;
            while self.to_next_out <= take - consumed {
                let step = self.to_next_out;
                consumed += step;
                self.hist_len += step;
                self.win_start += step;
                self.to_next_out = self.decim;
                let (oi, oq) = self.dot();
                out_i.push(oi);
                out_q.push(oq);
            }

            // The tail of the run produced no output; it still advances
            // the history and the countdown.
            let rest = take - consumed;
            self.to_next_out -= rest;
            self.hist_len += rest;
            self.win_start += rest;
            debug_assert_eq!(self.hist_len, self.win_start + ntaps);

            k += take;
            if self.hist_len == self.hist_cap {
                self.compact();
            }
        }
    }

    fn compact(&mut self) {
        // `win_start` is the same quantity `hist_len - ntaps` would
        // give, already maintained without the constant-folded
        // subtraction — see its field comment.
        let keep = self.win_start;
        let ntaps = self.ntaps();
        self.hist_i
            .as_mut_slice()
            .copy_within(keep..self.hist_len, 0);
        self.hist_q
            .as_mut_slice()
            .copy_within(keep..self.hist_len, 0);
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
    #[cfg(not(feature = "dotprod-extern"))]
    fn dot(&mut self) -> (f32, f32) {
        let ntaps = self.ntaps();
        let a = self.win_start;
        let hi = &self.hist_i.as_slice()[a..a + ntaps];
        let hq = &self.hist_q.as_slice()[a..a + ntaps];
        let h = &self.taps_rev[..];

        // The four-partial-sum unroll this used to spell out inline now
        // lives in `dotprod::dot_f32_portable`, which `dot_f32` falls
        // back to when no backend is configured — same arithmetic, and
        // an embedded build can now route it to esp-dsp instead
        // (issue #307).
        (
            super::dotprod::dot_f32(h, hi),
            super::dotprod::dot_f32(h, hq),
        )
    }

    /// Backend build: stage both windows into 16-byte-aligned,
    /// four-multiple buffers first, so `dot_f32`'s backend can take its
    /// PIE path. See [`FirStage::taps_pad`] for why the copy pays.
    ///
    /// The padded tail of all three buffers is zero and is never
    /// written, so the extra terms are exactly `0.0 · 0.0`.
    #[cfg(feature = "dotprod-extern")]
    fn dot(&mut self) -> (f32, f32) {
        let a = self.win_start;
        // 16 bytes is four `f32`, so the phase is the window start's
        // residue mod 4 and `a - phase` is always 16-byte aligned given
        // an aligned base — which is why `hist_i` is `AlignedF32`.
        let phase = a % 4;
        let h = self.taps_phase[phase].as_slice();
        let (lo, len) = (a - phase, h.len());
        (
            super::dotprod::dot_f32(h, &self.hist_i.as_slice()[lo..lo + len]),
            super::dotprod::dot_f32(h, &self.hist_q.as_slice()[lo..lo + len]),
        )
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

    /// The same equivalence across the shapes the 2026-08-30 block-mode
    /// body actually branches on, which one 500-sample / `decim = 4`
    /// case does not reach.
    ///
    /// `push_block` splits its input into runs bounded by the next
    /// compaction, emits every output whose window closes inside a run,
    /// then carries the remainder in `to_next_out`. The cases that
    /// exercise the seams are: a decimation of 1 (an output per input,
    /// so the inner loop runs `take` times), a large decimation with a
    /// long startup delay (`to_next_out` exceeds a whole run and the
    /// inner loop never runs), a block far larger than the history
    /// capacity (many runs), and a block of exactly one sample.
    ///
    /// The FT4 DDC's own two stages — 199 taps / `decim = 18` and 263
    /// taps / `decim = 1` — are in the list on purpose: this is the
    /// pair the change was made for.
    #[test]
    fn push_block_matches_push_one_across_shapes() {
        let audio: Vec<f32> = (0..4000).map(|k| (k as f32 * 0.011).sin()).collect();
        let neg: Vec<f32> = audio.iter().map(|&s| -s).collect();

        for &(ntaps, decim, margin, chunk) in &[
            (31usize, 1usize, 32usize, 500usize),
            (31, 4, 32, 500),
            (31, 7, 16, 1),
            (199, 18, 512, 1024),
            (263, 1, 512, 1024),
            (63, 5, 8, 4000),
            (15, 3, 4, 3),
        ] {
            let mut one = FirStage::new(ntaps, decim, 0.1, margin);
            let (mut oi, mut oq) = (Vec::new(), Vec::new());
            for (&i, &q) in audio.iter().zip(neg.iter()) {
                if let Some((a, b)) = one.push_one(i, q) {
                    oi.push(a);
                    oq.push(b);
                }
            }

            let mut block = FirStage::new(ntaps, decim, 0.1, margin);
            let (mut bi, mut bq) = (Vec::new(), Vec::new());
            for (ci, cq) in audio.chunks(chunk).zip(neg.chunks(chunk)) {
                block.push_block(ci, cq, &mut bi, &mut bq);
            }

            let label = format!("ntaps={ntaps} decim={decim} margin={margin} chunk={chunk}");
            assert_eq!(oi.len(), bi.len(), "{label}: output count");
            assert_eq!(oi, bi, "{label}: I");
            assert_eq!(oq, bq, "{label}: Q");
            assert!(!oi.is_empty(), "{label}: produced nothing to compare");
        }
    }
}
