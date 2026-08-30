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

/// Four `f32` under a 16-byte alignment guarantee — the backing unit
/// for [`AlignedF32`].
#[repr(align(16))]
#[derive(Clone, Copy)]
struct AlignedQuad(
    // Read only through the reinterpreting slices in
    // [`AlignedF32::as_slice`] / [`AlignedF32::as_mut_slice`] — the
    // field exists to size and align the backing store. Same shape and
    // same `allow` as `embedded-shared`'s `Align16Quad`.
    #[allow(dead_code)] [f32; 4],
);

/// A zero-filled `f32` buffer whose base is 16-byte aligned and whose
/// length is a multiple of four.
///
/// Both are preconditions of `dsps_dotprod_f32_aes3`'s PIE path;
/// missing either drops it to a scalar loop. Measured on a CoreS3
/// (`docs/notes/FT4_BENCHMARK.md` §27): **7 900 ps/tap when both hold,
/// 18 080 when either does not** — 2.3x, and `ft4::ddc`'s 199- and
/// 263-tap stages satisfied *neither*, so every dot the DDC has ever
/// done took the slow path.
///
/// The history buffers use it too, unconditionally: a `Vec<f32>` is
/// only guaranteed 4-byte aligned, so a window starting at a
/// four-multiple index would still not be 16-byte aligned and the
/// backend would take the slow path anyway. Making the base aligned
/// costs nothing and changes no arithmetic.
struct AlignedF32 {
    quads: Vec<AlignedQuad>,
    len: usize,
}

impl AlignedF32 {
    /// Rounds `len` **up** to a multiple of four and zero-fills.
    fn new(len: usize) -> Self {
        let quads = len.div_ceil(4);
        Self {
            quads: vec![AlignedQuad([0.0; 4]); quads],
            len: quads * 4,
        }
    }

    fn as_slice(&self) -> &[f32] {
        // SAFETY: `AlignedQuad` is `repr(align(16))` over `[f32; 4]`,
        // so the store is exactly `self.len` contiguous `f32`.
        unsafe { core::slice::from_raw_parts(self.quads.as_ptr() as *const f32, self.len) }
    }

    fn as_mut_slice(&mut self) -> &mut [f32] {
        // SAFETY: as `as_slice`.
        unsafe { core::slice::from_raw_parts_mut(self.quads.as_mut_ptr() as *mut f32, self.len) }
    }
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
}
