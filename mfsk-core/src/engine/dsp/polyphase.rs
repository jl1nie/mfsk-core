// SPDX-License-Identifier: GPL-3.0-or-later
//! Streaming rational (`L`/`M`) resampler over a complex (I, Q) history.
//!
//! [`FirStage`](super::fir_decimate::FirStage) only decimates by an
//! integer factor. This is its rational sibling — needed wherever the
//! target rate is not an integer divisor of the input rate. The FST4
//! embedded DDC needs exactly one such stage, right before
//! `compute_spectra`, because `NSPS = 3888 = 2^4·3^5` has no integer
//! divisor that lands on a power-of-two `nfft1` — see
//! `docs/notes/FST4_DDC_DESIGN.md` §1, §4.2.
//!
//! ## The identity
//!
//! Implements "insert `L-1` zeros between samples, low-pass, keep
//! every `M`-th sample" without ever materialising the zero-stuffed
//! intermediate. For a unit-DC-gain prototype `h` (length `ntaps`,
//! [`design_lowpass`] as usual, scaled by `L` to restore the passband
//! gain the zero-stuffing would otherwise dilute), output `y[m]` is
//!
//! ```text
//! y[m] = Σ_j h[p + j·L] · x[n0 - j],   p = (m·M) mod L,   n0 = ⌊m·M/L⌋
//! ```
//!
//! i.e. only the taps at phase `p` participate — a `⌈ntaps/L⌉`-deep
//! dot product per output, no multiplies against the zeros a literal
//! upsample would introduce. Consecutive outputs' `n0` differ by
//! `⌊(p+M)/L⌋`, with the same quantity giving the next phase
//! (`(p+M) mod L`) — a Bresenham-style carry, so the whole thing runs
//! as one pass over the input stream with no division in the hot path.
//!
//! Each phase's taps are stored pre-reversed and zero-padded to a
//! common `max_depth`, so every phase's dot product walks the same
//! trailing window shape [`FirStage::dot`](super::fir_decimate::FirStage)
//! does — same history-buffer/compaction structure, reused here.

use alloc::vec;
use alloc::vec::Vec;

use super::dotprod::dot_f32;
use super::fir_decimate::design_lowpass;

/// Streaming `L`/`M` rational resampler over a complex (I, Q) stream.
/// Input at rate `Fs`, output at rate `Fs·L/M`.
pub struct PolyphaseResampler {
    l: u32,
    m: u32,
    ntaps: usize,
    /// Per-phase reversed tap tables, `l` of them, each `max_depth`
    /// long and zero-padded at the front — see the module doc comment.
    taps_rev: Vec<Vec<f32>>,
    max_depth: usize,
    hist_i: Vec<f32>,
    hist_q: Vec<f32>,
    hist_len: usize,
    win_start: usize,
    /// Current output phase `p = (m·M) mod L`, for the *next* output
    /// not yet produced.
    phase: u32,
    /// Absolute input-sample index of the newest sample the next
    /// output needs (`n0` for that same next output).
    next_n0: usize,
    /// Absolute index of the newest sample pushed so far (`None`
    /// before the first [`push`](Self::push)).
    n_in_abs: Option<usize>,
}

impl PolyphaseResampler {
    /// `ntaps` must be odd, matching [`design_lowpass`]'s symmetric
    /// design. `hist_margin` behaves like
    /// [`FirStage::new`](super::fir_decimate::FirStage::new)'s: how
    /// many extra input samples accumulate between compactions.
    pub fn new(l: u32, m: u32, ntaps: usize, hist_margin: usize) -> Self {
        assert!(ntaps % 2 == 1, "ntaps must be odd for linear phase");
        assert!(l > 0 && m > 0, "l and m must be nonzero");

        let fc_norm = (1.0 / (2.0 * l as f32)).min(1.0 / (2.0 * m as f32));
        let mut h = design_lowpass(ntaps, fc_norm);
        for tap in h.iter_mut() {
            *tap *= l as f32;
        }

        let max_depth = ntaps.div_ceil(l as usize);
        let mut taps_rev = Vec::with_capacity(l as usize);
        for p in 0..l as usize {
            let mut phase_taps = vec![0.0f32; max_depth];
            let mut j = 0usize;
            while p + j * (l as usize) < ntaps {
                // window[k] holds x[n0-(max_depth-1-k)] (oldest at 0,
                // newest at max_depth-1 — same convention FirStage's
                // taps_rev uses), so the j-th tap (multiplying
                // x[n0-j]) lands at k = max_depth-1-j.
                phase_taps[max_depth - 1 - j] = h[p + j * (l as usize)];
                j += 1;
            }
            taps_rev.push(phase_taps);
        }

        let hist_cap = max_depth + hist_margin;
        let hist_i = vec![0.0f32; hist_cap];
        let hist_q = vec![0.0f32; hist_cap];

        Self {
            l,
            m,
            ntaps,
            taps_rev,
            max_depth,
            hist_i,
            hist_q,
            hist_len: max_depth,
            win_start: 0,
            phase: 0,
            next_n0: 0,
            n_in_abs: None,
        }
    }

    /// This stage's group delay, in *output* samples (`(ntaps-1)/2`
    /// at the interpolated rate, converted down by `M`) — for a
    /// caller that wants to trim the transient the way
    /// [`StreamingDdc::flush`](crate::wspr::ddc::StreamingDdc::flush)
    /// does for the integer case.
    pub fn group_delay_output(&self) -> usize {
        ((self.ntaps - 1) / 2) / self.m as usize
    }

    /// Push one complex input sample; appends every output it
    /// completes (0, 1, or more — more than 1 only when `l > m`,
    /// upsampling) to the caller's buffers.
    pub fn push(&mut self, i: f32, q: f32, out_i: &mut Vec<f32>, out_q: &mut Vec<f32>) {
        self.hist_i[self.hist_len] = i;
        self.hist_q[self.hist_len] = q;
        self.hist_len += 1;
        self.win_start += 1;
        let cur_abs = self.n_in_abs.map_or(0, |a| a + 1);
        self.n_in_abs = Some(cur_abs);

        while self.next_n0 <= cur_abs {
            // How far behind the newest held sample this output's
            // window ends — always 0 once the stream is running (see
            // the module doc comment), kept general rather than
            // asserted so `l > m` (interpolation) stays correct too.
            let back = cur_abs - self.next_n0;
            let end = self.win_start + self.max_depth - back;
            let start = end - self.max_depth;
            let (oi, oq) = self.dot(start);
            out_i.push(oi);
            out_q.push(oq);

            let carry = (self.phase + self.m) / self.l;
            self.phase = (self.phase + self.m) % self.l;
            self.next_n0 += carry as usize;
        }

        if self.hist_len == self.hist_i.len() {
            self.compact();
        }
    }

    fn compact(&mut self) {
        let keep = self.win_start;
        self.hist_i.copy_within(keep..self.hist_len, 0);
        self.hist_q.copy_within(keep..self.hist_len, 0);
        self.hist_len = self.max_depth;
        self.win_start = 0;
    }

    /// `Σ taps_rev[phase][k]·hist[start+k]`, over both channels — same
    /// shape as [`FirStage::dot`](super::fir_decimate::FirStage), just
    /// against a phase-selected tap table instead of a fixed one.
    fn dot(&self, start: usize) -> (f32, f32) {
        let depth = self.max_depth;
        let hi = &self.hist_i[start..start + depth];
        let hq = &self.hist_q[start..start + depth];
        let h = &self.taps_rev[self.phase as usize][..depth];

        // Two independent real dot products against the same
        // phase-selected tap table. Routed through
        // [`dot_f32`](super::dotprod::dot_f32) so an embedded build can
        // supply an esp-dsp backend: this is the single largest cost in
        // the FST4 embedded DDC pipeline, and the wideband cascade has
        // no integer stages at all, so *all* of its filter work lands
        // here (issue #307 — measured 3.6-9.2x available).
        (dot_f32(h, hi), dot_f32(h, hq))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // Only the test module below calls `f32` methods needing the
    // `Float` trait in `no_std` (design_lowpass's `.sin()`/`.cos()`
    // live in `fir_decimate`, which carries its own copy of this
    // import) — a crate-level `use` outside `#[cfg(test)]` would be
    // unused (and `-D warnings`-denied) on every real no_std build.
    #[cfg(not(feature = "std"))]
    use num_traits::Float;

    /// `f64` phase accumulation — see `wspr::ddc`'s own `tone` helper
    /// for why: `f32` phase noise at large sample counts would put a
    /// floor under any rejection measurement made with it.
    fn tone(freq_hz: f32, fs_hz: f32, amp: f32, n: usize) -> Vec<f32> {
        let w = 2.0 * core::f64::consts::PI * freq_hz as f64 / fs_hz as f64;
        (0..n)
            .map(|k| (amp as f64 * (w * k as f64).cos()) as f32)
            .collect()
    }

    fn run(r: &mut PolyphaseResampler, audio: &[f32]) -> (Vec<f32>, Vec<f32>) {
        let mut out_i = Vec::new();
        let mut out_q = Vec::new();
        for &s in audio {
            r.push(s, 0.0, &mut out_i, &mut out_q);
        }
        (out_i, out_q)
    }

    /// A DC-centred real input, resampled at unity ratio (`L=M=1`),
    /// should reproduce the input on the real channel once the filter
    /// has settled — the ratio-1 case is the identity, so this checks
    /// the phase-carry bookkeeping produces exactly one output per
    /// input with no drift.
    #[test]
    fn unity_ratio_produces_one_output_per_input() {
        let mut r = PolyphaseResampler::new(1, 1, 31, 64);
        let n = 2000;
        let audio = tone(0.0, 12_000.0, 0.7, n);
        let (out_i, out_q) = run(&mut r, &audio);
        assert_eq!(out_i.len(), n);
        assert_eq!(out_q.len(), n);
        let settled = 40..n;
        for k in settled {
            assert!(
                (out_i[k] - 0.7).abs() < 0.01,
                "k={k} out_i={} expected ~0.7",
                out_i[k]
            );
            assert!(
                out_q[k].abs() < 0.01,
                "k={k} out_q={} expected ~0",
                out_q[k]
            );
        }
    }

    /// `ntaps` for a target transition width (Hz) at this `L`/`Fs_in`
    /// pair — `design_lowpass`'s own `4/NTAPS`-of-the-design-rate rule
    /// (see `wspr::ddc::NTAPS`'s doc comment), with the design rate
    /// being the interpolated one, `L·Fs_in`. Odd, for linear phase.
    fn ntaps_for_transition(l: u32, fs_in: f32, transition_hz: f32) -> usize {
        let n = (4.0 * l as f32 * fs_in / transition_hz).ceil() as usize;
        if n.is_multiple_of(2) { n + 1 } else { n }
    }

    /// In-band tone: amplitude preserved (within the filter's ripple)
    /// after resampling by a representative small ratio (`9/64`, the
    /// FST4-60 sniper refine stage per the design doc's §4.4 table).
    #[test]
    fn in_band_tone_amplitude_is_preserved() {
        let l = 9u32;
        let m = 64u32;
        let fs_in = 790.12f32;
        let ntaps = ntaps_for_transition(l, fs_in, 20.0);
        let mut r = PolyphaseResampler::new(l, m, ntaps, 128);
        let n = 20_000;
        // Well inside both the input and output Nyquist (output rate
        // ~111 Hz, so keep the tone under ~50 Hz).
        let audio = tone(10.0, fs_in, 0.5, n);
        let (out_i, out_q) = run(&mut r, &audio);

        let expected_len = (n as u64 * l as u64 / m as u64) as usize;
        assert!(
            out_i.len().abs_diff(expected_len) <= 1,
            "got {} outputs, expected ~{expected_len}",
            out_i.len()
        );

        let settle = out_i.len() / 4;
        let span = settle..(out_i.len() - settle);
        let mut peak = 0.0f32;
        for k in span {
            let mag = (out_i[k] * out_i[k] + out_q[k] * out_q[k]).sqrt();
            if mag > peak {
                peak = mag;
            }
        }
        assert!(
            (peak - 0.5).abs() < 0.05,
            "peak magnitude {peak}, expected ~0.5"
        );
    }

    /// Out-of-band content (above the output Nyquist) must not alias
    /// into the resampled stream at any level that would read as
    /// signal.
    #[test]
    fn out_of_band_tone_is_rejected() {
        let l = 9u32;
        let m = 64u32;
        let fs_in = 790.12f32;
        let fs_out = fs_in * l as f32 / m as f32; // ~111.11 Hz
        let ntaps = ntaps_for_transition(l, fs_in, 20.0);

        let mut in_band = PolyphaseResampler::new(l, m, ntaps, 128);
        let mut out_band = PolyphaseResampler::new(l, m, ntaps, 128);
        let n = 20_000;

        let (i1, q1) = run(&mut in_band, &tone(10.0, fs_in, 0.5, n));
        // Comfortably past fs_out/2 (~55.5 Hz) so it must be
        // suppressed rather than folded back near the passband.
        let (i2, q2) = run(&mut out_band, &tone(fs_out, fs_in, 0.5, n));

        let rms = |i: &[f32], q: &[f32]| -> f32 {
            let settle = i.len() / 4;
            let span = settle..(i.len() - settle);
            let s: f32 =
                span.clone().map(|k| i[k] * i[k] + q[k] * q[k]).sum::<f32>() / span.len() as f32;
            s.sqrt()
        };
        let db = 20.0 * (rms(&i2, &q2) / rms(&i1, &q1)).log10();
        assert!(db < -40.0, "out-of-band rejection only {db} dB");
    }

    /// Down-sampling by an integer ratio expressed rationally (`L=1`)
    /// should track [`FirStage`](super::super::fir_decimate::FirStage)
    /// with an equivalent design — same identity, two code paths.
    #[test]
    fn integer_ratio_tracks_fir_stage() {
        use super::super::fir_decimate::FirStage;

        let decim = 8u32;
        let ntaps = 63;
        let mut poly = PolyphaseResampler::new(1, decim, ntaps, 128);
        let mut fir = FirStage::new(ntaps, decim as usize, 1.0 / (2.0 * decim as f32), 128);

        let n = 4000;
        let audio = tone(20.0, 12_000.0, 0.6, n);
        let (pi, pq) = run(&mut poly, &audio);
        let mut fi = Vec::new();
        let mut fq = Vec::new();
        for &s in &audio {
            if let Some((i, q)) = fir.push_one(s, 0.0) {
                fi.push(i);
                fq.push(q);
            }
        }

        // FirStage emits its first output only once its own group
        // delay has elapsed (so its output 0 is already
        // centre-aligned); PolyphaseResampler emits from n0=0
        // instead, so its stream leads FirStage's by
        // `group_delay_output()` samples. Skip that many before
        // comparing — same shift the design doc's DDC callers would
        // apply via a `flush`-style trim (`wspr::ddc::StreamingDdc`'s
        // own doc comment: "group delay already removed").
        let lead = poly.group_delay_output();
        let (pi, pq) = (&pi[lead..], &pq[lead..]);
        let len = pi.len().min(fi.len());

        let settle = len / 4;
        let span = settle..len;
        let (mut num_re, mut num_im, mut pf, mut pp) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        for k in span {
            let (f, p) = ((fi[k], fq[k]), (pi[k], pq[k]));
            num_re += (f.0 * p.0 + f.1 * p.1) as f64;
            num_im += (f.1 * p.0 - f.0 * p.1) as f64;
            pf += (f.0 * f.0 + f.1 * f.1) as f64;
            pp += (p.0 * p.0 + p.1 * p.1) as f64;
        }
        let coh = (num_re * num_re + num_im * num_im).sqrt() / (pf * pp).sqrt();
        assert!(coh > 0.99, "coherence with FirStage only {coh}");
    }
}
