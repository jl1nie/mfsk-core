//! MSK144 joint (CFO, symbol-timing) search via matched-filter
//! correlation.
//!
//! Ported from WSJT-X `msk144_freq_search.f90` + `msk144sync.f90`.
//! Given a multi-frame analytic-signal buffer at (or near) a candidate
//! center frequency `fc`, searches a small residual-CFO grid and, for
//! each trial: mixes the buffer to baseband, coherently sums the
//! frames selected by `navmask`, and correlates the two embedded 8-bit
//! sync words simultaneously against every one of the 864 possible
//! symbol-timing offsets. A circular shift by `ish` samples over one
//! `NSPM`-periodic buffer is the same as slicing a period-repeated
//! (doubled) buffer at offset `ish`, which is how WSJT-X avoids an
//! explicit per-shift rotation. The CFO trial with the largest
//! correlation peak wins.
//!
//! WSJT-X's OpenMP frequency-range partitioning across up to 4 threads
//! (`msk144sync.f90:59-86`) is a pure max-reduction over independent
//! CFO trials — this port runs the whole range in one pass, which is
//! numerically identical (no thread-dependent variation to
//! replicate).

use alloc::vec;
use alloc::vec::Vec;
use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::dsp::msk::{NSPM, sync_waveform};

/// Frequency-shift (NCO mixdown) an analytic-signal buffer by `f0_hz`,
/// at the fixed MSK144 sample rate of 12 kHz. Matches WSJT-X
/// `tweak1.f90`: the phase accumulator's first output sample already
/// carries one step of rotation (`w = w*wstep` runs *before* the
/// first multiply in the Fortran loop, so `cb(1) = wstep^1 * ca(1)`,
/// not `wstep^0`) — ported literally (`out[0] = wstep^1 * input[0]`)
/// rather than "simplified" to the more obvious `exp(i*dphi*k)`
/// starting at `k=0`. Only the per-frame *coherent averaging* and
/// *sync correlation* consume the result, both self-consistent under
/// either convention, but literal fidelity keeps this cross-checkable
/// against WSJT-X test vectors later.
pub fn tweak1(input: &[Complex32], f0_hz: f32) -> Vec<Complex32> {
    let mut out = vec![Complex32::new(0.0, 0.0); input.len()];
    tweak1_into(&mut out, input, f0_hz);
    out
}

/// [`tweak1`], writing into a caller-supplied buffer instead of
/// allocating a fresh `Vec` — lets a per-trial CFO search loop (see
/// [`msk144_freq_search`]) reuse one buffer across every trial instead
/// of allocating one per trial. `out.len()` must equal `input.len()`.
fn tweak1_into(out: &mut [Complex32], input: &[Complex32], f0_hz: f32) {
    debug_assert_eq!(out.len(), input.len());
    let dphi = 2.0 * core::f32::consts::PI * f0_hz / 12_000.0;
    let wstep = Complex32::new(dphi.cos(), dphi.sin());
    let mut w = Complex32::new(1.0, 0.0);
    for (o, &x) in out.iter_mut().zip(input) {
        w *= wstep;
        *o = w * x;
    }
}

/// Result of [`msk144_freq_search`]: the coherently-averaged,
/// frequency-corrected 864-sample frame at the best-scoring CFO
/// trial, plus the full correlation-magnitude array (unscaled `|cc|`,
/// matching WSJT-X's `xccs` — see `msk144_freq_search.f90:39-46`)
/// used to peak-pick sync candidates.
#[derive(Clone)]
pub struct FreqSearchResult {
    /// Coherently-averaged 864-sample complex frame, mixed to the
    /// winning trial frequency. Length always [`NSPM`].
    pub frame: Vec<Complex32>,
    /// Correlation magnitude (unscaled) at each of the 864 candidate
    /// symbol-timing shifts, for the winning trial. Length always
    /// [`NSPM`].
    pub xcc: Vec<f32>,
    /// Best correlation-peak metric found across the CFO grid
    /// (normalized by pulse energy and averaged-frame count; compare
    /// against the WSJT-X detection threshold of 1.3).
    pub xmax: f32,
    /// Winning frequency estimate (Hz): `fc + ferr`.
    pub fest: f32,
}

/// Joint (CFO, symbol-timing) matched-filter search.
///
/// `cdat` is a `nframes * NSPM`-sample analytic-signal buffer (still
/// at its native passband frequency, *not* yet mixed to baseband).
/// `fc` is a candidate center frequency (Hz); the search tries every
/// `ferr = ifr * delf` for `ifr` in `if1..=if2` and returns the trial
/// that maximizes the sync-word correlation peak. `navmask[i]`
/// selects which of the `nframes` frames to coherently sum before
/// correlating.
///
/// Ported from `msk144_freq_search.f90`.
pub fn msk144_freq_search(
    cdat: &[Complex32],
    fc: f32,
    if1: i32,
    if2: i32,
    delf: f32,
    nframes: usize,
    navmask: &[bool],
) -> FreqSearchResult {
    assert_eq!(
        cdat.len(),
        nframes * NSPM,
        "cdat must hold exactly nframes*NSPM samples"
    );
    assert_eq!(navmask.len(), nframes, "navmask must have nframes entries");

    let navg = navmask.iter().filter(|&&b| b).count().max(1);
    let fac = 1.0 / (48.0 * (navg as f32).sqrt());
    let cb = sync_waveform();

    let mut best_xmax = 0.0f32;
    let mut best_fest = fc;
    let mut best_frame = vec![Complex32::new(0.0, 0.0); NSPM];
    let mut best_xcc = vec![0.0f32; NSPM];

    // Scratch buffers reused across every CFO trial instead of
    // allocated fresh per trial — this loop runs `if2-if1+1` times per
    // call, and this function itself runs once per (candidate, dither,
    // navmask) combination in the outer decode loop, so the four
    // `vec![...]` allocations this replaces (`mixed`/`c`/`ct2`/`xcc`)
    // were a real hot-loop allocation source, same class as the
    // `fill_sync2d_row!`/`fill_sync2d_row!`-adjacent fix in
    // `engine::sync::coarse_sync`. `best_frame`/`best_xcc` only copy
    // from the scratch buffers on an actual improvement (rare relative
    // to the trial count), not every iteration.
    let mut mixed = vec![Complex32::new(0.0, 0.0); cdat.len()];
    let mut c = vec![Complex32::new(0.0, 0.0); NSPM];
    let mut ct2 = vec![Complex32::new(0.0, 0.0); 2 * NSPM];
    let mut xcc = vec![0.0f32; NSPM];

    for ifr in if1..=if2 {
        let ferr = ifr as f32 * delf;
        tweak1_into(&mut mixed, cdat, -(fc + ferr));

        // Coherent sum of the selected frames.
        c.fill(Complex32::new(0.0, 0.0));
        for i in 0..nframes {
            if navmask[i] {
                let ib = i * NSPM;
                for k in 0..NSPM {
                    c[k] += mixed[ib + k];
                }
            }
        }

        // Doubled buffer: a circular shift by `ish` samples over the
        // NSPM-periodic `c` is a plain slice of `[c, c]` at offset `ish`.
        ct2[0..NSPM].copy_from_slice(&c);
        ct2[NSPM..2 * NSPM].copy_from_slice(&c);

        // Correlate both embedded sync words (42-sample template, 336
        // samples = 56 symbols apart) simultaneously at every shift.
        let mut xmax_this = 0.0f32;
        for ish in 0..NSPM {
            let mut cc = Complex32::new(0.0, 0.0);
            for k in 0..42 {
                let x = ct2[ish + k] + ct2[336 + ish + k];
                // Fortran DOT_PRODUCT(a, b) = sum(conjg(a) * b).
                cc += x.conj() * cb[k];
            }
            let mag = cc.norm();
            xcc[ish] = mag;
            if mag > xmax_this {
                xmax_this = mag;
            }
        }

        let xb = xmax_this * fac;
        if xb > best_xmax {
            best_xmax = xb;
            best_fest = fc + ferr;
            best_frame.copy_from_slice(&c);
            best_xcc.copy_from_slice(&xcc);
        }
    }

    FreqSearchResult {
        frame: best_frame,
        xcc: best_xcc,
        xmax: best_xmax,
        fest: best_fest,
    }
}

/// One sync-timing candidate: a peak in the correlation-magnitude
/// array.
#[derive(Clone, Copy, Debug)]
pub struct SyncPeak {
    /// 0-based sample shift within the coherently-averaged frame where
    /// the sync-word correlation peaks (i.e. rotating the frame left
    /// by this many samples aligns bit 1 to sample 0 — see
    /// [`rotate_to_shift`]).
    pub shift: usize,
    /// Correlation magnitude (unscaled `|cc|`) at that peak.
    pub amplitude: f32,
}

/// Result of a full sync search: [`msk144_freq_search`] plus
/// peak-picking and the WSJT-X detection threshold.
#[derive(Clone)]
pub struct SyncResult {
    /// Coherently-averaged, frequency-corrected frame at the
    /// winning CFO trial. Length always [`NSPM`].
    pub frame: Vec<Complex32>,
    pub xmax: f32,
    pub fest: f32,
    /// Up to `npeaks` strongest, mutually-exclusive (>=8 samples
    /// apart — see below) correlation peaks, strongest first.
    pub peaks: Vec<SyncPeak>,
    /// `xmax >= 1.3` (`msk144sync.f90:98`).
    pub success: bool,
}

/// Full sync search: [`msk144_freq_search`] over `ferr` in
/// `[-ntol, +ntol]` (step `delf`), then pick the `npeaks` strongest
/// correlation peaks. Ported from `msk144sync.f90`.
///
/// Peak exclusion window: WSJT-X zeroes out `xcc(max(0,ic2-7)
/// .. min(NSPM-1,ic2+7))` where `ic2` is `MAXLOC`'s **1-based
/// position** (`ic2 = shift + 1` for our 0-based `shift`) — so in
/// 0-based terms the excluded window is `[shift-6, shift+8]`, not the
/// symmetric `[shift-7, shift+7]` the comment suggests. This is very
/// likely an accidental artifact of Fortran's 1-based `MAXLOC` mixed
/// with the array's explicit `0:NSPM-1` declaration, not deliberate
/// design — but this port replicates it exactly rather than
/// "fixing" it, matching this crate's convention of literal fidelity
/// to WSJT-X's actual (if quirky) behaviour.
pub fn msk144_sync(
    cdat: &[Complex32],
    nframes: usize,
    ntol: f32,
    delf: f32,
    navmask: &[bool],
    npeaks: usize,
    fc: f32,
) -> SyncResult {
    let n_half = (ntol / delf).round() as i32;
    let search = msk144_freq_search(cdat, fc, -n_half, n_half, delf, nframes, navmask);

    let mut xcc = search.xcc.clone();
    let mut peaks = Vec::with_capacity(npeaks);
    for _ in 0..npeaks {
        let (idx, &amp) = xcc
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .expect("xcc is non-empty");
        peaks.push(SyncPeak {
            shift: idx,
            amplitude: amp,
        });
        let lo = idx.saturating_sub(6);
        let hi = (idx + 8).min(NSPM - 1);
        for v in &mut xcc[lo..=hi] {
            *v = 0.0;
        }
    }

    let success = search.xmax >= 1.3;

    SyncResult {
        frame: search.frame,
        xmax: search.xmax,
        fest: search.fest,
        peaks,
        success,
    }
}

/// Circularly rotate a coherently-averaged frame so the sample at
/// `shift` (as reported by [`SyncPeak::shift`]) lands at index 0 —
/// i.e. `aligned[k] = frame[(k + shift) % NSPM]`. Produces a frame
/// ready for [`crate::engine::dsp::msk::matched_filter_softbits`].
pub fn rotate_to_shift(frame: &[Complex32], shift: usize) -> Vec<Complex32> {
    assert_eq!(frame.len(), NSPM, "frame must hold exactly NSPM samples");
    // Two contiguous copies instead of a per-element `% NSPM` index —
    // the access pattern is already two contiguous runs (`ct2`'s own
    // doubled-buffer copy a few lines above this function uses exactly
    // this shape), the per-element modulo just obscured it from the
    // optimizer.
    let shift = shift % NSPM;
    let mut out = vec![Complex32::new(0.0, 0.0); NSPM];
    out[..NSPM - shift].copy_from_slice(&frame[shift..]);
    out[NSPM - shift..].copy_from_slice(&frame[..shift]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::FecCodec;
    use crate::engine::dsp::msk::{build_bitseq, matched_filter_softbits, synth_frame};
    use crate::fec::Ldpc128_90;

    fn build_info_with_crc(pattern: impl Fn(usize) -> u8) -> [u8; 90] {
        let mut info = [0u8; 90];
        for i in 0..77 {
            info[i] = pattern(i);
        }
        let mut bytes = [0u8; 12];
        for (i, &b) in info[..77].iter().enumerate() {
            let byte_idx = i / 8;
            let bit_pos = 7 - (i % 8);
            bytes[byte_idx] |= (b & 1) << bit_pos;
        }
        let crc = crate::fec::ldpc_128_90::crc13(&bytes);
        for i in 0..13 {
            info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
        }
        info
    }

    fn synth_clean_frame(pattern: impl Fn(usize) -> u8) -> ([u8; 90], Vec<Complex32>) {
        let info = build_info_with_crc(pattern);
        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        let bitseq = build_bitseq(&codeword);
        let frame = synth_frame(&bitseq);
        (info, frame.to_vec())
    }

    fn upconvert(frame: &[Complex32], f0_hz: f32) -> Vec<Complex32> {
        tweak1(frame, f0_hz)
    }

    #[test]
    fn tweak1_matches_direct_nco() {
        let input: Vec<Complex32> = (0..10).map(|_| Complex32::new(1.0, 0.0)).collect();
        let out = tweak1(&input, 100.0);
        let dphi = 2.0 * core::f32::consts::PI * 100.0 / 12_000.0;
        for (k, &o) in out.iter().enumerate() {
            let expected_phase = dphi * (k as f32 + 1.0);
            let expected = Complex32::new(expected_phase.cos(), expected_phase.sin());
            assert!((o - expected).norm() < 1e-4, "sample {k}");
        }
    }

    /// Zero-CFO, zero-timing-offset sync: the frame is already
    /// exactly aligned and at exactly `fc`, so the search should find
    /// `shift == 0`, `fest` within one `delf` bin of `fc`, and
    /// `xmax >= 1.3`.
    #[test]
    fn sync_finds_aligned_burst_at_known_frequency() {
        let (_, frame) = synth_clean_frame(|i| ((i * 7 + 3) & 1) as u8);
        let fc_true = 1500.0f32;
        let cdat = upconvert(&frame, fc_true);

        let navmask = [true];
        let result = msk144_sync(&cdat, 1, 8.0, 2.0, &navmask, 2, fc_true);

        assert!(result.success, "xmax = {}", result.xmax);
        assert!(
            (result.fest - fc_true).abs() <= 2.0,
            "fest = {}, expected near {fc_true}",
            result.fest
        );
        assert_eq!(result.peaks[0].shift, 0, "expected zero timing offset");
    }

    /// A residual CFO error (search must find it within the trial
    /// grid) combined with a circular timing offset: confirms both
    /// axes of the joint search are recovered together, and that
    /// [`rotate_to_shift`] + [`matched_filter_softbits`] on the
    /// recovered (frame, shift) decodes the original message.
    #[test]
    fn sync_recovers_freq_and_timing_offset_end_to_end() {
        let (info, frame) = synth_clean_frame(|i| ((i * 3 + 5) & 1) as u8);

        let true_shift = 200usize;
        let mut rotated = vec![Complex32::new(0.0, 0.0); NSPM];
        for k in 0..NSPM {
            rotated[k] = frame[(k + NSPM - true_shift) % NSPM];
        }

        let fc_nominal = 1200.0f32;
        let true_cfo_err = 3.4f32; // within +/-8 Hz search tolerance
        let cdat = upconvert(&rotated, fc_nominal + true_cfo_err);

        let navmask = [true];
        let result = msk144_sync(&cdat, 1, 8.0, 2.0, &navmask, 2, fc_nominal);

        assert!(result.success, "xmax = {}", result.xmax);
        assert_eq!(
            result.peaks[0].shift, true_shift,
            "expected recovered shift to match the injected timing offset"
        );

        let aligned = rotate_to_shift(&result.frame, result.peaks[0].shift);
        let softbits = matched_filter_softbits(
            aligned
                .as_slice()
                .try_into()
                .expect("aligned frame has NSPM samples"),
        );

        let mut codeword = [0u8; 128];
        Ldpc128_90.encode(&info, &mut codeword);
        let bitseq = build_bitseq(&codeword);
        for i in 0..144 {
            assert_eq!(
                softbits[i] > 0.0,
                bitseq[i] == 1,
                "softbit {i} sign mismatch"
            );
        }
    }

    /// Pure noise (no embedded sync word) must not spuriously exceed
    /// the WSJT-X detection threshold.
    #[test]
    fn sync_rejects_pure_noise() {
        let noise: Vec<Complex32> = (0..NSPM)
            .map(|k| {
                let n1 = ((k as f32 * 12.9898).sin() * 43_758.547).fract();
                let n2 = ((k as f32 * 78.233).sin() * 12345.678).fract();
                Complex32::new(0.3 * (n1 - 0.5), 0.3 * (n2 - 0.5))
            })
            .collect();

        let navmask = [true];
        let result = msk144_sync(&noise, 1, 8.0, 2.0, &navmask, 2, 1500.0);
        assert!(
            !result.success,
            "noise should not exceed threshold: xmax = {}",
            result.xmax
        );
    }
}
