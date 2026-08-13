//! Coherent baseband subtraction — port of `wsprd.c::subtract_signal2`.
//!
//! WSPR has many concurrent transmitters per slot; once we've decoded a
//! strong signal, removing its contribution from the 375 Hz baseband
//! drops the noise floor for everything weaker by a few dB. wsprd's
//! 3-pass decoder uses this to expose sub-noise signals like W3BI
//! (-27 dB SNR on the WSJT-X golden) that the first pass can't see.
//!
//! Algorithm (matches `wsprd.c:541-660`):
//!
//! ```text
//! Measured: s(t) = a(t) · exp(j·θ(t))     // received baseband
//! Reference: r(t) = exp(j·φ(t))            // synthesised from decoded symbols
//! c(t) = LPF[s(t) · conj(r(t))]            // slow complex amplitude
//! s'(t) = s(t) − c(t) · r(t)               // residual
//! ```
//!
//! The LPF is a 360-tap sin-window FIR; its purpose is to keep only the
//! ~1 Hz envelope of `c(t)` so we don't subtract our own noise back
//! out. A run-length-correction (`norm = partialsum[…]`) compensates
//! for the LPF's startup transient at the first/last `nfilt/2` samples.
//!
//! ## The LPF is a direct `O(nc2 × NFILT)` convolution — an FFT rewrite was tried and reverted
//!
//! The naive per-sample loop (`Σ_j window[j]·ci[i−half+j]`) costs
//! `O(nc2 × NFILT)` — ~15 M mult-adds per channel, ~30 M total for the
//! complex `(ci, cq)` pair, per subtract call. Measured on CoreS3: one
//! pass's worth of subtracts (7 decodes) cost **17.7 s**, comparable to
//! that pass's own coarse search (13.4 s) and decode step (16.3 s).
//!
//! This is the same shape as an anti-pattern `engine::dsp::subtract`'s
//! `subtract_tones_lpf_fft` already solved for FT8/FT4 (an `O(N log
//! N)` FFT-based circular convolution instead of `O(N × filter_
//! width)`), so an FFT version of this LPF was implemented, ported to
//! `crate::engine::fft::FftPlanner` (portable to embedded, unlike
//! `engine::dsp::subtract`'s host-only `rustfft` usage) with overlap-
//! save blocking to fit the ESP32-S3 `esp-dsp` backend's 32 768-point
//! hardware ceiling. It passed every host differential test against
//! this direct convolution (bit-exact to float tolerance, including
//! at the realistic `nc2 > 32768` size that exercises the blocking
//! logic) — and then produced silently corrupted residuals on the
//! actual device (pass 1's `coarse_baseband` finding 0 candidates)
//! through **three** distinct embedded-only bugs in turn: an off-by-
//! one in the circular-delay kernel placement (caught by the host
//! differential test, fixed), the 32 768-point hardware ceiling
//! itself (silently computes with truncated twiddle data past that
//! size rather than erroring — caught by an on-device flash), and a
//! forward/inverse-FFT normalisation convention that differs between
//! the `rustfft` (host) and `esp-dsp` (embedded) backends behind the
//! same `engine::fft::FftPlanner` trait (also caught on-device). After
//! fixing all three, the residual was *still* wrong, pointing at a
//! fourth, undiagnosed embedded-only discrepancy — reverted at that
//! point rather than continue guessing against real hardware on
//! subtract logic, where a silent bug means missed or phantom
//! decodes, not just a slower pass. Full account, including the
//! evidence for each of the three found bugs, is in
//! `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Waste audit"
//! section — worth reading before attempting this again.

use alloc::vec;
use alloc::vec::Vec;

use core::f32::consts::PI;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::baseband::CENTER_HZ;
use super::demod::{N_SYMBOLS, NSPS_BASEBAND, TONE_SPACING_HZ};

const NFILT: usize = 360;

/// Subtract one decoded WSPR signal from `(idat, qdat)` in-place.
///
/// `f0_audio_hz`: signal **CENTER** frequency in audio Hz (= our
/// existing convention plus `1.5·tone_spacing`; callers that hold a
/// tone-0 freq should add `1.5·TONE_SPACING_HZ` first).
/// `shift_baseband`: lag in baseband samples where symbol 0 starts
/// (= `lag_audio / 32`). May be negative for signals that began before
/// `idat[0]`; samples outside the buffer are skipped.
/// `drift_hz`: total drift in Hz over the 162 symbols (matches wsprd's
/// `drift0` argument).
/// `channel_symbols`: 162 channel symbols (0..=3) — the same vector
/// the transmitter would emit. Recover via
/// `wspr::encode_channel_symbols(&info_bits)`.
pub fn subtract_signal_baseband(
    idat: &mut [f32],
    qdat: &mut [f32],
    f0_audio_hz: f32,
    shift_baseband: i32,
    drift_hz: f32,
    channel_symbols: &[u8; N_SYMBOLS],
) {
    debug_assert_eq!(idat.len(), qdat.len());
    let np = idat.len() as i32;
    let nsig = N_SYMBOLS * NSPS_BASEBAND; // 162 · 256 = 41472
    let f0_baseband_hz = f0_audio_hz - CENTER_HZ;

    // Build the reference signal r(t) = exp(j·φ(t)) at the per-symbol
    // tone, with linear drift across the 162 symbols. Matches
    // `wsprd.c:573-589`.
    //
    // Per-sample rotation recurrence (`c[j] = c[j-1]·cdφ − s[j-1]·sdφ`),
    // not a fresh `.cos()`/`.sin()` call every sample: within one
    // symbol `dphi` is constant, so the whole 256-sample run needs only
    // one `cos`/`sin` pair (for `cdphi`/`sdphi`) instead of 512
    // transcendental calls — the same technique
    // `demod::tone_amplitudes_into` already uses for its own oscillator
    // tables. 162 symbols × 2 transcendental calls = 324 total, not
    // `nsig × 2` = 82 944.
    //
    // `(c, s)` persists *across* symbol boundaries (unlike
    // `tone_amplitudes_into`'s per-symbol-fresh oscillators) — this
    // reference needs one phase-continuous 41 472-sample waveform, not
    // 162 independent per-symbol mixes. A pure multiply recurrence that
    // long drifts off the unit circle (each step's rounding error
    // compounds), so `(c, s)` is renormalised to unit magnitude once
    // per symbol (at the point `cdphi`/`sdphi` are recomputed anyway) —
    // bounds the drift to what one 256-sample run can accumulate,
    // rather than letting 41 472 samples' worth compound unchecked.
    let mut refi = vec![0.0f32; nsig];
    let mut refq = vec![0.0f32; nsig];
    let dt = 1.0 / super::baseband::BASEBAND_RATE;
    let twopidt = 2.0 * PI * dt;
    let mut c = 1.0f32;
    let mut s = 0.0f32;
    for i in 0..N_SYMBOLS {
        let norm = (c * c + s * s).sqrt();
        c /= norm;
        s /= norm;
        let cs = channel_symbols[i] as f32;
        // wsprd `wsprd.c:577-582`: per-symbol phase increment
        // (cs - 1.5)·df = tone offset from carrier centre. Drift folds
        // in linearly across the 162 symbols.
        let dphi = twopidt
            * (f0_baseband_hz
                + (drift_hz / 2.0) * (i as f32 - N_SYMBOLS as f32 / 2.0)
                    / (N_SYMBOLS as f32 / 2.0)
                + (cs - 1.5) * TONE_SPACING_HZ);
        let (sdphi, cdphi) = dphi.sin_cos();
        for j in 0..NSPS_BASEBAND {
            let ii = NSPS_BASEBAND * i + j;
            refi[ii] = c;
            refq[ii] = s;
            let (c_next, s_next) = (c * cdphi - s * sdphi, c * sdphi + s * cdphi);
            c = c_next;
            s = s_next;
        }
    }

    // Sin-window LPF coefficients (normalised to unit gain).
    // `wsprd.c:592-599`. Plus running partial sums for the
    // startup-transient correction.
    let mut window = [0.0f32; NFILT];
    let mut norm = 0.0f32;
    for i in 0..NFILT {
        window[i] = (PI * i as f32 / (NFILT - 1) as f32).sin();
        norm += window[i];
    }
    for w in window.iter_mut() {
        *w /= norm;
    }
    let mut partial = [0.0f32; NFILT];
    for i in 1..NFILT {
        partial[i] = partial[i - 1] + window[i];
    }

    // s(t) · conj(r(t)) — store with `nfilt` zero-pad at the start so
    // the LPF can be applied without negative indexing.
    let pad = NFILT;
    let nc2 = nsig + 2 * NFILT;
    let mut ci = vec![0.0f32; nc2];
    let mut cq = vec![0.0f32; nc2];
    // `k = shift_baseband + i` is in-bounds (`0 < k < np`) only for `i`
    // in `i_lo..i_hi` — clamp once instead of re-checking `k > 0 &&
    // k < np` on every one of the `2 × nsig` iterations below (same
    // fix already applied to `engine::dsp::subtract::apply_at_offset`;
    // this file has the identical anti-pattern, just never got it).
    // Out-of-range `i` leaves `ci[i+pad]`/`cq[i+pad]` at their
    // zero-init value in the first loop, and has no `idat`/`qdat` to
    // touch in the second, so the clamped range is a complete
    // substitute for the branch, not just a fast path alongside it.
    // i64 arithmetic avoids overflow for extreme `shift_baseband`.
    let i_lo = (1_i64 - shift_baseband as i64).clamp(0, nsig as i64) as usize;
    let i_hi = (np as i64 - shift_baseband as i64).clamp(0, nsig as i64) as usize;
    for i in i_lo..i_hi {
        let k = (shift_baseband + i as i32) as usize;
        let id = idat[k];
        let qd = qdat[k];
        ci[i + pad] = id * refi[i] + qd * refq[i];
        cq[i + pad] = qd * refi[i] - id * refq[i];
    }

    // LPF: cfi[i] = Σ w[j] · ci[i − nfilt/2 + j]. wsprd `wsprd.c:619-624`.
    let mut cfi = vec![0.0f32; nc2];
    let mut cfq = vec![0.0f32; nc2];
    let half = NFILT / 2;
    for i in half..(nc2 - half) {
        // `i - half + j` for `j in 0..NFILT` spans exactly
        // `[i-half, i-half+NFILT)`, which the outer loop's
        // `half..(nc2-half)` bound keeps within `ci`/`cq` — a plain
        // slice zip instead of `NFILT` manually-indexed accesses per
        // sample. Fused into one `fold` (not two separate `.sum()`
        // chains) so `window[j]` is read once per `j` and reused for
        // both products, matching the original single-pass loop —
        // two separate chains measured a real ~5-6% regression
        // (doubles the loop's iteration count, breaks whatever
        // fused vectorization the original got).
        let ci_win = &ci[i - half..i - half + NFILT];
        let cq_win = &cq[i - half..i - half + NFILT];
        let (acc_i, acc_q) = window
            .iter()
            .zip(ci_win)
            .zip(cq_win)
            .fold((0.0f32, 0.0f32), |(ai, aq), ((&w, &c_i), &c_q)| {
                (ai + w * c_i, aq + w * c_q)
            });
        cfi[i] = acc_i;
        cfq[i] = acc_q;
    }

    // Subtract c(t) · r(t) from idat/qdat. The startup-transient
    // correction (`norm = partial[half + i]` for i < half, mirrored at
    // the tail) compensates for the LPF's running sum being short of
    // unity at the boundaries. Matches `wsprd.c:632-660`.
    // Same `i_lo..i_hi` clamp as the camp-build loop above — identical
    // `k` formula, so identical bounds. `n`'s own `> 0.0` guard stays a
    // per-iteration check (unlike `k`'s range, `n` is a value, not a
    // pure index boundary — `partial[]`'s startup-transient correction
    // could in principle be non-positive for a differently-tuned
    // `NFILT`, so this isn't safe to hoist away).
    for i in i_lo..i_hi {
        let n = if i < half {
            partial[half + i]
        } else if i > nsig - 1 - half {
            partial[half + nsig - 1 - i]
        } else {
            1.0
        };
        if n > 0.0 {
            let k = (shift_baseband + i as i32) as usize;
            let j = i + pad;
            idat[k] -= (cfi[j] * refi[i] - cfq[j] * refq[i]) / n;
            qdat[k] -= (cfi[j] * refq[i] + cfq[j] * refi[i]) / n;
        }
    }
}

/// Run subtract_signal_baseband for each of `decodes` against
/// `(idat, qdat)`. Convenience wrapper for the 3-pass loop.
///
/// `audio_to_baseband_lag`: function that converts a decode's
/// audio-rate `start_sample` to the baseband-rate `shift` expected by
/// `subtract_signal_baseband`. Typically `lag_audio / 32`.
pub fn subtract_all<F>(
    idat: &mut [f32],
    qdat: &mut [f32],
    decodes: &[super::WsprResult],
    audio_to_baseband_lag: F,
) where
    F: Fn(&super::WsprResult) -> i32,
{
    for d in decodes {
        let symbols = super::encode_channel_symbols(&d.info_bits);
        let f0_audio = d.freq_hz + 1.5 * TONE_SPACING_HZ; // tone-0 → centre
        let shift_baseband = audio_to_baseband_lag(d);
        subtract_signal_baseband(
            idat,
            qdat,
            f0_audio,
            shift_baseband,
            0.0, // we don't currently estimate drift in the demod path
            &symbols,
        );
    }
    let _ = Vec::<u8>::new(); // silence unused-imports warning on no_std
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wspr::baseband::{NPOINTS_MAX, decimate_to_baseband};
    use crate::wspr::tx::synthesize_type1;

    #[test]
    fn subtract_attenuates_synth_tone() {
        // Build a clean synth signal, decimate to baseband, subtract
        // the same signal back out — residual should be much smaller
        // than the original baseband energy.
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.5).expect("synth");
        let mut padded = vec![0.0f32; NPOINTS_MAX];
        padded[..audio.len()].copy_from_slice(&audio);
        let (mut idat, mut qdat) = decimate_to_baseband(&padded);

        let pre_pwr: f32 =
            idat.iter().map(|&x| x * x).sum::<f32>() + qdat.iter().map(|&x| x * x).sum::<f32>();

        // Round-trip through the test recovers the same symbols
        // synthesize_type1 produced. Decode the synth audio, then
        // re-encode the recovered info_bits to channel symbols.
        let r = crate::wspr::decode_at(&audio, 12_000, 0, 1500.0).expect("decode synth");
        let symbols = crate::wspr::encode_channel_symbols(&r.info_bits);
        // Synth has tone-0 = 1500 Hz, so signal centre = 1500 + 2.197;
        // shift_baseband = 0 (synth starts at sample 0).
        subtract_signal_baseband(
            &mut idat,
            &mut qdat,
            1500.0 + 1.5 * TONE_SPACING_HZ,
            0,
            0.0,
            &symbols,
        );
        let post_pwr: f32 =
            idat.iter().map(|&x| x * x).sum::<f32>() + qdat.iter().map(|&x| x * x).sum::<f32>();
        assert!(
            post_pwr < pre_pwr * 0.5,
            "subtract should remove most of the signal energy: pre={:.2e} post={:.2e}",
            pre_pwr,
            post_pwr
        );
    }
}
