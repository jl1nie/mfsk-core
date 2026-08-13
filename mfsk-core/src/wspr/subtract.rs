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
//! ## The LPF: FFT-based overlap-save, after a first attempt that didn't survive contact with hardware
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
//! width)`), so an FFT version was implemented, ported to
//! `crate::engine::fft::FftPlanner` (portable to embedded, unlike
//! `engine::dsp::subtract`'s host-only `rustfft` usage) with overlap-
//! save blocking. **A first attempt** (`nfft = 32768`, chosen only
//! because it comfortably covered `nc2 ≈ 42192` in a single block)
//! passed every host differential test and then produced silently
//! corrupted residuals on the actual device through three distinct
//! embedded-only bugs (kernel-placement off-by-one, the hardware FFT
//! ceiling, a forward/inverse normalisation mismatch between backends)
//! — and *still* corrupted after fixing all three, which pointed at a
//! fourth, undiagnosed bug and led to a full revert to the direct
//! convolution above rather than continue guessing on safety-critical
//! subtract logic. That fourth bug — `esp-dsp`'s fc32 twiddle table is
//! a process-global one-shot, so a size switch after
//! `coarse_baseband`'s 512-point plans corrupted the *next* differently
//! -sized plan, not this LPF's own math — is now root-caused and fixed
//! (`embedded_shared::esp_dsp_fft::ensure_fc32_table`, issue #260). See
//! `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Waste audit"
//! section for the full account of the first attempt.
//!
//! **This implementation** uses `LPF_NFFT = 8192` — this project's
//! `sdkconfig.defaults` ceiling (`CONFIG_DSP_MAX_FFT_SIZE_8192`,
//! unchanged deliberately: raising it to fit `nc2` in one block was
//! not worth the memory/config-impact investigation when blocking
//! already works and this size is what `coarse_baseband` and every
//! other embedded FFT caller already share) — with six overlap-save
//! blocks covering `nc2 ≈ 42192`. Kernel placement
//! (`h[d] = window[half − d]`) and the cross-backend normalisation
//! `#[cfg]` both carry forward from the first attempt's (independently
//! correct, host-verified) fixes.

use alloc::vec;
use alloc::vec::Vec;

use core::f32::consts::PI;
use num_complex::Complex32;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use crate::engine::dsp::downsample::with_default_planner;

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
    let half = NFILT / 2;
    let (cfi, cfq) = lpf_apply_fft(&ci, &cq, &window);

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

/// Reference implementation: direct `O(nc2 × NFILT)` convolution.
/// `cfi[i] = Σ w[j] · ci[i − nfilt/2 + j]`, `wsprd.c:619-624`. Only
/// `half..(nc2-half)` is filled (matching wsprd's own convention —
/// the edges are handled separately by the caller's startup-transient
/// correction); the returned vectors are zero outside that range.
///
/// Kept as the differential-test reference for [`lpf_apply_fft`] —
/// not on the production call path (see that function's doc comment
/// for why an earlier FFT rewrite needed exactly this kind of
/// ongoing cross-check).
#[cfg_attr(not(test), allow(dead_code))]
fn lpf_apply_direct(ci: &[f32], cq: &[f32], window: &[f32; NFILT]) -> (Vec<f32>, Vec<f32>) {
    let nc2 = ci.len();
    debug_assert_eq!(cq.len(), nc2);
    let half = NFILT / 2;
    let mut cfi = vec![0.0f32; nc2];
    let mut cfq = vec![0.0f32; nc2];
    for i in half..(nc2 - half) {
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
    (cfi, cfq)
}

/// FFT-based overlap-save convolution — same math as
/// [`lpf_apply_direct`], `O(nc2 log LPF_NFFT)` instead of `O(nc2 ×
/// NFILT)`. See the module doc comment for the history (a first
/// `nfft = 32768` attempt found three real bugs, then a fourth,
/// undiagnosed one forced a full revert to the direct version; that
/// fourth bug — `esp-dsp`'s process-global one-shot twiddle table —
/// is now fixed at its source, `embedded_shared::esp_dsp_fft`).
///
/// `LPF_NFFT = 8192`: this project's `sdkconfig.defaults` ceiling
/// (`CONFIG_DSP_MAX_FFT_SIZE_8192`), shared with every other embedded
/// FFT caller (`coarse_baseband`'s 512-point spectrogram included) —
/// deliberately not raised, since blocking already covers `nc2` and
/// raising the ceiling would need its own memory/config-impact
/// investigation for a size no other caller needs. Six overlap-save
/// blocks cover `nc2 ≈ 42192`.
fn lpf_apply_fft(ci: &[f32], cq: &[f32], window: &[f32; NFILT]) -> (Vec<f32>, Vec<f32>) {
    const LPF_NFFT: usize = 8192;
    let nc2 = ci.len();
    debug_assert_eq!(cq.len(), nc2);
    let half = NFILT / 2;
    let mut cfi = vec![0.0f32; nc2];
    let mut cfq = vec![0.0f32; nc2];
    if nc2 <= 2 * half {
        // No valid output range (pathologically short input) — same
        // no-op as the direct version's `half..(nc2-half)` being empty.
        return (cfi, cfq);
    }

    // Kernel in FFT-domain circular placement: the direct form sums
    // `window[j] * x[i - half + j]`, i.e. `window[j]` weights the
    // sample `half - j` positions *before* `x[i]` — so in a
    // circular buffer of length `LPF_NFFT`, `window[j]` belongs at
    // index `(half - j) mod LPF_NFFT`, not `j + half`. The two
    // coincide only for an odd-length kernel exactly centred on an
    // integer index (e.g. FT8/FT4's cosine² window); `NFILT = 360` is
    // even, so this project's own sin window's centre sits at 179.5,
    // off-integer, and the naive placement is off by one tap.
    let mut kernel = vec![Complex32::new(0.0, 0.0); LPF_NFFT];
    for (j, &w) in window.iter().enumerate() {
        let d = half as isize - j as isize;
        let idx = d.rem_euclid(LPF_NFFT as isize) as usize;
        kernel[idx] = Complex32::new(w, 0.0);
    }

    let fft_fwd = with_default_planner(|planner| planner.plan_forward(LPF_NFFT));
    let fft_inv = with_default_planner(|planner| planner.plan_inverse(LPF_NFFT));
    fft_fwd.process(&mut kernel);

    // Cross-backend inverse-FFT normalisation: `rustfft` (host)
    // leaves forward *and* inverse unscaled, so the caller divides by
    // `N` once. `EspDspFft::process`'s inverse path (embedded) already
    // divides by `len` internally (its only way to emulate an inverse
    // via the hardware's forward-only kernel) — dividing again here
    // would silently halve-then-halve-again, removing essentially
    // nothing. Neither the trait nor its doc comment specifies a
    // convention, so this has to stay an explicit `#[cfg]`, not
    // something the type system catches.
    #[cfg(feature = "fft-rustfft")]
    let fac = 1.0f32 / LPF_NFFT as f32;
    #[cfg(not(feature = "fft-rustfft"))]
    let fac = 1.0f32;

    // Overlap-save: each `LPF_NFFT`-point block's circular convolution
    // is only trustworthy in its middle `LPF_NFFT - NFILT` samples
    // (the first/last `half` are contaminated by circular wraparound);
    // discard the rest and advance by the trustworthy width. Mirrors
    // `lpf_apply_direct`'s own `half..(nc2-half)` fill range exactly,
    // one block-worth of output at a time instead of one sample.
    let valid_per_block = LPF_NFFT - NFILT;
    let out_lo = half;
    let out_hi = nc2 - half;
    let mut block = vec![Complex32::new(0.0, 0.0); LPF_NFFT];
    let mut out_start = out_lo;
    while out_start < out_hi {
        let block_len = valid_per_block.min(out_hi - out_start);
        // This block's trustworthy output `[out_start, out_start+block_len)`
        // needs input samples `[out_start-half, out_start-half+LPF_NFFT)`
        // — `y[i]` depends on `x[i-half..i-half+NFILT)`, so the first
        // `half` and last `NFILT-1-half` samples of this block's own
        // uncontaminated middle come from data at the input window's
        // edges, which is exactly what the full `LPF_NFFT`-wide window
        // (rather than just `block_len` samples) provides.
        let in_start = out_start as isize - half as isize;
        for (n, b) in block.iter_mut().enumerate() {
            let src = in_start + n as isize;
            *b = if src >= 0 && (src as usize) < nc2 {
                Complex32::new(ci[src as usize], cq[src as usize])
            } else {
                Complex32::new(0.0, 0.0)
            };
        }
        fft_fwd.process(&mut block);
        for (b, k) in block.iter_mut().zip(kernel.iter()) {
            *b *= *k;
        }
        fft_inv.process(&mut block);
        for n in 0..block_len {
            let v = block[half + n] * fac;
            cfi[out_start + n] = v.re;
            cfq[out_start + n] = v.im;
        }
        out_start += block_len;
    }
    (cfi, cfq)
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

    /// `lpf_apply_fft` must match `lpf_apply_direct` to float tolerance
    /// on a realistic, non-trivial signal — not a toy. Uses `nc2 ≈
    /// 42192` (real WSPR shape: `nsig + 2·NFILT` with `nsig = 162 ×
    /// 256`), so the overlap-save blocking path (six `LPF_NFFT = 8192`
    /// blocks) is actually exercised, not just a single-block case
    /// that would hide a blocking bug. Content: a slowly-varying
    /// envelope (what the LPF is meant to pass) plus a fast component
    /// (what it's meant to reject) plus a deterministic pseudo-noise
    /// term, on both channels independently — closer to a real
    /// `s(t)·conj(r(t))` product than a single clean tone.
    #[test]
    fn lpf_fft_matches_direct() {
        let nsig = super::N_SYMBOLS * super::NSPS_BASEBAND;
        let nc2 = nsig + 2 * NFILT;
        let mut ci = vec![0.0f32; nc2];
        let mut cq = vec![0.0f32; nc2];
        // Deterministic (no external RNG dependency), non-trivial:
        // a slow ~1 Hz-equivalent envelope, a fast ~50-sample-period
        // component, and a cheap xorshift-style pseudo-noise term.
        let mut state: u32 = 0x1234_5678;
        for i in 0..nc2 {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            let noise = (state as f32 / u32::MAX as f32) - 0.5;
            let slow = (i as f32 / 4000.0).sin();
            let fast = (i as f32 / 25.0).cos();
            ci[i] = 0.6 * slow + 0.05 * fast + 0.02 * noise;
            cq[i] = 0.4 * slow.cos() - 0.03 * fast + 0.02 * noise;
        }

        let mut window = [0.0f32; NFILT];
        let mut norm = 0.0f32;
        for i in 0..NFILT {
            window[i] = (PI * i as f32 / (NFILT - 1) as f32).sin();
            norm += window[i];
        }
        for w in window.iter_mut() {
            *w /= norm;
        }

        let (direct_i, direct_q) = lpf_apply_direct(&ci, &cq, &window);
        let (fft_i, fft_q) = lpf_apply_fft(&ci, &cq, &window);

        assert_eq!(direct_i.len(), fft_i.len());
        let half = NFILT / 2;
        let mut max_abs_err = 0.0f32;
        let mut max_val = 0.0f32;
        for i in half..(nc2 - half) {
            max_abs_err = max_abs_err
                .max((direct_i[i] - fft_i[i]).abs())
                .max((direct_q[i] - fft_q[i]).abs());
            max_val = max_val.max(direct_i[i].abs()).max(direct_q[i].abs());
        }
        // Same bar the first attempt's differential test used: this is
        // where the kernel-placement off-by-one showed up as a ~1%
        // relative error. A correct implementation should be several
        // orders of magnitude tighter than that (float rounding only).
        assert!(
            max_abs_err < max_val * 1e-4,
            "FFT LPF diverges from direct convolution: max_abs_err={:.3e} max_val={:.3e} (ratio {:.3e})",
            max_abs_err,
            max_val,
            max_abs_err / max_val
        );
        // Outside `half..(nc2-half)` both must be exactly zero (same
        // no-fill convention).
        for i in 0..half {
            assert_eq!(direct_i[i], 0.0);
            assert_eq!(fft_i[i], 0.0, "fft LPF should leave the pre-half edge at 0");
        }
        for i in (nc2 - half)..nc2 {
            assert_eq!(direct_i[i], 0.0);
            assert_eq!(fft_i[i], 0.0, "fft LPF should leave the post-edge at 0");
        }
    }

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
