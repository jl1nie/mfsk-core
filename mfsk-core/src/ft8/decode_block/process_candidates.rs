//! Per-candidate decode loop + facade entry functions.
//!
//! The engine room of FT8 decoding. Contains the multipass driver,
//! refine / fine_refine passes, the per-candidate processing loop
//! that drives LLR → BP → OSD → AP, and `process_one_candidate_inner`
//! (still home to the OSD dispatch block ε.6 lifts into
//! `osd_strategy.rs`).
//!
//! Public-facade entries (`decode_block`, `decode_block_tuned`,
//! `decode_block_with_ap[_tuned]`, `decode_block_into[_tuned]`) live
//! here too, since the file body owns the `decode_block_multipass`
//! they wrap. The parent module (`decode_block.rs`) re-exports them
//! at the same `mfsk_core::ft8::decode_block::*` paths external
//! callers (`mfsk-ffi-ft8`, integration tests, `super::decode::*`)
//! already use.
//!
//! ε.5 of the `docs/CLEANUP_2026_05.md` `decode_block` split.

use alloc::boxed::Box;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::super::decode::{ApHint, DecodeDepth, DecodeResult, DecodeStrictness};
use super::super::llr::sync_quality;
use super::super::message::unpack77;
use super::super::params::{COSTAS, DEFAULT_BP_MAX_ITER, LDPC_N, NSPS, NTONES};
use super::super::wave_gen::message_to_tones;
use super::coarse_sync::coarse_sync;
use super::fill_symbol_spectra::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
use super::spectrogram::{Spectrogram, compute_spectrogram};
use super::types::{
    AudioSample, DEFAULT_Q_THRESH, NFFT_SPEC, NMS_ALPHA, NSTEP, SAMPLE_RATE_HZ, TONE_SPACING_HZ,
    TX_START_OFFSET_S,
};
use crate::core::scalar::{Cmplx, ComplexSpec};
use crate::core::sync::SyncCandidate;
use crate::fec::ldpc::bp::check_crc14;
#[cfg(feature = "fft-rustfft")]
use crate::fec::ldpc::osd::osd_decode_deep;

#[cfg(all(feature = "fixed-point", not(feature = "fft-rustfft")))]
use super::fill_symbol_spectra::fill_symbol_spectra_into;
#[cfg(feature = "fixed-point")]
use super::fill_symbol_spectra::symbol_spectra_direct_into;

// ── Public entry ────────────────────────────────────────────────────────────

/// Embedded FT8 decode for one 15-s slot.
///
/// Runs the same algorithm shape as [`decode_frame`](super::super::decode::decode_frame)
/// but talks only to power-of-two FFTs (via the
/// [`crate::core::fft::FftPlanner`] trait) and uses the min-sum LDPC
/// kernel to skip per-iteration `tanh` / `atanh`. No
/// `decode_sniper*` paths are involved; no wide-band 192 k FFT cache.
///
/// Sensitivity vs `decode_frame` is characterised on host AWGN
/// sweeps before any embedded port — see
/// `tests/ft8_decode_block_snr_sweep.rs`.
///
/// # Arguments
/// * `audio`     — 12 kHz i16 PCM, length up to NMAX = 180 000.
/// * `freq_min`  — lower edge of carrier search (Hz).
/// * `freq_max`  — upper edge of carrier search (Hz).
/// * `sync_min`  — minimum normalised Costas score (typical 1.0–2.0).
/// * `depth`     — `Bp` / `BpAll` / `BpAllOsd`.
/// * `max_cand`  — cap on Costas candidates evaluated.
pub fn decode_block<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        None,
        DecodeStrictness::Normal,
    )
}

/// Variant of [`decode_block`] that accepts a runtime `bp_max_iter`
/// (= per-LLR-variant BP iteration cap, default
/// [`DEFAULT_BP_MAX_ITER`]). Useful on time-budgeted targets.
#[allow(clippy::too_many_arguments)]
pub fn decode_block_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        bp_max_iter,
        None,
        DecodeStrictness::Normal,
    )
}

/// AP-aware variant of [`decode_block`]. Mirrors `decode_block`'s
/// behaviour exactly when `ap_hint = None`; with `Some(&ap)` runs
/// the full WSJT-X iaptype loop (5..12) per candidate after Steps 1-3
/// (BP staircase + OSD) all fail. Host `fft-rustfft` build only —
/// embedded fixed-point keeps its existing iaptype-1-only path.
///
/// Used by host `decode_frame_with_ap` after the v0.6.1 redirect, and
/// by mountain-top apps that want full AP rescue from a single entry
/// point. New in 0.6.1.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_with_ap<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        ap_hint,
        DecodeStrictness::Normal,
    )
}

/// Variant of [`decode_block_with_ap`] that accepts runtime
/// `bp_max_iter` and `strictness`. New in 0.6.1.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_with_ap_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    decode_block_multipass(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        bp_max_iter,
        ap_hint,
        strictness,
    )
}

/// WSJT-X `ft8_decode.f90:172-236` 3-pass loop driver. Each pass:
/// coarse_sync on the (subtracted) audio, fine refine, decode, then
/// LPF-subtract every fresh CRC-passing decode for the next pass.
///
/// Pass termination matches WSJT-X exactly:
/// - pass 1 always runs;
/// - pass 2 skips when pass 1 returned 0 decodes;
/// - pass 3 skips when pass 2 returned no NEW decodes.
///
/// On host (`fft-rustfft`) the audio is cloned to a working `Vec<i16>`
/// (subtract operates on i16 samples). Embedded targets compile through
/// the same path; the clone cost is dominated by the BP work it enables.
#[cfg(feature = "fft-rustfft")]
#[allow(clippy::too_many_arguments)]
fn decode_block_multipass<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    use alloc::vec::Vec as AllocVec;
    let mut work: AllocVec<i16> = audio.iter().map(|s| s.to_i16()).collect();
    let mut all: AllocVec<DecodeResult> = AllocVec::new();
    let mut prev_total: usize = 0;
    let mut sbase_and_spec: Option<(AllocVec<f32>, Spectrogram)> = None;
    for ipass in 0..3 {
        if ipass >= 1 && all.len() == prev_total {
            // Pass 2 skips on zero from pass 1; pass 3 on zero new.
            break;
        }
        prev_total = all.len();

        let spec = compute_spectrogram(work.as_slice(), freq_max);
        // Capture pass 1's spectrogram + per-bin baseline (= ORIGINAL
        // audio, before any subtract) for WSJT-X-faithful xsnr2 SNR.
        // ft8b.f90:449 reads sbase from the pre-subtract baseline so
        // xsnr2 stays stable across passes; xsig is also read from
        // the same spectrogram so the two share an absolute scale.
        if ipass == 0 {
            let mut avg = alloc::vec![0.0_f32; spec.n_freq];
            crate::ft8::baseline::avg_spectrum(&spec, &mut avg);
            let sbase_v = crate::ft8::baseline::fit_baseline(&avg, 0, spec.n_freq - 1);
            let spec_clone = Spectrogram {
                n_freq: spec.n_freq,
                n_time: spec.n_time,
                data: spec.data.clone(),
            };
            sbase_and_spec = Some((sbase_v, spec_clone));
        }
        let cands = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
        drop(spec);
        let cands = fine_refine_pass1(work.as_slice(), cands);
        let pass2 = refine_candidates(work.as_slice(), cands, max_cand);

        // **WSJT-X ft8b.f90:432-437 sequential subtract**: each
        // accepted decode immediately subtracts from `work` so the
        // NEXT candidate in this same pass sees a cleaner residual.
        // Without this, all candidates in a pass see the same raw
        // audio — strong real signals at one freq leak Costas-aligned
        // energy into nearby phantom candidates' bins, allowing
        // CRC-pass garbage to decode there. The driver's outer pass
        // loop is for OSD/AP differences (ndepth-dependent), not for
        // the subtract cadence.
        #[cfg(feature = "std")]
        let trace = std::env::var("MFSK_TRACE_PHANTOM").is_ok();
        #[cfg(not(feature = "std"))]
        let trace = false;
        for cand in pass2 {
            let single_results = process_candidates_tuned_with_ap(
                work.as_slice(),
                alloc::vec![cand],
                depth,
                DEFAULT_Q_THRESH,
                bp_max_iter,
                ap_hint,
                strictness,
            );
            for r in single_results {
                if all.iter().any(|x| x.message77 == r.message77) {
                    continue;
                }
                if trace {
                    #[cfg(feature = "std")]
                    if let Some(text) = crate::msg::wsjt77::unpack77(&r.message77) {
                        eprintln!(
                            "  TRACE pass={} freq={:>7.2} dt={:+.4} e={:>2} '{}'",
                            ipass, r.freq_hz, r.dt_sec, r.hard_errors, text,
                        );
                    }
                }
                crate::ft8::subtract::subtract_signal_lpf(work.as_mut_slice(), &r);
                all.push(r);
            }
        }
    }

    // Replace each result's snr_db with WSJT-X xsnr2 (pre-subtract
    // spectrogram + baseline). `xsig` is read directly from the
    // captured pass-1 spectrogram at each tone position, so xsig
    // and xbase share the exact same absolute scale (= the original
    // sync8 spectrogram). This is critical: feeding xsig from a
    // different chain (e.g. cd0 downsampled per-symbol DFT) loses
    // the calibration.
    //
    // **WSJT-X post-decode validity gates (#63).** Mirrors
    // `ft8b.f90:422-459`:
    //   - msg-type `i3 / n3` validity (lines 425-428)
    //   - `unpack77` success (line 430)
    //   - `nsync <= 10 && xsnr < -24.0 dB` bail-out (line 456)
    // The xsnr gate has to run on the RAW (un-clamped) `xsnr2` because
    // both WSJT-X (line 460) and our previous attempt clamp the value
    // to -24 dB for display *after* the gate. Clamping first would
    // collapse every "below floor" result to exactly -24, dead-letter
    // the `xsnr < -24.0` test, and let exactly the phantoms the gate
    // was designed to catch slip through (= the qso3_busy phantoms
    // named in issue #63's body).
    //
    // xsnr2/xbase post-process is f32-only. Fixed-point Spectrogram
    // cells are quantised post `>> FP_SPEC_SHIFT`, putting many noise
    // cells at u16 zero — `fit_baseline`'s `log10(p.max(1e-30))` then
    // produces sbase ≈ -250 dB and xsnr2 explodes. The original
    // adjacent-tone SNR from `process_candidates_into` (compute_snr_db)
    // is already on a sensible scale, so leave it untouched on the
    // fixed-point path; the msg-type / unpack77 / nsync gates also
    // run only on the f32 path (the fixed-point pipeline doesn't
    // surface OSD-pass results, so the phantom set doesn't apply).
    #[cfg(not(feature = "fixed-point"))]
    if let Some((sbase, spec)) = sbase_and_spec {
        let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
        let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
        let nsps_steps = (NSPS / NSTEP) as f32;
        all.retain_mut(|r| {
            // All-zero message77 reject (ft8b.f90:423). LDPC linearity:
            // encode(0) = 0, and CRC-14 of all-zero info bits is zero,
            // so an "all-zero codeword" survives both the BP and OSD
            // staircases by CRC-luck. Rejecting it here matches WSJT-X.
            if r.message77.iter().all(|&b| b == 0) {
                return false;
            }
            // msg-type validity (ft8b.f90:425-428). Bits 71..73 = n3,
            // 74..76 = i3 (MSB first) — matches the offsets `unpack77`
            // reads at `msg/wsjt77.rs:236-237`.
            let m = &r.message77;
            let bit3 = |off: usize| -> u8 { (m[off] << 2) | (m[off + 1] << 1) | m[off + 2] };
            let n3 = bit3(71);
            let i3 = bit3(74);
            if i3 > 5 || (i3 == 0 && n3 > 6) {
                return false;
            }
            if i3 == 0 && n3 == 2 {
                return false;
            }
            // unpack77 success (ft8b.f90:430).
            if crate::msg::wsjt77::unpack77(&r.message77).is_none() {
                return false;
            }
            // xsnr2 gate (ft8b.f90:456). Compute raw, gate, then clamp.
            // The clamp here MUST happen after the gate test — see the
            // function-level comment on `recompute_snr_xsnr2` for why
            // the pre-clamp ordering used to dead-letter the gate.
            let raw_snr = recompute_snr_xsnr2(r, &spec, &sbase, df, tstep, nsps_steps, 1.0);
            let nsync = recompute_nsync(r, &spec, df, tstep, nsps_steps);
            if nsync <= 10 && raw_snr < -24.0 {
                return false;
            }
            r.snr_db = raw_snr.max(-24.0);
            true
        });
    }
    #[cfg(feature = "fixed-point")]
    let _ = sbase_and_spec;
    all
}

/// Hard-decision sync count (= WSJT-X `ft8b.f90:163-176` nsync) read
/// from the pass-1 spectrogram at the result's refined (freq, dt).
/// 21-bit upper bound (3 sync blocks × 7 Costas positions).
///
/// Only the host f32 build calls this — `recompute_snr_xsnr2` /
/// `recompute_nsync` use the `xsnr2/xbase` formulation which is
/// f32-only (see the `#[cfg(not(feature = "fixed-point"))]` caller
/// at line ~1877).
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
fn recompute_nsync(
    result: &DecodeResult,
    spec: &Spectrogram,
    df: f32,
    tstep: f32,
    nsps_steps: f32,
) -> u32 {
    use crate::ft8::params::COSTAS;
    const NTONES: usize = 8;
    let carrier_bin_f = result.freq_hz / df;
    let tone_step = TONE_SPACING_HZ / df; // = 2.0 at NFFT=3840
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;
    // Costas blocks at symbol indices 0, 36, 72 (each 7 symbols long).
    let mut count = 0u32;
    for &block_off in &[0_usize, 36, 72] {
        for (sym_in_block, &expected) in COSTAS.iter().enumerate() {
            let k = block_off + sym_in_block;
            let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
            if m_bin < 0 || m_bin as usize >= spec.n_time {
                continue;
            }
            let m_bin = m_bin as usize;
            let mut best_t = 0;
            let mut best_p = f32::MIN;
            for t in 0..NTONES {
                let f_bin = (carrier_bin_f + (t as f32) * tone_step).round() as i32;
                if f_bin < 0 || f_bin as usize >= spec.n_freq {
                    continue;
                }
                let p = spec.power_acc(f_bin as usize, m_bin);
                if p > best_p {
                    best_p = p;
                    best_t = t;
                }
            }
            if best_t == expected {
                count += 1;
            }
        }
    }
    count
}

/// Slot-baseline xsnr2 SNR for any [`Spectrogram`] — `std`-free
/// alternative to `recompute_snr_xsnr2` for callers that haven't
/// run `baseline::fit_baseline`'s polynomial smoother (= the
/// embedded path: `mfsk-core` is built `default-features = false`
/// + `alloc` only there, so the polynomial fit is unavailable).
///
/// Computes the per-frequency baseline as the mean across time over
/// a ±50-bin window centred on the decode's carrier — same idea as
/// `avg_spectrum` but localised so it works incrementally and stays
/// cheap on Xtensa LX6/LX7. Then evaluates WSJT-X `ft8b.f90:449-454`:
///
/// ```text
///   xbase = mean_over_time(spec[carrier_window]) * cell_scale
///   xsig  = sum_over_79_decoded_tones(spec[tone_bin, m]) * cell_scale
///   xsnr2 = xsig / xbase / 3e6 - 1
///   snr_db = 10·log10(xsnr2) - 27        (clamped at -24 dB on degeneracy)
/// ```
///
/// `cell_scale = 1.0` for an `f32` spectrogram, `2^FP_SPEC_SHIFT`
/// (`= 4096.0` with the default fixed-point shift) for an embedded
/// u16 spectrogram so xsig and xbase land in the same WSJT-X
/// calibration regime.
///
/// Vs the per-Costas-block adjacent-tone SNR `compute_snr_db` returns
/// from `process_candidates*`: that ratio is preserved under the
/// `fill_symbol_spectra` per-block auto-gain so it's *internally*
/// consistent, but its absolute number drifts ~0–15 dB between
/// signals because each block's gain factor is signal-dependent.
/// This function reads `xsig`/`xbase` from the *single* sync8
/// spectrogram (uniform `FP_SPEC_SHIFT` auto-gain across the slot),
/// so the result is comparable across signals AND comparable to
/// WSJT-X / JTDX SNR reports.
pub fn xsnr2_db_simple(spec: &Spectrogram, result: &DecodeResult, cell_scale: f32) -> f32 {
    use crate::ft8::params::NN;
    use crate::ft8::wave_gen::message_to_tones;

    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let tstep = NSTEP as f32 / SAMPLE_RATE_HZ;
    let nsps_steps = (NSPS / NSTEP) as f32;
    let tone_step = TONE_SPACING_HZ / df;

    if spec.n_freq == 0 || spec.n_time == 0 {
        return -24.0;
    }

    // Per-freq baseline — **median** (P50) of cell values inside a
    // local window around the decode's carrier. A plain mean is
    // dragged upward by the very signal we're trying to measure, so
    // xbase tracks xsig and the ratio collapses (verified
    // empirically: W1FC at 0 dB had xbase ≈ 4.8 M while N1JFU at
    // -14 dB had ≈ 0.15 M before this change). Median ignores the
    // signal-bin outliers and reads true noise floor.
    //
    // Window: ±50 freq bins (~156 Hz at NFFT=3840) × time-decimated
    // by `n_time / 50` so the sort touches ~2 500 samples. Sort cost
    // on f32 ~25 k comparisons ~100 µs at LX7 240 MHz — well inside
    // the post-SlotEnd budget.
    let carrier_bin = (result.freq_hz / df)
        .round()
        .clamp(0.0, (spec.n_freq - 1) as f32) as usize;
    let f_lo = carrier_bin.saturating_sub(50);
    let f_hi = (carrier_bin + 50).min(spec.n_freq - 1);
    let t_stride = spec.n_time.div_ceil(50).max(1);
    let mut samples: alloc::vec::Vec<f32> =
        alloc::vec::Vec::with_capacity((f_hi - f_lo + 1) * spec.n_time.div_ceil(t_stride) + 4);
    for f in f_lo..=f_hi {
        let mut t = 0usize;
        while t < spec.n_time {
            samples.push(spec.power_acc(f, t));
            t += t_stride;
        }
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let median = if samples.is_empty() {
        0.0
    } else {
        samples[samples.len() / 2]
    };
    let xbase = median * cell_scale;
    if xbase <= 0.0 || !xbase.is_finite() {
        return -24.0;
    }

    // xsig at the 79 decoded-tone (freq, m) positions.
    let itone = message_to_tones(&result.message77);
    let carrier_bin_f = result.freq_hz / df;
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;
    let mut xsig: f32 = 0.0;
    for k in 0..NN {
        let t = itone[k] as f32;
        let f_bin = (carrier_bin_f + t * tone_step).round() as i32;
        let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
        if f_bin < 0 || f_bin as usize >= spec.n_freq || m_bin < 0 || m_bin as usize >= spec.n_time
        {
            continue;
        }
        xsig += spec.power_acc(f_bin as usize, m_bin as usize);
    }
    xsig *= cell_scale;

    // Empirical calibration constant — pair-matched against the
    // JTDX qso3_busy reference on real M5StickS3 silicon
    // (2026-05-05). With the median-of-window noise floor above the
    // raw `xsig/xbase` ratio spans ~1100 (-14 dB SNR) → ~100 000
    // (0 dB SNR) on the embedded u16 spectrogram; the value below
    // collapses that to JTDX-comparable dB within ±3 dB across
    // weak / mid / strong signals.
    //
    // (WSJT-X's `ft8b.f90:451` value `3e6` is calibrated to their
    // f32 spectrogram amplitude scale and doesn't carry over here.)
    const XSNR2_CAL_DB: f32 = 46.0;

    let ratio = xsig / xbase;
    if ratio <= 1.0 {
        return -24.0;
    }
    let snr = 10.0 * ratio.log10() - XSNR2_CAL_DB;
    snr.max(-24.0)
}

/// WSJT-X `ft8b.f90:449-454` xsnr2 SNR formula. Reads the signal's
/// per-symbol power from the pass-1 spectrogram (= pre-subtract,
/// matching WSJT-X's sync8 spectrogram convention) at each of the
/// 79 expected tones, then divides by the per-frequency baseline
/// `sbase`:
///
/// ```text
///   xbase = 10^((sbase[round(f1/df)] - 40) / 10)
///   xsnr2 = xsig / xbase / 3e6 - 1
///   xsnr2_db = 10·log10(xsnr2) - 27
/// ```
///
/// Both `xsig` and `xbase` are on the same spectrogram scale, so the
/// formula's `/3e6 - 27` calibration (WSJT-X's "2.5 kHz reference"
/// convention) maps directly into a WSJT-X-compatible dB number.
///
/// Falls back to `-24 dB` if the ratio degenerates.
///
/// f32-only — fixed-point spectrograms quantise to u16, putting noise
/// cells at zero and breaking the `log10` baseline; see the comment
/// block at the `retain_mut` caller for the full rationale.
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
fn recompute_snr_xsnr2(
    result: &DecodeResult,
    spec: &Spectrogram,
    sbase: &[f32],
    df: f32,
    tstep: f32,
    nsps_steps: f32,
    cell_scale: f32,
) -> f32 {
    let itone = crate::ft8::wave_gen::message_to_tones(&result.message77);
    let carrier_bin_f = result.freq_hz / df;
    let tone_step = TONE_SPACING_HZ / df;
    let t0 = (TX_START_OFFSET_S + result.dt_sec) / tstep;

    let mut xsig = 0.0_f32;
    for k in 0..79_usize {
        let t = itone[k] as f32;
        let f_bin = (carrier_bin_f + t * tone_step).round() as i32;
        let m_bin = (t0 + (k as f32) * nsps_steps).round() as i32;
        if f_bin < 0 || f_bin as usize >= spec.n_freq || m_bin < 0 || m_bin as usize >= spec.n_time
        {
            continue;
        }
        xsig += spec.power_acc(f_bin as usize, m_bin as usize);
    }
    // `cell_scale` reverts the fixed-point spectrogram's
    // `>> FP_SPEC_SHIFT` so xsig and xbase live in WSJT-X's calibration
    // regime. For the f32 spectrogram cell_scale=1.0 (no-op).
    xsig *= cell_scale;

    let bin = carrier_bin_f.round() as i32;
    let bin = bin.clamp(0, sbase.len() as i32 - 1) as usize;
    // Same compensation on xbase: sbase_db came from `fit_baseline` of
    // post-shift cells, so its log10 reads ~36 dB low in fixed-point.
    let sbase_db_compensated = sbase[bin] + 10.0 * cell_scale.log10();
    let xbase = 10f32.powf(0.1 * (sbase_db_compensated - 40.0));
    let arg = xsig / xbase / 3.0e6 - 1.0;
    // WSJT-X `ft8b.f90:445-454`: default `xsnr2 = 0.001` if the
    // ratio degenerates, then `xsnr2_db = 10·log10(xsnr2) - 27`
    // → -57 dB. Caller (`retain_mut` in `decode_block_multipass`)
    // applies the `xsnr < -24` gate against this raw value BEFORE
    // clamping it to the -24 dB display floor; the previous
    // `snr.max(-24.0)` here pre-clamped and made the gate fire only
    // for arithmetic underflow, not for "degenerate signal" cases.
    let xsnr2 = if arg > 0.1 { arg } else { 0.001 };
    10.0 * xsnr2.log10() - 27.0
}

/// Embedded path: single-pass `decode_block` (matches the previous
/// production behaviour, no subtract). Host-only `fft-rustfft` adds
/// the multipass driver. `_ap_hint` and `_strictness` parameters are
/// accepted for signature parity with the host variant but ignored
/// here — embedded fixed-point keeps its existing iaptype-1-only
/// hardcoded plumbing (full AP deferred to 0.7.x).
#[cfg(not(feature = "fft-rustfft"))]
#[allow(clippy::too_many_arguments)]
fn decode_block_multipass<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    _ap_hint: Option<&ApHint>,
    _strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass1 = fine_refine_pass1(audio, pass1);
    let pass2 = refine_candidates(audio, pass1, max_cand);
    process_candidates_tuned(audio, pass2, depth, DEFAULT_Q_THRESH, bp_max_iter)
}

/// Per-candidate WSJT-X-style 3-stage fine refine. Builds the
/// 192k-FFT cache once and downsamples per candidate. Host-only;
/// embedded paths skip this for compute reasons (cache is 1.5 MB,
/// 192k FFT is not in our embedded planner).
#[cfg(feature = "fft-rustfft")]
fn fine_refine_pass1<S: AudioSample>(
    audio: &[S],
    cands: alloc::vec::Vec<crate::core::sync::SyncCandidate>,
) -> alloc::vec::Vec<crate::core::sync::SyncCandidate> {
    if cands.is_empty() {
        return cands;
    }
    // Convert audio → Vec<i16> for the downsampler (no-op when S=i16).
    let audio_i16: alloc::vec::Vec<i16> = audio.iter().map(|s| s.to_i16()).collect();
    let fft_cache = crate::ft8::downsample::build_fft_cache(&audio_i16);
    cands
        .into_iter()
        .map(|c| {
            let (cd0, _) =
                crate::ft8::downsample::downsample(&audio_i16, c.freq_hz, Some(&fft_cache));
            let r = crate::ft8::refine_fine::fine_refine_3stage(&cd0, c.dt_sec);
            crate::core::sync::SyncCandidate {
                freq_hz: c.freq_hz + r.delf_hz,
                dt_sec: r.dt_sec,
                score: c.score,
            }
        })
        .collect()
}

/// Embedded build path — preserve the original (no fine refine) shape.
///
/// Attempted in 0.6.3 via `fill_symbol_spectra` iteration at
/// 41 (freq, dt) probe points per candidate, but the per-symbol
/// DFT cost (~6900 DFTs/cand × 30 cand × 1920-sample DFT ≈ 200k
/// DFTs/slot) tripped the FreeRTOS task watchdog after ~5 s of
/// uninterrupted compute on S3 LX7 — fundamentally too heavy
/// without the host's 192k FFT shortcut. Reverted to NO-OP. A
/// proper embedded fine_refine needs cd0 built via FIR decimate
/// (3:1 → 4:1 → 5:1, integer ratios, ~30 ms total) followed by
/// `refine_fine_3stage` on the 200 Hz baseband — deferred to a
/// future patch (estimate: ~150 lines for the FIR decimator).
#[cfg(not(feature = "fft-rustfft"))]
fn fine_refine_pass1<S: AudioSample>(
    _audio: &[S],
    cands: alloc::vec::Vec<crate::core::sync::SyncCandidate>,
) -> alloc::vec::Vec<crate::core::sync::SyncCandidate> {
    cands
}

/// Variant of [`decode_block`] that accepts caller-provided
/// basis scratch — required on Core2 / Cortex-M for the asm dot
/// product to reach its theoretical 1-cycle/sample speed (default
/// heap allocation routes the 60 KB × 2 basis to PSRAM /
/// non-cacheable RAM and erases the kernel's advantage).
///
/// Both `basis_re` and `basis_im` must be at least
/// [`BASIS_SCRATCH_LEN`] long; place them in **internal-RAM `.bss`**
/// (`static [i16; BASIS_SCRATCH_LEN]` arrays) for max throughput.
/// Same recall / depth / staircase as `decode_block`, just no
/// per-candidate allocation cycles.
#[cfg(feature = "fixed-point")]
pub fn decode_block_into<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    decode_block_into_tuned(
        audio,
        freq_min,
        freq_max,
        sync_min,
        depth,
        max_cand,
        DEFAULT_BP_MAX_ITER,
        basis_re,
        basis_im,
    )
}

/// Variant of [`decode_block_into`] that accepts a runtime
/// `bp_max_iter`. The recommended top-level entry point for embedded
/// LX6 / LX7 callers — `bp_max_iter` is the dominant time-scaling
/// knob in the post-SlotEnd budget.
#[cfg(feature = "fixed-point")]
#[allow(clippy::too_many_arguments)]
pub fn decode_block_into_tuned<S: AudioSample>(
    audio: &[S],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bp_max_iter: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    let spec = compute_spectrogram(audio, freq_max);
    let pass1 = coarse_sync(&spec, freq_min, freq_max, sync_min, pass1_limit());
    drop(spec);
    let pass2 = refine_candidates_into(audio, pass1, max_cand, basis_re, basis_im);
    process_candidates_into_tuned(
        audio,
        pass2,
        depth,
        DEFAULT_Q_THRESH,
        bp_max_iter,
        basis_re,
        basis_im,
    )
}

/// Pass-1 candidate cap — coarse_sync emits at most this many
/// candidates regardless of `max_cand`. Pass 2 re-ranks by
/// `sync_quality` (the same metric stage 3 uses to gate decode
/// attempts — much sharper than the per-bin power ratio) and
/// truncates to caller's `max_cand` for stage 3.
///
/// Sweep on real-QSO WAVs (host fp i16, BpAll, with the regularised
/// coarse_sync ratio in `RATIO_EPS_DEFAULT`) showed:
/// - PASS1 ∈ {30, 50}: 14/22 truth (drops one weak qso1 signal)
/// - PASS1 ∈ {75, 100}: 15/22 truth (full recall ceiling)
/// - PASS1=200: same 15/22 (no further gain — qso3's remaining gap
///   is at coarse_sync rank 100+, beyond Pass 2's reach).
///
/// 75 is the smallest PASS1 that keeps the full recall ceiling.
/// 30 is the smallest PASS1 that keeps the qso3 (busy band) truth
/// ceiling — it loses one borderline -17 dB qso1 signal (OH3NIV).
/// Core2 ships with 30 (speed-priority — Pass 2 cost ≈ 0.4 s vs
/// 1.0 s at PASS1=75). Override per-call via `MFSK_PASS1_LIMIT`
/// when std is enabled.
const PASS1_LIMIT_DEFAULT: usize = 30;
fn pass1_limit() -> usize {
    #[cfg(feature = "std")]
    {
        if let Ok(s) = std::env::var("MFSK_PASS1_LIMIT")
            && let Ok(v) = s.parse::<usize>()
        {
            return v;
        }
    }
    PASS1_LIMIT_DEFAULT
}

/// One Pass-2 output: the original candidate, its 79×8 Costas-only
/// spectrum (filled in stage 3 with the data-symbol DFT), and its
/// `sync_quality` score for ranking.
pub type RefinedCandidate = (SyncCandidate, Box<[[Cmplx<f32>; 8]; 79]>, u32);

/// Per-candidate Costas-block-0 DFT + sync_quality_block0 re-rank.
/// Keeps the top `max_cand` by Pass-2 score; **the cs spectrum is
/// retained** (block 0 only at this point) and stage 3 fills the
/// remaining 72 symbols via [`SymMask::NotBlock0`].
///
/// Cost: 7 sync symbols × 8 tones = 56 DFT per candidate vs
/// `SyncOnly`'s 168 — 1/3 the work. On Core2 ~13 ms/cand with the
/// asm dot product. PASS1=75 → Pass 2 ≈ 1.0 s.
///
/// The retained `q` is the **block-0** score (range 0..=7, expected
/// ~6-7 for real signals, ~0.875 for noise). Stage 3 recomputes the
/// full 21-symbol `sync_quality` after filling blocks 1, 2 and uses
/// that for its `q > 6` gate; the per-cand sort here is just for
/// truncating to `max_cand`.
fn refine_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
) -> Vec<RefinedCandidate> {
    refine_candidates_with(cands, max_cand, |c| {
        symbol_spectra_direct(audio, c.freq_hz, c.dt_sec, SymMask::SyncBlock0)
    })
}

/// Variant of [`refine_candidates`] that uses caller-provided basis
/// scratch (forwards to `symbol_spectra_direct_into`).
///
/// Public as of v0.6 (#49 cat C): used by the embedded
/// `embedded-shared::dual_core` worker to keep the basis scratch
/// allocated in internal DRAM across stages.
#[cfg(feature = "fixed-point")]
pub fn refine_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<RefinedCandidate> {
    refine_candidates_with(cands, max_cand, |c| {
        symbol_spectra_direct_into(
            audio,
            c.freq_hz,
            c.dt_sec,
            SymMask::SyncBlock0,
            basis_re,
            basis_im,
        )
    })
}

/// Common min-heap selection logic used by both `refine_candidates`
/// (heap-allocated basis per call) and `refine_candidates_into`
/// (caller-provided basis scratch). The closure abstracts how each
/// candidate's cs Box is produced.
fn refine_candidates_with<F>(
    cands: Vec<SyncCandidate>,
    max_cand: usize,
    mut cs_for: F,
) -> Vec<RefinedCandidate>
where
    F: FnMut(&SyncCandidate) -> Box<[[Cmplx<f32>; 8]; 79]>,
{
    use alloc::collections::BinaryHeap;
    use core::cmp::{Ordering, Reverse};

    // Min-heap on q so the smallest survivor is at the top — replace
    // it whenever a stronger candidate arrives. Bounds the live heap
    // footprint at `max_cand × cs Box` regardless of PASS1_LIMIT,
    // which is the heap-fragmentation fix Task #2 was opened for.
    // Old code collected all PASS1=30 cs Boxes (240 KB) before the
    // truncate; new code never holds more than max_cand=15 Boxes
    // (120 KB peak).
    //
    // The heap stores (q, cand_idx, RefinedCandidate); cand_idx
    // breaks ties deterministically (insertion order) so the
    // truncation result is reproducible across runs.
    struct Slot {
        q: u32,
        idx: u32,
        cand: SyncCandidate,
        cs: Box<[[Cmplx<f32>; 8]; 79]>,
    }
    impl PartialEq for Slot {
        fn eq(&self, other: &Self) -> bool {
            self.q == other.q && self.idx == other.idx
        }
    }
    impl Eq for Slot {}
    impl Ord for Slot {
        fn cmp(&self, other: &Self) -> Ordering {
            self.q.cmp(&other.q).then_with(|| self.idx.cmp(&other.idx))
        }
    }
    impl PartialOrd for Slot {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    let mut heap: BinaryHeap<Reverse<Slot>> = BinaryHeap::with_capacity(max_cand + 1);
    for (idx, c) in cands.into_iter().enumerate() {
        let cs = cs_for(&c);
        let q = sync_quality_block0(&cs);
        let slot = Slot {
            q,
            idx: idx as u32,
            cand: c,
            cs,
        };
        if heap.len() < max_cand {
            heap.push(Reverse(slot));
        } else if let Some(Reverse(top)) = heap.peek()
            && slot.q > top.q
        {
            heap.pop();
            heap.push(Reverse(slot));
            // else branch: drop slot.cs immediately when it leaves scope
        }
    }
    let mut out: Vec<RefinedCandidate> = heap
        .into_iter()
        .map(|r| {
            let s = r.0;
            (s.cand, s.cs, s.q)
        })
        .collect();
    out.sort_by_key(|r| core::cmp::Reverse(r.2));
    out
}

/// Hard-decision sync quality on Costas **block 0 only** (symbols
/// 0..7). Cheaper variant of [`sync_quality`] for Pass 2 — checks
/// only one of the three Costas blocks. Range 0..=7.
///
/// Pub-but-doc-hidden so embedded callers (e.g. the m5stack-core2
/// PoC's manual Pass 2) can re-rank coarse_sync candidates by this
/// metric without pulling in the full `decode_block` D-pattern.
#[doc(hidden)]
pub fn sync_quality_block0<S: crate::core::scalar::SpecScalar>(cs: &[[Cmplx<S>; 8]; 79]) -> u32
where
    S::Wide: PartialOrd,
{
    let mut count = 0u32;
    for (t, &expected) in COSTAS.iter().enumerate() {
        let sym = t; // block 0 starts at symbol 0
        let best = (0..NTONES)
            .max_by(|&a, &b| {
                let na = cs[sym][a].norm_sqr_wide();
                let nb = cs[sym][b].norm_sqr_wide();
                na.partial_cmp(&nb).unwrap_or(core::cmp::Ordering::Equal)
            })
            .unwrap_or(0);
        if best == expected {
            count += 1;
        }
    }
    count
}

/// LLR / BP scalar for the hot loop. `Q3i8` under `fixed-point`
/// (embedded integer pipeline; recall-equivalent to Q11i16 with half
/// the BP scratch — Issue #15 Phase 1 validated 2026-05-03), `f32`
/// otherwise (host / FPU targets). Both go through the same generic
/// NMS implementation in `fec::ldpc::bp`.
///
/// **0.6.3**: switched embedded LlrT from `Q3i8` (i8, ±16 range, 1/8
/// resolution) to `Q11i16` (i16, ±16 range, 1/2048 resolution). The
/// Q3i8 quantization step (~0.875 LLR units between codes) was the
/// dominant recall ceiling on Xtensa builds — host fixed-point +
/// rustfft hit 16/18 with f32, 9/18 with Q3i8, on `qso3_busy.wav`.
/// Q11i16's 1/2048 resolution recovers most of that gap (target
/// embedded recall: 6/18 → ~10/18). Cost: BP scratch doubles from
/// ~6 KB to ~12 KB, still inside S3 / Core2 internal-DRAM budget.
#[cfg(feature = "fixed-point")]
type LlrT = crate::core::scalar::Q11i16;
#[cfg(not(feature = "fixed-point"))]
type LlrT = f32;

/// BP-kind switch (host-only). **Default `tanh`** (= WSJT-X
/// `bpdecode174_91.f90` log-domain tanh-product, our
/// `BpKind::SumProduct`) — that's the golden reference. The embedded
/// ship path keeps `NormalizedMinSum` (α=0.75) for speed; on host we
/// pay the tanh / atanh per-iteration cost in exchange for
/// numerically-correct convergence. The env var
/// `MFSK_BP_KIND=nms` opts into the approximation for A/B
/// comparison, but is NOT for production use — NMS appears to "find
/// more decodes" only because its approximation error sometimes
/// happens to land on a CRC-passing codeword that tanh would
/// (correctly) reject.
#[cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]
#[inline]
fn bp_step_select<T: crate::core::scalar::LlrScalar>(
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<crate::fec::ldpc::Ldpc174_91Params, T>,
    llr: &[T; LDPC_N],
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<crate::fec::ldpc::bp::BpResult> {
    if std::env::var("MFSK_BP_KIND").as_deref() == Ok("nms") {
        return crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<T>(
            bp_scratch, llr, None, max_iter, verify, NMS_ALPHA,
        );
    }
    let llr_f32: [f32; LDPC_N] = core::array::from_fn(|i| llr[i].to_f32());
    crate::fec::ldpc::bp::bp_decode(&llr_f32, None, max_iter, verify)
}

#[cfg(any(not(feature = "fft-rustfft"), feature = "fixed-point"))]
#[inline]
fn bp_step_select<T: crate::core::scalar::LlrScalar>(
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<crate::fec::ldpc::Ldpc174_91Params, T>,
    llr: &[T; LDPC_N],
    max_iter: u32,
    verify: Option<fn(&[u8]) -> bool>,
) -> Option<crate::fec::ldpc::bp::BpResult> {
    crate::fec::ldpc::bp::bp_decode_nms_with_scratch::<T>(
        bp_scratch, llr, None, max_iter, verify, NMS_ALPHA,
    )
}

/// Stage 3: take Pass-2 refined candidates (cand + Costas-only cs +
/// sync_quality), fill in the data-symbol spectra, run LLR + BP/OSD
/// staircase. The Costas DFT was already done in Pass 2 — we only
/// add the data-symbol DFT here.
///
/// `q_thresh` is the post-fill `sync_quality` early-reject threshold
/// (see [`DEFAULT_Q_THRESH`]).
///
/// **Pub for benchmarking only — do not depend on it.**
#[doc(hidden)]
pub fn process_candidates<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
) -> Vec<DecodeResult> {
    process_candidates_tuned(audio, cands, depth, q_thresh, DEFAULT_BP_MAX_ITER)
}

/// Variant of [`process_candidates`] that accepts a runtime
/// `bp_max_iter` (= per-LLR-variant BP iteration cap). Embedded
/// LX6 / LX7 callers tune this to trade weak-signal recall for
/// post-SlotEnd time budget.
#[doc(hidden)]
pub fn process_candidates_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
) -> Vec<DecodeResult> {
    process_candidates_tuned_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        None,
        DecodeStrictness::Normal,
    )
}

/// AP-aware variant of [`process_candidates_tuned`]. When
/// `ap_hint = None` and `strictness = Normal`, behaviour is bit-
/// identical to `process_candidates_tuned`. Internal helper for the
/// `decode_block_with_ap` driver and host's redirected
/// `process_candidate`.
#[allow(clippy::too_many_arguments)]
fn process_candidates_tuned_with_ap<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult> {
    let mut cs_scratch: alloc::boxed::Box<[[Cmplx<f32>; 8]; 79]> =
        alloc::vec![[Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    process_candidates_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        &mut cs_scratch,
        |cs, cand, mask| {
            fill_symbol_spectra(cs, audio, cand.freq_hz, cand.dt_sec, mask, None);
        },
        ap_hint,
        strictness,
    )
}

/// Variant of [`process_candidates`] that uses caller-provided basis
/// scratch — required on Core2 / Cortex-M for the asm dot product
/// to reach its theoretical 1-cycle/sample speed (default heap
/// allocation routes the 60 KB × 2 basis to PSRAM /
/// non-cacheable RAM and erases the kernel's advantage).
///
/// **Pub for benchmarking + manually-staged callers** (e.g.
/// m5stack-core2 main.rs which logs per-stage wall-clock).
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
pub fn process_candidates_into<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    process_candidates_into_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        DEFAULT_BP_MAX_ITER,
        basis_re,
        basis_im,
    )
}

/// Variant of [`process_candidates_into`] that accepts a runtime
/// `bp_max_iter`. Same role as [`process_candidates_tuned`] but for
/// the caller-provided-basis path.
#[cfg(feature = "fixed-point")]
#[doc(hidden)]
#[allow(clippy::too_many_arguments)]
pub fn process_candidates_into_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
) -> Vec<DecodeResult> {
    let mut cs_scratch: alloc::boxed::Box<[[Cmplx<f32>; 8]; 79]> =
        alloc::vec![[Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    process_candidates_into_with_cs_scratch_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        basis_re,
        basis_im,
        &mut cs_scratch,
    )
}

/// Variant of [`process_candidates_into`] that also accepts a
/// caller-provided per-symbol-spectra scratch (`cs_scratch`, 5 KB =
/// `[[Cmplx<f32>; 8]; 79]`). Each candidate's PSRAM-resident `cs Box`
/// is copied into this scratch before `fill_symbol_spectra_into` /
/// LLR / BP run on it, then dropped — letting the BP / LLR hot loops
/// read `cs` from internal DRAM (~5–10× faster than PSRAM on Xtensa
/// LX6/LX7). Provide a `static mut` array in `.bss` for max win.
///
/// Public as of v0.6 (#49 cat C): used by the embedded
/// `embedded-shared::dual_core::stage3_split` worker.
#[cfg(feature = "fixed-point")]
pub fn process_candidates_into_with_cs_scratch<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
) -> Vec<DecodeResult> {
    process_candidates_into_with_cs_scratch_tuned(
        audio,
        cands,
        depth,
        q_thresh,
        DEFAULT_BP_MAX_ITER,
        basis_re,
        basis_im,
        cs_scratch,
    )
}

/// Variant of [`process_candidates_into_with_cs_scratch`] that accepts
/// a runtime `bp_max_iter`. This is the entry point used by the
/// embedded `dual_core::stage3_split` worker so LX6 / LX7 binaries
/// can dial the BP cap without rebuilding `mfsk-core`.
///
/// Public as of v0.6 (#49 cat C).
#[cfg(feature = "fixed-point")]
#[allow(clippy::too_many_arguments)]
pub fn process_candidates_into_with_cs_scratch_tuned<S: AudioSample>(
    audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    basis_re: &mut [i16],
    basis_im: &mut [i16],
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
) -> Vec<DecodeResult> {
    // Host fft-rustfft path doesn't need the basis buffers — cs is
    // built via the cd0 32-pt FFT instead. The parameters stay in
    // the signature so embedded callers (no fft-rustfft) get the
    // same shape; reference them here to silence unused warnings.
    #[cfg(feature = "fft-rustfft")]
    {
        let _ = &basis_re;
        let _ = &basis_im;
    }
    process_candidates_with_ap(
        audio,
        cands,
        depth,
        q_thresh,
        bp_max_iter,
        cs_scratch,
        |cs, cand, mask| {
            // Host fft-rustfft: use the cd0-based 32-pt FFT cs builder
            // (= ft8b.f90:154-161). Same path as host f32. fixed-point
            // with rustfft (= host) gets WSJT-X-faithful cs construction
            // identical to the f32 build.
            #[cfg(feature = "fft-rustfft")]
            {
                // basis_{re,im} unused on this path — borrow nothing.
                // fft_cache=None here because process_candidates_into_tuned
                // is the embedded entry, not the slot-cache pipeline.
                fill_symbol_spectra(cs, audio, cand.freq_hz, cand.dt_sec, mask, None);
            }
            // Embedded fixed-point (no fft-rustfft): keep the
            // basis-precompute + dot-product path — no 192k FFT
            // available on Xtensa.
            #[cfg(not(feature = "fft-rustfft"))]
            fill_symbol_spectra_into(
                cs,
                audio,
                cand.freq_hz,
                cand.dt_sec,
                mask,
                basis_re,
                basis_im,
            );
        },
        None,
        DecodeStrictness::Normal,
    )
}

/// Common body of `process_candidates` / `process_candidates_into`
/// / `process_candidates_into_with_cs_scratch` — the BP staircase
/// logic is identical between them; only the per-candidate
/// `fill_symbol_spectra(_into)` call differs (heap-allocated basis vs
/// caller-provided). `cs_scratch` is the per-symbol-spectra working
/// buffer that hot loops (BP / LLR / sync_quality) read from — see
/// [`process_candidates_into_with_cs_scratch`] for the rationale.
/// Per-candidate driver — dt is already parabolically refined by
/// coarse_sync. AP-aware: pass `ap_hint = None` for the legacy
/// non-AP behaviour (bit-identical to the pre-0.6.1 shape).
#[allow(clippy::too_many_arguments)]
fn process_candidates_with_ap<S: AudioSample, F>(
    _audio: &[S],
    cands: Vec<RefinedCandidate>,
    depth: DecodeDepth,
    q_thresh: u32,
    bp_max_iter: u32,
    cs_scratch: &mut [[Cmplx<f32>; 8]; 79],
    mut fill: F,
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
) -> Vec<DecodeResult>
where
    F: FnMut(&mut [[Cmplx<f32>; 8]; 79], &SyncCandidate, SymMask),
{
    // dt is already parabolically refined by coarse_sync; no grid here.

    let mut results: Vec<DecodeResult> = Vec::new();
    // BP scratch pool — instantiated once and reused across all
    // candidates × all 5 BP calls per candidate. Eliminates the
    // ~12 KB-per-call `tlsf_malloc` traffic that dominated stage-3
    // non-DFT cost on Core2 (~50–100 ms / qso). See
    // `mfsk_core::fec::ldpc::bp::BpScratch`.
    let mut bp_scratch =
        crate::fec::ldpc::bp::BpScratch::<crate::fec::ldpc::params::Ldpc174_91Params, LlrT>::new();
    for (cand, cs_box, _q_block0) in cands {
        // Stage cs into the caller's scratch (typically internal DRAM
        // on Xtensa) so the LLR / BP / sync_quality hot loops below
        // read from fast memory. The PSRAM-resident `cs_box` is
        // dropped immediately after the copy. Cost: ~60 µs (5 KB at
        // ~80 MB/s OCT PSRAM read on S3) vs many-hundred-µs gains in
        // BP iter loop.
        *cs_scratch = *cs_box;
        drop(cs_box);
        // Two-step fill: sync blocks first, gate by full sync_quality,
        // then fill data symbols only for survivors. Saves the 58 ×
        // 8 = 464 data-symbol DFTs on every candidate that fails the
        // q gate (typically half of `max_cand`). `SyncBlocks12`
        // (instead of `SyncOnly`) skips re-filling block 0 — Pass 2
        // already populated it via `symbol_spectra_direct_into` on
        // `SyncBlock0`, and that data survives in `cs_scratch` here.
        // Saves an additional 56 DFTs / candidate.
        fill(cs_scratch, &cand, SymMask::SyncBlocks12);
        let q = sync_quality(cs_scratch);
        if q <= q_thresh {
            continue;
        }
        fill(cs_scratch, &cand, SymMask::DataOnly);
        if let Some(r) = process_one_candidate_inner(
            cs_scratch,
            &cand,
            cand.dt_sec,
            q,
            depth,
            bp_max_iter,
            &mut bp_scratch,
            &results,
            ap_hint,
            strictness,
            0.0,
        ) {
            results.push(r);
        }
    }

    results
}

/// WSJT-X-faithful nharderrors gate (ft8b.f90:422). High hard-error
/// CRC-pass decodes are the dominant phantom source on busy bands;
/// reject above this and fall through to the next BP variant.
pub(super) const WSJTX_NHARDERRORS_MAX: u32 = 36;

/// Per-candidate decode core — runs the LLR-staircase, OSD fallback,
/// and AP iaptype loop on a *fully-filled* `cs_scratch`. Shared
/// between the embedded `process_candidates_with` driver (above) and
/// the host `process_candidate` redirect (decode.rs, post-0.6.1).
/// Returns `Some(DecodeResult)` on first success (Step 1, 2, 3, or 4
/// in order), `None` on full failure.
///
/// Caller responsibilities:
/// - `cs_scratch` filled with all 79 symbols × 8 tones (data + sync).
/// - `q = sync_quality(cs_scratch)` already computed and gated.
/// - `refined_dt` carries the parabolically-refined `dt_sec`.
/// - `known` is the dedup list (in-progress + prior-pass results).
/// - `ap_hint` is `None` for AP-off (embedded default); `Some(_)` to
///   activate Step 4 AP iaptype loop. `#[cfg(feature = "fft-rustfft")]`
///   only — embedded fixed-point builds always pass `None`.
/// - `strictness` controls the per-iaptype `nharderrors` cap; ignored
///   when `ap_hint` is `None`.
/// - `sync_cv` is the Costas-array power CV (host computes it for QSB
///   gain attenuation; embedded passes 0.0).
#[allow(clippy::too_many_arguments)]
#[allow(unused_variables)] // ap_hint / strictness only used under #[cfg(feature = "fft-rustfft")]
pub(in crate::ft8) fn process_one_candidate_inner(
    cs_scratch: &[[Cmplx<f32>; 8]; 79],
    cand: &SyncCandidate,
    refined_dt: f32,
    q: u32,
    depth: DecodeDepth,
    bp_max_iter: u32,
    bp_scratch: &mut crate::fec::ldpc::bp::BpScratch<
        crate::fec::ldpc::params::Ldpc174_91Params,
        LlrT,
    >,
    known: &[DecodeResult],
    ap_hint: Option<&ApHint>,
    strictness: DecodeStrictness,
    sync_cv: f32,
) -> Option<DecodeResult> {
    // ── Staircase: cheap → deeper → OSD ─────────────────────────
    //
    // 1) Bp(llra) on the fast nsym=1 LLR. Most candidates that
    //    decode at all decode here; the rest fall through.
    // 2) Full compute_llr (nsym=1+2+3) → Bp on all 4 variants
    //    (a/b/c/d).
    // 3) OSD-1 / OSD-3 fallback gated on sync_quality.
    //
    // `BpAll` and `BpAllOsd` enable the deeper stages; plain
    // `Bp` stops after step 1.
    let mut accepted: Option<(crate::fec::ldpc::bp::BpResult, u8)> = None;

    // Step 1: fast llra. The LLR / BP scalar is selected at compile
    // time via `fixed-point` (Q3i8) or default (f32) — see the
    // `LlrT` definition above. Both go through the *same* generic
    // NMS implementation, bit-identical AWGN behaviour by design.
    let llr_a_fast: super::super::llr::LlrSet<LlrT> =
        super::super::llr::compute_llr_fast(cs_scratch);
    let bp_step1 =
        bp_step_select::<LlrT>(bp_scratch, &llr_a_fast.llra, bp_max_iter, Some(check_crc14));
    if let Some(bp) = bp_step1
        && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
    {
        accepted = Some((bp, 0));
    }

    // Step 2: deeper-LLR variants. Lazy + LLR-shared with Step 1.
    //
    // **Variant a is skipped** — Step 1 already ran BP on the
    // identical input (`compute_llr_fast` and `compute_llr`
    // produce bit-identical `llra`, since nsym=1 work doesn't
    // depend on `max_nsym`). Re-running it would be guaranteed
    // failure.
    //
    // **Variant d reuses** Step 1's `llr_a_fast.llrd` — same
    // nsym=1 derivation, costs zero LLR work.
    //
    // **Variants b / c are lazy-computed**: only pay the nsym=2
    // work if d failed, and only pay the heavy nsym=3 work
    // (~80 % of `compute_llr`) if both d and b also failed.
    // Order chosen by ascending compute cost — same number of BP
    // calls as the old variant loop in the worst case, far fewer
    // in the typical case where any earlier variant decodes.
    if accepted.is_none() && matches!(depth, DecodeDepth::BpAll | DecodeDepth::BpAllOsd) {
        // Variant d: free reuse of Step 1's llrd.
        let bp_d =
            bp_step_select::<LlrT>(bp_scratch, &llr_a_fast.llrd, bp_max_iter, Some(check_crc14));
        if let Some(bp) = bp_d
            && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
        {
            accepted = Some((bp, 3));
        }
        // Variant b: lazy nsym=2 only.
        if accepted.is_none() {
            let llrb_arr: [LlrT; LDPC_N] =
                super::super::llr::compute_llr_partial::<LlrT>(cs_scratch, 2);
            let bp_b =
                bp_step_select::<LlrT>(bp_scratch, &llrb_arr, bp_max_iter, Some(check_crc14));
            if let Some(bp) = bp_b
                && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
            {
                accepted = Some((bp, 1));
            }
        }
        // Variant c: lazy nsym=3 (the expensive one).
        if accepted.is_none() {
            let llrc_arr: [LlrT; LDPC_N] =
                super::super::llr::compute_llr_partial::<LlrT>(cs_scratch, 3);
            let bp_c =
                bp_step_select::<LlrT>(bp_scratch, &llrc_arr, bp_max_iter, Some(check_crc14));
            if let Some(bp) = bp_c
                && bp.hard_errors <= WSJTX_NHARDERRORS_MAX
            {
                accepted = Some((bp, 2));
            }
        }
    }

    // Step 3: OSD fallback (sync_quality gated; only for BpAllOsd).
    // The actual dispatch — including the WSJT-X-faithfulness
    // deviation #63 tracks restoring — lives in `super::osd_strategy`
    // so #63 can patch it without touching the rest of this function.
    if accepted.is_none() {
        accepted = super::osd_strategy::try_fallback(cs_scratch, depth, q);
    }

    // Step 4: AP iaptype loop (host f32 only; gated on fft-rustfft).
    // Mirrors WSJT-X ft8b.f90 ipass 5..12 — runs only if Steps 1-3 all
    // failed and the caller supplied an `ap_hint`. Embedded fixed-point
    // builds always pass `ap_hint = None` so this entire block
    // constant-folds away.
    //
    // Iaptype priority (deepest-first, mirroring decode.rs:634-684):
    //   9/10/11 (call1+call2+RRR/RR73/73 → full 77-bit lock)
    //   7 (CQ + call2)
    //   8 (call1+call2 → ~61 bits)
    //   6 (ap as-supplied → ~33 bits)
    //   5 (mycall only → ~32 bits, WSJT-X iaptype 2)
    //   12 (blind CQ → always tried last, WSJT-X iaptype 1)
    #[cfg(feature = "fft-rustfft")]
    if accepted.is_none()
        && let Some(ap) = ap_hint
    {
        let llr_full_f32: super::super::llr::LlrSet<f32> =
            super::super::llr::compute_llr(cs_scratch);
        let apmag = llr_full_f32
            .llra
            .iter()
            .map(|v| v.abs())
            .fold(0.0f32, f32::max)
            * 1.01;
        let llr_variants: [&[f32; LDPC_N]; 4] = [
            &llr_full_f32.llra,
            &llr_full_f32.llrb,
            &llr_full_f32.llrc,
            &llr_full_f32.llrd,
        ];

        let mut ap_passes: alloc::vec::Vec<(ApHint, u8)> = alloc::vec::Vec::new();
        if ap.has_info() {
            if ap.call1.is_some() && ap.call2.is_some() {
                for (rpt, pid) in [("RRR", 9u8), ("RR73", 10), ("73", 11)] {
                    let ap_full = ap.clone().with_report(rpt);
                    ap_passes.push((ap_full, pid));
                }
            }
            if ap.call2.is_some() && ap.call1.is_none() {
                let ap7 = ap.clone().with_call1("CQ");
                ap_passes.push((ap7, 7));
            }
            if ap.call1.is_some() && ap.call2.is_some() {
                ap_passes.push((ap.clone(), 8));
            }
            ap_passes.push((ap.clone(), 6));
            if ap.call1.is_some() {
                let mut ap5 = ApHint::new();
                if let Some(ref c1) = ap.call1 {
                    ap5 = ap5.with_call1(c1);
                }
                ap_passes.push((ap5, 5));
            }
        }
        // Pass 12: blind-CQ (WSJT-X iaptype 1) — always tried.
        ap_passes.push((ApHint::new().with_call1("CQ"), 12));

        'ap_outer: for (ap_cfg, pass_id) in &ap_passes {
            let (ap_mask, ap_llr_override) = ap_cfg.build_ap(apmag);
            let locked_bits = ap_mask.iter().filter(|&&m| m).count();
            let max_errors: u32 = strictness.ap_max_errors(locked_bits);

            for &base_llr in &llr_variants {
                let mut llr_ap = *base_llr;
                for i in 0..LDPC_N {
                    if ap_mask[i] {
                        llr_ap[i] = ap_llr_override[i];
                    }
                }
                // Inline AP-result validator: hard-error gate, unpack,
                // plausibility, locked-call substring check.
                let validate = |msg77: [u8; 77], hard_errors: u32| -> bool {
                    if hard_errors >= max_errors {
                        return false;
                    }
                    let Some(text) = unpack77(&msg77) else {
                        return false;
                    };
                    if !crate::msg::wsjt77::is_plausible_message(&text) {
                        return false;
                    }
                    let upper = text.to_uppercase();
                    if let Some(ref c1) = ap_cfg.call1
                        && !upper.contains(&c1.to_uppercase())
                    {
                        return false;
                    }
                    if let Some(ref c2) = ap_cfg.call2
                        && !upper.contains(&c2.to_uppercase())
                    {
                        return false;
                    }
                    true
                };

                // AP + BP
                if let Some(bp) = crate::fec::ldpc::bp::bp_decode(
                    &llr_ap,
                    Some(&ap_mask),
                    bp_max_iter,
                    Some(check_crc14),
                ) && validate(bp.message77, bp.hard_errors)
                {
                    accepted = Some((bp, *pass_id));
                    break 'ap_outer;
                }
                // AP + OSD-Deep fallback (BpAllOsd only)
                if depth == DecodeDepth::BpAllOsd
                    && let Some(osd) = osd_decode_deep(&llr_ap, 2, Some(check_crc14))
                    && validate(osd.message77, osd.hard_errors)
                {
                    // Reuse `osd.codeword` — `OsdResult` already
                    // carries the decoded bits; the previous
                    // `vec![0; LDPC_N]` was both wasteful and dropped
                    // the real codeword on the floor (Gemini PR #86
                    // review).
                    let bp = crate::fec::ldpc::bp::BpResult {
                        message77: osd.message77,
                        info: osd.info,
                        codeword: osd.codeword,
                        hard_errors: osd.hard_errors,
                        iterations: 0,
                    };
                    accepted = Some((bp, *pass_id));
                    break 'ap_outer;
                }
            }
        }
    }

    let (bp, pass_id) = accepted?;
    let text = unpack77(&bp.message77)?;
    // Plausibility filter — reject CRC-passing-but-garbage
    // messages. With max_cand=200 × 4 LLR variants × OSD,
    // CRC-14's 1/16384 false-positive rate produces ~1-2 random
    // strings per slot. Same filter the host wide-band path
    // uses (`decode_frame::process_candidate`).
    if !crate::msg::wsjt77::is_plausible_message(&text) {
        return None;
    }
    if known.iter().any(|r| r.message77 == bp.message77) {
        return None;
    }
    let itone = message_to_tones(&bp.message77);
    let snr_db = super::super::llr::compute_snr_db(cs_scratch, &itone);
    Some(DecodeResult {
        message77: bp.message77,
        freq_hz: cand.freq_hz,
        dt_sec: refined_dt,
        hard_errors: bp.hard_errors,
        sync_score: cand.score,
        pass: pass_id,
        sync_cv,
        snr_db,
    })
}
