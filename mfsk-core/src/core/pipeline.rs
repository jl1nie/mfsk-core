//! Protocol-agnostic decode pipeline (basic path, no AP hints).
//!
//! Generic versions of `decode_frame` and `decode_frame_subtract` that drive
//! sync → downsample → LLR → FEC for any `P: Protocol`. AP-assisted decoding
//! (which depends on the 77-bit WSJT message bit layout) lives in
//! protocol-specific crates.

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use super::dsp::downsample::{DownsampleCfg, build_fft_cache, downsample_cached};
use super::dsp::subtract::{SubtractCfg, subtract_tones};
use super::equalize::{EqMode, equalize_local};
use super::llr::{compute_llr, compute_snr_db, symbol_spectra, sync_quality};
use super::sync::{SyncCandidate, coarse_sync, fine_sync_power_per_block, refine_candidate};
use super::tx::codeword_to_itone;
use super::{FecCodec, FecOpts, Protocol};
use num_complex::Complex;

/// FFT cache for the initial large forward transform; reusable across passes.
pub type FftCache = Vec<Complex<f32>>;

/// Decoding depth: which LLR variants to attempt and whether to use OSD.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecodeDepth {
    /// Belief-propagation only, using the llra metric (fast).
    Bp,
    /// BP across all four LLR variants (a, b, c, d).
    BpAll,
    /// BP on all variants, then OSD fallback when BP fails.
    BpAllOsd,
}

/// Decode strictness: trades off sensitivity vs false-positive rate.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum DecodeStrictness {
    Strict,
    #[default]
    Normal,
    Deep,
}

impl DecodeStrictness {
    /// Upper bound on `hard_errors` for non-AP OSD decode. Same calibration
    /// as the FT8 implementation; FT4/FST4 can re-tune later.
    pub fn osd_max_errors(self, osd_depth: u8) -> u32 {
        match (self, osd_depth) {
            (Self::Strict, 3) => 20,
            (Self::Strict, 4) => 24,
            (Self::Strict, _) => 22,
            (Self::Normal, 3) => 26,
            (Self::Normal, 4) => 30,
            (Self::Normal, _) => 29,
            (Self::Deep, 3) => 30,
            (Self::Deep, 4) => 36,
            (Self::Deep, _) => 40,
        }
    }

    /// Minimum coarse-sync score to enter OSD fallback.
    pub fn osd_score_min(self) -> f32 {
        match self {
            Self::Strict => 3.0,
            Self::Normal => 2.2,
            Self::Deep => 2.0,
        }
    }
}

/// One successfully decoded message. Protocol-agnostic; the 77-bit payload
/// assumption is shared by FT8 / FT4 / FT2 / FST4.
#[derive(Debug, Clone)]
pub struct DecodeResult {
    pub message77: [u8; 77],
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub hard_errors: u32,
    pub sync_score: f32,
    pub pass: u8,
    /// Coefficient of variation of the per-block Costas powers — near 0 for
    /// stable channels, elevated under QSB or fading.
    pub sync_cv: f32,
    pub snr_db: f32,
}

// ──────────────────────────────────────────────────────────────────────────
// Per-candidate processing
// ──────────────────────────────────────────────────────────────────────────

/// Decode a single sync candidate through the basic pipeline.
///
/// `fft_cache` must match the protocol's [`DownsampleCfg`]. `known` is used
/// to prevent redundant OSD work on frequencies with an existing decode.
pub fn process_candidate_basic<P: Protocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    refine_steps: i32,
    sync_q_min: u32,
) -> Option<DecodeResult> {
    let ntones = P::NTONES as usize;
    let n_sym = P::N_SYMBOLS as usize;
    let ds_rate = 12_000.0 / P::NDOWN as f32;
    let tx_start = P::TX_START_OFFSET_S;

    let cd0 = downsample_cached(fft_cache, cand.freq_hz, cfg);
    let refined = refine_candidate::<P>(&cd0, cand, refine_steps);
    let i_start = ((refined.dt_sec + tx_start) * ds_rate).round() as usize;

    let cs_raw = symbol_spectra::<P>(&cd0, i_start);
    let nsync = sync_quality::<P>(&cs_raw);
    if nsync <= sync_q_min {
        return None;
    }

    let per_block = fine_sync_power_per_block::<P>(&cd0, i_start);
    let sync_cv = if !per_block.is_empty() {
        let n = per_block.len() as f32;
        let mean = per_block.iter().sum::<f32>() / n;
        if mean > f32::EPSILON {
            let var = per_block.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / n;
            var.sqrt() / mean
        } else {
            0.0
        }
    } else {
        0.0
    };

    let _ = ntones;
    let _ = n_sym;
    let decode = |cs: &[Complex<f32>]| -> Option<DecodeResult> {
        let llr_set = compute_llr::<P>(cs);
        let variants = match depth {
            DecodeDepth::Bp => vec![(&llr_set.llra, 0u8)],
            DecodeDepth::BpAll | DecodeDepth::BpAllOsd => vec![
                (&llr_set.llra, 0),
                (&llr_set.llrb, 1),
                (&llr_set.llrc, 2),
                (&llr_set.llrd, 3),
            ],
        };

        let fec = P::Fec::default();
        let bp_opts = FecOpts {
            bp_max_iter: 30,
            osd_depth: 0,
            ap_mask: None,
        };

        for (llr, pass_id) in &variants {
            if let Some(r) = fec.decode_soft(llr, &bp_opts) {
                let msg77: [u8; 77] = r.info[..77].try_into().ok()?;
                let itone = encode_tones_for_snr::<P>(&msg77, &fec);
                let snr_db = compute_snr_db::<P>(cs, &itone);
                return Some(DecodeResult {
                    message77: msg77,
                    freq_hz: cand.freq_hz,
                    dt_sec: refined.dt_sec,
                    hard_errors: r.hard_errors,
                    sync_score: refined.score,
                    pass: *pass_id,
                    sync_cv,
                    snr_db,
                });
            }
        }

        if depth == DecodeDepth::BpAllOsd && nsync >= 12 && cand.score >= strictness.osd_score_min()
        {
            let freq_dup = known
                .iter()
                .any(|r| (r.freq_hz - cand.freq_hz).abs() < 20.0);
            if !freq_dup {
                let osd_depth: u8 = if nsync >= 18 { 3 } else { 2 };
                let osd_opts = FecOpts {
                    bp_max_iter: 30,
                    osd_depth: osd_depth as u32,
                    ap_mask: None,
                };
                for (llr, _) in &variants {
                    if let Some(r) = fec.decode_soft(llr, &osd_opts) {
                        if r.hard_errors >= strictness.osd_max_errors(osd_depth) {
                            continue;
                        }
                        let msg77: [u8; 77] = r.info[..77].try_into().ok()?;
                        let itone = encode_tones_for_snr::<P>(&msg77, &fec);
                        let snr_db = compute_snr_db::<P>(cs, &itone);
                        return Some(DecodeResult {
                            message77: msg77,
                            freq_hz: cand.freq_hz,
                            dt_sec: refined.dt_sec,
                            hard_errors: r.hard_errors,
                            sync_score: refined.score,
                            pass: if osd_depth == 3 { 5 } else { 4 },
                            sync_cv,
                            snr_db,
                        });
                    }
                }
                // OSD depth-4 Top-K pruning gated on high sync quality.
                if nsync >= 18 {
                    let osd4_opts = FecOpts {
                        bp_max_iter: 30,
                        osd_depth: 4,
                        ap_mask: None,
                    };
                    for (llr, _) in &variants {
                        if let Some(r) = fec.decode_soft(llr, &osd4_opts) {
                            if r.hard_errors >= strictness.osd_max_errors(4) {
                                continue;
                            }
                            let msg77: [u8; 77] = r.info[..77].try_into().ok()?;
                            let itone = encode_tones_for_snr::<P>(&msg77, &fec);
                            let snr_db = compute_snr_db::<P>(cs, &itone);
                            return Some(DecodeResult {
                                message77: msg77,
                                freq_hz: cand.freq_hz,
                                dt_sec: refined.dt_sec,
                                hard_errors: r.hard_errors,
                                sync_score: refined.score,
                                pass: 13,
                                sync_cv,
                                snr_db,
                            });
                        }
                    }
                }
            }
        }

        None
    };

    match eq_mode {
        EqMode::Off => decode(&cs_raw),
        EqMode::Local => {
            let mut cs_eq = cs_raw.clone();
            equalize_local::<P>(&mut cs_eq);
            decode(&cs_eq)
        }
        EqMode::Adaptive => {
            let mut cs_eq = cs_raw.clone();
            equalize_local::<P>(&mut cs_eq);
            if let Some(r) = decode(&cs_eq) {
                return Some(r);
            }
            decode(&cs_raw)
        }
    }
}

/// Re-encode the decoded 77-bit payload (plus its protocol-specific
/// CRC) back into tones for SNR estimation. Uses the protocol's
/// `Fec` for the code-level encode and `P::SYNC_MODE.blocks()` /
/// `P::GRAY_MAP` for the tone layout.
///
/// Supports the two info-layouts rs-ft8n ships today, selected by
/// `P::Fec::K`:
/// - **91-bit** (`K=91`): 77 msg + CRC-14 — FT8, FT4.
/// - **101-bit** (`K=101`): 77 msg + CRC-24 — FST4.
///
/// For any other K the helper falls back to "msg + zeros" which
/// still produces a valid codeword but with a broken CRC. That's a
/// fine approximation for SNR computation (amplitudes dominate);
/// real decodes still validate the CRC upstream.
fn encode_tones_for_snr<P: Protocol>(msg77: &[u8; 77], fec: &P::Fec) -> Vec<u8> {
    let k = P::Fec::K;
    let mut info = vec![0u8; k];
    info[..77].copy_from_slice(msg77);
    match k {
        91 => {
            let mut bytes = [0u8; 12];
            for (i, &bit) in msg77.iter().enumerate() {
                bytes[i / 8] |= (bit & 1) << (7 - i % 8);
            }
            let crc = crc14(&bytes);
            for i in 0..14 {
                info[77 + i] = ((crc >> (13 - i)) & 1) as u8;
            }
        }
        101 => {
            // Same convention as `check_crc24`: run crc24 over the
            // 101-bit word with the CRC slot zeroed.
            let mut with_zero = vec![0u8; 101];
            with_zero[..77].copy_from_slice(msg77);
            let crc = crc24(&with_zero);
            for i in 0..24 {
                info[77 + i] = ((crc >> (23 - i)) & 1) as u8;
            }
        }
        _ => {
            // Unknown layout — leave the tail zero. Only SNR is at
            // stake; decode correctness already verified upstream.
        }
    }
    let mut cw = vec![0u8; P::Fec::N];
    fec.encode(&info, &mut cw);
    codeword_to_itone::<P>(&cw)
}

/// Local CRC-24 for FST4 SNR re-encoding. Matches
/// `mfsk_core::fec::ldpc240_101::crc24` — duplicated here to keep the
/// dep graph acyclic.
fn crc24(bits: &[u8]) -> u32 {
    const POLY: [u8; 25] = [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1,
    ];
    let mut r = [0u8; 25];
    for (i, slot) in r.iter_mut().enumerate() {
        *slot = if i < bits.len() { bits[i] & 1 } else { 0 };
    }
    let n = bits.len().saturating_sub(25);
    for i in 0..=n {
        r[24] = if i + 25 <= bits.len() {
            bits[i + 24] & 1
        } else {
            0
        };
        if r[0] != 0 {
            for (rv, pv) in r.iter_mut().zip(POLY.iter()) {
                *rv ^= *pv;
            }
        }
        let first = r[0];
        for k in 0..24 {
            r[k] = r[k + 1];
        }
        r[24] = first;
    }
    let mut v = 0u32;
    for &b in &r[..24] {
        v = (v << 1) | (b as u32);
    }
    v
}

/// Local duplicate of the CRC-14 used by all LDPC(174,91)-based WSJT modes.
/// Keeping it inline in pipeline.rs avoids a cross-crate circular dep on
/// mfsk-fec, which depends on mfsk-core.
fn crc14(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            let msb = (crc >> 13) & 1;
            crc = ((crc << 1) | bit as u16) & 0x3FFF;
            if msb != 0 {
                crc ^= 0x2757;
            }
        }
    }
    crc
}

// ──────────────────────────────────────────────────────────────────────────
// Frame-level entry points
// ──────────────────────────────────────────────────────────────────────────

/// Decode one slot of audio: coarse sync → candidates → BP/OSD per candidate.
pub fn decode_frame<P: Protocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    refine_steps: i32,
    sync_q_min: u32,
) -> (Vec<DecodeResult>, FftCache) {
    let candidates = coarse_sync::<P>(audio, freq_min, freq_max, sync_min, freq_hint, max_cand);
    let fft_cache = build_fft_cache(audio, cfg);
    if candidates.is_empty() {
        return (Vec::new(), fft_cache);
    }

    #[cfg(feature = "parallel")]
    let raw: Vec<DecodeResult> = candidates
        .par_iter()
        .filter_map(|cand| {
            process_candidate_basic::<P>(
                cand,
                &fft_cache,
                cfg,
                depth,
                strictness,
                &[],
                eq_mode,
                refine_steps,
                sync_q_min,
            )
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let raw: Vec<DecodeResult> = candidates
        .iter()
        .filter_map(|cand| {
            process_candidate_basic::<P>(
                cand,
                &fft_cache,
                cfg,
                depth,
                strictness,
                &[],
                eq_mode,
                refine_steps,
                sync_q_min,
            )
        })
        .collect();

    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        if !results.iter().any(|x| x.message77 == r.message77) {
            results.push(r);
        }
    }
    (results, fft_cache)
}

/// Multi-pass decode with successive signal subtraction. Each pass decodes
/// the residual audio; decoded signals are reconstructed and subtracted so
/// subsequent passes can expose previously-masked weak signals.
pub fn decode_frame_subtract<P: Protocol>(
    audio: &[i16],
    ds_cfg: &DownsampleCfg,
    sub_cfg: &SubtractCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    refine_steps: i32,
    sync_q_min: u32,
) -> Vec<DecodeResult> {
    let mut residual = audio.to_vec();
    let mut all_results: Vec<DecodeResult> = Vec::new();
    let passes: &[f32] = &[1.0, 0.75, 0.5];
    let fec = P::Fec::default();

    for &factor in passes {
        let candidates = coarse_sync::<P>(
            &residual,
            freq_min,
            freq_max,
            sync_min * factor,
            freq_hint,
            max_cand,
        );
        if candidates.is_empty() {
            continue;
        }
        let fft_cache = build_fft_cache(&residual, ds_cfg);

        #[cfg(feature = "parallel")]
        let new: Vec<DecodeResult> = candidates
            .par_iter()
            .filter_map(|cand| {
                process_candidate_basic::<P>(
                    cand,
                    &fft_cache,
                    ds_cfg,
                    depth,
                    strictness,
                    &all_results,
                    EqMode::Off,
                    refine_steps,
                    sync_q_min,
                )
            })
            .collect();
        #[cfg(not(feature = "parallel"))]
        let new: Vec<DecodeResult> = candidates
            .iter()
            .filter_map(|cand| {
                process_candidate_basic::<P>(
                    cand,
                    &fft_cache,
                    ds_cfg,
                    depth,
                    strictness,
                    &all_results,
                    EqMode::Off,
                    refine_steps,
                    sync_q_min,
                )
            })
            .collect();

        let mut deduped: Vec<DecodeResult> = Vec::new();
        for r in new {
            if !all_results.iter().any(|k| k.message77 == r.message77)
                && !deduped.iter().any(|x| x.message77 == r.message77)
            {
                deduped.push(r);
            }
        }

        for r in &deduped {
            let gain = if r.sync_cv > 0.3 { 0.5 } else { 1.0 };
            let tones = encode_tones_for_snr::<P>(&r.message77, &fec);
            subtract_tones(&mut residual, &tones, r.freq_hz, r.dt_sec, gain, sub_cfg);
        }
        all_results.extend(deduped);
    }

    all_results
}
