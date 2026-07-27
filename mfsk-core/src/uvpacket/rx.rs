// SPDX-License-Identifier: GPL-3.0-or-later
//! RX path: 12 kHz f32 PCM audio → decoded frames (post 0.4.0
//! redesign).
//!
//! ## Pipeline
//!
//! 1. **Sync + mode identification** — correlate against all four
//!    127-chip preambles in one matched-filter pass per frequency
//!    candidate. The strongest peak's preamble identifies both the
//!    timing offset and the payload [`Mode`]. (No more brute-force
//!    LDPC sweep across `4 modes × 32 n_blocks` to discover layout.)
//! 2. **AFC** — frequency-grid coarse search around `audio_centre_hz`,
//!    parabolic refinement.
//! 3. **Symbol extraction** — sub-sample timing recovery + linear
//!    interpolation of the matched-filter output at NSPS spacing.
//! 4. **Equaliser training** — least-squares fit of a 9-tap T-spaced
//!    linear equaliser on the long preamble (known BPSK chips →
//!    closed-form normal-equations solve). Removes multipath ISI.
//! 5. **Differential demod** — `r_diff[k] = e[k] · conj(e[k-1])` on
//!    equaliser output. π/4-rotated to land on the standard QPSK
//!    constellation axes for the QPSK LLR step.
//! 6. **Amplitude / noise estimate** — joint estimator from the
//!    long preamble's known pair products; gives `a_sq_est`,
//!    `residual_rotation`, `sigma_sq_n_diff`.
//! 7. **Header decode** — 1 LDPC decode (Robust, unpunctured) of the
//!    first post-preamble block. Reads `n_payload_blocks` etc. from
//!    the recovered header bytes.
//! 8. **Payload decode** — `n_blocks` LDPC decodes (mode-puncture +
//!    de-interleave). Concatenate info bytes; verify the header's
//!    CRC over `header_word ++ padded_payload`.
//!
//! ## LDPC decode count
//!
//! `1 + n_blocks` per frame (vs `≤ 128` brute-force in the prior
//! design). Worst case (32-block frame) ≈ 33 LDPC decodes; typical
//! 16-byte payload Robust ≈ 1 + 2 = 3 LDPC decodes (~30 ms).

use std::f32::consts::PI;

use num_complex::Complex32;

use crate::engine::{FecCodec, FecOpts};
use crate::fec::Ldpc240_101;

use super::framing::{HEADER_BYTES, INFO_BYTES_PER_BLOCK, UnpackError, unpack_header};
use super::interleaver::deinterleave_llr;
use super::puncture::{Mode, de_puncture_llr};
use super::sync_pattern::{NUM_PREAMBLES, PREAMBLE_LEN, mode_for_index, preamble_for};
use super::tx::{RRC_ALPHA, RRC_SPAN_SYMS, SAMPLE_RATE_HZ, rrc_pulse};

/// LDPC mother-codeword length.
const N_LDPC: usize = 240;
/// LDPC info-bit count.
const K_LDPC: usize = 101;
/// Symbols carrying one full unpunctured LDPC codeword (240 ch bits / 2 bit/sym).
const HEADER_BLOCK_SYMS: usize = N_LDPC / 2;

/// Canonical 1200-baud per-symbol sample count. Used by every mode
/// except [`Mode::UltraRobust`] which runs at half baud (NSPS=20).
/// All auto-detect entry points compute one matched-filter output
/// per distinct nsps value (currently 10 and 20).
const NSPS_BASE: usize = 10;
/// Half-baud (600-baud) per-symbol sample count for UltraRobust.
const NSPS_ULTRA: usize = 20;
/// Both nsps variants the auto-detect path needs to scan.
const ALL_NSPS: [usize; 2] = [NSPS_BASE, NSPS_ULTRA];

/// RRC pulse length in samples for a given nsps (`span × nsps + 1`).
const fn rrc_len_for(nsps: usize) -> usize {
    RRC_SPAN_SYMS * nsps + 1
}

/// Symbol-peak position offset within the matched-filter output for
/// a transmitted symbol at TX baseband index 0, given nsps.
const fn sym_peak_offset_for(nsps: usize) -> usize {
    rrc_len_for(nsps) - 1
}

/// Equaliser tap count. 9 T-spaced (NSPS-spaced in baseband) taps,
/// centred (4 past + 1 centre + 4 future).
const EQ_N_TAPS: usize = 9;
const EQ_TAP_HALF: usize = EQ_N_TAPS / 2;
/// Diagonal loading for the LS solve (fraction of `R`'s trace).
const EQ_LS_DIAG_LOAD: f32 = 1e-2;

/// Errors returned by the decode functions.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum DecodeError {
    /// Audio buffer ended before a full preamble + header block
    /// could fit at `sample_offset`.
    Truncated,
    /// Audio buffer ended before all `n_blocks` payload blocks
    /// could fit (after header decode revealed `n_blocks`).
    PayloadTruncated { needed_samples: usize },
    /// Header LDPC block failed to decode (BP did not converge,
    /// even with OSD).
    HeaderFecFailed,
    /// At least one payload LDPC block failed to decode.
    PayloadFecFailed,
    /// Reassembled frame data failed CRC verification.
    Crc(UnpackError),
}

/// Result of a successful frame decode.
#[derive(Clone, Debug, PartialEq)]
pub struct DecodedFrame {
    pub app_type: u8,
    pub sequence: u8,
    pub mode: Mode,
    pub block_count: u8,
    pub payload: Vec<u8>,
    /// WSJT-X-compatible SNR estimate (dB, 2.5 kHz reference
    /// bandwidth). Floor at −30 dB. Computed from the joint
    /// amplitude / noise estimator on the long preamble.
    pub snr_db: f32,
}

/// AFC configuration.
#[derive(Clone, Copy, Debug)]
pub struct AfcOpts {
    /// One-sided search range in Hz (total window = ±search_hz).
    pub search_hz: f32,
}

impl Default for AfcOpts {
    fn default() -> Self {
        Self { search_hz: 200.0 }
    }
}

/// Default LDPC decode options.
pub fn default_fec_opts() -> FecOpts<'static> {
    FecOpts {
        bp_max_iter: 50,
        osd_depth: 2,
        ..FecOpts::default()
    }
}

// ────────────────────────────────────────────────────────────────────
// Public API — decode_known_layout
// ────────────────────────────────────────────────────────────────────

/// Decode a uvpacket frame whose start sample (`sample_offset`),
/// audio centre frequency, and payload [`Mode`] are known. Reads
/// `n_blocks` (and other header fields) from the dedicated header
/// block — no caller hint needed.
pub fn decode_known_layout(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    fec_opts: &FecOpts,
) -> Result<DecodedFrame, DecodeError> {
    decode_at_inner(audio, sample_offset, audio_centre_hz, mode, fec_opts)
}

/// AFC-wrapped variant of [`decode_known_layout`]. Searches for the
/// best frequency offset over `±afc_opts.search_hz` first, then
/// decodes at the corrected centre.
pub fn decode_known_layout_with_afc(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    fec_opts: &FecOpts,
    afc_opts: &AfcOpts,
) -> Result<DecodedFrame, DecodeError> {
    // AFC needs at least preamble + header-block worth of samples.
    let nsps = mode.nsps();
    let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * nsps + rrc_len_for(nsps);
    if sample_offset + needed > audio.len() {
        return Err(DecodeError::Truncated);
    }
    let delta_hz = estimate_freq_offset_for_mode(
        audio,
        sample_offset,
        needed,
        audio_centre_hz,
        mode,
        afc_opts,
    );
    decode_at_inner(
        audio,
        sample_offset,
        audio_centre_hz + delta_hz,
        mode,
        fec_opts,
    )
}

// ────────────────────────────────────────────────────────────────────
// Public API — auto-detect decode (single channel + multi-channel)
// ────────────────────────────────────────────────────────────────────

/// Auto-detect: scan the audio at `audio_centre_hz`, find every
/// candidate sync peak (any of the four preamble variants), and
/// decode each with the [`Mode`] the winning preamble identified.
pub fn decode(audio: &[f32], audio_centre_hz: f32) -> Vec<DecodedFrame> {
    decode_inner(audio, audio_centre_hz, &default_fec_opts())
}

/// Configuration for the multi-channel RX primitive and the
/// slot-occupancy survey helper.
#[derive(Clone, Copy, Debug)]
pub struct MultiChannelOpts {
    pub band_lo_hz: f32,
    pub band_hi_hz: f32,
    pub coarse_step_hz: f32,
    pub nms_radius_hz: f32,
}

impl Default for MultiChannelOpts {
    fn default() -> Self {
        Self {
            band_lo_hz: 300.0,
            band_hi_hz: 2700.0,
            coarse_step_hz: 25.0,
            nms_radius_hz: 600.0,
        }
    }
}

/// Per-slot received-signal-energy report.
#[derive(Clone, Copy, Debug)]
pub struct SlotEnergy {
    pub audio_centre_hz: f32,
    pub mean_mf_magnitude: f32,
}

/// Decode every uvpacket frame whose audio centre lies in
/// `[band_lo_hz, band_hi_hz]`. Returns `(detected_centre_hz, frame)`
/// pairs.
pub fn decode_multichannel(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    fec_opts: &FecOpts,
) -> Vec<(f32, DecodedFrame)> {
    decode_multichannel_inner(audio, mc_opts, fec_opts)
}

/// Measure the per-slot received-signal energy across the configured
/// band. Used by the LBT step before a random-slot TX.
pub fn measure_slot_energies(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    slot_spacing_hz: f32,
) -> Vec<SlotEnergy> {
    let mut out = Vec::new();
    if audio.is_empty() {
        return out;
    }
    // Slot-energy survey is mode-agnostic — use the canonical NSPS=10
    // MF; UltraRobust signals would still light up this filter (just
    // with ~3 dB MF mismatch loss, irrelevant for an energy survey).
    let half = slot_spacing_hz / 2.0;
    let mut centre = mc_opts.band_lo_hz + half;
    while centre <= mc_opts.band_hi_hz {
        let mf_out = downconvert_and_matched_filter(audio, centre, NSPS_BASE);
        let mean_mag = if mf_out.is_empty() {
            0.0
        } else {
            mf_out.iter().map(|c| c.norm_sqr()).sum::<f32>() / mf_out.len() as f32
        };
        out.push(SlotEnergy {
            audio_centre_hz: centre,
            mean_mf_magnitude: mean_mag,
        });
        centre += slot_spacing_hz;
    }
    out
}

// ────────────────────────────────────────────────────────────────────
// Public diagnostics
// ────────────────────────────────────────────────────────────────────

/// Peak / median / ratio of the differential preamble-correlation
/// score distribution at a given audio centre, evaluated against
/// every preamble variant and reported as `(mode, stats)`.
#[derive(Copy, Clone, Debug)]
pub struct SyncStats {
    pub global_max: f32,
    pub median: f32,
    /// `global_max / median`. Bounded above by ~`PREAMBLE_LEN − 1
    /// = 126` (Cauchy-Schwarz on 126 differential pairs).
    pub ratio: f32,
    pub n_scores: usize,
}

/// Run the differential preamble correlator at the given centre and
/// return both the winning mode and the score stats. Useful for
/// callers that want to inspect sync quality without running a full
/// LDPC decode.
///
/// `#[doc(hidden)]`: diagnostic escape hatch, not part of the
/// supported decode API (issue #203) — kept callable rather than
/// `pub(crate)` since it has no in-crate caller and exists purely for
/// external sync-quality debugging.
#[doc(hidden)]
pub fn diag_sync_at(audio: &[f32], audio_centre_hz: f32) -> Option<(Mode, SyncStats)> {
    // Need enough audio for the longest preamble (UltraRobust at
    // NSPS=20 = 127·20 + 121 = 2 661 samples, ≈ 222 ms at 12 kHz).
    let min_needed = PREAMBLE_LEN * NSPS_ULTRA + rrc_len_for(NSPS_ULTRA);
    if audio.len() < min_needed {
        return None;
    }
    let mut per_mode_max: [f32; NUM_PREAMBLES] = [0.0; NUM_PREAMBLES];
    let mut all_scores: Vec<f32> = Vec::new();
    let mut total_offsets = 0usize;
    // Compute one MF per nsps variant; correlate the matching
    // preambles against it.
    for &nsps in ALL_NSPS.iter() {
        let mf_out = downconvert_and_matched_filter(audio, audio_centre_hz, nsps);
        let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * nsps + 1);
        if max_corr_offset == 0 {
            continue;
        }
        // Resolve the nsps's preambles via mode_for_index → header_code
        // index, so per_mode_max[] stays indexed by header_code (the
        // public mode_idx the caller's per_mode_max snapshot uses).
        let mut local_idx_to_global = [0usize; NUM_PREAMBLES];
        let (nsps_modes, nsps_bits, np) = preambles_for_nsps(nsps);
        for j in 0..np {
            let m = nsps_modes[j].expect("populated by preambles_for_nsps");
            local_idx_to_global[j] = m.header_code() as usize;
        }
        if np == 0 {
            continue;
        }
        for offset in 0..max_corr_offset {
            let scores =
                preamble_differential_scores_multi(&mf_out, offset, &nsps_bits[..np], nsps);
            for j in 0..np {
                let s = scores[j];
                let g = local_idx_to_global[j];
                if s > per_mode_max[g] {
                    per_mode_max[g] = s;
                }
                all_scores.push(s);
                total_offsets += 1;
            }
        }
    }
    let (best_mode_idx, &best_score) = per_mode_max
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())?;
    let median = robust_median(&all_scores);
    let ratio = if median > 0.0 {
        best_score / median
    } else {
        0.0
    };
    let mode = mode_for_index(best_mode_idx)?;
    Some((
        mode,
        SyncStats {
            global_max: best_score,
            median,
            ratio,
            n_scores: total_offsets,
        },
    ))
}

/// AFC frequency-offset estimate for a known mode. Same algorithm
/// as inside [`decode_known_layout_with_afc`]'s AFC step, exposed
/// for diagnostic harnesses.
///
/// `#[doc(hidden)]`: diagnostic escape hatch, not part of the
/// supported decode API (issue #203) — see [`diag_sync_at`].
#[doc(hidden)]
pub fn diag_estimate_freq_offset(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    afc_opts: &AfcOpts,
) -> Option<f32> {
    let nsps = mode.nsps();
    let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * nsps + rrc_len_for(nsps);
    if sample_offset + needed > audio.len() {
        return None;
    }
    Some(estimate_freq_offset_for_mode(
        audio,
        sample_offset,
        needed,
        audio_centre_hz,
        mode,
        afc_opts,
    ))
}

// ────────────────────────────────────────────────────────────────────
// Internal — single-frame decode at a known layout/mode
// ────────────────────────────────────────────────────────────────────

fn decode_at_inner(
    audio: &[f32],
    sample_offset: usize,
    audio_centre_hz: f32,
    mode: Mode,
    fec_opts: &FecOpts,
) -> Result<DecodedFrame, DecodeError> {
    let nsps = mode.nsps();
    let rrc_len = rrc_len_for(nsps);
    // We don't yet know n_blocks. Phase 1: decode header. Phase 2:
    // once header reveals n_blocks, decode payload. The equaliser's
    // future-tap window past the last sampled symbol is zero-padded
    // by `sample_symbols`, so we only require the audio span for the
    // symbols themselves (preamble + header block).
    let need_for_header = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * nsps + rrc_len;
    if sample_offset + need_for_header > audio.len() {
        return Err(DecodeError::Truncated);
    }

    // Down-convert + matched-filter just enough audio for sync +
    // header. Payload extraction will re-MF the full span once we
    // know n_blocks. (MF cost is small vs LDPC cost.)
    let header_slice = &audio[sample_offset..sample_offset + need_for_header];
    let mf_out_header = downconvert_and_matched_filter(header_slice, audio_centre_hz, nsps);

    // Coherent integer-sample sync inside the ±NSPS jitter window
    // for the supplied mode's preamble.
    let preamble_bits = preamble_for(mode);
    let (best_off, best_mag2) = best_preamble_offset(&mf_out_header, preamble_bits, nsps);
    if best_mag2 <= 0.0 {
        return Err(DecodeError::HeaderFecFailed);
    }
    let frac_off =
        parabolic_subsample_refine(&mf_out_header, best_off, preamble_bits, best_mag2, nsps);

    // Train equaliser on preamble + decode header block.
    let header_syms_total = PREAMBLE_LEN + HEADER_BLOCK_SYMS;
    let symbols_header = sample_symbols(
        &mf_out_header,
        best_off,
        frac_off,
        header_syms_total + EQ_TAP_HALF,
        nsps,
    );
    let weights = train_ls_equaliser(&symbols_header, preamble_bits);
    let equalised_header = apply_equaliser(&symbols_header, &weights, header_syms_total);
    let r_diff_header = differential(&equalised_header);
    let stats = estimate_diff_stats(&r_diff_header[..PREAMBLE_LEN - 1], preamble_bits);
    let header_llrs = compute_llrs(
        &r_diff_header[PREAMBLE_LEN - 1..],
        &stats,
        HEADER_BLOCK_SYMS,
    );
    let header_codeword_llrs: Vec<f32> = header_llrs;

    let fec = Ldpc240_101;
    let header_decode = fec
        .decode_soft(&header_codeword_llrs, fec_opts)
        .ok_or(DecodeError::HeaderFecFailed)?;
    let header_info_bytes = bits_to_bytes_msb(&header_decode.info[..96]);

    // Phase 2: header decoded, parse it speculatively to discover
    // n_blocks. We can't verify CRC yet (CRC covers payload too).
    let (proto_n_blocks, _proto_app, _proto_seq) = peek_header_block_count(&header_info_bytes)?;

    // Now extract & decode the payload.
    let block_ch_bits = mode.ch_bits_per_block();
    let payload_syms = (proto_n_blocks as usize) * block_ch_bits / 2;
    let total_syms = PREAMBLE_LEN + HEADER_BLOCK_SYMS + payload_syms;
    let need_full = total_syms * nsps + rrc_len;
    if sample_offset + need_full > audio.len() {
        return Err(DecodeError::PayloadTruncated {
            needed_samples: need_full,
        });
    }
    let full_slice = &audio[sample_offset..sample_offset + need_full];
    let mf_out_full = downconvert_and_matched_filter(full_slice, audio_centre_hz, nsps);
    let symbols_full = sample_symbols(
        &mf_out_full,
        best_off,
        frac_off,
        total_syms + EQ_TAP_HALF,
        nsps,
    );
    let equalised_full = apply_equaliser(&symbols_full, &weights, total_syms);
    let r_diff_full = differential(&equalised_full);

    // Re-estimate amplitude/noise from the full-span preamble (same
    // weights, same preamble — values should match `stats`, but we
    // recompute to handle minor sample-position variation).
    let stats_full = estimate_diff_stats(&r_diff_full[..PREAMBLE_LEN - 1], preamble_bits);

    // Payload data starts at r_diff index `PREAMBLE_LEN + HEADER_BLOCK_SYMS - 1`.
    let payload_start = PREAMBLE_LEN + HEADER_BLOCK_SYMS - 1;
    let payload_llrs = compute_llrs(&r_diff_full[payload_start..], &stats_full, payload_syms);

    // De-interleave + de-puncture + decode each payload block.
    let n_blocks_u = proto_n_blocks as usize;
    let llr_per_block = deinterleave_llr(&payload_llrs, n_blocks_u);
    let mut decoded_info_bytes: Vec<u8> = Vec::with_capacity(n_blocks_u * INFO_BYTES_PER_BLOCK);
    for block_llrs in &llr_per_block {
        let full_llrs = de_puncture_llr(block_llrs, mode);
        let result = fec
            .decode_soft(&full_llrs, fec_opts)
            .ok_or(DecodeError::PayloadFecFailed)?;
        debug_assert_eq!(result.info.len(), K_LDPC);
        decoded_info_bytes.extend_from_slice(&bits_to_bytes_msb(&result.info[..96]));
    }

    // Verify the header CRC over (header_word ++ padded_payload).
    let mut all_bytes: Vec<u8> = Vec::with_capacity(HEADER_BYTES + decoded_info_bytes.len());
    all_bytes.extend_from_slice(&header_info_bytes[..HEADER_BYTES]);
    all_bytes.extend_from_slice(&decoded_info_bytes);
    let (frame_header, payload_slice) =
        unpack_header(&all_bytes, mode).map_err(DecodeError::Crc)?;
    let snr_db = compute_snr_2500hz(&stats_full, mode);
    Ok(DecodedFrame {
        app_type: frame_header.app_type,
        sequence: frame_header.sequence,
        mode,
        block_count: frame_header.block_count,
        payload: payload_slice.to_vec(),
        snr_db,
    })
}

/// Speculatively read the block_count field from the (unverified)
/// header info bytes. The CRC has not been verified yet — we use
/// this to decide how many payload symbols to extract; the actual
/// validation happens after payload decode via [`unpack_header`].
fn peek_header_block_count(header_info: &[u8]) -> Result<(u8, u8, u8), DecodeError> {
    if header_info.len() < HEADER_BYTES {
        return Err(DecodeError::HeaderFecFailed);
    }
    let header_word = u16::from_be_bytes([header_info[0], header_info[1]]);
    let blocks_code = ((header_word >> 11) & 0x1F) as u8;
    let app_type = ((header_word >> 7) & 0x0F) as u8;
    let sequence = ((header_word >> 2) & 0x1F) as u8;
    let reserved = (header_word & 0x3) as u8;
    if reserved != 0 {
        return Err(DecodeError::HeaderFecFailed);
    }
    Ok((blocks_code + 1, app_type, sequence))
}

// ────────────────────────────────────────────────────────────────────
// Internal — auto-detect decode pipeline (single channel)
// ────────────────────────────────────────────────────────────────────

const MAX_PICKED_PEAKS: usize = 3;

/// Collect every `(Mode, preamble bits)` pair whose nsps matches the
/// argument. Returns `(modes, preambles, count)`. Up to 4 entries —
/// matches `NUM_PREAMBLES`. Lets the auto-detect call sites batch
/// per-offset preamble correlations through
/// `preamble_differential_scores_multi`.
fn preambles_for_nsps(
    nsps: usize,
) -> (
    [Option<Mode>; NUM_PREAMBLES],
    [&'static [bool]; NUM_PREAMBLES],
    usize,
) {
    let mut modes: [Option<Mode>; NUM_PREAMBLES] = [None; NUM_PREAMBLES];
    let mut bits: [&'static [bool]; NUM_PREAMBLES] = [&[]; NUM_PREAMBLES];
    let mut count = 0;
    for i in 0..NUM_PREAMBLES {
        if let Some(m) = mode_for_index(i)
            && m.nsps() == nsps
        {
            modes[count] = Some(m);
            bits[count] = preamble_for(m);
            count += 1;
        }
    }
    (modes, bits, count)
}

/// Sync gate threshold: a candidate peak passes when
/// `score >= median × SYNC_GATE_RATIO`. With the new differential
/// score across 4 preamble variants, pure-noise extreme-value max
/// vs median ratio sits around 17-22 (empirically measured on
/// idle-mic snapshots in the deployed PWA, +0.04 amplitude RMS).
/// 30 cleanly clears that floor; real weak signals at +5-6 dB
/// Eb/N0 Robust reach 50+. Was 18 in the first iteration of the
/// 0.4.0 design, which let noise trigger the LDPC decode pipeline
/// — see `tests/uvpacket_pi4_dqpsk_eq.rs` for the threshold
/// characterisation.
const SYNC_GATE_RATIO: f32 = 30.0;

fn decode_inner(audio: &[f32], audio_centre_hz: f32, fec_opts: &FecOpts) -> Vec<DecodedFrame> {
    let min_needed = PREAMBLE_LEN * NSPS_ULTRA + rrc_len_for(NSPS_ULTRA);
    if audio.len() < min_needed {
        return Vec::new();
    }

    // Score every offset against every preamble variant, with the
    // matching MF per nsps. Keep peaks in the audio-domain offset
    // (mf_offset − sym_peak_offset_for(nsps)) so the cross-mode NMS
    // compares like with like.
    let mut peaks: Vec<(usize, Mode, f32)> = Vec::new();
    let mut all_scores: Vec<f32> = Vec::new();
    for &nsps in ALL_NSPS.iter() {
        let mf_out = downconvert_and_matched_filter(audio, audio_centre_hz, nsps);
        let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * nsps + 1);
        if max_corr_offset == 0 {
            continue;
        }
        let peak_off = sym_peak_offset_for(nsps);
        let (nsps_modes, nsps_bits, np) = preambles_for_nsps(nsps);
        if np == 0 {
            continue;
        }
        for mf_offset in 0..max_corr_offset {
            let scores =
                preamble_differential_scores_multi(&mf_out, mf_offset, &nsps_bits[..np], nsps);
            for j in 0..np {
                let score = scores[j];
                let m = nsps_modes[j].expect("populated by preambles_for_nsps");
                all_scores.push(score);
                if let Some(audio_off) = mf_offset.checked_sub(peak_off) {
                    peaks.push((audio_off, m, score));
                }
            }
        }
    }
    let median = robust_median(&all_scores);
    if median <= 0.0 {
        return Vec::new();
    }

    let threshold = median * SYNC_GATE_RATIO;
    peaks.retain(|&(_, _, s)| s >= threshold);
    peaks.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());

    // NMS in audio-domain offset (within ±NSPS_BASE samples — half a
    // canonical symbol period; UltraRobust's longer chip span is
    // automatically covered since its peaks land at integer-multiples
    // of 20 samples).
    let mut picked: Vec<(usize, Mode)> = Vec::new();
    for (audio_off, mode, _score) in peaks {
        if picked
            .iter()
            .all(|&(po, _)| audio_off.abs_diff(po) > NSPS_BASE)
        {
            picked.push((audio_off, mode));
            if picked.len() >= MAX_PICKED_PEAKS {
                break;
            }
        }
    }
    picked.sort_unstable_by_key(|&(o, _)| o);

    let afc_opts = AfcOpts::default();
    let mut frames: Vec<DecodedFrame> = Vec::new();
    let mut consumed_until: usize = 0;
    for (audio_off, mode) in picked {
        if audio_off < consumed_until {
            continue;
        }
        let nsps = mode.nsps();
        let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * nsps + rrc_len_for(nsps);
        if audio_off + needed > audio.len() {
            continue;
        }
        let delta_hz = estimate_freq_offset_for_mode(
            audio,
            audio_off,
            needed,
            audio_centre_hz,
            mode,
            &afc_opts,
        );
        if let Ok(frame) =
            decode_at_inner(audio, audio_off, audio_centre_hz + delta_hz, mode, fec_opts)
        {
            consumed_until = audio_off
                + (PREAMBLE_LEN
                    + HEADER_BLOCK_SYMS
                    + (frame.block_count as usize) * mode.ch_bits_per_block() / 2)
                    * nsps;
            frames.push(frame);
        }
    }
    frames
}

fn decode_multichannel_inner(
    audio: &[f32],
    mc_opts: &MultiChannelOpts,
    fec_opts: &FecOpts,
) -> Vec<(f32, DecodedFrame)> {
    // For each centre, find the strongest peak across both nsps
    // values (NSPS_BASE for Robust/Standard/Express, NSPS_ULTRA for
    // UltraRobust). Peak audio_offset = mf_offset − sym_peak_offset.
    let mut peaks: Vec<(f32, usize, Mode, f32)> = Vec::new();
    let mut all_scores: Vec<f32> = Vec::new();
    let mut centre = mc_opts.band_lo_hz;
    while centre <= mc_opts.band_hi_hz {
        let mut best_audio_off = 0usize;
        let mut best_mode = Mode::Robust;
        let mut best_score = 0.0_f32;
        for &nsps in ALL_NSPS.iter() {
            let mf_out = downconvert_and_matched_filter(audio, centre, nsps);
            let max_corr_offset = mf_out.len().saturating_sub((PREAMBLE_LEN - 1) * nsps + 1);
            if max_corr_offset == 0 {
                continue;
            }
            let peak_off = sym_peak_offset_for(nsps);
            let (nsps_modes, nsps_bits, np) = preambles_for_nsps(nsps);
            if np == 0 {
                continue;
            }
            for mf_offset in 0..max_corr_offset {
                let scores =
                    preamble_differential_scores_multi(&mf_out, mf_offset, &nsps_bits[..np], nsps);
                for j in 0..np {
                    let s = scores[j];
                    all_scores.push(s);
                    if s > best_score
                        && let Some(audio_off) = mf_offset.checked_sub(peak_off)
                    {
                        best_score = s;
                        best_audio_off = audio_off;
                        best_mode = nsps_modes[j].expect("populated by preambles_for_nsps");
                    }
                }
            }
        }
        if best_score > 0.0 {
            peaks.push((centre, best_audio_off, best_mode, best_score));
        }
        centre += mc_opts.coarse_step_hz;
    }
    if peaks.is_empty() {
        return Vec::new();
    }

    // Band-wide sync gate.
    let median = robust_median(&all_scores);
    if median <= 0.0 {
        return Vec::new();
    }
    let threshold = median * SYNC_GATE_RATIO;
    peaks.retain(|&(_, _, _, s)| s >= threshold);
    if peaks.is_empty() {
        return Vec::new();
    }

    // NMS in frequency.
    peaks.sort_by(|a, b| b.3.partial_cmp(&a.3).unwrap());
    let mut kept: Vec<(f32, usize, Mode)> = Vec::new();
    for (f, off, mode, _score) in peaks {
        if kept
            .iter()
            .all(|&(kf, _, _)| (f - kf).abs() > mc_opts.nms_radius_hz)
        {
            kept.push((f, off, mode));
        }
    }

    // Per-peak decode.
    let afc_opts = AfcOpts::default();
    let mut out: Vec<(f32, DecodedFrame)> = Vec::new();
    for (centre, audio_off, mode) in kept {
        let nsps = mode.nsps();
        let needed = (PREAMBLE_LEN + HEADER_BLOCK_SYMS) * nsps + rrc_len_for(nsps);
        if audio_off + needed > audio.len() {
            continue;
        }
        let delta_hz =
            estimate_freq_offset_for_mode(audio, audio_off, needed, centre, mode, &afc_opts);
        if let Ok(frame) = decode_at_inner(audio, audio_off, centre + delta_hz, mode, fec_opts) {
            out.push((centre + delta_hz, frame));
        }
    }
    out
}

// ────────────────────────────────────────────────────────────────────
// Internal — DSP helpers (matched filter, sync, equaliser, demod)
// ────────────────────────────────────────────────────────────────────

fn downconvert_and_matched_filter(
    audio: &[f32],
    audio_centre_hz: f32,
    nsps: usize,
) -> Vec<Complex32> {
    let two_pi_fc_dt = 2.0 * PI * audio_centre_hz / SAMPLE_RATE_HZ;
    let mut bb: Vec<Complex32> = Vec::with_capacity(audio.len());
    for (n, &s) in audio.iter().enumerate() {
        let phase = two_pi_fc_dt * n as f32;
        let (sin, cos) = phase.sin_cos();
        bb.push(Complex32::new(2.0 * s * cos, -2.0 * s * sin));
    }
    let rrc = rrc_pulse(RRC_ALPHA, RRC_SPAN_SYMS, nsps);
    let n_out = bb.len() + rrc.len() - 1;
    let mut out = vec![Complex32::new(0.0, 0.0); n_out];
    for (i, &x) in bb.iter().enumerate() {
        for (j, &h) in rrc.iter().enumerate() {
            out[i + j] += x * h;
        }
    }
    out
}

fn preamble_correlation(
    mf_out: &[Complex32],
    offset: usize,
    bits: &[bool],
    nsps: usize,
) -> Complex32 {
    let mut acc = Complex32::new(0.0, 0.0);
    for (i, &b) in bits.iter().enumerate() {
        let pos = offset + i * nsps;
        let s = if b { -1.0_f32 } else { 1.0 };
        acc += mf_out[pos] * s;
    }
    acc
}

/// Differential preamble correlator (multi-preamble shared form).
///
/// Score formula per preamble: `|Σ aᵢ · cᵢ|² / Σ |aᵢ|²` where
/// `aᵢ = mf[k]·conj(mf[k-1])` and `cᵢ = bᵢ·bᵢ₋₁ ∈ ±1`. Phase-rotation
/// invariant (cancels in the differential product) — survives clarifier
/// offset / LO walk much better than the coherent correlator.
///
/// The differential pair products `aᵢ` and the energy `Σ|aᵢ|²` are
/// preamble-independent, so the auto-detect call sites that score
/// every offset against multiple preambles at the same nsps share
/// them via this function. For K=3 preambles (the NSPS_BASE case)
/// this saves ~36 % per offset vs running an independent scalar
/// score per preamble. K=1 (NSPS_ULTRA) is slightly slower than the
/// scalar form would be, but the auto-detect path always batches
/// per-nsps so the K=1 overhead is irrelevant in practice.
///
/// All preambles must share the same length. Up to `NUM_PREAMBLES`
/// per call.
fn preamble_differential_scores_multi(
    mf_out: &[Complex32],
    offset: usize,
    preambles: &[&[bool]],
    nsps: usize,
) -> [f32; NUM_PREAMBLES] {
    let mut out = [0.0_f32; NUM_PREAMBLES];
    let np = preambles.len().min(NUM_PREAMBLES);
    if np == 0 {
        return out;
    }
    let n = preambles[0].len();
    if n < 2 {
        return out;
    }
    let last_pos = offset + (n - 1) * nsps;
    if last_pos >= mf_out.len() {
        return out;
    }
    let mut accs = [Complex32::new(0.0, 0.0); NUM_PREAMBLES];
    let mut prev_signs = [0.0_f32; NUM_PREAMBLES];
    for j in 0..np {
        prev_signs[j] = if preambles[j][0] { -1.0 } else { 1.0 };
    }
    let mut energy = 0.0_f32;
    let mut prev_sample = mf_out[offset];
    for i in 1..n {
        let pos = offset + i * nsps;
        let s = mf_out[pos];
        let a = s * prev_sample.conj();
        energy += a.norm_sqr();
        for j in 0..np {
            let cur_sign = if preambles[j][i] { -1.0_f32 } else { 1.0 };
            let c = cur_sign * prev_signs[j];
            accs[j] += a * c;
            prev_signs[j] = cur_sign;
        }
        prev_sample = s;
    }
    if energy > 0.0 {
        for j in 0..np {
            out[j] = accs[j].norm_sqr() / energy;
        }
    }
    out
}

fn best_preamble_offset(mf_out: &[Complex32], bits: &[bool], nsps: usize) -> (usize, f32) {
    let radius = nsps as isize;
    let base = sym_peak_offset_for(nsps) as isize;
    let n = bits.len();
    let mut best_off = sym_peak_offset_for(nsps);
    let mut best_mag2 = -1.0_f32;
    for jitter in -radius..=radius {
        let off = base + jitter;
        if off < 0 {
            continue;
        }
        let off = off as usize;
        if off + (n - 1) * nsps >= mf_out.len() {
            continue;
        }
        let mag2 = preamble_correlation(mf_out, off, bits, nsps).norm_sqr();
        if mag2 > best_mag2 {
            best_mag2 = mag2;
            best_off = off;
        }
    }
    (best_off, best_mag2)
}

fn parabolic_subsample_refine(
    mf_out: &[Complex32],
    best_off: usize,
    bits: &[bool],
    best_mag2: f32,
    nsps: usize,
) -> f32 {
    let n = bits.len();
    let need_minus = best_off > 0 && (best_off - 1) + (n - 1) * nsps < mf_out.len();
    let need_plus = (best_off + 1) + (n - 1) * nsps < mf_out.len();
    if !(need_minus && need_plus) {
        return 0.0;
    }
    let m_minus = preamble_correlation(mf_out, best_off - 1, bits, nsps).norm_sqr();
    let m_plus = preamble_correlation(mf_out, best_off + 1, bits, nsps).norm_sqr();
    let denom = 2.0 * (m_plus - 2.0 * best_mag2 + m_minus);
    if denom.abs() > 1e-9 {
        ((m_minus - m_plus) / denom).clamp(-0.5, 0.5)
    } else {
        0.0
    }
}

/// Sample `n_syms` symbols at NSPS spacing starting from
/// `best_off + frac_off`. Returns zero-padded values for any
/// out-of-range position (caller sees them as the equaliser's
/// edge contribution).
fn sample_symbols(
    mf_out: &[Complex32],
    best_off: usize,
    frac_off: f32,
    n_syms: usize,
    nsps: usize,
) -> Vec<Complex32> {
    let mut out: Vec<Complex32> = Vec::with_capacity(n_syms);
    for i in 0..n_syms {
        let pos = best_off as f32 + frac_off + (i * nsps) as f32;
        if pos < 0.0 || pos as usize + 1 >= mf_out.len() {
            out.push(Complex32::new(0.0, 0.0));
        } else {
            out.push(sample_mf_lerp(mf_out, pos));
        }
    }
    out
}

fn sample_mf_lerp(mf_out: &[Complex32], pos: f32) -> Complex32 {
    let p_int = pos.floor() as usize;
    let alpha = pos - p_int as f32;
    mf_out[p_int] * (1.0 - alpha) + mf_out[p_int + 1] * alpha
}

/// Train a 9-tap T-spaced linear equaliser via least-squares on the
/// preamble. Returns the tap weights `w` such that
/// `e[k] = Σ_n w[n] · symbols[k + n − TAP_HALF]` is optimised for
/// MMSE against the BPSK reference `±1`.
fn train_ls_equaliser(symbols: &[Complex32], preamble_bits: &[bool]) -> Vec<Complex32> {
    let n_pre = preamble_bits.len();
    let mut r_mat: Vec<Vec<Complex32>> = vec![vec![Complex32::new(0.0, 0.0); EQ_N_TAPS]; EQ_N_TAPS];
    let mut p_vec: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); EQ_N_TAPS];
    for k in EQ_TAP_HALF..n_pre.saturating_sub(EQ_TAP_HALF) {
        let d_re: f32 = if preamble_bits[k] { -1.0 } else { 1.0 };
        let d = Complex32::new(d_re, 0.0);
        let mut x = [Complex32::new(0.0, 0.0); EQ_N_TAPS];
        for n in 0..EQ_N_TAPS {
            x[n] = symbols[k + n - EQ_TAP_HALF];
        }
        for i in 0..EQ_N_TAPS {
            for j in 0..EQ_N_TAPS {
                r_mat[i][j] += x[i].conj() * x[j];
            }
            p_vec[i] += x[i].conj() * d;
        }
    }
    // Diagonal loading (small fraction of trace for stability).
    let trace: f32 = (0..EQ_N_TAPS).map(|i| r_mat[i][i].re).sum();
    let load = EQ_LS_DIAG_LOAD * trace / EQ_N_TAPS as f32;
    for i in 0..EQ_N_TAPS {
        r_mat[i][i] += Complex32::new(load, 0.0);
    }

    // Solve via Gauss-Jordan on the augmented matrix.
    let mut aug: Vec<Vec<Complex32>> = (0..EQ_N_TAPS)
        .map(|i| {
            let mut row = r_mat[i].clone();
            row.push(p_vec[i]);
            row
        })
        .collect();
    for i in 0..EQ_N_TAPS {
        let mut piv = i;
        let mut piv_mag = aug[i][i].norm();
        for r in (i + 1)..EQ_N_TAPS {
            let m = aug[r][i].norm();
            if m > piv_mag {
                piv_mag = m;
                piv = r;
            }
        }
        if piv != i {
            aug.swap(i, piv);
        }
        if aug[i][i].norm() < 1e-12 {
            return identity_equaliser();
        }
        let pivot = aug[i][i];
        for j in i..=EQ_N_TAPS {
            aug[i][j] /= pivot;
        }
        for r in 0..EQ_N_TAPS {
            if r == i {
                continue;
            }
            let factor = aug[r][i];
            for j in i..=EQ_N_TAPS {
                let sub = factor * aug[i][j];
                aug[r][j] -= sub;
            }
        }
    }
    (0..EQ_N_TAPS).map(|i| aug[i][EQ_N_TAPS]).collect()
}

fn identity_equaliser() -> Vec<Complex32> {
    let mut w = vec![Complex32::new(0.0, 0.0); EQ_N_TAPS];
    w[EQ_TAP_HALF] = Complex32::new(1.0, 0.0);
    w
}

fn apply_equaliser(symbols: &[Complex32], weights: &[Complex32], n_out: usize) -> Vec<Complex32> {
    let mut out: Vec<Complex32> = Vec::with_capacity(n_out);
    for k in 0..n_out {
        if k < EQ_TAP_HALF || k + EQ_TAP_HALF >= symbols.len() {
            out.push(Complex32::new(0.0, 0.0));
        } else {
            let mut e = Complex32::new(0.0, 0.0);
            for n in 0..EQ_N_TAPS {
                e += weights[n] * symbols[k + n - EQ_TAP_HALF];
            }
            out.push(e);
        }
    }
    out
}

fn differential(equalised: &[Complex32]) -> Vec<Complex32> {
    let mut out: Vec<Complex32> = Vec::with_capacity(equalised.len().saturating_sub(1));
    for k in 1..equalised.len() {
        out.push(equalised[k] * equalised[k - 1].conj());
    }
    out
}

struct DiffStats {
    a_sq_est: f32,
    sigma_sq_n_diff: f32,
    residual_rotation: f32,
}

fn estimate_diff_stats(r_diff_preamble: &[Complex32], preamble_bits: &[bool]) -> DiffStats {
    let n = r_diff_preamble.len();
    let mut signed_acc = Complex32::new(0.0, 0.0);
    for k in 0..n {
        let bk: f32 = if preamble_bits[k + 1] { -1.0 } else { 1.0 };
        let bkm1: f32 = if preamble_bits[k] { -1.0 } else { 1.0 };
        signed_acc += r_diff_preamble[k] * (bk * bkm1);
    }
    let signed_mean = signed_acc / (n.max(1) as f32);
    let a_sq_est = signed_mean.norm().max(1e-6);
    let residual_rotation = signed_mean.arg();

    let derot_resid = Complex32::from_polar(1.0, -residual_rotation);
    let mut noise_im_sq = 0.0_f32;
    for k in 0..n {
        let bk: f32 = if preamble_bits[k + 1] { -1.0 } else { 1.0 };
        let bkm1: f32 = if preamble_bits[k] { -1.0 } else { 1.0 };
        let aligned = r_diff_preamble[k] * (bk * bkm1) * derot_resid;
        noise_im_sq += aligned.im * aligned.im;
    }
    let sigma_sq_n_diff = (noise_im_sq / n.max(1) as f32).max(1e-6);
    DiffStats {
        a_sq_est,
        sigma_sq_n_diff,
        residual_rotation,
    }
}

/// Per-mode calibration constant (dB) absorbing the differential-demod
/// noise enhancement (`n·n*` term still contributes near threshold)
/// plus residual MF / RRC scaling not captured in `2·a²/σ²`. Calibrated
/// against AWGN truth via `tests/uvpacket_snr_calibration.rs`: 20
/// trials per `(mode, Eb/N0)` cell over `Eb/N0 ∈ {6 .. 22} dB`,
/// residual `(reported − truth_2500)` averaged within ±0.2 dB across
/// the sweep range. Indexed by `Mode::header_code()`
/// — `[UltraRobust, Robust, Standard, Express]`.
const SNR_CALIBRATION_DB: [f32; 4] = [-6.2, -3.0, -3.0, -3.4];

/// Compute WSJT-X-compatible SNR (dB, 2.5 kHz reference bandwidth)
/// from the joint amplitude / noise estimator on the long preamble.
///
/// Approach (single-carrier π/4-DQPSK has no "opposite tone" the way
/// FSK does, so we cannot reuse the FT8 / Q65 noise-tone estimator):
///
/// 1. Symbol SNR ≈ `2·a²/σ²` from the differential-pair statistics.
///    Exact at `Eb/N0 ≥ ~+5 dB`; slight optimistic bias at threshold
///    where the `n·n*` term in `σ²_diff` becomes a meaningful fraction
///    of the `2A²σ²` signal-cross-noise term.
/// 2. Convert MF-output SNR to 2.5 kHz reference using the mode's
///    symbol rate as the matched-filter equivalent noise bandwidth:
///    `+10·log10(baud / 2500)`.
/// 3. Subtract the empirical per-mode calibration (above) so the
///    reported SNR matches the WSJT-X convention at known Eb/N0.
///
/// Floor at −30 dB (matches WSJT-X's clamping convention).
fn compute_snr_2500hz(stats: &DiffStats, mode: Mode) -> f32 {
    let a = stats.a_sq_est.max(1e-9);
    let sigma_sq = stats.sigma_sq_n_diff.max(1e-9);
    let symbol_snr = 2.0 * a * a / sigma_sq;
    if symbol_snr <= 1e-9 {
        return -30.0;
    }
    let baud = SAMPLE_RATE_HZ / mode.nsps() as f32;
    let bw_correction = 10.0 * (baud / 2500.0).log10();
    let cal = SNR_CALIBRATION_DB[mode.header_code() as usize];
    let snr_db = 10.0 * symbol_snr.log10() + bw_correction + cal;
    snr_db.max(-30.0)
}

fn compute_llrs(r_diff: &[Complex32], stats: &DiffStats, n_data: usize) -> Vec<f32> {
    let llr_scale = stats.a_sq_est / stats.sigma_sq_n_diff;
    let combined_rotate = Complex32::from_polar(1.0, -stats.residual_rotation - PI / 4.0);
    let mut out: Vec<f32> = Vec::with_capacity(n_data * 2);
    for k in 0..n_data {
        let derot = r_diff[k] * combined_rotate;
        let (llr_b1, llr_b0) = qpsk_llrs(derot);
        out.push(llr_b1 * llr_scale);
        out.push(llr_b0 * llr_scale);
    }
    out
}

/// Soft-output QPSK demap to LLR (max-log). Returns
/// `(LLR(b1), LLR(b0))` with `LLR > 0 ⇔ bit = 1`. Matches the TX
/// Gray map [0, 1, 3, 2].
fn qpsk_llrs(r: Complex32) -> (f32, f32) {
    let re = r.re;
    let im = r.im;
    let llr_b1 = -(re + im);
    let llr_b0 = im.max(-re) - re.max(-im);
    (llr_b1, llr_b0)
}

fn bits_to_bytes_msb(bits: &[u8]) -> Vec<u8> {
    let n_bytes = bits.len() / 8;
    let mut out = vec![0u8; n_bytes];
    for byte_idx in 0..n_bytes {
        let mut byte = 0u8;
        for bit_idx in 0..8 {
            if bits[byte_idx * 8 + bit_idx] != 0 {
                byte |= 1 << (7 - bit_idx);
            }
        }
        out[byte_idx] = byte;
    }
    out
}

fn robust_median(scores: &[f32]) -> f32 {
    let mut nz: Vec<f32> = scores.iter().copied().filter(|&s| s > 0.0).collect();
    if nz.is_empty() {
        return 0.0;
    }
    let mid = nz.len() / 2;
    nz.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap());
    nz[mid]
}

/// AFC for a known mode: try a coarse grid of frequency offsets,
/// pick the one whose preamble correlation peaks highest, refine
/// by parabolic fit.
fn estimate_freq_offset_for_mode(
    audio: &[f32],
    sample_offset: usize,
    needed_samples: usize,
    audio_centre_hz: f32,
    mode: Mode,
    afc_opts: &AfcOpts,
) -> f32 {
    let coarse_step_hz: f32 = 25.0;
    let n_coarse = (afc_opts.search_hz / coarse_step_hz).ceil() as i32;
    let slice = &audio[sample_offset..sample_offset + needed_samples];
    let preamble_bits = preamble_for(mode);
    let nsps = mode.nsps();

    let mut grid_mags: Vec<(i32, f32)> = Vec::with_capacity(2 * n_coarse as usize + 1);
    for k in -n_coarse..=n_coarse {
        let f_test = audio_centre_hz + k as f32 * coarse_step_hz;
        let mf_out = downconvert_and_matched_filter(slice, f_test, nsps);
        let (_, mag2) = best_preamble_offset(&mf_out, preamble_bits, nsps);
        grid_mags.push((k, mag2));
    }
    let (best_k_idx, _) = grid_mags
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.1.partial_cmp(&b.1.1).unwrap())
        .map(|(idx, &(_, m))| (idx, m))
        .unwrap_or((0, 0.0));
    let best_k = grid_mags[best_k_idx].0;
    let frac = if best_k_idx > 0 && best_k_idx + 1 < grid_mags.len() {
        let m_minus = grid_mags[best_k_idx - 1].1;
        let m_zero = grid_mags[best_k_idx].1;
        let m_plus = grid_mags[best_k_idx + 1].1;
        let denom = 2.0 * (m_plus - 2.0 * m_zero + m_minus);
        if denom.abs() > 1e-9 {
            ((m_minus - m_plus) / denom).clamp(-0.5, 0.5)
        } else {
            0.0
        }
    } else {
        0.0
    };
    (best_k as f32 + frac) * coarse_step_hz
}
