// SPDX-License-Identifier: GPL-3.0-or-later
//! uvpacket receive path: 12 kHz PCM → coarse sync → per-candidate
//! downsample / LLR / deinterleave / LDPC decode → byte payload.
//!
//! Thin wrapper over [`crate::core::pipeline::decode_frame`]. The
//! pipeline already handles the four-LLR sweep, OSD fallback, dedup
//! and SNR estimation; uvpacket only owns the per-sub-mode
//! [`DownsampleCfg`] and the post-decode `unpack` to typed bytes.
//!
//! ## Sliding-window vs slotted
//!
//! Unlike FT8 / FT4 / FST4 (UTC-slotted) or Q65 (T/R period), uvpacket
//! has no UTC alignment — frames can land anywhere in the audio buffer.
//! [`crate::core::sync::coarse_sync`] (called inside
//! [`pipeline::decode_frame`]) already does a sliding-window correlation
//! across the full input, so passing one second of PCM lets it pick up
//! every frame whose first or second Costas block falls inside.
//!
//! ## Default audio length
//!
//! [`DEFAULT_AUDIO_SAMPLES`] = 12 000 samples (1 s at 12 kHz). All four
//! sub-modes' frames fit comfortably (UvPacket150 at 633 ms is the
//! longest). For longer buffers, build a custom [`DownsampleCfg`] via
//! [`downsample_cfg_for`] sized to your audio length.

use crate::core::dsp::downsample::DownsampleCfg;
use crate::core::equalize::EqMode;
use crate::core::pipeline::{self, DecodeDepth, DecodeStrictness};
use crate::core::{DecodeContext, MessageCodec, ModulationParams, Protocol};
use crate::msg::PacketBytesMessage;

/// Default audio buffer length the const [`DownsampleCfg`] tables size
/// for: one second of 12 kHz PCM.
pub const DEFAULT_AUDIO_SAMPLES: usize = 12_000;

/// Coarse-sync time-step refine range in downsampled samples. The two
/// Costas blocks at symbols 0 and 47 give independent timing votes;
/// ±1 symbol of refinement absorbs the integer-rounding error.
const REFINE_STEPS: i32 = 16;

/// Sync-quality gate: require at least 4 of the 8 sync symbols to look
/// right. Mid-frame Costas survives a head-end fade and vice versa, so
/// we don't need every sync symbol intact — half is enough for the
/// LDPC + interleaver to take it from there.
const SYNC_Q_MIN: u32 = 4;

/// One successfully decoded uvpacket frame.
#[derive(Debug, Clone)]
pub struct DecodedPacket {
    /// Decoded byte payload (1..=10 bytes).
    pub payload: Vec<u8>,
    /// Carrier centre frequency in Hz.
    pub freq_hz: f32,
    /// Time offset from the start of the audio buffer to the frame's
    /// first symbol, in seconds. Useful for de-duplicating frames that
    /// span multiple decode windows.
    pub dt_sec: f32,
    /// WSJT-X compatible SNR estimate in dB.
    pub snr_db: f32,
    /// Number of hard-decision LDPC errors corrected.
    pub hard_errors: u32,
    /// Pipeline pass tag (0..3 = BP variant a..d, 4..5 = OSD, 13 = OSD-4).
    pub pass: u8,
}

/// Build a [`DownsampleCfg`] sized for `audio_samples` 12 kHz PCM.
///
/// Picks `fft1_size` as the smallest power of two ≥ `audio_samples`
/// (good FFT performance, easy to reason about). Applies the sub-mode's
/// `NDOWN` to derive `fft2_size`. Pad and edge-taper match the FT4 /
/// FST4 conventions (1.5 tones each side, 101-bin Hann taper).
///
/// For `audio_samples == DEFAULT_AUDIO_SAMPLES` (12 000), `fft1_size`
/// = 16 384 — the same value the const table uses.
pub fn downsample_cfg_for<P: ModulationParams>(audio_samples: usize) -> DownsampleCfg {
    let fft1 = audio_samples.next_power_of_two().max(16_384);
    let ndown = P::NDOWN.max(1) as usize;
    DownsampleCfg {
        input_rate: 12_000,
        fft1_size: fft1,
        fft2_size: fft1 / ndown,
        tone_spacing_hz: P::TONE_SPACING_HZ,
        leading_pad_tones: 1.5,
        trailing_pad_tones: 1.5,
        ntones: P::NTONES,
        edge_taper_bins: 101,
    }
}

/// Decode every uvpacket frame found in `audio` (12 kHz i16 PCM).
///
/// `freq_min`/`freq_max` bound the sliding-window coarse search in Hz.
/// `sync_min` is the minimum Costas correlation score (try `0.5` for
/// Rayleigh-fading channels, `1.0`+ for clean ones). `max_cand` caps
/// the number of candidate frame positions tried per pass; 50 is a
/// reasonable upper bound for a 1 second buffer.
///
/// Returns one entry per successfully decoded frame, deduplicated by
/// info-bit content (so the same frame located by both head and
/// mid-frame sync only appears once).
pub fn decode_frame<P: Protocol>(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<DecodedPacket> {
    let cfg = downsample_cfg_for::<P>(audio.len());
    let (raw, _fft_cache) = pipeline::decode_frame::<P>(
        audio,
        &cfg,
        freq_min,
        freq_max,
        sync_min,
        /* freq_hint */ None,
        DecodeDepth::BpAllOsd,
        max_cand,
        DecodeStrictness::Normal,
        EqMode::Off,
        REFINE_STEPS,
        SYNC_Q_MIN,
    );

    let codec = PacketBytesMessage;
    let ctx = DecodeContext::default();
    raw.into_iter()
        .filter_map(|r| {
            // PacketBytesMessage::unpack runs verify_info (CRC-7) and
            // returns None on integrity-check failure — already
            // applied inside the FEC layer too, but we're defensive
            // here in case the pipeline path skipped the verifier
            // (e.g. an OSD candidate that the BP rejected).
            let payload = codec.unpack(&r.info, &ctx)?;
            Some(DecodedPacket {
                payload,
                freq_hz: r.freq_hz,
                dt_sec: r.dt_sec,
                snr_db: r.snr_db,
                hard_errors: r.hard_errors,
                pass: r.pass,
            })
        })
        .collect()
}
