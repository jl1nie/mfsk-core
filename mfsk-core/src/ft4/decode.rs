//! FT4 decode — thin wrapper over [`crate::core::pipeline`].
//!
//! Exposes `decode_frame` and `decode_frame_subtract` which drive the full
//! generic pipeline (coarse sync → refine → LLR → BP/OSD → optional SIC
//! multi-pass) specialised to the [`Ft4`] protocol. AP hints and sniper
//! single-frequency entry points are not provided here — they can be added
//! once the generic pipeline grows AP support.

use super::Ft4;
use crate::core::dsp::downsample::DownsampleCfg;
use crate::core::dsp::subtract::SubtractCfg;
use crate::core::equalize::EqMode;
use crate::core::pipeline::{self, FftCache};
use crate::msg::pipeline_ap;

pub use crate::core::pipeline::{DecodeDepth, DecodeResult, DecodeStrictness};
pub use crate::msg::ApHint;

/// FT4 downsample configuration: 12 kHz → ~666.7 Hz baseband, covering four
/// tones spaced 20.833 Hz apart plus headroom.
///
/// `fft1_size` is chosen as 92 160 = 2^12 · 3² · 5 (highly-composite, ≥ slot
/// audio length 7.5 s × 12 kHz = 90 000). `fft2_size` = fft1 / NDOWN = 5120
/// to yield the 666.7 Hz output rate.
pub const FT4_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 92_160,
    fft2_size: 5_120,
    tone_spacing_hz: 20.833,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FT4 subtract configuration: 48 ms symbols, frame origin at 0.5 s.
pub const FT4_SUBTRACT: SubtractCfg = SubtractCfg {
    sample_rate: 12_000.0,
    tone_spacing_hz: 20.833,
    samples_per_symbol: 576,
    base_offset_s: 0.5,
};

/// FT4's coarse sync now uses half-symbol (24 ms = 16 downsampled-sample)
/// steps; refine across ±1 symbol (32 samples) still to bridge rounding.
const REFINE_STEPS: i32 = 32;
/// FT4 has 16 sync symbols (4 × 4); require at least half correct.
const SYNC_Q_MIN: u32 = 8;

/// Decode one FT4 slot of 12 kHz PCM audio.
pub fn decode_frame(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<DecodeResult> {
    pipeline::decode_frame::<Ft4>(
        audio,
        &FT4_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
        DecodeStrictness::Normal,
        EqMode::Off,
        REFINE_STEPS,
        SYNC_Q_MIN,
    )
    .0
}

/// Decode one FT4 slot returning the FFT cache for pipelined subtraction.
pub fn decode_frame_with_cache(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    pipeline::decode_frame::<Ft4>(
        audio,
        &FT4_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
        DecodeStrictness::Normal,
        EqMode::Off,
        REFINE_STEPS,
        SYNC_Q_MIN,
    )
}

/// Multi-pass decode with successive interference cancellation.
pub fn decode_frame_subtract(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<DecodeResult> {
    pipeline::decode_frame_subtract::<Ft4>(
        audio,
        &FT4_DOWNSAMPLE,
        &FT4_SUBTRACT,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
        DecodeStrictness::Normal,
        REFINE_STEPS,
        SYNC_Q_MIN,
    )
}

/// Sniper-mode decode with optional AP hints — searches ±250 Hz of
/// `target_freq` and, if `ap_hint` is supplied, clamps the known parts of
/// the expected message to high-confidence LLRs before BP/OSD.
pub fn decode_sniper_ap(
    audio: &[i16],
    target_freq: f32,
    max_cand: usize,
    eq_mode: EqMode,
    ap_hint: Option<&ApHint>,
) -> Vec<DecodeResult> {
    // Clamp caller's candidate count: in sniper mode the target is
    // reliably in the top-5 after dedup even at -18 dB, so >15 just
    // burns CPU — especially important under the lite feature defaults
    // where every candidate runs BP + OSD per AP config.
    let max_cand = max_cand.min(15);
    pipeline_ap::decode_sniper_ap::<Ft4>(
        audio,
        &FT4_DOWNSAMPLE,
        target_freq,
        250.0,
        // Looser sync_min under sniper+AP: when AP locks ≥55 bits the FEC
        // can recover signals whose coarse-sync score wouldn't qualify for
        // a bare decode — we still need candidates to attempt the lock on.
        0.5,
        DecodeDepth::BpAllOsd,
        max_cand,
        DecodeStrictness::Normal,
        eq_mode,
        REFINE_STEPS,
        // Halve the sync-quality gate for AP: locked bits carry the
        // decision, so weak sync-quality signals may still succeed.
        SYNC_Q_MIN / 2,
        ap_hint,
    )
}
