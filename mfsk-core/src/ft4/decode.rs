//! FT4 decode — thin wrapper over [`crate::core::pipeline`].
//!
//! Exposes `decode_frame` and `decode_frame_subtract` which drive the full
//! generic pipeline (coarse sync → refine → LLR → BP/OSD → optional SIC
//! multi-pass) specialised to the [`Ft4`] protocol. AP hints and sniper
//! single-frequency entry points are not provided here — they can be added
//! once the generic pipeline grows AP support.

use alloc::vec::Vec;

use super::Ft4;
use crate::core::dsp::downsample::DownsampleCfg;
use crate::core::dsp::subtract::SubtractCfg;
use crate::core::equalize::EqMode;
use crate::core::pipeline::{self, FftCache};
use crate::msg::pipeline_ap;

use crate::core::pipeline::DecodeStrictness;
pub use crate::core::pipeline::{DecodeDepth, DecodeResult};
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
/// GFSK shaping matches WSJT-X FT4 (`lib/ft4/subtractft4.f90` declares
/// `bt=1.0`; `lib/ft4/gen_ft4wave.f90` calls `gfsk_pulse(1.0, tt)`).
pub const FT4_SUBTRACT: SubtractCfg = SubtractCfg {
    sample_rate: 12_000.0,
    tone_spacing_hz: 20.833,
    samples_per_symbol: 576,
    base_offset_s: 0.5,
    gfsk: Some(crate::core::dsp::subtract::GfskParams {
        bt: 1.0,
        hmod: 1.0,
        ramp_samples: 576 / 8,
    }),
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
    decode_frame_with_options(
        audio,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
    )
}

/// Decode one FT4 slot with an explicit `depth` knob.
///
/// Mirrors [`crate::ft8::decode::decode_frame`]'s `depth` parameter.
/// FT4's per-candidate strictness is hardcoded to `Normal` — the
/// FT4-specific re-tune of the FT8-calibrated thresholds never landed
/// and no caller exercised the `Strict` / `Deep` rungs (issue #72).
///
/// `freq_hint` is the same as in `ft8::decode::decode_frame`: when
/// `Some(f)`, narrows the coarse-sync to candidates near `f` ± a few Hz.
/// Pass `None` for full-band scan.
pub fn decode_frame_with_options(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    pipeline::decode_frame::<Ft4>(
        audio,
        &FT4_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
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
    decode_frame_with_cache_and_options(
        audio,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
    )
}

/// Same as [`decode_frame_with_cache`] but with an explicit `depth` knob
/// (see [`decode_frame_with_options`]).
///
/// `freq_hint`: when `Some(f)`, narrows the coarse-sync to candidates
/// near `f`. Pass `None` for full-band scan. Mirrors
/// [`decode_frame_with_options`] for parity.
pub fn decode_frame_with_cache_and_options(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    pipeline::decode_frame::<Ft4>(
        audio,
        &FT4_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
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
    decode_frame_subtract_with_options(
        audio,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
    )
}

/// Same as [`decode_frame_subtract`] but with an explicit `depth` knob
/// (see [`decode_frame_with_options`]).
///
/// `freq_hint`: when `Some(f)`, narrows the coarse-sync to candidates
/// near `f`. Pass `None` for full-band scan. Mirrors
/// [`decode_frame_with_options`] for parity.
pub fn decode_frame_subtract_with_options(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    pipeline::decode_frame_subtract::<Ft4>(
        audio,
        &FT4_DOWNSAMPLE,
        &FT4_SUBTRACT,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
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
    decode_sniper_ap_with_options(
        audio,
        target_freq,
        DecodeDepth::BpAllOsd,
        max_cand,
        eq_mode,
        ap_hint,
    )
}

/// Same as [`decode_sniper_ap`] but with an explicit `depth` knob
/// (see [`decode_frame_with_options`]).
pub fn decode_sniper_ap_with_options(
    audio: &[i16],
    target_freq: f32,
    depth: DecodeDepth,
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
        depth,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Compile-time check that `decode_frame_with_options` accepts every
    /// `DecodeDepth` rung. No actual decoding happens — empty audio
    /// returns no candidates fast — but this guards against future
    /// signature drift.
    #[test]
    fn decode_frame_with_options_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 7500]; // 7.5 s of silence at 12 kHz
        for &depth in &[DecodeDepth::BpAll, DecodeDepth::BpAllOsd] {
            let _ = decode_frame_with_options(&empty, 100.0, 3000.0, 1.0, None, depth, 5);
        }
    }

    /// Compile-time check that `decode_frame_with_cache_and_options`
    /// accepts every `DecodeDepth` rung.
    #[test]
    fn decode_frame_with_cache_and_options_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 7500];
        for &depth in &[DecodeDepth::BpAll, DecodeDepth::BpAllOsd] {
            let _ = decode_frame_with_cache_and_options(&empty, 100.0, 3000.0, 1.0, None, depth, 5);
        }
    }

    /// Compile-time check that `decode_frame_subtract_with_options`
    /// accepts every `DecodeDepth` rung.
    #[test]
    fn decode_frame_subtract_with_options_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 7500];
        for &depth in &[DecodeDepth::BpAll, DecodeDepth::BpAllOsd] {
            let _ = decode_frame_subtract_with_options(&empty, 100.0, 3000.0, 1.0, None, depth, 5);
        }
    }

    /// Compile-time check that `decode_sniper_ap_with_options` accepts
    /// every `DecodeDepth` rung.
    #[test]
    fn decode_sniper_ap_with_options_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 7500];
        for &depth in &[DecodeDepth::BpAll, DecodeDepth::BpAllOsd] {
            let _ = decode_sniper_ap_with_options(&empty, 1500.0, depth, 5, EqMode::Off, None);
        }
    }
}
