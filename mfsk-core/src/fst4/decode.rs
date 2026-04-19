//! FST4 decode — thin wrapper over [`crate::core::pipeline`].
//!
//! FST4-60A uses LDPC(240, 101) + CRC-24 over the 77-bit WSJT
//! message payload, with 5 × 8-symbol Costas sync blocks. The
//! generic pipeline handles all of that once we supply a
//! [`DownsampleCfg`] tuned for the protocol's geometry.

use super::Fst4s60;
use crate::core::dsp::downsample::DownsampleCfg;
use crate::core::equalize::EqMode;
use crate::core::pipeline::{self, FftCache};

pub use crate::core::pipeline::{DecodeDepth, DecodeResult, DecodeStrictness};

/// FST4-60A downsample configuration: 12 kHz → 62.5 Hz baseband
/// (NDOWN = 192), enough for the 4 tones spaced 3.125 Hz apart
/// (12.5 Hz occupied) plus a generous guard band for the narrow
/// 60-second slot.
///
/// `fft1_size` = 786 432 (= 2¹⁸ · 3, highly composite, ≥ 720 000
/// samples that a 60-s slot at 12 kHz contains). `fft2_size` =
/// fft1 / NDOWN = 4096.
pub const FST4_60A_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 786_432,
    fft2_size: 4_096,
    tone_spacing_hz: 3.125,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4 has 40 sync symbols (5 × 8); require at least a quarter of
/// them right for a candidate to survive coarse-sync scoring.
const SYNC_Q_MIN: u32 = 10;

/// Quarter-symbol time-step, expressed in downsampled samples.
const REFINE_STEPS: i32 = 40;

/// Decode one 60-second FST4-60A slot of 12 kHz PCM audio.
///
/// Typical arguments for a wide-band scan:
/// - `freq_min` / `freq_max` = 100.0 / 3000.0
/// - `sync_min` = 1.0 (lower than FT4 because symbols are 6× longer)
/// - `max_cand` = 50
pub fn decode_frame(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<DecodeResult> {
    pipeline::decode_frame::<Fst4s60>(
        audio,
        &FST4_60A_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        /*freq_hint*/ None,
        DecodeDepth::BpAllOsd,
        max_cand,
        DecodeStrictness::Normal,
        EqMode::Off,
        REFINE_STEPS,
        SYNC_Q_MIN,
    )
    .0
}

/// Same as [`decode_frame`] but also returns the large outer FFT
/// cache so callers can chain further processing (SIC, narrow-band
/// rescan) without recomputing it.
pub fn decode_frame_with_cache(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    pipeline::decode_frame::<Fst4s60>(
        audio,
        &FST4_60A_DOWNSAMPLE,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// Synth → decode_frame roundtrip for a clean FST4-60A signal.
    ///
    /// Gated behind `RUN_FST4_ROUNDTRIP=1` because the 60-s slot +
    /// 786 432-point outer FFT makes this a multi-second test.
    #[test]
    fn synth_decode_roundtrip_cq_ja1abc() {
        if std::env::var("RUN_FST4_ROUNDTRIP").is_err() {
            eprintln!("skipping FST4 roundtrip (set RUN_FST4_ROUNDTRIP=1 to enable)");
            return;
        }

        use super::super::encode::{message_to_tones, tones_to_i16};
        use crate::msg::wsjt77::{pack77, unpack77};

        let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
        let tones = message_to_tones(&msg77);
        let audio = tones_to_i16(&tones, 1500.0, 10_000);

        // Pad to a full 60-second slot with 1 s of leading silence.
        let mut slot = vec![0i16; 60 * 12_000];
        let offset = 12_000;
        let copy_len = audio.len().min(slot.len() - offset);
        slot[offset..offset + copy_len].copy_from_slice(&audio[..copy_len]);

        let results = decode_frame(&slot, 1000.0, 2000.0, 0.8, 20);
        assert!(
            !results.is_empty(),
            "expected at least one decode from clean synth, got none"
        );
        let texts: Vec<String> = results
            .iter()
            .filter_map(|r| unpack77(&r.message77))
            .collect();
        assert!(
            texts
                .iter()
                .any(|t| t.contains("JA1ABC") && t.contains("PM95")),
            "expected to recover 'JA1ABC PM95', got {:?}",
            texts
        );
    }
}
