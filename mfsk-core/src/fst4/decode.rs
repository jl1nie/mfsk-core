//! FST4 decode — thin wrapper over [`crate::core::pipeline`].
//!
//! Every FST4 sub-mode uses LDPC(240, 101) + CRC-24 over the 77-bit
//! WSJT message payload, with 5 × 8-symbol Costas sync blocks. The
//! generic pipeline handles all of that once we supply a
//! [`DownsampleCfg`] tuned for the sub-mode's geometry — see the
//! `FST4_*_DOWNSAMPLE` constants below and [`decode_frame_for`] /
//! [`decode_frame_with_options_for`] / [`decode_frame_with_cache_for`] /
//! [`decode_frame_with_cache_and_options_for`] for the generic entry
//! points. `decode_frame` and friends (no `_for` suffix) are FST4-60A
//! convenience wrappers kept for backward compatibility.

use super::Fst4s60;
use crate::core::Protocol;
use crate::core::dsp::downsample::DownsampleCfg;
use crate::core::equalize::EqMode;
use crate::core::pipeline::{self, FftCache};

use crate::core::pipeline::DecodeStrictness;
pub use crate::core::pipeline::{DecodeDepth, DecodeResult};

/// FST4-15 downsample configuration: 12 kHz → 666.7 Hz baseband
/// (NDOWN = 18, matching WSJT-X `fst4_decode.f90`'s `ndown` for
/// `ntrperiod.eq.15`). `fft1_size` = 180 000 (exactly `T_SLOT_S ×
/// 12 000`, already an exact multiple of NDOWN=18 so no padding is
/// needed). `fft2_size` = fft1 / NDOWN = 10 000.
pub const FST4_15_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 180_000,
    fft2_size: 10_000,
    tone_spacing_hz: 12_000.0 / 720.0,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4-30 downsample configuration: 12 kHz → 285.7 Hz baseband
/// (NDOWN = 42, matching WSJT-X `fst4_decode.f90`'s `ndown` for
/// `ntrperiod.eq.30`). `fft1_size` = 362 880 (= 8640 × 42, ≥ 360 000
/// samples that a 30-s slot contains). `fft2_size` = 8640.
pub const FST4_30_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 362_880,
    fft2_size: 8_640,
    tone_spacing_hz: 12_000.0 / 1_680.0,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4-60A downsample configuration: 12 kHz → 111.11 Hz baseband
/// (NDOWN = 108, matching WSJT-X `fst4_decode.f90`'s `fs2 = fs/ndown`
/// for `ntrperiod.eq.60`), enough for the 4 tones spaced 3.0864 Hz
/// apart (12.35 Hz occupied) plus a generous guard band for the
/// narrow 60-second slot.
///
/// `fft1_size` = 746 496 (= 2¹⁰ · 3⁶, highly composite, ≥ 720 000
/// samples that a 60-s slot at 12 kHz contains, and an exact multiple
/// of NDOWN=108). `fft2_size` = fft1 / NDOWN = 6912.
pub const FST4_60A_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 746_496,
    fft2_size: 6_912,
    tone_spacing_hz: 12_000.0 / 3_888.0,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4-120 downsample configuration: 12 kHz → 58.5 Hz baseband
/// (NDOWN = 205, matching WSJT-X `fst4_decode.f90`'s `ndown` for
/// `ntrperiod.eq.120`). `fft1_size` = 1 443 200 (= 7040 × 205, ≥
/// 1 440 000 samples that a 120-s slot contains). `fft2_size` = 7040.
/// NDOWN=205=5×41 has no small-prime factorisation, so `fft1_size`
/// unavoidably carries the factor 41 — rustfft still handles it
/// correctly via mixed-radix / Bluestein, just not at the same speed
/// as a power-of-two size.
pub const FST4_120_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 1_443_200,
    fft2_size: 7_040,
    tone_spacing_hz: 12_000.0 / 8_200.0,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4-300 downsample configuration: 12 kHz → 23.4 Hz baseband
/// (NDOWN = 512, matching WSJT-X `fst4_decode.f90`'s `ndown` for
/// `ntrperiod.eq.300`). `fft1_size` = 4 194 304 (= 2²², a pure
/// power-of-two chosen since NDOWN=512=2⁹ is already a power of two;
/// ≥ 3 600 000 samples that a 300-s slot contains). `fft2_size` =
/// 8192.
pub const FST4_300_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 4_194_304,
    fft2_size: 8_192,
    tone_spacing_hz: 12_000.0 / 21_504.0,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4-900 downsample configuration from WSJT-X `fst4_decode.f90`:
/// NDOWN=1664, `nfft1=6480*1664`, `nfft2=6480`.
pub const FST4_900_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 6_480 * 1_664,
    fft2_size: 6_480,
    tone_spacing_hz: 12_000.0 / 66_560.0,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4-1800 downsample configuration from WSJT-X `fst4_decode.f90`:
/// NDOWN=3360, `nfft1=6426*3360`, `nfft2=6426`.
pub const FST4_1800_DOWNSAMPLE: DownsampleCfg = DownsampleCfg {
    input_rate: 12_000,
    fft1_size: 6_426 * 3_360,
    fft2_size: 6_426,
    tone_spacing_hz: 12_000.0 / 134_400.0,
    leading_pad_tones: 1.5,
    trailing_pad_tones: 1.5,
    ntones: 4,
    edge_taper_bins: 101,
};

/// FST4 has 40 sync symbols (5 × 8); require at least a quarter of
/// them right for a candidate to survive coarse-sync scoring. Shared
/// by every sub-mode.
const SYNC_Q_MIN: u32 = 10;

/// Fine-refine time-domain search half-width, in *downsampled*
/// samples. Every FST4 sub-mode's downsampled samples-per-symbol
/// (`NSPS / NDOWN`) lands in the 36-42 range (WSJT-X picks each
/// sub-mode's `ndown` so that ratio stays roughly constant), so this
/// fixed raw-sample count corresponds to a consistent ~1-symbol
/// search window across all of them — no per-sub-mode retuning needed.
const REFINE_STEPS: i32 = 40;

/// Decode one slot of 12 kHz PCM audio for FST4 sub-mode `P`.
///
/// Typical arguments for a wide-band scan:
/// - `freq_min` / `freq_max` = 100.0 / 3000.0
/// - `sync_min` = 1.0 (lower than FT4 because symbols are 6× longer)
/// - `max_cand` = 50
///
/// `cfg` must match `P` — pass the corresponding `FST4_*_DOWNSAMPLE`
/// constant (e.g. [`FST4_120_DOWNSAMPLE`] for `P = Fst4s120`).
pub fn decode_frame_for<P: Protocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_frame_with_options_for::<P>(
        audio,
        cfg,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
    )
}

/// Decode one FST4 slot with an explicit `depth` knob. See
/// [`decode_frame_for`] for the `cfg` contract.
///
/// FST4's per-candidate strictness is hardcoded to `Normal` — the
/// FST4-specific re-tune of the FT8-calibrated thresholds never landed
/// and no caller exercised the `Strict` / `Deep` rungs (issue #72).
///
/// `freq_hint`: when `Some(f)`, narrows the coarse-sync to candidates
/// near `f`. Pass `None` for full-band scan.
pub fn decode_frame_with_options_for<P: Protocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    pipeline::decode_frame::<P>(
        audio,
        cfg,
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

/// Same as [`decode_frame_for`] but also returns the large outer FFT
/// cache so callers can chain further processing (SIC, narrow-band
/// rescan) without recomputing it.
pub fn decode_frame_with_cache_for<P: Protocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    decode_frame_with_cache_and_options_for::<P>(
        audio,
        cfg,
        freq_min,
        freq_max,
        sync_min,
        None,
        DecodeDepth::BpAllOsd,
        max_cand,
    )
}

/// Same as [`decode_frame_with_cache_for`] but with an explicit
/// `depth` knob (see [`decode_frame_with_options_for`]).
pub fn decode_frame_with_cache_and_options_for<P: Protocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    pipeline::decode_frame::<P>(
        audio,
        cfg,
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

/// Decode one 60-second FST4-60A slot of 12 kHz PCM audio.
///
/// Convenience wrapper over [`decode_frame_for`]`::<Fst4s60>` with
/// [`FST4_60A_DOWNSAMPLE`] — kept for backward compatibility with
/// callers that predate the multi-sub-mode `_for` entry points. Other
/// sub-modes: call `decode_frame_for::<Fst4s120>(audio, cfg, ...)`
/// etc. directly with the matching `FST4_*_DOWNSAMPLE` constant.
pub fn decode_frame(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_frame_for::<Fst4s60>(
        audio,
        &FST4_60A_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
    )
}

/// FST4-60A convenience wrapper over [`decode_frame_with_options_for`].
/// See [`decode_frame`] for the multi-sub-mode alternative.
pub fn decode_frame_with_options(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> Vec<DecodeResult> {
    decode_frame_with_options_for::<Fst4s60>(
        audio,
        &FST4_60A_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
    )
}

/// FST4-60A convenience wrapper over [`decode_frame_with_cache_for`].
/// See [`decode_frame`] for the multi-sub-mode alternative.
pub fn decode_frame_with_cache(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    decode_frame_with_cache_for::<Fst4s60>(
        audio,
        &FST4_60A_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        max_cand,
    )
}

/// FST4-60A convenience wrapper over
/// [`decode_frame_with_cache_and_options_for`]. See [`decode_frame`]
/// for the multi-sub-mode alternative.
pub fn decode_frame_with_cache_and_options(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
) -> (Vec<DecodeResult>, FftCache) {
    decode_frame_with_cache_and_options_for::<Fst4s60>(
        audio,
        &FST4_60A_DOWNSAMPLE,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synth → decode_frame roundtrip for a clean FST4-60A signal.
    ///
    /// Gated behind `RUN_FST4_ROUNDTRIP=1` because the 60-s slot +
    /// 746 496-point outer FFT makes this a multi-second test.
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
            .filter_map(|r| {
                let msg77: &[u8; 77] = r.message77().try_into().ok()?;
                unpack77(msg77)
            })
            .collect();
        assert!(
            texts
                .iter()
                .any(|t| t.contains("JA1ABC") && t.contains("PM95")),
            "expected to recover 'JA1ABC PM95', got {:?}",
            texts
        );
    }

    /// Generic synth → decode roundtrip shared by every non-60A
    /// sub-mode test below. Same structure as
    /// `synth_decode_roundtrip_cq_ja1abc`: encode a clean message,
    /// pad into a full slot with 1 s of leading silence, decode, and
    /// confirm the message comes back.
    ///
    /// This is a **self-consistency** check only — encode and decode
    /// share the same `NSPS`/`NDOWN`/`GFSK_BT` constants, so it cannot
    /// catch a wrong-vs-WSJT-X parameter the way real WSJT-X-generated
    /// audio would (this is exactly how issue #23's FST4-60A bug
    /// stayed hidden). No golden WAV exists locally for FST4-15/30/
    /// 120/300 (the WSJT-X sample tree only ships FST4-60A and
    /// FST4W-1800 recordings) — the `NSPS`/`NDOWN`/`TX_START_OFFSET_S`
    /// values themselves were verified directly against WSJT-X
    /// `fst4_decode.f90` / `fst4sim.f90` source (see
    /// `fst4::tests::all_submodes_match_wsjtx_fst4_decode_f90`), which
    /// is the strongest available check without either a real
    /// recording or a WSJT-X `fst4sim`-generated reference WAV.
    fn synth_roundtrip_for<P: crate::core::Protocol + crate::core::FrameLayout>(
        cfg: &DownsampleCfg,
        gfsk: &crate::core::dsp::gfsk::GfskCfg,
        freq_min: f32,
        freq_max: f32,
    ) {
        use super::super::encode::{message_to_tones, tones_to_i16_with_gfsk};
        use crate::msg::wsjt77::{pack77, unpack77};

        let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
        let tones = message_to_tones(&msg77);
        let audio = tones_to_i16_with_gfsk(&tones, (freq_min + freq_max) / 2.0, 10_000, gfsk);

        // Pad to a full slot with 1 s of leading silence.
        let slot_len = (P::T_SLOT_S * 12_000.0).round() as usize;
        let mut slot = vec![0i16; slot_len];
        let offset = 12_000usize;
        let copy_len = audio.len().min(slot_len.saturating_sub(offset));
        slot[offset..offset + copy_len].copy_from_slice(&audio[..copy_len]);

        let results = decode_frame_for::<P>(&slot, cfg, freq_min, freq_max, 0.8, 20);
        assert!(
            !results.is_empty(),
            "expected at least one decode from clean synth, got none"
        );
        let texts: Vec<String> = results
            .iter()
            .filter_map(|r| {
                let msg77: &[u8; 77] = r.message77().try_into().ok()?;
                unpack77(msg77)
            })
            .collect();
        assert!(
            texts
                .iter()
                .any(|t| t.contains("JA1ABC") && t.contains("PM95")),
            "expected to recover 'JA1ABC PM95', got {:?}",
            texts
        );
    }

    /// Gated behind `RUN_FST4_ROUNDTRIP=1` (see
    /// `synth_decode_roundtrip_cq_ja1abc`).
    #[test]
    fn synth_decode_roundtrip_fst4_15() {
        if std::env::var("RUN_FST4_ROUNDTRIP").is_err() {
            eprintln!("skipping FST4-15 roundtrip (set RUN_FST4_ROUNDTRIP=1 to enable)");
            return;
        }
        synth_roundtrip_for::<super::super::Fst4s15>(
            &FST4_15_DOWNSAMPLE,
            &super::super::encode::FST4_15_GFSK,
            1000.0,
            2000.0,
        );
    }

    /// Gated behind `RUN_FST4_ROUNDTRIP=1`.
    #[test]
    fn synth_decode_roundtrip_fst4_30() {
        if std::env::var("RUN_FST4_ROUNDTRIP").is_err() {
            eprintln!("skipping FST4-30 roundtrip (set RUN_FST4_ROUNDTRIP=1 to enable)");
            return;
        }
        synth_roundtrip_for::<super::super::Fst4s30>(
            &FST4_30_DOWNSAMPLE,
            &super::super::encode::FST4_30_GFSK,
            1000.0,
            2000.0,
        );
    }

    /// Gated behind `RUN_FST4_ROUNDTRIP=1`. Slower than the 15/30/60 s
    /// variants (109 s of audio, ~1.44M-point outer FFT).
    #[test]
    fn synth_decode_roundtrip_fst4_120() {
        if std::env::var("RUN_FST4_ROUNDTRIP").is_err() {
            eprintln!("skipping FST4-120 roundtrip (set RUN_FST4_ROUNDTRIP=1 to enable)");
            return;
        }
        synth_roundtrip_for::<super::super::Fst4s120>(
            &FST4_120_DOWNSAMPLE,
            &super::super::encode::FST4_120_GFSK,
            1000.0,
            2000.0,
        );
    }

    /// Gated behind `RUN_FST4_ROUNDTRIP=1`. Slowest of the roundtrip
    /// tests (287 s of audio, ~4.19M-point outer FFT) — expect several
    /// seconds.
    #[test]
    fn synth_decode_roundtrip_fst4_300() {
        if std::env::var("RUN_FST4_ROUNDTRIP").is_err() {
            eprintln!("skipping FST4-300 roundtrip (set RUN_FST4_ROUNDTRIP=1 to enable)");
            return;
        }
        synth_roundtrip_for::<super::super::Fst4s300>(
            &FST4_300_DOWNSAMPLE,
            &super::super::encode::FST4_300_GFSK,
            1000.0,
            2000.0,
        );
    }

    /// Compile-time check that `decode_frame_with_options` accepts every
    /// `DecodeDepth` rung. No actual decoding happens — empty audio
    /// returns no candidates fast — but this guards against future
    /// signature drift breaking downstream callers that do parameterised
    /// dispatch.
    #[test]
    fn decode_frame_with_options_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 60 * 1000]; // 60 s of silence
        for &depth in &[DecodeDepth::BpAll, DecodeDepth::BpAllOsd] {
            let _ = decode_frame_with_options(&empty, 100.0, 3000.0, 0.8, None, depth, 5);
        }
    }

    /// Compile-time check that `decode_frame_with_cache_and_options`
    /// accepts every `DecodeDepth` rung.
    #[test]
    fn decode_frame_with_cache_and_options_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 60 * 1000]; // 60 s of silence
        for &depth in &[DecodeDepth::BpAll, DecodeDepth::BpAllOsd] {
            let _ = decode_frame_with_cache_and_options(&empty, 100.0, 3000.0, 0.8, None, depth, 5);
        }
    }
}
