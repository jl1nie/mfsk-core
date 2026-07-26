//! FST4 decode — thin wrapper over [`crate::core::pipeline`].
//!
//! Every FST4 sub-mode uses LDPC(240, 101) + CRC-24 over the 77-bit
//! WSJT message payload, with 5 × 8-symbol Costas sync blocks. The
//! generic pipeline handles all of that once we supply a
//! [`DownsampleCfg`] tuned for the sub-mode's geometry — see the
//! `FST4_*_DOWNSAMPLE` constants below. Exposed via the shared
//! [`crate::msg::decode_request::DecodeRequest`] /
//! [`crate::msg::decode_request::SniperRequest`] builders, generic over
//! `P` (issue #191) — e.g. `DecodeRequest::<Fst4s120>::new(...)` with
//! `req.audio`/etc, dispatching internally to the matching
//! `FST4_*_DOWNSAMPLE` constant for that sub-mode.
//!
//! FST4 has no SIC (successive interference cancellation) path — no
//! `SubtractCfg` exists for it, so
//! [`SupportsFlatSic`](crate::msg::decode_request::SupportsFlatSic) is not
//! implemented for any sub-mode (issue #193: new numerical work, not a
//! refactor, kept out of this redesign's scope).

use crate::core::dsp::downsample::DownsampleCfg;
use crate::core::pipeline;

pub use crate::core::pipeline::{DecodeDepth, DecodeResult, DecodeStrictness};
pub use crate::msg::ApHint;
use crate::msg::decode_request::{DecodeOutcome, DecodeRequest, FrameDecodable, SniperRequest};

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

/// Dedup `raw` against caller-supplied `known` (by `info` equality) — the
/// generic engine has no `known` parameter at all, so this is a
/// best-effort post-filter rather than an in-loop skip (same rationale as
/// `ft4::decode`'s copy of this helper).
fn dedup_known(raw: Vec<DecodeResult>, known: &[DecodeResult]) -> Vec<DecodeResult> {
    raw.into_iter()
        .filter(|r| !known.iter().any(|k| k.info == r.info))
        .collect()
}

/// Implements [`FrameDecodable`] for one FST4 sub-mode ZST, wiring in its
/// `DownsampleCfg`. Every sub-mode shares the same generic engine
/// (`core::pipeline`/`msg::pipeline_ap`), `REFINE_STEPS`, and
/// `SYNC_Q_MIN` — only the downsample geometry differs.
macro_rules! impl_frame_decodable {
    ($proto:ty, $cfg:expr) => {
        impl FrameDecodable for $proto {
            type DecodeResult = DecodeResult;

            fn __single_pass(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
                let (raw, fft_cache) = pipeline::decode_frame::<$proto>(
                    req.audio,
                    &$cfg,
                    req.freq_min,
                    req.freq_max,
                    req.sync_min,
                    req.freq_hint,
                    req.depth,
                    req.max_cand,
                    req.strictness,
                    req.eq_mode,
                    REFINE_STEPS,
                    SYNC_Q_MIN,
                );
                DecodeOutcome {
                    results: dedup_known(raw, req.known),
                    fft_cache,
                }
            }

            fn __sniper(req: &SniperRequest<'_, Self>) -> DecodeOutcome<Self> {
                let results = crate::msg::pipeline_ap::decode_sniper_ap::<$proto>(
                    req.audio,
                    &$cfg,
                    req.target_freq,
                    250.0,
                    req.sync_min,
                    req.depth,
                    req.max_cand,
                    req.strictness,
                    req.eq_mode,
                    REFINE_STEPS,
                    SYNC_Q_MIN / 2,
                    req.ap_hint,
                );
                let fft_cache = crate::core::dsp::downsample::build_fft_cache(req.audio, &$cfg);
                DecodeOutcome { results, fft_cache }
            }
        }
    };
}

impl_frame_decodable!(super::Fst4s15, FST4_15_DOWNSAMPLE);
impl_frame_decodable!(super::Fst4s30, FST4_30_DOWNSAMPLE);
impl_frame_decodable!(super::Fst4s60, FST4_60A_DOWNSAMPLE);
impl_frame_decodable!(super::Fst4s120, FST4_120_DOWNSAMPLE);
impl_frame_decodable!(super::Fst4s300, FST4_300_DOWNSAMPLE);

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

        let results = DecodeRequest::<crate::fst4::Fst4s60>::new(&slot, 1000.0, 2000.0, 0.8, 20)
            .decode()
            .results;
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
    fn synth_roundtrip_for<P>(gfsk: &crate::core::dsp::gfsk::GfskCfg, freq_min: f32, freq_max: f32)
    where
        P: crate::core::Protocol
            + crate::core::FrameLayout
            + FrameDecodable<DecodeResult = DecodeResult>,
    {
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

        let results = DecodeRequest::<P>::new(&slot, freq_min, freq_max, 0.8, 20)
            .decode()
            .results;
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
            &super::super::encode::FST4_300_GFSK,
            1000.0,
            2000.0,
        );
    }

    /// Compile-time check that `DecodeRequest<Fst4s60>` accepts every
    /// `DecodeDepth` rung across single-pass and sniper. No actual
    /// decoding happens — empty audio returns no candidates fast — but
    /// this guards against future signature drift breaking downstream
    /// callers that do parameterised dispatch.
    #[test]
    fn decode_request_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 60 * 1000]; // 60 s of silence
        for &depth in &[DecodeDepth::BP_ONLY, DecodeDepth::FULL] {
            let _ = DecodeRequest::<crate::fst4::Fst4s60>::new(&empty, 100.0, 3000.0, 0.8, 5)
                .depth(depth)
                .decode();
            let _ = DecodeRequest::<crate::fst4::Fst4s60>::sniper(&empty, 1500.0, 5)
                .depth(depth)
                .decode();
        }
    }
}
