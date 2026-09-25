//! FST4 decode — thin wrapper over [`crate::engine::pipeline`].
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
//! [`SupportsSicRounds`](crate::msg::decode_request::SupportsSicRounds) is
//! not implemented for any sub-mode (issue #193: new numerical work, not a
//! refactor, kept out of this redesign's scope).

use crate::engine::dsp::downsample::DownsampleCfg;
use crate::engine::pipeline;

pub use crate::engine::pipeline::{DecodeDepth, DecodeResult, DecodeStrictness, FftCache};
pub use crate::msg::ApHint;
use crate::msg::decode_request::{DecodeOutcome, DecodeRequest, FrameDecodable, MessagePolicy};

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

/// FST4 has 40 sync symbols (5 × 8). Matches WSJT-X's own pre-ladder
/// gate exactly (`get_fst4_bitmetrics.f90`: `if(nsync .lt. 16)
/// badsync=.true.; return` — bails before the expensive nsym=1/2/4/8
/// correlation ladder, `engine::llr::compute_llr`, ever runs). Was `10`
/// (a quarter of 40) — looser than WSJT-X's `16` (40%), so candidates
/// WSJT-X would already reject pre-ladder were paying for the full
/// ladder (including the 65536-hypothesis nsym=8 rung) in our pipeline
/// too — issue #197. Shared by every sub-mode.
const SYNC_Q_MIN: u32 = 16;

/// Implements [`FrameDecodable`] for one FST4 sub-mode ZST, wiring in its
/// `DownsampleCfg`. Every sub-mode shares the same generic engine
/// (`engine::pipeline`/`msg::pipeline_ap`), `REFINE_STEPS`, and
/// `SYNC_Q_MIN` — only the downsample geometry differs.
macro_rules! impl_frame_decodable {
    ($proto:ty, $cfg:expr) => {
        impl pipeline::GenericPipelineProtocol for $proto {
            /// `fst4_decode.f90:592-621` — see
            /// [`crate::fst4::baseline`]'s module doc for the formula,
            /// the real-`jt9` ground-truth verification, and the two
            /// corrections (RMS-normalisation mismatch, downsample
            /// scale-convention mismatch) it took to land within ~1-2
            /// dB of jt9's own reported SNR (issue #255).
            fn snr_db(ctx: pipeline::SnrCtx<'_>) -> f32 {
                if ctx.fft_cache.is_empty() {
                    // A DDC receiver: no whole-slot FFT exists, by
                    // design (`fst4::ddc`), so WSJT-X's own formula has
                    // no inputs. Measure the noise outside the signal's
                    // own spectrum in the refined baseband instead —
                    // see `fst4_ddc_snr_db`, and its doc comment for
                    // why the symbol spectra cannot supply that
                    // reference themselves.
                    return crate::fst4::baseline::fst4_ddc_snr_db::<$proto>(
                        ctx.cd0,
                        ctx.ds_rate_hz,
                    )
                    .unwrap_or(-99.9);
                }
                crate::fst4::baseline::fst4_snr_db::<$proto>(
                    ctx.itone,
                    ctx.cand_freq_hz,
                    ctx.refined_freq_hz,
                    ctx.i_start,
                    ctx.fft_cache,
                    ctx.ds_cfg,
                    <$proto>::SNR_CALFAC,
                )
            }
        }

        impl crate::msg::decode_request::SupportsWideBandAp for $proto {}

        /// Opt-in only: `MESSAGE_FILTER_DEFAULT` stays `false` for every
        /// FST4 sub-mode, so a request that names no policy decodes
        /// bit-identically. Not measured since the OSD searches `Keff = 91`
        /// (#456), which took the false-positive rate per OSD call from
        /// 2^-24 to FT8's 2^-14 — see
        /// `FrameDecodable::MESSAGE_FILTER_DEFAULT`.
        impl crate::msg::decode_request::SupportsMessageFilter for $proto {
            fn __strategy_for<Pol: MessagePolicy>(
                tag: crate::msg::decode_request::StrategyTag,
            ) -> fn(&DecodeRequest<'_, Self, Pol>) -> DecodeOutcome<Self> {
                match tag {
                    crate::msg::decode_request::StrategyTag::SinglePass => {
                        Self::__single_pass::<Pol>
                    }
                    // Unreachable by construction: `.sic_rounds()` and
                    // `.sic_early()` live on impls gated by traits no
                    // FST4 sub-mode implements, so neither tag can be
                    // set on an FST4 request. WSJT-X's own
                    // `fst4_decode.f90` has no SIC path either.
                    _ => unreachable!("FST4 has no SIC strategy"),
                }
            }
        }

        impl FrameDecodable for $proto {
            type DecodeResult = DecodeResult;

            /// `fst4_decode.f90:570`: `nharderrors.ge.0 .and. unpk77_success`.
            /// With `Keff = 91` only 14 of the CRC's 24 bits detect a wrong
            /// codeword, so garbage that verifies is no longer 2^-24 rare and
            /// a message that will not unpack is what refuses most of it.
            ///
            /// It also does the job of `fst4_decode.f90:484-487`
            /// (`count(cw.eq.1).eq.0`: drop the all-zero codeword, whose CRC
            /// is 0 and so always verifies): the raw all-zero word descrambles
            /// to `FST4_RVEC`, and `unpack77(FST4_RVEC)` is `None` (checked
            /// 2026-09-25). No separate guard belongs in the (240,101) codec —
            /// uvpacket shares it, has no scramble, and can send zeros.
            const REQUIRES_UNPACK: bool = true;

            fn __single_pass<Pol: MessagePolicy>(
                req: &DecodeRequest<'_, Self, Pol>,
            ) -> DecodeOutcome<Self> {
                crate::msg::decode_request::generic_single_pass(req, &$cfg, SYNC_Q_MIN)
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
    #[test]
    fn synth_decode_roundtrip_cq_ja1abc() {
        use crate::msg::wsjt77::{pack77, unpack77};

        let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
        let tones = crate::engine::tx::message_to_tones::<crate::fst4::Fst4s60>(&msg77);
        let audio = crate::engine::tx::synthesize_i16::<crate::fst4::Fst4s60>(
            &tones, 12_000, 1500.0, 10_000,
        );

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
                let msg77 = r.message77();
                unpack77(msg77)
            })
            .collect();
        // Precision, not just recall. One signal went in, so anything
        // else that comes out is a phantom — and a decoder with no CRC
        // slack can pass a recall-only assertion while emitting them.
        // This is the cheapest place to notice that per sub-mode:
        // a clean synth slot has no interferer to blame.
        let phantoms: Vec<&String> = texts
            .iter()
            .filter(|t| !(t.contains("JA1ABC") && t.contains("PM95")))
            .collect();
        assert!(
            phantoms.is_empty(),
            "clean single-signal synth produced phantom decode(s): {phantoms:?}"
        );
        let phantoms: Vec<&String> = texts
            .iter()
            .filter(|t| !(t.contains("JA1ABC") && t.contains("PM95")))
            .collect();
        assert!(
            phantoms.is_empty(),
            "clean single-signal synth produced phantom decode(s): {phantoms:?}"
        );
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
    fn synth_roundtrip_for<P>(freq_min: f32, freq_max: f32)
    where
        P: crate::engine::Protocol
            + crate::engine::FrameLayout
            + crate::engine::tx::FskWaveform
            + FrameDecodable<DecodeResult = DecodeResult>,
    {
        use crate::msg::wsjt77::{pack77, unpack77};

        let msg77 = pack77("CQ", "JA1ABC", "PM95").expect("pack77");
        let tones = crate::engine::tx::message_to_tones::<crate::fst4::Fst4s60>(&msg77);
        let audio = crate::engine::tx::synthesize_i16::<P>(
            &tones,
            12_000,
            (freq_min + freq_max) / 2.0,
            10_000,
        );

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
                let msg77 = r.message77();
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

    #[test]
    fn synth_decode_roundtrip_fst4_15() {
        synth_roundtrip_for::<super::super::Fst4s15>(1000.0, 2000.0);
    }

    #[test]
    fn synth_decode_roundtrip_fst4_30() {
        synth_roundtrip_for::<super::super::Fst4s30>(1000.0, 2000.0);
    }

    #[test]
    fn synth_decode_roundtrip_fst4_120() {
        synth_roundtrip_for::<super::super::Fst4s120>(1000.0, 2000.0);
    }

    #[test]
    fn synth_decode_roundtrip_fst4_300() {
        synth_roundtrip_for::<super::super::Fst4s300>(1000.0, 2000.0);
    }
}
