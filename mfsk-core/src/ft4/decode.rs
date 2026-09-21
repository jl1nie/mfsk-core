//! FT4 decode — thin wrapper over [`crate::engine::pipeline`].
//!
//! Drives the full generic pipeline (coarse sync → refine → LLR → BP/OSD →
//! optional SIC multi-pass) specialised to the [`Ft4`] protocol, exposed
//! via the shared [`crate::msg::decode_request::DecodeRequest`] /
//! [`crate::msg::decode_request::SniperRequest`] builders (issue #191).

use alloc::vec::Vec;

use super::Ft4;
use crate::engine::dsp::downsample::DownsampleCfg;
use crate::engine::dsp::subtract::SubtractCfg;
use crate::engine::pipeline;

pub use crate::engine::pipeline::{DecodeDepth, DecodeResult, DecodeStrictness, FftCache};
pub use crate::msg::ApHint;
use crate::msg::decode_request::{
    DecodeOutcome, DecodeRequest, FrameDecodable, MessagePolicy, StrategyTag,
    SupportsMessageFilter, SupportsSicRounds,
};

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
    gfsk: Some(crate::engine::dsp::subtract::GfskParams {
        bt: 1.0,
        hmod: 1.0,
        ramp_samples: 576 / 8,
    }),
};

/// FT4 has 16 sync symbols (4 × 4); require at least half correct.
const SYNC_Q_MIN: u32 = 8;

impl pipeline::GenericPipelineProtocol for Ft4 {
    /// `ft4_decode.f90:226,452-457` — see [`pipeline::ft4_snr_db`]'s doc
    /// comment for the formula and its verification against a real
    /// local `jt9` build (issue #255).
    fn snr_db(ctx: pipeline::SnrCtx<'_>) -> f32 {
        pipeline::ft4_snr_db(ctx.cand_score)
    }
}

impl FrameDecodable for Ft4 {
    type DecodeResult = DecodeResult;

    /// **On, and measured before it was turned on.** See
    /// [`FrameDecodable::MESSAGE_FILTER_DEFAULT`] for why FT8 runs the
    /// codec's verdict: a CRC-14 false positive that reaches
    /// `.sic_rounds()` is *subtracted* from the audio, taking whatever
    /// real signal was underneath with it. FT4 has the same CRC-14 and
    /// the same `SupportsSicRounds`, so the argument transferred — what
    /// had never been measured was the cost, because the verdict's
    /// ITU-prefix allowlist was tuned on an FT8 recording and FT4 is a
    /// contest mode full of DX prefixes.
    ///
    /// Measured 2026-09-21 on the `ft4sim` corpus, 720 slots across the
    /// threshold window (−21..−13 dB, four ITU-R channels, 20 trials a
    /// cell — `ft4_sweep::ft4_phantom_rate` and the `ft4_snr_sweep`
    /// A/B behind `MFSK_FT4_SWEEP_CODEC_FILTER`):
    ///
    /// | | verdict off | verdict on |
    /// |---|---|---|
    /// | golden rows | 353 | **354** |
    /// | phantom rows | 7 | **2** |
    ///
    /// 35 of the 36 recall cells are identical; the one that moves goes
    /// *up* (`awgn −17 dB`, 16/20 → 17/20), because a rejection lets the
    /// candidate ladder keep going and reach a real decode a phantom
    /// had taken the slot from.
    ///
    /// **What that corpus cannot say**: every one of its slots carries
    /// the same callsign, so the allowlist's own risk — an unusual
    /// prefix on the air — is not exercised by it. The WSJT-X golden
    /// recording is (`ft4_message_policy`, where the verdict drops
    /// nothing), and a deployment that needs more can widen it with
    /// [`DecodeRequest::also_accept`].
    const MESSAGE_FILTER_DEFAULT: bool = true;

    fn __single_pass<Pol: MessagePolicy>(
        req: &DecodeRequest<'_, Self, Pol>,
    ) -> DecodeOutcome<Self> {
        // See `pipeline::known_filtered_on_result`'s doc comment: without
        // this, `on_result` could fire for a candidate `pipeline::dedup_known`
        // below then silently drops from the returned `Vec`.
        let filtered_cb = pipeline::known_filtered_on_result(req.known, req.on_result);
        let on_result: Option<&(dyn Fn(&DecodeResult) + Sync)> = filtered_cb
            .as_ref()
            .map(|f| f as &(dyn Fn(&DecodeResult) + Sync));
        // Every a-priori hypothesis WSJT-X would try, not just the
        // caller's literal hint — and the blind CQ one **whether or
        // not a hint was given at all**.
        //
        // WSJT-X runs AP passes on every decode: `ft4_decode.f90:328`
        // `npasses = 3 + nappasses(nQSOProgress)`, and its `iaptype = 1`
        // locks the first 29 bits to the CQ pattern using no knowledge
        // of the station at all. mfsk-core ran AP only when a caller
        // supplied a hint, so a blind decode attempted none — while FT8
        // has had the equivalent since issue #190, where adding it is
        // what closed FT8's own gap against the published figure.
        //
        // (`ap_passes`' pass 7 is *not* this: it needs the
        // correspondent's callsign, so it is upstream's iaptype 2/3,
        // not 1. `BLIND_CQ_MIN_NSYNC`'s doc comment claims otherwise
        // and is wrong.)
        let mut ap_hints: Vec<(crate::msg::ap::ApHint, u8)> = req
            .ap_hint
            .filter(|h| h.has_info())
            .map(crate::msg::pipeline_ap::ap_passes)
            .unwrap_or_default();
        ap_hints.push((crate::msg::ap::ApHint::new().with_call1("CQ"), 12));
        let ap_owned: Vec<(Vec<u8>, Vec<u8>, u8)> = ap_hints
            .iter()
            .map(|(cfg, pid)| {
                let (m, v) = crate::msg::pipeline_ap::ap_bits_for::<Ft4>(cfg);
                (m, v, *pid)
            })
            .collect();
        let ap: Vec<(&[u8], &[u8], u8)> = ap_owned
            .iter()
            .map(|(m, v, pid)| (m.as_slice(), v.as_slice(), *pid))
            .collect();
        let accept = crate::msg::decode_request::PolicyAccept::<Ft4, Pol>::new(&req.policy);
        let (raw, fft_cache, budget) = pipeline::decode_frame_budgeted::<Ft4, _>(
            req.audio,
            &FT4_DOWNSAMPLE,
            req.freq_min,
            req.freq_max,
            req.sync_min,
            req.freq_hint,
            req.depth,
            req.max_cand,
            req.strictness,
            req.eq_mode,
            SYNC_Q_MIN,
            req.fft_cache.as_ref().map(FftCache::as_slice),
            on_result,
            req.budget,
            &ap,
            &accept,
        );
        DecodeOutcome {
            results: pipeline::dedup_known(raw, req.known),
            fft_cache,
            budget,
        }
    }
}

impl crate::msg::decode_request::SupportsWideBandAp for Ft4 {}

/// FT4 reaches the message-text stage through the generic pipeline's
/// [`InfoAccept`] seam rather than an engine of its own, so both of its
/// strategies carry the policy and either can be rebuilt for a new one.
///
/// [`InfoAccept`]: crate::engine::pipeline::InfoAccept
impl SupportsMessageFilter for Ft4 {
    fn __strategy_for<Pol: MessagePolicy>(
        tag: StrategyTag,
    ) -> fn(&DecodeRequest<'_, Self, Pol>) -> DecodeOutcome<Self> {
        match tag {
            StrategyTag::SinglePass => Self::__single_pass::<Pol>,
            StrategyTag::FlatSic => Self::__flat_sic::<Pol>,
            // Unreachable by construction, not by assumption:
            // `.sic_early()` lives on `impl<P: SupportsSicEarly>`, and
            // FT4 does not implement that trait, so nothing can put
            // this tag on an FT4 request.
            StrategyTag::StagedSic => unreachable!("FT4 has no staged-SIC strategy"),
        }
    }
}

impl SupportsSicRounds for Ft4 {
    fn __flat_sic<Pol: MessagePolicy>(req: &DecodeRequest<'_, Self, Pol>) -> DecodeOutcome<Self> {
        // Same rationale as `__single_pass` above — this strategy is
        // held to `on_result`'s *exact-match* contract (sequential
        // SIC), so this gap was a genuine violation, not just an
        // avoidable tightening.
        let filtered_cb = pipeline::known_filtered_on_result(req.known, req.on_result);
        let on_result: Option<&(dyn Fn(&DecodeResult) + Sync)> = filtered_cb
            .as_ref()
            .map(|f| f as &(dyn Fn(&DecodeResult) + Sync));
        let accept = crate::msg::decode_request::PolicyAccept::<Ft4, Pol>::new(&req.policy);
        let (raw, budget) = pipeline::decode_frame_subtract::<Ft4, _>(
            req.audio,
            &FT4_DOWNSAMPLE,
            &FT4_SUBTRACT,
            req.freq_min,
            req.freq_max,
            req.sync_min,
            req.freq_hint,
            req.depth,
            req.max_cand,
            req.strictness,
            req.eq_mode,
            req.sic_rounds,
            SYNC_Q_MIN,
            // lpf_half/end-correction match WSJT-X `subtractft4.f90`:
            // NFILT=1400 (lpf_half=700), no end-correction. See
            // `ft4::subtract::{LPF_HALF_SAMPLES, subtract_signal_lpf,
            // refine_signal_freq}` for the same constants used elsewhere.
            //
            // WSJT-X's own `subtractft4` has no frequency-refine step at
            // all — it subtracts directly at the decoded `f0`. mfsk-core's
            // `refine_freq` call above exists to compensate for this
            // codebase's own `r.freq_hz` being integer-Hz-quantized
            // (`engine::sync2d::ft4_sync_search`'s df search only ever
            // produces integer Hz offsets — see its `idf`/`si` loops), not
            // to replicate anything WSJT-X does. That quantization bounds
            // the true continuous optimum to within ±0.5 Hz of the
            // reported freq, so a ±1.0 Hz radius (with 0.5 Hz margin) is
            // sufficient — the previous ±5.0 Hz was carried over from
            // `coarse_sync`'s ~2.93 Hz FFT-bin uncertainty, a different
            // (and much coarser) mechanism `ft4_coarse_sync` replaced for
            // FT4 back in `FT4_BENCHMARK.md` section 13, without this
            // radius being re-derived for the new, tighter bound (issue
            // #182 follow-up). Cuts `refine_freq`'s 0.1 Hz grid search from
            // 101 to 21 evaluations per call — the dominant remaining cost
            // in the subtract engine after the NCO fix (~16 ms/call ×
            // 14 real decodes ≈ 227 ms of the golden WAV's 280 ms total).
            700,
            false,
            1.0,
            req.fft_cache.as_ref().map(FftCache::as_slice),
            on_result,
            req.budget,
            &accept,
        );
        // Multi-pass SIC has no single "the" cache (residual changes every
        // pass) — rebuild from the original audio, matching the shape
        // `decode_frame_with_cache` used to return pre-#191.
        let fft_cache = FftCache(crate::engine::dsp::downsample::build_fft_cache(
            req.audio,
            &FT4_DOWNSAMPLE,
        ));
        DecodeOutcome {
            results: pipeline::dedup_known(raw, req.known),
            fft_cache,
            budget,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Pins `Ft4`'s `GenericPipelineProtocol::snr_db` override to the
    /// real-formula `ft4_snr_db` (issue #255) — a direct, non-flaky
    /// check that FT4 reports SNR through the *same* function whichever
    /// rung of the ladder produced the decode. When AP lived in its own
    /// parallel engine that engine called the generic adjacent-tone
    /// `compute_snr_db` directly (a missed 4th call site left over from
    /// a pre-trait ad-hoc fix); the engine is gone, but the trait
    /// override it bypassed is what this pins, without needing a full
    /// synthetic decode (whose reported SNR is also sensitive to
    /// search-bandwidth-dependent candidate scoring, an unrelated
    /// confound).
    #[test]
    fn snr_db_dispatches_to_ft4_formula() {
        let cs: [num_complex::Complex<f32>; 0] = [];
        let itone: [u8; 0] = [];
        let fft_cache: [num_complex::Complex<f32>; 0] = [];
        for &cand_score in &[0.5f32, 1.0, 1.5, 3.0, 10.0] {
            let via_trait = <Ft4 as pipeline::GenericPipelineProtocol>::snr_db(pipeline::SnrCtx {
                cs: &cs,
                itone: &itone,
                cd0: &[],
                ds_rate_hz: 0.0,
                cand_score,
                cand_freq_hz: 1000.0,
                fft_cache: &fft_cache,
                ds_cfg: &FT4_DOWNSAMPLE,
                refined_freq_hz: 1000.0,
                i_start: 0,
            });
            assert_eq!(via_trait, pipeline::ft4_snr_db(cand_score));
        }
    }
}
