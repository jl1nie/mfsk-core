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
use crate::msg::pipeline_ap;

pub use crate::engine::pipeline::{DecodeDepth, DecodeResult, DecodeStrictness, FftCache};
pub use crate::msg::ApHint;
use crate::msg::decode_request::{
    DecodeOutcome, DecodeRequest, FrameDecodable, SniperRequest, SupportsSicRounds,
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

/// FT4's coarse sync now uses half-symbol (24 ms = 16 downsampled-sample)
/// steps; refine across ±1 symbol (32 samples) still to bridge rounding.
const REFINE_STEPS: i32 = 32;
/// FT4 has 16 sync symbols (4 × 4); require at least half correct.
const SYNC_Q_MIN: u32 = 8;

/// Dedup `raw` against caller-supplied `known` (by `info` equality) — the
/// generic engine has no `known`/AP-hint parameter at all, so this is a
/// best-effort post-filter rather than an in-loop skip. Always correct
/// (never mis-reports a known signal as new), just cannot save the work
/// of re-decoding it the way FT8's engine-level `known` handling can.
fn dedup_known(raw: Vec<DecodeResult>, known: &[DecodeResult]) -> Vec<DecodeResult> {
    raw.into_iter()
        .filter(|r| !known.iter().any(|k| k.info == r.info))
        .collect()
}

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

    fn __single_pass(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
        // See `pipeline::known_filtered_on_result`'s doc comment: without
        // this, `on_result` could fire for a candidate `dedup_known`
        // below then silently drops from the returned `Vec`.
        let filtered_cb = pipeline::known_filtered_on_result(req.known, req.on_result);
        let on_result: Option<&(dyn Fn(&DecodeResult) + Sync)> = filtered_cb
            .as_ref()
            .map(|f| f as &(dyn Fn(&DecodeResult) + Sync));
        let (raw, fft_cache) = pipeline::decode_frame::<Ft4>(
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
        );
        DecodeOutcome {
            results: dedup_known(raw, req.known),
            fft_cache,
        }
    }

    fn __sniper(req: &SniperRequest<'_, Self>) -> DecodeOutcome<Self> {
        // Clamp caller's candidate count: in sniper mode the target is
        // reliably in the top-5 after dedup even at -18 dB, so >15 just
        // burns CPU — especially important under the lite feature
        // defaults where every candidate runs BP + OSD per AP config.
        let max_cand = req.max_cand.min(15);
        let results = pipeline_ap::decode_sniper_ap::<Ft4>(
            req.audio,
            &FT4_DOWNSAMPLE,
            req.target_freq,
            250.0,
            req.sync_min,
            req.depth,
            max_cand,
            req.strictness,
            req.eq_mode,
            REFINE_STEPS,
            // Halve the sync-quality gate for AP: locked bits carry the
            // decision, so weak sync-quality signals may still succeed.
            SYNC_Q_MIN / 2,
            req.ap_hint,
            req.on_result,
        );
        // `pipeline_ap::decode_sniper_ap` doesn't return its FFT cache;
        // sniper mode never exposed one before this redesign either
        // (old `decode_sniper_ap` returned `Vec<DecodeResult>` only), so
        // rebuild it once here purely to satisfy `DecodeOutcome`'s
        // uniform shape.
        let fft_cache = FftCache(crate::engine::dsp::downsample::build_fft_cache(
            req.audio,
            &FT4_DOWNSAMPLE,
        ));
        DecodeOutcome { results, fft_cache }
    }
}

impl SupportsSicRounds for Ft4 {
    fn __flat_sic(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self> {
        // Same rationale as `__single_pass` above — this strategy is
        // held to `on_result`'s *exact-match* contract (sequential
        // SIC), so this gap was a genuine violation, not just an
        // avoidable tightening.
        let filtered_cb = pipeline::known_filtered_on_result(req.known, req.on_result);
        let on_result: Option<&(dyn Fn(&DecodeResult) + Sync)> = filtered_cb
            .as_ref()
            .map(|f| f as &(dyn Fn(&DecodeResult) + Sync));
        let raw = pipeline::decode_frame_subtract::<Ft4>(
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
        );
        // Multi-pass SIC has no single "the" cache (residual changes every
        // pass) — rebuild from the original audio, matching the shape
        // `decode_frame_with_cache` used to return pre-#191.
        let fft_cache = FftCache(crate::engine::dsp::downsample::build_fft_cache(
            req.audio,
            &FT4_DOWNSAMPLE,
        ));
        DecodeOutcome {
            results: dedup_known(raw, req.known),
            fft_cache,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::decode_request::DecodeRequest;

    /// Compile-time check that `DecodeRequest<Ft4>` accepts every `osd`
    /// setting across single-pass, `sic_rounds`, and sniper. No actual
    /// decoding happens — empty audio returns no candidates fast — but
    /// this guards against future signature drift.
    #[test]
    fn decode_request_accepts_all_param_combos() {
        let empty = vec![0i16; 12 * 7500]; // 7.5 s of silence at 12 kHz
        for osd in [false, true] {
            let _ = DecodeRequest::<Ft4>::new(&empty, 100.0, 3000.0, 1.0, 5)
                .osd(osd)
                .decode();
            let _ = DecodeRequest::<Ft4>::new(&empty, 100.0, 3000.0, 1.0, 5)
                .osd(osd)
                .sic_rounds(3)
                .decode();
            let _ = DecodeRequest::<Ft4>::sniper(&empty, 1500.0, 5)
                .osd(osd)
                .decode();
        }
    }

    /// Pins `Ft4`'s `GenericPipelineProtocol::snr_db` override to the
    /// real-formula `ft4_snr_db` (issue #255) — a direct, non-flaky
    /// check that both of FT4's call paths (basic pipeline,
    /// `msg::pipeline_ap`'s AP path) go through the *same* function.
    /// The AP path used to call the generic adjacent-tone
    /// `compute_snr_db` directly instead (a missed 4th call site left
    /// over from a pre-trait ad-hoc fix); that regression would show up
    /// here as this test failing, without needing a full synthetic
    /// decode (whose reported SNR is also sensitive to search-bandwidth
    /// -dependent candidate scoring, an unrelated confound).
    #[test]
    fn snr_db_dispatches_to_ft4_formula() {
        let cs: [num_complex::Complex<f32>; 0] = [];
        let itone: [u8; 0] = [];
        let fft_cache: [num_complex::Complex<f32>; 0] = [];
        for &cand_score in &[0.5f32, 1.0, 1.5, 3.0, 10.0] {
            let via_trait = <Ft4 as pipeline::GenericPipelineProtocol>::snr_db(pipeline::SnrCtx {
                cs: &cs,
                itone: &itone,
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
