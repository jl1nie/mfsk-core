//! # `jt9` — JT9 decoder and synthesiser
//!
//! JT9 is a 9-FSK mode (8 data tones plus 1 sync tone at tone 0) with
//! a 60-second slot, plain FSK shaping, convolutional r=½ K=32 FEC
//! with Fano decoding, and the 72-bit JT message payload shared with
//! JT65. Since the FEC polynomials are identical to WSPR's
//! (`crate::fec::conv::fano::POLY1`/`POLY2`), the Fano decoder body
//! is reused unchanged via [`crate::fec::ConvFano232`] — only the
//! code dimensions differ (72 info + 31 tail → 206 coded bits).
//!
//! Sync is carried by 16 symbols at fixed positions in the 85-symbol
//! frame, each expected on tone 0. That distribution fits the
//! existing [`crate::engine::SyncMode::Block`] variant by expressing
//! each sync symbol as a length-1 [`crate::engine::SyncBlock`]; no new
//! `SyncMode` variant is required.
//!
//! References:
//! - WSJT-X `lib/jt9_decode.f90`, `lib/jt9sync.f90`, `lib/conv232.f90`,
//!   `lib/fano232.f90`, `lib/interleave9.f90`
//!
//! ## Quick example
//!
//! ```no_run
//! use mfsk_core::jt9::decode_scan_default;
//!
//! # let audio: Vec<f32> = vec![];
//! // `audio` is 720_000 f32 samples at 12 kHz (60 s slot).
//! for r in decode_scan_default(&audio, 12_000) {
//!     println!("{:+7.1} Hz  start={:>8} sample  {}",
//!              r.freq_hz, r.start_sample, r.message);
//! }
//! ```

use crate::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::ConvFano232;
use crate::msg::Jt72Codec;

pub mod baseband;
pub(crate) mod decode;
pub(crate) mod demod_bb;
pub mod interleave;
pub mod rx;
pub mod search;
pub(crate) mod softsym;
pub mod sync_pattern;
pub mod tx;

pub use decode::Jt9Depth;
pub use interleave::{deinterleave, deinterleave_llrs, interleave};
pub use rx::demodulate_aligned;
pub use search::{SearchParams, SyncCandidate, coarse_search};
pub use sync_pattern::{JT9_ISYNC, JT9_SYNC_BLOCKS, JT9_SYNC_POSITIONS};
pub use tx::{encode_channel_symbols, synthesize_audio, synthesize_standard};

/// Top-level convenience: decode a JT9 signal at a known (start_sample,
/// base_freq) and return the recovered message if Fano converges.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<crate::msg::Jt72Message> {
    use crate::engine::{DecodeContext, FecCodec, FecOpts, MessageCodec};

    let llrs = rx::demodulate_aligned(audio, sample_rate, start_sample, base_freq_hz);
    let codec = ConvFano232;
    let res = codec.decode_soft(&llrs, &FecOpts::default())?;
    let mut payload = [0u8; 72];
    payload.copy_from_slice(&res.info);
    crate::msg::Jt72Codec::default().unpack(&payload, &DecodeContext::default())
}

/// One successful JT9 decode with its alignment info.
#[derive(Clone, Debug)]
pub struct Jt9Result {
    pub message: crate::msg::Jt72Message,
    pub freq_hz: f32,
    pub start_sample: usize,
    /// Decode-side SNR estimate in dB, in WSJT-X's own displayed
    /// convention — a faithful port of `symspec2.f90:52-54`
    /// (`snrdb = db(max(1, sig-1)) - 61.3`), the tail of the very
    /// subroutine `softsym.rs`'s `symspec2_from_ss2` already ported.
    /// Verified against a real local `jt9` build on
    /// `WSJT-X/samples/JT9/130418_1742.wav`: within +0.33…+2.86 dB
    /// (mean +1.3 dB) across that file's four strongest decodes, the
    /// residual being largest on the strongest signal. See
    /// `tests/jt9_wsjtx_samples.rs::jt9_wsjtx_sample_snr_matches_real_jt9`.
    ///
    /// Before issue #255 this field carried a generic signal/noise
    /// power ratio that was documented as *not* WSJT-X-referenced and
    /// read **+31.8 dB high on average** on that same file.
    pub snr_db: f32,
}

/// Scan an audio buffer for any JT9 frames: runs coarse (freq, time)
/// search via [`search::coarse_search`] and uses the WSJT-X-faithful
/// `softsym` pipeline (`downsam9` + `peakdt9` + `symspec2`) on each
/// candidate in score order, collapsing duplicates that decode to the
/// same message within ±4 Hz / ±1 symbol.
pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
) -> Vec<Jt9Result> {
    decode_scan_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        Jt9Depth::default(),
        None,
    )
}

/// [`decode_scan`] with an explicit [`Jt9Depth`] instead of the crate
/// default — trades candidate-loop CPU cost for extra sensitivity on
/// candidates that don't converge (a real signal converges in
/// microseconds regardless of depth, so this only costs more on a
/// busy/noisy band). See [`Jt9Depth`]'s own doc comment for the
/// measured tradeoff and WSJT-X's own `-d`/"Decode Again" precedent
/// this mirrors.
pub fn decode_scan_with_depth(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    depth: Jt9Depth,
) -> Vec<Jt9Result> {
    decode_scan_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        depth,
        None,
    )
}

/// Streaming variant of [`decode_scan`]: fires `on_result` once per
/// candidate as it's accepted, *in addition to* (not instead of) the
/// returned `Vec` — purely additive, same shape as
/// [`crate::msg::decode_request::DecodeRequest::on_result`] (see that
/// method's doc comment and `docs/reference/LIBRARY.md`'s "public
/// decode entry point" section for the full portability rationale).
///
/// A `_streaming` sibling rather than a new parameter on
/// [`decode_scan`] itself, matching
/// `ft8::decode_block::decode_block_streaming`'s precedent — bolting
/// a parameter onto an existing plain `pub fn` is a breaking change.
///
/// **Delivery order/dedup contract**: `decode_scan`'s candidate loop
/// is sequential with no early exit and no parallelism (unlike WSPR's
/// `decode_scan`, which uses `rayon::par_iter()`) — `cb` fires exactly
/// once per result that ends up in the returned `Vec`, in the same
/// order. No divergence mechanism exists here. Candidates are tried in
/// coarse-score-descending order (same `cands.sort_unstable_by`
/// below), so `cb` also tends to favor stronger signals first, same
/// correlation-not-guarantee caveat documented on
/// [`crate::msg::decode_request::DecodeRequest::on_result`].
pub fn decode_scan_streaming(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    on_result: &(dyn Fn(&Jt9Result) + Sync),
) -> Vec<Jt9Result> {
    decode_scan_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        Jt9Depth::default(),
        Some(on_result),
    )
}

/// [`decode_scan_streaming`] with an explicit [`Jt9Depth`] — see
/// [`decode_scan_with_depth`].
pub fn decode_scan_streaming_with_depth(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    depth: Jt9Depth,
    on_result: &(dyn Fn(&Jt9Result) + Sync),
) -> Vec<Jt9Result> {
    decode_scan_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        depth,
        Some(on_result),
    )
}

fn decode_scan_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    depth: Jt9Depth,
    on_result: Option<&(dyn Fn(&Jt9Result) + Sync)>,
) -> Vec<Jt9Result> {
    use crate::engine::ModulationParams;
    let nsps = (sample_rate as f32 * <Jt9 as ModulationParams>::SYMBOL_DT).round() as usize;

    // Collect all coarse candidates above a very low threshold (the score
    // formula saturates near 1.0 for real JT9 signals regardless of SNR,
    // so threshold filtering is not effective here).
    let mut scan_params = *params;
    scan_params.score_threshold = 0.001;
    scan_params.max_candidates = 50_000; // no practical cap; NMS below limits processing

    let mut cands = search::coarse_search(audio, sample_rate, nominal_start_sample, &scan_params);

    // Sort by coarse score so high-quality candidates get tried first.
    // Duplicate-signal suppression happens after decode via `seen`
    // (message + freq ± 4 Hz + time ± 1 symbol), mirroring WSJT-X
    // `lib/jt9_decode.f90:157-163` which marks ±22 freq bins `done`
    // around each successful decode rather than pre-filtering by NMS.
    cands.sort_unstable_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    cands.truncate(params.max_candidates.max(32));

    // Build the big FFT once for the whole slot — `downsam9` extracts
    // one baseband per candidate frequency from this cached spectrum.
    let big_fft = softsym::AudioFft::build(audio);

    let mut seen: Vec<Jt9Result> = Vec::new();
    for c in cands {
        let Some(d) = decode::decode_at_baseband_with_fft_depth(&big_fft, c.freq_hz, depth) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == d.message
                && (prev.freq_hz - d.freq_hz).abs() <= 4.0
                && (prev.start_sample as i64 - d.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            if let Some(cb) = on_result {
                cb(&d);
            }
            seen.push(d);
        }
    }
    seen
}

/// Convenience: scan using [`search::SearchParams::default`].
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<Jt9Result> {
    decode_scan(audio, sample_rate, 0, &search::SearchParams::default())
}

/// JT9 protocol marker.
#[derive(Copy, Clone, Debug, Default)]
pub struct Jt9;

impl ModulationParams for Jt9 {
    const NTONES: u32 = 9;
    const BITS_PER_SYMBOL: u32 = 3; // 8 data tones + 1 sync
    /// Samples per symbol at the 12 kHz pipeline rate. 6912 gives a
    /// baud rate of 12 000 / 6912 ≈ 1.736 Hz, matching WSJT-X.
    const NSPS: u32 = 6912;
    const SYMBOL_DT: f32 = 6912.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 6912.0; // ≈ 1.736 Hz
    /// Data tones are 1..=8; Gray-map the 3 data bits within those
    /// eight tones. Tone 0 is reserved for sync and isn't part of
    /// the data constellation, so the Gray map has 8 entries, not 9.
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2, 6, 7, 5, 4];
    /// No Gaussian shaping — JT9 is plain (square) FSK. Value `0.0`
    /// signals "no GFSK" to TX synthesisers that check the constant.
    const GFSK_BT: f32 = 0.0;
    const GFSK_HMOD: f32 = 1.0;
    /// Two FFTs per symbol window — standard convention (same as FT8).
    const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
    /// Half-symbol coarse-sync step.
    const NSTEP_PER_SYMBOL: u32 = 2;
    /// 12 000 / 8 = 1500 Hz baseband. Adequate for the 9-tone
    /// constellation (9 × 1.736 ≈ 15.6 Hz occupied) plus guard.
    const NDOWN: u32 = 8;
}

impl FrameLayout for Jt9 {
    const N_DATA: u32 = 69;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 85;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Block(&JT9_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 60.0;
    /// JT9 transmissions start at the top of the minute (0 s into the
    /// slot). `tx_start` is 0 rather than WSPR's 1 s.
    const TX_START_OFFSET_S: f32 = 0.0;
}

impl Protocol for Jt9 {
    /// Convolutional r=½ K=32 with Layland-Lushbaugh polynomials —
    /// same as WSPR, different code dimensions (K=72, N=206).
    type Fec = ConvFano232;
    /// 72-bit message payload, shared with JT65.
    type Msg = Jt72Codec;
    const ID: ProtocolId = ProtocolId::Jt9;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::FecCodec;

    #[test]
    fn jt9_trait_surface() {
        assert_eq!(<Jt9 as ModulationParams>::NTONES, 9);
        assert_eq!(<Jt9 as ModulationParams>::BITS_PER_SYMBOL, 3);
        assert_eq!(<Jt9 as ModulationParams>::NSPS, 6912);
        assert!((<Jt9 as ModulationParams>::SYMBOL_DT - 0.576).abs() < 1e-3,);
        assert_eq!(<Jt9 as FrameLayout>::N_SYMBOLS, 85);
        assert_eq!(<Jt9 as FrameLayout>::N_SYNC, 16);
        assert_eq!(<Jt9 as FrameLayout>::N_DATA, 69);
        assert_eq!(<Jt9 as FrameLayout>::T_SLOT_S, 60.0);

        match <Jt9 as FrameLayout>::SYNC_MODE {
            SyncMode::Block(blocks) => {
                assert_eq!(blocks.len(), 16);
                assert_eq!(blocks[0].start_symbol, 0);
                assert_eq!(blocks[15].start_symbol, 84);
                for b in blocks {
                    assert_eq!(b.pattern, &[0u8]);
                }
            }
            SyncMode::Interleaved { .. } => panic!("JT9 must use Block sync"),
        }

        assert_eq!(<<Jt9 as Protocol>::Fec as FecCodec>::N, 206);
        assert_eq!(<<Jt9 as Protocol>::Fec as FecCodec>::K, 72);
    }

    /// `decode_scan_with_depth` wiring sanity — every depth tier must
    /// still recover a clean, strong synthetic signal (a real signal
    /// converges in Fano almost instantly regardless of cycle budget,
    /// see `Jt9Depth`'s own doc comment; this only checks the plumbing
    /// reaches `ConvFano232`, not sensitivity — that's the AWGN
    /// sweep's job).
    #[test]
    fn decode_scan_with_depth_finds_clean_signal_at_every_tier() {
        let freq = 1500.0;
        let audio =
            tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("pack+synth");
        let params = search::SearchParams::default();
        for depth in [
            Jt9Depth::Fast,
            Jt9Depth::Normal,
            Jt9Depth::Deep,
            Jt9Depth::Max,
        ] {
            let decodes = decode_scan_with_depth(&audio, 12_000, 0, &params, depth);
            assert!(
                !decodes.is_empty(),
                "depth={depth:?} found no decodes on a clean synthetic signal"
            );
        }
    }

    #[test]
    #[ignore = "manual diagnostic — phase breakdown probe for JT9's unexplained decode_scan cost"]
    fn phase_breakdown_diag() {
        use std::time::Instant;

        fn load_wav(path: &str) -> Vec<f32> {
            let bytes = std::fs::read(path).unwrap();
            let mut i = 12;
            loop {
                let id = &bytes[i..i + 4];
                let len =
                    u32::from_le_bytes([bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]])
                        as usize;
                if id == b"data" {
                    let start = i + 8;
                    let samples: &[u8] = &bytes[start..start + len];
                    return samples
                        .chunks_exact(2)
                        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                        .collect();
                }
                i += 8 + len + (len % 2);
            }
        }

        let files = [
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../embedded-poc/assets/jt9_sweep/jt9_awgn_m05_01.wav"
            ),
            concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../embedded-poc/assets/jt9_sweep/jt9_awgn_m05_02.wav"
            ),
        ];
        for path in files {
            if !std::path::Path::new(path).exists() {
                eprintln!("skipping {path} — sample not found (gitignored local corpus)");
                continue;
            }
            let mut audio = load_wav(path);
            audio.resize(720_000, 0.0);
            let n = 10;

            let t0 = Instant::now();
            let mut n_cands = 0;
            for _ in 0..n {
                let sp = search::SearchParams {
                    score_threshold: 0.001,
                    max_candidates: 50_000,
                    ..Default::default()
                };
                let mut c = search::coarse_search(&audio, 12000, 0, &sp);
                c.truncate(32);
                n_cands = c.len();
                std::hint::black_box(&c);
            }
            let search_elapsed = t0.elapsed();

            let t0 = Instant::now();
            for _ in 0..n {
                let fft = softsym::AudioFft::build(&audio);
                std::hint::black_box(&fft);
            }
            let bigfft_elapsed = t0.elapsed();

            let t0 = Instant::now();
            let mut last_len = 0;
            for _ in 0..n {
                let r = decode_scan_default(&audio, 12000);
                last_len = r.len();
            }
            let total_elapsed = t0.elapsed();

            let search_ms = search_elapsed.as_secs_f64() * 1000.0 / n as f64;
            let bigfft_ms = bigfft_elapsed.as_secs_f64() * 1000.0 / n as f64;
            let total_ms = total_elapsed.as_secs_f64() * 1000.0 / n as f64;
            eprintln!(
                "{path}: candidates={n_cands} decodes={last_len} coarse_search={search_ms:.2}ms \
                 big_fft_build={bigfft_ms:.2}ms total={total_ms:.2}ms \
                 per_candidate_loop(rest)={:.2}ms",
                total_ms - search_ms - bigfft_ms
            );
        }
    }

    /// Manual diagnostic: on the real busy-band golden WAV with the
    /// real production `SearchParams` (200 max candidates), where does
    /// the per-candidate loop's time actually go? Follow-up to
    /// `phase_breakdown_diag` above — that probe used a simpler
    /// low-candidate-count AWGN file where the once-per-scan big FFT
    /// dominates; on this golden file the candidate loop itself is the
    /// large majority of `decode_scan`'s wall time (per
    /// `docs/notes/BENCHMARKS.md`'s 2026-08-08 note: ~300ms total,
    /// only ~11ms in search+big-FFT). Breaks each candidate down into
    /// its four pipeline stages (`downsam9`+`peakdt9` / `afc9`+
    /// `twkfreq_poly` / `llrs_from_c5` / `ConvFano232::decode_soft`),
    /// and counts how many candidates each gate rejects vs. lets
    /// through, to find where the real cost concentrates.
    #[test]
    #[ignore = "manual diagnostic — JT9 candidate-loop stage breakdown on the real golden WAV"]
    fn candidate_loop_stage_diag() {
        use std::time::Instant;

        use crate::engine::{DecodeContext, FecCodec, FecOpts, MessageCodec};
        use crate::fec::ConvFano232;
        use crate::msg::Jt72Codec;

        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        );
        let bytes = std::fs::read(path).unwrap();
        let dl = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
        let audio: Vec<f32> = bytes[44..44 + dl]
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0)
            .collect();

        let sp = search::SearchParams {
            freq_min_hz: 1050.0,
            freq_max_hz: 1550.0,
            time_tolerance_symbols: 3,
            score_threshold: 0.05,
            max_candidates: 200,
        };
        let mut cands = search::coarse_search(&audio, 12_000, 0, &sp);
        cands.sort_unstable_by(|a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        cands.truncate(sp.max_candidates.max(32));

        let big_fft = softsym::AudioFft::build(&audio);

        let (mut t_downsam_peak, mut t_afc, mut t_llrs, mut t_fano) = (0f64, 0f64, 0f64, 0f64);
        let (mut t_fano_converged, mut t_fano_failed) = (0f64, 0f64);
        let (mut n_total, mut n_sync_pass, mut n_schk_pass, mut n_fano_converge, mut n_msg_ok) =
            (0usize, 0usize, 0usize, 0usize, 0usize);

        for c in &cands {
            n_total += 1;
            if c.freq_hz <= 0.0 {
                continue;
            }

            let t0 = Instant::now();
            let c2 = big_fft.downsam9(c.freq_hz);
            let (_lagpk, sync_score, mut c3) = softsym::peakdt9(&c2);
            t_downsam_peak += t0.elapsed().as_secs_f64() * 1000.0;
            if !sync_score.is_finite() || sync_score < 1.5 {
                continue;
            }
            n_sync_pass += 1;

            let t0 = Instant::now();
            let afc = softsym::afc9(&mut c3);
            softsym::twkfreq_poly(&mut c3, [afc.a0, afc.a1, 0.0]);
            t_afc += t0.elapsed().as_secs_f64() * 1000.0;
            let sync = (afc.syncpk + 1.0) / 4.0;
            if !sync.is_finite() || sync < 1.0 {
                continue;
            }

            let t0 = Instant::now();
            let (schk, llrs, _snr_db) = softsym::llrs_from_c5(&c3);
            t_llrs += t0.elapsed().as_secs_f64() * 1000.0;
            if !schk.is_finite() || schk < 1.5 {
                continue;
            }
            n_schk_pass += 1;

            let t0 = Instant::now();
            let res = ConvFano232.decode_soft(&llrs, &FecOpts::default());
            let this_fano_ms = t0.elapsed().as_secs_f64() * 1000.0;
            t_fano += this_fano_ms;
            match &res {
                Some(_) => t_fano_converged += this_fano_ms,
                None => t_fano_failed += this_fano_ms,
            }
            let Some(res) = res else { continue };
            n_fano_converge += 1;

            let mut payload = [0u8; 72];
            payload.copy_from_slice(&res.info);
            if Jt72Codec::default()
                .unpack(&payload, &DecodeContext::default())
                .is_some()
            {
                n_msg_ok += 1;
            }
        }

        eprintln!(
            "candidates: total={n_total} sync_pass={n_sync_pass} schk_pass={n_schk_pass} \
             fano_converge={n_fano_converge} msg_ok={n_msg_ok}"
        );
        eprintln!(
            "stage time (ms): downsam9+peakdt9={t_downsam_peak:.2} afc9+twkfreq={t_afc:.2} \
             llrs_from_c5={t_llrs:.2} fano_decode_soft={t_fano:.2} total={:.2}",
            t_downsam_peak + t_afc + t_llrs + t_fano
        );
        eprintln!(
            "fano_decode_soft split: converged={t_fano_converged:.2}ms (n={n_fano_converge}) \
             failed={t_fano_failed:.2}ms (n={})",
            n_schk_pass - n_fano_converge
        );
    }
}
