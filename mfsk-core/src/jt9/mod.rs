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
//! use mfsk_core::Mode;
//! use mfsk_core::decoder::{Decoder, SlotInput, default_params};
//! use mfsk_core::jt9::Jt9;
//!
//! # let audio: Vec<f32> = vec![];
//! // `audio` is 720_000 f32 samples at 12 kHz (60 s slot).
//! // The mode's defaults, with a band of your own (#711).
//! let mut decoder = Decoder::<Jt9>::new(default_params(Mode::Jt9).band(200.0, 4_000.0));
//! for row in decoder.decode(&SlotInput::f32(&audio)).rows {
//!     let d = &row.decoded;
//!     println!("{:+7.1} Hz  dt={:+.2} s  {}", d.freq_hz, d.dt_sec, d.text);
//! }
//! ```

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
use num_traits::Float;

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
use crate::engine::pipeline::ScanRow;
use crate::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::ConvFano232;
use crate::msg::Jt72Codec;

// Decode-side modules reach their FFT through `engine::fft`, whose own
// modules are gated on the FFT meta-feature; mirror that gate here so
// `--features <mode>` alone still builds. TX and the const tables stay
// unconditional, the same split `wspr::mod` uses.
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub(crate) mod decode;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub(crate) mod decode_request;
pub mod interleave;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod rx;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod search;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub(crate) mod softsym;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub(crate) mod sync9;
pub mod sync_pattern;
#[cfg(all(test, any(feature = "fft-rustfft", feature = "fft-extern")))]
mod test_util;
pub mod tx;

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use decode::Jt9Depth;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use decode_request::SniperRequest;
// The wide-band request is the engine's, reached through `__internal`
// since 0.13: `crate::decoder::Decoder` is the decode API.
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub(crate) use decode_request::DecodeRequest;
pub use interleave::{deinterleave, deinterleave_llrs, interleave};
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use rx::demodulate_aligned;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use search::{SearchParams, SyncCandidate, coarse_search};
pub use sync_pattern::{JT9_ISYNC, JT9_SYNC_BLOCKS, JT9_SYNC_POSITIONS};
pub use tx::{encode_channel_symbols, synthesize_standard};

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// Decode a JT9 signal at a known (start_sample, base_freq) and return
/// the recovered message if Fano converges. Public through
/// [`SniperRequest`].
fn decode_at_inner(
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

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// One successful JT9 decode with its alignment info.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Jt9Result {
    pub message: crate::msg::Jt72Message,
    pub freq_hz: f32,
    /// Sample index of symbol 0, **clamped at 0**: a frame starting
    /// before the buffer origin has no `usize` to land in. Use
    /// [`Self::dt_sec`] when the sign matters.
    pub start_sample: usize,
    /// Time offset from the nominal start, in seconds. Signed, and the
    /// authoritative one — see [`Self::start_sample`] (#397).
    pub dt_sec: f32,
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

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// Scan an audio buffer for JT9 frames the way `jt9_decode.f90:180-283` does: `sync9` scores every
/// 0.732 Hz bin of the band ([`sync9`]), then two passes walk the bins upwards and run `softsym` +
/// Fano at each bin that passes the gates, the first (`nqd = 1`) over `narrow`'s Rx window, the
/// second (`nqd = 0`) over the band. A decode marks bins `i-1..=i+22` done, so a signal is reported
/// once, at the lowest bin that decodes it. Public through [`DecodeRequest`].
///
/// `params` supplies the band only: `sync9`'s lag range is upstream's fixed `-2.5..+5.0` s of the
/// buffer and its gates are `ccflim`/`red2lim`, so `time_tolerance_*`, `score_threshold` and
/// `max_candidates` (the old scan's) are not read (#726).
///
/// Deliberate divergences: there is no `nqd = 1` pass without `narrow` (upstream always has an Rx
/// frequency); [`Jt9Depth::Max`] takes `nagain`'s limits (`limit` 100000, `ccflim` 2.4) but still
/// runs the band pass, where `nagain` stops after the Rx window.
#[allow(clippy::too_many_arguments)]
fn decode_scan_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    depth: Jt9Depth,
    narrow: Option<(f32, f32)>,
    on_result: Option<&(dyn Fn(&Jt9Result) + Sync)>,
    gate: &crate::engine::pipeline::BudgetGate<'_>,
) -> Vec<Jt9Result> {
    use sync9::{DF3, bin_range};

    let (ia_band, ib_band) = bin_range(params.freq_min_hz, params.freq_max_hz);
    let Some(s9) = sync9::sync9(audio, ia_band, ib_band) else {
        return Vec::new();
    };
    // `downsam9` reads the whole slot's spectrum; build it once for every bin tried.
    let big_fft = softsym::AudioFft::build(audio);

    // `jt9_decode.f90:199-201`.
    let df8 = 1500.0 / 864.0;
    let red2lim = 1.6;
    let mut done = alloc::vec![false; s9.ib() + 2];
    let mut ccfok = alloc::vec![false; s9.ib() + 2];
    let (mut ia1, mut ib1) = (1usize, 1usize);
    let mut found: Vec<Jt9Result> = Vec::new();

    for nqd in [1u8, 0] {
        // `jt9_decode.f90:204-222`: the Fano limit and `ccflim` by depth; the Rx pass always deep.
        let (limit, ccflim) = match (depth, nqd) {
            (Jt9Depth::Max, _) => (Jt9Depth::Max, 2.4),
            (_, 1) | (Jt9Depth::Deep, _) => (Jt9Depth::Deep, 2.5),
            (Jt9Depth::Normal, _) => (Jt9Depth::Normal, 2.7),
            (Jt9Depth::Fast, _) => (Jt9Depth::Fast, 3.0),
        };
        ccfok.iter_mut().for_each(|b| *b = false);
        let (ia, ib) = if nqd == 1 {
            let Some((f0, tol)) = narrow else { continue };
            let (ia, ib) = bin_range(f0 - tol, f0 + tol);
            let (ia, ib) = (ia.max(s9.ia), ib.min(s9.ib()));
            for i in ia..=ib {
                ccfok[i] = s9.ccfred(i) > ccflim - 2.0 && s9.red2(i) > red2lim - 1.0;
            }
            (ia1, ib1) = (ia, ib);
            (ia, ib)
        } else {
            for i in s9.ia..=s9.ib() {
                ccfok[i] = s9.ccfred(i) > ccflim && s9.red2(i) > red2lim;
            }
            for b in ccfok.iter_mut().take(ib1 + 1).skip(ia1) {
                *b = false;
            }
            (s9.ia, s9.ib())
        };
        if nqd == 0 {
            gate.note_candidates(ccfok.iter().filter(|&&b| b).count());
        }
        let gates = if nqd == 1 {
            decode::Gates::NARROW
        } else {
            decode::Gates::WIDE
        };

        let mut fgood = 0.0f32;
        for i in ia..=ib {
            if done[i] || !ccfok[i] {
                continue;
            }
            let f = (i - 1) as f32 * DF3;
            if !(nqd == 1 || (s9.ccfred(i) >= ccflim && (f - fgood).abs() > 10.0 * df8)) {
                continue;
            }
            // The caller's budget, once per `softsym` call (#593).
            if !gate.proceed() {
                return found;
            }
            let Some(mut d) = decode::decode_at_baseband_gated(&big_fft, f, limit, gates) else {
                continue;
            };
            // The builder reports dt from the buffer origin; dt is defined from the nominal start.
            d.dt_sec -= nominal_start_sample as f32 / sample_rate as f32;
            if let Some(cb) = on_result {
                cb(&d);
            }
            found.push(d);
            fgood = f;
            let iaa = i.saturating_sub(1).max(1);
            let ibb = (i + 22).min(s9.ib() + 1);
            for j in iaa..=ibb {
                ccfok[j] = false;
                done[j] = true;
            }
        }
    }
    found
}

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
impl ScanRow for Jt9Result {
    type Message = crate::msg::Jt72Message;
    fn scan_message(&self) -> &Self::Message {
        &self.message
    }
    fn scan_freq_hz(&self) -> f32 {
        self.freq_hz
    }
    fn scan_start_sample(&self) -> i64 {
        self.start_sample as i64
    }
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
    const TONE_SPACING_HZ: f32 = crate::engine::tone_spacing_hz(Self::GFSK_HMOD, Self::NSPS); // ≈ 1.736 Hz
    /// Data tones are 1..=8; Gray-map the 3 data bits within those
    /// eight tones. Tone 0 is reserved for sync and isn't part of
    /// the data constellation, so the Gray map has 8 entries, not 9.
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2, 6, 7, 5, 4];
    /// No Gaussian shaping — JT9 is plain (square) FSK. Value `0.0`
    /// signals "no GFSK" to TX synthesisers that check the constant.
    const GFSK_BT: f32 = 0.0;
    const GFSK_HMOD: f32 = 1.0;
}

impl crate::engine::tx::FskWaveform for Jt9 {
    const WAVEFORM: crate::engine::tx::Waveform = crate::engine::tx::Waveform::Cpfsk;
}

impl FrameLayout for Jt9 {
    const N_DATA: u32 = 69;
    const N_SYNC: u32 = 16;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Block(&JT9_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 60.0;
    /// DT = 0 is 1.0 s into the slot, as for WSPR: WSJT-X starts JT9 audio there
    /// (`Modulator.cpp:163` `delay_ms=1000`, `jt9sim.f90:124` `k=12000`), and real
    /// `jt9 -9` reports DT 0.0 for a frame placed there. This was 0.0 until #726, so every
    /// JT9 DT this crate reported through `Decoder` was 1.0 s more than `jt9`'s (measured on
    /// `130418_1742.wav`: `jt9` 0.0/0.1/-0.2, this crate +0.90..+1.04).
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Jt9 {
    /// Convolutional r=½ K=32 with Layland-Lushbaugh polynomials —
    /// same as WSPR, different code dimensions (K=72, N=206).
    type Fec = ConvFano232;
    /// 72-bit message payload, shared with JT65.
    type Msg = Jt72Codec;
    type SyncPhasors = ();
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

    /// `DecodeRequest::depth` wiring sanity — every depth tier must
    /// still recover a clean, strong synthetic signal (a real signal
    /// converges in Fano almost instantly regardless of cycle budget,
    /// see `Jt9Depth`'s own doc comment; this only checks the plumbing
    /// reaches `ConvFano232`, not sensitivity — that's the AWGN
    /// sweep's job).
    #[test]
    fn depth_finds_clean_signal_at_every_tier() {
        let freq = 1500.0;
        let audio =
            tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("pack+synth");
        for depth in [
            Jt9Depth::Fast,
            Jt9Depth::Normal,
            Jt9Depth::Deep,
            Jt9Depth::Max,
        ] {
            let decodes = DecodeRequest::new(&audio, 12_000).depth(depth).decode();
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
            let mut audio = super::test_util::load_wav_f32(path);
            audio.resize(720_000, 0.0);
            let n = 10;

            let t0 = Instant::now();
            let mut n_cands = 0;
            for _ in 0..n {
                let sp = search::SearchParams {
                    score_threshold: 0.001,
                    max_candidates: 50_000,
                    ..search::default_search_params()
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
                let r = DecodeRequest::new(&audio, 12000).decode();
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
        let audio = super::test_util::load_wav_f32(path);

        let sp = search::SearchParams {
            freq_min_hz: 1050.0,
            freq_max_hz: 1550.0,
            time_tolerance_early_sec: 1.728,
            time_tolerance_late_sec: 1.728,
            score_threshold: 0.05,
            max_candidates: 200,
        };
        let mut cands = search::coarse_search(&audio, 12_000, 0, &sp);
        cands.sort_unstable_by(|a, b| {
            b.score
                .partial_cmp(&a.score)
                .unwrap_or(core::cmp::Ordering::Equal)
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
