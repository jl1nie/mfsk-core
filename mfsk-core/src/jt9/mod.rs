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
//! existing [`crate::core::SyncMode::Block`] variant by expressing
//! each sync symbol as a length-1 [`crate::core::SyncBlock`]; no new
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

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
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

pub use interleave::{deinterleave, deinterleave_llrs, interleave};
pub use rx::{demodulate_aligned, demodulate_aligned_variant};
pub use search::{SearchParams, SyncCandidate, coarse_search, coarse_search_variant};
pub use sync_pattern::{JT9_ISYNC, JT9_SYNC_BLOCKS, JT9_SYNC_POSITIONS};
pub use tx::{
    encode_channel_symbols, synthesize_audio, synthesize_audio_variant,
    synthesize_audio_variant_period, synthesize_message_variant_period, synthesize_standard,
    synthesize_standard_variant, synthesize_standard_variant_period,
};

/// JT9 submode letter. Normal JT9 supports A-H; fast JT9 supports E-H.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Jt9Submode {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
}

impl Jt9Submode {
    pub const ALL: [Self; 8] = [
        Self::A,
        Self::B,
        Self::C,
        Self::D,
        Self::E,
        Self::F,
        Self::G,
        Self::H,
    ];
    pub const FAST: [Self; 4] = [Self::E, Self::F, Self::G, Self::H];

    pub const fn index(self) -> u32 {
        match self {
            Self::A => 0,
            Self::B => 1,
            Self::C => 2,
            Self::D => 3,
            Self::E => 4,
            Self::F => 5,
            Self::G => 6,
            Self::H => 7,
        }
    }
}

/// On-air JT9 waveform geometry. Scheduling period is orthogonal to the
/// fast E-H waveform and is carried by the protocol/profile variant.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Jt9Waveform {
    Normal(Jt9Submode),
    Fast(Jt9Submode),
}

impl Jt9Waveform {
    pub fn fast(submode: Jt9Submode) -> Option<Self> {
        if submode.index() >= Jt9Submode::E.index() {
            Some(Self::Fast(submode))
        } else {
            None
        }
    }

    pub const fn symbol_samples_12k(self) -> u32 {
        match self {
            Self::Normal(_) => 6_912,
            Self::Fast(Jt9Submode::E) => 480,
            Self::Fast(Jt9Submode::F) => 240,
            Self::Fast(Jt9Submode::G) => 120,
            Self::Fast(Jt9Submode::H) => 60,
            Self::Fast(_) => 0,
        }
    }

    pub const fn tone_spacing_hz(self) -> f32 {
        match self {
            Self::Normal(submode) => (12_000.0 / 6_912.0) * (1u32 << submode.index()) as f32,
            Self::Fast(_) => 12_000.0 / self.symbol_samples_12k() as f32,
        }
    }
}

/// Top-level convenience: decode a JT9 signal at a known (start_sample,
/// base_freq) and return the recovered message if Fano converges.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<crate::msg::Jt72Message> {
    decode_at_variant(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        Jt9Waveform::Normal(Jt9Submode::A),
    )
}

/// Decode an aligned normal or fast JT9 waveform.
pub fn decode_at_variant(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    variant: Jt9Waveform,
) -> Option<crate::msg::Jt72Message> {
    use crate::core::{DecodeContext, FecCodec, FecOpts, MessageCodec};

    let llrs =
        rx::demodulate_aligned_variant(audio, sample_rate, start_sample, base_freq_hz, variant);
    let codec = ConvFano232;
    let res = codec.decode_soft(&llrs, &FecOpts::default())?;
    let mut payload = [0u8; 72];
    payload.copy_from_slice(&res.info);
    crate::msg::Jt72Codec::default().unpack(&payload, &DecodeContext::default())
}

/// One successful JT9 decode with its alignment info.
#[derive(Clone, Debug)]
pub struct Jt9Decode {
    pub message: crate::msg::Jt72Message,
    pub freq_hz: f32,
    pub start_sample: usize,
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
) -> Vec<Jt9Decode> {
    use crate::core::ModulationParams;
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

    let mut seen: Vec<Jt9Decode> = Vec::new();
    for c in cands {
        let Some(d) = decode::decode_at_baseband_with_fft(&big_fft, c.freq_hz) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == d.message
                && (prev.freq_hz - d.freq_hz).abs() <= 4.0
                && (prev.start_sample as i64 - d.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            seen.push(d);
        }
    }
    seen
}

/// Convenience: scan using [`search::SearchParams::default`].
pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<Jt9Decode> {
    decode_scan(audio, sample_rate, 0, &search::SearchParams::default())
}

/// Scan a slot for any selectable normal A-H or fast E-H waveform.
///
/// JT9A keeps the more sensitive WSJT-X `softsym` path in
/// [`decode_scan`]. This variant-aware scanner is the common acquisition
/// path for the wider and fast waveforms: it searches the known sync-tone
/// positions on a quarter-symbol grid and refines the strongest
/// candidates before running the exact aligned demodulator/Fano decoder.
pub fn decode_scan_variant(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    variant: Jt9Waveform,
    params: &search::SearchParams,
) -> Vec<Jt9Decode> {
    if sample_rate == 0 || variant.symbol_samples_12k() == 0 {
        return Vec::new();
    }
    let symbol_dt = variant.symbol_samples_12k() as f64 / 12_000.0;
    let nsps = (sample_rate as f64 * symbol_dt).round() as usize;
    if nsps == 0 {
        return Vec::new();
    }
    let quarter = (nsps / 4).max(1) as i64;
    let frequency_step = sample_rate as f32 / nsps as f32 / 4.0;
    let time_refinements = [0, -quarter, quarter, -2 * quarter, 2 * quarter];
    let frequency_refinements = [
        0.0,
        -frequency_step,
        frequency_step,
        -2.0 * frequency_step,
        2.0 * frequency_step,
    ];

    let mut decodes = Vec::new();
    for candidate in
        search::coarse_search_variant(audio, sample_rate, nominal_start_sample, variant, params)
    {
        let mut found = None;
        'refine: for time_delta in time_refinements {
            let start = candidate.start_sample as i64 + time_delta;
            if start < 0 {
                continue;
            }
            for frequency_delta in frequency_refinements {
                if let Some(message) = decode_at_variant(
                    audio,
                    sample_rate,
                    start as usize,
                    candidate.freq_hz + frequency_delta,
                    variant,
                ) {
                    found = Some(Jt9Decode {
                        message,
                        freq_hz: candidate.freq_hz + frequency_delta,
                        start_sample: start as usize,
                    });
                    break 'refine;
                }
            }
        }
        if let Some(decode) = found {
            let duplicate = decodes.iter().any(|previous: &Jt9Decode| {
                previous.message == decode.message
                    && (previous.freq_hz - decode.freq_hz).abs() <= variant.tone_spacing_hz()
                    && (previous.start_sample as i64 - decode.start_sample as i64).abs()
                        <= nsps as i64
            });
            if !duplicate {
                decodes.push(decode);
            }
        }
    }
    decodes
}

/// Variant-aware scan with default acquisition settings.
pub fn decode_scan_variant_default(
    audio: &[f32],
    sample_rate: u32,
    variant: Jt9Waveform,
) -> Vec<Jt9Decode> {
    decode_scan_variant(
        audio,
        sample_rate,
        0,
        variant,
        &search::SearchParams::default(),
    )
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

macro_rules! jt9_normal_protocol {
    ($name:ident, $multiplier:literal) => {
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl ModulationParams for $name {
            const NTONES: u32 = 9;
            const BITS_PER_SYMBOL: u32 = 3;
            const NSPS: u32 = 6_912;
            const SYMBOL_DT: f32 = 6_912.0 / 12_000.0;
            const TONE_SPACING_HZ: f32 = (12_000.0 / 6_912.0) * $multiplier as f32;
            const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2, 6, 7, 5, 4];
            const GFSK_BT: f32 = 0.0;
            const GFSK_HMOD: f32 = 1.0;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
            const NSTEP_PER_SYMBOL: u32 = 2;
            const NDOWN: u32 = 8;
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 69;
            const N_SYNC: u32 = 16;
            const N_SYMBOLS: u32 = 85;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&JT9_SYNC_BLOCKS);
            const T_SLOT_S: f32 = 60.0;
            const TX_START_OFFSET_S: f32 = 0.0;
        }

        impl Protocol for $name {
            type Fec = ConvFano232;
            type Msg = Jt72Codec;
            const ID: ProtocolId = ProtocolId::Jt9;
        }
    };
}

jt9_normal_protocol!(Jt9b, 2);
jt9_normal_protocol!(Jt9c, 4);
jt9_normal_protocol!(Jt9d, 8);
jt9_normal_protocol!(Jt9e, 16);
jt9_normal_protocol!(Jt9f, 32);
jt9_normal_protocol!(Jt9g, 64);
jt9_normal_protocol!(Jt9h, 128);

macro_rules! jt9_fast_protocol {
    ($name:ident, $nsps:literal, $period:literal) => {
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl ModulationParams for $name {
            const NTONES: u32 = 9;
            const BITS_PER_SYMBOL: u32 = 3;
            const NSPS: u32 = $nsps;
            const SYMBOL_DT: f32 = $nsps as f32 / 12_000.0;
            const TONE_SPACING_HZ: f32 = 12_000.0 / $nsps as f32;
            const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2, 6, 7, 5, 4];
            const GFSK_BT: f32 = 0.0;
            const GFSK_HMOD: f32 = 1.0;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
            const NSTEP_PER_SYMBOL: u32 = 4;
            const NDOWN: u32 = 1;
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 69;
            const N_SYNC: u32 = 16;
            const N_SYMBOLS: u32 = 85;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&JT9_SYNC_BLOCKS);
            const T_SLOT_S: f32 = $period as f32;
            const TX_START_OFFSET_S: f32 = 0.0;
        }

        impl Protocol for $name {
            type Fec = ConvFano232;
            type Msg = Jt72Codec;
            const ID: ProtocolId = ProtocolId::Jt9;
        }
    };
}

jt9_fast_protocol!(Jt9eFast5, 480, 5);
jt9_fast_protocol!(Jt9fFast5, 240, 5);
jt9_fast_protocol!(Jt9gFast5, 120, 5);
jt9_fast_protocol!(Jt9hFast5, 60, 5);
jt9_fast_protocol!(Jt9eFast10, 480, 10);
jt9_fast_protocol!(Jt9fFast10, 240, 10);
jt9_fast_protocol!(Jt9gFast10, 120, 10);
jt9_fast_protocol!(Jt9hFast10, 60, 10);
jt9_fast_protocol!(Jt9eFast15, 480, 15);
jt9_fast_protocol!(Jt9fFast15, 240, 15);
jt9_fast_protocol!(Jt9gFast15, 120, 15);
jt9_fast_protocol!(Jt9hFast15, 60, 15);
jt9_fast_protocol!(Jt9eFast30, 480, 30);
jt9_fast_protocol!(Jt9fFast30, 240, 30);
jt9_fast_protocol!(Jt9gFast30, 120, 30);
jt9_fast_protocol!(Jt9hFast30, 60, 30);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::FecCodec;

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

    #[test]
    fn all_selectable_waveform_geometries_round_trip() {
        let normal = Jt9Submode::ALL.map(Jt9Waveform::Normal);
        let fast = Jt9Submode::FAST
            .map(|submode| Jt9Waveform::fast(submode).expect("E-H are fast-capable"));
        for variant in normal.into_iter().chain(fast) {
            let audio =
                synthesize_standard_variant("CQ", "K1ABC", "FN42", 12_000, 500.0, 0.3, variant)
                    .expect("pack");
            let decoded = decode_at_variant(&audio, 12_000, 0, 500.0, variant)
                .expect("clean JT9 waveform must decode");
            assert_eq!(decoded.to_string(), "CQ K1ABC FN42");
        }
    }

    #[test]
    fn variant_scanner_acquires_wide_and_fast_waveforms() {
        for variant in [
            Jt9Waveform::Normal(Jt9Submode::H),
            Jt9Waveform::Fast(Jt9Submode::E),
            Jt9Waveform::Fast(Jt9Submode::H),
        ] {
            let prefix = (variant.symbol_samples_12k() / 2) as usize;
            let signal =
                synthesize_standard_variant("CQ", "K1ABC", "FN42", 12_000, 500.0, 0.3, variant)
                    .expect("pack");
            let mut audio = vec![0.0; prefix];
            audio.extend(signal);
            let params = search::SearchParams {
                freq_min_hz: 490.0,
                freq_max_hz: 510.0,
                time_tolerance_symbols: 2,
                score_threshold: 0.05,
                max_candidates: 12,
            };
            let decodes = decode_scan_variant(&audio, 12_000, 0, variant, &params);
            assert!(
                decodes
                    .iter()
                    .any(|decode| decode.message.to_string() == "CQ K1ABC FN42"),
                "failed to scan {variant:?}: {decodes:?}",
            );
        }
    }
}
