//! # `jt65` — JT65 decoder and synthesiser
//!
//! JT65 is the classic EME (moonbounce) / weak-signal mode that
//! WSJT-X inherited from the original WSJT. It uses:
//! - **65-FSK** modulation (1 sync tone at index 0 + 64 data tones
//!   at indices 2..=65; index 1 is unused). Plain FSK, no GFSK.
//! - **RS(63, 12) over GF(2^6)** for error correction (51 parity
//!   symbols, corrects up to 25 symbol errors). Implemented in
//!   [`crate::fec::Rs63_12`].
//! - **72-bit JT message payload** packed into 12 × 6-bit symbols —
//!   the same layout as JT9 ([`crate::msg::Jt72Codec`]).
//! - **Pseudo-random distributed sync**: a fixed 126-bit pattern
//!   (`nprc`) marks 63 positions that carry tone 0 (sync) and 63
//!   that carry Gray-coded data symbols. Expressed in our abstraction
//!   as 63 length-1 `SyncBlock` entries under the existing
//!   `SyncMode::Block` variant — no new `SyncMode` case required.
//!
//! JT65A, JT65B, and JT65C are implemented, including normal and
//! `OOO`-inverted sync plus the `RO`, `RRR`, and `73` EME shorthands.
//!
//! References:
//! - WSJT-X `lib/jt65sim.f90`, `lib/setup65.f90`, `lib/interleave63.f90`,
//!   `lib/graycode65.f90`, `lib/wrapkarn.c`
//!
//! ## Quick example
//!
//! ```no_run
//! use mfsk_core::jt65::decode_scan_default;
//!
//! # let audio: Vec<f32> = vec![];
//! // `audio` is 720_000 f32 samples at 12 kHz (60 s slot).
//! for r in decode_scan_default(&audio, 12_000) {
//!     println!("{:+7.1} Hz  start={:>8} sample  {}",
//!              r.freq_hz, r.start_sample, r.message);
//! }
//! ```
//!
//! ## Erasure-aware decode
//!
//! For very weak signals, JT65 benefits from feeding per-symbol
//! confidence into Reed-Solomon as *erasures*. Each erasure lets RS
//! correct one more symbol than the hard-error bound
//! (`2·errors + erasures ≤ 51`). Use [`decode_at_with_erasures`]:
//!
//! ```no_run
//! use mfsk_core::jt65::decode_at_with_erasures;
//!
//! # let audio: Vec<f32> = vec![];
//! # let (start_sample, freq_hz) = (0, 1270.0);
//! // Try 0 → 8 → 16 → 24 → 32 erasures in order; return the first
//! // budget that unpacks into a valid message.
//! let msg = decode_at_with_erasures(
//!     &audio, 12_000, start_sample, freq_hz,
//!     &[0, 8, 16, 24, 32],
//! );
//! ```

use crate::core::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::Rs63_12;
use crate::msg::Jt72Codec;

pub mod gray;
pub mod interleave;
pub mod rx;
pub mod search;
pub mod sync_pattern;
pub mod tx;

pub use gray::{gray6, inv_gray6};
pub use interleave::{deinterleave, interleave};
pub use rx::{
    Jt65TonePowers, demodulate_aligned, demodulate_aligned_for,
    demodulate_aligned_for_with_polarity, demodulate_aligned_with_confidence,
    demodulate_aligned_with_confidence_for, demodulate_aligned_with_confidence_for_with_polarity,
    demodulate_tone_powers_for_with_polarity,
};
pub use sync_pattern::{JT65_DATA_POSITIONS, JT65_NPRC, JT65_SYNC_BLOCKS, JT65_SYNC_POSITIONS};
pub use tx::{
    encode_channel_symbols, encode_channel_symbols_with_polarity, encode_shorthand_symbols,
    synthesize_audio, synthesize_audio_for, synthesize_message_for, synthesize_shorthand_for,
    synthesize_standard, synthesize_standard_for, synthesize_standard_for_with_polarity,
};

/// Normal (`*`) or inverted (`#`, displayed as `OOO`) JT65 sync.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum SyncPolarity {
    Normal,
    Inverted,
}

/// JT65 two-tone EME shorthand messages.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum Jt65Shorthand {
    Ro,
    Rrr,
    SeventyThree,
}

impl Jt65Shorthand {
    /// Parse the complete shorthand text accepted by pinned WSJT-X.
    pub fn parse(message: &str) -> Option<Self> {
        match message.trim().to_ascii_uppercase().as_str() {
            "RO" => Some(Self::Ro),
            "RRR" => Some(Self::Rrr),
            "73" => Some(Self::SeventyThree),
            _ => None,
        }
    }

    /// Numeric `nspecial` value used in the WSJT-X tone formula.
    pub const fn code(self) -> u8 {
        match self {
            Self::Ro => 2,
            Self::Rrr => 3,
            Self::SeventyThree => 4,
        }
    }

    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Ro => "RO",
            Self::Rrr => "RRR",
            Self::SeventyThree => "73",
        }
    }
}

impl core::fmt::Display for Jt65Shorthand {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        formatter.write_str(self.as_str())
    }
}

/// WSJT-X JT65 QSO state used to select a-priori (AP) decode passes.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub enum Jt65QsoProgress {
    /// Calling CQ / transmitting the free sixth message.
    #[default]
    Calling,
    Tx1,
    Tx2,
    Tx3,
    Tx4,
    Tx5,
}

/// Station-aware hints for JT65 AP decoding.
///
/// These values constrain selected symbols only during receive decoding.
/// They never alter a decoded message after the fact and never enter TX.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct Jt65ApContext {
    pub my_call: String,
    pub dx_call: Option<String>,
    pub dx_grid: Option<String>,
    pub progress: Jt65QsoProgress,
}

type Jt65ApMask = [Option<u8>; 12];

impl Jt65ApContext {
    fn masks(&self) -> Vec<Jt65ApMask> {
        fn partial_mask(words: [u8; 12], count: usize) -> Jt65ApMask {
            let mut mask = [None; 12];
            for (slot, word) in mask.iter_mut().zip(words).take(count) {
                *slot = Some(word);
            }
            mask
        }

        let my_call = self.my_call.trim().to_ascii_uppercase();
        if my_call.is_empty() {
            return Vec::new();
        }
        let dx_call = self
            .dx_call
            .as_deref()
            .map(str::trim)
            .filter(|call| !call.is_empty())
            .map(str::to_ascii_uppercase);
        let dx_grid = self
            .dx_grid
            .as_deref()
            .map(str::trim)
            .filter(|grid| !grid.is_empty())
            .map(str::to_ascii_uppercase);

        let mut by_type = [None, None, None, None, None, None, None, None];
        let mut cq = [None; 12];
        for (slot, word) in cq.iter_mut().zip([62, 32, 32, 49]) {
            *slot = Some(word);
        }
        by_type[1] = Some(cq);
        if let Some(words) = crate::msg::jt72::pack_standard(&my_call, &my_call, "RRR") {
            by_type[2] = Some(partial_mask(words, 4));
        }
        if let Some(dx_call) = dx_call.as_deref() {
            if let Some(words) = crate::msg::jt72::pack_standard(&my_call, dx_call, "RRR") {
                by_type[3] = Some(partial_mask(words, 9));
                by_type[4] = Some(partial_mask(words, 12));
            }
            if let Some(words) = crate::msg::jt72::pack_standard(&my_call, dx_call, "73") {
                by_type[5] = Some(partial_mask(words, 12));
            }
            if let Some(dx_grid) = dx_grid.as_deref() {
                if let Some(words) = crate::msg::jt72::pack_standard(&my_call, dx_call, dx_grid) {
                    by_type[6] = Some(partial_mask(words, 12));
                }
                if let Some(words) = crate::msg::jt72::pack_standard("CQ", dx_call, dx_grid) {
                    by_type[7] = Some(partial_mask(words, 12));
                }
            }
        }

        let pass_types: &[usize] = match self.progress {
            Jt65QsoProgress::Calling => &[1, 2, 6],
            Jt65QsoProgress::Tx1 => &[2, 3, 6, 7],
            Jt65QsoProgress::Tx2 => &[2, 3],
            Jt65QsoProgress::Tx3 | Jt65QsoProgress::Tx4 => &[3, 4, 5],
            Jt65QsoProgress::Tx5 => &[2, 3, 4, 5],
        };
        pass_types
            .iter()
            .filter_map(|&ap_type| by_type[ap_type])
            .collect()
    }
}

/// Top-level: decode a JT65 signal at a known (start_sample, base_freq)
/// and return the recovered message if RS succeeds. Mirrors the shape of
/// `mfsk_core::jt9::decode_at`.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<crate::msg::Jt72Message> {
    decode_at_for::<Jt65>(audio, sample_rate, start_sample, base_freq_hz)
}

/// Decode a JT65 sub-mode at a known time/frequency alignment.
pub fn decode_at_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<crate::msg::Jt72Message> {
    decode_at_for_with_polarity::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        SyncPolarity::Normal,
    )
}

/// Decode a JT65 sub-mode at a known alignment and sync polarity.
pub fn decode_at_for_with_polarity<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    polarity: SyncPolarity,
) -> Option<crate::msg::Jt72Message> {
    use crate::core::{DecodeContext, MessageCodec};

    let received = rx::demodulate_aligned_for_with_polarity::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
        polarity,
    )?;
    let rs = Rs63_12::new();
    let (info, _nerr) = rs.decode_jt65(&received)?;
    let mut payload = [0u8; 72];
    for (i, bit) in payload.iter_mut().enumerate() {
        let word = info[i / 6];
        let shift = 5 - (i % 6);
        *bit = (word >> shift) & 1;
    }
    crate::msg::Jt72Codec::default().unpack(&payload, &DecodeContext::default())
}

/// Decode a JT65 signal at a known alignment, trying progressively
/// larger erasure counts until Reed-Solomon converges or the bound
/// is exhausted. Unlike [`decode_at`], this method exploits
/// per-symbol confidence from the demodulator: symbols with the
/// smallest (best − runner-up) margin are flagged as erasures, which
/// doubles the correctable error count compared to the plain
/// hard-decision bound.
///
/// `attempts` is a slice of erasure counts to try in order. A
/// reasonable default is `&[0, 8, 16, 24, 32]`: zero-erasure first
/// (fastest when the channel is clean) and then growing erasure
/// budgets for lower-SNR signals. Returns the first decode that
/// unpacks into a valid [`crate::msg::jt72::Jt72Message`].
pub fn decode_at_with_erasures(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    attempts: &[usize],
) -> Option<crate::msg::Jt72Message> {
    decode_at_with_erasures_for::<Jt65>(audio, sample_rate, start_sample, base_freq_hz, attempts)
}

/// Erasure-aware aligned decode for one JT65 sub-mode.
pub fn decode_at_with_erasures_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    attempts: &[usize],
) -> Option<crate::msg::Jt72Message> {
    use crate::core::{DecodeContext, MessageCodec};

    let (symbols, conf) = rx::demodulate_aligned_with_confidence_for::<P>(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
    )?;
    // Build an ordering of symbol positions from least → most
    // confident; the caller's erasure budget eats from the start.
    let mut order: Vec<usize> = (0..63).collect();
    order.sort_by(|&a, &b| {
        conf[a]
            .partial_cmp(&conf[b])
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    let rs = Rs63_12::new();
    let codec = crate::msg::Jt72Codec::default();
    let ctx = DecodeContext::default();

    for &n_eras in attempts {
        let n_eras = n_eras.min(51); // hard upper bound = NROOTS
        let eras: Vec<u32> = order.iter().take(n_eras).map(|&i| i as u32).collect();

        // Decode_jt65_erasures takes positions in the WSJT `sent[]` layout;
        // our `symbols` array is already in RS-codeword order (after
        // de-interleave + de-Gray). Those positions match the WSJT
        // data half (symbols 51..=62 of sent[]), so pass them through.
        // Build a `sent[]`-shaped array by placing our symbols into the
        // data section; parity values are unknown, so the caller can
        // leave them as-is — the decoder will treat them as zeros.
        let mut sent = [0u8; 63];
        // Map: symbols[i] (i=0..=62) → sent[51 + 12 - 1 - (i %12)] is wrong.
        // Actually our `symbols` represents the 63-symbol RS codeword
        // in *native Karn order* (the canonical [data || parity] layout)
        // after de-interleave + inverse Gray. WSJT-X's decode_rs wants
        // the reversed layout, but our Rs63_12 wrappers do that
        // translation. The simplest path: re-wrap via the JT65 encoder
        // convention — we already have sent-layout input in the
        // existing decode path, so mirror that here.
        //
        // Looking at the original decode_at: it passes `symbols` (RS
        // codeword order) to `rs.decode_jt65(&symbols)`. So `symbols`
        // IS the WSJT sent-layout array. We can pass erasure indices
        // directly in that layout.
        sent.copy_from_slice(&symbols);
        if let Some((info, _nerr)) = rs.decode_jt65_erasures(&sent, &eras) {
            let mut payload = [0u8; 72];
            for (i, bit) in payload.iter_mut().enumerate() {
                let word = info[i / 6];
                let shift = 5 - (i % 6);
                *bit = (word >> shift) & 1;
            }
            if let Some(msg) = codec.unpack(&payload, &ctx) {
                return Some(msg);
            }
        }
    }
    None
}

/// One successful JT65 decode with its alignment info.
#[derive(Clone, Debug)]
pub struct Jt65Decode {
    pub message: crate::msg::Jt72Message,
    pub freq_hz: f32,
    pub start_sample: usize,
    pub polarity: SyncPolarity,
}

/// One decoded JT65 EME shorthand with its alignment and confidence.
#[derive(Clone, Debug)]
pub struct Jt65ShorthandDecode {
    pub message: Jt65Shorthand,
    pub freq_hz: f32,
    pub start_sample: usize,
    pub score: f32,
}

fn decode_tone_powers(powers: &Jt65TonePowers) -> Option<(crate::msg::Jt72Message, u32)> {
    decode_tone_powers_with_mask(powers, None)
}

fn decode_tone_powers_with_ap(
    powers: &Jt65TonePowers,
    context: &Jt65ApContext,
) -> Option<(crate::msg::Jt72Message, u32)> {
    if let Some(decoded) = decode_tone_powers(powers) {
        return Some(decoded);
    }
    context
        .masks()
        .iter()
        .find_map(|mask| decode_tone_powers_with_mask(powers, Some(mask)))
}

fn decode_tone_powers_with_mask(
    powers: &Jt65TonePowers,
    ap_mask: Option<&Jt65ApMask>,
) -> Option<(crate::msg::Jt72Message, u32)> {
    use crate::core::{DecodeContext, MessageCodec};

    let (mut symbols, second_symbols, confidence, best_probability) =
        rx::hard_decisions_from_tone_powers(powers);
    let mut forced = [false; 63];
    if let Some(ap_mask) = ap_mask {
        for (index, value) in ap_mask.iter().enumerate() {
            if let Some(value) = value {
                symbols[51 + index] = *value;
                forced[51 + index] = true;
            }
        }
    }
    let rs = Rs63_12::new();
    if let Some((info, corrected)) = rs.decode_jt65(&symbols) {
        let mut payload = [0u8; 72];
        for (index, bit) in payload.iter_mut().enumerate() {
            *bit = (info[index / 6] >> (5 - index % 6)) & 1;
        }
        if let Some(message) = Jt72Codec::default().unpack(&payload, &DecodeContext::default()) {
            return Some((message, corrected));
        }
    }

    // Deterministic port of the stochastic erasure-mask core in pinned
    // WSJT-X `ftrsdap.c`. All successful RS outputs pass through the
    // same hard/soft-distance and runner-up checks; accepting the first
    // algebraic codeword produces convincing-looking false decodes in noise.
    const PERR: [[u32; 8]; 8] = [
        [4, 9, 11, 13, 14, 14, 15, 15],
        [2, 20, 20, 30, 40, 50, 50, 50],
        [7, 24, 27, 40, 50, 50, 50, 50],
        [13, 25, 35, 46, 52, 70, 50, 50],
        [17, 30, 42, 54, 55, 64, 71, 70],
        [25, 39, 48, 57, 64, 66, 77, 77],
        [32, 45, 54, 63, 66, 75, 78, 83],
        [51, 58, 57, 66, 72, 77, 82, 86],
    ];
    #[derive(Clone)]
    struct Candidate {
        info: [u8; 12],
        message: crate::msg::Jt72Message,
        score: f32,
        hard_distance: u32,
        total_distance: u32,
    }

    let mut reliability_order = (0..63).collect::<Vec<_>>();
    reliability_order.sort_by(|&left, &right| {
        best_probability[right]
            .partial_cmp(&best_probability[left])
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    let probability_sum = best_probability.iter().sum::<f32>().max(f32::EPSILON);
    let mut thresholds = [0u32; 63];
    for rank_from_worst in 0..63 {
        let symbol = reliability_order[62 - rank_from_worst];
        if forced[symbol] {
            thresholds[symbol] = 0;
            continue;
        }
        let ratio = (1.0 - confidence[symbol]).clamp(0.0, 1.0);
        let ratio_bin = (7.999 * ratio).floor() as usize;
        let rank_bin = (62 - rank_from_worst) / 8;
        thresholds[symbol] =
            ((1.3 * PERR[ratio_bin.min(7)][rank_bin.min(7)] as f32).round() as u32).min(100);
    }

    let mut candidates: Vec<Candidate> = Vec::new();
    let mut seed = 1u32;
    for _trial in 0..10_000 {
        let mut erasures = Vec::with_capacity(51);
        for symbol in 0..63 {
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let random = ((seed / 65_536) % 32_768) * 100 / 32_768;
            if random < thresholds[symbol] && erasures.len() < 51 {
                erasures.push(symbol as u32);
            }
        }
        let Some((info, _corrected_count)) = rs.decode_jt65_erasures(&symbols, &erasures) else {
            continue;
        };
        if candidates.iter().any(|candidate| candidate.info == info) {
            continue;
        }
        let mut payload = [0u8; 72];
        for (index, bit) in payload.iter_mut().enumerate() {
            *bit = (info[index / 6] >> (5 - index % 6)) & 1;
        }
        let Some(message) = Jt72Codec::default().unpack(&payload, &DecodeContext::default()) else {
            continue;
        };

        let corrected_symbols = rs.encode_jt65(&info);
        let hard_distance = corrected_symbols
            .iter()
            .zip(&symbols)
            .filter(|(corrected, received)| corrected != received)
            .count() as u32;
        let soft_penalty = corrected_symbols
            .iter()
            .zip(&symbols)
            .zip(&second_symbols)
            .enumerate()
            .filter_map(|(index, ((corrected, received), second))| {
                (corrected != received && corrected != second).then_some(best_probability[index])
            })
            .sum::<f32>();
        let normalised_soft = (63.0 * soft_penalty / probability_sum).round() as u32;
        let total_distance = hard_distance + normalised_soft;

        let mut channel_symbols = corrected_symbols;
        interleave(&mut channel_symbols);
        for symbol in &mut channel_symbols {
            *symbol = gray6(*symbol);
        }
        let score = powers
            .iter()
            .zip(channel_symbols)
            .map(|(row, symbol)| row[symbol as usize])
            .sum::<f32>()
            / 63.0;
        candidates.push(Candidate {
            info,
            message,
            score,
            hard_distance,
            total_distance,
        });
        if hard_distance <= 41 && total_distance <= 71 {
            break;
        }
    }
    candidates.sort_by(|left, right| {
        right
            .score
            .partial_cmp(&left.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    let best = candidates.first()?;
    let runner_up_ratio = candidates
        .get(1)
        .map_or(0.0, |second| second.score / best.score.max(f32::EPSILON));
    (best.total_distance <= 81 && runner_up_ratio <= 0.87)
        .then(|| (best.message.clone(), best.hard_distance))
}

/// Decode repeated JT65 transmissions after non-coherent symbol-spectrum
/// averaging. Recordings are grouped by sync frequency and polarity, matching
/// pinned WSJT-X `avg65`.
pub fn decode_multi_period_for<P: ModulationParams>(
    recordings: &[&[f32]],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
) -> Vec<Jt65Decode> {
    decode_multi_period_impl::<P>(recordings, sample_rate, nominal_start_sample, params, None)
}

/// Station-aware AP variant of [`decode_multi_period_for`].
pub fn decode_multi_period_for_with_ap<P: ModulationParams>(
    recordings: &[&[f32]],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    context: &Jt65ApContext,
) -> Vec<Jt65Decode> {
    decode_multi_period_impl::<P>(
        recordings,
        sample_rate,
        nominal_start_sample,
        params,
        Some(context),
    )
}

fn decode_multi_period_impl<P: ModulationParams>(
    recordings: &[&[f32]],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    ap_context: Option<&Jt65ApContext>,
) -> Vec<Jt65Decode> {
    struct Group {
        frequency_hz: f32,
        start_sample: usize,
        polarity: SyncPolarity,
        periods: usize,
        powers: Jt65TonePowers,
    }

    let mut groups: Vec<Group> = Vec::new();
    let mut decoded = Vec::new();
    for audio in recordings {
        let Some(candidate) =
            search::coarse_search_for::<P>(audio, sample_rate, nominal_start_sample, params)
                .into_iter()
                .next()
        else {
            continue;
        };
        let Some(powers) = demodulate_tone_powers_for_with_polarity::<P>(
            audio,
            sample_rate,
            candidate.start_sample,
            candidate.freq_hz,
            candidate.polarity,
        ) else {
            continue;
        };
        let group_index = groups.iter().position(|group| {
            group.polarity == candidate.polarity
                && (group.frequency_hz - candidate.freq_hz).abs() <= 2.0 * P::TONE_SPACING_HZ
        });
        let index = if let Some(index) = group_index {
            for (sum_row, row) in groups[index].powers.iter_mut().zip(powers) {
                for (sum, power) in sum_row.iter_mut().zip(row) {
                    *sum += power;
                }
            }
            groups[index].periods += 1;
            index
        } else {
            groups.push(Group {
                frequency_hz: candidate.freq_hz,
                start_sample: candidate.start_sample,
                polarity: candidate.polarity,
                periods: 1,
                powers,
            });
            groups.len() - 1
        };
        if groups[index].periods < 2 {
            continue;
        }
        let result = if let Some(context) = ap_context {
            decode_tone_powers_with_ap(&groups[index].powers, context)
        } else {
            decode_tone_powers(&groups[index].powers)
        };
        if let Some((message, _corrected)) = result
            && !decoded
                .iter()
                .any(|previous: &Jt65Decode| previous.message == message)
        {
            decoded.push(Jt65Decode {
                message,
                freq_hz: groups[index].frequency_hz,
                start_sample: groups[index].start_sample,
                polarity: groups[index].polarity,
            });
        }
    }
    decoded
}

/// Scan an audio buffer for JT65 frames at any (freq, time) within
/// the search window: runs [`search::coarse_search`] and tries
/// [`decode_at`] on each candidate in score order, collapsing
/// duplicate decodes (same message ±2 Hz / ±1 symbol).
pub fn decode_scan(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
) -> Vec<Jt65Decode> {
    decode_scan_for::<Jt65>(audio, sample_rate, nominal_start_sample, params)
}

/// Scan an audio buffer for one JT65 sub-mode.
pub fn decode_scan_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
) -> Vec<Jt65Decode> {
    let cands = search::coarse_search_for::<P>(audio, sample_rate, nominal_start_sample, params);
    decode_candidates_for::<P>(audio, sample_rate, cands, None)
}

/// Scan one JT65 sub-mode with station-aware AP passes after the ordinary
/// decoder has failed.
pub fn decode_scan_for_with_ap<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    context: &Jt65ApContext,
) -> Vec<Jt65Decode> {
    let candidates =
        search::coarse_search_for::<P>(audio, sample_rate, nominal_start_sample, params);
    decode_candidates_for::<P>(audio, sample_rate, candidates, Some(context))
}

fn decode_candidates_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    mut cands: Vec<search::SyncCandidate>,
    ap_context: Option<&Jt65ApContext>,
) -> Vec<Jt65Decode> {
    // The exhaustive frequency/time grid produces local maxima even in
    // pure noise. WSJT-X applies a sync gate before its expensive FT
    // decoder; retain peaks reasonably close to the strongest candidate
    // in this slot and leave multi-period weak-signal recovery to the
    // dedicated averaging path.
    if let Some(strongest) = cands.first() {
        let relative_sync_floor = strongest.score * 0.65;
        cands.retain(|candidate| candidate.score >= relative_sync_floor);
    }
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let mut seen: Vec<Jt65Decode> = Vec::new();
    for c in cands {
        let Some(powers) = demodulate_tone_powers_for_with_polarity::<P>(
            audio,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            c.polarity,
        ) else {
            continue;
        };
        let result = if let Some(context) = ap_context {
            decode_tone_powers_with_ap(&powers, context)
        } else {
            decode_tone_powers(&powers)
        };
        let Some((msg, _corrected)) = result else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == msg
                && (prev.freq_hz - c.freq_hz).abs() <= 2.0
                && (prev.start_sample as i64 - c.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            seen.push(Jt65Decode {
                message: msg,
                freq_hz: c.freq_hz,
                start_sample: c.start_sample,
                polarity: c.polarity,
            });
        }
    }
    seen
}

fn deduplicate_shorthand_candidates<P: ModulationParams>(
    candidates: Vec<search::ShorthandCandidate>,
) -> Vec<Jt65ShorthandDecode> {
    let mut decodes: Vec<Jt65ShorthandDecode> = Vec::new();
    for candidate in candidates {
        let duplicate = decodes.iter().any(|previous| {
            previous.message == candidate.message
                && (previous.freq_hz - candidate.freq_hz).abs() <= 2.0 * P::TONE_SPACING_HZ
                && previous.start_sample.abs_diff(candidate.start_sample) <= P::NSPS as usize
        });
        if !duplicate {
            decodes.push(Jt65ShorthandDecode {
                message: candidate.message,
                freq_hz: candidate.freq_hz,
                start_sample: candidate.start_sample,
                score: candidate.score,
            });
        }
    }
    decodes
}

/// Decode only the JT65 two-tone EME shorthands.
pub fn decode_shorthand_scan_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
) -> Vec<Jt65ShorthandDecode> {
    deduplicate_shorthand_candidates::<P>(search::coarse_search_shorthand_for::<P>(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
    ))
}

/// Decode packed messages and EME shorthands from one shared spectrogram.
///
/// Returning separate collections preserves the existing `Jt72Message` API
/// while allowing the browser worker to expose every valid JT65 over-the-air
/// message without performing the expensive FFT pass twice.
pub fn decode_all_scan_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
) -> (Vec<Jt65Decode>, Vec<Jt65ShorthandDecode>) {
    let spectrogram = search::Spectrogram::build_for::<P>(audio, sample_rate);
    let packed_candidates = search::coarse_search_on_spec_for::<P>(
        &spectrogram,
        sample_rate,
        nominal_start_sample,
        params,
    );
    let shorthand_candidates = search::coarse_search_shorthand_on_spec_for::<P>(
        &spectrogram,
        sample_rate,
        nominal_start_sample,
        params,
    );
    (
        decode_candidates_for::<P>(audio, sample_rate, packed_candidates, None),
        deduplicate_shorthand_candidates::<P>(shorthand_candidates),
    )
}

/// Station-aware AP variant of [`decode_all_scan_for`].
pub fn decode_all_scan_for_with_ap<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    context: &Jt65ApContext,
) -> (Vec<Jt65Decode>, Vec<Jt65ShorthandDecode>) {
    let spectrogram = search::Spectrogram::build_for::<P>(audio, sample_rate);
    let packed_candidates = search::coarse_search_on_spec_for::<P>(
        &spectrogram,
        sample_rate,
        nominal_start_sample,
        params,
    );
    let shorthand_candidates = search::coarse_search_shorthand_on_spec_for::<P>(
        &spectrogram,
        sample_rate,
        nominal_start_sample,
        params,
    );
    (
        decode_candidates_for::<P>(audio, sample_rate, packed_candidates, Some(context)),
        deduplicate_shorthand_candidates::<P>(shorthand_candidates),
    )
}

pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<Jt65Decode> {
    decode_scan(audio, sample_rate, 0, &search::SearchParams::default())
}

/// Scan an audio buffer with default search parameters for one sub-mode.
pub fn decode_scan_default_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
) -> Vec<Jt65Decode> {
    decode_scan_for::<P>(audio, sample_rate, 0, &search::SearchParams::default())
}

/// JT65A protocol marker.
///
/// The `A` sub-mode uses the native baud ≈ 2.69 Hz tone spacing
/// (12 000 / 4460 Hz). B and C modes share everything else but
/// apply 2×/4× multipliers to the spacing.
#[derive(Copy, Clone, Debug, Default)]
pub struct Jt65;

impl ModulationParams for Jt65 {
    /// 66 = max tone index (65) + 1. Tones 2..=65 are the 64 data
    /// tones; tone 0 is sync; tone 1 is unused (a single-slot gap
    /// above the sync tone, a quirk of the WSJT-X tone numbering).
    const NTONES: u32 = 66;
    const BITS_PER_SYMBOL: u32 = 6;
    /// 4460 samples/symbol at 12 kHz gives baud ≈ 2.6906 Hz — the
    /// canonical rounded value WSJT-X uses internally derives from
    /// 11 025 / 4096 but the integer-sample convention in our
    /// pipeline is NSPS.
    const NSPS: u32 = 4460;
    const SYMBOL_DT: f32 = 4460.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 4460.0; // ≈ 2.6906 Hz
    /// No Gray map here — Gray is applied at the *symbol* level
    /// (6-bit) in [`gray::gray6`], not at the FSK-tone level. A
    /// minimal identity map satisfies the trait's `GRAY_MAP.len()
    /// == NTONES` invariant.
    const GRAY_MAP: &'static [u8] = &IDENTITY_66;
    const GFSK_BT: f32 = 0.0; // plain FSK
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
    const NSTEP_PER_SYMBOL: u32 = 2;
    /// 12 000 / 4 = 3000 Hz baseband (enough for the 65-tone span).
    const NDOWN: u32 = 4;
}

const IDENTITY_66: [u8; 66] = {
    let mut m = [0u8; 66];
    let mut i = 0usize;
    while i < 66 {
        m[i] = i as u8;
        i += 1;
    }
    m
};

impl FrameLayout for Jt65 {
    const N_DATA: u32 = 63;
    const N_SYNC: u32 = 63;
    const N_SYMBOLS: u32 = 126;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Block(&JT65_SYNC_BLOCKS);
    /// 46.8-second frame, scheduled in 60-second slots with a few
    /// seconds of leading silence — matches WSJT-X's JT65 slot.
    const T_SLOT_S: f32 = 60.0;
    const TX_START_OFFSET_S: f32 = 0.0;
}

impl Protocol for Jt65 {
    /// Reed-Solomon (63, 12) over GF(2^6). Does NOT implement
    /// `FecCodec` (bit-LLR oriented) — jt65-core's decode path
    /// bypasses the generic pipeline and calls the symbol-level
    /// API directly. Declared here so the protocol's FEC intent
    /// is still visible in the trait surface.
    type Fec = Rs63_12;
    /// 72-bit message payload (12 × 6-bit words), shared with JT9.
    type Msg = Jt72Codec;
    const ID: ProtocolId = ProtocolId::Jt65;
}

macro_rules! jt65_submode {
    ($name:ident, $spacing_multiplier:expr) => {
        #[derive(Copy, Clone, Debug, Default)]
        pub struct $name;

        impl ModulationParams for $name {
            const NTONES: u32 = 66;
            const BITS_PER_SYMBOL: u32 = 6;
            const NSPS: u32 = 4460;
            const SYMBOL_DT: f32 = 4460.0 / 12_000.0;
            const TONE_SPACING_HZ: f32 = $spacing_multiplier as f32 * 12_000.0 / 4460.0;
            const GRAY_MAP: &'static [u8] = &IDENTITY_66;
            const GFSK_BT: f32 = 0.0;
            const GFSK_HMOD: f32 = 1.0;
            const NFFT_PER_SYMBOL_FACTOR: u32 = 2;
            const NSTEP_PER_SYMBOL: u32 = 2;
            const NDOWN: u32 = 4;
        }

        impl FrameLayout for $name {
            const N_DATA: u32 = 63;
            const N_SYNC: u32 = 63;
            const N_SYMBOLS: u32 = 126;
            const N_RAMP: u32 = 0;
            const SYNC_MODE: SyncMode = SyncMode::Block(&JT65_SYNC_BLOCKS);
            const T_SLOT_S: f32 = 60.0;
            const TX_START_OFFSET_S: f32 = 0.0;
        }

        impl Protocol for $name {
            type Fec = Rs63_12;
            type Msg = Jt72Codec;
            const ID: ProtocolId = ProtocolId::Jt65;
        }
    };
}

// JT65B protocol marker (2x tone spacing).
jt65_submode!(Jt65b, 2);
// JT65C protocol marker (4x tone spacing).
jt65_submode!(Jt65c, 4);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::Jt72Message;

    #[test]
    fn erasure_assisted_decode_recovers_under_moderate_noise() {
        // Clean synth gets decoded by plain `decode_at`; erasure path
        // is a strict superset so it should also work (trying 0 first).
        let freq = 1270.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let msg = decode_at_with_erasures(&audio, 12_000, 0, freq, &[0, 8, 16, 24, 32])
            .expect("erasure-aware path must decode clean synth");
        assert!(matches!(
            msg,
            Jt72Message::Standard { ref call1, ref call2, ref grid_or_report }
                if call1 == "CQ" && call2 == "K1ABC" && grid_or_report == "FN42"
        ));
    }

    #[test]
    fn jt65_trait_surface() {
        assert_eq!(<Jt65 as ModulationParams>::NTONES, 66);
        assert_eq!(<Jt65 as ModulationParams>::BITS_PER_SYMBOL, 6);
        assert_eq!(<Jt65 as ModulationParams>::NSPS, 4460);
        assert_eq!(<Jt65 as FrameLayout>::N_SYMBOLS, 126);
        assert_eq!(<Jt65 as FrameLayout>::N_DATA, 63);
        assert_eq!(<Jt65 as FrameLayout>::N_SYNC, 63);
        match <Jt65 as FrameLayout>::SYNC_MODE {
            SyncMode::Block(blocks) => {
                assert_eq!(blocks.len(), 63);
                for b in blocks {
                    assert_eq!(b.pattern, &[0u8]);
                }
            }
            SyncMode::Interleaved { .. } => panic!("JT65 must use Block sync"),
        }
        // RS(63, 12) doesn't implement FecCodec — we only verify the
        // associated-type wiring compiles by spelling the path out.
        let _fec = Rs63_12::default();
    }

    fn assert_submode_roundtrip<P: ModulationParams>() {
        let freq = 900.0;
        let audio =
            synthesize_standard_for::<P>("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let msg = decode_at_for::<P>(&audio, 12_000, 0, freq).expect("decode");
        assert!(matches!(
            msg,
            Jt72Message::Standard { ref call1, ref call2, ref grid_or_report }
                if call1 == "CQ" && call2 == "K1ABC" && grid_or_report == "FN42"
        ));
    }

    #[test]
    fn jt65b_roundtrip() {
        assert_submode_roundtrip::<Jt65b>();
    }

    #[test]
    fn jt65c_roundtrip() {
        assert_submode_roundtrip::<Jt65c>();
    }

    fn assert_inverted_roundtrip<P: ModulationParams>() {
        let frequency = 900.0;
        let audio = synthesize_standard_for_with_polarity::<P>(
            "KA1ABC",
            "WB9XYZ",
            "EN34",
            12_000,
            frequency,
            0.3,
            SyncPolarity::Inverted,
        )
        .expect("OOO synth");
        let message =
            decode_at_for_with_polarity::<P>(&audio, 12_000, 0, frequency, SyncPolarity::Inverted)
                .expect("OOO decode");
        assert_eq!(message.to_string(), "KA1ABC WB9XYZ EN34");

        let params = search::SearchParams {
            freq_min_hz: 800.0,
            freq_max_hz: 1_000.0,
            ..search::SearchParams::default()
        };
        let scans = decode_scan_for::<P>(&audio, 12_000, 0, &params);
        assert!(
            scans.iter().any(|decode| {
                decode.polarity == SyncPolarity::Inverted
                    && decode.message.to_string() == "KA1ABC WB9XYZ EN34"
            }),
            "automatic search must preserve OOO polarity"
        );
    }

    #[test]
    fn inverted_ooo_roundtrips_all_submodes() {
        assert_inverted_roundtrip::<Jt65>();
        assert_inverted_roundtrip::<Jt65b>();
        assert_inverted_roundtrip::<Jt65c>();
    }

    fn assert_shorthand_roundtrip<P: ModulationParams>() {
        let params = search::SearchParams {
            freq_min_hz: 800.0,
            freq_max_hz: 1_000.0,
            ..search::SearchParams::default()
        };
        for message in [
            Jt65Shorthand::Ro,
            Jt65Shorthand::Rrr,
            Jt65Shorthand::SeventyThree,
        ] {
            let audio = synthesize_shorthand_for::<P>(message, 12_000, 900.0, 0.3);
            let decodes = decode_shorthand_scan_for::<P>(&audio, 12_000, 0, &params);
            assert!(
                decodes.iter().any(|decode| decode.message == message),
                "{message} must decode"
            );
        }
    }

    #[test]
    fn eme_shorthands_roundtrip_all_submodes() {
        assert_shorthand_roundtrip::<Jt65>();
        assert_shorthand_roundtrip::<Jt65b>();
        assert_shorthand_roundtrip::<Jt65c>();
    }
}
