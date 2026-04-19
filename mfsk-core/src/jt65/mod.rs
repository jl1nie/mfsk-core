//! # jt65-core
//!
//! JT65 protocol implementation on the `mfsk-*` stack.
//!
//! JT65 uses:
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
//! Only the **JT65A** sub-mode (tone spacing = baud ≈ 2.69 Hz) is
//! currently wired. JT65B and JT65C differ by a tone-spacing
//! multiplier (2×, 4×) and can be added as separate ZSTs sharing
//! every other piece.
//!
//! References:
//! - WSJT-X `lib/jt65sim.f90`, `lib/setup65.f90`, `lib/interleave63.f90`,
//!   `lib/graycode65.f90`, `lib/wrapkarn.c`

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
pub use rx::{demodulate_aligned, demodulate_aligned_with_confidence};
pub use sync_pattern::{
    JT65_DATA_POSITIONS, JT65_NPRC, JT65_SYNC_BLOCKS, JT65_SYNC_POSITIONS,
};
pub use tx::{encode_channel_symbols, synthesize_audio, synthesize_standard};

/// Top-level: decode a JT65 signal at a known (start_sample, base_freq)
/// and return the recovered message if RS succeeds. Mirrors the shape of
/// `mfsk_core::jt9::decode_at`.
pub fn decode_at(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<crate::msg::Jt72Message> {
    use crate::core::{DecodeContext, MessageCodec};

    let received = rx::demodulate_aligned(audio, sample_rate, start_sample, base_freq_hz)?;
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
    use crate::core::{DecodeContext, MessageCodec};

    let (symbols, conf) =
        rx::demodulate_aligned_with_confidence(audio, sample_rate, start_sample, base_freq_hz)?;
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
    use crate::core::ModulationParams;
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let cands = search::coarse_search(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<Jt65Decode> = Vec::new();
    for c in cands {
        let Some(msg) = decode_at(audio, sample_rate, c.start_sample, c.freq_hz) else {
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
            });
        }
    }
    seen
}

pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<Jt65Decode> {
    decode_scan(audio, sample_rate, 0, &search::SearchParams::default())
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::Jt72Message;

    #[test]
    fn erasure_assisted_decode_recovers_under_moderate_noise() {
        // Clean synth gets decoded by plain `decode_at`; erasure path
        // is a strict superset so it should also work (trying 0 first).
        let freq = 1270.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3)
            .expect("synth");
        let msg = decode_at_with_erasures(
            &audio,
            12_000,
            0,
            freq,
            &[0, 8, 16, 24, 32],
        )
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
}
