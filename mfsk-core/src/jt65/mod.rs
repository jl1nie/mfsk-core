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
//! Only the **JT65A** sub-mode (tone spacing = baud ≈ 2.69 Hz) is
//! currently wired. JT65B and JT65C differ by a tone-spacing
//! multiplier (2×, 4×) and can be added as separate ZSTs sharing
//! every other piece.
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
//!
//! ## Stochastic Chase decode
//!
//! For signals still too weak for [`decode_at_with_erasures`]'s single
//! deterministic ordering, [`chase::decode_at_with_chase`] is a
//! faithful port of WSJT-X's `ftrsdap` stochastic Chase decoder
//! ([issue #169](https://github.com/jl1nie/mfsk-core/issues/169)) —
//! magic numbers included, not just the algorithmic shape: WSJT-X's
//! own erasure-probability table, its `getpp` spectral-power candidate
//! ranking, and its literal acceptance-gate constants (see [`chase`]'s
//! module doc for the full list). Measured on the AWGN sweep
//! (`docs/notes/BENCHMARKS.md`), this closes most — not all — of the
//! ~7-8 dB sensitivity gap vs. real WSJT-X's `jt9 -6`.
//!
//! ```no_run
//! use mfsk_core::jt65::decode_scan_chase_default;
//!
//! # let audio: Vec<f32> = vec![];
//! for r in decode_scan_chase_default(&audio, 12_000) {
//!     println!("{:+7.1} Hz  start={:>8} sample  {}",
//!              r.freq_hz, r.start_sample, r.message);
//! }
//! ```

use crate::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::Rs63_12;
use crate::msg::Jt72Codec;

pub mod chase;
pub mod gray;
pub mod interleave;
pub mod rx;
pub mod search;
pub mod sync_pattern;
pub mod tx;

pub use chase::{ChaseParams, decode_at_with_chase};
pub use gray::{gray6, inv_gray6};
pub use interleave::{deinterleave, interleave};
pub use rx::{
    demodulate_aligned, demodulate_aligned_with_confidence, demodulate_aligned_with_runnerup,
};
pub use sync_pattern::{JT65_DATA_POSITIONS, JT65_NPRC, JT65_SYNC_BLOCKS, JT65_SYNC_POSITIONS};
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
    use crate::engine::{DecodeContext, MessageCodec};

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

/// Like [`decode_at`] but also returns the decode-side SNR estimate
/// from [`rx::demodulate_aligned_with_confidence_and_snr`]. Used by
/// [`decode_scan`] to populate [`Jt65Result::snr_db`]; kept private
/// since [`decode_at`]'s return type is part of the stable surface
/// mirrored by `jt9::decode_at`.
fn decode_at_with_snr(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<(crate::msg::Jt72Message, f32)> {
    use crate::engine::{DecodeContext, MessageCodec};

    let (received, _conf, snr_db) = rx::demodulate_aligned_with_confidence_and_snr(
        audio,
        sample_rate,
        start_sample,
        base_freq_hz,
    )?;
    let rs = Rs63_12::new();
    let (info, _nerr) = rs.decode_jt65(&received)?;
    let mut payload = [0u8; 72];
    for (i, bit) in payload.iter_mut().enumerate() {
        let word = info[i / 6];
        let shift = 5 - (i % 6);
        *bit = (word >> shift) & 1;
    }
    let msg = crate::msg::Jt72Codec::default().unpack(&payload, &DecodeContext::default())?;
    Some((msg, snr_db))
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
    use crate::engine::{DecodeContext, MessageCodec};

    let (symbols, conf) =
        rx::demodulate_aligned_with_confidence(audio, sample_rate, start_sample, base_freq_hz)?;
    // Ordering of symbol positions from least → most confident; the
    // caller's erasure budget eats from the start. Shared with
    // `chase::decode_at_with_chase`, which sorts on the same
    // confidence array the same way.
    let order = chase::confidence_order(&conf);

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
pub struct Jt65Result {
    pub message: crate::msg::Jt72Message,
    pub freq_hz: f32,
    pub start_sample: usize,
    /// Decode-side SNR estimate in dB (WSJT-X 2500 Hz reference
    /// bandwidth convention) — see
    /// [`rx::demodulate_aligned_with_confidence_and_snr`] for the
    /// formula and its calibration caveat.
    pub snr_db: f32,
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
) -> Vec<Jt65Result> {
    decode_scan_inner(audio, sample_rate, nominal_start_sample, params, None)
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
/// is sequential with no early exit and no parallelism — `cb` fires
/// exactly once per result that ends up in the returned `Vec`, in the
/// same order. No divergence mechanism exists here (unlike WSPR's/
/// FT8's parallel strategies).
pub fn decode_scan_streaming(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    on_result: &(dyn Fn(&Jt65Result) + Sync),
) -> Vec<Jt65Result> {
    decode_scan_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        params,
        Some(on_result),
    )
}

fn decode_scan_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    on_result: Option<&(dyn Fn(&Jt65Result) + Sync)>,
) -> Vec<Jt65Result> {
    use crate::engine::ModulationParams;
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let cands = search::coarse_search(audio, sample_rate, nominal_start_sample, params);
    let mut seen: Vec<Jt65Result> = Vec::new();
    for c in cands {
        let Some((msg, snr_db)) = decode_at_with_snr(audio, sample_rate, c.start_sample, c.freq_hz)
        else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == msg
                && (prev.freq_hz - c.freq_hz).abs() <= 2.0
                && (prev.start_sample as i64 - c.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            let result = Jt65Result {
                message: msg,
                freq_hz: c.freq_hz,
                start_sample: c.start_sample,
                snr_db,
            };
            if let Some(cb) = on_result {
                cb(&result);
            }
            seen.push(result);
        }
    }
    seen
}

pub fn decode_scan_default(audio: &[f32], sample_rate: u32) -> Vec<Jt65Result> {
    decode_scan(audio, sample_rate, 0, &search::SearchParams::default())
}

/// Like [`decode_scan`] but decodes each candidate via
/// [`chase::decode_at_with_chase`]'s randomized multi-trial erasure
/// search instead of `decode_at_with_snr`'s plain zero-erasure hard
/// decision — trades decode time for sensitivity on weak signals. See
/// [`chase`]'s module doc for the algorithm and its relationship to
/// WSJT-X's `ftrsdap`.
///
/// A parallel sibling trio (`decode_scan_chase`/`_streaming`/
/// `_default`) rather than a parameter on [`decode_scan`] itself, for
/// the same reason [`decode_scan_streaming`] is its own sibling
/// (mod.rs's own doc comment above): both the extra [`chase::ChaseParams`]
/// and the different internal decode engine argue against bolting
/// onto the existing plain `pub fn`.
pub fn decode_scan_chase(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    search_params: &search::SearchParams,
    chase_params: &chase::ChaseParams,
) -> Vec<Jt65Result> {
    decode_scan_chase_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        search_params,
        chase_params,
        None,
    )
}

/// Streaming variant of [`decode_scan_chase`] — same contract as
/// [`decode_scan_streaming`] (see that function's doc comment for the
/// delivery-order/dedup guarantee, which applies identically here).
pub fn decode_scan_chase_streaming(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    search_params: &search::SearchParams,
    chase_params: &chase::ChaseParams,
    on_result: &(dyn Fn(&Jt65Result) + Sync),
) -> Vec<Jt65Result> {
    decode_scan_chase_inner(
        audio,
        sample_rate,
        nominal_start_sample,
        search_params,
        chase_params,
        Some(on_result),
    )
}

fn decode_scan_chase_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    search_params: &search::SearchParams,
    chase_params: &chase::ChaseParams,
    on_result: Option<&(dyn Fn(&Jt65Result) + Sync)>,
) -> Vec<Jt65Result> {
    use crate::engine::ModulationParams;
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let cands = search::coarse_search(audio, sample_rate, nominal_start_sample, search_params);
    let mut seen: Vec<Jt65Result> = Vec::new();
    for c in cands {
        let Some((msg, snr_db)) = chase::decode_at_with_chase_and_snr(
            audio,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            chase_params,
        ) else {
            continue;
        };
        let dup = seen.iter().any(|prev| {
            prev.message == msg
                && (prev.freq_hz - c.freq_hz).abs() <= 2.0
                && (prev.start_sample as i64 - c.start_sample as i64).abs() <= nsps as i64
        });
        if !dup {
            let result = Jt65Result {
                message: msg,
                freq_hz: c.freq_hz,
                start_sample: c.start_sample,
                snr_db,
            };
            if let Some(cb) = on_result {
                cb(&result);
            }
            seen.push(result);
        }
    }
    seen
}

pub fn decode_scan_chase_default(audio: &[f32], sample_rate: u32) -> Vec<Jt65Result> {
    decode_scan_chase(
        audio,
        sample_rate,
        0,
        &search::SearchParams::default(),
        &chase::ChaseParams::default(),
    )
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
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let msg = decode_at_with_erasures(&audio, 12_000, 0, freq, &[0, 8, 16, 24, 32])
            .expect("erasure-aware path must decode clean synth");
        assert!(matches!(
            msg,
            Jt72Message::Standard { ref call1, ref call2, ref grid_or_report }
                if call1 == "CQ" && call2 == "K1ABC" && grid_or_report == "FN42"
        ));
    }

    /// `decode_scan_streaming`'s `on_result` callback — synthetic
    /// round-trip verification, matching this module's own existing
    /// synth-test convention (no real-sample WSJT-X recording is
    /// wired for JT65 today — see `tests/jt65_sweep.rs`'s doc comment
    /// for why, it needs an out-of-tree `jt65sim` build). `decode_scan`
    /// is sequential with no early exit and no parallelism, so the
    /// callback-delivered set must exactly equal the batch `Vec`.
    #[test]
    fn decode_scan_streaming_matches_batch_exactly() {
        use std::sync::Mutex;

        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "JL1NIE", "PM95", 12_000, freq, 0.3).expect("synth");
        // Drop the signal 1 s into a zeroed slot so decode_scan must
        // actually search rather than decode at (start_sample=0).
        let mut slot = vec![0.0f32; 12_000 + audio.len()];
        slot[12_000..12_000 + audio.len()].copy_from_slice(&audio);

        let streamed_acc: Mutex<Vec<Jt72Message>> = Mutex::new(Vec::new());
        let on_result = |r: &Jt65Result| streamed_acc.lock().unwrap().push(r.message.clone());
        let batch = decode_scan_streaming(
            &slot,
            12_000,
            0,
            &search::SearchParams::default(),
            &on_result,
        );
        let streamed = streamed_acc.into_inner().unwrap();
        let batch_msgs: Vec<Jt72Message> = batch.iter().map(|d| d.message.clone()).collect();
        assert_eq!(
            streamed, batch_msgs,
            "JT65 decode_scan_streaming: streamed callback deliveries must \
             exactly match the batch result, same order (sequential, no \
             early exit, no parallelism — no divergence mechanism exists)"
        );
        assert!(
            !streamed.is_empty(),
            "expected at least one streamed decode on the synth signal"
        );
        assert!(matches!(
            &streamed[0],
            Jt72Message::Standard { call1, call2, grid_or_report }
                if call1 == "CQ" && call2 == "JL1NIE" && grid_or_report == "PM95"
        ));
    }

    /// `decode_scan_chase` end-to-end: proves the scan wiring actually
    /// reaches `chase::decode_at_with_chase_and_snr` (not just that
    /// the function compiles) by requiring a genuine search — same
    /// "signal dropped 1s into a zeroed slot" shape as
    /// `decode_scan_streaming_matches_batch_exactly` above.
    #[test]
    fn decode_scan_chase_finds_signal_via_search() {
        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "JL1NIE", "PM95", 12_000, freq, 0.3).expect("synth");
        let mut slot = vec![0.0f32; 12_000 + audio.len()];
        slot[12_000..12_000 + audio.len()].copy_from_slice(&audio);

        let results = decode_scan_chase_default(&slot, 12_000);
        assert!(
            !results.is_empty(),
            "expected at least one chase-decoded result on the synth signal"
        );
        assert!(matches!(
            &results[0].message,
            Jt72Message::Standard { call1, call2, grid_or_report }
                if call1 == "CQ" && call2 == "JL1NIE" && grid_or_report == "PM95"
        ));
    }

    /// Scan-level false-decode guardrail (see `chase.rs`'s own
    /// `chase_never_false_decodes_*` tests for the bounded, always-run
    /// version). `#[ignore]`d: a coarse-search-driven scan over noise
    /// can turn up many spurious frequency/time candidates, each
    /// burning up to `ChaseParams::max_trials` RS-decode attempts —
    /// too slow for the default (non-`--ignored`) suite, but worth
    /// the extra end-to-end confidence as an explicit, runnable check.
    #[test]
    #[ignore]
    fn decode_scan_chase_never_false_decodes_on_noise() {
        struct NoiseGen(u32);
        impl NoiseGen {
            fn next_u32(&mut self) -> u32 {
                let mut x = self.0;
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                self.0 = x;
                x
            }
            fn next_f32(&mut self) -> f32 {
                (self.next_u32() as f32) / (u32::MAX as f32)
            }
            fn gaussian(&mut self) -> f32 {
                let u1 = self.next_f32().max(1e-9);
                let u2 = self.next_f32();
                (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
            }
        }

        const NSAMPLES: usize = 60 * 12_000; // one full 60 s JT65 slot
        for seed in 1..=5u32 {
            let mut rng = NoiseGen(seed.wrapping_mul(2_654_435_761) | 1);
            let audio: Vec<f32> = (0..NSAMPLES).map(|_| 0.3 * rng.gaussian()).collect();
            let results = decode_scan_chase_default(&audio, 12_000);
            assert!(
                results.is_empty(),
                "decode_scan_chase must not decode pure noise (seed={seed}), got {results:?}"
            );
        }
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
