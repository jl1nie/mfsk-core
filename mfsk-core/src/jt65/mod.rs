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
//! use mfsk_core::jt65::DecodeRequest;
//!
//! # let audio: Vec<f32> = vec![];
//! // `audio` is 720_000 f32 samples at 12 kHz (60 s slot).
//! for r in DecodeRequest::new(&audio, 12_000).decode() {
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
//! (`2·errors + erasures ≤ 51`). Use [`SniperRequest::erasures`]:
//!
//! ```no_run
//! use mfsk_core::jt65::DecodeRequest;
//!
//! # let audio: Vec<f32> = vec![];
//! # let (start_sample, freq_hz) = (0, 1270.0);
//! // Try 0 → 8 → 16 → 24 → 32 erasures in order; return the first
//! // budget that unpacks into a valid message.
//! let msg = DecodeRequest::sniper(&audio, 12_000, start_sample, freq_hz)
//!     .erasures(&[0, 8, 16, 24, 32])
//!     .decode();
//! ```
//!
//! ## Stochastic Chase decode
//!
//! For signals still too weak for [`SniperRequest::erasures`]'s single
//! deterministic ordering, `DecodeRequest::chase` /
//! [`SniperRequest::chase`] is a
//! faithful port of WSJT-X's `ftrsdap` stochastic Chase decoder
//! ([issue #169](https://github.com/jl1nie/mfsk-core/issues/169)) —
//! magic numbers included, not just the algorithmic shape: WSJT-X's
//! own erasure-probability table, its `getpp` spectral-power candidate
//! ranking, and its literal acceptance-gate constants (see [`chase`]'s
//! module doc for the full list). A second, independent fix landed the
//! same day: [`search`]/[`rx`] gained a sub-bin frequency refinement +
//! NCO correction that eliminates FFT "scalloping loss" — this
//! benefits *every* decode path in this module, not just
//! the chase path (the erasure path inherits it too,
//! with no code changes of its own). Measured on the AWGN sweep
//! (`docs/notes/BENCHMARKS.md`), the two fixes together closed the
//! previously-documented ~7-8 dB sensitivity gap vs. real WSJT-X's
//! `jt9 -6` essentially entirely on this crate's corpus — see that
//! doc's JT65 section for the full story and honest caveats on the
//! WSJT-X comparison.
//!
//! ```no_run
//! use mfsk_core::jt65::{ChaseParams, DecodeRequest};
//!
//! # let audio: Vec<f32> = vec![];
//! for r in DecodeRequest::new(&audio, 12_000).chase(ChaseParams::default()).decode() {
//!     println!("{:+7.1} Hz  start={:>8} sample  {}",
//!              r.freq_hz, r.start_sample, r.message);
//! }
//! ```

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
use alloc::vec;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
use num_traits::Float;

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
use crate::engine::pipeline::scan_dedup_match_cross;
use crate::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use crate::fec::Rs63_12;
use crate::msg::Jt72Codec;

// Decode-side modules reach their FFT through `engine::fft`, whose own
// modules are gated on the FFT meta-feature; mirror that gate here so
// `--features <mode>` alone still builds. TX and the const tables stay
// unconditional, the same split `wspr::mod` uses.
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod averaging;
pub mod chase;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
#[cfg(any(feature = "internal-testing", test))]
pub mod decode_request;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
#[cfg(not(any(feature = "internal-testing", test)))]
pub(crate) mod decode_request;
pub mod interleave;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod rx;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod search;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
mod subtract;
pub mod sync_pattern;
pub mod tx;

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use chase::ChaseParams;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use decode_request::SniperRequest;
// The wide-band request is the engine's, reached through `internal-testing`
// since 0.13: `crate::decoder::Decoder` is the decode API.
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
#[cfg(any(feature = "internal-testing", test))]
pub use decode_request::DecodeRequest;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
#[cfg(not(any(feature = "internal-testing", test)))]
pub(crate) use decode_request::DecodeRequest;
pub use interleave::{deinterleave, interleave};
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use rx::{Jt65Demod, demodulate_aligned};
pub use sync_pattern::{JT65_DATA_POSITIONS, JT65_NPRC, JT65_SYNC_BLOCKS, JT65_SYNC_POSITIONS};
pub use tx::{encode_channel_symbols, synthesize_standard};

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// Hard-decision RS decode at a known (start_sample, base_freq), with
/// the decode-side SNR estimate ([`Jt65Demod::snr_db`]) alongside the
/// message. Public through [`SniperRequest`].
fn decode_at_with_snr(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
) -> Option<(crate::msg::Jt72Message, f32, [u8; 12])> {
    use crate::engine::{DecodeContext, MessageCodec};

    let demod = rx::demodulate_aligned(audio, sample_rate, start_sample, base_freq_hz)?;
    let snr_db = demod.snr_db;
    let rs = Rs63_12::new();
    let (info, _nerr) = rs.decode_jt65(&demod.symbols)?;
    let mut payload = [0u8; 72];
    for (i, bit) in payload.iter_mut().enumerate() {
        let word = info[i / 6];
        let shift = 5 - (i % 6);
        *bit = (word >> shift) & 1;
    }
    let msg = crate::msg::Jt72Codec::default().unpack(&payload, &DecodeContext::default())?;
    Some((msg, snr_db, info))
}

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// Decode a JT65 signal at a known alignment, trying progressively
/// larger erasure counts until Reed-Solomon converges or the bound
/// is exhausted. Unlike plain hard-decision RS, this exploits
/// per-symbol confidence from the demodulator: symbols with the
/// smallest (best − runner-up) margin are flagged as erasures, which
/// doubles the correctable error count compared to the plain
/// hard-decision bound.
///
/// `attempts` is a slice of erasure counts to try in order. A
/// reasonable default is `&[0, 8, 16, 24, 32]`: zero-erasure first
/// (fastest when the channel is clean) and then growing erasure
/// budgets for lower-SNR signals. Returns the first decode that
/// unpacks into a valid [`crate::msg::jt72::Jt72Message`]. Public
/// through [`SniperRequest::erasures`].
fn decode_at_with_erasures(
    audio: &[f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    attempts: &[usize],
) -> Option<crate::msg::Jt72Message> {
    use crate::engine::{DecodeContext, MessageCodec};

    let rx::Jt65Demod { symbols, conf, .. } =
        rx::demodulate_aligned(audio, sample_rate, start_sample, base_freq_hz)?;
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

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// One successful JT65 decode with its alignment info.
#[derive(Clone, Debug)]
pub struct Jt65Result {
    pub message: crate::msg::Jt72Message,
    pub freq_hz: f32,
    /// Frame start as an index into the audio buffer that was passed
    /// in.
    ///
    /// **Saturates at 0** for a frame that began *before* the buffer —
    /// which the scan can now find (issue #283), because a JT65 frame
    /// arriving several seconds early is still decodable from the part
    /// of it that landed inside the slot. Such a frame has no valid
    /// index here; use [`Self::dt_sec`], which is signed and always
    /// authoritative.
    pub start_sample: usize,
    /// Frame start in seconds from the nominal start
    /// (`DecodeRequest::nominal_start`) — the signed form of
    /// [`Self::start_sample`], and the only field that can express a
    /// frame beginning *before* the buffer (issue #283), where
    /// `start_sample` saturates at 0. Comparable directly with a
    /// reference decoder's DT column (#397).
    pub dt_sec: f32,
    /// Decode-side SNR estimate in dB (WSJT-X 2500 Hz reference
    /// bandwidth convention) — see
    /// [`Jt65Demod::snr_db`] for the
    /// formula and its calibration caveat.
    pub snr_db: f32,
}

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// Front-pad `audio` with silence so that a frame starting up to
/// `params.time_tolerance_sec` *before* `nominal_start_sample` still
/// has a non-negative index, and return the padded buffer with the
/// shifted nominal.
///
/// Issue #283: without this the coarse search clamps `row_min` at 0
/// and never scores an early frame at all, while real `jt9` decodes
/// it from whatever part of the frame landed inside the slot —
/// measured on a `jt65sim -t` sweep, `jt9 -6` decoded Δt down to
/// −3.0 s where this crate stopped at exactly `−nominal`, the clamp's
/// signature.
///
/// Padding rather than signed start indices throughout: the leading
/// silence *is* the erasure, so `extract_*_energies`' existing
/// unsigned arithmetic and full-frame bounds check keep working
/// untouched. WSJT-X reaches the same result differently, by scoring
/// the hypothesis and skipping out-of-buffer terms in the inner loop
/// (`xcor.f90:49-50`, `sync9.f90:40`); this crate's FT8 coarse sync
/// already does it that way, but JT65's demod is not structured for
/// it and the numerics come out identical either way.
///
/// Returns `None` when no padding is needed, so the common path keeps
/// borrowing the caller's slice with no copy.
fn pad_for_early_frames(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    time_tolerance_early_sec: f32,
) -> Option<(Vec<f32>, usize)> {
    let want = (time_tolerance_early_sec.max(0.0) * sample_rate as f32).round() as usize;
    let pad = want.saturating_sub(nominal_start_sample);
    if pad == 0 {
        return None;
    }
    let mut padded = vec![0.0f32; pad];
    padded.extend_from_slice(audio);
    Some((padded, pad))
}

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
/// Scan an audio buffer for JT65 frames at any (freq, time) within
/// the search window: runs [`search::coarse_search`] and decodes each
/// candidate in score order — hard-decision RS, or the stochastic
/// Chase search when `chase` is set — collapsing duplicate decodes
/// (same message ±2 Hz / ±1 symbol). Public through [`DecodeRequest`].
///
/// `npass > 1` is `jt65_decode.f90:110-145`'s pass loop: after each of the
/// first three passes the signals found are subtracted from the audio
/// ([`subtract::subtract65`]) and the next pass searches the residue, with
/// at most `50 / ipass` candidates (`:176`). The last pass of four does not
/// subtract (`nsubtract = 0`, `:138`). `npass == 1` is the single search the
/// 0.12 request always ran. The crate's coarse search has its own score
/// scale, so upstream's per-pass `thresh0` (2.5, 2.0, 2.0, 2.0) is not
/// applied; the candidate cap is.
fn decode_scan_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &search::SearchParams,
    chase: Option<&chase::ChaseParams>,
    npass: u8,
    mut averager: Option<(&mut averaging::Averager, i64, f32)>,
    on_result: Option<&(dyn Fn(&Jt65Result) + Sync)>,
) -> Vec<Jt65Result> {
    use crate::engine::ModulationParams;
    let nsps = (sample_rate as f32 * <Jt65 as ModulationParams>::SYMBOL_DT).round() as usize;
    let padding = pad_for_early_frames(
        audio,
        sample_rate,
        nominal_start_sample,
        params.time_tolerance_early_sec,
    );
    let (audio, pad) = match &padding {
        Some((buf, pad)) => (buf.as_slice(), *pad),
        None => (audio, 0),
    };
    let nominal_start_sample = nominal_start_sample + pad;
    // Passes that subtract work on a copy; one pass reads the input as is.
    let mut residue: Option<Vec<f32>> = (npass > 1).then(|| audio.to_vec());
    let mut seen: Vec<Jt65Result> = Vec::new();
    for ipass in 1..=npass.max(1) {
        let work: &[f32] = residue.as_deref().unwrap_or(audio);
        let mut pass_params = *params;
        if npass > 1 {
            pass_params.max_candidates = params.max_candidates.min(50 / usize::from(ipass));
        }
        let cands = search::coarse_search(work, sample_rate, nominal_start_sample, &pass_params);
        let subtract_after = npass > 1 && ipass < 4;
        let mut found: Vec<(f32, usize, [u8; 12])> = Vec::new();
        for c in cands {
            let decoded = match chase {
                Some(p) => {
                    chase::decode_at_with_chase(work, sample_rate, c.start_sample, c.freq_hz, p)
                }
                None => decode_at_with_snr(work, sample_rate, c.start_sample, c.freq_hz),
            };
            // `jt65_decode.f90:236-262`: a candidate the single period does
            // not decode is tried against the saved periods (`avg65`).
            let decoded = decoded.or_else(|| {
                let (av, period, ntol) = averager.as_mut()?;
                let demod = rx::demodulate_aligned(work, sample_rate, c.start_sample, c.freq_hz)?;
                let dt = (c.start_sample as f32 - nominal_start_sample as f32) / sample_rate as f32;
                let default_chase = chase::ChaseParams::default();
                av.try_average(
                    *period,
                    dt,
                    c.freq_hz,
                    *ntol,
                    demod,
                    chase.unwrap_or(&default_chase),
                )
            });
            let Some((msg, snr_db, info)) = decoded else {
                continue;
            };
            // `decode65b.f90:27`: the all-zero codeword is "no decode".
            if msg.is_all_zero_codeword() {
                continue;
            }
            let dup = scan_dedup_match_cross(
                &seen,
                &(msg.clone(), c.freq_hz, c.start_sample as i64),
                |r| &r.message,
                |r| r.freq_hz,
                // Saved results are in the caller's coordinates, candidates in the
                // padded ones (`pad` front silence): compare in the padded ones.
                // Without the `+ pad` no repeat ever matched and one strong frame
                // came back once per candidate around it (8 rows, #283 onwards).
                |r| (r.start_sample + pad) as i64,
                |(m, _, _)| m,
                |(_, f, _)| *f,
                |(_, _, t)| *t,
                // A strong frame decodes from every coarse candidate around it
                // (the Chase search tolerates a bin or two of misalignment): one
                // frame returned the same row 8 times at ±2 Hz. Upstream drops a
                // repeated text outright (`jt65_decode.f90:213-218`); 10 Hz keeps
                // two stations of one text apart (the golden's five carriers are
                // 400 Hz apart).
                10.0,
                nsps as i64,
            );
            if !dup {
                let result = Jt65Result {
                    message: msg,
                    freq_hz: c.freq_hz,
                    start_sample: c.start_sample.saturating_sub(pad),
                    // dt runs from the nominal start. `start_sample` is a
                    // `usize` and saturates at 0 for an early signal, so
                    // this field is the one that keeps the sign. Both terms
                    // are in padded coordinates — `nominal_start_sample` is
                    // shadowed with `+ pad` above — so the padding cancels
                    // and must not be subtracted again. It used to subtract
                    // `pad` alone and omit the nominal entirely, which was
                    // right only when the nominal was 0 (#397).
                    dt_sec: (c.start_sample as f32 - nominal_start_sample as f32)
                        / sample_rate as f32,
                    snr_db,
                };
                if let Some(cb) = on_result {
                    cb(&result);
                }
                seen.push(result);
                found.push((c.freq_hz, c.start_sample, info));
            }
        }
        if subtract_after && let Some(res) = residue.as_mut() {
            for (freq, start, info) in &found {
                let tones = tx::encode_channel_symbols(info);
                subtract::subtract65(res, *start, *freq, &tones, subtract::WSJTX);
            }
        }
    }
    seen
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
    /// (6-bit) in [`crate::engine::gray::gray`], not at the FSK-tone level. A
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

impl crate::engine::tx::FskWaveform for Jt65 {
    const WAVEFORM: crate::engine::tx::Waveform = crate::engine::tx::Waveform::Cpfsk;
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
    type SyncPhasors = ();
    const ID: ProtocolId = ProtocolId::Jt65;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::Jt72Message;

    #[test]
    fn erasure_assisted_decode_recovers_under_moderate_noise() {
        // Clean synth gets decoded by plain hard-decision RS; erasure path
        // is a strict superset so it should also work (trying 0 first).
        let freq = 1270.0;
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, freq, 0.3).expect("synth");
        let msg = DecodeRequest::sniper(&audio, 12_000, 0, freq)
            .erasures(&[0, 8, 16, 24, 32])
            .decode()
            .expect("erasure-aware path must decode clean synth");
        assert!(matches!(
            msg,
            Jt72Message::Standard { ref call1, ref call2, ref grid_or_report }
                if call1 == "CQ" && call2 == "K1ABC" && grid_or_report == "FN42"
        ));
    }

    /// `DecodeRequest::on_result` callback — synthetic
    /// round-trip verification, matching this module's own existing
    /// synth-test convention (no real-sample WSJT-X recording is
    /// wired for JT65 today — see `tests/jt65_sweep.rs`'s doc comment
    /// for why, it needs an out-of-tree `jt65sim` build). The scan
    /// is sequential with no early exit and no parallelism, so the
    /// callback-delivered set must exactly equal the batch `Vec`.
    #[test]
    fn on_result_matches_batch_exactly() {
        use std::sync::Mutex;

        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "JL1NIE", "PM95", 12_000, freq, 0.3).expect("synth");
        // Drop the signal 1 s into a zeroed slot so the scan must
        // actually search rather than decode at (start_sample=0).
        let mut slot = vec![0.0f32; 12_000 + audio.len()];
        slot[12_000..12_000 + audio.len()].copy_from_slice(&audio);

        let streamed_acc: Mutex<Vec<Jt72Message>> = Mutex::new(Vec::new());
        let on_result = |r: &Jt65Result| streamed_acc.lock().unwrap().push(r.message.clone());
        let batch = DecodeRequest::new(&slot, 12_000)
            .on_result(&on_result)
            .decode();
        let streamed = streamed_acc.into_inner().unwrap();
        let batch_msgs: Vec<Jt72Message> = batch.iter().map(|d| d.message.clone()).collect();
        assert_eq!(
            streamed, batch_msgs,
            "JT65 on_result: streamed callback deliveries must \
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

    /// `DecodeRequest::chase` end-to-end: proves the scan wiring actually
    /// reaches `chase::decode_at_with_chase` (not just that
    /// the function compiles) by requiring a genuine search — same
    /// "signal dropped 1s into a zeroed slot" shape as
    /// `on_result_matches_batch_exactly` above.
    #[test]
    fn chase_scan_finds_signal_via_search() {
        let freq = 1500.0;
        let audio = synthesize_standard("CQ", "JL1NIE", "PM95", 12_000, freq, 0.3).expect("synth");
        let mut slot = vec![0.0f32; 12_000 + audio.len()];
        slot[12_000..12_000 + audio.len()].copy_from_slice(&audio);

        let results = DecodeRequest::new(&slot, 12_000)
            .chase(chase::ChaseParams::default())
            .decode();
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
    fn chase_scan_never_false_decodes_on_noise() {
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
            let results = DecodeRequest::new(&audio, 12_000)
                .chase(chase::ChaseParams::default())
                .decode();
            assert!(
                results.is_empty(),
                "the chase scan must not decode pure noise (seed={seed}), got {results:?}"
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
