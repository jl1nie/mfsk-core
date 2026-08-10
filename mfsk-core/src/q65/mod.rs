//! # `q65` — Q65 decoder and synthesiser
//!
//! Q65 is the modern WSJT-X weak-signal mode (introduced in WSJT-X
//! 2.4.0, 2021) for EME, ionoscatter, meteor scatter and other very
//! low-SNR paths. Designed by Joe Taylor K1JT and Steve Franke K9AN
//! around the QRA (Q-ary Repeat-Accumulate) LDPC code family by Nico
//! Palermo IV3NWV.
//!
//! Distinguishing characteristics:
//! - **65-tone FSK** (1 sync tone + 64 data tones), plain FSK (no
//!   GFSK shaping).
//! - **`qra15_65_64_irr_e23`** Q-ary LDPC code over GF(64) — see
//!   [`crate::fec::qra15_65_64`]. Two CRC-12 symbols are appended to
//!   the 13 information symbols, then punctured before transmission,
//!   yielding 63 transmitted data symbols.
//! - **22 distributed sync symbols** all on tone 0, at fixed positions
//!   `isync = (0, 8, 11, 12, 14, 21, 22, 25, 26, 32, 34, 37, 45, 49,
//!   54, 59, 61, 65, 68, 73, 75, 84)` (0-indexed).
//! - **77-bit Wsjt77 message** padded with one bit to 78 bits,
//!   matching the FT8/FT4 message layer.
//! - **Native soft-decision** belief-propagation decoder over GF(64)
//!   using Walsh-Hadamard transforms in the probability domain.
//!
//! Sub-modes vary along two orthogonal axes:
//! - **T/R period**: 15, 30, 60, 120, 300 s (controls `nsps`).
//! - **Tone-spacing letter** A–E: spacing = baud × 2^(letter-1)
//!   (controls bandwidth / Doppler tolerance).
//!
//! Currently wired sub-modes: **Q65-15A** ([`Q65a15`]) and
//! **Q65-30A** ([`Q65a30`]) for terrestrial weak-signal work; the
//! EME band lineup [`Q65a60`] / [`Q65b60`] / [`Q65c60`] / [`Q65d60`] /
//! [`Q65e60`] (60 s T/R with tone-spacing multipliers ×1, ×2, ×4, ×8,
//! ×16); and three longer-period scatter modes: [`Q65d120`] (10 GHz
//! rainscatter/troposcatter), [`Q65e120`] (6 m ionoscatter with wider
//! Doppler), and [`Q65a300`] (optical scatter — the deepest wired
//! sub-mode, ~-34 dB AWGN threshold).
//! Generic `synthesize_standard_for<P>` picks up the right NSPS / tone
//! spacing from the type parameter. Decoding goes through the
//! [`DecodeRequest`]/[`SniperRequest`]/[`MultiPeriodRequest`] builders
//! (issue #204 — replaces the pre-#191-style `decode_at*`/
//! `decode_scan*`/`decode_multi_period*` suffix explosion `q65::rx`
//! used to expose as 15 separate public functions):
//! - [`DecodeRequest::new`] — wide-band scan; [`DecodeRequest::sniper`]
//!   narrows to a single known `(start_sample, base_freq_hz)`
//!   alignment ([`SniperRequest`]).
//! - `.ap_hint(...)` biases the QRA decoder with a known call-sign /
//!   report [`crate::msg::ApHint`], lifting the effective decode
//!   threshold by ~2 dB.
//! - `.fading(model, b90_ts)` swaps in the fast-fading metric
//!   ([`crate::fec::qra::intrinsics_fast_fading`]), recovering the
//!   5–8 dB the AWGN Bessel front end loses to Doppler-spread channels
//!   — required for microwave EME (libration spread ≥10 Hz at
//!   5.7 GHz and above).
//! - `.ap_list(candidates)` runs BP-free **template matching** against
//!   a candidate codeword list (typically built with
//!   [`standard_qso_codewords`]) instead of belief propagation — the
//!   mfsk-core port of `q65_decode_fullaplist`. Useful when the
//!   application has a known callsign pair but no QSO context.
//! - [`MultiPeriodRequest`] averages energies across multiple T/R
//!   slots for ionoscatter / weak-EME signals that no single-period
//!   decode can recover.
//!
//! References:
//! - WSJT-X `lib/qra/q65/q65.f90`, `lib/qra/q65/q65.c`,
//!   `lib/qra/q65/genq65.f90`, `lib/qra/q65/qra15_65_64_irr_e23.c`,
//!   `lib/q65_decode.f90`, `lib/q65params.f90`.
//! - Joe Taylor K1JT, "The Q65 Protocol for Weak-Signal
//!   Communication", QEX 2022.

pub mod ap_list;
pub mod decode_request;
pub mod protocol;
pub mod rx;
pub mod search;
pub(crate) mod snr;
pub mod sync_pattern;
pub mod tx;

pub use ap_list::{MAX_AP_CODEWORDS, standard_qso_codewords};
pub use decode_request::{DecodeRequest, MultiPeriodRequest, Q65SubMode, SniperRequest};
pub use protocol::{
    Q65Fec, Q65a15, Q65a30, Q65a60, Q65a300, Q65b60, Q65c60, Q65d60, Q65d120, Q65e60, Q65e120,
};
pub use rx::Q65Result;
pub use search::{SearchParams, SyncCandidate, coarse_search};
pub use sync_pattern::{Q65_DATA_POSITIONS, Q65_SYNC_BLOCKS, Q65_SYNC_POSITIONS};
pub use tx::{
    encode_channel_symbols, synthesize_audio, synthesize_audio_for, synthesize_standard,
    synthesize_standard_for,
};
