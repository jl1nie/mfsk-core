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
//! Currently wired sub-modes: **Q65-30A** ([`Q65a30`]) for
//! terrestrial weak-signal work plus the EME band lineup
//! [`Q65a60`] / [`Q65b60`] / [`Q65c60`] / [`Q65d60`] / [`Q65e60`]
//! (60 s T/R with tone-spacing multipliers ×1, ×2, ×4, ×8, ×16).
//! Generic `synthesize_standard_for<P>`, `decode_at_for<P>`,
//! `decode_scan_for<P>` helpers pick up the right NSPS / tone
//! spacing from the type parameter; `decode_at_with_ap_for<P>` /
//! `decode_scan_with_ap_for<P>` accept an [`crate::msg::ApHint`]
//! that biases the QRA decoder with known call-sign / report
//! information and lifts the effective decode threshold by ~2 dB.
//!
//! References:
//! - WSJT-X `lib/qra/q65/q65.f90`, `lib/qra/q65/q65.c`,
//!   `lib/qra/q65/genq65.f90`, `lib/qra/q65/qra15_65_64_irr_e23.c`,
//!   `lib/q65_decode.f90`, `lib/q65params.f90`.
//! - Joe Taylor K1JT, "The Q65 Protocol for Weak-Signal
//!   Communication", QEX 2022.

pub mod protocol;
pub mod rx;
pub mod search;
pub mod sync_pattern;
pub mod tx;

pub use protocol::{Q65Fec, Q65a30, Q65a60, Q65b60, Q65c60, Q65d60, Q65e60};
pub use rx::{
    Q65Decode, decode_at, decode_at_for, decode_at_with_ap, decode_at_with_ap_for, decode_scan,
    decode_scan_default, decode_scan_for, decode_scan_with_ap, decode_scan_with_ap_for,
};
pub use search::{SearchParams, SyncCandidate, coarse_search};
pub use sync_pattern::{Q65_DATA_POSITIONS, Q65_SYNC_BLOCKS, Q65_SYNC_POSITIONS};
pub use tx::{encode_channel_symbols, synthesize_audio, synthesize_standard};
