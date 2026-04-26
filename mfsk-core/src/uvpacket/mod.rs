// SPDX-License-Identifier: GPL-3.0-or-later
//! # `uvpacket` — AFSK/SSB packet protocol family for U/VHF
//!
//! A worked example of "what does adding a new `Protocol` look like?".
//! The PHY layer is intentionally simple — 4-FSK GFSK at three baud
//! rates, sharing one [`crate::fec::Ldpc174_91`] FEC instance, one
//! Costas-4 sync block, and one [`crate::msg::PacketBytesMessage`]
//! codec — so the reader can see the trait surface clearly without
//! drowning in protocol-specific complexity.
//!
//! ## What this protocol targets
//!
//! Amateur-radio packet data over **AFSK on SSB at U/VHF** (e.g. FT8
//! / FT4 / Q65 share the same modulation envelope). Multipath fading
//! environments — mobile or portable operation — where the dominant
//! channel impairment is **time-selective Rayleigh fading** with the
//! whole 3 kHz audio band fading together (flat across audio,
//! coherence bandwidth at U/VHF is MHz-class). Multiple concurrent
//! signals at different audio centre frequencies, like an FT8 channel.
//! No UTC slot — receivers run a sliding-window coarse search and
//! find sync wherever it appears.
//!
//! ## Why interleaver + binary LDPC, not Q65's metric
//!
//! UVHF flat fading produces **bursts of bad symbols in time** rather
//! than spectral spread per tone. The matched countermeasure is an
//! interleaver (spread codeword bits across time, so a deep fade hits
//! scattered bits rather than consecutive ones) plus binary LDPC.
//! Q65's `intrinsics_fast_fading` is calibrated for spectral spread
//! within a single tone (microwave EME / ionoscatter) and provides
//! no advantage on this channel. See `docs/ADDING_A_PROTOCOL.md` for
//! the full rationale.
//!
//! ## Sub-mode rate ladder
//!
//! All three sub-modes share the same FEC, message codec, sync
//! pattern, frame layout. The macro `uvpacket_submode!` emits one
//! ZST per (baud, NSPS, tone-spacing) tuple.
//!
//! | ZST            | Baud  | NSPS | Tone Δf | Audio BW | Gross | Net (R≈½) |
//! |----------------|-------|------|---------|----------|-------|-----------|
//! | [`UvPacket150`] | 150   | 80   | 150 Hz  | 600 Hz   |  300  |  150 bps  |
//! | [`UvPacket300`] | 300   | 40   | 300 Hz  | 1200 Hz  |  600  |  300 bps  |
//! | [`UvPacket600`] | 600   | 20   | 600 Hz  | 2400 Hz  | 1200  |  600 bps  |
//!
//! Net rates after Ldpc174_91's R = 91/174 ≈ 0.523 and the 4-bit
//! length overhead in [`crate::msg::PacketBytesMessage`].
//!
//! ## Frame structure
//!
//! - 4-symbol Costas-4 sync at the start of the frame
//!   ([`UVPACKET_SYNC_PATTERN`])
//! - 87 data symbols carrying one Ldpc174_91 codeword (174 bits ÷ 2
//!   bits-per-symbol)
//! - Total: 91 channel symbols
//!
//! Frame durations: 607 ms (150 baud) / 303 ms (300 baud) / 152 ms
//! (600 baud).
//!
//! ## Chain layer for >10-byte payloads
//!
//! Each frame carries 1–10 bytes (see [`crate::msg::PacketBytesMessage`]).
//! For longer messages — APRS frames, ECDSA-signed QSL exchanges,
//! short binary blobs — the [`chain`] module wraps a `&[u8]` of up to
//! ~576 bytes into a sequence of frames with a chain header (chain id +
//! sequence number + end marker). The chain decoder accepts frames in
//! any order and reassembles complete chains out of a multi-signal
//! audio buffer.
//!
//! ## See also
//!
//! - `docs/ADDING_A_PROTOCOL.md` — step-by-step walkthrough using
//!   this module as the example.
//! - `examples/signed-qsl/` — a real application built on top of
//!   uvpacket: ECDSA-signed QSL exchange that runs purely over
//!   amateur radio (no Internet / postal mail required for the
//!   authentic QSL).

pub mod chain;
pub mod protocol;
pub mod sync_pattern;
pub mod tx;
// `rx` lands in Phase 2 of `docs/0.3.0_PLAN.md` after the
// abstraction-leak fixes (Phases A + B) make
// `crate::core::pipeline::decode_frame::<P>` truly generic.

pub use protocol::{UvPacket150, UvPacket300, UvPacket600};
pub use sync_pattern::{UVPACKET_SYNC_BLOCKS, UVPACKET_SYNC_PATTERN};
pub use tx::{encode_to_tones, synthesize_audio_for, synthesize_packet};
