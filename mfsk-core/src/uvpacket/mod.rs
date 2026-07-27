// SPDX-License-Identifier: GPL-3.0-or-later
//! # `uvpacket` — applied example: a packet protocol for ham VHF / UHF
//!
//! **Scope note.** This module is **not** a member of the WSJT-X
//! mode family that the rest of `mfsk-core` ports. It is an in-tree
//! *applied example* of how the FEC infrastructure
//! ([`crate::fec::Ldpc240_101`], BP, OSD-2) can be reused outside
//! that family. It targets a different design point — narrow-FM /
//! voice-channel-SSB packet messaging for private amateur-radio
//! groups — and consequently diverges from WSJT-X assumptions in
//! every layer above the FEC.
//!
//! ## 0.4.0 redesign — what's different
//!
//! The 0.3.x line was a single-carrier coherent QPSK modem with a
//! 31-bit BPSK m-sequence preamble + periodic QPSK pilots, decoded
//! by a brute-force receiver that tried every `(mode × n_blocks)`
//! layout combination per sync peak (~128 LDPC decode attempts in
//! the worst case). Over-the-air loopback testing showed the
//! coherent demod failed under realistic SSB / FM impairments and
//! the brute-force decoder was structurally incorrect (no a-priori
//! way to know `(mode, n_blocks)` from sync alone).
//!
//! 0.4.0 rebuilds the whole stack:
//!
//! | Layer | 0.3.x | 0.4.0 |
//! |---|---|---|
//! | Modulation | coherent QPSK + pilots | **π/4-DQPSK**, no pilots |
//! | Sync | 31-chip BPSK m-sequence (mode-agnostic) | **127-chip m-sequence × 4 variants**, one per mode |
//! | Mode discovery | brute force LDPC decode | preamble-pattern → mode in one MF pass |
//! | Frame structure | spread header across LDPC blocks | **dedicated Robust header block** + payload |
//! | Equaliser | none | 9-tap T-spaced LS-trained on long preamble |
//! | LDPC decodes / frame | up to 128 (brute force) | **`1 + n_blocks`** (1 header + n payload) |
//!
//! The dedicated header block reads `(block_count, app_type,
//! sequence)` + CRC-16; the receiver knows the mode from sync, so
//! the header word doesn't carry it.
//!
//! ## Sub-modes (all share modem + preamble + FEC mother code)
//!
//! - [`UvRobust`] — `Ldpc240_101` native rate 0.42, 1008 net bps.
//!   Mountain / weak-signal posture.
//! - [`UvStandard`] — punctured to rate 1/2, 1200 net bps. Typical
//!   NFM with fading.
//! - [`UvUltraRobust`] — rate 2/3, 1600 net bps (+33 %).
//! - [`UvExpress`] — rate 3/4, 1800 net bps (+50 %). OSD-2 essentially
//!   mandatory at the BP threshold.
//!
//! ## Modulation
//!
//! - **π/4-shifted DQPSK** at 1200 baud, RRC pulse (α = 0.5,
//!   span 6 sym, 10 samples per symbol at 12 kHz).
//! - Audio centre 1700 Hz by default ([`AUDIO_CENTRE_HZ`]) — clears
//!   typical NFM HT 300 Hz HPF and 2.7 kHz LPF.
//! - Differential demodulation: `r_diff[k] = e[k]·conj(e[k-1])` on
//!   the equalised matched-filter output, then a -π/4 rotation
//!   lands the four `Δφ ∈ {±π/4, ±3π/4}` values on the standard
//!   QPSK constellation axes for [`crate::uvpacket::rx`]'s
//!   soft-demap to LDPC LLRs.
//!
//! ## Frame structure (on the wire)
//!
//! ```text
//! [ 127-chip BPSK preamble — variant identifies the Mode ]
//! [ Header LDPC block — Robust, Ldpc240_101 unpunctured ]
//! [ Payload LDPC blocks × n_blocks — at the frame Mode ]
//! ```
//!
//! - 4-byte header: `block_count (5b) + app_type (4b) + sequence (5b)
//!   + reserved (2b) + CRC-16 (16b)`. CRC covers
//!   `header_word ++ padded_payload`.
//! - Payload is variable: 1..=32 LDPC blocks × 12 byte each
//!   (96 info bits per block; the 5 spare bits in the LDPC info
//!   slot are zero-pad).
//! - Block-interleaver across all payload blocks spreads fade-burst
//!   erasures.
//!
//! ## Application API
//!
//! Byte pipe — bypasses [`crate::engine::MessageCodec`]. Callers
//! deliver raw bytes plus a 4-bit `app_type` tag; the modem
//! doesn't know or care what's inside.
//!
//! ```ignore
//! use mfsk_core::uvpacket::{tx, rx, AUDIO_CENTRE_HZ, Mode};
//! use mfsk_core::uvpacket::framing::FrameHeader;
//!
//! let header = FrameHeader {
//!     mode: Mode::Robust,
//!     block_count: 4,
//!     app_type: 1,
//!     sequence: 0,
//! };
//! let audio = tx::encode(&header, payload, AUDIO_CENTRE_HZ).unwrap();
//! for frame in rx::decode(&audio, AUDIO_CENTRE_HZ) {
//!     // dispatch on frame.app_type / frame.sequence ...
//! }
//! ```
//!
//! ## Empirical performance (post-redesign)
//!
//! Robust mode 50 % PER thresholds on the in-tree air-channel sims
//! (`tests/common/air_channel.rs`):
//!
//! | Channel | Eb/N0_info |
//! |---|---|
//! | AWGN | ~+6 dB |
//! | SSB mid-stress (clarifier 100 Hz + LO walk 2 rad/√s + 5 ms reverb) | ~+10 dB |
//! | SSB true-harsh (clarifier 250 Hz + walk 5 + multi-tap MP, wide AFC) | ~+12 dB |
//! | FM true-harsh (de-emphasis + drift 250 Hz + Rician K=8 + multi-tap MP) | ~+15 dB |
//!
//! The differential-demod path costs ~5 dB threshold loss vs the
//! old coherent path on AWGN, paid back many times over by
//! surviving real-channel impairments where the coherent path
//! scored 0/30.

pub mod framing;
pub mod interleaver;
pub mod message;
pub mod protocol;
pub mod puncture;
pub mod rx;
pub mod sync_pattern;
pub mod tx;

pub use message::UvPacketRawMessage;
pub use protocol::{AUDIO_CENTRE_HZ, UvExpress, UvRobust, UvStandard, UvUltraRobust};
pub use puncture::Mode;
