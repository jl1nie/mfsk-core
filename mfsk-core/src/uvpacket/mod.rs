// SPDX-License-Identifier: GPL-3.0-or-later
//! # `uvpacket` — AFSK / FM-data packet protocol family for U/VHF
//!
//! A worked example of "what does adding a new `Protocol` look like?".
//! The PHY layer is intentionally simple — 4-FSK GFSK at four baud
//! rates, sharing one [`crate::fec::Ldpc174_91`] FEC instance, two
//! Costas-4 sync blocks, one [`crate::msg::PacketBytesMessage`] codec
//! and one bit interleaver — so the reader can see the trait surface
//! clearly without drowning in protocol-specific complexity.
//!
//! ## What this protocol targets
//!
//! Amateur-radio packet data over **AFSK on SSB** at U/VHF for the
//! 150 / 300 / 600 baud sub-modes (BW ≤ 2.4 kHz fits the SSB voice
//! passband) and **FM data channels** for [`UvPacket1200`] (BW
//! ≈ 4.8 kHz, beyond SSB but well within a 12.5 kHz FM channel).
//!
//! Mobile / portable conditions are the design driver — **time-
//! selective Rayleigh fading** with the whole 3 kHz audio band fading
//! together (flat across audio; coherence bandwidth at U/VHF is
//! MHz-class). Multiple concurrent transmitters at different audio
//! centre frequencies are normal, like an FT8 channel. There is no
//! UTC slot — receivers run a sliding-window coarse search and find
//! sync wherever it appears.
//!
//! ## Burst-error tolerance
//!
//! Two mechanisms work together:
//!
//! 1. **Bit interleaver** — [`UVPACKET_INTERLEAVE`] permutes the 174
//!    LDPC codeword bits before they hit the channel so that
//!    consecutive codeword bits land 25 channel positions apart.
//!    A deep fade null spanning, say, 50 ms at UvPacket150 (≈ 7.5
//!    symbols = 14 channel bits) decoheres into 14 codeword-bit
//!    losses spread across the full 174-bit word — sparse enough
//!    for soft-decision LDPC to reconstruct.
//!
//! 2. **Two sync blocks** — Costas-4 patterns at symbol 0 (head) and
//!    symbol 47 (middle of the 95-symbol frame). A fade null that
//!    swallows the head sync still leaves the mid-frame sync intact
//!    for [`crate::core::sync::coarse_sync`] to lock onto, and vice
//!    versa.
//!
//! Compare with the alternatives: Q65's per-tone spectral-spread
//! metric (`intrinsics_fast_fading`) is calibrated for spectral spread
//! within a single tone (microwave EME / ionoscatter) and provides no
//! advantage on flat-band time-selective fading; FT8's three Costas-7
//! blocks plus its per-symbol Hamming-distance LLRs target a similar
//! channel shape but at a slower 6.25 baud / 50 Hz tone spacing.
//! uvpacket sits in the middle: faster than FT8, more burst-tolerant
//! than a packet protocol with no interleaver.
//!
//! ## Sub-mode rate ladder
//!
//! All four sub-modes share the same FEC, message codec, sync
//! pattern, frame layout, interleaver. The macro `uvpacket_submode!`
//! emits one ZST per (baud, NSPS, tone-spacing) tuple.
//!
//! | ZST              | Baud  | NSPS | Tone Δf | Audio BW | Gross | Net (R≈½) | Frame   | Channel        |
//! |------------------|-------|------|---------|----------|-------|-----------|---------|----------------|
//! | [`UvPacket150`]  |   150 |   80 |  150 Hz |  600 Hz  |  300  |  150 bps  |  633 ms | weak SSB       |
//! | [`UvPacket300`]  |   300 |   40 |  300 Hz | 1200 Hz  |  600  |  300 bps  |  317 ms | typical SSB    |
//! | [`UvPacket600`]  |   600 |   20 |  600 Hz | 2400 Hz  | 1200  |  600 bps  |  158 ms | clean SSB      |
//! | [`UvPacket1200`] |  1200 |   10 | 1200 Hz | 4800 Hz  | 2400  | 1.2 kbps  |   79 ms | FM data        |
//!
//! Net rates after Ldpc174_91's R = 91 / 174 ≈ 0.523 and the 1-byte
//! length / CRC-7 overhead in [`crate::msg::PacketBytesMessage`].
//! The 12 kHz pipeline sample rate caps practical baud at ≈ 1200
//! before Nyquist starts cutting tone separation; for higher rates a
//! wider sample rate (24 / 48 kHz) and / or higher-order modulation
//! (PSK / QAM) would be needed — out of scope for this protocol.
//!
//! ## Frame structure
//!
//! - 4 sync symbols (Costas-4) at symbol 0
//! - 43 data symbols (interleaved codeword bits 0..86)
//! - 4 sync symbols (Costas-4) at symbol 47
//! - 44 data symbols (interleaved codeword bits 87..173)
//! - Total: **95 channel symbols**, carrying one Ldpc174_91 codeword
//!   (174 bits ÷ 2 bits-per-symbol = 87 data symbols)
//!
//! ## Chain layer for >10-byte payloads
//!
//! Each frame carries 1–10 bytes (see [`crate::msg::PacketBytesMessage`]).
//! For longer messages — APRS frames, ECDSA-signed QSL exchanges,
//! short binary blobs — the [`chain`] module wraps a `&[u8]` of up to
//! ~512 bytes into a sequence of frames with a chain header (chain id +
//! sequence number + end marker). The chain decoder accepts frames in
//! any order and reassembles complete chains out of a multi-signal
//! audio buffer.
//!
//! ## Quick start
//!
//! ```no_run
//! # #[cfg(feature = "uvpacket")] {
//! use mfsk_core::uvpacket::{
//!     UvPacket600,
//!     tx::synthesize_packet,
//!     rx::decode_frame,
//! };
//!
//! // TX: encode a small payload as a 12 kHz f32 waveform, scale to i16.
//! let pcm_f32 = synthesize_packet::<UvPacket600>(b"hello", 1500.0, 0.5)
//!     .expect("payload fits");
//! let mut audio: Vec<i16> = pcm_f32
//!     .iter()
//!     .map(|&s| (s * 20_000.0) as i16)
//!     .collect();
//! audio.resize(12_000, 0); // pad to 1 s
//!
//! // RX: scan the 1 s buffer for any uvpacket frames in 100..3000 Hz.
//! let frames = decode_frame::<UvPacket600>(
//!     &audio,
//!     /* freq_min */ 100.0,
//!     /* freq_max */ 3_000.0,
//!     /* sync_min */ 1.0,
//!     /* max_cand */ 50,
//! );
//! assert!(!frames.is_empty(), "round-trip should decode");
//! assert_eq!(frames[0].payload, b"hello");
//! # }
//! ```
//!
//! ## See also
//!
//! - `docs/UVPACKET.md` — channel model, design rationale, multipath
//!   fading test results.
//! - `docs/ADDING_A_PROTOCOL.md` — step-by-step walkthrough using
//!   this module as the example.
//! - `examples/signed-qsl/` — a real application built on top of
//!   uvpacket: ECDSA-signed QSL exchange that runs purely over
//!   amateur radio (no Internet / postal mail required for the
//!   authentic QSL).

pub mod chain;
pub mod protocol;
pub mod rx;
pub mod sync_pattern;
pub mod tx;

pub use protocol::{UVPACKET_INTERLEAVE, UvPacket150, UvPacket300, UvPacket600, UvPacket1200};
pub use rx::{DEFAULT_AUDIO_SAMPLES, DecodedPacket, decode_frame, downsample_cfg_for};
pub use sync_pattern::{UVPACKET_SYNC_BLOCKS, UVPACKET_SYNC_PATTERN};
pub use tx::{encode_to_tones, synthesize_audio_for, synthesize_packet};
