//! # JTTY — a non-slotted 4-GFSK mode for RTTY-style contest exchanges
//!
//! Ported from WSJT-X 3.2.0 (`lib/jtty/`, tag `v3.2.0-rc1`). A transmission
//! may start at any time and is one or more self-contained 1.888 s frames;
//! there is no T/R slot, so — like MSK144 — JTTY has no
//! [`Protocol`](crate::engine::protocol::Protocol) marker type and is not in
//! [`registry::PROTOCOLS`](crate::registry::PROTOCOLS).
//!
//! Tracking issue #477; the upstream reading, design decisions and phased
//! plan are in `docs/notes/JTTY_UPSTREAM.md`. What exists:
//!
//! - **the wire level** (P1): the source grammar, the CRC-12, the tail-biting
//!   convolutional encoder and the transmit waveform;
//! - **the frame decoder** (P2): [`rx::Receiver`] takes audio and returns the
//!   validated frames in it, a faithful port of WSJT-X's `jtty_mdecode` *without*
//!   signal subtraction, retro re-sweep or message assembly (P3). On upstream's
//!   own recordings it decodes the same frames at the same frequencies and
//!   times, scores the same recall in every cell of an AWGN/fading sweep, and —
//!   being the same algorithm — also makes the same false decodes (see
//!   `docs/notes/JTTY_UPSTREAM.md`, "P2 results").
//!
//! With the `parallel` feature the independent work — frames, waveform
//! synthesis, the 237 sync-surface columns per window, the candidates of a window,
//! the rungs of the decode ladder, the windows of a recording — runs on rayon's
//! pool; every parallel step collects in order, so results do not depend on the
//! thread count.
//!
//! ## Frame
//!
//! ```text
//! 32-bit source word ── + reserved bit (always 0) ── + EOM bit ──▶ 34 bit
//!   34 bit ── + CRC-12 ──▶ 46 bit ── tail-biting conv. code R=1/2, K=10 ──▶ 46 tones
//!   13 sync tones + 46 data tones = 59 symbols = 1.888 s at 31.25 baud
//! ```
//!
//! | module | contents | upstream |
//! |---|---|---|
//! | [`source`] | the 32-bit grammar: [`source::Atom`] ⇄ word ⇄ text, validity | `jtty_source_codec.f90`, `jtty_source_encoding.txt` |
//! | [`correlate`] | complex correlation of the 46 data symbols with the four tone references, and the half-symbol energies | `jtty_payload_correlators.f90` |
//! | [`crc`] | CRC-12 over the 34-bit payload | `tbcc.f90` (`encode_crc12`), `jtty_tbcc_list_decoder.f90` (`jtty_tbcc_crc_valid`) |
//! | [`tbcc`] | the convolutional encoder and its trellis step | `tbcc.f90` (`tbcc_encode`), `jtty_tbcc_code_profile.f90` |
//! | `dsp` | analytic signal at 6 kHz, sync waveform, frequency shift (needs an FFT feature) | `ana64a.f90`, `gen_syncwave.f90`, `twkfreq.f90` |
//! | [`ladder`] | the four-rung decode ladder around it | `jtty_tbcc_decoder.f90` |
//! | `rx` | window → frames: sync surface, candidates, peak-up, gate, decode, validate (needs an FFT feature) | `jtty_mdecode.f90`, `jtty_peakup.f90` |
//! | [`trellis`] | list-WAVA decoding of that code from tone correlations | `jtty_tbcc_list_decoder.f90` |
//! | [`tx`] | atoms → tones → GFSK samples | `genjtty.f90`, `gen_jttywave.f90` |
//!
//! ## Text packing is not here
//!
//! Upstream's `pack_jtty` (turn a typed message into the minimum number of
//! atoms, with exchange profiles) is host UI policy and phase P5. Callers
//! here build [`source::Atom`]s directly.

pub mod correlate;
pub mod crc;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod dsp;
pub mod ladder;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod rx;
pub mod source;
pub mod tbcc;
pub mod trellis;
pub mod tx;

/// Samples per symbol at [`SAMPLE_RATE`] (`nsps = 384`; 31.25 baud).
pub const NSPS: usize = 384;
/// Audio sample rate of every JTTY routine in this crate, Hz.
pub const SAMPLE_RATE: f32 = 12_000.0;
/// Bits in the payload before the CRC: 32 source + reserved + EOM.
pub const PAYLOAD_BITS: usize = 34;
/// CRC bits appended to the payload.
pub const CRC_BITS: usize = 12;
/// Bits fed to the convolutional encoder (`PAYLOAD_BITS + CRC_BITS`), which is
/// also the number of data tones per frame (rate 1/2, two bits per tone).
pub const INFO_BITS: usize = PAYLOAD_BITS + CRC_BITS;
/// Sync tones at the start of every frame.
pub const SYNC_SYMBOLS: usize = 13;
/// Tones per frame: sync plus data.
pub const FRAME_SYMBOLS: usize = SYNC_SYMBOLS + INFO_BITS;
/// Most frames a message may span (`MAX_FRAMES` upstream).
pub const MAX_FRAMES: usize = 16;

/// The 13-symbol sync sequence (`is13` in `jtty_fec_mod.f90`); the peak
/// sidelobe of its autocorrelation is 2/13.
pub const SYNC: [u8; SYNC_SYMBOLS] = [0, 2, 2, 3, 0, 0, 3, 2, 1, 3, 1, 2, 0];

/// One frame's payload, bit 1 first (`[0]` is bit 1, `[32]` the reserved bit,
/// `[33]` the end-of-message flag) — the order upstream's `character*34` frames
/// are written and the convolutional encoder consumes them.
pub type Payload = [u8; PAYLOAD_BITS];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_arithmetic() {
        assert_eq!(INFO_BITS, 46);
        assert_eq!(FRAME_SYMBOLS, 59);
        // 59 symbols × 384 samples / 12 kHz = 1.888 s
        assert_eq!(FRAME_SYMBOLS * NSPS, 22_656);
        assert!((FRAME_SYMBOLS as f32 * NSPS as f32 / SAMPLE_RATE - 1.888).abs() < 1e-6);
    }
}
