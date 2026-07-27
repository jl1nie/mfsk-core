//! # `msk144` — MSK144 meteor-scatter mode (issue #25, in progress)
//!
//! MSK144 is fundamentally different from the FT/JT/Q-family modes
//! this crate is built around: it's continuous-phase binary MSK
//! (offset-QPSK) rather than M-ary FSK, and its decode loop scans a
//! 15 s slot for 50-200ms ionised-trail bursts that can start almost
//! anywhere, rather than assuming one frame at a known offset. See
//! `crate::engine::dsp::msk` for the OQPSK waveform primitives and
//! `crate::fec::Ldpc128_90` for the FEC layer — both are reused as-is
//! by [`tx`]; the burst-scan sync search and top-level decode driver
//! are still to come.
//!
//! Payload reuses the same `packjt77` 77-bit message format FT8/FT4/
//! FST4 use (`crate::msg::wsjt77`) plus a 13-bit CRC
//! (`crate::fec::ldpc_128_90::check_crc13`) — MSK144 needs no bespoke
//! message codec.

#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod decode;
pub mod frame_decode;
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub mod spd;
pub mod sync;
pub mod tx;
