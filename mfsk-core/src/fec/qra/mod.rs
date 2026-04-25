//! # `fec::qra` — Q-ary Repeat-Accumulate (QRA) LDPC codecs
//!
//! Generic encoder + soft-decision belief-propagation decoder for the
//! QRA LDPC code family designed by Nico Palermo (IV3NWV) and used as
//! the FEC layer of WSJT-X's Q65 mode.
//!
//! Unlike the bit-LDPC codes in [`super::ldpc`] (binary parity, BP in
//! the LLR domain), QRA codes operate over **GF(2^m)** symbol
//! alphabets — for Q65 specifically, GF(64) — and run BP in the
//! **probability domain** with per-edge messages of length M=2^m.
//! Variable-to-check messages exploit the Walsh-Hadamard transform
//! ([`npfwht`]) to efficiently combine M-ary distributions across
//! repeat-accumulate parity constraints.
//!
//! ## Layout
//!
//! - [`pdmath`] — probability-vector arithmetic (init, normalise,
//!   pointwise multiply / divide, hard decision).
//! - [`npfwht`] — non-binary Walsh-Hadamard transform of size M.
//! - [`code`]   — generic [`QraCode`] encoder + BP decoder, parameterised
//!   on a code description ([`QraCodeDesc`]).
//! - Concrete codes live in their own modules outside this one
//!   ([`super::qra15_65_64`] for `qra15_65_64_irr_e23`, the Q65 code).
//!
//! ## Reference
//!
//! Ported from <https://github.com/ja7eee/qracodes> and the Q65
//! integration in WSJT-X `lib/qra/q65/`.

pub mod code;
pub mod npfwht;
pub mod pdmath;
pub mod q65;

pub use code::{DecoderScratch, ExtrinsicResult, QraCode, QraCodeType};
pub use q65::{Q65Codec, Q65DecodeError};
