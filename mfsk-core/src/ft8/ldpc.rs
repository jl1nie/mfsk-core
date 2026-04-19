//! Re-export of the shared LDPC(174,91) codec from `mfsk-fec`.
//!
//! Kept as a façade module so existing `ft8-core::ldpc::{bp,osd,tables}`
//! callers continue to resolve. New code should prefer
//! [`crate::fec::Ldpc174_91`] (via the `FecCodec` trait) directly.

pub use crate::fec::ldpc::{bp, osd, tables};
pub use crate::fec::ldpc::{BpResult, OsdResult, bp_decode, ldpc_encode, osd_decode_deep};
