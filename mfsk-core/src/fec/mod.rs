//! # `fec` — forward-error-correction codecs
//!
//! Forward-error-correction codecs shared across WSJT-family protocols.
//!
//! Each codec implements [`crate::engine::FecCodec`] so generic pipeline code can
//! treat it uniformly. Protocol crates pick the codec via the
//! `type Fec = …;` associated type on [`crate::engine::Protocol`].
//!
//! ## Contents
//!
//! | Family                   | Module          | Shared by               |
//! |--------------------------|-----------------|-------------------------|
//! | LDPC (174, 91) + CRC-14  | [`ldpc`]        | FT8, FT4                |
//! | LDPC (240, 101) + CRC-24 | [`ldpc240_101`] | FST4, FST4W             |
//! | LDPC (128, 90) + CRC-13  | [`ldpc_128_90`] | MSK144                  |
//! | Convolutional r=1/2 K=32 | [`conv`]        | WSPR, JT9               |
//! | Reed-Solomon (63, 12)    | [`rs`]          | JT65                    |

pub mod conv;
pub mod ldpc;
pub mod ldpc240_101;
pub mod ldpc_128_90;
pub mod rs;

#[cfg(feature = "q65")]
pub mod qra;
#[cfg(feature = "q65")]
pub mod qra15_65_64;

pub use conv::{ConvFano, ConvFano232};
pub use ldpc::Ldpc174_91;
pub use ldpc_128_90::Ldpc128_90;
pub use ldpc240_101::Ldpc240_101;
pub use rs::Rs63_12;

// Re-export the FecCodec trait so callers can name it via [`crate::fec::FecCodec`]
// alongside the concrete codec types.
pub use crate::engine::FecCodec;
