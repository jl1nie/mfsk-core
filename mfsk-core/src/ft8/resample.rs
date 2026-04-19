//! Re-export of the generic linear resampler from `mfsk-core::dsp::resample`.
//!
//! The implementation is protocol-agnostic (no FT8 constants), so it lives in
//! `mfsk-core` and is shared by every MFSK protocol crate. Existing callers
//! that reach for `crate::ft8::resample::resample_to_12k` keep working via this
//! façade.

pub use crate::core::dsp::resample::{resample_f32_to_12k, resample_to_12k};
