//! Re-export of the WSJT 77-bit message codec from `mfsk-msg`.
//!
//! The actual implementation lives in [`crate::msg::wsjt77`]. This façade exists
//! so existing `ft8-core::message::{pack77_*, unpack77, …}` callers keep
//! working during the migration. New code should import from `mfsk-msg`
//! directly.

pub use crate::msg::wsjt77::*;
