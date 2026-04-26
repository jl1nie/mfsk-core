// SPDX-License-Identifier: GPL-3.0-or-later
//! Sync layout — one Costas-4 block at the start of the frame.
//!
//! The Costas array `[0, 1, 3, 2]` is the same sequence FT4 uses for
//! its first sync block (`FT4_COSTAS_A`). Reusing a known-good
//! Costas pattern keeps the autocorrelation properties well-
//! characterised — a hostile or accidental tone sequence has a low
//! cross-correlation with this pattern, which means
//! [`crate::core::sync::coarse_sync`] picks the real frame start out
//! of noise reliably.
//!
//! For uvpacket the sync sits at symbol 0 only — there is no
//! mid-frame or end-of-frame sync. With Ldpc174_91's strong soft-
//! decision FEC and the Costas-4 at the head, a single sync block is
//! sufficient for symbol-rate-and-frequency lock; bit errors are
//! corrected by the FEC, not by re-acquiring sync mid-frame.

use crate::core::SyncBlock;

/// 4-symbol Costas-4 array. Same sequence as FT4's `FT4_COSTAS_A`.
pub const UVPACKET_SYNC_PATTERN: [u8; 4] = [0, 1, 3, 2];

/// Single sync block at symbol 0.
pub const UVPACKET_SYNC_BLOCKS: [SyncBlock; 1] = [SyncBlock {
    start_symbol: 0,
    pattern: &UVPACKET_SYNC_PATTERN,
}];
