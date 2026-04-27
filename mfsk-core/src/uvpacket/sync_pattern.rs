// SPDX-License-Identifier: GPL-3.0-or-later
//! Sync layout — two Costas-4 blocks at the start and middle of the frame.
//!
//! The Costas array `[0, 1, 3, 2]` is the same sequence FT4 uses for
//! its first sync block (`FT4_COSTAS_A`). Reusing a known-good
//! Costas pattern keeps the autocorrelation properties well-
//! characterised — a hostile or accidental tone sequence has a low
//! cross-correlation with this pattern, which means
//! [`crate::core::sync::coarse_sync`] picks the real frame start out
//! of noise reliably.
//!
//! ## Why two blocks
//!
//! Mobile / portable U/VHF operation hits **time-selective Rayleigh
//! fading**: deep nulls of tens of milliseconds that can swallow an
//! entire 4-symbol sync block whole at the slower sub-modes. With a
//! single sync block at the frame head, a fade null hitting symbols
//! 0..3 makes coarse sync miss the frame entirely. Adding a second
//! block at symbol 47 (mid-frame for the 95-symbol layout) gives the
//! receiver a second chance: even when the first block is faded
//! out, the mid-frame block produces a sync correlation peak the
//! coarse search can latch onto, and the LDPC + bit-interleaver
//! combination handles the data-bit losses.
//!
//! Both blocks share the same Costas-4 pattern; the corresponding
//! per-position correlations sum coherently when both are clean and
//! work independently when one is faded. FT8 makes the same choice
//! for its three Costas-7 blocks.

use crate::core::SyncBlock;

/// 4-symbol Costas-4 array. Same sequence as FT4's `FT4_COSTAS_A`.
pub const UVPACKET_SYNC_PATTERN: [u8; 4] = [0, 1, 3, 2];

/// Two sync blocks: head (symbol 0) and middle (symbol 47).
///
/// With `N_DATA = 87` evenly split into 43 + 44 symbols on either
/// side of the mid-frame block, the second Costas falls at the
/// middle of the 95-symbol frame.
pub const UVPACKET_SYNC_BLOCKS: [SyncBlock; 2] = [
    SyncBlock {
        start_symbol: 0,
        pattern: &UVPACKET_SYNC_PATTERN,
    },
    SyncBlock {
        start_symbol: 47,
        pattern: &UVPACKET_SYNC_PATTERN,
    },
];
