//! JT9 sync-symbol layout.
//!
//! Ported verbatim from WSJT-X `lib/jt9sync.f90`. Each of 16 sync
//! symbols sits at a fixed position in the 85-symbol frame and
//! carries tone 0 (the dedicated sync tone below the 8 data tones).
//!
//! The positions are 0-indexed here (`1,2,5,…` → `0,1,4,…`).

use crate::core::SyncBlock;

/// 0-indexed symbol positions of the 16 sync symbols within the
/// 85-symbol JT9 frame.
pub const JT9_SYNC_POSITIONS: [u32; 16] =
    [0, 1, 4, 9, 15, 22, 32, 34, 50, 51, 54, 59, 65, 72, 82, 84];

/// Expected tone at each sync position (JT9 always uses tone 0 as the
/// sync reference — the "1-tone-below-data" marker).
const SYNC_TONE: [u8; 1] = [0];

/// 16 single-symbol sync blocks, each expecting tone 0. Expressing
/// distributed sync as 16 length-1 `SyncBlock` entries lets JT9
/// reuse the existing `SyncMode::Block` variant without introducing
/// a new enum case.
pub const JT9_SYNC_BLOCKS: [SyncBlock; 16] = [
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[0],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[1],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[2],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[3],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[4],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[5],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[6],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[7],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[8],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[9],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[10],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[11],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[12],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[13],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[14],
        pattern: &SYNC_TONE,
    },
    SyncBlock {
        start_symbol: JT9_SYNC_POSITIONS[15],
        pattern: &SYNC_TONE,
    },
];

/// The 85-element `isync` vector (1 = sync symbol, 0 = data symbol).
/// Useful for the demodulator when it wants to walk the frame and
/// decide whether each symbol carries data or is a sync reference.
pub const JT9_ISYNC: [u8; 85] = [
    1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_positions_count() {
        assert_eq!(JT9_SYNC_POSITIONS.len(), 16);
        assert_eq!(JT9_SYNC_BLOCKS.len(), 16);
    }

    #[test]
    fn isync_matches_positions() {
        let expected_sync: Vec<u32> = (0..85).filter(|&i| JT9_ISYNC[i as usize] == 1).collect();
        assert_eq!(expected_sync, &JT9_SYNC_POSITIONS[..]);
    }
}
