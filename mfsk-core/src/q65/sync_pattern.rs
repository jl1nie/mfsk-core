//! Q65 distributed sync pattern.
//!
//! Q65 transmits 22 sync symbols (always tone 0) at fixed positions
//! across the 85-symbol frame; the remaining 63 positions carry the
//! channel symbols of the QRA codeword (offset by +1 — Q65 symbol
//! value `s` is sent on tone `s + 1`, since tone 0 is reserved for
//! sync).
//!
//! The sync vector below is the 1-indexed `isync(22)` array from
//! WSJT-X `lib/qra/q65/q65.f90` (line ~10), translated to 0-indexed
//! positions.

use crate::core::SyncBlock;

/// 0-indexed positions (within the 85-symbol frame) of the 22 sync
/// symbols. Translated from the Fortran `isync(1..22)` constant in
/// `lib/qra/q65/q65.f90`:
/// `(1, 9, 12, 13, 15, 22, 23, 26, 27, 33, 35, 38, 46, 50, 55, 60,
/// 62, 66, 69, 74, 76, 85)` — each value reduced by one for Rust's
/// 0-based indexing.
pub const Q65_SYNC_POSITIONS: [u32; 22] = [
    0, 8, 11, 12, 14, 21, 22, 25, 26, 32, 34, 37, 45, 49, 54, 59, 61, 65, 68, 73, 75, 84,
];

/// Complement of [`Q65_SYNC_POSITIONS`]: the 63 frame slots that carry
/// data symbols, in ascending order. Computed at compile time so the
/// receiver / synthesiser can iterate it without re-deriving the
/// complement at runtime.
pub const Q65_DATA_POSITIONS: [u32; 63] = {
    let mut out = [0u32; 63];
    let mut k = 0usize;
    let mut i = 0u32;
    while i < 85 {
        // Linear "is i a sync position?" probe — N=22 is small enough
        // that an inner loop over `Q65_SYNC_POSITIONS` is the simplest
        // const-fn-friendly form.
        let mut is_sync = false;
        let mut j = 0usize;
        while j < 22 {
            if Q65_SYNC_POSITIONS[j] == i {
                is_sync = true;
                break;
            }
            j += 1;
        }
        if !is_sync {
            out[k] = i;
            k += 1;
        }
        i += 1;
    }
    out
};

/// Single-element pattern: every sync position emits tone 0.
const SYNC_TONE: [u8; 1] = [0];

/// 22 single-symbol [`SyncBlock`] entries, expressing Q65's distributed
/// sync via the existing `SyncMode::Block` variant — same trick used
/// for JT65 in [`crate::jt65::sync_pattern`].
pub const Q65_SYNC_BLOCKS: [SyncBlock; 22] = {
    let mut blocks = [SyncBlock {
        start_symbol: 0,
        pattern: &SYNC_TONE,
    }; 22];
    let mut i = 0usize;
    while i < 22 {
        blocks[i] = SyncBlock {
            start_symbol: Q65_SYNC_POSITIONS[i],
            pattern: &SYNC_TONE,
        };
        i += 1;
    }
    blocks
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sync_and_data_positions_partition_85() {
        // Sync (22) + data (63) must cover every position in 0..85
        // exactly once.
        let mut seen = [false; 85];
        for &p in &Q65_SYNC_POSITIONS {
            assert!(p < 85, "sync pos {p} out of range");
            assert!(!seen[p as usize], "duplicate sync pos {p}");
            seen[p as usize] = true;
        }
        for &p in &Q65_DATA_POSITIONS {
            assert!(p < 85, "data pos {p} out of range");
            assert!(!seen[p as usize], "duplicate data pos {p}");
            seen[p as usize] = true;
        }
        assert!(seen.iter().all(|&b| b), "some position uncovered");
    }

    #[test]
    fn sync_positions_are_strictly_ascending() {
        // The Fortran source lists them sorted; our translation
        // should preserve that ordering — many call-sites assume it.
        for w in Q65_SYNC_POSITIONS.windows(2) {
            assert!(w[0] < w[1], "sync positions not ascending: {w:?}");
        }
    }

    #[test]
    fn sync_blocks_match_positions() {
        // Each block must be a length-1 entry on tone 0 at the
        // corresponding sync position.
        assert_eq!(Q65_SYNC_BLOCKS.len(), 22);
        for (i, block) in Q65_SYNC_BLOCKS.iter().enumerate() {
            assert_eq!(block.start_symbol, Q65_SYNC_POSITIONS[i]);
            assert_eq!(block.pattern, &[0u8]);
        }
    }

    #[test]
    fn data_positions_strictly_ascending() {
        for w in Q65_DATA_POSITIONS.windows(2) {
            assert!(w[0] < w[1], "data positions not ascending: {w:?}");
        }
    }
}
