//! JT65 pseudo-random sync pattern (the `nprc` / `pr` vector in
//! WSJT-X `setup65.f90`). 126 bits, 63 ones (sync positions) and 63
//! zeros (data positions).
//!
//! Both TX and RX use this mask to interleave data and sync symbols
//! across the frame: at position `i` where `NPRC[i] == 1` the
//! transmitter emits tone 0 (sync), and where `NPRC[i] == 0` it
//! emits one of the 64 data tones.

use crate::core::SyncBlock;

/// Raw 126-bit sync pattern, as transcribed from WSJT-X `nprc`.
pub const JT65_NPRC: [u8; 126] = [
    1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1,
    1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1,
    0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0,
    1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
];

/// Positions (0-indexed) where `JT65_NPRC[i] == 1` — the 63 sync slots.
/// Data symbols fill the remaining 63 slots in frame order.
pub const JT65_SYNC_POSITIONS: [u32; 63] = {
    let mut out = [0u32; 63];
    let mut i = 0usize;
    let mut k = 0usize;
    while i < 126 {
        if JT65_NPRC[i] == 1 {
            out[k] = i as u32;
            k += 1;
        }
        i += 1;
    }
    out
};

/// Data positions (0-indexed) where `JT65_NPRC[i] == 0`.
pub const JT65_DATA_POSITIONS: [u32; 63] = {
    let mut out = [0u32; 63];
    let mut i = 0usize;
    let mut k = 0usize;
    while i < 126 {
        if JT65_NPRC[i] == 0 {
            out[k] = i as u32;
            k += 1;
        }
        i += 1;
    }
    out
};

/// Expected tone at each sync position (always tone 0).
const SYNC_TONE: [u8; 1] = [0];

/// 63 single-symbol sync blocks. Expressing distributed sync as
/// length-1 `SyncBlock` entries keeps JT65 on the existing
/// `SyncMode::Block` variant without introducing a new enum case.
pub const JT65_SYNC_BLOCKS: [SyncBlock; 63] = {
    let mut blocks = [SyncBlock {
        start_symbol: 0,
        pattern: &SYNC_TONE,
    }; 63];
    let mut i = 0usize;
    while i < 63 {
        blocks[i] = SyncBlock {
            start_symbol: JT65_SYNC_POSITIONS[i],
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
    fn sync_and_data_positions_partition_126() {
        assert_eq!(JT65_SYNC_POSITIONS.len(), 63);
        assert_eq!(JT65_DATA_POSITIONS.len(), 63);
        // Union = 0..126, no overlap.
        let mut seen = [false; 126];
        for &p in &JT65_SYNC_POSITIONS {
            assert!(!seen[p as usize], "duplicate sync pos {p}");
            seen[p as usize] = true;
        }
        for &p in &JT65_DATA_POSITIONS {
            assert!(!seen[p as usize], "duplicate data pos {p}");
            seen[p as usize] = true;
        }
        assert!(seen.iter().all(|&b| b));
    }
}
