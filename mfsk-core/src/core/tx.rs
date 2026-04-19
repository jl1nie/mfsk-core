//! Protocol-generic transmit-side helpers: message bits → tone sequence.
//!
//! The tone sequence assembly (slot Costas arrays into their positions, map
//! LDPC codeword bits into per-symbol Gray-coded tone indices) is protocol-
//! agnostic given [`Protocol`]: iterate `SYNC_BLOCKS` and fill each data
//! chunk between consecutive blocks with `chunk_len × BITS_PER_SYMBOL` bits.
//!
//! GFSK waveform synthesis lives in [`super::dsp::gfsk`] — the `tones_to_*`
//! helpers there consume the output of this module.

use super::Protocol;

/// Ordered list of `(first_data_symbol, chunk_len_in_symbols)`.
///
/// FT8: `[(7, 29), (43, 29)]`. FT4: `[(4, 29), (37, 29), (70, 29)]`.
pub fn data_chunks<P: Protocol>() -> Vec<(usize, usize)> {
    let blocks = P::SYNC_MODE.blocks();
    let mut chunks = Vec::with_capacity(blocks.len().saturating_sub(1));
    for i in 0..blocks.len().saturating_sub(1) {
        let after = blocks[i].start_symbol as usize + blocks[i].pattern.len();
        let before_next = blocks[i + 1].start_symbol as usize;
        if before_next > after {
            chunks.push((after, before_next - after));
        }
    }
    chunks
}

/// Convert an LDPC codeword (MSB-first per symbol group) into the `N_SYMBOLS`
/// tone-index sequence. Sync blocks are slotted into their positions from
/// [`Protocol::SYNC_BLOCKS`]; data symbols consume `BITS_PER_SYMBOL` codeword
/// bits each, passed through the Gray map.
///
/// Panics if `cw.len() < total_data_symbols × BITS_PER_SYMBOL`.
pub fn codeword_to_itone<P: Protocol>(cw: &[u8]) -> Vec<u8> {
    let n_sym = P::N_SYMBOLS as usize;
    let bps = P::BITS_PER_SYMBOL as usize;
    let gray = P::GRAY_MAP;

    let mut itone = vec![0u8; n_sym];

    for block in P::SYNC_MODE.blocks() {
        let start = block.start_symbol as usize;
        for (i, &c) in block.pattern.iter().enumerate() {
            itone[start + i] = c;
        }
    }

    let chunks = data_chunks::<P>();
    let mut cw_offset = 0usize;
    for (start_sym, chunk_len) in chunks {
        for k in 0..chunk_len {
            let b = cw_offset + k * bps;
            let mut v = 0u8;
            for j in 0..bps {
                v = (v << 1) | (cw[b + j] & 1);
            }
            itone[start_sym + k] = gray[v as usize];
        }
        cw_offset += chunk_len * bps;
    }

    itone
}
