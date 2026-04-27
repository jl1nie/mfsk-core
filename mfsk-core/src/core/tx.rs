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

/// Ordered list of `(first_data_symbol, chunk_len_in_symbols)` covering
/// every data slot in the frame — leading slots before the first sync
/// block, slots between consecutive sync blocks, and trailing slots
/// after the last sync block. Chunks of zero length are omitted.
///
/// FT8: `[(7, 29), (43, 29)]` — sync at 0/36/72, last block ends at the
/// end of the frame so there is no trailing chunk.
/// FT4: `[(4, 29), (37, 29), (70, 29)]` — same shape.
/// uvpacket: `[(4, 43), (51, 44)]` — sync at 0 and 47, the trailing
/// chunk picks up the data slots between block 1's tail (sym 51) and
/// the end of the 95-symbol frame.
pub fn data_chunks<P: Protocol>() -> Vec<(usize, usize)> {
    let blocks = P::SYNC_MODE.blocks();
    let n_sym = P::N_SYMBOLS as usize;
    let mut chunks: Vec<(usize, usize)> = Vec::with_capacity(blocks.len() + 1);

    // Leading data (before the first sync block). All existing
    // WSJT-family protocols put their first sync block at symbol 0
    // so this chunk is empty in practice; the branch is here for
    // protocols whose first sync sits later in the frame.
    if let Some(first) = blocks.first() {
        let pre = first.start_symbol as usize;
        if pre > 0 {
            chunks.push((0, pre));
        }
    } else if n_sym > 0 {
        // Pathological case: no sync blocks at all (shouldn't happen
        // for `SyncMode::Block`-using protocols). Treat the entire
        // frame as one data chunk so the helper degrades gracefully.
        chunks.push((0, n_sym));
        return chunks;
    }

    // Slots between consecutive sync blocks.
    for i in 0..blocks.len().saturating_sub(1) {
        let after = blocks[i].start_symbol as usize + blocks[i].pattern.len();
        let before_next = blocks[i + 1].start_symbol as usize;
        if before_next > after {
            chunks.push((after, before_next - after));
        }
    }

    // Trailing data (after the last sync block). FT8/FT4 put their
    // final sync at the tail so this is empty; uvpacket's mid-frame
    // sync layout produces a non-empty trailing chunk.
    if let Some(last) = blocks.last() {
        let after_last = last.start_symbol as usize + last.pattern.len();
        if n_sym > after_last {
            chunks.push((after_last, n_sym - after_last));
        }
    }

    chunks
}

/// Convert an LDPC codeword (MSB-first per symbol group) into the `N_SYMBOLS`
/// tone-index sequence. Sync blocks are slotted into their positions from
/// `Protocol::SYNC_BLOCKS`; data symbols consume `BITS_PER_SYMBOL` codeword
/// bits each, passed through the Gray map.
///
/// When [`P::CODEWORD_INTERLEAVE`](crate::core::FrameLayout::CODEWORD_INTERLEAVE)
/// is `Some`, the codeword bits are read in interleaved order: channel bit
/// position `j` gets `cw[INTERLEAVE[j]]`. This is the TX half of the
/// burst-error-tolerance scheme uvpacket uses; protocols with the default
/// `None` constant get the historical natural-order behaviour.
///
/// Panics if `cw.len() < total_data_symbols × BITS_PER_SYMBOL`.
pub fn codeword_to_itone<P: Protocol>(cw: &[u8]) -> Vec<u8> {
    let n_sym = P::N_SYMBOLS as usize;
    let bps = P::BITS_PER_SYMBOL as usize;
    let gray = P::GRAY_MAP;
    let interleave = P::CODEWORD_INTERLEAVE;

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
                let cw_idx = match interleave {
                    Some(table) => table[b + j] as usize,
                    None => b + j,
                };
                v = (v << 1) | (cw[cw_idx] & 1);
            }
            itone[start_sym + k] = gray[v as usize];
        }
        cw_offset += chunk_len * bps;
    }

    itone
}
