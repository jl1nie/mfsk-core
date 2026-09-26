// SPDX-License-Identifier: GPL-3.0-or-later
//! Q65 transmitter: 77-bit WSJT message → 85 channel tones → audio.
//!
//! Mirrors WSJT-X `lib/qra/q65/genq65.f90` (encoder) +
//! `lib/qra/q65/q65sim.f90` (waveform synthesiser):
//!
//! 1. Pack the 77-bit Wsjt77 message into 13 GF(64) symbols
//!    ([`crate::msg::q65::pack77_to_symbols`]).
//! 2. Run the QRA encoder ([`Q65Codec::encode`]) to get 63 channel
//!    symbols (the two CRC symbols are computed and punctured
//!    inside `Q65Codec`).
//! 3. Splice into the 85-slot frame: sync positions emit tone 0,
//!    data positions emit `sent[k] + 1` (Q65 reserves tone 0 for
//!    sync, so data symbols `0..=63` map to tones `1..=64`).
//! 4. Synthesise plain FSK (no GFSK shaping) at the Q65-30A baud.

use alloc::vec::Vec;

use crate::fec::qra::Q65Codec;
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::q65::pack77_q65;
use crate::msg::q65::pack77_to_symbols_flagged;

use super::Q65a30;
use super::sync_pattern::Q65_SYNC_POSITIONS;

/// Encode a 77-bit Wsjt77 message into the 85 channel tones a Q65
/// transmitter emits.
///
/// Tone values: `0` at the 22 sync positions, `1..=64` at the 63
/// data positions (corresponding QRA symbol values `0..=63`).
pub fn encode_channel_symbols(bits77: &[u8; 77]) -> [u8; 85] {
    encode_channel_symbols_flagged(bits77, false)
}

/// [`encode_channel_symbols`] with the spare 78th bit set to
/// `copied_last_tx` — WSJT-X 3.2's Q65 Pileup "copied last Tx" flag
/// (`genq65.f90`'s `iflag`; see [`crate::msg::q65::pack77_to_symbols_flagged`]).
pub fn encode_channel_symbols_flagged(bits77: &[u8; 77], copied_last_tx: bool) -> [u8; 85] {
    // 77 bits → 13 GF(64) info symbols (last symbol is 5 bits + the
    // spare bit — done inside `pack77_to_symbols_flagged`).
    let info_syms = pack77_to_symbols_flagged(bits77, copied_last_tx);

    // Q65 FEC encode: 13 user info → +CRC12 → QRA → puncture → 63 ch.
    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut sent = [0_i32; 63];
    codec.encode(&info_syms, &mut sent);

    // Splice into the 85-symbol frame, prefilled with sync (tone 0)
    // and overwriting data positions with `sent[k] + 1`.
    let mut tones = [0u8; 85];
    let mut sync_iter = Q65_SYNC_POSITIONS.iter().peekable();
    let mut k = 0usize;
    for i in 0..85u32 {
        if sync_iter.peek().is_some_and(|&&p| p == i) {
            sync_iter.next();
            tones[i as usize] = 0; // sync tone
        } else {
            let sym = sent[k];
            debug_assert!(
                (0..64).contains(&sym),
                "QRA symbol {sym} out of GF(64) range"
            );
            tones[i as usize] = (sym as u8) + 1;
            k += 1;
        }
    }
    debug_assert_eq!(k, 63, "exactly 63 data slots must be filled");
    tones
}

/// Pack a standard `<call1> <call2> <grid_or_report>` Wsjt77 message
/// and synthesise the Q65 audio for sub-mode `P`.
pub fn synthesize_standard_for<P: crate::engine::tx::FskWaveform>(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    synthesize_standard_flagged_for::<P>(
        call1,
        call2,
        grid_or_report,
        false,
        sample_rate,
        base_freq_hz,
        amplitude,
    )
}

/// [`synthesize_standard_for`] with WSJT-X 3.2's Q65 Pileup "copied last
/// Tx" flag — see [`encode_channel_symbols_flagged`].
pub fn synthesize_standard_flagged_for<P: crate::engine::tx::FskWaveform>(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    copied_last_tx: bool,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let bits = pack77_q65(call1, call2, grid_or_report)?;
    let tones = encode_channel_symbols_flagged(&bits, copied_last_tx);
    Some(crate::engine::tx::synthesize::<P>(
        &tones,
        sample_rate,
        base_freq_hz,
        amplitude,
    ))
}

/// Q65-30A convenience wrapper for [`synthesize_standard_for`].
pub fn synthesize_standard(
    call1: &str,
    call2: &str,
    grid_or_report: &str,
    sample_rate: u32,
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    synthesize_standard_for::<Q65a30>(
        call1,
        call2,
        grid_or_report,
        sample_rate,
        base_freq_hz,
        amplitude,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_produces_22_sync_63_data_tones() {
        let bits = [0u8; 77];
        let tones = encode_channel_symbols(&bits);
        let sync_count = tones.iter().filter(|&&t| t == 0).count();
        assert_eq!(sync_count, 22, "expected exactly 22 sync tones (tone 0)");
        let data_count = tones.iter().filter(|&&t| (1..=64).contains(&t)).count();
        assert_eq!(data_count, 63, "expected exactly 63 data tones (1..=64)");
    }

    #[test]
    fn sync_tones_appear_at_isync_positions() {
        let bits = [0u8; 77];
        let tones = encode_channel_symbols(&bits);
        for &p in &Q65_SYNC_POSITIONS {
            assert_eq!(
                tones[p as usize], 0,
                "frame slot {p} must carry the sync tone"
            );
        }
    }

    #[test]
    fn synthesized_length_matches_85_symbols() {
        let tones = [0u8; 85];
        let audio =
            crate::engine::tx::synthesize::<crate::q65::Q65a30>(&tones, 12_000, 1500.0, 0.3);
        assert_eq!(
            audio.len(),
            3600 * 85,
            "Q65-30A audio length = nsps * 85 = 3600 * 85 = 306 000 samples"
        );
    }

    #[test]
    fn synthesize_standard_message_succeeds() {
        let audio = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3)
            .expect("pack + synth must succeed for a standard message");
        assert_eq!(audio.len(), 3600 * 85);
        // Sanity: the synthesised waveform must contain non-zero
        // energy.
        let max_abs = audio
            .iter()
            .copied()
            .fold(0.0_f32, |acc, v| acc.max(v.abs()));
        assert!(max_abs > 0.1, "synthesised audio appears silent");
    }

    #[test]
    fn data_tones_in_valid_range() {
        // A non-trivial message should produce data tones spread
        // across the 1..=64 range.
        let bits77 = crate::msg::wsjt77::pack77("CQ", "JA1ABC", "PM95").expect("pack");
        let tones = encode_channel_symbols(&bits77);
        for (i, &t) in tones.iter().enumerate() {
            assert!(t <= 64, "tone[{i}] = {t} exceeds Q65 range 0..=64");
        }
    }
}
