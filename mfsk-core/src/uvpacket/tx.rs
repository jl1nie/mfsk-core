// SPDX-License-Identifier: GPL-3.0-or-later
//! TX path: bytes → 12 kHz f32 PCM audio (post 0.4.0 redesign).
//!
//! ## Frame structure
//!
//! ```text
//! [ 127-chip BPSK preamble (mode-encoded) ]
//! [ Header LDPC block — Robust, Ldpc240_101 unpunctured, 12 byte info ]
//! [ Payload LDPC blocks × n_blocks — at the frame mode ]
//! ```
//!
//! - **Mode** is conveyed by the preamble pattern selection (4
//!   distinct 127-chip m-sequences, one per [`Mode`]). The receiver
//!   identifies mode at sync time so it can decode header and
//!   payload at the right rate without trial-and-error.
//! - **Header block** is always Robust (unpunctured `Ldpc240_101`)
//!   and carries `(block_count, app_type, sequence)` plus CRC-16.
//!   The header block is the first thing the receiver decodes after
//!   sync; once it has `block_count` it knows exactly how many
//!   payload blocks follow.
//! - **Payload blocks** use the mode-specific puncture pattern from
//!   [`crate::uvpacket::puncture::puncture`]. They are
//!   block-interleaved (the existing
//!   [`crate::uvpacket::interleaver::interleave`] is reused) so a
//!   fade burst spreads its erasures across every payload codeword.
//!
//! ## Modulation
//!
//! - **π/4-shifted DQPSK** at 1200 baud. Each data symbol's
//!   absolute phase = previous symbol's phase + Δφ ∈ {±π/4, ±3π/4}.
//!   The differential chain seeds from the last preamble chip
//!   (BPSK ±1).
//! - **No pilots.** Differential demod doesn't need per-segment
//!   phase anchors.
//! - **RRC pulse**, α = 0.5, span 6 sym, 10 samples per symbol.
//! - Upconvert to [`super::AUDIO_CENTRE_HZ`] (1700 Hz default).

use std::f32::consts::PI;

use num_complex::Complex32;

use crate::engine::FecCodec;
use crate::fec::Ldpc240_101;

use super::framing::{FrameHeader, HEADER_BYTES, INFO_BYTES_PER_BLOCK, PackError, pack_header};
use super::interleaver::interleave;
use super::puncture::{Mode, puncture};
use super::sync_pattern::{PREAMBLE_LEN, preamble_for};

/// LDPC mother-codeword length.
const N_LDPC: usize = 240;
/// LDPC info-bit count.
const K_LDPC: usize = 101;
/// Payload bits per block (12 byte = 96 of the 101 LDPC info bits).
const PAYLOAD_BITS_PER_BLOCK: usize = INFO_BYTES_PER_BLOCK * 8;

/// Sample rate (Hz).
pub(super) const SAMPLE_RATE_HZ: f32 = 12_000.0;
/// RRC pulse span (symbols on each side of centre tap). Same for
/// every mode — only the per-symbol sample count varies via
/// [`Mode::nsps`].
pub(super) const RRC_SPAN_SYMS: usize = 6;
/// RRC roll-off factor.
pub(super) const RRC_ALPHA: f32 = 0.5;

/// π/4-shifted DQPSK Δφ table indexed by `pair = (b1<<1)|b0`
/// (Gray-adjacent in angle): pair 00 → +π/4, 01 → +3π/4,
/// 10 → -π/4, 11 → -3π/4.
const PI4_DQPSK_DELTA: [f32; 4] = [PI / 4.0, 3.0 * PI / 4.0, -PI / 4.0, -3.0 * PI / 4.0];

/// Total transmitted symbol count for a given mode + payload block
/// count: 127-sym preamble + 120-sym header block + n_payload_blocks
/// × (mode.ch_bits_per_block / 2) data symbols.
pub fn expected_total_symbols(mode: Mode, n_payload_blocks: u8) -> usize {
    let header_sym = N_LDPC / 2;
    let payload_sym = (n_payload_blocks as usize) * mode.ch_bits_per_block() / 2;
    PREAMBLE_LEN + header_sym + payload_sym
}

/// Output sample count for [`encode`] / [`encode_into`] given a frame
/// `mode` + `n_payload_blocks`. Embedded callers allocate (or claim
/// from a pool) `encode_output_len(mode, n_payload_blocks)` samples
/// and pass them as `out`.
#[inline]
pub fn encode_output_len(mode: Mode, n_payload_blocks: u8) -> usize {
    let nsps = mode.nsps();
    let rrc_len = RRC_SPAN_SYMS * nsps + 1;
    expected_total_symbols(mode, n_payload_blocks) * nsps + rrc_len
}

/// Encode a uvpacket frame to 12 kHz f32 PCM audio.
///
/// `header.mode` selects both the preamble variant and the
/// payload-block puncture pattern. `header.block_count` is the
/// number of **payload** LDPC blocks (1..=32); the dedicated header
/// block at the front is implicit and not counted in this field.
///
/// `audio_centre_hz` is the carrier frequency to upconvert to
/// (typically [`super::AUDIO_CENTRE_HZ`] = 1700 Hz).
pub fn encode(
    header: &FrameHeader,
    payload: &[u8],
    audio_centre_hz: f32,
) -> Result<Vec<f32>, PackError> {
    let mut audio = vec![0.0f32; encode_output_len(header.mode, header.block_count)];
    encode_into(&mut audio, header, payload, audio_centre_hz)?;
    Ok(audio)
}

/// Encode a uvpacket frame into a caller-provided audio buffer.
/// **No allocation of the output** — `out` must be sized to
/// [`encode_output_len`]`(header.mode, header.block_count)`. Internal
/// scratch buffers (LDPC info / codeword, interleaver staging, RRC
/// pulse, complex baseband) are still allocated; they total ~few-tens
/// of KB and matter little next to the I2S DMA buffer the caller
/// owns.
///
/// # Errors / panics
///
/// Returns `PackError::PayloadTooLarge` if the payload exceeds the
/// frame's capacity. Panics if `out.len() != encode_output_len(mode,
/// block_count)`.
pub fn encode_into(
    out: &mut [f32],
    header: &FrameHeader,
    payload: &[u8],
    audio_centre_hz: f32,
) -> Result<(), PackError> {
    let mode = header.mode;
    let n_blocks = header.block_count as usize;
    let payload_capacity = n_blocks * INFO_BYTES_PER_BLOCK;
    if payload.len() > payload_capacity {
        return Err(PackError::PayloadTooLarge(payload.len()));
    }
    assert_eq!(
        out.len(),
        encode_output_len(mode, header.block_count),
        "encode_into: out.len() must equal encode_output_len()"
    );

    // 1. Build header bytes (4-byte header word + CRC). The CRC
    //    covers `header_word ++ padded_payload` so the receiver can
    //    verify after concatenating decoded payload-block info.
    let mut padded_payload = vec![0u8; payload_capacity];
    padded_payload[..payload.len()].copy_from_slice(payload);
    let header_bytes = pack_header(header, &padded_payload)?;

    // 2. Header LDPC block info: 4 byte header + 8 byte zero pad
    //    (96 bits of info, padded to the 101-bit LDPC input slot).
    let mut header_info_bytes = [0u8; INFO_BYTES_PER_BLOCK];
    header_info_bytes[..HEADER_BYTES].copy_from_slice(&header_bytes);

    // 3. LDPC-encode header block (Robust, unpunctured).
    let fec = Ldpc240_101;
    let mut info_buf = vec![0u8; K_LDPC];
    let mut codeword_buf = vec![0u8; N_LDPC];
    bytes_to_bits_msb(&header_info_bytes, &mut info_buf[..PAYLOAD_BITS_PER_BLOCK]);
    fec.encode(&info_buf, &mut codeword_buf);
    let header_codeword: Vec<u8> = codeword_buf.clone();

    // 4. LDPC-encode + puncture every payload block.
    let block_ch_bits = mode.ch_bits_per_block();
    let mut interleaver_in: Vec<u8> = Vec::with_capacity(n_blocks * block_ch_bits);
    for block_idx in 0..n_blocks {
        let chunk = &padded_payload
            [block_idx * INFO_BYTES_PER_BLOCK..(block_idx + 1) * INFO_BYTES_PER_BLOCK];
        bytes_to_bits_msb(chunk, &mut info_buf[..PAYLOAD_BITS_PER_BLOCK]);
        // The 5 trailing info bits are zero pad (already cleared by
        // bytes_to_bits_msb writing only the first 96).
        for b in &mut info_buf[PAYLOAD_BITS_PER_BLOCK..] {
            *b = 0;
        }
        fec.encode(&info_buf, &mut codeword_buf);
        interleaver_in.extend_from_slice(&puncture(&codeword_buf, mode));
    }

    // 5. Block-interleave payload-only (header is standalone).
    let interleaved = interleave(&interleaver_in, n_blocks);

    // 6. Concatenate header codeword + interleaved payload bits to
    //    form the on-air channel-bit stream (excluding preamble).
    let mut data_bits: Vec<u8> = Vec::with_capacity(N_LDPC + interleaved.len());
    data_bits.extend_from_slice(&header_codeword);
    data_bits.extend_from_slice(&interleaved);

    // 7. Map bit pairs to π/4-DQPSK Δφ table indices.
    debug_assert!(data_bits.len().is_multiple_of(2));
    let n_data_syms = data_bits.len() / 2;
    let mut deltas: Vec<f32> = Vec::with_capacity(n_data_syms);
    for sym_idx in 0..n_data_syms {
        let pair = ((data_bits[sym_idx * 2] << 1) | data_bits[sym_idx * 2 + 1]) as usize;
        deltas.push(PI4_DQPSK_DELTA[pair]);
    }

    // 8. Build symbol stream: [mode preamble] + [data, differentially
    //    encoded from the last preamble chip].
    let preamble_bits = preamble_for(mode);
    let mut symbols: Vec<Complex32> = Vec::with_capacity(PREAMBLE_LEN + n_data_syms);
    for &b in preamble_bits.iter() {
        symbols.push(Complex32::new(if b { -1.0 } else { 1.0 }, 0.0));
    }
    let mut prev = symbols[symbols.len() - 1];
    for &delta in &deltas {
        let next = prev * Complex32::from_polar(1.0, delta);
        symbols.push(next);
        prev = next;
    }

    // 9. RRC pulse-shape into a complex baseband. The pulse width
    //    (and per-symbol stride) is mode-dependent: UltraRobust runs
    //    at half baud and uses NSPS=20, every other mode uses NSPS=10.
    let nsps = mode.nsps();
    let rrc = rrc_pulse(RRC_ALPHA, RRC_SPAN_SYMS, nsps);
    let total_samples = symbols.len() * nsps + rrc.len();
    let mut baseband = vec![Complex32::new(0.0, 0.0); total_samples];
    for (i, &sym) in symbols.iter().enumerate() {
        let start = i * nsps;
        for (j, &tap) in rrc.iter().enumerate() {
            let pos = start + j;
            if pos < baseband.len() {
                baseband[pos] += sym * tap;
            }
        }
    }

    // 10. Upconvert to audio centre, take real part — written into the
    //     caller-provided buffer.
    let two_pi_fc_dt = 2.0 * PI * audio_centre_hz / SAMPLE_RATE_HZ;
    for n in 0..total_samples {
        let phase = two_pi_fc_dt * n as f32;
        let (s, c) = phase.sin_cos();
        out[n] = baseband[n].re * c - baseband[n].im * s;
    }

    // 11. Peak-normalise to ≤ 1 (the σ-for-Eb/N0 formula assumes
    //     unit peak; RRC + sum-of-symbols can briefly overshoot).
    let peak = out.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
    if peak > 1.0 {
        let scale = 1.0 / peak;
        for s in out.iter_mut() {
            *s *= scale;
        }
    }
    Ok(())
}

/// Pack `bytes` MSB-first into the leading `8 × bytes.len()` slots
/// of `bits`. The remainder of `bits` (if any) is left unchanged.
fn bytes_to_bits_msb(bytes: &[u8], bits: &mut [u8]) {
    debug_assert!(bits.len() >= bytes.len() * 8);
    for (byte_idx, &byte) in bytes.iter().enumerate() {
        for bit_idx in 0..8 {
            bits[byte_idx * 8 + bit_idx] = (byte >> (7 - bit_idx)) & 1;
        }
    }
}

/// Generate root-raised-cosine pulse coefficients. Returns
/// `span_syms × samples_per_sym + 1` taps, normalised so `Σ h² = 1`.
pub(super) fn rrc_pulse(alpha: f32, span_syms: usize, samples_per_sym: usize) -> Vec<f32> {
    let n = span_syms * samples_per_sym;
    let mut h = vec![0.0_f32; n + 1];
    let center = n as f32 / 2.0;
    for (i, h_i) in h.iter_mut().enumerate() {
        let t = (i as f32 - center) / samples_per_sym as f32;
        *h_i = if t.abs() < 1e-6 {
            1.0 - alpha + 4.0 * alpha / PI
        } else if (t.abs() - 1.0 / (4.0 * alpha)).abs() < 1e-6 {
            (alpha / 2.0_f32.sqrt())
                * ((1.0 + 2.0 / PI) * (PI / (4.0 * alpha)).sin()
                    + (1.0 - 2.0 / PI) * (PI / (4.0 * alpha)).cos())
        } else {
            let pi_t = PI * t;
            let four_at = 4.0 * alpha * t;
            ((pi_t * (1.0 - alpha)).sin() + four_at * (pi_t * (1.0 + alpha)).cos())
                / (pi_t * (1.0 - four_at * four_at))
        };
    }
    let norm: f32 = h.iter().map(|x| x * x).sum::<f32>().sqrt();
    if norm > 0.0 {
        for x in h.iter_mut() {
            *x /= norm;
        }
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::uvpacket::AUDIO_CENTRE_HZ;

    fn header_for(mode: Mode, n_blocks: u8) -> FrameHeader {
        FrameHeader {
            mode,
            block_count: n_blocks,
            app_type: 1,
            sequence: 0,
        }
    }

    #[test]
    fn encode_succeeds_all_modes() {
        for mode in [
            Mode::Robust,
            Mode::Standard,
            Mode::UltraRobust,
            Mode::Express,
        ] {
            for n_blocks in [1u8, 4, 18, 32] {
                let header = header_for(mode, n_blocks);
                let cap = (n_blocks as usize) * INFO_BYTES_PER_BLOCK;
                let payload = vec![0xA5_u8; cap];
                let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
                assert!(!audio.is_empty(), "{mode:?} n={n_blocks}: empty audio");
            }
        }
    }

    #[test]
    fn encode_peak_amplitude_bounded() {
        let header = header_for(Mode::Robust, 4);
        let audio = encode(&header, b"hello", AUDIO_CENTRE_HZ).unwrap();
        let peak = audio.iter().map(|s| s.abs()).fold(0.0_f32, f32::max);
        assert!(peak <= 1.0001, "peak {peak} > 1");
    }

    #[test]
    fn encode_sample_count_matches_formula() {
        for mode in [
            Mode::Robust,
            Mode::Standard,
            Mode::UltraRobust,
            Mode::Express,
        ] {
            let n_blocks = 4u8;
            let header = header_for(mode, n_blocks);
            let cap = (n_blocks as usize) * INFO_BYTES_PER_BLOCK;
            let payload = vec![0xCC_u8; cap];
            let audio = encode(&header, &payload, AUDIO_CENTRE_HZ).unwrap();
            let n_syms = expected_total_symbols(mode, n_blocks);
            let nsps = mode.nsps();
            let rrc_len = RRC_SPAN_SYMS * nsps + 1;
            let expected = n_syms * nsps + rrc_len;
            assert_eq!(
                audio.len(),
                expected,
                "{mode:?}: got {} samples, expected {}",
                audio.len(),
                expected,
            );
        }
    }

    #[test]
    fn distinct_payloads_diverge() {
        let header = header_for(Mode::Robust, 4);
        let a = encode(&header, b"alpha", AUDIO_CENTRE_HZ).unwrap();
        let b = encode(&header, b"bravo", AUDIO_CENTRE_HZ).unwrap();
        assert_eq!(a.len(), b.len());
        let differences = a
            .iter()
            .zip(b.iter())
            .filter(|(x, y)| (**x - **y).abs() > 1e-4)
            .count();
        assert!(
            differences > a.len() / 4,
            "expected substantial divergence, got {differences} / {}",
            a.len(),
        );
    }

    #[test]
    fn distinct_modes_use_distinct_preambles() {
        // Audio for the same payload at different modes must differ
        // in the preamble region (first PREAMBLE_LEN × NSPS samples
        // ≈ 1270 samples).
        let payload = vec![0u8; 12];
        let h_r = header_for(Mode::Robust, 1);
        let h_s = header_for(Mode::Standard, 1);
        let a_r = encode(&h_r, &payload, AUDIO_CENTRE_HZ).unwrap();
        let a_s = encode(&h_s, &payload, AUDIO_CENTRE_HZ).unwrap();
        // Both modes use NSPS=10; UltraRobust would need a different
        // window since its NSPS=20 audio is twice as long. Use the
        // matching mode's NSPS here.
        let nsps = Mode::Robust.nsps();
        let diffs = a_r[..PREAMBLE_LEN * nsps]
            .iter()
            .zip(a_s[..PREAMBLE_LEN * nsps].iter())
            .filter(|(x, y)| (**x - **y).abs() > 1e-3)
            .count();
        assert!(
            diffs > PREAMBLE_LEN * nsps / 4,
            "Robust and Standard preambles should differ substantially in preamble window: {diffs}",
        );
    }

    #[test]
    fn oversize_payload_rejected() {
        let header = header_for(Mode::Robust, 1);
        let too_big = vec![0u8; INFO_BYTES_PER_BLOCK + 1];
        assert!(matches!(
            encode(&header, &too_big, AUDIO_CENTRE_HZ).unwrap_err(),
            PackError::PayloadTooLarge(_),
        ));
    }
}
