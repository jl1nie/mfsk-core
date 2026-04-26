// SPDX-License-Identifier: GPL-3.0-or-later
//! uvpacket transmit path: bytes → 91-bit message → 174-bit LDPC codeword
//! → 91-symbol channel sequence (4 sync + 87 data) → 12 kHz f32 PCM.

use crate::core::dsp::gfsk::{GfskCfg, synth_f32};
use crate::core::{DecodeContext, FecCodec, FrameLayout, MessageCodec, MessageFields, ModulationParams};
use crate::fec::Ldpc174_91;
use crate::msg::PacketBytesMessage;

use super::sync_pattern::UVPACKET_SYNC_PATTERN;

/// Pack the byte payload through the message codec, run LDPC encode,
/// and emit the 91-symbol channel tone sequence (sync + data).
///
/// Each tone value is in `0..NTONES` (`0..4` for uvpacket).
///
/// Returns `None` when the message codec rejects the payload (empty or
/// `> MAX_PAYLOAD_BYTES`).
pub fn encode_to_tones<P: ModulationParams + FrameLayout>(payload: &[u8]) -> Option<Vec<u8>> {
    // Stage 1 — message codec. The codec consumes bytes via the
    // free_text field of MessageFields.
    let fields = MessageFields {
        // SAFETY: PacketBytesMessage treats this as raw bytes — UTF-8
        // validation is irrelevant. Use unchecked construction so binary
        // payloads round-trip cleanly.
        free_text: Some(unsafe {
            std::str::from_utf8_unchecked(payload).to_string()
        }),
        ..Default::default()
    };
    let info_bits = PacketBytesMessage.pack(&fields)?;
    debug_assert_eq!(info_bits.len(), 91);

    // Stage 2 — LDPC encode 91 info bits → 174 codeword bits.
    let mut codeword = [0u8; 174];
    let codec = Ldpc174_91;
    codec.encode(&info_bits, &mut codeword);

    // Stage 3 — codeword bits → 4-FSK symbols (2 bits per symbol).
    // P::N_DATA = 87, so 174 / 2 = 87 data symbols.
    let n_data = P::N_DATA as usize;
    debug_assert_eq!(n_data, 87);
    let mut data_tones = Vec::with_capacity(n_data);
    for k in 0..n_data {
        let b1 = codeword[2 * k] & 1;
        let b0 = codeword[2 * k + 1] & 1;
        // Gray-encode the 2-bit symbol via the protocol's GRAY_MAP.
        let raw = (b1 << 1) | b0;
        let mapped = P::GRAY_MAP[raw as usize];
        data_tones.push(mapped);
    }

    // Stage 4 — splice sync at the front. uvpacket has one Costas-4
    // block at symbol 0; after that, the data symbols.
    let n_sym = P::N_SYMBOLS as usize;
    let mut tones = Vec::with_capacity(n_sym);
    tones.extend_from_slice(&UVPACKET_SYNC_PATTERN);
    tones.extend_from_slice(&data_tones);
    debug_assert_eq!(tones.len(), n_sym);
    Some(tones)
}

/// Synthesise a 12 kHz f32 PCM waveform from a uvpacket tone sequence
/// for sub-mode `P`. Output length is `N_SYMBOLS × NSPS` samples.
pub fn synthesize_audio_for<P: ModulationParams + FrameLayout>(
    tones: &[u8],
    base_freq_hz: f32,
    amplitude: f32,
) -> Vec<f32> {
    debug_assert_eq!(tones.len(), P::N_SYMBOLS as usize);
    let cfg = GfskCfg {
        sample_rate: 12_000.0,
        samples_per_symbol: P::NSPS as usize,
        bt: P::GFSK_BT,
        hmod: P::GFSK_HMOD,
        ramp_samples: (P::NSPS / 8).max(1) as usize,
    };
    synth_f32(tones, base_freq_hz, amplitude, &cfg)
}

/// One-shot helper: encode bytes + synthesise audio in a single call.
pub fn synthesize_packet<P: ModulationParams + FrameLayout>(
    payload: &[u8],
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let tones = encode_to_tones::<P>(payload)?;
    Some(synthesize_audio_for::<P>(&tones, base_freq_hz, amplitude))
}

// Quiet rust about unused import in --release strip-debug-assert paths.
#[allow(dead_code)]
fn _silence() {
    let _ = DecodeContext::default();
}
