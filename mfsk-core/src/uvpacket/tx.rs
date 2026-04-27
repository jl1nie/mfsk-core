// SPDX-License-Identifier: GPL-3.0-or-later
//! uvpacket transmit path: bytes → 91-bit info → 174-bit LDPC codeword
//! → 95-symbol channel sequence (8 sync + 87 data) → 12 kHz f32 PCM.
//!
//! Delegates the bit-to-symbol mapping to
//! [`crate::core::tx::codeword_to_itone`] so the TX side automatically
//! honours the protocol's
//! [`CODEWORD_INTERLEAVE`](crate::core::FrameLayout::CODEWORD_INTERLEAVE)
//! table and `SYNC_BLOCKS` layout — the receiver sees byte-identical
//! channel-bit ordering and Costas placement to the trait declaration.

use crate::core::dsp::gfsk::{GfskCfg, synth_f32};
use crate::core::tx::codeword_to_itone;
use crate::core::{FecCodec, FrameLayout, MessageCodec, MessageFields, ModulationParams, Protocol};
use crate::fec::Ldpc174_91;
use crate::msg::PacketBytesMessage;

/// Pack the byte payload through the message codec, run LDPC encode,
/// and emit the 95-symbol channel tone sequence (sync + data).
///
/// Each tone value is in `0..NTONES` (`0..4` for uvpacket).
///
/// Returns `None` when the message codec rejects the payload (empty or
/// `> MAX_PAYLOAD_BYTES`).
pub fn encode_to_tones<P: Protocol>(payload: &[u8]) -> Option<Vec<u8>> {
    // Stage 1 — message codec. The codec consumes bytes via the
    // free_text field of MessageFields.
    let fields = MessageFields {
        // SAFETY: PacketBytesMessage treats this as raw bytes — UTF-8
        // validation is irrelevant. Use unchecked construction so binary
        // payloads round-trip cleanly.
        free_text: Some(unsafe { std::str::from_utf8_unchecked(payload).to_string() }),
        ..Default::default()
    };
    let info_bits = PacketBytesMessage.pack(&fields)?;
    debug_assert_eq!(info_bits.len(), 91);

    // Stage 2 — LDPC encode 91 info bits → 174 codeword bits.
    let mut codeword = vec![0u8; 174];
    let codec = Ldpc174_91;
    codec.encode(&info_bits, &mut codeword);

    // Stage 3 — codeword → channel symbols. The generic helper applies
    // the bit interleaver (UVPACKET_INTERLEAVE), splices the two
    // Costas-4 sync blocks at the layout-declared positions, and
    // Gray-encodes each 2-bit symbol via P::GRAY_MAP.
    Some(codeword_to_itone::<P>(&codeword))
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
pub fn synthesize_packet<P: Protocol>(
    payload: &[u8],
    base_freq_hz: f32,
    amplitude: f32,
) -> Option<Vec<f32>> {
    let tones = encode_to_tones::<P>(payload)?;
    Some(synthesize_audio_for::<P>(&tones, base_freq_hz, amplitude))
}
