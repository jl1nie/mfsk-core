//! FST4W transmit: message → 50 payload bits → CRC-24 over 74 → LDPC(240,74)
//! → 160 tones (`genfst4.f90:66-71,91-119`).
//!
//! The tone mapping is the generic one ([`crate::engine::tx::info_to_tones`]):
//! the same Gray map and sync blocks as FST4. Only the info word differs — no
//! scrambler, 74 bits.

use alloc::vec::Vec;

use crate::engine::Protocol;
use crate::engine::tx::info_to_tones;
use crate::fec::ldpc240_74::{LDPC_K, PAYLOAD_BITS, append_crc24_50};

/// The 74-bit info word for 50 payload bits.
pub fn payload_to_info(payload: &[u8; PAYLOAD_BITS]) -> [u8; LDPC_K] {
    append_crc24_50(payload)
}

/// The 160 tones for 50 payload bits.
pub fn payload_to_tones<P: Protocol>(payload: &[u8; PAYLOAD_BITS]) -> Vec<u8> {
    info_to_tones::<P>(&payload_to_info(payload))
}

/// The 160 tones for a message text, or `None` for what `genfst4` calls a bad
/// message.
pub fn message_to_tones<P: Protocol>(text: &str) -> Option<Vec<u8>> {
    let payload = super::Fst4wMessage::pack_text(text)?;
    Some(payload_to_tones::<P>(&payload))
}
