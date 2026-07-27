// SPDX-License-Identifier: GPL-3.0-or-later
//! `UvPacketRawMessage` — trait-required-only message codec.
//!
//! The new uvpacket bypasses [`MessageCodec`] entirely; encoding /
//! decoding happens at the *frame* level in [`crate::uvpacket::tx`]
//! and [`crate::uvpacket::rx`]. The [`Protocol`] trait nevertheless
//! requires a [`MessageCodec`] associated type to keep generic
//! pipeline / registry code honest.
//!
//! `UvPacketRawMessage` satisfies that requirement with a trivial
//! passthrough that:
//! - declares `PAYLOAD_BITS = 101` (= the K of `Ldpc240_101`)
//! - returns `None` from `pack` (uvpacket TX does not go through
//!   this codec)
//! - returns the raw 101 info bits as a 13-byte `Vec<u8>` from
//!   `unpack` (with the trailing 3 bits zero-padded to a byte
//!   boundary)
//! - accepts unconditionally in `verify_info` — frame-level CRC-16
//!   is what catches corruption, not a per-LDPC-block CRC
//!
//! This codec is **not** intended to be invoked directly by user
//! code. The public uvpacket API is byte-pipe — see
//! [`crate::uvpacket::tx::encode`] and
//! [`crate::uvpacket::rx::decode`].
//!
//! [`MessageCodec`]: crate::engine::MessageCodec
//! [`Protocol`]: crate::engine::Protocol

use crate::engine::{DecodeContext, MessageCodec, MessageFields};

/// Trivial passthrough [`MessageCodec`] for the uvpacket family.
/// See module-level docs.
#[derive(Copy, Clone, Debug, Default)]
pub struct UvPacketRawMessage;

impl MessageCodec for UvPacketRawMessage {
    type Unpacked = Vec<u8>;

    /// Equal to `<Ldpc240_101 as FecCodec>::K`. The FEC produces 101
    /// info bits per LDPC block; this codec exposes those bits as
    /// raw bytes with the trailing 3 bits zero-padded to a byte
    /// boundary.
    const PAYLOAD_BITS: u32 = 101;

    /// No per-block CRC — the frame-level CRC-16 (in
    /// [`crate::uvpacket::framing`]) carries the integrity field.
    const CRC_BITS: u32 = 0;

    fn pack(&self, _fields: &MessageFields) -> Option<Vec<u8>> {
        // Bypass: callers should encode at the frame level, not the
        // message level. Returning `None` makes accidental misuse
        // visible at the call site.
        None
    }

    fn unpack(&self, info: &[u8], _ctx: &DecodeContext) -> Option<Self::Unpacked> {
        if info.len() != Self::PAYLOAD_BITS as usize {
            return None;
        }
        let n_bytes = info.len().div_ceil(8);
        let mut out = vec![0u8; n_bytes];
        for (i, &bit) in info.iter().enumerate() {
            if bit != 0 {
                out[i / 8] |= 1 << (7 - (i % 8));
            }
        }
        Some(out)
    }

    fn verify_info(_info: &[u8]) -> bool {
        // Per-block integrity is delegated to frame-level CRC-16.
        true
    }
}
