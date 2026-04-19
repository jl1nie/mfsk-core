//! # mfsk-msg
//!
//! Message-layer codecs for WSJT-family digital modes.
//!
//! | Module       | Payload bits | Used by                   |
//! |--------------|--------------|---------------------------|
//! | [`wsjt77`]   | 77           | FT8, FT4, FT2, FST4       |
//! | [`wspr`]     | 50           | WSPR                      |
//! | [`jt72`]     | 72           | JT65, JT9                 |
//!
//! [`hash_table::CallsignHashTable`] tracks hashed callsigns across decodes;
//! typically a single instance lives in the decoder's side-channel state and
//! is shared by every message unpack invocation.

pub mod ap;
pub mod hash_table;
pub mod jt72;
pub mod pipeline_ap;
pub mod wsjt77;
pub mod wspr;

pub use ap::ApHint;
pub use hash_table::CallsignHashTable;
pub use jt72::{Jt72Codec, Jt72Message};
pub use wspr::{Wspr50Message, WsprMessage};

use crate::core::{DecodeContext, MessageCodec, MessageFields};

/// WSJT 77-bit message codec used by FT8, FT4, FT2 and FST4.
///
/// Pure wrapper around the free functions in [`wsjt77`], implementing the
/// generic [`mfsk_core::core::MessageCodec`] trait so pipeline code can
/// consume messages without knowing which concrete protocol produced them.
#[derive(Copy, Clone, Debug, Default)]
pub struct Wsjt77Message;

impl MessageCodec for Wsjt77Message {
    type Unpacked = String;
    const PAYLOAD_BITS: u32 = 77;
    const CRC_BITS: u32 = 14;

    fn pack(&self, fields: &MessageFields) -> Option<Vec<u8>> {
        // Free text wins if set; otherwise fall back to the standard three-
        // field call/call/report packing used by the overwhelming majority of
        // FT8/FT4 QSOs.
        if let Some(txt) = &fields.free_text {
            return wsjt77::pack77_free_text(txt).map(|a| a.to_vec());
        }
        let call1 = fields.call1.as_deref()?;
        let call2 = fields.call2.as_deref()?;
        // Prefer grid; if the caller supplied a numeric report, format it
        // WSJT-X-style (sign-padded two-digit dB string).
        let report = if let Some(g) = &fields.grid {
            g.clone()
        } else if let Some(r) = fields.report {
            if r >= 0 { format!("+{:02}", r) } else { format!("{:03}", r) }
        } else {
            return None;
        };
        wsjt77::pack77(call1, call2, &report).map(|a| a.to_vec())
    }

    fn unpack(&self, payload: &[u8], ctx: &DecodeContext) -> Option<Self::Unpacked> {
        if payload.len() != 77 {
            return None;
        }
        let mut buf = [0u8; 77];
        buf.copy_from_slice(payload);

        // Prefer the hash-aware path when the caller threaded a table through
        // `DecodeContext`; fall back to the placeholder-emitting variant.
        if let Some(any) = ctx.callsign_hash_table.as_ref() {
            if let Some(ht) = any.downcast_ref::<CallsignHashTable>() {
                return wsjt77::unpack77_with_hash(&buf, ht);
            }
        }
        wsjt77::unpack77(&buf)
    }
}
