//! # `msg` — message-layer codecs and callsign hash table
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
pub mod callsign28;
/// Unified owned decode row for host UIs — see [`decoded::Decoded`].
pub mod decoded;
// `DecodeRequest`/`SniperRequest` builder (issue #191). Needs at least
// one `FrameDecodable` implementor (`ft8`/`ft4`/`fst4`) or its generic
// structs have zero concrete instantiations anywhere in the crate,
// making every field dead code under `-D warnings` (e.g. a `jt65`-only
// build: `fft-rustfft` is on via `jt65`'s own feature dependency, but
// no protocol implements `FrameDecodable`).
// The frame family's engine request. Since 0.13 the public decode API is
// `crate::decoder`; this stays reachable only for `internal-testing`, like
// the raw engine functions beneath it.
#[cfg(all(
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern"),
    any(feature = "ft8", feature = "ft4", feature = "fst4")
))]
pub mod decode_request;
#[cfg(all(
    not(feature = "internal-testing"),
    any(feature = "fft-rustfft", feature = "fft-extern"),
    any(feature = "ft8", feature = "ft4", feature = "fst4")
))]
pub(crate) mod decode_request;
pub mod hash_table;
pub mod jt72;
#[cfg(feature = "packet-bytes")]
pub mod packet_bytes;
// AP hypothesis generation for the protocols whose decoders take an
// `ApHint`: FT4 and FST4 on every FFT backend, and FT8 on the host one
// (`ft8::decode_block::process_candidates` calls `ap_passes` under
// `fft-rustfft` since #423 merged its inline copy; the embedded FT8 ladder
// has no AP rung). (It used to carry a whole parallel AP decode engine;
// that was deleted once AP became a rung on `engine::pipeline`'s own
// ladder.)
#[cfg(any(
    all(
        any(feature = "fft-rustfft", feature = "fft-extern"),
        any(feature = "ft4", feature = "fst4")
    ),
    all(feature = "fft-rustfft", feature = "ft8")
))]
pub mod pipeline_ap;
#[cfg(feature = "q65")]
pub mod q65;
pub mod wsjt77;
pub mod wspr;

pub use ap::{ApHint, ApPassMask};
pub use decoded::Decoded;
pub use hash_table::CallsignHashTable;
pub use jt72::{Jt72Codec, Jt72Message};
#[cfg(feature = "packet-bytes")]
pub use packet_bytes::PacketBytesMessage;
#[cfg(feature = "q65")]
pub use q65::Q65Message;
pub use wspr::{Wspr50Message, WsprMessage};

use alloc::format;
use alloc::vec::Vec;

use crate::engine::{DecodeContext, MessageCodec, MessageFields};

/// WSJT 77-bit message codec used by FT8, FT4, FT2 and FST4.
///
/// Pure wrapper around the free functions in [`wsjt77`], implementing the
/// generic [`crate::MessageCodec`] trait so pipeline code can
/// consume messages without knowing which concrete protocol produced them.
#[derive(Copy, Clone, Debug, Default)]
pub struct Wsjt77Message;

impl MessageCodec for Wsjt77Message {
    /// The decoded *fields*, not the string they render to — see
    /// [`wsjt77::Wsjt77Fields`]. `unpack77`/`unpack77_with_hash` are
    /// still there for callers that only want the rendering.
    type Unpacked = wsjt77::Wsjt77Fields;
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
        } else {
            let r = fields.report?;
            if r >= 0 {
                format!("+{:02}", r)
            } else {
                format!("{:03}", r)
            }
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
        if let Some(any) = ctx.callsign_hash_table.as_ref()
            && let Some(ht) = any.downcast_ref::<CallsignHashTable>()
        {
            return wsjt77::unpack77_fields(&buf, ht);
        }
        wsjt77::unpack77_fields(&buf, &CallsignHashTable::new())
    }

    /// Wsjt77 reserves the trailing K-77 info bits for a CRC. Two
    /// flavours coexist in the WSJT-X family: FT8 / FT4 / FT2 use
    /// LDPC(174, 91) with a 14-bit CRC at bits 77..91, while FST4
    /// uses LDPC(240, 101) with a 24-bit CRC at bits 77..101.
    /// Both share the same Wsjt77 77-bit message field; only the
    /// CRC width differs by FEC pairing. We length-dispatch on the
    /// `info` slice the FEC layer passes through here:
    ///
    /// - 91 → [`crate::fec::ldpc::check_crc14`]
    /// - 101 → [`crate::fec::ldpc240_101::check_crc24`]
    /// - other → reject (no Wsjt77-compatible CRC for that K)
    fn verify_info(info: &[u8]) -> bool {
        match info.len() {
            91 => crate::fec::ldpc::check_crc14(info),
            101 => crate::fec::ldpc240_101::check_crc24(info),
            _ => false,
        }
    }

    /// Same length dispatch as [`Self::verify_info`], appending instead
    /// of checking: CRC-14 for LDPC(174, 91), CRC-24 for LDPC(240, 101).
    fn append_crc(info: &mut [u8]) -> bool {
        let Some(msg77) = info.get(..77).and_then(|m| <&[u8; 77]>::try_from(m).ok()) else {
            return false;
        };
        let msg77 = *msg77;
        match info.len() {
            91 => info.copy_from_slice(&crate::fec::ldpc::append_crc14(&msg77)),
            101 => info.copy_from_slice(&crate::fec::ldpc240_101::append_crc24(&msg77)),
            _ => return false,
        }
        true
    }

    /// [`wsjt77::Wsjt77Fields::is_plausible`] — the callsign grammar
    /// over the callsign *fields*, with free text and
    /// telemetry exempt (nothing in them to check) and the EU VHF
    /// contest requiring a resolved hash (nothing else in it to check).
    fn is_plausible(message: &Self::Unpacked) -> bool {
        message.is_plausible()
    }
}
