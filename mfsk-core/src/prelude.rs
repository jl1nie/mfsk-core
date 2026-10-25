// SPDX-License-Identifier: GPL-3.0-only
//! What an application needs, in one glob import: `use mfsk_core::prelude::*;`.
//!
//! The decode API ([`Decoder`], [`AnyDecoder`], [`DecodeParams`], [`SlotInput`],
//! the row and result types), the transmit API ([`encode`], [`Encoded`],
//! [`TxAudio`]), the wideband-IQ front end ([`IqReceiver`] and its stream
//! types), and the two types every call mentions ([`Mode`], [`Decoded`]).
//! Nothing a glob import is likely to clash with is in it: the protocol ZSTs
//! stay at the crate root (`mfsk_core::Ft8`), and the traits and building
//! blocks of a protocol implementer stay in their modules. Each line carries
//! the feature gate of its module, so the prelude is whatever the build has.
//!
//! ```
//! # #[cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))] {
//! use mfsk_core::Ft8;
//! use mfsk_core::prelude::*;
//!
//! // Transmit: text -> tones -> a 12 kHz slot.
//! let tx = encode(Mode::Ft8, "CQ JA1ABC PM95").unwrap();
//! let frame = tx.synthesize(&TxAudio::new(12_000, 1_500.0));
//! let mut slot = vec![0.0f32; 180_000];
//! let start = 6_000; // 0.5 s into the period
//! slot[start..start + frame.len()].copy_from_slice(&frame);
//!
//! // Receive: one decoder, kept between periods.
//! let mut decoder = Decoder::<Ft8>::new(DecodeParams::for_band((100.0, 3_000.0)));
//! let result = decoder.decode(&SlotInput::f32(&slot));
//! assert_eq!(result.rows[0].decoded.text, tx.text);
//! # }
//! ```
//!
//! `WcallsError`, `HasApHint`, `AnyExtras` and `AnyExtrasRef` are not here:
//! the first is only named by code that matches on `set_wcalls`'s error, the
//! others are bounds and return types that are used without being named.

#[cfg(all(
    any(feature = "fft-rustfft", feature = "fft-extern"),
    any(
        feature = "ft8",
        feature = "ft4",
        feature = "fst4",
        feature = "wspr",
        feature = "jt9",
        feature = "jt65",
        feature = "q65"
    )
))]
pub use crate::decoder::{AnyDecoder, AnySlotResult, ModeMismatch, Unsupported};
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
pub use crate::decoder::{
    DecodeParams, Decoder, Depth, QsoContext, QsoProgress, Row, RowDetail, SlotInput, SlotResult,
    Station,
};
pub use crate::encoder::{EncodeError, EncodeOptions, Encoded, TxAudio, encode};
#[cfg(all(
    any(feature = "fft-rustfft", feature = "fft-extern"),
    any(
        feature = "ft8",
        feature = "ft4",
        feature = "fst4",
        feature = "wspr",
        feature = "jt9",
        feature = "jt65",
        feature = "q65"
    )
))]
pub use crate::iq::{ChannelId, ChannelSpec, Channelizer, CompletedSlot, IqReceiver};
pub use crate::iq::{IqError, IqSampleFormat, IqStream};
pub use crate::msg::{ApHint, Decoded};
pub use crate::registry::{Mode, ProtocolMeta};
