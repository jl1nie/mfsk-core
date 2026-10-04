// SPDX-License-Identifier: GPL-3.0-only
//! [`AnyDecoder`]: a [`Decoder`] whose mode is chosen at run time.
//!
//! An enum, not a trait object: one variant per [`Mode`] this build has,
//! dispatched by `match`, with no allocation on the decode path. Its rows
//! are the common [`Decoded`] (a caller that wants a mode's native result
//! holds the typed `Decoder<P>`).

use alloc::vec::Vec;

use super::{
    Audio, BudgetReport, DecodeParams, Decoder, OnRow, Row, RowDetail, SlotInput, SlotResult,
    default_params,
};
use crate::msg::{ApHint, Decoded};
use crate::registry::Mode;

/// An option the mode does not have.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Unsupported {
    pub mode: Mode,
    pub option: &'static str,
}

impl core::fmt::Display for Unsupported {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} has no {}", self.mode.name(), self.option)
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Unsupported {}

/// What one period of an [`AnyDecoder`] produced.
#[derive(Clone, Debug)]
pub struct AnySlotResult {
    /// In the order they were found.
    pub rows: Vec<Decoded>,
    /// What each row carries beyond the common one; same order and length
    /// as [`Self::rows`].
    pub details: Vec<RowDetail>,
    pub budget: BudgetReport,
}

fn erase<R>(r: SlotResult<R>) -> AnySlotResult {
    let (rows, details) = r
        .rows
        .into_iter()
        .map(|row| (row.decoded, row.detail))
        .unzip();
    AnySlotResult {
        rows,
        details,
        budget: r.budget,
    }
}

/// A mode's [`Decodable::Extras`](super::Decodable::Extras), borrowed.
/// Match it to set what the mode has.
#[non_exhaustive]
pub enum AnyExtras<'a> {
    #[cfg(feature = "ft8")]
    Ft8(&'a mut super::Ft8Extras),
    #[cfg(feature = "ft4")]
    Ft4(&'a mut super::Ft4Extras),
    #[cfg(feature = "fst4")]
    Fst4(&'a mut super::Fst4Extras),
    #[cfg(feature = "wspr")]
    Wspr(&'a mut super::WsprExtras),
    #[cfg(feature = "jt9")]
    Jt9(&'a mut super::Jt9Extras),
    #[cfg(feature = "jt65")]
    Jt65(&'a mut super::Jt65Extras),
    #[cfg(feature = "q65")]
    Q65(&'a mut super::Q65Extras),
    #[doc(hidden)]
    _Lifetime(core::marker::PhantomData<&'a ()>),
}

macro_rules! any_decoder {
    ($( $feat:literal $var:ident $ty:ty => $extras:ident ),* $(,)?) => {
        /// A decoder of any [`Mode`].
        #[non_exhaustive]
        pub enum AnyDecoder {
            $( #[cfg(feature = $feat)] $var(Decoder<$ty>), )*
        }

        impl AnyDecoder {
            /// A decoder of `mode` with `params`.
            pub fn new(mode: Mode, params: DecodeParams) -> Self {
                match mode {
                    $( #[cfg(feature = $feat)] Mode::$var => AnyDecoder::$var(Decoder::new(params)), )*
                }
            }

            pub fn mode(&self) -> Mode {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(_) => Mode::$var, )*
                }
            }

            pub fn params(&self) -> &DecodeParams {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => d.params(), )*
                }
            }

            /// Change the parameter block between periods; state is kept.
            pub fn params_mut(&mut self) -> &mut DecodeParams {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => d.params_mut(), )*
                }
            }

            /// The mode's library options, to be matched and set.
            pub fn extras_mut(&mut self) -> AnyExtras<'_> {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => AnyExtras::$extras(d.extras_mut()), )*
                }
            }

            /// Decode one period.
            pub fn decode(&mut self, slot: &SlotInput<'_>) -> AnySlotResult {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => erase(d.decode(slot)), )*
                }
            }

            /// Decode one period, handing each row to `on_row` as it is found.
            pub fn decode_with(
                &mut self,
                slot: &SlotInput<'_>,
                on_row: &(dyn Fn(&Decoded, &RowDetail) + Sync),
            ) -> AnySlotResult {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => {
                        let cb = |row: &Row<_>| on_row(&row.decoded, &row.detail);
                        let cb: OnRow<'_, _> = &cb;
                        erase(d.decode_with(slot, cb))
                    } )*
                }
            }

            /// A packed 77-bit message as text, `<...>` resolved against this
            /// decoder's callsign table.
            pub fn unpack77(&self, msg77: &[u8]) -> Option<alloc::string::String> {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => d.unpack77(msg77), )*
                }
            }

            /// Teach the decoder's hash table a callsign; `false` if the mode
            /// has no hashed calls to resolve.
            pub fn learn_callsign(&mut self, call: &str) -> bool {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => d.learn_callsign(call), )*
                }
            }

            /// Forget everything carried across periods.
            pub fn clear(&mut self) {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => d.clear(), )*
                }
            }
        }
    };
}

any_decoder! {
    "ft8" Ft8 crate::Ft8 => Ft8,
    "ft4" Ft4 crate::Ft4 => Ft4,
    "fst4" Fst4S15 crate::fst4::Fst4s15 => Fst4,
    "fst4" Fst4S30 crate::fst4::Fst4s30 => Fst4,
    "fst4" Fst4S60 crate::fst4::Fst4s60 => Fst4,
    "fst4" Fst4S120 crate::fst4::Fst4s120 => Fst4,
    "fst4" Fst4S300 crate::fst4::Fst4s300 => Fst4,
    "wspr" Wspr crate::Wspr => Wspr,
    "jt9" Jt9 crate::Jt9 => Jt9,
    "jt65" Jt65 crate::Jt65 => Jt65,
    "q65" Q65A15 crate::q65::Q65a15 => Q65,
    "q65" Q65A30 crate::q65::Q65a30 => Q65,
    "q65" Q65A60 crate::q65::Q65a60 => Q65,
    "q65" Q65B60 crate::q65::Q65b60 => Q65,
    "q65" Q65C60 crate::q65::Q65c60 => Q65,
    "q65" Q65D60 crate::q65::Q65d60 => Q65,
    "q65" Q65E60 crate::q65::Q65e60 => Q65,
    "q65" Q65D120 crate::q65::Q65d120 => Q65,
    "q65" Q65E120 crate::q65::Q65e120 => Q65,
    "q65" Q65A300 crate::q65::Q65a300 => Q65,
}

impl AnyDecoder {
    /// A decoder of `mode` with the mode's default block ([`default_params`]).
    pub fn with_defaults(mode: Mode) -> Self {
        Self::new(mode, default_params(mode))
    }

    /// Set or clear the free-form AP hint ("hunt this DX"), on the modes
    /// that take one: FT8, FT4, FST4 and Q65.
    pub fn set_ap_hint(&mut self, hint: Option<ApHint>) -> Result<(), Unsupported> {
        let mode = self.mode();
        let slot: Option<&mut Option<ApHint>> = match self.extras_mut() {
            #[cfg(feature = "ft8")]
            AnyExtras::Ft8(e) => Some(&mut e.ap_hint),
            #[cfg(feature = "ft4")]
            AnyExtras::Ft4(e) => Some(&mut e.ap_hint),
            #[cfg(feature = "fst4")]
            AnyExtras::Fst4(e) => Some(&mut e.ap_hint),
            #[cfg(feature = "q65")]
            AnyExtras::Q65(e) => Some(&mut e.ap_hint),
            _ => None,
        };
        match slot {
            Some(s) => {
                *s = hint;
                Ok(())
            }
            None => Err(Unsupported {
                mode,
                option: "AP hint",
            }),
        }
    }

    /// Decode one period of `i16` audio.
    pub fn decode_i16(&mut self, audio: &[i16], period: Option<i64>) -> AnySlotResult {
        let mut slot = SlotInput::new(Audio::I16(audio));
        slot.period = period;
        self.decode(&slot)
    }
}
