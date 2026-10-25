// SPDX-License-Identifier: GPL-3.0-only
//! [`AnyDecoder`]: a [`Decoder`] whose mode is chosen at run time.
//!
//! An enum, not a trait object: one variant per [`Mode`] this build has,
//! dispatched by `match`, with no allocation on the decode path. Its rows
//! are `Row<()>`: the common [`Decoded`](crate::msg::Decoded) and
//! [`RowDetail`](super::RowDetail) (a caller that wants a mode's native result
//! holds the typed `Decoder<P>`).

use super::{Audio, DecodeParams, Decoder, OnRow, Row, SlotInput, SlotResult, default_params};
use crate::msg::ApHint;
use crate::registry::Mode;

/// An option the mode does not have.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Unsupported {
    pub mode: Mode,
    pub option: &'static str,
}

impl core::fmt::Display for Unsupported {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} has no {}", self.mode.name(), self.option)
    }
}

impl core::error::Error for Unsupported {}

/// What one period of an [`AnyDecoder`] produced: the rows of
/// [`Decoder::decode`] with the mode's native result erased (`Row<()>`).
pub type AnySlotResult = SlotResult<()>;

fn erase_row<R>(row: &Row<R>) -> Row<()> {
    Row {
        decoded: row.decoded.clone(),
        detail: row.detail.clone(),
        native: (),
    }
}

fn erase<R>(r: SlotResult<R>) -> AnySlotResult {
    SlotResult {
        rows: r
            .rows
            .into_iter()
            .map(|row| Row {
                decoded: row.decoded,
                detail: row.detail,
                native: (),
            })
            .collect(),
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
    #[cfg(feature = "fst4w")]
    Fst4w(&'a mut super::Fst4wExtras),
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

impl core::fmt::Debug for AnyExtras<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            #[cfg(feature = "ft8")]
            AnyExtras::Ft8(e) => f.debug_tuple("Ft8").field(e).finish(),
            #[cfg(feature = "ft4")]
            AnyExtras::Ft4(e) => f.debug_tuple("Ft4").field(e).finish(),
            #[cfg(feature = "fst4")]
            AnyExtras::Fst4(e) => f.debug_tuple("Fst4").field(e).finish(),
            #[cfg(feature = "fst4w")]
            AnyExtras::Fst4w(e) => f.debug_tuple("Fst4w").field(e).finish(),
            #[cfg(feature = "wspr")]
            AnyExtras::Wspr(e) => f.debug_tuple("Wspr").field(e).finish(),
            #[cfg(feature = "jt9")]
            AnyExtras::Jt9(e) => f.debug_tuple("Jt9").field(e).finish(),
            #[cfg(feature = "jt65")]
            AnyExtras::Jt65(e) => f.debug_tuple("Jt65").field(e).finish(),
            #[cfg(feature = "q65")]
            AnyExtras::Q65(e) => f.debug_tuple("Q65").field(e).finish(),
            AnyExtras::_Lifetime(_) => f.write_str("_Lifetime"),
        }
    }
}

/// [`AnyExtras`], read-only: what [`AnyDecoder::extras`] returns.
#[derive(Debug)]
#[non_exhaustive]
pub enum AnyExtrasRef<'a> {
    #[cfg(feature = "ft8")]
    Ft8(&'a super::Ft8Extras),
    #[cfg(feature = "ft4")]
    Ft4(&'a super::Ft4Extras),
    #[cfg(feature = "fst4")]
    Fst4(&'a super::Fst4Extras),
    #[cfg(feature = "fst4w")]
    Fst4w(&'a super::Fst4wExtras),
    #[cfg(feature = "wspr")]
    Wspr(&'a super::WsprExtras),
    #[cfg(feature = "jt9")]
    Jt9(&'a super::Jt9Extras),
    #[cfg(feature = "jt65")]
    Jt65(&'a super::Jt65Extras),
    #[cfg(feature = "q65")]
    Q65(&'a super::Q65Extras),
    #[doc(hidden)]
    _Lifetime(core::marker::PhantomData<&'a ()>),
}

/// Why [`AnyDecoder::set_wcalls`] did not take the list.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum WcallsError {
    /// The mode has no known-call list (it is FST4W's).
    Unsupported(Unsupported),
    /// The list is longer than the decoder keeps.
    #[cfg(feature = "fst4w")]
    TooManyCalls(crate::fst4w::decode::TooManyCalls),
}

impl core::fmt::Display for WcallsError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WcallsError::Unsupported(e) => core::fmt::Display::fmt(e, f),
            #[cfg(feature = "fst4w")]
            WcallsError::TooManyCalls(e) => core::fmt::Display::fmt(e, f),
        }
    }
}

impl core::error::Error for WcallsError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            WcallsError::Unsupported(e) => Some(e),
            #[cfg(feature = "fst4w")]
            WcallsError::TooManyCalls(e) => Some(e),
        }
    }
}

macro_rules! any_decoder {
    ($( $feat:literal $var:ident $ty:ty => $extras:ident ),* $(,)?) => {
        /// A decoder of any [`Mode`].
        #[non_exhaustive]
        pub enum AnyDecoder {
            $( #[cfg(feature = $feat)] $var(Decoder<$ty>), )*
        }

        impl core::fmt::Debug for AnyDecoder {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => core::fmt::Debug::fmt(d, f), )*
                }
            }
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

            /// The mode's library options, read-only: [`AnyDecoder::extras_mut`]'s twin.
            pub fn extras(&self) -> AnyExtrasRef<'_> {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => AnyExtrasRef::$extras(d.extras()), )*
                }
            }

            /// Decode one period.
            pub fn decode(&mut self, slot: &SlotInput<'_>) -> AnySlotResult {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => erase(d.decode(slot)), )*
                }
            }

            /// Whether `decode_with` with the current mode, depth and extras
            /// delivers exactly the rows it returns, once each and in order
            /// (`STREAMING.md` §3a). `false` is "not promised", the parallel
            /// contract of §3b: ask again after changing the settings. See
            /// [`Decoder::delivery_is_exact`].
            pub fn delivery_is_exact(&self) -> bool {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => d.delivery_is_exact(), )*
                }
            }

            /// The prefix lengths, in 12 kHz samples, at which
            /// `decode_prefix` does work before the whole period under the
            /// current settings; empty when it does none. See
            /// [`Decoder::prefix_points`].
            pub fn prefix_points(&self) -> &'static [usize] {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => d.prefix_points(), )*
                }
            }

            /// Decode one period, handing each row to `on_row` as it is found.
            pub fn decode_with(
                &mut self,
                slot: &SlotInput<'_>,
                on_row: OnRow<'_, ()>,
            ) -> AnySlotResult {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => {
                        let cb = |row: &Row<_>| on_row(&erase_row(row));
                        let cb: OnRow<'_, _> = &cb;
                        erase(d.decode_with(slot, cb))
                    } )*
                }
            }

            /// Decode the period so far, keeping what this period has
            /// already found: [`Decoder::decode_prefix`]. A mode with no
            /// early decode returns nothing until the audio is the whole
            /// period, so a mode-generic caller needs no capability check.
            pub fn decode_prefix(&mut self, slot: &SlotInput<'_>) -> AnySlotResult {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => erase(d.decode_prefix(slot)), )*
                }
            }

            /// [`AnyDecoder::decode_prefix`], handing each row to `on_row`
            /// once, as it is found: [`Decoder::decode_prefix_with`].
            pub fn decode_prefix_with(
                &mut self,
                slot: &SlotInput<'_>,
                on_row: OnRow<'_, ()>,
            ) -> AnySlotResult {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => {
                        let cb = |row: &Row<_>| on_row(&erase_row(row));
                        let cb: OnRow<'_, _> = &cb;
                        erase(d.decode_prefix_with(slot, cb))
                    } )*
                }
            }

            /// Decode a slot an `IqReceiver` delivered, whichever delivery of its
            /// period it is: [`Decoder::decode_slot`].
            #[cfg(any(
                feature = "ft8",
                feature = "ft4",
                feature = "fst4",
                feature = "wspr",
                feature = "jt9",
                feature = "jt65",
                feature = "q65"
            ))]
            pub fn decode_slot(
                &mut self,
                slot: &crate::iq::CompletedSlot,
            ) -> Result<AnySlotResult, super::ModeMismatch> {
                self.decode_slot_with(slot, None, None)
            }

            /// [`AnyDecoder::decode_slot`] with a budget and a streaming callback:
            /// [`Decoder::decode_slot_with`].
            #[cfg(any(
                feature = "ft8",
                feature = "ft4",
                feature = "fst4",
                feature = "wspr",
                feature = "jt9",
                feature = "jt65",
                feature = "q65"
            ))]
            pub fn decode_slot_with(
                &mut self,
                slot: &crate::iq::CompletedSlot,
                budget: Option<super::BudgetCheck<'_>>,
                on_row: Option<OnRow<'_, ()>>,
            ) -> Result<AnySlotResult, super::ModeMismatch> {
                match self {
                    $( #[cfg(feature = $feat)] AnyDecoder::$var(d) => {
                        let cb = on_row.map(|f| move |row: &Row<_>| f(&erase_row(row)));
                        let cb_ref = cb.as_ref().map(|c| c as OnRow<'_, _>);
                        d.decode_slot_with(slot, budget, cb_ref).map(erase)
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
    "fst4" Fst4S900 crate::fst4::Fst4s900 => Fst4,
    "fst4" Fst4S1800 crate::fst4::Fst4s1800 => Fst4,
    "fst4w" Fst4W120 crate::fst4w::Fst4w120 => Fst4w,
    "fst4w" Fst4W300 crate::fst4w::Fst4w300 => Fst4w,
    "fst4w" Fst4W900 crate::fst4w::Fst4w900 => Fst4w,
    "fst4w" Fst4W1800 crate::fst4w::Fst4w1800 => Fst4w,
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

    /// FST4W's Keff-50 known-call list (`get_known_calls`), oldest first.
    /// `Unsupported` for every other mode.
    pub fn wcalls(&self) -> Result<&[alloc::string::String], Unsupported> {
        match self {
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W120(d) => Ok(d.wcalls()),
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W300(d) => Ok(d.wcalls()),
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W900(d) => Ok(d.wcalls()),
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W1800(d) => Ok(d.wcalls()),
            _ => Err(Unsupported {
                mode: self.mode(),
                option: "known-call list",
            }),
        }
    }

    /// Replace FST4W's known-call list (`set_known_calls`): at most 100
    /// entries of 20 characters. [`WcallsError::Unsupported`] for every other
    /// mode, [`WcallsError::TooManyCalls`] for a longer list.
    #[cfg_attr(not(feature = "fst4w"), allow(unused_variables))]
    pub fn set_wcalls(&mut self, calls: &[alloc::string::String]) -> Result<(), WcallsError> {
        match self {
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W120(d) => d.set_wcalls(calls).map_err(WcallsError::TooManyCalls),
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W300(d) => d.set_wcalls(calls).map_err(WcallsError::TooManyCalls),
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W900(d) => d.set_wcalls(calls).map_err(WcallsError::TooManyCalls),
            #[cfg(feature = "fst4w")]
            AnyDecoder::Fst4W1800(d) => d.set_wcalls(calls).map_err(WcallsError::TooManyCalls),
            _ => Err(WcallsError::Unsupported(Unsupported {
                mode: self.mode(),
                option: "known-call list",
            })),
        }
    }

    /// Decode one period of `i16` audio.
    pub fn decode_i16(&mut self, audio: &[i16], period: Option<i64>) -> AnySlotResult {
        let mut slot = SlotInput::new(Audio::I16(audio));
        slot.period = period;
        self.decode(&slot)
    }
}
