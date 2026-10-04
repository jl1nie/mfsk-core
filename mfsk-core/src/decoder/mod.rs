// SPDX-License-Identifier: GPL-3.0-only
//! The decode API: one persistent [`Decoder`] per mode, driven once per
//! period, after WSJT-X's own decoder.
//!
//! WSJT-X runs one decoder process (`jt9 -s`) per mode. Across periods it
//! keeps only what its SAVE and module variables hold: the callsign hash
//! tables (`packjt77`), FT8's a7 list (`ft8_a7.f90`), Q65's and JT65's
//! averaged spectra, wsprd's `hashtable.txt`. Each period the GUI fills one
//! parameter block (`lib/jt9com.f90`) and the decoder reads it.
//!
//! This module is that model:
//!
//! - [`Decoder<P>`] owns `P`'s cross-period state ([`Decodable::State`]),
//!   allocated lazily, never shared between decoders: two channels of the
//!   same mode have two hash tables.
//! - [`DecodeParams`] is the parameter block. [`Depth`] decides every search
//!   setting as `ndepth` does.
//! - [`Decodable::Extras`] holds what the library adds beyond upstream,
//!   typed per mode, so an option a mode lacks does not compile.
//! - [`SlotInput`] is one whole period of 12 kHz audio, as `i16` (`jt9`'s
//!   `id2`) or as `f32` at any level. There is no staged or early-decode
//!   entry point.
//!
//! A one-shot decode of a recording is `Decoder::<P>::new(params)` and one
//! call.

#[cfg(any(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    feature = "wspr",
    feature = "jt9",
    feature = "jt65",
    feature = "q65"
))]
mod any;
#[cfg(any(feature = "ft8", feature = "ft4", feature = "fst4"))]
mod frame;
mod params;
#[cfg(feature = "q65")]
mod q65;
#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65"))]
mod slow;

#[cfg(feature = "q65")]
pub use q65::{MAX_AVERAGED_PERIODS, Q65Extras, Q65State};
#[cfg(feature = "jt9")]
pub use slow::Jt9Extras;
#[cfg(feature = "jt65")]
pub use slow::Jt65Extras;
#[cfg(feature = "wspr")]
pub use slow::{WsprExtras, WsprState};

#[cfg(feature = "fst4")]
pub use crate::msg::decode_request::NoiseBlanker;
#[cfg(any(feature = "ft8", feature = "ft4", feature = "fst4"))]
pub use frame::{
    FrameState, Fst4Extras, Fst4Strategy, Ft4Extras, Ft4Strategy, Ft8Extras, Ft8Strategy,
    MessageFilter, Sniper, Tuning,
};

#[cfg(any(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    feature = "wspr",
    feature = "jt9",
    feature = "jt65",
    feature = "q65"
))]
pub use any::{AnyDecoder, AnyExtras, AnySlotResult, Unsupported};
#[cfg(any(feature = "wspr", feature = "jt9", feature = "jt65", feature = "q65"))]
pub use params::SearchTuning;
pub use params::{ApMode, Contest, DecodeParams, Depth, QsoContext, QsoProgress, Station};

use alloc::vec::Vec;

pub use crate::engine::pipeline::{BudgetCheck, BudgetReport};
use crate::msg::decoded::Decoded;
use crate::registry::Mode;

/// A period's audio at 12 kHz, from its nominal start.
///
/// `I16` is what `jt9` reads (`id2`) and what the embedded fixed-point path
/// carries. `F32` is at any level: modes whose engines work in `f32` (WSPR,
/// JT9, JT65, Q65) take it as is, without the 16-bit round trip, and the
/// FT8 / FT4 / FST4 engines, which take 16-bit audio as WSJT-X does, get it
/// scaled to a fixed RMS ([`F32_TO_I16_RMS`]) first, so a caller never picks
/// a level.
#[derive(Clone, Copy, Debug)]
pub enum Audio<'a> {
    I16(&'a [i16]),
    F32(&'a [f32]),
}

impl Audio<'_> {
    pub fn len(&self) -> usize {
        match self {
            Audio::I16(a) => a.len(),
            Audio::F32(a) => a.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// The RMS `f32` audio is scaled to before the 16-bit engines. 24 dB below
/// full scale leaves headroom for a strong station, and quantisation noise
/// (0.29 LSB RMS) sits 76 dB below the period's RMS, beneath any band noise
/// that carries a decodable signal.
pub const F32_TO_I16_RMS: f32 = 2_000.0;

/// One whole period of audio and what the caller knows about it.
#[non_exhaustive]
#[derive(Clone, Copy)]
pub struct SlotInput<'a> {
    pub audio: Audio<'a>,
    /// The period's index on the UTC grid (`t / period`). Decoder state
    /// that needs consecutive periods (FT8 a7, averaging) is used only when
    /// it is known; `None` (a lone recording) leaves it untouched.
    pub period: Option<i64>,
    /// Checked once per candidate before it is claimed; `false` stops the
    /// search and the rest is reported in [`SlotResult::budget`]. A
    /// candidate already running finishes.
    pub budget: Option<BudgetCheck<'a>>,
}

impl<'a> SlotInput<'a> {
    pub fn i16(audio: &'a [i16]) -> Self {
        Self::new(Audio::I16(audio))
    }

    pub fn f32(audio: &'a [f32]) -> Self {
        Self::new(Audio::F32(audio))
    }

    pub fn new(audio: Audio<'a>) -> Self {
        Self {
            audio,
            period: None,
            budget: None,
        }
    }

    pub fn period(mut self, index: i64) -> Self {
        self.period = Some(index);
        self
    }

    pub fn budget(mut self, check: BudgetCheck<'a>) -> Self {
        self.budget = Some(check);
        self
    }
}

/// What a row carries beyond the common [`Decoded`], in the shape every
/// mode shares (a mode without a field leaves it at its default).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RowDetail {
    /// Sync correlation score of the decode.
    pub sync_score: f32,
    /// Coefficient of variation of the per-block sync powers (fading).
    pub sync_cv: f32,
    /// Hard-decision errors the FEC corrected.
    pub hard_errors: u32,
    /// Which decode pass produced the row; private to the mode.
    pub pass: u8,
    /// The FEC information bits, empty for a mode that has none to give.
    pub info: Vec<u8>,
    /// The text needed the callsign hash table to resolve a `<...>`.
    pub hash_resolved: bool,
    /// Q65 Pileup's "copied last Tx" flag.
    pub copied_last_tx: bool,
}

/// One decoded message: the common row, with its text resolved against
/// the decoder's hash table, and the mode's native result.
#[derive(Clone, Debug)]
pub struct Row<R> {
    pub decoded: Decoded,
    pub detail: RowDetail,
    pub native: R,
}

/// What one period produced.
#[derive(Clone, Debug)]
pub struct SlotResult<R> {
    /// In the order they were found.
    pub rows: Vec<Row<R>>,
    pub budget: BudgetReport,
}

/// The callback [`Decoder::decode_with`] hands each row to as it is found.
pub type OnRow<'a, R> = &'a (dyn Fn(&Row<R>) + Sync);

/// A mode a [`Decoder`] can run. Implemented by every slot-decoded protocol
/// ZST.
pub trait Decodable: Sized {
    const MODE: Mode;
    /// What upstream keeps across periods for this mode, and nothing else.
    type State: Default + Send;
    /// Options the library adds beyond upstream.
    type Extras: Clone + Default + Send;
    /// The mode's native result.
    type Row: Clone + Send;

    /// Teach the decoder's hash table a callsign (WSJT-X's `save_hash_call`).
    /// `false` for a mode whose messages carry no hashed calls.
    #[doc(hidden)]
    fn __learn(_state: &mut Self::State, _call: &str) -> bool {
        false
    }

    /// A packed 77-bit message as text, `<...>` resolved against this
    /// decoder's table where it has one.
    #[doc(hidden)]
    fn __unpack77(_state: &Self::State, msg77: &[u8]) -> Option<alloc::string::String> {
        crate::msg::wsjt77::unpack77(msg77)
    }

    #[doc(hidden)]
    fn __decode(
        params: &DecodeParams,
        extras: &Self::Extras,
        state: &mut Self::State,
        slot: &SlotInput<'_>,
        on_row: Option<OnRow<'_, Self::Row>>,
    ) -> SlotResult<Self::Row>;
}

/// The decoder of one mode. See the module documentation.
pub struct Decoder<P: Decodable> {
    params: DecodeParams,
    extras: P::Extras,
    state: P::State,
}

impl<P: Decodable> Decoder<P> {
    /// A decoder with `params`. Allocates nothing until the first decode.
    pub fn new(params: DecodeParams) -> Self {
        Self {
            params,
            extras: P::Extras::default(),
            state: P::State::default(),
        }
    }

    /// A decoder with the mode's default block (its registry band).
    pub fn with_defaults() -> Self {
        Self::new(default_params(P::MODE))
    }

    pub fn params(&self) -> &DecodeParams {
        &self.params
    }

    /// Change the block between periods, as the GUI does; state is kept.
    pub fn params_mut(&mut self) -> &mut DecodeParams {
        &mut self.params
    }

    pub fn extras(&self) -> &P::Extras {
        &self.extras
    }

    pub fn extras_mut(&mut self) -> &mut P::Extras {
        &mut self.extras
    }

    /// Builder form of [`Decoder::extras_mut`].
    pub fn with_extras(mut self, extras: P::Extras) -> Self {
        self.extras = extras;
        self
    }

    /// Decode one period.
    pub fn decode(&mut self, slot: &SlotInput<'_>) -> SlotResult<P::Row> {
        P::__decode(&self.params, &self.extras, &mut self.state, slot, None)
    }

    /// Decode one period, handing each row to `on_row` as it is found.
    /// Rows found during the search are resolved against the hash table as
    /// it stood when the period began; the returned rows also see calls
    /// learned earlier in the same period.
    pub fn decode_with(
        &mut self,
        slot: &SlotInput<'_>,
        on_row: OnRow<'_, P::Row>,
    ) -> SlotResult<P::Row> {
        P::__decode(
            &self.params,
            &self.extras,
            &mut self.state,
            slot,
            Some(on_row),
        )
    }

    /// A packed 77-bit message as text, resolved against this decoder's
    /// callsign table.
    pub fn unpack77(&self, msg77: &[u8]) -> Option<alloc::string::String> {
        P::__unpack77(&self.state, msg77)
    }

    /// Teach the decoder's hash table a callsign; `false` if the mode has no
    /// hashed calls to resolve.
    pub fn learn_callsign(&mut self, call: &str) -> bool {
        P::__learn(&mut self.state, call)
    }

    /// Forget everything carried across periods (WSJT-X's "Clear Avg" and
    /// `ndepth & 128` auto-clear).
    pub fn clear(&mut self) {
        self.state = P::State::default();
    }
}

/// The parameter block a mode starts with, after the GUI: [`Depth::Deep`],
/// and AP off for FT8 and JT65, whose "Enable AP" boxes start unchecked
/// (`mainwindow_settings.cpp:604-605`).
///
/// The band: the GUI has none of its own (`nfa` is the waterfall's start,
/// 0; `nfb` its right edge), so FT8 and FT4 take `jt9`'s command-line
/// 200-4000 Hz (`jt9.f90:40-41`), and FST4 the GUI's own F Low / F High,
/// 600-1400 Hz (`mainwindow_settings.cpp:481-482`). The other modes keep
/// their registry band until their decoders move here.
pub fn default_params(mode: Mode) -> DecodeParams {
    let d = mode.meta().profile.defaults;
    #[allow(unreachable_patterns)]
    let band = match mode {
        #[cfg(feature = "ft8")]
        Mode::Ft8 => (200.0, 4000.0),
        #[cfg(feature = "ft4")]
        Mode::Ft4 => (200.0, 4000.0),
        #[cfg(feature = "fst4")]
        Mode::Fst4S15 | Mode::Fst4S30 | Mode::Fst4S60 | Mode::Fst4S120 | Mode::Fst4S300 => {
            (600.0, 1400.0)
        }
        _ => (d.freq_min_hz, d.freq_max_hz),
    };
    let p = DecodeParams::for_band(band);
    #[allow(unreachable_patterns)]
    match mode {
        #[cfg(feature = "ft8")]
        Mode::Ft8 => p.ap(ApMode::Off),
        #[cfg(feature = "jt65")]
        Mode::Jt65 => p.ap(ApMode::Off),
        _ => p,
    }
}
