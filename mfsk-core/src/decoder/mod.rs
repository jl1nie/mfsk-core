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
//!   `id2`) or as `f32` at any level, for [`Decoder::decode`].
//!   [`Decoder::decode_prefix`] takes the period so far instead, for FT8's
//!   early decode at `nzhsym` 41 (#572); the decoder infers the stage.
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
#[cfg(all(
    feature = "fst4w",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]
mod fst4w;
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

#[cfg(all(
    feature = "fst4w",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]
pub use crate::fst4w::decode::{Fst4wResult, Fst4wState, TooManyCalls};
#[cfg(feature = "fst4")]
pub use crate::msg::decode_request::NoiseBlanker;
#[cfg(any(feature = "ft8", feature = "ft4", feature = "fst4"))]
pub use frame::{
    FrameState, Fst4Extras, Fst4Strategy, Ft4Extras, Ft4Strategy, Ft8Extras, Ft8Strategy,
    MessageFilter, Sniper, Tuning,
};
#[cfg(all(
    feature = "fst4w",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]
pub use fst4w::Fst4wExtras;

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
#[non_exhaustive]
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
    /// candidate already running finishes, and the coarse search is not cut.
    /// Every mode polls it (WSPR, JT9, JT65 and Q65 since #593), except Q65's
    /// averaged decode, whose unit is a whole period.
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
#[non_exhaustive]
pub struct RowDetail {
    /// Sync score of the decode, on the scale of the mode's own search, so not
    /// comparable between modes: FT8's is the coarse candidate score (WSJT-X's
    /// `sync` column, against `sync_min` 1.3 / 2.1); FT4's and FST4's the refined
    /// one, FT4's on its own `sync_min` scale and FST4's on `fst4_sync_search`'s.
    /// `None` where the mode reports none: WSPR, JT9, JT65 and Q65, and FT8's a7
    /// and a8 list decodes, which do not go through a sync search (#594).
    pub sync_score: Option<f32>,
    /// Coefficient of variation of the per-block sync powers (fading). `None`
    /// wherever `sync_score` is.
    pub sync_cv: Option<f32>,
    /// Hard-decision errors the FEC corrected. `None` for WSPR, JT9, JT65 and
    /// Q65, whose decoders report no such count; `Some(0)` is a clean decode.
    pub hard_errors: Option<u32>,
    /// Which decode pass produced the row; private to the mode.
    pub pass: u8,
    /// The message's identity key, one bit per byte: what two rows are
    /// compared by, and what the crate's own de-duplication uses. FT8 and
    /// FT4 give their 91 FEC information bits and FST4 its 101 (the first 77
    /// the message); WSPR its 50; JT9 and JT65 the 72 bits of the
    /// message fields; Q65 its 77 (#592). A row from a mode-generic caller's
    /// side is compared by this, never by `text`, which differs between a
    /// streamed row and the returned one when a `<...>` resolves, and which
    /// two distinct signals can share.
    pub info: Vec<u8>,
    /// The text needed the callsign hash table to resolve a `<...>`.
    pub hash_resolved: bool,
    /// Which delivery of [`Decoder::decode_with`] this row is, or came from
    /// (#592). A row handed to the callback carries its own position in the
    /// period's deliveries (0, 1, 2...); a returned row carries the position of
    /// the delivery it was, the first one with the same bits, frequency and time,
    /// so a caller pairs the two **exactly**, with no rounding and no key to
    /// build. `None` on a returned row the callback never saw, on every row of a
    /// plain `decode`, and on returned rows in a build without `std` (which cannot
    /// keep the list; the callback's own rows still count).
    ///
    /// A parallel strategy (`delivery_is_exact` false) can deliver one row twice:
    /// the second delivery is a position no returned row points at.
    pub delivery: Option<u32>,
    /// Q65 Pileup's "copied last Tx" flag.
    pub copied_last_tx: bool,
    /// When a [`Decoder::decode_prefix`] sequence found the row: early
    /// enough to answer in this period, or at its end (#572). `None` from
    /// `decode` and `decode_with`, and from a prefix call with no
    /// `SlotInput::period`.
    pub stage: Option<Stage>,
}

/// When in a period a [`Decoder::decode_prefix`] sequence found a row
/// (#572). Read on a row, never passed in: the decoder infers the stage
/// from how much of the period it has been given.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stage {
    /// Found by a call made before the period ended: FT8's checkpoint A,
    /// `ft8_decode.f90` at `nzhsym == 41`, about 11.8 s in. Soon enough to
    /// answer the station in the next period.
    Early,
    /// Found by the call whose audio was the whole period.
    Final,
}

/// What a [`Decoder`] keeps for the [`Decoder::decode_prefix`] sequence of
/// the current period, beside the mode's own state: which period it is,
/// its deliveries (so a returned row pairs with one streamed by an earlier
/// call of the period), and the complete result once the final call has
/// made it.
struct PrefixBook<R> {
    period: Option<i64>,
    deliveries: Deliveries,
    done: Option<SlotResult<R>>,
}

impl<R> Default for PrefixBook<R> {
    fn default() -> Self {
        Self {
            period: None,
            deliveries: Deliveries::default(),
            done: None,
        }
    }
}

/// The samples of one period at 12 kHz: `t_slot_s · 12 000` (180 000 for
/// FT8). A prefix call whose audio is this long is the period's last.
fn period_samples(mode: Mode) -> usize {
    #[cfg(not(feature = "std"))]
    #[allow(unused_imports)]
    use num_traits::Float;
    (mode.meta().t_slot_s * 12_000.0).round() as usize
}

/// One decoded message: the common row, with its text resolved against
/// the decoder's hash table, and the mode's native result.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct Row<R> {
    pub decoded: Decoded,
    pub detail: RowDetail,
    pub native: R,
}

/// What one period produced.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct SlotResult<R> {
    /// In the order they were found.
    pub rows: Vec<Row<R>>,
    pub budget: BudgetReport,
}

/// The callback [`Decoder::decode_with`] hands each row to as it is found.
pub type OnRow<'a, R> = &'a (dyn Fn(&Row<R>) + Sync);

/// What `Decoder::decode_with` hands its callback, by position, so a returned
/// row can say which delivery it was (`RowDetail::delivery`, #592).
#[derive(Default)]
struct Deliveries {
    next: core::sync::atomic::AtomicU32,
    /// `(position, bits, freq_hz bits, dt_sec bits)` of each delivery. Exact
    /// float bits: the callback's row and the returned one are the same value.
    #[cfg(feature = "std")]
    seen: std::sync::Mutex<alloc::vec::Vec<Delivery>>,
}

/// One delivery: its position, and the bits, `freq_hz` bits and `dt_sec` bits it had.
#[cfg(feature = "std")]
type Delivery = (u32, alloc::vec::Vec<u8>, u32, u32);

impl Deliveries {
    fn record(&self, d: &Decoded, det: &RowDetail) -> u32 {
        let i = self
            .next
            .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        #[cfg(feature = "std")]
        self.seen.lock().unwrap().push((
            i,
            det.info.clone(),
            d.freq_hz.to_bits(),
            d.dt_sec.to_bits(),
        ));
        #[cfg(not(feature = "std"))]
        let _ = (d, det);
        i
    }

    fn find(&self, d: &Decoded, det: &RowDetail) -> Option<u32> {
        #[cfg(feature = "std")]
        {
            let (f, t) = (d.freq_hz.to_bits(), d.dt_sec.to_bits());
            return self
                .seen
                .lock()
                .unwrap()
                .iter()
                .find(|(_, info, sf, st)| *info == det.info && *sf == f && *st == t)
                .map(|(i, ..)| *i);
        }
        #[cfg(not(feature = "std"))]
        {
            let _ = (d, det);
            None
        }
    }
}

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

    /// Whether `decode_with` under these settings delivers exactly the rows
    /// it returns, once each and in order (`STREAMING.md` §3a). `false` is
    /// the parallel contract (§3b): completion order, and a row may arrive
    /// twice. A mode that is always sequential keeps the default.
    #[doc(hidden)]
    fn __delivery_is_exact(_params: &DecodeParams, _extras: &Self::Extras) -> bool {
        true
    }

    /// The prefix lengths `__decode_prefix` does work at before the whole
    /// period, under these settings. A mode with no early decode keeps the
    /// default: none.
    #[doc(hidden)]
    fn __prefix_points(_params: &DecodeParams, _extras: &Self::Extras) -> &'static [usize] {
        &[]
    }

    #[doc(hidden)]
    fn __decode(
        params: &DecodeParams,
        extras: &Self::Extras,
        state: &mut Self::State,
        slot: &SlotInput<'_>,
        on_row: Option<OnRow<'_, Self::Row>>,
    ) -> SlotResult<Self::Row>;

    /// One call of a prefix sequence for `slot.period` (always `Some`
    /// here). `full` is whether `slot.audio` is the whole period. A mode
    /// with no early decode keeps the default: nothing before the end, and
    /// the end is `__decode`.
    #[doc(hidden)]
    fn __decode_prefix(
        params: &DecodeParams,
        extras: &Self::Extras,
        state: &mut Self::State,
        slot: &SlotInput<'_>,
        on_row: Option<OnRow<'_, Self::Row>>,
        full: bool,
    ) -> SlotResult<Self::Row> {
        if full {
            Self::__decode(params, extras, state, slot, on_row)
        } else {
            SlotResult {
                rows: Vec::new(),
                budget: BudgetReport::default(),
            }
        }
    }
}

/// The decoder of one mode. See the module documentation.
pub struct Decoder<P: Decodable> {
    params: DecodeParams,
    extras: P::Extras,
    state: P::State,
    prefix: PrefixBook<P::Row>,
}

impl<P: Decodable> Decoder<P> {
    /// A decoder with `params`. Allocates nothing until the first decode.
    pub fn new(params: DecodeParams) -> Self {
        Self {
            params,
            extras: P::Extras::default(),
            state: P::State::default(),
            prefix: PrefixBook::default(),
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

    /// Whether [`Decoder::decode_with`] with the current settings delivers
    /// exactly the rows it returns, once each and in the same order
    /// (`STREAMING.md` §3a), so a caller needs no guard against a repeat.
    ///
    /// `false` means only that this is not promised: the parallel contract
    /// (§3b) delivers in completion order and may repeat a row. That is FT8's
    /// sniper and `SinglePass`, FT4's `SinglePass` (its `Fast` depth), every
    /// FST4 mode and WSPR, whether or not the `parallel` feature is on. It
    /// follows the mode, the depth and the [`Extras`](Decodable::Extras), so
    /// ask again after changing them. Either way a row's text can still differ
    /// between the streamed and the returned form (§2); compare
    /// [`RowDetail::info`].
    pub fn delivery_is_exact(&self) -> bool {
        P::__delivery_is_exact(&self.params, &self.extras)
    }

    /// The prefix lengths, in 12 kHz samples, at which
    /// [`Decoder::decode_prefix`] does work before the whole period, in
    /// increasing order: FT8 under `SicEarly` gives checkpoints A and B,
    /// `[141_696, 162_432]`; every other setting and mode gives none, and a
    /// prefix call then returns nothing until the whole period. The whole
    /// period is not listed: it is always the last call.
    ///
    /// For a caller that cuts the prefixes itself and wants to know when to:
    /// [`IqReceiver::set_prefix_points`](crate::iq::IqReceiver::set_prefix_points)
    /// takes this list per channel. It follows the depth and the
    /// [`Extras`](Decodable::Extras), so ask again after changing them
    /// (`docs/notes/IQ_PREFIX_DESIGN.md` §2).
    ///
    /// It describes the **next period to start**, from the settings as they
    /// are now, not a period already under way: that one keeps the settings
    /// its first call pinned ([`Decoder::decode_prefix`]), and on an
    /// `IqReceiver` the points its slot opened with, since `set_prefix_points`
    /// applies from the next slot that opens. So a change midway reaches the
    /// cutter and the decoder at the same period. A caller that cuts the
    /// prefixes itself and asks again midway may then skip a point the
    /// running period would have used, or add one it has no work at; neither
    /// changes the final call's rows (calls may be skipped or repeated), only
    /// when work is done (#631).
    pub fn prefix_points(&self) -> &'static [usize] {
        P::__prefix_points(&self.params, &self.extras)
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
        let deliveries = Deliveries::default();
        let wrapped = |row: &Row<P::Row>| {
            let mut r = row.clone();
            r.detail.delivery = Some(deliveries.record(&r.decoded, &r.detail));
            on_row(&r);
        };
        let mut out = P::__decode(
            &self.params,
            &self.extras,
            &mut self.state,
            slot,
            Some(&wrapped),
        );
        for row in &mut out.rows {
            row.detail.delivery = deliveries.find(&row.decoded, &row.detail);
        }
        out
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
        self.prefix = PrefixBook::default();
    }

    /// Decode the period so far, keeping what this period has already
    /// found (#572, `docs/notes/EARLY_DECODE_DESIGN.md`).
    ///
    /// Call it as audio arrives, with **everything of the period received up
    /// to now** and `slot.period` set. The decoder infers the stage from the
    /// audio length and what it holds for that period, so the caller tracks
    /// nothing. FT8 acts at 141 696 samples (checkpoint A, `nzhsym` 41,
    /// ~11.8 s: its rows are returned and marked [`Stage::Early`]), at
    /// 162 432 (checkpoint B: the A rows that fit are subtracted, nothing is
    /// returned) and at the period's full length, 180 000 samples (the rest
    /// of the search, then a7 / a8). A call between those returns at once
    /// with no rows; so does every call of a mode with no early decode until
    /// the full length. Calls may be skipped or repeated: the final call's
    /// rows are always the period's complete set, the same rows a whole-period
    /// [`Decoder::decode`] returns for the same 16-bit audio, in the same
    /// order. A call for the same period after that returns the complete set
    /// again without decoding.
    ///
    /// A call for another period discards what the last one held. With no
    /// `period` this is a one-shot [`Decoder::decode`] of the audio given,
    /// keeping nothing. Mixing in `decode` midway through a period's sequence
    /// is unspecified (duplicate rows at worst). Settings are pinned at the
    /// period's first call: FT8's strategy and search, whether it runs the
    /// sniper and on which frequency (#628: a sniper turned on midway would
    /// otherwise replace the staged sequence, and drop early rows already
    /// returned), and the gain of `f32` audio, which is measured on that
    /// first prefix. A change to these takes effect at the next period; the
    /// rest (a-priori hint and QSO context, message filter) is read at each
    /// call, as upstream's GUI refills `dec_data.params` from its current
    /// settings for each checkpoint (`widgets/mainwindow.cpp:2751`).
    ///
    /// Synchronous and CPU-bound: it holds the calling thread for the
    /// stage's work. Bound it with `SlotInput::budget`.
    pub fn decode_prefix(&mut self, slot: &SlotInput<'_>) -> SlotResult<P::Row> {
        self.prefix_call(slot, None)
    }

    /// [`Decoder::decode_prefix`], handing each row to `on_row` once, as it
    /// is found. A row's [`RowDetail::delivery`] counts across the whole
    /// period's calls, so the final call's rows pair with rows an earlier call
    /// streamed.
    pub fn decode_prefix_with(
        &mut self,
        slot: &SlotInput<'_>,
        on_row: OnRow<'_, P::Row>,
    ) -> SlotResult<P::Row> {
        self.prefix_call(slot, Some(on_row))
    }

    fn prefix_call(
        &mut self,
        slot: &SlotInput<'_>,
        on_row: Option<OnRow<'_, P::Row>>,
    ) -> SlotResult<P::Row> {
        let Some(period) = slot.period else {
            return match on_row {
                Some(cb) => self.decode_with(slot, cb),
                None => self.decode(slot),
            };
        };
        if self.prefix.period != Some(period) {
            self.prefix = PrefixBook {
                period: Some(period),
                ..PrefixBook::default()
            };
        }
        if let Some(done) = &self.prefix.done {
            return done.clone();
        }
        let len = match slot.audio {
            Audio::I16(a) => a.len(),
            Audio::F32(a) => a.len(),
        };
        let full = len >= period_samples(P::MODE);
        let stage = if full { Stage::Final } else { Stage::Early };
        let deliveries = &self.prefix.deliveries;
        let wrapped = |row: &Row<P::Row>| {
            let mut r = row.clone();
            r.detail.stage.get_or_insert(stage);
            r.detail.delivery = Some(deliveries.record(&r.decoded, &r.detail));
            if let Some(cb) = on_row {
                cb(&r);
            }
        };
        let mut out = P::__decode_prefix(
            &self.params,
            &self.extras,
            &mut self.state,
            slot,
            on_row.map(|_| &wrapped as OnRow<'_, P::Row>),
            full,
        );
        for row in &mut out.rows {
            row.detail.stage.get_or_insert(stage);
            if on_row.is_some() || full {
                row.detail.delivery = self.prefix.deliveries.find(&row.decoded, &row.detail);
            }
        }
        if full {
            self.prefix.done = Some(out.clone());
        }
        out
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
