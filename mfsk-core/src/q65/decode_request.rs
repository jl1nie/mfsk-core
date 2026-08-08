//! `DecodeRequest`/`SniperRequest`-style builder for Q65 (issue #204).
//!
//! Mirrors the shape and philosophy of [`crate::msg::decode_request`]
//! (issue #191) — a single builder per search shape instead of the
//! pre-#191-style `_with_ap`/`_for`/`_with_ap_list_for` suffix
//! explosion `q65::rx` used to expose as 15 separate public functions.
//!
//! Q65 can't just implement `FrameDecodable` and reuse
//! `msg::decode_request::{DecodeRequest, SniperRequest}` directly: those
//! types hardcode `audio: &'a [i16]`, but every Q65 rx function (and the
//! FFI layer wrapping it) operates on `&[f32]` PCM — WSJT-X's own Q65
//! decoder works in float throughout, unlike the WSJT77-family modes'
//! integer path. Q65 also has a `decode_multi_period_for` shape
//! ([`MultiPeriodRequest`]) that takes *multiple* audio buffers
//! (`&[&[f32]]`, one per T/R slot), not a single one — a fundamentally
//! different input shape than either `DecodeRequest` or `SniperRequest`.
//! Hence dedicated builder types here rather than a generic
//! `DecodeRequest<P>` extension.
//!
//! Three builders, matching the three input shapes `q65::rx` used to
//! expose per-function:
//! - [`DecodeRequest`] — wide-band scan (`decode_scan*` family).
//! - [`SniperRequest`] — single known `(start_sample, base_freq_hz)`
//!   (`decode_at*` family). Built via [`DecodeRequest::sniper`], same
//!   convention as `msg::decode_request::DecodeRequest::sniper`.
//! - [`MultiPeriodRequest`] — averaged multi-slot decode
//!   (`decode_multi_period*` family).
//!
//! Capabilities (`.ap_hint()` / `.ap_list()` / `.fading()`) are plain
//! inherent methods, not capability-gated marker traits like
//! `SupportsWideBandAp`: every Q65 sub-mode supports every capability
//! uniformly (unlike FT8/FT4/FST4, whose capability set genuinely
//! differs per protocol), so there is no invalid combination to guard
//! against at the type level. `.ap_list()` and `.fading()` are mutually
//! exclusive in the underlying engine (no `q65::rx` function ever
//! combined AP-list template matching with the fast-fading metric);
//! `.decode()` resolves precedence as ap_list > fading > ap_hint > plain,
//! documented on each builder's `decode()`.

use alloc::sync::Arc;
use alloc::vec::Vec;
use core::marker::PhantomData;

use crate::engine::Protocol;
use crate::engine::protocol::DecodeContext;
use crate::fec::qra::FadingModel;
use crate::msg::ApHint;
use crate::msg::hash_table::CallsignHashTable;

use super::Q65Result;
use super::search::SearchParams;

/// Build a [`DecodeContext`] from an optional caller-supplied hash
/// table, without cloning the table's contents — `Arc::clone` is a
/// refcount bump regardless of how many entries the table holds, so
/// this is cheap to call once per `decode()` even for a session-long
/// table that's grown to hundreds of entries.
fn ctx_from_hash_table(hash_table: Option<&Arc<CallsignHashTable>>) -> DecodeContext {
    DecodeContext {
        callsign_hash_table: hash_table
            .map(|ht| Arc::clone(ht) as Arc<dyn core::any::Any + Send + Sync>),
    }
}

/// Protocols usable with the Q65 builders — sealed to the Q65 sub-mode
/// ZSTs (`Q65a15`, `Q65a30`, …). Every one of `q65::rx`'s functions
/// this module wraps assumes Q65's specific 65-tone / GF(64) / 22-sync
/// frame shape; a marker (rather than bounding directly on
/// `ModulationParams`) stops a non-Q65 `Protocol` impl from compiling
/// against these builders and silently producing garbage.
pub trait Q65SubMode: Protocol {}

impl Q65SubMode for super::Q65a15 {}
impl Q65SubMode for super::Q65a30 {}
impl Q65SubMode for super::Q65a60 {}
impl Q65SubMode for super::Q65b60 {}
impl Q65SubMode for super::Q65c60 {}
impl Q65SubMode for super::Q65d60 {}
impl Q65SubMode for super::Q65e60 {}
impl Q65SubMode for super::Q65d120 {}
impl Q65SubMode for super::Q65e120 {}
impl Q65SubMode for super::Q65a300 {}

/// Wide-band Q65 decode request: search `nominal_start_sample` ±
/// `params.time_tolerance_symbols` across `params.freq_min_hz
/// ..params.freq_max_hz` for every candidate signal. Construct with
/// [`DecodeRequest::new`], chain builder methods, call
/// [`DecodeRequest::decode`].
///
/// Replaces `q65::rx`'s `decode_scan`/`decode_scan_default`/
/// `decode_scan_for`/`decode_scan_with_ap`/`decode_scan_with_ap_for`/
/// `decode_scan_fading_for`/`decode_scan_with_ap_list_for` (issue #204).
pub struct DecodeRequest<'a, P: Q65SubMode> {
    audio: &'a [f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: SearchParams,
    ap_hint: Option<&'a ApHint>,
    ap_list: Option<&'a [[i32; 63]]>,
    fading: Option<(FadingModel, f32)>,
    /// Set via [`DecodeRequest::on_result`] — see that method's doc
    /// comment for the delivery-order/dedup contract.
    on_result: Option<&'a (dyn Fn(&Q65Result) + Sync)>,
    /// Set via [`DecodeRequest::hash_table`] — resolves `<...>`
    /// hashed-callsign placeholders (WSJT-X Type 4). `None` by
    /// default: `<...>` stays unresolved, matching every `q65::rx`
    /// entry point's pre-existing behavior.
    hash_table: Option<Arc<CallsignHashTable>>,
    _marker: PhantomData<P>,
}

impl<'a, P: Q65SubMode> DecodeRequest<'a, P> {
    pub fn new(
        audio: &'a [f32],
        sample_rate: u32,
        nominal_start_sample: usize,
        params: SearchParams,
    ) -> Self {
        Self {
            audio,
            sample_rate,
            nominal_start_sample,
            params,
            ap_hint: None,
            ap_list: None,
            fading: None,
            on_result: None,
            hash_table: None,
            _marker: PhantomData,
        }
    }

    /// Single-target narrow-band request at a known alignment. Same
    /// convention as [`crate::msg::decode_request::DecodeRequest::sniper`].
    pub fn sniper(
        audio: &'a [f32],
        sample_rate: u32,
        start_sample: usize,
        base_freq_hz: f32,
    ) -> SniperRequest<'a, P> {
        SniperRequest::new(audio, sample_rate, start_sample, base_freq_hz)
    }

    /// A-priori callsign/grid/report hint applied to every candidate.
    /// Empirically gains 2-4 dB at threshold; see `decode_at_with_ap_for`'s
    /// former docs.
    pub fn ap_hint(mut self, hint: &'a ApHint) -> Self {
        self.ap_hint = Some(hint);
        self
    }

    /// BP-free template-matching decode against a pre-encoded
    /// candidate set (e.g. [`super::standard_qso_codewords`]) instead
    /// of belief propagation.
    pub fn ap_list(mut self, candidates: &'a [[i32; 63]]) -> Self {
        self.ap_list = Some(candidates);
        self
    }

    /// Fast-fading metric for Doppler-spread channels (microwave EME,
    /// fast scatter). `b90_ts` is spread-bandwidth × symbol period;
    /// `model` selects Gaussian (libration-limited EME, default in
    /// WSJT-X) or Lorentzian (heavier-tail scattering) calibration.
    pub fn fading(mut self, model: FadingModel, b90_ts: f32) -> Self {
        self.fading = Some((model, b90_ts));
        self
    }

    /// Fire `cb` once per candidate as it's accepted, *in addition to*
    /// (not instead of) `decode()`'s own returned `Vec` — purely
    /// additive streaming delivery alongside the existing batch
    /// result, same shape as
    /// [`crate::msg::decode_request::DecodeRequest::on_result`] (see
    /// that method's doc comment and `docs/reference/LIBRARY.md`'s
    /// "public decode entry point" section for the full portability
    /// rationale — synchronous callback, not async/channel, so
    /// `mfsk-core`'s engine layers stay usable on `no_std` targets).
    ///
    /// **Delivery order/dedup contract**: `q65::rx`'s scan loops
    /// (`decode_scan_for`/`decode_scan_with_ap_for`/
    /// `decode_scan_fading_for`/`decode_scan_with_ap_list_for`) are
    /// sequential and dedup-then-push, with no early exit — `cb`
    /// fires exactly once per result that ends up in the returned
    /// `Vec`, in the same order. No divergence mechanism exists here
    /// (unlike FT8's parallel single-pass strategy).
    pub fn on_result(mut self, cb: &'a (dyn Fn(&Q65Result) + Sync)) -> Self {
        self.on_result = Some(cb);
        self
    }

    /// Session [`CallsignHashTable`] to resolve `<...>` hashed-callsign
    /// (WSJT-X Type 4) placeholders. `None` by default — matches every
    /// `q65::rx` entry point's pre-existing behavior of leaving them
    /// unresolved. `Arc`-shared so passing the same table into repeated
    /// `decode()` calls across a session is a cheap refcount bump, not
    /// a table clone; the caller owns the table's lifecycle and grows
    /// it across the session as standard-format calls are heard.
    pub fn hash_table(mut self, ht: Arc<CallsignHashTable>) -> Self {
        self.hash_table = Some(ht);
        self
    }

    /// Resolves to `decode_scan_with_ap_list_for` if [`Self::ap_list`]
    /// was set, else `decode_scan_fading_for` if [`Self::fading`] was
    /// set (carrying any `.ap_hint()` along), else
    /// `decode_scan_with_ap_for`/`decode_scan_for` depending on
    /// [`Self::ap_hint`].
    pub fn decode(&self) -> Vec<Q65Result> {
        let ctx = ctx_from_hash_table(self.hash_table.as_ref());
        if let Some(candidates) = self.ap_list {
            return super::rx::decode_scan_with_ap_list_for::<P>(
                self.audio,
                self.sample_rate,
                self.nominal_start_sample,
                &self.params,
                candidates,
                self.on_result,
                &ctx,
            );
        }
        if let Some((model, b90_ts)) = self.fading {
            return super::rx::decode_scan_fading_for::<P>(
                self.audio,
                self.sample_rate,
                self.nominal_start_sample,
                &self.params,
                b90_ts,
                model,
                self.ap_hint,
                self.on_result,
                &ctx,
            );
        }
        match self.ap_hint {
            Some(hint) => super::rx::decode_scan_with_ap_for::<P>(
                self.audio,
                self.sample_rate,
                self.nominal_start_sample,
                &self.params,
                hint,
                self.on_result,
                &ctx,
            ),
            None => super::rx::decode_scan_for::<P>(
                self.audio,
                self.sample_rate,
                self.nominal_start_sample,
                &self.params,
                self.on_result,
                &ctx,
            ),
        }
    }
}

/// Single-target Q65 decode request at a known `(start_sample,
/// base_freq_hz)` alignment. Construct with [`DecodeRequest::sniper`]
/// or [`SniperRequest::new`].
///
/// Replaces `q65::rx`'s `decode_at`/`decode_at_for`/
/// `decode_at_with_ap`/`decode_at_with_ap_for`/`decode_at_fading_for`/
/// `decode_at_with_ap_list_for` (issue #204).
pub struct SniperRequest<'a, P: Q65SubMode> {
    audio: &'a [f32],
    sample_rate: u32,
    start_sample: usize,
    base_freq_hz: f32,
    ap_hint: Option<&'a ApHint>,
    ap_list: Option<&'a [[i32; 63]]>,
    fading: Option<(FadingModel, f32)>,
    /// Set via [`SniperRequest::on_result`] — see
    /// [`DecodeRequest::on_result`]'s doc comment for the delivery-
    /// order/dedup contract (this request always yields at most one
    /// result, so `cb` fires 0 or 1 times, simultaneously with
    /// `decode()`'s own return — kept for API consistency with
    /// [`DecodeRequest`]/[`MultiPeriodRequest`], not because it buys
    /// a timing head start here).
    on_result: Option<&'a (dyn Fn(&Q65Result) + Sync)>,
    /// See [`DecodeRequest::hash_table`].
    hash_table: Option<Arc<CallsignHashTable>>,
    _marker: PhantomData<P>,
}

impl<'a, P: Q65SubMode> SniperRequest<'a, P> {
    pub fn new(audio: &'a [f32], sample_rate: u32, start_sample: usize, base_freq_hz: f32) -> Self {
        Self {
            audio,
            sample_rate,
            start_sample,
            base_freq_hz,
            ap_hint: None,
            ap_list: None,
            fading: None,
            on_result: None,
            hash_table: None,
            _marker: PhantomData,
        }
    }

    /// See [`DecodeRequest::ap_hint`].
    pub fn ap_hint(mut self, hint: &'a ApHint) -> Self {
        self.ap_hint = Some(hint);
        self
    }

    /// See [`DecodeRequest::ap_list`].
    pub fn ap_list(mut self, candidates: &'a [[i32; 63]]) -> Self {
        self.ap_list = Some(candidates);
        self
    }

    /// See [`DecodeRequest::fading`].
    pub fn fading(mut self, model: FadingModel, b90_ts: f32) -> Self {
        self.fading = Some((model, b90_ts));
        self
    }

    /// See [`DecodeRequest::on_result`] and this struct's `on_result`
    /// field doc comment for why this fires at most once here.
    pub fn on_result(mut self, cb: &'a (dyn Fn(&Q65Result) + Sync)) -> Self {
        self.on_result = Some(cb);
        self
    }

    /// See [`DecodeRequest::hash_table`].
    pub fn hash_table(mut self, ht: Arc<CallsignHashTable>) -> Self {
        self.hash_table = Some(ht);
        self
    }

    /// Precedence: ap_list > fading (+ ap_hint) > ap_hint > plain — see
    /// [`DecodeRequest::decode`].
    pub fn decode(&self) -> Option<Q65Result> {
        let ctx = ctx_from_hash_table(self.hash_table.as_ref());
        let result = if let Some(candidates) = self.ap_list {
            super::rx::decode_at_with_ap_list_for::<P>(
                self.audio,
                self.sample_rate,
                self.start_sample,
                self.base_freq_hz,
                candidates,
                &ctx,
            )
        } else if let Some((model, b90_ts)) = self.fading {
            super::rx::decode_at_fading_for::<P>(
                self.audio,
                self.sample_rate,
                self.start_sample,
                self.base_freq_hz,
                b90_ts,
                model,
                self.ap_hint,
                &ctx,
            )
        } else {
            match self.ap_hint {
                Some(hint) => super::rx::decode_at_with_ap_for::<P>(
                    self.audio,
                    self.sample_rate,
                    self.start_sample,
                    self.base_freq_hz,
                    hint,
                    &ctx,
                ),
                None => super::rx::decode_at_for::<P>(
                    self.audio,
                    self.sample_rate,
                    self.start_sample,
                    self.base_freq_hz,
                    &ctx,
                ),
            }
        };
        if let (Some(cb), Some(r)) = (self.on_result, &result) {
            cb(r);
        }
        result
    }
}

/// Multi-period averaging Q65 decode request: maintains an EMA
/// spectrogram across `audio_slots` (one buffer per T/R period) and
/// decodes against energies averaged over all slots seen so far at
/// each surviving candidate — the strategy that lets ionoscatter and
/// weak EME signals decode when single-period BP/fading cannot.
///
/// Replaces `q65::rx`'s `decode_multi_period`/`decode_multi_period_for`
/// (issue #204). Unlike [`DecodeRequest`]/[`SniperRequest`], only
/// `.ap_list()` is meaningful here — multi-period decode always tries
/// the fading + plain ladder internally regardless (see
/// `decode_multi_period_for`'s former docs), so there is no
/// `.fading()`/`.ap_hint()` to set.
pub struct MultiPeriodRequest<'a, P: Q65SubMode> {
    audio_slots: &'a [&'a [f32]],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: SearchParams,
    ap_list: Option<&'a [[i32; 63]]>,
    /// Set via [`MultiPeriodRequest::on_result`] — see
    /// [`DecodeRequest::on_result`]'s doc comment for the general
    /// contract. Here, `cb` fires once per *slot* that yields an
    /// accepted (non-duplicate) decode — a natural streaming unit for
    /// multi-period EME/ionoscatter decode, where each T/R period's
    /// own result (if any) is worth surfacing as soon as that period
    /// finishes, rather than waiting for every slot in `audio_slots`.
    on_result: Option<&'a (dyn Fn(&Q65Result) + Sync)>,
    /// See [`DecodeRequest::hash_table`].
    hash_table: Option<Arc<CallsignHashTable>>,
    _marker: PhantomData<P>,
}

impl<'a, P: Q65SubMode> MultiPeriodRequest<'a, P> {
    pub fn new(
        audio_slots: &'a [&'a [f32]],
        sample_rate: u32,
        nominal_start_sample: usize,
        params: SearchParams,
    ) -> Self {
        Self {
            audio_slots,
            sample_rate,
            nominal_start_sample,
            params,
            ap_list: None,
            on_result: None,
            hash_table: None,
            _marker: PhantomData,
        }
    }

    /// See [`DecodeRequest::ap_list`]. Tried first (WSJT-X's `iavg=1`
    /// q3 path) at every slot before falling back to the fading/plain
    /// ladder.
    pub fn ap_list(mut self, candidates: &'a [[i32; 63]]) -> Self {
        self.ap_list = Some(candidates);
        self
    }

    /// See this struct's `on_result` field doc comment for the
    /// per-slot delivery unit this type uses.
    pub fn on_result(mut self, cb: &'a (dyn Fn(&Q65Result) + Sync)) -> Self {
        self.on_result = Some(cb);
        self
    }

    /// See [`DecodeRequest::hash_table`].
    pub fn hash_table(mut self, ht: Arc<CallsignHashTable>) -> Self {
        self.hash_table = Some(ht);
        self
    }

    pub fn decode(&self) -> Vec<Q65Result> {
        let ctx = ctx_from_hash_table(self.hash_table.as_ref());
        super::rx::decode_multi_period_for::<P>(
            self.audio_slots,
            self.sample_rate,
            self.nominal_start_sample,
            &self.params,
            self.ap_list,
            self.on_result,
            &ctx,
        )
    }
}
