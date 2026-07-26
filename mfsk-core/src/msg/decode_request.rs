//! Unified `DecodeRequest`/`SniperRequest` builder for the `decode_frame`
//! family (issue #191).
//!
//! Consolidates the FT8/FT4/FST4 `decode_frame*`/`decode_frame_subtract*`/
//! `decode_sniper*` suffix-exploded function families (31 public functions
//! across the three protocols, before this module) into two generic
//! builder types, `P: FrameDecodable` capability-gated so invalid
//! combinations (e.g. staged SIC on FT4, which has no such engine) are
//! compile errors rather than runtime no-ops or silent panics.
//!
//! Lives in `msg` rather than `core` because [`ApHint`] (a `msg::ap` type)
//! is a struct field on both builders, and `core` never depends on `msg`
//! (the reverse dependency direction is established crate-wide).
//!
//! Each concrete protocol (`Ft8`, `Ft4`, each FST4 sub-mode) implements
//! [`FrameDecodable`]'s hidden dispatch methods in its own module, calling
//! into whichever engine that protocol actually uses — FT8's own bespoke
//! one (`ft8::decode_block`), or the shared generic one
//! (`core::pipeline`/`msg::pipeline_ap`) FT4/FST4 share. `DecodeRequest`/
//! `SniperRequest` don't need to know which: `decode()` just calls the
//! dispatch function stashed in `self.strategy` (set by whichever gated
//! builder method — `new`, `.flat()`, `.staged()` — was actually callable
//! for this `P`, so an unsupported combination is a compile error, not a
//! reachable runtime state).

use alloc::vec::Vec;

use crate::core::equalize::EqMode;
use crate::core::pipeline::{DecodeDepth, DecodeStrictness, FftCache};
use crate::core::protocol::Protocol;

use super::ap::{ApHint, WsjtApCompatible};

/// Protocols with a `decode_frame`-family entry point via [`DecodeRequest`]
/// / [`SniperRequest`]. Implemented for `Ft8`, `Ft4`, and each FST4
/// sub-mode; intentionally not implemented for protocols with their own
/// bespoke decode API shape (Q65, WSPR, JT65, JT9, uvpacket) — those keep
/// their existing entry points untouched by this redesign.
pub trait FrameDecodable: Protocol {
    /// Result type this protocol's decode engine produces. FT8's is a
    /// 77-bit post-CRC payload (`ft8::decode::DecodeResult`); FT4/FST4's
    /// carries the full K-bit FEC info with CRC bits retained
    /// (`core::pipeline::DecodeResult`) — genuinely different bit ranges
    /// (issue #194), not force-unified here.
    type DecodeResult;

    #[doc(hidden)]
    fn __single_pass(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self>
    where
        Self: Sized;
    #[doc(hidden)]
    fn __sniper(req: &SniperRequest<'_, Self>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Protocols with a calibrated flat 3-pass SIC (fixed sync_min, sequential
/// subtract). FST4 has no `SubtractCfg` yet — enabling it there is new
/// numerical work requiring WSJT-X-reference calibration, not a refactor,
/// so it's deliberately not implemented here (tracked separately, issue
/// #193).
pub trait SupportsFlatSic: FrameDecodable {
    #[doc(hidden)]
    fn __flat_sic(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Protocols with the jt9.f90 checkpoint-emulation staged SIC (issue
/// #180). FT8 only today. Gated on capability, not identity — a future
/// protocol implementing the same checkpoint architecture just adds an
/// `impl` here, no trait redesign needed (see issue #192).
pub trait SupportsStagedSic: FrameDecodable {
    #[doc(hidden)]
    fn __staged_sic(req: &DecodeRequest<'_, Self>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Protocols whose AP-assisted decode is validated for a full-band
/// (multi-candidate) search, not just a single narrow-band sniper target.
/// FT8 only: FT4/FST4's AP engine (`msg::pipeline_ap::decode_sniper_ap`)
/// has an early-exit-after-first-hit optimization that's only correct
/// when hunting one target — enabling genuine wide-band AP for FT4/FST4
/// would be new, unvalidated capability, kept out of issue #191's scope.
/// [`SniperRequest::ap_hint`] is unaffected by this trait — narrow-band AP
/// is already validated for all three protocols.
pub trait SupportsWideBandAp: FrameDecodable {}

/// Decoded messages plus the FFT cache built along the way, reusable by a
/// follow-up pipelined [`DecodeRequest::fft_cache`] call. The cache is
/// always returned (it's already computed internally regardless of
/// whether the caller wants it back).
pub struct DecodeOutcome<P: FrameDecodable> {
    pub results: Vec<P::DecodeResult>,
    pub fft_cache: FftCache,
}

/// Wide-band decode request: search `freq_min..freq_max` for every
/// candidate signal. Construct with [`DecodeRequest::new`], chain builder
/// methods, call [`DecodeRequest::decode`].
///
/// Replaces the FT8 `decode_frame`/`decode_frame_with_ap`/
/// `decode_frame_with_ap_full`/`decode_frame_with_cache`/
/// `decode_frame_subtract*`/`decode_frame_subtract_with_known*` family and
/// the FT4/FST4 `decode_frame`/`_with_options`/`_with_cache`/
/// `_with_cache_and_options`/`decode_frame_subtract`/`_with_options`
/// family (issue #191).
pub struct DecodeRequest<'a, P: FrameDecodable> {
    pub(crate) audio: &'a [i16],
    pub(crate) freq_min: f32,
    pub(crate) freq_max: f32,
    pub(crate) sync_min: f32,
    pub(crate) freq_hint: Option<f32>,
    pub(crate) depth: DecodeDepth,
    pub(crate) max_cand: usize,
    pub(crate) strictness: DecodeStrictness,
    pub(crate) eq_mode: EqMode,
    pub(crate) ap_hint: Option<&'a ApHint>,
    pub(crate) known: &'a [P::DecodeResult],
    pub(crate) fft_cache: Option<FftCache>,
    strategy: fn(&DecodeRequest<'a, P>) -> DecodeOutcome<P>,
}

impl<'a, P: FrameDecodable> DecodeRequest<'a, P> {
    /// `sync_min` — minimum coarse-sync score (typical: 1.0-2.0).
    /// `max_cand` — maximum number of sync candidates to evaluate.
    pub fn new(
        audio: &'a [i16],
        freq_min: f32,
        freq_max: f32,
        sync_min: f32,
        max_cand: usize,
    ) -> Self {
        Self {
            audio,
            freq_min,
            freq_max,
            sync_min,
            freq_hint: None,
            depth: DecodeDepth::FULL,
            max_cand,
            strictness: DecodeStrictness::Normal,
            eq_mode: EqMode::Off,
            ap_hint: None,
            known: &[],
            fft_cache: None,
            strategy: P::__single_pass,
        }
    }

    /// ±250 Hz narrow-band, single-target preset. See [`SniperRequest`].
    pub fn sniper(audio: &'a [i16], target_freq: f32, max_cand: usize) -> SniperRequest<'a, P> {
        SniperRequest::new(audio, target_freq, max_cand)
    }

    /// Preferred frequency; matching candidates are tried first.
    pub fn freq_hint(mut self, f: f32) -> Self {
        self.freq_hint = Some(f);
        self
    }
    pub fn depth(mut self, d: DecodeDepth) -> Self {
        self.depth = d;
        self
    }
    pub fn strictness(mut self, s: DecodeStrictness) -> Self {
        self.strictness = s;
        self
    }
    pub fn eq_mode(mut self, e: EqMode) -> Self {
        self.eq_mode = e;
        self
    }
    /// Messages already decoded in an earlier pass — skipped (and, for SIC
    /// strategies, subtracted) rather than re-reported.
    pub fn known(mut self, k: &'a [P::DecodeResult]) -> Self {
        self.known = k;
        self
    }
    /// Reuse a previously-built [`FftCache`] (e.g. from an earlier
    /// [`DecodeOutcome::fft_cache`]) instead of rebuilding it from `audio`.
    pub fn fft_cache(mut self, c: FftCache) -> Self {
        self.fft_cache = Some(c);
        self
    }

    pub fn decode(&self) -> DecodeOutcome<P> {
        (self.strategy)(self)
    }
}

impl<'a, P: SupportsWideBandAp> DecodeRequest<'a, P> {
    /// A-priori callsign/grid/report hint applied to every candidate.
    pub fn ap_hint(mut self, ap: &'a ApHint) -> Self {
        self.ap_hint = Some(ap);
        self
    }
}

impl<'a, P: SupportsFlatSic> DecodeRequest<'a, P> {
    /// Flat 3-pass SIC: fixed `sync_min` across all passes, sequential
    /// subtract (each accepted decode is subtracted before the next
    /// candidate in the same pass is tried).
    pub fn flat(mut self) -> Self {
        self.strategy = P::__flat_sic;
        self
    }
}

impl<'a, P: SupportsStagedSic> DecodeRequest<'a, P> {
    /// jt9.f90 checkpoint-emulation staged SIC (issue #180) — decodes
    /// progressively larger audio prefixes, subtracting earlier
    /// checkpoints' signals before the next, faithfully reproducing
    /// WSJT-X's disk-decode architecture. Recall superset of `.flat()`.
    pub fn staged(mut self) -> Self {
        self.strategy = P::__staged_sic;
        self
    }
}

/// Narrow-band (±250 Hz), single-target decode request. Construct with
/// [`DecodeRequest::sniper`] or [`SniperRequest::new`] directly.
///
/// Intended for use after a 500 Hz hardware BPF (or when hunting one known
/// station): `sync_min` defaults to 0.8 (looser than
/// [`DecodeRequest`]'s typical 1.0-2.0) since the narrow band already
/// excludes the strong adjacent signals a low threshold would otherwise
/// admit.
///
/// Replaces FT8's `decode_sniper`/`decode_sniper_eq`/`decode_sniper_ap`
/// and FT4's `decode_sniper_ap`/`_with_options` (issue #191). FT8's
/// `decode_sniper_sic` (in-band interferer subtraction before a second
/// relaxed-threshold pass) is dropped rather than ported — it had zero
/// callers anywhere in the crate.
pub struct SniperRequest<'a, P: FrameDecodable> {
    pub(crate) audio: &'a [i16],
    pub(crate) target_freq: f32,
    pub(crate) sync_min: f32,
    pub(crate) depth: DecodeDepth,
    pub(crate) max_cand: usize,
    pub(crate) strictness: DecodeStrictness,
    pub(crate) eq_mode: EqMode,
    pub(crate) ap_hint: Option<&'a ApHint>,
    _protocol: core::marker::PhantomData<P>,
}

impl<'a, P: FrameDecodable> SniperRequest<'a, P> {
    pub fn new(audio: &'a [i16], target_freq: f32, max_cand: usize) -> Self {
        Self {
            audio,
            target_freq,
            sync_min: 0.8,
            depth: DecodeDepth::FULL,
            max_cand,
            strictness: DecodeStrictness::Normal,
            eq_mode: EqMode::Off,
            ap_hint: None,
            _protocol: core::marker::PhantomData,
        }
    }

    pub fn sync_min(mut self, v: f32) -> Self {
        self.sync_min = v;
        self
    }
    pub fn depth(mut self, d: DecodeDepth) -> Self {
        self.depth = d;
        self
    }
    pub fn strictness(mut self, s: DecodeStrictness) -> Self {
        self.strictness = s;
        self
    }
    pub fn eq_mode(mut self, e: EqMode) -> Self {
        self.eq_mode = e;
        self
    }

    pub fn decode(&self) -> DecodeOutcome<P> {
        P::__sniper(self)
    }
}

impl<'a, P: FrameDecodable> SniperRequest<'a, P>
where
    P::Msg: WsjtApCompatible,
{
    /// A-priori callsign/grid/report hint. The BP decoder locks the known
    /// bits at high confidence, effectively reducing the number of
    /// unknown bits and lowering the decode threshold by 1-3 dB when the
    /// hint matches a station actually on air.
    pub fn ap_hint(mut self, ap: &'a ApHint) -> Self {
        self.ap_hint = Some(ap);
        self
    }
}
