//! Unified `DecodeRequest`/`SniperRequest` builder for the `decode_frame`
//! family (issue #191).
//!
//! Consolidates the FT8/FT4/FST4 `decode_frame*`/`decode_frame_subtract*`/
//! `decode_sniper*` suffix-exploded function families (31 public functions
//! across the three protocols, before this module) into two generic
//! builder types, `P: FrameDecodable` capability-gated so invalid
//! combinations (e.g. `.sic_early()` on FT4, which has no such engine) are
//! compile errors rather than runtime no-ops or silent panics.
//!
//! Lives in `msg` rather than `engine` because [`ApHint`] (a `msg::ap` type)
//! is a struct field on both builders, and `engine` never depends on `msg`
//! (the reverse dependency direction is established crate-wide).
//!
//! Each concrete protocol (`Ft8`, `Ft4`, each FST4 sub-mode) implements
//! [`FrameDecodable`]'s hidden dispatch methods in its own module, calling
//! into whichever engine that protocol actually uses — FT8's own bespoke
//! one (`ft8::decode_block`), or the shared generic one
//! (`engine::pipeline`/`msg::pipeline_ap`) FT4/FST4 share. `DecodeRequest`/
//! `SniperRequest` don't need to know which: `decode()` just calls the
//! dispatch function stashed in `self.strategy` (set by whichever gated
//! builder method — `new`, `.sic_rounds()`, `.sic_early()` — was actually
//! callable for this `P`, so an unsupported combination is a compile
//! error, not a reachable runtime state).

use alloc::vec::Vec;

use crate::engine::equalize::EqMode;
pub use crate::engine::pipeline::{BudgetCheck, BudgetReport};
use crate::engine::pipeline::{DecodeDepth, DecodeStrictness, FftCache, LlrEffort};
use crate::engine::protocol::Protocol;

use super::ap::ApHint;
use super::wsjt77::Wsjt77Fields;
// Only `SniperRequest::ap_hint`'s bound names this, and that item is
// `ft8`-gated below.
#[cfg(feature = "ft8")]
use super::ap::WsjtApCompatible;

// ──────────────────────────────────────────────────────────────────────────
// Message-acceptance policy
// ──────────────────────────────────────────────────────────────────────────

/// How a request decides whether a decoded message's *text* is
/// acceptable, on top of the FEC and CRC layers that got it that far.
///
/// A CRC-14 false positive is a codeword the decoder converged on that
/// is not the transmitted one, so its information bits are effectively
/// uniform. With `max_cand = 200` × 4 LLR variants × OSD, 1/16384
/// produces one or two such strings per FT8 slot, and every one of them
/// unpacks to *something*. [`MessageCodec::is_plausible`] is what
/// refuses them; this trait is what lets a caller adjust that verdict
/// without the crate guessing at their band.
///
/// **Three implementors, and the default is zero-sized.**
/// [`DefaultPolicy`] carries nothing, so a request that never calls
/// [`DecodeRequest::also_accept`] or [`DecodeRequest::message_filter`]
/// monomorphises to exactly the code that existed before this trait —
/// `base(text)` inlined, no indirect call, no field. That is the reason
/// this is a type parameter rather than the `&'a dyn Fn` shape
/// [`DecodeRequest::on_result`] and [`DecodeRequest::budget`] use: those
/// fire once per *decode*, this fires once per candidate that reaches
/// the text stage, and the default has to cost nothing at all.
///
/// [`MessageCodec::is_plausible`]: crate::engine::protocol::MessageCodec::is_plausible
pub trait MessagePolicy: Sync {
    /// Whether this policy can ever return `false`.
    ///
    /// [`DefaultPolicy`] cannot, and the constant lets the generic
    /// pipeline's bridge
    /// skip both the codec verdict and the unpack altogether — so a
    /// request that names no policy pays nothing for the seam
    /// existing, not even the `String` the unpack would allocate.
    const CAN_REJECT: bool = true;

    /// `base` is the protocol's codec verdict
    /// ([`MessageCodec::is_plausible`]), **already computed**. It judges
    /// the payload *bits* — the message type lives there, and two of
    /// `Wsjt77Message`'s types carry no redundancy a text-shaped check
    /// could see — so it cannot be handed over as a `fn(&str) -> bool`.
    ///
    /// `text` is the rendered message, or `""` when it would not render
    /// — see `PolicyAccept`.
    ///
    /// [`MessageCodec::is_plausible`]: crate::engine::protocol::MessageCodec::is_plausible
    fn accepts(&self, base: bool, base_applies: bool, message: &Wsjt77Fields) -> bool;
}

/// The codec's own verdict and nothing else — what every request starts
/// with, and a zero-sized type so that costs nothing.
#[derive(Clone, Copy, Debug, Default)]
pub struct DefaultPolicy;

impl MessagePolicy for DefaultPolicy {
    /// Only the protocol's own `MESSAGE_FILTER_DEFAULT` decides here,
    /// and that is a constant too.
    const CAN_REJECT: bool = false;

    #[inline]
    fn accepts(&self, base: bool, base_applies: bool, _message: &Wsjt77Fields) -> bool {
        !base_applies || base
    }
}

/// The claim [`MessagePolicy`]'s doc comment makes about the default
/// costing nothing, held down at compile time rather than asserted in
/// prose: a [`DecodeRequest`] that never names a policy stores a field
/// of size zero.
const _: () = assert!(core::mem::size_of::<DefaultPolicy>() == 0);

/// The message codec's own verdict and nothing else — what this crate
/// applied by default until issue #383 measured what that was worth.
///
/// Zero-sized, like [`DefaultPolicy`], and the one-line way back to the
/// old behaviour:
///
/// ```ignore
/// DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, 1.5, 200)
///     .codec_filter()
///     .decode()
/// ```
///
/// It exists as its own policy because the verdict reads the payload
/// *bits* — [`MessageCodec::is_plausible`] has to, since the message
/// type lives there — so it cannot be handed to
/// [`DecodeRequest::message_filter`], which takes a predicate over
/// the decoded message.
///
/// [`MessageCodec::is_plausible`]: crate::engine::protocol::MessageCodec::is_plausible
#[derive(Clone, Copy, Debug, Default)]
pub struct CodecVerdict;

impl MessagePolicy for CodecVerdict {
    #[inline]
    fn accepts(&self, base: bool, _base_applies: bool, _message: &Wsjt77Fields) -> bool {
        base
    }
}

/// Widen the codec's verdict: accept what it accepts, **plus** whatever
/// the caller's predicate accepts.
///
/// For messages the shipped filter is too strict about — a callsign
/// whose prefix the ITU allowlist does not carry, a local contest
/// exchange — without giving up the filter for everything else. Set by
/// [`DecodeRequest::also_accept`].
///
/// Note what this implies for a protocol whose call site does *not*
/// apply the base by default: the widening still runs against the
/// codec's verdict, so opting in turns the codec's filter on. That is
/// deliberate — a caller asking for "the usual filter, plus this" gets
/// the usual filter.
#[derive(Clone, Copy, Debug)]
pub struct AlsoAccept<F>(pub F);

impl<F: Fn(&Wsjt77Fields) -> bool + Sync> MessagePolicy for AlsoAccept<F> {
    #[inline]
    fn accepts(&self, base: bool, _base_applies: bool, message: &Wsjt77Fields) -> bool {
        base || (self.0)(message)
    }
}

/// Replace the codec's verdict entirely with the caller's predicate.
///
/// The escape hatch for a caller who knows their band better than the
/// crate does — a closed network, a test harness that wants every
/// CRC-passing string, a mode whose traffic the ITU allowlist has no
/// opinion about. Set by [`DecodeRequest::message_filter`].
///
/// This can *lose* nothing and *gain* phantoms: the filter it replaces
/// removes roughly two thirds of the CRC survivors that reach it
/// (`msg::wsjt77`'s `phantom_survival_rates`). Prefer [`AlsoAccept`]
/// unless the whole verdict is wrong for the deployment.
#[derive(Clone, Copy, Debug)]
pub struct Only<F>(pub F);

impl<F: Fn(&Wsjt77Fields) -> bool + Sync> MessagePolicy for Only<F> {
    #[inline]
    fn accepts(&self, _base: bool, _base_applies: bool, message: &Wsjt77Fields) -> bool {
        (self.0)(message)
    }
}

/// Bridges a request's [`MessagePolicy`] to the generic pipeline's
/// [`InfoAccept`] seam: unpack the information bits to text, then ask
/// the policy.
///
/// **An un-unpackable codeword is presented as the empty message**,
/// rather than short-circuited. That is not a shortcut — it makes the
/// three policies fall out correctly on their own. The verdict on `""`
/// is `false`, so a protocol that filters by default refuses it (which
/// is what FT8's own path already does, via `unpack77(..)?`), a
/// protocol that does not accepts it exactly as this pipeline always
/// has, and a caller who supplied a predicate gets to decide by being
/// handed `""`.
///
/// Zero cost when nothing is in force: both
/// [`MessagePolicy::CAN_REJECT`] and
/// [`FrameDecodable::MESSAGE_FILTER_DEFAULT`] are compile-time
/// constants, so for a protocol with neither the whole body folds to
/// `true` and the `String` the unpack would allocate is never built.
// Constructed only by the protocols that decode through
// `engine::pipeline` — FT4 and the FST4 sub-modes. FT8 has its own
// engine and applies the policy inside it, so in a build with neither
// of those features this type really has no caller. Scoped to exactly
// that configuration rather than blanket, and caught by the feature
// matrix rather than by `full`.
#[cfg_attr(not(any(feature = "ft4", feature = "fst4")), allow(dead_code))]
pub(crate) struct PolicyAccept<'p, P: FrameDecodable, Pol: MessagePolicy> {
    policy: &'p Pol,
    // `fn() -> P` rather than `P`: the marker must not make this type
    // `!Sync` for a protocol that happens not to be, and `InfoAccept`
    // requires `Sync` because the pipeline shares it across rayon
    // workers. Protocols here are ZSTs, so this is documentation of
    // intent as much as anything.
    _protocol: core::marker::PhantomData<fn() -> P>,
}

#[cfg_attr(not(any(feature = "ft4", feature = "fst4")), allow(dead_code))]
impl<'p, P: FrameDecodable, Pol: MessagePolicy> PolicyAccept<'p, P, Pol> {
    pub(crate) fn new(policy: &'p Pol) -> Self {
        Self {
            policy,
            _protocol: core::marker::PhantomData,
        }
    }
}

impl<P, Pol> crate::engine::pipeline::InfoAccept for PolicyAccept<'_, P, Pol>
where
    P: FrameDecodable,
    P::Msg: crate::engine::protocol::MessageCodec<Unpacked = Wsjt77Fields>,
    Pol: MessagePolicy,
{
    #[inline]
    fn accept(&self, info: &[u8]) -> bool {
        use crate::engine::protocol::MessageCodec;
        // `DefaultPolicy` has no opinion, and both of these are
        // compile-time constants, so for a protocol that does not
        // filter and a request that named no policy the whole body
        // folds to `true` — the message is never even decoded.
        if !Pol::CAN_REJECT && !P::MESSAGE_FILTER_DEFAULT {
            return true;
        }
        // One decode, shared by the codec verdict and the caller's own
        // rule. Nothing below this renders a message and splits it
        // back into tokens.
        let Some(message) = <P::Msg>::default().unpack(
            &info[..77],
            &crate::engine::protocol::DecodeContext::default(),
        ) else {
            // A codeword whose message will not unpack names nothing.
            // WSJT-X refuses it too (`ft4_decode.f90:437`).
            return false;
        };
        let base = <P::Msg as MessageCodec>::is_plausible(&message);
        self.policy
            .accepts(base, P::MESSAGE_FILTER_DEFAULT, &message)
    }
}

/// Protocols whose decode path has a message-*text* stage a
/// [`MessagePolicy`] can be applied at, so
/// [`DecodeRequest::also_accept`] / [`DecodeRequest::message_filter`] /
/// [`DecodeRequest::codec_filter`] are callable. **`Ft8`, `Ft4` and
/// every FST4 sub-mode.**
///
/// Not a statement about the message codec: all three share
/// `Wsjt77Message`, so the verdict means the same thing for each. It is
/// a statement about the *pipeline*, and the two pipelines reach the
/// stage from opposite sides of the `engine` / `msg` boundary. FT8 has
/// its own bespoke engine (`ft8::decode_block`), which unpacks to text
/// inside the per-candidate ladder and applies the policy there. FT4
/// and the FST4 sub-modes share `engine::pipeline`, which returns
/// information bits and never forms a string — `engine` does not depend
/// on `msg`, so it cannot call `unpack77` at all — and reach the policy
/// through the [`InfoAccept`] seam instead, with `PolicyAccept`
/// unpacking on the `msg` side of it.
///
/// **This doc used to say "FT8 only, for now", and that giving FT4 and
/// FST4 a text stage was "its own change with its own measurement".**
/// That change is the `InfoAccept` seam, which shipped in the same
/// release the sentence did; the sentence outlived it by a few commits.
/// What is still FT8-only is applying the codec's verdict with no
/// caller involved — see [`FrameDecodable::MESSAGE_FILTER_DEFAULT`].
///
/// [`InfoAccept`]: crate::engine::pipeline::InfoAccept
///
/// Gating it means a caller who tries gets a compile error naming the
/// missing capability, rather than a builder method that silently does
/// nothing — the same reason [`SupportsSicEarly`] and [`SupportsSniper`]
/// are traits rather than runtime checks.
pub trait SupportsMessageFilter: FrameDecodable {
    /// Re-select this protocol's strategy dispatch for a new policy
    /// type.
    ///
    /// [`DecodeRequest`] stashes its strategy as a function pointer so
    /// that `decode()` can call a capability-gated entry point
    /// (`__flat_sic`, `__staged_sic`) without naming the capability —
    /// see the module comment. That pointer's type mentions the policy,
    /// so changing the policy invalidates it, and the tag is how it is
    /// rebuilt: a protocol that implements this trait implements all of
    /// its own strategies, so it can hand back the right pointer for
    /// any of them.
    ///
    /// Without this the builder would be order-dependent —
    /// `.also_accept()` would have to precede `.sic_early()` or
    /// silently reset it — which is exactly the kind of reachable
    /// runtime state this module exists to make unrepresentable.
    #[doc(hidden)]
    fn __strategy_for<Pol: MessagePolicy>(
        tag: StrategyTag,
    ) -> fn(&DecodeRequest<'_, Self, Pol>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Which strategy a [`DecodeRequest`] is carrying, kept beside the
/// function pointer so [`SupportsMessageFilter::__strategy_for`] can
/// rebuild it for a different policy type.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StrategyTag {
    SinglePass,
    FlatSic,
    StagedSic,
}

/// Protocols with a `decode_frame`-family entry point via [`DecodeRequest`]
/// / [`SniperRequest`]. Implemented for `Ft8`, `Ft4`, and each FST4
/// sub-mode; intentionally not implemented for protocols with their own
/// bespoke decode API shape (Q65, WSPR, JT65, JT9, uvpacket) — those keep
/// their existing entry points untouched by this redesign.
pub trait FrameDecodable: Protocol {
    /// Result type this protocol's decode engine produces.
    ///
    /// **The same concrete type for all three**, since 0.8.0:
    /// `engine::pipeline::DecodeResult`, which `ft8::decode` re-exports.
    /// It carries the full `Fec::K` info bits with the CRC retained —
    /// 91 for FT8 and FT4 (CRC-14), 101 for FST4 (CRC-24) — and
    /// `message77()` slices the leading 77 identically for each. That is
    /// what lets a caller generic over `DecodeRequest<P>` read every
    /// protocol's results the same way.
    ///
    /// This doc used to say the types were "genuinely different bit
    /// ranges (issue #194), not force-unified here". They were unified
    /// by that issue; the sentence outlived the divergence it described.
    type DecodeResult;

    /// Whether this protocol applies its message codec's
    /// [`MessageCodec::is_plausible`] verdict with no caller involved.
    ///
    /// **`true` for FT8 and FT4, `false` for FST4.** The reason is
    /// subtraction, not display. This crate's high-recall strategies
    /// (`.sic_rounds()`, `.sic_early()`) *subtract* each decode from
    /// the audio before looking again, so accepting a CRC survivor is
    /// not a cosmetic error — its waveform is removed from the
    /// residual, and whatever real signal was underneath goes with it.
    /// Measured on `qso3_busy.wav` (issue #383): with the verdict off,
    /// `.sic_early()` accepts the phantom `CQ G47OXF RD84`, subtracts
    /// it, and loses the real `CQ EA2BFM IN83` — 18/18 becomes 17/18.
    ///
    /// On the single-pass path the same verdict removes two garbage
    /// rows from a crowded slot at `max_cand = 200` and **nothing at
    /// all** at the depth that ships on embedded hardware, which is why
    /// the display argument alone would not justify the divergence:
    /// WSJT-X has no such filter, gating on `nbadcrc` and
    /// `nharderrors <= 36` only.
    ///
    /// FT4 turned it on once the same measurement existed for it
    /// (2026-09-21, `ft4sim` corpus, 720 slots across the threshold
    /// window): phantom rows 7 → 2, golden rows 353 → **354**, with 35
    /// of 36 recall cells identical. `Ft4::MESSAGE_FILTER_DEFAULT`
    /// carries the table and what the corpus cannot say.
    ///
    /// `false` for FST4, and not for want of measuring: CRC-24 puts its
    /// false-positive rate 512x below FT8's and FT4's, so there is
    /// almost nothing for the verdict to remove and the recall it could
    /// cost is the same. A caller opts in per request with
    /// [`DecodeRequest::codec_filter`].
    ///
    /// [`MessageCodec::is_plausible`]: crate::engine::protocol::MessageCodec::is_plausible
    const MESSAGE_FILTER_DEFAULT: bool = false;

    #[doc(hidden)]
    fn __single_pass<Pol: MessagePolicy>(req: &DecodeRequest<'_, Self, Pol>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Protocols with a narrow-band single-target search
/// ([`SniperRequest`]). **FT8 only.**
///
/// Not a capability the others lack for want of porting — it is a
/// capability they should not have. `SniperRequest` is the receive-side
/// half of narrowing a transceiver's *analogue* roofing filter, a
/// thing few radios can do and one operators do when chasing a DX
/// station whose carrier they already know. That is an FT8 activity.
///
/// - **FT4 is a contest mode.** Working many stations quickly across
///   the band is the opposite of pointing a 500 Hz analogue filter at
///   one of them. The two are fundamentally incompatible.
/// - **FST4 has its own narrow-band path**, and a better one: the DDC
///   channelizer (`fst4::ddc`), which narrows in the digital domain
///   without the sniper's halved sync gate or its assumption that the
///   audio arrived pre-filtered.
///
/// And the thing the sniper looked like it was *for* — reaching AP —
/// never belonged to it. A-priori decoding is a general option on the
/// wide-band path now, for every protocol, as it is upstream.
///
/// The baseline this leaves behind is the right one: **if the wide-band
/// decode is not WSJT-X-faithful without a sniper, that is a bug in the
/// wide-band decode.** WSJT-X has no sniper and reaches its published
/// sensitivity regardless. Chasing a shortfall with a special mode
/// hides the defect instead of fixing it — which is exactly what had
/// happened: FT4 sat 0.6 dB behind the published figure until the
/// always-on CQ AP pass upstream has always run was added to the
/// wide-band ladder, at which point it went 0.5 dB ahead.
// Gated on `ft8` for the same reason the module itself is gated on
// having any `FrameDecodable` implementor: `SupportsSniper` is
// implemented for `Ft8` alone, so in a build without it `SniperRequest`
// has zero concrete instantiations anywhere in the crate and every
// field is dead code under `-D warnings`.
#[cfg(feature = "ft8")]
pub trait SupportsSniper: FrameDecodable {
    #[doc(hidden)]
    fn __sniper<Pol: MessagePolicy>(req: &SniperRequest<'_, Self, Pol>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Protocols with a calibrated flat SIC (fixed sync_min, sequential
/// subtract, up to 3 rounds — see [`DecodeRequest::sic_rounds`]). FT8/FT4
/// only — not any FST4 sub-mode.
///
/// This isn't an unfinished mfsk-core port: WSJT-X's own `fst4_decode.f90`
/// has no subtract/SIC path at all (confirmed directly against the WSJT-X
/// source), because FST4 targets point-to-point links (EME/troposcatter/
/// LF-MF) rather than WSPR/FT8-style crowded shared bands where
/// simultaneous-signal collisions are the normal case — upstream never
/// needed it. If FST4 SIC is ever added (tracked separately, issue #193),
/// the short sub-modes (FST4-15/30, whose SNR regime sits close to FT8's
/// — matching issue #143's own scoping to just those two) are the more
/// plausible candidates than the long ones (FST4-120/300): the existing
/// LPF-based gain-tracking subtract approach has a real tension there
/// between the long averaging window deep SNR needs and the short window
/// real fading (EME libration, ionospheric variation) needs.
pub trait SupportsSicRounds: FrameDecodable {
    #[doc(hidden)]
    fn __flat_sic<Pol: MessagePolicy>(req: &DecodeRequest<'_, Self, Pol>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Protocols with the jt9.f90 checkpoint-emulation early decode (issue
/// #180; WSJT-X's own name for this is `ndec_early`/`MAX_EARLY` in
/// `ft8_decode.f90` — checkpointed at `nzhsym` = 41/47/50 out of 79
/// symbols). FT8 only today. Gated on capability, not identity — a future
/// protocol implementing the same checkpoint architecture just adds an
/// `impl` here, no trait redesign needed (see issue #192).
pub trait SupportsSicEarly: FrameDecodable {
    #[doc(hidden)]
    fn __staged_sic<Pol: MessagePolicy>(req: &DecodeRequest<'_, Self, Pol>) -> DecodeOutcome<Self>
    where
        Self: Sized;
}

/// Protocols whose AP-assisted decode is validated for a full-band
/// (multi-candidate) search, not just a single narrow-band sniper target.
///
/// Implemented for **`Ft8`, `Ft4` and every FST4 sub-mode** since
/// 2026-09-13. Until then it was FT8 only, because AP ran in a parallel
/// engine (`msg::pipeline_ap::decode_sniper_ap`) whose candidate loop
/// broke out on `if has_ap` — so *holding a hint*, not the width of the
/// search, was what made a decode single-target, and FT8 escaped only by
/// having its own AP path that never entered that engine. AP is now a
/// rung on `process_candidate_basic`'s own ladder and that engine is
/// gone; see `docs/notes/DESIGN_RATIONALE.md` §3 for the measurements.
///
/// [`SniperRequest::ap_hint`] does not need this trait: it is gated on
/// [`SupportsSniper`] plus `P::Msg: WsjtApCompatible`, and since the
/// sniper is FT8-only that reduces to FT8 either way.
pub trait SupportsWideBandAp: FrameDecodable {}

/// Callback type for [`DecodeRequest::on_result`]/[`SniperRequest::on_result`]
/// — factored into a named alias purely to keep `clippy::type_complexity`
/// quiet at the two struct-field sites; see `on_result`'s own doc comment
/// for the actual delivery contract.
type OnResultCallback<'a, P> = &'a (dyn Fn(&<P as FrameDecodable>::DecodeResult) + Sync);

/// Decoded messages plus the FFT cache built along the way, reusable by a
/// follow-up pipelined [`DecodeRequest::fft_cache`] call. The cache is
/// always returned (it's already computed internally regardless of
/// whether the caller wants it back).
pub struct DecodeOutcome<P: FrameDecodable> {
    pub results: Vec<P::DecodeResult>,
    pub fft_cache: FftCache,
    /// What a [`DecodeRequest::budget`] cut short, if one was set.
    /// [`BudgetReport::default()`] when it wasn't.
    pub budget: BudgetReport,
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
pub struct DecodeRequest<'a, P: FrameDecodable, Pol: MessagePolicy = DefaultPolicy> {
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
    /// Only consulted by [`SupportsSicRounds::__flat_sic`] (set via
    /// [`DecodeRequest::sic_rounds`]); ignored by every other strategy,
    /// including [`SupportsSicEarly::__staged_sic`], whose checkpoint
    /// structure is fixed. Not independently settable — see
    /// `sic_rounds`'s doc comment for why.
    pub(crate) sic_rounds: usize,
    /// Set via [`DecodeRequest::on_result`] — see that method's doc
    /// comment for the delivery-order/dedup contract.
    pub(crate) on_result: Option<OnResultCallback<'a, P>>,
    /// Set via [`DecodeRequest::budget`].
    pub(crate) budget: Option<BudgetCheck<'a>>,
    /// Set via [`DecodeRequest::also_accept`] /
    /// [`DecodeRequest::message_filter`]; [`DefaultPolicy`] otherwise,
    /// which is zero-sized and inlines to the codec's own verdict.
    ///
    /// FT8 is its only reader, because [`SupportsMessageFilter`] is
    /// implemented for FT8 alone — so in a build without that feature
    /// the field really is dead, and the allow is scoped to exactly
    /// that configuration rather than blanket. Step 5 (issue #383)
    /// gives FT4 a text stage, at which point this comes off.
    #[cfg_attr(not(feature = "ft8"), allow(dead_code))]
    pub(crate) policy: Pol,
    /// Which of the three `strategy` below is, so a policy change can
    /// rebuild the pointer — see [`SupportsMessageFilter::__strategy_for`].
    tag: StrategyTag,
    strategy: fn(&DecodeRequest<'a, P, Pol>) -> DecodeOutcome<P>,
}

impl<'a, P: FrameDecodable> DecodeRequest<'a, P, DefaultPolicy> {
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
            sic_rounds: 3,
            on_result: None,
            budget: None,
            policy: DefaultPolicy,
            tag: StrategyTag::SinglePass,
            strategy: P::__single_pass,
        }
    }
}

impl<'a, P: FrameDecodable, Pol: MessagePolicy> DecodeRequest<'a, P, Pol> {
    /// Preferred frequency; matching candidates are tried first.
    pub fn freq_hint(mut self, f: f32) -> Self {
        self.freq_hint = Some(f);
        self
    }
    /// Toggle OSD fallback when the BP staircase fails. `LlrEffort` is
    /// always `Full` for host decodes (the cheaper LLR variants exist
    /// solely for `decode_block_into`'s ESP32 power budget — see
    /// `DecodeDepth::EMBEDDED`'s doc comment; no host caller has ever
    /// needed `Minimal`). Default: `true` (matches the old
    /// `DecodeDepth::FULL` default).
    pub fn osd(mut self, on: bool) -> Self {
        self.depth = DecodeDepth {
            llr_effort: LlrEffort::Full,
            osd: on,
        };
        self
    }
    pub fn strictness(mut self, s: DecodeStrictness) -> Self {
        self.strictness = s;
        self
    }
    /// Local per-symbol equalisation of the symbol spectra.
    ///
    /// **A property of the input audio, not of the search.** It exists
    /// to flatten a passband that an *analogue* filter has tilted — a
    /// transceiver's roofing filter, or any band-pass ahead of the
    /// decoder. That is why it appears on both request types: the
    /// narrow-band one because a roofing filter is the reason that path
    /// exists at all, and this one because filtered audio can equally
    /// be handed to a wide-band decode.
    ///
    /// Measured both ways. `Local` recovers an FT8 signal sitting at a
    /// band-pass edge that `Off` misses entirely, through the wide-band
    /// SIC engine (`ft8::decode`'s `eq_mode_recovers_bpf_edge_signal`).
    /// On flat input it can only cost: the `ft4sim`-generated FT4
    /// golden has no receiver filter at all, and `Local` loses two
    /// decodes of fourteen there.
    ///
    /// Default `Off`, which is right for recorded or simulated audio
    /// and wrong for a narrowed receiver.
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
    /// Fire `cb` once per candidate as it's accepted, *in addition to*
    /// (not instead of) `decode()`'s own returned `DecodeOutcome` —
    /// this is purely additive, streaming delivery alongside the
    /// existing batch result, not a replacement for it.
    ///
    /// A plain synchronous callback, not an async/channel primitive —
    /// see `docs/reference/LIBRARY.md`'s "public decode entry point"
    /// section for why (portability: `mfsk-core`'s `engine`/protocol
    /// layers stay `std`-and-executor-free so embedded targets keep
    /// working; a host caller wanting cross-thread delivery to e.g. a
    /// GUI wraps `cb` itself, such as a `Sender::send` inside the
    /// closure — `mfsk-core` doesn't need to know about that).
    ///
    /// **Delivery order and dedup contract differs by strategy:**
    /// - `.sic_rounds(_)`/`.sic_early()` (sequential SIC): `cb` fires
    ///   exactly once per result that ends up in the returned `Vec`,
    ///   in the same order — zero divergence from the batch result.
    /// - the default single-pass strategy and [`SniperRequest`]
    ///   (parallelized via `rayon` under `feature = "parallel"`): `cb`
    ///   fires from whichever thread decoded that candidate, in
    ///   completion order (not candidate-exploration order), and
    ///   *before* the final cross-candidate dedup pass — on the rare
    ///   occasion two different sync candidates converge on the same
    ///   message, `cb` may fire for both even though only one survives
    ///   into the returned `Vec`. Callers wanting exact parity should
    ///   dedup by `.message77()` on their side, the same key the
    ///   crate's own dedup already uses.
    ///
    /// **Does "delivery order" mean strong signals report first?**
    /// Tends to, on both strategies, but it's a correlation, not a
    /// guarantee, for two independent reasons:
    /// - `coarse_sync` (`ft8::decode_block::coarse_sync`) returns
    ///   candidates sorted by Costas sync score *descending* — both
    ///   the sequential SIC loop and the parallel `par_iter()` sweep
    ///   process/dispatch that list in-order, and sync score does
    ///   correlate with SNR, so higher-scoring (typically stronger)
    ///   candidates tend to appear earlier either way.
    /// - Sync score is a pre-demod correlation-power measurement, not
    ///   a direct predictor of post-demod BP/OSD cost — fading,
    ///   interference, and frequency drift can decouple the two for
    ///   any individual signal, so a highly-scored candidate can still
    ///   need the full OSD escalation while a slightly-lower-scored
    ///   one converges in one BP pass.
    /// - On the **sequential** strategies specifically, this residual
    ///   mismatch has a real consequence the parallel strategies don't
    ///   share: a candidate ahead in the (mostly-but-not-perfectly)
    ///   strength-ordered list that needs deep OSD blocks every
    ///   candidate behind it — including ones that would individually
    ///   decode in microseconds — since there's only one thread. The
    ///   parallel strategies don't have this blocking problem; each
    ///   candidate's processing is independent of every other
    ///   candidate's cost.
    ///
    /// `cb` must be `Sync` for this reason — it may be called
    /// concurrently from multiple `rayon` worker threads.
    ///
    /// **Under [`DecodeRequest::budget`]**, the default strategy's
    /// deliveries change from "completion order, from a worker thread"
    /// to "sync-quality order, from the calling thread", because that
    /// is the order the scheduler runs candidates in. The dedup caveat
    /// above is unchanged — `cb` still fires before it. A budget can
    /// only make `cb` fire *fewer* times; it never fires for a result
    /// the budget then discards, because the SNR gate and the callback
    /// both sit inside the candidate that produced them.
    pub fn on_result(mut self, cb: OnResultCallback<'a, P>) -> Self {
        self.on_result = Some(cb);
        self
    }

    /// Bound this decode by a caller-supplied wall-clock budget: once
    /// `check()` returns `false`, no further work is *started*.
    ///
    /// Opt-in and purely subtractive. Without it — the default — the
    /// decode runs exactly as it always has, and
    /// [`DecodeOutcome::budget`] comes back all-zero. See
    /// [`BudgetCheck`] for why it is a closure and why `mfsk-core`
    /// holds no clock of its own.
    ///
    /// **What "no further work is started" means differs by strategy,
    /// and the difference is deliberate:**
    ///
    /// - The default single-pass strategy spends the budget
    ///   **cheapest-first**: every candidate gets the cheap sync triage
    ///   (which is never budget-gated — that is the latency invariant
    ///   this exists to keep), the survivors are then ordered by their
    ///   triage score, and the expensive ladder runs down that order
    ///   until the budget says stop. A short budget therefore drops the
    ///   *weakest* candidates rather than the tail of the frequency
    ///   sweep.
    /// - The SIC strategies ([`DecodeRequest::sic_rounds`],
    ///   [`DecodeRequest::sic_early`]) only poll at candidate and round
    ///   boundaries, in their existing order. They cannot be reordered:
    ///   each accepted decode is subtracted from the residual before
    ///   the next candidate is looked at, so the order *is* the
    ///   algorithm — and deferring the expensive rung out of an early
    ///   pass was measured to cost more wall-clock, not less, since an
    ///   early decode shrinks the work every later pass does.
    ///
    /// An in-flight candidate is never interrupted, so an overrun is
    /// bounded by one candidate's decode — the same semantics the
    /// embedded receivers already ship at their own app layer.
    ///
    /// **The triage sweep is a floor, and it is not small.** Because it
    /// is never gated, a budget shorter than it buys nothing: the
    /// sweep runs anyway and no ladder starts, so the call returns
    /// empty having spent that time. Measured on `qso3_busy.wav`
    /// through `bench/wasm` under Node — the environment this is for —
    /// the floor is ~13 ms against ~28 ms for the whole decode, so
    /// budgets of 5 and 10 ms return nothing in ~13 ms while 20 ms
    /// returns every station. `max_cand` is the knob that moves the
    /// floor; this one only spends what is above it.
    ///
    /// Implemented for FT8, FT4 and FST4, on every strategy. What the
    /// ordering key is differs by protocol, because each already
    /// computes a different one for free: FT8 ranks by the Costas sync
    /// quality its triage produces, FT4 needs no reordering at all
    /// (`ft4_coarse_sync` hands back candidates already ranked by
    /// score), and FST4 ranks by the refined `fst4_sync_search` score
    /// that `dedup_refined_candidates` has already computed for every
    /// candidate. FT8's own `decode_block` API — what the embedded
    /// boards call — is not affected and keeps its app-level deadline.
    pub fn budget(mut self, check: BudgetCheck<'a>) -> Self {
        self.budget = Some(check);
        self
    }

    pub fn decode(&self) -> DecodeOutcome<P> {
        (self.strategy)(self)
    }
}

impl<'a, P: SupportsMessageFilter, Pol: MessagePolicy> DecodeRequest<'a, P, Pol> {
    /// Accept what the protocol's message codec accepts, **plus**
    /// whatever `f` accepts. **FT8 only** — see [`SupportsMessageFilter`]
    /// for why the others cannot take one yet.
    ///
    /// For traffic the shipped filter is too strict about. It refuses a
    /// callsign whose prefix the ITU allowlist does not carry, which is
    /// the right default against CRC survivors and the wrong one on a
    /// band carrying special-event or experimental calls the list has
    /// no entry for:
    ///
    /// ```ignore
    /// DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, 1.5, 200)
    ///     .also_accept(|text| text.split_whitespace().all(is_cb_callsign))
    ///     .decode()
    /// ```
    ///
    /// Widening only — it cannot lose a decode the default would have
    /// made. Use [`Self::message_filter`] to replace the verdict
    /// instead of adding to it.
    ///
    /// The closure is stored by value in a [`AlsoAccept`], not behind a
    /// `&dyn`, so the whole predicate monomorphises into the decode.
    /// A request that never calls this carries [`DefaultPolicy`], which
    /// is zero-sized.
    pub fn also_accept<F: Fn(&Wsjt77Fields) -> bool + Sync>(
        self,
        f: F,
    ) -> DecodeRequest<'a, P, AlsoAccept<F>> {
        self.with_policy(AlsoAccept(f))
    }

    /// Replace the protocol's message-codec verdict with `f` entirely.
    /// **FT8 only** — see [`SupportsMessageFilter`].
    ///
    /// The escape hatch, and a sharp one: the filter it replaces removes
    /// about two thirds of the CRC survivors that reach it, so a
    /// permissive `f` will surface phantom decodes the default hides.
    /// Prefer [`Self::also_accept`] unless the whole verdict is wrong
    /// for the deployment.
    pub fn message_filter<F: Fn(&Wsjt77Fields) -> bool + Sync>(
        self,
        f: F,
    ) -> DecodeRequest<'a, P, Only<F>> {
        self.with_policy(Only(f))
    }

    /// Apply the message codec's own plausibility verdict
    /// ([`MessageCodec::is_plausible`]) and nothing else.
    ///
    /// The default accepts every codeword the FEC and CRC verified,
    /// which is what WSJT-X does; this is the stricter behaviour this
    /// crate used to impose on everyone. For `Wsjt77Message` it is an
    /// ITU-prefix allowlist over the callsign tokens plus structural
    /// checks for the types whose exchange fields are not callsigns.
    ///
    /// On a crowded FT8 slot at `max_cand = 200` it removes about two
    /// rows; at the depth that ships on embedded hardware it removes
    /// none. See [`FrameDecodable`]'s module for the measurements.
    ///
    /// [`MessageCodec::is_plausible`]: crate::engine::protocol::MessageCodec::is_plausible
    pub fn codec_filter(self) -> DecodeRequest<'a, P, CodecVerdict> {
        self.with_policy(CodecVerdict)
    }

    /// Rebuild with a different policy type, re-selecting the strategy
    /// pointer for it — see [`SupportsMessageFilter::__strategy_for`].
    fn with_policy<Q: MessagePolicy>(self, policy: Q) -> DecodeRequest<'a, P, Q> {
        DecodeRequest {
            audio: self.audio,
            freq_min: self.freq_min,
            freq_max: self.freq_max,
            sync_min: self.sync_min,
            freq_hint: self.freq_hint,
            depth: self.depth,
            max_cand: self.max_cand,
            strictness: self.strictness,
            eq_mode: self.eq_mode,
            ap_hint: self.ap_hint,
            known: self.known,
            fft_cache: self.fft_cache,
            sic_rounds: self.sic_rounds,
            on_result: self.on_result,
            budget: self.budget,
            policy,
            tag: self.tag,
            strategy: P::__strategy_for::<Q>(self.tag),
        }
    }
}

impl<'a, P: SupportsWideBandAp, Pol: MessagePolicy> DecodeRequest<'a, P, Pol> {
    /// A-priori callsign/grid/report hint applied to every candidate.
    ///
    /// Available for **`Ft8`, `Ft4` and every FST4 sub-mode** — every
    /// implementor of [`SupportsWideBandAp`]. A priori decoding is a
    /// general technique: it locks high-confidence bits and lowers the
    /// threshold by 1-3 dB, and nothing about it is narrow-band.
    ///
    /// It was FT8-only until 2026-09-13, and the two reasons are worth
    /// keeping because both were re-derived more than once:
    ///
    /// 1. The shared AP engine (`msg::pipeline_ap::decode_sniper_ap`)
    ///    broke out of its candidate loop on `if has_ap` — **the
    ///    presence of a hint was what made the search single-target**,
    ///    not the width of the search. FT8 escaped only by having its
    ///    own AP path that never entered that engine.
    /// 2. Removing that early exit was *not* enough, which is the part
    ///    that got asserted as "one line" twice and was wrong twice.
    ///    Routing FT4's wide-band decode through that engine returned
    ///    **4 decodes where the plain path returns 11** on the WSJT-X
    ///    golden, lost the hinted station itself, and returned the
    ///    identical set whether the hint named a present or an absent
    ///    station — so AP was changing nothing and the loss was entirely
    ///    the ladder: `process_candidate_ap` offered OSD at depth 2 with
    ///    no depth-3/4 escalation and no Top-K rescue, and most of that
    ///    recording's decodes come from exactly those.
    ///
    /// Both are resolved the same way: AP is a rung at the end of
    /// `process_candidate_basic`'s own ladder — everything above it has
    /// already run and failed, so it can only add decodes — and
    /// `msg::pipeline_ap` is hypothesis generation with no engine of its
    /// own. `docs/notes/DESIGN_RATIONALE.md` §3 has the full account,
    /// including the two bugs found while wiring it.
    pub fn ap_hint(mut self, ap: &'a ApHint) -> Self {
        self.ap_hint = Some(ap);
        self
    }
}

impl<'a, P: SupportsSicRounds, Pol: MessagePolicy> DecodeRequest<'a, P, Pol> {
    /// One round = coarse-sync + per-candidate decode + subtract, over the
    /// (shrinking) residual buffer. `n` is clamped to 1..=3 — WSJT-X's own
    /// `npass`/`nsp` never exceeds 3.
    ///
    /// **The two implementors differ in two ways this doc used to paper
    /// over**, and a caller comparing them will see both:
    ///
    /// - *Threshold schedule.* FT8 holds `sync_min` fixed across rounds.
    ///   FT4 **relaxes** it — the generic engine multiplies by
    ///   `[1.0, 0.75, 0.5]`, one factor per round, so a later round looks
    ///   deeper into the noise on a residual that has had the strong
    ///   signals removed.
    /// - *Termination.* FT8 stops early once a round adds nothing. FT4
    ///   runs every round it was given.
    ///
    /// Both subtract sequentially on FT8 (each accepted decode is removed
    /// before the next candidate in the same round is tried); FT4's
    /// generic engine subtracts a round's accepted decodes as one batch
    /// at the end of the round, which is also why a budget can decline a
    /// whole FT4 round but never cut inside one.
    ///
    /// Corresponds to WSJT-X FT8 `ft8_decode.f90:176` `ipass`/`npass` and
    /// FT4 `ft4_decode.f90` `isp`/`nsp` — **not** FT4's separate
    /// `ipass`/`npasses` AP-hint-variant loop inside a single candidate's
    /// decode attempt, which is an unrelated concept that happens to reuse
    /// the same word in WSJT-X's own source.
    ///
    /// `n` is folded into strategy selection (rather than an independently
    /// settable field) so `.sic_rounds(_).sic_early()` can't compile a
    /// combination where the round count is silently ignored — see issue
    /// #218 for the design discussion.
    ///
    /// Implemented for `Ft8`/`Ft4` only — not any FST4 sub-mode; see
    /// [`SupportsSicRounds`]'s doc comment for why (an upstream WSJT-X
    /// absence, not an mfsk-core gap).
    ///
    /// # You probably want this if you are comparing against WSJT-X
    ///
    /// The default strategy is single-pass, so a plain
    /// `DecodeRequest::new(…).decode()` does **no** subtraction —
    /// while real `jt9`/`wsjtx` run their multi-pass subtraction by
    /// default. Comparing the two without calling this is not
    /// like-for-like, and the difference is not small: on
    /// `WSJT-X/samples/FT4/000000_000002.wav` the default reaches
    /// 11 of the 14 decodes `jt9` reports, and `.sic_rounds(2)`
    /// reaches all 14 with no false decodes.
    ///
    /// The three it recovers are the ones subtraction exists for —
    /// weak signals inside a stronger neighbour's 83 Hz occupied
    /// bandwidth (`-15 dB` at 2300 Hz masked by `-1 dB` at 2310 Hz,
    /// and so on). Cost on that file: 5.0 ms → 71.3 ms against a
    /// 7.5 s slot. Two rounds suffice; three find nothing more.
    /// Pinned by `tests/ft4_wsjtx_samples.rs::
    /// ft4_wsjtx_sample_reaches_jt9_parity_with_sic`.
    pub fn sic_rounds(mut self, n: usize) -> Self {
        self.tag = StrategyTag::FlatSic;
        self.strategy = P::__flat_sic;
        self.sic_rounds = n.clamp(1, 3);
        self
    }
}

impl<'a, P: SupportsSicEarly, Pol: MessagePolicy> DecodeRequest<'a, P, Pol> {
    /// WSJT-X's early decode (`ft8_decode.f90`'s `ndec_early`/`MAX_EARLY`,
    /// checkpointed at `nzhsym` = 41/47/50 out of 79 symbols): decodes
    /// progressively larger audio prefixes, subtracting earlier
    /// checkpoints' signals before the next, faithfully reproducing
    /// WSJT-X's disk-decode architecture. Recall superset of
    /// `.sic_rounds()`. Checkpoint structure (A/B/C) is fixed — no
    /// tunable count, matching jt9 `-d2`/`-d3`'s `npass=3` either way.
    ///
    /// `Ft8` only — FT4/FST4 have no equivalent checkpoint architecture
    /// to port (see [`SupportsSicEarly`]'s doc comment). For FT4, use
    /// [`DecodeRequest::sic_rounds`] instead — a recall subset of the
    /// same underlying idea (flat multi-pass SIC) without the checkpoint
    /// structure.
    pub fn sic_early(mut self) -> Self {
        self.tag = StrategyTag::StagedSic;
        self.strategy = P::__staged_sic;
        self
    }
}

/// Narrow-band (±250 Hz), single-target decode request. Construct with
/// [`DecodeRequest::sniper`] or [`SniperRequest::new`] directly.
///
/// # What this is for, because it is repeatedly misread
///
/// **This is the software half of narrowing the radio's analogue
/// roofing filter.** The operator selects a ~500 Hz analogue roofing
/// filter — available only on the few transceivers that offer one at
/// that width, e.g. the Yaesu FTDX101MP and FTDX10 — points it at a DX
/// station whose carrier frequency is already known, and the audio
/// reaching this decoder is therefore *already* band-limited to that
/// slice. Searching ±250 Hz is not the decoder being clever about where
/// to look; it is the decoder matching what the hardware left in the
/// passband.
///
/// Two consequences follow, and both are load-bearing:
///
/// - **[`SniperRequest::eq_mode`] is here because of the filter.** An
///   analogue filter's skirt tilts the passband, and local equalisation
///   is what flattens it again. EQ is a property of the *input audio*,
///   not of this search strategy — which is why
///   [`DecodeRequest::eq_mode`] exists too, and why on flat synthetic
///   input (an `ft4sim` corpus has no receiver filter at all) it can
///   only cost decodes.
/// - **A-priori hints are orthogonal to this.** [`SniperRequest::ap_hint`]
///   works and is passed through to the decoder — but AP is not part of
///   what makes this the sniper, and the same hint reaches the wide-band
///   [`DecodeRequest::ap_hint`] for FT8, FT4 and every FST4 sub-mode.
///   The two were once coupled; see [`SupportsWideBandAp`].
///
/// It is **not** a general "hunt one known station" convenience. Reading
/// it that way — which an earlier version of this comment invited, by
/// offering "or when hunting one known station" as an alternative —
/// leads to treating it as the natural shape for any sked-style mode
/// and to concluding that AP belongs to it. Neither follows.
///
/// `sync_min` defaults to 0.8 (looser than [`DecodeRequest`]'s typical
/// 1.0-2.0) because the narrow band already excludes the strong
/// adjacent signals a low threshold would otherwise admit.
///
/// Replaces FT8's `decode_sniper`/`decode_sniper_eq`/`decode_sniper_ap`
/// and FT4's `decode_sniper_ap`/`_with_options` (issue #191). FT8's
/// `decode_sniper_sic` (in-band interferer subtraction before a second
/// relaxed-threshold pass) is dropped rather than ported — it had zero
/// callers anywhere in the crate.
#[cfg(feature = "ft8")]
pub struct SniperRequest<'a, P: FrameDecodable, Pol: MessagePolicy = DefaultPolicy> {
    pub(crate) audio: &'a [i16],
    pub(crate) target_freq: f32,
    pub(crate) sync_min: f32,
    pub(crate) depth: DecodeDepth,
    pub(crate) max_cand: usize,
    pub(crate) strictness: DecodeStrictness,
    pub(crate) eq_mode: EqMode,
    pub(crate) ap_hint: Option<&'a ApHint>,
    /// Set via [`SniperRequest::on_result`] — see
    /// [`DecodeRequest::on_result`]'s doc comment for the delivery-
    /// order/dedup contract (same rules apply here).
    pub(crate) on_result: Option<OnResultCallback<'a, P>>,
    /// Set via [`SniperRequest::budget`].
    pub(crate) budget: Option<BudgetCheck<'a>>,
    /// Half-width of the search window, Hz. Set via
    /// [`SniperRequest::search_hz`].
    pub(crate) search_hz: f32,
    /// See [`DecodeRequest`]'s field of the same name.
    pub(crate) policy: Pol,
    _protocol: core::marker::PhantomData<P>,
}

#[cfg(feature = "ft8")]
impl<'a, P: SupportsSniper> DecodeRequest<'a, P, DefaultPolicy> {
    /// Narrow-band, single-target preset. **FT8 only** — see
    /// [`SupportsSniper`] for why this is not a gap in the others.
    pub fn sniper(audio: &'a [i16], target_freq: f32, max_cand: usize) -> SniperRequest<'a, P> {
        SniperRequest::new(audio, target_freq, max_cand)
    }
}

#[cfg(feature = "ft8")]
impl<'a, P: SupportsSniper> SniperRequest<'a, P, DefaultPolicy> {
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
            on_result: None,
            budget: None,
            search_hz: 250.0,
            policy: DefaultPolicy,
            _protocol: core::marker::PhantomData,
        }
    }
}

// Same gate as the block above: `SupportsSniper` and `SniperRequest`
// are both `ft8`-only, so without that feature this block names two
// types that do not exist. Caught by the `alloc ft4 fft-extern` leg of
// `scripts/pre-push-check.sh`'s feature matrix, not by `full`.
#[cfg(feature = "ft8")]
impl<'a, P: SupportsSniper, Pol: MessagePolicy> SniperRequest<'a, P, Pol> {
    /// Half-width of the search window in Hz, centred on the target.
    /// Default 250.0, i.e. the 500 Hz span both callers have always
    /// hardcoded.
    ///
    /// **Changing this changes reported SNR, not just recall.**
    /// `coarse_sync` estimates its noise floor from the 40th percentile
    /// of exactly this window, and that estimate is what candidate
    /// scores — and FT4's `snr_db` — are normalised against. The 500 Hz
    /// default is matched to the roofing-filter passband this path is
    /// premised on: wide enough to sample real noise rather than filter
    /// stopband, narrow enough that FT4's own 83.3 Hz occupied
    /// bandwidth does not contaminate the percentile. See
    /// `docs/notes/SNR_FORMULAS.md`.
    ///
    /// Exposed because the width is a **candidate-population lever**
    /// worth measuring, and could not be measured while it was a
    /// literal at the dispatch site (issue #306). On FST4 the embedded
    /// runtime is dominated by false survivors reaching deep decoding —
    /// a real decode costs ~55 ms against ~14 s for a pathological
    /// false one — and narrowing the band is the one multiplier that
    /// the per-candidate cost work never touched. Whether it actually
    /// reduces the false-survivor count is unmeasured: this path also
    /// runs a halved sync gate, which may offset the narrower band.
    pub fn search_hz(mut self, hz: f32) -> Self {
        self.search_hz = hz;
        self
    }

    pub fn sync_min(mut self, v: f32) -> Self {
        self.sync_min = v;
        self
    }
    /// Toggle OSD fallback when the BP staircase fails. See
    /// [`DecodeRequest::osd`] for why `LlrEffort` isn't exposed here.
    pub fn osd(mut self, on: bool) -> Self {
        self.depth = DecodeDepth {
            llr_effort: LlrEffort::Full,
            osd: on,
        };
        self
    }
    pub fn strictness(mut self, s: DecodeStrictness) -> Self {
        self.strictness = s;
        self
    }
    /// Local per-symbol equalisation of the symbol spectra.
    ///
    /// **A property of the input audio, not of the search.** It exists
    /// to flatten a passband that an *analogue* filter has tilted — a
    /// transceiver's roofing filter, or any band-pass ahead of the
    /// decoder. That is why it appears on both request types: the
    /// narrow-band one because a roofing filter is the reason that path
    /// exists at all, and this one because filtered audio can equally
    /// be handed to a wide-band decode.
    ///
    /// Measured both ways. `Local` recovers an FT8 signal sitting at a
    /// band-pass edge that `Off` misses entirely, through the wide-band
    /// SIC engine (`ft8::decode`'s `eq_mode_recovers_bpf_edge_signal`).
    /// On flat input it can only cost: the `ft4sim`-generated FT4
    /// golden has no receiver filter at all, and `Local` loses two
    /// decodes of fourteen there.
    ///
    /// Default `Off`, which is right for recorded or simulated audio
    /// and wrong for a narrowed receiver.
    pub fn eq_mode(mut self, e: EqMode) -> Self {
        self.eq_mode = e;
        self
    }
    /// See [`DecodeRequest::on_result`] — same contract (this request
    /// type is always parallel-strategy-shaped, so the "may fire for a
    /// duplicate that's later excluded" caveat always applies here).
    pub fn on_result(mut self, cb: OnResultCallback<'a, P>) -> Self {
        self.on_result = Some(cb);
        self
    }

    /// See [`DecodeRequest::budget`] — same predicate, same contract.
    /// A sniper search is a handful of candidates in one ±250 Hz
    /// window, so there is nothing worth reordering here: the budget is
    /// polled per candidate, in the existing order.
    pub fn budget(mut self, check: BudgetCheck<'a>) -> Self {
        self.budget = Some(check);
        self
    }

    /// See [`DecodeRequest::also_accept`] — same contract. A sniper
    /// search reaches the same per-candidate text stage.
    pub fn also_accept<F: Fn(&Wsjt77Fields) -> bool + Sync>(
        self,
        f: F,
    ) -> SniperRequest<'a, P, AlsoAccept<F>>
    where
        P: SupportsMessageFilter,
    {
        self.with_policy(AlsoAccept(f))
    }

    /// See [`DecodeRequest::message_filter`] — same contract.
    pub fn message_filter<F: Fn(&Wsjt77Fields) -> bool + Sync>(
        self,
        f: F,
    ) -> SniperRequest<'a, P, Only<F>>
    where
        P: SupportsMessageFilter,
    {
        self.with_policy(Only(f))
    }

    /// See [`DecodeRequest::codec_filter`] — same contract.
    pub fn codec_filter(self) -> SniperRequest<'a, P, CodecVerdict>
    where
        P: SupportsMessageFilter,
    {
        self.with_policy(CodecVerdict)
    }

    /// No strategy pointer to rebuild here — [`Self::decode`] calls
    /// [`SupportsSniper::__sniper`] directly, since there is only one.
    fn with_policy<Q: MessagePolicy>(self, policy: Q) -> SniperRequest<'a, P, Q> {
        SniperRequest {
            audio: self.audio,
            target_freq: self.target_freq,
            sync_min: self.sync_min,
            depth: self.depth,
            max_cand: self.max_cand,
            strictness: self.strictness,
            eq_mode: self.eq_mode,
            ap_hint: self.ap_hint,
            on_result: self.on_result,
            budget: self.budget,
            search_hz: self.search_hz,
            policy,
            _protocol: core::marker::PhantomData,
        }
    }

    pub fn decode(&self) -> DecodeOutcome<P> {
        P::__sniper(self)
    }
}

#[cfg(feature = "ft8")]
impl<'a, P: SupportsSniper> SniperRequest<'a, P>
where
    P::Msg: WsjtApCompatible,
{
    /// A-priori callsign/grid/report hint. The BP decoder locks the known
    /// bits at high confidence, effectively reducing the number of
    /// unknown bits and lowering the decode threshold by 1-3 dB when the
    /// hint matches a station actually on air.
    ///
    /// **AP is not a narrow-band technique**, and its presence here says
    /// nothing about where it applies. The same hint reaches the
    /// wide-band [`DecodeRequest::ap_hint`] for `Ft8`, `Ft4` and every
    /// FST4 sub-mode. Do not infer from this method's location that a
    /// station must be frequency-known, or that AP belongs with a
    /// roofing filter — it is orthogonal to the ±250 Hz window, which is
    /// the thing that actually makes this the sniper.
    pub fn ap_hint(mut self, ap: &'a ApHint) -> Self {
        self.ap_hint = Some(ap);
        self
    }
}
