//! Protocol-agnostic decode pipeline (basic path, no AP hints).
//!
//! Generic versions of `decode_frame` and `decode_frame_subtract` that drive
//! sync → downsample → LLR → FEC for any `P: Protocol`. AP-assisted decoding
//! (which depends on the 77-bit WSJT message bit layout) lives in
//! protocol-specific crates.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use num_complex::Complex;
#[cfg(not(feature = "std"))]
use num_traits::Float;

use super::dsp::downsample::{DownsampleCfg, build_fft_cache, downsample_cached};
use super::dsp::subtract::SubtractCfg;
use super::equalize::{EqMode, equalize_local};
use super::llr::{
    compute_llr_fast, compute_llr_partial, compute_snr_db, descramble_info, symbol_spectra,
    sync_quality,
};
use super::protocol::BpPooledFec;
use super::sync::{SyncCandidate, coarse_sync, fine_sync_power_per_block};
use super::tx::codeword_to_itone;
use super::{FecCodec, FecOpts, MessageCodec, Protocol};

// ── Stage-timing trace (host diagnostic only) ───────────────────────────────
//
// `MFSK_TRACE_STAGE_FT4`/`MFSK_TRACE_STAGE_FST4` env vars, same idiom as
// `ft8::decode_block::process_candidates`'s existing `MFSK_TRACE_PHANTOM`:
// zero cost when unset (one `env::var` check per `decode_frame_impl` call),
// `eprintln!`s the per-stage wall-clock + candidate counts that found
// FST4's real hotspot (issue #245: OSD escalation attempts mostly failing,
// not the redundant-candidate pattern issue #244 fixed). Left in
// permanently rather than added-and-reverted so future investigations
// don't have to rebuild this from scratch — see
// `~/.claude/plans/moonlit-snuggling-puzzle.md`'s phase-wise benchmark
// plan. `NSYNC_FAIL`/`NSYNC_PASS`/`OSD_ATTEMPT` are global counters (not
// thread-local): fine for a debug env var read by one investigation at a
// time, not designed for isolating concurrent decode_frame_impl calls
// from different application threads.
#[cfg(feature = "std")]
static TRACE_NSYNC_FAIL: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "std")]
static TRACE_NSYNC_PASS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
#[cfg(feature = "std")]
static TRACE_OSD_ATTEMPT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

#[cfg(feature = "std")]
fn stage_trace_enabled<P: Protocol>() -> bool {
    let var = match P::ID {
        super::ProtocolId::Ft4 => "MFSK_TRACE_STAGE_FT4",
        super::ProtocolId::Fst4 => "MFSK_TRACE_STAGE_FST4",
        _ => return false,
    };
    std::env::var(var).is_ok()
}

/// FFT cache for the initial large forward transform; reusable across passes.
///
/// Opaque wrapper (issue #206, part of the pre-0.8.0 public-API review):
/// used to be `pub type FftCache = Vec<Complex<f32>>`, which leaked
/// `num_complex::Complex` — a dependency's type, not this crate's own —
/// into the public API. There's no public constructor and no way to
/// inspect the contents; obtain one from a `decode_frame`-family return
/// value / [`crate::msg::decode_request::DecodeOutcome::fft_cache`] and
/// pass it straight back into
/// [`crate::msg::decode_request::DecodeRequest::fft_cache`] or
/// `decode_frame`'s `precomputed_fft` param.
#[derive(Clone)]
pub struct FftCache(pub(crate) Vec<Complex<f32>>);

impl FftCache {
    pub(crate) fn as_slice(&self) -> &[Complex<f32>] {
        &self.0
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// How much extra work the BP staircase does per candidate before falling
/// back to more expensive strategies. The only axis embedded targets ever
/// configure — see [`DecodeDepth::osd`] for the (host-only) OSD escalation
/// axis.
///
/// Each bit's log-likelihood ratio (LLR) can be estimated by looking at
/// just its own symbol, or jointly across 2 or 3 *adjacent* symbols — a
/// wider joint estimate is a more reliable LLR (correlated symbol-decision
/// errors partially cancel) but costs proportionally more to compute, and
/// BP is tried again from scratch each time a wider estimate is added.
/// `LlrEffort` picks how wide this staircase climbs before giving up on a
/// candidate.
///
/// FT8-only in practice: `process_candidate_basic` below (the engine
/// FT4/FST4 share) always computes all LLR variants unconditionally and
/// never reads this field — only FT8's own `ft8::decode_block` engine has
/// an actual `Minimal`/`Full` staircase. Kept on the shared type (rather
/// than an FT8-local field) so [`DecodeDepth`] has one shape across every
/// protocol using [`crate::msg::decode_request::DecodeRequest`] (issue #191).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LlrEffort {
    /// Only the two cheap 1-symbol LLR estimates. ESP32 ship default — the
    /// 2-symbol/3-symbol estimates empirically add zero extra decodes on
    /// power-budgeted busy-band references (S3 log 2026-05-21; host
    /// re-measurement 2026-07-26: +8ms, 0 extra decodes on `qso3_busy.wav`).
    Minimal,
    /// All four LLR estimates, up to the 3-symbol joint one. Host default —
    /// full recall.
    Full,
}

/// Decode cost/recall configuration: [`LlrEffort`] plus whether to escalate
/// to OSD when the BP staircase fails.
///
/// `osd` is host-only: the OSD dispatch code is compiled out of
/// non-`fft-rustfft` builds entirely, so `osd: true` is a silent no-op on
/// embedded rather than a footgun. OSD has never shipped on an ESP32 target
/// and there is no plan to add it there — this isn't a current tuning
/// choice, it's a permanent architectural boundary.
///
/// Redesigned in 0.8.0 (issue #182 follow-up, then issue #191) from
/// FT8-local 3-/4-variant enums (`BpAll`/`BpAllOsd`/…) into this single
/// orthogonal struct shared by every protocol. The single-variant `Bp` rung
/// (llra-only, no all-variants pass) was retired in 0.7.0 — no production
/// caller was found by issue #74, and the cheapest staircase step never
/// functioned as a power-budget escape hatch.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DecodeDepth {
    pub llr_effort: LlrEffort,
    pub osd: bool,
}

impl DecodeDepth {
    /// ESP32 ship config: cheapest LLR effort, OSD off.
    pub const EMBEDDED: Self = Self {
        llr_effort: LlrEffort::Minimal,
        osd: false,
    };
    /// Full LLR effort, no OSD — host "fast" baseline (was `BpAll`).
    pub const BP_ONLY: Self = Self {
        llr_effort: LlrEffort::Full,
        osd: false,
    };
    /// Full LLR effort + OSD fallback — host default (was `BpAllOsd`).
    pub const FULL: Self = Self {
        llr_effort: LlrEffort::Full,
        osd: true,
    };
}

/// OSD depth-escalation gates: `(osd_attempt_min, osd_depth3_min)`.
///
/// The `12`/`18` pair was calibrated against FT8's `N_SYNC=21` (3 blocks x
/// 7-symbol Costas): 12/21 ~ attempt-OSD-at-all, 18/21 ~ escalate to
/// depth-3/depth-4. FT4's `N_SYNC=16` (4 blocks x 4-symbol Costas) is
/// smaller — `nsync` can never reach 18 there (empirically confirmed via
/// `ft4_diag_weak_trials`, issue #72: even -14dB AWGN decodes topped out
/// around 15/16), so depth-3 OSD and the depth-4 Top-K rescue were
/// silently dead code for every FT4 candidate. Scale by the same ratio
/// the FT8 numbers imply, applied to FT4's own `N_SYNC` (16 * 12/21 ~ 9,
/// 16 * 18/21 ~ 14) — reproduces 12/18 exactly for FT8 (`P::N_SYNC == 21`).
///
/// FST4's `N_SYNC=40` (5 blocks x 8-symbol Costas) is the opposite
/// problem: 18/40=45% is a far *looser* bar than FT8's 18/21=86%, so
/// roughly half of all real candidates cleared it regardless of actual
/// signal quality — not dead code, but the wrong kind of live code.
/// `Ldpc240_101::decode_soft` tries OSD twice per LLR variant at whatever
/// depth is requested (raw LLR, then WSJT-X's `zsave`-style running-BP-sum
/// retry, `fec/ldpc240_101/mod.rs:148-197` — both genuinely needed, issue
/// #146), across up to 5 LLR variants (`llra/llrb/llre/llrc/llrd`) — so
/// escalating unnecessarily is expensive: `fst4_60_diag_osd_escalation`
/// (`tests/fst4_sweep.rs`) measured the WSJT-X FST4-60 golden WAV at the
/// unscaled gates: 24 of 50 candidates attempted OSD depth-2/3 (only 1
/// succeeded), for 3.7 s combined, vs 2 escalating further to depth-4 for
/// another 2.1 s — on a WAV whose real signals were all found well under
/// either threshold.
///
/// Unlike FT4, reusing the same `N_SYNC`-scaled formula for FST4 (→
/// 23/34) is NOT safe: a controlled A/B (`FST4_BENCHMARK.md` section 8)
/// measured a real ~0.5 dB AWGN sensitivity regression — some real FST4
/// signals' `nsync` genuinely falls in [18, 34), unlike FT4 where the
/// scaled threshold only ever unlocked previously-dead code.
/// `osd_attempt_min` stays the shared `12` (raising it was most of that
/// 0.5 dB loss); `osd_depth3_min=20` is a hand-calibrated value verified
/// directly against the real `fst4_snr_sweep` AWGN/CCIR sweep (not the
/// `N_SYNC` formula) — matches the documented pre-fix baseline within
/// sampling noise on all 4 channels, plus FST4-120/300 AWGN spot-checks.
///
/// Integer round-to-nearest (`(A + B/2) / B`) instead of the f32
/// `.round()` this originally used — same result for FT4's `N_SYNC=16`
/// (9/14 either way), no float ops on a path embedded/no_std builds also
/// compile.
///
/// Exposed as `pub` (alongside [`process_candidate_basic`]) so
/// diagnostics/benchmarks that re-implement the staircase outside this
/// module (e.g. `tests/fst4_sweep.rs`) read the real gate instead of
/// duplicating the literals — a prior duplicated copy went stale after
/// this function's `(12, 20)` FST4 branch landed while the copy stayed
/// at the pre-fix `(12, 18)`.
///
/// **FT8 analog**: FT8 never calls this function — it has its own
/// bespoke OSD-fallback dispatch in `ft8::decode_block::osd_strategy`
/// (private module), reached by bypassing [`crate::engine::FecCodec`]
/// entirely (same root cause as issue #198). Independent
/// implementation, independently calibrated — review both when
/// tuning either (issue #192).
///
/// `pub` only under the `internal-testing` feature (issue #203) — no
/// in-crate production caller (FT4/FST4 reach this gate through
/// [`process_candidate_basic`], not directly); exists only for
/// `tests/fst4_sweep.rs`-style diagnostics to read the real gate. See
/// [`decode_frame`]'s doc comment for the feature-gating rationale.
#[cfg(feature = "internal-testing")]
pub fn osd_escalation_gates<P: Protocol>() -> (u32, u32) {
    osd_escalation_gates_impl::<P>()
}

#[cfg(not(feature = "internal-testing"))]
#[allow(dead_code)] // only reachable from tests/ (internal-testing feature)
pub(crate) fn osd_escalation_gates<P: Protocol>() -> (u32, u32) {
    osd_escalation_gates_impl::<P>()
}

fn osd_escalation_gates_impl<P: Protocol>() -> (u32, u32) {
    if P::ID == super::ProtocolId::Ft4 {
        ((12 * P::N_SYNC + 10) / 21, (18 * P::N_SYNC + 10) / 21)
    } else if P::ID == super::ProtocolId::Fst4 {
        (12, 20)
    } else {
        (12, 18)
    }
}

/// Decode strictness: trades off sensitivity vs false-positive rate.
///
/// `process_candidate_basic` bypasses `osd_max_errors` for FST4 (see the
/// `is_fst4` gate below — issue #146: WSJT-X's own FST4 decoder has no
/// such gate), so in practice these
/// numbers are FT4-exclusive. `Normal` (FT4's hardcoded strictness,
/// issue #72) was retuned 2026-07-18 against a `ft4sim` AWGN/CCIR sweep
/// (`docs/notes/FT4_BENCHMARK.md`) — no longer a placeholder copy of the
/// FT8 calibration. `Strict`/`Deep` are unused by any current caller but
/// kept for the API shape; their numbers are the original FT8-copied
/// values, unverified for FT4.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum DecodeStrictness {
    Strict,
    #[default]
    Normal,
    Deep,
}

impl DecodeStrictness {
    /// Upper bound on `hard_errors` for non-AP OSD decode.
    ///
    /// `Normal`'s values were retuned for FT4 (issue #72, 2026-07-18) by
    /// sweeping against `ft4sim`-generated AWGN/CCIR WAVs and picking the
    /// loosest thresholds that gained real (golden-message) recall without
    /// also growing false-accepts (any CRC-passing decode beyond the golden
    /// one) — see the `ft4_strictness_probe` test and
    /// `docs/notes/FT4_BENCHMARK.md` section 5 for the measurements.
    /// `Strict`/`Deep` remain the original FT8-copied placeholders.
    pub fn osd_max_errors(self, osd_depth: u8) -> u32 {
        match (self, osd_depth) {
            (Self::Strict, 3) => 20,
            (Self::Strict, 4) => 24,
            (Self::Strict, _) => 22,
            (Self::Normal, 3) => 28,
            (Self::Normal, 4) => 30,
            (Self::Normal, _) => 31,
            (Self::Deep, 3) => 30,
            (Self::Deep, 4) => 36,
            (Self::Deep, _) => 40,
        }
    }

    /// Upper bound on `hard_errors` for AP-assisted decode passes, graded by
    /// the number of locked bits (heavier locks → tighter threshold, since
    /// random bits flipping to agree with the lock is increasingly
    /// unlikely). Calibrated from a synthetic QSO scenario (REPORT AP at
    /// -18 dB: 15% FP rate with old thresholds 30/36) — shared by FT8's
    /// per-candidate AP loop and [`crate::msg::pipeline_ap`]'s generic
    /// sniper (issue #191 type consolidation; previously duplicated
    /// byte-for-byte in both places).
    pub fn ap_max_errors(self, locked_bits: usize) -> u32 {
        match (self, locked_bits >= 55) {
            (Self::Strict, true) => 20,
            (Self::Strict, false) => 24,
            (Self::Normal, true) => 25,
            (Self::Normal, false) => 30,
            (Self::Deep, true) => 30,
            (Self::Deep, false) => 36,
        }
    }

    /// FT8's own flat (not `osd_depth`-tiered) hard-error acceptance
    /// ceiling — shared by the BP staircase and the OSD fallback
    /// (`ft8::decode_block::process_candidates`/`osd_strategy`), which
    /// both apply the same bound WSJT-X does unconditionally on depth
    /// (`ft8b.f90:422`). Unlike [`Self::osd_max_errors`] (FT4-specific,
    /// depth-tiered), FT8's real dispatch has no such tiering to port —
    /// this is a single WSJT-X-faithful number, not three.
    ///
    /// **`Normal = 36` is WSJT-X's own universal ceiling — do not
    /// retune without re-running the issue #72 CCIR-fading sweep this
    /// value was widened *to*.** It was `22` before that investigation
    /// (see `osd_strategy.rs`'s `OSD_HARDERRORS_MAX`-era history
    /// comment): a deliberate mfsk-core-specific tightening that
    /// silently discarded real golden decodes under heavy fading,
    /// found by an AWGN/CCIR sweep against a *known* golden message.
    /// Widening back to 36 recovered them with zero regression across
    /// the full FT8 regression suite. `Normal` must stay at 36 to
    /// preserve that fix as the default.
    ///
    /// `Strict = 22` reuses that exact historical value — real prior
    /// art from the issue #72 investigation (known effect: filters
    /// `N1API F2VX 73`/`N1API HA6FQ -23`/`CQ EA2BFM IN83` on
    /// `qso3_busy.wav`), not a fresh guess — for callers who explicitly
    /// want fewer false-accepts at that recall cost.
    ///
    /// `Deep = 37` deliberately *exceeds* WSJT-X's own ceiling — an
    /// mfsk-core-original extension beyond 36, since WSJT-X itself has
    /// no looser tier to port.
    ///
    /// **Retuned 40 → 37 (2026-08-10, issue #253)**, prompted by a
    /// reproducible false decode via WebFT8's `Deep` + `.sic_early()`
    /// phase-2 pipeline (`7Y8CIH HN1GD OP30` on `qso3_busy.wav`,
    /// `hard_errors=31`). **This retune does not eliminate that specific
    /// decode** — 31 clears even `Normal`'s 36, so it isn't a `Deep`-
    /// specific problem; it's a garden-variety false accept sitting
    /// inside WSJT-X's own accepted 36-error ceiling, one that happens to
    /// only surface via `.sic_early()`'s residual-search architecture on
    /// this file (plain single-pass/`Strict` don't produce it; `Strict`
    /// at 22 does reject it). The retune below is independently justified
    /// by a real sweep, not a fix for that one anecdote. Calibrated the
    /// same way issue #72 calibrated FT4's numbers: `ft8_strictness_probe`
    /// (`tests/ft8_sweep.rs`) drives `DecodeRequest<Ft8>` with each level
    /// across both the plain single-pass strategy and `.sic_early()` over
    /// 16 `ft8sim` AWGN/CCIR cells (320 trials/level/strategy) at/below
    /// the sensitivity crossing, and reports golden recall (the known
    /// transmitted message) alongside false-accept count (any CRC-passing
    /// decode that *isn't* the golden message — unambiguous here, since
    /// each trial encodes exactly one real signal). Sweeping the ceiling
    /// value itself (37/38/39/40) found **golden recall was already
    /// saturated at 37** (105/320 single-pass, 108/320 sic_early — bit-
    /// for-bit identical from 37 through 40) while false-accepts kept
    /// climbing (single-pass 15→16, sic_early 20→21) — i.e. every value
    /// above 37 was pure false-accept risk with zero additional real
    /// recall on this corpus. At 36 (`Normal`) golden drops to 99/103;
    /// the entire `Normal → Deep` recall gain happens in the single
    /// 36 → 37 step. No longer "not yet swept" — this *is* the sweep,
    /// same discipline as [`Self::osd_max_errors`]'s FT4 retune, though
    /// that method's own `Strict`/`Deep` arms remain unswept placeholders.
    pub fn ft8_nharderrors_max(self) -> u32 {
        match self {
            Self::Strict => 22,
            Self::Normal => 36,
            Self::Deep => 37,
        }
    }
}

/// One successfully decoded message. Protocol-agnostic.
///
/// `info` carries the FEC's K information bits — for LDPC(174,91) that's 91
/// bits (77 message + 14 CRC for Wsjt77-family), for LDPC(240,101) that's 101
/// bits (77 message + 24 CRC for FST4), for uvpacket it's 91 bits with the
/// `PacketBytesMessage` layout (4-bit length + 80-bit payload + 7-bit CRC-7).
/// The pipeline is agnostic to the layout; `MessageCodec::unpack` /
/// `MessageCodec::verify_info` interpret it per-protocol.
#[derive(Debug, Clone)]
pub struct DecodeResult {
    /// FEC-decoded information bits; length = `<P::Fec as FecCodec>::K`.
    pub info: Box<[u8]>,
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub hard_errors: u32,
    pub sync_score: f32,
    pub pass: u8,
    /// Coefficient of variation of the per-block Costas powers — near 0 for
    /// stable channels, elevated under QSB or fading.
    pub sync_cv: f32,
    pub snr_db: f32,
}

impl DecodeResult {
    /// Slice the leading 77 message bits — the convention shared by every
    /// Wsjt77-family protocol (FT8 / FT4 / FT2 / FST4 / Q65). For uvpacket
    /// this still returns a 77-bit slice, but its interpretation is
    /// uvpacket-specific (length code + bytes + CRC fragment).
    ///
    /// Panics if `info` is shorter than 77 bits.
    pub fn message77(&self) -> &[u8] {
        &self.info[..77]
    }
}

/// Protocols with a dedicated 2-D (frequency + time) coarse-candidate
/// refine search wired into [`process_candidate_basic`] — currently `Ft4`
/// ([`super::sync2d::ft4_sync_search`]) and every FST4 sub-mode
/// ([`super::sync2d::fst4_sync_search`]).
///
/// Sealed by construction to this crate's own protocol modules: not a
/// `sealed`-trait pattern, just documentation of intent, since the
/// generic fallback this trait replaced (a bare `refine_candidate::<P>`
/// call, time-only, no frequency correction) was confirmed unreachable by
/// every call site in the crate before removal (issue #192) — FT8 has its
/// own separate bespoke engine and never instantiates this pipeline at
/// all. Adding a new protocol here means giving it a real `*_sync_search`
/// function first, not falling back to an unvalidated generic path.
///
/// Everything any current or foreseeable [`GenericPipelineProtocol`]
/// implementor's real WSJT-X SNR formula could need, gathered once at
/// the call site (issue #255). Unused fields cost nothing — all
/// borrowed, not owned.
///
/// Visibility mirrors [`GenericPipelineProtocol`] itself (issue #203):
/// `pub` only under `internal-testing`, `pub(crate)` otherwise — it
/// only ever appears in that trait's `snr_db` method signature.
#[cfg(feature = "internal-testing")]
pub struct SnrCtx<'a> {
    /// [`symbol_spectra`]`::<P>` output, `/1000`-scaled.
    pub cs: &'a [Complex<f32>],
    /// [`encode_tones_for_snr`]`::<P>` output.
    pub itone: &'a [u8],
    /// Coarse-sync candidate score (`SyncCandidate::score`) — FT4's
    /// `candidate(2,icand)` equivalent.
    // FT4's `snr_db` override is this field's only reader, so a build
    // with `fst4` but not `ft4` — a real CI feature-matrix cell — sees
    // it as dead.
    #[cfg_attr(not(feature = "ft4"), allow(dead_code))]
    pub cand_score: f32,
    /// Coarse-sync candidate frequency (Hz) — `candidates(icand,1)`
    /// equivalent. FST4's baseline lookup (`candidates(icand,5)`) is
    /// keyed by this, not the fine-refined frequency.
    #[allow(dead_code)] // read once FST4's override lands
    pub cand_freq_hz: f32,
    /// Big forward-FFT of the whole slot's raw audio
    /// ([`build_fft_cache`]'s output) — WSJT-X `c_bigfft` equivalent.
    /// Already computed by the caller for downsampling; FST4's
    /// baseline extraction reuses it rather than requiring its own.
    #[allow(dead_code)] // read once FST4's override lands
    pub fft_cache: &'a [Complex<f32>],
    /// The [`DownsampleCfg`] `fft_cache` was built from — supplies
    /// `fft1_size` (⇒ WSJT-X's `df1`) to FST4's baseline extraction.
    #[allow(dead_code)] // read once FST4's override lands
    pub ds_cfg: &'a DownsampleCfg,
    /// Fine-refined candidate frequency (Hz) — the frequency `cs` was
    /// actually computed at (`WSJT-X`'s `fc_synced`), as opposed to
    /// `cand_freq_hz`'s coarse pre-refine value. FST4's own `xsig`
    /// re-derivation needs this: WSJT-X's `fst4_decode.f90` downsamples
    /// its bitmetrics input at `fc_synced`, not the coarse candidate
    /// frequency `get_candidates_fst4.f90`'s baseline is keyed by.
    #[allow(dead_code)] // read once FST4's override lands
    pub refined_freq_hz: f32,
    /// Sample index (in the *downsampled* baseband) of the first
    /// symbol — the `i_start`/`i0` [`symbol_spectra`] was actually
    /// called with. Needed alongside `refined_freq_hz` to recompute a
    /// fresh, deterministic `cs` at the exact same alignment.
    #[allow(dead_code)] // read once FST4's override lands
    pub i_start: i32,
}
#[cfg(not(feature = "internal-testing"))]
pub(crate) struct SnrCtx<'a> {
    /// [`symbol_spectra`]`::<P>` output, `/1000`-scaled.
    pub cs: &'a [Complex<f32>],
    /// [`encode_tones_for_snr`]`::<P>` output.
    pub itone: &'a [u8],
    /// Coarse-sync candidate score (`SyncCandidate::score`) — FT4's
    /// `candidate(2,icand)` equivalent.
    // FT4's `snr_db` override is this field's only reader, so a build
    // with `fst4` but not `ft4` — a real CI feature-matrix cell — sees
    // it as dead.
    #[cfg_attr(not(feature = "ft4"), allow(dead_code))]
    pub cand_score: f32,
    /// Coarse-sync candidate frequency (Hz) — `candidates(icand,1)`
    /// equivalent. FST4's baseline lookup (`candidates(icand,5)`) is
    /// keyed by this, not the fine-refined frequency.
    #[allow(dead_code)] // read once FST4's override lands
    pub cand_freq_hz: f32,
    /// Big forward-FFT of the whole slot's raw audio
    /// ([`build_fft_cache`]'s output) — WSJT-X `c_bigfft` equivalent.
    /// Already computed by the caller for downsampling; FST4's
    /// baseline extraction reuses it rather than requiring its own.
    #[allow(dead_code)] // read once FST4's override lands
    pub fft_cache: &'a [Complex<f32>],
    /// The [`DownsampleCfg`] `fft_cache` was built from — supplies
    /// `fft1_size` (⇒ WSJT-X's `df1`) to FST4's baseline extraction.
    #[allow(dead_code)] // read once FST4's override lands
    pub ds_cfg: &'a DownsampleCfg,
    /// Fine-refined candidate frequency (Hz) — the frequency `cs` was
    /// actually computed at (`WSJT-X`'s `fc_synced`), as opposed to
    /// `cand_freq_hz`'s coarse pre-refine value. FST4's own `xsig`
    /// re-derivation needs this: WSJT-X's `fst4_decode.f90` downsamples
    /// its bitmetrics input at `fc_synced`, not the coarse candidate
    /// frequency `get_candidates_fst4.f90`'s baseline is keyed by.
    #[allow(dead_code)] // read once FST4's override lands
    pub refined_freq_hz: f32,
    /// Sample index (in the *downsampled* baseband) of the first
    /// symbol — the `i_start`/`i0` [`symbol_spectra`] was actually
    /// called with. Needed alongside `refined_freq_hz` to recompute a
    /// fresh, deterministic `cs` at the exact same alignment.
    #[allow(dead_code)] // read once FST4's override lands
    pub i_start: i32,
}

/// `pub` only under the `internal-testing` feature (issue #203) — see
/// [`decode_frame`]'s doc comment for the feature-gating rationale.
#[cfg(feature = "internal-testing")]
pub trait GenericPipelineProtocol: Protocol
where
    Self::Fec: BpPooledFec,
{
    /// Reported SNR (dB) for a decoded candidate. Default is the
    /// generic adjacent-tone-ratio heuristic ([`compute_snr_db`]) —
    /// known *not* to match any current protocol's real WSJT-X
    /// formula (issue #255's finding), kept only as a fallback for
    /// protocols not yet individually ported. **Overrides MUST cite
    /// the WSJT-X source file:line** their formula matches (see
    /// [`ft4_snr_db`]'s doc comment for the expected style).
    fn snr_db(ctx: SnrCtx<'_>) -> f32 {
        compute_snr_db::<Self>(ctx.cs, ctx.itone)
    }
}
#[cfg(not(feature = "internal-testing"))]
pub(crate) trait GenericPipelineProtocol: Protocol
where
    Self::Fec: BpPooledFec,
{
    /// Reported SNR (dB) for a decoded candidate. Default is the
    /// generic adjacent-tone-ratio heuristic ([`compute_snr_db`]) —
    /// known *not* to match any current protocol's real WSJT-X
    /// formula (issue #255's finding), kept only as a fallback for
    /// protocols not yet individually ported. **Overrides MUST cite
    /// the WSJT-X source file:line** their formula matches (see
    /// [`ft4_snr_db`]'s doc comment for the expected style).
    fn snr_db(ctx: SnrCtx<'_>) -> f32 {
        compute_snr_db::<Self>(ctx.cs, ctx.itone)
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Per-candidate processing
// ──────────────────────────────────────────────────────────────────────────

/// Decode a single sync candidate through the basic pipeline.
///
/// `fft_cache` must match the protocol's [`DownsampleCfg`]. `known` is used
/// to prevent redundant OSD work on frequencies with an existing decode.
///
/// `pub` only under the `internal-testing` feature (issue #203) — see
/// [`decode_frame`]'s doc comment for the feature-gating rationale.
#[cfg(feature = "internal-testing")]
pub fn process_candidate_basic<P: GenericPipelineProtocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    sync_q_min: u32,
) -> Option<DecodeResult>
where
    P::Fec: BpPooledFec,
{
    process_candidate_basic_impl::<P>(
        cand, fft_cache, cfg, depth, strictness, known, eq_mode, sync_q_min, None,
    )
}

#[cfg(not(feature = "internal-testing"))]
// Only reachable via `decode_frame`/`decode_frame_subtract`, themselves
// only called by `ft4`/`fst4`'s `decode` modules — dead code under any
// feature combination excluding both (e.g. `jt9`/`jt65`/`q65`-only).
#[allow(dead_code)]
pub(crate) fn process_candidate_basic<P: GenericPipelineProtocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    sync_q_min: u32,
) -> Option<DecodeResult>
where
    P::Fec: BpPooledFec,
{
    process_candidate_basic_impl::<P>(
        cand, fft_cache, cfg, depth, strictness, known, eq_mode, sync_q_min, None,
    )
}

/// FT4's own SNR formula (`ft4_decode.f90:226,452-457`):
///
/// ```text
///   snr = candidate(2,icand) - 1.0
///   xsnr = 10·log10(snr) - 14.8   (snr > 0.0, else -21.0)
///   nsnr = nint(max(-21.0, xsnr))
/// ```
///
/// `cand_score` must be the *coarse* `getcandidates4.f90`-equivalent
/// candidate score (`SyncCandidate::score` as returned by
/// [`crate::engine::ft4_coarse::ft4_coarse_sync`], already a faithful
/// port of that subroutine) — **not** `ft4_sync_search`'s own later
/// coherent Δt-search score (stored separately as `DecodeResult::
/// sync_score`), a different WSJT-X quantity entirely.
///
/// Overrides the generic adjacent-tone `compute_snr_db` for FT4
/// specifically, via `Ft4`'s [`GenericPipelineProtocol::snr_db`]
/// override in `ft4/decode.rs` (issue #255 follow-up, 2026-08-10 —
/// found via the same investigation that fixed FT8's `xsnr2`:
/// `compute_snr_db` is a single heuristic standing in for every
/// `GenericPipelineProtocol` implementor's own real WSJT-X formula,
/// and FT4's real one is this, not an adjacent-tone ratio). Verified
/// against a real local `jt9 -5` build on a clean isolated synthetic
/// signal: this formula lands within ~1.1 dB of jt9's own probed
/// `xsnr` (`-1.77` vs `-0.655`, `nsnr` displayed `-1`), down from
/// `compute_snr_db`'s ~6.9 dB gap (`-7.52` dB) on the same signal.
/// FST4 and any future `GenericPipelineProtocol` implementor keep the
/// trait's default (`compute_snr_db`) for now — each has its own
/// distinct real formula, not ported yet, and not assumed to be a
/// variant of this one (see issue #255).
///
/// `pub(crate)` (not `fn` private to this module) so the override in
/// `ft4/decode.rs` can call it — the override itself must live next
/// to `Ft4`'s `impl GenericPipelineProtocol` block, not here, so a
/// reader scanning that impl sees every protocol-specific override in
/// one place rather than half of them hidden in the generic engine.
// `ft4/decode.rs`'s `snr_db` override is the only caller, so a build
// with `fst4` but not `ft4` — a real CI feature-matrix cell — sees
// this as dead. Silenced rather than `cfg`'d away so the intra-doc
// links to it from `GenericPipelineProtocol::snr_db` (two of them)
// keep resolving under every feature set.
#[cfg_attr(not(feature = "ft4"), allow(dead_code))]
pub(crate) fn ft4_snr_db(cand_score: f32) -> f32 {
    let snr = cand_score - 1.0;
    if snr > 0.0 {
        (10.0 * snr.log10() - 14.8).max(-21.0)
    } else {
        -21.0
    }
}

fn process_candidate_basic_impl<P: GenericPipelineProtocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
    depth: DecodeDepth,
    strictness: DecodeStrictness,
    known: &[DecodeResult],
    eq_mode: EqMode,
    sync_q_min: u32,
    // When `Some`, reuses a refine result [`dedup_refined_candidates`]
    // already computed for this candidate — downsample + RMS-normalise
    // + `fst4_sync_search`/`ft4_sync_search` — instead of recomputing
    // it here (issue #244 follow-up: without this, every surviving
    // candidate paid that refine cost *twice*, once in the pre-decode
    // dedup pass and once again here, which measurably outweighed the
    // BP/OSD savings the dedup pass itself achieves on files with only
    // a handful of true near-duplicates — a real perf regression, not
    // a hypothetical one, caught by a controlled single-threaded
    // wall-clock A/B after the fact). `None` for every other caller
    // (FT4, and every `internal-testing` direct caller) — behaves
    // exactly as before.
    precomputed_refine: Option<(Vec<Complex<f32>>, f32, i32, f32)>,
) -> Option<DecodeResult>
where
    P::Fec: BpPooledFec,
{
    let ntones = P::NTONES as usize;
    let n_sym = P::N_SYMBOLS as usize;
    let ds_rate = 12_000.0 / P::NDOWN as f32;
    let tx_start = P::TX_START_OFFSET_S;

    let precomputed_freq = precomputed_refine
        .as_ref()
        .map(|&(_, freq_hz, i0, score)| (freq_hz, i0, score));
    let cd0 = match precomputed_refine {
        Some((cd0, ..)) => cd0,
        None => {
            let mut cd0 = downsample_cached(fft_cache, cand.freq_hz, cfg);
            // RMS-normalise the downsampled baseband to unit power.
            // Matches WSJT-X `ft4_decode.f90:231-232`:
            //   sum2 = sum(|cd2|²) / (NMAX/NDOWN)
            //   cd2  = cd2 / sqrt(sum2)
            // The LLR_SCALE=2.83 used by `compute_llr` is calibrated
            // against unit-RMS input; without this normalisation the
            // per-tone magnitudes feeding `tanh(llr/2)` inside BP land
            // at the wrong scale and the decoder converges on
            // systematically wrong codewords that just happen to
            // satisfy CRC-14 (the 4-CRC-false-positive symptom on the
            // FT4 reference WAV — issue #18). `refine_candidate_position`
            // applies this same normalisation before handing back a
            // `precomputed_refine` cd0, so this branch and that one
            // always agree on scale.
            let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
            if sum2 > f32::EPSILON {
                let inv = 1.0 / sum2.sqrt();
                for c in cd0.iter_mut() {
                    *c *= inv;
                }
            }
            cd0
        }
    };

    let _ = ntones;
    let _ = n_sym;
    // BP iteration budget: WSJT-X's `ft8b.f90:96` and `fst4/decode240_101.f90:27`
    // both use `max_iterations=30`, but `ft4_decode.f90:194` uses 40 — FT4 is
    // the outlier, not the other two. Scoped to `P::ID == Ft4` (issue #72,
    // discovered while checking whether BP/OSD strength explains the residual
    // AWGN gap after `docs/notes/FT4_BENCHMARK.md` section 9) so FT8/FST4 stay
    // byte-identical.
    let bp_max_iter: u32 = if P::ID == super::ProtocolId::Ft4 {
        40
    } else {
        30
    };
    let cd0_base = cd0;

    // Attempt a full decode (symbol_spectra -> nsync gate -> BP -> OSD) at
    // one explicit `(freq_hz, i0, score)` position. Factored out of the
    // single-position call below so FT4 can retry it at up to 3 positions
    // (see the segment loop further down) without duplicating the LLR/BP/
    // OSD logic.
    let try_position = |freq_hz: f32, i0: i32, score: f32| -> Option<DecodeResult> {
        let df_hz = freq_hz - cand.freq_hz;
        let cd0 = super::sync2d::freq_shift_cd0(&cd0_base, df_hz, ds_rate);
        let refined = SyncCandidate {
            freq_hz,
            dt_sec: (i0 as f32) / ds_rate - tx_start,
            score,
        };

        let cs_raw = symbol_spectra::<P>(&cd0, i0);
        let nsync = sync_quality::<P>(&cs_raw);
        if nsync <= sync_q_min {
            #[cfg(feature = "std")]
            TRACE_NSYNC_FAIL.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            return None;
        }
        #[cfg(feature = "std")]
        TRACE_NSYNC_PASS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

        let per_block = fine_sync_power_per_block::<P>(&cd0, i0);
        let sync_cv = if !per_block.is_empty() {
            let n = per_block.len() as f32;
            let mean = per_block.iter().sum::<f32>() / n;
            if mean > f32::EPSILON {
                let var = per_block.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / n;
                var.sqrt() / mean
            } else {
                0.0
            }
        } else {
            0.0
        };

        let decode = |cs: &[Complex<f32>]| -> Option<DecodeResult> {
            let fec = P::Fec::default();
            // Reused across every `decode_soft_pooled` call below (up to
            // 15 for FST4's full LLR-variant × OSD-escalation ladder, 12
            // for FT4) — one allocation per candidate instead of one per
            // call. See `BpPooledFec`'s doc comment (issue #199/#201's
            // shape, ported to the generic pipeline).
            let mut bp_scratch = <P::Fec as BpPooledFec>::Scratch::default();
            let bp_opts = FecOpts {
                bp_max_iter,
                osd_depth: 0,
                ap_mask: None,
                // Thread the protocol's message-codec verifier so CRC-bearing
                // protocols (FT8/FT4/FST4 → Wsjt77 → CRC-14) keep their
                // existing reject-on-CRC-fail behaviour. uvpacket-style
                // codecs that override `verify_info = |_| true` accept any
                // parity-converged candidate.
                verify_info: Some(<P::Msg as MessageCodec>::verify_info),
                ..FecOpts::default()
            };

            // RX half of the optional bit interleaver — same no-op for
            // protocols with `CODEWORD_INTERLEAVE = None` (FT4/FT8/FST4/etc)
            // as the previous `deinterleave_llr_set` call site.
            let deinterleave = |v: &mut Vec<f32>| {
                if let Some(table) = P::CODEWORD_INTERLEAVE {
                    deinterleave_llr_vec(v, table);
                }
            };
            let mut try_bp = |llr: &Vec<f32>, pass_id: u8| -> Option<DecodeResult> {
                let mut r = fec.decode_soft_pooled(llr, &bp_opts, &mut bp_scratch)?;
                let itone = encode_tones_for_snr::<P>(&r.info, &fec);
                let snr_db = P::snr_db(SnrCtx {
                    cs,
                    itone: &itone,
                    cand_score: cand.score,
                    cand_freq_hz: cand.freq_hz,
                    fft_cache,
                    ds_cfg: cfg,
                    refined_freq_hz: refined.freq_hz,
                    i_start: i0,
                });
                // FT4 pre-LDPC scramble (WSJT-X `genft4.f90:64`): undo
                // the rvec XOR before presenting the 77-bit payload.
                descramble_info::<P>(&mut r.info);
                Some(DecodeResult {
                    info: r.info.into_boxed_slice(),
                    freq_hz: refined.freq_hz,
                    dt_sec: refined.dt_sec,
                    hard_errors: r.hard_errors,
                    sync_score: refined.score,
                    pass: pass_id,
                    sync_cv,
                    snr_db,
                })
            };

            // Lazy nsym staircase: compute each LLR variant only as this
            // loop reaches it, instead of eagerly building the whole
            // `LlrSet` (nsym=1, 2, `LLR_NSYM_MID`, `LLR_NSYM_MAX`) up
            // front regardless of whether a cheap variant already lets BP
            // succeed. FST4's `LLR_NSYM_MAX=8` rung enumerates
            // `4^8=65536` tone-combination hypotheses per group — 128-
            // 256x FT8/FT4's own deepest rung — so skipping it whenever
            // an earlier variant already decodes is the dominant win.
            // Same variants, same try-order, same `pass_id`s as the
            // previous eager version; if every variant fails BP (as
            // today), `llr_set` below ends up fully populated exactly
            // once per field, so OSD's own variant reuse further down is
            // unaffected either way.
            let mut llr_set = compute_llr_fast::<P, f32>(cs);
            deinterleave(&mut llr_set.llra);
            deinterleave(&mut llr_set.llrd);
            if let Some(r) = try_bp(&llr_set.llra, 0) {
                return Some(r);
            }

            llr_set.llrb = compute_llr_partial::<P, f32, f32>(cs, 2);
            deinterleave(&mut llr_set.llrb);
            if let Some(r) = try_bp(&llr_set.llrb, 1) {
                return Some(r);
            }

            if let Some(mid) = P::LLR_NSYM_MID {
                llr_set.llre = compute_llr_partial::<P, f32, f32>(cs, mid as usize);
                // `llre` has no interleave handling in the previous
                // `deinterleave_llr_set` either — harmless while
                // `CODEWORD_INTERLEAVE` is `None` for every protocol
                // that sets `LLR_NSYM_MID` today (FST4 only).
                if let Some(r) = try_bp(&llr_set.llre, 6) {
                    return Some(r);
                }
            }

            llr_set.llrc = compute_llr_partial::<P, f32, f32>(cs, P::LLR_NSYM_MAX as usize);
            deinterleave(&mut llr_set.llrc);
            if let Some(r) = try_bp(&llr_set.llrc, 2) {
                return Some(r);
            }

            if let Some(r) = try_bp(&llr_set.llrd, 3) {
                return Some(r);
            }

            // llre (nsym=P::LLR_NSYM_MID, e.g. FST4's nsym=4 rung — see
            // `ModulationParams::LLR_NSYM_MID`) is empty for every protocol
            // that doesn't set LLR_NSYM_MID, so this is a Vec instead of a
            // fixed array only to make that slot conditional; no behaviour
            // change for FT8/FT4/etc.
            let mut variants: Vec<(&Vec<f32>, u8)> = Vec::with_capacity(5);
            variants.push((&llr_set.llra, 0u8));
            variants.push((&llr_set.llrb, 1));
            if !llr_set.llre.is_empty() {
                variants.push((&llr_set.llre, 6));
            }
            variants.push((&llr_set.llrc, 2));
            variants.push((&llr_set.llrd, 3));

            // WSJT-X's own FST4 decoder (`fst4_decode.f90`) has no
            // post-OSD hard-error gate: `decode240_101` is called
            // unconditionally after BP fails, and its only acceptance
            // test is `nharderrors.ge.0 .and. unpk77_success`
            // (`fst4_decode.f90:570`) — i.e. "OSD converged to a
            // CRC-24-verified codeword", full stop, no upper bound on how
            // many bits OSD had to flip to get there. `osd_max_errors` is
            // FT8-calibrated (doc'd as "can re-tune later", issue #72)
            // and was never re-tuned for FST4: near its own sensitivity
            // threshold, every OSD result that did run had a
            // CRC-verified hard-error count above `osd_max_errors`
            // (rejected despite being provably correct) — issue #146.
            // Bypass it for FST4 to match WSJT-X: trust the CRC-24
            // verification inside `decode_soft` alone.
            //
            // (A parallel pre-OSD *attempt* score gate, `osd_score_min`,
            // used to sit here too, bypassed for both FST4 and FT4 for
            // the identical reason — issue #146/#72 section 12. It ended
            // up with no live caller on any protocol once both bypassed
            // it and was removed outright, issue #230.)
            let is_fst4 = P::ID == super::ProtocolId::Fst4;
            // See `osd_escalation_gates`'s doc comment for the full
            // derivation/history of these two thresholds.
            let (osd_attempt_min, osd_depth3_min) = osd_escalation_gates::<P>();
            if depth.osd && nsync >= osd_attempt_min {
                let freq_dup = known
                    .iter()
                    .any(|r| (r.freq_hz - cand.freq_hz).abs() < 20.0);
                if !freq_dup {
                    #[cfg(feature = "std")]
                    TRACE_OSD_ATTEMPT.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                    let osd_depth: u8 = if nsync >= osd_depth3_min { 3 } else { 2 };
                    let osd_opts = FecOpts {
                        bp_max_iter,
                        osd_depth: osd_depth as u32,
                        ap_mask: None,
                        verify_info: Some(<P::Msg as MessageCodec>::verify_info),
                        ..FecOpts::default()
                    };
                    for (llr, _) in &variants {
                        if let Some(mut r) = fec.decode_soft_pooled(llr, &osd_opts, &mut bp_scratch)
                        {
                            if !is_fst4 && r.hard_errors >= strictness.osd_max_errors(osd_depth) {
                                continue;
                            }
                            let itone = encode_tones_for_snr::<P>(&r.info, &fec);
                            let snr_db = P::snr_db(SnrCtx {
                                cs,
                                itone: &itone,
                                cand_score: cand.score,
                                cand_freq_hz: cand.freq_hz,
                                fft_cache,
                                ds_cfg: cfg,
                                refined_freq_hz: refined.freq_hz,
                                i_start: i0,
                            });
                            descramble_info::<P>(&mut r.info);
                            return Some(DecodeResult {
                                info: r.info.into_boxed_slice(),
                                freq_hz: refined.freq_hz,
                                dt_sec: refined.dt_sec,
                                hard_errors: r.hard_errors,
                                sync_score: refined.score,
                                pass: if osd_depth == 3 { 5 } else { 4 },
                                sync_cv,
                                snr_db,
                            });
                        }
                    }
                    // OSD depth-4 Top-K pruning gated on high sync quality.
                    if nsync >= osd_depth3_min {
                        let osd4_opts = FecOpts {
                            bp_max_iter,
                            osd_depth: 4,
                            ap_mask: None,
                            verify_info: Some(<P::Msg as MessageCodec>::verify_info),
                            ..FecOpts::default()
                        };
                        for (llr, _) in &variants {
                            if let Some(mut r) =
                                fec.decode_soft_pooled(llr, &osd4_opts, &mut bp_scratch)
                            {
                                if !is_fst4 && r.hard_errors >= strictness.osd_max_errors(4) {
                                    continue;
                                }
                                let itone = encode_tones_for_snr::<P>(&r.info, &fec);
                                let snr_db = P::snr_db(SnrCtx {
                                    cs,
                                    itone: &itone,
                                    cand_score: cand.score,
                                    cand_freq_hz: cand.freq_hz,
                                    fft_cache,
                                    ds_cfg: cfg,
                                    refined_freq_hz: refined.freq_hz,
                                    i_start: i0,
                                });
                                descramble_info::<P>(&mut r.info);
                                return Some(DecodeResult {
                                    info: r.info.into_boxed_slice(),
                                    freq_hz: refined.freq_hz,
                                    dt_sec: refined.dt_sec,
                                    hard_errors: r.hard_errors,
                                    sync_score: refined.score,
                                    pass: 13,
                                    sync_cv,
                                    snr_db,
                                });
                            }
                        }
                    }
                }
            }

            None
        };

        match eq_mode {
            EqMode::Off => decode(&cs_raw),
            EqMode::Local => {
                let mut cs_eq = cs_raw.clone();
                equalize_local::<P>(&mut cs_eq);
                decode(&cs_eq)
            }
        }
    };

    // FT4 uses `ft4_sync_search`: a coherent full-slot Δt search (WSJT-X
    // `ft4_decode.f90` isync=1/2 + `sync4d.f90` scorer). A literal port of
    // WSJT-X's `iseg=1,2,3` per-segment retry structure (try up to 3
    // different Δt positions, not just the single global best) was
    // implemented and measured here — empirically ruled out, not just
    // unimplemented: `ft4_diag_segment_retry` (`tests/ft4_sweep.rs`,
    // issue #72, `docs/notes/FT4_BENCHMARK.md` section 11) found 0/17
    // rescues once the diagnostic was corrected to apply the same
    // `hard_errors >= osd_max_errors` gate and golden-message check
    // production does — an earlier uncorrected pass had over-reported
    // 10/17 by skipping that gate. Reverted to the single collapsed pass
    // to avoid 3x the search/decode cost for zero measured benefit.
    //
    // FST4 uses `fst4_sync_search`: faithful port of WSJT-X
    // `fst4_decode.f90:879-925`. Coarse pass sweeps ±1.5 s (full slot) so
    // the winner is always near the true peak; fine pass ±7×0.02·baud ×
    // ±4 samples locks in. Previous local-window approach (Sync2dConfig
    // ±10 samples) caused regression because noise peaks at the window
    // edge displaced the fine pass outside reach of the true position.
    //
    // `P: GenericPipelineProtocol` is implemented only for `Ft4` and each
    // FST4 sub-mode (issue #192) — no third case exists to fall back to,
    // so this is a plain two-way dispatch, not a `P::ID`-exhaustive match.
    //
    // Skipped entirely when `precomputed_refine` already carries this
    // candidate's refined position — see that parameter's doc comment.
    let (freq_hz, i0, score) = if let Some(r) = precomputed_freq {
        r
    } else if P::ID == super::ProtocolId::Ft4 {
        let s2 = super::sync2d::ft4_sync_search::<P>(&cd0_base, cand);
        (s2.freq_hz, s2.i0, s2.score)
    } else {
        let s2 = super::sync2d::fst4_sync_search::<P>(&cd0_base, cand);
        (s2.freq_hz, s2.i0, s2.score)
    };

    // A WSJT-X-style `smax` early exit (`ft4_decode.f90:279`:
    // `if(smax.lt.1.2) cycle`) was implemented and measured here — using
    // `ft4_sync_search`'s own coherent score, not `cand.score` — and
    // reverted for negligible benefit (dapper-soaring-nest plan Phase 4,
    // `FT4_BENCHMARK.md` section 15): a safely-margined cutoff only
    // filtered 0.5% of non-golden candidates in the calibration sweep
    // (`ft4_diag_smax_calibration`, `tests/ft4_sweep.rs`) — junk scores
    // cluster tightly just below the golden-succeeding floor rather than
    // spread far below it, so there's no safe gap wide enough to filter
    // much without risking a real signal.

    try_position(freq_hz, i0, score)
}

/// `llr[INTERLEAVE[j]] = channel_llr[j]` — inverse of the TX-side
/// permutation. Allocates one temporary `Vec<f32>` per call (per LLR
/// variant); the cost is tiny next to BP/OSD.
fn deinterleave_llr_vec(llr: &mut [f32], table: &[u16]) {
    debug_assert_eq!(
        llr.len(),
        table.len(),
        "interleave table length must match LLR length"
    );
    let original: Vec<f32> = llr.to_vec();
    for j in 0..llr.len() {
        llr[table[j] as usize] = original[j];
    }
}

/// Re-encode FEC info bits back into tones for SNR estimation.
///
/// Phase A reduced this to a 3-line helper: `r.info[..]` already
/// carries the K-bit info the FEC produced, including any CRC bits
/// that `MessageCodec::verify_info` already accepted. Feeding it
/// straight back into `fec.encode` reproduces the same codeword as
/// the previous "extract msg77 → recompute CRC → encode" path —
/// bit-identical because verifier acceptance enforces
/// `info[77..K] == crc(info[..77])` at the moment of acceptance.
fn encode_tones_for_snr<P: Protocol>(info: &[u8], fec: &P::Fec) -> Vec<u8> {
    let mut cw = vec![0u8; P::Fec::N];
    fec.encode(info, &mut cw);
    codeword_to_itone::<P>(&cw)
}

/// Wrap `cb` so it only forwards results not already present in `known`
/// (by `info` equality) — `None` in, `None` out.
///
/// This engine has no `known` parameter of its own (`decode_frame`/
/// `decode_frame_subtract` below don't take one) — `ft4`/`fst4`'s
/// `dedup_known` post-filters the *returned* `Vec` against `known`
/// after the fact instead. That's fine for the returned `Vec`, but
/// `on_result` fires *inside* this engine, before that post-filter
/// ever runs — so without this wrapper, a candidate matching `known`
/// still fires the caller's callback and then silently never appears
/// in the returned `Vec`, violating `DecodeRequest::on_result`'s own
/// documented contract (exact-match for the sequential SIC strategies;
/// even the parallel single-pass strategy's weaker "superset" contract
/// doesn't license dropping something the caller explicitly named via
/// `.known(...)`). Same root pattern as issue #243's `decode_block`
/// fix and its `ft8::decode`/`SupportsSicEarly::__staged_sic`
/// follow-up — closed here at the wrapper level instead of threading
/// `known` through this generic (protocol-agnostic) engine itself.
#[cfg(any(feature = "ft4", feature = "fst4"))]
pub(crate) fn known_filtered_on_result<'a>(
    known: &'a [DecodeResult],
    cb: Option<&'a (dyn Fn(&DecodeResult) + Sync)>,
) -> Option<impl Fn(&DecodeResult) + Sync + use<'a>> {
    cb.map(move |cb| {
        move |r: &DecodeResult| {
            if !known.iter().any(|k| k.info == r.info) {
                cb(r);
            }
        }
    })
}

// ──────────────────────────────────────────────────────────────────────────
// Frame-level entry points
// ──────────────────────────────────────────────────────────────────────────

/// Decode one slot of audio: coarse sync → candidates → BP/OSD per candidate.
///
/// `pub` only under the `internal-testing` feature (issue #203): the
/// crate's own `tests/` sweep/probe binaries are compiled as separate
/// crates and need real `pub` visibility to call this directly; on the
/// default feature set it's `pub(crate)`, since #191's `DecodeRequest`
/// is the supported public entry point.
#[cfg(feature = "internal-testing")]
#[allow(clippy::too_many_arguments)]
pub fn decode_frame<P: GenericPipelineProtocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    sync_q_min: u32,
    precomputed_fft: Option<&[Complex<f32>]>,
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> (Vec<DecodeResult>, FftCache)
where
    P::Fec: BpPooledFec,
{
    decode_frame_impl::<P>(
        audio,
        cfg,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        strictness,
        eq_mode,
        sync_q_min,
        precomputed_fft,
        on_result,
    )
}

#[cfg(not(feature = "internal-testing"))]
// Only called by `ft4`/`fst4`'s `decode` modules — dead code under any
// feature combination excluding both (e.g. `jt9`/`jt65`/`q65`-only).
#[allow(dead_code)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn decode_frame<P: GenericPipelineProtocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    sync_q_min: u32,
    precomputed_fft: Option<&[Complex<f32>]>,
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> (Vec<DecodeResult>, FftCache)
where
    P::Fec: BpPooledFec,
{
    decode_frame_impl::<P>(
        audio,
        cfg,
        freq_min,
        freq_max,
        sync_min,
        freq_hint,
        depth,
        max_cand,
        strictness,
        eq_mode,
        sync_q_min,
        precomputed_fft,
        on_result,
    )
}

/// Cheap refine-only step for [`dedup_refined_candidates`]: downsample +
/// RMS-normalise + sync-search only, mirroring the same steps at the top
/// of [`process_candidate_basic_impl`] but stopping *before*
/// `symbol_spectra`/LLR/BP/OSD. Returns the already-normalised `cd0`
/// alongside the refined `(freq_hz, i0, score)` triple — the caller
/// threads both back into `process_candidate_basic_impl`'s
/// `precomputed_refine` parameter for whichever candidates survive
/// dedup, so that function's own downsample/normalise/sync-search
/// block is skipped entirely rather than redone (issue #244 follow-up:
/// an earlier version of this function returned only the triple,
/// discarding `cd0` and letting `process_candidate_basic_impl`
/// recompute everything for survivors — doubling the refine cost for
/// every one of them, which a controlled single-threaded wall-clock
/// A/B measured as a net *regression*, exceeding the BP/OSD savings on
/// a file with few true near-duplicates).
pub(crate) fn refine_candidate_position<P: GenericPipelineProtocol>(
    cand: &SyncCandidate,
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
) -> (Vec<Complex<f32>>, f32, i32, f32)
where
    P::Fec: BpPooledFec,
{
    let mut cd0 = downsample_cached(fft_cache, cand.freq_hz, cfg);
    // Same RMS-normalisation `process_candidate_basic_impl` applies —
    // keeps `score` on a comparable scale across candidates so the
    // dedup tie-break below is meaningful, and keeps this `cd0` at the
    // same scale `process_candidate_basic_impl` expects when reused.
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
    let s2 = if P::ID == super::ProtocolId::Ft4 {
        super::sync2d::ft4_sync_search::<P>(&cd0, cand)
    } else {
        super::sync2d::fst4_sync_search::<P>(&cd0, cand)
    };
    (cd0, s2.freq_hz, s2.i0, s2.score)
}

/// Pre-decode near-duplicate dedup on *refined* sync positions —
/// WSJT-X `fst4_decode.f90:339-353`'s "remove duplicate candidates"
/// pass, ported (issue #244). Coarse candidates a few Hz apart can
/// independently refine onto the *same* true `(freq, dt)` once
/// `fst4_sync_search`'s wide coherent search locks them all onto the
/// real signal — without this, every one of them pays the full
/// LLR/BP/OSD staircase before a *post-decode*, message-based dedup
/// (`decode_frame_impl`'s own dedup further down) throws away all but
/// one. Measured (issue #244): up to 9x redundant BP/OSD calls for one
/// real FST4 signal, all but one immediately discarded.
///
/// Tolerance matches WSJT-X: `0.10 * baud` in frequency, `±2`
/// downsampled samples in the refined sync position
/// (`fst4_decode.f90:344,348`). Unlike WSJT-X's index-order tie-break
/// (which relies on its own candidate list already being
/// strength-sorted by the CLEAN algorithm), candidates here are sorted
/// by refined `score` descending first, so survivorship is
/// deterministic and score-driven regardless of `coarse_sync`'s own
/// ordering.
///
/// Scoped to the non-FT4 branch (i.e. FST4 today) — FT4's own
/// `ft4_coarse_sync` measured *zero* redundant firings on both a real
/// WSJT-X sample and a clean synthetic signal (issue #244's own
/// investigation), so this stays where it was actually measured to
/// help rather than being applied on spec.
///
/// Returns `(SyncCandidate, cd0, freq_hz, i0, score)` for survivors
/// only — the refine result callers thread into
/// `process_candidate_basic_impl`'s `precomputed_refine` parameter, so
/// it's never recomputed for anything this function already computed
/// it for.
type RefinedSurvivor = (SyncCandidate, Vec<Complex<f32>>, f32, i32, f32);

fn dedup_refined_candidates<P: GenericPipelineProtocol>(
    candidates: Vec<SyncCandidate>,
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
) -> Vec<RefinedSurvivor>
where
    P::Fec: BpPooledFec,
{
    #[cfg(feature = "parallel")]
    let refined: Vec<(Vec<Complex<f32>>, f32, i32, f32)> = candidates
        .par_iter()
        .map(|c| refine_candidate_position::<P>(c, fft_cache, cfg))
        .collect();
    #[cfg(not(feature = "parallel"))]
    let refined: Vec<(Vec<Complex<f32>>, f32, i32, f32)> = candidates
        .iter()
        .map(|c| refine_candidate_position::<P>(c, fft_cache, cfg))
        .collect();

    let freq_tol = 0.10 * P::TONE_SPACING_HZ;
    const I0_TOL: i32 = 2;

    let mut order: Vec<usize> = (0..candidates.len()).collect();
    order.sort_by(|&a, &b| {
        refined[b]
            .3
            .partial_cmp(&refined[a].3)
            .unwrap_or(core::cmp::Ordering::Equal)
    });

    let mut kept_positions: Vec<(f32, i32)> = Vec::new();
    let mut keep = vec![false; candidates.len()];
    for idx in order {
        let (_, f, i0, _) = &refined[idx];
        let dup = kept_positions
            .iter()
            .any(|&(kf, ki)| (f - kf).abs() < freq_tol && (i0 - ki).abs() <= I0_TOL);
        if !dup {
            kept_positions.push((*f, *i0));
            keep[idx] = true;
        }
    }

    candidates
        .into_iter()
        .zip(refined)
        .zip(keep)
        .filter_map(|((c, (cd0, f, i0, s)), k)| if k { Some((c, cd0, f, i0, s)) } else { None })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn decode_frame_impl<P: GenericPipelineProtocol>(
    audio: &[i16],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    eq_mode: EqMode,
    sync_q_min: u32,
    // Reuse a caller-supplied FFT cache (e.g. from an earlier
    // `DecodeOutcome::fft_cache`) instead of rebuilding it from `audio`
    // — mirrors FT8's own `decode_frame_inner`'s `precomputed_fft`
    // parameter (`ft8::decode`). Was silently accepted-but-ignored here
    // before this fix: `DecodeRequest::fft_cache` is an ungated field on
    // the shared `DecodeRequest<P>` struct, but only `Ft8`'s
    // `FrameDecodable` impl ever threaded it through — `Ft4`/FST4 always
    // rebuilt from `audio` regardless of what the caller passed.
    precomputed_fft: Option<&[Complex<f32>]>,
    // Fires once per accepted candidate, inside the per-candidate
    // closure below and *before* the cross-candidate dedup pass that
    // follows — same "possible transient duplicate" contract as FT8's
    // own parallel single-pass strategy (`ft8::decode::decode_frame_inner`),
    // not the sequential exact-match one. See `DecodeRequest::on_result`'s
    // doc comment for the full delivery-order writeup.
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> (Vec<DecodeResult>, FftCache)
where
    P::Fec: BpPooledFec,
{
    // FT4's own coarse-candidate stage (`engine::ft4_coarse::ft4_coarse_sync`,
    // a faithful `getcandidates4.f90` port) replaces the generic 2-D
    // (freq × lag) Costas-correlation search: WSJT-X's FT4 candidate
    // finder has no lag dimension at all, and the generic search's
    // up-to-8 lag-distinct candidates per frequency are redundant
    // downstream for FT4 — `ft4_sync_search` (below) already searches
    // Δt absolutely, ignoring each candidate's own `dt_sec`. See
    // `engine::ft4_coarse` module doc / `~/.claude/plans/dapper-soaring-nest.md`.
    #[cfg(feature = "std")]
    let trace = stage_trace_enabled::<P>();
    #[cfg(not(feature = "std"))]
    #[allow(unused_variables)]
    let trace = false;
    #[cfg(feature = "std")]
    let __trace_t0 = trace.then(std::time::Instant::now);
    let candidates = if P::ID == super::ProtocolId::Ft4 {
        super::ft4_coarse::ft4_coarse_sync(audio, freq_min, freq_max, sync_min, freq_hint, max_cand)
    } else {
        coarse_sync::<P>(audio, freq_min, freq_max, sync_min, freq_hint, max_cand)
    };
    #[cfg(feature = "std")]
    if let Some(t0) = __trace_t0 {
        eprintln!(
            "TRACE_STAGE coarse_sync={:.1}ms n_candidates={}",
            t0.elapsed().as_secs_f64() * 1000.0,
            candidates.len()
        );
    }
    let fft_cache = FftCache(match precomputed_fft {
        Some(c) => c.to_vec(),
        None => build_fft_cache(audio, cfg),
    });
    if candidates.is_empty() {
        return (Vec::new(), fft_cache);
    }
    #[cfg(feature = "std")]
    if trace {
        TRACE_NSYNC_FAIL.store(0, core::sync::atomic::Ordering::Relaxed);
        TRACE_NSYNC_PASS.store(0, core::sync::atomic::Ordering::Relaxed);
        TRACE_OSD_ATTEMPT.store(0, core::sync::atomic::Ordering::Relaxed);
    }

    // FT4 and FST4 diverge here: FT4 decodes its raw candidates
    // directly (measured zero redundant near-duplicates — issue #244).
    // FST4 first runs `dedup_refined_candidates`, then threads each
    // survivor's already-computed refine result into
    // `process_candidate_basic_impl` via `precomputed_refine` so it's
    // never recomputed (see that parameter's doc comment for why this
    // matters: an earlier version of this fix let survivors recompute
    // it, which measurably cost more than the BP/OSD it saved).
    let raw: Vec<DecodeResult> = if P::ID == super::ProtocolId::Ft4 {
        #[cfg(feature = "std")]
        let __trace_t1 = trace.then(std::time::Instant::now);
        #[cfg(feature = "parallel")]
        let raw: Vec<DecodeResult> = candidates
            .par_iter()
            .filter_map(|cand| {
                let r = process_candidate_basic::<P>(
                    cand,
                    fft_cache.as_slice(),
                    cfg,
                    depth,
                    strictness,
                    &[],
                    eq_mode,
                    sync_q_min,
                )?;
                if let Some(cb) = on_result {
                    cb(&r);
                }
                Some(r)
            })
            .collect();
        #[cfg(not(feature = "parallel"))]
        let raw: Vec<DecodeResult> = candidates
            .iter()
            .filter_map(|cand| {
                let r = process_candidate_basic::<P>(
                    cand,
                    fft_cache.as_slice(),
                    cfg,
                    depth,
                    strictness,
                    &[],
                    eq_mode,
                    sync_q_min,
                )?;
                if let Some(cb) = on_result {
                    cb(&r);
                }
                Some(r)
            })
            .collect();
        #[cfg(feature = "std")]
        if let Some(t1) = __trace_t1 {
            eprintln!(
                "TRACE_STAGE decode_loop={:.1}ms nsync_fail={} nsync_pass={} osd_attempt={} n_decoded={}",
                t1.elapsed().as_secs_f64() * 1000.0,
                TRACE_NSYNC_FAIL.load(core::sync::atomic::Ordering::Relaxed),
                TRACE_NSYNC_PASS.load(core::sync::atomic::Ordering::Relaxed),
                TRACE_OSD_ATTEMPT.load(core::sync::atomic::Ordering::Relaxed),
                raw.len()
            );
        }
        raw
    } else {
        #[cfg(feature = "std")]
        let __trace_t1 = trace.then(std::time::Instant::now);
        #[cfg(feature = "std")]
        let candidates_len = candidates.len();
        let deduped = dedup_refined_candidates::<P>(candidates, fft_cache.as_slice(), cfg);
        #[cfg(feature = "std")]
        let deduped_len = deduped.len();
        #[cfg(feature = "std")]
        if let Some(t1) = __trace_t1 {
            eprintln!(
                "TRACE_STAGE dedup_refined_candidates={:.1}ms n_before={} n_after={}",
                t1.elapsed().as_secs_f64() * 1000.0,
                candidates_len,
                deduped_len
            );
        }
        #[cfg(feature = "std")]
        let __trace_t2 = trace.then(std::time::Instant::now);
        #[cfg(feature = "parallel")]
        let raw: Vec<DecodeResult> = deduped
            .into_par_iter()
            .filter_map(|(cand, cd0, freq_hz, i0, score)| {
                let r = process_candidate_basic_impl::<P>(
                    &cand,
                    fft_cache.as_slice(),
                    cfg,
                    depth,
                    strictness,
                    &[],
                    eq_mode,
                    sync_q_min,
                    Some((cd0, freq_hz, i0, score)),
                )?;
                if let Some(cb) = on_result {
                    cb(&r);
                }
                Some(r)
            })
            .collect();
        #[cfg(not(feature = "parallel"))]
        let raw: Vec<DecodeResult> = deduped
            .into_iter()
            .filter_map(|(cand, cd0, freq_hz, i0, score)| {
                let r = process_candidate_basic_impl::<P>(
                    &cand,
                    fft_cache.as_slice(),
                    cfg,
                    depth,
                    strictness,
                    &[],
                    eq_mode,
                    sync_q_min,
                    Some((cd0, freq_hz, i0, score)),
                )?;
                if let Some(cb) = on_result {
                    cb(&r);
                }
                Some(r)
            })
            .collect();
        #[cfg(feature = "std")]
        if let Some(t2) = __trace_t2 {
            eprintln!(
                "TRACE_STAGE decode_loop={:.1}ms nsync_fail={} nsync_pass={} osd_attempt={} n_decoded={}",
                t2.elapsed().as_secs_f64() * 1000.0,
                TRACE_NSYNC_FAIL.load(core::sync::atomic::Ordering::Relaxed),
                TRACE_NSYNC_PASS.load(core::sync::atomic::Ordering::Relaxed),
                TRACE_OSD_ATTEMPT.load(core::sync::atomic::Ordering::Relaxed),
                raw.len()
            );
        }
        raw
    };

    // Dedup by decoded message, keeping the candidate with the highest
    // `sync_score` (the post-refine coherent Costas correlation) rather
    // than the first-processed one. `coarse_sync`'s NMS can keep more
    // than one (freq, dt) candidate per frequency bin, and more than one
    // can independently reach a self-consistent Costas lock on the same
    // real signal (not noise — both land in the same place after
    // `ft4_sync_search`'s refine). This is now mostly cosmetic
    // (`DecodeResult.freq_hz`/`dt_sec` come from the *refined* position,
    // not the raw candidate, so duplicates converge on nearly the same
    // reported values) but keeps the tie-break meaningful for the rare
    // case where refinement doesn't fully converge.
    let mut results: Vec<DecodeResult> = Vec::new();
    for r in raw {
        match results.iter_mut().find(|x| x.info == r.info) {
            Some(existing) if r.sync_score > existing.sync_score => *existing = r,
            Some(_) => {}
            None => results.push(r),
        }
    }
    (results, fft_cache)
}

/// Multi-pass decode with successive signal subtraction. Each pass decodes
/// the residual audio; decoded signals are reconstructed and subtracted so
/// subsequent passes can expose previously-masked weak signals.
#[allow(clippy::too_many_arguments)]
// Only `ft4::decode` calls this (issue #203's pub(crate) demotion made
// that reachability-dependent-on-feature visible to rustc): dead code
// under any feature combination that excludes `ft4` (`fst4`-only,
// `jt9`/`jt65`/`q65`/`uvpacket`, `ft8`+`alloc`/`fft-extern` embedded
// presets, etc).
#[allow(dead_code)]
pub(crate) fn decode_frame_subtract<P: GenericPipelineProtocol>(
    audio: &[i16],
    ds_cfg: &DownsampleCfg,
    sub_cfg: &SubtractCfg,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    freq_hint: Option<f32>,
    depth: DecodeDepth,
    max_cand: usize,
    strictness: DecodeStrictness,
    // Upper bound on SIC rounds, 1..=3 (`DecodeRequest::sic_rounds`
    // already clamps to this range — not re-validated here, this
    // function has exactly one caller). `passes.len() == 3`, so this
    // slices the shared progressive-`sync_min`-relaxation schedule
    // rather than iterating all of it.
    max_rounds: usize,
    sync_q_min: u32,
    // Channel-aware LPF subtract tuning (issue #178/#179 FT4 port).
    // Protocol-specific — mirrors WSJT-X's per-protocol `NFILT`/
    // end-correction choice (`subtractft8.f90` vs `subtractft4.f90`).
    // Passed in rather than derived from `P` to avoid growing the
    // `Protocol` trait for a single generic-pipeline caller (FT4, as
    // of this writing).
    lpf_half: usize,
    lpf_endcorrection: bool,
    refine_freq_radius_hz: f32,
    // Reuse a caller-supplied FFT cache for pass 0 only — every
    // subsequent pass's cache must be rebuilt regardless, since
    // `residual` has been mutated by subtraction by then. Safe to trust
    // unconditionally for pass 0 (no `known`-emptiness gate like FT8's
    // analog needs): unlike FT8, this function never pre-subtracts
    // `known` from `residual` before pass 0 (`known` is only used as a
    // post-filter — see `dedup_known` at each caller), so pass 0's
    // `residual` always equals `audio` verbatim, exactly what a
    // caller-supplied cache built from `audio` represents.
    precomputed_fft: Option<&[Complex<f32>]>,
    // Fires once per result as it's added to `all_results` below — the
    // final acceptance point for this sequential SIC loop, so delivery
    // is an exact match against the returned `Vec`, same order, same
    // contract as FT8's `.sic_rounds()`/`.sic_early()` strategies.
    on_result: Option<&(dyn Fn(&DecodeResult) + Sync)>,
) -> Vec<DecodeResult>
where
    P::Fec: BpPooledFec,
{
    #[cfg(feature = "std")]
    let trace = stage_trace_enabled::<P>();
    #[cfg(not(feature = "std"))]
    #[allow(unused_variables)]
    let trace = false;
    #[cfg(feature = "std")]
    if trace {
        TRACE_NSYNC_FAIL.store(0, core::sync::atomic::Ordering::Relaxed);
        TRACE_NSYNC_PASS.store(0, core::sync::atomic::Ordering::Relaxed);
        TRACE_OSD_ATTEMPT.store(0, core::sync::atomic::Ordering::Relaxed);
    }

    let mut residual = audio.to_vec();
    let mut all_results: Vec<DecodeResult> = Vec::new();
    let passes: &[f32] = &[1.0, 0.75, 0.5][..max_rounds];
    let fec = P::Fec::default();

    for (pass_idx, &factor) in passes.iter().enumerate() {
        #[cfg(feature = "std")]
        let __trace_tp = trace.then(std::time::Instant::now);
        // See the identical `P::ID == Ft4` branch in `decode_frame` above.
        let candidates = if P::ID == super::ProtocolId::Ft4 {
            super::ft4_coarse::ft4_coarse_sync(
                &residual,
                freq_min,
                freq_max,
                sync_min * factor,
                freq_hint,
                max_cand,
            )
        } else {
            coarse_sync::<P>(
                &residual,
                freq_min,
                freq_max,
                sync_min * factor,
                freq_hint,
                max_cand,
            )
        };
        #[cfg(feature = "std")]
        if let Some(tp) = __trace_tp {
            eprintln!(
                "TRACE_STAGE_SIC pass={} coarse_sync={:.1}ms n_candidates={}",
                pass_idx,
                tp.elapsed().as_secs_f64() * 1000.0,
                candidates.len()
            );
        }
        if candidates.is_empty() {
            continue;
        }
        let fft_cache = match (pass_idx, precomputed_fft) {
            (0, Some(c)) => c.to_vec(),
            _ => build_fft_cache(&residual, ds_cfg),
        };

        #[cfg(feature = "std")]
        let __trace_tp2 = trace.then(std::time::Instant::now);
        #[cfg(feature = "parallel")]
        let new: Vec<DecodeResult> = candidates
            .par_iter()
            .filter_map(|cand| {
                process_candidate_basic::<P>(
                    cand,
                    &fft_cache,
                    ds_cfg,
                    depth,
                    strictness,
                    &all_results,
                    EqMode::Off,
                    sync_q_min,
                )
            })
            .collect();
        #[cfg(not(feature = "parallel"))]
        let new: Vec<DecodeResult> = candidates
            .iter()
            .filter_map(|cand| {
                process_candidate_basic::<P>(
                    cand,
                    &fft_cache,
                    ds_cfg,
                    depth,
                    strictness,
                    &all_results,
                    EqMode::Off,
                    sync_q_min,
                )
            })
            .collect();
        #[cfg(feature = "std")]
        if let Some(tp2) = __trace_tp2 {
            eprintln!(
                "TRACE_STAGE_SIC pass={} decode_loop={:.1}ms nsync_fail={} nsync_pass={} osd_attempt={} n_new={}",
                pass_idx,
                tp2.elapsed().as_secs_f64() * 1000.0,
                TRACE_NSYNC_FAIL.swap(0, core::sync::atomic::Ordering::Relaxed),
                TRACE_NSYNC_PASS.swap(0, core::sync::atomic::Ordering::Relaxed),
                TRACE_OSD_ATTEMPT.swap(0, core::sync::atomic::Ordering::Relaxed),
                new.len()
            );
        }

        let mut deduped: Vec<DecodeResult> = Vec::new();
        for r in new {
            if !all_results.iter().any(|k| k.info == r.info)
                && !deduped.iter().any(|x| x.info == r.info)
            {
                deduped.push(r);
            }
        }

        for r in &deduped {
            // `r.info` is post-descramble (FT4 only); re-apply the rvec
            // XOR before re-encoding so the subtracted tones match what
            // was actually on the air. XOR is its own inverse, so calling
            // `descramble_info` here scrambles back to the wire form.
            let mut info_for_tx = r.info.to_vec();
            descramble_info::<P>(&mut info_for_tx);
            let tones = encode_tones_for_snr::<P>(&info_for_tx, &fec);
            // WSJT-X-faithful channel-aware LPF subtract, single shot
            // (issue #177/#178/#179): the old constant-amplitude
            // `subtract_tones` + coarse binary QSB gain (0.5 / 1.0 on
            // `sync_cv > 0.3`) is FT8's pre-0.6.2 design, never
            // migrated here when FT8 moved to `subtract_tones_lpf`. On
            // a synthetic busy-band scenario with a strong
            // Rayleigh-faded interferer 40 Hz from a weak target
            // (`ft4_busy_band_fading_probe.rs`), the old path recovered
            // the target 0/10 seeds; migrating to `subtract_tones_lpf`
            // (this call) recovers it reliably, 10/10.
            //
            // An intermediate version of this code iterated
            // `subtract_tones_lpf` to convergence per candidate (up to
            // 6, later 20, re-fits) — reading `ft4_decode.f90` /
            // `subtractft4.f90` directly showed WSJT-X never does
            // this: `subtractft4` is always a single call, and deeper
            // suppression of a persistent signal comes from the
            // *outer* multi-pass loop above (`for &factor in passes`)
            // re-detecting it as a fresh candidate in a later pass —
            // which this function already does independently of any
            // inner iteration. The inner convergence loop had no
            // WSJT-X counterpart and, once its iteration cap was
            // raised, repeatedly re-fit/re-subtracted the same
            // candidate against its own imperfect model with no
            // independent ground truth: on the real WSJT-X FT4 sample
            // this leaked distortion from `CQ RU AB5XS EM12` (560.0 Hz)
            // into `W9JA PY2APK RRR` (519.4 Hz, ~40 Hz away) and lost
            // that decode (`ft4_wsjtx_sample_iteration_diag.rs`).
            // Removed — the single-shot call here matches every
            // regression guard that previously seemed to require
            // convergence (this synthetic scenario 10/10, the real
            // sample 6/6, FT8's `qso3_busy.wav` 18/18) identically or
            // better.
            //
            // Also refines the carrier frequency first (`refine_freq`'s
            // own doc comment recommends this for real-signal input;
            // wasn't being called here either).
            let refined_freq = super::dsp::subtract::refine_freq(
                &residual,
                &tones,
                r.freq_hz,
                r.dt_sec,
                sub_cfg,
                refine_freq_radius_hz,
                0.1,
            );
            super::dsp::subtract::subtract_tones_lpf(
                &mut residual,
                &tones,
                refined_freq,
                r.dt_sec,
                sub_cfg,
                lpf_half,
                lpf_endcorrection,
            );
        }
        if let Some(cb) = on_result {
            for r in &deduped {
                cb(r);
            }
        }
        all_results.extend(deduped);
    }

    all_results
}
