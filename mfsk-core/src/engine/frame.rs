//! Result, depth and budget types of the decode pipeline.
//!
//! These are what [`crate::decoder`] and the per-mode `decode` modules hand to an
//! application, so they live here, in a public module, and not in the crate-private
//! `engine::pipeline` that produces them (#693).

use alloc::boxed::Box;
use alloc::vec::Vec;

use num_complex::Complex;

/// FFT cache for the initial large forward transform; reusable across passes.
///
/// Opaque wrapper (issue #206, part of the pre-0.8.0 public-API review):
/// used to be `pub type FftCache = Vec<Complex<f32>>`, which leaked
/// `num_complex::Complex` — a dependency's type, not this crate's own —
/// into the public API. There's no public constructor and no way to
/// inspect the contents; obtain one from a `decode_frame`-family return
/// value (the engine request's `DecodeOutcome::fft_cache`) and pass it
/// straight back into the request's `fft_cache` or `decode_frame`'s
/// `precomputed_fft` param.
#[derive(Clone)]
pub struct FftCache(pub(crate) Vec<Complex<f32>>);

impl core::fmt::Debug for FftCache {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FftCache")
            .field("len", &self.0.len())
            .finish_non_exhaustive()
    }
}

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
/// protocol using the engine's frame request (issue #191).
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

/// Wall-clock budget predicate for `DecodeRequest::budget` /
/// `SniperRequest::budget` — returns `false` once the caller's
/// allowance is spent.
///
/// Declared here rather than beside those builders because `engine`
/// never depends on `msg` (the direction is fixed crate-wide), and the
/// generic pipeline below has to name the type. `msg::decode_request`
/// re-exports it, so callers see it where they use it.
///
/// `&dyn Fn(…) + Sync` rather than a bare `fn() -> bool`, for two
/// reasons this crate has already paid for once each:
///
/// - a `fn` pointer cannot capture, so
///   `fst4::rung_major::decode_phase_split_timed`'s `budget_ok: Option<fn() -> bool>`
///   forced its only real consumer
///   (`embedded-shared::fst4_monitor`) to route the deadline through a
///   per-core `UnsafeCell<[i64; 2]>` global. A closure capturing an
///   absolute deadline needs none of that.
/// - `Sync`, not `FnMut`: the predicate is built from a *fixed*
///   captured deadline, so one of them can be shared as-is across a
///   `rayon` batch instead of needing an exclusive borrow per
///   candidate. Same shape, same reasoning as `wspr::decode`'s own
///   `budget` parameter.
///
/// `mfsk-core` deliberately contains no clock: `std::time::Instant::now`
/// is unimplemented on `wasm32-unknown-unknown` and absent on `no_std`.
/// The caller supplies one — host `Instant`, browser
/// `performance.now()`, embedded `esp_timer_get_time`.
pub type BudgetCheck<'a> = &'a (dyn Fn() -> bool + Sync);

/// What a budgeted decode left undone. All-zero (`Default`) means no
/// budget was set, or it was never reached.
///
/// Returned per call rather than accumulated in a global counter (the
/// shape `wspr::instrument` uses) because a per-slot number is exactly
/// what a caller adapting to a deadline needs, and a process-global one
/// cannot give it.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
#[non_exhaustive]
pub struct BudgetReport {
    /// The predicate returned `false` at least once — work was left
    /// undone.
    pub exhausted: bool,
    /// Units of work declined. A candidate on FT8's and FT4's
    /// single-pass engines and on every sniper; a whole SIC *round* on
    /// FT4's `.sic_rounds(n)`, which subtracts a round's decodes as one
    /// batch and so cannot be cut inside one.
    pub candidates_skipped: u32,
    /// Units of work actually run, counted the same way.
    pub stages_run: u32,
    /// Costas sync quality (0..=`N_SYNC`) of the best skipped
    /// candidate. **FT8 only** — that triage number is the key FT8's
    /// scheduler orders by, so it says directly whether the cut took
    /// noise or a station. `None` on FT4 and FST4, which rank by score.
    pub cut_at_sync: Option<u32>,
    /// Sync score of the best skipped candidate, on the scale that
    /// protocol's own search works in: the coarse, baseline-normalised
    /// score on FT8 and FT4 (so FT4's is directly comparable to its
    /// `sync_min`, WSJT-X's own 1.18), and the refined `fst4_sync_search`
    /// score on FST4, which is what its scheduler ranks by. Named after
    /// the embedded FT4 receiver's `SlotOutcome::cut_at_score`, which is
    /// the number that turned out to be worth surfacing to an operator.
    ///
    /// It falls as the budget grows on FT4 and FST4, where it *is* the
    /// ranking key. On FT8 it does not — there `cut_at_sync` is the key
    /// and this is supplementary.
    pub cut_at_score: Option<f32>,
    /// Rows subtracted from the residual before a later search sees it.
    /// **FT8 `SicEarly` only** — the checkpoint-B and -C subtraction
    /// loops, which are the only place this crate spends a budget on
    /// something other than a candidate.
    ///
    /// It is here because a cut inside those loops is a different event
    /// from a cut during a search, and the fields above cannot tell them
    /// apart: both report `cut_at_sync`/`cut_at_score` as `None`, since a
    /// row being subtracted has no candidate ranking attached (#589).
    /// A caller that sees `exhausted` with this below the row count ran
    /// out while cleaning up, not while looking.
    ///
    /// One `subtract_signal_lpf_refine_dt` measured 60-150 ms on the #587
    /// recordings, so this is also the coarsest granularity the budget has
    /// anywhere: between two increments the check cannot fire.
    pub rows_subtracted: u32,
    /// The first coarse search's candidates, carried here because every strategy already
    /// returns this report; the decoder moves it to `SlotResult::candidates` (and clears it
    /// here) before a caller sees either. `None` until a search noted one.
    pub(crate) candidates: Option<u32>,
}

impl BudgetReport {
    /// Note the number of candidates a coarse search handed on. Only the first search of a call
    /// counts: a SIC round's later search runs on audio with decodes already subtracted.
    pub(crate) fn note_candidates(&mut self, n: usize) {
        if self.candidates.is_none() {
            self.candidates = Some(u32::try_from(n).unwrap_or(u32::MAX));
        }
    }
}

/// Decode cost/recall configuration: [`LlrEffort`] plus whether to escalate
/// to OSD when the BP staircase fails.
///
/// `osd` is a **cost choice on every target, embedded included** —
/// correcting what this comment claimed until 2026-08-30 ("host-only",
/// "compiled out of non-`fft-rustfft` builds entirely", "a permanent
/// architectural boundary"). None of that is true of the code: neither
/// `fec::ldpc::osd` nor the OSD block in
/// `process_candidate_basic_impl` carries an FFT-backend `cfg`, and
/// both the FST4 (#306) and FT4 embedded benches have run
/// `DecodeDepth::FULL` on an ESP32-S3 and reported its cost —
/// `ft4-bench` measured 10 987 ms against `EMBEDDED`'s 8 642 ms over 31
/// candidates, i.e. ~2 345 ms of OSD.
///
/// What it buys is worth stating next to what it costs, because the
/// two are close: on 560 FT4 sweep files straddling four channels' 50 %
/// crossings, `FULL` decodes 237 against `EMBEDDED`'s 179
/// (`tests/ft4_llr_ladder_ablation.rs`). On the WSJT-X FT4 golden the
/// two are identical, which is how "OSD buys nothing" got recorded in
/// the first place — the golden's signals are simply strong enough not
/// to need it.
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
    /// Retry a candidate at `i0 ± 1` (FST4 only): `fst4_decode.f90:234-248`
    /// sets `jittermax` to 2 at `ndepth` 2 and 3 and 0 at `ndepth` 1,
    /// independently of whether OSD runs.
    pub jitter: bool,
}

impl DecodeDepth {
    /// ESP32 ship config: cheapest LLR effort, OSD off.
    pub const EMBEDDED: Self = Self {
        llr_effort: LlrEffort::Minimal,
        osd: false,
        jitter: false,
    };
    /// Full LLR effort, no OSD — host "fast" baseline (was `BpAll`).
    pub const BP_ONLY: Self = Self {
        llr_effort: LlrEffort::Full,
        osd: false,
        jitter: false,
    };
    /// Full LLR effort + OSD fallback — host default (was `BpAllOsd`).
    pub const FULL: Self = Self {
        llr_effort: LlrEffort::Full,
        osd: true,
        jitter: true,
    };
}

/// Decode strictness: trades off sensitivity vs false-positive rate.
///
/// `process_candidate_basic` no longer applies `osd_max_errors` to any protocol:
/// FST4 dropped it first (issue #146: WSJT-X's own FST4 decoder has no such gate)
/// and FT4 followed in #456, once its OSD stopped returning a wrong codeword for
/// 22.6 % of noise candidates (`ft4_decode.f90` has no such gate either). The
/// method stays as public API and for the diagnostics that mirror the old
/// ladder. `Normal` (FT4's hardcoded strictness,
/// issue #72) was retuned 2026-07-18 against a `ft4sim` AWGN/CCIR sweep
/// (`docs/notes/FT4_BENCHMARK.md`) — no longer a placeholder copy of the
/// FT8 calibration. `Strict`/`Deep` are unused by any current caller but
/// kept for the API shape; their numbers are the original FT8-copied
/// values, unverified for FT4.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[non_exhaustive]
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
    /// `docs/historical/FT4_BENCHMARK_LOG.md` section 5 for the measurements.
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

    /// Upper bound on `hard_errors` for AP-assisted decode passes.
    ///
    /// `Normal` is `ft8b.f90`'s: `nharderrors.gt.36` is the only bound its AP passes
    /// have (`ft4_decode.f90` has none), so it is 36 whatever the lock. It was 30, or
    /// 25 from 55 locked bits, calibrated against the old AP rung's false accepts
    /// ("REPORT AP at -18 dB: 15 % FP rate with old thresholds 30/36") — a rung that
    /// passed the CRC for 22 % of noise candidates until #459, and the bound was
    /// what kept them out. Measured on the FT8 sweep at 400 trials a cell (12 800
    /// files a condition, same files each way, `docs/historical/FT8_BENCHMARK_LOG.md` section
    /// 15): 36 against 30/25 finds 544 more hits with the right hint (+6.2 %, none
    /// lost, crossings 0.26-0.39 dB), 39 more with no hint, and costs 1 to 4
    /// extra phantoms in 12 800 files, the AP hallucination `ft8b.f90` has too.
    /// `Strict` and `Deep` keep the graded, FT8-copied values.
    ///
    /// Shared by FT8's per-candidate AP loop and the AP rung of this module's
    /// generic ladder (issue #191 type consolidation), so FT4 takes it too.
    pub fn ap_max_errors(self, locked_bits: usize) -> u32 {
        match (self, locked_bits >= 55) {
            (Self::Strict, true) => 20,
            (Self::Strict, false) => 24,
            (Self::Normal, _) => 36,
            (Self::Deep, true) => 30,
            (Self::Deep, false) => 36,
        }
    }

    /// FT8's own flat (not `osd_depth`-tiered) hard-error acceptance
    /// ceiling — shared by the BP staircase and the OSD fallback
    /// (`ft8::decode_block::process_candidates`/`osd_strategy`), which
    /// both apply the same bound WSJT-X does unconditionally on depth
    /// (`ft8b.f90:422`). Unlike [`Self::osd_max_errors`] (depth-tiered, and
    /// applied to no protocol since #456), FT8's real dispatch has no such tiering to port —
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
    /// same way issue #72 calibrated FT4's numbers: the FT8 sweep
    /// (`tests/ft8_sweep.rs`, `MFSK_FT8_SWEEP_STRICTNESS`) drives `DecodeRequest<Ft8>` with each level
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

/// How a row was decoded, in terms an application can read without knowing the decoder's
/// stages ([`RowDetail::method`](crate::decoder::RowDetail::method), #696). `pass` says the
/// same thing as a number that means something else in every mode (FT4 and FST4 even give 5 to
/// both an OSD depth-3 decode and an AP one), so a caller reads this.
///
/// `None` where a mode records no method: WSPR, JT9, JT65, Q65 and FST4W (their `pass` is 0, or
/// FST4W's LLR variant, not a decode method).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum DecodeMethod {
    /// Belief propagation on blind soft bits: no assumption about the message.
    Bp,
    /// The OSD fallback, after belief propagation failed.
    Osd,
    /// A-priori: some bits of the message were assumed. BP or OSD ran inside, and which is not
    /// told apart. `iaptype` is WSJT-X's number for what was assumed (1 `CQ ...`, 2 `MyCall ...`,
    /// 3 `MyCall DxCall ...`, 4 `... RRR`, 5 `... 73`, 6 `... RR73`) when the hypotheses came from
    /// the QSO context (`DecodeParams::qso` and `ap`); `None` for a free-form `ap_hint`, which locks
    /// whatever bits it names, and upstream has no number for that.
    Ap { iaptype: Option<u8> },
    /// FT8's a7 list decode (`ft8_a7.f90`): the message is one of the calls heard before.
    ListA7,
    /// FT8's a8 list decode: the message is one of the calls in the QSO.
    ListA8,
}

impl DecodeMethod {
    /// The method of an a-priori decode from its pass id: the id each `iaptype` carries in
    /// `decoder::frame::pass_of` (1 → 12, 2 → 5, 3 → 8, 4 → 9, 5 → 11, 6 → 10). That holds for
    /// the QSO context's hypotheses (`decoder::frame::qso_hint`) only: a free-form
    /// `extras.ap_hint` uses the same ids for whatever bits it locked (its `call1` need not be
    /// MyCall), so the decoder layer clears `iaptype` for those rows (`frame_decode`'s
    /// `free_hint`). Ids 6 and 7 (`msg::pipeline_ap::ap_passes`: the hint as supplied, `CQ` + its
    /// second call) have no `iaptype`.
    pub(crate) fn ap_from_pass(pass: u8) -> Self {
        let iaptype = match pass {
            12 => Some(1),
            5 => Some(2),
            8 => Some(3),
            9 => Some(4),
            11 => Some(5),
            10 => Some(6),
            _ => None,
        };
        DecodeMethod::Ap { iaptype }
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
#[non_exhaustive]
pub struct DecodeResult {
    /// FEC-decoded information bits; length = `<P::Fec as FecCodec>::K`.
    pub info: Box<[u8]>,
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub hard_errors: u32,
    pub sync_score: f32,
    pub pass: u8,
    /// How it was decoded; `None` where the producing path records none (the subtraction's
    /// synthetic results, a test fixture).
    pub method: Option<DecodeMethod>,
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
    ///
    /// A `&[u8; 77]` rather than a slice since #391: it is always 77 bits,
    /// and [`crate::engine::tx::message_to_tones`] takes it as one.
    pub fn message77(&self) -> &[u8; 77] {
        self.info[..77]
            .try_into()
            .expect("DecodeResult::info holds at least 77 bits")
    }
}
