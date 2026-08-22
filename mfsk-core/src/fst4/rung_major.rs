//! Rung-major candidate scheduling for FST4 (issue #306 item 3, VK3NV).
//!
//! `engine::pipeline::process_candidate_basic_impl` — the shared
//! FT4/FT8/FST4/MSK144 dispatch — processes candidates *depth-first*:
//! for each candidate in turn, try `nsym=1`, then `2`, then
//! `LLR_NSYM_MID`, then `LLR_NSYM_MAX`, then OSD, stopping at the first
//! success; a candidate that never decodes pays for every rung before
//! the loop moves to the next candidate. On a deadline-constrained
//! embedded target, that means a single hard-to-decode candidate early
//! in the list can delay every easy decode after it arbitrarily far —
//! this session's own real-hardware measurement found a worst-case
//! ordering pushes both real decodes on the FST4-60 golden recording to
//! ~30s of a 30.15s total, versus ~7.4s if the candidate order happened
//! to be favourable (`tests/fst4_sweep.rs::fst4_60_diag_rung_major_
//! scheduling`, host-timing projection; confirmed on real CoreS3
//! hardware via `embedded-shared::apps::fst4_bench`'s `rung_major`
//! mode).
//!
//! [`decode_rung_major`] is the real (not bench-only) implementation:
//! for each rung, sweep every still-undecided candidate before moving
//! to the next rung. Total work is unchanged (same LLR/BP/OSD calls,
//! same total sum) — only the *order*, which bounds worst-case
//! time-to-first-decode to one rung's cost regardless of candidate
//! order, instead of depending on where the easy candidates happen to
//! land in the input list.
//!
//! Deliberately **not** folded into `process_candidate_basic_impl` —
//! that function stays exactly as-is, serving FT4/FT8/FST4/MSK144
//! depth-first, unaffected by any of this. This is a separate,
//! FST4-specific entry point; adopting it as the FST4 embedded default
//! (or backporting rung-major scheduling to other protocols) is a
//! product decision for later, not implied by its existence here.
//!
//! **Omits `llrd`** (the normalised-nsym=1 BP variant, tried *last* in
//! production's staircase): a targeted ablation
//! (`tests/fst4_sweep.rs::fst4_60_diag_stage_ablation[_ccir_moderate]`)
//! found it contributes exactly zero additional recall beyond
//! `llra`/`llrb`/`llre`/`llrc` + OSD, on both the real AWGN and
//! CCIR-moderate FST4-60 corpora (100 trials/SNR near crossing,
//! byte-identical hit counts with and without it in every one of 4
//! configurations tested). A stage that never wins shouldn't be
//! scheduled at all, rung-major or not — this isn't a rung-major-
//! specific simplification, it's a genuine 5-stage (not 6-stage) design
//! for this protocol.
//!
//! `skip_llrc`: when `true`, the `LLR_NSYM_MAX` rung (`llrc`, FST4-60's
//! most expensive by far — see `docs/reference/EMBEDDED.md`'s
//! "Sixteenth attempt", `BP_ONLY` measured at 31.023s/41 candidates
//! versus `LLR_NSYM_MAX` alone dominating essentially all of that) is
//! skipped entirely, same shape as `engine::pipeline`'s existing
//! `skip_llr_nsym_max` — this is the `no8_osd` trade-off from earlier
//! in this investigation, folded into the rung-major design as an
//! explicit caller choice rather than a separate code path. Whether to
//! set it is a real sensitivity trade-off, not free — see
//! `docs/reference/EMBEDDED.md`'s "Fourteenth attempt" for the AWGN/
//! CCIR recall cost this carries.
//!
//! `offsets`: issue #308 ported `process_candidate_basic_impl`'s `i0±1`
//! timing-jitter retry (WSJT-X `fst4_decode.f90`'s `ijitter ∈ {0, +1,
//! -1}`) to host — a genuine recall win there, but real-hardware
//! measurement found it triples `full`'s total (40.102s → 121.281s) and
//! more than doubles `no8_osd`'s (13.643s → 34.200s) if ported here
//! unconditionally too, for a recall gain of only a few points at the
//! SNRs checked (`fst4_60_diag_i0_offset_ablation`: AWGN m27 74→82/100,
//! CCIR-moderate m26 18→23/100). Host has no ~7s deadline to protect,
//! so #308 is worth it there unconditionally; embedded is not one fixed
//! answer — a monitoring-style deployment that can tolerate spanning
//! slots has a very different cost/recall trade-off than one with a
//! hard per-slot deadline. Rather than picking one policy, `offsets` is
//! the caller's choice, same shape as `skip_llrc`/`skip_osd` below:
//!
//! | `offsets` | relative host cost | recall (AWGN m27 / CCIR-moderate m26) |
//! |---|---:|---|
//! | `&[0]` | 1.00× | 74/100 / 18/100 |
//! | `&[0, -1]` | 1.84× | 79/100 / 21/100 |
//! | `&[0, 1, -1]` | 2.83× (real hardware: `full` 3.02×, `no8_osd` 2.51×) | 82/100 / 23/100 |
//!
//! **On the scheduling of `offsets` themselves** (issue #310): the
//! offset-major outer loop below is a deliberate choice, not an
//! unexamined default — see `docs/notes/FST4_BENCHMARK.md` §13 for the
//! decision and §9/§12 for the measurements behind it. Short version:
//! folding offsets into a cost-ordered queue would triple the first-rung
//! sweep, which is the ordering-independent time-to-first-decode bound
//! this module exists to provide; offset setup (`symbol_spectra` + bit
//! metrics rebuild) is not free, so the natural unit is offset-setup +
//! a bundle of rungs; and the payoff can't be aimed, since ranking the
//! offsets by sync quality matched exhaustive retry in only 1 of 8
//! measured runs. The intended next step is a deadline check *between*
//! offsets, not a re-ordering within them. Provisional on three open
//! measurements listed in §13.
//!
//! **The escalation phase after the first rung should be depth-first,
//! not rung-major** — measured 2026-08-18, `FST4_BENCHMARK.md` §14's
//! breadth-vs-depth table. Rung-major's value is the latency bound, and
//! that bound is bought entirely by the *first* rung. Past it,
//! continuing breadth-first defers OSD — where most decodes come from —
//! until every candidate has had every cheaper stage, which costs 30
//! decodes in 386 at a realistic ~50 % budget (350 vs 380) and far more
//! at tighter ones (35 vs 166 at 10 %). Candidate ordering also stops
//! mattering under breadth-first: a 10 % budget buys ~98 % of the
//! first-stage sweep, so every ordering visits nearly the same
//! candidates. So the two decisions are coupled — order by `nsync`
//! (free; the first rung computed it for the gate) *and* go depth-first,
//! or neither is worth doing.
//!
//! (`-1` alone consistently outperforms `+1` alone at these SNRs, which
//! is why `&[0, -1]` is the natural two-offset middle ground rather than
//! `&[0, 1]` — see `fst4_60_diag_i0_offset_ablation`'s full 4-way table
//! for the `+1`-alone numbers.) `&[0]` is the deadline-tight default
//! `decode_rung_major`'s 2-arg wrapper uses; nothing here picks `&[0,
//! -1]` or `&[0, 1, -1]` as *the* embedded answer — that's a deployment
//! decision, not a decoder one.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex32;

use super::super::engine::llr::{
    compute_llr_fast, compute_llr_partial, descramble_info, symbol_spectra, sync_quality,
};
use super::super::engine::pipeline::{DecodeResult, osd_escalation_gates};
use super::super::engine::protocol::{BpPooledFec, FecOpts, MessageCodec, Protocol};
use super::super::engine::sync::SyncCandidate;

/// One candidate ready for [`decode_rung_major`] — coarse sync position
/// plus its already-refined, already-RMS-normalised baseband (same
/// shape `process_candidate_precomputed`'s `precomputed_refine` takes,
/// minus the fields this module doesn't need: score/dt were already
/// used to pick `i0`/`cd0` upstream).
pub struct RungMajorCandidate {
    /// Original coarse-sync position (pre-refine) — `cand.freq_hz` is
    /// what `refined_freq_hz` gets differenced against below, matching
    /// `process_candidate_basic_impl`'s own `try_position` closure
    /// exactly (`df_hz = freq_hz - cand.freq_hz`). **Not** the same
    /// value as `refined_freq_hz` in general — an earlier version of
    /// this function conflated the two (stored the refined frequency in
    /// both places), which zeroed out `df_hz` unconditionally and
    /// silently skipped the frequency correction `cd0` still needs.
    /// That bug artificially failed the `nsync` gate for exactly the
    /// marginal candidates whose coarse and refined frequencies differ
    /// most — the ones this module's whole timing story is about — so
    /// a real-hardware total that looked "too good" (candidates that
    /// should have reached the expensive rungs were silently dropped
    /// instead) is what first surfaced it.
    pub cand: SyncCandidate,
    /// Downsampled, RMS-normalised baseband **at `cand.freq_hz`** — not
    /// yet frequency-shifted to `refined_freq_hz`. Same convention
    /// `process_candidate_basic_impl`'s `precomputed_refine` uses.
    pub cd0: Vec<Complex32>,
    /// Refined frequency from `refine_candidate_position` — together
    /// with `cand.freq_hz` this gives `df_hz`, applied via
    /// `freq_shift_cd0` before any decode work happens.
    pub refined_freq_hz: f32,
    pub i0: i32,
}

/// Which schedule [`decode_scheduled`] walks the ladder with.
///
/// Both visit exactly the same (candidate, offset, sub-stage) units and
/// do the same total work — they differ only in order, and therefore
/// only under a budget.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Schedule {
    /// Every rung swept across every still-undecided candidate before
    /// any candidate descends. What this module shipped with, and what
    /// buys the ordering-independent time-to-first-decode bound.
    RungMajor,
    /// **Phase A** — the first rung (`llra` at `offsets[0]`) swept across
    /// every candidate, exactly as `RungMajor` does. This *is* the
    /// latency invariant, and it is bought entirely here.
    ///
    /// **Phase B** — the rest of `offsets[0]`'s ladder, depth-first, with
    /// candidates ordered by the `nsync` Phase A already computed for the
    /// gate.
    ///
    /// **Phase C** — the remaining offsets, depth-first, same order.
    ///
    /// Measured 2026-08-18 (`FST4_BENCHMARK.md` §14): past the first
    /// rung, continuing breadth-first defers OSD — where most decodes
    /// come from — until every candidate has had every cheaper stage,
    /// costing 30 decodes in 386 at a realistic ~50 % budget and 131 in
    /// 386 at 10 %. Candidate ordering is also worthless under
    /// breadth-first (a 10 % budget buys ~98 % of the first-stage sweep,
    /// so every order visits nearly the same candidates) and valuable
    /// under depth-first, so the two changes are a package.
    PhaseSplit,
}

/// Decode a full candidate list rung-major instead of depth-first,
/// trying only the refined `i0` position (`offsets = &[0]`) — the
/// deadline-tight default. See the module doc comment for the full
/// rationale, and [`decode_rung_major_timed`] if a caller wants
/// `offsets` or `skip_osd` control too. `skip_llrc` mirrors
/// `engine::pipeline`'s `skip_llr_nsym_max` (the `no8_osd` trade-off) —
/// `false` reproduces every candidate's full 4-BP-rung + OSD attempt,
/// `true` drops the `LLR_NSYM_MAX` rung from both BP and OSD's variant
/// list.
///
/// Returns one slot per input candidate, `Some` for the ones that
/// decoded. `DecodeResult::sync_cv` is left at `0.0` — this function
/// answers the wall-clock/scheduling question, and no caller of it has
/// wanted that field. `snr_db` *is* filled in, from the refined
/// baseband alone (`fst4::baseline::fst4_ddc_snr_db`); it used to be
/// `NaN` here only because the estimator that existed needed a
/// whole-slot FFT no caller of this function has.
pub fn decode_rung_major<P>(
    candidates: &[RungMajorCandidate],
    skip_llrc: bool,
) -> Vec<Option<DecodeResult>>
where
    P: Protocol,
    P::Fec: BpPooledFec,
{
    // 12 kHz: `candidates` here are always the canonical raw-ingest
    // downsample (see `decode_scheduled`'s `sample_rate_hz` doc) — this
    // convenience wrapper has no caller that varies it.
    decode_rung_major_timed::<P>(candidates, skip_llrc, false, &[0], None, 12_000.0).0
}

/// Same as [`decode_rung_major`], plus `offsets` (which `i0` timing
/// positions to try — see the module doc's table for the cost/recall
/// trade-off of `&[0]` / `&[0, -1]` / `&[0, 1, -1]`; must be non-empty)
/// and optional per-candidate, per-stage wall-clock timing — issue #306
/// item 3 follow-up (a same-candidate-set old-vs-new total came out far
/// larger than dropping one cheap `llrd` stage should plausibly
/// explain; this is the instrumentation to find out *which* stage the
/// gap is actually in, rather than guessing). `clock`, when `Some`, is
/// called immediately before and after each (candidate, offset,
/// sub-stage) unit of work — a plain `fn() -> i64` so both host
/// (`std::time::Instant`-backed) and embedded (`esp_timer_get_time`)
/// callers can supply one without this `no_std` crate depending on
/// either. Returns `(results, per_candidate_stage_us)` where the second
/// element is `None` when `clock` is `None`, else one
/// `Vec<i64>` of length `offsets.len() * 5` per input candidate,
/// offset-major (stage `s`: `s / 5` selects the offset in `offsets`,
/// `s % 5` selects the sub-stage in llra/llrb/llre/llrc/OSD order) — `0`
/// for any stage this candidate never reached, e.g. filtered by the
/// `nsync` gate or `skip_llrc`.
///
/// `sample_rate_hz` is the input rate `candidates`' `cd0` was
/// downsampled *from* (divide by `P::NDOWN` for `cd0`'s own rate) —
/// `12_000.0` for every caller today, matching `DownsampleCfg::input_rate`
/// (issue #323). Not yet reachable from a DDC-fed front end (#309);
/// threaded through now so wiring one up later is a call-site change
/// here, not a rediscovery of this function's own `ds_rate` hardcode.
pub fn decode_rung_major_timed<P>(
    candidates: &[RungMajorCandidate],
    skip_llrc: bool,
    skip_osd: bool,
    offsets: &[i32],
    clock: Option<fn() -> i64>,
    sample_rate_hz: f32,
) -> (Vec<Option<DecodeResult>>, Option<Vec<Vec<i64>>>)
where
    P: Protocol,
    P::Fec: BpPooledFec,
{
    decode_scheduled::<P>(
        candidates,
        skip_llrc,
        skip_osd,
        offsets,
        clock,
        Schedule::RungMajor,
        None,
        sample_rate_hz,
    )
}

/// [`Schedule::PhaseSplit`] — bounded first rung, then depth-first
/// escalation ordered by `nsync`, with an optional deadline gate.
///
/// `budget_ok`, when `Some`, is polled before every stage in Phase B and
/// Phase C and the schedule stops the moment it returns `false`. **Phase
/// A is never gated**: sweeping the cheapest rung across every candidate
/// is the latency invariant this module exists for, and making it
/// interruptible would give the guarantee away. With `budget_ok = None`
/// this runs the whole ladder and returns exactly the same decodes as
/// [`decode_rung_major_timed`] — only the order, and therefore only the
/// timing breakdown, differs.
///
/// Same `(results, per_candidate_stage_us)` shape and the same
/// offset-major stage indexing as [`decode_rung_major_timed`], so the
/// two are directly comparable on one corpus.
///
/// `sample_rate_hz`: see [`decode_rung_major_timed`]'s doc.
pub fn decode_phase_split_timed<P>(
    candidates: &[RungMajorCandidate],
    skip_llrc: bool,
    skip_osd: bool,
    offsets: &[i32],
    clock: Option<fn() -> i64>,
    budget_ok: Option<fn() -> bool>,
    sample_rate_hz: f32,
) -> (Vec<Option<DecodeResult>>, Option<Vec<Vec<i64>>>)
where
    P: Protocol,
    P::Fec: BpPooledFec,
{
    decode_scheduled::<P>(
        candidates,
        skip_llrc,
        skip_osd,
        offsets,
        clock,
        Schedule::PhaseSplit,
        budget_ok,
        sample_rate_hz,
    )
}

/// Shared implementation behind [`decode_rung_major_timed`] and
/// [`decode_phase_split_timed`]; `schedule` picks the walk order.
fn decode_scheduled<P>(
    candidates: &[RungMajorCandidate],
    skip_llrc: bool,
    // Drops the OSD stage entirely (BP-only) -- VK3NV's issue #306 item
    // 1 follow-up: the `full`-config BP/OSD split doesn't necessarily
    // carry over to `no8_osd` (dropping `llrc` changes how often BP
    // fails and OSD is even reached), so a trustworthy `no8_osd` split
    // needs its own direct BP-only measurement, not a subtraction from
    // `full`'s numbers. `false` for every other caller (behaves exactly
    // as before).
    skip_osd: bool,
    // Which `i0` offsets to try, offset-major (see the module doc's
    // cost/recall table) -- deployment choice, not a fixed policy.
    offsets: &[i32],
    clock: Option<fn() -> i64>,
    schedule: Schedule,
    budget_ok: Option<fn() -> bool>,
    // Input rate `candidates`' `cd0` was downsampled from (issue #323)
    // — was an independent `12_000.0 / P::NDOWN` hardcode, duplicating
    // `DownsampleCfg::input_rate` without reading it (there's no `cfg`
    // in scope here; `candidates` arrive pre-downsampled).
    sample_rate_hz: f32,
) -> (Vec<Option<DecodeResult>>, Option<Vec<Vec<i64>>>)
where
    P: Protocol,
    P::Fec: BpPooledFec,
{
    assert!(
        !offsets.is_empty(),
        "decode_rung_major_timed: offsets must be non-empty"
    );

    let nsym_mid = P::LLR_NSYM_MID
        .expect("decode_rung_major is FST4-specific: P::LLR_NSYM_MID must be set (see module doc)")
        as usize;
    let nsym_max = P::LLR_NSYM_MAX as usize;
    let ds_rate = sample_rate_hz / P::NDOWN as f32;
    let tx_start = P::TX_START_OFFSET_S;
    let (osd_attempt_min, osd_depth3_min) = osd_escalation_gates::<P>();
    let verify_info = Some(<P::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

    // 5 sub-stages per offset: llra, llrb, llre(mid), llrc(max, unless
    // skip_llrc), OSD. `llrd` intentionally omitted -- see module doc.
    const N_SUBSTAGES: usize = 5;
    let n_stages = offsets.len() * N_SUBSTAGES;

    #[derive(Default)]
    struct OffsetState {
        computed: bool,
        cs: Vec<crate::engine::scalar::Cmplx<f32>>,
        nsync: u32,
        llra: Vec<f32>,
        llrb: Vec<f32>,
        llre: Vec<f32>,
        llrc: Vec<f32>,
    }

    struct CandState<'a> {
        input: &'a RungMajorCandidate,
        cd0: Vec<Complex32>,
        offsets: Vec<OffsetState>,
        decoded: Option<DecodeResult>,
    }

    // Builds a `DecodeResult` from a raw `FecResult` plus the candidate
    // metadata needed to place it -- a free function, not a closure, so
    // it borrows only what's passed in and never conflicts with the
    // surrounding loop's mutable borrow of `st`.
    fn build_result<P: Protocol>(
        mut r: crate::engine::protocol::FecResult,
        input: &RungMajorCandidate,
        i0: i32,
        ds_rate: f32,
        tx_start: f32,
        pass: u8,
    ) -> DecodeResult {
        descramble_info::<P>(&mut r.info);
        DecodeResult {
            info: r.info.into_boxed_slice(),
            // Refined frequency, matching `process_candidate_basic_impl`'s
            // own `refined.freq_hz` -- *not* `input.cand.freq_hz` (the
            // pre-refine coarse position).
            freq_hz: input.refined_freq_hz,
            dt_sec: (i0 as f32) / ds_rate - tx_start,
            hard_errors: r.hard_errors,
            sync_score: input.cand.score,
            pass,
            sync_cv: 0.0,
            // Measurable here after all, and cheap: the refined
            // baseband this candidate already holds is 111 Hz wide
            // where the signal occupies ~12, so the noise can be read
            // straight out of the rest of it — see
            // `fst4::baseline::fst4_ddc_snr_db`. Was `NaN` because the
            // only estimator that existed needed a whole-slot FFT this
            // path never has.
            snr_db: crate::fst4::baseline::fst4_ddc_snr_db::<P>(&input.cd0, ds_rate)
                .unwrap_or(f32::NAN),
        }
    }

    // Matches `fst4::decode`'s own (private) `SYNC_Q_MIN` -- WSJT-X's
    // pre-ladder nsync gate (issue #197). Redeclared here for the same
    // reason `fst4_bench` redeclares it: not `pub` from `fst4::decode`,
    // deliberately (see that const's own doc comment).
    const SYNC_Q_MIN: u32 = 16;

    let mut states: Vec<CandState> = candidates
        .iter()
        .map(|input| {
            // Matches `process_candidate_basic_impl`'s `try_position`
            // closure exactly: `input.cd0` is baseband at the *coarse*
            // `input.cand.freq_hz`, not yet corrected to the refined
            // position -- `df_hz` applies that correction before
            // `symbol_spectra` ever runs. Skipping this (an earlier
            // version of this function did) silently mis-syncs every
            // candidate whose coarse and refined frequency estimates
            // differ, which in practice is exactly the marginal ones
            // this whole module exists to schedule better. The
            // frequency shift doesn't depend on the timing offset, so
            // it's still computed once per candidate, not per offset.
            let df_hz = input.refined_freq_hz - input.cand.freq_hz;
            let cd0 = super::super::engine::sync2d::freq_shift_cd0(&input.cd0, df_hz, ds_rate);
            CandState {
                input,
                cd0,
                offsets: (0..offsets.len()).map(|_| OffsetState::default()).collect(),
                decoded: None,
            }
        })
        .collect();

    let bp_opts = |osd_depth: u32| -> FecOpts<'static> {
        FecOpts {
            bp_max_iter: 30,
            osd_depth,
            ap_mask: None,
            verify_info,
            ..FecOpts::default()
        }
    };

    let mut per_stage_us: Vec<Vec<i64>> = vec![vec![0i64; n_stages]; candidates.len()];

    // One (candidate, offset, sub-stage) unit of work. Extracted so the
    // two schedules differ only in the order they call it -- the total
    // set of calls, and therefore the total work, is identical.
    let run_stage = |st: &mut CandState,
                     idx: usize,
                     offset_idx: usize,
                     substage: usize,
                     per_stage: &mut [Vec<i64>]| {
        if substage == 3 && skip_llrc {
            return;
        }
        if st.decoded.is_some() {
            return;
        }
        let stage = offset_idx * N_SUBSTAGES + substage;
        let ioffset = offsets[offset_idx];
        let t0 = clock.map(|c| c());

        let off = &mut st.offsets[offset_idx];
        if !off.computed {
            let i0 = st.input.i0 + ioffset;
            off.cs = symbol_spectra::<P>(&st.cd0, i0);
            off.nsync = sync_quality::<P>(&off.cs);
            off.computed = true;
        }
        if off.nsync <= SYNC_Q_MIN {
            if let (Some(clk), Some(t0)) = (clock, t0) {
                per_stage[idx][stage] = clk() - t0;
            }
            return;
        }

        let fec = P::Fec::default();
        let mut bp_scratch = <P::Fec as BpPooledFec>::Scratch::default();
        let i0 = st.input.i0 + ioffset;

        match substage {
            0 => {
                off.llra = compute_llr_fast::<P, f32>(&off.cs).llra;
                if let Some(r) = fec.decode_soft_pooled(&off.llra, &bp_opts(0), &mut bp_scratch) {
                    st.decoded = Some(build_result::<P>(r, st.input, i0, ds_rate, tx_start, 0));
                }
            }
            1 => {
                off.llrb = compute_llr_partial::<P, f32, f32>(&off.cs, 2);
                if let Some(r) = fec.decode_soft_pooled(&off.llrb, &bp_opts(0), &mut bp_scratch) {
                    st.decoded = Some(build_result::<P>(r, st.input, i0, ds_rate, tx_start, 1));
                }
            }
            2 => {
                off.llre = compute_llr_partial::<P, f32, f32>(&off.cs, nsym_mid);
                if let Some(r) = fec.decode_soft_pooled(&off.llre, &bp_opts(0), &mut bp_scratch) {
                    st.decoded = Some(build_result::<P>(r, st.input, i0, ds_rate, tx_start, 6));
                }
            }
            3 => {
                off.llrc = compute_llr_partial::<P, f32, f32>(&off.cs, nsym_max);
                if let Some(r) = fec.decode_soft_pooled(&off.llrc, &bp_opts(0), &mut bp_scratch) {
                    st.decoded = Some(build_result::<P>(r, st.input, i0, ds_rate, tx_start, 2));
                }
            }
            4 => {
                if skip_osd || off.nsync < osd_attempt_min {
                    if let (Some(clk), Some(t0)) = (clock, t0) {
                        per_stage[idx][stage] = clk() - t0;
                    }
                    return;
                }
                let osd_depth: u32 = if off.nsync >= osd_depth3_min { 3 } else { 2 };
                let mut variants: Vec<&Vec<f32>> = vec![&off.llra, &off.llrb, &off.llre];
                if !skip_llrc {
                    variants.push(&off.llrc);
                }
                let mut hit: Option<crate::engine::protocol::FecResult> = None;
                for llr in variants {
                    if let Some(r) =
                        fec.decode_soft_pooled(llr, &bp_opts(osd_depth), &mut bp_scratch)
                    {
                        hit = Some(r);
                        break;
                    }
                }
                if let Some(r) = hit {
                    let pass = if skip_llrc { 4 } else { 5 };
                    st.decoded = Some(build_result::<P>(r, st.input, i0, ds_rate, tx_start, pass));
                }
            }
            _ => unreachable!(),
        }

        if let (Some(clk), Some(t0)) = (clock, t0) {
            per_stage[idx][stage] = clk() - t0;
        }
    };

    match schedule {
        Schedule::RungMajor => {
            for stage in 0..n_stages {
                let offset_idx = stage / N_SUBSTAGES;
                let substage = stage % N_SUBSTAGES;
                for (idx, st) in states.iter_mut().enumerate() {
                    run_stage(st, idx, offset_idx, substage, &mut per_stage_us);
                }
            }
        }
        Schedule::PhaseSplit => {
            // Phase A -- the latency invariant. Cheapest rung across
            // every candidate, never budget-gated.
            for (idx, st) in states.iter_mut().enumerate() {
                run_stage(st, idx, 0, 0, &mut per_stage_us);
            }

            // `nsync` is now populated for offset 0 on every candidate
            // (Phase A computed it to run the gate), so ordering by it
            // costs one sort of the candidate list and nothing else.
            let mut order: Vec<usize> = (0..states.len()).collect();
            order.sort_by(|&a, &b| states[b].offsets[0].nsync.cmp(&states[a].offsets[0].nsync));

            // Phase B -- rest of offsets[0]'s ladder, depth-first.
            // Phase C -- the remaining offsets, same order.
            'budget: for offset_idx in 0..offsets.len() {
                for &idx in &order {
                    for substage in 0..N_SUBSTAGES {
                        if offset_idx == 0 && substage == 0 {
                            continue; // Phase A already ran it
                        }
                        if budget_ok.is_some_and(|ok| !ok()) {
                            break 'budget;
                        }
                        let st = &mut states[idx];
                        run_stage(st, idx, offset_idx, substage, &mut per_stage_us);
                        if st.decoded.is_some() {
                            break;
                        }
                    }
                }
            }
        }
    }

    let results = states.into_iter().map(|st| st.decoded).collect();
    (results, clock.map(|_| per_stage_us))
}
