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
//! **Deliberately does *not* include issue #308's `i0±1` timing-jitter
//! retry**, even though that fix landed in `process_candidate_basic_
//! impl` (host) the same day and is a genuine WSJT-X-fidelity
//! improvement there. Tried it here first — real-hardware measurement
//! found it triples `full`'s total (40.102s → 121.281s) and more than
//! doubles `no8_osd`'s (13.643s → 34.200s), for a recall gain of only a
//! few points at the SNRs checked (`fst4_60_diag_i0_offset_ablation`:
//! AWGN m27 74→82/100, CCIR-moderate m26 18→23/100 — real, but nowhere
//! near proportional to a 2-3× cost). Host has no ~7s deadline to
//! protect, so #308's fix is unconditionally worth it there; embedded's
//! tight budget means the same fix is a bad trade *for this specific
//! constraint*, not a bad fix in general. This is why the two call
//! sites are allowed to diverge instead of one "more faithful" ladder
//! serving both: WSJT-X-fidelity and embedded feasibility are different
//! questions here, and conflating them would have made either the host
//! recall fix contingent on embedded's budget, or the embedded ladder
//! contingent on a cost it can't afford.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex32;

use super::super::engine::llr::{compute_llr_fast, compute_llr_partial, descramble_info, symbol_spectra, sync_quality};
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

/// Decode a full candidate list rung-major instead of depth-first. See
/// the module doc comment for the full rationale. `skip_llrc` mirrors
/// `engine::pipeline`'s `skip_llr_nsym_max` (the `no8_osd` trade-off) —
/// `false` reproduces every candidate's full 4-BP-rung + OSD attempt,
/// `true` drops the `LLR_NSYM_MAX` rung from both BP and OSD's variant
/// list.
///
/// Returns one slot per input candidate, `Some` for the ones that
/// decoded. `DecodeResult::snr_db`/`sync_cv` are not computed here (NaN
/// / 0.0 placeholders) — same convention `process_candidate_precomputed`
/// establishes for its own `skip_snr` flag: this function answers the
/// wall-clock/scheduling question, not the SNR-reporting one. Callers
/// that need real SNR should re-derive it from the returned candidate
/// positions the way `process_candidate_basic` already does.
pub fn decode_rung_major<P>(candidates: &[RungMajorCandidate], skip_llrc: bool) -> Vec<Option<DecodeResult>>
where
    P: Protocol,
    P::Fec: BpPooledFec,
{
    decode_rung_major_timed::<P>(candidates, skip_llrc, false, None).0
}

/// Same as [`decode_rung_major`], plus optional per-candidate,
/// per-stage wall-clock timing — issue #306 item 3 follow-up (a
/// same-candidate-set old-vs-new total came out far larger than
/// dropping one cheap `llrd` stage should plausibly explain; this is
/// the instrumentation to find out *which* stage the gap is actually
/// in, rather than guessing). `clock`, when `Some`, is called
/// immediately before and after each (candidate, stage) unit of work —
/// a plain `fn() -> i64` so both host (`std::time::Instant`-backed) and
/// embedded (`esp_timer_get_time`) callers can supply one without this
/// `no_std` crate depending on either. Returns `(results,
/// per_candidate_stage_us)` where the second element is `None` when
/// `clock` is `None`, else one `[i64; 5]` per input candidate (stage
/// order: llra, llrb, llre, llrc, OSD — `0` for any stage this
/// candidate never reached, e.g. filtered by the `nsync` gate or
/// `skip_llrc`).
pub fn decode_rung_major_timed<P>(
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
    clock: Option<fn() -> i64>,
) -> (Vec<Option<DecodeResult>>, Option<Vec<[i64; 5]>>)
where
    P: Protocol,
    P::Fec: BpPooledFec,
{
    let nsym_mid = P::LLR_NSYM_MID.expect(
        "decode_rung_major is FST4-specific: P::LLR_NSYM_MID must be set (see module doc)",
    ) as usize;
    let nsym_max = P::LLR_NSYM_MAX as usize;
    let ds_rate = 12_000.0 / P::NDOWN as f32;
    let tx_start = P::TX_START_OFFSET_S;
    let (osd_attempt_min, osd_depth3_min) = osd_escalation_gates::<P>();
    let verify_info = Some(<P::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);

    // 5 stages: llra, llrb, llre(mid), llrc(max, unless skip_llrc), OSD.
    // `llrd` intentionally omitted -- see module doc.
    const N_STAGES: usize = 5;

    struct CandState<'a> {
        input: &'a RungMajorCandidate,
        cs: Vec<crate::engine::scalar::Cmplx<f32>>,
        nsync: u32,
        llra: Vec<f32>,
        llrb: Vec<f32>,
        llre: Vec<f32>,
        llrc: Vec<f32>,
        decoded: Option<DecodeResult>,
    }

    // Builds a `DecodeResult` from a raw `FecResult` plus the candidate
    // metadata needed to place it -- a free function, not a closure, so
    // it borrows only what's passed in and never conflicts with the
    // surrounding loop's mutable borrow of `st`.
    fn build_result<P: Protocol>(
        mut r: crate::engine::protocol::FecResult,
        input: &RungMajorCandidate,
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
            dt_sec: (input.i0 as f32) / ds_rate - tx_start,
            hard_errors: r.hard_errors,
            sync_score: input.cand.score,
            pass,
            sync_cv: 0.0,
            snr_db: f32::NAN,
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
            // this whole module exists to schedule better.
            let df_hz = input.refined_freq_hz - input.cand.freq_hz;
            let cd0 = super::super::engine::sync2d::freq_shift_cd0(&input.cd0, df_hz, ds_rate);
            let cs = symbol_spectra::<P>(&cd0, input.i0);
            let nsync = sync_quality::<P>(&cs);
            // Lazily filled per-stage below -- computing all four up
            // front here would defeat the whole point (the eager-vs-
            // lazy distinction `process_candidate_basic_impl`'s own doc
            // comment explains). Each is filled exactly once, the first
            // time its stage runs for this candidate, regardless of
            // which order stages run in overall. `cs` itself is the one
            // thing every stage needs, so it's computed once here and
            // kept, not recomputed per stage.
            CandState {
                input,
                cs,
                nsync,
                llra: Vec::new(),
                llrb: Vec::new(),
                llre: Vec::new(),
                llrc: Vec::new(),
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

    let mut per_stage_us: Vec<[i64; 5]> = vec![[0i64; 5]; candidates.len()];

    for stage in 0..N_STAGES {
        if stage == 3 && skip_llrc {
            continue;
        }
        for (idx, st) in states.iter_mut().enumerate() {
            if st.decoded.is_some() || st.nsync <= SYNC_Q_MIN {
                continue;
            }
            let fec = P::Fec::default();
            let mut bp_scratch = <P::Fec as BpPooledFec>::Scratch::default();
            let t0 = clock.map(|c| c());

            match stage {
                0 => {
                    st.llra = compute_llr_fast::<P, f32>(&st.cs).llra;
                    if let Some(r) = fec.decode_soft_pooled(&st.llra, &bp_opts(0), &mut bp_scratch) {
                        st.decoded = Some(build_result::<P>(r, st.input, ds_rate, tx_start, 0));
                    }
                }
                1 => {
                    st.llrb = compute_llr_partial::<P, f32, f32>(&st.cs, 2);
                    if let Some(r) = fec.decode_soft_pooled(&st.llrb, &bp_opts(0), &mut bp_scratch) {
                        st.decoded = Some(build_result::<P>(r, st.input, ds_rate, tx_start, 1));
                    }
                }
                2 => {
                    st.llre = compute_llr_partial::<P, f32, f32>(&st.cs, nsym_mid);
                    if let Some(r) = fec.decode_soft_pooled(&st.llre, &bp_opts(0), &mut bp_scratch) {
                        st.decoded = Some(build_result::<P>(r, st.input, ds_rate, tx_start, 6));
                    }
                }
                3 => {
                    st.llrc = compute_llr_partial::<P, f32, f32>(&st.cs, nsym_max);
                    if let Some(r) = fec.decode_soft_pooled(&st.llrc, &bp_opts(0), &mut bp_scratch) {
                        st.decoded = Some(build_result::<P>(r, st.input, ds_rate, tx_start, 2));
                    }
                }
                4 => {
                    if skip_osd || st.nsync < osd_attempt_min {
                        continue;
                    }
                    let osd_depth: u32 = if st.nsync >= osd_depth3_min { 3 } else { 2 };
                    let mut variants: Vec<&Vec<f32>> = vec![&st.llra, &st.llrb, &st.llre];
                    if !skip_llrc {
                        variants.push(&st.llrc);
                    }
                    let mut hit: Option<crate::engine::protocol::FecResult> = None;
                    for llr in variants {
                        if let Some(r) = fec.decode_soft_pooled(llr, &bp_opts(osd_depth), &mut bp_scratch) {
                            hit = Some(r);
                            break;
                        }
                    }
                    if let Some(r) = hit {
                        let pass = if skip_llrc { 4 } else { 5 };
                        st.decoded = Some(build_result::<P>(r, st.input, ds_rate, tx_start, pass));
                    }
                }
                _ => unreachable!(),
            }

            if let (Some(clk), Some(t0)) = (clock, t0) {
                per_stage_us[idx][stage] = clk() - t0;
            }
        }
    }

    let results = states.into_iter().map(|st| st.decoded).collect();
    (results, clock.map(|_| per_stage_us))
}
