//! OSD fallback strategy — Step 3 of the per-candidate decode
//! staircase.
//!
//! Runs only when Steps 1–2 (BP staircase) fail and the caller asks
//! for `BpAllOsd` depth at sufficient `sync_quality`. Computes a
//! fresh f32 LLR bundle for the candidate (OSD operates on f32
//! regardless of the embedded fixed-point `LlrT`), and for each of
//! the four LLR variants (a/b/c/d, matching `ft8b.f90`'s `ipass=1..4`
//! `llra/b/c/d`), tries the WSJT-X-faithful OSD entry
//! ([`osd_decode_npre1`] for low-`q` candidates, [`osd_decode_npre1_npre2`]
//! for `q >= Q_NDEEP3_THRESHOLD`) seeded with `bp_llr_zsum(llr, 1)`
//! then `bp_llr_zsum(llr, 2)` — mirroring `decode174_91.f90`'s own
//! `do i=1,nosd` loop over its `zsave(:,i)` snapshots — and applies
//! the `nharderrors > 36` cycle gate to weed out high-error CRC-luck
//! codewords.
//!
//! ε.6 of the `docs/CLEANUP_2026_05.md` `decode_block` split. As of
//! issue **#63** this module hosts the WSJT-X-faithful OSD dispatch
//! — the q-conditional split mirrors `osd174_91.f90`'s ndeep=2/3
//! dispatch table, replacing the previous mfsk-core-specific
//! brute-force ndeep=2/3 split (`osd_decode` / `osd_decode_deep`). As
//! of issue **#182** the OSD *input* is also WSJT-X-faithful: earlier
//! versions fed `osd_decode_npre1`/`_npre2` the raw channel LLR
//! directly, which is not what real WSJT-X does for FT8's blind
//! `ndepth=3` dispatch — that always sets `maxosd=2`
//! (`ft8b.f90:434-441`), and `decode174_91.f90` never calls
//! `osd174_91` on the raw channel LLR when `maxosd>0` (that's a
//! separate `maxosd=0` branch FT8's blind dispatch never takes). It
//! always calls with `zsave(:,i)`, the running BP variable-node soft-
//! estimate sum through the first `i` BP iterations.
//!
//! Host-only (`fft-rustfft`): `DecodeDepth::osd` is a no-op on
//! embedded builds (see its doc comment in `ft8::decode`), so this
//! whole module is compiled out of non-`fft-rustfft` targets entirely
//! rather than merely unreachable at runtime.
//!
//! **FT4/FST4 analog**: this module is FT8's bespoke OSD-fallback
//! dispatch, reached by bypassing [`crate::engine::FecCodec`] entirely
//! (same root cause as issue #198). FT4/FST4 get their OSD escalation
//! through [`crate::engine::pipeline::osd_escalation_gates`] instead —
//! an independent implementation, independently calibrated. Review
//! both when tuning either (issue #192).

#![cfg(feature = "fft-rustfft")]

use super::super::decode::{DecodeDepth, DecodeStrictness};
use crate::engine::scalar::Cmplx;
use crate::fec::ldpc::LDPC_N;
use crate::fec::ldpc::bp::{BpResult, BpScratch, bp_llr_zsum_with_scratch};
use crate::fec::ldpc::osd::{OsdResult, osd_decode_npre1, osd_decode_npre1_npre2};
use crate::fec::ldpc::params::Ldpc174_91Params;

/// `sync_quality` threshold for dispatching to the heavier WSJT-X
/// ndeep=3 entry ([`osd_decode_npre1_npre2`]) instead of ndeep=2
/// ([`osd_decode_npre1`]). Mirrors the pre-#63 dispatch's
/// `q >= 18` split between `osd_decode_deep(_, 3, _)` and
/// `osd_decode(_)` (ndeep=2), now with WSJT-X-faithful internals on
/// both sides.
const Q_NDEEP3_THRESHOLD: u32 = 18;

// OSD `nharderrors` ceiling — now [`DecodeStrictness::ft8_nharderrors_max`]
// (issue #221, strictness-wiring follow-up to #220), `Normal` (default) still
// matching WSJT-X's universal `WSJTX_NHARDERRORS_MAX = 36`
// (`ft8b.f90:422`) exactly, same as the BP variants in
// `process_one_candidate_inner`.
//
// History (issue #72 follow-up, 2026-07-18): `Normal`'s 36 was 22
// pre-issue-#72, a deliberate mfsk-core-specific deviation, now
// retracted. The gate had been tightened to 22 specifically to filter 3
// candidates on `qso3_busy.wav` — `N1API F2VX 73` (e=30), `N1API HA6FQ
// -23` (e=25), `CQ EA2BFM IN83` (e=31) — judged phantoms (CRC-14-luck
// false accepts) because JTDX's own 18-entry recall list wasn't
// independently verified for those specific entries (see issue #150). A
// CCIR-fading sensitivity investigation (`docs/notes/FT8_BENCHMARK.md`)
// using `ft8sim`-synthesized WAVs with a *known* golden message found
// the 22 ceiling silently discarding genuine golden decodes under heavy
// fading — traced multiple independent LLR variants converging on the
// exact transmitted text at `hard_errors` in the high 20s/low 30s.
// Loosening back to WSJT-X's 36 recovered those, and as a direct side
// effect also recovered the same 3 `qso3_busy.wav` candidates at their
// *exact* JTDX-claimed text — independent corroboration between two
// separate decoders on a CRC-14-protected message is strong evidence
// against coincidence, resolving them as real rather than phantom. Full
// regression suite (host `ft8_qso3_apoff_recall`,
// `ft8_decode_block_real_qso`, embedded `decode_block` paths) showed
// zero change from this widening — nothing was relying on the tightened
// gate to stay green. That retracted `22` is exactly
// `DecodeStrictness::Strict`'s value now — real prior art, not a fresh
// guess, for callers who explicitly want it back.

/// Pass-ID range for the `zsave(:,1)`-equivalent (BP-refined, 1
/// iteration) OSD attempts, `14..=17` for llr-variants a/b/c/d.
/// Exposed so `process_one_candidate_inner` can size its
/// pass-tracking buffers without re-declaring magic numbers.
///
/// Was the direct-channel-LLR OSD pass-ID range pre-issue-#182; that
/// loop is gone now (see [`try_fallback`]'s doc comment for why), so
/// this range was free to reclaim.
pub(super) const PASS_ID_OSD_ZSAVE1_A: u8 = 14;

/// Pass-ID range for the `zsave(:,2)`-equivalent (BP-refined, 2
/// iterations) OSD attempts — issue #182. `19..=22` for llr-variants
/// a/b/c/d. (18 was the old auto-AP pass ID, removed in the 0.8.0
/// `DecodeDepth` redesign — currently unused, left as a gap rather
/// than renumbered to avoid an unnecessary diff.)
pub(super) const PASS_ID_OSD_ZSAVE2_A: u8 = 19;

/// Try the OSD staircase for a candidate whose BP staircase didn't
/// converge. Returns `Some((bp_result, pass_id))` on the first
/// accepted CRC-pass codeword, `None` otherwise.
///
/// Gates internally on `depth.osd` and `q > 6` so the caller can
/// invoke unconditionally — keeps the per-candidate staircase in
/// `process_one_candidate_inner` straightforward (`accepted = accepted
/// .or_else(|| osd_strategy::try_fallback(...))`).
///
/// **`q > 6`, not `q >= 12`: WSJT-X-faithfulness fix (issue #180
/// follow-up).** The `q >= 12` gate was a deliberate mfsk-core-specific
/// deviation with no counterpart in `ft8b.f90`, which only bails
/// (`if(nsync .le. 6) ... return`) before attempting *any* decode —
/// once a candidate clears that bar, WSJT-X always attempts its full
/// staircase (BP-equivalent and OSD together), regardless of exactly
/// how far above 6 `nsync` is. Root-caused directly: `K1BZM DK8NE -10`
/// (-19 dB, `qso3_busy.wav`) sits at `q=11` on mfsk-core's own SIC
/// residual — verified to be an *exact* per-block match to real jt9's
/// own residual at the same coordinates (`is1=1 is2=7 is3=3`,
/// `nsync=11`, both sides, confirmed via a locally-instrumented jt9
/// rebuild) — yet never reached this function at all under the old
/// `q >= 12` gate. Manually running the OSD dispatch below anyway (llr
/// variant `d`, `osd_decode_npre1_npre2`) decodes it outright at
/// `hard_errors=22`, comfortably under `Normal`'s ceiling (see
/// [`DecodeStrictness::ft8_nharderrors_max`]). The gap was never a
/// sync/LLR/BP/OSD sensitivity problem — mfsk-core's own OSD
/// implementation was already capable of the decode the whole time;
/// this one-line gate was refusing to even try.
pub(super) fn try_fallback(
    cs_scratch: &[[Cmplx<f32>; 8]; 79],
    precomputed_llr: Option<&super::super::llr::LlrSet<f32>>,
    depth: DecodeDepth,
    q: u32,
    strictness: DecodeStrictness,
) -> Option<(BpResult, u8)> {
    if !depth.osd || q <= 6 {
        return None;
    }

    // OSD operates on `&[f32]` directly — independent of the embedded
    // `LlrT` choice. The caller (`process_one_candidate_inner`)
    // may pass a pre-computed LLR via `precomputed_llr` so the same
    // bundle gets reused for both OSD here and the AP loop after —
    // saves one full `compute_llr` (the nsym=3 LLR is the expensive
    // one) when both paths fire on the same candidate (Gemini PR #81
    // review). If `precomputed_llr` is `None` we compute locally
    // (embedded path with no AP, or callers that haven't been
    // updated).
    let owned_llr_storage;
    let llr_full_f32: &super::super::llr::LlrSet<f32> = match precomputed_llr {
        Some(llr) => llr,
        None => {
            owned_llr_storage = super::super::llr::compute_llr(cs_scratch);
            &owned_llr_storage
        }
    };

    // WSJT-X-faithful OSD dispatch (issue #63). Mirrors WSJT-X's
    // ndeep=2/3 split: ndeep=2 (= nord=1 + npre1=1, ~165 patterns
    // post-gate) for the default `q < 18` candidates; ndeep=3
    // (= ndeep=2 + npre2 weight-3 anchored pairs via a ntau=14
    // hash table) for cleaner candidates that justify the extra
    // 64 KB hash-table build.
    //
    // Both entries carry implicit `check_crc14` verifiers (mirror
    // WSJT-X's `nbadcrc` gate inside `osd174_91`), so no
    // `Some(check_crc14)` argument.
    let dispatch = |llr: &[f32; LDPC_N]| -> Option<OsdResult> {
        let osd = if q >= Q_NDEEP3_THRESHOLD {
            osd_decode_npre1_npre2(llr)
        } else {
            osd_decode_npre1(llr)
        };
        // WSJT-X-faithful ceiling (Normal) — see
        // `DecodeStrictness::ft8_nharderrors_max`'s docstring.
        osd.filter(|o| o.hard_errors <= strictness.ft8_nharderrors_max())
    };
    // Reuse `osd.codeword` instead of allocating a fresh zero vec —
    // `OsdResult` already carries the actual decoded codeword bits,
    // which the previous `vec![0; N]` dropped on the floor (Gemini PR
    // #86 review).
    let to_bp = |osd: OsdResult| BpResult {
        message77: osd.message77,
        info: osd.info,
        codeword: osd.codeword,
        hard_errors: osd.hard_errors,
        iterations: 0,
    };

    // WSJT-X-faithful BP-refined-LLR seed (issue #182), replacing what
    // used to be a direct-channel-LLR OSD loop here. `decode174_91.f90`
    // never calls `osd174_91` on the raw channel LLR when `maxosd>0` —
    // and FT8's blind `ndepth=3` dispatch always sets `maxosd=2`
    // (`ft8b.f90:434-441`), so the raw-channel-LLR OSD path
    // (`maxosd=0`, a *different* WSJT-X depth setting FT8's blind
    // dispatch never takes) was never actually what real `jt9 -d3`
    // does here. It always calls `osd174_91` with `zsave(:,i)`, the
    // running sum of the BP variable-node soft estimate `zn` across
    // the first `i` BP iterations — trying `i=1` then `i=2`
    // (`decode174_91.f90:52-64,137-148`) *before* moving to the next
    // `ipass`/llr variant (`ft8b.f90:294-298`, `do ipass=1,4`). Nesting
    // mirrored exactly here: outer loop over the 4 llr variants, inner
    // loop over the 2 `zsave` snapshots — not the other way around, so
    // the search order (and which candidate gets returned first when
    // more than one would eventually succeed) matches WSJT-X's own.
    //
    // Root-caused directly: `K1BZM DK8NE -10` (-19 dB, `qso3_busy.wav`)
    // never decoded via the old direct-channel-LLR loop at any OSD
    // depth up to and including a brute-force order-2 exhaustive
    // search (no gate) — the true codeword was not reachable from a
    // channel-LLR-selected MRB basis at all. Feeding
    // `bp_llr_zsum(llrd, 2)` into the *same*, unmodified
    // `osd_decode_npre1` decodes it outright at `hard_errors=17`
    // (better than jt9's own real `hard_errors=18` on this exact
    // candidate). The mechanism (`bp_llr_zsum`) already existed and is
    // wired for FST4-120 (`Ldpc240_101`, issue #146) but was never
    // ported to FT8 before now.
    //
    // Pass-ID space (post-0.6.1): BP variants 0..3, AP iaptypes 5..12
    // (mirroring WSJT-X ipass 5..12), host OSD-Deep 13, embedded OSD
    // zsave(1) a/b/c/d 14..17, auto-AP 18, zsave(2) a/b/c/d 19..22.
    //
    // `zsum_scratch` is local to this call (not caller-threaded like
    // `process_one_candidate_inner`'s `bp_scratch`): `try_fallback`
    // runs once per candidate, and the loop below already calls
    // `bp_llr_zsum_with_scratch` 8× (4 llr variants × 2 `n_iter`) per
    // call, so one scratch here already captures the whole win —
    // threading it in from the caller would only additionally pool
    // across candidates, which a real-WAV before/after measurement
    // (perf review, `MFSK_BP_KIND=nms` vs default on
    // `bench_qso3_busy_timing.rs`) found made no measurable wall-clock
    // difference for FT8's BP stage, so the extra plumbing through
    // `process_one_candidate_inner` and its five callers isn't
    // justified by anything measured.
    let mut zsum_scratch = BpScratch::<Ldpc174_91Params, f32>::new();
    for (idx, llr) in [
        &llr_full_f32.llra,
        &llr_full_f32.llrb,
        &llr_full_f32.llrc,
        &llr_full_f32.llrd,
    ]
    .into_iter()
    .enumerate()
    {
        for (n_iter, base) in [(1u32, PASS_ID_OSD_ZSAVE1_A), (2u32, PASS_ID_OSD_ZSAVE2_A)] {
            let zsum_slice =
                bp_llr_zsum_with_scratch::<Ldpc174_91Params>(&mut zsum_scratch, llr, n_iter);
            let mut zsum = [0f32; LDPC_N];
            zsum.copy_from_slice(zsum_slice);
            if let Some(osd) = dispatch(&zsum) {
                return Some((to_bp(osd), base + idx as u8));
            }
        }
    }

    None
}
