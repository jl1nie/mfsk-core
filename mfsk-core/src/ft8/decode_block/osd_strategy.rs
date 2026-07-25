//! OSD fallback strategy — Step 3 of the per-candidate decode
//! staircase.
//!
//! Runs only when Steps 1–2 (BP staircase) fail and the caller asks
//! for `BpAllOsd` depth at sufficient `sync_quality`. Computes a
//! fresh f32 LLR bundle for the candidate (OSD operates on f32
//! regardless of the embedded fixed-point `LlrT`), walks the four
//! LLR variants (a/b/c/d) through the WSJT-X-faithful OSD entry
//! ([`osd_decode_npre1`] for low-`q` candidates, [`osd_decode_npre1_npre2`]
//! for `q >= Q_NDEEP3_THRESHOLD`), and applies the
//! `nharderrors > 36` cycle gate to weed out high-error CRC-luck
//! codewords.
//!
//! ε.6 of the `docs/CLEANUP_2026_05.md` `decode_block` split. As of
//! issue **#63** this module hosts the WSJT-X-faithful OSD dispatch
//! — the q-conditional split mirrors `osd174_91.f90`'s ndeep=2/3
//! dispatch table, replacing the previous mfsk-core-specific
//! brute-force ndeep=2/3 split (`osd_decode` / `osd_decode_deep`).

use super::super::decode::DecodeDepth;
use crate::core::scalar::Cmplx;
use crate::fec::ldpc::bp::BpResult;
use crate::fec::ldpc::osd::{osd_decode_npre1, osd_decode_npre1_npre2};

/// `sync_quality` threshold for dispatching to the heavier WSJT-X
/// ndeep=3 entry ([`osd_decode_npre1_npre2`]) instead of ndeep=2
/// ([`osd_decode_npre1`]). Mirrors the pre-#63 dispatch's
/// `q >= 18` split between `osd_decode_deep(_, 3, _)` and
/// `osd_decode(_)` (ndeep=2), now with WSJT-X-faithful internals on
/// both sides.
const Q_NDEEP3_THRESHOLD: u32 = 18;

/// OSD `nharderrors` ceiling — matches WSJT-X's universal
/// `WSJTX_NHARDERRORS_MAX = 36` (`ft8b.f90:422`) exactly, same as the
/// BP variants in `process_one_candidate_inner`.
///
/// **History (issue #72 follow-up, 2026-07-18): was 22, a deliberate
/// mfsk-core-specific deviation, now retracted.** The gate had been
/// tightened to 22 specifically to filter 3 candidates on
/// `qso3_busy.wav` — `N1API F2VX 73` (e=30), `N1API HA6FQ -23` (e=25),
/// `CQ EA2BFM IN83` (e=31) — judged phantoms (CRC-14-luck false
/// accepts) because JTDX's own 18-entry recall list wasn't independently
/// verified for those specific entries (see issue #150). A CCIR-fading
/// sensitivity investigation (`docs/notes/FT8_BENCHMARK.md`) using
/// `ft8sim`-synthesized WAVs with a *known* golden message found the 22
/// ceiling silently discarding genuine golden decodes under heavy
/// fading — traced multiple independent LLR variants converging on the
/// exact transmitted text at `hard_errors` in the high 20s/low 30s.
/// Loosening back to WSJT-X's 36 recovered those, and as a direct
/// side effect also recovered the same 3 `qso3_busy.wav` candidates at
/// their *exact* JTDX-claimed text — independent corroboration between
/// two separate decoders on a CRC-14-protected message is strong
/// evidence against coincidence, resolving them as real rather than
/// phantom. Full regression suite (host `ft8_qso3_apoff_recall`,
/// `ft8_decode_block_real_qso`, embedded `decode_block` paths) showed
/// zero change from this widening — nothing was relying on the
/// tightened gate to stay green.
const OSD_HARDERRORS_MAX: u32 = 36;

/// Pass-ID range emitted by the OSD fallback (`14..=17` mirrors the
/// llr-variant order `a/b/c/d` — see the post-0.6.1 pass-ID layout
/// documented inline below). Exposed so `process_one_candidate_inner`
/// can size its pass-tracking buffers without re-declaring magic
/// numbers.
pub(super) const PASS_ID_OSD_A: u8 = 14;

/// Try the OSD staircase for a candidate whose BP staircase didn't
/// converge. Returns `Some((bp_result, pass_id))` on the first
/// accepted CRC-pass codeword, `None` otherwise.
///
/// Gates internally on `depth = BpAllOsd` and `q > 6` so the caller can
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
/// `hard_errors=22`, comfortably under [`OSD_HARDERRORS_MAX`]. The gap
/// was never a sync/LLR/BP/OSD sensitivity problem — mfsk-core's own
/// OSD implementation was already capable of the decode the whole
/// time; this one-line gate was refusing to even try.
pub(super) fn try_fallback(
    cs_scratch: &[[Cmplx<f32>; 8]; 79],
    precomputed_llr: Option<&super::super::llr::LlrSet<f32>>,
    depth: DecodeDepth,
    q: u32,
) -> Option<(BpResult, u8)> {
    if !matches!(depth, DecodeDepth::BpAllOsd) || q <= 6 {
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

    // Pass-ID space (post-0.6.1): BP variants 0..3, AP iaptypes
    // 5..12 (mirroring WSJT-X ipass 5..12), host OSD-Deep 13,
    // embedded OSD a/b/c/d 14/15/16/17. The +10 shift on OSD
    // frees 5..12 for the AP loop being plumbed into the same
    // per-candidate function in 0.6.1.
    for (llr, pid) in [
        (&llr_full_f32.llra, PASS_ID_OSD_A),
        (&llr_full_f32.llrb, PASS_ID_OSD_A + 1),
        (&llr_full_f32.llrc, PASS_ID_OSD_A + 2),
        (&llr_full_f32.llrd, PASS_ID_OSD_A + 3),
    ] {
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
        let osd = if q >= Q_NDEEP3_THRESHOLD {
            osd_decode_npre1_npre2(llr)
        } else {
            osd_decode_npre1(llr)
        };
        if let Some(osd) = osd {
            // WSJT-X-faithful ceiling — see [`OSD_HARDERRORS_MAX`]'s
            // docstring.
            if osd.hard_errors > OSD_HARDERRORS_MAX {
                continue;
            }
            // Reuse `osd.codeword` instead of allocating a fresh
            // zero vec — `OsdResult` already carries the actual
            // decoded codeword bits, which the previous `vec![0; N]`
            // dropped on the floor (Gemini PR #86 review).
            let bp = BpResult {
                message77: osd.message77,
                info: osd.info,
                codeword: osd.codeword,
                hard_errors: osd.hard_errors,
                iterations: 0,
            };
            return Some((bp, pid));
        }
    }
    None
}
