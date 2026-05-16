//! OSD fallback strategy — Step 3 of the per-candidate decode
//! staircase.
//!
//! Runs only when Steps 1–2 (BP staircase) fail and the caller asks
//! for `BpAllOsd` depth at sufficient `sync_quality`. Computes a
//! fresh f32 LLR bundle for the candidate (OSD operates on f32
//! regardless of the embedded fixed-point `LlrT`), walks the four
//! LLR variants (a/b/c/d) through the WSJT-X-faithful `nord=1 + npre1=1`
//! decoder ([`osd_decode_npre1`]), and applies the
//! `nharderrors > 36` cycle gate to weed out high-error CRC-luck
//! codewords.
//!
//! ε.6 of the `docs/CLEANUP_2026_05.md` `decode_block` split. As of
//! issue **#63** this module hosts the WSJT-X-faithful precoding
//! dispatch — `try_fallback` now matches `ft8b.f90:405`'s hard-coded
//! `norder=2` (= `nord=1 + npre1=1`) regardless of `q`. The hook
//! point that ε.6 carved out is now load-bearing for #63's
//! `osd_decode_npre1` (no longer pure brute-force ndeep=2/3).

use alloc::vec;

use super::super::decode::DecodeDepth;
use super::super::params::LDPC_N;
use super::process_candidates::WSJTX_NHARDERRORS_MAX;
use crate::core::scalar::Cmplx;
use crate::fec::ldpc::bp::BpResult;
use crate::fec::ldpc::osd::osd_decode_npre1;

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
/// Gates internally on `depth = BpAllOsd` and `q >= 12` so the caller
/// can invoke unconditionally — keeps the per-candidate staircase in
/// `process_one_candidate_inner` straightforward (`accepted = accepted
/// .or_else(|| osd_strategy::try_fallback(...))`).
pub(super) fn try_fallback(
    cs_scratch: &[[Cmplx<f32>; 8]; 79],
    depth: DecodeDepth,
    q: u32,
) -> Option<(BpResult, u8)> {
    if !matches!(depth, DecodeDepth::BpAllOsd) || q < 12 {
        return None;
    }

    // OSD operates on `&[f32]` directly — independent of the embedded
    // `LlrT` choice — so we compute a fresh f32 LLR bundle here. Only
    // fires when Steps 1+2 BP failed and `q >= 12`, so the extra
    // compute_llr is cheap relative to the OSD work itself.
    let llr_full_f32: super::super::llr::LlrSet<f32> = super::super::llr::compute_llr(cs_scratch);

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
        // WSJT-X-faithful `nord=1 + npre1=1` dispatch — matches
        // `ft8b.f90:405`'s hard-coded `norder=2` (which inside
        // `osd174_91` runs the npre1 precoding rule).
        //
        // Issue #63 swapped the previous q-conditional split
        // (`osd_decode_deep(_, 3, _)` for q≥18 / un-gated
        // `osd_decode` ndeep=2 otherwise) for this single call —
        // both old paths over-enumerated relative to WSJT-X and
        // surfaced CRC-luck phantoms at `nharderrors >= 25` that
        // the npre1 gate's `ntheta=10` parity-error filter rejects.
        //
        // `osd_decode_npre1` carries its own implicit `check_crc14`
        // verifier (mirrors WSJT-X's `nbadcrc` gate inside the
        // subroutine), so no `Some(check_crc14)` argument here.
        let osd = osd_decode_npre1(llr);
        if let Some(osd) = osd {
            // Mirror the BP-variant `nharderrors > 36` cycle gate
            // (ft8b.f90:422) on the OSD path. WSJT-X's OSD itself
            // returns CRC-pass codewords with negated `nhardmin` on
            // CRC fail (osd174_91:290), but we apply the same upper
            // bound to the hard-error count so high-error CRC-luck
            // codewords don't pass through OSD either. With npre1's
            // `ntheta=10` gate already filtering the most egregious
            // CRC-luck cases this is a belt-and-braces guard against
            // any survivor of both filters.
            if osd.hard_errors > WSJTX_NHARDERRORS_MAX {
                continue;
            }
            let bp = BpResult {
                message77: osd.message77,
                info: osd.info,
                codeword: vec![0u8; LDPC_N],
                hard_errors: osd.hard_errors,
                iterations: 0,
            };
            return Some((bp, pid));
        }
    }
    None
}
