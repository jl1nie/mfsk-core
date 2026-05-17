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

/// OSD-specific `nharderrors` ceiling. mfsk-core-specific deviation
/// from WSJT-X's universal `WSJTX_NHARDERRORS_MAX = 36`
/// (`ft8b.f90:422`): we tighten the gate to 22 *only* for OSD-pass
/// codewords (BP variants still use the looser 36 in
/// `process_one_candidate_inner`).
///
/// **Why this isn't WSJT-X-faithful but worth the deviation.** OSD
/// CRC-luck phantoms surface at the boundary where the encoded
/// candidate disagrees with `hdec` in 25–36 bits and happens to pass
/// CRC-14 (~1 in 16,384 random shot). On `qso3_busy.wav` this surfaces
/// 3 fixed phantoms (`N1API F2VX 73` e=30, `N1API HA6FQ -23` e=25,
/// `CQ EA2BFM IN83` e=31) that all of #86's npre1, #87's npre2, and
/// #88's WSJT-X post-OSD gates failed to filter because their
/// `nsync >= 13` puts them above WSJT-X's `nsync <= 10` bail-out.
///
/// Empirically on the same WAV no legitimate signal surfaces from OSD
/// (`pass >= 14`) with `hard_errors > 22` — real low-SNR signals that
/// BP can't decode but OSD recovers all land at e ≤ 16 here. The
/// gate doesn't touch BP variants 0–3 (which can legitimately produce
/// e=20 cases like `N1PJT HB9CQK -10` on `qso3_busy`), only the
/// OSD-pass codewords this module emits.
///
/// Trade-off: a borderline real signal needing OSD with e=23..36 on
/// some other reference WAV would be dropped here. Reference-suite
/// regression on the WSJT-X-distributed FT8 / FT4 / JT9 / WSPR /
/// FST4 / Q65 samples did not surface any such case in 0.6.3.
const OSD_HARDERRORS_MAX: u32 = 22;

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
    precomputed_llr: Option<&super::super::llr::LlrSet<f32>>,
    depth: DecodeDepth,
    q: u32,
) -> Option<(BpResult, u8)> {
    if !matches!(depth, DecodeDepth::BpAllOsd) || q < 12 {
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
            // OSD-specific tightened ceiling at 22 (vs WSJT-X's
            // universal 36 in BP). See [`OSD_HARDERRORS_MAX`]'s
            // docstring for the WSJT-X-deviation rationale and the
            // qso3_busy phantom-elimination data.
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
