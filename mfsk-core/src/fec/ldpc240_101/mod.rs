//! LDPC(240, 101) codec with CRC-24 for the WSJT FST4 / FST4W family.
//!
//! Phase 0c-B unified the implementation: this module no longer carries
//! its own BP / OSD / encode bodies — those live in
//! [`crate::fec::ldpc`] and are parameterised by [`Ldpc240_101Params`].
//! All that remains here are the protocol-specific tables (in [`tables`])
//! and the CRC-24 helpers ([`crc24`], [`check_crc24`]).
//!
//! The public surface is preserved as a type alias:
//! `pub type Ldpc240_101 = LdpcCodec<Ldpc240_101Params>` lives here so
//! callers continue to write `mfsk_core::fec::Ldpc240_101`.

pub mod tables;

use alloc::vec;
use alloc::vec::Vec;

use crate::engine::protocol::BpPooledFec;
use crate::engine::{FecCodec, FecOpts, FecResult};
use crate::fec::ldpc::bp::{
    BpScratch, bp_decode_generic_kind, bp_decode_generic_kind_with_scratch, bp_llr_zsum,
    bp_llr_zsum_with_scratch,
};
use crate::fec::ldpc::osd::{
    OsdResult, ldpc_encode_generic, osd_decode_generic, osd_decode_npre_generic,
};
use crate::fec::ldpc::params::Ldpc240_101Params;

pub const LDPC_N: usize = 240;
pub const LDPC_K: usize = 101;
pub const LDPC_M: usize = LDPC_N - LDPC_K; // 139

// ────────────────────────────────────────────────────────────────────
// CRC-24

/// CRC-24Q as used by WSJT-X FST4: polynomial 0x100065B, applied bit-
/// serially over the message padded with 24 zeros.
///
/// Matches the `get_crc24` subroutine in WSJT-X `lib/fst4/get_crc24.f90`.
pub fn crc24(bits: &[u8]) -> u32 {
    let mut r = [0u8; 25];
    for (i, slot) in r.iter_mut().enumerate() {
        *slot = if i < bits.len() { bits[i] & 1 } else { 0 };
    }
    const POLY: [u8; 25] = [
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 1,
    ];
    let n = bits.len().saturating_sub(25);
    for i in 0..=n {
        if i + 25 <= bits.len() {
            r[24] = bits[i + 24] & 1;
        } else {
            r[24] = 0;
        }
        let top = r[0];
        if top != 0 {
            for (rv, pv) in r.iter_mut().zip(POLY.iter()) {
                *rv ^= *pv;
            }
        }
        let first = r[0];
        for k in 0..24 {
            r[k] = r[k + 1];
        }
        r[24] = first;
    }
    let mut v = 0u32;
    for &b in &r[..24] {
        v = (v << 1) | (b as u32);
    }
    v
}

/// Verify CRC-24 for a 101-bit decoded word (77 msg + 24 CRC).
///
/// Accepts any `&[u8]` slice; lengths other than [`LDPC_K`] (= 101) are
/// rejected so the function is suitable as a `MessageCodec::verify_info`
/// implementation passed through `FecOpts::verify_info`.
pub fn check_crc24(decoded: &[u8]) -> bool {
    if decoded.len() != LDPC_K {
        return false;
    }
    let mut with_zero = [0u8; LDPC_K];
    with_zero[..77].copy_from_slice(&decoded[..77]);
    let expected = crc24(&with_zero);

    let mut got = 0u32;
    for &b in &decoded[77..101] {
        got = (got << 1) | (b as u32 & 1);
    }
    expected == got
}

// ────────────────────────────────────────────────────────────────────
// OSD dispatch (issue #306/#198): WSJT-X-faithful npre1/npre2 for
// ndeep=2/3, replacing `osd_decode_generic`'s unpruned k1/k2/k3
// combinatorial search for those two depths.
//
// `osd240_101.f90`'s ndeep table (see the module's issue #198 cross-
// reference): ndeep=2 is `nord=1, npre1=1, npre2=0, ntheta=12`; ndeep=3
// adds `npre2=1, ntau=14`. Both share `nt=40` with
// `osd::NPRE1_PARITY_WINDOW`. `Ldpc240_101`'s own callers below only
// ever request `osd_depth` 2 or 3 (`opts.osd_depth.min(3)` — order-4 was
// already unreachable in the previous dispatch too), so `ndeep<2` is a
// dead branch kept only for defensiveness; if hit, it falls back to the
// (still-shipping, still order-≤3-tested) combinatorial search rather
// than assuming a WSJT-X ndeep this port hasn't mapped a constant table
// for.

/// `ntheta` for FST4's ndeep=2 and ndeep=3 dispatch — both use 12
/// (`osd240_101.f90`'s `ndeep=2`/`ndeep=3` branches).
const FST4_NPRE_NTHETA: u32 = 12;

/// `ntau` for FST4's ndeep=3 dispatch (`osd240_101.f90`'s `ndeep=3`
/// branch). Ignored for ndeep=2 (no `npre2` pass).
const FST4_NPRE_NTAU: usize = 14;

/// Diagnostic-only override (issue #306/#198 bug hunt): when set, force
/// [`fst4_osd_decode`] to run the old unpruned combinatorial search
/// instead of the production npre1/npre2 dispatch, for the remainder of
/// the process. `internal-testing`-gated via [`fst4_osd_diag_force_old`]
/// below — reads as `false` (production npre1/npre2 path) in every
/// non-`internal-testing` build, so this can't affect a real release
/// artifact. `core::sync::atomic` rather than `std::thread_local!` since
/// this crate is `no_std` + `alloc`.
#[cfg(feature = "internal-testing")]
static OSD_DIAG_FORCE_OLD: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Force [`fst4_osd_decode`] (and hence the whole production decode
/// path, `decode_soft`/`decode_soft_pooled`/`DecodeRequest`) to use the
/// pre-port unpruned combinatorial OSD search instead of npre1/npre2,
/// for the rest of this process. Lets a test run the *exact* production
/// entry point (e.g. `decode_wav_fst4_60` in `tests/fst4_sweep.rs`)
/// twice — once per algorithm — for direct end-to-end A/B comparison,
/// without reimplementing `decode_soft`'s BP/zsum control flow
/// separately in the test and risking that reimplementation drifting
/// out of sync with the real thing. Diagnostic only, not thread-safe
/// against concurrently-running tests that also decode FST4 — callers
/// should serialise (e.g. `--test-threads=1`, or reset to `false`
/// immediately after the comparison).
#[cfg(feature = "internal-testing")]
pub fn fst4_osd_diag_force_old(force_old: bool) {
    OSD_DIAG_FORCE_OLD.store(force_old, core::sync::atomic::Ordering::Relaxed);
}

/// FST4's OSD dispatch: WSJT-X-faithful `npre1`/`npre1+npre2` for
/// `ndeep` 2/3 (see module-level doc comment above), falling back to
/// [`osd_decode_generic`] for any other depth.
fn fst4_osd_decode(llr: &[f32], ndeep: u8, verify: Option<fn(&[u8]) -> bool>) -> Option<OsdResult> {
    #[cfg(feature = "internal-testing")]
    let use_npre = !OSD_DIAG_FORCE_OLD.load(core::sync::atomic::Ordering::Relaxed);
    #[cfg(not(feature = "internal-testing"))]
    let use_npre = true;
    fst4_osd_decode_dispatch(llr, ndeep, verify, use_npre)
}

/// Shared implementation behind [`fst4_osd_decode`], parameterised on
/// which search to run — `use_npre=true` is production (WSJT-X-faithful
/// `npre1`/`npre1+npre2`), `use_npre=false` is the previous unpruned
/// `osd_decode_generic` combinatorial search. The `use_npre=false` arm
/// (driven by [`fst4_osd_diag_force_old`]) exists so a test can run the
/// *exact* production entry point through both searches for a direct
/// end-to-end A/B comparison, instead of reimplementing `decode_soft`'s
/// dispatch logic separately and risking it drifting out of sync with
/// the real thing.
fn fst4_osd_decode_dispatch(
    llr: &[f32],
    ndeep: u8,
    verify: Option<fn(&[u8]) -> bool>,
    use_npre: bool,
) -> Option<OsdResult> {
    if !use_npre {
        return osd_decode_generic::<Ldpc240_101Params>(llr, ndeep, LDPC_K, verify, false);
    }
    match ndeep {
        2 => osd_decode_npre_generic::<Ldpc240_101Params>(llr, FST4_NPRE_NTHETA, 0, false, verify),
        3 => osd_decode_npre_generic::<Ldpc240_101Params>(
            llr,
            FST4_NPRE_NTHETA,
            FST4_NPRE_NTAU,
            true,
            verify,
        ),
        _ => osd_decode_generic::<Ldpc240_101Params>(llr, ndeep, LDPC_K, verify, false),
    }
}

// ────────────────────────────────────────────────────────────────────
// FecCodec impl

/// Zero-sized LDPC(240, 101) codec — a thin wrapper that pins the
/// generic [`crate::fec::ldpc::params::LdpcParams`]-based
/// implementation to [`Ldpc240_101Params`].
#[derive(Copy, Clone, Debug, Default)]
pub struct Ldpc240_101;

impl crate::engine::protocol::sealed::Sealed for Ldpc240_101 {}
impl FecCodec for Ldpc240_101 {
    const N: usize = LDPC_N;
    const K: usize = LDPC_K;

    fn encode(&self, info: &[u8], codeword: &mut [u8]) {
        assert_eq!(info.len(), LDPC_K, "info must be {} bits", LDPC_K);
        assert_eq!(codeword.len(), LDPC_N, "codeword must be {} bits", LDPC_N);
        ldpc_encode_generic::<Ldpc240_101Params>(info, codeword);
    }

    fn decode_soft(&self, llr: &[f32], opts: &FecOpts<'_>) -> Option<FecResult> {
        let (llr_arr, ap_storage) = prepare_ap_llr(llr, opts);
        let ap_slice: Option<&[bool]> = ap_storage.as_deref();

        if let Some(r) = bp_decode_generic_kind::<Ldpc240_101Params>(
            &llr_arr,
            ap_slice,
            opts.bp_max_iter,
            opts.verify_info,
            opts.bp_kind,
        ) {
            return Some(FecResult {
                info: r.info,
                hard_errors: r.hard_errors,
                iterations: r.iterations,
            });
        }

        if opts.osd_depth == 0 {
            return None;
        }

        if let Some(r) = fst4_osd_decode(&llr_arr, opts.osd_depth.min(3) as u8, opts.verify_info) {
            return Some(FecResult {
                info: r.info,
                hard_errors: r.hard_errors,
                iterations: 0,
            });
        }

        // Issue #146: WSJT-X's `decode240_101` never feeds OSD the raw
        // channel LLR when BP fails — it feeds the running sum of BP's
        // variable-node soft estimate across the first two iterations
        // (`zsave` in `lib/fst4/decode240_101.f90:51-63`, `maxosd=2`).
        // Diagnostic measurement (`fst4_diag_zsum_osd` in
        // `tests/fst4_sweep.rs`) on FST4-120 near-threshold AWGN trials —
        // the sub-mode with the largest residual gap vs WSJT-X — found
        // this recovers real additional trials (35 of 106 OSD-relevant
        // trials, raw-LLR-OSD recall 37→62 when tried *in addition to*
        // the existing raw-LLR attempt) at the cost of 3 trials where
        // raw succeeds and zsum alone would not — hence "try both", not
        // "replace": only reached when the raw-LLR OSD attempt above
        // already failed, so it can only add successes, never remove
        // any. Skipped under AP hints (`ap_slice.is_some()`) — FST4
        // doesn't wire AP decoding yet (issue #143), and
        // `bp_llr_zsum` doesn't clamp AP-locked bits the way the main
        // BP loop does, so running it under an AP mask would drift
        // those bits away from their hinted value.
        if ap_slice.is_none() {
            let zsum = bp_llr_zsum::<Ldpc240_101Params>(&llr_arr, 2);
            if let Some(r) = fst4_osd_decode(&zsum, opts.osd_depth.min(3) as u8, opts.verify_info) {
                return Some(FecResult {
                    info: r.info,
                    hard_errors: r.hard_errors,
                    iterations: 0,
                });
            }
        }

        None
    }
}

impl BpPooledFec for Ldpc240_101 {
    type Scratch = BpScratch<Ldpc240_101Params, f32>;

    fn decode_soft_pooled(
        &self,
        llr: &[f32],
        opts: &FecOpts<'_>,
        scratch: &mut Self::Scratch,
    ) -> Option<FecResult> {
        let (llr_arr, ap_storage) = prepare_ap_llr(llr, opts);
        let ap_slice: Option<&[bool]> = ap_storage.as_deref();

        if let Some(r) = bp_decode_generic_kind_with_scratch::<Ldpc240_101Params>(
            scratch,
            &llr_arr,
            ap_slice,
            opts.bp_max_iter,
            opts.verify_info,
            opts.bp_kind,
        ) {
            return Some(FecResult {
                info: r.info,
                hard_errors: r.hard_errors,
                iterations: r.iterations,
            });
        }

        // OSD fallback (raw-LLR attempt) stays unpooled — out of scope
        // for this pass, identical to `decode_soft`'s tail above.
        if opts.osd_depth == 0 {
            return None;
        }

        if let Some(r) = fst4_osd_decode(&llr_arr, opts.osd_depth.min(3) as u8, opts.verify_info) {
            return Some(FecResult {
                info: r.info,
                hard_errors: r.hard_errors,
                iterations: 0,
            });
        }

        // zsum-seeded OSD retry (issue #146 — see `decode_soft`'s doc
        // comment for the full rationale) — pooled: `bp_llr_zsum_with_scratch`
        // reuses the same `scratch` the BP staircase above already used,
        // and returns a borrow instead of a fresh `Vec`.
        if ap_slice.is_none() {
            let zsum = bp_llr_zsum_with_scratch::<Ldpc240_101Params>(scratch, &llr_arr, 2);
            if let Some(r) = fst4_osd_decode(zsum, opts.osd_depth.min(3) as u8, opts.verify_info) {
                return Some(FecResult {
                    info: r.info,
                    hard_errors: r.hard_errors,
                    iterations: 0,
                });
            }
        }

        None
    }
}

/// Shared AP-hint LLR preparation for [`Ldpc240_101`]'s pooled and
/// unpooled `decode_soft` — same convention as
/// `crate::fec::ldpc::prepare_ap_llr` for [`crate::fec::ldpc::Ldpc174_91`],
/// just `Vec`-backed (`LDPC_N` = 240 here) instead of array-backed.
fn prepare_ap_llr(llr: &[f32], opts: &FecOpts<'_>) -> (Vec<f32>, Option<Vec<bool>>) {
    assert_eq!(llr.len(), LDPC_N, "llr must be {} values", LDPC_N);
    let mut llr_arr = vec![0f32; LDPC_N];
    llr_arr.copy_from_slice(llr);

    let ap_mask = match opts.ap_mask {
        Some((mask, values)) => {
            assert_eq!(mask.len(), LDPC_N, "ap mask must be {} bits", LDPC_N);
            assert_eq!(values.len(), LDPC_N, "ap values must be {} bits", LDPC_N);
            let apmag = llr_arr.iter().map(|x| x.abs()).fold(0.0f32, f32::max) * 1.01;
            let mut a = vec![false; LDPC_N];
            for i in 0..LDPC_N {
                if mask[i] != 0 {
                    a[i] = true;
                    llr_arr[i] = if values[i] != 0 { apmag } else { -apmag };
                }
            }
            Some(a)
        }
        None => None,
    };
    (llr_arr, ap_mask)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fec::ldpc::bp::bp_decode_generic;
    use crate::fec::ldpc::osd::ldpc_encode_generic;

    /// Round-trip: encode a 101-bit info word, feed perfect LLRs,
    /// decoder should recover the original info. Exercises the full
    /// BP path plus the generator sub-matrix via the generic helpers.
    #[test]
    fn roundtrip_perfect_llr() {
        let mut info = [0u8; LDPC_K];
        for i in 0..77 {
            info[i] = ((i * 7 + 3) & 1) as u8;
        }
        let crc = crc24(&info); // upper 24 bits still zero
        for i in 0..24 {
            info[77 + i] = ((crc >> (23 - i)) & 1) as u8;
        }

        let mut cw = [0u8; LDPC_N];
        ldpc_encode_generic::<Ldpc240_101Params>(&info, &mut cw);
        // Sanity: systematic encode keeps info bits in positions 0..K.
        assert_eq!(&cw[..LDPC_K], &info[..]);

        // Perfect LLR: ±8 per bit, sign follows the bit.
        let mut llr = vec![0f32; LDPC_N];
        for i in 0..LDPC_N {
            llr[i] = if cw[i] == 1 { 8.0 } else { -8.0 };
        }
        let r = bp_decode_generic::<Ldpc240_101Params>(&llr, None, 30, Some(check_crc24))
            .expect("BP converges on perfect LLR");
        assert_eq!(&r.info[..77], &info[..77]);
    }
}
