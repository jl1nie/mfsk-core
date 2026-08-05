//! FT8 LLR — thin wrapper over [`crate::engine::llr`].
//!
//! Preserves the pre-refactor `[[Complex;8];79]` input type for
//! compatibility with `decode`, `equalizer`, and external callers.
//! Internally flattens to the row-major layout used by the generic
//! implementation, then re-inflates the output.

use alloc::vec::Vec;

use super::Ft8;

use super::params::{LDPC_N, LLR_SCALE};
use crate::engine::scalar::{Cmplx, LlrScalar};

pub use crate::engine::llr::LlrSet as GenericLlrSet;

/// FT8 LLR bundle: four fixed-length (174-bit) variants. Generic over
/// the [`LlrScalar`] storage; defaults to `f32` for backward
/// compatibility (`LlrSet` ≡ `LlrSet<f32>`). The
/// [`crate::engine::scalar::Q11i16`] instantiation (`LlrSet<Q11i16>`)
/// feeds the integer-only NMS BP under the `fixed-point` feature
/// since 0.6.2 — the 0.5.x line used `LlrSet<Q3i8>`
/// ([`crate::engine::scalar::Q3i8`] stays in `engine::scalar` for the
/// comparison path). The Q3i8 ↔ Q11i16 rationale is in
/// `CHANGELOG.md` 0.5.4 + 0.6.2 / 0.6.3 entries and in
/// `docs/reference/EMBEDDED.md`'s `LlrScalar` section.
pub struct LlrSet<T: LlrScalar = f32> {
    pub llra: [T; LDPC_N],
    pub llrb: [T; LDPC_N],
    pub llrc: [T; LDPC_N],
    pub llrd: [T; LDPC_N],
}

/// Zero-cost layout cast: `cs` is already contiguous with the flat
/// `[Cmplx<f32>; 632]` layout `compute_llr_generic`/etc. consume — no
/// allocation or copy needed, just a reinterpreted view. Replaces a
/// former `flatten_cs` that allocated+copied a fresh `Vec` on every
/// call (this function runs 3-5+ times per candidate, up to ~1200
/// candidates/decode via `sync_quality` alone).
#[inline]
fn flatten_cs(cs: &[[Cmplx<f32>; 8]; 79]) -> &[Cmplx<f32>] {
    cs.as_slice().as_flattened()
}

#[inline]
fn inflate_llr<T: LlrScalar>(v: Vec<T>) -> [T; LDPC_N] {
    let mut out = [T::ZERO; LDPC_N];
    let n = v.len().min(LDPC_N);
    out[..n].copy_from_slice(&v[..n]);
    out
}

/// Compute soft LLRs from complex symbol spectra. Generic over the
/// LLR scalar `T` (`f32` host path, `Q11i16` under `fixed-point`
/// since 0.6.2). cs storage stays `Complex<f32>`-typed for source
/// compat; the internal `compute_llr_generic` consumes a layout-cast
/// `&[Cmplx<f32>]` view via `flatten_cs`.
pub fn compute_llr<T: LlrScalar>(cs: &[[Cmplx<f32>; 8]; 79]) -> LlrSet<T> {
    let flat = flatten_cs(cs);
    let g = crate::engine::llr::compute_llr_generic::<Ft8, f32, T>(flat, 3);
    // Sanity check scale consistency at build time.
    debug_assert!((crate::engine::llr::LLR_SCALE - LLR_SCALE).abs() < 1e-6);
    LlrSet {
        llra: inflate_llr(g.llra),
        llrb: inflate_llr(g.llrb),
        llrc: inflate_llr(g.llrc),
        llrd: inflate_llr(g.llrd),
    }
}

/// LLRs for the BP-only path: skips nsym=2 and nsym=3 (~5× faster
/// than [`compute_llr`]). `llrb` / `llrc` come back zero — only
/// `llra` and `llrd` are valid.
pub fn compute_llr_fast<T: LlrScalar>(cs: &[[Cmplx<f32>; 8]; 79]) -> LlrSet<T> {
    let flat = flatten_cs(cs);
    let g = crate::engine::llr::compute_llr_generic::<Ft8, f32, T>(flat, 1);
    LlrSet {
        llra: inflate_llr(g.llra),
        llrb: inflate_llr(g.llrb),
        llrc: inflate_llr(g.llrc),
        llrd: inflate_llr(g.llrd),
    }
}

/// Lazy single-`nsym` LLR — feeds the stage-3 BP staircase one
/// variant at a time so Step 2 doesn't pay the heavy nsym=3 cost
/// (~80 % of [`compute_llr`]) unless variants a/d/b all failed.
/// `nsym = 1` returns llra, `2` → llrb, `3` → llrc. Step 1's
/// `compute_llr_fast` already provides llra/llrd, so Step 2 in
/// `decode_block::process_candidates_with` only ever calls this for
/// `nsym = 2` and `nsym = 3`.
pub fn compute_llr_partial<T: LlrScalar>(cs: &[[Cmplx<f32>; 8]; 79], nsym: usize) -> [T; LDPC_N] {
    let flat = flatten_cs(cs);
    let v = crate::engine::llr::compute_llr_partial::<Ft8, f32, T>(flat, nsym);
    inflate_llr(v)
}

/// WSJT-X compatible SNR from 8-tone spectra + decoded 79-tone sequence.
pub fn compute_snr_db(cs: &[[Cmplx<f32>; 8]; 79], itone: &[u8; 79]) -> f32 {
    let flat = flatten_cs(cs);
    crate::engine::llr::compute_snr_db_generic::<Ft8, f32>(flat, itone)
}

/// Hard-decision sync quality (0..21). FT8 threshold ≤ 6 → bail out.
pub fn sync_quality(cs: &[[Cmplx<f32>; 8]; 79]) -> u32 {
    let flat = flatten_cs(cs);
    crate::engine::llr::sync_quality_generic::<Ft8, f32>(flat)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_spectra_zero_llr() {
        let cs: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
        let llr_set: LlrSet = compute_llr(&cs);
        let any_large = llr_set.llra.iter().any(|&x| x.abs() > 1.0);
        assert!(!any_large, "zero input should not produce large LLRs");
    }

    #[test]
    fn llr_length_is_174() {
        let cs: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
        let llr_set: LlrSet = compute_llr(&cs);
        assert_eq!(llr_set.llra.len(), 174);
        assert_eq!(llr_set.llrd.len(), 174);
    }

    #[test]
    fn sync_quality_costas_perfect() {
        use super::super::params::COSTAS;
        let mut cs = vec![[Cmplx::<f32>::default(); 8]; 79];
        for &sym_offset in &[0usize, 36, 72] {
            for t in 0..7 {
                let sym = sym_offset + t;
                cs[sym][COSTAS[t]] = Cmplx { re: 1.0, im: 0.0 };
            }
        }
        let cs_box: Box<[[Cmplx<f32>; 8]; 79]> = cs.try_into().unwrap();
        assert_eq!(sync_quality(&cs_box), 21);
    }
}
