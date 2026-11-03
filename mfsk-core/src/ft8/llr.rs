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

impl<T: LlrScalar> core::fmt::Debug for LlrSet<T> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("LlrSet").finish_non_exhaustive()
    }
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
    compute_llr_metric(cs, false)
}

/// [`compute_llr`] with WSJT-X's `imetric` chosen: `squared` = `imetric` 2
/// (`|cs|²` per tone group before the per-bit max; `ft8b.f90` v3.0.0).
pub fn compute_llr_metric<T: LlrScalar>(cs: &[[Cmplx<f32>; 8]; 79], squared: bool) -> LlrSet<T> {
    let flat = flatten_cs(cs);
    let g = crate::engine::llr::compute_llr_generic_metric::<Ft8, f32, T>(flat, 3, squared);
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
    compute_llr_fast_metric(cs, false)
}

/// [`compute_llr_fast`] with `imetric` chosen (see [`compute_llr_metric`]).
pub fn compute_llr_fast_metric<T: LlrScalar>(
    cs: &[[Cmplx<f32>; 8]; 79],
    squared: bool,
) -> LlrSet<T> {
    let flat = flatten_cs(cs);
    let g = crate::engine::llr::compute_llr_generic_metric::<Ft8, f32, T>(flat, 1, squared);
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
    compute_llr_partial_metric(cs, nsym, false)
}

/// [`compute_llr_partial`] with `imetric` chosen (see [`compute_llr_metric`]).
pub fn compute_llr_partial_metric<T: LlrScalar>(
    cs: &[[Cmplx<f32>; 8]; 79],
    nsym: usize,
    squared: bool,
) -> [T; LDPC_N] {
    let flat = flatten_cs(cs);
    let v = crate::engine::llr::compute_llr_partial_metric::<Ft8, f32, T>(flat, nsym, squared);
    inflate_llr(v)
}

/// WSJT-X's fifth metric `llre`: per bit, the raw `nsym` 1/2/3 metric of
/// largest magnitude (see [`crate::engine::llr::compute_llre_best_of`]).
/// `squared` is `imetric` 2, as for the other variants.
pub fn compute_llre_metric<T: LlrScalar>(cs: &[[Cmplx<f32>; 8]; 79], squared: bool) -> [T; LDPC_N] {
    let flat = flatten_cs(cs);
    inflate_llr(crate::engine::llr::compute_llre_best_of::<Ft8, f32, T>(
        flat, 3, squared,
    ))
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

    /// A direct port of `ft8b.f90` lines 186-257 (the `nsym` 1/2/3 bit metrics, `llrd`, `llre`,
    /// `normalizebmet`, `scalefac`), 1-based indices and all, to check the shared implementation
    /// against it on random spectra. The point of it is the last group of each 29-symbol half:
    /// `do k=1,29,nsym` runs it on into the next frame symbol (a Costas one), which the generic
    /// sweep only does for `LlrTail::ContinueIntoNext`. Against jt9's own LLRs (its `ft8b` traced on
    /// ft8_lib's 22 recordings, 638 candidates at the first search of the first period) the patch it
    /// had before agreed on every sign in 45 % (`llrb`) and 40 % (`llrc`) of them; this one, 94 %.
    #[test]
    fn llr_sets_match_ft8b_f90_including_the_last_group_of_each_half() {
        // ft8b.f90 `graymap`, written out so a wrong `Ft8::GRAY_MAP` cannot pass on both sides.
        let gray: [u8; 8] = [0, 1, 3, 2, 5, 6, 4, 7];
        let mut state = 0x2545_F491_4F6C_DD1Du64;
        let mut rnd = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            // sum of 4 uniforms: bell-shaped enough to give the metrics a spread
            (0..4)
                .map(|k| ((state >> (k * 16)) & 0xFFFF) as f32 / 65535.0)
                .sum::<f32>()
                - 2.0
        };
        for trial in 0..24 {
            let squared = trial % 2 == 1;
            let mut cs = vec![[Cmplx::<f32>::default(); 8]; 79];
            for row in cs.iter_mut() {
                for c in row.iter_mut() {
                    *c = Cmplx {
                        re: rnd(),
                        im: rnd(),
                    };
                }
            }
            let cs: Box<[[Cmplx<f32>; 8]; 79]> = cs.try_into().unwrap();
            // cs(tone, ks) of the Fortran, ks 1-based
            let z = |tone: usize, ks: usize| (cs[ks - 1][tone].re, cs[ks - 1][tone].im);
            let (mut a, mut b, mut c, mut d) = ([0f32; 180], [0f32; 180], [0f32; 180], [0f32; 180]);
            for nsym in 1..=3usize {
                let nt = 1usize << (3 * nsym);
                for ihalf in 1..=2usize {
                    let mut k = 1;
                    while k <= 29 {
                        let ks = if ihalf == 1 { k + 7 } else { k + 43 };
                        let s2: Vec<f32> = (0..nt)
                            .map(|i| {
                                let (i1, i2, i3) = (i / 64, (i & 63) / 8, i & 7);
                                let (re, im) = match nsym {
                                    1 => z(gray[i3] as usize, ks),
                                    2 => {
                                        let (x, y) = (
                                            z(gray[i2] as usize, ks),
                                            z(gray[i3] as usize, ks + 1),
                                        );
                                        (x.0 + y.0, x.1 + y.1)
                                    }
                                    _ => {
                                        let (x, y, w) = (
                                            z(gray[i1] as usize, ks),
                                            z(gray[i2] as usize, ks + 1),
                                            z(gray[i3] as usize, ks + 2),
                                        );
                                        (x.0 + y.0 + w.0, x.1 + y.1 + w.1)
                                    }
                                };
                                let m = (re * re + im * im).sqrt();
                                if squared { m * m } else { m }
                            })
                            .collect();
                        let i32_ = 1 + (k - 1) * 3 + (ihalf - 1) * 87;
                        let ibmax = [2, 5, 8][nsym - 1];
                        for ib in 0..=ibmax {
                            let bit = ibmax - ib;
                            let ones = (0..nt)
                                .filter(|i| i >> bit & 1 == 1)
                                .map(|i| s2[i])
                                .fold(f32::NEG_INFINITY, f32::max);
                            let zeros = (0..nt)
                                .filter(|i| i >> bit & 1 == 0)
                                .map(|i| s2[i])
                                .fold(f32::NEG_INFINITY, f32::max);
                            let bm = ones - zeros;
                            if i32_ + ib > 174 {
                                continue;
                            }
                            match nsym {
                                1 => {
                                    a[i32_ + ib] = bm;
                                    let den = ones.max(zeros);
                                    d[i32_ + ib] = if den > 0.0 { bm / den } else { 0.0 };
                                }
                                2 => b[i32_ + ib] = bm,
                                _ => c[i32_ + ib] = bm,
                            }
                        }
                        k += nsym;
                    }
                }
            }
            let mut e = [0f32; 180];
            for i in 1..=174 {
                e[i] = [a[i], b[i], c[i]]
                    .into_iter()
                    .fold(a[i], |best, v| if v.abs() > best.abs() { v } else { best });
            }
            let norm = |v: &[f32; 180]| -> Vec<f32> {
                let x = &v[1..=174];
                let (m, m2) = (
                    x.iter().sum::<f32>() / 174.0,
                    x.iter().map(|y| y * y).sum::<f32>() / 174.0,
                );
                let var = m2 - m * m;
                let sig = if var > 0.0 { var.sqrt() } else { m2.sqrt() };
                x.iter().map(|y| y / sig * 2.83).collect()
            };
            let ours: LlrSet = compute_llr_metric(&cs, squared);
            let ours_e: [f32; LDPC_N] = compute_llre_metric(&cs, squared);
            for (name, want, got) in [
                ("llra", norm(&a), &ours.llra[..]),
                ("llrb", norm(&b), &ours.llrb[..]),
                ("llrc", norm(&c), &ours.llrc[..]),
                ("llrd", norm(&d), &ours.llrd[..]),
                ("llre", norm(&e), &ours_e[..]),
            ] {
                for (i, (w, g)) in want.iter().zip(got).enumerate() {
                    assert!(
                        (w - g).abs() < 2e-3,
                        "trial {trial} squared={squared} {name} bit {}: ft8b.f90 {w}, ours {g}",
                        i + 1
                    );
                }
            }
        }
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
