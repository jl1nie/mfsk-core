//! Probability-domain vector arithmetic for QRA belief propagation.
//!
//! All BP messages live as length-`M` probability distributions
//! (`M = 64` for Q65). Operations stay in the probability domain
//! rather than log-domain to match the WSJT-X reference
//! (`lib/qra/q65/pdmath.c`).
//!
//! The C version dispatches to fully unrolled per-size kernels
//! (`pd_imul1`, `pd_imul2`, ..., `pd_imul64`) through a function-
//! pointer table indexed by `nlogdim`. Rust's slice operations
//! autovectorise cleanly without that scaffolding, so this module
//! exposes one generic function per operation that infers the size
//! from the slice length.
//!
//! Empty slices are accepted and treated as a no-op (the C version
//! never reaches that case because every concrete codec uses
//! `nlogdim >= 0`, hence size >= 1).

/// Returns the uniform distribution of size `n`, where `n` must be a
/// power of two in `1..=64`. The returned slice is a `'static`
/// constant; callers may copy from it but must not mutate.
///
/// Mirrors `pd_uniform(nlogdim)` from the C reference.
pub fn uniform(n: usize) -> &'static [f32] {
    debug_assert!(
        n.is_power_of_two() && n <= 64,
        "uniform: n must be a power of two in 1..=64, got {n}"
    );
    match n {
        1 => &UNIFORM1,
        2 => &UNIFORM2,
        4 => &UNIFORM4,
        8 => &UNIFORM8,
        16 => &UNIFORM16,
        32 => &UNIFORM32,
        64 => &UNIFORM64,
        _ => unreachable!(),
    }
}

const UNIFORM1: [f32; 1] = [1.0];
const UNIFORM2: [f32; 2] = [1.0 / 2.0; 2];
const UNIFORM4: [f32; 4] = [1.0 / 4.0; 4];
const UNIFORM8: [f32; 8] = [1.0 / 8.0; 8];
const UNIFORM16: [f32; 16] = [1.0 / 16.0; 16];
const UNIFORM32: [f32; 32] = [1.0 / 32.0; 32];
const UNIFORM64: [f32; 64] = [1.0 / 64.0; 64];

/// In-place pointwise multiplication: `dst[i] *= src[i]`.
///
/// Mirrors `pd_imul`. `dst` and `src` must have the same length.
pub fn imul(dst: &mut [f32], src: &[f32]) {
    assert_eq!(dst.len(), src.len(), "imul: length mismatch");
    for (d, &s) in dst.iter_mut().zip(src.iter()) {
        *d *= s;
    }
}

/// In-place normalisation: scales `pd` so its components sum to 1.
/// Returns the **pre-normalisation** sum.
///
/// If the sum is non-positive (numerical underflow or all-zero
/// distribution), `pd` is replaced by the uniform distribution and
/// the original (non-positive) sum is still returned. Mirrors
/// `pd_norm` from the C reference.
pub fn norm(pd: &mut [f32]) -> f32 {
    let sum: f32 = pd.iter().sum();
    if sum <= 0.0 {
        let n = pd.len();
        if n.is_power_of_two() && (1..=64).contains(&n) {
            pd.copy_from_slice(uniform(n));
        } else {
            // Defensive fallback for non-Q65 sizes: spread uniformly.
            let v = 1.0 / (n as f32);
            pd.fill(v);
        }
        return sum;
    }
    let inv = 1.0 / sum;
    for x in pd.iter_mut() {
        *x *= inv;
    }
    sum
}

/// Forward permutation: `dst[k] = src[perm[k]]`.
///
/// Mirrors `pd_fwdperm`. The `perm` indices are `i32` rather than
/// `usize` to match the natural type of the QRA `gfpmat` permutation
/// tables (which are declared `const int *` in the C reference).
pub fn fwdperm(dst: &mut [f32], src: &[f32], perm: &[i32]) {
    assert_eq!(dst.len(), src.len(), "fwdperm: dst/src length mismatch");
    assert_eq!(dst.len(), perm.len(), "fwdperm: perm length mismatch");
    for (d, &p) in dst.iter_mut().zip(perm.iter()) {
        *d = src[p as usize];
    }
}

/// Backward permutation: `dst[perm[k]] = src[k]`.
///
/// Mirrors `pd_bwdperm`. See [`fwdperm`] for the index-type rationale.
pub fn bwdperm(dst: &mut [f32], src: &[f32], perm: &[i32]) {
    assert_eq!(dst.len(), src.len(), "bwdperm: dst/src length mismatch");
    assert_eq!(dst.len(), perm.len(), "bwdperm: perm length mismatch");
    for (k, &s) in src.iter().enumerate() {
        dst[perm[k] as usize] = s;
    }
}

/// Maximum element of the distribution. Returns `0.0` for an empty
/// slice (matching the C reference's `cmax = 0` initialisation under
/// the assumption that probabilities are non-negative).
pub fn max(src: &[f32]) -> f32 {
    let mut m = 0.0_f32;
    for &v in src {
        if v >= m {
            m = v;
        }
    }
    m
}

/// `(max_value, max_index)`. Returns `None` if every element is
/// strictly negative (distributions are assumed non-negative; this
/// matches the C reference's `idxmax = -1` sentinel).
///
/// Tie-breaking matches the C reference: the descending-`>=` scan
/// resolves ties to the **lowest** index.
pub fn argmax(src: &[f32]) -> Option<(f32, usize)> {
    let mut cmax = 0.0_f32;
    let mut imax: Option<usize> = None;
    for i in (0..src.len()).rev() {
        let v = src[i];
        if v >= cmax {
            cmax = v;
            imax = Some(i);
        }
    }
    imax.map(|i| (cmax, i))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uniform_sums_to_one() {
        for &n in &[1usize, 2, 4, 8, 16, 32, 64] {
            let s: f32 = uniform(n).iter().sum();
            assert!(
                (s - 1.0).abs() < 1e-6,
                "uniform({n}) sums to {s}, expected 1.0"
            );
            assert_eq!(uniform(n).len(), n);
        }
    }

    #[test]
    fn imul_pointwise() {
        let mut a = [1.0, 2.0, 3.0, 4.0];
        let b = [0.5, 0.25, 2.0, 1.0];
        imul(&mut a, &b);
        assert_eq!(a, [0.5, 0.5, 6.0, 4.0]);
    }

    #[test]
    fn norm_scales_to_unit_sum() {
        let mut a = [1.0, 3.0, 2.0, 4.0];
        let s = norm(&mut a);
        assert_eq!(s, 10.0);
        assert!((a.iter().sum::<f32>() - 1.0).abs() < 1e-6);
        assert!((a[0] - 0.1).abs() < 1e-6);
    }

    #[test]
    fn norm_zero_resets_to_uniform() {
        let mut a = [0.0_f32; 4];
        let s = norm(&mut a);
        assert_eq!(s, 0.0);
        assert!(a.iter().all(|&x| (x - 0.25).abs() < 1e-6));
    }

    #[test]
    fn norm_negative_resets_to_uniform_q65_size() {
        // A 64-element distribution accidentally summing to <= 0 must
        // fall back to uniform without panicking — Q65 BP can hit this
        // when very-low-likelihood inputs underflow.
        let mut a = [-1e-10_f32; 64];
        let s = norm(&mut a);
        assert!(s <= 0.0);
        for &v in &a {
            assert!((v - 1.0 / 64.0).abs() < 1e-6);
        }
    }

    #[test]
    fn permutations_are_inverses() {
        let src = [10.0, 20.0, 30.0, 40.0];
        let perm = [3i32, 0, 2, 1];
        let mut fwd = [0.0; 4];
        fwdperm(&mut fwd, &src, &perm);
        // dst[k] = src[perm[k]]
        assert_eq!(fwd, [40.0, 10.0, 30.0, 20.0]);

        let mut back = [0.0; 4];
        bwdperm(&mut back, &fwd, &perm);
        // dst[perm[k]] = src[k] — should round-trip to the original.
        assert_eq!(back, src);
    }

    #[test]
    fn argmax_tie_breaks_to_lowest_index() {
        // Every element equal — the C-equivalent reverse scan with `>=`
        // settles on index 0.
        let v = [0.5_f32, 0.5, 0.5, 0.5];
        let (m, i) = argmax(&v).expect("non-negative distribution");
        assert_eq!(m, 0.5);
        assert_eq!(i, 0);
    }

    #[test]
    fn argmax_locates_unique_peak() {
        let v = [0.1_f32, 0.4, 0.2, 0.3];
        let (m, i) = argmax(&v).expect("non-negative distribution");
        assert!((m - 0.4).abs() < 1e-6);
        assert_eq!(i, 1);
    }

    #[test]
    fn argmax_all_negative_returns_none() {
        let v = [-1.0_f32, -2.0, -3.0];
        assert!(argmax(&v).is_none());
    }

    #[test]
    fn max_matches_argmax_value() {
        let v = [0.1_f32, 0.4, 0.2, 0.3];
        let (am, _) = argmax(&v).unwrap();
        assert_eq!(max(&v), am);
    }
}
