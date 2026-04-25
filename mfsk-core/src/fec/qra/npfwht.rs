//! Non-binary Walsh-Hadamard transform for QRA belief propagation.
//!
//! Ports `lib/qra/q65/npfwht.c` from the WSJT-X distribution. The C
//! reference unrolls a separate butterfly routine for each
//! supported size (1, 2, 4, ..., 64) and dispatches through a
//! function table; in Rust we use the standard iterative in-place
//! butterfly, which the optimiser autovectorises for any size that
//! fits in cache.
//!
//! ## Result equivalence
//!
//! The C reference walks butterfly stages from `dist = N/2` down to
//! `dist = 1`, ping-ponging between two scratch buffers. The
//! iterative version below walks `h = 1` upward to `N/2` in place.
//! Both implement the identical Walsh-Hadamard matrix (each
//! per-stage butterfly operates on disjoint index pairs and so the
//! stages commute), so the final transform is the same.
//!
//! ## Inverse / scaling
//!
//! The transform is its own inverse up to a factor of `N`:
//! `fwht(fwht(x)) == N · x`. Callers that need a true round-trip
//! can divide the second pass by `N`, but QRA BP normalises after
//! every step so the constant scale is absorbed elsewhere.

/// In-place Walsh-Hadamard transform on a buffer whose length is a
/// power of two in `1..=64`.
///
/// Mirrors `np_fwht(nlogdim, dst, src)` from the C reference, except
/// that the operation is in-place. Callers that need separate src
/// and dst buffers should `dst.copy_from_slice(src)` first.
pub fn fwht(buf: &mut [f32]) {
    let n = buf.len();
    debug_assert!(
        n.is_power_of_two() && n <= 64,
        "fwht: length must be a power of two in 1..=64, got {n}"
    );
    let mut h = 1;
    while h < n {
        let mut i = 0;
        while i < n {
            for j in i..i + h {
                let a = buf[j];
                let b = buf[j + h];
                buf[j] = a + b;
                buf[j + h] = a - b;
            }
            i += h * 2;
        }
        h *= 2;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Brute-force reference: build the natural-ordered Hadamard
    /// matrix `H_n` recursively and apply it as `y = H * x`.
    /// `H_n[i][j] = (-1)^(popcount(i & j))`.
    fn naive_hadamard(x: &[f32]) -> Vec<f32> {
        let n = x.len();
        (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| {
                        let s = if (i & j).count_ones() % 2 == 0 {
                            1.0
                        } else {
                            -1.0
                        };
                        s * x[j]
                    })
                    .sum()
            })
            .collect()
    }

    #[test]
    fn fwht_size_1_is_identity() {
        let mut a = [3.5_f32];
        fwht(&mut a);
        assert_eq!(a, [3.5]);
    }

    #[test]
    fn fwht_size_2_matches_butterfly() {
        let mut a = [1.0, 2.0];
        fwht(&mut a);
        // [a+b, a-b] = [3, -1]
        assert_eq!(a, [3.0, -1.0]);
    }

    #[test]
    fn fwht_size_4_matches_naive() {
        let x = [1.0, 2.0, 3.0, 4.0];
        let expected = naive_hadamard(&x);
        let mut a = x;
        fwht(&mut a);
        for (got, want) in a.iter().zip(expected.iter()) {
            assert!((got - want).abs() < 1e-5, "got {got}, want {want}");
        }
    }

    #[test]
    fn fwht_size_8_matches_naive() {
        let x = [1.0_f32, -2.0, 3.5, 0.0, 4.0, -1.0, 2.0, 0.5];
        let expected = naive_hadamard(&x);
        let mut a = x;
        fwht(&mut a);
        for (got, want) in a.iter().zip(expected.iter()) {
            assert!((got - want).abs() < 1e-5, "got {got}, want {want}");
        }
    }

    #[test]
    fn fwht_size_64_matches_naive_on_known_input() {
        let mut x = [0.0_f32; 64];
        for (i, v) in x.iter_mut().enumerate() {
            // Mildly aperiodic sequence so each output bin is exercised.
            *v = ((i as f32) * 0.137).sin() + 0.5;
        }
        let expected = naive_hadamard(&x);
        let mut a = x;
        fwht(&mut a);
        for (i, (got, want)) in a.iter().zip(expected.iter()).enumerate() {
            assert!((got - want).abs() < 1e-3, "bin {i}: got {got}, want {want}");
        }
    }

    #[test]
    fn fwht_double_apply_scales_by_n() {
        // FWHT is its own inverse up to N: fwht(fwht(x)) == N·x.
        for &n in &[2usize, 4, 8, 16, 32, 64] {
            let original: Vec<f32> = (0..n).map(|i| (i as f32) * 0.5 - 1.0).collect();
            let mut a = original.clone();
            fwht(&mut a);
            fwht(&mut a);
            for (i, (&got, &want)) in a.iter().zip(original.iter()).enumerate() {
                let expected = (n as f32) * want;
                assert!(
                    (got - expected).abs() < 1e-3,
                    "n={n} bin {i}: got {got}, want {expected}"
                );
            }
        }
    }
}
