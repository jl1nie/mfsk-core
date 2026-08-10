//! Spectrum baseline estimator — port of WSJT-X `baseline.f90`.
//!
//! Algorithm is shared across FT4 / FT8 / FST4 in WSJT-X; this module
//! is the single Rust port. Per-protocol callers feed an averaged
//! linear-power spectrum and a [freq_min..=freq_max] working range,
//! and receive a per-bin baseline (in **dB**) ready for normalisation.
//!
//! Algorithm (matches `WSJT-X/lib/ft4/ft4_baseline.f90`,
//! `WSJT-X/lib/ft8/baseline.f90`):
//! 1. Convert avg-power spectrum to dB
//! 2. Split `[freq_min..freq_max]` into `NSEG=10` equal-width segments
//! 3. In each segment take the `NPCT=10`-percentile (= "noise floor"
//!    candidate points). Save those points as `(x=bin-midbin, y=dB)`
//! 4. Fit a 5-term polynomial to the collected points
//! 5. Evaluate the polynomial at each bin → `sbase[bin]` (in dB), with
//!    a fixed `+0.65 dB` offset to match WSJT-X line 43
//!
//! Originally lived in `crate::ft8::baseline`; lifted to
//! `crate::engine::baseline` so FT4's `coarse_sync` can use it for
//! candidate-spectrum normalisation (see issue #18).

extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Tuning knobs for [`fit_baseline_with`] — same percentile+polyfit
/// algorithm shape, different constants per WSJT-X source file. Plain
/// runtime loop bounds (not const generics — no monomorphization win
/// for NSEG/NPCT/NTERMS, and mixing families in one binary is normal:
/// FT8 + FT4 + FST4 all decode from the same process).
#[derive(Clone, Copy, Debug)]
pub struct BaselineParams {
    pub nseg: usize,
    pub npct: usize,
    pub nterms: usize,
    pub offset_db: f32,
}

impl BaselineParams {
    /// `WSJT-X/lib/ft8/baseline.f90` / `WSJT-X/lib/ft4/ft4_baseline.f90`
    /// — identical constants in both files.
    pub const FT8_FT4: Self = Self {
        nseg: 10,
        npct: 10,
        nterms: 5,
        offset_db: 0.65,
    };
    /// `WSJT-X/lib/fst4/fst4_baseline.f90`.
    pub const FST4: Self = Self {
        nseg: 8,
        npct: 30,
        nterms: 3,
        offset_db: 0.2,
    };
}

/// Per-bin baseline in **dB** for `avg_spectrum_power[freq_min_bin..=freq_max_bin]`.
/// Returns a vec the same length as the input range.
///
/// `avg_spectrum_power[i]` is the mean linear power at FFT bin `i`
/// across all time slices of the slot. `freq_min_bin` and
/// `freq_max_bin` are inclusive bin indices into `avg_spectrum_power`.
///
/// Output indexing: `out[i - freq_min_bin]` is the baseline at bin `i`.
///
/// Thin wrapper over [`fit_baseline_with`] using
/// [`BaselineParams::FT8_FT4`] — existing FT8/FT4 callers are
/// unaffected by the generalisation.
pub fn fit_baseline(
    avg_spectrum_power: &[f32],
    freq_min_bin: usize,
    freq_max_bin: usize,
) -> Vec<f32> {
    fit_baseline_with(
        avg_spectrum_power,
        freq_min_bin,
        freq_max_bin,
        BaselineParams::FT8_FT4,
    )
}

/// Generalised form of [`fit_baseline`] parameterised by
/// [`BaselineParams`] — see that type's docs for which WSJT-X source
/// file each preset matches.
pub fn fit_baseline_with(
    avg_spectrum_power: &[f32],
    freq_min_bin: usize,
    freq_max_bin: usize,
    params: BaselineParams,
) -> Vec<f32> {
    let BaselineParams {
        nseg,
        npct,
        nterms,
        offset_db,
    } = params;

    let ia = freq_min_bin.min(avg_spectrum_power.len().saturating_sub(1));
    let ib = freq_max_bin.min(avg_spectrum_power.len().saturating_sub(1));
    if ib <= ia {
        return Vec::new();
    }
    let n = ib - ia + 1;

    // Convert linear power → dB across the working range.
    let s_db: Vec<f32> = avg_spectrum_power[ia..=ib]
        .iter()
        .map(|&p| 10.0 * p.max(1e-30).log10())
        .collect();

    let nlen = n / nseg;
    if nlen == 0 {
        return s_db;
    }
    let i0 = (n / 2) as i32;

    // Collect lower-envelope (x, y) points across all segments.
    let mut xs: Vec<f64> = Vec::with_capacity(n);
    let mut ys: Vec<f64> = Vec::with_capacity(n);
    for seg in 0..nseg {
        let ja = seg * nlen;
        let jb = (ja + nlen).min(n);
        if jb <= ja {
            continue;
        }
        let mut sorted: Vec<f32> = s_db[ja..jb].to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct_idx = (npct * sorted.len()) / 100;
        let base = sorted[pct_idx.min(sorted.len() - 1)];

        for j in ja..jb {
            if s_db[j] <= base {
                xs.push(j as f64 - i0 as f64);
                ys.push(s_db[j] as f64);
            }
        }
    }

    if xs.len() < nterms {
        // Not enough points; fall back to flat baseline = median of s_db.
        let mut flat = s_db.clone();
        flat.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let med = flat[flat.len() / 2];
        return vec![med + offset_db; n];
    }

    // nterms-term polynomial fit via normal equations
    // (Vandermonde A[i][k] = xs[i]^k, solve A^T A · a = A^T y).
    let coeffs = polyfit_nterm(&xs, &ys, nterms);

    // Evaluate at each bin.
    (0..n)
        .map(|i| {
            let t = i as f64 - i0 as f64;
            let mut p = coeffs[nterms - 1];
            for &c in coeffs[..nterms - 1].iter().rev() {
                p = p * t + c;
            }
            p as f32 + offset_db
        })
        .collect()
}

/// Polynomial fit y = sum_{k=0..nterms-1} a[k] * x^k  via normal equations.
/// Direct Gauss elimination on the `nterms`x`nterms` system.
fn polyfit_nterm(xs: &[f64], ys: &[f64], nterms: usize) -> Vec<f64> {
    debug_assert_eq!(xs.len(), ys.len());
    debug_assert!(xs.len() >= nterms);

    // Compute moments: mom[k] = sum xs^k for k = 0..2*(nterms-1)
    let mut mom = vec![0.0f64; 2 * nterms - 1];
    let mut rhs = vec![0.0f64; nterms];
    for (i, &x) in xs.iter().enumerate() {
        let mut xp = 1.0f64;
        for k in 0..nterms {
            mom[k] += xp;
            rhs[k] += xp * ys[i];
            xp *= x;
        }
        // Continue past nterms for the upper moments.
        for k in nterms..2 * nterms - 1 {
            mom[k] += xp;
            xp *= x;
        }
    }

    // Build augmented matrix [A | rhs] where A[i][j] = mom[i+j].
    let mut aug = vec![vec![0.0f64; nterms + 1]; nterms];
    for (i, row) in aug.iter_mut().enumerate() {
        row[..nterms].copy_from_slice(&mom[i..i + nterms]);
        row[nterms] = rhs[i];
    }

    // Gauss elimination with partial pivot.
    for i in 0..nterms {
        // Pivot.
        let mut max_row = i;
        let mut max_abs = aug[i][i].abs();
        for r in (i + 1)..nterms {
            if aug[r][i].abs() > max_abs {
                max_abs = aug[r][i].abs();
                max_row = r;
            }
        }
        if max_row != i {
            aug.swap(i, max_row);
        }
        if aug[i][i].abs() < 1e-30 {
            return vec![0.0; nterms];
        }
        // Eliminate.
        for r in (i + 1)..nterms {
            let factor = aug[r][i] / aug[i][i];
            for c in i..=nterms {
                aug[r][c] -= factor * aug[i][c];
            }
        }
    }

    // Back-substitute.
    let mut a = vec![0.0f64; nterms];
    for i in (0..nterms).rev() {
        let mut s = aug[i][nterms];
        for j in (i + 1)..nterms {
            s -= aug[i][j] * a[j];
        }
        a[i] = s / aug[i][i];
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flat_spectrum_gives_flat_baseline() {
        let n = 1000;
        let avg = vec![100.0f32; n]; // flat power
        let base = fit_baseline(&avg, 0, n - 1);
        assert_eq!(base.len(), n);
        // 10*log10(100) = 20 dB, +0.65 offset
        for &b in base.iter().take(50).chain(base.iter().skip(950)) {
            assert!(
                (b - 20.65).abs() < 0.5,
                "expected ~20.65 dB for flat power, got {b:.2}"
            );
        }
    }

    #[test]
    fn baseline_below_signal_peak() {
        // 1000-bin spectrum: noise floor 100, single signal spike 10000 at bin 500.
        let n = 1000;
        let mut avg = vec![100.0f32; n];
        avg[500] = 10000.0;
        let base = fit_baseline(&avg, 0, n - 1);
        // Baseline at bin 500 should still reflect noise (low percentile),
        // not the signal spike. Allow some polynomial overshoot.
        assert!(
            base[500] < 30.0,
            "baseline at signal bin {:.2} dB; expected near 20 dB noise floor",
            base[500]
        );
    }
}
