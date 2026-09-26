// SPDX-License-Identifier: GPL-3.0-or-later
//! Impulse-noise blanker — a port of WSJT-X's `lib/blanker.f90`, which
//! `fst4_decode.f90` runs on the raw samples before its whole-slot FFT.
//!
//! The threshold is the sample magnitude above which the loudest
//! `npct` percent of the `nz` samples lie; every sample over it is
//! zeroed, and so are the `ndropmax` samples after it. `nz` is the
//! upstream FFT length (`nfft1`), not the audio length: samples past the
//! audio count as zeros in the histogram, as they do upstream.

use alloc::vec;
use alloc::vec::Vec;

/// Blank `audio[..nz]` as `blanker(iwave,nz,ndropmax,npct,c_bigfft)` does,
/// returning the blanked copy (the same length as `audio`; samples past
/// `nz` are left alone, since upstream never reads them).
///
/// `npct = 0` blanks nothing: the threshold search stops at 32768, which
/// no `i16` magnitude exceeds.
pub fn blanker(audio: &[i16], nz: usize, ndropmax: usize, npct: u32) -> Vec<i16> {
    let mut out = audio.to_vec();
    let n_in = audio.len().min(nz);

    // `if(iwave(i).eq.-32768) iwave(i)=-32767`, then `hist(abs(iwave(i)))`.
    let mut hist = vec![0u32; 32769];
    for s in out[..n_in].iter_mut() {
        if *s == i16::MIN {
            *s = -32767;
        }
        hist[s.unsigned_abs() as usize] += 1;
    }
    hist[0] += (nz - n_in) as u32;

    // `n.ge.nint(nz*fblank/ndropmax)`, searching down from 32768.
    let fblank = 0.01 * npct as f32;
    // `nint` of a non-negative value, without `f32::round` (no_std).
    let target = (nz as f32 * fblank / ndropmax as f32 + 0.5) as u32;
    let mut n = 0u32;
    let mut nthresh: i32 = -1;
    for i in (0..=32768usize).rev() {
        n += hist[i];
        if n >= target {
            nthresh = i as i32;
            break;
        }
    }

    let mut ndrop = 0usize;
    for s in out[..n_in].iter_mut() {
        let mut i0 = *s as i32;
        if ndrop > 0 {
            i0 = 0;
            ndrop -= 1;
        }
        if i0.abs() > nthresh {
            i0 = 0;
            ndrop = ndropmax;
        }
        *s = i0 as i16;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_percent_is_the_identity() {
        let a: Vec<i16> = (0..1000).map(|i| ((i * 37) % 2001 - 1000) as i16).collect();
        assert_eq!(blanker(&a, 1000, 1, 0), a);
    }

    #[test]
    fn blanks_the_loudest_and_the_sample_after() {
        let mut a = vec![10i16; 1000];
        a[100] = 30000;
        a[500] = -32768;
        let b = blanker(&a, 1000, 1, 0);
        assert_eq!(b[500], -32767, "the -32768 clamp applies at any npct");
        let mut a2 = a.clone();
        a2[500] = -32767;
        let out = blanker(&a2, 1000, 1, 1);
        // Target 10 samples: the threshold falls to 10, and `abs > 10` blanks only
        // the two impulses — and the sample after each.
        assert_eq!(out[100], 0);
        assert_eq!(out[101], 0);
        assert_eq!(out[500], 0);
        assert_eq!(out[501], 0);
        assert_eq!(out[102], 10);
        assert_eq!(out.iter().filter(|&&s| s == 0).count(), 4);
    }

    #[test]
    fn padding_counts_as_zeros() {
        // 60 % of nz = 200 is 120 samples, more than the 100 non-zero ones, so
        // the search reaches the padding's zeros, the threshold is 0 and every
        // sample goes. With nz = 100 the same 60 % stops at 5, and `abs > 5`
        // blanks nothing.
        let a = vec![5i16; 100];
        assert!(blanker(&a, 200, 1, 60).iter().all(|&s| s == 0));
        assert_eq!(blanker(&a, 100, 1, 60), a);
    }
}
