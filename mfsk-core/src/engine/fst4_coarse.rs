//! FST4-specific coarse-candidate stage: a port of WSJT-X
//! `get_candidates_fst4` (`lib/fst4_decode.f90:804-879`) and
//! `fst4_baseline` (`lib/fst4/fst4_baseline.f90`), v3.2.0-rc1.
//!
//! Lives in `engine` beside [`super::ft4_coarse`] for the same reason:
//! `engine::pipeline` calls it behind a runtime `P::ID == Fst4` test, and a
//! `crate::fst4::*` path from `engine` would force the `fst4` feature on for
//! every build.
//!
//! Upstream finds FST4 candidates in the frequency domain alone. It sums the
//! whole-slot FFT into `df2 = baud/2` bins, takes the four-tone CCF
//! `s2(i) = s(i-3)+s(i-1)+s(i+1)+s(i+3)`, divides by a fitted noise
//! baseline (so noise sits near 1), and peels peaks off with a CLEAN loop
//! until one falls below `minsync`. Time is left entirely to
//! `fst4_sync_search`, which searches ±1.5 s absolutely.
//!
//! This crate used the generic 2-D Costas search ([`super::sync::coarse_sync`])
//! instead, OR-ed with an approximation of this CCF (#146). On noise that
//! search let `max_cand` candidates through on every file, each paying a
//! full `fst4_sync_search`. Upstream refines 2-6 (`timer.out`, `sync240`
//! calls, FST4-15 AWGN and CCIR-poor). Measured against `jt9 -7 -d 3` on the
//! T1 task (#554, FST4-15/30/60, single-threaded): 652 ms a file before,
//! 76 ms after, against upstream's 121 ms; no channel group behind upstream
//! either way.
//!
//! Deliberately faithful, quirks included:
//! - `fst4_baseline` keeps at most 1000 lower-envelope points and, past
//!   that, overwrites the last one (`if (k.lt.1000) k=k+1`). FST4-120 and
//!   FST4-300 have more than 1000 bins in a wideband window, so their fit
//!   leans on the low end of the band, as upstream's does.
//! - The baseline covers `ina+3..=inb-3`; the three bins at each end of
//!   the CCF stay unnormalised (`sbase` is 0 dB there). They lie outside
//!   the peak search window, which is 100 Hz inside the noise window.
//! - CLEAN subtracts 0.9 of the model at the peak itself, so a peak above
//!   `10 * minsync` can be taken again.

use alloc::vec;
use alloc::vec::Vec;
use num_complex::Complex;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph (see `ft4_coarse`)
use num_traits::Float;

use super::dsp::downsample::DownsampleCfg;
use super::protocol::Protocol;
use super::sync::SyncCandidate;

/// Modulation index: 1 for every FST4 sub-mode this crate wires
/// (`fst4_decode.f90`'s `hmod`, see `fst4::baseline`).
const HMOD: i64 = 1;

/// `get_candidates_fst4`'s candidate array size (`candidates(200,5)`).
pub const MAX_CANDIDATES: usize = 200;

/// Fortran `nint`: round half away from zero.
fn nint(x: f32) -> i64 {
    x.round() as i64
}

/// `fst4_baseline(s,np,ia,ib,npct,sbase)` with `nseg = 8`, `nterms = 3` and
/// the `+0.2` dB offset. Writes the linear baseline into `sbase[ia..=ib]`
/// and leaves the rest of `sbase` as it is (the caller fills it with 1.0,
/// upstream's `10**(0/10)`).
fn fst4_baseline(s: &[f32], ia: usize, ib: usize, npct: usize, sbase: &mut [f32]) {
    const NSEG: usize = 8;
    const KMAX: usize = 1000;
    let n = ib - ia + 1;
    let sw: Vec<f32> = (0..n).map(|k| 10.0 * s[ia + k].log10()).collect();
    let nlen = n / NSEG;
    if nlen == 0 {
        return;
    }
    // `i0=(ib-ia+1)/2` is a length, and `x(k)=i-i0` uses the absolute
    // index `i`. Fit and evaluation share the offset, so only conditioning
    // depends on it; kept as written.
    let i0 = (n / 2) as f64;
    let mut xs: Vec<f64> = Vec::with_capacity(KMAX);
    let mut ys: Vec<f64> = Vec::with_capacity(KMAX);
    let mut tmp: Vec<f32> = Vec::with_capacity(nlen);
    for seg in 0..NSEG {
        let ja = seg * nlen;
        let jb = ja + nlen;
        tmp.clear();
        tmp.extend_from_slice(&sw[ja..jb]);
        tmp.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        // `pctile`: j = nint(npts*0.01*npct), clamped to 1..npts, 1-based.
        let j = nint(nlen as f32 * 0.01 * npct as f32).clamp(1, nlen as i64) as usize;
        let base = tmp[j - 1];
        for (k, &v) in sw.iter().enumerate().take(jb).skip(ja) {
            if v <= base {
                let x = (ia + k) as f64 - i0;
                if xs.len() < KMAX {
                    xs.push(x);
                    ys.push(v as f64);
                } else {
                    xs[KMAX - 1] = x;
                    ys[KMAX - 1] = v as f64;
                }
            }
        }
    }
    if xs.len() < 3 {
        return;
    }
    let a = super::baseline::polyfit_nterm(&xs, &ys, 3);
    for (i, out) in sbase.iter_mut().enumerate().take(ib + 1).skip(ia) {
        let t = i as f64 - i0;
        let db = a[0] + t * (a[1] + t * a[2]) + 0.2;
        *out = 10f32.powf(db as f32 / 10.0);
    }
}

/// `get_candidates_fst4` over the window `fst4_decode.f90:286-292` builds
/// for a decode without Single Decode: the signal window is the caller's
/// band shifted up by 1.5 tones (the CCF peaks at the centre of the four
/// tones), the noise window that band widened by 100 Hz each side.
///
/// `fft_cache` is `c_bigfft`, the whole-slot FFT the pipeline already
/// builds for downsampling ([`super::dsp::downsample::build_fft_cache`]).
/// `minsync` is upstream's: 1.20, or 1.15 for the 15 s period
/// (`fst4_decode.f90:308-309`). At most `max_cand.min(MAX_CANDIDATES)`.
///
/// Candidates come back in CLEAN order, strongest first, with `freq_hz`
/// the tone-0 frequency (`fc - 1.5 baud`, this crate's convention),
/// `dt_sec` 0, and `score` the normalised CCF peak (`candidates(i,2)`).
pub fn fst4_coarse_sync<P: Protocol>(
    fft_cache: &[Complex<f32>],
    cfg: &DownsampleCfg,
    freq_min: f32,
    freq_max: f32,
    minsync: f32,
    max_cand: usize,
) -> Vec<SyncCandidate> {
    let fs = cfg.input_rate as f32;
    let nsps = P::NSPS as f32;
    let baud = fs / nsps;
    let df1 = fs / cfg.fft1_size as f32;
    let df2 = baud / 2.0;
    // `nd=df2/df1`: integer assignment, truncates.
    let nd = (df2 / df1) as i64;
    let ndh = nd / 2;

    // fst4_decode.f90:286-292 (`nfa`/`nfb` are integers there).
    let nfa0 = nint(freq_min);
    let nfb0 = nint(freq_max);
    let fa = 100.max(nint(nfa0 as f32 + 1.5 * baud));
    let fb = 4800.min(nint(nfb0 as f32 + 1.5 * baud));
    let nfa = 100.max(nfa0 - 100);
    let nfb = 4800.min(nfb0 + 100);

    let mut ia = nint(100f32.max(fa as f32) / df2);
    let mut ib = nint(4800f32.min(fb as f32) / df2);
    let mut ina = nint(100f32.max(nfa as f32) / df2);
    let mut inb = nint(4800f32.min(nfb as f32) / df2);
    ia = ia.max(ina);
    ib = ib.min(inb);
    let nnw = nint(48000.0 * nsps * 2.0 / fs);
    let jmax = (fft_cache.len() / 2) as i64;

    // Low-resolution power spectrum, 1-based like upstream's `s(nnw)`.
    let mut s = vec![0.0f32; nnw as usize + 1];
    for i in ina.max(1)..=inb.min(nnw) {
        let j0 = nint(i as f32 * df2 / df1);
        let mut acc = 0.0f32;
        for j in (j0 - ndh).max(0)..=(j0 + ndh).min(jmax) {
            let c = fft_cache[j as usize];
            acc += c.re * c.re + c.im * c.im;
        }
        s[i as usize] = acc;
    }

    ina = ina.max(1 + 3 * HMOD);
    inb = inb.min(nnw - 3 * HMOD);
    if inb - 3 * HMOD <= ina + 3 * HMOD {
        return Vec::new();
    }
    let h = HMOD as usize;
    let mut s2 = vec![0.0f32; nnw as usize + 1];
    for i in ina as usize..=inb as usize {
        s2[i] = s[i - 3 * h] + s[i - h] + s[i + h] + s[i + 3 * h];
    }
    let mut sbase = vec![1.0f32; nnw as usize + 1];
    fst4_baseline(
        &s2,
        (ina + 3 * HMOD) as usize,
        (inb - 3 * HMOD) as usize,
        30,
        &mut sbase,
    );
    if sbase[ina as usize..=inb as usize].iter().any(|&b| b <= 0.0) {
        return Vec::new();
    }
    for i in ina as usize..=inb as usize {
        s2[i] /= sbase[i];
    }

    ia = ia.max(3);
    ib = ib.min(nnw - 2);
    if ib < ia {
        return Vec::new();
    }
    // Model CCF peak removed around each candidate (`xdb(-3:3)`).
    const XDB: [f32; 7] = [0.25, 0.50, 0.75, 1.0, 0.75, 0.50, 0.25];
    let limit = max_cand.min(MAX_CANDIDATES);
    let mut out = Vec::new();
    while out.len() < limit {
        // `maxloc`: the first index of the maximum.
        let mut iploc = ia;
        let mut pval = s2[ia as usize];
        for i in ia + 1..=ib {
            if s2[i as usize] > pval {
                pval = s2[i as usize];
                iploc = i;
            }
        }
        if pval < minsync {
            break;
        }
        for (n, x) in XDB.iter().enumerate() {
            let k = iploc + 2 * HMOD * (n as i64 - 3);
            if (ia..=ib).contains(&k) {
                let u = k as usize;
                s2[u] = (s2[u] - 0.9 * pval * x).max(0.0);
            }
        }
        out.push(SyncCandidate {
            freq_hz: df2 * iploc as f32 - 1.5 * baud,
            dt_sec: 0.0,
            score: pval,
        });
    }
    out
}
