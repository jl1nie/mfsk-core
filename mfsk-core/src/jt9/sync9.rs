// SPDX-License-Identifier: GPL-3.0-only
//! JT9's candidate stage, as `jt9` runs it: the half-symbol spectra (`jt9_decode.f90:59-98`,
//! `decode_pcm` and `prepare_spectra`, beta6's form of what `symspec.f90` fills for JT9), the per-bin
//! sync and spectral-excess scores (`sync9.f90`), and the bin loop that picks which frequencies
//! `softsym` is run at (`jt9_decode.f90:204-284`, in `jt9/mod.rs`). Ported from WSJT-X v3.3.0-beta6.
//!
//! Until #726 this crate chose JT9 candidates with its own spectrogram scorer (`search::coarse_search`)
//! and tried them in score order. `jt9` instead walks 0.732 Hz bins upwards and decodes at the *first*
//! bin that passes its gates, then marks `i-1..=i+22` done; `peakdt9`'s DT depends on that bin, so the
//! two reported different DTs for the same signal (measured on `130418_1742.wav`: `K1JT N5KDV EM41`
//! -0.10 here, `jt9` 0.1, which is what `peakdt9` gives at 1344.0 Hz; `G7CNF N4HFA EL89` 0.0 here,
//! `jt9` -0.2, `peakdt9` at 1459.0 Hz).

use alloc::vec;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use num_complex::Complex;

use crate::engine::fft::default_planner;

/// `prepare_spectra`'s 16384-point FFT, so `df3 = 12000/16384 = 0.732 Hz`.
const NFFT3: usize = 16_384;
/// Bin width of `ss`, Hz (`jt9_decode.f90:183` `df3 = 1500.0/2048.0`, the same number).
pub(crate) const DF3: f32 = 12_000.0 / NFFT3 as f32;
/// `constants.f90` `NSMAX`: bins kept (`prepare_spectra` keeps `output(0:6826)`).
const NSMAX: usize = 6827;
/// `decode_pcm`'s cap on the half-symbols: `half_symbols = min(181, (count-2048)/3456)`.
const NZHSYM_MAX: usize = 181;
/// One half-symbol of JT9-1 (`nsps = 6912`), the step of `prepare_spectra`'s rows.
const KSTEP: usize = 3456;
/// `jt9sync.f90` `ii2`: the sync symbols' half-symbol positions, 1-based.
const II2: [i32; 16] = [
    1, 3, 9, 19, 31, 45, 65, 69, 101, 103, 109, 119, 131, 145, 165, 169,
];
/// `jt9_decode.f90:185` `tstep = 0.5*nsps/12000`.
const TSTEP: f32 = 0.5 * 6912.0 / 12_000.0;

/// `pctile.f90`: sort a copy, take element `nint(n*0.01*npct)`, 1-based, clamped to `1..=n`.
fn pctile(x: &[f32], npct: u32) -> f32 {
    if x.is_empty() {
        return 1.0;
    }
    let mut tmp = x.to_vec();
    tmp.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let n = tmp.len();
    let j = ((n as f32 * 0.01 * npct as f32).round() as usize).clamp(1, n);
    tmp[j - 1]
}

/// The output of `sync9` over bins `ia..=ib` (1-based, as `ss`'s second index).
pub(crate) struct Sync9 {
    /// First bin, 1-based.
    pub ia: usize,
    /// `ccfred(ia..=ib)`: best sync sum over lags, normalised to twice the band median.
    pub ccfred: Vec<f32>,
    /// `red2(ia..=ib)`: smoothed spectral excess over the bins ten either side.
    pub red2: Vec<f32>,
}

impl Sync9 {
    pub fn ccfred(&self, i: usize) -> f32 {
        self.ccfred[i - self.ia]
    }
    pub fn red2(&self, i: usize) -> f32 {
        self.red2[i - self.ia]
    }
    pub fn ib(&self) -> usize {
        self.ia + self.ccfred.len() - 1
    }
}

/// The bin range `jt9_decode.f90:190-191` scans for a band in Hz: `ia = max(1, nint(fa/df3))`,
/// `ib = min(NSMAX, nint(fb/df3))`.
pub(crate) fn bin_range(fa_hz: f32, fb_hz: f32) -> (usize, usize) {
    let ia = ((fa_hz / DF3).round() as i64).max(1) as usize;
    let ib = ((fb_hz / DF3).round() as i64).min(NSMAX as i64).max(0) as usize;
    (ia, ib)
}

/// `prepare_spectra` for bins `ia..=ib`, then `sync9.f90` over them.
///
/// `ss(row, i)` is the power of a 16384-point FFT, without a window, of the audio whose window *ends*
/// at sample `row * 3456` (1-based), zeros before the buffer. `prepare_spectra` scales the samples by
/// 0.1 and the power by `(1/16384)^2`; `sync9` only takes ratios of `ss` (percentiles, the median of
/// `ccfred`, an rms), so neither is applied here.
///
/// Returns `None` where `jt9` decodes nothing: fewer than 2 half-symbols of audio or all of it zero
/// (`decode_pcm`, `jt9_decode.f90:70-73`), or an empty range (`:192` `if(ib.le.ia) go to 999`).
/// `nzhsym` is `decode_pcm`'s `half_symbols`, so a buffer shorter than 60 s is scored over what it has.
pub(crate) fn sync9(audio: &[f32], ia: usize, ib: usize) -> Option<Sync9> {
    let count = audio.len().min(60 * 12_000);
    let nzhsym = (count.saturating_sub(2048) / KSTEP).min(NZHSYM_MAX);
    if nzhsym < 2 || audio[..count].iter().all(|&x| x == 0.0) || ib <= ia {
        return None;
    }
    let audio = &audio[..count];
    let nb = ib - ia + 1;
    // ss[bin - ia][ihsym - 1], bin-major so `sync9`'s per-bin loop reads a contiguous row.
    let mut ss = vec![0.0f32; nb * nzhsym];
    let r2c = default_planner().plan_real_forward(NFFT3);
    let mut xc = vec![0.0f32; NFFT3];
    let mut cx = vec![Complex::new(0.0f32, 0.0); NFFT3 / 2 + 1];
    for ihsym in 1..=nzhsym {
        // `last = row*3456`, `first = last-16384+1`; `input(i) = samples(first+i-1)`, 1-based.
        let ja = ihsym * KSTEP;
        for (i, x) in xc.iter_mut().enumerate() {
            let j = ja as i64 + i as i64 - (NFFT3 as i64 - 1); // 1-based
            *x = if j >= 1 && (j as usize) <= audio.len() {
                audio[j as usize - 1]
            } else {
                0.0
            };
        }
        r2c.process(&mut xc, &mut cx);
        // `spectra(row, i)` = |output(i-1)|^2.
        for b in 0..nb {
            ss[b * nzhsym + ihsym - 1] = cx[ia + b - 1].norm_sqr();
        }
    }

    // `sync9.f90:16-56`: per bin, normalise to the 40th percentile, clip at 3, remove the 45th
    // percentile, and take the best 16-sync-symbol sum over `lag1..=lag2`.
    let lag1 = -((2.5 / TSTEP + 0.9999) as i32);
    let lag2 = (5.0 / TSTEP + 0.9999) as i32;
    let mut ccfred = vec![0.0f32; nb];
    let mut ss1 = vec![0.0f32; nzhsym];
    for b in 0..nb {
        ss1.copy_from_slice(&ss[b * nzhsym..(b + 1) * nzhsym]);
        let xmed = pctile(&ss1, 40);
        if xmed <= 0.0 {
            continue;
        }
        for v in ss1.iter_mut() {
            *v = (*v / xmed - 1.0).min(3.0);
        }
        let sbase = pctile(&ss1, 45);
        for v in ss1.iter_mut() {
            *v -= sbase;
        }
        let mut smax = 0.0f32;
        for lag in lag1..=lag2 {
            let mut sum1 = 0.0f32;
            for &k in &II2 {
                let k = k + lag;
                if k >= 1 && k as usize <= nzhsym {
                    sum1 += ss1[k as usize - 1];
                }
            }
            if sum1 > smax {
                smax = sum1;
            }
        }
        ccfred[b] = smax;
    }
    let xmed = pctile(&ccfred, 50);
    let xmed = if xmed <= 0.0 { 1.0 } else { xmed };
    for v in ccfred.iter_mut() {
        *v = 2.0 * *v / xmed;
    }

    // `sync9.f90:63-88`: the average spectrum (zero outside `ia..=ib`, as `savg` is there), smoothed
    // by a 21-bin boxcar less a 5-bin flank each side, over 5x the rms of its 20th percentile, minus
    // the larger of the values ten bins either side.
    let mut savg = vec![0.0f32; nb];
    for (b, s) in savg.iter_mut().enumerate() {
        *s = ss[b * nzhsym..(b + 1) * nzhsym].iter().sum::<f32>() / nzhsym as f32;
    }
    let smo = |j: i32| -> f32 {
        if (0..=20).contains(&j) {
            1.0 / 21.0
        } else {
            -(1.0 / 21.0) * (21.0 / 10.0)
        }
    };
    let savg_at = |i: i64| -> f32 {
        if i >= ia as i64 && i <= ib as i64 {
            savg[(i - ia as i64) as usize]
        } else {
            0.0
        }
    };
    let mut savg2 = vec![0.0f32; nb];
    let mut sq = vec![0.0f32; nb];
    for b in 0..nb {
        let i = (ia + b) as i64;
        let mut sm = 0.0f32;
        for j in -5..=25 {
            if i + j as i64 >= 1 && i + (j as i64) < NSMAX as i64 {
                sm += smo(j) * savg_at(i + j as i64);
            }
        }
        savg2[b] = sm;
        sq[b] = sm * sm;
    }
    let mut red2 = vec![0.0f32; nb];
    let sq0 = pctile(&sq, 20);
    if sq0 > 0.0 {
        let rms = sq0.sqrt();
        for v in savg2.iter_mut() {
            *v /= 5.0 * rms;
        }
        for i in (ia + 11)..=ib.saturating_sub(10) {
            let b = i - ia;
            let r = savg2[b - 10].max(savg2[b + 10]);
            red2[b] = (savg2[b] - r).clamp(-99.0, 99.0);
        }
    }
    Some(Sync9 { ia, ccfred, red2 })
}
