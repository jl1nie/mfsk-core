// SPDX-License-Identifier: GPL-3.0-or-later
//! WSJT-X's **q3** decode: full-AP list decoding at the Rx frequency,
//! synchronised on all 85 symbols of each candidate message — a port of
//! `q65_dec0`'s list branch in `lib/qra/q65/q65.f90` (v3.2.0-rc1):
//!
//! - `q65_symspec`: symbol spectra `s1` at `NSTEP = 8` steps per symbol,
//!   every other step computed and the rest interpolated, each smoothed
//!   `nsmo = int(0.5 * mode_q65²)` times (`smo121`, none below 2).
//! - `q65_ccf_85`: for every codeword, the 2-D correlation of its 85 tones
//!   (sync included) against `s1` over `nfqso ± ia2` bins and
//!   `lag1..=lag2`; the best over `nfqso ± ntol` names the frequency, the
//!   lag and the message, and `better` is its lead over the runner-up.
//! - `q65_dec_q3` when `better >= 1.10` (or `mode_q65 >= 8`):
//!   `q65_s1_to_s3` + `q65_bzap` at that alignment, then `q65_dec1` for each
//!   `b90` in `ibwa..=ibwb`: Lorentzian fast-fading intrinsics and
//!   `q65_dec_fullaplist`, accepted when `plog > PLOG_MIN` and the message
//!   is not all zeros.
//!
//! Upstream runs this before anything else whenever it has a codeword
//! list (`ncw > 0`), and only around the Rx frequency.

use alloc::vec;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std makes f32's own methods shadow it
use num_traits::Float;

use crate::engine::dsp::symbol_fft::SymbolFft;
use crate::engine::{DecodeContext, ModulationParams};
use crate::fec::qra::{FadingModel, Q65Codec, intrinsics_fast_fading};
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;

use super::rx::{
    Energies, PLOG_MIN, Q65Result, SnrAudio, default_es_no_metric, finish, ibwa_for_submode,
    submode_index_from_params,
};
use super::sync_pattern::Q65_SYNC_POSITIONS;

/// `NSTEP`: time bins per symbol in `s1`.
const NSTEP: usize = 8;
/// `NBZAP` (`q65_bzap`).
const NBZAP: usize = 15;

/// Where and how widely to look — `q65_dec0`'s `nfqso`, `ntol` and the
/// window `lag1..=lag2` around the nominal start.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Q3Params {
    /// `nfqso`, the Rx frequency (tone 0), Hz.
    pub(crate) rx_freq_hz: f32,
    /// `ntol`, Hz.
    pub(crate) ftol_hz: f32,
    /// The slot's first sample in `audio` (`iwave(0)`); may be negative
    /// when the buffer starts after it, and the missing samples read as 0.
    pub(crate) slot_start: i64,
    /// `lag2` in seconds: 1.0, or the EME delay's 5.5 / 4.0.
    pub(crate) late_sec: f32,
    /// The "w3sz" stage 5 (`q65.f90:211-250`): the drift `q65_ccf_22`
    /// found near the Rx frequency, in Hz, taken out of `s1` before the
    /// 85-symbol sync. 0 for the ordinary q3 decode.
    pub(crate) drift_hz: f32,
}

/// `q65_symspec`'s `s1(iz, jz)`, stored `s1[(j - 1) * iz + (i - 1)]` for
/// Fortran's 1-based `(i, j)`; bin `i` is FFT bin `i`.
struct S1 {
    iz: usize,
    jz: usize,
    data: Vec<f32>,
}

impl S1 {
    #[inline]
    fn at(&self, i: usize, j: usize) -> f32 {
        self.data[(j - 1) * self.iz + (i - 1)]
    }
}

/// `smo121`, as `ft8::list_decode` has it.
fn smo121(x: &mut [f32]) {
    if x.len() < 3 {
        return;
    }
    let mut x0 = x[0];
    for i in 1..x.len() - 1 {
        let x1 = x[i];
        x[i] = 0.5 * x[i] + 0.25 * (x0 + x[i + 1]);
        x0 = x1;
    }
}

/// `q65_symspec` over the slot starting at `slot_start`.
fn symspec<P: ModulationParams>(audio: &[f32], sample_rate: u32, slot_start: i64) -> S1 {
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let istep = nsps / NSTEP;
    let df = sample_rate as f32 / nsps as f32;
    let mode_q65 = 1usize << submode_index_from_params::<P>();
    let iz = (5000.0 / df) as usize;
    let txt = 85.0 * nsps as f32 / 12_000.0;
    let jz = if nsps >= 6912 {
        ((txt + 2.0) * 12_000.0 / istep as f32) as usize
    } else {
        ((txt + 1.0) * 12_000.0 / istep as f32) as usize
    };
    // `nsmo=int(0.5*mode_q65*mode_q65); if(nsmo.lt.1) nsmo=1`, then
    // `if(nsmo.le.1) nsmo=0` inside the loop.
    let mut nsmo = (0.5 * (mode_q65 * mode_q65) as f32) as usize;
    if nsmo <= 1 {
        nsmo = 0;
    }
    let mut data = vec![0.0f32; iz * jz];
    let mut fft = SymbolFft::new(nsps);
    let mut j = 1;
    while j <= jz {
        let i1 = slot_start + ((j - 1) * istep) as i64;
        let spec = fft.with_input(|b| {
            for (k, slot) in b.iter_mut().enumerate() {
                let n = i1 + k as i64;
                let x = if n >= 0 && (n as usize) < audio.len() {
                    audio[n as usize]
                } else {
                    0.0
                };
                *slot = num_complex::Complex32::new(x, 0.0);
            }
        });
        let row = &mut data[(j - 1) * iz..j * iz];
        for (i, v) in row.iter_mut().enumerate() {
            *v = spec[i + 1].norm_sqr();
        }
        for _ in 0..nsmo {
            smo121(row);
        }
        if j >= 3 {
            for i in 0..iz {
                data[(j - 2) * iz + i] = 0.5 * (data[(j - 3) * iz + i] + data[(j - 1) * iz + i]);
            }
        }
        j += 2;
    }
    S1 { iz, jz, data }
}

/// `q65_ccf_85`'s choice: `(ipk, jpk, imsg_best, better)`.
struct Ccf85 {
    ipk: i64,
    jpk: i64,
    better: f32,
}

#[allow(clippy::too_many_arguments)]
fn ccf_85(
    s1: &S1,
    codewords: &[[i32; 63]],
    i0: i64,
    ia: i64,
    ia2: i64,
    iia: i64,
    lag1: i64,
    lag2: i64,
    j0: i64,
    mode_q65: i64,
) -> Option<Ccf85> {
    let nlag = (lag2 - lag1 + 1).max(0) as usize;
    let ni = (2 * ia2 + 1) as usize;
    let mut ccf = vec![0.0f32; ni * nlag];
    let mut ccf_best = 0.0f32;
    let mut best_at: Option<(i64, i64, usize)> = None;
    let mut best = vec![0.0f32; codewords.len()];
    let mut itone = [0i64; 85];
    for (imsg, cw) in codewords.iter().enumerate() {
        let mut sync = Q65_SYNC_POSITIONS.iter().peekable();
        let mut k = 0;
        for (jsym, t) in itone.iter_mut().enumerate() {
            if sync.peek().is_some_and(|&&p| p as usize == jsym) {
                sync.next();
                *t = 0;
            } else {
                *t = cw[k] as i64 + 1;
                k += 1;
            }
        }
        ccf.iter_mut().for_each(|v| *v = 0.0);
        for (il, lag) in (lag1..=lag2).enumerate() {
            for (k, &t) in itone.iter().enumerate() {
                let j = j0 + (NSTEP as i64) * k as i64 + 1 + lag;
                if j < 1 || j > s1.jz as i64 {
                    continue;
                }
                for i in -ia2..=ia2 {
                    let ii = i0 + mode_q65 * t + i;
                    if ii >= iia && ii <= s1.iz as i64 && ii >= 1 {
                        ccf[il * ni + (i + ia2) as usize] += s1.at(ii as usize, j as usize);
                    }
                }
            }
        }
        // `maxval(ccf(-ia:ia,:))` and its `maxloc`: first in column order.
        let mut ccfmax = 0.0f32;
        let mut loc: Option<(i64, i64)> = None;
        for (il, lag) in (lag1..=lag2).enumerate() {
            for i in -ia..=ia {
                let v = ccf[il * ni + (i + ia2) as usize];
                if loc.is_none() || v > ccfmax {
                    ccfmax = v;
                    loc = Some((i, lag));
                }
            }
        }
        if ccfmax > ccf_best
            && let Some((i, lag)) = loc
        {
            ccf_best = ccfmax;
            best_at = Some((i, lag, imsg));
        }
        best[imsg] = ccfmax;
    }
    let (ipk, jpk, imsg_best) = best_at?;
    best[imsg_best] = 0.0;
    let runner_up = best.iter().cloned().fold(0.0f32, f32::max);
    Some(Ccf85 {
        ipk,
        jpk,
        better: ccf_best / runner_up,
    })
}

/// `q65_s1_to_s3` then `q65_bzap`: the 63 data symbols' `LL` bins from
/// `i1 = i0 + ipk - 64 + mode_q65`, laid out as the wide energies the
/// fast-fading intrinsics read.
fn s1_to_s3(s1: &S1, i0: i64, ipk: i64, jpk: i64, j0: i64, mode_q65: i64) -> Vec<f32> {
    let ll = (64 * (2 + mode_q65)) as usize;
    let mut s3 = vec![0.0f32; ll * 63];
    let i1 = i0 + ipk - 64 + mode_q65;
    let i2 = i1 + ll as i64 - 1;
    if i1 >= 1 && i2 <= s1.iz as i64 {
        let mut j = j0 + jpk - 7;
        let mut n = 0usize;
        let mut sync = Q65_SYNC_POSITIONS.iter().peekable();
        for k in 0..85u32 {
            j += NSTEP as i64;
            if sync.peek().is_some_and(|&&p| p == k) {
                sync.next();
                continue;
            }
            if j >= 1 && j <= s1.jz as i64 {
                for b in 0..ll {
                    s3[n * ll + b] = s1.at(i1 as usize + b, j as usize);
                }
            }
            n += 1;
        }
    }
    // `q65_bzap`: a bin that holds the peak of more than NBZAP symbols
    // is a birdie; flatten it to 1.0.
    let mut hist = vec![0usize; ll];
    for n in 0..63 {
        let row = &s3[n * ll..(n + 1) * ll];
        let mut pk = 0;
        for (b, &v) in row.iter().enumerate() {
            if v > row[pk] {
                pk = b;
            }
        }
        hist[pk] += 1;
    }
    if hist.iter().any(|&h| h > NBZAP) {
        for (b, &h) in hist.iter().enumerate() {
            if h > NBZAP {
                for n in 0..63 {
                    s3[n * ll + b] = 1.0;
                }
            }
        }
    }
    s3
}

/// The q3 decode at `params.rx_freq_hz`. `None` when the 85-symbol sync
/// does not single out a message, or no `b90` gets a list decode over
/// the thresholds.
pub(crate) fn decode_q3_for<P: ModulationParams>(
    audio: &[f32],
    sample_rate: u32,
    params: Q3Params,
    codewords: &[[i32; 63]],
    ctx: &DecodeContext,
) -> Option<Q65Result> {
    if codewords.is_empty() {
        return None;
    }
    let nsps = (sample_rate as f32 * P::SYMBOL_DT).round() as usize;
    let submode = submode_index_from_params::<P>();
    let mode_q65 = 1i64 << submode;
    let df = sample_rate as f32 / nsps as f32;
    let dtstep = nsps as f32 / (NSTEP as f32 * sample_rate as f32);
    // `lag1=-1.0/dtstep`, `lag2=<late>/dtstep + 0.9999`, integer assignment.
    let lag1 = (-1.0 / dtstep) as i64;
    let lag2 = (params.late_sec / dtstep + 0.9999) as i64;
    let j0 = if nsps >= 7200 {
        (1.0 / dtstep) as i64
    } else {
        (0.5 / dtstep) as i64
    };
    let ia = (params.ftol_hz / df) as i64;
    let ia2 = ia.max(10 * mode_q65).max((100.0 / df).round() as i64);
    let iia = (200.0 / df) as i64;
    let i0 = (params.rx_freq_hz / df).round() as i64;

    let mut s1 = symspec::<P>(audio, sample_rate, params.slot_start);
    if params.drift_hz != 0.0 {
        // `s1w(w3f,w3t)=s1(mm,w3t)`, `mm=w3f+nint(drift*w3t/(jz*df))`, where
        // `mm` is in range; `s1w=s1` elsewhere.
        let src = s1.data.clone();
        let (iz, jz) = (s1.iz, s1.jz);
        for w3t in 1..=jz {
            let off = (params.drift_hz * w3t as f32 / (jz as f32 * df)).round() as i64;
            for w3f in 1..=iz {
                let mm = w3f as i64 + off;
                if mm >= 1 && mm <= iz as i64 {
                    s1.data[(w3t - 1) * iz + (w3f - 1)] = src[(w3t - 1) * iz + (mm as usize - 1)];
                }
            }
        }
    }
    let c = ccf_85(&s1, codewords, i0, ia, ia2, iia, lag1, lag2, j0, mode_q65)?;
    if !(c.better >= 1.10 || mode_q65 >= 8) {
        return None;
    }
    let s3 = s1_to_s3(&s1, i0, c.ipk, c.jpk, j0, mode_q65);

    let baud = 1.0 / P::SYMBOL_DT;
    let ibwa = ibwa_for_submode(submode);
    let ibwb = (ibwa + 6).min(15);
    let codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut intrinsics = vec![0.0f32; 64 * 63];
    let es_no = default_es_no_metric();
    for ibw in ibwa..=ibwb {
        let b90_ts = 1.72f32.powi(ibw) / baud;
        let _ = intrinsics_fast_fading(
            &QRA15_65_64_IRR_E23,
            &mut intrinsics,
            &s3,
            submode,
            b90_ts,
            FadingModel::Lorentzian,
            es_no,
        );
        let Some((idx, info, plog)) = codec.decode_with_codeword_list_llh(&intrinsics, codewords)
        else {
            continue;
        };
        // `if(sum(dat4).le.0) irc=-2`; `if(irc.ge.0 .and. plog.gt.PLOG_MIN)`.
        if info.iter().sum::<i32>() <= 0 || plog <= PLOG_MIN {
            continue;
        }
        // `f0=nfqso+ipk*df`, `xdt=jpk*dtstep` from the nominal start.
        let f0 = params.rx_freq_hz + c.ipk as f32 * df;
        let start = params.slot_start + (j0 + c.jpk) * (nsps / NSTEP) as i64;
        let start_sample = start.max(0) as usize;
        return finish::<P>(
            &info,
            &codewords[idx],
            0,
            Energies::Wide(&s3),
            SnrAudio::Slot(audio),
            sample_rate,
            start_sample,
            f0,
            ctx,
        );
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smo121_matches_the_fortran() {
        let mut x = [1.0, 4.0, 2.0, 8.0];
        smo121(&mut x);
        assert_eq!(x, [1.0, 2.75, 4.0, 8.0]);
    }
}
