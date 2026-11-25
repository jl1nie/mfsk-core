// SPDX-License-Identifier: GPL-3.0-only
//! **Experimental** (`docs/notes/MIDRANGE_EMBEDDED_EVAL.md`): the whole band as one complex baseband, built in
//! a single forward pass over the audio, from which every candidate's 200 Hz baseband (`cd0`) is cut.
//!
//! `cd0_fir` makes a candidate's `cd0` from the 12 kHz audio: mix to DC, 12 kHz -> 200 Hz in
//! three stages, about 3.0 M multiply-adds each, and one read of the audio per group of candidates. Here the
//! audio is read **once**: it is mixed to the middle of the band (1600 Hz) and decimated to a complex
//! [`BB_RATE_HZ`] (3 kHz, so the band 100-3100 Hz is critically sampled), kept as `i16` pairs (180 KB for a
//! slot). A candidate's `cd0` is then a shift by `freq - 1600` Hz and a 15:1 decimation (3:1, 5:1) of that
//! baseband, about a third of the multiply-adds, and no audio.
//!
//! The grid is the slot's, as `cd0_fir`'s: baseband sample `k` is centred on input sample `4 k` and `cd0[i]` on
//! `60 i` = baseband sample `15 i`, so the two are interchangeable downstream (`fine_refine_3stage`,
//! `fill_from_cd0`). The band edges are the cost: the first stage's transition (1400-1600 Hz either side of
//! the centre) puts the last ~100 Hz of each end at less than full gain, and the audio's own band is 0-6 kHz.
//!
//! Not WSJT-X: `ft8_downsample.f90` makes each `cd0` from one big FFT of the slot.

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::cd0_fir::{Cd0, Cd0Filters, Stage};
use super::source::AudioSource;
use super::spectrogram::{SpecCell, Spectrogram};
use super::types::SAMPLE_RATE_HZ;
use crate::engine::dsp::fir_decimate::design_lowpass_kaiser;
use crate::engine::sync::SyncCandidate;
use crate::ft8::decode::{DecodeDepth, DecodeResult};
use crate::ft8::params::DEFAULT_BP_MAX_ITER;
use crate::ft8::params::NSPS;
use crate::ft8::refine_fine::CD0_LEN;
use num_complex::Complex;

/// The band baseband's complex sample rate.
pub const BB_RATE_HZ: usize = 3000;
/// What the band is mixed around. The band `1600 +- 1500` Hz holds FT8's 200-3000 Hz and the tones above it.
pub const BB_CENTER_HZ: f32 = 1600.0;
/// Input samples per baseband sample.
const DB: usize = SAMPLE_RATE_HZ as usize / BB_RATE_HZ;
/// Stage 1, 12 kHz -> 6 kHz: cutoff 3000 Hz (pass 1500, what folds onto it stops from 4500), 17 taps.
const NA: usize = 17;
/// Stage 2, 6 kHz -> 3 kHz: cutoff 1500 Hz, pass 1400, stop 1600: a Kaiser filter's transition is about
/// `(A - 8) / (2.285 (N - 1))` rad (A = 63 dB at beta 6), 0.209 rad = 200 Hz at 6 kHz needs 117 taps.
const NB: usize = 117;
/// Group delay of both stages, in input samples (integer: both filters have an odd length).
const DELAY_IN: i64 = ((NA - 1) / 2 + 2 * (NB - 1) / 2) as i64;
/// Half the width of `subtract`'s amplitude filter, baseband samples: WSJT-X's `NFILT / 2` = 2000 at 12 kHz.
pub const SUB_H: usize = 500;
/// A candidate's stage 1, 3 kHz -> 1 kHz: 500 Hz cutoff, stops from 945 Hz (1000 - 55); 15 taps.
const NC: usize = 15;
/// A candidate's stage 2, 1 kHz -> 200 Hz: as `cd0_fir`'s third stage.
const ND: usize = 43;
/// Group delay of a candidate's two stages, in baseband samples.
const DELAY_BB: i64 = ((NC - 1) / 2 + 3 * (ND - 1) / 2) as i64;
/// Baseband samples per `cd0` sample.
const DC: usize = 15;

/// From the baseband frame's `|X|^2` to the audio spectrogram's cell.
struct CellScale {
    k: f32,
}

impl CellScale {
    /// f32 build: the audio cell is `|X|^2 / 300^2` of the `i16` frame's real FFT. Fixed-point build: the
    /// `i16` FFT of the frame scaled up by the slot's gain (`gain_for_peak`), divided by `NFFT_SPEC`, squared
    /// and `>> FP_SPEC_SHIFT`.
    #[cfg(not(feature = "fixed-point"))]
    fn new(_peak: i32) -> Self {
        Self {
            k: 16.0 / (300.0 * 300.0),
        }
    }

    #[cfg(feature = "fixed-point")]
    fn new(peak: i32) -> Self {
        let g = (1u32 << super::spectrogram::gain_for_peak(peak)) as f32;
        Self {
            k: 16.0 * g * g
                / (FIXED_FFT_SCALE
                    * FIXED_FFT_SCALE
                    * (1u32 << super::spectrogram::FP_SPEC_SHIFT) as f32),
        }
    }

    #[cfg(not(feature = "fixed-point"))]
    fn of(&self, p: f32) -> SpecCell {
        p * self.k
    }

    #[cfg(feature = "fixed-point")]
    fn of(&self, p: f32) -> SpecCell {
        (p * self.k).min(u16::MAX as f32) as u16
    }
}

/// The total scale-down of the 3840-point `i16` spectrogram FFT: esp-dsp's 256-point `sc16` kernel halves at
/// each of its 8 stages, the 15-point step of the mixed-radix plan (`Plan3840Sc16`) does not scale. Measured:
/// with it the band spectrogram's strongest cell is the audio's to 3 % (`the_band_spectrogram_is_the_audios`).
#[cfg(feature = "fixed-point")]
const FIXED_FFT_SCALE: f32 = 256.0;

/// The filters of the band baseband and of the cut, built once per decode.
pub struct BandFilters {
    a: Vec<f32>,
    b: Vec<f32>,
    c: Vec<f32>,
    d: Vec<f32>,
    /// `cd0_fir`'s DFT table and `fill_from_cd0`.
    cd0: Cd0Filters,
}

impl Default for BandFilters {
    fn default() -> Self {
        Self::new()
    }
}

impl BandFilters {
    pub fn new() -> Self {
        let k = |n: usize, fc: f64| design_lowpass_kaiser(n, fc, 6.0);
        Self {
            a: k(NA, 3000.0 / 12_000.0),
            b: k(NB, 1500.0 / 6_000.0),
            c: k(NC, 500.0 / 3_000.0),
            d: k(ND, 102.0 / 1_000.0),
            cd0: Cd0Filters::new(),
        }
    }
}

/// The band as a complex baseband at [`BB_RATE_HZ`]: sample `k` is centred on input sample `4 k`, tone
/// frequencies are offsets from [`BB_CENTER_HZ`].
pub struct BandBaseband {
    pub data: Vec<[i16; 2]>,
    /// The largest `|sample|` of the slot's first `NMAX` input samples, tracked while it was built: all the
    /// fixed-point spectrogram's gain needs from the whole slot, so the coarse search reads nothing.
    pub peak: i32,
}

impl BandBaseband {
    /// Build it from one forward read of `src`.
    pub fn from_source<A: AudioSource>(f: &BandFilters, src: &mut A) -> Self {
        let mut s = BandStream::new(f);
        let mut at = 0i64;
        let mut block = [0i16; 1024];
        while at < src.len() as i64 {
            src.read(at, &mut block);
            let n = block.len().min((src.len() as i64 - at) as usize);
            s.feed(&block[..n]);
            at += n as i64;
        }
        s.finish()
    }

    /// Sample `k` as `f32` pairs; zero outside the slot, as the audio is.
    #[inline]
    fn get(&self, k: i64) -> (f32, f32) {
        if k < 0 || k as usize >= self.data.len() {
            (0.0, 0.0)
        } else {
            let v = self.data[k as usize];
            (v[0] as f32, v[1] as f32)
        }
    }

    /// Subtract the signal of `tones` (tone 0 at `freq_hz`, frame start `0.5 + dt_sec` s) from the baseband:
    /// `subtractft8.f90`'s algorithm on the baseband (`docs/notes/MIDRANGE_EMBEDDED_EVAL.md` §4). The GFSK
    /// reference `r` at the baseband's rate comes from [`GfskStream`] (every fourth sample of the 12 kHz
    /// waveform, carrier `freq_hz - 1600`); its complex amplitude is `cos^2`-lowpassed `bb * conj(r)` (width
    /// `NFILT = 4000` at 12 kHz, 1000 here: [`SUB_H`] each side), with the truncated window's weight
    /// renormalised at the ends (`endcorrection`); then `bb -= amp * r`. There is no factor 2 of the real
    /// signal's: a baseband holds half of it. Streaming: the window slides in O(1) per sample (the `cos^2`
    /// is a constant and two phasors, `z0`, `zp`, `zm` below) and only `2 * SUB_H + 2` samples of
    /// `bb * conj(r)` are held, so the cost is two references and a few complex multiplies per sample,
    /// against `subtract_signal_lpf`'s ~600 M multiply-adds per signal at 12 kHz. Returns the largest
    /// `|amp|` (a baseband amplitude: half the audio's).
    pub fn subtract(&mut self, tones: &[u8], freq_hz: f32, dt_sec: f32) -> f32 {
        use crate::engine::dsp::gfsk::{GfskCfg, GfskStream};
        let cfg = GfskCfg {
            sample_rate: SAMPLE_RATE_HZ,
            samples_per_symbol: NSPS,
            bt: 2.0,
            hmod: 1.0,
            ramp_samples: NSPS / 8,
        };
        let nwave = (tones.len() * NSPS) as i64;
        // first input sample of the waveform, and the baseband samples inside it
        let s0 = ((0.5 + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
        let kfirst = (s0 + DB as i64 - 1).div_euclid(DB as i64).max(0);
        let klast = (s0 + nwave - 1)
            .div_euclid(DB as i64)
            .min(self.data.len() as i64 - 1);
        if klast < kfirst {
            return 0.0;
        }
        let n = (klast - kfirst + 1) as usize;
        let base = kfirst as usize;
        let j0 = (DB as i64 * kfirst - s0) as usize;
        let mk = || {
            let mut st = GfskStream::new(tones, freq_hz - BB_CENTER_HZ, &cfg);
            st.skip(j0);
            st
        };
        let (mut ra, mut rb) = (mk(), mk());

        const H: usize = SUB_H;
        const RL: usize = 2 * H + 2;
        let th = core::f64::consts::PI / H as f64;
        let (e1, e1c) = ((th.cos(), th.sin()), (th.cos(), -th.sin())); // e^{+j th}, e^{-j th}
        let mul = |a: (f64, f64), b: (f64, f64)| (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0);
        let add = |a: (f64, f64), b: (f64, f64)| (a.0 + b.0, a.1 + b.1);
        let sub = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0, a.1 - b.1);

        let mut ring = alloc::vec![(0.0f32, 0.0f32); RL];
        // c(t) = bb(t) conj(r(t)), pulled from reference `ra` in order; zero past the signal
        let mut next_in = 0usize;
        let mut c_in =
            |this: &Self, ra: &mut GfskStream, ring: &mut Vec<(f32, f32)>| -> (f64, f64) {
                if next_in >= n {
                    next_in += 1;
                    return (0.0, 0.0);
                }
                let (rr, ri) = ra.peek_complex();
                ra.skip(DB);
                let (xr, xi) = this.get((base + next_in) as i64);
                let c = (xr * rr + xi * ri, xi * rr - xr * ri);
                ring[next_in % RL] = c;
                next_in += 1;
                (c.0 as f64, c.1 as f64)
            };

        // sums over the window [t - H, t + H] of c, c e^{+j th m}, c e^{-j th m}, and of the same weights
        let (mut z0, mut zp, mut zm) = ((0.0f64, 0.0f64), (0.0, 0.0), (0.0, 0.0));
        let (mut w0, mut wp, mut wm) = (0.0f64, (0.0f64, 0.0f64), (0.0f64, 0.0f64));
        for m in 0..=H {
            let c = c_in(self, &mut ra, &mut ring);
            let ind = if m < n { 1.0 } else { 0.0 };
            let (ep, em) = ((th * m as f64).cos(), (th * m as f64).sin());
            let (up, um) = ((ep, em), (ep, -em));
            z0 = add(z0, c);
            zp = add(zp, mul(up, c));
            zm = add(zm, mul(um, c));
            w0 += ind;
            wp = add(wp, (up.0 * ind, up.1 * ind));
            wm = add(wm, (um.0 * ind, um.1 * ind));
        }

        let mut peak = 0.0f32;
        for t in 0..n {
            // amplitude at t: the cos^2 window = 1/2 + 1/4 (e^{+j th m} + e^{-j th m})
            let num = (
                0.5 * z0.0 + 0.25 * (zp.0 + zm.0),
                0.5 * z0.1 + 0.25 * (zp.1 + zm.1),
            );
            let den = 0.5 * w0 + 0.25 * (wp.0 + wm.0);
            let amp = if den > 1e-9 {
                (num.0 / den, num.1 / den)
            } else {
                (0.0, 0.0)
            };
            let (rr, ri) = rb.peek_complex();
            rb.skip(DB);
            let (ar, ai) = (amp.0 as f32, amp.1 as f32);
            peak = peak.max((ar * ar + ai * ai).sqrt());
            let (sr, si) = (ar * rr - ai * ri, ar * ri + ai * rr);
            let v = &mut self.data[base + t];
            let q = |x: f32| x.round().clamp(-32768.0, 32767.0) as i16;
            v[0] = q(v[0] as f32 - sr);
            v[1] = q(v[1] as f32 - si);

            // slide the window to t + 1: c[t - H] leaves, c[t + H + 1] enters
            let (leave, ind_l) = if t >= H {
                let c = ring[(t - H) % RL];
                ((c.0 as f64, c.1 as f64), 1.0)
            } else {
                ((0.0, 0.0), 0.0)
            };
            let ind_e = if t + H + 1 < n { 1.0 } else { 0.0 };
            let enter = c_in(self, &mut ra, &mut ring);
            z0 = add(sub(z0, leave), enter);
            // Z+(t+1) = e^{-j th} [Z+(t) + c_leave - e^{+j th} c_enter], Z- the conjugate rotation
            zp = mul(e1c, sub(add(zp, leave), mul(e1, enter)));
            zm = mul(e1, sub(add(zm, leave), mul(e1c, enter)));
            w0 += ind_e - ind_l;
            wp = mul(
                e1c,
                sub(add(wp, (ind_l, 0.0)), (e1.0 * ind_e, e1.1 * ind_e)),
            );
            wm = mul(
                e1,
                sub(add(wm, (ind_l, 0.0)), (e1c.0 * ind_e, e1c.1 * ind_e)),
            );
        }
        peak
    }

    /// The spectrogram of the bins `lo..hi` of `NFFT_SPEC` (3.125 Hz apart), on the audio spectrogram's frame
    /// grid, from this baseband: each frame's `NSPS / 4` baseband samples zero-padded to `NFFT_SPEC / 4` and
    /// transformed (complex), the bin `i` of the audio being the baseband's bin `i - 512` (1600 Hz = bin
    /// 512). The cells are the audio spectrogram's: the baseband's power is 1/16 of the audio frame's (a
    /// quarter of the samples, a quarter of the amplitude of a mixed tone), rescaled so that
    /// `coarse_sync`'s thresholds and the fixed-point build's `u16` cells mean what they do there.
    pub(super) fn spectrogram_bins(&self, lo: usize, hi: usize) -> Spectrogram {
        use super::types::{NFFT_SPEC, NSTEP};
        use crate::engine::fft::default_planner;
        let q = 4usize;
        assert!(lo <= hi && hi <= NFFT_SPEC / 2 + 1, "bins out of range");
        let n_freq = hi - lo;
        let n_time = crate::ft8::params::NMAX / NSTEP - 3;
        let (nfft, nsps) = (NFFT_SPEC / q, NSPS / q);
        let centre_bin = (BB_CENTER_HZ / (SAMPLE_RATE_HZ / NFFT_SPEC as f32)).round() as i64;
        let fft = default_planner().plan_forward(nfft);
        let mut buf = alloc::vec![Complex::new(0.0f32, 0.0); nfft];
        let mut data: Vec<SpecCell> = alloc::vec![SpecCell::default(); n_freq * n_time];
        let cell = CellScale::new(self.peak);
        for j in 0..n_time {
            let at = (j * NSTEP / q) as i64;
            for (k, c) in buf.iter_mut().enumerate() {
                *c = if k < nsps {
                    let (re, im) = self.get(at + k as i64);
                    Complex::new(re, im)
                } else {
                    Complex::new(0.0, 0.0)
                };
            }
            fft.process(&mut buf);
            let row = j * n_freq;
            for i in lo..hi {
                let m = (i as i64 - centre_bin).rem_euclid(nfft as i64) as usize;
                data[row + i - lo] = cell.of(buf[m].norm_sqr());
            }
        }
        Spectrogram {
            n_freq,
            n_time,
            data,
        }
    }

    /// The 200 Hz baseband of the signal whose tone 0 is at `freq_hz`, on `cd0_fir`'s grid.
    pub(super) fn cd0(&self, f: &BandFilters, freq_hz: f32) -> Cd0 {
        let w = core::f64::consts::TAU * (freq_hz - BB_CENTER_HZ) as f64 / BB_RATE_HZ as f64;
        let (cw, sw) = (w.cos() as f32, w.sin() as f32);
        let mut c = Stage::new(&f.c, 3);
        let mut d = Stage::new(&f.d, 5);
        let mut out: Vec<Complex<f32>> = Vec::with_capacity(CD0_LEN);
        let (mut pr, mut pi) = (1.0f32, 0.0f32);
        let mut k = -DELAY_BB;
        while out.len() < CD0_LEN {
            if k.rem_euclid(64) == 0 {
                let ph = w * k as f64;
                pr = ph.cos() as f32;
                pi = -(ph.sin() as f32);
            }
            let (xr, xi) = self.get(k);
            let (mr, mi) = (xr * pr - xi * pi, xr * pi + xi * pr);
            let (nr, ni) = (pr * cw + pi * sw, pi * cw - pr * sw);
            pr = nr;
            pi = ni;
            k += 1;
            if let Some((r1, i1)) = c.push(mr, mi)
                && let Some((r2, i2)) = d.push(r1, i1)
            {
                out.push(Complex::new(r2, i2));
            }
        }
        debug_assert_eq!(DC * CD0_LEN, 48_000);
        Cd0 { freq_hz, cd0: out }
    }
}

/// The band baseband, built as the audio arrives.
pub struct BandStream<'a> {
    f: &'a BandFilters,
    a: Stage<'a>,
    b: Stage<'a>,
    /// Input samples fed so far.
    n: i64,
    cw: f32,
    sw: f32,
    pr: f32,
    pi: f32,
    out: Vec<[i16; 2]>,
    peak: i32,
}

impl<'a> BandStream<'a> {
    pub fn new(f: &'a BandFilters) -> Self {
        let w = core::f64::consts::TAU * BB_CENTER_HZ as f64 / SAMPLE_RATE_HZ as f64;
        let mut s = Self {
            f,
            a: Stage::new(&f.a, 2),
            b: Stage::new(&f.b, 2),
            n: 0,
            cw: w.cos() as f32,
            sw: w.sin() as f32,
            pr: 1.0,
            pi: 0.0,
            out: Vec::with_capacity(crate::ft8::params::NMAX / DB + 8),
            peak: 1,
        };
        // the filters' history before sample 0 is silence: run it through, so output 0 is centred on input 0
        for _ in 0..DELAY_IN {
            s.push_filtered(0.0, 0.0);
        }
        s
    }

    fn push_filtered(&mut self, re: f32, im: f32) {
        if let Some((r1, i1)) = self.a.push(re, im)
            && let Some((r2, i2)) = self.b.push(r1, i1)
        {
            let q = |v: f32| v.round().clamp(-32768.0, 32767.0) as i16;
            self.out.push([q(r2), q(i2)]);
        }
    }

    /// Feed the next samples of the slot, in order.
    pub fn feed(&mut self, samples: &[i16]) {
        for &x in samples {
            // the phasor of every 4096th sample exactly, rotated in between
            if self.n % 4096 == 0 {
                let ph = core::f64::consts::TAU * BB_CENTER_HZ as f64 / SAMPLE_RATE_HZ as f64
                    * self.n as f64;
                self.pr = ph.cos() as f32;
                self.pi = -(ph.sin() as f32);
            }
            if (self.n as usize) < crate::ft8::params::NMAX {
                self.peak = self.peak.max((x as i32).abs());
            }
            let x = x as f32;
            let (mr, mi) = (x * self.pr, x * self.pi);
            let (nr, ni) = (
                self.pr * self.cw + self.pi * self.sw,
                self.pi * self.cw - self.pr * self.sw,
            );
            self.pr = nr;
            self.pi = ni;
            self.n += 1;
            self.push_filtered(mr, mi);
        }
    }

    /// The tail: silence through the filters until the last sample's output is out.
    pub fn finish(mut self) -> BandBaseband {
        let want = (self.n as usize).div_ceil(DB);
        let _ = self.f;
        while self.out.len() < want {
            self.push_filtered(0.0, 0.0);
        }
        BandBaseband {
            data: self.out,
            peak: self.peak,
        }
    }
}

/// Decode the candidates `gated` from the band baseband (their `cd0`s cut from it, nothing read from the
/// audio): [`decode_block_low_mem_src`](super::decode_block_low_mem_src)'s candidate loop.
pub fn decode_band(
    bb: &BandBaseband,
    f: &BandFilters,
    gated: &[(SyncCandidate, u8)],
    depth: DecodeDepth,
    group: usize,
) -> Vec<DecodeResult> {
    super::process_candidates::decode_from_basebands(
        gated,
        &f.cd0,
        group,
        depth,
        DEFAULT_BP_MAX_ITER,
        |freqs| freqs.iter().map(|&fr| bb.cd0(f, fr)).collect(),
    )
}

/// What [`decode_band_sic`] did.
#[derive(Debug, Default)]
pub struct SicReport {
    /// New decodes per pass.
    pub per_pass: Vec<usize>,
    /// Time spent in `subtract`, ms.
    pub subtract_ms: f64,
}

/// `passes` rounds of coarse search and decode on the band baseband, each on what is left after subtracting
/// the messages the rounds before it decoded ([`BandBaseband::subtract`]): `ft8_decode.f90`'s passes with
/// nothing re-read and no second copy of the audio. A message found twice is kept once (the first).
#[allow(clippy::too_many_arguments)]
pub fn decode_band_sic(
    bb: &mut BandBaseband,
    f: &BandFilters,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    depth: DecodeDepth,
    max_cand: usize,
    bands: usize,
    group: usize,
    passes: usize,
    now_ms: &mut dyn FnMut() -> f64,
) -> (Vec<DecodeResult>, SicReport) {
    let mut all: Vec<DecodeResult> = Vec::new();
    let mut rep = SicReport::default();
    for _ in 0..passes {
        let gated =
            super::coarse_sync_gated_band(bb, freq_min, freq_max, sync_min, max_cand, bands);
        let found = decode_band(bb, f, &gated, depth, group);
        let fresh: Vec<DecodeResult> = found
            .into_iter()
            .filter(|r| !all.iter().any(|a| a.message77() == r.message77()))
            .collect();
        rep.per_pass.push(fresh.len());
        let t0 = now_ms();
        for r in &fresh {
            let tones = crate::engine::tx::message_to_tones::<crate::ft8::Ft8>(r.message77());
            bb.subtract(&tones, r.freq_hz, r.dt_sec);
        }
        rep.subtract_ms += now_ms() - t0;
        let none = fresh.is_empty();
        all.extend(fresh);
        if none {
            break;
        }
    }
    (all, rep)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::tx::{message_to_tones, synthesize_i16};
    use crate::ft8::Ft8;
    use crate::ft8::decode_block::cd0_fir::build_group;
    use crate::ft8::decode_block::source::SliceSource;
    use crate::msg::wsjt77::pack77;

    fn slot_with(freq: f32, dt: f32) -> Vec<i16> {
        let m77 = pack77("CQ", "K1ABC", "FN42").unwrap();
        let tones = message_to_tones::<Ft8>(&m77);
        let pcm = synthesize_i16::<Ft8>(&tones, 12_000, freq, 8000);
        let mut slot = alloc::vec![0i16; 180_000];
        let at = ((0.5 + dt) * 12_000.0).round() as usize;
        slot[at..at + pcm.len()].copy_from_slice(&pcm);
        slot
    }

    /// The `cd0` cut from the band baseband is the one built from the audio, on a clean signal: the same
    /// samples to within 1 % (0.2 % measured: quantisation and the filters' difference), over the symbols' span. A grid off by one
    /// baseband sample (4 input samples), a centre off by a Hz or a scale that is not 1 moves it far more.
    #[test]
    fn cd0_from_the_band_is_the_direct_cd0() {
        let f = BandFilters::new();
        for (freq, dt) in [(217.3f32, -0.45f32), (1234.567, 0.31), (2900.2, 0.74)] {
            let slot = slot_with(freq, dt);
            let bb = BandBaseband::from_source(&f, &mut SliceSource(&slot));
            let a = bb.cd0(&f, freq);
            let d = build_group(&f.cd0, &mut SliceSource(&slot), &[freq])
                .pop()
                .unwrap();
            // the signal spans cd0 samples (0.5 + dt) * 200 .. + 79 * 32
            let lo = ((0.5 + dt) * 200.0) as usize + 4;
            let hi = lo + 79 * 32 - 8;
            let (mut num, mut den) = (0.0f64, 0.0f64);
            for i in lo..hi {
                let (x, y) = (a.cd0[i], d.cd0[i]);
                num += ((x - y).norm_sqr()) as f64;
                den += y.norm_sqr() as f64;
            }
            let err = (num / den).sqrt();
            assert!(err < 0.01, "freq {freq}: relative error {err}");
        }
    }

    /// A busy slot: `n` signals at spread frequencies and times over noise.
    fn busy_slot() -> Vec<i16> {
        let mut slot = alloc::vec![0i16; 180_000];
        let mut rng = 12345u32;
        let mut next = || {
            rng = rng.wrapping_mul(1664525).wrapping_add(1013904223);
            ((rng >> 16) as f32 / 32768.0) - 1.0
        };
        for (n, (freq, dt)) in [
            (300.0f32, 0.1f32),
            (700.0, -0.3),
            (1100.5, 0.5),
            (1500.0, 0.0),
            (1800.2, 0.2),
            (2200.0, -0.1),
            (2600.7, 0.4),
            (2900.0, 0.0),
        ]
        .iter()
        .enumerate()
        {
            let s = slot_with(*freq, *dt);
            let g = 0.12 + 0.03 * n as f32;
            for (o, x) in slot.iter_mut().zip(&s) {
                *o = o.saturating_add((*x as f32 * g) as i16);
            }
        }
        for o in slot.iter_mut() {
            *o = o.saturating_add((next() * 500.0) as i16);
        }
        slot
    }

    /// The spectrogram made from the band baseband is the audio's, cell for cell, where there is a signal: on
    /// the cells above 100x the median power the ratio is within 10 % on the main lobes (above 3000x) and 35 % on
    /// the side lobes of the rectangular window (whose frame edges the filters round), and the noise power
    /// agrees to 10 %. A scale off by the 16 of the quarter-rate
    /// frame, a wrong centre bin or a frame grid off by one baseband sample moves it far more. Runs in the
    /// f32 and in the fixed-point build (whose cells are `u16`).
    #[test]
    fn the_band_spectrogram_is_the_audios() {
        use crate::ft8::decode_block::spectrogram::{SpecCell, compute_spectrogram_bins};
        let f = BandFilters::new();
        let slot = busy_slot();
        let bb = BandBaseband::from_source(&f, &mut SliceSource(&slot));
        let (lo, hi) = (64usize, 977usize);
        let a = compute_spectrogram_bins(&slot, lo, hi);
        let b = bb.spectrogram_bins(lo, hi);
        assert_eq!((a.n_freq, a.n_time), (b.n_freq, b.n_time));
        let as_f = |c: SpecCell| c as f64;
        let mut sorted: Vec<f64> = a.data.iter().map(|&c| as_f(c)).collect();
        sorted.sort_by(|x, y| x.total_cmp(y));
        let median = sorted[sorted.len() / 2].max(1e-9);
        // main-lobe cells (> 3000x the median), side-lobe cells (100-3000x), and the rest (noise)
        let (mut main_n, mut main_worst) = (0usize, 0.0f64);
        let (mut side_n, mut side_worst) = (0usize, 0.0f64);
        let (mut sum_a, mut sum_b) = (0.0f64, 0.0f64);
        for (x, y) in a.data.iter().zip(&b.data) {
            let (x, y) = (as_f(*x), as_f(*y));
            let e = (y / x.max(1e-9) - 1.0).abs();
            if x > 3000.0 * median {
                main_n += 1;
                main_worst = main_worst.max(e);
            } else if x > 100.0 * median {
                side_n += 1;
                side_worst = side_worst.max(e);
            } else {
                sum_a += x;
                sum_b += y;
            }
        }
        let noise_ratio = sum_b / sum_a;
        std::eprintln!(
            "main {main_n} cells worst {main_worst:.3}, side {side_n} cells worst {side_worst:.3}, noise mean ratio {noise_ratio:.3}"
        );
        assert!(main_n > 200, "the slot has signals: {main_n} cells");
        assert!(
            main_worst < 0.06,
            "main-lobe worst relative error {main_worst}"
        );
        assert!(
            side_worst < 0.35,
            "side-lobe worst relative error {side_worst}"
        );
        assert!(
            (noise_ratio - 1.0).abs() < 0.10,
            "noise power ratio {noise_ratio}"
        );
    }

    /// What the mutation checks perturb the reference's frequency by; 0 in the committed test.
    const SUB_TEST_DF: f32 = 0.0;

    fn tones_of(freq: f32) -> Vec<u8> {
        let _ = freq;
        let m77 = pack77("CQ", "K1ABC", "FN42").unwrap();
        message_to_tones::<Ft8>(&m77)
    }

    fn energy(bb: &BandBaseband) -> f64 {
        bb.data
            .iter()
            .map(|v| (v[0] as f64).powi(2) + (v[1] as f64).powi(2))
            .sum()
    }

    /// `subtract` on a clean, self-synthesised signal takes out as much as the audio-path subtraction
    /// (`subtract_tones_lpf`, `subtractft8.f90`'s) does: 40 dB of the energy, the rest being the ramps at the
    /// ends of the frame and the quarter-second amplitude filter, which cannot follow them. Across the band
    /// and at times that are not on the baseband's grid, within 1 dB of the audio path. A reference in the
    /// wrong place (a Hz, a few ms, another message) leaves far more.
    #[test]
    fn subtracting_a_clean_signal_takes_it_out() {
        use crate::engine::dsp::subtract::{GfskParams, SubtractCfg, subtract_tones_lpf};
        let cfg = SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 6.25,
            samples_per_symbol: 1920,
            base_offset_s: 0.5,
            gfsk: Some(GfskParams {
                bt: 2.0,
                hmod: 1.0,
                ramp_samples: 240,
                frame_ramp: false,
            }),
        };
        let f = BandFilters::new();
        for (freq, dt) in [(217.3f32, -0.45f32), (1234.567, 0.31), (2900.2, 0.74)] {
            let slot = slot_with(freq, dt);
            let tones = tones_of(freq);
            let mut au = slot.clone();
            let e0: f64 = au.iter().map(|&v| (v as f64).powi(2)).sum();
            subtract_tones_lpf(&mut au, &tones, freq, dt, &cfg, 2000, true);
            let audio_db =
                10.0 * (e0 / au.iter().map(|&v| (v as f64).powi(2)).sum::<f64>().max(1.0)).log10();

            let mut bb = BandBaseband::from_source(&f, &mut SliceSource(&slot));
            let before = energy(&bb);
            bb.subtract(&tones, freq + SUB_TEST_DF, dt);
            let drop_db = 10.0 * (before / energy(&bb).max(1.0)).log10();
            assert!(
                drop_db > audio_db - 1.0,
                "freq {freq}: {drop_db:.1} dB, the audio path's {audio_db:.1}"
            );
        }
    }

    /// The streaming `subtract` is the direct convolution: `subtractft8.f90`'s filter written out (a
    /// `cos^2` window of `2 SUB_H + 1` taps, the truncated window's weight renormalised) on the same
    /// reference gives the same baseband to within 2 LSB. A window recursion with the wrong sign, a leaving
    /// sample off by one or a weight that is not renormalised at the ends differs by far more on a signal
    /// that is not clean (this one has noise, so the amplitude is not constant).
    #[test]
    fn the_sliding_window_is_the_direct_convolution() {
        use crate::engine::dsp::gfsk::{GfskCfg, GfskStream};
        let f = BandFilters::new();
        let (freq, dt) = (1500.3f32, 0.2f32);
        let mut slot = busy_slot();
        for (o, x) in slot.iter_mut().zip(slot_with(freq, dt)) {
            *o = o.saturating_add(x / 2);
        }
        let mut a = BandBaseband::from_source(&f, &mut SliceSource(&slot));
        let b0 = BandBaseband {
            data: a.data.clone(),
            peak: a.peak,
        };
        let tones = tones_of(freq);
        a.subtract(&tones, freq, dt);

        // direct
        let cfg = GfskCfg {
            sample_rate: 12_000.0,
            samples_per_symbol: 1920,
            bt: 2.0,
            hmod: 1.0,
            ramp_samples: 240,
        };
        let s0 = ((0.5 + dt) * 12_000.0).round() as i64;
        let kfirst = (s0 + 3).div_euclid(4).max(0);
        let klast = (s0 + 79 * 1920 - 1)
            .div_euclid(4)
            .min(b0.data.len() as i64 - 1);
        let n = (klast - kfirst + 1) as usize;
        let mut st = GfskStream::new(&tones, freq - BB_CENTER_HZ, &cfg);
        st.skip((4 * kfirst - s0) as usize);
        let mut r = Vec::new();
        for _ in 0..n {
            r.push(st.peek_complex());
            st.skip(4);
        }
        let c: Vec<(f64, f64)> = (0..n)
            .map(|t| {
                let (xr, xi) = b0.get(kfirst + t as i64);
                let (rr, ri) = r[t];
                ((xr * rr + xi * ri) as f64, (xi * rr - xr * ri) as f64)
            })
            .collect();
        let h = SUB_H as i64;
        let mut worst = 0.0f64;
        for t in 0..n {
            let (mut sr, mut si, mut sw) = (0.0f64, 0.0f64, 0.0f64);
            for m in -h..=h {
                let j = t as i64 + m;
                if j < 0 || j >= n as i64 {
                    continue;
                }
                let w = (core::f64::consts::PI * m as f64 / (2.0 * h as f64))
                    .cos()
                    .powi(2);
                sr += w * c[j as usize].0;
                si += w * c[j as usize].1;
                sw += w;
            }
            let (ar, ai) = ((sr / sw) as f32, (si / sw) as f32);
            let (rr, ri) = r[t];
            let want = [
                (b0.data[kfirst as usize + t][0] as f32 - (ar * rr - ai * ri)).round(),
                (b0.data[kfirst as usize + t][1] as f32 - (ar * ri + ai * rr)).round(),
            ];
            let got = a.data[kfirst as usize + t];
            worst = worst
                .max((got[0] as f64 - want[0] as f64).abs())
                .max((got[1] as f64 - want[1] as f64).abs());
        }
        std::eprintln!("worst difference {worst} LSB");
        assert!(worst <= 2.0, "worst difference {worst} LSB");
    }

    /// The baseband does not depend on how the audio arrives: the slot fed whole, a sample at a time, in
    /// uneven chunks (1, 7, 480, 4097 samples, cycled) or read from a source gives the same baseband, to the
    /// bit. This is the input model of the decoder: it asks for nothing but the samples in order, so a live
    /// stream, a slot in RAM and a file read forward are all the same to it. A phasor restarted per chunk or a
    /// filter flushed at a chunk's end would differ.
    #[test]
    fn the_baseband_does_not_depend_on_how_the_audio_arrives() {
        let f = BandFilters::new();
        let slot = busy_slot();
        let whole = BandBaseband::from_source(&f, &mut SliceSource(&slot));

        let mut s = BandStream::new(&f);
        for &x in &slot {
            s.feed(&[x]);
        }
        let by_sample = s.finish();
        assert_eq!(whole.data, by_sample.data, "one sample at a time");
        assert_eq!(whole.peak, by_sample.peak);

        let mut s = BandStream::new(&f);
        let (mut at, mut k) = (0usize, 0usize);
        let sizes = [1usize, 7, 480, 4097];
        while at < slot.len() {
            let n = sizes[k % sizes.len()].min(slot.len() - at);
            s.feed(&slot[at..at + n]);
            at += n;
            k += 1;
        }
        let uneven = s.finish();
        assert_eq!(whole.data, uneven.data, "uneven chunks");
        assert_eq!(whole.peak, uneven.peak);
    }
}
