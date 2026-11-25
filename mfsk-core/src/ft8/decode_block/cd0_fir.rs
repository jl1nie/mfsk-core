// SPDX-License-Identifier: GPL-3.0-only
//! The per-symbol tone spectra of one candidate by a time-domain FIR decimator, `ft8b.f90`'s way without
//! its 192 000-point FFT: mix the candidate's tone 0 to DC, decimate 12 kHz to 200 Hz in three short stages
//! (3:1, 4:1, 5:1), and take a 32-point DFT of each symbol's 32 samples at the 8 tone bins.
//!
//! `ft8_downsample.f90` makes the same 200 Hz baseband (`cd0`) from a 192 000-point FFT of the whole slot
//! (1.5 MB, and the FFT, which the embedded planners do not carry); `fill_symbol_spectra_goertzel` makes the
//! tone spectra straight from the 12 kHz samples, 632 DFTs of 1920 samples per candidate. This is the
//! third way: the cost is one pass over the candidate's 12 kHz samples (the mixer and the first, 15-tap,
//! stage), the rest runs at 4 kHz and below. On `qso3_busy.wav`, 30 candidates, host, scalar f32: about 4.5x
//! faster than Goertzel's 632 DFTs per candidate (21.7 ms against 102.8 ms with the first, shorter filters).
//!
//! **Streaming**: a candidate's baseband is built sample by sample ([`Cd0Stream`]), each stage keeping only
//! its filter history (under 1 KB), so a candidate costs its 200 Hz baseband (3 200 complex samples, 25.6 KB)
//! and nothing else, and one forward read of the audio can feed several candidates at once
//! ([`build_group`]): the radio reads its SD card forward, in blocks, and cannot hold the slot.
//!
//! **The baseband is the slot's**, as `ft8_downsample.f90`'s is: sample `i` is centred on input sample `60 i`
//! (`i / 200` s), `CD0_LEN` samples. `fine_refine_3stage` (`ft8b.f90`'s stages A-C) runs on it as it runs on
//! the host's, and the symbol spectra are read at `(0.5 + dt) * 200 + 32 k`: the symbol grid is the 5 ms one
//! of the 200 Hz rate, as in WSJT-X.
//!
//! **A deliberate divergence from WSJT-X**: its `cd0` is a tapered spectral window of one big FFT, this a
//! cascade of windowed-sinc FIR filters (Kaiser, `beta` 6) with the same job: pass the 8 tones and what
//! GFSK leaves around them (0-55 Hz above tone 0), refuse what would alias onto them at the 200 Hz rate.
//! The filters' group delay is an integer number of input samples, so output `i` is centred exactly on
//! input sample `60 i`.

use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::super::params::NN;
use super::fill_symbol_spectra::{SymMask, sym_in_mask};
use super::source::AudioSource;
use super::types::{SAMPLE_RATE_HZ, TX_START_OFFSET_S};
use crate::engine::scalar::Cmplx;
use crate::ft8::refine_fine::CD0_LEN;
use num_complex::Complex;

/// Stage 1: 12 kHz -> 4 kHz. Cutoff 1972 Hz, 15 taps: a Kaiser filter of `N` taps has a transition band of
/// about `(A - 8) / (2.285 (N - 1))` radians (`A` the stopband attenuation, 63 dB at `beta` 6), which has to
/// end before the first frequency that folds onto the tones: 3 945 Hz here, 945 Hz at stage 2, 145 Hz at 3.
/// 9 taps and 17 taps (the first try) left 990 Hz only 28 dB down; `the_cascade_is_flat_...` pins it.
const N1: usize = 15;
const M1: usize = 3;
/// Stage 2: 4 kHz -> 1 kHz. Cutoff 500 Hz, 25 taps; stops from 945 Hz (1000 - 55).
const N2: usize = 25;
const M2: usize = 4;
/// Stage 3: 1 kHz -> 200 Hz. Cutoff 102 Hz; stops from 145 Hz (200 - 55).
const N3: usize = 43;
const M3: usize = 5;
/// 12 kHz samples per 200 Hz sample.
const DECIM: usize = M1 * M2 * M3;
/// Group delay of the cascade, in 12 kHz samples (an integer: all three filters have an odd length).
const DELAY: i64 = ((N1 - 1) / 2 + M1 * (N2 - 1) / 2 + M1 * M2 * (N3 - 1) / 2) as i64;
/// Output (200 Hz) samples per FT8 symbol: 0.16 s.
const SYM: usize = 32;
/// Input samples mixed per chunk: one symbol.
const CHUNK: usize = SYM * DECIM;

fn bessel_i0(x: f32) -> f32 {
    let (mut sum, mut term) = (1.0f32, 1.0f32);
    for k in 1..24 {
        term *= (x / 2.0) / k as f32;
        sum += term * term;
    }
    sum
}

/// Kaiser-windowed sinc, `n` taps, cutoff `fc` in cycles per input sample, unit DC gain.
fn kaiser_lowpass<const N: usize>(fc: f32, beta: f32) -> [f32; N] {
    let mid = (N - 1) as f32 / 2.0;
    let mut h = [0.0f32; N];
    let mut sum = 0.0;
    for (i, v) in h.iter_mut().enumerate() {
        let x = i as f32 - mid;
        let sinc = if x.abs() < 1e-6 {
            2.0 * fc
        } else {
            (core::f32::consts::TAU * fc * x).sin() / (core::f32::consts::PI * x)
        };
        let r = 2.0 * x / (N - 1) as f32;
        let w = bessel_i0(beta * (1.0 - r * r).max(0.0).sqrt()) / bessel_i0(beta);
        *v = sinc * w;
        sum += *v;
    }
    for v in h.iter_mut() {
        *v /= sum;
    }
    h
}

/// The filters and the DFT table, built once per decode and shared by all candidates.
pub(super) struct Cd0Filters {
    t1: [f32; N1],
    t2: [f32; N2],
    t3: [f32; N3],
    /// `exp(-j 2 pi k n / 32)` for the 8 tone bins `k` and the 32 samples `n` of a symbol.
    dft: [[(f32, f32); SYM]; 8],
}

impl Cd0Filters {
    pub(super) fn new() -> Self {
        let mut dft = [[(0.0f32, 0.0f32); SYM]; 8];
        for (k, row) in dft.iter_mut().enumerate() {
            for (n, v) in row.iter_mut().enumerate() {
                let a = -core::f32::consts::TAU * ((k * n) % SYM) as f32 / SYM as f32;
                *v = (a.cos(), a.sin());
            }
        }
        Self {
            t1: kaiser_lowpass::<N1>(1972.0 / 12_000.0, 6.0),
            t2: kaiser_lowpass::<N2>(500.0 / 4_000.0, 6.0),
            t3: kaiser_lowpass::<N3>(102.0 / 1_000.0, 6.0),
            dft,
        }
    }
}

/// A ring buffer with a contiguous window: every sample is written twice (`n` apart), so the last `n` are
/// always one slice.
struct Ring {
    re: Vec<f32>,
    im: Vec<f32>,
    n: usize,
    head: usize,
    count: usize,
}

impl Ring {
    fn new(n: usize) -> Self {
        Self {
            re: alloc::vec![0.0; 2 * n],
            im: alloc::vec![0.0; 2 * n],
            n,
            head: 0,
            count: 0,
        }
    }

    fn push(&mut self, re: f32, im: f32) {
        let (h, n) = (self.head, self.n);
        self.re[h] = re;
        self.re[h + n] = re;
        self.im[h] = im;
        self.im[h + n] = im;
        self.head = if h + 1 == n { 0 } else { h + 1 };
        self.count += 1;
    }

    /// The last `n` samples, oldest first.
    fn window(&self) -> (&[f32], &[f32]) {
        (
            &self.re[self.head..self.head + self.n],
            &self.im[self.head..self.head + self.n],
        )
    }
}

/// One decimating FIR stage over a stream: an output for every `m` inputs once `taps.len()` have arrived
/// (windows start at inputs 0, `m`, 2 `m`, ...).
pub(super) struct Stage<'a> {
    taps: &'a [f32],
    m: usize,
    ring: Ring,
}

impl<'a> Stage<'a> {
    pub(super) fn new(taps: &'a [f32], m: usize) -> Self {
        Self {
            taps,
            m,
            ring: Ring::new(taps.len()),
        }
    }

    pub(super) fn push(&mut self, re: f32, im: f32) -> Option<(f32, f32)> {
        self.ring.push(re, im);
        let (c, n) = (self.ring.count, self.taps.len());
        if c < n || (c - n) % self.m != 0 {
            return None;
        }
        let (wr, wi) = self.ring.window();
        let (mut a, mut b) = (0.0f32, 0.0f32);
        for (k, &t) in self.taps.iter().enumerate() {
            a += t * wr[k];
            b += t * wi[k];
        }
        Some((a, b))
    }
}

/// One candidate's 200 Hz baseband, being built from the audio as it arrives. Feed it the samples in order
/// with [`Cd0Stream::feed`] (it takes what it needs and ignores the rest) until [`Cd0Stream::done`].
pub(super) struct Cd0Stream<'a> {
    freq_hz: f32,
    /// First input sample the baseband needs, and the next one it expects.
    origin: i64,
    at: i64,
    w: f64,
    cw: f32,
    sw: f32,
    pr: f32,
    pi: f32,
    s1: Stage<'a>,
    s2: Stage<'a>,
    s3: Stage<'a>,
    out: Vec<Complex<f32>>,
}

impl<'a> Cd0Stream<'a> {
    /// A baseband for the signal whose tone 0 is at `freq_hz`.
    pub(super) fn new(f: &'a Cd0Filters, freq_hz: f32) -> Self {
        // output 0 is centred on input sample 0; the cascade's first output needs its inputs from `-DELAY`
        let origin = -DELAY;
        let w = core::f64::consts::TAU * freq_hz as f64 / SAMPLE_RATE_HZ as f64;
        Self {
            freq_hz,
            origin,
            at: origin,
            w,
            cw: w.cos() as f32,
            sw: w.sin() as f32,
            pr: 1.0,
            pi: 0.0,
            s1: Stage::new(&f.t1, M1),
            s2: Stage::new(&f.t2, M2),
            s3: Stage::new(&f.t3, M3),
            out: Vec::with_capacity(CD0_LEN),
        }
    }

    /// The first input sample this baseband needs.
    pub(super) fn origin(&self) -> i64 {
        self.origin
    }

    pub(super) fn done(&self) -> bool {
        self.out.len() >= CD0_LEN
    }

    /// Feed the samples `start..start + samples.len()`. They must follow what was fed before (or start
    /// before `origin`, whose earlier part is ignored); a gap is a bug in the caller.
    pub(super) fn feed(&mut self, start: i64, samples: &[i16]) {
        let end = start + samples.len() as i64;
        if self.done() || end <= self.at {
            return;
        }
        assert!(start <= self.at, "cd0 stream: a gap in the audio fed to it");
        for &x in &samples[(self.at - start) as usize..] {
            if self.done() {
                break;
            }
            // the phasor of every `CHUNK`th sample, exactly; rotated sample to sample in between
            if (self.at - self.origin) % CHUNK as i64 == 0 {
                let ph = self.w * self.at as f64;
                self.pr = ph.cos() as f32;
                self.pi = -(ph.sin() as f32);
            }
            let x = x as f32;
            let (mr, mi) = (x * self.pr, x * self.pi);
            let (nr, ni) = (
                self.pr * self.cw + self.pi * self.sw,
                self.pi * self.cw - self.pr * self.sw,
            );
            self.pr = nr;
            self.pi = ni;
            self.at += 1;
            if let Some((r1, i1)) = self.s1.push(mr, mi)
                && let Some((r2, i2)) = self.s2.push(r1, i1)
                && let Some((r3, i3)) = self.s3.push(r2, i2)
            {
                self.out.push(Complex::new(r3, i3));
            }
        }
    }

    pub(super) fn finish(self) -> Cd0 {
        let mut cd0 = self.out;
        cd0.resize(CD0_LEN, Complex::new(0.0, 0.0));
        Cd0 {
            freq_hz: self.freq_hz,
            cd0,
        }
    }
}

/// One candidate's 200 Hz baseband: sample `i` is centred on input sample `60 i`, `CD0_LEN` of them, tone 0
/// at DC.
pub(super) struct Cd0 {
    /// The frequency tone 0 was mixed to DC from.
    pub(super) freq_hz: f32,
    pub(super) cd0: Vec<Complex<f32>>,
}

impl Cd0 {
    /// Move tone 0 by `delf_hz` (a refinement of the mixing frequency): multiply by `exp(-j 2 pi delf n / 200)`,
    /// the baseband a mixer at `freq_hz + delf_hz` would have made.
    pub(super) fn shift_freq(&mut self, delf_hz: f32) {
        let dphi = -core::f32::consts::TAU * delf_hz / 200.0;
        for (n, c) in self.cd0.iter_mut().enumerate() {
            let a = dphi * n as f32;
            *c *= Complex::new(a.cos(), a.sin());
        }
        self.freq_hz += delf_hz;
    }
}

/// Build the basebands of `freqs` (tone-0 frequencies, Hz) from one forward read of `src`: a [`Cd0Stream`]
/// each, all fed from the same blocks.
pub(super) fn build_group<A: AudioSource>(f: &Cd0Filters, src: &mut A, freqs: &[f32]) -> Vec<Cd0> {
    let mut streams: Vec<Cd0Stream> = freqs.iter().map(|&fr| Cd0Stream::new(f, fr)).collect();
    // every stream starts at the same sample here (the slot's origin), so the read is one sweep from there
    let mut at = streams.first().map_or(0, |s| s.origin());
    let mut block = [0i16; 1024];
    while at < src.len() as i64 + DELAY && streams.iter().any(|s| !s.done()) {
        src.read(at, &mut block);
        for s in streams.iter_mut() {
            s.feed(at, &block);
        }
        at += block.len() as i64;
    }
    streams.into_iter().map(Cd0Stream::finish).collect()
}

/// `fill_symbol_spectra_goertzel`'s contract from a [`Cd0`]: the 8 tone spectra of each symbol `mask`
/// names, scaled to the Goertzel sums' (x 60, the decimation). The symbols start at the 200 Hz sample
/// nearest `(0.5 + dt_sec) * 200`.
pub(super) fn fill_from_cd0(
    f: &Cd0Filters,
    out: &mut [[Cmplx<f32>; 8]; 79],
    cd0: &Cd0,
    dt_sec: f32,
    mask: SymMask,
) {
    let i0 = ((TX_START_OFFSET_S + dt_sec) * 200.0).round() as i64;
    for sym in 0..NN {
        if !sym_in_mask(sym, mask) {
            continue;
        }
        let base = i0 + (SYM * sym) as i64;
        for (k, row) in f.dft.iter().enumerate() {
            let (mut a, mut b) = (0.0f32, 0.0f32);
            for n in 0..SYM {
                let idx = base + n as i64;
                if idx < 0 || idx as usize >= CD0_LEN {
                    continue;
                }
                let x = cd0.cd0[idx as usize];
                let (c, s) = row[n];
                a += x.re * c - x.im * s;
                b += x.re * s + x.im * c;
            }
            out[sym][k] = Cmplx {
                re: a * DECIM as f32,
                im: b * DECIM as f32,
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::tx::{message_to_tones, synthesize_i16};
    use crate::ft8::Ft8;
    use crate::ft8::decode_block::fill_symbol_spectra::fill_symbol_spectra_goertzel;
    use crate::ft8::decode_block::source::{CountingSource, SliceSource};
    use crate::msg::wsjt77::pack77;

    /// A slot holding one clean signal of tone-0 frequency `freq` starting at `dt`, and its 79 tones.
    fn slot_with(freq: f32, dt: f32) -> (Vec<i16>, Vec<u8>) {
        let m77 = pack77("CQ", "K1ABC", "FN42").unwrap();
        let tones = message_to_tones::<Ft8>(&m77);
        let pcm = synthesize_i16::<Ft8>(&tones, 12_000, freq, 8000);
        let mut slot = vec![0i16; 180_000];
        let at = ((0.5 + dt) * 12_000.0).round() as usize;
        slot[at..at + pcm.len()].copy_from_slice(&pcm);
        (slot, tones)
    }

    type Spectra = [[Cmplx<f32>; 8]; 79];

    fn all_symbols(f: &mut Spectra, fill: &mut dyn FnMut(&mut Spectra, SymMask)) {
        fill(f, SymMask::SyncOnly);
        fill(f, SymMask::DataOnly);
    }

    fn one(f: &Cd0Filters, slot: &[i16], freq: f32) -> Cd0 {
        build_group(f, &mut SliceSource(slot), &[freq])
            .pop()
            .unwrap()
    }

    /// The baseband path's tone spectra are the Goertzel path's, to a few percent, on a clean signal: the same
    /// strongest tone in every one of the 79 symbols, the transmitted tone's magnitude within 5 %, and the
    /// other seven tones below 15 % of it. A symbol grid off by 5 ms, a group delay out by 90 samples or a
    /// scale that is not the 60 of the decimation moves the magnitude by more than that. (`dt` is a multiple
    /// of 5 ms here: the baseband's grid is the 200 Hz one.)
    #[test]
    fn tone_spectra_match_goertzel_on_a_clean_signal() {
        let filters = Cd0Filters::new();
        for (freq, dt) in [(217.3f32, -0.45f32), (1234.567, 0.31), (2987.2, 0.74)] {
            let (slot, tones) = slot_with(freq, dt);
            let mut g = [[Cmplx::<f32>::default(); 8]; 79];
            all_symbols(&mut g, &mut |cs, m| {
                fill_symbol_spectra_goertzel(cs, &slot, freq, dt, m)
            });
            let mut f = [[Cmplx::<f32>::default(); 8]; 79];
            let cd0 = one(&filters, &slot, freq);
            all_symbols(&mut f, &mut |cs, m| {
                fill_from_cd0(&filters, cs, &cd0, dt, m)
            });
            for (sym, &tone) in tones.iter().enumerate() {
                let tone = tone as usize;
                let mag = |c: &Cmplx<f32>| (c.re * c.re + c.im * c.im).sqrt();
                let (mg, mf) = (mag(&g[sym][tone]), mag(&f[sym][tone]));
                assert!(
                    (mf / mg - 1.0).abs() < 0.05,
                    "f {freq} dt {dt} symbol {sym}: baseband {mf} against Goertzel {mg}"
                );
                for k in (0..8).filter(|&k| k != tone) {
                    assert!(
                        mag(&f[sym][k]) < 0.15 * mf,
                        "f {freq} dt {dt} symbol {sym}: tone {k} leaks {}",
                        mag(&f[sym][k]) / mf
                    );
                }
            }
        }
    }

    /// The cascade's amplitude response after mixing: flat over the 0-50 Hz the eight tones and their
    /// GFSK skirts occupy, and at least 40 dB down at the frequencies the rate changes would fold onto it.
    /// Measured by mixing a tone `x` Hz above `f0` to `x` Hz and reading the baseband's amplitude.
    #[test]
    fn the_cascade_is_flat_over_the_tones_and_blind_to_what_would_alias() {
        let filters = Cd0Filters::new();
        let f0 = 1500.0f32;
        let response = |x: f32| -> f32 {
            let amp = 8000.0f32;
            let slot: Vec<i16> = (0..180_000)
                .map(|n| {
                    (amp * (core::f32::consts::TAU * (f0 + x) * n as f32 / 12_000.0).cos()) as i16
                })
                .collect();
            let cd0 = one(&filters, &slot, f0);
            // steady state, well inside the slot; a cosine mixes to amp/2 at +x
            let (mut a, mut n) = (0.0f32, 0);
            for c in &cd0.cd0[1000..1000 + 4 * SYM] {
                a += c.norm();
                n += 1;
            }
            a / n as f32 / (amp / 2.0)
        };
        for x in [0.0f32, 6.25, 20.0, 43.75, 50.0] {
            let r = response(x);
            assert!((r - 1.0).abs() < 0.02, "{x} Hz: response {r}");
        }
        // What folds onto -10..+55 Hz of the final 200 Hz baseband, stage by stage: 200 k +- 55 Hz at stage 3's
        // input (1 kHz), 1 000 k +- 55 at stage 2's (4 kHz), 4 000 +- 55 at stage 1's (12 kHz); one hazard
        // frequency each, below and above DC.
        for x in [
            -150.0f32, 150.0, 250.0, 400.0, -400.0, 950.0, 990.0, -1000.0, 1050.0, 1990.0, 3950.0,
            3990.0, 4010.0,
        ] {
            let r = response(x);
            assert!(r < 0.01, "{x} Hz: response {r} (not 40 dB down)");
        }
    }

    /// Several basebands from one forward read are each the baseband built alone: the streams share the
    /// blocks, not state. And the read is one sweep of the slot (what the radio can afford), whatever the
    /// number of candidates.
    #[test]
    fn a_group_is_built_from_one_forward_read_and_equals_the_basebands_built_alone() {
        let filters = Cd0Filters::new();
        let (slot, _) = slot_with(1234.567, 0.31);
        let freqs = [217.3f32, 1234.567, 1240.0, 2987.2];
        let mut src = CountingSource::new(SliceSource(&slot[..]));
        let group = build_group(&filters, &mut src, &freqs);
        let c = src.cost();
        assert_eq!(c.restarts, 0, "{c:?}");
        assert_eq!(c.samples_skipped, 0, "{c:?}");
        assert!(c.samples_read as usize <= slot.len() + 1024, "{c:?}");
        for (g, &fr) in group.iter().zip(&freqs) {
            let alone = one(&filters, &slot, fr);
            assert_eq!(alone.cd0.len(), g.cd0.len());
            for (a, b) in alone.cd0.iter().zip(&g.cd0) {
                assert_eq!(a.re.to_bits(), b.re.to_bits());
                assert_eq!(a.im.to_bits(), b.im.to_bits());
            }
        }
    }

    /// `shift_freq` is the baseband a mixer at the shifted frequency would have made.
    #[test]
    fn shifting_a_baseband_is_remixing_it() {
        let filters = Cd0Filters::new();
        let (slot, _) = slot_with(1000.0, 0.0);
        let mut a = one(&filters, &slot, 1000.0);
        a.shift_freq(1.5);
        let b = one(&filters, &slot, 1001.5);
        let err: f32 = a
            .cd0
            .iter()
            .zip(&b.cd0)
            .map(|(x, y)| (x - y).norm())
            .fold(0.0, f32::max);
        let peak: f32 = b.cd0.iter().map(|y| y.norm()).fold(0.0, f32::max);
        assert!(
            err < 0.01 * peak,
            "worst difference {err} against a peak of {peak}"
        );
    }
}
