// SPDX-License-Identifier: GPL-3.0-only
//! [`PfbChannelizer`]: many channels of one IQ stream through a shared
//! polyphase filter bank (#534). The design, and every number it rests on,
//! is `docs/notes/IQ_CHANNELIZER.md`.
//!
//! ```text
//! IQ @ Fs ─► AnalysisBank: prototype → M polyphase branches → commutator → M-point FFT
//!            (2x oversampled: a hop of M/2, sub-bands S = Fs/M apart at R = 2S)
//!                 │ sub-band b nearest the channel's window
//!                 ▼
//!            per channel: IqToAudio on the sub-band (NCO by the residual, to
//!            12 kHz complex, sharp filter, back up, Re) ─► audio @ 12 kHz
//! ```
//!
//! The bank does once per stream what [`IqToAudio`] does per channel at the
//! input rate, so the per-channel cost no longer depends on `Fs`. It is the
//! option for many channels; for the handful an amateur band needs, the
//! `Direct` path (one [`IqToAudio`] per channel) is cheaper, since the bank
//! has a fixed cost of about two and a half of them.
//!
//! **Oversampled by 2** so every channel's 6.4 kHz window lies inside the
//! flat part of the sub-band nearest its centre: the prototype passes
//! `S/2 + 3.5 kHz` and stops at `R − that`, where anything beyond folds
//! outside every window the sub-band serves. The fine 400 Hz transition is
//! the back end's, at 12 kHz.
//!
//! **Time.** The prototype's length is `M(K−1)+1`, so its group delay is
//! exactly `K−1` hops; the bank drops those outputs, and sub-band sample `s`
//! is centred on input sample `s·M/2`. A channel starts feeding its back end
//! at a sub-band sample that falls on a whole 12 kHz audio sample, so audio
//! index 0 is IQ sample 0 as on the `Direct` path.

use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex32;

use super::{
    AUDIO_CENTRE_HZ, AUDIO_RATE_HZ, DESIGN_MARGIN_DB, IqError, IqSampleFormat, IqStream, IqToAudio,
    REJECT_DB, check_placement,
};
use crate::engine::dsp::fir_decimate::{design_lowpass_kaiser, kaiser_order};
use crate::engine::fft::{Fft, with_planner};

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// Sub-band spacing the bank aims for, and the band it must fall in.
const TARGET_SPACING_HZ: f64 = 24_000.0;
const MIN_SPACING_HZ: f64 = 20_000.0;
const MAX_SPACING_HZ: f64 = 32_000.0;
/// Half the window (`STOP_HZ`) plus margin, beyond half a sub-band spacing:
/// the prototype's passband edge is `S/2 + this`.
const WINDOW_REACH_HZ: f64 = 3_500.0;
/// Input samples between history compactions.
const HIST_MARGIN: usize = 8_192;

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// The bank for a rate: `M` sub-bands (even), `S = Fs/M` apart, each at
/// `R = 2Fs/M` samples per second, and the prototype.
#[derive(Clone, Debug)]
pub(crate) struct PfbPlan {
    pub m: usize,
    pub spacing_hz: f64,
    pub sub_rate: u32,
    pub k: usize,
    /// `M·K` taps: the `M(K−1)+1`-tap prototype and trailing zeros.
    pub taps: Vec<f32>,
}

/// `M` even with `S = Fs/M` in 20…32 kHz and `2Fs/M` a whole rate,
/// preferring `R` a multiple of 12 kHz, then `S` nearest 24 kHz.
pub(crate) fn pfb_plan(fs: u32) -> Result<PfbPlan, IqError> {
    if fs < AUDIO_RATE_HZ {
        return Err(IqError::RateTooLow);
    }
    let mut best: Option<((bool, u64), usize)> = None;
    let mut m = 2usize;
    while (fs as f64 / m as f64) >= MIN_SPACING_HZ {
        let s = fs as f64 / m as f64;
        if s <= MAX_SPACING_HZ && (2 * fs as u64).is_multiple_of(m as u64) {
            let r = 2 * fs as u64 / m as u64;
            let score = (
                !r.is_multiple_of(AUDIO_RATE_HZ as u64),
                (s - TARGET_SPACING_HZ).abs() as u64,
            );
            if best.as_ref().is_none_or(|(b, _)| score < *b) {
                best = Some((score, m));
            }
        }
        m += 2;
    }
    let Some((_, m)) = best else {
        return Err(IqError::UnsupportedRate);
    };
    let s = fs as f64 / m as f64;
    let r = (2 * fs as u64 / m as u64) as u32;
    let (pass, stop) = (
        s / 2.0 + WINDOW_REACH_HZ,
        r as f64 - (s / 2.0 + WINDOW_REACH_HZ),
    );
    let (n, beta) = kaiser_order(REJECT_DB + DESIGN_MARGIN_DB, (stop - pass) / fs as f64);
    let k = (n - 1).div_ceil(m) + 1;
    let mut taps = design_lowpass_kaiser(m * (k - 1) + 1, (pass + stop) / 2.0 / fs as f64, beta);
    taps.resize(m * k, 0.0);
    Ok(PfbPlan {
        m,
        spacing_hz: s,
        sub_rate: r,
        k,
        taps,
    })
}

/// The analysis bank: every hop of `M/2` input samples, all `M` sub-bands'
/// next sample.
struct AnalysisBank {
    m: usize,
    hop: usize,
    /// The prototype reversed: `g[i] = h[M·K − 1 − i]`.
    g: Vec<f32>,
    /// Input history; the window for the newest sample is its last `M·K`.
    hist: Vec<Complex32>,
    /// Samples pushed since the bank's time zero, minus one: the newest's
    /// index, or `None` before the first.
    t: Option<u64>,
    /// `K − 1` hops: the prototype's group delay.
    delay: u64,
    u: Vec<Complex32>,
    /// `e^{−j2π q/M}`, the down-conversion phase table.
    twiddle: Vec<Complex32>,
}

impl AnalysisBank {
    fn new(plan: &PfbPlan) -> Self {
        let (m, k) = (plan.m, plan.k);
        let l = m * k;
        let g: Vec<f32> = plan.taps.iter().rev().copied().collect();
        let twiddle = (0..m)
            .map(|q| {
                let w = -core::f64::consts::TAU * q as f64 / m as f64;
                Complex32::new(w.cos() as f32, w.sin() as f32)
            })
            .collect();
        let mut s = Self {
            m,
            hop: m / 2,
            g,
            hist: Vec::with_capacity(l + HIST_MARGIN),
            t: None,
            delay: ((k - 1) * (m / 2)) as u64,
            u: vec![Complex32::new(0.0, 0.0); m],
            twiddle,
        };
        s.reset();
        s
    }

    /// Back to time zero: a history of zeros, as if the stream had been
    /// silent before its first sample.
    fn reset(&mut self) {
        self.hist.clear();
        self.hist
            .resize(self.m * (self.g.len() / self.m), Complex32::new(0.0, 0.0));
        self.t = None;
    }

    /// Push one sample. When it completes a hop past the prototype's delay,
    /// all `M` sub-bands' next sample is in `out` (sub-band `b` centred on
    /// `b·S`, `b > M/2` the negative ones) and this returns `true`.
    /// `ifft` is an `M`-point inverse FFT, planned by the caller: holding a
    /// `Box<dyn Fft>` would make the bank, and every receiver on it, `!Send`.
    fn push(&mut self, x: Complex32, ifft: &dyn Fft, out: &mut [Complex32]) -> bool {
        let l = self.g.len();
        if self.hist.len() == self.hist.capacity() {
            let keep = self.hist.len() - (l - 1);
            self.hist.drain(..keep);
        }
        self.hist.push(x);
        let t = self.t.map_or(0, |t| t + 1);
        self.t = Some(t);
        if t < self.delay || !(t - self.delay).is_multiple_of(self.hop as u64) {
            return false;
        }
        // Commutator and polyphase branches: u[p] = Σ_k h[p + kM] x[t − p − kM].
        let win = &self.hist[self.hist.len() - l..];
        self.u.fill(Complex32::new(0.0, 0.0));
        let m = self.m;
        for (gc, wc) in self.g.chunks_exact(m).zip(win.chunks_exact(m)) {
            for q in 0..m {
                self.u[m - 1 - q] += wc[q] * gc[q];
            }
        }
        // Y[b] = Σ_p u[p] e^{+j2π bp/M}, then the down-conversion phase
        // e^{−j2π b t/M} (for t a multiple of M/2 this is the (−1)^{b·s} of
        // the textbook oversampled bank).
        ifft.process(&mut self.u);
        let tm = (t % m as u64) as usize;
        for (b, (o, y)) in out.iter_mut().zip(&self.u).enumerate() {
            *o = *y * self.twiddle[(b * tm) % m];
        }
        true
    }
}

struct PfbChannel {
    dial_hz: f64,
    /// The sub-band it reads, `0..M`.
    band: usize,
    fe: IqToAudio,
    /// First sub-band sample it takes: one on a whole 12 kHz audio sample.
    start_s: u64,
    bi: Vec<f32>,
    bq: Vec<f32>,
}

/// Many channels of one IQ stream through a shared polyphase filter bank.
/// See the [module docs](self).
pub struct PfbChannelizer {
    stream: IqStream,
    plan: PfbPlan,
    bank: AnalysisBank,
    channels: Vec<Option<PfbChannel>>,
    samples_in: u64,
    /// Absolute input index of the bank's time zero.
    base: u64,
    /// Input samples still to discard before `base`.
    to_drop: u64,
    /// The next sub-band sample index the bank will produce.
    next_s: u64,
    /// A channel starts on a sub-band sample that is a multiple of this, so
    /// its audio lands on whole 12 kHz samples.
    s_step: u64,
    ybuf: Vec<Complex32>,
}

impl PfbChannelizer {
    /// A bank for `stream`, with no channels. `UnsupportedRate` when no `M`
    /// fits, which is every rate under 40 kS/s: keep the `Direct` path there.
    pub fn new(stream: IqStream) -> Result<Self, IqError> {
        let plan = pfb_plan(stream.sample_rate)?;
        let bank = AnalysisBank::new(&plan);
        let r = plan.sub_rate as u64;
        let s_step = r / gcd(r, AUDIO_RATE_HZ as u64);
        Ok(Self {
            stream,
            ybuf: vec![Complex32::new(0.0, 0.0); plan.m],
            plan,
            bank,
            channels: Vec::new(),
            samples_in: 0,
            base: 0,
            to_drop: 0,
            next_s: 0,
            s_step,
        })
    }

    /// The number of sub-bands, `M`.
    pub fn bands(&self) -> usize {
        self.plan.m
    }

    /// Sub-band spacing, Hz.
    pub fn spacing_hz(&self) -> f64 {
        self.plan.spacing_hz
    }

    /// Each sub-band's sample rate, `2·Fs/M`.
    pub fn sub_rate(&self) -> u32 {
        self.plan.sub_rate
    }

    /// Taps per polyphase branch, `K`.
    pub fn taps_per_branch(&self) -> usize {
        self.plan.k
    }

    /// The stream this was built for.
    pub fn stream(&self) -> IqStream {
        self.stream
    }

    /// Complex samples consumed: the stream's clock.
    pub fn samples_in(&self) -> u64 {
        self.samples_in
    }

    /// First sub-band sample at or after `s` a channel may start on.
    fn start_at(&self, s: u64) -> u64 {
        s.div_ceil(self.s_step) * self.s_step
    }

    /// Audio index (12 kHz, from IQ sample 0) a channel added now starts at.
    pub fn next_audio_index(&self) -> u64 {
        let s0 = self.start_at(self.next_s);
        let t = self.base as u128 + s0 as u128 * self.bank.hop as u128;
        (t * AUDIO_RATE_HZ as u128 / self.stream.sample_rate as u128) as u64
    }

    /// The back end for a dial against the current centre.
    fn make_channel(&self, dial_hz: f64, start_s: u64) -> Result<PfbChannel, IqError> {
        let fs = self.stream.sample_rate as f64;
        let off = dial_hz - self.stream.center_hz;
        // The SDR's own band edges and DC still apply.
        check_placement(fs, off)?;
        let s = self.plan.spacing_hz;
        let b = ((off + AUDIO_CENTRE_HZ) / s).round() as i64;
        let band = b.rem_euclid(self.plan.m as i64) as usize;
        let sub = IqStream {
            sample_rate: self.plan.sub_rate,
            center_hz: self.stream.center_hz + b as f64 * s,
            format: IqSampleFormat::Cf32,
            iq_swap: false,
        };
        Ok(PfbChannel {
            dial_hz,
            band,
            fe: IqToAudio::build(sub, dial_hz)?,
            start_s,
            bi: Vec::new(),
            bq: Vec::new(),
        })
    }

    /// Add a channel whose dial (audio 0 Hz) is `dial_hz`; the index is what
    /// [`Self::push_planar`] addresses its audio by. Placement errors as
    /// [`IqToAudio::new`]. Added mid-stream, it starts at
    /// [`Self::next_audio_index`].
    pub fn add_channel(&mut self, dial_hz: f64) -> Result<usize, IqError> {
        let ch = self.make_channel(dial_hz, self.start_at(self.next_s))?;
        self.channels.push(Some(ch));
        Ok(self.channels.len() - 1)
    }

    /// Remove channel `idx`; its index is not reused.
    pub fn remove_channel(&mut self, idx: usize) -> bool {
        self.channels
            .get_mut(idx)
            .map(|c| c.take().is_some())
            .unwrap_or(false)
    }

    /// Start the framing over at the next sample on a whole audio sample:
    /// history, back ends and open sub-band samples dropped, the clock kept.
    fn restart(&mut self) -> Result<(), IqError> {
        let fs = self.stream.sample_rate as u64;
        let q = fs / gcd(fs, AUDIO_RATE_HZ as u64);
        self.base = self.samples_in.div_ceil(q) * q;
        self.to_drop = self.base - self.samples_in;
        self.next_s = 0;
        self.bank.reset();
        let dials: Vec<Option<f64>> = self
            .channels
            .iter()
            .map(|c| c.as_ref().map(|c| c.dial_hz))
            .collect();
        let rebuilt = dials
            .into_iter()
            .map(|d| d.map(|d| self.make_channel(d, 0)).transpose())
            .collect::<Result<Vec<_>, _>>()?;
        self.channels = rebuilt;
        Ok(())
    }

    /// The tuner moved: every channel is re-placed against `center_hz` (`Err`,
    /// and nothing changes, if one no longer fits) and the framing restarts.
    pub fn retune(&mut self, center_hz: f64) -> Result<(), IqError> {
        let fs = self.stream.sample_rate as f64;
        for c in self.channels.iter().flatten() {
            check_placement(fs, c.dial_hz - center_hz)?;
        }
        self.stream.center_hz = center_hz;
        self.restart()
    }

    /// `lost` samples never arrived: the clock advances past them and the
    /// framing restarts.
    pub fn gap(&mut self, lost: u64) {
        self.samples_in += lost;
        self.restart()
            .expect("the same stream and dials placed before");
    }

    /// Push planar I/Q (equal lengths). Each channel's audio is appended to
    /// `out[channel]` (`out` at least as long as the indices handed out).
    pub fn push_planar(&mut self, i: &[f32], q: &[f32], out: &mut [Vec<f32>]) {
        debug_assert_eq!(i.len(), q.len());
        self.samples_in += i.len() as u64;
        let skip = (self.to_drop as usize).min(i.len());
        self.to_drop -= skip as u64;
        // Planned once per push (a cached plan on `std`), not held: see
        // `AnalysisBank::push`.
        let ifft = with_planner(|p| p.plan_inverse(self.plan.m));
        for (&a, &b) in i[skip..].iter().zip(&q[skip..]) {
            if !self
                .bank
                .push(Complex32::new(a, b), ifft.as_ref(), &mut self.ybuf)
            {
                continue;
            }
            let s = self.next_s;
            self.next_s += 1;
            for c in self.channels.iter_mut().flatten() {
                if s >= c.start_s {
                    let y = self.ybuf[c.band];
                    c.bi.push(y.re);
                    c.bq.push(y.im);
                }
            }
        }
        for (idx, slot) in self.channels.iter_mut().enumerate() {
            let Some(c) = slot.as_mut() else { continue };
            if !c.bi.is_empty() {
                c.fe.push_planar(&c.bi, &c.bq, &mut out[idx]);
                c.bi.clear();
                c.bq.clear();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::TAU;

    fn stream(fs: u32, center: f64) -> IqStream {
        IqStream {
            sample_rate: fs,
            center_hz: center,
            format: IqSampleFormat::Cf32,
            iq_swap: false,
        }
    }

    fn tone(fs: u32, center: f64, rf: f64, n: usize) -> (Vec<f32>, Vec<f32>) {
        let w = TAU * (rf - center) / fs as f64;
        (0..n)
            .map(|k| {
                let p = w * k as f64;
                (p.cos() as f32, p.sin() as f32)
            })
            .unzip()
    }

    fn rms(x: &[f32]) -> f64 {
        let t = &x[x.len() / 2..];
        (t.iter().map(|v| (*v as f64).powi(2)).sum::<f64>() / t.len() as f64).sqrt()
    }

    fn out_rms(fs: u32, dial: f64, rf: f64, secs: f64) -> f64 {
        let center = 14_200_000.0;
        let mut ch = PfbChannelizer::new(stream(fs, center)).unwrap();
        let idx = ch.add_channel(dial).unwrap();
        let (i, q) = tone(fs, center, rf, (fs as f64 * secs) as usize);
        let mut out = vec![Vec::new(); idx + 1];
        for (a, b) in i.chunks(7_919).zip(q.chunks(7_919)) {
            ch.push_planar(a, b, &mut out);
        }
        rms(&out[idx])
    }

    /// The plan table of `docs/notes/IQ_CHANNELIZER.md` §4, and the prototype's
    /// length rule.
    #[test]
    fn plans_match_the_design() {
        for (fs, m, s, r, k) in [
            (192_000u32, 8usize, 24_000.0, 48_000u32, 13usize),
            (768_000, 32, 24_000.0, 48_000, 13),
            (912_000, 38, 24_000.0, 48_000, 13),
            (2_400_000, 100, 24_000.0, 48_000, 13),
            (250_000, 10, 25_000.0, 50_000, 0),
            (2_048_000, 80, 25_600.0, 51_200, 0),
        ] {
            let p = pfb_plan(fs).unwrap();
            assert_eq!((p.m, p.spacing_hz, p.sub_rate), (m, s, r), "fs {fs}");
            if k != 0 {
                assert_eq!(p.k, k, "fs {fs}");
            }
            assert_eq!(p.taps.len(), p.m * p.k);
            assert_eq!(*p.taps.last().unwrap(), 0.0);
        }
        // 48 kS/s is M = 2, a bank that is only overhead but works.
        assert_eq!(pfb_plan(48_000).unwrap().m, 2);
        // Under 40 kS/s no spacing of 20 kHz or more fits two sub-bands.
        assert_eq!(pfb_plan(30_000).err(), Some(IqError::UnsupportedRate));
        assert_eq!(pfb_plan(8_000).err(), Some(IqError::RateTooLow));
    }

    /// A tone in the window has the gain the `Direct` path gives it, for dials
    /// that put the window anywhere across a sub-band, its edges included.
    #[test]
    fn passband_is_flat_across_a_sub_band() {
        let fs = 768_000u32;
        let center = 14_200_000.0;
        let mut worst = (f64::MAX, 0.0f64);
        for k in 0..=24 {
            // Window centre from −S/2 to +S/2 around sub-band 3's centre.
            let c = 3.0 * 24_000.0 - 12_000.0 + 1_000.0 * k as f64;
            let dial = center + c - AUDIO_CENTRE_HZ;
            for a in [300.0, 1_500.0, 2_900.0, 5_700.0] {
                let g = out_rms(fs, dial, dial + a, 0.4) * core::f64::consts::SQRT_2;
                worst = (worst.0.min(g), worst.1.max(g));
            }
        }
        let spread = 20.0 * (worst.1 / worst.0).log10();
        assert!((worst.1 - 1.0).abs() < 0.01, "gain {}", worst.1);
        assert!(spread < 0.05, "spread {spread} dB");
    }

    /// Selectivity: interferers aimed at the prototype's fold edges and at
    /// sub-band boundaries, for a window at the edge of its sub-band.
    #[test]
    fn rejects_interferers_at_every_fold() {
        let fs = 768_000u32;
        let center = 14_200_000.0;
        // Window centre 11 kHz above sub-band 2's centre: near its edge.
        let dial = center + 2.0 * 24_000.0 + 11_000.0 - AUDIO_CENTRE_HZ;
        let wanted = out_rms(fs, dial, dial + 1_500.0, 0.25);
        let centre = dial + AUDIO_CENTRE_HZ;
        let mut worst: (f64, f64) = (f64::MIN, 0.0);
        for base in [24_000.0f64, 48_000.0, 72_000.0, 96_000.0] {
            for sgn in [-1.0f64, 1.0] {
                for d in [
                    3_300.0, 3_700.0, 5_000.0, 9_400.0, 12_000.0, 15_600.0, 20_100.0,
                ] {
                    for e in [-1.0f64, 1.0] {
                        let rf = centre + sgn * base + e * d;
                        let o = rf - dial;
                        if (-200.0..=6_200.0).contains(&o) {
                            continue;
                        }
                        let v = 20.0 * (out_rms(fs, dial, rf, 0.25) / wanted).log10();
                        if v > worst.0 {
                            worst = (v, o);
                        }
                    }
                }
            }
        }
        assert!(
            worst.0 <= -(REJECT_DB - 1.0),
            "{:.1} dB at {:.0} Hz",
            worst.0,
            worst.1
        );
    }

    /// Audio index 0 is IQ sample 0: a burst starts at the same audio sample
    /// as through `IqToAudio`.
    #[test]
    fn timing_matches_direct() {
        let (fs, center) = (768_000u32, 7_000_000.0);
        let dial = center + 61_000.0;
        let n = fs as usize;
        let (mut i, mut q) = tone(fs, center, dial + 1_500.0, n);
        let on = 300_000; // 0.390625 s
        i[..on].fill(0.0);
        q[..on].fill(0.0);
        let first = |a: &[f32]| a.iter().position(|v| v.abs() > 0.25).unwrap();

        let mut ch = PfbChannelizer::new(stream(fs, center)).unwrap();
        let idx = ch.add_channel(dial).unwrap();
        let mut out = vec![Vec::new(); idx + 1];
        ch.push_planar(&i, &q, &mut out);

        let mut fe = IqToAudio::new(stream(fs, center), dial).unwrap();
        let mut d = Vec::new();
        let v: Vec<f32> = i.iter().zip(&q).flat_map(|(&a, &b)| [a, b]).collect();
        fe.push_cf32(&v, &mut d);
        assert!(
            first(&out[idx]).abs_diff(first(&d)) <= 1,
            "PFB {} vs Direct {}",
            first(&out[idx]),
            first(&d)
        );
    }

    #[test]
    fn placement_retune_and_gap() {
        let center = 14_200_000.0;
        let mut ch = PfbChannelizer::new(stream(768_000, center)).unwrap();
        assert_eq!(ch.add_channel(center - 1_000.0), Err(IqError::TooCloseToDc));
        assert_eq!(
            ch.add_channel(center + 383_000.0),
            Err(IqError::OutsideBand)
        );
        let a = ch.add_channel(center + 50_000.0).unwrap();
        assert_eq!(ch.retune(center + 500_000.0), Err(IqError::OutsideBand));
        assert_eq!(ch.stream().center_hz, center);
        assert!(ch.retune(center + 10_000.0).is_ok());
        let mut out = vec![Vec::new(); a + 1];
        ch.push_planar(&[0.0; 1_000], &[0.0; 1_000], &mut out);
        ch.gap(777);
        assert_eq!(ch.samples_in(), 1_777);
        // The next audio sample after a gap lands on a whole audio index.
        let k = ch.next_audio_index();
        assert_eq!((k as u128 * 768_000) % 12_000, 0);
        assert!(k * 64 >= 1_777);
        assert!(ch.remove_channel(a));
        assert!(!ch.remove_channel(a));
    }
}
