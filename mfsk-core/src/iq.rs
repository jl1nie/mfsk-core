// SPDX-License-Identifier: GPL-3.0-or-later
//! Wideband complex-IQ front end: one channel out of an IQ stream as the
//! 12 kHz real USB audio every decoder here takes (issue #534, phase 1).
//!
//! No decoder lives here and no device: [`IqToAudio`] is the DSP between
//! "IQ at any integer rate, centred somewhere" and "the audio a
//! transceiver's USB output would have carried for this dial frequency".
//! Feed its output to a `DecodeRequest` (after scaling to `i16`) exactly
//! as a WAV would be.
//!
//! ## Signal path
//!
//! ```text
//! IQ @ Fs ─ mix (dial + 3 kHz → DC) ─ FirStage × k ─ sharp FIR ─ L/M ─ ×e^{+jπn/2} ─ Re ─ audio @ 12 kHz
//! ```
//!
//! The channel's audio band is shifted so audio 3000 Hz sits at DC, which
//! makes the wanted 0…6 kHz a symmetric ±3 kHz window a complex low-pass can
//! cut. The low-pass passes ±2800 Hz and stops at ±3200 Hz (audio 200 Hz and
//! -200 Hz), so the sideband *below* the dial — which `Re()` would otherwise
//! fold onto the wanted one — is rejected from audio -200 Hz down, and the
//! usable audio starts at about 200 Hz, which is where an SSB receiver's own
//! filter starts. A signal at 0-200 Hz is attenuated, not decoded reliably.
//!
//! The integer decimation is a cascade of short [`FirStage`]s whose
//! transition bands are only as sharp as aliasing into the final ±2.8 kHz
//! needs; the one sharp filter (400 Hz transition) runs after them, at
//! 24-48 kS/s where it is a few hundred taps, not at the input rate where it
//! would be thousands. A last [`PolyphaseResampler`] `L/M` reaches exactly
//! 12 kHz from whatever integer rate is left, so any integer `Fs` works as
//! long as `L` stays small ([`IqError::UnsupportedRate`]).
//!
//! Group delay is compensated at the resampler; the FIR stages centre their
//! outputs on the input already. The reported offset is verified against the
//! WAV path in `tests/iq_front_end.rs`.
//!
//! Gain: a real audio tone that entered the IQ as `A·cos(ωt)·e^{jΩt}`
//! (double sideband, as a real signal mixed up) comes out at `A/2`; an
//! analytic (single sideband) IQ tone of amplitude `A` comes out at `A`.
//! Decoders here are scale-free, so this only matters when converting to `i16`.

use alloc::vec::Vec;
use core::f64::consts::TAU;

use crate::engine::dsp::fir_decimate::FirStage;
use crate::engine::dsp::polyphase::PolyphaseResampler;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// The audio rate every decoder takes.
pub const AUDIO_RATE_HZ: u32 = 12_000;
/// Where audio 3000 Hz is put, so the wanted 0…6 kHz is a ±3 kHz window.
const AUDIO_CENTRE_HZ: f64 = 3_000.0;
/// Pass edge of the channel low-pass, Hz from [`AUDIO_CENTRE_HZ`].
const PASS_HZ: f64 = 2_800.0;
/// Stop edge, Hz from [`AUDIO_CENTRE_HZ`]: audio -200 Hz on the LSB side.
const STOP_HZ: f64 = 3_200.0;
/// Rate the integer decimation stops at or above, so the sharp filter and
/// the resampler both work on at least 2x the output rate.
const MIN_INTERMEDIATE_HZ: u32 = 24_000;
/// Largest interpolation factor `L` the resampler is allowed: its tap table
/// is `32 * L` floats.
const MAX_L: u32 = 2_048;
/// Interleaved I/Q converted per pass.
const BLOCK: usize = 8_192;
/// Renormalise the NCO phasor this often.
const RENORM_EVERY: usize = 1_024;

/// Interleaved sample formats [`IqToAudio::push_bytes`] understands,
/// little-endian.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IqSampleFormat {
    /// `f32` I, `f32` Q.
    Cf32,
    /// `i16` I, `i16` Q; full scale 32768.
    Cs16,
}

impl IqSampleFormat {
    /// Bytes per complex sample.
    pub const fn bytes_per_sample(self) -> usize {
        match self {
            IqSampleFormat::Cf32 => 8,
            IqSampleFormat::Cs16 => 4,
        }
    }
}

/// What an IQ stream is: the receiver's tuning and the sample layout.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IqStream {
    /// Complex sample rate in Hz, any integer.
    pub sample_rate: u32,
    /// The RF frequency of DC in Hz (`f64`, so VHF/UHF and up fit).
    pub center_hz: f64,
    /// Sample layout of [`IqToAudio::push_bytes`].
    pub format: IqSampleFormat,
    /// Swap I and Q: sound-card IQ is often spectrally inverted.
    pub iq_swap: bool,
}

/// Why an [`IqToAudio`] could not be built.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum IqError {
    /// `sample_rate` below 12 kHz.
    RateTooLow,
    /// No `L/M` with `L <= 2048` reaches 12 kHz from this rate; the
    /// resampler would need a huge tap table.
    UnsupportedRate,
    /// The channel's 0…6 kHz audio window is not inside `±Fs/2`.
    OutsideBand,
    /// DC (the SDR's own spike) falls inside the channel's audio band.
    TooCloseToDc,
}

impl core::fmt::Display for IqError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            IqError::RateTooLow => "IQ sample rate is below 12 kHz",
            IqError::UnsupportedRate => "no small rational ratio from this IQ rate to 12 kHz",
            IqError::OutsideBand => "channel's audio window is outside the IQ band",
            IqError::TooCloseToDc => "channel's audio band contains the IQ stream's DC",
        })
    }
}

#[cfg(feature = "std")]
impl std::error::Error for IqError {}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// `(D, L, M)`: integer decimation `D`, then `L/M` to 12 kHz, with the
/// largest `D` that keeps the intermediate rate >= 24 kHz and `L` small.
fn plan(fs: u32) -> Result<(u32, u32, u32), IqError> {
    if fs < AUDIO_RATE_HZ {
        return Err(IqError::RateTooLow);
    }
    let d_max = (fs / MIN_INTERMEDIATE_HZ).max(1);
    for d in (1..=d_max).rev() {
        let num = AUDIO_RATE_HZ as u64 * d as u64;
        let g = gcd(num, fs as u64);
        let (l, m) = (num / g, fs as u64 / g);
        if l <= MAX_L as u64 {
            return Ok((d, l as u32, m as u32));
        }
    }
    Err(IqError::UnsupportedRate)
}

/// `d` as stage factors, none over 10 where the factorisation allows.
fn stage_factors(mut d: u32) -> Vec<u32> {
    let mut primes = Vec::new();
    let mut p = 2;
    while d > 1 {
        while d.is_multiple_of(p) {
            primes.push(p);
            d /= p;
        }
        p += 1;
        if p * p > d && d > 1 {
            primes.push(d);
            break;
        }
    }
    primes.sort_unstable_by(|a, b| b.cmp(a));
    let mut stages: Vec<u32> = Vec::new();
    for pr in primes {
        match stages.iter_mut().find(|s| **s * pr <= 10) {
            Some(s) => *s *= pr,
            None => stages.push(pr),
        }
    }
    stages
}

/// Odd tap count for a Blackman low-pass with the given transition width
/// (in cycles per sample), transition ~ 5.5 / N.
fn taps_for(transition_norm: f64) -> usize {
    let n = (5.5 / transition_norm).ceil() as usize;
    n.max(15) | 1
}

/// A complex phasor stepping by `e^{-jω}` per sample, kept in `f64`.
struct Nco {
    re: f64,
    im: f64,
    step_re: f64,
    step_im: f64,
    since_renorm: usize,
}

impl Nco {
    /// Multiplies by `e^{-j 2π f n / fs}`.
    fn new(f_hz: f64, fs: f64) -> Self {
        let w = TAU * f_hz / fs;
        Self {
            re: 1.0,
            im: 0.0,
            step_re: w.cos(),
            step_im: -w.sin(),
            since_renorm: 0,
        }
    }

    #[inline]
    fn mix(&mut self, i: &mut [f32], q: &mut [f32]) {
        for (xi, xq) in i.iter_mut().zip(q.iter_mut()) {
            let (pr, pi) = (self.re as f32, self.im as f32);
            let (a, b) = (*xi, *xq);
            *xi = a * pr - b * pi;
            *xq = a * pi + b * pr;
            let nr = self.re * self.step_re - self.im * self.step_im;
            let ni = self.re * self.step_im + self.im * self.step_re;
            self.re = nr;
            self.im = ni;
            self.since_renorm += 1;
            if self.since_renorm == RENORM_EVERY {
                let n = (self.re * self.re + self.im * self.im).sqrt();
                self.re /= n;
                self.im /= n;
                self.since_renorm = 0;
            }
        }
    }
}

/// One channel of a wideband IQ stream as 12 kHz real USB audio.
///
/// Streaming: push IQ in blocks of any size, in any of the typed or byte
/// forms, and audio is appended to the caller's buffer as it completes.
/// The sample count is the clock ([`Self::samples_in`]); nothing here reads
/// a time source.
pub struct IqToAudio {
    stream: IqStream,
    dial_hz: f64,
    nco: Nco,
    stages: Vec<FirStage>,
    sharp: FirStage,
    resampler: Option<PolyphaseResampler>,
    /// Output samples still to drop: the resampler's group delay.
    skip: usize,
    /// `n mod 4` of the next output, for the `e^{+jπn/2}` shift up.
    out_phase: u8,
    samples_in: u64,
    pending: Vec<u8>,
    bi: Vec<f32>,
    bq: Vec<f32>,
    ai: Vec<f32>,
    aq: Vec<f32>,
    ri: Vec<f32>,
    rq: Vec<f32>,
}

impl IqToAudio {
    /// A front end for the channel whose dial (audio 0 Hz) is `dial_hz`.
    pub fn new(stream: IqStream, dial_hz: f64) -> Result<Self, IqError> {
        let fs = stream.sample_rate;
        let (d, l, m) = plan(fs)?;
        // Audio 0 Hz .. 6 kHz must lie inside the band with the sharp
        // filter's transition to spare.
        let off = dial_hz - stream.center_hz;
        let half = fs as f64 / 2.0;
        if off < -half + 200.0 || off + 6_000.0 > half - 200.0 {
            return Err(IqError::OutsideBand);
        }
        // DC inside (or within 200 Hz of) the used audio band 200…5800 Hz.
        if -off > -200.0 && -off < 6_000.0 {
            return Err(IqError::TooCloseToDc);
        }

        let mut stages = Vec::new();
        let mut rate = fs as f64;
        for f in stage_factors(d) {
            let out = rate / f as f64;
            let transition = (out - STOP_HZ - PASS_HZ) / rate;
            let fc = (PASS_HZ + out - STOP_HZ) / 2.0 / rate;
            stages.push(FirStage::new(
                taps_for(transition),
                f as usize,
                fc as f32,
                BLOCK,
            ));
            rate = out;
        }
        let sharp = FirStage::new(
            taps_for((STOP_HZ - PASS_HZ) / rate),
            1,
            ((PASS_HZ + STOP_HZ) / 2.0 / rate) as f32,
            BLOCK,
        );
        let resampler = if l == 1 && m == 1 {
            None
        } else {
            Some(PolyphaseResampler::new(l, m, 32 * l as usize + 1, BLOCK))
        };
        let skip = resampler
            .as_ref()
            .map_or(0, PolyphaseResampler::group_delay_output);

        Ok(Self {
            stream,
            dial_hz,
            nco: Nco::new(off + AUDIO_CENTRE_HZ, fs as f64),
            stages,
            sharp,
            resampler,
            skip,
            out_phase: 0,
            samples_in: 0,
            pending: Vec::new(),
            bi: Vec::new(),
            bq: Vec::new(),
            ai: Vec::new(),
            aq: Vec::new(),
            ri: Vec::new(),
            rq: Vec::new(),
        })
    }

    /// The stream this was built for.
    pub fn stream(&self) -> IqStream {
        self.stream
    }

    /// The channel's dial frequency (audio 0 Hz), Hz.
    pub fn dial_hz(&self) -> f64 {
        self.dial_hz
    }

    /// Complex samples consumed so far: the stream's clock.
    pub fn samples_in(&self) -> u64 {
        self.samples_in
    }

    /// Push `f32` I/Q, interleaved (`i0 q0 i1 q1 …`); a trailing odd value
    /// is ignored.
    pub fn push_cf32(&mut self, iq: &[f32], out: &mut Vec<f32>) {
        for chunk in iq.chunks(2 * BLOCK) {
            let n = chunk.len() / 2;
            self.bi.clear();
            self.bq.clear();
            for &[i, q] in chunk.as_chunks::<2>().0 {
                self.bi.push(i);
                self.bq.push(q);
            }
            self.run(n, out);
        }
    }

    /// Push `i16` I/Q, interleaved, full scale 32768.
    pub fn push_cs16(&mut self, iq: &[i16], out: &mut Vec<f32>) {
        const S: f32 = 1.0 / 32_768.0;
        for chunk in iq.chunks(2 * BLOCK) {
            let n = chunk.len() / 2;
            self.bi.clear();
            self.bq.clear();
            for &[i, q] in chunk.as_chunks::<2>().0 {
                self.bi.push(i as f32 * S);
                self.bq.push(q as f32 * S);
            }
            self.run(n, out);
        }
    }

    /// Push a byte stream in [`IqStream::format`], little-endian. A sample
    /// split across two calls is carried over.
    pub fn push_bytes(&mut self, bytes: &[u8], out: &mut Vec<f32>) {
        let w = self.stream.format.bytes_per_sample();
        self.pending.extend_from_slice(bytes);
        let usable = self.pending.len() / w * w;
        let taken: Vec<u8> = self.pending.drain(..usable).collect();
        match self.stream.format {
            IqSampleFormat::Cf32 => {
                let v: Vec<f32> = taken
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|&b| f32::from_le_bytes(b))
                    .collect();
                self.push_cf32(&v, out);
            }
            IqSampleFormat::Cs16 => {
                let v: Vec<i16> = taken
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|&b| i16::from_le_bytes(b))
                    .collect();
                self.push_cs16(&v, out);
            }
        }
    }

    /// The `n` samples in `bi`/`bq` through the chain.
    fn run(&mut self, n: usize, out: &mut Vec<f32>) {
        self.samples_in += n as u64;
        if self.stream.iq_swap {
            core::mem::swap(&mut self.bi, &mut self.bq);
        }
        self.nco.mix(&mut self.bi, &mut self.bq);

        let (mut xi, mut xq) = (core::mem::take(&mut self.bi), core::mem::take(&mut self.bq));
        for st in self.stages.iter_mut() {
            self.ai.clear();
            self.aq.clear();
            st.push_block(&xi, &xq, &mut self.ai, &mut self.aq);
            core::mem::swap(&mut xi, &mut self.ai);
            core::mem::swap(&mut xq, &mut self.aq);
        }
        self.ai.clear();
        self.aq.clear();
        self.sharp.push_block(&xi, &xq, &mut self.ai, &mut self.aq);

        self.ri.clear();
        self.rq.clear();
        match self.resampler.as_mut() {
            Some(rs) => {
                for (&i, &q) in self.ai.iter().zip(&self.aq) {
                    rs.push(i, q, &mut self.ri, &mut self.rq);
                }
            }
            None => {
                core::mem::swap(&mut self.ri, &mut self.ai);
                core::mem::swap(&mut self.rq, &mut self.aq);
            }
        }

        // Shift audio 3000 Hz back up from DC: multiply by e^{+jπn/2}
        // (1, j, -1, -j), and keep the real part.
        for (&i, &q) in self.ri.iter().zip(&self.rq) {
            if self.skip > 0 {
                self.skip -= 1;
            } else {
                out.push(match self.out_phase {
                    0 => i,
                    1 => -q,
                    2 => -i,
                    _ => q,
                });
            }
            self.out_phase = (self.out_phase + 1) & 3;
        }

        self.bi = xi;
        self.bq = xq;
        self.bi.clear();
        self.bq.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone_iq(fs: u32, center: f64, rf: f64, n: usize, amp: f32) -> Vec<f32> {
        let mut v = Vec::with_capacity(2 * n);
        for k in 0..n {
            let ph = TAU * (rf - center) * k as f64 / fs as f64;
            v.push(amp * ph.cos() as f32);
            v.push(amp * ph.sin() as f32);
        }
        v
    }

    fn rms(x: &[f32]) -> f32 {
        (x.iter().map(|v| v * v).sum::<f32>() / x.len() as f32).sqrt()
    }

    fn stream(fs: u32, center: f64) -> IqStream {
        IqStream {
            sample_rate: fs,
            center_hz: center,
            format: IqSampleFormat::Cf32,
            iq_swap: false,
        }
    }

    /// An analytic tone at dial+1500 Hz comes out as a 1500 Hz real tone of
    /// the same amplitude; a tone at dial-1500 Hz (LSB side) is rejected.
    fn check_rate(fs: u32) {
        let center = 14_200_000.0;
        let dial = center + fs as f64 / 8.0;
        let mut fe = IqToAudio::new(stream(fs, center), dial).unwrap();
        let n = fs as usize * 2;
        let mut out = Vec::new();
        fe.push_cf32(&tone_iq(fs, center, dial + 1500.0, n, 0.5), &mut out);
        let tail = &out[out.len() / 2..];
        let a = rms(tail) * core::f32::consts::SQRT_2;
        assert!((a - 0.5).abs() < 0.02, "fs {fs}: amplitude {a}");
        // Frequency: zero crossings of the steady tail.
        let mut cross = 0;
        for w in tail.windows(2) {
            if w[0] < 0.0 && w[1] >= 0.0 {
                cross += 1;
            }
        }
        let f = cross as f32 * 12_000.0 / tail.len() as f32;
        assert!((f - 1500.0).abs() < 10.0, "fs {fs}: frequency {f}");

        let mut fe = IqToAudio::new(stream(fs, center), dial).unwrap();
        let mut out = Vec::new();
        fe.push_cf32(&tone_iq(fs, center, dial - 1500.0, n, 0.5), &mut out);
        let a = rms(&out[out.len() / 2..]) * core::f32::consts::SQRT_2;
        assert!(a < 0.005, "fs {fs}: LSB image {a}");
        assert_eq!(fe.samples_in(), n as u64);
    }

    #[test]
    fn tone_lands_on_audio_frequency_across_rates() {
        for fs in [48_000, 96_000, 192_000, 250_000, 768_000, 912_000] {
            check_rate(fs);
        }
    }

    #[test]
    fn rtl_rates() {
        for fs in [2_048_000, 2_400_000] {
            check_rate(fs);
        }
    }

    #[test]
    fn iq_swap_flips_the_sideband() {
        let (fs, center) = (96_000, 7_000_000.0);
        let dial = center + 10_000.0;
        let mut s = stream(fs, center);
        s.iq_swap = true;
        // Swapped, the same USB tone reads as LSB and is rejected...
        let mut fe = IqToAudio::new(s, dial).unwrap();
        let mut out = Vec::new();
        fe.push_cf32(&tone_iq(fs, center, dial + 1500.0, 96_000, 0.5), &mut out);
        assert!(rms(&out[out.len() / 2..]) < 0.005);
        // ...and the mirrored one decodes: swap(I,Q) conjugates the spectrum,
        // so the tone at -(dial+1500-center) about DC is what arrives as +.
        let mut fe = IqToAudio::new(s, dial).unwrap();
        let mut out = Vec::new();
        let mut v = tone_iq(fs, center, dial + 1500.0, 96_000, 0.5);
        for p in v.as_chunks_mut::<2>().0 {
            p.swap(0, 1);
        }
        fe.push_cf32(&v, &mut out);
        assert!(rms(&out[out.len() / 2..]) > 0.3);
    }

    #[test]
    fn cs16_and_byte_forms_match_cf32() {
        let (fs, center) = (96_000, 7_000_000.0);
        let dial = center + 10_000.0;
        let f = tone_iq(fs, center, dial + 1200.0, 48_000, 0.4);
        let i16s: Vec<i16> = f.iter().map(|v| (v * 32_768.0).round() as i16).collect();
        let mut a = IqToAudio::new(stream(fs, center), dial).unwrap();
        let mut out_f = Vec::new();
        a.push_cf32(&f, &mut out_f);

        let mut s = stream(fs, center);
        s.format = IqSampleFormat::Cs16;
        let mut b = IqToAudio::new(s, dial).unwrap();
        let mut out_b = Vec::new();
        let bytes: Vec<u8> = i16s.iter().flat_map(|v| v.to_le_bytes()).collect();
        // Split mid-sample to exercise the carry.
        b.push_bytes(&bytes[..1001], &mut out_b);
        b.push_bytes(&bytes[1001..], &mut out_b);
        assert_eq!(out_f.len(), out_b.len());
        let err = out_f
            .iter()
            .zip(&out_b)
            .map(|(x, y)| (x - y).abs())
            .fold(0.0, f32::max);
        assert!(err < 1e-3, "max diff {err}");
    }

    #[test]
    fn placement_is_validated() {
        let s = stream(96_000, 14_200_000.0);
        // DC inside the band: dial 1 kHz below centre.
        assert_eq!(
            IqToAudio::new(s, 14_199_000.0).err(),
            Some(IqError::TooCloseToDc)
        );
        // Band edge.
        assert_eq!(
            IqToAudio::new(s, 14_200_000.0 + 47_000.0).err(),
            Some(IqError::OutsideBand)
        );
        assert_eq!(
            IqToAudio::new(s, 14_200_000.0 - 60_000.0).err(),
            Some(IqError::OutsideBand)
        );
        assert_eq!(
            IqToAudio::new(stream(8_000, 0.0), 0.0).err(),
            Some(IqError::RateTooLow)
        );
        // A large prime rate has no small L/M to 12 kHz.
        assert_eq!(
            IqToAudio::new(stream(999_983, 0.0), 1_000.0).err(),
            Some(IqError::UnsupportedRate)
        );
    }

    #[test]
    fn plan_reaches_12k() {
        for fs in [48_000u32, 250_000, 768_000, 912_000, 2_048_000, 2_400_000] {
            let (d, l, m) = plan(fs).unwrap();
            let out = fs as f64 / d as f64 * l as f64 / m as f64;
            assert!((out - 12_000.0).abs() < 1e-6, "fs {fs}: {out}");
            assert!(fs / d >= MIN_INTERMEDIATE_HZ || d == 1);
        }
    }
}
