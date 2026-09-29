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
//! IQ @ Fs ─ mix (dial + 3 kHz → DC) ─ FirStage × k ─ L/M to 12 kHz ─ sharp FIR ─ ×e^{+jπn/2} ─ Re ─ audio @ 12 kHz
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
//! **Selectivity: [`REJECT_DB`] (120 dB) everywhere outside the window**,
//! every filter a Kaiser design for it ([`kaiser_order`]). 120 dB is the noise
//! floor of an ideal 16-bit ADC in 2500 Hz at 768 kS/s (−120 dBFS; −108 for 14
//! bits, −96 for 12), so a full-scale interferer leaks no higher than the
//! quietest SDR's own floor; f32 holds it (the arithmetic floor of these FIRs
//! is −136 dBFS). Before #534's channelizer study this path used Blackman
//! windows, which stop at about 74 dB: an interferer 512 Hz below the dial
//! came through at −87 dB.
//!
//! The integer decimation is a cascade of short [`FirStage`]s, each passing
//! ±3.2 kHz and stopping where its aliases would reach the window. A
//! [`PolyphaseResampler`] `L/M` then reaches exactly 12 kHz *complex* from
//! whatever integer rate is left (so any integer `Fs` works as long as `L`
//! stays small, [`IqError::UnsupportedRate`]); it only has to stop at 8.8 kHz,
//! where its aliases land outside the window. The one sharp filter (400 Hz
//! transition) runs last, at 12 kHz, where 120 dB is 237 taps — half the
//! rate, and so half the cost, of running it before the resampler.
//!
//! The FIR stages centre their outputs on the input; the resampler's group
//! delay is a whole number of output samples by construction and is dropped,
//! so audio index 0 is IQ sample 0 (`tests/iq_front_end.rs` checks the DT the
//! decoders report against the WAV path).
//!
//! Gain: a real audio tone that entered the IQ as `A·cos(ωt)·e^{jΩt}`
//! (double sideband, as a real signal mixed up) comes out at `A/2`; an
//! analytic (single sideband) IQ tone of amplitude `A` comes out at `A`.
//! Decoders here are scale-free, so this only matters when converting to `i16`.
//!
//! `IqReceiver` (needs an FFT backend and a protocol feature) builds on this: N channels of one
//! stream, slots cut on UTC from the sample count, each decoded with its
//! mode's own request, rows carrying the absolute RF frequency.

use alloc::vec::Vec;
use core::f64::consts::TAU;

#[cfg(all(
    any(feature = "fft-rustfft", feature = "fft-extern"),
    any(
        feature = "ft8",
        feature = "ft4",
        feature = "fst4",
        feature = "wspr",
        feature = "jt9",
        feature = "jt65",
        feature = "q65"
    )
))]
pub mod receiver;
#[cfg(all(
    any(feature = "fft-rustfft", feature = "fft-extern"),
    any(
        feature = "ft8",
        feature = "ft4",
        feature = "fst4",
        feature = "wspr",
        feature = "jt9",
        feature = "jt65",
        feature = "q65"
    )
))]
pub use receiver::{ChannelId, IqDecode, IqMode, IqReceiver};

use crate::engine::dsp::fir_decimate::{FirStage, design_lowpass_kaiser, kaiser_order};
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
/// Stopband attenuation of every filter on the path, dB. See the module
/// docs for why 120.
pub const REJECT_DB: f64 = 120.0;
/// Each filter is *designed* this much above [`REJECT_DB`]: Kaiser's order
/// estimate is short at the stop edge for short filters. Designed at exactly
/// 120 dB, the 768 → 96 kHz stage passed an alias 0.8 kHz past its stop edge
/// at −117.9 dB (`tests/iq_front_end.rs`'s sweep).
const DESIGN_MARGIN_DB: f64 = 3.0;
/// Rate the integer decimation stops at or above, so the resampler works on
/// at least 2x the output rate.
const MIN_INTERMEDIATE_HZ: u32 = 24_000;
/// Largest interpolation factor `L` the resampler is allowed: its tap table
/// is `32 * L` floats.
const MAX_L: u32 = 2_048;
/// Interleaved I/Q converted per pass.
const BLOCK: usize = 8_192;
/// Renormalise the NCO phasor this often.
const RENORM_EVERY: usize = 1_024;

/// Interleaved sample formats [`IqToAudio::push_bytes`] understands,
/// little-endian, I then Q.
///
/// The typed pushes (`push_cf32`, `push_cs16`) take the two 4- and 8-byte
/// forms already unpacked; the 8-bit and 24-bit ones exist as byte streams
/// only, which is what their sources produce.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IqSampleFormat {
    /// `f32` I, `f32` Q: SDR#, libairspyhf, GNU Radio, SoapySDR.
    Cf32,
    /// `i16` I, `i16` Q, full scale 32768: IQ WAV recordings, SoapySDR.
    Cs16,
    /// `i8` I, `i8` Q, full scale 128: HackRF.
    Cs8,
    /// `u8` I, `u8` Q, 128 = zero, full scale 128: RTL-SDR.
    Cu8,
    /// 24-bit signed I, Q, full scale 8 388 608: some WAV recordings.
    Cs24,
}

impl IqSampleFormat {
    /// Bytes per complex sample.
    pub const fn bytes_per_sample(self) -> usize {
        match self {
            IqSampleFormat::Cf32 => 8,
            IqSampleFormat::Cs16 => 4,
            IqSampleFormat::Cs8 | IqSampleFormat::Cu8 => 2,
            IqSampleFormat::Cs24 => 6,
        }
    }

    /// Append the samples in `bytes` (a whole number of them) to `bi` / `bq`
    /// as floats scaled to full scale 1.0.
    pub(crate) fn convert(self, bytes: &[u8], bi: &mut Vec<f32>, bq: &mut Vec<f32>) {
        debug_assert_eq!(bytes.len() % self.bytes_per_sample(), 0);
        match self {
            IqSampleFormat::Cf32 => {
                for c in bytes.as_chunks::<8>().0 {
                    bi.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
                    bq.push(f32::from_le_bytes([c[4], c[5], c[6], c[7]]));
                }
            }
            IqSampleFormat::Cs16 => {
                for c in bytes.as_chunks::<4>().0 {
                    bi.push(i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0);
                    bq.push(i16::from_le_bytes([c[2], c[3]]) as f32 / 32_768.0);
                }
            }
            IqSampleFormat::Cs8 => {
                for &[i, q] in bytes.as_chunks::<2>().0 {
                    bi.push(i as i8 as f32 / 128.0);
                    bq.push(q as i8 as f32 / 128.0);
                }
            }
            IqSampleFormat::Cu8 => {
                for &[i, q] in bytes.as_chunks::<2>().0 {
                    bi.push((i as f32 - 128.0) / 128.0);
                    bq.push((q as f32 - 128.0) / 128.0);
                }
            }
            IqSampleFormat::Cs24 => {
                // Sign-extend by placing the 24 bits at the top of an i32.
                let s24 = |b: &[u8]| (i32::from_le_bytes([0, b[0], b[1], b[2]]) >> 8) as f32;
                for c in bytes.as_chunks::<6>().0 {
                    bi.push(s24(&c[0..3]) / 8_388_608.0);
                    bq.push(s24(&c[3..6]) / 8_388_608.0);
                }
            }
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

        // Each stage passes the window (±STOP_HZ) and stops where its
        // aliases would land in it (`out − STOP_HZ`).
        let kaiser = |pass: f64, stop: f64, rate: f64| {
            let (n, beta) = kaiser_order(REJECT_DB + DESIGN_MARGIN_DB, (stop - pass) / rate);
            (n, beta, (pass + stop) / 2.0 / rate)
        };
        let mut stages = Vec::new();
        let mut rate = fs as f64;
        for f in stage_factors(d) {
            let out = rate / f as f64;
            let (n, beta, fc) = kaiser(STOP_HZ, out - STOP_HZ, rate);
            stages.push(FirStage::from_taps(
                &design_lowpass_kaiser(n, fc, beta),
                f as usize,
                BLOCK,
            ));
            rate = out;
        }
        // To 12 kHz complex: the prototype runs at `rate · L` and stops at
        // 12 kHz − STOP_HZ, where aliases fall outside the window. Its length
        // is rounded up to `2·M·k + 1` so the group delay is `k` whole
        // output samples.
        let resampler = if l == 1 && m == 1 {
            None
        } else {
            let up = rate * l as f64;
            let (n, beta, fc) = kaiser(STOP_HZ, AUDIO_RATE_HZ as f64 - STOP_HZ, up);
            let two_m = 2 * m as usize;
            let n = (n - 1).div_ceil(two_m) * two_m + 1;
            Some(PolyphaseResampler::from_prototype(
                l,
                m,
                &design_lowpass_kaiser(n, fc, beta),
                BLOCK,
            ))
        };
        let skip = resampler
            .as_ref()
            .map_or(0, PolyphaseResampler::group_delay_output);
        let (n, beta, fc) = kaiser(PASS_HZ, STOP_HZ, AUDIO_RATE_HZ as f64);
        let sharp = FirStage::from_taps(&design_lowpass_kaiser(n, fc, beta), 1, BLOCK);

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
        for chunk in taken.chunks(w * BLOCK) {
            self.bi.clear();
            self.bq.clear();
            self.stream
                .format
                .convert(chunk, &mut self.bi, &mut self.bq);
            self.run(chunk.len() / w, out);
        }
    }

    /// Push already-converted planar I/Q (equal lengths), as
    /// [`IqReceiver`] does once for all its channels.
    #[cfg(all(
        any(feature = "fft-rustfft", feature = "fft-extern"),
        any(
            feature = "ft8",
            feature = "ft4",
            feature = "fst4",
            feature = "wspr",
            feature = "jt9",
            feature = "jt65",
            feature = "q65"
        )
    ))]
    pub(crate) fn push_planar(&mut self, i: &[f32], q: &[f32], out: &mut Vec<f32>) {
        debug_assert_eq!(i.len(), q.len());
        self.bi.clear();
        self.bq.clear();
        self.bi.extend_from_slice(i);
        self.bq.extend_from_slice(q);
        self.run(i.len(), out);
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

        self.ri.clear();
        self.rq.clear();
        match self.resampler.as_mut() {
            Some(rs) => {
                self.ai.clear();
                self.aq.clear();
                for (&i, &q) in xi.iter().zip(&xq) {
                    rs.push(i, q, &mut self.ai, &mut self.aq);
                }
                // Its group delay, a whole number of samples, is dropped.
                let drop = self.skip.min(self.ai.len());
                self.skip -= drop;
                self.sharp.push_block(
                    &self.ai[drop..],
                    &self.aq[drop..],
                    &mut self.ri,
                    &mut self.rq,
                );
            }
            None => self.sharp.push_block(&xi, &xq, &mut self.ri, &mut self.rq),
        }

        // Shift audio 3000 Hz back up from DC: multiply by e^{+jπn/2}
        // (1, j, -1, -j), n counted from audio index 0, and keep the real part.
        for (&i, &q) in self.ri.iter().zip(&self.rq) {
            out.push(match self.out_phase {
                0 => i,
                1 => -q,
                2 => -i,
                _ => q,
            });
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

    /// Every byte format reads back what was put in, to its own precision.
    #[test]
    fn every_format_converts_to_the_same_floats() {
        let want: [(f32, f32); 5] = [
            (0.5, -0.25),
            (-1.0, 0.0),
            (0.999, -0.5),
            (0.0, 0.0),
            (0.125, 0.75),
        ];
        for (fmt, tol) in [
            (IqSampleFormat::Cf32, 0.0),
            (IqSampleFormat::Cs16, 1.0 / 32_768.0),
            (IqSampleFormat::Cs8, 1.0 / 128.0),
            (IqSampleFormat::Cu8, 1.0 / 128.0),
            (IqSampleFormat::Cs24, 1.0 / 8_388_608.0),
        ] {
            let mut bytes = Vec::new();
            for &(i, q) in &want {
                for v in [i, q] {
                    match fmt {
                        IqSampleFormat::Cf32 => bytes.extend(v.to_le_bytes()),
                        IqSampleFormat::Cs16 => {
                            bytes.extend(((v * 32_768.0).round() as i16).to_le_bytes())
                        }
                        IqSampleFormat::Cs8 => {
                            bytes.push(((v * 128.0).round().clamp(-128.0, 127.0) as i8) as u8)
                        }
                        IqSampleFormat::Cu8 => {
                            bytes.push((v * 128.0 + 128.0).round().clamp(0.0, 255.0) as u8)
                        }
                        IqSampleFormat::Cs24 => {
                            let x =
                                (v * 8_388_608.0).round().clamp(-8_388_608.0, 8_388_607.0) as i32;
                            bytes.extend(&x.to_le_bytes()[..3]);
                        }
                    }
                }
            }
            assert_eq!(bytes.len(), want.len() * fmt.bytes_per_sample(), "{fmt:?}");
            let (mut bi, mut bq) = (Vec::new(), Vec::new());
            fmt.convert(&bytes, &mut bi, &mut bq);
            for (k, &(i, q)) in want.iter().enumerate() {
                assert!(
                    (bi[k] - i).abs() <= tol + 1e-6,
                    "{fmt:?} I[{k}] {} vs {i}",
                    bi[k]
                );
                assert!(
                    (bq[k] - q).abs() <= tol + 1e-6,
                    "{fmt:?} Q[{k}] {} vs {q}",
                    bq[k]
                );
            }
        }
    }

    /// The byte path through the front end matches `Cf32` for every format,
    /// with samples split mid-way across calls.
    #[test]
    fn byte_forms_match_cf32_for_every_format() {
        let (fs, center) = (96_000, 7_000_000.0);
        let dial = center + 10_000.0;
        let f = tone_iq(fs, center, dial + 1200.0, 48_000, 0.4);
        let mut a = IqToAudio::new(stream(fs, center), dial).unwrap();
        let mut reference = Vec::new();
        a.push_cf32(&f, &mut reference);
        for fmt in [
            IqSampleFormat::Cs8,
            IqSampleFormat::Cu8,
            IqSampleFormat::Cs24,
        ] {
            let mut bytes = Vec::new();
            for &v in &f {
                match fmt {
                    IqSampleFormat::Cs8 => {
                        bytes.push(((v * 128.0).round().clamp(-128.0, 127.0) as i8) as u8)
                    }
                    IqSampleFormat::Cu8 => {
                        bytes.push((v * 128.0 + 128.0).round().clamp(0.0, 255.0) as u8)
                    }
                    _ => bytes.extend(&((v * 8_388_608.0).round() as i32).to_le_bytes()[..3]),
                }
            }
            let mut s = stream(fs, center);
            s.format = fmt;
            let mut b = IqToAudio::new(s, dial).unwrap();
            let mut out = Vec::new();
            let cut = 1_001;
            b.push_bytes(&bytes[..cut], &mut out);
            b.push_bytes(&bytes[cut..], &mut out);
            assert_eq!(out.len(), reference.len(), "{fmt:?}");
            // 8-bit quantisation noise (1/128 per component) is spread over
            // the wideband IQ and filtered down to the channel: compare RMS
            // error to the signal's, not sample by sample.
            let err = (out
                .iter()
                .zip(&reference)
                .map(|(x, y)| (x - y).powi(2))
                .sum::<f32>()
                / out.len() as f32)
                .sqrt();
            let sig = rms(&reference);
            let limit = if fmt == IqSampleFormat::Cs24 {
                1e-4
            } else {
                0.03
            };
            assert!(
                err / sig < limit,
                "{fmt:?}: error {} of signal {}",
                err,
                sig
            );
        }
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
