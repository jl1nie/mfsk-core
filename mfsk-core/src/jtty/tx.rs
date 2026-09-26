//! JTTY transmit path: atoms → 34-bit payloads → tones → GFSK samples.
//!
//! Ported from WSJT-X `lib/jtty/genjtty.f90` (`genjtty_frames`) and
//! `gen_jttywave.f90`, tag `v3.2.0-rc1`. Every frame is 13 sync tones followed
//! by 46 data tones; the waveform is a Gaussian frequency-pulse train with
//! `bt = 2`, modulation index 1 (tone spacing = baud = 31.25 Hz) and a cosine
//! ramp of `nsps/8` samples at each end of the whole transmission.
//!
//! ## Parallel by construction
//!
//! With the `parallel` feature the independent work runs on rayon's pool, in
//! every case with results collected in order so the output does not depend on
//! the thread count; without it the same code runs sequentially.
//!
//! - **Frames** are independent of one another (CRC, convolutional encoding).
//! - **The waveform** is built without a sample-by-sample loop: the phase
//!   *rate* at each output sample is a sum of at most three pulse terms, so it
//!   is computed per sample in parallel; the phase is its running sum, done as
//!   a chunked scan (chunk sums in parallel, a short sequential scan over the
//!   chunk offsets, then each chunk integrated in parallel); `sin` is
//!   evaluated in the same pass. Phase is carried in `f64`, so it does not
//!   drift the way a single-precision running sum does.
//!
//! ## Why not `engine::dsp::gfsk`
//!
//! When this was written that synthesiser sampled the Gaussian pulse one sample
//! earlier than every upstream `gen_*wave.f90` (#482, fixed since in #490) —
//! 0.049 rad for JTTY's largest tone step, which is what a comparison against
//! `sjtty`'s noiseless output showed (worst normalised sample error 4.9e-2,
//! against 1.1e-4 with the pulse index corrected). JTTY keeps its own all the
//! same: it is generic over the symbol length and produces the complex reference a
//! receiver subtracts (at 6 kHz, 192 samples per symbol), its phase is carried in
//! `f64`, and it is built without a sample loop, which the shared one is not.
//!
use alloc::vec::Vec;
use core::f64::consts::TAU;

use num_complex::Complex32;

use super::source::Atom;
use super::{
    FRAME_SYMBOLS, INFO_BITS, MAX_FRAMES, NSPS, Payload, SAMPLE_RATE, SYNC, SYNC_SYMBOLS, crc, tbcc,
};
use crate::engine::dsp::gfsk::gfsk_pulse;
use num_traits::Float;

/// Gaussian bandwidth-time product (`bt` in `sjtty` / the GUI).
const BT: f32 = 2.0;
/// Modulation index: tone spacing equals the symbol rate.
const HMOD: f64 = 1.0;

/// Frames below which splitting across threads is not worth a task: encoding one
/// is microseconds, so the parallel path is mostly about keeping the structure
/// the receiver's much heavier per-frame work will use.
#[cfg(feature = "parallel")]
const PAR_MIN_LEN: usize = 4;

/// Map `f` over `items` — on rayon's pool when `parallel` is on — preserving
/// order. `f` gets the item's index.
fn map_indexed<T: Sync, R: Send>(items: &[T], f: impl Fn(usize, &T) -> R + Sync + Send) -> Vec<R> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        items
            .par_iter()
            .enumerate()
            .with_min_len(PAR_MIN_LEN)
            .map(|(i, t)| f(i, t))
            .collect()
    }
    #[cfg(not(feature = "parallel"))]
    {
        items.iter().enumerate().map(|(i, t)| f(i, t)).collect()
    }
}

/// The 34-bit payloads of a message, one per atom, the end-of-message flag on
/// the last only. `None` for an empty message, more than [`MAX_FRAMES`] atoms,
/// or an atom that cannot be encoded.
pub fn payloads(atoms: &[Atom]) -> Option<Vec<Payload>> {
    if atoms.is_empty() || atoms.len() > MAX_FRAMES {
        return None;
    }
    let last = atoms.len() - 1;
    map_indexed(atoms, |i, a| a.encode(i == last))
        .into_iter()
        .collect()
}

/// One frame's 59 channel tones: the sync sequence, then the tail-biting
/// encoding of the payload plus its CRC-12.
pub fn frame_tones(payload: &Payload) -> [u8; FRAME_SYMBOLS] {
    let data = tbcc::encode(&crc::append(payload));
    core::array::from_fn(|i| {
        if i < SYNC_SYMBOLS {
            SYNC[i]
        } else {
            data[i - SYNC_SYMBOLS]
        }
    })
}

/// The channel tones of a whole message (`59 × atoms.len()` of them), or `None`
/// as for [`payloads`].
pub fn tones(atoms: &[Atom]) -> Option<Vec<u8>> {
    let payloads = payloads(atoms)?;
    let frames = map_indexed(&payloads, |_, p| frame_tones(p));
    Some(frames.into_iter().flatten().collect())
}

/// Order-preserving parallel `collect` over `0..len`.
fn collect_range<R: Send>(
    len: usize,
    min_len: usize,
    f: impl Fn(usize) -> R + Sync + Send,
) -> Vec<R> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        (0..len)
            .into_par_iter()
            .with_min_len(min_len)
            .map(f)
            .collect()
    }
    #[cfg(not(feature = "parallel"))]
    {
        let _ = min_len;
        (0..len).map(f).collect()
    }
}

/// Symbol length and sample rate a waveform is built at.
#[derive(Clone, Copy)]
struct Geometry {
    nsps: usize,
    fs: f64,
}

impl Geometry {
    /// The 12 kHz audio of a transmitter.
    const AUDIO: Self = Self {
        nsps: NSPS,
        fs: SAMPLE_RATE as f64,
    };

    /// `gen_jttywave.f90`'s pulse: `pulse(i)`, `i = 1..3·nsps`, at
    /// `tt = (i − 1.5·nsps)/nsps`.
    fn pulse(&self) -> Vec<f32> {
        (1..=3 * self.nsps)
            .map(|i| gfsk_pulse(BT, (i as f32 - 1.5 * self.nsps as f32) / self.nsps as f32))
            .collect()
    }
}

/// The phase rate, radians per sample, at output sample `n` of a transmission
/// of `tones` (`dphi` of `gen_jttywave.f90`, carrier included).
///
/// Upstream builds an array of `(nsym + 2)·nsps` rates in which symbol `j`
/// contributes `tone · pulse` over three symbol periods, the first and last
/// symbols are repeated once as dummies so the pulse train is continuous at the
/// ends, and the audio is read from index `nsps` on. Position `p = nsps + n`
/// is covered by at most three symbols, so each sample is a short sum.
fn dphi_at(n: usize, tones: &[u8], pulse: &[f32], nsps: usize, carrier: f64, slope: f64) -> f64 {
    let nsym = tones.len();
    let p = nsps + n;
    let q = p / nsps;
    let term = |tone: u8, m: usize| f64::from(tone) * f64::from(pulse[m]);
    // symbols j with j·nsps ≤ p < (j + 3)·nsps
    let body: f64 = (q.saturating_sub(2)..=q)
        .filter(|&j| j < nsym)
        .map(|j| term(tones[j], p - j * nsps))
        .sum();
    // dummy symbol before the first / after the last
    let head = if p < 2 * nsps {
        term(tones[0], nsps + p)
    } else {
        0.0
    };
    let tail = if p >= nsym * nsps {
        term(tones[nsym - 1], p - nsym * nsps)
    } else {
        0.0
    };
    carrier + slope * n as f64 + TAU * HMOD / nsps as f64 * (body + head + tail)
}

/// The phase, radians, at every output sample of `tones` transmitted at `f0_hz`
/// (drifting `drift_hz_per_s`): the running sum of the phase rate, built as a
/// chunked scan — chunk sums in parallel, a short sequential scan over the chunk
/// offsets, then each chunk integrated in parallel — and carried in `f64`.
fn phase_track(tones: &[u8], f0_hz: f32, drift_hz_per_s: f32, g: Geometry) -> Vec<f64> {
    let nwave = tones.len() * g.nsps;
    let pulse = g.pulse();
    let carrier = TAU * f64::from(f0_hz) / g.fs;
    // frequency f(t) = f0 + drift·t: the phase rate grows by 2π·drift/fs² per sample
    let slope = TAU * f64::from(drift_hz_per_s) / (g.fs * g.fs);

    let dphi = collect_range(nwave, 2 * g.nsps, |n| {
        dphi_at(n, tones, &pulse, g.nsps, carrier, slope)
    });

    let chunk = g.nsps;
    let sums: Vec<f64> = {
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            dphi.par_chunks(chunk).map(|c| c.iter().sum()).collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            dphi.chunks(chunk).map(|c| c.iter().sum()).collect()
        }
    };
    let starts: Vec<f64> = sums
        .iter()
        .scan(0.0, |acc, &s| {
            let start = *acc;
            *acc += s;
            Some(start)
        })
        .collect();

    let mut phase = alloc::vec![0f64; nwave];
    let integrate = |out: &mut [f64], d: &[f64], start: f64| {
        d.iter()
            .scan(start, |p, &x| {
                let now = *p;
                *p += x;
                Some(now)
            })
            .zip(out.iter_mut())
            .for_each(|(p, o)| *o = p);
    };
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        phase
            .par_chunks_mut(chunk)
            .zip(dphi.par_chunks(chunk))
            .zip(starts.par_iter())
            .for_each(|((o, d), &s)| integrate(o, d, s));
    }
    #[cfg(not(feature = "parallel"))]
    {
        phase
            .chunks_mut(chunk)
            .zip(dphi.chunks(chunk))
            .zip(&starts)
            .for_each(|((o, d), &s)| integrate(o, d, s));
    }
    phase
}

/// The half-cosine envelope over the first and last `nsps/8` samples
/// (`nramp = nint(nsps/8)`): `gain(i, n)` for sample `i` of `n`.
fn ramp_gain(i: usize, n: usize, nsps: usize) -> f64 {
    let nramp = nsps / 8;
    let up = |k: usize| (1.0 - (TAU * k as f64 / (2.0 * nramp as f64)).cos()) / 2.0;
    if i < nramp {
        up(i)
    } else if i >= n - nramp {
        1.0 - up(i - (n - nramp))
    } else {
        1.0
    }
}

/// Audio for a tone sequence: real samples at 12 kHz, peak about `amplitude`,
/// `f0_hz` the frequency of tone 0 (the lowest); the others are
/// `f0_hz + tone · 31.25 Hz`. Empty for an empty sequence.
pub fn synth_f32(tones: &[u8], f0_hz: f32, amplitude: f32) -> Vec<f32> {
    synth_drifting_f32(tones, f0_hz, amplitude, 0.0)
}

/// [`synth_f32`] with the whole signal drifting linearly in frequency, `f0_hz`
/// at the start and `f0_hz + drift_hz_per_s · t` after `t` seconds — a
/// transmission through a Doppler shift that changes as a satellite passes.
/// Upstream's `gen_jttywave` has no such term; this is for testing receivers.
pub fn synth_drifting_f32(
    tones: &[u8],
    f0_hz: f32,
    amplitude: f32,
    drift_hz_per_s: f32,
) -> Vec<f32> {
    if tones.is_empty() {
        return Vec::new();
    }
    let g = Geometry::AUDIO;
    let phase = phase_track(tones, f0_hz, drift_hz_per_s, g);
    let (n, amplitude) = (phase.len(), f64::from(amplitude));
    phase
        .iter()
        .enumerate()
        .map(|(i, p)| (amplitude * p.sin() * ramp_gain(i, n, g.nsps)) as f32)
        .collect()
}

/// A transmission synthesised **as it is played**: the samples of [`synth_f32`] in
/// pieces of any size, the phase carried from one call to the next.
///
/// [`synth_f32`] wants the whole tone sequence and builds the whole waveform, with
/// the phase in `f64`. A transmitter that feeds a sound device wants the next 20 ms
/// (or the next frame), with little memory and no `f64`: on an Xtensa LX7 `f64` is
/// software, an order of magnitude slower than `f32`, and a 16-frame message is
/// 362 496 samples. The waveform is a pure function of the sample index — the phase
/// *rate* at sample `n` is a short sum of at most three pulse terms
/// (`dphi_at` in the whole-message code) — so only the running phase has to be
/// carried, and that is all this holds.
///
/// `F` is the precision of the phase, the pulse table and the arithmetic:
///
/// - `Synth<f64>` matches [`synth_f32`] to rounding (tested: within `1e-9` of the
///   amplitude);
/// - `Synth<f32>` splits the phase in two, because a single `f32` running sum
///   cannot be made to work: the carrier's per-sample increment (0.785 rad at
///   1500 Hz) is not representable, and its representation error, `5e-8` rad per
///   sample, is *systematic* — 17 milliradians after 30 s, measured, a frequency
///   error of 0.0001 Hz that no receiver would notice but that makes the samples
///   differ from the `f64` ones. So the carrier is a 64-bit number-controlled
///   oscillator (an integer add per sample, exact to `2⁻⁶⁴` of a cycle) and only the
///   *modulation* phase, whose increments are small, is a wrapped `f32`. Tested
///   against [`synth_f32`] over the longest message (16 frames, 30 s, text whose
///   tones average high, the worst case for the modulation sum): the worst sample
///   differs by 4.4e-3 of the amplitude at 300, 1500 and 2800 Hz alike, a phase
///   error of 4.4 milliradians at the very end, where it is a slow drift rather
///   than noise — and the message decodes on the receiver here.
///
/// The output does not depend on how the samples are asked for: any chunking of
/// the same message gives identical bytes.
pub struct Synth<F: Float = f64> {
    tones: Vec<u8>,
    pulse: Vec<F>,
    /// Samples of the whole transmission.
    total: usize,
    /// Next sample to produce.
    n: usize,
    /// The modulation phase at sample `n`, in `[0, 2π)`: the running sum of the
    /// pulse terms alone.
    phase: F,
    /// The carrier, as a fraction of a cycle in 64 bits (an NCO), and its increment
    /// per sample.
    cphase: u64,
    cinc: u64,
    amplitude: F,
    /// `2π · HMOD / nsps`.
    k: F,
    /// Samples per symbol.
    nsps: usize,
}

impl<F: Float> Synth<F> {
    /// A transmission of `tones` (any number of whole frames) at `f0_hz`, the
    /// frequency of tone 0, with peak `amplitude`. Empty tones give an empty
    /// transmission.
    pub fn new(tones: &[u8], f0_hz: f32, amplitude: f32) -> Self {
        Self::with_geometry(tones, f0_hz, amplitude, Geometry::AUDIO)
    }

    /// [`Self::new`] at `nsps` samples per symbol and `fs_hz` samples per second: the
    /// 6 kHz, 192-sample analytic-signal geometry the receiver subtracts its reference
    /// in (`fill_complex`), or the 12 kHz audio one.
    pub fn at(tones: &[u8], f0_hz: f32, amplitude: f32, nsps: usize, fs_hz: f32) -> Self {
        Self::with_geometry(
            tones,
            f0_hz,
            amplitude,
            Geometry {
                nsps,
                fs: f64::from(fs_hz),
            },
        )
    }

    fn with_geometry(tones: &[u8], f0_hz: f32, amplitude: f32, g: Geometry) -> Self {
        let cast = |x: f64| F::from(x).expect("a float");
        Self {
            tones: tones.to_vec(),
            pulse: g.pulse().into_iter().map(|p| cast(f64::from(p))).collect(),
            total: tones.len() * g.nsps,
            n: 0,
            phase: F::zero(),
            cphase: 0,
            // f0 / fs of a cycle, in 2⁻⁶⁴ cycles
            cinc: (f64::from(f0_hz) / g.fs * 18_446_744_073_709_551_616.0) as u64,
            amplitude: cast(f64::from(amplitude)),
            k: cast(TAU * HMOD / g.nsps as f64),
            nsps: g.nsps,
        }
    }

    /// Samples in the whole transmission.
    pub fn total_samples(&self) -> usize {
        self.total
    }

    /// Samples not yet produced.
    pub fn remaining(&self) -> usize {
        self.total - self.n
    }

    /// Whether every sample has been produced.
    pub fn is_finished(&self) -> bool {
        self.n >= self.total
    }

    /// The angle (carrier plus modulation) and the envelope of the next sample, and advance.
    fn step(&mut self) -> (F, F) {
        let nsps = self.nsps;
        let nsym = self.tones.len();
        let tau = F::from(TAU).expect("a float");
        // radians per unit of the carrier's top 32 bits
        let unit = F::from(TAU / 4_294_967_296.0).expect("a float");
        let n = self.n;
        // the carrier's angle from its top 32 bits: resolution 1.5e-9 cycles
        let carrier = F::from((self.cphase >> 32) as u32).expect("a float") * unit;
        let out = (self.phase + carrier, ramp(n, self.total, nsps));

        // the phase rate at this sample: a sum of at most three pulse terms
        let p = nsps + n;
        let q = p / nsps;
        let term = |tone: u8, m: usize| F::from(tone).expect("a float") * self.pulse[m];
        let mut body = F::zero();
        for j in q.saturating_sub(2)..=q {
            if j < nsym {
                body = body + term(self.tones[j], p - j * nsps);
            }
        }
        if p < 2 * nsps {
            body = body + term(self.tones[0], nsps + p);
        }
        if p >= nsym * nsps {
            body = body + term(self.tones[nsym - 1], p - nsym * nsps);
        }
        self.cphase = self.cphase.wrapping_add(self.cinc);
        self.phase = self.phase + self.k * body;
        // keep the running phase small: in f32 its resolution is what limits the
        // waveform, and a wrapped phase has the same sine
        if self.phase >= tau {
            self.phase = self.phase - tau;
        } else if self.phase < F::zero() {
            self.phase = self.phase + tau;
        }
        self.n += 1;
        out
    }

    /// Write the next `min(out.len(), remaining())` samples into `out`, returning
    /// how many. `0` once the transmission is over.
    pub fn fill(&mut self, out: &mut [f32]) -> usize {
        let count = out.len().min(self.remaining());
        for o in &mut out[..count] {
            let (angle, gain) = self.step();
            *o = (self.amplitude * angle.sin() * gain)
                .to_f32()
                .unwrap_or(0.0);
        }
        count
    }

    /// [`Self::fill`] as the complex baseband waveform `amplitude · gain · exp(jθ)` — what
    /// [`synth_complex`] returns, in the caller's precision and in pieces.
    pub fn fill_complex(&mut self, out: &mut [Complex32]) -> usize {
        let count = out.len().min(self.remaining());
        for o in &mut out[..count] {
            let (angle, gain) = self.step();
            let g = self.amplitude * gain;
            *o = Complex32::new(
                (g * angle.cos()).to_f32().unwrap_or(0.0),
                (g * angle.sin()).to_f32().unwrap_or(0.0),
            );
        }
        count
    }
}

/// `ramp_gain` in the precision of the caller's choice of `f32` output: the
/// half-cosine envelope over the first and last `nsps/8` samples.
fn ramp<F: Float>(i: usize, n: usize, nsps: usize) -> F {
    let nramp = nsps / 8;
    if i >= nramp && i < n - nramp {
        return F::one();
    }
    F::from(ramp_gain(i, n, nsps)).expect("a float")
}

/// The complex baseband waveform of `tones` at `f0_hz`, unit amplitude, built
/// with `nsps` samples per symbol at `sample_rate_hz` — the reference a receiver
/// subtracts (`gen_jttywave` with `icmplx = 1`; the receiver's analytic signal is
/// at 6 kHz, 192 samples per symbol). `nsps` must be a multiple of 8.
pub fn synth_complex(tones: &[u8], f0_hz: f32, nsps: usize, sample_rate_hz: f32) -> Vec<Complex32> {
    assert!(
        nsps > 0 && nsps.is_multiple_of(8),
        "nsps must be a multiple of 8"
    );
    if tones.is_empty() {
        return Vec::new();
    }
    let g = Geometry {
        nsps,
        fs: f64::from(sample_rate_hz),
    };
    let phase = phase_track(tones, f0_hz, 0.0, g);
    let n = phase.len();
    phase
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let r = ramp_gain(i, n, nsps);
            Complex32::new((r * p.cos()) as f32, (r * p.sin()) as f32)
        })
        .collect()
}

/// Length in samples of a transmission of `frames` frames, `1.888 s` each.
pub const fn samples_for_frames(frames: usize) -> usize {
    frames * FRAME_SYMBOLS * NSPS
}

/// Bits the encoder consumes per frame, re-exported for callers sizing buffers.
pub const DATA_TONES: usize = INFO_BITS;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jtty::source::{CallAction, Role, decode_payload};

    fn cq() -> Atom {
        Atom::call(CallAction::Cq, "K1ABC")
    }

    #[test]
    fn one_frame_is_sync_then_46_data_tones() {
        let t = tones(&[cq()]).unwrap();
        assert_eq!(t.len(), FRAME_SYMBOLS);
        assert_eq!(&t[..SYNC_SYMBOLS], &SYNC[..]);
        assert!(t.iter().all(|&x| x < 4));
    }

    #[test]
    fn message_length_limits() {
        assert!(tones(&[]).is_none());
        assert!(payloads(&vec![cq(); MAX_FRAMES]).is_some());
        assert!(payloads(&vec![cq(); MAX_FRAMES + 1]).is_none());
        // an atom that cannot be encoded spoils the message
        assert!(tones(&[cq(), Atom::call(CallAction::Cq, "NOTACALL")]).is_none());
    }

    #[test]
    fn only_the_last_frame_carries_eom() {
        let atoms = vec![cq(), cq(), cq()];
        let p = payloads(&atoms).unwrap();
        let eoms: Vec<bool> = p.iter().map(|p| decode_payload(p).unwrap().1).collect();
        assert_eq!(eoms, [false, false, true]);
    }

    #[test]
    fn parallel_and_sequential_agree_exactly() {
        // 16 frames of different content: order and content must be exactly
        // what a plain sequential loop produces, whatever the thread count.
        let atoms: Vec<Atom> = (0..MAX_FRAMES)
            .map(|i| Atom::Number {
                role: Role::Full,
                kind: crate::jtty::source::NumberKind::Serial,
                value: (i * 977) as u32,
            })
            .collect();
        let want: Vec<u8> = atoms
            .iter()
            .enumerate()
            .flat_map(|(i, a)| frame_tones(&a.encode(i == atoms.len() - 1).unwrap()))
            .collect();
        assert_eq!(tones(&atoms).unwrap(), want);
    }

    /// The chunked scan against a plain sample-by-sample running sum.
    #[test]
    fn chunked_phase_scan_equals_a_plain_running_sum() {
        let t = tones(&[cq(), cq()]).unwrap();
        let got = synth_f32(&t, 1500.0, 1.0);

        let pulse: Vec<f32> = (1..=3 * NSPS)
            .map(|i| gfsk_pulse(BT, (i as f32 - 1.5 * NSPS as f32) / NSPS as f32))
            .collect();
        let carrier = TAU * 1500.0 / f64::from(SAMPLE_RATE);
        let mut phase = 0.0f64;
        let mut want: Vec<f32> = (0..t.len() * NSPS)
            .map(|n| {
                let s = phase.sin() as f32;
                phase += dphi_at(n, &t, &pulse, NSPS, carrier, 0.0);
                s
            })
            .collect();
        let nramp = NSPS / 8;
        let env = |i: usize| ((1.0 - (TAU * i as f64 / (2.0 * nramp as f64)).cos()) / 2.0) as f32;
        (0..nramp).for_each(|i| want[i] *= env(i));
        let n = want.len();
        (0..nramp).for_each(|i| want[n - nramp + i] *= 1.0 - env(i));

        let worst = got
            .iter()
            .zip(&want)
            .map(|(a, b)| (a - b).abs())
            .fold(0f32, f32::max);
        assert!(worst < 1e-6, "worst {worst}");
    }

    /// The result must not depend on how many threads ran it.
    #[cfg(feature = "parallel")]
    #[test]
    fn synthesis_is_identical_for_any_thread_count() {
        let atoms: Vec<Atom> = (0..6).map(|_| cq()).collect();
        let t = tones(&atoms).unwrap();
        let run = |threads: usize| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(threads)
                .build()
                .unwrap()
                .install(|| (tones(&atoms).unwrap(), synth_f32(&t, 1234.5, 0.7)))
        };
        let one = run(1);
        for threads in [2, 3, 8] {
            assert_eq!(run(threads), one, "{threads} threads");
        }
    }

    #[test]
    fn waveform_has_the_right_length_and_peak() {
        let t = tones(&[cq()]).unwrap();
        let w = synth_f32(&t, 1500.0, 0.5);
        assert_eq!(w.len(), samples_for_frames(1));
        let peak = w.iter().fold(0f32, |m, &x| m.max(x.abs()));
        assert!((0.45..=0.5001).contains(&peak), "peak {peak}");
    }

    /// Everything a `Synth` produces, asked for in pieces of `chunk` samples.
    fn play<F: Float>(tones: &[u8], chunk: usize) -> Vec<f32> {
        let mut s = Synth::<F>::new(tones, 1500.0, 1.0);
        let mut out = Vec::new();
        let mut buf = alloc::vec![0f32; chunk];
        loop {
            let n = s.fill(&mut buf);
            if n == 0 {
                break;
            }
            out.extend_from_slice(&buf[..n]);
        }
        assert!(s.is_finished() && s.remaining() == 0);
        out
    }

    #[test]
    fn synth_is_the_same_however_it_is_asked_for() {
        let t = tones(&[cq(), cq()]).unwrap();
        let whole = play::<f32>(&t, 1 << 20);
        assert_eq!(whole.len(), samples_for_frames(2));
        for chunk in [1, 7, 480, 4096, NSPS * FRAME_SYMBOLS] {
            assert_eq!(play::<f32>(&t, chunk), whole, "f32, chunks of {chunk}");
        }
        let whole = play::<f64>(&t, 1 << 20);
        for chunk in [1, 333, 22_656] {
            assert_eq!(play::<f64>(&t, chunk), whole, "f64, chunks of {chunk}");
        }
    }

    #[test]
    fn synth_f64_is_the_whole_message_synthesis() {
        let t = tones(&[cq(), cq(), cq()]).unwrap();
        let want = synth_f32(&t, 1500.0, 1.0);
        let got = play::<f64>(&t, 5000);
        assert_eq!(got.len(), want.len());
        let worst = got
            .iter()
            .zip(&want)
            .map(|(a, b)| (a - b).abs())
            .fold(0.0, f32::max);
        assert!(worst < 1e-6, "worst sample difference {worst}");
    }

    /// The longest message there is: 16 frames, 30 s. A single `f32` running phase drifts
    /// 17 milliradians from the `f64` one by the end from the carrier increment's
    /// representation error alone; with the carrier in an integer oscillator what is
    /// left is 4.4.
    #[test]
    fn synth_f32_stays_within_a_few_milliradians_over_sixteen_frames() {
        use crate::jtty::pack::{ExchangeProfile, tones as text_tones};
        // eighty characters of text: 16 frames of TEXT5, tones averaging high
        let t = text_tones(&"X".repeat(80), ExchangeProfile::Unknown)
            .unwrap()
            .unwrap();
        assert_eq!(t.len(), MAX_FRAMES * FRAME_SYMBOLS);
        for f0 in [300.0, 1500.0, 2800.0] {
            let want = synth_f32(&t, f0, 1.0);
            let mut synth = Synth::<f32>::new(&t, f0, 1.0);
            let mut got = alloc::vec![0f32; want.len()];
            assert_eq!(synth.fill(&mut got), want.len());
            let worst = got
                .iter()
                .zip(&want)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f32::max);
            // measured 4.4e-3, at every carrier
            assert!(worst < 6e-3, "{f0} Hz: worst sample difference {worst}");
        }
    }

    #[test]
    fn synth_starts_and_ends_at_zero_and_stops() {
        let t = tones(&[cq()]).unwrap();
        let s = play::<f32>(&t, 1000);
        assert!(s[0].abs() < 1e-6 && s.last().unwrap().abs() < 1e-3);
        assert!(s.iter().all(|x| x.abs() <= 1.0 + 1e-6));
        let mut synth = Synth::<f32>::new(&t, 1500.0, 1.0);
        let mut buf = alloc::vec![0f32; 100_000];
        assert_eq!(synth.fill(&mut buf), samples_for_frames(1));
        assert_eq!(synth.fill(&mut buf), 0);
        // no tones, no transmission
        let mut empty = Synth::<f32>::new(&[], 1500.0, 1.0);
        assert!(empty.is_finished());
        assert_eq!(empty.fill(&mut buf), 0);
    }
}
