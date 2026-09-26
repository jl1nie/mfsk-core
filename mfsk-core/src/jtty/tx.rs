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
//! That synthesiser samples the Gaussian pulse one sample earlier than every
//! upstream `gen_*wave.f90` (#482) — 0.049 rad for JTTY's largest tone step,
//! which is what a comparison against `sjtty`'s noiseless output showed (worst
//! normalised sample error 4.9e-2, against 1.1e-4 with the pulse index
//! corrected). The waveform is also what a receiver's subtraction will use as
//! its reference, so JTTY carries its own, faithful to `gen_jttywave.f90`.

use alloc::vec::Vec;
use core::f64::consts::TAU;

use super::source::Atom;
use super::{
    FRAME_SYMBOLS, INFO_BITS, MAX_FRAMES, NSPS, Payload, SAMPLE_RATE, SYNC, SYNC_SYMBOLS, crc, tbcc,
};
use crate::engine::dsp::gfsk::gfsk_pulse;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// Gaussian bandwidth-time product (`bt` in `sjtty` / the GUI).
const BT: f32 = 2.0;
/// Modulation index: tone spacing equals the symbol rate.
const HMOD: f64 = 1.0;
/// Samples per scan chunk when integrating the phase (one symbol).
const SCAN_CHUNK: usize = NSPS;

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
fn collect_range<R: Send>(len: usize, f: impl Fn(usize) -> R + Sync + Send) -> Vec<R> {
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        (0..len)
            .into_par_iter()
            .with_min_len(2 * NSPS)
            .map(f)
            .collect()
    }
    #[cfg(not(feature = "parallel"))]
    {
        (0..len).map(f).collect()
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
fn dphi_at(n: usize, tones: &[u8], pulse: &[f32], carrier: f64) -> f64 {
    let nsym = tones.len();
    let p = NSPS + n;
    let q = p / NSPS;
    let term = |tone: u8, m: usize| f64::from(tone) * f64::from(pulse[m]);
    // symbols j with j·nsps ≤ p < (j + 3)·nsps
    let body: f64 = (q.saturating_sub(2)..=q)
        .filter(|&j| j < nsym)
        .map(|j| term(tones[j], p - j * NSPS))
        .sum();
    // dummy symbol before the first / after the last
    let head = if p < 2 * NSPS {
        term(tones[0], NSPS + p)
    } else {
        0.0
    };
    let tail = if p >= nsym * NSPS {
        term(tones[nsym - 1], p - nsym * NSPS)
    } else {
        0.0
    };
    carrier + TAU * HMOD / NSPS as f64 * (body + head + tail)
}

/// Integrate `dphi` (exclusive running sum from `start`) and write
/// `amplitude · sin(phase)` into `out`.
fn integrate_chunk(out: &mut [f32], dphi: &[f64], start: f64, amplitude: f64) {
    dphi.iter()
        .scan(start, |phase, &d| {
            let now = *phase;
            *phase += d;
            Some(now)
        })
        .zip(out.iter_mut())
        .for_each(|(phase, o)| *o = (amplitude * phase.sin()) as f32);
}

/// Audio for a tone sequence: real samples at 12 kHz, peak about `amplitude`,
/// `f0_hz` the frequency of tone 0 (the lowest); the others are
/// `f0_hz + tone · 31.25 Hz`. Empty for an empty sequence.
pub fn synth_f32(tones: &[u8], f0_hz: f32, amplitude: f32) -> Vec<f32> {
    if tones.is_empty() {
        return Vec::new();
    }
    let nwave = tones.len() * NSPS;
    // gen_jttywave.f90: pulse(i), i = 1..3·nsps, at tt = (i − 1.5·nsps)/nsps
    let pulse: Vec<f32> = (1..=3 * NSPS)
        .map(|i| gfsk_pulse(BT, (i as f32 - 1.5 * NSPS as f32) / NSPS as f32))
        .collect();
    let carrier = TAU * f64::from(f0_hz) / f64::from(SAMPLE_RATE);

    let dphi = collect_range(nwave, |n| dphi_at(n, tones, &pulse, carrier));

    // Chunked scan: the phase at the start of each chunk is the sum of the
    // chunks before it.
    let sums: Vec<f64> = {
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            dphi.par_chunks(SCAN_CHUNK)
                .map(|c| c.iter().sum())
                .collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            dphi.chunks(SCAN_CHUNK).map(|c| c.iter().sum()).collect()
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

    let amplitude = f64::from(amplitude);
    let mut out = alloc::vec![0f32; nwave];
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        out.par_chunks_mut(SCAN_CHUNK)
            .zip(dphi.par_chunks(SCAN_CHUNK))
            .zip(starts.par_iter())
            .for_each(|((o, d), &s)| integrate_chunk(o, d, s, amplitude));
    }
    #[cfg(not(feature = "parallel"))]
    {
        out.chunks_mut(SCAN_CHUNK)
            .zip(dphi.chunks(SCAN_CHUNK))
            .zip(&starts)
            .for_each(|((o, d), &s)| integrate_chunk(o, d, s, amplitude));
    }

    // Half-cosine envelope over the first and last nsps/8 samples.
    let nramp = NSPS / 8;
    let env = |i: usize| (1.0 - (TAU * i as f64 / (2.0 * nramp as f64)).cos()) / 2.0;
    out[..nramp]
        .iter_mut()
        .enumerate()
        .for_each(|(i, o)| *o *= env(i) as f32);
    out[nwave - nramp..]
        .iter_mut()
        .enumerate()
        .for_each(|(i, o)| *o *= (1.0 - env(i)) as f32);
    out
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
                phase += dphi_at(n, &t, &pulse, carrier);
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
}
