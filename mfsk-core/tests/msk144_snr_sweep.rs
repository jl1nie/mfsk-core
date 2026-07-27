//! MSK144 threshold characterisation — synthetic SNR sweep.
//!
//! Self-contained (no WSJT-X build dependency): synthesizes the same
//! kind of signal WSJT-X's own `msk144sim.f90` generates -- multiple
//! meteor pings across a slot, `2.718*t*exp(-t)` envelope, `snrdb`
//! calibrated to WSJT-X's 2500 Hz reference bandwidth via the same
//! `fac = sqrt(6000/2500)` noise-power normalization `msk144sim.f90`
//! uses -- so numbers here are directly comparable to a real WSJT-X
//! (`jt9 -k`) run without a separate dB-convention translation.
//!
//! Provenance (2026-07-19, issue #156): a `jt9 -k` binary was built
//! from the WSJT-X source tree specifically to cross-validate this
//! crate's `decode_slot()` against WSJT-X's own decoder on
//! `msk144sim`-generated WAVs at known SNR. Result: 25 of 28
//! (config, SNR) cells matched exactly, the other 3 differed by
//! exactly 1 file out of 20 (a single borderline noise realization at
//! the steepest part of the threshold curve) -- i.e. no measurable
//! recall gap between this port and WSJT-X at any tested SNR. This
//! test reproduces that same synthetic-signal recipe as an ongoing
//! regression characterisation, without requiring a WSJT-X checkout
//! to run in CI.
//!
//! Like `ft4_snr_sweep.rs`/`fst4_sweep.rs`/`ft8_sweep.rs`, this prints
//! a recall table and does not hard-assert a threshold value --
//! floating-point/FFT-backend-sensitive threshold dB figures are not
//! a stable regression gate the way message-level golden-WAV recall
//! is; watch the printed table by eye for a session that shows a
//! multi-dB regression from the 2026-07-19 baseline above.

use mfsk_core::engine::FecCodec;
use mfsk_core::engine::dsp::msk::build_bitseq;
use mfsk_core::fec::Ldpc128_90;
use mfsk_core::fec::ldpc_128_90::crc13;
use mfsk_core::msg::wsjt77::pack77;
use mfsk_core::msk144::decode::{Depth, decode_slot};

const FS: f32 = 12_000.0;
const REF_BW: f32 = 2_500.0;
const NYQUIST: f32 = FS / 2.0;
const SEEDS: u64 = 20;

/// xorshift32 + Box-Muller, matching the deterministic-noise pattern
/// used by every other sweep test in this suite (`ft4_snr_sweep.rs`
/// etc.) -- reproducible across runs, no external RNG dependency.
struct Lcg {
    state: u32,
    spare: Option<f32>,
}
impl Lcg {
    fn new(seed: u32) -> Self {
        Self {
            state: seed | 1,
            spare: None,
        }
    }
    fn next_u32(&mut self) -> u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        self.state
    }
    fn uniform(&mut self) -> f32 {
        (self.next_u32() as f32 + 1.0) / (u32::MAX as f32 + 2.0)
    }
    fn gauss(&mut self) -> f32 {
        if let Some(x) = self.spare.take() {
            return x;
        }
        let u1 = self.uniform();
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * core::f32::consts::PI * u2;
        self.spare = Some(r * theta.sin());
        r * theta.cos()
    }
}

/// `mskrtd.f90`-natural-bitseq -> the itone[] sequence WSJT-X's own
/// `genmsk_128_90` would produce (`msk144sim.f90:52-76`'s input),
/// recovered via the differential relation `msk144::decode`'s own
/// independent-oracle test uses. Lets us drive the *same* simple
/// binary-FSK reference synthesizer `msk144sim` uses, rather than
/// this crate's own OQPSK TX path, so recall numbers aren't
/// self-fulfilling against our own modulator's idiosyncrasies.
fn build_i4tone(bitseq_natural: &[u8; 144]) -> [u8; 144] {
    let mut bp = [0i8; 144];
    for i in 0..144 {
        bp[i] = 2 * bitseq_natural[i] as i8 - 1;
    }
    let mut i4tone = [0i8; 144];
    for i in 1..=72usize {
        let b_2i_minus_1 = bp[2 * i - 2];
        let b_2i = bp[2 * i - 1];
        let b_wrap = bp[(2 * i) % 144];
        i4tone[2 * i - 2] = (b_2i * b_2i_minus_1 + 1) / 2;
        i4tone[2 * i - 1] = -((b_2i * b_wrap - 1) / 2);
    }
    let mut out = [0u8; 144];
    for i in 0..144 {
        out[i] = (-i4tone[i] + 1) as u8;
    }
    out
}

fn build_codeword_itone(call1: &str, call2: &str, report: &str) -> [u8; 144] {
    let msg77 = pack77(call1, call2, report).expect("valid message");
    let mut info = [0u8; 90];
    info[..77].copy_from_slice(&msg77);
    let mut bytes = [0u8; 12];
    for (i, &b) in info[..77].iter().enumerate() {
        let byte_idx = i / 8;
        let bit_pos = 7 - (i % 8);
        bytes[byte_idx] |= (b & 1) << bit_pos;
    }
    let crc = crc13(&bytes);
    for i in 0..13 {
        info[77 + i] = ((crc >> (12 - i)) & 1) as u8;
    }
    let mut codeword = [0u8; 128];
    Ldpc128_90.encode(&info, &mut codeword);
    let bitseq = build_bitseq(&codeword);
    build_i4tone(&bitseq)
}

/// Continuous-phase binary-FSK waveform for one 144-symbol frame
/// (`msk144sim.f90:52-76`), unit peak amplitude.
fn synth_frame_waveform(itone: &[u8; 144], freq_hz: f32, phi0: f32) -> (Vec<f32>, f32) {
    let twopi = 2.0 * core::f32::consts::PI;
    let baud = 2000.0f32;
    let dphi0 = twopi * (freq_hz - 0.25 * baud) / FS;
    let dphi1 = twopi * (freq_hz + 0.25 * baud) / FS;
    let mut phi = phi0;
    let mut out = Vec::with_capacity(144 * 6);
    for &tone in itone {
        let dphi = if tone == 0 { dphi0 } else { dphi1 };
        for _ in 0..6 {
            out.push(phi.cos());
            phi += dphi;
            if phi >= twopi {
                phi -= twopi;
            }
        }
    }
    (out, phi)
}

/// A full `ntr_period`-second slot: a meteor ping (`2.718*t*exp(-t)`
/// envelope, `msk144sim.f90`'s `makepings`) once per second at
/// `t=1..ntr_period-1`, `width`-seconds decay constant, AWGN at
/// `snr_db` (WSJT-X's 2500 Hz-referenced convention). `nfqso=1500 Hz`.
fn make_slot(itone: &[u8; 144], ntr_period: usize, width: f32, snr_db: f32, seed: u32) -> Vec<i16> {
    let npts = ntr_period * FS as usize;
    let sig = 2f32.sqrt() * 10f32.powf(0.05 * snr_db);
    let fac = (NYQUIST / REF_BW).sqrt();

    // Continuously-repeating carrier phase across the whole slot so
    // each ping instance samples a different, but phase-continuous,
    // point in the frame cycle -- matches `msk144sim`'s single
    // `waveform(k)` array built once over the whole `npts` buffer.
    let mut waveform = Vec::with_capacity(npts);
    let mut phi = 0.0f32;
    while waveform.len() < npts {
        let (frame, next_phi) = synth_frame_waveform(itone, 1500.0, phi);
        waveform.extend_from_slice(&frame);
        phi = next_phi;
    }
    waveform.truncate(npts);

    // `2.718`, not `std::f32::consts::E` (2.71828...) -- matching
    // `makepings.f90`'s literal exactly rather than a "cleaner" but
    // numerically different constant.
    #[allow(clippy::approx_constant)]
    const PING_ENVELOPE_SCALE: f32 = 2.718;

    let mut rng = Lcg::new(seed);
    let mut out = Vec::with_capacity(npts);
    for (i, &wf) in waveform.iter().enumerate().take(npts) {
        let iping = ((i / 12_000).max(1)).min(ntr_period - 1);
        let t = (i as f32 / FS - iping as f32) / width;
        let env = if !(0.0..=10.0).contains(&t) {
            0.0
        } else {
            PING_ENVELOPE_SCALE * t * (-t).exp()
        };
        let s = env * sig * wf + fac * rng.gauss();
        out.push((30.0 * s).clamp(i16::MIN as f32, i16::MAX as f32) as i16);
    }
    out
}

fn hit(decodes: &[mfsk_core::msk144::decode::SlotDecode], expected: &str) -> bool {
    decodes.iter().any(|d| d.message == expected)
}

fn run_sweep(name: &str, ntr_period: usize, width: f32) {
    let itone = build_codeword_itone("K1ABC", "W9XYZ", "EN37");
    let expected = "K1ABC W9XYZ EN37";

    println!(
        "\n=== MSK144 {name} SNR sweep (TRp={ntr_period}s width={width}, {SEEDS} seeds/SNR) ==="
    );

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;

    let trial = |snr: i32, seed: u64| -> bool {
        let audio = make_slot(
            &itone,
            ntr_period,
            width,
            snr as f32,
            0xA5A5_0000 + seed as u32,
        );
        hit(&decode_slot(&audio, 1500.0, 60.0, Depth::Deep), expected)
    };

    for snr in [-9, -8, -7, -6, -5, -4, -3] {
        #[cfg(feature = "parallel")]
        let hits: u64 = (0..SEEDS)
            .into_par_iter()
            .filter(|&s| trial(snr, s))
            .count() as u64;

        #[cfg(not(feature = "parallel"))]
        let hits: u64 = {
            let mut hits = 0u64;
            for seed in 0..SEEDS {
                if trial(snr, seed) {
                    hits += 1;
                }
            }
            hits
        };

        println!("  {snr:>3} dB   {hits:>3}/{SEEDS}");
    }
}

#[test]
#[ignore = "slow: 2 configs x 7 SNR x 20 seeds; ~22s in release with `parallel` \
            (rayon), ~4.5min without. CI's \"catchall characterization\" suite \
            runs this via `--test msk144_snr_sweep -- --ignored` with \
            `--features full` (implies `parallel`)."]
fn msk144_snr_sweep_short_and_long_ping() {
    // WSJT-X's own `msk144sim` examples: short ping (0.43 s ~ TRp=15),
    // long ping (2.5 s decay ~ TRp=30).
    run_sweep("short", 15, 0.12);
    run_sweep("long", 30, 2.5);
}
