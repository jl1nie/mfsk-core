// SPDX-License-Identifier: GPL-3.0-only
//! Where one FT8 `subtract` call spends its time, on this machine (#761, #762).
//!
//! ```text
//! cargo run --release -p mfsk-core --example subtract_profile -- embedded-poc/assets/qso3_busy.wav
//! ```
//!
//! `FILE` is a 12 kHz mono `.wav` (a whole FT8 slot, 180 000 samples). It times, as the median of 40
//! calls after 3 that are not counted:
//!
//! - one forward and one inverse FFT of the slot's length (`rustfft`, what `subtract` is built on);
//! - `synth_complex_f32_into`, the reference waveform one `subtract` call builds;
//! - `subtract_tones_lpf`: the first call (cold: it builds the window FFT and the plans once) and warm;
//! - `subtract_tones_lpf_refine_dt`, what the checkpoints and `sic_early` run (four applications).
//!
//! The figures are wall-clock on the machine that runs it, one thread. Nothing asserts on them. They are
//! what `docs/notes/FT8_SLOT_PROFILE.md` reads against, and what to send back from a board.

use std::time::Instant;

use mfsk_core::encoder::encode;
use mfsk_core::engine::dsp::gfsk::{GfskCfg, synth_complex_f32_into};
use mfsk_core::engine::dsp::subtract::{
    GfskParams, SubtractCfg, subtract_tones_lpf, subtract_tones_lpf_refine_dt,
};
use mfsk_core::registry::Mode;
use rustfft::FftPlanner;
use rustfft::num_complex::Complex32;

const ITERS: usize = 40;
const WARMUP: usize = 3;

fn load(path: &str) -> Vec<i16> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
    let u32_at =
        |i: usize| u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
    assert!(
        bytes.len() > 12 && &bytes[..4] == b"RIFF" && &bytes[8..12] == b"WAVE",
        "not a WAV file"
    );
    let mut p = 12;
    while p + 8 <= bytes.len() {
        let n = u32_at(p + 4) as usize;
        if &bytes[p..p + 4] == b"data" {
            let end = (p + 8 + n).min(bytes.len());
            return bytes[p + 8..end]
                .as_chunks::<2>()
                .0
                .iter()
                .map(|c| i16::from_le_bytes(*c))
                .collect();
        }
        p += 8 + n + (n & 1);
    }
    panic!("no data chunk");
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

/// Median of `ITERS` timed calls (milliseconds) after `WARMUP` that are not counted.
fn time(mut f: impl FnMut()) -> f64 {
    let mut v = Vec::with_capacity(ITERS);
    for i in 0..WARMUP + ITERS {
        let t = Instant::now();
        f();
        if i >= WARMUP {
            v.push(t.elapsed().as_secs_f64() * 1e3);
        }
    }
    median(v)
}

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: subtract_profile FILE.wav");
    let audio0 = load(&path);
    let n = audio0.len();
    println!("{path}: {n} samples, median of {ITERS} calls, one thread\n");

    // The FFT `subtract` does twice per call (forward, then inverse) over the slot's length.
    let mut planner = FftPlanner::<f32>::new();
    let (forward, inverse) = (planner.plan_fft_forward(n), planner.plan_fft_inverse(n));
    let mut buf: Vec<Complex32> = (0..n)
        .map(|i| Complex32::new((i % 97) as f32, (i % 89) as f32))
        .collect();
    let fft_forward = time(|| forward.process(&mut buf));
    for v in buf.iter_mut() {
        *v *= 1.0 / n as f32;
    }
    let fft_inverse = time(|| inverse.process(&mut buf));
    println!("rustfft {n} points: forward {fft_forward:.3} ms, inverse {fft_inverse:.3} ms");

    // FT8's subtract configuration (`ft8/subtract.rs`), a decoded message's tones.
    let tones = encode(Mode::Ft8, "CQ JA1ABC PM95")
        .expect("encodes")
        .tones()
        .to_vec();
    let cfg = SubtractCfg {
        sample_rate: 12_000.0,
        tone_spacing_hz: 6.25,
        samples_per_symbol: 1920,
        base_offset_s: 0.5,
        gfsk: Some(GfskParams {
            bt: 2.0,
            hmod: 1.0,
            ramp_samples: 1920 / 8,
            frame_ramp: false,
        }),
    };

    let gcfg = GfskCfg {
        sample_rate: 12_000.0,
        samples_per_symbol: 1920,
        bt: 2.0,
        hmod: 1.0,
        ramp_samples: 1920 / 8,
    };
    let nframe = tones.len() * 1920;
    let (mut cos, mut sin) = (vec![0.0f32; nframe], vec![0.0f32; nframe]);
    let synth = time(|| synth_complex_f32_into(&mut cos, &mut sin, &tones, 1500.0, 1.0, &gcfg));
    println!("reference waveform (synth_complex_f32_into, {nframe} samples): {synth:.3} ms");

    let mut cold = f64::NAN;
    let mut warm_runs = Vec::with_capacity(ITERS);
    for i in 0..WARMUP + ITERS {
        let mut audio = audio0.clone();
        let t = Instant::now();
        subtract_tones_lpf(&mut audio, &tones, 1500.0, 0.1, &cfg, 2000, true);
        let ms = t.elapsed().as_secs_f64() * 1e3;
        if i == 0 {
            cold = ms;
        }
        if i >= WARMUP {
            warm_runs.push(ms);
        }
    }
    let warm = median(warm_runs);
    println!("subtract_tones_lpf: first call (cold) {cold:.3} ms, warm {warm:.3} ms");
    println!(
        "  the FFT pair is {:.0} % of a warm call, the reference {:.0} %, the rest {:.0} %",
        (fft_forward + fft_inverse) / warm * 100.0,
        synth / warm * 100.0,
        (warm - fft_forward - fft_inverse - synth) / warm * 100.0
    );

    let mut refine = Vec::with_capacity(ITERS);
    for i in 0..WARMUP + ITERS {
        let mut audio = audio0.clone();
        let t = Instant::now();
        subtract_tones_lpf_refine_dt(&mut audio, &tones, 1500.0, 0.1, &cfg, 2000, true);
        if i >= WARMUP {
            refine.push(t.elapsed().as_secs_f64() * 1e3);
        }
    }
    println!(
        "subtract_tones_lpf_refine_dt (the checkpoints): warm {:.3} ms",
        median(refine)
    );
}
