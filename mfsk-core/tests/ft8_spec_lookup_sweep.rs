//! Phase 1.7.7-Stick host validation — DT sweep comparing the
//! current BASIS-based `fill_symbol_spectra_generic` (per-symbol
//! direct DFT, ground truth) against a proposed **spec-lookup +
//! DT phase correction** algorithm that reuses the existing
//! 3840-pt FFT spectrogram.
//!
//! The proposal: store the stage1_inc FFT output as **complex**
//! `(re, im)` rather than `|X|²`, then per-symbol per-tone is just
//! `spec[col, bin] * exp(j·2π·delta·bin / NFFT)` where `delta` is
//! the sub-NSTEP residual sample offset. No BASIS scratch needed
//! (= 120 KB internal DRAM freed on embedded target).
//!
//! Expected sweep behaviour:
//!   - dt close to integer multiples of NSTEP → ~0 dB loss
//!   - dt at ±NSTEP/2 offset → ~2-3 dB loss (window mismatch)
//!   - Average across uniform dt → ~1-1.5 dB loss
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft8 \
//!     --test ft8_spec_lookup_sweep -- --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]
#![allow(
    clippy::needless_range_loop,
    clippy::manual_memcpy,
    clippy::explicit_auto_deref
)]

use mfsk_core::core::scalar::Cmplx;
use mfsk_core::ft8::decode_block::{
    SymMask, coarse_sync_with_allsum, compute_spectrogram, fill_symbol_spectra_generic,
    precompute_coarse_allsum,
};
use mfsk_core::ft8::params::{NSPS, NSTEP, NTONES};
use num_complex::Complex;
use rustfft::FftPlanner;

// Mirror the (private) constants from compute_spectrogram /
// fill_symbol_spectra. NN (= 79 FT8 symbols) is also re-derived
// here since not directly re-exported from `mfsk_core::ft8::params`.
const NN: usize = 79;
const NFFT_SPEC: usize = 3840;
const SAMPLE_RATE_HZ: f32 = 12_000.0;
const TONE_SPACING_HZ: f32 = 6.25;
const TX_START_OFFSET_S: f32 = 0.5;
const NMAX: usize = 180_000;
// NSTEP is feature-gated (NSPS/4 on host fft-rustfft, NSPS/2 on embedded
// fixed-point). We pull it from mfsk_core to stay in sync.

// asset_path — local copy of the macro from tests/common/mod.rs to
// avoid pulling in `mod common` which depends on the `uvpacket`
// feature via channel.rs.
macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Inline RIFF/WAVE loader — strict 12 kHz / mono / 16-bit PCM.
fn load_wav_i16(path: &str) -> Vec<i16> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    assert!(bytes.len() >= 12);
    assert_eq!(&bytes[0..4], b"RIFF");
    assert_eq!(&bytes[8..12], b"WAVE");
    let mut i = 12usize;
    let mut sample_rate = 0u32;
    let mut bits = 0u16;
    let mut channels = 0u16;
    let mut data_off = 0usize;
    let mut data_len = 0usize;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
        i += 8;
        if id == b"fmt " && i + 16 <= bytes.len() {
            channels = u16::from_le_bytes(bytes[i + 2..i + 4].try_into().unwrap());
            sample_rate = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap());
            bits = u16::from_le_bytes(bytes[i + 14..i + 16].try_into().unwrap());
        } else if id == b"data" {
            data_off = i;
            data_len = len.min(bytes.len() - i);
        }
        i += len + if len % 2 == 1 { 1 } else { 0 };
    }
    assert_eq!(channels, 1);
    assert_eq!(sample_rate, 12_000);
    assert_eq!(bits, 16);
    bytes[data_off..data_off + data_len]
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

/// Build a complex-valued spectrogram (= same FFT pipeline as
/// `compute_spectrogram` host f32 path but stores complex instead of
/// `|X|²`). Layout: column-major, `data[t * n_freq + f]`.
fn build_complex_spectrogram(audio: &[i16], max_freq_hz: f32) -> (Vec<Complex<f32>>, usize, usize) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let band_top_hz = max_freq_hz + (NTONES as f32) * TONE_SPACING_HZ;
    let n_freq_full = NFFT_SPEC / 2;
    let n_freq = ((band_top_hz / df).ceil() as usize + 1).min(n_freq_full);
    let n_time = NMAX / NSTEP - 3;
    let scale = 1.0f32 / 300.0;

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(NFFT_SPEC);

    let mut data = vec![Complex::new(0.0_f32, 0.0); n_freq * n_time];
    let mut buf = vec![Complex::new(0.0_f32, 0.0); NFFT_SPEC];

    for j in 0..n_time {
        let ia = j * NSTEP;
        for (k, c) in buf.iter_mut().enumerate() {
            *c = if k < NSPS {
                let sample = if ia + k < audio.len() {
                    (audio[ia + k] as f32) * scale
                } else {
                    0.0
                };
                Complex::new(sample, 0.0)
            } else {
                Complex::new(0.0, 0.0)
            };
        }
        fft.process(&mut buf);
        let row_base = j * n_freq;
        for i in 0..n_freq {
            data[row_base + i] = buf[i];
        }
    }
    (data, n_freq, n_time)
}

/// Proposed pass2 algorithm: per (sym, tone) spec lookup with DT
/// phase correction. Fills all 79 symbols regardless of mask.
fn fill_from_complex_spec_all(
    out: &mut [[Cmplx<f32>; 8]; 79],
    spec: &[Complex<f32>],
    n_freq: usize,
    n_time: usize,
    freq_hz: f32,
    dt_sec: f32,
) {
    let df = SAMPLE_RATE_HZ / NFFT_SPEC as f32;
    let bin_step = (TONE_SPACING_HZ / df).round() as i32; // 2 for FT8
    let i0 = ((TX_START_OFFSET_S + dt_sec) * SAMPLE_RATE_HZ).round() as i64;
    let bin0 = (freq_hz / df).round() as i32;
    let two_pi_over_nfft = -2.0_f32 * core::f32::consts::PI / NFFT_SPEC as f32;

    for sym in 0..NN {
        let target_start: i64 = i0 + (sym as i64) * (NSPS as i64);
        let col_frac = target_start as f64 / NSTEP as f64;
        let col_int = col_frac.round() as i64;
        let delta_samples: i64 = target_start - col_int * (NSTEP as i64);

        if col_int < 0 || col_int >= n_time as i64 {
            for tone in 0..NTONES {
                out[sym][tone] = Complex::new(0.0, 0.0);
            }
            continue;
        }
        let col_int = col_int as usize;

        for tone in 0..NTONES {
            let bin = bin0 + (tone as i32) * bin_step;
            if bin < 0 || (bin as usize) >= n_freq {
                out[sym][tone] = Complex::new(0.0, 0.0);
                continue;
            }
            let bin_u = bin as usize;
            // Compensate for compute_spectrogram's `/300` input scaling
            // so magnitudes match the BASIS path's unscaled DFT.
            let cell = spec[col_int * n_freq + bin_u] * 300.0;

            // DT phase correction: shift theorem.
            //   FFT(x[n + delta]) = FFT(x[n]) * exp(+j·2π·delta·bin/NFFT)
            // Here delta_samples = target_start - col_start, so the
            // ideal window is `delta` samples after the col's window
            // → spec value gets rotated by +j·2π·delta·bin/NFFT.
            // (We use a negative two_pi_over_nfft so theta naturally
            // carries the correct sign.)
            let theta = two_pi_over_nfft * (delta_samples as f32) * (bin as f32);
            let phase = Complex::from_polar(1.0, theta);
            out[sym][tone] = cell * phase;
        }
    }
}

#[derive(Debug)]
struct ErrStats {
    n: usize,
    median_db: f32,
    mean_db: f32,
    abs_median_db: f32,
    p5_db: f32,
    p95_db: f32,
    min_db: f32,
    max_db: f32,
}

fn compare_per_cell(basis: &[[Cmplx<f32>; 8]; 79], lookup: &[[Cmplx<f32>; 8]; 79]) -> ErrStats {
    let mut diffs = Vec::with_capacity(NN * NTONES);
    let mut abs_diffs = Vec::with_capacity(NN * NTONES);
    for sym in 0..NN {
        for tone in 0..NTONES {
            let m_b = basis[sym][tone].norm();
            let m_l = lookup[sym][tone].norm();
            // Only count cells with non-negligible reference amplitude.
            if m_b > 1e-3 {
                let db = 20.0 * (m_l / m_b).log10();
                diffs.push(db);
                abs_diffs.push(db.abs());
            }
        }
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    abs_diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let n = diffs.len();
    if n == 0 {
        return ErrStats {
            n: 0,
            median_db: 0.0,
            mean_db: 0.0,
            abs_median_db: 0.0,
            p5_db: 0.0,
            p95_db: 0.0,
            min_db: 0.0,
            max_db: 0.0,
        };
    }
    let pct = |v: &[f32], p: f32| v[((n as f32 - 1.0) * p / 100.0).round() as usize];
    let mean = diffs.iter().sum::<f32>() / n as f32;
    ErrStats {
        n,
        median_db: pct(&diffs, 50.0),
        mean_db: mean,
        abs_median_db: pct(&abs_diffs, 50.0),
        p5_db: pct(&diffs, 5.0),
        p95_db: pct(&diffs, 95.0),
        min_db: *diffs.first().unwrap(),
        max_db: *diffs.last().unwrap(),
    }
}

/// Fill all 79 symbols of `cs` via the ground-truth (host f32
/// heterodyne) path by calling fill twice with complementary masks.
fn fill_basis_all(cs: &mut [[Cmplx<f32>; 8]; 79], audio: &[i16], freq_hz: f32, dt_sec: f32) {
    fill_symbol_spectra_generic::<f32, i16>(cs, audio, freq_hz, dt_sec, SymMask::SyncOnly);
    fill_symbol_spectra_generic::<f32, i16>(cs, audio, freq_hz, dt_sec, SymMask::DataOnly);
}

#[test]
fn dt_sweep_basis_vs_spec_lookup() {
    println!("\n=== Phase 1.7.7-Stick: spec-lookup vs BASIS DT sweep ===\n");
    println!("Loading {QSO3_PATH}");
    let audio = load_wav_i16(QSO3_PATH);
    println!(
        "  audio.len() = {} samples ({:.2} s)\n",
        audio.len(),
        audio.len() as f32 / SAMPLE_RATE_HZ
    );

    let spec_mag = compute_spectrogram(&audio, 3000.0);
    let allsum = precompute_coarse_allsum(&spec_mag, 200.0, 3000.0);
    let cands = coarse_sync_with_allsum(&spec_mag, 200.0, 3000.0, 1.0, 30, &allsum);
    println!("coarse_sync: {} candidates", cands.len());
    for (i, c) in cands.iter().take(8).enumerate() {
        println!(
            "  [{}] freq={:6.1} Hz  dt={:+5.3} s  score={:5.1}",
            i, c.freq_hz, c.dt_sec, c.score
        );
    }
    println!();

    let (spec_cplx, n_freq, n_time) = build_complex_spectrogram(&audio, 3000.0);
    assert_eq!(spec_cplx.len(), n_freq * n_time);
    assert_eq!(n_freq, spec_mag.n_freq);
    assert_eq!(n_time, spec_mag.n_time);
    println!(
        "complex spec: n_freq={} n_time={} bytes={} KB\n",
        n_freq,
        n_time,
        spec_cplx.len() * 8 / 1024
    );

    let cand = cands.first().expect("at least one candidate");
    println!(
        "--- DT sweep around top candidate freq={:.1} dt={:+.3} ---",
        cand.freq_hz, cand.dt_sec
    );
    println!(
        "NSTEP = {} samples = {:.1} ms",
        NSTEP,
        NSTEP as f32 * 1000.0 / SAMPLE_RATE_HZ
    );
    println!();
    println!("dt_off (ms) | n_cells | median | mean   | |med|  | p5     | p95    | max_loss");
    println!("------------|---------|--------|--------|--------|--------|--------|----------");

    // Sweep dt over ±NSTEP, fine enough to see structure
    let step = 60i64; // samples (= 5 ms)
    for off in (-(NSTEP as i64)..=(NSTEP as i64)).step_by(step as usize) {
        let dt_off_sec = (off as f32) / SAMPLE_RATE_HZ;
        let dt_test = cand.dt_sec + dt_off_sec;

        let mut cs_basis: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
        fill_basis_all(&mut cs_basis, &audio, cand.freq_hz, dt_test);

        let mut cs_lookup: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
        fill_from_complex_spec_all(
            &mut cs_lookup,
            &spec_cplx,
            n_freq,
            n_time,
            cand.freq_hz,
            dt_test,
        );

        let s = compare_per_cell(&cs_basis, &cs_lookup);
        let max_loss = if s.min_db.abs() > s.max_db.abs() {
            s.min_db
        } else {
            s.max_db
        };
        println!(
            "{:>+8.1}    | {:>5}   | {:+6.2} | {:+6.2} | {:>6.2} | {:+6.2} | {:+6.2} | {:+7.2}",
            off as f32 * 1000.0 / SAMPLE_RATE_HZ,
            s.n,
            s.median_db,
            s.mean_db,
            s.abs_median_db,
            s.p5_db,
            s.p95_db,
            max_loss
        );
    }

    println!("\n--- Per-candidate (top 8) at native dt ---");
    println!("idx | freq    | dt      | median | mean   | |med|  | p5     | p95    ");
    println!("----|---------|---------|--------|--------|--------|--------|--------");
    for (i, c) in cands.iter().take(8).enumerate() {
        let mut cs_basis: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
        fill_basis_all(&mut cs_basis, &audio, c.freq_hz, c.dt_sec);

        let mut cs_lookup: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::new(0.0_f32, 0.0); 8]; 79].try_into().unwrap();
        fill_from_complex_spec_all(
            &mut cs_lookup,
            &spec_cplx,
            n_freq,
            n_time,
            c.freq_hz,
            c.dt_sec,
        );

        let s = compare_per_cell(&cs_basis, &cs_lookup);
        println!(
            " {:>2} | {:6.1} | {:+6.3} | {:+6.2} | {:+6.2} | {:>6.2} | {:+6.2} | {:+6.2}",
            i, c.freq_hz, c.dt_sec, s.median_db, s.mean_db, s.abs_median_db, s.p5_db, s.p95_db
        );
    }
    println!();
}
