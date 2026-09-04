//! FT8 cold slot-phase acquisition — tiled vs wide lag search (#356b).
//!
//! A receiver with no clock can have its slot grid off by up to a full
//! FT8 half-period (±7.5 s). FT8 coarse sync normally searches ±2.5 s
//! (`SYNC_LAG_S_DEFAULT`), and `bounded_sync_lag_steps` caps a single
//! 15 s spectrogram at ~±6.28 s regardless of what lag is requested —
//! so a widened window cannot cover the whole period on its own, and
//! `bootstrap_dt_median` (a linear median) is the wrong estimator once
//! `dt` wraps.
//!
//! This measures two ways to acquire from an arbitrary phase, both
//! feeding [`circular_dt_estimate`] (the whole-period estimator, #356b
//! slice 1):
//!
//! - **wide** — one `coarse_sync_with_lag(spec, …, 7.5)` (clamped to
//!   ±6.28 s), one spectrogram;
//! - **tiled** — `ft8::acquire::acquire_slot_phase`: three ±2.5 s
//!   searches at 0 / 5 / 10 s of a 25 s capture, each window's offset
//!   folded back, union covers the full 15 s. The tested kernel
//!   unchanged (jl1nie's suggestion on the issue).
//!
//! Fixture: `qso3_busy.wav` (one real 15 s FT8 slot, ~15 signals),
//! circularly rolled by a known offset. The estimate should recover
//! `−offset` (mod 15 s), and `R` should be high when it did and low
//! when it did not.
//!
//! Asserts the conclusion — tiled recovers every offset to inside one
//! FT8 symbol (160 ms) — and prints the full table, wide included, so
//! the comparison that chose tiled stays visible (the same "show the
//! measurement" discipline as `ft8_coarse_sync_bootstrap`'s K=10/20
//! prints).
//!
//! ```sh
//! cargo test -p mfsk-core --features full --release \
//!     --test ft8_cold_acquisition -- --nocapture
//! ```
#![cfg(all(feature = "fft-rustfft", not(feature = "fixed-point")))]

use std::time::Instant;

use mfsk_core::engine::sync::{SyncCandidate, circular_dt_estimate};
use mfsk_core::ft8::acquire::{REQUIRED_SAMPLES, acquire_slot_phase};
use mfsk_core::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};

#[allow(dead_code)]
mod common;
use common::load_wav_i16;
use std::path::Path;

const SR: usize = 12_000;
const SLOT: usize = 180_000; // NMAX — one FT8 slot at 12 kHz
const PERIOD_S: f32 = 15.0;

const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
const MAX_CAND: usize = 200;

/// Circularly roll `audio` left by `n` samples (`n` may be negative).
fn roll(audio: &[i16], n: i64) -> Vec<i16> {
    let len = audio.len() as i64;
    (0..audio.len())
        .map(|k| audio[((k as i64 + n).rem_euclid(len)) as usize])
        .collect()
}

/// One coarse search over `audio`, returning `(dt_sec, score)` peaks.
fn coarse(audio: &[i16], lag_s: f32) -> Vec<SyncCandidate> {
    let spec = compute_spectrogram(audio, FREQ_MAX);
    coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, lag_s)
}

/// **wide**: one ±7.5 s (→ clamped ±6.28 s) search.
fn acquire_wide(audio: &[i16]) -> Vec<SyncCandidate> {
    coarse(audio, 7.5)
}

/// **tiled** — the shipped `ft8::acquire::acquire_slot_phase`. It wants
/// ≥ 25 s of contiguous audio and windows it at 0 / 5 / 10 s; feeding
/// it the 15 s slot repeated makes each later window a circular roll of
/// the slot, which is exactly the phase sweep this test wants.
fn acquire_tiled(slot: &[i16]) -> Option<(f32, f32)> {
    let mut long: Vec<i16> = Vec::with_capacity(REQUIRED_SAMPLES);
    while long.len() < REQUIRED_SAMPLES {
        long.extend_from_slice(slot);
    }
    acquire_slot_phase(&long, FREQ_MIN, FREQ_MAX, SYNC_MIN, MAX_CAND, 5)
}

/// Signed phase error, wrapped to (−½ period, ½ period].
fn phase_err(a: f32, b: f32) -> f32 {
    let mut d = a - b;
    while d > PERIOD_S / 2.0 {
        d -= PERIOD_S;
    }
    while d <= -PERIOD_S / 2.0 {
        d += PERIOD_S;
    }
    d
}

#[test]
fn ft8_cold_acquisition_tiled_vs_wide() {
    let audio = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));
    assert_eq!(
        audio.len(),
        SLOT,
        "qso3_busy.wav is expected to be exactly one slot"
    );

    // The undisplaced estimate — the phase all rolled cases are measured
    // against (the recording's own signals are not exactly at dt = 0).
    let (base_dt, base_r) = acquire_tiled(&audio).unwrap();
    println!("\nqso3_busy own phase (tiled, k=5): dt = {base_dt:+.3} s  R = {base_r:.3}\n");

    println!(
        "{:>7} | {:>28} | {:>28}",
        "roll s", "wide  dt / err / R  (ms)", "tiled  dt / err / R  (ms)"
    );
    println!("{:-<7}-+-{:-<28}-+-{:-<28}", "", "", "");

    let report = |name: &str, est: Option<(f32, f32)>, want: f32, t_ms: f64| -> f32 {
        match est {
            Some((dt, r)) => {
                let e = phase_err(dt, want);
                print!(
                    " {name}: {dt:+6.2} / {:+5.0} / {r:.2}  ({t_ms:4.0})",
                    e * 1000.0
                );
                e.abs()
            }
            None => {
                print!(" {name}: none");
                PERIOD_S
            }
        }
    };

    let mut worst_tiled = 0.0_f32;
    for &off_s in &[0.0_f32, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0] {
        let rolled = roll(&audio, (off_s * SR as f32) as i64);
        // Rolling the audio left by `off` puts every signal `off` s
        // earlier, so the grid should shift by `+off` to re-centre —
        // the estimate should read `base_dt − off` (mod period).
        let want = phase_err(base_dt - off_s, 0.0);

        print!("{off_s:>7.0} |");
        let t0 = Instant::now();
        let w = circular_dt_estimate(&acquire_wide(&rolled), 5, PERIOD_S);
        report("W", w, want, t0.elapsed().as_secs_f64() * 1e3);
        print!(" |");
        let t0 = Instant::now();
        let t = acquire_tiled(&rolled);
        let te = report("T", t, want, t0.elapsed().as_secs_f64() * 1e3);
        println!();

        worst_tiled = worst_tiled.max(te);
    }
    println!();

    // The result this test exists to establish: the tiled ±2.5 s search
    // recovers an arbitrary phase across the whole ±7.5 s period to
    // inside ~150 ms (an FT8 symbol is 160 ms). The single wide search
    // does not — its far-lag ghosts (issue #280) poison the estimate and
    // its `R` stays misleadingly high while doing so — which is why
    // slice 3 wires the tiled path, not the wide one.
    assert!(
        worst_tiled < 0.16,
        "tiled acquisition should recover every offset within one FT8 symbol; worst was {:.0} ms",
        worst_tiled * 1000.0
    );
}
