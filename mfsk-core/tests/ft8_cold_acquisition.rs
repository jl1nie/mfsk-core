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
//! - **tiled** — three `±2.5 s` searches on the audio rolled to centre
//!   windows at −5 / 0 / +5 s, each candidate's own window offset added
//!   back, union covers the full 15 s. Three spectrograms, three
//!   searches, the tested kernel unchanged (jl1nie's suggestion on the
//!   issue).
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

/// **tiled**: ±2.5 s searches centred at −5 / 0 / +5 s, window offset
/// folded back into each candidate's `dt_sec`.
fn acquire_tiled(audio: &[i16]) -> Vec<SyncCandidate> {
    let mut all = Vec::new();
    for w in [-5.0_f32, 0.0, 5.0] {
        let rolled = roll(audio, (w * SR as f32) as i64);
        for c in coarse(&rolled, 2.5) {
            all.push(SyncCandidate {
                freq_hz: c.freq_hz,
                dt_sec: c.dt_sec + w,
                score: c.score,
            });
        }
    }
    all
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
    let (base_dt, base_r) = circular_dt_estimate(&acquire_tiled(&audio), 5, PERIOD_S).unwrap();
    println!("\nqso3_busy own phase (tiled, k=5): dt = {base_dt:+.3} s  R = {base_r:.3}\n");

    println!(
        "{:>7} | {:>28} | {:>28}",
        "roll s", "wide  dt / err / R  (ms)", "tiled  dt / err / R  (ms)"
    );
    println!("{:-<7}-+-{:-<28}-+-{:-<28}", "", "", "");

    let row = |name: &str, cands: &[SyncCandidate], want: f32, t_ms: f64| -> f32 {
        match circular_dt_estimate(cands, 5, PERIOD_S) {
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
        let w = acquire_wide(&rolled);
        row("W", &w, want, t0.elapsed().as_secs_f64() * 1e3);
        print!(" |");
        let t0 = Instant::now();
        let t = acquire_tiled(&rolled);
        let te = row("T", &t, want, t0.elapsed().as_secs_f64() * 1e3);
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
