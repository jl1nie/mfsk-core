// SPDX-License-Identifier: GPL-3.0-only
//! Does the wide grid probe find a grid that is off by more than the
//! per-slot search can reach?
//!
//! The probe (`dual_core::DecodeConfig::wide_probe_lag_s`) exists for
//! the gap between ±0.88 s, which the per-slot search absorbs, and a
//! 25 s cold acquisition — see `docs/notes/CORES3_FT8_SLOT_BUDGET.md`
//! §8. On a radio with a *healthy* grid it reports the decode median
//! to within 0.29 s, which says it is not lying; it says nothing about
//! whether it can find a grid that is 1.5 s out, and that is the only
//! thing it is for.
//!
//! Reproduced here rather than on the board because the offset can be
//! swept: the board gives one point per flash, this gives the whole
//! range on three recordings in seconds.
//!
//! **Runs on the embedded time grid only** (`nstep-half`, implied by
//! `fixed-point`) — 184 rows, which is what the emit-point arithmetic
//! is written against.
//!
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_wide_grid_probe -- --nocapture
//! ```
#![cfg(all(feature = "ft8", not(feature = "fft-rustfft")))]

use mfsk_core::engine::sync::circular_dt_clusters;
use mfsk_core::ft8::decode_block::{
    coarse_sync_with_allsum_and_lag, compute_spectrogram, precompute_coarse_allsum,
};

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

#[path = "common/embedded_driver_harness.rs"]
mod harness;

use harness::load_wav_i16;

const SLOT: usize = 180_000;
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
/// `PASS1_LIMIT` on the board.
const PASS1: usize = 30;
/// `stage1_inc` emits at pair 87, so `2 * 87` rows hold a spectrum and
/// the rest are present and zero.
const FILLED: usize = 174;
/// `MFSK_FT8_WIDE_PROBE`'s window. `bounded_sync_lag_steps` clamps it
/// to what the row grid allows, ±2.48 s.
const WIDE_LAG_S: f32 = 2.5;
/// The probe's own cluster kernel — wide enough that a band's whole
/// station population forms one cluster (1.07 s end to end on
/// `qso3_busy`) instead of two that each lose to an artefact.
const KERNEL_S: f32 = 1.0;

fn roll(audio: &[i16], n: i64) -> Vec<i16> {
    let len = audio.len() as i64;
    (0..audio.len())
        .map(|k| audio[((k as i64 + n).rem_euclid(len)) as usize])
        .collect()
}

/// **Exactly what `dual_core` runs**, including the half-band: the
/// probe reuses `allsum_head`, which covers `freq_min..mid`, so it
/// sees the lower half of the spectrum only. Worth stating, because it
/// halves the station population the probe has to work with and
/// nothing in the board's log says so.
fn probe(slot_audio: &[i16]) -> Option<(f32, f32, f32)> {
    let mut spec = compute_spectrogram(slot_audio, FREQ_MAX);
    assert_eq!(spec.n_time, 184, "not the embedded time grid");
    // The bundle is emitted before the slot ends: rows past the emit
    // point are present and zero.
    let n_freq = spec.n_freq;
    for m in FILLED..spec.n_time {
        for f in 0..n_freq {
            spec.data[m * n_freq + f] = Default::default();
        }
    }
    // `MFSK_PROBE_FULL_BAND=1` asks the other question: the shipped
    // probe reuses `allsum_head` and therefore sees half the stations,
    // which halves the score mass a real cluster can muster. Running
    // the whole band costs a second allsum and roughly doubles the
    // probe's time; whether it buys accuracy is the point of the flag.
    let top = if std::env::var("MFSK_PROBE_FULL_BAND").is_ok() {
        FREQ_MAX
    } else {
        0.5 * (FREQ_MIN + FREQ_MAX)
    };
    let allsum = precompute_coarse_allsum(&spec, FREQ_MIN, top);
    let cands =
        coarse_sync_with_allsum_and_lag(&spec, FREQ_MIN, top, SYNC_MIN, PASS1, &allsum, WIDE_LAG_S);
    match circular_dt_clusters(&cands, KERNEL_S, 15.0, 2).as_slice() {
        [] => None,
        [(dt, mass)] => Some((*dt, *mass, f32::INFINITY)),
        [(dt, mass), (_, second), ..] => Some((*dt, *mass, *mass / second.max(1e-6))),
    }
}

fn wrap(mut d: f32) -> f32 {
    while d > 7.5 {
        d -= 15.0;
    }
    while d <= -7.5 {
        d += 15.0;
    }
    d
}

/// Sweep the grid offset and ask the probe where it thinks the grid
/// is. `MFSK_PROBE_WAV` picks the recording.
#[test]
#[ignore = "diagnostic — grid-offset sweep, prints a table"]
fn the_probe_finds_a_displaced_grid() {
    let name = std::env::var("MFSK_PROBE_WAV").unwrap_or_else(|_| "qso3_busy.wav".into());
    let path = format!(
        "{}/../embedded-poc/assets/{name}",
        env!("CARGO_MANIFEST_DIR")
    );
    let mut audio = load_wav_i16(std::path::Path::new(&path));
    audio.truncate(SLOT);

    // The recording's own phase, from the probe at zero offset. Every
    // error below is measured against `base + offset`, because what is
    // being tested is whether the probe *tracks* a displacement, not
    // whether the recording happens to sit at dt = 0.
    let base = probe(&audio).map(|(dt, _, _)| dt).unwrap_or(0.0);
    println!("\n  {name}: probe reads {base:+.2} s at zero offset\n");
    println!("  offset   expect    probe     err    mass    dom   verdict");
    println!("  {:-<58}", "");

    let mut in_band = Vec::new();
    let mut out_band = Vec::new();
    let mut step = -12i32;
    while step <= 12 {
        let off_s = step as f32 * 0.2;
        let rolled = roll(&audio, -((off_s * 12_000.0) as i64));
        let expect = wrap(base + off_s);
        match probe(&rolled) {
            Some((dt, mass, dom)) => {
                let err = wrap(dt - expect);
                // The probe's job is the band the per-slot search
                // cannot reach. Inside ±0.88 s the per-slot search
                // already handles it and the probe is redundant.
                let far = off_s.abs() > 0.88;
                if far {
                    out_band.push((err.abs(), dom));
                } else {
                    in_band.push((err.abs(), dom));
                }
                println!(
                    "  {off_s:+6.1} {expect:+8.2} {dt:+8.2} {err:+7.2} {mass:7.0} {dom:6.1}   {}",
                    if err.abs() <= 0.5 { "ok" } else { "MISS" }
                );
            }
            None => println!("  {off_s:+6.1} {expect:+8.2}    (nothing found)"),
        }
        step += 1;
    }

    let summary = |tag: &str, v: &[(f32, f32)]| {
        if v.is_empty() {
            return;
        }
        let hit = v.iter().filter(|(e, _)| *e <= 0.5).count();
        let worst = v.iter().map(|(e, _)| *e).fold(0.0f32, f32::max);
        let dom_hit: Vec<f32> = v
            .iter()
            .filter(|(e, _)| *e <= 0.5)
            .map(|(_, d)| *d)
            .collect();
        let dom_miss: Vec<f32> = v
            .iter()
            .filter(|(e, _)| *e > 0.5)
            .map(|(_, d)| *d)
            .collect();
        let mean = |x: &[f32]| {
            if x.is_empty() {
                f32::NAN
            } else {
                x.iter().sum::<f32>() / x.len() as f32
            }
        };
        println!(
            "  {tag}: {hit}/{} within 0.5 s, worst {worst:.2} s — dom mean {:.1} on hits, {:.1} on misses",
            v.len(),
            mean(&dom_hit),
            mean(&dom_miss)
        );
    };
    println!();
    summary("inside ±0.88 s (per-slot search's own range)", &in_band);
    summary("beyond ±0.88 s (what the probe is FOR) ", &out_band);
}
