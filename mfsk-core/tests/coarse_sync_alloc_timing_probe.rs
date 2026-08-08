//! Ad-hoc wall-clock timing probe: `engine::sync::coarse_sync` in
//! isolation, across the three protocols that share it via
//! `SyncMode::Block` (FT8/FT4/FST4-60A).
//!
//! Added alongside the fix that replaced `fill_sync2d_row!`'s
//! per-(freq-bin, lag)-cell `vec![0.0f32; num_blocks]` heap allocations
//! (`engine/sync.rs`) with reused fixed-size stack arrays — no existing
//! diag test isolated `coarse_sync` itself from the rest of a decode
//! call (LLR/BP/OSD), so this measures just the candidate search, not
//! end-to-end decode time.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full,internal-testing \
//!     --test coarse_sync_alloc_timing_probe -- --ignored --nocapture --test-threads=1
//! ```

use std::path::Path;
use std::time::Instant;

use mfsk_core::engine::sync::coarse_sync;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::ft4::Ft4;
use mfsk_core::ft8::Ft8;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const N_ITERS: usize = 10;

fn time_it<F: Fn() -> usize>(label: &str, f: F) {
    let n = f(); // warm-up
    println!("{label} warm-up: {n} candidates");
    let mut times = Vec::with_capacity(N_ITERS);
    for _ in 0..N_ITERS {
        let t0 = Instant::now();
        let n = f();
        let dt = t0.elapsed().as_secs_f64() * 1000.0;
        times.push(dt);
        std::hint::black_box(n);
    }
    let avg = times.iter().sum::<f64>() / times.len() as f64;
    let min = times.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    println!("{label}: avg={avg:.3}ms min={min:.3}ms max={max:.3}ms (n={N_ITERS})");
}

#[test]
#[ignore = "manual timing probe — coarse_sync alloc-in-loop fix (engine/sync.rs)"]
fn coarse_sync_timing() {
    let ft8 = Path::new(asset_path!("qso3_busy.wav"));
    let ft8_audio = load_i16(ft8);
    time_it("FT8 qso3_busy.wav coarse_sync", || {
        coarse_sync::<Ft8>(&ft8_audio, 100.0, 3000.0, 1.3, None, 15).len()
    });

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    if let Ok(p) = Path::new(&manifest)
        .join("../../WSJT-X/samples/FT4/000000_000002.wav")
        .canonicalize()
    {
        let raw = read_wsjtx_wav_i16(&p).unwrap();
        let mut audio = vec![0i16; 90_000];
        let n = raw.len().min(90_000);
        audio[..n].copy_from_slice(&raw[..n]);
        time_it("FT4 000000_000002.wav coarse_sync", || {
            coarse_sync::<Ft4>(&audio, 100.0, 2700.0, 0.05, None, 100).len()
        });
    } else {
        eprintln!("skipping FT4: sample not found");
    }

    if let Ok(p) = Path::new(&manifest)
        .join("../../WSJT-X/samples/FST4+FST4W/210115_0058.wav")
        .canonicalize()
    {
        let audio = read_wsjtx_wav_i16(&p).unwrap();
        time_it("FST4-60A 210115_0058.wav coarse_sync", || {
            coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.0, None, 50).len()
        });
    } else {
        eprintln!("skipping FST4-60A: sample not found");
    }
}

fn load_i16(path: &Path) -> Vec<i16> {
    read_wsjtx_wav_i16(path).expect("WAV must be 12 kHz mono PCM-16")
}
