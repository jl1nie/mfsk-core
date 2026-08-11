//! Ad-hoc wall-clock timing probe: `decode_block` (embedded ship
//! config) on `qso3_busy.wav`, host CPU. Used to compare decode
//! throughput across machines (e.g. Ryzen 9 vs Apple M5) — not a
//! correctness regression, no assertions.
//!
//! **Tier C — `#[ignore]`d, run before a release.** These asserted
//! nothing yet ran in the default suite, where
//! `timing_fst4_300_sanity_strong_signal` alone cost **41 s** — 40 %
//! of the whole suite's wall clock — for output nobody read on a
//! routine run. A test that cannot fail is not a regression gate; it
//! is a measurement, and measurements belong to the release
//! checklist alongside the other sensitivity work. Feeds
//! `docs/notes/BENCHMARKS.md`.
//!
//! Run (needs `full` — `timing_fst4_300`/`timing_fst4_300_sanity_strong_signal`
//! import `mfsk_core::fst4::Fst4s300`, which the default feature set
//! doesn't enable; `--test bench_qso3_busy_timing` alone as documented
//! here previously fails to compile with `E0432`):
//! ```sh
//! cargo test --release -p mfsk-core --features full --test bench_qso3_busy_timing -- --nocapture
//! ```

use std::path::Path;
use std::time::Instant;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

const N_ITERS: usize = 10;

#[test]
#[ignore = "tier C: wall-clock measurement, no assertions — run before a release"]
fn timing_qso3_busy() {
    let slot = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    // Warm-up (page faults, allocator, etc.) — excluded from timing.
    let n = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15).len();
    println!("warm-up: {n} decodes");

    let mut times = Vec::with_capacity(N_ITERS);
    for i in 0..N_ITERS {
        let t0 = Instant::now();
        let r = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15);
        let dt = t0.elapsed();
        println!(
            "iter {i:2}: {:8.3} ms  ({} decodes)",
            dt.as_secs_f64() * 1000.0,
            r.len()
        );
        times.push(dt.as_secs_f64() * 1000.0);
    }

    let total: f64 = times.iter().sum();
    let avg = total / times.len() as f64;
    let min = times.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    println!(
        "\n=== qso3_busy.wav decode_block timing (n={}) ===\navg={avg:.3} ms  min={min:.3} ms  max={max:.3} ms",
        times.len()
    );
}

/// Deepest FT8 config: `DecodeDepth::FULL` (full LLR effort + OSD
/// fallback, host-only), `sync_min=0.8`, `max_cand=60` — the exact
/// call `ft8_qso3_full_parity_recall.rs` uses.
#[test]
#[ignore = "tier C: wall-clock measurement, no assertions — run before a release"]
fn timing_qso3_busy_full_parity() {
    let slot = load_wav_i16(Path::new(asset_path!("qso3_busy.wav")));

    let n = decode_block(&slot, 100.0, 3000.0, 0.8, DecodeDepth::FULL, 60).len();
    println!("warm-up: {n} decodes");

    let mut times = Vec::with_capacity(N_ITERS);
    for i in 0..N_ITERS {
        let t0 = Instant::now();
        let r = decode_block(&slot, 100.0, 3000.0, 0.8, DecodeDepth::FULL, 60);
        let dt = t0.elapsed();
        println!(
            "iter {i:2}: {:8.3} ms  ({} decodes)",
            dt.as_secs_f64() * 1000.0,
            r.len()
        );
        times.push(dt.as_secs_f64() * 1000.0);
    }

    let total: f64 = times.iter().sum();
    let avg = total / times.len() as f64;
    let min = times.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    println!(
        "\n=== qso3_busy.wav decode_block FULL-parity timing (n={}) ===\navg={avg:.3} ms  min={min:.3} ms  max={max:.3} ms",
        times.len()
    );
}

/// FST4-300 (5-minute slot, longest FST4 sub-mode — heaviest compute,
/// not "deeper" in the DecodeDepth sense since FST4 always runs at
/// DecodeDepth::FULL, but the biggest per-slot workload). No golden
/// recall lock exists for this real recording (only FST4-60A does,
/// see BENCHMARKS.md) — timing only, no assertions. Slices the first
/// 300 s (3,600,000 samples @ 12 kHz) out of the WSJT-X sample
/// `201230_0300.wav` (30 min recording = 6 back-to-back FST4-300
/// periods). Same params as `tests/fst4_sweep.rs::decode_wav_fst4`
/// (sync_min=0.8, max_cand=50).
#[test]
#[ignore = "tier C: wall-clock measurement, no assertions — run before a release"]
fn timing_fst4_300() {
    use mfsk_core::fst4::Fst4s300;
    use mfsk_core::msg::decode_request::DecodeRequest;

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let path = Path::new(&manifest).join("../../WSJT-X/samples/FST4+FST4W/201230_0300.wav");
    let Ok(path) = path.canonicalize() else {
        eprintln!("skipping: WSJT-X FST4 sample not found (sibling checkout)");
        return;
    };
    let full = load_wav_i16(&path);
    println!(
        "full recording = {} samples ({:.1} s)",
        full.len(),
        full.len() as f32 / 12_000.0
    );
    for (i, chunk) in full.chunks(300 * 12_000).enumerate() {
        if chunk.len() < 300 * 12_000 {
            continue;
        }
        let r = DecodeRequest::<Fst4s300>::new(chunk, 100.0, 3000.0, 0.8, 50).decode();
        println!(
            "slot[{i}] {} decodes: {:?}",
            r.results.len(),
            r.results
                .iter()
                .map(|d| {
                    let mut m77 = [0u8; 77];
                    m77.copy_from_slice(d.message77());
                    unpack77(&m77)
                })
                .collect::<Vec<_>>()
        );
    }

    let slot: Vec<i16> = full.iter().take(300 * 12_000).copied().collect();
    println!(
        "slot samples = {} ({:.1} s)",
        slot.len(),
        slot.len() as f32 / 12_000.0
    );

    let r0 = DecodeRequest::<Fst4s300>::new(&slot, 100.0, 3000.0, 0.8, 50).decode();
    println!("warm-up: {} decodes", r0.results.len());

    let n_iters = 5;
    let mut times = Vec::with_capacity(n_iters);
    for i in 0..n_iters {
        let t0 = Instant::now();
        let r = DecodeRequest::<Fst4s300>::new(&slot, 100.0, 3000.0, 0.8, 50).decode();
        let dt = t0.elapsed();
        println!(
            "iter {i:2}: {:8.3} ms  ({} decodes)",
            dt.as_secs_f64() * 1000.0,
            r.results.len()
        );
        times.push(dt.as_secs_f64() * 1000.0);
    }

    let total: f64 = times.iter().sum();
    let avg = total / times.len() as f64;
    let min = times.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    println!(
        "\n=== FST4-300 (201230_0300.wav, first slot) timing (n={}) ===\navg={avg:.3} ms  min={min:.3} ms  max={max:.3} ms",
        times.len()
    );
}

/// Sanity check: does `Fst4s300` decode *anything* on a strong
/// synthetic signal? Uses the pre-generated AWGN sweep corpus
/// (`embedded-poc/assets/fst4_sweep/fst4_300_awgn_m05_*.wav`,
/// -5 dB — should be trivially decodable if the FST4-300 path works
/// at all).
#[test]
#[ignore = "tier C: wall-clock measurement, no assertions — run before a release"]
fn timing_fst4_300_sanity_strong_signal() {
    use mfsk_core::fst4::Fst4s300;
    use mfsk_core::msg::decode_request::DecodeRequest;

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let dir = Path::new(&manifest).join("../embedded-poc/assets/fst4_sweep");
    let mut hits = 0;
    let mut total = 0;
    for i in 1..=20 {
        let path = dir.join(format!("fst4_300_awgn_m05_{i:02}.wav"));
        if !path.exists() {
            continue;
        }
        total += 1;
        let audio = load_wav_i16(&path);
        let r = DecodeRequest::<Fst4s300>::new(&audio, 100.0, 3000.0, 0.8, 50).decode();
        let msgs: Vec<_> = r
            .results
            .iter()
            .map(|d| {
                let mut m77 = [0u8; 77];
                m77.copy_from_slice(d.message77());
                unpack77(&m77)
            })
            .collect();
        let hit = msgs.iter().any(|m| m.as_deref() == Some("CQ JL1NIE PM95"));
        if hit {
            hits += 1;
        }
        println!("trial {i:02}: hit={hit} decodes={:?}", msgs);
    }
    println!("\n=== FST4-300 strong-signal (-5 dB) sanity: {hits}/{total} hit ===");
}
