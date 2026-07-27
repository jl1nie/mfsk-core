//! One-off probe: `CQ DX DL8YHR JO41` @2606 Hz, found by WSJT-X's `jt9
//! -d3` (Deep) on `qso3_busy.wav` but missed by
//! `mfsk_core::ft8::decode::decode_frame_subtract` (AP-off, 200-3000 Hz,
//! `DecodeDepth::FULL`). Unlike the DK8NE/WA2FZW probes, this is a
//! `CQ` message — there is no plausible AP call1/call2 context to try,
//! since it isn't addressed to anyone. So the only questions are:
//! (a) does coarse_sync even surface a candidate near 2606 Hz, and
//! (b) if so, does BP/OSD converge on the right text at any depth /
//! strictness / dt-tolerance we haven't tried yet.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_dl8yhr_probe -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeStrictness};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

#[test]
#[ignore]
fn probe_dl8yhr_coarse_sync_candidates() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let spec = mfsk_core::ft8::decode_block::compute_spectrogram(&audio, 3000.0);
    let candidates = mfsk_core::ft8::decode_block::coarse_sync(&spec, 100.0, 3000.0, 0.8, 200);
    let near: Vec<_> = candidates
        .iter()
        .filter(|c| (c.freq_hz - 2606.0).abs() <= 10.0)
        .collect();
    println!("\n=== coarse_sync near 2606 Hz (CQ DX DL8YHR JO41 claimed site) ===");
    println!(
        "{} candidates total, {} within +/-10 Hz of 2606 Hz",
        candidates.len(),
        near.len()
    );
    for c in &near {
        println!(
            "  cand freq={:.2} dt={:.3} score={:.4}",
            c.freq_hz, c.dt_sec, c.score
        );
    }
}

#[test]
#[ignore]
fn probe_dl8yhr_decode_sweep() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let target = "CQ DX DL8YHR JO41";

    println!("\n=== CQ DX DL8YHR JO41 @2606 Hz — decode parameter sweep ===");

    for (label, dt_tol, max_cand) in [
        ("baseline (bench default)", 1.5, 200),
        ("wider dt_tol", 3.0, 200),
        ("more candidates", 1.5, 400),
        ("wider dt_tol + more candidates", 3.0, 400),
    ] {
        let single = DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, dt_tol, max_cand)
            .depth(DecodeDepth::FULL)
            .decode()
            .results;
        let hit_single = single
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .any(|m| m == target);

        let sub = DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, dt_tol, max_cand)
            .depth(DecodeDepth::FULL)
            .strictness(DecodeStrictness::Normal)
            .staged()
            .decode()
            .results;
        let hit_sub = sub
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .any(|m| m == target);

        let sub_deep = DecodeRequest::<Ft8>::new(&audio, 200.0, 3000.0, dt_tol, max_cand)
            .depth(DecodeDepth::FULL)
            .strictness(DecodeStrictness::Deep)
            .staged()
            .decode()
            .results;
        let hit_sub_deep = sub_deep
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .any(|m| m == target);

        println!(
            "  [{label}] dt_tol={dt_tol} max_cand={max_cand}  single={}  subtract/Normal={}  subtract/Deep={}",
            if hit_single { "HIT" } else { "miss" },
            if hit_sub { "HIT" } else { "miss" },
            if hit_sub_deep { "HIT" } else { "miss" },
        );
    }
}

#[test]
#[ignore]
fn probe_dl8yhr_lower_sync_min() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let spec = mfsk_core::ft8::decode_block::compute_spectrogram(&audio, 3000.0);
    for sync_min in [0.8, 0.5, 0.3, 0.1] {
        let candidates =
            mfsk_core::ft8::decode_block::coarse_sync(&spec, 100.0, 3000.0, sync_min, 2000);
        let mut near: Vec<_> = candidates
            .iter()
            .filter(|c| (c.freq_hz - 2606.0).abs() <= 8.0)
            .collect();
        near.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        println!(
            "sync_min={sync_min}  total={}  near2606(+/-8Hz)={}",
            candidates.len(),
            near.len()
        );
        for c in near.iter().take(6) {
            println!(
                "    freq={:.2} dt={:+.3} score={:.2}",
                c.freq_hz, c.dt_sec, c.score
            );
        }
    }
}
