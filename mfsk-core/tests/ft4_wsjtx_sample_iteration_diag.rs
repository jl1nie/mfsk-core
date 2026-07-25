//! Does the FT4 SIC fix's convergence check (#179) show natural
//! per-candidate iteration diversity (2-6, matching real FT8 data,
//! `ft8_qso3_iteration_count_diag.rs`) on a REAL FT4 recording, or does
//! it max out uniformly the way it did on the synthetic
//! `ft4_busy_band_fading_probe.rs` scenario?
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft4_wsjtx_sample_iteration_diag -- --ignored --nocapture
//! ```
#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

use mfsk_core::core::dsp::subtract::subtract_tones_lpf_converge;
use mfsk_core::ft4::decode::{FT4_SUBTRACT, decode_frame_subtract};
use mfsk_core::ft4::encode::message_to_tones;
use mfsk_core::ft4::subtract::refine_signal_freq;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000;

fn sample_path() -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let p = Path::new(&manifest)
        .join("../../WSJT-X/samples/FT4/000000_000002.wav")
        .canonicalize()
        .ok()?;
    if p.is_file() { Some(p) } else { None }
}

#[test]
#[ignore]
fn iteration_counts_on_real_ft4_sample() {
    let Some(path) = sample_path() else {
        eprintln!("skipping: WSJT-X FT4 sample not found");
        return;
    };
    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);

    let results = decode_frame_subtract(&audio, 100.0, 2700.0, 0.05, 100);

    let mut residual = audio.clone();
    println!("\nPer-candidate iteration count under flat 6x/1.0dB convergence (real FT4 sample):");
    println!(
        "{:>10} {:>8} {:>7} {:>12}  msg",
        "freq", "sync_cv", "snr", "iters_applied"
    );
    for r in &results {
        let refined_freq = refine_signal_freq(&residual, r);
        let msg: [u8; 77] = r.message77().try_into().unwrap();
        let tones = message_to_tones(&msg);
        let applied = subtract_tones_lpf_converge(
            &mut residual,
            &tones,
            refined_freq,
            r.dt_sec,
            &FT4_SUBTRACT,
            700,
            false,
            6,
            1.0,
        );
        let text = unpack77(&msg).unwrap_or_default();
        println!(
            "{:>10.1} {:>8.3} {:>+7.1} {:>12}  {}",
            r.freq_hz, r.sync_cv, r.snr_db, applied, text
        );
    }
}
