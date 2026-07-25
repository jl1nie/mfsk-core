//! Does #178's flat 6x/1.0dB `subtract_tones_lpf_converge` actually stop
//! early for stable (low `sync_cv`) real signals, or does it run to
//! (near) the iteration cap regardless — the same question the FT4
//! `diag_seed4_actual_iteration_counts` probe raised for a synthetic
//! multi-signal scenario, now checked against real qso3_busy.wav data.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_iteration_count_diag -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::ft8::decode::{DecodeDepth, DecodeStrictness, decode_frame_subtract};
use mfsk_core::ft8::message::unpack77;
use mfsk_core::ft8::subtract::{refine_signal_freq, subtract_signal_lpf_converge};

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

#[test]
#[ignore]
fn iteration_counts_on_real_qso3_busy() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let results = decode_frame_subtract(
        &audio,
        100.0,
        3000.0,
        0.8,
        None,
        DecodeDepth::BpAllOsd,
        200,
        DecodeStrictness::Normal,
    );

    let mut residual = audio.clone();
    println!(
        "\nPer-candidate iteration count under flat 6x/1.0dB convergence (qso3_busy.wav, real):"
    );
    println!(
        "{:>10} {:>8} {:>7} {:>12}  msg",
        "freq", "sync_cv", "snr", "iters_applied"
    );
    for r in &results {
        let refined_freq = refine_signal_freq(&residual, r);
        let mut r_refined = r.clone();
        r_refined.freq_hz = refined_freq;
        let applied = subtract_signal_lpf_converge(&mut residual, &r_refined, 6, 1.0);
        let text = unpack77(&r.message77).unwrap_or_default();
        println!(
            "{:>10.1} {:>8.3} {:>+7.1} {:>12}  {}",
            r.freq_hz, r.sync_cv, r.snr_db, applied, text
        );
    }
}
