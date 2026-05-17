//! Phase 1.7.7-Stick diagnostic: `decode_block_into` (= the canonical
//! embedded entry point: compute_spectrogram → coarse_sync →
//! refine_candidates_into → process_candidates_into_tuned, with
//! caller-provided BASIS scratch) called on host. Should match the
//! S3 wav_sim embedded baseline (7 decodes per `baseline177_*.log`).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features fft-rustfft,ft8,fixed-point \
//!     --test ft8_decode_block_fixed_point_baseline -- --nocapture
//! ```
#![cfg(all(feature = "fixed-point", feature = "fft-rustfft"))]

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{BASIS_SCRATCH_LEN, decode_block, decode_block_into};
use mfsk_core::msg::wsjt77::unpack77;

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

fn load_wav_i16(path: &str) -> Vec<i16> {
    let bytes = std::fs::read(path).unwrap();
    let mut i = 12usize;
    let mut data_off = 0usize;
    let mut data_len = 0usize;
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
        i += 8;
        if id == b"data" {
            data_off = i;
            data_len = len.min(bytes.len() - i);
        }
        i += len + if len % 2 == 1 { 1 } else { 0 };
    }
    bytes[data_off..data_off + data_len]
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

#[test]
fn decode_block_fixed_point_baseline() {
    let audio = load_wav_i16(QSO3_PATH);
    println!(
        "audio.len() = {} ({:.2} s)",
        audio.len(),
        audio.len() as f32 / 12_000.0
    );
    println!("Q11 LLR via fixed-point feature, BASIS dot product via decode_block_into");

    // EMBEDDED CANONICAL PATH: decode_block_into uses
    //   compute_spectrogram → coarse_sync → refine_candidates_into →
    //   process_candidates_into_tuned
    // with caller-provided BASIS scratch (= fill_symbol_spectra_into
    // path, same as m5stack-s3-app + Core2 production).
    let mut basis_re = vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im = vec![0i16; BASIS_SCRATCH_LEN];
    let r_into = decode_block_into(
        &audio,
        100.0,
        3000.0,
        1.0,
        DecodeDepth::BpAll,
        15,
        &mut basis_re,
        &mut basis_im,
    );
    println!(
        "\ndecode_block_into (embedded canonical): {} decodes",
        r_into.len()
    );
    for r in &r_into {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "<unpack-fail>".into());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1}  '{}'",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg
        );
    }

    // For contrast: decode_block (host multipass via_cd0 cd0+32pt path)
    let r = decode_block(&audio, 100.0, 3000.0, 1.0, DecodeDepth::BpAll, 15);
    println!(
        "\ndecode_block (host multipass via_cd0): {} decodes",
        r.len()
    );
    for r in &r {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "<unpack-fail>".into());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1}  '{}'",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg
        );
    }
}
