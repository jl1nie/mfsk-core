//! Diagnostic probe for issue #40: list and compare candidates from both
//! host and embedded coarse-sync pipelines on the qso3_busy.wav reference.
//!
//! `#[ignore]`d so it doesn't run in normal CI; invoke explicitly:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_coarse_sync_probe -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::Ft8;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

/// Frequencies (Hz) where the WSJT-X canonical / JTDX-extras decodes sit.
/// Sourced from `ft8_qso3_apoff_recall::WSJTX_GOLDEN` + #40 issue body.
const PROBE_FREQS: &[(f32, &str)] = &[
    (244.0, "K1BZM DK8NE -10 (apoff)"),
    (400.0, "W0RSJ EA3BMU RR73"),
    (466.0, "N1PJT HB9CQK -10"),
    (472.0, "KD2UGC F6GCP R-23"),
    (590.0, "K1JT HA0DU KN07"),
    (641.0, "N1JFU EA6EE R-7"),
    (723.0, "A92EE F5PSR -14"),
    (1197.0, "CQ F5RXL IN94"),
    (2039.0, "JTDX extras band"),
];

fn load_wav_i16(path: &Path) -> Vec<i16> {
    let bytes = std::fs::read(path).expect("WAV present");
    assert_eq!(&bytes[0..4], b"RIFF", "not a RIFF/WAV file");
    let mut i = 12usize;
    let (mut data_off, mut data_len) = (0usize, 0usize);
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
        i += 8;
        if id == b"data" {
            data_off = i;
            data_len = len;
        }
        i += len;
        if len % 2 == 1 {
            i += 1;
        }
    }
    bytes[data_off..data_off + data_len.min(bytes.len() - data_off)]
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

fn print_near(label: &str, cands: &[SyncCandidate], target_hz: f32, win_hz: f32) {
    let mut hits: Vec<&SyncCandidate> = cands
        .iter()
        .filter(|c| (c.freq_hz - target_hz).abs() <= win_hz)
        .collect();
    hits.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
    if hits.is_empty() {
        println!("    {label:<14} ∅  (no cand within ±{win_hz:.0} Hz)");
    } else {
        for (i, c) in hits.iter().take(5).enumerate() {
            println!(
                "    {label:<14} #{i}: {:>7.2} Hz  dt={:>+5.3}s  score={:.3}",
                c.freq_hz, c.dt_sec, c.score
            );
        }
    }
}

#[test]
#[ignore]
fn probe_coarse_sync_candidates_at_golden_freqs() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));

    // Host wide-band path (used by decode_frame_with_ap).
    let host_cands: Vec<SyncCandidate> =
        mfsk_core::core::sync::coarse_sync::<Ft8>(&audio, 100.0, 3000.0, 1.3, None, 200);

    // Embedded path (used by decode_block).
    let spec = mfsk_core::ft8::decode_block::compute_spectrogram(&audio, 3000.0);
    let block_cands: Vec<SyncCandidate> =
        mfsk_core::ft8::decode_block::coarse_sync(&spec, 100.0, 3000.0, 1.3, 200);

    println!(
        "\nhost coarse_sync (Ft8): {} cands; decode_block coarse_sync: {} cands\n",
        host_cands.len(),
        block_cands.len()
    );

    let win = 8.0;
    for (f, label) in PROBE_FREQS {
        println!("  target {f:>7.1} Hz — {label}");
        print_near("host", &host_cands, *f, win);
        print_near("decode_block", &block_cands, *f, win);
        println!();
    }

    // Trace the F5RXL CQ candidate through process_candidate's stages
    // to confirm the negative-dt → usize-saturation hypothesis.
    println!("\n  === trace 1196.88 Hz CQ F5RXL through fine refine ===");
    let target = host_cands
        .iter()
        .find(|c| (c.freq_hz - 1196.88).abs() < 1.0);
    if let Some(c) = target {
        let (cd0, _) = mfsk_core::ft8::downsample::downsample(&audio, c.freq_hz, None);
        let r = mfsk_core::ft8::refine_fine::fine_refine_3stage(&cd0, c.dt_sec);
        println!(
            "    cand:    freq={:.2}, dt={:+.3}s, score={:.3}",
            c.freq_hz, c.dt_sec, c.score
        );
        println!(
            "    refine:  delf={:+.3} Hz, refined_dt={:+.3}s, score={:.3}",
            r.delf_hz, r.dt_sec, r.score
        );
        let i_signed = ((r.dt_sec + 0.5) * 200.0).round() as i32;
        let i_usize = ((r.dt_sec + 0.5) * 200.0).round() as usize;
        println!("    i_start: signed={i_signed}, as_usize_cast={i_usize} (saturates if neg)");
    } else {
        println!("    (no host candidate near 1196.88 Hz)");
    }

    // Also dump the top 30 candidates from each list, with provenance tags.
    println!("\n  === host top-30 ===");
    for (i, c) in host_cands.iter().take(30).enumerate() {
        println!(
            "    {i:>2}  {:>7.2} Hz  dt={:>+5.3}s  score={:.3}",
            c.freq_hz, c.dt_sec, c.score
        );
    }
    println!("\n  === decode_block top-30 ===");
    for (i, c) in block_cands.iter().take(30).enumerate() {
        println!(
            "    {i:>2}  {:>7.2} Hz  dt={:>+5.3}s  score={:.3}",
            c.freq_hz, c.dt_sec, c.score
        );
    }
}
