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
use mfsk_core::ft8::decode_block::{
    BASIS_SCRATCH_LEN, DEFAULT_Q_THRESH, SymMask, coarse_sync, compute_spectrogram, decode_block,
    decode_block_into, fill_symbol_spectra_into, process_candidates_into_tuned,
    process_candidates_into_with_cs_scratch_tuned_with_fill, refine_candidates_into,
};
use mfsk_core::ft8::llr::sync_quality;
use mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER;
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

    // Regression guard: must match the embedded S3 wav_sim baseline
    // (`baseline177_2026-05-17.log` = 7 decodes on qso3_busy.wav).
    // Drops here mean host has diverged from embedded again — most
    // likely a feature mismatch (`fixed-point` should imply
    // `nstep-half`, see mfsk-core/Cargo.toml).
    assert!(
        r_into.len() >= 7,
        "decode_block_into recall regression: got {} decodes, expected ≥ 7 (S3 wav_sim baseline)",
        r_into.len()
    );

    // ── EMBEDDED-EQUIVALENT SPLIT path: replicate
    // `dual_core::coarse_sync_split_with_allsum` on host (= what
    // m5stack-s3-app / m5stack-core2-app actually run): coarse_sync
    // **per half** with max_cand=30 each, merge, sort, top-30. This
    // gives the tail (1550-3000 Hz) weak signals guaranteed slots
    // that the single-band top-30 would push out.
    let spec = compute_spectrogram(&audio, 3000.0);
    let (fmin, fmax, smin, ncand) = (100.0_f32, 3000.0, 1.0_f32, 30);
    let mid = 0.5 * (fmin + fmax);
    let mut head = coarse_sync(&spec, fmin, mid, smin, ncand);
    let tail = coarse_sync(&spec, mid, fmax, smin, ncand);
    head.extend(tail);
    head.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    head.truncate(ncand);
    let pass1 = head;
    drop(spec);
    let mut basis_re2 = vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im2 = vec![0i16; BASIS_SCRATCH_LEN];
    let pass2 = refine_candidates_into(&audio, pass1, 15, &mut basis_re2, &mut basis_im2);
    let r_split = process_candidates_into_tuned(
        &audio,
        pass2,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        DEFAULT_BP_MAX_ITER,
        &mut basis_re2,
        &mut basis_im2,
    );
    println!(
        "\nSPLIT coarse_sync (= dual_core::coarse_sync_split_with_allsum host equiv): {} decodes",
        r_split.len()
    );
    for r in &r_split {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "<unpack-fail>".into());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1}  '{}'",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg
        );
    }

    // Phase 1.7.7b: pass1 + pass2 dump in the same format as embedded
    // m5stack-s3-app/src/decode_pipeline.rs for side-by-side diff.
    let spec_dump = compute_spectrogram(&audio, 3000.0);
    let mut head_d = coarse_sync(&spec_dump, fmin, mid, smin, ncand);
    let tail_d = coarse_sync(&spec_dump, mid, fmax, smin, ncand);
    head_d.extend(tail_d);
    head_d.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    head_d.truncate(ncand);
    println!("\n=== pass1 dump ({} candidates) ===", head_d.len());
    for (i, c) in head_d.iter().enumerate() {
        println!(
            "  p1[{:2}] DF={:6.1} DT={:+.3} score={:.3}",
            i, c.freq_hz, c.dt_sec, c.score
        );
    }
    let mut basis_re_d = vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im_d = vec![0i16; BASIS_SCRATCH_LEN];
    let pass2_d = refine_candidates_into(&audio, head_d, 15, &mut basis_re_d, &mut basis_im_d);
    println!("\n=== pass2 dump ({} candidates) ===", pass2_d.len());
    for (i, rc) in pass2_d.iter().enumerate() {
        println!(
            "  p2[{:2}] DF={:6.1} DT={:+.3} q_block0={}",
            i, rc.0.freq_hz, rc.0.dt_sec, rc.2
        );
    }
    drop(spec_dump);

    // DT precision sanity: run BASIS at cd0-found DTs for the 3
    // missing signals (N1JFU=+0.270, K1JT HA0DU=+0.300, XE2X=+0.205)
    // to test whether pass1's parabolic-peak DT is too coarse on host.
    use mfsk_core::core::sync::SyncCandidate;
    let probe_cands: Vec<SyncCandidate> = vec![
        SyncCandidate {
            freq_hz: 640.6,
            dt_sec: 0.270,
            score: 9.21,
        },
        SyncCandidate {
            freq_hz: 589.6,
            dt_sec: 0.300,
            score: 13.7,
        },
        SyncCandidate {
            freq_hz: 2853.6,
            dt_sec: 0.205,
            score: 9.0,
        },
    ];
    let mut basis_re_p = vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im_p = vec![0i16; BASIS_SCRATCH_LEN];
    let probe_pass2 =
        refine_candidates_into(&audio, probe_cands, 3, &mut basis_re_p, &mut basis_im_p);
    let mut cs_scratch_p: Box<[[mfsk_core::core::scalar::Cmplx<f32>; 8]; 79]> =
        vec![[mfsk_core::core::scalar::Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    let r_probe = process_candidates_into_with_cs_scratch_tuned_with_fill(
        &audio,
        probe_pass2,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        DEFAULT_BP_MAX_ITER,
        &mut cs_scratch_p,
        |cs, cand, mask| {
            fill_symbol_spectra_into(
                cs,
                &audio,
                cand.freq_hz,
                cand.dt_sec,
                mask,
                &mut basis_re_p,
                &mut basis_im_p,
            );
        },
    );
    println!("\n=== DT-probe: BASIS at cd0-found DTs for 3 missing signals ===");
    println!("  {} decodes", r_probe.len());
    for r in &r_probe {
        let msg = unpack77(&r.message77).unwrap_or_else(|| "<unpack-fail>".into());
        println!(
            "  freq={:6.1} dt={:+.3} hard_err={:3} snr={:+.1}  '{}'",
            r.freq_hz, r.dt_sec, r.hard_errors, r.snr_db, msg
        );
    }

    // For each pass2 candidate, simulate what process_candidates_with_ap
    // does internally: take its block-0 cs_box, fill blocks 1+2 via
    // BASIS, then compute the full 21-symbol sync_quality. Anything
    // ≤ DEFAULT_Q_THRESH (=6) gets skipped before BP ever runs — this
    // is the q_thresh gate at process_candidates.rs:1341.
    println!(
        "\n=== pass2 q_full (full 21-sym sync_quality, gates BP at q > {}) ===",
        DEFAULT_Q_THRESH
    );
    let mut basis_re_q = vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im_q = vec![0i16; BASIS_SCRATCH_LEN];
    for (i, (cand, cs_box, q_block0)) in pass2_d.iter().enumerate() {
        let mut cs_scratch = **cs_box;
        fill_symbol_spectra_into(
            &mut cs_scratch,
            &audio,
            cand.freq_hz,
            cand.dt_sec,
            SymMask::SyncBlocks12,
            &mut basis_re_q,
            &mut basis_im_q,
        );
        let q_full = sync_quality(&cs_scratch);
        let gate = if q_full > DEFAULT_Q_THRESH {
            "PASS"
        } else {
            "SKIP"
        };
        println!(
            "  p2[{:2}] DF={:6.1} DT={:+.3} q_block0={} q_full={} {}",
            i, cand.freq_hz, cand.dt_sec, q_block0, q_full, gate
        );
    }

    // ── EMBEDDED-EQUIVALENT BASIS fill path: force the
    // `fill_symbol_spectra_into` (BASIS sin/cos dot-product) algorithm
    // even on host where `process_candidates_into_with_cs_scratch_tuned`
    // would otherwise pick `fill_symbol_spectra` (cd0 32-pt FFT) because
    // of the `#[cfg(feature = "fft-rustfft")]` gate. Routes through
    // `_with_fill` and passes the BASIS closure explicitly.
    let spec3 = compute_spectrogram(&audio, 3000.0);
    let mut head3 = coarse_sync(&spec3, fmin, mid, smin, ncand);
    let tail3 = coarse_sync(&spec3, mid, fmax, smin, ncand);
    head3.extend(tail3);
    head3.sort_by(|a, b| {
        b.score
            .partial_cmp(&a.score)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    head3.truncate(ncand);
    drop(spec3);
    let mut basis_re3 = vec![0i16; BASIS_SCRATCH_LEN];
    let mut basis_im3 = vec![0i16; BASIS_SCRATCH_LEN];
    let pass2_basis = refine_candidates_into(&audio, head3, 15, &mut basis_re3, &mut basis_im3);
    let mut cs_scratch_basis: Box<[[mfsk_core::core::scalar::Cmplx<f32>; 8]; 79]> =
        vec![[mfsk_core::core::scalar::Cmplx::<f32>::default(); 8]; 79]
            .try_into()
            .unwrap();
    let r_basis = process_candidates_into_with_cs_scratch_tuned_with_fill(
        &audio,
        pass2_basis,
        DecodeDepth::BpAll,
        DEFAULT_Q_THRESH,
        DEFAULT_BP_MAX_ITER,
        &mut cs_scratch_basis,
        |cs, cand, mask| {
            fill_symbol_spectra_into(
                cs,
                &audio,
                cand.freq_hz,
                cand.dt_sec,
                mask,
                &mut basis_re3,
                &mut basis_im3,
            );
        },
    );
    println!(
        "\nBASIS fill (= embedded fill_symbol_spectra_into, sin/cos dot product): {} decodes",
        r_basis.len()
    );
    for r in &r_basis {
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
