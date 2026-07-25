//! issue #177 follow-up: does subtracting W1FC F5BZB (~2571 Hz, the
//! signal whose spectral leakage the drift_probe found dominating the
//! 2568-2643 Hz scan by ~450x) actually clean up the DL8YHR candidate
//! region at 2606 Hz? If SIC is working as intended, sync_quality and
//! coarse-sync score near 2606 Hz should improve substantially after
//! W1FC is subtracted out. If they don't move much, the leakage isn't
//! primarily coming from W1FC's *fundamental* tones (which subtract
//! removes) — more likely a different mechanism (sidelobe/windowing,
//! or a different/stronger interferer).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_dl8yhr_subtract_probe -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::FrameLayout;
use mfsk_core::core::sync::{SyncDims, make_costas_ref, score_costas_block};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{DecodeDepth, decode_sniper};
use mfsk_core::ft8::downsample::downsample;
use mfsk_core::ft8::llr::sync_quality;
use mfsk_core::ft8::message::unpack77;
use mfsk_core::ft8::subtract::{refine_signal_freq, subtract_signal_lpf};

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

/// Score the strongest candidate near `target_freq` using the same
/// first/last-sync-block probe as `ft8_qso3_dl8yhr_drift_probe.rs`,
/// but now just reports total score at a fixed dt across a small
/// freq window, before/after mutation of `audio`.
fn scan_near(audio: &[i16], center_freq: f32, dt_sec: f32, label: &str) {
    let d = SyncDims::of::<Ft8>();
    let blocks = <Ft8 as FrameLayout>::SYNC_MODE.blocks();
    let first = &blocks[0];
    let csync_first = make_costas_ref(first.pattern, d.ds_spb);
    let nominal_i0 = ((dt_sec + Ft8::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
    let block_off = (first.start_symbol as usize * d.ds_spb) as i32;

    println!("  [{label}] freq scan around {center_freq:.2} Hz, dt={dt_sec}:");
    let mut freq = center_freq - 6.25 * 3.0;
    while freq <= center_freq + 6.25 * 3.0 {
        let (cd0, _cache) = downsample(audio, freq, None);
        let off = nominal_i0 + block_off;
        let score = score_costas_block(&cd0, &csync_first, d.ds_spb, off);
        println!("      freq={freq:>8.2}  score={score:>16.4}");
        freq += 6.25;
    }
}

#[test]
#[ignore]
fn probe_subtract_w1fc_effect_on_dl8yhr_region() {
    let mut audio = load_wav_i16(Path::new(QSO3_PATH));

    // 1. Decode W1FC F5BZB near 2571 Hz.
    let w1fc_results = decode_sniper(&audio, 2571.0, DecodeDepth::BpAllOsd, 20);
    let w1fc = w1fc_results
        .iter()
        .find(|r| {
            unpack77(&r.message77).as_deref() == Some("W1FC F5BZB -08")
                || unpack77(&r.message77).as_deref() == Some("W1FC F5BZB -8")
        })
        .expect("W1FC F5BZB should decode near 2571 Hz");
    println!(
        "W1FC decoded: freq={:.2} dt={:+.3} snr={:+.1} msg={:?}",
        w1fc.freq_hz,
        w1fc.dt_sec,
        w1fc.snr_db,
        unpack77(&w1fc.message77)
    );

    println!("\n=== BEFORE subtracting W1FC ===");
    scan_near(&audio, 2606.0, 0.2, "before");
    let q_before = {
        let (cd0, _c) = downsample(&audio, 2606.0, None);
        let refined = mfsk_core::core::sync::refine_candidate::<Ft8>(
            &cd0,
            &mfsk_core::core::sync::SyncCandidate {
                freq_hz: 2606.0,
                dt_sec: 0.2,
                score: 0.0,
            },
            10,
        );
        let cs = mfsk_core::ft8::decode_block::symbol_spectra_direct::<i16>(
            &audio,
            2606.0,
            refined.dt_sec,
            mfsk_core::ft8::decode_block::SymMask::SyncOnly,
            None,
        );
        sync_quality(&cs)
    };
    println!("  sync_quality at 2606 Hz (dt refined near 0.2): q={q_before}");

    // 2. Refine W1FC's frequency (per subtract.rs doc: coarse_sync bins
    //    are ~2.93 Hz wide, real signals sit off-bin) and subtract it.
    let refined_freq = refine_signal_freq(&audio, w1fc);
    println!(
        "\nW1FC refined freq: {:.2} Hz (was {:.2})",
        refined_freq, w1fc.freq_hz
    );
    let mut w1fc_refined = w1fc.clone();
    w1fc_refined.freq_hz = refined_freq;
    subtract_signal_lpf(&mut audio, &w1fc_refined);

    println!("\n=== AFTER subtracting W1FC ===");
    scan_near(&audio, 2606.0, 0.2, "after");
    let q_after = {
        let (cd0, _c) = downsample(&audio, 2606.0, None);
        let refined = mfsk_core::core::sync::refine_candidate::<Ft8>(
            &cd0,
            &mfsk_core::core::sync::SyncCandidate {
                freq_hz: 2606.0,
                dt_sec: 0.2,
                score: 0.0,
            },
            10,
        );
        let cs = mfsk_core::ft8::decode_block::symbol_spectra_direct::<i16>(
            &audio,
            2606.0,
            refined.dt_sec,
            mfsk_core::ft8::decode_block::SymMask::SyncOnly,
            None,
        );
        sync_quality(&cs)
    };
    println!("  sync_quality at 2606 Hz (dt refined near 0.2): q={q_after} (was {q_before})");

    // 3. Does DL8YHR decode now, after W1FC removal, with a full re-decode?
    let post_results = decode_sniper(&audio, 2606.0, DecodeDepth::BpAllOsd, 20);
    let hit = post_results
        .iter()
        .any(|r| unpack77(&r.message77).as_deref() == Some("CQ DX DL8YHR JO41"));
    println!(
        "\ndecode_sniper(2606 Hz) after W1FC subtraction: {} decode(s), DL8YHR hit: {hit}",
        post_results.len()
    );
    for r in &post_results {
        println!(
            "    {:.1} Hz dt={:+.3} snr={:+.1}  {:?}",
            r.freq_hz,
            r.dt_sec,
            r.snr_db,
            unpack77(&r.message77)
        );
    }
}

#[test]
#[ignore]
fn probe_subtract_depth_on_w1fc_itself() {
    let mut audio = load_wav_i16(Path::new(QSO3_PATH));

    let w1fc_results = decode_sniper(&audio, 2571.0, DecodeDepth::BpAllOsd, 20);
    let w1fc = w1fc_results
        .iter()
        .find(|r| unpack77(&r.message77).as_deref() == Some("W1FC F5BZB -8"))
        .expect("W1FC F5BZB should decode near 2571 Hz")
        .clone();

    println!("\n=== Costas score AT W1FC's own frequency, before vs after subtracting it ===");
    scan_near(&audio, w1fc.freq_hz, w1fc.dt_sec, "before (at W1FC freq)");
    let before_peak = {
        let (cd0, _c) = downsample(&audio, w1fc.freq_hz, None);
        let d = SyncDims::of::<Ft8>();
        let blocks = <Ft8 as FrameLayout>::SYNC_MODE.blocks();
        let first = &blocks[0];
        let csync = make_costas_ref(first.pattern, d.ds_spb);
        let i0 = ((w1fc.dt_sec + Ft8::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
        score_costas_block(&cd0, &csync, d.ds_spb, i0)
    };

    let refined_freq = refine_signal_freq(&audio, &w1fc);
    let mut w1fc_refined = w1fc.clone();
    w1fc_refined.freq_hz = refined_freq;
    subtract_signal_lpf(&mut audio, &w1fc_refined);

    scan_near(&audio, w1fc.freq_hz, w1fc.dt_sec, "after (at W1FC freq)");
    let after_peak = {
        let (cd0, _c) = downsample(&audio, w1fc.freq_hz, None);
        let d = SyncDims::of::<Ft8>();
        let blocks = <Ft8 as FrameLayout>::SYNC_MODE.blocks();
        let first = &blocks[0];
        let csync = make_costas_ref(first.pattern, d.ds_spb);
        let i0 = ((w1fc.dt_sec + Ft8::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
        score_costas_block(&cd0, &csync, d.ds_spb, i0)
    };

    let ratio_db = 10.0 * (before_peak / after_peak).log10();
    println!(
        "\nCostas-block power at W1FC's own freq/dt: before={before_peak:.1}  after={after_peak:.1}  suppression={ratio_db:.1} dB (power ratio, not amplitude)"
    );
}

#[test]
#[ignore]
fn probe_w1fc_sync_cv_and_snr_context() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let w1fc_results = decode_sniper(&audio, 2571.0, DecodeDepth::BpAllOsd, 20);
    let w1fc = w1fc_results
        .iter()
        .find(|r| unpack77(&r.message77).as_deref() == Some("W1FC F5BZB -8"))
        .expect("W1FC F5BZB should decode near 2571 Hz");
    println!(
        "\nW1FC: freq={:.2} dt={:+.3} snr_db={:+.1} sync_cv={:.4} hard_errors={} sync_score={:.2}",
        w1fc.freq_hz, w1fc.dt_sec, w1fc.snr_db, w1fc.sync_cv, w1fc.hard_errors, w1fc.sync_score
    );
    println!(
        "(sync_cv > 0.3 per decode.rs doc comment => QSB/fading flagged, partial-gain subtract kicks in)"
    );
}

#[test]
#[ignore]
fn probe_iterative_subtract_convergence() {
    let mut audio = load_wav_i16(Path::new(QSO3_PATH));

    let w1fc_results = decode_sniper(&audio, 2571.0, DecodeDepth::BpAllOsd, 20);
    let w1fc = w1fc_results
        .iter()
        .find(|r| unpack77(&r.message77).as_deref() == Some("W1FC F5BZB -8"))
        .expect("W1FC F5BZB should decode near 2571 Hz")
        .clone();

    let refined_freq = refine_signal_freq(&audio, &w1fc);
    let mut w1fc_refined = w1fc.clone();
    w1fc_refined.freq_hz = refined_freq;

    let d = SyncDims::of::<Ft8>();
    let blocks = <Ft8 as FrameLayout>::SYNC_MODE.blocks();
    let first = &blocks[0];
    let csync = make_costas_ref(first.pattern, d.ds_spb);
    let i0 = ((w1fc.dt_sec + Ft8::TX_START_OFFSET_S) * d.ds_rate).round() as i32;

    let score_at = |audio: &[i16]| -> f32 {
        let (cd0, _c) = downsample(audio, w1fc.freq_hz, None);
        score_costas_block(&cd0, &csync, d.ds_spb, i0)
    };

    let s0 = score_at(&audio);
    println!("\n=== iterative re-subtract of W1FC (same candidate, same audio buffer) ===");
    println!("  iter 0 (baseline): score={s0:.1}");

    let mut prev = s0;
    for iter in 1..=6 {
        subtract_signal_lpf(&mut audio, &w1fc_refined);
        let s = score_at(&audio);
        let step_db = 10.0 * (prev as f64 / s as f64).log10();
        let cum_db = 10.0 * (s0 as f64 / s as f64).log10();
        println!("  iter {iter}: score={s:.1}  step={step_db:.2} dB  cumulative={cum_db:.2} dB");
        prev = s;
    }

    let post_results = decode_sniper(&audio, 2606.0, DecodeDepth::BpAllOsd, 20);
    let hit = post_results
        .iter()
        .any(|r| unpack77(&r.message77).as_deref() == Some("CQ DX DL8YHR JO41"));
    println!(
        "\ndecode_sniper(2606 Hz) after 6x W1FC re-subtract: {} decode(s), DL8YHR hit: {hit}",
        post_results.len()
    );
    for r in &post_results {
        println!(
            "    {:.1} Hz dt={:+.3} snr={:+.1}  {:?}",
            r.freq_hz,
            r.dt_sec,
            r.snr_db,
            unpack77(&r.message77)
        );
    }
}
