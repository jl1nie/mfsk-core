//! issue #177: does `sync_cv` predict how many `subtract_signal_lpf`
//! iterations a candidate can absorb before diminishing returns, or
//! whether iterating it further risks over-subtraction? Decodes the
//! full `qso3_busy.wav` once (baseline `decode_frame_subtract`, no
//! modification), then for every decoded signal independently measures
//! its own iterative self-subtract convergence curve (same methodology
//! as `probe_iterative_subtract_convergence` in
//! `ft8_qso3_dl8yhr_subtract_probe.rs`, generalised to all candidates)
//! and reports it alongside that signal's `sync_cv`.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_sync_cv_iteration_correlation -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::FrameLayout;
use mfsk_core::engine::sync::{SyncDims, make_costas_ref, score_costas_block};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeStrictness};
use mfsk_core::ft8::downsample::downsample;
use mfsk_core::ft8::message::unpack77;
use mfsk_core::ft8::subtract::{refine_signal_freq, subtract_signal_lpf};
use mfsk_core::msg::decode_request::DecodeRequest;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

#[test]
#[ignore]
fn correlate_sync_cv_with_iteration_convergence() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let mut results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3000.0, 0.8, 200)
        .depth(DecodeDepth::FULL)
        .strictness(DecodeStrictness::Normal)
        .staged()
        .decode()
        .results;
    results.sort_by(|a, b| a.freq_hz.partial_cmp(&b.freq_hz).unwrap());

    let d = SyncDims::of::<Ft8>();
    let blocks = <Ft8 as FrameLayout>::SYNC_MODE.blocks();
    let first = &blocks[0];
    let csync = make_costas_ref(first.pattern, d.ds_spb);

    println!(
        "\n{:>10} {:>8} {:>7} {:>7} {:>8} {:>8} {:>8} {:>8}  msg",
        "freq", "sync_cv", "hard_e", "iter1", "iter2", "iter3", "iter4", "cum6dB"
    );

    #[allow(clippy::type_complexity)]
    let mut rows: Vec<(f32, f32, u32, f32, f32, f32, f32, f32, String)> = Vec::new();

    for r in &results {
        // Work on a private copy of the ORIGINAL audio for each signal —
        // we want each candidate's own single-signal convergence curve,
        // not a cumulative multi-signal subtract sequence.
        let mut work = audio.clone();
        let refined_freq = refine_signal_freq(&work, r);
        let mut r_refined = r.clone();
        r_refined.freq_hz = refined_freq;

        let i0 = ((r.dt_sec + Ft8::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
        let score_at = |a: &[i16]| -> f32 {
            let (cd0, _c) = downsample(a, r.freq_hz, None);
            score_costas_block(&cd0, &csync, d.ds_spb, i0)
        };

        let s0 = score_at(&work);
        let mut prev = s0;
        let mut step_db = [0.0f32; 6];
        for (i, step) in step_db.iter_mut().enumerate() {
            let _ = i;
            subtract_signal_lpf(&mut work, &r_refined);
            let s = score_at(&work);
            *step = 10.0 * (prev as f64 / s.max(1.0) as f64).log10() as f32;
            prev = s;
        }
        let cum6: f32 = step_db.iter().sum();

        let text = unpack77(r.message77()).unwrap_or_default();
        println!(
            "{:>10.1} {:>8.3} {:>7} {:>7.2} {:>8.2} {:>8.2} {:>8.2} {:>8.2}  {}",
            r.freq_hz,
            r.sync_cv,
            r.hard_errors,
            step_db[0],
            step_db[1],
            step_db[2],
            step_db[3],
            cum6,
            text
        );
        rows.push((
            r.freq_hz,
            r.sync_cv,
            r.hard_errors,
            step_db[0],
            step_db[1],
            step_db[2],
            step_db[3],
            cum6,
            text,
        ));
    }

    // Simple Pearson correlation: sync_cv vs iter1 (does high sync_cv
    // predict a shallower FIRST-iteration suppression, i.e. does fading
    // hurt the single-shot estimate as hypothesised?), and sync_cv vs
    // cum6 (does high sync_cv predict more residual benefit from
    // continued iteration?), and sync_cv vs (step4/step1) decay-rate
    // ratio (does high sync_cv predict slower per-iteration decay, i.e.
    // "needs more iterations before plateauing"?).
    fn pearson(xs: &[f32], ys: &[f32]) -> f32 {
        let n = xs.len() as f32;
        let mx = xs.iter().sum::<f32>() / n;
        let my = ys.iter().sum::<f32>() / n;
        let mut sxy = 0.0f32;
        let mut sxx = 0.0f32;
        let mut syy = 0.0f32;
        for i in 0..xs.len() {
            let dx = xs[i] - mx;
            let dy = ys[i] - my;
            sxy += dx * dy;
            sxx += dx * dx;
            syy += dy * dy;
        }
        sxy / (sxx.sqrt() * syy.sqrt())
    }

    let cvs: Vec<f32> = rows.iter().map(|r| r.1).collect();
    let iter1: Vec<f32> = rows.iter().map(|r| r.3).collect();
    let cum6: Vec<f32> = rows.iter().map(|r| r.7).collect();
    let decay_ratio: Vec<f32> = rows
        .iter()
        .map(|r| if r.3.abs() > 1e-3 { r.6 / r.3 } else { 0.0 })
        .collect();

    println!(
        "\nPearson r(sync_cv, iter1_dB)     = {:.3}",
        pearson(&cvs, &iter1)
    );
    println!(
        "Pearson r(sync_cv, cum6_dB)      = {:.3}",
        pearson(&cvs, &cum6)
    );
    println!(
        "Pearson r(sync_cv, step4/step1)  = {:.3}  (decay-rate ratio; near 0 = fast plateau, near 1 = still going strong at iter4)",
        pearson(&cvs, &decay_ratio)
    );
}
