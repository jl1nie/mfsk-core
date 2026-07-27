//! Checks whether `CQ DX DL8YHR JO41` (jt9-reported freq=2606 Hz,
//! dt=0.2 s) shows in-slot frequency drift that mfsk-core's refine
//! pipeline doesn't correct for (WSJT-X's `ft8b.f90` applies
//! `twkfreq1`, a polynomial phase de-rotation across the whole downsampled
//! window using coefficients from its sync fit, before computing symbol
//! spectra — mfsk-core's `refine_candidate` only refines `dt_sec`, never
//! touches `freq_hz`).
//!
//! Downsamples at a grid of nearby frequencies and scores the FIRST vs
//! LAST Costas sync block independently at each. If the frequency that
//! maximises the first-block score differs from the frequency that
//! maximises the last-block score, that's direct evidence of in-slot
//! frequency drift.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_dl8yhr_drift_probe -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::FrameLayout;
use mfsk_core::engine::sync::{SyncCandidate, SyncDims, make_costas_ref, score_costas_block};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::downsample::downsample;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

#[test]
#[ignore]
fn probe_dl8yhr_frequency_drift() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));

    let dt_sec = 0.2f32;
    let d = SyncDims::of::<Ft8>();
    let blocks = <Ft8 as FrameLayout>::SYNC_MODE.blocks();
    let first = &blocks[0];
    let last = &blocks[blocks.len() - 1];
    let csync_first = make_costas_ref(first.pattern, d.ds_spb);
    let csync_last = make_costas_ref(last.pattern, d.ds_spb);

    let nominal_i0 = ((dt_sec + Ft8::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
    let block_off_first = (first.start_symbol as usize * d.ds_spb) as i32;
    let block_off_last = (last.start_symbol as usize * d.ds_spb) as i32;

    println!("\n=== DL8YHR frequency-drift probe (dt fixed at {dt_sec}) ===");
    println!(
        "{:>10} {:>14} {:>14}",
        "freq_hz", "first-block", "last-block"
    );

    let mut best_first = (0.0f32, f32::MIN);
    let mut best_last = (0.0f32, f32::MIN);

    let mut freq = 2606.0f32 - 6.25 * 6.0;
    while freq <= 2606.0f32 + 6.25 * 6.0 {
        let (cd0, _cache) = downsample(&audio, freq, None);
        let off_first = nominal_i0 + block_off_first;
        let off_last = nominal_i0 + block_off_last;
        let score_first = score_costas_block(&cd0, &csync_first, d.ds_spb, off_first);
        let score_last = score_costas_block(&cd0, &csync_last, d.ds_spb, off_last);

        println!("{freq:>10.2} {score_first:>14.4} {score_last:>14.4}");

        if score_first > best_first.1 {
            best_first = (freq, score_first);
        }
        if score_last > best_last.1 {
            best_last = (freq, score_last);
        }
        freq += 6.25 / 4.0;
    }

    println!(
        "\nbest freq for FIRST sync block: {:.2} Hz (score {:.4})",
        best_first.0, best_first.1
    );
    println!(
        "best freq for LAST  sync block: {:.2} Hz (score {:.4})",
        best_last.0, best_last.1
    );
    println!(
        "delta = {:.2} Hz  (72 symbols * 0.16s = 11.52s apart => drift rate {:.3} Hz/s)",
        best_last.0 - best_first.0,
        (best_last.0 - best_first.0) / 11.52
    );

    // Dummy usage to keep SyncCandidate import from warning if unused elsewhere.
    let _ = SyncCandidate {
        freq_hz: 0.0,
        dt_sec: 0.0,
        score: 0.0,
    };
}
