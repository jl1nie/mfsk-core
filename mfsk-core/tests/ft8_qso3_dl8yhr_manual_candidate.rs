//! Bypass coarse_sync entirely and probe BP/OSD directly at jt9's exact
//! reported coordinates for `CQ DX DL8YHR JO41`: freq=2606 Hz, dt=0.2 s,
//! SNR=-17 dB (from `jt9 -8 -d3` on `qso3_busy.wav`; column format
//! confirmed against WSJT-X `lib/decoder.f90:641`
//! `1000 format(i6.6,i4,f5.1,i5,' ~ ',...)` = utc,snr,dt,freq).
//!
//! mfsk-core's own `coarse_sync` near this frequency ranks a candidate
//! at freq=2615.62 dt=+0.86 far higher (score=90.45) than anything near
//! jt9's coordinates (best nearby: freq=2609.38 dt=+0.385 score=4.39) —
//! see `ft8_qso3_dl8yhr_trace.rs`. Neither converges to real text via
//! BP/OSD. This test asks: if we skip coarse_sync's candidate selection
//! and hand-place a candidate exactly where jt9 says the signal is, does
//! BP/OSD succeed? If yes, the bug is in coarse_sync's peak
//! detection/scoring near 2606 Hz, not in BP/OSD.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_dl8yhr_manual_candidate -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::core::sync::{SyncCandidate, refine_candidate};
use mfsk_core::fec::ldpc::bp::bp_decode;
use mfsk_core::fec::ldpc::osd::{osd_decode_deep4, osd_decode_npre1, osd_decode_npre1_npre2};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode_block::{SymMask, fill_symbol_spectra, symbol_spectra_direct};
use mfsk_core::ft8::downsample::downsample;
use mfsk_core::ft8::llr::{compute_llr, sync_quality};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

#[test]
#[ignore]
fn manual_candidate_at_jt9_coordinates() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let target = "CQ DX DL8YHR JO41";

    // jt9's exact reported coordinates, and a small grid around it in
    // case its printed dt/freq are rounded (dt to 0.1s, freq to 1 Hz).
    let mut freqs = vec![2606.0f32];
    for df in [-6.25, -3.0, 3.0, 6.25, -9.0, 9.0] {
        freqs.push(2606.0 + df);
    }
    let mut dts = vec![0.2f32];
    for ddt in [-0.05, 0.05, -0.1, 0.1, -0.16, 0.16] {
        dts.push(0.2 + ddt);
    }

    let mut best: Option<(f32, f32, u32, u32, String)> = None; // freq, dt, q, hard_errors, text

    for &freq in &freqs {
        for &dt in &dts {
            let cand = SyncCandidate {
                freq_hz: freq,
                dt_sec: dt,
                score: 0.0,
            };
            let (cd0, _cache) = downsample(&audio, cand.freq_hz, None);
            let refined = refine_candidate::<Ft8>(&cd0, &cand, 10);

            let mut cs = symbol_spectra_direct::<i16>(
                &audio,
                cand.freq_hz,
                refined.dt_sec,
                SymMask::SyncOnly,
                None,
            );
            let q = sync_quality(&cs);
            fill_symbol_spectra(
                &mut cs,
                &audio,
                cand.freq_hz,
                refined.dt_sec,
                SymMask::DataOnly,
                None,
            );
            let llr_set = compute_llr::<f32>(&cs);

            for llr in [&llr_set.llra, &llr_set.llrb, &llr_set.llrc, &llr_set.llrd] {
                if let Some(bp) = bp_decode(llr, None, 40, None) {
                    let text = unpack77(&bp.message77).unwrap_or_default();
                    if text == target {
                        println!(
                            "BP HIT: freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q} hard_errors={} text=\"{text}\"",
                            refined.dt_sec, bp.hard_errors
                        );
                    }
                }
                let osd = if q >= 18 {
                    osd_decode_npre1_npre2(llr)
                } else {
                    osd_decode_npre1(llr)
                };
                if let Some(o) = osd {
                    let text = unpack77(&o.message77).unwrap_or_default();
                    if text == target {
                        println!(
                            "OSD(wsjtx-faithful) HIT: freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q} hard_errors={} text=\"{text}\"",
                            refined.dt_sec, o.hard_errors
                        );
                    }
                }
                if let Some(o) = osd_decode_deep4(llr, 30, None) {
                    let text = unpack77(&o.message77).unwrap_or_default();
                    if text == target {
                        println!(
                            "OSD(deep4) HIT: freq={freq:.2} dt={dt:.3} refined_dt={:+.3} q={q} hard_errors={} text=\"{text}\"",
                            refined.dt_sec, o.hard_errors
                        );
                    }
                }
            }

            let is_better = match &best {
                None => true,
                Some((_, _, bq, _, _)) => q > *bq,
            };
            if is_better {
                best = Some((freq, dt, q, 0, String::new()));
            }
        }
    }

    if let Some((freq, dt, q, _, _)) = best {
        println!("\nBest sync_quality across grid: freq={freq:.2} dt={dt:.3} q={q}");
    }
    println!(
        "(if no HIT lines printed above, BP/OSD cannot recover the text even when hand-placed at jt9's exact coordinates)"
    );
}
