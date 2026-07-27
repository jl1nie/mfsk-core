//! Per-candidate internal trace for `CQ DX DL8YHR JO41` @2606 Hz
//! (coarse_sync's best nearby candidate: freq=2615.62, dt=0.86,
//! score=90.45 — by far the strongest in-band candidate, see
//! `ft8_qso3_dl8yhr_probe.rs`). `decode_frame_subtract` misses this
//! message at every strictness/dt_tol/max_cand setting tried so far;
//! this traces sync_quality + BP + OSD hard_errors directly on that
//! one candidate to find which stage rejects it.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft8_qso3_dl8yhr_trace -- --ignored --nocapture
//! ```
#![cfg(feature = "fft-rustfft")]

use std::path::Path;

use mfsk_core::engine::sync::refine_candidate;
use mfsk_core::fec::ldpc::bp::bp_decode;
use mfsk_core::fec::ldpc::osd::{
    osd_decode, osd_decode_deep4, osd_decode_npre1, osd_decode_npre1_npre2,
};
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode_block::{
    SymMask, coarse_sync, compute_spectrogram, fill_symbol_spectra, symbol_spectra_direct,
};
use mfsk_core::ft8::downsample::downsample;
use mfsk_core::ft8::llr::{compute_llr, sync_quality};
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

use common::load_wav_i16;

#[test]
#[ignore]
fn trace_dl8yhr_candidate() {
    let audio = load_wav_i16(Path::new(QSO3_PATH));
    let target = "CQ DX DL8YHR JO41";

    let spec = compute_spectrogram(&audio, 3000.0);
    let candidates = coarse_sync(&spec, 100.0, 3000.0, 0.8, 200);
    let mut near: Vec<_> = candidates
        .iter()
        .filter(|c| (c.freq_hz - 2606.0).abs() <= 12.0)
        .collect();
    near.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());

    println!("\n=== trace: {} candidate(s) near 2606 Hz ===", near.len());

    for cand in near.iter().take(4) {
        let (cd0, _cache) = downsample(&audio, cand.freq_hz, None);
        let refined = refine_candidate::<Ft8>(&cd0, cand, 10);

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

        // BP staircase (4 llr variants).
        let mut bp_hit: Option<(&str, u32, String)> = None;
        for (label, llr) in [
            ("a", &llr_set.llra),
            ("b", &llr_set.llrb),
            ("c", &llr_set.llrc),
            ("d", &llr_set.llrd),
        ] {
            if let Some(bp) = bp_decode(llr, None, 40, None) {
                let text = unpack77(&bp.message77).unwrap_or_default();
                bp_hit = Some((label, bp.hard_errors, text));
                break;
            }
        }

        // OSD staircase — mirror try_fallback's own q-conditional dispatch,
        // plus osd_decode (legacy ndeep2) and osd_decode_deep4 for reference.
        let mut osd_results = Vec::new();
        for (label, llr) in [
            ("a", &llr_set.llra),
            ("b", &llr_set.llrb),
            ("c", &llr_set.llrc),
            ("d", &llr_set.llrd),
        ] {
            let wsjtx_faithful = if q >= 18 {
                osd_decode_npre1_npre2(llr)
            } else {
                osd_decode_npre1(llr)
            };
            let legacy2 = osd_decode(llr);
            let deep4 = osd_decode_deep4(llr, 30, None);
            osd_results.push((label, wsjtx_faithful, legacy2, deep4));
        }

        println!(
            "\n  candidate freq={:.2} dt={:+.3} score={:.2} refined_dt={:+.3}  sync_quality(q)={q}",
            cand.freq_hz, cand.dt_sec, cand.score, refined.dt_sec
        );
        match &bp_hit {
            Some((label, e, text)) => println!(
                "    BP[{label}] hard_errors={e}  text=\"{text}\"  match={}",
                text == target
            ),
            None => println!("    BP: no variant converged"),
        }
        for (label, wf, l2, d4) in &osd_results {
            let fmt = |r: &Option<mfsk_core::fec::ldpc::osd::OsdResult>| -> String {
                match r {
                    Some(o) => {
                        let text = unpack77(&o.message77).unwrap_or_default();
                        format!("e={} \"{}\" match={}", o.hard_errors, text, text == target)
                    }
                    None => "none".to_string(),
                }
            };
            println!(
                "    OSD[{label}] wsjtx-faithful(q>=18?npre12:npre1)={}  legacy-ndeep2={}  deep4(k=30)={}",
                fmt(wf),
                fmt(l2),
                fmt(d4)
            );
        }
    }
}
