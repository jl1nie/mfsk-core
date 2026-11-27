// SPDX-License-Identifier: GPL-3.0-only
//! `icap-cands <slot.wav>...`: every candidate of the cells flow (30 of them, the board's build: no AP pass, OSD off), one decode attempt each.
//! Each slot has `<slot>.ref` beside it (one reference message per line). TSV on stdout: slot, rank, freq, dt, score, hard sync, decoded,
//! in the reference, message, BP calls the candidate cost, BP message-update passes ([`BpStats`](mfsk_core::fec::ldpc::bp::BpStats)). `scripts/cands-yield.py`
//! reads it (design note section 27).
use icap_app_hosttest::{fft_shim, resample::resample, wav};
use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    CELLS_D_MIN_SCORE, CELLS_GATE, CELLS_GATE_OSD, CellTables, CellsWorker, Ft8BpScratch, Ft8Rate,
    GatedSearch, OsdSeeds, coarse_sync_gated_cells,
};
use mfsk_core::msg::wsjt77::unpack77;
use std::collections::HashSet;
fn main() {
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    // the floor of variant `d`: the flow's (16) unless D_MIN_SCORE says otherwise (0 = the whole staircase, as before Issue #765)
    let d_min = std::env::var("D_MIN_SCORE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(CELLS_D_MIN_SCORE);
    // GATE=1: the gate before BP (`CELLS_GATE`); MAX_CAND: how many of the search's candidates are decoded (30, the search keeps 30)
    let gate = std::env::var("GATE").is_ok_and(|v| v == "1");
    let max_cand: usize = std::env::var("MAX_CAND")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(30);
    // OSD=1: the OSD fallback on (the gate is then CELLS_GATE_OSD); OSD_SEEDS=ad: variants a and d only (`OsdSeeds::AD`)
    let osd = std::env::var("OSD").is_ok_and(|v| v == "1");
    let seeds = if std::env::var("OSD_SEEDS").is_ok_and(|v| v == "ad") {
        OsdSeeds::AD
    } else {
        OsdSeeds::Faithful
    };
    let depth = DecodeDepth {
        osd,
        ..DecodeDepth::EMBEDDED
    };
    let gate_model = if osd { &CELLS_GATE_OSD } else { &CELLS_GATE };
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
        .with_d_min_score(d_min)
        .with_osd_seeds(seeds)
        .with_gate(gate.then_some(gate_model));
    for path in std::env::args().skip(1) {
        let slot12 = wav::load_i16(&path);
        let a = resample(&slot12, 12_000, 8_000);
        let refp = path.replace(".wav", ".ref");
        let refs: HashSet<String> = std::fs::read_to_string(&refp)
            .unwrap()
            .lines()
            .map(|l| l.split_whitespace().collect::<Vec<_>>().join(" "))
            .collect();
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        let slot = std::path::Path::new(&path)
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_string();
        for (i, (c, hs)) in cands.iter().take(max_cand).enumerate() {
            let _ = w.take_bp_stats();
            let f = w.decode_candidate_hs(&cells, c, *hs, depth, 0.25);
            let bp = w.take_bp_stats();
            let (dec, truth, msg) = match &f {
                Some(f) => {
                    let m = unpack77(f.res.message77())
                        .map(|m| m.split_whitespace().collect::<Vec<_>>().join(" "))
                        .unwrap_or_default();
                    (1, refs.contains(&m) as u8, m)
                }
                None => (0, 0, String::new()),
            };
            println!(
                "{slot}\t{i}\t{:.2}\t{:.3}\t{}\t{hs}\t{dec}\t{truth}\t{msg}\t{}\t{}",
                c.freq_hz, c.dt_sec, c.score, bp.calls, bp.passes
            );
        }
    }
    let _ = fft_shim::factory_calls();
}
