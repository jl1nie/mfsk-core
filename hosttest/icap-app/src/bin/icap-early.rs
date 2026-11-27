// SPDX-License-Identifier: GPL-3.0-only
//! `icap-early <fill-rows> <slot.wav>...`: the P0 window path's **early emit** on a store that is still being written (design note section 31). Per slot: the complete flow
//! (`CellsOptions::p0_window()`: 30 candidates, `CELLS_GATE_OSD`, `OsdSeeds::AD`, the OSD capped at 6) against the early one: the cells of the first `fill-rows` rows only are
//! searched, the candidates whose rows are all written (`candidate_ready`) are decoded from that partial store, then the store is completed, the full search runs and the candidates
//! with no early counterpart (within 1 cell and 0.12 s) are decoded, then the capped OSD pass over everything queued. Each slot has `<slot>.ref` beside it. One TSV row a slot:
//! slot, complete decodes, of them in the reference, early decodes, of them in the reference, partial-search candidates, ready of them, decoded from the full store after the fill, complete decodes the early flow lost, ones it gained, **decodes at the emit** (the ready candidates alone, before the fill completes), of them in the reference.
use icap_app_hosttest::{fft_shim, resample::resample, wav};
use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    CELLS_D_MIN_SCORE, CELLS_GATE_OSD, CellTables, CellsWorker, Found, Ft8BpScratch, Ft8Rate,
    GatedSearch, HalfDbCells, OsdSeeds, candidate_ready, coarse_sync_gated_cells,
    merge_cells_parts, search_cells,
};
use mfsk_core::msg::wsjt77::unpack77;
use std::collections::HashSet;

fn msg(f: &Found) -> String {
    unpack77(f.res.message77())
        .map(|m| m.split_whitespace().collect::<Vec<_>>().join(" "))
        .unwrap_or_default()
}

fn main() {
    let mut args = std::env::args().skip(1);
    let fill: usize = args.next().unwrap().parse().unwrap();
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
        .with_d_min_score(CELLS_D_MIN_SCORE)
        .with_gate(Some(&CELLS_GATE_OSD))
        .with_osd_seeds(OsdSeeds::AD)
        .with_osd_deferred(true);
    let depth = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let gs = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    for path in args {
        let a = resample(&wav::load_i16(&path), 12_000, 8_000);
        let refs: HashSet<String> = std::fs::read_to_string(path.replace(".wav", ".ref"))
            .unwrap()
            .lines()
            .map(|l| l.split_whitespace().collect::<Vec<_>>().join(" "))
            .collect();
        let (cands, cells) =
            coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &gs, 1).unwrap();
        // the complete flow
        let mut full: Vec<Found> = Vec::new();
        for (c, hs) in cands.iter().take(30) {
            if let Some(f) = w.decode_candidate_hs(&cells, c, *hs, depth, 0.25) {
                full.push(f);
            }
        }
        w.osd_pass(&cells, 6, depth, &mut full);
        let _ = w.take_bp_stats();
        // the early flow: a store with only `fill` rows written
        let mut part = HalfDbCells::new(
            Ft8Rate::Hz8000,
            cells.first_bin(),
            cells.n_freq(),
            cells.n_time(),
        );
        for row in 0..fill.min(cells.n_time()) {
            for (c, bin) in part.row_mut(row).iter_mut().zip(cells.first_bin()..) {
                *c = cells.cell(row, bin).unwrap();
            }
        }
        part.mark_filled(fill);
        let plist = merge_cells_parts(
            &[search_cells(&part, &gs, part.carrier_bins())],
            &gs,
            Ft8Rate::Hz8000,
        );
        let mut early: Vec<Found> = Vec::new();
        let mut early_cands = Vec::new();
        let mut ready = 0;
        for (c, hs) in plist.iter().take(30) {
            if candidate_ready(&part, c) {
                ready += 1;
                early_cands.push(c.clone());
                if let Some(f) = w.decode_candidate_hs(&part, c, *hs, depth, 0.25) {
                    early.push(f);
                }
            }
        }
        // what the window has at the emit: the decodes of the ready candidates alone
        let at_emit: HashSet<String> = early.iter().map(msg).collect();
        // after the fill: the full search; the candidates with no early counterpart
        let mut late = 0;
        for (c, hs) in cands.iter().take(30) {
            let has = early_cands.iter().any(|e| {
                (e.freq_hz - c.freq_hz).abs() <= 3.2 && (e.dt_sec - c.dt_sec).abs() <= 0.12
            });
            if !has {
                late += 1;
                if let Some(f) = w.decode_candidate_hs(&cells, c, *hs, depth, 0.25) {
                    early.push(f);
                }
            }
        }
        w.osd_pass(&cells, 6, depth, &mut early);
        let _ = w.take_bp_stats();
        let fm: HashSet<String> = full.iter().map(msg).collect();
        let em: HashSet<String> = early.iter().map(msg).collect();
        let slot = std::path::Path::new(&path)
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_string();
        println!(
            "{slot}\t{}\t{}\t{}\t{}\t{}\t{ready}\t{late}\t{}\t{}\t{}\t{}",
            fm.len(),
            fm.iter().filter(|m| refs.contains(*m)).count(),
            em.len(),
            em.iter().filter(|m| refs.contains(*m)).count(),
            plist.len(),
            fm.difference(&em).count(),
            em.difference(&fm).count(),
            at_emit.len(),
            at_emit.iter().filter(|m| refs.contains(*m)).count()
        );
    }
    let _ = fft_shim::factory_calls();
}
