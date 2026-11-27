// SPDX-License-Identifier: GPL-3.0-only
//! `icap-sic <slot.wav>...`: the cells flow's SIC (`sic_passes`, the board's build: 8 kHz, fixed-point, no AP pass) with the OSD fallback in one of four ways, over slots that have
//! `<slot>.ref` beside them (one reference message per line). Environment:
//!
//! - `PASSES` (default 2): SIC passes in all.
//! - `OSD_MODE`: `off` (default; no OSD), `inline` (the OSD at once for every failed attempt, as `ft8b.f90`/`ft8dec`: the reference for the most the OSD can add),
//!   `round` (`sic_passes_osd`: in every round the best `OSD_CAP` queued attempts run the OSD, and their decodes are subtracted for the next pass),
//!   `final` (the rounds queue, no OSD runs in them; after the last pass the best `OSD_CAP` queued attempts run on the final residual cells: nothing of it is subtracted).
//! - `OSD_CAP` (default 6), `OSD_SEEDS=ad|faithful` (default `ad`), `MAX_CAND` (default 30).
//!
//! TSV on stdout: `D <slot> <message> <in the reference>` for every decode, then `N <slot> <BP passes> <OSD attempts queued> <OSD attempts run>`. Design note section 29.
use icap_app_hosttest::{fft_shim, resample::resample, wav};
use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    CELLS_D_MIN_SCORE, CellTables, CellsWorker, Found, Ft8BpScratch, Ft8Rate, GatedSearch, Grid,
    OsdSeeds, SicStorage, coarse_sync_gated_cells, sic_passes, sic_passes_osd, sic_passes_with,
    sic_signal_capacity,
};
use mfsk_core::msg::wsjt77::unpack77;
use std::collections::HashSet;

fn env_or<T: std::str::FromStr>(k: &str, d: T) -> T {
    std::env::var(k)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(d)
}

fn main() {
    let passes: usize = env_or("PASSES", 2);
    let max_cand: usize = env_or("MAX_CAND", 30);
    let cap: usize = env_or("OSD_CAP", 6);
    let mode = std::env::var("OSD_MODE").unwrap_or_else(|_| "off".into());
    let seeds = if std::env::var("OSD_SEEDS").is_ok_and(|v| v == "faithful") {
        OsdSeeds::Faithful
    } else {
        OsdSeeds::AD
    };
    let depth = DecodeDepth {
        osd: mode != "off",
        ..DecodeDepth::EMBEDDED
    };
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
        .with_d_min_score(CELLS_D_MIN_SCORE)
        .with_osd_seeds(seeds)
        .with_osd_deferred(mode == "final");
    let rate = Ft8Rate::Hz8000;
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let mut storage = SicStorage::new(rate, sic_signal_capacity(max_cand, passes));
    for path in std::env::args().skip(1) {
        let a = resample(&wav::load_i16(&path), 12_000, 8_000);
        let refs: HashSet<String> = std::fs::read_to_string(path.replace(".wav", ".ref"))
            .unwrap()
            .lines()
            .map(|l| l.split_whitespace().collect::<Vec<_>>().join(" "))
            .collect();
        let slot = std::path::Path::new(&path)
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_string();
        let (cands, mut cells) =
            coarse_sync_gated_cells(&mut SliceSource(&a), rate, &search, 1).unwrap();
        let _ = w.take_bp_stats();
        let _ = w.take_osd_counts();
        let mut src = SliceSource(&a);
        let results = match mode.as_str() {
            "round" => sic_passes_osd(
                &mut src,
                rate,
                &search,
                max_cand,
                depth,
                passes,
                &cands,
                &mut cells,
                &tables,
                &mut w,
                &mut storage.scratch(),
                cap,
            ),
            "final" => {
                // the worker defers (see `with_osd_deferred` below): the rounds queue the attempts that fail the BP staircase, nothing runs the OSD
                let mut res = sic_passes_with(
                    &mut src,
                    rate,
                    &search,
                    max_cand,
                    passes,
                    &cands,
                    &mut cells,
                    &tables,
                    &mut storage.scratch(),
                    |cells, cands, mc, known| {
                        w.decode_round(cells, cands, mc, depth, 0.25, known);
                    },
                );
                // then the best `cap` of the slot's queue on the final (residual) cells; what it decodes is not subtracted
                let mut known: Vec<Found> = res
                    .drain(..)
                    .map(|r| Found {
                        res: r,
                        grid: Grid { j0: 0, k: 0 },
                    })
                    .collect();
                w.osd_pass(&cells, cap, depth, &mut known);
                known.into_iter().map(|f| f.res).collect()
            }
            "inline" => sic_passes(
                &mut src,
                rate,
                &search,
                max_cand,
                depth,
                passes,
                &cands,
                &mut cells,
                &tables,
                &mut w,
                &mut storage.scratch(),
            ),
            _ => sic_passes(
                &mut src,
                rate,
                &search,
                max_cand,
                depth,
                passes,
                &cands,
                &mut cells,
                &tables,
                &mut w,
                &mut storage.scratch(),
            ),
        };
        for r in &results {
            let m = unpack77(r.message77())
                .map(|m| m.split_whitespace().collect::<Vec<_>>().join(" "))
                .unwrap_or_default();
            println!("D\t{slot}\t{m}\t{}", refs.contains(&m) as u8);
        }
        let (q, ran) = w.take_osd_counts();
        println!("N\t{slot}\t{}\t{q}\t{ran}", w.take_bp_stats().passes);
    }
    let _ = fft_shim::factory_calls();
}
