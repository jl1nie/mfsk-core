//! The cells flow stage by stage on the CoreS3, one core, wav replay from PSRAM, no WiFi / USB host: what the host's instruction counts become in milliseconds, per stage.
//!
//! Per recording (30 candidates, OSD off; each measurement made twice, the second is read):
//!
//! | stage | what is timed |
//! |---|---|
//! | `search` | `coarse_sync_gated_cells`: the demux (the 2 560-point FFTs, the soft `Plan2560Sc16` here: the app would use esp-dsp) and `band_best` |
//! | `band` | `search_cells` over every carrier of the made store, then `merge_cells_parts`: `band_best` and the merge alone (the part left after the slot's end) |
//! | `stage 3` | the 30 candidates through `CellsWorker::decode_candidate`, one at a time (timed each), with the variant `d` floor at 0 (the whole staircase) and at `CELLS_D_MIN_SCORE` (16, Issue #765) |
//!
//! For every candidate a `cand` line: its rank, score, hard sync, microseconds, BP calls and passes (`BpStats`), decoded or not. After each stage-3 run a `s3` line: total
//! time, calls, passes, decodes, and the least-squares line of time on passes over the candidates (`us = a + b * passes`: **b is the time of one BP pass**, `a` what a candidate
//! costs apart from BP). Logged: heap minima and the thread's stack high-water mark.
//!
//! **Predictions, written before the first run** (from the host's instruction counts of `scripts/s3-stage-instr.sh` and `icap-cands`, at the 8.8-10.7 ns per M instruction the
//! earlier board runs gave; a BP pass is about 65 000 instructions in the cells flow, measured per slot on x86):
//!
//! | | qso3_busy | qso1 | qso2 |
//! |---|---|---|---|
//! | `band` (64 M, `band_best` + merge) | 0.56-0.68 s | 0.55-0.67 s | 0.55-0.67 s |
//! | `search` - `band` (the demux) | 1.1-1.2 s | 1.15-1.25 s | 1.1-1.2 s |
//! | `stage 3`, floor 0 (144 / 119 / 135 M) | 1.27 s | 1.11 s | 1.23 s |
//! | `stage 3`, floor 16 (108 / 72 / 81 M) | 0.95 s | 0.67 s | 0.74 s |
//! | BP passes, floor 0 / 16 (host: exact) | 2 145 / 1 360 | 1 781 / 899 | 2 021 / 1 012 |
//! | decodes, both floors | 9 | 4 | 5 |
//!
//! **Why `band` is 33-40 % over (it was 0.56-0.68 s, it is 0.90-0.94 s), a second run (`band narrow` lines):** two hypotheses the host cannot tell apart (host: `band_best` 64 M instructions, 14.0 M
//! conditional branches, 0.49 M mispredicts, about 1 200 instructions and 260 branches a position; the board spends about 4 000 cycles a position). **(CPU)** the scoring loop is branch- and
//! integer-bound: a narrow range of carriers, whose cells fit the data cache, costs the same per position as the whole band (within 15 %). **(MEM)** it waits for PSRAM (the 172 KB store does not fit
//! the cache: 21 symbols x up to 4 reads a position, rows a stride apart): a narrow range costs **at most 55 %** of a position's time in the whole band. `band narrow W` times `search_cells` over
//! four ranges of W carriers (64 and 256; each range once to warm, once timed) and gives nanoseconds a position, beside the whole band's.
//!
//! **The regrouped scoring (`carrier_scores`), predicted before its run:** `band_best` on the host went from 63.8 M instructions / 14.0 M conditional branches to 42.4 M / 5.6 M (bit for bit the same scores).
//! The board spent about 4 000 cycles a position on the old loop (16.7 us). If the time follows the instructions, `band` falls to 0.60 s (-34 %); if it follows the branches (an in-order core with no
//! prediction pays for each taken one), toward 0.40 s. **Predicted: `band` 0.40-0.62 s** (it was 0.90-0.94 s); `band narrow` per position equal to the whole band's within 5 % (still CPU-bound);
//! the rest of the table unchanged (the demux, stage 3, the BP pass). Stop rule as above.
//!
//! **The flat-table NMS decoder, predicted before its run:** the BP pass went from 76 000 to 39 900 host instructions (-47.5 %: the searches for an edge's slot and for the check's place in a bit's list
//! gone, the bounds checks of the table-driven reads gone), bit for bit the same decodes and pass counts (so the passes below are the previous run's: floor 16 1 240 / 951 / 995, floor 0 1 987 / 1 838 / 1 947).
//! The board spent 0.69-0.75 ms a pass. **Predicted: one BP pass 0.35-0.48 ms** (if the time follows the instructions 0.36-0.39, the removed bounds checks are branches too: toward 0.35);
//! stage 3 at floor 16 **0.50-0.67 / 0.38-0.51 / 0.39-0.52 s** (it was 0.927 / 0.708 / 0.741), at floor 0 0.75-1.0 / 0.65-0.85 / 0.70-0.95 s. Decodes 9 / 4 / 5. The `band` and the demux unchanged.
//!
//! **The gate before BP (`CELLS_GATE`), predicted before its run:** a third stage-3 arm, `floor 16 gate 1` (the `d` floor of the flow and the gate). Host `icap-cands`, 30 candidates, decodes unchanged
//! (9 / 4 / 5): BP passes 1 360 -> 954 (-30 %), 899 -> 297 (-67 %), 1 012 -> 267 (-74 %). Applied to the board's passes of the floor-16 arm (1 240 / 951 / 995, whose host counts differ by up to 9 %):
//! **passes 870 (800-950) / 314 (280-360) / 263 (240-300)**, at 0.43 ms a pass and the non-BP 1.5-2.5 ms a candidate unchanged: **stage 3 at floor 16 with the gate 0.47 (0.42-0.52) / 0.22
//! (0.17-0.28) / 0.17 (0.12-0.22) s** (it is 0.623 / 0.484 / 0.464 s without); decodes 9 / 4 / 5. The gate's own cost (the statistics of 174 values, two passes) is under 0.1 ms a candidate.
//!
//! **The OSD fallback in stage 3 (`osd_emb`, `OsdSeeds`), predicted before its run:** two more arms at the flow's floor, no gate, `DecodeDepth { osd: true, .. }`: `osd faithful` (`ft8b.f90`'s ten runs a candidate)
//! and `osd ad` (variants `a` and `d`, four runs: none of the other six ever returned on 4 780 slots; the same decodes on all 143 400 candidates of the host corpora). Host `icap-cands`
//! (30 candidates, OSD on): decodes 13 / 5 / 5, both arms; BP passes 1 296 / 868 / 1 003. Instructions over OSD off (callgrind): **faithful +352 / +315 / +371 M, ad +116 / +102 / +122 M**.
//! `ft8-osd-bench` measured one `osd174_91_into` at 7.26 ms, 0.76 M host instructions, i.e. 9.6 ns an instruction; the BP pass is 10.9. At 9.6-10.9 ns:
//! **stage 3, `osd ad`: 1.75-1.9 / 1.42-1.55 / 1.63-1.8 s; `osd faithful`: 4.0-4.4 / 3.4-3.8 / 4.1-4.5 s** (the floor-16 arm without OSD is 0.625 / 0.442 / 0.464 s). Decodes 13 / 5 / 5 in both.
//! The unknown is what the rest of a `try_fallback` costs on the LX7 (the zsum BP, an f32 LLR): the prediction uses the host's instruction ratio for it. Stack: the OSD's own scratch is
//! on the heap, so the stack free should stay within 2 KB of the OSD-off arms' figure.
//!
//! One BP pass (`b`): **0.57-0.70 ms**; a candidate apart from BP (`a`): about 1.5 ms. The stage-3 figures are +-10 %. **Stop rule:** if any stage is more than 25 % off its prediction,
//! stop and explain it on the host before the next flash. The prediction rests on one ns-per-instruction ratio for every stage; the point of the run is to see where it breaks.

extern crate alloc;
extern crate std;

use alloc::boxed::Box;
use alloc::vec::Vec;

use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    CELLS_D_MIN_SCORE, CELLS_GATE, CellTables, OsdSeeds, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch,
    coarse_sync_gated_cells, merge_cells_parts, search_cells,
};
use mfsk_core::msg::wsjt77::unpack77;

const INTERNAL: u32 = (1 << 11) | (1 << 2);
const PSRAM: u32 = (1 << 10) | (1 << 2);
const MAX_CAND: usize = 30;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}
fn min_free(caps: u32) -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_minimum_free_size(caps) }
}
fn stack_free() -> usize {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) as usize }
}

pub fn run(target: &str, wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    log::info!("stage-bench: {target}, the cells flow stage by stage, one core (predictions in the module doc)");
    let h = std::thread::Builder::new()
        .name("stage_bench".into())
        .stack_size(48 * 1024)
        .spawn(move || body(wavs))
        .expect("stage-bench: could not spawn the bench thread");
    let _ = h.join();
    log::info!("stage-bench: end");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

struct Row {
    rank: usize,
    score: f32,
    hs: u8,
    us: i64,
    calls: u32,
    passes: u64,
    dec: bool,
}

/// `us = a + b * passes` by least squares over the rows (b in microseconds a pass).
fn fit(rows: &[Row]) -> (f64, f64) {
    let n = rows.len() as f64;
    let (sx, sy): (f64, f64) = rows.iter().fold((0.0, 0.0), |(x, y), r| (x + r.passes as f64, y + r.us as f64));
    let (mx, my) = (sx / n, sy / n);
    let (mut sxx, mut sxy) = (0.0, 0.0);
    for r in rows {
        let (dx, dy) = (r.passes as f64 - mx, r.us as f64 - my);
        sxx += dx * dx;
        sxy += dx * dy;
    }
    let b = if sxx > 0.0 { sxy / sxx } else { 0.0 };
    (my - b * mx, b)
}

fn body(wavs: &'static [(&'static str, &'static [u8])]) {
    log::info!("stage-bench: boot min free internal {} B, PSRAM {} B; stack free {} B", min_free(INTERNAL), min_free(PSRAM), stack_free());
    let search = GatedSearch::new(200.0, 3000.0, 1.3, MAX_CAND);
    let rate = Ft8Rate::Hz8000;
    let tables = CellTables::new();
    // SAFETY: this thread is the only user of the main staging buffer here.
    let cs = unsafe { crate::internal_pool::cs_scratch_main() };
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(cs, &mut bp, &tables);

    for (label, wav) in wavs {
        let slot12: Vec<i16> = wav[44..]
            .chunks_exact(2)
            .take(180_000)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        let a8 = resample_to_8k(&slot12, 12_000);

        // search (the demux and band_best) and band (band_best and the merge alone, on the store the search made)
        let (mut t_search, mut t_band) = (0i64, 0i64);
        let mut made = None;
        for _ in 0..2 {
            let t0 = now_us();
            let (cands, cells) = coarse_sync_gated_cells(&mut SliceSource(&a8), rate, &search, 1).unwrap();
            t_search = now_us() - t0;
            let t1 = now_us();
            let part = search_cells(&cells, &search, cells.carrier_bins());
            let again = merge_cells_parts(&[part], &search, rate);
            t_band = now_us() - t1;
            assert_eq!(again.len(), cands.len(), "{label}: the split search is the search");
            made = Some((cands, cells));
        }
        let (cands, cells) = made.unwrap();
        // band narrow: the same scoring over a few carriers, a position being one (carrier, lag) of the 60 lags of `search_cells`
        {
            let bins = cells.carrier_bins();
            let full = bins.len();
            log::info!("stage-bench: {label} band full {} ns a position ({full} carriers x 60 lags)", t_band * 1000 / (full as i64 * 60));
            for width in [64usize, 256] {
                let (mut tot, mut npos) = (0i64, 0i64);
                for q in 0..4usize {
                    let lo = bins.start + q * (full - width) / 3;
                    let r = lo..lo + width;
                    let _ = search_cells(&cells, &search, r.clone());
                    let t0 = now_us();
                    let _ = search_cells(&cells, &search, r);
                    tot += now_us() - t0;
                    npos += width as i64 * 60;
                }
                log::info!("stage-bench: {label} band narrow {width} {} ns a position", tot * 1000 / npos);
            }
        }
        log::info!(
            "stage-bench: {label} search {t_search} us, band {t_band} us (demux {} us), {} candidates; min free internal {} B, PSRAM {} B",
            t_search - t_band,
            cands.len(),
            min_free(INTERNAL),
            min_free(PSRAM)
        );

        // stage 3 at floor 0 and at the flow's floor
        for (floor, gate_on, osd) in [
            (0.0f32, false, None),
            (CELLS_D_MIN_SCORE, false, None),
            (CELLS_D_MIN_SCORE, true, None),
            (CELLS_D_MIN_SCORE, false, Some(OsdSeeds::Faithful)),
            (CELLS_D_MIN_SCORE, false, Some(OsdSeeds::AD)),
        ] {
            let depth = DecodeDepth { osd: osd.is_some(), ..DecodeDepth::EMBEDDED };
            let osd_label = match osd {
                None => "off",
                Some(OsdSeeds::AD) => "ad",
                Some(_) => "faithful",
            };
            let mut rows: Vec<Row> = Vec::new();
            let mut set: Vec<alloc::string::String> = Vec::new();
            let mut total = 0i64;
            for k in 0..2 {
                rows.clear();
                set.clear();
                worker = worker
                    .with_d_min_score(floor)
                    .with_osd_seeds(osd.unwrap_or(OsdSeeds::Faithful))
                    .with_gate(gate_on.then_some(&CELLS_GATE));
                let _ = worker.take_bp_stats();
                let t_all = now_us();
                for (i, (c, hs)) in cands.iter().take(MAX_CAND).enumerate() {
                    let t0 = now_us();
                    let f = worker.decode_candidate_hs(&cells, c, *hs, depth, 0.25);
                    let us = now_us() - t0;
                    let st = worker.take_bp_stats();
                    if let Some(f) = &f {
                        if let Some(t) = unpack77(f.res.message77()) {
                            set.push(t);
                        }
                    }
                    rows.push(Row { rank: i, score: c.score, hs: *hs, us, calls: st.calls, passes: st.passes, dec: f.is_some() });
                }
                total = now_us() - t_all;
                let _ = k;
            }
            set.sort();
            set.dedup();
            for r in &rows {
                log::info!(
                    "stage-bench: cand {label} floor {floor} gate {} osd {osd_label} #{} score {} hs {} {} us calls {} passes {} {}",
                    u8::from(gate_on), r.rank, r.score, r.hs, r.us, r.calls, r.passes, if r.dec { "DEC" } else { "-" }
                );
            }
            let (calls, passes): (u32, u64) = rows.iter().fold((0, 0), |(c, p), r| (c + r.calls, p + r.passes));
            let (a, b) = fit(&rows);
            log::info!(
                "stage-bench: s3 {label} floor {floor} gate {} osd {osd_label}: {total} us, calls {calls}, passes {passes}, {} decodes; us = {a:.0} + {b:.1} * passes (one BP pass {:.3} ms)",
                u8::from(gate_on),
                set.len(),
                b / 1000.0
            );
        }
        log::info!("stage-bench: {label} min free internal {} B, PSRAM {} B, stack free {} B", min_free(INTERNAL), min_free(PSRAM), stack_free());
    }
    log::info!("stage-bench: done");
}
