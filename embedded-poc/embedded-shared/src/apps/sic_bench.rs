//! SIC on the CoreS3, one core (the S3 plan's SIC-on-ESP32 work, step S2): `sic_passes` over a `SicStorage` and the firmware's internal-DRAM `cs` staging,
//! against `decode_block_cells_sic` (everything allocated per call), and the ring in PSRAM against the ring in internal DRAM.
//!
//! Arms, per recording and number of passes (30 candidates, OSD off; each run twice, the second is read; a run is the search of the cells, then the passes):
//!
//! | arm | memory |
//! |---|---|
//! | `alloc` | `decode_block_cells_sic`: the store, the worker's `cs` and BP scratch, the ring ... made by the call |
//! | `storage` | `sic_passes`: a `SicStorage` made once (its 32 KB ring is above the 2 KB internal threshold: PSRAM), the `cs` staging is `internal_pool`'s, a worker made once |
//! | `ring-int` | as `storage`, but the whole `SicScratch` (ring, chunk buffers, table: 37 888 B) is one internal-DRAM block, `SicScratch::new` over slices of it |
//!
//! A fourth arm, `2core`, runs every round through two workers (core 0: the bench thread, core 1: `sic_w1`, each with its own staging block and BP scratch) that take
//! candidates from one atomic index and are merged by candidate index (`sic_passes_with`; the host test `a_round_shared_by_two_workers_equals_the_sequential_round` shows the
//! merge equals the sequential round). The search, the subtraction and the signals stay on core 0. The `search` figure is the same in every arm (one core).
//!
//! **Predicted before the run of 2026-11-27 (after the `d` floor, the regrouped scoring and the flat-table BP; the S2 run, 2026-11-27 morning, was `storage` 5.99 / 4.77 / 4.97 s at 2 passes and 8.44 / 4.83 / 4.96 s at 3, `qso3_busy` / `qso1` / `qso2`):**
//! the first search loses the 0.45 s of `band` (1.79 -> 1.35 s) and the rounds lose the BP they ran (host: the BP of two passes 189 -> 68 M instructions on `qso3_busy`, 73 -> 36 on `qso1`, 93 -> 46
//! on `qso2`, at about 10 ns each). `storage` at 2 passes: **4.4 s (4.0-4.8) / 3.9 s (3.6-4.2) / 4.05 s (3.7-4.4)**; at 3 passes **6.8 s (6.3-7.3)** for `qso3_busy` (the others stop after the second
//! pass, as before: 3.9 / 4.05 s); the decoded sets as before (11 / 5 / 5 at 2 passes, 12 at 3 on `qso3_busy`) and equal across the arms. The `2core` arm: as `storage`, less what core 1 takes of the rounds.
//!
//! **The OSD in the SIC (`sic_passes_osd`: every round runs the BP staircase, then the OSD for the best `K` attempts that failed it, and the OSD's decodes are subtracted for
//! the next pass), predicted before its run:** two more arms, `osd6` and `osd8` (K = 6 / 8, seeds `a`, `d`, over the same `SicStorage` and worker as `storage`). Host `icap-sic` on the three
//! recordings (OSD queued / run): 2 passes K = 6 51 / 32 / 41 queued, **12 / 8 / 12 run**, K = 8 **16 / 10 / 15 run**; 3 passes K = 6 16 / 8 / 12 run (`qso3_busy` queues 55), K = 8 20 / 10 / 15; the BP work +1 %.
//! An OSD attempt on the board is about 36 ms (33 ms measured in `ft8-stage-bench`'s `osd ad` arm, plus the attempt's fill and BP pass: the `osd pass` line there, 0.22 s for 6). On the `storage`
//! figures of the S2 run of 2026-11-27 (4.24 / 3.43 / 3.49 s at 2 passes, 6.53 s at 3 for `qso3_busy`): **`osd6`, 2 passes: 4.55-4.80 / 3.65-3.85 / 3.85-4.05 s; `osd8`: 4.75-4.95 / 3.75-3.95 / 4.0-4.2 s;
//! 3 passes, `qso3_busy`: `osd6` 7.0-7.3 s, `osd8` 7.25-7.5 s.** Decodes (host; the board may differ by one, as its BP passes do): 2 passes `osd6` 13 / 5 / 5, `osd8` 14 / 5 / 5 (OSD off 11 / 4 / 5); 3 passes
//! `osd6` 14 / 5 / 5, `osd8` 15 / 5 / 5 (OSD off 12 / 4 / 5). Stop rule: any arm more than 25 % off, or a decode count more than one off the host's, stops the next flash.
//!
//! Logged: the time of the search and of the passes, the decodes (each arm's set against `alloc`'s), what making the memory cost in each heap, the heap minima
//! since boot (**cumulative**: an arm shows only what goes below an earlier arm's, so the order is alloc, storage, ring-int and the minima are floors, not
//! per-arm peaks) and the thread's stack high-water mark. No WiFi, no USB host: this says nothing about the 15.8 kB floor of the running firmware.

extern crate alloc;
extern crate std;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    CELLS_D_MIN_SCORE, CellTables, CellsWorker, Found, Ft8BpScratch, Ft8Rate, GatedSearch, HalfDbCells, SicScratch,
    SicStorage, SigState, coarse_sync_gated_cells, decode_block_cells_sic, sic_passes, sic_passes_osd,
    sic_passes_with, sic_signal_capacity,
};
use mfsk_core::engine::sync::SyncCandidate;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};
use mfsk_core::msg::wsjt77::unpack77;

const INTERNAL: u32 = (1 << 11) | (1 << 2);
const PSRAM: u32 = (1 << 10) | (1 << 2);
const MAX_CAND: usize = 30;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}
fn free(caps: u32) -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_free_size(caps) }
}
fn min_free(caps: u32) -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_minimum_free_size(caps) }
}
fn stack_free() -> usize {
    // ESP-IDF's StackType_t is a byte
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) as usize }
}

fn texts(out: &[DecodeResult]) -> Vec<String> {
    let mut t: Vec<String> = out.iter().filter_map(|d| unpack77(d.message77())).collect();
    t.sort();
    t
}

/// A round for the second core: the shared read-only inputs of one round (valid until the main core has received the reply).
struct Job {
    cells: *const HalfDbCells,
    cands: *const (SyncCandidate, u8),
    n: usize,
    next: *const AtomicUsize,
}
// SAFETY: the sender blocks on the reply before it lets any of the pointees go.
unsafe impl Send for Job {}
type Done = Vec<(usize, Option<Found>)>;

fn claim(w: &mut CellsWorker<'_>, j: &Job) -> Done {
    // SAFETY: see `Job`
    let (cells, next) = unsafe { (&*j.cells, &*j.next) };
    let mut mine = Vec::new();
    loop {
        let i = next.fetch_add(1, Ordering::Relaxed);
        if i >= j.n {
            break;
        }
        let cand = unsafe { &(*j.cands.add(i)).0 };
        mine.push((i, w.decode_candidate(cells, cand, DecodeDepth::EMBEDDED, 0.25)));
    }
    mine
}

fn spawn_pinned(
    name: &'static core::ffi::CStr,
    core: esp_idf_svc::hal::cpu::Core,
    stack: usize,
    f: impl FnOnce() + Send + 'static,
) -> std::thread::JoinHandle<()> {
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    let d = ThreadSpawnConfiguration::default();
    ThreadSpawnConfiguration { name: Some(name), stack_size: stack, pin_to_core: Some(core), ..d }
        .set()
        .expect("sic-bench: thread config");
    let h = std::thread::Builder::new().stack_size(stack).spawn(f).expect("sic-bench: spawn");
    let _ = ThreadSpawnConfiguration::default().set();
    h
}

pub fn run(target: &str, wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    log::info!("sic-bench: {target}, SIC on the board: one core and two (S3 plan, SIC-on-ESP32 steps S2 and B)");
    let h = spawn_pinned(c"sic_bench", esp_idf_svc::hal::cpu::Core::Core0, 48 * 1024, move || body(wavs));
    let _ = h.join();
    log::info!("sic-bench: end");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn body(wavs: &'static [(&'static str, &'static [u8])]) {
    log::info!(
        "sic-bench: boot free internal {} B, PSRAM {} B; stack free {} B",
        free(INTERNAL),
        free(PSRAM),
        stack_free()
    );
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let rate = Ft8Rate::Hz8000;
    let tables = CellTables::new();
    // the worker: the firmware's internal-DRAM staging, a BP scratch made once
    // SAFETY: this thread is the only user of the main staging buffer here.
    let cs = unsafe { crate::internal_pool::cs_scratch_main() };
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(cs, &mut bp, &tables)
        .with_d_min_score(CELLS_D_MIN_SCORE)
        .with_osd_seeds(mfsk_core::ft8::decode_block::OsdSeeds::AD);
    log::info!(
        "sic-bench: worker made; free internal {} B, PSRAM {} B",
        free(INTERNAL),
        free(PSRAM)
    );

    let (i0, p0) = (free(INTERNAL), free(PSRAM));
    let mut storage = SicStorage::new(rate, sic_signal_capacity(MAX_CAND, 3));
    log::info!(
        "sic-bench: SicStorage (ring 32 768 B, rd/raw/gidx/states) cost internal {} B, PSRAM {} B",
        i0 as i64 - free(INTERNAL) as i64,
        p0 as i64 - free(PSRAM) as i64
    );
    // one internal block for the whole scratch: ring 32 768 + rd 2 560 + raw 1 280 + gidx 1 280 = 37 888 B
    const BLOCK: usize = 32_768 + 2_560 + 1_280 + 1_280;
    let i1 = free(INTERNAL);
    let blk = unsafe { esp_idf_svc::sys::heap_caps_aligned_alloc(16, BLOCK, INTERNAL) } as *mut u8;
    let have_int = !blk.is_null();
    log::info!(
        "sic-bench: internal scratch block {} B: {} (internal free {} -> {})",
        BLOCK,
        if have_int { "taken" } else { "NOT available" },
        i1,
        free(INTERNAL)
    );
    let mut st = vec![SigState::default(); sic_signal_capacity(MAX_CAND, 3)];

    // the second core: its own staging (the worker half of the arena) and BP scratch, fed rounds through a channel
    let (job_tx, job_rx): (Sender<Job>, Receiver<Job>) = channel();
    let (done_tx, done_rx): (Sender<Done>, Receiver<Done>) = channel();
    let tables_addr = &tables as *const CellTables as usize;
    let _w1 = spawn_pinned(c"sic_w1", esp_idf_svc::hal::cpu::Core::Core1, 24 * 1024, move || {
        // SAFETY: only this thread touches the worker half; `tables` outlives it (the bench never returns)
        let cs = unsafe { crate::internal_pool::cs_scratch_worker() };
        let mut bp = Box::new(Ft8BpScratch::new());
        let tables = unsafe { &*(tables_addr as *const CellTables) };
        let mut w = CellsWorker::new(cs, &mut bp, tables).with_d_min_score(CELLS_D_MIN_SCORE);
        log::info!("sic-bench: sic_w1 up on core {}, stack free {} B", unsafe { esp_idf_svc::sys::xTaskGetCoreID(core::ptr::null_mut()) }, stack_free());
        while let Ok(j) = job_rx.recv() {
            let _ = done_tx.send(claim(&mut w, &j));
        }
    });

    for (label, wav) in wavs {
        let slot12: Vec<i16> = wav[44..]
            .chunks_exact(2)
            .take(180_000)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        let a8 = resample_to_8k(&slot12, 12_000);
        for passes in [2usize, 3] {
            // alloc
            let mut want: Vec<String> = Vec::new();
            let mut t_alloc = 0i64;
            for k in 0..2 {
                let t0 = now_us();
                let out = decode_block_cells_sic(&a8, rate, &search, MAX_CAND, DecodeDepth::EMBEDDED, passes).unwrap();
                t_alloc = now_us() - t0;
                if k == 1 {
                    want = texts(&out);
                }
            }
            log::info!(
                "sic-bench: {label} passes {passes} alloc    : {t_alloc} us, {} decodes; min free internal {} B, PSRAM {} B",
                want.len(),
                min_free(INTERNAL),
                min_free(PSRAM)
            );
            // storage (ring in PSRAM) and ring-int
            for arm in ["storage ", "osd6    ", "osd8    ", "ring-int", "2core   "] {
                if arm == "ring-int" && !have_int {
                    continue;
                }
                let (mut t_search, mut t_pass) = (0i64, 0i64);
                let mut got: Vec<String> = Vec::new();
                for k in 0..2 {
                    let t0 = now_us();
                    let (cands, mut cells) = coarse_sync_gated_cells(&mut SliceSource(&a8), rate, &search, 1).unwrap();
                    let t1 = now_us();
                    let out = if arm == "2core   " {
                        sic_passes_with(&mut SliceSource(&a8), rate, &search, MAX_CAND, passes, &cands, &mut cells, &tables, &mut storage.scratch(),
                            |cells, cands, max_cand, known| {
                                let n = cands.len().min(max_cand);
                                let next = AtomicUsize::new(0);
                                let j = |next: &AtomicUsize| Job { cells, cands: cands.as_ptr(), n, next };
                                job_tx.send(j(&next)).unwrap();
                                let mut all = claim(&mut worker, &j(&next));
                                all.extend(done_rx.recv().unwrap());
                                all.sort_by_key(|x| x.0);
                                for (_, f) in all {
                                    if let Some(f) = f {
                                        if !known.iter().any(|o| o.res.message77() == f.res.message77()) {
                                            known.push(f);
                                        }
                                    }
                                }
                            })
                    } else if arm == "osd6    " || arm == "osd8    " {
                        let depth = DecodeDepth { osd: true, ..DecodeDepth::EMBEDDED };
                        sic_passes_osd(&mut SliceSource(&a8), rate, &search, MAX_CAND, depth, passes, &cands, &mut cells, &tables, &mut worker, &mut storage.scratch(), if arm == "osd6    " { 6 } else { 8 })
                    } else if arm == "storage " {
                        sic_passes(&mut SliceSource(&a8), rate, &search, MAX_CAND, DecodeDepth::EMBEDDED, passes, &cands, &mut cells, &tables, &mut worker, &mut storage.scratch())
                    } else {
                        // SAFETY: `blk` is BLOCK bytes, 16-aligned, used only through these disjoint slices for the life of this call
                        let (ring, rest) = unsafe { (core::slice::from_raw_parts_mut(blk as *mut f32, 8192), blk.add(32_768)) };
                        let (rd, rest) = unsafe { (core::slice::from_raw_parts_mut(rest as *mut f32, 640), rest.add(2_560)) };
                        let (raw, rest) = unsafe { (core::slice::from_raw_parts_mut(rest as *mut i16, 640), rest.add(1_280)) };
                        let gidx = unsafe { core::slice::from_raw_parts_mut(rest, 1_280) };
                        let mut scratch = SicScratch::new(rate, ring, rd, raw, gidx, &mut st);
                        sic_passes(&mut SliceSource(&a8), rate, &search, MAX_CAND, DecodeDepth::EMBEDDED, passes, &cands, &mut cells, &tables, &mut worker, &mut scratch)
                    };
                    let t2 = now_us();
                    t_search = t1 - t0;
                    t_pass = t2 - t1;
                    if k == 1 {
                        got = texts(&out);
                    }
                }
                log::info!(
                    "sic-bench: {label} passes {passes} {arm}: {} us (search {t_search} + passes {t_pass}), {} decodes, set {} alloc's; min free internal {} B, PSRAM {} B, stack free {} B",
                    t_search + t_pass,
                    got.len(),
                    if arm.starts_with("osd") { "(OSD on, not compared to)" } else if got == want { "=" } else { "DIFFERS FROM" },
                    min_free(INTERNAL),
                    min_free(PSRAM),
                    stack_free()
                );
            }
        }
    }
}
