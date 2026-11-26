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
    CellTables, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch, SicScratch, SicStorage, SigState,
    coarse_sync_gated_cells, decode_block_cells_sic, sic_passes, sic_signal_capacity,
};
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

pub fn run(target: &str, wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    log::info!("sic-bench: {target}, SIC on the board, one core (S3 plan, SIC-on-ESP32 step S2)");
    let h = std::thread::Builder::new()
        .name("sic_bench".into())
        .stack_size(48 * 1024)
        .spawn(move || body(wavs))
        .expect("sic-bench: could not spawn the bench thread");
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
    let mut worker = CellsWorker::new(cs, &mut bp, &tables);
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
            for arm in ["storage ", "ring-int"] {
                if arm == "ring-int" && !have_int {
                    continue;
                }
                let (mut t_search, mut t_pass) = (0i64, 0i64);
                let mut got: Vec<String> = Vec::new();
                for k in 0..2 {
                    let t0 = now_us();
                    let (cands, mut cells) = coarse_sync_gated_cells(&mut SliceSource(&a8), rate, &search, 1).unwrap();
                    let t1 = now_us();
                    let out = if arm == "storage " {
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
                    if got == want { "=" } else { "DIFFERS FROM" },
                    min_free(INTERNAL),
                    min_free(PSRAM),
                    stack_free()
                );
            }
        }
    }
}
