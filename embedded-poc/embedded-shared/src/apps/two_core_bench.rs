//! The cells flow's stage 3 and the OSD pass on **two cores** of the CoreS3, against one core: the round's candidates dealt out between two workers (core 0: this thread, core 1:
//! `tc_w1`, each with its own internal-DRAM staging block and BP scratch), the attempts that fail the BP staircase queued by their worker, gathered, ranked
//! (`rank_osd_attempts`) and the best `K` dealt out between the same two workers; and the band search cut over two halves of the carriers (`search_cells` and `merge_cells_parts`,
//! the list of which is the one search's for any cut, host test). Wav replay from PSRAM, no WiFi, no USB host; each measurement made twice, the second is read.
//!
//! Per recording, 30 candidates, OSD seeds `a`, `d`, the variant `d` floor 16, K = 6 and 14: `band` (1 / 2 cores), and for each K the `round` (BP staircase over the 30, the OSD queued), the
//! `osd` pass (rank and run the best K) and their sum, on one core and on two; and whether the two-core set of decodes equals the one-core set.
//!
//! **Predictions, written before the first run.** One core (`ft8-stage-bench`'s `osd ad-cap6` arm and the `osd pass` line): stage 3 with the pass at K = 6 0.808 / 0.705 / 0.742 s, of which
//! the pass 0.216 / 0.252 / 0.268 s (36 ms an attempt at 6 attempts), so the round 0.59 / 0.45 / 0.47 s; the pass at K = 14 is 14 x 36 = 0.50 s; `band` 0.46 s (`ft8-stage-bench`).
//! How two cores scale: the SIC rounds of `ft8-sic-bench`'s `2core` arm shortened the passes by 0.26 / 0.16 / 0.22 s (-9 / -8 / -10 %) against a BP of the rounds of 0.68 / 0.36 / 0.46 s
//! on the host's instructions at 10 ns, i.e. the rounds ran **1.6-1.9 times** as fast (the search and the subtraction of a pass stay on one core). That ratio, applied here:
//! **`round` on two cores 0.31-0.37 / 0.24-0.28 / 0.25-0.30 s; `osd` pass K = 6 0.11-0.13 / 0.13-0.15 / 0.13-0.16 s (the attempts are all about equal: dealt in turn they balance), K = 14
//! 0.25-0.30 s; the sum at K = 6 0.42-0.50 / 0.37-0.43 / 0.38-0.46 s.** `band` on two halves: 0.25-0.30 s (the half of 0.46 s and the merge). Decodes as on one core (qso3_busy 11, `qso1` 5,
//! `qso2` 5 at K = 6; one more or fewer at K = 14: 12 / 5 / 5), the two sets equal. Allowing for the two cores' PSRAM and FFT-scratch contention is the point of the run: if `round` or the pass
//! is more than 25 % off, stop and find which (the host cannot tell). Window arithmetic if it holds (emit 0.81 s before the boundary): `band` on two cores ends at -0.54 s, the round at -0.2 s and
//! the pass at K = 6 at -0.07 s; at K = 14 at +0.05 s.
//!
//! **Measured (the first run, 2026-11-27) against the above:** `band` x1.10 (2 cores 0.416 / 0.418 / 0.406 s), the round x1.5 (0.383 / 0.297 / 0.308 s), the pass x1.5 (K = 6 0.148 / 0.168 / 0.161 s, K = 14
//! 0.363 / 0.388 / 0.385 s against 0.537 / 0.597 / 0.572 s on one core), sets equal. An OSD attempt: 38 ms on one core, 52 ms with both running. Hypothesis: the two cores share the PSRAM, where the OSD
//! fallback's memory is (the 4.5 KB `Osd174Scratch` and the f32 BP scratch's buffers above 2 KB: `tov` 2 088 B, `toc` 2 324 B, `tanhtoc`, the index tables).
//!
//! **The per-core times, predicted before their run (the placement was not the cause: section 30; the run logs, for the second two-core run of each K, each core's busy time in the round and its
//! share of the OSD pass with the time of every attempt in it, `per core` lines).** Two readings of the 52 ms an attempt takes on two cores against 38 ms on one. **(imbalance)** each attempt costs what it
//! does on one core, 36-42 ms on average on both cores, but the attempts differ (a decoding attempt, a stalling BP...) and the pass waits for the slower share: the two shares at K = 14 differ by 0.05 s or more
//! (say 0.30 and 0.35 s), the mean attempt on each core is 36-42 ms, `max / min` over the attempts of a share is above 1.5. **(slowdown)** every attempt is slower while the other core runs: the two shares are
//! within 0.03 s of each other (about 0.34-0.36 s), the mean attempt is 46-52 ms on **both** cores, the spread is small (`max / min` under 1.5). The round (atomic index, dealt as claimed) shows the same: its
//! two busy times within 0.02 s of each other, the candidates split about 15 / 15 (imbalance would not be visible in it, the slowdown would: the same x1.5). A mixture of the two is likely; the sizes of the parts
//! are what the run gives. Not a pass / fail test: the stop rule does not apply, the numbers are.
//!
//! **The test of it, predicted before its run:** the same bench twice in one boot, `[psram]` (the OSD memory made by `reserve_osd_work` under the 2 KB malloc limit: where the first run had it) and
//! `[internal]` (made while `heap_caps_malloc_extmem_enable(16 KB)`: in internal DRAM; the limit is put back to 2 KB after). The round, `band` and the staging are the same in both. **If the hypothesis is right:**
//! `[internal]` takes about 17 KB x 2 of internal DRAM, the OSD pass on **two cores at K = 14 falls to 0.27-0.31 / 0.30-0.34 / 0.30-0.34 s** (an attempt at about 40 ms: x1.8-2.0 against one core) and K = 6 to 0.11-0.13 s,
//! and the one-core pass (`[internal]`) stays within 10 % of `[psram]`'s or gets faster (0.54 / 0.60 / 0.57 s at K = 14; an attempt's own PSRAM reads are the cost it would lose). **If it is wrong** the two arms agree within 5 %
//! and the 52 ms is something else (the cores' shared cache or the FFT scratch lock). The round and `band` do not move in either case (their memory is not touched by this). The decode sets stay equal. Stop rule as above.

extern crate alloc;
extern crate std;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};

use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    CELLS_D_MIN_SCORE, CellTables, CellsWorker, Found, Ft8BpScratch, Ft8Rate, GatedSearch, HalfDbCells, OsdAttempt, OsdSeeds,
    coarse_sync_gated_cells, merge_cells_parts, rank_osd_attempts, search_cells,
};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::msg::wsjt77::unpack77;

const INTERNAL: u32 = (1 << 11) | (1 << 2);
const MAX_CAND: usize = 30;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}
fn free(caps: u32) -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_free_size(caps) }
}
const PSRAM: u32 = (1 << 10) | (1 << 2);
fn min_free_internal() -> usize {
    unsafe { esp_idf_svc::sys::heap_caps_get_minimum_free_size(INTERNAL) }
}
fn stack_free() -> usize {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) as usize }
}

fn texts(known: &[Found]) -> Vec<String> {
    let mut t: Vec<String> = known.iter().filter_map(|f| unpack77(f.res.message77())).collect();
    t.sort();
    t
}

/// Pointers to the read-only inputs of one request to the second core; valid until the main core has the reply (it blocks on it).
struct Ptr<T>(*const T);
unsafe impl<T> Send for Ptr<T> {}

enum Msg {
    Round { cells: Ptr<HalfDbCells>, cands: Ptr<(SyncCandidate, u8)>, n: usize, next: Ptr<AtomicUsize>, depth: DecodeDepth },
    Osd { cells: Ptr<HalfDbCells>, share: Vec<(usize, OsdAttempt)>, depth: DecodeDepth },
    Band { cells: Ptr<HalfDbCells>, search: Ptr<GatedSearch>, lo: usize, hi: usize },
    /// make the OSD fallback's memory now (`CellsWorker::reserve_osd_work`)
    Reserve,
}
enum Reply {
    /// the candidates' decodes, the attempts it queued, the microseconds it was busy and how many candidates it took
    Round(Vec<(usize, Option<Found>)>, Vec<OsdAttempt>, i64),
    /// the attempts' results and the microseconds each took
    Osd(Vec<(usize, Option<Found>)>, Vec<i64>),
    Band(mfsk_core::ft8::decode_block::CellsPart),
    Done,
}

fn claim(w: &mut CellsWorker<'_>, cells: &HalfDbCells, cands: *const (SyncCandidate, u8), n: usize, next: &AtomicUsize, depth: DecodeDepth) -> Vec<(usize, Option<Found>)> {
    let mut mine = Vec::new();
    loop {
        let i = next.fetch_add(1, Ordering::Relaxed);
        if i >= n {
            break;
        }
        // SAFETY: `i < n` and the pointee lives until the reply is received
        let (cand, hs) = unsafe { &*cands.add(i) };
        mine.push((i, w.decode_candidate_hs(cells, cand, *hs, depth, 0.25)));
    }
    mine
}

fn spawn_pinned(name: &'static core::ffi::CStr, core: esp_idf_svc::hal::cpu::Core, stack: usize, f: impl FnOnce() + Send + 'static) -> std::thread::JoinHandle<()> {
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    let d = ThreadSpawnConfiguration::default();
    ThreadSpawnConfiguration { name: Some(name), stack_size: stack, pin_to_core: Some(core), ..d }
        .set()
        .expect("two-core-bench: thread config");
    let h = std::thread::Builder::new().stack_size(stack).spawn(f).expect("two-core-bench: spawn");
    let _ = ThreadSpawnConfiguration::default().set();
    h
}

pub fn run(target: &str, wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    log::info!("two-core-bench: {target}, stage 3, the OSD pass and the band search on one core and on two (predictions in the module doc)");
    let h = spawn_pinned(c"tc_bench", esp_idf_svc::hal::cpu::Core::Core0, 48 * 1024, move || body(wavs));
    let _ = h.join();
    log::info!("two-core-bench: end");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn worker<'a>(cs: &'a mut [[mfsk_core::engine::scalar::Cmplx<f32>; 8]; 79], bp: &'a mut Ft8BpScratch, tables: &'a CellTables) -> CellsWorker<'a> {
    CellsWorker::new(cs, bp, tables)
        .with_d_min_score(CELLS_D_MIN_SCORE)
        .with_osd_seeds(OsdSeeds::AD)
        .with_osd_deferred(true)
}

fn body(wavs: &'static [(&'static str, &'static [u8])]) {
    log::info!("two-core-bench: boot min free internal {} B; stack free {} B", min_free_internal(), stack_free());
    // the placement test is done (section 30): the OSD memory stays where it was (PSRAM); this run is for the per-core times
    for internal in [false] {
        arm(internal, wavs);
    }
    log::info!("two-core-bench: done");
}

/// Everything above, with the OSD fallback's memory (about 17 KB a worker: the `Osd174Scratch` and the f32 BP scratch of its inputs, whose buffers are above the 2 KB
/// `SPIRAM_MALLOC_ALWAYSINTERNAL` limit) made by `reserve_osd_work` while the malloc limit is `internal ? 16 KB : 2 KB`: in internal DRAM or in PSRAM. The two arms run in turn in one boot.
fn arm(internal: bool, wavs: &'static [(&'static str, &'static [u8])]) {
    let place = if internal { "internal" } else { "psram" };
    let search = GatedSearch::new(200.0, 3000.0, 1.3, MAX_CAND);
    let rate = Ft8Rate::Hz8000;
    let tables = CellTables::new();
    let depth = DecodeDepth { osd: true, ..DecodeDepth::EMBEDDED };
    // SAFETY: this thread is the only user of the main staging buffer here.
    let cs0 = unsafe { crate::internal_pool::cs_scratch_main() };
    let mut bp0 = Box::new(Ft8BpScratch::new());
    let mut w0 = worker(cs0, &mut bp0, &tables);

    let (tx, rx): (Sender<Msg>, Receiver<Msg>) = channel();
    let (done_tx, done_rx): (Sender<Reply>, Receiver<Reply>) = channel();
    let tables_addr = &tables as *const CellTables as usize;
    let w1h = spawn_pinned(c"tc_w1", esp_idf_svc::hal::cpu::Core::Core1, 24 * 1024, move || {
        // SAFETY: only this thread touches the worker half of the arena; `tables` outlives it (the bench never returns)
        let cs1 = unsafe { crate::internal_pool::cs_scratch_worker() };
        let mut bp1 = Box::new(Ft8BpScratch::new());
        let tables = unsafe { &*(tables_addr as *const CellTables) };
        let mut w1 = worker(cs1, &mut bp1, tables);
        while let Ok(m) = rx.recv() {
            let r = match m {
                Msg::Round { cells, cands, n, next, depth } => {
                    // SAFETY: the sender blocks on the reply
                    let (cells, next) = unsafe { (&*cells.0, &*next.0) };
                    let t0 = now_us();
                    let found = claim(&mut w1, cells, cands.0, n, next, depth);
                    let busy = now_us() - t0;
                    Reply::Round(found, w1.take_osd_attempts(), busy)
                }
                Msg::Osd { cells, share, depth } => {
                    let cells = unsafe { &*cells.0 };
                    let mut each = Vec::new();
                    let res = share
                        .iter()
                        .map(|(r, a)| {
                            let t0 = now_us();
                            let f = w1.run_osd_attempt(cells, a, depth);
                            each.push(now_us() - t0);
                            (*r, f)
                        })
                        .collect();
                    Reply::Osd(res, each)
                }
                Msg::Reserve => {
                    w1.reserve_osd_work();
                    Reply::Done
                }
                Msg::Band { cells, search, lo, hi } => {
                    let (cells, search) = unsafe { (&*cells.0, &*search.0) };
                    Reply::Band(search_cells(cells, search, lo..hi))
                }
            };
            if done_tx.send(r).is_err() {
                break;
            }
        }
    });
    log::info!("two-core-bench: [{place}] workers made; min free internal {} B", min_free_internal());
    // the OSD fallback's memory: made now, in the heap the malloc limit picks (the limit is restored after; both workers are made inside it)
    let (i0, p0) = (free(INTERNAL), free(PSRAM));
    if internal {
        unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(16 * 1024) };
    }
    w0.reserve_osd_work();
    tx.send(Msg::Reserve).unwrap();
    let Reply::Done = done_rx.recv().unwrap() else { panic!("reply") };
    if internal {
        unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(2048) };
    }
    log::info!(
        "two-core-bench: [{place}] the OSD memory of both workers took {} B of internal DRAM and {} B of PSRAM; internal free {} B",
        i0 as i64 - free(INTERNAL) as i64,
        p0 as i64 - free(PSRAM) as i64,
        free(INTERNAL)
    );

    for (label, wav) in wavs {
        let slot12: Vec<i16> = wav[44..].chunks_exact(2).take(180_000).map(|b| i16::from_le_bytes([b[0], b[1]])).collect();
        let a8 = resample_to_8k(&slot12, 12_000);
        let (cands, cells) = coarse_sync_gated_cells(&mut SliceSource(&a8), rate, &search, 1).unwrap();

        // band: one core, then two halves of the carriers
        let (mut t1, mut t2) = (0i64, 0i64);
        let mut same = false;
        for _ in 0..2 {
            let t0 = now_us();
            let whole = merge_cells_parts(&[search_cells(&cells, &search, cells.carrier_bins())], &search, rate);
            t1 = now_us() - t0;
            let t0 = now_us();
            let r = cells.carrier_bins();
            let mid = r.start + (r.end - r.start) / 2;
            tx.send(Msg::Band { cells: Ptr(&cells), search: Ptr(&search), lo: mid, hi: r.end }).unwrap();
            let lo = search_cells(&cells, &search, r.start..mid);
            let Reply::Band(hi) = done_rx.recv().unwrap() else { panic!("reply") };
            let split = merge_cells_parts(&[lo, hi], &search, rate);
            t2 = now_us() - t0;
            same = split.len() == whole.len() && split.iter().zip(&whole).all(|(a, b)| a.0.freq_hz.to_bits() == b.0.freq_hz.to_bits() && a.0.dt_sec.to_bits() == b.0.dt_sec.to_bits() && a.1 == b.1);
        }
        log::info!("two-core-bench: [{place}] {label} band: 1 core {t1} us, 2 cores {t2} us, the candidate lists {}", if same { "equal" } else { "DIFFER" });

        for k in [6usize, 14] {
            // one core
            let (mut r1, mut o1) = (0i64, 0i64);
            let (mut set1, mut queued, mut ran) = (Vec::new(), 0u64, 0u64);
            for _ in 0..2 {
                let _ = w0.take_osd_counts();
                let mut known: Vec<Found> = Vec::new();
                let t0 = now_us();
                w0.decode_round(&cells, &cands, MAX_CAND, depth, 0.25, &mut known);
                r1 = now_us() - t0;
                let t1 = now_us();
                w0.osd_pass(&cells, k, depth, &mut known);
                o1 = now_us() - t1;
                (queued, ran) = w0.take_osd_counts();
                set1 = texts(&known);
            }
            log::info!(
                "two-core-bench: [{place}] {label} K {k} 1 core: round {r1} us + osd {o1} us = {} us, {} decodes, queued {queued} run {ran}",
                r1 + o1,
                set1.len()
            );
            // two cores
            let (mut r2, mut o2) = (0i64, 0i64);
            let mut set2 = Vec::new();
            let mut round_split = (0i64, 0usize, 0i64, 0usize);
            let (mut each0, mut each1): (Vec<i64>, Vec<i64>) = (Vec::new(), Vec::new());
            let (mut share0, mut share1) = (0i64, 0i64);
            for _ in 0..2 {
                let t0 = now_us();
                let next = AtomicUsize::new(0);
                let n = cands.len().min(MAX_CAND);
                tx.send(Msg::Round { cells: Ptr(&cells), cands: Ptr(cands.as_ptr()), n, next: Ptr(&next), depth }).unwrap();
                let tb = now_us();
                let mut all = claim(&mut w0, &cells, cands.as_ptr(), n, &next, depth);
                let busy0 = now_us() - tb;
                let n0 = all.len();
                let mut att = w0.take_osd_attempts();
                let Reply::Round(theirs, their_att, busy1) = done_rx.recv().unwrap() else { panic!("reply") };
                let n1 = theirs.len();
                round_split = (busy0, n0, busy1, n1);
                all.extend(theirs);
                att.extend(their_att);
                all.sort_by_key(|x| x.0);
                let mut known: Vec<Found> = Vec::new();
                for (_, f) in all {
                    if let Some(f) = f {
                        if !known.iter().any(|o| o.res.message77() == f.res.message77()) {
                            known.push(f);
                        }
                    }
                }
                r2 = now_us() - t0;
                let t1 = now_us();
                rank_osd_attempts(&mut att, k);
                let (mut mine, mut share) = (Vec::new(), Vec::new());
                for (r, a) in att.into_iter().enumerate() {
                    if r % 2 == 0 {
                        mine.push((r, a));
                    } else {
                        share.push((r, a));
                    }
                }
                tx.send(Msg::Osd { cells: Ptr(&cells), share, depth }).unwrap();
                let ts = now_us();
                each0.clear();
                let mut ran: Vec<(usize, Option<Found>)> = mine
                    .iter()
                    .map(|(r, a)| {
                        let t0 = now_us();
                        let f = w0.run_osd_attempt(&cells, a, depth);
                        each0.push(now_us() - t0);
                        (*r, f)
                    })
                    .collect();
                share0 = now_us() - ts;
                let Reply::Osd(theirs, e1) = done_rx.recv().unwrap() else { panic!("reply") };
                each1 = e1;
                share1 = each1.iter().sum();
                ran.extend(theirs);
                ran.sort_by_key(|x| x.0);
                for (_, f) in ran {
                    if let Some(f) = f {
                        if !known.iter().any(|o| o.res.message77() == f.res.message77()) {
                            known.push(f);
                        }
                    }
                }
                o2 = now_us() - t1;
                set2 = texts(&known);
            }
            let ms = |v: &[i64]| -> String { v.iter().map(|x| alloc::format!("{}", x / 1000)).collect::<Vec<_>>().join(" ") };
            log::info!(
                "two-core-bench: [{place}] {label} K {k} per core: round main {} us ({} cands) w1 {} us ({} cands); osd main {share0} us [{} ms] w1 {share1} us [{} ms] (wall {o2} us)",
                round_split.0, round_split.1, round_split.2, round_split.3, ms(&each0), ms(&each1)
            );
            log::info!(
                "two-core-bench: [{place}] {label} K {k} 2 cores: round {r2} us + osd {o2} us = {} us, {} decodes, set {} the one core's; round x{:.2}, osd x{:.2}; min free internal {} B, stack free {} B",
                r2 + o2,
                set2.len(),
                if set1 == set2 { "=" } else { "DIFFERS FROM" },
                r1 as f32 / r2 as f32,
                o1 as f32 / o2.max(1) as f32,
                min_free_internal(),
                stack_free()
            );
        }
    }
    drop(tx);
    let _ = w1h.join();
}
