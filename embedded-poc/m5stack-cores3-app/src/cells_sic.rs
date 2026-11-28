// SPDX-License-Identifier: GPL-3.0-only
//! The **late path** of the cells flow (`docs/notes/PROFILES.md` section 10): after the window path has published a slot, [`CellsOptions::p0_late`] (the same cells, **two subtraction passes**, the OSD
//! capped at 6 in every round) runs over a copy of the slot's cells and its audio on core 1, and what it decodes that the window path did not goes to the heard queue (the `Activator` hears it at the
//! next reply deadline), the station list (`UiState::publish_late`: rows of the slot already published, the watermark unmoved), `ALL.TXT` and the log.
//!
//! **Memory**: a second store (172 KB) and the slot's audio (240 KB) are handed over with the job and handed back when it is done ([`LateDone`]), so a slot costs no allocation after the first; the
//! subtraction's own memory ([`SicStorage`], 32 KB ring and the signals' states) is made once. A slot whose predecessor's late path is still running is not given one ([`LATE_BUSY`]): the window path
//! never waits for it. Task: core 1, priority 4 (below `uac_reader` 8 and the decode task 6), a 48 KB stack.
//!
//! **What is not known yet**: what this does to the window path on core 0 (the esp-dsp FFT scratch is behind a spin lock both use: `docs/notes/ESP32S3_TWO_CORE_BEHAVIOUR.md`); read the `longest job`
//! of the `CELLS slot` line with the late path running against without it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Mutex;

use embedded_shared::internal_pool;
use mfsk_app_shared::heard_queue::pack_bits;
use mfsk_app_shared::ui::state::{SlotDecode, UI};
use mfsk_core::engine::source::SliceSource;
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::ft8::decode_block::{
    sic_passes_osd, sic_signal_capacity, ApContext, CellTables, CellsOptions, CellsWorker, Found,
    Ft8BpScratch, Ft8Rate, GatedSearch, HalfDbCells, SicStorage,
};
use mfsk_core::msg::CallsignHashTable;

/// A late job is running (set by the window path when it hands one over, cleared by the task when it is done).
pub static LATE_BUSY: AtomicBool = AtomicBool::new(false);

/// The callsign hash table both paths resolve and learn through (a message decoded by one teaches the other).
static CALLS: Mutex<Option<CallsignHashTable>> = Mutex::new(None);

/// The rendered text of a decode's 77 bits, learning its callsigns.
pub fn render(bits: &[u8; 77]) -> Option<String> {
    let mut g = CALLS.lock().ok()?;
    let table = g.get_or_insert_with(CallsignHashTable::new);
    mfsk_core::msg::wsjt77::unpack77_learn(bits, table)
}

/// One slot's late work.
pub struct LateJob {
    pub audio: Vec<i16>,
    pub cells: HalfDbCells,
    /// The window path's candidates (the first pass of the subtraction decodes these).
    pub cands: Vec<(SyncCandidate, u8)>,
    /// The 77 bits (packed) of what the window path decoded: not reported again.
    pub known: Vec<u128>,
    pub slot_n: u32,
    pub period: u32,
    pub slot_unix: Option<i64>,
}

/// The memory handed back.
pub type LateDone = (Vec<i16>, HalfDbCells);

/// What core 1 is given: a slot's late work, or the partner's AP decode of the slot being received.
pub enum CoreOneJob {
    Late(LateJob),
    Ap(ApJob),
}

/// The partner's AP decode, started when the rows of the partner's place have arrived (before the slot ends): `cells` is a copy of the store so far, `audio` only travels with it (the pair is
/// handed back together).
pub struct ApJob {
    pub audio: Vec<i16>,
    pub cells: HalfDbCells,
    pub ctx: ApContext,
    /// The number the slot being received will have at its end.
    pub slot_n: u32,
}

/// The partner decode's result: what the window path folds in at the end of the slot (it has the complete cells for the SNR).
pub struct ApDone {
    pub slot_n: u32,
    pub found: Vec<Found>,
    pub us: i64,
}

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Start the late task: jobs in on `rx`, the memory back on `back`.
pub fn spawn(rx: Receiver<CoreOneJob>, back: Sender<LateDone>, ap: Sender<ApDone>) {
    let spawned = crate::board::spawn_named_tuned(
        c"sic",
        48 * 1024,
        Some(4),
        Some(esp_idf_svc::hal::cpu::Core::Core1),
        move || body(rx, back, ap),
    );
    if let Err(e) = spawned {
        log::error!("late path: the task did not start ({e})");
    }
}

fn body(rx: Receiver<CoreOneJob>, back: Sender<LateDone>, ap: Sender<ApDone>) {
    let options = CellsOptions::p0_late();
    let passes = options.sic.map_or(1, |s| s.passes);
    let osd_k = match options.osd {
        mfsk_core::ft8::decode_block::OsdOptions::Capped { k, .. } => k,
        _ => 0,
    };
    let rate = Ft8Rate::Hz8000;
    let search = GatedSearch::new(200.0, 3000.0, 1.3, options.max_cand);
    let depth = options.depth();
    let tables = CellTables::new();
    // SAFETY: the window path uses the main half of the arena; this task is the only user of the worker half.
    let cs = unsafe { internal_pool::cs_scratch_worker() };
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(cs, &mut bp, &tables).with_options(&options);
    worker.reserve_osd_work();
    let mut storage = SicStorage::new(rate, sic_signal_capacity(options.max_cand, passes));
    log::info!("late path ready: {passes} passes, OSD capped at {osd_k} a round, core 1");
    crate::log_free_internal("late-path-ready");

    while let Ok(msg) = rx.recv() {
        let mut job = match msg {
            CoreOneJob::Late(j) => j,
            CoreOneJob::Ap(j) => {
                let t0 = now_us();
                worker.set_ap_context(Some(j.ctx));
                let mut found: Vec<Found> = Vec::new();
                let _ = worker.take_bp_stats();
                let n = worker.decode_partner(&j.cells, depth, 0.25, &mut found);
                worker.set_ap_context(None);
                let st = worker.take_bp_stats();
                let us = now_us() - t0;
                log::info!(
                    "AP slot {}: partner {:4.0} Hz {:+.2} s, {n} decoded, {} ms on core 1 ({} BP calls, {} passes)",
                    j.slot_n,
                    j.ctx.freq_hz(),
                    j.ctx.dt_sec(),
                    us / 1000,
                    st.calls,
                    st.passes
                );
                let _ = ap.send(ApDone { slot_n: j.slot_n, found, us });
                let _ = back.send((j.audio, j.cells));
                LATE_BUSY.store(false, Ordering::Release);
                continue;
            }
        };
        let t0 = now_us();
        let res = sic_passes_osd(
            &mut SliceSource(&job.audio[..]),
            rate,
            &search,
            options.max_cand,
            depth,
            passes,
            &job.cands,
            &mut job.cells,
            &tables,
            &mut worker,
            &mut storage.scratch(),
            osd_k,
        );
        let t_sic = now_us();
        let mut rows: Vec<(String, f32, f32, f32, u32)> = Vec::new();
        for r in &res {
            let bits = r.message77();
            if job.known.contains(&pack_bits(&bits[..])) {
                continue;
            }
            if let Some(text) = render(&bits) {
                log::info!(
                    "LATE {:+5.1}dB {:+5.2}s {:4.0}Hz {}",
                    r.snr_db,
                    r.dt_sec,
                    r.freq_hz,
                    text
                );
                crate::qso_activator::push_heard(&text, &bits[..], r.snr_db, job.period);
                rows.push((text, r.freq_hz, r.snr_db, r.dt_sec, r.hard_errors));
            }
        }
        if !rows.is_empty() {
            if let Ok(mut ui) = UI.lock() {
                let decodes: Vec<SlotDecode> = rows
                    .iter()
                    .map(|(text, freq_hz, snr_db, dt_sec, hard)| SlotDecode {
                        freq_hz: *freq_hz,
                        snr_db: *snr_db,
                        dt_sec: *dt_sec,
                        text,
                        hard_errors: *hard,
                    })
                    .collect();
                crate::storage::publish_late(
                    &mut ui,
                    mfsk_app_shared::boot_mode::BootMode::Uac,
                    job.slot_unix,
                    &decodes,
                );
            }
        }
        log::info!(
            "LATE slot {}: {} decodes in all, {} new, SIC {} ms, then {} ms; min free internal after",
            job.slot_n,
            res.len(),
            rows.len(),
            (t_sic - t0) / 1000,
            (now_us() - t_sic) / 1000
        );
        crate::log_free_internal("late-slot-end");
        let _ = back.send((job.audio, job.cells));
        LATE_BUSY.store(false, Ordering::Release);
    }
}
