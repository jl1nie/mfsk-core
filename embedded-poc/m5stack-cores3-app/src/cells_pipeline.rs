// SPDX-License-Identifier: GPL-3.0-only
//! The FT8 receive path on the **cells flow** (`docs/notes/PROFILES.md` section 10, the P0 window path): the chunk queue's 12 kHz audio becomes 8 kHz, a [`CellsProducer`] makes the half-dB cells
//! while the slot is captured, the candidates whose rows have arrived are decoded at **row pair 87** (14.08 s into the slot), the rest when the slot ends, the capped OSD pass last, and the
//! SNR is `SnrBaseline`'s. Selected by the `ft8-cells-flow` feature in place of `decode_pipeline` (the old S3 flow: `dual_core`, `stage1_inc`).
//!
//! **One task that never leaves the audio waiting.** The decode is a queue of small jobs (a slice of the search, one candidate, the OSD pass, the end of the slot) and the loop takes **one job
//! between two looks at the chunk queue**, so a chunk waits at most one job (a candidate: tens of ms, the OSD pass in attempts of 5 ms) against the 485 ms the UAC ring and the queue hold. The first run
//! did the whole emit in one go (985 ms) and would have dropped audio on a radio. The next slot's audio is kept in a second buffer while the first slot's jobs finish (the producer's store is the first
//! slot's until its [`Job::Finalize`]).
//!
//! **What this step does not do** (plan step 3 / 4): no QSO machine (the `Activator` and its queue come next), no cold acquisition and no grid trim (the sink's slot grid is used as it is),
//! no FT8 A/B knobs. After the slot's end only the candidates the partial search found and did not decode (their rows had not arrived) are decoded, no second search: host, 60 slots, 1 019 decodes
//! of the complete flow, 1 014 here (`icap-early` with `NO_FULL=1`: 6 lost, 1 gained) for 470 ms less on the board.
//! The decode window (`CellsOptions::p0_window()`: 30 candidates, `d` floor 16, `CELLS_GATE_OSD`, seeds `AD`, OSD capped at 6, one core) is the library's.

use std::collections::VecDeque;

use embedded_shared::pipeline::{self, ChunkMsg};
use embedded_shared::{esp_dsp_fft, internal_pool};
use esp_idf_svc::sys::QueueHandle_t;
use mfsk_app_shared::ui::state::{SlotDecode, UI};
use mfsk_core::engine::dsp::resample::LinearResamplerI16To8k;
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::ft8::decode_block::{
    candidate_ready, gain_shift_for_peak, merge_cells_parts, search_cells, CellTables,
    CellsOptions, CellsPart, CellsProducer, CellsWorker, Found, Ft8BpScratch, Ft8Rate, GatedSearch,
    OsdOptions, SnrBaseline,
};

/// Samples of a slot at 8 kHz.
const SLOT_8K: usize = 120_000;
/// The row pair the early emit waits for (design note section 31): 174 rows, 14.08 s of audio.
const EMIT_ROWS: usize = 174;
/// The first second of a slot sets the producer's gain (what `stage1_inc` does): the cells are made from here on.
const GAIN_SAMPLES: usize = 8_000;
/// The search is cut into this many ranges of carriers, a job each (a range is about 60 ms on the board).
const SEARCH_PARTS: usize = 8;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// What one slot has: its audio, and what the jobs made of it.
struct Slot {
    audio: Vec<i16>,
    have: usize,
    peak: i32,
    emitted: bool,
    found: Vec<Found>,
    early_cands: Vec<SyncCandidate>,
    /// The candidates of the partial search that were not ready (their rows had not arrived).
    late: Vec<(SyncCandidate, u8)>,
    parts: Vec<CellsPart>,
    emit_at: i64,
    emit_decoded: usize,
    /// `esp_timer` microseconds at the slot's end (the `SlotEnd` message) and the sample count the sink reported.
    end_at: i64,
    wav_idx: usize,
    total_samples: usize,
    /// Microseconds spent in jobs, by kind (search, decode, OSD, finish).
    us_search: i64,
    us_decode: i64,
    us_osd: i64,
    longest_job: i64,
}

impl Slot {
    fn new() -> Self {
        Self {
            audio: vec![0i16; SLOT_8K],
            have: 0,
            peak: 0,
            emitted: false,
            found: Vec::new(),
            early_cands: Vec::new(),
            late: Vec::new(),
            parts: Vec::new(),
            emit_at: 0,
            emit_decoded: 0,
            end_at: 0,
            wav_idx: 0,
            total_samples: 0,
            us_search: 0,
            us_decode: 0,
            us_osd: 0,
            longest_job: 0,
        }
    }

    fn reset(&mut self) {
        self.audio.fill(0);
        self.have = 0;
        self.peak = 0;
        self.emitted = false;
        self.found.clear();
        self.early_cands.clear();
        self.late.clear();
        self.parts.clear();
        self.emit_at = 0;
        self.emit_decoded = 0;
        self.us_search = 0;
        self.us_decode = 0;
        self.us_osd = 0;
        self.longest_job = 0;
    }
}

/// One unit of the decode. A job runs on the slot that owns the producer's store.
enum Job {
    /// Score the carriers of this range of the store.
    Part(core::ops::Range<usize>),
    /// Merge the parts into the candidate list: decode the ready ones, keep the rest for after the slot's end.
    MergeEmit,
    /// The slot ended before the emit (a short slot): merge and decode all of them.
    MergeAll,
    Decode(SyncCandidate, u8),
    /// The slot's end: the rest of the rows.
    Finish,
    /// Decode the candidates kept for after the end.
    LateExpand,
    Osd,
    /// SNR, the list, the log; the producer is free for the next slot.
    Finalize,
}

/// The entry point, the shape of `decode_pipeline::run_with_source`: `source_spawn` gets the chunk queue (wav_sim, or the UAC reader's `set_chunk_q`). Never returns.
pub fn run_with_source<F: FnOnce(QueueHandle_t)>(source: &'static str, source_spawn: F) -> ! {
    crate::log_free_internal("pre-cells-pipeline");
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }
    esp_dsp_fft::prewarm(2560);
    let chunk_q = pipeline::create_chunk_queue(4);
    source_spawn(chunk_q);

    let options = CellsOptions::p0_window();
    let search = GatedSearch::new(200.0, 3000.0, 1.3, options.max_cand);
    let rate = Ft8Rate::Hz8000;
    let depth = options.depth();
    let osd_k = match options.osd {
        OsdOptions::Capped { k, .. } => k,
        _ => 0,
    };
    let tables = CellTables::new();
    // SAFETY: this task is the only user of the main staging buffer (`Ft8Controller::prepare` reserved the arena).
    let cs = unsafe { internal_pool::cs_scratch_main() };
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(cs, &mut bp, &tables).with_options(&options);
    worker.reserve_osd_work();
    // two slots: the one receiving audio (`cur`) and the one whose jobs are running (`owner`, the producer's store is its)
    let mut slots = [Slot::new(), Slot::new()];
    let mut cur = 0usize;
    let mut owner: Option<usize> = None;
    let mut producer: Option<CellsProducer> = None;
    let mut jobs: VecDeque<Job> = VecDeque::new();
    let mut rs = LinearResamplerI16To8k::new(12_000);
    let mut scratch: Vec<i16> = Vec::new();
    let mut calls = mfsk_core::msg::CallsignHashTable::new();
    let mut slot_n = 0u32;
    log::info!(
        "cells pipeline ready ({source}): {} candidates, d floor {}, OSD capped at {osd_k}, emit at {EMIT_ROWS} rows",
        options.max_cand,
        options.d_min_score
    );
    crate::log_free_internal("cells-pipeline-ready");

    loop {
        // a job waits? then look at the queue without waiting; otherwise wait for a message
        let msg = if jobs.is_empty() {
            Some(pipeline::recv_box::<ChunkMsg>(chunk_q))
        } else {
            pipeline::try_recv_box::<ChunkMsg>(chunk_q)
        };
        if let Some(msg) = msg {
            match *msg {
                ChunkMsg::Samples(v) => {
                    scratch.resize(rs.max_output_for(v.len()), 0);
                    let (_, n) = rs.process(&v, &mut scratch);
                    let slot = &mut slots[cur];
                    for &x in &scratch[..n] {
                        if slot.have < SLOT_8K {
                            if slot.have < GAIN_SAMPLES {
                                slot.peak = slot.peak.max((x as i32).abs());
                            }
                            slot.audio[slot.have] = x;
                            slot.have += 1;
                        }
                    }
                }
                ChunkMsg::SlotEnd {
                    wav_idx,
                    total_samples,
                } => {
                    slot_n += 1;
                    if owner == Some(1 - cur) {
                        // the previous slot's jobs are still running a whole slot later: it is lost
                        log::warn!("CELLS slot {slot_n}: the previous slot's decode is not done a slot later ({} jobs): dropped", jobs.len());
                        jobs.clear();
                        slots[1 - cur].reset();
                        owner = None;
                    }
                    let slot = &mut slots[cur];
                    slot.end_at = now_us();
                    slot.wav_idx = wav_idx;
                    slot.total_samples = total_samples;
                    if owner == Some(cur) {
                        jobs.push_back(Job::Finish);
                        if !slot.emitted {
                            for r in split(&producer, &search) {
                                jobs.push_back(Job::Part(r));
                            }
                            jobs.push_back(Job::MergeAll);
                        }
                        jobs.push_back(Job::LateExpand);
                        jobs.push_back(Job::Osd);
                        jobs.push_back(Job::Finalize);
                        cur = 1 - cur; // the audio of the next slot goes to the other buffer
                    } else {
                        log::warn!("CELLS slot {slot_n}: {} samples of 8 kHz, the producer was not started", slot.have);
                        slot.reset();
                    }
                }
                ChunkMsg::SlotResetMark => {
                    if owner == Some(cur) {
                        jobs.clear();
                        owner = None;
                    }
                    slots[cur].reset();
                }
            }
        } else if let Some(job) = jobs.pop_front() {
            let w = owner.expect("a job runs on the owner's slot");
            let t0 = now_us();
            let p = producer.as_mut().expect("the owner has the producer");
            match job {
                Job::Part(r) => {
                    let part = search_cells(p.store(), &search, r);
                    slots[w].parts.push(part);
                    slots[w].us_search += now_us() - t0;
                }
                Job::MergeEmit | Job::MergeAll => {
                    let all = matches!(job, Job::MergeAll);
                    let parts = core::mem::take(&mut slots[w].parts);
                    let list = merge_cells_parts(&parts, &search, rate);
                    let st = p.store();
                    // the decodes go to the front of the queue, in order, so that they run before the end-of-slot jobs
                    let mut decode_jobs = Vec::new();
                    for (c, hs) in list.into_iter().take(options.max_cand) {
                        if all || candidate_ready(st, &c) {
                            slots[w].early_cands.push(c.clone());
                            decode_jobs.push(Job::Decode(c, hs));
                        } else {
                            slots[w].late.push((c, hs));
                        }
                    }
                    for j in decode_jobs.into_iter().rev() {
                        jobs.push_front(j);
                    }
                    slots[w].us_search += now_us() - t0;
                }
                Job::Decode(c, hs) => {
                    if let Some(f) = worker.decode_candidate_hs(p.store(), &c, hs, depth, 0.25) {
                        slots[w].found.push(f);
                    }
                    slots[w].us_decode += now_us() - t0;
                }
                Job::Finish => {
                    // what is decoded when the slot ends is what the emit gave
                    slots[w].emit_decoded = slots[w].found.len();
                    let slot = &slots[w];
                    p.finish(&slot.audio[..slot.have]);
                }
                Job::LateExpand => {
                    let late = core::mem::take(&mut slots[w].late);
                    for (c, hs) in late.into_iter().rev() {
                        jobs.push_front(Job::Decode(c, hs));
                    }
                }
                Job::Osd => {
                    if osd_k > 0 {
                        worker.osd_pass(p.store(), osd_k, depth, &mut slots[w].found);
                    }
                    slots[w].us_osd += now_us() - t0;
                }
                Job::Finalize => {
                    let st = p.store();
                    let base = SnrBaseline::new(st, &tables);
                    for f in slots[w].found.iter_mut() {
                        base.apply(st, &tables, f);
                    }
                    let mut published: Vec<(String, f32, f32, f32, u32)> = Vec::new();
                    for f in &slots[w].found {
                        let r = &f.res;
                        if let Some(text) =
                            mfsk_core::msg::wsjt77::unpack77_learn(r.message77(), &mut calls)
                        {
                            log::info!(
                                "{:+5.1}dB {:+5.2}s {:4.0}Hz {}",
                                r.snr_db,
                                r.dt_sec,
                                r.freq_hz,
                                text
                            );
                            published.push((text, r.freq_hz, r.snr_db, r.dt_sec, r.hard_errors));
                        }
                    }
                    if let Ok(mut ui) = UI.lock() {
                        let rows: Vec<SlotDecode> = published
                            .iter()
                            .map(|(text, freq_hz, snr_db, dt_sec, hard)| SlotDecode {
                                freq_hz: *freq_hz,
                                snr_db: *snr_db,
                                dt_sec: *dt_sec,
                                text,
                                hard_errors: *hard,
                            })
                            .collect();
                        crate::storage::publish_slot(
                            &mut ui,
                            mfsk_app_shared::boot_mode::BootMode::Uac,
                            crate::storage::decoded_slot_unix(15_000),
                            &rows,
                        );
                    }
                    let done = now_us();
                    let s = &slots[w];
                    // `after the end` is what the reply deadline cares about: how long after the slot's last audio its decodes were all out
                    log::info!(
                        "CELLS slot {slot_n} wav {} samples {}: {} decodes ({} from the {} ready candidates at the emit), {} ms after the end; jobs: search {} ms, decode {} ms, OSD {} ms, longest job {} ms; emit {} ms before the end",
                        s.wav_idx,
                        s.total_samples,
                        s.found.len(),
                        s.emit_decoded,
                        s.early_cands.len(),
                        (done - s.end_at) / 1000,
                        s.us_search / 1000,
                        s.us_decode / 1000,
                        s.us_osd / 1000,
                        s.longest_job / 1000,
                        if s.emitted { (s.end_at - s.emit_at) / 1000 } else { -1 },
                    );
                    crate::log_free_internal("cells-slot-end");
                    if slot_n <= 3 || slot_n % 20 == 0 {
                        crate::board::log_task_stacks();
                    }
                    slots[w].reset();
                    owner = None;
                }
            }
            let took = now_us() - t0;
            if let Some(w) = owner {
                slots[w].longest_job = slots[w].longest_job.max(took);
            }
        }

        // the producer: start it on the receiving slot once its first second is in and the store is free, then make the rows the audio so far allows
        if owner.is_none() && slots[cur].have >= GAIN_SAMPLES {
            let shift = gain_shift_for_peak(slots[cur].peak);
            match producer.as_mut() {
                Some(p) => p.restart(shift),
                None => producer = CellsProducer::new(rate, &search, shift),
            }
            if producer.is_some() {
                owner = Some(cur);
            }
        }
        if owner == Some(cur) {
            if let Some(p) = producer.as_mut() {
                let slot = &mut slots[cur];
                let rows = p.advance(&slot.audio, slot.have);
                if !slot.emitted && rows >= EMIT_ROWS {
                    slot.emitted = true;
                    slot.emit_at = now_us();
                    for r in split(&producer, &search) {
                        jobs.push_back(Job::Part(r));
                    }
                    jobs.push_back(Job::MergeEmit);
                }
            }
        }
    }
}

/// The store's carriers cut into [`SEARCH_PARTS`] ranges.
fn split(producer: &Option<CellsProducer>, _search: &GatedSearch) -> Vec<core::ops::Range<usize>> {
    let Some(p) = producer else { return Vec::new() };
    let c = p.store().carrier_bins();
    let n = c.len();
    (0..SEARCH_PARTS)
        .map(|i| (c.start + i * n / SEARCH_PARTS)..(c.start + (i + 1) * n / SEARCH_PARTS))
        .collect()
}
