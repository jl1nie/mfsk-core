//! The old S3 flow (12 kHz spectrogram, `sync8` coarse search, pass 2 re-rank, stage 3 of Goertzel + BP) on **one core and on two**, the same candidates: the measurement the old two-core design never had.
//! (`PHASE_D_PIE_SIMD.md` took 122 ms of two-core wall as "244 ms of single-core work", an assumption: `dual_core::pass2_split`'s own comment says that until its counters existed nobody had measured the split
//! "in either direction". Section 30 of `FT8_WINDOW_AND_COARSE_DESIGN.md` and `ESP32S3_TWO_CORE_BEHAVIOUR.md` measured what two cores cost the cells flow; this is the same question asked of the flow the cells flow replaces.)
//!
//! Per recording (12 kHz), the 30 coarse candidates made once on one core; then, for `max_cand` 15 and 30:
//!
//! | stage | one core | two cores |
//! |---|---|---|
//! | pass 2 (Goertzel of Costas block 0, re-rank) | `refine_candidates_into` over the 30 | `dual_core::pass2_split` (a fixed head / tail split, as production) |
//! | stage 3 (Goertzel of the data symbols + BP) | `process_candidates_into_with_cs_scratch_tuned` a candidate at a time | `dual_core::stage3_split` (work stealing through an atomic index, as production) |
//!
//! `DecodeDepth::EMBEDDED` (BP, no OSD) and `DEFAULT_Q_THRESH` in both, so the sets of decodes must be equal (the bench checks it). The coarse search is not compared: its two-core form needs the incremental
//! spectrogram's allsum (`stage1_inc`), and `dual_core::coarse_sync_split` is a sequential fallback. Each measurement is made twice; the second is read. Wav replay, no WiFi, no USB host.
//!
//! **Predictions, written before the first run.** From the loop shapes of `ESP32S3_TWO_CORE_BEHAVIOUR.md` (a long unrolled body pays about x1.12 for the other core, a tight loop x1.3-1.4) and the cells flow's own
//! figures (BP round x1.5): the Goertzel of pass 2 and of stage 3's data symbols is the long-body case (`1.6 cycles a tone and sample`, `PHASE_D_PIE_SIMD.md`), the BP of stage 3 the short-loop one. **Pass 2: x1.6-1.9** (two
//! cores 0.14-0.19 s, one core 0.25-0.33 s: the Aug 2026 log has 0.165 s on two). **Stage 3: x1.5-1.75 at 30 candidates, x1.4-1.7 at 15** (two cores 0.9-1.2 s as in the Aug log at 15 and 30 candidates, `q>12` there and
//! `q>6` here, so the stage 3 here is longer: a one-core figure of 1.5-2.3 s at 15, 1.9-2.9 s at 30). If both are below x1.5 the old flow was no better placed than the cells flow, and the "worked well" of the old
//! design was the latency target met, not a scaling; if both are above x1.8 the loop shape explains why the BP / OSD of the cells flow scales worse. The sets of decodes: equal.

extern crate alloc;
extern crate std;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;

use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    DEFAULT_Q_THRESH, RefinedCandidate, coarse_sync_with_lag, compute_spectrogram, process_candidates_into_with_cs_scratch_tuned, refine_candidates_into,
};
use mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER;
use mfsk_core::msg::wsjt77::unpack77;

use crate::{dual_core, esp_dsp_fft, internal_pool};

const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
const PASS1_LIMIT: usize = 30;
const SYNC_LAG_S: f32 = 0.88;
const SLOT_LEN: usize = 180_000;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn spawn_pinned(name: &'static core::ffi::CStr, core: esp_idf_svc::hal::cpu::Core, stack: usize, f: impl FnOnce() + Send + 'static) -> std::thread::JoinHandle<()> {
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
    let d = ThreadSpawnConfiguration::default();
    ThreadSpawnConfiguration { name: Some(name), stack_size: stack, pin_to_core: Some(core), ..d }
        .set()
        .expect("old-s3-bench: thread config");
    let h = std::thread::Builder::new().stack_size(stack).spawn(f).expect("old-s3-bench: spawn");
    let _ = ThreadSpawnConfiguration::default().set();
    h
}

pub fn run(target: &str, wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    log::info!("old-s3-bench: {target}, the old S3 flow's pass 2 and stage 3 on one core and on two (predictions in the module doc)");
    esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);
    dual_core::init();
    // the bench thread is core 0 (PRO_CPU): `stage3_split` takes the main half of the staging arena there
    let h = spawn_pinned(c"old_s3_bench", esp_idf_svc::hal::cpu::Core::Core0, 48 * 1024, move || body(wavs));
    let _ = h.join();
    log::info!("old-s3-bench: done");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

fn messages(r: &[DecodeResult]) -> Vec<String> {
    let mut t: Vec<String> = Vec::new();
    for d in r {
        if let Some(m) = unpack77(d.message77()) {
            if !t.contains(&m) {
                t.push(m);
            }
        }
    }
    t.sort();
    t
}

fn clone_refined(r: &[RefinedCandidate]) -> Vec<RefinedCandidate> {
    r.iter().map(|(c, b, q)| (c.clone(), b.clone(), *q)).collect()
}

fn key(r: &[RefinedCandidate]) -> Vec<(u32, u32, u32)> {
    r.iter().map(|(c, _, q)| (c.freq_hz.to_bits(), c.dt_sec.to_bits(), *q)).collect()
}

fn body(wavs: &'static [(&'static str, &'static [u8])]) {
    for (label, wav) in wavs {
        let slot: Box<[i16]> = wav[44..].chunks_exact(2).take(SLOT_LEN).map(|b| i16::from_le_bytes([b[0], b[1]])).collect();
        let spec = compute_spectrogram(&slot, FREQ_MAX);
        let pass1: Vec<SyncCandidate> = coarse_sync_with_lag(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, PASS1_LIMIT, SYNC_LAG_S);
        drop(spec);
        log::info!("old-s3-bench: {label}: {} coarse candidates", pass1.len());
        for max_cand in [15usize, 30] {
            // pass 2
            let (mut p1, mut p2) = (0i64, 0i64);
            let (mut r1, mut r2) = (Vec::new(), Vec::new());
            for _ in 0..2 {
                let t0 = now_us();
                r1 = refine_candidates_into(&slot, pass1.clone(), max_cand);
                p1 = now_us() - t0;
                let t0 = now_us();
                r2 = dual_core::pass2_split(&slot, pass1.clone(), max_cand);
                p2 = now_us() - t0;
            }
            log::info!(
                "old-s3-bench: {label} max_cand {max_cand} pass 2: 1 core {p1} us, 2 cores {p2} us (x{:.2}); the re-ranked lists {}",
                p1 as f64 / p2 as f64,
                if key(&r1) == key(&r2) { "equal" } else { "DIFFER" }
            );
            // stage 3 over the one-core list
            let depth = DecodeDepth::EMBEDDED;
            let (mut s1, mut s2) = (0i64, 0i64);
            let (mut m1, mut m2) = (Vec::new(), Vec::new());
            for _ in 0..2 {
                // SAFETY: this thread is the only user of the main staging half while `stage3_split` is not running
                let cs = unsafe { internal_pool::cs_scratch_main() };
                let list = clone_refined(&r1);
                let t0 = now_us();
                let mut res: Vec<DecodeResult> = Vec::new();
                for c in list {
                    res.extend(process_candidates_into_with_cs_scratch_tuned(&slot, alloc::vec![c], depth, DEFAULT_Q_THRESH, DEFAULT_BP_MAX_ITER, cs));
                }
                s1 = now_us() - t0;
                m1 = messages(&res);
                let list = clone_refined(&r1);
                let t0 = now_us();
                let res2 = dual_core::stage3_split(&slot, list, depth, DEFAULT_Q_THRESH, DEFAULT_BP_MAX_ITER, dual_core::Stage3Stop::never()).results;
                s2 = now_us() - t0;
                m2 = messages(&res2);
            }
            log::info!(
                "old-s3-bench: {label} max_cand {max_cand} stage 3: 1 core {s1} us ({} decodes), 2 cores {s2} us ({} decodes) (x{:.2}); the sets {}",
                m1.len(),
                m2.len(),
                s1 as f64 / s2 as f64,
                if m1 == m2 { "equal" } else { "DIFFER" }
            );
            log::info!(
                "old-s3-bench: {label} max_cand {max_cand} pass 2 + stage 3: 1 core {} us, 2 cores {} us (x{:.2})",
                p1 + s1,
                p2 + s2,
                (p1 + s1) as f64 / (p2 + s2) as f64
            );
        }
    }
    log::info!("old-s3-bench: finished");
}
