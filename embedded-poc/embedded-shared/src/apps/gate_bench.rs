//! FT8 gate + `fine_sync_12k` + OSD bench (CoreS3 / ESP32-S3 / LX7): the S3 plan's stage 1
//! (`docs/notes/S3_GATE_OSD_ABLATION.md`, the plan's "段階 1").
//!
//! The host says (`ft8_embedded_pipeline_mirror::mirror_gate_and_osd`) that adding the hard-sync gate, the board's
//! own `fine_sync_12k` and OSD to the shipped pipeline takes `qso3_busy` from 6.6 to 11.4 decodes of 20, with no
//! unexplained decode. It cannot say what that costs here. This bench measures that, on the board, in the board's
//! numeric path, one core, one slot at a time:
//!
//! - the time of each stage (spectrogram, coarse search, `fine_sync_12k`, pass 2, stage 3) for each arm,
//! - what OSD adds (arm differences) and how many candidates reach stage 3 against how many decode (the rest are
//!   the candidates OSD is tried on, an upper bound: OSD's own gate is `osd_escalation_gates`),
//! - the stack high-water mark of the task it runs on, after each arm (OSD's arrays are on the stack:
//!   `osd_npre`'s `g`, `orig`, `indices`, about 9 KB; `dsp_worker`'s stack is 16 KB, the main task's 16 KB on the CoreS3),
//! - the minimum free internal DRAM and PSRAM since boot, after each arm.
//!
//! Every arm runs the same stage 3 (`process_candidates_into_with_cs_scratch_tuned`, one candidate at a time, the
//! `cs` scratch in internal DRAM from `internal_pool`, as `dual_core::stage3_split` does), so the arms differ only in
//! how candidates are picked and refined, and in OSD. Each runs twice; the second is the one to read (the first
//! pays for cold caches and first allocations).
//!
//! | arm | pass 1 | fine sync | OSD |
//! |---|---|---|---|
//! | `ship` | coarse score order (`coarse_sync`), 30 | no | no |
//! | `ship+fine` | as `ship` | `fine_sync_12k` on the 30 | no |
//! | `gate` | `coarse_sync_gated_spec`, the first `MAX_CAND` | no | no |
//! | `gate+fine` | as `gate` | `fine_sync_12k` on the `MAX_CAND` | no |
//! | `ship+fine+osd` | as `ship+fine` | | yes |
//! | `gate+fine+osd` | as `gate+fine` | | yes |
//!
//! The OSD arms run last, so that the stack high-water mark after the arms before them is the stack without OSD.
//!
//! **What it does not do**: the streaming spectrogram (`stage1_inc`: here the spectrogram is built whole after the
//! slot, `compute_spectrogram`, so stage 1 and the coarse search are measured as post-slot work, which on the board
//! they are not), the second core, the deadline, the audio prefix. Compare the candidate stages (fine sync, pass 2,
//! stage 3), not the totals.
//!
//! Per-target binaries supply the WAV list and a target name for the boot log
//! (`m5stack-cores3-app/src/bin/ft8_gate_bench.rs`).

extern crate alloc;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::vec::Vec;

use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::ft8::decode::{DecodeDepth, DecodeResult};
use mfsk_core::ft8::decode_block::{
    DEFAULT_Q_THRESH, RefinedCandidate, coarse_sync, coarse_sync_gated_spec, compute_spectrogram,
    fine_sync_12k, process_candidates_into_with_cs_scratch_tuned, refine_candidates_into,
};
use mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER;
use mfsk_core::msg::wsjt77::unpack77;

use crate::internal_pool;

/// `m5stack-cores3-app`'s `DecodeConfig`: 100-3000 Hz, `sync_min` 1.0, `PASS1_LIMIT` 30, `MAX_CAND` 15, the lag window the
/// board searches (`EMBEDDED_SYNC_LAG_S` is 1.0; `decode_pipeline` narrows it to the coverage, 0.88 in the mirror).
const FREQ_MIN: f32 = 100.0;
const FREQ_MAX: f32 = 3_000.0;
const SYNC_MIN: f32 = 1.0;
const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;
const SYNC_LAG_S: f32 = 0.88;
const SLOT_LEN: usize = 180_000;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn load_wav_slot(wav: &[u8]) -> Box<[i16]> {
    const HEADER: usize = 44;
    let payload = &wav[HEADER..];
    let n = payload.len() / 2;
    let mut slot: Vec<i16> = alloc::vec![0i16; SLOT_LEN];
    for i in 0..n.min(SLOT_LEN) {
        slot[i] = i16::from_le_bytes([payload[i * 2], payload[i * 2 + 1]]);
    }
    slot.into_boxed_slice()
}

#[derive(Clone, Copy)]
struct Arm {
    name: &'static str,
    gate: bool,
    fine: bool,
    osd: bool,
}

const ARMS: [Arm; 6] = [
    Arm {
        name: "ship",
        gate: false,
        fine: false,
        osd: false,
    },
    Arm {
        name: "ship+fine",
        gate: false,
        fine: true,
        osd: false,
    },
    Arm {
        name: "gate",
        gate: true,
        fine: false,
        osd: false,
    },
    Arm {
        name: "gate+fine",
        gate: true,
        fine: true,
        osd: false,
    },
    Arm {
        name: "ship+fine+osd",
        gate: false,
        fine: true,
        osd: true,
    },
    Arm {
        name: "gate+fine+osd",
        gate: true,
        fine: true,
        osd: true,
    },
];

/// What one arm did on one slot, times in microseconds.
struct Out {
    us_spec: i64,
    us_coarse: i64,
    us_fine: i64,
    us_pass2: i64,
    us_stage3: i64,
    n_pass1: usize,
    n_stage3: usize,
    messages: Vec<String>,
}

fn run_arm(slot: &[i16], arm: Arm) -> Out {
    let t0 = now_us();
    let spec = compute_spectrogram(slot, FREQ_MAX);
    let t1 = now_us();
    let mut cands: Vec<SyncCandidate> = if arm.gate {
        coarse_sync_gated_spec(
            &spec,
            FREQ_MIN,
            FREQ_MAX,
            SYNC_MIN,
            PASS1_LIMIT,
            Some(SYNC_LAG_S),
            None,
        )
        .into_iter()
        .map(|c| c.0)
        .collect()
    } else {
        coarse_sync(&spec, FREQ_MIN, FREQ_MAX, SYNC_MIN, PASS1_LIMIT)
    };
    drop(spec);
    let t2 = now_us();
    let n_pass1 = cands.len();
    // the gate's order is the ranking: its first `MAX_CAND` go on (the pass-2 re-rank would undo it)
    if arm.gate {
        cands.truncate(MAX_CAND);
    }
    let t_fine0 = now_us();
    if arm.fine {
        cands = fine_sync_12k(slot, &cands);
    }
    let t3 = now_us();
    let refined: Vec<RefinedCandidate> = refine_candidates_into(slot, cands, MAX_CAND);
    let t4 = now_us();
    let n_stage3 = refined.len();
    let depth = DecodeDepth {
        osd: arm.osd,
        ..DecodeDepth::EMBEDDED
    };
    // SAFETY: one caller at a time (the bench runs on the main task, no worker); the arena was reserved at boot.
    let cs = unsafe { internal_pool::cs_scratch_main() };
    let mut results: Vec<DecodeResult> = Vec::new();
    for c in refined {
        results.extend(process_candidates_into_with_cs_scratch_tuned(
            slot,
            alloc::vec![c],
            depth,
            DEFAULT_Q_THRESH,
            DEFAULT_BP_MAX_ITER,
            cs,
        ));
    }
    let t5 = now_us();
    // one row per message, the first found (`dual_core::dedup_by_message`)
    let mut seen: Vec<[u8; 77]> = Vec::new();
    let mut messages = Vec::new();
    for r in &results {
        let m = *r.message77();
        if !seen.contains(&m) {
            seen.push(m);
            if let Some(text) = unpack77(&m) {
                messages.push(text);
            }
        }
    }
    Out {
        us_spec: t1 - t0,
        us_coarse: t2 - t1,
        us_fine: t3 - t_fine0,
        us_pass2: t4 - t3,
        us_stage3: t5 - t4,
        n_pass1,
        n_stage3,
        messages,
    }
}

/// Memory and stack figures: minimum free internal DRAM and PSRAM since boot, the stack high-water mark of the
/// running task (the least free stack it has ever had, in bytes).
fn mem_line() -> String {
    const MALLOC_CAP_INTERNAL: u32 = 1 << 11;
    const MALLOC_CAP_8BIT: u32 = 1 << 2;
    const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
    unsafe {
        let internal =
            esp_idf_svc::sys::heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        let psram =
            esp_idf_svc::sys::heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        // words on Xtensa FreeRTOS: 4 bytes each
        let stack = esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) as usize * 4;
        alloc::format!("min_free_internal={internal} min_free_psram={psram} stack_free_min={stack}")
    }
}

/// Run the bench over `wavs` (`(label, wav bytes)`), then idle.
pub fn run(target_name: &str, wavs: &'static [(&'static str, &'static [u8])]) -> ! {
    crate::apps::compute_bench::init_logger_once();
    log::info!("mfsk-core-{target_name} gate bench starting, mfsk-core {}", mfsk_core::VERSION);
    log::info!("config: {FREQ_MIN}-{FREQ_MAX} Hz, sync_min {SYNC_MIN}, pass1 {PASS1_LIMIT}, max_cand {MAX_CAND}, lag {SYNC_LAG_S} s");
    log::info!("GATEBENCH mem boot {}", mem_line());
    crate::esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);

    let slots: Vec<(&str, Box<[i16]>)> = wavs
        .iter()
        .map(|(label, wav)| (*label, load_wav_slot(wav)))
        .collect();
    for (label, slot) in &slots {
        log::info!("GATEBENCH wav {label}");
        for arm in ARMS {
            // cold, then warm: read the warm one
            for pass in ["cold", "warm"] {
                let o = run_arm(slot, arm);
                log::info!(
                    "GATEBENCH wav={label} arm={} run={pass} us_spec={} us_coarse={} us_fine={} us_pass2={} us_stage3={} n_pass1={} n_stage3={} decodes={} {}",
                    arm.name,
                    o.us_spec,
                    o.us_coarse,
                    o.us_fine,
                    o.us_pass2,
                    o.us_stage3,
                    o.n_pass1,
                    o.n_stage3,
                    o.messages.len(),
                    mem_line()
                );
                if pass == "warm" {
                    for m in &o.messages {
                        log::info!("GATEBENCH msg wav={label} arm={} '{m}'", arm.name);
                    }
                }
            }
        }
    }
    log::info!("GATEBENCH done");
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(u32::MAX) };
    }
}
