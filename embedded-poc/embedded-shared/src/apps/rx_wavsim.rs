//! WAV-fed streaming RX bench, target-agnostic.
//!
//! Runs the full pipeline:
//! ```text
//! wav_sim → ChunkMsg → stage1_inc → SpecBundle/Slot → main → dual_core
//! ```
//!
//! Stage 2 (`coarse_sync_split_with_allsum`) starts on `SpecBundle`
//! arrival (≈ 200 ms before SlotEnd), so it overlaps the tail of audio
//! capture. Post-SlotEnd decode = pass 2 + stage 3 only.
//!
//! Per-target binaries (m5stack-core2 / m5stack-s3) just supply the
//! WAV byte slices and call `run()`. No target-specific code lives
//! here — chip / PSRAM / priority knobs are uniform across LX6 and LX7.

extern crate alloc;

use alloc::vec::Vec;

use mfsk_ft8::mfsk_ft8_basis_scratch_len;

use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
use mfsk_core::msg::wsjt77::unpack77;

use crate::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};

/// Per-core BASIS scratch. 60 KB × 4, internal DRAM `.bss`. Phase 0.7:
/// both pairs live here so rx_wavsim keeps a single static allocation
/// strategy; only the m5stack-s3-app runtime heap-allocates so it can
/// skip the 120 KB worker pair in WiFi mode.
static mut BASIS_RE: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_RE_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];
static mut BASIS_IM_C1: [i16; BASIS_SCRATCH_LEN] = [0; BASIS_SCRATCH_LEN];

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Run the rx_wavsim bench.
///
/// * `wavs` — playlist (≥ 1 baked WAV; cycled indefinitely).
/// * `pass1_limit` — coarse_sync candidate cap (typical 30 for ship
///   recall floor; raise to 100 to recover one extra borderline weak
///   signal at ~1.7× post-SlotEnd time on LX7).
/// * `max_cand` — pass 2 / stage 3 candidate cap (typical 15; raise
///   to 30 alongside `pass1_limit=100`).
/// * `q_thresh` — stage-3 `sync_quality` early-reject threshold; pass
///   [`mfsk_core::ft8::decode_block::DEFAULT_Q_THRESH`] (= 12) for
///   full recall.
/// * `bp_max_iter` — stage-3 BP iteration cap per LLR variant. Pass
///   [`mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER`] (= 30, WSJT-X
///   reference) for full recall; lower (e.g. 20 / 15) trades weak-
///   signal recall for post-SlotEnd time. Stage 3 wall-clock scales
///   roughly linearly with this on cands that exhaust all 4 LLR
///   variants (≈ half of `max_cand` on busy bands).
/// One sweep config — applied per-slot when `run_sweep` is used.
#[derive(Clone, Copy, Debug)]
pub struct RxSweepCfg {
    pub pass1_limit: usize,
    pub max_cand: usize,
    pub q_thresh: u32,
    pub bp_max_iter: u32,
}

/// Single-config wrapper around [`run_sweep`] for the simple case.
pub fn run(
    wavs: &'static [&'static [u8]],
    pass1_limit: usize,
    max_cand: usize,
    q_thresh: u32,
    bp_max_iter: u32,
) -> ! {
    let cfg = alloc::boxed::Box::leak(alloc::boxed::Box::new([RxSweepCfg {
        pass1_limit,
        max_cand,
        q_thresh,
        bp_max_iter,
    }]));
    run_sweep(wavs, cfg)
}

/// Sweep variant: rotate through `cfgs` per slot. Slot index `i` uses
/// cfg `i % cfgs.len()`. Each cfg gets a header banner so log readers
/// can group results by config.
pub fn run_sweep(wavs: &'static [&'static [u8]], cfgs: &'static [RxSweepCfg]) -> ! {
    assert!(!cfgs.is_empty(), "cfgs empty");
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log::info!(
        "rx-wavsim starting (mfsk-core {}, NFFT_SPEC={})",
        mfsk_core::VERSION,
        mfsk_core::ft8::decode_block::NFFT_SPEC
    );

    let need = mfsk_ft8_basis_scratch_len();
    assert!(BASIS_SCRATCH_LEN >= need, "BASIS_SCRATCH_LEN too small");

    // Bump main task priority above wav_sim (4) and stage1_inc (3).
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);

    #[allow(static_mut_refs)]
    unsafe {
        dual_core::init(BASIS_RE_C1.as_mut_ptr(), BASIS_IM_C1.as_mut_ptr());
    }
    let chunk_q = pipeline::create_chunk_queue(4);
    let slot_q = pipeline::create_slot_queue(2);
    let spec_q = pipeline::create_spec_queue(2);
    stage1_inc::spawn(chunk_q, slot_q, spec_q);
    wav_sim::spawn(wavs, chunk_q);

    log::info!(
        "rx-wavsim: decode loop ready (sweep n_cfgs={}); awaiting spec/slot from stage1_inc",
        cfgs.len(),
    );
    let mut slot_i: usize = 0;
    loop {
        let cfg = cfgs[slot_i % cfgs.len()];
        slot_i = slot_i.wrapping_add(1);
        // SpecBundle arrives ≈ 200 ms before SlotEnd, so stage 2 runs
        // in parallel with the tail of capture.
        let spec = pipeline::recv_box::<pipeline::SpecBundle>(spec_q);
        log::info!(
            "─── slot {} cfg pass1={} max_cand={} q={} BP={}",
            slot_i,
            cfg.pass1_limit,
            cfg.max_cand,
            cfg.q_thresh,
            cfg.bp_max_iter,
        );
        let t_s2_start = now_us();
        let pass1 = dual_core::coarse_sync_split_with_allsum(
            &spec.spec,
            100.0,
            3_000.0,
            1.0,
            cfg.pass1_limit,
            &spec.allsum_head,
            &spec.allsum_tail,
        );
        let t_s2_end = now_us();
        drop(spec);

        let slot = pipeline::recv_box::<pipeline::Slot>(slot_q);
        decode_one_slot(
            *slot,
            pass1,
            t_s2_end - t_s2_start,
            cfg.max_cand,
            cfg.q_thresh,
            cfg.bp_max_iter,
        );
    }
}

fn decode_one_slot(
    slot: pipeline::Slot,
    pass1: Vec<SyncCandidate>,
    stage2_us: i64,
    max_cand: usize,
    q_thresh: u32,
    bp_max_iter: u32,
) {
    let wav_idx = slot.wav_idx;
    let inc_us = slot.inc_total_us;
    let pass1_n = pass1.len();
    let post_slot_t0 = now_us();

    log::info!(
        "rx-wavsim: WAV[{wav_idx}] slot received (audio={} samples, pass1={pass1_n}, q_thresh={q_thresh})",
        slot.audio_len,
    );
    log::info!(
        "  stage 2 (during cap): {:>7} us  ({pass1_n} cand)",
        stage2_us
    );

    let t2 = now_us();
    #[allow(static_mut_refs)]
    let pass2 = unsafe {
        dual_core::pass2_split(slot.audio(), pass1, max_cand, &mut BASIS_RE, &mut BASIS_IM)
    };
    let t3 = now_us();
    log::info!(
        "  pass 2:               {:>7} us  → top {}",
        t3 - t2,
        pass2.len()
    );

    let depth = DecodeDepth::BpAll;
    let t4 = now_us();
    #[allow(static_mut_refs)]
    let results = unsafe {
        dual_core::stage3_split(
            slot.audio(),
            pass2,
            depth,
            q_thresh,
            bp_max_iter,
            &mut BASIS_RE,
            &mut BASIS_IM,
        )
    };
    let t5 = now_us();
    log::info!(
        "  stage 3:              {:>7} us  ({} result(s))",
        t5 - t4,
        results.len()
    );
    log::info!(
        "  ─── post-SlotEnd:     {:>7} us = {:.3} s",
        t5 - post_slot_t0,
        (t5 - post_slot_t0) as f64 / 1e6
    );
    log::info!(
        "  Phase-E: stage1_inc {} us in advance ({}% of capture)",
        inc_us,
        (inc_us * 100) / 15_000_000
    );

    for (i, r) in results.iter().enumerate() {
        if let Some(text) = unpack77(&r.message77) {
            log::info!(
                "    [{i}]  {:>4.0} Hz  SNR={:>5.1} dB  e={}  '{}'",
                r.freq_hz,
                r.snr_db,
                r.hard_errors,
                text
            );
        }
    }
}
