//! WAV-fed streaming RX bench, target-agnostic.
//!
//! Runs the full pipeline:
//! ```text
//! wav_sim → ChunkMsg → stage1_inc → SpecBundle/Slot → main → dual_core
//! ```
//!
//! Uses [`dual_core::run_speculative_slot`] — the same Phase-C
//! speculative runner as `m5stack-s3-app` and `m5stack-core2-app`.
//! Both the early (audio-tail overlap) and deferred (post-SlotEnd)
//! decode paths run; the per-slot log shows the Phase-C breakdown so
//! bench results are directly comparable to production timing.
//!
//! Per-target binaries (m5stack-core2 / m5stack-s3) just supply the
//! WAV byte slices and call `run()`. No target-specific code lives
//! here — chip / PSRAM / priority knobs are uniform across LX6 and LX7.

extern crate alloc;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::msg::wsjt77::unpack77;

use crate::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};

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

    // Phase 1.7.7: BASIS scratch retired — Goertzel needs no internal
    // DRAM scratch. Pass null to dual_core::init; empty slices to
    // run_speculative_slot. The 4 × 30 KB that used to live here as
    // `static mut BASIS_*` arrays is now free DRAM.
    let basis_re_main: &'static mut [i16] =
        alloc::boxed::Box::leak(alloc::vec![].into_boxed_slice());
    let basis_im_main: &'static mut [i16] =
        alloc::boxed::Box::leak(alloc::vec![].into_boxed_slice());

    // Bump main task priority above wav_sim (4) and stage1_inc (3).
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(mfsk_core::ft8::decode_block::NFFT_SPEC);
    dual_core::init(core::ptr::null_mut(), core::ptr::null_mut());

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
        let rx_cfg = cfgs[slot_i % cfgs.len()];
        slot_i = slot_i.wrapping_add(1);

        log::info!(
            "─── slot {} cfg pass1={} max_cand={} q={} BP={}",
            slot_i,
            rx_cfg.pass1_limit,
            rx_cfg.max_cand,
            rx_cfg.q_thresh,
            rx_cfg.bp_max_iter,
        );

        let dc = dual_core::DecodeConfig {
            freq_min: 100.0,
            freq_max: 3_000.0,
            sync_min: 1.0,
            pass1_limit: rx_cfg.pass1_limit,
            max_cand: rx_cfg.max_cand,
            q_thresh: rx_cfg.q_thresh,
            bp_max_iter: rx_cfg.bp_max_iter,
            depth: DecodeDepth::BpVariantsAd,
        };
        let out =
            dual_core::run_speculative_slot(spec_q, slot_q, &dc, basis_re_main, basis_im_main);
        let dual_core::SpeculativeOut {
            spec,
            slot,
            results,
            n_pass1,
            n_ready,
            n_deferred,
            bootstrap_dt_med: _,
            t_post_recv,
            t_coarse_done,
            t_early_done,
            t_slot_recv,
            t_done,
        } = out;

        let slotend = slot.slotend_us;
        let inc_us = slot.inc_total_us;
        let wav_idx = slot.wav_idx;
        let tail_window = (slotend - t_post_recv).max(0);
        let coarse_us = t_coarse_done - t_post_recv;
        let early_us = t_early_done - t_coarse_done;
        let tail_use = (slotend.min(t_early_done) - t_coarse_done).max(0);
        let post_slotend = (t_done - slotend).max(0);

        log::info!(
            "WAV[{wav_idx}] p1={n_pass1} ready={n_ready} defer={n_deferred} dec={} \
             tail_win={}us coarse={}us early={}us tail_use={}us post_slotend={}us \
             slot_wait={}us late={}us",
            results.len(),
            tail_window,
            coarse_us,
            early_us,
            tail_use,
            post_slotend,
            t_slot_recv - t_early_done,
            t_done - t_slot_recv,
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
        drop(spec);
    }
}
