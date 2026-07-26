//! WAV-fed decode pipeline for CoreS3 (Phase 0-Core).
//!
//! Structurally identical to `m5stack-core2-app/src/decode_pipeline.rs`.
//! All heavy lifting is in `embedded_shared` and `mfsk_app_shared` which
//! are board-agnostic. Board-specific touch points are `crate::log_free_internal`
//! and the `QSO_WAVS` slice (same qso3_busy.wav reference as Core2).
//! Phase 1-Core replaces the wav_sim source with a UAC host capture.

extern crate alloc;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{DEFAULT_Q_THRESH, NFFT_SPEC};
use mfsk_core::msg::wsjt77::unpack77;

use embedded_shared::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};
use esp_idf_svc::sys::QueueHandle_t;

use mfsk_app_shared::qso::{self, QsoManager, QsoState};
use mfsk_app_shared::ui::state::{DecodedRow, UI};

const MY_CALL: &str = "JL1NIE";
const MY_GRID: &str = "PM95";

static QSO_WAVS: &[&[u8]] = &[
    include_bytes!("../../assets/qso3_busy.wav"),
];

const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;

/// `BootMode::Decode` entry — runs the decode pipeline with the baked
/// `QSO_WAVS` playlist as the audio source. Thin wrapper around
/// [`run_with_source`].
pub fn run() -> ! {
    run_with_source(|q| wav_sim::spawn(QSO_WAVS, q))
}

/// Source-agnostic entry. Allocates the pipeline queues, spawns
/// `stage1_inc` + `wf_drain`, calls `source_spawn` (which must push
/// `ChunkMsg::Samples` + `ChunkMsg::SlotEnd` into the chunk queue),
/// then runs the decode loop. Never returns.
///
/// `Decode` mode passes `|q| wav_sim::spawn(QSO_WAVS, q)`.
/// `Uac` mode passes `|q| uac::set_chunk_q(q)` — the UAC reader thread
/// starts pushing once it sees the queue handle land in its static slot.
pub fn run_with_source<F: FnOnce(QueueHandle_t)>(source_spawn: F) -> ! {
    crate::log_free_internal("pre-decode-loop (post-Goertzel: no BASIS alloc)");

    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(NFFT_SPEC);
    dual_core::init();

    let chunk_q = pipeline::create_chunk_queue(4);
    let slot_q = pipeline::create_slot_queue(2);
    let spec_q = pipeline::create_spec_queue(2);
    let wf_q = pipeline::create_wf_queue(8);
    stage1_inc::spawn_with_wf(chunk_q, slot_q, spec_q, Some(wf_q));
    source_spawn(chunk_q);

    let wf_q_addr = wf_q as usize;
    std::thread::Builder::new()
        .stack_size(4 * 1024)
        .spawn(move || wf_drain(wf_q_addr as esp_idf_svc::sys::QueueHandle_t))
        .expect("spawn wf drainer");

    log::info!("decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz, cores3-app phase 0)");

    let mut qso = QsoManager::new(MY_CALL, MY_GRID);
    let initial = qso.call_cq(None);
    push_tx_line(&qso, Some(&initial));

    let mut slot_seq: u32 = 0;
    loop {
        let cfg = dual_core::DecodeConfig {
            freq_min: 100.0,
            freq_max: 3_000.0,
            sync_min: 1.0,
            pass1_limit: PASS1_LIMIT,
            max_cand: MAX_CAND,
            q_thresh: DEFAULT_Q_THRESH,
            bp_max_iter: mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
            depth: DecodeDepth::EMBEDDED,
        };
        let out = dual_core::run_speculative_slot(spec_q, slot_q, &cfg);
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
        let wav_idx = slot.wav_idx;

        let slotend = slot.slotend_us;
        let tail_window = (slotend - t_post_recv).max(0);
        let coarse_us = t_coarse_done - t_post_recv;
        let tail_use = (slotend.min(t_early_done) - t_coarse_done).max(0);
        let post_slotend = (t_done - slotend).max(0);
        log::info!(
            "WAV[{wav_idx}] p1={n_pass1} ready={n_ready} defer={n_deferred} dec={} \
             tail_win={}us coarse={}us early={}us tail_use={}us post_slotend={}us \
             slot_wait={}us late={}us",
            results.len(),
            tail_window,
            coarse_us,
            t_early_done - t_coarse_done,
            tail_use,
            post_slotend,
            t_slot_recv - t_early_done,
            t_done - t_slot_recv,
        );
        slot_seq = slot_seq.wrapping_add(1);

        for r in results.iter() {
            mfsk_app_shared::time_sync::record_decode_dt(r.dt_sec);
        }
        mfsk_app_shared::time_sync::finalize_slot();
        if let Some(off) = mfsk_app_shared::time_sync::slot_dt_offset() {
            log::info!(
                "  median DT = {:+.3} s ({} slots)",
                off,
                mfsk_app_shared::time_sync::slots_finalised()
            );
        }

        let mut had_response_this_slot = false;
        if let Ok(mut ui) = UI.lock() {
            for r in results.iter() {
                if let Some(text) = unpack77(&r.message77) {
                    let mut msg: heapless::String<22> = heapless::String::new();
                    let take = text.len().min(msg.capacity());
                    let _ = msg.push_str(&text[..take]);
                    const FP_SPEC_SHIFT: u32 = 12;
                    let cell_scale = (1u32 << FP_SPEC_SHIFT) as f32;
                    let calibrated_snr =
                        mfsk_core::ft8::decode_block::xsnr2_db_simple(&spec.spec, r, cell_scale);
                    let snr_i8 = calibrated_snr.round().clamp(-128.0, 127.0) as i8;
                    let row = DecodedRow {
                        df_hz: r.freq_hz.round().clamp(0.0, 65_535.0) as u16,
                        snr_db: snr_i8,
                        hard_errors: r.hard_errors.min(255) as u8,
                        msg,
                        slot_seq,
                        first_seq: slot_seq,
                    };
                    ui.push_decode(row);
                    log::info!(
                        "{:4.0}Hz {:+5.1}dB (raw={:+5.1}) {}",
                        r.freq_hz, calibrated_snr, r.snr_db, text
                    );
                    qso.set_rx_snr(snr_i8);
                    let parity_lock_ok =
                        mfsk_app_shared::parity::framing_settled_for_parity_lock();
                    if qso
                        .process_message(&text, wav_idx as u32, parity_lock_ok)
                        .is_some()
                    {
                        had_response_this_slot = true;
                    }
                }
            }
        }

        if !had_response_this_slot && qso.state != QsoState::Idle {
            let _ = qso.on_period_end();
        }
        if qso.state == QsoState::Idle {
            qso.call_cq(None);
        }
        let intent = qso.next_tx();
        push_tx_line(&qso, intent.as_ref());
    }
}

fn push_tx_line(qso: &QsoManager, intent: Option<&qso::TxIntent>) {
    let line = qso::format_tx_line(qso, intent);
    log::info!("[QSO] {}", line.as_str());
    if let Ok(mut ui) = UI.lock() {
        ui.set_tx_line(line.as_str());
    }
}

fn wf_drain(wf_q: esp_idf_svc::sys::QueueHandle_t) -> ! {
    loop {
        let tick = pipeline::recv_box::<pipeline::WfTick>(wf_q);
        if let Ok(mut ui) = UI.lock() {
            ui.push_waterfall(tick.row);
        }
    }
}
