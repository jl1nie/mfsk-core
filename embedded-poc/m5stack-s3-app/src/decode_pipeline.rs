//! WAV-fed decode pipeline (Phase 3).
//!
//! `embedded_shared::apps::rx_wavsim::run` 相当を本クレート内に inline:
//! - EspLogger init は外す (FanoutLogger と競合するため main で先に install 済)
//! - 結果は `log::info!` で吐き、FanoutLogger 経由で LCD scroll panel に流れる
//! - Phase 3 の次イテレーションで `tx_picker::OccupancyMap` / `snr_norm::NoiseFloorTracker`
//!   への ingest を追加し、専用 UI region に流し込む

extern crate alloc;

use alloc::vec::Vec;
use mfsk_core::core::sync::SyncCandidate;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{DEFAULT_Q_THRESH, NFFT_SPEC};

#[allow(unused_imports)]
use mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
use mfsk_core::msg::wsjt77::unpack77;
use mfsk_ft8::mfsk_ft8_basis_scratch_len;

use embedded_shared::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};

use crate::qso::{self, QsoManager, QsoState};
use crate::ui::state::{DecodedRow, UI};

/// Phase 4 dry-run: own callsign + Maidenhead grid burned into the
/// build. Phase 6 will replace this with `/littlefs/config.toml`.
const MY_CALL: &str = "JL1NIE";
const MY_GRID: &str = "PM95";

/// 開発用 WAV (qso3_busy のみ単独ループ — UI 構築用に最も多くのデコード結果を出す)。
static QSO_WAVS: &[&[u8]] = &[
    include_bytes!("../../assets/qso3_busy.wav"),
];

const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;

/// 別スレッドから呼ぶ。返らない。
///
/// Phase 0.7c: BASIS buffers (60 KB × 4) are allocated **inside this
/// thread** instead of main, because the thread's 32 KB stack must be
/// reserved while the largest free DRAM block is still ~139 KB. If
/// main allocated BASIS first the post-alloc largest dropped to
/// ~31 KB and `pthread_create` for this thread returned ENOMEM. The
/// per-pair lifetime is the same as the program; the buffers are
/// never freed (`heap_caps_aligned_alloc` + Box::leak via slice
/// `'static mut`).
pub fn run() -> ! {
    let need = mfsk_ft8_basis_scratch_len();
    assert!(BASIS_SCRATCH_LEN >= need, "BASIS_SCRATCH_LEN too small");

    crate::log_free_internal("pre-basis-alloc");
    let basis_re_main = crate::alloc_basis_dram("RE_main");
    let basis_im_main = crate::alloc_basis_dram("IM_main");
    let basis_re_c1 = crate::alloc_basis_dram("RE_c1");
    let basis_im_c1 = crate::alloc_basis_dram("IM_c1");
    crate::log_free_internal("post-basis-alloc");
    let basis_re_c1_ptr = basis_re_c1.as_mut_ptr();
    let basis_im_c1_ptr = basis_im_c1.as_mut_ptr();

    // wav_sim (4) / stage1_inc (3) より高い優先度。
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 6);
    }

    esp_dsp_fft::prewarm(NFFT_SPEC);
    dual_core::init(basis_re_c1_ptr, basis_im_c1_ptr);

    let chunk_q = pipeline::create_chunk_queue(4);
    let slot_q = pipeline::create_slot_queue(2);
    let spec_q = pipeline::create_spec_queue(2);
    // Streaming-WF tick queue. Depth 8 = ~640 ms buffering at the
    // 80 ms per-pair cadence; if the UI drainer falls behind beyond
    // that the oldest ticks are simply dropped (try_send_box).
    let wf_q = pipeline::create_wf_queue(8);
    stage1_inc::spawn_with_wf(chunk_q, slot_q, spec_q, Some(wf_q));
    wav_sim::spawn(QSO_WAVS, chunk_q);

    // Spawn a tiny drainer that forwards WfTicks to the shared
    // `UiState::waterfall` ring. Lives in its own thread so the
    // decode loop's blocking `recv_box::<SpecBundle>` doesn't gate
    // the WF cadence. `QueueHandle_t` is `*mut QueueDefinition` which
    // isn't `Send`; pass it as `usize` and re-cast inside the thread.
    let wf_q_addr = wf_q as usize;
    std::thread::Builder::new()
        .stack_size(4 * 1024)
        .spawn(move || wf_drain(wf_q_addr as esp_idf_svc::sys::QueueHandle_t))
        .expect("spawn wf drainer");

    log::info!("decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz, build=v0.6.2)");

    // QSO FSM (Phase 4 dry-run). auto-CQ kicks off immediately so the
    // very first slot already has a TX intent painted; no decode in
    // qso3_busy.wav contains "JL1NIE" so the FSM stays in Calling and
    // exercises the retry → reset → re-CQ loop visibly on the LCD.
    let mut qso = QsoManager::new(MY_CALL, MY_GRID);
    let initial = qso.call_cq(None);
    push_tx_line(&qso, Some(&initial));

    let mut slot_seq: u32 = 0;
    loop {
        // 広帯域受信 (200..3000 Hz)。phantom (qso3_busy で 3/7 件) は
        // UI に "?" 付きで表示するが、QSO FSM / TX picker は
        // multi-slot persistence test (連続 2-of-3 slot で同 callsign-pair
        // 検出) で gate するため自動応答に流れない。設計の根拠は host 側
        // 実測: 200..2000 で phantom 0 だが 2 kHz 超の real (qso1/qso2 R6WA)
        // を逃す → ユーザを "聞こえてる気"にさせない方が大事。
        let spec = pipeline::recv_box::<pipeline::SpecBundle>(spec_q);
        let pass1: Vec<SyncCandidate> = dual_core::coarse_sync_split_with_allsum(
            &spec.spec,
            100.0,
            3_000.0,
            1.0,
            PASS1_LIMIT,
            &spec.allsum_head,
            &spec.allsum_tail,
        );
        // Don't drop the spec yet — `xsnr2_db_simple` reads it after
        // stage 3 completes to recompute SNR free of the per-block
        // auto-gain bias `compute_snr_db` carries. Spec is ~360 KB
        // for the slot, lives in PSRAM, dropped at the end of this
        // loop iteration.

        let slot = pipeline::recv_box::<pipeline::Slot>(slot_q);
        let wav_idx = slot.wav_idx;
        let n_pass1 = pass1.len();

        // (fine_refine attempted via fill_symbol_spectra iteration in
        // 0.6.3-experimental; tripped task watchdog at ~5 s on S3 due
        // to 200k+ per-symbol DFTs per slot. Reverted; ship recall
        // stays at 6/18 on qso3_busy.wav until a cd0-via-FIR-decimate
        // refine lands.)

        // The audio gate around pass2/stage3 was added to silence the
        // DMA-underrun buzz the user heard during BP work, but the
        // *resume* transition produced a much harsher click — the
        // I2S driver / codec analog stage settles dirty after a
        // 1.3 s starvation and pops on first non-zero sample. Net
        // assessment: leave audio streaming the whole time. The
        // residual buzz at -10 dB DAC volume is mild background
        // texture; the click was an alarm.
        // crate::audio::AUDIO_GATE.store(false, ...);  // intentionally not muted

        let pass2 = dual_core::pass2_split(
            &slot.audio,
            pass1,
            MAX_CAND,
            basis_re_main,
            basis_im_main,
        );

        let depth = DecodeDepth::BpAll;
        let results = dual_core::stage3_split(
            &slot.audio,
            pass2,
            depth,
            DEFAULT_Q_THRESH,
            mfsk_core::ft8::params::DEFAULT_BP_MAX_ITER,
            basis_re_main,
            basis_im_main,
        );

        log::info!("WAV[{wav_idx}] p1={n_pass1} dec={}", results.len());
        slot_seq = slot_seq.wrapping_add(1);

        // Feed every result's DT into the slot's median estimator,
        // then finalise. The estimate is the device's local-clock
        // offset relative to the band consensus — used by Phase 4
        // QSO FSM as a fallback time source when no GPS is wired.
        for r in results.iter() {
            crate::time_sync::record_decode_dt(r.dt_sec);
        }
        crate::time_sync::finalize_slot();
        if let Some(off) = crate::time_sync::slot_dt_offset() {
            log::info!(
                "  median DT = {:+.3} s ({} slots)",
                off,
                crate::time_sync::slots_finalised()
            );
        }
        // Push every CRC-passing decode to the UI ring. WF rows are
        // streamed separately by the `wf_drain` task at per-pair
        // cadence (~80 ms) so this loop only handles decode results.
        // The QSO FSM is also fed here so its state matches the slot
        // boundary (first message → state advance → retry budget).
        let mut had_response_this_slot = false;
        if let Ok(mut ui) = UI.lock() {
            for r in results.iter() {
                if let Some(text) = unpack77(&r.message77) {
                    let mut msg: heapless::String<22> = heapless::String::new();
                    let take = text.len().min(msg.capacity());
                    let _ = msg.push_str(&text[..take]);
                    // xsnr2-based SNR (mfsk-core 0.5.7+) — slot-wide
                    // baseline (median over a ±156 Hz window) gives
                    // a noise floor independent of the per-Costas-block
                    // auto-gain that contaminates `compute_snr_db`,
                    // so the dB number is comparable across signals
                    // AND comparable to WSJT-X / JTDX (±3 dB on the
                    // qso3_busy reference). `cell_scale` reverts the
                    // embedded fixed-point FP_SPEC_SHIFT (= 12).
                    const FP_SPEC_SHIFT: u32 = 12;
                    let cell_scale = (1u32 << FP_SPEC_SHIFT) as f32;
                    let calibrated_snr =
                        mfsk_core::ft8::decode_block::xsnr2_db_simple(&spec.spec, r, cell_scale);
                    let snr_i8 = calibrated_snr.round().clamp(-128.0, 127.0) as i8;
                    // `first_seq` is provisionally `slot_seq`; if the
                    // msg is already in the ring `push_decode` will
                    // overwrite this with the existing entry's seq so
                    // recurring callsigns don't re-flash the highlight.
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
                    // Feed the FSM. SNR set first so an in-band reply
                    // ("JL1NIE W1AW FN31") gets reported with the
                    // correct rx_snr → tx_report.
                    qso.set_rx_snr(snr_i8);
                    if qso.process_message(&text).is_some() {
                        had_response_this_slot = true;
                    }
                }
            }
        }

        // Slot boundary. If no decode addressed us, retry — and if the
        // retry budget is gone, the FSM resets to Idle and we
        // immediately re-CQ so the dry-run keeps animating between
        // states instead of parking in Idle indefinitely.
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

/// Format the FSM's current intent and push it to the UI thread. Also
/// echoed to the log so headless captures still record QSO state.
fn push_tx_line(qso: &QsoManager, intent: Option<&qso::TxIntent>) {
    let line = qso::format_tx_line(qso, intent);
    log::info!("[QSO] {}", line.as_str());
    if let Ok(mut ui) = UI.lock() {
        ui.set_tx_line(line.as_str());
    }
}

/// Drain `WfTick`s from stage1_inc and forward each to
/// `UiState::waterfall`. Runs in its own thread because the decode
/// loop blocks ~14.8 s on `recv_box::<SpecBundle>`; piggybacking the
/// WF cadence on that loop would re-introduce the 15 s freeze the
/// streaming WF was added to fix.
///
/// `WfTick::row` is already palette-indexed (0..15) by stage1_inc, so
/// this loop is just a queue-receive + ring push under the UI mutex.
fn wf_drain(wf_q: esp_idf_svc::sys::QueueHandle_t) -> ! {
    loop {
        let tick = pipeline::recv_box::<pipeline::WfTick>(wf_q);
        if let Ok(mut ui) = UI.lock() {
            // `WfTick::row` len == `ui::state::WfLine` len (both 135).
            ui.push_waterfall(tick.row);
        }
    }
}
