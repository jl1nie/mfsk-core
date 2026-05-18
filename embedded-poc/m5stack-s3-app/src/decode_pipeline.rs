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

use mfsk_core::ft8::decode_block::BASIS_SCRATCH_LEN;
use mfsk_core::msg::wsjt77::unpack77;
use mfsk_ft8::mfsk_ft8_basis_scratch_len;

use embedded_shared::{dual_core, esp_dsp_fft, pipeline, stage1_inc, wav_sim};

use mfsk_app_shared::qso::{self, QsoManager, QsoState};
use mfsk_app_shared::ui::state::{DecodedRow, UI};

/// Fallback identity if the caller doesn't provide a shared QSO
/// instance. Used by `run()` (wav_sim test path) only — production
/// modes pass `qso` via `run_with_source_qso`. Empty in production
/// means cfg.toml didn't supply MY_CALL.
const MY_CALL: &str = env!("MY_CALL");
const MY_GRID: &str = env!("MY_GRID");

/// 開発用 WAV (qso3_busy のみ単独ループ — UI 構築用に最も多くのデコード結果を出す)。
static QSO_WAVS: &[&[u8]] = &[include_bytes!("../../assets/qso3_busy.wav")];

const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;

/// `BootMode::Decode` entry — runs the decode pipeline with the
/// baked `QSO_WAVS` playlist as the audio source (via `wav_sim`).
/// Thin wrapper around [`run_with_source`].
pub fn run() -> ! {
    run_with_source(|chunk_q| wav_sim::spawn(QSO_WAVS, chunk_q));
}

/// Source-agnostic entry. Allocates BASIS scratch, brings up
/// `esp_dsp_fft` + `dual_core`, creates the chunk / slot / spec / wf
/// queues, spawns `stage1_inc` + `wf_drain`, calls `source_spawn`
/// (which must spawn the task that pushes `ChunkMsg::Samples` +
/// `ChunkMsg::SlotEnd` into the chunk queue), then runs the decode
/// loop. Never returns.
///
/// `Decode` mode passes `|q| wav_sim::spawn(QSO_WAVS, q)`.
/// `Uac` mode passes `|q| uac::set_chunk_q(q)` — the UAC reader
/// thread is already running by then and starts pushing once it
/// sees the queue handle land in its static slot. The closure runs
/// **after** `stage1_inc` is up so the source's first push doesn't
/// race against an unprimed consumer.
///
/// Phase 0.7c: BASIS buffers (60 KB × 4) are allocated **inside this
/// thread** instead of main, because the thread's 32 KB stack must be
/// reserved while the largest free DRAM block is still ~139 KB. If
/// main allocated BASIS first the post-alloc largest dropped to
/// ~31 KB and `pthread_create` for this thread returned ENOMEM. The
/// per-pair lifetime is the same as the program; the buffers are
/// never freed (`heap_caps_aligned_alloc` + Box::leak via slice
/// `'static mut`).
pub fn run_with_source<F>(source_spawn: F) -> !
where
    F: FnOnce(esp_idf_svc::sys::QueueHandle_t),
{
    // Back-compat shim — wav_sim path uses a local FSM, no shared
    // QSO ownership needed.
    run_with_source_qso(source_spawn, None)
}

/// Variant that accepts a shared `QsoManager` so the FSM state is
/// the same one the TX scheduler (or capture_tx_thread) reads.
/// `BootMode::Qso` and other production modes call this directly;
/// `BootMode::Decode` (wav_sim) uses [`run_with_source`].
pub fn run_with_source_qso<F>(
    source_spawn: F,
    qso_shared: Option<std::sync::Arc<std::sync::Mutex<QsoManager>>>,
) -> !
where
    F: FnOnce(esp_idf_svc::sys::QueueHandle_t),
{
    // Phase 1.7.7-Stick: BASIS scratch retired — `mfsk-core` now uses
    // Goertzel (zero internal-DRAM scratch) for both pass-2
    // (`refine_candidates_into`) and stage-3
    // (`process_candidates_into_with_cs_scratch_tuned`) cs builds.
    // The 4 × 30 KB = 120 KB internal DRAM that used to live here
    // (one `BASIS_SCRATCH_LEN` i16 buffer per (re/im, main/worker
    // dual_core)) is now free for Qso-mode I2S bidir DMA descriptors
    // (the `i2s_alloc_dma_desc` largest=7680 KB requirement that
    // tripped on the old layout).
    //
    // `dual_core::pass2_split` / `stage3_split` and `dual_core::init`
    // still take `basis_re` / `basis_im` slice / pointer parameters
    // for API compat — they're unused inside mfsk-core but threaded
    // through the dispatcher boilerplate. Pass empty slices / null
    // pointers; mfsk-core no longer dereferences them.
    let _ = mfsk_ft8_basis_scratch_len();
    let _ = BASIS_SCRATCH_LEN;
    crate::log_free_internal("pre-decode-loop (post-Goertzel: no BASIS alloc)");
    let basis_re_main: &'static mut [i16] = alloc::boxed::Box::leak(alloc::vec![].into_boxed_slice());
    let basis_im_main: &'static mut [i16] = alloc::boxed::Box::leak(alloc::vec![].into_boxed_slice());
    let basis_re_c1_ptr: *mut i16 = core::ptr::null_mut();
    let basis_im_c1_ptr: *mut i16 = core::ptr::null_mut();

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
    // The source closure (wav_sim for `Decode`, UAC-reader hookup
    // for `Uac`) spawns its task here, after stage1_inc is up so
    // the first push doesn't race the consumer.
    source_spawn(chunk_q);

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

    log::info!(
        "decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz, build=v0.6.2)"
    );

    // QSO FSM. Production modes (Acoustic / Qso) pass a shared
    // `Arc<Mutex<QsoManager>>` so the TX scheduler / menu UI see the
    // same state. Back-compat (wav_sim) creates a local one with
    // auto-CQ pre-enabled so the dry-run loop keeps animating.
    let qso: std::sync::Arc<std::sync::Mutex<QsoManager>> = qso_shared.unwrap_or_else(|| {
        let mut q = QsoManager::new(MY_CALL, MY_GRID);
        q.set_auto_cq(true); // wav_sim test: keep TX strip lively
        std::sync::Arc::new(std::sync::Mutex::new(q))
    });
    // Initial CQ if configured + auto-CQ on.
    {
        let mut q = qso.lock().expect("qso lock");
        if q.auto_cq_enabled && !q.my_call.is_empty() && q.state == QsoState::Idle {
            let initial = q.call_cq(None);
            push_tx_line(&q, Some(&initial));
        } else {
            push_tx_line(&q, None);
        }
    }

    let mut slot_seq: u32 = 0;
    // Phase 1.7.4-Stick v3 (2026-05-17): global-clock-anchored sync.
    //
    // S3 codec sample clock is rock-stable (ppm-level drift) and is
    // therefore the global clock. Once we've locked the phase via
    // (1) BtnA coarse + (2) the highest-N slot's median DT, every
    // subsequent slot is exactly 180_000 samples = 15 s apart with
    // zero per-slot feedback. The auto-sync loop's only job is to
    // apply a ONE-SHOT correction when a better reference (higher N)
    // appears.
    //
    // The phase correction is always non-negative (= LENGTHEN current
    // slot, never SHORTEN) because stage1_inc requires the full 180 k
    // samples to complete all 92 FFT pairs. For negative DT we apply
    // (15 s + DT) — i.e. skip forward by ~one slot period — sacrificing
    // one slot's decode in exchange for clean alignment afterwards.
    let mut best_n: usize = 0;
    // Phase 1.7.6 tiered drift detection state (2026-05-17):
    //   drift_streak  — number of consecutive slots whose median DT
    //                   exceeds DRIFT_STREAK_SEC with the same sign
    //                   AND N ≥ 2. Resets on sign flip, |DT| below
    //                   threshold, or N < 2. ≥ 3 fires Tier-2 reset.
    //   last_drift_sign — sign of the previous qualifying slot's DT
    //                   (+1 / -1; 0 = none).
    let mut drift_streak: u8 = 0;
    let mut last_drift_sign: i8 = 0;
    let mut sync_gen_observed: u32 =
        crate::audio::SYNC_RESET_GEN.load(core::sync::atomic::Ordering::Acquire);
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
        // Auto-sync uses `results` median (post BP), not pass1 — pass1
        // candidates from a noise slot are random and would mis-anchor
        // the slot phase. Gemini PR #103 review removed the no-op
        // `pass1.iter().map(c.dt_sec).collect()` allocation that was
        // immediately discarded below.

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

        let t_pass2 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
        let pass2 =
            dual_core::pass2_split(&slot.audio, pass1, MAX_CAND, basis_re_main, basis_im_main);
        let t_stage3 = unsafe { esp_idf_svc::sys::esp_timer_get_time() };

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
        let t_done = unsafe { esp_idf_svc::sys::esp_timer_get_time() };

        log::info!(
            "WAV[{wav_idx}] p1={n_pass1} dec={} pass2={}us stage3={}us total={}us",
            results.len(),
            t_stage3 - t_pass2,
            t_done - t_stage3,
            t_done - t_pass2
        );
        slot_seq = slot_seq.wrapping_add(1);
        // Publish to UiState so decoded_list renders GREEN only for
        // rows whose `slot_seq == latest_slot_seq` — fixes the
        // "all rows green forever after audio stops" UI bug.
        if let Ok(mut ui) = UI.lock() {
            ui.latest_slot_seq = slot_seq;
            ui.bump_menu();
        }

        // Phase 1.7.4-Stick v3 global-clock-anchor auto-sync (2026-05-17).
        //
        // BtnA-gen check happens HERE (just before HWM update), not at
        // top of loop. The previous-iteration top-of-loop check missed
        // resets that landed while we were blocked in `recv_box`, so
        // the first post-BtnA decode would set HWM then have it cleared
        // on the next iteration — undoing the lock.
        let cur_gen = crate::audio::SYNC_RESET_GEN.load(core::sync::atomic::Ordering::Acquire);
        if cur_gen != sync_gen_observed {
            log::info!(
                "  auto-sync: BtnA generation {sync_gen_observed} → {cur_gen}, HWM cleared (was best_n={best_n})"
            );
            best_n = 0;
            sync_gen_observed = cur_gen;
        }

        let n_dec = results.len();
        let post_sync = mfsk_app_shared::time_sync::slots_finalised() > 0 || n_dec > 0;
        // |DT| < ALIGN_OK_SEC → already within FT8 ±2.5 s tolerance
        //   AND well inside stage1_inc's tolerance for shorten without
        //   losing pair 91 (= 177_600 samples needed). Post shift=0.
        // |DT| ≥ ALIGN_OK_SEC → post a SIGNED shift, capped at
        //   ±MAX_SHIFT_SAMPLES so:
        //   * +shift (lengthen): stage1_inc truncates tail, full spec ✓
        //   * -shift (shorten): slot still ≥ 177_600 samples → pair 91
        //     just barely completes → full 92/92 spec ✓
        //   Larger drifts converge over several HWM-update slots.
        const ALIGN_OK_SEC: f32 = 0.05;
        const MAX_SHIFT_SAMPLES: i32 = 2400; // ±0.2 s — pair 91 still completes
                                             // Tiered drift detection (Phase 1.7.6 Strategy D, 2026-05-17):
                                             //
                                             // Tier 1 — HIGH-TRUST IMMEDIATE: a slot more trustworthy than
                                             //   the current lock (N > best_n) with statistical confidence
                                             //   (N ≥ 3) and clear drift (|DT| > 0.5 s) triggers immediate
                                             //   reset. Catches "band peaks reveal drift" within one slot.
                                             //
                                             // Tier 2 — PERSISTENCE SAFETY-NET: even on a quiet band
                                             //   where Tier 1 never fires, 3 consecutive slots with N ≥ 2
                                             //   showing |DT| > 0.3 s in the SAME sign direction trigger
                                             //   reset. Same-sign requirement rejects bidirectional noise
                                             //   (random outliers cancel). Catches slow PPM-level drift
                                             //   that accumulates beyond FT8 tolerance over time.
                                             //
                                             // Either tier resets best_n=0; the subsequent HWM-update
                                             // path then re-locks using THIS slot's median.
        const TIER1_DT_SEC: f32 = 0.5;
        const TIER1_MIN_N: usize = 3;
        const TIER2_DT_SEC: f32 = 0.3;
        const TIER2_MIN_N: usize = 2;
        const TIER2_STREAK: u8 = 3;
        let median: Option<f32> = if n_dec > 0 {
            let mut dts: Vec<f32> = results.iter().map(|r| r.dt_sec).collect();
            dts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            Some(dts[dts.len() / 2])
        } else {
            None
        };
        let mut drift_reset_fired = false;
        // Tier 1
        if best_n > 0 && n_dec > best_n && n_dec >= TIER1_MIN_N {
            if let Some(m) = median {
                if m.abs() > TIER1_DT_SEC {
                    log::info!(
                        "  auto-sync: DRIFT Tier-1 (median DT={:+.3}s > ±{TIER1_DT_SEC}s, N={n_dec}>{best_n}≥{TIER1_MIN_N}), HWM reset",
                        m
                    );
                    best_n = 0;
                    drift_reset_fired = true;
                    drift_streak = 0;
                    last_drift_sign = 0;
                }
            }
        }
        // Tier 2 (persistence) — update streak regardless of Tier 1
        // outcome so the counter stays current; only fire reset if
        // Tier 1 didn't already.
        if !drift_reset_fired {
            if let Some(m) = median {
                if n_dec >= TIER2_MIN_N && m.abs() > TIER2_DT_SEC {
                    let sign = if m >= 0.0 { 1i8 } else { -1i8 };
                    if last_drift_sign == sign {
                        drift_streak = drift_streak.saturating_add(1);
                    } else {
                        drift_streak = 1;
                        last_drift_sign = sign;
                    }
                    if drift_streak >= TIER2_STREAK && best_n > 0 {
                        log::info!(
                            "  auto-sync: DRIFT Tier-2 ({} slots |DT|>±{TIER2_DT_SEC}s same sign, latest DT={:+.3}s N={n_dec}), HWM reset",
                            drift_streak, m
                        );
                        best_n = 0;
                        drift_streak = 0;
                        last_drift_sign = 0;
                    }
                } else {
                    // Slot doesn't meet Tier 2 streak criteria
                    // (|DT| too small OR N too small) → break streak.
                    drift_streak = 0;
                    last_drift_sign = 0;
                }
            } else {
                // 0 decodes — no DT measurement, break streak.
                drift_streak = 0;
                last_drift_sign = 0;
            }
        }
        if n_dec > best_n {
            let med = median.expect("n_dec > 0 implies median is Some");
            let shift_samples: i32 = if med.abs() < ALIGN_OK_SEC {
                0
            } else {
                (med * 12000.0).round() as i32
            };
            let shift_samples = shift_samples.clamp(-MAX_SHIFT_SAMPLES, MAX_SHIFT_SAMPLES);
            mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(shift_samples);
            log::info!(
                "  auto-sync (HWM update N={n_dec}>{best_n}): median DT={:+.3}s → {:+} samples (capped ±{}s)",
                med, shift_samples,
                MAX_SHIFT_SAMPLES as f32 / 12000.0
            );
            best_n = n_dec;
        } else if post_sync {
            mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(0);
            if n_dec > 0 {
                log::info!(
                    "  auto-sync (hold): N={n_dec} ≤ best_n={best_n}, shift=0 (no overwrite)"
                );
            } else {
                log::info!(
                    "  auto-sync (frozen): no decode this slot, shift=0 (best_n={best_n} held)"
                );
            }
        } else {
            log::info!("  auto-sync (cold): no source — BtnA required");
        }

        // Feed every result's DT into the slot's median estimator,
        // then finalise. The estimate is the device's local-clock
        // offset relative to the band consensus — used by Phase 4
        // QSO FSM as a fallback time source when no GPS is wired.
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
                        r.freq_hz,
                        calibrated_snr,
                        r.snr_db,
                        text
                    );
                    // Feed the FSM. SNR set first so an in-band reply
                    // ("JL1NIE W1AW FN31") gets reported with the
                    // correct rx_snr → tx_report.
                    //
                    // `rx_slot = wav_idx` decouples peer-parity locking
                    // from "current slot when decode finished" — even
                    // if BP spilled into the next slot, the FSM still
                    // attributes the message to the slot whose audio
                    // it came from (issue #110).
                    //
                    // `parity_lock_ok` gates the lock on bootstrap
                    // convergence: when DT is still wandering, our
                    // wav_idx parity may not match the band yet, and
                    // locking would freeze us to the wrong parity for
                    // the whole QSO.
                    let parity_lock_ok =
                        mfsk_app_shared::parity::framing_settled_for_parity_lock();
                    if let Ok(mut q) = qso.lock() {
                        q.set_rx_snr(snr_i8);
                        if q.process_message(&text, wav_idx as u32, parity_lock_ok)
                            .is_some()
                        {
                            had_response_this_slot = true;
                        }
                    }
                }
            }
        }

        // Slot boundary. If no decode addressed us, retry — and if the
        // retry budget is gone, the FSM resets to Idle and we
        // immediately re-CQ so the dry-run keeps animating between
        // states instead of parking in Idle indefinitely.
        // Slot boundary processing on the shared FSM. Lock once, do
        // all the state updates, then release before the next iter's
        // expensive decode work runs (so TX scheduler / menu UX aren't
        // starved while BP iterations chew on the next slot).
        if let Ok(mut q) = qso.lock() {
            if !had_response_this_slot && q.state != QsoState::Idle {
                let _ = q.on_period_end();
            }
            // Auto-CQ re-arm only if the gate allows and we have an
            // identity. Production paths leave auto_cq_enabled `false`
            // by default so the FSM doesn't TX until the user opts in
            // from the menu.
            if q.state == QsoState::Idle && q.auto_cq_enabled && !q.my_call.is_empty() {
                q.call_cq(None);
            }
            let intent = q.next_tx();
            push_tx_line(&q, intent.as_ref());
        }
    }
}

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
