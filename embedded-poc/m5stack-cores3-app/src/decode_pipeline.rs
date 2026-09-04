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

/// Operator identity, from `cfg.toml`'s `[station]` section.
///
/// These were literals until 2026-08-23 — this repository shipped one
/// operator's callsign compiled into the source, so anyone else's build
/// identified as them. `m5stack-s3-app` has taken them from `cfg.toml`
/// since Phase 1.7; this crate was never brought across.
///
/// Empty is a valid value and leaves the QSO FSM idle, which is the
/// right behaviour for a receiver with no operator configured.
const MY_CALL: &str = env!("MY_CALL");
const MY_GRID: &str = env!("MY_GRID");

static QSO_WAVS: &[&[u8]] = &[include_bytes!("../../assets/qso3_busy.wav")];

const PASS1_LIMIT: usize = 30;
const MAX_CAND: usize = 15;

/// Wall-clock budget for stage 3 (BP/OSD), milliseconds from the
/// SpecBundle arriving. `0` = no deadline, the historical behaviour
/// (#357).
///
/// Compile-time knob for the qso3_busy degradation sweep: rebuild with
/// `MFSK_FT8_BUDGET_MS=2500 cargo build ...` and read `cut=` / `dec=` /
/// `post_slotend` / `slot_wait` off the per-slot log. Default `0` so a
/// plain build is a clean baseline.
const FT8_BUDGET_MS: i64 = match option_env!("MFSK_FT8_BUDGET_MS") {
    Some(s) => parse_i64(s),
    None => 0,
};

/// `const`-context `i64` parse — `str::parse` is not `const`. Digits
/// only; anything else is a build-time panic, which is what you want
/// for a typo in an env var that would otherwise silently disable the
/// budget.
const fn parse_i64(s: &str) -> i64 {
    let b = s.as_bytes();
    let mut i = 0;
    let mut v: i64 = 0;
    while i < b.len() {
        assert!(b[i] >= b'0' && b[i] <= b'9', "MFSK_FT8_BUDGET_MS: digits only");
        v = v * 10 + (b[i] - b'0') as i64;
        i += 1;
    }
    v
}

/// `BootMode::Decode` entry — runs the decode pipeline with the baked
/// `QSO_WAVS` playlist as the audio source. Thin wrapper around
/// [`run_with_source`].
pub fn run() -> ! {
    run_with_source("wav", |q| wav_sim::spawn(QSO_WAVS, q))
}

/// Source-agnostic entry. Allocates the pipeline queues, spawns
/// `stage1_inc` + `wf_drain`, calls `source_spawn` (which must push
/// `ChunkMsg::Samples` + `ChunkMsg::SlotEnd` into the chunk queue),
/// then runs the decode loop. Never returns.
///
/// `Decode` mode passes `|q| wav_sim::spawn(QSO_WAVS, q)`.
/// `Uac` mode passes `|q| uac::set_chunk_q(q)` — the UAC reader thread
/// starts pushing once it sees the queue handle land in its static slot.
pub fn run_with_source<F: FnOnce(QueueHandle_t)>(source: &'static str, source_spawn: F) -> ! {
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
    crate::board::spawn_named(c"wf_drain", 4 * 1024, move || {
        wf_drain(wf_q_addr as esp_idf_svc::sys::QueueHandle_t)
    })
    .expect("spawn wf drainer");

    log::info!("decode pipeline ready (q_thresh={DEFAULT_Q_THRESH}, band 200..3000 Hz, cores3-app phase 0)");

    let mut qso = QsoManager::new(MY_CALL, MY_GRID);
    let initial = qso.call_cq(None);
    push_tx_line(&qso, Some(&initial));

    let mut slot_seq: u32 = 0;
    // Air-sync lock strength (#356): the decode count of the slot the
    // current slot-shift estimate came from. A later slot only overrides
    // it with strictly more decodes, so one noisy decode cannot pull a
    // good lock off. Only used for the live (`uac`) source.
    let mut best_n: usize = 0;
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
            budget_ms: FT8_BUDGET_MS,
        };
        let out = dual_core::run_speculative_slot(spec_q, slot_q, &cfg);
        let dual_core::SpeculativeOut {
            spec,
            slot,
            results,
            n_pass1,
            n_cut,
            n_ready,
            n_deferred,
            bootstrap_dt_med,
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
        // `slot_wait` near zero means the pipeline never got to wait for
        // the next slot — it was still working when the slot ended.
        //
        // On FT8 that is an operating limit rather than a fault: 15 s
        // is genuinely tight, `coarse` scales with how much signal is
        // on the band, and stations transmit in alternating periods, so
        // the busier of the two runs out of time and defers candidates.
        // Measured 2026-08-23 on 40 m: seven decodes on one period,
        // one or two on the other, with `coarse` at 101 ms against
        // 180 ms. Worth surfacing precisely because it is expected —
        // the alternative is reading it out of one field in seven.
        //
        // WSPR and FST4 are the other case: their monitor loops are
        // built with deliberate slack, so an over-budget slot there
        // means a fault. Do not carry this framing across.
        let slot_wait_us = t_slot_recv - t_early_done;
        // Only judge a full slot.
        //
        // The first slots after the grid anchors to UTC are partial by
        // construction — the anchor shortens the current one so the
        // next boundary lands on the grid — and comparing a truncated
        // slot against a whole slot's budget produces a warning about
        // nothing. Two of them fired within seconds of adding this,
        // which is exactly how an indicator teaches its reader to skip
        // it.
        const FULL_SLOT_SAMPLES: usize = 180_000;
        let full_slot = slot.audio().len() >= FULL_SLOT_SAMPLES;
        if full_slot && slot_wait_us < 10_000 {
            log::warn!(
                "SLOT[{wav_idx}] src={source} OVER BUDGET — no idle before the next slot \
                 ({post_slotend} us past slot end, {n_deferred} candidates deferred)"
            );
        }
        log::info!(
            "SLOT[{wav_idx}] src={source} p1={n_pass1} ready={n_ready} defer={n_deferred} \
             cut={n_cut} dec={} budget={FT8_BUDGET_MS}ms \
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
        let n_dec = results.len();
        // This slot's median, or `None` if it produced no decodes — in
        // which case `finalize_slot` kept the previous estimate and
        // `slot_dt_offset` would report *that*, which is not this slot.
        let slot_median = if n_dec > 0 {
            mfsk_app_shared::time_sync::slot_dt_offset()
        } else {
            None
        };
        if let Some(off) = mfsk_app_shared::time_sync::slot_dt_offset() {
            log::info!(
                "  median DT = {:+.3} s ({} slots)",
                off,
                mfsk_app_shared::time_sync::slots_finalised()
            );
        }

        // Self-align the slot grid from the air (#356), for the live
        // source before NTP has disciplined the clock. `Ft8ChunkSink`
        // hands the phase to the UTC drift check the moment
        // `clock_is_disciplined()` turns true; on a hilltop with no
        // network it never does, and the grid would otherwise free-run
        // at a phase uniform over 15 s against a mode that tolerates
        // ±2.5 s. Coarse sync's own DT — the confirmed-decode median
        // once decodes exist, the top-5 candidate median
        // (`bootstrap_dt_med`) before then — is a time reference present
        // wherever the receiver is, posted through
        // `set_bootstrap_slot_shift_12k`.
        //
        // Gated on `!clock_is_disciplined()` so that once NTP is up this
        // whole block — the median read, the shift maths, the log — does
        // not run at all, rather than computing a correction
        // `Ft8ChunkSink` would only drain. The `wav` source defines its
        // own boundaries and is never touched.
        if source == "uac" && !mfsk_app_shared::time_sync::clock_is_disciplined() {
            /// Already inside FT8's tolerance — post 0 rather than chase
            /// jitter into the shorten direction stage1_inc dislikes.
            const ALIGN_OK_SEC: f32 = 0.05;
            /// Per-slot shift cap. ±0.2 s keeps the slot in
            /// [177_600, 182_400] so stage1_inc still completes pair 91;
            /// a larger offset converges over several slots.
            const MAX_SHIFT_SAMPLES: i32 = 2_400;
            /// A slot this far out, with more decodes than the lock,
            /// drops the lock — the StickS3 auto-sync's Tier 1, without
            /// the quiet-band persistence net.
            const DRIFT_RESET_SEC: f32 = 0.5;
            const DRIFT_RESET_MIN_N: usize = 3;

            let to_samples = |dt: f32| -> i32 {
                let s = if dt.abs() < ALIGN_OK_SEC {
                    0
                } else {
                    (dt * 12_000.0).round() as i32
                };
                s.clamp(-MAX_SHIFT_SAMPLES, MAX_SHIFT_SAMPLES)
            };

            if let Some(m) = slot_median {
                if best_n > 0
                    && n_dec > best_n
                    && n_dec >= DRIFT_RESET_MIN_N
                    && m.abs() > DRIFT_RESET_SEC
                {
                    log::info!("  air-sync: drift (DT={m:+.3}s, N={n_dec}>{best_n}) — lock reset");
                    best_n = 0;
                }
            }

            let cold_bootstrap = bootstrap_dt_med.filter(|_| n_dec == 0 && best_n == 0);
            if let Some(m) = slot_median.filter(|_| n_dec > best_n) {
                let shift = to_samples(m);
                mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(shift);
                // Once converged, `n_dec` keeps setting new highs on a
                // busy band with `shift` at 0 — say nothing then.
                if shift != 0 {
                    log::info!("  air-sync: lock DT={m:+.3}s (N={n_dec}) → {shift:+} samples");
                }
                best_n = n_dec;
            } else if let Some(m) = cold_bootstrap {
                let shift = to_samples(m);
                mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(shift);
                log::info!(
                    "  air-sync: cold bootstrap DT={m:+.3}s (p1={n_pass1}, top-5 coarse) → {shift:+} samples"
                );
            } else {
                // Locked and this slot did not beat it, or nothing to go
                // on — post 0 so a previous slot's hint is not re-applied.
                mfsk_app_shared::time_sync::set_bootstrap_slot_shift_12k(0);
            }
        }

        let mut had_response_this_slot = false;
        if let Ok(mut ui) = UI.lock() {
            for r in results.iter() {
                if let Some(text) = unpack77(r.message77()) {
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
                        r.freq_hz,
                        calibrated_snr,
                        r.snr_db,
                        text
                    );
                    qso.set_rx_snr(snr_i8);
                    let parity_lock_ok = mfsk_app_shared::parity::framing_settled_for_parity_lock();
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
    let n: u32 = 0;
    loop {
        let tick = pipeline::recv_box::<pipeline::WfTick>(wf_q);
        if let Ok(mut ui) = UI.lock() {
            ui.push_waterfall(tick.row);
        }
        let _ = n;
    }
}
