//! Phase 1.7-Stick: slot-driven TX scheduler.
//!
//! Polls the global `QsoManager` every 100 ms; when its state advances
//! to a TX-ready intent (CQ / Reply / Report / Final 73), the scheduler:
//!
//!   1. Loads the current band's DF from `UiState::current_df_hz`
//!      (Auto-DF picker output, or 1500 Hz default if uninitialised).
//!   2. Acquires the radio: `civ::set_ptt(true)`, sleep 100 ms for the
//!      transmit chain to settle.
//!   3. Mutes mic capture via `audio::AUDIO_GATE` so the synthesised
//!      audio leaking back into the mic doesn't blow up the
//!      decode_pipeline's auto-sync.
//!   4. Packs the `TxIntent` into a 77-bit message via `pack77`, runs
//!      `tx::synthesize_message77` (≈300 KB PSRAM Vec) and pushes the
//!      samples through `tx::play` (blocks ~12.6 s on the I2S TX DMA).
//!   5. Releases the radio: `civ::set_ptt(false)`, unmute mic, advance
//!      the QSO FSM via `on_period_end()`.
//!
//! TX cadence: WSJT-X-style every slot (4 CQ/min when auto-CQ is on);
//! the FSM enforces internal retry budgets so we don't spam Final 73
//! beyond `FINAL_MAX_RETRIES`.
//!
//! **I2S RX coexistence (BootMode::Qso) is deferred to Phase 1.7.1**.
//! For Phase 1.7 v1, this scheduler is wired into `BootMode::TxTest`
//! only — it owns the I2S TX driver, and there's no RX side. Qso mode
//! (decode + TX same crate) needs either I2S full-duplex (ESP32-S3
//! `i2s_new_channel` with both Rx + Tx under one controller) or a
//! driver-swap mechanism, neither of which is in scope here.

use std::sync::{Arc, Mutex};

use anyhow::Result;
use esp_idf_hal::i2s::{I2sDriver, I2sTx};
use esp_idf_svc::hal::delay::FreeRtos;
use mfsk_app_shared::qso::QsoManager;
use mfsk_app_shared::ui::state::UI;
use mfsk_core::ft8::message::pack77;

/// Spawn the scheduler thread. Consumes the I2S TX driver — the
/// scheduler is the sole writer of the speaker DAC for its lifetime.
/// The shared `QsoManager` is mutated by both the scheduler (via
/// `on_period_end()` after each TX) and the decode_pipeline (via
/// `process_message()` on every decode); the mutex serialises them.
pub fn spawn(i2s: I2sDriver<'static, I2sTx>, qso: Arc<Mutex<QsoManager>>) -> Result<()> {
    std::thread::Builder::new()
        .stack_size(8 * 1024)
        .name("tx_sched".into())
        .spawn(move || scheduler_thread(i2s, qso))
        .map_err(|e| anyhow::anyhow!("tx_scheduler spawn failed: {e}"))?;
    log::info!("tx_scheduler: spawned");
    Ok(())
}

fn scheduler_thread(mut i2s: I2sDriver<'static, I2sTx>, qso: Arc<Mutex<QsoManager>>) {
    // Prio above stage1_inc/dual_core so the I2S DMA refill doesn't
    // starve during BP — same rationale as audio_thread (Phase 1.5
    // capture_thread used prio 8 for the same reason).
    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), 8);
    }
    if let Err(e) = i2s.tx_enable() {
        log::error!("tx_scheduler: i2s tx_enable failed: {e:?}");
        return;
    }
    log::info!("tx_scheduler: i2s enabled; waiting for BLE pair + first slot");
    // BLE pair grace — same 10 s window the Phase 1.6 test loop used.
    FreeRtos::delay_ms(10_000);

    loop {
        // Poll cadence: 100 ms is fast enough to catch any slot-boundary
        // TX trigger within one FT8 symbol period. Not aligned to slot
        // start yet — `time_sync::bootstrap_slot_shift` would let us
        // schedule precisely against the band's slot cadence; for now
        // we rely on the FSM's per-slot state and the fact that auto-CQ
        // fires immediately when next_tx() exposes an intent.
        let maybe_intent = {
            let q = match qso.lock() {
                Ok(g) => g,
                Err(e) => {
                    log::error!("tx_scheduler: qso lock poisoned: {e}");
                    FreeRtos::delay_ms(1000);
                    continue;
                }
            };
            let auto_cq = q.auto_cq_enabled;
            // For Idle state, only fire CQ when auto_cq is enabled.
            // For mid-QSO states (Calling/Report/Final), always honour
            // the next_tx() intent — auto_cq only gates the cold-start
            // Idle → Calling auto-transition.
            let intent = q.next_tx();
            if intent.is_none() && auto_cq && q.state == mfsk_app_shared::qso::QsoState::Idle {
                // Synthesise a fresh CQ by transitioning Idle → Calling.
                drop(q);
                let mut q = match qso.lock() {
                    Ok(g) => g,
                    Err(_) => continue,
                };
                Some(q.call_cq(None))
            } else {
                intent
            }
        };

        let Some(intent) = maybe_intent else {
            FreeRtos::delay_ms(100);
            continue;
        };

        // ── Slot is GO — bracket TX with PTT + mic mute. ───────────
        let df_hz = {
            let ui = UI.lock().ok();
            ui.as_ref().map(|u| u.current_df_hz).unwrap_or(0)
        };
        let df_hz = if df_hz == 0 { 1500u16 } else { df_hz };

        let formatted = intent.formatted();
        log::info!(
            "tx_sched: firing TX → '{}' @ {df_hz} Hz",
            formatted.as_str()
        );

        // Build the 77-bit message. pack77 wants 3 string args
        // (call1, call2, report) — TxIntent stores them separately,
        // and `formatted()` is for display only. Use the raw fields.
        let msg77 = match pack77(intent.call1.as_str(), intent.call2.as_str(), intent.report.as_str()) {
            Some(m) => m,
            None => {
                log::warn!(
                    "tx_sched: pack77 failed for '{}' '{}' '{}' — skipping slot",
                    intent.call1.as_str(),
                    intent.call2.as_str(),
                    intent.report.as_str()
                );
                FreeRtos::delay_ms(100);
                continue;
            }
        };

        // Mute mic so the speaker output doesn't feed back through
        // the ES8311 ADC into decode_pipeline's coarse_sync and
        // bootstrap_slot_shift. AUDIO_GATE is the existing static
        // shared with audio::audio_thread/capture_thread.
        crate::audio::AUDIO_GATE.store(false, std::sync::atomic::Ordering::Release);

        crate::civ::set_ptt(true);
        // Radio settle — IC-705 needs ~100 ms from PTT-on to the
        // transmitter being ready to accept audio (per K7MDL2 +
        // WebFT8 reference).
        FreeRtos::delay_ms(100);

        let samples = match crate::tx::synthesize_message77(&msg77, df_hz as f32) {
            Ok(s) => s,
            Err(e) => {
                log::error!("tx_sched: synth failed: {e:#}");
                crate::civ::set_ptt(false);
                crate::audio::AUDIO_GATE.store(true, std::sync::atomic::Ordering::Release);
                FreeRtos::delay_ms(100);
                continue;
            }
        };

        let t_start = std::time::Instant::now();
        if let Err(e) = crate::tx::play(&mut i2s, &samples) {
            log::error!("tx_sched: play failed: {e:#}");
        }
        log::info!(
            "tx_sched: TX complete in {:?} ({} samples)",
            t_start.elapsed(),
            samples.len()
        );

        crate::civ::set_ptt(false);
        crate::audio::AUDIO_GATE.store(true, std::sync::atomic::Ordering::Release);

        // Advance the FSM — if the peer didn't reply during our TX
        // slot, `on_period_end` increments the retry counter (or
        // resets to Idle when budget is exhausted).
        if let Ok(mut q) = qso.lock() {
            let _ = q.on_period_end();
        }

        // Small slot gap so we don't immediately retransmit if FSM
        // returned another intent on the same iteration. FT8 slots
        // are 15 s — leaving 2 s here is conservative.
        FreeRtos::delay_ms(2_000);
    }
}
