// SPDX-License-Identifier: GPL-3.0-or-later
//! FT4 receiver — the boot mode, taking audio from a radio.
//!
//! The decode side is `embedded_shared::apps::ft4_rx`, shared with the
//! `ft4-demo` bin; this is the board half: the UAC sink, the slot
//! handoff, and the screen.
//!
//! ## What is different from FST4 and WSPR
//!
//! **The coarse stage runs during capture.** `Ft4SavgBuilder`
//! accumulates the periodogram from the audio callback, so what is
//! left after the slot closes is the peak search — 6 ms instead of
//! 761 (`docs/notes/FT4_BENCHMARK.md` §32). That is 754 ms of a
//! 1 960 ms budget that stops being spent, and the budget is what
//! bounds the candidate list rather than being spare headroom: §34's
//! cutoff currently takes 11 decodes down to 9 or 10, and §23 shows
//! the deepest decoding rank tracking the candidate count at every
//! occupancy measured. At ~197 ms a candidate, 754 ms is about four
//! more stations on a crowded band.
//!
//! §33 measured that the accumulation keeps up with the audio: the
//! worst block is 25 % of its own real-time budget at the UAC's
//! ~256-sample read size. It runs in the slot task rather than the
//! callback all the same — `SlotAccum` holds a `Box<dyn Fft>`, which is
//! not `Send`, so it cannot live in a static the way `Fst4Sink`'s
//! staging buffer does. What matters is that the work overlaps
//! capture, not which thread does it; the sink stays a bounded
//! `Vec<i16>` hand-off, exactly as FST4's does.
//!
//! **The screen is the FT8 controller's.** Waterfall, decoded list and
//! status bar all go through `mfsk_app_shared::ui::state::UI` and are
//! drawn by `display::run_log_panel` — the same state and loop the FT8
//! path uses, not a second implementation. What FT4 adds is rate: 152
//! spectrogram rows a slot against FT8's one, handed over by the same
//! hook that feeds the coarse stage, so the waterfall costs no
//! transform of its own.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use embedded_shared::apps::ft4_rx as rx;
use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::ui::state::{DecodedRow, UI};

/// The deadline the candidate loop is held to, from slot close.
///
/// The transceiver budget, even though this receiver does not transmit
/// yet: it is the constraint a QSO-capable build will have, and running
/// it now means the numbers on screen are the ones that will still be
/// true then. `rx::RX_ONLY_BUDGET_MS` is the alternative if a
/// listen-only board should try every candidate.
const BUDGET_MS: i64 = rx::TX_TURNAROUND_BUDGET_MS;

/// Stack for the decode task. `decode_slot` keeps a slot of audio and
/// one `cd0` per candidate, both heap; the frames themselves are
/// modest. Matches what `decode_pipeline` asks for on the FT8 path.
const DECODE_STACK: u32 = 32 * 1024;

/// Raw 12 kHz samples between the audio callback and the slot task.
///
/// Bounded: the task drains it continuously, but decoding a slot blocks
/// it for ~2.4 s, during which capture keeps arriving. 2.4 s is 28 800
/// samples, so [`STAGING_CAP`] holds four seconds and the task catches
/// up inside the remaining five of the slot. Overflow drops and says
/// so — silently discarding audio is how a receiver looks broken for
/// reasons no log explains.
static STAGING: Mutex<Vec<i16>> = Mutex::new(Vec::new());

/// Four seconds of 12 kHz audio.
const STAGING_CAP: usize = 48_000;

static AUDIO_LIVE: AtomicBool = AtomicBool::new(false);
static SLOT_SEQ: AtomicU32 = AtomicU32::new(0);

pub fn run(peripherals: Peripherals, nvs_part: EspDefaultNvsPartition) -> ! {
    log::info!("=== mfsk-core-m5stack-cores3-app ft4-app boot ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);

    // The candidate loop is compute-bound for hundreds of milliseconds
    // with no yield point; IDLE starves and the watchdog fires. Same
    // call and reason as every receiver and bench here.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("ft4_app: task watchdog deinit -> {r}");

    // Before WiFi and the USB host take their share, and before the
    // display task's SPI driver fragments what is left. The coarse
    // stage's 2304-point workspace is 41 % faster in internal DRAM
    // (§26.3) and this is the only moment the block is available.
    let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
    log::info!(
        "ft4_app: mixed-radix FFT scratch in {} DRAM",
        if internal { "INTERNAL" } else { "PSRAM" }
    );

    let nvs = boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace");

    spawn_slot_task();

    // Register the sink before the display task, whose body installs
    // the USB host driver — the same "wire the consumer before the
    // driver" ordering `fst4` and `wspr` rely on.
    crate::uac::set_audio_sink(Ft4Sink);

    crate::display::run_log_panel(
        peripherals.i2c0,
        peripherals.spi2,
        peripherals.pins,
        &crate::FANOUT,
        nvs,
        BootMode::Ft4,
    )
}

/// Audio callback. Accumulates the slot **and** its periodogram, and
/// hands both over when one completes.
struct Ft4Sink;

impl crate::uac::AudioSink for Ft4Sink {
    fn push_samples(&mut self, samples_12k_mono: &[i16]) {
        if !AUDIO_LIVE.swap(true, Ordering::AcqRel) {
            log::info!("ft4_app: real UAC audio active");
        }
        let Ok(mut staging) = STAGING.lock() else {
            return;
        };
        if staging.len() + samples_12k_mono.len() > STAGING_CAP {
            log::warn!(
                "ft4_app: audio staging full ({} samples) — dropping {}; the slot task is behind",
                staging.len(),
                samples_12k_mono.len(),
            );
            return;
        }
        staging.extend_from_slice(samples_12k_mono);
    }
}

extern "C" fn slot_task_entry(_arg: *mut core::ffi::c_void) {
    slot_loop();
}

fn spawn_slot_task() {
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(slot_task_entry),
            c"ft4_slot".as_ptr(),
            DECODE_STACK,
            core::ptr::null_mut(),
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("ft4_app: slot task spawn failed ({DECODE_STACK} B stack)");
    }
}

/// Drain captured audio into the accumulator, and decode each slot it
/// completes.
///
/// One task, not two: the accumulation is ~12 % duty (§33) and the
/// decode ~2.4 s of a 7.5 s slot, so they fit in series with room, and
/// a second hand-off would only add a place for a slot to go missing.
/// The cost is that rows for the next slot are computed in a burst
/// after a decode finishes rather than smoothly — the waterfall
/// stutters, `savg` does not, because it is bit-identical at any block
/// size (`ft4_savg_builder_matches_whole_slot`).
fn slot_loop() -> ! {
    let mut accum = rx::SlotAccum::new();
    let mut block: Vec<i16> = Vec::with_capacity(STAGING_CAP);
    loop {
        block.clear();
        if let Ok(mut staging) = STAGING.lock() {
            core::mem::swap(&mut *staging, &mut block);
        }
        if block.is_empty() {
            unsafe { esp_idf_svc::sys::vTaskDelay(50 / port_tick_ms()) };
            continue;
        }

        // The rows the coarse stage is already transforming, mapped to
        // palette indices on the way past. §35.2: 174 us a row after
        // the mapping was made integer.
        let done = accum.push_with_rows(&block, &mut |row| {
            let cells = rx::wf_row(row);
            if let Ok(mut ui) = UI.lock() {
                ui.push_waterfall(cells);
            }
        });
        let Some(slot) = done else {
            continue;
        };

        let seq = SLOT_SEQ.fetch_add(1, Ordering::AcqRel) + 1;
        let o = rx::decode_slot(&slot, BUDGET_MS);
        log::info!(
            "ft4_app: slot {seq} — {} of {} candidates tried, {} decodes in {} ms of \
             {BUDGET_MS} ms{}",
            o.tried,
            o.cands,
            o.decodes.len(),
            o.elapsed_us / 1000,
            match o.cut_at_score {
                // Baseline-normalised, so 1.2 is WSJT-X's own
                // threshold: a cut near it gave up almost nothing.
                Some(sc) => format!(" — cut {} weakest at score {sc:.2}", o.cands - o.tried),
                None => String::new(),
            },
        );
        if let Ok(mut ui) = UI.lock() {
            ui.update_status(|st| {
                st.free_heap_kb = free_heap_kb();
            });
            for d in &o.decodes {
                let mut msg: heapless::String<22> = heapless::String::new();
                let _ = msg.push_str(&d.msg[..d.msg.len().min(22)]);
                ui.push_decode(DecodedRow {
                    df_hz: d.freq_hz.round().clamp(0.0, 65_535.0) as u16,
                    snr_db: d.snr_db.round().clamp(-128.0, 127.0) as i8,
                    hard_errors: d.hard_errors.min(255) as u8,
                    msg,
                    slot_seq: seq,
                    first_seq: seq,
                });
                log::info!(
                    "    {:>6.1} Hz  {:>+5.2} s  {:>3.0} dB  {}",
                    d.freq_hz,
                    d.dt_sec,
                    d.snr_db,
                    d.msg,
                );
            }
        }
    }
}

fn port_tick_ms() -> u32 {
    (1_000 / esp_idf_svc::sys::configTICK_RATE_HZ).max(1)
}

fn free_heap_kb() -> u32 {
    const CAPS: u32 = (1 << 11) | (1 << 2);
    (unsafe { esp_idf_svc::sys::heap_caps_get_free_size(CAPS) } / 1024) as u32
}
