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
use esp_idf_svc::eventloop::EspSystemEventLoop;
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

/// Replay the baked golden slot when no radio is feeding audio.
///
/// **On by default for FT4**, unlike FST4's, whose equivalent is behind
/// `MFSK_FST4_REPLAY`. The reason is the band, not the code: FT4
/// activity is thin enough that a receiver pointed at a real antenna
/// can sit for a long time decoding nothing, which is
/// indistinguishable from a receiver that is broken. FST4's own doc
/// warns that replayed stations look exactly like received ones on
/// screen — that is true here too, and the log line below is what
/// tells them apart.
///
/// Set `MFSK_FT4_REPLAY=0` for a build that only ever decodes what the
/// antenna heard.
const REPLAY_GOLDEN: bool = match option_env!("MFSK_FT4_REPLAY") {
    Some(v) => !matches!(v.as_bytes(), [b'0']),
    None => true,
};

/// One FT4 slot of 12 kHz PCM — the WSJT-X golden, baked by
/// `ft4_bake_golden_precomputed`. 19 signals, 14 in the search band,
/// 11 decoding in a single pass at `DecodeDepth::EMBEDDED`.
#[cfg(feature = "ft4-replay")]
const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/ft4_golden_audio.bin");
#[cfg(not(feature = "ft4-replay"))]
const GOLDEN_AUDIO: &[u8] = &[];

/// Replay feed size — the ~256 samples a UAC read produces, so the
/// replay exercises the block cadence a radio will rather than a
/// friendlier one.
const REPLAY_BLOCK: usize = 256;

/// The FT4 slot, in milliseconds.
///
/// `time_sync::samples_to_next_slot_12k_ms` wants the grid period, and
/// 7.5 s is not a whole number of them — which is why the FT8 path's
/// whole-second `samples_to_next_slot_12k` could not be reused here.
const FT4_SLOT_MS: u64 = 7_500;

/// Smallest DT-median correction worth applying to the grid.
///
/// ~24 ms — half an FT4 symbol. Below this it is jitter, and the UTC
/// anchor has the grid within ~100 ms already; chasing it every slot
/// would only add noise.
const FT4_DT_TRIM_MIN_SAMPLES: u32 = 288;

/// Spectrogram rows per waterfall row.
///
/// The coarse stage produces one row per 48 ms — 152 a slot — and the
/// shared waterfall was built for FT8, which pushes **one per 15 s
/// slot**. Feeding it all 152 turns the 100-row ring over 1.5 times
/// per slot, so the display shows about 4.8 s of band and races; and
/// since every `push_waterfall` bumps `wf_push_seq`, the display loop
/// repaints its 48 KB region ~20 times a second chasing them. On the
/// panel that reads as the waterfall overflowing, which is exactly
/// what it is.
///
/// Every 6th row is ~one per 288 ms: still visibly flowing during
/// capture, which is the point of having the rows at all, but 100 rows
/// is now ~29 s of history and the repaint rate is ~3.5 Hz.
///
/// The rows themselves are not thrown away cheaply — they are free, and
/// the decoder consumes all of them. This decimates the *drawing*
/// only.
const WF_ROW_DECIM: usize = 6;

/// Stack for the decode task.
///
/// **8 KB, measured** — the same code in `ft4-demo`'s feed thread used
/// 2 584 B of 32 KB (2026-09-01, `board::log_task_stacks`). Everything
/// large is heap: a slot of audio, one `cd0` per candidate. The 32 KB
/// this used to ask for came from the FT8 path and cost internal DRAM
/// the decoder's own allocations needed more (`ft4_rx::WORKER_STACK`
/// carries the argument).
const DECODE_STACK: u32 = 8 * 1024;

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

    // WiFi, through the same path `fst4_app` and `wspr_app` use
    // (`crate::net`). FT4 did not bring the network up at all until
    // 2026-09-01, which was survivable for a receiver replaying a
    // baked slot and is not for a QSO: the slot grid needs absolute
    // time, and NTP is where it comes from.
    //
    // Driver construction is synchronous and stays here — `modem` is
    // consumed by value and is not returned on `Err` — while the slow,
    // flaky half is what `net::spawn` backgrounds. It runs **before**
    // the slot task so that the internal DRAM WiFi wants is claimed
    // before the decoder's own worker asks for its stack, rather than
    // after: which of the two loses is a fact worth having on the log,
    // not a race worth hiding.
    let modem = peripherals.modem;
    if crate::WIFI_SSID.is_empty() {
        log::warn!("ft4_app: WIFI_SSID empty (no cfg.toml) — no NTP, no UDP log, no config page");
    } else {
        let sysloop = EspSystemEventLoop::take().expect("sysloop");
        match mfsk_app_shared::wifi::wifi_driver_init(modem, sysloop, Some(nvs_part.clone())) {
            Ok(driver) => {
                let settings_nvs = mfsk_app_shared::settings::open_nvs(nvs_part.clone())
                    .expect("settings NVS open");
                crate::net::spawn(
                    driver,
                    std::sync::Arc::new(std::sync::Mutex::new(settings_nvs)),
                    crate::net::Config {
                        name: "ft4_app::net",
                        // FT4's decode budget is 1 750 ms from window
                        // close to key-up, and the WiFi task runs at
                        // priority 23: an association campaign in the
                        // middle of a slot is a missed QSO, not a slow
                        // log.
                        policy: crate::net::DECODE_FIRST,
                        power_save: true,
                        // The shared `StatusInfo` has no NTP field —
                        // the FT8 panel shows time through `utc_sod`,
                        // which NTP sets by setting the clock. So this
                        // only says whether it happened; putting it on
                        // the status bar is part of moving the other
                        // two receivers onto this panel, not of this
                        // change.
                        on_ntp: |synced| log::info!("ft4_app: NTP synced = {synced}"),
                    },
                );
            }
            Err(e) => log::error!("ft4_app: WiFi driver init failed (permanent this boot): {e:#}"),
        }
    }

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
/// completes. Falls back to replaying the golden while no radio is
/// feeding it — see [`REPLAY_GOLDEN`].
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

    // The replay source, decoded from the baked asset once.
    let golden: Vec<i16> = GOLDEN_AUDIO
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
    let replaying = REPLAY_GOLDEN && !golden.is_empty();
    if replaying {
        log::warn!(
            "ft4_app: no radio yet — replaying {} baked samples. Decodes below are from a \
             recording, not the antenna; they read identically on screen. \
             MFSK_FT4_REPLAY=0 disables this.",
            golden.len(),
        );
    } else if REPLAY_GOLDEN {
        log::info!("ft4_app: replay requested but no golden linked — build with --features ft4-replay");
    }
    let mut gpos = 0usize;
    let mut row_seq: usize = 0;
    // Slot-grid alignment (#354). `false` until the first block of real
    // UAC audio; the replay source is not real-time, so anchoring it to
    // UTC would be meaningless.
    let mut live_prev = false;
    let mut last_finalised = mfsk_app_shared::time_sync::slots_finalised();
    // Paces the replay to 12 kHz. Absolute, not per-block, so a slow
    // decode does not make the replay drift slower than real time.
    let t_start = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let mut fed: u64 = 0;

    loop {
        block.clear();
        if let Ok(mut staging) = STAGING.lock() {
            core::mem::swap(&mut *staging, &mut block);
        }

        if block.is_empty() {
            if !replaying || AUDIO_LIVE.load(Ordering::Acquire) {
                unsafe { esp_idf_svc::sys::vTaskDelay(50 / port_tick_ms()) };
                continue;
            }
            // One UAC-sized block of the golden, at 12 kHz.
            let take = REPLAY_BLOCK.min(golden.len() - gpos);
            block.extend_from_slice(&golden[gpos..gpos + take]);
            gpos = (gpos + take) % golden.len();
            fed += take as u64;
            let due_us = (fed * 1_000_000 / 12_000) as i64;
            let now = unsafe { esp_idf_svc::sys::esp_timer_get_time() } - t_start;
            if due_us > now {
                unsafe {
                    esp_idf_svc::sys::vTaskDelay((((due_us - now) / 1_000).max(1) as u32) / port_tick_ms())
                };
            }
        }

        // Slot-grid alignment (#354). The FT8 path anchors its 15 s grid
        // to UTC in `Ft8ChunkSink`; FT4's boundary logic lives in
        // `SlotAccum`, so the anchor is driven from here — the board
        // half owns the clock (`time_sync`), the shared half only moves
        // the grid when told.
        let live = AUDIO_LIVE.load(Ordering::Acquire);
        if live && !live_prev {
            // Any golden partial in the accumulator is not this slot —
            // start the live grid from a clean window.
            accum = rx::SlotAccum::new();
            log::info!("ft4_app: live audio — slot accumulator reset for UTC alignment");
        }
        live_prev = live;
        if live {
            if let Some(remain) =
                mfsk_app_shared::time_sync::samples_to_next_slot_12k_ms(FT4_SLOT_MS)
            {
                let was_aligned = accum.is_aligned();
                accum.anchor_or_reanchor(remain);
                if !was_aligned && accum.is_aligned() {
                    log::info!(
                        "ft4_app: slot grid anchored to UTC — {} ms to the next boundary",
                        remain / 12,
                    );
                }
                // Grid lock state (#356b). FT4's coarse stage has no DT
                // dimension, so there is no air-lock here — the grid is
                // the RTC's, or NTP's once that lands.
                mfsk_app_shared::time_sync::note_grid_lock(
                    if mfsk_app_shared::time_sync::clock_is_disciplined() {
                        mfsk_app_shared::time_sync::GridLock::Ntp
                    } else {
                        mfsk_app_shared::time_sync::GridLock::Rtc
                    },
                );
            }
        }

        // The rows the coarse stage is already transforming, mapped to
        // palette indices on the way past. §35.2: 174 us a row after
        // the mapping was made integer.
        let done = accum.push_with_rows(&block, &mut |row| {
            row_seq += 1;
            if !row_seq.is_multiple_of(WF_ROW_DECIM) {
                return;
            }
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
            "ft4_app: slot {seq} grid={} — {} of {} candidates tried, {} decodes in {} ms of \
             {BUDGET_MS} ms{}",
            mfsk_app_shared::time_sync::grid_lock().label(),
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

        // DT-median grid trim (#354). The UTC anchor gets the grid
        // within ~100 ms; the median DT of the slot's decodes closes the
        // rest — the STAGING latency, and the residual after an
        // RTC-only anchor. Only acted on when a new median actually
        // landed (`finalize_slot` is a no-op on a slot with no decodes),
        // and after correction the next slot's DTs sit near zero, so the
        // trim settles itself. FT4's coarse stage returns `dt = 0`, so
        // there is no cold-start path here for a slot that decodes
        // nothing with no clock — that is #356.
        //
        // Live audio only: the replay source is a whole recorded slot
        // the decoder finds its own `dt` in, so its DTs say nothing
        // about a grid.
        if live {
            for d in &o.decodes {
                mfsk_app_shared::time_sync::record_decode_dt(d.dt_sec);
            }
            mfsk_app_shared::time_sync::finalize_slot();
            let finalised = mfsk_app_shared::time_sync::slots_finalised();
            if finalised != last_finalised {
                last_finalised = finalised;
                if let Some(off_sec) = mfsk_app_shared::time_sync::slot_dt_offset() {
                    // Cross-slot phase filter (#356b) — 7.5 s period.
                    mfsk_app_shared::time_sync::observe_slot_phase(off_sec, 7.5);
                    let delta = (off_sec * 12_000.0).round() as i32;
                    if delta.unsigned_abs() >= FT4_DT_TRIM_MIN_SAMPLES {
                        accum.shift_next_window(delta);
                        log::info!(
                            "ft4_app: DT median {off_sec:+.3} s ({finalised} slots) — grid {delta:+} samples"
                        );
                    }
                }
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
