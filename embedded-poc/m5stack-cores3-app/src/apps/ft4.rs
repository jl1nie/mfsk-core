// SPDX-License-Identifier: GPL-3.0-only
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
//! path uses, not a second implementation. The waterfall's rows come
//! from the audio itself through `waterfall_feed`, the same feed every
//! mode uses; FT4 used to hand over rows from its coarse periodogram
//! instead.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};


use embedded_shared::apps::ft4_rx as rx;
use crate::boot::{BootCtx, Display, Panel, Receiver};
use mfsk_app_shared::ui::state::{SlotDecode, UI};
use mfsk_app_shared::boot_mode::BootMode;

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

/// Smallest DT median worth a log line. ~24 ms, half an FT4 symbol.
///
/// **It used to be the smallest correction worth *applying***, and the
/// grid was steered by it every slot. Measured on the board through the
/// real sink (2026-09-21, `logs/ft4sim_2026-09-21.log`), that steering
/// fought the clock re-anchor and neither won: the DT median alternated
/// −0.45 s / −0.25 s and `slot grid +45x ms off the clock — trimmed`
/// fired on all 15 slots, for as long as the run lasted.
///
/// Two mechanisms, pulling opposite ways once a slot. The trim moved
/// the window toward the audio; the re-anchor put it back on the
/// clock's grid; the next slot measured the same offset again. It was
/// invisible until the replay went through `Ft4Sink` instead of around
/// it, because nothing else anchored at all.
///
/// FT8 removed its own per-slot DT servo on 2026-09-05 for a different
/// reason with the same conclusion — a decode's DT is *that station's*
/// clock error, not ours. FT4 cannot even take the other side of that
/// trade: its band does not carry enough stations for a median to mean
/// anything. So the phase comes from NTP or from the fix FT8 persisted,
/// and this number only decides whether the measurement is worth
/// printing.
const FT4_DT_REPORT_MIN_SAMPLES: u32 = 288;

/// Stack for the decode task.
///
/// **10 KB, measured by stage** (2026-09-22, FT4 SIM through the real
/// sink, `uxTaskGetStackHighWaterMark` after each step of `slot_loop`,
/// `logs/ft4sim_stackprobe*_2026-09-22.log`):
///
/// | after | free of 8 KB |
/// |---|---|
/// | `slot_loop`'s setup | ~4 970 B |
/// | `take_provisional` (before the fix below) | ~1 050 B |
/// | `decode_slot_with` | 1 136 B (with the fix) / 664 B (without) |
///
/// This task runs the per-candidate decode itself (`run_candidates`)
/// on top of `slot_loop`'s ~3 KB, so it needs the candidate worker's
/// ~5 KB *plus* that. 8 KB left 444-732 B across runs — the board's
/// own `[stacks]` line calls anything under 2 048 B tight. 10 KB puts
/// the measured peak ~3.1 KB clear.
///
/// The provisional coarse pass had been taking 3.6-3.9 KB of it on its
/// own: `engine::baseline` sorted with the stable `sort_by`, whose
/// driftsort scratch is a 4 KB stack array. It picks a percentile, so
/// stability buys nothing; it is `sort_unstable_by` now.
///
/// The old figure, 8 KB from `ft4-demo`'s feed thread using 2 584 B
/// (2026-09-01), was measured on that thread, which never ran this
/// loop's setup or the provisional pass.
const DECODE_STACK: u32 = 10 * 1024;

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

/// Grid-phase correction (µs) recovered from a prior FT8 cold
/// acquisition and persisted across the reboot (#356b), or `i32::MIN`
/// if there is none / it was stale. `run` loads it, `slot_loop` folds
/// it into its first UTC anchor.
static ACQUIRED_GRID_FIX_US: AtomicI32 = AtomicI32::new(i32::MIN);

/// A fix older than this is not trusted — the ESP crystal holds phase
/// for hours at −3.3 ppm but a day's drift is a whole FT4 symbol many
/// times over, and a stale fix pointing at the wrong phase is worse
/// than starting from the RTC alone.
const GRID_FIX_MAX_AGE_S: i64 = 2 * 3600;
/// Minimum acquisition confidence to seed FT4's grid from.
const GRID_FIX_MIN_R: f32 = 0.55;

/// FT4 through the shared boot sequence — see `crate::boot`.
///
/// **The slot task starts in `start`, not `spawn_workers`.** It takes
/// its stack immediately, and the WiFi driver's synchronous
/// internal-DRAM claim runs between the two steps: this receiver's own
/// comment for the ordering used to read "runs before the slot task so
/// that the internal DRAM WiFi wants is claimed before the decoder's
/// own worker asks for its stack", and that is exactly what the
/// sequence now guarantees for everyone.
pub struct Ft4Rx;

impl Receiver for Ft4Rx {
    const TAG: &'static str = "ft4_app";

    fn prepare(ctx: &BootCtx) {
        let internal = embedded_shared::esp_dsp_fft::reserve_mixed_scratch();
        log::info!(
            "ft4_app: mixed-radix FFT scratch in {} DRAM",
            if internal { "INTERNAL" } else { "PSRAM" }
        );
        seed_grid_from_ft8(ctx);
    }

    fn attach_panel(_ctx: &BootCtx, display: Display) -> Panel {
        Panel::DrawsLast(display)
    }

    fn net_config(_ctx: &BootCtx) -> Option<crate::net::Config> {
        Some(crate::net::Config {
            name: "ft4_app::net",
            // Modem power save, for the reason `crate::net` records:
            // an associated but fully-awake STA hands every broadcast
            // frame on the LAN to a priority-23 driver task, and FT4's
            // decode budget is 1 750 ms from the capture window
            // closing to key-up.
            power_save: true,
            ntp: true,
            without: "no NTP, no UDP log, no config page",
            bringup: crate::net::Bringup::Connect,
            http: true,
            on_ntp: |synced| log::info!("ft4_app: NTP synced = {synced}"),
        })
    }

    fn start(_ctx: &BootCtx) {
        spawn_slot_task();
        crate::uac::set_audio_sink(Ft4Sink);
        sim_feed_if_asked();
    }

    fn run_forever(ctx: BootCtx, panel: Panel) -> ! {
        let Panel::DrawsLast(display) = panel else {
            unreachable!("ft4 draws from run_forever");
        };
        // **The panel stays at `main`'s 1 here, below the decode** —
        // unlike FT8 (`display::PANEL_PRIORITY`). FT4 is the fast
        // protocol and its reply deadline is the tight one: above the
        // decode the panel cost it ~200 ms of loop end and one of eleven
        // decodes in some slots (2026-09-21). The screen pauses for the
        // decode instead.
        crate::display::run_log_panel(
            display.i2c0,
            display.spi2,
            display.pins,
            &crate::FANOUT,
            ctx.nvs,
            ctx.mode,
        )
    }
}

/// The FT8 controller writes `grid_fix` when it locks; FT4 reads it so
/// a mode change does not cost the phase. The alternative here is no
/// phase at all — FT4's own capture window closes at 6.775 s of the
/// slot against a ±1.0 s search, so a grid that is a second out cuts
/// the frame rather than shifting it.
fn seed_grid_from_ft8(ctx: &BootCtx) {
    let nvs = ctx.nvs.lock().expect("NVS mutex poisoned");
    let Some(fix) = mfsk_app_shared::grid_fix::load(&nvs) else {
        return;
    };
    let now_epoch = mfsk_app_shared::time_sync::utc_now_ms()
        .map(|ms| (ms / 1000) as i64)
        .unwrap_or(0);
    match fix.correction_for(now_epoch, 7.5, GRID_FIX_MAX_AGE_S, GRID_FIX_MIN_R) {
        Some(p) => {
            ACQUIRED_GRID_FIX_US.store((p * 1_000_000.0) as i32, Ordering::Release);
            log::info!(
                "ft4_app: seeding grid from a persisted FT8 fix — {p:+.3} s (R {:.2})",
                fix.confidence
            );
        }
        None => log::info!(
            "ft4_app: persisted grid fix present but stale/weak (R {:.2}, {} s old) — ignoring",
            fix.confidence,
            now_epoch - fix.epoch_at_fix
        ),
    }
}

struct Ft4Sink;

impl crate::uac::AudioSink for Ft4Sink {
    fn push_samples(&mut self, samples_12k_mono: &[i16]) {
        if !AUDIO_LIVE.swap(true, Ordering::AcqRel) {
            if crate::uac::sim_feeding() {
                log::info!("ft4_app: SIM audio active — baked slot through the real sink");
            } else {
                log::info!("ft4_app: real UAC audio active");
            }
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
/// The cost is that the next slot's periodogram is accumulated in a
/// burst after a decode finishes rather than smoothly, which `savg`
/// does not notice: it is bit-identical at any block size
/// (`ft4_savg_builder_matches_whole_slot`).
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
    // Slot-grid alignment (#354). `false` until the first block of real
    // UAC audio; the replay source is not real-time, so anchoring it to
    // UTC would be meaningless.
    let mut live_prev = false;
    let mut last_finalised = mfsk_app_shared::time_sync::slots_finalised();
    // A usable persisted FT8 acquisition fix (#356b) was loaded in
    // `run` — seed the first anchor from it, and hold `GridLock::Air`
    // until NTP takes over.
    let mut seeded_from_air = ACQUIRED_GRID_FIX_US.load(Ordering::Acquire) != i32::MIN;
    // Paces the replay to 12 kHz. Absolute, not per-block, so a slow
    // decode does not make the replay drift slower than real time.
    let t_start = unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let mut fed: u64 = 0;
    // **Callsign hashes, resolved across slots.** A 28-bit callsign
    // field can carry a 22-bit hash instead of a call, and that only
    // names a station the table has heard before — so the table
    // outlives the slot and lives here. `rx::decode_candidate` runs on
    // two cores and cannot hold a `&mut` to it (its own comment: "no
    // shared mutable state ... which is what lets two cores run it at
    // once"), so the resolving and the learning both happen below,
    // after the workers have joined.
    let mut calls = mfsk_core::msg::CallsignHashTable::new();
    // The per-candidate DDCs for the window being captured, started on
    // core 1 once the provisional list is known — see
    // `rx::EarlyBasebands`. Replaced whenever a new window reaches that
    // point, so a window thrown away by a re-anchor takes its worker
    // with it.
    let mut early: Option<rx::EarlyBasebands> = None;

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
            // The replay does not pass through `uac`, so it offers the
            // waterfall its audio itself.
            crate::waterfall_feed::push(&block);
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
            if let Some(mut remain) =
                mfsk_app_shared::time_sync::samples_to_next_slot_12k_ms(FT4_SLOT_MS)
            {
                // Into the accumulator's frame before anything else.
                //
                // The clock answers "how far to the boundary from
                // *now*"; `SlotAccum` needs "from where the
                // accumulator is", and the two are `block` apart —
                // audio that has arrived and not yet been fed. During
                // capture that is one UAC read (~21 ms) and would not
                // matter. Once a slot, it is the decode: `block` holds
                // the ~1.4 s that piled up while `decode_slot` ran, and
                // 1.4 s is past `REANCHOR_THRESH_SAMPLES` and
                // comparable to the whole Δt search. Unconverted, a
                // grid that is not drifting reads as off by exactly the
                // backlog, in the same direction, every slot.
                remain += block.len();
                let was_aligned = accum.is_aligned();
                // **Every anchor, not just the first.**
                //
                // This used to fold the persisted FT8 fix in once, on
                // the reasoning that "after that the DT-median trim
                // owns the fine correction". It does not, and cannot:
                // FT4's band carries too few stations for a DT median
                // to be a phase reference, which is the whole reason
                // the trim below is gone. With nothing owning the
                // correction, a re-anchor would drop the sub-second
                // part and put the grid back on the RTC's raw phase —
                // whole seconds, since `read_into_system_clock` commits
                // `tv_usec: 0`.
                //
                // So the clock supplies the seconds and the fix
                // supplies the remainder, on every anchor, for as long
                // as the fix is fresh. That is what "FT8 locks, FT4
                // runs off it" means in code.
                if seeded_from_air {
                    let fix_us = ACQUIRED_GRID_FIX_US.load(Ordering::Acquire);
                    let period = (FT4_SLOT_MS * 12) as i64;
                    let shifted = remain as i64 + (fix_us as i64 * 12 / 1000);
                    remain = shifted.rem_euclid(period) as usize;
                }
                let err = accum.phase_error(remain);
                accum.anchor_or_reanchor(remain);
                if !was_aligned && accum.is_aligned() {
                    log::info!(
                        "ft4_app: slot grid anchored to {} — {} ms to the next boundary",
                        if seeded_from_air { "the FT8 air fix" } else { "UTC" },
                        // `remain` carries the staged backlog added
                        // above, so it can exceed a slot — `block` holds
                        // up to `STAGING_CAP`. The grid takes it modulo
                        // the period itself; this line is what an
                        // operator reads to confirm the anchor, so it
                        // reduces too rather than printing 11 s of a
                        // 7.5 s grid.
                        remain as u64 % (FT4_SLOT_MS * 12) / 12,
                    );
                } else if let Some(delta) = err {
                    // Past the threshold, so the grid just moved. Worth
                    // a line: with a disciplined clock this is the
                    // band's own offset, and with an RTC-seeded one it
                    // is that chip's drift, accumulating in view.
                    log::info!(
                        "ft4_app: slot grid {:+} ms off the clock — trimmed",
                        delta / 12
                    );
                }
                // Grid lock state (#356b). FT4's coarse stage has no DT
                // dimension of its own, so the grid is the RTC's, or the
                // persisted FT8 air fix, or NTP's once that lands — NTP
                // upgrades either of the first two.
                if mfsk_app_shared::time_sync::clock_is_disciplined() {
                    mfsk_app_shared::time_sync::note_grid_lock(
                        mfsk_app_shared::time_sync::GridLock::Ntp,
                    );
                    seeded_from_air = false;
                } else {
                    mfsk_app_shared::time_sync::note_grid_lock(if seeded_from_air {
                        mfsk_app_shared::time_sync::GridLock::Air
                    } else {
                        mfsk_app_shared::time_sync::GridLock::Rtc
                    });
                }
            }
        }

        let done = accum.push(&block);
        if let Some(carriers) = accum.take_provisional() {
            early = rx::EarlyBasebands::start(accum.half_stream(), &carriers);
        }
        let Some(slot) = done else {
            continue;
        };

        let seq = SLOT_SEQ.fetch_add(1, Ordering::AcqRel) + 1;
        let o = rx::decode_slot_with(&slot, BUDGET_MS, early.take());
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
        // Resolve against earlier slots, then learn this slot's own
        // calls for the next — in decode order, which is the order
        // WSJT-X registers them in.
        let mut o = o;
        for d in o.decodes.iter_mut() {
            if let Some(t) = mfsk_core::msg::wsjt77::unpack77_learn(&d.msg77, &mut calls) {
                d.msg = t;
            }
        }
        let o = o;
        if let Ok(mut ui) = UI.lock() {
            ui.update_status(|st| {
                st.free_heap_kb = free_heap_kb();
            });
            // The station list's one entry point, as every mode.
            let rows: Vec<SlotDecode> = o
                .decodes
                .iter()
                .map(|d| SlotDecode {
                    freq_hz: d.freq_hz,
                    snr_db: d.snr_db,
                    dt_sec: d.dt_sec,
                    text: &d.msg,
                    hard_errors: d.hard_errors,
                })
                .collect();
            crate::storage::publish_slot(
                &mut ui,
                BootMode::Ft4,
                crate::storage::decoded_slot_unix(7_500),
                &rows,
            );
            for d in &o.decodes {
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
                    // A **readout**, and nothing steers on it.
                    mfsk_app_shared::time_sync::observe_slot_phase(off_sec, 7.5);
                    let delta = (off_sec * 12_000.0).round() as i32;
                    if delta.unsigned_abs() >= FT4_DT_REPORT_MIN_SAMPLES {
                        log::info!(
                            "ft4_app: DT median {off_sec:+.3} s ({finalised} slots, {delta:+} \
                             samples) — not applied"
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

/// `MFSK_CORES3_SIM`: feed the baked FT4 slot through **`Ft4Sink`**,
/// the same path a radio's audio takes.
///
/// The replay inside `slot_loop` is not this. It pushes samples
/// straight into the staging buffer, so it exercises the decoder and
/// nothing above it: no sink, no `SlotAccum` anchor, no slot grid —
/// which is precisely the machinery #354 and #356 are about, and the
/// reason FT4's grid wiring has never been tested on hardware. It also
/// declines to anchor at all (`live_prev` stays false, because "the
/// replay source is not real-time"), so a replay run cannot tell a
/// working grid from a broken one.
///
/// This is the FT8 harness's shape (`MFSK_CORES3_SIM`, `ebc4e42`)
/// applied to FT4: real-time pacing into the registered sink, the
/// phase set by `MFSK_SIM_OFFSET_MS` so a deliberate grid error can be
/// aimed, and `MFSK_SIM_NO_CLOCK` to take the RTC away and make the
/// receiver find the phase without one.
///
/// The audio is `assets/ft4_golden_audio.bin` — the WSJT-X FT4 sample
/// `000000_000002.wav`, a real off-air recording of six stations, not
/// a synthesised frame. Needs `--features ft4-replay`, which is what
/// links it.
fn sim_feed_if_asked() {
    if option_env!("MFSK_CORES3_SIM").is_none() {
        return;
    }
    if GOLDEN_AUDIO.is_empty() {
        log::error!("ft4_app: MFSK_CORES3_SIM set but no golden linked — build with ft4-replay");
        return;
    }
    if option_env!("MFSK_SIM_NO_CLOCK").is_some() {
        mfsk_app_shared::time_sync::suppress_clock(true);
        log::warn!("ft4_app SIM: clock suppressed — the grid has to come from the air or the fix");
    }
    let offset_ms: usize = option_env!("MFSK_SIM_OFFSET_MS")
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    crate::uac::spawn_sim_feed(
        crate::uac::SimSource::Pcm(GOLDEN_AUDIO),
        rx::SLOT_SAMPLES,
        offset_ms * 12,
    );
}
