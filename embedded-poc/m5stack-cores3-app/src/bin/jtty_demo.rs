// SPDX-License-Identifier: GPL-3.0-only
//! JTTY receiver, busy-band demo — the desk equivalent of a contest pileup on the air.
//!
//! Replays `mfsk_core::jtty::testsig::pileups`' `"band, 6 long messages"` scene (six
//! stations sending across the band at once) at its real 12 kHz rate, forever, through
//! the real two-core split (`Front` on core 1, `Back` here) that `jtty-bench` (#499 E0)
//! already proved fits the window budget on this board. Unlike the bench, this is not a
//! timing measurement: it exists to *see* the receiver work — the waterfall and the
//! decoded-message list, on the CoreS3's own screen.
//!
//! ## The screen is the FT8 controller's
//!
//! Waterfall, decoded list and status bar all go through `mfsk_app_shared::ui::state::UI`
//! and are drawn by `display::run_log_panel` — the same state and loop every other mode
//! uses, per `ft4_demo.rs`'s own precedent. JTTY has no slot, so `DecodedRow` is a loose
//! fit: a row is pushed once, when a message completes (`MessageUpdate::complete`), not
//! updated live while it grows — that live-update wiring is `docs/notes/JTTY_CORES3_APP.md`
//! §9's E2, not this. `snr_db`/`hard_errors`/`dt_ds` have no JTTY equivalent yet
//! (`MessageUpdate` carries none of them — see that design doc's §11, still open) and are
//! zeroed rather than guessed.
//!
//! **This is a separate bin rather than a boot mode**, exactly for `ft4_demo.rs`'s reason:
//! it brings up no USB host, so it can be reflashed freely and the serial console stays
//! attached. `BootMode::Decode` is passed to the panel and the waterfall feed for the same
//! reason — it is the mode that does not take the PHY.
//!
//! Build: `cargo build --release --features jtty-rx --bin jtty-demo`.

use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use esp_idf_hal::cpu::Core;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::ui::state::{DecodedRow, UI};
use mfsk_core::jtty::rx::{Back, Front, Params, Prepared, Receiver, STEP};
use mfsk_core::jtty::testsig::pileups;

use mfsk_core_m5stack_cores3_app as app;

/// The two cores' queue depth — 6, `jtty-bench`'s finding on this pattern (#512, §14), where
/// with `Back` alone on core 0 this case peaked at 5 deep with nothing dropped. 7 was tried
/// here (logs/jtty_demo_queue7_2026-09-28.log): the queue filled to 7 every pass and drops
/// went 2 -> 1, completions unchanged — `Back` falls behind for a stretch rather than
/// through a burst, so depth only moves the drop later, for 605 KB of PSRAM a slot.
const QUEUE_DEPTH: usize = 6;

/// Longest a JTTY message runs (`mfsk_app_shared::jtty_tx::MAX_SAMPLES` / 12 kHz), so a
/// completed row does not read "stale" moments after it finishes.
const UI_SLOT_PERIOD_MS: u32 = 30_000;

/// `Front`'s thread stack — the same size `jtty-bench`'s pipeline/selftest benches use.
const FRONT_STACK: usize = 16 * 1024;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("=== mfsk-core-m5stack-cores3-app jtty-demo ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);

    // `Back`'s ladder has no yield point for hundreds of ms at a time — same call, same
    // reason, as every JTTY bench and every other receiver here.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("jtty-demo: task watchdog deinit -> {r}");

    let peripherals = Peripherals::take().expect("peripherals taken twice");
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = Arc::new(std::sync::Mutex::new(
        boot_mode::open_nvs(nvs_part.clone()).expect("NVS open mfsk namespace"),
    ));

    // No WiFi: this demo needs no network, and skipping it keeps the board flashable and
    // free of the association/UDP-log complications a live radio session already has to
    // deal with separately.
    // No slot rules — JTTY has no slot, so a rule every 15 s (Decode's
    // period) marks nothing on the air — and the column-width transform:
    // the panel shares core 0 with `Back`, which has no slack to give it.
    app::waterfall_feed::init_with(app::waterfall_feed::FeedConfig {
        slot_period_ms: 0,
        nfft: embedded_shared::waterfall::WF_NFFT_COLUMN,
    });
    if let Ok(mut ui) = UI.lock() {
        ui.set_slot_period_ms(UI_SLOT_PERIOD_MS);
    }

    // Pinned to core 0: `Back` runs here, and a comparison with `jtty-bench` (Back on core 0)
    // means nothing if the scheduler is free to move it to core 1 when the panel leaves.
    let spawn =
        app::board::spawn_named_tuned(c"jttyfeed", FRONT_STACK, None, Some(Core::Core0), feed_loop);
    if let Err(e) = spawn {
        log::error!("jtty-demo: feed thread spawn failed ({e})");
    }

    // `jttyfeed` (this thread's `Back::process`) runs at the pthread default (5) and, on
    // this busy pattern, close to continuously — unlike FT4's demo, which never raises its
    // panel priority because FT4's decode duty is low (`apps/ft4.rs`'s own comment). Left
    // at `main`'s default (1) the panel starved: the first hardware run showed 7 panel
    // frames in 20 s and a 19.8 s gap between two of them — no waterfall motion, no list
    // updates, exactly what "動作がスムーズではない" reported. `apps/ft8.rs::run_forever`
    // has the same fix for the same reason (its decode is also not idle-friendly): raise
    // this thread — which is about to become the panel loop — above the feed thread,
    // matching `display::PANEL_PRIORITY`.
    if PANEL_ON_CORE1 {
        // Diagnostic: core 0 left to `Back` alone, as in `jtty-bench`, to test whether the
        // panel sharing core 0 is what `Back` falls behind by. Not the design — in the real
        // mode core 1 carries WiFi and lwIP (docs/notes/JTTY_CORES3_APP.md §4). Same stack
        // and priority FST4 gives its core-1 panel.
        app::display::spawn_log_panel(
            app::boot::Display { i2c0: peripherals.i2c0, spi2: peripherals.spi2, pins: peripherals.pins },
            nvs,
            BootMode::Decode,
            24 * 1024,
            app::display::PANEL_PRIORITY,
        );
        loop {
            std::thread::sleep(std::time::Duration::from_secs(60));
        }
    }

    unsafe {
        esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), app::display::PANEL_PRIORITY)
    };

    app::display::run_log_panel(
        peripherals.i2c0,
        peripherals.spi2,
        peripherals.pins,
        &app::FANOUT,
        nvs,
        BootMode::Decode,
    )
}

/// `MFSK_JTTY_DEMO_QUIET=1`: no log line per message update. Measurement switch: the
/// updates arrive from inside `Back::process`, so a console that blocks on them is time
/// taken from `Back`.
const QUIET_UPDATES: bool = option_env!("MFSK_JTTY_DEMO_QUIET").is_some();

/// `MFSK_JTTY_DEMO_PANEL_CORE1=1`: run the panel as a core-1 task instead of on `main`
/// (core 0). A diagnostic switch, compile-time like the crate's other measurement knobs.
const PANEL_ON_CORE1: bool = option_env!("MFSK_JTTY_DEMO_PANEL_CORE1").is_some();

/// Build the busy-pattern audio once, then loop it through a fresh `Front`/`Back` pair
/// forever — each pass is its own stream, exactly as a real reset would give the receiver
/// a clean sample count instead of one that runs backwards when the recording wraps.
fn feed_loop() {
    let case = pileups(1)
        .into_iter()
        .find(|c| c.pattern.starts_with("band, 6 long messages"))
        .expect("jtty-demo: \"band, 6 long messages\" pattern not found in testsig::pileups");
    let audio = Arc::new(case.audio().expect("jtty-demo: pileups case did not pack"));
    log::info!(
        "jtty-demo: replaying \"{}\" #{} — {} stations, {:.1} s, forever",
        case.pattern,
        case.trial,
        case.stations.len(),
        audio.len() as f32 / 12_000.0
    );

    // Build the receiver while allocations up to 64 KB prefer internal DRAM, then restore the
    // board's 2 KB rule — exactly what `jtty-bench` does. `Receiver::new().with_f32_metrics()`
    // allocates its hot buffers once, here: the trellis survivors (2 x 32 KB), the aligned
    // side-surface FFT buffer (32 KB) and the power row (~16 KB). At this board's
    // `SPIRAM_MALLOC_ALWAYSINTERNAL=2048` they otherwise land in PSRAM, where a ladder rung is
    // 410 ms instead of 190 and the side-surface transform 10 ms instead of 2.3
    // (docs/notes/JTTY_CORES3_APP.md §3). The first demo build did that: Front at 161% of
    // core 1, 16-17 of ~42 windows dropped, and the decode count swinging run to run.
    // Restored straight after, because the channel-0 surface is 152 B over 64 KiB and must
    // not follow it inside while decoding (same §3).
    let internal_free =
        || unsafe { esp_idf_svc::sys::heap_caps_get_free_size(esp_idf_svc::sys::MALLOC_CAP_INTERNAL) };
    let before = internal_free();
    unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(64 * 1024) };
    let rx = Arc::new(Receiver::new().with_f32_metrics());
    unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(2048) };
    log::info!(
        "jtty-demo: receiver built, {} KB of internal DRAM taken (hot buffers ~112 KB expected)",
        (before - internal_free()) / 1024
    );
    let params = Params::default().embedded();
    let mut pass = 0u32;

    loop {
        pass += 1;
        let audio = audio.clone();
        let (tx, rq) = std::sync::mpsc::sync_channel::<Prepared>(QUEUE_DEPTH);
        let depth = Arc::new(AtomicUsize::new(0));
        let dropped = Arc::new(AtomicUsize::new(0));
        // Front's own time, µs, over the pass (a pass is ~6 s of it: u32 holds it).
        let front_us = Arc::new(AtomicU32::new(0));
        let t_start = now_us();

        let front_cfg = ThreadSpawnConfiguration {
            name: Some(c"jtty_front"),
            stack_size: FRONT_STACK,
            priority: 4,
            pin_to_core: Some(Core::Core1),
            ..ThreadSpawnConfiguration::default()
        };
        let _ = front_cfg.set();
        let front_handle = {
            let (rx, audio, depth, dropped, front_us) =
                (rx.clone(), audio.clone(), depth.clone(), dropped.clone(), front_us.clone());
            std::thread::Builder::new().stack_size(FRONT_STACK).spawn(move || {
                let mut front = Front::new(rx, params).expect("embedded settings");
                for (i, chunk) in audio.chunks(STEP).enumerate() {
                    let due = t_start + ((i + 1) * STEP) as i64 * 1_000_000 / 12_000;
                    let wait = due - now_us();
                    if wait > 0 {
                        std::thread::sleep(std::time::Duration::from_micros(wait as u64));
                    }
                    app::waterfall_feed::push(chunk);
                    let mut room = || depth.load(Ordering::Relaxed) < QUEUE_DEPTH;
                    let mut ready = Vec::new();
                    let t = now_us();
                    front.push_or_drop(chunk, &mut room, &mut |p| ready.push(p));
                    front_us.fetch_add((now_us() - t) as u32, Ordering::Relaxed);
                    dropped.store(front.dropped(), Ordering::Relaxed);
                    for p in ready {
                        depth.fetch_add(1, Ordering::Relaxed);
                        if tx.send(p).is_err() {
                            return;
                        }
                    }
                }
            })
        };
        let _ = ThreadSpawnConfiguration::default().set();
        let Ok(front_handle) = front_handle else {
            log::error!("jtty-demo: front thread not started; retrying next pass");
            std::thread::sleep(std::time::Duration::from_secs(1));
            continue;
        };

        let mut back = Back::new(rx.clone(), params);
        let mut n_complete = 0u32;
        // The queue at its fullest, counting the window just taken: the margin left under
        // `QUEUE_DEPTH`, which the drop count alone only shows once it is gone.
        let mut deepest = 0usize;
        // Per pass, as `jtty-bench`'s CASE line reports them: Back's time a window, mean and
        // worst, which says whether the work fits where the drop count only says where the
        // backlog sat. And the update callback's share of it (the logging below), apart.
        let (mut windows, mut back_total, mut back_worst) = (0i64, 0i64, 0i64);
        let (mut cb_total, mut cb_worst) = (0i64, 0i64);
        while let Ok(p) = rq.recv() {
            deepest = deepest.max(depth.fetch_sub(1, Ordering::Relaxed));
            let t = now_us();
            back.process(p, &mut |u| {
                let tc = now_us();
                on_update(&mut n_complete, u);
                let dc = now_us() - tc;
                cb_total += dc;
                cb_worst = cb_worst.max(dc);
            });
            let dt = now_us() - t;
            windows += 1;
            back_total += dt;
            back_worst = back_worst.max(dt);
        }
        back.finish(&mut |u| on_update(&mut n_complete, u));
        let _ = front_handle.join();

        let w = windows.max(1);
        log::info!(
            "jtty-demo: pass {pass} done — {n_complete}/{} messages, {} window(s) dropped, \
             queue at most {deepest} of {QUEUE_DEPTH} | {windows} windows: front {} ms, back {} ms \
             mean {} worst, of which update callbacks {} ms in all (worst {} ms)",
            case.stations.len(),
            dropped.load(Ordering::Relaxed),
            i64::from(front_us.load(Ordering::Relaxed)) / w / 1000,
            back_total / w / 1000,
            back_worst / 1000,
            cb_total / 1000,
            cb_worst / 1000,
        );
        log_waterfall_peaks(&case.stations);
    }
}

/// Where the waterfall puts its bright columns, against where the scene's stations are —
/// a check that the rows are the band's spectrum at the right frequencies whatever builds
/// them. Every held row is max-held per column (a station's tones hop), and every
/// column at level 10 of 15 or more that is a local maximum is reported at its centre
/// frequency. A station's tones sit from its `f0_hz` up.
fn log_waterfall_peaks(stations: &[mfsk_core::jtty::testsig::Station<'static>]) {
    use mfsk_app_shared::ui::state::WF_COLS;
    let Ok(ui) = UI.lock() else { return };
    let mut held = [0u8; WF_COLS];
    // Every row the panel holds (100, ~17 s): the scene's messages end before its 20 s
    // do, so the last couple of seconds alone are mostly noise.
    for row in ui.waterfall_iter() {
        for (h, &v) in held.iter_mut().zip(row.iter()) {
            *h = (*h).max(v & 0x0F);
        }
    }
    drop(ui);
    let lo = embedded_shared::waterfall::WF_FREQ_LO_HZ;
    let col_hz = (embedded_shared::waterfall::WF_FREQ_HI_HZ - lo) / WF_COLS as f32;
    let col_of = |f: f32| (((f - lo) / col_hz) as usize).min(WF_COLS - 1);
    // The band's floor: the median held level over every column.
    let mut sorted = held;
    sorted.sort_unstable();
    let floor = sorted[WF_COLS / 2];
    // Each station: its brightest held level over its own tones (f0 .. f0 + 100 Hz), and
    // the brightest over a same-width stretch 150 Hz below it, where nothing transmits.
    let mut out: heapless::String<200> = heapless::String::new();
    for s in stations {
        let span = |a: f32| held[col_of(a)..=col_of(a + 100.0)].iter().copied().max().unwrap_or(0);
        let on = span(s.f0_hz);
        let off = if s.f0_hz - 150.0 >= lo { span(s.f0_hz - 150.0) } else { 0 };
        let _ = core::fmt::Write::write_fmt(&mut out, format_args!(" {:.0}:{on}/{off}", s.f0_hz));
    }
    log::info!("jtty-demo: waterfall level at station:on/150Hz-below (0-15), floor {floor}:{out}");
}

/// One `MessageUpdate`: log it, and — once it is complete — push it as a `DecodedRow` so
/// it shows on the shared panel. Growing-but-incomplete updates are logged only; live
/// in-place row updates are `docs/notes/JTTY_CORES3_APP.md` §9's E2, not this demo.
fn on_update(n_complete: &mut u32, u: mfsk_core::jtty::assemble::MessageUpdate) {
    if !QUIET_UPDATES {
        log::info!(
        "jtty-demo: {:>7.1} Hz {:>6.2} s {} \"{}\"",
        u.f1_hz,
        u.start_s,
        if u.complete { "done " } else { "     " },
        u.text
        );
    }
    if !u.complete {
        return;
    }
    *n_complete += 1;
    let Ok(mut ui) = UI.lock() else { return };
    let mut msg: heapless::String<22> = heapless::String::new();
    let _ = msg.push_str(&u.text[..u.text.len().min(22)]);
    // No real slot here; `id` (stable for the message's life) stands in for both fields,
    // the way a slot sequence would — a repeat of the same text still dedupes on `msg`.
    let seq = (u.id & 0xffff_ffff) as u32;
    ui.push_decode(DecodedRow {
        df_hz: u.f1_hz.round().clamp(0.0, 65_535.0) as u16,
        snr_db: 0,
        hard_errors: 0,
        dt_ds: 0,
        msg,
        slot_seq: seq,
        first_seq: seq,
    });
}
