// SPDX-License-Identifier: GPL-3.0-or-later
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
use std::sync::atomic::{AtomicUsize, Ordering};

use esp_idf_hal::cpu::Core;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::ui::state::{DecodedRow, UI};
use mfsk_core::jtty::rx::{Back, Front, Params, Prepared, Receiver, STEP};
use mfsk_core::jtty::testsig::pileups;

use mfsk_core_m5stack_cores3_app as app;

/// The two cores' queue depth. Matches `jtty-bench`'s own finding (#512, PR §14): a depth
/// of 6 loses no windows on this same busy pattern on this board.
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
    app::waterfall_feed::init(BootMode::Decode);
    if let Ok(mut ui) = UI.lock() {
        ui.set_slot_period_ms(UI_SLOT_PERIOD_MS);
    }

    let spawn = app::board::spawn_named(c"jttyfeed", FRONT_STACK, feed_loop);
    if let Err(e) = spawn {
        log::error!("jtty-demo: feed thread spawn failed ({e})");
    }

    app::display::run_log_panel(
        peripherals.i2c0,
        peripherals.spi2,
        peripherals.pins,
        &app::FANOUT,
        nvs,
        BootMode::Decode,
    )
}

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

    let rx = Arc::new(Receiver::new().with_f32_metrics());
    let params = Params::default().embedded();
    let mut pass = 0u32;

    loop {
        pass += 1;
        let audio = audio.clone();
        let (tx, rq) = std::sync::mpsc::sync_channel::<Prepared>(QUEUE_DEPTH);
        let depth = Arc::new(AtomicUsize::new(0));
        let dropped = Arc::new(AtomicUsize::new(0));
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
            let (rx, audio, depth, dropped) = (rx.clone(), audio.clone(), depth.clone(), dropped.clone());
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
                    front.push_or_drop(chunk, &mut room, &mut |p| ready.push(p));
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
        while let Ok(p) = rq.recv() {
            depth.fetch_sub(1, Ordering::Relaxed);
            back.process(p, &mut |u| on_update(&mut n_complete, u));
        }
        back.finish(&mut |u| on_update(&mut n_complete, u));
        let _ = front_handle.join();

        log::info!(
            "jtty-demo: pass {pass} done — {n_complete}/{} messages, {} window(s) dropped",
            case.stations.len(),
            dropped.load(Ordering::Relaxed)
        );
    }
}

/// One `MessageUpdate`: log it, and — once it is complete — push it as a `DecodedRow` so
/// it shows on the shared panel. Growing-but-incomplete updates are logged only; live
/// in-place row updates are `docs/notes/JTTY_CORES3_APP.md` §9's E2, not this demo.
fn on_update(n_complete: &mut u32, u: mfsk_core::jtty::assemble::MessageUpdate) {
    log::info!(
        "jtty-demo: {:>7.1} Hz {:>6.2} s {} \"{}\"",
        u.f1_hz,
        u.start_s,
        if u.complete { "done " } else { "     " },
        u.text
    );
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
