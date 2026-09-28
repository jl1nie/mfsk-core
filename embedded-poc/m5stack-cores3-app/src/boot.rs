// SPDX-License-Identifier: GPL-3.0-or-later
//! One boot sequence, for every receiver in this binary.
//!
//! ## Why this exists
//!
//! This crate carries four receivers and used to carry four `main`s.
//! `main.rs` held the FT8 controller's, inline, and dispatched into
//! `apps::{ft4, wspr, fst4}::run`, each of which opened with its own
//! copy of the same seven steps in its own order:
//!
//! ```text
//! watchdog → arenas → NVS → worker tasks → panel → WiFi → go → forever
//! ```
//!
//! The copies drifted, and the drift was not cosmetic. The WiFi
//! decision existed four times (#381). The *sequence* after the
//! association existed twice — `net.rs` for three receivers and a
//! hand-rolled thread in `main.rs` for the FT8 controller, which had a
//! UDP-sink retry the shared one lacked and lacked the modem power
//! save the shared one had. Two receivers opened the `"mfsk"` NVS
//! namespace a second time to get a handle the first one already held.
//!
//! ## What a receiver still chooses
//!
//! The order above is **measured**, not aesthetic, and it is the part
//! that is now written once. Two things about it that look arbitrary
//! and are not:
//!
//! - **Arenas before anything else.** The decode scratch is reserved
//!   while the heap is whole: after WiFi and the USB host have taken
//!   their share the largest free internal block is 31 744 B, against
//!   155 648 B before they start (measured on this board, #163).
//! - **The WiFi driver before the decoder's workers.** Driver
//!   construction is synchronous and claims internal DRAM; only the
//!   association is backgrounded. A receiver that spawns its worker
//!   stacks first wins the race and the driver's allocation fails
//!   later, somewhere less legible. `fst4_app` and `wspr_app` spawn
//!   their scan tasks earlier than this — they are allowed to, because
//!   those tasks wait on a flag and allocate nothing until
//!   [`Receiver::start`] sets it.
//!
//! So a receiver implements the differences and nothing else: which
//! arenas, which tasks, which panel shape, which network policy, and
//! what to do forever.

use std::sync::{Arc, Mutex};

use esp_idf_svc::hal::gpio::Pins;
use esp_idf_svc::hal::i2c::I2C0;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::spi::SPI2;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use mfsk_app_shared::boot_mode::BootMode;

/// The panel's peripherals, kept together so the boot sequence can
/// hand them to whichever of the two panel shapes a receiver uses
/// without splitting `Peripherals` twice.
pub struct Display {
    pub i2c0: I2C0<'static>,
    pub spi2: SPI2<'static>,
    pub pins: Pins,
}

/// What every receiver is handed.
///
/// One NVS handle, on the `"mfsk"` namespace that `boot_mode`,
/// `grid_src`, `wifi_pref` and `settings` all share. It is an
/// `Arc<Mutex<_>>` because the panel task commits mode changes from
/// its own thread; it used to be opened a second time by the receivers
/// that needed that shape.
pub struct BootCtx {
    pub nvs_part: EspDefaultNvsPartition,
    pub nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
    pub mode: BootMode,
}

/// What [`Receiver::attach_panel`] did with the peripherals.
///
/// Two shapes, because two are what the receivers need: WSPR and FST4
/// draw from a task and spend their own thread in a scan loop, while
/// the FT8 controller and FT4 make the panel *be* the main loop. The
/// enum carries the peripherals back for the second kind rather than
/// letting a receiver stash them, so "who owns the panel" stays a
/// question the type answers.
pub enum Panel {
    /// Drawn by a task this receiver already spawned.
    Spawned,
    /// Drawn by [`Receiver::run_forever`], which never returns.
    DrawsLast(Display),
}

/// One receiver in this binary: the parts of the boot that differ.
pub trait Receiver {
    /// Log prefix, and the name in the banner.
    const TAG: &'static str;

    /// Whether to turn the task watchdog off.
    ///
    /// The three whole-receiver apps do; the FT8 controller never has,
    /// and its `task_wdt(IDLE0)` lines during a cold acquisition are a
    /// known, watched symptom rather than noise — silencing them here
    /// would have deleted a signal while refactoring, which is how a
    /// refactor stops being one.
    const DEINIT_WDT: bool = true;

    /// Whether to hold the USB host install until the UDP log sink is
    /// up — see [`crate::wait_for_log_sink`]. The FT8 controller does;
    /// nothing else ever has.
    const WAIT_FOR_LOG_SINK: bool = false;

    /// Step 1: arenas, scratch, and any state the tasks below will
    /// read — all of it while the heap is whole.
    fn prepare(_ctx: &BootCtx) {}

    /// Step 2: tasks that wait for [`Receiver::start`] before they
    /// allocate. Anything that takes its stack *now* belongs in
    /// `start` instead, after the WiFi driver has had its share.
    fn spawn_workers(_ctx: &BootCtx) {}

    /// Step 3: the panel.
    fn attach_panel(ctx: &BootCtx, display: Display) -> Panel;

    /// Step 4: the network, or `None` for a receiver that wants no
    /// radio at all. The three ordinary ways to end up without one —
    /// `WIFI: OFF`, an empty SSID, a driver that will not start — are
    /// `net::bring_up`'s to decide, not a receiver's.
    fn net_config(ctx: &BootCtx) -> Option<crate::net::Config>;

    /// Step 5: let the decoding start. Runs after the WiFi driver has
    /// claimed its internal DRAM.
    fn start(_ctx: &BootCtx) {}

    /// Step 6: never returns.
    fn run_forever(ctx: BootCtx, panel: Panel) -> !;
}

/// Run one receiver through the shared sequence.
pub fn run<R: Receiver>(
    mode: BootMode,
    peripherals: Peripherals,
    nvs_part: EspDefaultNvsPartition,
    nvs: EspNvs<NvsDefault>,
) -> ! {
    log::info!(
        "=== mfsk-core-m5stack-cores3-app {} boot === mfsk-core {}",
        R::TAG,
        mfsk_core::VERSION
    );
    if R::DEINIT_WDT {
        // SAFETY: no arguments, and idempotent — the IDF returns an
        // error code rather than trapping if it was never started.
        let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
        log::info!("{}: task watchdog deinit -> {r}", R::TAG);
    }

    let ctx = BootCtx {
        nvs_part,
        nvs: Arc::new(Mutex::new(nvs)),
        mode,
    };
    crate::set_wait_for_log_sink(R::WAIT_FOR_LOG_SINK);
    // Before anything can deliver audio: the ring is what the audio
    // paths append to, for every mode alike.
    crate::waterfall_feed::init(mode);
    // What "heard this slot" is measured against on the station list —
    // one slot for the slotted modes, a few seconds for JTTY, which has
    // none (`BootMode::fresh_row_ms`).
    if let Ok(mut ui) = mfsk_app_shared::ui::state::UI.lock() {
        ui.set_slot_period_ms(mode.fresh_row_ms());
    }
    R::prepare(&ctx);

    let display = Display {
        i2c0: peripherals.i2c0,
        spi2: peripherals.spi2,
        pins: peripherals.pins,
    };
    // Taken out of `Peripherals` here so `net::bring_up` can own driver
    // construction for everyone. It is consumed by value and is not
    // handed back on `Err`, which is why it used to have to live in
    // each receiver's own `run`.
    let modem = peripherals.modem;

    R::spawn_workers(&ctx);
    let panel = R::attach_panel(&ctx, display);
    crate::net::bring_up(
        modem,
        ctx.nvs_part.clone(),
        ctx.nvs.clone(),
        R::net_config(&ctx),
    );
    // After WiFi has taken its contiguous internal DRAM: the storage
    // task's stack is internal by necessity (a flash op from a PSRAM
    // stack aborts), and 5 KB of it is better taken from what is left
    // than carved out of the block WiFi needs.
    // `MFSK_STORAGE_OFF=1`: the A/B knob for whether the flash writes
    // cost the decoder anything — compile-time, bench only.
    if option_env!("MFSK_STORAGE_OFF").is_none() {
        crate::storage::enable();
    } else {
        log::warn!("storage: MFSK_STORAGE_OFF — no logs this boot");
    }
    R::start(&ctx);
    R::run_forever(ctx, panel)
}
