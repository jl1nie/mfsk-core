// SPDX-License-Identifier: GPL-3.0-only
//! JTTY receiver — the boot mode (#499, E1 of `docs/notes/JTTY_CORES3_APP.md`).
//!
//! Receive only. What differs from every other mode here is that JTTY has no
//! slot: frames start when the sender likes, a message is several of them,
//! and **the sample count is the receiver's only clock** (§1, §4). So this
//! file is mostly about keeping that count honest between the radio and the
//! decoder, and about the two decode halves `jtty-bench` and `jtty-demo`
//! measured on this board:
//!
//! ```text
//! uac reader / SIM (core 0, 8)     jtty_front (core 1, 4)            jtty_back (core 0, 5)
//!   JttySink::push_samples           drain staging (gaps as zeros)      recv (generation, window)
//!     SampleClock: fill / reset  ──►   Front::push_or_drop  ──► queue ──►  Back::process
//!     Staging: positioned gaps         room = queue < 6                    complete → row + all.txt
//! ```
//!
//! What jtty-demo (#518) settled, and is simply done here:
//! - the receiver is built while allocations up to 64 KB prefer internal DRAM
//!   — its trellis survivors, FFT buffer and power row in PSRAM put Front at
//!   161 % of core 1 and dropped 16 of ~42 windows;
//! - both stacks are internal. #516's "PSRAM stack" figures never ran a PSRAM
//!   stack (§12 of the design note), and Back's stack is reserved from the
//!   worker arena in `prepare`, before anything else can fragment the heap —
//!   at `start` a 32 KB internal stack could no longer be found;
//! - the panel runs above Back (`display::PANEL_PRIORITY`): below it, it
//!   starved for 20 s at a time;
//! - the waterfall uses the column-width transform and draws no slot rules
//!   (`waterfall_feed::FeedConfig::for_mode`).
//!
//! **No WiFi in this mode** (user decision, 2026-09-28, §6): the WiFi driver's
//! ~105 KB of internal DRAM is what this receiver needs. No UDP console, no
//! NTP, no config page; the `all.txt` anchor comes from the BM8563 RTC, which
//! `pmic::init` reads into the system clock at every boot.
//!
//! **Not here yet**: rows that grow while a message is still arriving (E2 —
//! a row is published once, on `complete`), an S/N for the row and the log
//! (E2; the row shows `+0` as every mode does for a non-finite S/N, and
//! `all.txt` leaves both columns blank), the operator's `f0` / `ftol` as a
//! setting (a fixed default, §6), and on-air use (E3).

use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use std::sync::mpsc::{sync_channel, Receiver as ChanRx, SyncSender};
use std::sync::{Arc, Mutex, OnceLock};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use mfsk_app_shared::jtty_rx_clock::{SampleClock, Staging, Verdict};
use mfsk_app_shared::ui::state::{SlotDecode, UI};
use mfsk_core::jtty::assemble::MessageUpdate;
use mfsk_core::jtty::rx::{Back, Front, Params, Prepared, Receiver as Decoder, NCHUNK, STEP};

use crate::boot::{BootCtx, Display, Panel, Receiver};

/// Windows that may wait between the cores: 6, `jtty-bench`'s finding on the
/// six-station scene (#512, `JTTY_EMBEDDED_BUDGET.md` §14). 7 was tried on
/// jtty-demo and only moved a drop later (#518).
const QUEUE_DEPTH: usize = 6;

/// Audio staged between the sink and the front end: 4 s, as FT4's. The
/// front end drains it every few tens of milliseconds and holds it only for
/// one `push` (≲ 0.5 s at the worst window measured).
const STAGING_CAP: usize = 48_000;

/// Front's peak on the golden and band6 scenes was 2 576 B of a 16 KB stack
/// (13 808 B free at worst, `[stacks]`, 2026-09-28): 8 KB is 3.2x that.
const FRONT_STACK: usize = 8 * 1024;
/// Back's peak was 13 088 B of 32 KB (19 680 B free at worst, band6,
/// 2026-09-28) — the ladder's correlations live on it. 20 KB is 1.56x that,
/// with `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK` as the net if a scene goes
/// deeper. Taken from `embedded_shared::worker_arena` in `prepare`.
const BACK_STACK: usize = 20 * 1024;
const FRONT_PRIORITY: u8 = 4;
const BACK_PRIORITY: u8 = 5;

/// How often the per-run figures (§7) are logged.
const REPORT_US: i64 = 30_000_000;

/// The receiver's tables and scratch, built once in `prepare` (a second
/// build moved ~32 KB to PSRAM and halved a busy band's speed, §14).
static DECODER: OnceLock<Arc<Decoder>> = OnceLock::new();

/// What arrives from the radio (or the SIM feed), and its clock.
struct Sink {
    staging: Staging,
    clock: SampleClock,
    /// Bumped by a reset: a window from an older generation never reaches a
    /// `Back` built for a newer one, and message ids (which restart at 1 in a
    /// new `Assembler`) cannot be confused across it.
    generation: u32,
    /// UTC ms of the current stream's sample 0, for `all.txt`; `None` without
    /// a clock.
    anchor_unix_ms: Option<i64>,
    /// `time_sync::clock_epoch()` the anchor was taken at. The clock can be
    /// set after the stream starts — the BM8563 is read by the panel's
    /// `pmic::init`, after `start` — so the anchor is taken again whenever
    /// the clock's source changes.
    anchor_epoch: Option<u32>,
    /// `esp_timer` µs of the current stream's sample 0, for the decode delay.
    t0_us: Option<i64>,
}

static SINK: Mutex<Option<Sink>> = Mutex::new(None);

// Counters for the status strip and the report. u32 atomics: Xtensa has no
// 64-bit ones.
static GAPS_FILLED: AtomicU32 = AtomicU32::new(0); // samples of zeros inserted
static OVERFLOWED: AtomicU32 = AtomicU32::new(0); // samples lost to a full ring
static RESETS: AtomicU32 = AtomicU32::new(0);
static DROPPED: AtomicU32 = AtomicU32::new(0); // windows, all generations
static QUEUED: AtomicUsize = AtomicUsize::new(0);
static FRONT_US: AtomicU32 = AtomicU32::new(0); // since the last report
static FRONT_WINDOWS: AtomicU32 = AtomicU32::new(0); // windows Front finished (sent or dropped), idem
/// Back is inside `Back::process`. Front files each push's time under whether
/// Back was working through all of it, none of it, or part: the question is
/// whether Front runs slower while the other core is decoding (the two share
/// the data cache and the PSRAM bus), which a per-core CPU share cannot show.
static BACK_BUSY: AtomicBool = AtomicBool::new(false);
/// [idle, busy, mixed] × (µs, windows), since the last report.
static FRONT_BUCKET_US: [AtomicU32; 3] = [const { AtomicU32::new(0) }; 3];
static FRONT_BUCKET_WINDOWS: [AtomicU32; 3] = [const { AtomicU32::new(0) }; 3];

/// Front to Back: a window, or the start of a new stream.
enum Msg {
    Window { generation: u32, prepared: Prepared },
    Reset { generation: u32 },
}

pub struct JttyRx;

impl Receiver for JttyRx {
    const TAG: &'static str = "jtty_app";

    fn prepare(_ctx: &BootCtx) {
        // The esp-dsp tables the receiver plans (256 and 2 048 points), while
        // the heap is whole: installed lazily they would come after WiFi.
        embedded_shared::esp_dsp_fft::prewarm(256);
        embedded_shared::esp_dsp_fft::prewarm(2048);
        let (before, largest_before, psram_before) = heap_now();
        let blocks_before = heap_blocks();
        log_free_internal(&blocks_before);
        // Allocations up to 64 KB prefer internal DRAM while the receiver is
        // built, then the board's 2 KB rule again: the channel-0 surface is
        // 152 B over 64 KiB and must not follow into internal DRAM while
        // decoding (§3). Largest allocation first: built in `new().with_f32_metrics()`'s
        // order, both trellis survivor arrays came last and landed in PSRAM (§13).
        unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(64 * 1024) };
        let decoder = Arc::new(Decoder::new_with_f32_metrics());
        unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(2048) };
        let (after, largest_after, psram_after) = heap_now();
        // `jtty-bench` and `jtty-demo` took 152-154 KB internal for the same
        // build; less here means a hot buffer went to PSRAM.
        log::info!(
            "jtty_app: receiver built — internal {} B taken, PSRAM {} B taken | internal before {} B (largest {} B), after {} B (largest {} B)",
            before - after,
            psram_before - psram_after,
            before,
            largest_before,
            after,
            largest_after
        );
        log_new_blocks(&blocks_before, &heap_blocks());
        let _ = DECODER.set(decoder);
        // Back's stack, while the heap is still whole (worker_arena's reason).
        if embedded_shared::worker_arena::reserve(embedded_shared::worker_arena::Owner::JttyBack, BACK_STACK) {
            let (f, l, _) = heap_now();
            log::info!("jtty_app: Back's {BACK_STACK} B stack reserved — internal now {f} B (largest {l} B)");
        }
        // The storage task's too: spawned lazily as in every mode, it found no
        // 5 KB internal block once this receiver ran, and wrote no all.txt (§14).
        if crate::storage::reserve_stack() {
            let (f, l, _) = heap_now();
            log::info!("jtty_app: storage stack reserved — internal now {f} B (largest {l} B)");
        }
        if let Ok(mut g) = SINK.lock() {
            *g = Some(Sink {
                staging: Staging::new(STAGING_CAP),
                clock: SampleClock::new(),
                generation: 0,
                anchor_unix_ms: None,
                anchor_epoch: None,
                t0_us: None,
            });
        }
    }

    fn attach_panel(_ctx: &BootCtx, display: Display) -> Panel {
        Panel::DrawsLast(display)
    }

    fn net_config(_ctx: &BootCtx) -> Option<crate::net::Config> {
        // Never (§6, user decision 2026-09-28): beside the WiFi driver internal
        // DRAM ran to 1-7 KB free and the receiver dropped windows (§12).
        log::warn!(
            "jtty_app: no WiFi in JTTY mode — no UDP log, no NTP, no config page; all.txt time from the RTC"
        );
        None
    }

    fn start(_ctx: &BootCtx) {
        let params = Params::default().embedded();
        log::info!(
            "jtty_app: receiving {:.0} Hz ± {:.0} Hz and the band scan, queue {QUEUE_DEPTH}",
            params.f0_hz,
            params.ftol_hz
        );
        let (tx, rx) = sync_channel::<Msg>(QUEUE_DEPTH + 2);
        spawn_back(rx, params);
        spawn_front(tx, params);
        crate::uac::set_audio_sink(JttySink { announced: false });
        sim_feed_if_asked();
    }

    fn run_forever(ctx: BootCtx, panel: Panel) -> ! {
        let Panel::DrawsLast(display) = panel else {
            unreachable!("jtty draws from run_forever");
        };
        // Above Back, as FT8's panel is above its decode (§4): Back is busy
        // continuously, and below it the panel starved (jtty-demo, #518).
        unsafe {
            esp_idf_svc::sys::vTaskPrioritySet(core::ptr::null_mut(), crate::display::PANEL_PRIORITY)
        };
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

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// The audio callback's half: stage the samples and keep the clock. Never
/// blocks on anything but the staging lock, which the front end holds only
/// to swap the buffer out.
struct JttySink {
    announced: bool,
}

impl crate::uac::AudioSink for JttySink {
    fn push_samples(&mut self, samples: &[i16]) {
        if !self.announced {
            self.announced = true;
            log::info!(
                "jtty_app: {} audio active",
                if crate::uac::sim_feeding() { "SIM" } else { "real UAC" }
            );
        }
        let now = now_us();
        let Ok(mut g) = SINK.lock() else { return };
        let Some(s) = g.as_mut() else { return };
        // Judged before the block is staged: what is missing is missing
        // *before* these samples, so that is where its zeros go.
        match s.clock.delivered(samples.len(), now) {
            Verdict::Ok => {}
            Verdict::Fill(n) => {
                s.staging.gap(n);
                s.clock.filled(n);
                GAPS_FILLED.fetch_add(n as u32, Ordering::Relaxed);
                log::warn!("jtty_app: {} ms of audio missing — filled with zeros", n * 1000 / 12_000);
            }
            Verdict::Reset => {
                // Too much to fill: a new stream starts with this block.
                s.generation = s.generation.wrapping_add(1);
                s.staging.clear();
                s.clock = SampleClock::new();
                s.anchor_unix_ms = None;
                s.anchor_epoch = None;
                s.t0_us = None;
                RESETS.fetch_add(1, Ordering::Relaxed);
                log::warn!(
                    "jtty_app: audio stopped for more than a second — stream reset (generation {})",
                    s.generation
                );
                let _ = s.clock.delivered(samples.len(), now);
            }
        }
        let t0 = *s.t0_us.get_or_insert(s.clock.t0_us().unwrap_or(now));
        let epoch = mfsk_app_shared::time_sync::clock_epoch();
        if s.anchor_epoch != Some(epoch) {
            // Sample 0's UTC: now, less the audio since (the clock
            // reconciliation keeps samples and `esp_timer` within ~20 ms).
            s.anchor_epoch = Some(epoch);
            s.anchor_unix_ms = mfsk_app_shared::time_sync::utc_now_ms().map(|ms| ms as i64 - (now - t0) / 1_000);
            log::info!(
                "jtty_app: stream {} all.txt anchor {:?} ms, clock from {:?}",
                s.generation,
                s.anchor_unix_ms,
                mfsk_app_shared::time_sync::clock_source()
            );
        }
        let lost = s.staging.push(samples);
        if lost > 0 {
            OVERFLOWED.fetch_add(lost as u32, Ordering::Relaxed);
        }
    }
}

/// The front end: core 1, internal stack. Drains the staging ring, starts a
/// new `Front` when the sink has reset the stream, and hands windows to Back
/// — or drops them when six are already waiting (`Front::push_or_drop`), so
/// the audio path never waits on the decoder.
fn spawn_front(tx: SyncSender<Msg>, params: Params) {
    let cfg = ThreadSpawnConfiguration {
        name: Some(c"jtty_front"),
        stack_size: FRONT_STACK,
        priority: FRONT_PRIORITY,
        pin_to_core: Some(Core::Core1),
        ..ThreadSpawnConfiguration::default()
    };
    if let Err(e) = cfg.set() {
        log::error!("jtty_app: front end thread config refused ({e}) — not spawning it");
        return;
    }
    let spawned = std::thread::Builder::new().stack_size(FRONT_STACK).spawn(move || {
        let decoder = DECODER.get().expect("prepare builds the receiver").clone();
        let mut front = Front::new(decoder.clone(), params).expect("embedded settings");
        let mut generation = 0u32;
        let mut block: Vec<i16> = Vec::with_capacity(STAGING_CAP);
        loop {
            block.clear();
            let now_gen = {
                let Ok(mut g) = SINK.lock() else { continue };
                let Some(s) = g.as_mut() else { continue };
                s.staging.drain_into(&mut block);
                s.generation
            };
            if now_gen != generation {
                generation = now_gen;
                front = Front::new(decoder.clone(), params).expect("embedded settings");
                if tx.send(Msg::Reset { generation }).is_err() {
                    return;
                }
            }
            if block.is_empty() {
                std::thread::sleep(core::time::Duration::from_millis(10));
                continue;
            }
            let t = now_us();
            let busy_at_start = BACK_BUSY.load(Ordering::Relaxed);
            let mut ready = Vec::new();
            let dropped_before = front.dropped();
            let mut room = || QUEUED.load(Ordering::Relaxed) < QUEUE_DEPTH;
            front.push_or_drop(&block, &mut room, &mut |p| ready.push(p));
            let dropped_now = (front.dropped() - dropped_before) as u32;
            DROPPED.fetch_add(dropped_now, Ordering::Relaxed);
            FRONT_US.fetch_add((now_us() - t) as u32, Ordering::Relaxed);
            FRONT_WINDOWS.fetch_add(dropped_now + ready.len() as u32, Ordering::Relaxed);
            // Only pushes that finished a window: the rest are FIR-only and short.
            let finished = dropped_now + ready.len() as u32;
            if finished > 0 {
                let b = match (busy_at_start, BACK_BUSY.load(Ordering::Relaxed)) {
                    (false, false) => 0,
                    (true, true) => 1,
                    _ => 2,
                };
                FRONT_BUCKET_US[b].fetch_add((now_us() - t) as u32, Ordering::Relaxed);
                FRONT_BUCKET_WINDOWS[b].fetch_add(finished, Ordering::Relaxed);
            }
            for prepared in ready {
                QUEUED.fetch_add(1, Ordering::Relaxed);
                if tx.send(Msg::Window { generation, prepared }).is_err() {
                    return;
                }
            }
        }
    });
    let _ = ThreadSpawnConfiguration::default().set();
    if let Err(e) = spawned {
        log::error!("jtty_app: front end spawn failed ({e})");
    }
}

/// The back end: core 0 below the panel, on the internal stack `prepare`
/// reserved. A FreeRTOS task rather than a pthread because a pthread's stack
/// is allocated at spawn, from whatever the heap has left by then.
fn spawn_back(rx: ChanRx<Msg>, params: Params) {
    use esp_idf_svc::sys;
    static mut BACK_TCB: core::mem::MaybeUninit<sys::StaticTask_t> = core::mem::MaybeUninit::uninit();

    let Some(stack) = embedded_shared::worker_arena::claim(embedded_shared::worker_arena::Owner::JttyBack, BACK_STACK)
    else {
        log::error!("jtty_app: no reserved stack for the back end — not spawning it");
        return;
    };
    unsafe extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `arg` is the box leaked below, handed to this task alone.
        let (rx, params) = *unsafe { Box::from_raw(arg as *mut (ChanRx<Msg>, Params)) };
        back_loop(rx, params);
        // A FreeRTOS task must not return.
        unsafe { sys::vTaskDelete(core::ptr::null_mut()) };
    }
    let arg = Box::into_raw(Box::new((rx, params))) as *mut core::ffi::c_void;
    // SAFETY: the stack is the arena block this mode reserved and nothing else
    // claims; the TCB is static and this runs once per boot.
    let h = unsafe {
        sys::xTaskCreateStaticPinnedToCore(
            Some(entry),
            c"jtty_back".as_ptr(),
            BACK_STACK as u32,
            arg,
            u32::from(BACK_PRIORITY),
            stack,
            core::ptr::addr_of_mut!(BACK_TCB) as *mut sys::StaticTask_t,
            0,
        )
    };
    if h.is_null() {
        log::error!("jtty_app: back end task not created");
        // SAFETY: the task never started, so the box is still ours.
        drop(unsafe { Box::from_raw(arg as *mut (ChanRx<Msg>, Params)) });
    }
}

/// Per-report figures (§7), kept by the back end.
#[derive(Default)]
struct Report {
    windows: u32,
    back_total_us: i64,
    back_worst_us: i64,
    delay_total_us: i64,
    delay_worst_us: i64,
    deepest: usize,
    stale: u32,
    messages: u32,
}

fn back_loop(rx: ChanRx<Msg>, params: Params) {
    let decoder = DECODER.get().expect("prepare builds the receiver").clone();
    let mut back = Back::new(decoder.clone(), params);
    let mut generation = 0u32;
    // Messages seen and not yet complete: while any is open, the band is not
    // quiet enough for a flash write (§5).
    let mut open: Vec<u64> = Vec::new();
    let mut rep = Report::default();
    let mut last_report = now_us();
    let mut last_status = 0i64;
    let mut last_quiet = 0i64;
    // The first report takes the baseline, each later one prints its span.
    let mut cpu = CpuTally::default();
    while let Ok(msg) = rx.recv() {
        match msg {
            Msg::Reset { generation: g } => {
                back.finish(&mut |u| {
                    if !u.complete {
                        log::info!("jtty_app: reset left \"{}\" incomplete", u.text);
                    }
                });
                back = Back::new(decoder.clone(), params);
                generation = g;
                open.clear();
            }
            Msg::Window { generation: g, prepared } => {
                let queued = QUEUED.fetch_sub(1, Ordering::Relaxed);
                rep.deepest = rep.deepest.max(queued);
                if g != generation {
                    rep.stale += 1;
                    continue;
                }
                let w = prepared.window();
                let t = now_us();
                BACK_BUSY.store(true, Ordering::Relaxed);
                back.process(prepared, &mut |u| on_update(u, &mut open, &mut rep));
                BACK_BUSY.store(false, Ordering::Relaxed);
                let end = now_us();
                let dt = end - t;
                rep.windows += 1;
                rep.back_total_us += dt;
                rep.back_worst_us = rep.back_worst_us.max(dt);
                // From the window's last sample reaching the board to its
                // decode being done.
                if let Some(t0) = SINK.lock().ok().and_then(|g| g.as_ref().and_then(|s| s.t0_us)) {
                    let audio_done = t0 + ((w * STEP + NCHUNK) as i64) * 1_000_000 / 12_000;
                    let delay = end - audio_done;
                    rep.delay_total_us += delay;
                    rep.delay_worst_us = rep.delay_worst_us.max(delay);
                }
            }
        }
        let now = now_us();
        // Quiet: nothing open, nothing waiting. Cheap to report; storage
        // decides whether it has held anything long enough to write.
        if open.is_empty() && QUEUED.load(Ordering::Relaxed) == 0 && now - last_quiet > 1_000_000 {
            last_quiet = now;
            crate::storage::quiet_rx();
        }
        if now - last_status > 1_000_000 {
            last_status = now;
            set_status_strip();
        }
        if now - last_report >= REPORT_US {
            log_report(&mut rep, now - last_report);
            if crate::uac::sim_feeding() {
                cpu.log();
            }
            last_report = now;
        }
    }
}

/// One message update. A row and an `all.txt` line only on `complete` in E1.
fn on_update(u: MessageUpdate, open: &mut Vec<u64>, rep: &mut Report) {
    if !u.complete {
        if !open.contains(&u.id) {
            open.push(u.id);
        }
        return;
    }
    open.retain(|&id| id != u.id);
    rep.messages += 1;
    log::info!("jtty_app: {:>7.1} Hz {:>7.2} s \"{}\"", u.f1_hz, u.start_s, u.text);
    let anchor = SINK.lock().ok().and_then(|g| g.as_ref().and_then(|s| s.anchor_unix_ms));
    let Ok(mut ui) = UI.lock() else { return };
    // `publish_slot` numbers every call, so a row's key is unique across a
    // stream reset even though message ids restart at 1.
    ui.publish_slot([SlotDecode {
        freq_hz: u.f1_hz,
        // No S/N yet (E2): non-finite draws `+0`, as for any mode without one.
        snr_db: f32::NAN,
        dt_sec: 0.0,
        text: &u.text,
        hard_errors: 0,
    }]);
    let dial_hz = ui.status.rig_freq_hz.map(u64::from);
    drop(ui);
    // Stamped from the stream's UTC anchor plus the message's own start, not
    // from a slot (§5). No clock, no line: a line stamped 1970 is worse.
    if let Some(a) = anchor {
        let unix = (a + (u.start_s * 1000.0) as i64) / 1000;
        crate::storage::append_all_txt_no_slot(mfsk_app_shared::all_txt::message_line(
            unix,
            dial_hz,
            "JTTY",
            u.f1_hz.round() as i32,
            &u.text,
        ));
    }
}

/// Queue depth, windows dropped, gaps filled — on the strip under the list (§5).
fn set_status_strip() {
    let mut s: heapless::String<32> = heapless::String::new();
    let _ = core::fmt::Write::write_fmt(
        &mut s,
        format_args!(
            "Q{} DROP{} GAP{}ms RST{}",
            QUEUED.load(Ordering::Relaxed),
            DROPPED.load(Ordering::Relaxed),
            GAPS_FILLED.load(Ordering::Relaxed) / 12,
            RESETS.load(Ordering::Relaxed)
        ),
    );
    if let Ok(mut ui) = UI.lock() {
        ui.set_acq_line(&s);
    }
}

fn log_report(rep: &mut Report, span_us: i64) {
    use esp_idf_svc::sys::{
        heap_caps_get_free_size, heap_caps_get_largest_free_block, heap_caps_get_minimum_free_size,
        MALLOC_CAP_INTERNAL, MALLOC_CAP_SPIRAM,
    };
    let w = i64::from(rep.windows.max(1));
    // Per window Front finished, sent or dropped — as `jtty-bench` divides it
    // (where none are dropped).
    let fw = i64::from(FRONT_WINDOWS.swap(0, Ordering::Relaxed).max(1));
    let front_ms = i64::from(FRONT_US.swap(0, Ordering::Relaxed)) / fw / 1000;
    // SAFETY: null = the calling task, which is Back.
    let back_hw = unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) };
    let (ifree, ilarge, imin, pfree, plarge) = unsafe {
        (
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024,
            heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024,
            heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024,
            heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024,
        )
    };
    log::info!(
        "jtty_app: {} s: {} windows ({} stale), {} messages | front {front_ms} ms, back {} ms mean {} worst, \
         delay {} ms mean {} worst | queue at most {} of {QUEUE_DEPTH}, dropped {} (all time), gaps {} ms, \
         overflow {} ms, resets {} | internal {ifree} KB free (min {imin}, largest {ilarge}), PSRAM {pfree} KB \
         (largest {plarge}) | Back stack {back_hw} B free of {BACK_STACK}",
        span_us / 1_000_000,
        rep.windows,
        rep.stale,
        rep.messages,
        rep.back_total_us / w / 1000,
        rep.back_worst_us / 1000,
        rep.delay_total_us / w / 1000,
        rep.delay_worst_us / 1000,
        rep.deepest,
        DROPPED.load(Ordering::Relaxed),
        GAPS_FILLED.load(Ordering::Relaxed) / 12,
        OVERFLOWED.load(Ordering::Relaxed) / 12,
        RESETS.load(Ordering::Relaxed),
    );
    // Every task's stack high-water: the panel logs it (`board::log_task_stacks`).
    *rep = Report::default();
}

/// One heap block of 1 KB or more: (address, size, used).
type Block = (usize, usize, bool);

/// Every heap block of 1 KB or more, used or free, across all heaps.
///
/// The walker runs under the heap lock, so it must not allocate: the Vec is
/// sized first and blocks past its capacity are counted, not stored.
fn heap_blocks() -> Vec<Block> {
    struct Acc {
        v: Vec<Block>,
        missed: usize,
    }
    unsafe extern "C" fn walk(
        _heap: esp_idf_svc::sys::walker_heap_into_t,
        b: esp_idf_svc::sys::walker_block_info_t,
        user: *mut core::ffi::c_void,
    ) -> bool {
        // SAFETY: `user` is the `Acc` below, alive for the whole walk.
        let acc = unsafe { &mut *(user as *mut Acc) };
        if b.size >= 1024 {
            if acc.v.len() < acc.v.capacity() {
                acc.v.push((b.ptr as usize, b.size, b.used));
            } else {
                acc.missed += 1;
            }
        }
        true
    }
    let mut acc = Acc { v: Vec::with_capacity(512), missed: 0 };
    // SAFETY: the callback only writes into `acc`, which outlives the call.
    unsafe { esp_idf_svc::sys::heap_caps_walk_all(Some(walk), &mut acc as *mut Acc as *mut core::ffi::c_void) };
    if acc.missed > 0 {
        log::warn!("jtty_app: heap walk: {} blocks past the snapshot's capacity", acc.missed);
    }
    acc.v
}

/// Internal DRAM or PSRAM, by address (ESP32-S3 data-bus windows).
fn region(addr: usize) -> &'static str {
    if (0x3FC8_8000..0x3FD0_0000).contains(&addr) {
        "internal"
    } else if (0x3C00_0000..0x3E00_0000).contains(&addr) {
        "PSRAM"
    } else {
        "other"
    }
}

/// The free internal blocks the build has to fit into.
fn log_free_internal(blocks: &[Block]) {
    let mut free: Vec<usize> =
        blocks.iter().filter(|b| !b.2 && region(b.0) == "internal").map(|b| b.1).collect();
    free.sort_unstable_by(|a, b| b.cmp(a));
    log::info!("jtty_app: free internal blocks >= 1 KB before the build (B): {free:?}");
}

/// What the build allocated, block by block: which hot buffer went where.
fn log_new_blocks(before: &[Block], after: &[Block]) {
    let mut new: Vec<&Block> =
        after.iter().filter(|b| b.2 && !before.iter().any(|o| o.2 && o.0 == b.0)).collect();
    new.sort_unstable_by(|a, b| b.1.cmp(&a.1));
    for (addr, size, _) in new.iter().filter(|b| b.1 >= 4096) {
        log::info!("jtty_app: build allocated {size} B at {addr:#x} ({})", region(*addr));
    }
    let small: usize = new.iter().filter(|b| b.1 < 4096).map(|b| b.1).sum();
    log::info!("jtty_app: build allocated {small} B more in blocks of 1-4 KB");
}

/// Internal free, internal largest block, PSRAM free — bytes.
fn heap_now() -> (usize, usize, usize) {
    use esp_idf_svc::sys::{heap_caps_get_free_size, heap_caps_get_largest_free_block, MALLOC_CAP_INTERNAL, MALLOC_CAP_SPIRAM};
    // SAFETY: read-only queries.
    unsafe {
        (
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
            heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        )
    }
}

/// Each task's CPU over a report span, grouped by the core it is pinned to
/// (`-` = unpinned) — FreeRTOS's run-time counters, as `board::log_task_cpu`,
/// but over 30 s rather than the panel's few seconds: a task's counter moves
/// only when it is switched out, so one busy stretch of Front (~0.4 s) lands
/// in whichever short interval it ends in (the panel's line read 189 %).
///
/// SIM only: `uxTaskGetSystemState` holds the scheduler while it walks every
/// stack, which costs a radio's isochronous packets (`board::log_task_cpu`).
#[derive(Default)]
struct CpuTally {
    prev: Vec<(usize, u32)>,
    prev_us: i64,
}

impl CpuTally {
    fn log(&mut self) {
        use esp_idf_svc::sys;
        const MAX_TASKS: usize = 40;
        let mut tasks: Vec<sys::TaskStatus_t> = Vec::with_capacity(MAX_TASKS);
        let mut total = 0u32;
        // SAFETY: room for MAX_TASKS entries; the call reports how many it wrote.
        let n = unsafe { sys::uxTaskGetSystemState(tasks.as_mut_ptr(), MAX_TASKS as u32, &mut total) } as usize;
        // SAFETY: the first `n` entries were written just now.
        unsafe { tasks.set_len(n.min(MAX_TASKS)) };
        let now = now_us();
        let wall = now - self.prev_us;
        let mut rows: Vec<(i32, u32, String)> = Vec::new();
        for t in &tasks {
            let h = t.xHandle as usize;
            if let Some(&(_, b)) = self.prev.iter().find(|(k, _)| *k == h) {
                // SAFETY: the handle was reported a moment ago; names are NUL-terminated.
                let core = unsafe { sys::xTaskGetCoreID(t.xHandle) };
                let core = if core as u32 == 0x7FFF_FFFF { -1 } else { core };
                let name = unsafe { core::ffi::CStr::from_ptr(t.pcTaskName) }.to_string_lossy().into_owned();
                rows.push((core, t.ulRunTimeCounter.wrapping_sub(b), name));
            }
        }
        let first = self.prev_us == 0;
        self.prev = tasks.iter().map(|t| (t.xHandle as usize, t.ulRunTimeCounter)).collect();
        self.prev_us = now;
        if first || wall <= 0 {
            return;
        }
        rows.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.cmp(&a.1)));
        let mut line = String::new();
        let mut core_seen = i32::MIN;
        for (core, d, name) in &rows {
            let pct10 = i64::from(*d) * 1000 / wall;
            if pct10 < 5 && !name.starts_with("IDLE") {
                continue;
            }
            if *core != core_seen {
                core_seen = *core;
                let label = if *core < 0 { "-".to_string() } else { core.to_string() };
                let _ = core::fmt::Write::write_fmt(&mut line, format_args!(" | core {label}:"));
            }
            let _ = core::fmt::Write::write_fmt(&mut line, format_args!(" {name} {}.{}%", pct10 / 10, pct10 % 10));
        }
        log::info!("jtty_app: cpu over {} s{line}", wall / 1_000_000);
        // Front's time a finished window, by what core 0's Back was doing.
        let mut f = String::new();
        for (i, label) in ["Back idle", "Back busy", "mixed"].iter().enumerate() {
            let us = FRONT_BUCKET_US[i].swap(0, Ordering::Relaxed);
            let n = FRONT_BUCKET_WINDOWS[i].swap(0, Ordering::Relaxed);
            let _ = core::fmt::Write::write_fmt(
                &mut f,
                format_args!(" | {label}: {n} windows, {} ms", if n > 0 { us / n / 1000 } else { 0 }),
            );
        }
        log::info!("jtty_app: front per window{f}");
    }
}

/// `MFSK_CORES3_SIM`: feed baked audio through the real [`JttySink`], as one
/// continuous stream (`uac::spawn_sim_feed_continuous`: no slot re-alignment
/// cutting samples, §7). `MFSK_JTTY_SIM_SCENE` picks it:
///
/// - `golden` (default): WSJT-X's JTTY sample recording
///   (`assets/golden/jtty/260807_134110.wav`, one station, 30 s);
/// - `band6`: `testsig::pileups`' "band, 6 long messages", trial 0 — the
///   scene jtty-demo and jtty-bench measured.
///
/// `MFSK_SIM_NO_CLOCK` takes the clock away, as for the other modes: no
/// `all.txt` lines then, which is the point of trying it.
fn sim_feed_if_asked() {
    if option_env!("MFSK_CORES3_SIM").is_none() {
        return;
    }
    if option_env!("MFSK_SIM_NO_CLOCK").is_some() {
        mfsk_app_shared::time_sync::suppress_clock(true);
        log::warn!("jtty_app SIM: clock suppressed — no all.txt anchor this run");
    }
    const GOLDEN: &[u8] = include_bytes!("../../../assets/golden/jtty/260807_134110.wav");
    let scene = option_env!("MFSK_JTTY_SIM_SCENE").unwrap_or("golden");
    let src = match scene {
        "band6" => {
            let case = mfsk_core::jtty::testsig::pileups(1)
                .into_iter()
                .find(|c| c.pattern.starts_with("band, 6 long messages"))
                .expect("band, 6 long messages is a testsig::pileups pattern");
            let audio = case.audio().expect("the band6 scene packs");
            let bytes: Vec<u8> = audio.iter().flat_map(|s| s.to_le_bytes()).collect();
            crate::uac::SimSource::Pcm(Box::leak(bytes.into_boxed_slice()))
        }
        "golden" => crate::uac::SimSource::Wav(GOLDEN),
        other => {
            log::error!("jtty_app SIM: unknown MFSK_JTTY_SIM_SCENE '{other}' — not feeding");
            return;
        }
    };
    log::warn!("jtty_app SIM: scene '{scene}', looped as one stream");
    crate::uac::spawn_sim_feed_continuous(src);
}
