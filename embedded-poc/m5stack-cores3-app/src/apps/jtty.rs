// SPDX-License-Identifier: GPL-3.0-or-later
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
//! - Front's stack is internal (in PSRAM it cost +25 % a window, #516), Back's
//!   is PSRAM (measured free, #516) and 32 KB (the demo's 16 KB peaked with
//!   ~3 KB left);
//! - the panel runs above Back (`display::PANEL_PRIORITY`): below it, it
//!   starved for 20 s at a time;
//! - the waterfall uses the column-width transform and draws no slot rules
//!   (`waterfall_feed::FeedConfig::for_mode`).
//!
//! **Not here yet**: rows that grow while a message is still arriving (E2 —
//! a row is published once, on `complete`), an S/N for the row and the log
//! (E2; the row shows `+0` as every mode does for a non-finite S/N, and
//! `all.txt` leaves both columns blank), the operator's `f0` / `ftol` as a
//! setting (a fixed default, §6), and on-air use (E3).

use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::mpsc::{sync_channel, Receiver as ChanRx, SyncSender};
use std::sync::{Arc, Mutex, OnceLock};

use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
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

const FRONT_STACK: usize = 16 * 1024;
const BACK_STACK: usize = 32 * 1024;
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
        let internal = || unsafe {
            esp_idf_svc::sys::heap_caps_get_free_size(esp_idf_svc::sys::MALLOC_CAP_INTERNAL)
        };
        let before = internal();
        let largest_before = unsafe {
            esp_idf_svc::sys::heap_caps_get_largest_free_block(esp_idf_svc::sys::MALLOC_CAP_INTERNAL)
        };
        // Allocations up to 64 KB prefer internal DRAM while the receiver is
        // built, then the board's 2 KB rule again: the channel-0 surface is
        // 152 B over 64 KiB and must not follow into internal DRAM while
        // decoding (§3).
        unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(64 * 1024) };
        let decoder = Arc::new(Decoder::new().with_f32_metrics());
        unsafe { esp_idf_svc::sys::heap_caps_malloc_extmem_enable(2048) };
        log::info!(
            "jtty_app: receiver built — {} KB of internal DRAM taken ({} KB free before, largest block {} KB), {} KB left",
            (before - internal()) / 1024,
            before / 1024,
            largest_before / 1024,
            internal() / 1024
        );
        let _ = DECODER.set(decoder);
        if let Ok(mut g) = SINK.lock() {
            *g = Some(Sink {
                staging: Staging::new(STAGING_CAP),
                clock: SampleClock::new(),
                generation: 0,
                anchor_unix_ms: None,
                t0_us: None,
            });
        }
    }

    fn attach_panel(_ctx: &BootCtx, display: Display) -> Panel {
        Panel::DrawsLast(display)
    }

    fn net_config(_ctx: &BootCtx) -> Option<crate::net::Config> {
        // `MFSK_JTTY_NO_WIFI=1`: a measurement knob — the same receiver with
        // the WiFi driver's internal DRAM left unclaimed (§3's question).
        if option_env!("MFSK_JTTY_NO_WIFI").is_some() {
            log::warn!("jtty_app: MFSK_JTTY_NO_WIFI — no network this boot");
            return None;
        }
        // As FT4 (§6): the console and the files over WiFi, NTP for the
        // `all.txt` anchor. Decoding needs no clock.
        Some(crate::net::Config {
            name: "jtty_app::net",
            power_save: true,
            ntp: true,
            without: "no NTP, no UDP log, no config page",
            bringup: crate::net::Bringup::Connect,
            http: true,
            on_ntp: |synced| log::info!("jtty_app: NTP synced = {synced}"),
        })
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
                s.t0_us = None;
                RESETS.fetch_add(1, Ordering::Relaxed);
                log::warn!(
                    "jtty_app: audio stopped for more than a second — stream reset (generation {})",
                    s.generation
                );
                let _ = s.clock.delivered(samples.len(), now);
            }
        }
        if s.t0_us.is_none() {
            let t0 = s.clock.t0_us().unwrap_or(now);
            s.t0_us = Some(t0);
            s.anchor_unix_ms = mfsk_app_shared::time_sync::utc_now_ms()
                .map(|ms| ms as i64 - (now - t0) / 1_000);
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
            let mut ready = Vec::new();
            let dropped_before = front.dropped();
            let mut room = || QUEUED.load(Ordering::Relaxed) < QUEUE_DEPTH;
            front.push_or_drop(&block, &mut room, &mut |p| ready.push(p));
            DROPPED.fetch_add((front.dropped() - dropped_before) as u32, Ordering::Relaxed);
            FRONT_US.fetch_add((now_us() - t) as u32, Ordering::Relaxed);
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

/// The back end: core 0 below the panel, its 32 KB stack in PSRAM.
fn spawn_back(rx: ChanRx<Msg>, params: Params) {
    let mut cfg = ThreadSpawnConfiguration {
        name: Some(c"jtty_back"),
        stack_size: BACK_STACK,
        priority: BACK_PRIORITY,
        pin_to_core: Some(Core::Core0),
        ..ThreadSpawnConfiguration::default()
    };
    // `Cap8bit` is not optional: `esp_pthread_set_cfg` refuses stack caps
    // without it (`pthread.c`, ESP_ERR_INVALID_ARG), and a refused config
    // leaves the *previous* one in force — which is how `jtty-bench`'s
    // stack-placement run (#516) measured an unpinned, default-priority
    // thread with an internal stack and reported it as "PSRAM".
    // `MFSK_JTTY_BACK_STACK_INTERNAL=1`: measurement knob, the stack in
    // internal DRAM instead.
    cfg.stack_alloc_caps = if option_env!("MFSK_JTTY_BACK_STACK_INTERNAL").is_some() {
        MallocCap::Internal | MallocCap::Cap8bit
    } else {
        MallocCap::Spiram | MallocCap::Cap8bit
    };
    if let Err(e) = cfg.set() {
        log::error!("jtty_app: back end thread config refused ({e}) — not spawning it");
        return;
    }
    let spawned = std::thread::Builder::new()
        .stack_size(BACK_STACK)
        .spawn(move || back_loop(rx, params));
    let _ = ThreadSpawnConfiguration::default().set();
    if let Err(e) = spawned {
        log::error!("jtty_app: back end spawn failed ({e})");
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
                back.process(prepared, &mut |u| on_update(u, &mut open, &mut rep));
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
    let front_ms = i64::from(FRONT_US.swap(0, Ordering::Relaxed)) / w / 1000;
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
         (largest {plarge})",
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
    // Stack high-water marks: the panel logs every task's (`board::log_task_stacks`).
    *rep = Report::default();
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
