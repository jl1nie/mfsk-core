//! CoreS3 FST4-60 wideband monitor receiver — the application on top of
//! the pipeline `fst4_ddc_bench` measured (issues #306/#307/#327).
//!
//! What it does, once per 60 s slot: down-convert and build the
//! spectrogram *as audio arrives*, then after the slot search
//! 100-3000 Hz for candidates and decode them in coarse-score order,
//! deepest first, until the band is covered. Decodes land on the LCD
//! and in the log. On the real hardware this pipeline reaches its first
//! decode 2.2 s after the slot ends and covers all 50 candidates in
//! 33 s of the next slot.
//!
//! **A new binary, not a mode of `wspr_app`.** The two share the
//! CoreS3's board/PMIC/UAC/WiFi plumbing and nothing else: different
//! decoder, different slot length, different memory profile, and — the
//! part that actually decides it — a different task/priority layout,
//! below.
//!
//! ## Task layout, and why it is not `wspr_app`'s
//!
//! | task | core | prio | duty |
//! |---|---|---|---|
//! | display | 1 | **7** | ~30 ms every 500 ms |
//! | capture (DDC + spectrogram) | 1 | **6** | 5.7 s per 60 s slot |
//! | scan (coarse search + candidate loop) | 0 | 5 | up to 34 s per slot |
//! | `fst4_dual_core` worker | 1 | 5 | with the scan task |
//! | network (WiFi/NTP/HTTP) | 1 | 2 | one-time, then idle |
//!
//! `wspr_app` keeps its compute on core 0 and everything else on core
//! 1, so a lower-priority display task on core 1 is never starved. FST4
//! cannot do that: its candidate loop is *dual-core* (#327, 1.67×), so
//! core 1 is occupied at priority 5 for most of the post-slot window.
//! A display or capture task below that priority would be starved
//! exactly the way `wspr_app`'s display task was before its own fix.
//!
//! So both are put *above* the decode instead. That inverts the usual
//! intuition and is deliberate: **audio arrival is real-time and the
//! decode is best-effort.** Capture must not miss samples — a dropped
//! block is a hole in the slot that no amount of later compute
//! recovers — while a decode that finishes 2 s later is still a decode.
//! Both preempting tasks are small (9.5% and ~6% duty), so the loop
//! stretches by roughly that much and still fits the slot.
//!
//! ## Memory, and why the spectrogram is dropped early
//!
//! A slot in flight is ~1.5 MB of baseband plus ~3.3 MB of
//! spectrogram, against 8 MB of PSRAM. A receiver always has two slots
//! alive — decoding slot N while capturing slot N+1 — and 2 × 4.8 MB
//! does not fit.
//!
//! It does not have to. Only the coarse search reads the spectrogram,
//! and that is the first second of the post-slot work; the candidate
//! loop needs the baseband alone. So the scan task drops the
//! spectrogram the moment the search returns, taking the finished slot
//! down to 1.5 MB for the 33 s the loop runs. Peak is then one full
//! slot plus one baseband, ~6.3 MB.
//!
//! ## Audio source
//!
//! Real USB audio arrives through `uac.rs`'s `AudioSink` — shared with
//! the FT8 controller and `wspr_app` — and is **not hardware-verified**
//! (issue #163, no UAC source has ever enumerated on this board).
//! Until one does, the capture task replays the baked golden FST4-60
//! slot instead, so the app is fully exercisable with no radio
//! attached and the screen shows real decodes of real (if replayed)
//! signals. `A` in the status bar is which source is live.
//!
//! **No wall-clock slot alignment yet** for the real-audio path — the
//! same open item `wspr_app` and `uac.rs` already carry. Slot
//! boundaries are counted in samples from whenever the stream started,
//! not UTC.

#![allow(dead_code)] // board.rs/pmic.rs/uac.rs carry items this bin doesn't use (no touch, no TX).

use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};

use esp_idf_hal::delay::{Ets, FreeRtos};
use esp_idf_hal::gpio::{AnyIOPin, PinDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use display_interface_spi::SPIInterface;
use mipidsi::options::{ColorInversion, Orientation, Rotation};
use mipidsi::{models::ILI9342CRgb565, Builder};

use embedded_shared::fst4_dual_core;
use embedded_shared::fst4_monitor::{
    self, CapturedSlot, MonitorConfig, MonitorHit, SlotCapture,
};
use mfsk_app_shared::log_sink::{FanoutLogger, LogFanout};
use mfsk_app_shared::settings;
use mfsk_app_shared::{http_config, ntp, udp_log};
use mfsk_app_shared::ui::fst4_list::{self, Fst4SpotRow, Fst4UiState};

#[path = "../board.rs"]
mod board;
#[path = "../pmic.rs"]
mod pmic;
#[path = "../uac.rs"]
mod uac;

/// The same baked golden slot `fst4-ddc-bench` runs — raw `i16` little
/// endian at 12 kHz, byte-wise rather than transmuted (1-byte
/// `include_bytes!` alignment faults an unaligned `i16` load on
/// Xtensa).
const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/fst4_60_golden_audio.bin");

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");
const UDP_LOG_TARGET: &str = env!("UDP_LOG_TARGET");
const UDP_LOG_PORT: &str = env!("UDP_LOG_PORT");

static FANOUT: LogFanout = LogFanout::new();
static LOGGER: FanoutLogger = FanoutLogger::new(&FANOUT, log::LevelFilter::Info);

/// FST4-60's slot. Capture paces itself to this; the decode has the
/// same period to finish in.
const SLOT_SAMPLES_12K: usize = 60 * 12_000;
const SLOT_US: i64 = 60_000_000;

/// Feed size for the replay source. One second of audio — large enough
/// that per-block overhead is irrelevant, small enough to interleave
/// with the decode. The spectrogram is bit-identical at any block size.
const REPLAY_BLOCK: usize = 12_000;

/// The bench reserves 96 KiB for the identical work and reports
/// **~91 KiB still untouched** — i.e. ~5 KiB actually used across
/// `coarse_search` and the candidate loop, which allocate their real
/// state on the heap. 48 KiB is a 10x margin on that, and the 48 KiB it
/// gives back is internal DRAM the WiFi PHY needs: the first boot of
/// this app aborted in `phy_track_pll_init` with `ESP_ERR_NO_MEM`
/// against 15 KB of internal DRAM left, with a 96 KiB reservation here
/// as the largest single cause.
const SCAN_STACK: u32 = 48 * 1024;
/// Small, and deliberately **not** in PSRAM: this is the DSP-heavy task
/// (7.9 s per slot of FIR and FFT), and a PSRAM stack would slow every
/// inner loop that touches a local. The network task is the one that
/// can afford external memory.
const CAPTURE_STACK: u32 = 16 * 1024;
const DISPLAY_STACK: u32 = 24 * 1024;
const NETWORK_STACK: u32 = 24 * 1024;

/// See this file's task-layout table: display and capture sit *above*
/// the decode, not below it.
const DISPLAY_PRIORITY: u32 = 7;
const CAPTURE_PRIORITY: u32 = 6;
const SCAN_PRIORITY: u32 = 5;
const NETWORK_PRIORITY: u32 = 2;

/// `MFSK_FST4_APP_CAPTURE_SLOTS=<n>` — stop capturing after `n` slots
/// (`0`, the default, never stops).
///
/// Diagnostic only, and the one knob that isolates the question the
/// first hardware run raised: the candidate loop takes 52 s in this app
/// against the bench's 33 s, and the front end 16 s against 5.9 s, but
/// "they contend" is a hypothesis, not a measurement. With `=1` the
/// capture task posts slot 0 and then idles, so slot 0's decode runs
/// with everything else the app does — WiFi, display, the same task
/// layout — and *only* the capture removed.
const CAPTURE_SLOTS: usize = {
    let v = option_env!("MFSK_FST4_APP_CAPTURE_SLOTS");
    match v {
        Some(s) => {
            let b = s.as_bytes();
            let mut i = 0;
            let mut acc = 0usize;
            while i < b.len() {
                acc = acc * 10 + (b[i] - b'0') as usize;
                i += 1;
            }
            acc
        }
        None => 0,
    }
};

/// `MFSK_FST4_APP_NO_WIFI=1` — skip WiFi bring-up entirely.
///
/// Diagnostic. The WiFi driver's own task runs at FreeRTOS priority 23,
/// far above anything this app creates, so an association that keeps
/// retrying preempts the decode at will — and this app's AP is exactly
/// the one `wifi::connect_with_retry` documents needing unbounded
/// retry. This switch is what separates "the decode is slow" from "the
/// radio is eating the decode".
const NO_WIFI: bool = match option_env!("MFSK_FST4_APP_NO_WIFI") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};

/// `MFSK_FST4_APP_USB_HOST=1` — install the USB host + UAC class
/// driver, and take real audio from a radio. See the call site for why
/// this is off by default while #163 is open.
const USB_HOST: bool = match option_env!("MFSK_FST4_APP_USB_HOST") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};

/// The shipped monitor, with one number changed.
///
/// **The candidate loop is slower in an app than in the bench**, and
/// measurably so: 46-48 s here against the bench's 33 s for the same
/// 50 candidates. The capture task is the reason — from the second
/// slot on it runs its FIR/FFT front end concurrently with the decode
/// on both cores, and the two contend for PSRAM bandwidth. The same
/// contention runs the other way too: the front end itself goes 5.9 s
/// on the first slot (nothing else running) to ~15 s once the decoder
/// is busy.
///
/// 45 s therefore truncated the band at ~30 of 50 candidates. 52 s
/// covers them while still landing before the next slot's handoff at
/// ~T+61 s. Nothing about time-to-first-decode changes — that is 2.4 s
/// post-slot either way, since the signal is at coarse rank 1.
const CFG: MonitorConfig = MonitorConfig {
    deadline_ms: 52_000,
    ..MonitorConfig::FST4_60_WIDEBAND
};

const MALLOC_CAP_SPIRAM: u32 = 1 << 10;

/// Same bound `wspr_app` uses — one attempt at boot, then the app runs
/// regardless.
const NTP_SYNC_TIMEOUT_MS: u32 = 20_000;

static SCAN_GO: AtomicBool = AtomicBool::new(false);
/// Flips true on the first real sample from a UAC device, after which
/// the capture task stops replaying the golden slot.
static UAC_AUDIO_ACTIVE: AtomicBool = AtomicBool::new(false);

/// The finished slot waiting to be decoded. One deep on purpose: if
/// the scan task has not taken the previous slot by the time the next
/// one ends, the new slot replaces it and that is the correct
/// behaviour for a monitor — freshest audio wins, and the log says so.
static SLOT_READY: Mutex<Option<CapturedSlot>> = Mutex::new(None);

/// Real audio waiting to be fed into the slot being captured.
///
/// The capture task owns its [`SlotCapture`] on its own stack and is
/// the only thing that ever touches it; the USB reader thread only
/// appends raw samples here. That split is not just tidiness — a
/// `SlotCapture` holds the spectrogram builder's `Box<dyn Fft>`, which
/// is not `Send`, and the reader thread has no business running 5.7 s
/// of DSP per slot in the first place. `wspr_app`'s sink does run its
/// DDC inline on that thread; this one deliberately does not.
///
/// Bounded: if the capture task ever stalls, old audio is dropped
/// rather than growing without limit, and the drop is logged.
static AUDIO_STAGING: Mutex<Vec<i16>> = Mutex::new(Vec::new());
/// ~2 s of 12 kHz audio. Enough to absorb scheduling jitter at
/// [`CAPTURE_PRIORITY`] *and* the inter-slot wait described at
/// [`SPECTRA_FREE`], far short of a slot.
const STAGING_CAP: usize = 24_000;

/// Whether the previous slot's spectrogram has been released.
///
/// Two spectrograms do not fit: one slot in flight is ~1.5 MB of
/// baseband plus ~3.3 MB of spectrogram, and 8 MB of PSRAM cannot hold
/// two. The first boot of this app proved it — the capture task
/// allocated slot N+1 the instant it posted slot N and aborted on
/// `memory allocation of 758772 bytes failed`, which is exactly one
/// slot's baseband reserve.
///
/// So the capture task waits for this before allocating the next
/// capture, and the scan task sets it the moment `coarse_search`
/// returns and the spectrogram can go. That wait is ~1.2 s of a 60 s
/// slot; real audio arriving during it is held in [`AUDIO_STAGING`].
static SPECTRA_FREE: AtomicBool = AtomicBool::new(true);

static FST4_UI: Mutex<Fst4UiState> = Mutex::new(Fst4UiState::new());

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

fn log_heap(tag: &str) {
    const MALLOC_CAP_8BIT: u32 = 1 << 2;
    const MALLOC_CAP_INTERNAL: u32 = 1 << 11;
    unsafe {
        log::info!(
            "[mem] {tag}: internal {} KB, PSRAM {} KB (largest contig {} KB)",
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024,
        );
    }
}

fn main() -> ! {
    esp_idf_svc::sys::link_patches();
    LOGGER.install();

    log::info!("=== mfsk-core-m5stack-cores3-app fst4-app boot ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);

    // The candidate loop is compute-bound for tens of seconds with no
    // yield point on either core; IDLE0/IDLE1 starve and the task
    // watchdog fires. Same call, same reason, as every bench here.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    // **Before WiFi**, per `fst4_dual_core::init`'s ordering
    // constraint: its stack is a `.bss` reservation the linker makes,
    // but the TCB and queues are still allocations, and WSPR measured a
    // worker spawn silently lost to heap fragmentation with the radio
    // up.
    fst4_dual_core::init();

    let peripherals = Peripherals::take().expect("peripherals taken twice");
    let nvs_part = EspDefaultNvsPartition::take().expect("NVS partition take");
    let nvs = settings::open_nvs(nvs_part.clone()).expect("settings NVS open");
    let nvs = Arc::new(Mutex::new(nvs));

    // Scan task's 96 KiB stack first, before anything else can
    // fragment internal DRAM — `wspr_app`'s own comment explains what
    // this ordering is worth on real hardware (a display task's
    // concurrent SPI-driver allocations beat it to the block once).
    log_heap("pre-scan-spawn");
    spawn_scan_task();
    spawn_capture_task();
    log_heap("post-scan-spawn");

    // Register the sink before the display task, whose body installs
    // the USB host driver — same "wire the consumer before installing
    // the driver" ordering `wspr_app` relies on.
    uac::set_audio_sink(Fst4Sink);

    spawn_display_task(DisplayCtx {
        i2c0: peripherals.i2c0,
        spi2: peripherals.spi2,
        pins: peripherals.pins,
    });
    log_heap("post-display-spawn");

    // WiFi *driver construction* is synchronous and stays here, before
    // `SCAN_GO`: `peripherals.modem` is consumed by value and is not
    // returned on `Err`, so a failure is permanent for this boot, and
    // the internal-DRAM it needs has to be claimed in the same quiet
    // window the task stacks were. The slow, unbounded half
    // (associate/DHCP retry) is what the network task backgrounds.
    let sysloop = EspSystemEventLoop::take().expect("sysloop");
    let wifi_driver = if NO_WIFI {
        log::warn!("fst4_app: MFSK_FST4_APP_NO_WIFI=1 — no radio this boot (diagnostic build)");
        None
    } else if WIFI_SSID.is_empty() {
        log::warn!("fst4_app: WIFI_SSID empty (no cfg.toml) — NTP and HTTP config unavailable");
        None
    } else {
        match mfsk_app_shared::wifi::wifi_driver_init(peripherals.modem, sysloop, Some(nvs_part)) {
            Ok(w) => Some(w),
            Err(e) => {
                log::error!("fst4_app: WiFi driver init failed (unrecoverable this boot): {e:#}");
                None
            }
        }
    };
    log_heap("post-wifi-driver-init");

    if let Some(wifi_driver) = wifi_driver {
        spawn_network_task(NetworkCtx { wifi_driver, nvs });
    }
    log_heap("post-network-spawn");

    // Decode start does not wait on WiFi — same conclusion `wspr_app`
    // reached the hard way: this app's test AP needs unbounded retry,
    // so gating boot on it means an unbounded hang, not a delay.
    SCAN_GO.store(true, Ordering::Release);

    loop {
        FreeRtos::delay_ms(1000);
    }
}

// ── Capture task ─────────────────────────────────────────────────────

extern "C" fn capture_task_entry(_arg: *mut core::ffi::c_void) {
    capture_loop();
}

fn spawn_capture_task() {
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(capture_task_entry),
            c"fst4_capture".as_ptr(),
            CAPTURE_STACK,
            core::ptr::null_mut(),
            CAPTURE_PRIORITY,
            core::ptr::null_mut(),
            1,
        )
    };
    if created != 1 {
        log::error!("fst4_app: failed to create fst4_capture task");
    }
}

/// Finish a slot and post it for the scan task.
fn post_slot(cap: SlotCapture) {
    let t0 = now_us();
    let slot = cap.finish();
    let mut ready = SLOT_READY.lock().expect("SLOT_READY poisoned");
    if ready.is_some() {
        log::warn!("fst4_app::capture: previous slot not consumed — dropping it for the new one");
    }
    log::info!(
        "fst4_app::capture: slot ready ({} baseband samples, flush {} ms)",
        slot.coarse_i.len(),
        (now_us() - t0) / 1000,
    );
    *ready = Some(slot);
}

/// Owns the slot boundary and the [`SlotCapture`] behind it, fed
/// either from real UAC audio (via [`AUDIO_STAGING`]) or, until a
/// device enumerates, by replaying the golden slot at real-time
/// cadence.
fn capture_loop() -> ! {
    while !SCAN_GO.load(Ordering::Acquire) {
        FreeRtos::delay_ms(200);
    }
    log::info!("fst4_app: capture loop starting");
    log_heap("capture-start");

    let mut block: Vec<i16> = Vec::with_capacity(REPLAY_BLOCK);
    let mut slots_done = 0usize;
    loop {
        // Wait for the previous slot's spectrogram to be released
        // before claiming memory for this one — see [`SPECTRA_FREE`].
        let t_wait0 = now_us();
        while !SPECTRA_FREE.swap(false, Ordering::AcqRel) {
            FreeRtos::delay_ms(20);
        }
        let waited_ms = (now_us() - t_wait0) / 1000;
        if waited_ms > 0 {
            log::info!("fst4_app::capture: waited {waited_ms} ms for the previous slot's memory");
        }

        let slot_start = now_us();
        let mut cap = SlotCapture::with_input_hint(CFG, true, SLOT_SAMPLES_12K);
        let mut fed = 0usize;
        let mut t_compute = 0i64;

        while fed < SLOT_SAMPLES_12K {
            let live = UAC_AUDIO_ACTIVE.load(Ordering::Acquire);
            if live {
                // Real audio: take whatever has arrived and feed it.
                // The slot still ends on sample count, so a stream
                // that stops mid-slot ends the slot short rather than
                // hanging — see this file's alignment caveat.
                let staged = core::mem::take(&mut *AUDIO_STAGING.lock().expect("staging poisoned"));
                if staged.is_empty() {
                    FreeRtos::delay_ms(20);
                    continue;
                }
                let t = now_us();
                cap.push_i16(&staged);
                t_compute += now_us() - t;
                fed += staged.len();
            } else {
                let take = REPLAY_BLOCK.min(SLOT_SAMPLES_12K - fed);
                let samples = GOLDEN_AUDIO.len() / 2;
                block.clear();
                block.extend(
                    GOLDEN_AUDIO
                        .chunks_exact(2)
                        .cycle()
                        .skip(fed % samples)
                        .take(take)
                        .map(|b| i16::from_le_bytes([b[0], b[1]])),
                );
                let t = now_us();
                cap.push_i16(&block);
                t_compute += now_us() - t;
                fed += take;

                // Pace to real time, so the front end's cost is a duty
                // cycle rather than a flat-out run.
                let due_us = slot_start + (fed as i64) * 1_000_000 / 12_000;
                let sleep_ms = ((due_us - now_us()).max(0) / 1000) as u32;
                if sleep_ms > 0 {
                    FreeRtos::delay_ms(sleep_ms);
                }
            }
        }

        log::info!(
            "fst4_app::capture: front end {} ms of a {} ms slot ({:.1}% duty), source {}",
            t_compute / 1000,
            SLOT_US / 1000,
            t_compute as f64 / SLOT_US as f64 * 100.0,
            if UAC_AUDIO_ACTIVE.load(Ordering::Acquire) { "UAC" } else { "golden replay" },
        );
        post_slot(cap);
        slots_done += 1;
        if CAPTURE_SLOTS > 0 && slots_done >= CAPTURE_SLOTS {
            log::info!(
                "fst4_app::capture: MFSK_FST4_APP_CAPTURE_SLOTS={CAPTURE_SLOTS} reached — idling; \
                 the decode below runs with no capture beside it"
            );
            loop {
                FreeRtos::delay_ms(10_000);
            }
        }
    }
}

/// Real-audio sink. All it does is stage samples for the capture task
/// — see [`AUDIO_STAGING`] for why the DSP is not done here.
struct Fst4Sink;

impl uac::AudioSink for Fst4Sink {
    fn push_samples(&mut self, samples_12k_mono: &[i16]) {
        if !UAC_AUDIO_ACTIVE.swap(true, Ordering::AcqRel) {
            log::info!("fst4_app: real UAC audio active — golden replay stops at the next slot");
        }
        let mut staging = AUDIO_STAGING.lock().expect("staging poisoned");
        if staging.len() + samples_12k_mono.len() > STAGING_CAP {
            log::warn!(
                "fst4_app: audio staging full ({} samples) — dropping {}; capture task is behind",
                staging.len(),
                samples_12k_mono.len(),
            );
            return;
        }
        staging.extend_from_slice(samples_12k_mono);
    }
}

// ── Scan task ────────────────────────────────────────────────────────

extern "C" fn scan_task_entry(_arg: *mut core::ffi::c_void) {
    scan_loop();
}

fn spawn_scan_task() {
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(scan_task_entry),
            c"fst4_scan".as_ptr(),
            SCAN_STACK,
            core::ptr::null_mut(),
            SCAN_PRIORITY,
            core::ptr::null_mut(),
            0, // core 0; `fst4_dual_core`'s worker takes core 1.
        )
    };
    if created != 1 {
        log::error!("fst4_app: failed to create fst4_scan task");
    }
}

fn scan_loop() -> ! {
    while !SCAN_GO.load(Ordering::Acquire) {
        FreeRtos::delay_ms(200);
    }
    log::info!("fst4_app: scan loop starting");

    let mut slot_num = 0u32;
    loop {
        let mut slot = loop {
            if let Some(s) = SLOT_READY.lock().expect("SLOT_READY poisoned").take() {
                break s;
            }
            FreeRtos::delay_ms(200);
        };

        let t_post0 = now_us();
        let (candidates, fill_us, rank_us) = fst4_monitor::coarse_search(&slot);
        // Drop the spectrogram now — only the search reads it, and the
        // 33 s candidate loop below runs while the *next* slot is being
        // captured. See this file's memory section, and [`SPECTRA_FREE`]
        // for what the capture task is waiting on.
        slot.spectra = None;
        SPECTRA_FREE.store(true, Ordering::Release);
        let t_search_ms = (now_us() - t_post0) / 1000;
        log::info!(
            "fst4_app::scan: slot {slot_num} — {} candidates in {} ms (fill {} ms, rank {} ms)",
            candidates.len(),
            t_search_ms,
            fill_us / 1000,
            rank_us / 1000,
        );

        let hits = fst4_monitor::run_candidate_loop(&slot, &candidates);
        let decoded = fst4_monitor::distinct_decodes(&hits);
        let loop_ms = (now_us() - t_post0) / 1000 - t_search_ms;
        let first_ms = decoded.first().map_or(0, |h| h.t_ms + t_search_ms);

        for h in &decoded {
            log::info!(
                "fst4_app::scan: {:?} @ {:.1} Hz snr {:+.0} dB dt {:+.2} s — post-slot {} ms",
                h.msg.as_deref().unwrap_or(""),
                h.refined_hz,
                h.snr_db,
                h.dt_sec,
                h.t_ms + t_search_ms,
            );
        }
        // Per-candidate cost, split the way the bench splits it, so the
        // two are directly comparable: the bench logs `refine N ms
        // decode M ms` per candidate and this is the same pair summed.
        let recenter_ms: i64 = hits.iter().map(|h| h.recenter_us).sum::<i64>() / 1000;
        let refine_ms: i64 = hits.iter().map(|h| h.refine_us - h.recenter_us).sum::<i64>() / 1000;
        let decode_ms: i64 = hits.iter().map(|h| h.decode_us).sum::<i64>() / 1000;
        let n = hits.len().max(1) as i64;
        log::info!(
            "fst4_app::scan: slot {slot_num} done — {} tried, {} distinct decodes, \
             search {} ms + loop {} ms, first decode {} ms post-slot \
             [recentre {} ms/cand, sync-search {} ms/cand, decode {} ms/cand]",
            hits.len(),
            decoded.len(),
            t_search_ms,
            loop_ms,
            first_ms,
            recenter_ms / n,
            refine_ms / n,
            decode_ms / n,
        );

        let rows: Vec<Fst4SpotRow> = decoded.iter().map(|h| to_row(h, t_search_ms)).collect();
        {
            let mut ui = FST4_UI.lock().expect("FST4_UI poisoned");
            ui.last_slot_cands = candidates.len();
            ui.last_slot_tried = hits.len();
            ui.last_first_decode_ms = first_ms;
            ui.audio_live = UAC_AUDIO_ACTIVE.load(Ordering::Acquire);
            ui.set_slot(&current_hhmm(), &rows);
        }
        log_heap("post-slot");
        slot_num = slot_num.wrapping_add(1);
    }
}

fn to_row(h: &MonitorHit, search_ms: i64) -> Fst4SpotRow {
    let mut msg: heapless::String<22> = heapless::String::new();
    let text = h.msg.as_deref().unwrap_or("");
    let _ = msg.push_str(&text[..text.len().min(22)]);
    Fst4SpotRow {
        utc_hhmm: current_hhmm(),
        freq_hz: h.refined_hz,
        snr_db: h.snr_db.is_finite().then(|| h.snr_db.clamp(-99.0, 99.0) as i8),
        dt_sec: h.dt_sec,
        msg,
        t_s: (h.t_ms + search_ms) as f32 / 1000.0,
    }
}

fn current_hhmm() -> heapless::String<4> {
    use core::fmt::Write as _;
    let mut s: heapless::String<4> = heapless::String::new();
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let _ = write!(&mut s, "{:02}{:02}", (now / 3600) % 24, (now / 60) % 60);
    s
}

fn current_hhmmss() -> heapless::String<8> {
    use core::fmt::Write as _;
    let mut s: heapless::String<8> = heapless::String::new();
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let _ = write!(
        &mut s,
        "{:02}:{:02}:{:02}",
        (now / 3600) % 24,
        (now / 60) % 60,
        now % 60
    );
    s
}

// ── Display task ─────────────────────────────────────────────────────

struct DisplayCtx {
    i2c0: esp_idf_hal::i2c::I2C0<'static>,
    spi2: esp_idf_hal::spi::SPI2<'static>,
    pins: esp_idf_hal::gpio::Pins,
}

extern "C" fn display_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_display_task` leaked exactly this pointer.
    let ctx = unsafe { Box::from_raw(arg as *mut DisplayCtx) };
    display_loop(*ctx);
}

fn spawn_display_task(ctx: DisplayCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(display_task_entry),
            c"fst4_display".as_ptr(),
            DISPLAY_STACK,
            ptr,
            DISPLAY_PRIORITY,
            core::ptr::null_mut(),
            1,
        )
    };
    if created != 1 {
        log::error!("fst4_app: failed to create fst4_display task");
    }
}

fn display_loop(ctx: DisplayCtx) -> ! {
    let mut display = match pmic::init(ctx.i2c0, ctx.pins.gpio12, ctx.pins.gpio11) {
        Ok(mut i2c) => {
            // VBUS boost before the USB host driver, or the host stack
            // sees no device — same sequence `wspr_app` documents.
            if let Err(e) = pmic::enable_usb_host_vbus(&mut i2c) {
                log::error!("BUS_OUT_EN failed: {e:#}");
            }
            drop(i2c);

            // Installing the USB host driver **detaches
            // USB-Serial-JTAG**, i.e. the serial console, the moment it
            // returns — which is why `wspr_app` wires UDP log fanout
            // first and why this is opt-in here rather than
            // unconditional.
            //
            // Default off, deliberately, for as long as issue #163 is
            // open: no UAC source has ever enumerated on this board, so
            // turning it on today costs the only telemetry channel that
            // works without a WiFi association and buys nothing. Build
            // with `MFSK_FST4_APP_USB_HOST=1` to attach a radio; the
            // app then depends on the UDP log sink for its logs, same
            // as `wspr_app` does.
            if USB_HOST {
                log::info!("fst4_app: installing USB host + UAC class driver");
                if let Err(e) = uac::start_host() {
                    log::error!("fst4_app: UAC host start failed: {e:#}");
                }
            } else {
                log::info!(
                    "fst4_app: USB host disabled (build with MFSK_FST4_APP_USB_HOST=1) \
                     — serial console kept, golden slot replayed"
                );
            }

            let driver = SpiDriver::new(
                ctx.spi2,
                ctx.pins.gpio36,
                ctx.pins.gpio37,
                Option::<AnyIOPin>::None,
                &SpiDriverConfig::new(),
            )
            .expect("SPI2 driver");
            let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
            let spi_dev =
                SpiDeviceDriver::new(driver, Some(ctx.pins.gpio3), &spi_cfg).expect("SPI device");
            let dc = PinDriver::output(ctx.pins.gpio35).expect("DC gpio35");
            let di = SPIInterface::new(spi_dev, dc);

            let mut delay = Ets;
            match Builder::new(ILI9342CRgb565, di)
                .display_size(320, 240)
                .orientation(Orientation::new().rotate(Rotation::Deg90))
                .invert_colors(ColorInversion::Inverted)
                .init(&mut delay)
            {
                Ok(d) => d,
                Err(e) => {
                    log::error!("display init failed: {e:?}");
                    loop {
                        log::info!("alive (no LCD)");
                        FreeRtos::delay_ms(2000);
                    }
                }
            }
        }
        Err(e) => {
            log::error!("PMIC init failed: {e:#}");
            loop {
                log::info!("alive (no PMIC/LCD)");
                FreeRtos::delay_ms(2000);
            }
        }
    };
    log::info!("LCD init OK ({}x{})", board::LCD_WIDTH, board::LCD_HEIGHT);

    {
        let ui = FST4_UI.lock().expect("FST4_UI poisoned");
        if let Err(e) = fst4_list::render_all(&mut display, &ui) {
            log::error!("fst4_app::display: render_all FAILED: {e:?}");
        }
    }

    let mut last_dirty = u32::MAX;
    let mut tick: u32 = 0;
    loop {
        let heap_kb = (unsafe { esp_idf_svc::sys::esp_get_free_heap_size() } / 1024) as u32;
        let hhmmss = current_hhmmss();
        let dirty = {
            let mut ui = FST4_UI.lock().expect("FST4_UI poisoned");
            ui.free_heap_kb = heap_kb;
            ui.utc_hhmmss = hhmmss.clone();
            ui.dirty_seq()
        };
        {
            let ui = FST4_UI.lock().expect("FST4_UI poisoned");
            if let Err(e) = fst4_list::render_status(&mut display, &ui) {
                log::warn!("fst4_app::display: render_status failed: {e:?}");
            }
            if dirty != last_dirty {
                if let Err(e) = fst4_list::render_slot(&mut display, &ui) {
                    log::warn!("fst4_app::display: render_slot failed: {e:?}");
                }
                if let Err(e) = fst4_list::render_history(&mut display, &ui) {
                    log::warn!("fst4_app::display: render_history failed: {e:?}");
                }
                last_dirty = dirty;
            }
        }
        // Direct evidence that this task keeps running through the
        // scan task's compute-bound stretch — the same reason
        // `wspr_app`'s display loop logs one.
        if tick % 20 == 0 {
            log::info!(
                "fst4_app::display: alive tick={tick} dirty={dirty} utc={hhmmss} heap={heap_kb}k"
            );
        }
        tick = tick.wrapping_add(1);
        FreeRtos::delay_ms(500);
    }
}

// ── Network task ─────────────────────────────────────────────────────

struct NetworkCtx {
    wifi_driver: esp_idf_svc::wifi::BlockingWifi<esp_idf_svc::wifi::EspWifi<'static>>,
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
}

extern "C" fn network_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_network_task` leaked exactly this pointer.
    let ctx = unsafe { Box::from_raw(arg as *mut NetworkCtx) };
    network_loop(*ctx);
}

/// Stack in PSRAM, not internal DRAM: `wspr_app` measured WiFi driver
/// bring-up taking internal DRAM to 10 KB free / 7 KB largest block,
/// nowhere near a 24 KiB stack, regardless of spawn ordering.
fn spawn_network_task(ctx: NetworkCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(network_task_entry),
            c"fst4_net".as_ptr(),
            NETWORK_STACK,
            ptr,
            NETWORK_PRIORITY,
            core::ptr::null_mut(),
            1,
            MALLOC_CAP_SPIRAM,
        )
    };
    if created != 1 {
        log::error!("fst4_app: failed to create fst4_net task");
    }
}

/// How long the radio is left alone between association campaigns.
///
/// **Bounded attempts, then quiet — not `wspr_app`'s unbounded retry.**
/// Measured on this hardware (2026-08-22): while the driver is trying
/// to associate with an AP it cannot reach, the decoder loses ~40% of
/// its throughput — `fst4_sync_search` goes 711 -> 1395 ms per
/// candidate and the candidate loop 33 -> 53 s, covering 48 of 50
/// candidates instead of all 50. The WiFi task runs at FreeRTOS
/// priority 23, above anything an application creates, so it preempts
/// at will, and this is *not* something the caller's own retry cadence
/// controls: a single `connect()` keeps the radio busy for the whole
/// `ESP_ERR_TIMEOUT` window, which is minutes.
///
/// A monitor receiver's job is to decode. WiFi buys it NTP, a config
/// page and remote logging — all of which can wait three minutes.
const RECONNECT_IDLE_MS: u32 = 180_000;
/// Attempts per campaign. Four is what `wifi.rs`'s own bisection
/// established as the number that beats the AP comeback-time
/// coin-flip; more than that is what costs decode throughput.
const CONNECT_ATTEMPTS_PER_CAMPAIGN: u32 = 4;

fn network_loop(mut ctx: NetworkCtx) -> ! {
    let info = loop {
        match mfsk_app_shared::wifi::connect_with_retry(
            &mut ctx.wifi_driver,
            WIFI_SSID,
            WIFI_PSK,
            Some(CONNECT_ATTEMPTS_PER_CAMPAIGN),
        ) {
            Ok(i) => break i,
            Err(e) => {
                log::warn!(
                    "fst4_app::net: no association after {CONNECT_ATTEMPTS_PER_CAMPAIGN} \
                     attempts ({e:#}) — leaving the radio alone for {} s so it stops \
                     preempting the decode",
                    RECONNECT_IDLE_MS / 1000,
                );
                FreeRtos::delay_ms(RECONNECT_IDLE_MS);
            }
        }
    };
    log::info!("fst4_app::net: WiFi up, ip {}", info.ip);

    // UDP log sink — the serial console goes away the moment the USB
    // host driver installs, so this is the only log this app has once
    // it is running for real.
    let target_ip: std::net::IpAddr = if UDP_LOG_TARGET.is_empty() || UDP_LOG_TARGET == "auto" {
        std::net::IpAddr::V4(info.subnet_broadcast)
    } else {
        match UDP_LOG_TARGET.parse() {
            Ok(ip) => ip,
            Err(e) => {
                log::warn!(
                    "fst4_app::net: UDP_LOG_TARGET '{UDP_LOG_TARGET}' parse failed ({e}); \
                     using subnet broadcast"
                );
                std::net::IpAddr::V4(info.subnet_broadcast)
            }
        }
    };
    let addr = std::net::SocketAddr::new(target_ip, UDP_LOG_PORT.parse().unwrap_or(9999));
    match udp_log::UdpLogSink::new(addr) {
        Ok(sink) => {
            if let Ok(mut slot) = FANOUT.udp.try_lock() {
                *slot = Some(sink);
            }
            FANOUT.drain_staging_to_udp();
            log::info!("fst4_app::net: UDP log sink up -> {addr}");
        }
        Err(e) => log::warn!("fst4_app::net: UDP socket bind failed: {e}"),
    }

    let initial_settings = {
        let g = ctx.nvs.lock().expect("settings NVS mutex poisoned");
        settings::load(&g)
    };
    let (ntp_synced, _sntp_keepalive) = if initial_settings.ntp_enabled {
        match ntp::start(&initial_settings.ntp_server) {
            Ok(sntp) => {
                let synced = ntp::wait_synced(&sntp, NTP_SYNC_TIMEOUT_MS);
                (synced, Some(sntp))
            }
            Err(e) => {
                log::warn!("fst4_app::net: NTP start failed: {e:#}");
                (false, None)
            }
        }
    } else {
        log::info!("fst4_app::net: NTP sync disabled in settings");
        (false, None)
    };
    if !ntp_synced {
        // Slot timestamps on screen stay relative to boot until this
        // succeeds. The decode itself does not depend on it — slot
        // boundaries are counted in samples, not wall clock (see this
        // file's alignment caveat).
        log::warn!("fst4_app::net: NTP never synced — UTC column is not absolute");
    }
    FST4_UI.lock().expect("FST4_UI poisoned").ntp_synced = ntp_synced;

    let _http_server = match http_config::start(ctx.nvs.clone()) {
        Ok(s) => {
            log::info!("fst4_app::net: HTTP config server up");
            Some(s)
        }
        Err(e) => {
            log::warn!("fst4_app::net: HTTP config server failed: {e:#}");
            None
        }
    };

    // `ctx.wifi_driver`/`_sntp_keepalive`/`_http_server` are never read
    // again but their `Drop`s tear down the association / stop syncing
    // / stop listening, so this task keeps them alive by never
    // returning.
    loop {
        FreeRtos::delay_ms(60_000);
    }
}
