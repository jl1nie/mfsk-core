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
use mipidsi::options::{ColorInversion, Orientation};
use mipidsi::{models::ILI9342CRgb565, Builder};

use embedded_shared::fst4_dual_core;
use embedded_shared::fst4_monitor::{
    self, CapturedSlot, MonitorConfig, MonitorHit, SlotCapture,
};
use mfsk_app_shared::settings;
use mfsk_app_shared::{http_config, ntp, udp_log};
use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::ui::fst4_list::{self, Fst4SpotRow, Fst4UiState};
use mfsk_app_shared::ui::{link_bar, mode_picker};


/// The same baked golden slot `fst4-ddc-bench` runs — raw `i16` little
/// endian at 12 kHz, byte-wise rather than transmuted (1-byte
/// `include_bytes!` alignment faults an unaligned `i16` load on
/// Xtensa).
const GOLDEN_AUDIO: &[u8] = include_bytes!("../../../assets/fst4_60_golden_audio.bin");



/// FST4-60's slot. Capture paces itself to this; the decode has the
/// same period to finish in.
const SLOT_SAMPLES_12K: usize = 60 * 12_000;
const SLOT_US: i64 = 60_000_000;
/// The same slot in seconds — what the UTC grid is computed from.
const SLOT_SECS: u64 = 60;

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
const SCAN_STACK: u32 = 24 * 1024;
/// In PSRAM, like the display and network tasks.
///
/// The first version of this file argued the opposite — DSP task,
/// keep its stack fast — and that reasoning was measured and found
/// backwards. This task's actual working set is the `SlotCapture`'s
/// heap buffers; its *stack* holds only small locals. Meanwhile every
/// KiB of internal DRAM it reserves is a KiB the decoder's own small
/// allocations fall back to PSRAM without, which costs far more (see
/// `fst4_dual_core::WORKER_STACK_BYTES` for the measurement).
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

/// `MFSK_FST4_APP_NO_CONNECT=1` — bring the WiFi *driver* up but never
/// associate. Diagnostic: separates the cost of the radio existing
/// from the cost of it carrying a network's traffic.
const NO_CONNECT: bool = match option_env!("MFSK_FST4_APP_NO_CONNECT") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};

/// `MFSK_FST4_APP_HOG_KB=<n>` — reserve `n` KB of **internal** DRAM at
/// boot and never release it. Diagnostic: reproduces the WiFi driver's
/// memory footprint without the radio, which is what separates
/// "internal DRAM starvation" from everything else the driver leaves
/// behind.
const HOG_KB: usize = {
    let v = option_env!("MFSK_FST4_APP_HOG_KB");
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

/// `MFSK_FST4_APP_WIFI_STOP=1` — with `NO_CONNECT`, also stop the
/// radio after the driver is up. Diagnostic, see the call site.
const WIFI_STOP: bool = match option_env!("MFSK_FST4_APP_WIFI_STOP") {
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
/// How long the real-audio path may go silent before the capture task
/// gives up on the slot. Generous — a USB hiccup should not truncate a
/// slot — but bounded, because the alternative is a receiver that
/// stops decoding and gives no reason.
const AUDIO_STALL_US: i64 = 5_000_000;

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

/// The whole FST4 receiver, given the resources rather than taking them.
///
/// Split out of `main` so one binary can carry every mode and pick at
/// boot from the NVS `boot_mode`, instead of a mode change meaning a
/// re-flash — which on this board means unplugging the radio, because
/// the USB host driver owns the port the flasher would use. The
/// singletons are taken once by whoever calls this.
///
/// Everything below is unchanged from when it was `main`, including
/// the ordering constraints: the worker-stack reservation and the scan
/// task's stack both have to land before WiFi starts.
pub fn run(peripherals: Peripherals, nvs_part: EspDefaultNvsPartition) -> ! {
    log::info!("=== mfsk-core-m5stack-cores3-app fst4-app boot ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);

    // The candidate loop is compute-bound for tens of seconds with no
    // yield point on either core; IDLE0/IDLE1 starve and the task
    // watchdog fires. Same call, same reason, as every bench here.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    // Take FST4's worker stack before WiFi, while the heap is still
    // whole — after WiFi and the USB host the largest free internal
    // block is 31,744 B, and this needs far more. See
    // `embedded_shared::worker_arena` for the measurements.
    if !fst4_dual_core::reserve_arena() {
        log::error!("worker stack reservation failed — decoding will run single-core");
    }
    fst4_dual_core::init();

    if HOG_KB > 0 {
        const MALLOC_CAP_INTERNAL_8BIT: u32 = (1 << 11) | (1 << 2);
        // 4 KiB blocks, matching what `SPIRAM_MALLOC_ALWAYSINTERNAL`
        // treats as "small enough to want internal DRAM" — the same
        // shape the allocations this is meant to crowd out have.
        let mut got = 0usize;
        for _ in 0..HOG_KB / 4 {
            let p = unsafe { esp_idf_svc::sys::heap_caps_malloc(4096, MALLOC_CAP_INTERNAL_8BIT) };
            if p.is_null() {
                break;
            }
            got += 4;
        }
        log::warn!("fst4_app: MFSK_FST4_APP_HOG_KB={HOG_KB} — reserved {got} KB of internal DRAM");
    }

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
    crate::uac::set_audio_sink(Fst4Sink);

    spawn_display_task(DisplayCtx {
        i2c0: peripherals.i2c0,
        spi2: peripherals.spi2,
        pins: peripherals.pins,
        nvs: nvs.clone(),
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
    } else if crate::WIFI_SSID.is_empty() {
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
        if NO_CONNECT {
            log::warn!("fst4_app: MFSK_FST4_APP_NO_CONNECT=1 — driver up, no association");
            if WIFI_STOP {
                // Separates "the radio is on" from "the radio's memory
                // is spoken for": `esp_wifi_stop` silences the receiver
                // while every buffer the driver allocated stays
                // allocated.
                let r = unsafe { esp_idf_svc::sys::esp_wifi_stop() };
                log::warn!("fst4_app: esp_wifi_stop() -> {r} (radio silenced, memory retained)");
            }
            core::mem::forget(wifi_driver);
        } else {
            spawn_network_task(NetworkCtx { wifi_driver, nvs });
        }
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
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(capture_task_entry),
            c"fst4_capture".as_ptr(),
            CAPTURE_STACK,
            core::ptr::null_mut(),
            CAPTURE_PRIORITY,
            core::ptr::null_mut(),
            1,
            MALLOC_CAP_SPIRAM,
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
        // Anchor the slot grid to UTC when the clock is real. For the
        // replay source this is cosmetic — a recording is not
        // real-time — but it is the same call the live path makes, so
        // the phase source is exercised on every boot rather than only
        // when a radio is attached.
        let mut fed = match mfsk_app_shared::time_sync::samples_to_next_slot_12k(SLOT_SECS) {
            Some(remain) if UAC_AUDIO_ACTIVE.load(Ordering::Acquire) => {
                log::info!(
                    "fst4_app::capture: slot anchored to UTC — {} ms to the next boundary",
                    remain / 12,
                );
                SLOT_SAMPLES_12K.saturating_sub(remain)
            }
            Some(remain) => {
                log::info!(
                    "fst4_app::capture: UTC phase available ({} ms to boundary); replay source \
                     is not real-time so the grid stays stream-relative",
                    remain / 12,
                );
                0
            }
            None => {
                log::warn!(
                    "fst4_app::capture: no UTC yet (NTP unsynced) — slot grid is stream-relative"
                );
                0
            }
        };
        let mut t_compute = 0i64;
        let mut last_audio_us = now_us();

        while fed < SLOT_SAMPLES_12K {
            let live = UAC_AUDIO_ACTIVE.load(Ordering::Acquire);
            if live {
                // Real audio: take whatever has arrived and feed it.
                let staged = core::mem::take(&mut *AUDIO_STAGING.lock().expect("staging poisoned"));
                if staged.is_empty() {
                    // A stream that stops mid-slot must not hang the
                    // receiver. The comment this replaces claimed the
                    // sample count already handled that; it did not —
                    // this loop would have spun here forever, and the
                    // first time anyone found out would have been a
                    // live bring-up session with a radio attached
                    // (issue #163).
                    if now_us() - last_audio_us > AUDIO_STALL_US {
                        log::warn!(
                            "fst4_app::capture: no UAC audio for {} s — ending slot short at \
                             {fed}/{SLOT_SAMPLES_12K} samples",
                            AUDIO_STALL_US / 1_000_000,
                        );
                        break;
                    }
                    FreeRtos::delay_ms(20);
                    continue;
                }
                last_audio_us = now_us();
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

impl crate::uac::AudioSink for Fst4Sink {
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
        // Two lines, not one: the UDP log — the only console once the
        // USB host driver detaches serial — caps a datagram at
        // `log_sink::LINE_MAX`, and the combined form ran 195 bytes
        // against a 160-byte cap, losing the per-candidate breakdown
        // exactly when it was being read remotely (issue #163).
        log::info!(
            "fst4_app::scan: slot {slot_num} done — {} tried, {} distinct decodes, \
             search {} ms + loop {} ms, first decode {} ms post-slot",
            hits.len(),
            decoded.len(),
            t_search_ms,
            loop_ms,
            first_ms,
        );
        // The budget, stated rather than left to be derived from the
        // numbers above.
        //
        // **This receiver is designed to sit far inside it.** A monitor
        // running FST4-60 has a 60 s slot against a candidate loop
        // measured in seconds, and that slack is deliberate — it is
        // what absorbs a crowded slot, a WiFi burst and the display
        // task without dropping candidates. A low occupancy figure here
        // is the design working, not headroom to reclaim.
        //
        // Which is why the over-budget branch is a `warn`: unlike FT8,
        // where a 15 s slot on a busy band genuinely runs out of time
        // and losing candidates is the operating limit, this one
        // exceeding its slot means something is wrong rather than
        // something is busy.
        let used_ms = t_search_ms + loop_ms;
        let budget_ms = SLOT_US / 1000;
        if used_ms > budget_ms {
            log::warn!(
                "fst4_app::scan: slot {slot_num} OVER BUDGET — {used_ms} ms of {budget_ms} ms \
                 ({:.0}%). This receiver is built with slack; exceeding the slot means a fault, \
                 not a busy band",
                used_ms as f64 / budget_ms as f64 * 100.0,
            );
        } else {
            log::info!(
                "fst4_app::scan: slot {slot_num} budget — {used_ms} ms of {budget_ms} ms \
                 ({:.0}%), {} ms spare",
                used_ms as f64 / budget_ms as f64 * 100.0,
                budget_ms - used_ms,
            );
        }
        log::info!(
            "fst4_app::scan: slot {slot_num} per candidate — recentre {} ms, \
             sync-search {} ms, decode {} ms",
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
    /// For `boot_mode::commit_and_restart` when the mode picker
    /// commits — this task cannot write flash itself, its stack is in
    /// PSRAM. Same `"mfsk"` namespace `settings` uses, so one handle
    /// serves both.
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
}

extern "C" fn display_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_display_task` leaked exactly this pointer.
    let ctx = unsafe { Box::from_raw(arg as *mut DisplayCtx) };
    display_loop(*ctx);
}

fn spawn_display_task(ctx: DisplayCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(display_task_entry),
            c"fst4_display".as_ptr(),
            DISPLAY_STACK,
            ptr,
            DISPLAY_PRIORITY,
            core::ptr::null_mut(),
            1,
            MALLOC_CAP_SPIRAM,
        )
    };
    if created != 1 {
        log::error!("fst4_app: failed to create fst4_display task");
    }
}

fn display_loop(ctx: DisplayCtx) -> ! {
    // Kept across the whole loop: the touch controller shares this bus
    // and the mode picker is this receiver's only way back out.
    let mut touch_i2c: Option<esp_idf_hal::i2c::I2cDriver<'static>> = None;
    let mut display = match crate::pmic::init(ctx.i2c0, ctx.pins.gpio12, ctx.pins.gpio11) {
        Ok(mut i2c) => {
            // VBUS boost before the USB host driver, or the host stack
            // sees no device — same sequence `wspr_app` documents.
            // Stay a peripheral while something else is powering the
            // port.
            //
            // One USB-C connector cannot both take power in and hand it
            // out, so "host or peripheral" is a question about the
            // cable, not the build. The FT8 controller has checked this
            // since #163; this receiver did not, and the moment its USB
            // host stopped being opt-in that gap became the board
            // refusing to enumerate on a PC at all — the app takes the
            // PHY before a flasher can reach it, and the only way back
            // is holding the button into DOWNLOAD mode. On WSL every
            // one of those costs a `usbipd attach` as well.
            //
            // Charging is also the useful thing to do while plugged in.
            let external = match crate::pmic::vbus_present(&mut i2c) {
                Ok((present, raw)) => {
                    log::info!(
                        "AXP2101 status1=0x{raw:02x} — VBUS {} (bit5)",
                        if present {
                            "PRESENT (external power)"
                        } else {
                            "absent (battery)"
                        },
                    );
                    present
                }
                Err(e) => {
                    log::warn!("AXP2101 VBUS read failed: {e:#} — assuming battery");
                    false
                }
            };
            let host_mode = !external;
            if external {
                log::warn!(
                    "external USB power detected — staying a peripheral so the battery charges \
                     and the port stays flashable. Unplug from the PC and reset to take audio \
                     from a radio."
                );
            } else if let Err(e) = crate::pmic::enable_usb_host_vbus(&mut i2c) {
                log::error!("BUS_OUT_EN failed: {e:#}");
            }
            // The bus is kept, not dropped: the FT5x06 is on it, and
            // the mode picker is the only way out of this receiver.
            touch_i2c = Some(i2c);

            // Installing the USB host driver **detaches
            // USB-Serial-JTAG**, i.e. the serial console, the moment it
            // returns — which is why `wspr_app` wires UDP log fanout
            // first and why this is opt-in here rather than
            // unconditional.
            //
            // Unconditional now, as `wspr_app` has always been.
            //
            // It was behind `MFSK_FST4_APP_USB_HOST=1` on the stated
            // condition that #163 stayed open — no UAC source had ever
            // enumerated on this board, so installing the host cost the
            // only telemetry channel that works without WiFi and bought
            // nothing. #163 closed 2026-08-23 with ten minutes of live
            // capture, and a build-time flag cannot work at all in a
            // binary that picks its receiver at boot: choosing FST4
            // from the mode picker would land in a receiver whose radio
            // was compiled out.
            if host_mode {
                // `start_host_when_ready` waits for the log sink itself
                // and says what it found, so this receiver no longer
                // reports on it separately.
                crate::uac::start_host_when_ready();
            } else {
                log::info!(
                    "fst4_app: USB host not installed (peripheral mode) — the serial console \
                     stays up and audio falls back to the synthetic generator"
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
                .display_size(crate::board::NATIVE_W, crate::board::NATIVE_H)
                .orientation(Orientation::new().rotate(crate::board::ROTATION))
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
    log::info!("LCD init OK ({}x{})", crate::board::CANVAS_W, crate::board::CANVAS_H);

    {
        let ui = FST4_UI.lock().expect("FST4_UI poisoned");
        if let Err(e) = fst4_list::render_all(&mut display, &ui) {
            log::error!("fst4_app::display: render_all FAILED: {e:?}");
        }
    }

    // Mode picker: held open, so it costs no layout. Centred on this
    // 320x240 panel.
    let touch_int = PinDriver::input(ctx.pins.gpio21, esp_idf_hal::gpio::Pull::Up).ok();
    let mut boot_summary_sent = false;
    let mut rtc_stored = false;
    let mut last_contact = crate::touch::Contact::default();
    let mut picker = mode_picker::ModePicker::new(embedded_graphics::prelude::Point::new(
        (crate::board::CANVAS_W as i32 - mode_picker::WIDTH as i32) / 2,
        (crate::board::CANVAS_H as i32 - mode_picker::height() as i32) / 2,
    ));

    let mut last_dirty = u32::MAX;
    let mut tick: u32 = 0;
    loop {

        // Freeze the tables while the overlay is up — it only
        // redraws on change, so a repaint underneath erases it and it
        // never comes back.
        if picker.is_open() {
            picker.render(&mut display, BootMode::Fst4).ok();
            FreeRtos::delay_ms(50);
            if let (Some(int), Some(i2c)) = (touch_int.as_ref(), touch_i2c.as_mut()) {
                let c = if int.is_low() {
                    crate::touch::read(i2c).unwrap_or_default()
                } else {
                    crate::touch::Contact::default()
                };
                // One line per change of contact state, so an
                // otherwise silent capture separates "nothing was
                // touched" from "touched, but the hold never reached
                // OPEN_MS". The picker's own log only speaks while the
                // overlay is up, which is exactly the case that cannot
                // be reached when the hold is the thing failing.
                if c != last_contact {
                    if c.points > 0 {
                        log::info!("touch: {} pt at ({}, {})", c.points, c.x, c.y);
                    } else {
                        log::info!("touch: released");
                    }
                    last_contact = c;
                }
                if let Some(target) = picker.update(c.points > 0, c.x, c.y) {
                    log::warn!("boot_mode -> {} (touch), restarting", target.label());
                    // Not written here: this task's stack is in
                    // PSRAM, and a flash write aborts from one. See
                    // `boot_mode::commit_and_restart`.
                    boot_mode::commit_and_restart(ctx.nvs.clone(), target);
                }
            }
            if picker.take_just_closed() {
                last_dirty = u32::MAX;
            }
            continue;
        }

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
            // Same bar, same place, in every mode — see
            // `link_bar`'s own doc comment for why it is not a field in
            // this receiver's header.
            // Same 1 Hz re-read the FT8 controller does: the enable
            // bits are only worth showing if something has looked
            // recently.
            if let Some(i2c) = touch_i2c.as_mut() {
                crate::pmic::refresh_power_state(i2c);
                // Store the clock once it becomes real, so the next
                // boot has one before WiFi does. NTP is what makes it
                // real; this is what makes it survive a power cycle.
                if !rtc_stored && mfsk_app_shared::time_sync::utc_now_ms().is_some() {
                    rtc_stored = true;
                    if let Err(e) = crate::rtc::write_from_system_clock(i2c) {
                        log::warn!("rtc: could not store the clock: {e:#}");
                    }
                }
                // Once, the first frame after a log sink exists. In
                // host mode there is no serial console, and everything
                // this reports is printed seconds before WiFi
                // associates — the staging ring has been overwritten by
                // then. Same one-shot the FT8 controller does.
                if !boot_summary_sent
                    && crate::FANOUT.udp.try_lock().map(|g| g.is_some()).unwrap_or(false)
                {
                    boot_summary_sent = true;
                    let (host_attempted, ..) = crate::pmic::power_state();
                    let r = crate::uac::HOST_RESULT.read();
                    log::warn!(
                        "[boot-summary] mode=FST4 host_mode={host_attempted} start_host: {}",
                        if r.is_empty() { "never called" } else { r.as_str() }
                    );
                    let rt = crate::rtc::RTC_RESULT.read();
                    log::warn!(
                        "[boot-summary] rtc: {}",
                        if rt.is_empty() { "no result recorded" } else { rt.as_str() }
                    );
                    crate::pmic::log_boot_summary(i2c);
                }
            }
            link_bar::render(
                &mut display,
                &crate::uac::link_info(),
                fst4_list::PANEL_WIDTH,
                fst4_list::PANEL_HEIGHT as i32 - link_bar::HEIGHT as i32,
            )
            .ok();
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
        // Touch at 50 ms while the render cadence stays at 500.
        //
        // The picker is only as responsive as whatever calls it, and
        // this loop redraws spot tables — it has no business running
        // ten times faster. So the wait polls instead of sleeping
        // through: a tap or an 800 ms hold lands either way. Checking
        // costs one GPIO read while no finger is down.
        for _ in 0..10 {
            if let (Some(int), Some(i2c)) = (touch_int.as_ref(), touch_i2c.as_mut()) {
                let c = if int.is_low() {
                    crate::touch::read(i2c).unwrap_or_default()
                } else {
                    crate::touch::Contact::default()
                };
                // One line per change of contact state, so an
                // otherwise silent capture separates "nothing was
                // touched" from "touched, but the hold never reached
                // OPEN_MS". The picker's own log only speaks while the
                // overlay is up, which is exactly the case that cannot
                // be reached when the hold is the thing failing.
                if c != last_contact {
                    if c.points > 0 {
                        log::info!("touch: {} pt at ({}, {})", c.points, c.x, c.y);
                    } else {
                        log::info!("touch: released");
                    }
                    last_contact = c;
                }
                if let Some(target) = picker.update(c.points > 0, c.x, c.y) {
                    log::warn!("boot_mode -> {} (touch), restarting", target.label());
                    boot_mode::commit_and_restart(ctx.nvs.clone(), target);
                }
                picker.render(&mut display, BootMode::Fst4).ok();
                if picker.take_just_closed() {
                    last_dirty = u32::MAX;
                }
            }
            FreeRtos::delay_ms(50);
        }
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
            crate::WIFI_SSID,
            crate::WIFI_PSK,
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

    // Modem power save. The default is `WIFI_PS_NONE`, which keeps the
    // receiver on continuously and hands every frame on the network —
    // including all the broadcast and multicast traffic a home LAN
    // carries — to a driver task running at FreeRTOS priority 23, above
    // anything this application creates. Measured: an *associated,
    // otherwise idle* STA cost the candidate loop 33 -> 53 s and
    // `fst4_sync_search` 711 -> 1500 ms per candidate.
    //
    // `MIN_MODEM` lets the radio sleep between DTIM beacons, which is
    // all a receiver that only needs NTP, a config page and a log sink
    // ever wanted from the association.
    let r = unsafe { esp_idf_svc::sys::esp_wifi_set_ps(esp_idf_svc::sys::wifi_ps_type_t_WIFI_PS_MIN_MODEM) };
    log::info!("fst4_app::net: esp_wifi_set_ps(MIN_MODEM) -> {r}");

    // UDP log sink — the serial console goes away the moment the USB
    // host driver installs, so this is the only log this app has once
    // it is running for real.
    let target_ip: std::net::IpAddr = if crate::UDP_LOG_TARGET.is_empty() || crate::UDP_LOG_TARGET == "auto" {
        std::net::IpAddr::V4(info.subnet_broadcast)
    } else {
        match crate::UDP_LOG_TARGET.parse() {
            Ok(ip) => ip,
            Err(e) => {
                log::warn!(
                    "fst4_app::net: UDP_LOG_TARGET '{}' parse failed ({e}); \
                     using subnet broadcast",
                    crate::UDP_LOG_TARGET
                );
                std::net::IpAddr::V4(info.subnet_broadcast)
            }
        }
    };
    let addr = std::net::SocketAddr::new(target_ip, crate::UDP_LOG_PORT.parse().unwrap_or(9999));
    match udp_log::UdpLogSink::new(addr) {
        Ok(sink) => {
            if let Ok(mut slot) = crate::FANOUT.udp.try_lock() {
                *slot = Some(sink);
            }
            crate::FANOUT.drain_staging_to_udp();
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
