//! USB Audio Class host capture — Phase 1 iso IN streaming (#30 + #31).
//!
//! `start_host()` installs the ESP-IDF USB host stack + the
//! `espressif/usb_host_uac` class driver, registers a driver event
//! callback that forwards hot-plug events to an app task, spawns the
//! USB events pump, and falls through. From there:
//!
//! - **driver_event_cb** (class-driver task ctx) pushes `RxConnected` /
//!   `TxConnected` onto a `std::sync::mpsc::channel`.
//! - **app_task** (`uac_app`, std::thread) consumes events; on the first
//!   `RxConnected` it calls `uac_host_device_open` + `uac_host_device_start`
//!   with the IC-705's fixed `48 kHz / stereo / 16-bit` config and
//!   spawns the reader thread.
//! - **reader_thread** (`uac_reader`, std::thread) polls
//!   `uac_host_device_read` into a 4 KB stack buffer, accumulates stats,
//!   and logs `bytes/packets/errors` to UDP every ~1 s.
//!
//! The reader currently **drops the samples on the floor** after counting
//! them. Wiring the bytes into the resample + decoder push chain lands
//! in #32 (replace the inner of `reader_thread` with the
//! 48 k stereo → 12 k mono pipeline). Disconnect / reconnect polish is
//! #35 (reader exits on any read error and the device handle leaks —
//! acceptable for #31 since `BootMode::Uac` is sticky until reboot).
//!
//! ## 接続方式 (確定済)
//!
//! - Component: `espressif/usb_host_uac@^1.4` を `Cargo.toml` の
//!   `extra_components` 経由で managed component として取得。
//!   bindings は `esp_idf_svc::sys::uac::*` に生成。
//! - IC-705 USB Audio は **固定 48 kHz / stereo / 16-bit** (16 kHz は
//!   selectable ではない)。S3 側で stereo → mono (L ch 抽出 / R ch 破棄)
//!   + 48 kHz → 12 kHz の 4:1 decimation を `embedded_shared` 経由で
//!   行う (#32)。
//! - Reference example は esp-usb 上流の
//!   `host/class/uac/usb_host_uac/examples/audio_player/main/main.c`。
//!   init 順序 (`usb_host_install` → events task spawn →
//!   `uac_host_install` → `RX_CONNECTED` 通知で device_open →
//!   device_start) をそのまま Rust に移植する。
//!
//! ## OTG 排他
//!
//! `usb_host_install()` を呼んだ瞬間に USB-Serial-JTAG endpoint が
//! detach されるため、`BootMode::Uac` では WiFi STA + UDP log を必ず
//! 起動させる (`main.rs` dispatch arm で強制)。
//!
//! ## 進捗
//!
//! - [x] managed component + bindings (#29)
//! - [x] host install + hot-plug callback (#30)
//! - [x] iso IN streaming + stats logging (#31, このファイル)
//! - [ ] 48 kHz stereo → 12 kHz mono resampler + ringbuf → decode (#32)
//! - [ ] verification on hardware (#33-#34)
//! - [ ] disconnect/reconnect polish (#35)

use std::sync::atomic::{AtomicU32, AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Sender};
use std::sync::OnceLock;

use anyhow::{anyhow, Result};
use embedded_shared::pipeline::{send_box, ChunkMsg, CHUNK_LEN};
use esp_idf_svc::sys;
use mfsk_core::core::dsp::resample::LinearResamplerI16To12k;

/// Newtype around the IDF `uac_host_device_handle_t` (`*mut uac_interface`)
/// to assert thread-safety for the `move` into the reader thread. The
/// IDF UAC driver documents the handle as safe to call from any task
/// once `uac_host_device_start` returns ESP_OK.
struct DeviceHandle(sys::uac::uac_host_device_handle_t);
// SAFETY: per usb_host_uac docs, the handle is opaque to callers and
// the IDF synchronises internal state. We never mutate the pointer or
// dereference its target on the Rust side — every use goes through
// `uac_host_device_*` IDF calls.
unsafe impl Send for DeviceHandle {}

/// USB host event-pump task stack — modest budget; the loop just
/// blocks on `usb_host_lib_handle_events` and dispatches flags.
const USB_EVENTS_TASK_STACK: usize = 4096;

/// UAC class-driver background-task config. 1.4.x supports
/// `tskNO_AFFINITY` but pinning to core 0 (PRO_CPU) matches the upstream
/// audio_player example and keeps the decoder's core 1 (APP_CPU) free.
const UAC_DRIVER_TASK_STACK: usize = 4096;
const UAC_DRIVER_TASK_PRIORITY: usize = 5;
const UAC_DRIVER_TASK_CORE: sys::BaseType_t = 0;

/// `uac_app` task stack. Just runs `recv()` → device_open/start →
/// spawn reader. 4 KB is overkill but keeps headroom for the OnceLock
/// + sender state and any future device-cleanup paths.
const APP_TASK_STACK: usize = 4096;

/// `uac_reader` task stack. 10 KB: 4 KB for the `READER_BUFFER_BYTES`
/// stack array, the rest for the function's frame, `Instant`, the
/// format machinery for the 1 Hz log line (Rust's `format_args!` +
/// `write!` chain is surprisingly stack-heavy on Xtensa), and the
/// IDF `uac_host_device_read` call's own stack usage. Earlier sizes
/// at 4 KB (= buffer alone) and 8 KB both flagged by Gemini PR #98
/// as tight; 10 KB gives the log path real headroom.
const READER_TASK_STACK: usize = 10240;

/// Read buffer size per `uac_host_device_read` call. 4 KB = 1024
/// stereo i16 samples = ~21 ms at 48 kHz stereo — short enough that
/// disconnect detection latency stays under one FT8 symbol period
/// (160 ms), large enough that we're not paying ring-buffer overhead
/// per-sample. #32 may tune this once the resample chain is wired
/// (FT8 symbol-block sized chunks reduce intermediate buffering).
const READER_BUFFER_BYTES: usize = 4096;

/// Read call timeout — short enough that a disconnect surfaces quickly
/// (we currently exit the reader on any non-OK return; #35 will
/// distinguish recoverable from terminal errors), long enough that
/// the loop doesn't poll-spin when the IDF ringbuf is briefly empty.
/// 100 ms ≈ half an FT8 symbol period; matches the audio_player
/// reference example's default.
const READER_READ_TIMEOUT_MS: u32 = 100;

/// IC-705 USB Audio stream config. Fixed by the IC-705 firmware;
/// the device descriptor reports a single supported alt-setting at
/// 48 kHz stereo 16-bit. Embedded 16 kHz path mentioned in early
/// design notes does not exist on this radio.
const STREAM_CHANNELS: u8 = 2;
const STREAM_BIT_RESOLUTION: u8 = 16;
const STREAM_SAMPLE_FREQ_HZ: u32 = 48_000;

/// Class-driver-side ringbuf the IDF code copies iso IN packets into
/// before `uac_host_device_read` drains them. Sized for ~85 ms of
/// audio (48 k × stereo × 2 B × 0.085 ≈ 16 KB) — plenty of slack for
/// us to lag a render frame without losing packets.
const STREAM_BUFFER_BYTES: u32 = 16 * 1024;

/// Threshold the IDF driver uses to decide when to fire `RX_DONE`
/// callbacks. Half the buffer is the canonical setting from the
/// audio_player reference. We don't currently consume the callback
/// (the reader polls), so this only affects how the IDF schedules
/// internal copies; tuning it doesn't change our latency budget.
const STREAM_BUFFER_THRESHOLD: u32 = STREAM_BUFFER_BYTES / 2;

/// Hot-plug event reified for cross-task delivery. Driver events are
/// translated by `driver_event_cb` (which runs in the class-driver
/// background task context and can't block) into one of these and
/// pushed onto the channel that `app_task` reads.
#[derive(Debug, Clone, Copy)]
enum DriverEvent {
    /// A streaming-IN interface enumerated on `addr.iface_num`.
    /// We open + start the first RxConnected we see; subsequent ones
    /// (e.g. multi-channel devices) are logged but ignored for #31.
    RxConnected { addr: u8, iface_num: u8 },
    /// A streaming-OUT interface enumerated. IC-705 exposes one for
    /// CW/mod injection; we don't use it for FT8 RX.
    TxConnected { addr: u8, iface_num: u8 },
}

/// Sender half of the driver→app channel. Populated by `start_host`
/// before `uac_host_install` registers the callback. `OnceLock`
/// (not `OnceCell`) so the C callback context can safely read it.
static EVENT_SENDER: OnceLock<Sender<DriverEvent>> = OnceLock::new();

/// Stats counters maintained by the reader thread. Read once per
/// second by the same thread for the UDP log line. Atomics
/// (`Relaxed`) so a future inspector (e.g. LCD overlay) can sample
/// them lock-free.
static RX_BYTES: AtomicU32 = AtomicU32::new(0);
static RX_PACKETS: AtomicU32 = AtomicU32::new(0);
static RX_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Gate against spawning multiple readers when the IDF driver fires
/// `RxConnected` more than once for the same physical attach (e.g.
/// IC-705 advertises both an RX and TX interface and the driver may
/// reissue events on alt-setting changes). Set by `app_task` via
/// `compare_exchange` before spawning the reader; released either
/// (a) explicitly on `handle_rx_connected` failure or
/// (b) automatically via [`ReaderActiveGuard`] when the reader thread
/// exits (normal exit, error exit, or panic).
static READER_ACTIVE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

/// RAII guard that releases [`READER_ACTIVE`] on drop. Held by
/// `reader_thread` for its entire lifetime so the gate gets reset
/// even on panic — without the guard a panic in the read /
/// resample / push chain would leave the gate stuck `true` and
/// every subsequent `RxConnected` would be ignored until reboot
/// (Gemini PR #98 r4 review).
struct ReaderActiveGuard;
impl Drop for ReaderActiveGuard {
    fn drop(&mut self) {
        READER_ACTIVE.store(false, std::sync::atomic::Ordering::Release);
    }
}

/// Set by [`device_event_cb`] when the IDF driver fires
/// `DRIVER_EVENT_DISCONNECTED` (USB cable unplug, IC-705 power off,
/// VBUS sag). The reader thread polls this at the top of every loop
/// iteration and exits cleanly — disconnect latency = at most one
/// `READER_READ_TIMEOUT_MS` (100 ms) instead of waiting for the next
/// `device_read` to fail. Also lets the reader's cleanup path skip
/// `device_stop` / `device_close` (the IDF driver already
/// invalidated the handle when DISCONNECTED fired) so the post-
/// disconnect cleanup doesn't log spurious `INVALID_ARG` errors.
///
/// Reset by [`handle_rx_connected`] at the start of a new session so
/// a stale `true` from a previous attach can't kill the freshly-
/// spawned reader on its first iteration.
static READER_STOP_REQUESTED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Chunk queue handle the reader thread pushes decimated 12 kHz mono
/// samples into. Set by [`set_chunk_q`] from the `decode_pipeline`'s
/// source-spawn closure (`run_with_source(|q| uac::set_chunk_q(q))`).
/// Stored as `usize` because `QueueHandle_t` is `*mut QueueDefinition`
/// which isn't `Sync` by default; reader thread re-casts on each use.
/// `0` = not yet wired (reader logs + drops samples until decode pipeline
/// reaches the source-spawn step).
static CHUNK_Q_ADDR: AtomicUsize = AtomicUsize::new(0);

/// Wire the chunk queue handle. Called once from
/// `decode_pipeline::run_with_source`'s source-spawn closure in the
/// pipeline thread, before the decode loop blocks on `recv_box`.
pub fn set_chunk_q(q: sys::QueueHandle_t) {
    CHUNK_Q_ADDR.store(q as usize, Ordering::Release);
    log::info!("uac: chunk_q wired (addr={:#x})", q as usize);
}

/// `SlotEnd` cadence in 12 kHz mono samples. Same as `wav_sim`'s
/// `SLOT_SAMPLES` — 180_000 = 15 s @ 12 kHz, one FT8 slot. UAC
/// streams continuously so the reader synthesizes the slot boundary
/// from the post-resample sample count. **No wall-clock alignment**
/// in #32 — the slot is bound by sample count, not by UTC :00/:15/:30/:45.
/// Real wall-clock alignment lands with the NTP-fed time_sync hook in
/// #34 verification (decode DT will then read as offset from the
/// midpoint of whatever 15 s window the reader happened to start in).
const SLOT_SAMPLES_12K: usize = 180_000;

/// Driver event callback. Invoked by the UAC class-driver background
/// task on every `RX_CONNECTED` / `TX_CONNECTED` notification (i.e.
/// every time an audio streaming interface enumerates).
///
/// Runs in the class-driver task context — must not block / allocate
/// significantly. We forward to the app task via the mpsc channel
/// (lock-free for the single-producer case) and return.
extern "C" fn driver_event_cb(
    addr: u8,
    iface_num: u8,
    event: sys::uac::uac_host_driver_event_t,
    _arg: *mut core::ffi::c_void,
) {
    let driver_event = match event {
        sys::uac::uac_host_driver_event_t_UAC_HOST_DRIVER_EVENT_RX_CONNECTED => {
            DriverEvent::RxConnected { addr, iface_num }
        }
        sys::uac::uac_host_driver_event_t_UAC_HOST_DRIVER_EVENT_TX_CONNECTED => {
            DriverEvent::TxConnected { addr, iface_num }
        }
        other => {
            log::warn!("uac: unknown driver event addr={addr} iface={iface_num} raw={other}");
            return;
        }
    };
    log::info!("uac: driver event {driver_event:?}");
    if let Some(sender) = EVENT_SENDER.get() {
        if let Err(e) = sender.send(driver_event) {
            log::error!("uac: app channel send failed (app_task gone): {e}");
        }
    } else {
        log::error!("uac: driver event before EVENT_SENDER init — dropped {driver_event:?}");
    }
}

/// Device-level event callback. Set in `uac_host_device_config_t` at
/// `uac_host_device_open` time, fires on `RX_DONE` / `TX_DONE` /
/// `TRANSFER_ERROR` / `DRIVER_EVENT_DISCONNECTED`. #31 only logs
/// these — the reader thread polls `uac_host_device_read` rather
/// than waiting on the callback. #35 will use `DISCONNECTED` here
/// to signal the reader to stop and clean up the handle.
extern "C" fn device_event_cb(
    _handle: sys::uac::uac_host_device_handle_t,
    event: sys::uac::uac_host_device_event_t,
    _arg: *mut core::ffi::c_void,
) {
    let kind = match event {
        sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_RX_DONE => "RX_DONE",
        sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_TX_DONE => "TX_DONE",
        sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR => "TRANSFER_ERROR",
        sys::uac::uac_host_device_event_t_UAC_HOST_DRIVER_EVENT_DISCONNECTED => "DISCONNECTED",
        other => {
            log::warn!("uac: unknown device event raw={other}");
            return;
        }
    };
    // RX_DONE is the high-frequency one (every ~10 ms once streaming);
    // logging it would saturate UDP. Suppress, log only the other
    // three which are exceptional.
    if event != sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_RX_DONE {
        log::info!("uac: device event {kind}");
    }
    // Disconnect signal: the IDF driver invalidates the handle after
    // this callback returns, so any pending `device_read` will fail.
    // Setting the flag lets the reader thread exit on its next loop
    // iteration (≤ 100 ms latency) instead of waiting for the failing
    // read to surface — and lets the reader skip the
    // `device_stop` / `device_close` cleanup since the IDF already
    // released the underlying state (#35 disconnect polish).
    if event == sys::uac::uac_host_device_event_t_UAC_HOST_DRIVER_EVENT_DISCONNECTED {
        READER_STOP_REQUESTED.store(true, Ordering::Release);
    }
}

/// USB host event-pump body. Blocks indefinitely on
/// `usb_host_lib_handle_events`. Returned `event_flags` are
/// intentionally ignored (see PR #92 review history for why).
fn usb_events_task() {
    const FOREVER: sys::TickType_t = sys::TickType_t::MAX;
    loop {
        let mut event_flags: u32 = 0;
        let err = unsafe { sys::usb_host_lib_handle_events(FOREVER, &mut event_flags as *mut u32) };
        if err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: usb_host_lib_handle_events err={err:#x}");
            if err == sys::ESP_ERR_INVALID_STATE as sys::esp_err_t {
                break;
            }
            esp_idf_svc::hal::delay::FreeRtos::delay_ms(50);
            continue;
        }
        let _ = event_flags;
    }
    log::error!("uac: usb_events_task exiting — host stack gone");
}

/// Convert a `(addr, iface_num)` `RxConnected` into an open + started
/// UAC device + reader thread. Returns the device handle on success
/// (currently unused — the handle moves into the reader thread; #35
/// will need a way to signal it on disconnect).
fn handle_rx_connected(addr: u8, iface_num: u8) -> Result<()> {
    log::info!("uac: opening device addr={addr} iface={iface_num}");
    let dev_config = sys::uac::uac_host_device_config_t {
        addr,
        iface_num,
        buffer_size: STREAM_BUFFER_BYTES,
        buffer_threshold: STREAM_BUFFER_THRESHOLD,
        callback: Some(device_event_cb),
        callback_arg: core::ptr::null_mut(),
    };
    let mut handle: sys::uac::uac_host_device_handle_t = core::ptr::null_mut();
    let err = unsafe {
        sys::uac::uac_host_device_open(
            &dev_config as *const _,
            &mut handle as *mut sys::uac::uac_host_device_handle_t,
        )
    };
    if err != sys::ESP_OK as sys::esp_err_t {
        return Err(anyhow!(
            "uac_host_device_open(addr={addr}, iface={iface_num}) failed err={err:#x}"
        ));
    }
    log::info!("uac: device opened, starting stream {STREAM_CHANNELS}ch / {STREAM_BIT_RESOLUTION}b / {STREAM_SAMPLE_FREQ_HZ}Hz");

    let stream_config = sys::uac::uac_host_stream_config_t {
        channels: STREAM_CHANNELS,
        bit_resolution: STREAM_BIT_RESOLUTION,
        sample_freq: STREAM_SAMPLE_FREQ_HZ,
        flags: 0,
    };
    let err = unsafe { sys::uac::uac_host_device_start(handle, &stream_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        // Best-effort close; if it fails we can't do much beyond logging.
        let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: device_close after device_start failure also failed err={close_err:#x}"
            );
        }
        return Err(anyhow!(
            "uac_host_device_start failed err={err:#x} (config 48k/stereo/16b — IC-705 should support this; check the device descriptor in UDP log)"
        ));
    }

    // Reader spawn failure rollback: device_open + device_start
    // succeeded, so the handle owns USB resources; bail without
    // releasing them would mean the IDF driver thinks the device is
    // streaming forever (next RxConnected would race against a stuck
    // alt-setting). Stop + close before bubbling the error.
    // Reset stats for the new session so the 1 Hz throughput log
    // reflects the current device, not accumulated bytes from a
    // previous attach (Gemini PR #98 r3 review). `Relaxed` since
    // no concurrent reader exists at this point — the new reader
    // is about to spawn below.
    RX_BYTES.store(0, Ordering::Relaxed);
    RX_PACKETS.store(0, Ordering::Relaxed);
    RX_ERRORS.store(0, Ordering::Relaxed);

    // Clear any stale stop-request from a previous attach (#35).
    // The previous session's `device_event_cb` may have set this if
    // the device was unplugged before the reader noticed; we don't
    // want it tripping the fresh reader on its first iteration.
    READER_STOP_REQUESTED.store(false, Ordering::Release);

    let handle_wrapped = DeviceHandle(handle);
    if let Err(e) = std::thread::Builder::new()
        .stack_size(READER_TASK_STACK)
        .name("uac_reader".into())
        .spawn(move || reader_thread(handle_wrapped))
    {
        let stop_err = unsafe { sys::uac::uac_host_device_stop(handle) };
        let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
        if stop_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: device_stop after reader spawn failure also failed err={stop_err:#x}"
            );
        }
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: device_close after reader spawn failure also failed err={close_err:#x}"
            );
        } else {
            log::info!("uac: rolled back device_open + device_start after reader spawn failure");
        }
        return Err(anyhow!("uac_reader spawn failed: {e}"));
    }
    log::info!("uac: reader thread spawned");
    Ok(())
}

/// Reader thread body. Polls `uac_host_device_read` for raw
/// 48 kHz/stereo/16-bit iso IN packets, extracts the left channel,
/// resamples to 12 kHz mono via `LinearResamplerI16To12k`, and pushes
/// `CHUNK_LEN`-sized chunks into the decode pipeline's chunk queue
/// (with `SlotEnd` every `SLOT_SAMPLES_12K` samples).
///
/// Counts iso IN throughput in `RX_BYTES` / `RX_PACKETS` / `RX_ERRORS`
/// and logs a 1 Hz status line.
fn reader_thread(handle: DeviceHandle) {
    // RAII: clears READER_ACTIVE on any exit path including panic.
    let _gate = ReaderActiveGuard;
    let mut buf = [0u8; READER_BUFFER_BYTES];
    let mut resampler = LinearResamplerI16To12k::new(STREAM_SAMPLE_FREQ_HZ);
    // L-channel scratch (one device_read worth of mono samples). At
    // 4 KB raw / stereo i16, max = 1024 mono samples per read.
    let mut left_scratch = [0i16; READER_BUFFER_BYTES / 4];
    // Resampled output staging. Sized for at most one read's worth
    // of input → ~256 output samples at 48k→12k (4:1). Doubled for
    // headroom against the resampler's per-call rounding.
    let mut dst_scratch = [0i16; 512];
    // Per-chunk accumulator. Filled to exactly `CHUNK_LEN` (1200,
    // = 100 ms @ 12 kHz, matching wav_sim) before flushing.
    let mut chunk: Vec<i16> = Vec::with_capacity(CHUNK_LEN);
    // Per-slot sample count. SlotEnd emitted every SLOT_SAMPLES_12K.
    let mut slot_samples: usize = 0;
    let mut wav_idx: usize = 0;
    let mut last_log = std::time::Instant::now();
    let mut last_bytes: u32 = 0;
    // Track whether we exited via DISCONNECTED so the cleanup path
    // can skip the redundant `device_stop` / `device_close` calls
    // (#35) — the IDF driver already invalidated the handle at
    // callback time, so calling them just logs spurious INVALID_ARG.
    let mut disconnect_triggered = false;
    loop {
        // Top-of-loop disconnect check (#35). `device_event_cb` sets
        // the flag immediately on DISCONNECTED; we exit on the next
        // iteration (≤ READER_READ_TIMEOUT_MS = 100 ms latency)
        // rather than waiting for the failing read to surface.
        if READER_STOP_REQUESTED.load(Ordering::Acquire) {
            log::info!("uac: reader exiting — DISCONNECTED signaled by device_event_cb");
            disconnect_triggered = true;
            break;
        }
        let mut bytes_read: u32 = 0;
        let err = unsafe {
            sys::uac::uac_host_device_read(
                handle.0,
                buf.as_mut_ptr(),
                buf.len() as u32,
                &mut bytes_read as *mut u32,
                READER_READ_TIMEOUT_MS,
            )
        };
        if err != sys::ESP_OK as sys::esp_err_t {
            // ESP_ERR_TIMEOUT is a routine ringbuf-empty signal (the
            // 100 ms timeout fires when the IDF driver hasn't yet
            // received a fresh iso IN frame). NOT a reason to exit
            // (Gemini PR #98 review). Just continue the loop.
            if err == sys::ESP_ERR_TIMEOUT as sys::esp_err_t {
                continue;
            }
            RX_ERRORS.fetch_add(1, Ordering::Relaxed);
            // Other errors (INVALID_STATE on disconnect, INVALID_ARG,
            // ringbuf failures) are terminal — exit so #35 can swap in
            // a proper detect+reset path. `BootMode::Uac` is
            // sticky-until-reboot so silent reader death is at least
            // observable in the next UDP log tick (rx=0B/s).
            log::error!("uac: device_read err={err:#x}, reader exiting");
            break;
        }
        if bytes_read == 0 {
            // 0-byte read = timeout-with-no-data (rare); skip the
            // resample / push pipeline and continue.
            continue;
        }
        // RX_BYTES uses `fetch_add` which wraps on AtomicU32 overflow
        // (~4 GB ≈ 6 h of streaming). xtensa-esp32s3 has no 64-bit
        // atomic intrinsics. The wrap is harmless for the 1 Hz delta
        // computation below — `wrapping_sub` on the u32 values gives
        // the correct per-second window even across the boundary.
        RX_BYTES.fetch_add(bytes_read, Ordering::Relaxed);
        RX_PACKETS.fetch_add(1, Ordering::Relaxed);

        // Decode interleaved stereo i16 → take left channel only.
        // bytes_read is always a multiple of 4 (stereo i16) per the
        // IDF driver's frame alignment. left_scratch is sized for
        // the max bytes_read / 4 case so the slice can't overflow.
        let stereo_samples = (bytes_read as usize) / 4;
        debug_assert!(stereo_samples <= left_scratch.len());
        for i in 0..stereo_samples {
            let off = i * 4;
            // i16 LE, little-endian (USB Audio Class default).
            left_scratch[i] = i16::from_le_bytes([buf[off], buf[off + 1]]);
        }

        // Load the chunk_q handle each loop iteration so the reader
        // gracefully bridges the gap between USB enumeration and
        // decode_pipeline init. While the gate is unwired we just
        // drop the current read buffer (lossy by design — the race
        // window is bounded by how fast pipeline thread can spawn +
        // alloc BASIS, ~200 ms). Gemini PR #99 review fixed the
        // earlier "accumulates samples" wording which contradicted
        // the actual `continue`.
        let chunk_q_addr = CHUNK_Q_ADDR.load(Ordering::Acquire);
        if chunk_q_addr == 0 {
            continue;
        }
        let chunk_q = chunk_q_addr as sys::QueueHandle_t;

        // Feed the resampler in a loop until the input is drained.
        // process() returns (consumed, produced); if consumed < input
        // we loop back with the unconsumed tail.
        let mut src_offset = 0usize;
        while src_offset < stereo_samples {
            let (consumed, produced) =
                resampler.process(&left_scratch[src_offset..stereo_samples], &mut dst_scratch);
            for &s in &dst_scratch[..produced] {
                chunk.push(s);
                if chunk.len() >= CHUNK_LEN {
                    let to_send = core::mem::replace(&mut chunk, Vec::with_capacity(CHUNK_LEN));
                    send_box(chunk_q, Box::new(ChunkMsg::Samples(to_send)));
                    slot_samples += CHUNK_LEN;
                    if slot_samples >= SLOT_SAMPLES_12K {
                        send_box(
                            chunk_q,
                            Box::new(ChunkMsg::SlotEnd {
                                wav_idx,
                                total_samples: slot_samples,
                            }),
                        );
                        wav_idx = wav_idx.wrapping_add(1);
                        slot_samples = 0;
                    }
                }
            }
            // Defensive: if process() makes zero progress (shouldn't,
            // given the input is non-empty), break to avoid an
            // infinite loop. Pull the un-resampled tail to the next
            // call's left_scratch start by shifting.
            if consumed == 0 && produced == 0 {
                break;
            }
            src_offset += consumed;
        }

        // 1 Hz throughput log. `Instant::now()` is the FreeRTOS tick
        // count under the hood — sub-microsecond cost.
        let now = std::time::Instant::now();
        if now.duration_since(last_log).as_secs() >= 1 {
            let bytes = RX_BYTES.load(Ordering::Relaxed);
            let packets = RX_PACKETS.load(Ordering::Relaxed);
            let errors = RX_ERRORS.load(Ordering::Relaxed);
            let bps = bytes.wrapping_sub(last_bytes);
            // 48 k × stereo × 2 B = 192_000 B/s expected for a fully-
            // streaming IC-705. The throughput delta is the diagnostic
            // we care about here (anything well below ~190 kB/s
            // suggests packet drops or wrong stream config).
            log::info!(
                "uac: rx tick: {bps} B/s (total {bytes} B / {packets} pkt / {errors} err / slot={slot_samples}/12k)"
            );
            last_log = now;
            last_bytes = bytes;
        }
    }
    // Cleanup paths differ by exit reason (#35):
    //
    // - Disconnect-triggered exit: the IDF driver already invalidated
    //   the handle when DISCONNECTED fired in `device_event_cb`.
    //   Calling `device_stop` / `device_close` on the invalidated
    //   handle returns INVALID_ARG/STATE — harmless but it would log
    //   confusing errors. Skip.
    // - Read-error exit (terminal non-TIMEOUT error from device_read):
    //   the handle is still nominally valid; explicit stop + close
    //   releases the IDF state so a re-enumeration takes a clean path.
    //
    // Either way `_gate: ReaderActiveGuard` drops below to release
    // `READER_ACTIVE` so the next RxConnected can re-take it.
    if disconnect_triggered {
        log::info!(
            "uac: reader_thread cleanup (disconnect path — skipping device_stop/close, IDF already invalidated handle)"
        );
    } else {
        let stop_err = unsafe { sys::uac::uac_host_device_stop(handle.0) };
        if stop_err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: device_stop on reader exit failed err={stop_err:#x}");
        }
        let close_err = unsafe { sys::uac::uac_host_device_close(handle.0) };
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: device_close on reader exit failed err={close_err:#x}");
        } else {
            log::info!("uac: reader_thread cleanup complete (device stopped + closed)");
        }
    }
    // `_gate: ReaderActiveGuard` is dropped here, clearing
    // READER_ACTIVE so the next RxConnected can re-take the gate
    // (Gemini PR #98 r3 + r4 review — RAII so panic also clears).
}

/// App task body. Consumes driver events from the channel; on first
/// `RxConnected` opens the device + starts the stream + spawns the
/// reader. Subsequent `RxConnected` events (e.g. a hub adds another
/// audio device, or IC-705 re-enumerates after a USB reset) are
/// re-handled — #35 will refine this to detect "same device returned"
/// vs "new device" and clean up the previous handle.
fn app_task(rx: std::sync::mpsc::Receiver<DriverEvent>) {
    while let Ok(event) = rx.recv() {
        match event {
            DriverEvent::RxConnected { addr, iface_num } => {
                // Dedup: the IDF driver re-fires `RxConnected` on alt-
                // setting transitions and (per Gemini PR #98 review)
                // on multi-interface devices, which would spawn
                // additional reader threads racing the first on
                // device_start. compare_exchange wins atomically;
                // losers just log and drop the event.
                if READER_ACTIVE
                    .compare_exchange(
                        false,
                        true,
                        std::sync::atomic::Ordering::AcqRel,
                        std::sync::atomic::Ordering::Acquire,
                    )
                    .is_err()
                {
                    log::info!(
                        "uac: ignoring duplicate RxConnected addr={addr} iface={iface_num} (reader already active)"
                    );
                    continue;
                }
                if let Err(e) = handle_rx_connected(addr, iface_num) {
                    log::error!("uac: RxConnected handler failed: {e:#}");
                    // Release the gate so a future re-attach can retry
                    // (e.g. IC-705 power cycle during bring-up).
                    READER_ACTIVE.store(false, std::sync::atomic::Ordering::Release);
                }
            }
            DriverEvent::TxConnected { addr, iface_num } => {
                log::info!(
                    "uac: TX_CONNECTED addr={addr} iface={iface_num} — ignored (RX only for FT8)"
                );
            }
        }
    }
    log::error!("uac: app_task exiting — driver event channel closed");
}

/// Install the USB host stack + the UAC class driver. Returns once
/// both are running in their respective background tasks; the caller
/// (main.rs UAC dispatch arm) falls through to the display loop.
pub fn start_host() -> Result<()> {
    // Set up the driver→app channel BEFORE installing the class
    // driver — `driver_event_cb` may fire as soon as `uac_host_install`
    // returns (an IC-705 already plugged in would enumerate immediately).
    let (tx, rx) = channel::<DriverEvent>();
    EVENT_SENDER
        .set(tx)
        .map_err(|_| anyhow!("uac: EVENT_SENDER double init — start_host called twice?"))?;
    std::thread::Builder::new()
        .stack_size(APP_TASK_STACK)
        .name("uac_app".into())
        .spawn(move || app_task(rx))
        .map_err(|e| anyhow!("uac_app spawn failed: {e}"))?;
    log::info!("uac: app_task spawned (stack={APP_TASK_STACK} B)");

    log::info!("uac: installing USB host stack");
    let host_config = sys::usb_host_config_t {
        skip_phy_setup: false,
        root_port_unpowered: false,
        intr_flags: sys::ESP_INTR_FLAG_LEVEL1 as i32,
        enum_filter_cb: None,
        peripheral_map: 0,
        fifo_settings_custom: sys::usb_host_config_t__bindgen_ty_1 {
            nptx_fifo_lines: 0,
            ptx_fifo_lines: 0,
            rx_fifo_lines: 0,
        },
    };
    let err = unsafe { sys::usb_host_install(&host_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        return Err(anyhow!("usb_host_install failed (err={err:#x})"));
    }

    if let Err(e) = std::thread::Builder::new()
        .stack_size(USB_EVENTS_TASK_STACK)
        .name("usb_events".into())
        .spawn(usb_events_task)
    {
        let uninstall_err = unsafe { sys::usb_host_uninstall() };
        if uninstall_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: usb_host_uninstall after spawn failure also failed (err={uninstall_err:#x}); host stack left in inconsistent state"
            );
        } else {
            log::info!("uac: rolled back usb_host_install after spawn failure");
        }
        return Err(anyhow!("usb_events_task spawn failed: {e}"));
    }
    log::info!("uac: usb_events_task spawned (stack={USB_EVENTS_TASK_STACK} B)");

    log::info!("uac: installing UAC class driver");
    let uac_config = sys::uac::uac_host_driver_config_t {
        create_background_task: true,
        task_priority: UAC_DRIVER_TASK_PRIORITY,
        stack_size: UAC_DRIVER_TASK_STACK,
        core_id: UAC_DRIVER_TASK_CORE,
        callback: Some(driver_event_cb),
        callback_arg: core::ptr::null_mut(),
    };
    let err = unsafe { sys::uac::uac_host_install(&uac_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        // Rollback: unblock events pump, wait for settle, uninstall host stack.
        let unblock_err = unsafe { sys::usb_host_lib_unblock() };
        if unblock_err != sys::ESP_OK as sys::esp_err_t {
            log::warn!("uac: usb_host_lib_unblock before rollback returned err={unblock_err:#x}");
        }
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);
        let uninstall_err = unsafe { sys::usb_host_uninstall() };
        if uninstall_err != sys::ESP_OK as sys::esp_err_t {
            log::error!(
                "uac: usb_host_uninstall after uac_host_install failure also failed (err={uninstall_err:#x}); host stack left in inconsistent state"
            );
        } else {
            log::info!("uac: rolled back usb_host_install after uac_host_install failure");
        }
        return Err(anyhow!("uac_host_install failed (err={err:#x})"));
    }

    log::info!(
        "uac: host + class driver up — waiting for IC-705 enumeration (driver task core={UAC_DRIVER_TASK_CORE}, prio={UAC_DRIVER_TASK_PRIORITY})"
    );
    Ok(())
}
