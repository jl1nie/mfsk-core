//! USB Audio Class host capture — Phase 1 iso IN streaming (#163).
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
//! The reader resamples 48 k stereo → 12 k mono and pushes into
//! whichever [`AudioSink`] the running binary registered via
//! [`set_audio_sink`] — `main.rs`'s FT8 controller wires
//! [`set_chunk_q`] (its pre-existing chunk-queue sink, now
//! [`Ft8ChunkSink`] under the hood), `wspr_app.rs` wires its own DDC
//! push sink. Samples are dropped on the floor only while no sink is
//! registered yet (race window at boot, bounded). Disconnect and
//! re-open are handled: `DISCONNECTED` sets [`READER_STOP_REQUESTED`],
//! a stall watchdog covers the overflow case that ends the stream
//! without ever returning an error, and the re-open gate retries the
//! interface. What is left is telling "same device returned" from
//! "new device" in [`app_task`] — no issue filed for it.
//!
//! `TxConnected` (the IC-705's USB audio OUT interface) enumerates
//! today and is otherwise ignored — FT8 RX never needed it. Phase T0
//! of the TX/QSO feasibility work adds an opt-in probe,
//! `handle_tx_connected`, gated on `TX_PROBE_ENABLED`
//! (`MFSK_CORES3_TX_PROBE` at build time): open the interface, write
//! digital silence a fixed number of times, close it again. Silence
//! only — see that function's doc comment for why a nonzero test tone
//! is not safe to send without knowing the radio's PTT-source setting.
//!
//! ## 接続方式 (確定済)
//!
//! - Component: `espressif/usb_host_uac@^1.4` を `Cargo.toml` の
//!   `extra_components` 経由で managed component として取得。
//!   bindings は `esp_idf_svc::sys::uac::*` に生成。
//! - IC-705 USB Audio は **固定 48 kHz / stereo / 16-bit** (16 kHz は
//!   selectable ではない)。S3 側で stereo → mono (L ch 抽出 / R ch 破棄)
//!   + 48 kHz → 12 kHz の 4:1 decimation を `embedded_shared` 経由で
//!   行う。
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
//! - [x] managed component + bindings
//! - [x] host install + hot-plug callback
//! - [x] iso IN streaming + stats logging (このファイル)
//! - [x] 48 kHz stereo → 12 kHz mono resampler + sink push (FT8
//!   side; `wspr_app.rs` wires its own DDC-push sink the same way)
//! - [x] verification on hardware — #163, closed 2026-08-23: an
//!   IC-705 sustained 192,512 B/s for ten minutes (125 MB, 30,520
//!   packets, zero errors) with the FT8 decode pipeline running slots
//!   off the live stream. `wspr_app` / `fst4_app` share this file but
//!   have not been run against a radio (#313).
//! - [x] disconnect/reconnect — the DISCONNECTED signal, the stall
//!   watchdog and the re-open gate all shipped with #163; "same device
//!   returned" detection in [`app_task`] is what remains.

use std::sync::atomic::{AtomicI32, AtomicU32, Ordering};
use std::sync::mpsc::{channel, Sender};
use std::sync::{Mutex, OnceLock};

use anyhow::{anyhow, Result};
use embedded_shared::pipeline::{send_box, ChunkMsg, CHUNK_LEN};
use esp_idf_svc::hal::task::thread::{MallocCap, ThreadSpawnConfiguration};
use esp_idf_svc::sys;
use mfsk_core::engine::dsp::resample::LinearResamplerI16To12k;

/// Spawn a named `std::thread` with its stack allocated from PSRAM
/// instead of internal DRAM.
///
/// Root-caused during the `wspr_app` crash-loop investigation
/// (2026-08-16): `spawn_network_task`'s own doc comment already
/// records that `wifi_driver_init` alone drives internal DRAM from
/// ~57 KB free down to ~10 KB free / 7 KB largest contiguous block —
/// and that measurement predates the `fst4-bench` factory-partition
/// growth the same day. A real device log
/// (`wspr_app_restore_2026-08-16f.log`) showed **2 KB** free after
/// `wifi_driver_init`, and every one of this module's three
/// `std::thread::Builder` spawns (`uac_app` 4 KiB, `usb_events`
/// 4 KiB, `uac_reader` 10 KiB) allocated its stack from that same
/// internal-DRAM pool via the ESP-IDF pthread compat layer — the
/// first (`uac_app`) failed outright (`Failed to create task!` /
/// `Not enough space`), which was already a known, gracefully-logged
/// condition. What wasn't understood until this investigation: with
/// internal DRAM this tight, small (<4 KiB —
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096` forces anything below
/// that size into internal DRAM regardless of PSRAM availability)
/// allocations made later by `scan_loop`/`ddc_loop` on the *other*
/// tasks can fail too, and on this esp-idf-svc std target an alloc
/// failure surfaces as a genuine Rust panic rather than an abort —
/// which then poisons whichever `std::sync::Mutex` it held
/// (`BASEBAND_BUFS`/`DDC_READY_IDX`/`WSPR_UI`/`ctx.nvs`), so the next
/// task to touch that lock panics too. That's the double-panic crash
/// loop, and it explains why the observed backtrace lands in a
/// different task on different boots — it's whichever task loses the
/// race to be second.
///
/// Mirrors `spawn_network_task`'s existing PSRAM-stack fix (same
/// `MALLOC_CAP_SPIRAM`/`CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y`
/// mechanism, just via `std::thread`'s `esp_pthread_cfg_t` hook
/// instead of `xTaskCreatePinnedToCoreWithCaps` since these three are
/// `std::thread`s, not raw FreeRTOS tasks) rather than inventing a
/// new one. `ThreadSpawnConfiguration::set()` only affects spawns
/// made by *this* calling thread (it's per-caller state in the IDF
/// pthread layer, not global), so this is reset back to the default
/// config immediately after spawning — the calling thread's own
/// stack, and anything it spawns later without going through this
/// helper, is unaffected.
fn spawn_psram_thread<F>(
    name: &'static core::ffi::CStr,
    stack_size: usize,
    f: F,
) -> std::io::Result<std::thread::JoinHandle<()>>
where
    F: FnOnce() + Send + 'static,
{
    // `name` goes into the spawn configuration, not into
    // `Builder::name()`. The latter is Rust-side only, so every task
    // here reported as 'pthread' — and a coredump that says
    // `task 'pthread'` cannot tell four candidate threads apart, which
    // is where a stack-overflow hunt stalls. Refs #163.
    let psram_cfg = ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        ..Default::default()
    };
    if let Err(e) = psram_cfg.set() {
        log::warn!("uac: ThreadSpawnConfiguration::set (PSRAM stack) failed for {name:?}: {e:?} — falling back to internal-DRAM stack");
    }
    let result = std::thread::Builder::new().stack_size(stack_size).spawn(f);
    // Restore the default (internal-DRAM) config regardless of
    // whether the spawn above succeeded, so this calling thread's
    // own subsequent spawns (if any) aren't silently left on PSRAM.
    let default_cfg = ThreadSpawnConfiguration::default();
    if let Err(e) = default_cfg.set() {
        log::warn!("uac: ThreadSpawnConfiguration::set (restore default) failed: {e:?}");
    }
    result
}

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

/// `uac_reader` task stack.
///
/// **Measured, finally, rather than estimated.** This constant went
/// 4 KB -> 8 KB -> 10 KB, each step a review comment saying the
/// previous looked tight (PR #98), and each step still an estimate.
/// With a radio actually streaming, `uxTaskGetStackHighWaterMark`
/// reported **36 bytes** free at the low-water point: the true usage
/// is ~10.2 KB against a 10,240 B stack.
///
/// That is what was rebooting the board 40-76 s into every capture
/// session on 2026-08-23. It never showed as a stack overflow because
/// nothing was checking — until `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK`
/// turned it into an immediate panic naming this task, instead of a
/// silent write past the end of the stack. Whether a given boot
/// survived came down to how deeply interrupts happened to be nested
/// at the next context switch, which is why it looked intermittent.
///
/// What actually lives here: `READER_BUFFER_BYTES` (4 KB) plus
/// `left_scratch` (2 KB) plus `dst_scratch` (1 KB) as stack arrays,
/// the resampler, and the 1 Hz log line — `format_args!` through the
/// fanout is far heavier on Xtensa than it looks, and it is on this
/// path once a second.
///
/// 24 KB, and it is free: this task is spawned through
/// [`spawn_psram_thread`], so its stack is PSRAM and none of this
/// competes with the internal DRAM the USB host stack needs.
/// Re-check `[stack] uac_reader hw=` in the log before trimming it.
const READER_TASK_STACK: usize = 24576;

/// Read buffer size per `uac_host_device_read` call. 4 KB = 1024
/// stereo i16 samples = ~21 ms at 48 kHz stereo — short enough that
/// disconnect detection latency stays under one FT8 symbol period
/// (160 ms), large enough that we're not paying ring-buffer overhead
/// per-sample. Sizing it to the sink's own chunk geometry would cut
/// one round of intermediate buffering; unmeasured, and the geometry
/// it would have to match is per-sink now.
const READER_BUFFER_BYTES: usize = 4096;

/// Read call timeout — short enough that a disconnect surfaces quickly
/// (`ESP_ERR_TIMEOUT` is routine ringbuf-empty and continues; every
/// other error ends the session and routes to the re-open gate),
/// long enough that
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
    /// (e.g. multi-channel devices) are logged but ignored.
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
/// What the USB side is doing, for the screen.
///
/// A receiver whose only feedback is a log line over WiFi is not a
/// receiver you can use: plugging the radio in has to show something,
/// now, on the device itself. This is the state the display renders,
/// updated at every transition and once a second while streaming.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum UacState {
    /// Host stack not installed (wrong boot mode, or install failed).
    Off,
    /// Installed, nothing attached — the state a bare board sits in.
    Waiting,
    /// A device enumerated and its audio interface was opened.
    Streaming,
    /// Attached but the stream could not be started, or the reader
    /// died. Distinct from `Waiting` because the fix is different.
    Error,
}

static UAC_STATE: AtomicU32 = AtomicU32::new(0);
/// Post-resample 12 kHz samples in the last tick — the rate check.
static UAC_SA_PER_S: AtomicU32 = AtomicU32::new(0);
/// Signal level in the last tick, as −dBFS × 10 (so 324 = −32.4 dBFS).
/// `u32::MAX` means "no samples yet".
static UAC_RMS_MDB: AtomicU32 = AtomicU32::new(u32::MAX);

pub(crate) fn set_state(st: UacState) {
    UAC_STATE.store(st as u32, Ordering::Release);
}

/// The USB side's current state and its last-second signal figures.
pub fn status() -> (UacState, u32, Option<f32>) {
    let st = match UAC_STATE.load(Ordering::Acquire) {
        1 => UacState::Waiting,
        2 => UacState::Streaming,
        3 => UacState::Error,
        _ => UacState::Off,
    };
    let rms = UAC_RMS_MDB.load(Ordering::Acquire);
    let rms = (rms != u32::MAX).then(|| -(rms as f32) / 10.0);
    (st, UAC_SA_PER_S.load(Ordering::Acquire), rms)
}

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

/// How long the reader tolerates a silent device before declaring the
/// session dead.
///
/// `uac_host_device_read` returning `ESP_ERR_TIMEOUT` is routine — the
/// driver's ring buffer is momentarily empty. What is not routine is
/// every read timing out forever, which is what a
/// `USB_TRANSFER_STATUS_OVERFLOW` leaves behind: the isochronous
/// stream stops, the device stays enumerated, and no error is ever
/// returned to us. Measured 2026-08-23 — 100 s of clean capture, one
/// overflow, then silence until the cable was pulled two minutes
/// later. Three seconds is ~150 missed frames; nothing healthy is
/// quiet that long at 48 kHz.
const STALL_TIMEOUT_MS: u64 = 3_000;

/// Pause between a failed session and the re-open attempt.
const REOPEN_DELAY_MS: u64 = 500;

/// Re-opens allowed without a second of successful streaming in
/// between. Generous, because the case worth surviving is a device
/// that works for minutes and hiccups; the cap only exists to stop a
/// wedged device from spinning open/fail forever.
const REOPEN_MAX_ATTEMPTS: u32 = 30;

/// Consecutive re-opens since audio last flowed. Reset by the 1 Hz
/// tick whenever bytes actually moved.
static REOPEN_ATTEMPTS: AtomicU32 = AtomicU32::new(0);

/// RAII guard that releases [`READER_ACTIVE`] on drop. Held by
/// `reader_thread` for its entire lifetime so the gate gets reset
/// even on panic — without the guard a panic in the read /
/// resample / push chain would leave the gate stuck `true` and
/// every subsequent `RxConnected` would be ignored until reboot
/// (Gemini PR #98 r4 review).
struct ReaderActiveGuard {
    /// `Some((addr, iface))` when the session ended in a way a fresh
    /// one might survive — a stall or a terminal read error, as
    /// opposed to the device being unplugged. `None` on a disconnect:
    /// the driver fires `RxConnected` by itself when it comes back.
    reopen: Option<(u8, u8)>,
}

impl Drop for ReaderActiveGuard {
    fn drop(&mut self) {
        // Release the gate *first*. `app_task` dedups `RxConnected`
        // against it, so a re-open posted while it is still held gets
        // dropped as a duplicate and the radio never comes back.
        READER_ACTIVE.store(false, std::sync::atomic::Ordering::Release);

        let Some((addr, iface_num)) = self.reopen else {
            return;
        };

        let attempt = REOPEN_ATTEMPTS.fetch_add(1, Ordering::AcqRel) + 1;
        if attempt > REOPEN_MAX_ATTEMPTS {
            log::error!(
                "uac: {attempt} re-opens with no audio in between — giving up until the device \
                 is re-attached"
            );
            set_state(UacState::Error);
            return;
        }

        // Let the driver settle before asking for the interface again;
        // the stop/close in the cleanup above has to land first.
        std::thread::sleep(std::time::Duration::from_millis(REOPEN_DELAY_MS));

        match EVENT_SENDER.get() {
            Some(tx) => {
                log::warn!(
                    "uac: re-opening addr={addr} iface={iface_num} (attempt {attempt}/{REOPEN_MAX_ATTEMPTS})"
                );
                if tx
                    .send(DriverEvent::RxConnected { addr, iface_num })
                    .is_err()
                {
                    log::error!("uac: re-open send failed — app_task is gone");
                }
            }
            None => log::error!("uac: re-open impossible — EVENT_SENDER not initialised"),
        }
    }
}

/// Set by [`device_event_cb`] when the IDF driver fires
/// `DRIVER_EVENT_DISCONNECTED` (USB cable unplug, IC-705 power off,
/// VBUS sag). The reader thread polls this at the top of every loop
/// iteration and exits cleanly — disconnect latency = at most one
/// `READER_READ_TIMEOUT_MS` instead of waiting for the next
/// `device_read` to fail. Also lets the reader's cleanup path skip
/// `device_stop` / `device_close` (the IDF driver already
/// invalidated the handle when DISCONNECTED fired) so the post-
/// disconnect cleanup doesn't log spurious `INVALID_ARG` errors.
///
/// Reset by [`handle_rx_connected`] at the start of a new session,
/// before `uac_host_device_open` registers the device callback, so
/// a stale `true` from a previous attach can't kill the freshly-
/// spawned reader on its first iteration and a fresh DISCONNECT
/// firing during open isn't silently dropped by the reset.
static READER_STOP_REQUESTED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Receives freshly-resampled 12 kHz mono audio from `reader_thread`,
/// one `uac_host_device_read` batch's worth at a time (length varies —
/// not chunked to any fixed size; implementations that need fixed-size
/// chunks buffer internally, same as the pre-abstraction reader body
/// did inline). `uac`'s own host/driver/hot-plug machinery is
/// otherwise consumer-agnostic; this is the one point where FT8's
/// chunk-queue pipeline ([`Ft8ChunkSink`]) and WSPR's DDC push
/// (`wspr_app`'s own sink, in `src/bin/wspr_app.rs`) diverge.
///
/// Added 2026-08-15 wiring real UAC audio into `wspr_app` — before
/// this, `reader_thread` pushed straight into an FT8-specific
/// `QueueHandle_t` (`CHUNK_Q_ADDR`/`set_chunk_q`), which is exactly
/// the coupling this trait removes.
pub trait AudioSink: Send + 'static {
    fn push_samples(&mut self, samples_12k_mono: &[i16]);
}

/// Registered sink slot. `None` until [`set_audio_sink`] runs — the
/// reader thread just drops samples until then, same "wired late,
/// drop until ready" contract the old `CHUNK_Q_ADDR` had.
static AUDIO_SINK: Mutex<Option<Box<dyn AudioSink>>> = Mutex::new(None);

// ── Cold-acquisition capture ring (#356b) ────────────────────────────
//
// `decode_pipeline` arms this when the FT8 grid is lost past what the
// ±1 s coarse search can recover (no clock, no decodes, no
// `bootstrap_dt_med` for a run of slots). While armed, `Ft8ChunkSink`
// appends raw 12 kHz audio here; once it holds
// `ft8::acquire::REQUIRED_SAMPLES`, `decode_pipeline` runs the tiled
// acquisition on it and disarms.

/// `ft8::acquire::REQUIRED_SAMPLES` (25 s) plus one chunk of slack, so
/// the last `extend_from_slice` never undershoots. ~600 KB on PSRAM
/// while armed, freed the moment `decode_pipeline` takes it.
const ACQUIRE_RING_CAP: usize = mfsk_core::ft8::acquire::REQUIRED_SAMPLES + CHUNK_LEN;

static ACQUIRE_ARMED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static ACQUIRE_RING: Mutex<Vec<i16>> = Mutex::new(Vec::new());

/// Start filling the acquisition ring from scratch.
pub fn arm_acquisition() {
    if let Ok(mut r) = ACQUIRE_RING.lock() {
        r.clear();
        r.reserve(ACQUIRE_RING_CAP);
    }
    ACQUIRE_ARMED.store(true, Ordering::Release);
}

/// Stop filling and drop the buffer.
pub fn disarm_acquisition() {
    ACQUIRE_ARMED.store(false, Ordering::Release);
    if let Ok(mut r) = ACQUIRE_RING.lock() {
        *r = Vec::new();
    }
}

/// Take the captured audio once the ring holds at least `min_samples`,
/// leaving the ring empty; `None` while it is still filling. Also
/// disarms — one acquisition per arm.
pub fn take_acquisition_audio(min_samples: usize) -> Option<Vec<i16>> {
    let mut r = ACQUIRE_RING.lock().ok()?;
    if r.len() < min_samples {
        return None;
    }
    ACQUIRE_ARMED.store(false, Ordering::Release);
    Some(core::mem::take(&mut r))
}

// ── `MFSK_CORES3_SIM` — a radio, faked ─────────────────────────────
//
// Feeds a baked FT8 slot through the *real* `Ft8ChunkSink` at
// real-time pace, so every alignment state machine (UTC anchor,
// air-sync, cold acquisition, the capture ring, the NVS grid fix)
// runs exactly as it would with an IC-705 — but on a board that,
// flashed over USB, stays a peripheral and keeps its console. The
// only things this cannot show are real band density and a real
// off-air clock spread.

/// Feed `wav` (a 44-byte-header 12 kHz mono PCM slot) into the
/// registered [`AudioSink`] forever, preceded by `lead_silence`
/// samples so the sink's slot grid starts `lead_silence / 12` ms
/// mis-aligned from the signal — the condition the grid code exists
/// to recover from.
pub fn spawn_sim_feed(wav: &'static [u8], lead_silence: usize) {
    struct Cfg {
        wav: &'static [u8],
        lead: usize,
    }
    let cfg = Box::into_raw(Box::new(Cfg { wav, lead: lead_silence })) as *mut core::ffi::c_void;

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `spawn_sim_feed` leaked exactly this box. Drop it once
        // its two fields are copied into locals — the task never
        // returns, so nothing else needs it.
        let (wav, lead) = {
            let cfg = unsafe { Box::from_raw(arg as *mut Cfg) };
            (cfg.wav, cfg.lead)
        };
        let pcm: Vec<i16> = wav[44..]
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        // Loop a *whole number of slots*, not the raw file length.
        // `qso3_busy.wav` is 180 101 samples — 101 past one 15 s slot —
        // so wrapping at `pcm.len()` slid the content 101 samples
        // (8.4 ms) under the slot grid every loop, and spliced a
        // discontinuity into whichever frame straddled the seam. That
        // is a harness artifact, and it is not a small one: with the
        // grid held perfectly still (zero shifts applied) the measured
        // median DT still crept +8.2 ms/slot, against 8.42 ms/slot
        // predicted from the 101 samples — the receiver was being
        // blamed for the test rig's own drift, and `dec` wobbled 4-7 as
        // the window crossed the seam. Truncating to whole slots makes
        // the loop phase-continuous, so identical audio really does
        // reach the decoder identically every slot.
        let loop_len = if pcm.len() >= SLOT_SAMPLES_12K {
            pcm.len() / SLOT_SAMPLES_12K * SLOT_SAMPLES_12K
        } else {
            pcm.len()
        };
        log::warn!(
            "uac SIM: feeding {loop_len} of {} baked samples on loop ({} slot(s), {} trimmed for phase continuity), {} ms lead silence — no radio",
            pcm.len(),
            loop_len / SLOT_SAMPLES_12K.max(1),
            pcm.len() - loop_len,
            lead / 12
        );
        const BLK: usize = 256;
        let t0 = unsafe { sys::esp_timer_get_time() };
        let mut fed: u64 = 0;
        let mut src = 0usize; // index into pcm, after the lead is done
        let mut lead_left = lead;
        let silence = [0i16; BLK];
        loop {
            let block: &[i16] = if lead_left >= BLK {
                lead_left -= BLK;
                &silence
            } else if lead_left > 0 {
                let n = lead_left;
                lead_left = 0;
                &silence[..n]
            } else {
                let end = (src + BLK).min(loop_len);
                let s = &pcm[src..end];
                src = if end == loop_len { 0 } else { end };
                s
            };
            if let Ok(mut g) = AUDIO_SINK.lock() {
                if let Some(sink) = g.as_mut() {
                    sink.push_samples(block);
                }
            }
            fed += block.len() as u64;
            let due = (fed * 1_000_000 / 12_000) as i64;
            let now = unsafe { sys::esp_timer_get_time() } - t0;
            if due > now {
                unsafe {
                    sys::vTaskDelay(
                        (((due - now) / 1_000).max(1) as u32)
                            / (1_000 / sys::configTICK_RATE_HZ).max(1),
                    )
                };
            }
        }
    }

    let r = unsafe {
        sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"uac_sim".as_ptr(),
            4096,
            cfg,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if r != 1 {
        log::error!("uac SIM: feed task spawn failed");
    }
}

/// NVS partition handle for persisting the cold-acquisition grid fix
/// across the reboot into FT4 mode (#356b). Set from `main` before the
/// decode pipeline spawns.
static GRID_FIX_NVS: Mutex<Option<esp_idf_svc::nvs::EspDefaultNvsPartition>> = Mutex::new(None);

/// Hand the decode pipeline a way to persist the acquired grid phase.
pub fn set_grid_fix_nvs(part: esp_idf_svc::nvs::EspDefaultNvsPartition) {
    if let Ok(mut slot) = GRID_FIX_NVS.lock() {
        *slot = Some(part);
    }
}

/// Persist a cold-acquisition grid fix, from a short-lived
/// internal-stack task — an NVS write disables the flash cache, which
/// a PSRAM stack (or the decode task mid-flight) must not be holding.
/// One-shot, fire and forget.
pub fn persist_grid_fix(fix: mfsk_app_shared::grid_fix::GridFix) {
    let Some(part) = GRID_FIX_NVS.lock().ok().and_then(|g| g.clone()) else {
        log::warn!("uac: grid-fix NVS not wired — acquired phase will not survive a reboot");
        return;
    };

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `persist_grid_fix` leaked exactly this box.
        let boxed = unsafe {
            Box::from_raw(
                arg as *mut (
                    esp_idf_svc::nvs::EspDefaultNvsPartition,
                    mfsk_app_shared::grid_fix::GridFix,
                ),
            )
        };
        let (part, fix) = *boxed;
        match mfsk_app_shared::boot_mode::open_nvs(part) {
            Ok(mut nvs) => match mfsk_app_shared::grid_fix::save(&mut nvs, &fix) {
                Ok(()) => log::warn!(
                    "grid-fix persisted: {:+} us (R {:.2}) — will seed FT4's grid on next boot",
                    fix.offset_us,
                    fix.confidence
                ),
                Err(e) => log::error!("grid-fix save failed: {e}"),
            },
            Err(e) => log::error!("grid-fix NVS open failed: {e}"),
        }
        unsafe { sys::vTaskDelete(core::ptr::null_mut()) };
    }

    let ptr = Box::into_raw(Box::new((part, fix))) as *mut core::ffi::c_void;
    let created = unsafe {
        sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"grid_fix_save".as_ptr(),
            4096,
            ptr,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("uac: could not spawn grid-fix save task");
        // Reclaim the leaked box so it does not just leak.
        drop(unsafe {
            Box::from_raw(
                ptr as *mut (
                    esp_idf_svc::nvs::EspDefaultNvsPartition,
                    mfsk_app_shared::grid_fix::GridFix,
                ),
            )
        });
    }
}

/// Register the audio sink. Call before [`start_host`] so the sink is
/// live before the class driver can enumerate a device and spawn
/// `reader_thread` — same ordering `main.rs` already relies on for
/// [`set_chunk_q`] (spawn the pipeline / register the sink first,
/// install the UAC driver second).
pub fn set_audio_sink<S: AudioSink>(sink: S) {
    match AUDIO_SINK.lock() {
        Ok(mut slot) => {
            *slot = Some(Box::new(sink));
            log::info!("uac: audio sink registered");
        }
        Err(e) => log::error!("uac: AUDIO_SINK mutex poisoned, sink not registered: {e}"),
    }
}

/// FT8 chunk-queue sink — the pre-abstraction `reader_thread` behavior
/// verbatim: chunks resampled 12 kHz mono audio to [`CHUNK_LEN`]
/// (100 ms) blocks and emits `ChunkMsg::SlotEnd` every
/// [`SLOT_SAMPLES_12K`] (15 s, one FT8 slot), publishing the new slot
/// index via `time_sync::publish_capture_slot`.
struct Ft8ChunkSink {
    chunk_q: sys::QueueHandle_t,
    chunk: Vec<i16>,
    slot_samples: usize,
    /// Samples this slot runs before `SlotEnd` fires. Normally
    /// [`SLOT_SAMPLES_12K`]; the air-sync shift (#356) moves it by up to
    /// one slot's worth while the clock is not NTP-disciplined.
    slot_target: usize,
    wav_idx: usize,
    /// The one-time rough anchor from the system clock (RTC *or* NTP)
    /// has run. It gets the grid inside FT8's ±2.5 s and, more to the
    /// point, inside the ±1 s coarse search — so the air-sync
    /// refinement below has candidates to measure a DT from. Distinct
    /// from "the clock is trusted": a plausible RTC value anchors here,
    /// only NTP hands the phase over to the UTC drift check.
    coarse_anchored: bool,
    /// One line has been logged saying NTP disciplined the clock and
    /// air-sync stood down — not one per slot.
    utc_owns_phase_logged: bool,
}
// SAFETY: `QueueHandle_t` is a raw pointer into IDF-owned state; the
// IDF queue API is thread-safe by design (that's the whole point of a
// FreeRTOS queue), and `Ft8ChunkSink` never dereferences the pointer
// itself — every use goes through `pipeline::send_box`, which wraps
// the IDF `xQueueGenericSend`. Matches `DeviceHandle`'s own identical
// `Send` rationale above.
unsafe impl Send for Ft8ChunkSink {}

impl Ft8ChunkSink {
    fn new(chunk_q: sys::QueueHandle_t) -> Self {
        Self {
            chunk_q,
            chunk: Vec::with_capacity(CHUNK_LEN),
            slot_samples: 0,
            slot_target: SLOT_SAMPLES_12K,
            coarse_anchored: false,
            utc_owns_phase_logged: false,
            wav_idx: 0,
        }
    }
}

impl AudioSink for Ft8ChunkSink {
    fn push_samples(&mut self, samples: &[i16]) {
        // Feed the cold-acquisition ring while `decode_pipeline` has it
        // armed (#356b). Bounded — stop appending once it is full and
        // let the pipeline pick it up.
        if ACQUIRE_ARMED.load(Ordering::Acquire) {
            if let Ok(mut r) = ACQUIRE_RING.lock() {
                if r.len() < ACQUIRE_RING_CAP {
                    let room = ACQUIRE_RING_CAP - r.len();
                    r.extend_from_slice(&samples[..samples.len().min(room)]);
                }
            }
        }

        // One-time rough anchor from the system clock, RTC or NTP.
        //
        // Without any anchor the boundary is whatever 15 s window the
        // reader started in, and FT8's coarse sync only searches ±2.5 s
        // of it — so a real signal is outside the window about 2/3 of
        // the time and the receiver looks broken for a reason that has
        // nothing to do with the audio (#313/#163). Costs no samples:
        // the *current* partial slot is simply given the right length,
        // so the next boundary lands on the grid.
        //
        // A merely-plausible RTC value is enough here — the point is to
        // get inside the ±1 s coarse search so the air-sync refinement
        // (#356) has a DT to work with. Only NTP hands the phase to the
        // UTC drift check below.
        if !self.coarse_anchored {
            if let Some(remain) = mfsk_app_shared::time_sync::samples_to_next_slot_12k(SLOT_SECS) {
                self.slot_samples = SLOT_SAMPLES_12K.saturating_sub(remain);
                self.coarse_anchored = true;
                // Grid lock state (#356b): a plausible clock, disciplined
                // or not. `decode_pipeline`'s air-sync raises this to
                // `Air` once it locks; only NTP raises it to `Ntp`.
                mfsk_app_shared::time_sync::note_grid_lock(
                    if mfsk_app_shared::time_sync::clock_is_disciplined() {
                        mfsk_app_shared::time_sync::GridLock::Ntp
                    } else {
                        mfsk_app_shared::time_sync::GridLock::Rtc
                    },
                );
                log::info!(
                    "uac: slot grid coarse-anchored to the system clock — {} ms to the next boundary",
                    remain / 12,
                );
            }
        }
        for &s in samples {
            self.chunk.push(s);
            if self.chunk.len() >= CHUNK_LEN {
                let to_send = core::mem::replace(&mut self.chunk, Vec::with_capacity(CHUNK_LEN));
                send_box(self.chunk_q, Box::new(ChunkMsg::Samples(to_send)));
                self.slot_samples += CHUNK_LEN;
                if self.slot_samples >= self.slot_target {
                    send_box(
                        self.chunk_q,
                        Box::new(ChunkMsg::SlotEnd {
                            wav_idx: self.wav_idx,
                            total_samples: self.slot_samples,
                        }),
                    );
                    self.wav_idx = self.wav_idx.wrapping_add(1);
                    self.slot_samples = 0;
                    // Next slot is the nominal length unless the air-sync
                    // shift below moves it.
                    self.slot_target = SLOT_SAMPLES_12K;
                    // Issue #110: publish capture-slot boundary for
                    // any TX scheduler running in this BootMode.
                    // BootMode::Uac is currently RX-only on the S3
                    // board, but published unconditionally to keep
                    // the slot index live in case downstream
                    // logging / TX wiring is added.
                    let now_us = unsafe { sys::esp_timer_get_time() };
                    mfsk_app_shared::time_sync::publish_capture_slot(self.wav_idx as u32, now_us);

                    // One phase authority per boundary, never both.
                    // Once NTP has disciplined the clock it is trusted
                    // absolutely; before then — or forever, off-grid —
                    // the grid rides coarse sync's own DT through
                    // `decode_pipeline`'s air-sync (#356).
                    if mfsk_app_shared::time_sync::clock_is_disciplined() {
                        mfsk_app_shared::time_sync::note_grid_lock(
                            mfsk_app_shared::time_sync::GridLock::Ntp,
                        );
                        if !self.utc_owns_phase_logged {
                            self.utc_owns_phase_logged = true;
                            log::info!(
                                "uac: NTP-disciplined — UTC owns the slot phase, air-sync stood down"
                            );
                        }
                        // Drain the air-sync hint so a later loss of NTP
                        // does not re-apply something stale.
                        let _ = mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                        // Re-anchor only when the phase error is a
                        // fraction of what FT8 can search — corrects the
                        // NTP step and long-term drift, not jitter.
                        if let Some(remain) =
                            mfsk_app_shared::time_sync::samples_to_next_slot_12k(SLOT_SECS)
                        {
                            let err_ms = if remain > SLOT_SAMPLES_12K / 2 {
                                -(((SLOT_SAMPLES_12K - remain) / 12) as i32)
                            } else {
                                (remain / 12) as i32
                            };
                            if err_ms.unsigned_abs() > SLOT_DRIFT_REANCHOR_MS {
                                log::warn!("uac: slot phase {err_ms:+} ms off UTC — re-anchoring");
                                self.slot_samples = SLOT_SAMPLES_12K.saturating_sub(remain);
                            }
                        }
                    } else {
                        let acq = mfsk_app_shared::time_sync::take_acquisition_shift_12k();
                        if acq != 0 {
                            // Cold-acquisition one-shot (#356b): the grid
                            // was lost past ±1 s and `decode_pipeline`
                            // recovered the phase from a 25 s FT8
                            // capture. Uncapped — lengthen this one slot
                            // by up to a whole period, then straight back
                            // to nominal and hand to the narrow tracking.
                            self.slot_target =
                                (SLOT_SAMPLES_12K as i32 + acq).clamp(30_000, 300_000) as usize;
                            self.coarse_anchored = true;
                            mfsk_app_shared::time_sync::note_grid_lock(
                                mfsk_app_shared::time_sync::GridLock::Air,
                            );
                            log::info!(
                                "uac: cold-acquisition slot shift {acq:+} — next slot {} samples",
                                self.slot_target,
                            );
                        } else {
                            // Air-sync shift from `decode_pipeline`
                            // (#356): the DT of coarse sync's own
                            // candidates. Sign is WSJT-X's — DT > 0 means
                            // the slot opened early, lengthen the next.
                            // Capped ±2400 by the producer, so the slot
                            // stays in [177_600, 182_400] and stage1_inc
                            // still completes.
                            let air_shift =
                                mfsk_app_shared::time_sync::take_bootstrap_slot_shift_12k();
                            if air_shift != 0 {
                                self.slot_target = (SLOT_SAMPLES_12K as i32 + air_shift)
                                    .clamp(60_000, 200_000)
                                    as usize;
                                log::info!(
                                    "uac: air-sync slot shift {air_shift:+} — next slot {} samples",
                                    self.slot_target,
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Wire the FT8 decode pipeline's chunk queue as the audio sink.
/// Called once from `decode_pipeline::run_with_source`'s source-spawn
/// closure in the pipeline thread, before the decode loop blocks on
/// `recv_box`. Thin wrapper around [`set_audio_sink`] kept under its
/// original name so `main.rs`/`decode_pipeline.rs` need no changes.
pub fn set_chunk_q(q: sys::QueueHandle_t) {
    set_audio_sink(Ft8ChunkSink::new(q));
    log::info!("uac: chunk_q wired (addr={:#x})", q as usize);
}

/// `SlotEnd` cadence in 12 kHz mono samples. Same as `wav_sim`'s
/// `SLOT_SAMPLES` — 180_000 = 15 s @ 12 kHz, one FT8 slot. UAC streams
/// continuously, so the reader synthesises the boundary from the
/// post-resample sample count.
///
/// **Anchored to UTC since 2026-08-22** (#313 open item 1). The count
/// still sets the slot's *length*; `time_sync::samples_to_next_slot_12k`
/// sets its *phase*, once at first sight of a real clock and again
/// whenever the two drift more than [`SLOT_DRIFT_REANCHOR_MS`] apart.
/// Without NTP there is no phase source and the boundary falls back to
/// stream-relative — the sink says which of the two it is doing rather
/// than leaving a reader to guess.
const SLOT_SAMPLES_12K: usize = 180_000;
/// The same slot, in seconds — what the UTC grid is computed from.
const SLOT_SECS: u64 = 15;
/// Phase error that triggers a re-anchor rather than a note. 250 ms is
/// a tenth of the ±2.5 s FT8 searches, so this corrects long-term
/// drift (and the NTP step) without chasing jitter.
const SLOT_DRIFT_REANCHOR_MS: u32 = 250;

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
    DRIVER_EVENTS.fetch_add(1, Ordering::Relaxed);
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
/// `TRANSFER_ERROR` / `DRIVER_EVENT_DISCONNECTED`. The reader thread
/// polls `uac_host_device_read` rather than waiting on the callback,
/// so the first three are logged and nothing else; `DISCONNECTED`
/// sets [`READER_STOP_REQUESTED`], which is how the reader learns to
/// stop and to skip the cleanup the IDF driver has already done.
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
    // released the underlying state.
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
/// The handle moves into the reader thread; a disconnect reaches it
/// through [`READER_STOP_REQUESTED`], not through the handle.
fn handle_rx_connected(addr: u8, iface_num: u8) -> Result<()> {
    log::info!("uac: opening device addr={addr} iface={iface_num}");
    // Clear any stale stop-request BEFORE `uac_host_device_open` —
    // that call registers `device_event_cb` for the new handle, after
    // which a fast DISCONNECT (e.g. cable yanked mid-bringup) would
    // race against the reset and have its `true` silently dropped.
    // Resetting now is safe because (1) `app_task` processes
    // `DriverEvent`s sequentially on the mpsc channel, so the previous
    // session's DISCONNECTED callback finished firing before this
    // RxConnected was dispatched, and (2) the `READER_ACTIVE` gate
    // guarantees the previous `reader_thread` has fully exited before
    // we re-enter `handle_rx_connected` (Gemini PR #107 review —
    // the older "callback unregistered at device_close" rationale was
    // incorrect since the disconnect cleanup path explicitly skips
    // `device_close`).
    READER_STOP_REQUESTED.store(false, Ordering::Release);
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

    let handle_wrapped = DeviceHandle(handle);
    if let Err(e) = spawn_psram_thread(c"uac_reader", READER_TASK_STACK, move || {
        reader_thread(handle_wrapped, addr, iface_num)
    }) {
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
    set_state(UacState::Streaming);
    Ok(())
}

/// Phase T0 (TX/QSO feasibility, not yet wired to anything that keys
/// PTT): compile-time opt-in probe that the IC-705's USB audio OUT
/// interface — `DriverEvent::TxConnected`, enumerated and logged since
/// #163 but never opened — actually accepts `uac_host_device_open` /
/// `_start` / `_write` the same way `handle_rx_connected` does for the
/// IN side. Off by default; every other `TxConnected` build keeps
/// today's behaviour (log and ignore).
///
/// **Writes digital silence only, never a tone.** The goal here is
/// proving the write path, not proving the radio can be driven — a
/// nonzero signal into the IC-705's USB MOD input could key TX by
/// itself if the radio's PTT source is set to VOX, and this file has
/// no way to know that setting from software. True zero samples carry
/// no audio level to VOX-detect, so they're the safe way to exercise
/// `uac_host_device_write` before anything here can assert PTT on
/// purpose.
///
/// Before running this against a real radio: confirm the IC-705's
/// `PTT SOURCE` menu is anything but `VOX`.
const TX_PROBE_ENABLED: bool = option_env!("MFSK_CORES3_TX_PROBE").is_some();

/// One silent write attempt, this many times, before closing again.
/// Enough to see whether the ring buffer keeps accepting writes past
/// the first one (an OUT endpoint that stalls after N packets would
/// look identical to success on a single write).
const TX_PROBE_WRITES: u32 = 20;
/// One video frame's worth at 48 kHz stereo 16-bit — matches
/// `STREAM_BUFFER_THRESHOLD`'s sizing logic, scaled down since this
/// only needs to exercise the path, not sustain real-time throughput.
const TX_PROBE_CHUNK_BYTES: usize = 1920; // 10 ms @ 48k stereo 16b
const TX_PROBE_WRITE_TIMEOUT_MS: u32 = 100;

fn handle_tx_connected(addr: u8, iface_num: u8) {
    log::info!("uac: TX_CONNECTED addr={addr} iface={iface_num} — probe start (silence only)");
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
        log::error!("uac: tx probe device_open failed err={err:#x}");
        return;
    }

    // Same fixed config the IN side uses. Unverified for the OUT
    // interface specifically — the IC-705's TX descriptor has never
    // been queried, so a `device_start` failure here is expected
    // information, not a bug: check the device descriptor dump in the
    // UDP log (same as the RX open-failure comment above) and correct
    // this constant from what the radio actually reports.
    let stream_config = sys::uac::uac_host_stream_config_t {
        channels: STREAM_CHANNELS,
        bit_resolution: STREAM_BIT_RESOLUTION,
        sample_freq: STREAM_SAMPLE_FREQ_HZ,
        flags: 0,
    };
    let err = unsafe { sys::uac::uac_host_device_start(handle, &stream_config as *const _) };
    if err != sys::ESP_OK as sys::esp_err_t {
        log::error!(
            "uac: tx probe device_start failed err={err:#x} (tried {STREAM_CHANNELS}ch / \
             {STREAM_BIT_RESOLUTION}b / {STREAM_SAMPLE_FREQ_HZ}Hz — this is a guess, not read \
             from the OUT interface's own descriptor)"
        );
        let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: tx probe device_close after start failure failed err={close_err:#x}");
        }
        return;
    }

    let silence = [0u8; TX_PROBE_CHUNK_BYTES];
    let mut wrote_ok = 0u32;
    for i in 0..TX_PROBE_WRITES {
        let mut buf = silence;
        let err = unsafe {
            sys::uac::uac_host_device_write(
                handle,
                buf.as_mut_ptr(),
                buf.len() as u32,
                TX_PROBE_WRITE_TIMEOUT_MS,
            )
        };
        if err == sys::ESP_OK as sys::esp_err_t {
            wrote_ok += 1;
        } else {
            log::warn!("uac: tx probe write {i}/{TX_PROBE_WRITES} failed err={err:#x}");
        }
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(10);
    }
    log::info!("uac: tx probe wrote {wrote_ok}/{TX_PROBE_WRITES} silent chunks OK");

    let stop_err = unsafe { sys::uac::uac_host_device_stop(handle) };
    let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
    if stop_err != sys::ESP_OK as sys::esp_err_t {
        log::error!("uac: tx probe device_stop failed err={stop_err:#x}");
    }
    if close_err != sys::ESP_OK as sys::esp_err_t {
        log::error!("uac: tx probe device_close failed err={close_err:#x}");
    }
    log::info!("uac: tx probe done, interface closed");
}

/// Reader thread body. Polls `uac_host_device_read` for raw
/// 48 kHz/stereo/16-bit iso IN packets, extracts the left channel,
/// resamples to 12 kHz mono via `LinearResamplerI16To12k`, and pushes
/// `CHUNK_LEN`-sized chunks into the decode pipeline's chunk queue
/// (with `SlotEnd` every `SLOT_SAMPLES_12K` samples).
///
/// Counts iso IN throughput in `RX_BYTES` / `RX_PACKETS` / `RX_ERRORS`
/// and logs a 1 Hz status line.
fn reader_thread(handle: DeviceHandle, addr: u8, iface_num: u8) {
    // RAII: clears READER_ACTIVE on any exit path including panic,
    // and — when `reopen` is set below — asks for a fresh session.
    let mut gate = ReaderActiveGuard { reopen: None };
    let mut buf = [0u8; READER_BUFFER_BYTES];
    let mut resampler = LinearResamplerI16To12k::new(STREAM_SAMPLE_FREQ_HZ);
    // L-channel scratch (one device_read worth of mono samples). At
    // 4 KB raw / stereo i16, max = 1024 mono samples per read.
    let mut left_scratch = [0i16; READER_BUFFER_BYTES / 4];
    // Resampled output staging. Sized for at most one read's worth
    // of input → ~256 output samples at 48k→12k (4:1). Doubled for
    // headroom against the resampler's per-call rounding.
    let mut dst_scratch = [0i16; 512];
    let mut last_log = std::time::Instant::now();
    // When audio last actually arrived — the stall watchdog's clock.
    let mut last_data = std::time::Instant::now();
    let mut last_bytes: u32 = 0;
    // Post-resample signal statistics for the 1 Hz tick — issue #163.
    //
    // The byte counter below says the *transport* is alive. It cannot
    // distinguish a radio streaming real audio from one streaming
    // 192 kB/s of digital silence, which is what a muted source, a
    // wrong input selection or an unconfigured codec all look like.
    // A live bring-up session is expensive enough that "some bytes
    // arrived" is not a result worth coming away with, so measure the
    // thing the decoder actually consumes: how many 12 kHz samples per
    // second (a rate check — it should be 12 000), and how loud they
    // are.
    let mut out_samples: u32 = 0;
    let mut peak: i32 = 0;
    let mut sum_sq: u64 = 0;
    let mut clipped: u32 = 0;
    // Track whether we exited via DISCONNECTED so the cleanup path
    // can skip the redundant `device_stop` / `device_close` calls
    // — the IDF driver already invalidated the handle at
    // callback time, so calling them just logs spurious INVALID_ARG.
    let mut disconnect_triggered = false;
    loop {
        // Top-of-loop disconnect check. `device_event_cb` sets
        // the flag immediately on DISCONNECTED; we exit on the next
        // iteration (≤ `READER_READ_TIMEOUT_MS` latency) rather than
        // waiting for the failing read to surface.
        if READER_STOP_REQUESTED.load(Ordering::Acquire) {
            log::info!("uac: reader exiting — DISCONNECTED signaled by device_event_cb");
            // Hot-unplug: back to waiting, and the screen says so.
            set_state(UacState::Waiting);
            disconnect_triggered = true;
            break;
        }
        // 1 Hz throughput log. `Instant::now()` is the FreeRTOS tick
        // count under the hood — sub-microsecond cost.
        let now = std::time::Instant::now();
        if now.duration_since(last_log).as_secs() >= 1 {
            let bytes = RX_BYTES.load(Ordering::Relaxed);
            let packets = RX_PACKETS.load(Ordering::Relaxed);
            let errors = RX_ERRORS.load(Ordering::Relaxed);
            let bps = bytes.wrapping_sub(last_bytes);
            // A second of real audio means the last re-open worked, so
            // the budget resets. A device that hiccups every few
            // minutes then gets retried forever, which is the point;
            // the cap is only there for one that never streams at all.
            if bps > 0 {
                REOPEN_ATTEMPTS.store(0, Ordering::Relaxed);
            }
            // 48 k × stereo × 2 B = 192_000 B/s expected for a fully-
            // streaming IC-705. The throughput delta is the diagnostic
            // we care about here (anything well below ~190 kB/s
            // suggests packet drops or wrong stream config).
            let rms = if out_samples > 0 {
                ((sum_sq / out_samples as u64) as f64).sqrt()
            } else {
                0.0
            };
            // dBFS against full scale, so "is there signal" is one
            // glance rather than an i16 magnitude to interpret.
            let dbfs = if rms > 0.0 {
                20.0 * (rms / 32_768.0).log10()
            } else {
                -99.0
            };
            // Internal DRAM goes in the per-second line, not just the
            // 6 s alive tick. The board reboots about one second after
            // the stream starts, with nothing from the Rust panic hook
            // — which is what an allocation failure or a hardware
            // exception looks like, both handled in C below the log
            // path. Three attached devices now hold a 4 KB control
            // buffer each, and the decode pipeline allocates on top of
            // that, so "how much was left when it died" is the first
            // thing worth knowing. Refs #163.
            let free_internal = unsafe {
                sys::heap_caps_get_free_size(sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT)
            };
            let largest_internal = unsafe {
                sys::heap_caps_get_largest_free_block(
                    sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT,
                )
            };
            log::info!(
                "uac: rx tick: {bps} B/s (total {bytes} B / {packets} pkt / {errors} err) \
                 | audio {out_samples} sa/s (want 12000), rms {dbfs:.1} dBFS, peak {peak}, \
                 clipped {clipped} | internal={free_internal} largest={largest_internal}",
            );
            UAC_SA_PER_S.store(out_samples, Ordering::Release);
            UAC_RMS_MDB.store(
                if out_samples > 0 {
                    ((-dbfs) * 10.0).clamp(0.0, (u32::MAX - 1) as f64) as u32
                } else {
                    u32::MAX
                },
                Ordering::Release,
            );
            last_log = now;
            last_bytes = bytes;
            out_samples = 0;
            peak = 0;
            sum_sq = 0;
            clipped = 0;
        }

        // Stall watchdog.
        //
        // The timeout path below used to `continue`, which skipped the
        // log above as well — so a reader whose device had gone quiet
        // spun at 10 Hz emitting nothing at all, indistinguishable
        // from a dead thread. Both halves are fixed here: the tick
        // runs before the read so "0 B/s" is visible, and going quiet
        // for long enough now ends the session instead of hanging on
        // to it. Refs #163.
        if now.duration_since(last_data).as_millis() as u64 >= STALL_TIMEOUT_MS {
            log::error!(
                "uac: no audio for {} ms — ending session (addr={addr} iface={iface_num})",
                now.duration_since(last_data).as_millis()
            );
            set_state(UacState::Error);
            gate.reopen = Some((addr, iface_num));
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
            // ringbuf failures) are terminal — end the session and
            // hand the interface to the re-open gate below, which
            // retries it up to `REOPEN_MAX_ATTEMPTS` times.
            // `BootMode::Uac` is sticky-until-reboot, so a reader that
            // dies past that is at least observable in the next UDP
            // log tick (rx=0B/s).
            log::error!("uac: device_read err={err:#x}, ending session");
            set_state(UacState::Error);
            gate.reopen = Some((addr, iface_num));
            break;
        }
        if bytes_read > 0 {
            last_data = std::time::Instant::now();
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

        // Route through the registered `AudioSink` (FT8's chunk
        // queue, WSPR's DDC push, ...). While unregistered we just
        // drop the current read buffer (lossy by design — the race
        // window is bounded by how fast the consumer thread can spawn
        // + register, ~200 ms). Gemini PR #99 review fixed the
        // earlier "accumulates samples" wording which contradicted
        // the actual `continue`.
        let mut sink_guard = match AUDIO_SINK.lock() {
            Ok(g) => g,
            Err(e) => {
                log::error!("uac: AUDIO_SINK mutex poisoned: {e}");
                continue;
            }
        };
        let Some(sink) = sink_guard.as_mut() else {
            continue;
        };

        // Feed the resampler in a loop until the input is drained.
        // process() returns (consumed, produced); if consumed < input
        // we loop back with the unconsumed tail.
        let mut src_offset = 0usize;
        while src_offset < stereo_samples {
            let (consumed, produced) =
                resampler.process(&left_scratch[src_offset..stereo_samples], &mut dst_scratch);
            if produced > 0 {
                for &v in &dst_scratch[..produced] {
                    let a = (v as i32).abs();
                    if a > peak {
                        peak = a;
                    }
                    if a >= 32_000 {
                        clipped += 1;
                    }
                    sum_sq += (a as u64) * (a as u64);
                }
                out_samples += produced as u32;
                sink.push_samples(&dst_scratch[..produced]);
            }
            // Defensive: if process() makes zero progress (shouldn't,
            // given the input is non-empty), break to avoid an
            // infinite loop.
            if consumed == 0 && produced == 0 {
                break;
            }
            src_offset += consumed;
        }
        drop(sink_guard);
    }
    // Cleanup paths differ by exit reason:
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
    // Re-consult the atomic at cleanup time so a DISCONNECT that
    // fired between the top-of-loop check and a later break (read
    // error, chunk-push failure, slot-push failure) still routes
    // to the skip path. Without this, DISCONNECT racing a chunk
    // push would still emit the spurious INVALID_ARG logs.
    //
    // Either way `_gate: ReaderActiveGuard` drops below to release
    // `READER_ACTIVE` so the next RxConnected can re-take it.
    let disconnect_triggered =
        disconnect_triggered || READER_STOP_REQUESTED.load(Ordering::Acquire);
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
/// re-handled — telling "same device returned" from "new device" and
/// closing the previous handle is still open, with no issue of its
/// own.
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
                if TX_PROBE_ENABLED {
                    handle_tx_connected(addr, iface_num);
                } else {
                    log::info!(
                        "uac: TX_CONNECTED addr={addr} iface={iface_num} — ignored (RX only for FT8)"
                    );
                }
            }
        }
    }
    log::error!("uac: app_task exiting — driver event channel closed");
}

/// Install the USB host stack + the UAC class driver. Returns once
/// both are running in their respective background tasks; the caller
/// (main.rs UAC dispatch arm) falls through to the display loop.
/// Whether [`start_host`] installed the host driver on this boot.
///
/// `UacState::Off` cannot answer this: it means both "the host is
/// installed and idle" and "there is no host, because a PC is powering
/// the port". Those are the two states the link bar has to tell apart,
/// and confusing them is what made a charging board look like a broken
/// UAC stack.
static HOST_INSTALLED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

pub fn host_installed() -> bool {
    HOST_INSTALLED.load(core::sync::atomic::Ordering::Acquire)
}

/// What `start_host` concluded, kept for re-emission once a log sink
/// exists.
///
/// The result is printed the moment it happens, which on battery is
/// before WiFi has associated — so it goes into the fanout's staging
/// ring, and the ring is small enough that the alive tick pushes it out
/// before the UDP sink can replay it. The single most important line of
/// the boot was therefore never seen on the only channel that works in
/// host mode. Allocation-free slot, same one the diagnostic probes use.
pub static HOST_RESULT: crate::log_slot::LogSlot = crate::log_slot::LogSlot::new();

pub fn start_host() -> Result<()> {
    // Set up the driver→app channel BEFORE installing the class
    // driver — `driver_event_cb` may fire as soon as `uac_host_install`
    // returns (an IC-705 already plugged in would enumerate immediately).
    let (tx, rx) = channel::<DriverEvent>();
    EVENT_SENDER
        .set(tx)
        .map_err(|_| anyhow!("uac: EVENT_SENDER double init — start_host called twice?"))?;
    spawn_psram_thread(c"uac_app", APP_TASK_STACK, move || app_task(rx))
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

    if let Err(e) = spawn_psram_thread(c"usb_events", USB_EVENTS_TASK_STACK, usb_events_task) {
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
    set_state(UacState::Waiting);
    HOST_INSTALLED.store(true, core::sync::atomic::Ordering::Release);
    HOST_RESULT.store("host+class driver installed OK");
    spawn_device_count_probe();
    Ok(())
}

/// 何秒かおきに USB ホストライブラリが把握しているデバイス数を吐く。
///
/// 「列挙されない」には段階があり、どこで止まっているかで打ち手が
/// まったく変わる — VBUS が出ていないのか、ハブは見えているが下流が
/// 歩けていないのか、下流まで見えていて UAC ドライバが掴めていないのか。
/// `usb_host_lib_info()` の `num_devices` は列挙まで終わった台数なので、
/// IC-705 のように内蔵ハブを持つ機器なら、ハブ + CDC + オーディオで
/// 複数台に見えるのが正常。0 のまま動かないなら、そもそもポートに
/// 何も見えていないということで、UAC ドライバより手前の問題になる。
///
/// Refs #163.
fn spawn_device_count_probe() {
    let _ = crate::board::spawn_named(c"uac_probe", 3072, || {
        let mut last: i32 = -1;
        let mut since_log = u32::MAX;
        loop {
            let mut info = sys::usb_host_lib_info_t::default();
            // SAFETY: ホストライブラリ導入後にのみ呼ばれる。
            let err = unsafe { sys::usb_host_lib_info(&mut info) };
            if err == sys::ESP_OK {
                DEVICE_COUNT.store(info.num_devices, Ordering::Relaxed);
                CLIENT_COUNT.store(info.num_clients, Ordering::Relaxed);
                // 変化時は即、そうでなくても 10 秒おきに。LCD の
                // ログパネルは数行しか出ないので、変化時だけだと
                // 起動直後の 1 行が流れて消えて読めない。
                if info.num_devices != last || since_log >= 5 {
                    log::info!(
                        "uac: usb_host_lib_info — num_devices={} num_clients={}",
                        info.num_devices,
                        info.num_clients
                    );
                    last = info.num_devices;
                    since_log = 0;
                } else {
                    since_log += 1;
                }
            } else {
                log::warn!("uac: usb_host_lib_info failed (err={err:#x})");
            }
            std::thread::sleep(std::time::Duration::from_secs(2));
        }
    });
}

/// ホストライブラリが把握している列挙済みデバイス数。probe 未起動なら `-1`。
static DEVICE_COUNT: AtomicI32 = AtomicI32::new(-1);
/// 登録済みクライアント数 (UAC ドライバが 1 つ登録する)。
static CLIENT_COUNT: AtomicI32 = AtomicI32::new(-1);
/// クラスドライバのイベント通知回数。0 のままなら UAC ドライバは
/// 一度も呼ばれていない。
static DRIVER_EVENTS: AtomicU32 = AtomicU32::new(0);
/// 直近のエラーコード (0 = なし)。
static LAST_ERR: AtomicU32 = AtomicU32::new(0);

/// 画面の USB パネルに出す一式: (デバイス数, クライアント数,
/// ドライバイベント数, 直近エラー)。
pub fn usb_counters() -> (i32, i32, u32, u32) {
    (
        DEVICE_COUNT.load(Ordering::Relaxed),
        CLIENT_COUNT.load(Ordering::Relaxed),
        DRIVER_EVENTS.load(Ordering::Relaxed),
        LAST_ERR.load(Ordering::Relaxed),
    )
}

/// Record the last USB host error for the link bar.
///
/// **Nothing calls this**, which means `LAST_ERR` is always 0 and the
/// link bar's error field has never shown anything. Surfaced by the
/// 2026-08-30 lib split — in a bin crate a `pub(crate)` item that is
/// never used is not flagged. Kept rather than deleted because the
/// counter it feeds *is* displayed, so the gap is a missing call site
/// somewhere in the host-event path, not a redundant setter.
#[allow(dead_code)]
pub(crate) fn note_err(err: u32) {
    LAST_ERR.store(err, Ordering::Relaxed);
}

/// The state of the USB and network links, as the shared
/// [`link_bar`](mfsk_app_shared::ui::link_bar) draws it.
///
/// Built here rather than in each receiver's render loop: all three
/// need it, the mapping from `UacState` to what an operator should see
/// is a judgement (`Off` means two different things depending on
/// whether a host was ever installed), and three copies of a judgement
/// drift.
pub fn link_info() -> mfsk_app_shared::ui::link_bar::LinkInfo {
    use mfsk_app_shared::ui::link_bar::{LinkInfo, UsbLink};

    let (state, _sa, _rms) = status();
    let (devices, _clients, _events, _err) = usb_counters();
    let (tried_vbus, ..) = crate::pmic::power_state();
    // The role comes from what the firmware *decided*, which is exactly
    // "did it enable VBUS", not from whether the driver came up. A
    // board that chose host mode and has no host is a fault, and has to
    // read as one.
    let usb = if !host_installed() {
        if tried_vbus {
            UsbLink::NoHost
        } else {
            UsbLink::Peripheral
        }
    } else {
        match state {
            UacState::Streaming => UsbLink::Streaming,
            UacState::Error => UsbLink::Error,
            UacState::Off | UacState::Waiting => UsbLink::Waiting,
        }
    };
    let (tried, p0, p1, _st1) = crate::pmic::power_state();
    LinkInfo {
        usb,
        devices: devices.clamp(0, 9) as u8,
        wifi_rssi: mfsk_app_shared::wifi::rssi_cached(),
        expander_ok: crate::pmic::expander_ok(),
        battery_mv: crate::pmic::battery_mv_cached(),
        vbus_mv: crate::pmic::vbus_mv_cached(),
        clock_set: mfsk_app_shared::time_sync::utc_now_ms().is_some(),
        grid: mfsk_app_shared::time_sync::grid_lock(),
        vbus: tried.then(|| {
            (
                p1 & crate::board::AW9523_P1_BOOST_EN != 0,
                p0 & crate::board::AW9523_P0_USB_OTG_EN != 0,
                p0 & crate::board::AW9523_P0_BUS_OUT_EN != 0,
            )
        }),
    }
}

/// Bring the USB host up, once the things that make its failure
/// legible are in place.
///
/// **One sequence, for all three receivers.** `start_host` was always
/// shared; the steps around it were not, and the difference was not
/// deliberate — the FT8 controller waited for a log sink and installed
/// `esp_log_bridge` first, WSPR and FST4 called `start_host` straight
/// after enabling VBUS. So the two receivers that most needed the
/// enumeration trace could not produce one: `ENUM` is a C-side tag, and
/// without the bridge it goes to a console that host mode is about to
/// take away. A board that would not enumerate had, in those modes, no
/// way to say why.
///
/// **What is not here any more: a serviceability delay.** The FT8 path
/// slept `USB_HOST_DELAY_MS` (6 s) before installing, on the reasoning
/// that this was the last window in which a flasher could reach the
/// port. That was true when the firmware could take host mode with a
/// PC attached. Since #163 it decides from VBUS and only becomes a host
/// when nothing is powering the port — which means there is no PC, and
/// no flasher to wait for. Six seconds of nothing, on every boot,
/// guarding against a case that can no longer happen.
///
/// The wait that remains is for the UDP log sink, and it is bounded:
/// a receiver with no WiFi still has to start.
/// How long to wait for the UDP log sink before installing the host
/// anyway. A receiver with no WiFi still has to start.
const LOG_SINK_WAIT_MS: u32 = 45_000;

pub fn start_host_when_ready() {
    log::info!("uac: installing USB host — the serial console goes away when it returns");

    // Hold until the log sink exists, if it is coming.
    //
    // Everything interesting about enumeration is logged in the
    // few hundred milliseconds after `start_host()`, and on
    // battery there is no serial console to catch it — the VBUS
    // gate means a board in host mode is a board with no cable to
    // a PC. WiFi takes ~30 s, so without this wait those lines land
    // in the staging ring and are gone by the time anything can
    // read them. Bounded, because a receiver with no WiFi still has
    // to work.
    // Only worth waiting if a sink is actually coming. UAC mode
    // now leaves WiFi off by default, and waiting 45 s for a sink
    // that will never exist is just a receiver that takes 45 s
    // longer to start. Refs #163.
    let sink_expected = crate::wifi_enabled_for_this_boot();
    let mut waited_ms = 0u32;
    while sink_expected && waited_ms < LOG_SINK_WAIT_MS {
        if crate::FANOUT
            .udp
            .try_lock()
            .map(|g| g.is_some())
            .unwrap_or(false)
        {
            log::info!("uac: log sink up after {waited_ms} ms — installing USB host now");
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(250));
        waited_ms += 250;
    }
    if sink_expected && waited_ms >= LOG_SINK_WAIT_MS {
        log::warn!(
            "uac: no log sink after {waited_ms} ms — installing USB host anyway; \
             enumeration will only be visible on screen"
        );
    }

    // Turn up ESP-IDF's own USB enumeration logging before the
    // stack starts.
    //
    // The UAC class driver only reports what it recognises, so a
    // device that never finishes enumeration — or a hub whose
    // downstream ports are never walked — is indistinguishable
    // from nothing being plugged in. These tags are the ones the
    // host library uses on that path, and they are quiet outside
    // attach/detach, so leaving them up costs nothing while the
    // board waits. Issue #163.
    // `ENUM` alone at DEBUG. It carries the per-stage verdicts —
    // GET_FULL_DEV_DESC, CHECK_SHORT_CONFIG_DESC, and which one
    // FAILED — which is the whole diagnostic.
    //
    // The rest stay at INFO deliberately. At DEBUG, `EXT_PORT` /
    // `USBH` / `EXT_HUB` emit a "Processing actions" line per state
    // transition, so one attach is several hundred lines inside a
    // few milliseconds. Every one of those becomes a UDP datagram
    // sent from the USB task, and the board reliably fell off the
    // network right after enumeration whenever they were on — the
    // log volume was costing us the log. Refs #163.
    unsafe {
        esp_idf_svc::sys::esp_log_level_set(
            c"ENUM".as_ptr(),
            esp_idf_svc::sys::esp_log_level_t_ESP_LOG_DEBUG,
        );
    }
    for tag in [
        c"USB HOST".as_ptr(),
        c"USBH".as_ptr(),
        c"HUB".as_ptr(),
        c"EXT_HUB".as_ptr(),
        c"EXT_PORT".as_ptr(),
    ] {
        unsafe {
            esp_idf_svc::sys::esp_log_level_set(
                tag,
                esp_idf_svc::sys::esp_log_level_t_ESP_LOG_INFO,
            );
        }
    }

    crate::log_free_internal("pre-uac-host-install");
    // Serial is about to go away with the PHY; move the C-side log
    // output somewhere that survives (see `esp_log_bridge`).
    crate::esp_log_bridge::install();

    if let Err(e) = start_host() {
        log::error!("UAC host start failed: {e:#}");
        let mut msg: heapless::String<96> = heapless::String::new();
        {
            use core::fmt::Write as _;
            let _ = write!(&mut msg, "start_host FAILED: {e:#}");
        }
        HOST_RESULT.store(msg.as_str());
    }
    crate::log_free_internal("post-uac-host-install");
}
