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
/// (`BASEBAND_BUFS`/`DDC_READY_IDX`/`ctx.nvs`), so the next
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
pub(crate) fn spawn_psram_thread<F>(
    name: &'static core::ffi::CStr,
    stack_size: usize,
    priority: Option<u8>,
    pin_to_core: Option<esp_idf_svc::hal::cpu::Core>,
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
    let default_cfg_for_prio = ThreadSpawnConfiguration::default();
    let psram_cfg = ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        stack_alloc_caps: MallocCap::Spiram | MallocCap::Cap8bit,
        priority: priority.unwrap_or(default_cfg_for_prio.priority),
        pin_to_core: pin_to_core.or(default_cfg_for_prio.pin_to_core),
        ..default_cfg_for_prio
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
/// **The same as [`AUDIO_TASK_PRIORITY`] and for the same reason.**
///
/// This was 5 while the reader was raised to 6 on 2026-09-19, which
/// raised one half of the audio path and left the half feeding it
/// sharing core 0 with the decode thread at equal priority — 100 Hz
/// round-robin, so the class driver got about half a core for as long
/// as a decode ran. That is visible in the honest priority argument the
/// reader's constant already makes: a 16 KB ring is 85 ms of audio, and
/// nothing in the path may stall longer than that.
///
/// **8 since 2026-09-22, above the panel's 7.** `display::PANEL_PRIORITY`
/// went to 7 on 2026-09-21 so the screen would not stop for a decode,
/// which put it above this task on the same core; that was checked on
/// the SIM, where a starved feeder only lags. On a radio a class driver
/// that cannot resubmit its isochronous URBs within their 48 ms loses
/// the frames, uncounted. Measured on air the same day
/// (`logs/live_ft8_fea56c84_2026-09-22.log`): reader gaps up to 102 ms
/// against an 85 ms ring, and slot-grid steps of +350 and +380 samples
/// — audio that never arrived. The audio path is short bursts of work,
/// so above everything the app runs costs the panel nothing. At 8, on
/// the same radio and band 20 minutes later, the longest reader gap was
/// 14 ms over 251 s, none reached 50 ms, and the 12 kHz rate read
/// 12 002.0 sa/s where it had read 11 996.8 with the losses in it.
const UAC_DRIVER_TASK_PRIORITY: usize = 8;
const UAC_DRIVER_TASK_CORE: sys::BaseType_t = 0;

/// `uac_app` task stack. Just runs `recv()` → device_open/start →
/// spawn reader. 4 KB is overkill but keeps headroom for the OnceLock
/// + sender state and any future device-cleanup paths.
// **8 KiB because the transmit path runs here.** `handle_tx_connected`
// is dispatched from `app_task`, and until 2026-09-20 its body had
// never executed: `uac_host_device_start` refused every call, so the
// silent-write buffer and `write_ft8_frame` below it were dead code
// that no stack measurement had ever covered. The first `start` that
// succeeded (mono, 96 B/frame) ran them and the task overflowed 4096 B
// inside a second — a reboot 30 s into the capture with no panic on
// any surviving console. The buffers are on the heap now; this is the
// margin, not the fix. The stack is PSRAM-backed (`spawn_psram_thread`),
// so the extra 4 KiB costs no internal DRAM.
const APP_TASK_STACK: usize = 8192;

/// Priority of whichever task is feeding the [`AudioSink`] — the USB
/// reader on a radio, the `MFSK_CORES3_SIM` feeder without one.
///
/// **Above the decode thread**, for the reason `stage1_inc`
/// takes 6 above `dsp_worker`'s 5: the audio path must be able to
/// preempt the decoder, because its work cannot be deferred and then
/// caught up on.
///
/// Both were priority 5 until 2026-09-19, and pthreads take
/// `CONFIG_PTHREAD_TASK_CORE_DEFAULT` = no affinity, so the audio task
/// and the decode task time-sliced one core at 100 Hz whenever the
/// decoder ran long. Cold acquisition runs ~13 s on the decode thread
/// with two `vTaskDelay(1)`s in it, and the sink's slot-boundary
/// publishes fell **4.06 s** behind over one
/// (`logs/sim_slotlen_rerun_noclock_offset3000_2026-09-19.log`, the
/// per-slot `hint_err` field) — enough to make
/// `decode_pipeline::slot_end_hint` name the wrong slot's boundary.
///
/// On the SIM feeder that is lag and nothing worse; it paces itself
/// and the audio is a `&'static [u8]`. On a radio it is loss: the
/// reader drains the IDF UAC ring, [`STREAM_BUFFER_BYTES`] = 16 KB =
/// **85 ms** at 48 kHz stereo, and `pipeline::send_box` into a
/// four-chunk (400 ms) queue blocks rather than dropping, so a
/// starved reader stops reading and the ring overruns. That is the
/// case a busy band brings on — more candidates, a longer decode —
/// and it is why this is a priority rather than a tuning knob.
///
/// **8 since 2026-09-22**, above the panel (7) as well — see
/// [`UAC_DRIVER_TASK_PRIORITY`] for the loss that caused it.
const AUDIO_TASK_PRIORITY: u8 = 8;

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
/// Audio a cold acquisition captures, in samples at 12 kHz.
///
/// **`REQUIRED_SAMPLES`, i.e. 25 s, and it cannot simply be grown.**
/// The trials that follow `acquire_slot_phases` cut a whole slot
/// starting at the candidate phase, so an offset past 10 s runs off
/// the end — a third of the phase space has no slot behind it
/// (2026-09-20). Two slots would remove the limit and was tried: the
/// ring goes to 720 KB, `take_acquisition_audio` hands that Vec out
/// while `arm_acquisition` reserves another, and the board died of
/// `rust_oom` in `stage1_inc` on the next slot's spectrogram.
///
/// The fix came from the trial's own window instead, which is where
/// the room was: those trials now cut at the nearest offset a slot
/// does fit behind, never more than 2.5 s away and so always inside
/// `decode_block_tuned`'s own search. See the clamp in
/// `decode_pipeline`'s trial loop. **This constant is load-bearing
/// for that argument** — the clamp's 2.5 s bound is
/// `(SLOT − (CAPTURE − SLOT)) / 2`, so shortening the capture widens
/// it past the search and the unreachable phases go back to
/// undecodable.
pub const ACQUIRE_CAPTURE_SAMPLES: usize = mfsk_core::ft8::acquire::REQUIRED_SAMPLES;
const ACQUIRE_RING_CAP: usize = ACQUIRE_CAPTURE_SAMPLES + CHUNK_LEN;


static ACQUIRE_ARMED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static ACQUIRE_RING: Mutex<Vec<i16>> = Mutex::new(Vec::new());

/// Samples into its slot the capture's first sample was, or
/// `usize::MAX` before a capture has started. Diagnostic (2026-09-18):
/// the ring starts filling at whatever point `arm_acquisition` is
/// called, and the phase acquisition returns is measured from the
/// capture's start, not from a slot boundary. Recorded as an atomic in
/// the audio path — no log call there, whose stack has overflowed
/// before — and printed by `decode_pipeline` beside the phase.
static ACQUIRE_START_IN_SLOT: std::sync::atomic::AtomicUsize =
    std::sync::atomic::AtomicUsize::new(usize::MAX);

/// See [`ACQUIRE_START_IN_SLOT`]. `None` until a capture has started.
pub fn acquisition_start_in_slot() -> Option<usize> {
    match ACQUIRE_START_IN_SLOT.load(Ordering::Acquire) {
        usize::MAX => None,
        n => Some(n),
    }
}

/// Start filling the acquisition ring from scratch.
pub fn arm_acquisition() {
    // **Already armed is left alone.** The capture is 25 s of audio; a
    // second arm that clears the ring throws away what has been
    // gathered and starts the wait again. That matters now that the
    // pipeline arms at boot — the `Acquire` action a slot later must
    // not undo it.
    if ACQUIRE_ARMED.load(Ordering::Acquire) {
        return;
    }
    if let Ok(mut r) = ACQUIRE_RING.lock() {
        r.clear();
        r.reserve(ACQUIRE_RING_CAP);
    }
    ACQUIRE_ARMED.store(true, Ordering::Release);
}

/// Whether a persisted grid fix was handed over by
/// [`seed_grid_fix_us`] — i.e. whether this boot has a phase from
/// anywhere other than the air.
pub fn grid_fix_seeded() -> bool {
    PENDING_GRID_FIX_US.load(Ordering::Acquire) != i32::MIN
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
/// How much of the acquisition capture is in hand, while one is armed:
/// `(have, want)` in 12 kHz samples. `None` when nothing is armed.
pub fn acquisition_fill(want: usize) -> Option<(usize, usize)> {
    if !ACQUIRE_ARMED.load(Ordering::Acquire) {
        return None;
    }
    ACQUIRE_RING.lock().ok().map(|r| (r.len().min(want), want))
}

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
/// Where a sim feed's samples come from.
///
/// Two shapes because the two baked assets are two shapes: FT8's
/// recordings are `.wav` files with a 44-byte header, and the FT4
/// golden is the raw `i16` slot `ft4_bench` already links
/// (`assets/ft4_golden_audio.bin`). Neither is worth converting to the
/// other at build time just to share one loop.
#[derive(Clone, Copy)]
pub enum SimSource {
    /// A `.wav` file; the 44-byte header is skipped.
    Wav(&'static [u8]),
    /// Raw little-endian `i16` at 12 kHz, no header.
    Pcm(&'static [u8]),
}

/// True once a sim feed is running, so a sink can say *sim* where it
/// would otherwise say *real UAC audio*.
///
/// That line is not decoration: "has FT4 ever decoded off the air on
/// this board" was answered by grepping the logs for it, and a replay
/// that claims to be a radio makes that question unanswerable.
static SIM_FEEDING: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

pub fn sim_feeding() -> bool {
    SIM_FEEDING.load(core::sync::atomic::Ordering::Acquire)
}

/// Feed baked audio to whatever sink is registered, at 12 kHz, forever.
///
/// `slot_samples` is the receiver's own slot at 12 kHz — 180 000 for
/// FT8, 90 000 for FT4 — and the loop is truncated to a whole number
/// of them; see the note inside for what a partial slot did to the
/// measurement it was supposed to make.
pub fn spawn_sim_feed(src: SimSource, slot_samples: usize, lead_silence: usize) {
    spawn_sim_feed_inner(src, slot_samples, lead_silence, false);
}

/// Feed baked audio **as one continuous stream**: the whole recording,
/// looped end to start, from the moment the task starts — no slot, so
/// nothing to align the first sample to and no re-alignment at the top
/// of each pass.
///
/// For a receiver with no slot (JTTY). The slotted feed's re-alignment
/// skips or holds samples at every pass to keep a *slot* on the clock;
/// for a stream whose only clock is its sample count that is a cut in
/// the audio, and `docs/notes/JTTY_CORES3_APP.md` §7 names it as the
/// thing a JTTY SIM loop must avoid. The recording's sample 0 is the
/// stream's sample 0, so a host run over the same recording repeated
/// sees exactly what the board does.
pub fn spawn_sim_feed_continuous(src: SimSource) {
    spawn_sim_feed_inner(src, usize::MAX, 0, true);
}

fn spawn_sim_feed_inner(src: SimSource, slot_samples: usize, lead_silence: usize, continuous: bool) {
    struct Cfg {
        src: SimSource,
        slot: usize,
        lead: usize,
        continuous: bool,
    }
    let cfg = Box::into_raw(Box::new(Cfg {
        src,
        slot: slot_samples,
        lead: lead_silence,
        continuous,
    })) as *mut core::ffi::c_void;

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `spawn_sim_feed` leaked exactly this box. Drop it once
        // its fields are copied into locals — the task never returns,
        // so nothing else needs it.
        let (src, slot_samples, lead, continuous) = {
            let cfg = unsafe { Box::from_raw(arg as *mut Cfg) };
            (cfg.src, cfg.slot, cfg.lead, cfg.continuous)
        };
        let bytes = match src {
            SimSource::Wav(w) => &w[44..],
            SimSource::Pcm(p) => p,
        };
        let pcm: Vec<i16> = bytes
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        SIM_FEEDING.store(true, core::sync::atomic::Ordering::Release);
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
        let loop_len = if pcm.len() >= slot_samples {
            pcm.len() / slot_samples * slot_samples
        } else {
            pcm.len()
        };
        if continuous {
            log::warn!(
                "uac SIM: feeding {loop_len} baked samples on loop as one continuous stream \
                 (no slot, no re-alignment) — no radio"
            );
        } else {
            log::warn!(
                "uac SIM: feeding {loop_len} of {} baked samples on loop ({} slot(s), {} trimmed for phase continuity), {} ms lead silence — no radio",
                pcm.len(),
                loop_len / slot_samples.max(1),
                pcm.len() - loop_len,
                lead / 12
            );
        }
        // **Put the recording on the grid before feeding it.**
        //
        // A sim feed that just starts at boot places its slot
        // boundaries wherever the boot fell, which is uniform over the
        // period — and a receiver whose phase comes from the clock then
        // decodes nothing, with no way to tell that from a decoder
        // fault. Measured 2026-09-21: with FT4's DT servo removed the
        // harness read `0 decodes` on every slot at a −906 ms offset,
        // and the only thing wrong was the harness.
        //
        // So the first real sample lands on a clock slot boundary, and
        // `lead` — `MFSK_SIM_OFFSET_MS` — is a *deliberate* error
        // measured from there rather than an unknown one added to it.
        // With no clock (`MFSK_SIM_NO_CLOCK`) there is no boundary to
        // aim at, which is its own experiment: the receiver has to find
        // the phase without one.
        //
        // **And aim at the clock the receiver will use, not whatever
        // is in the system clock at the moment.** `utc_now_ms` calls a
        // clock plausible by its value, and after a reflash the
        // system time survives the reset with the previous session's
        // stale reading — so this aligned to it, and the BM8563 read
        // then replaced it a moment later, while the receiver anchored
        // to the replacement. The feed's phase against the grid was
        // therefore whatever the stale clock's error was. Measured
        // 2026-09-21: every boot whose "waiting N ms" line came before
        // `rtc: system clock set` and anchored far from a boundary
        // (4 916, 5 787 ms) decoded **zero** of 13 candidates, on
        // unchanged decoder code — read at the time as a flaky grid.
        // Bounded: a board with no RTC and no network still feeds.
        if option_env!("MFSK_SIM_NO_CLOCK").is_none() && !continuous {
            let t_wait = unsafe { sys::esp_timer_get_time() };
            while mfsk_app_shared::time_sync::clock_source()
                == mfsk_app_shared::time_sync::ClockSource::Unset
                && unsafe { sys::esp_timer_get_time() } - t_wait < 10_000_000
            {
                unsafe { sys::vTaskDelay(1) };
            }
            log::warn!(
                "uac SIM: clock source {:?} after {} ms — aligning to it",
                mfsk_app_shared::time_sync::clock_source(),
                (unsafe { sys::esp_timer_get_time() } - t_wait) / 1_000,
            );
        }
        // **The recording's timeline starts at the boundary to the
        // microsecond** (`t0 = start_at` below), whenever the task
        // actually wakes: this feed is the reference the receiver's
        // grid is measured against (`grid_vs_sim_log`). It used to
        // start from the moment `vTaskDelay` returned, so the
        // recording itself sat wherever the 10 ms tick put it.
        let now_esp = unsafe { sys::esp_timer_get_time() };
        let align_us: Option<i64> = if continuous {
            None
        } else {
            mfsk_app_shared::time_sync::utc_now_us().map(|u| {
                let period_us = (slot_samples / 12) as u64 * 1_000;
                (period_us - u % period_us) as i64
            })
        };
        let start_at = match align_us {
            Some(to_boundary_us) => {
                log::warn!(
                    "uac SIM: waiting {} ms for the next {} ms boundary, then {} ms of \
                     deliberate offset — nothing is fed until then",
                    to_boundary_us / 1_000,
                    slot_samples / 12,
                    lead / 12,
                );
                now_esp + to_boundary_us + (lead as i64 * 1_000 / 12)
            }
            None if continuous => now_esp,
            None => {
                log::warn!(
                    "uac SIM: no clock — feeding from now, after {} ms of deliberate offset",
                    lead / 12
                );
                now_esp + (lead as i64 * 1_000 / 12)
            }
        };
        sleep_until_esp_us(start_at);
        // The deliberate offset, kept for the re-alignment below before
        // `lead` is reused for the (now always empty) silence prefix.
        let offset_samples = lead % slot_samples.max(1);
        let lead = 0usize;
        const BLK: usize = 256;
        /// 1 ms. It was 20 ms while the error was read from the moment
        /// the task woke — a tick of jitter; read from the block's due
        /// time it is the clock's own difference.
        const REALIGN_SAMPLES: u64 = 12;
        const NO_CLOCK_SIM: bool = option_env!("MFSK_SIM_NO_CLOCK").is_some();
        // `MFSK_SIM_CLOCK_STEP_MS=N`: step the system clock by N ms once,
        // 90 s into the feed — what an NTP correction does to a board
        // whose RTC was off. For exercising what has to follow it (the
        // receiver's grid trim, this feed's re-alignment) without
        // waiting for a board with a bad RTC.
        let clock_step_ms: Option<i64> =
            option_env!("MFSK_SIM_CLOCK_STEP_MS").and_then(|v| v.parse().ok());
        let mut clock_stepped = false;
        // The recording's first sample is *at* `start_at`, whatever
        // moment the task actually woke.
        let t0 = start_at;
        let mut fed: u64 = 0;
        let mut src = 0usize; // index into pcm, after the lead is done
        let mut lead_left = lead;
        let silence = [0i16; BLK];
        // Samples pushed into the sink so far, and where the previous
        // block started in the recording — for `SIM_PASS_START`.
        let mut pushed_total: u32 = 0;
        let mut last_src_start = usize::MAX;
        loop {
            let block: &[i16] = if lead_left >= BLK {
                lead_left -= BLK;
                &silence
            } else if lead_left > 0 {
                let n = lead_left;
                lead_left = 0;
                &silence[..n]
            } else {
                // **Back on the clock at every pass**, as a station
                // transmitting on UTC would be. The feed is placed on a
                // boundary once, at start, and after that runs on
                // `esp_timer`; when NTP lands and moves the clock, the
                // receiver follows it and the recording did not — the
                // slot rules and the signals on the waterfall stayed a
                // clock step apart and the decodes' DT walked out of the
                // search (2026-09-21, 1.6 s). So at the top of each pass,
                // where the recording's sample 0 should meet a boundary,
                // measure against the clock and wait or skip the
                // difference.
                if let (Some(step), false) = (clock_step_ms, clock_stepped) {
                    if unsafe { sys::esp_timer_get_time() } - t0 >= 90_000_000 {
                        clock_stepped = true;
                        let mut tv = sys::timeval { tv_sec: 0, tv_usec: 0 };
                        // SAFETY: valid out-pointer; timezone unused.
                        unsafe { sys::gettimeofday(&mut tv, core::ptr::null_mut()) };
                        let us = tv.tv_sec as i64 * 1_000_000 + tv.tv_usec as i64 + step * 1_000;
                        let tv = sys::timeval {
                            tv_sec: (us / 1_000_000) as _,
                            tv_usec: (us % 1_000_000) as _,
                        };
                        unsafe { sys::settimeofday(&tv, core::ptr::null()) };
                        log::warn!("uac SIM: stepped the system clock by {step:+} ms (MFSK_SIM_CLOCK_STEP_MS)");
                    }
                }
                if src == 0 && !NO_CLOCK_SIM && loop_len == slot_samples {
                    // Measured from when this block's first sample is
                    // *due*, not from when the task got round to it: a
                    // tick-late wake would otherwise read as a phase
                    // error and be "corrected" into one.
                    let due_esp = t0 + (fed * 1_000_000 / 12_000) as i64;
                    let lag_us = unsafe { sys::esp_timer_get_time() } - due_esp;
                    if let Some(to_b) = mfsk_app_shared::time_sync::utc_now_us().map(|u| {
                        mfsk_app_shared::time_sync::samples_to_next_slot_12k_from_us(
                            u.saturating_sub(lag_us.max(0) as u64),
                            (slot_samples / 12) as u64,
                        ) as usize
                    }) {
                        // To the *target* — the boundary plus any
                        // deliberate `MFSK_SIM_OFFSET_MS` — not the
                        // boundary. Positive: still ahead (early).
                        let to_b = (to_b + offset_samples) % slot_samples;
                        let err = if to_b < slot_samples / 2 {
                            to_b as i64
                        } else {
                            to_b as i64 - slot_samples as i64
                        };
                        if err.unsigned_abs() > REALIGN_SAMPLES {
                            if err > 0 {
                                // Pacing is by `fed`; counting the wait
                                // as fed holds the next block back.
                                fed += err as u64;
                            } else {
                                src = ((-err) as usize).min(loop_len - 1);
                            }
                            log::info!(
                                "uac SIM: re-aligned to the clock by {:+} samples",
                                err
                            );
                        }
                    }
                }
                let end = (src + BLK).min(loop_len);
                let s = &pcm[src..end];
                if src < last_src_start || pushed_total == 0 {
                    // A new pass (or a re-alignment that skipped into
                    // one): its sample 0 is `src` samples before this
                    // block, whether or not it was actually pushed.
                    SIM_PASS_START.store(
                        pushed_total.wrapping_sub(src as u32),
                        core::sync::atomic::Ordering::Release,
                    );
                }
                last_src_start = src;
                src = if end == loop_len { 0 } else { end };
                s
            };
            // **Handed over once it has been "recorded"**, as a radio's
            // audio is: a block covering [t, t + 21 ms) arrives after
            // t + 21 ms, never before. This used to push first and sleep
            // after, so the sink was handed audio from the future and
            // its clock-read anchor was off by a block the other way
            // from a radio's.
            fed += block.len() as u64;
            sleep_until_esp_us(t0 + (fed * 1_000_000 / 12_000) as i64);
            if let Ok(mut g) = AUDIO_SINK.lock() {
                if let Some(sink) = g.as_mut() {
                    sink.push_samples(block);
                }
            }
            crate::waterfall_feed::push(block);
            pushed_total = pushed_total.wrapping_add(block.len() as u32);
        }
    }

    let r = unsafe {
        sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"uac_sim".as_ptr(),
            // 8 KB: at 4 KB the `[stacks]` line read 104-1 728 B free,
            // and the task overflowed (canary watchpoint, `uac_sim`)
            // logging its NTP re-alignment through the UDP sink — three
            // of six SIM boots on 2026-09-22 (`logs/sim_gridprobe_*`).
            // SIM builds only; nothing ships with this task.
            8192,
            cfg,
            AUDIO_TASK_PRIORITY as u32,
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

/// How many slots between live-phase writes while the air owns the
/// grid. 20 slots is 5 minutes, and the phase moves ~1.2 ms a slot
/// (measured on a radio 2026-09-21, `logs/airdt_2026-09-21.log`), so a
/// fix read at the worst moment is ~24 ms stale — against FT4's ±1.0 s
/// search and its 0.3 s lock bar.
const LIVE_PHASE_EVERY_SLOTS: u32 = 20;

/// Last slot index a live phase was written at, and what it said.
static LIVE_PHASE_LAST: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(u32::MAX);

/// **Keep the sub-second phase where a reboot can find it.**
///
/// `persist_grid_fix` used to be reached from exactly one place: the
/// cold-acquisition success arm. So a receiver that never ran an
/// acquisition — which is the *normal* case, since the RTC places the
/// grid and one DT trim centres it — persisted nothing, and the reboot
/// into FT4 found either no fix or one from a previous day. Measured
/// 2026-09-21: `ft4_app: persisted grid fix present but stale/weak
/// (R 1.00, 88394 s old) — ignoring`.
///
/// That matters because **FT4 cannot place its own grid**: cold
/// acquisition is FT8-only, and FT4's band is too thin to derive a
/// phase from anyway. Its phase has to come from NTP or from whatever
/// FT8 left behind. This is what leaves something behind.
///
/// `err_ms` is this boundary's distance from the clock's own slot
/// grid, which is exactly what `GridFix::correction_for` hands back to
/// a reader: positive means the band's boundary falls later than the
/// clock's, and a reader adds it to its own `samples_to_next_slot`.
///
/// Written only while [`GridLock::Air`] stands — the air is then the
/// phase authority, which is the one case a reader cannot reconstruct
/// for itself. Under NTP the reader has NTP; under `Rtc` the phase is
/// the clock's own and unproven.
///
/// [`GridLock::Air`]: mfsk_app_shared::time_sync::GridLock::Air
fn persist_live_phase_if_air_owns_it(err_ms: i64, slot_idx: u32) {
    use core::sync::atomic::Ordering as O;
    if mfsk_app_shared::time_sync::grid_lock() != mfsk_app_shared::time_sync::GridLock::Air {
        return;
    }
    let last = LIVE_PHASE_LAST.load(O::Acquire);
    if last != u32::MAX && slot_idx.wrapping_sub(last) < LIVE_PHASE_EVERY_SLOTS {
        return;
    }
    let Some(now_ms) = mfsk_app_shared::time_sync::utc_now_ms() else {
        return;
    };
    LIVE_PHASE_LAST.store(slot_idx, O::Release);
    persist_grid_fix(mfsk_app_shared::grid_fix::GridFix {
        offset_us: (err_ms * 1000) as i32,
        period_s: SLOT_SECS as f32,
        epoch_at_fix: (now_ms / 1000) as i64,
        // The air raised this lock, which needs `LOCK_MIN_DECODES`
        // decodes *and* a median DT inside `LOCK_MAX_PHASE_S`. A grid
        // that has held that and is still producing slots is as
        // confident as the acquisition path's own saturated 1.0.
        confidence: 1.0,
    });
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
/// Sleep until `esp_timer` reads at least `at_us`, in whole ticks,
/// rounded **up**. Never early — `vTaskDelay` of a truncated count wakes
/// before the deadline, and a block handed over before its last sample
/// is due is audio from the future. Late by up to a tick is fine: that
/// is what a radio's delivery looks like too, and `GridPhase` takes the
/// minimum over a slot of blocks precisely so that lateness drops out.
/// No spin: this task sits on core 0 above the decoder and the panel,
/// and a spin there would take their time.
fn sleep_until_esp_us(at_us: i64) {
    let tick_us = 1_000_000 / sys::configTICK_RATE_HZ as i64;
    loop {
        let left = at_us - unsafe { sys::esp_timer_get_time() };
        if left <= 0 {
            return;
        }
        unsafe { sys::vTaskDelay(((left + tick_us - 1) / tick_us) as u32) };
    }
}

/// Where the SIM feed began its current pass over the recording, as an
/// index into the stream it has pushed through the sink (wrapping u32,
/// ~99 h at 12 kHz; only differences are used). `u32::MAX`
/// until the first pass, and forever off the SIM.
pub(crate) static SIM_PASS_START: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(u32::MAX);

/// **The grid against the recording, in samples — no clock involved.**
///
/// On the SIM the recording's sample 0 is where its slot begins, so the
/// distance from there to the boundary the sink declared is the grid's
/// true error. Every other reading of the grid on this board goes
/// through `utc_now_ms()`, at the time a chunk happens to be delivered,
/// which is the thing under suspicion. Positive: the boundary is later
/// than the recording's start, so every DT reads that much low.
fn grid_vs_sim_log(slot: usize, boundary_at: u32) {
    let p = SIM_PASS_START.load(Ordering::Acquire);
    if p == u32::MAX {
        return;
    }
    let slot_len = SLOT_SAMPLES_12K as i64;
    let d = (boundary_at.wrapping_sub(p) as i32 as i64).rem_euclid(slot_len);
    let d = if d > slot_len / 2 { d - slot_len } else { d };
    log::info!(
        "uac: slot {} boundary {:+} samples ({:+.1} ms) from the SIM recording's start",
        slot + 1,
        d,
        d as f32 / 12.0,
    );
}

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
    /// Samples handed downstream since this sink was created. A slot
    /// boundary's index in the stream, for [`SIM_PASS_START`] to be
    /// compared against without either side reading a clock.
    sent_total: u32,
    /// This slot's blocks against UTC — see `time_sync::GridPhase`.
    phase: mfsk_app_shared::time_sync::GridPhase,
    /// The grid has been put on UTC once under NTP; from here it is
    /// held, and moved again only past [`HOLD_REANCHOR_SAMPLES`].
    fine_locked: bool,
}
// SAFETY: `QueueHandle_t` is a raw pointer into IDF-owned state; the
// IDF queue API is thread-safe by design (that's the whole point of a
// FreeRTOS queue), and `Ft8ChunkSink` never dereferences the pointer
/// **How long the reader spent blocked handing chunks on.** Max and
/// total per 1 Hz tick, in µs, reset by the tick that prints them.
///
/// The question these answer cannot be answered from the outside: a
/// second with fewer bytes in it says audio was lost, not where. The
/// reader drains a 16 KB ring (85 ms, [`STREAM_BUFFER_BYTES`]) and
/// blocks on a four-chunk queue (400 ms), so "the reader was held
/// longer than the ring" and "the driver was not scheduled" produce
/// the same missing bytes and want opposite fixes.
///
/// Two relaxed atomics per chunk, no allocation, no lock, nothing on
/// the stack — the constraints `embedded-poc/CLAUDE.md` sets for a
/// probe on this board.
static SINK_BLOCK_MAX_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static SINK_BLOCK_SUM_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// **Longest gap between two reads, per tick, in µs.** The question
/// [`SINK_BLOCK_MAX_US`] cannot answer.
///
/// A deficit second is never followed by a surplus one — measured on a
/// radio 2026-09-19, 34-38 reads of 4 096 B where every other second
/// does exactly 47 — so the samples are discarded rather than queued,
/// and the ring is only [`STREAM_BUFFER_BYTES`] = 85 ms deep. Anything
/// in the path that stops for longer than that loses audio outright,
/// and "the reader was not scheduled" and "the class driver did not
/// resubmit transfers" look identical from the byte count while
/// wanting opposite fixes. A read normally completes every 21.3 ms
/// (4 096 B at 192 000 B/s), so this separates them on its own: a
/// figure near 21 ms exonerates this thread.
static READ_GAP_MAX_US: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
/// Reads that came back `ESP_ERR_TIMEOUT` — the ring stayed empty for
/// the whole [`READER_READ_TIMEOUT_MS`]. Counted because the loop
/// otherwise `continue`s past it in silence, and it is the driver side
/// of the same question.
static READ_TIMEOUTS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Hand a chunk on, timing how long that took.
fn send_chunk_timed(q: sys::QueueHandle_t, msg: Box<ChunkMsg>) {
    let t0 = unsafe { sys::esp_timer_get_time() };
    send_box(q, msg);
    let dt = (unsafe { sys::esp_timer_get_time() } - t0).clamp(0, u32::MAX as i64) as u32;
    SINK_BLOCK_SUM_US.fetch_add(dt, Ordering::Relaxed);
    SINK_BLOCK_MAX_US.fetch_max(dt, Ordering::Relaxed);
}

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
            sent_total: 0,
            phase: mfsk_app_shared::time_sync::GridPhase::new(),
            fine_locked: false,
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
                if r.is_empty() {
                    ACQUIRE_START_IN_SLOT
                        .store(self.slot_samples + self.chunk.len(), Ordering::Release);
                }
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
        // **`AIR DT` takes the anchor too.** It used not to, on the
        // reading that an RTC-guessed phase only got in the air's way:
        // "measured 0.68 s and 1.65 s out on 2026-09-19, each time
        // costing minutes before the capture put it right".
        //
        // Those measurements were taken while the audio path was losing
        // 6.5 % of its samples (the isochronous URB budget, aa10bf2e).
        // The RTC was not 1.65 s wrong; the grid was *drifting* that far
        // after the anchor placed it correctly, at +1035 ms a slot. With
        // the loss gone the same anchor holds to −5.8 ms a slot, and
        // skipping it costs exactly what it was supposed to save:
        // measured on a radio 2026-09-19, an `AIR DT` start with a
        // good RTC spent **2 min 9 s** and two 25 s captures — the
        // first of which decoded nothing at all — rediscovering a phase
        // the clock already had.
        //
        // The air still owns the phase: this is a one-shot placement,
        // `clock_is_disciplined()` stays false without NTP, and the
        // acquisition path is untouched. If the RTC really is seconds
        // out — the hilltop case this mode exists for — the anchor
        // places the grid wrongly, nothing decodes, and the under-par
        // run reaches acquisition exactly as before. There is no case
        // where starting from the clock is worse than starting from
        // nothing.
        if !self.coarse_anchored {
            if let Some(remain) = mfsk_app_shared::time_sync::samples_to_next_slot_12k(SLOT_SECS) {
                // The clock puts the boundary within a second; a
                // persisted air fix puts it within milliseconds. Same
                // fold and the same sign as `apps/ft4.rs` does for its
                // own grid — µs to 12 kHz samples, modulo the period.
                let fix_us = PENDING_GRID_FIX_US.load(Ordering::Acquire);
                let remain = if fix_us == i32::MIN {
                    remain
                } else {
                    let shifted = remain as i64 + (fix_us as i64 * 12 / 1000);
                    let r = shifted.rem_euclid(SLOT_SAMPLES_12K as i64) as usize;
                    log::info!(
                        "uac: folding a persisted air fix into the anchor — {:+} ms                          ({} ms to the boundary, was {})",
                        fix_us / 1000,
                        r / 12,
                        remain / 12,
                    );
                    r
                };
                // **Shorten this slot; do not pre-load its counter.**
                //
                // Both ways move the boundary to `remain` samples from
                // now. Only one of them tells the truth downstream:
                // `slot_samples` is "how much of this slot has been
                // pushed", and stage1_inc fills its buffer from what it
                // actually receives, so a pre-loaded counter makes
                // `SlotEnd` report a full slot while the buffer holds
                // `remain` samples. `finalize_slot` then hands the
                // decoder a truncated slot labelled as a whole one —
                // the last second missing, every station at positive DT
                // losing its tail. Seen on the radio as
                // `audio_fill=168000 != reported total 180756` and, in
                // NTP mode where the drift re-anchor fires often, as
                // every other slot decoding nothing (2026-09-19).
                // **From the block's last sample, not its first.** The
                // clock was read now, and what arrives now is audio
                // that has already happened — a radio's block ends
                // about now; its first sample is a block older. So the
                // boundary is `remain` past the *end* of this block,
                // and if the block itself spans the boundary, inside
                // it (the per-sample split below lands it exactly).
                let before = self.slot_samples + self.chunk.len();
                let mut target = before + samples.len() + remain;
                if target > SLOT_SAMPLES_12K && target - SLOT_SAMPLES_12K >= before {
                    target -= SLOT_SAMPLES_12K;
                }
                self.slot_target = target;
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
                    "uac: slot grid coarse-anchored to the system clock — {} ms to the next \
                     boundary (clock sub-second {} ms, slot_samples now {})",
                    remain / 12,
                    mfsk_app_shared::time_sync::utc_now_ms().map_or(-1, |ms| (ms % 1000) as i64),
                    self.slot_samples,
                );
            }
        }
        // One observation of the grid against UTC per block, taken
        // before the block is split: samples from this block's last
        // sample to the grid's next boundary, against the clock's.
        if self.coarse_anchored {
            if let Some(u) = mfsk_app_shared::time_sync::utc_now_us() {
                let at_end = (self.slot_samples + self.chunk.len() + samples.len()) as i64;
                self.phase.observe(
                    mfsk_app_shared::time_sync::clock_epoch(),
                    self.slot_target as i64 - at_end,
                    mfsk_app_shared::time_sync::samples_to_next_slot_12k_from_us(
                        u,
                        SLOT_SECS as u64 * 1_000,
                    ),
                    SLOT_SAMPLES_12K as i64,
                );
            }
        }
        for &s in samples {
            self.chunk.push(s);
            // **The boundary falls on the sample, not on a chunk.** The
            // chunk used to be sent only when full, and the slot could
            // end only after a whole chunk: every boundary was rounded
            // up to the next 1 200 samples (100 ms) of the stream. Every
            // later slot is 150 chunks long, so whatever the first
            // rounding was — anything in 0-100 ms, set by where the
            // stream happened to start — stayed for the whole session,
            // and the NTP re-anchor's ±200 ms dead zone never took it
            // out. Measured against the SIM recording's own start
            // (`grid_vs_sim_log`, no clock involved; 2026-09-22,
            // `logs/sim_gridprobe_*`): rounded, three boots put the
            // boundary +100, +0/+46/+100 and +100 ms late, W1FC read
            // DT +0.12 instead of +0.22 and one station of 7 missed
            // key-up; split here, five boots put it at -9 to +2 ms and
            // four of them decoded all 7 by key-up.
            let at_boundary = self.slot_samples + self.chunk.len() >= self.slot_target;
            if self.chunk.len() >= CHUNK_LEN || at_boundary {
                let n = self.chunk.len();
                let to_send = core::mem::replace(&mut self.chunk, Vec::with_capacity(CHUNK_LEN));
                send_chunk_timed(self.chunk_q, Box::new(ChunkMsg::Samples(to_send)));
                self.slot_samples += n;
                self.sent_total = self.sent_total.wrapping_add(n as u32);
                if self.slot_samples >= self.slot_target {
                    grid_vs_sim_log(self.wav_idx, self.sent_total);
                    let phase_est = self.phase.take();
                    // This slot's USB timing, kept for the one summary line
                    // below. One datagram per slot: the UDP log drops
                    // lines, and an estimate whose correction line went
                    // missing reads as a step that never happened.
                    let usb_summary = {
                        let n = READS_IN_SLOT.swap(0, Ordering::Relaxed);
                        let wait = READ_WAIT_MAX_US.swap(0, Ordering::Relaxed);
                        let at = READ_WAIT_MAX_AT_MS.swap(u32::MAX, Ordering::Relaxed);
                        let over = READ_WAITS_OVER.swap(0, Ordering::Relaxed);
                        let gap = READ_GAP_SLOT_MAX_US.swap(0, Ordering::Relaxed);
                        let gap_at = READ_GAP_SLOT_MAX_AT_MS.swap(u32::MAX, Ordering::Relaxed);
                        let ms = |a: u32| if a == u32::MAX { -1 } else { a as i64 };
                        format!(
                            "reads={n} wait={:.1}@{} over{}={over} gap={:.1}@{}",
                            wait as f32 / 1_000.0,
                            ms(at),
                            READ_WAIT_WARN_US / 1_000,
                            gap as f32 / 1_000.0,
                            ms(gap_at),
                        )
                    };
                    if let Some(e) = phase_est {
                        log::info!(
                            "uac: slot {} grid {:+} samples ({:+.1} ms) against UTC, min over the slot's blocks",
                            self.wav_idx + 1,
                            e,
                            e as f32 / 12.0,
                        );
                    }
                    send_chunk_timed(
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

                    // **Where this boundary fell against UTC**, in ms
                    // into the 15 s grid, signed to the nearer tick.
                    //
                    // The one measurement that separates the two
                    // explanations for a slot grid that reads ~0.75 s
                    // out after the anchor and 0.04 s out once the
                    // per-slot tracker has had a turn (measured on a
                    // radio 2026-09-19, the same 0.7 s in both NTP and
                    // `AIR DT` on the first full slot):
                    //
                    //   ≈0    — the boundary is declared at the right
                    //           *time*, so the offset is in the audio
                    //           this sink is holding, not in the
                    //           arithmetic;
                    //   ≈±750 — the declaration itself is late or
                    //           early, and the bug is in `slot_target`
                    //           / `slot_samples` or in the anchor.
                    //
                    // Guessing between them was tried and the sign did
                    // not work out either way, which is why this is a
                    // measurement rather than a fix.
                    if let Some(ms) = mfsk_app_shared::time_sync::utc_now_ms() {
                        let into = (ms % (SLOT_SECS as u64 * 1000)) as i64;
                        let err = if into > SLOT_SECS as i64 * 500 {
                            into - SLOT_SECS as i64 * 1000
                        } else {
                            into
                        };
                        log::info!(
                            "uac: slot {} boundary {err:+} ms off the UTC grid (target {} samples)",
                            self.wav_idx,
                            self.slot_target,
                        );
                        persist_live_phase_if_air_owns_it(err, self.wav_idx as u32);
                    }

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
                            // **Correct late by shortening, leave
                            // small early alone, and take one short
                            // slot for a large early.**
                            //
                            // This used to fire only past
                            // `SLOT_DRIFT_REANCHOR_MS`, and then took
                            // the whole error out of one slot. That is
                            // the worse half of the trade twice over:
                            // the audio arrives at 12 032 sa/s against
                            // a nominal 12 000 (measured on an IC-705,
                            // 2026-09-19), so the error refills and the
                            // correction is not rare; and a slot cut
                            // more than 12 000 samples short never
                            // reaches `stage1_inc::SPEC_EMIT_PAIR` at
                            // 168 000, so it emits a partial SpecBundle
                            // and the decoder gets no tail window at
                            // all. Measured: alternating slots at
                            // `pair_done=85/92`, `tail_win=0`, `dec` 0-2
                            // against 7 on the slots between them.
                            //
                            // Following it every slot keeps the phase
                            // inside one chunk of UTC, and the clamp
                            // spreads a genuine step (an NTP jump) over
                            // a few slots that all still decode instead
                            // of one that cannot.
                            let target = if remain <= DEAD_ZONE_SAMPLES
                                || remain >= SLOT_SAMPLES_12K - DEAD_ZONE_SAMPLES
                            {
                                // On the grid to within the dead zone,
                                // which is as close as one clock read at
                                // a chunk's delivery can say. The slot's
                                // blocks say better (`GridPhase`): take
                                // that out, to the sample. Before this,
                                // an error inside ±200 ms stayed for the
                                // session — up to 100 ms of it from the
                                // chunk rounding, ~10 ms from the anchor.
                                let min = if self.fine_locked {
                                    HOLD_REANCHOR_SAMPLES
                                } else {
                                    FINE_PHASE_MIN_SAMPLES
                                };
                                match phase_est {
                                    Some(e) if e.unsigned_abs() >= min => {
                                        self.fine_locked = true;
                                        let e = e.clamp(
                                            -(DEAD_ZONE_SAMPLES as i64),
                                            DEAD_ZONE_SAMPLES as i64,
                                        );
                                        log::info!(
                                            "uac: grid {e:+} samples off UTC — next slot {} samples",
                                            SLOT_SAMPLES_12K as i64 - e
                                        );
                                        (SLOT_SAMPLES_12K as i64 - e) as usize
                                    }
                                    _ => SLOT_SAMPLES_12K,
                                }
                            } else {
                                // End this slot on the next UTC
                                // boundary. Exact, and done in one
                                // slot whichever way the error points.
                                remain
                            };
                            if err_ms.unsigned_abs() > SLOT_DRIFT_REANCHOR_MS {
                                log::warn!(
                                    "uac: slot phase {err_ms:+} ms off UTC — next slot {target} \
                                     samples"
                                );
                            }
                            self.slot_target = target;
                            log::info!(
                                "uac: slot {} summary est={} next={} {usb_summary}",
                                self.wav_idx,
                                phase_est.map_or(String::from("-"), |e| format!("{e:+}")),
                                target,
                            );
                        }
                    } else {
                        let acq = mfsk_app_shared::time_sync::take_acquisition_shift_12k();
                        if acq != 0 {
                            // Cold-acquisition one-shot (#356b): the grid
                            // was lost past ±1 s and `decode_pipeline`
                            // recovered the phase from a 25 s FT8
                            // capture, or the DT trim wants a smaller
                            // move through the same channel.
                            //
                            // **A slot can only be shortened.** This
                            // used to be `(SLOT + acq).clamp(30_000,
                            // 300_000)`, and `acq` reaches +90 000 (dt
                            // is normalised to ±7.5 s), so a positive
                            // shift asked for a slot of up to 270 000
                            // samples. `stage1_inc::NMAX` is 180 000 and
                            // is the spectrogram's geometry, not a
                            // buffer — the excess is dropped with a
                            // warning, and the sink then reports a total
                            // the builder never held. That is the same
                            // `audio_fill != reported total` split the
                            // grid anchor had this morning.
                            //
                            // A shift of `+acq` and one of `acq - SLOT`
                            // land the boundary in the same place, so
                            // the positive case is taken by *shortening*
                            // to `acq`. Never fired on hardware — both
                            // acquisitions measured 2026-09-19 were
                            // negative — which is the whole reason to
                            // fix it now rather than after it does.
                            let acq_abs = acq.unsigned_abs() as usize;
                            self.slot_target = if acq_abs <= DEAD_ZONE_SAMPLES {
                                SLOT_SAMPLES_12K
                            } else if acq > 0 {
                                acq as usize
                            } else {
                                (SLOT_SAMPLES_12K as i32 + acq).max(MIN_ACQ_SLOT_SAMPLES) as usize
                            };
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

                    // **How much of this slot is still to come**, from
                    // the boundary published just above — not the
                    // slot's nominal length.
                    //
                    // The two branches above can both move the grid:
                    // the UTC coarse anchor and the drift re-anchor set
                    // `slot_samples` (this slot starts part-way in), and
                    // acquisition / air-sync set `slot_target` (it runs
                    // long or short). Publishing `slot_target` alone was
                    // right only when neither had fired, and wrong by
                    // exactly the jump when one had — which is the
                    // moment the decode task most needs the number,
                    // since the floor is about to judge the next bundle
                    // by it. Seen on a radio as `hint_err` near −1 s
                    // right after each re-anchor (2026-09-19).
                    let remaining = self.slot_target.saturating_sub(self.slot_samples);
                    mfsk_app_shared::time_sync::publish_capture_slot_len_us(
                        remaining as i64 * (SLOT_SECS as i64 * 1_000_000)
                            / SLOT_SAMPLES_12K as i64,
                    );
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

/// A persisted grid-phase fix waiting to be folded into the first
/// coarse anchor, in microseconds — `i32::MIN` when there is none.
///
/// The RTC gives the boundary to within a second; this gives the rest.
/// It is the same record `apps/ft4.rs` reads on the FT8 → FT4 reboot,
/// used here for the case it was always for and never wired to: a
/// station that acquired from the air yesterday and is switched on
/// again today, with no network to ask.
static PENDING_GRID_FIX_US: core::sync::atomic::AtomicI32 =
    core::sync::atomic::AtomicI32::new(i32::MIN);

/// Hand the sink a persisted fix, before the first audio arrives.
pub fn seed_grid_fix_us(offset_us: i32) {
    PENDING_GRID_FIX_US.store(offset_us, Ordering::Release);
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
pub const SLOT_SAMPLES_12K: usize = 180_000;
/// The same slot, in seconds — what the UTC grid is computed from.
const SLOT_SECS: u64 = 15;
/// Phase error that is worth a log line. The correction itself runs
/// every slot (see the tracking block); this is only the threshold for
/// saying so, so it stays at a tenth of the ±2.5 s FT8 searches.
const SLOT_DRIFT_REANCHOR_MS: u32 = 250;

/// How far off the grid may sit before a correction is worth a slot.
///
/// **Corrections are exact and cost at most one slot; they are never
/// spread.** Ending the slot on the next UTC boundary puts the grid
/// right in one step whichever way the error points, and when the
/// error is large that slot is short enough to emit a partial
/// SpecBundle and decode nothing. That is the whole price, and it is
/// the right one: spreading a 3 s error over six 0.5 s steps gives six
/// slots at a phase the band cannot be found at, where one short slot
/// gives one.
///
/// A clamped, spread version shipped for one run and taught this the
/// expensive way. `stage1_inc::NMAX` is 180 000 and it is the
/// spectrogram's geometry (`N_TIME = NMAX / NSTEP - 3`), not a buffer
/// that can be grown — so a grid running *early* cannot be pulled back
/// by running one slot long, and clamping it to a 174 000 floor
/// shortens where lengthening was wanted. The error then grew by
/// exactly one clamp step per slot: on a radio it walked +503, +1027,
/// +1494 … +7002 ms, and every one of those slots decoded nothing —
/// 26 of 123 slots in a 30-minute run, 2026-09-19.
///
/// The dead zone is what keeps the exact correction from firing on
/// jitter: once the audio rate error is gone (measured −5.8 ms per
/// slot) the phase sits near zero and crosses it, and a correction
/// that costs a slot must not trigger on that. 200 ms, a fifth of the
/// ±1.0 s the coarse search covers.
const DEAD_ZONE_SAMPLES: usize = 2_400;

/// The smallest `GridPhase` error the NTP path corrects: 6 samples,
/// 0.5 ms. Below that the estimate is the clock's microsecond read and
/// the minimum's own spread; moving the grid for it would only chase
/// noise.
const FINE_PHASE_MIN_SAMPLES: u64 = 6;

/// **Once on UTC, hold.** After the first fine correction the grid is
/// moved again only when `GridPhase` says it is 10 ms out.
///
/// Correcting every slot past [`FINE_PHASE_MIN_SAMPLES`] moved the grid
/// 3-6 ms on most slots on air (2026-09-22,
/// `logs/live_ft8_62e0bee6_2026-09-22.log`), and this board's FT8 gains
/// or loses a weak station on 5 ms of phase — so the correction was
/// reshuffling the station list. Held, the grid showed two things: a
/// drift of 3-4 samples a slot (the IC-705's 48 kHz against the
/// ESP32's crystal, ~20 ppm), and 36-sample steps that were isochronous
/// packets skipped while `board::log_task_cpu` held a critical section.
/// With that gone the drift alone reaches this in ~30 slots.
const HOLD_REANCHOR_SAMPLES: u64 = 120;

/// Floor for a cold-acquisition slot, in samples. `acq` is bounded by
/// ±90 000, so the shortening branch cannot go below 90 000 on its own;
/// this is the guard for a future wider bound rather than a live one.
const MIN_ACQ_SLOT_SAMPLES: i32 = 30_000;

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
    // **Latch it, because the log line may never arrive.**
    //
    // Enumeration happens inside `start_host`, which runs as soon as
    // the UDP sink object exists — and on 2026-09-20 that was 33 s
    // before WiFi finished associating, so every line between the two
    // was written and dropped. The resulting log showed no
    // `TxConnected` and no `RxConnected` while the receiver was plainly
    // decoding, which says nothing about the radio and everything
    // about the path the evidence took.
    //
    // A counter survives that. `usb_host_lib_info`'s periodic line
    // reports it, so "did the OUT interface ever appear" is answerable
    // from any later point in the log.
    match driver_event {
        DriverEvent::RxConnected { iface_num, .. } => {
            RX_IFACE_SEEN.store(iface_num as i32, Ordering::Relaxed);
        }
        DriverEvent::TxConnected { iface_num, .. } => {
            TX_IFACE_SEEN.store(iface_num as i32, Ordering::Relaxed);
        }
    }
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
/// **How long each read waited for its 4 096 bytes**, per slot.
///
/// `uac_host_device_read` blocks until the whole request is in the
/// ring (`_ring_buffer_pop` loops to the full size or the timeout), and
/// the ring fills only as the class driver completes isochronous URBs.
/// So a read normally takes ~21 ms — 4 096 B at 192 kB/s — and a lapse
/// in URB completions lengthens the one read it falls in by the lapse.
/// The IDF host keeps two 8 ms DMA buffers in flight and restarts a
/// drained isochronous schedule `XFER_LIST_ISOC_MARGIN` (3) frames late
/// (`hcd_dwc.c`), so a lapse costs at least 3 ms of audio — the
/// 36-sample steps the slot grid showed on air (2026-09-22). A step in
/// a slot whose longest read stayed ~21 ms is the other case: packets
/// dropped individually inside a steady flow (`uac-host` at DEBUG then
/// names them). `READ_GAP_MAX_US` does not answer this: it times the
/// reader's own work *between* reads. Written by the reader; read and
/// reset once a slot by the sink. Times in µs.
static READ_WAIT_MAX_US: AtomicU32 = AtomicU32::new(0);
/// Milliseconds into the 15 s UTC slot at which the longest wait ended.
static READ_WAIT_MAX_AT_MS: AtomicU32 = AtomicU32::new(u32::MAX);
static READ_WAITS_OVER: AtomicU32 = AtomicU32::new(0);
static READS_IN_SLOT: AtomicU32 = AtomicU32::new(0);
/// The other half: the longest stretch *between* reads this slot, and
/// when it ended. A stall that stops the reader too shows here rather
/// than in the wait if it caught the reader outside a read.
static READ_GAP_SLOT_MAX_US: AtomicU32 = AtomicU32::new(0);
static READ_GAP_SLOT_MAX_AT_MS: AtomicU32 = AtomicU32::new(u32::MAX);
/// A read this long has waited through more than half a read's worth
/// of missing URB completions.
const READ_WAIT_WARN_US: u32 = 30_000;

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
    if let Err(e) = spawn_psram_thread(
        c"uac_reader",
        READER_TASK_STACK,
        Some(AUDIO_TASK_PRIORITY),
        // **PRO_CPU, with the rest of the capture path.**
        //
        // Left unpinned it shares whichever core has room, and (at the
        // priority 6 it had then) it could land on APP_CPU beside
        // `stage1_inc` — also 6, and the task whose lateness the whole
        // slot grid rides on. Equal priority there is round-robin, so
        // the reader's resampling takes half of stage1_inc's core
        // whenever they coincide.
        //
        // Measured on a radio (2026-09-19, `logs/udp_monitor`): with
        // the reader unpinned, `hint_err` ran −0.6..−1.1 s and every
        // other slot's SpecBundle arrived 69-90 ms before key-up, where
        // the 500 ms floor dropped it — a real band decoding 8-13
        // stations a slot, and half the slots never tried. The SIM
        // feeder is pinned to core 0, which is why nothing on the bench
        // showed it.
        Some(esp_idf_svc::hal::cpu::Core::Core0),
        move || reader_thread(handle_wrapped, addr, iface_num),
    ) {
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

/// The radio's audio OUT interface as `addr << 8 | iface`, or
/// `u32::MAX` before `TxConnected`. Written by `app_task`, which only
/// records it.
static TX_IFACE: AtomicU32 = AtomicU32::new(u32::MAX);

/// `(addr, iface)` of the radio's audio OUT interface, once enumerated.
pub(crate) fn tx_iface() -> Option<(u8, u8)> {
    let v = TX_IFACE.load(Ordering::Acquire);
    (v != u32::MAX).then_some(((v >> 8) as u8, v as u8))
}

/// Device callback for an OUT handle. Not [`device_event_cb`]: that one
/// stops the RX reader on `DISCONNECTED`, and a transmit handle's
/// events are not the receiver's.
pub(crate) extern "C" fn tx_device_event_cb(
    _handle: sys::uac::uac_host_device_handle_t,
    event: sys::uac::uac_host_device_event_t,
    _arg: *mut core::ffi::c_void,
) {
    if event == sys::uac::uac_host_device_event_t_UAC_HOST_DEVICE_EVENT_TX_DONE {
        TX_DONE_EVENTS.fetch_add(1, Ordering::Relaxed);
    } else {
        log::warn!("uac: tx device event raw={event}");
    }
}

/// `TX_DONE` callbacks from OUT handles — how often the driver drained
/// the OUT ring to below its threshold.
pub(crate) static TX_DONE_EVENTS: AtomicU32 = AtomicU32::new(0);

/// One silent write attempt, this many times, before closing again.
/// Enough to see whether the ring buffer keeps accepting writes past
/// the first one (an OUT endpoint that stalls after N packets would
/// look identical to success on a single write).
const TX_PROBE_WRITES: u32 = 20;
/// One video frame's worth at 48 kHz stereo 16-bit — matches
/// `STREAM_BUFFER_THRESHOLD`'s sizing logic, scaled down since this
/// only needs to exercise the path, not sustain real-time throughput.
const TX_PROBE_CHUNK_BYTES: usize = 1920; // 10 ms @ 48k stereo 16b
/// Passed to `uac_host_device_write`, which takes **ticks**, not ms
/// (`uac_host.h`: "Timeout in ticks"). At `CONFIG_FREERTOS_HZ=100`
/// this is 1 s — long enough that a full ring blocks rather than
/// fails, which is the backpressure `write_ft8_frame` paces on.
const TX_PROBE_WRITE_TIMEOUT_MS: u32 = 100;

/// **Amplitude for a real FT8 frame, and why it is not the default.**
///
/// `MFSK_CORES3_TX_AMPLITUDE=<0..32767>` at build time. **Unset means
/// zero, which means the whole transmit chain runs and the radio hears
/// nothing.** That is the point: `GfskStream`, the 12 k → 48 k
/// upsample, the chunked writes and their timing are all exercised
/// identically at amplitude 0, so the path can be proven before
/// anything can reach the air.
///
/// A nonzero value puts a real signal into the IC-705's USB MOD input,
/// and if the radio's `PTT SOURCE` is `VOX` **that keys the
/// transmitter with no further action from this board**. Software here
/// cannot read that menu. Before setting this:
///
/// - `PTT SOURCE` is not `VOX`, or PTT is under deliberate control
/// - the radio is on a dummy load, or on a band and power the operator
///   intends to transmit on
/// - the frequency is one this station is licensed and willing to
///   transmit on, at the moment the board decides to
///
/// 20 000 of 32 767 is what `m5stack-s3-app`'s `tx.rs` uses (≈ −4 dBFS,
/// headroom for the GFSK envelope while staying above the noise an ALC
/// needs).
pub(crate) const TX_AMPLITUDE: i16 = match option_env!("MFSK_CORES3_TX_AMPLITUDE") {
    Some(s) => crate::decode_pipeline::parse_u32(s) as i16,
    None => 0,
};

/// One 20 ms chunk at 12 kHz — what `GfskStream` is filled with and
/// what `m5stack-s3-app`'s `tx::play` sends, chosen there for DMA
/// underrun headroom against ringbuffer overhead.
const TX_CHUNK_12K: usize = 240;
/// Buffer for one chunk at 48 kHz 16-bit after 4x zero-order hold —
/// sized for the **stereo** worst case, `240 * 4 * 2 * 2`.
///
/// The format actually in use is decided at runtime by which entry of
/// `TX_CONFIGS` `uac_host_device_start` accepts, and on this board
/// that is mono, which fills half of this. Sized for the larger so
/// one buffer serves both; `write_ft8_frame` passes the bytes it
/// really wrote as the transfer length, never `len()`.
const TX_CHUNK_BYTES: usize = TX_CHUNK_12K * 4 * 2 * 2;

/// Send one FT8 frame to the radio's USB audio input, synthesising it
/// a chunk at a time.
///
/// Returns `(chunks_written, worst_chunk_us, total_us)`.
///
/// **The waveform never exists as a buffer.** `GfskStream` produces
/// 20 ms at a time straight into the upsample scratch — measured on
/// this board at 439 us per chunk against the 20 ms the chunk
/// represents, a 2.2 % duty. The batch synthesiser it replaces cost
/// 472 ms up front and 1.2 MB of PSRAM temporaries, which put the
/// decoder's deadline before the end of the slot it was decoding
/// (`docs/notes/CORES3_FT8_SLOT_BUDGET.md` §10).
pub(crate) fn write_ft8_frame(
    handle: sys::uac::uac_host_device_handle_t,
    msg77: &[u8; 77],
    df_hz: f32,
    amplitude: i16,
    // Channel count `uac_host_device_start` actually accepted. The
    // driver sizes its isochronous packet from this, so writing the
    // other layout does not merely sound wrong — it hands the radio
    // twice or half the audio it is pacing for. On this board the
    // accepted format is mono, because a 2-channel 48 kHz alt is
    // 192 B/frame against a 128 B periodic-OUT FIFO limit.
    channels: u8,
) -> (u32, i64, i64) {
    let tones = mfsk_core::engine::tx::message_to_tones::<mfsk_core::ft8::Ft8>(msg77);
    let mut stream = mfsk_core::engine::dsp::gfsk::GfskStream::new(
        &tones,
        df_hz,
        &mfsk_core::ft8::wave_gen::FT8_GFSK,
    );
    let mut mono = [0i16; TX_CHUNK_12K];
    // Heap: 3 840 B is most of `APP_TASK_STACK` on its own.
    let mut out = vec![0u8; TX_CHUNK_BYTES];
    let now_us = || unsafe { esp_idf_svc::sys::esp_timer_get_time() };
    let (mut chunks, mut worst, t_start) = (0u32, 0i64, now_us());
    while stream.remaining() > 0 {
        let t0 = now_us();
        let n = stream.fill_i16(&mut mono, amplitude);
        // `remaining() > 0` should always yield `n > 0`, and the host
        // tests pin that against `synth_f32` under ragged chunking.
        // The guard is here because the alternative to being wrong
        // about it is a task spinning forever inside an enumeration
        // callback, on a board whose console is gone.
        if n == 0 {
            log::warn!(
                "uac: tx frame stalled at chunk {chunks} with {} left",
                stream.remaining()
            );
            break;
        }
        // 4x zero-order hold, L = R. The receive side's
        // `LinearResamplerI16To12k` run backwards; ZOH rather than
        // interpolation because the radio's own input filter is what
        // shapes the image, and `m5stack-s3-app` has driven an IC-705
        // this way since Phase 1.6.
        let mut o = 0usize;
        for &sm in mono.iter().take(n) {
            let [lo, hi] = sm.to_le_bytes();
            for _ in 0..4 {
                out[o] = lo;
                out[o + 1] = hi;
                o += 2;
                if channels == 2 {
                    out[o] = lo;
                    out[o + 1] = hi;
                    o += 2;
                }
            }
        }
        let err = unsafe {
            sys::uac::uac_host_device_write(
                handle,
                out.as_mut_ptr(),
                o as u32,
                TX_PROBE_WRITE_TIMEOUT_MS,
            )
        };
        if err != sys::ESP_OK as sys::esp_err_t {
            log::warn!("uac: tx write failed at chunk {chunks} err={err:#x}");
            break;
        }
        chunks += 1;
        worst = worst.max(now_us() - t0);
    }
    (chunks, worst, now_us() - t_start)
}

/// **This blocks `app_task` for the length of the frame, and
/// `RxConnected` waits behind it.** `app_task` drains one mpsc
/// receiver in order; the radio delivers `TxConnected` (iface 1)
/// before `RxConnected` (iface 2), so a 12.64 s frame delays the
/// receiver by that much. Observed 2026-09-20: `TxConnected` at
/// `.780`, `RxConnected` queued at `.785` and not served until the
/// probe returned. That costs the first slot of a capture and nothing
/// after it, which is the right price for a probe and the wrong one
/// for anything that ships — a real transmit path has to be driven
/// from the QSO state machine on its own schedule, not from an
/// enumeration callback.
fn handle_tx_connected(addr: u8, iface_num: u8) {
    log::info!("uac: TX_CONNECTED addr={addr} iface={iface_num} — probe start (silence only)");
    TX_STAGE.store(1, Ordering::Relaxed);
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
    TX_STAGE.store(2, Ordering::Relaxed);

    // **Try several, because the first one is known to be refused and
    // the reason is a size limit, not a format mismatch.**
    //
    // 2026-09-20, measured: `(2, 16, 48000)` — which *does* match the
    // radio's own OUT descriptor, `iface 1 alt 1` — returns
    // `ESP_ERR_NOT_SUPPORTED` (0x106). That error does not come from
    // the UAC driver at all. `hcd_pipe_alloc` rejects the pipe in
    // `pipe_alloc_hcd_support_verification`: on an ESP32-S3 there is
    // no HS PHY, so `otg_dfifo_depth` is 256 lines, the BALANCED FIFO
    // bias gives `ptx_fifo_lines = 256/8 = 32`, and the periodic-OUT
    // MPS limit is therefore `32 * 4` = **128 bytes**. Alt 1 carries
    // 192 bytes per frame and does not fit.
    //
    // The other way out is `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT`,
    // which raises that limit to ~824 B — and takes the RX FIFO from
    // 160 lines to 34. This board's receive path is the one that
    // already lost 2.6-6.5 % of its audio to an isochronous URB budget
    // (see `embedded-poc/CLAUDE.md`), so spending the RX FIFO to gain
    // a transmit format is the wrong trade.
    //
    // Mono is: 48 kHz x 1 ch x 2 B = **96 B/frame**, inside the limit,
    // and the radio lists three OUT alts at 96. FT8 transmit is mono
    // anyway — `write_ft8_frame` duplicates L = R today.
    //
    // The ladder runs to the end rather than stopping at mono so that
    // one flash reports which formats this radio and this FIFO admit,
    // instead of answering one question per hardware session.
    const TX_CONFIGS: [(u8, u8, u32); 4] = [
        (2, 16, 48_000), // 192 B/frame — over the 128 B limit, kept as the control
        (1, 16, 48_000), // 96 B/frame
        (2, 16, 24_000), // 96 B/frame
        (1, 16, 24_000), // 48 B/frame
    ];
    let mut err = sys::ESP_FAIL;
    let mut chosen = -1i32;
    for (idx, (ch, bits, freq)) in TX_CONFIGS.iter().enumerate() {
        let stream_config = sys::uac::uac_host_stream_config_t {
            channels: *ch,
            bit_resolution: *bits,
            sample_freq: *freq,
            flags: 0,
        };
        err = unsafe { sys::uac::uac_host_device_start(handle, &stream_config as *const _) };
        log::warn!("uac: tx start {ch}ch/{bits}b/{freq}Hz -> {err:#x}");
        if err == sys::ESP_OK as sys::esp_err_t {
            chosen = idx as i32;
            break;
        }
    }
    TX_CFG_IDX.store(chosen, Ordering::Relaxed);
    TX_START_ERR.store(err, Ordering::Relaxed);
    if err != sys::ESP_OK as sys::esp_err_t {
        // The config is no longer a guess: the 2026-09-20 enumeration
        // dump shows the OUT interface (PCM2901, 08bb:2901, iface 1)
        // carrying `alt 1 ... maxpkt 192`, and 192 B per 1 ms frame is
        // 48 samples x 2 ch x 2 B — 48 kHz stereo 16-bit, exactly what
        // `STREAM_*` says. A failure here is therefore about something
        // other than the format.
        //
        // Data only in the line: spelled out it measured 258 bytes
        // against the fanout's 160.
        log::error!("uac: tx probe device_start failed err={err:#x} (see enumeration dump)");
        let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
        if close_err != sys::ESP_OK as sys::esp_err_t {
            log::error!("uac: tx probe device_close after start failure failed err={close_err:#x}");
        }
        return;
    }

    TX_STAGE.store(3, Ordering::Relaxed);

    // Heap, and one buffer rather than a fresh copy per iteration: two
    // 1 920 B arrays live at once on a task stack that also has to hold
    // `write_ft8_frame` below.
    let mut silence = vec![0u8; TX_PROBE_CHUNK_BYTES];
    let mut wrote_ok = 0u32;
    for i in 0..TX_PROBE_WRITES {
        let err = unsafe {
            sys::uac::uac_host_device_write(
                handle,
                silence.as_mut_ptr(),
                silence.len() as u32,
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
    TX_SILENT_OK.store(wrote_ok, Ordering::Relaxed);
    TX_STAGE.store(4, Ordering::Relaxed);
    log::info!("uac: tx probe wrote {wrote_ok}/{TX_PROBE_WRITES} silent chunks OK");

    // **The whole transmit chain, at whatever amplitude is configured
    // — which is zero unless someone set it.**
    //
    // Twenty 10 ms writes prove the endpoint accepts data. They do not
    // prove a transmission: a frame is 632 consecutive chunks over
    // 12.64 s, paced by the ring's own backpressure, with `GfskStream`
    // producing each one 439 us before it is needed. Whether that
    // holds for twelve seconds is a different question from whether
    // twenty writes succeed, and it is the question a QSO depends on.
    //
    // At `TX_AMPLITUDE = 0` every part of that runs and the radio
    // hears silence — see that constant for what a nonzero value
    // means and what has to be true first.
    if wrote_ok > 0 {
        let msg77 = match mfsk_core::msg::wsjt77::pack77(
            "CQ",
            crate::decode_pipeline::MY_CALL,
            crate::decode_pipeline::MY_GRID,
        ) {
            Some(m) => m,
            None => {
                log::warn!(
                    "uac: tx frame skipped — pack77 failed for CQ {} {}",
                    crate::decode_pipeline::MY_CALL,
                    crate::decode_pipeline::MY_GRID
                );
                [0u8; 77]
            }
        };
        let channels = if chosen >= 0 {
            TX_CONFIGS[chosen as usize].0
        } else {
            STREAM_CHANNELS
        };
        let (chunks, worst_us, total_us) =
            write_ft8_frame(handle, &msg77, 1_500.0, TX_AMPLITUDE, channels);
        TX_FRAME_CHUNKS.store(chunks, Ordering::Relaxed);
        TX_FRAME_MS.store((total_us / 1_000) as i32, Ordering::Relaxed);
        TX_STAGE.store(5, Ordering::Relaxed);
        // The frame is 12 640 ms of audio in 632 chunks of 20 ms. A
        // `total` short of that is the ring not applying backpressure
        // (the writes are being accepted faster than the endpoint
        // drains, so the timing proves nothing); a `total` over it is
        // this board failing to keep up, and `worst` says which chunk.
        //
        // **Data only in the line.** Spelled out, this measured 213
        // bytes against the fanout's 160 (`log_sink::LINE_MAX`) and
        // arrived with a `~` — the numbers survived, the sentence
        // explaining them did not, which is the wrong half to lose to
        // a clip and the wrong place for it anyway.
        log::warn!(
            "uac: tx frame {} {chunks}ch worst {worst_us}us total {}ms / 12640ms",
            if TX_AMPLITUDE == 0 {
                "SILENT"
            } else {
                "LIVE-RF"
            },
            total_us / 1_000,
        );
    }

    let stop_err = unsafe { sys::uac::uac_host_device_stop(handle) };
    let close_err = unsafe { sys::uac::uac_host_device_close(handle) };
    if stop_err != sys::ESP_OK as sys::esp_err_t {
        log::error!("uac: tx probe device_stop failed err={stop_err:#x}");
    }
    if close_err != sys::ESP_OK as sys::esp_err_t {
        log::error!("uac: tx probe device_close failed err={close_err:#x}");
    }
    TX_STAGE.store(6, Ordering::Relaxed);
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
    // Completion time of the previous read, for `READ_GAP_MAX_US`.
    let mut last_read_done: i64 = 0;
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
            //
            // **The largest block goes beside it, because the free
            // total has been the misleading number.** Measured over
            // 363 samples of a 44-minute run (2026-09-20), the
            // largest contiguous block is a median of 51 % of the
            // free total and a quarter of samples are under 43 %; at
            // the run's low points it was 448-512 B while `int` still
            // read 1.4 kB. A DMA buffer has to fit in one block, so
            // reading `int` alone overstates the headroom by about
            // 2x.
            let (free_internal, largest_internal) = unsafe {
                let caps = sys::MALLOC_CAP_INTERNAL | sys::MALLOC_CAP_8BIT;
                (
                    sys::heap_caps_get_free_size(caps),
                    sys::heap_caps_get_largest_free_block(caps),
                )
            };
            // `bps` is bytes in this interval, and the interval is
            // only *at least* a second — so it is printed, or a slow
            // tick reads as lost audio.
            let int_ms = now.duration_since(last_log).as_millis();
            let blk_max = SINK_BLOCK_MAX_US.swap(0, Ordering::Relaxed);
            let blk_sum = SINK_BLOCK_SUM_US.swap(0, Ordering::Relaxed);
            let gap_max = READ_GAP_MAX_US.swap(0, Ordering::Relaxed);
            let to = READ_TIMEOUTS.swap(0, Ordering::Relaxed);
            log::info!(
                "uac: rx tick: {bps} B/{int_ms}ms ({packets} pkt / {errors} err) \
                 | audio {out_samples} sa/s, rms {dbfs:.1} dBFS, peak {peak}, clip {clipped} \
                 | blk {blk_max}/{blk_sum}us gap {gap_max}us to {to} \
                 | int={free_internal} lrg={largest_internal}",
            );
            let _ = bytes;
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
        let t_read_start = unsafe { sys::esp_timer_get_time() };
        if last_read_done > 0 {
            let gap = (t_read_start - last_read_done).clamp(0, u32::MAX as i64) as u32;
            READ_GAP_MAX_US.fetch_max(gap, Ordering::Relaxed);
            if gap > READ_GAP_SLOT_MAX_US.load(Ordering::Relaxed) {
                READ_GAP_SLOT_MAX_US.store(gap, Ordering::Relaxed);
                let at = mfsk_app_shared::time_sync::utc_now_ms()
                    .map_or(u32::MAX, |ms| (ms % (SLOT_SECS as u64 * 1_000)) as u32);
                READ_GAP_SLOT_MAX_AT_MS.store(at, Ordering::Relaxed);
            }
        }
        let t_read = unsafe { sys::esp_timer_get_time() };
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
                READ_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
                last_read_done = unsafe { sys::esp_timer_get_time() };
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
        last_read_done = unsafe { sys::esp_timer_get_time() };
        {
            let waited = (last_read_done - t_read).clamp(0, u32::MAX as i64) as u32;
            READS_IN_SLOT.fetch_add(1, Ordering::Relaxed);
            if waited > READ_WAIT_MAX_US.load(Ordering::Relaxed) {
                READ_WAIT_MAX_US.store(waited, Ordering::Relaxed);
                let at = mfsk_app_shared::time_sync::utc_now_ms()
                    .map_or(u32::MAX, |ms| (ms % (SLOT_SECS as u64 * 1_000)) as u32);
                READ_WAIT_MAX_AT_MS.store(at, Ordering::Relaxed);
            }
            if waited > READ_WAIT_WARN_US {
                READ_WAITS_OVER.fetch_add(1, Ordering::Relaxed);
            }
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
                // Never blocks — see `waterfall_feed::push`.
                crate::waterfall_feed::push(&dst_scratch[..produced]);
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
                // Recorded for whoever opens the OUT side later
                // (`tx_bringup`); opening it here would block the
                // `RxConnected` queued behind this event.
                TX_IFACE.store(((addr as u32) << 8) | iface_num as u32, Ordering::Release);
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
    spawn_psram_thread(c"uac_app", APP_TASK_STACK, None, None, move || app_task(rx))
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

    if let Err(e) =
        spawn_psram_thread(
            c"usb_events",
            USB_EVENTS_TASK_STACK,
            // Part of the audio path: it is what dispatches the host
            // library's events, so it answers to
            // `UAC_DRIVER_TASK_PRIORITY`'s argument, not to a default.
            Some(AUDIO_TASK_PRIORITY),
            None,
            usb_events_task,
        )
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
/// No-op client callback. The dump opens devices to read descriptors
/// and never submits a transfer, so it has no events to handle — but
/// `usb_host_client_register` requires a function pointer.
extern "C" fn dump_client_cb(
    _event: *const sys::usb_host_client_event_msg_t,
    _arg: *mut core::ffi::c_void,
) {
}

/// Print every enumerated device and every interface it offers.
///
/// **Written because the comments around `uac_host_device_start`
/// already told the reader to "check the device descriptor dump in the
/// UDP log", and there was no such dump.** The OUT interface's
/// channels/bits/rate have been a guess since the probe was written,
/// and whether the radio offers an audio OUT interface at all is
/// unestablished — both are answerable straight from the descriptors.
///
/// An IC-705 on USB should show a CDC/serial function for CI-V and an
/// audio function with a streaming interface each way, which is what
/// `num_devices=3` has been hinting at without saying.
///
/// Registers its own client rather than borrowing the UAC driver's:
/// the host stack allows several, and a reader that owns nothing
/// cannot disturb the capture that is running.
fn dump_enumeration() {
    let cfg = sys::usb_host_client_config_t {
        is_synchronous: false,
        max_num_event_msg: 5,
        __bindgen_anon_1: sys::usb_host_client_config_t__bindgen_ty_1 {
            async_: sys::usb_host_client_config_t__bindgen_ty_1__bindgen_ty_1 {
                client_event_callback: Some(dump_client_cb),
                callback_arg: core::ptr::null_mut(),
            },
        },
    };
    let mut client: sys::usb_host_client_handle_t = core::ptr::null_mut();
    // SAFETY: called after the host library is installed; `cfg` lives
    // for the duration of the call.
    let err = unsafe { sys::usb_host_client_register(&cfg, &mut client) };
    if err != sys::ESP_OK {
        log::warn!("uac: enumeration dump — client_register failed err={err:#x}");
        return;
    }

    let mut addrs = [0u8; 8];
    let mut n: core::ffi::c_int = 0;
    // SAFETY: `addrs` is `len` bytes and `n` is written by the callee.
    let err = unsafe {
        sys::usb_host_device_addr_list_fill(addrs.len() as i32, addrs.as_mut_ptr(), &mut n)
    };
    if err != sys::ESP_OK {
        log::warn!("uac: enumeration dump — addr_list_fill failed err={err:#x}");
        unsafe { sys::usb_host_client_deregister(client) };
        return;
    }
    log::warn!("uac: enumeration dump — {n} device(s)");

    for &addr in addrs.iter().take(n.max(0) as usize) {
        let mut dev: sys::usb_device_handle_t = core::ptr::null_mut();
        // SAFETY: `client` is registered; `dev` is written on success.
        if unsafe { sys::usb_host_device_open(client, addr, &mut dev) } != sys::ESP_OK {
            log::warn!("uac:   addr {addr}: open failed");
            continue;
        }
        let mut dd: *const sys::usb_device_desc_t = core::ptr::null();
        // SAFETY: `dev` is open; the pointer returned is owned by the
        // stack and valid until the device is closed.
        if unsafe { sys::usb_host_get_device_descriptor(dev, &mut dd) } == sys::ESP_OK
            && !dd.is_null()
        {
            // The bindings wrap the packed body in an anonymous
            // union, and it is `#[repr(packed)]`, so every field is
            // copied out rather than referenced — a reference into a
            // packed struct is UB even unread.
            //
            // SAFETY: the union has a single variant in these
            // bindings and the stack filled the descriptor.
            let (vid, pid, cls, sub, proto, ncfg) = unsafe {
                let d = core::ptr::addr_of!((*dd).__bindgen_anon_1);
                (
                    core::ptr::addr_of!((*d).idVendor).read_unaligned(),
                    core::ptr::addr_of!((*d).idProduct).read_unaligned(),
                    core::ptr::addr_of!((*d).bDeviceClass).read_unaligned(),
                    core::ptr::addr_of!((*d).bDeviceSubClass).read_unaligned(),
                    core::ptr::addr_of!((*d).bDeviceProtocol).read_unaligned(),
                    core::ptr::addr_of!((*d).bNumConfigurations).read_unaligned(),
                )
            };
            log::warn!(
                "uac:   addr {addr}: VID {vid:04x} PID {pid:04x} class {cls}/{sub}/{proto} \
                 configs {ncfg}"
            );
        }
        let mut cd: *const sys::usb_config_desc_t = core::ptr::null();
        // SAFETY: as above.
        if unsafe { sys::usb_host_get_active_config_descriptor(dev, &mut cd) } == sys::ESP_OK
            && !cd.is_null()
        {
            // Same anonymous-union wrapping as the device descriptor.
            // SAFETY: as above.
            let total = unsafe {
                core::ptr::addr_of!((*cd).__bindgen_anon_1.wTotalLength).read_unaligned()
            } as usize;
            // SAFETY: the stack guarantees `wTotalLength` bytes behind
            // the descriptor; walked read-only.
            let bytes = unsafe { core::slice::from_raw_parts(cd as *const u8, total) };
            walk_config(bytes);
        }
        // SAFETY: opened above by this client.
        unsafe { sys::usb_host_device_close(client, dev) };
    }
    // SAFETY: registered above, and every device it opened is closed.
    unsafe { sys::usb_host_client_deregister(client) };
}

/// Walk a configuration descriptor's TLV chain, printing interfaces
/// and endpoints.
///
/// Manual rather than through a helper because what matters is the
/// *audio streaming* interfaces and their directions, and a generic
/// pretty-printer buries those in the class-specific records the UAC
/// spec puts between them.
fn walk_config(bytes: &[u8]) {
    use core::fmt::Write as _;
    const T_INTERFACE: u8 = 0x04;
    const T_ENDPOINT: u8 = 0x05;
    const T_CS_INTERFACE: u8 = 0x24;
    const CS_FORMAT_TYPE: u8 = 0x02;
    let mut i = 0usize;
    while i + 2 <= bytes.len() {
        let len = bytes[i] as usize;
        if len < 2 || i + len > bytes.len() {
            break;
        }
        match bytes[i + 1] {
            T_INTERFACE if len >= 9 => {
                let (num, alt, neps) = (bytes[i + 2], bytes[i + 3], bytes[i + 4]);
                let (cls, sub, proto) = (bytes[i + 5], bytes[i + 6], bytes[i + 7]);
                let name = match (cls, sub) {
                    (0x01, 0x01) => " (audio control)",
                    (0x01, 0x02) => " (AUDIO STREAMING)",
                    (0x02, _) => " (CDC control — CI-V)",
                    (0x0a, _) => " (CDC data — CI-V)",
                    _ => "",
                };
                log::warn!(
                    "uac:     iface {num} alt {alt}: class {cls:#04x}/{sub:#04x}/{proto:#04x} \
                     {neps} endpoint(s){name}"
                );
            }
            T_ENDPOINT if len >= 7 => {
                let addr = bytes[i + 2];
                let attr = bytes[i + 3];
                let mps = u16::from_le_bytes([bytes[i + 4], bytes[i + 5]]);
                log::warn!(
                    "uac:       ep {:#04x} {} {} maxpkt {mps} ivl {}",
                    addr,
                    if addr & 0x80 != 0 { "IN " } else { "OUT" },
                    match attr & 0x03 {
                        0 => "control",
                        1 => "isochronous",
                        2 => "bulk",
                        _ => "interrupt",
                    },
                    // `bInterval == 0` on an ISOC/INTR endpoint is its
                    // own rejection in `pipe_alloc_hcd_support_verification`,
                    // separate from the MPS limit, so print it beside
                    // the size rather than inferring it later.
                    if len >= 7 { bytes[i + 6] } else { 0 },
                );
            }
            // Class-specific AS interface descriptor. The Type I
            // Format descriptor is what `uac_host_device_start`
            // matches `channels` / `bit_resolution` / `sample_freq`
            // against, so without it a `start` failure cannot be told
            // from a format this radio does not offer. The endpoint
            // sizes alone were not enough on 2026-09-20.
            T_CS_INTERFACE if len >= 8 && bytes[i + 2] == CS_FORMAT_TYPE => {
                let (ftype, nch, subframe, bits, nfreq) =
                    (bytes[i + 3], bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]);
                let mut f: heapless::String<48> = heapless::String::new();
                if nfreq == 0 {
                    // Continuous: tSamFreq[0] is lower, [1] is upper.
                    for k in 0..2 {
                        let o = i + 8 + k * 3;
                        if o + 3 <= i + len {
                            let hz = u32::from(bytes[o])
                                | (u32::from(bytes[o + 1]) << 8)
                                | (u32::from(bytes[o + 2]) << 16);
                            let _ = write!(&mut f, "{}{hz}", if k == 0 { "" } else { ".." });
                        }
                    }
                } else {
                    for k in 0..usize::from(nfreq).min(4) {
                        let o = i + 8 + k * 3;
                        if o + 3 <= i + len {
                            let hz = u32::from(bytes[o])
                                | (u32::from(bytes[o + 1]) << 8)
                                | (u32::from(bytes[o + 2]) << 16);
                            let _ = write!(&mut f, "{}{hz}", if k == 0 { "" } else { "," });
                        }
                    }
                }
                log::warn!(
                    "uac:       fmt type {ftype} {nch}ch {bits}b (sub {subframe}) freq {}                      [{}]",
                    if nfreq == 0 { "cont" } else { "list" },
                    f.as_str(),
                );
            }
            _ => {}
        }
        i += len;
    }
}

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
                    // The latched interface numbers ride along: they
                    // answer "did an audio IN / OUT interface ever
                    // appear" from any point in the log, which the
                    // one-shot enumeration lines cannot when the
                    // network that carries them is not up yet.
                    let (rx, tx) = (
                        RX_IFACE_SEEN.load(Ordering::Relaxed),
                        TX_IFACE_SEEN.load(Ordering::Relaxed),
                    );
                    let fmt = |v: i32| {
                        if v < 0 {
                            String::from("never")
                        } else {
                            format!("iface {v}")
                        }
                    };
                    // **Dump the descriptors the first time the device
                    // count settles**, not at install: enumeration
                    // finishes after `start_host` returns, and on
                    // 2026-09-20 the network that carries the log was
                    // 33 s behind it. Printed from this loop the dump
                    // lands wherever the log is actually reaching.
                    // **Three times, not once.** The first attempt at
                    // this was a one-shot on the first device sighting
                    // and it landed in the same hole everything else
                    // did: the loop starts within a second of boot,
                    // WiFi associated 33 s later, and the dump was
                    // written to a sink with nowhere to send it. The
                    // periodic lines are ~12 s apart, so three of them
                    // outlast any plausible association delay, and a
                    // descriptor dump repeated twice costs a few log
                    // lines against a whole reflash cycle.
                    if info.num_devices > 0 && DUMPED.fetch_add(1, Ordering::Relaxed) < 3 {
                        dump_enumeration();
                    }
                    let start = TX_START_ERR.load(Ordering::Relaxed);
                    let stage = TX_STAGE.load(Ordering::Relaxed);
                    // **Terse on purpose.** This line carries the whole
                    // TX answer and the fanout clips at
                    // `log_sink::LINE_MAX` (160) with a `~`; the
                    // spelled-out success case measured 161 bytes,
                    // i.e. the one outcome worth reading would have
                    // arrived as `... in 1264~`. Same trap the
                    // `SLOT[...]` line was already split for.
                    let tx_state = if stage == 0 {
                        String::from("never entered")
                    } else if stage == 1 {
                        String::from("open !ok")
                    } else if start == i32::MIN {
                        String::from("open ok, start pending")
                    } else if start != sys::ESP_OK {
                        format!("start err {start:#x}")
                    } else {
                        format!(
                            "cfg{} st{stage} sil {}/{} frame {}ch {}ms",
                            TX_CFG_IDX.load(Ordering::Relaxed),
                            TX_SILENT_OK.load(Ordering::Relaxed),
                            TX_PROBE_WRITES,
                            TX_FRAME_CHUNKS.load(Ordering::Relaxed),
                            TX_FRAME_MS.load(Ordering::Relaxed),
                        )
                    };
                    log::info!(
                        "uac: usb_host_lib_info — num_devices={} num_clients={} | audio IN {} \
                         | audio OUT {} | tx {}",
                        info.num_devices,
                        info.num_clients,
                        fmt(rx),
                        fmt(tx),
                        tx_state,
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
/// Interface number of the last `RxConnected` / `TxConnected` the
/// driver raised, or `-1` if it never has. See the latch in
/// `driver_event_cb`: the enumeration log can be written before the
/// network that carries it exists, and a missing line then reads as a
/// missing interface.
/// Last `uac_host_device_start` result on the OUT interface, silent
/// writes that succeeded, and frame chunks written — latched for the
/// same reason the interface numbers are. `handle_tx_connected` runs
/// during enumeration, which on this board is tens of seconds before
/// the network that carries its log.
/// How far [`handle_tx_connected`] got: 0 never entered, 1 entered,
/// 2 opened, 3 started, 4 silent writes done, 5 frame done, 6 closed.
///
/// Without this, `TX_START_ERR`'s initial value means three different
/// things at once — the handler was never called, `device_open`
/// failed, or the 12.6 s frame is still running — and telling them
/// apart costs a whole reflash cycle. It is monotonic and one word,
/// so reading it late is as good as watching it.
static TX_STAGE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Index into `TX_CONFIGS` that `device_start` accepted, or `-1`.
static TX_CFG_IDX: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(-1);

static TX_START_ERR: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(i32::MIN);
static TX_SILENT_OK: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static TX_FRAME_CHUNKS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static TX_FRAME_MS: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(-1);

/// How many times [`dump_enumeration`] has run. Repeated rather than
/// one-shot so it cannot be swallowed by a log path that is not up
/// yet — see the call site.
static DUMPED: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

pub(crate) static RX_IFACE_SEEN: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(-1);
/// See [`RX_IFACE_SEEN`]. **`-1` here is the answer to "does the
/// IC-705 offer a USB audio OUT interface at all"**, which nothing has
/// established yet.
static TX_IFACE_SEEN: core::sync::atomic::AtomicI32 = core::sync::atomic::AtomicI32::new(-1);

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
        grid_src: crate::grid_source(),
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
///
/// **8 s, not 45.** The radio is not recognised until the host stack is
/// installed, so this wait is time the operator spends looking at a
/// board that has not noticed the IC-705 — reported from the bench as
/// exactly that. Measured on this network (2026-09-19): the sink comes
/// up 4.75-5.25 s after boot normally, and 24.5 s when the association
/// is cold, so 8 s keeps the usual case unchanged (WiFi still wins the
/// race, and its DMA buffers are still allocated before the host
/// stack's) while a slow association no longer costs the radio twenty
/// seconds.
///
/// What the wait was protecting is smaller than it looks: the one line
/// that matters, what `start_host` concluded, is kept in
/// [`HOST_RESULT`] and re-emitted by the panel once a sink exists,
/// whenever that is. The rest is the IDF's own enumeration chatter at
/// debug level.
const LOG_SINK_WAIT_MS: u32 = 8_000;

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
    // Two questions, and both have to be yes: does this receiver want
    // the wait (only the FT8 controller does), and is a sink actually
    // coming (`net::bring_up` knows, after the last thing that could
    // stop it has not happened).
    let sink_expected = crate::wait_for_log_sink() && crate::wifi_enabled_for_this_boot();
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
    // `uac-host` at DEBUG: in the streaming path it logs only on loss —
    // "Bad RX Isoc packet", "RX Ringbuffer overflow", "push failed" —
    // each silently dropping audio otherwise (`stream_rx_xfer_done`).
    // Its other DEBUG lines are one-off descriptor details at open. The
    // line is printed from the class-driver task before it resubmits,
    // so a burst of them costs time on the path being measured; one
    // line per dropped packet is the price of knowing. 2026-09-22.
    unsafe {
        esp_idf_svc::sys::esp_log_level_set(
            c"uac-host".as_ptr(),
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

    let started = start_host();
    if let Err(e) = &started {
        log::error!("UAC host start failed: {e:#}");
        let mut msg: heapless::String<96> = heapless::String::new();
        {
            use core::fmt::Write as _;
            let _ = write!(&mut msg, "start_host FAILED: {e:#}");
        }
        HOST_RESULT.store(msg.as_str());
    }
    if started.is_ok() {
        crate::civ_usb::start();
        crate::tx_bringup::start();
    }
    crate::log_free_internal("post-uac-host-install");
}
