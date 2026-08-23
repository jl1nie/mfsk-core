//! CoreS3 WSPR receiver app — step 7 of the CoreS3 WSPR app plan
//! (see memory `project_wspr_app_cores3_ui`, plan file
//! `~/.claude/plans/mighty-chasing-castle.md`).
//!
//! Boot sequence: NVS settings → spawn the scan task (reserving its
//! stack before anything else can fragment internal DRAM, see the
//! comment at its call site, core 0) → spawn the DDC + display tasks
//! (core 1) → release `SCAN_GO` immediately → spawn the network task
//! (WiFi STA → UDP log sink → NTP → HTTP config server, all in the
//! background, core 1) → `main` itself becomes an idle supervisor
//! loop. **Decode start no longer waits on WiFi at all** (2026-08-15)
//! — see `spawn_network_task`'s own doc comment for why: this app's
//! test AP needs unbounded retry to reliably connect, so blocking
//! boot on it would mean an unbounded hang, not a bounded delay.
//!
//! **Display runs on its own task, pinned to core 1 — not inline in
//! `main`.** `CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0=y` (this crate's
//! sdkconfig) pins `main` to core 0, the same core the scan task is
//! pinned to; the scan task runs at FreeRTOS priority 5 and is
//! compute-bound for 90-110 s at a stretch with no yield point (that's
//! *why* the watchdog gets disabled). A first version of this app ran
//! the render loop inline in `main` and was starved almost the entire
//! 2-minute cycle as a result — real-hardware observation (the LCD
//! looked frozen most of the time), not a theoretical concern. Moving
//! the render loop to its own core-1 task fixes it: `wspr_dual_core`'s
//! worker also lands on core 1 (`APP_CPU = 1`), but only for the
//! fraction of each pass spent in `pass01_split`/`pass2_split`, not
//! the whole scan — far less contention than sharing a core with the
//! scan task itself.
//!
//! A **new binary**, not folded into `main.rs`: WSPR never touches
//! the FT8 controller's `decode_block`/UAC stack, and this app's own
//! task-stack placement constraints (see below) are unrelated to that
//! binary's.
//!
//! **Live audio capture wiring landed 2026-08-15, not yet
//! hardware-verified** (issue #163 — needs a real IC-705 or other UAC
//! source connected to CoreS3's USB-A port to confirm end-to-end).
//! `uac.rs` (shared with the FT8 controller's `main.rs`, via its new
//! `AudioSink` trait — see that file's own doc comment) installs the
//! USB host + UAC class driver and, once a device enumerates and
//! starts streaming, resamples 48 kHz stereo down to 12 kHz mono and
//! pushes it into [`WsprDdcSink`], which feeds
//! `mfsk_core::wspr::ddc::StreamingDdcCascade` and rotates
//! [`BASEBAND_BUFS`] exactly the way `ddc_loop`'s synthetic generator
//! already did — same double-buffered producer/consumer handoff via
//! [`DDC_READY_IDX`], just driven by real USB arrival instead of a
//! self-paced synthetic loop.
//!
//! **Until a device actually connects (or if none ever does), `ddc_loop`
//! keeps generating the same synthetic burst as before** — real audio
//! only takes over `BASEBAND_BUFS` production once [`UAC_AUDIO_ACTIVE`]
//! flips true on the first real sample `WsprDdcSink` receives; see
//! that flag's own doc comment for the handoff mechanics. This keeps
//! the app fully functional (and host-independently testable) with no
//! USB device attached, exactly as it was before this session.
//!
//! **No wall-clock slot alignment yet** for the real-audio path — same
//! open item `uac.rs`'s FT8 `reader_thread` already carries for its
//! own `SLOT_SAMPLES_12K`: the WSPR slot boundary is bound by raw
//! sample count from whenever the UAC stream started, not UTC
//! :00/:02/:04 marks. Needs the NTP-fed `time_sync` hook, same as the
//! FT8 side's own open item.
//!
//! **The one slot this doesn't apply to**: the very first, which
//! still decodes the baked WAV-derived golden baseband
//! (`assets/wspr_golden_baseband.bin`, same as `wspr_bench.rs`) so
//! the app has *some* meaningful decode to show immediately at boot
//! rather than waiting out a full DDC cycle first. Every slot after
//! the first goes through the real DDC pipeline — see `scan_loop`'s
//! own comment for exactly where the switch happens. **The synthetic
//! fallback source is a real, correctly-encoded WSPR test transmission
//! (`K1ABC`), not noise and not a bare tone** — see
//! [`DdcTestSource`]'s doc comment for the false-decode/missing-load/
//! OOM bugs (2026-08-15) that earlier placeholder choices caused on
//! real hardware, and why "a genuine signal, streamed incrementally"
//! is what avoids all three.
//!
//! **UDP log fanout** (added 2026-08-15, ahead of wiring in real UAC
//! capture — see this file's own `DdcTestSource` history for why that
//! swap hasn't landed yet): once USB-OTG host mode claims the
//! USB-Serial-JTAG endpoint for live audio, the serial console goes
//! with it — same reason `main.rs` (the FT8 controller) installs
//! `mfsk_app_shared::log_sink::FanoutLogger` instead of the plain
//! `EspLogger`. This bin now does the same: `LOGGER.install()` routes
//! every `log::info!`/`log::warn!`/etc. through console (guarded by
//! `usb_serial_jtag_is_connected()`) *and* a UDP datagram sink once
//! WiFi is up, so `embedded-poc/scripts/udp-log-listen.sh` on the host
//! PC keeps showing logs even with no serial port to attach to. The
//! LCD-scroll-panel half of `LogFanout` is left unused here (nobody
//! calls `display::run_log_panel` — this bin's own `display_loop`
//! draws the WSPR UI instead), which is harmless: `FanoutLogger` still
//! pushes into it, just nothing ever reads it back out.
//!
//! One consequence worth being explicit about regardless: the band
//! selector in the web settings form changes what dial frequency gets
//! *reported* to wsprnet, not what actually gets decoded — there is
//! no live receiver behind it yet. Picking a band other than 20 m
//! makes every spot's `tqrg` field describe a signal that was not
//! really heard on that band.
#![allow(dead_code)] // board.rs/pmic.rs/uac.rs carry fields/fns this bin doesn't use (no touch, no TX).


use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use esp_idf_hal::delay::{Ets, FreeRtos};
use esp_idf_hal::gpio::{AnyIOPin, PinDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::{config::Config as SpiConfig, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use esp_idf_hal::units::FromValueType;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use display_interface_spi::SPIInterface;
use mipidsi::{
    models::ILI9342CRgb565,
    options::{ColorInversion, Orientation},
    Builder,
};

use mfsk_core::wspr::ddc::{StreamingDdcCascade, AUDIO_RATE_HZ};
use mfsk_core::wspr::decode::WsprResult;

use embedded_shared::apps::wspr_scan::{load_baseband, now_us, run_scan, NBB, SLOT_US};
use embedded_shared::wspr_dual_core;

use mfsk_app_shared::civil_time::civil_from_unix;
use mfsk_app_shared::settings::{self, Settings};
use mfsk_app_shared::boot_mode::{self, BootMode};
use mfsk_app_shared::ui::{link_bar, mode_picker};
use mfsk_app_shared::ui::wspr_list;
use mfsk_app_shared::ui::wspr_row::WsprSpotRow;
use mfsk_app_shared::ui::wspr_state::WSPR_UI;
use mfsk_app_shared::wspr_bands::{WsprBand, WSPR_BANDS};
use mfsk_app_shared::{http_config, ntp, udp_log};

const GOLDEN_BASEBAND: &[u8] = include_bytes!("../../../assets/wspr_golden_baseband.bin");



/// How long to wait for the boot-time NTP attempt before giving up
/// and running without absolute time (`ntp_synced` stays false, and
/// wsprnet reporting stays off for the whole run — see `scan_loop`).
/// 20 s is generous for `pool.ntp.org` over a home network; unmeasured
/// on real hardware, same caveat as this plan's other untuned
/// timeouts.
const NTP_SYNC_TIMEOUT_MS: u32 = 20_000;

/// Stack for the scan task. Same 72 KiB `wspr-bench` settled on after
/// measuring a 63 192 B peak through `run_scan`'s full pass 0/1/2
/// sequence (see that bin's own `SCAN_STACK` doc comment) — this app
/// calls the identical function, so the identical stack applies.
const SCAN_STACK: u32 = 72 * 1024;

/// Stack for the display task. `embedded_graphics`/`mipidsi` drawing
/// is shallow compared to the decode path; this is roughly what
/// `main`'s own 32 KiB stack (`CONFIG_ESP_MAIN_TASK_STACK_SIZE`) was
/// covering for the same code when it ran inline, not an independent
/// hardware measurement.
const DISPLAY_STACK: u32 = 24 * 1024;

/// Below the scan/dual-core-worker priority (5) so the display task
/// never delays a decode — on core 1 it only ever contends with
/// `wspr_dual_core`'s intermittent worker, and losing that contention
/// just means a render waits a few hundred ms, not that a spot gets
/// dropped.
const DISPLAY_PRIORITY: u32 = 3;

/// Internal-DRAM free/largest-contiguous-block snapshot, same shape
/// as `wspr-bench`'s own `log_heap` — evidence for the task-spawn
/// ordering this file's `main` doc comment argues for, not just a
/// "it didn't error this run" assumption.
fn log_heap(tag: &str) {
    let caps = esp_idf_svc::sys::MALLOC_CAP_INTERNAL | esp_idf_svc::sys::MALLOC_CAP_8BIT;
    unsafe {
        log::info!(
            "wspr_app heap {tag}: internal free {} KB (largest contig {} KB)",
            esp_idf_svc::sys::heap_caps_get_free_size(caps) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(caps) / 1024,
        );
    }
}

/// Set once WiFi/NTP/HTTP-server bring-up finishes; the scan task
/// blocks on this before its first `run_scan`. See the comment at
/// `spawn_scan_task`'s call site for why the task is *created* before
/// that bring-up even though it doesn't start working until after.
static SCAN_GO: AtomicBool = AtomicBool::new(false);

// ── DDC producer / decode consumer pipeline ─────────────────────────
//
// Two baseband buffers, each independently lockable, so the DDC task
// can write buffer B while the scan task still holds buffer A locked
// for its own ~80-100 s `run_scan` call. The `Mutex` *is* the
// correctness mechanism here, not just a formality: if `run_scan`
// ever took longer than one DDC cycle (~120 s), the DDC task would
// simply block on the lock rather than race it — natural backpressure
// instead of a buffer stomp. Measured decode times (~79-103 s) leave
// comfortable headroom under that, but the lock means the code doesn't
// have to *assume* that margin holds.

/// A `Vec` can't be heap-allocated in a `const` initializer, so the
/// two statics below start with empty buffers — harmless, since
/// `ddc_loop` always fully *replaces* `idat`/`qdat` (never appends or
/// assumes a prior size) the first time it claims either index, before
/// `scan_loop` ever reads it.
struct SlotBuf {
    idat: Vec<f32>,
    qdat: Vec<f32>,
}

impl SlotBuf {
    const fn new() -> Self {
        Self {
            idat: Vec::new(),
            qdat: Vec::new(),
        }
    }
}

static BASEBAND_BUFS: [Mutex<SlotBuf>; 2] =
    [Mutex::new(SlotBuf::new()), Mutex::new(SlotBuf::new())];

/// Index into [`BASEBAND_BUFS`] the DDC task most recently finished
/// writing, or `None` if nothing's ready yet. Set by `ddc_loop`,
/// taken (cleared) by `scan_loop` — a one-slot mailbox, not a queue:
/// if decode ever fell behind by more than one DDC cycle a second
/// `ddc_loop` write would just overwrite this before decode read it,
/// silently dropping a slot rather than queuing it. Not a concern at
/// today's measured timing (DDC ~20 s vs. decode ~80-100 s against a
/// shared ~120 s cadence).
///
/// **Known theoretical gap, not currently reachable**: if `run_scan`
/// ever took longer than *two* DDC cycles (~240 s — today's measured
/// 79-103 s leaves wide margin), `ddc_loop` could wrap back around and
/// start overwriting the very buffer `scan_loop` is still mid-decode
/// on (via `mem::take`'d local copies, so no data race — just a
/// discarded DDC cycle when `scan_loop` writes its own copy back
/// afterward). A generation counter would close this properly; not
/// added here because nothing in this pipeline can currently trigger
/// it, and a synthetic-source milestone isn't the place to design
/// defensively for a case the real timing doesn't produce.
static DDC_READY_IDX: Mutex<Option<usize>> = Mutex::new(None);

/// Set `true` on the first real sample [`WsprDdcSink`] ever receives
/// from `uac.rs`'s reader thread. `ddc_loop` checks this at the top of
/// every iteration and, once true, stops generating synthetic slots
/// forever — from that point on `WsprDdcSink::push_samples` (called
/// from the UAC reader's own thread context) is the sole
/// [`BASEBAND_BUFS`] writer. One-way switch by design: real audio
/// never stops arriving once a device is streaming (the reader thread
/// only exits on disconnect, at which point there is no more audio to
/// decode anyway — falling back to synthetic mid-session would just
/// silently swap what a station believes it's receiving).
static UAC_AUDIO_ACTIVE: AtomicBool = AtomicBool::new(false);

/// **History, three real-hardware bugs deep (2026-08-15):**
///
/// 1. First cut: a single fixed 1540 Hz tone (same one `wspr-bench`'s
///    own `ddc_load_task`/`bench_ddc` still use for their own
///    CPU-timing-only purposes, which never feed a real decoder).
///    Feeding that tone through the *real* `run_scan` here produced an
///    anomalous 72 coarse candidates (a real busy band doesn't produce
///    that many), and one of those occasionally converged through
///    Fano/OSD anyway — WSPR's convolutional code has no CRC-strength
///    check, so a sufficiently structured, non-noise-like input has a
///    real (if small) false-accept rate. Symptom: after running for a
///    while, the "discovered stations" pane started showing entries
///    like `<#00000> A000... 0` — not display corruption, that's
///    `msg/wspr.rs`'s own `<#{hash:05x}>` format for a genuine (but
///    meaningless) Type-3 decode of noise bits.
/// 2. Fixed by switching to pure per-chunk pseudorandom noise (see
///    [`fill_noise`]) — no false decodes in a 10-minute/5-slot
///    real-hardware verification. But pure noise also means a real
///    quiet band: 0 coarse candidates, decode finishing in ~1 s
///    instead of the ~80-100 s compute-bound Fano/OSD/subtract cost
///    real audio would produce — the concurrency/timing behaviour
///    this whole pipeline exists to validate (does the decode task
///    starve display/WiFi/HTTP, or vice versa) became invisible past
///    slot 0. Pointed out by the user: "デコードがないと負荷がわからない."
/// 3. Tried streaming a real, correctly-encoded WSPR burst instead —
///    first by building the *whole* 162-symbol waveform into one
///    ~5.3 MB `Vec` up front (`build_ddc_test_track`, since removed).
///    Two more real-hardware failures came out of that single attempt:
///    the placeholder callsign `"N0CALL"` doesn't actually fit WSPR's
///    compressed packing (6 chars with its digit at position 1 —
///    `msg::callsign28::pack_call28` only accepts that shape at ≤5
///    chars), so `pack_type1` returned `None` and the `.expect()`
///    aborted every boot; and even after switching to the
///    known-valid `K1ABC`/`FN42`/`37` (the exact combination
///    `mfsk_core::wspr::tx`'s own `synthesizes_valid_message` test
///    already proves packs), the 5.3 MB allocation itself failed
///    (`memory allocation of 5472256 bytes failed` → abort) every
///    boot — PSRAM has 8 MB nominal, but not 5.3 MB free/contiguous
///    at the moment `ddc_loop` first runs, concurrently with
///    `scan_loop`'s own slot-0 golden decode. **The `panicked`
///    string this crate's own logs are usually greped for doesn't
///    appear for this failure mode** — `handle_alloc_error` aborts
///    directly, not through the normal panic machinery — which is
///    why the first fix attempt looked clean in a `grep panicked`
///    pass and needed the user's own on-device observation ("history
///    に全くたまっていない") to catch.
///
/// Fixed for good by **streaming the burst incrementally**
/// ([`DdcTestSource`]) instead of ever materializing it — same
/// per-sample cosine-with-continuous-phase math
/// `synthesize_audio_into` uses, just computed one [`DDC_CHUNK_LEN`]
/// chunk at a time into the same small stack-sized buffer noise used
/// to fill, carrying only ~200 B of generator state (symbol table +
/// position + phase) between calls instead of the full waveform. This
/// keeps every property the previous two attempts were chasing at
/// once: a real signal decodes to a real, inspectable, always-correct
/// result (no false-decode risk), it exercises the full
/// coarse/Fano/OSD/subtract cost every DDC-fed slot same as the
/// golden-baseband decode, *and* it no longer competes with
/// `scan_loop`'s own concurrent PSRAM use for a multi-megabyte
/// allocation neither task actually needs.
const DDC_TEST_CALL: &str = "K1ABC";
const DDC_TEST_GRID: &str = "FN42";
const DDC_TEST_DBM: i32 = 37;
const DDC_TEST_BASE_FREQ_HZ: f32 = 1_500.0;
const DDC_TEST_AMPLITUDE: f32 = 0.3;
/// Amplitude for the noise surrounding [`DDC_TEST_CALL`]'s burst in
/// [`DdcTestSource`] (lead-in/trail pad) — same scale the original
/// fixed-tone constant used, now applied by [`noise_sample`] instead.
const DDC_NOISE_AMPLITUDE: f32 = 0.3;
/// Chunk size [`ddc_loop`] feeds to [`StreamingDdcCascade::push`].
const DDC_CHUNK_LEN: usize = 4096;
/// Silence-noise lead-in before the burst starts, within the slot's
/// capture window — gives `run_scan` something resembling a realistic
/// (if synthetic) `dt` rather than the burst landing exactly on
/// sample 0. 1 s at `AUDIO_RATE_HZ`.
const DDC_LEAD_PAD_SAMPLES: usize = 12_000;
/// One WSPR slot's worth of audio at `AUDIO_RATE_HZ`, matching
/// `wspr-bench`'s own `NPOINTS_MAX`-equivalent constant (114 s is the
/// capture portion of a 120 s slot).
const DDC_SLOT_AUDIO_SAMPLES: usize = 114 * 12_000;
/// Stack for the DDC task. It streams through a small fixed chunk
/// buffer, [`DdcTestSource`]'s own small state struct, and
/// `StreamingDdcCascade`'s own (now internal-DRAM-sized) FIR history/
/// taps for both stages — no decode-sized or multi-megabyte frames,
/// so this is deliberately much smaller than `SCAN_STACK`.
const DDC_STACK: u32 = 12 * 1024;

/// The whole WSPR receiver, given the resources rather than taking them.
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
    log::info!("=== mfsk-core-m5stack-cores3-app wspr-app boot ===");
    log::info!("mfsk-core {}", mfsk_core::VERSION);

    // The candidate loop runs compute-bound for tens of seconds at a
    // stretch with no yield point; IDLE0 starves and the task
    // watchdog fires. Same call `wspr-bench` makes, same reason —
    // this app pays the identical decode cost.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");
    // Take WSPR's worker stack before WiFi, while the heap is still
    // whole — after WiFi and the USB host the largest free internal
    // block is 31,744 B, and this needs far more. See
    // `embedded_shared::worker_arena` for the measurements.
    if !wspr_dual_core::reserve_arena() {
        log::error!("worker stack reservation failed — decoding will run single-core");
    }
    wspr_dual_core::init();

    let nvs = settings::open_nvs(nvs_part.clone()).expect("settings NVS open");
    let nvs = Arc::new(Mutex::new(nvs));

    // Spawn the scan task's 72 KiB stack **before anything else can
    // fragment internal DRAM** — WiFi is the case `wspr-bench` (issue
    // #260) measured (largest contiguous block 236 -> 184 KB *before
    // the radio even starts*, just from linking), but the display
    // task's own SPI-driver/mipidsi allocations are the same kind of
    // risk if they're left free to race this claim: a first version
    // of this fix spawned the display task first and hit exactly that
    // — `xTaskCreatePinnedToCore` for the scan stack failed on real
    // hardware because the display task's concurrent bring-up (on the
    // other core, but the same heap) was contending for the same
    // block. Scan first, unconditionally, before anything else on
    // either core gets a chance to allocate. The task itself blocks
    // on `SCAN_GO` until bring-up below finishes — this is "reserve
    // the memory now, start the work once settings/WiFi/NTP are
    // ready," not "decode before init is done."
    log_heap("pre-scan-spawn");
    spawn_scan_task(ScanCtx { nvs: nvs.clone() });
    log_heap("post-scan-spawn");

    // DDC producer task — small stack (12 KiB), spawned here rather
    // than after WiFi/display for the same "claim it before anything
    // else can fragment the heap" reasoning as the scan task, even
    // though its own footprint is far less likely to actually collide.
    spawn_ddc_task();
    log_heap("post-ddc-spawn");

    // Register the real-audio sink **before** `spawn_display_task`
    // below, whose task body eventually calls `crate::uac::start_host()`
    // (after PMIC/VBUS bring-up — see `display_loop`'s own comment).
    // Same "wire the consumer before installing the driver" ordering
    // `main.rs` relies on for its own `set_chunk_q` call: by the time
    // the display task's thread actually reaches `start_host()` (real
    // I2C/PMIC work first), this synchronous call here has long since
    // returned, so there is no race despite the two running on
    // different tasks.
    crate::uac::set_audio_sink(WsprDdcSink::new());

    // Display task: its own LCD bring-up + render loop, pinned to
    // core 1 — see this file's top doc comment for why inline-in-
    // `main` starved it against the scan task on real hardware.
    spawn_display_task(DisplayCtx {
        i2c0: peripherals.i2c0,
        spi2: peripherals.spi2,
        pins: peripherals.pins,
        nvs: nvs.clone(),
    });
    log_heap("post-display-spawn");

    // WiFi *driver construction* — still synchronous, still here,
    // still before `SCAN_GO`. **2026-08-15, real-hardware finding**:
    // this is the one piece of WiFi bring-up that genuinely can't be
    // backgrounded — `peripherals.modem` is consumed by value into
    // `EspWifi::new()`, and esp-idf-svc doesn't hand it back on `Err`,
    // so a failure here (real hardware hit exactly this once
    // `SCAN_GO` started firing before WiFi bring-up: the driver's own
    // rx-buffer allocation lost the internal-DRAM race against
    // `scan_loop`'s concurrent decode-time churn — `ESP_ERR_NO_MEM`,
    // unrecoverable, no modem left to retry with) is permanent for the
    // rest of this boot. Keeping it here, before `SCAN_GO`, claims
    // WiFi's internal-DRAM needs in the same safe window the scan/DDC/
    // display task stacks already do. See `wifi::wifi_driver_init`'s
    // own doc comment for the full explanation.
    //
    // What *is* backgrounded (`spawn_network_task` below): the
    // scan/associate/DHCP retry loop, which is the slow, potentially-
    // unbounded part (`wifi::connect_with_retry`, itself internally
    // safe to retry indefinitely against an already-alive driver) —
    // and everything downstream of a successful connect (UDP log
    // sink, NTP, HTTP config server).
    let sysloop = EspSystemEventLoop::take().expect("sysloop");
    let wifi_driver = if crate::WIFI_SSID.is_empty() {
        log::warn!(
            "wspr_app: WIFI_SSID empty (no cfg.toml) — NTP/HTTP config/wsprnet all unavailable"
        );
        None
    } else {
        match mfsk_app_shared::wifi::wifi_driver_init(peripherals.modem, sysloop, Some(nvs_part)) {
            Ok(w) => Some(w),
            Err(e) => {
                log::error!("wspr_app: WiFi driver init failed (unrecoverable this boot): {e:#}");
                None
            }
        }
    };
    log_heap("post-wifi-driver-init");

    // Network task: spawned here, **still before `SCAN_GO`**, for the
    // same reason as the WiFi driver above — its own 24 KiB task stack
    // needs internal DRAM too, and real hardware hit exactly this:
    // moving this spawn *after* `SCAN_GO` (a first cut of this fix)
    // made `xTaskCreatePinnedToCore` itself fail
    // (`failed to create wspr_net task`) once scan_loop's own
    // decode-time churn was already racing it for the same pool. The
    // task's own *work* (WiFi association/DHCP retry, NTP, HTTP) still
    // doesn't block `SCAN_GO` below — it starts running immediately
    // once spawned, same as scan/DDC/display always have, just with
    // its stack safely claimed first.
    if let Some(wifi_driver) = wifi_driver {
        spawn_network_task(NetworkCtx { wifi_driver, nvs });
    }
    log_heap("post-network-spawn");

    // Release scan/DDC immediately — the potentially-slow/unbounded
    // half of WiFi bring-up (association/DHCP), plus NTP/HTTP/UDP-log,
    // now happen in their own background task and must not gate decode
    // start. **2026-08-15**: this app's own test AP needs
    // `wifi::connect_with_retry`'s *unbounded* retry mode to reliably
    // connect at all (see that function's own doc comment — reproduces
    // even against `wifikey2`, an independent, previously-working
    // reference implementation, against this same AP), so blocking
    // boot on WiFi the way this file used to would mean an indefinite
    // hang on a bad day rather than a bounded delay.
    SCAN_GO.store(true, Ordering::Release);

    // `main` (pinned to core 0 by sdkconfig) has nothing left to do —
    // the scan task owns core 0's real work, the display and network
    // tasks run independently on core 1. An idle supervisor loop, same
    // shape as `wspr-bench`'s own `main` tail.
    loop {
        FreeRtos::delay_ms(1000);
    }
}

// ── Display task ──────────────────────────────────────────────────────

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
    // SAFETY: `spawn_display_task` leaked exactly this pointer via
    // `Box::into_raw`, and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut DisplayCtx) };
    display_loop(*ctx);
}

fn spawn_display_task(ctx: DisplayCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(display_task_entry),
            c"wspr_display".as_ptr(),
            DISPLAY_STACK,
            ptr,
            DISPLAY_PRIORITY,
            core::ptr::null_mut(),
            1, // core 1 — deliberately NOT the scan task's core 0, see
               // this file's top doc comment.
        )
    };
    if created != 1 {
        log::error!("wspr_app: failed to create wspr_display task");
    }
}

/// LCD bring-up (AXP2101 + AW9523B → SPI2 → mipidsi, mirrors
/// `display.rs`'s FT8-controller sequence minus the BootMode/UAC
/// branches this app has no use for) followed by the render loop.
/// Never returns.
fn display_loop(ctx: DisplayCtx) -> ! {
    // Kept across the whole loop: the touch controller shares this bus
    // and the mode picker is this receiver's only way back out.
    let mut touch_i2c: Option<esp_idf_hal::i2c::I2cDriver<'static>> = None;
    let mut display = match crate::pmic::init(ctx.i2c0, ctx.pins.gpio12, ctx.pins.gpio11) {
        Ok(mut i2c) => {
            // Enable USB VBUS boost **before** `crate::uac::start_host()` —
            // AW9523B P0_1 (BUS_OUT_EN) HIGH drives the VBUS switch;
            // omission leaves VBUS floating and the host stack sees no
            // device. Same call/ordering `display.rs`'s FT8-controller
            // sequence makes for `BootMode::Uac`, just unconditional
            // here since this app has no other boot mode to gate on.
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

            // Install USB host + UAC class driver. Detaches
            // USB-Serial-JTAG (the serial console) the moment this
            // returns — this is exactly why UDP log fanout was wired
            // ahead of this call landing (see this file's own
            // `LOGGER`/`FanoutLogger` doc comment). Unverified on real
            // hardware as of this writing (issue #163) — if no device
            // ever enumerates, `ddc_loop`'s synthetic generator just
            // keeps running (see [`UAC_AUDIO_ACTIVE`]'s own doc
            // comment), so this is safe to always attempt.
            if host_mode {
                log::info!("wspr_app: installing USB host + UAC class driver");
                if let Err(e) = crate::uac::start_host() {
                    log::error!("wspr_app: UAC host start failed: {e:#}");
                    let mut msg: heapless::String<96> = heapless::String::new();
                    {
                        use core::fmt::Write as _;
                        let _ = write!(&mut msg, "start_host FAILED: {e:#}");
                    }
                    crate::uac::HOST_RESULT.store(msg.as_str());
                }
            } else {
                log::info!(
                    "wspr_app: USB host not installed (peripheral mode) — the serial console \
                     stays up and audio falls back to the synthetic generator"
                );
            }

            let driver = SpiDriver::new(
                ctx.spi2,
                ctx.pins.gpio36, // SCK  (crate::board::LCD_PIN_SCK)
                ctx.pins.gpio37, // MOSI (crate::board::LCD_PIN_MOSI)
                Option::<AnyIOPin>::None,
                &SpiDriverConfig::new(),
            )
            .expect("SPI2 driver");
            let spi_cfg = SpiConfig::new().baudrate(20_u32.MHz().into());
            let spi_dev = SpiDeviceDriver::new(driver, Some(ctx.pins.gpio3), &spi_cfg) // CS (crate::board::LCD_PIN_CS)
                .expect("SPI device (CS=3)");
            let dc = PinDriver::output(ctx.pins.gpio35).expect("DC gpio35"); // crate::board::LCD_PIN_DC
            let di = SPIInterface::new(spi_dev, dc);

            // **2026-08-15 real-hardware fix**: this board's chip is
            // ILI9342C (see `board.rs`'s own doc comment), whose
            // native `FRAMEBUFFER_SIZE` in mipidsi is already
            // `(320, 240)` — landscape. The previous code used the
            // *ILI9341* model instead (`FRAMEBUFFER_SIZE = (240,
            // 320)`, portrait-native) plus a manual
            // `.orientation(Deg90)` to compensate — a Builder-time
            // rotation that, empirically, never correctly swapped
            // mipidsi's own internal width/height bookkeeping (every
            // symptom chased today — partial coverage, stripes, the
            // panel reporting 240×320 instead of 320×240 — traces
            // back to this).
            //
            // **Then, per a follow-up layout request, rotated back to
            // portrait on purpose**: `.orientation(Deg90)` on top of
            // this now-correct 320×240 landscape base gives a clean
            // 240×320 canvas — confirmed via `lcd_minimal.rs`'s
            // orientation-cycling diagnostic (`R90 NORMAL`, unmirrored)
            // on real hardware. Different from the old bug: this
            // rotation is layered on the *correct* native-landscape
            // model, not used to fake landscape out of a
            // portrait-native one, so it doesn't hit the same
            // bookkeeping issue.
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

    // The 2026-08-15 real-hardware investigation that led here (three
    // real bugs: AXP2101 DLDO1/backlight never enabled, board.rs's
    // LCD_RST/TP_RST bits swapped, and `DrawTarget::clear()` itself
    // giving partial coverage on this mipidsi/SPI setup — see
    // `pmic.rs`'s and `wspr_list::render_all`'s doc comments) used a
    // standalone `lcd-minimal` bin for the raw-driver / orientation
    // diagnostics rather than growing throwaway test code in this
    // file. `render_all` below is the real first paint.
    {
        let ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
        if let Err(e) = wspr_list::render_all(&mut display, &ui) {
            log::error!("wspr_app::display: render_all FAILED: {e:?}");
        }
    }

    // Status bar repaints every tick (cheap, one line); discovered/
    // history panes only repaint when `WsprUiState::dirty_seq`
    // actually changed, same gating `decoded_list`/`waterfall` use
    // for the FT8 UI. The lock is held across the SPI draw calls
    // rather than snapshotted out first (as the FT8 display loops
    // do) — contention is the scan task's once-per-slot `set_slot`/
    // `update_status` against this loop's 500 ms tick, which is rare
    // and brief enough that the simpler form was chosen over
    // threading a full `WsprUiState` snapshot type through for this
    // first pass.
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
            picker.render(&mut display, BootMode::Wspr).ok();
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
            let mut ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
            ui.free_heap_kb = heap_kb;
            ui.utc_hhmmss = hhmmss.clone();
            ui.dirty_seq()
        };
        {
            let ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
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
                        "[boot-summary] mode=WSPR host_mode={host_attempted} start_host: {}",
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
                wspr_list::PANEL_WIDTH,
                wspr_list::PANEL_HEIGHT as i32 - link_bar::HEIGHT as i32,
            )
            .ok();
            if let Err(e) = wspr_list::render_status(&mut display, &ui) {
                log::warn!("wspr_app::display: render_status failed: {e:?}");
            }
            if dirty != last_dirty {
                if let Err(e) = wspr_list::render_discovered(&mut display, &ui) {
                    log::warn!("wspr_app::display: render_discovered failed: {e:?}");
                }
                if let Err(e) = wspr_list::render_history(&mut display, &ui) {
                    log::warn!("wspr_app::display: render_history failed: {e:?}");
                }
                last_dirty = dirty;
            }
        }
        // ~10 s cadence. Proves this task keeps running (and the UTC
        // clock keeps advancing) through the scan task's 90-110 s
        // compute-bound stretch on the other core, same purpose as
        // the FT8 controller `display.rs`'s own periodic "alive"
        // line — direct evidence over a plausible architecture, after
        // a first cut of this fix turned out to have its own bug
        // (see `main`'s doc comment on spawn ordering).
        if tick % 20 == 0 {
            log::info!(
                "wspr_app::display: alive tick={tick} dirty={dirty} utc={hhmmss} heap={heap_kb}k"
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
                picker.render(&mut display, BootMode::Wspr).ok();
                if picker.take_just_closed() {
                    last_dirty = u32::MAX;
                }
            }
            FreeRtos::delay_ms(50);
        }
    }
}

// ── Network task (WiFi + UDP log + NTP + HTTP config server) ────────────

/// Stack for the network task. Its own peak is unmeasured (no
/// decode-sized frames — WiFi scan/connect, one `EspSntp`, one
/// `EspHttpServer::new` call, all against small stack-local structs),
/// picked to comfortably exceed the display task's own 24 KiB for a
/// similarly "no huge frames, but not tiny either" workload.
///
/// **Lives in PSRAM, not internal DRAM — real-hardware finding,
/// 2026-08-15.** `wifi_driver_init` alone (rx/tx buffer pools, its own
/// ~6.5 KiB driver task) was measured taking internal DRAM from 57 KB
/// free down to **10 KB free / 7 KB largest contiguous block** — nowhere
/// near enough left for a 24 KiB stack via a plain internal-DRAM
/// `xTaskCreatePinnedToCore`, regardless of spawn ordering (moving the
/// spawn earlier, before `scan_loop`'s own churn, was tried first and
/// didn't help — this is a genuine budget shortfall, not a race).
/// [`spawn_network_task`] uses `xTaskCreatePinnedToCoreWithCaps`
/// (`MALLOC_CAP_SPIRAM`, same mechanism `http_config`'s httpd task
/// stack already uses, gated by the same
/// `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y`) instead, sidestepping
/// the internal-DRAM budget entirely.
const NETWORK_STACK: u32 = 24 * 1024;
/// Below the DDC/display tasks — this task's own work (WiFi bring-up,
/// one NTP sync, one HTTP server start) is a one-time cost, not
/// something that should ever contend for CPU against the DDC
/// producer or display render loop once it's done.
const NETWORK_PRIORITY: u32 = 2;

struct NetworkCtx {
    /// Already constructed + started in `main`, before `SCAN_GO` — see
    /// the comment at that call site for why driver construction
    /// itself can't be backgrounded the way the rest of this can.
    wifi_driver: esp_idf_svc::wifi::BlockingWifi<esp_idf_svc::wifi::EspWifi<'static>>,
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
}

extern "C" fn network_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_network_task` leaked exactly this pointer via
    // `Box::into_raw`, and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut NetworkCtx) };
    network_loop(*ctx);
}

fn spawn_network_task(ctx: NetworkCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    // See NETWORK_STACK's own doc comment — PSRAM-backed stack, same
    // mechanism (and same enabling Kconfig) `http_config`'s httpd task
    // already uses, for the same reason (internal DRAM too tight once
    // WiFi's own driver buffers are live).
    let caps = esp_idf_svc::sys::MALLOC_CAP_SPIRAM | esp_idf_svc::sys::MALLOC_CAP_8BIT;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCoreWithCaps(
            Some(network_task_entry),
            c"wspr_net".as_ptr(),
            NETWORK_STACK,
            ptr,
            NETWORK_PRIORITY,
            core::ptr::null_mut(),
            1, // core 1, alongside display/DDC — never core 0 (scan).
            caps,
        )
    };
    if created != 1 {
        log::error!("wspr_app: failed to create wspr_net task");
    }
}

/// Association/DHCP (persistent retry — see `wifi::connect_with_retry`'s
/// own doc comment for why this AP needs it) → UDP log sink → NTP →
/// HTTP config server, entirely in the background, against the driver
/// `main` already constructed. `main`'s own boot doesn't wait on any
/// of this — `SCAN_GO` fires right after `main` spawns this task, not
/// after it finishes.
///
/// **Retroactive, not just prospective**: `run_one_slot` reads
/// `WSPR_UI.ntp_synced` fresh every slot, so wsprnet reporting turns
/// on for the *next* slot decoded after this task eventually finishes
/// — including slots whose decode started before WiFi came up. A slot
/// already in progress when this task finishes doesn't retroactively
/// get reported (the check happens once, at that slot's own report
/// time, not continuously), but nothing is permanently lost the way
/// it was when WiFi failing once meant "no wsprnet reporting for the
/// rest of this boot."
fn network_loop(mut ctx: NetworkCtx) -> ! {
    // Blocks until connected — see `connect_with_retry`'s own doc
    // comment. Only returns `Err` for a malformed SSID/PSK (checked
    // once, not worth retrying), not for a transient connect failure.
    let info = match mfsk_app_shared::wifi::connect_with_retry(
        &mut ctx.wifi_driver,
        crate::WIFI_SSID,
        crate::WIFI_PSK,
        None,
    ) {
        Ok(i) => i,
        Err(e) => {
            log::error!("wspr_app: WiFi setup failed permanently: {e:#}");
            loop {
                FreeRtos::delay_ms(60_000);
            }
        }
    };
    log::info!("wspr_app: WiFi up, ip {}", info.ip);

    // UDP log sink — see this file's top doc comment for why (USB
    // host mode later takes the serial console with it). Same
    // target-resolution + staging-drain sequence `main.rs` uses.
    let target_ip: std::net::IpAddr = if crate::UDP_LOG_TARGET.is_empty() || crate::UDP_LOG_TARGET == "auto" {
        std::net::IpAddr::V4(info.subnet_broadcast)
    } else {
        match crate::UDP_LOG_TARGET.parse() {
            Ok(ip) => ip,
            Err(e) => {
                log::warn!(
                    "wspr_app: UDP_LOG_TARGET '{}' parse failed ({e}); using subnet bcast",
                    crate::UDP_LOG_TARGET
                );
                std::net::IpAddr::V4(info.subnet_broadcast)
            }
        }
    };
    let port: u16 = crate::UDP_LOG_PORT.parse().unwrap_or(9999);
    let addr = std::net::SocketAddr::new(target_ip, port);
    match udp_log::UdpLogSink::new(addr) {
        Ok(sink) => {
            if let Ok(mut slot) = crate::FANOUT.udp.try_lock() {
                *slot = Some(sink);
            }
            crate::FANOUT.drain_staging_to_udp();
            log::info!("wspr_app: UDP log sink up → {addr}");
        }
        Err(e) => log::warn!("wspr_app: UDP socket bind failed: {e}"),
    }

    // ── NTP: one attempt, now that WiFi is confirmed up. Not
    // per-slot — see `ntp.rs`'s own doc comment for why absolute time
    // can't come from anywhere else on a cold start. ────────────────
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
                log::warn!("wspr_app: NTP start failed: {e:#}");
                (false, None)
            }
        }
    } else {
        log::info!("wspr_app: NTP sync disabled in settings");
        (false, None)
    };
    if !ntp_synced {
        log::warn!("wspr_app: NTP never synced — wsprnet reporting stays off until it does");
    }
    {
        let mut ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
        ui.update_status(|u| u.ntp_synced = ntp_synced);
    }

    // ── HTTP config server. ──────────────────────────────────────────
    let _http_server = match http_config::start(ctx.nvs.clone()) {
        Ok(s) => {
            log::info!("wspr_app: HTTP config server up");
            Some(s)
        }
        Err(e) => {
            log::warn!("wspr_app: HTTP config server failed: {e:#}");
            None
        }
    };

    // `ctx.wifi_driver`/`_sntp_keepalive`/`_http_server` are never read
    // again but must outlive this task (their `Drop`s tear down the
    // association / stop syncing / stop listening) — this task never
    // returns, so keeping them in scope (`ctx` itself, plus the two
    // locals) through the tail loop below already covers that.
    loop {
        FreeRtos::delay_ms(60_000);
    }
}

// ── Scan task ─────────────────────────────────────────────────────────

struct ScanCtx {
    nvs: Arc<Mutex<EspNvs<NvsDefault>>>,
}

extern "C" fn scan_task_entry(arg: *mut core::ffi::c_void) {
    // SAFETY: `spawn_scan_task` leaked exactly this pointer via
    // `Box::into_raw`, and this is the only place that reclaims it.
    let ctx = unsafe { Box::from_raw(arg as *mut ScanCtx) };
    scan_loop(*ctx);
}

fn spawn_scan_task(ctx: ScanCtx) {
    let ptr = Box::into_raw(Box::new(ctx)) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(scan_task_entry),
            c"wspr_scan".as_ptr(),
            SCAN_STACK,
            ptr,
            5,
            core::ptr::null_mut(),
            0, // pinned to core 0, matching wspr-bench — wspr_dual_core's
               // worker expects the caller on a known core.
        )
    };
    if created != 1 {
        log::error!("wspr_app: failed to create wspr_scan task");
    }
}

/// One `run_scan` per slot, forever. Never returns.
///
/// **Slot 0** decodes the baked golden baseband directly, bypassing
/// the DDC pipeline entirely — see this file's top doc comment for
/// why. **Every slot after that** waits on [`DDC_READY_IDX`] and
/// decodes whichever buffer `ddc_loop` most recently produced,
/// holding that buffer's `Mutex` for the whole `run_scan` call. No
/// separate "sleep to next slot boundary" is needed once the DDC
/// pipeline takes over — waiting on `DDC_READY_IDX` already paces
/// this loop to `ddc_loop`'s own `SLOT_US` cadence.
fn scan_loop(ctx: ScanCtx) -> ! {
    while !SCAN_GO.load(Ordering::Acquire) {
        FreeRtos::delay_ms(200);
    }
    log::info!("wspr_app: scan loop starting");

    // Slot 0 only: `idat`/`qdat` land in PSRAM automatically (`NBB`
    // f32 samples is 184 320 B, well past this crate's
    // `SPIRAM_MALLOC_ALWAYSINTERNAL` threshold of 4096 B), same
    // reasoning `wspr-bench` documents for its own manually-placed
    // buffers, just without the manual `heap_caps_malloc` call since
    // this app isn't choosing between SRAM/PSRAM on purpose.
    {
        let mut idat = vec![0.0f32; NBB];
        let mut qdat = vec![0.0f32; NBB];
        load_baseband(GOLDEN_BASEBAND, &mut idat, &mut qdat);
        run_one_slot(&ctx, &mut idat, &mut qdat, "0 (golden)", false);
    }

    let mut slot_num = 1u32;
    loop {
        // Block for `ddc_loop`'s handoff. `Option::take` clears the
        // mailbox so a slow decode can't accidentally re-read a
        // buffer `ddc_loop` is already partway through overwriting
        // for the *next* slot — the two tasks only ever touch a given
        // buffer index one at a time, enforced by `BASEBAND_BUFS`'
        // own per-index `Mutex`, but clearing the mailbox here also
        // stops this loop from spinning on a stale index.
        let idx = loop {
            let mut ready = DDC_READY_IDX.lock().expect("ddc ready mutex poisoned");
            if let Some(idx) = ready.take() {
                break idx;
            }
            drop(ready);
            FreeRtos::delay_ms(200);
        };
        let mut buf = BASEBAND_BUFS[idx].lock().expect("ddc buf mutex poisoned");
        let mut idat = core::mem::take(&mut buf.idat);
        let mut qdat = core::mem::take(&mut buf.qdat);
        // Drop the buffer lock before `run_scan` — the data itself
        // moved out via `mem::take`, so holding the lock for the
        // ~80-100 s decode would only block `ddc_loop` from claiming
        // this index again for no benefit. `ddc_loop` always fully
        // replaces `idat`/`qdat` rather than reusing them (see
        // `SlotBuf`'s own doc comment), so there's nothing to hand
        // back afterward — the local `idat`/`qdat` below just get
        // dropped once `run_one_slot` returns.
        drop(buf);

        run_one_slot(&ctx, &mut idat, &mut qdat, &slot_num.to_string(), true);
        slot_num = slot_num.wrapping_add(1);
    }
}

/// One slot's decode + UI update + wsprnet report — the body every
/// `scan_loop` iteration runs, factored out so the golden-baseband
/// slot 0 and the DDC-fed slots after it share it exactly rather than
/// duplicating the reporting/UI plumbing.
///
/// `is_synthetic_source` — `true` for every DDC-fed slot (the audio is
/// [`build_ddc_test_track`]'s fabricated [`DDC_TEST_CALL`] burst, not
/// a real reception), `false` for slot 0 (a real, if stale, WAV-derived
/// recording of real stations). Gates wsprnet reporting: a "station"
/// this device fabricated itself has no business appearing in a public
/// spot database as a real reception, identically every ~2 minutes,
/// forever — the golden-baseband slot's *is* real (if replayed)
/// signal content, so that path is left as this file's pre-existing
/// behaviour rather than newly restricted here too.
fn run_one_slot(
    ctx: &ScanCtx,
    idat: &mut [f32],
    qdat: &mut [f32],
    slot_label: &str,
    is_synthetic_source: bool,
) {
    let settings = {
        let g = ctx.nvs.lock().expect("settings NVS mutex poisoned");
        settings::load(&g)
    };
    let band = &WSPR_BANDS[settings.band_idx as usize];

    let (stats, results) = run_scan(idat, qdat);
    for s in &stats {
        s.log();
    }
    // The source belongs on the line that reports the result, not on a
    // separate one that only appears when it is synthetic. Reading a
    // decode count without knowing whether the audio came from a radio
    // is how a baked test signal gets mistaken for a live one — the FT8
    // controller labelled every slot `WAV[n]` whatever the source, and
    // that cost real time on 2026-08-23.
    log::info!(
        "wspr_app: slot {slot_label} src={} decoded {} station(s)",
        if is_synthetic_source { "synthetic" } else { "uac" },
        results.len()
    );

    let (date, time) = current_date_time();
    let rows: Vec<WsprSpotRow> = results.iter().map(|r| to_row(r, &time)).collect();

    {
        let mut ui = WSPR_UI.lock().expect("WSPR_UI mutex poisoned");
        ui.set_slot(time.clone(), &rows);
        ui.update_status(|u| {
            u.band_label = heapless::String::try_from(band.label).unwrap_or_default();
            u.dial_mhz = band.dial_mhz;
            u.wsprnet_enabled = mfsk_app_shared::wsprnet::SpotSink::from_config(Some(
                &settings.wsprnet_spot_config,
            ))
            .is_enabled();
        });
    }

    let ntp_synced = WSPR_UI.lock().expect("WSPR_UI mutex poisoned").ntp_synced;
    if is_synthetic_source {
        log::info!("wspr_app: slot {slot_label} is DDC-synthetic — skipping wsprnet report");
    } else if ntp_synced {
        report_to_wsprnet(&results, &settings, band, &date, &time);
    } else {
        log::info!("wspr_app: NTP never synced this run — skipping wsprnet report");
    }
}

// ── DDC producer task ────────────────────────────────────────────────

extern "C" fn ddc_task_entry(_arg: *mut core::ffi::c_void) {
    ddc_loop();
}

fn spawn_ddc_task() {
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(ddc_task_entry),
            c"wspr_ddc".as_ptr(),
            DDC_STACK,
            core::ptr::null_mut(),
            4, // between the display task (3) and wspr_dual_core's
            // worker (5) — see this section's own doc comment.
            core::ptr::null_mut(),
            1, // core 1: pinning DDC to core 0 alongside the scan
               // task would starve it exactly the way the display
               // task was starved before that bug was fixed (scan is
               // priority 5, continuous, never yields) — see this
               // file's top doc comment on that fix. Core 1 already
               // hosts the display task and `wspr_dual_core`'s
               // intermittent worker; DDC only needs ~20 s spread
               // across a ~120 s cycle (per `wspr-bench`'s own
               // measurement), so it fits in what's left over.
        )
    };
    if created != 1 {
        log::error!("wspr_app: failed to create wspr_ddc task");
    }
}

/// Streams synthetic audio through [`StreamingDdcCascade`] into
/// whichever [`BASEBAND_BUFS`] slot isn't currently checked out by
/// `scan_loop`, alternating slots, forever. Paces itself to
/// [`SLOT_US`] the same way `scan_loop` used to before the DDC
/// pipeline existed — real audio would arrive at its own fixed rate
/// regardless of whether the previous slot's decode has finished, and
/// this keeps that property even with a synthetic source.
///
/// **Switched from `StreamingDdc` to `StreamingDdcCascade`,
/// 2026-08-15** — real-hardware measurement (raw-noise-only
/// benchmark, no burst synthesis) found the single-stage filter
/// costing ~30 s of a 120 s slot (25%), confirmed via pointer address
/// to be running out of PSRAM (`0x3c2...`) rather than the internal
/// DRAM its own design assumed — `DdcBufs`' buffers (7 172 B/9 220 B/
/// 9 220 B) all clear `SPIRAM_MALLOC_ALWAYSINTERNAL`=4096 with no
/// placement override anywhere in the codebase. The cascade
/// (`mfsk_core::wspr::ddc::StreamingDdcCascade`) fixes both problems
/// the same move was chasing: ~4.5× less compute (Crochiere-Rabiner
/// two-stage design, see that type's own doc comment) *and* small
/// enough that every buffer clears the 4 KB threshold on its own, so
/// it lands in internal DRAM automatically — no manual placement
/// needed here at all, unlike the old `DdcBufs`/`new_in` API this
/// replaces.
fn ddc_loop() -> ! {
    while !SCAN_GO.load(Ordering::Acquire) {
        FreeRtos::delay_ms(200);
    }
    log::info!("wspr_app: ddc loop starting");

    // Rebuilt fresh each slot (cheap — packs 50 bits + encodes 162
    // symbols, no audio-sized work) rather than kept across
    // iterations, so each slot's noise pad is independently seeded.
    // See [`DdcTestSource`]'s own doc comment for why this streams
    // instead of materializing the whole burst.
    let mut chunk = vec![0.0f32; DDC_CHUNK_LEN];

    let mut write_idx = 0usize;
    loop {
        // Real UAC audio has taken over `BASEBAND_BUFS` production —
        // see [`UAC_AUDIO_ACTIVE`]'s own doc comment. Idle forever
        // rather than exit, in case a future generation-counter fix
        // ever wants this task back; today it simply never wakes up
        // again once real audio starts.
        if UAC_AUDIO_ACTIVE.load(Ordering::Acquire) {
            FreeRtos::delay_ms(1000);
            continue;
        }

        let slot_start_us = now_us();
        let mut source = DdcTestSource::new();

        let (oi, oq) = {
            let mut ddc = StreamingDdcCascade::new();
            let mut oi: Vec<f32> = Vec::with_capacity(NBB + 64);
            let mut oq: Vec<f32> = Vec::with_capacity(NBB + 64);
            let mut fed = 0usize;
            while fed < DDC_SLOT_AUDIO_SAMPLES {
                source.fill_next(&mut chunk);
                ddc.push(&chunk, &mut oi, &mut oq);
                fed += chunk.len();
            }
            ddc.flush(&mut oi, &mut oq);
            // Match `ddc_to_baseband`'s own finish: zero-pad/truncate
            // to exactly `NBB` (== `NFFT2`), what `run_scan` expects.
            oi.resize(NBB, 0.0);
            oq.resize(NBB, 0.0);
            (oi, oq)
        };

        {
            let mut buf = BASEBAND_BUFS[write_idx]
                .lock()
                .expect("ddc buf mutex poisoned");
            buf.idat = oi;
            buf.qdat = oq;
        }
        {
            let mut ready = DDC_READY_IDX.lock().expect("ddc ready mutex poisoned");
            *ready = Some(write_idx);
        }
        log::info!("wspr_app::ddc: produced buffer {write_idx}");
        write_idx = 1 - write_idx;

        let elapsed_us = now_us() - slot_start_us;
        // Kept permanently (added 2026-08-15 during the task-priority
        // redesign, see wspr-app's own design memory) — the
        // pacing-sleep-free cost of one full slot's worth of
        // `fill_next`+`ddc.push`+`ddc.flush` calls, exactly the number
        // a schedulability analysis needs. This is what caught
        // `StreamingDdc` running out of PSRAM at ~25% slot occupancy
        // in the first place; worth watching on an ongoing basis now
        // that `StreamingDdcCascade` should be sitting far lower.
        log::info!(
            "wspr_app::ddc: compute occupancy {} ms / {} ms slot budget ({:.1}%)",
            elapsed_us / 1000,
            SLOT_US / 1000,
            elapsed_us as f64 / SLOT_US as f64 * 100.0
        );
        let remaining_ms = ((SLOT_US - elapsed_us).max(0) / 1000) as u32;
        if remaining_ms > 0 {
            FreeRtos::delay_ms(remaining_ms);
        }
    }
}

// ── Real UAC audio sink ──────────────────────────────────────────────

/// [`crate::uac::AudioSink`] implementation that feeds real captured 12 kHz
/// mono audio into a per-slot [`StreamingDdcCascade`], reusing exactly
/// the shape `ddc_loop`'s synthetic path already used and had
/// hardware-verified (fresh cascade each slot, [`DDC_SLOT_AUDIO_SAMPLES`]
/// raw input then `flush()`, resize/truncate to [`NBB`]) — only the raw
/// audio *source* changes (real UAC-resampled samples, arriving via
/// `push_samples` calls from `uac.rs`'s reader thread, instead of
/// [`DdcTestSource`]'s self-paced synthetic generator). See this file's
/// top doc comment for the handoff mechanics and the still-open
/// wall-clock-alignment / hardware-verification caveats.
///
/// Deliberately never materializes a full slot's raw audio up front —
/// that shape (a ~5.3 MB `Vec`) is exactly what OOM'd `DdcTestSource`'s
/// first cut on real hardware (see that struct's own doc comment);
/// `push_samples` only ever holds one incoming USB read's worth of
/// samples (≤ a few hundred) at a time.
struct WsprDdcSink {
    ddc: StreamingDdcCascade,
    out_i: Vec<f32>,
    out_q: Vec<f32>,
    /// Raw 12 kHz input samples fed into `ddc` so far this slot.
    fed: usize,
    write_idx: usize,
    /// Reused f32-conversion scratch buffer — one incoming batch's
    /// worth at a time, never a full slot.
    scratch: Vec<f32>,
}

impl WsprDdcSink {
    fn new() -> Self {
        Self {
            ddc: StreamingDdcCascade::new(),
            out_i: Vec::with_capacity(NBB + 64),
            out_q: Vec::with_capacity(NBB + 64),
            fed: 0,
            write_idx: 0,
            scratch: Vec::new(),
        }
    }

    /// Flush the current slot's cascade, hand the finished baseband to
    /// [`BASEBAND_BUFS`]/[`DDC_READY_IDX`] exactly like `ddc_loop`
    /// does, and start a fresh cascade for the next slot.
    fn finish_slot(&mut self) {
        self.ddc.flush(&mut self.out_i, &mut self.out_q);
        self.out_i.resize(NBB, 0.0);
        self.out_q.resize(NBB, 0.0);
        self.out_i.truncate(NBB);
        self.out_q.truncate(NBB);

        let idat = core::mem::replace(&mut self.out_i, Vec::with_capacity(NBB + 64));
        let qdat = core::mem::replace(&mut self.out_q, Vec::with_capacity(NBB + 64));
        {
            let mut buf = BASEBAND_BUFS[self.write_idx]
                .lock()
                .expect("ddc buf mutex poisoned");
            buf.idat = idat;
            buf.qdat = qdat;
        }
        {
            let mut ready = DDC_READY_IDX.lock().expect("ddc ready mutex poisoned");
            *ready = Some(self.write_idx);
        }
        log::info!(
            "wspr_app::ddc: produced buffer {} (real UAC audio)",
            self.write_idx
        );
        self.write_idx = 1 - self.write_idx;

        // Fresh cascade per slot, matching `ddc_loop`'s own per-slot
        // `StreamingDdcCascade::new()` — not a continuously-running
        // filter across slot boundaries (see this struct's own doc
        // comment for why that shape was reused rather than redesigned).
        self.ddc = StreamingDdcCascade::new();
        self.fed = 0;
    }
}

impl crate::uac::AudioSink for WsprDdcSink {
    fn push_samples(&mut self, samples_12k_mono: &[i16]) {
        if !UAC_AUDIO_ACTIVE.swap(true, Ordering::AcqRel) {
            log::info!("wspr_app::ddc: real UAC audio arriving — synthetic generator handing off");
        }

        let mut remaining = samples_12k_mono;
        while !remaining.is_empty() {
            let room = DDC_SLOT_AUDIO_SAMPLES - self.fed;
            let take = remaining.len().min(room);
            self.scratch.clear();
            self.scratch
                .extend(remaining[..take].iter().map(|&s| s as f32 / 32768.0));
            self.ddc
                .push(&self.scratch, &mut self.out_i, &mut self.out_q);
            self.fed += take;
            remaining = &remaining[take..];
            if self.fed >= DDC_SLOT_AUDIO_SAMPLES {
                self.finish_slot();
            }
        }
    }
}

/// Streams the same passband audio [`ddc_loop`] feeds every DDC-fed
/// slot — [`DDC_LEAD_PAD_SAMPLES`] of noise, then a real,
/// correctly-encoded WSPR Type-1 burst ([`DDC_TEST_CALL`]
/// [`DDC_TEST_GRID`] [`DDC_TEST_DBM`]), then noise again out to
/// [`DDC_SLOT_AUDIO_SAMPLES`] — **one [`DDC_CHUNK_LEN`] chunk at a
/// time**, without ever materializing the whole burst. Only the 162
/// packed symbols (162 B) plus a handful of position/phase scalars
/// live in `self`; everything else is recomputed per sample from
/// [`tx::synthesize_audio_into`]'s own per-sample formula (continuous
/// phase across symbol boundaries, plus its raised-cosine ramp at the
/// very start/end of the burst) — see this file's `DDC_TEST_CALL`
/// doc comment for why a whole-burst `Vec` (this struct's predecessor)
/// isn't safe to allocate here regardless of size, and `tx.rs` itself
/// for the reference implementation this mirrors.
///
/// [`tx::synthesize_audio_into`]: mfsk_core::wspr::tx::synthesize_audio_into
struct DdcTestSource {
    symbols: [u8; 162],
    nsps: usize,
    nramp: usize,
    /// Absolute sample position within the whole per-slot stream
    /// (lead pad + burst + trail pad), advanced by [`Self::fill_next`].
    pos: usize,
    /// Continuous phase carried across symbol boundaries within the
    /// burst — reset to `0.0` at the burst's first sample, otherwise
    /// only ever incremented, matching `synthesize_audio_into`.
    phase: f32,
    rng: u32,
}

impl DdcTestSource {
    fn new() -> Self {
        let info = mfsk_core::msg::wspr::pack_type1(DDC_TEST_CALL, DDC_TEST_GRID, DDC_TEST_DBM)
            .expect(
                "DDC_TEST_CALL/DDC_TEST_GRID/DDC_TEST_DBM must pack as a valid WSPR type-1 message",
            );
        let symbols = mfsk_core::wspr::encode_channel_symbols(&info);
        let nsps = (AUDIO_RATE_HZ as f64
            * <mfsk_core::wspr::Wspr as mfsk_core::engine::ModulationParams>::SYMBOL_DT as f64)
            .round() as usize;
        let nramp = mfsk_core::engine::dsp::envelope::ramp_samples(AUDIO_RATE_HZ as u32, nsps);
        Self {
            symbols,
            nsps,
            nramp,
            pos: 0,
            phase: 0.0,
            rng: 0x2026_0815,
        }
    }

    fn burst_len(&self) -> usize {
        self.nsps * self.symbols.len()
    }

    /// Fill `chunk` with the next `chunk.len()` samples of the stream,
    /// advancing all internal state.
    fn fill_next(&mut self, chunk: &mut [f32]) {
        let burst_start = DDC_LEAD_PAD_SAMPLES;
        let burst_end = burst_start + self.burst_len();
        for v in chunk.iter_mut() {
            *v = if self.pos < burst_start || self.pos >= burst_end {
                noise_sample(&mut self.rng)
            } else {
                // Symbol boundaries only change which tone `phase`
                // advances by (`freq`/`dphi` below) — `phase` itself
                // is never reset here, staying continuous across the
                // boundary exactly like `synthesize_audio_into`.
                let i = self.pos - burst_start; // 0..burst_len
                let symbol_idx = i / self.nsps;
                let freq = DDC_TEST_BASE_FREQ_HZ
                    + self.symbols[symbol_idx] as f32
                        * <mfsk_core::wspr::Wspr as mfsk_core::engine::ModulationParams>::TONE_SPACING_HZ;
                let dphi = core::f32::consts::TAU * freq / AUDIO_RATE_HZ;
                let sample = DDC_TEST_AMPLITUDE * self.phase.cos();
                self.phase += dphi;
                if self.phase > core::f32::consts::TAU {
                    self.phase -= core::f32::consts::TAU;
                } else if self.phase < -core::f32::consts::TAU {
                    self.phase += core::f32::consts::TAU;
                }
                let env = if i < self.nramp {
                    (1.0 - (core::f32::consts::TAU * i as f32 / (2.0 * self.nramp as f32)).cos())
                        / 2.0
                } else if i >= self.burst_len() - self.nramp {
                    let k = i - (self.burst_len() - self.nramp);
                    (1.0 + (core::f32::consts::TAU * k as f32 / (2.0 * self.nramp as f32)).cos())
                        / 2.0
                } else {
                    1.0
                };
                sample * env
            };
            self.pos += 1;
        }
    }
}

/// One fresh pseudorandom noise sample in `[-DDC_NOISE_AMPLITUDE,
/// +DDC_NOISE_AMPLITUDE]`, advancing `rng` in place. xorshift32 —
/// cheap and dependency-free, matching this crate's existing
/// preference for hand-rolled ASCII/percent-decode helpers over
/// pulling in a crate for something this small. Not cryptographic
/// quality; doesn't need to be — used only for [`DdcTestSource`]'s
/// lead/trail pad, where the requirement is just "not periodic," not
/// real randomness.
fn noise_sample(rng: &mut u32) -> f32 {
    let mut x = *rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *rng = x;
    let unit = (x as f32 / u32::MAX as f32) * 2.0 - 1.0; // -> [-1, 1]
    unit * DDC_NOISE_AMPLITUDE
}

// ── Decode-result plumbing ───────────────────────────────────────────

struct ParsedMsg {
    call: heapless::String<13>,
    grid: heapless::String<6>,
    dbm: i8,
}

/// `WsprResult.message` is the decoded 50-bit payload rendered as
/// text — `"<call> <grid> <dbm>"` for a type-1 message. Split rather
/// than re-derive: the decoder already did the unpacking, and both
/// the UI row and the wsprnet report want the same three fields back
/// out (same approach `wspr_bench.rs`'s `report_spots` uses).
fn parse_message(msg: &str) -> ParsedMsg {
    let mut it = msg.split_whitespace();
    let call = it.next().unwrap_or("");
    let grid = it.next().unwrap_or("");
    let dbm: i8 = it.next().and_then(|d| d.parse().ok()).unwrap_or(0);
    ParsedMsg {
        call: heapless::String::try_from(call).unwrap_or_default(),
        grid: heapless::String::try_from(grid).unwrap_or_default(),
        dbm,
    }
}

fn to_row(r: &WsprResult, hhmm: &heapless::String<4>) -> WsprSpotRow {
    let parsed = parse_message(&r.message.to_string());
    WsprSpotRow {
        utc_hhmm: hhmm.clone(),
        call: parsed.call,
        grid: parsed.grid,
        power_dbm: parsed.dbm,
        freq_offset_hz: r.freq_hz,
        snr_db: r.snr_db as i8,
        dt_sec: r.dt_sec,
        drift_hz: r.drift_hz as i8,
    }
}

/// Off by default (`SpotSink::Disabled`) or missing callsign both
/// skip silently by design — `settings.rs`'s own doc comment: an
/// empty callsign means "not configured," not an error to surface
/// every 2 minutes in the log.
fn report_to_wsprnet(
    results: &[WsprResult],
    settings: &Settings,
    band: &WsprBand,
    date: &str,
    time: &str,
) {
    use mfsk_app_shared::wsprnet::{report_slot, Mode, Reporter, Spot, SpotSink};

    let sink = SpotSink::from_config(Some(settings.wsprnet_spot_config.as_str()));
    if !sink.is_enabled() {
        return;
    }
    if settings.call.is_empty() {
        log::warn!("wspr_app: wsprnet reporting is on but no callsign is set — skipping this slot");
        return;
    }

    let reporter = Reporter {
        call: settings.call.to_string(),
        grid: settings.grid.to_string(),
        dial_mhz: band.dial_mhz,
        version: format!("mfsk-core-{}", mfsk_core::VERSION),
        mode: Mode::Wspr2,
    };

    let spots: Vec<Spot> = results
        .iter()
        .map(|r| {
            let parsed = parse_message(&r.message.to_string());
            Spot {
                date: date.to_string(),
                time: time.to_string(),
                call: parsed.call.to_string(),
                grid: parsed.grid.to_string(),
                dbm: parsed.dbm as i32,
                snr_db: r.snr_db as i32,
                dt_sec: r.dt_sec,
                drift_hz: r.drift_hz as i32,
                audio_hz: r.freq_hz,
            }
        })
        .collect();

    let bodies = report_slot(&spots, &reporter, &sink);
    log::info!("wspr_app: wsprnet reported {} spot(s)", bodies.len());
}

// ── Clock formatting ─────────────────────────────────────────────────

fn current_hhmmss() -> heapless::String<8> {
    let mut s = heapless::String::new();
    if let Ok(dur) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        let (_, _, _, h, m, sec) = civil_from_unix(dur.as_secs() as i64);
        let _ = write!(&mut s, "{h:02}:{m:02}:{sec:02}");
    } else {
        let _ = s.push_str("--:--:--");
    }
    s
}

/// `(yyMMdd, HHmm)` — wsprnet's `date`/`time` fields, and the second
/// also doubles as [`WsprSpotRow::utc_hhmm`]. Meaningless (reads
/// whatever the unsynced system clock happens to hold) unless NTP
/// synced this run — callers gate on that separately rather than this
/// function returning an `Option`, since the WSPR UI still wants
/// *some* 4 digits to show even before/without a sync.
fn current_date_time() -> (heapless::String<6>, heapless::String<4>) {
    let mut date = heapless::String::new();
    let mut time = heapless::String::new();
    if let Ok(dur) = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH) {
        let (y, mo, d, h, mi, _s) = civil_from_unix(dur.as_secs() as i64);
        let _ = write!(&mut date, "{:02}{mo:02}{d:02}", y.rem_euclid(100));
        let _ = write!(&mut time, "{h:02}{mi:02}");
    } else {
        let _ = date.push_str("000000");
        let _ = time.push_str("0000");
    }
    (date, time)
}
