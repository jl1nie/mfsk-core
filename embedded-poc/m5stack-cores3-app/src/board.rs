//! M5Stack CoreS3 公式 pinout.
//!
//! ESP32-S3-WROOM-1-N16R8 (LX7 dual-core), ILI9342C 320×240 IPS LCD,
//! FT6336U 5-point capacitive touch, AXP2101 PMIC, AW9523B I/O expander,
//! ES7210 dual-mic ADC, AW88298 speaker amp, GC0308 camera, BMI270 IMU,
//! USB-OTG host on GPIO 19/20.
//!
//! Phase 0-Core uses: AXP2101 + AW9523B (power / LCD RST / backlight),
//! ILI9342C LCD, and I2C0 bus. All other peripherals deferred.

#![allow(dead_code)]

// ── LCD: ILI9342C, 320×240 landscape ─────────────────────────────────
// FSPI (SPI2_HOST). LCD RST は AW9523B P1_0、BL は AW9523B P0_4 経由。
pub const LCD_SPI_HOST: u8 = 1; // SPI2_HOST / FSPI
pub const LCD_PIN_SCK: i32 = 36;
pub const LCD_PIN_MOSI: i32 = 37;
pub const LCD_PIN_CS: i32 = 3;
pub const LCD_PIN_DC: i32 = 35;
// ── Panel geometry: one statement of the rotation, everything else
//    derived from it ────────────────────────────────────────────────
//
// There are two frames on this board and confusing them is what made
// the mode picker unpressable: the LCD controller and the FT5x06 both
// work in the panel's native 320x240, while every receiver draws
// through mipidsi with a quarter turn applied, giving a 240x320
// canvas. `LCD_WIDTH`/`LCD_HEIGHT` used to be the only names here and
// meant the native frame, but layout code read them as the canvas —
// so the picker was sized against 320 and hung 24 px off a 240-wide
// screen, and touch coordinates were hit-tested in the wrong frame
// entirely.
//
// Change [`ROTATION`] and both the canvas size and `touch`'s mapping
// follow. Nothing else should name a rotation.

/// Native panel, and the frame the touch controller reports in
/// (M5GFX configures the FT5x06 as `x_max=319, y_max=239`).
pub const NATIVE_W: u16 = 320;
pub const NATIVE_H: u16 = 240;

/// The quarter turn every receiver applies.
pub const ROTATION: mipidsi::options::Rotation = mipidsi::options::Rotation::Deg90;

/// The canvas receivers actually draw into, after [`ROTATION`].
pub const CANVAS_W: u16 = NATIVE_H;
pub const CANVAS_H: u16 = NATIVE_W;

// ── I2C bus 0 (AXP2101 + AW9523B + FT6336U + BMI270 共有) ────────────
pub const I2C0_SCL: i32 = 11;
pub const I2C0_SDA: i32 = 12;
pub const AXP2101_I2C_ADDR: u8 = 0x34;
pub const AW9523B_I2C_ADDR: u8 = 0x58;
/// FT5x06-family touch controller. Reads `CIPHER:0x64 FIRMID:0x03
/// VENDID:0x01` on this board — see `touch.rs` for why the vendor byte
/// not being M5's 0x11 is not a problem.
pub const TOUCH_I2C_ADDR: u8 = 0x38;
pub const IMU_I2C_ADDR: u8 = 0x69; // BMI270

// AW9523B pin assignments (M5Stack CoreS3).
//
// **2026-08-23 correction — the USB host power pins were all wrong**,
// found on hardware chasing "the host stack enumerates nothing" (#163).
// Checked against M5Stack's own `M5Unified` (`src/utility/Power_Class.cpp`,
// the `board_M5StackCoreS3` path), which is the authority here:
//
// ```cpp
// static constexpr const uint32_t _core_s3_bus_en = 0b00000010; // port0 bit1
// static constexpr const uint32_t _core_s3_usb_en = 0b00100000; // port0 bit5
// static constexpr const uint32_t port1_bitmask_boost = 0b10000000; // port1 bit7
// // setUsbOutput(true) -> p0 |= _core_s3_usb_en; p1 |= port1_bitmask_boost;
// // setExtOutput(true) -> p0 |= _core_s3_bus_en; p1 |= port1_bitmask_boost;
// ```
//
// So USB host VBUS needs **two** pins, and neither is the one this file
// used to name `BUS_OUT_EN`: port1 bit7 runs the 5 V boost converter,
// port0 bit5 gates that rail onto the USB connector. Port0 bit1 is the
// *external* 5 V output (M-Bus / Grove) and has nothing to do with USB —
// yet it was the only bit host mode ever asserted. Its output-register
// readback duly said HIGH, which is why this looked for a long time like
// a device-side or cable fault: no converter was running, so no device
// ever saw VBUS, so nothing pulled up D+, which the host stack cannot
// distinguish from an empty port.
//
//   P0_0 = required by M5GFX, purpose unnamed (OUTPUT — `CFG0`'s
//          0b0001_1000 makes it one, and M5GFX drives it high before
//          the touch controller will answer). **Not** the touch
//          interrupt, which this comment used to claim: M5GFX puts
//          TP_INT on GPIO 21.
//   P0_1 = BUS_OUT_EN  (external 5V out — M-Bus/Grove, NOT USB)
//   P0_5 = USB_OTG_EN  (gates the 5V boost onto the USB connector)
//   P0_2 = SPK_EN      (AW88298 speaker amp enable; Phase 3-Core).
//          This table said P0_7 until 2026-08-23. The bit M5Unified
//          actually toggles is 2: `_speaker_enabled_cb_cores3` does
//          `bitOn(aw9523, 0x02, 0b00000100)` on enable and `bitOff` on
//          the same bit on disable. Copying M5GFX's display init, which
//          raises bits 0 and 2 together, therefore switched the
//          amplifier on — and the IC-705 stopped enumerating.
//   P1_0 = TP_RST      (touch reset, active LOW)
//   P1_1 = LCD_RST     (ILI9342C reset, active LOW)
//   P1_7 = BOOST_EN    (the 5V boost converter itself)
//
// Earlier correction (2026-08-15, same cross-check against M5GFX):
// LCD_RST/TP_RST were swapped, and **LCD_BL is not an AW9523 pin at
// all** — the backlight runs off AXP2101's DLDO1 rail (register `0x90`
// bit `0x80` to enable, `0x99` for voltage), which `pmic.rs` writes
// directly. `AW9523_P0_LCD_BL` survives below only as a documented
// wrong guess that `pmic::init` still writes as part of the port-0
// safe-default pattern; nothing depends on it.
/// External 5 V output (M-Bus / Grove). **Not** the USB host rail.
pub const AW9523_P0_BUS_OUT_EN: u8 = 1 << 1;
/// Gates the 5 V boost onto the USB connector (M5Unified `_core_s3_usb_en`).
pub const AW9523_P0_USB_OTG_EN: u8 = 1 << 5;
/// The 5 V boost converter itself (M5Unified `port1_bitmask_boost`).
pub const AW9523_P1_BOOST_EN: u8 = 1 << 7;
pub const AW9523_P0_LCD_BL: u8 = 1 << 4;
pub const AW9523_P0_SPK_EN: u8 = 1 << 2;
pub const AW9523_P1_TP_RST: u8 = 1 << 0;
pub const AW9523_P1_LCD_RST: u8 = 1 << 1;

// ── USB OTG (Phase 1-Core) ────────────────────────────────────────────
pub const USB_OTG_DP: i32 = 20;
pub const USB_OTG_DM: i32 = 19;

// ── Audio (Phase 3-Core) ──────────────────────────────────────────────
pub const ES7210_I2C_ADDR: u8 = 0x40;
pub const AW88298_I2C_ADDR: u8 = 0x36;

/// Spawn a `std::thread` under a name FreeRTOS actually keeps.
///
/// `std::thread::Builder::name()` sets a Rust-side name and nothing
/// else: the FreeRTOS task is still called "pthread". That is fine
/// until something crashes, at which point the coredump reports
/// `task 'pthread'` and cannot say *which* of them — which is exactly
/// where the 2026-08-23 stack-overflow hunt stalled, with four
/// candidate threads and no way to tell them apart. The task name
/// lives in `esp_pthread_cfg_t`, so it has to be set through the
/// spawn configuration instead.
///
/// Starting from `Default::default()` (i.e. `esp_pthread_get_default_config`)
/// keeps priority and affinity exactly as a plain `Builder` spawn
/// would have them; only the name and the stack size are ours.
pub fn spawn_named<F>(
    name: &'static core::ffi::CStr,
    stack_size: usize,
    f: F,
) -> std::io::Result<std::thread::JoinHandle<()>>
where
    F: FnOnce() + Send + 'static,
{
    spawn_named_tuned(name, stack_size, None, None, f)
}

/// [`spawn_named`] with the scheduling left explicit: priority, core,
/// or both.
///
/// A plain pthread here takes `CONFIG_PTHREAD_TASK_PRIO_DEFAULT` (5)
/// and `CONFIG_PTHREAD_TASK_CORE_DEFAULT` (-1, no affinity), which is
/// what every thread on this board had until 2026-09-19 — including
/// the two whose relative scheduling decides whether audio keeps its
/// cadence. The raw FreeRTOS tasks around them were never left that
/// way: `stage1_inc` takes priority 6 *because* it must preempt
/// `dsp_worker` at 5, and `net` is pinned to core 1 "never core 0,
/// which carries capture". This is the same control for the threads.
pub fn spawn_named_tuned<F>(
    name: &'static core::ffi::CStr,
    stack_size: usize,
    priority: Option<u8>,
    pin_to_core: Option<esp_idf_svc::hal::cpu::Core>,
    f: F,
) -> std::io::Result<std::thread::JoinHandle<()>>
where
    F: FnOnce() + Send + 'static,
{
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;

    let default = ThreadSpawnConfiguration::default();
    let cfg = ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        priority: priority.unwrap_or(default.priority),
        pin_to_core: pin_to_core.or(default.pin_to_core),
        ..default
    };
    if let Err(e) = cfg.set() {
        log::warn!("spawn_named: config set failed for {name:?}: {e:?} — task will be 'pthread'");
    }
    let result = std::thread::Builder::new().stack_size(stack_size).spawn(f);
    // Restore, so a later plain spawn does not inherit this name.
    if let Err(e) = ThreadSpawnConfiguration::default().set() {
        log::warn!("spawn_named: config restore failed: {e:?}");
    }
    result
}

/// This task's remaining stack, in bytes.
///
/// Allocates nothing, blocks on nothing, and costs four bytes of the
/// stack it measures — which is why it is the first thing to reach for
/// here rather than the last. The main task's own overflow was found
/// with one of these in a single flash.
pub fn log_stack_hw(tag: &str) {
    // SAFETY: null = the calling task.
    let hw = unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) };
    log::info!("[stack] {tag} hw={hw}");
}

/// Every task's remaining stack, from one place, in one line.
///
/// Written after two stack overflows in one day (`display::run_log_panel`
/// and `uac_reader`, both from stack sizes that had been estimated
/// rather than measured). The lesson was not "add a probe to the task
/// you suspect" — the probe added to `uac_reader`'s 1 Hz tick used
/// enough stack to make its crash *faster*, because `format_args!`
/// through the fanout is a few hundred bytes and that task had 168.
///
/// The high-water mark is monotonic: it is the smallest free space the
/// task has *ever* had, so one late sample carries as much information
/// as a thousand frequent ones. That makes a single low-frequency
/// reporter strictly better than per-task probes — safer, less code,
/// and it covers the IDF's own tasks (WiFi, lwIP, the USB host) which
/// no per-task probe of ours could reach.
///
/// Needs `CONFIG_FREERTOS_USE_TRACE_FACILITY=y`.
///
/// Costs ~1.4 KB of the *caller's* stack for the snapshot array, so
/// call it from a task with room — a display loop, not an audio
/// reader. It reports its own caller's headroom too, so if that ever
/// stops being true it says so.
/// **Not while a radio's USB audio is streaming** — use
/// [`log_idle_cpu`] then. `uxTaskGetSystemState` runs under
/// `taskENTER_CRITICAL` on a multi-core IDF (`freertos_idf_additions_priv.h`:
/// `prvENTER_CRITICAL_OR_SUSPEND_ALL`) and calls `vTaskGetInfo(…, pdTRUE, …)`
/// for every task, which walks each stack byte by byte for its
/// high-water mark. For those milliseconds core 0's USB interrupt waits,
/// the host controller's next isochronous buffer goes in late, and its
/// first 1-4 packets come back `USB_TRANSFER_STATUS_SKIPPED` — audio that
/// is never counted. Measured on an IC-705 (2026-09-22,
/// `logs/live_ft8_62e0bee6_2026-09-22.log`): every `Bad RX Isoc packet …
/// status 6` burst followed a `[cpu]` line, and the FT8 slot grid fell
/// 36 samples behind UTC per burst. [`log_task_stacks`] is the same call.
///
/// Each task's CPU since the last call, as a percentage of one core —
/// from FreeRTOS's run-time counters (`CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`,
/// esp_timer microseconds). The busiest ten, one line.
///
/// For the question wall-clock timers inside a task cannot answer once
/// that task sleeps on DMA or is preempted: how much of a core it
/// actually used. Added for the FT4 panel (2026-09-21), whose drawing
/// is DMA it sleeps through, so its own timers read wall time.
/// Idle share of each core since the last call — the two numbers
/// `log_task_cpu` can give without `uxTaskGetSystemState`: one counter
/// read per core under the kernel lock, no task walk, no stack scan.
/// What the panel prints while USB audio streams.
pub fn log_idle_cpu() {
    use esp_idf_svc::sys;
    use std::sync::Mutex;
    static PREV: Mutex<(i64, u32, u32)> = Mutex::new((0, 0, 0));
    // SAFETY: core ids 0 and 1 exist on the ESP32-S3.
    let (i0, i1) = unsafe {
        (
            sys::ulTaskGetIdleRunTimeCounterForCore(0),
            sys::ulTaskGetIdleRunTimeCounterForCore(1),
        )
    };
    let now = unsafe { sys::esp_timer_get_time() };
    let Ok(mut prev) = PREV.lock() else {
        return;
    };
    let (t, p0, p1) = *prev;
    *prev = (now, i0, i1);
    drop(prev);
    let wall = now - t;
    if t == 0 || wall <= 0 {
        return;
    }
    log::info!(
        "[cpu] IDLE0:{}% IDLE1:{}% (per-task off while USB audio streams)",
        (i0.wrapping_sub(p0) as i64) * 100 / wall,
        (i1.wrapping_sub(p1) as i64) * 100 / wall,
    );
}

pub fn log_task_cpu() {
    use esp_idf_svc::sys;
    use std::sync::Mutex;
    const MAX_TASKS: usize = 40;
    static PREV: Mutex<(i64, heapless::Vec<(usize, u32), MAX_TASKS>)> =
        Mutex::new((0, heapless::Vec::new()));

    let mut tasks: [sys::TaskStatus_t; MAX_TASKS] = unsafe { core::mem::zeroed() };
    let mut total: u32 = 0;
    // SAFETY: `tasks` is `MAX_TASKS` correctly-typed entries.
    let n = unsafe {
        sys::uxTaskGetSystemState(tasks.as_mut_ptr(), MAX_TASKS as sys::UBaseType_t, &mut total)
    } as usize;
    let now = unsafe { sys::esp_timer_get_time() };
    let Ok(mut prev) = PREV.lock() else {
        return;
    };
    let wall = now - prev.0;
    let mut rows: heapless::Vec<(u32, heapless::String<20>), MAX_TASKS> = heapless::Vec::new();
    let mut next: heapless::Vec<(usize, u32), MAX_TASKS> = heapless::Vec::new();
    for t in tasks.iter().take(n) {
        let h = t.xHandle as usize;
        let before = prev.1.iter().find(|(k, _)| *k == h).map(|(_, v)| *v);
        let _ = next.push((h, t.ulRunTimeCounter));
        if let Some(b) = before {
            let mut name: heapless::String<20> = heapless::String::new();
            // SAFETY: FreeRTOS task names are NUL-terminated C strings.
            let cname = unsafe { core::ffi::CStr::from_ptr(t.pcTaskName) };
            let _ = name.push_str(cname.to_str().unwrap_or("?"));
            let _ = rows.push((t.ulRunTimeCounter.wrapping_sub(b), name));
        }
    }
    let first = prev.0 == 0;
    *prev = (now, next);
    drop(prev);
    if first || wall <= 0 {
        return;
    }
    rows.sort_unstable_by(|a, b| b.0.cmp(&a.0));
    let mut line: heapless::String<256> = heapless::String::new();
    use core::fmt::Write as _;
    for (d, name) in rows.iter().take(10) {
        let _ = write!(line, " {}:{}%", name, (*d as i64) * 100 / wall);
    }
    log::info!("[cpu]{line}");
}

pub fn log_task_stacks() {
    use esp_idf_svc::sys;

    /// Enough for the IDF's own tasks (WiFi, lwIP, USB host, timers,
    /// idle x2, ipc x2) plus this app's. Overflow is reported rather
    /// than silently truncated.
    const MAX_TASKS: usize = 40;
    /// Anything with less than this much left gets named individually.
    const TIGHT_BYTES: u32 = 2048;

    let mut tasks: [sys::TaskStatus_t; MAX_TASKS] = unsafe { core::mem::zeroed() };
    // SAFETY: `tasks` is `MAX_TASKS` correctly-typed entries; a null
    // run-time pointer is allowed and skips run-time accounting.
    let n = unsafe {
        sys::uxTaskGetSystemState(
            tasks.as_mut_ptr(),
            MAX_TASKS as sys::UBaseType_t,
            core::ptr::null_mut(),
        )
    } as usize;
    if n == 0 {
        log::warn!("[stacks] uxTaskGetSystemState returned 0 — is FREERTOS_USE_TRACE_FACILITY on?");
        return;
    }

    let name_of = |t: &sys::TaskStatus_t| -> heapless::String<20> {
        let mut s: heapless::String<20> = heapless::String::new();
        if t.pcTaskName.is_null() {
            let _ = s.push_str("?");
            return s;
        }
        // SAFETY: FreeRTOS keeps the name alive for the task's life,
        // and these entries were populated moments ago.
        for i in 0..20 {
            let c = unsafe { *t.pcTaskName.add(i) };
            if c == 0 {
                break;
            }
            if s.push(c as u8 as char).is_err() {
                break;
            }
        }
        s
    };

    let mut worst = u32::MAX;
    let mut worst_name: heapless::String<20> = heapless::String::new();
    let mut tight: heapless::String<96> = heapless::String::new();
    for t in tasks.iter().take(n) {
        let hw = t.usStackHighWaterMark;
        if hw < worst {
            worst = hw;
            worst_name = name_of(t);
        }
        if hw < TIGHT_BYTES {
            use core::fmt::Write as _;
            let _ = write!(&mut tight, " {}:{hw}", name_of(t));
        }
    }

    // SAFETY: null = the calling task.
    let self_hw = unsafe { sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) };
    log::info!(
        "[stacks] {n} tasks{}, min {worst_name}:{worst}, tight(<{TIGHT_BYTES}):{}, self:{self_hw}",
        if n == MAX_TASKS { " (TRUNCATED)" } else { "" },
        if tight.is_empty() {
            " none"
        } else {
            tight.as_str()
        }
    );

    // And the other end of the same list. The summary above answers
    // "is anything about to overflow"; this answers "what is being
    // wasted", which is the question when internal DRAM is the
    // constraint — every KiB of task stack is a KiB the decoder's own
    // allocations do not get, and on this board that is the difference
    // between a 2 304-point workspace in internal DRAM and the same
    // workspace in PSRAM (41 % slower, `FT4_BENCHMARK_LOG.md` §26.3).
    //
    // One task per line rather than a joined string: this is diagnostic
    // output on a 30 s cadence, and the names are what get read.
    // **Priority and core, beside the stack.** `TaskStatus_t` carries
    // both and they were being discarded, so every priority argument in
    // this tree was an argument about the value *passed* rather than the
    // one in effect — `uac_host_install`'s `task_priority`, a pthread's
    // `ThreadSpawnConfiguration`, `xTaskCreatePinnedToCore`'s core, all
    // asserted and none observed. That mattered on 2026-09-19, when
    // audio was being lost during every decode and "the class driver
    // is not being scheduled" could not be told from "the class driver
    // runs fine and the data is dropped elsewhere". `xCoreID` reads
    // `tskNO_AFFINITY` (0x7FFFFFFF) as `-` .
    for t in tasks.iter().take(n) {
        // `TaskStatus_t` carries the priority but not the core in this
        // binding; `xTaskGetCoreID` does, and returns `tskNO_AFFINITY`
        // (0x7FFFFFFF) for an unpinned task.
        // SAFETY: the handle was populated moments ago by
        // `uxTaskGetSystemState` and this task has not yielded since.
        let raw = unsafe { sys::xTaskGetCoreID(t.xHandle) };
        let core = if raw as u32 == 0x7FFF_FFFF { -1 } else { raw as i32 };
        log::info!(
            "[stacks]   {:<16} free {:<6} prio {:<2} base {:<2} core {}",
            name_of(t).as_str(),
            t.usStackHighWaterMark,
            t.uxCurrentPriority,
            t.uxBasePriority,
            core,
        );
    }
}
