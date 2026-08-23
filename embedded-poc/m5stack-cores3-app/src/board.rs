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
pub const LCD_WIDTH: u16 = 320;
pub const LCD_HEIGHT: u16 = 240;

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
//   P0_7 = SPK_EN      (AW88298 speaker amp enable; Phase 3-Core)
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
pub const AW9523_P0_SPK_EN: u8 = 1 << 7;
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
    use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;

    let cfg = ThreadSpawnConfiguration {
        name: Some(name),
        stack_size,
        ..Default::default()
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
}
