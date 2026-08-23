//! AXP2101 + AW9523B bring-up for M5Stack CoreS3 (Phase 0-Core).
//!
//! AXP2101 (0x34): main PMIC. Phase 0 verified presence via Chip-ID
//! (reg 0x03 → 0x4A for AXP2101) and *assumed* power-on defaults were
//! enough for every rail. **That assumption was wrong** (found
//! 2026-08-15, real hardware: PMIC/AW9523B/SPI/mipidsi every step
//! logged success, but the physical panel stayed completely dark —
//! not even a backlight glow). Cross-checked against M5Stack's own
//! `M5GFX.cpp` (`github.com/m5stack/M5GFX`, `M5StackCoreS3`
//! `setBrightness`): the LCD backlight is powered by AXP2101's
//! **DLDO1** rail, enabled via register `0x90` bit `0x80` and its
//! voltage set via register `0x99` — a rail this module never touched
//! at all, so DLDO1 stayed at whatever its power-on-reset state
//! happened to be (evidently off, or too low to matter). `chip-id`
//! is a read-only sanity check and says nothing about which rails are
//! actually enabled; this file now writes the same two registers
//! M5GFX does before returning.
//!
//! AW9523B (0x58): I/O expander. Phase 0 configures:
//!   - Port 0 direction: P0_0 (TP_INT) = input, rest = output.
//!   - Port 1 direction: all outputs.
//!   - Safe defaults: BUS_OUT_EN=LOW (no USB VBUS), SPK_EN=LOW.
//!   - LCD_BL (P0_4) = HIGH. **Also found wrong 2026-08-15**: M5GFX's
//!     CoreS3 code never touches an AW9523 pin for backlight at all —
//!     P0_4 here does not correspond to backlight control on this
//!     board (kept as a no-op safe-default write; not removed, since
//!     an unverified guess about what P0_4 *actually* drives isn't
//!     worth swapping in for another).
//!   - LCD_RST/TP_RST: LOW → delay → HIGH → reset cycle. Which
//!     individual Port-1 bit is which was also swapped versus M5GFX
//!     (`AW9523_P1_LCD_RST`/`AW9523_P1_TP_RST` below) — harmless here
//!     since both bits are driven together, but fixed for when
//!     Phase 6-Core (touch) needs to toggle TP_RST alone.
//!
//! I2C bus is returned to the caller; display.rs holds it for the reset
//! delay and then drops it (Phase 6-Core touch driver will reclaim it).

use std::sync::atomic::{AtomicU16, AtomicU8, Ordering};

use anyhow::{anyhow, Result};
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Gpio11, Gpio12},
    i2c::{I2cConfig, I2cDriver, I2C0},
    units::FromValueType,
};

use crate::board::{
    AW9523B_I2C_ADDR, AW9523_P0_BUS_OUT_EN, AW9523_P0_LCD_BL, AW9523_P0_USB_OTG_EN,
    AW9523_P1_BOOST_EN, AW9523_P1_LCD_RST, AW9523_P1_TP_RST, AXP2101_I2C_ADDR,
};

// AW9523B register map (AW9523B datasheet §6).
const AW9523_REG_IN0: u8 = 0x00; // Input port 0 (read)
const AW9523_REG_IN1: u8 = 0x01; // Input port 1 (read)
const AW9523_REG_OUT0: u8 = 0x02; // Output port 0 latch
const AW9523_REG_OUT1: u8 = 0x03; // Output port 1 latch
const AW9523_REG_CFG0: u8 = 0x04; // Port 0 direction: 0=output, 1=input
const AW9523_REG_CFG1: u8 = 0x05; // Port 1 direction: 0=output, 1=input
const AW9523_REG_GCR: u8 = 0x11; // Global config (push-pull vs open-drain)
const AW9523_REG_LEDMODE0: u8 = 0x12; // Port 0 mode: 1=GPIO, 0=LED (current sink)
const AW9523_REG_LEDMODE1: u8 = 0x13; // Port 1 mode: 1=GPIO, 0=LED

// AXP2101 register map (AXP2101 datasheet).
const AXP2101_REG_CHIP_ID: u8 = 0x03; // Chip ID — 0x4A for AXP2101
/// PMU status 1. Bit 5 reads high while VBUS is present and in range
/// ("VBUS good"), which on this board means *something external is
/// supplying the USB-C port* — a charger, or a PC.
const AXP2101_REG_STATUS1: u8 = 0x00;
const AXP2101_STATUS1_VBUS_GOOD: u8 = 1 << 5;
const AXP2101_REG_LDO_ONOFF0: u8 = 0x90; // LDOs ON/OFF control 0 — bit 0x80 = DLDO1 enable
const AXP2101_REG_DLDO1_VOLTAGE: u8 = 0x99; // DLDO1 (LCD backlight) voltage setting
const AXP2101_DLDO1_ENABLE_BIT: u8 = 0x80;
/// `33 - 5`, matching M5GFX's own encoding for these AXP2101 LDO
/// voltage registers (same formula it uses for ALDO3/ALDO4, both
/// documented there as 3.3 V): register value `V` → `0.5 V + 0.1 V ×
/// V`, so `28` is 3.3 V. Backlight brightness isn't otherwise exposed
/// by this app yet — a fixed "on, reasonably bright" value is enough
/// to prove/keep the panel lit.
const AXP2101_DLDO1_VOLTAGE_3V3: u8 = 28;

const I2C_TIMEOUT_TICKS: u32 = 100;

fn read_reg(i2c: &mut I2cDriver<'_>, addr: u8, reg: u8) -> Result<u8> {
    let mut buf = [0u8; 1];
    i2c.write_read(addr, &[reg], &mut buf, I2C_TIMEOUT_TICKS)
        .map_err(|e| anyhow!("I2C read 0x{addr:02x} reg 0x{reg:02x} failed: {e}"))?;
    Ok(buf[0])
}

fn write_reg(i2c: &mut I2cDriver<'_>, addr: u8, reg: u8, val: u8) -> Result<()> {
    i2c.write(addr, &[reg, val], I2C_TIMEOUT_TICKS)
        .map_err(|e| anyhow!("I2C write 0x{addr:02x} reg 0x{reg:02x}={val:#04x} failed: {e}"))
}

pub fn scan(i2c: &mut I2cDriver<'_>) {
    log::info!("I2C0 bus scan:");
    for addr in 0x08u8..=0x77 {
        if i2c.write(addr, &[0u8], I2C_TIMEOUT_TICKS).is_ok() {
            log::info!("  device @ 0x{addr:02x}");
        }
    }
}

/// Initialise I2C0, scan the bus, bring up AXP2101 + AW9523B for LCD.
/// Returns the driver; the caller holds it until LCD SPI init completes,
/// then may drop it (Phase 6-Core touch will reclaim the bus).
pub fn init<'d>(i2c0: I2C0<'d>, sda: Gpio12<'d>, scl: Gpio11<'d>) -> Result<I2cDriver<'d>> {
    // 100 kHz, not 400.
    //
    // The FT5x06 touch controller does not answer at 400 kHz — it
    // never appeared in a bus scan, at any point in the boot, with its
    // reset verified released and every rail M5GFX enables turned on.
    // M5GFX itself runs the bus at 400 kHz but drops to 100 kHz for
    // every FT5x06 transfer:
    //
    // ```c
    // static constexpr int32_t i2c_freq                = 400000;
    // static constexpr int32_t ft5x06_version_i2c_freq = 100000;
    // ```
    //
    // esp-idf-hal's `I2cDriver` fixes the clock at construction, so
    // rather than carry two drivers for one bus, the whole bus runs at
    // the speed its slowest device needs. Nothing here is
    // throughput-bound: the AXP2101 and the AW9523B see a handful of
    // register accesses at boot and a touch poll at 2 Hz, and the LCD
    // is on SPI. Refs #163.
    let cfg = I2cConfig::new().baudrate(100u32.kHz().into());
    let mut i2c = I2cDriver::new(i2c0, sda, scl, &cfg)
        .map_err(|e| anyhow!("I2C0 driver init failed: {e}"))?;

    scan(&mut i2c);

    // ── AXP2101 presence check ────────────────────────────────────────
    match read_reg(&mut i2c, AXP2101_I2C_ADDR, AXP2101_REG_CHIP_ID) {
        Ok(id) => log::info!("AXP2101 chip-id=0x{id:02x} (expect 0x4A)"),
        Err(e) => log::warn!("AXP2101 read failed: {e:#}"),
    }

    // ── AXP2101 DLDO1 (LCD backlight power rail) ────────────────────────
    // Read-modify-write, not a blind overwrite: register 0x90 also
    // gates other LDOs (ALDO1-4 etc.) that may already be correctly
    // configured by power-on defaults — only the DLDO1 bit should
    // change here. Matches M5GFX's `bitOn`/`setBrightness` sequence
    // for CoreS3 (see this module's doc comment).
    match read_reg(&mut i2c, AXP2101_I2C_ADDR, AXP2101_REG_LDO_ONOFF0) {
        Ok(cur) => {
            let new_val = cur | AXP2101_DLDO1_ENABLE_BIT;
            if let Err(e) = write_reg(&mut i2c, AXP2101_I2C_ADDR, AXP2101_REG_LDO_ONOFF0, new_val) {
                log::warn!("AXP2101 DLDO1 enable (reg 0x90) failed: {e:#}");
            } else {
                log::info!("AXP2101 reg 0x90: 0x{cur:02x} -> 0x{new_val:02x} (DLDO1 enabled)");
            }
        }
        Err(e) => log::warn!("AXP2101 reg 0x90 read failed: {e:#} — DLDO1 enable skipped"),
    }
    if let Err(e) = write_reg(
        &mut i2c,
        AXP2101_I2C_ADDR,
        AXP2101_REG_DLDO1_VOLTAGE,
        AXP2101_DLDO1_VOLTAGE_3V3,
    ) {
        log::warn!("AXP2101 DLDO1 voltage (reg 0x99) failed: {e:#}");
    } else {
        log::info!("AXP2101 reg 0x99 = {AXP2101_DLDO1_VOLTAGE_3V3} (DLDO1 ~3.3V, backlight power)");
    }

    // Battery-voltage ADC on, so `battery_mv` has something to read.
    let adc = read_reg(&mut i2c, AXP2101_I2C_ADDR, AXP2101_REG_ADC_EN).unwrap_or(0);
    // bit0 = VBAT, bit2 = VBUS, bit3 = VSYS.
    write_reg(&mut i2c, AXP2101_I2C_ADDR, AXP2101_REG_ADC_EN, adc | 0x0D)?;

    // ── AW9523B init ──────────────────────────────────────────────────
    // Values taken verbatim from M5Stack's own `M5GFX.cpp` (CoreS3 board
    // section), rather than derived from the pin table in `board.rs` —
    // that table has now been wrong twice (LCD_BL, and the whole USB
    // VBUS group), and the vendor's init sequence is the ground truth
    // for a board we cannot probe.
    //
    //   0x04 CONFIG_P0 = 0b00011000   P0_3/P0_4 input, rest output
    //   0x05 CONFIG_P1 = 0b00001100   P1_2/P1_3 input, rest output
    //   0x11 GCR       = 0b00010000   port 0 push-pull
    //   0x12/0x13 LEDMODE = 0xFF      every pin in GPIO mode
    //
    // The LEDMODE registers matter more than they look: a pin left in
    // LED mode is a constant-current sink, so writing its output latch
    // does nothing an attached load can feel. Nothing here had ever
    // written them.
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_GCR, 0x10)?;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_CFG0, 0b0001_1000)?;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_CFG1, 0b0000_1100)?;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_LEDMODE0, 0xFF)?;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_LEDMODE1, 0xFF)?;

    // Port 0, matching M5GFX's own CoreS3 bring-up.
    //
    // It writes `bitOn(0x02, 0b00000101)` — bits 0 and 2 — before it
    // touches anything else, and this code never set either. Their
    // names are not in the source (only bit 1, BUS_OUT_EN, is
    // conditional there, for VBUS), so they are carried across as
    // "required, purpose unnamed" rather than guessed at. What is
    // certain is that the FT5x06 does not answer without the rest of
    // M5's sequence, and this is the part of it that was missing.
    //
    // `CFG0 = 0b00011000` above makes P0_0 an *output*, which also
    // settles a stale comment in `board.rs` calling it the touch
    // interrupt input; M5GFX puts TP_INT on GPIO 21, not on the
    // expander at all.
    const AW9523_P0_M5_REQUIRED: u8 = 0b0000_0101;
    let p0_out = AW9523_P0_LCD_BL | AW9523_P0_M5_REQUIRED;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT0, p0_out)?;
    log::info!("AW9523B port0 out=0x{p0_out:02x} (LCD_BL + M5GFX bits 0/2, BUS_OUT_EN=OFF)");

    // Port 1: assert LCD_RST and TP_RST LOW first (active-low reset).
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT1, 0x00)?;
    FreeRtos::delay_ms(20);
    // Release resets (HIGH = normal operation).
    let p1_out = AW9523_P1_LCD_RST | AW9523_P1_TP_RST;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT1, p1_out)?;
    FreeRtos::delay_ms(100);
    log::info!("AW9523B LCD_RST + TP_RST cycle complete");

    // Rails last, exactly where M5GFX puts them — after the expander
    // is configured and its outputs are driven. Doing it later, from
    // the display loop, meant the touch controller had its reset
    // released while it was still unpowered and never saw a reset edge
    // afterwards.
    if let Err(e) = apply_peripheral_rails(&mut i2c) {
        log::warn!("peripheral rail bring-up failed: {e:#}");
    }

    Ok(i2c)
}

/// Drive AW9523B P0_1 (BUS_OUT_EN) HIGH to enable USB VBUS boost.
/// Must be called BEFORE `usb_host_install()` — omission leaves VBUS
/// floating and the host stack sees no device enumeration.
/// Is something external powering the USB-C port right now?
///
/// This board has one USB-C connector and it cannot both take power in
/// and hand power out. Asserting `BUS_OUT_EN` while a PC is already
/// driving VBUS puts the board's boost in opposition to the host's
/// supply — and on a battery that is not full, the board browns out.
/// That is not theory: it is how a whole bench session was lost
/// (2026-08-22, issue #163). The battery ran flat because the host
/// firmware never charges, then re-flashing failed mid-write with a
/// broken pipe because the board kept dying as soon as it booted.
///
/// So the decision "am I a host or a peripheral" is not a build-time
/// choice, it is a question about the cable that is plugged in, and
/// this is how to ask it.
///
/// Returns the raw register alongside the verdict — the bit position
/// is from the AXP2101 datasheet rather than measured on this board,
/// so the caller logs both and the reader can check the claim.
/// AXP2101 ADC channel enable. bit0 = VBAT, bit2 = VBUS, bit3 = VSYS.
const AXP2101_REG_ADC_EN: u8 = 0x30;
/// Battery-voltage ADC result, 14-bit, big-endian, already in mV.
const AXP2101_REG_VBAT_H: u8 = 0x34;
/// VBUS-voltage ADC result, same encoding.
const AXP2101_REG_VBUS_H: u8 = 0x38;

/// Battery voltage in mV, or `None` if the read failed.
///
/// On battery the 5 V boost that feeds USB host VBUS runs off this
/// rail, so "the host enumerates nothing" and "the pack is too flat to
/// boost" are worth being able to tell apart without a meter.
pub fn battery_mv(i2c: &mut I2cDriver<'_>) -> Option<u16> {
    let hi = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_VBAT_H).ok()?;
    let lo = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_VBAT_H + 1).ok()?;
    let mv = (((hi & 0x3F) as u16) << 8) | lo as u16;
    BATTERY_MV.store(mv, Ordering::Relaxed);
    Some(mv)
}

/// VBUS 電圧 (mV)。
///
/// **これがホストモードの決定打になる。** USB-C コネクタの VBUS ピンは
/// AXP2101 の VBUS 入力と同じネットなので、自前の昇圧出力が本当に
/// コネクタへ乗っていれば、ここが約 5000 mV を示す。出力レジスタの
/// 読み戻しは「そう書いた」以上のことを何も保証しないが、これは
/// 電圧そのものの測定で、テスターを当てるのと同じ意味を持つ。
///
/// Refs #163.
pub fn vbus_mv(i2c: &mut I2cDriver<'_>) -> Option<u16> {
    let hi = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_VBUS_H).ok()?;
    let lo = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_VBUS_H + 1).ok()?;
    let mv = (((hi & 0x3F) as u16) << 8) | lo as u16;
    VBUS_MV.store(mv, Ordering::Relaxed);
    Some(mv)
}

/// 画面用: 直近の VBUS 電圧 (mV)。
pub fn vbus_mv_cached() -> u16 {
    VBUS_MV.load(Ordering::Relaxed)
}

static VBUS_MV: AtomicU16 = AtomicU16::new(0);

/// 直近に読んだ電池電圧 (mV)。0 = まだ読めていない。
static BATTERY_MV: AtomicU16 = AtomicU16::new(0);

/// 画面用: 直近の電池電圧 (mV)。
pub fn battery_mv_cached() -> u16 {
    BATTERY_MV.load(Ordering::Relaxed)
}

pub fn vbus_present(i2c: &mut I2cDriver<'_>) -> Result<(bool, u8)> {
    let raw = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_STATUS1)?;
    AXP_STATUS1.store(raw, Ordering::Relaxed);
    Ok((raw & AXP2101_STATUS1_VBUS_GOOD != 0, raw))
}

/// VBUS 有効化を試みたか (0 = まだ)。ペリフェラルモードでは 0 のまま。
static VBUS_ATTEMPTED: AtomicU8 = AtomicU8::new(0);
/// 有効化後に読み戻した AW9523B の出力レジスタ (port0 / port1)。
/// BOOST_EN (P1_7) と USB_OTG_EN (P0_5) の両方が立っていることを
/// 画面で確認できるよう、加工せず生のまま持つ。
static VBUS_OUT0: AtomicU8 = AtomicU8::new(0);
static VBUS_OUT1: AtomicU8 = AtomicU8::new(0);
/// 直近に読んだ AXP2101 STATUS1 (bit5 = VBUS 入力あり)。
static AXP_STATUS1: AtomicU8 = AtomicU8::new(0);

/// (VBUS 有効化を試みたか, port0, port1, AXP2101 STATUS1)。
/// Read every rail and expander pin that could explain a missing
/// device, and say what is actually in the registers.
///
/// Written because the FT6336U does not answer at 0x38 even after
/// `init` releases TP_RST, and the obvious next move — start guessing
/// which AW9523B pin the reset really is — is the move that cost a
/// night on the USB VBUS enables (#163). The pin theory is already
/// weak: `init` drives `LCD_RST | TP_RST` together, so a swap between
/// those two bits would release both either way.
///
/// The likelier shape is a rail nobody turns on. `init` only touches
/// DLDO1, for the backlight; the LCD working says nothing about a
/// panel controller on a different LDO. This dump is read-only and
/// answers that before anything is written.
/// Bring the peripheral rails up the way M5GFX does.
///
/// Ported from M5GFX's own CoreS3 autodetect (`src/M5GFX.cpp`), which
/// writes exactly this before it touches the FT5x06:
///
/// ```c
/// writeRegister8(i2c_port, axp_i2c_addr, 0x90, 0xBF);
/// writeRegister8(i2c_port, axp_i2c_addr, 0x94, 33 - 5);
/// writeRegister8(i2c_port, axp_i2c_addr, 0x95, 33 - 5);
/// ```
///
/// `init` had only ever enabled DLDO1, for the backlight — enough to
/// get a picture, and the reason the touch controller never answered
/// at 0x38. Measured on this board before the fix: `0x90 = 0x8b`
/// (ALDO1/2/4 + DLDO1), against M5's `0xBF`, so **ALDO3, BLDO1 and
/// BLDO2 were all off**. Guessing at ALDO3 alone — the one 3.3 V rail
/// that was configured-but-off — was not enough, which is the whole
/// argument for reading the vendor's source instead of eliminating
/// candidates one flash at a time.
///
/// 0x94/0x95 set ALDO3/ALDO4 to 3.3 V (code 28); this board already
/// held 0x1c there, so those two writes are a no-op that is kept for
/// fidelity to the source they came from.
///
/// Bit 6 (CPUSLDO) stays off in M5's value too.
pub fn apply_peripheral_rails(i2c: &mut I2cDriver<'_>) -> Result<()> {
    const M5GFX_LDO_ONOFF0: u8 = 0xBF;
    const ALDO3_VOLT: u8 = 0x94;
    const ALDO4_VOLT: u8 = 0x95;
    const VOLT_3V3: u8 = 33 - 5;

    let before = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_LDO_ONOFF0)?;
    write_reg(i2c, AXP2101_I2C_ADDR, ALDO3_VOLT, VOLT_3V3)?;
    write_reg(i2c, AXP2101_I2C_ADDR, ALDO4_VOLT, VOLT_3V3)?;
    write_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_LDO_ONOFF0, M5GFX_LDO_ONOFF0)?;
    FreeRtos::delay_ms(50);
    let after = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_LDO_ONOFF0)?;
    log::info!(
        "AXP2101 peripheral rails (M5GFX sequence): 0x90 0x{before:02x} -> 0x{after:02x} \
         (want 0x{M5GFX_LDO_ONOFF0:02x})"
    );
    Ok(())
}

pub fn dump_rails(i2c: &mut I2cDriver<'_>) {
    // AXP2101 LDO enables: 0x90 bit per rail, 0x91 for DLDO2.
    // Voltage select regs run 0x92..0x99 in the same order.
    const NAMES: [&str; 8] = [
        "ALDO1", "ALDO2", "ALDO3", "ALDO4", "BLDO1", "BLDO2", "CPUSLDO", "DLDO1",
    ];
    match read_reg(i2c, AXP2101_I2C_ADDR, 0x90) {
        Ok(en) => {
            let mut line = String::new();
            for (i, name) in NAMES.iter().enumerate() {
                let on = en & (1 << i) != 0;
                let mv = read_reg(i2c, AXP2101_I2C_ADDR, 0x92 + i as u8).unwrap_or(0xFF);
                let _ = core::fmt::Write::write_fmt(
                    &mut line,
                    format_args!(" {name}={}({mv:#04x})", if on { "ON" } else { "off" }),
                );
            }
            log::info!("AXP2101 LDO enable 0x90=0x{en:02x}:{line}");
        }
        Err(e) => log::warn!("AXP2101 0x90 read failed: {e:#}"),
    }

    // The expander's real state, not the cached copy `power_state`
    // returns — that one is only filled when USB host VBUS is enabled,
    // so in peripheral mode it reads as zeros and means nothing.
    for (reg, name) in [
        (AW9523_REG_OUT0, "OUT0"),
        (AW9523_REG_OUT1, "OUT1"),
        (AW9523_REG_CFG0, "CFG0"),
        (AW9523_REG_CFG1, "CFG1"),
    ] {
        match read_reg(i2c, AW9523B_I2C_ADDR, reg) {
            Ok(v) => log::info!("AW9523B {name}=0x{v:02x}"),
            Err(e) => log::warn!("AW9523B {name} read failed: {e:#}"),
        }
    }
}

pub fn power_state() -> (bool, u8, u8, u8) {
    (
        VBUS_ATTEMPTED.load(Ordering::Relaxed) != 0,
        VBUS_OUT0.load(Ordering::Relaxed),
        VBUS_OUT1.load(Ordering::Relaxed),
        AXP_STATUS1.load(Ordering::Relaxed),
    )
}

pub fn enable_usb_host_vbus(i2c: &mut I2cDriver<'_>) -> Result<()> {
    // Boost first, then the gate. P1_7 runs the converter that makes
    // the 5 V; P0_5 puts that rail on the USB connector. The other
    // order hands the connector a supply that is still ramping.
    //
    // M5Unified writes both registers in one 2-byte transaction; doing
    // it in two with a settling delay is the same thing electrically,
    // and lets the boost stabilise first.
    let p1 = read_reg(i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT1)?;
    write_reg(
        i2c,
        AW9523B_I2C_ADDR,
        AW9523_REG_OUT1,
        p1 | AW9523_P1_BOOST_EN,
    )?;
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);

    // Both gates, not just the USB one.
    //
    // M5Unified's `setUsbOutput` asserts USB_OTG_EN alone, and doing
    // exactly that leaves the connector's VBUS net dead (measured: the
    // AXP2101 VBUS ADC rails at full scale on battery, against 4932 mV
    // when a PC is supplying the port). The schematic shows the two
    // enables driving a bank of ME1502A load switches (U14/U17/U18/U19)
    // rather than one, with pins of U17/U19 appearing on both nets — so
    // a series arrangement needing both is plausible. BUS_OUT_EN is the
    // external 5 V rail, which costs nothing extra to have on while the
    // pack is full, and this is the cheapest remaining experiment that
    // the on-board voltmeter can settle. Refs #163.
    let p0 = read_reg(i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT0)?;
    write_reg(
        i2c,
        AW9523B_I2C_ADDR,
        AW9523_REG_OUT0,
        p0 | AW9523_P0_USB_OTG_EN | AW9523_P0_BUS_OUT_EN,
    )?;
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);

    let after0 = read_reg(i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT0)?;
    let after1 = read_reg(i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT1)?;
    VBUS_OUT0.store(after0, Ordering::Relaxed);
    VBUS_OUT1.store(after1, Ordering::Relaxed);
    VBUS_ATTEMPTED.store(1, Ordering::Relaxed);

    let boost = after1 & AW9523_P1_BOOST_EN != 0;
    let otg = after0 & AW9523_P0_USB_OTG_EN != 0;
    if boost && otg {
        log::info!(
            "AW9523B P1_7 (BOOST_EN) + P0_5 (USB_OTG_EN) HIGH — USB VBUS up \
             (p0=0x{after0:02x} p1=0x{after1:02x})"
        );
    } else {
        log::error!(
            "AW9523B VBUS readback incomplete: BOOST_EN={boost} USB_OTG_EN={otg} \
             (p0=0x{after0:02x} p1=0x{after1:02x}) — no USB device will enumerate"
        );
    }
    Ok(())
}
