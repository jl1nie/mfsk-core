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

use std::sync::atomic::{AtomicU8, Ordering};

use anyhow::{anyhow, Result};
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Gpio11, Gpio12},
    i2c::{I2cConfig, I2cDriver, I2C0},
    units::FromValueType,
};

use crate::board::{
    AW9523B_I2C_ADDR, AW9523_P0_BOOST_EN, AW9523_P0_BUS_OUT_EN, AW9523_P0_LCD_BL, AW9523_P1_LCD_RST, AW9523_P1_TP_RST,
    AXP2101_I2C_ADDR,
};

// AW9523B register map (AW9523B datasheet §6).
const AW9523_REG_IN0: u8 = 0x00; // Input port 0 (read)
const AW9523_REG_IN1: u8 = 0x01; // Input port 1 (read)
const AW9523_REG_OUT0: u8 = 0x02; // Output port 0 latch
const AW9523_REG_OUT1: u8 = 0x03; // Output port 1 latch
const AW9523_REG_CFG0: u8 = 0x04; // Port 0 direction: 0=output, 1=input
const AW9523_REG_CFG1: u8 = 0x05; // Port 1 direction: 0=output, 1=input
const AW9523_REG_GCR: u8 = 0x11; // Global config (push-pull vs open-drain)

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
    let cfg = I2cConfig::new().baudrate(400u32.kHz().into());
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

    // ── AW9523B init ──────────────────────────────────────────────────
    // GCR[4] = 0: push-pull mode for port 0 (default is open-drain on
    // some silicon revisions; set explicitly to guarantee output drive).
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_GCR, 0x10)?;

    // Port 0 direction: P0_0 (TP_INT) = input (bit=1), rest = output.
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_CFG0, 0x01)?;
    // Port 1 direction: all outputs (P1_0=TP_RST, P1_1=LCD_RST, ...).
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_CFG1, 0x00)?;

    // Safe default output state: BUS_OUT_EN (P0_1)=LOW (no USB VBUS,
    // Phase 1-Core enables it), SPK_EN (P0_7)=LOW; only LCD_BL on.
    let p0_out = AW9523_P0_LCD_BL;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT0, p0_out)?;
    log::info!("AW9523B port0 out=0x{p0_out:02x} (LCD_BL=ON, BUS_OUT_EN=OFF, SPK_EN=OFF)");

    // Port 1: assert LCD_RST and TP_RST LOW first (active-low reset).
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT1, 0x00)?;
    FreeRtos::delay_ms(20);
    // Release resets (HIGH = normal operation).
    let p1_out = AW9523_P1_LCD_RST | AW9523_P1_TP_RST;
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT1, p1_out)?;
    FreeRtos::delay_ms(100);
    log::info!("AW9523B LCD_RST + TP_RST cycle complete");

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
pub fn vbus_present(i2c: &mut I2cDriver<'_>) -> Result<(bool, u8)> {
    let raw = read_reg(i2c, AXP2101_I2C_ADDR, AXP2101_REG_STATUS1)?;
    AXP_STATUS1.store(raw, Ordering::Relaxed);
    Ok((raw & AXP2101_STATUS1_VBUS_GOOD != 0, raw))
}

/// VBUS 有効化を試みたか (0 = まだ)。ペリフェラルモードでは 0 のまま。
static VBUS_ATTEMPTED: AtomicU8 = AtomicU8::new(0);
/// 有効化後に読み戻した AW9523B port0 出力レジスタ。BOOST_EN と
/// BUS_OUT_EN の両方が立っていることを画面で確認できるように、
/// 加工せず生のまま持つ。
static VBUS_OUT0: AtomicU8 = AtomicU8::new(0);
/// 直近に読んだ AXP2101 STATUS1 (bit5 = VBUS 入力あり)。
static AXP_STATUS1: AtomicU8 = AtomicU8::new(0);

/// (VBUS 有効化を試みたか, port0 読み戻し, AXP2101 STATUS1)。
pub fn power_state() -> (bool, u8, u8) {
    (
        VBUS_ATTEMPTED.load(Ordering::Relaxed) != 0,
        VBUS_OUT0.load(Ordering::Relaxed),
        AXP_STATUS1.load(Ordering::Relaxed),
    )
}

pub fn enable_usb_host_vbus(i2c: &mut I2cDriver<'_>) -> Result<()> {
    // Boost first, then the switch. P0_2 runs the converter that makes
    // the 5 V; P0_1 connects its output to the port. Asserting them in
    // the other order hands the connector a rail that is still ramping.
    let current = read_reg(i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT0)?;
    write_reg(
        i2c,
        AW9523B_I2C_ADDR,
        AW9523_REG_OUT0,
        current | AW9523_P0_BOOST_EN,
    )?;
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);

    let new_val = current | AW9523_P0_BOOST_EN | AW9523_P0_BUS_OUT_EN;
    write_reg(i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT0, new_val)?;
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);

    // Read back. BUS_OUT_EN gates a boost converter that makes the
    // 5 V VBUS out of the battery, and a device that never sees VBUS
    // never pulls up D+ — which looks identical, from the host stack's
    // side, to "nothing is plugged in". Distinguishing "we asked for
    // VBUS" from "the pin is actually high" is worth one I2C read.
    let after = read_reg(i2c, AW9523B_I2C_ADDR, AW9523_REG_OUT0)?;
    VBUS_OUT0.store(after, Ordering::Relaxed);
    VBUS_ATTEMPTED.store(1, Ordering::Relaxed);

    let boost = after & AW9523_P0_BOOST_EN != 0;
    let sw = after & AW9523_P0_BUS_OUT_EN != 0;
    if boost && sw {
        log::info!(
            "AW9523B P0_2 (BOOST_EN) + P0_1 (BUS_OUT_EN) HIGH — USB VBUS up (out0=0x{after:02x})"
        );
    } else {
        log::error!(
            "AW9523B VBUS readback incomplete: BOOST_EN={boost} BUS_OUT_EN={sw} \
             (out0=0x{after:02x}, wrote 0x{new_val:02x}) — no USB device will enumerate"
        );
    }
    Ok(())
}
