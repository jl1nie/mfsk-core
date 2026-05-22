//! AXP2101 + AW9523B bring-up for M5Stack CoreS3 (Phase 0-Core).
//!
//! AXP2101 (0x34): main PMIC. Phase 0 verifies presence via Chip-ID
//! (reg 0x03 → 0x4A for AXP2101) and relies on power-on defaults for all
//! rails. Full rail tuning (ALDO voltages etc.) is a Phase 1-Core item.
//!
//! AW9523B (0x58): I/O expander. Phase 0 configures:
//!   - Port 0 direction: P0_0 (TP_INT) = input, rest = output.
//!   - Port 1 direction: all outputs.
//!   - Safe defaults: BUS_OUT_EN=LOW (no USB VBUS), SPK_EN=LOW.
//!   - LCD_BL (P0_4) = HIGH → backlight on.
//!   - LCD_RST (P1_0): LOW → delay → HIGH → ILI9342C reset cycle.
//!
//! I2C bus is returned to the caller; display.rs holds it for the reset
//! delay and then drops it (Phase 6-Core touch driver will reclaim it).

use anyhow::{anyhow, Result};
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Gpio11, Gpio12},
    i2c::{I2cConfig, I2cDriver, I2C0},
    units::FromValueType,
};

use crate::board::{
    AW9523B_I2C_ADDR, AW9523_P0_LCD_BL, AW9523_P1_LCD_RST, AW9523_P1_TP_RST, AXP2101_I2C_ADDR,
};

// AW9523B register map (AW9523B datasheet §6).
const AW9523_REG_IN0: u8 = 0x00;  // Input port 0 (read)
const AW9523_REG_IN1: u8 = 0x01;  // Input port 1 (read)
const AW9523_REG_OUT0: u8 = 0x02; // Output port 0 latch
const AW9523_REG_OUT1: u8 = 0x03; // Output port 1 latch
const AW9523_REG_CFG0: u8 = 0x04; // Port 0 direction: 0=output, 1=input
const AW9523_REG_CFG1: u8 = 0x05; // Port 1 direction: 0=output, 1=input
const AW9523_REG_GCR: u8  = 0x11; // Global config (push-pull vs open-drain)

// AXP2101 register map (AXP2101 datasheet).
const AXP2101_REG_CHIP_ID: u8 = 0x03; // Chip ID — 0x4A for AXP2101

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

    // ── AW9523B init ──────────────────────────────────────────────────
    // GCR[4] = 0: push-pull mode for port 0 (default is open-drain on
    // some silicon revisions; set explicitly to guarantee output drive).
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_GCR, 0x10)?;

    // Port 0 direction: P0_0 (TP_INT) = input (bit=1), rest = output.
    write_reg(&mut i2c, AW9523B_I2C_ADDR, AW9523_REG_CFG0, 0x01)?;
    // Port 1 direction: all outputs (P1_0=LCD_RST, P1_1=TP_RST, ...).
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
