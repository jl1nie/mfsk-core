//! AXP192 PMIC bring-up for M5Stack Core2.
//!
//! Equivalent role to the sibling s3-app's `pmic.rs` (which talks to
//! M5PM1). Sequence is cribbed from M5Unified `M5Core2.cpp` minus the
//! battery / charger fine-tuning we don't need for Phase 2:
//!
//! 1. Set LDO2 = 3.3 V (LCD VDD + backlight) and LDO3 = 2.0 V
//!    (vibration motor — held off in software via GPIO control later).
//! 2. Set DCDC3 = 2.8 V (LCD logic rail).
//! 3. Enable LDO2, LDO3, DCDC1, DCDC3, EXTEN — register 0x12 = 0xFF.
//! 4. Configure AXP192 GPIO4 as NMOS open-drain output and pulse it
//!    low → high to reset the ILI9342C panel (the panel's RST line is
//!    wired to AXP192 GPIO4, not an ESP32 pin).
//!
//! All I2C writes assume the bus has been initialised by the caller
//! at the standard 400 kHz Core2 default. `init_lcd_power` returns
//! the same `I2cDriver` so the caller can keep it for IMU / touch.

use anyhow::{anyhow, Result};
use esp_idf_hal::{
    delay::FreeRtos,
    gpio::{Gpio21, Gpio22},
    i2c::{I2cConfig, I2cDriver, I2C0},
    units::FromValueType,
};

use crate::board::AXP192_I2C_ADDR;

/// Bit in reg 0x96 for the GPIO4 output level. (GPIO3 = bit0,
/// GPIO4 = bit1, per AXP192 datasheet.)
const GPIO4_STATE_BIT: u8 = 1 << 1;

/// Register addresses we touch. Names track the AXP192 datasheet.
const REG_POWER_OUT_CTL: u8 = 0x12;
const REG_DCDC3_VOLT: u8 = 0x27;
const REG_LDO2_LDO3_VOLT: u8 = 0x28;
const REG_GPIO34_FUNCTION: u8 = 0x95;
const REG_GPIO34_STATE: u8 = 0x96;
const REG_CHIP_ID: u8 = 0x03;

const I2C_TIMEOUT_TICKS: u32 = 100;

fn read_reg(i2c: &mut I2cDriver<'_>, reg: u8) -> Result<u8> {
    let mut buf = [0u8; 1];
    i2c.write_read(AXP192_I2C_ADDR, &[reg], &mut buf, I2C_TIMEOUT_TICKS)
        .map_err(|e| anyhow!("AXP192 read reg 0x{reg:02x} failed: {e}"))?;
    Ok(buf[0])
}

fn write_reg(i2c: &mut I2cDriver<'_>, reg: u8, val: u8) -> Result<()> {
    i2c.write(AXP192_I2C_ADDR, &[reg, val], I2C_TIMEOUT_TICKS)
        .map_err(|e| anyhow!("AXP192 write reg 0x{reg:02x}={val:#04x} failed: {e}"))
}

/// I2C scan helper for the diag log line. Same shape as the s3-app
/// `pmic::scan` so the boot transcript reads the same on both boards.
pub fn scan(i2c: &mut I2cDriver<'_>) {
    log::info!("I2C bus scan:");
    for addr in 0x08u8..=0x77 {
        if i2c.write(addr, &[0u8], I2C_TIMEOUT_TICKS).is_ok() {
            log::info!("  device @ 0x{addr:02x}");
        }
    }
}

/// Bring up the I2C0 bus on the Core2 default pinout and run the LCD
/// power sequence on the AXP192. Returns the driver so the caller can
/// reuse the bus for IMU / touch later. Panics-by-Result; the caller
/// can log and continue without an LCD if we fail (the FanoutLogger
/// path still works without one).
pub fn init_lcd_power<'d>(
    i2c0: I2C0<'d>,
    sda: Gpio21<'d>,
    scl: Gpio22<'d>,
) -> Result<I2cDriver<'d>> {
    let cfg = I2cConfig::new().baudrate(400u32.kHz().into());
    let mut i2c = I2cDriver::new(i2c0, sda, scl, &cfg)
        .map_err(|e| anyhow!("I2C0 driver init failed: {e}"))?;

    scan(&mut i2c);

    // Optional: read chip id. AXP192 doesn't expose a strong ID byte,
    // but reading reg 0x03 (POWER_STATUS) is a cheap liveness probe.
    let status = read_reg(&mut i2c, REG_CHIP_ID)?;
    log::info!("AXP192 reg 0x03 (POWER_STATUS) = 0x{status:02x}");

    // ── LDO2 = 3.3 V (LCD VDD + backlight), LDO3 = 2.0 V ──────────────
    //   reg 0x28 = [LDO2 voltage upper nibble | LDO3 voltage lower nibble]
    //   step 0.1 V from 1.8 V base; LDO2=3.3 V → 0xF, LDO3=2.0 V → 0x2.
    let v_ldo23 = (0xFu8 << 4) | 0x2u8;
    write_reg(&mut i2c, REG_LDO2_LDO3_VOLT, v_ldo23)?;
    log::info!("AXP192 LDO2=3.3V LDO3=2.0V (reg 0x28={v_ldo23:#04x})");

    // ── DCDC3 = 2.8 V (LCD logic rail) ────────────────────────────────
    //   reg 0x27 step = (V-0.7)/0.025 → 2.8 V = 84 = 0x54.
    write_reg(&mut i2c, REG_DCDC3_VOLT, 0x54)?;
    log::info!("AXP192 DCDC3=2.8V (reg 0x27=0x54)");

    // ── Enable all power outputs (M5Unified Core2 default) ───────────
    //   bit0 DCDC1 / bit1 DCDC3 / bit2 LDO2 / bit3 LDO3 / bit4 DCDC2 /
    //   bit6 EXTEN. 0xFF == every rail on except the reserved bit5.
    write_reg(&mut i2c, REG_POWER_OUT_CTL, 0xFF)?;
    log::info!("AXP192 power outputs enabled (reg 0x12=0xFF)");

    // Settle the rails before driving the LCD reset.
    FreeRtos::delay_ms(20);

    // ── LCD reset via AXP192 GPIO4 (NMOS open-drain output) ──────────
    //   reg 0x95 bits[6:4] = GPIO4 function; 0b000 = NMOS open-drain
    //   output. Preserve the lower nibble (GPIO3 config).
    let g34 = read_reg(&mut i2c, REG_GPIO34_FUNCTION)?;
    let g34_new = g34 & 0x0F; // GPIO4 = NMOS open-drain output
    write_reg(&mut i2c, REG_GPIO34_FUNCTION, g34_new)?;

    // Drive LOW (assert RST).
    let g_state = read_reg(&mut i2c, REG_GPIO34_STATE)?;
    write_reg(&mut i2c, REG_GPIO34_STATE, g_state & !GPIO4_STATE_BIT)?;
    FreeRtos::delay_ms(20);
    // Release RST (HIGH).
    let g_state2 = read_reg(&mut i2c, REG_GPIO34_STATE)?;
    write_reg(&mut i2c, REG_GPIO34_STATE, g_state2 | GPIO4_STATE_BIT)?;
    FreeRtos::delay_ms(100);

    log::info!("AXP192 LCD reset cycle complete (GPIO4 LO→HI)");

    Ok(i2c)
}
