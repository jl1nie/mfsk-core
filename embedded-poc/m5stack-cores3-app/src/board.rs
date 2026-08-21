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
pub const TOUCH_I2C_ADDR: u8 = 0x38; // FT6336U (Phase 6-Core)
pub const IMU_I2C_ADDR: u8 = 0x69; // BMI270

// AW9523B pin assignments (M5Stack CoreS3 schematic):
//   P0_0 = TP_INT      (FT6336U interrupt, input)
//   P0_1 = BUS_OUT_EN  (USB host VBUS boost, HIGH=enable; Phase 1-Core)
//   P0_2 = BOOST_EN    (5V boost for USB VBUS)
//   P0_4 = LCD_BL      (kept as a documented guess only — see below)
//   P0_7 = SPK_EN      (AW88298 speaker amp enable; Phase 3-Core)
//   P1_0 = TP_RST      (FT6336U reset, active LOW; Phase 6-Core)
//   P1_1 = LCD_RST     (ILI9342C reset, active LOW)
//
// **2026-08-15 correction, cross-checked against M5Stack's own
// `M5GFX.cpp` (`github.com/m5stack/M5GFX`, CoreS3 board section)**:
// two things this table originally got wrong, found chasing a real
// "LCD shows nothing" bug on hardware.
//
// 1. LCD_RST/TP_RST were swapped (P1_0/P1_1 reversed from M5GFX's
//    `rst_control`, which uses `1 << 1` for LCD_RST). Harmless in
//    `pmic::init` today — both bits are driven together — but wrong
//    if Phase 6-Core ever needs to toggle TP_RST alone.
// 2. **LCD_BL is not an AW9523 pin at all.** M5GFX's CoreS3
//    `setBrightness` never touches an AW9523 register — the backlight
//    is powered by AXP2101's DLDO1 rail (register `0x90` bit `0x80`
//    to enable, `0x99` for voltage), which `pmic.rs` now writes
//    directly. `AW9523_P0_LCD_BL` is kept below purely as a
//    documented "this was the original, apparently-wrong guess";
//    `pmic::init` still writes it as part of the port-0 safe-default
//    pattern, but nothing depends on it actually controlling the
//    backlight anymore.
pub const AW9523_P0_BUS_OUT_EN: u8 = 1 << 1;
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
