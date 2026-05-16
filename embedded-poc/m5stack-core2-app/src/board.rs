//! M5Stack Core2 公式 pinout (https://docs.m5stack.com/en/core/core2).
//!
//! ESP32-D0WD-V3 (LX6 dual-core)、ILI9342C 320×240 IPS LCD、3 touch zones
//! via FT6336U (Phase 2 では未使用)、NS4168 speaker amp、SPM1423 PDM mic、
//! AXP192 PMIC、MPU6886 IMU。Phase 2 で使うのは AXP192 と LCD のみ —
//! audio source は `embedded_shared::wav_sim` の baked WAV ループ、
//! 入力は無し (タッチは Phase 2.5+)。
//!
//! References:
//! - M5Unified `M5Core2.cpp` (canonical pinout)
//! - AXP192 datasheet
//! - ILI9342C datasheet (mostly ILI9341 compatible — mipidsi の ILI9341
//!   model で初期化シーケンスは通る)

#![allow(dead_code)]

// ── LCD: ILI9342C, 320×240 landscape ─────────────────────────────────
// VSPI (SPI3_HOST) を使う。LCD reset は AXP192 GPIO4 経由なので ESP32
// 側からは直接アクセスしない (mipidsi Builder には dummy 不要 — RST 線
// を渡さない構成にする)。BL も AXP192 LDO2 経由。
pub const LCD_SPI_HOST: u8 = 2; // SPI3_HOST / VSPI
pub const LCD_PIN_SCK: i32 = 18;
pub const LCD_PIN_MOSI: i32 = 23;
pub const LCD_PIN_MISO: i32 = 38; // 表示には使わないが SPI driver には渡せる
pub const LCD_PIN_CS: i32 = 5;
pub const LCD_PIN_DC: i32 = 15;
// LCD_RST / LCD_BL は AXP192 経由 — board::AXP192_LCD_RST_PIN / LDO2 で参照。
pub const LCD_WIDTH: u16 = 320;
pub const LCD_HEIGHT: u16 = 240;

// ── I2C bus 0 (AXP192 + IMU + Touch 共有) ────────────────────────────
pub const I2C0_SCL: i32 = 22;
pub const I2C0_SDA: i32 = 21;
pub const AXP192_I2C_ADDR: u8 = 0x34;
pub const IMU_I2C_ADDR: u8 = 0x68; // MPU6886
pub const TOUCH_I2C_ADDR: u8 = 0x38; // FT6336U (Phase 2 未使用)

// AXP192 内部 GPIO 割当 (Core2 board wiring):
//   GPIO0 → speaker amp EN (NS4168)
//   GPIO1 → (unused on stock board)
//   GPIO2 → green LED / vibration / case heat-set (variant-dependent)
//   GPIO4 → LCD RST
pub const AXP192_GPIO_LCD_RST: u8 = 4;

// ── Speaker / Microphone (Phase 2 未使用) ───────────────────────────
pub const I2S_BCLK: i32 = 12;
pub const I2S_LRCK: i32 = 0;
pub const I2S_DOUT: i32 = 2;
pub const PDM_CLK: i32 = 0;
pub const PDM_DAT: i32 = 34;

// ── Power button + Touch IRQ ─────────────────────────────────────────
// 物理ボタンは AXP192 PEK のみ (= 電源ボタン)。ESP32 側からは
// AXP192 reg 0x46 / IRQ ライン (GPIO35 input) で読む。Phase 2 では未使用。
pub const AXP192_IRQ_PIN: i32 = 35;
pub const TOUCH_IRQ_PIN: i32 = 39;

// ── Grove (Port A) ───────────────────────────────────────────────────
pub const GROVE_A_SDA: i32 = 32;
pub const GROVE_A_SCL: i32 = 33;
