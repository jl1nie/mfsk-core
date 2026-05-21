//! M5StickS3 公式 pinout (https://docs.m5stack.com/en/core/StickS3).
//!
//! ESP32-S3-PICO-1-N8R8、1.14" 135x240 ST7789P3、KEY1/KEY2、ES8311 internal
//! audio codec + MEMS mic、BMI270 IMU、M5PM1 PMIC。本アプリは LCD/KEY1/KEY2
//! と USB-OTG (host) を主に使う。内蔵マイクと codec はバイパス (FT8 音声は
//! IC-705 USB UAC から来るため)。

#![allow(dead_code)]

// ── LCD: ST7789P3 (ST7789 互換 init で動く想定) ──────────────────────
// SPI2_HOST + display-interface-spi で駆動。
pub const LCD_SPI_HOST: u8 = 2;
pub const LCD_PIN_SCK: i32 = 40;
pub const LCD_PIN_MOSI: i32 = 39;
pub const LCD_PIN_CS: i32 = 41;
pub const LCD_PIN_DC: i32 = 45;
pub const LCD_PIN_RST: i32 = 21;
pub const LCD_PIN_BL: i32 = 38;
pub const LCD_WIDTH: u16 = 135;
pub const LCD_HEIGHT: u16 = 240;
// 推定回転 (実機で要確認): M5StickS3 は縦長使用が標準。
// ST7789P3 は 0/90/180/270° で MADCTL を切替。
pub const LCD_ROTATION_DEFAULT_DEG: u16 = 0;

// ── Buttons (BtnA / BtnB に対応する GPIO) ──────────────────────────────
// 入力、active-low (内部 pull-up 必要)。
//
// M5Stack 公式ピン表は KEY1=GPIO11 / KEY2=GPIO12 と記載しているが、
// 実機検証 (2026-05-21) で物理ボタンと firmware ハンドラの紐付けが
// 逆だったため swap した — 物理 BtnA (天面右、丸ボタン) = GPIO12、
// 物理 BtnB (側面、長押しメニュー想定) = GPIO11。
// 公式表記との不一致は本番アプリ動作 (BtnA 長押しでデモ trigger /
// BtnB 長押しでメニュー) を一次資料として優先した結果。
pub const BTN_A_PIN: i32 = 12;
pub const BTN_B_PIN: i32 = 11;

// ── I2C bus 0 (PMIC + IMU 共有) ──────────────────────────────────────
pub const I2C0_SCL: i32 = 48;
pub const I2C0_SDA: i32 = 47;
pub const PMIC_I2C_ADDR: u8 = 0x6E; // M5PM1
pub const IMU_I2C_ADDR: u8 = 0x68; // BMI270

// ── ES8311 audio codec (使わない: 本アプリは USB UAC 経由) ────────────
// 参考までに残す。Phase 1 fallback で内蔵 mic を使う場合に再活性化。
pub const ES8311_MCLK: i32 = 18;
pub const ES8311_DOUT: i32 = 14; // codec → S3 I2S DIN
pub const ES8311_BCLK: i32 = 17;
pub const ES8311_LRCK: i32 = 15;
pub const ES8311_DIN: i32 = 16; // S3 I2S DOUT → codec

// ── USB-OTG (S3 内蔵ペリフェラル、ピン固定) ──────────────────────────
pub const USB_DP: i32 = 20;
pub const USB_DM: i32 = 19;

// ── Grove (外部接続用) ───────────────────────────────────────────────
pub const GROVE_DAT_A: i32 = 9;
pub const GROVE_DAT_B: i32 = 10;
