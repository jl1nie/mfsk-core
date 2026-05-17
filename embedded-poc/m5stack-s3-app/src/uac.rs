//! USB Audio Class host capture — Phase 1 skeleton.
//!
//! IC-705 を ESP32-S3 USB-OTG host で UAC class device として認識し、
//! isochronous IN endpoint から音声を吸い上げて decode pipeline に流す
//! 経路。本ファイルは Phase 1 の **component 取り込み + bindings
//! sanity check** のみ。実装本体は #30 (host install + hot-plug) と
//! それ以降の todo で順に積む。
//!
//! ## 接続方式 (確定済)
//!
//! - Component: `espressif/usb_host_uac@^1.4` を `Cargo.toml` の
//!   `extra_components` 経由で managed component として取得。
//!   bindings は `esp_idf_sys::uac::*` に生成 (Cargo.toml の
//!   `bindings_module = "uac"`)。
//! - IC-705 USB Audio は **固定 48 kHz / stereo / 16-bit** (16 kHz は
//!   selectable ではない)。S3 側で stereo → mono (L ch 抽出 / R ch 破棄)
//!   + 48 kHz → 12 kHz の 4:1 decimation を `embedded_shared` 経由で行う。
//! - Reference example は esp-usb 上流の
//!   `host/class/uac/usb_host_uac/examples/audio_player/main/main.c`。
//!   そこの init 順序 (`usb_host_install` → `uac_host_install` →
//!   `RX_CONNECTED` 通知で `uac_host_device_open` →
//!   `uac_host_device_start`) をそのまま Rust に移植する想定。
//!
//! ## OTG 排他
//!
//! `usb_host_install()` を呼んだ瞬間に USB-Serial-JTAG endpoint が
//! detach されるため、`BootMode::Uac` では WiFi STA + UDP log を必ず
//! 起動させる (`main.rs` dispatch arm で強制)。
//!
//! ## 進捗
//!
//! - [x] managed component + bindings (#29, このファイル)
//! - [ ] host install + hot-plug callback (#30)
//! - [ ] iso IN streaming (#31)
//! - [ ] 48 kHz stereo → 12 kHz mono resampler + ringbuf → decode (#32)
//! - [ ] verification on hardware (#33-#34)
//! - [ ] disconnect/reconnect polish (#35)

/// Component / binding sanity check. Compile-only assertion that
/// `esp_idf_svc::sys::uac::*` resolves — if `cargo check --release`
/// succeeds with this referenced, we know the managed component is on
/// the include path and bindgen generated the expected symbols. The
/// `sys` re-export comes from `esp-idf-svc` (we don't pull `esp-idf-sys`
/// as a direct dep — it's transitive via svc/hal).
///
/// Not called from `main.rs` yet; pulled into the build only via the
/// `_BINDINGS_PRESENT` constant so the linker can DCE it once real
/// entry points land in #30+.
#[allow(dead_code)]
const _BINDINGS_PRESENT: usize =
    core::mem::size_of::<esp_idf_svc::sys::uac::uac_host_driver_config_t>();
