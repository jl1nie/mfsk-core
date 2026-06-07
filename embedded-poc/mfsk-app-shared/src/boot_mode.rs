//! Runtime boot-mode selection.
//!
//! Phase 0.7c: the build is single-binary; the choice between
//! decode-only and WiFi-only DRAM layouts is made at boot from
//! (a) an NVS flag and (b) an optional KEY1-held override. KEY2 long
//! press in the running app flips the NVS flag and `esp_restart()`s.
//!
//! Why this isn't compile-time anymore: with Phase 0.7a's heap-allocated
//! BASIS, the only thing build-time `WIFI_SSID` is still load-bearing
//! for is "does cfg.toml carry credentials" — actual mode choice is
//! now data, not code. Users in the field can flip without a host.


use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

const NVS_NAMESPACE: &str = "mfsk";
const NVS_KEY: &str = "boot_mode";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootMode {
    Decode,
    Wifi,
    /// Phase 1.5-Stick (m5stack-s3-app only, 2026-05-17 pivot): 内蔵
    /// MEMS mic → ES8311 ADC mode → I2S RX → decode_pipeline。M5StickS3
    /// で UAC が hardware-blocked なので、IC-705 SPEAKER OUT を acoustic
    /// に拾う demo path。USB-Serial-JTAG は奪われないので serial console
    /// は生きるが、UDP log があると診断楽なので main.rs dispatch arm 側
    /// で WiFi STA は up させる (失敗しても続行)。Core2 / CoreS3 では
    /// 内蔵 mic 構成が違うので board crate 側でフォールバック (Core2 に
    /// mic なし、CoreS3 は ES7210)。
    Acoustic,
    /// Phase 2-Stick bring-up only (2026-05-17): BLE CI-V scanning +
    /// pairing + GATT command exchange, **decode pipeline OFF**.
    /// BASIS 120 KB / stage1_inc / dual_core を spawn しないので
    /// BT controller + NimBLE host が ample DRAM を持って動ける。
    /// 単独で IC-705 pairing → set_ptt / GPS query が通ることを
    /// 確認するための一時 mode。共存 (Acoustic + BLE) が安定したら
    /// 削除候補。
    CivTest,
    /// Phase 1.6-Stick bring-up only (2026-05-17): FT8 TX synthesis
    /// via ES8311 DAC + I2S TX → built-in speaker → IC-705 microphone
    /// (acoustic VOX). CI-V PTT bracketing via BLE. **decode pipeline
    /// OFF** (BASIS 120 KB は不要 — RX 側は別 mode へ)。test loop が
    /// 30 秒おきに `CQ JL1NIE PM95` を 12.6 s TX。on-air に乗ったか
    /// は別 receiver (WSJT-X etc.) で確認。
    TxTest,
    /// Phase 1.7-Stick (2026-05-17): full QSO operation = Acoustic
    /// RX capture (Phase 1.5) + BLE CI-V (Phase 2) + TX synth (Phase
    /// 1.6) + QSO FSM + menu UX (band select, auto-DF, CQ enable).
    /// Memory is tight (BASIS 120 KB + BLE 30 KB internal); if BASIS
    /// IM_c1 fails to alloc, the boot crashes — fallback is to use
    /// `Acoustic` (no TX) or `TxTest` (no RX) separately. CoreS3
    /// (Phase B-Core) is the long-term canonical platform for QSO.
    Qso,
    /// Phase 1 (#30 onward, m5stack-s3-app only): USB-OTG host で
    /// IC-705 を UAC class device として認識し audio capture。
    /// `usb_host_install()` を呼んだ瞬間に USB-Serial-JTAG が detach
    /// されるため、UDP log 経路を必ず up させる (main.rs dispatch arm
    /// 側で WiFi STA 起動を強制)。Core2 など USB-OTG host を持たない
    /// 板では選んでも no-op (board crate 側でフォールバック)。
    ///
    /// **M5StickS3 警告 (2026-05-17 hardware verification)**:
    /// M5StickS3 は ESP32-S3 silicon としては host 対応だが、board 自体
    /// に VBUS source 回路 / USB-C ID pin 配線 / host 電源 IC が無く、
    /// `usb_host_install()` が hang する。実機検証は M5Stack CoreS3
    /// (AXP2101 + AW9523B) または Espressif ESP32-S3-USB-OTG dev kit
    /// 待ち。M5StickS3 で Uac を選ぶと黒画面で固まるため、KEY1-held
    /// cold boot (`flipped()` で Decode へ戻す) で復帰。詳細は memory
    /// `project_m5stick_s3_no_usb_host`。
    Uac,
}

impl BootMode {
    pub fn as_str(self) -> &'static str {
        match self {
            BootMode::Decode => "decode",
            BootMode::Wifi => "wifi",
            BootMode::Acoustic => "acoustic",
            BootMode::CivTest => "civtest",
            BootMode::TxTest => "txtest",
            BootMode::Qso => "qso",
            BootMode::Uac => "uac",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            BootMode::Decode => "DECODE",
            BootMode::Wifi => "WIFI",
            BootMode::Acoustic => "ACOUSTIC",
            BootMode::CivTest => "CIVTEST",
            BootMode::TxTest => "TXTEST",
            BootMode::Qso => "QSO",
            BootMode::Uac => "UAC",
        }
    }

    /// Parse a cfg.toml `boot_mode` string, defaulting to Decode on unknown values.
    pub fn from_cfg_str(s: &str) -> Self {
        match s {
            "wifi" => BootMode::Wifi,
            "decode" => BootMode::Decode,
            "acoustic" => BootMode::Acoustic,
            "civtest" => BootMode::CivTest,
            "txtest" => BootMode::TxTest,
            "qso" => BootMode::Qso,
            "uac" => BootMode::Uac,
            other => {
                log::warn!("cfg boot_mode unknown value '{other}'; defaulting to decode");
                BootMode::Decode
            }
        }
    }

    /// 4-mode cycle: Decode → Wifi → Acoustic → Uac → Decode → ...
    /// KEY2 long-press from `flip_and_restart` walks the cycle one
    /// step at a time. Boot-time KEY1 override
    /// (`override_held_at_boot`) also calls `flipped()` once, so a
    /// single KEY1-held boot from e.g. Decode lands in Wifi (same as
    /// the old 2-mode behaviour for the Decode↔Wifi pair); cycle
    /// past Wifi by long-pressing KEY2 in the running app.
    pub fn flipped(self) -> Self {
        match self {
            BootMode::Decode => BootMode::Wifi,
            BootMode::Wifi => BootMode::Acoustic,
            BootMode::Acoustic => BootMode::CivTest,
            BootMode::CivTest => BootMode::TxTest,
            BootMode::TxTest => BootMode::Qso,
            BootMode::Qso => BootMode::Uac,
            BootMode::Uac => BootMode::Decode,
        }
    }
}

/// Open the `mfsk` namespace in the default NVS partition. Returned
/// handle is reusable for both reads and writes; held by `main` for
/// the program's lifetime so the flip path doesn't have to re-open.
pub fn open_nvs(part: EspDefaultNvsPartition) -> Result<EspNvs<NvsDefault>, esp_idf_svc::sys::EspError> {
    EspNvs::new(part, NVS_NAMESPACE, true)
}

/// Read the stored mode, defaulting to Decode if the key is absent
/// (first boot) or the stored value is malformed.
pub fn read(nvs: &EspNvs<NvsDefault>) -> BootMode {
    let mut buf = [0u8; 16];
    match nvs.get_str(NVS_KEY, &mut buf) {
        Ok(Some(s)) if s == "wifi" => BootMode::Wifi,
        Ok(Some(s)) if s == "decode" => BootMode::Decode,
        Ok(Some(s)) if s == "acoustic" => BootMode::Acoustic,
        Ok(Some(s)) if s == "civtest" => BootMode::CivTest,
        Ok(Some(s)) if s == "txtest" => BootMode::TxTest,
        Ok(Some(s)) if s == "qso" => BootMode::Qso,
        Ok(Some(s)) if s == "uac" => BootMode::Uac,
        Ok(Some(other)) => {
            log::warn!("NVS boot_mode unrecognised value '{other}'; defaulting to decode");
            BootMode::Decode
        }
        Ok(None) => BootMode::Decode,
        Err(e) => {
            log::warn!("NVS boot_mode read failed ({e}); defaulting to decode");
            BootMode::Decode
        }
    }
}

/// Write the mode to NVS. Caller is responsible for committing /
/// restarting if needed.
pub fn write(nvs: &EspNvs<NvsDefault>, mode: BootMode) -> Result<(), esp_idf_svc::sys::EspError> {
    nvs.set_str(NVS_KEY, mode.as_str())
}

/// Quick poll of the override button (active-low, pull-up enabled) at
/// boot. Returns true if held during the first ~20 ms, which the
/// caller treats as "invert the stored mode for this boot only,
/// without writing NVS". The momentary read is fine because the
/// stored mode is what persists; this is a one-shot override.
///
/// `gpio_pin` is the board's KEY1/BtnA pin (the board crate passes
/// `board::BTN_A_PIN`; on Core2 this will be a different GPIO).
pub fn override_held_at_boot(gpio_pin: i32) -> bool {
    use esp_idf_svc::sys::{
        ESP_OK, GPIO_MODE_DEF_INPUT, gpio_get_level, gpio_pullup_en, gpio_set_direction,
    };
    // Check both gpio_set_direction + gpio_pullup_en return codes —
    // silent failure would mean a floating input + a default-mode
    // boot regardless of what the user pressed. Log + fall through
    // returning `false` so the stored mode wins; better than
    // pretending the button worked. Gemini PR #76 review.
    let dir_err = unsafe { gpio_set_direction(gpio_pin, GPIO_MODE_DEF_INPUT) };
    if dir_err != ESP_OK {
        log::warn!(
            "KEY1 gpio_set_direction(pin={gpio_pin}) failed err={dir_err:#x} — override disabled"
        );
        return false;
    }
    let pull_err = unsafe { gpio_pullup_en(gpio_pin) };
    if pull_err != ESP_OK {
        log::warn!(
            "KEY1 gpio_pullup_en(pin={gpio_pin}) failed err={pull_err:#x} — override disabled"
        );
        return false;
    }
    // Settle pull-up. KEY1 is active-low; level 0 = pressed.
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);
    let pressed = unsafe { gpio_get_level(gpio_pin) } == 0;
    if pressed {
        log::info!("KEY1 held at boot — inverting stored mode for this boot");
    }
    pressed
}

/// Decide the boot mode: stored NVS value, optionally inverted by a
/// KEY1-held override. The board crate passes its own button pin.
pub fn determine(nvs: &EspNvs<NvsDefault>, override_pin: i32) -> BootMode {
    let stored = read(nvs);
    if override_held_at_boot(override_pin) {
        stored.flipped()
    } else {
        stored
    }
}

/// Same as `determine` but without a boot-time GPIO override. Used by
/// boards that have no usable physical override button (M5Stack Core2
/// has touch + AXP192 power button only — no plain GPIO). Mode is
/// purely the stored NVS value; flip path is whatever the board crate
/// wires up at runtime (touch event, serial cmd, etc.).
pub fn determine_no_override(nvs: &EspNvs<NvsDefault>) -> BootMode {
    read(nvs)
}

/// Persist the new mode and reboot. Returns `!`. Logs and skips reboot
/// if the NVS write fails — better to keep the device alive in the
/// previous mode than to brick on a flash-wear failure.
pub fn flip_and_restart(nvs: &EspNvs<NvsDefault>, current: BootMode) -> ! {
    let next = current.flipped();
    log::info!("boot_mode: flip {} → {}; restarting", current.label(), next.label());
    if let Err(e) = write(nvs, next) {
        log::error!("NVS write failed ({e}); aborting flip — staying in {}", current.label());
        // Fall through to a busy loop so the caller still sees `!`.
        loop {
            esp_idf_svc::hal::delay::FreeRtos::delay_ms(1000);
        }
    }
    // Small delay so the log line lands on UDP/CDC before reboot.
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(200);
    unsafe {
        esp_idf_svc::sys::esp_restart();
    }
    #[allow(unreachable_code)]
    loop {}
}
