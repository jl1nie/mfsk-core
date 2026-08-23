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
    /// CoreS3 standalone WSPR receiver (`src/bin/wspr_app.rs`, Phase E
    /// / #260): DDC + Fano decode over 120 s slots, spot list on the
    /// LCD, settings over HTTP. Shares `uac.rs` with the FT8 line for
    /// live audio.
    ///
    /// Deliberately **not** in [`BootMode::flipped`]'s button cycle —
    /// CoreS3 has no buttons, and putting a CoreS3-only mode in
    /// M5StickS3's KEY1/KEY2 walk would offer a mode that board cannot
    /// run. Selected from `cfg.toml`, NVS, or the HTTP config page.
    Wspr,
    /// CoreS3 standalone FST4 receiver (`src/bin/fst4_app.rs`, #306 /
    /// #307). Same reasoning as [`BootMode::Wspr`] for the cycle.
    Fst4,
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
            BootMode::Wspr => "wspr",
            BootMode::Fst4 => "fst4",
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
            BootMode::Wspr => "WSPR",
            BootMode::Fst4 => "FST4",
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
            "wspr" => BootMode::Wspr,
            "fst4" => BootMode::Fst4,
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
            // Not part of the cycle — see their doc comments. A board
            // that somehow lands here walks back to a mode it can run.
            BootMode::Wspr | BootMode::Fst4 => BootMode::Decode,
        }
    }
}

/// Open the `mfsk` namespace in the default NVS partition. Returned
/// handle is reusable for both reads and writes; held by `main` for
/// the program's lifetime so the flip path doesn't have to re-open.
pub fn open_nvs(
    part: EspDefaultNvsPartition,
) -> Result<EspNvs<NvsDefault>, esp_idf_svc::sys::EspError> {
    EspNvs::new(part, NVS_NAMESPACE, true)
}

/// Read the stored mode, defaulting to Decode if the key is absent
/// (first boot) or the stored value is malformed.
/// Whether a mode has ever been stored.
///
/// [`read`] cannot answer this: it returns `Decode` both for "the key
/// says decode" and for "there is no key". The difference matters
/// because `cfg.toml`'s `boot_mode` is a *seed* — something to apply
/// on a fresh board — and not an override to reapply on every boot.
/// Reapplying it means a mode chosen at runtime, from the touch panel
/// or the HTTP page, survives exactly until the next restart, which is
/// the opposite of what a persisted setting is for.
pub fn is_set(nvs: &EspNvs<NvsDefault>) -> bool {
    let mut buf = [0u8; 16];
    matches!(nvs.get_str(NVS_KEY, &mut buf), Ok(Some(_)))
}

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
        Ok(Some(s)) if s == "wspr" => BootMode::Wspr,
        Ok(Some(s)) if s == "fst4" => BootMode::Fst4,
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
        gpio_get_level, gpio_pullup_en, gpio_set_direction, ESP_OK, GPIO_MODE_DEF_INPUT,
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
    log::info!(
        "boot_mode: flip {} → {}; restarting",
        current.label(),
        next.label()
    );
    if let Err(e) = write(nvs, next) {
        log::error!(
            "NVS write failed ({e}); aborting flip — staying in {}",
            current.label()
        );
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

/// Persist `mode` and restart, from a task whose stack is safe to hold
/// a flash write.
///
/// **Why this is not just `write` + `esp_restart`.** NVS writes to SPI
/// flash, and a flash write disables the flash cache — which also
/// unmaps PSRAM. ESP-IDF therefore asserts that the calling task's
/// stack is not in PSRAM (`esp_task_stack_is_sane_cache_disabled`,
/// `cache_utils.c`). The WSPR and FST4 receivers draw from a display
/// task whose stack is deliberately `MALLOC_CAP_SPIRAM`, to leave
/// internal DRAM for the decoder — so calling `write` from there does
/// not fail, it aborts:
///
/// ```text
/// assert failed: spi_flash_disable_interrupts_caches_and_other_cpu
///     cache_utils.c:127 (esp_task_stack_is_sane_cache_disabled())
/// ```
///
/// The abort reboots the board, which is indistinguishable from the
/// restart the caller intended — except NVS never changed, so the same
/// mode comes back. From the panel it reads as a button that does
/// nothing, and that is how it read from the bench for several
/// sessions: the touch, the hit test, the arming and the commit were
/// all working, and the log line announcing the switch was printed
/// immediately before the abort.
///
/// A short-lived task created with `xTaskCreatePinnedToCore` (rather
/// than the `WithCaps` variant) gets an internal-DRAM stack, which is
/// the only requirement. It costs that stack for a few hundred
/// milliseconds and then the board is gone.
pub fn commit_and_restart(nvs: std::sync::Arc<std::sync::Mutex<EspNvs<NvsDefault>>>, mode: BootMode) {
    struct Req {
        nvs: std::sync::Arc<std::sync::Mutex<EspNvs<NvsDefault>>>,
        mode: BootMode,
    }

    extern "C" fn entry(arg: *mut core::ffi::c_void) {
        // SAFETY: `commit_and_restart` leaked exactly this pointer.
        let req = unsafe { Box::from_raw(arg as *mut Req) };
        match req.nvs.lock() {
            Ok(nvs) => match write(&nvs, req.mode) {
                Ok(()) => log::warn!("boot_mode: committed {} — restarting", req.mode.label()),
                Err(e) => log::error!("boot_mode write failed: {e} — not restarting"),
            },
            Err(e) => log::error!("boot_mode: NVS lock poisoned: {e} — not restarting"),
        }
        drop(req);
        // Let the line reach the log sink; in UAC host mode that is the
        // only channel there is.
        std::thread::sleep(std::time::Duration::from_millis(400));
        // SAFETY: no arguments, does not return.
        unsafe { esp_idf_svc::sys::esp_restart() };
    }

    let ptr = Box::into_raw(Box::new(Req { nvs, mode })) as *mut core::ffi::c_void;
    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(entry),
            c"boot_commit".as_ptr(),
            6144,
            ptr,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("boot_mode: could not spawn the commit task — mode not saved");
        // SAFETY: the task was never created, so nothing else owns it.
        drop(unsafe { Box::from_raw(ptr as *mut Req) });
    }
}
