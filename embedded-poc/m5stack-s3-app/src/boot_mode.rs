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
}

impl BootMode {
    pub fn as_str(self) -> &'static str {
        match self {
            BootMode::Decode => "decode",
            BootMode::Wifi => "wifi",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            BootMode::Decode => "DECODE",
            BootMode::Wifi => "WIFI",
        }
    }

    pub fn flipped(self) -> Self {
        match self {
            BootMode::Decode => BootMode::Wifi,
            BootMode::Wifi => BootMode::Decode,
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

/// Quick poll of KEY1 (active-low, pull-up enabled) at boot. Returns
/// true if the user is holding KEY1 down during the first ~500 ms,
/// which the caller treats as "invert the stored mode for this boot
/// only, without writing NVS". The momentary read is fine because the
/// stored mode is what persists; this is a one-shot override.
pub fn key1_held_at_boot() -> bool {
    use esp_idf_svc::sys::{gpio_get_level, gpio_pullup_en, gpio_set_direction, GPIO_MODE_DEF_INPUT};
    let pin = crate::board::BTN_A_PIN;
    unsafe {
        gpio_set_direction(pin, GPIO_MODE_DEF_INPUT);
        gpio_pullup_en(pin);
    }
    // Settle pull-up. KEY1 is active-low; level 0 = pressed.
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(20);
    let pressed = unsafe { gpio_get_level(pin) } == 0;
    if pressed {
        log::info!("KEY1 held at boot — inverting stored mode for this boot");
    }
    pressed
}

/// Decide the boot mode: stored NVS value, optionally inverted by
/// KEY1-held override.
pub fn determine(nvs: &EspNvs<NvsDefault>) -> BootMode {
    let stored = read(nvs);
    if key1_held_at_boot() {
        stored.flipped()
    } else {
        stored
    }
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
