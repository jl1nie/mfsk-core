//! Runtime-editable settings, persisted in NVS: NTP on/off and the server.
//!
//! Follows [`crate::boot_mode`]'s shape exactly — same `"mfsk"`
//! namespace (new keys alongside `boot_mode`'s own, no new
//! namespace), same "one `EspNvs` handle, `get_*`/`set_*` per field,
//! default gracefully on absent/malformed" pattern.
//!
//! Written via the web form in [`crate::http_config`], read by `net::bring_up`
//! once WiFi is up.
//!
//! **This was the WSPR app's settings**: callsign, grid, TX power, band and the
//! wsprnet endpoint sat beside the NTP fields until the CoreS3 WSPR receiver was
//! removed (2026-09-30), and nothing else read them. They are gone from the
//! struct and the form; what is left is what every receiver uses. The NVS keys
//! keep their `wspr_` prefix, on purpose: renaming them would lose the NTP
//! server an operator already set. Keys the old form wrote (`wspr_call`,
//! `wspr_grid`, `wspr_txdbm`, `wspr_band`, `wspr_spot`) stay in flash, unread.

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

const NVS_NAMESPACE: &str = "mfsk";

// The `wspr_` prefix is history, kept so a stored NTP server survives.
const K_NTP_EN: &str = "wspr_ntpen";
const K_NTP_SRV: &str = "wspr_ntpsrv";

/// Default NTP server — a well-known public pool, not this project's
/// own infrastructure.
const DEFAULT_NTP_SERVER: &str = "pool.ntp.org";

/// The settings the web form (`http_config`) exposes.
///
/// `ntp_enabled` defaults `true`: without NTP there is no way to hit an
/// absolute UTC slot grid from a cold start except from the air
/// (`TIME: AIR DT`), so it is closer to "required" than "optional".
pub struct Settings {
    pub ntp_enabled: bool,
    pub ntp_server: heapless::String<48>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            ntp_enabled: true,
            ntp_server: heapless::String::try_from(DEFAULT_NTP_SERVER).unwrap_or_default(),
        }
    }
}

/// Open the `mfsk` namespace in the default NVS partition — the same
/// namespace [`crate::boot_mode::open_nvs`] opens, just a different
/// handle (ESP-IDF's NVS supports independent concurrent opens of the
/// same namespace; this crate has not needed to verify that assumption
/// under contention, since the settings handle and the boot-mode
/// handle are typically opened by different binaries).
pub fn open_nvs(
    part: EspDefaultNvsPartition,
) -> Result<EspNvs<NvsDefault>, esp_idf_svc::sys::EspError> {
    EspNvs::new(part, NVS_NAMESPACE, true)
}

/// Read all settings, defaulting each field independently on a
/// missing key or a read/parse failure — one corrupt field should
/// not take the rest of the settings down with it.
pub fn load(nvs: &EspNvs<NvsDefault>) -> Settings {
    let defaults = Settings::default();

    let ntp_enabled = match nvs.get_u8(K_NTP_EN) {
        Ok(Some(v)) => v != 0,
        _ => defaults.ntp_enabled,
    };

    let mut ntp_srv_buf = [0u8; 56];
    let ntp_server = match nvs.get_str(K_NTP_SRV, &mut ntp_srv_buf) {
        Ok(Some(s)) if !s.is_empty() => heapless::String::try_from(s).unwrap_or_default(),
        _ => defaults.ntp_server,
    };

    Settings {
        ntp_enabled,
        ntp_server,
    }
}

/// Write all settings. Caller ([`crate::http_config`]'s `/save`
/// handler) is expected to have already validated every field —
/// this function does not re-validate, it only persists.
pub fn save(nvs: &EspNvs<NvsDefault>, s: &Settings) -> Result<(), esp_idf_svc::sys::EspError> {
    nvs.set_u8(K_NTP_EN, s.ntp_enabled as u8)?;
    nvs.set_str(K_NTP_SRV, &s.ntp_server)?;
    Ok(())
}
