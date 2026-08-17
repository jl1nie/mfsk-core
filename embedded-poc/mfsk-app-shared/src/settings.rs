//! Runtime-editable WSPR app settings, persisted in NVS.
//!
//! Follows [`crate::boot_mode`]'s shape exactly — same `"mfsk"`
//! namespace (new keys alongside `boot_mode`'s own, no new
//! namespace), same "one `EspNvs` handle, `get_*`/`set_*` per field,
//! default gracefully on absent/malformed" pattern. `boot_mode` is
//! the only precedent for persisted runtime settings this crate had
//! before this file; nothing here invents a new idiom.
//!
//! Written via the web form in [`crate::http_config`], read once per
//! WSPR slot by the app's scan loop — see that module's own doc
//! comment for why polling NVS each slot (rather than requiring a
//! reboot after saving) was the choice made here, and why it's a
//! genuine tradeoff rather than an obviously-correct default.

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

const NVS_NAMESPACE: &str = "mfsk";

const K_CALL: &str = "wspr_call";
const K_GRID: &str = "wspr_grid";
const K_TXDBM: &str = "wspr_txdbm";
const K_BAND: &str = "wspr_band";
const K_SPOT: &str = "wspr_spot";
const K_NTP_EN: &str = "wspr_ntpen";
const K_NTP_SRV: &str = "wspr_ntpsrv";

/// Default NTP server — a well-known public pool, not this project's
/// own infrastructure.
const DEFAULT_NTP_SERVER: &str = "pool.ntp.org";

/// All WSPR app settings the web form (`http_config`) exposes.
///
/// **Two defaults point in opposite directions on purpose**:
/// `ntp_enabled` defaults `true` — without NTP there is no way to
/// hit WSPR's absolute 2-minute UTC slot grid from a cold start at
/// all, so it is closer to "required" than "optional". `wsprnet`
/// reporting defaults *off* (via [`wsprnet_spot_config`]'s default
/// `"off"`, matching [`crate::wsprnet::SpotSink::Disabled`]'s own
/// default) — spotting is "a public claim that a named station was
/// heard", per `wsprnet.rs`'s own doc comment, so opting in is a
/// deliberate call-site decision, not a default.
///
/// [`wsprnet_spot_config`]: Settings::wsprnet_spot_config
pub struct Settings {
    /// Own callsign. Empty means "not configured" — the receiver
    /// still decodes and displays spots, it just can't identify
    /// itself to wsprnet (reporting silently stays off in that case,
    /// same as `SpotSink::Disabled`).
    pub call: heapless::String<16>,
    /// Own 4- or 6-character Maidenhead grid locator.
    pub grid: heapless::String<8>,
    /// Stored only — this device has no RF-TX capability at all
    /// today (`mfsk_core::wspr::tx` is an audio-waveform synthesizer,
    /// not a radio driver). Forward-looking storage for whenever real
    /// TX exists; not read by anything yet.
    pub tx_power_dbm: i8,
    /// Index into [`crate::wspr_bands::WSPR_BANDS`].
    pub band_idx: u8,
    /// Raw string fed straight to
    /// [`crate::wsprnet::SpotSink::from_config`] — `"off"`/`"dummy"`/
    /// a POST URL. Kept as one field rather than a separate
    /// enabled-bool + URL pair: `SpotSink::from_config` already
    /// parses exactly this shape, and splitting it risks the two
    /// halves going out of sync (bool=true, URL empty = what?).
    pub wsprnet_spot_config: heapless::String<80>,
    pub ntp_enabled: bool,
    pub ntp_server: heapless::String<48>,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            call: heapless::String::new(),
            grid: heapless::String::new(),
            tx_power_dbm: 0,
            band_idx: crate::wspr_bands::DEFAULT_BAND_IDX,
            wsprnet_spot_config: heapless::String::try_from("off").unwrap_or_default(),
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

    let mut call_buf = [0u8; 24];
    let call = match nvs.get_str(K_CALL, &mut call_buf) {
        Ok(Some(s)) => heapless::String::try_from(s).unwrap_or_default(),
        _ => defaults.call,
    };

    let mut grid_buf = [0u8; 12];
    let grid = match nvs.get_str(K_GRID, &mut grid_buf) {
        Ok(Some(s)) => heapless::String::try_from(s).unwrap_or_default(),
        _ => defaults.grid,
    };

    let tx_power_dbm = nvs
        .get_i8(K_TXDBM)
        .ok()
        .flatten()
        .unwrap_or(defaults.tx_power_dbm);

    let band_idx = match nvs.get_u8(K_BAND) {
        Ok(Some(b)) if (b as usize) < crate::wspr_bands::WSPR_BANDS.len() => b,
        _ => defaults.band_idx,
    };

    let mut spot_buf = [0u8; 96];
    let wsprnet_spot_config = match nvs.get_str(K_SPOT, &mut spot_buf) {
        Ok(Some(s)) => heapless::String::try_from(s).unwrap_or_default(),
        _ => defaults.wsprnet_spot_config,
    };

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
        call,
        grid,
        tx_power_dbm,
        band_idx,
        wsprnet_spot_config,
        ntp_enabled,
        ntp_server,
    }
}

/// Write all settings. Caller ([`crate::http_config`]'s `/save`
/// handler) is expected to have already validated every field —
/// this function does not re-validate, it only persists.
pub fn save(nvs: &EspNvs<NvsDefault>, s: &Settings) -> Result<(), esp_idf_svc::sys::EspError> {
    nvs.set_str(K_CALL, &s.call)?;
    nvs.set_str(K_GRID, &s.grid)?;
    nvs.set_i8(K_TXDBM, s.tx_power_dbm)?;
    nvs.set_u8(K_BAND, s.band_idx)?;
    nvs.set_str(K_SPOT, &s.wsprnet_spot_config)?;
    nvs.set_u8(K_NTP_EN, s.ntp_enabled as u8)?;
    nvs.set_str(K_NTP_SRV, &s.ntp_server)?;
    Ok(())
}
