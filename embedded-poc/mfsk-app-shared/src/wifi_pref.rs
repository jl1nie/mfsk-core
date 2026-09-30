//! Whether this boot brings WiFi up — the operator's choice, persisted
//! beside `boot_mode` and `grid_src`.
//!
//! ## Why this is its own setting and not a consequence of another
//!
//! Until this existed, `main.rs` decided from the boot mode alone
//! (`matches!(mode, Wifi | Uac)`), so a board set to
//! [`crate::grid_src::GridSource::AirDt`] — which means "no
//! infrastructure here, take the phase off the band" — still ran an
//! association campaign at boot (issue #381).
//!
//! The obvious fix, tying WiFi to the grid source, was rejected at first: the
//! two answer different questions. The grid source says where *time*
//! comes from; this says whether there is a *console*. A hilltop with a
//! phone hotspot wants `AIR DT` **and** a log; a home bench with a
//! working clock might still want the radio silent. One setting cannot
//! express both, and the one it would express is the less useful half.
//!
//! **Since 2026-09-30 the tie is made anyway, on top of this setting**
//! ([`crate::wifi_policy`]): `TIME: AIR DT` turns WiFi off whatever this
//! says, because on the first slots an air-placed grid needs the
//! association costs the decoder ~40 % (below), and the maintainer would
//! rather have those slots than the log. This setting stays as it was:
//! it is the choice under `TIME: NTP`, and its stored value is untouched
//! by the grid source, so switching back finds it as left. The price is
//! the one named above — in `BootMode::Uac` there is then **no console**
//! under `AIR DT`, only the panel — and the panel shows `WIFI: OFF`
//! there so it does not claim a radio that is not running.
//!
//! ## What the association actually costs
//!
//! Two different costs, and only the first is what #381 was about:
//!
//! - **A campaign that fails.** `wifi::connect_sta` gives up after
//!   `CONNECT_MAX_ATTEMPTS` (4) at `CONNECT_RETRY_DELAY_MS` (3 s)
//!   apart, and while the driver is hunting for an AP that is not
//!   there it runs at FreeRTOS priority 23 — above anything this app
//!   creates. Measured 2026-08-22: `fst4_sync_search` 711 → 1 395 ms
//!   per candidate, ~40 % of the decoder's throughput, and it lands on
//!   exactly the first slots an `AIR DT` boot needs in order to place
//!   its grid.
//! - **An association that succeeds and then idles.** `fst4_app`'s
//!   candidate loop went 33 → 53 s until it set `WIFI_PS_MIN_MODEM`
//!   (see `net.rs`'s module docs). The FT8/UAC path does not go through
//!   `net.rs` and so runs at the IDF default `WIFI_PS_NONE`; whether
//!   FT8 pays the same price is **unmeasured**. The 160-slot live run
//!   of 2026-09-21 decoded 6-8 per slot with WiFi associated and
//!   logging, so it is survivable, which is not the same as free.
//!
//! ## Default
//!
//! [`WifiPref::On`] — what every build did before this existed. A
//! setting that changes behaviour the first time it is *read* rather
//! than the first time it is *set* is how a receiver quietly stops
//! doing something nobody asked it to stop doing.

use esp_idf_svc::nvs::{EspNvs, NvsDefault};

/// Same namespace as `boot_mode` and `grid_src`, so one NVS handle
/// serves all three.
const NVS_KEY: &str = "wifi_pref";

#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum WifiPref {
    /// Associate at boot, as every build did before this setting.
    #[default]
    On,
    /// Leave the radio down. In `BootMode::Uac` this also means **no
    /// console at all** — the USB host driver has taken
    /// USB-Serial-JTAG, so the UDP log and the HTTP config page are the
    /// only way out and both go with WiFi. The panel is what is left.
    Off,
}

impl WifiPref {
    pub fn as_str(self) -> &'static str {
        match self {
            WifiPref::On => "on",
            WifiPref::Off => "off",
        }
    }

    /// What the picker shows.
    pub fn label(self) -> &'static str {
        match self {
            WifiPref::On => "WIFI: ON",
            WifiPref::Off => "WIFI: OFF",
        }
    }

    pub fn enabled(self) -> bool {
        self == WifiPref::On
    }

    pub fn from_str(s: &str) -> Self {
        match s {
            "off" => WifiPref::Off,
            _ => WifiPref::On,
        }
    }
}

/// The stored choice, or [`WifiPref::On`] when nothing is stored.
pub fn read(nvs: &EspNvs<NvsDefault>) -> WifiPref {
    let mut buf = [0u8; 8];
    match nvs.get_str(NVS_KEY, &mut buf) {
        Ok(Some(s)) => WifiPref::from_str(s),
        _ => WifiPref::default(),
    }
}

pub fn write(nvs: &EspNvs<NvsDefault>, pref: WifiPref) -> Result<(), esp_idf_svc::sys::EspError> {
    nvs.set_str(NVS_KEY, pref.as_str())
}
