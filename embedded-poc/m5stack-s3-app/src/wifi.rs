//! WiFi STA bring-up for Phase 0.6 UDP log streaming.
//!
//! Thin port of `wifikey2/wifikey/src/wifi.rs` (same author, esp-idf-svc
//! 0.51 → 0.52 API is unchanged for the calls we use). AP fallback,
//! NVS persistence, and reconnect were dropped — Phase 0.6 only needs
//! "boot, scan, connect to one known SSID, expose IP."
//!
//! Caller hands over `peripherals.modem` and a fresh `EspSystemEventLoop`;
//! we return a `WifiHandle` whose ownership keeps the driver alive (the
//! socket bound on top of it must live as long as the WiFi stack).

use anyhow::{anyhow, Result};
use esp_idf_hal::modem::WifiModemPeripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};

/// Live WiFi STA handle. Drop ends the WiFi association (and any sockets
/// bound on top stop receiving), so callers must keep this alive.
pub struct WifiHandle {
    _wifi: BlockingWifi<EspWifi<'static>>,
    pub ip: std::net::Ipv4Addr,
}

/// Connect to one specific SSID. Blocks until DHCP returns an IP or
/// the underlying esp-idf stack errors out.
pub fn connect_sta<M>(
    modem: M,
    sysloop: EspSystemEventLoop,
    ssid: &str,
    psk: &str,
) -> Result<WifiHandle>
where
    M: WifiModemPeripheral + 'static,
{
    if ssid.is_empty() {
        return Err(anyhow!("WIFI_SSID empty — cfg.toml missing or [wifi].ssid blank"));
    }

    let esp_wifi = EspWifi::new(modem, sysloop.clone(), None)?;
    let mut wifi = BlockingWifi::wrap(esp_wifi, sysloop)?;

    // First-pass scan to learn the channel (skips ~250 ms of brute
    // channel-walking on `connect()`). Mirrors wifikey2's pattern.
    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;
    wifi.start()?;
    let ap_infos = wifi.scan()?;
    let channel = ap_infos
        .iter()
        .find(|ap| ap.ssid.as_str() == ssid)
        .map(|ap| ap.channel);
    if channel.is_none() {
        log::warn!("WiFi: SSID '{ssid}' not seen in scan; trying anyway");
    }

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: ssid.try_into().map_err(|_| anyhow!("SSID too long for esp-idf"))?,
        password: psk.try_into().map_err(|_| anyhow!("PSK too long for esp-idf"))?,
        channel,
        ..Default::default()
    }))?;

    wifi.connect()?;
    wifi.wait_netif_up()?;

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
    let ip = ip_info.ip;
    log::info!("WiFi STA up: {} ch={:?}", ip, channel);

    Ok(WifiHandle { _wifi: wifi, ip })
}
