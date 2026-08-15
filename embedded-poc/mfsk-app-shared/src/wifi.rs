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
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::modem::WifiModemPeripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};

/// Association attempts before giving up. **2026-08-15, real-hardware
/// finding**: across this session's own flash-and-capture logs, a bare
/// single `connect()` (no retry — the behaviour this constant fixes)
/// succeeded only ~5 of 10 independent boots against the same AP,
/// every failure showing the identical pattern in the serial log:
/// `wifi:Association refused temporarily time 1000, comeback time
/// 1100 (TUs)` followed immediately by `state: assoc -> init` — the
/// esp-idf driver gives up rather than actually waiting out the AP's
/// requested "comeback time" and retrying itself, so a caller that
/// only calls `connect()` once inherits that ~50% coin-flip. A short
/// bounded retry (this constant × [`CONNECT_RETRY_DELAY_MS`]) is the
/// standard workaround for this exact 802.11 status-code-30 pattern.
const CONNECT_MAX_ATTEMPTS: u32 = 4;
/// Delay between attempts — comfortably past the ~1.1 s "comeback
/// time" the failing log line itself reports, so a retry isn't just
/// re-triggering the same refusal.
const CONNECT_RETRY_DELAY_MS: u32 = 2_000;

/// Live WiFi STA handle. Drop ends the WiFi association (and any sockets
/// bound on top stop receiving), so callers must keep this alive.
pub struct WifiHandle {
    _wifi: BlockingWifi<EspWifi<'static>>,
    pub ip: std::net::Ipv4Addr,
    /// Directed subnet broadcast (`ip | ~mask`). Preferred over
    /// limited broadcast `255.255.255.255` because some APs and
    /// routers drop the limited form for power-save / multicast
    /// suppression but pass directed-broadcast through.
    pub subnet_broadcast: std::net::Ipv4Addr,
}

/// Connect to one specific SSID. Blocks until DHCP returns an IP or
/// the underlying esp-idf stack errors out.
pub fn connect_sta<M>(
    modem: M,
    sysloop: EspSystemEventLoop,
    nvs: Option<EspDefaultNvsPartition>,
    ssid: &str,
    psk: &str,
) -> Result<WifiHandle>
where
    M: WifiModemPeripheral + 'static,
{
    if ssid.is_empty() {
        return Err(anyhow!("WIFI_SSID empty — cfg.toml missing or [wifi].ssid blank"));
    }

    // NVS が `Some` だと PHY 校正データを `phy_init/cal` キーにキャッシュ
    // できる (boot 高速化 + DRAM 節約)。`None` だと毎回フルキャリブで
    // "NVS has not been initialized" エラーが出る。
    let esp_wifi = EspWifi::new(modem, sysloop.clone(), nvs)?;
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

    // See CONNECT_MAX_ATTEMPTS's own doc comment for why this isn't a
    // single `connect()?` — real-hardware association-refusal pattern
    // found 2026-08-15, ~50% first-attempt failure rate against the
    // test AP.
    let mut last_err = None;
    for attempt in 1..=CONNECT_MAX_ATTEMPTS {
        match wifi.connect().and_then(|()| wifi.wait_netif_up()) {
            Ok(()) => {
                last_err = None;
                break;
            }
            Err(e) => {
                log::warn!("WiFi: connect attempt {attempt}/{CONNECT_MAX_ATTEMPTS} failed: {e:?}");
                // Best-effort — the driver may already be back in a
                // disconnected state (that's the whole problem this
                // retry works around), so a failing disconnect() here
                // isn't itself an error worth propagating.
                let _ = wifi.disconnect();
                last_err = Some(e);
                if attempt < CONNECT_MAX_ATTEMPTS {
                    FreeRtos::delay_ms(CONNECT_RETRY_DELAY_MS);
                }
            }
        }
    }
    if let Some(e) = last_err {
        return Err(e.into());
    }

    let ip_info = wifi.wifi().sta_netif().get_ip_info()?;
    let ip = ip_info.ip;

    // Compute the directed subnet broadcast from CIDR mask. Bits not
    // covered by the prefix are flipped on, so e.g. 192.168.1.42 /24
    // → 192.168.1.255.
    let prefix = ip_info.subnet.mask.0;
    let mask_u32 = if prefix == 0 {
        0
    } else if prefix >= 32 {
        u32::MAX
    } else {
        u32::MAX << (32 - prefix)
    };
    let ip_u32: u32 = u32::from_be_bytes(ip.octets());
    let bcast_u32 = ip_u32 | !mask_u32;
    let subnet_broadcast = std::net::Ipv4Addr::from(bcast_u32.to_be_bytes());

    log::info!(
        "WiFi STA up: {}/{} (gw {}, bcast {}) ch={:?}",
        ip, prefix, ip_info.subnet.gateway, subnet_broadcast, channel
    );

    Ok(WifiHandle {
        _wifi: wifi,
        ip,
        subnet_broadcast,
    })
}
