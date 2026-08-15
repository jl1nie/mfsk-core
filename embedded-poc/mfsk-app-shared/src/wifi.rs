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
///
/// **Widened to a full disconnect+rescan+reconfigure retry, not just
/// a bare re-`connect()`**, matching `wifikey2`'s own
/// `WifiManager::reconnect()` — a first cut here only retried
/// `connect()+wait_netif_up()` on the same `set_configuration()` from
/// the first attempt, which fixed the "Association refused" pattern
/// but not a second, distinct failure mode also seen this session
/// (L2 associates fine, then `wait_netif_up()` times out with no
/// association-refusal log line at all).
///
/// **That second failure mode was bisected to completion, 2026-08-15,
/// with a temporary `LWIP_DHCP_DEBUG` build (not kept — see this
/// file's git history for the throwaway diagnostic).** Root cause:
/// **not** this crate's code at all. lwIP's own DHCP client traced
/// out perfectly correct behaviour every time — `dhcp_discover()`
/// broadcasting a `DISCOVER` to `255.255.255.255:67`, retried on
/// exponential backoff (500 ms → 1 s → 2 s → 4 s ×N) — with **zero**
/// `OFFER`/`NAK` ever received in reply, across 4 full connect
/// attempts (28+ individual `DISCOVER`s). Neither this retry shape
/// nor `esp_wifi_set_ps(0)` above changed that outcome in isolation
/// (each was bisected out separately and the failure reproduced
/// identically both times); only a DHCP reservation for this device's
/// MAC on the router made the server actually answer. This is
/// unambiguously a router/DHCP-server-side problem — dynamic
/// (pool-based) allocation wasn't answering this client at all, while
/// a static reservation bypasses whatever that path's issue was.
/// Both changes stay regardless, as legitimate general-purpose
/// improvements (the association-refusal fix above is real and
/// unrelated; power-save-disable is a defensible default for a
/// mains-powered debug/logging device even though it didn't fix this
/// particular symptom) — see this crate's design memory for the full
/// investigation trail, including the two single-change bisection
/// attempts that ruled each of them out individually.
const CONNECT_MAX_ATTEMPTS: u32 = 4;
/// Delay between attempts. `wifikey2::WifiManager::reconnect()` uses
/// a fixed 3000 ms with the comment "wait for AP to clear association
/// (comeback time ≈ 1 s)" — matched here rather than kept at this
/// file's previous, narrower 2000 ms, now that the retry itself also
/// matches that implementation's shape.
const CONNECT_RETRY_DELAY_MS: u32 = 3_000;

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

    // Disable WiFi modem-sleep power-save. **2026-08-15, real-hardware
    // finding**: repeated boots against this session's own bridge-mode
    // AP showed L2 association succeeding instantly every time
    // (`wifi:connected with ...`) followed by *zero* DHCP activity for
    // the full 15 s `wait_netif_up()` timeout — not an association
    // failure, a silent DHCP one. Default power-save (`WIFI_PS_MIN_MODEM`)
    // buffers broadcast/multicast frames for delivery only at DTIM
    // beacons; a DHCP OFFER is exactly such a frame, and some APs (this
    // session's included) don't reliably deliver it to a station that's
    // still asleep between DTIMs this early in association. Same root
    // cause class `esp-idf-svc`'s own `espnow.rs` disables modem-sleep
    // for (`esp_wifi_set_ps(0)`, citing
    // <https://github.com/espressif/esp-idf/issues/7496>) — that one's
    // about queued unicast frames rather than a lost broadcast DHCP
    // reply, but the fix is the identical call. Applied before `scan()`/
    // `connect()` so it's in effect for the whole DHCP handshake, not
    // just steady-state traffic after.
    //
    // **Bisection, 2026-08-15**: temporarily disabling this call (DHCP
    // reservation also removed) showed the wifikey2-style full-reconnect
    // retry shape *alone* is NOT sufficient — 4/4 attempts failed with
    // the identical DHCP-silent pattern. Re-enabled here to test power-
    // save-disable in isolation (DHCP reservation still removed) —
    // narrows down whether this call specifically is load-bearing.
    esp_idf_svc::sys::esp!(unsafe { esp_idf_svc::sys::esp_wifi_set_ps(0) })?;

    // See CONNECT_MAX_ATTEMPTS's own doc comment for why each attempt
    // redoes the scan + set_configuration from scratch (mirrors
    // wifikey2::WifiManager::reconnect() + connect_to_profile()),
    // rather than reusing one scan/configuration across retries. The
    // ssid/psk `try_into()` is cheap enough to just repeat per attempt
    // too — simpler than naming and cloning the intermediate
    // `heapless::String` across attempts, which trips over `heapless`
    // being pulled in at two different (structurally identical but
    // nominally distinct) versions via `embedded_svc` vs. this crate's
    // own direct dependency.
    let mut last_err = None;
    let mut channel = None;
    for attempt in 1..=CONNECT_MAX_ATTEMPTS {
        let outcome = (|| -> Result<()> {
            let ap_infos = wifi.scan()?;
            channel = ap_infos.iter().find(|ap| ap.ssid.as_str() == ssid).map(|ap| ap.channel);
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
            Ok(())
        })();

        match outcome {
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
