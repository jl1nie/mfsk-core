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
/// Ceiling once the retry loop has backed off — see
/// [`connect_with_retry`]'s doc comment for what the backoff is
/// protecting.
const CONNECT_RETRY_MAX_DELAY_MS: u32 = 60_000;
/// Attempts to make at the fast [`CONNECT_RETRY_DELAY_MS`] cadence
/// before backing off. Deliberately the same count as
/// [`CONNECT_MAX_ATTEMPTS`]: that is the window the bisection above
/// established as what actually fixes the AP's comeback-time
/// coin-flip, and nothing here should shorten it.
const CONNECT_FAST_ATTEMPTS: u32 = CONNECT_MAX_ATTEMPTS;

/// Live WiFi STA handle. Drop ends the WiFi association (and any sockets
/// bound on top stop receiving), so callers must keep this alive.
///
/// Only returned by [`connect_sta`]/[`connect_sta_persistent`], which
/// own the driver for their caller's convenience. A caller using
/// [`wifi_driver_init`]/[`connect_with_retry`] directly already holds
/// its own `BlockingWifi` and gets [`WifiConnectedInfo`] instead — see
/// that split's own doc comment on [`wifi_driver_init`] for why.
pub struct WifiHandle {
    _wifi: BlockingWifi<EspWifi<'static>>,
    pub ip: std::net::Ipv4Addr,
    /// Directed subnet broadcast (`ip | ~mask`). Preferred over
    /// limited broadcast `255.255.255.255` because some APs and
    /// routers drop the limited form for power-save / multicast
    /// suppression but pass directed-broadcast through.
    pub subnet_broadcast: std::net::Ipv4Addr,
}

/// Just the connection result — no driver ownership. Returned by
/// [`connect_with_retry`] for callers that already hold their own
/// `BlockingWifi` (constructed via [`wifi_driver_init`]) and manage
/// its lifetime themselves.
pub struct WifiConnectedInfo {
    pub ip: std::net::Ipv4Addr,
    /// See [`WifiHandle::subnet_broadcast`]'s own doc comment.
    pub subnet_broadcast: std::net::Ipv4Addr,
}

/// Connect to one specific SSID. Blocks until DHCP returns an IP or
/// the retry budget in [`CONNECT_MAX_ATTEMPTS`] is exhausted.
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
    connect_sta_inner(modem, sysloop, nvs, ssid, psk, Some(CONNECT_MAX_ATTEMPTS))
}

/// Like [`connect_sta`], but **retries forever** instead of giving up
/// after [`CONNECT_MAX_ATTEMPTS`]. For a caller that can afford to
/// wait indefinitely for WiFi in the background — never call this
/// from a boot path that other work needs to proceed past.
///
/// **2026-08-15, real-hardware finding**: this AP+device combination
/// fails to connect on the first attempt often enough that a small
/// fixed retry budget isn't reliable — confirmed reproducing even
/// with `wifikey2` (this file's own stated reference implementation,
/// a previously-working, independent codebase) against the exact same
/// AP. `wifikey2`'s own caller (`main.rs`) works around this with an
/// *unbounded* outer `loop { ...; FreeRtos::delay_ms(5000); }` around
/// one connect attempt — no cap at all. Ported that shape here rather
/// than just raising [`CONNECT_MAX_ATTEMPTS`], per the user's own
/// direction after this was diagnosed together. Symptoms seen
/// alternating across attempts at this same AP (see this crate's
/// design memory for the full trail): sometimes a WPA2 4-way-handshake
/// timeout (`wifi:m f deauth reason:15`, station-side EAPOL giving
/// up), sometimes a fully-associated station whose DHCP `DISCOVER`
/// gets zero replies — different failure points, same AP, same
/// resolution (keep trying).
pub fn connect_sta_persistent<M>(
    modem: M,
    sysloop: EspSystemEventLoop,
    nvs: Option<EspDefaultNvsPartition>,
    ssid: &str,
    psk: &str,
) -> Result<WifiHandle>
where
    M: WifiModemPeripheral + 'static,
{
    connect_sta_inner(modem, sysloop, nvs, ssid, psk, None)
}

/// Shared implementation. `max_attempts: None` retries forever
/// (see [`connect_sta_persistent`]); `Some(n)` gives up after `n`
/// attempts, returning the last error (see [`connect_sta`]).
fn connect_sta_inner<M>(
    modem: M,
    sysloop: EspSystemEventLoop,
    nvs: Option<EspDefaultNvsPartition>,
    ssid: &str,
    psk: &str,
    max_attempts: Option<u32>,
) -> Result<WifiHandle>
where
    M: WifiModemPeripheral + 'static,
{
    let mut wifi = wifi_driver_init(modem, sysloop, nvs)?;
    let info = connect_with_retry(&mut wifi, ssid, psk, max_attempts)?;
    Ok(WifiHandle {
        _wifi: wifi,
        ip: info.ip,
        subnet_broadcast: info.subnet_broadcast,
    })
}

/// Construct the WiFi driver and bring the radio up — the part of
/// bring-up that **cannot be retried** if it fails: `modem` is
/// consumed by value into `EspWifi::new()`, and esp-idf-svc doesn't
/// hand it back on `Err` (checked against the pinned checkout: the
/// failed call's `M` is dropped along with the half-built driver).
/// There is exactly one `Modem` peripheral for the whole process
/// (`Peripherals::take()` is one-shot), so a caller that wants this
/// step to be retry-safe has to call it *before* anything else can
/// starve it of the internal DRAM it needs, not paper over a failure
/// here after the fact.
///
/// **2026-08-15, real-hardware finding**: exactly this happened when
/// `wspr-app` first moved its WiFi bring-up into a background task
/// spawned *after* releasing `SCAN_GO` — the WiFi driver's own rx
/// buffer allocation (`wifi:malloc buffer fail` / `Expected to init
/// 10 rx buffer, actual is 2` → `ESP_ERR_NO_MEM`) now raced
/// `scan_loop`'s own concurrent decode-time internal-DRAM churn
/// instead of running before it, and the resulting failure was
/// unrecoverable — no modem left to retry with. Fixed at the call
/// site by keeping *this* function synchronous, early in boot
/// (claiming WiFi's internal-DRAM needs the same way the scan/DDC/
/// display task stacks already do), and only backgrounding
/// [`connect_with_retry`] — the part that's actually slow/flaky and
/// safe to retry indefinitely against an already-alive driver.
pub fn wifi_driver_init<M>(
    modem: M,
    sysloop: EspSystemEventLoop,
    nvs: Option<EspDefaultNvsPartition>,
) -> Result<BlockingWifi<EspWifi<'static>>>
where
    M: WifiModemPeripheral + 'static,
{
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

    Ok(wifi)
}

/// Scan + associate + DHCP, against an already-constructed driver
/// (see [`wifi_driver_init`]). `max_attempts: None` retries forever
/// (see [`connect_sta_persistent`]); `Some(n)` gives up after `n`
/// attempts, returning the last error (see [`connect_sta`]).
///
/// See [`CONNECT_MAX_ATTEMPTS`]'s own doc comment for why each attempt
/// redoes the scan + `set_configuration()` from scratch (mirrors
/// `wifikey2::WifiManager::reconnect()` + `connect_to_profile()`),
/// rather than reusing one scan/configuration across retries. The
/// ssid/psk `try_into()` is cheap enough to just repeat per attempt
/// too — simpler than naming and cloning the intermediate
/// `heapless::String` across attempts, which trips over `heapless`
/// being pulled in at two different (structurally identical but
/// nominally distinct) versions via `embedded_svc` vs. this crate's
/// own direct dependency.
/// Delay before the next attempt: the established fast cadence for the
/// first [`CONNECT_FAST_ATTEMPTS`], then doubling to
/// [`CONNECT_RETRY_MAX_DELAY_MS`].
///
/// **Why back off at all, when the whole point of the unbounded mode is
/// to keep trying**: a retry loop is not free to the rest of the
/// device. Measured on the CoreS3 FST4 receiver (2026-08-22) against an
/// AP that never associates, this loop at its fixed 3 s cadence cost
/// the decoder **40% of its throughput** — `fst4_sync_search` went 711
/// -> 1380 ms per candidate and the candidate loop 33 -> 54 s, covering
/// 42 of 50 candidates instead of all 50. The WiFi driver's own task
/// runs at FreeRTOS priority 23, above anything an application creates,
/// so it preempts at will; the fix is to ask for the radio less often,
/// not to ask for a different priority.
///
/// Backing off does not change what the unbounded mode promises — it
/// still retries forever — only how often it interrupts the work the
/// device exists to do.
fn retry_delay_ms(attempt: u32) -> u32 {
    if attempt <= CONNECT_FAST_ATTEMPTS {
        return CONNECT_RETRY_DELAY_MS;
    }
    let steps = (attempt - CONNECT_FAST_ATTEMPTS).min(5);
    CONNECT_RETRY_DELAY_MS
        .saturating_mul(1u32 << steps)
        .min(CONNECT_RETRY_MAX_DELAY_MS)
}

pub fn connect_with_retry(
    wifi: &mut BlockingWifi<EspWifi<'static>>,
    ssid: &str,
    psk: &str,
    max_attempts: Option<u32>,
) -> Result<WifiConnectedInfo> {
    if ssid.is_empty() {
        return Err(anyhow!(
            "WIFI_SSID empty — cfg.toml missing or [wifi].ssid blank"
        ));
    }

    let mut last_err;
    let mut channel = None;
    let mut attempt: u32 = 0;
    loop {
        attempt += 1;
        let outcome = (|| -> Result<()> {
            // Re-scan only when there is something to learn: with the
            // channel already known, a scan is pure cost — and an
            // active scan is the most expensive thing in this loop,
            // sweeping every channel with the radio and the driver
            // task (FreeRTOS priority 23) preempting everything the
            // application is doing. Every fifth attempt anyway, in
            // case the AP moved.
            if channel.is_none() || attempt % 5 == 0 {
                let ap_infos = wifi.scan()?;
                channel = ap_infos
                    .iter()
                    .find(|ap| ap.ssid.as_str() == ssid)
                    .map(|ap| ap.channel);
                if channel.is_none() {
                    log::warn!("WiFi: SSID '{ssid}' not seen in scan; trying anyway");
                }
            }
            wifi.set_configuration(&Configuration::Client(ClientConfiguration {
                ssid: ssid
                    .try_into()
                    .map_err(|_| anyhow!("SSID too long for esp-idf"))?,
                password: psk
                    .try_into()
                    .map_err(|_| anyhow!("PSK too long for esp-idf"))?,
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
                match max_attempts {
                    Some(max) => log::warn!("WiFi: connect attempt {attempt}/{max} failed: {e:?}"),
                    None => log::warn!(
                        "WiFi: connect attempt {attempt} failed: {e:?} (retrying forever)"
                    ),
                }
                // Best-effort — the driver may already be back in a
                // disconnected state (that's the whole problem this
                // retry works around), so a failing disconnect() here
                // isn't itself an error worth propagating.
                let _ = wifi.disconnect();
                last_err = Some(e);
                let keep_going = match max_attempts {
                    Some(max) => attempt < max,
                    None => true,
                };
                if keep_going {
                    FreeRtos::delay_ms(retry_delay_ms(attempt));
                } else {
                    break;
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
        ip,
        prefix,
        ip_info.subnet.gateway,
        subnet_broadcast,
        channel
    );

    Ok(WifiConnectedInfo {
        ip,
        subnet_broadcast,
    })
}

/// Station RSSI in dBm, or `None` when not associated.
///
/// Read straight from the driver rather than tracked as state, so it
/// cannot go stale after a silent disassociation — the call succeeds
/// only while a station connection exists, which is exactly the
/// question the link bar asks.
///
/// Cached for a second because the render loops that want it run at up
/// to 10 Hz, and every call takes the WiFi driver's own lock. No
/// allocation and no `std::sync::Mutex`: a `static` one heap-allocates
/// its pthread mutex on first lock, which has already crashed this
/// firmware once.
pub fn rssi_cached() -> Option<i8> {
    // Milliseconds in a `u32`, not microseconds in an `i64`: Xtensa
    // has no 64-bit atomics. `wrapping_sub` keeps the comparison right
    // across the ~49-day wrap.
    use core::sync::atomic::{AtomicI32, AtomicU32, Ordering};
    const UNKNOWN: i32 = i32::MIN;
    static VALUE: AtomicI32 = AtomicI32::new(UNKNOWN);
    static AT_MS: AtomicU32 = AtomicU32::new(0);
    static SEEDED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

    let now = (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1000) as u32;
    if !SEEDED.swap(true, Ordering::Relaxed)
        || now.wrapping_sub(AT_MS.load(Ordering::Relaxed)) > 1_000
    {
        let mut ap: esp_idf_svc::sys::wifi_ap_record_t = unsafe { core::mem::zeroed() };
        let v = if unsafe { esp_idf_svc::sys::esp_wifi_sta_get_ap_info(&mut ap) }
            == esp_idf_svc::sys::ESP_OK
        {
            ap.rssi as i32
        } else {
            UNKNOWN
        };
        VALUE.store(v, Ordering::Relaxed);
        AT_MS.store(now, Ordering::Relaxed);
    }
    match VALUE.load(Ordering::Relaxed) {
        UNKNOWN => None,
        v => Some(v as i8),
    }
}
