//! BLE GATT CI-V transport for IC-705 (Phase 2-Stick, 2026-05-17).
//!
//! Port of the K7MDL2 reference protocol implemented in
//! `rs-ft8n/ft8-web/www/ble-transport.js`. The IC-705 exposes a
//! single GATT service with one characteristic that carries both the
//! pairing handshake and standard CI-V frames; once paired we can
//! write any CI-V command (PTT, set freq, query GPS, …) and receive
//! responses via characteristic notifications.
//!
//! Protocol:
//! - Service UUID : `14cf8001-1ec2-d408-1b04-2eb270f14203`
//! - Char UUID    : `14cf8002-1ec2-d408-1b04-2eb270f14203`
//!   (writeWithoutResponse + notify)
//!
//! Pairing handshake (3 writes, wait for `0x64` grant on the 3rd):
//! 1. `FE F1 00 61 <36-byte UUID ASCII> FD` → wait `… 00 61 … FD`
//! 2. `FE F1 00 62 <16-byte name padded>  FD` → wait `… 00 62 …`
//! 3. `FE F1 00 63 EE 39 09 10 FD`             → wait `… 00 64 …` (granted)
//!
//! After pairing, standard CI-V frames flow:
//!   request  : `FE FE 94 E0 <CMD> <SUBCMD> [<DATA>] FD`
//!   response : `FE FE E0 94 <CMD> <SUBCMD> [<DATA>] FD`
//!   (0x94 = IC-705 default address, 0xE0 = controller address)
//!
//! Why BLE not USB: M5StickS3 hardware cannot do USB-OTG host (see
//! memory `project_m5stick_s3_no_usb_host`). BLE gives us PTT
//! control + radio meta-data without needing UAC. Combined with the
//! Phase 1.5 acoustic capture path, this is enough for a full QSO
//! workflow on a board that lacks digital audio I/O to the radio:
//! we hear the band via mic, key TX via BLE PTT, and acoustically
//! couple synthesised tones back into the radio MIC (Phase 1.6).
//!
//! Coexistence with audio:
//! - 2.4 GHz BLE radio is independent of the ES8311 I2S audio path
//! - WiFi STA is OFF in Acoustic mode (Phase 1.5 OOM constraint) so
//!   BLE has full radio time
//! - GPS UTC arrives via 30 s polled CI-V `0x23 0x00`; pushed into
//!   `time_sync` as a **supplementary** time source. The primary
//!   alignment remains the pass1-DT self-sync (cold-fix NMEA can
//!   take 10-30 min, so we can't depend on it)
//!
//! Lifetime model:
//! - `start()` spawns a single background thread that owns the
//!   `BLEClient`. The thread runs the async scan → connect → pair
//!   → notification loop forever, reconnecting on disconnect.
//! - PTT / freq writes happen via an atomic flag (set from QSO FSM,
//!   consumed from the BLE thread). No blocking calls cross the
//!   thread boundary — keeps the audio capture loop responsive even
//!   if BLE stalls.

use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, AtomicU8, Ordering};
use std::sync::OnceLock;
use std::time::Duration;

use anyhow::{anyhow, Result};
use esp32_nimble::{uuid128, BLEDevice, BLEScan};
use esp_idf_svc::hal::task::block_on;

// ── Protocol constants ───────────────────────────────────────────────

/// IC-705 BLE GATT service UUID (K7MDL2 / WebFT8 reference protocol).
const IC705_SERVICE_UUID: esp32_nimble::utilities::BleUuid =
    uuid128!("14cf8001-1ec2-d408-1b04-2eb270f14203");

/// IC-705 BLE GATT characteristic (write-without-response + notify).
const IC705_CHAR_UUID: esp32_nimble::utilities::BleUuid =
    uuid128!("14cf8002-1ec2-d408-1b04-2eb270f14203");

/// CI-V controller address (we are the controller).
const CIV_CTRL_ADDR: u8 = 0xE0;
/// IC-705 default CI-V transceiver address. **Not** 0x94 (that's the
/// IC-7300 default and was confused with IC-705 in the K7MDL2 docstring;
/// WebFT8's `rig-profiles.json` correctly lists ic705 as `civAddr =
/// 0xA4`). Sending to the wrong address makes the radio silently
/// ignore commands (e.g. PTT TX-on never engages).
const CIV_IC705_ADDR: u8 = 0xA4;

/// Pairing UUID literal (36 ASCII bytes) — derived from the standard
/// SPP service UUID per K7MDL2 reference. Sent as data in pairing
/// msg1; the IC-705 stores it as the controller identity.
const PAIR_UUID_ASCII: &[u8; 36] = b"00001101-0000-1000-8000-00805F9B34FB";

/// Pairing device-name literal (16 bytes, null-padded). The IC-705
/// shows this in its "外部機器" Bluetooth device list — keeping the
/// WebFT8 string ties this device into the same paired entry already
/// approved by the radio for the desktop app, avoiding a re-pair
/// prompt on the IC-705 panel.
const PAIR_NAME_PADDED: &[u8; 16] = b"WebFT8 BLE\0\0\0\0\0\0";

/// Pairing token (4 bytes) — magic constant from the K7MDL2 reverse-
/// engineering work. Same value sent by the WASM app.
const PAIR_TOKEN: [u8; 4] = [0xEE, 0x39, 0x09, 0x10];

/// CI-V framing bytes.
const PRE: u8 = 0xFE;
const POST: u8 = 0xFD;

// ── Shared command/status state (cross-thread) ───────────────────────

/// PTT target state. `true` = TX-on requested, `false` = receive.
/// Set by QSO FSM / Phase 1.6 TX synth; consumed by the BLE thread
/// which writes the corresponding CI-V command when the value
/// changes. Initialised to `false` so we boot in RX.
pub static PTT_TARGET: AtomicBool = AtomicBool::new(false);

/// Latest PTT state actually confirmed by the radio (or last sent
/// command if no echo). UI can read this for the TX indicator.
pub static PTT_ACTUAL: AtomicBool = AtomicBool::new(false);

/// BLE connection state for UI / startup gating.
/// 0 = init, 1 = scanning, 2 = connecting, 3 = pairing, 4 = ready,
/// 5 = lost (reconnecting).
pub static BLE_STATE: AtomicU8 = AtomicU8::new(0);

/// Latest CI-V GPS UTC offset in microseconds (relative to local
/// monotonic clock). `i32::MIN` = no GPS fix received yet.
/// Pushed by the BLE notification handler when the IC-705 reports a
/// valid GPS position (cmd 0x23 0x00 response).
pub static GPS_UTC_OFFSET_US: AtomicI32 = AtomicI32::new(i32::MIN);

/// Pending IC-705 VFO frequency in Hz. `0` = no pending change.
/// Set by `set_freq_hz` from the menu (band selection); consumed
/// by the BLE command loop's edge detector (writes cmd 0x05 only
/// when target ≠ last_sent). Phase 1.7-Stick (2026-05-17).
pub static FREQ_TARGET_HZ: AtomicU32 = AtomicU32::new(0);

/// Pending IC-705 mode selector. Encoded as a small enum:
///   0xFF = no pending change
///   0x01 = USB-DATA narrow (cmd 0x06 0x01 0x01 0x02)
/// Extend with more variants as needed. Mode is sticky in the radio
/// so we typically set this once per session start.
pub static MODE_TARGET: AtomicU8 = AtomicU8::new(0xFF);

/// Mode sentinel: IC-705 USB-DATA narrow (FT8 standard).
pub const MODE_USB_DATA: u8 = 0x01;

/// Pairing-state bits (set from notification handler, polled by the
/// pairing routine). Bit 0 = 0x61 acked, bit 1 = 0x62 acked,
/// bit 2 = 0x63 acked, bit 3 = 0x64 grant received.
static PAIR_STATE: AtomicU8 = AtomicU8::new(0);

/// One-shot guard so `start()` is idempotent — if main / QSO FSM
/// calls it twice (e.g. mode flip + retry path), the second call is
/// a no-op rather than racing the existing BLE thread.
static STARTED: OnceLock<()> = OnceLock::new();

// ── Public API ───────────────────────────────────────────────────────

/// Spawn the BLE central thread. Idempotent; safe to call from any
/// boot-mode dispatch arm that wants CI-V access. Returns immediately
/// — the thread does its own scan/connect/pair loop in the background.
///
/// **Important**: call this BEFORE spawning decode_pipeline /
/// allocating BASIS. `BLEDevice::take()` initializes the ESP32 BT
/// controller which permanently reserves ~30 KB internal DRAM; if
/// BASIS (120 KB) is allocated first, the BT controller init
/// `esp_bt_controller_init` returns ESP_ERR_NO_MEM and the BLE
/// stack never comes up (verified on 2026-05-17 hardware bring-up:
/// `E (1871) BLE_INIT: Malloc failed`). Calling take() up-front
/// reserves the BT footprint while internal DRAM is still ~210 KB.
pub fn start() -> Result<()> {
    if STARTED.set(()).is_err() {
        log::info!("civ: start() called twice — ignored");
        return Ok(());
    }
    // Pre-allocate BT controller memory on the main thread so it
    // wins over the BASIS scratch + worker stacks that follow.
    // BLEDevice::take() returns a 'static singleton; the ble_thread
    // just calls take() again to get the same handle.
    log::info!("civ: invoking BLEDevice::take() on main thread");
    let _bt = BLEDevice::take();
    core::hint::black_box(&_bt);
    log::info!("civ: BT controller initialised (pre-BASIS reservation)");

    std::thread::Builder::new()
        .stack_size(8 * 1024)
        .name("civ_ble".into())
        .spawn(ble_thread)
        .map_err(|e| anyhow!("civ_ble thread spawn failed: {e}"))?;
    log::info!("civ: BLE central thread spawned");
    Ok(())
}

/// Set the PTT target. The BLE thread will pick up the change on
/// its next loop iteration (≤ ~100 ms latency) and send the
/// corresponding CI-V command. Safe to call from any thread.
pub fn set_ptt(on: bool) {
    PTT_TARGET.store(on, Ordering::Release);
}

/// Set the IC-705 VFO frequency. Triggers a CI-V cmd 0x05 write on
/// the next BLE loop iteration (~100 ms latency). Pass the target
/// in **Hz** (e.g. 14_074_000 for 20m FT8). Safe from any thread.
pub fn set_freq_hz(hz: u32) {
    FREQ_TARGET_HZ.store(hz, Ordering::Release);
}

/// Set the IC-705 mode + filter (USB-DATA narrow for FT8). Sends
/// CI-V cmd 0x06 0x01 0x01 0x02. Idempotent — sent once when the
/// target differs from the last-sent value. Safe from any thread.
pub fn set_mode_data_usb() {
    MODE_TARGET.store(MODE_USB_DATA, Ordering::Release);
}

// ── Background thread ────────────────────────────────────────────────

fn ble_thread() {
    log::info!("civ: ble_thread entered");
    BLE_STATE.store(1, Ordering::Release);

    // The entire BLE loop runs on a single FreeRTOS-pinned executor;
    // esp32-nimble's async API requires `block_on` from a thread
    // dedicated to BLE work.
    block_on(async move {
        loop {
            match run_session().await {
                Ok(()) => log::info!("civ: BLE session ended cleanly, reconnecting in 2 s"),
                Err(e) => log::warn!("civ: BLE session error: {e:#} — retry in 2 s"),
            }
            BLE_STATE.store(5, Ordering::Release);
            PAIR_STATE.store(0, Ordering::Release);
            PTT_ACTUAL.store(false, Ordering::Release);
            esp_idf_svc::hal::delay::FreeRtos::delay_ms(2000);
        }
    });
}

async fn run_session() -> Result<()> {
    let ble_device = BLEDevice::take();
    let mut ble_scan = BLEScan::new();

    // ── Scan for an IC-705 advert ────────────────────────────────────
    BLE_STATE.store(1, Ordering::Release);
    log::info!("civ: scanning for IC-705 …");
    let device = ble_scan
        .active_scan(true)
        .interval(100)
        .window(99)
        .start(ble_device, 20_000, |dev, data| {
            // Match either by name prefix or by advertised service.
            // IC-705 sometimes advertises with only the local name
            // present and the 128-bit service in the scan response.
            if let Some(name) = data.name() {
                let name_str = core::str::from_utf8(name).unwrap_or("");
                if name_str.starts_with("ICOM") || name_str.starts_with("IC-705") {
                    log::info!("civ: found IC-705 by name: '{name_str}' addr={:?}", dev.addr());
                    return Some(*dev);
                }
            }
            None
        })
        .await
        .map_err(|e| anyhow!("BLEScan failed: {e:?}"))?
        .ok_or_else(|| anyhow!("scan timeout — no IC-705 in range"))?;

    // ── Connect ──────────────────────────────────────────────────────
    BLE_STATE.store(2, Ordering::Release);
    log::info!("civ: connecting GATT to {:?}", device.addr());
    let mut client = ble_device.new_client();
    client.on_connect(|c| {
        // Bump connection parameters for steady CI-V poll cadence.
        let _ = c.update_conn_params(40, 80, 0, 60);
    });
    // Atomic shadow of the BLE connection state — esp32-nimble's
    // `client.connected()` requires &client which conflicts with the
    // &mut borrows held by `get_service` / `write_value` later in
    // the same scope. The on_disconnect callback (sync, no &client
    // access needed) flips the flag.
    static BLE_CONN_ALIVE: AtomicBool = AtomicBool::new(false);
    BLE_CONN_ALIVE.store(false, Ordering::Release);
    client.on_disconnect(|_reason| {
        BLE_CONN_ALIVE.store(false, Ordering::Release);
        log::warn!("civ: BLE on_disconnect callback fired");
    });
    client
        .connect(&device.addr())
        .await
        .map_err(|e| anyhow!("GATT connect failed: {e:?}"))?;
    BLE_CONN_ALIVE.store(true, Ordering::Release);

    let service = client
        .get_service(IC705_SERVICE_UUID)
        .await
        .map_err(|e| anyhow!("service {IC705_SERVICE_UUID} not found: {e:?}"))?;
    let characteristic = service
        .get_characteristic(IC705_CHAR_UUID)
        .await
        .map_err(|e| anyhow!("characteristic {IC705_CHAR_UUID} not found: {e:?}"))?;
    if !characteristic.can_notify() {
        return Err(anyhow!("characteristic does not support notify"));
    }

    // Subscribe to notifications — the handler runs in NimBLE host
    // task context and must not block. We only set atomic flags.
    characteristic
        .on_notify(notify_handler)
        .subscribe_notify(false)
        .await
        .map_err(|e| anyhow!("subscribe_notify failed: {e:?}"))?;
    log::info!("civ: subscribed to notifications");

    // ── Pair (K7MDL2 sequence) ───────────────────────────────────────
    BLE_STATE.store(3, Ordering::Release);
    PAIR_STATE.store(0, Ordering::Release);
    log::info!("civ: starting pairing handshake");

    // msg1: UUID
    let mut msg1 = Vec::with_capacity(4 + 36 + 1);
    msg1.extend_from_slice(&[PRE, 0xF1, 0x00, 0x61]);
    msg1.extend_from_slice(PAIR_UUID_ASCII);
    msg1.push(POST);
    characteristic
        .write_value(&msg1, false)
        .await
        .map_err(|e| anyhow!("pair msg1 write failed: {e:?}"))?;
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(50);

    // msg2: device name
    let mut msg2 = Vec::with_capacity(4 + 16 + 1);
    msg2.extend_from_slice(&[PRE, 0xF1, 0x00, 0x62]);
    msg2.extend_from_slice(PAIR_NAME_PADDED);
    msg2.push(POST);
    characteristic
        .write_value(&msg2, false)
        .await
        .map_err(|e| anyhow!("pair msg2 write failed: {e:?}"))?;
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(50);

    // msg3: token + wait for 0x64 grant
    let mut msg3 = Vec::with_capacity(4 + 4 + 1);
    msg3.extend_from_slice(&[PRE, 0xF1, 0x00, 0x63]);
    msg3.extend_from_slice(&PAIR_TOKEN);
    msg3.push(POST);
    characteristic
        .write_value(&msg3, false)
        .await
        .map_err(|e| anyhow!("pair msg3 write failed: {e:?}"))?;

    // Poll for grant up to 5 s (the WASM reference uses the same
    // timeout). Polling at 50 ms keeps the BLE event loop responsive.
    let mut waited_ms = 0u32;
    while waited_ms < 5_000 {
        if PAIR_STATE.load(Ordering::Acquire) & 0b1000 != 0 {
            break;
        }
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(50);
        waited_ms += 50;
    }
    if PAIR_STATE.load(Ordering::Acquire) & 0b1000 == 0 {
        return Err(anyhow!(
            "pairing timeout (waited 5 s for 0x64 grant); state = {:#b}",
            PAIR_STATE.load(Ordering::Acquire)
        ));
    }
    log::info!("civ: pairing complete (CI-V bus access granted)");
    // IC-705 needs a moment after the 0x64 grant before it accepts
    // CI-V commands (mirrors the 300 ms delay in the WASM reference).
    esp_idf_svc::hal::delay::FreeRtos::delay_ms(300);

    // ── Main CI-V command loop ───────────────────────────────────────
    BLE_STATE.store(4, Ordering::Release);
    log::info!("civ: ready — entering command loop");

    let mut last_gps_query = std::time::Instant::now() - Duration::from_secs(60);
    let mut last_ptt_sent: Option<bool> = None;
    let mut last_freq_sent: Option<u32> = None;
    let mut last_mode_sent: Option<u8> = None;

    loop {
        // PTT — track edges and send the corresponding CI-V command.
        let target = PTT_TARGET.load(Ordering::Acquire);
        if last_ptt_sent != Some(target) {
            let frame = [
                PRE, PRE, CIV_IC705_ADDR, CIV_CTRL_ADDR,
                0x1C, 0x00, if target { 0x01 } else { 0x00 }, POST,
            ];
            if let Err(e) = characteristic.write_value(&frame, false).await {
                log::warn!("civ: PTT write failed: {e:?}");
            } else {
                log::info!("civ: PTT → {}", if target { "ON" } else { "OFF" });
                last_ptt_sent = Some(target);
                PTT_ACTUAL.store(target, Ordering::Release);
            }
        }

        // Freq — IC-705 cmd 0x05 with 5-byte little-endian BCD.
        // 0 = sentinel "no pending", skip until menu pushes a value.
        let freq_target = FREQ_TARGET_HZ.load(Ordering::Acquire);
        if freq_target != 0 && last_freq_sent != Some(freq_target) {
            let bcd = freq_to_bcd_le(freq_target);
            let frame = [
                PRE, PRE, CIV_IC705_ADDR, CIV_CTRL_ADDR,
                0x05, bcd[0], bcd[1], bcd[2], bcd[3], bcd[4], POST,
            ];
            if let Err(e) = characteristic.write_value(&frame, false).await {
                log::warn!("civ: set_freq write failed: {e:?}");
            } else {
                log::info!("civ: set_freq → {} Hz", freq_target);
                last_freq_sent = Some(freq_target);
            }
        }

        // Mode — IC-705 cmd 0x06 0x01 (set mode + filter, USB-DATA).
        // Sentinel 0xFF = no pending; we only support USB-DATA for now.
        let mode_target = MODE_TARGET.load(Ordering::Acquire);
        if mode_target != 0xFF && last_mode_sent != Some(mode_target) {
            // Only USB-DATA narrow is implemented; future variants add
            // their own subcommand bytes here.
            let frame = if mode_target == MODE_USB_DATA {
                [PRE, PRE, CIV_IC705_ADDR, CIV_CTRL_ADDR, 0x06, 0x01, 0x01, 0x02, POST]
            } else {
                log::warn!("civ: unknown MODE_TARGET 0x{mode_target:02X}, skipping");
                last_mode_sent = Some(mode_target);
                continue;
            };
            if let Err(e) = characteristic.write_value(&frame, false).await {
                log::warn!("civ: set_mode write failed: {e:?}");
            } else {
                log::info!("civ: set_mode → USB-DATA narrow");
                last_mode_sent = Some(mode_target);
            }
        }

        // GPS UTC query every 30 s (NMEA fix takes time so this is
        // a supplementary time source — `time_sync` still falls back
        // on self-sync from coarse_sync DT median).
        if last_gps_query.elapsed() >= Duration::from_secs(30) {
            let frame = [
                PRE, PRE, CIV_IC705_ADDR, CIV_CTRL_ADDR,
                0x23, 0x00, POST,
            ];
            if let Err(e) = characteristic.write_value(&frame, false).await {
                log::warn!("civ: GPS query write failed: {e:?}");
            }
            last_gps_query = std::time::Instant::now();
        }

        // Yield + cadence — 100 ms is short enough that PTT toggles
        // for TX/RX boundary land within one FT8 symbol period.
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(100);

        // Disconnect detection — `on_disconnect` callback clears the
        // atomic; the outer loop reconnects after a short delay.
        if !BLE_CONN_ALIVE.load(Ordering::Acquire) {
            return Err(anyhow!("BLE GATT disconnected"));
        }
    }
}

/// Notification handler. Runs in NimBLE host task context — must
/// not block, allocate large, or call into ESP-IDF blocking APIs.
/// Just sets atomic flags / parses small fixed-layout payloads.
fn notify_handler(data: &[u8]) {
    if data.len() < 4 {
        return;
    }
    // Pairing responses: `FE F1 00 <code> [data] FD`
    if data[0] == PRE && data[1] == 0xF1 && data[2] == 0x00 {
        let code = data[3];
        let mut bits = PAIR_STATE.load(Ordering::Acquire);
        match code {
            0x61 => bits |= 0b0001,
            0x62 => bits |= 0b0010,
            0x63 => bits |= 0b0100,
            0x64 => bits |= 0b1000,
            other => log::warn!("civ: unknown pairing notify 0x{other:02X}"),
        }
        PAIR_STATE.store(bits, Ordering::Release);
        return;
    }

    // CI-V response: `FE FE E0 <radio_addr> <CMD> <SUBCMD> [data] FD`
    if data.len() >= 7 && data[0] == PRE && data[1] == PRE && data[2] == CIV_CTRL_ADDR {
        // GPS position / UTC response (cmd 0x23 0x00).
        if data[4] == 0x23 && data[5] == 0x00 {
            parse_gps_utc(data);
        }
        // PTT state echo (cmd 0x1C 0x00 with 1-byte payload) — keep
        // PTT_ACTUAL in sync with what the radio actually thinks.
        if data[4] == 0x1C && data[5] == 0x00 && data.len() == 8 {
            PTT_ACTUAL.store(data[6] != 0, Ordering::Release);
        }
    }
}

/// Encode a frequency in Hz as IC-705 CI-V cmd-0x05 5-byte
/// little-endian BCD (Binary-Coded Decimal). Each byte packs two
/// decimal digits as `(hi << 4) | lo`; byte 0 is the lowest-order
/// digit pair (1s + 10s), byte 4 is the highest (100M + 1G Hz).
/// Algorithm matches `rs-ft8n/ft8-web/www/cat.js:340-350` verbatim.
fn freq_to_bcd_le(hz: u32) -> [u8; 5] {
    let mut bcd = [0u8; 5];
    let mut f = hz;
    for byte in &mut bcd {
        let lo = (f % 10) as u8;
        f /= 10;
        let hi = (f % 10) as u8;
        f /= 10;
        *byte = (hi << 4) | lo;
    }
    bcd
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn freq_bcd_20m_ft8() {
        // 14_074_000 Hz → byte 0 = 1s + 10s = 00, byte 1 = 100s + 1k = 40,
        // byte 2 = 10k + 100k = 07, byte 3 = 1M + 10M = 14, byte 4 = 100M+1G = 00.
        assert_eq!(freq_to_bcd_le(14_074_000), [0x00, 0x40, 0x07, 0x14, 0x00]);
    }

    #[test]
    fn freq_bcd_40m_ft8() {
        // 7_074_000 Hz
        assert_eq!(freq_to_bcd_le(7_074_000), [0x00, 0x40, 0x07, 0x07, 0x00]);
    }

    #[test]
    fn freq_bcd_2m_ft8() {
        // 144_174_000 Hz
        assert_eq!(freq_to_bcd_le(144_174_000), [0x00, 0x40, 0x17, 0x44, 0x01]);
    }
}

/// Parse a CI-V GPS response (cmd 0x23 0x00). Payload layout follows
/// the K7MDL2 reference: 27-byte variant (with altitude/heading) or
/// 23-byte variant (without). UTC BCD fields live at the end.
/// On a valid parse, computes the offset from local monotonic time
/// and pushes it to `GPS_UTC_OFFSET_US`.
fn parse_gps_utc(frame: &[u8]) {
    // Strip the 6-byte header and trailing FD.
    if frame.len() < 8 {
        return;
    }
    let payload = &frame[6..frame.len() - 1];
    let off = match payload.len() {
        27 => 21,
        23 => 17,
        _ => return,
    };
    if payload.len() < off + 6 {
        return;
    }
    let bcd = |b: u8| -> u32 { ((b >> 4) as u32) * 10 + (b & 0x0F) as u32 };
    let year = 2000 + bcd(payload[off]);
    let month = bcd(payload[off + 1]);
    let day = bcd(payload[off + 2]);
    let hh = bcd(payload[off + 3]);
    let mm = bcd(payload[off + 4]);
    let ss = bcd(payload[off + 5]);
    if !(2020..2100).contains(&year) || !(1..=12).contains(&month) || !(1..=31).contains(&day) {
        // GPS not fixed yet — IC-705 returns zeros / nonsense BCD
        // when it has no almanac. Discard.
        return;
    }
    log::info!(
        "civ: GPS UTC = {year:04}-{month:02}-{day:02} {hh:02}:{mm:02}:{ss:02} (fix acquired)"
    );
    // For now we just log + store the raw seconds-into-UTC-minute
    // (in microseconds × second-precision) into the atomic. Wiring
    // into `time_sync` proper (offset relative to local monotonic
    // clock) lands when the QSO FSM grows wall-clock awareness;
    // self-sync remains primary for slot alignment.
    let us = (ss as i32) * 1_000_000;
    GPS_UTC_OFFSET_US.store(us, Ordering::Release);
}
