//! NTP time sync (`esp_idf_svc::sntp`), so a cold boot can reach an
//! absolute UTC clock — which WSPR's 2-minute slot grid needs and
//! nothing else in this crate provides.
//!
//! **Distinct from [`crate::time_sync`]**: that module is
//! self-referential DT feedback against the receiver's own *decoded*
//! signal (monotonic-clock-only, no notion of absolute UTC at all —
//! it corrects drift once a decode has already happened). It cannot
//! bootstrap a cold-start clock: getting a usable decode in the first
//! place already assumes the receiver is slot-aligned, which is
//! exactly what's missing before this module runs. This one talks to
//! an external time source instead and is the thing that actually
//! sets the system clock.
//!
//! No new Cargo dependency or sdkconfig change — `esp_idf_svc::sntp`
//! is already reachable through the existing `esp-idf-svc` dependency
//! (verified by reading the checked-out crate source directly, not
//! assumed from docs).

use esp_idf_svc::sntp::{EspSntp, SntpConf, SyncStatus};
use esp_idf_svc::sys::EspError;

/// Start SNTP against one configured server.
///
/// `SntpConf::servers` is a fixed-size array (`SNTP_MAX_SERVERS`
/// slots, currently 4) that upstream's own `new_default()` fills with
/// four different `*.pool.ntp.org` hosts — every slot is used as a
/// fallback if an earlier one doesn't answer. This crate only exposes
/// one configurable server (`Settings::ntp_server`), so every slot
/// gets that same host rather than leaving the rest empty: an empty
/// `to_cstring_arg("")` slot is untested territory in this dependency
/// (upstream's own example only ever calls `new_default()`), and
/// repeating the one known-good host side-steps that edge case
/// instead of relying on it being harmless.
///
/// Returned handle must be kept alive by the caller — dropping it
/// calls `sntp_stop()` (see `EspSntp`'s own `Drop` impl).
pub fn start(server: &str) -> Result<EspSntp<'static>, EspError> {
    let mut conf = SntpConf::default();
    for slot in conf.servers.iter_mut() {
        *slot = server;
    }
    log::info!("NTP: starting sync against {server}");
    EspSntp::new(&conf)
}

/// Poll [`EspSntp::get_sync_status`] until it reports `Completed` or
/// `timeout_ms` elapses. Returns whether sync completed.
///
/// esp-idf-svc's SNTP wrapper is callback-based
/// (`sntp_set_time_sync_notification_cb`) with no blocking wait
/// exposed to safe Rust, so — like every other `wait_*` helper this
/// crate has rolled for a similarly callback-shaped esp-idf API —
/// this is a plain poll loop, not a genuine blocking call.
pub fn wait_synced(sntp: &EspSntp<'_>, timeout_ms: u32) -> bool {
    const POLL_INTERVAL_MS: u32 = 200;
    let mut waited_ms = 0u32;
    loop {
        if sntp.get_sync_status() == SyncStatus::Completed {
            log::info!("NTP: synced after {waited_ms} ms");
            // The one place that can say the clock is disciplined
            // rather than merely plausible. `rtc::write_from_system_clock`
            // is gated on this; see `time_sync::ClockSource`.
            crate::time_sync::note_clock_from_ntp();
            return true;
        }
        if waited_ms >= timeout_ms {
            log::warn!("NTP: sync did not complete within {timeout_ms} ms");
            return false;
        }
        esp_idf_svc::hal::delay::FreeRtos::delay_ms(POLL_INTERVAL_MS);
        waited_ms += POLL_INTERVAL_MS;
    }
}
