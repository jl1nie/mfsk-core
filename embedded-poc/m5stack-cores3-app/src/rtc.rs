//! BM8563 real-time clock (PCF8563-compatible), I2C 0x51.
//!
//! **Why this exists.** Every receiver in this binary anchors its slot
//! grid to UTC, and none of them decode without it — FT8 tolerates
//! ±2.5 s against a 15 s grid, so an unset clock produces a waterfall
//! full of signal and no decodes at all. Until now the only source of
//! that clock was NTP, which means every power-on reset threw it away
//! and every boot out of WiFi range never had one. The chip that fixes
//! that has been sitting on the bus, answering at 0x51, unused.
//!
//! Two calls, both cheap:
//!
//! - [`read_into_system_clock`] at boot, before anything needs the
//!   time. Costs one 7-byte I2C read.
//! - [`write_from_system_clock`] after NTP syncs, so the next boot
//!   starts where this one ended.
//!
//! **Register layout** (from `M5Unified`'s `PCF8563_Class`, not
//! guessed): 0x02..0x08 hold seconds, minutes, hours, day, weekday,
//! month and year, all BCD. Bit 7 of the seconds register is `VL` —
//! the oscillator has stopped since the last write, so the contents
//! mean nothing. Bit 7 of the month register is the century flag, set
//! for 19xx. The high bits of the other fields are undefined and must
//! be masked before decoding.

use anyhow::{anyhow, Result};
use esp_idf_hal::i2c::I2cDriver;

pub const RTC_I2C_ADDR: u8 = 0x51;

/// What [`read_into_system_clock`] concluded, for `[boot-summary]`.
///
/// This runs inside `pmic::init`, seconds before WiFi exists, so its
/// own log line lands in the fanout's staging ring and is overwritten
/// long before a UDP sink can replay it. In host mode that is the only
/// channel there is, which makes "did the clock come from the chip or
/// from the network" unanswerable from the record — exactly the
/// question this feature exists to settle.
pub static RTC_RESULT: crate::log_slot::LogSlot = crate::log_slot::LogSlot::new();

const REG_SECONDS: u8 = 0x02;
/// Seconds register bit 7: contents invalid since the last write.
const VL_MASK: u8 = 0x80;
/// Month register bit 7: the year is 19xx rather than 20xx.
const CENTURY_MASK: u8 = 0x80;

const I2C_TIMEOUT_TICKS: u32 = 1000;

fn bcd_to_u8(v: u8) -> u8 {
    (v >> 4) * 10 + (v & 0x0F)
}

fn u8_to_bcd(v: u8) -> u8 {
    ((v / 10) << 4) | (v % 10)
}

/// Days since 1970-01-01 for a proleptic Gregorian date.
///
/// Howard Hinnant's `days_from_civil`, the inverse of what
/// `civil_time::civil_from_unix` does. Valid for any date this chip can
/// hold.
fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = (y - era * 400) as u64; // [0, 399]
    let mp = ((m + 9) % 12) as u64; // Mar = 0
    let doy = (153 * mp + 2) / 5 + d as u64 - 1; // [0, 365]
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // [0, 146096]
    era * 146_097 + doe as i64 - 719_468
}

/// Read the clock and, if it is valid, set the system time from it.
///
/// Returns the Unix seconds it set, or `None` when the chip reports
/// `VL` (never written, or the backup cell went flat), when the fields
/// do not decode, or when the date is implausible. A wrong clock is
/// worse than no clock here: the slot grid would anchor confidently to
/// the wrong phase and the failure would look like a decoder bug.
/// Read the BM8563 and return the epoch seconds it holds, **without**
/// touching the system clock.
///
/// Split out of [`read_into_system_clock`] for the drift measurement
/// (#354): comparing the RTC against an NTP-disciplined clock means
/// reading one without disturbing the other. Everything about decoding
/// and validation is that function's; this is the half before the
/// commit.
pub fn read_epoch(i2c: &mut I2cDriver<'_>) -> Option<i64> {
    let mut buf = [0u8; 7];
    if let Err(e) = i2c.write_read(RTC_I2C_ADDR, &[REG_SECONDS], &mut buf, I2C_TIMEOUT_TICKS) {
        log::warn!("rtc: BM8563 read failed: {e} — clock stays unset");
        RTC_RESULT.store("BM8563 unreadable");
        return None;
    }
    if buf[0] & VL_MASK != 0 {
        log::info!("rtc: BM8563 reports VL — never set, or the backup cell is flat");
        RTC_RESULT.store("BM8563 VL (never set / flat cell)");
        return None;
    }

    let sec = bcd_to_u8(buf[0] & 0x7f) as u32;
    let min = bcd_to_u8(buf[1] & 0x7f) as u32;
    let hour = bcd_to_u8(buf[2] & 0x3f) as u32;
    let day = bcd_to_u8(buf[3] & 0x3f) as u32;
    let month = bcd_to_u8(buf[5] & 0x1f) as u32;
    let year = bcd_to_u8(buf[6]) as i64
        + if buf[5] & CENTURY_MASK != 0 {
            1900
        } else {
            2000
        };

    // Decode first, commit second. An I2C glitch or a half-written
    // register set produces values like 45:00:80, and BCD nibbles above
    // 9 decode to nonsense rather than failing.
    if sec > 59 || min > 59 || hour > 23 || day == 0 || day > 31 || month == 0 || month > 12 {
        log::warn!(
            "rtc: BM8563 contents do not decode ({year:04}-{month:02}-{day:02} \
             {hour:02}:{min:02}:{sec:02}) — clock stays unset"
        );
        return None;
    }

    let unix = days_from_civil(year, month, day) * 86_400
        + hour as i64 * 3600
        + min as i64 * 60
        + sec as i64;
    // Same threshold `time_sync::utc_now_ms` uses to call a clock
    // plausible; below it, setting the clock buys nothing.
    if unix < 1_600_000_000 {
        log::warn!("rtc: BM8563 holds {unix} — before this firmware existed; ignoring");
        return None;
    }

    Some(unix)
}

/// [`read_epoch`], then commit it to the system clock.
pub fn read_into_system_clock(i2c: &mut I2cDriver<'_>) -> Option<i64> {
    let unix = read_epoch(i2c)?;
    let tv = esp_idf_svc::sys::timeval {
        tv_sec: unix as esp_idf_svc::sys::time_t,
        tv_usec: 0,
    };
    // SAFETY: `tv` is a valid `timeval` for the duration of the call
    // and the timezone argument is documented as unused.
    let rc = unsafe { esp_idf_svc::sys::settimeofday(&tv, core::ptr::null()) };
    if rc != 0 {
        log::warn!("rtc: settimeofday failed (rc={rc})");
        return None;
    }
    log::info!("rtc: system clock set from BM8563 — epoch {unix}");
    {
        let mut msg: heapless::String<64> = heapless::String::new();
        use core::fmt::Write as _;
        let _ = write!(&mut msg, "clock from BM8563 (epoch {unix})");
        RTC_RESULT.store(msg.as_str());
    }
    Some(unix)
}

/// Write the system clock back to the chip.
///
/// Call after NTP syncs. Cheap enough to do unconditionally; the point
/// is that the *next* boot has a clock before WiFi exists, or without
/// WiFi at all.
pub fn write_from_system_clock(i2c: &mut I2cDriver<'_>) -> Result<()> {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_err(|e| anyhow!("system clock is before the epoch: {e}"))?
        .as_secs() as i64;
    if now < 1_600_000_000 {
        return Err(anyhow!(
            "system clock is not set ({now}) — nothing worth storing"
        ));
    }
    let (y, mo, d, h, mi, s) = mfsk_app_shared::civil_time::civil_from_unix(now);

    // Weekday is not used by anything here, but the chip has the field
    // and leaving it stale makes a register dump confusing.
    let weekday = ((days_from_civil(y, mo, d) + 4).rem_euclid(7)) as u8;

    let buf = [
        REG_SECONDS,
        u8_to_bcd(s as u8),
        u8_to_bcd(mi as u8),
        u8_to_bcd(h as u8),
        u8_to_bcd(d as u8),
        weekday & 0x07,
        u8_to_bcd(mo as u8) | if y < 2000 { CENTURY_MASK } else { 0 },
        u8_to_bcd((y % 100) as u8),
    ];
    i2c.write(RTC_I2C_ADDR, &buf, I2C_TIMEOUT_TICKS)
        .map_err(|e| anyhow!("BM8563 write failed: {e}"))?;
    log::info!("rtc: BM8563 set to {y:04}-{mo:02}-{d:02} {h:02}:{mi:02}:{s:02} UTC");
    Ok(())
}
