//! Persisted slot-grid phase fix — survives the reboot between boot
//! modes (#356b).
//!
//! The cold FT8 acquisition (`ft8::acquire`) recovers slot phase, not
//! absolute UTC. On the CoreS3 switching from FT8 to FT4 is a reboot,
//! so that phase has to be written somewhere non-volatile or the lock
//! is lost. It goes in the `"mfsk"` NVS namespace, alongside
//! `boot_mode`'s keys — same idiom as [`crate::settings`], deliberately
//! **not** the RTC: the RTC's seconds-only write granularity would
//! throw away the sub-second phase the acquisition worked to get, and
//! the "only NTP writes the RTC" invariant (#354, which cost a 186 s
//! ratcheted error) is worth keeping absolute.
//!
//! The phase is stored modulo the acquisition's own slot period (15 s
//! for FT8); [`GridFix::correction_for`] re-wraps it to whatever period
//! the reader's grid uses (7.5 s for FT4) and refuses a fix that is too
//! old — the ESP crystal holds phase for hours (−3.3 ppm), but not
//! days, and a stale fix pointing at the wrong phase is worse than none.

#[cfg(target_os = "espidf")]
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

#[cfg(target_os = "espidf")]
const K_OFFSET_US: &str = "grid_off_us";
#[cfg(target_os = "espidf")]
const K_PERIOD_MS: &str = "grid_per_ms";
#[cfg(target_os = "espidf")]
const K_EPOCH: &str = "grid_epoch";
#[cfg(target_os = "espidf")]
const K_R_X100: &str = "grid_r_x100";

/// A recovered grid-phase fix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GridFix {
    /// Grid phase relative to the band, microseconds, wrapped to
    /// (−½ period, ½ period].
    pub offset_us: i32,
    /// The slot period `offset_us` is modulo (15 s from FT8).
    pub period_s: f32,
    /// Unix seconds when the fix was taken — for the staleness check.
    pub epoch_at_fix: i64,
    /// The acquisition's mean-resultant confidence, `[0, 1]`.
    pub confidence: f32,
}

impl GridFix {
    /// The phase correction to apply to a grid of `target_period_s`
    /// (seconds), or `None` if the fix is too weak or too old.
    ///
    /// `now_epoch` is the reader's current wall clock (RTC-seeded is
    /// fine); `max_age_s` is the holdover budget and `min_confidence`
    /// the acceptance bar. Returns the signed phase, wrapped to
    /// (−½ target period, ½ target period] — add it to a
    /// `samples_to_next_slot` result, or hand it straight to a grid
    /// anchor.
    pub fn correction_for(
        &self,
        now_epoch: i64,
        target_period_s: f32,
        max_age_s: i64,
        min_confidence: f32,
    ) -> Option<f32> {
        if !target_period_s.is_finite() || target_period_s <= 0.0 {
            return None;
        }
        if self.confidence < min_confidence {
            return None;
        }
        if (now_epoch - self.epoch_at_fix).abs() > max_age_s {
            return None;
        }
        let mut p = self.offset_us as f32 / 1e6;
        let half = target_period_s / 2.0;
        p = p.rem_euclid(target_period_s);
        if p > half {
            p -= target_period_s;
        }
        Some(p)
    }
}

/// Read the persisted fix, or `None` if any key is absent.
#[cfg(target_os = "espidf")]
pub fn load(nvs: &EspNvs<NvsDefault>) -> Option<GridFix> {
    let offset_us = nvs.get_i32(K_OFFSET_US).ok().flatten()?;
    let period_ms = nvs.get_i32(K_PERIOD_MS).ok().flatten()?;
    let epoch_at_fix = nvs.get_i64(K_EPOCH).ok().flatten()?;
    let r_x100 = nvs.get_u8(K_R_X100).ok().flatten()?;
    Some(GridFix {
        offset_us,
        period_s: period_ms as f32 / 1000.0,
        epoch_at_fix,
        confidence: r_x100 as f32 / 100.0,
    })
}

/// Persist a fix.
#[cfg(target_os = "espidf")]
pub fn save(nvs: &mut EspNvs<NvsDefault>, fix: &GridFix) -> Result<(), esp_idf_svc::sys::EspError> {
    nvs.set_i32(K_OFFSET_US, fix.offset_us)?;
    nvs.set_i32(K_PERIOD_MS, (fix.period_s * 1000.0) as i32)?;
    nvs.set_i64(K_EPOCH, fix.epoch_at_fix)?;
    nvs.set_u8(K_R_X100, (fix.confidence * 100.0).clamp(0.0, 100.0) as u8)?;
    Ok(())
}

/// Forget the persisted fix — after applying it once, or on a
/// deliberate re-acquire.
#[cfg(target_os = "espidf")]
pub fn clear(nvs: &mut EspNvs<NvsDefault>) -> Result<(), esp_idf_svc::sys::EspError> {
    let _ = nvs.remove(K_OFFSET_US);
    let _ = nvs.remove(K_PERIOD_MS);
    let _ = nvs.remove(K_EPOCH);
    let _ = nvs.remove(K_R_X100);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix(offset_us: i32, epoch: i64, r: f32) -> GridFix {
        GridFix {
            offset_us,
            period_s: 15.0,
            epoch_at_fix: epoch,
            confidence: r,
        }
    }

    #[test]
    fn ft8_offset_rewraps_to_the_ft4_period() {
        // +6.0 s on a 15 s grid is −1.5 s on a 7.5 s grid.
        let c = fix(6_000_000, 1000, 0.9)
            .correction_for(1000, 7.5, 3600, 0.5)
            .unwrap();
        assert!((c - (-1.5)).abs() < 1e-3, "{c}");
        // −2.0 s stays −2.0 s (already inside ±3.75).
        let c = fix(-2_000_000, 1000, 0.9)
            .correction_for(1000, 7.5, 3600, 0.5)
            .unwrap();
        assert!((c - (-2.0)).abs() < 1e-3, "{c}");
    }

    #[test]
    fn same_period_is_identity() {
        let c = fix(300_000, 1000, 0.9)
            .correction_for(1000, 15.0, 3600, 0.5)
            .unwrap();
        assert!((c - 0.3).abs() < 1e-3, "{c}");
    }

    #[test]
    fn rejects_stale_and_weak() {
        assert!(
            fix(0, 1000, 0.9)
                .correction_for(1000 + 7200, 7.5, 3600, 0.5)
                .is_none()
        ); // 2 h old, 1 h budget
        assert!(
            fix(0, 1000, 0.4)
                .correction_for(1000, 7.5, 3600, 0.5)
                .is_none()
        ); // R below bar
        assert!(
            fix(0, 1000, 0.9)
                .correction_for(1000, 0.0, 3600, 0.5)
                .is_none()
        ); // bad target period
    }
}
