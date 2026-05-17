//! Slot-boundary time sync.
//!
//! Two correction sources, both feeding the same `slot_index()` /
//! `next_slot_boundary()` API:
//!
//! 1. **GPS UTC offset** (CI-V `0x23 0x00`, Phase 2) — populates
//!    `update_gps_utc()`; not implemented in this Phase 3 build.
//!
//! 2. **Median DT estimation** across the slot's decoded messages —
//!    `record_decode_dt()` per result + `finalize_slot()` at slot
//!    end. Median is robust to weak-signal misalignment + per-tone
//!    fading; same rationale as `mfsk-core`'s dt-estimator (commit
//!    269ba0a). Used as:
//!    - the fallback time source when GPS is unavailable;
//!    - a sanity check on incoming GPS packets (drop GPS values
//!      that diverge from `slot_dt_offset()` by more than ~1 s).
//!
//! DF (audio frequency offset) is not tracked here — IC-705's USB
//! Audio path is baseband-locked to its internal TCXO, so any LO
//! drift is invisible to the S3.

use core::sync::atomic::{AtomicI32, Ordering};
use std::sync::Mutex;

/// Maximum decodes-per-slot we'll keep for the median computation.
/// qso3_busy peaks at ~7 on M5StickS3 / ship config, so 50 leaves
/// >7× headroom for a more aggressive build.
const MAX_DT_PER_SLOT: usize = 50;

/// Latest finalised median(DT) for the slot, in microseconds.
/// `i32::MIN` = "no slot finalised yet", which lets readers detect
/// cold-boot vs an actual zero offset.
static MEDIAN_DT_US: AtomicI32 = AtomicI32::new(i32::MIN);

/// Number of slots whose decodes we've folded into the median —
/// status bar can show this so the user knows the time estimate
/// is settling.
static SLOT_FINALISED_COUNT: AtomicI32 = AtomicI32::new(0);

/// Live decoder buffer for the **current** slot. Sorted at
/// `finalize_slot()` time; cleared right after.
static CURRENT_SLOT_DT: Mutex<heapless::Vec<f32, MAX_DT_PER_SLOT>> =
    Mutex::new(heapless::Vec::new());

/// Append one decode's DT (seconds, signed) to the in-progress
/// slot. Drops silently when the per-slot cap is reached — the cap
/// is high enough that exceeding it on the embedded ship config
/// would itself indicate a phantom storm worth ignoring.
pub fn record_decode_dt(dt_sec: f32) {
    if !dt_sec.is_finite() {
        return;
    }
    if let Ok(mut buf) = CURRENT_SLOT_DT.lock() {
        let _ = buf.push(dt_sec);
    }
}

/// Compute the median over the current slot's collected DTs, store
/// it as the new offset, then clear the buffer. Should be called
/// once per slot, after the last `record_decode_dt()` for that
/// slot. No-op if the slot produced no decodes — we keep the
/// previous estimate, which is better than zeroing on a transient
/// loss of signal.
pub fn finalize_slot() {
    let median_us: Option<i32> = CURRENT_SLOT_DT.lock().ok().and_then(|mut buf| {
        if buf.is_empty() {
            return None;
        }
        // Median of f32 — partial_cmp because NaN was filtered at
        // record_decode_dt time; unwrap_or is defensive.
        buf.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        let median_sec = buf[buf.len() / 2];
        let v = (median_sec * 1_000_000.0).round() as i32;
        buf.clear();
        Some(v)
    });
    if let Some(us) = median_us {
        MEDIAN_DT_US.store(us, Ordering::Release);
        SLOT_FINALISED_COUNT.fetch_add(1, Ordering::AcqRel);
    }
}

/// Latest finalised median(DT) in seconds, or `None` if no slot has
/// been finalised yet (cold boot).
pub fn slot_dt_offset() -> Option<f32> {
    let v = MEDIAN_DT_US.load(Ordering::Acquire);
    if v == i32::MIN {
        None
    } else {
        Some((v as f32) / 1_000_000.0)
    }
}

/// How many slots have contributed at least one decode. Useful for
/// "estimate is still warming up" indicators in the UI.
pub fn slots_finalised() -> u32 {
    SLOT_FINALISED_COUNT.load(Ordering::Acquire).max(0) as u32
}

// ────────────────────────────────────────────────────────────────
// Bootstrap slot-boundary auto-sync for live audio sources (UAC /
// Acoustic). wav_sim doesn't use this — its slot boundaries are
// wav-defined and already aligned. NTP is also not used (山頂運用 /
// off-grid: no WiFi). Self-sync via raw coarse_sync candidate DT
// is the only realistic time source.
//
// Flow:
//   - decode_pipeline computes median DT from pass1 candidates after
//     each coarse_sync (raw, not confirmed decodes — gives a signal
//     even when 0 decodes happen yet, which is the cold-start case).
//   - It writes the corresponding sample-count offset via
//     `set_bootstrap_slot_shift_12k`.
//   - The audio source (capture_thread for ES8311 mic /
//     UAC reader_thread for IC-705 USB) reads + clears via
//     `take_bootstrap_slot_shift_12k` at SlotEnd time and lengthens
//     / shortens the next slot by that many samples.
//   - One pass through this loop is usually enough to align (within
//     ±50 ms); subsequent slots see median DT ≈ 0 and the shift goes
//     to zero, so the mechanism is idempotent in steady state.
//
// FT8 slot-shift convention:
//   median DT > 0 → signal arrived LATER than our slot's TX_START
//                 → our slot started TOO EARLY
//                 → lengthen NEXT slot by +DT samples to catch up
//   median DT < 0 → our slot started TOO LATE → shorten next slot.

/// Sample-count adjustment to add to the next slot boundary (12 kHz
/// units). Positive = next slot lasts longer than the nominal 15 s,
/// negative = shorter. Reset to 0 after the consumer reads it via
/// `take_bootstrap_slot_shift_12k`.
static BOOTSTRAP_SLOT_SHIFT_12K: AtomicI32 = AtomicI32::new(0);

/// Count of slots where coarse_sync emitted at least one pass1
/// candidate (i.e. any signal-shaped energy in band). Increments
/// every call to `set_bootstrap_slot_shift_12k`. Used by the menu
/// UX to gate Auto-DF: chicken-and-egg avoidance for the case where
/// `slots_finalised` (= confirmed BP-passing decodes) stays at 0
/// because the band is misaligned or weak. `slots_observed` lets the
/// UI say "ok, sync is at least running" after 2-3 slots even when
/// no full decode has landed yet.
static SLOTS_OBSERVED: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Producer side (decode_pipeline). Overwrites any prior unread value
/// — the most recent slot's estimate is always the freshest. Also
/// increments `SLOTS_OBSERVED` so the UX layer can know how many
/// coarse_sync windows have run.
pub fn set_bootstrap_slot_shift_12k(samples: i32) {
    BOOTSTRAP_SLOT_SHIFT_12K.store(samples, Ordering::Release);
    SLOTS_OBSERVED.fetch_add(1, Ordering::AcqRel);
}

/// How many slots have run coarse_sync since boot. Climbs every
/// ~15 s once the audio source is producing samples. UX gate for
/// Auto-DF: typically allow after `slots_observed() >= 2` so the
/// shift estimator has had a chance to settle.
pub fn slots_observed() -> u32 {
    SLOTS_OBSERVED.load(Ordering::Acquire)
}

/// Consumer side (audio capture thread). Returns the current shift
/// and clears it atomically. Call at SlotEnd time.
pub fn take_bootstrap_slot_shift_12k() -> i32 {
    BOOTSTRAP_SLOT_SHIFT_12K.swap(0, Ordering::AcqRel)
}
