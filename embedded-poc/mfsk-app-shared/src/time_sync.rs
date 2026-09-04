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

use crate::parity::Parity;

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

// ────────────────────────────────────────────────────────────────
// Cross-slot phase filter — the "tracking" half of #356

/// Filtered slot-phase estimate, microseconds, wrapped to
/// (−½ period, ½ period]. `i32::MIN` = no observation yet.
static GRID_PHASE_US: AtomicI32 = AtomicI32::new(i32::MIN);

/// Observations folded into [`GRID_PHASE_US`]. The "still settling" /
/// "locked after a few" counter #356 wants — a single-slot median is
/// too noisy to lock a grid to, four is enough.
static GRID_PHASE_OBS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// EMA weight for a new per-slot observation. `0.4` reaches a step
/// within ~4 slots (≈1 min on FT8) and then holds against a single
/// deep-fade slot's outlier — the issue's "four slots is enough" made
/// a filter constant.
const GRID_PHASE_ALPHA: f32 = 0.4;

/// Fold one slot's phase observation into the cross-slot filter.
///
/// `dt_sec` is that slot's own phase estimate — the decode-DT median
/// while tracking, the `ft8::acquire` circular estimate during cold
/// start — and `period_s` is the slot length, so the EMA update
/// wraps correctly at ±½ period (`+7.4 s` and `−7.4 s` are one small
/// step apart, not a period). A single-slot median stays the *input*;
/// this is the layer across slots that `MEDIAN_DT_US` never had.
pub fn observe_slot_phase(dt_sec: f32, period_s: f32) {
    if !dt_sec.is_finite() || !period_s.is_finite() || period_s <= 0.0 {
        return;
    }
    let period = (period_s * 1e6) as i32;
    let half = period / 2;
    let wrap = |x: i32| -> i32 {
        let m = x.rem_euclid(period);
        if m > half { m - period } else { m }
    };
    let obs = wrap((dt_sec * 1e6) as i32);
    let prev = GRID_PHASE_US.load(Ordering::Acquire);
    let next = if prev == i32::MIN {
        obs
    } else {
        wrap(prev + (GRID_PHASE_ALPHA * wrap(obs - prev) as f32) as i32)
    };
    GRID_PHASE_US.store(next, Ordering::Release);
    GRID_PHASE_OBS.fetch_add(1, Ordering::AcqRel);
}

/// The cross-slot-filtered grid phase, seconds, or `None` before the
/// first [`observe_slot_phase`].
pub fn filtered_slot_phase() -> Option<f32> {
    let v = GRID_PHASE_US.load(Ordering::Acquire);
    (v != i32::MIN).then_some(v as f32 / 1e6)
}

/// How many slot observations the filter has folded in.
pub fn slot_phase_observations() -> u32 {
    GRID_PHASE_OBS.load(Ordering::Acquire)
}

/// Clear the filter — on QSY, or a deliberate re-acquire.
pub fn reset_slot_phase() {
    GRID_PHASE_US.store(i32::MIN, Ordering::Release);
    GRID_PHASE_OBS.store(0, Ordering::Release);
}

// ────────────────────────────────────────────────────────────────
// Capture-slot index + parity (issue #110).
//
// Published by the audio source (capture_thread / wav_sim driver /
// TxTest 15 s timer) at each slot boundary. Read by the TX scheduler
// to decide whether to fire in this slot (parity match) and how much
// time is left in the slot (TX must complete within 15 s).
//
// `wav_idx` is the audio-source canonical slot counter — same value
// that ends up in `embedded_shared::pipeline::SpecBundle.wav_idx`
// and `Slot.wav_idx`. Using one counter for both audio framing and
// FSM gating guarantees the FSM's notion of "peer's slot" matches
// the audio that actually fed the decode.
//
// Stored together behind a `Mutex` because Xtensa LX6 / LX7 don't
// expose lock-free 64-bit atomics — and we need the (wav_idx,
// start_us) pair to be torn-read-free anyway. Contention is trivial:
// producer writes once per 15 s slot boundary, consumer reads once
// per 100 ms scheduler poll.
//
// `None` = not yet published (cold boot). Readers return `None` so
// the TX scheduler skips this iteration rather than firing on parity
// bit 0 of an undefined counter.

static CAPTURE_SLOT: Mutex<Option<(u32, i64)>> = Mutex::new(None);

/// Audio source side. Call once per slot boundary, AT slot START
/// (i.e. when capture of the new slot's audio begins). `wav_idx` is
/// the slot identifier that will travel with this slot's audio
/// through stage1_inc → SpecBundle/Slot → decode. `now_mono_us` is
/// `esp_timer_get_time()` (or equivalent monotonic μs).
pub fn publish_capture_slot(wav_idx: u32, now_mono_us: i64) {
    if let Ok(mut g) = CAPTURE_SLOT.lock() {
        *g = Some((wav_idx, now_mono_us));
    }
}

/// Current capture slot's `wav_idx`, or `None` before the audio
/// source has published any boundary.
pub fn current_capture_wav_idx() -> Option<u32> {
    CAPTURE_SLOT.lock().ok().and_then(|g| g.map(|(w, _)| w))
}

/// Parity of the current capture slot. `None` before first publish.
pub fn current_capture_parity() -> Option<Parity> {
    current_capture_wav_idx().map(Parity::from_slot_index)
}

/// Microseconds elapsed since the current capture slot started.
/// Clamped to `>= 0` in case the monotonic clock somehow regresses
/// (it doesn't on esp-idf, but defensive). `None` before first publish.
pub fn time_into_capture_slot_us(now_mono_us: i64) -> Option<i64> {
    CAPTURE_SLOT
        .lock()
        .ok()
        .and_then(|g| g.map(|(_, start)| (now_mono_us - start).max(0)))
}

/// Atomic combined read of `(wav_idx, time_into_slot_us)`. Callers
/// that need *both* values to refer to the same slot (e.g. the TX
/// scheduler's parity + launch-deadline check) must use this instead
/// of pairing `current_capture_wav_idx()` with
/// `time_into_capture_slot_us()` — the two-call form holds the
/// mutex twice and races with `publish_capture_slot()` if a slot
/// boundary crosses between them, yielding `(slot N, time_into_N+1)`.
/// `None` before the audio source has published any boundary.
pub fn current_capture_info(now_mono_us: i64) -> Option<(u32, i64)> {
    CAPTURE_SLOT
        .lock()
        .ok()
        .and_then(|g| g.map(|(w, start)| (w, (now_mono_us - start).max(0))))
}

/// Test-only reset. The statics persist for the whole process so
/// each test case starts uninitialised.
#[cfg(test)]
pub fn reset_capture_slot_for_test() {
    if let Ok(mut g) = CAPTURE_SLOT.lock() {
        *g = None;
    }
}

// ──────────────────────────────────────────────────────────────────────
// UTC slot phase — the bootstrap the two correction sources above cannot
// provide

/// Below this, the system clock has not been set by NTP and is still
/// counting from the epoch the ESP-IDF boot left it at. 2020-09-13.
const PLAUSIBLE_UNIX_SECS: u64 = 1_600_000_000;

/// Wall-clock UTC in milliseconds, or `None` while the clock is still
/// unset.
///
/// The distinction matters more than it looks: a receiver that aligns
/// its slots to an unset clock is not "roughly aligned", it is aligned
/// to an arbitrary phase *and cannot tell*. Callers are expected to
/// say which of the two they are doing.
pub fn utc_now_ms() -> Option<u64> {
    let d = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .ok()?;
    (d.as_secs() >= PLAUSIBLE_UNIX_SECS).then_some(d.as_millis() as u64)
}

/// The modular arithmetic behind [`samples_to_next_slot_12k`] and
/// [`samples_to_next_slot_12k_ms`], with the clock read passed in.
///
/// Split out so the phase maths can be tested without a settable
/// system clock — `utc_now_ms` reads `SystemTime::now()` and there is
/// no seam. Exactly on a boundary (`now_ms % period_ms == 0`) this
/// returns a whole slot, not zero: the *next* boundary is a full
/// period away.
fn samples_to_next_slot_12k_from(now_ms: u64, period_ms: u64) -> usize {
    let into_slot = now_ms % period_ms;
    let remain_ms = period_ms - into_slot;
    (remain_ms * 12) as usize
}

/// 12 kHz samples from now until the next UTC boundary of a
/// `period_ms`-millisecond slot grid.
///
/// FT4's slot is 7.5 s, which [`samples_to_next_slot_12k`]'s whole-second
/// argument cannot express — this is the same phase source in
/// milliseconds. See that function for why UTC is the bootstrap the
/// DT-median correction cannot be.
///
/// `None` while the clock is unset ([`utc_now_ms`]).
pub fn samples_to_next_slot_12k_ms(period_ms: u64) -> Option<usize> {
    Some(samples_to_next_slot_12k_from(utc_now_ms()?, period_ms))
}

/// 12 kHz samples from now until the next UTC boundary of a
/// `slot_len_s`-second slot grid, and how far the current stream
/// position is from that grid.
///
/// This is the phase source a capture path needs and neither
/// [`slot_dt_offset`] nor GPS currently supplies. The DT-median
/// correction above is a *refinement*: it needs decodes to exist
/// before it can say anything, and on FT8 a slot that is more than
/// ±2.5 s out produces no decodes at all — so it cannot pull itself
/// up from an arbitrary phase. UTC can, and NTP is already in every
/// app that has WiFi.
///
/// `None` while the clock is unset ([`utc_now_ms`]).
pub fn samples_to_next_slot_12k(slot_len_s: u64) -> Option<usize> {
    samples_to_next_slot_12k_ms(slot_len_s * 1_000)
}

// ──────────────────────────────────────────────────────────────────────
// Where the system clock came from

/// What last set the system clock.
///
/// A `SystemTime` carries a value and nothing else, and that is the
/// whole of the bug this exists to prevent: on the CoreS3 the boot
/// order is `pmic::init` → `rtc::read_into_system_clock` → (30 s
/// later) WiFi → NTP, so by the time anything asks "is the clock
/// set?" the answer is yes and the honest answer is "yes, from the
/// chip we were about to correct".
///
/// Measured consequence, 2026-09-02: every CoreS3 log from 08-31
/// onwards shows `rtc: BM8563 set to X` about one second after
/// `rtc: system clock set from BM8563 — X`, with the same X, and
/// *before* `NTP: starting sync`. The chip had been writing its own
/// value back to itself on every boot for the whole history of those
/// logs, so NTP never reached it. Worse, the round trip is lossy in
/// one direction — `read_epoch` returns whole seconds and
/// `read_into_system_clock` commits them with `tv_usec: 0`, then the
/// write happens ~1 s later and truncates again — so each boot set
/// the RTC back by between 0.98 and 1.98 s. The 8 h drift run found
/// the chip **186 s** behind NTP at t0, which 62.75 ppm cannot
/// explain over any plausible interval; ~120 boots of that ratchet
/// can (#354).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// Never set: still counting from whatever the ESP-IDF boot left.
    Unset,
    /// Read out of the battery-backed RTC. Plausible, not disciplined.
    Rtc,
    /// Disciplined by NTP. The only source worth writing back to the
    /// RTC.
    Ntp,
}

static CLOCK_SOURCE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// The system clock now holds a value read out of the RTC.
pub fn note_clock_from_rtc() {
    // Never demote a disciplined clock: NTP may already have run.
    let _ = CLOCK_SOURCE.compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire);
}

/// The system clock has been disciplined by NTP.
pub fn note_clock_from_ntp() {
    CLOCK_SOURCE.store(2, Ordering::Release);
}

/// What last set the system clock.
pub fn clock_source() -> ClockSource {
    match CLOCK_SOURCE.load(Ordering::Acquire) {
        1 => ClockSource::Rtc,
        2 => ClockSource::Ntp,
        _ => ClockSource::Unset,
    }
}

/// Whether the system clock is good enough to write back into the RTC.
///
/// This is the predicate the three RTC-write sites want.
/// [`utc_now_ms`] is not: it answers "is this value plausible", which
/// an RTC-seeded clock satisfies by construction.
pub fn clock_is_disciplined() -> bool {
    clock_source() == ClockSource::Ntp
}

// ────────────────────────────────────────────────────────────────
// What the slot grid's phase is anchored to

/// What the slot grid's *phase* is currently anchored to — a separate
/// question from [`ClockSource`], which is what last set the system
/// *clock*.
///
/// The two come apart in the field (#356): with no network, the grid
/// can be locked to off-air FT8 Costas phase (`Air`) while the system
/// clock is only ever a plausible `Rtc` value. The issue's rule — "an
/// operator must never have to guess what the grid is anchored to" — is
/// why this is a first-class value, meant for the panel, every log line,
/// and the ADIF record.
///
/// Ordered worst-to-best so `>=` comparisons read naturally. `Air` and
/// `Ntp` are both "good": `Ntp` additionally carries a UTC epoch,
/// `Air` carries only phase-mod-slot-length, but for the capture window
/// that is all either one is used for.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug, Default)]
pub enum GridLock {
    /// Never anchored — free-running from the first captured sample.
    #[default]
    FreeRun,
    /// A plausible RTC value set the phase. Good to seconds, drifting
    /// at tens of ppm.
    Rtc,
    /// Locked to off-air FT8 Costas phase (#356). No UTC epoch; the
    /// phase, mod the slot length, tracks the band's median clock.
    Air,
    /// NTP disciplined the system clock and the grid follows it.
    Ntp,
}

impl GridLock {
    /// Short tag for the link bar / log line / ADIF `APP_*` field.
    pub fn label(self) -> &'static str {
        match self {
            GridLock::FreeRun => "free-run",
            GridLock::Rtc => "rtc",
            GridLock::Air => "air",
            GridLock::Ntp => "ntp",
        }
    }
}

static GRID_LOCK: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// Record what anchored the grid phase. Idempotent; the caller owns the
/// precedence policy (e.g. whether a fresh `Air` lock should override a
/// stale `Ntp` one) — this only stores.
pub fn note_grid_lock(state: GridLock) {
    GRID_LOCK.store(state as u8, Ordering::Release);
}

/// What the grid phase is anchored to right now.
pub fn grid_lock() -> GridLock {
    match GRID_LOCK.load(Ordering::Acquire) {
        1 => GridLock::Rtc,
        2 => GridLock::Air,
        3 => GridLock::Ntp,
        _ => GridLock::FreeRun,
    }
}

/// Whether the grid is anchored well enough to demodulate against —
/// `Air` or `Ntp`. `Rtc` is deliberately *not* enough: it can be
/// seconds out, past what the ±1 s coarse search can recover (#356).
pub fn grid_is_locked() -> bool {
    grid_lock() >= GridLock::Air
}

#[cfg(test)]
mod clock_source_tests {
    use super::*;

    /// The ordering the CoreS3 boot actually performs, and the one
    /// that used to launder an RTC error back into the chip.
    #[test]
    fn rtc_then_ntp_ends_disciplined_and_rtc_cannot_demote() {
        CLOCK_SOURCE.store(0, Ordering::Release);
        assert_eq!(clock_source(), ClockSource::Unset);
        assert!(!clock_is_disciplined());

        note_clock_from_rtc();
        assert_eq!(clock_source(), ClockSource::Rtc);
        // The whole point: a plausible clock is not a disciplined one.
        assert!(!clock_is_disciplined());

        note_clock_from_ntp();
        assert!(clock_is_disciplined());

        // A later RTC read (a second receiver starting, say) must not
        // pull the state back down.
        note_clock_from_rtc();
        assert!(clock_is_disciplined());
    }
}

#[cfg(test)]
mod slot_phase_filter_tests {
    use super::*;

    #[test]
    fn ema_converges_then_holds_against_an_outlier() {
        reset_slot_phase();
        assert!(filtered_slot_phase().is_none());

        for _ in 0..6 {
            observe_slot_phase(0.30, 15.0);
        }
        let after = filtered_slot_phase().unwrap();
        assert!((after - 0.30).abs() < 0.02, "converged to {after}");
        assert!(slot_phase_observations() >= 6);

        // One deep-fade slot throws a wild median; the filter barely moves.
        observe_slot_phase(-5.0, 15.0);
        let jerked = filtered_slot_phase().unwrap();
        assert!((jerked - 0.30).abs() < 2.4, "outlier moved it to {jerked}");
    }

    #[test]
    fn ema_wraps_at_the_half_period() {
        reset_slot_phase();
        // Estimates jittering around +7.4 / −7.4 (the same phase on a
        // 15 s grid) must not average toward zero.
        for dt in [7.4_f32, -7.4, 7.3, -7.45, 7.35] {
            observe_slot_phase(dt, 15.0);
        }
        let p = filtered_slot_phase().unwrap();
        assert!(p.abs() > 7.0, "wrapped estimate landed at {p}");
    }

    #[test]
    fn rejects_bad_input_and_resets() {
        reset_slot_phase();
        observe_slot_phase(f32::NAN, 15.0);
        observe_slot_phase(0.1, 0.0);
        observe_slot_phase(0.1, f32::INFINITY);
        assert!(filtered_slot_phase().is_none());

        observe_slot_phase(0.1, 15.0);
        assert!(filtered_slot_phase().is_some());
        reset_slot_phase();
        assert!(filtered_slot_phase().is_none());
        assert_eq!(slot_phase_observations(), 0);
    }
}

#[cfg(test)]
mod grid_lock_tests {
    use super::*;

    #[test]
    fn ordering_and_locked_predicate() {
        assert!(GridLock::FreeRun < GridLock::Rtc);
        assert!(GridLock::Rtc < GridLock::Air);
        assert!(GridLock::Air < GridLock::Ntp);

        GRID_LOCK.store(0, Ordering::Release);
        assert_eq!(grid_lock(), GridLock::FreeRun);
        assert!(!grid_is_locked());

        note_grid_lock(GridLock::Rtc);
        assert_eq!(grid_lock(), GridLock::Rtc);
        // An RTC value alone can be seconds out — not "locked".
        assert!(!grid_is_locked());

        note_grid_lock(GridLock::Air);
        assert_eq!(grid_lock(), GridLock::Air);
        assert!(grid_is_locked());

        note_grid_lock(GridLock::Ntp);
        assert!(grid_is_locked());

        // `note_grid_lock` only stores — precedence is the caller's.
        note_grid_lock(GridLock::FreeRun);
        assert_eq!(grid_lock(), GridLock::FreeRun);
    }

    #[test]
    fn labels_are_stable() {
        assert_eq!(GridLock::FreeRun.label(), "free-run");
        assert_eq!(GridLock::Rtc.label(), "rtc");
        assert_eq!(GridLock::Air.label(), "air");
        assert_eq!(GridLock::Ntp.label(), "ntp");
    }
}

#[cfg(test)]
mod slot_phase_tests {
    use super::samples_to_next_slot_12k_from;

    #[test]
    fn ft4_seven_point_five_second_grid() {
        // 2.0 s into a 7.5 s slot → 5.5 s left → 66 000 samples @ 12 kHz.
        assert_eq!(samples_to_next_slot_12k_from(2_000, 7_500), 66_000);
    }

    #[test]
    fn ft8_fifteen_second_grid_still_matches_the_seconds_form() {
        // 10 s into 15 s → 5 s → 60 000. `samples_to_next_slot_12k(15)`
        // now routes through here; this pins that it did not change.
        assert_eq!(samples_to_next_slot_12k_from(10_000, 15_000), 60_000);
    }

    #[test]
    fn on_the_boundary_is_a_whole_slot_not_zero() {
        assert_eq!(samples_to_next_slot_12k_from(0, 7_500), 90_000);
        assert_eq!(samples_to_next_slot_12k_from(45_000, 7_500), 90_000);
    }

    #[test]
    fn millisecond_offsets_the_seconds_form_could_not_reach() {
        // 3 750 ms into a 7.5 s slot — the half-slot FT4 sequencing
        // boundary — → 3.75 s → 45 000.
        assert_eq!(samples_to_next_slot_12k_from(3_750, 7_500), 45_000);
        // 7 499 ms in → 1 ms → 12 samples.
        assert_eq!(samples_to_next_slot_12k_from(7_499, 7_500), 12);
    }
}
