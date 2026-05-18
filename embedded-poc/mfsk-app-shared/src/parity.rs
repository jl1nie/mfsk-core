//! Slot parity primitives shared by the QSO FSM, TX scheduler, and
//! TX picker.
//!
//! FT8 alternates TX between even and odd 15 s slots. This module
//! owns the canonical `Parity` enum plus the framing-settled
//! threshold the FSM uses to gate peer-parity locking against the
//! bootstrap_slot_shift convergence window. See issue #110 for the
//! UTC-arbitrary / self-sync rationale.

/// One bit of slot identity. `Even` = slot index bit 0 == 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    Even,
    Odd,
}

impl Parity {
    pub fn from_slot_index(slot: u32) -> Self {
        if slot & 1 == 0 {
            Parity::Even
        } else {
            Parity::Odd
        }
    }

    pub fn opposite(self) -> Self {
        match self {
            Parity::Even => Parity::Odd,
            Parity::Odd => Parity::Even,
        }
    }
}

/// `|median(DT)|` below which we trust our slot framing enough to
/// lock `peer_parity` from a decoded message. Picked at 1.5 s — well
/// inside the FT8 ±2.5 s coarse-sync window so a clean decode at this
/// threshold is unambiguously inside OUR captured slot, not the
/// previous one.
pub const SAFE_DT_THRESHOLD_SEC: f32 = 1.5;

/// `slots_observed` minimum before peer_parity locking is allowed.
/// One bootstrap pass is normally enough to align (within ±50 ms);
/// two gives us a re-check sample.
pub const FRAMING_MIN_SLOTS_OBSERVED: u32 = 2;

/// Issue #110: must finish PTT + synth + I2S write inside the slot.
/// Budget: 15 s slot − ~12.6 s frame − 100 ms PTT settle − 1 s
/// safety margin ≈ 1.3 s window from capture-slot start. Audio +
/// TxTest schedulers both gate `time_into_capture_slot_us` ≤ this
/// before firing TX.
pub const TX_LAUNCH_DEADLINE_US: i64 = 1_300_000;

/// Issue #110 gate: is the framing-self-sync settled enough to trust
/// `wav_idx` parity for `QsoManager::process_message`'s
/// `parity_lock_ok` argument? Two conditions, both required:
///
/// 1. `time_sync::slots_observed() >= FRAMING_MIN_SLOTS_OBSERVED` —
///    the bootstrap_slot_shift estimator has had ≥2 passes to
///    converge.
/// 2. `time_sync::slot_dt_offset()` is finite and below
///    `SAFE_DT_THRESHOLD_SEC` — the slot we measured DT for is
///    aligned closely enough that a decode landing inside it can be
///    unambiguously attributed to its parity (not the adjacent
///    slot).
///
/// Returns `false` during the first ~2 slots after boot / BtnA
/// re-sync AND on any slot whose median DT is outside the safe
/// window. Board-agnostic — depends only on `time_sync` global
/// state, so both `m5stack-s3-app` and `m5stack-core2-app` (and any
/// future board crate) consume the same function instead of
/// duplicating it in each `decode_pipeline.rs`.
pub fn framing_settled_for_parity_lock() -> bool {
    use crate::time_sync;

    if time_sync::slots_observed() < FRAMING_MIN_SLOTS_OBSERVED {
        return false;
    }
    time_sync::slot_dt_offset()
        .map(|dt| dt.abs() < SAFE_DT_THRESHOLD_SEC)
        .unwrap_or(false)
}
