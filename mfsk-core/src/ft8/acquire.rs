//! Cold slot-phase acquisition from off-air FT8 (issue #356).
//!
//! A receiver with no clock has its slot grid at an arbitrary phase.
//! FT8's coarse search covers only ±2.5 s, and a single spectrogram
//! cannot be widened past ~±6.28 s (`bounded_sync_lag_steps`); a
//! widened search also drowns in the far-lag ghosts of issue #280.
//!
//! [`acquire_slot_phase`] instead runs the tested ±2.5 s search over
//! three windows of a longer capture, spaced 5 s apart so their union
//! covers the whole 15 s period, folds each window's offset back into
//! the candidate `dt_sec`, and takes a circular estimate
//! ([`circular_dt_estimate`]) over the lot.
//! `tests/ft8_cold_acquisition.rs` is the measurement
//! that chose this over the wide search — tiled recovers every offset
//! across the full period to inside one FT8 symbol; wide is off by up
//! to 720 ms with a misleadingly high confidence.
//!
//! This is the acquisition half only. Once it returns a phase, the
//! caller shifts its grid once and hands over to the existing narrow
//! ±1 s tracking (`bootstrap_dt_median` / the decode-DT median).

use alloc::vec::Vec;

use crate::engine::sync::{SyncCandidate, circular_dt_estimate};
use crate::ft8::decode_block::{coarse_sync_with_lag, compute_spectrogram};
use crate::ft8::params::NMAX;

/// One FT8 slot, samples at 12 kHz.
const SLOT: usize = NMAX;

/// Window start offsets, in seconds. Three ±2.5 s searches at 0 / 5 /
/// 10 s cover phases `[−2.5, +12.5]` = the whole 15 s period.
const WINDOW_OFFSETS_S: [f32; 3] = [0.0, 5.0, 10.0];

/// Samples of contiguous audio [`acquire_slot_phase`] needs: the last
/// window is `audio[10 s .. 25 s]`.
pub const REQUIRED_SAMPLES: usize = SLOT + (10 * 12_000);

/// The ±2.5 s window each tiled search uses — WSJT-X's own default,
/// where the far-lag ghosts of #280 are not yet a problem.
const TILE_LAG_S: f32 = 2.5;

/// One tiled search over `audio[off_samples ..][.. SLOT]`, with the
/// window's own phase offset folded into every candidate's `dt_sec`.
fn tile(
    audio: &[i16],
    off_samples: usize,
    off_s: f32,
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    out: &mut Vec<SyncCandidate>,
) {
    let window = &audio[off_samples..off_samples + SLOT];
    let spec = compute_spectrogram(window, freq_max);
    for c in coarse_sync_with_lag(&spec, freq_min, freq_max, sync_min, max_cand, TILE_LAG_S) {
        out.push(SyncCandidate {
            freq_hz: c.freq_hz,
            dt_sec: c.dt_sec + off_s,
            score: c.score,
        });
    }
}

/// Recover the slot grid's phase from `audio`, or `None` if nothing
/// coherent was found.
///
/// `audio` must hold at least [`REQUIRED_SAMPLES`] (25 s at 12 kHz);
/// only the first [`REQUIRED_SAMPLES`] are read. `top_k` is the number
/// of highest-score candidates the circular estimate averages — 5 is
/// the value `bootstrap_dt_median` established.
///
/// Returns `(dt_sec, r)`:
///
/// - `dt_sec` in `(−7.5, 7.5]` — how far the grid's phase is from the
///   band's, i.e. shift the grid by `−dt_sec` (or, equivalently, its
///   representative mod 7.5 s for an FT4 grid) to line up.
/// - `r` in `[0, 1]` — the mean resultant length: how much the
///   candidates agree. A real lock sits near 1; a quiet band or a
///   phase the search missed sits low. The caller decides the
///   threshold (≈0.6 was clean on `qso3_busy` across the whole
///   period) and how many slots to require before trusting it.
///
/// `None` if `audio` is too short or the search produced no candidates.
pub fn acquire_slot_phase(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    top_k: usize,
) -> Option<(f32, f32)> {
    if audio.len() < REQUIRED_SAMPLES {
        return None;
    }
    let mut cands = Vec::new();
    for (off_s, off_samples) in WINDOW_OFFSETS_S
        .iter()
        .map(|&s| (s, (s * 12_000.0) as usize))
    {
        tile(
            audio,
            off_samples,
            off_s,
            freq_min,
            freq_max,
            sync_min,
            max_cand,
            &mut cands,
        );
    }
    circular_dt_estimate(&cands, top_k, 15.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn too_short_audio_is_none() {
        assert_eq!(REQUIRED_SAMPLES, 300_000);
        let short = alloc::vec![0i16; REQUIRED_SAMPLES - 1];
        assert!(acquire_slot_phase(&short, 100.0, 3000.0, 1.0, 200, 5).is_none());
    }

    #[test]
    fn silence_finds_nothing() {
        let silence = alloc::vec![0i16; REQUIRED_SAMPLES];
        assert!(acquire_slot_phase(&silence, 100.0, 3000.0, 1.0, 200, 5).is_none());
    }
}
