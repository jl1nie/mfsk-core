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
//! the candidate `dt_sec`, and reduces the lot with
//! [`circular_dt_medoid`].
//!
//! The medoid rather than a mean, and over few candidates rather than
//! many, because a real band supplies a loud outlier: on `qso3_busy`
//! fifteen stations cluster at a median DT of +0.260 s while F5RXL
//! sits at -0.770 s, strong enough to capture a score-weighted mean
//! outright (#358).
//! `tests/ft8_cold_acquisition.rs` is the measurement
//! that chose this over the wide search — tiled recovers every offset
//! across the full period to inside one FT8 symbol; wide is off by up
//! to 720 ms with a misleadingly high confidence.
//!
//! This is the acquisition half only. Once it returns a phase, the
//! caller shifts its grid once and hands over to the existing narrow
//! ±1 s tracking (`bootstrap_dt_median` / the decode-DT median).

use alloc::vec::Vec;

use crate::engine::sync::{SyncCandidate, circular_dt_clusters, circular_dt_medoid};
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

/// Candidate slot phases from `audio`, heaviest first — the shortlist
/// [`acquire_slot_phase`] used to collapse into one answer (#358).
///
/// The collapse is the broken part. The tiles propose a usable phase
/// every time; what no statistic over the candidates can do is say
/// *which* of the clusters is the grid rather than a loud outlier
/// station or a correlation artefact. So this hands back the clusters
/// and leaves the choice to the caller, whose one reliable test is to
/// try decoding at each — a phantom counts, since its sync is real and
/// its dt is the grid's.
///
/// Measured on `qso3_busy` across the whole period, fixed-point: the
/// first entry is usable 23 times in 40, and one of the first five
/// every time. Empty if `audio` is too short or nothing was found.
///
/// Each entry is `(dt_sec, weight)`, `dt_sec` in `(−7.5, 7.5]`.
pub fn acquire_slot_phases(
    audio: &[i16],
    freq_min: f32,
    freq_max: f32,
    sync_min: f32,
    max_cand: usize,
    max_out: usize,
) -> Vec<(f32, f32)> {
    if audio.len() < REQUIRED_SAMPLES {
        return Vec::new();
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
    circular_dt_clusters(&cands, CLUSTER_KERNEL_S, 15.0, max_out)
}

/// Two candidate phases this close are the same cluster. Half a second
/// is comfortably inside the coarse window and comfortably outside the
/// spread of one band's stations, which on `qso3_busy` is 1.07 s
/// end to end.
const CLUSTER_KERNEL_S: f32 = 0.5;

/// Recover the slot grid's phase from `audio`, or `None` if nothing
/// coherent was found.
///
/// **Superseded by [`acquire_slot_phases`] for acquisition** — this
/// reduces to a single answer, and #358 measured that reduction
/// returning a phase which decodes nothing on 16 of 40 offsets, at
/// `r` up to 1.00. Kept for callers that want a point estimate and
/// can tolerate that.
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
    // Estimate per tile and take the most coherent one — never pool the
    // three candidate lists into one top-`top_k`.
    //
    // Only one tile can contain the signal: its own ±2.5 s search is the
    // only one the phase falls inside. The other two still return their
    // best `max_cand` peaks — noise, and the true signal's ghosts at the
    // far edge of their windows — at scores that compete with the real
    // ones. Pooling first let those into the top-`top_k`, and the
    // circular mean of a correct 7.5 s and a spurious 2.5 s lands
    // between the two. The signature was unmistakable once the sweep
    // was fine enough to see it: the error repeated *exactly* every 5 s,
    // the tile spacing (roll 2.25/2.50/2.75 s and 7.25/7.50/7.75 s gave
    // -359/-273/-486 ms apiece). The old whole-second sweep sampled
    // only phases where the pooling happened to be harmless.
    //
    // Selected on *score mass*, not on `r`. `r` was tried first and is
    // the wrong discriminator: a tile with no signal still returns its
    // best peaks, and those cluster at its window edge just as tightly
    // as real ones do, so it reports `r` up to 1.00 while being ~7 s
    // wrong. Agreement says the candidates are consistent, not that
    // they are a signal. The summed score of the top-`top_k` does say
    // that — coarse sync's score is a signal-to-noise ratio, and the
    // tile actually holding the frames wins it outright.
    let mut best: Option<(f32, f32)> = None;
    let mut best_mass = f32::NEG_INFINITY;
    let mut cands = Vec::new();
    for (off_s, off_samples) in WINDOW_OFFSETS_S
        .iter()
        .map(|&s| (s, (s * 12_000.0) as usize))
    {
        cands.clear();
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
        if let Some((dt, r)) = circular_dt_medoid(&cands, top_k, 15.0) {
            // Same top-`top_k` the estimate averaged over.
            let mut scores: Vec<f32> = cands.iter().map(|c| c.score).collect();
            scores.sort_by(|a, b| b.partial_cmp(a).unwrap_or(core::cmp::Ordering::Equal));
            let mass: f32 = scores.iter().take(top_k).sum();
            if mass > best_mass {
                best_mass = mass;
                best = Some((dt, r));
            }
        }
    }
    best
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
