//! Adaptive per-tone equaliser using the protocol's Costas pilot tones.
//!
//! Estimates the channel response `H(tone)` by averaging the pilot-tone
//! observations gathered from every [`SyncBlock`](super::SyncBlock) across
//! the frame, then applies a Wiener-regularised zero-forcing correction to
//! every symbol's complex spectrum so the downstream LLR sees flat tones.
//!
//! Protocol differences handled automatically:
//! - **FT8** (3 × Costas-7): tones 0..6 observed 3× each, tone 7 never →
//!   extrapolated as `2·H[6] − H[5]`.
//! - **FT4** (4 × Costas-4): every tone observed 4× each → extrapolation
//!   branch is not exercised.
//! - Future protocols with any subset of observed tones use the same
//!   machinery; missing tones are linearly extrapolated from their two
//!   lower neighbours.

use super::Protocol;
use num_complex::Complex;

/// Equaliser operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EqMode {
    /// No equalisation (passthrough).
    Off,
    /// Per-signal equalisation using local Costas pilot tones.
    Local,
    /// Try without EQ first; fall back to EQ only if BP decode fails.
    Adaptive,
}

/// Apply local (per-signal) Wiener equalisation to a flat symbol-spectra
/// buffer in place. `cs` is laid out row-major by symbol (length
/// `N_SYMBOLS × NTONES`) — the same layout produced by
/// [`super::llr::symbol_spectra`].
pub fn equalize_local<P: Protocol>(cs: &mut [Complex<f32>]) {
    let ntones = P::NTONES as usize;
    let _n_sym = P::N_SYMBOLS as usize;

    // Gather per-tone observations across all sync blocks.
    let mut obs: Vec<Vec<Complex<f32>>> = vec![Vec::new(); ntones];
    for block in P::SYNC_MODE.blocks() {
        let start = block.start_symbol as usize;
        for (k, &tone) in block.pattern.iter().enumerate() {
            let t = tone as usize;
            if t < ntones {
                obs[t].push(cs[(start + k) * ntones + t]);
            }
        }
    }

    // Per-tone pilot estimate: mean of observations. Missing tones are
    // linearly extrapolated from the previous two in ascending order.
    let mut pilots = vec![Complex::new(0.0f32, 0.0); ntones];
    let mut observed = vec![false; ntones];
    for t in 0..ntones {
        if !obs[t].is_empty() {
            let n = obs[t].len() as f32;
            pilots[t] = obs[t].iter().copied().sum::<Complex<f32>>() / n;
            observed[t] = true;
        }
    }
    for t in 0..ntones {
        if !observed[t] {
            // Try `2·p[t-1] − p[t-2]` if both predecessors are observed.
            if t >= 2 && observed[t - 1] && observed[t - 2] {
                pilots[t] = pilots[t - 1] * 2.0 - pilots[t - 2];
            } else if t >= 1 && observed[t - 1] {
                // Fall back to flat extrapolation.
                pilots[t] = pilots[t - 1];
            }
            // else: stays zero — callers must ensure pattern visits enough tones.
        }
    }

    // Noise variance from the scatter of observations around the per-tone mean.
    let (total_var, count) = obs
        .iter()
        .enumerate()
        .filter(|(_, o)| !o.is_empty())
        .fold((0.0f32, 0usize), |(v, n), (t, obs_t)| {
            let mean = pilots[t];
            (
                v + obs_t.iter().map(|o| (*o - mean).norm_sqr()).sum::<f32>(),
                n + obs_t.len(),
            )
        });
    let noise_var = if count > 0 { total_var / count as f32 } else { 1.0 };

    // Regularise by median pilot power × 0.3 (prevents over-correction at low SNR).
    let mut powers: Vec<f32> = pilots.iter().map(|p| p.norm_sqr()).collect();
    powers.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median_power = powers[powers.len() / 2];
    let noise_var = noise_var.max(median_power * 0.3);

    // Wiener weights.
    let mut weights = vec![Complex::new(0.0f32, 0.0); ntones];
    for t in 0..ntones {
        let p = pilots[t];
        weights[t] = p.conj() / (p.norm_sqr() + noise_var);
    }

    // Normalise mean |w| → 1 so downstream SNR estimates remain meaningful.
    let mean_mag = weights.iter().map(|w| w.norm()).sum::<f32>() / ntones as f32;
    if mean_mag > f32::EPSILON {
        for w in weights.iter_mut() {
            *w /= mean_mag;
        }
    }

    // Apply to every symbol.
    let n_sym = cs.len() / ntones;
    for sym in 0..n_sym {
        for (t, w) in weights.iter().enumerate() {
            cs[sym * ntones + t] *= *w;
        }
    }
}
