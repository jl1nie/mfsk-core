//! Gaussian Frequency-Shift-Keying (GFSK) waveform synthesis.
//!
//! Protocol-agnostic: given an FSK tone sequence, produces phase-continuous
//! PCM with Gaussian-shaped frequency transitions. FT8/FT4/FT2/FST4 all use
//! this shape and differ only in samples-per-symbol, BT product and
//! modulation index (`hmod`). Tone *spacing* is implicitly
//! `sample_rate · hmod / samples_per_symbol` — no separate parameter needed.
//!
//! Ported from WSJT-X `gen_ft8wave.f90` + `gfsk_pulse.f90`.

use std::f32::consts::PI;

/// Runtime parameters of a GFSK waveform generator.
#[derive(Clone, Copy, Debug)]
pub struct GfskCfg {
    /// PCM sample rate in Hz (12 000 for WSJT).
    pub sample_rate: f32,
    /// Samples per modulation symbol (FT8 = 1920, FT4 = 576, …).
    pub samples_per_symbol: usize,
    /// Bandwidth-time product. FT8/FT4 use 2.0 (fairly wide Gaussian);
    /// FST4 uses 1.0.
    pub bt: f32,
    /// Modulation index. 1.0 for FT8 (orthogonal tones at `1/T` spacing).
    pub hmod: f32,
    /// Cosine ramp length at start/end of the waveform, in samples.
    /// `0` disables ramping. FT8 uses `samples_per_symbol / 8`.
    pub ramp_samples: usize,
}

/// Gaussian pulse matching WSJT-X `gfsk_pulse` (3-symbol wide).
#[inline]
fn gfsk_pulse(bt: f32, t: f32) -> f32 {
    let c = PI * (2.0_f32 / 2.0_f32.ln()).sqrt();
    0.5 * (erf(c * bt * (t + 0.5)) - erf(c * bt * (t - 0.5)))
}

/// Approximate erf(x) — Abramowitz & Stegun 7.1.26, accurate to ~1e-5.
#[inline]
fn erf(x: f32) -> f32 {
    let sign = if x >= 0.0 { 1.0 } else { -1.0 };
    let x = x.abs();
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    sign * (1.0 - poly * (-x * x).exp())
}

/// Synthesise a PCM waveform from an FSK tone sequence.
///
/// - `tones[j]` is the integer tone index for symbol `j` (0..NTONES).
/// - `f0_hz` is the carrier (tone-0) frequency.
/// - `amplitude` is the peak of the returned f32 signal (typically 1.0).
///
/// Output length is `tones.len() · cfg.samples_per_symbol`. The pipeline is:
/// build a per-sample phase-rate array `dphi` via a 3-symbol Gaussian pulse
/// shape, add the carrier offset, integrate → phase, take `sin`. Finally, a
/// half-cosine envelope of length `cfg.ramp_samples` smooths both ends.
pub fn synth_f32(tones: &[u8], f0_hz: f32, amplitude: f32, cfg: &GfskCfg) -> Vec<f32> {
    let nsps = cfg.samples_per_symbol;
    let nsym = tones.len();
    let twopi = 2.0 * PI;
    let dt = 1.0 / cfg.sample_rate;

    let pulse_len = 3 * nsps;
    let pulse: Vec<f32> = (0..pulse_len)
        .map(|i| {
            let tt = (i as f32 - 1.5 * nsps as f32) / nsps as f32;
            gfsk_pulse(cfg.bt, tt)
        })
        .collect();

    let total = (nsym + 2) * nsps;
    let mut dphi = vec![0.0f32; total];
    let dphi_peak = twopi * cfg.hmod / nsps as f32;

    for (j, &tone) in tones.iter().enumerate() {
        let ib = j * nsps;
        for i in 0..pulse_len {
            if ib + i < total {
                dphi[ib + i] += dphi_peak * pulse[i] * tone as f32;
            }
        }
    }

    // Dummy symbols (ramp-in / ramp-out for smooth pulse overlap)
    for i in 0..(2 * nsps).min(total) {
        dphi[i] += dphi_peak * tones[0] as f32 * pulse[nsps + i];
    }
    let ofs = nsym * nsps;
    for i in 0..(2 * nsps) {
        if ofs + i < total {
            dphi[ofs + i] += dphi_peak * tones[nsym - 1] as f32 * pulse[i];
        }
    }

    // Carrier
    for d in dphi.iter_mut() {
        *d += twopi * f0_hz * dt;
    }

    let nwave = nsym * nsps;
    let mut wave = vec![0.0f32; nwave];
    let mut phi = 0.0f32;
    for k in 0..nwave {
        wave[k] = amplitude * phi.sin();
        phi += dphi[nsps + k];
        if phi > twopi {
            phi -= twopi;
        }
    }

    // Half-cosine envelope on each end
    let nramp = cfg.ramp_samples.min(nwave / 2);
    if nramp > 0 {
        for i in 0..nramp {
            let env = (1.0 - (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
            wave[i] *= env;
        }
        let k1 = nwave - nramp;
        for i in 0..nramp {
            let env = (1.0 + (twopi * i as f32 / (2.0 * nramp as f32)).cos()) / 2.0;
            wave[k1 + i] *= env;
        }
    }

    wave
}

/// i16 variant: peak value of the returned PCM equals `amplitude_i16`.
#[inline]
pub fn synth_i16(tones: &[u8], f0_hz: f32, amplitude_i16: i16, cfg: &GfskCfg) -> Vec<i16> {
    synth_f32(tones, f0_hz, 1.0, cfg)
        .iter()
        .map(|&s| (s * amplitude_i16 as f32) as i16)
        .collect()
}
