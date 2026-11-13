//! Plain continuous-phase FSK synthesis: the transmit waveform of WSPR,
//! JT9, JT65 and Q65.
//!
//! This is WSJT-X's positive-`toneSpacing` path, generated sample by
//! sample in `Modulator::modulate` (`m_phi += m_dphi; sample =
//! m_amp·sin(m_phi)`). It applies no symbol shaping; see
//! [`super::envelope`] for why these four modes get none and why they
//! still get an envelope ramp. FT8, FT4 and FST4 use the pre-computed,
//! filtered waveform instead ([`super::gfsk`]).
//!
//! Each of the four `tx.rs` files used to carry its own copy of this
//! loop. The body below is that loop exactly, so the output is
//! bit-identical to what they produced.

use alloc::vec;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::envelope;
use super::nco::Nco;

/// Samples per symbol at `sample_rate`, from a symbol duration in
/// seconds (`ModulationParams::SYMBOL_DT`). The protocols' own `NSPS`
/// constants are for 12 kHz; this scales them to any rate.
#[inline]
pub fn nsps(sample_rate: u32, symbol_dt: f32) -> usize {
    (sample_rate as f32 * symbol_dt).round() as usize
}

/// Walk the accumulated phase of a continuous-phase FSK transmission,
/// calling `f(sample_index, &nco)` for every sample; `nco.phase()` is in `[0, 2π)`.
///
/// Symbol `k` runs at `f0_hz + tones[k] · tone_spacing_hz` for `nsps` samples
/// and the phase carries across symbol boundaries. As WSJT-X's `Modulator::modulate`
/// does, the phase is advanced **before** the sample is taken (`m_phi += m_dphi; sample
/// = m_amp·sin(m_phi)`), so sample `n` is the phase after `n + 1` steps. The phase is a
/// [`Nco`]: a `u64` of turns, exact, with no `f64` per sample (the Modulator's `m_phi` is a
/// `double`, which an LX7 cannot afford and an `f32` sum cannot match: 0.04 rad of drift over
/// WSPR's 110 s). This is the one place that arithmetic is written: [`synth_f32_into`] takes
/// `sin` of it for the transmit waveform, and `engine::dsp::subtract` takes `cos` and `sin` of it to
/// rebuild the same waveform on the receive side, so the two cannot drift apart (#425).
#[inline]
pub(crate) fn for_each_phase(
    tones: &[u8],
    nsps: usize,
    f0_hz: f32,
    tone_spacing_hz: f32,
    sample_rate_hz: f32,
    mut f: impl FnMut(usize, &Nco),
) {
    let mut nco = Nco::new(f0_hz, sample_rate_hz);
    let mut idx = 0usize;
    for &sym in tones {
        nco.set_carrier(
            f64::from(f0_hz) + f64::from(sym) * f64::from(tone_spacing_hz),
            sample_rate_hz,
        );
        for _ in 0..nsps {
            nco.step(0.0);
            f(idx, &nco);
            idx += 1;
        }
    }
}

/// Synthesise `tones` into `out` as continuous-phase FSK, then apply
/// the transmit-envelope ramp ([`envelope::apply_ramp`]).
///
/// Symbol `k` is a sinusoid at `f0_hz + tones[k] · tone_spacing_hz`
/// lasting `nsps` samples. Phase carries across symbol boundaries.
/// No allocation.
///
/// # Panics
///
/// Panics if `out.len() != nsps · tones.len()`.
pub fn synth_f32_into(
    out: &mut [f32],
    tones: &[u8],
    nsps: usize,
    f0_hz: f32,
    tone_spacing_hz: f32,
    sample_rate: u32,
    amplitude: f32,
) {
    assert_eq!(
        out.len(),
        nsps * tones.len(),
        "cpfsk::synth_f32_into: out.len() must equal nsps * tones.len()"
    );
    for_each_phase(
        tones,
        nsps,
        f0_hz,
        tone_spacing_hz,
        sample_rate as f32,
        |idx, nco| out[idx] = amplitude * nco.sin(),
    );

    // Transmit-envelope ramp (issue #259). Without it the burst starts
    // and ends on a step discontinuity, a broadband click at both edges.
    envelope::apply_ramp(out, envelope::ramp_samples(sample_rate, nsps));
}

/// Allocating form of [`synth_f32_into`].
pub fn synth_f32(
    tones: &[u8],
    nsps: usize,
    f0_hz: f32,
    tone_spacing_hz: f32,
    sample_rate: u32,
    amplitude: f32,
) -> Vec<f32> {
    let mut out = vec![0.0f32; nsps * tones.len()];
    synth_f32_into(
        &mut out,
        tones,
        nsps,
        f0_hz,
        tone_spacing_hz,
        sample_rate,
        amplitude,
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::TAU;

    /// The waveform is `Modulator::modulate`'s: an `f64` phase, advanced before the sample is taken,
    /// `sin` of it. Checked at WSPR's length, where an `f32` sum had drifted by 0.04 rad (-35 dB).
    #[test]
    fn synth_matches_the_modulator_in_f64() {
        fn modulator(
            tones: &[u8],
            nsps: usize,
            f0: f32,
            spacing: f32,
            fs: u32,
            amp: f32,
        ) -> Vec<f32> {
            let tau = 2.0 * core::f64::consts::PI;
            let mut out = vec![0.0f32; nsps * tones.len()];
            let mut phi = 0.0f64;
            let mut idx = 0usize;
            for &sym in tones {
                let dphi =
                    tau / f64::from(fs) * (f64::from(f0) + f64::from(sym) * f64::from(spacing));
                for _ in 0..nsps {
                    phi += dphi;
                    if phi > tau {
                        phi -= tau;
                    }
                    out[idx] = amp * phi.sin() as f32;
                    idx += 1;
                }
            }
            envelope::apply_ramp(&mut out, envelope::ramp_samples(fs, nsps));
            out
        }
        let tones: Vec<u8> = (0..162).map(|k| ((k * 7 + 3) % 4) as u8).collect();
        for (nsps, f0, spacing, fs) in [
            (8192usize, 1500.0f32, 1.4648f32, 12_000u32),
            (6912, 1200.3, 1.7361, 12_000),
            (4460, 987.65, 2.6917, 12_000),
            (1024, 2900.0, 5.859, 48_000),
        ] {
            let new = synth_f32(&tones, nsps, f0, spacing, fs, 0.8);
            let old = modulator(&tones, nsps, f0, spacing, fs, 0.8);
            let worst = new
                .iter()
                .zip(&old)
                .fold(0f32, |m, (a, b)| m.max((a - b).abs()));
            assert!(worst < 2e-4, "nsps {nsps}, f0 {f0}: worst {worst:.2e}");
        }
    }

    #[test]
    fn length_and_ramped_edges() {
        let tones = [0u8, 1, 2, 3];
        let out = synth_f32(&tones, 1000, 1500.0, 1.5, 12_000, 0.5);
        assert_eq!(out.len(), 4000);
        // The ramp starts at zero envelope.
        assert_eq!(out[0], 0.0);
        assert!(out.iter().all(|s| s.abs() <= 0.5));
    }

    #[test]
    fn phase_is_continuous_across_symbols() {
        // At a symbol boundary the sample-to-sample step stays below the
        // largest per-sample step either tone can take: no phase jump.
        let nsps = 1000;
        let out = synth_f32(&[0u8, 7], nsps, 1000.0, 50.0, 12_000, 1.0);
        let max_dphi = TAU * 1350.0 / 12_000.0;
        let step = (out[nsps] - out[nsps - 1]).abs();
        assert!(step <= max_dphi, "step {step} > {max_dphi}");
    }
}
