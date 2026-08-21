//! Frequency response of the FST4 DDC cascades — passband flatness,
//! transition, stopband and alias rejection. Issue #307.
//!
//! Both cascades shipped with only a single-tone smoke test each
//! (`fst4::ddc`'s own `#[cfg(test)]` module), which confirms a centred
//! tone survives and says nothing about what an *off*-centre or
//! *out-of-band* one does. `wideband_cascade` in particular reached
//! real hardware with no non-test caller at all until #330, and
//! `fst4_wideband_max_cand` then attributed an ~9/100 recall gap at
//! −27 dB to the wideband front end. This is the missing
//! characterisation — and it **exonerated the filter**, which sent that
//! investigation to the real cause (`fst4_rung_major_recall_gap`: the
//! omitted `llrd` stage, worth ~10/100 there).
//!
//! The design constants make the wideband config look tighter than the
//! sniper one, which is what prompted this:
//!
//! | | `Fs_c/2` | search half-width | transition | margin |
//! |---|---:|---:|---:|---:|
//! | sniper | ~395 Hz | 250 Hz | 150 Hz | ~70 Hz |
//! | wideband | ~1580 Hz | **1450 Hz** | 300 Hz | **−20 Hz** |
//!
//! i.e. the wideband transition band is designed to *start* at
//! ~1430 Hz, inside the ±1450 Hz the search covers, which looks like a
//! defect on paper. Measured, it is not: the wideband response is
//! −0.23 dB at its band edge while the *sniper* — which has positive
//! margin on paper — droops −2.87 dB at its own. The paper margin is
//! the wrong quantity; only the measured curve settles it, which is
//! the point of this test existing.
//!
//! Method: drive a real 12 kHz tone at a known audio frequency through
//! the cascade and measure the complex baseband's RMS, normalised to a
//! dead-centre reference tone. Aliasing shows up as an out-of-band tone
//! producing output that isn't in the stopband.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use mfsk_core::engine::dsp::ddc::{DdcCascadeConfig, StreamingComplexDdc};
use mfsk_core::fst4::ddc::{grid_for, sniper_cascade, wideband_cascade};

const FS_IN: f32 = 12_000.0;
/// Long enough that the cascades' group delay (the wideband resampler
/// alone is 10 241 taps) is a small fraction of what's measured.
const SECONDS: f32 = 8.0;

fn tone(freq_hz: f32, n: usize) -> Vec<i16> {
    (0..n)
        .map(|k| {
            let p = core::f64::consts::TAU * freq_hz as f64 * k as f64 / FS_IN as f64;
            (p.sin() * 12_000.0) as i16
        })
        .collect()
}

/// Baseband RMS for a real input tone at `freq_hz`, with the cascade's
/// startup transient trimmed.
fn response(cfg: &DdcCascadeConfig, freq_hz: f32) -> f32 {
    let n = (FS_IN * SECONDS) as usize;
    let x = tone(freq_hz, n);
    let mut ddc = StreamingComplexDdc::new(cfg);
    let (mut i, mut q) = (Vec::new(), Vec::new());
    ddc.push_i16(&x, &mut i, &mut q);
    ddc.flush(&mut i, &mut q);

    // Drop the leading transient and a matching tail; the middle is
    // steady state.
    let skip = i.len() / 8;
    let end = i.len() - i.len() / 8;
    if end <= skip {
        return 0.0;
    }
    let seg = end - skip;
    let sum: f32 = (skip..end).map(|k| i[k] * i[k] + q[k] * q[k]).sum();
    (sum / seg as f32).sqrt()
}

fn db(x: f32, reference: f32) -> f32 {
    if x <= 0.0 || reference <= 0.0 {
        return -200.0;
    }
    20.0 * (x / reference).log10()
}

struct Curve {
    name: &'static str,
    center: f32,
    fs_c: f32,
    half_width: f32,
    /// (offset_hz, dB relative to centre)
    points: Vec<(f32, f32)>,
}

fn measure(
    name: &'static str,
    cfg: &DdcCascadeConfig,
    center: f32,
    fs_c: f32,
    half_width: f32,
) -> Curve {
    let reference = response(cfg, center);
    let mut offsets: Vec<f32> = Vec::new();
    // Dense through the passband edge and transition, where the
    // question is; coarse out into the stopband.
    let nyq = fs_c / 2.0;
    for frac in [0.0f32, 0.25, 0.5, 0.75, 0.9, 0.95, 1.0] {
        offsets.push(frac * half_width);
    }
    for extra in [1.02f32, 1.05, 1.10, 1.20, 1.4, 1.8, 2.5, 3.5] {
        offsets.push(extra * nyq);
    }
    offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    offsets.dedup();

    let points = offsets
        .iter()
        .filter(|o| center + *o < FS_IN / 2.0 - 100.0)
        .map(|&o| (o, db(response(cfg, center + o), reference)))
        .collect();

    Curve {
        name,
        center,
        fs_c,
        half_width,
        points,
    }
}

fn report(c: &Curve) {
    let nyq = c.fs_c / 2.0;
    eprintln!(
        "\n{} — centre {:.0} Hz, Fs_c {:.1} Hz (Nyquist {:.1}), search half-width {:.0} Hz",
        c.name, c.center, c.fs_c, nyq, c.half_width
    );
    eprintln!(
        "  {:>10}  {:>10}  {:>9}  region",
        "offset(Hz)", "audio(Hz)", "resp(dB)"
    );
    for &(o, d) in &c.points {
        let region = if o <= c.half_width {
            "passband (searched)"
        } else if o < nyq {
            "guard"
        } else {
            "stopband (must reject)"
        };
        eprintln!(
            "  {:>10.0}  {:>10.0}  {:>9.2}  {}",
            o,
            c.center + o,
            d,
            region
        );
    }
}

#[test]
fn fst4_ddc_cascades_pass_their_search_band_and_reject_beyond_nyquist() {
    let sniper_grid = grid_for(600.0);
    let wide_grid = grid_for(2900.0);

    let sniper = measure(
        "sniper_cascade",
        &sniper_cascade(1216.0),
        1216.0,
        sniper_grid.fs_c,
        250.0,
    );
    let wide = measure(
        "wideband_cascade",
        &wideband_cascade(1550.0),
        1550.0,
        wide_grid.fs_c,
        1450.0,
    );

    report(&sniper);
    report(&wide);

    for c in [&sniper, &wide] {
        let nyq = c.fs_c / 2.0;

        // Passband: everything the search actually covers must survive.
        // 3 dB is generous — it is a "is this filter broken" gate, not a
        // ripple spec — but it is exactly what a signal at the band edge
        // pays, so a violation is a real sensitivity loss for that
        // signal, not a cosmetic one.
        for &(o, d) in c.points.iter().filter(|(o, _)| *o <= c.half_width) {
            assert!(
                d > -3.0,
                "{}: passband droop at offset {o:.0} Hz ({:.0} Hz audio): {d:.2} dB — \
                 the search covers this offset, so a signal there is attenuated",
                c.name,
                c.center + o,
            );
        }

        // Stopband: content past Nyquist folds back into the baseband,
        // so it must be well down. `design_lowpass` uses a Blackman
        // window (~-74 dB), and the cascade is at least one such stage,
        // so -50 dB is a loose gate on a design that should reach -74.
        for &(o, d) in c.points.iter().filter(|(o, _)| *o >= nyq * 1.15) {
            assert!(
                d < -50.0,
                "{}: alias leakage at offset {o:.0} Hz ({:.0} Hz audio): {d:.2} dB — \
                 past Nyquist ({nyq:.0} Hz) this folds into the searched band",
                c.name,
                c.center + o,
            );
        }
    }
}
