// SPDX-License-Identifier: GPL-3.0-only
//! The FT8 and FT4 transmit waveforms against WSJT-X v3.3.0-beta6's own `gen_ft8wave` / `gen_ft4wave`
//! (tier A: a vendored fixture, a missing one is a failure).
//!
//! `tests/fixtures/gen_wave/` is made by `scripts/wave/build_fixtures.sh` from a `git archive` of the tag: the
//! tone list and the real 12 kHz waveform at 1500 Hz as int16, peak-normalised to 32767. Ours is peak-normalised
//! the same way.
//!
//! Tolerance, and why it is not zero: beta6 rewrote the generators (`lib/waveform_kernels.f90`) with a
//! polynomial Gaussian tail, a degree-9 sine and a double-precision phase. Against its own beta4 generator that
//! is a different waveform by -68 dB (FT8, max 7.1e-4) and -83 dB (FT4, max 1.4e-4); this crate keeps the exact
//! `erf` / `sin` form, which matched the beta4 generators to -105 dB (FT8) and -103 dB (FT4). Measured here
//! against beta6: FT8 -68 dB / max 7.0e-4, FT4 -82 dB / max 1.6e-4 (the int16 quantisation of the fixture, 1.5e-5
//! of peak, is below both). The bounds leave about 2x headroom; a structural difference (the FT4 ramp symbols,
//! the pulse, the phase start) is 10-40 dB larger.

#![cfg(all(feature = "ft8", feature = "ft4"))]

use mfsk_core::engine::tx::synthesize;

fn path(name: &str) -> String {
    format!(
        "{}/tests/fixtures/gen_wave/{name}",
        env!("CARGO_MANIFEST_DIR")
    )
}

fn tones(name: &str) -> Vec<u8> {
    std::fs::read_to_string(path(name))
        .expect("tests/fixtures/gen_wave is vendored")
        .split_whitespace()
        .map(|t| t.parse().unwrap())
        .collect()
}

fn wave(name: &str) -> Vec<f32> {
    let b = std::fs::read(path(name)).expect("tests/fixtures/gen_wave is vendored");
    let (pairs, rest) = b.as_chunks::<2>();
    assert!(rest.is_empty());
    pairs
        .iter()
        .map(|&c| f32::from(i16::from_le_bytes(c)) / 32767.0)
        .collect()
}

/// (max |difference|, relative rms in dB) of `ours` against `up`, both peak-normalised.
fn compare(ours: &[f32], up: &[f32]) -> (f32, f32) {
    assert_eq!(ours.len(), up.len());
    let peak = ours.iter().fold(0f32, |a, &b| a.max(b.abs()));
    let (mut max, mut e2, mut s2) = (0f32, 0f64, 0f64);
    for (&o, &u) in ours.iter().zip(up) {
        let d = o / peak - u;
        max = max.max(d.abs());
        e2 += f64::from(d) * f64::from(d);
        s2 += f64::from(u) * f64::from(u);
    }
    (max, (10.0 * (e2 / s2).log10()) as f32)
}

#[test]
fn fixture_is_the_one_documented() {
    let tag = std::fs::read_to_string(path("TAG")).expect("TAG is vendored");
    assert!(
        tag.starts_with("v3.3.0-beta6 7255454d37c4c834dc88869bce810e39df91582b"),
        "{tag}"
    );
}

#[test]
fn ft8_waveform_matches_gen_ft8wave() {
    let t = tones("ft8.tones");
    let ours = synthesize::<mfsk_core::ft8::Ft8>(&t, 12_000, 1500.0, 1.0);
    let (max, db) = compare(&ours, &wave("ft8.i16"));
    println!("FT8 max {max:.2e}, rel rms {db:.1} dB");
    assert!(max < 1.5e-3, "max {max:.2e}");
    assert!(db < -60.0, "rel rms {db:.1} dB");
}

/// FT4's frame is `gen_ft4wave`'s `(103 + 2) * 576` samples: a ramp symbol each side, whole-symbol ramps.
#[test]
fn ft4_waveform_matches_gen_ft4wave() {
    let t = tones("ft4.tones");
    let ours = synthesize::<mfsk_core::ft4::Ft4>(&t, 12_000, 1500.0, 1.0);
    assert_eq!(ours.len(), 105 * 576);
    let (max, db) = compare(&ours, &wave("ft4.i16"));
    println!("FT4 max {max:.2e}, rel rms {db:.1} dB");
    assert!(max < 3.5e-4, "max {max:.2e}");
    assert!(db < -72.0, "rel rms {db:.1} dB");
}
