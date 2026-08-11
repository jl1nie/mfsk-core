// SPDX-License-Identifier: GPL-3.0-or-later
//! Self-tests for `tests/common/` — the shared test scaffolding.
//!
//! These live here rather than beside the code they cover, which is
//! unusual and deliberate.
//!
//! `tests/common/` is a *module*, not a crate: every integration test
//! that needs a WAV loader, a channel model or the golden helpers
//! writes `mod common;`, and 43 of them do. A `#[cfg(test)] mod tests`
//! inside `common/channel.rs` is therefore compiled and executed once
//! per including binary — 23 tests × 43 binaries ≈ 990 redundant runs
//! per `cargo test`, which is where ~750 of the suite's reported
//! "passed" count came from. Wall-clock cost is negligible; the cost
//! is that a four-digit pass count implies far more coverage than
//! exists.
//!
//! Collecting them into one binary makes the number honest: these 23
//! tests run once. `common`'s helpers stay where they are; only their
//! tests moved, and the three private helpers they exercise
//! (`corpus::require_enabled`, `air_channel::{analytic_signal, bpf}`)
//! were widened to `pub` — `common` is test-only scaffolding, so that
//! costs nothing.

// Deliberately **no** `#![cfg(...)]` gate on this binary.
//
// A crate-level `#![cfg(any(feature = "fft-rustfft", feature =
// "fft-extern"))]` was tried and removed: under
// `cargo test -p mfsk-core --no-default-features --test
// common_selftest` it silently reduced this file to "0 passed; ok" —
// the exact green-lie this suite's restructure exists to eliminate,
// reintroduced by the very commit meant to make the pass count
// honest.
//
// Ungated, an unsupported feature set fails to *compile* instead,
// which is loud. That is not a regression: `common/channel.rs`
// imports `mfsk_core::uvpacket::Mode` at file scope, so every one of
// the 43 binaries that write `mod common;` already requires the
// `uvpacket` feature — verified on an untouched one
// (`ft8_streaming_decode` fails the same way without it). This binary
// simply shares its siblings' constraint rather than hiding from it.

#[allow(dead_code)]
mod common;

mod channel_selftest {
    #[allow(unused_imports)]
    use crate::common::channel::*;

    /// AWGN channel with σ = 0 must be a no-op (modulo the f32
    /// representation of `0.0 × x = 0.0`).
    #[test]
    fn awgn_with_zero_sigma_is_identity() {
        let mut c = AwgnChannel::new(0.0, 42);
        let mut buf = vec![0.5_f32; 100];
        let original = buf.clone();
        c.apply(&mut buf);
        assert_eq!(buf, original);
    }

    /// AWGN with a non-zero σ produces non-zero noise — and seeded
    /// runs are reproducible.
    #[test]
    fn awgn_seeded_is_reproducible() {
        let mut a = AwgnChannel::new(0.5, 123);
        let mut b = AwgnChannel::new(0.5, 123);
        let mut buf_a = vec![0.0_f32; 200];
        let mut buf_b = vec![0.0_f32; 200];
        a.apply(&mut buf_a);
        b.apply(&mut buf_b);
        assert_eq!(buf_a, buf_b);
        // …and at least 90 % of samples are non-zero.
        let nonzero = buf_a.iter().filter(|&&s| s != 0.0).count();
        assert!(nonzero > 180);
    }

    /// At a high Eb/N0 (= 100 dB) the σ for every mode should be
    /// vanishingly small; at 0 dB it should be a meaningful fraction
    /// of the signal envelope.
    /// Higher-rate modes spend less channel energy per info bit, so
    /// at a fixed Eb/N0_info the channel-domain noise must be lower
    /// for them — i.e. σ decreases with rate.
    /// σ scales as `√P` (since variance scales as P at fixed Eb/N0).
    /// Rayleigh envelope statistics: over a long buffer, the
    /// magnitude has E[|h|²] ≈ 1 by construction.
    #[test]
    fn rayleigh_envelope_has_unit_mean_square() {
        let mut chan = RayleighFlatChannel::new(5.0, 0.0, 0xABC1_2345);
        let mut audio = vec![1.0_f32; 12_000]; // 1 sec at fs=12kHz
        chan.apply(&mut audio);
        let mean_sq: f32 = audio.iter().map(|s| s * s).sum::<f32>() / audio.len() as f32;
        // Allow generous tolerance because 1 sec at 5 Hz Doppler
        // ≈ 5 fading cycles → high finite-sample variance.
        assert!(
            (0.5..2.0).contains(&mean_sq),
            "Rayleigh mean-square {mean_sq} far off the expected 1.0",
        );
    }

    /// Rayleigh + AWGN at huge Doppler approaches AWGN-only stats
    /// in the limit (envelope decorrelates fast, magnitude
    /// distribution converges to Rayleigh independent of past).
    /// Sanity smoke test that nothing panics or NaNs.
    #[test]
    fn rayleigh_apply_does_not_nan() {
        for &fd in &[0.5_f32, 1.0, 5.0, 20.0] {
            let mut chan = RayleighFlatChannel::new(fd, 0.1, 0xFEED + fd as u64);
            let mut audio = vec![0.5_f32; 4096];
            chan.apply(&mut audio);
            assert!(audio.iter().all(|s| s.is_finite()));
        }
    }
}

mod air_channel_selftest {
    #[allow(unused_imports)]
    use crate::common::air_channel::*;
    use std::f32::consts::PI;

    #[test]
    fn analytic_signal_real_part_matches_input() {
        let n = 512;
        let f = 1500.0 / SAMPLE_RATE_HZ;
        let x: Vec<f32> = (0..n).map(|i| (2.0 * PI * f * i as f32).cos()).collect();
        let anal = analytic_signal(&x);
        // Skip edges (FFT-Hilbert has wrap-around there).
        for (i, (a, xi)) in anal[64..n - 64].iter().zip(&x[64..n - 64]).enumerate() {
            assert!((a.re - xi).abs() < 0.05, "i={i}: re={} vs x={xi}", a.re,);
        }
    }

    #[test]
    fn analytic_signal_quadrature_is_sine_for_cosine_input() {
        let n = 512;
        let f = 1500.0 / SAMPLE_RATE_HZ;
        let x: Vec<f32> = (0..n).map(|i| (2.0 * PI * f * i as f32).cos()).collect();
        let anal = analytic_signal(&x);
        for (offset, a) in anal[64..n - 64].iter().enumerate() {
            let i = offset + 64;
            let want = (2.0 * PI * f * i as f32).sin();
            assert!(
                (a.im - want).abs() < 0.05,
                "i={i}: im={} vs sin={want}",
                a.im,
            );
        }
    }

    #[test]
    fn bpf_passes_in_band_blocks_out_of_band() {
        let n = 4096;
        let fs = SAMPLE_RATE_HZ;
        // 1500 Hz tone (in the SSB passband) and 100 Hz tone (out).
        let in_band: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let out_band: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 100.0 * i as f32 / fs).cos())
            .collect();
        let in_filtered = bpf(&in_band, 300.0, 2700.0, 100.0, fs);
        let out_filtered = bpf(&out_band, 300.0, 2700.0, 100.0, fs);
        let pwr =
            |buf: &[f32]| buf[200..n - 200].iter().map(|s| s * s).sum::<f32>() / (n - 400) as f32;
        let in_pwr = pwr(&in_filtered);
        let out_pwr = pwr(&out_filtered);
        assert!(in_pwr > 0.4, "in-band power {in_pwr} too low");
        assert!(out_pwr < 0.05, "out-of-band power {out_pwr} too high");
    }

    #[test]
    fn ssb_channel_off_preserves_passband_signal() {
        let n = 2048;
        let fs = SAMPLE_RATE_HZ;
        let original: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let mut audio = original.clone();
        let chan = SsbChannel {
            bpf_hz: (300.0, 2700.0),
            bpf_transition_hz: 100.0,
            clarifier_offset_hz: 0.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel::off(),
            seed: 1,
        };
        chan.apply(&mut audio);
        let pwr =
            |buf: &[f32]| buf[300..n - 300].iter().map(|s| s * s).sum::<f32>() / (n - 600) as f32;
        let r = pwr(&audio) / pwr(&original);
        assert!(
            (r - 1.0).abs() < 0.15,
            "SSB-off should preserve power within 15%, got ratio {r}",
        );
    }

    #[test]
    fn ssb_channel_with_phase_walk_introduces_phase_drift() {
        // A 1500 Hz cosine through SSB with walk should accumulate
        // measurable instantaneous phase deviation by burst-end.
        let n = 12_000; // 1 sec
        let fs = SAMPLE_RATE_HZ;
        let original: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let mut audio = original.clone();
        let chan = SsbChannel {
            bpf_hz: (300.0, 2700.0),
            bpf_transition_hz: 100.0,
            clarifier_offset_hz: 0.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel {
                lo_phase_walk_rad_per_sqrt_s: 2.0,
                ..PhaseFadingModel::off()
            },
            seed: 7,
        };
        chan.apply(&mut audio);
        // Compare the late-burst correlation against the early-burst
        // correlation. Phase walk should de-correlate them.
        let early: f32 = audio[1000..2000]
            .iter()
            .zip(&original[1000..2000])
            .map(|(a, b)| a * b)
            .sum::<f32>()
            / 1000.0;
        let late: f32 = audio[10_000..11_000]
            .iter()
            .zip(&original[10_000..11_000])
            .map(|(a, b)| a * b)
            .sum::<f32>()
            / 1000.0;
        // Early correlation should be close to 0.5 (cos² mean), late
        // should be reduced (or sign-reversed) by accumulated phase
        // drift.
        assert!(
            (early - late).abs() > 0.1 || late.abs() < 0.3,
            "phase walk produced no measurable drift: early={early} late={late}",
        );
    }

    #[test]
    fn ssb_default_channel_does_not_nan() {
        let mut audio = vec![0.5_f32; 12_000];
        SsbChannel::default().apply(&mut audio);
        assert!(audio.iter().all(|s| s.is_finite()));
    }

    #[test]
    fn fm_default_channel_does_not_nan() {
        let mut audio = vec![0.5_f32; 12_000];
        FmChannel::default().apply(&mut audio);
        assert!(audio.iter().all(|s| s.is_finite()));
    }

    #[test]
    fn rician_k_infinity_gives_unit_envelope() {
        // K = +∞ + Doppler 5 Hz: should still be near-unit envelope
        // (since LOS component dominates).
        let n = 12_000;
        let fs = SAMPLE_RATE_HZ;
        let original: Vec<f32> = (0..n)
            .map(|i| (2.0 * PI * 1500.0 * i as f32 / fs).cos())
            .collect();
        let mut audio = original.clone();
        let chan = SsbChannel {
            bpf_hz: (300.0, 2700.0),
            bpf_transition_hz: 100.0,
            clarifier_offset_hz: 0.0,
            awgn_sigma: 0.0,
            phase_fading: PhaseFadingModel {
                doppler_hz: 5.0,
                rician_k_db: f32::INFINITY,
                ..PhaseFadingModel::off()
            },
            seed: 11,
        };
        chan.apply(&mut audio);
        let r = audio[400..n - 400].iter().map(|s| s * s).sum::<f32>()
            / original[400..n - 400].iter().map(|s| s * s).sum::<f32>();
        assert!((r - 1.0).abs() < 0.2, "unit-K envelope ratio {r}");
    }
}

mod corpus_selftest {
    #[allow(unused_imports)]
    use crate::common::corpus::*;

    #[test]
    fn vendored_golden_assets_are_present() {
        // These are committed to the repo, so their absence is a real
        // failure on any machine, not a skip.
        for rel in [
            "ft4/000000_000002.wav",
            "fst4/210115_0058.wav",
            "wspr/150426_0918.wav",
            "msk144/181211_120500.wav",
            "msk144/181211_120800.wav",
            "jt65/jt65a_5sig_m18.wav",
        ] {
            assert!(
                golden_dir().join(rel).exists(),
                "vendored golden asset {rel} is missing from {}",
                golden_dir().display()
            );
        }
    }

    #[test]
    fn missing_asset_is_none_without_the_env_var() {
        // Guard the developer-machine half of the contract. (The
        // panicking half is exercised by CI itself running the whole
        // tier-B suite with MFSK_REQUIRE_CORPUS=1.)
        if require_enabled() {
            return;
        }
        assert!(golden_path("nope/does-not-exist.wav").is_none());
    }
}

mod golden_selftest {
    #[allow(unused_imports)]
    use crate::common::golden::*;

    static EXPECTED: &[GoldenEntry] = &[
        GoldenEntry {
            msg: "CQ K1ABC FN42",
            freq_hz: Some(1000.0),
            dt_sec: Some(0.0),
            snr_db: Some(-10.0),
        },
        GoldenEntry {
            msg: "K1ABC W9XYZ EN37",
            freq_hz: None,
            dt_sec: None,
            snr_db: None,
        },
    ];

    fn set(min_hits: usize, max_extra: usize) -> GoldenSet {
        GoldenSet {
            name: "test",
            expected: EXPECTED,
            min_hits,
            max_extra,
        }
    }

    fn v(msg: &str, freq_hz: f32, snr_db: Option<f32>) -> DecodeView {
        DecodeView {
            msg: msg.to_string(),
            freq_hz,
            dt_sec: 0.0,
            snr_db,
        }
    }

    #[test]
    fn passes_when_recall_and_precision_both_hold() {
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(-11.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
        ];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    /// The regression that motivated this module: full recall, but the
    /// decoder also invented something.
    #[test]
    #[should_panic(expected = "outside the golden set")]
    fn full_recall_does_not_excuse_a_phantom() {
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(-10.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
            v("UZC/7D0DKY 17", 1501.0, None),
        ];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    #[test]
    #[should_panic(expected = "recall regressed")]
    fn missing_entry_fails_recall() {
        let d = [v("CQ K1ABC FN42", 1000.0, Some(-10.0))];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    #[test]
    #[should_panic(expected = "reported SNR is out of tolerance")]
    fn snr_outside_tolerance_fails() {
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(22.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
        ];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    /// A frequency far from the golden entry must not count as a hit,
    /// or the freq column is decorative.
    #[test]
    #[should_panic(expected = "recall regressed")]
    fn wrong_frequency_is_not_a_hit() {
        let d = [
            v("CQ K1ABC FN42", 1400.0, Some(-10.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
        ];
        assert_golden(&d, &set(2, 9), Tolerances::default(), |x| x.clone());
    }

    #[test]
    fn documented_budgets_are_honoured() {
        // A tracked gap (min_hits below len) and a tracked phantom
        // budget both pass without weakening the other assertion.
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(-10.0)),
            v("SOMETHING ELSE", 1600.0, None),
        ];
        assert_golden(&d, &set(1, 1), Tolerances::default(), |x| x.clone());
    }
}

/// uvpacket's Eb/N0 helper lives in its own gated module (it is built
/// on uvpacket's π/4-DQPSK PHY constants, not on anything shared), so
/// its self-tests are gated the same way.
#[cfg(feature = "uvpacket")]
mod uvpacket_channel_selftest {
    #[allow(unused_imports)]
    use crate::common::channel::*;
    use crate::common::uvpacket_channel::*;
    use mfsk_core::uvpacket::Mode;

    #[test]
    fn sigma_decreases_with_eb_n0() {
        // Use a representative 4-FSK-era signal power (P = 0.5) so
        // the test bounds are absolute rather than measurement-tied.
        let p = 0.5;
        for mode in [
            Mode::Robust,
            Mode::Standard,
            Mode::UltraRobust,
            Mode::Express,
        ] {
            let sigma_clean = awgn_sigma_for_eb_n0_info(mode, 100.0, p);
            let sigma_zero = awgn_sigma_for_eb_n0_info(mode, 0.0, p);
            assert!(sigma_clean < 1e-3, "{mode:?}: clean σ {sigma_clean}");
            assert!(sigma_zero > 0.5, "{mode:?}: 0-dB σ {sigma_zero}");
            assert!(
                sigma_clean < sigma_zero,
                "{mode:?}: σ should decrease as Eb/N0 grows",
            );
        }
    }

    #[test]
    fn sigma_decreases_with_rate() {
        // UltraRobust shares Robust's FEC rate (only the symbol
        // rate differs), so σ at fixed Eb/N0 is identical between
        // them — exclude UltraRobust from this strict-monotonic
        // test and check only the FEC-rate-distinct modes.
        let eb_n0 = 0.0;
        let p = 0.5;
        let sigmas: Vec<f32> = [Mode::Robust, Mode::Standard, Mode::Express]
            .iter()
            .map(|&m| awgn_sigma_for_eb_n0_info(m, eb_n0, p))
            .collect();
        for w in sigmas.windows(2) {
            assert!(
                w[0] > w[1],
                "expected σ to decrease across rates: {sigmas:?}",
            );
        }
    }

    #[test]
    fn sigma_scales_with_signal_power() {
        let s1 = awgn_sigma_for_eb_n0_info(Mode::Robust, 0.0, 1.0);
        let s4 = awgn_sigma_for_eb_n0_info(Mode::Robust, 0.0, 4.0);
        // 4× power → 2× σ (within float epsilon).
        assert!((s4 / s1 - 2.0).abs() < 1e-3, "s1={s1}, s4={s4}");
    }
}
