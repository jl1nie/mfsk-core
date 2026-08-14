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

mod sharing_ratchet_selftest {
    //! A code-sharing audit (2026-08-14, prompted by a user question
    //! about whether faithful WSJT-X porting has caused excessive
    //! per-protocol divergence) found the same shape of problem this
    //! file's `golden_coverage_selftest` already fixes on the test
    //! side, one layer down: a shared mechanism gets built, 2-3
    //! protocols adopt it, and the rest silently never do. Four
    //! examples found: `GenericPipelineProtocol` (2/8 protocols),
    //! `known_filtered_on_result` (2/8), the `snr_db` trait override
    //! (3/8), `engine::scalar::Cmplx` fixed-point support (3/8).
    //!
    //! This is the ratchet for that: it doesn't require every
    //! protocol adopt every mechanism (much of the divergence is
    //! inherent — see `docs/reference/LIBRARY.md` §0.5, the source of
    //! truth this table is kept in sync with), but it fails loudly if
    //! a protocol that *currently* uses a shared mechanism silently
    //! stops — the same silent-regression shape `golden_coverage_selftest`
    //! guards against for test precision coverage.
    //!
    //! Adoption is detected by scanning source text for evidence
    //! (`impl … for`, a specific call site) rather than by runtime
    //! trait-object reflection, which Rust doesn't offer across a
    //! heterogeneous set of unrelated ZSTs. This is the same technique
    //! `golden_coverage_selftest` uses and has the same tradeoff: it
    //! can be fooled by a renamed identifier, but nothing here is
    //! trying to be adversarial, and a false pass just means this test
    //! itself needs updating alongside whatever renamed the thing.
    //!
    //! **If you land a new adoption**: update the relevant
    //! `EXPECTED_ADOPTERS` list here, `docs/reference/LIBRARY.md` §0.5,
    //! and the measured-sharing-percentage paragraphs in `README.md` /
    //! `lib.rs` (`docs/reference/LIBRARY.md` §0.5 is the one table all
    //! three of those should trace back to). This test does not fail
    //! when adoption *increases* — only when it decreases — so
    //! updating those is on the honor system, not enforced here.

    use std::fs;
    use std::path::Path;

    const SRC_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/src");

    /// Every protocol directory under `src/`, gated the same way its
    /// `pub mod` declaration in `lib.rs` is (feature name == dir name
    /// for all of these). uvpacket is excluded — it's an experimental
    /// non-WSJT-X example, not a protocol this ratchet tracks.
    ///
    /// Only consumed under `full` (its sole caller below is gated the
    /// same way) — gated identically so it isn't dead code under a
    /// narrower feature set.
    #[cfg(feature = "full")]
    fn protocol_dirs() -> Vec<&'static str> {
        let mut dirs = vec![];
        if cfg!(feature = "ft8") {
            dirs.push("ft8");
        }
        if cfg!(feature = "ft4") {
            dirs.push("ft4");
        }
        if cfg!(feature = "fst4") {
            dirs.push("fst4");
        }
        if cfg!(feature = "jt9") {
            dirs.push("jt9");
        }
        if cfg!(feature = "jt65") {
            dirs.push("jt65");
        }
        if cfg!(feature = "q65") {
            dirs.push("q65");
        }
        if cfg!(feature = "wspr") {
            dirs.push("wspr");
        }
        if cfg!(feature = "msk144") {
            dirs.push("msk144");
        }
        dirs
    }

    /// Concatenated source of every `.rs` file directly or indirectly
    /// under `src/<dir>/` (single level of subdirectories, matching
    /// this crate's actual module nesting — e.g. `ft8/decode_block/`).
    fn dir_source(dir: &str) -> String {
        fn collect(path: &Path, out: &mut String) {
            let Ok(entries) = fs::read_dir(path) else {
                return;
            };
            for entry in entries.flatten() {
                let p = entry.path();
                if p.is_dir() {
                    collect(&p, out);
                } else if p.extension().is_some_and(|e| e == "rs")
                    && let Ok(s) = fs::read_to_string(&p)
                {
                    out.push_str(&s);
                    out.push('\n');
                }
            }
        }
        let mut out = String::new();
        collect(&Path::new(SRC_DIR).join(dir), &mut out);
        out
    }

    /// Protocols currently routing decode through the shared
    /// `engine::pipeline::GenericPipelineProtocol` trait. FT8
    /// implements the request/outcome *builder* shape
    /// (`FrameDecodable`) but not this — it keeps its own bespoke
    /// decode engine (`ft8::decode_block`), tracked as issue #192.
    const EXPECTED_GENERIC_PIPELINE_ADOPTERS: &[&str] = &["ft4", "fst4"];

    /// Protocols using `engine::pipeline::known_filtered_on_result`
    /// (issue #243's fix) rather than a bespoke post-filter.
    const EXPECTED_KNOWN_FILTER_ADOPTERS: &[&str] = &["ft4", "fst4"];

    /// Protocols whose SNR is computed via the shared
    /// `GenericPipelineProtocol::snr_db` trait method (issue #255)
    /// rather than an inline formula in the protocol's own module.
    /// Note this trait itself only exists on `GenericPipelineProtocol`,
    /// so it can't be wider than `EXPECTED_GENERIC_PIPELINE_ADOPTERS`
    /// without the trait surface changing too.
    const EXPECTED_SNR_TRAIT_ADOPTERS: &[&str] = &["ft4", "fst4"];

    /// Protocols routing their own decode-scan candidate dedup through
    /// `engine::pipeline::scan_dedup_match`/`scan_dedup_match_cross`
    /// (Stage 1 of the code-sharing audit) rather than a hand-rolled
    /// `seen.iter().any(...)` predicate. FT8 (spread across multiple
    /// inline sites in a much larger bespoke engine) and MSK144 (a
    /// structurally different single-slot state machine, not a
    /// `Vec`-accumulating scan) are deliberately not adopters here —
    /// see the plan file for why each was out of scope for this pass.
    const EXPECTED_SCAN_DEDUP_ADOPTERS: &[&str] = &["jt9", "jt65", "wspr", "q65"];

    /// Protocols building their coarse-search spectrogram via the
    /// shared `engine::spectrogram::Spectrogram` (found in a bottom-up
    /// sweep after the top-down `BpPooledFec` blocker closed off
    /// Stage 2/3 — see the plan file) rather than their own copy. WSPR
    /// is a deliberate non-adopter: it uses the fixed-point-capable
    /// `engine::fft` backend abstraction (not raw `rustfft`, which
    /// this shared implementation hardcodes) and normalises scores
    /// against a fitted per-bin baseline rather than a flat noise
    /// floor — a real algorithmic difference, not just a naming one.
    const EXPECTED_SPECTROGRAM_ADOPTERS: &[&str] = &["jt9", "jt65", "q65"];

    fn assert_no_regression(
        mechanism: &str,
        expected_adopters: &[&str],
        detect: impl Fn(&str) -> bool,
    ) {
        let mut regressed: Vec<&str> = Vec::new();
        for &dir in expected_adopters {
            let src = dir_source(dir);
            if !detect(&src) {
                regressed.push(dir);
            }
        }
        assert!(
            regressed.is_empty(),
            "{mechanism}: previously-adopting protocol(s) no longer show adoption \
             evidence in their source: {regressed:?}. If this is intentional (the \
             protocol moved off the shared mechanism for a documented reason), \
             remove it from this test's expected-adopters list and update \
             `docs/reference/LIBRARY.md` §0.5 to match. If it's accidental, this \
             is exactly the silent-regression shape this test exists to catch."
        );
    }

    #[test]
    fn generic_pipeline_adoption_does_not_regress() {
        assert_no_regression(
            "GenericPipelineProtocol",
            EXPECTED_GENERIC_PIPELINE_ADOPTERS,
            |src| src.contains("GenericPipelineProtocol for"),
        );
    }

    #[test]
    fn known_filter_adoption_does_not_regress() {
        assert_no_regression(
            "known_filtered_on_result",
            EXPECTED_KNOWN_FILTER_ADOPTERS,
            |src| src.contains("known_filtered_on_result"),
        );
    }

    #[test]
    fn snr_trait_adoption_does_not_regress() {
        assert_no_regression(
            "snr_db trait override",
            EXPECTED_SNR_TRAIT_ADOPTERS,
            |src| src.contains("fn snr_db(ctx"),
        );
    }

    #[test]
    fn scan_dedup_adoption_does_not_regress() {
        assert_no_regression("scan_dedup_match", EXPECTED_SCAN_DEDUP_ADOPTERS, |src| {
            src.contains("scan_dedup_match")
        });
    }

    #[test]
    fn spectrogram_adoption_does_not_regress() {
        assert_no_regression(
            "engine::spectrogram::Spectrogram",
            EXPECTED_SPECTROGRAM_ADOPTERS,
            |src| src.contains("engine::spectrogram") || src.contains("spectrogram::Spectrogram"),
        );
    }

    /// Sanity check on the scan itself, mirroring
    /// `golden_coverage_selftest`'s own guard: if `protocol_dirs()`
    /// ever returns nothing (e.g. run with `--no-default-features`),
    /// every check above trivially "passes" by having nothing to
    /// check — which is not evidence of anything. This crate's own
    /// `full` feature set is what CI's merge-gate test runs under, so
    /// assert the full protocol set is actually present under it.
    #[test]
    #[cfg(feature = "full")]
    fn protocol_dir_scan_finds_something() {
        let dirs = protocol_dirs();
        assert!(
            dirs.len() >= 8,
            "expected at least 8 protocol directories under `full`, found {}: {dirs:?} \
             — did a protocol directory get renamed or a feature gate change?",
            dirs.len()
        );
        for &dir in &dirs {
            assert!(
                !dir_source(dir).is_empty(),
                "protocol dir {dir:?} scanned as empty — check SRC_DIR / feature gating"
            );
        }
    }
}

mod golden_coverage_selftest {
    //! The permanent fix for a pattern that kept recurring across
    //! protocols: a real-signal test file that checks recall (did the
    //! expected message decode) without ever checking precision (did
    //! *only* the expected messages decode). FT4/FST4/JT9/MSK144 all
    //! independently reinvented `GoldenEntry::msg()`-only calls to
    //! `assert_golden` that skipped freq/dt/SNR too — three separate
    //! bespoke recall-only checks per protocol was the norm before this
    //! session's sweep, and Q65/FT8 had no precision check *at all*
    //! until then. This test makes it structural: any real-signal
    //! golden test file must call `assert_golden(` or say why not, so
    //! the gap can't quietly reopen the next time a protocol test is
    //! added or edited.
    //!
    //! Scope is intentionally narrow — the two file-naming conventions
    //! already established for external-reference-decoder comparisons
    //! (`*_wsjtx_samples.rs` per protocol, `ft8_qso3_*_recall.rs` for
    //! FT8's qso3_busy.wav family, which predates and doesn't fit the
    //! first pattern). Other files that load a real WAV for a different
    //! purpose — embedded-vs-host consistency, SIC round-count
    //! monotonicity, wall-clock timing — use different, already-settled
    //! naming and are correctly outside this net; broadening the
    //! trigger to "loads any real WAV" would flag those for no reason.
    //!
    //! A file that has a real, considered reason not to use
    //! `assert_golden` (e.g. an `#[ignore]`d research-ceiling probe with
    //! no phantom-budget concept, or a strict-superset-across-two-runs
    //! invariant that doesn't fit the single-`expected`-list model) opts
    //! out with a `GOLDEN-EXEMPT:` marker comment explaining why —
    //! `ft8_qso3_jtdx_recall.rs`, `ft8_qso3_jtdx_high_sensitivity_recall.rs`
    //! and `ft8_qso3_apon_recall.rs` are the three current exemptions.
    //! The marker exists so an exemption is a decision on record, not a
    //! silent gap this test can't tell apart from one.

    use std::fs;
    use std::path::Path;

    const TESTS_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests");
    const EXEMPT_MARKER: &str = "GOLDEN-EXEMPT:";
    const REQUIRED_CALL: &str = "assert_golden(";

    /// True for the two established real-signal-golden naming
    /// conventions. See the module doc above for why the net isn't
    /// wider than this.
    fn is_real_signal_golden_file(name: &str) -> bool {
        (name.ends_with("_wsjtx_samples.rs"))
            || (name.starts_with("ft8_qso3_") && name.ends_with("_recall.rs"))
    }

    #[test]
    fn every_real_signal_golden_file_uses_assert_golden_or_is_exempt() {
        let dir = Path::new(TESTS_DIR);
        let entries =
            fs::read_dir(dir).unwrap_or_else(|e| panic!("reading {} failed: {e}", dir.display()));

        let mut checked: Vec<String> = Vec::new();
        let mut offenders: Vec<String> = Vec::new();
        for entry in entries {
            let entry = entry.expect("readdir entry");
            let name = entry.file_name().to_string_lossy().into_owned();
            if !name.ends_with(".rs") || !is_real_signal_golden_file(&name) {
                continue;
            }
            let src = fs::read_to_string(entry.path())
                .unwrap_or_else(|e| panic!("reading {name} failed: {e}"));
            checked.push(name.clone());
            if !src.contains(REQUIRED_CALL) && !src.contains(EXEMPT_MARKER) {
                offenders.push(name);
            }
        }

        // Sanity check on the scan itself — if the naming convention
        // ever changes and this stops matching anything, the test would
        // trivially "pass" while checking nothing, exactly the
        // green-lie this file exists to prevent.
        assert!(
            checked.len() >= 10,
            "expected at least 10 real-signal golden test files under {}, found {}: {checked:?} \
             — did the naming convention change? Update `is_real_signal_golden_file`.",
            dir.display(),
            checked.len()
        );

        assert!(
            offenders.is_empty(),
            "{} real-signal golden test file(s) call neither `{REQUIRED_CALL}` nor carry a \
             `{EXEMPT_MARKER}` comment explaining why not: {offenders:#?}\n\
             Every test that checks a decoder's output against a real recording's known \
             messages must assert precision (no extra decodes), not just recall (all \
             expected decodes present) — see `common::golden`'s module doc for the exact \
             failure mode this catches (WSPR's 50% phantom rate behind a green recall test). \
             Either wire the file through `common::golden::assert_golden`, or add a \
             `{EXEMPT_MARKER}` comment stating the considered reason it doesn't fit that model.",
            offenders.len()
        );
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
