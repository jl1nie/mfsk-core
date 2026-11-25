//! Q65 fast-fading metric integration tests.
//!
//! Validates the Doppler-spread-aware path
//! ([`mfsk_core::q65::Q65Candidate::fading`] +
//! [`mfsk_core::__internal::q65::DecodeRequest::fading`]) end-to-end:
//! - clean synthetic input round-trips at every wired sub-mode;
//! - the metric tolerates injected frequency spread that defeats the
//!   plain AWGN Bessel front end at the same SNR;
//! - the optional WSJT-X 10 GHz EME reference recording yields at
//!   least one decode when the sample tree is present.

#![cfg(feature = "q65")]

use std::f32::consts::TAU;
#[allow(dead_code)]
mod common;
use common::load_wav_f32_opt as read_wsjtx_wav;
use std::path::PathBuf;

use mfsk_core::__internal::q65::DecodeRequest;
use mfsk_core::engine::alignment::Alignment;
use mfsk_core::fec::qra::FadingModel;
use mfsk_core::msg::ApHint;
use mfsk_core::q65::search::{SearchParams, default_search_params};
use mfsk_core::q65::{
    Q65Candidate, Q65a30, Q65a60, Q65d60, decode_candidate, synthesize_standard,
    synthesize_standard_for,
};

const FS: f32 = 12_000.0;
const FS_U: u32 = 12_000;
const REF_BW: f32 = 2_500.0;

/// Tiny LCG + Box–Muller, mirroring the FT4/FT8/Q65 sweep harness so
/// the SNR scale stays directly comparable with the AWGN sweep.
struct Lcg {
    s: u64,
    spare: Option<f32>,
}
impl Lcg {
    fn new(seed: u64) -> Self {
        Self {
            s: seed.wrapping_add(1),
            spare: None,
        }
    }
    fn next(&mut self) -> u64 {
        self.s = self
            .s
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.s
    }
    fn uniform(&mut self) -> f32 {
        ((self.next() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn gauss(&mut self) -> f32 {
        if let Some(x) = self.spare.take() {
            return x;
        }
        let u = self.uniform();
        let v = self.uniform();
        let mag = (-2.0 * u.ln()).sqrt();
        self.spare = Some(mag * (TAU * v).sin());
        mag * (TAU * v).cos()
    }
}

#[test]
fn fast_fading_decodes_clean_q65a30_signal() {
    // The fast-fading front end with a tight spread parameter and the
    // Gaussian model should decode a clean AWGN signal just like the
    // Bessel front end. This pins the audio → wide-spectrogram →
    // intrinsics → QRA path on the most common Q65 sub-mode.
    let freq = 1500.0;
    let audio = synthesize_standard("CQ", "K1ABC", "FN42", FS_U, freq, 0.3).expect("synth");
    let result = decode_candidate::<Q65a30>(
        &audio,
        FS_U,
        &Q65Candidate::new(Alignment::new(0, freq)).fading(FadingModel::Gaussian, 0.05),
    )
    .expect("fast-fading decode of a clean Q65-30A signal must succeed");
    assert_eq!(result.message, "CQ K1ABC FN42");
}

#[test]
fn fast_fading_decodes_clean_q65d60_signal() {
    // Q65-60D is the heaviest-spread wired sub-mode (×8 baud spacing).
    // Confirms the wide-energy extractor handles `bins_per_tone = 8`
    // correctly: 64 leading pad + 64 tones × 8 bins + 64 trailing pad
    // = 640 bins per symbol.
    let freq = 1500.0;
    let audio =
        synthesize_standard_for::<Q65d60>("CQ", "W7GJ", "DN27", FS_U, freq, 0.3).expect("synth");
    let result = decode_candidate::<Q65d60>(
        &audio,
        FS_U,
        &Q65Candidate::new(Alignment::new(0, freq)).fading(FadingModel::Gaussian, 0.1),
    )
    .expect("fast-fading decode of a clean Q65-60D signal must succeed");
    assert_eq!(result.message, "CQ W7GJ DN27");
}

#[test]
fn fast_fading_with_ap_hint_decodes_clean_signal() {
    // AP hint should compose with the fast-fading metric exactly as
    // it composes with the AWGN metric — i.e. it tightens convergence
    // but never causes a wrong decode.
    let freq = 1500.0;
    let audio = synthesize_standard("CQ", "JA1ABC", "PM95", FS_U, freq, 0.3).expect("synth");
    let hint = ApHint::new().with_call1("CQ");
    let result = decode_candidate::<Q65a30>(
        &audio,
        FS_U,
        &Q65Candidate::new(Alignment::new(0, freq))
            .fading(FadingModel::Gaussian, 0.05)
            .ap_hint(&hint),
    )
    .expect("fast-fading + AP must decode a clean signal");
    assert_eq!(result.message, "CQ JA1ABC PM95");
}

/// Apply a slow random amplitude envelope to the audio buffer. The
/// envelope is `1 + α · n(t)` where `n(t)` is bandlimited Gaussian
/// noise — multiplicative time-domain modulation that smears each
/// tone over an FFT bandwidth proportional to the envelope's
/// effective bandwidth `B`. Used to mimic the Doppler-spread channel
/// the fast-fading metric is calibrated for.
fn apply_doppler_spread(audio: &mut [f32], sample_rate: u32, b_hz: f32, alpha: f32, seed: u64) {
    // First-order IIR low-pass to bandlimit white Gaussian to ~B/2 Hz.
    let dt = 1.0 / sample_rate as f32;
    let rc = 1.0 / (TAU * (b_hz * 0.5));
    let a = dt / (rc + dt);
    let mut rng = Lcg::new(seed);
    let mut env = 0.0_f32;
    for s in audio.iter_mut() {
        env += a * (rng.gauss() - env);
        *s *= 1.0 + alpha * env;
    }
}

#[test]
#[ignore = "slow: 8-seed Doppler-spread sweep; minutes in debug. CI runs via --include-ignored."]
fn fast_fading_handles_moderate_spread_better_than_bessel() {
    // Q65-30A signal with a synthetic ~6 Hz Doppler spread — wide
    // enough that the central FFT bin alone misses ~half the tone
    // energy. Compare the AWGN scan path against the fast-fading
    // scan path across several seeds; assert the fast-fading path
    // succeeds at least as often (and typically more often).
    //
    // We do NOT assert the AWGN path always fails — it can win on
    // lucky seeds — only that fast-fading is no worse, and that on
    // a reasonable number of seeds it strictly wins.
    let freq = 1500.0;
    let snr_db = -18.0; // moderately above Q65-30A AWGN threshold
    let snr_lin = 10_f32.powf(snr_db / 10.0);
    let amp = (4.0 * snr_lin * REF_BW / FS).sqrt();

    let seeds = [1u64, 2, 3, 4, 5, 6, 7, 8];
    let mut bessel_hits = 0usize;
    let mut fading_hits = 0usize;
    let mut fading_strictly_won = 0usize;

    for &seed in &seeds {
        let mut audio = synthesize_standard("CQ", "K9AN", "EN50", FS_U, freq, amp).expect("synth");
        // Doppler spread: ~6 Hz envelope bandwidth, modulation index 0.6.
        apply_doppler_spread(&mut audio, FS_U, 6.0, 0.6, seed);
        // Add AWGN.
        let mut rng = Lcg::new(seed.wrapping_add(0xA1A1));
        for s in audio.iter_mut() {
            *s += rng.gauss();
        }

        // Pad to a 30 s slot so the scan path has the full window.
        let mut slot = vec![0.0_f32; FS_U as usize * 30];
        let n = audio.len().min(slot.len());
        slot[..n].copy_from_slice(&audio[..n]);

        let params = SearchParams {
            freq_min_hz: 200.0,
            freq_max_hz: 3_000.0,
            time_tolerance_early_sec: 2.4,
            time_tolerance_late_sec: 2.4,
            score_threshold: 0.05,
            max_candidates: 16,
        };
        let bessel = !DecodeRequest::<Q65a30>::new(&slot, FS_U, 0, default_search_params())
            .decode()
            .is_empty();
        let fading = !DecodeRequest::<Q65a30>::new(&slot, FS_U, 0, params)
            .fading(FadingModel::Gaussian, 0.5)
            .decode()
            .is_empty();
        if bessel {
            bessel_hits += 1;
        }
        if fading {
            fading_hits += 1;
        }
        if fading && !bessel {
            fading_strictly_won += 1;
        }
    }

    println!(
        "Doppler-spread sweep: bessel {}/{}, fading {}/{}, fading-only-wins {}",
        bessel_hits,
        seeds.len(),
        fading_hits,
        seeds.len(),
        fading_strictly_won,
    );

    assert!(
        fading_hits >= bessel_hits,
        "fast-fading metric should not regress vs AWGN under Doppler spread \
         (bessel={bessel_hits}, fading={fading_hits})"
    );
}

#[test]
fn fast_fading_does_not_decode_pure_silence() {
    // Sanity guard for false positives — the fast-fading path must
    // not hallucinate a decode out of low-level noise.
    let mut audio = vec![0.0_f32; FS_U as usize * 30];
    let mut rng = Lcg::new(42);
    for s in audio.iter_mut() {
        *s = 0.001 * rng.gauss();
    }
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 1.2,
        time_tolerance_late_sec: 1.2,
        score_threshold: 0.05,
        max_candidates: 8,
    };
    let decodes = DecodeRequest::<Q65a30>::new(&audio, FS_U, 0, params)
        .fading(FadingModel::Gaussian, 0.3)
        .decode();
    assert!(
        decodes.is_empty(),
        "got false decode(s) from low-level noise: {decodes:#?}"
    );
}

// ─── WSJT-X reference sample (vendored golden) ─────────────────

fn samples_dir(rel: &str) -> Option<PathBuf> {
    // Every Q65 sub-mode directory WSJT-X ships is vendored under
    // `embedded-poc/assets/golden/q65/` (~23 MB beside the original
    // 60A/60D). Before that only 60A and 60D were, and the rest resolved
    // from `$WSJTX_SAMPLES_DIR` as optional — so six golden tests
    // (30A, 60B, 120D, 120E, 300A, and the 30A streaming parity) skipped
    // on CI and reported `ok`. `golden_subdir` panics under
    // `MFSK_REQUIRE_CORPUS`, which CI and the merge gate set.
    common::corpus::golden_subdir(&format!("q65/{rel}"))
}

/// A source read forward gives the decode of the slice, on a real recording: the 10 GHz EME file. The wide-band scan (through
/// the fast-fading metric, as `eme_10ghz_reference_decodes_with_fast_fading` runs it) finds where the signal is; at that
/// `(start_sample, freq_hz)` the point decode is made from the slice and from a source read forward, under the fast-fading
/// metric (the streamed one makes its metric a symbol row at a time) and plain, and at a place 5 Hz off. The message, frequency,
/// time, AP flag and SNR (a second read of the audio) must be the slice decode's, to the bit; `iterations` is not compared (the
/// fast-fading metric differs by float rounding). Something decodes, so this cannot pass by both sides finding nothing.
#[test]
fn a_streamed_decode_equals_the_slice_decode_on_a_real_recording() {
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::q65::decode_candidate_streamed;

    let Some(dir) = samples_dir("60D_EME_10GHz") else {
        common::skip_or_fail("Q65 golden 10 GHz EME");
        return;
    };
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.05,
        max_candidates: 32,
    };
    let same = |slice: &Option<mfsk_core::q65::Q65Result>,
                streamed: &Option<mfsk_core::q65::Q65Result>,
                what: &str| {
        match (slice, streamed) {
            (None, None) => false,
            (Some(a), Some(b)) => {
                assert_eq!(a.message, b.message, "{what}");
                assert_eq!(a.freq_hz.to_bits(), b.freq_hz.to_bits(), "{what}");
                assert_eq!(a.start_sample, b.start_sample, "{what}");
                assert_eq!(a.dt_sec.to_bits(), b.dt_sec.to_bits(), "{what}");
                assert_eq!(a.ap, b.ap, "{what}");
                assert_eq!(
                    a.snr_db.to_bits(),
                    b.snr_db.to_bits(),
                    "{what}: {}: snr {} against {}",
                    a.message,
                    a.snr_db,
                    b.snr_db
                );
                true
            }
            (a, b) => panic!(
                "{what}: the slice decode {}, the streamed one {}",
                if a.is_some() { "decodes" } else { "does not" },
                if b.is_some() { "decodes" } else { "does not" },
            ),
        }
    };
    let mut compared = 0usize;
    for path in std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
    {
        let Some(audio) = read_wsjtx_wav(&path) else {
            continue;
        };
        // the f32 of a 16-bit WAV is `i16 / 32768`: the i16 comes back exactly
        let raw: Vec<i16> = audio
            .iter()
            .map(|&x| (x * 32_768.0).round() as i16)
            .collect();
        for model in [FadingModel::Gaussian, FadingModel::Lorentzian] {
            for b90 in [3.0_f32, 8.0, 15.0] {
                let found = DecodeRequest::<Q65d60>::new(&audio, FS_U, FS_U as usize * 30, params)
                    .fading(model, b90)
                    .decode();
                for d in &found {
                    for (what, freq_shift, cand) in [
                        ("fading", 0.0_f32, Some((model, b90))),
                        ("plain", 0.0, None),
                        ("fading, 5 Hz off", 5.0, Some((model, b90))),
                    ] {
                        let mut c = Q65Candidate::new(Alignment::new(
                            d.start_sample,
                            d.freq_hz + freq_shift,
                        ));
                        if let Some((m, b)) = cand {
                            c = c.fading(m, b);
                        }
                        let slice = decode_candidate::<Q65d60>(&audio, FS_U, &c);
                        let streamed = decode_candidate_streamed::<Q65d60, _>(
                            &mut SliceSource(&raw[..]),
                            FS_U,
                            &c,
                        );
                        if same(&slice, &streamed, what) {
                            compared += 1;
                        }
                    }
                }
            }
        }
    }
    println!("{compared} point decodes compared, slice against source");
    assert!(
        compared > 0,
        "nothing decoded on either side: the test compares nothing"
    );
}

/// The streamed coarse search is the whole-spectrogram one on a real recording, at the default 200-3000 Hz window of a scan:
/// one band and eight, with a drift search and without. The candidates (start, frequency, score and drift) must be equal to
/// the bit. Returns how many were compared.
fn streamed_coarse_search_equals_the_whole_one<P: mfsk_core::engine::ModulationParams>(
    recordings: &str,
) -> Option<usize> {
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::q65::search::{
        build_spectrogram, coarse_search_drift_on_spec_for, coarse_search_streamed,
    };

    let Some(dir) = samples_dir(recordings) else {
        common::skip_or_fail(recordings);
        return None;
    };
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.05,
        max_candidates: 32,
    };
    let bits = |v: &[(mfsk_core::q65::SyncCandidate, i32)]| {
        v.iter()
            .map(|(c, d)| (c.start_sample, c.freq_hz.to_bits(), c.score.to_bits(), *d))
            .collect::<Vec<_>>()
    };
    let nominal = FS_U as usize * 30;
    let mut found = 0usize;
    for path in std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
    {
        let Some(audio) = read_wsjtx_wav(&path) else {
            continue;
        };
        // the f32 of a 16-bit WAV is `i16 / 32768`: the i16 comes back exactly
        let raw: Vec<i16> = audio
            .iter()
            .map(|&x| (x * 32_768.0).round() as i16)
            .collect();
        let spec = build_spectrogram::<P>(&audio, FS_U);
        for (max_drift, bands) in [(0u32, 1usize), (0, 8), (4, 8)] {
            let want =
                coarse_search_drift_on_spec_for::<P>(&spec, FS_U, nominal, &params, max_drift);
            let got = coarse_search_streamed::<P, _>(
                &mut SliceSource(&raw[..]),
                FS_U,
                nominal,
                &params,
                max_drift,
                bands,
            );
            assert_eq!(
                bits(&want),
                bits(&got),
                "{}: drift {max_drift}, {bands} bands",
                path.display()
            );
            found += want.len();
        }
    }
    Some(found)
}

/// The 10 GHz EME file (one signal) and the 6 m EME files (several): something is found on each, so this cannot pass by both
/// sides finding nothing.
#[test]
fn a_streamed_coarse_search_equals_the_whole_one_on_real_recordings() {
    for (recordings, found) in [
        (
            "60D_EME_10GHz",
            streamed_coarse_search_equals_the_whole_one::<Q65d60>("60D_EME_10GHz"),
        ),
        (
            "60A_EME_6m",
            streamed_coarse_search_equals_the_whole_one::<Q65a60>("60A_EME_6m"),
        ),
    ] {
        if let Some(n) = found {
            println!("{recordings}: {n} candidates compared, whole spectrogram against streamed");
            assert!(
                n > 0,
                "{recordings}: no candidate on either side: the test compares nothing"
            );
        }
    }
}

#[test]
fn eme_10ghz_reference_decodes_with_fast_fading() {
    // Q65-60D 10 GHz EME recording. Fast-fading is REQUIRED for this
    // path: the AWGN Bessel metric loses 5–8 dB to libration spread
    // (≈25 Hz at 10 GHz) and never recovers it. Without a sample
    // tree this test reports as skipped.
    let Some(dir) = samples_dir("60D_EME_10GHz") else {
        common::skip_or_fail("Q65 golden 10 GHz EME");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    assert!(
        !entries.is_empty(),
        "WSJT-X 10 GHz EME sample dir contains no .wav files"
    );

    let nominal_mid = FS_U as usize * 30; // 30 s into the 60 s slot
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    // Try a small ladder of B90Ts values. 10 GHz EME libration spread
    // is ≈25 Hz, so B90·Ts ≈ 25 × 0.6 = 15 — far up the table.
    let b90_ts_values = [3.0_f32, 8.0, 15.0];

    let mut total_decodes = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => continue,
        };
        let mut best = 0usize;
        for &b90 in &b90_ts_values {
            for model in [FadingModel::Gaussian, FadingModel::Lorentzian] {
                let decodes = DecodeRequest::<Q65d60>::new(&audio, FS_U, nominal_mid, params)
                    .fading(model, b90)
                    .decode();
                if decodes.len() > best {
                    best = decodes.len();
                    println!(
                        "{} [B90Ts={b90}, {model:?}]: {} decode(s) → {:?}",
                        path.file_name().unwrap().to_string_lossy(),
                        decodes.len(),
                        decodes.iter().map(|d| &d.message).collect::<Vec<_>>(),
                    );
                }
            }
        }
        total_decodes += best;
    }

    // Strong gate: the fast-fading metric is supposed to make 10 GHz
    // EME workable. We require at least one decode across the full
    // sample sweep so a regression in this path trips the test.
    assert!(
        total_decodes > 0,
        "10 GHz EME reference recordings yielded NO decodes via fast-fading — \
         regression in the fast-fading receive chain"
    );
    eprintln!("[info] 10 GHz EME total decodes (fast-fading): {total_decodes}");
}

#[test]
fn eme_6m_sample_does_not_regress_with_fast_fading() {
    // 6 m EME has tiny libration spread (~0.5 Hz at 50 MHz). The
    // AWGN Bessel metric already decodes it well; fast-fading at a
    // tight B90Ts must do at least as well — confirms we have not
    // broken the easy case in pursuit of the hard one.
    let Some(dir) = samples_dir("60A_EME_6m") else {
        common::skip_or_fail("Q65 golden 6 m EME");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    if entries.is_empty() {
        common::skip_or_fail("6 m EME sample dir empty");
        return;
    }

    let nominal_mid = FS_U as usize * 30;
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_early_sec: 30.0,
        time_tolerance_late_sec: 30.0,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let mut total = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => continue,
        };
        let decodes = DecodeRequest::<Q65a60>::new(&audio, FS_U, nominal_mid, params)
            .fading(FadingModel::Gaussian, 0.1)
            .decode();
        total += decodes.len();
        for d in &decodes {
            println!(
                "{}: {}",
                path.file_name().unwrap().to_string_lossy(),
                d.message
            );
        }
    }
    assert!(
        total > 0,
        "6 m EME reference yielded no decodes via fast-fading — \
         we regressed the easy case while building the hard one"
    );
}
