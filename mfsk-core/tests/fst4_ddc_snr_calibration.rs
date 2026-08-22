//! What SNR can a DDC receiver report, and how close is it to the one
//! WSJT-X's own formula gives? — issue #307/#346.
//!
//! FST4's shipped SNR estimate (`fst4::baseline::fst4_snr_db`) reads
//! its noise baseline and its unnormalised signal power out of the
//! whole-slot forward FFT. A DDC front end never computes that FFT —
//! not computing it is the entire reason the front end exists — so on
//! the wideband monitor path the field came back `NaN` and the receiver
//! app had nothing to show.
//!
//! `fst4_snr_db_from_cs` estimates it from the symbol spectra the
//! decoder already built instead. This test is what says whether that
//! estimate is worth showing: it decodes the same recording twice, once
//! through each path, and compares them signal by signal.
//!
//! Two tests, and the second is the one that carries the weight: the
//! golden has two signals, while `fst4_ddc_snr_vs_nominal` runs the
//! estimator against a corpus whose SNR is **known by construction**
//! across the range a weak-signal monitor operates in.
//!
//! Measured (2026-08-22). Against `fst4sim`'s nominal, AWGN:
//!
//! | nominal | n | mean | bias | sd |
//! |---:|---:|---:|---:|---:|
//! | -20 | 20 | -20.0 | +0.0 | 0.44 |
//! | -22 | 20 | -21.9 | +0.1 | 0.66 |
//! | -24 | 20 | -23.7 | +0.3 | 0.94 |
//! | -26 | 20 | -25.4 | +0.6 | 1.41 |
//! | -27 | 17 | -26.0 | +1.0 | 1.70 |
//! | -28 |  7 | -27.9 | +0.1 | 2.41 |
//!
//! The small positive bias at the weakest cells is selection: only
//! decoded trials can be measured, and a lucky-high realisation decodes
//! more often than a lucky-low one.
//!
//! And against the real recording, where `jt9` itself is the ground
//! truth (probed values from issue #255, recorded in
//! `fst4::baseline`'s own doc comment):
//!
//! | signal | jt9 | production (whole-slot FFT) | DDC |
//! |---|---:|---:|---:|
//! | N5TM | -6.90 | -8.61 | **-5.7** |
//! | K9KFR | 16.14 | 16.82 | **15.7** |
//!
//! On both signals the DDC estimate is *closer to jt9* than the
//! shipped whole-slot formula is. That is not a claim that it is
//! better in general — two signals — but it does say the DDC path is
//! not the poor relation here.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::PathBuf;

use num_complex::Complex;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

use mfsk_core::engine::dsp::ddc::StreamingComplexDdc;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync::{AudioSource, RxGrid, SyncCandidate, coarse_sync};
use mfsk_core::engine::sync2d::fst4_sync_search;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{
    REFINE_DS_RATE_HZ, grid_for, wideband_cascade, wideband_refine_recenter,
};
use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

const CENTER_HZ: f32 = 1550.0;
const HALF: f32 = 1450.0;
const SYNC_MIN: f32 = 0.8;
const MAX_CAND: usize = 50;
const SYNC_Q_MIN: u32 = 16;

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    )
}

fn msg_of(m: &[u8]) -> Option<String> {
    m.try_into().ok().and_then(|a: &[u8; 77]| unpack77(a))
}

/// Mirrors `embedded_shared::fst4_monitor::ddc_refine`.
fn ddc_refine(c: &SyncCandidate, ci: &[f32], cq: &[f32], d: usize) -> Vec<Complex<f32>> {
    let mut r = wideband_refine_recenter(c.freq_hz, CENTER_HZ);
    let (mut a, mut b) = (Vec::new(), Vec::new());
    r.push(ci, cq, &mut a, &mut b);
    r.flush(&mut a, &mut b);
    let t = ((d as f32 * REFINE_DS_RATE_HZ / 12_000.0).round() as usize
        + r.group_delay_output_samples())
    .min(a.len());
    let mut cd0: Vec<Complex<f32>> = a[t..]
        .iter()
        .zip(b[t..].iter())
        .map(|(&i, &q)| Complex::new(i, q))
        .collect();
    let s2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len().max(1) as f32;
    if s2 > f32::EPSILON {
        let inv = 1.0 / s2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
    cd0
}

/// Every distinct message the wideband DDC monitor path decodes, with
/// the SNR it reports for it.
fn ddc_decodes(audio: &[i16]) -> Vec<(String, f32)> {
    let cfg = wideband_cascade(CENTER_HZ);
    let mut ddc = StreamingComplexDdc::new(&cfg);
    let (mut ci, mut cq) = (Vec::new(), Vec::new());
    ddc.push_i16(audio, &mut ci, &mut cq);
    ddc.flush(&mut ci, &mut cq);
    let delay = ddc.group_delay_input_samples();
    let grid = grid_for(2900.0);
    let cands = coarse_sync::<Fst4s60>(
        AudioSource::Complex(&ci, &cq),
        CENTER_HZ - HALF,
        CENTER_HZ + HALF,
        SYNC_MIN,
        None,
        MAX_CAND,
        RxGrid::complex(grid.fs_c, CENTER_HZ),
    );

    let mut out: Vec<(String, f32)> = Vec::new();
    for c in &cands {
        let cd0 = ddc_refine(c, &ci, &cq, delay);
        let s2 = fst4_sync_search::<Fst4s60>(&cd0, c);
        let pre = (cd0, s2.freq_hz, s2.i0, s2.score);
        // `fft_cache = &[]` and `skip_snr = false`: exactly what the
        // embedded monitor now passes, which is what routes FST4's
        // `snr_db` override to the no-whole-slot-FFT estimator.
        let Some(r) = process_candidate_precomputed::<Fst4s60>(
            c,
            &[],
            &FST4_60A_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            pre,
            false,
            false,
        ) else {
            continue;
        };
        let Some(text) = msg_of(r.message77()) else {
            continue;
        };
        if !out.iter().any(|(m, _)| m == &text) {
            out.push((text, r.snr_db));
        }
    }
    out
}

#[test]
fn fst4_ddc_snr_calibration() {
    let Some(path) = sample_path() else {
        if std::env::var("MFSK_REQUIRE_CORPUS").is_ok() {
            panic!("MFSK_REQUIRE_CORPUS set but fst4/210115_0058.wav is absent");
        }
        eprintln!("fst4/210115_0058.wav absent — skipping");
        return;
    };
    let audio = load_wav_i16_opt(&path).expect("golden WAV readable");

    let production: Vec<(String, f32)> =
        DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, SYNC_MIN, MAX_CAND)
            .decode()
            .results
            .iter()
            .filter_map(|r| msg_of(r.message77()).map(|m| (m, r.snr_db)))
            .collect();
    let ddc = ddc_decodes(&audio);

    eprintln!("\nFST4-60 golden — SNR by path");
    eprintln!(
        "  {:<24} {:>12} {:>12} {:>8}",
        "message", "production", "DDC", "delta"
    );
    let mut deltas: Vec<f32> = Vec::new();
    for (msg, prod_snr) in &production {
        let Some((_, ddc_snr)) = ddc.iter().find(|(m, _)| m == msg) else {
            eprintln!("  {msg:<24} {prod_snr:>12.1} {:>12}", "(not decoded)");
            continue;
        };
        eprintln!(
            "  {msg:<24} {prod_snr:>12.1} {ddc_snr:>12.1} {:>8.1}",
            ddc_snr - prod_snr
        );
        assert!(
            ddc_snr.is_finite(),
            "{msg}: DDC path reported a non-finite SNR — the whole point of \
             `fst4_snr_db_from_cs` is that this path has a number to report"
        );
        deltas.push(ddc_snr - prod_snr);
    }
    assert!(
        deltas.len() >= 2,
        "need at least two signals to say anything about the estimator's slope; got {}",
        deltas.len()
    );

    let mean = deltas.iter().sum::<f32>() / deltas.len() as f32;
    let spread = deltas.iter().cloned().fold(f32::MIN, f32::max)
        - deltas.iter().cloned().fold(f32::MAX, f32::min);
    eprintln!("  mean delta {mean:+.2} dB, spread {spread:.2} dB");
    eprintln!(
        "  (mean is what `SNR_CS_OFFSET_DB` absorbs; spread is what says the estimator tracks)"
    );

    assert!(
        spread <= 4.0,
        "DDC SNR does not track the production estimate: {spread:.1} dB of spread across \
         signals means no constant offset can line the two up"
    );
    assert!(
        mean.abs() <= 2.0,
        "DDC SNR is offset from the production estimate by {mean:+.1} dB"
    );

    // `jt9` itself, on this recording — the values issue #255 probed a
    // real WSJT-X build for. The production estimator sits 0.7-1.7 dB
    // from these; anything in that neighbourhood is doing as well as
    // the thing it stands in for.
    for (msg, jt9) in [("CQ N5TM EL29", -6.90f32), ("CQ K9KFR EN71", 16.14)] {
        let Some((_, ddc_snr)) = ddc.iter().find(|(m, _)| m == msg) else {
            panic!("{msg} did not decode on the DDC path");
        };
        let err = ddc_snr - jt9;
        eprintln!("  vs jt9: {msg:<24} {jt9:>+6.2} -> {ddc_snr:>+6.2} ({err:+.2} dB)");
        assert!(
            err.abs() <= 3.0,
            "{msg}: DDC SNR {ddc_snr:+.1} is {err:+.1} dB from jt9's own {jt9:+.1}"
        );
    }
}

/// The golden above is two signals. This is the same estimator against
/// a corpus whose SNR is **known by construction** — `fst4sim`'s own
/// nominal dB, in the same 2500 Hz reference the estimator reports in —
/// across the range a weak-signal monitor actually operates in.
///
/// Tier C, needs the generated corpus:
///
/// ```sh
/// scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh
/// cargo test -p mfsk-core --features full,internal-testing --release \
///   --test fst4_ddc_snr_calibration fst4_ddc_snr_vs_nominal -- --ignored --nocapture
/// ```
#[test]
#[ignore = "tier C — needs the fst4_60_awgn_* sweep corpus; run with --ignored --nocapture"]
fn fst4_ddc_snr_vs_nominal() {
    const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
    const TRIALS: usize = 20;

    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let dir = std::path::Path::new(&manifest).join("../embedded-poc/assets/fst4_sweep");

    eprintln!("\nFST4-60 AWGN — DDC SNR estimate vs fst4sim's nominal");
    eprintln!(
        "  {:>8} {:>5} {:>10} {:>8} {:>8}",
        "nominal", "n", "mean", "bias", "sd"
    );
    let mut all_bias: Vec<f32> = Vec::new();
    for nominal in [-20i32, -22, -24, -26, -27, -28] {
        let mut snrs: Vec<f32> = Vec::new();
        for trial in 1..=TRIALS {
            let path = dir.join(format!("fst4_60_awgn_m{:02}_{trial:02}.wav", -nominal));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            if let Some((_, snr)) = ddc_decodes(&audio)
                .into_iter()
                .find(|(m, _)| m == GOLDEN_MSG)
                && snr.is_finite()
            {
                snrs.push(snr);
            }
        }
        if snrs.is_empty() {
            eprintln!("  {nominal:>8} {:>5} {:>10}", 0, "(no decode)");
            continue;
        }
        let n = snrs.len() as f32;
        let mean = snrs.iter().sum::<f32>() / n;
        let sd = (snrs.iter().map(|s| (s - mean).powi(2)).sum::<f32>() / n).sqrt();
        let bias = mean - nominal as f32;
        eprintln!(
            "  {nominal:>8} {:>5} {mean:>10.1} {bias:>+8.1} {sd:>8.2}",
            snrs.len()
        );
        all_bias.push(bias);
    }

    assert!(
        all_bias.len() >= 3,
        "corpus too sparse to say anything: {} cells produced decodes",
        all_bias.len()
    );
    let spread = all_bias.iter().cloned().fold(f32::MIN, f32::max)
        - all_bias.iter().cloned().fold(f32::MAX, f32::min);
    let mean_bias = all_bias.iter().sum::<f32>() / all_bias.len() as f32;
    eprintln!("  mean bias {mean_bias:+.2} dB, bias spread {spread:.2} dB across cells");
    assert!(
        spread <= 3.0,
        "the estimate does not track nominal SNR: bias varies {spread:.1} dB across cells"
    );
}
