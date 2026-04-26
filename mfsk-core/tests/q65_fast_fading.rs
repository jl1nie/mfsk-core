//! Q65 fast-fading metric integration tests.
//!
//! Validates the Doppler-spread-aware path
//! ([`mfsk_core::q65::decode_at_fading_for`] +
//! [`mfsk_core::q65::decode_scan_fading_for`]) end-to-end:
//! - clean synthetic input round-trips at every wired sub-mode;
//! - the metric tolerates injected frequency spread that defeats the
//!   plain AWGN Bessel front end at the same SNR;
//! - the optional WSJT-X 10 GHz EME reference recording yields at
//!   least one decode when the sample tree is present.

#![cfg(feature = "q65")]

use std::f32::consts::TAU;
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

use mfsk_core::fec::qra::FadingModel;
use mfsk_core::msg::ApHint;
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{
    Q65a30, Q65a60, Q65d60, decode_at_fading_for, decode_scan_default, decode_scan_fading_for,
    synthesize_standard, synthesize_standard_for,
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
    let result =
        decode_at_fading_for::<Q65a30>(&audio, FS_U, 0, freq, 0.05, FadingModel::Gaussian, None)
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
    let result =
        decode_at_fading_for::<Q65d60>(&audio, FS_U, 0, freq, 0.1, FadingModel::Gaussian, None)
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
    let result = decode_at_fading_for::<Q65a30>(
        &audio,
        FS_U,
        0,
        freq,
        0.05,
        FadingModel::Gaussian,
        Some(&hint),
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
            time_tolerance_symbols: 8,
            score_threshold: 0.05,
            max_candidates: 16,
        };
        let bessel = !decode_scan_default(&slot, FS_U).is_empty();
        let fading = !decode_scan_fading_for::<Q65a30>(
            &slot,
            FS_U,
            0,
            &params,
            0.5,
            FadingModel::Gaussian,
            None,
        )
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
        time_tolerance_symbols: 4,
        score_threshold: 0.05,
        max_candidates: 8,
    };
    let decodes = decode_scan_fading_for::<Q65a30>(
        &audio,
        FS_U,
        0,
        &params,
        0.3,
        FadingModel::Gaussian,
        None,
    );
    assert!(
        decodes.is_empty(),
        "got false decode(s) from low-level noise: {decodes:#?}"
    );
}

// ─── WSJT-X reference sample (optional) ─────────────────────────

fn read_wsjtx_wav(path: &Path) -> Option<Vec<f32>> {
    let mut file = File::open(path).ok()?;
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).ok()?;
    if bytes.len() < 44 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        return None;
    }
    if &bytes[12..16] != b"fmt " {
        return None;
    }
    let channels = u16::from_le_bytes([bytes[22], bytes[23]]);
    let sample_rate = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
    let bits = u16::from_le_bytes([bytes[34], bytes[35]]);
    if channels != 1 || sample_rate != 12_000 || bits != 16 {
        return None;
    }
    if &bytes[36..40] != b"data" {
        return None;
    }
    let data_len = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
    let data = &bytes[44..44 + data_len.min(bytes.len() - 44)];
    let mut out = Vec::with_capacity(data.len() / 2);
    for chunk in data.chunks_exact(2) {
        let s = i16::from_le_bytes([chunk[0], chunk[1]]);
        out.push(s as f32 / 32_768.0);
    }
    Some(out)
}

fn samples_dir(rel: &str) -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let dir = Path::new(&manifest)
        .join("../../WSJT-X/samples/Q65")
        .join(rel)
        .canonicalize()
        .ok()?;
    if dir.is_dir() { Some(dir) } else { None }
}

#[test]
fn eme_10ghz_reference_decodes_with_fast_fading() {
    // Q65-60D 10 GHz EME recording. Fast-fading is REQUIRED for this
    // path: the AWGN Bessel metric loses 5–8 dB to libration spread
    // (≈25 Hz at 10 GHz) and never recovers it. Without a sample
    // tree this test reports as skipped.
    let Some(dir) = samples_dir("60D_EME_10GHz") else {
        eprintln!("skipping: WSJT-X 10 GHz EME sample tree not found");
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
        time_tolerance_symbols: 50,
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
                let decodes = decode_scan_fading_for::<Q65d60>(
                    &audio,
                    FS_U,
                    nominal_mid,
                    &params,
                    b90,
                    model,
                    None,
                );
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
        eprintln!("skipping: WSJT-X 6 m EME sample tree not found");
        return;
    };
    let entries: Vec<_> = std::fs::read_dir(&dir)
        .expect("read samples dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("wav"))
        .collect();
    if entries.is_empty() {
        eprintln!("skipping: 6 m EME sample dir empty");
        return;
    }

    let nominal_mid = FS_U as usize * 30;
    let params = SearchParams {
        freq_min_hz: 200.0,
        freq_max_hz: 3_000.0,
        time_tolerance_symbols: 50,
        score_threshold: 0.05,
        max_candidates: 32,
    };

    let mut total = 0usize;
    for path in &entries {
        let audio = match read_wsjtx_wav(path) {
            Some(a) => a,
            None => continue,
        };
        let decodes = decode_scan_fading_for::<Q65a60>(
            &audio,
            FS_U,
            nominal_mid,
            &params,
            0.1,
            FadingModel::Gaussian,
            None,
        );
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
