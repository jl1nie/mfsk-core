//! FST4-60 real-world signal validation against
//! `WSJT-X/samples/FST4+FST4W/210115_0058.wav` (12 kHz mono, 60 s).
//!
//! Skipped when the WSJT-X tree is absent.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::PathBuf;

use mfsk_core::fst4::Fst4s60;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    )
}

struct Golden {
    msg: &'static str,
    freq_hz: f32,
    dt_sec: f32,
}

const GOLDEN: &[Golden] = &[Golden {
    msg: "CQ N5TM EL29",
    freq_hz: 1101.0,
    dt_sec: 0.3,
}];

const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5;

#[test]
fn fst4_60_wsjtx_sample_recall_vs_golden() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FST4 sample not found at \
             ../../WSJT-X/samples/FST4+FST4W/210115_0058.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    let decodes = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, 1.0, 50)
        .decode()
        .results;

    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .filter_map(|d| {
            let mut m77 = [0u8; 77];
            m77.copy_from_slice(d.message77());
            unpack77(&m77).map(|s| (s, d.freq_hz, d.dt_sec))
        })
        .collect();

    eprintln!("FST4-60 sample decoded {} message(s):", decoded.len());
    for (m, f, dt) in &decoded {
        eprintln!("  freq={:6.1} Hz dt={:+.2} s : {}", f, dt, m);
    }

    let mut hits = 0usize;
    for g in GOLDEN {
        let hit = decoded.iter().any(|(m, f, dt)| {
            m == g.msg
                && (f - g.freq_hz).abs() <= FREQ_TOL_HZ
                && (dt - g.dt_sec).abs() <= DT_TOL_SEC
        });
        if hit {
            hits += 1;
        } else {
            eprintln!(
                "  MISSING: '{}' @ {:.1} Hz dt={:+.2}",
                g.msg, g.freq_hz, g.dt_sec
            );
        }
    }
    eprintln!("recall: {}/{} golden FST4-60 decodes", hits, GOLDEN.len());

    assert_eq!(
        hits,
        GOLDEN.len(),
        "FST4-60 WSJT-X sample recall regressed: {}/{}",
        hits,
        GOLDEN.len()
    );
}

/// SNR ground truth for issue #255's FST4 real-formula port
/// (`fst4::baseline::fst4_snr_db`): a real local `jt9 -7 -d3` build's
/// own reported SNR for both of this WAV's real decodes
/// (`SNRAUDIT_FST4_PROBE` instrumentation added to `fst4_decode.f90`
/// for that investigation, not committed there). `±3.0` dB tolerance
/// — generous relative to the `~1-2` dB gap the investigation actually
/// landed on, since this is a regression gate, not a precision claim.
struct SnrGolden {
    msg: &'static str,
    jt9_snr_db: f32,
}

const SNR_GOLDEN: &[SnrGolden] = &[
    SnrGolden {
        msg: "CQ N5TM EL29",
        jt9_snr_db: -6.90,
    },
    SnrGolden {
        msg: "CQ K9KFR EN71",
        jt9_snr_db: 16.14,
    },
];

const SNR_TOL_DB: f32 = 3.0;

#[test]
fn fst4_60_wsjtx_sample_snr_matches_jt9_ground_truth() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FST4 sample not found at \
             ../../WSJT-X/samples/FST4+FST4W/210115_0058.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    let decodes = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, 1.0, 50)
        .decode()
        .results;

    let decoded: Vec<(String, f32)> = decodes
        .iter()
        .filter_map(|d| {
            let mut m77 = [0u8; 77];
            m77.copy_from_slice(d.message77());
            unpack77(&m77).map(|s| (s, d.snr_db))
        })
        .collect();

    for g in SNR_GOLDEN {
        let hit = decoded.iter().find(|(m, _)| m == g.msg);
        let Some((_, snr_db)) = hit else {
            panic!(
                "'{}' not decoded at all — recall regression, see \
                 fst4_60_wsjtx_sample_recall_vs_golden",
                g.msg
            );
        };
        eprintln!(
            "{}: ours={:.2} dB, jt9={:.2} dB, diff={:.2} dB",
            g.msg,
            snr_db,
            g.jt9_snr_db,
            (snr_db - g.jt9_snr_db).abs()
        );
        assert!(
            (snr_db - g.jt9_snr_db).abs() <= SNR_TOL_DB,
            "'{}' SNR diverged from jt9 ground truth by {:.2} dB \
             (ours={:.2}, jt9={:.2}) — beyond the {} dB regression gate",
            g.msg,
            (snr_db - g.jt9_snr_db).abs(),
            snr_db,
            g.jt9_snr_db,
            SNR_TOL_DB
        );
    }
}

/// Regression for issue #244: before the pre-decode `dedup_refined_candidates`
/// pass (`engine::pipeline::decode_frame_impl`), this exact WAV fired
/// `on_result` **9 times** for the same 2 real signals (`CQ K9KFR EN71`
/// ×4, `CQ N5TM EL29` ×5) — every one of the redundant candidates paid
/// the full LLR/BP/OSD staircase before a *post-decode* message-based
/// dedup threw the extras away. This is the precise real-world number
/// a user report reproduced independently of this crate's own
/// investigation. Now fixed to fire exactly once per final decode —
/// asserted exactly, not just "fewer than 9", so any regression back
/// toward redundant firings is caught immediately rather than needing
/// its own rediscovery.
#[test]
fn fst4_60_wsjtx_sample_on_result_fires_once_per_decode() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FST4 sample not found at \
             ../../WSJT-X/samples/FST4+FST4W/210115_0058.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    let fired: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
    let cb = |r: &mfsk_core::fst4::decode::DecodeResult| {
        let mut m77 = [0u8; 77];
        m77.copy_from_slice(r.message77());
        if let Some(t) = unpack77(&m77) {
            fired.lock().unwrap().push(t);
        }
    };
    let outcome = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, 1.0, 50)
        .on_result(&cb)
        .decode();

    let fired = fired.into_inner().unwrap();
    println!(
        "batch: {} decode(s), on_result firings: {}",
        outcome.results.len(),
        fired.len()
    );
    for m in &fired {
        println!("  fired: {m}");
    }

    assert_eq!(
        fired.len(),
        outcome.results.len(),
        "on_result should fire exactly once per final decode on this golden \
         WAV (pre-#244 fix this was 9 firings for 2 decodes)"
    );
    assert!(
        !outcome.results.is_empty(),
        "expected real decodes on the FST4-60 golden WAV"
    );
}

/// Diagnostic probe for FST4-60A sync/timing regressions (issue #23's
/// original root cause: `Fst4s60`'s `NSPS`/`NDOWN`/`GFSK_BT` didn't match
/// WSJT-X `fst4_decode.f90`, so real-audio decodes drifted ~0.3-0.6 s off
/// the true frame start even though the synth roundtrip self-decoded
/// fine). Not a pass/fail gate — mirrors
/// `jt9::decode::gate_diag::probe_missing_goldens`'s approach: brute-force
/// scan the (freq, dt) plane around the golden point independent of
/// `coarse_sync`'s quantised bins, so a future timing bug shows up as a
/// displaced `nsync` peak instead of a silent `decode_frame` miss. Run
/// with:
///   cargo test --test fst4_wsjtx_samples fst4_60_diagnose_golden \
///       --features fst4,fft-rustfft,internal-testing -- --ignored --nocapture
///
/// `internal-testing` (issue #203) is required because this probe calls
/// `engine::pipeline::process_candidate_basic` directly, which is
/// `pub(crate)` on the default feature set.
#[test]
#[ignore = "diagnostic probe, not a recall gate — run manually"]
fn fst4_60_diagnose_golden() {
    use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::llr::{symbol_spectra, sync_quality};
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::{SyncCandidate, coarse_sync};
    use mfsk_core::engine::{FrameLayout, ModulationParams};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    let Some(path) = sample_path() else {
        eprintln!("skipping: WSJT-X FST4 sample not found");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let golden = &GOLDEN[0];
    let ds_rate = 12_000.0 / <Fst4s60 as ModulationParams>::NDOWN as f32;
    let tx_start = <Fst4s60 as FrameLayout>::TX_START_OFFSET_S;
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);

    // Brute-force (freq, dt) grid scan around the golden point,
    // independent of coarse_sync's quantised bins — for each freq
    // offset, downsample once and sweep dt over a wide window computing
    // `nsync` directly. A correctly-tuned decoder puts the peak (nsync
    // near 40/40) right on the golden coordinates; a timing/geometry bug
    // shows up as a peak displaced by a fraction of a second or more.
    let mut grid: Vec<(f32, f32, u32)> = Vec::new(); // (freq, dt, nsync)
    let mut at_golden: Option<u32> = None;
    for fi in -12..=12i32 {
        let freq = golden.freq_hz + fi as f32 * 0.25;
        let cd0 = downsample_cached(&fft_cache, freq, &FST4_60A_DOWNSAMPLE);
        let i0_lo = ((-1.0 + tx_start) * ds_rate).round() as i32;
        let i0_hi = ((2.0 + tx_start) * ds_rate).round() as i32;
        for i0 in i0_lo..=i0_hi {
            let cs = symbol_spectra::<Fst4s60>(&cd0, i0);
            let nsync = sync_quality::<Fst4s60>(&cs);
            let dt = i0 as f32 / ds_rate - tx_start;
            if (freq - golden.freq_hz).abs() < 0.13 && (dt - golden.dt_sec).abs() < 0.01 {
                at_golden = Some(nsync);
            }
            grid.push((freq, dt, nsync));
        }
    }
    grid.sort_by_key(|c| std::cmp::Reverse(c.2));
    eprintln!("brute-force (freq,dt) grid scan, top-10 by nsync (of 40):");
    for (freq, dt, n) in grid.iter().take(10) {
        eprintln!("  freq={:8.2} Hz  dt={:+7.3} s  nsync={}", freq, dt, n);
    }
    eprintln!(
        "nsync at golden point (freq={:.1}, dt={:.2}): {:?}  (peak should sit here, not displaced)",
        golden.freq_hz, golden.dt_sec, at_golden
    );

    // Cross-check against `coarse_sync` + the real decode path: is there
    // a candidate inside the golden tolerance window, and does it decode?
    let candidates = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.0, None, 2000);
    let mut in_window: Vec<&SyncCandidate> = candidates
        .iter()
        .filter(|c| {
            (c.freq_hz - golden.freq_hz).abs() <= FREQ_TOL_HZ
                && (c.dt_sec - golden.dt_sec).abs() <= DT_TOL_SEC
        })
        .collect();
    in_window.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
    eprintln!(
        "\ncoarse_sync candidates inside golden tolerance window (freq +-{} Hz, dt +-{} s):",
        FREQ_TOL_HZ, DT_TOL_SEC
    );
    if in_window.is_empty() {
        eprintln!("  (none — coarse_sync proposes nothing in the golden window at all)");
        return;
    }
    for cand in &in_window {
        let result = process_candidate_basic::<Fst4s60>(
            cand,
            &fft_cache,
            &FST4_60A_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            0, // sync_q_min=0: bypass the gate, we want to see every attempt
        );
        let outcome = match result {
            Some(r) => {
                let mut m77 = [0u8; 77];
                m77.copy_from_slice(&r.info[..77]);
                format!(
                    "DECODED hard_errors={} msg={:?}",
                    r.hard_errors,
                    unpack77(&m77)
                )
            }
            None => "no decode".to_string(),
        };
        eprintln!(
            "  freq={:8.2} Hz  dt={:+7.3} s  score={:.3}  -> {}",
            cand.freq_hz, cand.dt_sec, cand.score, outcome
        );
    }
}

/// Precision: nothing beyond the two real signals may be emitted.
///
/// FST4 had no false-decode guard. Real `jt9 -7 -p 60` reports exactly
/// the two decodes below on this recording, and so does this crate —
/// so the budget is 0 with full recall, and any future candidate-
/// selection change that starts inventing signals fails here.
#[test]
fn fst4_wsjtx_sample_precision_vs_reference_decoder() {
    use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};

    static REFERENCE: &[GoldenEntry] = &[
        GoldenEntry::msg("CQ N5TM EL29"),
        GoldenEntry::msg("CQ K9KFR EN71"),
    ];

    let Some(path) = sample_path() else {
        eprintln!("skipping: FST4 golden recording not found");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let out = DecodeRequest::<Fst4s60>::new(&audio, 100.0, 3000.0, 1.2, 50).decode();

    assert_golden(
        &out.results,
        &GoldenSet {
            name: "FST4-60 210115_0058.wav",
            expected: REFERENCE,
            min_hits: 2,
            max_extra: 0,
        },
        Tolerances::default(),
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: None,
        },
    );
}
