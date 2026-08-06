//! WSPR real-world signal validation against
//! `WSJT-X/samples/WSPR/150426_0918.wav` (12 kHz mono, 120 s = 1
//! WSPR slot).
//!
//! Skipped when the WSJT-X tree is absent.

#![cfg(all(feature = "wspr", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

use mfsk_core::wspr::SearchParams;
use mfsk_core::wspr::decode::decode_scan_subtract;

#[allow(dead_code)]
mod common;

use common::load_wav_f32_opt as read_wsjtx_wav_f32;

fn sample_path() -> Option<PathBuf> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").ok()?;
    let p = Path::new(&manifest)
        .join("../../WSJT-X/samples/WSPR/150426_0918.wav")
        .canonicalize()
        .ok()?;
    if p.is_file() { Some(p) } else { None }
}

/// Each golden entry carries the WSPR Type-1 message string in its
/// `Display` form ("call grid pwr") plus the audio carrier in Hz
/// (= wsprd's `freq_MHz × 1e6`).
struct Golden {
    msg: &'static str,
    freq_hz: f32,
    dt_sec: f32,
}

const GOLDEN: &[Golden] = &[
    Golden {
        msg: "ND6P DM04 30",
        freq_hz: 1446.0,
        dt_sec: 1.1,
    },
    Golden {
        msg: "W5BIT EL09 17",
        freq_hz: 1460.0,
        dt_sec: 0.1,
    },
    Golden {
        msg: "WD4LHT EL89 30",
        freq_hz: 1489.0,
        dt_sec: 0.6,
    },
    Golden {
        msg: "NM7J DM26 30",
        freq_hz: 1503.0,
        dt_sec: -0.8,
    },
    Golden {
        msg: "KI7CI DM09 37",
        freq_hz: 1517.0,
        dt_sec: 0.5,
    },
    Golden {
        msg: "DJ6OL JO52 37",
        freq_hz: 1530.0,
        dt_sec: -1.9,
    },
    Golden {
        msg: "W3HH EL89 30",
        freq_hz: 1587.0,
        dt_sec: 0.8,
    },
    Golden {
        msg: "W3BI FN20 30",
        freq_hz: 1594.0,
        dt_sec: 0.7,
    },
];

const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5;

#[test]
fn wspr_wsjtx_sample_recall_vs_golden() {
    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X WSPR sample not found at ../../WSJT-X/samples/WSPR/150426_0918.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_f32(&path).expect("WAV must be 12 kHz mono PCM-16");

    // Golden carriers span 1446..1594 Hz — widen well past the
    // ±100-Hz default to keep edges in scope. WSPR slots commonly
    // hold 6–10 transmissions; weak signals score 0.15–0.4 and
    // get crowded out of the default 16-candidate budget by the
    // strong (0.7+) signals' alternate alignments. Bump candidate
    // count so the weak signals have a chance.
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1620.0,
        max_candidates: 100,
        score_threshold: 0.05,
        ..SearchParams::default()
    };

    let decodes = decode_scan_subtract(&audio, 12_000, 0, &params);

    let decoded: Vec<(String, f32, f32)> = decodes
        .iter()
        .map(|d| (d.message.to_string(), d.freq_hz, d.dt_sec))
        .collect();

    eprintln!("WSPR sample decoded {} message(s):", decoded.len());
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
    eprintln!("recall: {}/{} golden WSPR decodes", hits, GOLDEN.len());

    assert_eq!(
        hits,
        GOLDEN.len(),
        "WSPR WSJT-X sample recall regressed: {}/{}",
        hits,
        GOLDEN.len()
    );
}

/// Ad-hoc wall-clock timing probe for `decode_scan_subtract` on the
/// real WSJT-X golden WAV — same params as
/// `wspr_wsjtx_sample_recall_vs_golden`. Modeled on
/// `bench_qso3_busy_timing.rs`'s warm-up-then-N-iteration structure
/// (perf-review Phase 0, WSPR round — no prior WSPR timing harness
/// existed before this).
///
/// Run: `cargo test --release --features full --test
/// wspr_wsjtx_samples wspr_speed_diag -- --ignored --nocapture`
#[test]
#[ignore = "manual diagnostic — WSPR decode_scan_subtract timing (perf-review Phase 0)"]
fn wspr_speed_diag() {
    use std::time::Instant;

    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X WSPR sample not found at ../../WSJT-X/samples/WSPR/150426_0918.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_f32(&path).expect("WAV must be 12 kHz mono PCM-16");
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1620.0,
        max_candidates: 100,
        score_threshold: 0.05,
        ..SearchParams::default()
    };

    // Warm-up (page faults, allocator, etc.) — excluded from timing.
    let n = decode_scan_subtract(&audio, 12_000, 0, &params).len();
    eprintln!("warm-up: {n} decodes");

    const N_ITERS: usize = 5;
    let mut times = Vec::with_capacity(N_ITERS);
    for i in 0..N_ITERS {
        let t0 = Instant::now();
        let r = decode_scan_subtract(&audio, 12_000, 0, &params);
        let dt = t0.elapsed();
        eprintln!(
            "iter {i:2}: {:8.3} ms  ({} decodes)",
            dt.as_secs_f64() * 1000.0,
            r.len()
        );
        times.push(dt.as_secs_f64() * 1000.0);
    }

    let total: f64 = times.iter().sum();
    let avg = total / times.len() as f64;
    let min = times.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = times.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    eprintln!(
        "\n=== WSPR 150426_0918.wav decode_scan_subtract timing (n={}) ===\navg={avg:.3} ms  min={min:.3} ms  max={max:.3} ms",
        times.len()
    );
}

/// Cost-split diagnostic — where does `decode_scan_subtract`'s
/// wall-clock actually go? Follow-up to `wspr_speed_diag`: that probe
/// found only the FFT-planner-cache item (of 4 tried) showed a real
/// measured win, all ~1-2% of total — implying the dominant cost lives
/// somewhere `wspr_speed_diag` doesn't break out. Replicates
/// `decode_scan`'s exact 2-pass sequence (`decode.rs`) using its own
/// `pub` building blocks, with `Instant` timers around each stage
/// instead of just the end-to-end call.
///
/// Run: `cargo test --release --features full --test wspr_wsjtx_samples
/// wspr_diag_candidate_cost_split -- --ignored --nocapture`
#[test]
#[ignore = "manual diagnostic — WSPR decode cost-split (perf-review follow-up)"]
fn wspr_diag_candidate_cost_split() {
    use std::time::Instant;

    use mfsk_core::wspr::baseband::decimate_to_baseband;
    use mfsk_core::wspr::coarse_baseband::coarse_baseband;
    use mfsk_core::wspr::decode::{decode_at_baseband, decode_at_baseband_nblocks};
    use mfsk_core::wspr::encode_channel_symbols;
    use mfsk_core::wspr::subtract::subtract_signal_baseband;

    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X WSPR sample not found at ../../WSJT-X/samples/WSPR/150426_0918.wav"
        );
        return;
    };
    let audio = read_wsjtx_wav_f32(&path).expect("WAV must be 12 kHz mono PCM-16");
    let params = SearchParams {
        freq_min_hz: 1400.0,
        freq_max_hz: 1620.0,
        max_candidates: 100,
        score_threshold: 0.05,
        ..SearchParams::default()
    };
    let sample_rate = 12_000u32;
    let max_drift = 4i32;

    // Mirror decode_scan's own NEGATIVE_DT_PAD_SEC padding exactly so
    // candidate counts/alignment match the production path.
    let pad = (3.0 * sample_rate as f32) as usize;
    let mut padded = vec![0.0f32; pad + audio.len()];
    padded[pad..].copy_from_slice(&audio);

    let t0 = Instant::now();
    let (mut idat, mut qdat) = decimate_to_baseband(&padded);
    let t_decimate = t0.elapsed();

    let t0 = Instant::now();
    let mut cands1 = coarse_baseband(&idat, &qdat, pad, params.max_candidates, max_drift);
    cands1.truncate(params.max_candidates);
    let t_coarse1 = t0.elapsed();
    let n_cand1 = cands1.len();

    let t0 = Instant::now();
    let mut pass1: Vec<(mfsk_core::wspr::WsprResult, usize)> = Vec::new();
    for c in &cands1 {
        if let Some(mut d) =
            decode_at_baseband(&idat, &qdat, sample_rate, c.start_sample, c.freq_hz, 0.0)
        {
            let start_refined = d.start_sample;
            d.start_sample = start_refined.saturating_sub(pad);
            pass1.push((d, start_refined));
        }
    }
    let t_pass1 = t0.elapsed();
    let n_pass1_decoded = pass1.len();

    let t0 = Instant::now();
    for (d, start_refined) in &pass1 {
        let symbols = encode_channel_symbols(&d.info_bits);
        let f0_audio = d.freq_hz + 1.5 * mfsk_core::wspr::demod::TONE_SPACING_HZ;
        let shift_baseband = (*start_refined as i32) / 32;
        subtract_signal_baseband(
            &mut idat,
            &mut qdat,
            f0_audio,
            shift_baseband,
            0.0,
            &symbols,
        );
    }
    let t_subtract = t0.elapsed();

    let t0 = Instant::now();
    let cands2 = coarse_baseband(&idat, &qdat, pad, params.max_candidates, max_drift);
    let t_coarse2 = t0.elapsed();
    let n_cand2 = cands2.len();

    let t0 = Instant::now();
    let mut n_pass2_decoded = 0;
    for c in &cands2 {
        if decode_at_baseband_nblocks(
            &idat,
            &qdat,
            sample_rate,
            c.start_sample,
            c.freq_hz,
            c.drift_hz,
            &[1, 2, 3],
        )
        .is_some()
        {
            n_pass2_decoded += 1;
        }
    }
    let t_pass2 = t0.elapsed();

    let per_cand1_us = t_pass1.as_secs_f64() * 1e6 / n_cand1.max(1) as f64;
    let per_cand2_us = t_pass2.as_secs_f64() * 1e6 / n_cand2.max(1) as f64;

    eprintln!(
        "\n=== WSPR 150426_0918.wav decode_scan_subtract cost split ===\n\
         decimate_to_baseband      = {:8.1} ms\n\
         coarse_baseband (pass 1)  = {:8.1} ms  ({} candidates)\n\
         pass-1 refine+Fano total  = {:8.1} ms  ({:.1} µs/cand, {} decoded, nblocks=[1])\n\
         subtract_signal_baseband  = {:8.1} ms  ({} decodes subtracted)\n\
         coarse_baseband (pass 2)  = {:8.1} ms  ({} candidates)\n\
         pass-2 refine+Fano total  = {:8.1} ms  ({:.1} µs/cand, {} decoded, nblocks=[1,2,3])\n\
         ------------------------------------------------\n\
         sum of stages above       = {:8.1} ms",
        t_decimate.as_secs_f64() * 1000.0,
        t_coarse1.as_secs_f64() * 1000.0,
        n_cand1,
        t_pass1.as_secs_f64() * 1000.0,
        per_cand1_us,
        n_pass1_decoded,
        t_subtract.as_secs_f64() * 1000.0,
        pass1.len(),
        t_coarse2.as_secs_f64() * 1000.0,
        n_cand2,
        t_pass2.as_secs_f64() * 1000.0,
        per_cand2_us,
        n_pass2_decoded,
        (t_decimate + t_coarse1 + t_pass1 + t_subtract + t_coarse2 + t_pass2).as_secs_f64()
            * 1000.0,
    );
}
