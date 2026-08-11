//! WSPR real-world signal validation against
//! `WSJT-X/samples/WSPR/150426_0918.wav` (12 kHz mono, 120 s = 1
//! WSPR slot).
//!
//! Skipped when the WSJT-X tree is absent.

#![cfg(all(feature = "wspr", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::PathBuf;

use mfsk_core::wspr::SearchParams;
use mfsk_core::wspr::WsprResult;
use mfsk_core::wspr::decode::{
    decode_scan, decode_scan_streaming, decode_scan_subtract, decode_scan_subtract_streaming,
};

#[allow(dead_code)]
mod common;

use common::load_wav_f32_opt as read_wsjtx_wav_f32;

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream("wspr/150426_0918.wav", Some("WSPR/150426_0918.wav"))
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
    // wsprd's 9th spot on this file: `0918 -23 2.2 10.140165 0
    // G8VDQ IO91 37` at dial 10.1387 MHz, i.e. 1465.0 Hz audio. It was
    // absent from this table only because we could not reach it; the
    // reference decoder always could.
    Golden {
        msg: "G8VDQ IO91 37",
        freq_hz: 1465.0,
        dt_sec: 2.2,
    },
];

const FREQ_TOL_HZ: f32 = 4.0;
const DT_TOL_SEC: f32 = 0.5;

/// The recall test's audio + params, shared by the phantom and
/// carried-table tests so all three measure the same configuration.
fn sample_and_params() -> Option<(Vec<f32>, SearchParams)> {
    let path = sample_path()?;
    let audio = read_wsjtx_wav_f32(&path)?;
    Some((
        audio,
        SearchParams {
            freq_min_hz: 1400.0,
            freq_max_hz: 1620.0,
            max_candidates: 100,
            ..SearchParams::default()
        },
    ))
}

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

    // All 9 — every spot `wsprd` reports on this file — from a single
    // isolated slot and without a carried table.
    //
    // This was 7 for a long time, and the two it was missing each had a
    // different cause:
    //
    // `W3BI FN20 30` (-25 dB) was reachable only by OSD, and OSD is
    // gated the way `wsprd.c:1396` gates it — accepted only for a
    // callsign an earlier Fano decode confirmed (`WsprCallsignTable`) —
    // so within one slot nothing confirmed it. (The gate is not
    // negotiable: the `nhardmin <= 44` threshold it replaced recovered
    // W3BI *and* admitted 8 phantoms on this same recording, a 50%
    // false-decode rate reported from live operation. W3BI landed at
    // `nhardmin = 39`, the phantoms at 40/40/40/41/41/41/42/42 — not
    // separable.) It is now a plain Fano decode and the gate never
    // comes up.
    //
    // `G8VDQ IO91 37` (-23 dB) needed the subtraction of the *other*
    // signals to be right: we were reconstructing every replica as
    // stationary while decoding it with a drift estimate, so what got
    // removed did not match what was there, and the residue sat on top
    // of G8VDQ.
    //
    // `wspr_carried_table_does_not_perturb_a_fully_fano_slot` still
    // covers the cross-slot table path; it just no longer has an
    // OSD-only decode to use as its example.
    const EXPECTED_RECALL: usize = 9;
    assert_eq!(
        hits,
        EXPECTED_RECALL,
        "WSPR WSJT-X sample recall changed: {}/{} (expected {})",
        hits,
        GOLDEN.len(),
        EXPECTED_RECALL
    );
}

/// The decoder must emit **nothing beyond** the real signals.
///
/// WSPR has no CRC, so this is the property that actually matters in
/// operation and the one that regressed: before the OSD gate was
/// fixed this file produced 16 decodes — 8 real, 8 phantom, with
/// callsigns like `UZC/7D0DKY` and `05S/C30EQG`. Real `wsprd` reports
/// 9 real and 0 phantom on the same audio.
#[test]
fn wspr_wsjtx_sample_has_no_phantom_decodes() {
    let Some((audio, params)) = sample_and_params() else {
        return;
    };
    let decodes = decode_scan(&audio, 12_000, 0, &params);
    let phantoms: Vec<String> = decodes
        .iter()
        .map(|d| d.message.to_string())
        .filter(|m| !GOLDEN.iter().any(|g| g.msg == m))
        .collect();
    assert!(
        phantoms.is_empty(),
        "WSPR emitted {} phantom decode(s) on an 8-signal recording: {phantoms:?}",
        phantoms.len()
    );
}

/// Carrying a cross-slot table must never change what this recording
/// yields, and must never admit a phantom.
///
/// This test used to demonstrate the OSD gate positively: `W3BI FN20 30`
/// was reachable only by OSD, so seeding the table with W3BI recovered
/// exactly one extra decode and seeding a bogus callsign recovered
/// none. That demonstration is gone — not because the gate changed, but
/// because W3BI is now a plain Fano decode (see
/// `wspr_wsjtx_sample_recall_vs_golden`), so this recording no longer
/// contains an OSD-only signal to demonstrate with.
///
/// What remains testable here is the safety half: the table must not
/// perturb a slot whose signals Fano already reaches. The gate's own
/// admit/reject logic is covered directly by
/// `wspr::decode::callsign_table_tests` in the library, which does not
/// depend on any particular recording having an OSD-only signal in it.
#[test]
fn wspr_carried_table_does_not_perturb_a_fully_fano_slot() {
    use mfsk_core::wspr::decode::{WsprCallsignTable, decode_scan_with_table};

    let Some((audio, params)) = sample_and_params() else {
        return;
    };

    let baseline: Vec<String> = mfsk_core::wspr::decode::decode_scan(&audio, 12_000, 0, &params)
        .iter()
        .map(|d| d.message.to_string())
        .collect();

    // A table naming a station that is *not* in this recording must not
    // open the gate for anything.
    let mut bogus = WsprCallsignTable::new();
    bogus.record(&mfsk_core::msg::WsprMessage::Type1 {
        callsign: "ZZ9ZZZ".into(),
        grid: "AA00".into(),
        power_dbm: 30,
    });
    let with_bogus: Vec<String> = decode_scan_with_table(&audio, 12_000, 0, &params, &mut bogus)
        .iter()
        .map(|d| d.message.to_string())
        .collect();

    // A table naming a station that *is* here must not change it either
    // — W3BI needs no help any more.
    let mut known = WsprCallsignTable::new();
    known.record(&mfsk_core::msg::WsprMessage::Type1 {
        callsign: "W3BI".into(),
        grid: "FN20".into(),
        power_dbm: 30,
    });
    let with_known: Vec<String> = decode_scan_with_table(&audio, 12_000, 0, &params, &mut known)
        .iter()
        .map(|d| d.message.to_string())
        .collect();

    assert_eq!(
        with_bogus, baseline,
        "seeding an absent callsign changed the decode set — the OSD gate \
         is not actually keyed on the table contents"
    );
    assert_eq!(
        with_known, baseline,
        "seeding a present callsign changed the decode set"
    );

    for (label, msgs) in [("bogus", &with_bogus), ("known", &with_known)] {
        let phantoms: Vec<&String> = msgs
            .iter()
            .filter(|m| !GOLDEN.iter().any(|g| g.msg == m.as_str()))
            .collect();
        assert!(
            phantoms.is_empty(),
            "carried table ({label}) admitted phantom decode(s): {phantoms:?}"
        );
    }
}

/// `decode_scan_subtract_streaming`'s `on_result` callback — real-
/// signal verification, mirroring `tests/ft8_streaming_decode.rs`'s
/// sequential-exact-match contract. `decode_scan_subtract`'s own
/// SIC-pass dedup-then-push loop is sequential (each pass's
/// `decode_scan` call is internally parallel, but its results are
/// *not* streamed — only `decode_scan_subtract`'s own outer
/// accept point fires `on_result`, see that function's doc comment),
/// so the callback-delivered set must exactly equal the batch `Vec`.
#[test]
fn wspr_scan_subtract_streaming_matches_batch_exactly() {
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

    let streamed: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
    let on_result = |r: &WsprResult| {
        streamed.lock().unwrap().push(r.message.to_string());
    };
    let batch = decode_scan_subtract_streaming(&audio, 12_000, 0, &params, &on_result);

    let batch_msgs: Vec<String> = batch.iter().map(|d| d.message.to_string()).collect();
    let streamed = streamed.into_inner().unwrap();
    eprintln!(
        "WSPR decode_scan_subtract_streaming: batch={} streamed={}",
        batch_msgs.len(),
        streamed.len()
    );
    assert_eq!(
        streamed, batch_msgs,
        "WSPR decode_scan_subtract_streaming: streamed callback deliveries must \
         exactly match the batch result, same order (sequential outer SIC \
         accept point, no divergence mechanism exists on this path)"
    );
    assert!(
        !batch.is_empty(),
        "expected real decodes on the WSPR golden WAV"
    );
}

/// `decode_scan_streaming`'s `on_result` callback — real-signal
/// verification of the *parallel* strategy's superset/possible-
/// duplicate contract (both pass 1 and pass 2's per-candidate decode
/// step run under `rayon::par_iter()`, same shape as FT8's default
/// single-pass strategy). On this golden WAV the parallel path
/// happens not to hit the documented same-message-two-candidates
/// duplicate case (verified equal, not just superset, below) — the
/// assertion is a superset check because that's the documented
/// contract, not because equality is expected to fail here (mirrors
/// `ft8_streaming_single_pass_superset_of_batch`'s own reasoning).
#[test]
fn wspr_scan_streaming_superset_of_batch() {
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

    let streamed: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
    let on_result = |r: &WsprResult| {
        streamed.lock().unwrap().push(r.message.to_string());
    };
    let batch = decode_scan_streaming(&audio, 12_000, 0, &params, &on_result);

    let batch_msgs: std::collections::BTreeSet<String> =
        batch.iter().map(|d| d.message.to_string()).collect();
    let streamed: std::collections::BTreeSet<String> =
        streamed.into_inner().unwrap().into_iter().collect();
    eprintln!(
        "WSPR decode_scan_streaming: batch={} streamed={}",
        batch_msgs.len(),
        streamed.len()
    );

    let missing_from_stream: Vec<&String> = batch_msgs.difference(&streamed).collect();
    assert!(
        missing_from_stream.is_empty(),
        "every batch result must have fired via callback at least once: missing {:?}",
        missing_from_stream
    );
    assert!(
        !batch_msgs.is_empty(),
        "expected real decodes on the WSPR golden WAV"
    );
    assert_eq!(
        streamed, batch_msgs,
        "no duplicate-delivery case expected on this golden WAV (if this starts \
         failing, the superset contract is what's guaranteed, not this equality)"
    );

    // Also confirm decode_scan_streaming's own return matches plain
    // decode_scan's (the streaming sibling must not change behavior).
    let plain = decode_scan(&audio, 12_000, 0, &params);
    let plain_msgs: std::collections::BTreeSet<String> =
        plain.iter().map(|d| d.message.to_string()).collect();
    assert_eq!(
        batch_msgs, plain_msgs,
        "decode_scan_streaming's returned Vec must match plain decode_scan's"
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

/// Pass-nesting ablation — `WSPR_BENCHMARK.md`'s Option C question:
/// does the *outer* `NPASSES=2` SIC (`decode_scan_subtract`, 12 kHz)
/// earn its ~2x cost over the *inner* 2-pass SIC (`decode_scan` itself,
/// 375 Hz baseband) it wraps, or is one layer doing all the real work?
///
/// Three conditions on the real WSJT-X golden WAV, all using `pub`
/// building blocks (no source changes needed — `decode_scan`
/// *is* "inner=1+2, outer=1" already; the outer ablation needs no
/// separate helper since a single `decode_scan` call already skips
/// the outer loop entirely):
/// - **A**: inner pass-1 only, outer=1 — cheapest possible baseline.
/// - **B**: inner pass-1+2, outer=1 (`decode_scan` called directly).
/// - **D**: inner pass-1+2, outer=2 (`decode_scan_subtract`, the
///   current production default).
///
/// `B \ A` isolates what the *inner* pass 2 contributes; `D \ B`
/// isolates what the *outer* pass 2 contributes. (Condition C — inner=1
/// only, outer=2 — is skipped: it needs the private `WSPR_SUBTRACT`
/// LPF config to replicate the outer loop without `decode_scan`'s own
/// inner pass 2, and A vs B / B vs D already answer the question this
/// document needs answered without it.)
///
/// Run: `cargo test --release --features full --test wspr_wsjtx_samples
/// wspr_diag_pass_ablation -- --ignored --nocapture`
#[test]
#[ignore = "manual diagnostic — WSPR outer/inner 2-pass ablation (WSPR_BENCHMARK.md Option C)"]
fn wspr_diag_pass_ablation() {
    use std::time::Instant;

    use mfsk_core::wspr::WsprResult;
    use mfsk_core::wspr::baseband::decimate_to_baseband;
    use mfsk_core::wspr::coarse_baseband::coarse_baseband;
    use mfsk_core::wspr::decode::{decode_at_baseband, decode_scan, decode_scan_subtract};

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
    const FREQ_DEDUP_HZ: f32 = 5.0;
    const TIME_DEDUP_SAMPLES: i64 = 8192;

    // Condition A: inner pass-1 only, outer=1. Mirrors
    // wspr_diag_candidate_cost_split's first half (decimate + coarse
    // + pass-1 loop), stopping before subtract/re-coarse/pass-2.
    let pad = (3.0 * sample_rate as f32) as usize;
    let mut padded = vec![0.0f32; pad + audio.len()];
    padded[pad..].copy_from_slice(&audio);

    let t0 = Instant::now();
    let (idat, qdat) = decimate_to_baseband(&padded);
    let mut cands = coarse_baseband(&idat, &qdat, pad, params.max_candidates, max_drift);
    cands.truncate(params.max_candidates);
    let mut a_results: Vec<WsprResult> = Vec::new();
    for c in &cands {
        let Some(mut d) =
            decode_at_baseband(&idat, &qdat, sample_rate, c.start_sample, c.freq_hz, 0.0)
        else {
            continue;
        };
        let start_refined = d.start_sample;
        d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32 - 1.0;
        d.start_sample = start_refined.saturating_sub(pad);
        d.snr_db = c.snr_db;
        let dup = a_results.iter().any(|prev| {
            prev.message == d.message
                && (prev.freq_hz - d.freq_hz).abs() <= FREQ_DEDUP_HZ
                && (prev.start_sample as i64 - d.start_sample as i64).abs() <= TIME_DEDUP_SAMPLES
        });
        if !dup {
            a_results.push(d);
        }
    }
    let t_a = t0.elapsed();

    // Condition B: inner pass-1+2, outer=1.
    let t0 = Instant::now();
    let b_results = decode_scan(&audio, sample_rate, 0, &params);
    let t_b = t0.elapsed();

    // Condition D: inner pass-1+2, outer=2 (production default).
    let t0 = Instant::now();
    let d_results = decode_scan_subtract(&audio, sample_rate, 0, &params);
    let t_d = t0.elapsed();

    let hits = |results: &[WsprResult]| -> Vec<&'static str> {
        GOLDEN
            .iter()
            .filter(|g| {
                results.iter().any(|r| {
                    r.message.to_string() == g.msg
                        && (r.freq_hz - g.freq_hz).abs() <= FREQ_TOL_HZ
                        && (r.dt_sec - g.dt_sec).abs() <= DT_TOL_SEC
                })
            })
            .map(|g| g.msg)
            .collect()
    };
    let a_hits = hits(&a_results);
    let b_hits = hits(&b_results);
    let d_hits = hits(&d_results);

    eprintln!("\n=== WSPR 150426_0918.wav pass-nesting ablation ===");
    eprintln!(
        "A inner=1only outer=1        {:8.1} ms  {}/{} golden  {:?}",
        t_a.as_secs_f64() * 1000.0,
        a_hits.len(),
        GOLDEN.len(),
        a_hits
    );
    eprintln!(
        "B inner=1+2   outer=1        {:8.1} ms  {}/{} golden  {:?}",
        t_b.as_secs_f64() * 1000.0,
        b_hits.len(),
        GOLDEN.len(),
        b_hits
    );
    eprintln!(
        "D inner=1+2   outer=2 (prod) {:8.1} ms  {}/{} golden  {:?}",
        t_d.as_secs_f64() * 1000.0,
        d_hits.len(),
        GOLDEN.len(),
        d_hits
    );
    eprintln!(
        "\ninner pass-2 contributes (B \\ A): {:?}",
        b_hits
            .iter()
            .filter(|m| !a_hits.contains(m))
            .collect::<Vec<_>>()
    );
    eprintln!(
        "outer pass-2 contributes (D \\ B): {:?}",
        d_hits
            .iter()
            .filter(|m| !b_hits.contains(m))
            .collect::<Vec<_>>()
    );
}

/// Carrying a callsign table **across slots** must not manufacture
/// decodes.
///
/// This is the flow `decode_scan_with_table` exists for — a WSPR
/// receiver hears the same beacons every 2 minutes, so a station
/// confirmed by Fano in one slot stays OSD-recoverable in later ones.
/// The existing `wspr_carried_table_recovers_osd_only_decode` seeds
/// the table by hand and so tests the *gate*; this tests the *flow*,
/// where the table is built by decoding and then fed forward.
///
/// The hazard being guarded is specific and real: OSD synthesises a
/// valid codeword for any input, and the gate lets a result through
/// whenever its callsign is already in the table. Every slot that
/// carries the table forward therefore widens the set of callsigns
/// OSD is permitted to emit, so a bug here would show up as phantoms
/// that *accumulate with slot count* — invisible to any single-slot
/// test.
///
/// Repeating one recording is a fair model of that: the same
/// callsigns recur, which is exactly the real condition, and it holds
/// the signal content constant so any growth in the output is
/// attributable to the table rather than to new audio.
///
/// (A synthetic two-slot version — strong slot populates the table,
/// weak slot is recovered from it — was tried and abandoned: across
/// -25…-27 dB and six noise seeds, a clean single-signal slot never
/// produced a case where the table added a decode. OSD's advantage on
/// the real recording comes from the pass-2 environment after eight
/// stronger signals have been subtracted, which a lone synthetic tone
/// cannot reproduce. Recording that here so the absence is not
/// mistaken for an untried idea.)
#[test]
fn wspr_carried_table_across_slots_adds_no_phantoms() {
    use mfsk_core::wspr::decode::{WsprCallsignTable, decode_scan_with_table};

    let Some((audio, params)) = sample_and_params() else {
        return;
    };

    let mut table = WsprCallsignTable::new();
    let mut per_slot: Vec<(usize, Vec<String>)> = Vec::new();

    // Five slots of the same beacons, table fed forward each time.
    for slot in 0..5 {
        let d = decode_scan_with_table(&audio, 12_000, 0, &params, &mut table);
        let msgs: Vec<String> = d.iter().map(|r| r.message.to_string()).collect();
        let phantoms: Vec<&String> = msgs
            .iter()
            .filter(|m| !GOLDEN.iter().any(|g| g.msg == m.as_str()))
            .collect();
        eprintln!(
            "  slot {slot}: {} decode(s), {} phantom(s)",
            msgs.len(),
            phantoms.len()
        );
        assert!(
            phantoms.is_empty(),
            "slot {slot} of a carried-table run emitted phantom(s): {phantoms:?}. \
             The table grows every slot, so this is the direction the OSD gate \
             fails in — a single-slot test cannot see it."
        );
        per_slot.push((msgs.len(), msgs));
    }

    // Decode count must not grow with slot count. Growth would mean
    // the table is admitting results it did not admit before, which
    // on identical audio can only be table-driven.
    let first = per_slot[0].0;
    for (i, (n, _)) in per_slot.iter().enumerate() {
        assert_eq!(
            *n, first,
            "slot {i} returned {n} decodes vs slot 0's {first} on identical audio — \
             the carried table is changing the outcome, which it must only ever do \
             by recovering a *known* callsign, not by finding new ones"
        );
    }
}
