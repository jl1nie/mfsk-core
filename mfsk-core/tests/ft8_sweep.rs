//! FT8 SNR sweep + fading benchmark against `ft8sim`-generated signals.
//!
//! This test is `#[ignore]` — run it manually when investigating FT8
//! sensitivity. What this sweep gives FT8: a true Watterson-fading
//! AWGN/CCIR corpus generated from WSJT-X's own `ft8sim`, as opposed to the
//! existing CI "ft8 characterization" suite (`ft8_decode_block_snr_sweep`
//! and friends), which is homegrown LCG-noise synthesis — not validated
//! against any WSJT-X-native ground truth.
//!
//! **Update (2026-08-10, issue #253)**: the claim that used to sit here —
//! that FT8's OSD gate isn't reachable from an external probe because
//! `process_candidate` isn't `pub` — was wrong for the `DecodeRequest`
//! entry point specifically: `.strictness(s)` on `DecodeRequest<Ft8>`
//! reaches the exact same shared gate (`DecodeStrictness::ft8_nharderrors_max`,
//! called from `ft8::decode_block::process_candidates`/`osd_strategy`,
//! which `ft8::decode::decode_frame_inner` also routes through). See
//! the sweep's `MFSK_FT8_SWEEP_STRICTNESS` / `_STRATEGY` knobs below (they replaced
//! a manual `ft8_strictness_probe`), added after a reproducible false decode
//! (`7Y8CIH HN1GD OP30` on `qso3_busy.wav` via WebFT8's `Deep` +
//! `.sic_early()` phase-2 pipeline, `hard_errors=31` under `Deep`'s
//! `ft8_nharderrors_max=40` — a ceiling the type's own doc comment already
//! flagged as "not yet swept against a fading corpus").
//!
//! ```sh
//! # 1. Generate WAVs (once, or when widening the SNR grid):
//! scripts/build_ft8sim.sh
//! scripts/gen_ft8_sweep_wavs.sh
//!
//! # 2. Run the sweep:
//! cargo test --test ft8_sweep --release --features ft8,fft-rustfft,parallel,uvpacket \
//!   -- --ignored --nocapture
//! ```
//!
//! (`uvpacket` is only required because `tests/common/channel.rs`, pulled in
//! via `mod common`, unconditionally imports `mfsk_core::uvpacket` — unrelated
//! to FT8 itself. `MFSK_FT8_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/ft8_sweep`, relative to `CARGO_MANIFEST_DIR`.)
//!
//! Output is a recall table — no assertions, statistics only. Set
//! `MFSK_FT8_SWEEP_CSV=/path/out.csv` to also dump raw per-trial pass/fail
//! rows for bootstrap-CI analysis of the 50%-crossing estimate. See
//! `docs/notes/FST4_BENCHMARK.md` for the shared methodology this mirrors —
//! FT8 has no sub-modes, so there's one grid instead of one per T/R period.

#![cfg(all(feature = "ft8", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::{load_wav_i16_opt, parse_snr_tag};
use mfsk_core::msg::wsjt77::unpack77;

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;
const DT_TOL_SEC: f32 = 0.6;

fn sweep_dir() -> PathBuf {
    common::sweep_dir("MFSK_FT8_SWEEP_DIR", "ft8_sweep")
}

// ── Channel conditions (must match gen_ft8_sweep_wavs.sh CHANNELS) ─────────
#[allow(dead_code)]
const CHANNELS: &[&str] = &["awgn", "ccir_good", "ccir_moderate", "ccir_poor"];

/// Which decode strategy the sweep drives, from `MFSK_FT8_SWEEP_STRATEGY`:
/// `single` (default), `sic_early` or `sic_rounds`. The phantom-prone code
/// lives in the non-default strategies (`CONTRIBUTING.md`: #243 was in
/// `__staged_sic`, #253 in `.sic_early()`), so a precision baseline that
/// only covered `decode()` would test the path least likely to break;
/// `run-sensitivity-sweeps.sh` also runs `sic_early` into its own CSV.
enum Strategy {
    Single,
    SicEarly,
    SicRounds,
}

fn strategy_from_env() -> Strategy {
    match std::env::var("MFSK_FT8_SWEEP_STRATEGY").as_deref() {
        Ok("sic_early") => Strategy::SicEarly,
        Ok("sic_rounds") => Strategy::SicRounds,
        Ok("single") | Err(_) => Strategy::Single,
        Ok(other) => {
            panic!("MFSK_FT8_SWEEP_STRATEGY={other}: expected single|sic_early|sic_rounds")
        }
    }
}

/// `MFSK_FT8_SWEEP_STRICTNESS=strict|normal|deep`; unset leaves the
/// request's default.
fn strictness_from_env() -> Option<mfsk_core::engine::pipeline::DecodeStrictness> {
    use mfsk_core::engine::pipeline::DecodeStrictness;
    match std::env::var("MFSK_FT8_SWEEP_STRICTNESS").as_deref() {
        Ok("strict") => Some(DecodeStrictness::Strict),
        Ok("normal") => Some(DecodeStrictness::Normal),
        Ok("deep") => Some(DecodeStrictness::Deep),
        Err(_) => None,
        Ok(other) => panic!("MFSK_FT8_SWEEP_STRICTNESS={other}: expected strict|normal|deep"),
    }
}

/// `(pass, extra)`: whether the injected message came out at the right
/// frequency and time, and how many *other* distinct messages came out
/// (see `common::distinct_extras`).
/// `MFSK_FT8_SWEEP_AP_HINT=<call1>,<call2>` passes those two callsigns as the request's
/// `ap_hint` (either may be empty), the operator's a-priori knowledge of a QSO. The
/// corpus message is `CQ JL1NIE PM95`, so `CQ,JL1NIE` is the right hint (a heavy lock)
/// and any other pair a wrong one, which is what tells whether locking bits the signal
/// does not have lets phantoms through (#456). Unset, the sweep has no hint, as always.
fn ap_hint_requested() -> Option<mfsk_core::msg::ap::ApHint> {
    let v = std::env::var("MFSK_FT8_SWEEP_AP_HINT").ok()?;
    let mut parts = v.splitn(2, ',');
    let (c1, c2) = (
        parts.next().unwrap_or("").trim(),
        parts.next().unwrap_or("").trim(),
    );
    let mut h = mfsk_core::msg::ap::ApHint::new();
    if !c1.is_empty() {
        h = h.with_call1(c1);
    }
    if !c2.is_empty() {
        h = h.with_call2(c2);
    }
    Some(h)
}

/// `MFSK_FT8_SWEEP_FREQ_HINT=<Hz>` passes that frequency as the request's `freq_hint`, the
/// operator's QSO frequency. Every signal in this corpus is at 1500 Hz, so `1500` is at it
/// and anything 50 Hz or more away (say `900`) is not: `ft8b.f90` tries the heavy AP
/// hypotheses (MyCall and DxCall locked) only within `napwid` of it (#456). Unset, no hint.
fn freq_hint_requested() -> Option<f32> {
    std::env::var("MFSK_FT8_SWEEP_FREQ_HINT")
        .ok()
        .and_then(|v| v.trim().parse().ok())
}

fn decode_wav_ft8(audio: &[i16]) -> (bool, u32) {
    use mfsk_core::ft8::Ft8;

    use mfsk_core::msg::decode_request::DecodeRequest;
    let hint = ap_hint_requested();
    let mut req = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50);
    if let Some(f) = freq_hint_requested() {
        req = req.freq_hint(f);
    }
    if let Some(h) = hint.as_ref() {
        req = req.ap_hint(h);
    }
    if let Some(level) = strictness_from_env() {
        req = req.strictness(level);
    }
    let results = match strategy_from_env() {
        Strategy::Single => req.decode().results,
        Strategy::SicEarly => req.sic_early().decode().results,
        Strategy::SicRounds => req.sic_rounds(3).decode().results,
    };
    let pass = results.iter().any(|d| {
        unpack77(d.message77()).as_deref() == Some(GOLDEN_MSG)
            && (d.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
            && d.dt_sec.abs() <= DT_TOL_SEC
    });
    let texts: Vec<String> = results
        .iter()
        .filter_map(|d| unpack77(d.message77()))
        .collect();
    (pass, common::distinct_extras(&texts, GOLDEN_MSG))
}

// ── Filename parsing ─────────────────────────────────────────────────────────

/// Parse `ft8_<channel>_<snr_tag>_<trial>.wav`.
/// snr_tag: `m05` = -5, `p05` = +5.
struct WavMeta {
    channel: String,
    snr_db: i32,
    trial: u32,
    path: PathBuf,
}

fn collect_wavs(dir: &Path) -> Vec<WavMeta> {
    let mut out = Vec::new();
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return out,
    };
    for entry in entries.flatten() {
        let path = entry.path();
        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_string();
        // ft8_awgn_m05_01
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() < 4 || parts[0] != "ft8" {
            continue;
        }
        let trial: u32 = match parts.last().and_then(|s| s.parse().ok()) {
            Some(v) => v,
            None => continue,
        };
        let snr_tag = parts[parts.len() - 2];
        let snr_db = match parse_snr_tag(snr_tag) {
            Some(v) => v,
            None => continue,
        };
        // channel = everything between "ft8" and snr_tag
        let channel = parts[1..parts.len() - 2].join("_");
        out.push(WavMeta {
            channel,
            snr_db,
            trial,
            path,
        });
    }
    // Sort: channel → snr desc → trial
    out.sort_by_key(|m| (m.channel.clone(), std::cmp::Reverse(m.snr_db), m.trial));
    out
}

// ── Main sweep test ──────────────────────────────────────────────────────────

#[test]
#[ignore = "manual pre-merge benchmark — run with --ignored --nocapture"]
fn ft8_snr_sweep() {
    let dir = sweep_dir();
    let all_wavs = collect_wavs(&dir);

    if all_wavs.is_empty() {
        eprintln!(
            "No WAVs found in {:?}\n\
             Run: scripts/build_ft8sim.sh && scripts/gen_ft8_sweep_wavs.sh",
            dir
        );
        return;
    }

    // Optional env-var filters — narrow the sweep to the region of interest.
    // MFSK_FT8_SWEEP_CHANNELS=awgn       (comma-separated channel names)
    // MFSK_FT8_SWEEP_SNR_MIN=-24         (inclusive lower bound, dB)
    // MFSK_FT8_SWEEP_SNR_MAX=-17         (inclusive upper bound, dB)
    // MFSK_FT8_SWEEP_CSV=/path/out.csv   (optional: dump raw per-trial
    //   pass/fail rows — channel,snr_db,trial,pass,extra; `extra` is the
    //   number of decoded messages that were not the injected one)
    let chan_filter: Option<Vec<String>> = std::env::var("MFSK_FT8_SWEEP_CHANNELS")
        .ok()
        .map(|s| s.split(',').map(|v| v.trim().to_string()).collect());
    let snr_min: Option<i32> = std::env::var("MFSK_FT8_SWEEP_SNR_MIN")
        .ok()
        .and_then(|s| s.trim().parse().ok());
    let snr_max: Option<i32> = std::env::var("MFSK_FT8_SWEEP_SNR_MAX")
        .ok()
        .and_then(|s| s.trim().parse().ok());

    let wavs: Vec<WavMeta> = all_wavs
        .into_iter()
        .filter(|w| {
            chan_filter
                .as_ref()
                .is_none_or(|f| f.iter().any(|c| c == &w.channel))
        })
        .filter(|w| snr_min.is_none_or(|m| w.snr_db >= m))
        .filter(|w| snr_max.is_none_or(|m| w.snr_db <= m))
        .collect();

    eprintln!("\n{:-<64}", "");
    eprintln!(
        "  {:<14} {:>7}   {:>6}  {:<22}       Extra",
        "Channel", "SNR(dB)", "Recall", "Bar"
    );
    eprintln!("{:-<64}", "");

    let mut csv = common::sweep_csv_writer("MFSK_FT8_SWEEP_CSV", "channel,snr_db,trial,pass,extra");

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;
    use std::collections::BTreeMap;
    use std::io::Write;

    let mut groups: BTreeMap<(String, i32), Vec<&WavMeta>> = BTreeMap::new();
    for wav in &wavs {
        groups
            .entry((wav.channel.clone(), wav.snr_db))
            .or_default()
            .push(wav);
    }

    let mut last_chan: Option<String> = None;
    for ((chan, snr), wav_group) in &groups {
        #[cfg(feature = "parallel")]
        let results: Vec<(u32, (bool, u32))> = wav_group
            .par_iter()
            .filter_map(|wav| {
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_wav_ft8(&audio)))
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let results: Vec<(u32, (bool, u32))> = wav_group
            .iter()
            .filter_map(|wav| {
                load_wav_i16_opt(&wav.path).map(|audio| (wav.trial, decode_wav_ft8(&audio)))
            })
            .collect();

        let trials = results.len() as u32;
        if trials == 0 {
            continue;
        }
        let hits = results.iter().filter(|&&(_, (h, _))| h).count() as u32;
        let extras: u32 = results.iter().map(|&(_, (_, e))| e).sum();

        if let Some(f) = csv.as_mut() {
            for &(trial, (pass, extra)) in &results {
                writeln!(f, "{chan},{snr},{trial},{},{extra}", pass as u8).unwrap();
            }
        }

        if Some(chan) != last_chan.as_ref() {
            eprintln!("{:-<64}", "");
            last_chan = Some(chan.clone());
        }
        let pct = hits as f32 / trials as f32 * 100.0;
        let bar_len = (hits as usize * 20).div_ceil(trials as usize);
        let bar = format!("{}{}", "#".repeat(bar_len), ".".repeat(20 - bar_len));
        eprintln!(
            "  {:<14}  {:>4} dB   {:>2}/{:<2}  [{}]  {:4.0}%   {:>3}",
            chan, snr, hits, trials, bar, pct, extras
        );
    }
    eprintln!("{:-<64}", "");
    eprintln!(
        "\nChannels (ITU-R Watterson): awgn=no fading | \
         ccir_good=fdop 0.1Hz/del 0.5ms | \
         ccir_moderate=0.5/1.0 | ccir_poor=1.0/2.0"
    );
}

/// Per-trial stage attribution for CCIR moderate/poor losing trials
/// (`FT8_BENCHMARK.md` CCIR fading gap investigation, issue #72
/// follow-up, 2026-07-18), mirroring `ft4_diag_weak_trials`/
/// `fst4_diag_weak_trials`. `process_candidate`/`process_one_candidate_inner`
/// (`src/ft8/decode.rs`, `src/ft8/decode_block/process_candidates.rs`)
/// are not `pub` outside `crate::ft8`, so this replicates
/// `process_candidate`'s prefix (coarse_sync -> fine_refine_3stage ->
/// nsync gate) directly against the public building blocks it itself
/// calls, and uses the real `decode_frame` (production entry point, no
/// sniper mode / no EqMode substitution — those are a hardware-roofing-
/// filter-specific accommodation, not a valid general stand-in, per
/// correction) as an opaque black box for the final BP/OSD stage — same
/// limitation `fst4_diag_weak_trials` accepted for its own black-box
/// decode call.
#[test]
#[ignore = "manual diagnostic — CCIR fading stage attribution (issue #72 follow-up)"]
fn ft8_diag_weak_trials() {
    use mfsk_core::engine::dsp::downsample::downsample_cached;
    use mfsk_core::engine::sync::fine_sync_power_per_block;
    use mfsk_core::ft8::Ft8;
    use mfsk_core::ft8::decode_block::{coarse_sync, compute_spectrogram};
    use mfsk_core::ft8::downsample::{FT8_CFG, build_fft_cache};
    use mfsk_core::ft8::llr::sync_quality;
    use mfsk_core::ft8::refine_fine::fine_refine_3stage;

    let dir = sweep_dir();
    for &(chan, snr_tag) in &[
        ("ccir_moderate", "m18"),
        ("ccir_moderate", "m17"),
        ("ccir_poor", "m18"),
        ("ccir_poor", "m17"),
    ] {
        for trial in 1..=20u32 {
            let path = dir.join(format!("ft8_{chan}_{snr_tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                continue;
            };
            if decode_wav_ft8(&audio).0 {
                continue; // only trace losing trials
            }

            let spec = compute_spectrogram(&audio, 3000.0);
            let candidates = coarse_sync(&spec, 100.0, 3000.0, 0.8, 50);
            let near: Vec<_> = candidates
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
                .collect();
            eprintln!(
                "{chan} {snr_tag} trial {trial}: {} candidates total, {} near golden freq",
                candidates.len(),
                near.len()
            );
            if near.is_empty() {
                continue;
            }
            let fft_cache = build_fft_cache(&audio);
            for c in &near {
                let cd0 = downsample_cached(&fft_cache, c.freq_hz, &FT8_CFG);
                let refine = fine_refine_3stage(&cd0, c.dt_sec);
                let refined_freq = c.freq_hz + refine.delf_hz;
                let i_start = ((refine.dt_sec + 0.5) * 200.0).round() as i32;
                let shifted =
                    mfsk_core::engine::sync2d::freq_shift_cd0(&cd0, refine.delf_hz, 200.0);
                let scores = fine_sync_power_per_block::<Ft8>(&shifted, i_start);
                let mean = scores.iter().sum::<f32>() / scores.len().max(1) as f32;
                let sync_cv = if mean > f32::EPSILON {
                    (scores.iter().map(|&x| (x - mean).powi(2)).sum::<f32>() / scores.len() as f32)
                        .sqrt()
                        / mean
                } else {
                    0.0
                };
                let mut cs_raw: [[mfsk_core::engine::scalar::Cmplx<f32>; 8]; 79] =
                    [[Default::default(); 8]; 79];
                mfsk_core::ft8::decode_block::fill_symbol_spectra(
                    &mut cs_raw,
                    &audio,
                    refined_freq,
                    refine.dt_sec,
                    mfsk_core::ft8::decode_block::SymMask::SyncOnly,
                    Some(&fft_cache),
                );
                mfsk_core::ft8::decode_block::fill_symbol_spectra(
                    &mut cs_raw,
                    &audio,
                    refined_freq,
                    refine.dt_sec,
                    mfsk_core::ft8::decode_block::SymMask::DataOnly,
                    Some(&fft_cache),
                );
                let nsync = sync_quality(&cs_raw);
                eprintln!(
                    "  cand freq={:.2} dt={:.3} coarse_score={:.4} refined_freq={:.2} \
                     refined_dt={:.3} refine_score={:.3} sync_cv={:.3} nsync={}/21 (gate>6={})",
                    c.freq_hz,
                    c.dt_sec,
                    c.score,
                    refined_freq,
                    refine.dt_sec,
                    refine.score,
                    sync_cv,
                    nsync,
                    nsync > 6
                );
            }
            // Final BP/OSD stage: opaque black box (see doc comment). Already
            // known false (filtered above), restated for readability.
            eprintln!("  -> full pipeline (decode_frame) decode: false");
        }
    }
}
