//! Which rungs of FT4's LLR/BP/OSD ladder earn their cost? — the
//! embedded FT4 budget line (`docs/notes/FT4_BENCHMARK.md` §17-21).
//!
//! After §21 cut the candidate count from 31 to 12, the projected slot
//! is `(2 492 search + 1 943 LLR/BP) × 12/31 ≈ 1 717 ms` against a
//! 1 960 ms budget. The LLR/BP half of that has never been broken down:
//! FT4 climbs four BP rungs (`llra` nsym=1, `llrb` nsym=2, `llrc`
//! nsym=`LLR_NSYM_MAX`=4, `llrd` nsym=1 bit-normalised) and then, if
//! `depth.osd`, re-tries all four through OSD at depth 2 or 3.
//!
//! FST4's equivalent ablation found its `nsym=8` rung was ~99 % of the
//! BP-side cost and that `llrd` "never contributes additional recall".
//! Neither statement was ever checked for FT4, whose `nsym=4` rung is
//! two orders of magnitude cheaper (4⁴ = 256 tone hypotheses per group
//! against 4⁸ = 65 536) — so the answer is not portable and has to be
//! measured here.
//!
//! **Measured on weak data, not on the golden.** The golden recording's
//! signals are strong enough that the whole ladder collapses to its
//! first rung: `EMBEDDED` and `FULL` decode identically there, which is
//! exactly how the bench came to record "OSD buys nothing". The same
//! comparison over 560 sweep files straddling four channels' 50 %
//! crossings gives 237 against 179 (§21.4). A rung question answered on
//! the golden would answer the wrong question.
//!
//! Method mirrors `fst4_llrd_ablation_ddc.rs`: one shared front end per
//! file (coarse candidates, oracle-filtered to the golden frequency so
//! candidate *selection* cannot contaminate a rung question, then
//! refine → `symbol_spectra` → `sync_quality`), and then each config
//! runs its own ladder over the same `cs`. The escalation gates,
//! `bp_max_iter = 40` and the `osd_max_errors` strictness check are the
//! production ones, so the `abcd+OSD` config reproduces
//! `process_candidate_basic` at `DecodeDepth::FULL` and `abcd` alone
//! reproduces `EMBEDDED` — which is the harness's own self-check.
//!
//! `#[ignore]` — tier C, needs the ft4_sweep corpus:
//!
//! ```sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!   --test ft4_llr_ladder_ablation -- --ignored --nocapture
//! ```

#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use std::time::Instant;

use mfsk_core::engine::dsp::downsample::{build_fft_cache, downsample_cached};
use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
use mfsk_core::engine::llr::{compute_llr_partial, descramble_info, symbol_spectra, sync_quality};
use mfsk_core::engine::pipeline::{DecodeStrictness, osd_escalation_gates};
use mfsk_core::engine::sync2d::{freq_shift_cd0, ft4_sync_search};
use mfsk_core::engine::{FecCodec, FecOpts, MessageCodec, Protocol};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 5.0;

const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 3000.0;
/// WSJT-X's own (`ft4_decode.f90:195`) — see §21.
const SYNC_MIN: f32 = 1.2;
const MAX_CAND: usize = 50;
const SYNC_Q_MIN: u32 = 8;
/// `ft4_decode.f90:194`. FT4 is the outlier; FT8/FST4 use 30.
const BP_MAX_ITER: u32 = 40;

const CHANNELS: &[&str] = &["awgn", "ccir_good", "ccir_moderate", "ccir_poor"];
const TAGS: &[&str] = &["m14", "m15", "m16", "m17", "m18", "m19", "m20"];
const TRIALS: u32 = 20;

/// One BP rung, in the order `process_candidate_basic_impl` climbs
/// them. `nsym` is what `compute_llr_partial` is asked for; `llrd` is
/// the bit-normalised nsym=1 variant, which is a different metric
/// rather than a deeper one.
#[derive(Copy, Clone, PartialEq, Debug)]
enum Rung {
    A,
    B,
    C,
    D,
}

struct Config {
    name: &'static str,
    rungs: &'static [Rung],
    osd: bool,
}

const CONFIGS: &[Config] = &[
    Config {
        name: "abcd+OSD  (= FULL)",
        rungs: &[Rung::A, Rung::B, Rung::C, Rung::D],
        osd: true,
    },
    Config {
        name: "abc +OSD",
        rungs: &[Rung::A, Rung::B, Rung::C],
        osd: true,
    },
    Config {
        name: "abd +OSD",
        rungs: &[Rung::A, Rung::B, Rung::D],
        osd: true,
    },
    Config {
        name: "ab  +OSD",
        rungs: &[Rung::A, Rung::B],
        osd: true,
    },
    Config {
        name: "a   +OSD",
        rungs: &[Rung::A],
        osd: true,
    },
    Config {
        name: "abcd      (= EMBEDDED)",
        rungs: &[Rung::A, Rung::B, Rung::C, Rung::D],
        osd: false,
    },
    Config {
        name: "abc",
        rungs: &[Rung::A, Rung::B, Rung::C],
        osd: false,
    },
    Config {
        name: "abd",
        rungs: &[Rung::A, Rung::B, Rung::D],
        osd: false,
    },
    Config {
        name: "ab",
        rungs: &[Rung::A, Rung::B],
        osd: false,
    },
    Config {
        name: "a",
        rungs: &[Rung::A],
        osd: false,
    },
];

fn sweep_dir() -> std::path::PathBuf {
    if let Ok(d) = std::env::var("MFSK_FT4_SWEEP_DIR") {
        return std::path::PathBuf::from(d);
    }
    std::path::Path::new(&std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default())
        .join("../embedded-poc/assets/ft4_sweep")
        .to_path_buf()
}

/// Like [`run_ladder`] but returns the decoded message instead of
/// matching it against the corpus's known one — the golden recording
/// carries fourteen different signals, so "did it decode" is a set, not
/// a boolean.
fn decode_any(cfg: &Config, cs: &[num_complex::Complex<f32>], nsync: u32) -> Option<String> {
    let fec = <Ft4 as Protocol>::Fec::default();
    let verify_info =
        Some(<<Ft4 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);
    let fast = mfsk_core::engine::llr::compute_llr_fast::<Ft4, f32>(cs);
    let llr_for = |rung: &Rung| -> Vec<f32> {
        match rung {
            Rung::A => fast.llra.clone(),
            Rung::B => compute_llr_partial::<Ft4, f32, f32>(cs, 2),
            Rung::C => compute_llr_partial::<Ft4, f32, f32>(
                cs,
                <Ft4 as mfsk_core::ModulationParams>::LLR_NSYM_MAX as usize,
            ),
            Rung::D => fast.llrd.clone(),
        }
    };
    let unpack = |info: &[u8]| -> Option<String> {
        let mut m77 = [0u8; 77];
        m77.copy_from_slice(&info[..77]);
        unpack77(&m77)
    };

    let bp_opts = FecOpts {
        bp_max_iter: BP_MAX_ITER,
        osd_depth: 0,
        ap_mask: None,
        verify_info,
        ..FecOpts::default()
    };
    let mut computed: Vec<Vec<f32>> = Vec::with_capacity(cfg.rungs.len());
    for rung in cfg.rungs {
        let llr = llr_for(rung);
        if let Some(mut r) = fec.decode_soft(&llr, &bp_opts) {
            descramble_info::<Ft4>(&mut r.info);
            if let Some(m) = unpack(&r.info) {
                return Some(m);
            }
        }
        computed.push(llr);
    }

    if cfg.osd {
        let (osd_attempt_min, osd_depth3_min) = osd_escalation_gates::<Ft4>();
        if nsync >= osd_attempt_min {
            let osd_depth: u8 = if nsync >= osd_depth3_min { 3 } else { 2 };
            let osd_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: osd_depth as u32,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let max_err = DecodeStrictness::Normal.osd_max_errors(osd_depth);
            for llr in &computed {
                if let Some(mut r) = fec.decode_soft(llr, &osd_opts) {
                    if r.hard_errors >= max_err {
                        continue;
                    }
                    descramble_info::<Ft4>(&mut r.info);
                    if let Some(m) = unpack(&r.info) {
                        return Some(m);
                    }
                }
            }
        }
    }
    None
}

fn is_golden_info(info: &[u8]) -> bool {
    let mut m77 = [0u8; 77];
    m77.copy_from_slice(&info[..77]);
    unpack77(&m77).as_deref() == Some(GOLDEN_MSG)
}

/// Run one config's ladder over one candidate's symbol spectra.
/// Returns `(decoded_golden, elapsed)`.
///
/// The LLR variants are computed inside, per config and per rung
/// reached, because that is where the cost is — a config that never
/// climbs to `llrc` never pays for `compute_llr_partial(cs, 4)` either.
fn run_ladder(cfg: &Config, cs: &[num_complex::Complex<f32>], nsync: u32) -> (bool, f64) {
    let t0 = Instant::now();
    let fec = <Ft4 as Protocol>::Fec::default();
    let verify_info =
        Some(<<Ft4 as Protocol>::Msg as MessageCodec>::verify_info as fn(&[u8]) -> bool);
    let bp_opts = FecOpts {
        bp_max_iter: BP_MAX_ITER,
        osd_depth: 0,
        ap_mask: None,
        verify_info,
        ..FecOpts::default()
    };

    // `llra` and `llrd` come out of a single `compute_llr_fast` call,
    // exactly as `process_candidate_basic_impl` gets them — `llrd` is
    // nsym=1 bit-normalised, a different metric rather than a deeper
    // one, so its own LLR cost is already paid by rung A. Charging it
    // separately would inflate what dropping it appears to save.
    let fast = mfsk_core::engine::llr::compute_llr_fast::<Ft4, f32>(cs);

    let mut hit = false;
    let mut computed: Vec<Vec<f32>> = Vec::with_capacity(cfg.rungs.len());
    for rung in cfg.rungs {
        let llr: Vec<f32> = match rung {
            Rung::A => fast.llra.clone(),
            Rung::B => compute_llr_partial::<Ft4, f32, f32>(cs, 2),
            Rung::C => compute_llr_partial::<Ft4, f32, f32>(
                cs,
                <Ft4 as mfsk_core::ModulationParams>::LLR_NSYM_MAX as usize,
            ),
            Rung::D => fast.llrd.clone(),
        };
        if !hit
            && fec.decode_soft(&llr, &bp_opts).is_some_and(|mut r| {
                descramble_info::<Ft4>(&mut r.info);
                is_golden_info(&r.info)
            })
        {
            hit = true;
        }
        computed.push(llr);
        if hit {
            break;
        }
    }

    // OSD retries every variant the config carries, same order, same
    // gates as `process_candidate_basic_impl`.
    if !hit && cfg.osd {
        let (osd_attempt_min, osd_depth3_min) = osd_escalation_gates::<Ft4>();
        if nsync >= osd_attempt_min {
            // Rungs BP never reached still have to be computed for OSD
            // to try them — the production ladder is in the same
            // position, since BP failing means it climbed them all.
            while computed.len() < cfg.rungs.len() {
                let rung = cfg.rungs[computed.len()];
                computed.push(match rung {
                    Rung::A => fast.llra.clone(),
                    Rung::B => compute_llr_partial::<Ft4, f32, f32>(cs, 2),
                    Rung::C => compute_llr_partial::<Ft4, f32, f32>(
                        cs,
                        <Ft4 as mfsk_core::ModulationParams>::LLR_NSYM_MAX as usize,
                    ),
                    Rung::D => fast.llrd.clone(),
                });
            }
            let osd_depth: u8 = if nsync >= osd_depth3_min { 3 } else { 2 };
            let osd_opts = FecOpts {
                bp_max_iter: BP_MAX_ITER,
                osd_depth: osd_depth as u32,
                ap_mask: None,
                verify_info,
                ..FecOpts::default()
            };
            let max_err = DecodeStrictness::Normal.osd_max_errors(osd_depth);
            for llr in &computed {
                if let Some(mut r) = fec.decode_soft(llr, &osd_opts) {
                    if r.hard_errors >= max_err {
                        continue;
                    }
                    descramble_info::<Ft4>(&mut r.info);
                    if is_golden_info(&r.info) {
                        hit = true;
                        break;
                    }
                }
            }
        }
    }

    (hit, t0.elapsed().as_secs_f64())
}

#[test]
#[ignore = "tier C — needs the ft4_sweep corpus; run with --ignored --nocapture"]
fn ft4_llr_ladder_ablation_across_the_crossing() {
    use rayon::prelude::*;

    let dir = sweep_dir();
    let mut work: Vec<(&str, &str, u32)> = Vec::new();
    for &ch in CHANNELS {
        for &tag in TAGS {
            for trial in 1..=TRIALS {
                work.push((ch, tag, trial));
            }
        }
    }

    let per_file: Vec<(bool, Vec<(bool, f64)>)> = work
        .par_iter()
        .map(|&(ch, tag, trial)| {
            let path = dir.join(format!("ft4_{ch}_{tag}_{trial:02}.wav"));
            let Some(audio) = load_wav_i16_opt(&path) else {
                return (false, Vec::new());
            };
            let cands = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
            let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);
            let ds_rate = 12_000.0 / <Ft4 as mfsk_core::ModulationParams>::NDOWN as f32;

            let mut totals = vec![(false, 0.0f64); CONFIGS.len()];
            for c in cands
                .iter()
                .filter(|c| (c.freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ)
            {
                let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FT4_DOWNSAMPLE);
                let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
                if sum2 > f32::EPSILON {
                    let inv = 1.0 / sum2.sqrt();
                    for z in cd0.iter_mut() {
                        *z *= inv;
                    }
                }
                let s2 = ft4_sync_search::<Ft4>(&cd0, c);
                let cd0 = freq_shift_cd0(&cd0, s2.freq_hz - c.freq_hz, ds_rate);
                let cs = symbol_spectra::<Ft4>(&cd0, s2.i0);
                let nsync = sync_quality::<Ft4>(&cs);
                if nsync <= SYNC_Q_MIN {
                    continue;
                }
                for (i, cfg) in CONFIGS.iter().enumerate() {
                    let (hit, dt) = run_ladder(cfg, &cs, nsync);
                    totals[i].0 |= hit;
                    totals[i].1 += dt;
                }
            }
            (true, totals)
        })
        .collect();

    let present = per_file.iter().filter(|f| f.0).count();
    if present == 0 {
        eprintln!("skipping: no FT4 sweep corpus under {}", dir.display());
        return;
    }
    assert_eq!(present, work.len(), "corpus is incomplete");

    eprintln!(
        "{} files, {} configs, gates {:?}\n",
        present,
        CONFIGS.len(),
        osd_escalation_gates::<Ft4>()
    );
    eprintln!("config                   decodes   vs FULL   CPU ms   vs FULL");
    let full_hits = per_file
        .iter()
        .filter(|f| f.1.first().is_some_and(|c| c.0))
        .count();
    let full_ms: f64 = per_file
        .iter()
        .filter_map(|f| f.1.first())
        .map(|c| c.1 * 1000.0)
        .sum();
    for (i, cfg) in CONFIGS.iter().enumerate() {
        let hits = per_file
            .iter()
            .filter(|f| f.1.get(i).is_some_and(|c| c.0))
            .count();
        let ms: f64 = per_file
            .iter()
            .filter_map(|f| f.1.get(i))
            .map(|c| c.1 * 1000.0)
            .sum();
        eprintln!(
            "{:<22}   {hits:>7}   {:>+7}   {ms:>6.0}   {:>6.2}x",
            cfg.name,
            hits as i32 - full_hits as i32,
            ms / full_ms
        );
    }
    eprintln!(
        "\nCPU ms is summed across rayon threads — a ratio between configs, \
         not a wall-clock."
    );

    // Per-cell, for the two configs that bracket the OSD question —
    // 56 decodes is not a currency anything else in this file is
    // quoted in, and a per-SNR breakdown converts it into one.
    for (label, ci) in [("FULL (abcd+OSD)", 0usize), ("EMBEDDED (abcd)", 5)] {
        eprintln!("\n{label}: recall of {TRIALS} per cell");
        eprint!("{:<16}", "channel");
        for t in TAGS {
            eprint!("{t:>5}");
        }
        eprintln!();
        for (chi, ch) in CHANNELS.iter().enumerate() {
            eprint!("{ch:<16}");
            for ti in 0..TAGS.len() {
                let base = (chi * TAGS.len() + ti) * TRIALS as usize;
                let n = per_file[base..base + TRIALS as usize]
                    .iter()
                    .filter(|f| f.1.get(ci).is_some_and(|c| c.0))
                    .count();
                eprint!("{n:>5}");
            }
            eprintln!();
        }
    }

    assert!(
        full_hits > 0,
        "the FULL config decoded nothing — harness is broken"
    );
}

/// The same configs on the WSJT-X golden — the real off-air file the
/// sweep corpus cannot substitute for, and the one a "drop this rung"
/// proposal has to survive before it is worth making.
///
/// Reported as distinct messages rather than trial counts, since this
/// is one slot with fourteen signals in it. Not `#[ignore]`: it needs
/// only the vendored golden.
#[test]
fn ft4_llr_ladder_ablation_on_the_golden() {
    let Some(path) = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    ) else {
        if std::env::var("MFSK_REQUIRE_CORPUS").is_ok() {
            panic!("MFSK_REQUIRE_CORPUS=1 but the FT4 golden recording is missing");
        }
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };
    let raw = load_wav_i16_opt(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; 90_000];
    let copy = raw.len().min(audio.len());
    audio[..copy].copy_from_slice(&raw[..copy]);

    // The bench search (`bench_assets`), i.e. what a board would run.
    let cands = ft4_coarse_sync(&audio, FREQ_MIN_HZ, 2700.0, SYNC_MIN, None, 100);
    let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);
    let ds_rate = 12_000.0 / <Ft4 as mfsk_core::ModulationParams>::NDOWN as f32;

    let mut per_config: Vec<Vec<String>> = vec![Vec::new(); CONFIGS.len()];
    for c in &cands {
        let mut cd0 = downsample_cached(&fft_cache, c.freq_hz, &FT4_DOWNSAMPLE);
        let sum2: f32 = cd0.iter().map(|z| z.norm_sqr()).sum::<f32>() / cd0.len() as f32;
        if sum2 > f32::EPSILON {
            let inv = 1.0 / sum2.sqrt();
            for z in cd0.iter_mut() {
                *z *= inv;
            }
        }
        let s2 = ft4_sync_search::<Ft4>(&cd0, c);
        let cd0 = freq_shift_cd0(&cd0, s2.freq_hz - c.freq_hz, ds_rate);
        let cs = symbol_spectra::<Ft4>(&cd0, s2.i0);
        let nsync = sync_quality::<Ft4>(&cs);
        if nsync <= SYNC_Q_MIN {
            continue;
        }
        for (i, cfg) in CONFIGS.iter().enumerate() {
            if let Some(msg) = decode_any(cfg, &cs, nsync)
                && !per_config[i].contains(&msg)
            {
                per_config[i].push(msg);
            }
        }
    }

    eprintln!("golden, {} candidates:", cands.len());
    for (i, cfg) in CONFIGS.iter().enumerate() {
        eprintln!("  {:<22} {} distinct", cfg.name, per_config[i].len());
    }

    let full = per_config[0].len();
    assert_eq!(
        full, 11,
        "the FULL arm moved — fix `ft4_wsjtx_samples` first"
    );
    // `abc+OSD` is index 1 and `abc` index 6 — the two `llrd`-free
    // configs. On the sweep corpus they cost nothing; if they cost
    // something *here*, on real off-air signals, that is what decides
    // whether dropping the rung is a real option.
    assert_eq!(
        per_config[1].len(),
        full,
        "dropping llrd costs decodes on the golden (with OSD)"
    );
    assert_eq!(
        per_config[6].len(),
        per_config[5].len(),
        "dropping llrd costs decodes on the golden (BP only)"
    );
}
