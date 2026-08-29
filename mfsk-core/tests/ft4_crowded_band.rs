//! FT4 in a crowded band — the questions the single-signal sweep corpus
//! structurally cannot answer.
//!
//! `docs/notes/FT4_BENCHMARK.md` §21-22 measured the candidate budget
//! and the LLR ladder on `ft4_sweep`, where every file holds exactly one
//! signal at `DT = 0.0`. Two of those conclusions came with an explicit
//! caveat attached: every decode there ranks 0 (a property of the
//! fixture, not a finding), and the device budget projection leans on a
//! candidate count taken from a **single** real recording. FT4 is a
//! contest mode; the band it actually runs in is dense.
//!
//! This file measures the same things against WSJT-X's own multi-signal
//! simulator (`lib/ft4/ft4sim_mult.f90`, built by
//! `scripts/build_ft4sim.sh`, corpus by `scripts/gen_ft4_mult_wavs.sh`),
//! which lays N signals into one slot — each at its own SNR and
//! frequency, each at a **random DT in ±0.5 s** — and prints the ground
//! truth, which the corpus keeps as `manifest.tsv`.
//!
//! What it adds over `ft4_sweep`:
//!
//! | question | `ft4_sweep` | here |
//! |---|---|---|
//! | candidate count under occupancy | one signal only | 10 / 20 / 30 per slot |
//! | rank of a weak decode | always 0 | measured |
//! | phantom decodes | not measured | measured against the manifest |
//! | timing spread | `DT = 0.0` | random ±0.5 s |
//!
//! What it does **not** add: fading (the simulator is AWGN-only, so
//! `ft4_sweep`'s CCIR channels stay the instrument for that) and real
//! off-air artefacts (the WSJT-X golden stays the instrument for those).
//!
//! `#[ignore]` — tier C, needs the generated corpus:
//!
//! ```sh
//! scripts/build_ft4sim.sh && scripts/gen_ft4_mult_wavs.sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!   --test ft4_crowded_band -- --ignored --nocapture
//! ```

#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use std::collections::BTreeMap;
use std::path::PathBuf;

use mfsk_core::engine::dsp::downsample::build_fft_cache;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

/// The device search (`ft4_wsjtx_samples::bench_assets`), which is the
/// configuration whose budget this corpus exists to check.
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 3000.0;
const SYNC_MIN: f32 = 1.2;
const MAX_CAND: usize = 100;
const SYNC_Q_MIN: u32 = 8;
/// A decode is matched to a ground-truth row by message *and*
/// frequency; the manifest stores `nint(f0)`.
const FREQ_TOL_HZ: f32 = 10.0;

fn corpus_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_FT4_MULT_DIR") {
        return PathBuf::from(d);
    }
    PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/ft4_mult"
    ))
}

#[derive(Clone)]
struct Truth {
    snr_db: i32,
    freq_hz: f32,
    message: String,
}

/// `manifest.tsv` → one entry per WAV, in file order.
fn load_manifest(dir: &std::path::Path) -> BTreeMap<String, Vec<Truth>> {
    let mut out: BTreeMap<String, Vec<Truth>> = BTreeMap::new();
    let Ok(text) = std::fs::read_to_string(dir.join("manifest.tsv")) else {
        return out;
    };
    for line in text.lines().skip(1) {
        let f: Vec<&str> = line.split('\t').collect();
        if f.len() < 5 {
            continue;
        }
        out.entry(f[0].to_string()).or_default().push(Truth {
            snr_db: f[1].trim().parse().unwrap_or(0),
            freq_hz: f[3].trim().parse().unwrap_or(0.0),
            message: f[4].trim().to_string(),
        });
    }
    out
}

/// Occupancy is encoded in the file name: `ft4_mult_n<occ>_<idx>.wav`.
fn occupancy_of(wav: &str) -> u32 {
    wav.split('_')
        .find_map(|p| p.strip_prefix('n').and_then(|n| n.parse().ok()))
        .unwrap_or(0)
}

/// One slot's outcome: candidate count, the (rank, snr) of every true
/// decode, and the phantoms.
struct SlotResult {
    occupancy: u32,
    n_cands: usize,
    hits: Vec<(usize, i32)>,
    truth_snrs: Vec<i32>,
    phantoms: usize,
    /// Same slot through the production `DecodeRequest` with
    /// `sic_rounds(2)` — the multi-pass subtraction an embedded
    /// receiver does *not* have. `(snr of each hit, phantom count)`.
    sic_hits: Vec<i32>,
    sic_phantoms: usize,
}

#[test]
#[ignore = "tier C — needs the ft4_mult corpus; see the module doc for the two commands"]
fn ft4_crowded_band_candidate_budget_and_precision() {
    use rayon::prelude::*;

    let dir = corpus_dir();
    let manifest = load_manifest(&dir);
    if manifest.is_empty() {
        eprintln!(
            "skipping: no crowded-band corpus under {} — run scripts/gen_ft4_mult_wavs.sh",
            dir.display()
        );
        return;
    }

    let work: Vec<(&String, &Vec<Truth>)> = manifest.iter().collect();
    let results: Vec<SlotResult> = work
        .par_iter()
        .filter_map(|(wav, truth)| {
            let audio = load_wav_i16_opt(dir.join(wav.as_str()))?;
            let cands = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
            let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);

            // Single pass, no SIC — the shape an embedded receiver runs
            // (`dual_core` has no subtract path). The host default is
            // more capable; this is deliberately the harder config.
            let mut decoded: Vec<(usize, String, f32)> = Vec::new();
            for (rank, c) in cands.iter().enumerate() {
                if let Some(d) = process_candidate_basic::<Ft4>(
                    c,
                    &fft_cache,
                    &FT4_DOWNSAMPLE,
                    DecodeDepth::FULL,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
                    SYNC_Q_MIN,
                ) {
                    let msg = unpack77(d.message77()).unwrap_or_default();
                    if !decoded
                        .iter()
                        .any(|(_, m, f)| *m == msg && (*f - d.freq_hz).abs() < FREQ_TOL_HZ)
                    {
                        decoded.push((rank, msg, d.freq_hz));
                    }
                }
            }

            let mut hits = Vec::new();
            let mut matched = vec![false; truth.len()];
            let mut phantoms = 0usize;
            for (rank, msg, freq) in &decoded {
                let found = truth.iter().enumerate().position(|(i, t)| {
                    !matched[i] && t.message == *msg && (t.freq_hz - freq).abs() <= FREQ_TOL_HZ
                });
                match found {
                    Some(i) => {
                        matched[i] = true;
                        hits.push((*rank, truth[i].snr_db));
                    }
                    None => phantoms += 1,
                }
            }

            // Second arm: the production entry point with successive
            // interference cancellation, which is what WSJT-X itself
            // runs (`ft4_decode.f90` `nsp=3`, `dosubtract=.true.`) and
            // what `dual_core` has no path for.
            let sic = mfsk_core::msg::decode_request::DecodeRequest::<Ft4>::new(
                &audio,
                FREQ_MIN_HZ,
                FREQ_MAX_HZ,
                SYNC_MIN,
                MAX_CAND,
            )
            .sic_rounds(2)
            .decode();
            let mut sic_hits = Vec::new();
            let mut sic_matched = vec![false; truth.len()];
            let mut sic_phantoms = 0usize;
            for d in &sic.results {
                let msg = unpack77(d.message77()).unwrap_or_default();
                match truth.iter().enumerate().position(|(i, t)| {
                    !sic_matched[i]
                        && t.message == msg
                        && (t.freq_hz - d.freq_hz).abs() <= FREQ_TOL_HZ
                }) {
                    Some(i) => {
                        sic_matched[i] = true;
                        sic_hits.push(truth[i].snr_db);
                    }
                    None => sic_phantoms += 1,
                }
            }

            Some(SlotResult {
                occupancy: occupancy_of(wav),
                n_cands: cands.len(),
                hits,
                truth_snrs: truth.iter().map(|t| t.snr_db).collect(),
                phantoms,
                sic_hits,
                sic_phantoms,
            })
        })
        .collect();

    assert!(
        !results.is_empty(),
        "corpus present but nothing decoded end-to-end"
    );

    // ── Candidate count and rank depth, per occupancy ────────────────
    eprintln!("occupancy  slots  mean cands  max cands  decodes  deepest rank  rank p90");
    let mut occs: Vec<u32> = results.iter().map(|r| r.occupancy).collect();
    occs.sort_unstable();
    occs.dedup();
    for occ in &occs {
        let rs: Vec<&SlotResult> = results.iter().filter(|r| r.occupancy == *occ).collect();
        let mean = rs.iter().map(|r| r.n_cands).sum::<usize>() as f32 / rs.len() as f32;
        let max = rs.iter().map(|r| r.n_cands).max().unwrap_or(0);
        let mut ranks: Vec<usize> = rs.iter().flat_map(|r| r.hits.iter().map(|h| h.0)).collect();
        ranks.sort_unstable();
        let deepest = ranks.last().copied().unwrap_or(0);
        let p90 = ranks
            .get((ranks.len() as f32 * 0.9) as usize)
            .copied()
            .unwrap_or(0);
        eprintln!(
            "{occ:>9}  {:>5}  {mean:>10.1}  {max:>9}  {:>7}  {deepest:>12}  {p90:>8}",
            rs.len(),
            ranks.len()
        );
    }

    // ── Recall vs SNR, per occupancy ─────────────────────────────────
    eprintln!("\nrecall by SNR (decoded / present) — single pass, no SIC:");
    let mut snrs: Vec<i32> = results.iter().flat_map(|r| r.truth_snrs.clone()).collect();
    snrs.sort_unstable();
    snrs.dedup();
    eprint!("{:<10}", "occupancy");
    for s in &snrs {
        eprint!("{s:>7}");
    }
    eprintln!();
    for occ in &occs {
        eprint!("{occ:<10}");
        for s in &snrs {
            let present: usize = results
                .iter()
                .filter(|r| r.occupancy == *occ)
                .map(|r| r.truth_snrs.iter().filter(|&&x| x == *s).count())
                .sum();
            let got: usize = results
                .iter()
                .filter(|r| r.occupancy == *occ)
                .map(|r| r.hits.iter().filter(|h| h.1 == *s).count())
                .sum();
            if present == 0 {
                eprint!("      -");
            } else {
                eprint!("{:>6.0}%", 100.0 * got as f32 / present as f32);
            }
        }
        eprintln!();
    }

    // Same table for the SIC arm. The gap between the two is the part
    // of a crowded band that is lost to *mutual interference* rather
    // than to noise — and it is the part an embedded receiver, which
    // has no subtract path, cannot get back.
    eprintln!("\nrecall by SNR — production `sic_rounds(2)`:");
    eprint!("{:<10}", "occupancy");
    for s in &snrs {
        eprint!("{s:>7}");
    }
    eprintln!();
    for occ in &occs {
        eprint!("{occ:<10}");
        for s in &snrs {
            let present: usize = results
                .iter()
                .filter(|r| r.occupancy == *occ)
                .map(|r| r.truth_snrs.iter().filter(|&&x| x == *s).count())
                .sum();
            let got: usize = results
                .iter()
                .filter(|r| r.occupancy == *occ)
                .map(|r| r.sic_hits.iter().filter(|&&x| x == *s).count())
                .sum();
            if present == 0 {
                eprint!("      -");
            } else {
                eprint!("{:>6.0}%", 100.0 * got as f32 / present as f32);
            }
        }
        eprintln!();
    }

    eprintln!("\ntotals: single pass vs sic_rounds(2)");
    for occ in &occs {
        let rs: Vec<&SlotResult> = results.iter().filter(|r| r.occupancy == *occ).collect();
        let present: usize = rs.iter().map(|r| r.truth_snrs.len()).sum();
        let plain: usize = rs.iter().map(|r| r.hits.len()).sum();
        let sic: usize = rs.iter().map(|r| r.sic_hits.len()).sum();
        eprintln!(
            "  occupancy {occ:>2}: {plain}/{present} ({:.0}%) -> {sic}/{present} ({:.0}%) with SIC",
            100.0 * plain as f32 / present as f32,
            100.0 * sic as f32 / present as f32
        );
    }

    // ── Precision ────────────────────────────────────────────────────
    eprintln!("\nphantoms (decodes with no ground-truth row):");
    let mut total_phantom = 0usize;
    let mut total_decode = 0usize;
    for occ in &occs {
        let rs: Vec<&SlotResult> = results.iter().filter(|r| r.occupancy == *occ).collect();
        let ph: usize = rs.iter().map(|r| r.phantoms + r.sic_phantoms).sum();
        let hits: usize = rs.iter().map(|r| r.hits.len() + r.sic_hits.len()).sum();
        total_phantom += ph;
        total_decode += ph + hits;
        eprintln!(
            "  occupancy {occ:>2}: {ph} phantom / {} decodes ({:.2}% of decodes, {:.3} per slot)",
            ph + hits,
            if ph + hits == 0 {
                0.0
            } else {
                100.0 * ph as f32 / (ph + hits) as f32
            },
            ph as f32 / rs.len() as f32
        );
    }

    // The one hard assertion: FT4 ships `max_extra: 0` on its golden,
    // so a crowded band producing a stream of phantoms would be a
    // precision regression the golden cannot see. 1 % of decodes is
    // loose enough not to trip on a single unlucky slot and tight
    // enough that a systematic false-decode path fails here.
    let phantom_pct = 100.0 * total_phantom as f32 / total_decode.max(1) as f32;
    assert!(
        phantom_pct < 1.0,
        "{total_phantom} phantoms of {total_decode} decodes ({phantom_pct:.2}%)"
    );
}
