//! FST4-60 sniper: real-path vs. DDC-path **full decode** recall, by SNR.
//!
//! `tests/fst4_ddc_sniper_sensitivity.rs` measured coarse_sync's own
//! score, which is only half the story — this crate's real
//! signal/noise selectivity happens downstream, in the LDPC/CRC
//! decode itself (see that file's own doc comment on why binary
//! coarse_sync detection isn't informative). This test goes all the
//! way through: coarse_sync -> refine (`fst4_sync_search`, the *real*
//! production refine algorithm, not the simpler `fine_sync_power` the
//! stage 4 unit test used) -> LLR -> BP/OSD -> CRC -> message unpack,
//! for both the real 12 kHz path and the DDC path, and compares
//! whether the corpus's golden message actually comes out.
//!
//! The DDC side reuses `engine::pipeline::process_candidate_precomputed`
//! — an `internal-testing`-gated hook built exactly for this: feed a
//! precomputed `(cd0, freq_hz, i0, score)` refine result in directly,
//! skipping `downsample_cached` entirely (its own doc comment: "how
//! the LLR/BP/OSD stage alone gets measured... without waiting on
//! [non-power-of-two FFT support]"). So the DDC path here is genuinely
//! coarse_sync + refine fed by the DDC baseband, then the *unmodified*
//! production LLR/BP/OSD/CRC/unpack — no separate reimplementation of
//! the hard part.
//!
//! `#[ignore]` — tier C, needs the generated corpus:
//!
//! ```sh
//! scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh
//! cargo test --test fst4_ddc_sniper_full_decode --release \
//!   --features fst4,fft-rustfft,parallel,internal-testing \
//!   -- --ignored --nocapture
//! ```
//!
//! Measured 2026-08-21 (~120 s): at the 50%-crossing region the two
//! paths are statistically indistinguishable — `-27 dB: real 80/100,
//! ddc 77/100` (binomial σ ≈ 4 at n=100, so a 3-hit gap is well under
//! 1σ), `-28 dB: 7/20 both`, `-29 dB: 1/20 both`, `-30 dB: 0/20 both`;
//! both saturate at/near 100% from -26 dB up. This is the number that
//! actually answers the sensitivity question the stage 3 test's own
//! ~1/3 score ratio raised, and it says the DDC coarse-sync + refine
//! pipeline costs no measurable full-decode recall at FST4-60A's real
//! operating point.

#![cfg(all(feature = "fst4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt;

use num_complex::Complex;

use mfsk_core::engine::dsp::ddc::ddc_block;
use mfsk_core::engine::dsp::downsample::build_fft_cache;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync::{AudioSource, RxGrid, SyncCandidate, coarse_sync};
use mfsk_core::engine::sync2d::fst4_sync_search;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::ddc::{grid_for, sniper_cascade, sniper_refine_recenter};
use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

// Matches `tests/fst4_sweep.rs`'s own `GOLDEN_MSG` — the message the
// `fst4_60_awgn_*.wav` corpus actually encodes (not this file's own
// earlier synthetic tests' "CQ JA1ABC PM95", a different message used
// only in `fst4::ddc`'s unit tests).
const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const SEARCH_HALF_WIDTH_HZ: f32 = 250.0;
const SYNC_MIN: f32 = 0.8;
const MAX_CAND: usize = 50;
/// `fst4::decode::SYNC_Q_MIN` — private to that module, reproduced
/// here as a literal (both call `pipeline::decode_frame`/
/// `process_candidate_precomputed` with the same value, so this stays
/// an apples-to-apples comparison against the real path's own
/// `DecodeRequest`).
const SYNC_Q_MIN: u32 = 16;
/// Cap on how many DDC-path candidates get refined+decoded per trial
/// — coarse_sync can return up to `MAX_CAND`, and refine+decode is the
/// expensive step; the real path's own `decode_frame` tries every
/// candidate; this caps it to keep a multi-SNR sweep tractable while
/// still trying enough candidates that a truncation artifact would
/// show up as a gap between the two paths' recall at the *same* SNR
/// coarse_sync returns the golden candidate at (which the companion
/// sensitivity test already confirms is at rank 1 in the clean case).
const MAX_CAND_TRIED: usize = 8;

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_FST4_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/fst4_sweep")
        .to_path_buf()
}

fn parse_snr_tag(tag: &str) -> Option<i32> {
    if let Some(rest) = tag.strip_prefix('m') {
        rest.parse::<i32>().ok().map(|v| -v)
    } else if let Some(rest) = tag.strip_prefix('p') {
        rest.parse::<i32>().ok()
    } else {
        None
    }
}

struct WavMeta {
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
        let parts: Vec<&str> = stem.split('_').collect();
        if parts.len() != 5 || parts[0] != "fst4" || parts[1] != "60" || parts[2] != "awgn" {
            continue;
        }
        let trial: u32 = match parts[4].parse() {
            Ok(v) => v,
            Err(_) => continue,
        };
        let snr_db = match parse_snr_tag(parts[3]) {
            Some(v) => v,
            None => continue,
        };
        out.push(WavMeta {
            snr_db,
            trial,
            path,
        });
    }
    out.sort_by_key(|m| (-m.snr_db, m.trial));
    out
}

fn is_golden(msg77: &[u8]) -> bool {
    let arr: &[u8; 77] = match msg77.try_into() {
        Ok(a) => a,
        Err(_) => return false,
    };
    unpack77(arr).as_deref() == Some(GOLDEN_MSG)
}

fn real_path_decodes(audio: &[i16]) -> bool {
    DecodeRequest::<Fst4s60>::new(
        audio,
        GOLDEN_FREQ_HZ - SEARCH_HALF_WIDTH_HZ,
        GOLDEN_FREQ_HZ + SEARCH_HALF_WIDTH_HZ,
        SYNC_MIN,
        MAX_CAND,
    )
    .decode()
    .results
    .iter()
    .any(|r| is_golden(r.message77()))
}

/// Recentre-and-decimate `cand`'s own frequency out of the coarse DDC
/// baseband, trim the (best-effort) combined group delay, RMS-
/// normalise (matching `refine_candidate_position_impl`'s own
/// normalisation, so `process_candidate_precomputed`'s LLR scaling
/// sees the same convention it would from `downsample_cached`), then
/// run the *real* refine algorithm and hand the result to production
/// LLR/BP/OSD/CRC/unpack.
fn ddc_refine_and_decode(
    cand: &SyncCandidate,
    coarse_i: &[f32],
    coarse_q: &[f32],
    coarse_delay_orig: usize,
    fft_cache: &[Complex<f32>],
) -> Option<String> {
    let mut refine = sniper_refine_recenter(cand.freq_hz, GOLDEN_FREQ_HZ);
    let mut cd0_i = Vec::new();
    let mut cd0_q = Vec::new();
    refine.push(coarse_i, coarse_q, &mut cd0_i, &mut cd0_q);
    refine.flush(&mut cd0_i, &mut cd0_q);
    let refine_delay_out = refine.group_delay_output_samples();

    const REFINE_DS_RATE_HZ: f32 = 111.111_11;
    let total_delay_out = (coarse_delay_orig as f32 * REFINE_DS_RATE_HZ / 12_000.0).round()
        as usize
        + refine_delay_out;
    let trim = total_delay_out.min(cd0_i.len());

    let mut cd0: Vec<Complex<f32>> = cd0_i[trim..]
        .iter()
        .zip(cd0_q[trim..].iter())
        .map(|(&i, &q)| Complex::new(i, q))
        .collect();

    // Same RMS-normalisation `refine_candidate_position_impl` applies.
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len().max(1) as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }

    let s2 = fst4_sync_search::<Fst4s60>(&cd0, cand);
    let precomputed = (cd0, s2.freq_hz, s2.i0, s2.score);

    let result = process_candidate_precomputed::<Fst4s60>(
        cand,
        fft_cache,
        &FST4_60A_DOWNSAMPLE,
        DecodeDepth::FULL,
        DecodeStrictness::Normal,
        &[],
        EqMode::Off,
        SYNC_Q_MIN,
        precomputed,
        true, // skip_snr — snr_db not needed for this recall comparison
        false,
    )?;
    let mut m77 = [0u8; 77];
    m77.copy_from_slice(result.message77());
    unpack77(&m77)
}

fn ddc_path_decodes(audio: &[i16]) -> bool {
    let coarse_cfg = sniper_cascade(GOLDEN_FREQ_HZ);
    let (coarse_i, coarse_q) = ddc_block(audio, &coarse_cfg);
    // Recompute the same delay estimate `ddc_block` doesn't expose —
    // matches `fst4::ddc::tests::ddc_refine_matches_downsample_cached_
    // reference`'s own derivation.
    let coarse_delay_orig = {
        use mfsk_core::engine::dsp::ddc::StreamingComplexDdc;
        StreamingComplexDdc::new(&coarse_cfg).group_delay_input_samples()
    };

    let grid = grid_for(600.0);
    let cands = coarse_sync::<Fst4s60>(
        AudioSource::Complex(&coarse_i, &coarse_q),
        GOLDEN_FREQ_HZ - SEARCH_HALF_WIDTH_HZ,
        GOLDEN_FREQ_HZ + SEARCH_HALF_WIDTH_HZ,
        SYNC_MIN,
        Some(GOLDEN_FREQ_HZ),
        MAX_CAND,
        RxGrid::complex(grid.fs_c, GOLDEN_FREQ_HZ),
    );
    if cands.is_empty() {
        return false;
    }

    let fft_cache = build_fft_cache(audio, &FST4_60A_DOWNSAMPLE);
    for cand in cands.iter().take(MAX_CAND_TRIED) {
        if let Some(msg) =
            ddc_refine_and_decode(cand, &coarse_i, &coarse_q, coarse_delay_orig, &fft_cache)
            && msg == GOLDEN_MSG
        {
            return true;
        }
    }
    false
}

#[test]
#[ignore = "tier C — needs the fst4_60_awgn_* sweep corpus; run with --ignored --nocapture"]
fn fst4_60_sniper_ddc_vs_real_full_decode() {
    let dir = sweep_dir();
    let wavs = collect_wavs(&dir);
    if wavs.is_empty() {
        eprintln!(
            "No fst4_60_awgn_*.wav found in {dir:?}\n\
             Run: scripts/build_fst4sim.sh && scripts/gen_fst4_sweep_wavs.sh"
        );
        return;
    }

    #[cfg(feature = "parallel")]
    use rayon::prelude::*;
    use std::collections::BTreeMap;
    use std::io::Write;

    let mut groups: BTreeMap<i32, Vec<&WavMeta>> = BTreeMap::new();
    for w in &wavs {
        groups.entry(w.snr_db).or_default().push(w);
    }

    let mut csv = common::sweep_csv_writer(
        "MFSK_FST4_DDC_SNIPER_DECODE_CSV",
        "snr_db,trial,real_pass,ddc_pass",
    );

    eprintln!("\n{:-<56}", "");
    eprintln!(
        "  {:>7}  {:>6}  {:>10}  {:>10}",
        "SNR(dB)", "n", "real", "ddc"
    );
    eprintln!("{:-<56}", "");

    for (snr, group) in &groups {
        #[cfg(feature = "parallel")]
        let results: Vec<(u32, bool, bool)> = group
            .par_iter()
            .filter_map(|w| {
                load_wav_i16_opt(&w.path)
                    .map(|audio| (w.trial, real_path_decodes(&audio), ddc_path_decodes(&audio)))
            })
            .collect();

        #[cfg(not(feature = "parallel"))]
        let results: Vec<(u32, bool, bool)> = group
            .iter()
            .filter_map(|w| {
                load_wav_i16_opt(&w.path)
                    .map(|audio| (w.trial, real_path_decodes(&audio), ddc_path_decodes(&audio)))
            })
            .collect();

        let n = results.len() as u32;
        if n == 0 {
            continue;
        }
        let real_hits = results.iter().filter(|&&(_, r, _)| r).count() as u32;
        let ddc_hits = results.iter().filter(|&&(_, _, d)| d).count() as u32;

        if let Some(f) = csv.as_mut() {
            for &(trial, r, d) in &results {
                writeln!(f, "{snr},{trial},{},{}", r as u8, d as u8).unwrap();
            }
        }

        eprintln!("  {snr:>7}  {n:>6}  {real_hits:>3}/{n:<3}    {ddc_hits:>3}/{n:<3}");
    }
    eprintln!("{:-<56}", "");
}
