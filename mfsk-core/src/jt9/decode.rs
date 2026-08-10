//! JT9 decode pipeline: WSJT-X-faithful softsym (downsam9 → peakdt9 →
//! AFC → twkfreq → symspec2 → fano232).
//!
//! Replaces the earlier box-car path in `baseband.rs` + `demod_bb.rs`.
//! The big audio FFT is computed once and reused across candidates.

use crate::engine::{DecodeContext, FecCodec, FecOpts, MessageCodec};
use crate::fec::ConvFano232;
use crate::msg::{Jt72Codec, Jt72Message};

use super::Jt9Result;
use super::softsym::{AudioFft, FSAMPLE_DOWN, afc9, llrs_from_c5, peakdt9, twkfreq_poly};

/// Sync-score gate: peakdt9 returns `(sync_avg/data_avg)−1`. For pure
/// noise this is ≈ 0; clean JT9 frames sit in the 0.3–10 range. WSJT-X
/// uses ccfbest > 30 (raw integrated power) on its own scale; the 1.5
/// threshold here is calibrated from the 130418_1742 sample (golden
/// signals score 1.5..15, phantoms < 0.8).
const SYNC_GATE: f32 = 1.5;

/// JT9 Fano-decode depth — mirrors WSJT-X `jt9_decode.f90`'s `ndepth`
/// cycle-per-bit budget tiers (the `-d`/`--depth` CLI flag, plus
/// "Decode Again" which uses the deepest tier unconditionally). Only
/// the cycle *budget* is ported here, not WSJT-X's own `ccflim`/
/// `schklim` gate-loosening at deeper tiers — this crate's own
/// sync-score/`sync`/`schk` thresholds were calibrated separately
/// against this crate's own candidate pipeline and are unchanged by
/// depth (see `docs/notes/BENCHMARKS.md`'s JT9 section for the
/// measured tradeoff and the real-`jt9`-at-`-d1` comparison this
/// default was checked against).
///
/// Every non-`Fast` tier costs more only on candidates that
/// ultimately fail to converge — a real signal converges in
/// microseconds regardless of the budget (measured,
/// `jt9::tests::candidate_loop_stage_diag`), so raising depth mainly
/// costs time on a busy/noisy band, not on a quiet one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Jt9Depth {
    /// `limit=5000` — WSJT-X's own automatic per-slot scan default
    /// (`jt9 -d1`, `jt9_decode.f90:84`). **The crate default** as of
    /// task #24's fix — see that variant's own doc comment for why
    /// `Fast` and `Normal` turned out to perform identically once the
    /// real bug was found and fixed.
    #[default]
    Fast,
    /// `limit=10000` — this crate's original hardcoded default before
    /// depth became configurable (2026-08-08). Initially *kept* as the
    /// crate default over `Fast` after finding `Normal` closer (but
    /// not equal) to real `jt9 -9 -d1`'s sensitivity — but that
    /// comparison's real cause turned out to be unrelated to cycle
    /// budget at all (task #24): `jt9::search::coarse_search`'s
    /// frequency grid (one bin = one tone spacing, ≈1.736 Hz) had no
    /// sub-bin refinement, so candidates routinely landed 0.3-1 Hz
    /// off the true frequency — inside Fano's sharp non-convergence
    /// zone regardless of cycle budget. Fixed via
    /// `search::refine_freq_hz` (3-point parabolic interpolation,
    /// same technique as JT65's own scalloping-loss fix, issue #169).
    /// Post-fix, `Fast` and `Normal` score identically on the AWGN
    /// sweep (both now *exceed* real `jt9 -d1`'s own −26/−27 dB
    /// numbers) — so `Normal`'s extra cycle budget buys nothing
    /// measurable anymore; `Fast` moved to being the default since
    /// it's strictly cheaper for the same result. See
    /// `docs/notes/BENCHMARKS.md` for the before/after numbers.
    Normal,
    /// `limit=30000` — WSJT-X `-d3`.
    Deep,
    /// `limit=100000` — WSJT-X's "Decode Again" tier (`nagain`,
    /// unconditional deepest budget, not gated by `-d`).
    Max,
}

impl Jt9Depth {
    fn max_cycles_per_bit(self) -> u64 {
        match self {
            Jt9Depth::Fast => 5_000,
            Jt9Depth::Normal => 10_000,
            Jt9Depth::Deep => 30_000,
            Jt9Depth::Max => 100_000,
        }
    }
}

/// Try to decode a JT9 signal centred at `freq_hz` using a pre-built
/// audio FFT. Returns `None` when the sync gate is missed, Fano fails
/// to converge, or the message is not `Jt72Message::Standard`.
///
/// `decode_scan`'s production candidate loop calls
/// [`decode_at_baseband_with_fft_depth`] directly (it always has an
/// explicit [`Jt9Depth`] to hand); this default-depth convenience is
/// currently only exercised by tests (`decode_at_baseband` below).
#[cfg_attr(not(test), allow(dead_code))]
pub fn decode_at_baseband_with_fft(big_fft: &AudioFft, freq_hz: f32) -> Option<Jt9Result> {
    decode_at_baseband_with_fft_depth(big_fft, freq_hz, Jt9Depth::default())
}

/// [`decode_at_baseband_with_fft`] with an explicit [`Jt9Depth`]
/// instead of the crate default.
pub fn decode_at_baseband_with_fft_depth(
    big_fft: &AudioFft,
    freq_hz: f32,
    depth: Jt9Depth,
) -> Option<Jt9Result> {
    if freq_hz <= 0.0 {
        return None;
    }

    let c2 = big_fft.downsam9(freq_hz);
    let (lagpk, sync_score, mut c3) = peakdt9(&c2);
    if !sync_score.is_finite() || sync_score < SYNC_GATE {
        return None;
    }

    // WSJT-X-faithful AFC: 3-parameter chi-square optimisation over
    // (frequency, drift, integer-sample time shift). `afc9` mutates
    // `c3` to apply the discovered integer shift; the caller mixes
    // out the residual frequency + drift with `twkfreq_poly`.
    let afc = afc9(&mut c3);
    twkfreq_poly(&mut c3, [afc.a0, afc.a1, 0.0]);

    // WSJT-X two-stage sync gate (`lib/jt9_decode.f90:139`): both
    // sync = (syncpk + 1)/4 and schk (the chkss2 normalised tone-0
    // sync power) must clear their thresholds before we spend Fano
    // cycles. Drops phantom convergences in busy bands.
    let sync = (afc.syncpk + 1.0) / 4.0;
    if !sync.is_finite() || sync < 1.0 {
        return None;
    }
    let (schk, llrs, snr_db) = llrs_from_c5(&c3);
    if !schk.is_finite() || schk < 1.5 {
        return None;
    }
    let opts = FecOpts {
        max_cycles_per_bit: Some(depth.max_cycles_per_bit()),
        ..FecOpts::default()
    };
    let res = ConvFano232.decode_soft(&llrs, &opts)?;
    let mut payload = [0u8; 72];
    payload.copy_from_slice(&res.info);
    let msg = Jt72Codec::default().unpack(&payload, &DecodeContext::default())?;

    // JT9 has no CRC — Fano can converge on plausible-looking junk.
    // Accepting only Standard form drops most hashed-callsign garbage.
    match &msg {
        Jt72Message::Standard { .. } => {}
        _ => return None,
    }

    // Translate (lagpk, df) back to audio-sample / Hz coordinates so
    // downstream consumers see a consistent timing/freq report.
    let start_sample = lag_to_audio_sample(lagpk);
    let freq_corrected = freq_hz - afc.a0;
    Some(Jt9Result {
        message: msg,
        freq_hz: freq_corrected,
        start_sample,
        snr_db,
    })
}

/// One-shot variant: builds the big FFT inline. Useful for tests and
/// callers that don't reuse the same audio across many candidates.
#[cfg(test)]
#[allow(dead_code)]
pub fn decode_at_baseband(
    audio: &[f32],
    _sample_rate: u32,
    _start_sample: usize,
    freq_hz: f32,
) -> Option<Jt9Result> {
    let big = AudioFft::build(audio);
    decode_at_baseband_with_fft(&big, freq_hz)
}

#[cfg(test)]
mod depth_tests {
    use super::Jt9Depth;

    #[test]
    fn cycle_budgets_match_wsjtx_ndepth_ladder() {
        assert_eq!(Jt9Depth::Fast.max_cycles_per_bit(), 5_000);
        assert_eq!(Jt9Depth::Normal.max_cycles_per_bit(), 10_000);
        assert_eq!(Jt9Depth::Deep.max_cycles_per_bit(), 30_000);
        assert_eq!(Jt9Depth::Max.max_cycles_per_bit(), 100_000);
    }

    #[test]
    fn default_is_fast() {
        assert_eq!(Jt9Depth::default(), Jt9Depth::Fast);
    }
}

/// Convert peakdt9 lag (in 27.78 Hz baseband samples) back to a
/// 12 kHz audio-sample index. Reverses the offsetting that peakdt9
/// applies when slicing c3 out of c2.
fn lag_to_audio_sample(lagpk: i64) -> usize {
    // peakdt9 places sample 0 of c3 at c2 index `lagpk - i0 - NSPSD + 1`
    // where i0 = 5 * NSPSD. Each 27.78 Hz sample equals NDOWN = 432
    // audio samples at 12 kHz.
    let i0 = 5 * 16i64;
    let nspsd = 16i64;
    let ndown = 432i64;
    let c2_offset = lagpk - i0 - nspsd + 1;
    // Audio-sample index of symbol-0 within the original audio.
    let sample = c2_offset.max(0) * ndown;
    let _ = FSAMPLE_DOWN; // silence unused
    sample as usize
}

#[cfg(test)]
#[allow(dead_code)]
mod gate_diag {
    use super::*;
    use crate::engine::DecodeContext;
    use crate::msg::Jt72Codec;
    use std::path::Path;

    /// Walk specific frequencies (the missing JT9 goldens) through the
    /// pipeline manually so we can see *where* they drop: peakdt9
    /// sync_score → afc9 syncpk + sync = (syncpk+1)/4 → schk → Fano
    /// hard_errors at three retry depths.
    #[test]
    #[ignore]
    fn probe_missing_goldens() {
        let path = Path::new(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/130418_1742.wav"
        ));
        if !path.exists() {
            eprintln!("skipping — sample not found");
            return;
        }
        let bytes = std::fs::read(path).unwrap();
        let dl = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
        let audio: Vec<f32> = bytes[44..44 + dl]
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32_768.0)
            .collect();

        let big_fft = AudioFft::build(&audio);

        for &(label, nominal_freq) in &[
            ("CQ GM7GAX IO75", 1119.0f32),
            ("CQ M0WAY IO82", 1290.0f32),
            ("TF3G N7MQ CN84 (control: passes)", 1186.0f32),
            ("K1JT KF4RWA 73 (control: passes)", 1224.0f32),
        ] {
            eprintln!("\n=== {} @ {:.1} Hz ===", label, nominal_freq);
            // Sweep ±2 Hz in 0.5 Hz steps so we see the freq with the
            // best sync, not just the nominal.
            for df_i in -4..=4i32 {
                let freq = nominal_freq + df_i as f32 * 0.5;
                let c2 = big_fft.downsam9(freq);
                let (_lagpk, peakdt_score, mut c3) = super::super::softsym::peakdt9(&c2);
                if !peakdt_score.is_finite() || peakdt_score < 0.3 {
                    continue;
                }
                let afc = super::super::softsym::afc9(&mut c3);
                super::super::softsym::twkfreq_poly(&mut c3, [afc.a0, afc.a1, 0.0]);
                let sync = (afc.syncpk + 1.0) / 4.0;
                let (schk, llrs, _snr_db) = super::super::softsym::llrs_from_c5(&c3);

                let mut hits = Vec::new();
                for &lim in &[10_000u64, 30_000, 100_000] {
                    let opts = FecOpts {
                        max_cycles_per_bit: Some(lim),
                        ..FecOpts::default()
                    };
                    let dec = ConvFano232.decode_soft(&llrs, &opts);
                    let s = match dec {
                        Some(r) => {
                            let mut p = [0u8; 72];
                            p.copy_from_slice(&r.info);
                            let m = Jt72Codec::default().unpack(&p, &DecodeContext::default());
                            format!(
                                "limit={} hard={} msg={:?}",
                                lim,
                                r.hard_errors,
                                m.map(|x| x.to_string())
                            )
                        }
                        None => format!("limit={} no-converge", lim),
                    };
                    hits.push(s);
                }
                eprintln!(
                    "  freq={:.1} peakdt={:.3} sync={:.3} schk={:.3} | {}",
                    freq,
                    peakdt_score,
                    sync,
                    schk,
                    hits.join(" / ")
                );
            }
        }
    }

    /// Task #24 follow-up: for the specific `jt9_awgn_m26_*.wav` files
    /// this crate misses, run `decode_at_baseband_with_fft_depth` at
    /// the *exact* frequency `coarse_search` actually returns (not an
    /// approximate 0.5 Hz grid) across every [`super::Jt9Depth`] tier,
    /// to see whether the production candidate itself ever converges.
    #[test]
    #[ignore]
    fn probe_exact_candidate_freq_m26_files() {
        use super::super::search::{SearchParams, coarse_search};
        use super::Jt9Depth;

        fn load_wav(path: &str) -> Vec<f32> {
            let bytes = std::fs::read(path).unwrap();
            let mut i = 12;
            loop {
                let id = &bytes[i..i + 4];
                let len =
                    u32::from_le_bytes([bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]])
                        as usize;
                if id == b"data" {
                    let start = i + 8;
                    let samples: &[u8] = &bytes[start..start + len];
                    return samples
                        .chunks_exact(2)
                        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                        .collect();
                }
                i += 8 + len + (len % 2);
            }
        }

        for n in [9, 13, 17] {
            let path = format!(
                concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/../embedded-poc/assets/jt9_sweep/jt9_awgn_m26_{:02}.wav"
                ),
                n
            );
            if !std::path::Path::new(&path).exists() {
                eprintln!("skipping m26_{n:02}.wav — sample not found (gitignored local corpus)");
                continue;
            }
            let mut audio = load_wav(&path);
            audio.resize(720_000, 0.0);

            let sp = SearchParams {
                score_threshold: 0.001,
                max_candidates: 50_000,
                ..Default::default()
            };
            let mut cands = coarse_search(&audio, 12_000, 0, &sp);
            cands.sort_unstable_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
            cands.truncate(5); // just the top few — the real signal should be near the top

            let big_fft = AudioFft::build(&audio);
            eprintln!("\n=== m26_{n:02}.wav: top candidates ===");
            for c in &cands {
                let mut results = Vec::new();
                for depth in [
                    Jt9Depth::Fast,
                    Jt9Depth::Normal,
                    Jt9Depth::Deep,
                    Jt9Depth::Max,
                ] {
                    let r = super::decode_at_baseband_with_fft_depth(&big_fft, c.freq_hz, depth);
                    results.push(format!("{depth:?}={}", r.is_some()));
                }
                eprintln!(
                    "  freq={:.3} score={:.4} | {}",
                    c.freq_hz,
                    c.score,
                    results.join(" ")
                );
            }
        }
    }

    /// Walk specific frequencies (the missing JT9 goldens) through the
    /// pipeline manually so we can see *where* they drop: peakdt9
    /// sync_score → afc9 syncpk + sync = (syncpk+1)/4 → schk → Fano
    /// hard_errors at three retry depths.
    #[test]
    #[ignore]
    fn probe_missing_m26_files() {
        fn load_wav(path: &str) -> Vec<f32> {
            let bytes = std::fs::read(path).unwrap();
            let mut i = 12;
            loop {
                let id = &bytes[i..i + 4];
                let len =
                    u32::from_le_bytes([bytes[i + 4], bytes[i + 5], bytes[i + 6], bytes[i + 7]])
                        as usize;
                if id == b"data" {
                    let start = i + 8;
                    let samples: &[u8] = &bytes[start..start + len];
                    return samples
                        .chunks_exact(2)
                        .map(|c| i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0)
                        .collect();
                }
                i += 8 + len + (len % 2);
            }
        }

        // 09, 13, 17: this crate misses these at Jt9Depth::Normal;
        // real `jt9 -d1` decodes all three. jt9sim's signal is always
        // at 1400 Hz.
        for n in [9, 13, 17] {
            let path = format!(
                concat!(
                    env!("CARGO_MANIFEST_DIR"),
                    "/../embedded-poc/assets/jt9_sweep/jt9_awgn_m26_{:02}.wav"
                ),
                n
            );
            if !std::path::Path::new(&path).exists() {
                eprintln!("skipping m26_{n:02}.wav — sample not found (gitignored local corpus)");
                continue;
            }
            eprintln!("\n=== jt9_awgn_m26_{n:02}.wav ===");
            let mut audio = load_wav(&path);
            audio.resize(720_000, 0.0);
            let big_fft = AudioFft::build(&audio);

            for df_i in -6..=6i32 {
                let freq = 1400.0 + df_i as f32 * 0.5;
                let c2 = big_fft.downsam9(freq);
                let (_lagpk, peakdt_score, mut c3) = peakdt9(&c2);
                if !peakdt_score.is_finite() || peakdt_score < 0.3 {
                    continue;
                }
                let afc = afc9(&mut c3);
                twkfreq_poly(&mut c3, [afc.a0, afc.a1, 0.0]);
                let sync = (afc.syncpk + 1.0) / 4.0;
                let (schk, llrs, _snr_db) = llrs_from_c5(&c3);

                let mut hits = Vec::new();
                for &lim in &[5_000u64, 10_000, 30_000, 100_000] {
                    let opts = FecOpts {
                        max_cycles_per_bit: Some(lim),
                        ..FecOpts::default()
                    };
                    let dec = ConvFano232.decode_soft(&llrs, &opts);
                    let s = match dec {
                        Some(r) => {
                            let mut p = [0u8; 72];
                            p.copy_from_slice(&r.info);
                            let m = Jt72Codec::default().unpack(&p, &DecodeContext::default());
                            format!(
                                "limit={} hard={} msg={:?}",
                                lim,
                                r.hard_errors,
                                m.map(|x| x.to_string())
                            )
                        }
                        None => format!("limit={lim} no-converge"),
                    };
                    hits.push(s);
                }
                eprintln!(
                    "  freq={freq:.1} peakdt={peakdt_score:.3} sync={sync:.3} schk={schk:.3} | {}",
                    hits.join(" / ")
                );
            }
        }
    }
}
