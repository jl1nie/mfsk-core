//! FT4 real-world signal validation against the WSJT-X sample
//! recording shipped under `WSJT-X/samples/FT4/000000_000002.wav`.
//!
//! The sample is 12 kHz mono PCM-16, 6.048 s long (shorter than the
//! nominal 7.5 s FT4 slot — we zero-pad to `SLOT_SAMPLES = 90 000`
//! before running the decoder). WSJT-X's own GUI decode of this file
//! yields six FT4 messages (`reference_ft4_wsjtx_sample_decode.md`);
//! a real `jt9 -5 -p 15 -L 300 -H 2700 -d 3` CLI run over the full
//! 300–2700 Hz band finds all 14 signals actually present, with
//! freq/dt/SNR — that fuller set is `FT4_FULL_REFERENCE` below, and
//! is what every test in this file checks against via
//! `common::golden::assert_golden`.
//!
//! This is the FT4 counterpart to `q65_wsjtx_samples.rs`. It exists
//! to catch regressions in the *generic* DSP path that FT4 shares
//! with FT8: in particular, the `engine::dsp::subtract` rewrite that
//! turned on GFSK shaping for FT4 (commit cec9472) had no real-WAV
//! coverage before this test was added.
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path so developers cloning only `mfsk-core` won't see
//! a failure they can't fix.

#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::PathBuf;

use mfsk_core::ft4::Ft4;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::golden::GoldenEntry;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s × 12 kHz

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream("ft4/000000_000002.wav", Some("FT4/000000_000002.wav"))
}

const FREQ_TOL_HZ: f32 = 12.0;
const DT_TOL_SEC: f32 = 0.3;

/// Every decode real `jt9 -5 -p 15 -L 300 -H 2700 -d 3` reports on
/// this recording — freq/dt/SNR captured from that exact run
/// (2026-08-14), not estimated. Previously message-only
/// (`GoldenEntry::msg`): `assert_golden` was checking recall and
/// precision on text alone, so a decode landing on the right
/// message at the *wrong* frequency or time would have passed
/// silently — the same blind spot this file's own SNR test
/// (below) already closed for SNR specifically.
static FT4_FULL_REFERENCE: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "AC6BW KR9A R 559 WI",
        freq_hz: Some(2300.0),
        dt_sec: Some(0.2),
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "CQ RU AB5XS EM12",
        freq_hz: Some(560.0),
        dt_sec: Some(-0.1),
        snr_db: Some(-7.0),
    },
    GoldenEntry {
        msg: "CQ RU N9OY EN43",
        freq_hz: Some(1640.0),
        dt_sec: Some(-0.2),
        snr_db: Some(-4.0),
    },
    GoldenEntry {
        msg: "CQ RU W0FRC DM79",
        freq_hz: Some(2560.0),
        dt_sec: Some(-0.3),
        snr_db: Some(-15.0),
    },
    GoldenEntry {
        msg: "K1JT WB4HXE 559 GA",
        freq_hz: Some(1910.0),
        dt_sec: Some(0.2),
        snr_db: Some(-11.0),
    },
    GoldenEntry {
        msg: "KB0VHA KA1YQC R 539 MA",
        freq_hz: Some(1149.0),
        dt_sec: Some(0.3),
        snr_db: Some(16.0),
    },
    GoldenEntry {
        msg: "N1TRK KB7RUQ RR73",
        freq_hz: Some(422.0),
        dt_sec: Some(-0.4),
        snr_db: Some(-10.0),
    },
    GoldenEntry {
        msg: "N1TRK N4FKH 569 VA",
        freq_hz: Some(297.0),
        dt_sec: Some(-0.2),
        snr_db: Some(-10.0),
    },
    GoldenEntry {
        msg: "NI6G W7DRW 569 AZ",
        freq_hz: Some(2567.0),
        dt_sec: Some(-0.3),
        snr_db: Some(-7.0),
    },
    GoldenEntry {
        msg: "NZ7P WA7JAY 589 CA",
        freq_hz: Some(727.0),
        dt_sec: Some(0.1),
        snr_db: Some(-13.0),
    },
    GoldenEntry {
        msg: "VE3LON K7RL R 549 WA",
        freq_hz: Some(2067.0),
        dt_sec: Some(0.3),
        snr_db: Some(5.0),
    },
    GoldenEntry {
        msg: "W7BOB KJ7G RR73",
        freq_hz: Some(2413.0),
        dt_sec: Some(-0.4),
        snr_db: Some(-17.0),
    },
    GoldenEntry {
        msg: "W9JA PY2APK RRR",
        freq_hz: Some(520.0),
        dt_sec: Some(-0.3),
        snr_db: Some(-9.0),
    },
    GoldenEntry {
        msg: "WD9IGY KX1X 73",
        freq_hz: Some(2310.0),
        dt_sec: Some(0.2),
        snr_db: Some(-1.0),
    },
];

/// Wide-band recall: with `sic_rounds(3)` and a 100–2700 Hz search
/// (vs. `ft4_wsjtx_sample_reaches_jt9_parity_with_sic`'s narrower
/// 300–2700 Hz + `sic_rounds(2)`), this reaches the same full 14/14
/// jt9 parity, zero phantoms — verified 2026-08-14, so the budget is
/// `max_extra: 0` rather than a looser recall-only check.
#[test]
fn ft4_wsjtx_sample_recall_vs_golden() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

    let Some(path) = sample_path() else {
        eprintln!(
            "skipping: WSJT-X FT4 sample not found at ../../WSJT-X/samples/FT4/000000_000002.wav"
        );
        return;
    };

    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    // Zero-pad / truncate to the FT4 slot length the decoder expects.
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);

    // Wide search — FT4 audio band is ~100..2700 Hz.
    // `max_cand=100` matches WSJT-X's own `getcandidates4.f90`
    // `MAXCAND=100`: since `engine::ft4_coarse::ft4_coarse_sync`
    // (dapper-soaring-nest plan, Phase 1) emits one candidate per
    // frequency-domain peak instead of the old generic Costas-lag
    // search's up-to-8-per-frequency, this real 6-signal WAV needed
    // only 31 candidates even at the old `max_cand=2000` ceiling — no
    // large safety margin required any more. (The stale comment this
    // replaced described the old search's redundancy-driven budget,
    // now obsolete.)
    let out = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .sic_rounds(3)
        .decode();

    assert_golden(
        &out.results,
        &GoldenSet {
            name: "FT4 000000_000002.wav (wide band, sic_rounds(3))",
            expected: FT4_FULL_REFERENCE,
            // Full parity with real jt9, same as the sic_rounds(2)
            // narrow-band test — the wider search doesn't cost
            // precision here. If this drops, the FT4 path (likely
            // subtract / GFSK shaping) has regressed.
            min_hits: 14,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: 2.0,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}

/// Precision: the decoder must emit **nothing** the reference decoder
/// doesn't. Also pins reported SNR against real `jt9`'s.
///
/// FT4 had no false-decode guard of any kind. A real local
/// `jt9 -5 -p 15 -L 300 -H 2700 -d 3` run over the same WAV finds the
/// 14 `FT4_FULL_REFERENCE` entries below. Against that set this crate
/// emits **11 decodes, all real, zero phantom** — so the budget is 0,
/// and the recall floor states the 3 it does not reach rather than
/// hiding them.
///
/// FT4 was also, for a while, the only implemented protocol with
/// **no SNR check of any kind** — neither against a reference decoder
/// nor against an injected value. That is the gap issue #255 walked
/// into: FT4's reported SNR was ~6.9 dB low for as long as the crate
/// had existed, because `engine::llr::compute_snr_db`'s generic
/// adjacent-tone heuristic stood in for `ft4_decode.f90`'s real
/// `10·log10(candidate_score − 1) − 14.8`, and nothing measured it.
/// `FT4_FULL_REFERENCE.snr_db` pins the fixed formula
/// (`engine::pipeline::ft4_snr_db`) via the `Tolerances { snr_db: 2.0,
/// .. }` below. Measured agreement across the 11 decodes this crate
/// and `jt9` share, spanning −17…+16 dB: mean error +0.06 dB, max
/// |error| 0.46 dB — the closest SNR agreement of any protocol here.
/// 2 dB is loose enough that a refactor or a rebuilt corpus won't trip
/// it, tight enough that the ~6.9 dB error #255 fixed could never pass
/// again.
///
/// Written through `common::golden::assert_golden`, which asserts
/// recall, precision, and SNR together — see its module doc for why
/// that is one call and not three.
#[test]
fn ft4_wsjtx_sample_precision_vs_reference_decoder() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

    let Some(path) = sample_path() else {
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let out = DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50).decode();

    assert_golden(
        &out.results,
        &GoldenSet {
            name: "FT4 000000_000002.wav",
            expected: FT4_FULL_REFERENCE,
            // 11 of jt9's 14. Raising this is a sensitivity win;
            // lowering it is a regression.
            min_hits: 11,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: 2.0,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}

/// With SIC enabled, FT4 reaches **full parity with real `jt9`** on
/// this recording — 14/14, zero phantoms.
///
/// `ft4_wsjtx_sample_precision_vs_reference_decoder` above asserts
/// 11/14, which is what the *default* single-pass strategy reaches.
/// That is not a decoder deficiency: all three it misses are weak
/// signals sitting inside a stronger neighbour's 83 Hz occupied
/// bandwidth, which is exactly what successive interference
/// cancellation exists to recover:
///
/// | missed | SNR | masked by | Δf |
/// |---|---:|---|---:|
/// | `W9JA PY2APK RRR` @ 520 Hz | -9 | `CQ RU AB5XS EM12` @ 560, -7 | 40 Hz |
/// | `AC6BW KR9A R 559 WI` @ 2300 Hz | -15 | `WD9IGY KX1X 73` @ 2310, -1 | 10 Hz |
/// | `CQ RU W0FRC DM79` @ 2560 Hz | -15 | `NI6G W7DRW 569 AZ` @ 2567, -7 | 7 Hz |
///
/// The comparison against `jt9` was therefore not like-for-like:
/// real `jt9` runs its own multi-pass subtraction by default, and
/// `DecodeRequest`'s default strategy is `__single_pass` (for FT8
/// too — `sic_rounds` is a field default that only takes effect once
/// `.sic_rounds()` switches the strategy). A caller wanting
/// WSJT-X-equivalent recall must ask for it.
///
/// Two rounds suffice; three costs more and finds nothing extra.
/// Measured on this file: 5.0 ms default → 71.3 ms with
/// `.sic_rounds(2)`, against a 7.5 s slot — about 1 % of the slot for
/// a 27 % recall gain.
#[test]
fn ft4_wsjtx_sample_reaches_jt9_parity_with_sic() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

    let Some(path) = sample_path() else {
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let out = DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50)
        .sic_rounds(2)
        .decode();

    assert_golden(
        &out.results,
        &GoldenSet {
            name: "FT4 000000_000002.wav + sic_rounds(2)",
            expected: FT4_FULL_REFERENCE,
            // Full parity with real jt9. This is a hard floor: SIC is
            // the whole point of this test, so 13 is a regression.
            min_hits: 14,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: 2.0,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}

// ── On-device bench assets (issue-TBD, FT4 embedded feasibility) ─────────

/// Search parameters the baked assets — and therefore the on-device
/// `ft4-bench` — are generated for.
///
/// Deliberately the same band / `sync_min` / `max_cand` as
/// `ft4_wsjtx_sample_recall_and_precision` above, minus its
/// `.sic_rounds(3)`: embedded FT8 ships a single pass (`dual_core` has
/// no subtract path at all), so a multi-pass number would not describe
/// what the board would actually run.
///
/// **Mirrored in `embedded-shared::apps::ft4_bench`.** Changing one
/// without the other makes the device measure a different search than
/// the assets were baked for, which no assertion here can catch.
#[cfg(feature = "internal-testing")]
mod bench_assets {
    pub const FREQ_MIN_HZ: f32 = 100.0;
    pub const FREQ_MAX_HZ: f32 = 2700.0;
    pub const SYNC_MIN: f32 = 0.05;
    pub const MAX_CAND: usize = 100;
    /// `ft4::decode`'s own private `SYNC_Q_MIN` — FT4 has 16 sync
    /// symbols (4 × Costas-4) and requires at least half correct.
    pub const SYNC_Q_MIN: u32 = 8;
}

/// Bake the FT4 golden's slot audio, wideband FFT cache and
/// coarse-candidate list for the on-device `ft4-bench`.
///
/// Same shape and the same reason as
/// `fst4_wsjtx_samples.rs::fst4_bake_golden_precomputed`: the wideband
/// transform `build_fft_cache` runs — `fft1_size = 92_160` for FT4 — is
/// not a power of two and is far past `CONFIG_DSP_MAX_FFT_SIZE_8192`,
/// so ESP-DSP cannot serve it at all. Baking it here and feeding it
/// back through `decode_frame`'s `precomputed_fft` seam (which
/// `ft4/decode.rs` already threads) is what lets the device measure the
/// per-candidate work without that stage existing on-device first.
///
/// What the device still computes for itself, and why this is a
/// different split from FST4's decoder-only bench:
///
/// - `downsample_cached`'s inverse transform (`fft2_size = 5120`) —
///   served by `engine::dsp::fft_mixed_5120` (1024 × 5), added for
///   exactly this.
/// - `engine::sync2d::ft4_sync_search` — no FFT at all, and the
///   dominant per-candidate cost by a wide margin on host
///   (`ft4_diag_candidate_cost_split`: ~123 ms of ~150 ms over 50
///   candidates). Measuring it is the whole point.
/// - `engine::llr::symbol_spectra`'s per-symbol DFT: `ds_spb =
///   NSPS/NDOWN = 32`, a power of two, so no new kernel needed.
///
/// The one stage neither computed nor baked is `ft4_coarse_sync`'s own
/// `NFFT1 = 2304` (= 256 × 9) periodogram — the candidate *list* is
/// baked instead. It measured 0.3 ms on host over the whole slot, so
/// excluding it understates the device total by roughly that times the
/// device/host ratio; the bench says so in its own output rather than
/// leaving the reader to assume the number is a whole slot.
///
/// Three files, all little-endian (host and Xtensa are both LE):
///
/// ```text
/// ft4_golden_audio.bin:      i16[90000]                  =   180 000 bytes
/// ft4_golden_fft_cache.bin:  (f32 re, f32 im)[92160]     =   737 280 bytes
/// ft4_golden_candidates.bin: u32 n, then n × 3 × f32     =  12 + 12·n bytes
/// ```
///
/// Run:
/// `cargo test -p mfsk-core --features full,internal-testing --release \
///  --test ft4_wsjtx_samples ft4_bake_golden_precomputed -- --ignored --nocapture`
#[test]
#[ignore = "asset generator — writes embedded-poc/assets/ft4_golden_{audio,fft_cache,candidates}.bin"]
#[cfg(feature = "internal-testing")]
fn ft4_bake_golden_precomputed() {
    use mfsk_core::engine::dsp::downsample::build_fft_cache;
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::ft4_coarse::ft4_coarse_sync;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_basic};
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;

    use bench_assets::*;

    let Some(path) = sample_path() else {
        panic!("FT4 golden not found — this generator needs the real recording");
    };
    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);

    let candidates = ft4_coarse_sync(&audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    assert!(
        !candidates.is_empty(),
        "coarse stage found nothing — the assets would be useless"
    );

    let fft_cache = build_fft_cache(&audio, &FT4_DOWNSAMPLE);
    assert_eq!(
        fft_cache.len(),
        FT4_DOWNSAMPLE.fft1_size,
        "fft_cache length is fixed by fft1_size"
    );

    let asset = |name: &str| {
        PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/"
        ))
        .join(name)
    };

    let mut audio_bytes = Vec::with_capacity(audio.len() * 2);
    for &s in &audio {
        audio_bytes.extend_from_slice(&s.to_le_bytes());
    }
    std::fs::write(asset("ft4_golden_audio.bin"), &audio_bytes).expect("write audio asset");

    let mut cache_bytes = Vec::with_capacity(fft_cache.len() * 8);
    for c in &fft_cache {
        cache_bytes.extend_from_slice(&c.re.to_le_bytes());
        cache_bytes.extend_from_slice(&c.im.to_le_bytes());
    }
    std::fs::write(asset("ft4_golden_fft_cache.bin"), &cache_bytes).expect("write fft cache asset");

    let mut cand_bytes = Vec::with_capacity(4 + candidates.len() * 12);
    cand_bytes.extend_from_slice(&(candidates.len() as u32).to_le_bytes());
    for c in &candidates {
        cand_bytes.extend_from_slice(&c.freq_hz.to_le_bytes());
        cand_bytes.extend_from_slice(&c.dt_sec.to_le_bytes());
        cand_bytes.extend_from_slice(&c.score.to_le_bytes());
    }
    std::fs::write(asset("ft4_golden_candidates.bin"), &cand_bytes).expect("write candidate asset");

    eprintln!(
        "wrote audio {} B ({} samples), fft_cache {} B ({} bins), candidates {} B ({} cands)",
        audio_bytes.len(),
        audio.len(),
        cache_bytes.len(),
        fft_cache.len(),
        cand_bytes.len(),
        candidates.len(),
    );

    // Round-trip: reload all three exactly as the device bench will,
    // and confirm the candidate loop over the reloaded assets reaches
    // the same decodes as the loop over the in-memory originals. This
    // validates the *assets*, not the decoder — a fixed decode count
    // would belong in a recall test, and there are three above.
    let reloaded_audio: Vec<i16> = audio_bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|b| i16::from_le_bytes(*b))
        .collect();
    assert_eq!(reloaded_audio, audio, "audio asset round-trip mismatch");

    let reloaded_cache: Vec<num_complex::Complex32> = cache_bytes
        .as_chunks::<8>()
        .0
        .iter()
        .map(|b| {
            num_complex::Complex32::new(
                f32::from_le_bytes(b[0..4].try_into().unwrap()),
                f32::from_le_bytes(b[4..8].try_into().unwrap()),
            )
        })
        .collect();
    assert_eq!(
        reloaded_cache.len(),
        fft_cache.len(),
        "fft_cache asset round-trip length mismatch"
    );

    let n = u32::from_le_bytes(cand_bytes[0..4].try_into().unwrap()) as usize;
    let reloaded_cands: Vec<SyncCandidate> = cand_bytes[4..]
        .as_chunks::<12>()
        .0
        .iter()
        .take(n)
        .map(|b| SyncCandidate {
            freq_hz: f32::from_le_bytes(b[0..4].try_into().unwrap()),
            dt_sec: f32::from_le_bytes(b[4..8].try_into().unwrap()),
            score: f32::from_le_bytes(b[8..12].try_into().unwrap()),
        })
        .collect();
    assert_eq!(reloaded_cands.len(), candidates.len(), "candidate count");

    // The exact loop `decode_frame_impl` runs for FT4 — including its
    // `known = &[]` (FT4 decodes raw candidates directly and dedups
    // afterwards), so the bench is not accidentally cheaper.
    let run = |cands: &[SyncCandidate], cache: &[num_complex::Complex32], depth: DecodeDepth| {
        let mut msgs: Vec<String> = cands
            .iter()
            .filter_map(|cand| {
                let r = process_candidate_basic::<Ft4>(
                    cand,
                    cache,
                    &FT4_DOWNSAMPLE,
                    depth,
                    DecodeStrictness::Normal,
                    &[],
                    EqMode::Off,
                    SYNC_Q_MIN,
                )?;
                let m77: &[u8; 77] = r.message77().try_into().ok()?;
                unpack77(m77)
            })
            .collect();
        msgs.sort();
        msgs.dedup();
        msgs
    };

    for (label, depth) in [
        ("FULL", DecodeDepth::FULL),
        ("EMBEDDED", DecodeDepth::EMBEDDED),
    ] {
        let fresh = run(&candidates, &fft_cache, depth);
        let baked = run(&reloaded_cands, &reloaded_cache, depth);
        assert_eq!(
            baked, fresh,
            "{label}: the baked assets must decode to exactly what the \
             freshly-computed ones do — a mismatch means the device \
             bench will not be running the search these assets describe"
        );
        eprintln!("{label}: {} distinct decodes {:?}", fresh.len(), fresh);
    }
}
