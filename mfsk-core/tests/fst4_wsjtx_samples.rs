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
use common::golden::GoldenEntry;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

fn sample_path() -> Option<PathBuf> {
    common::corpus::golden_path_or_upstream(
        "fst4/210115_0058.wav",
        Some("FST4+FST4W/210115_0058.wav"),
    )
}

struct Golden {
    // Only used by `fst4_60_diagnose_golden` below (an `--ignored`
    // diagnostic probe), which reads `freq_hz`/`dt_sec` but not `msg` —
    // kept for readability of the literal at its definition site.
    #[allow(dead_code)]
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
const SNR_TOL_DB: f32 = 3.0;

/// Both real signals on this recording, freq/dt/SNR from a local
/// `jt9 -7 -p 60 -b A -d 3` run over the vendored WAV (2026-08-14):
///
/// ```text
/// 0058  16  0.4 1331 `  CQ K9KFR EN71
/// 0058  -7  0.3 1101 `  CQ N5TM EL29
/// ```
///
/// SNR is kept at the finer precision from issue #255's FST4
/// real-formula port investigation (`SNRAUDIT_FST4_PROBE`
/// instrumentation added to `fst4_decode.f90`, not committed there:
/// -6.90 / 16.14 vs. the CLI's rounded -7 / 16). `±3.0` dB tolerance
/// is generous relative to the `~1-2` dB gap that investigation
/// actually landed on, since this is a regression gate, not a
/// precision claim.
static FST4_60_FULL_REFERENCE: &[GoldenEntry] = &[
    GoldenEntry {
        msg: "CQ N5TM EL29",
        freq_hz: Some(1101.0),
        dt_sec: Some(0.3),
        snr_db: Some(-6.90),
    },
    GoldenEntry {
        msg: "CQ K9KFR EN71",
        freq_hz: Some(1331.0),
        dt_sec: Some(0.4),
        snr_db: Some(16.14),
    },
];

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
    let candidates = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 0.0, None, 2000, 12_000.0);
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

/// Bake the WSJT-X golden's raw audio and its FST4-60A forward-FFT
/// cache to two flat binaries for an ESP32-S3 decoder-only bench
/// (issue #306).
///
/// Mirrors `wspr_bake_golden_baseband`
/// (`mfsk-core/tests/wspr_wsjtx_samples.rs`): `build_fft_cache`'s
/// `fft1_size = 746_496`-point transform is exactly the stage the
/// ESP-DSP backend cannot serve (`CONFIG_DSP_MAX_FFT_SIZE_8192`, and
/// WSPR's `decimate_to_baseband` hit the identical ceiling at a
/// larger size) — baking it here and feeding it back in via
/// `decode_frame`'s `precomputed_fft` parameter is what lets the
/// device measure `coarse_sync` + LLR/BP/OSD without that stage
/// existing on-device first, same shape issue #260 used for WSPR.
///
/// Two files, both little-endian (host and Xtensa are both LE, so the
/// device reads either with a plain transmute of `include_bytes!`):
///
/// ```text
/// fst4_60_golden_audio.bin:      i16[720000] LE           = 1 440 000 bytes
/// fst4_60_golden_fft_cache.bin:  (f32 re, f32 im)[746496]  = 5 971 968 bytes
/// ```
///
/// Run:
/// `cargo test -p mfsk-core --features full,internal-testing --release \
///  --test fst4_wsjtx_samples fst4_bake_golden_precomputed -- --ignored --nocapture`
#[test]
#[ignore = "asset generator — writes embedded-poc/assets/fst4_60_golden_{audio,fft_cache}.bin"]
fn fst4_bake_golden_precomputed() {
    use mfsk_core::engine::dsp::downsample::build_fft_cache;
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, decode_frame};
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    let Some(path) = sample_path() else {
        panic!("FST4 golden not found — this generator needs the real recording");
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");

    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);
    assert_eq!(
        fft_cache.len(),
        FST4_60A_DOWNSAMPLE.fft1_size,
        "fft_cache length is fixed by fft1_size"
    );

    let mut audio_bytes = Vec::with_capacity(audio.len() * 2);
    for &s in &audio {
        audio_bytes.extend_from_slice(&s.to_le_bytes());
    }
    let audio_out = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/fst4_60_golden_audio.bin"
    ));
    std::fs::write(&audio_out, &audio_bytes).expect("write audio asset");

    let mut cache_bytes = Vec::with_capacity(fft_cache.len() * 8);
    for c in &fft_cache {
        cache_bytes.extend_from_slice(&c.re.to_le_bytes());
        cache_bytes.extend_from_slice(&c.im.to_le_bytes());
    }
    let cache_out = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/fst4_60_golden_fft_cache.bin"
    ));
    std::fs::write(&cache_out, &cache_bytes).expect("write fft cache asset");

    eprintln!(
        "wrote {} ({} bytes = {} samples) and {} ({} bytes = {} complex bins)",
        audio_out.display(),
        audio_bytes.len(),
        audio.len(),
        cache_out.display(),
        cache_bytes.len(),
        fft_cache.len(),
    );

    // Round-trip check, on the host: re-load both files exactly as the
    // device bench will, and confirm `decode_frame` fed the baked
    // cache reaches the same 2 goldens as `DecodeRequest`'s own
    // fresh-computed path (same params as
    // `fst4_wsjtx_sample_precision_vs_reference_decoder`: freq_min
    // 100.0, freq_max 3000.0, sync_min 1.2, max_cand 50; `SYNC_Q_MIN`
    // (16) and the other defaults `DecodeRequest::new` sets —
    // `DecodeDepth::FULL`, `DecodeStrictness::Normal`, `EqMode::Off`,
    // no freq_hint — are threaded through explicitly here since this
    // test calls `decode_frame` directly rather than through
    // `DecodeRequest`).
    let reloaded_audio: Vec<i16> = audio_bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
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
    assert_eq!(reloaded_audio, audio, "audio asset round-trip mismatch");
    assert_eq!(
        reloaded_cache.len(),
        fft_cache.len(),
        "fft_cache asset round-trip length mismatch"
    );

    const SYNC_Q_MIN: u32 = 16;
    let (baked_results, _cache) = decode_frame::<Fst4s60>(
        &reloaded_audio,
        &FST4_60A_DOWNSAMPLE,
        100.0,
        3000.0,
        1.2,
        None,
        DecodeDepth::FULL,
        50,
        DecodeStrictness::Normal,
        EqMode::Off,
        SYNC_Q_MIN,
        Some(&reloaded_cache),
        None,
    );
    let mut baked_msgs: Vec<String> = baked_results
        .iter()
        .filter_map(|r| {
            let m77: &[u8; 77] = r.message77().try_into().ok()?;
            unpack77(m77)
        })
        .collect();
    baked_msgs.sort();

    let mut reference_msgs: Vec<String> = FST4_60_FULL_REFERENCE
        .iter()
        .map(|g| g.msg.to_string())
        .collect();
    reference_msgs.sort();

    assert_eq!(
        baked_msgs, reference_msgs,
        "decode_frame fed the baked audio+fft_cache must reach the same \
         decodes as the golden reference — a mismatch here means the \
         baked assets are not what the device bench will actually see"
    );
}

/// Bake every *refined* (post-`dedup_refined_candidates`) FST4-60A
/// candidate's `(cd0, freq_hz, i0, score)` to a flat binary, for a
/// decoder-only embedded bench that skips FFTs entirely (issues
/// #306/#307).
///
/// `fst4_bake_golden_precomputed` above lets the device skip
/// `build_fft_cache`'s wideband FFT; it still needs `coarse_sync`
/// (whose spectrogram FFT — `NSPS × 2 = 7776` for FST4-60A — isn't a
/// power of two, issue #307's actual finding) and `downsample_cached`
/// (whose inverse FFT, `fft2_size = 6912`, isn't either) to run before
/// LLR/BP/OSD are ever reached. Baking each real candidate already
/// refined — the same `(cd0, freq_hz, i0, score)` tuple
/// `process_candidate_basic_impl`'s `precomputed_refine` parameter
/// accepts — lets the device call the new
/// `engine::pipeline::process_candidate_precomputed` (issue #306/#307)
/// directly, with **no FFT anywhere in the on-device path**, so a
/// wall-clock number for the LLR/BP/OSD stage doesn't have to wait on
/// #307's non-power-of-2 FFT kernel.
///
/// Baked candidates are the same set `decode_frame_impl` actually
/// decodes — every raw `coarse_sync` candidate, refined via
/// `refine_candidate_position`, then reduced to one-per-true-position
/// by the identical near-duplicate rule `dedup_refined_candidates`
/// uses (`0.10 × tone_spacing_hz` in frequency, `±2` downsampled
/// samples, highest-`score` survivor kept) — replicated here rather
/// than exposed from the crate, since it's ~20 lines and exposing the
/// private `RefinedSurvivor` type alias for one caller wasn't worth
/// the extra public surface. A wall-clock number over anything other
/// than this real candidate population (e.g. only the 2 that decode)
/// would understate the cost the way an early WSPR estimate did
/// (issue #260: "the cost is not where any of us was looking" — most
/// of a real candidate loop's time goes to candidates that *fail*).
///
/// Layout, little-endian:
/// ```text
/// n_candidates: u32
/// per candidate:
///     coarse freq_hz, dt_sec, score:      f32, f32, f32   (12 B)
///     refined freq_hz, i0, score:         f32, i32, f32   (12 B)
///     cd0[fft2_size]:                     (f32 re, f32 im) × 6912  (55 296 B)
/// ```
///
/// Run:
/// `cargo test -p mfsk-core --features full,internal-testing --release \
///  --test fst4_wsjtx_samples fst4_bake_golden_refined_candidates -- --ignored --nocapture`
#[test]
#[ignore = "asset generator — writes embedded-poc/assets/fst4_60_golden_refined_candidates.bin"]
fn fst4_bake_golden_refined_candidates() {
    use mfsk_core::engine::dsp::downsample::build_fft_cache;
    use mfsk_core::engine::equalize::EqMode;
    use mfsk_core::engine::pipeline::{
        DecodeDepth, DecodeStrictness, process_candidate_precomputed, refine_candidate_position,
    };
    use mfsk_core::engine::sync::coarse_sync;
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;

    let Some(path) = sample_path() else {
        panic!("FST4 golden not found — this generator needs the real recording");
    };
    let audio = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let fft_cache = build_fft_cache(&audio, &FST4_60A_DOWNSAMPLE);

    const SYNC_Q_MIN: u32 = 16;
    let raw_candidates = coarse_sync::<Fst4s60>(&audio, 100.0, 3000.0, 1.2, None, 50, 12_000.0);
    eprintln!("coarse_sync: {} raw candidates", raw_candidates.len());

    // Refine every raw candidate, then apply dedup_refined_candidates'
    // exact rule (see this test's doc comment for why it's replicated
    // rather than exposed).
    let refined: Vec<(Vec<num_complex::Complex32>, f32, i32, f32)> = raw_candidates
        .iter()
        .map(|c| refine_candidate_position::<Fst4s60>(c, &fft_cache, &FST4_60A_DOWNSAMPLE))
        .collect();
    let freq_tol = 0.10 * FST4_60A_DOWNSAMPLE.tone_spacing_hz;
    const I0_TOL: i32 = 2;
    let mut order: Vec<usize> = (0..raw_candidates.len()).collect();
    order.sort_by(|&a, &b| {
        refined[b]
            .3
            .partial_cmp(&refined[a].3)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let mut kept_positions: Vec<(f32, i32)> = Vec::new();
    let mut survivors: Vec<usize> = Vec::new();
    for idx in order {
        let (_, f, i0, _) = &refined[idx];
        let dup = kept_positions
            .iter()
            .any(|&(kf, ki)| (f - kf).abs() < freq_tol && (i0 - ki).abs() <= I0_TOL);
        if !dup {
            kept_positions.push((*f, *i0));
            survivors.push(idx);
        }
    }
    // Stable order for reproducibility across bake runs, independent
    // of the score-descending order used only for dedup tie-breaking.
    survivors.sort_unstable();
    eprintln!(
        "dedup_refined_candidates: {} survivors of {} raw",
        survivors.len(),
        raw_candidates.len()
    );

    let mut bytes = Vec::new();
    bytes.extend_from_slice(&(survivors.len() as u32).to_le_bytes());
    for &idx in &survivors {
        let cand = &raw_candidates[idx];
        let (cd0, rf, ri0, rscore) = &refined[idx];
        assert_eq!(
            cd0.len(),
            FST4_60A_DOWNSAMPLE.fft2_size,
            "cd0 length is fixed by fft2_size"
        );
        bytes.extend_from_slice(&cand.freq_hz.to_le_bytes());
        bytes.extend_from_slice(&cand.dt_sec.to_le_bytes());
        bytes.extend_from_slice(&cand.score.to_le_bytes());
        bytes.extend_from_slice(&rf.to_le_bytes());
        bytes.extend_from_slice(&ri0.to_le_bytes());
        bytes.extend_from_slice(&rscore.to_le_bytes());
        for c in cd0 {
            bytes.extend_from_slice(&c.re.to_le_bytes());
            bytes.extend_from_slice(&c.im.to_le_bytes());
        }
    }

    let out = PathBuf::from(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/fst4_60_golden_refined_candidates.bin"
    ));
    std::fs::write(&out, &bytes).expect("write refined-candidates asset");
    eprintln!(
        "wrote {} ({} bytes = {} candidates)",
        out.display(),
        bytes.len(),
        survivors.len(),
    );

    // Round-trip check: re-load the baked bytes exactly as the device
    // bench will, and confirm process_candidate_precomputed over the
    // baked candidates reaches the same 2 goldens as
    // fst4_wsjtx_sample_precision_vs_reference_decoder's DecodeRequest
    // path (same freq window / sync_min / SYNC_Q_MIN / depth /
    // strictness / eq_mode as every other test/bake in this file).
    let mut off = 0usize;
    let n = u32::from_le_bytes(bytes[off..off + 4].try_into().unwrap()) as usize;
    off += 4;
    let mut baked_results = Vec::new();
    for _ in 0..n {
        let freq_hz = f32::from_le_bytes(bytes[off..off + 4].try_into().unwrap());
        let dt_sec = f32::from_le_bytes(bytes[off + 4..off + 8].try_into().unwrap());
        let score = f32::from_le_bytes(bytes[off + 8..off + 12].try_into().unwrap());
        let rf = f32::from_le_bytes(bytes[off + 12..off + 16].try_into().unwrap());
        let ri0 = i32::from_le_bytes(bytes[off + 16..off + 20].try_into().unwrap());
        let rscore = f32::from_le_bytes(bytes[off + 20..off + 24].try_into().unwrap());
        off += 24;
        let mut cd0 = Vec::with_capacity(FST4_60A_DOWNSAMPLE.fft2_size);
        for _ in 0..FST4_60A_DOWNSAMPLE.fft2_size {
            let re = f32::from_le_bytes(bytes[off..off + 4].try_into().unwrap());
            let im = f32::from_le_bytes(bytes[off + 4..off + 8].try_into().unwrap());
            cd0.push(num_complex::Complex32::new(re, im));
            off += 8;
        }
        let cand = mfsk_core::engine::sync::SyncCandidate {
            freq_hz,
            dt_sec,
            score,
        };
        if let Some(r) = process_candidate_precomputed::<Fst4s60>(
            &cand,
            &fft_cache,
            &FST4_60A_DOWNSAMPLE,
            DecodeDepth::FULL,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            (cd0, rf, ri0, rscore),
            false, // real SNR — this round-trip mirrors the host decode_frame path exactly
            false, // full nsym staircase — this round-trip mirrors the host decode_frame path exactly
        ) {
            baked_results.push(r);
        }
    }
    assert_eq!(off, bytes.len(), "round-trip byte accounting mismatch");

    let mut baked_msgs: Vec<String> = baked_results
        .iter()
        .filter_map(|r| {
            let m77: &[u8; 77] = r.message77().try_into().ok()?;
            unpack77(m77)
        })
        .collect();
    baked_msgs.sort();
    let mut reference_msgs: Vec<String> = FST4_60_FULL_REFERENCE
        .iter()
        .map(|g| g.msg.to_string())
        .collect();
    reference_msgs.sort();
    assert_eq!(
        baked_msgs, reference_msgs,
        "process_candidate_precomputed over the baked refined candidates must reach the \
         same decodes as the golden reference"
    );
}

/// Recall, precision, and SNR together: real `jt9 -7 -p 60` reports
/// exactly the two decodes in `FST4_60_FULL_REFERENCE` on this
/// recording, and so does this crate — full recall, budget 0, and
/// freq/dt/SNR must land within tolerance of jt9's own numbers. FST4
/// had no false-decode guard before this test, and no SNR check
/// before issue #255's real-formula port
/// (`fst4::baseline::fst4_snr_db`) — this single `assert_golden` call
/// covers what used to be three separate checks (a message+freq+dt
/// recall test with only 1 of the 2 golden entries populated, a
/// message-only precision test, and a standalone SNR-vs-jt9 test).
#[test]
fn fst4_wsjtx_sample_precision_vs_reference_decoder() {
    use common::golden::{DecodeView, GoldenSet, Tolerances, assert_golden};

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
            expected: FST4_60_FULL_REFERENCE,
            min_hits: 2,
            max_extra: 0,
        },
        Tolerances {
            freq_hz: FREQ_TOL_HZ,
            dt_sec: DT_TOL_SEC,
            snr_db: SNR_TOL_DB,
        },
        |d| DecodeView {
            msg: unpack77(d.message77()).unwrap_or_default(),
            freq_hz: d.freq_hz,
            dt_sec: d.dt_sec,
            snr_db: Some(d.snr_db),
        },
    );
}
