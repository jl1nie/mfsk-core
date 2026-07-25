//! Synthetic "busy FT4 band" scenario, mirroring FT8's qso3_busy.wav
//! shape (a crowd of stations plus one strong *fading* signal sitting
//! close to a weak target) — used to check whether FT4's SIC has the
//! same shallow-suppression-under-fading gap #178 found and fixed for
//! FT8's `decode_frame_subtract_with_ap`.
//!
//! # Status / how this investigation actually concluded
//!
//! This scenario found a real bug (old constant-amplitude subtract
//! recovered the target 0/10 seeds) and the #179 fix
//! (`subtract_tones_lpf_converge`, matching #178's FT8 approach)
//! improved that to 9/10 — but seed=4 exposed something worse: not
//! just "misses the target," the *whole residual* came back empty
//! (0 decodes at all) after converge-subtracting all 5 candidates,
//! vs. 2 decodes with a single shot. Cross-checked against real
//! WSJT-X `jt9 -5` on the same dumped WAV (`dump_failing_seed_wav`) —
//! it decodes the target fine, so this wasn't "too hard regardless."
//!
//! First reflex was to gate iteration count on `sync_cv > 0.3` (only
//! iterate candidates that look like they're fading). **Reverted —
//! this was the same mistake as the fixed-3x attempt in #178, just
//! one level removed: swapping one hand-picked magic number (a flat
//! iteration count) for another (a hand-picked gate threshold)
//! instead of understanding the actual mechanism.**
//!
//! Checked what real data does instead (as it should have been
//! checked first): `ft8_qso3_iteration_count_diag.rs` (real
//! qso3_busy.wav) and `ft4_wsjtx_sample_iteration_diag.rs` (real
//! WSJT-X FT4 sample, `samples/FT4/000000_000002.wav` — exists,
//! wasn't checked for until this investigation) both show natural
//! per-candidate iteration diversity (2-6 on FT8, 3-6 on FT4) with no
//! `sync_cv` correlation on *either* protocol. The flat, ungated
//! convergence stop rule works correctly on real signals; it's
//! *this* scenario's near-noiseless synthetic crowd tones (no
//! per-signal AWGN — only the fading channel added any noise, and
//! only to itself before mixing) that made every candidate pin near
//! the iteration cap, an artifact of the test construction, not the
//! algorithm. Bumping the added noise floor up to 6000 (comparable to
//! the 9000-amplitude crowd signals themselves) didn't reproduce the
//! natural diversity real data shows either — so this was left
//! unresolved rather than tuned further; the real-data checks are
//! what the shipped fix (`e7df249`, flat/ungated) is actually
//! validated against.
//!
//! Kept as a record of what was tried, not as a validated regression
//! gate — `busy_band_fading_baseline` is expected to show 9/10, one
//! genuine known-unreproduced-on-real-data edge case included.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --features full \
//!     --test ft4_busy_band_fading_probe -- --ignored --nocapture
//! ```
use std::collections::BTreeSet;

use mfsk_core::core::{FrameLayout, MessageCodec, MessageFields, ModulationParams};
use mfsk_core::ft4::decode::{DecodeDepth, decode_frame_subtract};
use mfsk_core::ft4::{Ft4, encode};
use mfsk_core::msg::Wsjt77Message;

#[allow(dead_code)]
mod common;
use common::channel::RayleighFlatChannel;

const NSPS: usize = <Ft4 as ModulationParams>::NSPS as usize;
const NN: usize = <Ft4 as FrameLayout>::N_SYMBOLS as usize;
const SLOT_SAMPLES: usize = 90_000;

fn pack(call1: &str, call2: &str, grid: &str) -> [u8; 77] {
    let bits = Wsjt77Message
        .pack(&MessageFields {
            call1: Some(call1.into()),
            call2: Some(call2.into()),
            grid: Some(grid.into()),
            ..MessageFields::default()
        })
        .expect("pack succeeds");
    let mut out = [0u8; 77];
    out.copy_from_slice(&bits);
    out
}

fn tone_pcm(msg77: &[u8; 77], freq_hz: f32, amp: i16) -> Vec<i16> {
    let itone = encode::message_to_tones(msg77);
    assert_eq!(itone.len(), NN);
    let pcm = encode::tones_to_i16(&itone, freq_hz, amp);
    assert_eq!(pcm.len(), NN * NSPS);
    pcm
}

fn mix_i16(audio: &mut [i16], pcm: &[i16], pad: usize) {
    for (i, &s) in pcm.iter().enumerate() {
        let idx = pad + i;
        if idx >= audio.len() {
            break;
        }
        let v = audio[idx] as i32 + s as i32;
        audio[idx] = v.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    }
}

/// Builds the scenario: a small crowd + one Rayleigh-faded strong
/// signal 100 Hz from a weak target, all riding on a common receiver
/// AWGN noise floor. Returns (audio, target_msg77).
///
/// Earlier revision had *zero* background noise on the crowd/target —
/// only the strong signal's own fading channel added any AWGN, and
/// only to itself before mixing. That's not how a real receiver
/// sounds. A shared noise floor was added (sigma swept 300 -> 3000 ->
/// 6000, comparable to the 9000-amplitude crowd signals themselves)
/// hoping it would reproduce the natural 2-6 iteration-count diversity
/// real signals show; it didn't — this scenario still pins most
/// candidates at 5-6/6 regardless. That sweep was abandoned rather
/// than pushed further (see the module doc): tuning a synthetic
/// scenario's noise level to match a target outcome is the same
/// mistake as tuning the algorithm's own thresholds, just moved one
/// level up. The noise floor stays here because it's more realistic
/// than none, not because it "fixed" anything.
fn build_scenario(seed: u64) -> (Vec<i16>, [u8; 77]) {
    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let mut audio = vec![0i16; SLOT_SAMPLES];

    // Crowd: 4 clean stations scattered across the band.
    let crowd = [
        (pack("CQ", "JQ1AAA", "PM95"), 900.0_f32, 9_000_i16),
        (pack("CQ", "JQ1BBB", "PM96"), 1250.0, 9_000),
        (pack("CQ", "JQ1CCC", "PM85"), 2100.0, 9_000),
        (pack("CQ", "JQ1DDD", "QN02"), 2450.0, 9_000),
    ];
    for (msg, freq, amp) in &crowd {
        mix_i16(&mut audio, &tone_pcm(msg, *freq, *amp), pad);
    }

    // Strong, fading interferer — the mechanism under test.
    let strong_msg = pack("CQ", "K1ABC", "FN42");
    let strong_pcm = tone_pcm(&strong_msg, 1500.0, 24_000);
    let mut strong_f32: Vec<f32> = strong_pcm.iter().map(|&s| s as f32).collect();
    RayleighFlatChannel::new(5.0, 60.0, seed).apply(&mut strong_f32);
    let strong_faded: Vec<i16> = strong_f32
        .iter()
        .map(|&s| s.clamp(i16::MIN as f32, i16::MAX as f32) as i16)
        .collect();
    mix_i16(&mut audio, &strong_faded, pad);

    // Weak target, 100 Hz away (~5 FT4 tone-spacings) from the strong
    // interferer — close enough that its leakage matters.
    let target_msg = pack("CQ", "DL8YHR", "JO41");
    mix_i16(&mut audio, &tone_pcm(&target_msg, 1600.0, 4_500), pad);

    // Shared receiver noise floor across the whole slot — every real
    // off-air recording has this; the earlier revision didn't.
    let mut audio_f32: Vec<f32> = audio.iter().map(|&s| s as f32).collect();
    common::channel::AwgnChannel::new(6000.0, seed.wrapping_add(0xA11CE)).apply(&mut audio_f32);
    let audio: Vec<i16> = audio_f32
        .iter()
        .map(|&s| s.clamp(i16::MIN as f32, i16::MAX as f32) as i16)
        .collect();

    (audio, target_msg)
}

#[test]
#[ignore]
fn busy_band_fading_baseline() {
    let mut hits = 0;
    const TRIALS: u64 = 10;
    for seed in 0..TRIALS {
        let (audio, target) = build_scenario(seed);
        let results = decode_frame_subtract(&audio, 100.0, 3000.0, 0.6, 15);
        let msgs: BTreeSet<Vec<u8>> = results.iter().map(|r| r.message77().to_vec()).collect();
        let hit = msgs.contains(target.as_slice());
        println!("seed={seed}  decoded={}  target_hit={hit}", results.len());
        if hit {
            hits += 1;
        }
    }
    println!("\ndecode_frame_subtract (current pipeline): {hits}/{TRIALS} target recoveries");
}

#[test]
#[ignore]
fn busy_band_fading_single_pass_reference() {
    // Sanity: without the strong fading interferer at all, does the
    // weak target decode cleanly? Confirms the scenario's SNR/spacing
    // choices aren't just "too hard regardless."
    use mfsk_core::ft4::decode::decode_frame_with_options;
    let target_msg = pack("CQ", "DL8YHR", "JO41");
    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let mut audio = vec![0i16; SLOT_SAMPLES];
    mix_i16(&mut audio, &tone_pcm(&target_msg, 1600.0, 4_500), pad);
    let results =
        decode_frame_with_options(&audio, 100.0, 3000.0, 0.6, None, DecodeDepth::BpAllOsd, 15);
    let hit = results
        .iter()
        .any(|r| r.message77() == target_msg.as_slice());
    println!("no-interferer reference: target_hit={hit}");
    assert!(
        hit,
        "weak target should decode cleanly with no interferer present"
    );
}

#[test]
#[ignore]
fn diag_crowd_only_no_strong_interferer() {
    use mfsk_core::ft4::decode::decode_frame_with_options;
    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let crowd = [
        (pack("CQ", "JQ1AAA", "PM95"), 900.0_f32, 9_000_i16),
        (pack("CQ", "JQ1BBB", "PM96"), 1250.0, 9_000),
        (pack("CQ", "JQ1CCC", "PM85"), 2100.0, 9_000),
        (pack("CQ", "JQ1DDD", "QN02"), 2450.0, 9_000),
    ];
    for (msg, freq, amp) in &crowd {
        mix_i16(&mut audio, &tone_pcm(msg, *freq, *amp), pad);
    }
    let target_msg = pack("CQ", "DL8YHR", "JO41");
    mix_i16(&mut audio, &tone_pcm(&target_msg, 1600.0, 4_500), pad);

    let results =
        decode_frame_with_options(&audio, 100.0, 3000.0, 0.6, None, DecodeDepth::BpAllOsd, 15);
    let hit = results
        .iter()
        .any(|r| r.message77() == target_msg.as_slice());
    println!(
        "crowd-only (no strong interferer): decoded={} target_hit={hit}",
        results.len()
    );
}

#[test]
#[ignore]
fn diag_strong_only_no_crowd() {
    use mfsk_core::ft4::decode::decode_frame_subtract;
    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let mut audio = vec![0i16; SLOT_SAMPLES];

    let strong_msg = pack("CQ", "K1ABC", "FN42");
    let strong_pcm = tone_pcm(&strong_msg, 1500.0, 24_000);
    let mut strong_f32: Vec<f32> = strong_pcm.iter().map(|&s| s as f32).collect();
    RayleighFlatChannel::new(5.0, 60.0, 0).apply(&mut strong_f32);
    let strong_faded: Vec<i16> = strong_f32
        .iter()
        .map(|&s| s.clamp(i16::MIN as f32, i16::MAX as f32) as i16)
        .collect();
    mix_i16(&mut audio, &strong_faded, pad);

    let target_msg = pack("CQ", "DL8YHR", "JO41");
    mix_i16(&mut audio, &tone_pcm(&target_msg, 1600.0, 4_500), pad);

    let results = decode_frame_subtract(&audio, 100.0, 3000.0, 0.6, 15);
    let hit = results
        .iter()
        .any(|r| r.message77() == target_msg.as_slice());
    println!(
        "strong-fading-only (no crowd): decoded={} target_hit={hit}",
        results.len()
    );
    for r in &results {
        println!("  freq={:.1} dt={:+.3}", r.freq_hz, r.dt_sec);
    }
}

#[test]
#[ignore]
fn diag_target_score_before_after_subtract() {
    use mfsk_core::core::sync::{SyncDims, make_costas_ref, score_costas_block};
    use mfsk_core::ft4::decode::decode_frame_subtract;
    use mfsk_core::ft4::subtract::{refine_signal_freq, subtract_signal_lpf};

    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let mut audio = vec![0i16; SLOT_SAMPLES];

    let strong_msg = pack("CQ", "K1ABC", "FN42");
    let strong_pcm = tone_pcm(&strong_msg, 1500.0, 24_000);
    let mut strong_f32: Vec<f32> = strong_pcm.iter().map(|&s| s as f32).collect();
    RayleighFlatChannel::new(5.0, 60.0, 0).apply(&mut strong_f32);
    let strong_faded: Vec<i16> = strong_f32
        .iter()
        .map(|&s| s.clamp(i16::MIN as f32, i16::MAX as f32) as i16)
        .collect();
    mix_i16(&mut audio, &strong_faded, pad);

    let target_msg = pack("CQ", "DL8YHR", "JO41");
    mix_i16(&mut audio, &tone_pcm(&target_msg, 1600.0, 4_500), pad);

    let d = SyncDims::of::<Ft4>();
    let blocks = <Ft4 as FrameLayout>::SYNC_MODE.blocks();
    let first = &blocks[0];
    let csync = make_costas_ref(first.pattern, d.ds_spb);
    let i0 = ((0.0f32 + Ft4::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
    let ds_cfg = mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
    let score_at = |a: &[i16], f: f32| -> f32 {
        let (cd0, _c) = mfsk_core::core::dsp::downsample::downsample(a, f, &ds_cfg);
        score_costas_block(&cd0, &csync, d.ds_spb, i0)
    };

    println!(
        "target(1540) score BEFORE any subtract: {:.1}",
        score_at(&audio, 1540.0)
    );
    println!(
        "strong(1500) score BEFORE any subtract:  {:.1}",
        score_at(&audio, 1500.0)
    );

    // Decode the strong signal and subtract it via the real pipeline
    // entry point (decode_frame_subtract) — inspect its own internal
    // effect by just calling subtract_signal_lpf directly here too,
    // for a controlled before/after on this exact audio buffer.
    let results = decode_frame_subtract(&audio, 100.0, 3000.0, 0.6, 15);
    println!("decode_frame_subtract results: {}", results.len());
    for r in &results {
        println!("  freq={:.1} dt={:+.3}", r.freq_hz, r.dt_sec);
    }

    // Separately: manual single-signal subtract + convergence check.
    let mut residual = audio.clone();
    let strong_hit = results
        .iter()
        .find(|r| r.message77() == strong_msg.as_slice())
        .expect("strong must decode");
    let refined_freq = refine_signal_freq(&residual, strong_hit);
    println!(
        "strong refined freq: {refined_freq:.2} (was {:.2})",
        strong_hit.freq_hz
    );
    for i in 1..=6 {
        subtract_signal_lpf(&mut residual, strong_hit);
        println!(
            "  after manual iter {i}: target(1540) score={:.1}  strong(1500) score={:.1}",
            score_at(&residual, 1540.0),
            score_at(&residual, refined_freq),
        );
    }
}

#[test]
#[ignore]
fn dump_failing_seed_wav() {
    // seed=4 is the one seed that still misses the target after the
    // #178/#179 fix (9/10 hit rate) — write it out so it can be tested
    // against the real WSJT-X `jt9 -5` (FT4) decoder directly, rather
    // than assuming 10/10 is the achievable ceiling.
    let (audio, target) = build_scenario(4);
    println!("target msg: {:?}", pack("CQ", "DL8YHR", "JO41") == target);

    let path = std::env::var("FT4_DUMP_WAV_PATH")
        .unwrap_or_else(|_| "/tmp/ft4_busy_band_seed4.wav".to_string());
    write_wav_i16(std::path::Path::new(&path), &audio, 12_000);
    println!(
        "wrote {path} ({} samples, {:.2}s)",
        audio.len(),
        audio.len() as f32 / 12_000.0
    );
}

/// Minimal mono 16-bit PCM WAV writer (no `hound` dev-dependency here).
fn write_wav_i16(path: &std::path::Path, samples: &[i16], sample_rate: u32) {
    use std::io::Write;
    let data_len = (samples.len() * 2) as u32;
    let mut f = std::fs::File::create(path).expect("create wav");
    f.write_all(b"RIFF").unwrap();
    f.write_all(&(36 + data_len).to_le_bytes()).unwrap();
    f.write_all(b"WAVE").unwrap();
    f.write_all(b"fmt ").unwrap();
    f.write_all(&16u32.to_le_bytes()).unwrap();
    f.write_all(&1u16.to_le_bytes()).unwrap(); // PCM
    f.write_all(&1u16.to_le_bytes()).unwrap(); // mono
    f.write_all(&sample_rate.to_le_bytes()).unwrap();
    f.write_all(&(sample_rate * 2).to_le_bytes()).unwrap(); // byte rate
    f.write_all(&2u16.to_le_bytes()).unwrap(); // block align
    f.write_all(&16u16.to_le_bytes()).unwrap(); // bits per sample
    f.write_all(b"data").unwrap();
    f.write_all(&data_len.to_le_bytes()).unwrap();
    for &s in samples {
        f.write_all(&s.to_le_bytes()).unwrap();
    }
}

#[test]
#[ignore]
fn diag_seed4_why_still_missing() {
    use mfsk_core::core::sync::{SyncDims, make_costas_ref, score_costas_block};
    use mfsk_core::ft4::decode::decode_frame_subtract;
    use mfsk_core::ft4::subtract::{refine_signal_freq, subtract_signal_lpf};

    let (audio, target) = build_scenario(4);
    let results = decode_frame_subtract(&audio, 100.0, 3000.0, 0.6, 15);
    println!("decode_frame_subtract on seed=4: {} results", results.len());
    for r in &results {
        println!(
            "  freq={:.1} dt={:+.3} snr={:+.1} sync_cv={:.3} hard_errors={}",
            r.freq_hz, r.dt_sec, r.snr_db, r.sync_cv, r.hard_errors
        );
    }
    let hit = results.iter().any(|r| r.message77() == target.as_slice());
    println!("target hit: {hit}");

    // Manually subtract every OTHER decoded signal from a fresh copy of
    // the audio (mirroring what the real multi-pass driver does
    // cumulatively) and see what's left at 1600 Hz.
    let mut residual = audio.clone();
    for r in &results {
        let refined_freq = refine_signal_freq(&residual, r);
        let mut rr = r.clone();
        rr.freq_hz = refined_freq;
        subtract_signal_lpf(&mut residual, &rr);
    }

    let d = SyncDims::of::<Ft4>();
    let blocks = <Ft4 as FrameLayout>::SYNC_MODE.blocks();
    let first = &blocks[0];
    let csync = make_costas_ref(first.pattern, d.ds_spb);
    let i0 = ((0.0f32 + Ft4::TX_START_OFFSET_S) * d.ds_rate).round() as i32;
    let ds_cfg = mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
    let mut freq = 1600.0 - 20.833 * 3.0;
    println!("\nresidual scan near 1600 Hz after subtracting all found decodes:");
    while freq <= 1600.0 + 20.833 * 3.0 {
        let (cd0, _c) = mfsk_core::core::dsp::downsample::downsample(&residual, freq, &ds_cfg);
        let score = score_costas_block(&cd0, &csync, d.ds_spb, i0);
        println!("  freq={freq:>8.2}  score={score:>16.4}");
        freq += 20.833 / 2.0;
    }

    // Does a second decode pass on this manually-cleaned residual find it?
    use mfsk_core::ft4::decode::{DecodeDepth, decode_frame_with_options};
    let pass2 = decode_frame_with_options(
        &residual,
        100.0,
        3000.0,
        0.5,
        None,
        DecodeDepth::BpAllOsd,
        15,
    );
    let hit2 = pass2.iter().any(|r| r.message77() == target.as_slice());
    println!(
        "\nmanual-residual re-decode: {} results, target hit: {hit2}",
        pass2.len()
    );
    for r in &pass2 {
        println!(
            "  freq={:.1} dt={:+.3} snr={:+.1}",
            r.freq_hz, r.dt_sec, r.snr_db
        );
    }
}

#[test]
#[ignore]
fn diag_seed4_converge_vs_single_shot() {
    use mfsk_core::core::dsp::subtract::subtract_tones_lpf_converge;
    use mfsk_core::ft4::decode::{
        DecodeDepth, FT4_SUBTRACT, decode_frame_subtract, decode_frame_with_options,
    };
    use mfsk_core::ft4::encode::message_to_tones;
    use mfsk_core::ft4::subtract::{refine_signal_freq, subtract_signal_lpf};

    let (audio, target) = build_scenario(4);
    let results = decode_frame_subtract(&audio, 100.0, 3000.0, 0.6, 15);

    for (label, iters) in [("single-shot (1x)", 1u32), ("converge (6x max)", 6u32)] {
        let mut residual = audio.clone();
        for r in &results {
            let refined_freq = refine_signal_freq(&residual, r);
            let mut rr = r.clone();
            rr.freq_hz = refined_freq;
            if iters == 1 {
                subtract_signal_lpf(&mut residual, &rr);
            } else {
                let msg: [u8; 77] = rr.message77().try_into().unwrap();
                let tones = message_to_tones(&msg);
                subtract_tones_lpf_converge(
                    &mut residual,
                    &tones,
                    rr.freq_hz,
                    rr.dt_sec,
                    &FT4_SUBTRACT,
                    700,
                    false,
                    iters,
                    1.0,
                );
            }
        }
        let pass2 = decode_frame_with_options(
            &residual,
            100.0,
            3000.0,
            0.5,
            None,
            DecodeDepth::BpAllOsd,
            15,
        );
        let hit = pass2.iter().any(|r| r.message77() == target.as_slice());
        println!(
            "[{label}] re-decode: {} results, target hit: {hit}",
            pass2.len()
        );
        for r in &pass2 {
            println!(
                "    freq={:.1} dt={:+.3} snr={:+.1}",
                r.freq_hz, r.dt_sec, r.snr_db
            );
        }
    }
}

#[test]
#[ignore]
fn diag_seed4_converge_param_sweep() {
    use mfsk_core::core::dsp::subtract::subtract_tones_lpf_converge;
    use mfsk_core::ft4::decode::{
        DecodeDepth, FT4_SUBTRACT, decode_frame_subtract, decode_frame_with_options,
    };
    use mfsk_core::ft4::encode::message_to_tones;
    use mfsk_core::ft4::subtract::refine_signal_freq;

    let (audio, target) = build_scenario(4);
    let results = decode_frame_subtract(&audio, 100.0, 3000.0, 0.6, 15);

    for (max_iters, min_step_db) in [
        (6u32, 1.0f32),
        (3, 1.0),
        (2, 1.0),
        (6, 2.0),
        (6, 3.0),
        (2, 2.0),
    ] {
        let mut residual = audio.clone();
        for r in &results {
            let refined_freq = refine_signal_freq(&residual, r);
            let msg: [u8; 77] = r.message77().try_into().unwrap();
            let tones = message_to_tones(&msg);
            subtract_tones_lpf_converge(
                &mut residual,
                &tones,
                refined_freq,
                r.dt_sec,
                &FT4_SUBTRACT,
                700,
                false,
                max_iters,
                min_step_db,
            );
        }
        let pass2 = decode_frame_with_options(
            &residual,
            100.0,
            3000.0,
            0.5,
            None,
            DecodeDepth::BpAllOsd,
            15,
        );
        let hit = pass2.iter().any(|r| r.message77() == target.as_slice());
        println!(
            "max_iters={max_iters} min_step_db={min_step_db}  re-decode={} target_hit={hit}",
            pass2.len()
        );
    }
}

#[test]
#[ignore]
fn diag_seed4_actual_iteration_counts() {
    use mfsk_core::core::dsp::subtract::subtract_tones_lpf_converge;
    use mfsk_core::ft4::decode::{FT4_SUBTRACT, decode_frame_subtract};
    use mfsk_core::ft4::encode::message_to_tones;
    use mfsk_core::ft4::subtract::refine_signal_freq;

    let (audio, _target) = build_scenario(4);
    let results = decode_frame_subtract(&audio, 100.0, 3000.0, 0.6, 15);

    let mut residual = audio.clone();
    println!("\nPer-candidate iteration count under flat (no sync_cv gate) 6x/1.0dB convergence:");
    for r in &results {
        let refined_freq = refine_signal_freq(&residual, r);
        let msg: [u8; 77] = r.message77().try_into().unwrap();
        let tones = message_to_tones(&msg);
        let applied = subtract_tones_lpf_converge(
            &mut residual,
            &tones,
            refined_freq,
            r.dt_sec,
            &FT4_SUBTRACT,
            700,
            false,
            6,
            1.0,
        );
        println!(
            "  freq={:.1} sync_cv={:.3} snr={:+.1}  -> iterations_applied={applied}",
            r.freq_hz, r.sync_cv, r.snr_db
        );
    }
}
