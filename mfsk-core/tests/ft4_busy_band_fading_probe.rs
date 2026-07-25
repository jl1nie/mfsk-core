//! Synthetic "busy FT4 band" scenario, mirroring FT8's qso3_busy.wav
//! shape (a crowd of stations plus one strong *fading* signal sitting
//! close to a weak target) — used to check whether FT4's SIC has the
//! same shallow-suppression-under-fading gap #178 found and fixed for
//! FT8's `decode_frame_subtract_with_ap`.
//!
//! FT4 has no real-recording golden WAV equivalent to qso3_busy.wav in
//! this repo, so this builds a synthetic scenario instead: a handful of
//! clean crowd signals scattered across the band, one STRONG signal
//! with Rayleigh fading applied (`common::channel::RayleighFlatChannel`,
//! already used elsewhere for the CCIR fading investigation, issue #72)
//! sitting close (40 Hz, ~2 tone-spacings — FT4 tone spacing is
//! 20.833 Hz vs FT8's 6.25 Hz) to a weak target, and asks whether
//! `ft4::decode::decode_frame_subtract` (which goes through
//! `core::pipeline::decode_frame_subtract` — constant-amplitude
//! `subtract_tones`, crude binary QSB gain, batch-not-sequential
//! subtract, per #179's survey) can recover the weak target after
//! removing the fading interferer.
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
/// signal 40 Hz from a weak target. Returns (audio, target_msg77).
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

    // Weak target, 40 Hz away (~2 FT4 tone-spacings) from the strong
    // interferer — close enough that its leakage matters.
    let target_msg = pack("CQ", "DL8YHR", "JO41");
    mix_i16(&mut audio, &tone_pcm(&target_msg, 1600.0, 4_500), pad);

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
