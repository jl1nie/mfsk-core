//! FT4 counterpart to `ft8_sic_rounds_recall.rs`: regression coverage
//! for issue #218's `.sic_rounds(n)` on FT4's
//! `engine::pipeline::decode_frame_subtract`, whose `&[1.0, 0.75,
//! 0.5][..max_rounds]` slice is genuinely new logic (not just a
//! rename) that the FT8-side golden/monotonicity test doesn't
//! exercise at all.
//!
//! Self-contained synthetic scenario (no external WSJT-X sample tree
//! dependency, unlike `ft4_wsjtx_samples.rs`) so this always runs in
//! CI: six stations of varying amplitude, several close enough in
//! frequency that sequential subtraction across rounds matters — the
//! weakest pair only clears once a stronger neighbour has been
//! subtracted in an earlier round.
//!
//! **`ft4_sic_rounds_recall_is_monotonic`** — `sic_rounds(1)`'s decode
//! set must be a subset of `sic_rounds(2)`'s, which must be a subset
//! of `sic_rounds(3)`'s, mirroring the FT8-side invariant. Also
//! asserts `sic_rounds(3)` actually recovers more than `sic_rounds(1)`
//! on this scene (i.e. the scenario is discriminating, not vacuously
//! monotonic because every round already finds everything).
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core \
//!     --features fft-rustfft,ft4 \
//!     --test ft4_sic_rounds_recall -- --nocapture
//! ```
#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::collections::BTreeSet;

use mfsk_core::engine::{FrameLayout, MessageCodec, MessageFields, ModulationParams};
use mfsk_core::ft4::{Ft4, encode};
use mfsk_core::msg::Wsjt77Message;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;

const NSPS: usize = <Ft4 as ModulationParams>::NSPS as usize;
const NN: usize = <Ft4 as FrameLayout>::N_SYMBOLS as usize;
const SLOT_SAMPLES: usize = 90_000; // 7.5 s x 12 kHz

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

fn mix_i16(audio: &mut [i16], msg77: &[u8; 77], freq_hz: f32, amp: i16, pad: usize) {
    let itone = encode::message_to_tones(msg77);
    assert_eq!(itone.len(), NN);
    let pcm = encode::tones_to_i16(&itone, freq_hz, amp);
    assert_eq!(pcm.len(), NN * NSPS);
    for (i, &s) in pcm.iter().enumerate() {
        let idx = pad + i;
        if idx >= audio.len() {
            break;
        }
        let v = audio[idx] as i32 + s as i32;
        audio[idx] = v.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    }
}

/// Six stations at three amplitude tiers, several close enough (20-40
/// Hz, a couple of FT4 tone-spacings) that a weaker one only surfaces
/// once a stronger neighbour has been subtracted in an earlier round.
fn build_busy_scene(seed: u64) -> (Vec<i16>, Vec<[u8; 77]>) {
    let pad = (<Ft4 as FrameLayout>::TX_START_OFFSET_S * 12_000.0) as usize;
    let mut audio = vec![0i16; SLOT_SAMPLES];

    let stations: &[(&str, &str, &str, f32, i16)] = &[
        ("CQ", "JQ1AAA", "PM95", 500.0, 22_000),
        ("CQ", "JQ1BBB", "PM96", 900.0, 22_000),
        ("CQ", "JQ1CCC", "PM85", 1300.0, 18_000),
        ("CQ", "JQ1DDD", "QN02", 1330.0, 8_000),
        ("CQ", "JQ1EEE", "QN12", 1800.0, 14_000),
        ("CQ", "JQ1FFF", "QN22", 1825.0, 5_500),
    ];

    let mut msgs = Vec::with_capacity(stations.len());
    for (call1, call2, grid, freq, amp) in stations {
        let msg = pack(call1, call2, grid);
        mix_i16(&mut audio, &msg, *freq, *amp, pad);
        msgs.push(msg);
    }

    let mut audio_f32: Vec<f32> = audio.iter().map(|&s| s as f32).collect();
    common::channel::AwgnChannel::new(4500.0, seed).apply(&mut audio_f32);
    let audio: Vec<i16> = audio_f32
        .iter()
        .map(|&s| s.clamp(i16::MIN as f32, i16::MAX as f32) as i16)
        .collect();

    (audio, msgs)
}

fn decode_messages(audio: &[i16], n_rounds: usize) -> BTreeSet<String> {
    DecodeRequest::<Ft4>::new(audio, 100.0, 2700.0, 0.6, 50)
        .sic_rounds(n_rounds)
        .decode()
        .results
        .iter()
        .filter_map(|r| {
            let mut msg77 = [0u8; 77];
            msg77.copy_from_slice(r.message77());
            unpack77(&msg77)
        })
        .collect()
}

#[test]
fn ft4_sic_rounds_recall_is_monotonic() {
    let (audio, _msgs) = build_busy_scene(0);

    let r1 = decode_messages(&audio, 1);
    let r2 = decode_messages(&audio, 2);
    let r3 = decode_messages(&audio, 3);

    eprintln!("sic_rounds(1): {} decodes: {r1:?}", r1.len());
    eprintln!("sic_rounds(2): {} decodes: {r2:?}", r2.len());
    eprintln!("sic_rounds(3): {} decodes: {r3:?}", r3.len());

    assert!(
        r1.is_subset(&r2),
        "sic_rounds(1) found a decode sic_rounds(2) missed: {:?}",
        r1.difference(&r2).collect::<Vec<_>>()
    );
    assert!(
        r2.is_subset(&r3),
        "sic_rounds(2) found a decode sic_rounds(3) missed: {:?}",
        r2.difference(&r3).collect::<Vec<_>>()
    );
    assert!(
        r1.len() <= r2.len() && r2.len() <= r3.len(),
        "recall not monotonic: r1={} r2={} r3={}",
        r1.len(),
        r2.len(),
        r3.len()
    );

    // Scenario must actually discriminate between round counts — if
    // round 1 already found everything round 3 does, the subset
    // assertions above pass vacuously and this test isn't exercising
    // `&[1.0, 0.75, 0.5][..max_rounds]`'s slicing at all.
    assert!(
        r1.len() < r3.len(),
        "scenario not discriminating: sic_rounds(1) already matched \
         sic_rounds(3) ({} decodes) — strengthen the close-frequency \
         pairs so subtraction across rounds actually matters",
        r3.len()
    );
}
