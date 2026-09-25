//! `ft8b.f90`'s post-CRC message filter (#439): with no contest active, a
//! standard message carrying `/R` is dropped; `.contest(true)` (WSJT-X's
//! `ncontest != 0`) keeps it.
//!
//! The signals are clean, so a missing decode is the filter's doing and not
//! sensitivity. The control message shows the filter does not touch an
//! ordinary one.
#![cfg(all(feature = "fft-rustfft", feature = "ft8"))]

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::params::NMAX;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::{pack77_type1, unpack77};

/// One clean transmission, DT 0.5 s, in silence plus faint noise.
fn frame(msg: &[u8; 77], freq: f32) -> Vec<i16> {
    let itone = mfsk_core::engine::tx::message_to_tones::<Ft8>(msg);
    let pcm = mfsk_core::engine::tx::synthesize::<Ft8>(&itone, 12_000, freq, 1.0);
    let pad = 6000usize;
    let mut audio = vec![0.0f32; NMAX];
    for (i, &s) in pcm.iter().enumerate() {
        if pad + i < NMAX {
            audio[pad + i] = s;
        }
    }
    let mut rng = 0x9e37_79b9_7f4a_7c15u64;
    for s in audio.iter_mut() {
        rng = rng
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        *s += ((rng >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.02;
    }
    audio
        .iter()
        .map(|&s| (s * 8000.0).clamp(-32768.0, 32767.0) as i16)
        .collect()
}

fn texts(audio: &[i16], contest: bool) -> Vec<String> {
    DecodeRequest::<Ft8>::new(audio, 200.0, 3000.0, 1.3, 200)
        .contest(contest)
        .decode()
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect()
}

#[test]
fn slash_r_is_dropped_outside_a_contest_and_kept_in_one() {
    let msg = pack77_type1("JA1ABC/R", "K1ABC", "PM95").expect("pack a /R message");
    let shown = unpack77(&msg).expect("unpack");
    assert!(
        shown.contains("/R"),
        "the test message must carry /R: {shown}"
    );
    let audio = frame(&msg, 1500.0);

    let default = texts(&audio, false);
    assert!(
        !default.iter().any(|t| t == &shown),
        "/R message decoded with no contest active: {default:?}"
    );
    let contest = texts(&audio, true);
    assert!(
        contest.iter().any(|t| t == &shown),
        "/R message not decoded with .contest(true): {contest:?}"
    );
}

#[test]
fn an_ordinary_message_is_unaffected() {
    let msg = pack77_type1("JA1ABC", "K1ABC", "PM95").expect("pack");
    let shown = unpack77(&msg).expect("unpack");
    let audio = frame(&msg, 1500.0);
    for contest in [false, true] {
        let got = texts(&audio, contest);
        assert!(
            got.iter().any(|t| t == &shown),
            "contest={contest}: {shown:?} missing from {got:?}"
        );
    }
}
