//! #697: `SlotResult::candidates` for the slow modes — what the first coarse search of the call kept,
//! after its own threshold and cap. One test per mode, each built to fail if its definition is wrong:
//! the cap is read at its edge (a count taken after refinement would be smaller, one taken before the
//! truncate larger), and a later pass cannot change it. Tracked recordings and synthesised signals
//! only: nothing here reads a `.gitignore`d corpus.
#![cfg(all(feature = "jt9", feature = "jt65", feature = "fft-rustfft"))]

#[allow(dead_code)]
mod common;

use mfsk_core::decoder::{DecodeParams, Decoder, Depth, SlotInput};
use mfsk_core::{Jt9, Jt65};

macro_rules! asset_path {
    ($f:expr) => {
        concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/", $f)
    };
}

/// Deterministic standard-normal noise, as `decode_method.rs`'s.
fn gauss(seed: u64, n: usize) -> Vec<f32> {
    let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    let mut u = move || {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        ((s >> 11) as f32 + 0.5) / (1u64 << 53) as f32
    };
    (0..n)
        .map(|_| (-2.0 * u().ln()).sqrt() * (std::f32::consts::TAU * u()).cos())
        .collect()
}

/// JT9 keeps `truncate(max(max_candidates, 32))` of its coarse list (`jt9/mod.rs`): 32 by default, and
/// lowering `max_candidates` below 32 changes nothing; above it, the count is the cap until the list is
/// shorter, and then it is the list. `130418_1742.wav`'s list is 2190 long, so 5000 and 3000 both give
/// 2190: a count taken after dedup or decoding would not.
#[test]
fn jt9_counts_the_truncated_coarse_list() {
    let Some(a) = common::load_wav_f32_opt(asset_path!("130418_1742.wav")) else {
        common::skip_or_fail("130418_1742.wav");
        return;
    };
    let count = |cap: Option<usize>| {
        let mut d =
            Decoder::<Jt9>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(Depth::Deep));
        d.extras_mut().search.max_candidates = cap;
        d.decode(&SlotInput::f32(&a)).candidates
    };
    assert_eq!(count(None), Some(32), "the floor of 32");
    assert_eq!(count(Some(5)), Some(32), "below the floor, the floor");
    assert_eq!(count(Some(40)), Some(40));
    assert_eq!(count(Some(100)), Some(100));
    let all = count(Some(5_000)).expect("a search ran");
    assert!(all > 100, "the whole coarse list is {all}");
    assert_eq!(count(Some(3_000)), Some(all), "above the list, the list");
}

/// JT65 counts pass 1's list, capped at `min(max_candidates, 50)` (`jt65/mod.rs`, `jt65_decode.f90:323`),
/// and a later pass cannot change it: the same at `Normal` (2 passes) and `Deep` (4).
#[test]
fn jt65_counts_pass_ones_capped_list() {
    let Some(a) = common::load_wav_f32_opt(asset_path!("golden/jt65/jt65a_5sig_m18.wav")) else {
        common::skip_or_fail("jt65a_5sig_m18.wav");
        return;
    };
    let count = |depth: Depth, cap: Option<usize>| {
        let mut d = Decoder::<Jt65>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(depth));
        d.extras_mut().search.max_candidates = cap;
        d.decode(&SlotInput::f32(&a)).candidates
    };
    for depth in [Depth::Normal, Depth::Deep] {
        assert_eq!(count(depth, None), Some(8), "{depth:?}: the default cap");
        assert_eq!(count(depth, Some(3)), Some(3), "{depth:?}");
        assert_eq!(count(depth, Some(20)), Some(20), "{depth:?}");
    }
}

/// A signal makes candidates, and so does noise: the count is what was searched, not what was found
/// (it is not a recall figure). Synthesised, with deterministic noise.
#[test]
fn a_synthesised_signal_and_plain_noise_both_report_a_search() {
    let jt9 = mfsk_core::jt9::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1_500.0, 0.3)
        .expect("pack + synth");
    let r = Decoder::<Jt9>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(Depth::Deep))
        .decode(&SlotInput::f32(&jt9));
    assert!(!r.rows.is_empty(), "JT9 must decode");
    assert!(r.candidates.is_some_and(|n| n >= 1), "{:?}", r.candidates);

    let jt65 =
        mfsk_core::jt65::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1_270.0, 0.3)
            .expect("pack + synth");
    let r = Decoder::<Jt65>::new(DecodeParams::for_band((200.0, 4_000.0)))
        .decode(&SlotInput::f32(&jt65));
    assert!(!r.rows.is_empty(), "JT65 must decode");
    assert!(r.candidates.is_some_and(|n| n >= 1), "{:?}", r.candidates);

    // 60 s of noise only: nothing decodes, and a search still reports its list.
    let noise: Vec<f32> = gauss(7, 720_000).iter().map(|v| v * 0.05).collect();
    let r = Decoder::<Jt9>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(Depth::Deep))
        .decode(&SlotInput::f32(&noise));
    assert!(r.rows.is_empty(), "JT9: noise decoded {}", r.rows.len());
    assert!(
        r.candidates.is_some_and(|n| n >= 1),
        "JT9: {:?}",
        r.candidates
    );
    let r = Decoder::<Jt65>::new(DecodeParams::for_band((200.0, 4_000.0)))
        .decode(&SlotInput::f32(&noise));
    assert!(r.rows.is_empty(), "JT65: noise decoded {}", r.rows.len());
    assert!(
        r.candidates.is_some_and(|n| n >= 1),
        "JT65: {:?}",
        r.candidates
    );
}
