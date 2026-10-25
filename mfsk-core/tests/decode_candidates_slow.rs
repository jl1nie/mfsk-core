//! #697: `SlotResult::candidates` for the slow modes — what the first coarse search of the call kept,
//! after its own threshold and cap. One test per mode, each built to fail if its definition is wrong:
//! the cap is read at its edge (a count taken after refinement would be smaller, one taken before the
//! truncate larger), and a later pass cannot change it. Tracked recordings and synthesised signals
//! only: nothing here reads a `.gitignore`d corpus.
#![cfg(all(feature = "jt9", feature = "jt65", feature = "fft-rustfft"))]

#[allow(dead_code)]
mod common;

use mfsk_core::decoder::{DecodeParams, Decoder, Depth, Q65Extras, SlotInput};
use mfsk_core::q65::Q65a15;
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

/// A 15 s Q65-15A slot with a station at each of `freqs` (1 s in), over deterministic noise.
fn q65_slot(freqs: &[f32], noise_sigma: f32) -> Vec<f32> {
    const CALLS: [&str; 6] = ["K1ABC", "JA1ABC", "W9XYZ", "DL1XYZ", "VK3NV", "G4ABC"];
    let mut audio = vec![0f32; 12_000 * 15];
    for (i, &f) in freqs.iter().enumerate() {
        let s = mfsk_core::q65::tx::synthesize_standard_for::<Q65a15>(
            "CQ",
            CALLS[i % CALLS.len()],
            "FN42",
            12_000,
            f,
            0.2,
        )
        .expect("pack + synth");
        let n = s.len().min(audio.len() - 12_000);
        for (a, v) in audio[12_000..12_000 + n].iter_mut().zip(&s) {
            *a += v;
        }
    }
    for (a, n) in audio.iter_mut().zip(gauss(3, 12_000 * 15)) {
        *a += noise_sigma * n;
    }
    audio
}

fn q65_decode(
    audio: &[f32],
    cap: Option<usize>,
) -> mfsk_core::decoder::SlotResult<mfsk_core::q65::Q65Result> {
    let mut e = Q65Extras::default();
    e.search.max_candidates = cap;
    Decoder::<Q65a15>::new(DecodeParams::for_band((200.0, 3_000.0)))
        .with_extras(e)
        .decode(&SlotInput::f32(audio))
}

/// Q65 counts the first `scan_with` list: after its sync threshold, capped at `max_candidates` (8 by
/// default), before any decode. Five stations in a slot give a list of at least five, of which fewer
/// decode: the count is the list, not the rows. Read at the cap's edge: `cap` below the list gives the cap,
/// above it the list, and the default (8) gives `min(8, list)`. (The exact list length depends on the
/// FFT's rounding, so the test asserts those relations and not a bare number.)
#[test]
fn q65_counts_the_capped_first_list_not_the_rows() {
    let audio = q65_slot(&[600.0, 900.0, 1_200.0, 1_500.0, 1_800.0], 0.05);
    let all = q65_decode(&audio, Some(500));
    let list = all.candidates.expect("a search ran") as usize;
    assert!(list >= 5, "five stations: a list of {list}");
    assert!(
        all.rows.len() < list,
        "{} rows from {list} candidates: the count is not the rows",
        all.rows.len()
    );
    assert_eq!(
        q65_decode(&audio, Some(3)).candidates,
        Some(3),
        "below the list, the cap"
    );
    assert_eq!(
        q65_decode(&audio, Some(list + 10)).candidates,
        Some(list as u32),
        "above the list, the list"
    );
    assert_eq!(
        q65_decode(&audio, None).candidates,
        Some(list.min(8) as u32),
        "the default cap is 8"
    );
}

/// A search that finds a bin or two reports them; silence reports an empty list, `Some(0)`, not `None`:
/// the search ran. With averaging on, each period's call searches the running-average spectrogram and
/// reports its own count.
#[test]
fn q65_silence_is_an_empty_list_and_averaging_searches_each_period() {
    let silence = vec![0f32; 12_000 * 15];
    assert_eq!(q65_decode(&silence, None).candidates, Some(0));

    let audio = q65_slot(&[1_500.0], 0.05);
    let mut e = Q65Extras::default();
    e.search.max_candidates = Some(8);
    let mut params = DecodeParams::for_band((200.0, 3_000.0));
    params.averaging = true;
    let mut d = Decoder::<Q65a15>::new(params).with_extras(e);
    for period in 100..103 {
        let r = d.decode(&SlotInput::f32(&audio).period(period));
        assert!(
            r.candidates.is_some_and(|n| n >= 1),
            "period {period}: {:?}",
            r.candidates
        );
    }
}
