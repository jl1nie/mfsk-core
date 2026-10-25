//! #696: `RowDetail::method` says how a row was decoded, in words, where `pass` is a number
//! that means something else in every mode (FT4 and FST4 give 5 to both an OSD depth-3 decode and
//! an a-priori one). The map from each mode's ids is checked against real decodes.
#![cfg(all(feature = "ft8", feature = "ft4", feature = "fft-rustfft"))]

use mfsk_core::decoder::{DecodeMethod, DecodeParams, Decoder, Depth, SlotInput};
use mfsk_core::msg::ApHint;
use mfsk_core::{Ft4, Ft8};

#[allow(dead_code)]
mod common;

macro_rules! asset_path {
    ($f:expr) => {
        concat!(env!("CARGO_MANIFEST_DIR"), "/../embedded-poc/assets/", $f)
    };
}

/// FT8 ids do not collide inside FT8: 0..=4 BP, 5..=12 a-priori, 14..=23 OSD (the ladder's ids,
/// `ft8::decode_block::process_candidates`), and every row of a real busy period has a method.
#[test]
fn ft8_rows_name_their_method_by_the_ladder_they_came_from() {
    let Some(audio) = common::load_wav_i16_opt(asset_path!("qso3_busy.wav")) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    let mut d = Decoder::<Ft8>::with_defaults();
    let rows = d.decode(&SlotInput::i16(&audio)).rows;
    assert!(rows.len() >= 10, "the period must decode: {}", rows.len());
    let mut bp = 0;
    for row in &rows {
        let (pass, method) = (row.detail.pass, row.detail.method);
        let method = method.unwrap_or_else(|| panic!("pass {pass} has no method"));
        match pass {
            0..=4 => assert_eq!(method, DecodeMethod::Bp, "pass {pass}"),
            5..=12 => assert!(matches!(method, DecodeMethod::Ap { .. }), "pass {pass}"),
            14..=23 => assert_eq!(method, DecodeMethod::Osd, "pass {pass}"),
            30 => assert_eq!(method, DecodeMethod::ListA7),
            31 => assert_eq!(method, DecodeMethod::ListA8),
            _ => panic!("an FT8 pass id the map does not know: {pass}"),
        }
        bp += usize::from(method == DecodeMethod::Bp);
    }
    assert!(bp > 0, "a busy period decodes some rows by plain BP");
}

/// The collision: on FT4 (and FST4) pass 5 is the OSD depth-3 rung **and** the a-priori `iaptype`
/// 2 pass. The method is set where the row is made, so the two come out as what they are. The
/// CCIR sweep near threshold, decoded at `Depth::Deep` with a hint, has both.
#[test]
fn ft4_pass_5_is_osd_or_a_priori_by_how_the_row_was_made() {
    let mut seen: Vec<(u8, DecodeMethod)> = Vec::new();
    let mut files = 0;
    for snr in ["m13", "m14", "m16", "m18", "m10", "m05"] {
        for n in 1..=20 {
            let path = format!(
                "{}/../embedded-poc/assets/ft4_sweep/ft4_ccir_good_{snr}_{n:02}.wav",
                env!("CARGO_MANIFEST_DIR")
            );
            let Some(audio) = common::load_wav_i16_opt(&path) else {
                continue;
            };
            files += 1;
            let mut d =
                Decoder::<Ft4>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(Depth::Deep));
            // The corpus message is `CQ JL1NIE PM95`.
            d.set_ap_hint(Some(ApHint::new().with_call1("CQ").with_call2("JL1NIE")));
            for row in d.decode(&SlotInput::i16(&audio)).rows {
                let method = row.detail.method.expect("FT4 records a method");
                if !seen.contains(&(row.detail.pass, method)) {
                    seen.push((row.detail.pass, method));
                }
            }
        }
    }
    if files == 0 {
        common::skip_or_fail("ft4_sweep");
        return;
    }
    println!("{seen:?}");
    assert!(
        seen.contains(&(5, DecodeMethod::Osd)),
        "OSD depth 3: {seen:?}"
    );
    assert!(
        seen.contains(&(5, DecodeMethod::Ap { iaptype: Some(2) })),
        "a-priori, iaptype 2: {seen:?}"
    );
    assert!(seen.contains(&(0, DecodeMethod::Bp)), "{seen:?}");
    assert!(
        seen.contains(&(4, DecodeMethod::Osd)),
        "OSD depth 2: {seen:?}"
    );
    // Nothing is labelled by its number alone: no pass has two methods but 5's pair.
    for pass in (0..=12u8).filter(|&p| p != 5) {
        let methods: Vec<_> = seen.iter().filter(|(p, _)| *p == pass).collect();
        assert!(methods.len() <= 1, "pass {pass} has {methods:?}");
    }
}

/// Modes that record no method say so: their `pass` is 0, and 0 is not "BP".
#[cfg(all(feature = "jt9", feature = "jt65", feature = "wspr"))]
#[test]
fn modes_that_record_no_method_give_none() {
    use mfsk_core::{Jt9, Jt65, Wspr};
    let jt9 = mfsk_core::jt9::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1500.0, 0.3)
        .expect("pack + synth");
    let rows = Decoder::<Jt9>::new(DecodeParams::for_band((200.0, 4_000.0)).depth(Depth::Deep))
        .decode(&SlotInput::f32(&jt9))
        .rows;
    assert!(!rows.is_empty(), "JT9 must decode");
    assert!(rows.iter().all(|r| r.detail.method.is_none()));

    let jt65 = mfsk_core::jt65::tx::synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
        .expect("pack + synth");
    let rows = Decoder::<Jt65>::new(DecodeParams::for_band((200.0, 4_000.0)))
        .decode(&SlotInput::f32(&jt65))
        .rows;
    assert!(!rows.is_empty(), "JT65 must decode");
    assert!(rows.iter().all(|r| r.detail.method.is_none()));

    let wspr = mfsk_core::wspr::tx::synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
        .expect("valid message");
    let rows = Decoder::<Wspr>::new(DecodeParams::for_band((1_400.0, 1_600.0)))
        .decode(&SlotInput::f32(&wspr))
        .rows;
    assert!(!rows.is_empty(), "WSPR must decode");
    assert!(rows.iter().all(|r| r.detail.method.is_none()));
}

/// #696: `SlotResult::candidates` is what the coarse search kept for the call: a number on a call
/// that searched, `None` on one that did not (FT8's checkpoint B only subtracts), and the same
/// number whether the audio came whole or as the prefixes' last call. Not a recall figure: it
/// counts noise candidates, so it is at least the number of rows a call decodes from the search.
#[test]
fn candidates_is_what_the_coarse_search_kept_for_that_call() {
    let Some(audio) = common::load_wav_i16_opt(asset_path!("qso3_busy.wav")) else {
        common::skip_or_fail("qso3_busy.wav");
        return;
    };
    const A: usize = 141_696;
    const B: usize = 162_432;
    let whole = Decoder::<Ft8>::with_defaults().decode(&SlotInput::i16(&audio).period(7));
    let n = whole.candidates.expect("a whole-period decode searches");
    assert!(
        n as usize >= whole.rows.len() / 2 && n > 0,
        "{n} candidates, {} rows",
        whole.rows.len()
    );

    let mut d = Decoder::<Ft8>::with_defaults();
    let calls: Vec<_> = [A, B, audio.len()]
        .iter()
        .map(|&len| d.decode_prefix(&SlotInput::i16(&audio[..len]).period(9)))
        .collect();
    assert!(calls[0].candidates.is_some(), "checkpoint A searches");
    assert_eq!(calls[1].candidates, None, "checkpoint B only subtracts");
    assert!(calls[2].candidates.is_some(), "the end searches");
    // A repeated call for the period returns its complete set again, count included.
    assert_eq!(
        d.decode_prefix(&SlotInput::i16(&audio).period(9))
            .candidates,
        calls[2].candidates
    );

    // FT4, from the CCIR sweep: a number on any file.
    let path = format!(
        "{}/../embedded-poc/assets/ft4_sweep/ft4_ccir_good_m13_01.wav",
        env!("CARGO_MANIFEST_DIR")
    );
    let Some(ft4) = common::load_wav_i16_opt(&path) else {
        common::skip_or_fail("ft4_sweep");
        return;
    };
    let r =
        Decoder::<Ft4>::new(DecodeParams::for_band((200.0, 4_000.0))).decode(&SlotInput::i16(&ft4));
    assert!(r.candidates.is_some_and(|n| n > 0), "{:?}", r.candidates);
}
