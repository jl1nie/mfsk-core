// SPDX-License-Identifier: GPL-3.0-only
//! Tier A: `mfsk_core::encoder` is one route over the per-mode transmit
//! code, not a new one. It must (1) produce what the receive side reads back,
//! for every mode; (2) give the tones and waveforms the existing per-mode
//! functions give; (3) refuse what a mode cannot send, with the right error;
//! (4) pass the hashed calls and the Q65 flag through.

use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;
use mfsk_core::encoder::{EncodeError, EncodeOptions, TxAudio, encode, encode_with};
use mfsk_core::msg::wsjt77::Pack77Error;

const FS: u32 = 12_000;
/// Peak amplitude of the frame, against noise of about +-300.
const AMP: f32 = 3_000.0;

/// The text each mode's round trip sends, and its carrier: inside the
/// decoder's default band (FST4 600-1400 Hz, FST4W 1400-1600 Hz).
fn message_and_carrier(mode: Mode) -> (&'static str, f32) {
    match mode.name() {
        "WSPR" => ("K1ABC FN42 37", 1_500.0),
        n if n.starts_with("FST4W") => ("K1ABC FN42 37", 1_500.0),
        "JT9" | "JT65" => ("CQ JA1ABC PM95", 1_500.0),
        n if n.starts_with("FST4") => ("CQ JA1ABC PM95", 1_000.0),
        _ => ("CQ JA1ABC PM95", 1_500.0),
    }
}

/// `frame` at the mode's nominal start in a period of silence, plus a little
/// deterministic noise, as i16.
fn period_of(mode: Mode, frame: &[f32], seed: u32) -> Vec<i16> {
    let meta = mode.meta();
    let slot = meta.slot_samples_12k as usize;
    let start = (meta.tx_start_offset_s * FS as f32).round() as usize;
    let mut x = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
    (0..slot)
        .map(|i| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let n = ((x >> 16) as i16 / 109) as f32;
            let s = i.checked_sub(start).and_then(|k| frame.get(k)).copied();
            (s.unwrap_or(0.0) * AMP + n).clamp(-32_768.0, 32_767.0) as i16
        })
        .collect()
}

#[test]
fn every_mode_encodes_to_what_the_decoder_reads() {
    let mut failures = Vec::new();
    for &mode in Mode::ALL {
        let t0 = std::time::Instant::now();
        let (text, f0) = message_and_carrier(mode);
        let tx = encode(mode, text).unwrap_or_else(|e| panic!("{}: {e}", mode.name()));
        assert_eq!(tx.mode, mode);
        let audio = tx.synthesize(&TxAudio::new(FS, f0));
        assert_eq!(audio.len(), tx.synth_len(FS), "{}", mode.name());
        let period = period_of(mode, &audio, 5);
        let mut dec = AnyDecoder::with_defaults(mode);
        let res = dec.decode_i16(&period, Some(0));
        let got: Vec<&str> = res.rows.iter().map(|r| r.decoded.text.as_str()).collect();
        let ok = got.contains(&tx.text.as_str());
        eprintln!(
            "{:10} sent {:?} decoded {:?} ({:.1} s)",
            mode.name(),
            tx.text,
            got,
            t0.elapsed().as_secs_f32()
        );
        if !ok {
            failures.push(format!(
                "{}: sent {:?}, decoded {:?}",
                mode.name(),
                tx.text,
                got
            ));
        }
    }
    assert!(!Mode::ALL.is_empty());
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
fn i16_and_length_follow_the_generic_route() {
    #[cfg(feature = "ft8")]
    {
        use mfsk_core::engine::tx::{synth_len, synthesize_i16};
        let tx = encode(Mode::Ft8, "CQ JA1ABC PM95").unwrap();
        let a = TxAudio::new(FS, 1_234.5);
        let v = tx.synthesize_i16(&a, 20_000);
        assert_eq!(
            v,
            synthesize_i16::<mfsk_core::Ft8>(&tx.tones, FS, 1_234.5, 20_000)
        );
        assert_eq!(v.len(), synth_len::<mfsk_core::Ft8>(FS));
        assert_eq!(v.len(), tx.synth_len(FS));
        let f = tx.synthesize(&a.amplitude(0.5));
        let g = tx.synthesize(&a);
        assert!(f.iter().zip(&g).all(|(x, y)| (x * 2.0 - y).abs() < 1e-6));
    }
}

/// The new route must not move any waveform the old routes made.
#[test]
fn tones_match_the_existing_routes() {
    use mfsk_core::engine::tx::{message_to_tones, synthesize};
    use mfsk_core::msg::wsjt77::pack77;

    let bits = pack77("CQ", "JA1ABC", "PM95").expect("pack77");

    #[cfg(feature = "ft8")]
    assert_eq!(
        encode(Mode::Ft8, "CQ JA1ABC PM95").unwrap().tones,
        message_to_tones::<mfsk_core::Ft8>(&bits)
    );
    #[cfg(feature = "ft4")]
    assert_eq!(
        encode(Mode::Ft4, "CQ JA1ABC PM95").unwrap().tones,
        message_to_tones::<mfsk_core::Ft4>(&bits)
    );
    #[cfg(feature = "fst4")]
    {
        assert_eq!(
            encode(Mode::Fst4S60, "CQ JA1ABC PM95").unwrap().tones,
            message_to_tones::<mfsk_core::fst4::Fst4s60>(&bits)
        );
        assert_eq!(
            encode(Mode::Fst4S1800, "CQ JA1ABC PM95").unwrap().tones,
            message_to_tones::<mfsk_core::fst4::Fst4s1800>(&bits)
        );
    }
    #[cfg(feature = "q65")]
    {
        use mfsk_core::q65::tx::synthesize_standard_for;
        let tx = encode(Mode::Q65A30, "CQ JA1ABC PM95").unwrap();
        assert_eq!(
            tx.tones,
            mfsk_core::q65::encode_channel_symbols(&bits).to_vec()
        );
        let old = synthesize_standard_for::<mfsk_core::q65::Q65a30>(
            "CQ", "JA1ABC", "PM95", FS, 1_500.0, 1.0,
        )
        .unwrap();
        assert_eq!(tx.synthesize(&TxAudio::new(FS, 1_500.0)), old);
        let _ = synthesize::<mfsk_core::q65::Q65a30>;
    }
    #[cfg(feature = "fst4w")]
    {
        use mfsk_core::fst4w::encode::message_to_tones as old;
        for text in [
            "K1ABC FN42 37",
            "K1ABC FN42  3",
            "PJ4/K1ABC 37",
            "<K1ABC> FN42AB 37",
        ] {
            let new = encode(Mode::Fst4W120, text);
            let old = old::<mfsk_core::fst4w::Fst4w120>(text);
            assert_eq!(new.ok().map(|e| e.tones), old, "{text:?}");
        }
    }
    #[cfg(feature = "wspr")]
    {
        let tx = encode(Mode::Wspr, "JA1ABC PM95 37").unwrap();
        let old =
            mfsk_core::wspr::synthesize_type1("JA1ABC", "PM95", 37, FS, 1_500.0, 1.0).unwrap();
        assert_eq!(tx.synthesize(&TxAudio::new(FS, 1_500.0)), old);
        assert_eq!(tx.tones.len(), 162);
        assert_eq!(tx.text, "JA1ABC PM95 37");
        // lower case and extra blanks are not part of the message
        assert_eq!(
            encode(Mode::Wspr, " ja1abc  pm95 37").unwrap().tones,
            tx.tones
        );
    }
    #[cfg(feature = "jt9")]
    {
        let tx = encode(Mode::Jt9, "CQ JA1ABC PM95").unwrap();
        let old =
            mfsk_core::jt9::synthesize_standard("CQ", "JA1ABC", "PM95", FS, 1_500.0, 1.0).unwrap();
        assert_eq!(tx.synthesize(&TxAudio::new(FS, 1_500.0)), old);
        assert_eq!(tx.text, "CQ JA1ABC PM95");
    }
    #[cfg(feature = "jt65")]
    {
        let tx = encode(Mode::Jt65, "K1ABC JA1ABC -15").unwrap();
        let old = mfsk_core::jt65::synthesize_standard("K1ABC", "JA1ABC", "-15", FS, 1_500.0, 1.0)
            .unwrap();
        assert_eq!(tx.synthesize(&TxAudio::new(FS, 1_500.0)), old);
        assert_eq!(tx.text, "K1ABC JA1ABC -15");
    }
}

/// FST4W through `pack77_message` + `prefer_wspr_50bit` against today's
/// `Fst4wMessage::pack_text` route: same tones when both accept, and both
/// refuse the same texts.
#[cfg(feature = "fst4w")]
#[test]
fn fst4w_agrees_with_pack_text() {
    use mfsk_core::fst4w::encode::message_to_tones;
    use mfsk_core::fst4w::{Fst4w300, Fst4wMessage};
    let texts = [
        "K1ABC FN42 37",
        "K1ABC FN42 0",
        "JA1ABC PM95 10",
        "G4ABC IO91 23",
        "PJ4/K1ABC 37",
        "K1ABC/7 30",
        "<K1ABC> FN42AB 37",
        "<PJ4/K1ABC> FN42AB 37",
        "  K1ABC FN42 37",
        "k1abc fn42 37",
        "K1ABC FN42 38",
        "CQ K1ABC FN42",
        "HELLO WORLD",
        "TNX 73 GL",
        "",
    ];
    let mut agree = 0;
    for t in texts {
        let new = encode(Mode::Fst4W300, t).ok().map(|e| e.tones);
        let old = message_to_tones::<Fst4w300>(t);
        assert_eq!(new, old, "FST4W {t:?}");
        assert_eq!(new.is_some(), Fst4wMessage::pack_text(t).is_some(), "{t:?}");
        agree += 1;
    }
    eprintln!("FST4W: {agree} of {} texts agree", texts.len());
}

#[test]
fn errors() {
    #[cfg(feature = "ft8")]
    {
        let e = encode(Mode::Ft8, "THIS TEXT IS WAY TOO LONG").unwrap_err();
        assert_eq!(
            e,
            EncodeError::Message {
                mode: Mode::Ft8,
                reason: Pack77Error::FreeTextTooLong
            }
        );
        let s = e.to_string();
        assert!(s.contains("FT8") && s.contains("too long"), "{s}");
        let src = std::error::Error::source(&e).expect("source");
        assert_eq!(src.to_string(), Pack77Error::FreeTextTooLong.to_string());
        let e = encode(Mode::Ft8, "HELLO{").unwrap_err();
        assert!(matches!(
            e,
            EncodeError::Message {
                reason: Pack77Error::FreeTextInvalid,
                ..
            }
        ));
    }
    #[cfg(feature = "wspr")]
    {
        use mfsk_core::msg::wspr::WsprPackError;
        // What `wqencode` does not take is `EncodeError::Wspr`, with its reason.
        for (t, want) in [
            ("K1ABC ZZ99 37", WsprPackError::BadGrid),
            ("K1ABC FN42 xx", WsprPackError::BadPower),
            ("", WsprPackError::NotWsprMessage),
            ("ONE", WsprPackError::BadCall),
            ("<K1ABC FN42LX 37", WsprPackError::UnclosedHashedCall),
        ] {
            let e = encode(Mode::Wspr, t).unwrap_err();
            assert_eq!(
                e,
                EncodeError::Wspr {
                    mode: Mode::Wspr,
                    reason: want
                },
                "{t:?}"
            );
            assert!(std::error::Error::source(&e).is_some());
            assert!(e.to_string().contains("WSPR"), "{e}");
        }
    }
    // JT9 / JT65 carry free text and compound calls (packmsg); only a character
    // the packer cannot take is refused.
    #[cfg(feature = "jt65")]
    {
        assert!(encode(Mode::Jt65, "HELLO WORLD").is_ok());
        assert!(encode(Mode::Jt65, "CQ PJ4/K1ABC FK52").is_ok());
        let e = encode(Mode::Jt65, "CQ K1ABC \u{c5}").unwrap_err();
        assert!(matches!(
            e,
            EncodeError::Jt72 {
                mode: Mode::Jt65,
                ..
            }
        ));
        assert!(std::error::Error::source(&e).is_some());
    }
    #[cfg(feature = "jt9")]
    assert!(encode(Mode::Jt9, "TNX 73 GL EVERYONE").is_ok());
    #[cfg(feature = "fst4w")]
    assert!(matches!(
        encode(Mode::Fst4W120, "HELLO WORLD"),
        Err(EncodeError::Message {
            reason: Pack77Error::PreferredFamilyRejected,
            ..
        })
    ));
}

#[cfg(feature = "ft8")]
#[test]
fn hashed_calls_pass_through() {
    let tx = encode(Mode::Ft8, "<W9XYZ> PJ4/K1ABC RR73").unwrap();
    assert!(
        tx.hashed_calls.iter().any(|c| c == "W9XYZ"),
        "{:?}",
        tx.hashed_calls
    );
    // Whatever the packer reports is passed through unchanged.
    let packed = mfsk_core::msg::wsjt77::pack77_message(
        "<W9XYZ> PJ4/K1ABC RR73",
        &mfsk_core::msg::wsjt77::Pack77Options::new(),
    )
    .unwrap();
    assert_eq!(tx.hashed_calls, packed.hashed_calls);
    assert_eq!(tx.text, "<W9XYZ> PJ4/K1ABC RR73");
    let plain = encode(Mode::Ft8, "CQ JA1ABC PM95").unwrap();
    assert_eq!(plain.hashed_calls, ["JA1ABC"]);
    #[cfg(feature = "wspr")]
    assert!(
        encode(Mode::Wspr, "K1ABC FN42 37")
            .unwrap()
            .hashed_calls
            .is_empty()
    );
}

#[cfg(feature = "q65")]
#[test]
fn q65_copied_last_tx_changes_the_tones_as_the_flagged_encoder_does() {
    use mfsk_core::msg::wsjt77::pack77;
    let bits = pack77("CQ", "JA1ABC", "PM95").unwrap();
    let off = encode(Mode::Q65A60, "CQ JA1ABC PM95").unwrap();
    let on = encode_with(
        Mode::Q65A60,
        "CQ JA1ABC PM95",
        &EncodeOptions::new().q65_copied_last_tx(true),
    )
    .unwrap();
    assert_ne!(off.tones, on.tones);
    assert_eq!(
        off.tones,
        mfsk_core::q65::encode_channel_symbols_flagged(&bits, false).to_vec()
    );
    assert_eq!(
        on.tones,
        mfsk_core::q65::encode_channel_symbols_flagged(&bits, true).to_vec()
    );
    assert_eq!(off.text, on.text);
    // the option is Q65's: another mode ignores it
    #[cfg(feature = "ft8")]
    assert_eq!(
        encode_with(
            Mode::Ft8,
            "CQ JA1ABC PM95",
            &EncodeOptions::new().q65_copied_last_tx(true)
        )
        .unwrap()
        .tones,
        encode(Mode::Ft8, "CQ JA1ABC PM95").unwrap().tones
    );
}

/// WSPR types 2 and 3 (`wqencode`): encode -> synthesize -> decode reads back what `Encoded::text` says.
#[cfg(feature = "wspr")]
#[test]
fn wspr_types_2_and_3_round_trip() {
    let mode = Mode::Wspr;
    for (text, shown, hashed) in [
        ("PJ4/K1ABC 37", "PJ4/K1ABC 37", None),
        ("K1ABC/7 20", "K1ABC/7 20", None),
        ("k1abc/p  7", "K1ABC/P  7", None),
        (
            "<PJ4/K1ABC> FN42LX 37",
            "<PJ4/K1ABC> FN42LX 37",
            Some("PJ4/K1ABC"),
        ),
        ("<k1abc> pm95tl 3", "<K1ABC> PM95TL  3", Some("K1ABC")),
    ] {
        let tx = encode(mode, text).unwrap_or_else(|e| panic!("{text:?}: {e}"));
        assert_eq!(tx.text, shown, "{text:?}");
        assert_eq!(
            tx.hashed_calls.first().map(String::as_str),
            hashed,
            "{text:?}"
        );
        assert_eq!(tx.tones.len(), 162);
        let audio = tx.synthesize(&TxAudio::new(FS, 1_500.0));
        let period = period_of(mode, &audio, 7);
        let mut dec = AnyDecoder::with_defaults(mode);
        let res = dec.decode_i16(&period, Some(0));
        let got: Vec<&str> = res.rows.iter().map(|r| r.decoded.text.as_str()).collect();
        if hashed.is_none() {
            assert!(got.contains(&tx.text.as_str()), "{text:?}: {got:?}");
        } else {
            // A receiver that has not heard the call reads the hash; the rest is the same.
            let tail = tx.text.split_once("> ").unwrap().1;
            assert!(
                got.iter()
                    .any(|g| g.starts_with("<#") && g.split_once("> ").map(|x| x.1) == Some(tail)),
                "{text:?}: {got:?}"
            );
        }
    }
}

/// JT9 / JT65 free text and compound calls: encode, synthesize, decode; the decoder
/// prints what `Encoded::text` says (upstream's blank collapsing and case folding included).
#[test]
fn jt9_and_jt65_free_text_and_compound_calls_round_trip() {
    let cases: &[(&str, &str)] = &[
        ("TNX 73 GL", "TNX 73 GL"),
        ("hello  world", "HELLO WORLD"),
        ("CQ PJ4/K1ABC FK52", "CQ PJ4/K1ABC FK52"),
        ("DE W9XYZ/QRP", "DE W9XYZ/QRP"),
        ("K1ABC/P W9XYZ", "K1ABC/P W9XYZ"),
        ("CQ DX K1ABC FN42", "CQ DX K1ABC FN42"),
    ];
    #[allow(unused_mut)]
    let mut modes: Vec<Mode> = Vec::new();
    #[cfg(feature = "jt9")]
    modes.push(Mode::Jt9);
    #[cfg(feature = "jt65")]
    modes.push(Mode::Jt65);
    for mode in modes {
        for &(text, want) in cases {
            let tx = encode(mode, text).unwrap();
            assert_eq!(tx.text, want, "{} {text:?}", mode.name());
            let audio = tx.synthesize(&TxAudio::new(FS, 1_500.0));
            let period = period_of(mode, &audio, 7);
            let mut dec = AnyDecoder::with_defaults(mode);
            let res = dec.decode_i16(&period, Some(0));
            let got: Vec<&str> = res.rows.iter().map(|r| r.decoded.text.as_str()).collect();
            assert!(
                got.contains(&tx.text.as_str()),
                "{} sent {:?}, decoded {got:?}",
                mode.name(),
                tx.text
            );
        }
    }
}

/// A power outside the set is rounded as `wqencode` does (38 -> 37), not refused.
#[cfg(feature = "wspr")]
#[test]
fn wspr_power_is_rounded_as_upstream() {
    let a = encode(Mode::Wspr, "K1ABC FN42 38").unwrap();
    let b = encode(Mode::Wspr, "K1ABC FN42 37").unwrap();
    assert_eq!(a.tones, b.tones);
    assert_eq!(a.text, "K1ABC FN42 37");
    assert_eq!(
        encode(Mode::Wspr, "K1ABC FN42 99").unwrap().text,
        "K1ABC FN42 60"
    );
}
