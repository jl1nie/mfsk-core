//! WSJT-X 3.2's Q65 Pileup "copied last Tx" flag: the spare 78th
//! payload bit (`genq65.f90`'s `iflag`, `q65_decode.f90`'s `iflagdec`,
//! `q65_ap.f90` under `lq65pileup`; issue #470).

#![cfg(feature = "q65")]

#[allow(dead_code)]
mod common;

use common::channel::AwgnChannel;
use mfsk_core::msg::ApHint;
use mfsk_core::msg::q65::pack77_q65;
use mfsk_core::q65::{
    DecodeRequest, Q65a30, encode_channel_symbols, encode_channel_symbols_flagged,
    synthesize_standard_flagged_for,
};

const FS: u32 = 12_000;
const FREQ: f32 = 1500.0;

fn slot(msg: (&str, &str, &str), flag: bool, sigma: f32, seed: u64) -> Vec<f32> {
    let mut a = synthesize_standard_flagged_for::<Q65a30>(msg.0, msg.1, msg.2, flag, FS, FREQ, 1.0)
        .unwrap();
    if sigma > 0.0 {
        AwgnChannel::new(sigma, seed).apply(&mut a);
    }
    a
}

fn sniper(audio: &[f32], hint: Option<&ApHint>, pileup: bool) -> Option<(String, bool)> {
    let mut req = DecodeRequest::<Q65a30>::sniper(audio, FS, 0, FREQ).pileup(pileup);
    if let Some(h) = hint {
        req = req.ap_hint(h);
    }
    req.decode().map(|r| (r.message, r.copied_last_tx))
}

#[test]
fn the_flag_changes_only_the_codeword_and_is_reported() {
    let bits = pack77_q65("K1ABC", "JA1ABC", "-15").unwrap();
    assert_ne!(
        encode_channel_symbols(&bits),
        encode_channel_symbols_flagged(&bits, true)
    );
    assert_eq!(
        encode_channel_symbols(&bits),
        encode_channel_symbols_flagged(&bits, false)
    );
    let msg = ("K1ABC", "JA1ABC", "-15");
    assert_eq!(
        sniper(&slot(msg, true, 0.0, 0), None, false),
        Some(("K1ABC JA1ABC -15".into(), true))
    );
    assert_eq!(
        sniper(&slot(msg, false, 0.0, 0), None, false),
        Some(("K1ABC JA1ABC -15".into(), false))
    );
}

#[test]
fn pileup_ap_frees_bit_78_for_a_mycall_dxcall_hint() {
    let msg = ("K1ABC", "JA1ABC", "-15");
    let hint = ApHint::new().with_call1("K1ABC").with_call2("JA1ABC");
    let audio = slot(msg, true, 0.0, 0);
    // Outside Pileup the hint locks the flag to 0, as `q65_ap.f90` does,
    // so a flagged reply cannot satisfy it.
    assert_eq!(sniper(&audio, Some(&hint), false), None);
    assert_eq!(
        sniper(&audio, Some(&hint), true),
        Some(("K1ABC JA1ABC -15".into(), true))
    );
    // An unflagged reply decodes either way.
    let plain = slot(msg, false, 0.0, 0);
    assert!(sniper(&plain, Some(&hint), false).is_some());
    assert!(sniper(&plain, Some(&hint), true).is_some());
}

/// Flagged `K1ABC JA1ABC -15` in noise, 20 seeds per level: the plain
/// decode, the MyCall + DxCall hint with bit 78 locked, and the same hint
/// under Pileup. At sigma 20 (2026-09-26): 6 / 0 / 20.
#[test]
#[ignore]
fn pileup_measure() {
    let msg = ("K1ABC", "JA1ABC", "-15");
    let hint = ApHint::new().with_call1("K1ABC").with_call2("JA1ABC");
    for sigma in [20.0f32, 30.0, 40.0, 50.0, 60.0, 70.0] {
        let (mut p, mut a0, mut a1) = (0, 0, 0);
        for seed in 0..20 {
            let audio = slot(msg, true, sigma, seed);
            p += sniper(&audio, None, false).is_some() as u32;
            a0 += sniper(&audio, Some(&hint), false).is_some() as u32;
            a1 += sniper(&audio, Some(&hint), true).is_some() as u32;
        }
        println!("sigma {sigma}: plain {p} ap {a0} ap+pileup {a1}");
    }
}
