//! WSJT-X's q3 decode — full-AP list decoding at the Rx frequency with
//! the 85-symbol sync (`q65_ccf_85` + `q65_dec_q3`).
//!
//! The gates run on this crate's own synthesis: a clean list message is
//! found at the Rx frequency, and noise yields nothing. `q3_measure`
//! (ignored) decodes `q65sim` WAVs listed in `MFSK_Q65_Q3_WAVS`, for
//! comparison with `jt9 -3 -c K1ABC -x JA1ABC -g PM95 -f 1500 -F 10`.

#![cfg(feature = "q65")]

#[allow(dead_code)]
mod common;

use common::channel::AwgnChannel;
use mfsk_core::q65::search::default_search_params;
use mfsk_core::q65::{DecodeRequest, Q65a30, standard_qso_codewords, synthesize_standard};

const FS: u32 = 12_000;
const RX: f32 = 1500.0;
const NOMINAL: usize = FS as usize / 2;

fn slot(msg: Option<(&str, &str, &str)>, sigma: f32, seed: u64) -> Vec<f32> {
    let mut a = vec![0f32; 30 * FS as usize];
    if let Some((c1, c2, r)) = msg {
        let sig = synthesize_standard(c1, c2, r, FS, RX, 1.0).unwrap();
        a[NOMINAL..NOMINAL + sig.len()].copy_from_slice(&sig);
    }
    AwgnChannel::new(sigma, seed).apply(&mut a);
    a
}

/// Messages from the q3 decode alone: the list at the Rx frequency, with a
/// search window that holds nothing else.
fn q3(audio: &[f32], list: &[[i32; 63]]) -> Vec<String> {
    let mut params = default_search_params();
    params.freq_min_hz = 3900.0;
    params.freq_max_hz = 3950.0;
    let max_drift: u32 = std::env::var("MFSK_Q65_Q3_MAX_DRIFT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    DecodeRequest::<Q65a30>::new(audio, FS, NOMINAL, params)
        .ap_list(list)
        .rx_freq(RX)
        .max_drift(max_drift)
        .decode()
        .into_iter()
        .map(|r| r.message)
        .collect()
}

#[test]
fn q3_finds_a_list_message_at_the_rx_frequency() {
    let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    let a = slot(Some(("K1ABC", "JA1ABC", "-15")), 1.0, 1);
    assert_eq!(q3(&a, &list), ["K1ABC JA1ABC -15"]);
}

#[test]
fn q3_finds_nothing_in_noise() {
    let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    for seed in 0..5 {
        assert!(q3(&slot(None, 1.0, seed), &list).is_empty(), "seed {seed}");
    }
}

/// Decode each WAV in `MFSK_Q65_Q3_WAVS` (Q65-30A, list for K1ABC /
/// JA1ABC / PM95, Rx 1500 Hz, F Tol 10): prints the q3-only result, the
/// plain scan's, and the Rx-frequency-less `.ap_list()` scan's.
///
/// 2026-09-26, `q65sim` "K1ABC JA1ABC -15", 20 WAVs per level, -24 / -26 /
/// -28 / -30 dB: q3 here 20 / 20 / 7 / 2, `jt9 -3 -d 1 -c K1ABC -x JA1ABC
/// -g PM95 -f 1500 -F 10` q3 20 / 20 / 7 / 2, the same files. With
/// `MFSK_Q65_Q3_MAX_DRIFT=50` on 10 WAVs at -24 dB drifting 60 / 120 Hz
/// per minute: 0 / 0 without stage 5, 7 / 1 with it.
#[test]
#[ignore]
fn q3_measure() {
    let Ok(list_env) = std::env::var("MFSK_Q65_Q3_WAVS") else {
        return;
    };
    let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    for path in list_env.split_whitespace() {
        let a = common::load_wav_f32(path);
        let plain: Vec<String> =
            DecodeRequest::<Q65a30>::new(&a, FS, NOMINAL, default_search_params())
                .decode()
                .into_iter()
                .map(|r| r.message)
                .collect();
        let legacy: Vec<String> =
            DecodeRequest::<Q65a30>::new(&a, FS, NOMINAL, default_search_params())
                .ap_list(&list)
                .decode()
                .into_iter()
                .map(|r| r.message)
                .collect();
        println!(
            "{path} q3={:?} plain={plain:?} legacy={legacy:?}",
            q3(&a, &list)
        );
    }
}

/// Noise-only slots, q3 at F Tol 100 Hz: the false-decode count
/// (`MFSK_Q65_Q3_NOISE`, default 100 slots; `MFSK_Q65_Q3_MAX_DRIFT` for
/// stage 5). 0 in 100 either way (2026-09-26).
#[test]
#[ignore]
fn q3_noise_measure() {
    let n: u64 = std::env::var("MFSK_Q65_Q3_NOISE")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(100);
    let list = standard_qso_codewords("K1ABC", "JA1ABC", "PM95");
    let mut params = default_search_params();
    params.freq_min_hz = 3900.0;
    params.freq_max_hz = 3950.0;
    let max_drift: u32 = std::env::var("MFSK_Q65_Q3_MAX_DRIFT")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);
    let mut false_decodes = 0;
    for seed in 0..n {
        let a = slot(None, 1.0, 1000 + seed);
        false_decodes += DecodeRequest::<Q65a30>::new(&a, FS, NOMINAL, params)
            .ap_list(&list)
            .rx_freq(RX)
            .ftol(100.0)
            .max_drift(max_drift)
            .decode()
            .len();
    }
    println!("q3 noise: {false_decodes} decodes in {n} slots");
}
