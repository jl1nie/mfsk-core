//! WSJT-X's a7 and a8 list decoders (`ft8_a7.f90`, `ft8_a8d.f90`, #464):
//! measurements against real `jt9` on `ft8sim` corpora, and gates.
#![cfg(feature = "fft-rustfft")]

use std::path::{Path, PathBuf};

use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::WsjtxDepth;
use mfsk_core::ft8::list_decode::{PASS_ID_A7, PASS_ID_A8};
use mfsk_core::msg::ap::ApHint;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

fn d3<'a>(audio: &'a [i16], ap: Option<&'a ApHint>) -> DecodeRequest<'a, Ft8> {
    DecodeRequest::<Ft8>::wsjtx_depth(audio, 100.0, 3000.0, 1.3, 600, WsjtxDepth::D3, ap)
}

/// `(found, found by the list decoder)` for `want` in one decode.
fn found(
    results: &[mfsk_core::engine::pipeline::DecodeResult],
    want: &str,
    list_pass: u8,
) -> (bool, bool) {
    let hit = results
        .iter()
        .find(|r| unpack77(r.message77()).as_deref() == Some(want));
    (hit.is_some(), hit.is_some_and(|r| r.pass == list_pass))
}

fn wavs(dir: &Path) -> Vec<PathBuf> {
    let mut v: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "wav"))
        .collect();
    v.sort();
    v
}

// ── Gates (tier A+B): one synthetic slot each, deterministic noise ──────────

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn gauss(&mut self) -> f64 {
        let u1 = ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64;
        let u2 = ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64;
        (-2.0 * u1.ln()).sqrt() * (core::f64::consts::TAU * u2).cos()
    }
}

/// A 15 s slot holding `text` at 1500 Hz, DT 0, at `snr_db` in 2500 Hz, over white
/// Gaussian noise from `seed`; `text` `None` for noise alone.
fn slot(text: Option<&str>, snr_db: f64, seed: u64) -> Vec<i16> {
    const AMP: f64 = 100.0;
    let mut audio = vec![0f64; 180_000];
    if let Some(t) = text {
        let w: Vec<&str> = t.split(' ').collect();
        let m77 = mfsk_core::msg::wsjt77::pack77(w[0], w[1], w.get(2).copied().unwrap_or(""))
            .expect("packs");
        let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&m77);
        let pcm = mfsk_core::engine::tx::synthesize_i16::<Ft8>(&tones, 12_000, 1500.0, AMP as i16);
        for (a, &p) in audio[6_000..].iter_mut().zip(pcm.iter()) {
            *a = p as f64;
        }
    }
    // Signal power AMP^2/2 against noise in 2500 of the 6000 Hz: sigma^2 * 2500/6000.
    let sigma = (AMP * AMP / 2.0 / (10f64.powf(snr_db / 10.0) * 2500.0 / 6000.0)).sqrt();
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15 ^ seed);
    audio
        .iter()
        .map(|&a| (a + sigma * rng.gauss()).round().clamp(-32768.0, 32767.0) as i16)
        .collect()
}

fn texts(r: &[mfsk_core::engine::pipeline::DecodeResult]) -> Vec<(u8, String)> {
    r.iter()
        .map(|r| (r.pass, unpack77(r.message77()).unwrap_or_default()))
        .collect()
}

/// a7: a message the ladder cannot decode is found once the previous cycle's
/// decode of the same pair is passed (`ft8_a7d`).
#[test]
fn a7_finds_the_pairs_next_message() {
    let prev_audio = slot(Some("K1ABC W9XYZ -10"), -10.0, 1);
    let prev = DecodeRequest::<Ft8>::new(&prev_audio, 100.0, 3000.0, 1.3, 50)
        .decode()
        .results;
    assert!(
        texts(&prev).iter().any(|(_, t)| t == "K1ABC W9XYZ -10"),
        "{:?}",
        texts(&prev)
    );

    let cur = slot(Some("K1ABC W9XYZ RR73"), -23.0, 2);
    let plain = DecodeRequest::<Ft8>::new(&cur, 100.0, 3000.0, 1.3, 50)
        .decode()
        .results;
    assert!(
        !texts(&plain).iter().any(|(_, t)| t == "K1ABC W9XYZ RR73"),
        "the ladder alone decodes it, so this does not test a7: {:?}",
        texts(&plain)
    );
    let with = DecodeRequest::<Ft8>::new(&cur, 100.0, 3000.0, 1.3, 50)
        .previous_cycle(&prev)
        .decode()
        .results;
    assert!(
        texts(&with).contains(&(PASS_ID_A7, "K1ABC W9XYZ RR73".into())),
        "a7 did not find it: {:?}",
        texts(&with)
    );
}

/// a8: MyCall, HisCall, HisGrid and the QSO frequency find a message the ladder
/// cannot decode; without the QSO frequency a8 does not run (`ft8_a8d`).
#[test]
fn a8_finds_the_qso_message_at_the_qso_frequency() {
    let hint = ApHint::new()
        .with_call1("K1ABC")
        .with_call2("W9XYZ")
        .with_grid("EN37");
    let cur = slot(Some("K1ABC W9XYZ R-12"), -24.0, 3);
    let with = DecodeRequest::<Ft8>::new(&cur, 100.0, 3000.0, 1.3, 50)
        .ap_hint(&hint)
        .freq_hint(1500.0)
        .decode()
        .results;
    assert!(
        texts(&with).contains(&(PASS_ID_A8, "K1ABC W9XYZ R-12".into())),
        "a8 did not find it: {:?}",
        texts(&with)
    );
    let without = DecodeRequest::<Ft8>::new(&cur, 100.0, 3000.0, 1.3, 50)
        .ap_hint(&hint)
        .decode()
        .results;
    assert!(
        !texts(&without).iter().any(|(p, _)| *p == PASS_ID_A8),
        "a8 ran without a QSO frequency: {:?}",
        texts(&without)
    );
}

/// Noise alone, with a previous cycle and a full a8 hint: nothing comes back.
#[test]
fn list_decoders_find_nothing_in_noise() {
    let prev_audio = slot(Some("K1ABC W9XYZ -10"), -10.0, 1);
    let prev = DecodeRequest::<Ft8>::new(&prev_audio, 100.0, 3000.0, 1.3, 50)
        .decode()
        .results;
    let hint = ApHint::new()
        .with_call1("K1ABC")
        .with_call2("W9XYZ")
        .with_grid("EN37");
    for seed in 10..14 {
        let noise = slot(None, 0.0, seed);
        let r = DecodeRequest::<Ft8>::new(&noise, 100.0, 3000.0, 1.3, 50)
            .previous_cycle(&prev)
            .ap_hint(&hint)
            .freq_hint(1500.0)
            .decode()
            .results;
        assert!(r.is_empty(), "seed {seed}: {:?}", texts(&r));
    }
}

/// a7 on a corpus laid out as `<dir>/prev.wav` (`K1ABC W9XYZ -10`, strong) and
/// `<dir>/m<snr>/cur_<nn>.wav` (`K1ABC W9XYZ RR73` at that SNR, same frequency and
/// DT). Prints, per SNR, how many trials decode the RR73 without the previous
/// cycle and with it, and how many of the latter came from a7.
/// `MFSK_A7_CORPUS=<dir> cargo test --release --test ft8_list_decode a7_measure -- --ignored --nocapture`
#[test]
#[ignore = "measurement, needs MFSK_A7_CORPUS"]
fn a7_measure() {
    let Ok(dir) = std::env::var("MFSK_A7_CORPUS") else {
        return;
    };
    let dir = PathBuf::from(dir);
    let prev_audio = load_wav_i16(dir.join("prev.wav"));
    let prev = d3(&prev_audio, None).decode().results;
    assert!(
        prev.iter()
            .any(|r| unpack77(r.message77()).as_deref() == Some("K1ABC W9XYZ -10")),
        "the previous cycle's signal must decode"
    );
    let mut sub: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_dir())
        .collect();
    sub.sort();
    for s in sub {
        let (mut plain, mut with, mut by_a7, mut extra) = (0, 0, 0, 0);
        let files = wavs(&s);
        for f in &files {
            let audio = load_wav_i16(f);
            plain += found(
                &d3(&audio, None).decode().results,
                "K1ABC W9XYZ RR73",
                PASS_ID_A7,
            )
            .0 as u32;
            let r = d3(&audio, None).previous_cycle(&prev).decode().results;
            let (a, b) = found(&r, "K1ABC W9XYZ RR73", PASS_ID_A7);
            with += a as u32;
            by_a7 += b as u32;
            extra += r
                .iter()
                .filter(|r| r.pass == PASS_ID_A7)
                .filter(|r| unpack77(r.message77()).as_deref() != Some("K1ABC W9XYZ RR73"))
                .count() as u32;
            println!(
                "A7ROW {} {} {} {}",
                s.file_name().unwrap().to_string_lossy(),
                f.file_name().unwrap().to_string_lossy(),
                a as u8,
                b as u8
            );
        }
        println!(
            "A7 {}: {} files, plain {plain}, with previous cycle {with} (a7 {by_a7}), wrong a7 decodes {extra}",
            s.file_name().unwrap().to_string_lossy(),
            files.len()
        );
    }
}

/// a8 on `<dir>/m<snr>/*.wav` holding `K1ABC W9XYZ R-12` at 1500 Hz, with MyCall
/// K1ABC, HisCall W9XYZ, HisGrid EN37 and the QSO frequency 1500 Hz given.
/// `MFSK_A8_CORPUS=<dir> ... a8_measure -- --ignored --nocapture`
#[test]
#[ignore = "measurement, needs MFSK_A8_CORPUS"]
fn a8_measure() {
    let Ok(dir) = std::env::var("MFSK_A8_CORPUS") else {
        return;
    };
    let hint = ApHint::new()
        .with_call1("K1ABC")
        .with_call2("W9XYZ")
        .with_grid("EN37");
    let want = std::env::var("MFSK_A8_WANT").unwrap_or_else(|_| "K1ABC W9XYZ R-12".into());
    let mut sub: Vec<PathBuf> = std::fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.is_dir())
        .collect();
    sub.sort();
    for s in sub {
        let (mut ap, mut by_a8, mut extra) = (0, 0, 0);
        let files = wavs(&s);
        for f in &files {
            let audio = load_wav_i16(f);
            let r = d3(&audio, Some(&hint)).freq_hint(1500.0).decode().results;
            let (a, b) = found(&r, &want, PASS_ID_A8);
            ap += a as u32;
            by_a8 += b as u32;
            extra += r
                .iter()
                .filter(|r| unpack77(r.message77()).as_deref() != Some(want.as_str()))
                .count() as u32;
            println!(
                "A8ROW {} {} {} {}",
                s.file_name().unwrap().to_string_lossy(),
                f.file_name().unwrap().to_string_lossy(),
                a as u8,
                b as u8
            );
        }
        println!(
            "A8 {}: {} files, decoded {ap} (a8 {by_a8}), other decodes {extra}",
            s.file_name().unwrap().to_string_lossy(),
            files.len()
        );
    }
}
