//! FST4's impulse-noise blanker (`blanker.f90`, WSJT-X's **NB** setting;
//! issue #469).
//!
//! One FST4-15 signal in white noise, with a train of full-scale clicks
//! on top — the ignition / power-line noise NB exists for. The clicks'
//! energy is spread across the whole band by the slot FFT, burying the
//! signal; zeroing them first recovers it, as upstream does.

#[allow(dead_code)]
mod common;

use common::channel::AwgnChannel;
use mfsk_core::fst4::Fst4s15;
use mfsk_core::msg::decode_request::{DecodeRequest, NoiseBlanker};
use mfsk_core::msg::wsjt77::{pack77, unpack77};

const FREQ: f32 = 1500.0;

/// The slot: the message at `FREQ`, white noise of `sigma`, and a click of
/// `CLICK_LEN` full-scale samples every `CLICK_EVERY` samples.
fn slot(sigma: f32, clicks: bool) -> Vec<i16> {
    slot_seeded(sigma, clicks, 0x469)
}

fn slot_seeded(sigma: f32, clicks: bool, seed: u64) -> Vec<i16> {
    const CLICK_EVERY: usize = 600; // 20 per second
    const CLICK_LEN: usize = 3; // 0.5 % of the samples
    let msg = pack77("CQ", "JA1ABC", "PM95").unwrap();
    let tones = mfsk_core::engine::tx::message_to_tones::<Fst4s15>(&msg);
    let sig = mfsk_core::engine::tx::synthesize::<Fst4s15>(&tones, 12_000, FREQ, 1.0);
    let mut a = vec![0f32; 15 * 12_000];
    let off = 12_000;
    let n = sig.len().min(a.len() - off);
    a[off..off + n].copy_from_slice(&sig[..n]);
    AwgnChannel::new(sigma, seed).apply(&mut a);
    // Noise at 300 rms, as a receiver with sensible gain delivers it: the
    // full-scale clicks are ~100 times louder.
    let scale = 300.0 / sigma;
    let mut out: Vec<i16> = a
        .iter()
        .map(|&x| (x * scale).clamp(-32767.0, 32767.0) as i16)
        .collect();
    if clicks {
        for (i, s) in out.iter_mut().enumerate() {
            if i % CLICK_EVERY < CLICK_LEN {
                *s = if (i / CLICK_EVERY).is_multiple_of(2) {
                    32767
                } else {
                    -32768
                };
            }
        }
    }
    out
}

fn decoded(audio: &[i16], nb: Option<NoiseBlanker>) -> Vec<String> {
    let mut req = DecodeRequest::<Fst4s15>::new(audio, 1200.0, 1800.0, 1.2, 100).freq_hint(FREQ);
    if let Some(nb) = nb {
        req = req.noise_blanker(nb);
    }
    req.decode()
        .results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .collect()
}

const WANT: &str = "CQ JA1ABC PM95";

/// Noise relative to the signal's unit amplitude. Measured over 2..14
/// (seed `0x469`): the clean slot decodes up to 10, the clicked one never
/// does without NB, and NB 2 % recovers it up to 10. 6 sits in the middle.
const SIGMA: f32 = 6.0;

#[test]
fn nb_zero_is_the_default_decode() {
    let audio = slot(SIGMA, true);
    assert_eq!(
        decoded(&audio, Some(NoiseBlanker::Percent(0))),
        decoded(&audio, None)
    );
}

#[test]
fn blanking_recovers_a_signal_under_clicks() {
    let audio = slot(SIGMA, true);
    let plain = decoded(&audio, None);
    assert!(
        !plain.iter().any(|t| t == WANT),
        "the clicks should bury the signal without NB: {plain:?}"
    );
    let nb = decoded(&audio, Some(NoiseBlanker::Percent(2)));
    assert_eq!(nb, [WANT], "NB 2 %");
    // The sweep finds it at some level above 0, near the hint.
    let sweep = decoded(
        &audio,
        Some(NoiseBlanker::Sweep {
            step: 5,
            ftol_hz: 20.0,
        }),
    );
    assert_eq!(sweep, [WANT], "NB sweep");
    // Without a hint the sweep runs its 0 % level only.
    let no_hint = DecodeRequest::<Fst4s15>::new(&audio, 1200.0, 1800.0, 1.2, 100)
        .noise_blanker(NoiseBlanker::Sweep {
            step: 5,
            ftol_hz: 20.0,
        })
        .decode()
        .results;
    assert!(no_hint.is_empty(), "{} decodes", no_hint.len());
}

#[test]
fn blanking_leaves_a_clean_signal_alone() {
    let audio = slot(SIGMA, false);
    assert_eq!(decoded(&audio, None), [WANT]);
    assert_eq!(decoded(&audio, Some(NoiseBlanker::Percent(2))), [WANT]);
}

/// Writes a 16-bit mono 12 kHz WAV, as `jt9` reads.
fn write_wav(path: &std::path::Path, a: &[i16]) {
    let mut b = Vec::with_capacity(44 + 2 * a.len());
    let data = (2 * a.len()) as u32;
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + data).to_le_bytes());
    b.extend_from_slice(b"WAVEfmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    for v in [1u16, 1] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    for v in [12_000u32, 24_000] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    for v in [2u16, 16] {
        b.extend_from_slice(&v.to_le_bytes());
    }
    b.extend_from_slice(b"data");
    b.extend_from_slice(&data.to_le_bytes());
    for s in a {
        b.extend_from_slice(&s.to_le_bytes());
    }
    std::fs::write(path, b).unwrap();
}

/// Against `jt9`: writes the clicked slots to `MFSK_NB_WAV_DIR` and prints
/// this crate's verdict per file at NB 0 and NB 2 %, for
/// `jt9 -7 -p 15 -X $((256*(NB+3)))` to be run on the same files.
#[test]
#[ignore]
fn nb_measure() {
    let Some(dir) = std::env::var_os("MFSK_NB_WAV_DIR") else {
        return;
    };
    let dir = std::path::PathBuf::from(dir);
    std::fs::create_dir_all(&dir).unwrap();
    for sigma in [4u32, 6, 8, 10, 12] {
        for seed in 0..10u64 {
            let a = slot_seeded(sigma as f32, true, 1000 + seed);
            let name = format!("nb_s{sigma:02}_{seed}.wav");
            write_wav(&dir.join(&name), &a);
            let hit = |nb| decoded(&a, nb).iter().any(|t| t == WANT) as u8;
            println!(
                "{name} {} {}",
                hit(Some(NoiseBlanker::Percent(0))),
                hit(Some(NoiseBlanker::Percent(2)))
            );
        }
    }
}
