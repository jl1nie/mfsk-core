//! FT4 a-priori decoding: it works now, and it does not invent messages.
//!
//! ## The bug this pins
//!
//! An `ApHint` describes the **message**. FT4 and FST4 XOR that message
//! with their own RVEC before CRC and FEC encode
//! (`ModulationParams::INFO_SCRAMBLE_RVEC`), so the info bits inside the
//! codeword — the thing the decoder is actually working on — are the
//! *scrambled* message. `ap_bits_for` handed the decoder message-space
//! values, which meant roughly half the locked bits were pinned to the
//! opposite of the truth.
//!
//! That is worse than not locking them: BP was being told, with high
//! confidence, that bits were the wrong value. It is why AP-hinted FT4
//! decoding measured *worse* than plain decoding, and why the sniper —
//! whose entire premise is trading band for AP gain — was losing to a
//! wide-band decode on the same audio.
//!
//! FT8 has no RVEC and was never affected, which is the other reason
//! this survived: the protocol where AP is most used is the one where
//! it worked.
//!
//! ## What is asserted
//!
//! 1. **Gain.** AP must beat plain decoding at threshold. Without the
//!    fix this fails in the most informative way — the hinted run scores
//!    *lower* than the unhinted one.
//! 2. **Precision.** A hint naming a station that is not transmitting
//!    must not produce that station. This is AP's real risk: locking
//!    bits tells every candidate, including noise, what some of its bits
//!    "are".
#![cfg(all(feature = "ft4", feature = "fft-rustfft"))]

use mfsk_core::ft4::Ft4;
use mfsk_core::msg::ap::ApHint;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

const FS: f32 = 12_000.0;
const SLOT: usize = 90_000;
const REF_BW: f32 = 2_500.0;
const TRIALS: u64 = 12;

/// Same LCG and SNR convention as `ft4_snr_sweep`, so the numbers here
/// are comparable to that file's.
struct Lcg(u64);
impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed.wrapping_add(1))
    }
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }
    fn u(&mut self) -> f32 {
        ((self.next() >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    }
    fn gauss(&mut self) -> f32 {
        let (u, v) = (self.u(), self.u());
        (-2.0 * u.ln()).sqrt() * (2.0 * core::f32::consts::PI * v).cos()
    }
}

fn make_slot(msg77: &[u8; 77], freq_hz: f32, snr_db: f32, seed: u64) -> Vec<i16> {
    let mut mix = vec![0.0f32; SLOT];
    let amp = (4.0 * 10f32.powf(snr_db / 10.0) * REF_BW / FS).sqrt();
    let itone = mfsk_core::engine::tx::message_to_tones::<mfsk_core::ft4::Ft4>(msg77);
    let pcm =
        mfsk_core::engine::tx::synthesize::<mfsk_core::ft4::Ft4>(&itone, 12_000, freq_hz, amp);
    let start = (0.5 * FS) as usize;
    for (i, s) in pcm.iter().take(SLOT - start).enumerate() {
        mix[start + i] += s;
    }
    let mut rng = Lcg::new(seed);
    for s in mix.iter_mut() {
        *s += rng.gauss();
    }
    let peak = mix.iter().map(|x| x.abs()).fold(0.0f32, f32::max).max(1e-6);
    let scale = 29_000.0 / peak;
    mix.iter()
        .map(|&s| (s * scale).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

/// The QSO frequency the hinted requests pass as `freq_hint`: where `make_slot` puts the
/// signal. The hints here lock both callsigns, and `ft4_decode.f90` tries such a hypothesis
/// only within `napwid` of `nfqso` (#456), so without it they would not run at all. At the
/// signal is also the hardest place for the "does not conjure" tests below.
const QSO_HZ: f32 = 1000.0;

fn hits(audio: &[i16], hint: Option<&ApHint>, want: &[u8; 77]) -> bool {
    let req = DecodeRequest::<Ft4>::new(audio, 300.0, 2700.0, 1.2, 50);
    let req = match hint {
        Some(h) => req.ap_hint(h).freq_hint(QSO_HZ),
        None => req,
    };
    req.decode().results.iter().any(|r| r.message77() == want)
}

#[test]
fn ap_beats_plain_decoding_at_threshold() {
    let msg = pack77("CQ", "JA1ABC", "PM95").expect("pack");
    let hint = ApHint::new()
        .with_call1("CQ")
        .with_call2("JA1ABC")
        .with_grid("PM95");

    let mut plain_total = 0;
    let mut ap_total = 0;
    for snr in [-17i32, -18, -19] {
        let (mut plain, mut aped) = (0, 0);
        for seed in 0..TRIALS {
            let audio = make_slot(&msg, QSO_HZ, snr as f32, 0x51EED + seed);
            if hits(&audio, None, &msg) {
                plain += 1;
            }
            if hits(&audio, Some(&hint), &msg) {
                aped += 1;
            }
        }
        println!("{snr:>4} dB: plain {plain:>2}/{TRIALS}, ap {aped:>2}/{TRIALS}");
        plain_total += plain;
        ap_total += aped;
        assert!(
            aped >= plain,
            "AP decoded fewer than plain at {snr} dB ({aped} vs {plain}) — locked bits are \
             hurting, which is what message-space values in a scrambled codeword look like"
        );
    }
    assert!(
        ap_total > plain_total,
        "AP bought nothing across the threshold region ({ap_total} vs {plain_total} of \
         {}); on FT4 that is the signature of the hint being XORed against the wrong space",
        3 * TRIALS
    );
}

/// AP's risk is manufactured decodes: locking bits tells every
/// candidate, noise included, what some of its bits "are". A hint for a
/// station that is not on the air must not put it there.
#[test]
fn a_hint_for_a_silent_station_does_not_conjure_it() {
    let present = pack77("CQ", "JA1ABC", "PM95").expect("pack");
    let absent_hint = ApHint::new()
        .with_call1("CQ")
        .with_call2("ZL4ZZZ")
        .with_grid("RE66");
    let absent_msg = pack77("CQ", "ZL4ZZZ", "RE66").expect("pack");

    for snr in [-15i32, -18, -21] {
        for seed in 0..TRIALS {
            let audio = make_slot(&present, QSO_HZ, snr as f32, 0xA11CE + seed);
            let got = DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50)
                .ap_hint(&absent_hint)
                .freq_hint(QSO_HZ)
                .decode()
                .results;
            assert!(
                !got.iter().any(|r| *r.message77() == absent_msg),
                "AP conjured the hinted-but-silent station at {snr} dB, seed {seed}"
            );
            for r in &got {
                let text = unpack77(r.message77()).unwrap_or_default();
                assert!(
                    !text.contains("ZL4ZZZ"),
                    "a decode mentions the absent hinted call at {snr} dB: {text:?}"
                );
            }
        }
    }
}

/// Pure noise plus a confident hint is the strongest form of the same
/// question: with nothing to decode, AP must return nothing.
#[test]
fn a_hint_over_noise_decodes_nothing() {
    let hint = ApHint::new()
        .with_call1("CQ")
        .with_call2("JA1ABC")
        .with_grid("PM95");
    for seed in 0..TRIALS {
        let mut rng = Lcg::new(0xDEAD + seed);
        let noise: Vec<i16> = (0..SLOT)
            .map(|_| (rng.gauss() * 3_000.0).clamp(-32_768.0, 32_767.0) as i16)
            .collect();
        let got = DecodeRequest::<Ft4>::new(&noise, 300.0, 2700.0, 1.2, 50)
            .ap_hint(&hint)
            .freq_hint(QSO_HZ)
            .decode()
            .results;
        assert!(
            got.is_empty(),
            "AP produced {} decode(s) from pure noise (seed {seed}): {:?}",
            got.len(),
            got.iter()
                .map(|r| unpack77(r.message77()).unwrap_or_default())
                .collect::<Vec<_>>()
        );
    }
}
