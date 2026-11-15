// SPDX-License-Identifier: GPL-3.0-only
//! The synthetic recording behind the `t3` cases of `tests/wsprd_beta6_parity.rs`, one function
//! shared by the test (which builds it in memory) and by `examples/gen_wsprd_t3_wav.rs` (which
//! writes it for `scripts/wsprd/gen_fixtures.sh` to hand to `wsprd`), so `wsprd`'s fixtures and
//! the test read the same samples without a 2.9 MB WAV in the tree.
//!
//! 120 s at 12 kHz: noise, and two transmissions starting 1 s in, three Hz apart:
//! `<K1ABC> FN42AB 37` (a Type 3, whose call `wsprd` has no table entry for, so it prints
//! `<...>`) at about -5 dB in 2500 Hz, and `W1AW FN31 30` at about -27 dB. The noise is the sum of
//! twelve integer uniform draws from an integer xorshift (Irwin-Hall, near Gaussian), so no
//! platform's libm touches it; the two waveforms are the crate's own synthesiser, rounded to `i16`
//! (which does use `sin`: a platform whose libm differs in the last bit could move a sample by one).

use mfsk_core::engine::tx::synthesize;
use mfsk_core::msg::wspr::pack_message;
use mfsk_core::wspr::{Wspr, encode_channel_symbols};

pub const RATE: u32 = 12_000;
pub const SECONDS: usize = 120;
/// Noise standard deviation in `i16` units, and the two peak amplitudes that put the
/// transmissions near -5 and -27 dB in 2500 Hz (`A^2/2 = snr * sigma^2 * 2500/6000`).
const SIGMA: i64 = 3000;
const AMP_T3: f32 = 1540.0;
const AMP_W1AW: f32 = 122.0;
const START: usize = 12_000;

struct Xorshift(u32);

impl Xorshift {
    fn next(&mut self) -> u32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        x
    }

    /// Zero-mean, standard deviation `SIGMA`.
    fn gauss(&mut self) -> i64 {
        let mut sum = 0i64;
        for _ in 0..12 {
            sum += i64::from(self.next() >> 16);
        }
        ((sum - 6 * 65_536) * SIGMA) >> 16
    }
}

fn tones(text: &str) -> [u8; 162] {
    encode_channel_symbols(&pack_message(text).expect("pack").bits)
}

/// The recording as 16-bit samples.
pub fn build() -> Vec<i16> {
    mix(&[
        ("<K1ABC> FN42AB 37", AMP_T3, 1497.0f32),
        ("W1AW FN31 30", AMP_W1AW, 1500.0f32),
    ])
}

/// A second recording, `far`: one strong `K1ABC FN42 37` at `f1` of about -126 Hz, outside
/// `wsprd`'s +/-110 Hz window until `-e -10` has moved it twice (the window is -120 on the first
/// pass and -130 on the second), so a pass 0 that decodes nothing is followed by a pass 1 that
/// does when `-B` leaves it in.
pub fn build_far() -> Vec<i16> {
    mix(&[("K1ABC FN42 37", 1090.0, 1373.0f32)])
}

fn mix(txs: &[(&str, f32, f32)]) -> Vec<i16> {
    let mut out = vec![0f32; RATE as usize * SECONDS];
    for &(text, amp, f0) in txs {
        let w = synthesize::<Wspr>(&tones(text), RATE, f0, amp);
        for (o, s) in out[START..].iter_mut().zip(&w) {
            *o += s;
        }
    }
    let mut rng = Xorshift(0x1234_5678);
    out.iter()
        .map(|&s| (s + rng.gauss() as f32).round().clamp(-32768.0, 32767.0) as i16)
        .collect()
}

/// The 44-byte header `wsprd` skips (`readwavfile` reads and ignores 22 shorts) plus the data.
pub fn wav_bytes(samples: &[i16]) -> Vec<u8> {
    let n = (samples.len() * 2) as u32;
    let mut b = Vec::with_capacity(44 + n as usize);
    b.extend_from_slice(b"RIFF");
    b.extend_from_slice(&(36 + n).to_le_bytes());
    b.extend_from_slice(b"WAVEfmt ");
    b.extend_from_slice(&16u32.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&1u16.to_le_bytes());
    b.extend_from_slice(&RATE.to_le_bytes());
    b.extend_from_slice(&(RATE * 2).to_le_bytes());
    b.extend_from_slice(&2u16.to_le_bytes());
    b.extend_from_slice(&16u16.to_le_bytes());
    b.extend_from_slice(b"data");
    b.extend_from_slice(&n.to_le_bytes());
    for s in samples {
        b.extend_from_slice(&s.to_le_bytes());
    }
    b
}
