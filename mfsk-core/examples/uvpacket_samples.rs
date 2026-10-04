// SPDX-License-Identifier: GPL-3.0-only
//! Generate representative uvpacket WAV files for documentation /
//! sanity-listening. Outputs to `audio_samples/uvpacket/` at the
//! workspace root.
//!
//! Run with:
//!
//! ```text
//! cargo run --release --example uvpacket_samples --features uvpacket
//! ```
//!
//! Each file is 12 kHz, mono, 16-bit PCM. Bursts are wrapped in
//! 200 ms of leading silence and 200 ms of trailing silence so the
//! envelope is easy to localise by ear.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::{AUDIO_CENTRE_HZ, Mode, rx, tx};

const SAMPLE_RATE: u32 = 12_000;
const SILENCE_S: f32 = 0.2;

/// Box-Muller AWGN. Same LCG-driven design as
/// `tests/common/channel.rs` (kept independent here so the example
/// has no test-harness dependency).
struct Awgn {
    state: u64,
    sigma: f32,
}

impl Awgn {
    fn new(sigma: f32, seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
            sigma,
        }
    }
    fn apply(&mut self, audio: &mut [f32]) {
        for s in audio.iter_mut() {
            *s += self.sigma * self.gaussian();
        }
    }
    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }
    fn uniform(&mut self) -> f32 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }
}

/// Single-pole-LPF flat Rayleigh fading (matches
/// `RayleighFlatChannel` in tests). Real-valued audio is multiplied
/// by `|h(n)|`.
struct Rayleigh {
    re_state: f32,
    im_state: f32,
    alpha: f32,
    inn_sigma: f32,
    state: u64,
    awgn: Awgn,
}

impl Rayleigh {
    fn new(f_doppler_hz: f32, awgn_sigma: f32, seed: u64) -> Self {
        let alpha = 1.0 - (-2.0 * std::f32::consts::PI * f_doppler_hz / SAMPLE_RATE as f32).exp();
        let inn_var = 0.5 * (2.0 - alpha) / alpha;
        Self {
            re_state: 0.0,
            im_state: 0.0,
            alpha,
            inn_sigma: inn_var.sqrt(),
            state: seed.wrapping_add(0xBF58_476D_1CE4_E5B9),
            awgn: Awgn::new(awgn_sigma, seed.wrapping_add(1)),
        }
    }
    fn apply(&mut self, audio: &mut [f32]) {
        let pre = (5.0 / self.alpha.max(1e-6)) as usize;
        for _ in 0..pre {
            let _ = self.next_mag();
        }
        for s in audio.iter_mut() {
            *s *= self.next_mag();
        }
        self.awgn.apply(audio);
    }
    fn next_mag(&mut self) -> f32 {
        let ure = self.gaussian() * self.inn_sigma;
        let uim = self.gaussian() * self.inn_sigma;
        self.re_state = (1.0 - self.alpha) * self.re_state + self.alpha * ure;
        self.im_state = (1.0 - self.alpha) * self.im_state + self.alpha * uim;
        (self.re_state * self.re_state + self.im_state * self.im_state).sqrt()
    }
    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos()
    }
    fn uniform(&mut self) -> f32 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }
}

/// Compute σ for target Eb/N0_info given measured signal power.
fn awgn_sigma(mode: Mode, eb_n0_db: f32, signal_power: f32) -> f32 {
    const K_INFO: f32 = 101.0;
    let info_rate = K_INFO / mode.ch_bits_per_block() as f32;
    let e_b_ch = signal_power / (1200.0 * 2.0);
    let e_b_info = e_b_ch / info_rate;
    let n0 = e_b_info / 10f32.powf(eb_n0_db / 10.0);
    (n0 * SAMPLE_RATE as f32 / 2.0).sqrt()
}

fn signal_power(audio: &[f32]) -> f32 {
    if audio.is_empty() {
        return 0.0;
    }
    audio.iter().map(|s| s * s).sum::<f32>() / audio.len() as f32
}

fn pad_with_silence(burst: &[f32]) -> Vec<f32> {
    let n_silence = (SILENCE_S * SAMPLE_RATE as f32) as usize;
    let mut out = Vec::with_capacity(burst.len() + 2 * n_silence);
    out.extend(std::iter::repeat_n(0.0_f32, n_silence));
    out.extend_from_slice(burst);
    out.extend(std::iter::repeat_n(0.0_f32, n_silence));
    out
}

/// Write a 12 kHz mono 16-bit PCM WAV file.
fn write_wav(path: &Path, samples: &[f32]) -> std::io::Result<()> {
    // Normalise to peak ≤ 0.95 of full-scale i16 so noisy bursts
    // don't clip at ±1.0 envelope spikes.
    let peak = samples
        .iter()
        .fold(0.0_f32, |a, &s| a.max(s.abs()))
        .max(1e-9);
    let scale = 0.95 / peak * f32::from(i16::MAX);
    let pcm: Vec<i16> = samples
        .iter()
        .map(|&s| (s * scale).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16)
        .collect();

    let n_samples = pcm.len() as u32;
    let byte_rate = SAMPLE_RATE * 2;
    let data_bytes = n_samples * 2;
    let riff_size = 36 + data_bytes;

    let mut w = BufWriter::new(File::create(path)?);
    w.write_all(b"RIFF")?;
    w.write_all(&riff_size.to_le_bytes())?;
    w.write_all(b"WAVE")?;
    w.write_all(b"fmt ")?;
    w.write_all(&16u32.to_le_bytes())?; // fmt chunk size
    w.write_all(&1u16.to_le_bytes())?; // PCM
    w.write_all(&1u16.to_le_bytes())?; // mono
    w.write_all(&SAMPLE_RATE.to_le_bytes())?;
    w.write_all(&byte_rate.to_le_bytes())?;
    w.write_all(&2u16.to_le_bytes())?; // block align
    w.write_all(&16u16.to_le_bytes())?; // bits per sample
    w.write_all(b"data")?;
    w.write_all(&data_bytes.to_le_bytes())?;
    for s in pcm.iter() {
        w.write_all(&s.to_le_bytes())?;
    }
    w.flush()?;
    Ok(())
}

fn typical_header(mode: Mode, n_blocks: u8) -> FrameHeader {
    FrameHeader {
        mode,
        block_count: n_blocks,
        app_type: 1,
        sequence: 0,
    }
}

fn typical_payload(n: usize) -> Vec<u8> {
    (0..n).map(|i| ((i ^ 0x5A) & 0xFF) as u8).collect()
}

fn main() -> std::io::Result<()> {
    let out_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("audio_samples")
        .join("uvpacket");
    std::fs::create_dir_all(&out_dir)?;
    eprintln!("output dir: {}", out_dir.display());

    let n_blocks = 4u8;
    let payload_size = 20;

    // 1. Robust clean.
    {
        let header = typical_header(Mode::Robust, n_blocks);
        let burst = tx::encode(&header, &typical_payload(payload_size), AUDIO_CENTRE_HZ).unwrap();
        let audio = pad_with_silence(&burst);
        let path = out_dir.join("uv_robust_clean.wav");
        write_wav(&path, &audio)?;
        eprintln!(
            "  {} ({} samples, {:.2} s)",
            path.display(),
            audio.len(),
            audio.len() as f32 / SAMPLE_RATE as f32
        );
    }

    // 2. Robust @ +8 dB AWGN (= clean-decode threshold per Phase 2'a).
    for eb_n0 in [8.0_f32, 4.0, 2.0] {
        let header = typical_header(Mode::Robust, n_blocks);
        let mut burst =
            tx::encode(&header, &typical_payload(payload_size), AUDIO_CENTRE_HZ).unwrap();
        let sigma = awgn_sigma(Mode::Robust, eb_n0, signal_power(&burst));
        let mut chan = Awgn::new(sigma, 0xCAFE_BABE);
        chan.apply(&mut burst);
        let audio = pad_with_silence(&burst);
        let label = format!("uv_robust_awgn_{:+02.0}db.wav", eb_n0);
        let path = out_dir.join(label);
        // Verify decode for the threshold reference:
        let result = rx::decode_known_layout(
            &audio[((SILENCE_S * SAMPLE_RATE as f32) as usize)..],
            0,
            AUDIO_CENTRE_HZ,
            Mode::Robust,
            &Default::default(),
        );
        write_wav(&path, &audio)?;
        eprintln!(
            "  {} (decode: {})",
            path.display(),
            match result {
                Ok(_) => "ok",
                Err(_) => "fail",
            }
        );
    }

    // 3. Robust + 5 Hz Rayleigh + +15 dB AWGN.
    {
        let header = typical_header(Mode::Robust, n_blocks);
        let mut burst =
            tx::encode(&header, &typical_payload(payload_size), AUDIO_CENTRE_HZ).unwrap();
        let sigma = awgn_sigma(Mode::Robust, 15.0, signal_power(&burst));
        let mut chan = Rayleigh::new(5.0, sigma, 0xDEAD_BEEF);
        chan.apply(&mut burst);
        let audio = pad_with_silence(&burst);
        let path = out_dir.join("uv_robust_rayleigh_5hz_+15db.wav");
        let result = rx::decode_known_layout(
            &audio[((SILENCE_S * SAMPLE_RATE as f32) as usize)..],
            0,
            AUDIO_CENTRE_HZ,
            Mode::Robust,
            &Default::default(),
        );
        write_wav(&path, &audio)?;
        eprintln!(
            "  {} (decode: {})",
            path.display(),
            match result {
                Ok(_) => "ok",
                Err(_) => "fail",
            }
        );
    }

    // 4. Express clean — for envelope / spectral comparison.
    {
        let header = typical_header(Mode::Express, n_blocks);
        let burst = tx::encode(&header, &typical_payload(payload_size), AUDIO_CENTRE_HZ).unwrap();
        let audio = pad_with_silence(&burst);
        let path = out_dir.join("uv_express_clean.wav");
        write_wav(&path, &audio)?;
        eprintln!(
            "  {} ({} samples, {:.2} s)",
            path.display(),
            audio.len(),
            audio.len() as f32 / SAMPLE_RATE as f32
        );
    }

    eprintln!("done.");
    Ok(())
}
