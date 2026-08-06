//! `ft8::decode_block::decode_block_streaming` — the true no_std
//! embedded-path streaming callback (`#[cfg(not(feature =
//! "fft-rustfft"))]`, mirrors host's `DecodeRequest::on_result`).
//!
//! `decode_block_streaming` only exists when `fft-rustfft` is *off*,
//! so this file (unlike this crate's other host recall tests) must be
//! run against a feature combo that excludes it — the test binary
//! itself is still linked against the host's real `std` regardless
//! (only the *library*'s `std`/`fft-rustfft` features are toggled),
//! so ordinary `std::fs` WAV loading still works here.
//!
//! Deliberately does **not** `mod common;` — that shared test module
//! unconditionally pulls in `common::channel`/`common::air_channel`,
//! which need `rustfft`/`uvpacket` (via `fft-rustfft`/`uvpacket`),
//! defeating the entire point of this file (running with
//! `fft-rustfft` off). Inlines a minimal WAV loader instead —
//! source-faithful copy of `tests/common/mod.rs::load_wav_i16_opt`'s
//! core parse loop, trimmed to just what this file needs.
//!
//! Run:
//! ```sh
//! cargo test --release -p mfsk-core --no-default-features \
//!     --features std,ft8,fft-extern \
//!     --test ft8_decode_block_streaming -- --nocapture
//! ```
#![cfg(not(feature = "fft-rustfft"))]

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{decode_block, decode_block_streaming};

// Local, not `tests/common/mod.rs`'s — see the module doc above for why.
macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

// `fft-extern`'s contract (`engine::fft::default_planner`'s doc
// comment): the final binary supplies this symbol. A real embedded
// target bridges to a chip-native FFT here; this test binary — like
// the doc comment's own "tests / unusual targets can return any
// Box<dyn FftPlanner> impl" — just wraps the dev-dependency `rustfft`
// directly (not through this crate's own `fft-rustfft`-gated
// wrapper, which is deliberately off in this file).
mod fft_extern_impl {
    use std::sync::Arc;

    use mfsk_core::engine::fft::{Fft, FftPlanner};
    use num_complex::Complex32;

    struct TestFftPlanner {
        inner: rustfft::FftPlanner<f32>,
    }

    struct TestFftAdapter {
        inner: Arc<dyn rustfft::Fft<f32>>,
    }

    impl Fft for TestFftAdapter {
        fn process(&self, buf: &mut [Complex32]) {
            self.inner.process(buf);
        }
        fn len(&self) -> usize {
            self.inner.len()
        }
    }

    impl FftPlanner for TestFftPlanner {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft> {
            Box::new(TestFftAdapter {
                inner: self.inner.plan_fft_forward(len),
            })
        }
        fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
            Box::new(TestFftAdapter {
                inner: self.inner.plan_fft_inverse(len),
            })
        }
    }

    #[unsafe(no_mangle)]
    pub extern "Rust" fn mfsk_core_make_default_fft_planner() -> Box<dyn FftPlanner> {
        Box::new(TestFftPlanner {
            inner: rustfft::FftPlanner::new(),
        })
    }
}

/// Minimal RIFF/WAVE → mono i16 loader — see this file's module doc
/// for why this doesn't just `mod common;`.
fn load_wav_i16(path: impl AsRef<Path>) -> Vec<i16> {
    let p = path.as_ref();
    let bytes = std::fs::read(p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()));
    assert!(
        bytes.len() >= 12 && &bytes[0..4] == b"RIFF" && &bytes[8..12] == b"WAVE",
        "{} is not a RIFF/WAVE file",
        p.display()
    );
    let mut i = 12usize;
    let (mut sample_rate, mut bits, mut channels) = (0u32, 0u16, 0u16);
    let (mut data_off, mut data_len) = (0usize, 0usize);
    while i + 8 <= bytes.len() {
        let id = &bytes[i..i + 4];
        let len = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap()) as usize;
        i += 8;
        if id == b"fmt " && i + 16 <= bytes.len() {
            channels = u16::from_le_bytes(bytes[i + 2..i + 4].try_into().unwrap());
            sample_rate = u32::from_le_bytes(bytes[i + 4..i + 8].try_into().unwrap());
            bits = u16::from_le_bytes(bytes[i + 14..i + 16].try_into().unwrap());
        } else if id == b"data" {
            data_off = i;
            data_len = len.min(bytes.len() - i);
        }
        i += len + (len % 2);
    }
    assert!(
        channels == 1 && sample_rate == 12_000 && bits == 16 && data_off != 0,
        "{} must be 12 kHz / mono / 16-bit PCM",
        p.display()
    );
    bytes[data_off..data_off + data_len]
        .chunks_exact(2)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

#[test]
fn decode_block_streaming_matches_decode_block_exactly() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));

    let batch = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15);

    let mut streamed = Vec::new();
    let streamed_count = decode_block_streaming(
        &slot,
        100.0,
        3000.0,
        1.3,
        DecodeDepth::EMBEDDED,
        15,
        &mut |r| streamed.push(r.message77().to_vec()),
    )
    .len();

    println!(
        "batch: {} decode(s), streaming return: {} decode(s), callback firings: {}",
        batch.len(),
        streamed_count,
        streamed.len()
    );

    // Embedded path is always sequential (no rayon on this cfg arm) —
    // callback firings must exactly match the returned Vec, in the
    // same order, and that Vec must exactly match plain decode_block's
    // (same coarse_sync/refine/process_candidates_tuned pipeline,
    // just with a callback threaded through — no behavior change).
    let batch_msgs: Vec<Vec<u8>> = batch.iter().map(|r| r.message77().to_vec()).collect();
    assert_eq!(
        streamed, batch_msgs,
        "streamed callback order/content must exactly match decode_block's own output"
    );
    assert_eq!(streamed_count, batch.len());
    assert!(!batch.is_empty(), "expected real decodes on qso3_busy.wav");
}
