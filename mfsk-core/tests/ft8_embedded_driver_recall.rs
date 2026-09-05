//! What `decode_block` — the board's driver, `#[cfg(not(feature =
//! "fft-rustfft"))]` — actually recovers of the fixed WSJT-X/JTDX
//! golden for `qso3_busy.wav`, instead of against a same-build
//! `DecodeRequest` reference.
//!
//! ## Why not `ft8_qso3_apoff_recall.rs`
//!
//! That file already runs this exact ship config
//! (`decode_block(..., DecodeDepth::EMBEDDED, 15)`, `sync_min=1.3`)
//! against the fixed 20-entry `QSO3_KNOWN_REAL_SIGNALS` golden — but
//! it is `#![cfg(feature = "fft-rustfft")]`. `decode_block`'s own
//! `decode_block_multipass` has two bodies split on that exact flag:
//! three passes with subtraction between them under `fft-rustfft`,
//! one pass with none without it. So that test's 14/12 floors, and
//! every other host recall-floor test's numbers, describe the
//! **3-pass driver — not the one any embedded build ships.**
//!
//! ## Why not a same-build `DecodeRequest` "truth" either
//!
//! `ft8_fixed_point_reference_drift.rs` assumed `DecodeRequest`'s
//! output was immune to this because its own code has no
//! `fft-rustfft` cfg branch. It compiles a completely different
//! internal path when `fft-rustfft` is off regardless — measured
//! directly: `DecodeRequest::new(...).decode()` (`max_cand=200`) on
//! `qso3_busy.wav` returns 15 messages under `full`, **8** under
//! `alloc,ft8,fft-extern` — because its default single-pass strategy
//! (`Ft8::__single_pass` -> `decode_frame_inner`) itself gates
//! `apply_wsjtx_xsnr2` and reaches into `fill_symbol_spectra.rs` /
//! `osd_strategy.rs`, both independently `fft-rustfft`-split. A
//! same-build reference is not a fixed reference; this crate's own
//! `#359` thread already said the floors want one, and this is why.
//!
//! So this file measures the ship config against the **external,
//! feature-independent** golden — WSJT-X's own AP-off decode plus
//! JTDX's, reconciled in `common::ft8_qso3::QSO3_KNOWN_REAL_SIGNALS` —
//! reusing the same `#359`/`ft8_decode_block_streaming.rs` pattern of
//! providing `fft-extern`'s contract directly rather than pulling in
//! `mod common` (which needs `fft-rustfft`/`uvpacket` to compile at
//! all). `common::golden`/`common::ft8_qso3` have no such dependency,
//! so they are mounted directly by path instead of duplicated.
//!
//! Run against the board's actual matrix entry:
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_embedded_driver_recall -- --nocapture
//! ```
//! and its f32 twin (drop `fixed-point`) for the pre-quantisation number.
//!
//! Refs #359, #357.
#![cfg(not(feature = "fft-rustfft"))]

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::decode_block;
use mfsk_core::msg::wsjt77::unpack77;

#[path = "common/ft8_qso3.rs"]
mod ft8_qso3;
#[path = "common/golden.rs"]
mod golden;

use ft8_qso3::{DF_TOL_HZ, QSO3_KNOWN_REAL_SIGNALS, SNR_TOL_DB};

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

// Same shim as `ft8_decode_block_streaming.rs` — see that file's
// module doc for why a host test provides `fft-extern`'s contract
// itself instead of using this crate's own `fft-rustfft` wrapper.
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

// `fixed-point`'s i16 sibling of the shim above. `engine::fft` already
// carries a host stub for this (`RustFftPlanner16`,
// `#[cfg(feature = "fft-rustfft")]`) — copied here rather than reused
// because that cfg is exactly the one this file turns off. Its own
// doc comment is the reason this isn't a generic 1/N-scaled rustfft
// wrap: that mismatch previously cost host `compute_spectrogram`
// (fixed-point) 3 decodes against embedded's 7 on this same
// `qso3_busy.wav`, on this exact 3840-pt spectrogram FFT. Same fix:
// route the FT8 spectrogram size through the software port of the
// embedded mixed-radix sc16 kernel, `Plan3840Sc16`.
#[cfg(feature = "fixed-point")]
mod fft_extern_impl_16 {
    use std::sync::Arc;

    use mfsk_core::engine::dsp::fft_mixed_3840_sc16::Plan3840Sc16;
    use mfsk_core::engine::fft::{Fft16, FftPlanner16};
    use num_complex::Complex;

    pub struct TestFftPlanner16 {
        inner: rustfft::FftPlanner<f32>,
    }

    struct GenericAdapter {
        inner: Arc<dyn rustfft::Fft<f32>>,
    }

    impl Fft16 for GenericAdapter {
        fn process(&self, buf: &mut [Complex<i16>]) {
            assert_eq!(buf.len(), self.inner.len());
            let mut tmp: Vec<num_complex::Complex32> = buf
                .iter()
                .map(|c| num_complex::Complex32::new(c.re as f32, c.im as f32))
                .collect();
            self.inner.process(&mut tmp);
            let scale = 1.0 / tmp.len() as f32;
            for (dst, src) in buf.iter_mut().zip(tmp.iter()) {
                dst.re = (src.re * scale)
                    .round()
                    .clamp(i16::MIN as f32, i16::MAX as f32) as i16;
                dst.im = (src.im * scale)
                    .round()
                    .clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            }
        }
        fn len(&self) -> usize {
            self.inner.len()
        }
    }

    struct MixedRadix3840Adapter {
        plan: Plan3840Sc16,
    }

    impl Fft16 for MixedRadix3840Adapter {
        fn process(&self, buf: &mut [Complex<i16>]) {
            self.plan.process(buf);
        }
        fn len(&self) -> usize {
            3840
        }
    }

    impl FftPlanner16 for TestFftPlanner16 {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft16> {
            if len == 3840 {
                return Box::new(MixedRadix3840Adapter {
                    plan: Plan3840Sc16::new(),
                });
            }
            Box::new(GenericAdapter {
                inner: self.inner.plan_fft_forward(len),
            })
        }
        fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft16> {
            Box::new(GenericAdapter {
                inner: self.inner.plan_fft_inverse(len),
            })
        }
    }

    #[unsafe(no_mangle)]
    pub extern "Rust" fn mfsk_core_make_default_fft_planner_16() -> Box<dyn FftPlanner16> {
        Box::new(TestFftPlanner16 {
            inner: rustfft::FftPlanner::new(),
        })
    }
}

/// Source-faithful copy of `ft8_decode_block_streaming.rs`'s loader —
/// duplicated rather than shared because each integration-test file is
/// its own crate and this one deliberately carries no `mod common`.
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
        .as_chunks::<2>()
        .0
        .iter()
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect()
}

/// Hard-assertion floor for the driver every embedded build actually
/// ships (`decode_block_multipass`'s `not(fft-rustfft)` body) against
/// the fixed, feature-independent golden. Measured 2026-09-06:
///
/// | numeric | recall | phantoms |
/// |---|---:|---:|
/// | f32 | 4/20 | 0 |
/// | fixed-point (what the board runs) | 7/20 | 0 |
///
/// `assert_golden` isn't reused — it also checks per-entry SNR, and no
/// independent SNR reference exists for this driver/golden pairing yet
/// (unlike `ft8_qso3_apoff_recall.rs`'s tolerance-checked SNRs). This
/// stays a plain recall/phantom check until one does.
#[cfg(not(feature = "fixed-point"))]
const MIN_HITS: usize = 4;
#[cfg(feature = "fixed-point")]
const MIN_HITS: usize = 7;
const MAX_EXTRA: usize = 0;

#[test]
fn true_ship_driver_meets_fixed_golden_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let decoded = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15);
    let msgs: Vec<String> = decoded
        .iter()
        .filter_map(|d| unpack77(d.message77()))
        .collect();

    println!(
        "\ndriver: single-pass, no subtraction (cfg(not(fft-rustfft)))  numeric: {}\n",
        if cfg!(feature = "fixed-point") {
            "fixed-point"
        } else {
            "f32"
        }
    );

    let hits: Vec<&str> = QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .filter(|g| msgs.iter().any(|m| m == g.msg))
        .map(|g| g.msg)
        .collect();
    let missing: Vec<&str> = QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .map(|g| g.msg)
        .filter(|m| !hits.contains(m))
        .collect();
    let phantoms: Vec<&str> = msgs
        .iter()
        .filter(|m| !QSO3_KNOWN_REAL_SIGNALS.iter().any(|g| g.msg == **m))
        .map(|m| m.as_str())
        .collect();

    println!(
        "recall {}/{} (floor {MIN_HITS})  phantoms {} (ceiling {MAX_EXTRA})",
        hits.len(),
        QSO3_KNOWN_REAL_SIGNALS.len(),
        phantoms.len()
    );
    println!("hit:      {hits:?}");
    println!("missing:  {missing:?}");
    println!("phantoms: {phantoms:?}");

    assert!(
        hits.len() >= MIN_HITS,
        "recall regressed: {}/{} decoded, floor is {MIN_HITS}. Missing: {missing:?}",
        hits.len(),
        QSO3_KNOWN_REAL_SIGNALS.len()
    );
    assert!(
        phantoms.len() <= MAX_EXTRA,
        "emitted {} decode(s) outside the golden set, budget is {MAX_EXTRA}. Phantoms: {phantoms:?}",
        phantoms.len()
    );
    // Reserved for when this test also checks per-entry SNR.
    let _ = (DF_TOL_HZ, SNR_TOL_DB);
}
