//! What a `not(feature = "fft-rustfft")` integration test needs before
//! it can run at all: the `fft-extern` contract, and a WAV loader that
//! does not reach for `mod common`'s own (which pulls `fft-rustfft`
//! through `air_channel.rs`).
//!
//! Mounted by `#[path]` from each such test — `ft8_embedded_driver_
//! recall.rs` and `ft8_embedded_grid_phase.rs` today — because every
//! integration test is its own crate and the `no_mangle` factories
//! below have to be linked into each binary separately. It was inline
//! in the first of those until a second test wanted the same 180 lines
//! (2026-09-16).
//!
//! The shim is deliberately *not* `engine::fft`'s own
//! `RustFftPlanner`: that type is `#[cfg(feature = "fft-rustfft")]`,
//! and that flag is exactly the one these tests turn off — enabling it
//! would also switch `decode_block_multipass` to its three-pass
//! subtracting body, i.e. measure the host driver instead of the
//! board's.

// Same shim as `ft8_decode_block_streaming.rs`, which still carries
// its own copy.
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

    use mfsk_core::engine::dsp::fft_mixed_2560_sc16::Plan2560Sc16;
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

    /// The 8 kHz spectrogram's 2560 = 256 x 10 (gain `1/256` like 3840, not the generic `1/N`).
    struct MixedRadix2560Adapter {
        plan: Plan2560Sc16,
    }

    impl Fft16 for MixedRadix2560Adapter {
        fn process(&self, buf: &mut [Complex<i16>]) {
            self.plan.process(buf);
        }
        fn len(&self) -> usize {
            2560
        }
    }

    impl FftPlanner16 for TestFftPlanner16 {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft16> {
            if len == 3840 {
                return Box::new(MixedRadix3840Adapter {
                    plan: Plan3840Sc16::new(),
                });
            }
            if len == 2560 {
                return Box::new(MixedRadix2560Adapter {
                    plan: Plan2560Sc16::new(),
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

use std::path::Path;

/// Source-faithful copy of `ft8_decode_block_streaming.rs`'s loader.
pub fn load_wav_i16(path: impl AsRef<Path>) -> Vec<i16> {
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
