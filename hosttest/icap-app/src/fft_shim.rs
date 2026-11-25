// SPDX-License-Identifier: GPL-3.0-only
//! `fft-extern`'s contract (`mfsk_core::engine::fft::default_planner`): the final binary defines the
//! planner factory. This one wraps rustfft; on the radio it is an own mixed-radix NEON FFT
//! (`apps/q65/decoder/q65beacon/src/fft.rs` in icap-sdk is the starting point).
//!
//! The wrapper is deliberately not `engine::fft::RustFftPlanner`: that type is `fft-rustfft`, the flag this
//! crate must not have on, since it also switches `decode_block` to the three-pass host driver.

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};

use mfsk_core::engine::fft::{Fft, FftPlanner};
use num_complex::Complex32;

static CALLS: AtomicUsize = AtomicUsize::new(0);

/// How many times mfsk-core asked for a planner (the i16 one counts too under `fixed-point`). Zero after a decode means the library was built with
/// `fft-rustfft` (feature unification) and ran the host driver, not the embedded one.
pub fn factory_calls() -> usize {
    CALLS.load(Relaxed)
}

struct Planner {
    inner: rustfft::FftPlanner<f32>,
}

struct Adapter {
    inner: Arc<dyn rustfft::Fft<f32>>,
}

impl Fft for Adapter {
    fn process(&self, buf: &mut [Complex32]) {
        self.inner.process(buf);
    }
    fn len(&self) -> usize {
        self.inner.len()
    }
}

impl FftPlanner for Planner {
    fn plan_forward(&mut self, len: usize) -> Box<dyn Fft> {
        Box::new(Adapter {
            inner: self.inner.plan_fft_forward(len),
        })
    }
    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
        Box::new(Adapter {
            inner: self.inner.plan_fft_inverse(len),
        })
    }
}

#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_make_default_fft_planner() -> Box<dyn FftPlanner> {
    CALLS.fetch_add(1, Relaxed);
    Box::new(Planner {
        inner: rustfft::FftPlanner::new(),
    })
}

/// The `fixed-point` build also asks for an i16 planner (`default_planner_16`). The 3840-point spectrogram
/// FFT goes through `Plan3840Sc16`, the software port of the embedded mixed-radix kernel (a generic 1/N-scaled
/// rustfft wrap costs recall, see `tests/common/embedded_driver_harness.rs` in mfsk-core); other sizes use
/// the same wrap with a 1/N scale.
#[cfg(feature = "fixed-point")]
mod fx {
    use std::sync::Arc;

    use mfsk_core::engine::dsp::fft_mixed_3840_sc16::Plan3840Sc16;
    use mfsk_core::engine::fft::{Fft16, FftPlanner16};
    use num_complex::Complex;

    struct Planner16 {
        inner: rustfft::FftPlanner<f32>,
    }

    struct Generic {
        inner: Arc<dyn rustfft::Fft<f32>>,
    }

    impl Fft16 for Generic {
        fn process(&self, buf: &mut [Complex<i16>]) {
            assert_eq!(buf.len(), self.inner.len());
            let mut tmp: Vec<num_complex::Complex32> = buf
                .iter()
                .map(|c| num_complex::Complex32::new(c.re as f32, c.im as f32))
                .collect();
            self.inner.process(&mut tmp);
            let scale = 1.0 / tmp.len() as f32;
            let q = |v: f32| (v * scale).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            for (dst, src) in buf.iter_mut().zip(tmp.iter()) {
                dst.re = q(src.re);
                dst.im = q(src.im);
            }
        }
        fn len(&self) -> usize {
            self.inner.len()
        }
    }

    struct Mixed3840 {
        plan: Plan3840Sc16,
    }

    impl Fft16 for Mixed3840 {
        fn process(&self, buf: &mut [Complex<i16>]) {
            self.plan.process(buf);
        }
        fn len(&self) -> usize {
            3840
        }
    }

    impl FftPlanner16 for Planner16 {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft16> {
            if len == 3840 {
                return Box::new(Mixed3840 {
                    plan: Plan3840Sc16::new(),
                });
            }
            Box::new(Generic {
                inner: self.inner.plan_fft_forward(len),
            })
        }
        fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft16> {
            Box::new(Generic {
                inner: self.inner.plan_fft_inverse(len),
            })
        }
    }

    #[unsafe(no_mangle)]
    pub extern "Rust" fn mfsk_core_make_default_fft_planner_16() -> Box<dyn FftPlanner16> {
        super::CALLS.fetch_add(1, super::Relaxed);
        Box::new(Planner16 {
            inner: rustfft::FftPlanner::new(),
        })
    }
}
