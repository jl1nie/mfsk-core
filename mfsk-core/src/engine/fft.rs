// SPDX-License-Identifier: GPL-3.0-or-later
//! Pluggable FFT backend for the decode pipeline.
//!
//! `mfsk-core`'s decode-side modules (sync correlation, LLR symbol
//! spectra, downsample / subtract) all need a single-precision
//! complex FFT. The host build links `rustfft` (the de-facto Rust
//! FFT crate; SIMD-optimised on x86 / aarch64). On embedded targets
//! we cannot use rustfft (it is std-only and its `FftPlanner`
//! depends on thread-local state), so this module defines a small
//! [`Fft`] / [`FftPlanner`] trait pair that callers fulfil with a
//! backend appropriate for their target.
//!
//! ## Built-in backends
//!
//! - [`fft-rustfft`](crate#features) (default for std builds) —
//!   forwards to `rustfft::FftPlanner`. Supports any size, SIMD
//!   accelerated where the host CPU has it. See [`RustFftPlanner`].
//! - [`fft-extern`](crate#features) (caller-provided) — the calling
//!   binary defines a Rust `extern` factory function and the rest of
//!   `mfsk-core`'s decode pipeline picks it up automatically. Use
//!   this on ESP32-S3 to bridge to `esp-dsp` via `esp-idf-sys`, on
//!   RP2350 to bridge to CMSIS-DSP, or for any custom backend (FPGA
//!   accelerator, etc.). See [`default_planner`] for the contract.
//!
//! ## Trait shape
//!
//! The trait deliberately mirrors `rustfft`'s `Fft` / `FftPlanner`
//! API so the existing call sites need only a thin adaptation:
//!
//! - [`FftPlanner::plan_forward`] / [`FftPlanner::plan_inverse`]
//!   return a boxed [`Fft`] for the requested size.
//! - [`Fft::process`] runs the transform in-place on a complex slice
//!   (length must equal [`Fft::len`]).
//!
//! Backends are free to cache plans internally; callers should keep
//! a single planner per decode session and reuse it across sizes.

use alloc::boxed::Box;

use num_complex::{Complex, Complex32};

/// In-place complex single-precision FFT for one fixed length.
pub trait Fft {
    /// Run the FFT in-place. `buf.len()` must equal [`Fft::len`];
    /// shorter or longer slices are a programming error.
    fn process(&self, buf: &mut [Complex32]);

    /// Length of the transform this instance was planned for.
    fn len(&self) -> usize;

    /// Convenience — returns `true` when the backend cannot transform
    /// any data (e.g. caller passed `len == 0` to the planner).
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Four `Complex32` carrying a 16-byte alignment guarantee. Backing
/// store for [`AlignedComplexBuf`]; not useful on its own.
#[repr(align(16))]
#[derive(Clone, Copy)]
struct Align16Quad(
    // Only ever read through the reinterpreting slice in
    // `AlignedComplexBuf::as_mut_slice` — the field exists to size and
    // align the allocation.
    #[allow(dead_code)] [Complex32; 4],
);

impl Align16Quad {
    const ZERO: Self = Self([Complex32::new(0.0, 0.0); 4]);
}

/// A zeroed `Complex32` working buffer guaranteed to start on a
/// 16-byte boundary.
///
/// [`Fft::process`] takes a plain `&mut [Complex32]`, and `Complex32`
/// is `repr(C)` over two `f32`, so its alignment is only 4. Backends
/// that dispatch to SIMD kernels generally want more: the ESP32-S3
/// (LX7) PIE kernels `esp-dsp` compiles under `aes3` move float
/// *pairs* with `ee.ldf.64.ip` / `ee.stf.64.ip` and require 16, and
/// the `mfsk-core`-side backend in `embedded-shared` honours that by
/// copying any under-aligned caller buffer through an aligned staging
/// area and copying the result back.
///
/// That fallback is correct but not free, and on issue #260 it was
/// measured firing on **every** call: a device probe of the WSPR
/// candidate loop counted 1 182 staged transforms against 2 in-place
/// ones, i.e. the allocator hands back a 16-byte-aligned block only
/// by luck. For the 8 192-point transforms in [`crate::wspr::subtract`]
/// that is 64 KB copied in and 64 KB copied back out per call, 13
/// times per subtraction, against PSRAM-resident buffers.
///
/// Callers that plan a hot, repeatedly-transformed buffer should
/// allocate it through this type instead of `vec![Complex32; n]`.
/// Backends with no alignment requirement (`rustfft` on the host) are
/// unaffected — an over-aligned buffer is still a valid one.
pub struct AlignedComplexBuf {
    quads: alloc::vec::Vec<Align16Quad>,
    len: usize,
}

impl AlignedComplexBuf {
    /// Allocate `len` zeroed `Complex32`, 16-byte aligned.
    ///
    /// `len` is rounded up to a multiple of 4 internally; the slice
    /// handed back is exactly `len` long. Every size an FFT backend
    /// plans is a power of two ≥ 4, so in practice no rounding occurs.
    pub fn zeroed(len: usize) -> Self {
        Self {
            quads: alloc::vec![Align16Quad::ZERO; len.div_ceil(4)],
            len,
        }
    }

    /// The buffer as a `Complex32` slice — what [`Fft::process`] takes.
    pub fn as_mut_slice(&mut self) -> &mut [Complex32] {
        // SAFETY: `Align16Quad` is `repr(align(16))` over
        // `[Complex32; 4]` with no padding, so `quads` is exactly
        // `4 * quads.len()` contiguous `Complex32` starting on a
        // 16-byte boundary, and `len <= 4 * quads.len()` by
        // construction in `zeroed`.
        unsafe {
            core::slice::from_raw_parts_mut(self.quads.as_mut_ptr() as *mut Complex32, self.len)
        }
    }

    /// Read-only view of the same buffer.
    pub fn as_slice(&self) -> &[Complex32] {
        // SAFETY: as in `as_mut_slice`.
        unsafe { core::slice::from_raw_parts(self.quads.as_ptr() as *const Complex32, self.len) }
    }
}

/// Plans (and typically caches) [`Fft`] instances on demand.
///
/// Callers should construct one planner per decode session and reuse
/// it across calls — backends like `rustfft` cache their twiddle
/// tables between `plan_*` invocations of the same size.
pub trait FftPlanner {
    /// Plan a forward FFT of length `len`. Returns a boxed instance
    /// the caller drives via [`Fft::process`].
    fn plan_forward(&mut self, len: usize) -> Box<dyn Fft>;

    /// Plan an inverse FFT of length `len`.
    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft>;
}

// ── i16 (fixed-point) trait pair ─────────────────────────────────────
//
// Mirror of [`Fft`] / [`FftPlanner`] for `Complex<i16>` data. Used by
// the embedded `decode_block` under the `fixed-point` feature flag —
// bandwidth-bound stages (spectrogram, allsum) halve their PSRAM
// traffic vs the f32 path. On embedded targets the planner wraps a
// chip-native i16 FFT (esp-dsp `dsps_fft2r_sc16`, CMSIS-DSP
// `arm_cfft_q15`, …); on host a stub re-quantises through rustfft
// for sensitivity validation only.

/// In-place complex i16 FFT for one fixed length. The data layout is
/// interleaved `{re, im, re, im, …}` matching `num_complex::Complex<i16>`'s
/// `repr(C)` layout, which in turn matches the `int16_t *` ABI used by
/// the major embedded FFT libs.
pub trait Fft16 {
    fn process(&self, buf: &mut [Complex<i16>]);
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// Plans (and typically caches) [`Fft16`] instances on demand.
pub trait FftPlanner16 {
    fn plan_forward(&mut self, len: usize) -> Box<dyn Fft16>;
    fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft16>;
}

// ── rustfft backend ──────────────────────────────────────────────────

#[cfg(feature = "fft-rustfft")]
mod rustfft_backend {
    use super::*;
    use alloc::sync::Arc;

    /// rustfft-backed [`FftPlanner`]. Single-precision, any size,
    /// SIMD-accelerated where the host CPU supports it.
    pub struct RustFftPlanner {
        inner: rustfft::FftPlanner<f32>,
    }

    impl RustFftPlanner {
        /// Construct a planner. Cheap; reuse the same instance across
        /// all decodes in a session so rustfft's twiddle cache hits.
        pub fn new() -> Self {
            Self {
                inner: rustfft::FftPlanner::new(),
            }
        }
    }

    impl Default for RustFftPlanner {
        fn default() -> Self {
            Self::new()
        }
    }

    struct RustFftAdapter {
        inner: Arc<dyn rustfft::Fft<f32>>,
    }

    impl Fft for RustFftAdapter {
        fn process(&self, buf: &mut [Complex32]) {
            self.inner.process(buf);
        }
        fn len(&self) -> usize {
            self.inner.len()
        }
    }

    impl FftPlanner for RustFftPlanner {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft> {
            Box::new(RustFftAdapter {
                inner: self.inner.plan_fft_forward(len),
            })
        }
        fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft> {
            Box::new(RustFftAdapter {
                inner: self.inner.plan_fft_inverse(len),
            })
        }
    }
}

#[cfg(feature = "fft-rustfft")]
pub use rustfft_backend::RustFftPlanner;

// ── rustfft-based host stub for the i16 traits ───────────────────────
//
// Quantises i16 input → f32 → rustfft → f32 → i16, with stage-wise
// scaling to match what `dsps_fft2r_sc16`'s asm does on Xtensa. Speed
// is irrelevant on host; correctness for the AWGN sweep gate is the
// only goal.

#[cfg(feature = "fft-rustfft")]
mod rustfft_backend_i16 {
    use super::*;
    use alloc::sync::Arc;

    pub struct RustFftPlanner16 {
        inner: rustfft::FftPlanner<f32>,
    }

    impl RustFftPlanner16 {
        pub fn new() -> Self {
            Self {
                inner: rustfft::FftPlanner::new(),
            }
        }
    }

    impl Default for RustFftPlanner16 {
        fn default() -> Self {
            Self::new()
        }
    }

    struct RustFft16Adapter {
        inner: Arc<dyn rustfft::Fft<f32>>,
    }

    impl Fft16 for RustFft16Adapter {
        fn process(&self, buf: &mut [Complex<i16>]) {
            assert_eq!(buf.len(), self.inner.len());
            let mut tmp: alloc::vec::Vec<Complex32> = buf
                .iter()
                .map(|c| Complex32::new(c.re as f32, c.im as f32))
                .collect();
            self.inner.process(&mut tmp);
            // Generic fallback: per-stage /2 total /N (correct for
            // pure radix-2 sizes used by AWGN-sweep tests). The
            // 3840-pt FT8 spectrogram path takes the dedicated
            // mixed-radix sc16 adapter below.
            let n = tmp.len() as f32;
            let scale = 1.0 / n;
            for (dst, src) in buf.iter_mut().zip(tmp.iter()) {
                let re = (src.re * scale)
                    .round()
                    .clamp(i16::MIN as f32, i16::MAX as f32);
                let im = (src.im * scale)
                    .round()
                    .clamp(i16::MIN as f32, i16::MAX as f32);
                dst.re = re as i16;
                dst.im = im as i16;
            }
        }
        fn len(&self) -> usize {
            self.inner.len()
        }
    }

    /// Phase 1.7.7a host adapter for the 3840-pt FT8 spectrogram FFT.
    /// Bit-exact match to embedded `MixedRadix3840Sc16Fft` (esp-dsp
    /// 256-pt sc16 + f32 PFA), via the software port in
    /// `engine::dsp::fft_mixed_3840_sc16::Plan3840Sc16`.
    ///
    /// Without this adapter `RustFft16Adapter` ran a single rustfft
    /// 3840-pt + flat `1/N` scaling — mathematically the same DFT
    /// but with completely different per-stage quantisation error
    /// than embedded. The mismatch caused weak-signal recall to
    /// diverge between host `compute_spectrogram` (fixed-point) and
    /// the S3 wav_sim baseline (host 3 vs embedded 7 on qso3_busy).
    struct MixedRadix3840Sc16Adapter {
        plan: crate::engine::dsp::fft_mixed_3840_sc16::Plan3840Sc16,
    }

    impl Fft16 for MixedRadix3840Sc16Adapter {
        fn process(&self, buf: &mut [Complex<i16>]) {
            self.plan.process(buf);
        }
        fn len(&self) -> usize {
            3840
        }
    }

    impl FftPlanner16 for RustFftPlanner16 {
        fn plan_forward(&mut self, len: usize) -> Box<dyn Fft16> {
            if len == 3840 {
                return Box::new(MixedRadix3840Sc16Adapter {
                    plan: crate::engine::dsp::fft_mixed_3840_sc16::Plan3840Sc16::new(),
                });
            }
            Box::new(RustFft16Adapter {
                inner: self.inner.plan_fft_forward(len),
            })
        }
        fn plan_inverse(&mut self, len: usize) -> Box<dyn Fft16> {
            assert_ne!(
                len, 3840,
                "inverse 3840-pt sc16 FFT not implemented \
                 (FT8 spectrogram path is forward only)"
            );
            Box::new(RustFft16Adapter {
                inner: self.inner.plan_fft_inverse(len),
            })
        }
    }
}

#[cfg(feature = "fft-rustfft")]
pub use rustfft_backend_i16::RustFftPlanner16;

// ── Default planner constructor ──────────────────────────────────────

/// Construct the default [`FftPlanner`] for the active FFT backend
/// feature.
///
/// **Resolution order**:
/// 1. `fft-rustfft` (host default) — returns a fresh
///    [`RustFftPlanner`].
/// 2. `fft-extern` — calls the binary-provided factory function
///    declared as:
///    ```ignore
///    #[unsafe(no_mangle)]
///    pub extern "Rust" fn mfsk_core_make_default_fft_planner()
///        -> Box<dyn mfsk_core::engine::fft::FftPlanner>;
///    ```
///    The binary must define this symbol; missing it is a link-time
///    error. Typical ESP32-S3 implementation wraps an
///    `EspDspPlanner` (esp-dsp ASM); RP2350 builds wrap a
///    CMSIS-DSP-backed planner; tests / unusual targets can return
///    any `Box<dyn FftPlanner>` impl.
///
/// At least one of these features must be enabled whenever decode-
/// side code that calls `default_planner()` is compiled.
#[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
#[inline]
pub fn default_planner() -> Box<dyn FftPlanner> {
    #[cfg(feature = "fft-rustfft")]
    {
        Box::new(RustFftPlanner::new())
    }
    #[cfg(all(not(feature = "fft-rustfft"), feature = "fft-extern"))]
    {
        unsafe extern "Rust" {
            fn mfsk_core_make_default_fft_planner() -> Box<dyn FftPlanner>;
        }
        // SAFETY: the linker enforces that exactly one binary in the
        // dependency closure defines this symbol; if the symbol is
        // missing the link fails. The factory's safety contract is
        // simply that it returns a valid `Box<dyn FftPlanner>`.
        unsafe { mfsk_core_make_default_fft_planner() }
    }
}

/// i16 sibling of [`default_planner`]. Gated behind `fixed-point` —
/// only embedded builds with that feature need an i16 backend.
#[cfg(all(
    feature = "fixed-point",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]
#[inline]
pub fn default_planner_16() -> Box<dyn FftPlanner16> {
    #[cfg(feature = "fft-rustfft")]
    {
        Box::new(RustFftPlanner16::new())
    }
    #[cfg(all(not(feature = "fft-rustfft"), feature = "fft-extern"))]
    {
        unsafe extern "Rust" {
            fn mfsk_core_make_default_fft_planner_16() -> Box<dyn FftPlanner16>;
        }
        unsafe { mfsk_core_make_default_fft_planner_16() }
    }
}

// ── tests ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests_aligned_buf {
    use super::*;

    /// The whole point of the type. A plain `vec![Complex32; n]` makes
    /// no such promise — that is what the ESP32-S3 PIE backend has to
    /// defend against by copying through staging.
    #[test]
    fn zeroed_is_16_byte_aligned_and_the_requested_length() {
        // Several sizes, including the 8192 `wspr::subtract` plans and
        // a non-multiple-of-4 to exercise the round-up in `zeroed`.
        for len in [4usize, 512, 4096, 8192, 7] {
            let mut buf = AlignedComplexBuf::zeroed(len);
            assert_eq!(buf.as_mut_slice().len(), len, "len {len}");
            assert_eq!(buf.as_slice().len(), len, "len {len}");
            assert_eq!(
                buf.as_mut_slice().as_ptr() as usize % 16,
                0,
                "len {len} not 16-byte aligned",
            );
            assert!(
                buf.as_slice().iter().all(|c| c.re == 0.0 && c.im == 0.0),
                "len {len} not zeroed",
            );
        }
    }

    /// Writes must survive the reinterpreting slice — i.e. the two
    /// views really do alias one allocation, rather than the mutable
    /// one handing out a temporary.
    #[test]
    fn writes_are_visible_through_the_shared_view() {
        let mut buf = AlignedComplexBuf::zeroed(8);
        buf.as_mut_slice()[5] = Complex32::new(1.5, -2.5);
        assert_eq!(buf.as_slice()[5], Complex32::new(1.5, -2.5));
        assert_eq!(buf.as_slice()[4], Complex32::new(0.0, 0.0));
    }
}

#[cfg(all(test, feature = "fft-rustfft"))]
mod tests_rustfft {
    use super::*;
    use core::f32::consts::TAU;

    #[test]
    fn rustfft_roundtrip_64() {
        let mut planner = RustFftPlanner::new();
        let n = 64;
        let mut buf: alloc::vec::Vec<Complex32> = (0..n)
            .map(|k| Complex32::from_polar(1.0, TAU * 3.0 * k as f32 / n as f32))
            .collect();
        let original = buf.clone();
        let fwd = planner.plan_forward(n);
        let inv = planner.plan_inverse(n);
        fwd.process(&mut buf);
        inv.process(&mut buf);
        let scale = 1.0 / n as f32;
        for c in buf.iter_mut() {
            *c *= scale;
        }
        for (a, b) in buf.iter().zip(original.iter()) {
            assert!((a - b).norm() < 1e-4);
        }
    }

    #[test]
    fn rustfft_forward_picks_correct_bin() {
        let mut planner = RustFftPlanner::new();
        let n = 128;
        let bin = 7;
        let mut buf: alloc::vec::Vec<Complex32> = (0..n)
            .map(|k| Complex32::from_polar(1.0, TAU * bin as f32 * k as f32 / n as f32))
            .collect();
        let fwd = planner.plan_forward(n);
        fwd.process(&mut buf);
        let peak = buf
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.norm().partial_cmp(&b.1.norm()).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak, bin);
    }
}
