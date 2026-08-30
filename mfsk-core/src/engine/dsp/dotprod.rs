// SPDX-License-Identifier: GPL-3.0-or-later
//! Real f32 dot product, with an optional caller-provided backend.
//!
//! The DSP hot loops in this crate reduce to `Σ a[k]·b[k]` over a
//! contiguous pair of f32 slices —
//! [`PolyphaseResampler::dot`](super::polyphase::PolyphaseResampler)
//! and [`FirStage::dot`](super::fir_decimate::FirStage) both literally,
//! and `engine::sync2d`'s coherent correlator once its complex product
//! is expanded per channel. On a host that loop is fine. On Xtensa it
//! is the single largest cost in the FST4 embedded DDC pipeline, and
//! the portable form leaves most of the chip on the table.
//!
//! Measured on real CoreS3 hardware (issue #307,
//! `embedded-poc/m5stack-cores3-app/src/bin/dotprod_bench.rs`,
//! 20 000 iterations per depth):
//!
//! | depth | this loop | esp-dsp `ae32` | esp-dsp `aes3` (aligned) |
//! |---|---:|---:|---:|
//! | 107 | 6733 ns | 1918 ns | 905 ns |
//! | 288 | 18052 ns | 4937 ns | 2031 ns |
//! | 422 | 26432 ns | 7171 ns | 2881 ns |
//!
//! i.e. ~15 cycles/MAC portable versus ~4.1 (`ae32`) and ~1.6 (`aes3`
//! 4-wide PIE). Hence this seam.
//!
//! ## Why a dot-product hook rather than a whole-stage one
//!
//! `docs/notes/FST4_DDC_DESIGN.md` §4.5 originally proposed replacing
//! an entire `FirDecimator` stage, on the reasoning that esp-dsp's
//! `fir_f32_t` owns its own delay line and position state, so swapping
//! only the inner product would leave two state machines fighting.
//! That is true of `dsps_fird_f32`, but it does not apply to
//! `dsps_dotprod_f32`, which is stateless — it takes two pointers and a
//! length. The narrower seam keeps every existing history-buffer,
//! phase-carry and compaction invariant in Rust, where the unit tests
//! already cover them, and lets the backend be a pure function.
//!
//! It also serves the rational resampler, which the stage-level hook
//! could not: `dsps_fird_f32` decimates by an integer factor and cannot
//! express `PolyphaseResampler`'s per-output tap-phase rotation. The
//! wideband FST4 cascade has *no* integer stages at all, so the
//! stage-level hook would have accelerated none of it.
//!
//! ## Alignment is not a correctness concern
//!
//! `dsps_dotprod_f32_aes3` checks `len % 4 == 0` and 16-byte alignment
//! on both pointers and branches to its scalar body when either fails
//! (verified in that function's own assembly, and measured: the
//! misaligned path lands at `ae32` speed, still ~3.6x this loop). So a
//! backend may be handed any slice pair; unaligned input costs
//! throughput, never correctness. This is materially different from
//! `engine::fft`'s ESP-DSP backend, where misalignment corrupted an
//! allocator block header.

extern crate alloc;
use alloc::vec;
use alloc::vec::Vec;

/// Four `f32` under a 16-byte alignment guarantee — the backing unit
/// for [`AlignedF32`].
#[repr(align(16))]
#[derive(Clone, Copy)]
pub(crate) struct AlignedQuad(
    // Read only through the reinterpreting slices in
    // [`AlignedF32::as_slice`] / [`AlignedF32::as_mut_slice`] — the
    // field exists to size and align the backing store. Same shape and
    // same `allow` as `embedded-shared`'s `Align16Quad`.
    #[allow(dead_code)] [f32; 4],
);

/// A zero-filled `f32` buffer whose base is 16-byte aligned and whose
/// length is a multiple of four.
///
/// Both are preconditions of `dsps_dotprod_f32_aes3`'s PIE path;
/// missing either drops it to a scalar loop. Measured on a CoreS3
/// (`docs/notes/FT4_BENCHMARK.md` §27): **7 900 ps/tap when both hold,
/// 18 080 when either does not** — 2.3x, and `ft4::ddc`'s 199- and
/// 263-tap stages satisfied *neither*, so every dot the DDC has ever
/// done took the slow path.
///
/// Lives here rather than with any one caller because it exists
/// purely to satisfy [`dot_f32`]'s backend preconditions;
/// `fir_decimate` and `sync2d` both build on it.
///
/// The FIR's history buffers use it unconditionally: a `Vec<f32>` is
/// only guaranteed 4-byte aligned, so a window starting at a
/// four-multiple index would still not be 16-byte aligned and the
/// backend would take the slow path anyway. Making the base aligned
/// costs nothing and changes no arithmetic.
pub(crate) struct AlignedF32 {
    quads: Vec<AlignedQuad>,
    len: usize,
}

impl AlignedF32 {
    /// Rounds `len` **up** to a multiple of four and zero-fills.
    pub(crate) fn new(len: usize) -> Self {
        let quads = len.div_ceil(4);
        Self {
            quads: vec![AlignedQuad([0.0; 4]); quads],
            len: quads * 4,
        }
    }

    pub(crate) fn as_slice(&self) -> &[f32] {
        // SAFETY: `AlignedQuad` is `repr(align(16))` over `[f32; 4]`,
        // so the store is exactly `self.len` contiguous `f32`.
        unsafe { core::slice::from_raw_parts(self.quads.as_ptr() as *const f32, self.len) }
    }

    pub(crate) fn as_mut_slice(&mut self) -> &mut [f32] {
        // SAFETY: as `as_slice`.
        unsafe { core::slice::from_raw_parts_mut(self.quads.as_mut_ptr() as *mut f32, self.len) }
    }
}

/// Backend hook: `Σ a[k]·b[k]`, where `a.len() == b.len()`.
///
/// With `dotprod-extern`, the binary supplies:
///
/// ```ignore
/// #[unsafe(no_mangle)]
/// pub extern "Rust" fn mfsk_core_dotprod_f32(a: &[f32], b: &[f32]) -> f32 { .. }
/// ```
///
/// Slices, not raw pointers, so the hook carries its own length and the
/// caller cannot pass a mismatched pair — the same shape
/// `mfsk_core_make_default_fft_planner` uses for its own factory.
#[inline]
pub fn dot_f32(a: &[f32], b: &[f32]) -> f32 {
    debug_assert_eq!(a.len(), b.len(), "dot_f32 operands must be equal length");

    #[cfg(feature = "dotprod-extern")]
    {
        unsafe extern "Rust" {
            fn mfsk_core_dotprod_f32(a: &[f32], b: &[f32]) -> f32;
        }
        // SAFETY: the linker enforces that exactly one binary in the
        // dependency closure defines this symbol; a missing symbol
        // fails the link. The contract is a pure function of two
        // equal-length slices.
        unsafe { mfsk_core_dotprod_f32(a, b) }
    }
    #[cfg(not(feature = "dotprod-extern"))]
    {
        dot_f32_portable(a, b)
    }
}

/// The portable reference implementation, and the fallback when no
/// backend is configured. Kept public so a backend can defer to it for
/// lengths too short to be worth an FFI hop, and so tests can compare
/// against it directly.
///
/// Four partial sums rather than one: the accumulation is a dependent
/// chain and Xtensa's `fadd` has ~3-4 cycle latency against 1-cycle
/// throughput, the same reasoning
/// [`FirStage::dot`](super::fir_decimate::FirStage) documents.
#[inline]
pub fn dot_f32_portable(a: &[f32], b: &[f32]) -> f32 {
    let n = a.len().min(b.len());
    let (a, b) = (&a[..n], &b[..n]);
    let mut acc = [0.0f32; 4];
    let chunks = n / 4;
    for c in 0..chunks {
        let i = c * 4;
        for l in 0..4 {
            acc[l] += a[i + l] * b[i + l];
        }
    }
    let mut s = acc[0] + acc[1] + acc[2] + acc[3];
    for k in chunks * 4..n {
        s += a[k] * b[k];
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn portable_matches_naive_within_tolerance() {
        // Summation order differs (four partials vs one chain), so this
        // is a tolerance check, not bit-exactness.
        for n in [0usize, 1, 3, 4, 7, 107, 288, 422] {
            let a: Vec<f32> = (0..n).map(|i| (i as f32 * 0.37).sin()).collect();
            let b: Vec<f32> = (0..n).map(|i| (i as f32 * 0.71).cos()).collect();
            let naive: f32 = a.iter().zip(b.iter()).map(|(x, y)| x * y).sum();
            let got = dot_f32_portable(&a, &b);
            assert!(
                (naive - got).abs() <= 1e-4 * naive.abs().max(1.0),
                "n={n}: naive={naive} got={got}"
            );
        }
    }

    #[test]
    fn dot_f32_agrees_with_portable() {
        // Without `dotprod-extern` these are the same function; with it
        // this pins the backend against the reference.
        let a: Vec<f32> = (0..422).map(|i| (i as f32 * 0.13).sin()).collect();
        let b: Vec<f32> = (0..422).map(|i| (i as f32 * 0.29).cos()).collect();
        let want = dot_f32_portable(&a, &b);
        let got = dot_f32(&a, &b);
        assert!((want - got).abs() <= 1e-4 * want.abs().max(1.0));
    }
}
