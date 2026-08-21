//! esp-dsp backend for `mfsk_core::engine::dsp::dotprod` — issue #307.
//!
//! Satisfies the `dotprod-extern` hook with `dsps_dotprod_f32_aes3`,
//! the LX7 PIE (4-wide f32) dot product. Same extern-hook shape
//! `esp_dsp_fft` uses for `mfsk_core_make_default_fft_planner`, and the
//! same hand-written `extern "C"` declaration approach — esp-idf-sys's
//! bindgen doesn't cover esp-dsp's headers, so we declare the one
//! symbol we need.
//!
//! ## Why `dsps_dotprod_f32_aes3` and not `dsps_fird_f32_aes3`
//!
//! `docs/notes/FST4_DDC_DESIGN.md` §4.5 planned a whole-FIR-stage
//! replacement, because esp-dsp's `fir_f32_t` owns a delay line and
//! would duplicate `FirStage`'s own state. `dsps_dotprod_f32` is
//! stateless, so that objection doesn't apply, and the narrower seam
//! reaches somewhere the stage-level one could not:
//! `PolyphaseResampler`'s per-output tap-phase rotation is not integer
//! decimation and `dsps_fird_f32` cannot express it — and the wideband
//! FST4 cascade is *entirely* polyphase, with no integer stages at all.
//!
//! Note also that `dsps_dotprode_f32` (the strided variant, which would
//! have suited interleaved complex data) ships **no `_aes3_` version**
//! — only ansi/ae32/arp4. Planar-with-contiguous-taps is the layout
//! that gets PIE on this chip, which is exactly what `FirStage` and
//! `PolyphaseResampler` already use.
//!
//! ## Alignment
//!
//! `dsps_dotprod_f32_aes3`'s own assembly checks `len % 4 == 0` **and**
//! 16-byte alignment on both pointers, and branches to its scalar body
//! when either fails. So this backend is safe to call on any slice
//! pair — unlike `esp_dsp_fft`, where misalignment corrupted a TLSF
//! block header. Measured on real CoreS3 (`dotprod-bench`), against the
//! portable Rust loop, 20 000 iterations:
//!
//! | depth | portable | aes3 misaligned | aes3 aligned |
//! |---|---:|---:|---:|
//! | 107 | 6733 ns | 1985 ns (3.4x) | 905 ns (7.4x) |
//! | 288 | 18052 ns | 4986 ns (3.6x) | 2031 ns (8.9x) |
//! | 422 | 26432 ns | 7255 ns (3.6x) | 2881 ns (9.2x) |
//!
//! The misaligned column is the one that applies today:
//! `PolyphaseResampler`'s window start advances by a Bresenham carry,
//! so the history pointer lands on an arbitrary 4-byte offset.
//! Capturing the remaining ~2.5x needs aligned history buffers and a
//! tap depth padded to a multiple of 4, which is a separate change —
//! deliberately not bundled here, so the cheap win lands first and the
//! alignment work can be measured on its own.

/// Below this length the FFI hop costs more than it saves; the portable
/// loop wins. `dotprod-bench`'s shortest measured depth is 107, where
/// the backend is already ~3.4x ahead, so the exact crossover is well
/// below anything this crate's DSP actually uses — this guard exists
/// for correctness of the general contract, not as a tuned threshold.
const MIN_FFI_LEN: usize = 16;

unsafe extern "C" {
    /// `*dest = Σ src1[i]·src2[i]`, `i ∈ [0, len)`.
    ///
    /// Falls back internally to the scalar `ae32` body unless
    /// `len % 4 == 0` and both pointers are 16-byte aligned — see the
    /// module doc comment.
    fn dsps_dotprod_f32_aes3(src1: *const f32, src2: *const f32, dest: *mut f32, len: i32) -> i32;
}

/// Backend for `mfsk_core::engine::dsp::dotprod::dot_f32`.
///
/// The linker binds this to the `dotprod-extern` hook; exactly one
/// definition may exist in the dependency closure.
#[unsafe(no_mangle)]
pub extern "Rust" fn mfsk_core_dotprod_f32(a: &[f32], b: &[f32]) -> f32 {
    let n = a.len().min(b.len());
    if n < MIN_FFI_LEN {
        return mfsk_core::engine::dsp::dotprod::dot_f32_portable(&a[..n], &b[..n]);
    }
    let mut out = 0.0f32;
    // SAFETY: both pointers are valid for `n` f32 reads (taken from
    // slices, length clamped to the shorter), `out` is a valid
    // single-f32 destination, and the callee only reads/writes within
    // those bounds. It imposes no alignment precondition — misalignment
    // selects its scalar path rather than misbehaving.
    unsafe {
        dsps_dotprod_f32_aes3(a.as_ptr(), b.as_ptr(), &mut out, n as i32);
    }
    out
}
