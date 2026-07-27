//! Scalar abstraction for the LLR / BP arithmetic in the fixed-point
//! embedded path. Both `f32` (host / RasPi / FPU-equipped MCUs) and
//! [`Q11i16`] (FPU-less / consistency-focused embedded targets)
//! implement [`LlrScalar`], so [`crate::engine::llr::compute_llr`] and
//! [`crate::fec::ldpc::bp::bp_decode_generic_nms`] can be written
//! once and instantiated for either scalar.
//!
//! Q-format conventions live in `~/.claude/plans/embedded-i16-scalar-design.md`:
//! - `Q11i16` is a Q11.5 fixed-point i16 (range ±16, 1/2048 LSB).
//!   Sized to comfortably hold post-`LLR_SCALE` (≈2.83) LLR values
//!   in ±10 with headroom.
//! - α (NMS scaling) is multiplied as Q15 (`alpha * 32768 → i32`)
//!   and the product right-shifted 15 places.
//! - Wide accumulator (sums during BP variable-node update) is
//!   `f32` for the f32 path and `i32` for the Q11i16 path — chosen
//!   so `llr + 3·tov` never overflows.
//!
//! Subset of operations covered: enough to express the **NMS BP
//! kernel** and the LLR computation. SumProduct BP needs `tanh` /
//! `atanh` and stays f32-only by design.

use core::cmp::Ordering;

/// Scalar trait the LLR/BP NMS implementation uses. `f32` and
/// [`Q11i16`] both implement it.
pub trait LlrScalar: Copy + Default + core::fmt::Debug {
    /// Sum accumulator type. `f32` for `f32`, `i32` for [`Q11i16`].
    type Wide: Copy + Default;

    /// Additive identity.
    const ZERO: Self;
    /// Largest representable value (used as the `min1` / `min2`
    /// initial sentinel in the min-sum check-node update).
    const POS_INF_LIKE: Self;

    /// Convert from f32 with saturation. Used at the LLR pipeline
    /// boundary (final scale-and-round) and at debug paths.
    fn from_f32(x: f32) -> Self;
    /// Convert to f32 (lossless for `f32`, `× 2^-11` for `Q11i16`).
    fn to_f32(self) -> f32;

    /// Saturating negation (`i16::MIN.neg()` clamps to `i16::MAX`).
    fn neg_sat(self) -> Self;
    /// Saturating absolute value.
    fn abs_sat(self) -> Self;

    /// Sign predicate for hard-decision parity check.
    fn is_negative(self) -> bool;

    /// Total-order comparator. NaN-safe for f32 (treats NaN as
    /// equal to itself, equal to all). Used by min-sum's `<` test.
    fn cmp_total(self, other: Self) -> Ordering;
    #[inline]
    fn lt_total(self, other: Self) -> bool {
        matches!(self.cmp_total(other), Ordering::Less)
    }

    /// Multiply by a normalised α (0..1) constant, with saturating
    /// rounding. Bench paths only ever pass `NMS_ALPHA = 0.75`.
    fn mul_alpha(self, alpha: f32) -> Self;

    /// Promote to wide accumulator.
    fn to_wide(self) -> Self::Wide;
    /// Wide identity.
    fn wide_zero() -> Self::Wide;
    /// Wide a + b.
    fn wide_add(a: Self::Wide, b: Self::Wide) -> Self::Wide;
    /// Wide a − b.
    fn wide_sub(a: Self::Wide, b: Self::Wide) -> Self::Wide;
    /// Demote wide → narrow with saturation.
    fn from_wide_sat(w: Self::Wide) -> Self;
    /// Wide sign predicate (avoids round-trip through `Self`).
    fn wide_is_positive(w: Self::Wide) -> bool;
}

impl LlrScalar for f32 {
    type Wide = f32;
    const ZERO: f32 = 0.0;
    const POS_INF_LIKE: f32 = f32::INFINITY;

    #[inline]
    fn from_f32(x: f32) -> Self {
        x
    }
    #[inline]
    fn to_f32(self) -> f32 {
        self
    }
    #[inline]
    fn neg_sat(self) -> Self {
        -self
    }
    #[inline]
    fn abs_sat(self) -> Self {
        // `f32::abs` is no_std-safe via `num_traits::Float` already
        // imported elsewhere; here it's just `self.abs()` (inherent
        // method under std, libm under no_std).
        #[cfg(feature = "std")]
        {
            self.abs()
        }
        #[cfg(not(feature = "std"))]
        {
            use num_traits::Float;
            Float::abs(self)
        }
    }
    #[inline]
    fn is_negative(self) -> bool {
        self < 0.0
    }
    #[inline]
    fn cmp_total(self, other: Self) -> Ordering {
        // `partial_cmp` returns None on NaN; treat NaN as equal so
        // the min-sum loop never panics on noisy LLRs.
        self.partial_cmp(&other).unwrap_or(Ordering::Equal)
    }
    #[inline]
    fn mul_alpha(self, alpha: f32) -> Self {
        self * alpha
    }

    #[inline]
    fn to_wide(self) -> Self::Wide {
        self
    }
    #[inline]
    fn wide_zero() -> Self::Wide {
        0.0
    }
    #[inline]
    fn wide_add(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a + b
    }
    #[inline]
    fn wide_sub(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a - b
    }
    #[inline]
    fn from_wide_sat(w: Self::Wide) -> Self {
        w
    }
    #[inline]
    fn wide_is_positive(w: Self::Wide) -> bool {
        w > 0.0
    }
}

/// LLR Q11 fixed-point: inner i16 = `value × 2^11`. Range ±16,
/// resolution 1/2048.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Q11i16(pub i16);

const Q11_FRAC: u32 = 11;
const Q11_ONE: i32 = 1 << Q11_FRAC; // 2048

impl LlrScalar for Q11i16 {
    type Wide = i32;
    const ZERO: Q11i16 = Q11i16(0);
    /// Min-sum sentinel for "never beat me" — `i16::MAX` represents
    /// the largest finite Q11 magnitude.
    const POS_INF_LIKE: Q11i16 = Q11i16(i16::MAX);

    #[inline]
    fn from_f32(x: f32) -> Self {
        let v = (x * Q11_ONE as f32) as i32;
        Q11i16(v.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
    #[inline]
    fn to_f32(self) -> f32 {
        (self.0 as f32) / (Q11_ONE as f32)
    }
    #[inline]
    fn neg_sat(self) -> Self {
        // i16::MIN.wrapping_neg() == i16::MIN; saturate to i16::MAX
        // so the sign flip is symmetric.
        Q11i16(self.0.checked_neg().unwrap_or(i16::MAX))
    }
    #[inline]
    fn abs_sat(self) -> Self {
        Q11i16(self.0.saturating_abs())
    }
    #[inline]
    fn is_negative(self) -> bool {
        self.0 < 0
    }
    #[inline]
    fn cmp_total(self, other: Self) -> Ordering {
        self.0.cmp(&other.0)
    }
    #[inline]
    fn mul_alpha(self, alpha: f32) -> Self {
        let aq15 = (alpha * 32768.0) as i32;
        let prod = (self.0 as i32) * aq15;
        // Arithmetic shift — preserves sign of the input.
        let v = prod >> 15;
        Q11i16(v.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }

    #[inline]
    fn to_wide(self) -> Self::Wide {
        self.0 as i32
    }
    #[inline]
    fn wide_zero() -> Self::Wide {
        0
    }
    #[inline]
    fn wide_add(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a.saturating_add(b)
    }
    #[inline]
    fn wide_sub(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a.saturating_sub(b)
    }
    #[inline]
    fn from_wide_sat(w: Self::Wide) -> Self {
        Q11i16(w.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
    #[inline]
    fn wide_is_positive(w: Self::Wide) -> bool {
        w > 0
    }
}

/// LLR Q3 fixed-point: inner i8 = `value × 2^3`. Range ±16, resolution
/// 1/8. Halves BP scratch memory vs [`Q11i16`] at the cost of coarser
/// LLR quantisation; recall in the FT8 operating SNR band stays within
/// 2 % of the f32 / Q11 path empirically (Karlis Goba's `ft8_lib` uses
/// `int8_t log174[174]` as precedent). Wide accumulator is `i16` —
/// FT8's max variable-node degree (~3) puts the sum well below i16
/// range even at saturated input (4 × 127 = 508).
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Q3i8(pub i8);

const Q3_FRAC: u32 = 3;
const Q3_ONE: i32 = 1 << Q3_FRAC; // 8

impl LlrScalar for Q3i8 {
    type Wide = i16;
    const ZERO: Q3i8 = Q3i8(0);
    const POS_INF_LIKE: Q3i8 = Q3i8(i8::MAX);

    #[inline]
    fn from_f32(x: f32) -> Self {
        let v = (x * Q3_ONE as f32) as i32;
        Q3i8(v.clamp(i8::MIN as i32, i8::MAX as i32) as i8)
    }
    #[inline]
    fn to_f32(self) -> f32 {
        (self.0 as f32) / (Q3_ONE as f32)
    }
    #[inline]
    fn neg_sat(self) -> Self {
        Q3i8(self.0.checked_neg().unwrap_or(i8::MAX))
    }
    #[inline]
    fn abs_sat(self) -> Self {
        Q3i8(self.0.saturating_abs())
    }
    #[inline]
    fn is_negative(self) -> bool {
        self.0 < 0
    }
    #[inline]
    fn cmp_total(self, other: Self) -> Ordering {
        self.0.cmp(&other.0)
    }
    #[inline]
    fn mul_alpha(self, alpha: f32) -> Self {
        let aq15 = (alpha * 32768.0) as i32;
        let prod = (self.0 as i32) * aq15;
        let v = prod >> 15;
        Q3i8(v.clamp(i8::MIN as i32, i8::MAX as i32) as i8)
    }

    #[inline]
    fn to_wide(self) -> Self::Wide {
        self.0 as i16
    }
    #[inline]
    fn wide_zero() -> Self::Wide {
        0
    }
    #[inline]
    fn wide_add(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a.saturating_add(b)
    }
    #[inline]
    fn wide_sub(a: Self::Wide, b: Self::Wide) -> Self::Wide {
        a.saturating_sub(b)
    }
    #[inline]
    fn from_wide_sat(w: Self::Wide) -> Self {
        Q3i8(w.clamp(i8::MIN as i16, i8::MAX as i16) as i8)
    }
    #[inline]
    fn wide_is_positive(w: Self::Wide) -> bool {
        w > 0
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Spec scalar — for cs (complex symbol spectra) entries
// ──────────────────────────────────────────────────────────────────────────

/// Scalar trait for complex symbol-spectra (cs) entries: `f32` and
/// [`Q14i16`] both implement it. Separated from [`LlrScalar`] because
/// the Q-format (Q14 vs Q11) and the operations needed (norm² for
/// sync_quality / LLR, conversion to f32 for SNR estimation) differ.
///
/// The DFT (`fill_symbol_spectra_generic` on the fixed-point path)
/// natively produces i16 Q15-ish output; `Q14i16` is one bit
/// narrower to give the squared sum (norm²) a 1-bit headroom in
/// i32 even when both re and im saturate.
pub trait SpecScalar: Copy + Default + core::fmt::Debug {
    /// Wide accumulator for `re² + im²` and other squared sums —
    /// `f32` for `f32`, `i32` for [`Q14i16`].
    type Wide: Copy + Default + core::fmt::Debug;

    /// Lossless promotion to f32 (`× 2^-14` for [`Q14i16`]).
    fn to_f32(self) -> f32;
    /// Saturating cast from f32 (rounds in the natural direction).
    fn from_f32(x: f32) -> Self;
    /// Saturating cast from f32 with a pre-applied scale factor.
    /// `f32` ignores `scale` (no-op for the host path); fixed-point
    /// types compute `(x * scale)` and saturate to their range.
    /// Used by `fill_symbol_spectra` to apply a per-cs auto-gain.
    fn from_f32_scaled(x: f32, scale: f32) -> Self;
    /// Whether this scalar requires a peak-scan auto-gain pass before
    /// writing. `f32` returns `false` (one-pass, bit-identical to
    /// the pre-Phase-2.6 implementation); fixed-point types return
    /// `true` so `fill_symbol_spectra_generic` knows to scan and
    /// scale.
    const NEEDS_AUTOGAIN: bool;

    /// `re² + im²` in the wide type. For `f32` this is just
    /// `re*re + im*im`; for `Q14i16` it's `(re as i32)² + (im as i32)²`.
    fn norm_sqr_wide(re: Self, im: Self) -> Self::Wide;
    /// Wide → f32 (used by SNR / debug paths that accept some
    /// precision loss in exchange for downstream f32 maths).
    fn wide_to_f32(w: Self::Wide) -> f32;
}

impl SpecScalar for f32 {
    type Wide = f32;
    const NEEDS_AUTOGAIN: bool = false;
    #[inline]
    fn to_f32(self) -> f32 {
        self
    }
    #[inline]
    fn from_f32(x: f32) -> Self {
        x
    }
    #[inline]
    fn from_f32_scaled(x: f32, _scale: f32) -> Self {
        // f32 path: scale is ignored (the auto-gain dispatch via
        // `NEEDS_AUTOGAIN = false` skips the scan entirely).
        x
    }
    #[inline]
    fn norm_sqr_wide(re: Self, im: Self) -> Self::Wide {
        re * re + im * im
    }
    #[inline]
    fn wide_to_f32(w: Self::Wide) -> f32 {
        w
    }
}

/// Complex spectrum value Q14: inner i16 = `value × 2^14`, range
/// ±2 at 1/16384 resolution. The DFT inner loop on Core2 outputs
/// roughly i16 Q15; one bit headroom (Q14) keeps `re² + im²` in
/// i32 even when both components saturate.
#[derive(Copy, Clone, Default, Debug, PartialEq, Eq)]
pub struct Q14i16(pub i16);

const Q14_FRAC: u32 = 14;
const Q14_ONE: i32 = 1 << Q14_FRAC; // 16384

impl SpecScalar for Q14i16 {
    type Wide = i32;
    const NEEDS_AUTOGAIN: bool = true;
    #[inline]
    fn to_f32(self) -> f32 {
        (self.0 as f32) / (Q14_ONE as f32)
    }
    #[inline]
    fn from_f32(x: f32) -> Self {
        let v = (x * Q14_ONE as f32) as i32;
        Q14i16(v.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
    #[inline]
    fn from_f32_scaled(x: f32, scale: f32) -> Self {
        // Apply caller-supplied auto-gain scale; saturate to i16.
        // The `scale` brings raw cs magnitudes (typically 1e4-1e8)
        // into ±i16 range so relative magnitudes are preserved.
        let v = (x * scale) as i32;
        Q14i16(v.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
    }
    #[inline]
    fn norm_sqr_wide(re: Self, im: Self) -> Self::Wide {
        let r = re.0 as i32;
        let i = im.0 as i32;
        r * r + i * i
    }
    #[inline]
    fn wide_to_f32(w: Self::Wide) -> f32 {
        // The wide is `re*re + im*im` in raw Q14² units (i.e.
        // (value × 2^14)² = value² × 2^28). Convert back by dividing
        // by 2^28 to get the true |z|².
        (w as f32) / ((Q14_ONE as f32) * (Q14_ONE as f32))
    }
}

/// Complex sample with both components in scalar type `S`.
///
/// Alias for `num_complex::Complex<S>` — the two types are
/// structurally identical (`#[repr(C)] { re: S, im: S }`) and pre-0.7
/// the codebase kept them as separate types with hand-written
/// layout-compat casts. Collapsing to an alias eliminated five
/// `unsafe` casts plus the five wrapper functions; `SpecScalar`-bound
/// helper methods that were previously inherent now live on the
/// [`ComplexSpec`] extension trait below.
///
/// ## Naming convention
///
/// `Cmplx<S>` and `num_complex::Complex<S>` are the same type via
/// this alias, but the two names are used to signal intent:
///
/// - **`Cmplx<S>`** for mfsk-core's spec-scalar storage —
///   per-symbol spectra (cs), LLR inputs, sync-quality buffers.
///   The `S: SpecScalar` bound is enforced by the [`ComplexSpec`]
///   trait's `impl` block, so methods like `norm_sqr_wide`
///   only resolve when the scalar choice is one mfsk-core
///   understands (`f32`, `Q14i16`, `Q11i16`).
///
/// - **`Complex<f32>`** (directly from `num_complex`) for vanilla
///   DSP buffers — FFT input/output, the 192 k cd0 cache,
///   per-symbol rotators, anything that exists only as raw signal
///   processing scratch. These have no `SpecScalar` semantics
///   and stay agnostic to mfsk-core's storage typing.
pub type Cmplx<S> = num_complex::Complex<S>;

/// `SpecScalar`-aware extension methods on [`Cmplx`] / `Complex<S>`.
///
/// Lives on a trait (rather than inherent on `Cmplx`) because `Cmplx`
/// is an alias for an upstream type — `impl` blocks on aliases aren't
/// allowed. Import this trait wherever `.norm_sqr_wide()` /
/// `.norm_sqr_f32()` / `.norm_f32()` is called on a `Cmplx<S>`.
pub trait ComplexSpec<S: SpecScalar> {
    /// `|z|²` in the wide accumulator type. For `f32` this returns
    /// `f32`; for `Q14i16` it returns `i32` raw — divide by `2^28`
    /// to recover the f32 magnitude squared (see
    /// [`SpecScalar::wide_to_f32`]).
    fn norm_sqr_wide(self) -> S::Wide;

    /// `|z|²` as `f32` (lossy for `Q14i16`, but the SNR / sync_quality
    /// paths only need an ordering-correct float).
    fn norm_sqr_f32(self) -> f32;

    /// `|z|` as `f32`. Convenience for places that already use
    /// `Complex::norm()`.
    fn norm_f32(self) -> f32;
}

impl<S: SpecScalar> ComplexSpec<S> for Cmplx<S> {
    #[inline]
    fn norm_sqr_wide(self) -> S::Wide {
        S::norm_sqr_wide(self.re, self.im)
    }

    #[inline]
    fn norm_sqr_f32(self) -> f32 {
        S::wide_to_f32(self.norm_sqr_wide())
    }

    #[inline]
    fn norm_f32(self) -> f32 {
        // Compute via the wide product so f32 quantisation noise on
        // small magnitudes doesn't compound.
        let n2 = self.norm_sqr_f32();
        #[cfg(feature = "std")]
        {
            n2.sqrt()
        }
        #[cfg(not(feature = "std"))]
        {
            use num_traits::Float;
            n2.sqrt()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn q11_round_trip() {
        for f in [-10.0, -1.5, -0.001, 0.0, 0.001, 1.5, 10.0] {
            let q = Q11i16::from_f32(f);
            let back = q.to_f32();
            assert!(
                (f - back).abs() < 1.0 / Q11_ONE as f32 + 1e-6,
                "f={f} back={back}"
            );
        }
    }

    #[test]
    fn q11_saturation() {
        // Way above range → i16::MAX
        assert_eq!(Q11i16::from_f32(1e6).0, i16::MAX);
        assert_eq!(Q11i16::from_f32(-1e6).0, i16::MIN);
    }

    #[test]
    fn q11_mul_alpha() {
        let q = Q11i16::from_f32(8.0);
        let scaled = q.mul_alpha(0.75);
        // 8.0 × 0.75 = 6.0 ± 1 LSB
        let f = scaled.to_f32();
        assert!((f - 6.0).abs() < 0.01, "f={f}");
    }

    #[test]
    fn q11_neg_handles_min() {
        // i16::MIN cannot be negated in two's complement; saturate.
        let q = Q11i16(i16::MIN);
        assert_eq!(q.neg_sat().0, i16::MAX);
    }

    #[test]
    fn q3i8_round_trip() {
        // Q3 LSB = 1/8 = 0.125. Use values comfortably inside ±16.
        for f in [-15.0, -1.5, -0.125, 0.0, 0.125, 1.5, 15.0] {
            let q = Q3i8::from_f32(f);
            let back = q.to_f32();
            assert!(
                (f - back).abs() < 1.0 / Q3_ONE as f32 + 1e-6,
                "f={f} back={back}"
            );
        }
    }

    #[test]
    fn q3i8_saturation() {
        assert_eq!(Q3i8::from_f32(1e6).0, i8::MAX);
        assert_eq!(Q3i8::from_f32(-1e6).0, i8::MIN);
    }

    #[test]
    fn q3i8_mul_alpha() {
        let q = Q3i8::from_f32(8.0);
        let scaled = q.mul_alpha(0.75);
        // 8.0 × 0.75 = 6.0 ± 1 LSB (0.125)
        let f = scaled.to_f32();
        assert!((f - 6.0).abs() < 0.2, "f={f}");
    }

    #[test]
    fn q3i8_neg_handles_min() {
        // i8::MIN cannot be negated in two's complement; saturate.
        let q = Q3i8(i8::MIN);
        assert_eq!(q.neg_sat().0, i8::MAX);
    }

    #[test]
    fn f32_mul_alpha_unchanged() {
        assert!((8.0_f32.mul_alpha(0.75) - 6.0).abs() < 1e-6);
    }

    #[test]
    fn q14_round_trip() {
        for f in [-1.5, -0.001, 0.0, 0.001, 1.5] {
            let q = Q14i16::from_f32(f);
            let back = q.to_f32();
            assert!(
                (f - back).abs() < 1.0 / Q14_ONE as f32 + 1e-6,
                "f={f} back={back}"
            );
        }
    }

    #[test]
    fn cmplx_q14_norm_matches_f32() {
        let cf = Cmplx::<f32>::new(0.6, 0.8); // |z|² = 1.0
        let cq = Cmplx::<Q14i16>::new(Q14i16::from_f32(0.6), Q14i16::from_f32(0.8));
        assert!((cf.norm_sqr_f32() - 1.0).abs() < 1e-6);
        assert!((cq.norm_sqr_f32() - 1.0).abs() < 1e-3);
    }

    #[test]
    fn cmplx_layout_compat_with_num_complex() {
        // sanity: Cmplx<f32> must be layout-compatible with
        // num_complex::Complex<f32> for the future zero-copy bridge.
        // `#[repr(C)]` on both + same field order does the trick.
        assert_eq!(
            core::mem::size_of::<Cmplx<f32>>(),
            core::mem::size_of::<num_complex::Complex<f32>>()
        );
        assert_eq!(
            core::mem::align_of::<Cmplx<f32>>(),
            core::mem::align_of::<num_complex::Complex<f32>>()
        );
    }
}
