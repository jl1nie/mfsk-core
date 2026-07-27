//! Shared `#[repr(C)]` ABI types for `mfsk-ffi` and `mfsk-ffi-ft8`
//! (issue #205).
//!
//! Before this crate, the two FFI crates independently evolved
//! incompatible conventions for the same domain: clashing status
//! vocabularies (`MfskStatus` vs `MfskFt8Status`, both using
//! `-1..-4` for different meanings), duplicate result structs with
//! different text-ownership models (heap `CString` pointer vs inline
//! fixed buffer), and decode entry points that hardcode every tuning
//! knob positionally with no room to grow. This crate is the single
//! source of truth for the status enum, result record/list shape, and
//! an opaque decode-options handle shape both crates now share.
//!
//! `#![no_std]`, zero dependencies: this crate holds plain data
//! definitions only, no allocation logic. Each consuming crate (which
//! already has its own `alloc`/`std` capability) implements the
//! `_new`/`_free` allocation logic for [`MfskDecodeOptions`] itself,
//! casting to/from a private inner struct — the same opaque-handle
//! pattern `mfsk-ffi`'s own `MfskDecoder` already established.
//!
//! This crate is not published and is not a C ABI on its own — it
//! exists purely so `mfsk-ffi` and `mfsk-ffi-ft8` re-emit *identical*
//! type definitions into their independently cbindgen-generated
//! `mfsk.h` / `mfsk_ft8.h` headers. The two headers are not designed
//! to be `#include`d together in the same translation unit (C, unlike
//! C++, does not permit two textually-identical struct/enum
//! redefinitions) — link against whichever single header matches your
//! target (desktop/Kotlin vs embedded).

#![no_std]

use core::ffi::c_char;
use core::marker::PhantomData;

// ──────────────────────────────────────────────────────────────────────────
// Status codes
// ──────────────────────────────────────────────────────────────────────────

/// Outcome of a fallible `mfsk_*` / `mfsk_ft8_*` call.
///
/// Zero is success; negative values are errors. Both crates additionally
/// expose a `_last_error()` function (`mfsk_last_error` /
/// `mfsk_ft8_last_error`) returning a human-readable string for the
/// specific failure reason — the numeric code stays a small, stable
/// set; the string carries the detail (e.g. which buffer was too
/// short, which enum value was out of range).
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum MfskStatus {
    /// Success (zero or more results in the output list).
    Ok = 0,
    /// A required pointer argument was null.
    NullPointer = -1,
    /// A non-null argument was invalid: malformed UTF-8, an
    /// out-of-range enum discriminant, an audio buffer too short for
    /// the protocol's slot length, a caller-provided output buffer
    /// too small, etc. See `_last_error()` for which.
    InvalidArg = -2,
    /// The supplied protocol tag is not recognised or not supported
    /// by this build (e.g. a protocol compiled out via feature flags).
    UnknownProtocol = -3,
    /// The call ran without a fatal error but produced no usable
    /// result — e.g. an encode helper that could not pack the given
    /// callsign/grid/report into the protocol's message payload.
    DecodeFailed = -4,
    /// Internal error: an invariant of the Rust implementation was
    /// violated. Always a bug; please report it.
    Internal = -5,
}

// ──────────────────────────────────────────────────────────────────────────
// Decode depth
// ──────────────────────────────────────────────────────────────────────────

/// Decode cost/recall tradeoff, shared across every protocol both FFI
/// crates expose — mirrors `mfsk_core`'s generic
/// `engine::pipeline::DecodeDepth` (`BP_ONLY` / `FULL`).
///
/// Discriminant `0` is intentionally unassigned — FT8's pre-0.7.0
/// single-metric `Bp` rung was retired (issue #74); existing callers
/// passing `1`/`2` remain valid.
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum MfskDecodeDepth {
    /// Full LLR-variant staircase + BP, no OSD fallback.
    BpAll = 1,
    /// Above + OSD fallback (host-only; a no-op on protocols/builds
    /// without an OSD path).
    BpAllOsd = 2,
}

// ──────────────────────────────────────────────────────────────────────────
// Result record + list
// ──────────────────────────────────────────────────────────────────────────

/// Max UTF-8 bytes (excluding the NUL terminator) in [`MfskResult::text`].
///
/// Every protocol's decoded text runs through the shared 77-/72-/50-bit
/// WSJT message layer, whose longest producible string (the DXpedition
/// `"CALL1 RR73; CALL2 <...> REPORT"` format) stays well under this —
/// see `msg::wsjt77::unpack77`'s doc comment for the token grammar.
pub const MFSK_TEXT_CAP: usize = 39;

/// Size of [`MfskResult::text`] in bytes (`MFSK_TEXT_CAP` + 1 for the
/// NUL terminator) — for Rust-side use (`write_text`/`empty_result`
/// helpers in the consuming crates). **Not** used in the `text` field
/// below: cbindgen's cross-crate handling of a `pub use`-re-exported
/// struct (this one, re-exported by `mfsk-ffi`/`mfsk-ffi-ft8`) can't
/// turn a named `usize` constant defined in this crate into a C
/// `#define` the *consuming* crate's header can reference — it
/// resolves the array length internally (falling back to an opaque,
/// field-less forward declaration if it can't, as a compound
/// `MFSK_TEXT_CAP + 1` expression did during issue #205's header
/// verification) but never emits the constant itself into the
/// generated `mfsk.h`/`mfsk_ft8.h`. The field below therefore uses a
/// bare literal; the `const _` assertion keeps it in sync with this
/// constant at compile time.
pub const MFSK_TEXT_BUF_LEN: usize = MFSK_TEXT_CAP + 1;
const _: () = assert!(MFSK_TEXT_BUF_LEN == 40);

/// One successfully decoded message, shared shape across every
/// protocol both FFI crates expose.
///
/// `text` is a fixed inline buffer (not a heap pointer): the whole
/// [`MfskResultList`] is one allocation, freed in one call, with no
/// per-message ownership to track — the model `mfsk-ffi-ft8` already
/// used, now shared by `mfsk-ffi` too (issue #205; previously
/// `mfsk-ffi`'s `MfskMessage` held a heap `CString` pointer per
/// message instead).
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct MfskResult {
    /// NUL-terminated UTF-8 decoded message text. ASCII in practice.
    ///
    /// Length is [`MFSK_TEXT_BUF_LEN`] (`40`); written as a bare
    /// literal here rather than the constant — see
    /// [`MFSK_TEXT_BUF_LEN`]'s doc comment for why.
    pub text: [c_char; 40],
    /// Carrier (tone-0) frequency in Hz.
    pub freq_hz: f32,
    /// Time offset in seconds from the protocol's nominal frame start.
    pub dt_sec: f32,
    /// WSJT-X-compatible SNR estimate, dB (2500 Hz reference bandwidth).
    pub snr_db: f32,
    /// Hard-decision errors corrected by the FEC (0 if not applicable).
    pub hard_errors: u32,
    /// Decode pass/stage identifier; meaning is protocol-specific.
    pub pass: u8,
    /// Padding to keep the struct's layout stable across compilers.
    pub _pad: [u8; 3],
}

/// List of decoded messages, owned by the FFI side. Free with the
/// crate's `_result_list_free` function.
#[repr(C)]
pub struct MfskResultList {
    /// Pointer to the first result, or null if `len == 0`.
    pub items: *mut MfskResult,
    /// Number of valid entries.
    pub len: usize,
    /// Total allocation length (private — only the free function
    /// needs this; may exceed `len`).
    pub _capacity: usize,
}

impl MfskResultList {
    /// A zero-length list — the value every decode entry point writes
    /// to `*out` before attempting to decode, so a caller who bails
    /// out early on an error status still sees a well-formed
    /// (free-safe, no-op) list rather than uninitialised memory.
    pub const fn empty() -> Self {
        Self {
            items: core::ptr::null_mut(),
            len: 0,
            _capacity: 0,
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Decode options (opaque handle)
// ──────────────────────────────────────────────────────────────────────────

/// Opaque decode-tuning-options handle (issue #205) — construct with
/// each crate's own `_options_new(...)`, release with
/// `_options_free`.
///
/// Both `mfsk-ffi` and `mfsk-ffi-ft8` used to hardcode (or take
/// entirely positionally, with no room to add more later) every
/// decode-tuning knob. Wrapping them behind an opaque handle now means
/// a future knob is a new, optional setter function — the options
/// constructor and every decode function's signature stay stable
/// forever; only additive growth on the setter side.
///
/// Zero-sized marker type + phantom pointer, matching the established
/// `MfskDecoder` opaque-handle shape in `mfsk-ffi` — each consuming
/// crate `Box`es its own private options struct and casts the raw
/// pointer to/from this type. Defined once here purely so both
/// crates' generated headers agree on the type name / pointer shape.
#[repr(C)]
pub struct MfskDecodeOptions {
    _priv: [u8; 0],
    _marker: PhantomData<*mut ()>,
}
