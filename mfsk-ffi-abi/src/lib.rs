//! Shared `#[repr(C)]` ABI types for `mfsk-ffi` (issue #205).
//!
//! Two FFI crates used to re-emit these; `mfsk-ffi-ft8` was retired
//! once the v2 ABI covered its streaming and TX pipelines for every
//! mode rather than FT8 alone. The split is kept because the types are
//! plain data with no `std` requirement, which is what lets a future
//! `no_std` C ABI reuse them without reopening this question.
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
//! existed so both FFI crates re-emitted *identical*
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
    /// The mode exists in this build but does not offer what was asked
    /// for — distinct from [`Self::UnknownProtocol`], which means the
    /// mode is not here at all.
    ///
    /// Added for the v2 introspection surface. Existing discriminants
    /// are unchanged, so this is additive: a caller switching on the
    /// values it knows falls through to its default case.
    Unsupported = -6,
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
    /// Whatever the mode publishes as its default.
    ///
    /// Discriminant 0 used to be deliberately unassigned, which made a
    /// `memset`-to-zero options struct carry an invalid discriminant —
    /// harmless as a C int, undefined the moment Rust reads it as an
    /// enum. Giving 0 a meaning removes that edge and makes the
    /// obvious C idiom mean the obvious thing.
    ModeDefault = 0,
    /// Full LLR-variant staircase + BP, no OSD fallback.
    BpAll = 1,
    /// Above + OSD fallback (host-only; a no-op on protocols/builds
    /// without an OSD path).
    BpAllOsd = 2,
}

// ──────────────────────────────────────────────────────────────────────────
// Strictness / equalisation (FFI builder-parity pass, issue #162 follow-up)
// ──────────────────────────────────────────────────────────────────────────

/// Accept/reject threshold profile, mirrors `mfsk_core`'s
/// `engine::pipeline::DecodeStrictness`. Applies to FT8/FT4/FST4-60A;
/// ignored (accepted but unused) for protocols with no tunable
/// threshold, same convention as [`MfskDecodeDepth`].
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Default)]
pub enum MfskStrictness {
    /// Tightest acceptance thresholds, fewest false-accepts.
    Strict = 0,
    /// Default — WSJT-X's own ceiling for FT8; independently-tuned
    /// values for FT4/FST4.
    #[default]
    Normal = 1,
    /// Loosest; deliberately exceeds WSJT-X's own FT8 ceiling
    /// (mfsk-core-original extension, exploratory).
    Deep = 2,
}

/// Equalisation mode, mirrors `mfsk_core`'s `engine::equalize::EqMode`.
/// Applies to FT8/FT4/FST4-60A; ignored elsewhere.
#[repr(C)]
#[derive(Debug, Copy, Clone, Eq, PartialEq, Default)]
pub enum MfskEqMode {
    /// No equalisation (passthrough).
    #[default]
    Off = 0,
    /// Per-signal equalisation using local Costas pilot tones.
    Local = 1,
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
/// struct (this one, re-exported by `mfsk-ffi`) can't
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
/// per-message ownership to track — the model `mfsk-ffi-ft8` had
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
/// Both FFI crates used to hardcode (or take
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
/// Emitted as an incomplete type (`struct X;`) rather than a struct with a
/// zero-length array member: `uint8_t _priv[0]` is a GCC/Clang extension
/// that ISO C rejects (`-Werror=pedantic`), and MSVC accepts only under a
/// warning. A pointer to an incomplete type is exactly as opaque, is
/// standard in both C and C++, and is what every consumer already treats
/// this as. Binary-compatible: the handle only ever crosses as a pointer.
pub struct MfskDecodeOptions {
    _marker: PhantomData<*mut ()>,
}

// ──────────────────────────────────────────────────────────────────────────
// Mode addressing and introspection (FFI v2 slice 1)
// ──────────────────────────────────────────────────────────────────────────

/// Every mode this library knows how to address, one per
/// `mfsk_core::registry::PROTOCOLS` entry plus MSK144.
///
/// **The discriminants are ABI and are never reordered or reused.**
/// They are deliberately *not* registry indices: registry membership is
/// feature-gated, so a build without `q65` shifts every index after it
/// while these numbers stay put. Ask `mfsk_mode_count` /
/// `mfsk_mode_at` which of them this particular build actually has.
///
/// The lesson is one this ABI already learned once — `MfskQ65SubMode`
/// carries explicit discriminants for exactly this reason — and the
/// cost of relearning it is silent misdispatch at a C boundary, so the
/// full list is assigned here in one go, including modes that are not
/// wired yet.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MfskMode {
    /// FT8 — 15 s slot, 8-GFSK, LDPC(174,91).
    Ft8 = 0,
    /// FT4 — 7.5 s slot, 4-GFSK, LDPC(174,91).
    Ft4 = 1,
    /// FST4-15 — 15 s period. The one FST4 sub-mode that starts 0.5 s
    /// into the slot rather than 1.0 s.
    Fst4s15 = 2,
    /// FST4-30 — 30 s period.
    Fst4s30 = 3,
    /// FST4-60A — 60 s period. The only FST4 sub-mode the pre-v2 ABI
    /// could reach.
    Fst4s60 = 4,
    /// FST4-120 — 120 s period.
    Fst4s120 = 5,
    /// FST4-300 — 300 s period. Its slot transform is 4 194 304 points;
    /// see `MfskModeInfo::decode_fft1_size` before budgeting for it.
    Fst4s300 = 6,
    /// WSPR — 120 s slot, 4-FSK, convolutional r=½ K=32 + Fano.
    Wspr = 7,
    /// JT9 — 60 s slot, 9-FSK.
    Jt9 = 8,
    /// JT65 — 60 s slot, 65-FSK, Reed-Solomon(63,12).
    Jt65 = 9,
    /// Q65-15A.
    Q65a15 = 10,
    /// Q65-30A.
    Q65a30 = 11,
    /// Q65-60A.
    Q65a60 = 12,
    /// Q65-60B.
    Q65b60 = 13,
    /// Q65-60C.
    Q65c60 = 14,
    /// Q65-60D.
    Q65d60 = 15,
    /// Q65-60E.
    Q65e60 = 16,
    /// Q65-120D.
    Q65d120 = 17,
    /// Q65-120E.
    Q65e120 = 18,
    /// Q65-300A.
    Q65a300 = 19,
    /// MSK144 — addressed here for completeness and dispatched
    /// specially. It is not FSK, has no `Protocol` marker type and no
    /// registry entry, so `mfsk_mode_info` reports what is knowable and
    /// its capability word is narrow. That is an architectural fact
    /// about MSK144, not a gap to be closed.
    Msk144 = 20,
    /// uvpacket, robust profile — experimental, not a WSJT mode.
    UvRobust = 21,
    /// uvpacket, standard profile.
    UvStandard = 22,
    /// uvpacket, ultra-robust profile.
    UvUltraRobust = 23,
    /// uvpacket, express profile.
    UvExpress = 24,
}

/// How a mode's `sync_min` is measured — the trap this table exists to
/// defuse.
#[repr(C)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum MfskSyncScale {
    /// Absolute Costas correlation score. Noise has no fixed value, so
    /// a threshold is empirical. FT8, FST4.
    CostasAbsolute = 0,
    /// The spectrum is divided by a fitted baseline before scoring, so
    /// **noise sits at ~1.0 by construction** and any threshold at or
    /// below that admits every peak in the band. FT4 only — and it is
    /// why WSJT-X's own 1.2 (`ft4_decode.f90:195`) is a floor rather
    /// than a preference, not a number to copy to another mode.
    BaselineNormalised = 1,
    /// Sync power as a fraction of sync plus noise, so it lies in 0‥1:
    /// noise scores near 0, a clean aligned frame near 1. WSPR, JT9,
    /// JT65 and every Q65 sub-mode, whose shared default is 0.1.
    SyncFraction = 2,
}

/// Geometry and capability for one mode. **Size-versioned**: set
/// `size = sizeof(MfskModeInfo)` before the call, or pass a zeroed
/// struct and the library fills `size` in. A library newer than the
/// header writes only the prefix the caller declared.
///
/// `MfskResult` grew a field in 0.8.1 with nothing marking it; that
/// must not be repeatable.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskModeInfo {
    /// `sizeof(MfskModeInfo)` as the caller understands it.
    pub size: u32,
    /// The mode this describes — echoed back so a caller can pass the
    /// struct around on its own.
    pub mode: MfskMode,
    /// Stable display name, NUL-terminated (`"FT8"`, `"FST4-120"`).
    /// Also the key `mfsk_mode_from_name` accepts.
    pub name: [core::ffi::c_char; 16],
    /// Number of FSK tones.
    pub ntones: u32,
    /// Information bits per modulated symbol.
    pub bits_per_symbol: u32,
    /// Samples per symbol at 12 kHz.
    pub nsps: u32,
    /// Symbol duration, seconds.
    pub symbol_dt: f32,
    /// Tone-to-tone spacing, Hz.
    pub tone_spacing_hz: f32,
    /// Gaussian bandwidth-time product; 0 for plain FSK.
    pub gfsk_bt: f32,
    /// FSK modulation index.
    pub gfsk_hmod: f32,
    /// Data symbols per frame.
    pub n_data: u32,
    /// Sync symbols per frame; 0 for interleaved-sync protocols.
    pub n_sync: u32,
    /// Total channel symbols per frame.
    pub n_symbols: u32,
    /// Nominal slot length, seconds.
    pub t_slot_s: f32,
    /// Slot length in samples at 12 kHz — `t_slot_s` made exact, so a
    /// caller sizes a buffer without repeating the multiply. FT4
    /// 90 000, FT8 180 000, FST4-300 3 600 000.
    pub slot_samples_12k: u32,
    /// Seconds from the start of the slot buffer to the first frame
    /// symbol — the `dt = 0` reference. 0.5 for FT8, FT4 and FST4-15;
    /// 1.0 for the other FST4 sub-modes. A host that synthesises a slot
    /// has to know this and previously could not ask.
    pub tx_start_offset_s: f32,
    /// FEC information bits — 91 (CRC-14) or 101 (CRC-24).
    pub fec_k: u32,
    /// FEC codeword length in bits.
    pub fec_n: u32,
    /// Message-codec payload width in bits.
    pub payload_bits: u32,
    /// Length of the forward FFT the decoder takes over the whole slot,
    /// or 0 for a mode with its own front end.
    ///
    /// **This is the field that makes "one call shape for every mode"
    /// wrong as a memory story.** FT4 takes 92 160 points and FST4-300
    /// takes 4 194 304 — a factor of 45 that no other field here hints
    /// at. A mobile caller deciding which modes it can afford should
    /// read this one.
    pub decode_fft1_size: u32,
    /// Bitwise OR of the `MFSK_CAP_*` constants, which `mfsk-ffi`
    /// defines — they have to live in the crate cbindgen generates
    /// from, or they reach C as an unnamed `uint64_t` and every caller
    /// re-derives the bit positions by hand, which is the failure this
    /// whole surface exists to end.
    pub caps: u64,
}

/// A mode's default search parameters, published per mode instead of
/// hidden in three incompatible branches of one function.
///
/// Size-versioned on the same contract as [`MfskModeInfo`].
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskDecodeDefaults {
    /// `sizeof(MfskDecodeDefaults)` as the caller understands it.
    pub size: u32,
    /// Low edge of the default search band, Hz.
    pub freq_min_hz: f32,
    /// High edge of the default search band, Hz.
    pub freq_max_hz: f32,
    /// Default sync threshold — **read `sync_scale` before copying this
    /// number anywhere.**
    pub sync_min: f32,
    /// Default candidate budget.
    pub max_cand: u32,
    /// What scale `sync_min` is measured on. FT4's is not comparable
    /// with FT8's or FST4's, and a caller that copies one across modes
    /// is wrong with nothing to tell it so.
    pub sync_scale: MfskSyncScale,
}

// ──────────────────────────────────────────────────────────────────────────
// Decode parameters and result rows (FFI v2 slice 3)
// ──────────────────────────────────────────────────────────────────────────

/// Inline capacity for an a-priori hint field. A callsign is at most 13
/// characters in WSJT's own grammar; 16 leaves room and keeps the struct
/// aligned.
pub const MFSK_AP_FIELD_LEN: usize = 16;

/// Capacity of [`MfskDecode::text`], including the NUL.
///
/// Widened from the 40 that [`MfskResult`] carries. That number was the
/// longest WSJT-77 message plus one, which is true and was still too
/// tight the moment a hash-resolved `<...>` callsign expands in place.
pub const MFSK_DECODE_TEXT_LEN: usize = 64;

/// Everything a decode can be asked to do, as one size-versioned
/// struct passed by `const*`.
///
/// This replaces an opaque handle with eight setter functions. Three
/// reasons, in order of how much they matter:
///
/// 1. **The handle was unsound.** Its accessor fabricated a
///    `&'static mut` from a raw pointer with no synchronisation, which
///    is UB under Stacked Borrows the moment two setters' borrows
///    overlap — single-threaded, never mind concurrently. A plain
///    `#[repr(C)]` struct the caller owns has no such question.
/// 2. **Marshalling.** Kotlin and Swift wrappers copy one struct
///    instead of sequencing eight fallible calls.
/// 3. **Growth is still safe**, via `size` — see [`MfskModeInfo`].
///
/// Initialise with `mfsk_decode_params_init(mode, &params)`, which
/// fills in that mode's published defaults; then override what you
/// want. Zeroing the struct by hand is *not* equivalent: a zero
/// `max_cand` or a zero frequency band decodes nothing.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskDecodeParams {
    /// `sizeof(MfskDecodeParams)` as the caller understands it.
    pub size: u32,
    /// Low edge of the search band, Hz.
    pub freq_min_hz: f32,
    /// High edge of the search band, Hz.
    pub freq_max_hz: f32,
    /// Sync threshold. **Not comparable across modes** — see
    /// `MfskDecodeDefaults::sync_scale`.
    pub sync_min: f32,
    /// Candidate budget.
    pub max_cand: u32,
    /// Cost/recall rung.
    pub depth: MfskDecodeDepth,
    /// Accept/reject threshold profile.
    pub strictness: MfskStrictness,
    /// Equalisation. A property of the *input audio* — it flattens a
    /// passband an analogue filter has tilted — not of the search, so
    /// it belongs here rather than only on a narrow-band call.
    pub eq_mode: MfskEqMode,
    /// Prioritise candidates near this frequency. NaN means unset,
    /// which is what `mfsk_decode_params_init` writes.
    pub freq_hint_hz: f32,
    /// Successive-interference-cancellation rounds, 0 for none.
    /// Requires `MFSK_CAP_SIC_ROUNDS`.
    pub sic_rounds: u8,
    /// Checkpoint-emulation early decode. Requires `MFSK_CAP_SIC_EARLY`.
    pub sic_early: bool,
    /// Whether the three `ap_*` fields below carry a hint.
    pub has_ap_hint: bool,
    /// A-priori hint: the transmitting station, NUL-terminated, or
    /// empty. Requires `MFSK_CAP_AP_WIDEBAND` (or `_AP_NARROW` on a
    /// narrow-band call).
    pub ap_call1: [core::ffi::c_char; MFSK_AP_FIELD_LEN],
    /// A-priori hint: the correspondent, or `"CQ"`.
    pub ap_call2: [core::ffi::c_char; MFSK_AP_FIELD_LEN],
    /// A-priori hint: the grid square.
    pub ap_grid: [core::ffi::c_char; MFSK_AP_FIELD_LEN],
    /// Half-width of a narrow-band search, Hz; 0 for the mode's
    /// default. Only meaningful with `MFSK_CAP_SNIPER`.
    pub search_hz: f32,
}

/// One decoded transmission, written into caller memory.
///
/// Shaped on `mfsk_core`'s own `msg::decoded::Decoded`, which
/// `docs/notes/DECODED_ROW.md` says was made flat "precisely so it can
/// map to a C struct in `mfsk-ffi` later". This is that later.
///
/// Rows go into an array the caller allocates, which deletes the whole
/// "a Rust global-allocator pointer crosses the boundary and must come
/// back to be freed" category — the thing that makes Kotlin and Swift
/// wrappers fiddly and leaks when an exception unwinds past the free.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskDecode {
    /// `sizeof(MfskDecode)` as the caller understands it.
    pub size: u32,
    /// The **concrete sub-mode**, not the family. `Decoded::protocol`
    /// collapses all five FST4 periods onto one id; this must not,
    /// because the addressing model does not.
    pub mode: MfskMode,
    /// Decoded message text, NUL-terminated.
    pub text: [core::ffi::c_char; MFSK_DECODE_TEXT_LEN],
    /// Carrier frequency, Hz.
    pub freq_hz: f32,
    /// Time offset from the slot's `dt = 0` reference, seconds.
    pub dt_sec: f32,
    /// Estimated SNR in a 2500 Hz reference bandwidth, dB.
    pub snr_db: f32,
    /// Sync correlation score for this decode.
    pub sync_score: f32,
    /// Coefficient of variation of the per-block sync powers — near 0
    /// on a stable channel, elevated under QSB or fading. Free to
    /// report, and the only fading indicator the row carries.
    pub sync_cv: f32,
    /// Hard-decision errors the FEC had to correct.
    pub hard_errors: u32,
    /// Width of the FEC information block — 91 (CRC-14) or 101
    /// (CRC-24). Says how many bits `mfsk_decoder_copy_info` returns.
    pub info_bits: u16,
    /// Which decode pass produced this row. **Protocol-private**: the
    /// numbers mean different things for different modes, and are for
    /// diagnostics, not for logic.
    pub pass: u8,
    /// Bit 0: the text required the callsign hash table to resolve a
    /// `<...>` reference. Other bits reserved, currently zero.
    pub flags: u8,
}

/// [`MfskDecode::flags`] bit 0.
pub const MFSK_DECODE_FLAG_HASH_RESOLVED: u8 = 1 << 0;

/// Opaque decode-session handle (FFI v2).
///
/// Deliberately **not** the same type as `MfskDecoder`, the pre-v2
/// handle: the two own different Rust values, and a `MfskDecoder*` that
/// wandered into `mfsk_session_close` (or the reverse) would be
/// undefined behaviour that no compiler had any way to notice. Distinct
/// incomplete types make that a C type error instead, which is what the
/// legacy surface's own history argues for — it is being retired in
/// part because a handle whose meaning depends on which function you
/// pass it to is exactly the failure mode this redesign exists to end.
///
/// "Session" rather than "decoder" because it is the right word for
/// what it owns: a callsign hash table and the previous slot's rows,
/// both of which only mean anything across more than one call.
pub struct MfskDecodeSession {
    _marker: PhantomData<*mut ()>,
}
