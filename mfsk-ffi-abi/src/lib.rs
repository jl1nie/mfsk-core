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
//! `_new`/`_free` allocation logic for `MfskDecoder` itself,
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

use core::marker::PhantomData;

// ──────────────────────────────────────────────────────────────────────────
// Status codes
// ──────────────────────────────────────────────────────────────────────────

/// Outcome of a fallible `mfsk_*` call.
///
/// Zero is success; negative values are errors. `mfsk_last_error` (and
/// `mfsk_decoder_last_error` per decoder handle) returns a human-readable
/// string for the specific failure reason — the numeric code stays a small, stable
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
    /// FST4-60A — 60 s period.
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
    /// JTTY — WSJT-X 3.2's weak-signal keyboard mode. **Not slotted**:
    /// 1.888 s frames that start whenever the sender likes, a message of
    /// several of them, assembled by the receiver. It has no `Protocol`
    /// marker type and no registry entry (like MSK144), and it is driven
    /// through its own handle, `mfsk_jtty_open` and the calls after it,
    /// which is what `MFSK_CAP_STREAM_RECEIVER` says. `mfsk_mode_info`
    /// describes one frame: `t_slot_s` is the frame period, not a slot.
    Jtty = 25,
}

/// Geometry and capability for one mode. **Size-versioned**: set
/// `size = sizeof(MfskModeInfo)` before the call, or pass a zeroed
/// struct and the library fills `size` in. A library newer than the
/// header writes only the prefix the caller declared.
///
/// An earlier result struct grew a field with nothing marking it; that
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

// ──────────────────────────────────────────────────────────────────────────
// Decode parameters and result rows (FFI v2 slice 3)
// ──────────────────────────────────────────────────────────────────────────

/// Inline capacity for an a-priori hint field. A callsign is at most 13
/// characters in WSJT's own grammar; 16 leaves room and keeps the struct
/// aligned.
pub const MFSK_AP_FIELD_LEN: usize = 16;

/// Capacity of [`MfskDecode::text`], including the NUL.
///
/// Widened from the 40 that the old `MfskResult` carried. That number was the
/// longest WSJT-77 message plus one, which is true and was still too
/// tight the moment a hash-resolved `<...>` callsign expands in place.
pub const MFSK_DECODE_TEXT_LEN: usize = 64;

/// Bytes of [`MfskDecode::key`]: the 77 message bits of a 77-bit mode, packed (10 bytes).
pub const MFSK_DECODE_KEY_LEN: usize = 10;

/// The per-period parameter block, after WSJT-X's `params` common block
/// (`lib/jt9com.f90`) — what the GUI fills before each period and the
/// decoder reads. **Size-versioned**: initialise with
/// `mfsk_params_init(mode, &p)`, which writes the mode's defaults, then
/// override what you want. Every field is a plain integer or float (no
/// `enum` or `bool`), so a value from a config file or a newer header is a
/// wrong answer this ABI can refuse, never an invalid Rust value.
///
/// A mode reads what its upstream decoder reads and ignores the rest, as
/// `jt9` does; `depth` decides every search setting the way `ndepth` does.
/// The library's own options are in [`MfskExtras`].
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskParams {
    /// `sizeof(MfskParams)` as the caller understands it.
    pub size: u32,
    /// `MFSK_DEPTH_*`: `ndepth & 7`. 0 is the default, Deep (the GUI's).
    pub depth: u32,
    /// `MFSK_PARAM_*` bits: averaging (`ndepth & 16`, JT65 and Q65), deep
    /// search (`ndepth & 32`, JT65), EME delay (`emedelay`).
    pub flags: u32,
    /// `MFSK_AP_*`: AP off, CQ only (`lapcqonly`), or every hypothesis the
    /// QSO context allows. `mfsk_params_init` writes the mode's own default
    /// (off for FT8 and JT65, as the GUI's "Enable AP" boxes).
    pub ap_mode: u32,
    /// `MFSK_CONTEST_*`: `ncontest`.
    pub contest: u32,
    /// `nQSOProgress`, 0..=5: CALLING, REPLYING, REPORT, ROGER_REPORT,
    /// ROGERS, SIGNOFF.
    pub qso_progress: u32,
    /// Low edge of the audio band searched, Hz (`nfa`).
    pub band_lo_hz: f32,
    /// High edge, Hz (`nfb`).
    pub band_hi_hz: f32,
    /// The Rx frequency, Hz (`nfqso`). NaN is unset.
    pub rx_freq_hz: f32,
    /// Tolerance around the Rx frequency, Hz (`ntol`). NaN is unset.
    pub tol_hz: f32,
    /// The Tx frequency, Hz (`nftx`). NaN is unset.
    pub tx_freq_hz: f32,
    /// `mycall`, NUL-terminated, or empty.
    pub mycall: [core::ffi::c_char; 16],
    /// `mygrid`.
    pub mygrid: [core::ffi::c_char; 8],
    /// `hiscall`.
    pub hiscall: [core::ffi::c_char; 16],
    /// `hisgrid`.
    pub hisgrid: [core::ffi::c_char; 8],
}

/// The library's options beyond the parameter block, per mode. **Size-
/// versioned**; initialise with `mfsk_extras_init`, which writes "unset"
/// everywhere (NaN for a float, 0 for a count, -1 for a choice that has a
/// default), then set what you want. An option the mode does not have is
/// refused with `MFSK_STATUS_UNSUPPORTED`, naming it, never dropped.
/// `mfsk_decoder_set_extras` replaces the whole block, so what you leave
/// unset goes back to the depth's value.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskExtras {
    /// `sizeof(MfskExtras)` as the caller understands it.
    pub size: u32,
    /// Sync threshold over the depth's. NaN is the depth's. **Not
    /// comparable across modes.**
    pub sync_min: f32,
    /// Candidate budget over the depth's. 0 is the depth's.
    pub max_cand: u32,
    /// OSD over the depth's: -1 the depth's, 0 off, 1 on.
    pub osd: i32,
    /// Accept/reject profile: -1 default, 0 strict, 1 normal, 2 deep.
    pub strictness: i32,
    /// `MFSK_STRATEGY_*`: 0 the depth's, 1 one pass, 2 `sic_rounds` rounds
    /// of subtraction, 3 the checkpointed passes (FT8).
    pub strategy: u32,
    /// Rounds for `MFSK_STRATEGY_SIC_ROUNDS`.
    pub sic_rounds: u32,
    /// 0 off, 1 local per-signal equalisation. A property of the audio.
    pub eq_mode: u32,
    /// 0 the protocol's own message filter, 1 the codec's verdict alone.
    pub message_filter: u32,
    /// Non-zero turns on FT8's a7 list decoder (`ft8_a7.f90`), fed by the
    /// decoder's own decodes two periods back. Needs a period index.
    pub a7: u32,
    /// Half-width of FT8's roofing-filter search around the Rx frequency,
    /// Hz; 0 is the wide-band search.
    pub sniper_hz: f32,
    /// Non-zero when the four `ap_*` fields carry a free-form hint, beside
    /// the QSO-context AP (it wins when given): the message's fields in
    /// order, `ap_call1` being `"CQ"` for a CQ.
    pub has_ap_hint: u32,
    /// See `has_ap_hint`.
    pub ap_call1: [core::ffi::c_char; 16],
    /// See `has_ap_hint`.
    pub ap_call2: [core::ffi::c_char; 16],
    /// See `has_ap_hint`.
    pub ap_grid: [core::ffi::c_char; 16],
    /// See `has_ap_hint`: `"RRR"`, `"RR73"`, `"73"` or a report.
    pub ap_report: [core::ffi::c_char; 16],
    /// FST4 noise blanker, percent (`0..=25`); 0 off.
    pub nb_percent: u32,
    /// FST4: non-zero decodes once per blanking level; 5, 2 or 1.
    pub nb_sweep_step: u32,
    /// FST4: half-width of the blanked passes' window around the Rx
    /// frequency, Hz. Needed with `nb_sweep_step`.
    pub nb_ftol_hz: f32,
    /// WSPR, JT9, JT65 and Q65: how far before the nominal start a frame may
    /// begin, seconds. NaN is the mode's own.
    pub t_early_s: f32,
    /// As `t_early_s`, after the nominal start.
    pub t_late_s: f32,
    /// As `t_early_s`: coarse-sync acceptance, 0..1.
    pub score_threshold: f32,
    /// WSPR: Fano cycles per bit (`wsprd -C`); 0 is the depth's.
    pub max_cycles_per_bit: u32,
    /// JT65: Chase trials (`nvec`); 0 is the depth's.
    pub chase_trials: u32,
    /// Q65 Pileup: a reply carrying "copied last Tx" matches an AP hint.
    pub pileup: u32,
    /// Q65 Max Drift in spectrum bins (`0..=50`); 0 off.
    pub max_drift: u32,
    /// Q65 fast-fading metric: spread bandwidth times symbol period. NaN
    /// is the plain metric.
    pub fading_b90_ts: f32,
    /// Q65: 0 Gaussian, 1 Lorentzian. Read only with `fading_b90_ts`.
    pub fading_model: u32,
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
    /// Sync score for this decode, on the scale of the mode's own search (not
    /// comparable between modes). `0.0` and [`MFSK_DECODE_FLAG_HAS_SYNC_SCORE`]
    /// clear where the mode reports none: WSPR, JT9, JT65, Q65, and FT8's a7 and
    /// a8 list decodes.
    pub sync_score: f32,
    /// Coefficient of variation of the per-block sync powers — near 0
    /// on a stable channel, elevated under QSB or fading. Free to
    /// report, and the only fading indicator the row carries. `0.0` and
    /// [`MFSK_DECODE_FLAG_HAS_SYNC_CV`] clear where `sync_score` is absent.
    pub sync_cv: f32,
    /// Hard-decision errors the FEC had to correct. `0` and
    /// [`MFSK_DECODE_FLAG_HAS_HARD_ERRORS`] clear for WSPR, JT9, JT65 and Q65,
    /// whose decoders report no such count (a clean decode is `0` with the flag set).
    pub hard_errors: u32,
    /// Width of the FEC information block — 91 (CRC-14) or 101
    /// (CRC-24). Says how many bits `mfsk_decoder_copy_info` returns.
    pub info_bits: u16,
    /// Which decode pass produced this row. **Protocol-private**: the
    /// numbers mean different things for different modes, and are for
    /// diagnostics, not for logic.
    pub pass: u8,
    /// Bit 0: the text required the callsign hash table to resolve a
    /// `<...>` reference. Bit 1: the sender set Q65 Pileup's "copied last
    /// Tx" flag. Bits 2-4: `sync_score`, `sync_cv` and `hard_errors` are
    /// real values and not the `0` of a mode that reports none. Other bits
    /// reserved, currently zero.
    pub flags: u8,
    /// How many bits of [`Self::key`] are the message's: 77 for FT8, FT4, FST4 and
    /// Q65, 72 for JT9 and JT65, 50 for WSPR. `0`: no key.
    pub key_bits: u8,
    /// The message's identity key, `key_bits` bits packed most significant bit
    /// first, zero-padded: the same message heard on two channels or in two decoders
    /// has the same key, which the text may not (a `<...>` resolves in one and not
    /// the other), and a row can be matched by it. One message at two frequencies
    /// has one key. For the 77-bit modes it is the first 77 bits of
    /// `mfsk_decoder_copy_info`'s block (the rest is the CRC, a function of them).
    pub key: [u8; MFSK_DECODE_KEY_LEN],
    /// Which delivery of the period this row is, or came from (`RowDetail::delivery`,
    /// #592): a row handed to the callback carries its position (0, 1, 2...), a
    /// returned row the position of the delivery it was, so the two are paired
    /// exactly. `-1`: none (a returned row the callback never saw, or any row of a
    /// call with no callback).
    pub delivery: i32,
    /// When a `mfsk_decoder_decode_prefix_*` sequence found the row
    /// (`RowDetail::stage`, #572): [`MFSK_STAGE_EARLY`] for a call made
    /// before the period ended (FT8's checkpoint A, ~11.8 s in),
    /// [`MFSK_STAGE_FINAL`] for the call whose audio was the whole period,
    /// [`MFSK_STAGE_NONE`] from a plain decode. Appended.
    pub stage: u8,
}

/// [`MfskDecode::stage`]: not from a prefix sequence.
pub const MFSK_STAGE_NONE: u8 = 0;
/// [`MfskDecode::stage`]: found before the period ended, in time to answer
/// the station in the next one.
pub const MFSK_STAGE_EARLY: u8 = 1;
/// [`MfskDecode::stage`]: found by the call whose audio was the whole period.
pub const MFSK_STAGE_FINAL: u8 = 2;

/// [`MfskDecode::flags`] bit 0.
pub const MFSK_DECODE_FLAG_HASH_RESOLVED: u8 = 1 << 0;

/// [`MfskDecode::flags`] bit 1: the sender set WSJT-X 3.2's **Q65 Pileup**
/// "copied last Tx" flag, the spare 78th payload bit (`genq65.f90`'s `iflag`).
/// WSJT-X marks such a decode with `#`. Q65 rows only.
pub const MFSK_DECODE_FLAG_COPIED_LAST_TX: u8 = 1 << 1;

/// [`MfskDecode::flags`] bit 2: `sync_score` is a value the mode reported.
pub const MFSK_DECODE_FLAG_HAS_SYNC_SCORE: u8 = 1 << 2;

/// [`MfskDecode::flags`] bit 3: `sync_cv` is a value the mode reported.
pub const MFSK_DECODE_FLAG_HAS_SYNC_CV: u8 = 1 << 3;

/// [`MfskDecode::flags`] bit 4: `hard_errors` is a count the mode reported.
pub const MFSK_DECODE_FLAG_HAS_HARD_ERRORS: u8 = 1 << 4;

/// Opaque decoder handle: one persistent decoder of one mode, driven once
/// per period like WSJT-X's own (`jt9 -s`).
///
/// It owns what upstream keeps across periods and nothing else: the
/// callsign hash table (never shared with another decoder, as upstream's
/// is not), FT8's a7 list, Q65's and JT65's averages, WSPR's call table.
/// Emitted as an incomplete type, so a `MfskDecoder*` cannot wander into a
/// call that wants another handle.
pub struct MfskDecoder {
    _marker: PhantomData<*mut ()>,
}

// ──────────────────────────────────────────────────────────────────────────
// JTTY receiver handle (#477, P4b)
// ──────────────────────────────────────────────────────────────────────────

/// Bytes in [`MfskJttyUpdate::text`], including the NUL. A message keeps at
/// most 80 characters (`jtty::assemble`); the rest is room for the gap marks
/// that upstream's display puts between frames that were never heard.
pub const MFSK_JTTY_TEXT_BUF_LEN: usize = 128;

/// Receive settings for `mfsk_jtty_open` / `mfsk_jtty_set_params`.
/// **Size-versioned**, like `MfskModeInfo`: set `size = sizeof(MfskJttyParams)`,
/// or call `mfsk_jtty_params_init`, which fills in the defaults (`rjtty`'s).
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskJttyParams {
    /// `sizeof(MfskJttyParams)` as the caller understands it.
    pub size: u32,
    /// Take each decoded frame off the signal and search again, and re-search
    /// the windows before it. Non-zero (the default) is upstream's receiver;
    /// zero is a single-signal receiver that loses a weak station under a
    /// strong one.
    pub subtract: u32,
    /// The operator's receive frequency, Hz (channel 0's centre). Default 1500.
    pub f0_hz: f32,
    /// Half-width of channel 0, Hz. Default 50.
    pub ftol_hz: f32,
    /// Sync-gate S/N floor on channel 0, dB. Default 4.6.
    pub smin_db: f32,
    /// Lowest audio frequency channels 1 and 2 look at, Hz. Default 200.
    pub nfa_hz: f32,
    /// Highest audio frequency channels 1 and 2 look at, Hz. Default 2800.
    pub nfb_hz: f32,
}

/// One message as far as it is known, as `mfsk_jtty_poll` hands it out.
/// **Size-versioned.**
///
/// A message is reported each time it grows and once more when it completes.
/// Updates are **coalesced per message between polls**: if a message grew
/// twice since the last poll, the poll returns its latest text once — the
/// same rule as upstream's `jtty_get_updates`. `id` is stable for the life of
/// a message, so a caller replaces its display row by `id`.
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskJttyUpdate {
    /// `sizeof(MfskJttyUpdate)` as the caller understands it.
    pub size: u32,
    /// Non-zero once the end-of-message frame has arrived. A message that was
    /// given up on (no continuation came, or `mfsk_jtty_finish` was called) is
    /// reported one last time with this still 0.
    pub complete: u32,
    /// Stable for the life of the message.
    pub id: u64,
    /// Frequency of the latest frame, Hz.
    pub f1_hz: f32,
    /// Start of the first frame, seconds from the first sample pushed since
    /// `mfsk_jtty_open` / `mfsk_jtty_reset`.
    pub start_s: f32,
    /// The text so far, NUL-terminated UTF-8. Frames that were never heard
    /// show as ` ... `; TEXT5 spaces as `~`, as upstream shows them.
    pub text: [core::ffi::c_char; 128],
}
const _: () = assert!(MFSK_JTTY_TEXT_BUF_LEN == 128);

/// One decode out of `mfsk_iq_poll`: the row of a channel of a wideband IQ
/// stream, with the absolute RF frequency and where its slot started.
///
/// Size-versioned like the other rows: `size` is `sizeof(MfskIqDecode)` as
/// the caller understands it (0 means the whole struct).
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskIqDecode {
    /// `sizeof(MfskIqDecode)` as the caller understands it.
    pub size: u32,
    /// The handle `mfsk_iq_add_channel` returned.
    pub channel: u32,
    /// The channel's concrete mode.
    pub mode: MfskMode,
    /// Non-zero when a time anchor was set, so `slot_start_utc_ns` means
    /// something. Zero on a free-running grid.
    pub has_utc: u32,
    /// RF frequency of tone 0, Hz: the channel's dial plus `freq_hz`.
    pub abs_freq_hz: f64,
    /// The slot's index on the mode's UTC grid (UTC `period * T` with a
    /// clock set, counted from sample 0 without one).
    pub period: i64,
    /// Index (of the IQ stream, complex samples) the slot started at.
    pub slot_start_sample: u64,
    /// UTC of the slot start, ns since the Unix epoch, when `has_utc`.
    pub slot_start_utc_ns: i64,
    /// Audio frequency of tone 0 within the channel, Hz.
    pub freq_hz: f32,
    /// Time offset from the slot's `dt = 0` reference, seconds.
    pub dt_sec: f32,
    /// Estimated SNR in a 2500 Hz reference bandwidth, dB.
    pub snr_db: f32,
    /// Decoded message text, NUL-terminated.
    pub text: [core::ffi::c_char; MFSK_DECODE_TEXT_LEN],
    // ── Appended: the row's detail, as `MfskDecode` carries it ───────────
    // A caller built against the shorter struct sets the shorter `size` and
    // never sees these. Each has the meaning of the `MfskDecode` field of the
    // same name.
    /// As `MfskDecode::sync_score`: valid when `flags` has
    /// `MFSK_DECODE_FLAG_HAS_SYNC_SCORE`.
    pub sync_score: f32,
    /// As `MfskDecode::sync_cv`: valid when `flags` has
    /// `MFSK_DECODE_FLAG_HAS_SYNC_CV`.
    pub sync_cv: f32,
    /// As `MfskDecode::hard_errors`: valid when `flags` has
    /// `MFSK_DECODE_FLAG_HAS_HARD_ERRORS`.
    pub hard_errors: u32,
    /// As `MfskDecode::delivery`: which delivery of the channel decoder's
    /// callback (`mfsk_iq_channel_decoder` + `mfsk_decoder_set_on_decode`)
    /// this row was, so the two pair exactly; -1 when it had none.
    pub delivery: i32,
    /// As `MfskDecode::pass`. Protocol-private.
    pub pass: u8,
    /// `MFSK_DECODE_FLAG_*`, as on `MfskDecode`.
    pub flags: u8,
    /// As `MfskDecode::key_bits`.
    pub key_bits: u8,
    /// As `MfskDecode::key`: the message bits, packed. Compare rows by this,
    /// with the frequency, not by text.
    pub key: [u8; MFSK_DECODE_KEY_LEN],
}

/// The wideband IQ receiver handle.
///
/// Emitted as an incomplete type; what `mfsk_iq_open` allocates.
pub struct MfskIqReceiver {
    _marker: PhantomData<*mut ()>,
}

/// The JTTY receiver handle.
///
/// Emitted as an incomplete type, like `MfskDecoder`; the receiver is
/// what `mfsk_jtty_open` allocates.
pub struct MfskJttyReceiver {
    _marker: PhantomData<*mut ()>,
}

// ──────────────────────────────────────────────────────────────────────────
// Q65 extended decode (#466)
// ──────────────────────────────────────────────────────────────────────────

/// The DX station `mfsk_q65_history_lookup` found — `q65_hist`'s `dxcall` and
/// `dxgrid`. **Size-versioned.**
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskQ65Dx {
    /// `sizeof(MfskQ65Dx)` as the caller understands it.
    pub size: u32,
    /// Non-zero when the message carried a grid.
    pub has_grid: u32,
    /// The DX call, NUL-terminated, up to 12 characters.
    pub call: [core::ffi::c_char; 16],
    /// The four-character grid, NUL-terminated, when `has_grid`.
    pub grid: [core::ffi::c_char; 8],
}

/// One remembered caller, `mfsk_q65_callers_get`'s answer. **Size-versioned.**
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MfskQ65Caller {
    /// `sizeof(MfskQ65Caller)` as the caller understands it.
    pub size: u32,
    /// Its audio frequency when last heard, Hz.
    pub freq_hz: i32,
    /// When it was last heard, Unix seconds, as the caller passed to
    /// `mfsk_q65_callers_record`.
    pub last_heard: u64,
    /// Its call (up to six characters), NUL-terminated.
    pub call: [core::ffi::c_char; 8],
    /// The four-character grid it sent, NUL-terminated.
    pub grid: [core::ffi::c_char; 8],
}

/// The 100 most recent Q65 decodes and their frequencies — `q65_hist`. Used
/// to find the DX call on a "Decode Again" with none entered. Emitted as an
/// incomplete type, like `MfskDecoder`. **Not thread-safe.**
pub struct MfskQ65History {
    _marker: PhantomData<*mut ()>,
}

/// The contest caller list, up to 50 stations that called with a grid —
/// `q65_hist2`. Emitted as an incomplete type. **Not thread-safe.**
pub struct MfskQ65Callers {
    _marker: PhantomData<*mut ()>,
}
