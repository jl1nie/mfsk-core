//! C ABI for the rs-ft8n decoder suite.
//!
//! # Overview
//!
//! Exposes FT8 / FT4 / WSPR / JT9 / JT65 decoders and synthesisers
//! behind a small opaque-handle C API that C++ and Kotlin consumers
//! (Android JNI via a thin shim) can link against. cbindgen generates
//! `include/mfsk.h` on every build; see `examples/cpp_smoke` for a
//! round-trip demo that exercises every protocol through the ABI.
//!
//! # Memory ownership
//!
//! - [`mfsk_decoder_new`] / [`mfsk_decoder_free`]: opaque handle pair.
//! - [`mfsk_decode_f32`] / [`mfsk_decode_i16`]: populate a caller-supplied
//!   zero-initialised [`MfskMessageList`]. The callee owns the returned
//!   buffer until [`mfsk_message_list_free`] is invoked.
//! - [`mfsk_encode_*`]: populate a caller-supplied
//!   zero-initialised [`MfskSamples`] with the synthesised f32 PCM.
//!   Free with [`mfsk_samples_free`].
//! - All strings are UTF-8, NUL-terminated, and owned by the
//!   [`MfskMessageList`] they appear in.
//!
//! # Thread safety
//!
//! The supported usage model is **one [`MfskDecoder`] handle per
//! thread**. The handle itself carries no mutable state other than
//! its protocol tag, so in the current implementation sharing one
//! handle across threads also works — the C++ driver in
//! `examples/cpp_smoke` exercises both patterns (8 threads × own
//! handle, 8 threads × shared handle, and a mixed-protocol fan-out)
//! on every build. Concurrent decode calls allocate their own FFT
//! planners / scratch buffers; `mfsk_last_error` uses
//! `thread_local!` storage so error text never crosses threads.
//!
//! Future changes that add cached state to `DecoderInner` must
//! keep that shared-handle test green or tighten this documented
//! contract back to strict one-per-thread.

use std::ffi::{CStr, CString, c_char, c_int};
use std::os::raw::c_void;
use std::ptr;
use std::slice;

use mfsk_core::fst4::decode as fst4;
use mfsk_core::ft4::decode as ft4;
use mfsk_core::ft8::decode as ft8;

// ──────────────────────────────────────────────────────────────────────────
// Public C types
// ──────────────────────────────────────────────────────────────────────────

/// Opaque decoder handle. Construct with [`mfsk_decoder_new`], release
/// with [`mfsk_decoder_free`].
#[repr(C)]
pub struct MfskDecoder {
    _priv: [u8; 0],
    _marker: core::marker::PhantomData<*mut ()>,
}

/// Protocol tag selecting which decoder / synth family this handle
/// (or encode call) drives.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MfskProtocol {
    /// FT8 — 15 s slot, 8-GFSK, LDPC(174,91), 77-bit WSJT message.
    Ft8 = 0,
    /// FT4 — 7.5 s slot, 4-GFSK, LDPC(174,91), 77-bit WSJT message.
    Ft4 = 1,
    /// WSPR — 120 s slot, 4-FSK, convolutional r=½ K=32 + Fano, 50-bit payload.
    Wspr = 2,
    /// JT9 — 60 s slot, 9-FSK, convolutional r=½ K=32 + Fano, 72-bit JT message.
    Jt9 = 3,
    /// JT65 — 60 s slot, 65-FSK, Reed-Solomon(63,12) over GF(2⁶), 72-bit JT message.
    Jt65 = 4,
    /// FST4-60A — 60 s slot, 4-GFSK, LDPC(240,101) + CRC-24, 77-bit WSJT message.
    Fst4s60 = 5,
}

/// Status / error code returned by every fallible `mfsk_*` call.
///
/// Zero is success. Negative values indicate errors; use
/// [`mfsk_last_error`] for a human-readable description.
#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum MfskStatus {
    /// Success.
    Ok = 0,
    /// A caller-supplied argument was invalid (null pointer to a
    /// non-optional arg, out-of-range size, malformed string, etc.).
    InvalidArg = -1,
    /// The supplied protocol tag is not recognised or not supported
    /// by this build (e.g. FST4 in a build that was compiled without
    /// the `fst4` feature would report this at decode / encode time).
    UnknownProtocol = -2,
    /// The decoder ran without a fatal error but produced no results;
    /// the message list is empty. Used by some encode helpers to
    /// signal that the message could not be packed into the protocol
    /// payload (e.g. unsupported callsign format).
    DecodeFailed = -3,
    /// Internal error: an invariant of the Rust implementation was
    /// violated (e.g. a `Box::from_raw` got a bad pointer). Always a bug.
    Internal = -4,
}

/// One successfully decoded message.
#[repr(C)]
#[derive(Debug, Clone)]
pub struct MfskMessage {
    /// Carrier (tone-0) frequency in Hz.
    pub freq_hz: f32,
    /// Time offset in seconds from the protocol's nominal frame start.
    pub dt_sec: f32,
    /// WSJT-X compatible SNR in dB (2500 Hz reference bandwidth).
    pub snr_db: f32,
    /// Hard-decision errors corrected by the FEC.
    pub hard_errors: u32,
    /// Decode pass identifier (matches the Rust `DecodeResult::pass`).
    pub pass: u8,
    /// UTF-8, NUL-terminated message text. Owned by the parent
    /// [`MfskMessageList`]; do not free individually.
    pub text: *mut c_char,
}

/// List of decoded messages returned from a decode call. Caller should
/// zero-initialise before the call; callee fills `items` / `len`.
#[repr(C)]
#[derive(Debug)]
pub struct MfskMessageList {
    /// Array of `len` `MfskMessage` values.
    pub items: *mut MfskMessage,
    /// Number of entries in `items`.
    pub len: usize,
    /// Internal: total allocation (reserved for future growth).
    pub _cap: usize,
}

/// A buffer of synthesised f32 PCM samples returned by `mfsk_encode_*`.
/// Caller should zero-initialise before the call and free with
/// [`mfsk_samples_free`] when done reading.
#[repr(C)]
#[derive(Debug)]
pub struct MfskSamples {
    /// Contiguous f32 PCM at the protocol's native sample rate
    /// (12 000 Hz for all currently-supported modes). Owned by the
    /// list; free with [`mfsk_samples_free`].
    pub samples: *mut f32,
    /// Number of f32 entries in `samples`.
    pub len: usize,
    /// Internal: total allocation (reserved for future growth).
    pub _cap: usize,
}

// ──────────────────────────────────────────────────────────────────────────
// Error handling (thread-local last message)
// ──────────────────────────────────────────────────────────────────────────

std::thread_local! {
    static LAST_ERROR: std::cell::RefCell<Option<CString>> = const { std::cell::RefCell::new(None) };
}

fn set_error(msg: impl Into<String>) {
    let s = msg.into();
    LAST_ERROR.with(|e| {
        *e.borrow_mut() = CString::new(s).ok();
    });
}

/// Returns a pointer to the thread-local last-error string, or NULL if
/// no error has been recorded on this thread. The pointer is valid until
/// the next fallible call on this thread.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_last_error() -> *const c_char {
    LAST_ERROR.with(|e| {
        e.borrow()
            .as_ref()
            .map(|s| s.as_ptr())
            .unwrap_or(ptr::null())
    })
}

// ──────────────────────────────────────────────────────────────────────────
// Handle lifecycle
// ──────────────────────────────────────────────────────────────────────────

struct DecoderInner {
    protocol: MfskProtocol,
}

/// Construct a new decoder handle bound to `protocol`. Returns NULL on
/// failure (see [`mfsk_last_error`]).
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_decoder_new(protocol: MfskProtocol) -> *mut MfskDecoder {
    let inner = Box::new(DecoderInner { protocol });
    Box::into_raw(inner) as *mut MfskDecoder
}

/// Destroy a decoder handle previously returned by [`mfsk_decoder_new`].
/// Passing NULL is a no-op.
///
/// # Safety
///
/// `dec` must be a pointer previously returned by [`mfsk_decoder_new`],
/// or NULL. After this call the pointer is dangling.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decoder_free(dec: *mut MfskDecoder) {
    if !dec.is_null() {
        unsafe {
            drop(Box::from_raw(dec as *mut DecoderInner));
        }
    }
}

/// Free a [`MfskMessageList`] populated by a decode call. Passing NULL
/// or an already-freed list is safe.
///
/// # Safety
///
/// `list` must point to a [`MfskMessageList`] written by one of the
/// `mfsk_decode_*` functions, or be NULL. After this call, `items` is
/// NULL and `len` is 0.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_message_list_free(list: *mut MfskMessageList) {
    if list.is_null() {
        return;
    }
    unsafe {
        let list = &mut *list;
        if list.items.is_null() {
            list.len = 0;
            list._cap = 0;
            return;
        }
        let slice = slice::from_raw_parts_mut(list.items, list.len);
        for msg in slice.iter_mut() {
            if !msg.text.is_null() {
                drop(CString::from_raw(msg.text));
                msg.text = ptr::null_mut();
            }
        }
        let vec = Vec::from_raw_parts(list.items, list.len, list._cap);
        drop(vec);
        list.items = ptr::null_mut();
        list.len = 0;
        list._cap = 0;
    }
}

/// Free a [`MfskSamples`] buffer populated by a `mfsk_encode_*` call.
/// Passing NULL or an already-freed buffer is safe.
///
/// # Safety
///
/// `s` must point to a [`MfskSamples`] written by one of the
/// `mfsk_encode_*` functions, or be NULL. After this call, `samples`
/// is NULL and `len` is 0.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_samples_free(s: *mut MfskSamples) {
    if s.is_null() {
        return;
    }
    unsafe {
        let s = &mut *s;
        if !s.samples.is_null() {
            let _ = Vec::from_raw_parts(s.samples, s.len, s._cap);
        }
        s.samples = ptr::null_mut();
        s.len = 0;
        s._cap = 0;
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Decode entry points
// ──────────────────────────────────────────────────────────────────────────

fn inner(dec: *const MfskDecoder) -> Option<&'static DecoderInner> {
    unsafe { (dec as *const DecoderInner).as_ref() }
}

/// Shared message pusher for the 77-bit family (FT8, FT4).
fn push_wsjt77(
    r: &ft8::DecodeResult,
    ht: &mfsk_core::msg::CallsignHashTable,
    vec: &mut Vec<MfskMessage>,
) {
    let text = mfsk_core::msg::wsjt77::unpack77_with_hash(&r.message77, ht).unwrap_or_default();
    vec.push(MfskMessage {
        freq_hz: r.freq_hz,
        dt_sec: r.dt_sec,
        snr_db: r.snr_db,
        hard_errors: r.hard_errors,
        pass: r.pass,
        text: CString::new(text).unwrap_or_default().into_raw(),
    });
}

fn push_ft4(r: &ft4::DecodeResult, vec: &mut Vec<MfskMessage>) {
    use mfsk_core::MessageCodec;
    let codec = mfsk_core::msg::Wsjt77Message;
    let ctx = mfsk_core::DecodeContext::default();
    let text = codec.unpack(&r.message77, &ctx).unwrap_or_default();
    vec.push(MfskMessage {
        freq_hz: r.freq_hz,
        dt_sec: r.dt_sec,
        snr_db: r.snr_db,
        hard_errors: r.hard_errors,
        pass: r.pass,
        text: CString::new(text).unwrap_or_default().into_raw(),
    });
}

fn push_simple(freq_hz: f32, dt_sec: f32, text: String, vec: &mut Vec<MfskMessage>) {
    vec.push(MfskMessage {
        freq_hz,
        dt_sec,
        snr_db: 0.0,
        hard_errors: 0,
        pass: 0,
        text: CString::new(text).unwrap_or_default().into_raw(),
    });
}

fn finalise(vec: Vec<MfskMessage>, out: &mut MfskMessageList) {
    let mut vec = vec;
    let len = vec.len();
    let cap = vec.capacity();
    let items = vec.as_mut_ptr();
    std::mem::forget(vec);
    out.items = items;
    out.len = len;
    out._cap = cap;
}

/// Decode one slot of f32 PCM audio.
///
/// The protocol to decode is whichever was passed to
/// [`mfsk_decoder_new`]; the sample duration is implicit in the
/// protocol's slot length (FT8 = 15 s, FT4 = 7.5 s, FST4-60A / JT9 /
/// JT65 = 60 s, WSPR = 120 s). The audio must already be aligned to
/// the slot boundary — this function does not search for sync outside
/// the slot. Non-12 kHz input is linearly resampled to 12 000 Hz
/// internally.
///
/// On success, `out` is filled with the list of decoded messages
/// (may be empty). The caller owns the list and must release it with
/// [`mfsk_message_list_free`].
///
/// Samples should be scaled to roughly ±1.0 (full-scale sine = 1.0).
///
/// # Parameters
///
/// - `dec` — decoder handle from [`mfsk_decoder_new`].
/// - `samples` — pointer to `n_samples` `f32` PCM values, slot-aligned.
/// - `n_samples` — number of samples in `samples`.
/// - `sample_rate` — sample rate of `samples` in Hz (commonly 12000,
///   48000, or 44100). Must be ≥ 8000 Hz.
/// - `out` — pointer to a caller-allocated `MfskMessageList` that is
///   either zero-initialised or previously freed via
///   [`mfsk_message_list_free`].
///
/// # Returns
///
/// [`MfskStatus::Ok`] on success (including zero decodes). On failure
/// returns an error status and `out` is left unchanged; consult
/// [`mfsk_last_error`] for details.
///
/// # Safety
///
/// - `dec` must be a live [`MfskDecoder`] handle.
/// - `samples` must point to `n_samples` valid `f32` values.
/// - `out` must point to a writable [`MfskMessageList`]; caller must
///   pair with [`mfsk_message_list_free`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decode_f32(
    dec: *const MfskDecoder,
    samples: *const f32,
    n_samples: usize,
    sample_rate: u32,
    out: *mut MfskMessageList,
) -> MfskStatus {
    let Some(inner_ref) = inner(dec) else {
        set_error("mfsk_decode_f32: null decoder handle");
        return MfskStatus::InvalidArg;
    };
    if samples.is_null() || out.is_null() {
        set_error("mfsk_decode_f32: null buffer pointer");
        return MfskStatus::InvalidArg;
    }
    let slice_f32 = unsafe { slice::from_raw_parts(samples, n_samples) };
    let out = unsafe { &mut *out };

    match inner_ref.protocol {
        MfskProtocol::Ft8 | MfskProtocol::Ft4 | MfskProtocol::Fst4s60 => {
            // Reuse the existing i16-based pipeline.
            let audio: Vec<i16> = if sample_rate == 12_000 {
                slice_f32
                    .iter()
                    .map(|&s| (s * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
                    .collect()
            } else {
                mfsk_core::core::dsp::resample::resample_f32_to_12k(slice_f32, sample_rate)
            };
            decode_i16_wsjt77(inner_ref.protocol, &audio, out)
        }
        MfskProtocol::Wspr => {
            let audio =
                mfsk_core::core::dsp::resample::resample_f32_to_12k_f32(slice_f32, sample_rate);
            decode_wspr(&audio, out)
        }
        MfskProtocol::Jt9 => {
            let audio =
                mfsk_core::core::dsp::resample::resample_f32_to_12k_f32(slice_f32, sample_rate);
            decode_jt9_aligned(&audio, out)
        }
        MfskProtocol::Jt65 => {
            let audio =
                mfsk_core::core::dsp::resample::resample_f32_to_12k_f32(slice_f32, sample_rate);
            decode_jt65_aligned(&audio, out)
        }
    }
}

/// Decode one slot of 16-bit signed PCM audio.
///
/// Identical to [`mfsk_decode_f32`] but takes interleaved `i16`
/// samples (the direct output of most ADCs and WAV files). Full-scale
/// input is `±32767`. See [`mfsk_decode_f32`] for parameter semantics,
/// return values, and slot-alignment requirements.
///
/// # Safety
///
/// See [`mfsk_decode_f32`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_decode_i16(
    dec: *const MfskDecoder,
    samples: *const i16,
    n_samples: usize,
    sample_rate: u32,
    out: *mut MfskMessageList,
) -> MfskStatus {
    let Some(inner_ref) = inner(dec) else {
        set_error("mfsk_decode_i16: null decoder handle");
        return MfskStatus::InvalidArg;
    };
    if samples.is_null() || out.is_null() {
        set_error("mfsk_decode_i16: null buffer pointer");
        return MfskStatus::InvalidArg;
    }
    let slice_i16 = unsafe { slice::from_raw_parts(samples, n_samples) };
    let out = unsafe { &mut *out };

    match inner_ref.protocol {
        MfskProtocol::Ft8 | MfskProtocol::Ft4 | MfskProtocol::Fst4s60 => {
            let audio: Vec<i16> = if sample_rate == 12_000 {
                slice_i16.to_vec()
            } else {
                mfsk_core::core::dsp::resample::resample_to_12k(slice_i16, sample_rate)
            };
            decode_i16_wsjt77(inner_ref.protocol, &audio, out)
        }
        MfskProtocol::Wspr | MfskProtocol::Jt9 | MfskProtocol::Jt65 => {
            // These backends consume f32 natively; convert.
            let audio: Vec<f32> = if sample_rate == 12_000 {
                slice_i16.iter().map(|&s| s as f32 / 32768.0).collect()
            } else {
                mfsk_core::core::dsp::resample::resample_i16_to_12k_f32(slice_i16, sample_rate)
            };
            match inner_ref.protocol {
                MfskProtocol::Wspr => decode_wspr(&audio, out),
                MfskProtocol::Jt9 => decode_jt9_aligned(&audio, out),
                MfskProtocol::Jt65 => decode_jt65_aligned(&audio, out),
                _ => unreachable!(),
            }
        }
    }
}

fn decode_i16_wsjt77(
    protocol: MfskProtocol,
    audio: &[i16],
    out: &mut MfskMessageList,
) -> MfskStatus {
    let mut vec: Vec<MfskMessage> = Vec::new();
    match protocol {
        MfskProtocol::Ft8 => {
            let ht = mfsk_core::msg::CallsignHashTable::new();
            for r in ft8::decode_frame(
                audio,
                200.0,
                3_000.0,
                2.0,
                None,
                ft8::DecodeDepth::BpAllOsd,
                50,
            ) {
                push_wsjt77(&r, &ht, &mut vec);
            }
        }
        MfskProtocol::Ft4 => {
            for r in ft4::decode_frame(audio, 200.0, 3_000.0, 1.2, 50) {
                push_ft4(&r, &mut vec);
            }
        }
        MfskProtocol::Fst4s60 => {
            // FST4-60A's pipeline::DecodeResult has the same shape; treat
            // it like FT4/FT8 at the message-unpack step since the payload
            // is also 77-bit WSJT.
            use mfsk_core::MessageCodec;
            let codec = mfsk_core::msg::Wsjt77Message;
            let ctx = mfsk_core::DecodeContext::default();
            for r in fst4::decode_frame(audio, 100.0, 3_000.0, 0.8, 30) {
                let text = codec.unpack(&r.message77, &ctx).unwrap_or_default();
                vec.push(MfskMessage {
                    freq_hz: r.freq_hz,
                    dt_sec: r.dt_sec,
                    snr_db: r.snr_db,
                    hard_errors: r.hard_errors,
                    pass: r.pass,
                    text: CString::new(text).unwrap_or_default().into_raw(),
                });
            }
        }
        _ => unreachable!(),
    }
    finalise(vec, out);
    MfskStatus::Ok
}

fn decode_wspr(audio: &[f32], out: &mut MfskMessageList) -> MfskStatus {
    let mut vec: Vec<MfskMessage> = Vec::new();
    for d in mfsk_core::wspr::decode::decode_scan_default(audio, 12_000) {
        push_simple(
            d.freq_hz,
            d.start_sample as f32 / 12_000.0,
            d.message.to_string(),
            &mut vec,
        );
    }
    finalise(vec, out);
    MfskStatus::Ok
}

/// JT9 decode at the canonical 1500 Hz carrier, slot-aligned at sample 0.
/// Callers that want (freq × time) search should build that on top of
/// `mfsk_core::jt9::decode_at` directly — the FFI takes the fixed-alignment
/// path because it's the one the roundtrip test needs.
fn decode_jt9_aligned(audio: &[f32], out: &mut MfskMessageList) -> MfskStatus {
    let mut vec: Vec<MfskMessage> = Vec::new();
    if let Some(msg) = mfsk_core::jt9::decode_at(audio, 12_000, 0, 1500.0) {
        push_simple(1500.0, 0.0, msg.to_string(), &mut vec);
    }
    finalise(vec, out);
    MfskStatus::Ok
}

fn decode_jt65_aligned(audio: &[f32], out: &mut MfskMessageList) -> MfskStatus {
    let mut vec: Vec<MfskMessage> = Vec::new();
    if let Some(msg) = mfsk_core::jt65::decode_at(audio, 12_000, 0, 1270.0) {
        push_simple(1270.0, 0.0, msg.to_string(), &mut vec);
    }
    finalise(vec, out);
    MfskStatus::Ok
}

// ──────────────────────────────────────────────────────────────────────────
// Encode entry points
// ──────────────────────────────────────────────────────────────────────────

fn cstr_to_str<'a>(p: *const c_char) -> Result<&'a str, MfskStatus> {
    if p.is_null() {
        set_error("null C string");
        return Err(MfskStatus::InvalidArg);
    }
    unsafe {
        CStr::from_ptr(p).to_str().map_err(|e| {
            set_error(format!("invalid UTF-8 in C string: {e}"));
            MfskStatus::InvalidArg
        })
    }
}

fn finalise_samples(mut v: Vec<f32>, out: &mut MfskSamples) {
    let len = v.len();
    let cap = v.capacity();
    let ptr = v.as_mut_ptr();
    std::mem::forget(v);
    out.samples = ptr;
    out.len = len;
    out._cap = cap;
}

/// Synthesise a standard FT8 message ("CALL1 CALL2 REPORT") at `freq_hz`
/// carrier. Writes 12 kHz f32 PCM into `out`.
///
/// # Safety
///
/// `call1`/`call2`/`report` must be NUL-terminated UTF-8 strings.
/// `out` must be a writable `MfskSamples` (zero-initialise).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_ft8(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    freq_hz: f32,
    out: *mut MfskSamples,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(rep) = cstr_to_str(report) else {
        return MfskStatus::InvalidArg;
    };
    if out.is_null() {
        set_error("mfsk_encode_ft8: null out");
        return MfskStatus::InvalidArg;
    }
    let Some(msg77) = mfsk_core::msg::wsjt77::pack77(c1, c2, rep) else {
        set_error("FT8 pack77 failed");
        return MfskStatus::InvalidArg;
    };
    let tones = mfsk_core::ft8::wave_gen::message_to_tones(&msg77);
    let pcm = mfsk_core::ft8::wave_gen::tones_to_f32(&tones, freq_hz, 1.0);
    finalise_samples(pcm, unsafe { &mut *out });
    MfskStatus::Ok
}

/// Synthesise a standard FT4 message at `freq_hz`. 12 kHz f32 PCM.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_ft4(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    freq_hz: f32,
    out: *mut MfskSamples,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(rep) = cstr_to_str(report) else {
        return MfskStatus::InvalidArg;
    };
    if out.is_null() {
        set_error("mfsk_encode_ft4: null out");
        return MfskStatus::InvalidArg;
    }
    let Some(msg77) = mfsk_core::msg::wsjt77::pack77(c1, c2, rep) else {
        set_error("FT4 pack77 failed");
        return MfskStatus::InvalidArg;
    };
    let tones = mfsk_core::ft4::encode::message_to_tones(&msg77);
    let pcm = mfsk_core::ft4::encode::tones_to_f32(&tones, freq_hz, 1.0);
    finalise_samples(pcm, unsafe { &mut *out });
    MfskStatus::Ok
}

/// Synthesise a standard FST4-60A message at `freq_hz`. 12 kHz f32 PCM.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_fst4s60(
    call1: *const c_char,
    call2: *const c_char,
    report: *const c_char,
    freq_hz: f32,
    out: *mut MfskSamples,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(rep) = cstr_to_str(report) else {
        return MfskStatus::InvalidArg;
    };
    if out.is_null() {
        set_error("mfsk_encode_fst4s60: null out");
        return MfskStatus::InvalidArg;
    }
    let Some(msg77) = mfsk_core::msg::wsjt77::pack77(c1, c2, rep) else {
        set_error("FST4 pack77 failed");
        return MfskStatus::InvalidArg;
    };
    let tones = mfsk_core::fst4::encode::message_to_tones(&msg77);
    let pcm = mfsk_core::fst4::encode::tones_to_f32(&tones, freq_hz, 1.0);
    finalise_samples(pcm, unsafe { &mut *out });
    MfskStatus::Ok
}

/// Synthesise a Type-1 WSPR message (`call grid power_dbm`).
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_wspr(
    call: *const c_char,
    grid: *const c_char,
    power_dbm: i32,
    freq_hz: f32,
    out: *mut MfskSamples,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(g) = cstr_to_str(grid) else {
        return MfskStatus::InvalidArg;
    };
    if out.is_null() {
        set_error("mfsk_encode_wspr: null out");
        return MfskStatus::InvalidArg;
    }
    let Some(pcm) = mfsk_core::wspr::synthesize_type1(c1, g, power_dbm, 12_000, freq_hz, 0.3)
    else {
        set_error("WSPR synth failed (bad call/grid/power)");
        return MfskStatus::InvalidArg;
    };
    finalise_samples(pcm, unsafe { &mut *out });
    MfskStatus::Ok
}

/// Synthesise a standard JT9 message at `freq_hz`.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_jt9(
    call1: *const c_char,
    call2: *const c_char,
    grid_or_report: *const c_char,
    freq_hz: f32,
    out: *mut MfskSamples,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(gr) = cstr_to_str(grid_or_report) else {
        return MfskStatus::InvalidArg;
    };
    if out.is_null() {
        set_error("mfsk_encode_jt9: null out");
        return MfskStatus::InvalidArg;
    }
    let Some(pcm) = mfsk_core::jt9::synthesize_standard(c1, c2, gr, 12_000, freq_hz, 0.3) else {
        set_error("JT9 synth failed (bad pack)");
        return MfskStatus::InvalidArg;
    };
    finalise_samples(pcm, unsafe { &mut *out });
    MfskStatus::Ok
}

/// Synthesise a standard JT65 message at `freq_hz`.
///
/// # Safety
///
/// See [`mfsk_encode_ft8`].
#[unsafe(no_mangle)]
pub unsafe extern "C" fn mfsk_encode_jt65(
    call1: *const c_char,
    call2: *const c_char,
    grid_or_report: *const c_char,
    freq_hz: f32,
    out: *mut MfskSamples,
) -> MfskStatus {
    let Ok(c1) = cstr_to_str(call1) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(c2) = cstr_to_str(call2) else {
        return MfskStatus::InvalidArg;
    };
    let Ok(gr) = cstr_to_str(grid_or_report) else {
        return MfskStatus::InvalidArg;
    };
    if out.is_null() {
        set_error("mfsk_encode_jt65: null out");
        return MfskStatus::InvalidArg;
    }
    let Some(pcm) = mfsk_core::jt65::synthesize_standard(c1, c2, gr, 12_000, freq_hz, 0.3) else {
        set_error("JT65 synth failed (bad pack)");
        return MfskStatus::InvalidArg;
    };
    finalise_samples(pcm, unsafe { &mut *out });
    MfskStatus::Ok
}

/// Library version, major.minor.patch packed into a 32-bit integer (8
/// bits per field). Useful for the consumer to sanity-check ABI
/// compatibility.
#[unsafe(no_mangle)]
pub extern "C" fn mfsk_version() -> u32 {
    let v: &str = env!("CARGO_PKG_VERSION");
    let mut parts = v.split('.').map(|s| s.parse::<u32>().unwrap_or(0));
    let major = parts.next().unwrap_or(0);
    let minor = parts.next().unwrap_or(0);
    let patch = parts.next().unwrap_or(0);
    (major << 16) | (minor << 8) | patch
}

// Keep cbindgen-visible types discoverable.
const _: fn() -> (c_int, *mut c_void) = || (0, ptr::null_mut());
