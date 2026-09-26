//! Shared helpers for the C-ABI tests.
//!
//! The v2 surface hands rows to caller memory and takes one parameter
//! struct, which is better for a C consumer and slightly wordier from
//! Rust. These wrappers keep the tests about behaviour rather than
//! about marshalling.
#![allow(dead_code)]

use mfsk::*;

pub const FS: u32 = 12_000;

/// A row the library will overwrite. `size` is the caller's half of the
/// growth contract; everything else is don't-care.
pub fn blank_row() -> MfskDecode {
    MfskDecode {
        size: std::mem::size_of::<MfskDecode>() as u32,
        mode: MfskMode::Ft8,
        text: [0; MFSK_DECODE_TEXT_LEN],
        freq_hz: 0.0,
        dt_sec: 0.0,
        snr_db: 0.0,
        sync_score: 0.0,
        sync_cv: 0.0,
        hard_errors: 0,
        info_bits: 0,
        pass: 0,
        flags: 0,
    }
}

/// `mode`'s published defaults.
pub fn params(mode: MfskMode) -> MfskDecodeParams {
    let mut p = std::mem::MaybeUninit::<MfskDecodeParams>::zeroed();
    assert_eq!(
        unsafe { mfsk_decode_params_init(mode as u32, p.as_mut_ptr()) },
        MfskStatus::Ok,
        "{mode:?} has no defaults to initialise from"
    );
    unsafe { p.assume_init() }
}

/// Open a session, asserting it succeeded.
pub fn open(mode: MfskMode, p: Option<&MfskDecodeParams>) -> *mut MfskDecodeSession {
    let mut st = MfskStatus::Internal;
    let d = unsafe {
        mfsk_session_open(
            mode as u32,
            p.map(|p| p as *const _).unwrap_or(std::ptr::null()),
            &mut st,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "{mode:?} failed to open");
    assert!(!d.is_null());
    d
}

pub fn text_of(r: &MfskDecode) -> String {
    let b: &[u8] =
        unsafe { std::slice::from_raw_parts(r.text.as_ptr() as *const u8, r.text.len()) };
    let end = b.iter().position(|&c| c == 0).unwrap_or(b.len());
    String::from_utf8_lossy(&b[..end]).into_owned()
}

/// Decode i16 PCM through a session, returning the rows.
pub fn decode_i16(dec: *mut MfskDecodeSession, audio: &[i16]) -> Vec<MfskDecode> {
    decode_i16_with(dec, audio, None)
}

pub fn decode_i16_with(
    dec: *mut MfskDecodeSession,
    audio: &[i16],
    p: Option<&MfskDecodeParams>,
) -> Vec<MfskDecode> {
    let mut rows = vec![blank_row(); 64];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_session_decode_i16(
            dec,
            audio.as_ptr(),
            audio.len(),
            FS,
            p.map(|p| p as *const _).unwrap_or(std::ptr::null()),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "decode failed");
    rows.truncate(n);
    rows
}

pub fn decode_f32(dec: *mut MfskDecodeSession, audio: &[f32]) -> Vec<MfskDecode> {
    let mut rows = vec![blank_row(); 64];
    let mut n = 0usize;
    let st = unsafe {
        mfsk_session_decode_f32(
            dec,
            audio.as_ptr(),
            audio.len(),
            FS,
            std::ptr::null(),
            rows.as_mut_ptr(),
            rows.len(),
            &mut n,
        )
    };
    assert_eq!(st, MfskStatus::Ok, "decode failed");
    rows.truncate(n);
    rows
}

pub fn texts(rows: &[MfskDecode]) -> Vec<String> {
    rows.iter().map(text_of).collect()
}

pub fn any_contains(rows: &[MfskDecode], needle: &str) -> bool {
    rows.iter().any(|r| text_of(r).contains(needle))
}

/// Set an AP hint on a params struct.
pub fn with_ap(p: &mut MfskDecodeParams, call1: &str, call2: &str, grid: &str) {
    fn put(dst: &mut [std::ffi::c_char], s: &str) {
        let b = s.as_bytes();
        let n = b.len().min(dst.len() - 1);
        for (d, &c) in dst.iter_mut().zip(&b[..n]) {
            *d = c as std::ffi::c_char;
        }
        dst[n] = 0;
    }
    p.has_ap_hint = true;
    put(&mut p.ap_call1, call1);
    put(&mut p.ap_call2, call2);
    put(&mut p.ap_grid, grid);
}

/// Synthesise a frame through the three-stage TX pipeline, as i16 PCM.
///
/// Replaces the `mfsk_encode_* → MfskSamples → free` dance every test
/// used to open with. Nothing is allocated across the boundary.
pub fn synth_frame_i16(
    mode: MfskMode,
    call1: &str,
    call2: &str,
    report: &str,
    freq_hz: f32,
) -> Vec<i16> {
    use std::ffi::CString;
    let (a, b, c) = (
        CString::new(call1).unwrap(),
        CString::new(call2).unwrap(),
        CString::new(report).unwrap(),
    );
    let mut msg = [0u8; 77];
    assert_eq!(
        unsafe { mfsk_pack77(a.as_ptr(), b.as_ptr(), c.as_ptr(), msg.as_mut_ptr()) },
        MfskStatus::Ok,
        "pack77({call1}, {call2}, {report})"
    );
    let mut tones = vec![0u8; mfsk_symbol_count(mode as u32)];
    let mut n = 0usize;
    assert_eq!(
        unsafe {
            mfsk_message_to_tones(
                mode as u32,
                msg.as_ptr(),
                tones.as_mut_ptr(),
                tones.len(),
                &mut n,
            )
        },
        MfskStatus::Ok
    );
    let mut pcm = vec![0i16; mfsk_synth_output_len(mode as u32)];
    let mut w = 0usize;
    assert_eq!(
        unsafe {
            mfsk_tones_to_i16(
                mode as u32,
                tones.as_ptr(),
                tones.len(),
                freq_hz,
                8_000,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut w,
            )
        },
        MfskStatus::Ok
    );
    pcm
}

/// The same frame placed in a full slot, at that mode's TX offset.
pub fn synth_slot_i16(
    mode: MfskMode,
    call1: &str,
    call2: &str,
    report: &str,
    freq_hz: f32,
) -> Vec<i16> {
    let mut info = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
    assert_eq!(
        unsafe { mfsk_mode_info(mode as u32, info.as_mut_ptr()) },
        MfskStatus::Ok
    );
    let info = unsafe { info.assume_init() };
    let frame = synth_frame_i16(mode, call1, call2, report, freq_hz);
    let mut slot = vec![0i16; info.slot_samples_12k as usize];
    let start = (info.tx_start_offset_s * FS as f32) as usize;
    for (i, s) in frame.iter().enumerate() {
        if let Some(d) = slot.get_mut(start + i) {
            *d = d.saturating_add(*s);
        }
    }
    slot
}

/// f32 PCM from one of the `mfsk_encode_*` convenience calls, for the
/// modes with no tone stage.
pub fn encode_f32(
    enc: unsafe extern "C" fn(
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        *const std::ffi::c_char,
        f32,
        *mut f32,
        usize,
        *mut usize,
    ) -> MfskStatus,
    a: &str,
    b: &str,
    c: &str,
    freq: f32,
) -> Vec<f32> {
    use std::ffi::CString;
    let (a, b, c) = (
        CString::new(a).unwrap(),
        CString::new(b).unwrap(),
        CString::new(c).unwrap(),
    );
    let mut need = 0usize;
    let st = unsafe {
        enc(
            a.as_ptr(),
            b.as_ptr(),
            c.as_ptr(),
            freq,
            std::ptr::null_mut(),
            0,
            &mut need,
        )
    };
    assert_eq!(
        st,
        MfskStatus::InvalidArg,
        "a zero-capacity call should report the size"
    );
    let mut pcm = vec![0.0f32; need];
    let mut got = 0usize;
    assert_eq!(
        unsafe {
            enc(
                a.as_ptr(),
                b.as_ptr(),
                c.as_ptr(),
                freq,
                pcm.as_mut_ptr(),
                pcm.len(),
                &mut got,
            )
        },
        MfskStatus::Ok
    );
    pcm.truncate(got);
    pcm
}

pub fn f32_to_i16(pcm: &[f32]) -> Vec<i16> {
    pcm.iter()
        .map(|&s| (s * 32767.0).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

/// Deterministic white Gaussian noise — the generator of
/// `mfsk-core/tests/common/channel.rs`, which this crate's tests cannot
/// reach, so the same PCG + Box-Muller in a few lines.
pub struct Awgn {
    sigma: f32,
    state: u64,
}

impl Awgn {
    pub fn new(sigma: f32, seed: u64) -> Self {
        Self {
            sigma,
            state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15),
        }
    }

    fn uniform(&mut self) -> f32 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.state >> 32) as f32 + 1.0) / 4_294_967_297.0
    }

    pub fn apply(&mut self, audio: &mut [f32]) {
        for s in audio.iter_mut() {
            let (u1, u2) = (self.uniform(), self.uniform());
            *s += self.sigma * (-2.0 * u1.ln()).sqrt() * (2.0 * std::f32::consts::PI * u2).cos();
        }
    }
}
