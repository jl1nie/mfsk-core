//! `DecodeRequest::fft_cache` — FFT-cache-reuse verification for FT4.
//!
//! Companion to `ft4_streaming_decode.rs`, same root cause: `fft_cache`
//! is a field on the *shared* `DecodeRequest<P>` struct, so
//! `.fft_cache(cache)` type-checks and compiles for `DecodeRequest<Ft4>`
//! even though — until this fix — `Ft4`'s `FrameDecodable::
//! __single_pass`/`SupportsSicRounds::__flat_sic` never threaded
//! `req.fft_cache` down into `engine::pipeline::decode_frame`/
//! `decode_frame_subtract`, which always rebuilt from `audio` instead.
//! Unlike `on_result`, a silently-ignored `fft_cache` doesn't break
//! *correctness* (the rebuilt-from-`audio` cache is numerically
//! identical to the "correct" supplied one in the common case) — it
//! only wastes the FFT rebuild `fft_cache(..)` exists to avoid. So a
//! same-audio round-trip test can't distinguish "reused" from
//! "silently rebuilt": both give the same answer.
//!
//! Proof here is a **differential test**: build a real (but wrong)
//! `FftCache` from a *different* buffer (silence, same length), hand
//! it to `.fft_cache(..)` while decoding the *real* golden audio. If
//! the cache is actually being consumed, the wrong frequency-domain
//! content poisons every downstream downsample/LLR step and the
//! golden message must NOT come back. If it's silently ignored (the
//! pre-fix bug), the decode rebuilds its own correct cache from the
//! real audio internally and the golden message comes back anyway —
//! which is exactly the false-negative this test is designed to catch.
//!
//! Skipped when the WSJT-X tree is not present at the expected
//! sibling path.

#![cfg(all(feature = "ft4", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::PathBuf;

use mfsk_core::ft4::Ft4;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000; // 7.5 s × 12 kHz
const GOLDEN_MSG: &str = "N1TRK N4FKH 569 VA"; // see ft4_wsjtx_samples.rs

fn load_slot() -> Option<Vec<i16>> {
    let path: PathBuf = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = read_wsjtx_wav_i16(&path)?;
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

fn recovers_golden(results: &[mfsk_core::ft4::decode::DecodeResult]) -> bool {
    results
        .iter()
        .filter_map(|r| unpack77(r.message77()))
        .any(|m| m == GOLDEN_MSG)
}

#[test]
fn ft4_fft_cache_override_actually_used_single_pass() {
    let Some(audio) = load_slot() else {
        eprintln!(
            "skipping: WSJT-X FT4 sample not found at ../../WSJT-X/samples/FT4/000000_000002.wav"
        );
        return;
    };

    // Sanity: the uncached decode must recover the golden message —
    // otherwise the differential check below is meaningless.
    let plain = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .decode()
        .results;
    assert!(
        recovers_golden(&plain),
        "expected the uncached decode to recover '{GOLDEN_MSG}'"
    );

    // A real (correctly-shaped) but *wrong* FftCache, built from silence.
    let silence = vec![0i16; SLOT_SAMPLES];
    let wrong_cache = DecodeRequest::<Ft4>::new(&silence, 100.0, 2700.0, 0.05, 100)
        .decode()
        .fft_cache;

    let poisoned = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .fft_cache(wrong_cache)
        .decode()
        .results;
    assert!(
        !recovers_golden(&poisoned),
        "'.fft_cache(wrong_cache)' must actually be consumed: decoding real audio \
         against a cache built from silence still recovered '{GOLDEN_MSG}', which \
         means the supplied cache was silently ignored and the engine rebuilt its \
         own (correct) cache from `audio` instead — the exact bug this test guards \
         against. decoded: {:?}",
        poisoned
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .collect::<Vec<_>>()
    );
}

#[test]
fn ft4_fft_cache_override_actually_used_flat_sic() {
    let Some(audio) = load_slot() else {
        eprintln!(
            "skipping: WSJT-X FT4 sample not found at ../../WSJT-X/samples/FT4/000000_000002.wav"
        );
        return;
    };

    // sic_rounds(1) deliberately, not 3: with multiple rounds, a
    // poisoned round-0 cache that causes round 0 to accept nothing just
    // leaves the residual untouched, and round 1/2 (which always
    // rebuild their own cache from the *mutated* residual — see
    // `decode_frame_subtract`'s `(pass_idx, precomputed_fft)` match,
    // only `pass_idx == 0` ever consults `precomputed_fft`) pick every
    // signal back up with a correctly-built cache, self-healing the
    // very effect this test wants to observe. A single round removes
    // that safety net.
    let plain = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .sic_rounds(1)
        .decode()
        .results;
    assert!(
        recovers_golden(&plain),
        "expected the uncached SIC (1 round) decode to recover '{GOLDEN_MSG}'"
    );

    let silence = vec![0i16; SLOT_SAMPLES];
    let wrong_cache = DecodeRequest::<Ft4>::new(&silence, 100.0, 2700.0, 0.05, 100)
        .decode()
        .fft_cache;

    let poisoned = DecodeRequest::<Ft4>::new(&audio, 100.0, 2700.0, 0.05, 100)
        .sic_rounds(1)
        .fft_cache(wrong_cache)
        .decode()
        .results;
    assert!(
        !recovers_golden(&poisoned),
        "'.fft_cache(wrong_cache)' must actually be consumed on the flat-SIC \
         (round-0) path too — decoding real audio against a cache built from \
         silence still recovered '{GOLDEN_MSG}'. decoded: {:?}",
        poisoned
            .iter()
            .filter_map(|r| unpack77(r.message77()))
            .collect::<Vec<_>>()
    );
}
