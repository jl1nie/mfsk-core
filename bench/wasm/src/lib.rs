//! Thin wasm-bindgen wrapper around mfsk-core's FT8 decode entry point,
//! for the Node-based `+simd128` before/after benchmark. Not a real API
//! surface — see `bench/wasm/README.md`.

use mfsk_core::ft8::Ft8;
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::unpack77;
use wasm_bindgen::prelude::*;

/// Decode one FT8 slot from mono 16-bit PCM samples.
///
/// Mirrors `mfsk-core/tests/ft8_sweep.rs`'s `decode_wav_ft8` call shape
/// (`DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50)`, OSD on by
/// default) so the benchmark exercises the same pipeline the crate's own
/// tests do.
///
/// Returns one `message|freq_hz|dt_sec` line per decode, newline-joined —
/// good enough for `bench.mjs` to print/diff, not a real API contract.
#[wasm_bindgen]
pub fn decode_wav(audio: &[i16]) -> String {
    let outcome = DecodeRequest::<Ft8>::new(audio, 100.0, 3000.0, 0.8, 50).decode();

    outcome
        .results
        .iter()
        .map(|d| {
            let msg = unpack77(d.message77()).unwrap_or_default();
            format!("{msg}|{:.1}|{:.2}", d.freq_hz, d.dt_sec)
        })
        .collect::<Vec<_>>()
        .join("\n")
}
