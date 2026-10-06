//! One decode per mode, on silence, on `wasm32-unknown-unknown`.
//!
//! Silence is enough: #583 was a `std::time::Instant::now()` panic in code
//! every decode runs, not in anything a signal reaches. The check is that each
//! call *returns*; what it decodes is the host tiers' business.

use mfsk_core::decoder::AnyDecoder;
use mfsk_core::registry::{self, Mode};
use wasm_bindgen::prelude::*;

/// Registry names of every mode this build carries.
#[wasm_bindgen]
pub fn modes() -> Vec<String> {
    Mode::ALL.iter().map(|m| m.name().to_string()).collect()
}

/// Decode one silent slot of `name`; returns the row count.
#[wasm_bindgen]
pub fn decode_silence(name: &str) -> usize {
    let mode = *Mode::ALL
        .iter()
        .find(|m| m.name() == name)
        .expect("unknown mode");
    let n = registry::by_name(name).expect("not in the registry").slot_samples_12k as usize;
    AnyDecoder::with_defaults(mode)
        .decode_i16(&vec![0i16; n], None)
        .rows
        .len()
}

/// MSK144 sits outside the registry (no `Protocol` ZST); 1 s of a quiet tone
/// so the scan runs both of its stages.
#[wasm_bindgen]
pub fn msk144() -> usize {
    let a: Vec<i16> = (0..12_000)
        .map(|i| ((i as f32 * 0.37).sin() * 300.0) as i16)
        .collect();
    mfsk_core::msk144::decode::decode_slot(&a, 1500.0, 100.0, mfsk_core::msk144::decode::Depth::default()).len()
}
