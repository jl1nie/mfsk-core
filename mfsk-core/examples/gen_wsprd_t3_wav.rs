// SPDX-License-Identifier: GPL-3.0-only
//! Write the synthetic Type 3 recording of `tests/common/wspr_t3.rs` as a WAV, for
//! `scripts/wsprd/gen_fixtures.sh` to run `wsprd` on:
//!
//! ```text
//! cargo run --release --example gen_wsprd_t3_wav --features full,internal-testing -- 250101_0000.wav
//! ```

#[path = "../tests/common/wspr_t3.rs"]
mod wspr_t3;

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: gen_wsprd_t3_wav <out.wav> [far]");
    let far = std::env::args().nth(2).as_deref() == Some("far");
    let samples = if far {
        wspr_t3::build_far()
    } else {
        wspr_t3::build()
    };
    std::fs::write(&path, wspr_t3::wav_bytes(&samples)).expect("write");
}
