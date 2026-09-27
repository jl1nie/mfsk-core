// SPDX-License-Identifier: GPL-3.0-or-later
//! The host's side of the CoreS3's pattern run (#499): the same `jtty::testsig::catalogue`
//! recordings through the embedded receiver (`Params::embedded()`, `f32` metrics) in two halves,
//! one `CASE` line each in the board's format, for `scripts/jtty_board_stats.py` to compare.
//!
//! ```text
//! cargo test -p mfsk-core --release --features full --test jtty_board_patterns -- --ignored --nocapture
//! ```

#![cfg(all(feature = "jtty", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::sync::Arc;

use mfsk_core::jtty::rx::{Back, Front, Params, Receiver};
use mfsk_core::jtty::testsig::catalogue;

/// Trials of each pattern; the board runs the same number.
pub const TRIALS: u32 = 10;

#[test]
#[ignore = "prints; compare with the board's log"]
fn board_patterns_on_the_host() {
    let rx = Arc::new(Receiver::new().with_f32_metrics());
    let params = Params::default().embedded();
    for case in catalogue(TRIALS) {
        let audio = case.audio().expect("packs");
        let mut front = Front::new(rx.clone(), params).expect("embedded settings");
        let mut back = Back::new(rx.clone(), params);
        let mut done: Vec<String> = Vec::new();
        let mut on = |u: mfsk_core::jtty::assemble::MessageUpdate| {
            if u.complete {
                done.push(u.text);
            }
        };
        let mut ready = Vec::new();
        front.push(&audio, &mut |p| ready.push(p));
        for p in ready {
            back.process(p, &mut on);
        }
        back.finish(&mut on);
        done.sort();
        println!("CASE\t{}\t{}\t{}", case.pattern, case.trial, done.join("|"));
    }
}
