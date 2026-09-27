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
use mfsk_core::jtty::testsig::{catalogue, pileups};

/// Trials of each pattern; the board runs the same number.
pub const TRIALS: u32 = 10;

#[test]
#[ignore = "prints; compare with the board's log"]
fn board_patterns_on_the_host() {
    let rx = Arc::new(Receiver::new().with_f32_metrics());
    // `MFSK_JTTY_BUDGET=n` (0: none) and `MFSK_JTTY_UPSTREAM_PARAMS=1` (rjtty's settings) to
    // compare against the embedded ones
    let mut params = if std::env::var_os("MFSK_JTTY_UPSTREAM_PARAMS").is_some() {
        Params::default()
    } else {
        Params::default().embedded()
    };
    if let Some(b) = std::env::var("MFSK_JTTY_BUDGET")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
    {
        params.ladder_budget = (b > 0).then_some(b);
    }
    // `MFSK_JTTY_SIDE_BUDGET=n`: the side channels' own ladder budget
    if let Some(b) = std::env::var("MFSK_JTTY_SIDE_BUDGET")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
    {
        params.side_ladder_budget = Some(b);
    }
    // `MFSK_JTTY_SUB=1`: subtract channel 0's frames within their window (no retro sweep)
    if std::env::var_os("MFSK_JTTY_SUB").is_some() {
        params.subtract = true;
        params.retro_sweep = false;
        params.subtract_side_channels = false;
    }
    if let Some(w) = std::env::var("MFSK_JTTY_SKIP_HZ")
        .ok()
        .and_then(|s| s.parse().ok())
    {
        params.skip_decoded_hz = w;
    }
    // `MFSK_JTTY_PATTERNS=pileups`: the pileup and busy-band patterns instead
    let cases = if std::env::var("MFSK_JTTY_PATTERNS").as_deref() == Ok("pileups") {
        pileups(TRIALS)
    } else {
        catalogue(TRIALS)
    };
    for case in cases {
        #[cfg(feature = "jtty-stats")]
        let before = rx.stats();
        let audio = case.audio().expect("packs");
        let mut done: Vec<String> = Vec::new();
        let mut on = |u: mfsk_core::jtty::assemble::MessageUpdate| {
            if u.complete {
                done.push(u.text);
            }
        };
        match Front::new(rx.clone(), params) {
            Some(mut front) => {
                let mut back = Back::new(rx.clone(), params);
                let mut ready = Vec::new();
                front.push(&audio, &mut |p| ready.push(p));
                for p in ready {
                    back.process(p, &mut on);
                }
                back.finish(&mut on);
            }
            // settings with subtraction need the whole stream in one place
            None => {
                let mut stream = mfsk_core::jtty::rx::Stream::new(rx.clone(), params);
                stream.push(&audio, &mut on);
                stream.finish(&mut on);
            }
        }
        done.sort();
        println!("CASE\t{}\t{}\t{}", case.pattern, case.trial, done.join("|"));
        #[cfg(feature = "jtty-stats")]
        {
            let a = rx.stats();
            println!(
                "CALLS\t{}\t{}\t{}\t{}",
                case.pattern,
                case.trial,
                a.count(mfsk_core::jtty::stats::Counter::LadderCalls)
                    - before.count(mfsk_core::jtty::stats::Counter::LadderCalls),
                a.rungs[2] - before.rungs[2]
            );
        }
    }
    // ladder calls and the rungs they ran, over every case
    #[cfg(feature = "jtty-stats")]
    {
        use mfsk_core::jtty::stats::Counter;
        let s = rx.stats();
        println!(
            "LADDER\tcalls {}\trungs {:?}",
            s.count(Counter::LadderCalls),
            s.rungs
        );
    }
}
