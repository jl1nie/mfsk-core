// SPDX-License-Identifier: GPL-3.0-only
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

/// The host's side of the CoreS3 JTTY mode's SIM feed (#499, E1): the same
/// recordings `apps::jtty`'s `MFSK_CORES3_SIM` loops through the board's real
/// sink — `uac::spawn_sim_feed_continuous`, the whole recording end to start
/// with no slot re-alignment — decoded here by `Params::embedded()` in two
/// halves with nothing dropped. When the board drops no window, its completed
/// messages should be these (`docs/notes/JTTY_CORES3_APP.md` §7).
///
/// ```text
/// cargo test -p mfsk-core --release --features full --test jtty_board_patterns -- --ignored --nocapture sim_streams
/// ```
#[test]
#[ignore = "prints; compare with the board's SIM log"]
fn sim_streams_looped_on_the_host() {
    const PASSES: usize = 3;
    let rx = Arc::new(Receiver::new().with_f32_metrics());
    let params = Params::default().embedded();
    let golden = std::fs::read(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../embedded-poc/assets/golden/jtty/260807_134110.wav"
    ))
    .expect("the golden JTTY recording is vendored");
    // the board skips exactly the 44-byte header (`SimSource::Wav`)
    let golden: Vec<i16> = golden[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|&b| i16::from_le_bytes(b))
        .collect();
    let band6 = pileups(1)
        .into_iter()
        .find(|c| c.pattern.starts_with("band, 6 long messages"))
        .expect("a pileups pattern")
        .audio()
        .expect("packs");
    for (scene, one) in [("golden", golden), ("band6", band6)] {
        let stream: Vec<i16> = (0..PASSES).flat_map(|_| one.iter().copied()).collect();
        let mut front = Front::new(rx.clone(), params).expect("embedded settings");
        let mut back = Back::new(rx.clone(), params);
        let mut done: Vec<(f64, f32, String)> = Vec::new();
        let mut on = |u: mfsk_core::jtty::assemble::MessageUpdate| {
            if u.complete {
                done.push((u.start_s, u.f1_hz, u.text));
            }
        };
        // the SIM feed's own block size
        for chunk in stream.chunks(256) {
            let mut ready = Vec::new();
            front.push(chunk, &mut |p| ready.push(p));
            for p in ready {
                back.process(p, &mut on);
            }
        }
        back.finish(&mut on);
        println!(
            "SIM\t{scene}\t{} s a pass\t{PASSES} passes\t{} messages",
            one.len() / 12_000,
            done.len()
        );
        for (start, f, text) in &done {
            println!("SIM\t{scene}\t{start:>8.2} s\t{f:>7.1} Hz\t{text}");
        }
    }
}
