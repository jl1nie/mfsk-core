//! `Protocol::DECODE_FFT1_SIZE` must equal the `DownsampleCfg` the
//! decoder actually uses.
//!
//! The constant is written as a literal on each `impl Protocol` rather
//! than read from the `DownsampleCfg`, because those consts live behind
//! an FFT backend feature (`ft4::decode` is
//! `#[cfg(any(fft-rustfft, fft-extern))]`) while the `Protocol` impl is
//! not — a build with `--features ft4` alone has the protocol and no
//! downsampler, so the trait cannot name it.
//!
//! That makes it exactly the kind of hand-copied number this repo has
//! been bitten by before, which is what this file is for. It compiles
//! only with a backend present, and there it holds both halves next to
//! each other.
//!
//! The value is published through `ProtocolMeta::decode_fft1_size` so a
//! C or mobile caller can size a buffer, or decide which modes it can
//! afford at all: FT4 takes 92 160 points and FST4-300 takes 4 194 304,
//! a factor of 45 that no other registry field hints at.
#![cfg(all(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use mfsk_core::engine::protocol::Protocol;
use mfsk_core::fst4::{Fst4s15, Fst4s30, Fst4s60, Fst4s120, Fst4s300};
use mfsk_core::{Ft4, Ft8, PROTOCOLS};

#[test]
fn the_literal_matches_the_downsample_config() {
    use mfsk_core::fst4::decode as f;

    use mfsk_core::fst4w::decode as w;
    use mfsk_core::fst4w::{Fst4w120, Fst4w300, Fst4w900, Fst4w1800};

    let pairs: [(&str, u32, usize); 11] = [
        (
            "FT8",
            Ft8::DECODE_FFT1_SIZE,
            mfsk_core::ft8::downsample::FT8_CFG.fft1_size,
        ),
        (
            "FT4",
            Ft4::DECODE_FFT1_SIZE,
            mfsk_core::ft4::decode::FT4_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4-15",
            Fst4s15::DECODE_FFT1_SIZE,
            f::FST4_15_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4-30",
            Fst4s30::DECODE_FFT1_SIZE,
            f::FST4_30_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4-60A",
            Fst4s60::DECODE_FFT1_SIZE,
            f::FST4_60A_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4-120",
            Fst4s120::DECODE_FFT1_SIZE,
            f::FST4_120_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4-300",
            Fst4s300::DECODE_FFT1_SIZE,
            f::FST4_300_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4W-120",
            Fst4w120::DECODE_FFT1_SIZE,
            f::FST4_120_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4W-300",
            Fst4w300::DECODE_FFT1_SIZE,
            f::FST4_300_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4W-900",
            Fst4w900::DECODE_FFT1_SIZE,
            w::FST4W_900_DOWNSAMPLE.fft1_size,
        ),
        (
            "FST4W-1800",
            Fst4w1800::DECODE_FFT1_SIZE,
            w::FST4W_1800_DOWNSAMPLE.fft1_size,
        ),
    ];

    for (name, declared, actual) in pairs {
        assert_eq!(
            declared as usize, actual,
            "{name}: Protocol::DECODE_FFT1_SIZE says {declared} but its DownsampleCfg \
             takes a {actual}-point transform — the literal has drifted from the decoder"
        );
    }
}

/// Every mode the decode handle drives must publish a real size, and
/// every mode that does not use the shared downsampler must publish 0
/// rather than a plausible-looking guess.
#[test]
fn the_registry_publishes_it_exactly_where_it_applies() {
    use mfsk_core::registry::caps;

    for p in PROTOCOLS {
        // FST4W has its own ladder but FST4's whole-slot transform, so it
        // publishes the size too.
        let drives_handle =
            p.profile.caps & caps::DECODE_HANDLE != 0 || p.name.starts_with("FST4W-");
        if drives_handle {
            assert!(
                p.decode_fft1_size > 0,
                "{} drives the decode handle but publishes no FFT size",
                p.name
            );
        } else {
            assert_eq!(
                p.decode_fft1_size, 0,
                "{} has its own decode front end, so it must publish 0 rather than \
                 a number a caller would size a buffer from",
                p.name
            );
        }
    }
}

/// The spread is the point — a caller that budgets for FT4 and then
/// runs FST4-300 is off by a factor of 45, and FST4W-1800 by 234.
#[test]
fn the_spread_across_modes_is_published() {
    let sizes: Vec<u32> = PROTOCOLS
        .iter()
        .filter(|p| p.decode_fft1_size > 0)
        .map(|p| p.decode_fft1_size)
        .collect();
    let (min, max) = (
        *sizes.iter().min().expect("some mode decodes"),
        *sizes.iter().max().expect("some mode decodes"),
    );
    assert_eq!(min, 92_160, "FT4 is the cheapest slot transform");
    assert_eq!(
        max, 21_591_360,
        "FST4W-1800 is the most expensive (6426 x 3360, upstream's nfft1)"
    );
    assert!(max / min >= 234);
}
