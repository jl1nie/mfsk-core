// SPDX-License-Identifier: GPL-3.0-only
//! `coarse_sync_banded` is `coarse_sync` on a spectrogram built a band of carriers at a time: the same
//! candidates, to the bit, for any number of bands. Tier A (invariant), on real recordings: the busy slot
//! `qso3_busy.wav` and ft8_lib's `websdr_test1.wav`.
//!
//! Why it holds, and so what would break it: a band's cell is the full spectrogram's cell (same frame FFT,
//! same auto-gain shift in the fixed-point build), a carrier's score reads only the bins `i..=i + 14`, and
//! the 40th-percentile noise floor is taken over every carrier's peak after all bands are scored. A change
//! that lets a carrier's score depend on another carrier (a per-spectrogram normalisation, say) fails here.
//!
//! Runs in both numeric builds (`full` and `full,fixed-point` on the host's rustfft stub):
//! ```sh
//! cargo test -p mfsk-core --release --features full,internal-testing --test ft8_coarse_banded
//! cargo test -p mfsk-core --release --features full,internal-testing,fixed-point --test ft8_coarse_banded
//! ```
#![cfg(feature = "fft-rustfft")]

#[allow(dead_code)]
mod common;
use common::load_wav_i16;

use mfsk_core::ft8::decode_block::source::{CountingSource, SliceSource};
use mfsk_core::ft8::decode_block::{
    HARD_SYNC_POOL, LowMem, coarse_sync, coarse_sync_banded, coarse_sync_gated,
    coarse_sync_gated_spec, coarse_sync_gated_src, compute_spectrogram, decode_block_low_mem,
    decode_block_low_mem_src,
};

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

fn bits(c: &[mfsk_core::engine::sync::SyncCandidate]) -> Vec<(u32, u32, u32)> {
    c.iter()
        .map(|k| (k.freq_hz.to_bits(), k.dt_sec.to_bits(), k.score.to_bits()))
        .collect()
}

#[test]
fn banded_coarse_sync_equals_the_full_spectrogram() {
    for wav in [
        asset_path!("qso3_busy.wav"),
        asset_path!("websdr_test1.wav"),
    ] {
        let slot = load_wav_i16(wav);
        let spec = compute_spectrogram(&slot, 3000.0);
        // the ship settings (15 kept), and a long list with a low threshold, where the dedupe and the
        // percentile floor have the most to disagree about
        for (sync_min, max_cand) in [(1.3f32, 15usize), (0.8, 300)] {
            let full = coarse_sync(&spec, 200.0, 3000.0, sync_min, max_cand);
            assert!(
                full.len() >= 10,
                "{wav}: only {} candidates, the test would prove little",
                full.len()
            );
            for bands in [1usize, 2, 3, 7, 40] {
                let banded = coarse_sync_banded(&slot, 200.0, 3000.0, sync_min, max_cand, bands);
                assert_eq!(
                    bits(&full),
                    bits(&banded),
                    "{wav}, {bands} bands, sync_min {sync_min}, max_cand {max_cand}"
                );
            }
        }
    }
}

/// `coarse_sync_gated` reads each candidate's hard-sync count from the band's spectrogram, so the count is
/// the one test that could depend on which band a carrier fell in. It must not: the same candidates with the
/// same counts for any number of bands. And what it promises: the list is by count, best first, the counts
/// are never more than 21, and the candidates are the coarse list's (the first `HARD_SYNC_POOL` by score),
/// reordered.
#[test]
fn gated_coarse_sync_is_the_same_for_any_number_of_bands_and_is_a_reordering() {
    let slot = load_wav_i16(asset_path!("qso3_busy.wav"));
    let spec = compute_spectrogram(&slot, 3000.0);
    let pool = coarse_sync(&spec, 200.0, 3000.0, 1.3, HARD_SYNC_POOL);
    for keep in [15usize, 30] {
        let one = coarse_sync_gated(&slot, 200.0, 3000.0, 1.3, keep, 1);
        assert_eq!(one.len(), keep);
        assert!(
            one.windows(2).all(|w| w[0].1 >= w[1].1),
            "not by hard sync, best first"
        );
        assert!(one.iter().all(|c| c.1 <= 21));
        assert!(
            one[0].1 >= 15,
            "the best candidate of a busy slot has {} of 21",
            one[0].1
        );
        for c in &one {
            assert!(
                pool.iter()
                    .any(|p| bits(std::slice::from_ref(p)) == bits(std::slice::from_ref(&c.0))),
                "a gated candidate that is not in the coarse pool"
            );
        }
        for bands in [2usize, 3, 7, 40] {
            let b = coarse_sync_gated(&slot, 200.0, 3000.0, 1.3, keep, bands);
            let key =
                |v: &[(mfsk_core::engine::sync::SyncCandidate, u8)]| -> Vec<((u32, u32, u32), u8)> {
                    v.iter()
                        .map(|c| (bits(std::slice::from_ref(&c.0))[0], c.1))
                        .collect()
                };
            assert_eq!(key(&one), key(&b), "keep {keep}, {bands} bands");
        }
    }
}

/// The coarse search from an `AudioSource`, as the radio runs it: the same candidates as from the slice,
/// to the bit, and the reads are the ones it can afford: forward only (a restart per band, no more) and a
/// few passes' worth of bytes. Counted with the radio's cost model (`CountingSource`): an 8 kHz slot is 233 KB,
/// and the number printed is that times the passes (one per band, plus one for the fixed-point build's gain).
#[test]
fn gated_coarse_sync_from_a_source_is_the_slice_version_and_reads_forward() {
    let slot = load_wav_i16(asset_path!("qso3_busy.wav"));
    for bands in [1usize, 4] {
        let want = coarse_sync_gated(&slot, 200.0, 3000.0, 1.3, 30, bands);
        let mut src = CountingSource::new(SliceSource(&slot[..]));
        let got = coarse_sync_gated_src(&mut src, 200.0, 3000.0, 1.3, 30, bands);
        let key =
            |v: &[(mfsk_core::engine::sync::SyncCandidate, u8)]| -> Vec<((u32, u32, u32), u8)> {
                v.iter()
                    .map(|c| (bits(std::slice::from_ref(&c.0))[0], c.1))
                    .collect()
            };
        assert_eq!(key(&want), key(&got), "{bands} bands");
        let c = src.cost();
        let passes = bands + usize::from(cfg!(feature = "fixed-point"));
        println!(
            "{bands} bands: {} KB moved in {} calls, {} restarts ({passes} passes of {} KB)",
            c.bytes_moved() / 1000,
            c.calls,
            c.restarts,
            2 * slot.len() / 1000
        );
        // each pass is one forward sweep: a restart per pass after the first, nothing skipped
        assert_eq!(c.restarts as usize, passes - 1);
        assert_eq!(
            c.samples_skipped as usize, 0,
            "a pass skipped samples it did not need to"
        );
        // a pass moves the slot once, plus at most a block of read-ahead
        assert!(
            c.samples_read as usize <= passes * (slot.len() + 2048),
            "{c:?}"
        );
    }
}

/// The gate on a spectrogram the caller already holds (`stage1_inc`'s, on the boards) is the gate on the audio:
/// the same candidates in the same order with the same hard-sync counts, to the bit, on a whole slot. A gate
/// that read anything the spectrogram does not hold (the audio, another band's cells) or counted differently
/// would differ.
#[test]
fn gated_coarse_sync_from_a_spectrogram_is_the_audio_version() {
    for wav in [
        asset_path!("qso3_busy.wav"),
        asset_path!("websdr_test1.wav"),
    ] {
        let slot = load_wav_i16(wav);
        let spec = compute_spectrogram(&slot, 3000.0);
        for (sync_min, keep) in [(1.3f32, 15usize), (1.0, 30), (0.8, 300)] {
            let from_audio = coarse_sync_gated(&slot, 200.0, 3000.0, sync_min, keep, 4);
            let from_spec =
                coarse_sync_gated_spec(&spec, 200.0, 3000.0, sync_min, keep, None, None);
            assert!(
                from_audio.len() >= 10,
                "{wav}: only {} candidates",
                from_audio.len()
            );
            assert_eq!(from_audio.len(), from_spec.len(), "{wav} {sync_min}");
            for (a, b) in from_audio.iter().zip(&from_spec) {
                assert_eq!(
                    bits(std::slice::from_ref(&a.0)),
                    bits(std::slice::from_ref(&b.0))
                );
                assert_eq!(a.1, b.1, "hard-sync count");
            }
        }
    }
}

/// `decode_block_low_mem` is the low-memory decode in **every** build: here, with `fft-rustfft` (where `decode_block` is
/// the host's 3-pass decoder), it gives what `decode_block_low_mem_src` gives over a `SliceSource`, not `decode_block`'s.
/// A host body that ignored the low-memory request (it did, once) would give the 3-pass decoder's messages and frequencies
/// instead, and differ.
#[test]
fn the_low_memory_decode_is_the_same_decode_in_every_build() {
    use mfsk_core::ft8::decode::DecodeDepth;
    let slot = load_wav_i16(asset_path!("qso3_busy.wav"));
    let key = |r: &[mfsk_core::ft8::decode::DecodeResult]| -> Vec<(Vec<u8>, u32, u32)> {
        r.iter()
            .map(|d| {
                (
                    d.message77().to_vec(),
                    d.freq_hz.to_bits(),
                    d.dt_sec.to_bits(),
                )
            })
            .collect()
    };
    let a = decode_block_low_mem(&slot, 200.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15, 4);
    let b = decode_block_low_mem_src(
        &mut SliceSource(&slot[..]),
        200.0,
        3000.0,
        1.3,
        DecodeDepth::EMBEDDED,
        15,
        LowMem {
            coarse_bands: 4,
            ..LowMem::default()
        },
    );
    assert!(!a.is_empty(), "the busy slot decodes something");
    assert_eq!(key(&a), key(&b));
}
