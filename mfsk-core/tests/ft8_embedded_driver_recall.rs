//! What `decode_block` — the board's driver, `#[cfg(not(feature =
//! "fft-rustfft"))]` — actually recovers of the fixed WSJT-X/JTDX
//! golden for `qso3_busy.wav`, instead of against a same-build
//! `DecodeRequest` reference.
//!
//! ## Why not `ft8_qso3_apoff_recall.rs`
//!
//! That file already runs this exact ship config
//! (`decode_block(..., DecodeDepth::EMBEDDED, 15)`, `sync_min=1.3`)
//! against the fixed 20-entry `QSO3_KNOWN_REAL_SIGNALS` golden — but
//! it is `#![cfg(feature = "fft-rustfft")]`. `decode_block`'s own
//! `decode_block_multipass` has two bodies split on that exact flag:
//! three passes with subtraction between them under `fft-rustfft`,
//! one pass with none without it. So that test's 14/12 floors, and
//! every other host recall-floor test's numbers, describe the
//! **3-pass driver — not the one any embedded build ships.**
//!
//! ## Why not a same-build `DecodeRequest` "truth" either
//!
//! `ft8_fixed_point_reference_drift.rs` assumed `DecodeRequest`'s
//! output was immune to this because its own code has no
//! `fft-rustfft` cfg branch. It compiles a completely different
//! internal path when `fft-rustfft` is off regardless — measured
//! directly: `DecodeRequest::new(...).decode()` (`max_cand=200`) on
//! `qso3_busy.wav` returns 15 messages under `full`, **8** under
//! `alloc,ft8,fft-extern` — because its default single-pass strategy
//! (`Ft8::__single_pass` -> `decode_frame_inner`) itself gates
//! `apply_wsjtx_xsnr2` and reaches into `fill_symbol_spectra.rs` /
//! `osd_strategy.rs`, both independently `fft-rustfft`-split. A
//! same-build reference is not a fixed reference; this crate's own
//! `#359` thread already said the floors want one, and this is why.
//!
//! So this file measures the ship config against the **external,
//! feature-independent** golden — WSJT-X's own AP-off decode plus
//! JTDX's, reconciled in `common::ft8_qso3::QSO3_KNOWN_REAL_SIGNALS` —
//! reusing the same `#359`/`ft8_decode_block_streaming.rs` pattern of
//! providing `fft-extern`'s contract directly rather than pulling in
//! `mod common` (which needs `fft-rustfft`/`uvpacket` to compile at
//! all). `common::golden`/`common::ft8_qso3` have no such dependency,
//! so they are mounted directly by path instead of duplicated.
//!
//! Run against the board's actual matrix entry:
//! ```sh
//! cargo test -p mfsk-core --release --no-default-features \
//!     --features alloc,ft8,fft-extern,fixed-point,internal-testing \
//!     --test ft8_embedded_driver_recall -- --nocapture
//! ```
//! and its f32 twin (drop `fixed-point`) for the pre-quantisation number.
//!
//! Refs #359, #357.
#![cfg(not(feature = "fft-rustfft"))]

use std::path::Path;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::source::SliceSource;
use mfsk_core::ft8::decode_block::{LowMem, decode_block, decode_block_low_mem};
use mfsk_core::msg::wsjt77::unpack77;

#[path = "common/ft8_qso3.rs"]
mod ft8_qso3;
#[path = "common/golden.rs"]
mod golden;

use ft8_qso3::{DF_TOL_HZ, QSO3_KNOWN_REAL_SIGNALS, SNR_TOL_DB};

macro_rules! asset_path {
    ($asset:literal) => {
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/",
            $asset
        )
    };
}

const QSO3_PATH: &str = asset_path!("qso3_busy.wav");

// The `fft-extern` contract and the WAV loader, shared with
// `ft8_embedded_grid_phase.rs` — see that module's own doc for why a
// host test provides the contract itself.
#[path = "common/embedded_driver_harness.rs"]
mod harness;

use harness::load_wav_i16;

/// Hard-assertion floor for the driver every embedded build actually
/// ships (`decode_block_multipass`'s `not(fft-rustfft)` body) against
/// the fixed, feature-independent golden. Measured 2026-09-06:
///
/// | numeric | recall | phantoms |
/// |---|---:|---:|
/// | f32 | 4/20 | 0 |
/// | fixed-point (what the board runs) | 7/20 | 0 |
///
/// `assert_golden` isn't reused — it also checks per-entry SNR, and no
/// independent SNR reference exists for this driver/golden pairing yet
/// (unlike `ft8_qso3_apoff_recall.rs`'s tolerance-checked SNRs). This
/// stays a plain recall/phantom check until one does.
#[cfg(not(feature = "fixed-point"))]
const MIN_HITS: usize = 4;
#[cfg(feature = "fixed-point")]
const MIN_HITS: usize = 7;
const MAX_EXTRA: usize = 0;

// Floors of `the_low_memory_profile_meets_its_floor` (measured 2026-11-24, `qso3_busy.wav`, of 20).
#[cfg(not(feature = "fixed-point"))]
const MIN_HITS_LOW_MEM_15: usize = 8;
#[cfg(feature = "fixed-point")]
const MIN_HITS_LOW_MEM_15: usize = 9;
const MIN_HITS_LOW_MEM_30: usize = 10;

#[test]
fn true_ship_driver_meets_fixed_golden_floor() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    let decoded = decode_block(&slot, 100.0, 3000.0, 1.3, DecodeDepth::EMBEDDED, 15);
    let msgs: Vec<String> = decoded
        .iter()
        .filter_map(|d| unpack77(d.message77()))
        .collect();

    println!(
        "\ndriver: single-pass, no subtraction (cfg(not(fft-rustfft)))  numeric: {}\n",
        if cfg!(feature = "fixed-point") {
            "fixed-point"
        } else {
            "f32"
        }
    );

    let hits: Vec<&str> = QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .filter(|g| msgs.iter().any(|m| m == g.msg))
        .map(|g| g.msg)
        .collect();
    let missing: Vec<&str> = QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .map(|g| g.msg)
        .filter(|m| !hits.contains(m))
        .collect();
    let phantoms: Vec<&str> = msgs
        .iter()
        .filter(|m| !QSO3_KNOWN_REAL_SIGNALS.iter().any(|g| g.msg == **m))
        .map(|m| m.as_str())
        .collect();

    println!(
        "recall {}/{} (floor {MIN_HITS})  phantoms {} (ceiling {MAX_EXTRA})",
        hits.len(),
        QSO3_KNOWN_REAL_SIGNALS.len(),
        phantoms.len()
    );
    println!("hit:      {hits:?}");
    println!("missing:  {missing:?}");
    println!("phantoms: {phantoms:?}");

    assert!(
        hits.len() >= MIN_HITS,
        "recall regressed: {}/{} decoded, floor is {MIN_HITS}. Missing: {missing:?}",
        hits.len(),
        QSO3_KNOWN_REAL_SIGNALS.len()
    );
    assert!(
        phantoms.len() <= MAX_EXTRA,
        "emitted {} decode(s) outside the golden set, budget is {MAX_EXTRA}. Phantoms: {phantoms:?}",
        phantoms.len()
    );
    // Reserved for when this test also checks per-entry SNR.
    let _ = (DF_TOL_HZ, SNR_TOL_DB);
}

/// The messages `decode_block_low_mem` decodes from `qso3_busy.wav`, as (message, in the golden set).
fn low_mem_messages(max_cand: usize, bands: usize) -> Vec<(String, bool)> {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    decode_block_low_mem(
        &mut SliceSource(&slot[..]),
        100.0,
        3000.0,
        1.3,
        DecodeDepth::EMBEDDED,
        max_cand,
        LowMem::new(bands, 4),
    )
    .iter()
    .filter_map(|d| unpack77(d.message77()))
    .map(|m| {
        let known = QSO3_KNOWN_REAL_SIGNALS.iter().any(|g| g.msg == m);
        (m, known)
    })
    .collect()
}

/// The low-memory profile on the busy slot against the same fixed WSJT-X / JTDX golden as
/// `true_ship_driver_meets_fixed_golden_floor`, at the ship setting (15 candidates) and at 30. It picks its
/// candidates by hard sync (`coarse_sync_gated`) and refines them (`fine_sync_12k`), which `decode_block`
/// does not, so it is held to its own floor, not to `decode_block`'s. Measured 2026-11-25 (without OSD, which
/// the ICAP profile adds on top: `hosttest/icap-app`):
///
/// | max_cand | f32 | fixed-point |
/// |---|---:|---:|
/// | 15 | 8 | 9 |
/// | 30 | 10 | 10 |
///
/// (of 20; `decode_block`: 4 and 7). Zero phantoms: every message must be in the golden set.
#[test]
fn the_low_memory_profile_meets_its_floor() {
    for (max_cand, floor) in [(15usize, MIN_HITS_LOW_MEM_15), (30, MIN_HITS_LOW_MEM_30)] {
        let msgs = low_mem_messages(max_cand, 4);
        let hits = msgs.iter().filter(|m| m.1).count();
        let phantoms: Vec<&str> = msgs.iter().filter(|m| !m.1).map(|m| m.0.as_str()).collect();
        println!("low_mem max_cand {max_cand}: {hits}/20 (floor {floor}), phantoms {phantoms:?}");
        assert!(
            hits >= floor,
            "max_cand {max_cand}: {hits}/20, floor {floor}"
        );
        assert!(
            phantoms.is_empty(),
            "max_cand {max_cand}: phantoms {phantoms:?}"
        );
    }
}

/// The number of bands the spectrogram is built in changes memory and time, never the decodes: same
/// messages, frequencies, DTs and SNRs, to the bit, in the same order.
#[test]
fn the_number_of_bands_changes_no_decode() {
    let slot = load_wav_i16(Path::new(QSO3_PATH));
    for max_cand in [15usize, 30] {
        let run = |bands| {
            decode_block_low_mem(
                &mut SliceSource(&slot[..]),
                100.0,
                3000.0,
                1.3,
                DecodeDepth::EMBEDDED,
                max_cand,
                LowMem::new(bands, 4),
            )
        };
        let a = run(1);
        assert!(!a.is_empty(), "no decodes: the test would prove nothing");
        for bands in [2usize, 3, 8, 40] {
            let b = run(bands);
            assert_eq!(a.len(), b.len(), "max_cand {max_cand}, {bands} bands");
            for (x, y) in a.iter().zip(&b) {
                assert_eq!(
                    x.message77(),
                    y.message77(),
                    "max_cand {max_cand}, {bands} bands"
                );
                assert_eq!(x.freq_hz.to_bits(), y.freq_hz.to_bits());
                assert_eq!(x.dt_sec.to_bits(), y.dt_sec.to_bits());
                assert_eq!(x.snr_db.to_bits(), y.snr_db.to_bits());
            }
        }
    }
}

/// Deterministic Gaussian-ish noise (sum of 12 uniforms), so the test is the same on every machine.
struct Noise(u64);
impl Noise {
    fn next(&mut self) -> f32 {
        let mut a = 0.0f32;
        for _ in 0..12 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            a += ((self.0 >> 40) as f32) / (1u64 << 24) as f32;
        }
        a - 6.0
    }
}

/// A slot of `signals` (message, tone-0 frequency Hz, dt s) at amplitude `amp`, in noise of standard
/// deviation `sigma` per sample (12 kHz, 6 kHz wide: SNR in 2500 Hz = amp^2/2 / (sigma^2 x 2500/6000)).
fn synth_slot(signals: &[(&str, &str, &str, f32, f32)], amp: i16, sigma: f32) -> Vec<i16> {
    use mfsk_core::engine::tx::{message_to_tones, synthesize_i16};
    use mfsk_core::ft8::Ft8;
    use mfsk_core::msg::wsjt77::pack77;
    let mut slot = vec![0f32; 180_000];
    for &(a, b, c, freq, dt) in signals {
        let m77 = pack77(a, b, c).expect("pack");
        let tones = message_to_tones::<Ft8>(&m77);
        let pcm = synthesize_i16::<Ft8>(&tones, 12_000, freq, amp);
        let at = ((0.5 + dt) * 12_000.0).round() as i64;
        for (i, &v) in pcm.iter().enumerate() {
            let idx = at + i as i64;
            if idx >= 0 && (idx as usize) < slot.len() {
                slot[idx as usize] += v as f32;
            }
        }
    }
    let mut n = Noise(0x5EED);
    slot.iter()
        .map(|&v| (v + sigma * n.next()).round().clamp(-32768.0, 32767.0) as i16)
        .collect()
}

/// The 200 Hz baseband path (`cd0_fir`) on signals whose frequency and start are off every grid the code
/// has: the filters must pass the eight tones at the band's low and high ends, and the symbol grid must
/// land on the symbols for any `dt` (a wrong group delay, a wrong decimation phase or a mixer that drifts
/// fails here, not just loses a few dB). Amplitude 3000 in noise of 10 000: about -9 dB in 2500 Hz.
#[test]
fn the_baseband_path_decodes_signals_off_every_grid() {
    let cases: [(&str, &str, &str, f32, f32); 6] = [
        ("CQ", "K1ABC", "FN42", 217.3, -0.45),
        ("K1ABC", "W9XYZ", "-12", 612.8, 0.0),
        ("CQ", "JA1ABC", "PM95", 1234.567, 0.31),
        ("W9XYZ", "K1ABC", "RR73", 1999.99, 1.1),
        ("CQ", "DL1XYZ", "JO62", 2870.4, -0.2),
        ("OH3NIV", "ZS6S", "-03", 2987.2, 0.74),
    ];
    for (i, c) in cases.iter().enumerate() {
        let slot = synth_slot(std::slice::from_ref(c), 3000, 10_000.0);
        let out = decode_block_low_mem(
            &mut SliceSource(&slot[..]),
            100.0,
            3000.0,
            1.3,
            DecodeDepth::EMBEDDED,
            15,
            LowMem::new(3, 4),
        );
        let want = format!("{} {} {}", c.0, c.1, c.2);
        let hit = out
            .iter()
            .find(|d| unpack77(d.message77()).is_some_and(|m| m == want));
        let d = hit
            .unwrap_or_else(|| panic!("case {i}: {want} at {} Hz, dt {}: not decoded", c.3, c.4));
        assert!(
            (d.freq_hz - c.3).abs() < 3.0,
            "case {i}: {} Hz decoded as {}",
            c.3,
            d.freq_hz
        );
        assert!(
            (d.dt_sec - c.4).abs() < 0.05,
            "case {i}: dt {} decoded as {}",
            c.4,
            d.dt_sec
        );
    }
}

/// Three signals 40-45 Hz apart (a real QSO's neighbours), each decoded: the band limit of the baseband path
/// must reject its neighbours, whose tones sit 6-8 tone spacings away.
#[test]
fn the_baseband_path_separates_neighbours() {
    let sig: [(&str, &str, &str, f32, f32); 3] = [
        ("CQ", "K1ABC", "FN42", 1500.0, 0.1),
        ("CQ", "JA1ABC", "PM95", 1545.0, 0.35),
        ("CQ", "DL1XYZ", "JO62", 1585.0, -0.1),
    ];
    let slot = synth_slot(&sig, 3000, 6000.0);
    let out = decode_block_low_mem(
        &mut SliceSource(&slot[..]),
        100.0,
        3000.0,
        1.3,
        DecodeDepth::EMBEDDED,
        15,
        LowMem::new(2, 4),
    );
    let msgs: Vec<String> = out.iter().filter_map(|d| unpack77(d.message77())).collect();
    for m in ["CQ K1ABC FN42", "CQ JA1ABC PM95", "CQ DL1XYZ JO62"] {
        assert!(msgs.iter().any(|x| x == m), "{m} not decoded; got {msgs:?}");
    }
}

/// The baseband path's sensitivity: 24 signals at random frequencies (300-2800 Hz) and starts, one per slot,
/// amplitude 3000 in noise of 26 000 (about -18 dB in 2500 Hz, where FT8's BP/OSD ladder is at its limit): 21
/// decode, in f32 and in fixed-point. Unlike the clean-signal tests this one moves with the filters: with
/// stage 3's cutoff at 40 Hz instead of 102 it is 14, with stage 1's at 5.5 kHz or stage 2's at 1.8 kHz it is
/// 0. (A symbol grid 5 ms late costs one, which is inside what one run can tell: the grid is pinned by
/// `tone_spectra_match_goertzel_on_a_clean_signal` in `cd0_fir.rs` instead.) 22 since the refinement moved
/// onto the baseband (`fine_refine_3stage`); the filters alone gave 21.
#[test]
fn the_baseband_path_keeps_its_sensitivity() {
    let mut n = Noise(7);
    let (mut got, total) = (0usize, 24usize);
    for i in 0..total {
        let f = 300.0 + (n.next().abs() * 400.0) % 2500.0;
        let dt = ((n.next() * 0.3) % 1.0) as f32;
        let call = format!("K{}ABC", i % 10);
        let slot = synth_slot(&[("CQ", &call, "FN42", f, dt)], 3000, 26_000.0);
        let out = decode_block_low_mem(
            &mut SliceSource(&slot[..]),
            100.0,
            3000.0,
            1.3,
            DecodeDepth::EMBEDDED,
            15,
            LowMem::new(3, 4),
        );
        let want = format!("CQ {call} FN42");
        if out
            .iter()
            .any(|d| unpack77(d.message77()).is_some_and(|m| m == want))
        {
            got += 1;
        }
    }
    println!("weak signals decoded: {got}/{total}");
    assert!(got >= 22, "{got}/{total} decoded, floor 22");
}
