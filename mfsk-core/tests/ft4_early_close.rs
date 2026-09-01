// SPDX-License-Identifier: GPL-3.0-or-later
//! Can an FT4 receiver close its capture window when the *frame's*
//! audio is complete, rather than when the slot is?
//!
//! ## Why this is the question a QSO-capable build has to answer
//!
//! FT4 exists for fast QSOs, so the deadline is not the end of the
//! slot — it is the moment this station has to key up. Within a 7.5 s
//! slot beginning at 0:
//!
//! ```text
//!   0.50 s  the other station's transmission starts
//!   5.54 s  its frame ends (105 symbols x 48 ms)
//!   6.04 s  ...plus the +0.5 s of DT the search window allows:
//!           every sample the decoder will read has now arrived
//!   7.50 s  slot boundary
//!   8.00 s  THIS station's transmission must start
//! ```
//!
//! So the decode window is **6.04 → 8.00 s**, 1.96 s wide. A receiver
//! that waits for the slot boundary at 7.50 s and then spends 1.96 s
//! answers at 9.46 s — 1.46 s after it needed to be transmitting. The
//! width is right and the anchor is not, and 1.46 s of that anchor is
//! spent waiting for audio the decoder never reads: `ft4_sync_search`'s
//! window tops out at `i0 = 667` and a frame is 105 x 32 = 3 360
//! downsampled samples, so 4 027 of the 5 000 in a slot.
//!
//! ## What could go wrong, and is what this file measures
//!
//! Truncating the slot is not free by construction:
//!
//! - **The RMS normalisation.** `process_candidate_basic_impl`
//!   normalises `cd0` over its whole 5 120 samples and `LLR_SCALE` is
//!   calibrated against that. Fewer real samples and more zero padding
//!   is less noise power in the same divisor, so every LLR feeding BP
//!   rescales — `sqrt(5120/4027)` ≈ 1.13 if the padding were free.
//! - **The coarse baseline.** `Ft4SavgBuilder` averages ~152 rows over
//!   a slot and `getcandidates4`'s threshold is relative to a fitted
//!   baseline, so a shorter average moves the scores `SYNC_MIN = 1.2`
//!   is compared against.
//!
//! Both are measured here rather than reasoned about, on the WSJT-X
//! golden recording, through the same pipeline `ft4_rx::decode_slot`
//! runs on the board.
//!
//! ```sh
//! MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core \
//!   --features full,internal-testing --release \
//!   --test ft4_early_close -- --nocapture
//! ```

#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use num_complex::Complex;

use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::ft4_coarse::{Ft4SavgBuilder, ft4_coarse_sync, ft4_coarse_sync_from_savg};
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync2d::ft4_sync_search_window;
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::ddc::{candidate_baseband_half, decimate_slot};
use mfsk_core::ft4::decode::FT4_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

#[allow(dead_code)]
mod common;
use common::load_wav_i16_opt as read_wsjtx_wav_i16;

const SLOT_SAMPLES: usize = 90_000;
const FREQ_MIN_HZ: f32 = 100.0;
const FREQ_MAX_HZ: f32 = 2700.0;
const SYNC_MIN: f32 = 1.2;
const MAX_CAND: usize = 100;
const SYNC_Q_MIN: u32 = 8;
const NARROW_WINDOW: (i32, i32) = (0, 667);

/// The last input sample `ft4_sync_search_window` can reach:
/// `(i0_max + 105 symbols x 32) x NDOWN`.
const NEEDED_SAMPLES: usize = (667 + 105 * 32) * 18;

fn slot_audio() -> Option<Vec<i16>> {
    let path = common::corpus::golden_path_or_upstream(
        "ft4/000000_000002.wav",
        Some("FT4/000000_000002.wav"),
    )?;
    let raw = read_wsjtx_wav_i16(&path).expect("WAV must be 12 kHz mono PCM-16");
    let mut audio = vec![0i16; SLOT_SAMPLES];
    let copy = raw.len().min(SLOT_SAMPLES);
    audio[..copy].copy_from_slice(&raw[..copy]);
    Some(audio)
}

fn rms_normalise(cd0: &mut [Complex<f32>]) {
    let sum2: f32 = cd0.iter().map(|c| c.norm_sqr()).sum::<f32>() / cd0.len() as f32;
    if sum2 > f32::EPSILON {
        let inv = 1.0 / sum2.sqrt();
        for c in cd0.iter_mut() {
            *c *= inv;
        }
    }
}

/// `ft4_rx::decode_slot`'s pipeline, over however much audio it is
/// given: coarse from the same samples, shared decimation, half-rate
/// per-candidate DDC, narrowed Δt search, `EMBEDDED` depth.
fn decode(audio: &[i16]) -> (usize, Vec<String>) {
    let cands = ft4_coarse_sync(audio, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    let half = decimate_slot(audio);
    let mut msgs: Vec<String> = Vec::new();
    for c in &cands {
        let mut cd0 = candidate_baseband_half(&half, c.freq_hz);
        rms_normalise(&mut cd0);
        let s2 = ft4_sync_search_window::<Ft4>(&cd0, c, NARROW_WINDOW.0, NARROW_WINDOW.1);
        let Some(r) = process_candidate_precomputed::<Ft4>(
            c,
            &[],
            &FT4_DOWNSAMPLE,
            DecodeDepth::EMBEDDED,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            (cd0, s2.freq_hz, s2.i0, s2.score),
            false,
            false,
        ) else {
            continue;
        };
        if let Some(t) = unpack77(r.message77())
            && !msgs.contains(&t)
        {
            msgs.push(t);
        }
    }
    msgs.sort();
    (cands.len(), msgs)
}

/// The **real** early close: the periodogram is built from `keep`
/// samples and finished there, so it averages fewer rows rather than
/// averaging in rows of zeros. This is the arm that says whether
/// `SYNC_MIN = 1.2` still means the same thing — `getcandidates4`
/// normalises against a fitted baseline, and the baseline is fitted to
/// however many rows there were.
fn decode_short(audio: &[i16], keep: usize) -> (usize, Vec<String>) {
    let mut b = Ft4SavgBuilder::new(keep);
    b.push_with_rows(&audio[..keep], &mut |_| {});
    let savg = b.finish();
    let cands =
        ft4_coarse_sync_from_savg(&savg, FREQ_MIN_HZ, FREQ_MAX_HZ, SYNC_MIN, None, MAX_CAND);
    // The DDC sees only what arrived; `flush_to` supplies the zero
    // padding the reference's own `fft1_size` padding stands for.
    let half = decimate_slot(&audio[..keep]);
    let mut msgs: Vec<String> = Vec::new();
    for c in &cands {
        let mut cd0 = candidate_baseband_half(&half, c.freq_hz);
        rms_normalise(&mut cd0);
        let s2 = ft4_sync_search_window::<Ft4>(&cd0, c, NARROW_WINDOW.0, NARROW_WINDOW.1);
        let Some(r) = process_candidate_precomputed::<Ft4>(
            c,
            &[],
            &FT4_DOWNSAMPLE,
            DecodeDepth::EMBEDDED,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            (cd0, s2.freq_hz, s2.i0, s2.score),
            false,
            false,
        ) else {
            continue;
        };
        if let Some(t) = unpack77(r.message77())
            && !msgs.contains(&t)
        {
            msgs.push(t);
        }
    }
    msgs.sort();
    (cands.len(), msgs)
}

fn truncated(audio: &[i16], keep: usize) -> Vec<i16> {
    let mut v = vec![0i16; audio.len()];
    v[..keep].copy_from_slice(&audio[..keep]);
    v
}

/// Decode the golden with the capture window closed at a range of
/// points, against the whole slot.
#[test]
fn ft4_early_close_costs_nothing_on_the_golden() {
    let Some(audio) = slot_audio() else {
        if std::env::var("MFSK_REQUIRE_CORPUS").is_ok() {
            panic!("MFSK_REQUIRE_CORPUS=1 but the FT4 golden recording is missing");
        }
        eprintln!("skipping: FT4 golden recording not found");
        return;
    };

    let (full_c, full_msgs) = decode(&audio);
    eprintln!(
        "whole slot   7.500 s  {full_c:>3} candidates  {} decodes",
        full_msgs.len()
    );
    assert_eq!(
        full_msgs.len(),
        11,
        "the whole-slot arm changed — fix that before reading the rest"
    );

    eprintln!(
        "needed by the search: {NEEDED_SAMPLES} samples = {:.3} s",
        NEEDED_SAMPLES as f32 / 12_000.0
    );
    for keep in [NEEDED_SAMPLES, 73_200, 75_000, 78_000, 84_000] {
        let (c, msgs) = decode(&truncated(&audio, keep));
        let missing: Vec<&String> = full_msgs.iter().filter(|m| !msgs.contains(m)).collect();
        let extra: Vec<&String> = msgs.iter().filter(|m| !full_msgs.contains(m)).collect();
        eprintln!(
            "close at   {:.3} s  {c:>3} candidates  {} decodes  (missing {}, extra {})",
            keep as f32 / 12_000.0,
            msgs.len(),
            missing.len(),
            extra.len()
        );
        for m in &missing {
            eprintln!("      -- {m}");
        }
        for m in &extra {
            eprintln!("      ++ {m}");
        }
    }

    // The shipped close point (`ft4_rx::CAPTURE_CLOSE_SAMPLES`),
    // asserted rather than merely printed: 6.041 s is what the search
    // can reach and 6.25 s adds the DDC chain's group delay, so the
    // last symbols of a signal at the top of the DT window are
    // filtered against real history rather than against the flush's
    // zeros. If this ever costs a decode on the golden, the receiver's
    // window is wrong and not merely tight.
    const SHIPPED_CLOSE: usize = 75_000;
    let (_, shipped) = decode_short(&audio, SHIPPED_CLOSE);
    assert_eq!(
        shipped,
        full_msgs,
        "closing the capture window at {:.3} s changed the decode set",
        SHIPPED_CLOSE as f32 / 12_000.0
    );

    eprintln!("-- and the same closes with the periodogram finished short --");
    for keep in [NEEDED_SAMPLES, 73_200, 75_000, 78_000, 84_000] {
        let (c, msgs) = decode_short(&audio, keep);
        let missing: Vec<&String> = full_msgs.iter().filter(|m| !msgs.contains(m)).collect();
        let extra: Vec<&String> = msgs.iter().filter(|m| !full_msgs.contains(m)).collect();
        eprintln!(
            "close at   {:.3} s  {c:>3} candidates  {} decodes  (missing {}, extra {})",
            keep as f32 / 12_000.0,
            msgs.len(),
            missing.len(),
            extra.len()
        );
        for m in &missing {
            eprintln!("      -- {m}");
        }
        for m in &extra {
            eprintln!("      ++ {m}");
        }
    }
}
