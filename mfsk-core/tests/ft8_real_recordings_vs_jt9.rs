// SPDX-License-Identifier: GPL-3.0-only
//! Tier B: the two vendored real FT8 recordings against what WSJT-X's own `jt9` decodes from them.
//!
//! Why this exists. `qso3_busy.wav` was the only real FT8 recording with a `jt9` reference list; these
//! two (`191111_110130.wav`, `191111_110200.wav`, ft8_lib's `test/wav`, MIT) were vendored and used
//! for a coarse-sync timing test only. On 2026-11-03 a comparison over ft8_lib's 22 recordings found
//! `mfsk-core` missing 18 of `jt9`'s 401 decodes, one of them `DO2JHE LZ1JZ -13` in
//! `191111_110130.wav`: a decode `jt9` makes by OSD at 29 hard errors on LLRs that differ from ours in
//! one to four signs (the last group of each half of the multi-symbol bit metrics, `LlrTail`). Nothing in
//! the suite compared that file with `jt9`, so nothing could see it.
//!
//! Reference: WSJT-X `v3.2.0-rc1` `jt9 -8 -d 3 -L 200 -H 3000` (`scripts/build_jt9_upstream.sh`), the
//! task `ft8/t1` of `scripts/upstream_tasks.json`, run on each file on 2026-11-03: every line it printed,
//! with its frequency, DT and SNR. The decoder is the request `ft8_sweep`'s `t1` task makes,
//! `DecodeParams::for_band((200, 3000))`, `Depth::Deep`, `ApMode::Full` (AP on, no callsigns, so the CQ
//! hypothesis only). A decode `jt9` makes and this crate does not is a regression of recall; one this
//! crate makes and `jt9` does not is a phantom until shown otherwise.
//!
//! The margin on `DO2JHE LZ1JZ -13` is thin by nature: `jt9` gets it by OSD at 29 hard errors,
//! so a platform whose FFT rounds differently (rustfft's NEON kernel on aarch64) could in principle flip
//! it. CI tier B runs on x86-64 only; if this fails elsewhere, compare `jt9`'s LLRs before loosening it.
#![cfg(feature = "fft-rustfft")]

#[allow(dead_code)]
mod common;
use common::golden::{DecodeView, GoldenEntry, GoldenSet, Tolerances, assert_golden};
use common::load_wav_i16;

use mfsk_core::decoder::{ApMode, DecodeParams, Decoder, Depth, SlotInput};
use mfsk_core::ft8::Ft8;

fn decode(path: &str) -> Vec<DecodeView> {
    let audio = load_wav_i16(path);
    let params = DecodeParams::for_band((200.0, 3000.0))
        .depth(Depth::Deep)
        .ap(ApMode::Full);
    Decoder::<Ft8>::new(params)
        .decode(&SlotInput::i16(&audio))
        .rows
        .into_iter()
        .map(|r| DecodeView {
            msg: r.decoded.text.clone(),
            freq_hz: r.decoded.freq_hz,
            dt_sec: r.decoded.dt_sec,
            snr_db: Some(r.decoded.snr_db),
        })
        .collect()
}

/// `jt9 -8 -d 3 -L 200 -H 3000 191111_110130.wav`: 6 decodes. `DO2JHE LZ1JZ -13` is the marginal one
/// (OSD, 29 hard errors, `llre`).
const WAV_110130: &[GoldenEntry] = &[
    GoldenEntry::msg("CQ R7IW LN35").at(1291.0, 0.9).snr(-2.0),
    GoldenEntry::msg("CQ TA6CQ KN70").at(682.0, 0.7).snr(-10.0),
    GoldenEntry::msg("CQ DX R6WA LN32")
        .at(2096.0, 0.9)
        .snr(-8.0),
    GoldenEntry::msg("OH3NIV ZS6S -03")
        .at(990.0, 1.0)
        .snr(-15.0),
    GoldenEntry::msg("DO2JHE LZ1JZ -13")
        .at(1149.0, 0.8)
        .snr(-18.0),
    GoldenEntry::msg("TK4LS YC1MRF 73")
        .at(2479.0, 1.2)
        .snr(-19.0),
];

/// `jt9 -8 -d 3 -L 200 -H 3000 191111_110200.wav`: 5 decodes.
const WAV_110200: &[GoldenEntry] = &[
    GoldenEntry::msg("CQ R7IW LN35").at(1292.0, 0.9).snr(-6.0),
    GoldenEntry::msg("CQ DX R6WA LN32")
        .at(2097.0, 0.9)
        .snr(-11.0),
    GoldenEntry::msg("CQ TA6CQ KN70").at(683.0, 0.7).snr(-9.0),
    GoldenEntry::msg("OH3NIV ZS6S RR73")
        .at(990.0, 1.0)
        .snr(-14.0),
    GoldenEntry::msg("CQ LZ1JZ KN22").at(1030.0, 0.6).snr(-15.0),
];

#[test]
fn recording_191111_110130_matches_jt9() {
    let set = GoldenSet {
        name: "191111_110130 vs jt9",
        expected: WAV_110130,
        min_hits: 6,
        max_extra: 0,
    };
    assert_golden(
        &decode(asset_path!("191111_110130.wav")),
        &set,
        Tolerances::default(),
        |d| d.clone(),
    );
}

#[test]
fn recording_191111_110200_matches_jt9() {
    let set = GoldenSet {
        name: "191111_110200 vs jt9",
        expected: WAV_110200,
        min_hits: 5,
        max_extra: 0,
    };
    assert_golden(
        &decode(asset_path!("191111_110200.wav")),
        &set,
        Tolerances::default(),
        |d| d.clone(),
    );
}
