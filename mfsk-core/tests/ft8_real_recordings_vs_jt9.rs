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

/// `jt9 -8 -d 3 -L 200 -H 3000 191111_110645.wav`.
const WAV_110645: &[GoldenEntry] = &[
    GoldenEntry::msg("PA3EPP SP8NFO R+01")
        .at(906.0, 0.7)
        .snr(12.0),
    GoldenEntry::msg("PC2J IZ1ANK +01").at(2191.0, 1.5).snr(0.0),
    GoldenEntry::msg("CQ JA OH1LWZ KP11")
        .at(2657.0, 0.9)
        .snr(2.0),
    GoldenEntry::msg("VK4BLE OH1EDK -20")
        .at(2576.0, 0.7)
        .snr(1.0),
    GoldenEntry::msg("CQ OH8GDU KP24").at(2281.0, 0.9).snr(-8.0),
    GoldenEntry::msg("VK4BLE OH8JK R-17")
        .at(430.0, 0.9)
        .snr(-1.0),
    GoldenEntry::msg("CQ DL1UDO JO31")
        .at(2447.0, 0.9)
        .snr(-15.0),
    GoldenEntry::msg("CQ RU3XL KO84").at(699.0, 1.8).snr(-17.0),
    GoldenEntry::msg("CQ F4FSY JN25").at(1283.0, 0.9).snr(4.0),
    GoldenEntry::msg("CQ OR18TRA").at(2111.0, 1.0).snr(-12.0),
    GoldenEntry::msg("SV1GN RK6AUV R-03")
        .at(1405.0, 1.0)
        .snr(-11.0),
    GoldenEntry::msg("PB5DX EI3CTB IO63")
        .at(1617.0, 0.9)
        .snr(-23.0),
    GoldenEntry::msg("CQ UB3AQS KO85")
        .at(1049.0, 0.9)
        .snr(-15.0),
    GoldenEntry::msg("SP7XIF JA2GQT -13")
        .at(2726.0, 1.4)
        .snr(-19.0),
    GoldenEntry::msg("WB2QJ ES3AT KO18")
        .at(2092.0, 0.9)
        .snr(-23.0),
    GoldenEntry::msg("CQ DG0OFT JO50").at(594.0, 0.8).snr(-25.0),
    GoldenEntry::msg("<...> DA0FONTANE")
        .at(1114.0, 0.7)
        .snr(-18.0),
];

/// `jt9 -8 -d 3 -L 200 -H 3000 websdr_test11.wav`.
const WEBSDR_TEST11: &[GoldenEntry] = &[
    GoldenEntry::msg("CQ PD1ECA JO32").at(1881.0, -0.0).snr(7.0),
    GoldenEntry::msg("CQ IK4LZH JN54").at(903.0, 0.1).snr(13.0),
    GoldenEntry::msg("CQ HA1RB JN86").at(1653.0, 0.1).snr(15.0),
    GoldenEntry::msg("CQ SV2FPI KN10").at(1125.0, 0.1).snr(-8.0),
    GoldenEntry::msg("CQ F8IJV/P IN97").at(2830.0, 0.3).snr(7.0),
    GoldenEntry::msg("K2DSW IU8LLZ R-16")
        .at(2601.0, -1.2)
        .snr(-2.0),
    GoldenEntry::msg("PD3JO IZ2ODN JN55")
        .at(1544.0, -0.0)
        .snr(-5.0),
    GoldenEntry::msg("CQ 9A7DA JN86").at(1431.0, 0.0).snr(-7.0),
    GoldenEntry::msg("CQ PY1SX GG87").at(1955.0, -0.0).snr(-4.0),
    GoldenEntry::msg("OE4RWD NU2Q RR73")
        .at(734.0, -0.2)
        .snr(0.0),
    GoldenEntry::msg("R7EL VE9FI FN75")
        .at(1320.0, 0.2)
        .snr(-6.0),
    GoldenEntry::msg("F4DFQ F5LOW IN95")
        .at(2198.0, 1.5)
        .snr(8.0),
    GoldenEntry::msg("CQ N2BJ EN61").at(609.0, 0.1).snr(-16.0),
    GoldenEntry::msg("M0LMR IW1AYD 73")
        .at(2036.0, -1.1)
        .snr(-12.0),
    GoldenEntry::msg("K4VBM HA8EK RR73")
        .at(2480.0, -2.3)
        .snr(-18.0),
    GoldenEntry::msg("EA8BEV LU3DW -13")
        .at(311.0, -1.5)
        .snr(-4.0),
    GoldenEntry::msg("KC8MUE V51MA RRR")
        .at(1219.0, 0.1)
        .snr(-18.0),
    GoldenEntry::msg("CQ YO9HP KN35").at(1951.0, 0.1).snr(-8.0),
    GoldenEntry::msg("CQ 2E0PKK IO90")
        .at(2406.0, 0.1)
        .snr(-15.0),
    GoldenEntry::msg("OK1AW G3JFS R+02")
        .at(1686.0, 0.2)
        .snr(-6.0),
    GoldenEntry::msg("G3PXT HA5MG R+01")
        .at(1619.0, -0.8)
        .snr(-7.0),
    GoldenEntry::msg("VE6BTC W1JGM R-22")
        .at(1410.0, -0.4)
        .snr(-18.0),
    GoldenEntry::msg("Z81D W3GQ EM95").at(725.0, 0.3).snr(-9.0),
    GoldenEntry::msg("K3ZK IK2ZDT RR73")
        .at(2219.0, 0.1)
        .snr(2.0),
    GoldenEntry::msg("CQ OE4ATS JN87")
        .at(2304.0, -0.2)
        .snr(-15.0),
    GoldenEntry::msg("PY5HT IW9CTR RR73")
        .at(726.0, 0.0)
        .snr(-13.0),
];

/// `jt9 -8 -d 3 -L 200 -H 3000 websdr_test5.wav`.
const WEBSDR_TEST5: &[GoldenEntry] = &[
    GoldenEntry::msg("CQ F5RRS JN36").at(1494.0, 0.2).snr(8.0),
    GoldenEntry::msg("R2ZBK UA3IBD -15")
        .at(1172.0, 0.1)
        .snr(14.0),
    GoldenEntry::msg("SV8EUB OM7AZA JN98")
        .at(1397.0, 0.2)
        .snr(15.0),
    GoldenEntry::msg("SB7W DL6CHF JO52").at(788.0, 0.2).snr(2.0),
    GoldenEntry::msg("SP2EWQ DL8TG R+07")
        .at(2747.0, 0.2)
        .snr(11.0),
    GoldenEntry::msg("ON8GE DL6ZNG 73")
        .at(2040.0, -0.0)
        .snr(6.0),
    GoldenEntry::msg("CQ DD2XJ JO53").at(1054.0, 0.5).snr(12.0),
    GoldenEntry::msg("LZ2KV GW1YQM IO82")
        .at(349.0, 0.2)
        .snr(-3.0),
    GoldenEntry::msg("UT9LB RZ3OA KO91").at(692.0, 0.2).snr(9.0),
    GoldenEntry::msg("OZ1KNX OZ5D -11")
        .at(1801.0, 0.2)
        .snr(23.0),
    GoldenEntry::msg("CQ DJ0AH JN57").at(2392.0, 0.2).snr(6.0),
    GoldenEntry::msg("OM7ZM RW6FY 73").at(1992.0, 0.6).snr(4.0),
    GoldenEntry::msg("EA8PP DL5OBC JO52")
        .at(2184.0, 0.3)
        .snr(-4.0),
    GoldenEntry::msg("EA2AA HA1BL JN87")
        .at(2105.0, 0.2)
        .snr(7.0),
    GoldenEntry::msg("RA6FSD 4X5MZ RR73")
        .at(570.0, 0.3)
        .snr(-3.0),
    GoldenEntry::msg("CQ RA3QUE KO91").at(1624.0, 0.2).snr(-3.0),
    GoldenEntry::msg("CQ UA3YFS KO73").at(2409.0, -1.3).snr(4.0),
    GoldenEntry::msg("EA8PP JH0INP PM96")
        .at(940.0, -0.6)
        .snr(-12.0),
    GoldenEntry::msg("UT8UU ON4FG RR73")
        .at(2133.0, 0.2)
        .snr(5.0),
    GoldenEntry::msg("EA8PP UN7IT LO80")
        .at(1108.0, 0.5)
        .snr(-7.0),
    GoldenEntry::msg("OH1WR RA4UDC RR73")
        .at(1874.0, -1.7)
        .snr(8.0),
    GoldenEntry::msg("CQ DO1RPK JO32").at(1503.0, 0.1).snr(-1.0),
    GoldenEntry::msg("LZ2KV SV8LMQ 73").at(1028.0, 0.4).snr(0.0),
    GoldenEntry::msg("OZ0JD SM5NAS R-08")
        .at(1218.0, 0.2)
        .snr(-8.0),
    GoldenEntry::msg("ON6OM DL8FBD 73").at(1766.0, 0.2).snr(9.0),
    GoldenEntry::msg("DB4BU DK5OK RR73")
        .at(1822.0, 0.1)
        .snr(6.0),
    GoldenEntry::msg("EA2AA S56ECR JN65")
        .at(2047.0, 0.3)
        .snr(-5.0),
    GoldenEntry::msg("CQ ON7PM JO20").at(735.0, 0.2).snr(-15.0),
    GoldenEntry::msg("CQ DO6AZ JO50").at(1481.0, 0.6).snr(-5.0),
    GoldenEntry::msg("CQ DO1FPI JN58")
        .at(1357.0, 0.2)
        .snr(-11.0),
];

/// `jt9 -8 -d 3 -L 200 -H 3000 websdr_test2.wav`.
const WEBSDR_TEST2: &[GoldenEntry] = &[
    GoldenEntry::msg("CQ IT9PQO JM78").at(1495.0, 1.4).snr(7.0),
    GoldenEntry::msg("IK4LZH 9A9TT JN76")
        .at(1109.0, 0.3)
        .snr(4.0),
    // SNR not checked: +15 dB here against `jt9`'s +11, the same before #721 (a separate gap in the
    // strong-signal SNR estimate, not this test's subject).
    GoldenEntry::msg("SP4FCA G4CUS R+13").at(309.0, 1.1),
    GoldenEntry::msg("UR4MSF E75C R+10")
        .at(1188.0, 1.1)
        .snr(0.0),
    GoldenEntry::msg("LA9XBA F6CAM JO10")
        .at(1430.0, 1.0)
        .snr(18.0),
    GoldenEntry::msg("DC8VA LZ1CWK R-09")
        .at(598.0, 1.2)
        .snr(6.0),
    GoldenEntry::msg("CQ 2E0VDS JO02").at(2672.0, 2.1).snr(0.0),
    GoldenEntry::msg("UA9CJM ON8BB -20")
        .at(2597.0, 1.0)
        .snr(-1.0),
    GoldenEntry::msg("EA1ABT I8LWL JN70")
        .at(2267.0, 1.2)
        .snr(11.0),
    GoldenEntry::msg("4F3OM F6GGA JN37").at(638.0, 1.1).snr(0.0),
    GoldenEntry::msg("A41ZZ YO7CGS R-18")
        .at(2016.0, 1.1)
        .snr(-1.0),
    GoldenEntry::msg("S9CT 9A4ZM -04").at(389.0, 1.1).snr(-7.0),
    GoldenEntry::msg("CQ LZ2II KN22").at(1706.0, 1.7).snr(7.0),
    GoldenEntry::msg("OH3KAV 2M0OGG RR73")
        .at(1642.0, 1.4)
        .snr(-6.0),
    GoldenEntry::msg("S9CT F4HPY JN28").at(458.0, 1.8).snr(-5.0),
    GoldenEntry::msg("HA8RC R4OF 73").at(2393.0, 1.1).snr(-14.0),
    GoldenEntry::msg("S9CT EW8KT KO42")
        .at(2344.0, 1.6)
        .snr(-17.0),
    GoldenEntry::msg("CQ M0OIC IO92").at(1707.0, 1.0).snr(2.0),
    GoldenEntry::msg("CQ SV1GN KM17").at(895.0, 1.8).snr(-6.0),
    GoldenEntry::msg("YO8TVD M0JBF IO91")
        .at(2158.0, 1.1)
        .snr(-16.0),
    GoldenEntry::msg("CQ G4IJC JO02").at(1642.0, 1.1).snr(-19.0),
    GoldenEntry::msg("CQ IZ8IQO JM89").at(711.0, 1.1).snr(-25.0),
    GoldenEntry::msg("DM8PV GM7VFR RR73")
        .at(2026.0, 1.3)
        .snr(-13.0),
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

/// One `jt9` decode of a non-standard call with an unresolved hash, `<...> DA0FONTANE` (BP, 15 hard errors):
/// `i3=4`'s 11-character call was refused by the 7-character callsign grammar until #721.
#[test]
fn recording_191111_110645_matches_jt9() {
    let set = GoldenSet {
        name: "191111_110645 vs jt9",
        expected: WAV_110645,
        min_hits: WAV_110645.len(),
        max_extra: 0,
    };
    assert_golden(
        &decode(asset_path!("191111_110645.wav")),
        &set,
        Tolerances::default(),
        |d| d.clone(),
    );
}

/// `K3ZK IK2ZDT RR73` (+2 dB, 2219 Hz) sits 10 Hz from one of six copies of a splattering
/// `K4VBM HA8EK RR73`. `jt9` subtracts every copy (`ft8b.f90` subtracts each valid decode before the
/// duplicate check); until #721 a duplicate was dropped unsubtracted and K3ZK never became a candidate.
#[test]
fn recording_websdr_test11_matches_jt9() {
    let set = GoldenSet {
        name: "websdr_test11 vs jt9",
        expected: WEBSDR_TEST11,
        min_hits: WEBSDR_TEST11.len(),
        max_extra: 0,
    };
    assert_golden(
        &decode(asset_path!("websdr_test11.wav")),
        &set,
        Tolerances::default(),
        |d| d.clone(),
    );
}

/// `CQ DO6AZ JO50` is the blind-CQ AP pass at nsync 9 (OSD, 26 hard errors), which an `nsync >= 12` gate
/// of this crate's own skipped until #721. The one output `jt9` lacks, `EA8PP JA6VQA -16` (OSD at 21 hard
/// errors), is a QSO `jt9` decodes in other recordings of the session (`JA6VQA EA8PP R-24`, `... 73`).
#[test]
fn recording_websdr_test5_matches_jt9() {
    let set = GoldenSet {
        name: "websdr_test5 vs jt9",
        expected: WEBSDR_TEST5,
        min_hits: WEBSDR_TEST5.len(),
        max_extra: 1,
    };
    assert_golden(
        &decode(asset_path!("websdr_test5.wav")),
        &set,
        Tolerances::default(),
        |d| d.clone(),
    );
}

/// `CQ IZ8IQO JM89` (-25 dB, AP): `jt9`'s coarse candidate is at lag 29; with this crate's lag grid two
/// steps off `sync8.f90`'s (until #721) it was at 27, and `ft8b`'s ±10-sample search found another peak.
/// The one output `jt9` lacks, `UA9SIX GM0LIR R-07`, continues a QSO `jt9` decodes in the next recording
/// (`GM0LIR UA9SIX -09`).
#[test]
fn recording_websdr_test2_matches_jt9() {
    let set = GoldenSet {
        name: "websdr_test2 vs jt9",
        expected: WEBSDR_TEST2,
        min_hits: WEBSDR_TEST2.len(),
        max_extra: 1,
    };
    assert_golden(
        &decode(asset_path!("websdr_test2.wav")),
        &set,
        Tolerances::default(),
        |d| d.clone(),
    );
}
