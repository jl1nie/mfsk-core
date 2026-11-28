//! The cells flow (`decode_block_cells`, 8 kHz, fixed-point) on the three vendored recordings against **ft8dec**
//! (icap-sdk 1ca9f04, `apps/ft8/ft8dec.c`, MIT: its defaults, 300 candidates, OSD on, 3 passes, on the same 8 kHz audio, measured 2026-11-26).
//!
//! The board's `ft8-cells-bench` runs the same call and its decoded sets were equal to these on the CoreS3 (S3 plan step 2).
//! What fails here: a decode that ft8dec's list does not contain (a phantom ceiling against an independent decoder), a recall drop below
//! the measured counts, the retry or the subtraction passes decoding fewer than the plain flow.
//!
//! `cells_flow_one_call_for_valgrind` is the entry point of `scripts/compare-ft8dec-cells.sh` (ignored: it asserts nothing).
#![cfg(all(feature = "fixed-point", feature = "internal-testing"))]
use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    Ft8Rate, GatedSearch, decode_block_cells, decode_block_cells_sic,
};
use mfsk_core::msg::wsjt77::unpack77;

/// ft8dec's own list (300 candidates, OSD, 3 passes) on each recording's 8 kHz audio.
const FT8DEC: [(&str, &[&str]); 3] = [
    (
        "qso3_busy",
        &[
            "A92EE F5PSR -14",
            "CQ F5RXL IN94",
            "CQ EA2BFM IN83",
            "K1BZM EA3CJ JN01",
            "K1JT EA3AGB -15",
            "K1JT HA0DU KN07",
            "KD2UGC F6GCP R-23",
            "N1API HA6FQ -23",
            "N1JFU EA6EE R-07",
            "N1PJT HB9CQK -10",
            "W0RSJ EA3BMU RR73",
            "W1DIG SV9CVY -14",
            "W1FC F5BZB -08",
            "WA2FZW DL5AXX RR73",
            "WM3PEN EA6VQ -09",
            "XE2X HA2NP RR73",
        ],
    ),
    (
        "qso1",
        &[
            "CQ DX R6WA LN32",
            "CQ R7IW LN35",
            "CQ TA6CQ KN70",
            "OH3NIV ZS6S -03",
            "TK4LS YC1MRF 73",
        ],
    ),
    (
        "qso2",
        &[
            "CQ DX R6WA LN32",
            "CQ LZ1JZ KN22",
            "CQ R7IW LN35",
            "CQ TA6CQ KN70",
            "OH3NIV ZS6S RR73",
        ],
    ),
];

fn audio8k(name: &str) -> Vec<i16> {
    let p = format!(
        "{}/../embedded-poc/assets/{name}.wav",
        env!("CARGO_MANIFEST_DIR")
    );
    let w = std::fs::read(&p).unwrap_or_else(|e| panic!("vendored wav {p}: {e}"));
    let s: Vec<i16> = w[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .take(180_000)
        .map(|b| i16::from_le_bytes(*b))
        .collect();
    let a8 = resample_to_8k(&s, 12_000);
    if let Ok(d) = std::env::var("CELLS_DUMP_DIR") {
        let raw: Vec<u8> = a8.iter().flat_map(|v| v.to_le_bytes()).collect();
        std::fs::write(format!("{d}/{name}_8k.raw"), raw).unwrap();
    }
    a8
}

fn texts(out: &[mfsk_core::ft8::decode::DecodeResult]) -> Vec<String> {
    let mut t: Vec<_> = out.iter().filter_map(|d| unpack77(d.message77())).collect();
    t.sort();
    t
}

fn search() -> GatedSearch {
    GatedSearch::new(200.0, 3000.0, 1.3, 30)
}

/// The plain flow, 15 and 30 candidates, OSD off (as the board ships): 9 / 5 / 5 decodes, every one in ft8dec's list; the subtraction passes
/// lose nothing and decode nothing that the retry-free rule would call a duplicate. **Fails if** a candidate decodes to text ft8dec does not
/// have on a recording (precision), or the counts fall (recall).
#[test]
fn cells_flow_matches_ft8dec_on_the_vendored_recordings() {
    for (name, reference) in FT8DEC {
        let a8 = audio8k(name);
        let want = match name {
            "qso3_busy" => 9,
            _ => 5,
        };
        for max_cand in [15usize, 30] {
            let got = texts(
                &decode_block_cells(
                    &a8,
                    Ft8Rate::Hz8000,
                    &search(),
                    max_cand,
                    DecodeDepth::EMBEDDED,
                )
                .unwrap(),
            );
            assert_eq!(got.len(), want, "{name} max_cand {max_cand}: {got:?}");
            for t in &got {
                assert!(
                    reference.contains(&t.as_str()),
                    "{name} max_cand {max_cand}: {t:?} is not in ft8dec's list"
                );
            }
        }
        let plain = texts(
            &decode_block_cells(&a8, Ft8Rate::Hz8000, &search(), 30, DecodeDepth::EMBEDDED)
                .unwrap(),
        );
        for passes in [2usize, 3] {
            let sic = texts(
                &decode_block_cells_sic(
                    &a8,
                    Ft8Rate::Hz8000,
                    &search(),
                    30,
                    DecodeDepth::EMBEDDED,
                    passes,
                )
                .unwrap(),
            );
            for t in &plain {
                assert!(sic.contains(t), "{name} sic{passes} lost {t:?}");
            }
            assert!(sic.len() >= plain.len(), "{name} sic{passes}");
        }
    }
}

/// One call selected by `CELLS_MODE` (`none` | `c15` | `c30` | `sic2` | `sic3`) on `CELLS_WAV`, for instruction counts (`scripts/compare-ft8dec-cells.sh`):
/// `none` is the load + resample baseline to subtract.
#[test]
#[ignore = "entry point of scripts/compare-ft8dec-cells.sh: asserts nothing"]
fn cells_flow_one_call_for_valgrind() {
    let (Ok(mode), Ok(name)) = (std::env::var("CELLS_MODE"), std::env::var("CELLS_WAV")) else {
        return;
    };
    let a8 = audio8k(&name);
    let srch = search();
    let n = match mode.as_str() {
        "none" => 0,
        "c15" => decode_block_cells(&a8, Ft8Rate::Hz8000, &srch, 15, DecodeDepth::EMBEDDED)
            .unwrap()
            .len(),
        "c30" => decode_block_cells(&a8, Ft8Rate::Hz8000, &srch, 30, DecodeDepth::EMBEDDED)
            .unwrap()
            .len(),
        "sic2" => decode_block_cells_sic(&a8, Ft8Rate::Hz8000, &srch, 30, DecodeDepth::EMBEDDED, 2)
            .unwrap()
            .len(),
        "sic3" => decode_block_cells_sic(&a8, Ft8Rate::Hz8000, &srch, 30, DecodeDepth::EMBEDDED, 3)
            .unwrap()
            .len(),
        m => panic!("mode {m}"),
    };
    println!("ONECALL {name} {mode}: {n} decodes");
}
