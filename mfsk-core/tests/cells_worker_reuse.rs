//! `CellsWorker::decode_round` over the caller's memory (S3 plan, SIC on the CoreS3, step S1): one worker serves several rounds (the passes of a subtraction
//! flow) and gives **the same decodes** as a fresh worker for each round. **Fails if** a round over a reused worker decodes differently from the same round
//! over a fresh one (state leaking from one round into the next through the staging block or the BP scratch).
//!

#![cfg(all(feature = "fixed-point", feature = "internal-testing"))]
use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::engine::scalar::Cmplx;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::{
    CellTables, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch, coarse_sync_gated_cells,
};
use mfsk_core::msg::wsjt77::unpack77;

fn audio(name: &str) -> Vec<i16> {
    let p = format!(
        "{}/../embedded-poc/assets/{name}.wav",
        env!("CARGO_MANIFEST_DIR")
    );
    let w = std::fs::read(&p).unwrap_or_else(|e| panic!("vendored wav {p}: {e}"));
    let s: Vec<i16> = w[44..]
        .chunks_exact(2)
        .take(180_000)
        .map(|b| i16::from_le_bytes([b[0], b[1]]))
        .collect();
    resample_to_8k(&s, 12_000)
}

fn texts(known: &[mfsk_core::ft8::decode_block::Found]) -> Vec<String> {
    known
        .iter()
        .filter_map(|f| unpack77(f.res.message77()))
        .collect()
}

#[test]
fn a_reused_worker_decodes_what_a_fresh_one_decodes() {
    let tables = CellTables::new();
    // a reused worker, over both recordings' candidates in turn (qso3_busy first: 9+ decodes; then qso1)
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut reused = CellsWorker::new(&mut cs, &mut bp, &tables);
    for name in ["qso3_busy", "qso1", "qso3_busy", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        let mut known_reused = Vec::new();
        let n = reused.decode_round(
            &cells,
            &cands,
            30,
            DecodeDepth::EMBEDDED,
            0.25,
            &mut known_reused,
        );
        assert_eq!(n, known_reused.len());

        let mut cs2: Box<[[Cmplx<f32>; 8]; 79]> =
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
        let mut bp2 = Box::new(Ft8BpScratch::new());
        let mut known_fresh = Vec::new();
        CellsWorker::new(&mut cs2, &mut bp2, &tables).decode_round(
            &cells,
            &cands,
            30,
            DecodeDepth::EMBEDDED,
            0.25,
            &mut known_fresh,
        );

        assert!(!known_reused.is_empty(), "{name}: the recording decodes");
        assert_eq!(
            texts(&known_reused),
            texts(&known_fresh),
            "{name}: reused worker differs from a fresh one"
        );
        let pos = |k: &[mfsk_core::ft8::decode_block::Found]| -> Vec<(i32, i64)> {
            k.iter().map(|f| (f.grid.j0, f.grid.k)).collect()
        };
        assert_eq!(pos(&known_reused), pos(&known_fresh), "{name}: grid places");
    }
}
