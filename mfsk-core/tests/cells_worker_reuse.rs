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

/// `sic_passes` over **one** `SicStorage`, one worker and one table set, for several slots in turn (the board's way: everything made once), gives what
/// `decode_block_cells_sic` gives with everything made fresh, in the same order and to the bit (`DecodeResult` equality by message, frequency and DT).
/// **Fails if** state of one slot (the ring, the signals' correlation states, the symbol table) leaks into the next, or a pass reads a stale cell of the store.
#[test]
fn sic_over_one_storage_equals_sic_with_fresh_memory() {
    use mfsk_core::ft8::decode_block::{
        SicStorage, decode_block_cells_sic, sic_passes, sic_signal_capacity,
    };
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(&mut cs, &mut bp, &tables);
    let (max_cand, passes) = (30usize, 3usize);
    let mut storage = SicStorage::new(Ft8Rate::Hz8000, sic_signal_capacity(max_cand, passes));
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    for name in ["qso3_busy", "qso1", "qso3_busy", "qso2"] {
        let a = audio(name);
        let want = decode_block_cells_sic(
            &a,
            Ft8Rate::Hz8000,
            &search,
            max_cand,
            DecodeDepth::EMBEDDED,
            passes,
        )
        .unwrap();
        let (cands, mut cells) =
            coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1).unwrap();
        let got = sic_passes(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &search,
            max_cand,
            DecodeDepth::EMBEDDED,
            passes,
            &cands,
            &mut cells,
            &tables,
            &mut worker,
            &mut storage.scratch(),
        );
        assert!(!want.is_empty(), "{name}");
        let key = |r: &[mfsk_core::ft8::decode::DecodeResult]| -> Vec<(Option<String>, u32, u32)> {
            r.iter()
                .map(|d| {
                    (
                        unpack77(d.message77()),
                        d.freq_hz.to_bits(),
                        d.dt_sec.to_bits(),
                    )
                })
                .collect()
        };
        assert_eq!(
            key(&got),
            key(&want),
            "{name}: shared memory differs from fresh memory"
        );
    }
}

/// A scratch built for fewer signals than the flow can decode is refused, not overrun.
#[test]
#[should_panic(expected = "the SicScratch holds")]
fn a_scratch_too_small_for_the_flow_is_refused() {
    use mfsk_core::ft8::decode_block::{SicStorage, sic_passes};
    let a = audio("qso1");
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(&mut cs, &mut bp, &tables);
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let (cands, mut cells) =
        coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1).unwrap();
    let mut storage = SicStorage::new(Ft8Rate::Hz8000, 4);
    sic_passes(
        &mut SliceSource(&a),
        Ft8Rate::Hz8000,
        &search,
        30,
        DecodeDepth::EMBEDDED,
        3,
        &cands,
        &mut cells,
        &tables,
        &mut worker,
        &mut storage.scratch(),
    );
}

/// A `SicScratch` over the caller's own slices (a firmware's arena, here plain `Vec`s filled with garbage) gives what `SicStorage` gives. **Fails if** a buffer's
/// previous contents matter (the ring is zeroed per flow, the states reset, the symbol table rebuilt) or `SicScratch::new` mis-slices.
#[test]
fn a_scratch_over_the_callers_slices_equals_the_storage() {
    use mfsk_core::ft8::decode_block::{
        SicScratch, SicStorage, SigState, sic_passes, sic_signal_capacity,
    };
    let a = audio("qso3_busy");
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut worker = CellsWorker::new(&mut cs, &mut bp, &tables);
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let cap = sic_signal_capacity(30, 3);
    let run = |scratch: &mut SicScratch<'_>, worker: &mut CellsWorker<'_>| {
        let (cands, mut cells) =
            coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1).unwrap();
        sic_passes(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &search,
            30,
            DecodeDepth::EMBEDDED,
            3,
            &cands,
            &mut cells,
            &tables,
            worker,
            scratch,
        )
        .iter()
        .map(|d| {
            (
                unpack77(d.message77()),
                d.freq_hz.to_bits(),
                d.dt_sec.to_bits(),
            )
        })
        .collect::<Vec<_>>()
    };
    let mut storage = SicStorage::new(Ft8Rate::Hz8000, cap);
    let want = run(&mut storage.scratch(), &mut worker);
    assert!(want.len() >= 9);
    let (mut ring, mut rd, mut raw, mut gidx) = (
        vec![f32::NAN; 8192],
        vec![1e9f32; 700],
        vec![-7i16; 700],
        vec![255u8; 1300],
    );
    let mut st = vec![SigState::default(); cap + 3];
    let mut mine = SicScratch::new(
        Ft8Rate::Hz8000,
        &mut ring,
        &mut rd,
        &mut raw,
        &mut gidx,
        &mut st,
    );
    assert_eq!(run(&mut mine, &mut worker), want);
    assert_eq!(
        run(&mut mine, &mut worker),
        want,
        "a second flow over the same slices"
    );
}
