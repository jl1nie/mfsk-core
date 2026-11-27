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

/// `sic_passes_with` and a round shared by two workers through an atomic index (what a two-core firmware runs, work stealing) give **the same decodes, in the same
/// order,** as `sic_passes` over one worker. **Fails if** the merge by candidate index differs from the sequential round (a candidate decoded twice or dropped,
/// dedupe order changed), or if a worker's state leaks into the other's result.
#[test]
fn a_round_shared_by_two_workers_equals_the_sequential_round() {
    use mfsk_core::ft8::decode_block::{
        Found, SicStorage, sic_passes, sic_passes_with, sic_signal_capacity,
    };
    use std::sync::atomic::{AtomicUsize, Ordering};
    let tables = CellTables::new();
    let rate = Ft8Rate::Hz8000;
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let mk = || -> (Box<[[Cmplx<f32>; 8]; 79]>, Box<Ft8BpScratch>) {
        (
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap(),
            Box::new(Ft8BpScratch::new()),
        )
    };
    let (mut cs0, mut bp0) = mk();
    let (mut cs1, mut bp1) = mk();
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        for passes in [1usize, 3] {
            let mut storage = SicStorage::new(rate, sic_signal_capacity(30, passes));
            let (cands, mut cells) =
                coarse_sync_gated_cells(&mut SliceSource(&a), rate, &search, 1).unwrap();
            let mut w0 = CellsWorker::new(&mut cs0, &mut bp0, &tables);
            let want = sic_passes(
                &mut SliceSource(&a),
                rate,
                &search,
                30,
                DecodeDepth::EMBEDDED,
                passes,
                &cands,
                &mut cells,
                &tables,
                &mut w0,
                &mut storage.scratch(),
            );
            let (cands, mut cells) =
                coarse_sync_gated_cells(&mut SliceSource(&a), rate, &search, 1).unwrap();
            let mut w0 = CellsWorker::new(&mut cs0, &mut bp0, &tables);
            let mut w1 = CellsWorker::new(&mut cs1, &mut bp1, &tables);
            let got = sic_passes_with(
                &mut SliceSource(&a),
                rate,
                &search,
                30,
                passes,
                &cands,
                &mut cells,
                &tables,
                &mut storage.scratch(),
                |cells, cands, max_cand, known| {
                    let n = cands.len().min(max_cand);
                    let next = AtomicUsize::new(0);
                    let run = |w: &mut CellsWorker<'_>| -> Vec<(usize, Option<Found>)> {
                        let mut mine = Vec::new();
                        loop {
                            let i = next.fetch_add(1, Ordering::Relaxed);
                            if i >= n {
                                break;
                            }
                            mine.push((
                                i,
                                w.decode_candidate(cells, &cands[i].0, DecodeDepth::EMBEDDED, 0.25),
                            ));
                        }
                        mine
                    };
                    let (a, b) = std::thread::scope(|s| {
                        let h = s.spawn(|| run(&mut w1));
                        (run(&mut w0), h.join().unwrap())
                    });
                    let mut all: Vec<_> = a.into_iter().chain(b).collect();
                    all.sort_by_key(|x| x.0);
                    for (_, f) in all {
                        if let Some(f) = f
                            && !known.iter().any(|o| o.res.message77() == f.res.message77())
                        {
                            known.push(f);
                        }
                    }
                },
            );
            let key = |v: &[mfsk_core::ft8::decode::DecodeResult]| {
                v.iter()
                    .map(|d| {
                        (
                            d.message77().to_vec(),
                            d.freq_hz.to_bits(),
                            d.dt_sec.to_bits(),
                        )
                    })
                    .collect::<Vec<_>>()
            };
            assert_eq!(key(&got), key(&want), "{name} passes {passes}");
            assert!(!want.is_empty());
        }
    }
}

/// `CellsWorker::bp_stats` / `take_bp_stats` report the BP work of the rounds a worker ran. **Fails if** the counters stay at zero through a round that
/// decoded something, count fewer successes than the round has decodes, or `take_bp_stats` does not start again from zero.
#[test]
fn a_worker_reports_the_bp_work_it_ran() {
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables);
    assert_eq!(w.bp_stats(), Default::default());
    let a = audio("qso3_busy");
    let (cands, cells) = coarse_sync_gated_cells(
        &mut SliceSource(&a),
        Ft8Rate::Hz8000,
        &GatedSearch::new(200.0, 3000.0, 1.3, 30),
        1,
    )
    .unwrap();
    let mut known = Vec::new();
    w.decode_round(&cells, &cands, 30, DecodeDepth::EMBEDDED, 0.25, &mut known);
    let st = w.take_bp_stats();
    assert!(!known.is_empty());
    assert!(
        st.decoded as usize >= known.len(),
        "{st:?} for {} decodes",
        known.len()
    );
    assert!(
        st.calls >= 30,
        "every candidate runs BP at least once or twice: {st:?}"
    );
    assert!(st.passes > 0);
    assert_eq!(w.bp_stats(), Default::default());
}

/// `CellsWorker::with_d_min_score` (Issue #765): under the floor a candidate gets variant `a` only, at the grid and at the retry; from the floor up the worker is the default one. On the
/// three recordings' 30 candidates each, against a default worker: for a candidate of score 16 or more the decode (message, frequency, DT) and the BP stats are **identical**; under it
/// the gated worker never decodes what the default one does not, never makes more BP calls, makes exactly half the calls on a candidate that fails (`a` and `d` become `a`), and
/// over all of them makes fewer. **Fails if** the floor is not applied (no call saved), applied above its score (a candidate of 16 or more differs), or the gate lets a decode through the
/// default worker does not have.
#[test]
fn a_worker_skips_variant_d_under_its_score_floor() {
    use mfsk_core::ft8::decode_block::CELLS_D_MIN_SCORE;
    let tables = CellTables::new();
    let mut cs0: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut cs1: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let (mut bp0, mut bp1) = (Box::new(Ft8BpScratch::new()), Box::new(Ft8BpScratch::new()));
    let mut base = CellsWorker::new(&mut cs0, &mut bp0, &tables);
    let mut gated =
        CellsWorker::new(&mut cs1, &mut bp1, &tables).with_d_min_score(CELLS_D_MIN_SCORE);
    let (mut low, mut low_failed, mut calls_base, mut calls_gated) = (0, 0, 0u32, 0u32);
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        for (c, _) in cands.iter().take(30) {
            let b = base.decode_candidate(&cells, c, DecodeDepth::EMBEDDED, 0.25);
            let sb = base.take_bp_stats();
            let g = gated.decode_candidate(&cells, c, DecodeDepth::EMBEDDED, 0.25);
            let sg = gated.take_bp_stats();
            let key = |f: &Option<mfsk_core::ft8::decode_block::Found>| {
                f.as_ref().map(|f| {
                    (
                        f.res.message77().to_vec(),
                        f.res.freq_hz.to_bits(),
                        f.res.dt_sec.to_bits(),
                    )
                })
            };
            calls_base += sb.calls;
            calls_gated += sg.calls;
            if c.score >= CELLS_D_MIN_SCORE {
                assert_eq!(
                    key(&g),
                    key(&b),
                    "{name}: score {} decodes as the default worker",
                    c.score
                );
                assert_eq!(
                    sg, sb,
                    "{name}: score {} costs what the default worker's does",
                    c.score
                );
            } else {
                low += 1;
                if g.is_some() {
                    assert_eq!(
                        key(&g).map(|k| k.0),
                        key(&b).map(|k| k.0),
                        "{name}: a gated decode is the default worker's"
                    );
                }
                assert!(
                    sg.calls <= sb.calls,
                    "{name}: score {}: {sg:?} against {sb:?}",
                    c.score
                );
                if b.is_none() {
                    low_failed += 1;
                    assert_eq!(
                        sg.calls * 2,
                        sb.calls,
                        "{name}: a failing candidate under the floor makes half the calls"
                    );
                }
            }
        }
    }
    assert!(
        low >= 20 && low_failed >= 10,
        "the recordings must have candidates under the floor: {low}, {low_failed}"
    );
    assert!(
        calls_gated < calls_base,
        "the floor saves BP calls: {calls_gated} against {calls_base}"
    );
}

/// `CellsWorker::with_gate(Some(&CELLS_GATE))`: on the three recordings' 30 candidates each, against a worker without the gate: **the same decodes** (message, frequency, DT: the gate only skips), and fewer BP
/// calls over all of them; a worker with no gate set decodes as `decode_candidate` does. **Fails if** the gate skips a candidate that decodes without it, never skips one (no call saved), or
/// changes what a worker without it does.
#[test]
fn the_gate_skips_hopeless_candidates_and_nothing_that_decodes() {
    use mfsk_core::ft8::decode_block::CELLS_GATE;
    let tables = CellTables::new();
    let mk = || -> (Box<[[Cmplx<f32>; 8]; 79]>, Box<Ft8BpScratch>) {
        (
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap(),
            Box::new(Ft8BpScratch::new()),
        )
    };
    let ((mut cs0, mut bp0), (mut cs1, mut bp1)) = (mk(), mk());
    let mut base = CellsWorker::new(&mut cs0, &mut bp0, &tables);
    let mut gated = CellsWorker::new(&mut cs1, &mut bp1, &tables).with_gate(Some(&CELLS_GATE));
    let (mut calls_base, mut calls_gated, mut skipped) = (0u32, 0u32, 0);
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        for (c, hs) in cands.iter().take(30) {
            let key = |f: &Option<mfsk_core::ft8::decode_block::Found>| {
                f.as_ref().map(|f| {
                    (
                        f.res.message77().to_vec(),
                        f.res.freq_hz.to_bits(),
                        f.res.dt_sec.to_bits(),
                    )
                })
            };
            let b = base.decode_candidate_hs(&cells, c, *hs, DecodeDepth::EMBEDDED, 0.25);
            let sb = base.take_bp_stats();
            // a worker with no gate: the hs-aware call is the plain one
            let plain = base.decode_candidate(&cells, c, DecodeDepth::EMBEDDED, 0.25);
            assert_eq!(key(&plain), key(&b), "{name}: no gate, the same decode");
            let _ = base.take_bp_stats();
            let g = gated.decode_candidate_hs(&cells, c, *hs, DecodeDepth::EMBEDDED, 0.25);
            let sg = gated.take_bp_stats();
            assert_eq!(
                key(&g),
                key(&b),
                "{name}: score {} hs {hs}: the gate must not change a decode",
                c.score
            );
            calls_base += sb.calls;
            calls_gated += sg.calls;
            skipped += (sg.calls < sb.calls) as usize;
        }
    }
    assert!(
        skipped >= 10 && calls_gated < calls_base,
        "the gate saves BP calls: {calls_gated} against {calls_base}, {skipped} candidates"
    );
}

/// The same with OSD on and `CELLS_GATE_OSD`: **the same decodes** as a worker without a gate on the three recordings, and fewer OSD runs and BP calls over all of them (`BpStats::osd_runs`). A gate fitted
/// without OSD is not the one to use here (it loses decodes OSD makes). **Fails if** the OSD-on gate skips a candidate that decodes, never skips one, or the OSD runs are not counted.
#[test]
fn the_osd_gate_skips_hopeless_candidates_and_saves_osd_runs() {
    use mfsk_core::ft8::decode_block::CELLS_GATE_OSD;
    let depth = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let tables = CellTables::new();
    let mk = || -> (Box<[[Cmplx<f32>; 8]; 79]>, Box<Ft8BpScratch>) {
        (
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap(),
            Box::new(Ft8BpScratch::new()),
        )
    };
    let ((mut cs0, mut bp0), (mut cs1, mut bp1)) = (mk(), mk());
    let mut base = CellsWorker::new(&mut cs0, &mut bp0, &tables);
    let mut gated = CellsWorker::new(&mut cs1, &mut bp1, &tables).with_gate(Some(&CELLS_GATE_OSD));
    let (mut osd_base, mut osd_gated) = (0u32, 0u32);
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        for (c, hs) in cands.iter().take(30) {
            let key = |f: &Option<mfsk_core::ft8::decode_block::Found>| {
                f.as_ref().map(|f| {
                    (
                        f.res.message77().to_vec(),
                        f.res.freq_hz.to_bits(),
                        f.res.dt_sec.to_bits(),
                    )
                })
            };
            let b = base.decode_candidate_hs(&cells, c, *hs, depth, 0.25);
            osd_base += base.take_bp_stats().osd_runs;
            let g = gated.decode_candidate_hs(&cells, c, *hs, depth, 0.25);
            osd_gated += gated.take_bp_stats().osd_runs;
            assert_eq!(
                key(&g),
                key(&b),
                "{name}: score {} hs {hs}: the OSD gate must not change a decode",
                c.score
            );
        }
    }
    assert!(osd_base >= 60, "the recordings make OSD runs: {osd_base}");
    assert!(
        osd_gated < osd_base,
        "the gate saves OSD runs: {osd_gated} against {osd_base}"
    );
}

/// `OsdSeeds::AD` (variants `a` and `d`, four OSD runs a candidate) against the faithful ten runs, OSD on, no gate: **the same decodes** on the three recordings. The runs it skips
/// (`b`, `c`, `e`) never returned on 4 780 slots (design note, OSD seeds), so a recording that loses a decode here is a new finding, not noise. **Fails if** the decode lists differ.
#[test]
fn osd_seeds_ad_decodes_what_the_ten_runs_do() {
    use mfsk_core::ft8::decode_block::OsdSeeds;
    let depth = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let tables = CellTables::new();
    let mk = || -> (Box<[[Cmplx<f32>; 8]; 79]>, Box<Ft8BpScratch>) {
        (
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap(),
            Box::new(Ft8BpScratch::new()),
        )
    };
    let ((mut cs0, mut bp0), (mut cs1, mut bp1)) = (mk(), mk());
    let mut faithful = CellsWorker::new(&mut cs0, &mut bp0, &tables);
    let mut ad = CellsWorker::new(&mut cs1, &mut bp1, &tables).with_osd_seeds(OsdSeeds::AD);
    let mut decoded = 0;
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        for (c, hs) in cands.iter().take(30) {
            let key = |f: &Option<mfsk_core::ft8::decode_block::Found>| {
                f.as_ref().map(|f| {
                    (
                        f.res.message77().to_vec(),
                        f.res.freq_hz.to_bits(),
                        f.res.dt_sec.to_bits(),
                    )
                })
            };
            let w = faithful.decode_candidate_hs(&cells, c, *hs, depth, 0.25);
            let x = ad.decode_candidate_hs(&cells, c, *hs, depth, 0.25);
            decoded += w.is_some() as u32;
            assert_eq!(key(&x), key(&w), "{name}: score {} hs {hs}", c.score);
        }
    }
    assert!(decoded >= 14, "the recordings decode with OSD: {decoded}");
}

/// The messages a worker decodes over the three recordings (30 candidates each, OSD seeds `a`, `d`), with the OSD at once (`None`), not at all (`Some(0)`), or deferred and
/// run for the best `k` of the slot (`Some(k)`); and how many attempts were queued.
fn trio_messages(osd: Option<Option<usize>>) -> (Vec<String>, usize) {
    use mfsk_core::ft8::decode_block::OsdSeeds;
    let depth = DecodeDepth {
        osd: osd.is_some(),
        ..DecodeDepth::EMBEDDED
    };
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
        .with_osd_seeds(OsdSeeds::AD)
        .with_osd_deferred(matches!(osd, Some(Some(_))));
    let (mut all, mut queued) = (Vec::new(), 0);
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        let mut known = Vec::new();
        w.decode_round(&cells, &cands, 30, depth, 0.25, &mut known);
        queued += w.osd_pending();
        if let Some(Some(k)) = osd {
            w.osd_pass(&cells, k, depth, &mut known);
            assert_eq!(w.osd_pending(), 0, "the pass empties the queue");
        }
        all.extend(texts(&known).into_iter().map(|t| format!("{name}: {t}")));
    }
    all.sort();
    (all, queued)
}

/// The deferred OSD: with every queued attempt run (`k` unbounded) the slot decodes what the OSD at once does; with none (`k = 0`) what the flow does without OSD; with the
/// best few, in between, and the ones it takes are the right ones (`k` = 2 / 6 / 8 keeps 1 / 2 / 3 of the OSD's 4 decodes here; the model's sign reversed keeps none of them). **Fails if** the unbounded
/// pass differs from the OSD at once, `k = 0` from no OSD, the cap's decodes are not a subset of the OSD's, or the best-`k` count falls short.
#[test]
fn a_deferred_osd_pass_runs_the_best_k_of_the_queue() {
    let (inline, _) = trio_messages(Some(None));
    let (off, _) = trio_messages(None);
    let (all, queued) = trio_messages(Some(Some(usize::MAX)));
    let (none, _) = trio_messages(Some(Some(0)));
    assert!(
        inline.len() > off.len(),
        "the OSD decodes more here: {} / {}",
        inline.len(),
        off.len()
    );
    assert!(queued > 20, "attempts were queued: {queued}");
    assert_eq!(all, inline, "every queued attempt run = the OSD at once");
    assert_eq!(none, off, "none run = no OSD");
    for k in [1usize, 2, 6, 8] {
        let (capped, _) = trio_messages(Some(Some(k)));
        assert!(
            off.iter().all(|m| capped.contains(m)),
            "k {k}: the BP decodes stay"
        );
        assert!(
            capped.iter().all(|m| inline.contains(m)),
            "k {k}: nothing the OSD at once does not decode"
        );
        // measured on these recordings, k 1 / 2 / 6 / 8 -> 19 / 20 / 21 / 22 decodes (OSD off 19, at once 23); with the rank reversed 19 at every k
        let at_least = match k {
            1 => 0,
            2 => 1,
            6 => 2,
            _ => 3,
        };
        assert!(
            capped.len() >= off.len() + at_least,
            "k {k}: {} decodes, OSD off {}",
            capped.len(),
            off.len()
        );
    }
}

/// `decode_round_osd` over the three recordings: **fails if** `k = 0` decodes anything `decode_round` without OSD does not (or misses one it does), unbounded `k`
/// differs from `decode_round` with the OSD at once, or the worker's own deferral setting is not restored.
#[test]
fn decode_round_osd_is_decode_round_at_the_two_ends() {
    use mfsk_core::ft8::decode_block::OsdSeeds;
    let osd = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let off = DecodeDepth::EMBEDDED;
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables).with_osd_seeds(OsdSeeds::AD);
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        let run = |w: &mut CellsWorker<'_>,
                   f: &dyn Fn(
            &mut CellsWorker<'_>,
            &mut Vec<mfsk_core::ft8::decode_block::Found>,
        )| {
            let mut known = Vec::new();
            f(w, &mut known);
            let mut t = texts(&known);
            t.sort();
            t
        };
        let none = run(&mut w, &|w, k| {
            w.decode_round(&cells, &cands, 30, off, 0.25, k);
        });
        let at_once = run(&mut w, &|w, k| {
            w.decode_round(&cells, &cands, 30, osd, 0.25, k);
        });
        let k0 = run(&mut w, &|w, k| {
            w.decode_round_osd(&cells, &cands, 30, osd, 0.25, k, 0);
        });
        let all = run(&mut w, &|w, k| {
            w.decode_round_osd(&cells, &cands, 30, osd, 0.25, k, usize::MAX);
        });
        assert_eq!(k0, none, "{name}: k = 0 is decode_round without OSD");
        assert_eq!(
            all, at_once,
            "{name}: unbounded k is decode_round with the OSD at once"
        );
        assert_eq!(
            w.osd_pending(),
            0,
            "{name}: the queue is empty after the round"
        );
        // and the worker is not left deferring: a plain round with the OSD runs it at once
        let again = run(&mut w, &|w, k| {
            w.decode_round(&cells, &cands, 30, osd, 0.25, k);
        });
        assert_eq!(again, at_once, "{name}: the deferral setting is restored");
    }
}

/// `sic_passes_osd`: **fails if** `k = 0` differs from `sic_passes` without OSD, unbounded `k` from `sic_passes` with the OSD at once, a capped `k = 6` is not between
/// them (it must hold every decode of `k = 0` and none that unbounded `k` lacks), or the OSD of the first pass decodes nothing the flow returns (the pass-0 OSD decodes
/// must be in the result: they are what the next pass subtracts).
#[test]
fn sic_passes_osd_is_sic_passes_at_the_two_ends_and_between_them_in_the_middle() {
    use mfsk_core::ft8::decode::DecodeResult;
    use mfsk_core::ft8::decode_block::{
        OsdSeeds, SicStorage, sic_passes, sic_passes_osd, sic_signal_capacity,
    };
    let osd = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let off = DecodeDepth::EMBEDDED;
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables).with_osd_seeds(OsdSeeds::AD);
    let (max_cand, passes) = (30usize, 2usize);
    let mut storage = SicStorage::new(Ft8Rate::Hz8000, sic_signal_capacity(max_cand, passes));
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let msgs = |r: &[DecodeResult]| {
        let mut t: Vec<String> = r.iter().filter_map(|d| unpack77(d.message77())).collect();
        t.sort();
        t
    };
    let (mut total_off, mut total_osd, mut total_6) = (0, 0, 0);
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let mut run = |w: &mut CellsWorker<'_>, cap: Option<usize>, depth: DecodeDepth| {
            let (cands, mut cells) =
                coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1).unwrap();
            match cap {
                None => sic_passes(
                    &mut SliceSource(&a),
                    Ft8Rate::Hz8000,
                    &search,
                    max_cand,
                    depth,
                    passes,
                    &cands,
                    &mut cells,
                    &tables,
                    w,
                    &mut storage.scratch(),
                ),
                Some(k) => sic_passes_osd(
                    &mut SliceSource(&a),
                    Ft8Rate::Hz8000,
                    &search,
                    max_cand,
                    depth,
                    passes,
                    &cands,
                    &mut cells,
                    &tables,
                    w,
                    &mut storage.scratch(),
                    k,
                ),
            }
        };
        let none = msgs(&run(&mut w, None, off));
        let at_once = msgs(&run(&mut w, None, osd));
        let k0 = msgs(&run(&mut w, Some(0), osd));
        let all = msgs(&run(&mut w, Some(usize::MAX), osd));
        let k6 = msgs(&run(&mut w, Some(6), osd));
        assert_eq!(k0, none, "{name}: k = 0 is the SIC without OSD");
        assert_eq!(
            all, at_once,
            "{name}: unbounded k is the SIC with the OSD at once"
        );
        assert!(
            none.iter().all(|m| k6.contains(m)),
            "{name}: k = 6 keeps the BP decodes"
        );
        assert!(
            k6.iter().all(|m| at_once.contains(m)),
            "{name}: k = 6 decodes nothing the OSD at once does not"
        );
        total_off += none.len();
        total_osd += at_once.len();
        total_6 += k6.len();
    }
    assert!(
        total_osd > total_off,
        "the OSD decodes more here: {total_osd} / {total_off}"
    );
    assert!(
        total_6 > total_off,
        "the capped OSD in the SIC decodes some of them: {total_6} / {total_off}"
    );
}

/// The two-core OSD pass, with the two cores played in turn: the round's candidates dealt out between two workers (even / odd index), each queueing its failed attempts, the
/// attempts gathered, ranked and the best `k` dealt out between the same two workers, the decodes merged in rank order. **Fails if** the set of decodes differs from one worker's
/// `decode_round_osd` for `k` = 0, 6 and unbounded, or `rank_osd_attempts` does not keep exactly the `k` best scores in order.
#[test]
fn the_osd_pass_split_between_two_workers_equals_one_workers() {
    use mfsk_core::ft8::decode_block::{Found, OsdSeeds, rank_osd_attempts};
    let osd = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let tables = CellTables::new();
    let mk = || -> (Box<[[Cmplx<f32>; 8]; 79]>, Box<Ft8BpScratch>) {
        (
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap(),
            Box::new(Ft8BpScratch::new()),
        )
    };
    let ((mut cs0, mut bp0), (mut cs1, mut bp1), (mut cs2, mut bp2)) = (mk(), mk(), mk());
    let mut one = CellsWorker::new(&mut cs0, &mut bp0, &tables).with_osd_seeds(OsdSeeds::AD);
    let mut w0 = CellsWorker::new(&mut cs1, &mut bp1, &tables)
        .with_osd_seeds(OsdSeeds::AD)
        .with_osd_deferred(true);
    let mut w1 = CellsWorker::new(&mut cs2, &mut bp2, &tables)
        .with_osd_seeds(OsdSeeds::AD)
        .with_osd_deferred(true);
    let mut ranked_checks = 0;
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        for k in [0usize, 6, usize::MAX] {
            let mut want = Vec::new();
            one.decode_round_osd(&cells, &cands, 30, osd, 0.25, &mut want, k);
            // the round, two workers: candidate i to worker i % 2, the decodes merged by candidate index
            let mut found: Vec<(usize, Option<Found>)> = Vec::new();
            for (i, (c, hs)) in cands.iter().take(30).enumerate() {
                let w = if i % 2 == 0 { &mut w0 } else { &mut w1 };
                found.push((i, w.decode_candidate_hs(&cells, c, *hs, osd, 0.25)));
            }
            let mut known: Vec<Found> = Vec::new();
            for (_, f) in found {
                if let Some(f) = f
                    && !known.iter().any(|o| o.res.message77() == f.res.message77())
                {
                    known.push(f);
                }
            }
            // the OSD pass: gather, rank, deal out
            let mut att = w0.take_osd_attempts();
            att.extend(w1.take_osd_attempts());
            let mut scores: Vec<f32> = att.iter().map(|x| x.logit()).collect();
            scores.sort_by(|a, b| b.partial_cmp(a).unwrap());
            rank_osd_attempts(&mut att, k);
            let kept: Vec<f32> = att.iter().map(|x| x.logit()).collect();
            assert_eq!(
                kept[..],
                scores[..scores.len().min(k)],
                "{name} k {k}: the best scores, in order"
            );
            ranked_checks += 1;
            let mut ran: Vec<(usize, Option<Found>)> = Vec::new();
            for (r, x) in att.iter().enumerate() {
                let w = if r % 2 == 0 { &mut w0 } else { &mut w1 };
                ran.push((r, w.run_osd_attempt(&cells, x, osd)));
            }
            for (_, f) in ran {
                if let Some(f) = f
                    && !known.iter().any(|o| o.res.message77() == f.res.message77())
                {
                    known.push(f);
                }
            }
            let (mut got, mut want) = (texts(&known), texts(&want));
            got.sort();
            want.sort();
            assert_eq!(got, want, "{name} k {k}: two workers = one worker");
        }
    }
    assert_eq!(ranked_checks, 9);
}

/// A worker that made the OSD fallback's memory up front (`reserve_osd_work`) decodes what one that makes it on its first use does. **Fails if** the decodes differ over the three
/// recordings at K = 6 (the reserved scratch must be the one the fallback uses, with its buffers already grown).
#[test]
fn a_worker_that_reserved_the_osd_memory_decodes_the_same() {
    use mfsk_core::ft8::decode_block::OsdSeeds;
    let osd = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let tables = CellTables::new();
    let mk = || -> (Box<[[Cmplx<f32>; 8]; 79]>, Box<Ft8BpScratch>) {
        (
            vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap(),
            Box::new(Ft8BpScratch::new()),
        )
    };
    let ((mut cs0, mut bp0), (mut cs1, mut bp1)) = (mk(), mk());
    let mut lazy = CellsWorker::new(&mut cs0, &mut bp0, &tables).with_osd_seeds(OsdSeeds::AD);
    let mut made = CellsWorker::new(&mut cs1, &mut bp1, &tables).with_osd_seeds(OsdSeeds::AD);
    made.reserve_osd_work();
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        let (cands, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .unwrap();
        let (mut x, mut y) = (Vec::new(), Vec::new());
        lazy.decode_round_osd(&cells, &cands, 30, osd, 0.25, &mut x, 6);
        made.decode_round_osd(&cells, &cands, 30, osd, 0.25, &mut y, 6);
        let (mut x, mut y) = (texts(&x), texts(&y));
        x.sort();
        y.sort();
        assert_eq!(x, y, "{name}");
        assert!(x.len() >= 4, "{name}: decodes: {}", x.len());
    }
}

/// `(message, frequency bits, DT bits, SNR bits)` of each decode: what a preset must reproduce exactly.
fn full_keys(r: &[mfsk_core::ft8::decode::DecodeResult]) -> Vec<(Option<String>, u32, u32, u32)> {
    r.iter()
        .map(|d| {
            (
                unpack77(d.message77()),
                d.freq_hz.to_bits(),
                d.dt_sec.to_bits(),
                d.snr_db.to_bits(),
            )
        })
        .collect()
}

/// `CellsOptions` and `decode_block_cells_with`: each preset gives, on the three recordings, **exactly** what the chain it stands for gives (messages, frequency, DT and SNR, bit for bit): `PLAIN` is
/// `decode_block_cells`; `p0_window()` the worker with the `d` floor 16, `CELLS_GATE_OSD`, seeds `AD`, the OSD deferred, `decode_round_osd(k = 6)` and the SNR baseline; `p0_late()` the same
/// under `sic_passes_osd` (2 passes, k = 6); an `Inline` OSD the worker's plain `decode_round` with the OSD on. **Fails if** any option is read wrongly (a floor, the gate, the seeds, the cap, the
/// passes) or the one-shot entry does something the chain does not.
#[test]
fn the_presets_are_the_chains_they_stand_for() {
    use mfsk_core::ft8::decode_block::{
        CELLS_GATE_OSD, CellsOptions, OsdOptions, OsdSeeds, SicStorage, SnrBaseline,
        decode_block_cells, decode_block_cells_with, sic_passes_osd, sic_signal_capacity,
    };
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 30);
    let tables = CellTables::new();
    for name in ["qso3_busy", "qso1", "qso2"] {
        let a = audio(name);
        // PLAIN = decode_block_cells
        let plain =
            decode_block_cells(&a, Ft8Rate::Hz8000, &search, 30, DecodeDepth::EMBEDDED).unwrap();
        let got =
            decode_block_cells_with(&a, Ft8Rate::Hz8000, &search, &CellsOptions::PLAIN).unwrap();
        assert_eq!(
            full_keys(&got),
            full_keys(&plain),
            "{name}: PLAIN is decode_block_cells"
        );

        let (cands, mut cells) =
            coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search, 1).unwrap();
        let mk = || -> (Box<[[Cmplx<f32>; 8]; 79]>, Box<Ft8BpScratch>) {
            (
                vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap(),
                Box::new(Ft8BpScratch::new()),
            )
        };
        let osd = DecodeDepth {
            osd: true,
            ..DecodeDepth::EMBEDDED
        };

        // p0_window = the chain, with the SNR applied
        let (mut cs, mut bp) = mk();
        let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
            .with_d_min_score(16.0)
            .with_gate(Some(&CELLS_GATE_OSD))
            .with_osd_seeds(OsdSeeds::AD)
            .with_osd_deferred(true);
        let mut known = Vec::new();
        w.decode_round_osd(&cells, &cands, 30, osd, 0.25, &mut known, 6);
        let base = SnrBaseline::new(&cells, &tables);
        for f in &mut known {
            base.apply(&cells, &tables, f);
        }
        let want: Vec<_> = known.into_iter().map(|f| f.res).collect();
        let got = decode_block_cells_with(&a, Ft8Rate::Hz8000, &search, &CellsOptions::p0_window())
            .unwrap();
        assert_eq!(full_keys(&got), full_keys(&want), "{name}: p0_window");
        assert!(want.len() >= 4, "{name}: decodes {}", want.len());

        // an inline OSD = the worker's plain round with the OSD on
        let (mut cs, mut bp) = mk();
        let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
            .with_d_min_score(16.0)
            .with_osd_seeds(OsdSeeds::AD);
        let mut known = Vec::new();
        w.decode_round(&cells, &cands, 30, osd, 0.25, &mut known);
        let base = SnrBaseline::new(&cells, &tables);
        for f in &mut known {
            base.apply(&cells, &tables, f);
        }
        let want: Vec<_> = known.into_iter().map(|f| f.res).collect();
        let opts = CellsOptions {
            gate: None,
            osd: OsdOptions::Inline {
                seeds: OsdSeeds::AD,
            },
            ..CellsOptions::PLAIN
        };
        let got = decode_block_cells_with(&a, Ft8Rate::Hz8000, &search, &opts).unwrap();
        assert_eq!(full_keys(&got), full_keys(&want), "{name}: an inline OSD");

        // p0_late = sic_passes_osd, 2 passes, k = 6
        let (mut cs, mut bp) = mk();
        let mut w = CellsWorker::new(&mut cs, &mut bp, &tables)
            .with_d_min_score(16.0)
            .with_gate(Some(&CELLS_GATE_OSD))
            .with_osd_seeds(OsdSeeds::AD)
            .with_osd_deferred(true);
        let mut storage = SicStorage::new(Ft8Rate::Hz8000, sic_signal_capacity(30, 2));
        let want = sic_passes_osd(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &search,
            30,
            osd,
            2,
            &cands,
            &mut cells,
            &tables,
            &mut w,
            &mut storage.scratch(),
            6,
        );
        let got = decode_block_cells_with(&a, Ft8Rate::Hz8000, &search, &CellsOptions::p0_late())
            .unwrap();
        assert_eq!(full_keys(&got), full_keys(&want), "{name}: p0_late");
    }
}
