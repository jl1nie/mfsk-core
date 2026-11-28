//! `CellsProducer`, the cells made while the slot is being captured, against `coarse_sync_gated_cells`, which makes them from the whole slot: **bit for bit the same cells** for the same audio
//! and gain, in whatever blocks the audio arrives, and **no row reads a sample that has not arrived**.
//!
//! **Fails if** a cell differs from the whole-slot store's (a second demux that drifted from the first), if a row written before the slot's end depends on audio past `available` (the test
//! fills that audio with noise and compares the rows made so far with the clean run), if the number of rows available at a given sample count changes (the early emit's row pair 87 is a sample count),
//! or if a producer reused for a second slot (`restart`) makes other cells than a fresh one.
#![cfg(all(feature = "fixed-point", feature = "internal-testing"))]
use mfsk_core::engine::dsp::resample::resample_to_8k;
use mfsk_core::engine::source::SliceSource;
use mfsk_core::ft8::decode_block::{
    CellsProducer, Ft8Rate, GatedSearch, HalfDbCells, coarse_sync_gated_cells, gain_shift_for_peak,
};

fn audio(name: &str) -> Vec<i16> {
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
    resample_to_8k(&s, 12_000)
}

fn search() -> GatedSearch {
    GatedSearch::new(200.0, 3000.0, 1.3, 30)
}

fn peak(a: &[i16]) -> i32 {
    a.iter().map(|&x| (x as i32).abs()).max().unwrap_or(0)
}

fn same_rows(a: &HalfDbCells, b: &HalfDbCells, rows: usize, what: &str) {
    assert_eq!(
        (a.first_bin(), a.n_freq(), a.n_time()),
        (b.first_bin(), b.n_freq(), b.n_time()),
        "{what}: the shape"
    );
    for r in 0..rows {
        for bin in a.first_bin()..a.first_bin() + a.n_freq() {
            assert_eq!(a.cell(r, bin), b.cell(r, bin), "{what}: row {r} bin {bin}");
        }
    }
}

#[test]
fn the_streamed_cells_are_the_whole_slots_cells() {
    for name in ["qso3_busy", "qso1"] {
        let a = audio(name);
        let (_, whole) =
            coarse_sync_gated_cells(&mut SliceSource(&a), Ft8Rate::Hz8000, &search(), 1).unwrap();
        let shift = gain_shift_for_peak(peak(&a));
        let mut p = CellsProducer::new(Ft8Rate::Hz8000, &search(), shift).unwrap();
        // the audio arrives in blocks of 1 000 samples (a CoreS3 chunk is 100 ms = 800)
        let mut have = 0;
        while have < a.len() {
            have = (have + 1000).min(a.len());
            p.advance(&a, have);
        }
        assert_eq!(p.finish(&a), p.rows());
        same_rows(p.store(), &whole, whole.n_time(), name);
        // a slot whose audio stops short (a dropped block at the end): the last pair waits for `finish`, which reads zeros there, as the whole-slot flow does
        let short = &a[..a.len() - 1_000];
        let (_, whole) =
            coarse_sync_gated_cells(&mut SliceSource(short), Ft8Rate::Hz8000, &search(), 1)
                .unwrap();
        let mut p = CellsProducer::new(Ft8Rate::Hz8000, &search(), shift).unwrap();
        assert!(
            p.advance(short, short.len()) < p.rows(),
            "{name}: the last pair waits for finish"
        );
        p.finish(short);
        same_rows(p.store(), &whole, whole.n_time(), "short slot");
    }
}

#[test]
fn a_row_never_reads_audio_that_has_not_arrived() {
    let a = audio("qso3_busy");
    let shift = gain_shift_for_peak(peak(&a));
    let mut clean = CellsProducer::new(Ft8Rate::Hz8000, &search(), shift).unwrap();
    let available = 60_000;
    let rows = clean.advance(&a, available);
    // the same, with noise where the audio has not arrived yet
    let mut dirty_audio = a.clone();
    for (i, x) in dirty_audio.iter_mut().enumerate().skip(available) {
        *x = ((i * 7919) % 20_000) as i16 - 10_000;
    }
    let mut dirty = CellsProducer::new(Ft8Rate::Hz8000, &search(), shift).unwrap();
    assert_eq!(
        dirty.advance(&dirty_audio, available),
        rows,
        "the rows available depend on the sample count only"
    );
    same_rows(clean.store(), dirty.store(), rows, "60 000 samples");
    // a pair of rows is there once its second frame is: the pair jj reads up to (2 jj + 1) x 640 + 1 920 (8 kHz, nstep 640, a 2 560 sample Hann frame that starts 640 early)
    assert_eq!(rows, 90, "rows at 7.5 s");
}

#[test]
fn row_pair_87_needs_14_08_seconds_of_audio() {
    // the early emit's row pair 87 (174 rows; design note section 31) is a sample count: its frames read up to 173 x 640 + 1 920 = 112 640 samples, 14.08 s, 0.92 s before the slot's end
    let a = audio("qso1");
    let shift = gain_shift_for_peak(peak(&a));
    let mut p = CellsProducer::new(Ft8Rate::Hz8000, &search(), shift).unwrap();
    assert_eq!(p.advance(&a, 112_639), 172);
    assert_eq!(p.advance(&a, 112_640), 174);
}

#[test]
fn a_restarted_producer_makes_the_cells_of_a_fresh_one() {
    let (a, b) = (audio("qso3_busy"), audio("qso2"));
    let mut p =
        CellsProducer::new(Ft8Rate::Hz8000, &search(), gain_shift_for_peak(peak(&a))).unwrap();
    p.advance(&a, a.len());
    p.finish(&a);
    p.restart(gain_shift_for_peak(peak(&b)));
    assert_eq!(p.store().filled_rows(), 0);
    p.advance(&b, b.len());
    p.finish(&b);
    let (_, whole) =
        coarse_sync_gated_cells(&mut SliceSource(&b), Ft8Rate::Hz8000, &search(), 1).unwrap();
    same_rows(p.store(), &whole, whole.n_time(), "restarted");
}

/// `copy_from` makes a second store the first's (the cells and the rows filled) without allocating, for the subtraction passes that write their residual into the store they get.
/// **Fails if** a cell or the fill differs after the copy, or a store of another shape is accepted.
#[test]
fn a_copied_store_is_the_store() {
    let a = audio("qso1");
    let mut p =
        CellsProducer::new(Ft8Rate::Hz8000, &search(), gain_shift_for_peak(peak(&a))).unwrap();
    p.advance(&a, 70_000);
    let s = p.store();
    let mut spare = HalfDbCells::new(Ft8Rate::Hz8000, s.first_bin(), s.n_freq(), s.n_time());
    spare.copy_from(s);
    assert_eq!(spare.filled_rows(), s.filled_rows());
    same_rows(&spare, s, s.n_time(), "copy");
    let wrong = std::panic::catch_unwind(|| {
        let mut other = HalfDbCells::new(Ft8Rate::Hz8000, 0, 10, 4);
        other.copy_from(&HalfDbCells::new(Ft8Rate::Hz8000, 0, 11, 4));
    });
    assert!(wrong.is_err(), "another shape is refused");
}
