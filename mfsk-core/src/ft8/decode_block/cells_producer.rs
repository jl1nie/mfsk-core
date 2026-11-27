//! The half-dB cells **made while the slot is being captured** (the CoreS3's producer, `docs/notes/S3_CELLS_IMPLEMENTATION_PLAN.md` section 4.3, `PROFILES.md` section 10).
//!
//! [`coarse_sync_gated_cells`](super::coarse_sync_gated_cells) reads the whole slot and makes the store in one go. A board that decodes before the slot has ended has the audio **a block at a time**:
//! [`CellsProducer::advance`] makes every pair of rows whose frames are fully in the audio so far, writes them into the store and marks them filled (`HalfDbCells::mark_filled`), so
//! that the search ([`search_cells`](super::search_cells)) and the decode of the candidates that are [`ready`](super::candidate_ready) can run on a store that is still growing, and
//! [`finish`](CellsProducer::finish) makes the rest (the last frames reach past the slot's end, where the audio is zero) at the slot's end.
//!
//! It is **not a second demux**: [`Framer`] and [`demux_pair`] are the code of the whole-slot demux (`spectrogram_cells_into`), so the cells are bit for bit the whole-slot flow's
//! for the same audio and the same gain (`tests/cells_producer.rs`). The one thing it cannot have is the slot's peak (the gain `shift`): the caller gives it, from the first second of
//! the slot (what `stage1_inc` does) or the previous slot's. A row never reads a sample the caller has not yet given.

use alloc::boxed::Box;
use alloc::vec;
use alloc::vec::Vec;

use num_complex::Complex;

use crate::engine::fft::Fft16;

use super::coarse_lib::{HalfDbCells, TONE_BINS, carrier_bins_of};
use super::coarse_sync::GatedSearch;
use super::spectrogram::{Framer, HalfDb, demux_pair, gain_for_peak, hann_q15};
use super::types::Ft8Rate;

/// The gain `shift` of a slot whose largest sample magnitude is `peak_abs` (what the whole-slot flow reads from the slot): the peak lands at twice the FFT size, at most 8 doublings.
pub fn gain_shift_for_peak(peak_abs: i32) -> u32 {
    gain_for_peak(peak_abs)
}

/// A producer of one slot's cells. Reusable: [`restart`](Self::restart) for the next slot.
pub struct CellsProducer {
    rate: Ft8Rate,
    /// Bins of the demux band (`first_bin ..`), as `coarse_sync_gated_cells` makes them.
    lo: usize,
    hi: usize,
    shift: u32,
    store: HalfDbCells,
    fft: Box<dyn Fft16>,
    win_q15: Vec<i16>,
    buf: Vec<Complex<i16>>,
    fa: Vec<i16>,
    fb: Vec<i16>,
    /// Pairs of rows made.
    pairs: usize,
}

impl CellsProducer {
    /// A producer for `search`'s band at `rate` with the gain `shift`. `None` unless `rate` is the 8 kHz cells rate (`Ft8Rate::coarse_ft8lib`) and the band is not empty.
    pub fn new(rate: Ft8Rate, search: &GatedSearch, shift: u32) -> Option<Self> {
        if !rate.coarse_ft8lib() {
            return None;
        }
        let (ia, ib) = carrier_bins_of(search.freq_min, search.freq_max, rate)?;
        let (lo, hi) = (ia, ib + TONE_BINS);
        let (nfft, n_time) = (rate.nfft(), rate.nmax() / rate.nstep() - 3);
        assert!(
            n_time % 2 == 0,
            "the producer makes pairs of rows: an even slot"
        );
        let flen = if rate.coarse_hann_2t() {
            nfft
        } else {
            rate.nsps()
        };
        let mut planner = crate::engine::fft::default_planner_16();
        Some(Self {
            rate,
            lo,
            hi,
            shift,
            store: HalfDbCells::new(rate, ia, hi - lo, n_time),
            fft: planner.plan_forward(nfft),
            win_q15: if rate.coarse_hann_2t() {
                hann_q15(nfft)
            } else {
                Vec::new()
            },
            buf: vec![Complex::new(0i16, 0i16); nfft],
            fa: vec![0i16; flen],
            fb: vec![0i16; flen],
            pairs: 0,
        })
    }

    /// Begin the next slot with the gain `shift`: the store is emptied (no row filled), the memory kept.
    pub fn restart(&mut self, shift: u32) {
        self.shift = shift;
        self.pairs = 0;
        self.store.clear();
    }

    /// The cells so far (`filled_rows` of them).
    pub fn store(&self) -> &HalfDbCells {
        &self.store
    }

    /// The store, for a search that writes into it (the subtraction passes).
    pub fn store_mut(&mut self) -> &mut HalfDbCells {
        &mut self.store
    }

    /// Pairs of rows made.
    pub fn pairs(&self) -> usize {
        self.pairs
    }

    /// Rows in the slot.
    pub fn rows(&self) -> usize {
        self.store.n_time()
    }

    /// The last sample (exclusive) the frames of pair `jj` read.
    fn pair_end(&self, jj: usize) -> usize {
        let nstep = self.rate.nstep();
        let lead = if self.rate.coarse_hann_2t() {
            self.rate.nsps() / 2
        } else {
            0
        };
        let flen = self.fa.len();
        (2 * jj + 1) * nstep + flen - lead
    }

    fn make_pair(&mut self, audio: &[i16], jj: usize) {
        let nstep = self.rate.nstep();
        let hann = self.rate.coarse_hann_2t();
        let lead = if hann { self.rate.nsps() / 2 } else { 0 };
        let (gain_num, gain_den) = self.rate.input_gain();
        let flen = self.fa.len();
        let read = |start: i64, out: &mut [i16]| {
            for (i, o) in out.iter_mut().enumerate() {
                let idx = start + i as i64;
                *o = if idx >= 0 && (idx as usize) < audio.len() {
                    audio[idx as usize]
                } else {
                    0
                };
            }
        };
        read((2 * jj * nstep) as i64 - lead as i64, &mut self.fa);
        read(((2 * jj + 1) * nstep) as i64 - lead as i64, &mut self.fb);
        let framer = Framer {
            flen,
            shift: self.shift,
            win_q15: &self.win_q15,
            hann,
            gain_num,
            gain_den,
        };
        framer.pack(&mut self.buf, &self.fa, Some(&self.fb));
        self.fft.process(&mut self.buf);
        let (ra, rb) = self.store.row_pair_mut(2 * jj);
        demux_pair(
            &self.buf,
            self.rate.nfft(),
            self.lo,
            self.hi,
            HalfDb,
            ra,
            rb,
        );
        self.pairs = jj + 1;
        self.store.mark_filled(2 * (jj + 1));
    }

    /// Make every pair of rows whose frames lie entirely in `audio[..available]` (the slot's samples so far, from the slot's start; `audio` may be longer, its samples past `available` are
    /// **not read**). Returns the rows filled.
    pub fn advance(&mut self, audio: &[i16], available: usize) -> usize {
        let n_pairs = self.rows() / 2;
        let avail = available.min(audio.len());
        while self.pairs < n_pairs && self.pair_end(self.pairs) <= avail {
            let jj = self.pairs;
            self.make_pair(&audio[..avail], jj);
        }
        self.store.filled_rows()
    }

    /// The slot has ended with `audio` (its samples; fewer than a slot's are read as zero beyond them): make the remaining pairs and the odd row, if any. Returns the rows filled (all).
    pub fn finish(&mut self, audio: &[i16]) -> usize {
        let n_pairs = self.rows() / 2;
        while self.pairs < n_pairs {
            let jj = self.pairs;
            self.make_pair(audio, jj);
        }
        self.store.filled_rows()
    }
}
