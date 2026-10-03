//! JT65 message averaging — a port of `jt65_decode.f90`'s `avg65` for the HF
//! sub-modes (`ndepth & 16`, "Enable averaging" in the WSJT-X GUI).
//!
//! A candidate the single-period decode fails on is remembered with its
//! period, DT, frequency and its 63 × 64 symbol powers (upstream `s3a`/`s1`,
//! at most [`CAPACITY`] = `MAXAVE`). The saved periods of the same parity
//! whose DT is within 0.2 s and frequency within `ntol` of the new one are
//! summed, and with two or more the sum is decoded as one period would be
//! (`extract`: the stochastic Chase search of [`super::chase`]).
//!
//! Not ported: the `ismo` smoothing loop, which only runs for JT65B/C
//! (`mode65 >= 2`), and the `nflip` sync-type test (HF has one sync pattern).
//! `demod64a.f90` forms its probabilities as `s1/psum`, so the sum needs no
//! rescaling by the number of periods.
//!
//! Memory: one entry is 63 × 64 `f32` = 16 KB, so a full set is 1 MB. JT65
//! needs `std`/`fft`, so this is not an embedded cost.

use alloc::boxed::Box;
use alloc::vec::Vec;

use crate::msg::Jt72Message;

use super::chase::{ChaseParams, decode_demod_with_chase};
use super::rx;

/// `MAXAVE` in `jt65_decode.f90`.
pub const CAPACITY: usize = 64;
/// `dtdiff` in `avg65`, seconds.
const DT_TOLERANCE_S: f32 = 0.2;

struct Entry {
    period: i64,
    dt: f32,
    freq: f32,
    pwr: Box<[[f32; 64]; 63]>,
}

/// What a [`super::Jt65`] decoder keeps between periods.
#[derive(Default)]
pub struct Averager {
    entries: Vec<Entry>,
    /// Ring position of the next overwrite once full.
    next: usize,
    /// `nutc0`/`nfreq0`: avg65 runs once per period and frequency.
    last: Option<(i64, f32)>,
}

impl Averager {
    /// Forget everything (`clearave`, the GUI's "Clear Avg").
    pub fn clear(&mut self) {
        self.entries.clear();
        self.next = 0;
        self.last = None;
    }

    /// Periods held.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// `avg65` for a candidate whose single-period decode failed: save it,
    /// sum what matches, decode the sum. Returns the message with the
    /// candidate's own `snr_db` (upstream reports the candidate's SNR and the
    /// number of periods summed) and the info words.
    pub(super) fn try_average(
        &mut self,
        period: i64,
        dt: f32,
        freq: f32,
        ntol: f32,
        demod: rx::Jt65Demod,
        chase: &ChaseParams,
    ) -> Option<(Jt72Message, f32, [u8; 12])> {
        if let Some((p, f)) = self.last
            && p == period
            && (freq - f).abs() <= ntol
        {
            return None;
        }
        self.last = Some((period, freq));

        // The same period and frequency is not saved twice (`avg65`: the
        // slot is given back), but it still takes part in the sum.
        let duplicate = self
            .entries
            .iter()
            .any(|e| e.period == period && (freq - e.freq).abs() <= ntol);
        if !duplicate {
            let e = Entry {
                period,
                dt,
                freq,
                pwr: Box::new(demod.raw_pwr),
            };
            if self.entries.len() < CAPACITY {
                self.entries.push(e);
            } else {
                self.entries[self.next] = e;
                self.next = (self.next + 1) % CAPACITY;
            }
        }

        let mut sum = [[0f32; 64]; 63];
        let mut nsum = 0usize;
        for e in &self.entries {
            if e.period.rem_euclid(2) != period.rem_euclid(2)
                || (dt - e.dt).abs() > DT_TOLERANCE_S
                || (freq - e.freq).abs() > ntol
            {
                continue;
            }
            for (row, add) in sum.iter_mut().zip(e.pwr.iter()) {
                for (s, a) in row.iter_mut().zip(add.iter()) {
                    *s += *a;
                }
            }
            nsum += 1;
        }
        if nsum < 2 {
            return None;
        }
        let summed = rx::from_pwr(&sum);
        let snr_db = demod.snr_db;
        decode_demod_with_chase(summed, chase).map(|(msg, _, info)| (msg, snr_db, info))
    }
}
