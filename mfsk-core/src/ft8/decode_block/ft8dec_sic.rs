// SPDX-License-Identifier: GPL-3.0-only
//! `ft8dec`'s streaming subtraction, for the cells flow: the audio **minus every signal decoded so far**, served as a
//! [`PcmSource`] that the byte demux reads forward, so no copy of the slot is made, only a ring of one and a half symbols
//! beyond a frame.
//!
//! **A deliberate divergence from WSJT-X** (`subtractft8.f90`: an FFT-based low-pass of the correlation of the whole slot with each
//! signal's reference). Ported from `ft8dec.c` (icap-sdk 1ca9f04, MIT), as the file names its lines in the comments below: each
//! decoded signal's GFSK reference (BT 2, the `gstep` transition of 24/128 of a symbol each side of a boundary) is correlated, symbol
//! by symbol, with the **input** chunk (`sig_corr`, 243-279), and subtracted, with a complex amplitude **linearly interpolated between
//! symbol centres**, from the **output** chunk 1.5 symbols behind (`sig_sub`, 280-319); `build_wf` (321-379) is the loop that
//! drives them, here [`ResidualSource::read`]. After the whole slot has been read, the frequency of each signal is corrected from the
//! phase steps between its symbols (`build_wf` 373-377: [`ResidualSource::into_signals`]).
//!
//! Why: the exact subtraction the library has (`subtract_tones_lpf`) costs about 18 M instructions a signal on the host (two full-length
//! FFTs); this one has no FFT: a few operations a sample over the signal's own span
//! (`docs/notes/FT8_WINDOW_AND_COARSE_DESIGN.md`, 22-23).

use alloc::vec;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std (the dev-only rustfft) makes f32's own methods shadow it
use num_traits::Float;

use super::super::decode::{DecodeDepth, DecodeResult};
use super::super::params::NN;
use super::cells_flow::{CellTables, Found, RETRY_MIN_SHIFT, View, decode_round};
use super::coarse_lib::{coarse_sync_gated_cells, search_near};
use super::coarse_sync::GatedSearch;
use super::spectrogram::spectrogram_gain;
use super::types::{AudioSample, Ft8Rate, SAMPLE_RATE_HZ};
use crate::engine::source::{PcmSource, SliceSource};
use crate::engine::tx::message_to_tones;

/// Tone spacing, Hz (`ft8dec`'s `df`: fs / block).
const TONE_HZ: f32 = 6.25;
/// GFSK transition zone, 1/128 symbol each side of a symbol boundary (`ft8dec.c` 217: `1 - gstep[24]` = 0.002).
const GZ: usize = 24;
/// Entries of the reference oscillator's sine table (`ft8dec.c` 108: linear interpolation, error < 2e-5).
const STAB: usize = 512;

/// GFSK (BT = 2) frequency step `S(d) = (1 + erf(pi sqrt(2 / ln 2) 2 d)) / 2`, `d = j / 128` symbol from the boundary (`ft8dec.c` 89-93).
// the table is `ft8dec.c`'s, digit for digit (7 digits: more than an `f32` keeps)
#[allow(clippy::excessive_precision)]
const GSTEP: [f32; 33] = [
    0.5000000, 0.5469344, 0.5932215, 0.6382405, 0.6814223, 0.7222704, 0.7603782, 0.7954389,
    0.8272514, 0.8557185, 0.8808406, 0.9027049, 0.9214715, 0.9373570, 0.9506183, 0.9615363,
    0.9704009, 0.9774992, 0.9831046, 0.9874701, 0.9908230, 0.9933627, 0.9952600, 0.9966577,
    0.9976732, 0.9984008, 0.9989150, 0.9992733, 0.9995196, 0.9996865, 0.9997981, 0.9998717,
    0.9999195,
];

/// A decoded signal to subtract (`ft8dec.c`'s `sig_t`, 95-100): the frequency of tone 0, the sample at which symbol 0 starts (may be before the
/// slot) and the 79 tones.
#[derive(Clone, Debug)]
pub(super) struct SubSig {
    pub(super) f0: f32,
    pub(super) t0: i64,
    pub(super) tones: [u8; NN],
}

/// Per signal while streaming (`sst_t`, 102-107).
#[derive(Clone, Copy, Default)]
struct SigState {
    /// Reference phase at the correlation / subtraction sample (2^32 = one turn).
    phc: u32,
    phs: u32,
    /// Symbol correlations, ring by symbol index & 3.
    cr: [f32; 4],
    ci: [f32; 4],
    /// `sum c[k] conj(c[k - 1])`: the frequency correction.
    pr: f32,
    pi: f32,
    started_c: bool,
    started_s: bool,
}

/// What the signals share: the sine table and the sample-in-symbol to 1/128-symbol map.
struct Tabs {
    sn: [f32; STAB + STAB / 4 + 1],
    gidx: Vec<u8>,
    block: usize,
    fscale: f32,
}

impl Tabs {
    fn new(fs: usize, block: usize) -> Self {
        let mut sn = [0.0f32; STAB + STAB / 4 + 1];
        for (i, v) in sn.iter_mut().enumerate() {
            *v = (core::f64::consts::TAU * i as f64 / STAB as f64).sin() as f32;
        }
        Self {
            sn,
            gidx: (0..block).map(|i| (i * 128 / block) as u8).collect(),
            block,
            fscale: 4294967296.0 / fs as f32,
        }
    }

    /// cos / sin of a 32-bit phase: the table with linear interpolation (`psc`, 149-157).
    #[inline]
    fn psc(&self, ph: u32) -> (f32, f32) {
        let j = (ph >> 23) as usize;
        let fr = (ph & 0x7F_FFFF) as f32 * (1.0 / 8388608.0);
        let t = &self.sn[j..];
        let si = t[0] + (t[1] - t[0]) * fr;
        let co = t[STAB / 4] + (t[STAB / 4 + 1] - t[STAB / 4]) * fr;
        (co, si)
    }

    /// Phase increment per sample (2^32 / fs units) of symbol `k` at 1/128-symbol position `d` (`ph_inc`, 218-224).
    #[inline]
    fn ph_inc(&self, s: &SubSig, k: usize, d: usize) -> u32 {
        let mut t = s.tones[k] as f32;
        if d < GZ {
            let p = if k > 0 { s.tones[k - 1] } else { s.tones[0] } as f32;
            t += (p - t) * (1.0 - GSTEP[d]);
        } else if d >= 128 - GZ {
            let nx = if k < NN - 1 {
                s.tones[k + 1]
            } else {
                s.tones[NN - 1]
            } as f32;
            t += (nx - t) * (1.0 - GSTEP[128 - d]);
        }
        ((s.f0 + TONE_HZ * t) * self.fscale) as i32 as u32
    }

    /// The reference phase at sample `u` of the signal (phase 0 at `u = 0`; `ph_at`, 226-236).
    fn ph_at(&self, s: &SubSig, u: i64) -> u32 {
        let mut ph = 0u32;
        let end = u.min((NN * self.block) as i64);
        for v in 0..end.max(0) as usize {
            let ks = v / self.block;
            let d = self.gidx[v - ks * self.block] as usize;
            ph = ph.wrapping_add(self.ph_inc(s, ks, d));
        }
        ph
    }

    /// Correlate the **input** chunk `x` (samples `a .. a + x.len()` of the stream) with the signal's reference (`sig_corr`, 243-279).
    fn sig_corr(&self, s: &SubSig, t: &mut SigState, a: i64, x: &[f32]) {
        let block = self.block;
        let n = x.len();
        let mut u = a - s.t0;
        let mut j = 0usize;
        if u < 0 {
            j = (-u) as usize;
            u = 0;
        }
        if u >= (NN * block) as i64 || j >= n {
            return;
        }
        let u = u as usize;
        let (mut ks, mut wi) = (u / block, u % block);
        if !t.started_c {
            t.started_c = true;
            t.phc = self.ph_at(s, u as i64);
        }
        let mut flat = self.ph_inc(s, ks, 64);
        let mut ph = t.phc;
        let (mut co, mut si) = (0.0f32, 0.0f32);
        let (mut fc, mut fsn) = self.psc(flat);
        let mut rot = false; // in the flat part: rotate instead of table lookups
        while j < n {
            if wi == 0 {
                let q = ks & 3;
                t.cr[q] = 0.0;
                t.ci[q] = 0.0;
                flat = self.ph_inc(s, ks, 64);
                (fc, fsn) = self.psc(flat);
            }
            let d = self.gidx[wi] as usize;
            let tz = !(GZ..128 - GZ).contains(&d);
            if tz || !rot {
                (co, si) = self.psc(ph);
                rot = !tz;
            } else {
                let t2 = co * fc - si * fsn;
                si = si * fc + co * fsn;
                co = t2;
            }
            t.cr[ks & 3] += x[j] * co;
            t.ci[ks & 3] -= x[j] * si;
            ph = ph.wrapping_add(if tz { self.ph_inc(s, ks, d) } else { flat });
            wi += 1;
            if wi == block {
                if ks > 0 {
                    // symbol `ks` complete: its phase step against `ks - 1`
                    let (qa, qb) = (ks & 3, (ks - 1) & 3);
                    t.pr += t.cr[qa] * t.cr[qb] + t.ci[qa] * t.ci[qb];
                    t.pi += t.ci[qa] * t.cr[qb] - t.cr[qa] * t.ci[qb];
                }
                wi = 0;
                ks += 1;
                if ks == NN {
                    break;
                }
            }
            j += 1;
        }
        t.phc = ph;
    }

    /// Subtract the signal from the **output** chunk (samples `a .. a + n` of the stream, in the ring at `idx & mask`), with the complex
    /// amplitude linearly interpolated between symbol centres (`sig_sub`, 280-319).
    fn sig_sub(
        &self,
        s: &SubSig,
        t: &mut SigState,
        a: i64,
        ring: &mut [f32],
        mask: usize,
        n: usize,
    ) {
        let block = self.block;
        let half = block / 2;
        let ab = 2.0 / block as f32;
        let inv = 1.0 / block as f32;
        let mut u = a - s.t0;
        let mut j = 0usize;
        if u < 0 {
            j = (-u) as usize;
            u = 0;
        }
        if u >= (NN * block) as i64 || j >= n {
            return;
        }
        let u = u as usize;
        let (mut ks, mut wi) = (u / block, u % block);
        let mut ri = (a as usize).wrapping_add(j) & mask;
        if !t.started_s {
            t.started_s = true;
            t.phs = self.ph_at(s, u as i64);
        }
        let mut flat = self.ph_inc(s, ks, 64);
        let mut ph = t.phs;
        let (mut co, mut si) = (0.0f32, 0.0f32);
        let (mut fc, mut fsn) = self.psc(flat);
        let mut rot = false;
        let (mut ar, mut ai, mut dar, mut dai) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        let mut seg = true; // (re)start the amplitude ramp
        while j < n {
            if wi == 0 {
                flat = self.ph_inc(s, ks, 64);
                (fc, fsn) = self.psc(flat);
            }
            if seg || wi == 0 || wi == half {
                // amplitude: linear between symbol centres
                let (k0, k1, w);
                if wi < half {
                    k0 = if ks > 0 { ks - 1 } else { 0 };
                    k1 = ks;
                    w = if ks > 0 {
                        (wi + half) as f32 * inv
                    } else {
                        1.0
                    };
                } else {
                    k0 = ks;
                    k1 = if ks < NN - 1 { ks + 1 } else { ks };
                    w = if ks < NN - 1 {
                        (wi - half) as f32 * inv
                    } else {
                        0.0
                    };
                }
                let dr = t.cr[k1 & 3] - t.cr[k0 & 3];
                let di = t.ci[k1 & 3] - t.ci[k0 & 3];
                ar = ab * (t.cr[k0 & 3] + dr * w);
                ai = ab * (t.ci[k0 & 3] + di * w);
                let edge = (wi < half && ks == 0) || (wi >= half && ks == NN - 1);
                dar = if edge { 0.0 } else { ab * dr * inv };
                dai = if edge { 0.0 } else { ab * di * inv };
                seg = false;
            }
            let d = self.gidx[wi] as usize;
            let tz = !(GZ..128 - GZ).contains(&d);
            if tz || !rot {
                (co, si) = self.psc(ph);
                rot = !tz;
            } else {
                let t2 = co * fc - si * fsn;
                si = si * fc + co * fsn;
                co = t2;
            }
            ring[ri] -= ar * co - ai * si;
            ar += dar;
            ai += dai;
            ri = (ri + 1) & mask;
            ph = ph.wrapping_add(if tz { self.ph_inc(s, ks, d) } else { flat });
            wi += 1;
            if wi == block {
                wi = 0;
                ks += 1;
                if ks == NN {
                    break;
                }
            }
            j += 1;
        }
        t.phs = ph;
    }
}

/// The audio minus the signals, read forward (`build_wf`'s streaming loop, 321-379). The input is pulled from `inner` in chunks of half a
/// symbol and correlated with every signal; the output chunk 1.5 symbols behind has each signal's reference subtracted and is final.
/// `read` serves only final samples; a read behind the ring (8192 samples) is an error.
pub(super) struct ResidualSource<'a, A: PcmSource> {
    inner: &'a mut A,
    sigs: Vec<SubSig>,
    st: Vec<SigState>,
    tabs: Tabs,
    sub: usize,
    lag: usize,
    fs: usize,
    ring: Vec<f32>,
    mask: usize,
    /// Next input sample (all before it are in the ring and correlated) and next output sample (all before it are final).
    n: i64,
    m: i64,
    rd: Vec<f32>,
    raw: Vec<i16>,
}

/// Ring samples: a power of two above a demux read (3 072 samples: three blocks of the forward window) plus the lag, a half symbol of
/// chunk and the demux's reach back.
const RING: usize = 8192;

impl<'a, A: PcmSource> ResidualSource<'a, A> {
    pub(super) fn new(inner: &'a mut A, rate: Ft8Rate, sigs: &[SubSig]) -> Self {
        let (fs, block) = (rate.hz() as usize, rate.nsps());
        let sub = block / 2;
        Self {
            inner,
            sigs: sigs.to_vec(),
            st: vec![SigState::default(); sigs.len()],
            tabs: Tabs::new(fs, block),
            sub,
            lag: 3 * sub,
            fs,
            ring: vec![0.0; RING],
            mask: RING - 1,
            n: 0,
            m: 0,
            rd: vec![0.0; sub],
            raw: vec![0i16; sub],
        }
    }

    /// One step of `build_wf`: the next input chunk, read, put in the ring and correlated; and, once the lag has been primed, the output chunk
    /// that many samples behind, subtracted.
    fn step(&mut self) {
        let sub = self.sub;
        self.inner.read(self.n, &mut self.raw);
        for (j, (&r, d)) in self.raw.iter().zip(self.rd.iter_mut()).enumerate() {
            *d = r as f32;
            self.ring[(self.n as usize + j) & self.mask] = r as f32;
        }
        for (s, t) in self.sigs.iter().zip(self.st.iter_mut()) {
            self.tabs.sig_corr(s, t, self.n, &self.rd);
        }
        self.n += sub as i64;
        if self.n - self.m >= (self.lag + sub) as i64 {
            for (s, t) in self.sigs.iter().zip(self.st.iter_mut()) {
                self.tabs
                    .sig_sub(s, t, self.m, &mut self.ring, self.mask, sub);
            }
            self.m += sub as i64;
        }
    }

    /// The signals with the frequency corrected from the symbol phase steps (`build_wf` 373-377): `f0 += (pi / pr) fs / (2 pi block)` when the steps
    /// agree (`pr > 0`, `|pi| < pr`). Call after the whole slot has been read.
    pub(super) fn into_signals(self) -> Vec<SubSig> {
        let block = self.tabs.block as f32;
        self.sigs
            .into_iter()
            .zip(self.st)
            .map(|(mut s, t)| {
                if t.pr > 0.0 && t.pi.abs() < t.pr {
                    s.f0 += (t.pi / t.pr) * self.fs as f32 / (core::f32::consts::TAU * block);
                }
                s
            })
            .collect()
    }
}

impl<A: PcmSource> PcmSource for ResidualSource<'_, A> {
    fn len(&self) -> usize {
        self.inner.len()
    }

    fn read(&mut self, start: i64, out: &mut [i16]) {
        let end = start + out.len() as i64;
        while self.m < end {
            self.step();
        }
        let oldest = self.n - RING as i64;
        let len = self.inner.len() as i64;
        for (i, o) in out.iter_mut().enumerate() {
            let idx = start + i as i64;
            *o = if idx < 0 || idx >= len {
                0
            } else {
                assert!(
                    idx >= oldest,
                    "read at {idx} behind the ring (from {oldest})"
                );
                self.ring[idx as usize & self.mask]
                    .round()
                    .clamp(i16::MIN as f32, i16::MAX as f32) as i16
            };
        }
    }
}

/// A pass after the first decodes at most `max_cand / PASS_CAND_DIV` candidates (at least `PASS_CAND_MIN`). **A deliberate divergence from `ft8dec`**, which decodes
/// `max_cand` (300) candidates every pass: next to a subtracted signal the candidates are mostly what is left of it, and each that fails costs an OSD (about
/// 10 M instructions). Measured (60 slots, 30 candidates, 2 passes): all 30 candidates of the pass 1 197 hits at 1 195 M instructions a slot, 15 of them
/// 1 192 at 876 M, 12 1 185 at 824 M, 10 1 175 at 791 M.
const PASS_CAND_DIV: usize = 2;
const PASS_CAND_MIN: usize = 8;
/// `ft8dec`'s early rejection (`nsync_min` = 7, `decode_cand`): a candidate of a pass after the first needs this many of the 21 Costas symbols read right in the
/// cells before its LLRs are made. The search's candidates all have at least 7 (it is the order the search sorts by), so with the cap above it rejects nothing and
/// costs nothing here (measured: a minimum of 9 or 11 with the cap changed the cost by 5 M and the hits by 0).
const PASS_MIN_HS: u8 = 7;

/// Carriers (cells of 3.125 Hz) either side of a newly subtracted signal that the next pass searches: `ft8dec`'s +-9 bins of 6.25 Hz (`ft8dec.c` 833-841).
const NEAR_CELLS: usize = 18;

/// The FT8 flow of the S3 at 8 kHz from the half-dB cells **with `ft8dec`'s subtraction passes**: pass 0 is [`decode_block_cells`](super::decode_block_cells)'s
/// (the search of `search`, the first `max_cand` candidates at their grid place, the retry at the estimated place); then, up to `passes` in all, each pass
/// subtracts every signal decoded so far from the audio **as it is read** ([`ResidualSource`]: no copy of the slot, a ring of 8 192 samples), makes the
/// cells of the residual, and searches and decodes **only the carriers within +-18 cells of the signals the previous pass added** (`ft8dec.c` 984-1010,
/// `round_` 830-911), at most `max_cand / 2` candidates (8 at least; a divergence from `ft8dec`) with `ft8dec`'s hard-sync minimum of 7 (`PASS_CAND_DIV`, `PASS_MIN_HS`); it stops when a pass adds nothing. `None` unless `rate.coarse_ft8lib()` (`Hz8000`, `fixed-point`). `passes` 1 is pass 0 alone.
///
/// Every decode reports the place of [`View::refine`] (`ft8dec.c` 874-882: the parabola on the energy of its 79 tones, the time through `tcal`), finer than
/// the grid's. **A deliberate divergence from WSJT-X** (the subtraction, the passes) and from `ft8dec` (the staircase after the spectra is the library's, as
/// in [`decode_block_cells`](super::decode_block_cells)). `snr_db` is not validated. Peak memory: the cells of one pass (172 KB at 200-3000 Hz), the ring
/// and the signals' states (about 40 KB), the spectra of one candidate.
pub fn decode_block_cells_sic<S: AudioSample>(
    audio: &[S],
    rate: Ft8Rate,
    search: &GatedSearch,
    max_cand: usize,
    depth: DecodeDepth,
    passes: usize,
) -> Option<Vec<DecodeResult>> {
    let (cands, cells) = coarse_sync_gated_cells(&mut SliceSource(audio), rate, search, 1)?;
    let df = SAMPLE_RATE_HZ / super::types::NFFT_SPEC as f32;
    let (nstep, fs) = (rate.nstep() as i64, rate.hz() as f32);
    let tables = CellTables::new();
    let mut known: Vec<Found> = Vec::new();
    let mut sigs: Vec<SubSig> = Vec::new();

    // pass 0
    decode_round(
        &cells,
        &tables,
        &cands,
        max_cand,
        depth,
        RETRY_MIN_SHIFT,
        &mut known,
    );
    add_signals(
        &mut known,
        0,
        &View {
            cells: &cells,
            t: &tables,
        },
        &mut sigs,
        df,
        nstep,
        fs,
    );
    drop(cells);

    // the slot's gain, from the audio (read only if a later pass needs it): the residual's peak is not above it, and a residual is not read for it
    let gain = (passes > 1).then(|| spectrogram_gain(&mut SliceSource(audio)));
    let mut new_from = 0usize;
    for _ in 1..passes {
        if sigs.len() == new_from {
            break; // the last pass added nothing
        }
        let near: Vec<f32> = sigs[new_from..].iter().map(|s| s.f0).collect();
        let before = sigs.len();
        let mut inner = SliceSource(audio);
        let mut residual = ResidualSource::new(&mut inner, rate, &sigs);
        let (cands, cells) = search_near(
            &mut residual,
            rate,
            search,
            gain.expect("passes > 1"),
            &near,
            NEAR_CELLS,
        );
        sigs = residual.into_signals();
        // `ft8dec.c` 857-862: a candidate farther than 9 bins (56.25 Hz) from every new signal is skipped, with the frequencies as corrected by the pass: the
        // mask of `search_near` is made before the correction (under 0.78 Hz) and keeps the +-18 cells inclusive, which can reach 57.8 Hz
        let cands: Vec<_> = cands
            .into_iter()
            .filter(|(c, _)| {
                sigs[new_from..before]
                    .iter()
                    .any(|s| (c.freq_hz - s.f0).abs() < 9.0 * TONE_HZ)
            })
            .collect();
        let view = View {
            cells: &cells,
            t: &tables,
        };
        let first_new = known.len();
        let cands: Vec<_> = cands.into_iter().filter(|c| c.1 >= PASS_MIN_HS).collect();
        decode_round(
            &cells,
            &tables,
            &cands,
            (max_cand / PASS_CAND_DIV).max(PASS_CAND_MIN).min(max_cand),
            depth,
            RETRY_MIN_SHIFT,
            &mut known,
        );
        add_signals(&mut known, first_new, &view, &mut sigs, df, nstep, fs);
        new_from = before;
    }
    Some(known.into_iter().map(|f| f.res).collect())
}

/// For each decode from `from` on: its place by [`View::refine`] (the decode reports it) and a [`SubSig`] for the next pass's subtraction.
fn add_signals(
    known: &mut [Found],
    from: usize,
    view: &View<'_>,
    sigs: &mut Vec<SubSig>,
    df: f32,
    nstep: i64,
    fs: f32,
) {
    for f in &mut known[from..] {
        let mut tones = [0u8; NN];
        tones.copy_from_slice(&message_to_tones::<crate::ft8::Ft8>(f.res.message77()));
        let (f0, t0) = view.refine(f.grid, &tones, df, nstep);
        f.res.freq_hz = f0;
        f.res.dt_sec = t0 as f32 / fs - 0.5 + crate::ft8::refine_fine::XDT_OFFSET_S;
        sigs.push(SubSig { f0, t0, tones });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::engine::tx::{message_to_tones, synthesize_i16};
    use crate::ft8::Ft8;
    use crate::msg::wsjt77::pack77;

    const FS: usize = 8000;

    /// A 15 s slot of 8 kHz with the signals `(call1, call2, report, tone-0 Hz, DT s, amplitude)`, and the `SubSig`s that describe them.
    fn slot(signals: &[(&str, &str, &str, f32, f32, i16)]) -> (Vec<i16>, Vec<SubSig>) {
        let mut v = vec![0f32; 15 * FS];
        let mut sigs = Vec::new();
        for &(a, b, c, f, dt, amp) in signals {
            let tones = message_to_tones::<Ft8>(&pack77(a, b, c).unwrap());
            let pcm = synthesize_i16::<Ft8>(&tones, FS as u32, f, amp);
            let at = ((0.5 + dt) * FS as f32).round() as i64;
            for (i, &s) in pcm.iter().enumerate() {
                let idx = at + i as i64;
                if idx >= 0 && (idx as usize) < v.len() {
                    v[idx as usize] += s as f32;
                }
            }
            let mut t = [0u8; NN];
            t.copy_from_slice(&tones);
            sigs.push(SubSig {
                f0: f,
                t0: at,
                tones: t,
            });
        }
        (v.iter().map(|&x| x.round() as i16).collect(), sigs)
    }

    fn residual(audio: &[i16], sigs: &[SubSig], chunk: usize) -> Vec<i16> {
        let mut src = SliceSource(audio);
        let mut r = ResidualSource::new(&mut src, Ft8Rate::Hz8000, sigs);
        let mut out = vec![0i16; audio.len()];
        for (i, c) in out.chunks_mut(chunk).enumerate() {
            r.read((i * chunk) as i64, c);
        }
        out
    }

    fn energy(x: &[i16]) -> f64 {
        x.iter().map(|&v| (v as f64) * (v as f64)).sum()
    }

    /// dB of the residual's energy against the original's, over the span of the signal that starts at sample `t0`.
    fn left_db(orig: &[i16], res: &[i16], t0: i64) -> f64 {
        let (a, b) = (t0 as usize, (t0 as usize + NN * 1280).min(orig.len()));
        10.0 * (energy(&res[a..b]) / energy(&orig[a..b])).log10()
    }

    /// The subtraction of one clean signal at its true place leaves less than -25 dB of it, and with the place 1.5 Hz and 40 ms off it still removes
    /// more than half the energy: the **reference** (a signal synthesised by the library's own GFSK, whose tones and place are known) is what is
    /// checked against, not the code's own correlation. **Fails if** the reference phase (`ph_inc`, the sine table), the GFSK transition (`GSTEP`),
    /// the interpolation of the amplitude between symbol centres or the lag is wrong (each mutation-checked).
    #[test]
    fn a_clean_signal_is_removed_and_a_misplaced_one_mostly() {
        let (a, sigs) = slot(&[("CQ", "K1ABC", "FN42", 1500.0, 0.3, 8000)]);
        let r = residual(&a, &sigs, 3072);
        let on = left_db(&a, &r, sigs[0].t0);
        assert!(on < -25.0, "{on:.1} dB left at the true place");
        let mut off = sigs.clone();
        off[0].f0 += 1.5;
        off[0].t0 += 320;
        let r = residual(&a, &off, 3072);
        let l = left_db(&a, &r, sigs[0].t0);
        assert!(l < -3.0, "{l:.1} dB left 1.5 Hz and 40 ms off");
    }

    /// Two signals 60 Hz apart (their tones do not share frequencies), overlapping in time: both are removed. **Fails if** a signal's subtraction reads another's correlation (the state
    /// is per signal) or the ring mixes the two.
    #[test]
    fn two_overlapping_signals_are_both_removed() {
        let (a, sigs) = slot(&[
            ("CQ", "K1ABC", "FN42", 1500.0, 0.3, 8000),
            ("K1ABC", "W9XYZ", "-12", 1560.0, 0.1, 5000),
        ]);
        let r = residual(&a, &sigs, 3072);
        for s in &sigs {
            let l = left_db(&a, &r, s.t0);
            assert!(l < -20.0, "{l:.1} dB left of the signal at {} Hz", s.f0);
        }
        // and with no signals the audio is served as it is
        let same = residual(&a, &[], 1000);
        assert_eq!(same, a);
    }

    /// The residual does not depend on how it is read: chunks of 100, 777 and 3 072 samples give the same samples. **Fails if** a sample is served
    /// before the lag has finished subtracting it (a small read would then see the unsubtracted audio).
    #[test]
    fn the_residual_does_not_depend_on_the_read_size() {
        let (a, sigs) = slot(&[("CQ", "K1ABC", "FN42", 1500.0, 0.3, 8000)]);
        let big = residual(&a, &sigs, 3072);
        assert_eq!(big, residual(&a, &sigs, 777));
        assert_eq!(big, residual(&a, &sigs, 100));
        assert_ne!(big, a, "something was subtracted");
    }

    /// The frequency correction from the symbol phase steps pulls a signal 0.6 Hz off (a phase step of 35 degrees a symbol) to within 0.2 Hz of the
    /// truth, either way. It applies only below 45 degrees (`|pi| < pr`, about 0.78 Hz): `ft8dec`'s own limit. **Fails if** the sign or the scale of
    /// `f0 += (pi / pr) fs / (2 pi block)` is wrong.
    #[test]
    fn the_phase_steps_correct_the_frequency() {
        let (a, sigs) = slot(&[("CQ", "K1ABC", "FN42", 1500.0, 0.3, 8000)]);
        for off in [0.6f32, -0.6] {
            let mut s = sigs.clone();
            s[0].f0 += off;
            let mut src = SliceSource(&a);
            let mut r = ResidualSource::new(&mut src, Ft8Rate::Hz8000, &s);
            let mut out = vec![0i16; a.len()];
            for (i, c) in out.chunks_mut(3072).enumerate() {
                r.read((i * 3072) as i64, c);
            }
            let fixed = r.into_signals();
            assert!(
                (fixed[0].f0 - 1500.0).abs() < 0.2,
                "{} Hz after correcting {} Hz",
                fixed[0].f0,
                1500.0 + off
            );
        }
    }

    fn noisy(mut a: Vec<i16>, sd: f32, seed: u64) -> Vec<i16> {
        let mut st = seed;
        for x in a.iter_mut() {
            let mut g = 0.0f32;
            for _ in 0..12 {
                st = st
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                g += ((st >> 40) as f32) / (1u64 << 24) as f32;
            }
            *x = (*x as f32 + (g - 6.0) * sd)
                .round()
                .clamp(-32768.0, 32767.0) as i16;
        }
        a
    }

    fn depth() -> DecodeDepth {
        DecodeDepth {
            osd: true,
            ..DecodeDepth::EMBEDDED
        }
    }

    fn search() -> GatedSearch {
        GatedSearch::new(200.0, 3000.0, 1.3, 30)
    }

    fn texts(out: &[DecodeResult]) -> Vec<alloc::string::String> {
        let mut t: Vec<_> = out
            .iter()
            .filter_map(|d| crate::msg::wsjt77::unpack77(d.message77()))
            .collect();
        t.sort();
        t
    }

    /// A signal the strong one next to it hides is decoded by the second pass and not by the first: a strong station (amplitude 12 000) and a weak one
    /// (1 200, 20 dB down) 12.5 Hz above it (two tone spacings), noise sd 1 500. Pass 1 finds the strong one only; two passes find both. **Fails if** the
    /// subtraction leaves the strong signal's energy where the weak one is (the residual's search finds nothing new), removes the weak one with it, or the
    /// next pass does not search near what was subtracted (`NEAR_CELLS`).
    #[test]
    fn the_second_pass_decodes_a_signal_the_strong_one_hid() {
        let (a, _) = slot(&[
            ("CQ", "K1ABC", "FN42", 1500.0, 0.2, 12000),
            ("K1ABC", "W9XYZ", "-12", 1512.5, 0.2, 1200),
        ]);
        let a = noisy(a, 1500.0, 7);
        let one = decode_block_cells_sic(&a, Ft8Rate::Hz8000, &search(), 30, depth(), 1).unwrap();
        assert_eq!(texts(&one), ["CQ K1ABC FN42"], "pass 1 alone");
        let two = decode_block_cells_sic(&a, Ft8Rate::Hz8000, &search(), 30, depth(), 2).unwrap();
        assert_eq!(
            texts(&two),
            ["CQ K1ABC FN42", "K1ABC W9XYZ -12"],
            "two passes"
        );
    }

    /// One pass is `decode_block_cells`'s decodes (the same messages; the places differ: this flow reports the refined place), and `None` at 12 kHz.
    /// **Fails if** pass 0 changes what is decoded or the rate gate is dropped.
    #[test]
    fn one_pass_is_the_cells_flow() {
        let signals = [
            ("CQ", "K1ABC", "FN42", 700.0f32, 0.2f32, 6000i16),
            ("K1ABC", "W9XYZ", "-12", 1500.3, 0.9, 5000),
            ("CQ", "JA1ABC", "PM95", 2300.7, -0.6, 5000),
        ];
        let a = noisy(slot(&signals).0, 1500.0, 11);
        let sic = decode_block_cells_sic(&a, Ft8Rate::Hz8000, &search(), 15, depth(), 1).unwrap();
        let cells =
            super::super::decode_block_cells(&a, Ft8Rate::Hz8000, &search(), 15, depth()).unwrap();
        assert_eq!(texts(&sic), texts(&cells));
        assert_eq!(texts(&sic).len(), 3);
        assert!(decode_block_cells_sic(&a, Ft8Rate::Hz12000, &search(), 15, depth(), 2).is_none());
    }

    /// A decode reports the refined place: a clean signal at 1501.2 Hz, DT 0.23 s (0.38 cell and 0.38 row off the grid) is reported within 0.5 Hz and
    /// 15 ms of the truth (the library's DT carries its -5 ms offset), where the grid is 1.2 Hz and 30 ms off. **Fails if** the parabola, `tcal` or
    /// the conversion to Hz and samples is wrong.
    #[test]
    fn a_decode_reports_the_refined_place() {
        let (a, _) = slot(&[("CQ", "K1ABC", "FN42", 1501.2, 0.23, 8000)]);
        let a = noisy(a, 1500.0, 3);
        let out = decode_block_cells_sic(&a, Ft8Rate::Hz8000, &search(), 15, depth(), 1).unwrap();
        assert_eq!(out.len(), 1);
        let d = &out[0];
        assert!((d.freq_hz - 1501.2).abs() <= 0.5, "{} Hz", d.freq_hz);
        assert!(
            (d.dt_sec - (0.23 + crate::ft8::refine_fine::XDT_OFFSET_S)).abs() <= 0.015,
            "{} s",
            d.dt_sec
        );
    }

    /// A signal whose amplitude changes along it (a slow ramp from 1 to 5 and a 0.78 Hz swell of 80 %, a start that is not a multiple of 640 samples) is
    /// removed to under -24 dB (measured -27.8). **Fails if** the interpolation reads the wrong neighbour symbol (`k1`: -20.2 dB, caught). It does **not**
    /// pin the slope of the ramp (`dar`, `dai` set to 0 costs 1.1 dB) or the recompute at `wi == half` (no change: the interpolation is not what limits
    /// the depth here, the GFSK reference and the table are): found by mutation, stated.
    #[test]
    fn a_fading_signal_is_removed_and_the_next_symbol_is_the_one_interpolated_to() {
        let tones = message_to_tones::<Ft8>(&pack77("CQ", "K1ABC", "FN42").unwrap());
        let pcm = synthesize_i16::<Ft8>(&tones, FS as u32, 1500.0, 2500);
        // a start that is not a multiple of a half symbol (640 samples): the amplitude is recomputed mid-chunk at `wi == half`, not only at a chunk's start
        let at = ((0.5 + 0.3375) * FS as f32).round() as i64;
        let mut v = vec![0i16; 15 * FS];
        let n = pcm.len() as f32;
        for (i, &x) in pcm.iter().enumerate() {
            let g = 1.0
                + 0.8 * (core::f32::consts::TAU * 0.78 * i as f32 / FS as f32).sin()
                + 4.0 * i as f32 / n;
            v[at as usize + i] = (x as f32 * g).round().clamp(-32768.0, 32767.0) as i16;
        }
        let mut t = [0u8; NN];
        t.copy_from_slice(&tones);
        let sigs = [SubSig {
            f0: 1500.0,
            t0: at,
            tones: t,
        }];
        let r = residual(&v, &sigs, 3072);
        let l = left_db(&v, &r, at);
        assert!(l < -24.0, "{l:.1} dB left of a fading signal");
    }
}
