//! The JTTY frame decoder: one window of audio → the frames in it.
//!
//! Ported from WSJT-X `lib/jtty/jtty_mdecode.f90` (`jtty_mdecode`,
//! `process_channel`, `decode_and_merge`), `jtty_peakup.f90` and the single-signal
//! `jtty_decode.f90`, tag `v3.2.0-rc1`. **Phase P2: no signal subtraction, no
//! retro re-sweep, no message assembly** — those are P3. A [`FrameDecode`] is one
//! validated frame, with where and when it was found.
//!
//! ## One window
//!
//! A window is 1.25 frames (28 320 samples at 12 kHz); a receiver starts one every
//! quarter frame ([`STEP`]), and each window looks for frames *starting* in its own
//! first quarter frame. For a window:
//!
//! 1. **Analytic signal** at 6 kHz ([`dsp::analytic_6k`]).
//! 2. **Sync surface** — the correlation of the 13-symbol sync waveform with the
//!    signal at 237 start offsets (2 ms apart) over the first quarter frame, one
//!    8192-point FFT each (0.73 Hz bins), smoothed across frequency by
//!    `1-2-3-2-1`. The offsets are independent, so they run on rayon's pool.
//! 3. **Candidates** in three frequency channels — the operator's frequency
//!    ±`ftol` (channel 0), and 1350 / 1650 Hz ±150 Hz (channels 1, 2): the
//!    strongest surface peaks, each masking out its neighbourhood before the next
//!    is taken. Channel 0's are refined by `peakup` (a search over ±4 ms and ±2.5 Hz); channels 1 and 2 are
//!    searched only after channel 0's successes are erased from the surface.
//! 4. **Per candidate**: shift it to 0 Hz, count the sync tones it gets right and
//!    estimate the signal-to-noise ratio from them (a **gate**), correlate the 46
//!    data symbols, run the decode ladder, and keep the payload only if it is a
//!    valid source word ([`source::decode_payload`]). Without subtraction the
//!    candidates are independent, so they too run on the pool, and the results are
//!    put back in order before duplicates are dropped.
//!
//! Every parallel step collects in order, so the output does not depend on the
//! thread count.

use alloc::string::String;
use alloc::vec::Vec;
use core::f32::consts::PI;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::correlate::{ToneRefs, correlate_payload};
use super::dsp::{self, FS6, NSS, db};
use super::ladder::Ladder;
use super::source::{self, Atom};
use super::{FRAME_SYMBOLS, INFO_BITS, NSPS, Payload, SYNC, SYNC_SYMBOLS, crc, tbcc};
use crate::engine::fft::default_planner;

/// Samples of one window at 12 kHz: 1.25 frames.
pub const NCHUNK: usize = FRAME_SYMBOLS * NSPS + FRAME_SYMBOLS * NSPS / 4;
/// Samples between the starts of successive windows at 12 kHz: a quarter frame.
pub const STEP: usize = FRAME_SYMBOLS * NSPS / 4;

/// FFT length of the sync search.
const NFFT: usize = 8192;
/// Bin width of the sync surface, Hz.
const DF: f32 = FS6 / NFFT as f32;
/// Samples between sync-surface columns at 6 kHz (2 ms).
const COL_STEP: usize = 12;
/// Sync-surface columns: start offsets over one quarter frame.
const NCOLS: usize = FRAME_SYMBOLS * NSS / 4 / COL_STEP + 1;
/// Half-width of the peak-suppression rectangle: 10 Hz in bins …
const NFZ: usize = 14;
/// … and 16 ms in columns.
const NTZ: usize = 8;
/// Candidates taken from channels 1 and 2.
const OTHER_CANDIDATES: usize = 2;
/// Sync tones that must be right on channel 0, and the S/N floor there is
/// [`Params::smin_db`]; channels 1 and 2 need more.
const CH0_MIN_SYNC: usize = 7;
const OTHER_MIN_SYNC: usize = 9;
const OTHER_MIN_SNR_DB: f32 = 5.0;
/// Two frames with the same text closer than this (seconds) are one.
const DUPE_TIME_S: f32 = 0.032;
/// Channel-1/2 candidates this close (Hz, seconds) to a channel-0 success are it.
const SAME_FRAME_HZ: f32 = 3.0;
const SAME_FRAME_S: f32 = 0.05;

/// What to look for.
#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// Operator's receive frequency, Hz (channel 0's centre).
    pub f0_hz: f32,
    /// Half-width of channel 0, Hz.
    pub ftol_hz: f32,
    /// S/N floor of the sync gate on channel 0, dB (`smin`).
    pub smin_db: f32,
    /// Lowest audio frequency channels 1 and 2 may look at, Hz (the waterfall's).
    pub nfa_hz: f32,
    /// Highest audio frequency channels 1 and 2 may look at, Hz.
    pub nfb_hz: f32,
}

impl Default for Params {
    /// `rjtty`'s defaults: 1500 Hz ± 50 Hz, `smin` 4.6, band 200–2800 Hz.
    fn default() -> Self {
        Self {
            f0_hz: 1500.0,
            ftol_hz: 50.0,
            smin_db: 4.6,
            nfa_hz: 200.0,
            nfb_hz: 2800.0,
        }
    }
}

/// One validated frame.
#[derive(Clone, Debug)]
pub struct FrameDecode {
    /// 0 for the operator's frequency, 1 and 2 for the fixed side channels.
    pub channel: u8,
    /// Frequency of the lowest tone, Hz.
    pub f1_hz: f32,
    /// Start of the frame within its window, seconds.
    pub xdt_s: f32,
    /// Start of the frame, seconds from the start of the audio.
    pub tsync_s: f32,
    /// Signal-to-noise estimate from the sync and data tones (`snrdb`); upstream
    /// displays `round(snr_db − 20)`.
    pub snr_db: f32,
    /// Sync tones received correctly, of 13.
    pub nsync: usize,
    /// Symbols (of 59) whose hard decision differs from the decoded frame.
    pub nsymerrs: usize,
    /// The frame's atom.
    pub atom: Atom,
    /// End-of-message flag.
    pub eom: bool,
    /// The 34-bit payload.
    pub payload: Payload,
    /// Which ladder rung accepted it (1‥3 full-symbol, 4 half-symbol).
    pub rung: usize,
    /// 1-based rank of the accepted word in that rung's list of four (the
    /// maximum-likelihood word is rank 1).
    pub rank: usize,
    /// Distinct closed words that rung found (`pool`).
    pub pool: usize,
}

/// The frame decoder. Holds the immutable tables; one instance serves any number
/// of threads.
pub struct Receiver {
    ladder: Ladder,
    refs: ToneRefs,
    csync: Vec<Complex32>,
}

impl Default for Receiver {
    fn default() -> Self {
        Self::new()
    }
}

/// The part of the sync surface the three channels can see, `[column][bin − lo]`.
struct Surface {
    lo: usize,
    width: usize,
    data: Vec<f32>,
}

impl Surface {
    fn at(&self, col: usize, bin: usize) -> f32 {
        self.data[col * self.width + (bin - self.lo)]
    }
    fn set(&mut self, col: usize, bin: usize, v: f32) {
        self.data[col * self.width + (bin - self.lo)] = v;
    }
}

/// A bin range `[ja, jb]` searched in one channel.
#[derive(Clone, Copy)]
struct Band {
    ja: usize,
    jb: usize,
}

/// `jtty_search_window`: the bins around `fc ± fwid`, first/last usable bins 3 and
/// `nh2 − 2`; for channels 1 and 2 also clamped to the waterfall `[nfa, nfb]`.
fn search_band(fc: f32, fwid: f32, nfab: Option<(f32, f32)>) -> Option<Band> {
    let (first, last) = (3usize, NFFT / 2 - 2);
    let mut fc = fc;
    if let Some((nfa, nfb)) = nfab {
        if nfa > nfb {
            return None;
        }
        fc = fc.min(nfb).max(nfa);
    }
    let mut ja = first.max(((fc - fwid) / DF) as usize);
    let mut jb = last.min(((fc + fwid) / DF) as usize);
    if let Some((nfa, nfb)) = nfab {
        ja = ja.max((nfa / DF).ceil() as usize);
        jb = jb.min((nfb / DF).floor() as usize);
    }
    (ja <= jb).then_some(Band { ja, jb })
}

/// A peak to try: where the sync surface said it was.
#[derive(Clone, Copy)]
struct Pick {
    channel: u8,
    f_hz: f32,
    xdt_s: f32,
}

/// What became of a candidate.
struct Outcome {
    frame: FrameDecode,
    text: String,
}

impl Receiver {
    /// Build the tables (the ladder's trellis plans, tone references, sync wave).
    pub fn new() -> Self {
        Self {
            ladder: Ladder::new(),
            refs: ToneRefs::new(NSS),
            csync: dsp::sync_wave(),
        }
    }

    /// The frames whose start lies in the first quarter frame of `audio`, which
    /// must be exactly [`NCHUNK`] samples of 12 kHz audio beginning `t0_s` seconds
    /// into the recording.
    ///
    /// # Panics
    /// If `audio.len() != NCHUNK`.
    pub fn decode_window(&self, audio: &[i16], t0_s: f32, p: &Params) -> Vec<FrameDecode> {
        assert_eq!(audio.len(), NCHUNK, "a window is exactly NCHUNK samples");
        let c0 = dsp::analytic_6k(audio);

        let ch0 = search_band(p.f0_hz, p.ftol_hz, None);
        let ch1 = search_band(1350.0, 150.0, Some((p.nfa_hz, p.nfb_hz)));
        let ch2 = search_band(1650.0, 150.0, Some((p.nfa_hz, p.nfb_hz)));
        let bands = [ch0, ch1, ch2];
        let (Some(lo), Some(hi)) = (
            bands.iter().flatten().map(|b| b.ja).min(),
            bands.iter().flatten().map(|b| b.jb).max(),
        ) else {
            return Vec::new();
        };
        let mut surface = self.sync_surface(&c0, lo, hi);

        let mut outcomes: Vec<Option<Outcome>> = Vec::new();
        // ---- channel 0: peaks, then everything about each candidate in parallel
        let mut ch0_ok: Vec<(f32, f32)> = Vec::new();
        if let Some(band) = ch0 {
            let nc = 2usize.max(8usize.min((p.ftol_hz / (NFZ as f32 * DF)).round() as usize));
            let picks = pick_masked(&surface, band, nc);
            let picks: Vec<Pick> = picks
                .into_iter()
                .map(|(bin, col)| Pick {
                    channel: 0,
                    f_hz: bin as f32 * DF,
                    xdt_s: (col * COL_STEP) as f32 / FS6,
                })
                .collect();
            let res = self.process_all(&c0, &picks, t0_s, p);
            ch0_ok.extend(
                res.iter()
                    .flatten()
                    .map(|o| (o.frame.f1_hz, o.frame.tsync_s)),
            );
            outcomes.extend(res);
            // erase what channel 0 found so channels 1 and 2 do not rediscover it
            for &(f1, _) in ch0_ok.iter().take(16) {
                let centre = (f1 / DF).round() as isize;
                let (ja, jb) = (
                    (centre - NFZ as isize).max(0) as usize,
                    (centre + NFZ as isize) as usize,
                );
                for band in [ch1, ch2].into_iter().flatten() {
                    for bin in ja.max(band.ja)..=jb.min(band.jb) {
                        (0..NCOLS).for_each(|col| surface.set(col, bin, 0.0));
                    }
                }
            }
        }
        // ---- channels 1 and 2: peaks picked in turn (the second sees the first's
        // suppression), decoded together
        let mut picks: Vec<Pick> = Vec::new();
        for (channel, band) in [(1u8, ch1), (2u8, ch2)] {
            let Some(band) = band else { continue };
            for _ in 0..OTHER_CANDIDATES {
                let (bin, col) = pick_max(&surface, band, None);
                suppress(&mut surface, band, bin, col);
                picks.push(Pick {
                    channel,
                    f_hz: bin as f32 * DF,
                    xdt_s: (col * COL_STEP) as f32 / FS6,
                });
            }
        }
        outcomes.extend(self.process_all(&c0, &picks, t0_s, p));

        // ---- drop duplicates, in candidate order
        let mut seen: Vec<(String, f32)> = Vec::new();
        let mut frames = Vec::new();
        for o in outcomes.into_iter().flatten() {
            let f = &o.frame;
            let dupe = seen
                .iter()
                .any(|(t, ts)| *t == o.text && (ts - f.tsync_s).abs() < DUPE_TIME_S)
                || (f.channel != 0
                    && ch0_ok.iter().any(|&(f1, ts)| {
                        (f1 - f.f1_hz).abs() < SAME_FRAME_HZ
                            && (ts - f.tsync_s).abs() < SAME_FRAME_S
                    }));
            seen.push((o.text.clone(), f.tsync_s));
            if !dupe {
                frames.push(o.frame);
            }
        }
        frames
    }

    /// Every candidate's outcome, in order; parallel across candidates.
    fn process_all(
        &self,
        c0: &[Complex32],
        picks: &[Pick],
        t0_s: f32,
        p: &Params,
    ) -> Vec<Option<Outcome>> {
        let one = |pick: &Pick| self.process(c0, pick, t0_s, p);
        #[cfg(feature = "parallel")]
        {
            use rayon::prelude::*;
            picks.par_iter().map(one).collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            picks.iter().map(one).collect()
        }
    }

    /// The sync surface over bins `lo..=hi`, all [`NCOLS`] columns (`build_s0`).
    fn sync_surface(&self, c0: &[Complex32], lo: usize, hi: usize) -> Surface {
        let width = hi - lo + 1;
        let column =
            |col: usize, fft: &dyn crate::engine::fft::Fft, buf: &mut Vec<Complex32>| -> Vec<f32> {
                let i0 = col * COL_STEP;
                buf.iter_mut()
                    .zip(self.csync.iter().zip(&c0[i0..]))
                    .for_each(|(b, (s, x))| *b = s.conj() * *x);
                buf[self.csync.len()..].fill(Complex32::new(0.0, 0.0));
                fft.process(buf);
                let power: Vec<f32> = buf[lo - 2..=hi + 2].iter().map(|z| z.norm_sqr()).collect();
                // 1-2-3-2-1 smoothing across frequency
                power
                    .windows(5)
                    .map(|w| w[0] + 2.0 * w[1] + 3.0 * w[2] + 2.0 * w[3] + w[4])
                    .collect()
            };
        let zero = || alloc::vec![Complex32::new(0.0, 0.0); NFFT];
        #[cfg(feature = "parallel")]
        let rows: Vec<Vec<f32>> = {
            use rayon::prelude::*;
            (0..NCOLS)
                .into_par_iter()
                .with_min_len(16)
                .map_init(
                    || (default_planner().plan_forward(NFFT), zero()),
                    |(fft, buf), col| column(col, fft.as_ref(), buf),
                )
                .collect()
        };
        #[cfg(not(feature = "parallel"))]
        let rows: Vec<Vec<f32>> = {
            let fft = default_planner().plan_forward(NFFT);
            let mut buf = zero();
            (0..NCOLS)
                .map(|col| column(col, fft.as_ref(), &mut buf))
                .collect()
        };
        Surface {
            lo,
            width,
            data: rows.into_iter().flatten().collect(),
        }
    }

    /// One candidate: refine, gate, correlate, decode, validate.
    fn process(&self, c0: &[Complex32], pick: &Pick, t0_s: f32, p: &Params) -> Option<Outcome> {
        let (xdt, f1) = if pick.channel == 0 {
            let (x, f, _) = peakup(c0, &self.csync, pick.xdt_s, pick.f_hz);
            (x, f)
        } else {
            (pick.xdt_s, pick.f_hz)
        };
        let mut c1 = alloc::vec![Complex32::new(0.0, 0.0); c0.len()];
        dsp::shift_frequency(c0, &mut c1, FS6, -f1);

        // ---- sync gate: which tone is strongest in each of the 13 sync symbols
        let start = (xdt * FS6).round() as usize;
        let (mut pt, mut pa) = (0f32, 0f32);
        let mut hits = 0usize;
        for (j, &sent) in SYNC.iter().enumerate() {
            let i0 = start + j * NSS;
            if i0 + NSS > c1.len() {
                break;
            }
            let pow: [f32; 4] = core::array::from_fn(|k| {
                self.refs
                    .conj(k)
                    .iter()
                    .zip(&c1[i0..i0 + NSS])
                    .fold(Complex32::new(0.0, 0.0), |a, (&r, &x)| a + r * x)
                    .norm_sqr()
            });
            let best = (0..4).fold(0, |b, k| if pow[k] > pow[b] { k } else { b });
            hits += usize::from(best == usize::from(sent));
            pt += pow[usize::from(sent)];
            pa += pow.iter().sum::<f32>();
        }
        let pn = (pa - pt) / 3.0;
        let snr = if pn > 0.0 { db(pt / pn) } else { -99.9 };
        let passes = if pick.channel == 0 {
            hits >= CH0_MIN_SYNC && snr >= p.smin_db
        } else {
            hits >= OTHER_MIN_SYNC && snr >= OTHER_MIN_SNR_DB
        };
        if !passes {
            return None;
        }

        // ---- decode
        let (zsym, zhalf) = correlate_payload(&self.refs, &c1, start + SYNC_SYMBOLS * NSS);
        let accepted = self.ladder.decode(&zsym, &zhalf)?;
        let (atom, eom) = source::decode_payload(&accepted.payload)?;

        // ---- S/N and symbol errors from the decoded frame
        let tones = tbcc::encode(&crc::append(&accepted.payload));
        let (mut pt, mut pa) = (pt, pa);
        let mut errs = SYNC_SYMBOLS - hits;
        for (s, &sent) in tones.iter().enumerate() {
            let pow: [f32; 4] = core::array::from_fn(|k| zsym[s][k].norm_sqr());
            let best = (0..4).fold(0, |b, k| if pow[k] > pow[b] { k } else { b });
            errs += usize::from(best != usize::from(sent));
            pt += pow[usize::from(sent)];
            pa += pow.iter().sum::<f32>();
        }
        let pn = (pa - pt) / 3.0;
        let snr_db = if pn > 0.0 { db(pt / pn) } else { snr };
        debug_assert_eq!(tones.len(), INFO_BITS);

        let text = atom.render();
        Some(Outcome {
            text,
            frame: FrameDecode {
                channel: pick.channel,
                f1_hz: f1,
                xdt_s: xdt,
                tsync_s: t0_s + xdt,
                snr_db,
                nsync: hits,
                nsymerrs: errs,
                atom,
                eom,
                payload: accepted.payload,
                rung: accepted.rung,
                rank: accepted.rank,
                pool: accepted.pool,
            },
        })
    }

    /// Every frame in a recording: a window every [`STEP`] samples, duplicates
    /// (the same frame seen by two windows) dropped. Windows are independent, so
    /// they run in parallel; the result is ordered by window.
    pub fn scan(&self, audio: &[i16], p: &Params) -> Vec<FrameDecode> {
        if audio.len() < NCHUNK {
            return Vec::new();
        }
        let n = (audio.len() - NCHUNK) / STEP + 1;
        let window = |w: usize| {
            let s = w * STEP;
            self.decode_window(&audio[s..s + NCHUNK], s as f32 / 12_000.0, p)
        };
        #[cfg(feature = "parallel")]
        let per_window: Vec<Vec<FrameDecode>> = {
            use rayon::prelude::*;
            (0..n).into_par_iter().map(window).collect()
        };
        #[cfg(not(feature = "parallel"))]
        let per_window: Vec<Vec<FrameDecode>> = (0..n).map(window).collect();

        // the same frame can sit at the edge of two windows
        let mut out: Vec<FrameDecode> = Vec::new();
        for f in per_window.into_iter().flatten() {
            let dup = out.iter().any(|g| {
                g.payload == f.payload
                    && (g.f1_hz - f.f1_hz).abs() < 12.0
                    && (g.tsync_s - f.tsync_s).abs() < 0.05
            });
            if !dup {
                out.push(f);
            }
        }
        out
    }
}

/// The strongest unmasked point of the surface in `band`: `(bin, column)`. Ties go
/// to the earliest column, then the lowest bin (Fortran's `maxloc` order).
fn pick_max(s: &Surface, band: Band, mask: Option<&[bool]>) -> (usize, usize) {
    let mut best: Option<(f32, usize, usize)> = None;
    for col in 0..NCOLS {
        for bin in band.ja..=band.jb {
            if mask.is_some_and(|m| !m[col * s.width + (bin - s.lo)]) {
                continue;
            }
            let v = s.at(col, bin);
            if best.is_none_or(|(b, _, _)| v > b) {
                best = Some((v, bin, col));
            }
        }
    }
    best.map_or((band.ja, 0), |(_, bin, col)| (bin, col))
}

/// The rectangle around a peak that is suppressed — `[bin+1−nfz, bin+1+nfz]` ×
/// `[col+1−ntz, col+1+ntz]`, off by one on the high side exactly as upstream.
fn peak_rect(
    band: Band,
    bin: usize,
    col: usize,
) -> (
    core::ops::RangeInclusive<usize>,
    core::ops::RangeInclusive<usize>,
) {
    (
        (bin + 1).saturating_sub(NFZ).max(band.ja)..=(bin + 1 + NFZ).min(band.jb),
        (col + 1).saturating_sub(NTZ)..=(col + 1 + NTZ).min(NCOLS - 1),
    )
}

/// Zero the surface around a peak.
fn suppress(s: &mut Surface, band: Band, bin: usize, col: usize) {
    let (bins, cols) = peak_rect(band, bin, col);
    for c in cols {
        for b in bins.clone() {
            s.set(c, b, 0.0);
        }
    }
}

/// Channel 0's peaks: `nc` of them, each masking its neighbourhood in a private
/// mask, so a candidate that later fails does not spoil the surface channels 1 and
/// 2 will search.
fn pick_masked(s: &Surface, band: Band, nc: usize) -> Vec<(usize, usize)> {
    let mut mask = alloc::vec![false; s.data.len()];
    for col in 0..NCOLS {
        for bin in band.ja..=band.jb {
            mask[col * s.width + (bin - s.lo)] = true;
        }
    }
    (0..nc)
        .map(|_| {
            let (bin, col) = pick_max(s, band, Some(&mask));
            let (bins, cols) = peak_rect(band, bin, col);
            for c in cols {
                for b in bins.clone() {
                    mask[c * s.width + (b - s.lo)] = false;
                }
            }
            (bin, col)
        })
        .collect()
}

/// Refine a candidate's frequency and start time (`jtty_peakup.f90`): search ±4 ms
/// and ±2.5 Hz (0.5 Hz steps) for the position where the 13 sync symbols, each
/// correlated on its own, add up strongest, then fit a line to the unwrapped
/// phase of those 13 phasors to remove the sub-half-hertz residual — accepted only
/// when the fit is clean (rms residual under 1 rad) and moves the frequency by at
/// most 0.5 Hz. Returns `(start s, frequency Hz, strength)`.
fn peakup(c0: &[Complex32], csync: &[Complex32], xdt0: f32, f0: f32) -> (f32, f32, f32) {
    const HOP: usize = 4;
    let npsync = SYNC_SYMBOLS * NSS;
    let nchunk = c0.len();
    let dt = 1.0 / FS6;
    let qstep: [Complex32; SYNC_SYMBOLS] =
        core::array::from_fn(|i| csync[i * NSS + HOP] * csync[i * NSS].conj());
    let ia = ((xdt0 - 0.004) / dt).round().max(0.0) as usize;
    let ib_signed = ((xdt0 + 0.004) / dt).round().min((nchunk - npsync) as f32) as isize;

    let (mut pmax, mut fpk, mut xdtpk) = (0f32, 0f32, 0f32);
    let mut zbest = [Complex32::new(0.0, 0.0); SYNC_SYMBOLS];
    let mut c1 = alloc::vec![Complex32::new(0.0, 0.0); nchunk];
    if ib_signed >= ia as isize {
        let ib = ib_signed as usize;
        for idf in -5i32..=5 {
            let a1 = -f0 + 0.5 * idf as f32;
            dsp::shift_frequency(&c0[..ib + npsync], &mut c1[..ib + npsync], FS6, a1);
            let mut zcur: [Complex32; SYNC_SYMBOLS] = core::array::from_fn(|i| {
                (0..NSS).fold(Complex32::new(0.0, 0.0), |acc, n| {
                    acc + csync[i * NSS + n].conj() * c1[ia + i * NSS + n]
                })
            });
            for i0 in (ia..=ib).step_by(HOP) {
                let p: f32 = zcur.iter().map(|z| z.norm_sqr()).sum();
                if p > pmax {
                    pmax = p;
                    fpk = -a1;
                    xdtpk = i0 as f32 * dt;
                    zbest = zcur;
                }
                if i0 + HOP <= ib {
                    for (i, z) in zcur.iter_mut().enumerate() {
                        let istart = i * NSS;
                        let removed = (0..HOP).fold(Complex32::new(0.0, 0.0), |acc, r| {
                            acc + csync[istart + r].conj() * c1[i0 + istart + r]
                        });
                        *z = qstep[i] * (*z - removed);
                        *z += (0..HOP).fold(Complex32::new(0.0, 0.0), |acc, r| {
                            acc + csync[istart + NSS - HOP + r].conj() * c1[i0 + istart + NSS + r]
                        });
                    }
                }
            }
        }
    }

    // stage 2: the 13 phasors ride a near-linear phase ramp
    if pmax > 0.0 {
        let tsym = NSS as f32 / FS6;
        let two_pi = 2.0 * PI;
        let phase: Vec<f32> = zbest.iter().map(|z| z.im.atan2(z.re)).collect();
        // unwrap: each step is the phase difference folded into (−π, π]
        let mut u2 = Vec::with_capacity(SYNC_SYMBOLS);
        u2.push(phase[0]);
        for i in 1..SYNC_SYMBOLS {
            let mut d = phase[i] - phase[i - 1];
            while d > PI {
                d -= two_pi;
            }
            while d < -PI {
                d += two_pi;
            }
            u2.push(u2[i - 1] + d);
        }
        let (xm, ym) = (6.0f32, u2.iter().sum::<f32>() / 13.0);
        let (sxy, sxx) = u2
            .iter()
            .enumerate()
            .fold((0f32, 0f32), |(sxy, sxx), (i, &u)| {
                (
                    sxy + (i as f32 - xm) * (u - ym),
                    sxx + (i as f32 - xm).powi(2),
                )
            });
        let slope = sxy / sxx;
        let intercept = ym - slope * xm;
        let resid_rms = (u2
            .iter()
            .enumerate()
            .map(|(i, &u)| (u - (slope * i as f32 + intercept)).powi(2))
            .sum::<f32>()
            / 13.0)
            .sqrt();
        let dfhz = slope / (two_pi * tsym);
        if resid_rms < 1.0 && dfhz.abs() <= 0.5 {
            let ztot = zbest
                .iter()
                .enumerate()
                .fold(Complex32::new(0.0, 0.0), |acc, (i, &z)| {
                    acc + z * Complex32::new((-slope * i as f32).cos(), (-slope * i as f32).sin())
                });
            fpk += dfhz;
            pmax = ztot.norm_sqr();
        }
    }
    (xdtpk, fpk, pmax)
}
