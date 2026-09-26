//! The JTTY receiver: audio → frames → messages.
//!
//! Ported from WSJT-X `lib/jtty/jtty_mdecode.f90` (`jtty_mdecode`,
//! `jtty_mdecode_step`, `process_channel`, `decode_and_merge`) and
//! `jtty_peakup.f90`, tag `v3.2.0-rc1`. A [`FrameDecode`] is one validated frame,
//! with where and when it was found; [`Receiver::scan_messages`] runs the whole
//! chain over a recording and returns messages ([`MessageUpdate`]).
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
//!    is taken. Channel 0's are refined by `peakup` (a search over ±4 ms and
//!    ±2.5 Hz); channels 1 and 2 are searched only after channel 0's successes
//!    are erased from the surface.
//! 4. **Per candidate**: shift it to 0 Hz, count the sync tones it gets right and
//!    estimate the signal-to-noise ratio from them (a **gate**), correlate the 46
//!    data symbols, run the decode ladder, and keep the payload only if it is a
//!    valid source word ([`source::decode_payload`]).
//!
//! ## Several signals
//!
//! What makes it a multi-signal receiver:
//!
//! - **Subtraction.** A decoded frame is re-encoded and taken off the window's
//!   analytic signal ([`super::subtract`]), and the channel is searched again on
//!   the residual — a weak station under a strong one appears only then. Channel 0
//!   is run twice if it subtracted anything, and channels 1 and 2 likewise.
//! - **Retro re-sweep.** A frame that overlaps a strong signal in an *earlier*
//!   window can only be found with that signal gone, so for every signal a window
//!   subtracted, the three windows before it are searched again with it removed.
//! - **Sticky-sync retry.** If a channel decoded nothing but an active message is
//!   due to continue in this window at a remembered frequency and time, the frame
//!   is decoded there without the gate — one attempt.
//! - **Assembly** ([`super::assemble`]): duplicate frames are absorbed and the rest
//!   joined into messages.
//!
//! **Schedule (D4).** Upstream decodes a pass's candidates one after another, each
//! on what the previous ones' subtraction left. Here a pass is decoded at once, on
//! rayon's pool; then the results are settled *in order* (duplicates dropped,
//! subtractions made, messages merged), and the candidates that failed are decoded
//! again on the residual while a round keeps subtracting. That is not the same
//! schedule, so results can differ in edge cases — measured in
//! `docs/notes/JTTY_UPSTREAM.md`, "P3 results". Windows themselves are taken in
//! order: the message state, and the retry that reads it, depend on the windows
//! before. The analytic signal and sync surface of a window do not, and a
//! recording computes those for a batch of windows ahead, in parallel.
//!
//! Every parallel step collects in order, so the output does not depend on the
//! thread count.

use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::f32::consts::PI;

use num_complex::Complex32;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::assemble::{Assembler, FRAME_PERIOD_S, MessageUpdate};
use super::correlate::ToneRefs;
use super::dsp::{self, FS6, NSS, db};
use super::ladder::Ladder;
use super::source::{self, Atom};
use super::subtract::subtract_frame;
use super::{FRAME_SYMBOLS, INFO_BITS, NSPS, Payload, SYNC, SYNC_SYMBOLS, crc, tbcc};

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
/// Decimation of the sync search when [`Params::decimate_sync`] is on: the product of window and
/// sync wave is summed 16 samples at a time, so `NFFT / 16` = 512 points cover 375 Hz.
const SYNC_DECIM: usize = 16;
/// Sync tones (of 13) a channel-0 candidate's *unrefined* gate must see before
/// [`Params::raw_first`] spends a peak-up on it; noise expects about 3.3.
const RAW_FIRST_MIN_SYNC: usize = 6;
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
    /// Take each decoded frame off the signal and search again, and re-search the
    /// windows before it (on, as upstream). Off, every window is searched once,
    /// as a single-signal receiver would: a weak station under a strong one is
    /// then lost.
    pub subtract: bool,
    /// Decode a pass's candidates one after another, each against the signal as the
    /// earlier ones' subtraction left it, as upstream does. Off (the default), a pass
    /// is decoded at once — on rayon's pool under `parallel` — and the candidates that
    /// failed are decoded again after a subtraction. Sequential gives up that
    /// parallelism, and in return the sidelobes of a signal that has just decoded are
    /// gated against what is left of it, not against the signal: on a band with
    /// stations they are most of the candidates the ladder rejects (#499).
    pub sequential: bool,
    /// Carry every frame that decodes into the windows after it: a frame is 1.888 s long
    /// and a window starts every quarter of that, so the same transmission is in up to
    /// four windows, and in the three after the one that decoded it only its tail is
    /// there — which the sync search finds partial matches in, and the ladder then
    /// rejects. With this on, a decoded frame is subtracted from the later windows it
    /// overlaps too, so those never reach the ladder (a persistent residual, as a
    /// continuously running receiver would keep). Off is upstream's behaviour (#499).
    pub carry: bool,
    /// Search channel 0 only; channels 1 and 2 (the fixed side channels at 1350 and 1650 Hz) are
    /// skipped. Their candidates are not gated on channel 0's rules, and on a weak signal they
    /// were a second, unrefined attempt at the same peak, worth 19 of 360 weak passes on
    /// `jtty_sweep` when nothing replaces them — [`Self::raw_first`] does (#499).
    pub ch0_only: bool,
    /// Build the sync surface at 1/16 the FFT length (512 points, the same 0.732 Hz bins) by
    /// summing the sync-wave product 16 samples at a time after mixing the band centre to DC.
    /// Falls back to the full transform when the band does not fit 375 Hz. Made the same
    /// decision on all 360 `jtty_sweep` files as the 8 192-point surface; the point is a
    /// transform that fits the LX7's data cache. Meant for [`Self::ch0_only`] (the union band
    /// of all three channels is 600 Hz and never fits) (#499).
    pub decimate_sync: bool,
    /// Channel 0 tries each pick unrefined first and refines it (`peakup`) only if that attempt
    /// failed after its sync gate saw at least 6 of 13 tones; the default refines every pick
    /// before its gate. Refining pulls a weak pick towards a noise peak as often as away
    /// from one: 164 weak passes of 360 against the default's 160, with no added unexpected
    /// decodes, and in noise alone 1.8 refinements a window instead of 5.0, for 34 % more ladder
    /// calls (#499).
    pub raw_first: bool,
}

impl Params {
    /// The three search options an embedded receiver wants together: [`Self::ch0_only`],
    /// [`Self::decimate_sync`], [`Self::raw_first`].
    #[must_use]
    pub fn embedded(self) -> Self {
        Self {
            ch0_only: true,
            decimate_sync: true,
            raw_first: true,
            ..self
        }
    }
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
            subtract: true,
            sequential: false,
            carry: false,
            ch0_only: false,
            decimate_sync: false,
            raw_first: false,
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
    #[cfg(feature = "jtty-stats")]
    stats: super::stats::Stats,
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
fn search_band(fc: f32, fwid: f32, nfab: Option<(f32, f32)>) -> Option<(Band, f32)> {
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
    (ja <= jb).then_some((Band { ja, jb }, fc))
}

/// The three frequency channels of a receive setting: their bin ranges (and
/// centres) and the bin range the sync surface must cover.
type Channels = ([Option<(Band, f32)>; 3], usize, usize);

fn channels(p: &Params) -> Option<Channels> {
    let side = |fc: f32| {
        if p.ch0_only {
            None
        } else {
            search_band(fc, 150.0, Some((p.nfa_hz, p.nfb_hz)))
        }
    };
    let chans = [
        search_band(p.f0_hz, p.ftol_hz, None),
        side(1350.0),
        side(1650.0),
    ];
    let lo = chans.iter().flatten().map(|(b, _)| b.ja).min()?;
    let hi = chans.iter().flatten().map(|(b, _)| b.jb).max()?;
    Some((chans, lo, hi))
}

/// What a window needs before any state is consulted: its analytic signal and the
/// sync surface built from it. Independent of every other window, so a recording
/// computes these ahead, in parallel.
struct Pre {
    c0: Vec<Complex32>,
    surface: Surface,
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
            #[cfg(feature = "jtty-stats")]
            stats: Default::default(),
        }
    }

    /// The work counters and stage timers so far (`jtty-stats`), the ladder's rung counts included.
    #[cfg(feature = "jtty-stats")]
    pub fn stats(&self) -> super::stats::Snapshot {
        let mut s = self.stats.snapshot();
        s.rungs = self.ladder.rung_counts();
        s
    }

    /// Every candidate that passed the sync gate and reached the ladder (`jtty-stats`).
    #[cfg(feature = "jtty-stats")]
    pub fn gated_candidates(&self) -> Vec<super::stats::GatedCandidate> {
        self.stats.gated()
    }

    /// Zero the counters and timers (`jtty-stats`).
    #[cfg(feature = "jtty-stats")]
    pub fn reset_stats(&self) {
        self.stats.reset();
        self.ladder.reset_rung_counts();
    }

    /// [`Self::new`] with the ladder's trellis metrics in `f32` ([`Ladder::with_f32_metrics`]).
    pub fn with_f32_metrics(mut self) -> Self {
        self.ladder = self.ladder.with_f32_metrics();
        self
    }

    /// The frames whose start lies in the first quarter frame of `audio`, which
    /// must be exactly [`NCHUNK`] samples of 12 kHz audio beginning `t0_s` seconds
    /// into the recording — one window, with its subtraction passes but on its
    /// own: no earlier windows, no retro re-sweep, no message state.
    ///
    /// # Panics
    /// If `audio.len() != NCHUNK`.
    pub fn decode_window(&self, audio: &[i16], t0_s: f32, p: &Params) -> Vec<FrameDecode> {
        let mut asm = Assembler::new();
        let mut frames = Vec::new();
        self.analyze(
            audio,
            t0_s,
            p,
            None,
            &[],
            None,
            &mut asm,
            &mut |_| {},
            &mut frames,
        );
        frames
    }

    /// One window against the message state `asm`: frames are merged into it as
    /// they are found and pushed to `frames` if they contributed to a message;
    /// `interferer` is a signal already decoded elsewhere that is subtracted
    /// before anything is searched (the retro re-sweep). Returns what this call
    /// subtracted, for the re-sweep of the windows before it.
    ///
    /// The order is upstream's: channel 0, and again if it subtracted anything;
    /// then channels 1 and 2 together, and again if they did.
    fn analyze(
        &self,
        audio: &[i16],
        t0_s: f32,
        p: &Params,
        interferer: Option<&Subtracted>,
        carried: &[Subtracted],
        pre: Option<Pre>,
        asm: &mut Assembler,
        sink: &mut dyn FnMut(MessageUpdate),
        frames: &mut Vec<FrameDecode>,
    ) -> Vec<Subtracted> {
        assert_eq!(audio.len(), NCHUNK, "a window is exactly NCHUNK samples");
        if interferer.is_some() {
            stat_add!(self, RetroWindows, 1);
        } else {
            stat_add!(self, Windows, 1);
        }
        let Some((chans, lo, hi)) = channels(p) else {
            return Vec::new();
        };
        let (mut c0, surface) = match pre {
            // the state-independent work, already done (only valid with no interferer)
            Some(Pre { c0, surface }) if interferer.is_none() && carried.is_empty() => {
                (c0, Some(surface))
            }
            // the analytic signal is done, the surface is not: frames carried in change it
            Some(Pre { c0, .. }) if interferer.is_none() => (c0, None),
            _ => {
                let mut c0 = self.analytic(audio);
                if let Some(x) = interferer {
                    stat_add!(self, Subtractions, 1);
                    stat_time!(self, Subtract);
                    subtract_frame(&mut c0, &x.tones(), x.f1_hz, x.tsync_s - t0_s);
                }
                (c0, None)
            }
        };
        for x in carried {
            stat_add!(self, Subtractions, 1);
            stat_time!(self, Subtract);
            subtract_frame(&mut c0, &x.tones(), x.f1_hz, x.tsync_s - t0_s);
        }
        let mut w = Work {
            rx: self,
            p,
            t0: t0_s,
            c0,
            surface,
            lo,
            hi,
            seen: Vec::new(),
            ch0_ok: Vec::new(),
            subtracted: Vec::new(),
            subtractions: 0,
            any_sub: false,
            asm,
            sink,
            frames,
        };
        // Phase A: channel 0, twice if the first pass subtracted anything
        for pass in 1..=2 {
            if pass == 2 && !w.any_sub {
                break;
            }
            if let Some((band, fc)) = chans[0] {
                w.channel(0, band, fc, p.ftol_hz);
            }
        }
        // Phase B: channels 1 and 2, twice if the first pass subtracted anything
        w.any_sub = false;
        for pass in 1..=2 {
            if pass == 2 && !w.any_sub {
                break;
            }
            for ch in [1usize, 2] {
                if let Some((band, fc)) = chans[ch] {
                    w.channel(ch as u8, band, fc, 150.0);
                }
            }
            w.surface = None; // rebuilt from what is left before a second pass
        }
        w.subtracted
    }

    /// The sync surface of the band `p` searches over the analytic window `c0`, as a checksum
    /// (for `embedded-shared`'s `jtty-bench`, which times it on the LX7).
    #[doc(hidden)]
    pub fn bench_surface(&self, c0: &[Complex32], p: &Params) -> f32 {
        let Some((_, lo, hi)) = channels(p) else {
            return 0.0;
        };
        let s = self.sync_surface(c0, lo, hi, p.decimate_sync);
        s.data.iter().step_by(97).sum()
    }

    /// Everything a window costs after its analytic signal, from the analytic window `c0`:
    /// sync surface, picks, candidates, ladder. Returns the number of frames found (for
    /// `jtty-bench`, which times it on the LX7, where the analytic stage itself cannot run).
    #[doc(hidden)]
    pub fn bench_window(&self, c0: Vec<Complex32>, p: &Params) -> usize {
        let Some((_, lo, hi)) = channels(p) else {
            return 0;
        };
        let surface = self.sync_surface(&c0, lo, hi, p.decimate_sync);
        let audio = alloc::vec![0i16; NCHUNK];
        let mut asm = Assembler::new();
        let mut frames = Vec::new();
        self.analyze(
            &audio,
            0.0,
            p,
            None,
            &[],
            Some(Pre { c0, surface }),
            &mut asm,
            &mut |_| {},
            &mut frames,
        );
        frames.len()
    }

    fn prepare(&self, audio: &[i16], p: &Params) -> Option<Pre> {
        let (_, lo, hi) = channels(p)?;
        let c0 = self.analytic(audio);
        let surface = self.sync_surface(&c0, lo, hi, p.decimate_sync);
        Some(Pre { c0, surface })
    }

    /// The analytic signal of a window, counted and timed.
    fn analytic(&self, audio: &[i16]) -> Vec<Complex32> {
        stat_add!(self, Analytic, 1);
        stat_time!(self, Analytic);
        dsp::analytic_6k(audio)
    }

    /// Every candidate's outcome, in order; parallel across candidates.
    fn process_all(
        &self,
        c0: &[Complex32],
        picks: &[Pick],
        t0_s: f32,
        p: &Params,
    ) -> Vec<Option<Outcome>> {
        let one = |pick: &Pick| self.process(c0, pick, t0_s, p, true);
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
    fn sync_surface(&self, c0: &[Complex32], lo: usize, hi: usize, decimate: bool) -> Surface {
        stat_add!(self, SurfaceBuilds, 1);
        stat_time!(self, Surface);
        if decimate && hi - lo + 5 <= NFFT / SYNC_DECIM {
            return self.sync_surface_decimated(c0, lo, hi);
        }
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
                    || (dsp::with_planner(|p| p.plan_forward(NFFT)), zero()),
                    |(fft, buf), col| column(col, fft.as_ref(), buf),
                )
                .collect()
        };
        #[cfg(not(feature = "parallel"))]
        let rows: Vec<Vec<f32>> = {
            let fft = dsp::with_planner(|p| p.plan_forward(NFFT));
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

    /// [`Self::sync_surface`] at 1/[`SYNC_DECIM`] the FFT length ([`Params::decimate_sync`]): the
    /// conjugated sync wave is mixed by the band centre once, each column's product with the
    /// window is summed `SYNC_DECIM` samples at a time, and the 512-point transform's bins are the
    /// full transform's, shifted by the centre bin (the mixer's phase at a column's first sample
    /// only rotates all bins together, and only powers are kept).
    fn sync_surface_decimated(&self, c0: &[Complex32], lo: usize, hi: usize) -> Surface {
        const NF: usize = NFFT / SYNC_DECIM;
        let width = hi - lo + 1;
        let mid = (lo + hi) / 2;
        // The sync wave is 13 symbols, each a pure tone that turns a whole number of times, so
        // (conjugated and mixed by the band centre) it is `c_i · b_t[n]` for symbol `i` sending
        // tone `t = SYNC[i]`: four 192-sample tables and one phasor per symbol, 6 KB where the
        // whole wave is 20 KB. Read per column with a window that is also 20 KB, the whole wave
        // did not stay in the LX7's 32 KB data cache (547 ms a surface, 2.3 ms a column, against
        // 0.23 ms for the transform).
        const _: () = assert!(NSS.is_multiple_of(SYNC_DECIM));
        let wm = f64::from(mid as f32 * DF) * core::f64::consts::TAU / f64::from(FS6);
        let base: [Vec<Complex32>; 4] = core::array::from_fn(|t| {
            let psi = -(core::f64::consts::TAU * t as f64 / NSS as f64 + wm);
            let step = Complex32::new(psi.cos() as f32, psi.sin() as f32);
            let mut w = Complex32::new((-wm).cos() as f32, (-wm).sin() as f32);
            (0..NSS)
                .map(|_| {
                    let now = w;
                    w *= step;
                    now
                })
                .collect()
        });
        let per_symbol = -wm * NSS as f64;
        let turn = Complex32::new(per_symbol.cos() as f32, per_symbol.sin() as f32);
        let mut phasor = Complex32::new(1.0, 0.0);
        let symbol_phasor: [Complex32; SYNC_SYMBOLS] = core::array::from_fn(|_| {
            let now = phasor;
            phasor *= turn;
            now
        });
        let groups = NSS / SYNC_DECIM;
        let m = SYNC_SYMBOLS * groups;
        let column =
            |col: usize, fft: &dyn crate::engine::fft::Fft, buf: &mut Vec<Complex32>| -> Vec<f32> {
                let x = &c0[col * COL_STEP..];
                for (i, &sent) in SYNC.iter().enumerate() {
                    let table = &base[usize::from(sent)];
                    for g in 0..groups {
                        let (at, out) = (g * SYNC_DECIM, i * groups + g);
                        buf[out] = symbol_phasor[i]
                            * table[at..at + SYNC_DECIM]
                                .iter()
                                .zip(&x[i * NSS + at..i * NSS + at + SYNC_DECIM])
                                .fold(Complex32::new(0.0, 0.0), |a, (&r, &v)| a + r * v);
                    }
                }
                buf[m..].fill(Complex32::new(0.0, 0.0));
                fft.process(buf);
                let power: Vec<f32> = (lo - 2..=hi + 2)
                    .map(|b| buf[(b + NF - mid % NF) % NF].norm_sqr())
                    .collect();
                power
                    .windows(5)
                    .map(|w| w[0] + 2.0 * w[1] + 3.0 * w[2] + 2.0 * w[3] + w[4])
                    .collect()
            };
        let zero = || alloc::vec![Complex32::new(0.0, 0.0); NF];
        #[cfg(feature = "parallel")]
        let rows: Vec<Vec<f32>> = {
            use rayon::prelude::*;
            (0..NCOLS)
                .into_par_iter()
                .with_min_len(16)
                .map_init(
                    || (dsp::with_planner(|p| p.plan_forward(NF)), zero()),
                    |(fft, buf), col| column(col, fft.as_ref(), buf),
                )
                .collect()
        };
        #[cfg(not(feature = "parallel"))]
        let rows: Vec<Vec<f32>> = {
            let fft = dsp::with_planner(|p| p.plan_forward(NF));
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
    fn process(
        &self,
        c0: &[Complex32],
        pick: &Pick,
        t0_s: f32,
        p: &Params,
        peak: bool,
    ) -> Option<Outcome> {
        if peak && p.raw_first && pick.channel == 0 {
            let (outcome, tones) = self.attempt(c0, pick, t0_s, p, peak, false);
            if outcome.is_some() || tones < RAW_FIRST_MIN_SYNC {
                return outcome;
            }
        }
        self.attempt(c0, pick, t0_s, p, peak, true).0
    }

    /// One try at a candidate: with `refine` (channel 0 only) the pick is first pulled to the
    /// local peak. Also returns the sync tones (of 13) the gate saw.
    fn attempt(
        &self,
        c0: &[Complex32],
        pick: &Pick,
        t0_s: f32,
        p: &Params,
        peak: bool,
        refine: bool,
    ) -> (Option<Outcome>, usize) {
        let mut tones = 0;
        let outcome = self.attempt_inner(c0, pick, t0_s, p, peak, refine, &mut tones);
        (outcome, tones)
    }

    #[allow(clippy::too_many_arguments)]
    fn attempt_inner(
        &self,
        c0: &[Complex32],
        pick: &Pick,
        t0_s: f32,
        p: &Params,
        peak: bool,
        refine: bool,
        gate_tones: &mut usize,
    ) -> Option<Outcome> {
        let (xdt, f1) = if pick.channel == 0 && peak && refine {
            stat_add!(self, Peakups, 1);
            stat_time!(self, Peakup);
            let (x, f, _) = peakup(c0, &self.csync, pick.xdt_s, pick.f_hz);
            (x, f)
        } else {
            (pick.xdt_s, pick.f_hz)
        };
        // The window is not shifted to put the candidate's lowest tone at 0 Hz, as upstream does
        // (a 14 160-sample mix and an allocation of the same size, 12 ms and 113 KB of PSRAM on the
        // LX7); the four tone references are rotated instead, 768 samples (#499).
        let rot = {
            stat_add!(self, Shifts, 1);
            stat_time!(self, Shift);
            self.refs.rotated(-f1, FS6)
        };

        // ---- sync gate: which tone is strongest in each of the 13 sync symbols
        let start = (xdt * FS6).round() as usize;
        #[cfg(feature = "jtty-stats")]
        let gate_span = self.stats.time(super::stats::Stage::Gate);
        let (mut pt, mut pa) = (0f32, 0f32);
        let mut hits = 0usize;
        for (j, &sent) in SYNC.iter().enumerate() {
            let i0 = start + j * NSS;
            if i0 + NSS > c0.len() {
                break;
            }
            let pow: [f32; 4] = core::array::from_fn(|k| rot.power(k, &c0[i0..i0 + NSS]));
            let best = (0..4).fold(0, |b, k| if pow[k] > pow[b] { k } else { b });
            hits += usize::from(best == usize::from(sent));
            pt += pow[usize::from(sent)];
            pa += pow.iter().sum::<f32>();
        }
        *gate_tones = hits;
        let pn = (pa - pt) / 3.0;
        let snr = if pn > 0.0 { db(pt / pn) } else { -99.9 };
        let passes = if !peak {
            true // a retry at a remembered sync point is not gated
        } else if pick.channel == 0 {
            hits >= CH0_MIN_SYNC && snr >= p.smin_db
        } else {
            hits >= OTHER_MIN_SYNC && snr >= OTHER_MIN_SNR_DB
        };
        #[cfg(feature = "jtty-stats")]
        drop(gate_span);
        if !passes {
            stat_add!(self, GateFail, 1);
            return None;
        }
        stat_add!(self, GatePass, 1);

        // ---- decode
        let (zsym, zhalf) = {
            stat_add!(self, Correlations, 1);
            stat_time!(self, Correlate);
            rot.correlate_payload(c0, start + SYNC_SYMBOLS * NSS)
        };
        stat_add!(self, LadderCalls, 1);
        let accepted = {
            stat_time!(self, Ladder);
            self.ladder.decode(&zsym, &zhalf)
        };
        #[cfg(feature = "jtty-stats")]
        self.stats.record(super::stats::GatedCandidate {
            window_s: t0_s,
            channel: pick.channel,
            f1_hz: f1,
            tsync_s: t0_s + xdt,
            nsync: hits,
            snr_db: snr,
            accepted: accepted.is_some(),
        });
        let accepted = accepted?;
        stat_add!(self, LadderAccepts, 1);
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

    /// Every message in a recording, as its updates in the order they happened.
    ///
    /// A window every [`STEP`] samples; for each, the message state is aged
    /// ([`Assembler::prune`]), the window is decoded with its subtraction passes,
    /// and for every signal it subtracted the three windows before it are
    /// searched again with that signal removed (the retro re-sweep — a frame
    /// that overlaps a strong signal is only findable once it is gone).
    pub fn scan_messages(&self, audio: &[i16], p: &Params) -> Vec<MessageUpdate> {
        let mut updates = Vec::new();
        self.run(audio, p, &mut |u| updates.push(u), &mut Vec::new());
        updates
    }

    /// Every frame in a recording that contributed to a message, in the order
    /// found (a repeat of a frame already seen is not one).
    pub fn scan(&self, audio: &[i16], p: &Params) -> Vec<FrameDecode> {
        let mut frames = Vec::new();
        self.run(audio, p, &mut |_| {}, &mut frames);
        frames
    }

    fn run(
        &self,
        audio: &[i16],
        p: &Params,
        sink: &mut dyn FnMut(MessageUpdate),
        frames: &mut Vec<FrameDecode>,
    ) {
        if audio.len() < NCHUNK {
            return;
        }
        let n = (audio.len() - NCHUNK) / STEP + 1;
        let audio = Audio {
            buf: audio,
            base: 0,
        };
        let mut asm = Assembler::new();
        // Windows are decoded in order — the message state, and the sticky-sync
        // retry that reads it, depend on the windows before — but the analytic
        // signal and sync surface of a window do not, and they are most of its
        // cost, so a batch of windows has them computed on the pool first.
        const BATCH: usize = 16;
        for first in (0..n).step_by(BATCH) {
            let batch: Vec<usize> = (first..n.min(first + BATCH)).collect();
            let prepare = |w: &usize| self.prepare(audio.window(*w), p);
            #[cfg(feature = "parallel")]
            let pres: Vec<Option<Pre>> = {
                use rayon::prelude::*;
                batch.par_iter().map(prepare).collect()
            };
            #[cfg(not(feature = "parallel"))]
            let pres: Vec<Option<Pre>> = batch.iter().map(prepare).collect();
            for (w, pre) in batch.into_iter().zip(pres) {
                self.step(&audio, w, p, pre, &mut asm, sink, frames);
            }
        }
    }

    /// Window `w`: age the message state, decode it, and re-sweep the windows
    /// before it for every signal it subtracted.
    #[allow(clippy::too_many_arguments)]
    fn step(
        &self,
        audio: &Audio,
        w: usize,
        p: &Params,
        pre: Option<Pre>,
        asm: &mut Assembler,
        sink: &mut dyn FnMut(MessageUpdate),
        frames: &mut Vec<FrameDecode>,
    ) {
        let t_of = |k: usize| (k * STEP) as f32 / 12_000.0;
        asm.prune(t_of(w), sink);
        // frames decoded earlier that still lie in window `k` (`Params::carry`)
        let carried_in = |asm: &Assembler, k: usize| -> Vec<Subtracted> {
            if !p.carry {
                return Vec::new();
            }
            let (a, b) = (t_of(k), t_of(k) + NCHUNK as f32 / 12_000.0);
            asm.carried
                .iter()
                .filter(|x| x.tsync_s < b && x.tsync_s + super::assemble::FRAME_PERIOD_S > a)
                .cloned()
                .collect()
        };
        let carried = carried_in(asm, w);
        let subtracted = self.analyze(
            audio.window(w),
            t_of(w),
            p,
            None,
            &carried,
            pre,
            asm,
            sink,
            frames,
        );
        for x in &subtracted {
            for k in 1..=super::assemble::MAX_RETRO_STEPS {
                if w >= k {
                    let carried = carried_in(asm, w - k);
                    let _ = self.analyze(
                        audio.window(w - k),
                        t_of(w - k),
                        p,
                        Some(x),
                        &carried,
                        None,
                        asm,
                        sink,
                        frames,
                    );
                }
            }
        }
        if p.carry {
            asm.carried.extend(subtracted);
        }
    }
}

/// Audio whose first sample is sample `base` of the recording: window `k` is the
/// [`NCHUNK`] samples starting at `k · STEP`.
struct Audio<'a> {
    buf: &'a [i16],
    base: usize,
}

impl<'a> Audio<'a> {
    fn window(&self, k: usize) -> &'a [i16] {
        let start = k * STEP - self.base;
        &self.buf[start..start + NCHUNK]
    }
}

/// A receiver fed audio as it arrives (`Receiver::scan_messages`, a sample at a
/// time): the same windows, subtraction, retro re-sweep, retry and assembly, so
/// the messages it reports are exactly those of a scan of the whole recording,
/// whatever the chunk sizes.
///
/// It keeps only the audio a re-sweep can still reach — the current window and the
/// three before it, about 45 000 samples. A window is decoded as soon as its last
/// sample has arrived, inside [`push`](Self::push), on the caller's thread (and on
/// rayon's pool, under `parallel`); results come out through the callback, which
/// is the shape `STREAMING.md` §4 argues for. A poll-style wrapper for callers
/// that cannot take a closure (the C ABI) is built on top of this, not into it.
///
/// The tables are shared: any number of streams (one per audio channel, say) may
/// hold the same [`Arc<Receiver>`].
///
/// **Time is the sample count.** There is no clock: window `k` starts at sample
/// `k · STEP`, and a frame continues a message only if it starts one to three frame
/// periods (±0.1 s) after the last. From a live source the count must therefore
/// follow real time — a dropped run of samples shifts every later frame earlier and
/// breaks the message, so push zeros for what was lost (or [`reset`](Self::reset)
/// after a long gap). A source clock a few hundred ppm off is harmless.
pub struct Stream {
    rx: Arc<Receiver>,
    params: Params,
    buf: Vec<i16>,
    /// index in the recording of `buf[0]`
    base: usize,
    /// the next window to decode
    next: usize,
    asm: Assembler,
}

impl Stream {
    /// A stream that starts at sample 0 of a recording.
    pub fn new(rx: Arc<Receiver>, params: Params) -> Self {
        Self {
            rx,
            params,
            buf: Vec::new(),
            base: 0,
            next: 0,
            asm: Assembler::new(),
        }
    }

    /// The receive settings.
    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Change the settings; they apply from the next window.
    pub fn set_params(&mut self, params: Params) {
        self.params = params;
    }

    /// Samples fed so far.
    pub fn samples_seen(&self) -> usize {
        self.base + self.buf.len()
    }

    /// Samples held for the re-sweep of earlier windows: at most about
    /// `NCHUNK + 3 · STEP` (45 000) plus what has just arrived.
    pub fn buffered_samples(&self) -> usize {
        self.buf.len()
    }

    /// Feed 12 kHz mono audio (any number of samples, including none). Every
    /// window this completes is decoded, and each message update it produces is
    /// passed to `on_update`, in the order it happened.
    pub fn push(&mut self, samples: &[i16], on_update: &mut dyn FnMut(MessageUpdate)) {
        self.buf.extend_from_slice(samples);
        let mut frames = Vec::new();
        while self.base + self.buf.len() >= self.next * STEP + NCHUNK {
            let audio = Audio {
                buf: &self.buf,
                base: self.base,
            };
            self.rx.step(
                &audio,
                self.next,
                &self.params,
                None,
                &mut self.asm,
                on_update,
                &mut frames,
            );
            frames.clear();
            self.next += 1;
        }
        // nothing before the oldest window a re-sweep can still reach is needed
        let keep_from = self.next.saturating_sub(super::assemble::MAX_RETRO_STEPS) * STEP;
        if keep_from > self.base {
            self.buf.drain(..keep_from - self.base);
            self.base = keep_from;
        }
    }

    /// The stream has ended: report every message still waiting for a
    /// continuation as incomplete (an [`Assembler::prune`] far in the future).
    pub fn finish(&mut self, on_update: &mut dyn FnMut(MessageUpdate)) {
        self.asm.prune(f32::MAX / 4.0, on_update);
    }

    /// Forget everything and start again at sample 0.
    pub fn reset(&mut self) {
        self.buf.clear();
        self.base = 0;
        self.next = 0;
        self.asm = Assembler::new();
    }
}

/// A frame that was subtracted from a window: enough to subtract it again from
/// the windows before it.
#[derive(Clone, Debug)]
pub struct Subtracted {
    /// Frequency of the frame's lowest tone, Hz.
    pub f1_hz: f32,
    /// Start of the frame, seconds from the start of the audio.
    pub tsync_s: f32,
    /// Its payload, from which the transmitted waveform is rebuilt.
    pub payload: Payload,
}

impl Subtracted {
    fn tones(&self) -> [u8; FRAME_SYMBOLS] {
        super::tx::frame_tones(&self.payload)
    }
}

/// Subtractions and channel-0 successes kept per window (`MAX_SUBTRACTED`, 16).
const MAX_KEPT: usize = 16;
/// Rounds of re-decoding failed candidates against the residual within one pass.
const MAX_ROUNDS: usize = 3;

/// The state of one window while it is being analysed: the analytic signal as it
/// stands after the subtractions so far, the sync surface (until a subtraction
/// spoils it), and everything the duplicate rules and the re-sweep need.
struct Work<'a> {
    rx: &'a Receiver,
    p: &'a Params,
    t0: f32,
    c0: Vec<Complex32>,
    surface: Option<Surface>,
    lo: usize,
    hi: usize,
    /// text and start time of every frame decoded so far in this window
    seen: Vec<(String, f32)>,
    /// `(f1, tsync)` of channel-0 successes, so channels 1 and 2 do not repeat them
    ch0_ok: Vec<(f32, f32)>,
    subtracted: Vec<Subtracted>,
    subtractions: usize,
    any_sub: bool,
    asm: &'a mut Assembler,
    sink: &'a mut dyn FnMut(MessageUpdate),
    frames: &'a mut Vec<FrameDecode>,
}

impl Work<'_> {
    fn surface(&mut self) -> &mut Surface {
        if self.surface.is_none() {
            self.surface =
                Some(
                    self.rx
                        .sync_surface(&self.c0, self.lo, self.hi, self.p.decimate_sync),
                );
        }
        self.surface.as_mut().unwrap()
    }

    /// One pass over one channel (`process_channel`): pick peaks, decode them all
    /// against the signal as it now is, then — in order — drop duplicates,
    /// subtract what decoded and merge it into the messages. If nothing at all
    /// decoded, try the continuation an active message is due to produce.
    fn channel(&mut self, ch: u8, band: Band, fc: f32, fwid: f32) {
        // the surface is built (and timed) before the picks are, so the two stages do not nest
        let _ = self.surface();
        let picks: Vec<Pick> = {
            #[cfg(feature = "jtty-stats")]
            let rx = self.rx;
            stat_time!(rx, Pick);
            self.pick_candidates(ch, band, fwid)
        };
        stat_add!(self.rx, PicksCh0, if ch == 0 { picks.len() } else { 0 });
        stat_add!(self.rx, PicksOther, if ch == 0 { 0 } else { picks.len() });
        self.decode_picks(ch, fc, fwid, picks);
    }

    fn pick_candidates(&mut self, ch: u8, band: Band, fwid: f32) -> Vec<Pick> {
        if ch == 0 {
            let nc = 2usize.max(8usize.min((fwid / (NFZ as f32 * DF)).round() as usize));
            pick_masked(self.surface(), band, nc)
                .into_iter()
                .map(|(bin, col)| pick_at(0, bin, col))
                .collect()
        } else {
            // erase what channel 0 found, so this does not rediscover it
            let found: Vec<f32> = self.ch0_ok.iter().map(|&(f1, _)| f1).collect();
            let surface = self.surface();
            for f1 in found {
                let centre = (f1 / DF).round() as isize;
                let (ja, jb) = (
                    (centre - NFZ as isize).max(0) as usize,
                    (centre + NFZ as isize) as usize,
                );
                for bin in ja.max(band.ja)..=jb.min(band.jb) {
                    (0..NCOLS).for_each(|col| surface.set(col, bin, 0.0));
                }
            }
            (0..OTHER_CANDIDATES)
                .map(|_| {
                    let (bin, col) = pick_max(surface, band, None);
                    suppress(surface, band, bin, col);
                    pick_at(ch, bin, col)
                })
                .collect()
        }
    }

    fn decode_picks(&mut self, ch: u8, fc: f32, fwid: f32, picks: Vec<Pick>) {
        // Decode every candidate against the signal as it now is. Upstream takes
        // them one after another, each seeing what the earlier ones' subtraction
        // left; a whole pass is decoded at once here, so whenever a round subtracted
        // something the candidates that failed are decoded again against the
        // residual, until a round subtracts nothing.
        let mut decoded = false;
        let mut todo: Vec<Pick> = Vec::new();
        if self.p.sequential {
            // upstream's order: each candidate against what the earlier ones left
            for pick in &picks {
                let outcome = self.rx.process(&self.c0, pick, self.t0, self.p, true);
                decoded |= self.settle(alloc::vec![outcome], ch);
            }
        } else {
            todo = picks;
        }
        for _round in 0..MAX_ROUNDS {
            let before = self.subtractions;
            #[cfg(feature = "jtty-stats")]
            let ladder_before = self
                .rx
                .stats
                .snapshot()
                .count(super::stats::Counter::LadderCalls);
            let outcomes = self.rx.process_all(&self.c0, &todo, self.t0, self.p);
            #[cfg(feature = "jtty-stats")]
            if _round > 0 {
                let now = self
                    .rx
                    .stats
                    .snapshot()
                    .count(super::stats::Counter::LadderCalls);
                stat_add!(self.rx, RetryCandidates, todo.len());
                stat_add!(self.rx, RetryLadderCalls, now - ladder_before);
            }
            let failed: Vec<Pick> = todo
                .iter()
                .zip(&outcomes)
                .filter(|(_, o)| o.is_none())
                .map(|(p, _)| *p)
                .collect();
            decoded |= self.settle(outcomes, ch);
            if self.subtractions == before || failed.is_empty() || !self.p.subtract {
                break;
            }
            stat_add!(self.rx, ExtraRounds, 1);
            todo = failed;
        }

        if !decoded {
            // sticky-sync retry: an active message whose next frame is due to start
            // in this window, on this channel's frequencies — one attempt
            let due: Option<(f32, f32)> = self.asm.continuations().find(|&(f1, tsync)| {
                (f1 - fc).abs() <= fwid
                    && ((self.t0 - tsync) - FRAME_PERIOD_S).abs() <= 0.1
                    && tsync + FRAME_PERIOD_S - self.t0 >= 0.0
            });
            if let Some((f1, tsync)) = due {
                stat_add!(self.rx, StickyRetries, 1);
                let pick = Pick {
                    channel: ch,
                    f_hz: f1,
                    xdt_s: tsync + FRAME_PERIOD_S - self.t0,
                };
                let outcome = self.rx.process(&self.c0, &pick, self.t0, self.p, false);
                decoded = self.settle(alloc::vec![outcome], ch);
            }
        }
        let _ = decoded;
    }

    /// The sequential part for a pass's outcomes; `true` if any candidate decoded
    /// (a duplicate counts).
    fn settle(&mut self, outcomes: Vec<Option<Outcome>>, ch: u8) -> bool {
        let mut any = false;
        for o in outcomes.into_iter().flatten() {
            any = true;
            let f = &o.frame;
            if ch == 0 && self.ch0_ok.len() < MAX_KEPT {
                self.ch0_ok.push((f.f1_hz, f.tsync_s));
            }
            let dupe = self
                .seen
                .iter()
                .any(|(t, ts)| *t == o.text && (ts - f.tsync_s).abs() < DUPE_TIME_S)
                || (ch != 0
                    && self.ch0_ok.iter().any(|&(f1, ts)| {
                        (f1 - f.f1_hz).abs() < SAME_FRAME_HZ
                            && (ts - f.tsync_s).abs() < SAME_FRAME_S
                    }));
            self.seen.push((o.text.clone(), f.tsync_s));
            if dupe {
                continue;
            }
            // take the frame off the signal so weaker ones beneath it can be found
            if self.p.subtract {
                stat_add!(self.rx, Subtractions, 1);
                #[cfg(feature = "jtty-stats")]
                let rx = self.rx;
                stat_time!(rx, Subtract);
                subtract_frame(
                    &mut self.c0,
                    &super::tx::frame_tones(&f.payload),
                    f.f1_hz,
                    f.xdt_s,
                );
                self.any_sub = true;
                self.subtractions += 1;
                self.surface = None;
            }
            if self.p.subtract && self.subtracted.len() < MAX_KEPT {
                self.subtracted.push(Subtracted {
                    f1_hz: f.f1_hz,
                    tsync_s: f.tsync_s,
                    payload: f.payload,
                });
            }
            if self.asm.push_frame(f, &mut *self.sink) {
                self.frames.push(o.frame);
            }
        }
        any
    }
}

fn pick_at(channel: u8, bin: usize, col: usize) -> Pick {
    Pick {
        channel,
        f_hz: bin as f32 * DF,
        xdt_s: (col * COL_STEP) as f32 / FS6,
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
#[doc(hidden)] // public for `embedded-shared`'s `jtty-bench`, which times it on the LX7
pub fn peakup(c0: &[Complex32], csync: &[Complex32], xdt0: f32, f0: f32) -> (f32, f32, f32) {
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
    if ib_signed >= ia as isize {
        let ib = ib_signed as usize;
        // Only samples `ia..ib + npsync` are read, and the shift is applied to those alone, into a
        // buffer of that length (about 3 000 samples, 24 KB, where the whole window is 113 KB):
        // the shift's phase then starts at `ia` instead of at the window's first sample, one
        // constant factor over everything that is read, which the power sums and the slope of the
        // phasors' phase do not see (#499).
        let mut c1 = alloc::vec![Complex32::new(0.0, 0.0); ib + npsync - ia];
        for idf in -5i32..=5 {
            let a1 = -f0 + 0.5 * idf as f32;
            dsp::shift_frequency(&c0[ia..ib + npsync], &mut c1, FS6, a1);
            let mut zcur: [Complex32; SYNC_SYMBOLS] = core::array::from_fn(|i| {
                (0..NSS).fold(Complex32::new(0.0, 0.0), |acc, n| {
                    acc + csync[i * NSS + n].conj() * c1[i * NSS + n]
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
                            acc + csync[istart + r].conj() * c1[i0 - ia + istart + r]
                        });
                        *z = qstep[i] * (*z - removed);
                        *z += (0..HOP).fold(Complex32::new(0.0, 0.0), |acc, r| {
                            acc + csync[istart + NSS - HOP + r].conj()
                                * c1[i0 - ia + istart + NSS + r]
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

#[cfg(all(test, feature = "std"))]
mod profile {
    use super::*;

    /// Where does a window's time go? (`cargo test --release … profile -- --ignored --nocapture`)
    #[test]
    #[ignore]
    fn where_the_time_goes() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../embedded-poc/assets/golden/jtty/260807_134110.wav"
        );
        let Ok(bytes) = std::fs::read(path) else {
            return;
        };
        let audio: Vec<i16> = bytes[44..]
            .chunks(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect();
        let win = &audio[5 * STEP..5 * STEP + NCHUNK]; // has a frame in it
        let rx = Receiver::new();
        let p = Params::default();
        let time = |label: &str, reps: usize, f: &mut dyn FnMut()| {
            let t = std::time::Instant::now();
            (0..reps).for_each(|_| f());
            eprintln!(
                "PROF {label:<28} {:8.3} ms",
                t.elapsed().as_secs_f64() * 1000.0 / reps as f64
            );
        };
        let c0 = dsp::analytic_6k(win);
        time("analytic_6k", 50, &mut || {
            std::hint::black_box(dsp::analytic_6k(win));
        });
        let (lo, hi) = (1636usize, 2459usize);
        time("sync_surface (pool)", 50, &mut || {
            std::hint::black_box(rx.sync_surface(&c0, lo, hi, false));
        });
        let pick = Pick {
            channel: 0,
            f_hz: 1507.0,
            xdt_s: 0.302,
        };
        time("process (a real candidate)", 50, &mut || {
            std::hint::black_box(rx.process(&c0, &pick, 0.0, &p, true));
        });
        let noise = Pick {
            channel: 0,
            f_hz: 1533.0,
            xdt_s: 0.1,
        };
        time("process (a failing candidate)", 50, &mut || {
            std::hint::black_box(rx.process(&c0, &noise, 0.0, &p, true));
        });
        time("peakup alone", 200, &mut || {
            std::hint::black_box(peakup(&c0, &rx.csync, 0.302, 1507.0));
        });
        time("decode_window (whole)", 20, &mut || {
            std::hint::black_box(rx.decode_window(win, 0.0, &p));
        });
    }
}
