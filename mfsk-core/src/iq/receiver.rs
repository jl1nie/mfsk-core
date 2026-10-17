// SPDX-License-Identifier: GPL-3.0-only
//! [`IqReceiver`]: N channels of one wideband IQ stream, cut into slots on
//! UTC from the sample count. It decodes nothing.
//!
//! What it does *not* do is find anything, or decode: the caller says which
//! dial frequency carries which mode ([`IqReceiver::add_channel`]), tells it
//! what UTC the stream is at ([`IqReceiver::set_time`]), and gets back each
//! completed slot as a [`CompletedSlot`]: 12 kHz audio and the slot's index
//! on the UTC grid. Decoding is the caller's, with one
//! [`AnyDecoder`](crate::decoder::AnyDecoder) per channel, so each channel
//! has its own options and its own callsign table, and the decode can run on
//! any thread (a slot is owned and `Send`).
//!
//! ## Time
//!
//! The sample count is the clock. Channel audio index `k` is time
//! `k / 12000` s after sample 0 (each channel's front end drops its own
//! group delay, so index 0 is the audio of IQ sample 0), and slot `j` of a
//! mode with period `T` covers UTC `[j·T, (j+1)·T)`. With no clock set the
//! grid free-runs from sample 0, right for replaying a recording.
//!
//! [`IqReceiver::set_time`] takes observations of the clock, and follows
//! them at a bounded rate ([`SampleClock`]):
//! a drifting crystal or host clock moves slot boundaries by
//! milliseconds, and no slot is lost. A slot always starts on its own
//! boundary: when the stream is slightly faster than the clock its first
//! samples are the previous slot's last. Only a jump past the step threshold
//! drops the slots that straddle it.
//!
//! [`IqReceiver::retune`] and [`IqReceiver::gap`] also drop every open slot:
//! audio that straddles a change of centre or a hole in the samples is not a
//! slot. The sample clock continues. A retune moves every channel that still
//! fits and pauses the rest ([`RetuneReport`]); a paused channel keeps its
//! dial and resumes when a later retune brings it back inside the band.
//!
//! Each slot is scaled to a fixed RMS (the decoders are scale-free; the IQ's
//! own level is not something to inherit), and a silent or NaN slot is not
//! returned.

use alloc::vec::Vec;

use super::track::RateTracker;
use super::{IqError, IqStream, IqToAudio, PfbChannelizer, RateTrackConfig, check_placement};
use crate::decoder::SlotInput;
use crate::registry::{Mode, ProtocolMeta};
use crate::slotgrid::{ClockChange, SampleClock, SlotCutter, SlotGrid};

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// Interleaved I/Q converted per pass.
const BLOCK: usize = 8_192;
/// The finest grid a learned rate offset is applied on, ns: 15 s, a whole number of FT8 and FT4 slots.
/// With longer-slotted channels active it is their common boundary ([`IqReceiver::rate_grid_ns`]).
const SLOT_GRID_NS: i64 = 15_000_000_000;
/// RMS the slot is scaled to, in `i16` units.
const TARGET_RMS: f32 = 2_000.0;
/// How an [`IqReceiver`] turns IQ into each channel's audio. Both meet the
/// same 120 dB selectivity and give the decoders the same audio (the IQ
/// decode tests run through each); they differ in how cost grows.
///
/// - `Direct` (the default): one [`IqToAudio`] per channel, each mixing and
///   decimating from the input rate. Cheapest for the handful of channels an
///   amateur band needs: 0.93 % of a core per channel at 768 kS/s.
/// - `Pfb`: one [`PfbChannelizer`] shared by every channel. A fixed cost of
///   about two and a half `Direct` channels, then a small back end per
///   channel at the sub-band rate, independent of `Fs`: the choice for many
///   channels. Measured break-even and cost are in
///   `docs/notes/IQ_CHANNELIZER.md`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum Channelizer {
    #[default]
    Direct,
    Pfb,
}

/// Handle of a channel added with [`IqReceiver::add_channel`]; not reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ChannelId(pub usize);

/// Whether a channel is being received.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ChannelState {
    Active,
    /// Its audio window no longer fits the IQ band after a retune.
    Paused(IqError),
}

/// What a [`IqReceiver::retune`] did to the channels.
#[non_exhaustive]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RetuneReport {
    /// Active before, and no longer fit.
    pub paused: Vec<ChannelId>,
    /// Paused before, and fit again.
    pub resumed: Vec<ChannelId>,
}

/// One slot of one channel: complete, or, on a channel with prefix points
/// ([`IqReceiver::set_prefix_points`]), the slot so far
/// ([`CompletedSlot::is_whole`]).
#[non_exhaustive]
#[derive(Clone, Debug)]
pub struct CompletedSlot {
    pub channel: ChannelId,
    pub mode: Mode,
    pub dial_hz: f64,
    /// RF Hz per audio Hz about the 3000 Hz channel centre. Normally one;
    /// differs when an acquisition clock is pinned separately from payload rate.
    pub audio_rf_scale: f64,
    /// The slot's index on the grid: UTC `period · T` when a clock is set,
    /// counted from sample 0 otherwise. Consecutive slots of a channel have
    /// consecutive indices.
    pub period: i64,
    /// IQ sample index the slot started at.
    pub start_sample: u64,
    /// UTC of the slot start in ns, when a clock is set.
    pub utc_ns: Option<i64>,
    /// 12 kHz audio from the slot's start, scaled to a fixed RMS.
    pub audio: Vec<f32>,
}

impl CompletedSlot {
    /// A slot from its channel, mode, period index and audio, with
    /// `dial_hz` 0, `start_sample` 0 and no `utc_ns`; assign those fields
    /// when they matter. The receiver makes its own slots; this is for a
    /// consumer that builds one: a test of code that takes a slot, or a
    /// replay of a recorded one (`CompletedSlot` is `#[non_exhaustive]`, so
    /// a struct expression is closed to it, #592).
    pub fn new(channel: ChannelId, mode: Mode, period: i64, audio: Vec<f32>) -> Self {
        CompletedSlot {
            channel,
            mode,
            dial_hz: 0.0,
            audio_rf_scale: 1.0,
            period,
            start_sample: 0,
            utc_ns: None,
            audio,
        }
    }

    /// Whether this is the whole slot rather than a prefix of it. Only a
    /// channel with prefix points delivers prefixes; decode each delivery of
    /// such a channel, this one included, with `decode_prefix`.
    pub fn is_whole(&self) -> bool {
        self.audio.len() >= self.mode.meta().slot_samples_12k as usize
    }

    /// The slot as a decoder takes it.
    pub fn input(&self) -> SlotInput<'_> {
        SlotInput::f32(&self.audio).period(self.period)
    }

    /// RF frequency of an audio frequency of this channel.
    pub fn abs_freq_hz(&self, audio_hz: f32) -> f64 {
        self.dial_hz + 3000.0 + (audio_hz as f64 - 3000.0) * self.audio_rf_scale
    }
}

struct Channel {
    id: ChannelId,
    dial_hz: f64,
    /// What cuts this channel's audio into slots; `None` on a channel that only
    /// keeps its audio ([`IqReceiver::add_audio_channel`]).
    slots: Option<Slots>,
    state: ChannelState,
    /// `Some` on the `Direct` path while active; `None` when the shared bank
    /// feeds it or it is paused.
    fe: Option<IqToAudio>,
    /// Its index in the shared bank, on the `Pfb` path while active.
    bank_idx: usize,
    scratch: Vec<f32>,
    /// The audio index (12 kHz, on the receiver's clock) of the next sample
    /// this channel produces.
    k_next: u64,
    /// The channel's audio since the last [`IqReceiver::take_audio`], when
    /// kept (a waterfall, or a receiver with no slot such as JTTY).
    tap: Option<Tap>,
}

/// A slotted channel's mode and slot cutting.
struct Slots {
    mode: Mode,
    meta: &'static ProtocolMeta,
    cutter: SlotCutter<f32>,
    /// Prefix points are set ([`IqReceiver::set_prefix_points`]).
    prefixes: bool,
    /// The open slot's gain, pinned at its first prefix: its period index
    /// and the gain.
    pin: Option<(i64, f32)>,
    /// The last period index delivered, prefix or whole.
    last_j: Option<i64>,
}

/// A channel's kept audio, as runs of contiguous samples: a retune, a gap or
/// a clock re-anchoring starts a new run, so a caller reading with
/// [`IqReceiver::take_audio_from`] sees where the audio broke.
#[derive(Default)]
struct Tap {
    /// Each run's first audio index and its samples, oldest first.
    runs: alloc::collections::VecDeque<(u64, Vec<f32>)>,
    /// The next samples start a new run even if they follow on: the clock was
    /// re-anchored, so the audio's index still runs on but its UTC jumped.
    broken: bool,
}

impl Tap {
    fn push(&mut self, k: u64, audio: &[f32]) {
        if audio.is_empty() {
            return;
        }
        match self.runs.back_mut() {
            Some((start, run)) if !self.broken && *start + run.len() as u64 == k => {
                run.extend_from_slice(audio)
            }
            _ => self.runs.push_back((k, audio.to_vec())),
        }
        self.broken = false;
    }
}

/// The gain taking `buf` to [`TARGET_RMS`], or `None` for silence or NaN.
fn slot_gain(buf: &[f32]) -> Option<f32> {
    let n = buf.len() as f32;
    let rms = (buf.iter().map(|v| v * v).sum::<f32>() / n).sqrt();
    // Silence, or NaN from a broken input: nothing to decode.
    if rms.is_nan() || rms <= 0.0 {
        return None;
    }
    Some(TARGET_RMS / 32_768.0 / rms)
}

impl Slots {
    fn grid_period_ns(&self) -> i128 {
        (self.meta.t_slot_s * 10.0).round() as i128 * 100_000_000
    }

    /// Cut `audio` (the samples just produced); completed slots go to `out`.
    fn feed(
        &mut self,
        id: ChannelId,
        dial_hz: f64,
        clock: &SampleClock,
        fs: u32,
        audio: &[f32],
        out: &mut Vec<CompletedSlot>,
    ) {
        let anchor = clock.anchor_ns();
        let mode = self.mode;
        let period_ns = self.grid_period_ns();
        let slot = |j: i64, start_k: u64, audio: Vec<f32>| CompletedSlot {
            channel: id,
            mode,
            dial_hz,
            audio_rf_scale: 1.0,
            period: j,
            start_sample: (start_k as u128 * fs as u128 / 12_000) as u64,
            utc_ns: anchor.map(|_| (j as i128 * period_ns) as i64),
            audio,
        };
        let (prefixes, pin, last_j) = (self.prefixes, &mut self.pin, &mut self.last_j);
        // A period this channel already delivered part of, opened again (a
        // clock stepped back): its audio is another recording, and a decoder
        // continuing that period's prefix sequence would splice the two
        // (`IQ_PREFIX_DESIGN.md` §5). Only a channel with prefixes refuses it.
        let reopened = |j: i64, pin: &Option<(i64, f32)>, last_j: &Option<i64>| {
            prefixes && pin.is_none_or(|(p, _)| p != j) && last_j.is_some_and(|l| j <= l)
        };
        // Both callbacks push to `out`; one at a time, through a cell.
        let out = core::cell::RefCell::new(out);
        let state = core::cell::RefCell::new((pin, last_j));
        self.cutter.feed_parts(
            anchor,
            audio,
            |j, start_k, buf| {
                let mut st = state.borrow_mut();
                let (pin, last_j) = &mut *st;
                if reopened(j, pin, last_j) {
                    return;
                }
                // The slot's level is set by its first delivery and kept
                // (`IQ_PREFIX_DESIGN.md` §3).
                let g = match **pin {
                    Some((p, g)) if p == j => g,
                    _ => match slot_gain(buf) {
                        Some(g) => g,
                        None => return,
                    },
                };
                **pin = Some((j, g));
                **last_j = Some(j);
                let audio = buf.iter().map(|&v| v * g).collect();
                out.borrow_mut().push(slot(j, start_k, audio));
            },
            |j, start_k, buf| {
                let mut st = state.borrow_mut();
                let (pin, last_j) = &mut *st;
                if reopened(j, pin, last_j) {
                    return;
                }
                let g = match pin.take() {
                    Some((p, g)) if p == j => Some(g),
                    _ => slot_gain(&buf),
                };
                let Some(g) = g else { return };
                **last_j = Some(j);
                out.borrow_mut()
                    .push(slot(j, start_k, buf.iter().map(|&v| v * g).collect()));
            },
        );
    }
}

impl Channel {
    /// Feed `self.scratch` (the audio just produced): kept when tapped, and
    /// cut into slots on a slotted channel, completed ones going to `out`.
    fn feed(&mut self, clock: &SampleClock, fs: u32, out: &mut Vec<CompletedSlot>) {
        let audio = core::mem::take(&mut self.scratch);
        if let Some(t) = self.tap.as_mut() {
            t.push(self.k_next, &audio);
        }
        if let Some(s) = self.slots.as_mut() {
            s.feed(self.id, self.dial_hz, clock, fs, &audio, out);
        }
        self.k_next += audio.len() as u64;
        self.scratch = audio;
        self.scratch.clear();
    }

    /// Forget the open slot and the continuity: the next slot is found from
    /// the clock again, at audio index `k`.
    fn restart(&mut self, k: u64) {
        self.k_next = k;
        if let Some(s) = self.slots.as_mut() {
            s.cutter.restart(k);
            s.pin = None;
        }
    }

    /// Forget the open slot and the continuity, keeping the position (the
    /// clock was re-anchored): kept audio starts a new run.
    fn forget_slots(&mut self) {
        if let Some(s) = self.slots.as_mut() {
            s.cutter.forget_slots();
            s.pin = None;
        }
        if let Some(t) = self.tap.as_mut() {
            t.broken = true;
        }
    }
}

/// N channels of one IQ stream. See the [module docs](self).
pub struct IqReceiver {
    stream: IqStream,
    channels: Vec<Channel>,
    /// The shared bank, on the [`Channelizer::Pfb`] path.
    bank: Option<PfbChannelizer>,
    /// The bank's audio by its channel index, between the bank and `feed`.
    bank_out: Vec<Vec<f32>>,
    clock: SampleClock,
    samples_in: u64,
    /// Input samples as the nominal rate would count the time they took: `samples_in` when there is no
    /// rate offset, else each push adds `n / (1 + eps)` (#686).
    virt: f64,
    /// The stream's real rate against its nominal one, ppm, and whether one was ever set (a receiver
    /// that never sets one does its arithmetic as it always did).
    ppm: f64,
    offset_set: bool,
    acquisition_ppm: Option<f64>,
    /// Learns the rate offset from the readings of [`Self::set_time`], once asked ([`Self::track_rate`]).
    tracker: Option<RateTracker>,
    /// What the tracker has found and not applied yet, ppm: it is applied on the next slot boundary.
    pending_ppm: Option<f64>,
    /// The 15 s cell of UTC the last reading was in.
    grid_cell: i64,
    /// The grid `grid_cell` counts, ns.
    grid_ns: i64,
    next_id: usize,
    pending: Vec<u8>,
    bi: Vec<f32>,
    bq: Vec<f32>,
}

impl core::fmt::Debug for IqReceiver {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("IqReceiver")
            .field("stream", &self.stream)
            .field("n_channels", &self.channels.len())
            .field("samples_in", &self.samples_in)
            .field("ppm", &self.ppm)
            .finish_non_exhaustive()
    }
}

impl IqReceiver {
    /// A receiver on the [`Channelizer::Direct`] path.
    pub fn new(stream: IqStream) -> Self {
        Self {
            stream,
            channels: Vec::new(),
            bank: None,
            bank_out: Vec::new(),
            clock: SampleClock::new(stream.sample_rate),
            samples_in: 0,
            virt: 0.0,
            ppm: 0.0,
            offset_set: false,
            acquisition_ppm: None,
            tracker: None,
            pending_ppm: None,
            grid_cell: i64::MIN,
            grid_ns: 0,
            next_id: 0,
            pending: Vec::new(),
            bi: Vec::new(),
            bq: Vec::new(),
        }
    }

    /// A receiver on the chosen path. [`Channelizer::Pfb`] fails for a rate
    /// no bank fits (`UnsupportedRate`: every rate under 40 kS/s).
    pub fn with_channelizer(stream: IqStream, kind: Channelizer) -> Result<Self, IqError> {
        let mut rx = Self::new(stream);
        if kind == Channelizer::Pfb {
            rx.bank = Some(PfbChannelizer::new(stream)?);
        }
        Ok(rx)
    }

    /// The path this receiver uses.
    pub fn channelizer(&self) -> Channelizer {
        if self.bank.is_some() {
            Channelizer::Pfb
        } else {
            Channelizer::Direct
        }
    }

    /// Replace the clock's slew and step limits. See
    /// [`SampleClock`].
    pub fn with_clock(mut self, clock: SampleClock) -> Self {
        self.clock = clock;
        self
    }

    /// The largest rate offset taken, ppm: the chain's filters are those of the nominal rate, which is
    /// right to a few per cent: at 5 % the edges of the channel window move 160 Hz
    /// ([`IqToAudio::set_rate_offset_ppm`]).
    pub const MAX_RATE_OFFSET_PPM: f64 = 50_000.0;

    /// The stream's real rate against its nominal one, in ppm: positive when the source delivers more
    /// samples per second than [`IqStream::sample_rate`] says. Every channel's audio is put on a true
    /// 12 kHz grid, [`Self::set_time`] and [`Self::utc_of`] read their sample counts at the real rate,
    /// and a change while running moves the grid continuously (#686). Taken as far as
    /// [`Self::MAX_RATE_OFFSET_PPM`]; choose the nominal rate with [`plannable_rate`](super::plannable_rate).
    pub fn set_rate_offset_ppm(&mut self, ppm: f64) {
        let ppm = ppm.clamp(-Self::MAX_RATE_OFFSET_PPM, Self::MAX_RATE_OFFSET_PPM);
        self.ppm = ppm;
        self.offset_set = true;
        if let Some(bank) = self.bank.as_mut() {
            bank.set_rate_offset_ppm(ppm);
        }
        for c in &mut self.channels {
            if let Some(fe) = c.fe.as_mut() {
                fe.set_rate_offset_ppm(ppm);
            }
        }
    }

    /// The rate offset in force, ppm.
    pub fn rate_offset_ppm(&self) -> f64 {
        self.ppm
    }

    /// Pin the acquisition-clock offset used for frequency translation while
    /// retaining the separate payload rate used for audio and UTC. Deleted
    /// samples can lower the latter without changing the ADC clock. `None`
    /// restores the usual single-clock model. A change drops open slots and
    /// restarts the channel framing, as a retune does.
    pub fn set_acquisition_rate_ppm(&mut self, ppm: Option<f64>) {
        if self.acquisition_ppm == ppm {
            return;
        }
        self.acquisition_ppm = ppm;
        if let Some(bank) = self.bank.as_mut() {
            bank.set_acquisition_rate_ppm(ppm);
        }
        for c in &mut self.channels {
            if let Some(fe) = c.fe.as_mut() {
                fe.set_acquisition_rate_ppm(ppm);
            }
        }
        let sample = self
            .bank
            .as_ref()
            .map_or(self.samples_in, |b| b.next_start_sample());
        let k = self.audio_of_sample(sample);
        self.reset_channels(k);
    }

    /// RF Hz per audio Hz, about the channel's 3000 Hz centre.
    pub fn audio_rf_scale(&self) -> f64 {
        (1.0 + self.acquisition_ppm.unwrap_or(self.ppm) * 1e-6) / self.ratio()
    }

    /// Learn the rate offset from the readings given to [`Self::set_time`] (#686): the stream's anchor
    /// (`utc - samples / Fs`) falls by the offset that is missing, per second, and a least-squares line
    /// through a few minutes of readings finds it; the offset moves by that, a limited step at a time
    /// (`RateTrackConfig::max_step_ppm`, 400 ppm), and the window starts again. `None` stops it. The readings are
    /// the caller's to clean of network delay and its jitter. A reading that steps the clock empties
    /// the window. Starts from the offset in force (0 if none was set) and stays within
    /// [`Self::MAX_RATE_OFFSET_PPM`].
    pub fn track_rate(&mut self, config: Option<RateTrackConfig>) {
        self.tracker = config.map(RateTracker::new);
    }

    /// `1 + eps`.
    fn ratio(&self) -> f64 {
        1.0 + self.ppm * 1e-6
    }

    /// The nominal-rate sample count that stands for input sample `t` on the true time axis: `t` itself
    /// when no offset was set.
    fn sample_equiv(&self, t: u64) -> u64 {
        if !self.offset_set {
            return t;
        }
        let v = self.virt + (t as f64 - self.samples_in as f64) / self.ratio();
        v.round().max(0.0) as u64
    }

    /// Nanoseconds the stream had run when input sample `at_sample` came, on the receiver's time axis:
    /// `at_sample / Fs` without a rate offset, and with one the real elapsed time, rate changes included
    /// (#686). The UTC of that sample is the stream's start plus this, so a caller that estimates the start
    /// from arrival times (`arrival - elapsed_ns`) needs no rate of its own.
    pub fn elapsed_ns(&self, at_sample: u64) -> i64 {
        (self.sample_equiv(at_sample) as i128 * 1_000_000_000 / self.stream.sample_rate as i128)
            as i64
    }

    /// Audio index (12 kHz, true time) of input sample `t`.
    fn audio_of_sample(&self, t: u64) -> u64 {
        (self.sample_equiv(t) as u128 * 12_000 / self.stream.sample_rate as u128) as u64
    }

    /// Audio index of the stream's current position.
    fn audio_index_now(&self) -> u64 {
        self.audio_of_sample(self.samples_in)
    }

    /// A front end for `dial_hz` in the stream, with the rate offset in force.
    fn front_end(&self, dial_hz: f64) -> Result<IqToAudio, IqError> {
        let mut fe = IqToAudio::new(self.stream, dial_hz)?;
        if self.ppm != 0.0 {
            fe.set_rate_offset_ppm(self.ppm);
        }
        fe.set_acquisition_rate_ppm(self.acquisition_ppm);
        Ok(fe)
    }

    /// Add a channel whose dial (audio 0 Hz) is `dial_hz`. Errors as
    /// [`IqToAudio::new`]: too close to DC, or a window outside `±Fs/2`.
    /// Added mid-stream, it starts with the next sample (on the `Pfb` path,
    /// the next sub-band sample that falls on a whole audio sample).
    pub fn add_channel(&mut self, dial_hz: f64, mode: Mode) -> Result<ChannelId, IqError> {
        let (fe, bank_idx, k_next) = self.place(dial_hz)?;
        let id = ChannelId(self.next_id);
        self.next_id += 1;
        let meta = mode.meta();
        self.channels.push(Channel {
            id,
            dial_hz,
            slots: Some(Slots {
                mode,
                meta,
                cutter: SlotCutter::new(
                    SlotGrid::new((meta.t_slot_s * 10.0).round() as i64 * 100_000_000, 12_000),
                    k_next,
                ),
                prefixes: false,
                pin: None,
                last_j: None,
            }),
            state: ChannelState::Active,
            fe,
            bank_idx,
            scratch: Vec::new(),
            k_next,
            tap: None,
        });
        Ok(id)
    }

    /// Add a channel that cuts no slots and keeps its continuous 12 kHz audio
    /// for [`Self::take_audio_from`]: for a receiver with no slot of its own,
    /// JTTY's (`jtty::rx::Stream`), whose frames start at any moment. Placed
    /// as [`Self::add_channel`] places a slotted one, with the same errors; a
    /// retune pauses and resumes it the same way. Its audio is unscaled, as
    /// [`Self::take_audio`]'s is. Keep draining it: it grows until taken.
    pub fn add_audio_channel(&mut self, dial_hz: f64) -> Result<ChannelId, IqError> {
        let (fe, bank_idx, k_next) = self.place(dial_hz)?;
        let id = ChannelId(self.next_id);
        self.next_id += 1;
        self.channels.push(Channel {
            id,
            dial_hz,
            slots: None,
            state: ChannelState::Active,
            fe,
            bank_idx,
            scratch: Vec::new(),
            k_next,
            tap: Some(Tap::default()),
        });
        Ok(id)
    }

    /// Deliver this channel's slots also as prefixes of these lengths, in
    /// 12 kHz samples: each slot that opens from now on goes out through
    /// `push_*` first cut at each point, then whole. Pass the channel's
    /// decoder's [`AnyDecoder::prefix_points`](crate::decoder::AnyDecoder::prefix_points)
    /// and decode **every** delivery of the channel with `decode_prefix`, the
    /// whole slot included. Empty (the default) delivers whole slots only.
    /// `false` if the channel is unknown.
    ///
    /// On a channel with points, a slot's level is measured on its first
    /// prefix and kept for its later prefixes and its whole, so a decoder sees
    /// one period at one level; and a period index the channel already
    /// delivered part of is not delivered again (a clock stepped back). A slot
    /// dropped by a retune, a gap or a clock step keeps the prefixes it already
    /// delivered and has no whole. Design: `docs/notes/IQ_PREFIX_DESIGN.md`.
    pub fn set_prefix_points(&mut self, id: ChannelId, points: &[usize]) -> bool {
        match self
            .channels
            .iter_mut()
            .find(|c| c.id == id)
            .and_then(|c| c.slots.as_mut())
        {
            Some(s) => {
                s.cutter.set_points(points);
                s.prefixes = s.cutter.has_points();
                true
            }
            None => false,
        }
    }

    /// Place a dial in the current stream: its front end (`Direct`) or its
    /// bank index (`Pfb`), and the audio index it starts at.
    fn place(&mut self, dial_hz: f64) -> Result<(Option<IqToAudio>, usize, u64), IqError> {
        Ok(match self.bank.as_mut() {
            Some(bank) => {
                let t = bank.next_start_sample();
                let idx = bank.add_channel(dial_hz)?;
                let k = self.audio_of_sample(t);
                if self.bank_out.len() <= idx {
                    self.bank_out.resize_with(idx + 1, Vec::new);
                }
                (None, idx, k)
            }
            None => (Some(self.front_end(dial_hz)?), 0, self.audio_index_now()),
        })
    }

    /// Start or stop keeping a channel's continuous 12 kHz audio for
    /// [`Self::take_audio`] (a waterfall, a level meter). Off by default on a
    /// slotted channel, on from the start on an audio channel
    /// ([`Self::add_audio_channel`]): a tapped channel that is never drained
    /// grows without bound.
    pub fn tap_audio(&mut self, id: ChannelId, on: bool) -> bool {
        match self.channels.iter_mut().find(|c| c.id == id) {
            Some(c) => {
                c.tap = on.then(Tap::default);
                true
            }
            None => false,
        }
    }

    /// Move the audio a tapped channel produced since the last call onto the end
    /// of `out`, unscaled (the slots handed to a decoder are normalised, this
    /// is the level that came out of the channel filter), across any break in
    /// it (a waterfall does not mind one). A paused channel produces none.
    /// `false` if the channel is unknown or not tapped.
    pub fn take_audio(&mut self, id: ChannelId, out: &mut Vec<f32>) -> bool {
        match self
            .channels
            .iter_mut()
            .find(|c| c.id == id)
            .and_then(|c| c.tap.as_mut())
        {
            Some(t) => {
                for (_, mut run) in t.runs.drain(..) {
                    out.append(&mut run);
                }
                true
            }
            None => false,
        }
    }

    /// As [`Self::take_audio`], up to the first break in the audio: a retune,
    /// a gap, or the clock re-anchored. Returns the audio index (12 kHz, as
    /// [`Self::utc_of_audio`] takes it) of the first sample appended, or `None`
    /// when nothing is waiting or the channel is unknown or not tapped. A
    /// caller that keeps state across samples (JTTY's receiver) calls it until
    /// `None`, and treats an index other than the one it expected next as a
    /// break: the audio before and after it are not one recording.
    pub fn take_audio_from(&mut self, id: ChannelId, out: &mut Vec<f32>) -> Option<u64> {
        let (k, mut run) = self
            .channels
            .iter_mut()
            .find(|c| c.id == id)?
            .tap
            .as_mut()?
            .runs
            .pop_front()?;
        out.append(&mut run);
        Some(k)
    }

    /// The UTC (ns since the Unix epoch) of audio index `k` on the receiver's
    /// 12 kHz clock, once a clock is set ([`Self::set_time`]).
    pub fn utc_of_audio(&self, k: u64) -> Option<i64> {
        self.clock
            .anchor_ns()
            .map(|a| a + (k as i128 * 1_000_000_000 / 12_000) as i64)
    }

    /// Remove a channel; `false` if it was not there.
    pub fn remove_channel(&mut self, id: ChannelId) -> bool {
        let Some(at) = self.channels.iter().position(|c| c.id == id) else {
            return false;
        };
        let c = self.channels.remove(at);
        if c.state == ChannelState::Active
            && let Some(bank) = self.bank.as_mut()
        {
            bank.remove_channel(c.bank_idx);
        }
        true
    }

    /// Whether a channel is being received.
    pub fn channel_state(&self, id: ChannelId) -> Option<ChannelState> {
        self.channels.iter().find(|c| c.id == id).map(|c| c.state)
    }

    /// The stream's sample `at_sample` (in input samples, as
    /// [`Self::samples_in`] counts) was at UTC `utc_ns` (ns since the Unix
    /// epoch). Call it as often as the caller has a reading; the receiver
    /// follows the readings at a bounded rate, so noisy readings move the
    /// grid by milliseconds and lose nothing.
    pub fn set_time(&mut self, utc_ns: i64, at_sample: u64) -> ClockChange {
        let equiv = self.sample_equiv(at_sample);
        let change = self.clock.observe(utc_ns, equiv);
        if let Some(tracker) = self.tracker.as_mut() {
            if !matches!(change, ClockChange::Slewed { .. }) {
                tracker.reset();
            }
            let x = equiv as f64 / self.stream.sample_rate as f64;
            if let Some(step) = tracker.observe(x, utc_ns as f64 * 1e-9 - x) {
                self.pending_ppm = Some(self.pending_ppm.unwrap_or(0.0) + step);
            }
        }
        // A rate offset moves every tone by its distance from the IQ centre (4 Hz for 150 ppm at 26 kHz): inside
        // a slot that is a drift nothing decodes (an RSP1 behind `rsp_tcp`, 2026-10-12). What the tracker found
        // is applied when the clock crosses a boundary of the 15 s grid, a whole number of FT8 and FT4 slots.
        let grid = self.rate_grid_ns();
        let cell = utc_ns.div_euclid(grid);
        // A channel added or removed changes the grid; the cell number then means something else and is
        // not a boundary crossed.
        let regridded = grid != self.grid_ns;
        self.grid_ns = grid;
        if cell != self.grid_cell {
            self.grid_cell = cell;
            if regridded {
                // wait for the next boundary of the new grid
            } else if let Some(step) = self.pending_ppm.take() {
                self.set_rate_offset_ppm(self.ppm + step);
                // The readings in the window were made against the offset just replaced.
                if let Some(tracker) = self.tracker.as_mut() {
                    tracker.reset();
                }
            }
        }
        if matches!(change, ClockChange::First | ClockChange::Stepped { .. }) {
            // The grid jumped: audio spanning the jump is not a slot.
            for c in &mut self.channels {
                c.forget_slots();
            }
        }
        change
    }

    /// The grid a learned rate offset is applied on, ns: a boundary of every active slotted channel's
    /// period, so a change never falls inside a slot of any of them. 15 s with FT8 and FT4 only; a 60 s
    /// or 120 s mode (JT65, WSPR) raises it to its own period (the least common multiple, in tenths of a
    /// second), where the 15 s grid fell inside its transmissions (#686). JTTY has no slot and does not
    /// count: its tones shift by the offset change wherever it is applied.
    fn rate_grid_ns(&self) -> i64 {
        let tenths = |ns: i64| ns / 100_000_000;
        let gcd = |mut a: i64, mut b: i64| {
            while b != 0 {
                (a, b) = (b, a % b);
            }
            a
        };
        let mut grid = tenths(SLOT_GRID_NS);
        for c in &self.channels {
            if let (ChannelState::Active, Some(s)) = (c.state, c.slots.as_ref()) {
                let p = tenths(s.grid_period_ns() as i64);
                if p > 0 {
                    grid = grid / gcd(grid, p) * p;
                }
            }
        }
        grid * 100_000_000
    }

    /// Like [`Self::set_time`], but the clock is set to the reading at once instead of slewing to it:
    /// for a caller who knows the clock was wrong, not drifting (the first fix of a stream, a server
    /// that paused, a rate that changed). Open slots are dropped, as for any step.
    pub fn step_time(&mut self, utc_ns: i64, at_sample: u64) -> ClockChange {
        let change = self.clock.step_to(utc_ns, self.sample_equiv(at_sample));
        if let Some(tracker) = self.tracker.as_mut() {
            tracker.reset();
        }
        for c in &mut self.channels {
            c.forget_slots();
        }
        change
    }

    /// UTC the clock puts the stream's sample `at_sample` at, once set.
    pub fn utc_of(&self, at_sample: u64) -> Option<i64> {
        self.clock.utc_of(self.sample_equiv(at_sample))
    }

    /// Complex samples consumed, gaps included: the stream's clock.
    pub fn samples_in(&self) -> u64 {
        self.samples_in
    }

    /// Every channel's open slot dropped and its next audio index `k`.
    fn reset_channels(&mut self, k: u64) {
        for c in &mut self.channels {
            c.restart(k);
        }
        for out in &mut self.bank_out {
            out.clear();
        }
        self.pending.clear();
    }

    /// The tuner moved: every channel that still fits `center_hz` is
    /// re-placed, the rest are paused (and a paused one that fits again
    /// resumes), and the open slots of all are dropped. The sample clock
    /// continues.
    pub fn retune(&mut self, center_hz: f64) -> RetuneReport {
        let stream = IqStream {
            center_hz,
            ..self.stream
        };
        let fs = stream.sample_rate as f64;
        let mut report = RetuneReport::default();
        let fits: Vec<Result<(), IqError>> = self
            .channels
            .iter()
            .map(|c| check_placement(fs, c.dial_hz - center_hz))
            .collect();
        if let Some(bank) = self.bank.as_mut() {
            for (c, f) in self.channels.iter_mut().zip(&fits) {
                if f.is_err() && c.state == ChannelState::Active {
                    bank.remove_channel(c.bank_idx);
                }
            }
            bank.retune(center_hz)
                .expect("every remaining channel was checked to fit");
            self.stream = stream;
            for (c, f) in self.channels.iter_mut().zip(&fits) {
                match (*f, c.state) {
                    (Ok(()), ChannelState::Paused(_)) => {
                        let idx = bank.add_channel(c.dial_hz).expect("checked to fit");
                        c.bank_idx = idx;
                        c.state = ChannelState::Active;
                        report.resumed.push(c.id);
                    }
                    (Err(e), ChannelState::Active) => {
                        c.state = ChannelState::Paused(e);
                        report.paused.push(c.id);
                    }
                    (Err(e), ChannelState::Paused(_)) => c.state = ChannelState::Paused(e),
                    (Ok(()), ChannelState::Active) => {}
                }
            }
            let need = self
                .channels
                .iter()
                .map(|c| c.bank_idx + 1)
                .max()
                .unwrap_or(0);
            if self.bank_out.len() < need {
                self.bank_out.resize_with(need, Vec::new);
            }
            let t = self.bank.as_ref().expect("bank").next_start_sample();
            let k = self.audio_of_sample(t);
            self.reset_channels(k);
            return report;
        }
        self.stream = stream;
        let ppm = self.ppm;
        for (c, f) in self.channels.iter_mut().zip(&fits) {
            match *f {
                Ok(()) => {
                    let mut fe = IqToAudio::new(stream, c.dial_hz).expect("checked to fit");
                    if ppm != 0.0 {
                        fe.set_rate_offset_ppm(ppm);
                    }
                    fe.set_acquisition_rate_ppm(self.acquisition_ppm);
                    c.fe = Some(fe);
                    if matches!(c.state, ChannelState::Paused(_)) {
                        report.resumed.push(c.id);
                    }
                    c.state = ChannelState::Active;
                }
                Err(e) => {
                    c.fe = None;
                    if c.state == ChannelState::Active {
                        report.paused.push(c.id);
                    }
                    c.state = ChannelState::Paused(e);
                }
            }
        }
        let k = self.audio_index_now();
        self.reset_channels(k);
        report
    }

    /// `lost` samples never arrived: the clock advances past them and the
    /// open slots are dropped, since audio that spans the hole is not a slot.
    pub fn gap(&mut self, lost: u64) {
        self.samples_in += lost;
        self.virt += lost as f64 / self.ratio();
        if let Some(bank) = self.bank.as_mut() {
            bank.gap(lost);
            let t = bank.next_start_sample();
            let k = self.audio_of_sample(t);
            self.reset_channels(k);
            return;
        }
        let (stream, ppm) = (self.stream, self.ppm);
        for c in &mut self.channels {
            if c.state == ChannelState::Active {
                let mut fe = IqToAudio::new(stream, c.dial_hz).expect("placed before");
                if ppm != 0.0 {
                    fe.set_rate_offset_ppm(ppm);
                }
                fe.set_acquisition_rate_ppm(self.acquisition_ppm);
                c.fe = Some(fe);
            }
        }
        let k = self.audio_index_now();
        self.reset_channels(k);
    }

    fn run_block(&mut self, n: usize, out: &mut Vec<CompletedSlot>) {
        self.samples_in += n as u64;
        self.virt += n as f64 / self.ratio();
        if self.stream.iq_swap {
            core::mem::swap(&mut self.bi, &mut self.bq);
        }
        let fs = self.stream.sample_rate;
        if let Some(bank) = self.bank.as_mut() {
            bank.push_planar(&self.bi, &self.bq, &mut self.bank_out);
        }
        for c in &mut self.channels {
            if c.state != ChannelState::Active {
                continue;
            }
            match c.fe.as_mut() {
                Some(fe) => fe.push_planar(&self.bi, &self.bq, &mut c.scratch),
                None => core::mem::swap(&mut c.scratch, &mut self.bank_out[c.bank_idx]),
            }
            let first = out.len();
            c.feed(&self.clock, fs, out);
            let scale =
                (1.0 + self.acquisition_ppm.unwrap_or(self.ppm) * 1e-6) / (1.0 + self.ppm * 1e-6);
            for slot in &mut out[first..] {
                slot.audio_rf_scale = scale;
            }
        }
    }

    /// Push `f32` I/Q, interleaved. Slots that complete are appended to
    /// `out`.
    pub fn push_cf32(&mut self, iq: &[f32], out: &mut Vec<CompletedSlot>) {
        for chunk in iq.chunks(2 * BLOCK) {
            self.bi.clear();
            self.bq.clear();
            for &[i, q] in chunk.as_chunks::<2>().0 {
                self.bi.push(i);
                self.bq.push(q);
            }
            self.run_block(chunk.len() / 2, out);
        }
    }

    /// Push `i16` I/Q, interleaved, full scale 32768.
    pub fn push_cs16(&mut self, iq: &[i16], out: &mut Vec<CompletedSlot>) {
        const S: f32 = 1.0 / 32_768.0;
        for chunk in iq.chunks(2 * BLOCK) {
            self.bi.clear();
            self.bq.clear();
            for &[i, q] in chunk.as_chunks::<2>().0 {
                self.bi.push(i as f32 * S);
                self.bq.push(q as f32 * S);
            }
            self.run_block(chunk.len() / 2, out);
        }
    }

    /// Push a byte stream in [`IqStream::format`], little-endian; a sample
    /// split across calls is carried over.
    pub fn push_bytes(&mut self, bytes: &[u8], out: &mut Vec<CompletedSlot>) {
        let w = self.stream.format.bytes_per_sample();
        self.pending.extend_from_slice(bytes);
        let usable = self.pending.len() / w * w;
        let taken: Vec<u8> = self.pending.drain(..usable).collect();
        for chunk in taken.chunks(w * BLOCK) {
            self.bi.clear();
            self.bq.clear();
            self.stream
                .format
                .convert(chunk, &mut self.bi, &mut self.bq);
            self.run_block(chunk.len() / w, out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A receiver moves to a worker thread, and so do its slots.
    #[test]
    fn receiver_slots_and_channelizers_are_send() {
        fn assert_send<T: Send>() {}
        assert_send::<IqReceiver>();
        assert_send::<CompletedSlot>();
        assert_send::<crate::iq::PfbChannelizer>();
        assert_send::<IqToAudio>();
    }

    /// The app-facing types print without dumping audio (`dbg!` on a decoder
    /// or a receiver must stay readable), and the errors are errors.
    #[cfg(feature = "ft8")]
    #[test]
    fn app_facing_types_are_debug() {
        fn assert_debug<T: core::fmt::Debug>() {}
        assert_debug::<crate::decoder::Decoder<crate::Ft8>>();
        assert_debug::<crate::decoder::AnyDecoder>();
        assert_debug::<crate::decoder::SlotInput<'static>>();
        assert_debug::<crate::decoder::AnyExtras<'static>>();
        assert_debug::<crate::decoder::DecodeParams>();
        assert_debug::<IqReceiver>();
        assert_debug::<CompletedSlot>();

        let audio = alloc::vec![0i16; 180_000];
        let slot = SlotInput::new(crate::decoder::Audio::I16(&audio));
        assert!(alloc::format!("{slot:?}").len() < 400);
    }

    #[test]
    fn errors_are_errors() {
        fn assert_error<T: core::error::Error>() {}
        assert_error::<IqError>();
        assert_error::<crate::decoder::Unsupported>();
    }

    #[cfg(feature = "q65")]
    #[test]
    fn q65_error_is_error() {
        fn assert_error<T: core::error::Error>() {}
        assert_error::<crate::fec::qra::Q65DecodeError>();
    }

    #[cfg(feature = "uvpacket")]
    #[test]
    fn uvpacket_errors_are_errors() {
        fn assert_error<T: core::error::Error>() {}
        assert_error::<crate::uvpacket::framing::PackError>();
        assert_error::<crate::uvpacket::framing::UnpackError>();
        assert_error::<crate::uvpacket::rx::DecodeError>();
    }

    /// Every mode this build has is a registry entry, with the slot the
    /// mode's period says.
    #[test]
    fn every_mode_has_a_whole_slot() {
        for &m in Mode::ALL {
            let meta = m.meta();
            assert_eq!(
                meta.slot_samples_12k,
                (meta.t_slot_s * 12_000.0) as u32,
                "{m:?}"
            );
            assert_eq!((meta.t_slot_s * 10.0).fract(), 0.0, "{m:?}: whole tenths");
        }
    }

    /// The grid a rate offset is applied on follows the active channels: 15 s with FT8 alone, 120 s
    /// once a WSPR channel is in, and back when it is removed.
    #[cfg(all(feature = "ft8", feature = "wspr"))]
    #[test]
    fn the_rate_grid_is_a_boundary_of_every_active_channel() {
        let mut rx = IqReceiver::new(IqStream::new(
            96_000,
            14_085_000.0,
            crate::iq::IqSampleFormat::Cf32,
        ));
        assert_eq!(rx.rate_grid_ns(), 15_000_000_000);
        let ft8 = rx.add_channel(14_070_000.0, Mode::Ft8).unwrap();
        assert_eq!(rx.rate_grid_ns(), 15_000_000_000);
        let wspr = rx.add_channel(14_090_000.0, Mode::Wspr).unwrap();
        assert_eq!(rx.rate_grid_ns(), 120_000_000_000);
        assert!(rx.remove_channel(wspr));
        assert_eq!(rx.rate_grid_ns(), 15_000_000_000);
        assert!(rx.remove_channel(ft8));
    }
}
