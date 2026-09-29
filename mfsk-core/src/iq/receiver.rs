// SPDX-License-Identifier: GPL-3.0-or-later
//! [`IqReceiver`]: N channels of one wideband IQ stream, slots cut on UTC
//! from the sample count, each decoded with its mode's own request (#534,
//! phase 2).
//!
//! What it does *not* do is find anything: the caller says which dial
//! frequency carries which mode ([`IqReceiver::add_channel`]) and what UTC
//! sample 0 fell on ([`IqReceiver::set_time_anchor`]). The decoders search
//! the channel's audio 200-3000 Hz themselves, so a dial only has to place
//! that window.
//!
//! ## Time
//!
//! The sample count is the clock. Channel audio index `k` is time
//! `k / 12000` s after sample 0 (each channel's front end drops its own
//! group delay, so index 0 is the audio of IQ sample 0), and slot `j` of a
//! mode with period `T` covers UTC `[j·T, (j+1)·T)`. With no anchor the
//! grid free-runs from sample 0 — right for replaying a recording. A slot
//! is decoded once all of it has arrived; the partial one the stream opened
//! in the middle of is not.
//!
//! [`IqReceiver::retune`], [`IqReceiver::gap`] and re-anchoring drop every
//! open slot: audio that straddles a change of centre, a hole in the
//! samples or a moved grid is not a slot. The sample clock continues.
//!
//! ## Decoding happens inside `push`
//!
//! A slot that completes is decoded before the `push_*` call returns and its
//! rows are delivered through the callback; on a busy FT8 band that is
//! hundreds of milliseconds. A caller that cannot block should push from a
//! worker thread. Each slot is scaled to a fixed RMS before the `i16`
//! conversion the decoders take (they are scale-free; the IQ's own level is
//! not something to inherit).

use alloc::boxed::Box;
use alloc::vec::Vec;

use super::{IqError, IqSampleFormat, IqStream, IqToAudio};
use crate::engine::pipeline::DecodeResult;
use crate::msg::decode_request::{DecodeRequest, FrameDecodable};
use crate::msg::decoded::Decoded;
use crate::registry::{self, ProtocolMeta};

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

/// Interleaved I/Q converted per pass.
const BLOCK: usize = 8_192;
/// RMS the slot is scaled to before `i16` conversion.
const TARGET_RMS: f32 = 2_000.0;
const NS: i128 = 1_000_000_000;

/// The modes an [`IqReceiver`] channel can carry: the ones that share
/// [`DecodeRequest`] (FT8, FT4, FST4). WSPR, JT9/JT65 and Q65 have their own
/// request types and follow.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IqMode {
    #[cfg(feature = "ft8")]
    Ft8,
    #[cfg(feature = "ft4")]
    Ft4,
    #[cfg(feature = "fst4")]
    Fst4S15,
    #[cfg(feature = "fst4")]
    Fst4S30,
    #[cfg(feature = "fst4")]
    Fst4S60,
    #[cfg(feature = "fst4")]
    Fst4S120,
    #[cfg(feature = "fst4")]
    Fst4S300,
}

impl IqMode {
    fn registry_name(self) -> &'static str {
        match self {
            #[cfg(feature = "ft8")]
            IqMode::Ft8 => "FT8",
            #[cfg(feature = "ft4")]
            IqMode::Ft4 => "FT4",
            #[cfg(feature = "fst4")]
            IqMode::Fst4S15 => "FST4-15",
            #[cfg(feature = "fst4")]
            IqMode::Fst4S30 => "FST4-30",
            #[cfg(feature = "fst4")]
            IqMode::Fst4S60 => "FST4-60A",
            #[cfg(feature = "fst4")]
            IqMode::Fst4S120 => "FST4-120",
            #[cfg(feature = "fst4")]
            IqMode::Fst4S300 => "FST4-300",
        }
    }

    fn meta(self) -> &'static ProtocolMeta {
        registry::by_name(self.registry_name()).expect("every IqMode variant is a registry entry")
    }

    /// `(freq_min, freq_max, sync_min, max_cand)` from the registry, then the
    /// mode's own `DecodeRequest`.
    fn decode(self, audio: &[i16]) -> Vec<DecodeResult> {
        match self {
            #[cfg(feature = "ft8")]
            IqMode::Ft8 => decode_with::<crate::Ft8>(audio, self.meta()),
            #[cfg(feature = "ft4")]
            IqMode::Ft4 => decode_with::<crate::Ft4>(audio, self.meta()),
            #[cfg(feature = "fst4")]
            IqMode::Fst4S15 => decode_with::<crate::fst4::Fst4s15>(audio, self.meta()),
            #[cfg(feature = "fst4")]
            IqMode::Fst4S30 => decode_with::<crate::fst4::Fst4s30>(audio, self.meta()),
            #[cfg(feature = "fst4")]
            IqMode::Fst4S60 => decode_with::<crate::fst4::Fst4s60>(audio, self.meta()),
            #[cfg(feature = "fst4")]
            IqMode::Fst4S120 => decode_with::<crate::fst4::Fst4s120>(audio, self.meta()),
            #[cfg(feature = "fst4")]
            IqMode::Fst4S300 => decode_with::<crate::fst4::Fst4s300>(audio, self.meta()),
        }
    }
}

fn decode_with<P: FrameDecodable<DecodeResult = DecodeResult>>(
    audio: &[i16],
    meta: &ProtocolMeta,
) -> Vec<DecodeResult> {
    let d = meta.profile.defaults;
    DecodeRequest::<P>::new(
        audio,
        d.freq_min_hz,
        d.freq_max_hz,
        d.sync_min,
        d.max_cand as usize,
    )
    .decode()
    .results
}

/// The decode callback [`IqReceiver::on_decode`] stores.
type DecodeCallback = Box<dyn FnMut(&IqDecode) + Send>;

/// Handle of a channel added with [`IqReceiver::add_channel`]; not reused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ChannelId(pub usize);

/// One decode, with where it came from.
#[derive(Clone, Debug)]
pub struct IqDecode {
    pub channel: ChannelId,
    pub mode: IqMode,
    /// The cross-mode row: text, audio frequency, DT, SNR.
    pub decoded: Decoded,
    /// RF frequency of tone 0: the channel's dial plus the audio frequency.
    pub abs_freq_hz: f64,
    /// IQ sample index the slot started at.
    pub slot_start_sample: u64,
    /// UTC of the slot start in ns, when a time anchor is set.
    pub slot_start_utc_ns: Option<i64>,
}

struct OpenSlot {
    buf: Vec<f32>,
    start_k: u64,
    start_utc_ns: Option<i64>,
}

struct Channel {
    id: ChannelId,
    dial_hz: f64,
    mode: IqMode,
    meta: &'static ProtocolMeta,
    fe: IqToAudio,
    /// Audio index of the next sample this channel will emit.
    k_next: u64,
    slot: Option<OpenSlot>,
    scratch: Vec<f32>,
}

fn ceil_div(a: i128, b: i128) -> i128 {
    a.div_euclid(b) + i128::from(a.rem_euclid(b) != 0)
}

impl Channel {
    /// Slot period in ns (`t_slot_s` is a multiple of 0.1 s for every mode).
    fn period_ns(&self) -> i128 {
        (self.meta.t_slot_s * 10.0).round() as i128 * 100_000_000
    }

    /// First slot start at or after audio index `k`: `(slot number, start
    /// index)`. Exact in integers: `k` is at UTC `anchor + k/12000`.
    fn next_boundary(&self, k: u64, anchor_ns: i64) -> (i128, u64) {
        let (p, a) = (self.period_ns(), anchor_ns as i128);
        let j = ceil_div(a * 12_000 + k as i128 * NS, p * 12_000);
        let start = ceil_div(j * p * 12_000 - a * 12_000, NS);
        (j, start as u64)
    }

    /// Feed `self.scratch` (the audio just produced); completed slots go to
    /// `rows`.
    fn feed(&mut self, anchor: Option<i64>, fs: u32, rows: &mut Vec<IqDecode>) {
        let audio = core::mem::take(&mut self.scratch);
        let slot_len = self.meta.slot_samples_12k as usize;
        let (mut pos, mut k) = (0usize, self.k_next);
        let end_k = k + audio.len() as u64;
        while pos < audio.len() {
            match self.slot.as_mut() {
                None => {
                    let (j, start_k) = self.next_boundary(k, anchor.unwrap_or(0));
                    if start_k >= end_k {
                        break;
                    }
                    pos += (start_k - k) as usize;
                    k = start_k;
                    self.slot = Some(OpenSlot {
                        buf: Vec::with_capacity(slot_len),
                        start_k,
                        start_utc_ns: anchor.map(|_| (j * self.period_ns()) as i64),
                    });
                }
                Some(s) => {
                    let take = (slot_len - s.buf.len()).min(audio.len() - pos);
                    s.buf.extend_from_slice(&audio[pos..pos + take]);
                    pos += take;
                    k += take as u64;
                    if s.buf.len() == slot_len {
                        let done = self.slot.take().expect("just matched");
                        self.decode_slot(done, fs, rows);
                    }
                }
            }
        }
        self.k_next = end_k;
        self.scratch = audio;
        self.scratch.clear();
    }

    fn decode_slot(&self, slot: OpenSlot, fs: u32, rows: &mut Vec<IqDecode>) {
        let n = slot.buf.len() as f32;
        let rms = (slot.buf.iter().map(|v| v * v).sum::<f32>() / n).sqrt();
        // Silence, or NaN from a broken input: nothing to decode.
        if rms.is_nan() || rms <= 0.0 {
            return;
        }
        let g = TARGET_RMS / rms;
        let audio: Vec<i16> = slot
            .buf
            .iter()
            .map(|&v| (v * g).round().clamp(-32_768.0, 32_767.0) as i16)
            .collect();
        let start_sample = (slot.start_k as u128 * fs as u128 / 12_000) as u64;
        for r in self.mode.decode(&audio) {
            if let Some(decoded) = r.to_decoded(self.meta.id, None) {
                rows.push(IqDecode {
                    channel: self.id,
                    mode: self.mode,
                    abs_freq_hz: self.dial_hz + decoded.freq_hz as f64,
                    decoded,
                    slot_start_sample: start_sample,
                    slot_start_utc_ns: slot.start_utc_ns,
                });
            }
        }
    }
}

/// N channels of one IQ stream. See the [module docs](self).
pub struct IqReceiver {
    stream: IqStream,
    channels: Vec<Channel>,
    anchor_ns: Option<i64>,
    samples_in: u64,
    next_id: usize,
    on_decode: Option<DecodeCallback>,
    pending: Vec<u8>,
    bi: Vec<f32>,
    bq: Vec<f32>,
}

impl IqReceiver {
    pub fn new(stream: IqStream) -> Self {
        Self {
            stream,
            channels: Vec::new(),
            anchor_ns: None,
            samples_in: 0,
            next_id: 0,
            on_decode: None,
            pending: Vec::new(),
            bi: Vec::new(),
            bq: Vec::new(),
        }
    }

    /// Audio index of the stream's current position.
    fn audio_index_now(&self) -> u64 {
        (self.samples_in as u128 * 12_000 / self.stream.sample_rate as u128) as u64
    }

    /// Add a channel whose dial (audio 0 Hz) is `dial_hz`. Errors as
    /// [`IqToAudio::new`]: too close to DC, or a window outside `±Fs/2`.
    /// Added mid-stream, it starts with the next sample.
    pub fn add_channel(&mut self, dial_hz: f64, mode: IqMode) -> Result<ChannelId, IqError> {
        let fe = IqToAudio::new(self.stream, dial_hz)?;
        let id = ChannelId(self.next_id);
        self.next_id += 1;
        self.channels.push(Channel {
            id,
            dial_hz,
            mode,
            meta: mode.meta(),
            fe,
            k_next: self.audio_index_now(),
            slot: None,
            scratch: Vec::new(),
        });
        Ok(id)
    }

    /// Remove a channel; `false` if it was not there.
    pub fn remove_channel(&mut self, id: ChannelId) -> bool {
        let n = self.channels.len();
        self.channels.retain(|c| c.id != id);
        self.channels.len() != n
    }

    /// Set what UTC (ns since the Unix epoch) IQ sample 0 fell on. Slot
    /// boundaries move with it, so every open slot is dropped.
    pub fn set_time_anchor(&mut self, utc_ns_at_sample_0: i64) {
        self.anchor_ns = Some(utc_ns_at_sample_0);
        self.drop_open_slots();
    }

    /// Where every decode is delivered.
    pub fn on_decode(&mut self, cb: impl FnMut(&IqDecode) + Send + 'static) {
        self.on_decode = Some(Box::new(cb));
    }

    /// Complex samples consumed, gaps included: the stream's clock.
    pub fn samples_in(&self) -> u64 {
        self.samples_in
    }

    fn drop_open_slots(&mut self) {
        for c in &mut self.channels {
            c.slot = None;
        }
    }

    /// Fresh front ends, no open slots, audio index re-derived from the
    /// clock. All-or-nothing: `Err` leaves everything as it was.
    fn rebuild(&mut self, stream: IqStream) -> Result<(), IqError> {
        let fes = self
            .channels
            .iter()
            .map(|c| IqToAudio::new(stream, c.dial_hz))
            .collect::<Result<Vec<_>, _>>()?;
        self.stream = stream;
        let k = self.audio_index_now();
        for (c, fe) in self.channels.iter_mut().zip(fes) {
            c.fe = fe;
            c.slot = None;
            c.k_next = k;
        }
        self.pending.clear();
        Ok(())
    }

    /// The tuner moved: every channel is re-placed against `center_hz`
    /// (`Err`, and nothing changes, if one no longer fits) and the open
    /// slots are dropped. The sample clock continues.
    pub fn retune(&mut self, center_hz: f64) -> Result<(), IqError> {
        self.rebuild(IqStream {
            center_hz,
            ..self.stream
        })
    }

    /// `lost` samples never arrived: the clock advances past them and the
    /// open slots are dropped, since audio that spans the hole is not a slot.
    pub fn gap(&mut self, lost: u64) {
        self.samples_in += lost;
        let stream = self.stream;
        self.rebuild(stream)
            .expect("the same stream and dials built before");
    }

    fn run_block(&mut self, n: usize, rows: &mut Vec<IqDecode>) {
        self.samples_in += n as u64;
        if self.stream.iq_swap {
            core::mem::swap(&mut self.bi, &mut self.bq);
        }
        let (anchor, fs) = (self.anchor_ns, self.stream.sample_rate);
        for c in &mut self.channels {
            c.fe.push_planar(&self.bi, &self.bq, &mut c.scratch);
            c.feed(anchor, fs, rows);
        }
    }

    fn deliver(&mut self, rows: Vec<IqDecode>) {
        if let Some(cb) = self.on_decode.as_mut() {
            for r in &rows {
                cb(r);
            }
        }
    }

    /// Push `f32` I/Q, interleaved.
    pub fn push_cf32(&mut self, iq: &[f32]) {
        let mut rows = Vec::new();
        for chunk in iq.chunks(2 * BLOCK) {
            self.bi.clear();
            self.bq.clear();
            for &[i, q] in chunk.as_chunks::<2>().0 {
                self.bi.push(i);
                self.bq.push(q);
            }
            self.run_block(chunk.len() / 2, &mut rows);
        }
        self.deliver(rows);
    }

    /// Push `i16` I/Q, interleaved, full scale 32768.
    pub fn push_cs16(&mut self, iq: &[i16]) {
        const S: f32 = 1.0 / 32_768.0;
        let mut rows = Vec::new();
        for chunk in iq.chunks(2 * BLOCK) {
            self.bi.clear();
            self.bq.clear();
            for &[i, q] in chunk.as_chunks::<2>().0 {
                self.bi.push(i as f32 * S);
                self.bq.push(q as f32 * S);
            }
            self.run_block(chunk.len() / 2, &mut rows);
        }
        self.deliver(rows);
    }

    /// Push a byte stream in [`IqStream::format`], little-endian; a sample
    /// split across calls is carried over.
    pub fn push_bytes(&mut self, bytes: &[u8]) {
        let w = self.stream.format.bytes_per_sample();
        self.pending.extend_from_slice(bytes);
        let usable = self.pending.len() / w * w;
        let taken: Vec<u8> = self.pending.drain(..usable).collect();
        match self.stream.format {
            IqSampleFormat::Cf32 => {
                let v: Vec<f32> = taken
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|&b| f32::from_le_bytes(b))
                    .collect();
                self.push_cf32(&v);
            }
            IqSampleFormat::Cs16 => {
                let v: Vec<i16> = taken
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .map(|&b| i16::from_le_bytes(b))
                    .collect();
                self.push_cs16(&v);
            }
        }
    }
}
