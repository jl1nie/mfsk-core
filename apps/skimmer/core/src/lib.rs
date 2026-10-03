// SPDX-License-Identifier: GPL-3.0-or-later
//! A SpyServer skimmer over `mfsk_core::iq::IqReceiver`: the shared half of
//! the sample apps in `apps/skimmer/` (the `skimmer` CLI and the Tauri GUI).
//!
//! [`run`] connects, plans a stream, decodes, and reports everything as an
//! [`Event`] until `stop` is set, reconnecting on errors.
//!
//! **Sharing the radio.** The server gives control to one client at a time,
//! and the operator's client takes it when it connects, whichever was first
//! (seen with SDR# and an Airspy HF+, 2026-10-03: SDR# started while this held
//! control, took it and moved the device; closing SDR# gave it back with no
//! reconnect). A client without control cannot move the device outside its
//! band. So:
//!
//! - **Alone**: this has control and keeps it, tuning the device to its IQ
//!   centre and able to write the gain.
//! - **Beside SDR#**: this is the guest. It never tunes the device or writes the
//!   gain; it sets only its own IQ (DDC) centre, which a client without control
//!   may place anywhere in the device's band. Channels outside the band are
//!   paused; a move by the server plans again ([`Event::Moved`]).
//! - [`Config::yield_control`] is the older behaviour: given control, leave and
//!   reconnect after [`Config::retry`]. It woke the radio and dropped it every
//!   retry and is not needed to let SDR# in.
//!
//! **Time.** SpyServer sends no timestamps; see [`anchor`].

pub mod anchor;
pub mod modes;
pub mod plan;
pub mod spyserver;
pub mod waterfall;

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub use mfsk_core::Mode;
use mfsk_core::decoder::{AnyDecoder, default_params};
pub use mfsk_core::decoder::{ApMode, Contest, Depth, QsoContext, QsoProgress, Station};
pub use mfsk_core::iq::Channelizer;
use mfsk_core::iq::CompletedSlot;
use mfsk_core::iq::{IqReceiver, IqSampleFormat, IqStream};
use mfsk_core::msg::ap::ApHint;
use mfsk_core::slotgrid::ClockChange;

use anchor::AnchorEstimate;
use plan::{Plan, plan};
use spyserver::*;

pub fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as i64
}

/// One channel: a mode at a USB dial frequency, and how it is decoded.
#[derive(Clone, Debug, PartialEq)]
pub struct ChannelSpec {
    pub mode: Mode,
    pub dial_hz: f64,
    pub options: ChannelOptions,
}

/// A channel's part of WSJT-X's parameter block (`jt9com`) plus the library's
/// "hunt one DX" hint. `None` / `false` / empty keep the mode's default, as
/// the GUI's untouched boxes do. Each mode reads what its upstream decoder
/// reads and ignores the rest.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ChannelOptions {
    /// Audio band searched (`nfa`, `nfb`), Hz.
    pub band_hz: Option<(f32, f32)>,
    /// The Rx frequency (`nfqso`), tolerance (`ntol`) and Tx frequency
    /// (`nftx`), Hz.
    pub rx_freq_hz: Option<f32>,
    pub tol_hz: Option<f32>,
    pub tx_freq_hz: Option<f32>,
    /// `ndepth & 7`; `None` is Deep, the GUI default.
    pub depth: Option<Depth>,
    /// AP (`lft8apon` / `lapcqonly`); `None` is the mode's GUI default.
    pub ap: Option<ApMode>,
    /// `ndepth & 16` (JT65, Q65), `ndepth & 32` (JT65), `emedelay`.
    pub averaging: bool,
    pub deep_search: bool,
    pub eme_delay: bool,
    /// `hiscall`, `hisgrid`, `nQSOProgress`: with the station, what the QSO
    /// context AP of FT8, FT4 and FST4 is derived from.
    pub qso: QsoContext,
    /// `ncontest`.
    pub contest: Contest,
    /// A station to hunt: its call is given to the decoder as an a-priori
    /// hint (FT8, FT4, FST4, Q65; ignored by modes without AP).
    pub dx_call: Option<String>,
}

/// Options changed while the skimmer runs: [`LiveOptions::set`] from any
/// thread, picked up by the stream loop between IQ messages and handed to
/// the channel's decoder thread, which applies them before its next slot.
/// The operator's station (`mycall`, `mygrid`) is one for all channels.
#[derive(Debug, Default)]
pub struct LiveOptions {
    generation: std::sync::atomic::AtomicU64,
    state: std::sync::Mutex<LiveState>,
    /// A gain set while running, stored as index + 1 (`0`: none): wins over
    /// [`Config::gain`], also on a reconnect.
    gain: std::sync::atomic::AtomicU64,
    /// The channel whose waterfall rows are sent whole, as index + 1 (`0`:
    /// none); the others send a coarse thumbnail.
    wf_focus: std::sync::atomic::AtomicU64,
    /// The focused channel's rows at 1.5 Hz per bin instead of 2.9.
    wf_fine: std::sync::atomic::AtomicBool,
}

#[derive(Debug, Default)]
struct LiveState {
    options: Vec<ChannelOptions>,
    station: Station,
}

impl LiveOptions {
    /// Replace channel `index`'s options (an index of `Config::channels`).
    pub fn set(&self, index: usize, options: ChannelOptions) {
        let mut st = self.state.lock().unwrap();
        if st.options.len() <= index {
            st.options.resize(index + 1, ChannelOptions::default());
        }
        st.options[index] = options;
        drop(st);
        self.generation.fetch_add(1, Ordering::Release);
    }

    /// Replace the operator's station, for every channel.
    pub fn set_station(&self, station: Station) {
        self.state.lock().unwrap().station = station;
        self.generation.fetch_add(1, Ordering::Release);
    }

    pub fn station(&self) -> Station {
        self.state.lock().unwrap().station.clone()
    }

    fn generation(&self) -> u64 {
        self.generation.load(Ordering::Acquire)
    }

    /// The channel (an index of `Config::channels`) whose waterfall is sent in
    /// full; `None` sends thumbnails only.
    pub fn set_waterfall_focus(&self, channel: Option<usize>) {
        self.wf_focus
            .store(channel.map_or(0, |c| c as u64 + 1), Ordering::Release);
    }

    /// 1.5 Hz bins (8192-point FFT) for the focused channel instead of 2.9 Hz.
    pub fn set_waterfall_fine(&self, fine: bool) {
        self.wf_fine.store(fine, Ordering::Release);
    }

    fn waterfall_focus(&self) -> Option<usize> {
        match self.wf_focus.load(Ordering::Acquire) {
            0 => None,
            c => Some((c - 1) as usize),
        }
    }

    /// Set the device gain index in a running skimmer. Takes effect on the next
    /// IQ message if this client holds control; a guest's request is ignored by
    /// the server, so it is not sent.
    pub fn set_gain(&self, gain: u32) {
        self.gain.store(u64::from(gain) + 1, Ordering::Release);
    }

    fn gain(&self) -> Option<u32> {
        match self.gain.load(Ordering::Acquire) {
            0 => None,
            g => Some((g - 1) as u32),
        }
    }

    fn get(&self, index: usize) -> Option<(ChannelOptions, Station)> {
        let st = self.state.lock().unwrap();
        st.options
            .get(index)
            .cloned()
            .map(|o| (o, st.station.clone()))
    }
}

impl ChannelSpec {
    pub fn new(mode: Mode, dial_hz: f64) -> Self {
        ChannelSpec {
            mode,
            dial_hz,
            options: ChannelOptions::default(),
        }
    }

    /// This channel's decoder, as configured.
    fn decoder(&self, station: &Station) -> AnyDecoder {
        let mut d = AnyDecoder::with_defaults(self.mode);
        apply_options(&mut d, &self.options, station);
        d
    }
}

/// Set `o` and the station on `d`: unset fields return to the mode's
/// defaults. A mode without AP simply hunts nothing.
fn apply_options(d: &mut AnyDecoder, o: &ChannelOptions, station: &Station) {
    let dflt = default_params(d.mode());
    let p = d.params_mut();
    p.band_hz = o.band_hz.unwrap_or(dflt.band_hz);
    p.rx_freq_hz = o.rx_freq_hz;
    p.tol_hz = o.tol_hz;
    p.tx_freq_hz = o.tx_freq_hz;
    p.depth = o.depth.unwrap_or(dflt.depth);
    p.ap = o.ap.unwrap_or(dflt.ap);
    p.averaging = o.averaging;
    p.deep_search = o.deep_search;
    p.eme_delay = o.eme_delay;
    p.station = station.clone();
    p.qso = o.qso.clone();
    p.contest = o.contest;
    let hint = o.dx_call.as_ref().map(|dx| ApHint {
        call2: Some(dx.clone()),
        ..ApHint::default()
    });
    let _ = d.set_ap_hint(hint);
}

/// What a channel's worker has to do.
enum Job {
    Slot(CompletedSlot),
    Options(ChannelOptions, Station),
}

/// One channel's decoder thread. The socket reader only cuts slots and
/// queues them, so a slow decode (a busy FT8 band, FST4-300) delays that
/// channel's rows and nothing else; a full queue drops the slot, counted.
struct Worker {
    tx: std::sync::mpsc::SyncSender<Job>,
    handle: Option<std::thread::JoinHandle<()>>,
}

/// Slots a worker may have waiting. A slot decodes in a fraction of its
/// period, so more than this means the channel cannot keep up.
const WORKER_QUEUE: usize = 4;

impl Worker {
    fn spawn(
        channel: usize,
        dial_hz: f64,
        mut decoder: AnyDecoder,
        results: std::sync::mpsc::Sender<Decode>,
        busy: std::sync::Arc<std::sync::atomic::AtomicUsize>,
        longest_us: std::sync::Arc<std::sync::atomic::AtomicU64>,
    ) -> Self {
        let (tx, rx) = std::sync::mpsc::sync_channel::<Job>(WORKER_QUEUE);
        let handle = std::thread::Builder::new()
            .name(format!("decode-{channel}"))
            .spawn(move || {
                for job in rx {
                    let slot = match job {
                        Job::Slot(slot) => slot,
                        Job::Options(o, station) => {
                            apply_options(&mut decoder, &o, &station);
                            continue;
                        }
                    };
                    let t = Instant::now();
                    for d in decoder.decode(&slot.input()).rows {
                        let _ = results.send(Decode {
                            channel,
                            mode: slot.mode,
                            slot_utc_ns: slot.utc_ns,
                            dial_hz,
                            freq_hz: slot.abs_freq_hz(d.freq_hz),
                            snr_db: d.snr_db,
                            dt_s: d.dt_sec,
                            text: d.text,
                        });
                    }
                    longest_us.fetch_max(t.elapsed().as_micros() as u64, Ordering::Relaxed);
                    busy.fetch_sub(1, Ordering::Relaxed);
                }
            })
            .expect("spawn decoder thread");
        Worker {
            tx,
            handle: Some(handle),
        }
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        // Closing the queue ends the thread after the slot in hand.
        let (dead, _) = std::sync::mpsc::sync_channel(1);
        drop(std::mem::replace(&mut self.tx, dead));
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The IQ format asked of the server. Float needs no digital-gain guess to
/// stay clear of clipping, at twice the bytes (3.6 MB/s at 456 kS/s). A
/// server that forces another format is read as it comes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WireFormat {
    Float,
    Int16,
}

#[derive(Clone, Debug)]
pub struct Config {
    pub server: String,
    pub channels: Vec<ChannelSpec>,
    /// IQ centre; default 25 kHz below the lowest dial.
    pub center_hz: Option<f64>,
    /// IQ rate; default the lowest that holds the most channels.
    pub rate: Option<u32>,
    /// Device gain index, applied when holding control.
    pub gain: Option<u32>,
    /// Hold control and tune the radio even if [`Self::yield_control`] is set.
    pub tune: bool,
    /// When given control, leave it for an operator's client started later
    /// (and reconnect after [`Self::retry`] as a guest) instead of holding it.
    /// Default `false`: take control and tune.
    pub yield_control: bool,
    /// Produce [`Event::Waterfall`] rows (a fine spectrum of each channel's
    /// audio). Off by default: the rows cost an FFT per channel per 0.17 s and a
    /// window to draw them.
    pub waterfall: bool,
    pub format: WireFormat,
    /// `None` chooses by channel count: see [`AUTO_PFB_CHANNELS`].
    pub channelizer: Option<Channelizer>,
    pub iq_swap: bool,
    /// Re-anchor when the arrival-time estimate moves by more than this.
    pub reanchor: Duration,
    /// Wait between a lost connection (or giving up control) and the next try.
    pub retry: Duration,
    /// Per-channel options changed while running.
    pub live: std::sync::Arc<LiveOptions>,
}

impl Config {
    pub fn new(server: impl Into<String>, channels: Vec<ChannelSpec>) -> Self {
        Config {
            server: server.into(),
            channels,
            center_hz: None,
            rate: None,
            gain: None,
            tune: false,
            yield_control: false,
            waterfall: false,
            format: WireFormat::Float,
            channelizer: None,
            iq_swap: false,
            reanchor: Duration::from_millis(500),
            retry: Duration::from_secs(10),
            live: Default::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct DeviceInfo {
    /// SpyServer's device type: 1 Airspy, 2 Airspy HF+, 3 RTL-SDR.
    pub kind: u32,
    pub max_rate: u32,
    pub bandwidth_hz: f64,
    /// Highest gain index the device takes (`SET_GAIN`: 0 to this).
    pub max_gain: u32,
    /// Whether this client may change the gain: only the one with control.
    pub can_control: bool,
    /// The gain index the device has now (SDR# set it, or the server's default).
    pub gain: u32,
}

/// What the radio reports about itself and this client's standing with it:
/// the current gain index, the highest one, and whether this client may write
/// it. Sent with [`Event::Radio`] when it connects and whenever the server's
/// sync message changes either value (another client moved the gain).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RadioState {
    pub gain: u32,
    pub max_gain: u32,
    pub can_control: bool,
}

/// A [`waterfall::Row`] of one channel. `focus` rows are whole (the channel the
/// window shows large); the others are pooled four bins to one and every other
/// row, a thumbnail.
#[derive(Clone, Debug)]
pub struct WaterfallRow {
    /// An index of `Config::channels`.
    pub channel: usize,
    pub focus: bool,
    pub row: waterfall::Row,
}

/// One decoded message.
#[derive(Clone, Debug)]
pub struct Decode {
    /// Index into [`Config::channels`].
    pub channel: usize,
    pub mode: Mode,
    /// UTC of the slot start, ns since the epoch.
    pub slot_utc_ns: Option<i64>,
    /// The channel's dial.
    pub dial_hz: f64,
    /// RF frequency of tone 0.
    pub freq_hz: f64,
    pub snr_db: f32,
    pub dt_s: f32,
    pub text: String,
}

#[derive(Clone, Debug)]
pub struct Status {
    pub streamed_s: f64,
    /// Arrival delay of the last message past the anchor estimate.
    pub delay_ms: f64,
    /// How far the anchor estimate has moved from the anchor in use.
    pub drift_ms: f64,
    /// Longest single push since the last status (cutting and channelizing; decoding runs on the workers).
    pub longest_push_ms: f64,
    /// Bytes read off the socket and not yet decoded.
    pub queued_bytes: usize,
    /// Longest slot decode on a worker since the last status.
    pub longest_decode_ms: f64,
    /// Slots waiting for, or in, a decoder thread.
    pub queued_slots: usize,
    /// Slots dropped because a channel's decoder could not keep up.
    pub dropped_slots: u64,
    pub gaps: u64,
    pub reanchors: u64,
}

#[derive(Clone, Debug)]
pub enum Event {
    Connecting {
        server: String,
    },
    Connected {
        device: DeviceInfo,
        control: bool,
        device_hz: f64,
    },
    /// The radio's gain or this client's control changed (see [`RadioState`]).
    Radio(RadioState),
    /// A spectrum row of one channel's audio (see [`waterfall`]).
    Waterfall(WaterfallRow),
    /// Got control with `yield_control` set: leaving it for the operator's client.
    Yielded,
    /// No channel fits the band around the device centre; waiting for it to move.
    NoChannelFits {
        device_hz: f64,
    },
    /// Decoding on this stream; `active[i]` says whether channel `i` is in it.
    Streaming {
        rate: u32,
        decimation: u32,
        center_hz: f64,
        device_hz: f64,
        active: Vec<bool>,
        channelizer: Channelizer,
    },
    /// The server moved the device or this client's IQ centre; planning again.
    Moved {
        device_hz: f64,
        iq_hz: f64,
    },
    Decode(Decode),
    /// The server dropped IQ messages; slots spanning the hole are lost.
    Gap {
        messages: u32,
        at_s: f64,
    },
    Reanchor {
        by_s: f64,
    },
    /// Once a minute of samples.
    Status(Status),
    /// The connection failed or dropped; retrying after `retry`.
    Disconnected {
        error: String,
    },
}

/// A WSJT-X `ALL.TXT`-style line: `yyMMdd_hhmmss  dial-MHz Rx MODE snr dt
/// audio-Hz message`. WSJT-X's column widths, not guaranteed identical.
pub fn all_txt_line(d: &Decode) -> String {
    let t = d
        .slot_utc_ns
        .map(|ns| {
            let s = ns.div_euclid(1_000_000_000);
            let (days, sod) = (s.div_euclid(86_400), s.rem_euclid(86_400));
            let (y, m, dd) = civil_from_days(days);
            format!(
                "{:02}{:02}{:02}_{:02}{:02}{:02}",
                y % 100,
                m,
                dd,
                sod / 3600,
                sod / 60 % 60,
                sod % 60
            )
        })
        .unwrap_or_else(|| "000000_000000".into());
    format!(
        "{t} {:>10.3} Rx {:<6} {:>4.0} {:>4.1} {:>5.0} {}",
        d.dial_hz / 1e6,
        modes::mode_name(d.mode),
        d.snr_db,
        d.dt_s,
        d.freq_hz - d.dial_hz,
        d.text
    )
}

/// Days since 1970-01-01 to (year, month, day), proleptic Gregorian
/// (Howard Hinnant's `civil_from_days`).
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (yoe + era * 400 + i64::from(m <= 2), m, d)
}

/// With [`Config::channelizer`] unset, streams with at least this many
/// active channels use the filter bank, fewer use `Direct`. Measured
/// break-even (`docs/notes/IQ_CHANNELIZER.md` §7b, one thread): about four
/// channels at both 768 kS/s and 2.4 MS/s (4 channels: Direct 3.68 % of a
/// core against the bank's 3.34 % at 768 kS/s).
pub const AUTO_PFB_CHANNELS: usize = 4;

/// The channelizer a stream with `active` channels uses.
pub fn channelizer_for(cfg: &Config, active: usize) -> Channelizer {
    cfg.channelizer.unwrap_or(if active >= AUTO_PFB_CHANNELS {
        Channelizer::Pfb
    } else {
        Channelizer::Direct
    })
}

/// How long the minimum-delay anchor estimate looks back. Long enough that a
/// quiet moment on the network falls inside it, short enough to follow an
/// NTP step within a slot or two.
const ANCHOR_WINDOW_S: f64 = 30.0;

enum End {
    Stopped,
    Yielded,
}

/// A connection failure in a few words. The OS texts are long, localised and
/// say nothing about the cause that matters here: a SpyServer at its client
/// limit (another client, or a machine that went to sleep with its connection
/// open) accepts the socket and closes it at once.
fn describe(e: &std::io::Error) -> String {
    use std::io::ErrorKind::*;
    match e.kind() {
        UnexpectedEof => "closed by the server (client limit?)".into(),
        ConnectionRefused => "refused (is the server running?)".into(),
        TimedOut => "no reply".into(),
        ConnectionReset | ConnectionAborted | BrokenPipe => "connection lost".into(),
        _ => e.to_string(),
    }
}

/// Run until `stop` is set: connect, stream, decode, reconnect on failure.
pub fn run(cfg: &Config, stop: &AtomicBool, mut on_event: impl FnMut(Event)) {
    while !stop.load(Ordering::Relaxed) {
        on_event(Event::Connecting {
            server: cfg.server.clone(),
        });
        match session(cfg, stop, &mut on_event) {
            Ok(End::Stopped) => return,
            Ok(End::Yielded) => on_event(Event::Yielded),
            Err(e) if is_stop(&e) => return,
            Err(e) => on_event(Event::Disconnected {
                error: describe(&e),
            }),
        }
        let until = Instant::now() + cfg.retry;
        while Instant::now() < until {
            if stop.load(Ordering::Relaxed) {
                return;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

fn session(
    cfg: &Config,
    stop: &AtomicBool,
    on_event: &mut impl FnMut(Event),
) -> std::io::Result<End> {
    let mut c = Conn::connect(&cfg.server)?;
    let mut hello = PROTOCOL_VERSION.to_le_bytes().to_vec();
    hello.extend_from_slice(b"mfsk-core skimmer");
    c.command(CMD_HELLO, &hello)?;

    let (mut dev, mut sync) = (None, None);
    while dev.is_none() || sync.is_none() {
        let m = c.read(stop)?;
        match m.kind {
            MSG_DEVICE_INFO => dev = Some(Device::parse(&m.body)),
            MSG_CLIENT_SYNC => sync = Some(Sync::parse(&m.body)),
            _ => {}
        }
    }
    let (dev, mut sync) = (dev.unwrap(), sync.unwrap());
    on_event(Event::Connected {
        device: DeviceInfo {
            kind: dev.kind,
            max_rate: dev.max_rate,
            bandwidth_hz: dev.bandwidth_hz,
            max_gain: dev.max_gain,
            can_control: sync.can_control,
            gain: sync.gain,
        },
        control: sync.can_control,
        device_hz: sync.device_hz,
    });
    on_event(Event::Radio(RadioState {
        gain: sync.gain,
        max_gain: dev.max_gain,
        can_control: sync.can_control,
    }));
    // Given control: hold it and tune, unless asked to leave it for an
    // operator's client started later (holding control would lock that client
    // out of tuning, and the protocol has no way to hand it on).
    let hold = sync.can_control && (cfg.tune || !cfg.yield_control);
    if sync.can_control && !hold {
        return Ok(End::Yielded);
    }
    // `--gain` is written once, when this client first holds control: a plan
    // made again later (the operator's client moved the device) must not write
    // it over what that client has set.
    let mut gain_once = if hold { cfg.gain } else { None };

    loop {
        // Whether this client has control is the server's latest word, not what
        // it was when we connected: SDR# takes control when it connects, and a
        // client without it cannot tune the device or write the gain.
        let tune = sync.can_control && (cfg.tune || !cfg.yield_control);
        let Some(p) = plan(
            &cfg.channels,
            cfg.center_hz,
            cfg.rate,
            &dev,
            sync.device_hz,
            tune,
        ) else {
            on_event(Event::NoChannelFits {
                device_hz: sync.device_hz,
            });
            sync = wait_for_move(&mut c, stop, sync)?;
            continue;
        };
        let writes = Writes {
            tune,
            set_gain: gain_once.take(),
            gain_now: sync.gain,
        };
        let got = apply(&mut c, stop, &dev, &p, cfg, writes)?;
        sync = got;
        if got.iq_hz != p.center_hz {
            // Refused or clamped by the server: plan against what it says.
            on_event(Event::Moved {
                device_hz: got.device_hz,
                iq_hz: got.iq_hz,
            });
            continue;
        }
        on_event(Event::Streaming {
            rate: p.rate,
            decimation: p.decimation,
            center_hz: p.center_hz,
            device_hz: sync.device_hz,
            active: (0..cfg.channels.len())
                .map(|i| p.active.contains(&i))
                .collect(),
            channelizer: channelizer_for(cfg, p.active.len()),
        });
        c.set(SET_STREAMING_ENABLED, 1)?;
        match stream(&mut c, stop, cfg, &dev, &p, sync, on_event)? {
            None => return Ok(End::Stopped),
            Some(s) => {
                on_event(Event::Moved {
                    device_hz: s.device_hz,
                    iq_hz: s.iq_hz,
                });
                sync = s;
            }
        }
    }
}

/// Apply a plan with streaming off, and read back what the server made of
/// it. CLIENT_SYNC comes when the decimation changes, not on a frequency
/// alone (measured), so the decimation is stepped away and back after the
/// frequency, and PING / PONG marks the end of the replies.
fn apply(
    c: &mut Conn,
    stop: &AtomicBool,
    dev: &Device,
    p: &Plan,
    cfg: &Config,
    w: Writes,
) -> std::io::Result<Sync> {
    let Writes {
        tune,
        set_gain,
        gain_now,
    } = w;
    c.set(SET_STREAMING_ENABLED, 0)?;
    c.set(
        SET_IQ_FORMAT,
        match cfg.format {
            WireFormat::Float => FORMAT_FLOAT,
            WireFormat::Int16 => FORMAT_INT16,
        },
    )?;
    c.set(SET_IQ_FREQUENCY, p.center_hz as u32)?;
    // The gain the radio has, or the one just written: the Airspy One's digital
    // gain makes up the difference.
    let mut gain = gain_now;
    if tune && let Some(g) = set_gain {
        c.set(SET_GAIN, g)?;
        gain = g;
    }
    let other = if p.decimation > dev.rates[0].0 {
        p.decimation - 1
    } else {
        p.decimation + 1
    };
    c.set(SET_IQ_DECIMATION, other)?;
    c.set(SET_IQ_DECIMATION, p.decimation)?;
    // As SDR++ does: 3 dB of digital gain per decimation stage keeps the
    // level in the integer formats (`computeDigitalGain`); the Airspy One
    // also makes up its device gain.
    c.set(
        SET_IQ_DIGITAL_GAIN,
        digital_gain(dev, p.decimation, Some(gain)),
    )?;
    c.set(SET_STREAMING_MODE, STREAM_MODE_IQ_ONLY)?;
    c.command(CMD_PING, &[])?;
    let mut last = None;
    loop {
        let m = c.read(stop)?;
        match m.kind {
            MSG_CLIENT_SYNC => last = Some(Sync::parse(&m.body)),
            MSG_PONG => break,
            _ => {}
        }
    }
    last.ok_or_else(|| std::io::Error::other("no CLIENT_SYNC in reply to the settings"))
}

/// Streaming off; block until a CLIENT_SYNC shows a different device centre.
///
/// With streaming off nothing arrives unprompted, so a PING every
/// [`PING_EVERY`] keeps `Conn::read`'s stall check from taking an idle
/// connection for a dead one, and finds a dead one.
fn wait_for_move(c: &mut Conn, stop: &AtomicBool, was: Sync) -> std::io::Result<Sync> {
    c.set(SET_STREAMING_ENABLED, 0)?;
    loop {
        c.command(CMD_PING, &[])?;
        loop {
            let m = c.read(stop)?;
            match m.kind {
                MSG_CLIENT_SYNC => {
                    let s = Sync::parse(&m.body);
                    if s.device_hz != was.device_hz {
                        return Ok(s);
                    }
                }
                MSG_PONG => break,
                _ => {}
            }
        }
        let until = Instant::now() + PING_EVERY;
        while Instant::now() < until {
            if stop.load(Ordering::Relaxed) {
                return Err(std::io::ErrorKind::Interrupted.into());
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

/// The receiver of one stream.
struct Live {
    rx: IqReceiver,
    /// By `ChannelId`: the channel's decoder thread; its decoder keeps its
    /// own callsign table from slot to slot.
    workers: Vec<Option<Worker>>,
    /// By `ChannelId`: the index into `Config::channels`.
    cfg_index: Vec<usize>,
    /// Rows the workers found, waiting to be reported.
    results: std::sync::mpsc::Receiver<Decode>,
    /// Slots queued or being decoded.
    busy: std::sync::Arc<std::sync::atomic::AtomicUsize>,
    /// Longest slot decode since the last status, in µs.
    longest_us: std::sync::Arc<std::sync::atomic::AtomicU64>,
    /// By `ChannelId`: the channel's waterfall, when `Config::waterfall`.
    wfs: Vec<Option<ChannelWaterfall>>,
    format: IqSampleFormat,
}

/// A channel's waterfall at both resolutions, and the row count that thins its
/// thumbnail.
struct ChannelWaterfall {
    coarse: waterfall::Waterfall,
    fine: waterfall::Waterfall,
    ticks: u32,
    /// The last ~2 minutes of rows at each resolution, so that choosing this
    /// channel for the large waterfall (or the fine resolution) shows its past
    /// at once instead of starting empty.
    hist_coarse: std::collections::VecDeque<waterfall::Row>,
    hist_fine: std::collections::VecDeque<waterfall::Row>,
}

/// How much waterfall history is kept, ns: the longest screen any mode shows.
const WF_KEEP_NS: i64 = 135_000_000_000;

fn remember(hist: &mut std::collections::VecDeque<waterfall::Row>, row: &waterfall::Row) {
    hist.push_back(row.clone());
    while hist
        .front()
        .is_some_and(|f| f.utc_ns < row.utc_ns - WF_KEEP_NS)
    {
        hist.pop_front();
    }
}

/// The audio band a waterfall shows: the decoders' own bands lie inside it.
const WF_BAND_HZ: (f32, f32) = (100.0, 3_100.0);

impl ChannelWaterfall {
    fn new() -> Self {
        ChannelWaterfall {
            coarse: waterfall::Waterfall::new(4096, 2048, WF_BAND_HZ.0, WF_BAND_HZ.1),
            fine: waterfall::Waterfall::new(8192, 4096, WF_BAND_HZ.0, WF_BAND_HZ.1),
            ticks: 0,
            hist_coarse: Default::default(),
            hist_fine: Default::default(),
        }
    }
}

/// What [`apply`] may write to the device: only a client with control can.
struct Writes {
    /// Whether this client has control (and wants to use it).
    tune: bool,
    /// A gain to write now (`--gain`, once).
    set_gain: Option<u32>,
    /// The gain the radio has now, for the Airspy One's digital gain.
    gain_now: u32,
}

/// The IQ digital gain SDR++ sets (`computeDigitalGain`): 3 dB per decimation
/// stage keeps the level in the integer formats, and the Airspy One also makes
/// up its device gain.
fn digital_gain(dev: &Device, decimation: u32, gain: Option<u32>) -> u32 {
    (decimation as f32 * 3.01) as u32
        + if dev.kind == DEVICE_AIRSPY_ONE {
            dev.max_gain.saturating_sub(gain.unwrap_or(0))
        } else {
            0
        }
}

/// A receiver for the plan's channels, built for the format the server sends.
fn receiver(cfg: &Config, p: &Plan, format: IqSampleFormat) -> std::io::Result<Live> {
    let stream = IqStream::new(p.rate, p.center_hz, format).iq_swap(cfg.iq_swap);
    let mut rx = IqReceiver::with_channelizer(stream, channelizer_for(cfg, p.active.len()))
        .map_err(|e| std::io::Error::other(format!("{} S/s: {e}", p.rate)))?;
    let (rtx, results) = std::sync::mpsc::channel();
    let busy = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let longest_us = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
    let mut workers: Vec<Option<Worker>> = Vec::new();
    let mut cfg_index: Vec<usize> = Vec::new();
    let mut wfs: Vec<Option<ChannelWaterfall>> = Vec::new();
    for &i in &p.active {
        let ch = &cfg.channels[i];
        let id = rx
            .add_channel(ch.dial_hz, ch.mode)
            .map_err(|e| std::io::Error::other(format!("{}: {e}", ch.dial_hz)))?;
        if workers.len() <= id.0 {
            workers.resize_with(id.0 + 1, || None);
        }
        if cfg_index.len() <= id.0 {
            cfg_index.resize(id.0 + 1, usize::MAX);
        }
        cfg_index[id.0] = i;
        if cfg.waterfall {
            rx.tap_audio(id, true);
            if wfs.len() <= id.0 {
                wfs.resize_with(id.0 + 1, || None);
            }
            wfs[id.0] = Some(ChannelWaterfall::new());
        }
        workers[id.0] = Some(Worker::spawn(
            i,
            ch.dial_hz,
            ch.decoder(&cfg.live.station()),
            rtx.clone(),
            busy.clone(),
            longest_us.clone(),
        ));
    }
    Ok(Live {
        rx,
        workers,
        cfg_index,
        results,
        busy,
        longest_us,
        wfs,
        format,
    })
}

/// Decode until stopped (`None`) or the server says the device or this
/// client's IQ centre moved (`Some`).
fn stream(
    c: &mut Conn,
    stop: &AtomicBool,
    cfg: &Config,
    dev: &Device,
    p: &Plan,
    sync: Sync,
    on_event: &mut impl FnMut(Event),
) -> std::io::Result<Option<Sync>> {
    let rate = p.rate;
    // Built on the first IQ message, in the format the server actually sends.
    let mut live: Option<Live> = None;
    let mut est = AnchorEstimate::new(rate, ANCHOR_WINDOW_S);
    let mut anchor: Option<i64> = None;
    let mut next_seq: Option<u32> = None;
    let mut last_report = 0u64;
    let (mut gaps, mut reanchors) = (0u64, 0u64);
    let mut worst_push = Duration::ZERO;
    let mut dropped_slots = 0u64;
    let mut seen_generation = cfg.live.generation();
    let mut applied_gain = cfg.live.gain();
    // The (focus, fine) choice whose history has been sent.
    let mut wf_sent: (Option<usize>, bool) = (None, false);
    let mut radio = RadioState {
        gain: sync.gain,
        max_gain: dev.max_gain,
        can_control: sync.can_control,
    };
    loop {
        let m = match c.read(stop) {
            Ok(m) => m,
            Err(e) if is_stop(&e) => return Ok(None),
            Err(e) => return Err(e),
        };
        if m.kind == MSG_CLIENT_SYNC {
            let s = Sync::parse(&m.body);
            let now = RadioState {
                gain: s.gain,
                max_gain: dev.max_gain,
                can_control: s.can_control,
            };
            if now != radio {
                radio = now;
                on_event(Event::Radio(now));
            }
            if s.device_hz != sync.device_hz || s.iq_hz != sync.iq_hz {
                return Ok(Some(s));
            }
            continue;
        }
        let Some(format) = sample_format(m.kind) else {
            continue;
        };
        if live.as_ref().is_none_or(|l| l.format != format) {
            live = Some(receiver(cfg, p, format)?);
            (anchor, next_seq) = (None, None);
            est = AnchorEstimate::new(rate, ANCHOR_WINDOW_S);
        }
        let Live {
            rx,
            workers,
            cfg_index,
            results,
            busy,
            longest_us,
            wfs,
            ..
        } = live.as_mut().unwrap();

        // A gain changed while running (only the controlling client's request
        // is honoured by the server). The Airspy One's digital gain makes up
        // the device gain, so it follows.
        if radio.can_control
            && let Some(g) = cfg.live.gain().filter(|g| Some(*g) != applied_gain)
        {
            c.set(SET_GAIN, g)?;
            c.set(
                SET_IQ_DIGITAL_GAIN,
                digital_gain(dev, p.decimation, Some(g)),
            )?;
            applied_gain = Some(g);
        }

        // Options changed since the last message go to their channels; one
        // whose queue is full is tried again with the next message.
        let generation = cfg.live.generation();
        if generation != seen_generation {
            let mut all_sent = true;
            for (id, w) in workers.iter().enumerate() {
                let Some(w) = w else { continue };
                if let Some((o, station)) = cfg.live.get(cfg_index[id])
                    && w.tx.try_send(Job::Options(o, station)).is_err()
                {
                    all_sent = false;
                }
            }
            if all_sent {
                seen_generation = generation;
            }
        }
        let n = m.body.len() / format.bytes_per_sample();

        // The sequence counts messages; a hole of d messages is taken to be
        // d messages of this one's length.
        if let Some(want) = next_seq {
            let missed = m.seq.wrapping_sub(want);
            if missed != 0 && missed < u32::MAX / 2 {
                gaps += 1;
                on_event(Event::Gap {
                    messages: missed,
                    at_s: rx.samples_in() as f64 / rate as f64,
                });
                rx.gap(missed as u64 * n as u64);
            }
        }
        next_seq = Some(m.seq.wrapping_add(1));

        let t = Instant::now();
        let mut slots = Vec::new();
        rx.push_bytes(&m.body, &mut slots);
        if cfg.waterfall {
            let focus = cfg.live.waterfall_focus();
            let fine = cfg.live.wf_fine.load(Ordering::Acquire);
            let end = rx.utc_of(rx.samples_in());
            // A new choice of channel or resolution: send its past first.
            if (focus, fine) != wf_sent {
                wf_sent = (focus, fine);
                if let Some(ch) = focus
                    && let Some(id) = cfg_index.iter().position(|&c| c == ch)
                    && let Some(Some(wf)) = wfs.get(id)
                {
                    let hist = if fine { &wf.hist_fine } else { &wf.hist_coarse };
                    for row in hist {
                        on_event(Event::Waterfall(WaterfallRow {
                            channel: ch,
                            focus: true,
                            row: row.clone(),
                        }));
                    }
                }
            }
            let mut audio = Vec::new();
            let mut coarse_rows = Vec::new();
            let mut fine_rows = Vec::new();
            for (id, wf) in wfs.iter_mut().enumerate() {
                let Some(wf) = wf else { continue };
                audio.clear();
                rx.take_audio(mfsk_core::iq::ChannelId(id), &mut audio);
                let channel = cfg_index[id];
                let is_focus = focus == Some(channel);
                coarse_rows.clear();
                fine_rows.clear();
                // Both resolutions run for every channel: choosing one later
                // then has a history to show.
                wf.coarse.push(&audio, end, &mut coarse_rows);
                wf.fine.push(&audio, end, &mut fine_rows);
                for r in &coarse_rows {
                    remember(&mut wf.hist_coarse, r);
                }
                for r in &fine_rows {
                    remember(&mut wf.hist_fine, r);
                }
                let sent = if is_focus && fine {
                    &fine_rows
                } else {
                    &coarse_rows
                };
                for row in sent {
                    if is_focus {
                        on_event(Event::Waterfall(WaterfallRow {
                            channel,
                            focus: true,
                            row: row.clone(),
                        }));
                    } else {
                        wf.ticks = wf.ticks.wrapping_add(1);
                        if wf.ticks.is_multiple_of(2) {
                            on_event(Event::Waterfall(WaterfallRow {
                                channel,
                                focus: false,
                                row: row.pooled(4),
                            }));
                        }
                    }
                }
            }
        }
        for slot in slots {
            let Some(w) = workers[slot.channel.0].as_ref() else {
                continue;
            };
            busy.fetch_add(1, Ordering::Relaxed);
            if w.tx.try_send(Job::Slot(slot)).is_err() {
                busy.fetch_sub(1, Ordering::Relaxed);
                dropped_slots += 1;
            }
        }
        while let Ok(d) = results.try_recv() {
            on_event(Event::Decode(d));
        }
        worst_push = worst_push.max(t.elapsed());

        // The arrival stamps the *end* of this message; `best` is the UTC of
        // sample 0 the lowest delay in the window implies. The receiver
        // follows it at a bounded rate, so a drifting host clock moves the
        // slot boundaries by milliseconds and loses no slot.
        let samples = rx.samples_in();
        let best = est.push(samples, rate, m.arrival_ns);
        let warming_up = samples < 2 * rate as u64;
        // The first two seconds settle the minimum; no slot is complete yet.
        if !warming_up || anchor.is_none() {
            let at = best + (samples as i128 * 1_000_000_000 / rate as i128) as i64;
            match rx.set_time(at, samples) {
                ClockChange::Stepped { by_ns } if anchor.is_some() => {
                    reanchors += 1;
                    on_event(Event::Reanchor {
                        by_s: by_ns as f64 * 1e-9,
                    });
                }
                _ => {}
            }
            anchor = rx.utc_of(0);
        }
        if samples - last_report >= 60 * rate as u64 {
            last_report = samples;
            let a = anchor.unwrap_or(best);
            on_event(Event::Status(Status {
                streamed_s: samples as f64 / rate as f64,
                delay_ms: (m.arrival_ns - a) as f64 * 1e-6 - samples as f64 * 1e3 / rate as f64,
                drift_ms: (best - a) as f64 * 1e-6,
                longest_push_ms: worst_push.as_secs_f64() * 1e3,
                queued_bytes: c.queued_bytes(),
                queued_slots: busy.load(Ordering::Relaxed),
                longest_decode_ms: longest_us.swap(0, Ordering::Relaxed) as f64 * 1e-3,
                dropped_slots,
                gaps,
                reanchors,
            }));
            worst_push = Duration::ZERO;
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn connection_failures_read_in_a_few_words() {
        use std::io::{Error, ErrorKind};
        for (kind, want) in [
            (
                ErrorKind::UnexpectedEof,
                "closed by the server (client limit?)",
            ),
            (
                ErrorKind::ConnectionRefused,
                "refused (is the server running?)",
            ),
            (ErrorKind::TimedOut, "no reply"),
            (ErrorKind::ConnectionReset, "connection lost"),
        ] {
            assert_eq!(super::describe(&Error::from(kind)), want);
        }
    }

    use super::*;

    #[test]
    fn all_txt_line_columns() {
        let d = Decode {
            channel: 0,
            mode: Mode::Ft8,
            // 2026-10-03 04:42:15 UTC
            slot_utc_ns: Some(1_791_002_535 * 1_000_000_000),
            dial_hz: 7_041_000.0,
            freq_hz: 7_042_811.0,
            snr_db: -7.0,
            dt_s: 0.1,
            text: "CQ JO1ZQG/P PM95".into(),
        };
        assert_eq!(
            all_txt_line(&d),
            "261003_044215      7.041 Rx FT8      -7  0.1  1811 CQ JO1ZQG/P PM95"
        );
    }

    #[test]
    fn channelizer_auto_switches_at_the_break_even() {
        let mut cfg = Config::new("x", Vec::new());
        assert_eq!(
            channelizer_for(&cfg, AUTO_PFB_CHANNELS - 1),
            Channelizer::Direct
        );
        assert_eq!(channelizer_for(&cfg, AUTO_PFB_CHANNELS), Channelizer::Pfb);
        cfg.channelizer = Some(Channelizer::Direct);
        assert_eq!(channelizer_for(&cfg, 32), Channelizer::Direct);
    }

    #[test]
    fn civil_dates() {
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(11_016), (2000, 2, 29));
        assert_eq!(civil_from_days(20_729), (2026, 10, 3));
    }
}
