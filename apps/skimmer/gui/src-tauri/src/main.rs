// SPDX-License-Identifier: GPL-3.0-or-later
//! Tauri shell for the SpyServer skimmer: `skimmer_core::run_all` on a thread,
//! its events forwarded to the window as `skimmer` events, settings kept as
//! JSON in the app's config directory, decodes appended to an ALL.TXT-style
//! log.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use serde::{Deserialize, Serialize};
use skimmer_core::store;
use skimmer_core::modes::{MODES, frame_geometry, mode_name, parse_mode, slot_seconds};
use skimmer_core::modes::{parse_contest, parse_depth, parse_progress};
use skimmer_core::{
    ApMode, ChannelOptions, ChannelSpec, Channelizer, Config, Contest, Event, LiveOptions,
    QsoContext, QsoProgress, RadioState, Station, WireFormat, all_txt_line,
};
use tauri::{AppHandle, Emitter, Manager, State};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
struct ChannelSetting {
    mode: String,
    dial_hz: f64,
    /// Audio band searched, Hz; both or neither.
    band_lo: Option<f32>,
    band_hi: Option<f32>,
    /// The Rx frequency, its tolerance and the Tx frequency, Hz.
    rx_freq_hz: Option<f32>,
    tol_hz: Option<f32>,
    tx_freq_hz: Option<f32>,
    /// "fast", "normal" or "deep"; empty is the library default.
    depth: Option<String>,
    /// "off", "cq" or "full"; empty is the mode's GUI default.
    ap: Option<String>,
    /// A station to hunt (a-priori hint).
    dx_call: Option<String>,
    /// The QSO in progress: `hiscall`, `hisgrid`, `nQSOProgress` (0-5).
    his_call: Option<String>,
    his_grid: Option<String>,
    progress: Option<String>,
    /// An `ncontest` activity by name; empty is none.
    contest: Option<String>,
    averaging: bool,
    deep_search: bool,
    eme_delay: bool,
    /// This channel's own call and locator; empty uses Settings'.
    my_call: Option<String>,
    my_grid: Option<String>,
    /// Which server (an index into `Settings::servers`) listens to it.
    server: usize,
    /// Its step in that server's rotation (an index into `step_minutes`).
    step: usize,
}

impl ChannelSetting {
    fn options(&self) -> Result<ChannelOptions, String> {
        let name = &self.mode;
        let band_hz = match (self.band_lo, self.band_hi) {
            (Some(lo), Some(hi)) if lo < hi && lo >= 0.0 => Some((lo, hi)),
            (None, None) => None,
            _ => return Err(format!("{name}: band must be LO < HI")),
        };
        let positive = |what: &str, v: Option<f32>| match v {
            Some(x) if !(x.is_finite() && x >= 0.0) => Err(format!("{name}: bad {what}")),
            other => Ok(other),
        };
        let text = |v: &Option<String>| {
            v.as_deref()
                .map(str::trim)
                .filter(|t| !t.is_empty())
                .map(str::to_ascii_uppercase)
        };
        let depth = match self.depth.as_deref() {
            None | Some("") => None,
            Some(d) => Some(parse_depth(d).ok_or_else(|| format!("unknown depth {d:?}"))?),
        };
        let ap = match self.ap.as_deref() {
            None | Some("") => None,
            Some("off") => Some(ApMode::Off),
            Some("cq") => Some(ApMode::CqOnly),
            Some("full") => Some(ApMode::Full),
            Some(o) => return Err(format!("{name}: unknown AP mode {o:?}")),
        };
        let progress = match self.progress.as_deref() {
            None | Some("") => QsoProgress::default(),
            Some(p) => parse_progress(p).ok_or_else(|| format!("{name}: bad progress {p:?}"))?,
        };
        let contest = match self.contest.as_deref() {
            None => Contest::None,
            Some(c) => parse_contest(c).ok_or_else(|| format!("{name}: unknown contest {c:?}"))?,
        };
        Ok(ChannelOptions {
            band_hz,
            rx_freq_hz: positive("Rx frequency", self.rx_freq_hz)?,
            tol_hz: positive("tolerance", self.tol_hz)?,
            tx_freq_hz: positive("Tx frequency", self.tx_freq_hz)?,
            depth,
            ap,
            averaging: self.averaging,
            deep_search: self.deep_search,
            eme_delay: self.eme_delay,
            qso: QsoContext {
                his_call: text(&self.his_call).unwrap_or_default(),
                his_grid: text(&self.his_grid).unwrap_or_default(),
                progress,
            },
            contest,
            dx_call: text(&self.dx_call),
            station: Station {
                call: text(&self.my_call).unwrap_or_default(),
                grid: text(&self.my_grid).unwrap_or_default(),
            },
        })
    }
}

/// One SpyServer.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
struct ServerSetting {
    /// Shown in the window and kept in the database; unique.
    name: String,
    address: String,
    /// Where this server's antenna is (a locator): the origin of the bearings
    /// of what it hears.
    grid: String,
    /// Fixed delay between the SDR and this PC taken off arrival times, ms.
    network_delay_ms: f64,
    /// Hold control and tune the radio even beside an operator's client.
    tune: bool,
    /// Leave control to an SDR# started later, instead of holding it.
    yield_control: bool,
    /// A rotation: minutes of each step. A channel's `step` says which step it
    /// is heard in. One entry or none: no rotation.
    step_minutes: Vec<u32>,
}

impl Default for ServerSetting {
    fn default() -> Self {
        ServerSetting {
            name: "SpyServer".into(),
            address: "127.0.0.1:5555".into(),
            grid: String::new(),
            network_delay_ms: 0.0,
            tune: false,
            yield_control: false,
            step_minutes: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
struct Settings {
    servers: Vec<ServerSetting>,
    /// Every channel of every server, in one list.
    channels: Vec<ChannelSetting>,
    /// From before there were several servers: read, moved into `servers`,
    /// not written back.
    #[serde(skip_serializing)]
    server: String,
    #[serde(skip_serializing)]
    tune: bool,
    #[serde(skip_serializing)]
    yield_control: bool,
    #[serde(skip_serializing)]
    network_delay_ms: f64,
    /// "float" or "int16".
    format: String,
    /// Draw the channels' waterfalls, at 1.5 Hz per bin if `waterfall_fine`.
    waterfall: bool,
    waterfall_fine: bool,
    /// "system" (the PC clock) or "ntp" (the PC clock corrected against `ntp_server`).
    clock_source: String,
    ntp_server: String,
    /// "auto" (filter bank from `AUTO_PFB_CHANNELS` active channels), "direct" or "pfb".
    channelizer: String,
    /// Every decode in a SQLite file (statistics, maps) beside the ALL.TXT.
    db_enabled: bool,
    log_enabled: bool,
    /// Folder the ALL.TXT goes in.
    log_dir: String,
    /// The operator (`mycall`, `mygrid`), for the QSO-context AP.
    my_call: String,
    my_grid: String,
}

impl Settings {
    fn station(&self) -> Station {
        Station {
            call: self.my_call.trim().to_ascii_uppercase(),
            grid: self.my_grid.trim().to_ascii_uppercase(),
        }
    }

    /// A settings file from one server's days becomes a list of one.
    fn migrate(&mut self) {
        if self.servers.is_empty() {
            let mut s = ServerSetting::default();
            if !self.server.trim().is_empty() {
                s.address = self.server.trim().to_string();
            }
            s.tune = self.tune;
            s.yield_control = self.yield_control;
            s.network_delay_ms = self.network_delay_ms;
            self.servers.push(s);
        }
        // Names are the database's key: unique, and never empty.
        let mut seen = std::collections::HashSet::new();
        for (i, s) in self.servers.iter_mut().enumerate() {
            if s.name.trim().is_empty() {
                s.name = if i == 0 { "SpyServer".into() } else { format!("Server {}", i + 1) };
            }
            while !seen.insert(s.name.clone()) {
                s.name = format!("{} ({})", s.name, i + 1);
            }
        }
        let n = self.servers.len();
        for c in &mut self.channels {
            c.server = c.server.min(n - 1);
        }
    }
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            servers: vec![ServerSetting::default()],
            channels: Vec::new(),
            server: String::new(),
            tune: false,
            yield_control: false,
            network_delay_ms: 0.0,
            format: "float".into(),
            waterfall: true,
            waterfall_fine: false,
            clock_source: "ntp".into(),
            ntp_server: "pool.ntp.org".into(),
            channelizer: "auto".into(),
            db_enabled: true,
            // ALL.TXT grows without bound; the database is the record now.
            log_enabled: false,
            log_dir: String::new(),
            my_call: String::new(),
            my_grid: String::new(),
        }
    }
}

/// An event and the server (an index in `Settings::servers`) it came from.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
struct Tagged {
    server: usize,
    #[serde(flatten)]
    event: UiEvent,
}

/// What the window receives, as `{ "type": "...", ...fields }`.
#[derive(Clone, Debug, Serialize)]
#[serde(tag = "type", rename_all = "camelCase", rename_all_fields = "camelCase")]
enum UiEvent {
    Connecting {
        server: String,
    },
    Connected {
        device_kind: u32,
        max_rate: u32,
        bandwidth_hz: f64,
        max_gain: u32,
        gain: u32,
        control: bool,
        device_hz: f64,
    },
    Radio {
        gain: u32,
        max_gain: u32,
        can_control: bool,
    },
    Waterfall {
        channel: usize,
        focus: bool,
        utc_ms: f64,
        f_lo_hz: f32,
        bin_hz: f32,
        levels: Vec<u8>,
    },
    Yielded,
    NoChannelFits {
        device_hz: f64,
    },
    Streaming {
        /// The global index of each channel of this server, in its own order.
        channels: Vec<usize>,
        rate: u32,
        decimation: u32,
        center_hz: f64,
        device_hz: f64,
        active: Vec<bool>,
        channelizer: &'static str,
    },
    Moved {
        device_hz: f64,
        iq_hz: f64,
    },
    Decode {
        channel: usize,
        mode: &'static str,
        /// ms, not ns: a JavaScript number holds it exactly.
        slot_utc_ms: Option<f64>,
        dial_hz: f64,
        freq_hz: f64,
        snr_db: f32,
        dt_s: f32,
        text: String,
    },
    Gap {
        messages: u32,
        at_s: f64,
    },
    Reanchor {
        by_s: f64,
    },
    Clock {
        text: String,
    },
    Step {
        index: usize,
        of: usize,
        ends_utc_s: i64,
    },
    Status {
        streamed_s: f64,
        clock: String,
        delay_ms: f64,
        drift_ms: f64,
        longest_push_ms: f64,
        queued_bytes: usize,
        queued_slots: usize,
        dropped_slots: u64,
        longest_decode_ms: f64,
        gaps: u64,
        reanchors: u64,
    },
    Disconnected {
        error: String,
    },
}

impl UiEvent {
    /// `map` turns the server's own channel numbers into the window's: the
    /// channels of every server are one list there.
    fn new(e: Event, map: &[usize]) -> Self {
        let global = |c: usize| map.get(c).copied().unwrap_or(c);
        match e {
            Event::Connecting { server } => UiEvent::Connecting { server },
            Event::Connected {
                device,
                control,
                device_hz,
            } => UiEvent::Connected {
                device_kind: device.kind,
                max_rate: device.max_rate,
                bandwidth_hz: device.bandwidth_hz,
                max_gain: device.max_gain,
                gain: device.gain,
                control,
                device_hz,
            },
            Event::Radio(r) => UiEvent::Radio {
                gain: r.gain,
                max_gain: r.max_gain,
                can_control: r.can_control,
            },
            Event::Waterfall(w) => UiEvent::Waterfall {
                channel: global(w.channel),
                focus: w.focus,
                utc_ms: w.row.utc_ns as f64 / 1e6,
                f_lo_hz: w.row.f_lo_hz,
                bin_hz: w.row.bin_hz,
                levels: w.row.levels,
            },
            Event::Yielded => UiEvent::Yielded,
            Event::NoChannelFits { device_hz } => UiEvent::NoChannelFits { device_hz },
            Event::Streaming {
                rate,
                decimation,
                center_hz,
                device_hz,
                active,
                channelizer,
            } => UiEvent::Streaming {
                channels: map.to_vec(),
                rate,
                decimation,
                center_hz,
                device_hz,
                active,
                channelizer: match channelizer {
                    Channelizer::Pfb => "filter bank",
                    _ => "direct",
                },
            },
            Event::Moved { device_hz, iq_hz } => UiEvent::Moved { device_hz, iq_hz },
            Event::Decode(d) => UiEvent::Decode {
                channel: global(d.channel),
                mode: mode_name(d.mode),
                slot_utc_ms: d.slot_utc_ns.map(|ns| (ns / 1_000_000) as f64),
                dial_hz: d.dial_hz,
                freq_hz: d.freq_hz,
                snr_db: d.snr_db,
                dt_s: d.dt_s,
                text: d.text,
            },
            Event::Gap { messages, at_s } => UiEvent::Gap { messages, at_s },
            Event::Reanchor { by_s } => UiEvent::Reanchor { by_s },
            Event::Clock(text) => UiEvent::Clock { text },
            Event::Step {
                index,
                of,
                ends_utc_s,
            } => UiEvent::Step {
                index,
                of,
                ends_utc_s,
            },
            Event::Status(s) => UiEvent::Status {
                clock: s.clock,
                streamed_s: s.streamed_s,
                delay_ms: s.delay_ms,
                drift_ms: s.drift_ms,
                longest_push_ms: s.longest_push_ms,
                queued_bytes: s.queued_bytes,
                queued_slots: s.queued_slots,
                dropped_slots: s.dropped_slots,
                longest_decode_ms: s.longest_decode_ms,
                gaps: s.gaps,
                reanchors: s.reanchors,
            },
            Event::Disconnected { error } => UiEvent::Disconnected { error },
        }
    }
}

/// One server of the running skimmer.
struct ServerRun {
    /// Its index in `Settings::servers`.
    server: usize,
    live: Arc<LiveOptions>,
    /// The window's number of each of its channels, in its own order.
    channels: Vec<usize>,
}

struct Running {
    servers: Vec<ServerRun>,
    stop: Arc<AtomicBool>,
    thread: JoinHandle<()>,
}

impl Running {
    /// The server that listens to window channel `global`, and its own number.
    fn channel(&self, global: usize) -> Option<(&ServerRun, usize)> {
        self.servers
            .iter()
            .find_map(|r| r.channels.iter().position(|&g| g == global).map(|l| (r, l)))
    }

    fn server(&self, server: usize) -> Option<&ServerRun> {
        self.servers.iter().find(|r| r.server == server)
    }
}

#[derive(Default)]
struct AppState {
    /// Each server's radio as the running skimmer last reported it (by index in
    /// `Settings::servers`); the window asks.
    radio: Arc<Mutex<Vec<Option<RadioState>>>>,
    running: Mutex<Option<Running>>,
}

fn settings_path(app: &AppHandle) -> Result<PathBuf, String> {
    let dir = app.path().app_config_dir().map_err(|e| e.to_string())?;
    Ok(dir.join("settings.json"))
}

const LOG_FILE: &str = "ALL.TXT";
const DB_FILE: &str = "skimmer.db";
/// Beside ALL.TXT: one line per health event with the host's wall clock, so a
/// long run can be read back (queue, push, decode, drift, drops).
const HEALTH_FILE: &str = "STATUS.log";

fn default_log_dir(app: &AppHandle) -> String {
    app.path()
        .document_dir()
        .or_else(|_| app.path().app_data_dir())
        .map(|d| d.join("mfsk-skimmer"))
        .map(|p| p.to_string_lossy().into_owned())
        .unwrap_or_default()
}

#[tauri::command]
fn load_settings(app: AppHandle) -> Settings {
    let mut s: Settings = settings_path(&app)
        .ok()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|t| serde_json::from_str(&t).ok())
        .unwrap_or_default();
    s.migrate();
    if s.log_dir.is_empty() {
        s.log_dir = default_log_dir(&app);
    }
    s
}

#[tauri::command]
fn save_settings(app: AppHandle, settings: Settings) -> Result<(), String> {
    let path = settings_path(&app)?;
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
    }
    let text = serde_json::to_string_pretty(&settings).map_err(|e| e.to_string())?;
    std::fs::write(path, text).map_err(|e| e.to_string())
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct ModeInfo {
    name: &'static str,
    /// Slot (T/R period); slots start on multiples of it from 00:00 UTC.
    slot_s: f32,
    /// Seconds from the slot start to the first symbol at dt = 0.
    offset_s: f32,
    /// Length of a frame, s.
    frame_s: f32,
    /// Width of a frame on the band, Hz.
    width_hz: f32,
}

/// Active channels from which "auto" picks the filter bank.
#[tauri::command]
fn auto_pfb_channels() -> usize {
    skimmer_core::AUTO_PFB_CHANNELS
}

#[tauri::command]
fn modes() -> Vec<ModeInfo> {
    MODES
        .iter()
        .map(|&(name, m, _)| {
            let (offset_s, frame_s, width_hz) = frame_geometry(m);
            ModeInfo {
                name,
                slot_s: slot_seconds(m),
                offset_s,
                frame_s,
                width_hz,
            }
        })
        .collect()
}

/// A server's configuration and the window's number of each of its channels.
struct Planned {
    /// Index in `Settings::servers`.
    server: usize,
    cfg: Config,
    channels: Vec<usize>,
}

/// One `Config` per server that has channels.
fn configs(s: &Settings) -> Result<Vec<Planned>, String> {
    let mut out = Vec::new();
    for (si, srv) in s.servers.iter().enumerate() {
        let mine: Vec<(usize, &ChannelSetting)> = s
            .channels
            .iter()
            .enumerate()
            .filter(|(_, c)| c.server == si)
            .collect();
        if mine.is_empty() {
            continue;
        }
        let channels = mine
            .iter()
            .map(|(_, c)| {
                let mode =
                    parse_mode(&c.mode).ok_or_else(|| format!("unknown mode {:?}", c.mode))?;
                let mut spec = ChannelSpec::new(mode, c.dial_hz);
                spec.options = c.options()?;
                Ok::<ChannelSpec, String>(spec)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mut cfg = Config::new(srv.address.trim(), channels);
        cfg.name = srv.name.clone();
        cfg.live.set_station(s.station());
        cfg.tune = srv.tune;
        cfg.yield_control = srv.yield_control;
        cfg.waterfall = s.waterfall;
        cfg.ntp = (s.clock_source == "ntp" && !s.ntp_server.trim().is_empty())
            .then(|| s.ntp_server.trim().to_string());
        cfg.live.set_network_delay_ms(srv.network_delay_ms);
        cfg.format = if s.format == "int16" {
            WireFormat::Int16
        } else {
            WireFormat::Float
        };
        cfg.channelizer = match s.channelizer.as_str() {
            "direct" => Some(Channelizer::Direct),
            "pfb" => Some(Channelizer::Pfb),
            _ => None,
        };
        // A rotation: the channels of each step, in turn. Steps nobody is in
        // are left out; one step is no rotation.
        if srv.step_minutes.len() > 1 {
            let last = srv.step_minutes.len() - 1;
            cfg.steps = srv
                .step_minutes
                .iter()
                .enumerate()
                .filter_map(|(k, &minutes)| {
                    let chs: Vec<usize> = mine
                        .iter()
                        .enumerate()
                        .filter(|(_, (_, c))| c.step.min(last) == k)
                        .map(|(local, _)| local)
                        .collect();
                    (!chs.is_empty()).then_some(skimmer_core::Step {
                        channels: chs,
                        minutes,
                    })
                })
                .collect();
            if cfg.steps.len() < 2 {
                cfg.steps.clear();
            }
        }
        out.push(Planned {
            server: si,
            cfg,
            channels: mine.iter().map(|(g, _)| *g).collect(),
        });
    }
    if out.is_empty() {
        return Err("no channels".into());
    }
    Ok(out)
}

fn open_log(dir: &str) -> Result<File, String> {
    let dir = PathBuf::from(dir);
    std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let path = dir.join(LOG_FILE);
    OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
        .map_err(|e| format!("{}: {e}", path.display()))
}

fn open_health(dir: &str) -> Option<File> {
    OpenOptions::new()
        .create(true)
        .append(true)
        .open(PathBuf::from(dir).join(HEALTH_FILE))
        .ok()
}

fn health_line(ev: &Event) -> Option<String> {
    let t = skimmer_core::now_ns() / 1_000_000;
    let body = match ev {
        Event::Status(s) => format!(
            "status streamed {:.0}s delay {:.0}ms drift {:+.0}ms push {:.0}ms decode {:.0}ms \
             queue {:.0}kB slots {}/{} gaps {} reanchors {}",
            s.streamed_s,
            s.delay_ms,
            s.drift_ms,
            s.longest_push_ms,
            s.longest_decode_ms,
            s.queued_bytes as f64 / 1e3,
            s.queued_slots,
            s.dropped_slots,
            s.gaps,
            s.reanchors
        ),
        Event::Gap { messages, at_s } => format!("gap {messages} msg at {at_s:.1}s"),
        Event::Reanchor { by_s } => format!("reanchor {by_s:+.3}s"),
        Event::Clock(text) => format!("clock {text}"),
        Event::Disconnected { error } => format!("disconnected {error}"),
        Event::Connecting { server } => format!("connecting {server}"),
        Event::Connected {
            control,
            device_hz,
            device,
        } => format!(
            "connected control={control} device {device_hz:.0} Hz gain {}/{}",
            device.gain, device.max_gain
        ),
        Event::Radio(r) => format!(
            "radio gain {}/{} control={}",
            r.gain, r.max_gain, r.can_control
        ),
        Event::Yielded => "yielded: got control, leaving it (yield_control is set)".to_string(),
        Event::Streaming { rate, active, .. } => format!("streaming {rate} S/s {active:?}"),
        Event::Moved { device_hz, iq_hz } => format!("moved device {device_hz:.0} iq {iq_hz:.0}"),
        _ => return None,
    };
    Some(format!("{t} {body}"))
}

fn halt(state: &AppState) {
    let running = state.running.lock().unwrap().take();
    if let Some(r) = running {
        r.stop.store(true, Ordering::Relaxed);
        let _ = r.thread.join();
    }
}

/// Change the operator's station in a running skimmer, for every channel.
#[tauri::command]
fn set_station(state: State<'_, AppState>, my_call: String, my_grid: String) {
    if let Some(r) = state.running.lock().unwrap().as_ref() {
        for s in &r.servers {
            s.live.set_station(Station {
                call: my_call.trim().to_ascii_uppercase(),
                grid: my_grid.trim().to_ascii_uppercase(),
            });
        }
    }
}

/// Change a server's fixed network delay in a running skimmer, ms.
#[tauri::command]
fn set_network_delay(state: State<'_, AppState>, server: usize, ms: f64) {
    if let Some(r) = state.running.lock().unwrap().as_ref()
        && let Some(s) = r.server(server)
    {
        s.live.set_network_delay_ms(ms);
    }
}

/// Change a server's device gain index in a running skimmer. Only the client
/// with control can; as a guest the server ignores it, so nothing is sent.
#[tauri::command]
fn set_gain(state: State<'_, AppState>, server: usize, gain: u32) {
    if let Some(r) = state.running.lock().unwrap().as_ref()
        && let Some(s) = r.server(server)
    {
        s.live.set_gain(gain);
    }
    // Show it at once: the server's sync, which corrects this, may not come for
    // a write of ours.
    if let Some(Some(radio)) = state.radio.lock().unwrap().get_mut(server)
        && radio.can_control
    {
        radio.gain = gain;
    }
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct RadioDto {
    gain: u32,
    max_gain: u32,
    can_control: bool,
}

/// A server's radio: its gain, highest index and whether this client may write
/// it, as the running skimmer last heard; `None` when not connected. The
/// window polls this rather than relying on one event at connect.
#[tauri::command]
fn radio_state(state: State<'_, AppState>, server: usize) -> Option<RadioDto> {
    state
        .radio
        .lock()
        .unwrap()
        .get(server)
        .copied()
        .flatten()
        .map(|r| RadioDto {
            gain: r.gain,
            max_gain: r.max_gain,
            can_control: r.can_control,
        })
}

/// Which channel's waterfall is sent whole (the rest send thumbnails), and at
/// which resolution.
#[tauri::command]
fn set_waterfall(state: State<'_, AppState>, focus: Option<usize>, fine: bool) {
    if let Some(r) = state.running.lock().unwrap().as_ref() {
        for s in &r.servers {
            let local = focus.and_then(|g| s.channels.iter().position(|&x| x == g));
            s.live.set_waterfall_focus(local);
            s.live.set_waterfall_fine(fine);
        }
    }
}

/// `MFSK_SKIMMER_AUTOSTART=1` asks for an unattended run that only needs the
/// logs. The window asks once it is listening, so it sees the stream events
/// and its Start button knows the skimmer is running.
#[tauri::command]
fn autostart_requested() -> bool {
    std::env::var_os("MFSK_SKIMMER_AUTOSTART").is_some()
}

/// Change channel `index`'s decode options in a running skimmer; the channel's
/// decoder takes them before its next slot. No-op when nothing is running.
#[tauri::command]
fn set_channel_options(
    state: State<'_, AppState>,
    index: usize,
    channel: ChannelSetting,
) -> Result<(), String> {
    let options = channel.options()?;
    if let Some(r) = state.running.lock().unwrap().as_ref()
        && let Some((s, local)) = r.channel(index)
    {
        s.live.set(local, options);
    }
    Ok(())
}

/// Stops a running skimmer first. Async so the join never blocks the UI thread.
#[tauri::command]
async fn start(app: AppHandle, state: State<'_, AppState>, mut settings: Settings) -> Result<(), String> {
    halt(&state);
    settings.migrate();
    let planned = configs(&settings)?;
    let mut log = if settings.log_enabled {
        Some(open_log(&settings.log_dir)?)
    } else {
        None
    };
    let mut health = if settings.log_enabled || settings.db_enabled {
        open_health(&settings.log_dir)
    } else {
        None
    };
    let mut db = if settings.db_enabled {
        let dir = PathBuf::from(&settings.log_dir);
        std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let path = dir.join(DB_FILE);
        let places: Vec<(String, String)> = settings
            .servers
            .iter()
            .map(|s| (s.name.clone(), s.grid.trim().to_ascii_uppercase()))
            .collect();
        Some(store::Writer::open(&path, &places).map_err(|e| format!("{}: {e}", path.display()))?)
    } else {
        None
    };
    let mut decodes = 0u64;
    let radio = state.radio.clone();
    *radio.lock().unwrap() = vec![None; settings.servers.len()];
    let stop = Arc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let emitter = app.clone();
    let names: Vec<String> = settings.servers.iter().map(|s| s.name.clone()).collect();
    let runs: Vec<ServerRun> = planned
        .iter()
        .map(|p| ServerRun {
            server: p.server,
            live: p.cfg.live.clone(),
            channels: p.channels.clone(),
        })
        .collect();
    let maps: Vec<(usize, Vec<usize>)> = planned.iter().map(|p| (p.server, p.channels.clone())).collect();
    let cfgs: Vec<Config> = planned.into_iter().map(|p| p.cfg).collect();
    let thread = std::thread::spawn(move || {
        skimmer_core::run_all(&cfgs, &flag, |i, ev| {
            let (server, map) = &maps[i];
            let name = &names[*server];
            if matches!(ev, Event::Decode(_)) {
                decodes += 1;
            }
            match &ev {
                Event::Radio(r) => {
                    if let Some(slot) = radio.lock().unwrap().get_mut(*server) {
                        *slot = Some(*r);
                    }
                }
                Event::Disconnected { .. } | Event::Yielded | Event::Connecting { .. } => {
                    if let Some(slot) = radio.lock().unwrap().get_mut(*server) {
                        *slot = None;
                    }
                }
                _ => {}
            }
            if let (Some(line), Some(f)) = (health_line(&ev), health.as_mut()) {
                let extra = if matches!(ev, Event::Status(_)) {
                    format!(" decodes {decodes}")
                } else {
                    String::new()
                };
                let _ = writeln!(f, "{name} {line}{extra}");
            }
            if let Some(w) = db.as_mut() {
                match &ev {
                    Event::Decode(d) => w.push(name, d),
                    // A quiet band must not leave the last decodes unwritten.
                    Event::Status(_) => w.flush(),
                    _ => {}
                }
            }
            if let (Event::Decode(d), Some(f)) = (&ev, log.as_mut())
                && let Err(e) = writeln!(f, "{}", all_txt_line(d))
            {
                let _ = emitter.emit(
                    "skimmer",
                    Tagged {
                        server: *server,
                        event: UiEvent::Disconnected {
                            error: format!("log: {e}"),
                        },
                    },
                );
            }
            let _ = emitter.emit(
                "skimmer",
                Tagged {
                    server: *server,
                    event: UiEvent::new(ev, map),
                },
            );
        });
    });
    *state.running.lock().unwrap() = Some(Running {
        servers: runs,
        stop,
        thread,
    });
    Ok(())
}

fn reader(dir: &str) -> Result<store::Reader, String> {
    let path = PathBuf::from(dir).join(DB_FILE);
    store::Reader::open(&path).map_err(|e| format!("{}: {e}", path.display()))
}

#[tauri::command]
fn db_activity(dir: String, q: store::Query) -> Result<Vec<store::Activity>, String> {
    reader(&dir)?.activity(&q)
}

#[tauri::command]
fn db_stations(dir: String, q: store::Query, limit: usize) -> Result<Vec<store::Station>, String> {
    reader(&dir)?.stations(&q, limit)
}

#[tauri::command]
fn db_decodes(dir: String, q: store::Query, limit: usize) -> Result<Vec<store::Spot>, String> {
    reader(&dir)?.decodes(&q, limit)
}

#[tauri::command]
fn db_points(dir: String, q: store::Query, slice_s: i64) -> Result<Vec<store::Point>, String> {
    reader(&dir)?.points(&q, slice_s, 300_000)
}

#[tauri::command]
fn db_snr_hist(dir: String, q: store::Query) -> Result<Vec<(i64, i64)>, String> {
    reader(&dir)?.snr_histogram(&q)
}

#[tauri::command]
fn db_bands(dir: String) -> Result<Vec<String>, String> {
    reader(&dir)?.bands().map_err(|e| e.to_string())
}

#[tauri::command]
fn db_summary(dir: String, q: store::Query) -> Result<store::Summary, String> {
    reader(&dir)?.summary(&q)
}

#[tauri::command]
fn db_dt(dir: String, bucket_s: i64, since: i64, until: i64) -> Result<Vec<store::DtPoint>, String> {
    reader(&dir)?.dt_median(bucket_s, since, until).map_err(|e| e.to_string())
}

/// First and last decode (UTC seconds) and the number of rows stored.
#[tauri::command]
fn db_span(dir: String) -> Result<(Option<i64>, Option<i64>, i64), String> {
    reader(&dir)?.span().map_err(|e| e.to_string())
}

#[tauri::command]
async fn stop(state: State<'_, AppState>) -> Result<(), String> {
    halt(&state);
    Ok(())
}

fn main() {
    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .manage(AppState::default())
        .invoke_handler(tauri::generate_handler![
            load_settings,
            save_settings,
            modes,
            auto_pfb_channels,
            start,
            stop,
            set_channel_options,
            set_station,
            set_network_delay,
            db_activity,
            db_stations,
            db_decodes,
            db_points,
            db_summary,
            db_bands,
            db_snr_hist,
            db_dt,
            db_span,
            set_gain,
            radio_state,
            set_waterfall,
            autostart_requested
        ])
        .on_window_event(|window, event| {
            if let tauri::WindowEvent::Destroyed = event {
                halt(&window.state::<AppState>());
            }
        })
        .run(tauri::generate_context!())
        .expect("error while running the skimmer");
}
