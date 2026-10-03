// SPDX-License-Identifier: GPL-3.0-or-later
//! Tauri shell for the SpyServer skimmer: `skimmer_core::run` on a thread,
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
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
struct Settings {
    server: String,
    channels: Vec<ChannelSetting>,
    /// "float" or "int16".
    format: String,
    tune: bool,
    /// Leave control to an SDR# started later, instead of holding it.
    yield_control: bool,
    /// Draw the channels' waterfalls, at 1.5 Hz per bin if `waterfall_fine`.
    waterfall: bool,
    waterfall_fine: bool,
    /// "auto" (filter bank from `AUTO_PFB_CHANNELS` active channels), "direct" or "pfb".
    channelizer: String,
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
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            server: "127.0.0.1:5555".into(),
            channels: Vec::new(),
            format: "float".into(),
            tune: false,
            yield_control: false,
            waterfall: true,
            waterfall_fine: false,
            channelizer: "auto".into(),
            log_enabled: true,
            log_dir: String::new(),
            my_call: String::new(),
            my_grid: String::new(),
        }
    }
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
    Status {
        streamed_s: f64,
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

impl From<Event> for UiEvent {
    fn from(e: Event) -> Self {
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
                channel: w.channel,
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
                channel: d.channel,
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
            Event::Status(s) => UiEvent::Status {
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

struct Running {
    live: Arc<LiveOptions>,
    stop: Arc<AtomicBool>,
    thread: JoinHandle<()>,
}

#[derive(Default)]
struct AppState {
    /// The radio as the running skimmer last reported it; the window asks.
    radio: Arc<Mutex<Option<RadioState>>>,
    running: Mutex<Option<Running>>,
}

fn settings_path(app: &AppHandle) -> Result<PathBuf, String> {
    let dir = app.path().app_config_dir().map_err(|e| e.to_string())?;
    Ok(dir.join("settings.json"))
}

const LOG_FILE: &str = "ALL.TXT";
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

fn config(s: &Settings) -> Result<Config, String> {
    let channels = s
        .channels
        .iter()
        .map(|c| {
            let mode = parse_mode(&c.mode).ok_or_else(|| format!("unknown mode {:?}", c.mode))?;
            let mut spec = ChannelSpec::new(mode, c.dial_hz);
            spec.options = c.options()?;
            Ok::<ChannelSpec, String>(spec)
        })
        .collect::<Result<Vec<_>, _>>()?;
    if channels.is_empty() {
        return Err("no channels".into());
    }
    let mut cfg = Config::new(s.server.trim(), channels);
    cfg.live.set_station(s.station());
    cfg.tune = s.tune;
    cfg.yield_control = s.yield_control;
    cfg.waterfall = s.waterfall;
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
    Ok(cfg)
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
        r.live.set_station(Station {
            call: my_call.trim().to_ascii_uppercase(),
            grid: my_grid.trim().to_ascii_uppercase(),
        });
    }
}

/// Change the device gain index in a running skimmer. Only the client with
/// control can; as a guest the server ignores it, so nothing is sent.
#[tauri::command]
fn set_gain(state: State<'_, AppState>, gain: u32) {
    if let Some(r) = state.running.lock().unwrap().as_ref() {
        r.live.set_gain(gain);
    }
    // Show it at once: the server's sync, which corrects this, may not come for
    // a write of ours.
    if let Some(radio) = state.radio.lock().unwrap().as_mut()
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

/// The radio's gain, its highest index and whether this client may write it,
/// as the running skimmer last heard; `None` when not connected. The window
/// polls this rather than relying on one event at connect.
#[tauri::command]
fn radio_state(state: State<'_, AppState>) -> Option<RadioDto> {
    state.radio.lock().unwrap().map(|r| RadioDto {
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
        r.live.set_waterfall_focus(focus);
        r.live.set_waterfall_fine(fine);
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
    if let Some(r) = state.running.lock().unwrap().as_ref() {
        r.live.set(index, options);
    }
    Ok(())
}

/// Stops a running skimmer first. Async so the join never blocks the UI thread.
#[tauri::command]
async fn start(app: AppHandle, state: State<'_, AppState>, settings: Settings) -> Result<(), String> {
    halt(&state);
    let cfg = config(&settings)?;
    let mut log = if settings.log_enabled {
        Some(open_log(&settings.log_dir)?)
    } else {
        None
    };
    let mut health = if settings.log_enabled {
        open_health(&settings.log_dir)
    } else {
        None
    };
    let mut decodes = 0u64;
    let live = cfg.live.clone();
    let radio = state.radio.clone();
    *radio.lock().unwrap() = None;
    let stop = Arc::new(AtomicBool::new(false));
    let flag = stop.clone();
    let emitter = app.clone();
    let thread = std::thread::spawn(move || {
        skimmer_core::run(&cfg, &flag, |ev| {
            if matches!(ev, Event::Decode(_)) {
                decodes += 1;
            }
            match &ev {
                Event::Radio(r) => *radio.lock().unwrap() = Some(*r),
                Event::Disconnected { .. } | Event::Yielded | Event::Connecting { .. } => {
                    *radio.lock().unwrap() = None
                }
                _ => {}
            }
            if let (Some(line), Some(f)) = (health_line(&ev), health.as_mut()) {
                let extra = if matches!(ev, Event::Status(_)) {
                    format!(" decodes {decodes}")
                } else {
                    String::new()
                };
                let _ = writeln!(f, "{line}{extra}");
            }
            if let (Event::Decode(d), Some(f)) = (&ev, log.as_mut())
                && let Err(e) = writeln!(f, "{}", all_txt_line(d))
            {
                let _ = emitter.emit(
                    "skimmer",
                    UiEvent::Disconnected {
                        error: format!("log: {e}"),
                    },
                );
            }
            let _ = emitter.emit("skimmer", UiEvent::from(ev));
        });
    });
    *state.running.lock().unwrap() = Some(Running { live, stop, thread });
    Ok(())
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
