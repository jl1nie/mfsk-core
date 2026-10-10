// SPDX-License-Identifier: GPL-3.0-only
//! JTTY channels (#650). JTTY has no slot: its frames start at any moment and a
//! message grows while it is received, so a JTTY channel is not decoded a slot at
//! a time like the others. The receiver keeps the channel's continuous audio
//! (`IqReceiver::add_audio_channel`); a worker thread per channel runs
//! WSJT-X's receiver over it (`jtty::rx::Stream`, channel 0 at the Rx frequency
//! ±`ftol` plus the side channels at 1350 / 1650 Hz, as upstream) and reports
//! each message as it grows, completes or is given up on.

use std::sync::Arc;
use std::sync::mpsc;

use mfsk_core::jtty::assemble::MessageUpdate;
pub use mfsk_core::jtty::assemble::UpdateKind;
use mfsk_core::jtty::rx::{Params, Receiver, Stream};

use crate::ChannelOptions;

/// Channel 0's centre when the channel sets no Rx frequency: upstream's default.
pub const DEFAULT_RX_HZ: f32 = 1500.0;
/// Channel 0's half-width when the channel sets no tolerance: upstream's default.
pub const DEFAULT_FTOL_HZ: f32 = 50.0;

/// One JTTY message as far as it is known: reported each time it grows, and once
/// more when it completes, is given up on, or the reception ends.
#[derive(Clone, Debug, PartialEq)]
pub struct JttyMessage {
    /// Index into [`crate::Config::channels`].
    pub channel: usize,
    /// Stable for the life of the message and unique on its channel: the row a
    /// display updates in place.
    pub key: u64,
    /// UTC of the start of its first frame, ns, when the clock is known.
    pub start_utc_ns: Option<i64>,
    /// The channel's dial.
    pub dial_hz: f64,
    /// RF frequency of the lowest tone of its latest frame.
    pub freq_hz: f64,
    /// SNR in 2 500 Hz of its first frame, as WSJT-X v3.3 reports it.
    pub snr_db: f32,
    /// The text so far, as upstream displays it (a gap as ` ... `).
    pub text: String,
    /// The callsigns of its call atoms, in order, each once.
    pub calls: Vec<String>,
    /// Why this report: it grew, completed, was given up on, or the reception ended.
    pub kind: UpdateKind,
}

impl JttyMessage {
    /// Whether the message is over: completed, given up on, or cut off.
    pub fn is_final(&self) -> bool {
        self.kind != UpdateKind::Growing
    }

    /// The station that sent it, by the rule WSJT-X spots with
    /// (`JttyReceiveResultController::apply`): a completed message whose first
    /// word is `CQ`, `DE` or a callsign and whose second is a standard callsign
    /// was sent by the second; a grid after it is its locator.
    pub fn sender(&self) -> Option<(String, Option<String>)> {
        if self.kind != UpdateKind::Complete {
            return None;
        }
        let f: Vec<&str> = self.text.split_whitespace().collect();
        if f.len() < 2 {
            return None;
        }
        let first_ok = f[0].eq_ignore_ascii_case("CQ")
            || f[0].eq_ignore_ascii_case("DE")
            || is_standard_call(f[0]);
        if !first_ok || !is_standard_call(f[1]) {
            return None;
        }
        let grid = f
            .get(2)
            .filter(|g| is_grid(g))
            .map(|g| g.to_ascii_uppercase());
        Some((f[1].to_ascii_uppercase(), grid))
    }
}

/// A standard callsign as WSJT-X's `Radio::is_standard_callsign` has it: an
/// optional prefix letter, then a 1-2 character prefix with a digit, then 1-3
/// letters; no portable suffix.
fn is_standard_call(s: &str) -> bool {
    let b = s.as_bytes();
    if !(3..=6).contains(&b.len()) || !b.iter().all(u8::is_ascii_alphanumeric) {
        return false;
    }
    // The area digit is the last digit; letters follow it, 1-3 of them.
    let Some(d) = b.iter().rposition(u8::is_ascii_digit) else {
        return false;
    };
    let tail = &b[d + 1..];
    (1..=2).contains(&d)
        && (1..=3).contains(&tail.len())
        && tail.iter().all(u8::is_ascii_alphabetic)
}

/// A four- or six-character Maidenhead locator.
fn is_grid(s: &str) -> bool {
    let b = s.to_ascii_uppercase().into_bytes();
    let four = b.len() >= 4
        && (b'A'..=b'R').contains(&b[0])
        && (b'A'..=b'R').contains(&b[1])
        && b[2].is_ascii_digit()
        && b[3].is_ascii_digit();
    match b.len() {
        4 => four,
        6 => four && (b'A'..=b'X').contains(&b[4]) && (b'A'..=b'X').contains(&b[5]),
        _ => false,
    }
}

/// The receive settings for a channel's options: upstream's, with the Rx
/// frequency and tolerance the channel sets.
pub fn params_for(o: &ChannelOptions) -> Params {
    let mut p = Params::default();
    p.f0_hz = o.rx_freq_hz.unwrap_or(DEFAULT_RX_HZ);
    p.ftol_hz = o.tol_hz.unwrap_or(DEFAULT_FTOL_HZ);
    p
}

/// The level JTTY's receiver is fed at: the slot channels' (`TARGET_RMS` in the
/// receiver), so a strong signal is not clipped and a weak one is not lost in
/// the `i16` steps.
const TARGET_RMS: f32 = 2_000.0;
/// How slowly the level follows the band: a mean square over about this many
/// samples (5 s). Fast enough for a QSB fade or a gain change, slow enough that
/// one strong frame does not duck the weak ones around it.
const AGC_SAMPLES: f32 = 60_000.0;

/// A slow AGC from the channel's unscaled `f32` audio to `i16`.
#[derive(Default)]
struct Agc {
    mean_square: f32,
}

impl Agc {
    fn process(&mut self, audio: &[f32], out: &mut Vec<i16>) {
        if audio.is_empty() {
            return;
        }
        let ms = audio.iter().map(|v| v * v).sum::<f32>() / audio.len() as f32;
        if !ms.is_finite() {
            return;
        }
        self.mean_square = if self.mean_square <= 0.0 {
            ms
        } else {
            let a = (audio.len() as f32 / AGC_SAMPLES).min(1.0);
            self.mean_square + a * (ms - self.mean_square)
        };
        let g = if self.mean_square > 0.0 {
            TARGET_RMS / self.mean_square.sqrt()
        } else {
            0.0
        };
        out.extend(
            audio
                .iter()
                .map(|&v| (v * g).round().clamp(-32_768.0, 32_767.0) as i16),
        );
    }
}

/// What a JTTY worker is handed.
pub(crate) enum Job {
    /// A run of the channel's audio: its audio index and, with a clock, its UTC.
    Audio {
        k: u64,
        utc_ns: Option<i64>,
        samples: Vec<f32>,
    },
    /// The channel's options changed (its Rx frequency or tolerance).
    Options(ChannelOptions),
}

/// One JTTY channel's receiver thread. Dropping it ends the reception: the
/// messages still open are reported as cut off (`UpdateKind::ReceptionEnded`).
pub(crate) struct Worker {
    tx: Option<mpsc::Sender<Job>>,
    handle: Option<std::thread::JoinHandle<()>>,
}

impl Worker {
    pub(crate) fn spawn(
        channel: usize,
        dial_hz: f64,
        options: &ChannelOptions,
        rx: Arc<Receiver>,
        results: mpsc::Sender<JttyMessage>,
    ) -> Self {
        let (tx, jobs) = mpsc::channel::<Job>();
        let params = params_for(options);
        let handle = std::thread::Builder::new()
            .name(format!("jtty-{channel}"))
            .spawn(move || run(channel, dial_hz, params, rx, jobs, results))
            .expect("spawn JTTY thread");
        Worker {
            tx: Some(tx),
            handle: Some(handle),
        }
    }

    pub(crate) fn send(&self, job: Job) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(job);
        }
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        // Closing the queue ends the thread after what it has in hand.
        drop(self.tx.take());
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

/// The worker's loop: audio runs into the stream, a break between runs (a
/// retune, a gap, the clock re-anchored) ends the reception and starts another.
fn run(
    channel: usize,
    dial_hz: f64,
    params: Params,
    rx: Arc<Receiver>,
    jobs: mpsc::Receiver<Job>,
    results: mpsc::Sender<JttyMessage>,
) {
    let mut stream = Stream::new(rx, params);
    let mut agc = Agc::default();
    let mut pcm = Vec::new();
    // Where the current reception began (audio index, UTC) and where the next
    // run should start; a message's key carries the reception's number.
    let mut origin: Option<Option<i64>> = None;
    let mut next_k: Option<u64> = None;
    let mut reception: u64 = 0;
    let emit = |u: MessageUpdate, utc0: Option<i64>, reception: u64| {
        let _ = results.send(JttyMessage {
            channel,
            key: reception << 32 | (u.id & 0xffff_ffff),
            start_utc_ns: utc0.map(|t| t + (u.start_s * 1e9).round() as i64),
            dial_hz,
            freq_hz: dial_hz + u.f1_hz as f64,
            snr_db: u.snr_db,
            text: u.text,
            calls: u.calls,
            kind: u.kind,
        });
    };
    for job in jobs {
        match job {
            Job::Options(o) => stream.set_params(params_for(&o)),
            Job::Audio { k, utc_ns, samples } => {
                if let Some(utc0) = origin
                    && next_k != Some(k)
                {
                    // Not one recording any more: end this reception.
                    stream.finish(&mut |u| emit(u, utc0, reception));
                    stream.reset();
                    origin = None;
                    reception += 1;
                }
                let utc0 = *origin.get_or_insert(utc_ns);
                pcm.clear();
                agc.process(&samples, &mut pcm);
                stream.push(&pcm, &mut |u| emit(u, utc0, reception));
                next_k = Some(k + samples.len() as u64);
            }
        }
    }
    if let Some(utc0) = origin {
        stream.finish(&mut |u| emit(u, utc0, reception));
    }
}

/// The line upstream's `write_all("Rx", ...)` writes to ALL.TXT for a JTTY
/// message that is over (`mainwindow.cpp`, v3.3.0-beta1): the time, the dial in
/// MHz as `%10.3f ` straight after it, `Rx`, the mode in six, `   0  0.0` where
/// the slot modes put SNR and DT, the audio frequency as `%5d`, the text.
/// Upstream stamps the reception's start; this stamps the message's.
pub fn all_txt_line(m: &JttyMessage) -> String {
    let t = m.start_utc_ns.unwrap_or(0).div_euclid(1_000_000_000);
    let (d, s) = (t.div_euclid(86_400), t.rem_euclid(86_400));
    let (y, mo, da) = crate::civil_from_days(d);
    format!(
        "{:02}{:02}{:02}_{:02}{:02}{:02}{:10.3} Rx {:<6}   0  0.0{:5} {}",
        y % 100,
        mo,
        da,
        s / 3600,
        s / 60 % 60,
        s % 60,
        m.dial_hz / 1e6,
        "JTTY",
        (m.freq_hz - m.dial_hz).round() as i64,
        m.text
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn msg(text: &str, kind: UpdateKind) -> JttyMessage {
        JttyMessage {
            channel: 0,
            key: 1,
            start_utc_ns: Some(1_791_002_535 * 1_000_000_000),
            dial_hz: 7_078_000.0,
            freq_hz: 7_079_500.0,
            snr_db: -7.0,
            text: text.into(),
            calls: Vec::new(),
            kind,
        }
    }

    /// Upstream's spotting rule: the second word sends, when the first is CQ,
    /// DE or a call; the grid after it rides along; only a completed message.
    #[test]
    fn the_sender_is_upstreams_second_word() {
        let s = |t: &str| msg(t, UpdateKind::Complete).sender();
        assert_eq!(
            s("CQ K1ABC FN42"),
            Some(("K1ABC".into(), Some("FN42".into())))
        );
        assert_eq!(s("JA1ABC K1ABC 599"), Some(("K1ABC".into(), None)));
        assert_eq!(s("DE JA1ABC"), Some(("JA1ABC".into(), None)));
        assert_eq!(
            s("HELLO K1ABC"),
            None,
            "first word neither CQ, DE nor a call"
        );
        assert_eq!(s("CQ TEST"), None, "second word not a call");
        assert_eq!(msg("CQ K1ABC", UpdateKind::Growing).sender(), None);
        assert_eq!(msg("CQ K1ABC", UpdateKind::Expired).sender(), None);
    }

    #[test]
    fn standard_calls_and_grids() {
        for c in ["K1ABC", "JA1ABC", "W9XYZ", "G4ABC", "3D2AB", "VK3NV"] {
            assert!(is_standard_call(c), "{c}");
        }
        for c in ["CQ", "TEST", "K1ABC/P", "1234", "ABCDEFG"] {
            assert!(!is_standard_call(c), "{c}");
        }
        assert!(is_grid("PM95") && is_grid("pm95xx") && !is_grid("PM9") && !is_grid("599"));
    }

    /// The level settles on `TARGET_RMS` whatever the input's scale.
    #[test]
    fn the_agc_brings_any_level_to_the_target() {
        for scale in [1e-4f32, 1e-2, 0.5] {
            let mut agc = Agc::default();
            let audio: Vec<f32> = (0..120_000)
                .map(|i| scale * ((i as f32) * 0.3).sin())
                .collect();
            let mut out = Vec::new();
            for c in audio.chunks(4096) {
                agc.process(c, &mut out);
            }
            let tail = &out[out.len() - 12_000..];
            let rms =
                (tail.iter().map(|&v| (v as f32).powi(2)).sum::<f32>() / tail.len() as f32).sqrt();
            assert!((rms - TARGET_RMS).abs() < 50.0, "scale {scale}: rms {rms}");
        }
    }

    #[test]
    fn all_txt_line_is_upstreams() {
        assert_eq!(
            all_txt_line(&msg("CQ K1ABC FN42", UpdateKind::Complete)),
            "261003_044215     7.078 Rx JTTY     0  0.0 1500 CQ K1ABC FN42"
        );
    }
}
