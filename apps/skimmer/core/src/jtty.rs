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
    /// UTC of the end of its latest frame: how far it has got.
    pub end_utc_ns: Option<i64>,
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

/// A standard callsign as WSJT-X's `Radio::is_standard_callsign` has it, with its optional `/R`
/// or `/P` (the same expression: `pskreporter::is_standard_callsign`).
fn is_standard_call(s: &str) -> bool {
    crate::pskreporter::is_standard_callsign(s)
}

/// A four- or six-character Maidenhead locator that is not `RR73` (`Radio::decoded_grid_pattern`).
fn is_grid(s: &str) -> bool {
    crate::pskreporter::is_decoded_grid(s)
}

/// The receive settings for a channel's options: upstream's, with the Rx
/// frequency and tolerance the channel sets.
pub fn params_for(o: &ChannelOptions) -> Params {
    Params {
        f0_hz: o.rx_freq_hz.unwrap_or(DEFAULT_RX_HZ),
        ftol_hz: o.tol_hz.unwrap_or(DEFAULT_FTOL_HZ),
        ..Params::default()
    }
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

/// Hand every JTTY channel's audio since the last call to its thread, run by
/// run with its audio index and UTC (a break in the audio ends a reception
/// there). `workers` is by `ChannelId`. With `keep`, also returns each channel's
/// audio as given (a waterfall draws the same audio the thread reads).
pub(crate) fn feed(
    rx: &mut mfsk_core::iq::IqReceiver,
    workers: &[Option<Worker>],
    keep: bool,
) -> Vec<(usize, Vec<f32>)> {
    let mut kept: Vec<(usize, Vec<f32>)> = Vec::new();
    for (id, w) in workers.iter().enumerate() {
        let Some(w) = w else { continue };
        loop {
            let mut samples = Vec::new();
            let Some(k) = rx.take_audio_from(mfsk_core::iq::ChannelId(id), &mut samples) else {
                break;
            };
            if keep {
                match kept.iter_mut().find(|(i, _)| *i == id) {
                    Some((_, a)) => a.extend_from_slice(&samples),
                    None => kept.push((id, samples.clone())),
                }
            }
            w.send(Job::Audio {
                k,
                utc_ns: rx.utc_of_audio(k),
                samples,
            });
        }
    }
    kept
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
    // The audio index the current reception began at (`None` between
    // receptions), and where the next run should start. A message's key
    // carries the reception's number.
    let mut base_k: Option<u64> = None;
    let mut next_k: Option<u64> = None;
    let mut reception: u64 = 0;
    // The latest audio index with a known UTC. The clock is set a little after
    // the first IQ arrives, so the first run of a reception often has none; a
    // later one does, and the UTC of an earlier index follows from it (12 kHz).
    let mut anchor: Option<(u64, i64)> = None;
    let utc_at = |anchor: Option<(u64, i64)>, k: u64| {
        anchor.map(|(ak, au)| au + (k as i64 - ak as i64) * 1_000_000_000 / 12_000)
    };
    let emit = |u: MessageUpdate, utc0: Option<i64>, reception: u64| {
        let _ = results.send(JttyMessage {
            channel,
            key: reception << 32 | (u.id & 0xffff_ffff),
            start_utc_ns: utc0.map(|t| t + (u.start_s * 1e9).round() as i64),
            end_utc_ns: utc0.map(|t| t + (u.end_s * 1e9).round() as i64),
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
                if let Some(utc) = utc_ns {
                    anchor = Some((k, utc));
                }
                if let Some(b) = base_k
                    && next_k != Some(k)
                {
                    // Not one recording any more: end this reception.
                    let utc0 = utc_at(anchor, b);
                    stream.finish(&mut |u| emit(u, utc0, reception));
                    stream.reset();
                    base_k = None;
                    reception += 1;
                }
                let b = *base_k.get_or_insert(k);
                let utc0 = utc_at(anchor, b);
                pcm.clear();
                agc.process(&samples, &mut pcm);
                stream.push(&pcm, &mut |u| emit(u, utc0, reception));
                next_k = Some(k + samples.len() as u64);
            }
        }
    }
    if let Some(b) = base_k {
        let utc0 = utc_at(anchor, b);
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
            end_utc_ns: Some(1_791_002_537 * 1_000_000_000),
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
        // `Radio::is_standard_callsign` takes `/R` and `/P`: this said it did not until the
        // answers of WSJT-X's own regex were compared (`tests/pskreporter_spot_rules.rs`)
        for c in [
            "K1ABC", "JA1ABC", "W9XYZ", "G4ABC", "3D2AB", "VK3NV", "K1ABC/P", "K1ABC/R",
        ] {
            assert!(is_standard_call(c), "{c}");
        }
        for c in ["CQ", "TEST", "K1ABC/M", "K1ABC/QRP", "1234", "ABCDEFG"] {
            assert!(!is_standard_call(c), "{c}");
        }
        // and `RR73` has a grid's shape but is not one
        assert!(is_grid("PM95") && is_grid("pm95xx") && !is_grid("PM9") && !is_grid("599"));
        assert!(!is_grid("RR73") && !is_grid("rr73"));
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

    /// 1500 Hz audio of a transmission, 12 kHz `f32`, with a second of lead-in
    /// and room to finish, as the receiver's audio channel hands it out.
    fn audio_of(atoms: &[mfsk_core::jtty::source::Atom]) -> Vec<f32> {
        let tones = mfsk_core::jtty::tx::tones(atoms).unwrap();
        let mut a = vec![0.0f32; 12_000];
        a.extend(
            mfsk_core::jtty::tx::synth_f32(&tones, 1500.0, 3000.0)
                .iter()
                .map(|&x| x / 32_768.0),
        );
        a.extend(std::iter::repeat_n(0.0, 4 * 12_000));
        a
    }

    fn worker() -> (Worker, mpsc::Receiver<JttyMessage>) {
        let (tx, rx) = mpsc::channel();
        let w = Worker::spawn(
            3,
            7_078_000.0,
            &ChannelOptions::default(),
            Arc::new(Receiver::new()),
            tx,
        );
        (w, rx)
    }

    /// The thread's whole path: audio runs in, a message grows, completes with its
    /// callsigns, at the dial plus the audio frequency and the UTC of its start.
    #[test]
    fn a_worker_turns_audio_into_messages_with_their_calls() {
        use mfsk_core::jtty::source::{Atom, CallAction};
        let audio = audio_of(&[
            Atom::call(CallAction::Call, "JA1ABC"),
            Atom::call(CallAction::Call, "K1ABC"),
        ]);
        let (w, rx) = worker();
        let utc0 = 1_791_002_535_000_000_000i64;
        let mut k = 0u64;
        for c in audio.chunks(8_000) {
            w.send(Job::Audio {
                k,
                utc_ns: Some(utc0 + (k as i64) * 1_000_000_000 / 12_000),
                samples: c.to_vec(),
            });
            k += c.len() as u64;
        }
        drop(w); // ends the reception and joins
        let all: Vec<JttyMessage> = rx.try_iter().collect();
        let done = all
            .iter()
            .find(|m| m.kind == UpdateKind::Complete)
            .unwrap_or_else(|| panic!("no complete message: {all:?}"));
        assert_eq!(done.channel, 3);
        assert_eq!(done.calls, ["JA1ABC", "K1ABC"], "{:?}", done.text);
        assert!((done.freq_hz - 7_079_500.0).abs() < 3.0, "{}", done.freq_hz);
        // The first frame starts 1 s into the audio.
        let t = done.start_utc_ns.unwrap() - utc0;
        assert!((t - 1_000_000_000).abs() < 150_000_000, "{t}");
        let sender = done.sender();
        assert_eq!(sender.map(|s| s.0), Some("K1ABC".to_string()));
        // Every report of the message has its key.
        assert!(
            all.iter()
                .filter(|m| m.text.contains("JA1ABC"))
                .all(|m| m.key == done.key)
        );
    }

    /// The clock is set a little after the audio starts (the anchor estimate needs
    /// a window of IQ), so a reception's first run has no UTC: its messages still
    /// get one, from the first run that has.
    #[test]
    fn a_message_gets_its_utc_from_a_later_run_when_the_first_has_none() {
        use mfsk_core::jtty::source::{Atom, CallAction};
        let audio = audio_of(&[Atom::call(CallAction::Cq, "K1ABC")]);
        let (w, rx) = worker();
        let utc0 = 1_791_002_535_000_000_000i64;
        let mut k = 0u64;
        for (i, c) in audio.chunks(8_000).enumerate() {
            // The first two runs: no clock yet.
            let utc = (i >= 2).then(|| utc0 + (k as i64) * 1_000_000_000 / 12_000);
            w.send(Job::Audio {
                k,
                utc_ns: utc,
                samples: c.to_vec(),
            });
            k += c.len() as u64;
        }
        drop(w);
        let all: Vec<JttyMessage> = rx.try_iter().collect();
        let done = all
            .iter()
            .find(|m| m.kind == UpdateKind::Complete)
            .unwrap_or_else(|| panic!("{all:?}"));
        let t = done.start_utc_ns.expect("a UTC from a later run") - utc0;
        // The frame starts 1 s into the audio.
        assert!((t - 1_000_000_000).abs() < 150_000_000, "{t}");
    }

    /// A jump in the audio index ends the reception: a message cut off by it is
    /// reported so, and the audio after it starts another with its own keys.
    #[test]
    fn a_break_in_the_audio_ends_the_reception() {
        use mfsk_core::jtty::source::{Atom, CallAction};
        // Two frames; the audio stops after the first and resumes elsewhere.
        let full = audio_of(&[
            Atom::call(CallAction::Cq, "K1ABC"),
            Atom::call(CallAction::Call, "JA1ABC"),
        ]);
        let cut = 12_000 + 1_888 * 12 + 2 * 12_000; // after the first frame
        let (w, rx) = worker();
        w.send(Job::Audio {
            k: 0,
            utc_ns: None,
            samples: full[..cut].to_vec(),
        });
        // The same transmission again, after a gap in the index.
        w.send(Job::Audio {
            k: 10_000_000,
            utc_ns: None,
            samples: full.clone(),
        });
        drop(w);
        let all: Vec<JttyMessage> = rx.try_iter().collect();
        let first = all
            .iter()
            .find(|m| m.text.contains("K1ABC"))
            .expect("first");
        let ended = all.iter().any(|m| m.key == first.key && m.is_final());
        assert!(ended, "the cut message is reported over: {all:?}");
        let keys: std::collections::HashSet<u64> = all.iter().map(|m| m.key).collect();
        assert!(
            keys.len() >= 2,
            "the second reception has other keys: {all:?}"
        );
    }

    /// A JTTY transmission as double-sideband IQ at 48 kS/s: 12 kHz audio
    /// up to 48 kHz by linear interpolation (the channel's filter removes the
    /// images), mixed up by the dial's offset from the centre.
    fn iq_of(audio: &[i16], offset_hz: f64) -> Vec<f32> {
        const FS: usize = 48_000;
        let n = audio.len() * 4;
        let w = std::f64::consts::TAU * offset_hz / FS as f64;
        let mut out = Vec::with_capacity(2 * n);
        let mut peak = 0f32;
        for i in 0..n {
            let x = i as f64 / 4.0;
            let k = x as usize;
            let a = f64::from(audio[k]);
            let b = f64::from(*audio.get(k + 1).unwrap_or(&0));
            let v = a + (b - a) * (x - k as f64);
            let (re, im) = (
                (v * (w * i as f64).cos()) as f32,
                (v * (w * i as f64).sin()) as f32,
            );
            peak = peak.max(re.abs()).max(im.abs());
            out.extend([re, im]);
        }
        for v in &mut out {
            *v = *v * 0.7 / peak;
        }
        out
    }

    /// The skimmer's whole JTTY path short of the network: a transmission as IQ
    /// into an `IqReceiver`, its audio channel's runs to the thread by `feed`,
    /// the message and its callsigns out. Through both channelizers.
    fn through_iq(channelizer: mfsk_core::iq::Channelizer) -> Vec<JttyMessage> {
        use mfsk_core::iq::{IqReceiver, IqSampleFormat, IqStream};
        use mfsk_core::jtty::source::{Atom, CallAction};
        const CENTER: f64 = 7_070_000.0;
        const DIAL: f64 = 7_078_000.0;
        let tones = mfsk_core::jtty::tx::tones(&[
            Atom::call(CallAction::Call, "JA1ABC"),
            Atom::call(CallAction::Call, "K1ABC"),
        ])
        .unwrap();
        let mut audio: Vec<i16> = vec![0; 2 * 12_000];
        audio.extend(
            mfsk_core::jtty::tx::synth_f32(&tones, 1500.0, 3000.0)
                .iter()
                .map(|&x| x as i16),
        );
        audio.extend(std::iter::repeat_n(0, 4 * 12_000));
        let mut iq = iq_of(&audio, DIAL - CENTER);
        // A little noise under it, so the level is the band's, not the signal's.
        let mut x: u32 = 0x1357_9bdf;
        for v in &mut iq {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *v += ((x >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.02;
        }

        let stream = IqStream::new(48_000, CENTER, IqSampleFormat::Cf32);
        let mut rx = IqReceiver::with_channelizer(stream, channelizer).unwrap();
        let id = rx.add_audio_channel(DIAL).unwrap();
        rx.set_time(1_791_002_535_000_000_000, 0);
        let (tx, results) = mpsc::channel();
        let mut workers: Vec<Option<Worker>> = (0..=id.0).map(|_| None).collect();
        workers[id.0] = Some(Worker::spawn(
            7,
            DIAL,
            &ChannelOptions::default(),
            Arc::new(Receiver::new()),
            tx,
        ));
        let mut slots = Vec::new();
        // The stream loop's rhythm: a message of IQ, then the audio it made.
        for c in iq.chunks(2 * 4_096) {
            rx.push_cf32(c, &mut slots);
            feed(&mut rx, &workers, false);
        }
        assert!(slots.is_empty(), "an audio channel cuts no slots");
        drop(workers); // ends the reception and joins
        results.try_iter().collect()
    }

    fn check(all: &[JttyMessage]) {
        let done = all
            .iter()
            .find(|m| m.kind == UpdateKind::Complete)
            .unwrap_or_else(|| panic!("no complete message: {all:?}"));
        assert_eq!(done.channel, 7);
        assert_eq!(done.calls, ["JA1ABC", "K1ABC"], "{:?}", done.text);
        assert_eq!(done.sender().map(|s| s.0), Some("K1ABC".to_string()));
        assert!((done.freq_hz - 7_079_500.0).abs() < 3.0, "{}", done.freq_hz);
        // The audio started at the clock's anchor and the frame 2 s in.
        let t = done.start_utc_ns.unwrap() - 1_791_002_535_000_000_000;
        assert!(
            (t - 2_000_000_000).abs() < 200_000_000,
            "start {t} ns after the anchor"
        );
        assert!(done.snr_db > -20.0 && done.snr_db < 40.0, "{}", done.snr_db);
    }

    #[test]
    fn jtty_through_iq_direct() {
        check(&through_iq(mfsk_core::iq::Channelizer::Direct));
    }

    #[test]
    fn jtty_through_iq_pfb() {
        check(&through_iq(mfsk_core::iq::Channelizer::Pfb));
    }

    /// A hole in the IQ ends the reception mid-message: the half-heard message is
    /// reported cut off (not lost, and not joined to what comes after).
    #[test]
    fn a_gap_in_the_iq_cuts_the_message_off() {
        use mfsk_core::iq::{IqReceiver, IqSampleFormat, IqStream};
        use mfsk_core::jtty::source::{Atom, CallAction};
        const CENTER: f64 = 7_070_000.0;
        const DIAL: f64 = 7_078_000.0;
        let tones = mfsk_core::jtty::tx::tones(&[
            Atom::call(CallAction::Cq, "K1ABC"),
            Atom::call(CallAction::Call, "JA1ABC"),
        ])
        .unwrap();
        let mut audio: Vec<i16> = vec![0; 12_000];
        audio.extend(
            mfsk_core::jtty::tx::synth_f32(&tones, 1500.0, 3000.0)
                .iter()
                .map(|&x| x as i16),
        );
        audio.extend(std::iter::repeat_n(0, 4 * 12_000));
        let iq = iq_of(&audio, DIAL - CENTER);
        let mut rx = IqReceiver::new(IqStream::new(48_000, CENTER, IqSampleFormat::Cf32));
        let id = rx.add_audio_channel(DIAL).unwrap();
        let (tx, results) = mpsc::channel();
        let mut workers: Vec<Option<Worker>> = (0..=id.0).map(|_| None).collect();
        workers[id.0] = Some(Worker::spawn(
            1,
            DIAL,
            &ChannelOptions::default(),
            Arc::new(Receiver::new()),
            tx,
        ));
        let mut slots = Vec::new();
        // Through the first frame and a little of the second, then a lost second.
        let cut = 2 * (48_000 * 3 + 48_000 / 2);
        rx.push_cf32(&iq[..cut], &mut slots);
        feed(&mut rx, &workers, false);
        rx.gap(48_000);
        rx.push_cf32(&iq[cut..], &mut slots);
        feed(&mut rx, &workers, false);
        drop(workers);
        let all: Vec<JttyMessage> = results.try_iter().collect();
        let first = all
            .iter()
            .find(|m| m.text.contains("K1ABC"))
            .unwrap_or_else(|| panic!("the first frame was not heard: {all:?}"));
        assert!(
            all.iter()
                .any(|m| m.key == first.key && m.is_final() && m.kind != UpdateKind::Complete),
            "the half-heard message ends cut off: {all:?}"
        );
    }

    #[test]
    fn all_txt_line_is_upstreams() {
        assert_eq!(
            all_txt_line(&msg("CQ K1ABC FN42", UpdateKind::Complete)),
            "261003_044215     7.078 Rx JTTY     0  0.0 1500 CQ K1ABC FN42"
        );
    }
}
