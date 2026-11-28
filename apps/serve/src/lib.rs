// SPDX-License-Identifier: GPL-3.0-only
//! `mfsk-serve`: one resident process that decodes and encodes through `mfsk-core`.
//!
//! A host program that handles one slot at a time can start a process per slot or keep one.
//! This is the second: it holds one [`AnyDecoder`] per mode for as long as it runs, so what a
//! decoder keeps between calls (the callsign hash table, the two latest periods, a period's
//! checkpoint state) is kept, and `learn` / [`Encoded::hashed_calls`](mfsk_core::encoder::Encoded)
//! reach it.
//!
//! The protocol is JSON lines: one request per line in, one or more response lines out, every
//! line one JSON object carrying the request's `id` (whatever JSON value it was, or absent).
//! Requests are served in order, one at a time. A line that cannot be served gets one
//! `{"type":"error"}` line and the next request is served as usual. See `README.md`.

use std::collections::HashMap;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::Path;
use std::time::{Duration, Instant};

use mfsk_core::decoder::{
    AnyDecoder, AnySlotResult, Audio, Depth, Profile, SlotInput, default_params,
};
use mfsk_core::encoder::{TxAudio, encode};
use mfsk_core::registry::Mode;
use serde::Deserialize;
use serde_json::{Value, json};

/// The audio rate every decode request takes (WSJT-X's own).
pub const DECODE_RATE: u32 = 12_000;

/// The highest sample rate an `encode` request may ask for. The library's own limit is memory
/// (`Encoded::synthesize` grows with rate times frame length), so a wild rate is refused here.
pub const MAX_ENCODE_RATE: u32 = 192_000;

/// Default peak of an `encode` request, in i16 counts.
pub const DEFAULT_AMPLITUDE: i16 = 14_000;

#[derive(Deserialize)]
#[serde(tag = "op", rename_all = "lowercase")]
enum Request {
    Decode(DecodeReq),
    Encode(EncodeReq),
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DecodeReq {
    id: Option<Value>,
    mode: String,
    /// A `.wav` (mono, 12 kHz, s16 or f32) or a `.raw` (s16le, mono, 12 kHz).
    wav: String,
    /// `fast`, `normal` or `deep`; the profile's, else the library's default (`deep`), when absent.
    depth: Option<String>,
    /// `p1` or `p2`: a hardware class's depth and AP, and, unless `budget_ms` is given, its share of the period as the budget
    /// (`mfsk_core::decoder::Profile`; the values are extrapolated, not measured on the hardware).
    profile: Option<String>,
    /// `[lo, hi]` in Hz; the mode's default band when absent.
    band: Option<[f32; 2]>,
    /// The period's index on the UTC grid; required with `prefix`.
    period: Option<i64>,
    /// Stop after the candidate in hand once this many milliseconds have passed.
    budget_ms: Option<u64>,
    /// Calls handed to `learn_callsign` before decoding.
    #[serde(default)]
    learn: Vec<String>,
    /// The call is `decode_prefix`: `wav` is the period so far.
    #[serde(default)]
    prefix: bool,
    /// Use only the first this many samples of the file.
    samples: Option<usize>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct EncodeReq {
    id: Option<Value>,
    mode: String,
    text: String,
    f0_hz: f32,
    rate: u32,
    /// Peak of the s16 waveform; [`DEFAULT_AMPLITUDE`] when absent.
    amplitude: Option<i16>,
    /// Where the raw s16le samples are written.
    out: String,
}

/// The decoders of one process, one per mode, and what answers a request line.
#[derive(Default)]
pub struct Server {
    decoders: HashMap<Mode, AnyDecoder>,
}

enum Samples {
    I16(Vec<i16>),
    F32(Vec<f32>),
}

impl Samples {
    fn len(&self) -> usize {
        match self {
            Samples::I16(v) => v.len(),
            Samples::F32(v) => v.len(),
        }
    }

    fn truncate(&mut self, n: usize) {
        match self {
            Samples::I16(v) => v.truncate(n),
            Samples::F32(v) => v.truncate(n),
        }
    }

    fn audio(&self) -> Audio<'_> {
        match self {
            Samples::I16(v) => Audio::I16(v),
            Samples::F32(v) => Audio::F32(v),
        }
    }
}

impl Server {
    /// A server with no decoder yet; each is built on its mode's first request.
    pub fn new() -> Self {
        Self::default()
    }

    /// Answer one request line with its response lines. Never panics: a request that does is
    /// answered with an `error` line and the mode's decoder is dropped (rebuilt, empty, on its
    /// next request), because its state is unknown after a panic.
    pub fn handle(&mut self, line: &str) -> Vec<Value> {
        let value: Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(e) => return vec![error(None, format!("not JSON: {e}"))],
        };
        let id = value.get("id").cloned();
        let request: Request = match serde_json::from_value(value) {
            Ok(r) => r,
            Err(e) => return vec![error(id, format!("bad request: {e}"))],
        };
        let mode_name = match &request {
            Request::Decode(r) => r.mode.clone(),
            Request::Encode(r) => r.mode.clone(),
        };
        let Some(mode) = Mode::from_name(&mode_name) else {
            return vec![error(id, format!("unknown mode {mode_name:?}"))];
        };
        let run = catch_unwind(AssertUnwindSafe(|| match request {
            Request::Decode(r) => self.decode(mode, r),
            Request::Encode(r) => self.encode(mode, r),
        }));
        match run {
            Ok(Ok(lines)) => lines,
            Ok(Err(message)) => vec![error(id, message)],
            Err(_) => {
                self.decoders.remove(&mode);
                vec![error(
                    id,
                    "internal error (a panic); the decoder was reset".into(),
                )]
            }
        }
    }

    fn decoder(&mut self, mode: Mode) -> &mut AnyDecoder {
        self.decoders
            .entry(mode)
            .or_insert_with(|| AnyDecoder::with_defaults(mode))
    }

    fn decode(&mut self, mode: Mode, r: DecodeReq) -> Result<Vec<Value>, String> {
        if r.prefix && r.period.is_none() {
            return Err("`prefix` needs `period`".into());
        }
        let profile = match r.profile.as_deref() {
            None => None,
            Some(name) => Some(
                Profile::from_name(name)
                    .ok_or_else(|| format!("unknown profile {name:?} (p1, p2)"))?,
            ),
        };
        // the mode's defaults, then the profile's, then what the request names
        let mut params = default_params(mode);
        if let Some(p) = profile {
            params = p.apply(params);
        }
        let depth = match r.depth.as_deref().map(str::to_ascii_lowercase).as_deref() {
            None => params.depth,
            Some("fast") => Depth::Fast,
            Some("normal") => Depth::Normal,
            Some("deep") => Depth::Deep,
            Some(other) => return Err(format!("unknown depth {other:?} (fast, normal, deep)")),
        };
        params = params.depth(depth);
        if let Some([lo, hi]) = r.band {
            if !(lo.is_finite() && hi.is_finite() && lo >= 0.0 && lo < hi) {
                return Err(format!("band [{lo}, {hi}] is not a range"));
            }
            params = params.band(lo, hi);
        }
        let mut samples = read_audio(Path::new(&r.wav))?;
        if let Some(n) = r.samples {
            samples.truncate(n);
        }
        if samples.len() == 0 {
            return Err("no samples".into());
        }

        let decoder = self.decoder(mode);
        *decoder.params_mut() = params;
        for call in &r.learn {
            decoder.learn_callsign(call);
        }

        let started = Instant::now();
        // an explicit `budget_ms` wins; else the profile's share of this mode's period
        let budget_ms = r.budget_ms.or_else(|| {
            profile.map(|p| (p.spec().slot_budget * mode.meta().t_slot_s * 1e3) as u64)
        });
        let deadline = budget_ms.map(|ms| started + Duration::from_millis(ms));
        let within = move || deadline.is_none_or(|d| Instant::now() < d);
        let mut slot = SlotInput::new(samples.audio());
        slot.period = r.period;
        if deadline.is_some() {
            slot = slot.budget(&within);
        }
        let result: AnySlotResult = if r.prefix {
            decoder.decode_prefix(&slot)
        } else {
            decoder.decode(&slot)
        };
        let elapsed_ms = started.elapsed().as_secs_f64() * 1e3;

        let mut lines = Vec::with_capacity(result.rows.len() + 1);
        for row in &result.rows {
            let d = &row.decoded;
            lines.push(with_id(
                &r.id,
                json!({
                    "type": "row",
                    "mode": mode.name(),
                    "text": d.text,
                    "freq_hz": d.freq_hz,
                    "dt_sec": d.dt_sec,
                    "snr_db": d.snr_db,
                }),
            ));
        }
        let b = &result.budget;
        lines.push(with_id(
            &r.id,
            json!({
                "type": "done",
                "rows": result.rows.len(),
                "candidates": result.candidates,
                "budget": {
                    "exhausted": b.exhausted,
                    "candidates_skipped": b.candidates_skipped,
                    "stages_run": b.stages_run,
                    "cut_at_sync": b.cut_at_sync,
                },
                "elapsed_ms": elapsed_ms,
            }),
        ));
        Ok(lines)
    }

    fn encode(&mut self, mode: Mode, r: EncodeReq) -> Result<Vec<Value>, String> {
        if r.rate == 0 || r.rate > MAX_ENCODE_RATE {
            return Err(format!("rate {} is outside 1..={MAX_ENCODE_RATE}", r.rate));
        }
        if !(r.f0_hz.is_finite() && r.f0_hz > 0.0) {
            return Err(format!("f0_hz {} is not a frequency", r.f0_hz));
        }
        let encoded = encode(mode, &r.text).map_err(|e| e.to_string())?;
        let pcm = encoded.synthesize_i16(
            &TxAudio::new(r.rate, r.f0_hz),
            r.amplitude.unwrap_or(DEFAULT_AMPLITUDE),
        );
        let bytes: Vec<u8> = pcm.iter().flat_map(|s| s.to_le_bytes()).collect();
        std::fs::write(&r.out, bytes).map_err(|e| format!("cannot write {}: {e}", r.out))?;
        // What the message sends as hashes is what the other station's reply will refer to.
        let decoder = self.decoder(mode);
        for call in encoded.hashed_calls() {
            decoder.learn_callsign(call);
        }
        Ok(vec![with_id(
            &r.id,
            json!({
                "type": "encoded",
                "mode": mode.name(),
                "text": encoded.text(),
                "hashed_calls": encoded.hashed_calls(),
                "samples": pcm.len(),
                "out": r.out,
            }),
        )])
    }
}

fn with_id(id: &Option<Value>, mut v: Value) -> Value {
    if let (Some(id), Some(obj)) = (id, v.as_object_mut()) {
        obj.insert("id".into(), id.clone());
    }
    v
}

fn error(id: Option<Value>, message: String) -> Value {
    with_id(&id, json!({ "type": "error", "error": message }))
}

/// Read a `.raw` (s16le) or `.wav` (PCM s16 or IEEE f32) file, mono at [`DECODE_RATE`].
fn read_audio(path: &Path) -> Result<Samples, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    if path
        .extension()
        .is_some_and(|e| e.eq_ignore_ascii_case("raw"))
    {
        return Ok(Samples::I16(
            bytes
                .as_chunks::<2>()
                .0
                .iter()
                .map(|c| i16::from_le_bytes(*c))
                .collect(),
        ));
    }
    parse_wav(&bytes)
}

fn parse_wav(b: &[u8]) -> Result<Samples, String> {
    if b.len() < 12 || &b[0..4] != b"RIFF" || &b[8..12] != b"WAVE" {
        return Err("not a RIFF/WAVE file".into());
    }
    let u16_at = |i: usize| u16::from_le_bytes([b[i], b[i + 1]]);
    let u32_at = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    let mut fmt: Option<(u16, u16, u32, u16)> = None;
    let mut p = 12;
    while p + 8 <= b.len() {
        let id = &b[p..p + 4];
        let n = u32_at(p + 4) as usize;
        let body = p + 8;
        if id == b"fmt " && body + 16 <= b.len() {
            fmt = Some((
                u16_at(body),
                u16_at(body + 2),
                u32_at(body + 4),
                u16_at(body + 14),
            ));
        } else if id == b"data" {
            let end = (body + n).min(b.len());
            let data = &b[body..end];
            let Some((tag, channels, rate, bits)) = fmt else {
                return Err("`data` before `fmt `".into());
            };
            if channels != 1 {
                return Err(format!("mono expected, the file has {channels} channels"));
            }
            if rate != DECODE_RATE {
                return Err(format!("{DECODE_RATE} Hz expected, the file is {rate} Hz"));
            }
            return match (tag, bits) {
                (1, 16) => Ok(Samples::I16(
                    data.as_chunks::<2>()
                        .0
                        .iter()
                        .map(|c| i16::from_le_bytes(*c))
                        .collect(),
                )),
                (3, 32) => Ok(Samples::F32(
                    data.as_chunks::<4>()
                        .0
                        .iter()
                        .map(|c| f32::from_le_bytes(*c))
                        .collect(),
                )),
                _ => Err(format!(
                    "format tag {tag} with {bits} bits is not s16 PCM or f32"
                )),
            };
        }
        p = body + n + (n & 1);
    }
    Err("no `data` chunk".into())
}

/// Time `mode`'s decoder on one recording, per depth: whole-period calls (the first is cold) and
/// the period fed in at the decoder's checkpoints (`decode_prefix`). One line per call on `out`.
/// Wall-clock figures for a person to read; nothing asserts on them.
pub fn bench(
    path: &Path,
    mode: Mode,
    iters: usize,
    out: &mut dyn std::io::Write,
) -> Result<(), String> {
    let samples = read_audio(path)?;
    let total = samples.len();
    let w = |e: std::io::Error| e.to_string();
    writeln!(
        out,
        "mode={} samples={total} iterations={iters}",
        mode.name()
    )
    .map_err(w)?;
    for depth in [Depth::Fast, Depth::Normal, Depth::Deep] {
        let mut dec = AnyDecoder::new(mode, default_params(mode).depth(depth));
        let points: Vec<usize> = dec.prefix_points().to_vec();
        for i in 0..iters {
            let slot = SlotInput::new(samples.audio());
            let t = Instant::now();
            let r = dec.decode(&slot);
            let ms = t.elapsed().as_secs_f64() * 1e3;
            let cold = if i == 0 { " (cold)" } else { "" };
            writeln!(
                out,
                "depth={depth:?} whole #{i}{cold}: {ms:8.1} ms  rows={} candidates={:?}",
                r.rows.len(),
                r.candidates
            )
            .map_err(w)?;
        }
        let mut dec = AnyDecoder::new(mode, default_params(mode).depth(depth));
        for i in 0..iters {
            let mut line = format!("depth={depth:?} prefix period {i}:");
            for len in points.iter().copied().filter(|&n| n < total).chain([total]) {
                let mut cut = match &samples {
                    Samples::I16(v) => Samples::I16(v[..len].to_vec()),
                    Samples::F32(v) => Samples::F32(v[..len].to_vec()),
                };
                cut.truncate(len);
                let mut slot = SlotInput::new(cut.audio());
                slot.period = Some(i as i64);
                let t = Instant::now();
                let r = dec.decode_prefix(&slot);
                line += &format!(
                    "  @{len}: {:7.1} ms rows={};",
                    t.elapsed().as_secs_f64() * 1e3,
                    r.rows.len()
                );
            }
            writeln!(out, "{line}").map_err(w)?;
        }
    }
    Ok(())
}
