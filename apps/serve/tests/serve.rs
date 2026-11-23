// SPDX-License-Identifier: GPL-3.0-only
//! Tier A for `mfsk-serve`: the protocol, and the state that is the reason for keeping a process.
//!
//! No test reads a clock or sleeps. The audio is the vendored `qso3_busy.wav` and signals the
//! tests synthesise with a fixed-seed noise; a missing fixture is a failure, not a skip.
//! Run with `--release` (the decodes are slow in a debug build):
//! `cargo test -p mfsk-serve --release`.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use mfsk_core::decoder::{AnyDecoder, Depth, default_params};
use mfsk_core::registry::Mode;
use mfsk_serve::Server;
use serde_json::{Value, json};

const QSO3: &str = concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../embedded-poc/assets/qso3_busy.wav"
);

fn fixture() -> &'static str {
    assert!(
        Path::new(QSO3).is_file(),
        "vendored fixture missing: {QSO3}"
    );
    QSO3
}

fn tmp(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("mfsk_serve_{}_{name}", std::process::id()))
}

fn ask(server: &mut Server, request: Value) -> Vec<Value> {
    server.handle(&request.to_string())
}

fn texts(lines: &[Value]) -> BTreeSet<String> {
    lines
        .iter()
        .filter(|l| l["type"] == "row")
        .map(|l| l["text"].as_str().unwrap().to_string())
        .collect()
}

fn done(lines: &[Value]) -> &Value {
    let last = lines.last().expect("a response");
    assert_eq!(last["type"], "done", "last line: {last}");
    last
}

/// The 16-bit samples of the vendored recording, read the way a person would (RIFF chunks).
fn qso3_samples() -> Vec<i16> {
    let b = std::fs::read(fixture()).unwrap();
    let mut p = 12;
    while p + 8 <= b.len() {
        let n = u32::from_le_bytes(b[p + 4..p + 8].try_into().unwrap()) as usize;
        if &b[p..p + 4] == b"data" {
            return b[p + 8..p + 8 + n]
                .as_chunks::<2>()
                .0
                .iter()
                .map(|c| i16::from_le_bytes(*c))
                .collect();
        }
        p += 8 + n + (n & 1);
    }
    panic!("no data chunk");
}

/// One FT8 period: `frame` (what an `encode` request wrote) at the nominal 0.5 s start in
/// fixed-seed noise, as a `.raw` file. A +40 dB signal, so a decode failure is the server's.
fn period_with(frame_raw: &Path, name: &str) -> PathBuf {
    let frame: Vec<i16> = std::fs::read(frame_raw)
        .unwrap()
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| i16::from_le_bytes(*c))
        .collect();
    let mut state = 0x2545_F491_4F6C_DD1Du64; // xorshift; the sum of 12 uniforms is ~N(0, 1)
    let mut gauss = move || {
        let mut a = 0.0f32;
        for _ in 0..12 {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            a += (state >> 40) as f32 / (1u64 << 24) as f32;
        }
        a - 6.0
    };
    let start = 6_000;
    let mut bytes = Vec::with_capacity(360_000);
    for i in 0..180_000usize {
        let s = frame.get(i.wrapping_sub(start)).copied().unwrap_or(0) as f32;
        let v = (s * 0.4 + 20.0 * gauss()).round() as i16;
        bytes.extend_from_slice(&v.to_le_bytes());
    }
    let path = tmp(&format!("{name}.raw"));
    std::fs::write(&path, bytes).unwrap();
    path
}

/// `text` as an FT8 period at `f0_hz`, written by an `encode` request on a server of its own
/// (so the decoder under test learns nothing from the encoding).
fn ft8_period(text: &str, f0_hz: f32, name: &str) -> PathBuf {
    let frame = tmp(&format!("{name}.frame.raw"));
    let r = ask(
        &mut Server::new(),
        json!({"op":"encode","mode":"FT8","text":text,"f0_hz":f0_hz,"rate":12000,"amplitude":15000,
               "out":frame}),
    );
    assert_eq!(r[0]["type"], "encoded", "{r:?}");
    period_with(&frame, name)
}

#[test]
fn a_decode_request_returns_what_the_library_returns() {
    let mut server = Server::new();
    let lines = ask(
        &mut server,
        json!({"op":"decode","id":7,"mode":"FT8","wav":fixture(),"depth":"normal"}),
    );
    assert!(
        lines.iter().all(|l| l["id"] == 7),
        "every line carries the id"
    );
    let d = done(&lines);
    assert_eq!(d["rows"].as_u64().unwrap() as usize, lines.len() - 1);

    let mut direct = AnyDecoder::new(Mode::Ft8, default_params(Mode::Ft8).depth(Depth::Normal));
    let want: BTreeSet<String> = direct
        .decode_i16(&qso3_samples(), None)
        .rows
        .iter()
        .map(|r| r.decoded.text.clone())
        .collect();
    assert!(
        want.len() >= 20,
        "the busy band decodes {} messages",
        want.len()
    );
    assert_eq!(texts(&lines), want);
}

#[test]
fn the_checkpoints_end_with_the_period_the_whole_call_gives() {
    let call = |server: &mut Server, samples: usize| {
        ask(
            server,
            json!({"op":"decode","mode":"FT8","wav":fixture(),"depth":"normal",
                   "period":7,"prefix":true,"samples":samples}),
        )
    };
    let mut server = Server::new();
    let a = call(&mut server, 141_696);
    let b = call(&mut server, 162_432);
    let end = call(&mut server, 180_000);

    let whole = ask(
        &mut Server::new(),
        json!({"op":"decode","mode":"FT8","wav":fixture(),"depth":"normal"}),
    );
    assert!(!texts(&a).is_empty(), "checkpoint A already decodes");
    assert!(texts(&b).is_empty(), "checkpoint B only subtracts");
    assert!(
        texts(&a).is_subset(&texts(&end)),
        "the last call holds the whole period"
    );
    assert_eq!(
        texts(&end),
        texts(&whole),
        "and it is what a whole-period call returns"
    );
}

#[test]
fn a_hashed_call_is_resolved_by_what_the_process_has_learned() {
    // `JA1ABC` is sent as a hash beside a compound call: a receiver that has not heard it
    // prints `<...>`.
    let hashed = ft8_period("<JA1ABC> PJ4/K1ABC RR73", 1500.0, "hashed");
    let decode = |server: &mut Server, extra: Value| {
        let mut req = json!({"op":"decode","mode":"FT8","wav":hashed,"depth":"normal"});
        req.as_object_mut()
            .unwrap()
            .extend(extra.as_object().unwrap().clone());
        texts(&ask(server, req))
    };

    let cold = decode(&mut Server::new(), json!({}));
    assert!(
        cold.contains("<...> PJ4/K1ABC RR73"),
        "a fresh process: {cold:?}"
    );

    let told = decode(&mut Server::new(), json!({"learn": ["JA1ABC"]}));
    assert!(
        told.contains("<JA1ABC> PJ4/K1ABC RR73"),
        "after `learn`: {told:?}"
    );

    // Heard in full in an earlier request of the same process.
    let heard = ft8_period("CQ JA1ABC PM95", 1000.0, "heard");
    let mut server = Server::new();
    let first = texts(&ask(
        &mut server,
        json!({"op":"decode","mode":"FT8","wav":heard,"depth":"normal"}),
    ));
    assert!(first.contains("CQ JA1ABC PM95"), "{first:?}");
    let later = decode(&mut server, json!({}));
    assert!(
        later.contains("<JA1ABC> PJ4/K1ABC RR73"),
        "next request, same process: {later:?}"
    );
}

#[test]
fn what_an_encode_request_sends_as_a_hash_is_learned() {
    let reply = ft8_period("<JA1ABC> PJ4/K1ABC RR73", 1500.0, "reply");
    let mut server = Server::new();
    let sent = ask(
        &mut server,
        json!({"op":"encode","id":"tx","mode":"FT8","text":"CQ JA1ABC PM95","f0_hz":1500,
               "rate":48000,"out":tmp("tx.raw")}),
    );
    assert_eq!(sent[0]["id"], "tx");
    assert_eq!(sent[0]["text"], "CQ JA1ABC PM95");
    assert_eq!(sent[0]["hashed_calls"], json!(["JA1ABC"]));
    // 79 symbols of 0.16 s at 48 kHz.
    assert_eq!(sent[0]["samples"], 79 * 7_680);
    let got = texts(&ask(
        &mut server,
        json!({"op":"decode","mode":"FT8","wav":reply,"depth":"normal"}),
    ));
    assert!(got.contains("<JA1ABC> PJ4/K1ABC RR73"), "{got:?}");
}

#[test]
fn an_encoded_message_decodes_back() {
    let period = ft8_period("CQ JA1ABC PM95", 1503.7, "roundtrip");
    let lines = ask(
        &mut Server::new(),
        json!({"op":"decode","mode":"FT8","wav":period,"depth":"fast"}),
    );
    let row = lines
        .iter()
        .find(|l| l["text"] == "CQ JA1ABC PM95")
        .unwrap_or_else(|| panic!("not decoded: {lines:?}"));
    assert!(
        (row["freq_hz"].as_f64().unwrap() - 1503.7).abs() < 1.0,
        "{row}"
    );
    assert!((row["dt_sec"].as_f64().unwrap()).abs() < 0.05, "{row}");
}

#[test]
fn a_request_that_cannot_be_served_gets_one_error_line_and_the_next_is_served() {
    let mut server = Server::new();
    let bad: [(Value, &str); 7] = [
        (json!("not an object"), "bad request"),
        (
            json!({"op":"decode","id":1,"mode":"NOPE","wav":"x"}),
            "unknown mode",
        ),
        (
            json!({"op":"decode","id":"two","mode":"FT8","wav":"/does/not/exist.wav"}),
            "cannot read",
        ),
        (
            json!({"op":"decode","id":3,"mode":"FT8","wav":fixture(),"prefix":true}),
            "`prefix` needs `period`",
        ),
        (
            json!({"op":"decode","id":4,"mode":"FT8","wav":fixture(),"depth":"warp"}),
            "unknown depth",
        ),
        (
            json!({"op":"decode","id":5,"mode":"FT8","wav":fixture(),"bogus":1}),
            "unknown field",
        ),
        (
            json!({"op":"encode","id":6,"mode":"FT8","text":"CQ JA1ABC PM95","f0_hz":1500,"rate":0,"out":tmp("x")}),
            "rate",
        ),
    ];
    for (request, expect) in bad {
        let lines = server.handle(&request.to_string());
        assert_eq!(lines.len(), 1, "{request}: {lines:?}");
        assert_eq!(lines[0]["type"], "error");
        assert!(
            lines[0]["error"].as_str().unwrap().contains(expect),
            "{expect}: {}",
            lines[0]
        );
        assert_eq!(
            lines[0]["id"],
            request.get("id").cloned().unwrap_or(Value::Null)
        );
    }
    let lines = server.handle("this is not JSON");
    assert_eq!((lines.len(), &lines[0]["type"]), (1, &json!("error")));
    assert!(lines[0].get("id").is_none());

    let ok = ask(
        &mut server,
        json!({"op":"decode","mode":"FT8","wav":fixture(),"depth":"fast"}),
    );
    assert!(
        done(&ok)["rows"].as_u64().unwrap() >= 10,
        "still serving after the errors"
    );
}

#[test]
fn a_file_that_is_not_12_khz_mono_is_refused() {
    let mut wav = Vec::new();
    wav.extend_from_slice(b"RIFF");
    wav.extend_from_slice(&(36u32 + 8).to_le_bytes());
    wav.extend_from_slice(b"WAVEfmt ");
    wav.extend_from_slice(&16u32.to_le_bytes());
    for v in [1u16, 1] {
        wav.extend_from_slice(&v.to_le_bytes()); // PCM, mono
    }
    wav.extend_from_slice(&48_000u32.to_le_bytes()); // the wrong rate
    wav.extend_from_slice(&96_000u32.to_le_bytes());
    for v in [2u16, 16] {
        wav.extend_from_slice(&v.to_le_bytes());
    }
    wav.extend_from_slice(b"data");
    wav.extend_from_slice(&8u32.to_le_bytes());
    wav.extend_from_slice(&[0u8; 8]);
    let path = tmp("48k.wav");
    std::fs::write(&path, wav).unwrap();
    let lines = ask(
        &mut Server::new(),
        json!({"op":"decode","mode":"FT8","wav":path}),
    );
    assert_eq!(lines[0]["type"], "error");
    assert!(
        lines[0]["error"]
            .as_str()
            .unwrap()
            .contains("12000 Hz expected"),
        "{}",
        lines[0]
    );
}

#[test]
fn a_budget_that_is_spent_before_the_call_cuts_the_search() {
    let lines = ask(
        &mut Server::new(),
        json!({"op":"decode","mode":"FT8","wav":fixture(),"depth":"fast","budget_ms":0}),
    );
    let d = done(&lines);
    assert_eq!(d["budget"]["exhausted"], true, "{d}");
    assert!(
        texts(&lines).len() < 14,
        "the search was cut, so fewer than the whole call's rows"
    );
}

#[test]
fn bench_names_every_call_it_times() {
    let mut out = Vec::new();
    mfsk_serve::bench(Path::new(fixture()), Mode::Ft8, 1, &mut out).unwrap();
    let out = String::from_utf8(out).unwrap();
    for depth in ["Fast", "Normal", "Deep"] {
        assert!(
            out.contains(&format!("depth={depth} whole #0 (cold)")),
            "{out}"
        );
        assert!(
            out.contains(&format!("depth={depth} prefix period 0")),
            "{out}"
        );
    }
    assert!(
        out.contains("@141696:") && out.contains("@180000:"),
        "{out}"
    );
}
