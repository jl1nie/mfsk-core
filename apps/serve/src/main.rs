// SPDX-License-Identifier: GPL-3.0-only
//! `mfsk-serve`: JSON lines on stdin, JSON lines on stdout. See `README.md` and `mfsk_serve`.
//!
//! ```text
//! mfsk-serve                                  serve requests from stdin until it closes
//! mfsk-serve decode FILE [--mode M] [--depth D] [--band LO-HI]     one request, then exit
//! mfsk-serve bench FILE [--mode M] [--iters N]                      time the decoder on FILE
//! ```
//!
//! Any of them takes `--threads N` (or `MFSK_SERVE_THREADS=N`) to cap the decoder's worker threads.

use std::io::{BufRead, Write};
use std::process::ExitCode;

use mfsk_core::registry::Mode;
use mfsk_serve::Server;
use serde_json::json;

fn usage() -> ExitCode {
    eprintln!(
        "usage: mfsk-serve\n\
         \x20      mfsk-serve decode FILE [--mode MODE] [--depth fast|normal|deep] [--band LO-HI]\n\
         \x20      mfsk-serve bench FILE [--mode MODE] [--iters N]\n\
         FILE: .wav (mono, 12 kHz, s16 or f32) or .raw (s16le, mono, 12 kHz); MODE defaults to FT8\n\
         any command: --threads N (or MFSK_SERVE_THREADS=N) caps the worker threads; default: every core"
    );
    ExitCode::from(2)
}

fn emit(lines: &[serde_json::Value]) -> std::io::Result<()> {
    let stdout = std::io::stdout();
    let mut out = stdout.lock();
    for l in lines {
        writeln!(out, "{l}")?;
    }
    out.flush()
}

/// `--threads N` (anywhere on the line) or `MFSK_SERVE_THREADS=N` caps the worker threads the
/// decoder may use (rayon's global pool); without either, rayon's own default applies (every
/// core, or `RAYON_NUM_THREADS`). `N` of 0 means that default too. Returns the arguments without
/// the option, or `Err` for a value that is not a number.
fn take_threads(args: Vec<String>) -> Result<Vec<String>, String> {
    let mut rest = Vec::with_capacity(args.len());
    let mut threads = std::env::var("MFSK_SERVE_THREADS").ok();
    let mut it = args.into_iter();
    while let Some(a) = it.next() {
        if a == "--threads" {
            threads = Some(it.next().ok_or("--threads needs a value")?);
        } else if let Some(v) = a.strip_prefix("--threads=") {
            threads = Some(v.to_string());
        } else {
            rest.push(a);
        }
    }
    if let Some(v) = threads {
        let n: usize = v
            .parse()
            .map_err(|_| format!("threads: not a number: {v:?}"))?;
        if n > 0 {
            // A second call (already built) is ignored: the first setting stands.
            let _ = rayon::ThreadPoolBuilder::new()
                .num_threads(n)
                .build_global();
        }
    }
    Ok(rest)
}

fn main() -> ExitCode {
    let args = match take_threads(std::env::args().skip(1).collect()) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("mfsk-serve: {e}");
            return ExitCode::FAILURE;
        }
    };
    match args.first().map(String::as_str) {
        None => serve(),
        Some("decode") => one_shot(&args[1..], false),
        Some("bench") => one_shot(&args[1..], true),
        Some(_) => usage(),
    }
}

fn serve() -> ExitCode {
    let mut server = Server::new();
    for line in std::io::stdin().lock().lines() {
        let Ok(line) = line else {
            return ExitCode::FAILURE;
        };
        if line.trim().is_empty() {
            continue;
        }
        if emit(&server.handle(&line)).is_err() {
            return ExitCode::FAILURE; // the reader of stdout is gone
        }
    }
    ExitCode::SUCCESS
}

fn one_shot(args: &[String], bench: bool) -> ExitCode {
    let Some(file) = args.first() else {
        return usage();
    };
    let (mut mode, mut depth, mut band, mut iters) = ("FT8".to_string(), None, None, 6usize);
    let mut i = 1;
    while i < args.len() {
        let Some(value) = args.get(i + 1) else {
            return usage();
        };
        match args[i].as_str() {
            "--mode" => mode = value.clone(),
            "--depth" if !bench => depth = Some(value.clone()),
            "--band" if !bench => {
                let Some((lo, hi)) = value.split_once('-') else {
                    return usage();
                };
                let (Ok(lo), Ok(hi)) = (lo.parse::<f32>(), hi.parse::<f32>()) else {
                    return usage();
                };
                band = Some([lo, hi]);
            }
            "--iters" if bench => {
                let Ok(n) = value.parse() else { return usage() };
                iters = n;
            }
            _ => return usage(),
        }
        i += 2;
    }
    if bench {
        let Some(m) = Mode::from_name(&mode) else {
            eprintln!("unknown mode {mode:?}");
            return ExitCode::from(2);
        };
        return match mfsk_serve::bench(file.as_ref(), m, iters, &mut std::io::stdout()) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("{e}");
                ExitCode::FAILURE
            }
        };
    }
    let request =
        json!({ "op": "decode", "mode": mode, "wav": file, "depth": depth, "band": band });
    let lines = Server::new().handle(&request.to_string());
    let failed = lines.iter().any(|l| l["type"] == "error");
    if emit(&lines).is_err() || failed {
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}
