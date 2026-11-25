// SPDX-License-Identifier: GPL-3.0-only
//! `icap-profile [--rate 8000|12000] [--max-cand N]`: the 22 real recordings through the embedded FT8
//! path at the radio's input rate, against `jt9`. Prints per-file decodes, peak heap and time, then totals.

use std::collections::BTreeMap;

use icap_app_hosttest::{fft_shim, heap, profile, resample::resample, wav};
use mfsk_core::ft8::decode_block::Ft8Rate;

fn main() {
    let (mut rate, mut max_cand, mut bands, mut diag, mut sic) =
        (8_000u32, 15usize, None::<usize>, false, 0usize);
    let mut dump: Option<String> = None;
    let mut native = false;
    // `--reps N`: decode each `--wav` slot N times (the load and the resample once): the difference of two N is the decode's
    // instructions alone
    let mut reps = 1usize;
    let mut stream: Option<usize> = None;
    let mut band: Option<usize> = None;
    let mut band_coarse = false;
    let mut band_sic = 0usize;
    let mut variant: Option<profile::Variant> = None;
    let mut sub = mfsk_core::__internal::ft8_band_bb::SubMode::F64;
    let mut band_ms = [0.0f64; 3];
    let (mut sd_bytes, mut sd_calls, mut sd_restarts, mut sd_max) = (0u64, 0u64, 0u64, 0u64);
    let mut wavs: Vec<String> = Vec::new();
    let a: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < a.len() {
        match a[i].as_str() {
            "--rate" => {
                rate = a[i + 1].parse().expect("--rate");
                i += 2;
            }
            "--reps" => {
                reps = a[i + 1].parse().expect("--reps N");
                i += 2;
            }
            "--native" => {
                native = true;
                i += 1;
            }
            "--low-mem" => {
                bands = Some(a[i + 1].parse().expect("--low-mem"));
                i += 2;
            }
            "--stream" => {
                stream = Some(a[i + 1].parse::<usize>().expect("--stream GROUP"));
                i += 2;
            }
            "--variant" => {
                variant = Some(profile::Variant::parse(&a[i + 1]).expect("--variant v0..v13"));
                i += 2;
            }
            "--band-sub" => {
                use mfsk_core::__internal::ft8_band_bb::SubMode;
                sub = match a[i + 1].as_str() {
                    "f64" => SubMode::F64,
                    "f32" => SubMode::F32,
                    x => panic!("--band-sub f64 | f32, not {x}"),
                };
                i += 2;
            }
            "--band-sic" => {
                band_sic = a[i + 1].parse().expect("--band-sic PASSES");
                i += 2;
            }
            "--band-coarse" => {
                band_coarse = true;
                i += 1;
            }
            "--band" => {
                band = Some(a[i + 1].parse::<usize>().expect("--band GROUP"));
                i += 2;
            }
            "--sic" => {
                sic = a[i + 1].parse().expect("--sic");
                i += 2;
            }
            "--wav" => {
                wavs.push(a[i + 1].clone());
                i += 2;
            }
            "--dump" => {
                dump = Some(a[i + 1].clone());
                i += 2;
            }
            "--diag" => {
                diag = true;
                i += 1;
            }
            "--max-cand" => {
                max_cand = a[i + 1].parse().expect("--max-cand");
                i += 2;
            }
            x => panic!(
                "unknown argument {x} (usage: icap-profile [--rate 8000|12000] [--max-cand N] [--low-mem BANDS] [--native] [--reps N] [--stream GROUP] [--sic PASSES] [--dump FILE] [--wav FILE]... [--diag])"
            ),
        }
    }
    assert!(
        !native || (band.is_none() && band_sic == 0 && sic == 0 && variant.is_none()),
        "--native is the plain low-memory decode: it does not combine with --band, --band-sic, --sic or --variant"
    );
    if !wavs.is_empty() {
        // Per-signal listing of given recordings (12 kHz mono WAV): what each pass found, and for what a later
        // pass found, the strongest decode within 60 Hz that was already there.
        for w in &wavs {
            let slot12 = wav::load_i16(w);
            let slot = if native {
                resample(&slot12, 12_000, 8_000)
            } else if rate == 12_000 {
                slot12
            } else {
                resample(&resample(&slot12, 12_000, rate), rate, 12_000)
            };
            if let Some(v) = variant {
                let r = profile::run_slot_variant(&slot, v, max_cand);
                println!(
                    "\n== {w}  (variant {v:?}, max_cand {max_cand}, {:.0} ms, peak heap {} B)",
                    r.ms, r.peak_heap
                );
                let mut d = r.decodes.clone();
                d.sort_by(|a, b| a.1.total_cmp(&b.1));
                for (m, f) in d {
                    println!("{f:>7.1}  {m}");
                }
                continue;
            }
            if let (Some(group), 0) = (stream, band_sic) {
                let mut s = None;
                for _ in 0..reps {
                    s = Some(profile::run_slot_stream_at(
                        &slot,
                        if native {
                            Ft8Rate::Hz8000
                        } else {
                            Ft8Rate::Hz12000
                        },
                        max_cand,
                        mfsk_core::ft8::decode_block::LowMem::new(bands.unwrap_or(4), group),
                    ));
                }
                let s = s.expect("--reps 0");
                println!(
                    "\n== {w}  (method A, max_cand {max_cand}, {:.0} ms, peak heap {} B, SD {} KB)",
                    s.run.ms,
                    s.run.peak_heap,
                    s.cost.bytes_moved() / 1000
                );
                let mut d = s.run.decodes.clone();
                d.sort_by(|a, b| a.1.total_cmp(&b.1));
                for (m, f) in d {
                    println!("{f:>7.1}  {m}");
                }
                continue;
            }
            if band_sic > 0 {
                let s = profile::run_slot_band_sic_with(
                    &slot,
                    max_cand,
                    bands.unwrap_or(4),
                    band.unwrap_or(4),
                    band_sic,
                    sub,
                );
                println!(
                    "\n== {w}  (method C, max_cand {max_cand}, passes {band_sic}, new per pass {:?}, {:.0} ms of which subtraction {:.0}, peak heap {} B)",
                    s.per_pass, s.run.ms, s.subtract_ms, s.run.peak_heap
                );
                let mut d = s.run.decodes.clone();
                d.sort_by(|a, b| a.1.total_cmp(&b.1));
                for (m, f) in d {
                    println!("{f:>7.1}  {m}");
                }
                continue;
            }
            let s = profile::run_slot_sic(&slot, max_cand, bands.unwrap_or(4), sic.max(1));
            println!(
                "\n== {w}  (max_cand {max_cand}, passes {}, new per pass {:?}, {:.0} ms of which subtraction {:.0})",
                sic.max(1),
                s.per_pass,
                s.run.ms,
                s.subtract_ms
            );
            println!(
                "{:>4} {:>7} {:>6} {:>5}  {:<32} nearest stronger decode of an earlier pass within 60 Hz",
                "pass", "freq", "dt", "snr", "message"
            );
            let mut f: Vec<&profile::Found> = s.found.iter().collect();
            f.sort_by(|a, b| a.freq_hz.total_cmp(&b.freq_hz));
            for d in f {
                let hid = if d.pass == 0 {
                    String::new()
                } else {
                    s.found
                        .iter()
                        .filter(|e| e.pass < d.pass && (e.freq_hz - d.freq_hz).abs() <= 60.0)
                        .max_by(|x, y| x.snr_db.total_cmp(&y.snr_db))
                        .map_or("none (not hidden by a decoded signal)".to_string(), |e| {
                            format!(
                                "{:+.0} Hz away, {:+.0} dB stronger ({})",
                                e.freq_hz - d.freq_hz,
                                e.snr_db - d.snr_db,
                                e.msg
                            )
                        })
                };
                println!(
                    "{:>4} {:>7.1} {:>6.2} {:>5.0}  {:<32} {hid}",
                    d.pass + 1,
                    d.freq_hz,
                    d.dt_sec,
                    d.snr_db,
                    d.msg
                );
            }
        }
        return;
    }
    let mut runs = BTreeMap::new();
    let mut peaks: Vec<(String, usize)> = Vec::new();
    let (mut sic_ms, mut pass_new) = (0.0f64, [0usize; 8]);
    let (mut ms_all, mut peak_max, mut big_at_peak) = (0.0f64, 0usize, Vec::new());
    println!(
        "input rate {rate} Hz (then back to 12 kHz for mfsk-core), max_cand {max_cand}, {}",
        bands.map_or("decode_block".to_string(), |b| format!(
            "decode_block_low_mem, {b} coarse bands"
        ))
    );
    println!(
        "{:<16} {:>7} {:>9} {:>11}",
        "file", "decodes", "ms", "peak heap B"
    );
    for stem in profile::stems() {
        let slot12 = wav::load_i16(profile::asset_path(&stem));
        let slot = if native {
            // the radio's rate, read as it is: one resample from the recording, none back
            assert!(
                rate == 8_000 && bands.is_some(),
                "--native is --rate 8000 with --low-mem"
            );
            resample(&slot12, 12_000, 8_000)
        } else if rate == 12_000 {
            slot12
        } else {
            resample(&resample(&slot12, 12_000, rate), rate, 12_000)
        };
        let r = if native {
            profile::run_slot_at(&slot, Ft8Rate::Hz8000, max_cand, bands)
        } else if let Some(v) = variant {
            profile::run_slot_variant(&slot, v, max_cand)
        } else if let (Some(group), true) = (band, band_sic > 0) {
            let s = profile::run_slot_band_sic_with(
                &slot,
                max_cand,
                bands.unwrap_or(4),
                group,
                band_sic,
                sub,
            );
            sd_bytes += s.cost.bytes_moved();
            sd_calls += s.cost.calls;
            sd_restarts += s.cost.restarts;
            sd_max = sd_max.max(s.cost.bytes_moved());
            sic_ms += s.subtract_ms;
            for (k, n) in s.per_pass.iter().enumerate() {
                pass_new[k.min(7)] += n;
            }
            s.run
        } else if let Some(group) = band {
            let s = profile::run_slot_band(&slot, max_cand, bands.unwrap_or(4), group, band_coarse);
            sd_bytes += s.cost.bytes_moved();
            sd_calls += s.cost.calls;
            sd_restarts += s.cost.restarts;
            sd_max = sd_max.max(s.cost.bytes_moved());
            for k in 0..3 {
                band_ms[k] += s.stage_ms[k];
            }
            s.run
        } else if let Some(group) = stream {
            let s = profile::run_slot_stream(
                &slot,
                max_cand,
                mfsk_core::ft8::decode_block::LowMem::new(bands.unwrap_or(4), group),
            );
            sd_bytes += s.cost.bytes_moved();
            sd_calls += s.cost.calls;
            sd_restarts += s.cost.restarts;
            sd_max = sd_max.max(s.cost.bytes_moved());
            s.run
        } else if sic > 0 {
            let s = profile::run_slot_sic(&slot, max_cand, bands.unwrap_or(4), sic.max(1));
            sic_ms += s.subtract_ms;
            for (k, n) in s.per_pass.iter().enumerate() {
                pass_new[k.min(7)] += n;
            }
            s.run
        } else {
            profile::run_slot(&slot, max_cand, bands)
        };
        peaks.push((stem.clone(), r.peak_heap));
        println!(
            "{stem:<16} {:>7} {:>9.1} {:>11}",
            r.decodes.len(),
            r.ms,
            r.peak_heap
        );
        ms_all += r.ms;
        if r.peak_heap > peak_max {
            peak_max = r.peak_heap;
            big_at_peak = r.big.clone();
        }
        runs.insert(stem, r.decodes);
    }
    if let Some(path) = &dump {
        let mut out = String::new();
        for (stem, ds) in &runs {
            for (m, f) in ds {
                out.push_str(&format!("{stem}\t{m}\t{f:.1}\n"));
            }
        }
        std::fs::write(path, out).expect("--dump");
    }
    let s = profile::score(&runs);
    println!(
        "recall {}/{} = {:.1} %   extras (not in jt9) {}   time {:.0} ms   peak heap above the audio {} B ({:.2} MB)",
        s.hits,
        s.reference,
        100.0 * s.hits as f64 / s.reference as f64,
        s.extras,
        ms_all,
        peak_max,
        peak_max as f64 / 1e6
    );
    println!("allocations >= 64 KiB in the file with the highest peak (bytes x times):");
    for (b, n) in &big_at_peak {
        println!("  {b:>9} x {n}");
    }
    println!(
        "numeric path: {}",
        if cfg!(feature = "fixed-point") {
            "fixed-point (u16 spectrogram, nstep-half)"
        } else {
            "f32"
        }
    );
    if band.is_some() && band_sic == 0 {
        println!(
            "method C stages, total ms: coarse search {:.0}, band baseband {:.0}, candidates {:.0}",
            band_ms[0], band_ms[1], band_ms[2]
        );
    }
    if stream.is_some() || band.is_some() {
        let n = peaks.len().max(1) as u64;
        println!(
            "SD reads per slot (radio cost model; the 12 kHz slot here is 360 KB, 8 kHz would be two thirds of it): mean {} KB, max {} KB, mean {} calls, mean {} restarts",
            sd_bytes / n / 1000,
            sd_max / 1000,
            sd_calls / n,
            sd_restarts / n
        );
    }
    if band_sic > 0 {
        println!(
            "band subtraction: {band_sic} passes, new decodes per pass {:?}, subtract time {sic_ms:.0} ms of the total",
            &pass_new[..band_sic.min(8)]
        );
    }
    if sic > 0 {
        println!(
            "subtraction: {sic} passes, new decodes per pass {:?}, subtract time {sic_ms:.0} ms of the total",
            &pass_new[..sic.min(8)]
        );
    }
    if diag {
        let (stem, _) = peaks.iter().max_by_key(|p| p.1).unwrap().clone();
        let slot12 = wav::load_i16(profile::asset_path(&stem));
        let slot = if rate == 12_000 {
            slot12
        } else {
            resample(&resample(&slot12, 12_000, rate), rate, 12_000)
        };
        heap::diag_start();
        let r = profile::run_slot(&slot, max_cand, bands);
        let (all, late) = heap::diag_report();
        println!(
            "\nheap at the peak of {stem} (meter: {} B above the audio, diagnostics add some), by where it was allocated:",
            r.peak_heap
        );
        for (title, h) in [
            ("at the peak", &all),
            ("highest point after coarse sync", &late),
        ] {
            println!(
                "--- {title}: {} B held by allocations of 16 B or more made during the call",
                h.total
            );
            for (bytes, n, at) in h.groups.iter().take(16) {
                println!("  {bytes:>9} B  x{n:<3} {at}");
            }
        }
    }
    if fft_shim::factory_calls() == 0 {
        eprintln!(
            "WARNING: mfsk-core never asked for the extern FFT planner: it was built with fft-rustfft (run by package, `-p icap-app-hosttest`), so this is the host driver, not the embedded one"
        );
    }
}
