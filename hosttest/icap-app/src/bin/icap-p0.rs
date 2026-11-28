// SPDX-License-Identifier: GPL-3.0-only
//! `icap-p0 <window|late> <slot.wav>...`: the P0 presets (`CellsOptions::p0_window()` / `p0_late()`) over 12 kHz slots resampled to 8 kHz, in **the board's build** (`fft-extern`, no
//! `fft-rustfft`: no AP rung), for the calibration of `docs/notes/PI3_JT9_GAP.md` section 21 against `pi3_sweep`'s cells flow (`full`: with the blind-CQ AP). Each slot has `<slot>.ref` beside it
//! (one message a line). stdout: `S <slot> <decodes> <in the reference> <ms>` per slot, `D <slot> <message>` per decode, `T <decodes> <hits> <extras> <ms>`.
use icap_app_hosttest::{fft_shim, profile, resample::resample, wav};
use mfsk_core::ft8::decode_block::{CellsOptions, Ft8Rate};
use std::collections::HashSet;

fn main() {
    let mut args = std::env::args().skip(1);
    let which = args.next().expect("window|late");
    let options = match which.as_str() {
        "window" => CellsOptions::p0_window(),
        "late" => CellsOptions::p0_late(),
        other => panic!("unknown preset {other:?} (window, late)"),
    };
    let (mut total, mut hits, mut extras, mut ms) = (0usize, 0usize, 0usize, 0f64);
    for path in args {
        let a = resample(&wav::load_i16(&path), 12_000, 8_000);
        let refs: HashSet<String> = std::fs::read_to_string(path.replace(".wav", ".ref"))
            .map(|t| t.lines().map(profile::norm).collect())
            .unwrap_or_default();
        let run = profile::run_slot_s3_options(&a, Ft8Rate::Hz8000, &options);
        let msgs: HashSet<&String> = run.decodes.iter().map(|d| &d.0).collect();
        let h = msgs.iter().filter(|m| refs.contains(**m)).count();
        let slot = std::path::Path::new(&path)
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_string();
        println!("S {slot} {} {h} {:.0}", msgs.len(), run.ms);
        for m in &msgs {
            println!("D {slot} {m}");
        }
        total += msgs.len();
        hits += h;
        extras += msgs.len() - h;
        ms += run.ms;
    }
    println!("T {total} {hits} {extras} {ms:.0}");
    let _ = fft_shim::factory_calls();
}
