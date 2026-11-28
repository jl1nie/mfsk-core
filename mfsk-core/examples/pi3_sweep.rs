// SPDX-License-Identifier: GPL-3.0-only
//! One configuration of the decoder over a list of slots: decodes, wall time, and the decodes themselves, for the Pi 3B study (`docs/notes/PI3_JT9_GAP.md`).
//!
//! ```text
//! cargo run --release -p mfsk-core --features full,fixed-point,internal-testing --example pi3_sweep -- <config> slot.wav...
//! ```
//!
//! `slot.wav` is a 12 kHz mono 16-bit slot with `slot.ref` beside it (one reference message a line). `config`:
//! - `deep:full`, `deep:cq`, `deep:off`, `normal:full|cq|off`, `fast:full|cq|off`: the faithful `Decoder<Ft8>` (`Depth`, `ApMode`);
//! - `cells:p0_window`, `cells:p0_late`, `cells:sic<N>[osd][:c<M>]`: the cells flow at 8 kHz (`fixed-point`): N passes (1 = the plain flow), the faithful OSD at once with `osd`, M candidates (default 30).
//!
//! Threads are the rayon pool's: `RAYON_NUM_THREADS=n`, with `taskset -c 0-(n-1)` to keep the machine's other cores out of it.
//! stdout: `S <slot> <decodes> <in the reference> <ms>` per slot, `D <slot> <message>` per decode, `T <total decodes> <hits> <extras> <ms>`.

use std::collections::HashSet;
use std::time::Instant;

use mfsk_core::decoder::{ApMode, DecodeParams, Decoder, Depth, SlotInput};
use mfsk_core::ft8::Ft8;

/// A reference signal: frequency, DT, SNR, message, and whether the flow missed it.
type Truth = (f32, f32, f32, String, u8);

fn norm(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

fn load(path: &str) -> Vec<i16> {
    let w = std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    w[44..]
        .as_chunks::<2>()
        .0
        .iter()
        .take(180_000)
        .map(|b| i16::from_le_bytes(*b))
        .collect()
}

fn faithful(depth: Depth, ap: ApMode, audio: &[i16]) -> Vec<String> {
    let params = DecodeParams::for_band((200.0, 3000.0)).depth(depth).ap(ap);
    Decoder::<Ft8>::new(params)
        .decode(&SlotInput::i16(audio))
        .rows
        .into_iter()
        .map(|r| norm(&r.decoded.text))
        .collect()
}

/// `p0_window`, `p0_late`, or `sic<N>[osd][:c<M>][:gate]`: N passes (1 = the plain flow), the faithful OSD at once with `osd`, M candidates (the search keeps M; default 30), the pre-BP gate with `gate`.
#[cfg(feature = "fixed-point")]
fn cells(name: &str, audio: &[i16]) -> Vec<String> {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::ft8::decode::DecodeDepth;
    use mfsk_core::ft8::decode_block::{
        CellsOptions, Ft8Rate, GatedSearch, decode_block_cells, decode_block_cells_sic,
        decode_block_cells_with,
    };
    use mfsk_core::msg::wsjt77::unpack77;
    let a = resample_to_8k(audio, 12_000);
    let results = match name {
        "p0_window" => decode_block_cells_with(
            &a,
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            &CellsOptions::p0_window(),
        ),
        "p0_late" => decode_block_cells_with(
            &a,
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            &CellsOptions::p0_late(),
        ),
        other => {
            let mut parts = other.split(':');
            let head = parts.next().unwrap();
            let rest = head
                .strip_prefix("sic")
                .unwrap_or_else(|| panic!("unknown cells config {other}"));
            let osd = rest.ends_with("osd");
            let passes: usize = rest.trim_end_matches("osd").parse().expect("passes");
            let (mut cands, mut gate) = (30usize, false);
            for p in parts {
                if let Some(m) = p.strip_prefix('c') {
                    cands = m.parse().expect("candidates");
                } else if p == "gate" {
                    gate = true;
                }
            }
            assert!(
                !gate,
                "the gate is a CellsOptions field: use p0_window / p0_late or extend this sweep"
            );
            let search = GatedSearch::new(200.0, 3000.0, 1.3, cands);
            let depth = DecodeDepth {
                osd,
                ..DecodeDepth::EMBEDDED
            };
            if passes == 1 {
                decode_block_cells(&a, Ft8Rate::Hz8000, &search, cands, depth)
            } else {
                decode_block_cells_sic(&a, Ft8Rate::Hz8000, &search, cands, depth, passes)
            }
        }
    };
    results
        .expect("8 kHz")
        .iter()
        .filter_map(|d| unpack77(d.message77()))
        .map(|m| norm(&m))
        .collect()
}

/// `cands:<M>`: the cells flow's search at M candidates, printed as `C <slot> <rank> <freq> <dt> <hard sync>` (nothing is decoded): where a reference signal sits in the list.
#[cfg(feature = "fixed-point")]
fn candidates(m: usize, slot: &str, audio: &[i16]) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::ft8::decode_block::{Ft8Rate, GatedSearch, coarse_sync_gated_cells};
    let a = resample_to_8k(audio, 12_000);
    let (cands, _) = coarse_sync_gated_cells(
        &mut SliceSource(&a),
        Ft8Rate::Hz8000,
        &GatedSearch::new(200.0, 3000.0, 1.3, m),
        1,
    )
    .expect("8 kHz");
    for (i, (c, hs)) in cands.iter().enumerate() {
        println!("C {slot} {i} {:.2} {:.3} {hs}", c.freq_hz, c.dt_sec);
    }
}

/// `probe <truth.tsv>`: for each line `slot freq dt snr message missed(0|1)` (a reference signal), decode **at the true place** (`freq`, `dt`) from the cells of its slot, 3 x 3 places round it
/// (+-3.125 Hz, +-0.08 s), with and without the OSD, and print `P <slot> <freq> <dt> <snr> <missed> <true noOSD> <true OSD> <any of 9, OSD> <message>`.
#[cfg(feature = "fixed-point")]
fn probe(truth: &str, dir: &str) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::engine::scalar::Cmplx;
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode::DecodeDepth;
    use mfsk_core::ft8::decode_block::{
        CellTables, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch, coarse_sync_gated_cells,
    };
    use mfsk_core::msg::wsjt77::unpack77;
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables);
    let mut by_slot: std::collections::BTreeMap<String, Vec<Truth>> = Default::default();
    for l in std::fs::read_to_string(truth).unwrap().lines() {
        let f: Vec<&str> = l.splitn(6, '\t').collect();
        by_slot.entry(f[0].to_string()).or_default().push((
            f[1].parse().unwrap(),
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            norm(f[4]),
            f[5].parse().unwrap(),
        ));
    }
    for (slot, sigs) in by_slot {
        let audio = load(&format!("{dir}/{slot}.wav"));
        let a = resample_to_8k(&audio, 12_000);
        let (_, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .expect("8 kHz");
        let try_at = |w: &mut CellsWorker, f: f32, dt: f32, osd: bool, msg: &str| -> bool {
            let depth = DecodeDepth {
                osd,
                ..DecodeDepth::EMBEDDED
            };
            let c = SyncCandidate {
                freq_hz: f,
                dt_sec: dt,
                score: 1.0,
            };
            w.decode_candidate_hs(&cells, &c, 21, depth, 0.25)
                .and_then(|x| unpack77(x.res.message77()))
                .is_some_and(|m| norm(&m) == msg)
        };
        for (f, dt, snr, msg, missed) in sigs {
            let t_no = try_at(&mut w, f, dt, false, &msg);
            let t_osd = try_at(&mut w, f, dt, true, &msg);
            let mut any = t_osd;
            if !any {
                'o: for df in [-3.125f32, 0.0, 3.125] {
                    for ddt in [-0.08f32, 0.0, 0.08] {
                        if try_at(&mut w, f + df, dt + ddt, true, &msg) {
                            any = true;
                            break 'o;
                        }
                    }
                }
            }
            println!(
                "P {slot} {f} {dt} {snr} {missed} {} {} {} {msg}",
                t_no as u8, t_osd as u8, any as u8
            );
        }
    }
}

/// `probe2 <truth.tsv> <dir>`: the **oracle**: for each reference signal that the flow missed, subtract **every other reference signal** (jt9's message, its frequency refined, its DT refined: what the
/// faithful pipeline's own subtraction does) from the 12 kHz audio, then decode the target at its true place and with the whole cells flow (OSD on). Printed as
/// `Q <slot> <freq> <dt> <snr> <true place> <whole flow> <others subtracted> <message>`. If the target decodes here, what stopped it was the others; if not, it is the cells' LLR / the decoder.
#[cfg(feature = "fixed-point")]
fn probe2(truth: &str, dir: &str) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::engine::dsp::subtract::{
        GfskParams, SubtractCfg, refine_freq, subtract_tones_lpf_refine_dt,
    };
    use mfsk_core::engine::scalar::Cmplx;
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::engine::sync::SyncCandidate;
    use mfsk_core::ft8::decode::DecodeDepth;
    use mfsk_core::ft8::decode_block::{
        CellTables, CellsWorker, Ft8BpScratch, Ft8Rate, GatedSearch, coarse_sync_gated_cells,
        decode_block_cells,
    };
    use mfsk_core::msg::wsjt77::{pack77, unpack77};
    let cfg = SubtractCfg {
        sample_rate: 12_000.0,
        tone_spacing_hz: 6.25,
        samples_per_symbol: 1920,
        base_offset_s: 0.5,
        gfsk: Some(GfskParams {
            bt: 2.0,
            hmod: 1.0,
            ramp_samples: 1920 / 8,
            frame_ramp: false,
        }),
    };
    let tables = CellTables::new();
    let mut cs: Box<[[Cmplx<f32>; 8]; 79]> =
        vec![[Cmplx::<f32>::default(); 8]; 79].try_into().unwrap();
    let mut bp = Box::new(Ft8BpScratch::new());
    let mut w = CellsWorker::new(&mut cs, &mut bp, &tables);
    let mut by_slot: std::collections::BTreeMap<String, Vec<Truth>> = Default::default();
    for l in std::fs::read_to_string(truth).unwrap().lines() {
        let f: Vec<&str> = l.splitn(6, '\t').collect();
        by_slot.entry(f[0].to_string()).or_default().push((
            f[1].parse().unwrap(),
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            norm(f[4]),
            f[5].parse().unwrap(),
        ));
    }
    let tones_of = |msg: &str| -> Option<Vec<u8>> {
        let t: Vec<&str> = msg.split(' ').collect();
        let bits = match t.len() {
            3 => pack77(t[0], t[1], t[2]),
            2 => pack77(t[0], t[1], ""),
            _ => None,
        }?;
        Some(mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits))
    };
    for (slot, sigs) in by_slot {
        let audio = load(&format!("{dir}/{slot}.wav"));
        let mut order: Vec<usize> = (0..sigs.len()).collect();
        order.sort_by(|&i, &j| sigs[j].2.partial_cmp(&sigs[i].2).unwrap());
        for (ti, (f, dt, snr, msg, missed)) in sigs.iter().enumerate() {
            if *missed == 0 {
                continue;
            }
            let mut r = audio.clone();
            let mut done = 0;
            for &o in &order {
                if o == ti {
                    continue;
                }
                let (of, odt, _, omsg, _) = &sigs[o];
                if let Some(tones) = tones_of(omsg) {
                    let f0 = refine_freq(&r, &tones, *of, *odt, &cfg, 2.5, 0.1);
                    subtract_tones_lpf_refine_dt(&mut r, &tones, f0, *odt, &cfg, 2000, true);
                    done += 1;
                }
            }
            let a = resample_to_8k(&r, 12_000);
            let (_, cells) = coarse_sync_gated_cells(
                &mut SliceSource(&a),
                Ft8Rate::Hz8000,
                &GatedSearch::new(200.0, 3000.0, 1.3, 30),
                1,
            )
            .expect("8 kHz");
            let depth = DecodeDepth {
                osd: true,
                ..DecodeDepth::EMBEDDED
            };
            let c = SyncCandidate {
                freq_hz: *f,
                dt_sec: *dt,
                score: 1.0,
            };
            let at_true = w
                .decode_candidate_hs(&cells, &c, 21, depth, 0.25)
                .and_then(|x| unpack77(x.res.message77()))
                .is_some_and(|m| norm(&m) == *msg);
            let flow = decode_block_cells(
                &a,
                Ft8Rate::Hz8000,
                &GatedSearch::new(200.0, 3000.0, 1.3, 30),
                30,
                depth,
            )
            .map(|v| {
                v.iter()
                    .filter_map(|d| unpack77(d.message77()))
                    .any(|m| norm(&m) == *msg)
            })
            .unwrap_or(false);
            println!(
                "Q {slot} {f} {dt} {snr} {} {} {done} {msg}",
                at_true as u8, flow as u8
            );
        }
    }
}

/// `probe3 <truth.tsv> <dir>`: the symbol information of a reference signal in two representations, as hard-decision tone errors out of 79 (random guessing is about 69): a DFT of each symbol from the 12 kHz
/// audio at the signal's true frequency and time (`exact`), the same with the time snapped to the cells' 80 ms rows (`snap`), and the cells' (`cells`: the best of the 3 x 3 places round the true one,
/// rows of 80 ms, bins of 3.125 Hz). Also the mean power of the right tone over the mean of the others (dB), both ways. `R <slot> <freq> <dt> <snr> <missed> <exact> <snap> <cells> <dB exact> <dB cells> <message>`.
#[cfg(feature = "fixed-point")]
fn probe3(truth: &str, dir: &str) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::engine::source::SliceSource;
    use mfsk_core::ft8::decode_block::{Ft8Rate, GatedSearch, coarse_sync_gated_cells};
    use mfsk_core::msg::wsjt77::pack77;
    let mut by_slot: std::collections::BTreeMap<String, Vec<Truth>> = Default::default();
    for l in std::fs::read_to_string(truth).unwrap().lines() {
        let f: Vec<&str> = l.splitn(6, '\t').collect();
        by_slot.entry(f[0].to_string()).or_default().push((
            f[1].parse().unwrap(),
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            norm(f[4]),
            f[5].parse().unwrap(),
        ));
    }
    // the 8 tone powers of symbol k at (f, dt) from the 12 kHz audio
    let tone_powers = |audio: &[i16], f: f32, dt: f32, k: usize| -> [f64; 8] {
        let start = ((dt + 0.5) * 12_000.0).round() as i64 + 1920 * k as i64;
        let mut p = [0f64; 8];
        for (t, pt) in p.iter_mut().enumerate() {
            let w = 2.0 * std::f64::consts::PI * (f as f64 + 6.25 * t as f64) / 12_000.0;
            let (mut re, mut im) = (0f64, 0f64);
            for n in 0..1920usize {
                let i = start + n as i64;
                if i >= 0 && (i as usize) < audio.len() {
                    let x = audio[i as usize] as f64;
                    re += x * (w * n as f64).cos();
                    im -= x * (w * n as f64).sin();
                }
            }
            *pt = re * re + im * im;
        }
        p
    };
    for (slot, sigs) in by_slot {
        let audio = load(&format!("{dir}/{slot}.wav"));
        let a = resample_to_8k(&audio, 12_000);
        let (_, cells) = coarse_sync_gated_cells(
            &mut SliceSource(&a),
            Ft8Rate::Hz8000,
            &GatedSearch::new(200.0, 3000.0, 1.3, 30),
            1,
        )
        .expect("8 kHz");
        let all_sigs = sigs.clone();
        for (ti, (f, dt, snr, msg, missed)) in sigs.into_iter().enumerate() {
            let audio = if std::env::var("PROBE_ORACLE").is_ok_and(|v| v == "1") && missed == 1 {
                use mfsk_core::engine::dsp::subtract::{
                    GfskParams, SubtractCfg, refine_freq, subtract_tones_lpf_refine_dt,
                };
                use mfsk_core::msg::wsjt77::pack77;
                let cfg = SubtractCfg {
                    sample_rate: 12_000.0,
                    tone_spacing_hz: 6.25,
                    samples_per_symbol: 1920,
                    base_offset_s: 0.5,
                    gfsk: Some(GfskParams {
                        bt: 2.0,
                        hmod: 1.0,
                        ramp_samples: 1920 / 8,
                        frame_ramp: false,
                    }),
                };
                let mut r = audio.clone();
                let mut order: Vec<usize> = (0..all_sigs.len()).collect();
                order.sort_by(|&i, &j| all_sigs[j].2.partial_cmp(&all_sigs[i].2).unwrap());
                for o in order {
                    if o == ti {
                        continue;
                    }
                    let (of, odt, _, omsg, _) = &all_sigs[o];
                    let t: Vec<&str> = omsg.split(' ').collect();
                    let bits = match t.len() {
                        3 => pack77(t[0], t[1], t[2]),
                        2 => pack77(t[0], t[1], ""),
                        _ => None,
                    };
                    if let Some(bits) = bits {
                        let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
                        let f0 = refine_freq(&r, &tones, *of, *odt, &cfg, 2.5, 0.1);
                        subtract_tones_lpf_refine_dt(&mut r, &tones, f0, *odt, &cfg, 2000, true);
                    }
                }
                r
            } else {
                audio.clone()
            };
            let t: Vec<&str> = msg.split(' ').collect();
            let bits = match t.len() {
                3 => pack77(t[0], t[1], t[2]),
                2 => pack77(t[0], t[1], ""),
                _ => None,
            };
            let Some(bits) = bits else {
                println!("N {slot} {f} {dt} {snr} {missed} no-pack77 {msg}");
                continue;
            };
            let rt = mfsk_core::msg::wsjt77::unpack77(&bits).is_some_and(|m| norm(&m) == msg);
            let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
            let score = |dtq: f32| -> (usize, f64) {
                let (mut err, mut good, mut other) = (0usize, 0f64, 0f64);
                for (k, &tr) in tones.iter().enumerate() {
                    let p = tone_powers(&audio, f, dtq, k);
                    let best = (0..8)
                        .max_by(|&i, &j| p[i].partial_cmp(&p[j]).unwrap())
                        .unwrap();
                    err += (best != tr as usize) as usize;
                    good += p[tr as usize];
                    other += (p.iter().sum::<f64>() - p[tr as usize]) / 7.0;
                }
                (err, 10.0 * (good / other.max(1e-9)).log10())
            };
            let (e_exact, db_exact) = score(dt);
            // PROBE_SEARCH=1: the least errors over a local search round the printed place (+-1.5 Hz in 0.5 Hz, +-0.10 s in 0.02 s): is jt9's rounded place the limit?
            let e_search = if std::env::var("PROBE_SEARCH").is_ok_and(|v| v == "1") {
                let mut best = e_exact;
                for i in -3i32..=3 {
                    for j in -5i32..=5 {
                        let (ff, dd) = (f + 0.5 * i as f32, dt + 0.02 * j as f32);
                        let (mut err, _) = (0usize, 0);
                        for (k, &tr) in tones.iter().enumerate() {
                            let p = tone_powers(&audio, ff, dd, k);
                            let bt = (0..8)
                                .max_by(|&x, &y| p[x].partial_cmp(&p[y]).unwrap())
                                .unwrap();
                            err += (bt != tr as usize) as usize;
                        }
                        best = best.min(err);
                    }
                }
                best
            } else {
                0
            };
            let _ = e_search;
            let (e_snap, _) = score(((dt + 0.5) / 0.08).round() * 0.08 - 0.5);
            // cells: best of the 3 x 3 places; the cell is a half-dB byte (0.5 dB steps of power), so the "ratio" is the mean cell difference
            let (mut e_cells, mut db_cells) = (usize::MAX, f64::MIN);
            let (j0, k0) = (
                ((dt + 0.5) / 0.08).round() as i64,
                (f / 3.125).round() as i64,
            );
            for dr in -1i64..=1 {
                for dc in -1i64..=1 {
                    let (mut err, mut good, mut other) = (0usize, 0f64, 0f64);
                    for (k, &tr) in tones.iter().enumerate() {
                        let row = (j0 + dr + 2 * k as i64) as usize;
                        let v: Vec<f64> = (0..8)
                            .map(|t| {
                                cells
                                    .cell(row, (k0 + dc + 2 * t as i64) as usize)
                                    .unwrap_or(0) as f64
                                    * 0.5
                            })
                            .collect();
                        let best = (0..8)
                            .max_by(|&i, &j| v[i].partial_cmp(&v[j]).unwrap())
                            .unwrap();
                        err += (best != tr as usize) as usize;
                        good += v[tr as usize];
                        other += (v.iter().sum::<f64>() - v[tr as usize]) / 7.0;
                    }
                    if err < e_cells {
                        e_cells = err;
                        db_cells = (good - other) / 79.0;
                    }
                }
            }
            println!(
                "R {slot} {f} {dt} {snr} {missed} {e_exact} {e_snap} {e_cells} {db_exact:.1} {db_cells:.1} {e_search} {} {msg}",
                rt as u8
            );
        }
    }
}

/// `probe4 <list.tsv> <corpus dir>`: tone errors (/79, hard decision of a 12 kHz DFT per symbol at the signal's printed place) on **the audio `jt9` itself had when it decoded the signal**: each line
/// `slot freq dt snr missed message audio` where `audio` is `orig` (the slot) or a `resid_*.bin` of `jt9`'s trace (float32, after the previous pass's subtraction). `U <slot> <missed> <errors> <message>`.
#[cfg(feature = "fixed-point")]
fn probe4(list: &str, dir: &str) {
    use mfsk_core::msg::wsjt77::pack77;
    let tone_powers = |audio: &[i16], f: f32, dt: f32, k: usize| -> [f64; 8] {
        let start = ((dt + 0.5) * 12_000.0).round() as i64 + 1920 * k as i64;
        let mut p = [0f64; 8];
        for (t, pt) in p.iter_mut().enumerate() {
            let w = 2.0 * std::f64::consts::PI * (f as f64 + 6.25 * t as f64) / 12_000.0;
            let (mut re, mut im) = (0f64, 0f64);
            for n in 0..1920usize {
                let i = start + n as i64;
                if i >= 0 && (i as usize) < audio.len() {
                    let x = audio[i as usize] as f64;
                    re += x * (w * n as f64).cos();
                    im -= x * (w * n as f64).sin();
                }
            }
            *pt = re * re + im * im;
        }
        p
    };
    for l in std::fs::read_to_string(list).unwrap().lines() {
        let c: Vec<&str> = l.splitn(7, '\t').collect();
        let (slot, f, dt, missed, msg, src) = (
            c[0],
            c[1].parse::<f32>().unwrap(),
            c[2].parse::<f32>().unwrap(),
            c[4],
            norm(c[5]),
            c[6],
        );
        let audio: Vec<i16> = if src == "orig" {
            load(&format!("{dir}/{slot}.wav"))
        } else {
            let b = std::fs::read(src).unwrap();
            b.as_chunks::<4>()
                .0
                .iter()
                .map(|x| f32::from_le_bytes(*x).round().clamp(-32768.0, 32767.0) as i16)
                .take(180_000)
                .collect()
        };
        let t: Vec<&str> = msg.split(' ').collect();
        let bits = match t.len() {
            3 => pack77(t[0], t[1], t[2]),
            2 => pack77(t[0], t[1], ""),
            _ => None,
        };
        let Some(bits) = bits else { continue };
        let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
        let mut err = 0usize;
        for (k, &tr) in tones.iter().enumerate() {
            let p = tone_powers(&audio, f, dt, k);
            let best = (0..8)
                .max_by(|&i, &j| p[i].partial_cmp(&p[j]).unwrap())
                .unwrap();
            err += (best != tr as usize) as usize;
        }
        println!("U {slot} {missed} {err} {msg}");
    }
}

/// `probe5 <slot.wav> <freq> <dt> <message> [df] [ddt]`: where, round the printed place, a signal's tones are best seen in the 12 kHz audio (the least hard-decision tone errors of 79): the search is
/// +-`df` Hz (0.25 steps) x +-`ddt` s (0.01 s steps). Prints the best few places.
#[cfg(feature = "fixed-point")]
fn probe5(wav: &str, f: f32, dt: f32, msg: &str, df: f32, ddt: f32) {
    use mfsk_core::msg::wsjt77::pack77;
    let audio = load(wav);
    let t: Vec<&str> = msg.split(' ').collect();
    let bits = match t.len() {
        3 => pack77(t[0], t[1], t[2]),
        2 => pack77(t[0], t[1], ""),
        _ => None,
    }
    .expect("pack77");
    let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
    let tone_powers = |ff: f32, dd: f32, k: usize| -> [f64; 8] {
        let start = ((dd + 0.5) * 12_000.0).round() as i64 + 1920 * k as i64;
        let mut p = [0f64; 8];
        for (tt, pt) in p.iter_mut().enumerate() {
            let w = 2.0 * std::f64::consts::PI * (ff as f64 + 6.25 * tt as f64) / 12_000.0;
            let (mut re, mut im) = (0f64, 0f64);
            for n in 0..1920usize {
                let i = start + n as i64;
                if i >= 0 && (i as usize) < audio.len() {
                    let x = audio[i as usize] as f64;
                    re += x * (w * n as f64).cos();
                    im -= x * (w * n as f64).sin();
                }
            }
            *pt = re * re + im * im;
        }
        p
    };
    let mut res: Vec<(usize, f32, f32)> = Vec::new();
    let (nf, nd) = ((df / 0.25) as i32, (ddt / 0.01) as i32);
    for i in -nf..=nf {
        for j in -nd..=nd {
            let (ff, dd) = (f + 0.25 * i as f32, dt + 0.01 * j as f32);
            let mut err = 0usize;
            for (k, &tr) in tones.iter().enumerate() {
                let p = tone_powers(ff, dd, k);
                let b = (0..8)
                    .max_by(|&x, &y| p[x].partial_cmp(&p[y]).unwrap())
                    .unwrap();
                err += (b != tr as usize) as usize;
            }
            res.push((err, ff, dd));
        }
    }
    res.sort_by_key(|r| r.0);
    let at = res
        .iter()
        .find(|r| (r.1 - f).abs() < 0.01 && (r.2 - dt).abs() < 0.001)
        .map(|r| r.0);
    println!("printed place f {f} dt {dt}: errors {:?}", at);
    for r in res.iter().take(6) {
        println!(
            "  best: {} errors at f {:.2} (df {:+.2}) dt {:+.2} (ddt {:+.2})",
            r.0,
            r.1,
            r.1 - f,
            r.2,
            r.2 - dt
        );
    }
}

/// `probe6 <slot.wav> <freq> <dt> <message>`: the same tone errors as `probe5`, but on **`ft8_downsample.f90`'s baseband** (a 192 000-point FFT of the slot, the band `f0 - 1.5 baud .. f0 + 8.5 baud`
/// with 101-bin cosine tapers, shifted to 0 and brought back at 200 Hz: 32 samples a symbol), the symbols then read with a 32-point DFT, over the starts within +-8 samples (5 ms) of the printed place.
#[cfg(feature = "fixed-point")]
fn probe6(wav: &str, f0: f32, dt: f32, msg: &str, resid: Option<&str>) {
    use mfsk_core::msg::wsjt77::pack77;
    use rustfft::FftPlanner;
    use rustfft::num_complex::Complex32;
    let audio: Vec<f32> = match resid {
        Some(r) => std::fs::read(r)
            .unwrap()
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| f32::from_le_bytes(*b))
            .take(180_000)
            .collect(),
        None => load(wav).iter().map(|&x| x as f32).collect(),
    };
    let t: Vec<&str> = msg.split(' ').collect();
    let bits = match t.len() {
        3 => pack77(t[0], t[1], t[2]),
        2 => pack77(t[0], t[1], ""),
        _ => None,
    }
    .expect("pack77");
    let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
    const NFFT1: usize = 192_000;
    const NFFT2: usize = 3200;
    let mut planner = FftPlanner::<f32>::new();
    let mut x: Vec<Complex32> = vec![Complex32::new(0.0, 0.0); NFFT1];
    for (i, &a) in audio.iter().take(180_000).enumerate() {
        x[i] = Complex32::new(a, 0.0);
    }
    planner.plan_fft_forward(NFFT1).process(&mut x);
    let df = 12_000.0 / NFFT1 as f32;
    let baud = 12_000.0 / 1920.0;
    let i0 = (f0 / df).round() as i64;
    let it = (((f0 + 8.5 * baud) / df).round() as usize).min(NFFT1 / 2);
    let ib = (((f0 - 1.5 * baud) / df).round() as i64).max(1) as usize;
    let mut c1 = vec![Complex32::new(0.0, 0.0); NFFT2];
    let mut k = 0usize;
    for (dst, src) in c1.iter_mut().zip(&x[ib..=it]) {
        *dst = *src;
        k += 1;
    }
    let taper: Vec<f32> = (0..=100)
        .map(|i| 0.5 * (1.0 + (i as f32 * std::f32::consts::PI / 100.0).cos()))
        .collect();
    for i in 0..=100 {
        c1[i] *= taper[100 - i];
        c1[k - 1 - 100 + i] *= taper[i];
    }
    // cshift(c1, i0 - ib): element j of the result is c1[(j + shift) mod n]
    let shift = (i0 - ib as i64).rem_euclid(NFFT2 as i64) as usize;
    let rot: Vec<Complex32> = (0..NFFT2).map(|j| c1[(j + shift) % NFFT2]).collect();
    let mut c = rot;
    planner.plan_fft_inverse(NFFT2).process(&mut c);
    let fac = 1.0 / ((NFFT1 * NFFT2) as f32).sqrt();
    for v in c.iter_mut() {
        *v *= fac;
    }
    let mut sym_fft = planner.plan_fft_forward(32);
    let _ = &mut sym_fft;
    let center = ((dt + 0.5) * 200.0).round() as i64;
    let mut best = (usize::MAX, 0i64);
    let mut at_printed = 0usize;
    for off in -8i64..=8 {
        let mut err = 0usize;
        for (kk, &tr) in tones.iter().enumerate() {
            let start = center + off + 32 * kk as i64;
            let mut buf = vec![Complex32::new(0.0, 0.0); 32];
            for (n, b) in buf.iter_mut().enumerate() {
                let i = start + n as i64;
                if i >= 0 && (i as usize) < NFFT2 {
                    *b = c[i as usize];
                }
            }
            sym_fft.process(&mut buf);
            let b = (0..8)
                .max_by(|&x, &y| buf[x].norm_sqr().partial_cmp(&buf[y].norm_sqr()).unwrap())
                .unwrap();
            err += (b != tr as usize) as usize;
        }
        if off == 0 {
            at_printed = err;
        }
        if std::env::var("PROBE_SYMS").is_ok() && (off == 0 || off == 1) {
            let mut line = String::new();
            for kk in 0..79usize {
                let start = center + off + 32 * kk as i64;
                let mut buf = vec![Complex32::new(0.0, 0.0); 32];
                for (n, b) in buf.iter_mut().enumerate() {
                    let i = start + n as i64;
                    if i >= 0 && (i as usize) < NFFT2 {
                        *b = c[i as usize];
                    }
                }
                sym_fft.process(&mut buf);
                let b = (0..8)
                    .max_by(|&x, &y| buf[x].norm_sqr().partial_cmp(&buf[y].norm_sqr()).unwrap())
                    .unwrap();
                line.push_str(&format!("{b}"));
            }
            println!("off {off:+}: argmax {line}");
        }
        if err < best.0 {
            best = (err, off);
        }
    }
    println!(
        "jt9-style baseband: errors at the printed place {at_printed}, best {} at offset {:+} samples ({:+.0} ms)",
        best.0,
        best.1,
        best.1 as f32 * 5.0
    );
}

/// `tones <message>`: the 79 transmitted tones of a message (the encoder's), one line.
fn tones_of(msg: &str) {
    use mfsk_core::msg::wsjt77::pack77;
    let t: Vec<&str> = msg.split(' ').collect();
    let bits = match t.len() {
        3 => pack77(t[0], t[1], t[2]),
        2 => pack77(t[0], t[1], ""),
        _ => None,
    }
    .expect("pack77");
    let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
    println!(
        "{}",
        tones
            .iter()
            .map(|x| x.to_string())
            .collect::<Vec<_>>()
            .join(" ")
    );
}

/// `probe7 <truth.tsv> <dir>`: for every reference signal, the time course of its tones in the 12 kHz audio: at the best of a local search (+-0.5 Hz, +-0.03 s) the hard-decision errors and the power of
/// the right tone over the mean of the other seven (dB) in six runs of symbols. `V <slot> <missed> <snr> <errors of the 6 runs> <dB of the 6 runs> <message>`.
fn probe7(truth: &str, dir: &str) {
    use mfsk_core::msg::wsjt77::pack77;
    let segs = [
        (0usize, 13usize),
        (13, 26),
        (26, 40),
        (40, 53),
        (53, 66),
        (66, 79),
    ];
    let mut by_slot: std::collections::BTreeMap<String, Vec<Truth>> = Default::default();
    for l in std::fs::read_to_string(truth).unwrap().lines() {
        let f: Vec<&str> = l.splitn(6, '\t').collect();
        by_slot.entry(f[0].to_string()).or_default().push((
            f[1].parse().unwrap(),
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            norm(f[4]),
            f[5].parse().unwrap(),
        ));
    }
    for (slot, sigs) in by_slot {
        let audio = load(&format!("{dir}/{slot}.wav"));
        for (f, dt, snr, msg, missed) in sigs {
            let t: Vec<&str> = msg.split(' ').collect();
            let bits = match t.len() {
                3 => pack77(t[0], t[1], t[2]),
                2 => pack77(t[0], t[1], ""),
                _ => None,
            };
            let Some(bits) = bits else { continue };
            let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
            let powers = |ff: f32, dd: f32| -> Vec<[f64; 8]> {
                (0..79)
                    .map(|k| {
                        let start = ((dd + 0.5) * 12_000.0).round() as i64 + 1920 * k as i64;
                        let mut p = [0f64; 8];
                        for (tt, pt) in p.iter_mut().enumerate() {
                            let w = 2.0 * std::f64::consts::PI * (ff as f64 + 6.25 * tt as f64)
                                / 12_000.0;
                            let (mut re, mut im) = (0f64, 0f64);
                            for n in 0..1920usize {
                                let i = start + n as i64;
                                if i >= 0 && (i as usize) < audio.len() {
                                    let x = audio[i as usize] as f64;
                                    re += x * (w * n as f64).cos();
                                    im -= x * (w * n as f64).sin();
                                }
                            }
                            *pt = re * re + im * im;
                        }
                        p
                    })
                    .collect()
            };
            let errs_of = |p: &Vec<[f64; 8]>| -> usize {
                p.iter()
                    .zip(tones.iter())
                    .filter(|(pk, tr)| {
                        (0..8)
                            .max_by(|&x, &y| pk[x].partial_cmp(&pk[y]).unwrap())
                            .unwrap()
                            != **tr as usize
                    })
                    .count()
            };
            let mut best: Option<(usize, Vec<[f64; 8]>)> = None;
            let mut best_at = (f, dt);
            for i in -2i32..=2 {
                for j in -3i32..=3 {
                    let p = powers(f + 0.25 * i as f32, dt + 0.01 * j as f32);
                    let e = errs_of(&p);
                    if best.as_ref().is_none_or(|b| e < b.0) {
                        best = Some((e, p));
                        best_at = (f + 0.25 * i as f32, dt + 0.01 * j as f32);
                    }
                }
            }
            let p = best.unwrap().1;
            let mut es = Vec::new();
            let mut dbs = Vec::new();
            for &(lo, hi) in &segs {
                let mut e = 0usize;
                let (mut good, mut other) = (0f64, 0f64);
                for k in lo..hi {
                    let b = (0..8)
                        .max_by(|&x, &y| p[k][x].partial_cmp(&p[k][y]).unwrap())
                        .unwrap();
                    e += (b != tones[k] as usize) as usize;
                    good += p[k][tones[k] as usize];
                    other += (p[k].iter().sum::<f64>() - p[k][tones[k] as usize]) / 7.0;
                }
                es.push(e.to_string());
                dbs.push(format!("{:.1}", 10.0 * (good / other.max(1e-9)).log10()));
            }
            println!(
                "V {slot} {missed} {snr} {} {} {:.3} {:.3} {msg}",
                es.join(","),
                dbs.join(","),
                best_at.0,
                best_at.1
            );
        }
    }
}

/// `probe8 <probe7 output> <dir>`: the cells' **design** in `f64`, without the 16-bit transform and the half-dB byte: 8 kHz audio, the row of 80 ms read with a Hann window over two symbols (2 560
/// samples starting 640 before the symbol's start), the cell at bin `b` (3.125 Hz) the magnitude of the DFT at `b x 3.125` Hz; the same hard decision per symbol at the grid place (nearest row and bin
/// of the 12 kHz search's best place) and the best of the 3 x 3; and, for comparison, a plain one-symbol rectangular window at the exact place. `W <slot> <missed> <12k> <hann grid> <hann 3x3> <rect exact> <msg>`.
#[cfg(feature = "fixed-point")]
fn probe8(probe: &str, dir: &str) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::msg::wsjt77::pack77;
    let hann: Vec<f64> = (0..2560usize)
        .map(|k| {
            let s = (std::f64::consts::PI * k as f64 / 2560.0).sin();
            s * s
        })
        .collect();
    let mut cache: Option<(String, Vec<i16>)> = None;
    for l in std::fs::read_to_string(probe).unwrap().lines() {
        if !l.starts_with("V ") {
            continue;
        }
        let t: Vec<&str> = l.splitn(9, ' ').collect();
        let slot = t[1].to_string();
        let e12: usize = t[4].split(',').map(|x| x.parse::<usize>().unwrap()).sum();
        let (bf, bd): (f64, f64) = (t[6].parse().unwrap(), t[7].parse().unwrap());
        let msg = t[8].to_string();
        if cache.as_ref().is_none_or(|c| c.0 != slot) {
            cache = Some((
                slot.clone(),
                resample_to_8k(&load(&format!("{dir}/{slot}.wav")), 12_000),
            ));
        }
        let a8 = &cache.as_ref().unwrap().1;
        let parts: Vec<&str> = msg.split(' ').collect();
        let bits = match parts.len() {
            3 => pack77(parts[0], parts[1], parts[2]),
            2 => pack77(parts[0], parts[1], ""),
            _ => None,
        };
        let Some(bits) = bits else { continue };
        let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
        // magnitude^2 of a DFT of `len` samples from `start`, windowed by `w` (or rectangular), at `hz`
        let dft = |start: i64, len: usize, w: Option<&Vec<f64>>, hz: f64| -> f64 {
            let om = 2.0 * std::f64::consts::PI * hz / 8000.0;
            let (mut re, mut im) = (0f64, 0f64);
            for n in 0..len {
                let i = start + n as i64;
                if i >= 0 && (i as usize) < a8.len() {
                    let x = a8[i as usize] as f64 * w.map_or(1.0, |w| w[n]);
                    re += x * (om * n as f64).cos();
                    im -= x * (om * n as f64).sin();
                }
            }
            re * re + im * im
        };
        let errors_hann = |j0: i64, k0: i64| -> usize {
            tones
                .iter()
                .enumerate()
                .filter(|(k, tr)| {
                    let row = j0 + 2 * *k as i64;
                    let p: Vec<f64> = (0..8)
                        .map(|tt| {
                            dft(
                                row * 640 - 640,
                                2560,
                                Some(&hann),
                                (k0 + 2 * tt as i64) as f64 * 3.125,
                            )
                        })
                        .collect();
                    (0..8)
                        .max_by(|&x, &y| p[x].partial_cmp(&p[y]).unwrap())
                        .unwrap()
                        != **tr as usize
                })
                .count()
        };
        let (j0, k0) = (
            ((bd + 0.5) / 0.08).round() as i64,
            (bf / 3.125).round() as i64,
        );
        let grid = errors_hann(j0, k0);
        let mut best = grid;
        for dr in -1i64..=1 {
            for dc in -1i64..=1 {
                best = best.min(errors_hann(j0 + dr, k0 + dc));
            }
        }
        let rect = tones
            .iter()
            .enumerate()
            .filter(|(k, tr)| {
                let start = ((bd + 0.5) * 8000.0).round() as i64 + 1280 * *k as i64;
                let p: Vec<f64> = (0..8)
                    .map(|tt| dft(start, 1280, None, bf + 6.25 * tt as f64))
                    .collect();
                (0..8)
                    .max_by(|&x, &y| p[x].partial_cmp(&p[y]).unwrap())
                    .unwrap()
                    != **tr as usize
            })
            .count();
        // the same search the 12 kHz figure had (+-0.5 Hz in 0.25 Hz, +-0.03 s in 0.01 s), on the 8 kHz audio, a one-symbol rectangular window: the minimum of 35 places, as the 12 kHz one is
        let mut rect35 = usize::MAX;
        for i in -2i32..=2 {
            for j in -3i32..=3 {
                let (ff, dd) = (bf + 0.25 * i as f64, bd + 0.01 * j as f64);
                let e = tones
                    .iter()
                    .enumerate()
                    .filter(|(k, tr)| {
                        let start = ((dd + 0.5) * 8000.0).round() as i64 + 1280 * *k as i64;
                        let p: Vec<f64> = (0..8)
                            .map(|tt| dft(start, 1280, None, ff + 6.25 * tt as f64))
                            .collect();
                        (0..8)
                            .max_by(|&x, &y| p[x].partial_cmp(&p[y]).unwrap())
                            .unwrap()
                            != **tr as usize
                    })
                    .count();
                rect35 = rect35.min(e);
            }
        }
        println!(
            "W {slot} {} {e12} {grid} {best} {rect} {rect35} {msg}",
            t[2]
        );
    }
}

/// `probe10 <truth.tsv> <dir> <slot> <message> <out.wav>`: the slot's audio with **every other reference signal subtracted** (jt9's messages, frequency refined by `refine_freq`, DT by
/// `subtract_tones_lpf_refine_dt`, strongest first) written as a 12 kHz wav; prints the energy left against the original, in dB.
fn probe10(truth: &str, dir: &str, slot: &str, target: &str, out: &str) {
    use mfsk_core::engine::dsp::subtract::{
        GfskParams, SubtractCfg, refine_freq, subtract_tones_lpf_refine_dt,
    };
    use mfsk_core::msg::wsjt77::pack77;
    let cfg = SubtractCfg {
        sample_rate: 12_000.0,
        tone_spacing_hz: 6.25,
        samples_per_symbol: 1920,
        base_offset_s: 0.5,
        gfsk: Some(GfskParams {
            bt: 2.0,
            hmod: 1.0,
            ramp_samples: 1920 / 8,
            frame_ramp: false,
        }),
    };
    let mut sigs: Vec<Truth> = Vec::new();
    for l in std::fs::read_to_string(truth).unwrap().lines() {
        let f: Vec<&str> = l.splitn(6, '\t').collect();
        if f[0] == slot {
            sigs.push((
                f[1].parse().unwrap(),
                f[2].parse().unwrap(),
                f[3].parse().unwrap(),
                norm(f[4]),
                f[5].parse().unwrap(),
            ));
        }
    }
    sigs.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
    let orig = load(&format!("{dir}/{slot}.wav"));
    let mut r = orig.clone();
    let mut n = 0;
    for (f, dt, _, msg, _) in &sigs {
        if msg == target {
            continue;
        }
        let t: Vec<&str> = msg.split(' ').collect();
        let bits = match t.len() {
            3 => pack77(t[0], t[1], t[2]),
            2 => pack77(t[0], t[1], ""),
            _ => None,
        };
        if let Some(bits) = bits {
            let tones = mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits);
            let te = |x: &[i16], ff: f32| -> f64 {
                let mut e = 0.0;
                for (k, &t) in tones.iter().enumerate() {
                    let w = 2.0 * std::f64::consts::PI * (ff as f64 + 6.25 * t as f64) / 12_000.0;
                    let start = ((*dt + 0.5) * 12_000.0).round() as i64 + 1920 * k as i64;
                    let (mut re, mut im) = (0f64, 0f64);
                    for i in 0..1920usize {
                        let ix = start + i as i64;
                        if ix >= 0 && (ix as usize) < x.len() {
                            re += x[ix as usize] as f64 * (w * i as f64).cos();
                            im -= x[ix as usize] as f64 * (w * i as f64).sin();
                        }
                    }
                    e += re * re + im * im;
                }
                e
            };
            let f0 = refine_freq(&r, &tones, *f, *dt, &cfg, 2.5, 0.1);
            let before = te(&r, f0);
            subtract_tones_lpf_refine_dt(&mut r, &tones, f0, *dt, &cfg, 2000, true);
            if n < 6 {
                println!(
                    "  {msg}: printed f {f} dt {dt}, refined f {f0:.2}: its own tone energy left {:.1} dB",
                    10.0 * (te(&r, f0) / before).log10()
                );
            }
            n += 1;
        }
    }
    let e = |v: &[i16]| v.iter().map(|&x| (x as f64) * (x as f64)).sum::<f64>();
    println!(
        "subtracted {n} signals: energy left {:.1} dB of the original",
        10.0 * (e(&r) / e(&orig)).log10()
    );
    let mut pcm = Vec::new();
    for x in &r {
        pcm.extend_from_slice(&x.to_le_bytes());
    }
    let mut w = b"RIFF".to_vec();
    w.extend_from_slice(&(36 + pcm.len() as u32).to_le_bytes());
    w.extend_from_slice(b"WAVEfmt ");
    for v in [
        16u32.to_le_bytes().to_vec(),
        1u16.to_le_bytes().to_vec(),
        1u16.to_le_bytes().to_vec(),
        12000u32.to_le_bytes().to_vec(),
        24000u32.to_le_bytes().to_vec(),
        2u16.to_le_bytes().to_vec(),
        16u16.to_le_bytes().to_vec(),
    ] {
        w.extend_from_slice(&v);
    }
    w.extend_from_slice(b"data");
    w.extend_from_slice(&(pcm.len() as u32).to_le_bytes());
    w.extend_from_slice(&pcm);
    std::fs::write(out, w).unwrap();
}

/// `probe11 <truth2.tsv> <dir>`: which signals to subtract, and how well. For every reference signal the cells flow misses (one pass, OSD, 100 candidates), the slot's audio with
/// (a) **only the signals that same pass decodes** subtracted (jt9's precise places, `refine_freq` + `subtract_tones_lpf_refine_dt`: the subtraction's quality is not at issue), then decoded again
/// by one pass; (b) **every other reference signal** subtracted (the oracle). Prints `X <slot> <snr> <decoded after (a)> <decoded after (b)> <subtracted in (a)> <msg>` and a summary.
#[cfg(feature = "fixed-point")]
fn probe11(truth: &str, dir: &str) {
    use mfsk_core::engine::dsp::resample::resample_to_8k;
    use mfsk_core::engine::dsp::subtract::{
        GfskParams, SubtractCfg, refine_freq, subtract_tones_lpf_refine_dt,
    };
    use mfsk_core::ft8::decode::DecodeDepth;
    use mfsk_core::ft8::decode_block::{Ft8Rate, GatedSearch, decode_block_cells};
    use mfsk_core::msg::wsjt77::{pack77, unpack77};
    let cfg = SubtractCfg {
        sample_rate: 12_000.0,
        tone_spacing_hz: 6.25,
        samples_per_symbol: 1920,
        base_offset_s: 0.5,
        gfsk: Some(GfskParams {
            bt: 2.0,
            hmod: 1.0,
            ramp_samples: 1920 / 8,
            frame_ramp: false,
        }),
    };
    let mut by_slot: std::collections::BTreeMap<String, Vec<Truth>> = Default::default();
    for l in std::fs::read_to_string(truth).unwrap().lines() {
        let f: Vec<&str> = l.splitn(6, '\t').collect();
        by_slot.entry(f[0].to_string()).or_default().push((
            f[1].parse().unwrap(),
            f[2].parse().unwrap(),
            f[3].parse().unwrap(),
            norm(f[4]),
            f[5].parse().unwrap(),
        ));
    }
    let depth = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let search = GatedSearch::new(200.0, 3000.0, 1.3, 100);
    // the cells flow's own pass-0 decodes with the places it estimated
    let decode_places = |audio: &[i16]| -> Vec<(String, f32, f32)> {
        let a8 = resample_to_8k(audio, 12_000);
        decode_block_cells(&a8, Ft8Rate::Hz8000, &search, 100, depth)
            .unwrap_or_default()
            .iter()
            .filter_map(|d| unpack77(d.message77()).map(|m| (norm(&m), d.freq_hz, d.dt_sec)))
            .collect()
    };
    let decode_set = |audio: &[i16]| -> std::collections::HashSet<String> {
        let a8 = resample_to_8k(audio, 12_000);
        decode_block_cells(&a8, Ft8Rate::Hz8000, &search, 100, depth)
            .unwrap_or_default()
            .iter()
            .filter_map(|d| unpack77(d.message77()))
            .map(|m| norm(&m))
            .collect()
    };
    let tones_of = |msg: &str| -> Option<Vec<u8>> {
        let t: Vec<&str> = msg.split(' ').collect();
        let bits = match t.len() {
            3 => pack77(t[0], t[1], t[2]),
            2 => pack77(t[0], t[1], ""),
            _ => None,
        }?;
        Some(mfsk_core::engine::tx::message_to_tones::<Ft8>(&bits))
    };
    let subtract = |audio: &mut Vec<i16>, f: f32, dt: f32, msg: &str| -> bool {
        let Some(tones) = tones_of(msg) else {
            return false;
        };
        let f0 = refine_freq(audio, &tones, f, dt, &cfg, 2.5, 0.1);
        subtract_tones_lpf_refine_dt(audio, &tones, f0, dt, &cfg, 2000, true);
        true
    };
    let (mut n, mut a_ok, mut b_ok, mut c_ok, mut d_ok) = (0, 0, 0, 0, 0);
    for (slot, mut sigs) in by_slot {
        sigs.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
        let orig = load(&format!("{dir}/{slot}.wav"));
        let pass0 = decode_set(&orig);
        let own = decode_places(&orig);
        // (d): the same signals at the places the cells flow estimated, **refined on the 12 kHz audio first**: for each DT of a +-60 ms grid (5 ms) the frequency by `refine_freq`, the DT and frequency
        // with the largest tone correlation; then subtracted. Once a slot: the set does not depend on the target.
        let decoded_d: std::collections::HashSet<String> = {
            let mut rd = orig.clone();
            for (m2, f2, dt2) in &own {
                let Some(tones) = tones_of(m2) else { continue };
                let energy = |x: &[i16], ff: f32, dd: f32| -> f64 {
                    let mut e = 0.0;
                    for (k, &t) in tones.iter().enumerate() {
                        let w =
                            2.0 * std::f64::consts::PI * (ff as f64 + 6.25 * t as f64) / 12_000.0;
                        let start = ((dd + 0.5) * 12_000.0).round() as i64 + 1920 * k as i64;
                        let (mut re, mut im) = (0f64, 0f64);
                        for i in 0..1920usize {
                            let ix = start + i as i64;
                            if ix >= 0 && (ix as usize) < x.len() {
                                re += x[ix as usize] as f64 * (w * i as f64).cos();
                                im -= x[ix as usize] as f64 * (w * i as f64).sin();
                            }
                        }
                        e += re * re + im * im;
                    }
                    e
                };
                let mut best = (f64::MIN, *f2, *dt2);
                for j in -12i32..=12 {
                    let dd = *dt2 + 0.005 * j as f32;
                    let ff = refine_freq(&rd, &tones, *f2, dd, &cfg, 2.5, 0.1);
                    let e = energy(&rd, ff, dd);
                    if e > best.0 {
                        best = (e, ff, dd);
                    }
                }
                subtract_tones_lpf_refine_dt(&mut rd, &tones, best.1, best.2, &cfg, 2000, true);
            }
            decode_set(&rd)
        };
        for (f, dt, snr, msg, missed) in &sigs {
            if *missed == 0 {
                continue;
            }
            n += 1;
            let (mut ra, mut rb) = (orig.clone(), orig.clone());
            let mut na = 0;
            for (f2, dt2, _, m2, _) in &sigs {
                if m2 == msg {
                    continue;
                }
                subtract(&mut rb, *f2, *dt2, m2);
                if pass0.contains(m2) && subtract(&mut ra, *f2, *dt2, m2) {
                    na += 1;
                }
            }
            // (c): the cells flow's own decodes at the places it estimated
            let mut rc = orig.clone();
            for (m2, f2, dt2) in &own {
                subtract(&mut rc, *f2, *dt2, m2);
            }
            let oc = decode_set(&rc).contains(msg);
            c_ok += oc as usize;
            let od = decoded_d.contains(msg);
            d_ok += od as usize;
            let (oa, ob) = (decode_set(&ra).contains(msg), decode_set(&rb).contains(msg));
            a_ok += oa as usize;
            b_ok += ob as usize;
            println!(
                "X {slot} {snr} {} {} {} {na} {msg} (f {f} dt {dt})",
                oa as u8, ob as u8, oc as u8
            );
        }
    }
    println!(
        "T {n} missed: decoded after subtracting only what the cells flow decoded, at jt9's precise places (a): {a_ok}; at the places the cells flow estimated (c): {c_ok}; at those places refined on the 12 kHz audio first (d): {d_ok}; every other reference signal (b): {b_ok}"
    );
}

fn main() {
    let mut args = std::env::args().skip(1);
    let config = args.next().expect("config");
    #[cfg(feature = "fixed-point")]
    if config == "probe11" {
        let truth = args.next().unwrap();
        let dir = args.next().unwrap();
        probe11(&truth, &dir);
        return;
    }
    if config == "probe10" {
        let truth = args.next().unwrap();
        let dir = args.next().unwrap();
        let slot = args.next().unwrap();
        let msg = args.next().unwrap();
        let out = args.next().unwrap();
        probe10(&truth, &dir, &slot, &msg, &out);
        return;
    }
    #[cfg(feature = "fixed-point")]
    if config == "probe8" {
        let probe = args.next().expect("probe7 output");
        let dir = args.next().expect("corpus dir");
        probe8(&probe, &dir);
        return;
    }
    if config == "probe7" {
        let truth = args.next().expect("truth.tsv");
        let dir = args.next().expect("corpus dir");
        probe7(&truth, &dir);
        return;
    }
    if config == "tones" {
        tones_of(&args.next().expect("message"));
        return;
    }
    #[cfg(feature = "fixed-point")]
    if config == "probe6" {
        let wav = args.next().expect("wav");
        let (f, dt): (f32, f32) = (
            args.next().unwrap().parse().unwrap(),
            args.next().unwrap().parse().unwrap(),
        );
        let msg = args.next().unwrap();
        let resid = args.next();
        probe6(&wav, f, dt, &msg, resid.as_deref());
        return;
    }
    #[cfg(feature = "fixed-point")]
    if config == "probe5" {
        let wav = args.next().expect("wav");
        let (f, dt): (f32, f32) = (
            args.next().unwrap().parse().unwrap(),
            args.next().unwrap().parse().unwrap(),
        );
        let msg = args.next().unwrap();
        let (df, ddt): (f32, f32) = (
            args.next().map_or(6.0, |x| x.parse().unwrap()),
            args.next().map_or(0.7, |x| x.parse().unwrap()),
        );
        probe5(&wav, f, dt, &msg, df, ddt);
        return;
    }
    #[cfg(feature = "fixed-point")]
    if config == "probe4" {
        let list = args.next().expect("list.tsv");
        let dir = args.next().expect("corpus dir");
        probe4(&list, &dir);
        return;
    }
    #[cfg(feature = "fixed-point")]
    if config == "probe3" {
        let truth = args.next().expect("truth.tsv");
        let dir = args.next().expect("corpus dir");
        probe3(&truth, &dir);
        return;
    }
    #[cfg(feature = "fixed-point")]
    if config == "probe2" {
        let truth = args.next().expect("truth.tsv");
        let dir = args.next().expect("corpus dir");
        probe2(&truth, &dir);
        return;
    }
    #[cfg(feature = "fixed-point")]
    if config == "probe" {
        let truth = args.next().expect("truth.tsv");
        let dir = args.next().expect("corpus dir");
        probe(&truth, &dir);
        return;
    }
    let (mut hits, mut extras, mut total, mut total_ms) = (0usize, 0usize, 0usize, 0f64);
    for path in args {
        let audio = load(&path);
        #[cfg(feature = "fixed-point")]
        if let Some(m) = config.strip_prefix("cands:") {
            let slot = std::path::Path::new(&path)
                .file_stem()
                .unwrap()
                .to_string_lossy()
                .to_string();
            candidates(m.parse().expect("candidates"), &slot, &audio);
            continue;
        }
        let refs: HashSet<String> = std::fs::read_to_string(path.replace(".wav", ".ref"))
            .map(|t| t.lines().map(norm).collect())
            .unwrap_or_default();
        let t0 = Instant::now();
        let msgs: Vec<String> = match config.split_once(':') {
            Some(("cells", name)) => {
                #[cfg(feature = "fixed-point")]
                {
                    cells(name, &audio)
                }
                #[cfg(not(feature = "fixed-point"))]
                {
                    let _ = name;
                    panic!("the cells flow needs --features fixed-point")
                }
            }
            Some((d, a)) => {
                let depth = match d {
                    "fast" => Depth::Fast,
                    "normal" => Depth::Normal,
                    "deep" => Depth::Deep,
                    _ => panic!("depth {d}"),
                };
                let ap = match a {
                    "off" => ApMode::Off,
                    "cq" => ApMode::CqOnly,
                    "full" => ApMode::Full,
                    _ => panic!("ap {a}"),
                };
                faithful(depth, ap, &audio)
            }
            None => panic!("config is depth:ap or cells:name"),
        };
        let ms = t0.elapsed().as_secs_f64() * 1e3;
        let slot = std::path::Path::new(&path)
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_string();
        let uniq: HashSet<&String> = msgs.iter().collect();
        let h = uniq.iter().filter(|m| refs.contains(**m)).count();
        println!("S {slot} {} {h} {ms:.0}", uniq.len());
        for m in &uniq {
            println!("D {slot} {m}");
        }
        hits += h;
        extras += uniq.len() - h;
        total += uniq.len();
        total_ms += ms;
    }
    println!("T {total} {hits} {extras} {total_ms:.0}");
}
