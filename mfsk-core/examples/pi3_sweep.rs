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
#[cfg(feature = "fixed-point")]
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

fn main() {
    let mut args = std::env::args().skip(1);
    let config = args.next().expect("config");
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
