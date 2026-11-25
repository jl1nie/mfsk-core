// SPDX-License-Identifier: GPL-3.0-only
//! One decode of a slot through the embedded path, and the scoring against the `jt9` reference.

use std::collections::BTreeMap;
use std::time::Instant;

use mfsk_core::ft8::decode::DecodeDepth;
use mfsk_core::ft8::decode_block::band_bb::{
    BandBaseband, BandFilters, SubMode, decode_band, decode_band_sic,
};
use mfsk_core::ft8::decode_block::source::{CountingSource, ReadCost, SliceSource};
use mfsk_core::ft8::decode_block::{
    LowMem, coarse_sync_gated_band, coarse_sync_gated_src, decode_block, decode_block_low_mem,
    decode_block_low_mem_src,
};
use mfsk_core::ft8::subtract::subtract_signal_lpf;
use mfsk_core::msg::wsjt77::unpack77;

use crate::heap;

/// `jt9 -8 -d 3 -L 200 -H 3000` of WSJT-X v3.3.0-beta6 on ft8_lib's 22 recordings: the tier-B reference
/// (`mfsk-core/tests/ft8_real_recordings_vs_jt9.rs`), 402 decodes.
pub const REFERENCE: &str = include_str!("../../../mfsk-core/tests/fixtures/ft8_lib_jt9_beta6.tsv");

/// Frequency tolerance when pairing a decode with a reference line, Hz (as `bench/ft8-compare`).
pub const FTOL_HZ: f32 = 10.0;

/// The ship configuration of `decode_block` (`ft8_embedded_driver_recall.rs`): the knobs the embedded
/// build reads. `max_cand` is the one that moves the result (see [`Run`]).
pub const SYNC_MIN: f32 = 1.3;

/// The depth the ICAP profile decodes at: `EMBEDDED`'s cheap LLR effort, with the OSD fallback when the
/// `osd` feature is on (it is a no-op without it).
pub const LOW_MEM_DEPTH: DecodeDepth = DecodeDepth {
    llr_effort: mfsk_core::engine::frame::LlrEffort::Minimal,
    osd: cfg!(feature = "osd"),
    jitter: false,
};

pub fn norm(m: &str) -> String {
    m.split_whitespace()
        .collect::<Vec<_>>()
        .join(" ")
        .to_uppercase()
}

/// `(file stem, message, freq Hz)` per reference line.
pub fn reference() -> Vec<(String, String, f32)> {
    REFERENCE
        .lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split('\t').collect();
            (f[0].to_string(), norm(f[4]), f[1].parse().unwrap())
        })
        .collect()
}

/// The recording stems the reference names, in order.
pub fn stems() -> Vec<String> {
    let mut s: Vec<String> = reference().into_iter().map(|r| r.0).collect();
    s.dedup();
    s
}

pub fn asset_path(stem: &str) -> String {
    format!(
        "{}/../../embedded-poc/assets/{stem}.wav",
        env!("CARGO_MANIFEST_DIR")
    )
}

/// What one `decode_block` call did.
pub struct Run {
    pub decodes: Vec<(String, f32)>,
    pub ms: f64,
    /// High-water mark of live heap above what was live before the call (the audio is not counted).
    pub peak_heap: usize,
    /// Allocations of 64 KiB or more during the call: `(bytes, times)`, largest first.
    pub big: Vec<(usize, usize)>,
}

/// Decode one slot of 12 kHz audio with the embedded driver, band 200-3000 Hz, the coarse search in
/// bands of carriers and with no block-0 spectrum kept per candidate (`decode_block_low_mem`), or as
/// `decode_block` does with `None`.
pub fn run_slot(slot: &[i16], max_cand: usize, low_mem_bands: Option<usize>) -> Run {
    let base = heap::reset_peak();
    let t = Instant::now();
    let out = match low_mem_bands {
        None => decode_block(
            slot,
            200.0,
            3000.0,
            SYNC_MIN,
            DecodeDepth::EMBEDDED,
            max_cand,
        ),
        Some(bands) => decode_block_low_mem(
            slot,
            200.0,
            3000.0,
            SYNC_MIN,
            LOW_MEM_DEPTH,
            max_cand,
            bands,
        ),
    };
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let peak_heap = heap::peak().saturating_sub(base);
    let big = heap::big_allocs();
    let decodes = out
        .iter()
        .filter_map(|d| unpack77(d.message77()).map(|m| (norm(&m), d.freq_hz)))
        .collect();
    Run {
        decodes,
        ms,
        peak_heap,
        big,
    }
}

/// Decodes `jt9` lacks that are real signals, with the evidence (`docs/notes/FT8_REAL_VS_JT9.md`; tier B's
/// `ATTESTED` list is the full one for the wide-band decoder, these are the ones the embedded path has made).
pub const ATTESTED_EXTRAS: &[(&str, &str, &str)] = &[
    (
        "websdr_test13",
        "EA8BEV LU3DW RR73",
        "in ft8_lib's own list for the file (309 Hz, -4 dB), and a QSO's closing message heard in the neighbouring files",
    ),
    (
        "websdr_test11",
        "K4VBM HA8EK RR73",
        "ft8_lib lists it at 2080, 2130 and 2230 Hz: a splattering signal's copies, this one at another frequency; the wide-band decoder makes it too",
    ),
    (
        "websdr_test10",
        "AE0XI R7CA -15",
        "in ft8_lib's own list for the file (335 Hz); the wide-band decoder makes `AE0XI R7CA RR73` there",
    ),
    (
        "191111_110215",
        "SP7NHS YC2FCE -15",
        "tier B's `ATTESTED` list, 'undecided': made by the wide-band decoder too, recovered in 6 of 16 noisy copies",
    ),
    (
        "191111_110700",
        "<...> DF1XG JO53",
        "ICAP's own decoder (`ft8dec`) lists it at 2111 Hz, -21 dB; found by the baseband path's subtraction passes",
    ),
    (
        "websdr_test2",
        "UA9SIX GM0LIR R-07",
        "`ft8dec` lists it at 1517 Hz, -12 dB; found by the baseband path's subtraction passes",
    ),
    (
        "websdr_test6",
        "CQ UT9LB KN89",
        "`ft8dec` lists it at 690 Hz, -5 dB; found by the baseband path's subtraction passes",
    ),
    (
        "websdr_test9",
        "K4VBM HA8EK -15",
        "`jt9` has it at 2229 Hz (+11 dB); this is a copy 150 Hz higher, the strong signal's splatter (as `websdr_test11`'s copies of the same pair), kept where the first found of a message is kept",
    ),
    (
        "191111_110615",
        "PA3EPP SP8NFO KN09",
        "`jt9` has it at 906 Hz (+22 dB); this is a copy 250 Hz lower, the strong signal's splatter, kept because the per-candidate entry points have no per-message dedupe (the boards' `run_speculative_slot` does: `dedup_by_message`)",
    ),
    (
        "191111_110645",
        "PA3EPP SP8NFO R+01",
        "`jt9` has it at 906 Hz (+12 dB); this is a copy 150 Hz lower, as above",
    ),
    (
        "191111_110645",
        "PB5DX OM4TC JN98",
        "undecided: no other decoder lists it. The message is consistent (OM4TC is a Slovak call and JN98 a Slovak grid) and PB5DX is heard in the neighbouring recordings, which a CRC chance match would not produce",
    ),
];

#[derive(Debug, Default, PartialEq)]
pub struct Score {
    pub reference: usize,
    pub hits: usize,
    /// Decodes with no `jt9` line.
    pub extras: usize,
    /// Of those, the ones not in [`ATTESTED_EXTRAS`]: phantoms until shown otherwise.
    pub unexplained: usize,
}

/// Pair each decode with an unused reference line of the same message within [`FTOL_HZ`]; a decode with
/// none is an extra (a phantom, or a real signal `jt9` lacks: tier B's `ATTESTED` list sorts those).
pub fn score(runs: &BTreeMap<String, Vec<(String, f32)>>) -> Score {
    let refs = reference();
    let mut s = Score {
        reference: refs.len(),
        ..Default::default()
    };
    for (stem, decodes) in runs {
        let mut used = vec![false; refs.len()];
        for (m, f) in decodes {
            let hit = refs.iter().enumerate().position(|(j, r)| {
                !used[j] && &r.0 == stem && &r.1 == m && (r.2 - f).abs() <= FTOL_HZ
            });
            match hit {
                Some(j) => {
                    used[j] = true;
                    s.hits += 1;
                }
                None => {
                    s.extras += 1;
                    if !ATTESTED_EXTRAS.iter().any(|a| a.0 == stem && a.1 == m) {
                        s.unexplained += 1;
                    }
                }
            }
        }
    }
    s
}

/// What [`run_slot_sic`] did beyond a [`Run`].
pub struct SicRun {
    pub run: Run,
    /// Time spent subtracting, ms (included in `run.ms`).
    pub subtract_ms: f64,
    /// New decodes per pass.
    pub per_pass: Vec<usize>,
    /// Every decode with the pass that found it (0 = on the audio as recorded).
    pub found: Vec<Found>,
}

/// One decode of [`run_slot_sic`].
pub struct Found {
    pub pass: usize,
    pub msg: String,
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: f32,
}

/// `passes` rounds of [`run_slot`]'s low-memory decode, each on the audio left after subtracting what the
/// rounds before it decoded (`ft8::subtract::subtract_signal_lpf`, `subtractft8.f90`'s filter, here as the
/// no-FFT direct convolution an embedded build has). A message already decoded is not counted again.
///
/// This **measures what subtraction is worth** to the embedded path; it is not a design for the radio. It
/// keeps a second copy of the audio, which the radio cannot (the copy is not in `run.peak_heap`), and the
/// direct convolution costs ~6e8 multiply-adds per subtracted signal.
pub fn run_slot_sic(slot: &[i16], max_cand: usize, low_mem_bands: usize, passes: usize) -> SicRun {
    let mut work = slot.to_vec();
    let base = heap::reset_peak();
    let t = Instant::now();
    let mut found: Vec<mfsk_core::ft8::decode::DecodeResult> = Vec::new();
    let mut pass_of: Vec<usize> = Vec::new();
    let mut per_pass = Vec::new();
    let mut subtract_ms = 0.0;
    for pass in 0..passes {
        let out = decode_block_low_mem(
            &work,
            200.0,
            3000.0,
            SYNC_MIN,
            LOW_MEM_DEPTH,
            max_cand,
            low_mem_bands,
        );
        let fresh: Vec<_> = out
            .into_iter()
            .filter(|r| !found.iter().any(|f| f.message77() == r.message77()))
            .collect();
        per_pass.push(fresh.len());
        let ts = Instant::now();
        for r in &fresh {
            subtract_signal_lpf(&mut work, r);
        }
        subtract_ms += ts.elapsed().as_secs_f64() * 1e3;
        pass_of.extend(std::iter::repeat_n(pass, fresh.len()));
        found.extend(fresh);
    }
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let peak_heap = heap::peak().saturating_sub(base);
    let decodes = found
        .iter()
        .filter_map(|d| unpack77(d.message77()).map(|m| (norm(&m), d.freq_hz)))
        .collect();
    SicRun {
        run: Run {
            decodes,
            ms,
            peak_heap,
            big: Vec::new(),
        },
        subtract_ms,
        per_pass,
        found: found
            .iter()
            .zip(&pass_of)
            .filter_map(|(d, &pass)| {
                unpack77(d.message77()).map(|m| Found {
                    pass,
                    msg: norm(&m),
                    freq_hz: d.freq_hz,
                    dt_sec: d.dt_sec,
                    snr_db: d.snr_db,
                })
            })
            .collect(),
    }
}

/// One decode through the radio's interface: the slot is only reachable through an `AudioSource`, read forward.
pub struct StreamRun {
    pub run: Run,
    /// What the reads cost on the radio (`CountingSource`): bytes moved, calls, restarts.
    pub cost: ReadCost,
}

/// [`run_slot`]'s low-memory profile as the radio runs it: `decode_block_low_mem_src` over a counted source.
pub fn run_slot_stream(slot: &[i16], max_cand: usize, cfg: LowMem) -> StreamRun {
    let mut src = CountingSource::new(SliceSource(slot));
    let base = heap::reset_peak();
    let t = Instant::now();
    let out = decode_block_low_mem_src(
        &mut src,
        200.0,
        3000.0,
        SYNC_MIN,
        LOW_MEM_DEPTH,
        max_cand,
        cfg,
    );
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let peak_heap = heap::peak().saturating_sub(base);
    let decodes = out
        .iter()
        .filter_map(|d| unpack77(d.message77()).map(|m| (norm(&m), d.freq_hz)))
        .collect();
    StreamRun {
        run: Run {
            decodes,
            ms,
            peak_heap,
            big: Vec::new(),
        },
        cost: src.cost(),
    }
}

/// Method C of `docs/notes/MIDRANGE_EMBEDDED_EVAL.md`: the whole band as one complex baseband (`band_bb`,
/// one forward read of the audio, 180 KB), every candidate's 200 Hz baseband cut from it. The candidates
/// come from the same gated coarse search as [`run_slot_stream`]'s, which still reads the audio `bands`
/// more times (the coarse search from the baseband is the next step), so `cost` counts the two together.
pub struct BandRun {
    pub run: Run,
    /// Reads of the audio: the baseband's one, plus the coarse search's.
    pub cost: ReadCost,
    /// Time of the three stages, ms: coarse search, band baseband, candidates.
    pub stage_ms: [f64; 3],
    /// Peak heap with only the band baseband live (the baseband's own size, for the report).
    pub band_bytes: usize,
}

pub fn run_slot_band(
    slot: &[i16],
    max_cand: usize,
    bands: usize,
    group: usize,
    coarse_from_band: bool,
) -> BandRun {
    let ms = |t: Instant| t.elapsed().as_secs_f64() * 1e3;
    let mut src = CountingSource::new(SliceSource(slot));
    let base = heap::reset_peak();
    let t = Instant::now();
    let f = BandFilters::new();
    // `coarse_from_band`: the audio is read once in all (for the baseband) and the coarse search runs on it.
    // Otherwise the coarse search reads the slot `bands` times (+ the gain in fixed-point) first: two sources,
    // one file, so the costs add (the file is reopened between the two, which `restarts` counts)
    let mut coarse_src = CountingSource::new(SliceSource(slot));
    let (gated, bb, ms_coarse, ms_bb) = if coarse_from_band {
        let t1 = Instant::now();
        let bb = BandBaseband::from_source(&f, &mut src);
        let ms_bb = ms(t1);
        let t2 = Instant::now();
        let gated = coarse_sync_gated_band(&bb, 200.0, 3000.0, SYNC_MIN, max_cand, bands);
        (gated, bb, ms(t2), ms_bb)
    } else {
        let t1 = Instant::now();
        let gated =
            coarse_sync_gated_src(&mut coarse_src, 200.0, 3000.0, SYNC_MIN, max_cand, bands);
        let ms_coarse = ms(t1);
        let t2 = Instant::now();
        let bb = BandBaseband::from_source(&f, &mut src);
        (gated, bb, ms_coarse, ms(t2))
    };
    let t3 = Instant::now();
    let out = decode_band(&bb, &f, &gated, LOW_MEM_DEPTH, group);
    let ms_cand = ms(t3);
    let peak_heap = heap::peak().saturating_sub(base);
    let decodes = out
        .iter()
        .filter_map(|d| unpack77(d.message77()).map(|m| (norm(&m), d.freq_hz)))
        .collect();
    BandRun {
        run: Run {
            decodes,
            ms: ms(t),
            peak_heap,
            big: Vec::new(),
        },
        cost: {
            let (a, b) = (coarse_src.cost(), src.cost());
            ReadCost {
                calls: a.calls + b.calls,
                samples_read: a.samples_read + b.samples_read,
                samples_skipped: a.samples_skipped + b.samples_skipped,
                restarts: a.restarts + b.restarts + u64::from(!coarse_from_band),
            }
        },
        stage_ms: [ms_coarse, ms_bb, ms_cand],
        band_bytes: bb.data.len() * 4,
    }
}

/// Method C with `passes` rounds of subtraction on the baseband ([`decode_band_sic`]): one read of the audio
/// in all, the coarse search, the candidates and the subtraction all on the 180 KB baseband.
pub struct BandSicRun {
    pub run: Run,
    pub cost: ReadCost,
    pub per_pass: Vec<usize>,
    pub subtract_ms: f64,
}

pub fn run_slot_band_sic(
    slot: &[i16],
    max_cand: usize,
    bands: usize,
    group: usize,
    passes: usize,
) -> BandSicRun {
    run_slot_band_sic_with(slot, max_cand, bands, group, passes, SubMode::F64)
}

/// [`run_slot_band_sic`] with the subtraction's arithmetic chosen.
pub fn run_slot_band_sic_with(
    slot: &[i16],
    max_cand: usize,
    bands: usize,
    group: usize,
    passes: usize,
    sub: SubMode,
) -> BandSicRun {
    let mut src = CountingSource::new(SliceSource(slot));
    let base = heap::reset_peak();
    let t = Instant::now();
    let f = BandFilters::new();
    let mut bb = BandBaseband::from_source(&f, &mut src);
    let t_sub = Instant::now();
    let (out, rep) = decode_band_sic(
        &mut bb,
        &f,
        200.0,
        3000.0,
        SYNC_MIN,
        LOW_MEM_DEPTH,
        max_cand,
        bands,
        group,
        passes,
        sub,
        &mut || t_sub.elapsed().as_secs_f64() * 1e3,
    );
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let peak_heap = heap::peak().saturating_sub(base);
    let decodes = out
        .iter()
        .filter_map(|d| unpack77(d.message77()).map(|m| (norm(&m), d.freq_hz)))
        .collect();
    BandSicRun {
        run: Run {
            decodes,
            ms,
            peak_heap,
            big: Vec::new(),
        },
        cost: src.cost(),
        per_pass: rep.per_pass,
        subtract_ms: rep.subtract_ms,
    }
}

/// The ablation of `docs/notes/MIDRANGE_EMBEDDED_EVAL.md` / the S3 plan, stage 0: which part of the low-memory
/// profile's recall comes from which piece, with the candidates' symbol spectra from Goertzel (what the ESP32 boards
/// run) or from the FIR baseband. All with the coarse search in 1 band, `LlrEffort::Minimal`, 4-iteration-free defaults.
///
/// | | candidates | fine refine | symbol spectra | OSD |
/// |---|---|---|---|---|
/// | `V0` | coarse score order (`decode_block`, the ESP32 driver) | none (the embedded `fine_refine_pass1` is a no-op) | Goertzel | no |
/// | `V1` | hard-sync gate | none | Goertzel | no |
/// | `V2` | hard-sync gate | none | Goertzel | yes |
/// | `V3` | coarse score order | none | Goertzel | yes |
/// | `V4` | hard-sync gate | FIR baseband (`fine_refine_3stage`) | FIR baseband | yes (the ICAP profile) |
/// | `V5` | hard-sync gate | FIR baseband | FIR baseband | no |
/// | `V6` | hard-sync gate | `fine_sync_12k` (Goertzel; the board's `fine_sync_split`) | Goertzel | no |
/// | `V7` | hard-sync gate | `fine_sync_12k` | Goertzel | yes |
/// | `V8` | coarse score order | `fine_sync_12k` | Goertzel | no (the board with time to spare) |
/// | `V9` | hard-sync gate, **the first `max_cand` by hard sync** (no pass-2 re-rank by block-0 quality) | `fine_sync_12k` | Goertzel | yes |
/// | `V10` | as `V9` | `fine_sync_12k` | Goertzel | no |
/// | `V12` | as `V9`, the gate read from the whole-slot spectrogram (`coarse_sync_gated_spec`, `None`) | `fine_sync_12k` | Goertzel | yes |
/// | `V11` | as `V9`, the gate read from the spectrogram **as the boards have it when they search**: the last [`UNMEASURED_ROWS`] rows zero, `valid_rows` given, the lag window the boards' (the coverage ceiling), block 2 scored on the rows there are | `fine_sync_12k` | Goertzel | yes |
/// | `V13` | as `V11` but with the default lag window and block 2 gated (`PartialBlock2::Gate`, the boards' open experiment) | `fine_sync_12k` | Goertzel | yes |
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Variant {
    V0,
    V1,
    V2,
    V3,
    V4,
    V5,
    V6,
    V7,
    V8,
    V9,
    V10,
    V11,
    V12,
    V13,
}

impl Variant {
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "v0" => Self::V0,
            "v1" => Self::V1,
            "v2" => Self::V2,
            "v3" => Self::V3,
            "v4" => Self::V4,
            "v5" => Self::V5,
            "v6" => Self::V6,
            "v7" => Self::V7,
            "v8" => Self::V8,
            "v9" => Self::V9,
            "v10" => Self::V10,
            "v11" => Self::V11,
            "v12" => Self::V12,
            "v13" => Self::V13,
            _ => return None,
        })
    }
}

/// Rows of the spectrogram the boards have not measured when they search: `SPEC_EMIT_PAIR` 87 of 92 pairs of the
/// 184-row `nstep-half` grid, so the last 10 (0.8 s). The fixed-point harness only.
pub const UNMEASURED_ROWS: usize = 10;

/// Candidates kept by the gate before the pass-2 re-rank: `pass1_limit` of the ESP32 driver.
pub const GATE_KEEP: usize = 30;

/// One slot through variant `v` with `max_cand` candidates. `V1` and `V2` need the fixed-point build (the
/// per-candidate entry points are `fixed-point`-gated); in the f32 build they decode nothing and say so.
pub fn run_slot_variant(slot: &[i16], v: Variant, max_cand: usize) -> Run {
    let base = heap::reset_peak();
    let t = Instant::now();
    let osd_on = DecodeDepth {
        osd: true,
        ..DecodeDepth::EMBEDDED
    };
    let out: Vec<mfsk_core::ft8::decode::DecodeResult> = match v {
        Variant::V0 => decode_block(
            slot,
            200.0,
            3000.0,
            SYNC_MIN,
            DecodeDepth::EMBEDDED,
            max_cand,
        ),
        Variant::V3 => decode_block(slot, 200.0, 3000.0, SYNC_MIN, osd_on, max_cand),
        Variant::V4 => {
            decode_block_low_mem(slot, 200.0, 3000.0, SYNC_MIN, LOW_MEM_DEPTH, max_cand, 1)
        }
        Variant::V5 => decode_block_low_mem(
            slot,
            200.0,
            3000.0,
            SYNC_MIN,
            DecodeDepth {
                osd: false,
                ..LOW_MEM_DEPTH
            },
            max_cand,
            1,
        ),
        Variant::V1
        | Variant::V2
        | Variant::V6
        | Variant::V7
        | Variant::V8
        | Variant::V9
        | Variant::V10
        | Variant::V11
        | Variant::V12
        | Variant::V13 => {
            #[cfg(feature = "fixed-point")]
            {
                use mfsk_core::ft8::decode_block::{
                    DEFAULT_Q_THRESH, SpecFill, coarse_sync, coarse_sync_gated,
                    coarse_sync_gated_spec, compute_spectrogram, fine_sync_12k,
                    process_candidates_into, refine_candidates_into,
                };
                let keep = GATE_KEEP.max(max_cand);
                let mut cands: Vec<_> = if v == Variant::V8 {
                    let spec = compute_spectrogram(slot, 3000.0);
                    coarse_sync(&spec, 200.0, 3000.0, SYNC_MIN, keep)
                } else if matches!(v, Variant::V11 | Variant::V12 | Variant::V13) {
                    let mut spec = compute_spectrogram(slot, 3000.0);
                    let fill = if matches!(v, Variant::V11 | Variant::V13) {
                        let valid = spec.n_time - UNMEASURED_ROWS;
                        for row in valid..spec.n_time {
                            let (lo, hi) = (row * spec.n_freq, (row + 1) * spec.n_freq);
                            spec.data[lo..hi].fill(Default::default());
                        }
                        Some(valid)
                    } else {
                        None
                    };
                    coarse_sync_gated_spec(
                        &spec,
                        200.0,
                        3000.0,
                        SYNC_MIN,
                        keep,
                        None,
                        fill.map(|valid_rows| SpecFill {
                            valid_rows,
                            gate_block2: v == Variant::V13,
                        }),
                    )
                    .into_iter()
                    .map(|c| c.0)
                    .collect()
                } else {
                    coarse_sync_gated(slot, 200.0, 3000.0, SYNC_MIN, keep, 1)
                        .into_iter()
                        .map(|c| c.0)
                        .collect()
                };
                if matches!(v, Variant::V9 | Variant::V10 | Variant::V11 | Variant::V12) {
                    cands.truncate(max_cand);
                }
                if matches!(
                    v,
                    Variant::V6
                        | Variant::V7
                        | Variant::V8
                        | Variant::V9
                        | Variant::V10
                        | Variant::V11
                        | Variant::V12
                        | Variant::V13
                ) {
                    cands = fine_sync_12k(slot, &cands);
                }
                let refined = refine_candidates_into(slot, cands, max_cand);
                let depth = if matches!(
                    v,
                    Variant::V2
                        | Variant::V7
                        | Variant::V9
                        | Variant::V11
                        | Variant::V12
                        | Variant::V13
                ) {
                    osd_on
                } else {
                    DecodeDepth::EMBEDDED
                };
                process_candidates_into(slot, refined, depth, DEFAULT_Q_THRESH)
            }
            #[cfg(not(feature = "fixed-point"))]
            {
                Vec::new()
            }
        }
    };
    let ms = t.elapsed().as_secs_f64() * 1e3;
    let peak_heap = heap::peak().saturating_sub(base);
    let decodes = out
        .iter()
        .filter_map(|d| unpack77(d.message77()).map(|m| (norm(&m), d.freq_hz)))
        .collect();
    Run {
        decodes,
        ms,
        peak_heap,
        big: Vec::new(),
    }
}
