//! Q65 AWGN SNR sweep against `q65sim`-generated signals, across all
//! six sub-mode ZSTs this crate actually wires (`Q65a30`, `Q65a60`,
//! `Q65b60`, `Q65c60`, `Q65d60`, `Q65e60`).
//!
//! This test is `#[ignore]` — run it manually when investigating Q65
//! sensitivity. `q65sim` is WSJT-X's canonical Q65 signal generator
//! (Fortran, `WSJT-X/lib/qra/q65/q65sim.f90`) and — unlike `jt9sim` —
//! has a real CMakeLists.txt target, so build it via the full WSJT-X
//! CMake build:
//!
//! ```sh
//! cmake --build ~/wsjtx-build --target q65sim -j"$(nproc)"
//! ```
//!
//! Then:
//!
//! ```sh
//! # 1. Generate WAVs (once, or when widening the SNR grid):
//! scripts/gen_q65_sweep_wavs.sh ~/wsjtx-build/q65sim
//!
//! # 2. Run the sweep:
//! cargo test --test q65_sim_sweep --release --features q65,fft-rustfft,uvpacket \
//!   -- --ignored --nocapture
//! ```
//!
//! (`MFSK_Q65_SWEEP_DIR` overrides the default corpus location
//! `../embedded-poc/assets/q65_sweep`, relative to `CARGO_MANIFEST_DIR`.)
//!
//! ## Scope: only the six sub-modes actually wired
//!
//! WSJT-X's Q65 also supports 15/120/300 s T/R periods and other
//! (period, sub-mode) combinations, but this crate only wires the
//! 30 s A sub-mode plus all five 60 s sub-modes (see
//! `docs/reference/LIBRARY.md` §0.5). This sweep intentionally
//! covers only what's shipped, not a hypothetical superset — same
//! scoping call as `tests/jt65_sweep.rs`/`tests/jt9_sweep.rs`.
//!
//! ## AWGN-only; plain BP decode *and* the CQ-AP-hinted variant
//!
//! Reports two curves per sub-mode: `decode_scan_for` (plain AWGN
//! Bessel + BP, no assumptions) and `decode_scan_with_ap_for` with a
//! `call1 = "CQ"` hint. Both matter for a fair WSJT-X comparison —
//! see the issue #171 follow-up note below. Neither the fast-fading
//! nor full AP-list strategies are swept here (exercised separately
//! by `tests/q65_fast_fading.rs` / `tests/q65_ap_list.rs`, and don't
//! need an independent-reference generator sweep the same way since
//! they compose with this baseline path rather than replacing its
//! core FEC). All q65sim signals here are clean AWGN (fDop=0, no
//! Doppler/drift/tracking) — fading-channel sensitivity is a separate
//! axis already covered by the fast-fading test.
//!
//! ## Per-submode threshold grids
//!
//! Centred on `q65params.f90`'s analytical formula
//! (`-27 + 10*log10(7200/nsps)`, which depends only on T/R period,
//! not sub-mode letter — under pure AWGN, wider tone spacing doesn't
//! change matched-filter sensitivity, only Doppler/fading tolerance):
//! -24 dB for Q65-30A, -27 dB for all five 60 s sub-modes.
//!
//! ## Provenance (2026-07-19)
//!
//! Built after the JT65 (#24/#168) and JT9 sweep work in the same
//! session, prompted by the observation that Q65's only prior
//! independent-reference validation was 3 real off-air WSJT-X sample
//! recordings (`tests/q65_wsjtx_samples.rs`) plus a self-synth-only
//! "sweep" (`tests/q65_snr_sweep.rs`, own encoder + own AWGN — the
//! same self-roundtrip-blind pattern that hid the JT9 (#19) and JT65
//! interleave bugs). This sweep closes that gap with a real
//! independent-reference generator across every wired sub-mode.
//!
//! ## Issue #171 follow-up: the "plain vs plain" comparison was
//! ## partly a methodology gap, not just an implementation gap
//!
//! This sweep first found (and #171's fix narrowed) a real
//! fine-timing-precision bug in `coarse_search_for` — see the
//! CHANGELOG `## 0.7.5` entry. But after that fix, a residual gap
//! against WSJT-X's own `jt9 -3` remained at the deep end (e.g.
//! Q65-30A ~0% at -26 dB vs. WSJT-X's ~40%). Tracing further: `jt9`'s
//! default decode is not actually "plain" in the sense
//! `decode_scan_for` is — WSJT-X's Q65 decode chain always has access
//! to the free "CQ ??? ???" AP hypothesis (`aptype=1` in
//! `extract.f90`'s AP table, requiring no user-supplied callsign), so
//! *every* real decode attempt implicitly gets some AP-list benefit
//! for CQ-calling signals, whether or not the user configured `-c`/
//! `-x`. Re-measuring with `decode_scan_with_ap_for` + a `"CQ"` hint
//! closes the remaining gap almost exactly: Q65-30A -26 dB 0%→40%
//! (WSJT-X: 40%), -25 dB 33%→93% (WSJT-X: 87%), -24 dB 93%→100%
//! (WSJT-X: 100%); Q65-60A -28 dB 47%→93%, -29 dB 7%→53%. This isn't
//! a reason to silently fold AP into `decode_scan_for` (the crate's
//! four decoder strategies stay deliberately separate,
//! `docs/reference/LIBRARY.md` §3) — it's a usage note: an
//! application wanting WSJT-X-equivalent behavior for CQ traffic
//! should call the AP-hinted path with at least a `"CQ"` hint, not
//! the plain one, the same way WSJT-X's own decoder always does
//! internally.
//!
//! Output is a recall table — no assertions, statistics only.

#![cfg(all(feature = "q65", any(feature = "fft-rustfft", feature = "fft-extern")))]

use std::path::{Path, PathBuf};

#[allow(dead_code)]
mod common;
use common::load_wav_f32_opt;
use mfsk_core::msg::ApHint;
use mfsk_core::q65::search::SearchParams;
use mfsk_core::q65::{
    Q65a30, Q65a60, Q65b60, Q65c60, Q65d60, Q65e60, decode_scan_for, decode_scan_with_ap_for,
};

const GOLDEN_MSG: &str = "CQ JL1NIE PM95";
const GOLDEN_FREQ_HZ: f32 = 1500.0;
const FREQ_TOL_HZ: f32 = 4.0;

const SUBMODES: &[&str] = &["a30", "a60", "b60", "c60", "d60", "e60"];

fn sweep_dir() -> PathBuf {
    if let Ok(d) = std::env::var("MFSK_Q65_SWEEP_DIR") {
        return PathBuf::from(d);
    }
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    Path::new(&manifest)
        .join("../embedded-poc/assets/q65_sweep")
        .to_path_buf()
}

/// Parse `q65_<submode>_awgn_<snr_tag>_<trial>.wav`.
/// snr_tag: `m27` = -27, `p05` = +5.
fn parse_snr_tag(tag: &str) -> Option<i32> {
    if let Some(rest) = tag.strip_prefix('m') {
        rest.parse::<i32>().ok().map(|v| -v)
    } else if let Some(rest) = tag.strip_prefix('p') {
        rest.parse::<i32>().ok()
    } else {
        None
    }
}

/// `(plain_hit, cq_ap_hinted_hit)`.
fn decode_wav_q65(submode: &str, audio: &[f32], cq_hint: &ApHint) -> (bool, bool) {
    let params = SearchParams::default();
    let hit = |freq_hz: f32, msg: &str| {
        msg == GOLDEN_MSG && (freq_hz - GOLDEN_FREQ_HZ).abs() <= FREQ_TOL_HZ
    };
    macro_rules! decode_both {
        ($p:ty) => {{
            let plain = decode_scan_for::<$p>(audio, 12_000, 0, &params)
                .iter()
                .any(|d| hit(d.freq_hz, &d.message));
            let cq = decode_scan_with_ap_for::<$p>(audio, 12_000, 0, &params, cq_hint)
                .iter()
                .any(|d| hit(d.freq_hz, &d.message));
            (plain, cq)
        }};
    }
    match submode {
        "a30" => decode_both!(Q65a30),
        "a60" => decode_both!(Q65a60),
        "b60" => decode_both!(Q65b60),
        "c60" => decode_both!(Q65c60),
        "d60" => decode_both!(Q65d60),
        "e60" => decode_both!(Q65e60),
        _ => (false, false),
    }
}

#[test]
#[ignore]
fn q65_awgn_snr_sweep() {
    let dir = sweep_dir();
    let Ok(entries) = std::fs::read_dir(&dir) else {
        eprintln!(
            "skipping q65_awgn_snr_sweep: corpus dir not found at {:?}\n\
             Run scripts/gen_q65_sweep_wavs.sh ~/wsjtx-build/q65sim",
            dir
        );
        return;
    };

    let cq_hint = ApHint::new().with_call1("CQ");

    // (submode, snr) -> (plain_hits, cq_hits, trials)
    let mut cells: std::collections::BTreeMap<(String, i32), (u32, u32, u32)> =
        std::collections::BTreeMap::new();

    for entry in entries.flatten() {
        let path = entry.path();
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        let Some(rest) = stem.strip_prefix("q65_") else {
            continue;
        };
        let Some((submode, rest)) = rest.split_once("_awgn_") else {
            continue;
        };
        if !SUBMODES.contains(&submode) {
            continue;
        }
        let Some((tag, _trial)) = rest.split_once('_') else {
            continue;
        };
        let Some(snr) = parse_snr_tag(tag) else {
            continue;
        };
        let Some(audio) = load_wav_f32_opt(&path) else {
            continue;
        };
        let (plain_hit, cq_hit) = decode_wav_q65(submode, &audio, &cq_hint);
        let cell = cells.entry((submode.to_string(), snr)).or_insert((0, 0, 0));
        cell.2 += 1;
        if plain_hit {
            cell.0 += 1;
        }
        if cq_hit {
            cell.1 += 1;
        }
    }

    if cells.is_empty() {
        eprintln!(
            "skipping q65_awgn_snr_sweep: no q65_*_awgn_*.wav files found in {:?}",
            dir
        );
        return;
    }

    println!("Q65 AWGN SNR sweep — {:?}", dir);
    println!(
        "(plain = decode_scan_for; CQ = decode_scan_with_ap_for + \"CQ\" hint — see module docs, issue #171)"
    );
    for submode in SUBMODES {
        println!("\n-- Q65-{submode} --");
        println!(
            "{:>6}  {:>12}  {:>6}  {:>12}  {:>6}",
            "SNR", "plain hits", "pct", "CQ hits", "pct"
        );
        for ((sm, snr), (plain_hits, cq_hits, trials)) in &cells {
            if sm != submode {
                continue;
            }
            let plain_pct = *plain_hits as f32 / *trials as f32 * 100.0;
            let cq_pct = *cq_hits as f32 / *trials as f32 * 100.0;
            println!(
                "{:>+5}dB  {:>12}  {:5.1}%  {:>12}  {:5.1}%",
                snr,
                format!("{plain_hits}/{trials}"),
                plain_pct,
                format!("{cq_hits}/{trials}"),
                cq_pct
            );
        }
    }
}
