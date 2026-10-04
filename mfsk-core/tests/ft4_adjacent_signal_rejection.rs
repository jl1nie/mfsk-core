// SPDX-License-Identifier: GPL-3.0-only
//! Can the FT4 receiver still hear its signal with a strong neighbour
//! on the band? — the question none of the existing fixtures ask.
//!
//! `ft4_sweep` and `scripts/run-sensitivity-sweeps.sh ft4` are four
//! ITU-R channels of **single-signal** `ft4sim` files: they measure the
//! noise floor and say nothing about selectivity. `ft4_crowded_band`
//! does place many signals in one slot, but from a generated tier-C
//! corpus at frequencies the simulator chose, so it cannot put an
//! interferer on a specific offset. The WSJT-X golden has fourteen
//! signals in fixed positions. So the front end's rejection — the whole
//! purpose of `ft4::ddc`'s 101 + 263 taps — is currently unmeasured.
//!
//! That matters because the front end is the thing most likely to
//! change. Both cheaper baseband producers on the table replace a
//! designed low-pass with something blunter:
//!
//! - a boxcar-and-decimate of the kind FT8's `fine_sync_12k` uses, whose
//!   sinc nulls sit on the fold centres but whose rejection between them
//!   is tens of dB rather than infinite; and
//! - anything else that drops the sharp stage.
//!
//! Against AWGN the difference is fractions of a dB and every existing
//! test would pass. Against a neighbour on a fold it is the difference
//! between a decode and a phantom. **This file is the baseline those
//! changes have to be measured against**, taken with today's FIR chain.
//!
//! ## Where the offsets come from
//!
//! `ft4::ddc` mixes `f0 + 31.25 Hz` — the centre of the reference band
//! `[f0 - 31.25, f0 + 93.75]` — to DC and decimates to
//! `12 000 / 18 = 666.667 Hz`. Anything at `centre + k * 666.667 Hz`
//! therefore folds onto the wanted signal once the sharp filter stops
//! removing it, and an interferer whose own carrier sits at
//! `f0 + 31.25 + k * 666.667` lands its four tones exactly where the
//! wanted signal's are. Those are the cells that matter; the rest are
//! controls that say the instrument is not blind.
//!
//! ```sh
//! cargo test -p mfsk-core --features full,internal-testing --release \
//!     --test ft4_adjacent_signal_rejection -- --nocapture
//! ```
#![cfg(all(
    feature = "ft4",
    feature = "internal-testing",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use mfsk_core::engine::{MessageCodec, MessageFields};

#[allow(dead_code)]
mod common;
use common::channel::AwgnChannel;
use common::ft4_rx_mirror as rx;

const FS: f32 = 12_000.0;
/// The bandwidth SNR is quoted in, as everywhere else in this crate.
const REF_BW: f32 = 2_500.0;
/// The wanted signal's carrier. Chosen so every offset below stays
/// inside the receiver's own `[100, 2700] Hz` search band.
const WANTED_HZ: f32 = 700.0;
/// Comfortably above threshold on its own: this file measures what a
/// neighbour takes away, so the baseline has to be a decode.
const WANTED_SNR_DB: f32 = -8.0;
/// `TX_START_OFFSET_S`.
const START_S: f32 = 0.5;

/// `ft4::ddc`'s band centre relative to the candidate's carrier.
const BAND_CENTRE_HZ: f32 = 31.25;
/// The baseband rate the per-candidate chain decimates to, and
/// therefore the spacing of the images a blunt filter would let fold.
const DS_RATE_HZ: f32 = FS / 18.0;
const TONE_SPACING_HZ: f32 = 20.833_334;

/// Amplitude for a given SNR against unit-variance noise.
fn amp_for(snr_db: f32) -> f32 {
    let snr_lin = 10f32.powf(snr_db / 10.0);
    (4.0 * snr_lin * REF_BW / FS).sqrt()
}

fn pack(call1: &str, call2: &str, grid: &str) -> [u8; 77] {
    let bits = mfsk_core::msg::Wsjt77Message
        .pack(&MessageFields {
            call1: Some(call1.into()),
            call2: Some(call2.into()),
            grid: Some(grid.into()),
            ..Default::default()
        })
        .expect("message packs");
    let mut out = [0u8; 77];
    out.copy_from_slice(&bits);
    out
}

fn add_signal(mix: &mut [f32], msg: &[u8; 77], freq_hz: f32, amp: f32) {
    let itone = mfsk_core::engine::tx::message_to_tones::<mfsk_core::ft4::Ft4>(msg);
    let pcm =
        mfsk_core::engine::tx::synthesize::<mfsk_core::ft4::Ft4>(&itone, 12_000, freq_hz, amp);
    let start = (START_S * FS) as usize;
    for (i, &s) in pcm.iter().enumerate() {
        if start + i < mix.len() {
            mix[start + i] += s;
        }
    }
}

/// One slot: the wanted signal, optionally a neighbour, and noise.
///
/// Both signals start at the same instant. That is the harsh case on
/// purpose — a neighbour that folds onto the wanted band lands its
/// Costas arrays on top of the wanted ones — and it is the case a
/// selectivity claim has to survive.
fn slot(interferer: Option<(f32, f32)>) -> Vec<i16> {
    slot_at(WANTED_SNR_DB, interferer, 0x5EED_1234)
}

/// [`slot`] with the wanted signal's SNR and the noise seed chosen —
/// what the threshold sweep varies.
fn slot_at(snr_db: f32, interferer: Option<(f32, f32)>, seed: u64) -> Vec<i16> {
    let mut mix = vec![0.0f32; rx::SLOT_SAMPLES];
    add_signal(&mut mix, &wanted(), WANTED_HZ, amp_for(snr_db));
    if let Some((offset_hz, level_db)) = interferer {
        add_signal(
            &mut mix,
            &other(),
            WANTED_HZ + offset_hz,
            amp_for(snr_db + level_db),
        );
    }
    // Deterministic noise, not a silent slot: a noiseless synthetic
    // fixture flatters every decoder and has misled this repository
    // before.
    AwgnChannel::new(1.0, seed).apply(&mut mix);
    let peak = mix.iter().map(|x| x.abs()).fold(0.0f32, f32::max).max(1e-6);
    let scale = 29_000.0 / peak;
    mix.iter()
        .map(|&s| (s * scale).clamp(-32_768.0, 32_767.0) as i16)
        .collect()
}

/// The two messages. Different calls and different grids, so their
/// data symbols differ everywhere the Costas arrays do not — an
/// interferer that happened to carry the same payload would be a
/// second copy of the wanted signal rather than interference.
fn wanted() -> [u8; 77] {
    pack("JA1ABC", "W1AW", "PM95")
}

fn other() -> [u8; 77] {
    pack("VK3XYZ", "G0ABC", "QF22")
}

/// The cells, and why each one is here.
fn cases() -> Vec<(&'static str, f32)> {
    vec![
        (
            "overlapping (+1 tone)  — positive control, must hurt",
            TONE_SPACING_HZ,
        ),
        ("adjacent  +100 Hz      — outside the reference band", 100.0),
        (
            "mid-null  +364.6 Hz    — least boxcar rejection, folds clear of the band",
            BAND_CENTRE_HZ + DS_RATE_HZ / 2.0,
        ),
        (
            "fold k=1  +697.9 Hz    — folds to band centre, ON the boxcar null",
            BAND_CENTRE_HZ + DS_RATE_HZ,
        ),
        (
            "fold k=2  +1364.6 Hz   — the second image, also on a null",
            BAND_CENTRE_HZ + 2.0 * DS_RATE_HZ,
        ),
        // **The worst case, and the one the first version of this file
        // missed.** `k * 666.667` is exactly where a 9-sample boxcar's
        // sinc null sits, so an interferer placed there is the one it
        // rejects *best* — the fold rows above pass for a reason that
        // has nothing to do with selectivity. Sliding the carrier down
        // by the frame's own width keeps every folded tone inside the
        // wanted band while moving it as far off the null as it can go.
        (
            "fold k=1 off-null low   +635.4 Hz — folds in band, worst rejection",
            BAND_CENTRE_HZ + DS_RATE_HZ - 62.5,
        ),
        (
            "fold k=1 off-null high  +760.4 Hz — folds onto the top tones",
            BAND_CENTRE_HZ + DS_RATE_HZ + 62.5,
        ),
    ]
}

#[test]
fn ft4_front_end_rejects_a_neighbour_off_the_wanted_band() {
    let wanted = mfsk_core::msg::wsjt77::unpack77(&wanted()).expect("wanted unpacks");
    let other = mfsk_core::msg::wsjt77::unpack77(&other()).expect("interferer unpacks");

    let alone = rx::run_slot(&slot(None));
    eprintln!("\nFT4 adjacent-signal rejection — today's 101+263-tap front end");
    eprintln!("  wanted {wanted:?} at {WANTED_HZ} Hz, {WANTED_SNR_DB} dB");
    eprintln!("  neighbour {other:?}");
    eprintln!(
        "  alone: {} decode(s), wanted {}",
        alone.len(),
        alone.contains(&wanted)
    );
    assert!(
        alone.contains(&wanted),
        "the wanted signal must decode on its own, or the fixture measures nothing"
    );

    eprintln!(
        "\n  {:<62} {:>6} {:>6} {:>6}",
        "case", "+0 dB", "+10", "+20"
    );
    let mut lost = Vec::new();
    for (name, offset) in cases() {
        let mut cells = Vec::new();
        for level in [0.0f32, 10.0, 20.0] {
            let got = rx::run_slot(&slot(Some((offset, level))));
            let hit = got.contains(&wanted);
            cells.push(if hit { "ok" } else { "LOST" });
            if !hit {
                lost.push((name, level));
            }
        }
        eprintln!(
            "  {name:<62} {:>6} {:>6} {:>6}",
            cells[0], cells[1], cells[2]
        );
    }
    eprintln!();

    // The baseline, asserted rather than printed and forgotten: today's
    // chain loses the wanted signal only where the neighbour is *on top
    // of it*, and survives every fold at +20 dB. A front end that
    // trades taps for arithmetic has to reproduce this table.
    let overlapping = cases()[0].0;
    let expected_lost = [
        (overlapping, 0.0f32),
        (overlapping, 10.0),
        (overlapping, 20.0),
    ];
    assert_eq!(
        lost, expected_lost,
        "the rejection baseline moved — see the table above"
    );
}

/// What the boxcar front end costs on the same table.
///
/// This is the question the file was built to answer. The FIR arm
/// above survives a +20 dB neighbour everywhere; the proposal is to
/// replace its 101 + 263 taps with a mix-and-bin pass costing a
/// sixteenth of the arithmetic, and `the_fold_offsets_really_do_fold`
/// measures that such a pass admits 44 dB more at the fold offsets.
/// Whether 44 dB matters depends on whether the coherent Costas
/// correlation and the LLR stage can absorb it, which is not something
/// to reason about — so this runs it.
///
/// Diagnostic rather than a gate: it prints both arms side by side. The
/// FIR arm's assertions live in the test above and are the contract.
#[test]
fn what_the_boxcar_front_end_costs_against_a_neighbour() {
    let wanted = mfsk_core::msg::wsjt77::unpack77(&wanted()).expect("wanted unpacks");

    let alone_fir = rx::run_slot_with(&slot(None), rx::Variant::SHIPPED);
    let alone_box = rx::run_slot_with(&slot(None), rx::Variant::BOXCAR);
    eprintln!("\nFT4 adjacent-signal rejection — FIR against boxcar/9");
    eprintln!(
        "  alone: FIR {}, boxcar {}",
        alone_fir.contains(&wanted),
        alone_box.contains(&wanted)
    );

    eprintln!(
        "  {:<62} {:>15} {:>15}",
        "case", "FIR +0/+10/+20", "box +0/+10/+20"
    );
    for (name, offset) in cases() {
        let mut fir = String::new();
        let mut boxcar = String::new();
        for level in [0.0f32, 10.0, 20.0] {
            let audio = slot(Some((offset, level)));
            fir.push_str(
                if rx::run_slot_with(&audio, rx::Variant::SHIPPED).contains(&wanted) {
                    " ok "
                } else {
                    "LOST"
                },
            );
            boxcar.push_str(
                if rx::run_slot_with(&audio, rx::Variant::BOXCAR).contains(&wanted) {
                    " ok "
                } else {
                    "LOST"
                },
            );
        }
        eprintln!("  {name:<62} {fir:>15} {boxcar:>15}");
    }
    eprintln!();
}

/// **The number that decides the cheaper front end**: how much
/// threshold does it cost?
///
/// The tables above are taken at −8 dB, where the wanted signal has
/// margin to spare, and they say the boxcar loses nothing. That is not
/// the same as saying it costs nothing: a blunter filter admits more
/// noise and more of a neighbour, and what that buys is paid at
/// threshold, not in the middle of the range. So this sweeps the wanted
/// signal down until it stops decoding, with and without the worst
/// neighbour, and reports where each arm crosses 50 %.
///
/// `#[ignore]` — a few seconds rather than the milliseconds the rest of
/// this file costs:
///
/// ```sh
/// cargo test -p mfsk-core --features full,internal-testing --release \
///     --test ft4_adjacent_signal_rejection -- --ignored --nocapture
/// ```
#[test]
#[ignore = "sweep — seconds, not milliseconds; run with --ignored"]
fn what_the_boxcar_front_end_costs_at_threshold() {
    const SEEDS: u64 = 32;
    let wanted = mfsk_core::msg::wsjt77::unpack77(&wanted()).expect("wanted unpacks");
    // The worst geometry the table above found: folds into the band,
    // as far off the boxcar's null as it can get.
    let worst = BAND_CENTRE_HZ + DS_RATE_HZ - 62.5;

    // Coarse above threshold, half-decibel through it: a 1 dB grid
    // cannot resolve a difference this file expects to be under 1 dB.
    let snrs: Vec<f32> = (0..=16).map(|i| -14.0 - i as f32 * 0.5).collect();
    let arms: [(&str, Option<(f32, f32)>); 2] =
        [("alone", None), ("+20 dB fold", Some((worst, 20.0)))];

    // Four arms: what ships, each half of the cheaper front end, and
    // both together.
    let variants: [(&str, rx::Variant); 5] = [
        ("shipped", rx::Variant::SHIPPED),
        ("boxcar", rx::Variant::BOXCAR),
        ("binned", rx::Variant::BINNED_SEARCH),
        ("both", rx::Variant::BOTH),
        ("snapped", rx::Variant::SNAPPED),
    ];

    eprintln!("\nFT4 threshold — {SEEDS} seeds per point, decode rate %");
    eprint!("  {:<8}", "SNR");
    for (name, _) in &variants {
        eprint!("  {:>10}", format!("{name} alone"));
        eprint!("  {:>10}", format!("{name} fold"));
    }
    eprintln!();

    let mut curves: Vec<[Vec<(f32, f32)>; 2]> =
        variants.iter().map(|_| [Vec::new(), Vec::new()]).collect();
    for &snr in &snrs {
        eprint!("  {snr:<8.1}");
        for (vi, (_, v)) in variants.iter().enumerate() {
            for (ai, (_, interferer)) in arms.iter().enumerate() {
                let mut hits = 0;
                for seed in 0..SEEDS {
                    let audio = slot_at(snr, *interferer, 0x5EED_0000 + seed);
                    if rx::run_slot_with(&audio, *v).contains(&wanted) {
                        hits += 1;
                    }
                }
                let rate = hits as f32 / SEEDS as f32;
                curves[vi][ai].push((snr, rate));
                eprint!("  {:>10.0}", rate * 100.0);
            }
        }
        eprintln!();
    }

    // Linear interpolation of the 50 % crossing, walking down from the
    // top — the same shape `sweep-regression-check.py` uses on the
    // tier-C CSVs.
    let cross = |c: &Vec<(f32, f32)>| -> f32 {
        for w in c.windows(2) {
            if w[0].1 >= 0.5 && w[1].1 < 0.5 {
                let t = (w[0].1 - 0.5) / (w[0].1 - w[1].1);
                return w[0].0 + t * (w[1].0 - w[0].0);
            }
        }
        f32::NAN
    };
    let base = [cross(&curves[0][0]), cross(&curves[0][1])];
    eprintln!("\n  50 % crossing, and the cost against what ships");
    eprintln!(
        "  {:<10} {:>10} {:>10} {:>10} {:>10}",
        "arm", "alone", "vs ship", "fold", "vs ship"
    );
    for (vi, (name, _)) in variants.iter().enumerate() {
        let a = cross(&curves[vi][0]);
        let f = cross(&curves[vi][1]);
        // Positive = needs a stronger signal, i.e. costs sensitivity.
        eprintln!(
            "  {name:<10} {a:>10.2} {:>10.2} {f:>10.2} {:>10.2}",
            a - base[0],
            f - base[1]
        );
    }
    eprintln!();
}

/// Do the fold offsets actually fold?
///
/// The table above is only meaningful if its "fold" rows are where a
/// blunt decimator would let a neighbour through. That is an arithmetic
/// claim about the offsets, and this checks it directly rather than
/// leaving it to a comment: mix an interferer-only slot to the wanted
/// candidate's band centre and decimate by nine with a **boxcar** —
/// the cheapest producer anyone would reach for — then compare how much
/// of it lands in band.
///
/// Today's FIR chain is the control: the same interferer, through
/// `candidate_baseband_half`, should leave nothing.
#[test]
fn the_fold_offsets_really_do_fold() {
    use num_complex::Complex;

    /// Mix at `f_c` and boxcar-decimate by nine, from the 6 kHz shared
    /// stream — the shape `ft4::ddc`'s two FIR stages would be replaced
    /// by.
    fn boxcar_baseband(half: &[f32], f_c: f32) -> Vec<Complex<f32>> {
        let half_rate = FS / 2.0;
        let mut out = Vec::with_capacity(half.len() / 9 + 1);
        let mut acc = Complex::new(0.0f32, 0.0);
        for (n, &x) in half.iter().enumerate() {
            let phi = -core::f32::consts::TAU * f_c * n as f32 / half_rate;
            acc += Complex::new(phi.cos(), phi.sin()) * x;
            if n % 9 == 8 {
                out.push(acc / 9.0);
                acc = Complex::new(0.0, 0.0);
            }
        }
        out
    }

    /// Power inside the reference band only.
    ///
    /// **Not total power** — that was the first version of this, and it
    /// ranked the non-folding +100 Hz neighbour highest, because a
    /// signal 100 Hz off the centre sits comfortably inside the
    /// ±333 Hz baseband without folding at all. What decides whether a
    /// neighbour reaches the decoder is how much of it lands in the
    /// ±62.5 Hz the four tones occupy, so that is what this integrates,
    /// by direct DFT across the band.
    fn band_power(v: &[Complex<f32>], rate_hz: f32) -> f32 {
        let mut total = 0.0f32;
        let mut f = -62.5f32;
        while f <= 62.5 {
            let mut acc = Complex::new(0.0f32, 0.0);
            for (n, &x) in v.iter().enumerate() {
                let phi = -core::f32::consts::TAU * f * n as f32 / rate_hz;
                acc += Complex::new(phi.cos(), phi.sin()) * x;
            }
            total += acc.norm_sqr() / (v.len().max(1) as f32);
            f += 5.0;
        }
        total
    }

    // Interferer only, so whatever lands in band came from it.
    let centre = WANTED_HZ + BAND_CENTRE_HZ;
    let mut rows = Vec::new();
    for (name, offset) in cases() {
        let mut mix = vec![0.0f32; rx::SLOT_SAMPLES];
        add_signal(&mut mix, &other(), WANTED_HZ + offset, amp_for(20.0));
        let pcm: Vec<i16> = mix.iter().map(|&s| (s * 4_000.0) as i16).collect();
        let (_savg, half) = rx::capture(&pcm);
        let boxcar = band_power(&boxcar_baseband(&half, centre), FS / 2.0 / 9.0);
        let fir = band_power(
            &mfsk_core::ft4::ddc::candidate_baseband_half(&half, WANTED_HZ),
            FS / 18.0,
        );
        rows.push((name, offset, boxcar, fir));
    }

    eprintln!("\n  in-band power from an interferer-only slot (+20 dB), by producer:");
    eprintln!(
        "  {:<62} {:>12} {:>12} {:>9}",
        "case", "boxcar/9", "FIR 101+263", "ratio dB"
    );
    for (name, _, boxcar, fir) in &rows {
        let ratio_db = 10.0 * (boxcar / fir.max(f32::MIN_POSITIVE)).log10();
        eprintln!("  {name:<62} {boxcar:>12.3e} {fir:>12.3e} {ratio_db:>9.1}");
    }
    eprintln!();

    // **The comparison is boxcar against FIR at the same offset**, not
    // one offset against another. An earlier version compared the fold
    // rows to the +100 Hz row and failed, correctly: +100 Hz is
    // adjacent, so its skirt reaches into the band under *any*
    // producer, and it legitimately carries more in-band power than a
    // fold does. What makes an offset a hazard is how much more a blunt
    // decimator admits there than the designed chain does.
    for &(name, _, boxcar, fir) in &rows[3..] {
        assert!(
            boxcar > fir * 1_000.0,
            "{name}: boxcar {boxcar:.3e} vs FIR {fir:.3e} — under 30 dB apart, \
             so this offset is not the fold the row claims, and the rejection \
             table above proves nothing about it"
        );
    }
    // The control: the mid-null offset is attenuated by the boxcar
    // *and* lands clear of the band, so the two producers stay close.
    // If this ever separates, the offsets have drifted.
    let (name, _, boxcar, fir) = rows[2];
    assert!(
        boxcar < fir * 1_000.0,
        "{name}: boxcar {boxcar:.3e} vs FIR {fir:.3e} — this is supposed to be \
         the row that does *not* fold"
    );
}
