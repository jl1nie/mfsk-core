//! Tier A: `decoder::params_read(mode)` against the decoders (#698).
//!
//! The table is published so an application can hide the settings a mode
//! does not read. The wrong answer that costs decodes is a field left out of
//! a mode's set that its decoder reads, so `unread_fields_change_nothing`
//! sets every field outside the set to a value chosen to change the result
//! if it were read (a band off the signal, a tolerance of 1 Hz, the sender's
//! own call as the QSO partner, ...) and requires the decode to be the same.
//! It catches a dropped field only where the poison reaches a strong signal:
//! dropping `BandHz` from the FT family fails it on FT8, FT4 and FST4-15,
//! but dropping `RxFreqHz` from JT9 does not, because JT9's Rx window is an
//! extra pass near the Rx frequency that a strong signal elsewhere does not
//! need (both tried when this test was written).
//!
//! Cells the poison cannot reach at all, because the read sits behind
//! settings `default_params` leaves off (a station call, both QSO calls, an Rx
//! frequency, AP on) and a strong signal decodes without AP anyway, so this
//! test does not check them: FT4 / FST4 `tx_freq_hz` (the both-callsign AP
//! rule), Q65 `qso.progress` (its AP list), FST4 `tol_hz` (the noise-blanker
//! sweep is off), and every "only with `rx_freq_hz`" read. Their empty cells
//! rest on the code and the review of #698.
//! A field inside the set is only checked where a deterministic probe exists
//! (`read_fields_are_reachable`: the band and the Rx window); the AP context,
//! contest, EME delay, depth, averaging and deep-search cells need a signal
//! at threshold to show an effect, and rest on the decoder lines
//! `params_read`'s doc cites.
//!
//! One mode per decoder implementation: the FST4, FST4W and Q65 sub-modes
//! each share one (`decoder/frame.rs`, `fst4w.rs`, `q65.rs` macros), and
//! `every_sub_mode_has_its_familys_set` holds them to it.
#![cfg(all(
    feature = "ft8",
    feature = "ft4",
    feature = "fst4",
    feature = "fst4w",
    feature = "wspr",
    feature = "jt9",
    feature = "jt65",
    feature = "q65",
    any(feature = "fft-rustfft", feature = "fft-extern")
))]

use mfsk_core::Mode;
use mfsk_core::decoder::{
    AnyDecoder, AnySlotResult, ApMode, Contest, DecodeParams, ParamField, QsoProgress,
    default_params, params_read,
};
use mfsk_core::encoder::{TxAudio, encode};

const FS: u32 = 12_000;
/// Peak amplitude of the frame, against noise of about +-300.
const AMP: f32 = 3_000.0;

/// One mode per decoder implementation.
const REPRESENTATIVES: [&str; 8] = [
    "FT8",
    "FT4",
    "FST4-15",
    "FST4W-120",
    "WSPR",
    "JT9",
    "JT65",
    "Q65-15A",
];

/// The text sent, the sender's call and grid, and the carrier (inside the
/// mode's default band, as `tests/encoder.rs` places it).
fn message(mode: Mode) -> (&'static str, &'static str, &'static str, f32) {
    match mode.name() {
        "WSPR" => ("K1ABC FN42 37", "K1ABC", "FN42", 1_500.0),
        n if n.starts_with("FST4W") => ("K1ABC FN42 37", "K1ABC", "FN42", 1_500.0),
        n if n.starts_with("FST4") => ("CQ JA1ABC PM95", "JA1ABC", "PM95", 1_000.0),
        _ => ("CQ JA1ABC PM95", "JA1ABC", "PM95", 1_500.0),
    }
}

/// The frame at the mode's nominal start in deterministic noise
/// (`tests/encoder.rs`'s `period_of`).
fn period(mode: Mode, seed: u32) -> Vec<i16> {
    let (text, _, _, f0) = message(mode);
    let frame = encode(mode, text)
        .unwrap_or_else(|e| panic!("{}: {e}", mode.name()))
        .synthesize(&TxAudio::new(FS, f0));
    let meta = mode.meta();
    let slot = meta.slot_samples_12k as usize;
    let start = (meta.tx_start_offset_s * FS as f32).round() as usize;
    let mut x = seed.wrapping_mul(2_654_435_761).wrapping_add(1);
    (0..slot)
        .map(|i| {
            x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let n = ((x >> 16) as i16 / 109) as f32;
            let s = i.checked_sub(start).and_then(|k| frame.get(k)).copied();
            (s.unwrap_or(0.0) * AMP + n).clamp(-32_768.0, 32_767.0) as i16
        })
        .collect()
}

/// What a decode produced that a setting could change: the rows, in full,
/// and the candidate count.
fn outcome(r: &AnySlotResult) -> String {
    format!("{:?} candidates={:?}", r.rows, r.candidates)
}

/// Two consecutive periods of the same frame (different noise), so a
/// setting that averages has something to average. Returns both outcomes.
fn decode_two(mode: Mode, params: DecodeParams, audio: &[Vec<i16>; 2]) -> [String; 2] {
    let mut d = AnyDecoder::new(mode, params);
    let a = outcome(&d.decode_i16(&audio[0], Some(0)));
    let b = outcome(&d.decode_i16(&audio[1], Some(1)));
    [a, b]
}

/// `params` with `field` set to a value that changes the decode if the mode
/// reads it.
fn poison(mode: Mode, field: ParamField, mut p: DecodeParams) -> DecodeParams {
    let (_, call, grid, f0) = message(mode);
    match field {
        ParamField::BandHz => p.band_hz = (3_500.0, 3_700.0),
        ParamField::RxFreqHz => p.rx_freq_hz = Some(3_600.0),
        ParamField::TolHz => p.tol_hz = Some(1.0),
        ParamField::TxFreqHz => p.tx_freq_hz = Some(f0),
        ParamField::Depth => p.depth = mfsk_core::decoder::Depth::Fast,
        ParamField::Averaging => p.averaging = true,
        ParamField::DeepSearch => p.deep_search = true,
        ParamField::StationCall => p.station.call = call.into(),
        ParamField::StationGrid => p.station.grid = grid.into(),
        ParamField::QsoHisCall => p.qso.his_call = call.into(),
        ParamField::QsoHisGrid => p.qso.his_grid = grid.into(),
        ParamField::QsoProgress => p.qso.progress = QsoProgress::RogerReport,
        ParamField::Ap => {
            p.ap = if p.ap == ApMode::Off {
                ApMode::Full
            } else {
                ApMode::Off
            }
        }
        ParamField::Contest => p.contest = Contest::EuVhf,
        ParamField::EmeDelay => p.eme_delay = true,
        other => panic!("no poison for {other:?}"),
    }
    p
}

/// The decoders ignore every field `params_read` leaves out. Fails if a mode's
/// set drops a field its decoder reads and the poison reaches it.
#[test]
fn unread_fields_change_nothing() {
    let mut failures = Vec::new();
    for name in REPRESENTATIVES {
        let mode = Mode::from_name(name).expect(name);
        let audio = [period(mode, 5), period(mode, 6)];
        let base = decode_two(mode, default_params(mode), &audio);
        let (text, ..) = message(mode);
        assert!(
            base[0].contains(text),
            "{name}: baseline does not decode {text:?}: {}",
            base[0]
        );
        let read = params_read(mode);
        for &field in ParamField::ALL {
            if read.contains(field) {
                continue;
            }
            let got = decode_two(mode, poison(mode, field, default_params(mode)), &audio);
            if got != base {
                failures.push(format!(
                    "{name}: {field:?} is outside params_read but changed the decode\n  base {:?}\n  got  {:?}",
                    base, got
                ));
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

/// The fields with a deterministic probe are read where the set says so.
/// Fails if `band_hz` is claimed for a mode that ignores it (as it was for
/// WSPR when this test was written: wsprd's search takes no band), or if
/// FST4W's Rx window is not its search.
#[test]
fn read_fields_are_reachable() {
    for name in REPRESENTATIVES {
        let mode = Mode::from_name(name).expect(name);
        let (text, ..) = message(mode);
        let audio = [period(mode, 5), period(mode, 6)];
        let read = params_read(mode);
        if read.contains(ParamField::BandHz) {
            // FST4W reads the band only as the fallback of an unset Rx window,
            // which is what `default_params` leaves it.
            let off = poison(mode, ParamField::BandHz, default_params(mode));
            let got = decode_two(mode, off, &audio);
            assert!(
                !got[0].contains(text),
                "{name}: band off the signal still decodes: {}",
                got[0]
            );
        }

        // Where the Rx window *is* the search (FST4W's `nfqso ± ntol`), a
        // window off the signal loses it. JT9's window is an extra pass over
        // the wide scan (`jt9_decode.f90`'s `nqd = 1`) and Q65's places the
        // AP-list and averaged q3 decodes (`q65_decode.f90:263-272`), so a
        // strong signal decodes either way and no probe is made for them.
        if name == "FST4W-120" {
            assert!(read.contains(ParamField::RxFreqHz) && read.contains(ParamField::TolHz));
            let mut p = default_params(mode);
            p.rx_freq_hz = Some(3_600.0);
            p.tol_hz = Some(10.0);
            let got = decode_two(mode, p, &audio);
            assert!(
                !got[0].contains(text),
                "{name}: an Rx window off the signal still decodes: {}",
                got[0]
            );
        }
    }
}

/// Every sub-mode has its family's set, and `station.grid` is in none
/// (no upstream decoder reads `mygrid`).
#[test]
fn every_sub_mode_has_its_familys_set() {
    let family = |m: Mode| match m.name() {
        n if n.starts_with("FST4W") => "FST4W-120",
        n if n.starts_with("FST4") => "FST4-15",
        n if n.starts_with("Q65") => "Q65-15A",
        n => n,
    };
    for &m in Mode::ALL {
        let rep = Mode::from_name(family(m)).unwrap();
        assert_eq!(params_read(m), params_read(rep), "{}", m.name());
        assert!(
            !params_read(m).contains(ParamField::StationGrid),
            "{}",
            m.name()
        );
        assert!(params_read(m).contains(ParamField::Depth), "{}", m.name());
    }
}
