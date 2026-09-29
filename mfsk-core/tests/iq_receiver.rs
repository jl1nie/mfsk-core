//! Phase 2 of #534: `IqReceiver` — N channels of one stream, slots cut on
//! UTC from the sample count, absolute frequency in the row.
//!
//! Real recordings placed as IQ (double sideband, see `iq_front_end`): an
//! FT8 recording and an FT4 one in the *same* stream on different dials. The
//! contract is the WAV path's, with the registry's default search: the same
//! messages, at the same audio frequency and DT, plus the RF frequency the
//! dial makes of it.
#![cfg(all(feature = "ft8", feature = "ft4", feature = "fft-rustfft"))]

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

use mfsk_core::engine::pipeline::DecodeResult;
use mfsk_core::iq::{ChannelId, IqDecode, IqError, IqMode, IqReceiver, IqSampleFormat, IqStream};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::{Ft4, Ft8, by_name};

#[allow(dead_code)]
mod common;
use common::iq::{add_into, interleave, synth_iq};

const FT8_WAV: &str = asset_path!("qso3_busy.wav");
const FT4_WAV: &str = asset_path!("golden/ft4/000000_000002.wav");

const FS: u32 = 192_000;
const CENTER: f64 = 14_077_000.0;
const FT8_DIAL: f64 = CENTER + 20_000.0;
const FT4_DIAL: f64 = CENTER - 50_000.0;
/// A multiple of 15 s and of 7.5 s, so both grids start on it.
const T0_NS: i64 = 1_700_000_010 * 1_000_000_000;

type Set = BTreeMap<String, (f32, f32)>;

fn reference<P: mfsk_core::msg::decode_request::FrameDecodable<DecodeResult = DecodeResult>>(
    name: &str,
    audio: &[i16],
    id: mfsk_core::engine::protocol::ProtocolId,
) -> Set {
    let d = by_name(name).unwrap().profile.defaults;
    DecodeRequest::<P>::new(
        audio,
        d.freq_min_hz,
        d.freq_max_hz,
        d.sync_min,
        d.max_cand as usize,
    )
    .decode()
    .results
    .iter()
    .filter_map(|r| r.to_decoded(id, None))
    .map(|d| (d.text, (d.freq_hz, d.dt_sec)))
    .collect()
}

struct Scene {
    ft8: Vec<i16>,
    ft4: Vec<i16>,
    ft8_ref: Set,
    ft4_ref: Set,
}

fn scene() -> Option<Scene> {
    let ft8 = common::load_wav_i16_opt(FT8_WAV)?;
    let mut ft4 = common::load_wav_i16_opt(FT4_WAV)?;
    ft4.resize(90_000, 0); // 6.048 s recording in a 7.5 s slot
    let ft8_ref = reference::<Ft8>("FT8", &ft8, mfsk_core::engine::protocol::ProtocolId::Ft8);
    let ft4_ref = reference::<Ft4>("FT4", &ft4, mfsk_core::engine::protocol::ProtocolId::Ft4);
    assert!(
        ft8_ref.len() >= 10,
        "FT8 WAV path decoded {}",
        ft8_ref.len()
    );
    assert!(ft4_ref.len() >= 8, "FT4 WAV path decoded {}", ft4_ref.len());
    Some(Scene {
        ft8,
        ft4,
        ft8_ref,
        ft4_ref,
    })
}

/// The front end drops its own group delay, so a slot's last audio sample
/// comes out a few filter lengths after the recording's last IQ sample; a
/// live stream supplies that by carrying on, a test by padding.
fn pad(mut iq: Vec<(f32, f32)>) -> Vec<(f32, f32)> {
    iq.resize(iq.len() + FS as usize / 2, (0.0, 0.0));
    iq
}

fn known_ft8() -> Vec<&'static str> {
    common::ft8_qso3::QSO3_KNOWN_REAL_SIGNALS
        .iter()
        .map(|e| e.msg)
        .collect()
}

fn stream() -> IqStream {
    IqStream {
        sample_rate: FS,
        center_hz: CENTER,
        format: IqSampleFormat::Cf32,
        iq_swap: false,
    }
}

fn receiver() -> (IqReceiver, Arc<Mutex<Vec<IqDecode>>>) {
    let mut rx = IqReceiver::new(stream());
    let rows = Arc::new(Mutex::new(Vec::new()));
    let sink = rows.clone();
    rx.on_decode(move |r| sink.lock().unwrap().push(r.clone()));
    (rx, rows)
}

fn set_of(rows: &[IqDecode], ch: ChannelId) -> Set {
    rows.iter()
        .filter(|r| r.channel == ch)
        .map(|r| {
            (
                r.decoded.text.clone(),
                (r.decoded.freq_hz, r.decoded.dt_sec),
            )
        })
        .collect()
}

/// `a` (through IQ) against `b` (the WAV path). A decode `b` lacks is a
/// phantom unless it is one of `known`, the real signals in the recording:
/// the weakest sit at the search's edge and a last-bit numeric difference
/// (the front end's filters) can tip one either way.
fn same(a: &Set, b: &Set, what: &str, known: &[&str]) {
    let missing: Vec<_> = b.keys().filter(|k| !a.contains_key(*k)).collect();
    let extra: Vec<_> = a
        .keys()
        .filter(|k| !b.contains_key(*k) && !known.contains(&k.as_str()))
        .collect();
    println!(
        "  {what}: ref {} got {} missing {} extra {}",
        b.len(),
        a.len(),
        missing.len(),
        extra.len()
    );
    assert!(extra.is_empty(), "{what}: phantom {extra:?}");
    assert!(missing.len() <= 1, "{what}: lost {missing:?}");
    for (k, &(f, dt)) in b {
        if let Some(&(f2, dt2)) = a.get(k) {
            assert!((f - f2).abs() < 2.0, "{what}: freq {f} vs {f2}");
            assert!((dt - dt2).abs() < 0.05, "{what}: dt {dt} vs {dt2}");
        }
    }
}

/// FT8 and FT4 recordings in one IQ stream, starting on a slot boundary.
fn wideband(s: &Scene) -> Vec<(f32, f32)> {
    let mut iq = synth_iq(&s.ft8, FS, CENTER, FT8_DIAL);
    add_into(&mut iq, &synth_iq(&s.ft4, FS, CENTER, FT4_DIAL));
    pad(iq)
}

#[test]
fn two_channels_one_stream() {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = wideband(&s);
    let (mut rx, rows) = receiver();
    let ft8 = rx.add_channel(FT8_DIAL, IqMode::Ft8).unwrap();
    let ft4 = rx.add_channel(FT4_DIAL, IqMode::Ft4).unwrap();
    rx.set_time_anchor(T0_NS);
    // Odd block sizes, so slot ends fall inside a push.
    for chunk in interleave(&iq).chunks(2 * 77_777) {
        rx.push_cf32(chunk);
    }
    assert_eq!(rx.samples_in(), iq.len() as u64);
    let rows = rows.lock().unwrap();
    same(&set_of(&rows, ft8), &s.ft8_ref, "FT8 channel", &known_ft8());
    same(&set_of(&rows, ft4), &s.ft4_ref, "FT4 channel", &[]);

    for r in rows.iter() {
        let dial = if r.channel == ft8 { FT8_DIAL } else { FT4_DIAL };
        assert!((r.abs_freq_hz - (dial + r.decoded.freq_hz as f64)).abs() < 1e-6);
        assert_eq!(r.slot_start_utc_ns, Some(T0_NS));
        assert_eq!(r.slot_start_sample, 0);
    }
    assert!(rows.iter().any(|r| r.mode == IqMode::Ft8));
    assert!(rows.iter().any(|r| r.mode == IqMode::Ft4));
}

#[test]
fn stream_that_opens_mid_slot_decodes_the_next_whole_one() {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    // 3 s of nothing, then the recording on the boundary.
    let lead = 3 * FS as usize;
    let mut iq = vec![(0.0f32, 0.0f32); lead];
    iq.extend(synth_iq(&s.ft8, FS, CENTER, FT8_DIAL));
    let iq = pad(iq);
    let (mut rx, rows) = receiver();
    let ch = rx.add_channel(FT8_DIAL, IqMode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS - 3_000_000_000);
    rx.push_cf32(&interleave(&iq));
    let rows = rows.lock().unwrap();
    same(
        &set_of(&rows, ch),
        &s.ft8_ref,
        "FT8 after a partial slot",
        &known_ft8(),
    );
    assert!(rows.iter().all(|r| r.slot_start_utc_ns == Some(T0_NS)));
    assert!(rows.iter().all(|r| r.slot_start_sample == lead as u64));
}

#[test]
fn free_running_grid_without_an_anchor() {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = pad(synth_iq(&s.ft8, FS, CENTER, FT8_DIAL));
    let (mut rx, rows) = receiver();
    let ch = rx.add_channel(FT8_DIAL, IqMode::Ft8).unwrap();
    rx.push_cf32(&interleave(&iq));
    let rows = rows.lock().unwrap();
    same(
        &set_of(&rows, ch),
        &s.ft8_ref,
        "FT8 free-running",
        &known_ft8(),
    );
    assert!(rows.iter().all(|r| r.slot_start_utc_ns.is_none()));
}

/// Two copies of the recording back to back: a retune or a gap in the first
/// must cost that slot and nothing else, and must not manufacture decodes.
fn two_slots(s: &Scene) -> Vec<(f32, f32)> {
    let one = synth_iq(&s.ft8, FS, CENTER, FT8_DIAL);
    let mut iq = one.clone();
    iq.extend(one);
    pad(iq)
}

#[test]
fn retune_mid_slot_drops_that_slot_only() {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = two_slots(&s);
    let (mut rx, rows) = receiver();
    let ch = rx.add_channel(FT8_DIAL, IqMode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    let cut = 7 * FS as usize;
    rx.push_cf32(&interleave(&iq[..cut]));
    rx.retune(CENTER).unwrap();
    rx.push_cf32(&interleave(&iq[cut..]));
    let rows = rows.lock().unwrap();
    assert!(
        rows.iter()
            .all(|r| r.slot_start_utc_ns == Some(T0_NS + 15_000_000_000))
    );
    same(
        &set_of(&rows, ch),
        &s.ft8_ref,
        "second slot after a retune in the first",
        &known_ft8(),
    );
}

#[test]
fn gap_mid_slot_drops_that_slot_only() {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = two_slots(&s);
    let (mut rx, rows) = receiver();
    let ch = rx.add_channel(FT8_DIAL, IqMode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    let (cut, lost) = (5 * FS as usize, 1_000usize);
    rx.push_cf32(&interleave(&iq[..cut]));
    rx.gap(lost as u64);
    rx.push_cf32(&interleave(&iq[cut + lost..]));
    assert_eq!(rx.samples_in(), iq.len() as u64);
    let rows = rows.lock().unwrap();
    assert!(
        rows.iter()
            .all(|r| r.slot_start_utc_ns == Some(T0_NS + 15_000_000_000))
    );
    same(
        &set_of(&rows, ch),
        &s.ft8_ref,
        "second slot after a gap in the first",
        &known_ft8(),
    );
}

#[test]
fn placement_is_refused_and_a_bad_retune_changes_nothing() {
    let mut rx = IqReceiver::new(stream());
    // DC inside the band.
    assert_eq!(
        rx.add_channel(CENTER - 1_000.0, IqMode::Ft8),
        Err(IqError::TooCloseToDc)
    );
    // Past the band edge.
    assert_eq!(
        rx.add_channel(CENTER + 95_000.0, IqMode::Ft8),
        Err(IqError::OutsideBand)
    );
    let ch = rx.add_channel(FT8_DIAL, IqMode::Ft8).unwrap();
    // A retune that puts the channel outside is refused whole.
    assert_eq!(rx.retune(CENTER + 200_000.0), Err(IqError::OutsideBand));
    assert!(rx.remove_channel(ch));
    assert!(!rx.remove_channel(ch));
    // Nothing to place: any retune is fine.
    assert_eq!(rx.retune(CENTER + 200_000.0), Ok(()));
}

#[test]
fn byte_stream_matches_typed_push() {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = pad(synth_iq(&s.ft8, FS, CENTER, FT8_DIAL));
    let f = interleave(&iq);
    let bytes: Vec<u8> = f.iter().flat_map(|v| v.to_le_bytes()).collect();
    let (mut rx, rows) = receiver();
    let ch = rx.add_channel(FT8_DIAL, IqMode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    // Split inside samples.
    for chunk in bytes.chunks(100_003) {
        rx.push_bytes(chunk);
    }
    assert_eq!(rx.samples_in(), iq.len() as u64);
    same(
        &set_of(&rows.lock().unwrap(), ch),
        &s.ft8_ref,
        "FT8 from bytes",
        &known_ft8(),
    );
}
