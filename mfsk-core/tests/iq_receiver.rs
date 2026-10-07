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

use mfsk_core::Mode;
use mfsk_core::decoder::AnyDecoder;
use mfsk_core::iq::{
    ChannelId, ChannelState, Channelizer, CompletedSlot, IqError, IqReceiver, IqSampleFormat,
    IqStream,
};
use mfsk_core::msg::Decoded;

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

/// The WAV path: the mode's default decoder over the recording.
fn reference(mode: Mode, audio: &[i16]) -> Set {
    AnyDecoder::with_defaults(mode)
        .decode_i16(audio, None)
        .rows
        .into_iter()
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
    let ft8_ref = reference(Mode::Ft8, &ft8);
    let ft4_ref = reference(Mode::Ft4, &ft4);
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
    IqStream::new(FS, CENTER, IqSampleFormat::Cf32)
}

/// One decode with where it came from, as the 0.12 receiver delivered it.
#[derive(Clone)]
struct IqDecode {
    channel: ChannelId,
    mode: Mode,
    decoded: Decoded,
    slot_start_sample: u64,
    slot_start_utc_ns: Option<i64>,
}

/// A receiver, the slots it has returned, and the decoding the caller does
/// with them: one decoder per channel, in the order the slots arrived.
struct Rx {
    inner: IqReceiver,
    slots: Vec<CompletedSlot>,
}

impl Rx {
    fn add_channel(&mut self, dial: f64, mode: Mode) -> Result<ChannelId, IqError> {
        self.inner.add_channel(dial, mode)
    }
    fn set_time_anchor(&mut self, utc_ns_at_sample_0: i64) {
        self.inner.set_time(utc_ns_at_sample_0, 0);
    }
    fn push_cf32(&mut self, iq: &[f32]) {
        self.inner.push_cf32(iq, &mut self.slots);
    }
    fn push_bytes(&mut self, b: &[u8]) {
        self.inner.push_bytes(b, &mut self.slots);
    }
    fn samples_in(&self) -> u64 {
        self.inner.samples_in()
    }
    fn gap(&mut self, lost: u64) {
        self.inner.gap(lost);
    }
    fn rows(&self) -> Vec<IqDecode> {
        let mut decoders: std::collections::HashMap<usize, AnyDecoder> = Default::default();
        let mut rows = Vec::new();
        for slot in &self.slots {
            let d = decoders
                .entry(slot.channel.0)
                .or_insert_with(|| AnyDecoder::with_defaults(slot.mode));
            for decoded in d.decode(&slot.input()).rows {
                rows.push(IqDecode {
                    channel: slot.channel,
                    mode: slot.mode,
                    decoded,
                    slot_start_sample: slot.start_sample,
                    slot_start_utc_ns: slot.utc_ns,
                });
            }
        }
        rows
    }
}

fn receiver(kind: Channelizer) -> Rx {
    Rx {
        inner: IqReceiver::with_channelizer(stream(), kind).unwrap(),
        slots: Vec::new(),
    }
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
    assert!(missing.len() <= b.len() / 10, "{what}: lost {missing:?}");
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

fn two_channels_one_stream(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = wideband(&s);
    let mut rx = receiver(kind);
    let ft8 = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    let ft4 = rx.add_channel(FT4_DIAL, Mode::Ft4).unwrap();
    rx.set_time_anchor(T0_NS);
    // Odd block sizes, so slot ends fall inside a push.
    for chunk in interleave(&iq).chunks(2 * 77_777) {
        rx.push_cf32(chunk);
    }
    assert_eq!(rx.samples_in(), iq.len() as u64);
    let rows = rx.rows();
    same(&set_of(&rows, ft8), &s.ft8_ref, "FT8 channel", &known_ft8());
    same(&set_of(&rows, ft4), &s.ft4_ref, "FT4 channel", &[]);

    for r in rows.iter() {
        let dial = if r.channel == ft8 { FT8_DIAL } else { FT4_DIAL };
        // The RF frequency of a decode is its channel's dial plus the audio
        // frequency: `CompletedSlot::abs_freq_hz`.
        assert!(dial > 0.0 && r.decoded.freq_hz > 0.0);
        assert_eq!(r.slot_start_utc_ns, Some(T0_NS));
        assert_eq!(r.slot_start_sample, 0);
    }
    assert!(rows.iter().any(|r| r.mode == Mode::Ft8));
    assert!(rows.iter().any(|r| r.mode == Mode::Ft4));
}

fn stream_that_opens_mid_slot_decodes_the_next_whole_one(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    // 3 s of nothing, then the recording on the boundary.
    let lead = 3 * FS as usize;
    let mut iq = vec![(0.0f32, 0.0f32); lead];
    iq.extend(synth_iq(&s.ft8, FS, CENTER, FT8_DIAL));
    let iq = pad(iq);
    let mut rx = receiver(kind);
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS - 3_000_000_000);
    rx.push_cf32(&interleave(&iq));
    let rows = rx.rows();
    same(
        &set_of(&rows, ch),
        &s.ft8_ref,
        "FT8 after a partial slot",
        &known_ft8(),
    );
    assert!(rows.iter().all(|r| r.slot_start_utc_ns == Some(T0_NS)));
    assert!(rows.iter().all(|r| r.slot_start_sample == lead as u64));
}

fn free_running_grid_without_an_anchor(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = pad(synth_iq(&s.ft8, FS, CENTER, FT8_DIAL));
    let mut rx = receiver(kind);
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    rx.push_cf32(&interleave(&iq));
    let rows = rx.rows();
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

fn retune_mid_slot_drops_that_slot_only(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = two_slots(&s);
    let mut rx = receiver(kind);
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    let cut = 7 * FS as usize;
    rx.push_cf32(&interleave(&iq[..cut]));
    rx.inner.retune(CENTER);
    rx.push_cf32(&interleave(&iq[cut..]));
    let rows = rx.rows();
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

fn gap_mid_slot_drops_that_slot_only(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = two_slots(&s);
    let mut rx = receiver(kind);
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    let (cut, lost) = (5 * FS as usize, 1_000usize);
    rx.push_cf32(&interleave(&iq[..cut]));
    rx.gap(lost as u64);
    rx.push_cf32(&interleave(&iq[cut + lost..]));
    assert_eq!(rx.samples_in(), iq.len() as u64);
    let rows = rx.rows();
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

/// A real timestamp is almost never on the 12 kHz grid. Anchored 40 µs off
/// it, two back-to-back slots must both decode: slot starts round up to a
/// sample, and the second slot used to be skipped for the one after it
/// (`next_boundary`). Every other test anchors on `T0_NS`, which is on the
/// grid, and so never saw it.
fn off_grid_anchor_decodes_back_to_back_slots(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = two_slots(&s);
    let mut rx = receiver(kind);
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS + 40_000);
    rx.push_cf32(&interleave(&iq));
    let rows = rx.rows();
    for (n, start) in [T0_NS, T0_NS + 15_000_000_000].into_iter().enumerate() {
        let slot: Vec<IqDecode> = rows
            .iter()
            .filter(|r| r.slot_start_utc_ns == Some(start))
            .cloned()
            .collect();
        same(
            &set_of(&slot, ch),
            &s.ft8_ref,
            &format!("slot {n} on an off-grid anchor"),
            &known_ft8(),
        );
    }
}

fn placement_is_refused_and_a_bad_retune_changes_nothing(kind: Channelizer) {
    let mut rx = IqReceiver::with_channelizer(stream(), kind).unwrap();
    // DC inside the band.
    assert_eq!(
        rx.add_channel(CENTER - 1_000.0, Mode::Ft8),
        Err(IqError::TooCloseToDc)
    );
    // Past the band edge.
    assert_eq!(
        rx.add_channel(CENTER + 95_000.0, Mode::Ft8),
        Err(IqError::OutsideBand)
    );
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    // A retune that puts the channel outside pauses it and keeps it.
    let report = rx.retune(CENTER + 200_000.0);
    assert_eq!(report.paused, vec![ch]);
    assert_eq!(
        rx.channel_state(ch),
        Some(ChannelState::Paused(IqError::OutsideBand))
    );
    // Back inside the band it resumes, with the same id.
    let report = rx.retune(CENTER);
    assert_eq!(report.resumed, vec![ch]);
    assert_eq!(rx.channel_state(ch), Some(ChannelState::Active));
    assert!(rx.remove_channel(ch));
    assert!(!rx.remove_channel(ch));
    // Nothing to place: any retune is fine.
    assert_eq!(rx.retune(CENTER + 200_000.0), Default::default());
}

fn byte_stream_matches_typed_push(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = pad(synth_iq(&s.ft8, FS, CENTER, FT8_DIAL));
    let f = interleave(&iq);
    let bytes: Vec<u8> = f.iter().flat_map(|v| v.to_le_bytes()).collect();
    let mut rx = receiver(kind);
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    // Split inside samples.
    for chunk in bytes.chunks(100_003) {
        rx.push_bytes(chunk);
    }
    assert_eq!(rx.samples_in(), iq.len() as u64);
    same(
        &set_of(&rx.rows(), ch),
        &s.ft8_ref,
        "FT8 from bytes",
        &known_ft8(),
    );
}

/// The 8-bit and 24-bit byte formats through the receiver: the recording
/// quantised the way an RTL-SDR (`Cu8`), a HackRF (`Cs8`) or a 24-bit IQ WAV
/// (`Cs24`) delivers it, at 48 kS/s so the wanted signal is a fifth of the
/// band. 8 bits leave ~48 dB below full scale, spread over the band and
/// filtered down to 3 kHz; the recording's own noise floor sits well above.
fn byte_formats_match_the_wav_path(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let fs = 48_000u32;
    let dial = CENTER + 6_000.0;
    let iq = pad_at(synth_iq(&s.ft8, fs, CENTER, dial), fs);
    // Scale the peak to 0.7 of full scale: a receiver's gain does that.
    let peak = iq
        .iter()
        .fold(0.0f32, |m, &(i, q)| m.max(i.abs()).max(q.abs()));
    let g = 0.7 / peak;
    for fmt in [
        IqSampleFormat::Cu8,
        IqSampleFormat::Cs8,
        IqSampleFormat::Cs24,
    ] {
        let mut bytes = Vec::new();
        for &(i, q) in &iq {
            for v in [i * g, q * g] {
                match fmt {
                    IqSampleFormat::Cu8 => {
                        bytes.push((v * 128.0 + 128.0).round().clamp(0.0, 255.0) as u8)
                    }
                    IqSampleFormat::Cs8 => {
                        bytes.push(((v * 128.0).round().clamp(-128.0, 127.0) as i8) as u8)
                    }
                    _ => bytes.extend(&((v * 8_388_608.0).round() as i32).to_le_bytes()[..3]),
                }
            }
        }
        let mut rx = Rx {
            inner: IqReceiver::with_channelizer(IqStream::new(fs, CENTER, fmt), kind).unwrap(),
            slots: Vec::new(),
        };
        let ch = rx.add_channel(dial, Mode::Ft8).unwrap();
        rx.set_time_anchor(T0_NS);
        for chunk in bytes.chunks(100_003) {
            rx.push_bytes(chunk);
        }
        same(
            &set_of(&rx.rows(), ch),
            &s.ft8_ref,
            &format!("FT8 from {fmt:?}"),
            &known_ft8(),
        );
    }
}

fn pad_at(mut iq: Vec<(f32, f32)>, fs: u32) -> Vec<(f32, f32)> {
    iq.resize(iq.len() + fs as usize / 2, (0.0, 0.0));
    iq
}

/// A retune that leaves one of two channels outside the band pauses that one
/// and nothing else: the channel that stays keeps producing slots, with the
/// next index, and the paused one resumes with the same id when the band
/// comes back.
fn partial_retune_pauses_only_what_no_longer_fits(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    // Two copies of the FT8 recording: slot 0 before the retune, slot 1 after.
    let iq = two_slots(&s);
    let mut rx = receiver(kind);
    let near = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    // 90 kHz up: inside a 192 kS/s band now, outside once the centre moves.
    let far = rx.add_channel(CENTER + 70_000.0, Mode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    let cut = 20 * FS as usize;
    rx.push_cf32(&interleave(&iq[..cut]));
    // The centre moves 60 kHz down: `near` (FT8_DIAL = centre + 20 kHz) is now
    // 80 kHz up and still fits; `far` is 130 kHz up and does not.
    let report = rx.inner.retune(CENTER - 60_000.0);
    assert_eq!(report.paused, vec![far]);
    assert!(report.resumed.is_empty());
    assert_eq!(rx.inner.channel_state(near), Some(ChannelState::Active));
    assert!(matches!(
        rx.inner.channel_state(far),
        Some(ChannelState::Paused(_))
    ));
    // The recording is still at the old centre, so `near` hears nothing it
    // can decode; the point is that it keeps cutting slots on the grid.
    rx.push_cf32(&interleave(&iq[cut..]));
    let periods: Vec<i64> = rx
        .slots
        .iter()
        .filter(|s| s.channel == near)
        .map(|s| s.period)
        .collect();
    assert!(
        periods.windows(2).all(|w| w[1] == w[0] + 1),
        "periods of the channel that stayed: {periods:?}"
    );
    // The paused channel cut nothing after the retune: at most slot 0 of the
    // grid (UTC `T0_NS`), which had completed before it.
    let first = T0_NS / 15_000_000_000;
    assert!(
        rx.slots
            .iter()
            .all(|s| s.channel != far || s.period == first)
    );
    // Back to the original centre: `far` resumes with its own id.
    let report = rx.inner.retune(CENTER);
    assert_eq!(report.resumed, vec![far]);
    assert_eq!(rx.inner.channel_state(far), Some(ChannelState::Active));
}

/// The clock moves between two slots by a few milliseconds, as a drifting
/// host clock does: both slots come back, consecutive, each decoding to the
/// recording's messages.
fn slewing_between_slots_loses_none(kind: Channelizer) {
    let Some(s) = scene() else {
        common::skip_or_fail("FT8/FT4 recordings");
        return;
    };
    let iq = two_slots(&s);
    let mut rx = receiver(kind);
    let ch = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    rx.set_time_anchor(T0_NS);
    // Half a second of samples at a time; every reading is 2 ms further off
    // than the last, so the clock slews at its limit throughout.
    let half = (FS as usize) / 2;
    let mut at = 0u64;
    for (n, chunk) in iq.chunks(half).enumerate() {
        rx.push_cf32(&interleave(chunk));
        at += chunk.len() as u64;
        let utc = T0_NS + (at as i128 * 1_000_000_000 / FS as i128) as i64 + 2_000_000 * n as i64;
        rx.inner.set_time(utc, at);
    }
    let periods: Vec<i64> = rx
        .slots
        .iter()
        .filter(|s| s.channel == ch)
        .map(|s| s.period)
        .collect();
    assert_eq!(periods.len(), 2, "{periods:?}");
    assert_eq!(periods[1], periods[0] + 1);
    let rows = rx.rows();
    for p in &periods {
        let slot: Vec<IqDecode> = rows
            .iter()
            .filter(|r| r.slot_start_utc_ns == Some(*p * 15_000_000_000))
            .cloned()
            .collect();
        same(
            &set_of(&slot, ch),
            &s.ft8_ref,
            "a slot while slewing",
            &known_ft8(),
        );
    }
}

#[test]
fn partial_retune_direct() {
    partial_retune_pauses_only_what_no_longer_fits(Channelizer::Direct);
}
#[test]
fn partial_retune_pfb() {
    partial_retune_pauses_only_what_no_longer_fits(Channelizer::Pfb);
}
#[test]
fn slewing_direct() {
    slewing_between_slots_loses_none(Channelizer::Direct);
}
#[test]
fn slewing_pfb() {
    slewing_between_slots_loses_none(Channelizer::Pfb);
}

// Every scene above through both paths: same recordings, same expectations.
#[test]
fn two_channels_one_stream_direct() {
    two_channels_one_stream(Channelizer::Direct);
}
#[test]
fn two_channels_one_stream_pfb() {
    two_channels_one_stream(Channelizer::Pfb);
}
#[test]
fn stream_that_opens_mid_slot_decodes_the_next_whole_one_direct() {
    stream_that_opens_mid_slot_decodes_the_next_whole_one(Channelizer::Direct);
}
#[test]
fn stream_that_opens_mid_slot_decodes_the_next_whole_one_pfb() {
    stream_that_opens_mid_slot_decodes_the_next_whole_one(Channelizer::Pfb);
}
#[test]
fn free_running_grid_without_an_anchor_direct() {
    free_running_grid_without_an_anchor(Channelizer::Direct);
}
#[test]
fn free_running_grid_without_an_anchor_pfb() {
    free_running_grid_without_an_anchor(Channelizer::Pfb);
}
#[test]
fn retune_mid_slot_drops_that_slot_only_direct() {
    retune_mid_slot_drops_that_slot_only(Channelizer::Direct);
}
#[test]
fn retune_mid_slot_drops_that_slot_only_pfb() {
    retune_mid_slot_drops_that_slot_only(Channelizer::Pfb);
}
#[test]
fn gap_mid_slot_drops_that_slot_only_direct() {
    gap_mid_slot_drops_that_slot_only(Channelizer::Direct);
}
#[test]
fn gap_mid_slot_drops_that_slot_only_pfb() {
    gap_mid_slot_drops_that_slot_only(Channelizer::Pfb);
}
#[test]
fn off_grid_anchor_decodes_back_to_back_slots_direct() {
    off_grid_anchor_decodes_back_to_back_slots(Channelizer::Direct);
}
#[test]
fn off_grid_anchor_decodes_back_to_back_slots_pfb() {
    off_grid_anchor_decodes_back_to_back_slots(Channelizer::Pfb);
}
#[test]
fn placement_is_refused_and_a_bad_retune_changes_nothing_direct() {
    placement_is_refused_and_a_bad_retune_changes_nothing(Channelizer::Direct);
}
#[test]
fn placement_is_refused_and_a_bad_retune_changes_nothing_pfb() {
    placement_is_refused_and_a_bad_retune_changes_nothing(Channelizer::Pfb);
}
#[test]
fn byte_stream_matches_typed_push_direct() {
    byte_stream_matches_typed_push(Channelizer::Direct);
}
#[test]
fn byte_stream_matches_typed_push_pfb() {
    byte_stream_matches_typed_push(Channelizer::Pfb);
}
#[test]
fn byte_formats_match_the_wav_path_direct() {
    byte_formats_match_the_wav_path(Channelizer::Direct);
}
#[test]
fn byte_formats_match_the_wav_path_pfb() {
    byte_formats_match_the_wav_path(Channelizer::Pfb);
}

/// `tap_audio` / `take_audio`: the channel's continuous 12 kHz audio, for a
/// waterfall. A 1000 Hz tone 1 kHz above the dial comes out at 1000 Hz, at the
/// 12 kHz rate, and a drained tap starts again from empty.
fn tapped_audio_is_the_channel_audio(kind: Channelizer) {
    let tone: Vec<i16> = (0..12_000 * 6)
        .map(|n| (8_000.0 * (std::f64::consts::TAU * 1_000.0 * n as f64 / 12_000.0).sin()) as i16)
        .collect();
    let iq = synth_iq(&tone, FS, CENTER, FT8_DIAL);
    let stream = IqStream::new(FS, CENTER, IqSampleFormat::Cf32);
    let mut rx = IqReceiver::with_channelizer(stream, kind).unwrap();
    let id = rx.add_channel(FT8_DIAL, Mode::Ft8).unwrap();
    assert!(!rx.take_audio(id, &mut Vec::new()), "not tapped yet");
    assert!(rx.tap_audio(id, true));
    assert!(rx.tap_audio(id, false) && rx.tap_audio(id, true));
    let mut slots = Vec::new();
    let mut got = Vec::new();
    for chunk in interleave(&iq).chunks(2 * 4096) {
        rx.push_cf32(chunk, &mut slots);
        assert!(rx.take_audio(id, &mut got));
    }
    // 6 s of audio, less the filter's startup.
    assert!(
        (got.len() as i64 - 72_000).abs() < 1_500,
        "{} samples",
        got.len()
    );
    // Zero-crossing rate of the second half: 1000 Hz.
    let half = &got[got.len() / 2..];
    let crossings = half
        .windows(2)
        .filter(|w| w[0] <= 0.0 && w[1] > 0.0)
        .count();
    let hz = crossings as f64 * 12_000.0 / half.len() as f64;
    assert!((hz - 1_000.0).abs() < 10.0, "{hz} Hz");
    let mut again = Vec::new();
    assert!(rx.take_audio(id, &mut again) && again.is_empty());
    assert!(!rx.tap_audio(ChannelId(99), true));
}

#[test]
fn tapped_audio_direct() {
    tapped_audio_is_the_channel_audio(Channelizer::Direct);
}
#[test]
fn tapped_audio_pfb() {
    tapped_audio_is_the_channel_audio(Channelizer::Pfb);
}

/// A consumer can build a slot for its own tests (#592), and the decoder
/// takes it as it takes one the receiver cut.
#[test]
fn a_slot_built_outside_the_receiver_decodes_like_one_it_cut() {
    let mut slot = CompletedSlot::new(ChannelId(3), Mode::Ft8, 42, vec![0.0; 180_000]);
    slot.dial_hz = 14_074_000.0;
    assert_eq!(slot.input().period, Some(42));
    assert_eq!(slot.abs_freq_hz(1_500.0), 14_075_500.0);
    let out = AnyDecoder::with_defaults(slot.mode).decode(&slot.input());
    assert!(out.rows.is_empty());
}
