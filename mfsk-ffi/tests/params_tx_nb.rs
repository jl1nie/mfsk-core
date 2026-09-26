//! `tx_freq_hz` (FT8's `nftx`) and the FST4 noise blanker on `MfskDecodeParams` (#466).
//!
//! Both are fields added after `search_hz`, so the size-versioning contract applies: an
//! older, shorter struct leaves them at their defaults. Both are refused on a mode that
//! does not have them, as every other option here is, rather than dropped.

mod common;

use common::*;
use mfsk::*;

struct Lcg(u64);
impl Lcg {
    fn uniform(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
    fn gauss(&mut self) -> f64 {
        let (a, b) = (self.uniform().max(1e-12), self.uniform());
        (-2.0 * a.ln()).sqrt() * (2.0 * std::f64::consts::PI * b).cos()
    }
}

/// `slot` plus white Gaussian noise at `snr_db` in a 2 500 Hz bandwidth (12 kHz sampling).
fn noisy(slot: &[i16], snr_db: f64, seed: u64) -> Vec<i16> {
    let peak = slot.iter().map(|&x| f64::from(x).abs()).fold(0.0, f64::max);
    let noise_power = peak * peak / 2.0 / 10f64.powf(snr_db / 10.0) * 6000.0 / 2500.0;
    let sd = noise_power.sqrt();
    let mut r = Lcg(seed);
    slot.iter()
        .map(|&x| {
            (f64::from(x) + sd * r.gauss())
                .round()
                .clamp(-32768.0, 32767.0) as i16
        })
        .collect()
}

fn refused(mode: MfskMode, what: &str, mutate: impl FnOnce(&mut MfskDecodeParams)) {
    let mut p = params(mode);
    mutate(&mut p);
    let mut st = MfskStatus::Ok;
    let d = unsafe { mfsk_session_open(mode as u32, &p, &mut st) };
    assert!(d.is_null(), "{what} should not open a handle");
    assert_eq!(st, MfskStatus::Unsupported, "{what}");
    let why = unsafe { std::ffi::CStr::from_ptr(mfsk_last_error()) }
        .to_str()
        .unwrap();
    assert!(!why.is_empty(), "{what}: the reason must be recorded");
}

#[test]
fn defaults_and_capabilities() {
    let p = params(MfskMode::Ft8);
    assert!(p.tx_freq_hz.is_nan(), "unset is NaN, like freq_hint_hz");
    assert_eq!((p.nb_percent, p.nb_sweep_step), (0, 0));
    assert_eq!(p.nb_ftol_hz, 20.0, "WSJT-X's F Tol default");

    let has = |m: MfskMode, bit| mfsk_mode_caps(m as u32) & bit != 0;
    assert!(has(MfskMode::Ft8, MFSK_CAP_TX_FREQ));
    assert!(
        !has(MfskMode::Ft4, MFSK_CAP_TX_FREQ),
        "ft4_decode.f90 has no nftx"
    );
    for m in [MfskMode::Fst4s15, MfskMode::Fst4s60, MfskMode::Fst4s300] {
        assert!(has(m, MFSK_CAP_NOISE_BLANKER), "{m:?}");
        assert!(!has(m, MFSK_CAP_TX_FREQ), "{m:?}");
    }
    for m in [MfskMode::Ft8, MfskMode::Ft4] {
        assert!(!has(m, MFSK_CAP_NOISE_BLANKER), "{m:?}");
    }
}

/// The point of `nftx`: an a-priori hypothesis that locks both callsigns is tried only
/// within 50 Hz of the QSO frequency or the transmit frequency. A station answering from
/// 1500 Hz while the receive frequency sits at 800 Hz is found only through the second.
///
/// At −21 dB on white noise (12 files, fixed seeds) the hint alone finds 3 of them and the
/// transmit frequency brings it to 10 — the count a hint *at* 1500 Hz gets.
#[test]
fn the_transmit_frequency_opens_the_ap_window() {
    let slot = synth_slot_i16(MfskMode::Ft8, "K1ABC", "JA1XYZ", "-15", 1500.0);
    let count = |hint: f32, tx: Option<f32>| {
        (1..=12u64)
            .filter(|&seed| {
                let audio = noisy(&slot, -21.0, seed);
                let mut p = params(MfskMode::Ft8);
                p.freq_hint_hz = hint;
                with_ap(&mut p, "K1ABC", "JA1XYZ", "");
                if let Some(t) = tx {
                    p.tx_freq_hz = t;
                }
                let d = open(MfskMode::Ft8, Some(&p));
                let rows = decode_i16(d, &audio);
                unsafe { mfsk_session_close(d) };
                any_contains(&rows, "K1ABC JA1XYZ")
            })
            .count()
    };
    let far = count(800.0, None);
    let far_with_tx = count(800.0, Some(1500.0));
    let near = count(1500.0, None);
    assert!(far <= 5, "hint 800 Hz alone: {far}");
    assert!(
        far_with_tx >= far + 4,
        "tx_freq 1500 Hz should open the window: {far} -> {far_with_tx}"
    );
    assert!(
        far_with_tx.abs_diff(near) <= 1,
        "{far_with_tx} vs {near} at a hint on the signal"
    );
}

#[test]
fn what_a_mode_does_not_have_is_refused() {
    refused(MfskMode::Ft4, "tx_freq_hz on FT4", |p| {
        p.tx_freq_hz = 1500.0
    });
    refused(MfskMode::Fst4s60, "tx_freq_hz on FST4", |p| {
        p.tx_freq_hz = 1500.0
    });
    refused(MfskMode::Ft8, "tx_freq_hz on the narrow-band search", |p| {
        p.tx_freq_hz = 1500.0;
        p.freq_hint_hz = 1500.0;
        p.search_hz = 250.0;
    });
    refused(MfskMode::Ft8, "a noise blanker on FT8", |p| {
        p.nb_percent = 5
    });
    refused(MfskMode::Ft4, "a noise-blanker sweep on FT4", |p| {
        p.nb_sweep_step = 5;
        p.freq_hint_hz = 1500.0;
    });
    refused(MfskMode::Fst4s60, "nb_percent above the GUI's 25", |p| {
        p.nb_percent = 26
    });
    refused(MfskMode::Fst4s60, "a sweep step of 3", |p| {
        p.nb_sweep_step = 3;
        p.freq_hint_hz = 1500.0;
    });
    refused(MfskMode::Fst4s60, "a sweep without freq_hint_hz", |p| {
        p.nb_sweep_step = 5
    });
    refused(MfskMode::Fst4s60, "a sweep with no width", |p| {
        p.nb_sweep_step = 5;
        p.freq_hint_hz = 1500.0;
        p.nb_ftol_hz = 0.0;
    });
}

#[test]
fn the_fst4_blanker_reaches_the_decoder() {
    let slot = synth_slot_i16(MfskMode::Fst4s60, "K1ABC", "JA1XYZ", "-15", 1500.0);
    let audio = noisy(&slot, -25.0, 3);
    let decode = |nb_percent: u8, sweep: u8| {
        let mut p = params(MfskMode::Fst4s60);
        p.freq_hint_hz = 1500.0;
        p.nb_percent = nb_percent;
        p.nb_sweep_step = sweep;
        let d = open(MfskMode::Fst4s60, Some(&p));
        let rows = decode_i16(d, &audio);
        unsafe { mfsk_session_close(d) };
        rows
    };
    let plain = decode(0, 0);
    let blanked = decode(5, 0);
    let swept = decode(0, 5);
    for rows in [&plain, &blanked, &swept] {
        assert!(any_contains(rows, "K1ABC JA1XYZ"), "{:?}", texts(rows));
    }
    // A blanked pass sees different samples: the row is the same message, scored differently.
    let sync = |rows: &[MfskDecode]| {
        rows.iter()
            .find(|r| text_of(r).contains("K1ABC"))
            .unwrap()
            .sync_score
    };
    assert_ne!(
        sync(&plain),
        sync(&blanked),
        "nb_percent 5 changed nothing — is it wired?"
    );
}

/// The fields were added after `search_hz`: a caller built against the older header passes a
/// shorter struct, and what it never wrote must stay at the defaults — not be read from
/// whatever is past the end of its struct.
#[test]
fn an_older_shorter_struct_leaves_the_new_fields_at_their_defaults() {
    use std::mem::offset_of;
    let mut p = params(MfskMode::Ft4);
    // garbage where the new fields would be, which FT4 would refuse if it were read
    p.tx_freq_hz = 1500.0;
    p.nb_percent = 5;
    p.size = (offset_of!(MfskDecodeParams, search_hz) + 4) as u32;
    let mut st = MfskStatus::Ok;
    let d = unsafe { mfsk_session_open(MfskMode::Ft4 as u32, &p, &mut st) };
    assert_eq!(st, MfskStatus::Ok, "the tail past `size` must not be read");
    unsafe { mfsk_session_close(d) };
}
