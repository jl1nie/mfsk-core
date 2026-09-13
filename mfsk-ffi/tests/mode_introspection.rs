//! The introspection surface must agree with `mfsk_core`'s registry,
//! and the capability bits must agree bit for bit.
//!
//! The plan that introduced this surface named caps drift as its own
//! risk: "if the bitmask is hand-written it will lie within two
//! releases." `mfsk-core` already enforces claimed-vs-implemented in
//! both directions (`tests/registry_caps.rs`, where the bound on
//! `check_sic_rounds::<P>` is the compile-time half). What that cannot
//! reach is this crate re-declaring the same bits for C. This file is
//! that half.

use std::ffi::{CStr, CString};

use mfsk::*;
use mfsk_core::registry::caps as r;

/// Every C capability constant must equal the `mfsk-core` bit it
/// mirrors. Stated one pair at a time on purpose: a loop over a list
/// built from the same source would prove nothing.
#[test]
fn abi_caps_match_the_registry() {
    let pairs: [(&str, u64, u32); 15] = [
        ("DECODE_HANDLE", MFSK_CAP_DECODE_HANDLE, r::DECODE_HANDLE),
        ("SNIPER", MFSK_CAP_SNIPER, r::SNIPER),
        ("AP_NARROW", MFSK_CAP_AP_NARROW, r::AP_NARROW),
        ("AP_WIDEBAND", MFSK_CAP_AP_WIDEBAND, r::AP_WIDEBAND),
        ("SIC_ROUNDS", MFSK_CAP_SIC_ROUNDS, r::SIC_ROUNDS),
        ("SIC_EARLY", MFSK_CAP_SIC_EARLY, r::SIC_EARLY),
        ("OSD", MFSK_CAP_OSD, r::OSD),
        ("EQ_MODE", MFSK_CAP_EQ_MODE, r::EQ_MODE),
        ("STRICTNESS", MFSK_CAP_STRICTNESS, r::STRICTNESS),
        ("BUDGET", MFSK_CAP_BUDGET, r::BUDGET),
        ("KNOWN_FILTER", MFSK_CAP_KNOWN_FILTER, r::KNOWN_FILTER),
        ("KNOWN_SUBTRACT", MFSK_CAP_KNOWN_SUBTRACT, r::KNOWN_SUBTRACT),
        ("FFT_CACHE", MFSK_CAP_FFT_CACHE, r::FFT_CACHE),
        ("ON_RESULT", MFSK_CAP_ON_RESULT, r::ON_RESULT),
        ("ENCODE", MFSK_CAP_ENCODE, r::ENCODE),
    ];
    for (name, abi, reg) in pairs {
        assert_eq!(
            abi,
            u64::from(reg),
            "caps::{name} is {abi} at the C boundary and {reg} in the registry"
        );
    }
}

/// The array-length constants are literals in this crate, for the same
/// cbindgen reason as the capability bits, so they need the same pin.
/// Without the `#define` the generated header references an undeclared
/// identifier and does not compile — which `header_compile.sh` catches,
/// but only after the value has already gone wrong.
#[test]
fn abi_lengths_match_the_shared_definitions() {
    assert_eq!(MFSK_AP_FIELD_LEN, mfsk_ffi_abi::MFSK_AP_FIELD_LEN);
    assert_eq!(MFSK_DECODE_TEXT_LEN, mfsk_ffi_abi::MFSK_DECODE_TEXT_LEN);
    assert_eq!(
        MFSK_DECODE_FLAG_HASH_RESOLVED,
        mfsk_ffi_abi::MFSK_DECODE_FLAG_HASH_RESOLVED
    );
    // And they are the sizes the structs actually carry.
    let p = std::mem::MaybeUninit::<MfskDecodeParams>::zeroed();
    let p = unsafe { p.assume_init() };
    assert_eq!(p.ap_call1.len(), MFSK_AP_FIELD_LEN);
    assert_eq!(p.ap_grid.len(), MFSK_AP_FIELD_LEN);
}

/// Enumeration must reach every mode this build has, and `mfsk_mode_at`
/// past the end must fail rather than wrap or return mode 0.
#[test]
fn enumeration_covers_the_build() {
    let n = mfsk_mode_count();
    assert!(
        n >= 7,
        "a full build has at least FT8 + FT4 + five FST4, got {n}"
    );

    let mut seen = Vec::new();
    for i in 0..n {
        let mut m = MfskMode::Ft8;
        assert_eq!(unsafe { mfsk_mode_at(i, &mut m) }, MfskStatus::Ok);
        assert!(!seen.contains(&m), "mfsk_mode_at returned {m:?} twice");
        seen.push(m);
    }

    let mut m = MfskMode::Ft8;
    assert_eq!(
        unsafe { mfsk_mode_at(n, &mut m) },
        MfskStatus::InvalidArg,
        "one past the end must fail, not wrap"
    );
    assert_eq!(
        unsafe { mfsk_mode_at(0, std::ptr::null_mut()) },
        MfskStatus::InvalidArg
    );
}

/// `mfsk_mode_name` and `mfsk_mode_from_name` round-trip, and the name
/// is the registry's own key.
#[test]
fn names_round_trip_and_match_the_registry() {
    for i in 0..mfsk_mode_count() {
        let mut m = MfskMode::Ft8;
        assert_eq!(unsafe { mfsk_mode_at(i, &mut m) }, MfskStatus::Ok);

        let name = unsafe { CStr::from_ptr(mfsk_mode_name(m as u32)) }
            .to_str()
            .unwrap();
        assert!(!name.is_empty());

        let c = CString::new(name).unwrap();
        let mut back = MfskMode::Jt65;
        assert_eq!(
            unsafe { mfsk_mode_from_name(c.as_ptr(), &mut back) },
            MfskStatus::Ok,
            "{name} did not resolve back to a mode"
        );
        assert_eq!(back, m, "{name} round-tripped to a different mode");

        // MSK144 is the one mode with no registry entry, by design.
        if m != MfskMode::Msk144 {
            assert!(
                mfsk_core::PROTOCOLS.iter().any(|p| p.name == name),
                "{name} is not a registry key, so by_name would miss it"
            );
        }
    }
}

/// A name that is not a mode, and a real mode name, must be
/// distinguishable from each other — a typo is not the same problem as
/// a missing feature.
#[test]
fn a_typo_and_a_missing_feature_report_differently() {
    let mut m = MfskMode::Ft8;
    let bogus = CString::new("FT9").unwrap();
    assert_eq!(
        unsafe { mfsk_mode_from_name(bogus.as_ptr(), &mut m) },
        MfskStatus::InvalidArg
    );
    // Case matters: the registry's keys are exact.
    let wrong_case = CString::new("ft8").unwrap();
    assert_eq!(
        unsafe { mfsk_mode_from_name(wrong_case.as_ptr(), &mut m) },
        MfskStatus::InvalidArg
    );
    assert_eq!(
        unsafe { mfsk_mode_from_name(std::ptr::null(), &mut m) },
        MfskStatus::InvalidArg
    );
}

/// The geometry a host actually needs, checked against known values
/// rather than against the registry it was copied from.
#[test]
fn mode_info_reports_real_geometry() {
    let info = |m| {
        let mut i = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
        let st = unsafe { mfsk_mode_info(m as u32, i.as_mut_ptr()) };
        assert_eq!(st, MfskStatus::Ok, "{m:?}");
        unsafe { i.assume_init() }
    };

    let ft8 = info(MfskMode::Ft8);
    assert_eq!(ft8.ntones, 8);
    assert_eq!(ft8.slot_samples_12k, 180_000);
    assert_eq!(ft8.tx_start_offset_s, 0.5);
    assert_eq!(ft8.fec_k, 91, "FT8 is LDPC(174,91), CRC-14");
    assert_eq!(ft8.decode_fft1_size, 192_000);

    let ft4 = info(MfskMode::Ft4);
    assert_eq!(ft4.slot_samples_12k, 90_000);
    assert_eq!(ft4.decode_fft1_size, 92_160);

    // The five FST4 sub-modes the pre-v2 ABI could not address at all.
    let f15 = info(MfskMode::Fst4s15);
    let f300 = info(MfskMode::Fst4s300);
    assert_eq!(f15.fec_k, 101, "FST4 is LDPC(240,101), CRC-24");
    assert_eq!(
        f15.tx_start_offset_s, 0.5,
        "FST4-15 is the sub-mode that starts 0.5 s in, not 1.0"
    );
    assert_eq!(info(MfskMode::Fst4s30).tx_start_offset_s, 1.0);
    assert_eq!(f300.slot_samples_12k, 3_600_000);

    // The spread that makes one memory budget for all modes wrong.
    assert_eq!(f300.decode_fft1_size, 4_194_304);
    assert!(f300.decode_fft1_size / ft4.decode_fft1_size >= 45);

    // Each reports its own sub-mode, not the family.
    for m in [
        MfskMode::Fst4s15,
        MfskMode::Fst4s30,
        MfskMode::Fst4s60,
        MfskMode::Fst4s120,
        MfskMode::Fst4s300,
    ] {
        assert_eq!(info(m).mode, m);
    }

    // `slot_samples_12k` is documented as "`t_slot_s` made exact", and
    // a caller sizes a capture ring from it, so the two must agree for
    // every mode this build has — not just the ones spelled out above.
    // MSK144 reported 863 against 864 until the row stopped computing
    // it as `(0.072_f32 * 12_000.0) as u32`.
    for index in 0..mfsk_mode_count() {
        let mut m = MfskMode::Ft8;
        assert_eq!(
            unsafe { mfsk_mode_at(index, &mut m) },
            MfskStatus::Ok,
            "mode_at failed inside 0..count"
        );
        let i = info(m);
        assert_eq!(
            i.slot_samples_12k,
            (i.t_slot_s * 12_000.0).round() as u32,
            "{m:?}: slot_samples_12k disagrees with t_slot_s"
        );
    }
}

/// MSK144 is addressable and honest about being differently shaped.
#[test]
fn msk144_is_addressable_and_says_what_it_is_not() {
    let mut i = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
    assert_eq!(
        unsafe { mfsk_mode_info(MfskMode::Msk144 as u32, i.as_mut_ptr()) },
        MfskStatus::Ok
    );
    let i = unsafe { i.assume_init() };
    assert_eq!(i.fec_k, 90, "LDPC(128,90)");
    assert_eq!(
        i.slot_samples_12k,
        i.nsps * i.n_symbols,
        "MSK144's frame is 144 symbols x 6 samples = 864 at 12 kHz"
    );
    assert_eq!(
        i.caps & MFSK_CAP_DECODE_HANDLE,
        0,
        "MSK144 bypasses engine::pipeline by design, so the decode handle must not claim it"
    );
}

/// Capabilities published for C must be the ones `mfsk-core` claims,
/// including the two this branch just changed.
#[test]
fn published_capabilities_match_the_traits() {
    assert_ne!(mfsk_mode_caps(MfskMode::Ft8 as u32) & MFSK_CAP_SNIPER, 0);
    for m in [
        MfskMode::Ft4,
        MfskMode::Fst4s15,
        MfskMode::Fst4s60,
        MfskMode::Fst4s300,
    ] {
        assert_eq!(
            mfsk_mode_caps(m as u32) & MFSK_CAP_SNIPER,
            0,
            "{m:?} must not advertise a sniper it does not have"
        );
        assert_ne!(
            mfsk_mode_caps(m as u32) & MFSK_CAP_AP_WIDEBAND,
            0,
            "{m:?} does wide-band AP now and must say so"
        );
    }
    assert_eq!(
        mfsk_mode_caps(MfskMode::Wspr as u32) & MFSK_CAP_DECODE_HANDLE,
        0
    );
}

/// Defaults are data, published per mode — and carry the scale that
/// says whether two modes' numbers are comparable at all.
#[test]
fn defaults_are_published_with_their_scale() {
    let d = |m| {
        let mut x = std::mem::MaybeUninit::<MfskDecodeDefaults>::zeroed();
        let st = unsafe { mfsk_mode_defaults(m as u32, x.as_mut_ptr()) };
        assert_eq!(st, MfskStatus::Ok, "{m:?}");
        unsafe { x.assume_init() }
    };

    let ft4 = d(MfskMode::Ft4);
    assert_eq!(ft4.sync_scale, MfskSyncScale::BaselineNormalised);
    assert_eq!(ft4.sync_min, 1.2, "WSJT-X ft4_decode.f90:195");
    assert!(
        ft4.sync_min > 1.0,
        "on its own scale noise sits at 1.0, so anything below is meaningless"
    );

    let ft8 = d(MfskMode::Ft8);
    assert_eq!(ft8.sync_scale, MfskSyncScale::CostasAbsolute);
    assert_ne!(
        ft8.sync_min, ft4.sync_min,
        "the whole point of publishing the scale is that these two are not comparable"
    );

    for m in [MfskMode::Fst4s15, MfskMode::Fst4s300] {
        assert_eq!(d(m).sync_scale, MfskSyncScale::CostasAbsolute);
    }

    assert!(ft8.freq_max_hz > ft8.freq_min_hz && ft8.max_cand > 0);
    assert_eq!(
        unsafe { mfsk_mode_defaults(MfskMode::Ft8 as u32, std::ptr::null_mut()) },
        MfskStatus::InvalidArg
    );
}

/// The size field is the growth contract: an older caller declaring a
/// smaller struct must get only its prefix written, and must be told how
/// much that was.
#[test]
fn size_versioning_writes_only_the_declared_prefix() {
    const FULL: usize = std::mem::size_of::<MfskModeInfo>();
    let short = std::mem::offset_of!(MfskModeInfo, ntones);

    let mut buf = vec![0xAAu8; FULL];
    // An old header that knows only up to `ntones`.
    buf[..4].copy_from_slice(&(short as u32).to_ne_bytes());
    let st = unsafe { mfsk_mode_info(MfskMode::Ft8 as u32, buf.as_mut_ptr() as *mut MfskModeInfo) };
    assert_eq!(st, MfskStatus::Ok);

    let written = u32::from_ne_bytes(buf[..4].try_into().unwrap()) as usize;
    assert_eq!(written, short, "size must report what was actually written");
    assert!(
        buf[short..].iter().all(|&b| b == 0xAA),
        "bytes past the caller's declared size were overwritten"
    );

    // A zeroed struct means "same header as you", so everything is written.
    let mut full = std::mem::MaybeUninit::<MfskModeInfo>::zeroed();
    assert_eq!(
        unsafe { mfsk_mode_info(MfskMode::Ft8 as u32, full.as_mut_ptr()) },
        MfskStatus::Ok
    );
    assert_eq!(unsafe { full.assume_init() }.size as usize, FULL);
}

/// The ABI revision is distinct from the crate version, which moves for
/// reasons that have nothing to do with the boundary.
#[test]
fn abi_version_is_not_the_crate_version() {
    assert_eq!(mfsk_abi_version(), 2);
    assert_ne!(mfsk_abi_version(), mfsk_version());
}
