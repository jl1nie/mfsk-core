// SPDX-License-Identifier: GPL-3.0-or-later
//! Regression test for issue #257 — the FT4 sniper "blind annulus".
//!
//! `SniperRequest::<Ft4>` asks `coarse_sync` for a ±250 Hz search
//! window, but used to decode a signal only within ~±14 Hz of
//! `target_freq` or beyond ~±100 Hz, returning nothing at all in
//! between at any SNR.
//!
//! Root cause was `coarse_sync`'s `freq_hint` ranking, not its scoring:
//! the hint applied *strict lexicographic* precedence ("within 10 Hz of
//! the aim point" first, score only as a tie-break). Sniper runs at
//! `sync_min = 0.8` with `max_cand` clamped to 15, and `coarse_sync`'s
//! per-bin NMS emits up to 8 lag peaks per frequency bin — so on FT4
//! (`df` = 5.21 Hz, three bins inside ±10 Hz) the aim point alone
//! produced more than 15 candidates, nearly all noise-floor lags
//! scoring ~1.0. They took every slot, and the real signal 16-99 Hz
//! away — scoring 12-17, by far the strongest candidate in the band —
//! was truncated off the list before it was ever handed to a decoder.
//!
//! Below ~14 Hz the annulus closed because the promoted aim-point
//! candidates are within `refine_candidate_position`'s own ±12 Hz
//! frequency pull-in; beyond ~100 Hz (just past FT4's 83.3 Hz occupied
//! bandwidth) it closed because the aim-adjacent bins no longer catch
//! any of the signal's energy, so they fall under `sync_min` and stop
//! crowding the list on their own.
//!
//! `engine::sync::rank_candidates` now reserves the aim point at most
//! *half* the candidate budget and fills the rest by score.

use mfsk_core::MessageCodec;
use mfsk_core::MessageFields;
use mfsk_core::ft4::Ft4;
use mfsk_core::ft4::encode::{message_to_tones, tones_to_f32};
use mfsk_core::msg::Wsjt77Message;
use mfsk_core::msg::decode_request::{DecodeRequest, SniperRequest};

const FS: f32 = 12_000.0;
const SLOT: usize = 90_000;
const TARGET_HZ: f32 = 1000.0;
/// -4 dB in WSJT-X's 2500 Hz reference bandwidth — comfortably above
/// FT4's threshold, so a miss is a ranking failure and never marginal
/// sensitivity.
const SNR_DB: f32 = -4.0;
const SEEDS: u64 = 8;

fn packed_cq() -> [u8; 77] {
    let bits = Wsjt77Message
        .pack(&MessageFields {
            call1: Some("CQ".into()),
            call2: Some("JA1ABC".into()),
            grid: Some("PM95".into()),
            ..Default::default()
        })
        .expect("pack CQ JA1ABC PM95");
    let mut msg = [0u8; 77];
    msg.copy_from_slice(&bits);
    msg
}

/// One AWGN slot carrying `msg` at `TARGET_HZ + offset_hz`, 0.5 s in.
///
/// Deliberately self-contained (LCG + Box-Muller) rather than reusing
/// `tests/common/channel.rs`: this is the exact generator the issue
/// #257 report was measured with, so keeping it bit-for-bit lets the
/// numbers here be compared against the ones in that thread.
fn slot_with_offset(msg: &[u8; 77], offset_hz: f32, seed: u64) -> Vec<i16> {
    let amp = (4.0 * 10f32.powf(SNR_DB / 10.0) * 2500.0 / FS).sqrt();
    let pcm = tones_to_f32(&message_to_tones(msg), TARGET_HZ + offset_hz, amp);

    let mut mix = vec![0.0f32; SLOT];
    for (i, &s) in pcm.iter().enumerate() {
        let j = (0.5 * FS) as usize + i;
        if j < SLOT {
            mix[j] += s;
        }
    }

    let mut state = seed.wrapping_add(1);
    let mut uniform = move || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((state >> 11) as f32 + 1.0) / ((1u64 << 53) as f32 + 1.0)
    };
    for s in mix.iter_mut() {
        let (a, b) = (uniform(), uniform());
        *s += (-2.0 * a.ln()).sqrt() * (2.0 * core::f32::consts::PI * b).cos();
    }

    let peak = mix.iter().map(|s| s.abs()).fold(0.0f32, f32::max);
    mix.iter().map(|&s| (s * 29_000.0 / peak) as i16).collect()
}

/// Sniper (aimed at `TARGET_HZ`) and wide-band decode counts over
/// `SEEDS` independent noise realisations of the same offset.
fn recall_at(offset_hz: f32) -> (u32, u32) {
    let msg = packed_cq();
    let (mut sniper, mut wide) = (0, 0);
    for seed in 0..SEEDS {
        let audio = slot_with_offset(&msg, offset_hz, seed);
        if SniperRequest::<Ft4>::new(&audio, TARGET_HZ, 15)
            .decode()
            .results
            .iter()
            .any(|r| r.message77() == msg)
        {
            sniper += 1;
        }
        if DecodeRequest::<Ft4>::new(&audio, 300.0, 2700.0, 1.2, 50)
            .decode()
            .results
            .iter()
            .any(|r| r.message77() == msg)
        {
            wide += 1;
        }
    }
    (sniper, wide)
}

/// The annulus itself: offsets that used to return **0/8** while
/// wide-band decoded the identical buffers 8/8.
///
/// 16 and 60 Hz bracket the failing band (`0` at every measured point
/// from 16 to 60 in the issue's own sweep); 40 Hz sits in its middle.
#[test]
fn sniper_decodes_across_the_former_blind_annulus() {
    for offset in [16.0f32, 20.0, 40.0, 60.0] {
        let (sniper, wide) = recall_at(offset);
        assert_eq!(
            wide, SEEDS as u32,
            "wide-band should decode {offset} Hz off-aim {SEEDS}/{SEEDS} — \
             if this fails the signal generator regressed, not the sniper path"
        );
        assert_eq!(
            sniper, SEEDS as u32,
            "sniper aimed at {TARGET_HZ} Hz missed a signal {offset} Hz away \
             ({sniper}/{SEEDS}) — issue #257's blind annulus is back; check \
             `engine::sync::rank_candidates`' aim-point budget reservation"
        );
    }
}

/// The offsets that worked *before* the fix, kept so the reservation
/// can never be widened into a regression at the aim point itself —
/// where `far` is empty and the reservation must degrade to "rank
/// purely by score".
#[test]
fn sniper_still_decodes_at_and_near_the_aim_point() {
    for offset in [0.0f32, 12.0, 100.0] {
        let (sniper, _) = recall_at(offset);
        assert_eq!(
            sniper, SEEDS as u32,
            "sniper regressed at {offset} Hz off-aim ({sniper}/{SEEDS}) — \
             this offset decoded fine before issue #257's fix"
        );
    }
}
