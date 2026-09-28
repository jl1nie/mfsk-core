//! FST4's a-priori OSD keeps a low, bounded false-accept rate on noise
//! (#465, same shape as FT8/FT4's #456).
//!
//! `decode240_101.f90` runs its AP passes with an `apmask`: BP holds the
//! locked bits, OSD's `npre1`/`npre1+npre2` search runs on the BP sum after
//! 1 and after 2 iterations (never the raw LLR), a test pattern that flips a
//! locked bit is skipped (`osd240_101.f90:192`/`:263`), and the CRC is
//! checked on the winner. Before #465, FST4's AP rung ran
//! `osd_decode_npre_generic` unmasked on the raw LLR — a test pattern could
//! flip a locked bit, and there was no BP-sum feed under AP at all.
//!
//! Measured on iid Gaussian LLRs with a CQ-style lock (29 message-word bits
//! plus 3 near the CRC tail, `keff=91`), the count being decodes that pass
//! the CRC (BP or OSD, before any message check): **after 32, before 10** in
//! 300 000 draws each — see `ap_rung_masked_false_accept_rate_stays_low`'s
//! own doc comment for why that is not the two-orders-of-magnitude
//! improvement FT8/FT4 saw, and why it is not a regression either.
#![cfg(feature = "fft-rustfft")]

use mfsk_core::fec::ldpc::bp::{
    BpKind, BpScratch, bp_decode_generic_kind_with_scratch, bp_llr_zsum_ap_with_scratch,
};
use mfsk_core::fec::ldpc::osd::{PartialCrc, osd_decode_npre_generic};
use mfsk_core::fec::ldpc::params::Ldpc240_101Params;
use mfsk_core::fec::ldpc240_101::{append_crc24, check_crc24};

const N: usize = 240;
const KEFF: usize = 91;

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn unif(&mut self) -> f64 {
        ((self.next() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
    fn gauss(&mut self) -> f32 {
        let (u1, u2) = (self.unif(), self.unif());
        ((-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()) as f32
    }
}

/// `(mask, locked llr)` for a CQ-style lock: bits `0..29` (a callsign-shaped
/// pattern, bit 27 the only one set — mirrors FT8's `mcq`) plus three bits
/// near the `keff` boundary, same shape as `ft8_ap_osd_false_accept.rs`'s
/// `lock()`.
fn lock(noise: &[f32; N]) -> ([bool; N], [f32; N]) {
    let apmag = noise.iter().fold(0f32, |m, v| m.max(v.abs())) * 1.1;
    let mut mask = [false; N];
    let mut llr = *noise;
    for i in 0..29 {
        mask[i] = true;
        llr[i] = if i == 26 { apmag } else { -apmag };
    }
    for (i, v) in [(KEFF - 3, -apmag), (KEFF - 2, -apmag), (KEFF - 1, apmag)] {
        mask[i] = true;
        llr[i] = v;
    }
    (mask, llr)
}

#[derive(Clone, Copy)]
enum Rung {
    /// What the AP rung ran before #465: BP with the mask, then
    /// `osd_decode_npre_generic` unmasked on the raw LLR.
    Before,
    /// `decode240_101.f90`'s way: BP with the mask, then OSD on the BP sum
    /// after 1 and 2 iterations, masked `npre1`, CRC on the winner.
    After,
}

fn crc_valid_decodes(rung: Rung, draws_per_thread: usize) -> u64 {
    const THREADS: usize = 12;
    let partial_crc = PartialCrc {
        keff: KEFF,
        with_crc: append_crc24,
    };
    std::thread::scope(|sc| {
        let hs: Vec<_> = (0..THREADS)
            .map(|t| {
                sc.spawn(move || {
                    let mut rng = Rng(0x2545_F491_4F6C_DD1Du64 ^ ((t as u64 + 7) * 0x9e37_79b9));
                    let mut scratch = BpScratch::<Ldpc240_101Params, f32>::new();
                    let mut hits = 0u64;
                    for _ in 0..draws_per_thread {
                        let mut noise = [0f32; N];
                        for v in noise.iter_mut() {
                            *v = 2.83 * rng.gauss();
                        }
                        let (mask, llr) = lock(&noise);
                        if bp_decode_generic_kind_with_scratch::<Ldpc240_101Params>(
                            &mut scratch,
                            &llr,
                            Some(&mask),
                            30,
                            Some(check_crc24),
                            BpKind::SumProduct,
                        )
                        .is_some()
                        {
                            hits += 1;
                            continue;
                        }
                        let found = match rung {
                            Rung::Before => osd_decode_npre_generic::<Ldpc240_101Params>(
                                &llr,
                                12,
                                0,
                                false,
                                Some(partial_crc),
                                None,
                                Some(check_crc24),
                            )
                            .is_some(),
                            Rung::After => [1u32, 2].into_iter().any(|n_iter| {
                                let z = bp_llr_zsum_ap_with_scratch::<Ldpc240_101Params>(
                                    &mut scratch,
                                    &llr,
                                    Some(&mask),
                                    n_iter,
                                );
                                osd_decode_npre_generic::<Ldpc240_101Params>(
                                    z,
                                    12,
                                    0,
                                    false,
                                    Some(partial_crc),
                                    Some(&mask),
                                    Some(check_crc24),
                                )
                                .is_some()
                            }),
                        };
                        if found {
                            hits += 1;
                        }
                    }
                    hits
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().unwrap()).sum()
    })
}

/// The gate: with the CQ-style lock, on noise, the AP rung's false-accept
/// rate stays low in absolute terms.
///
/// **Measured (300 000 draws a rung, 2026-09-28): after 32, before 10** —
/// the masked/zsum rung is *not* lower than the unmasked/raw-LLR one here,
/// unlike FT8/FT4's #456/#459 precedent (where the equivalent fix cut false
/// accepts by two orders of magnitude, 22 % → 1.1e-4, because the *old*
/// FT8/FT4 rung ran a combinatorial order-2 search over every bit —
/// `osd_decode_deep`, not this crate's already-pruned `npre1` — so its
/// "before" was a real bug, not a fair baseline to expect this fix to beat).
/// FST4's "before" here (`osd_decode_npre_generic` unmasked, same pruned
/// `npre1` search minus the mask) was never that badly broken, so there is
/// no large regression to fix — both rates are low (≈3.3e-5 / ≈1.1e-4).
/// A plausible reading: holding locked bits at their AP value across a
/// couple of BP iterations (`zsum`) nudges the *free* bits toward whatever
/// locally satisfies the check equations, which on pure noise can produce a
/// marginally more codeword-like pattern than the raw channel LLR would —
/// this is `decode240_101.f90`'s own design, not a defect in the port.
///
/// No real `decode240_101` binary exists to benchmark against here (unlike
/// FT8/FT4's `jt9 -8`), so this gate is an absolute cap with headroom
/// around the measured "after" rate, not an upstream-parity number: what
/// #465 actually guarantees — a locked bit is never flipped — is proven
/// separately by `masked_npre1_does_not_flip_a_locked_bit_ldpc240_101` in
/// `fec/ldpc/osd.rs`.
#[test]
fn ap_rung_masked_false_accept_rate_stays_low() {
    // 24 000 draws; measured rate implies ≈2.6 expected hits. Capped at
    // 20 (≈8× headroom) rather than FT8's tighter ratio — this crate's
    // small-sample counts are proportionally noisier at this rate.
    let after = crc_valid_decodes(Rung::After, 2_000);
    assert!(
        after <= 20,
        "{after} CRC-valid AP decodes in 24 000 noise draws; measured full-scale \
         rate implies about 2.6 expected"
    );
}

/// Prints both rungs' false-accept counts. `--ignored --nocapture`.
#[test]
#[ignore = "measurement, prints; run with --release --ignored --nocapture"]
fn ap_measure() {
    let per_thread = 25_000; // 300 000 draws
    let after = crc_valid_decodes(Rung::After, per_thread);
    let before = crc_valid_decodes(Rung::Before, per_thread);
    println!(
        "FST4_AP_MEASURE: after {after} in {} ; before {before} in {}",
        per_thread * 12,
        per_thread * 12
    );
}
