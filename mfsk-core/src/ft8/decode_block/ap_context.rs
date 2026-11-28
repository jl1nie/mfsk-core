// SPDX-License-Identifier: GPL-3.0-only
//! The QSO context of the cells flow's AP rung: what the operator's own QSO says about the message that is about to arrive, as a **fixed-size, `Copy`** value (no `String`, no `Vec`),
//! so a board can hold one and the cells flow can carry it in its pass context.
//!
//! **Ported from** `lib/ft8/ft8b.f90` (the `ipass` 5..12 AP hypotheses, `iaptype` 2..6) through this crate's own [`crate::msg::ap::ApHint::build_bits`] and
//! `msg::pipeline_ap::ap_passes`: the lock of each pass is the same 174 `(mask, value)` bits, built here without allocating (`tests::the_locks_are_the_faithful_hypotheses`
//! compares them bit for bit). Only the passes that need a known partner are here (5 `MyCall ??? ???`, 8 `MyCall DxCall ???`, 9 / 10 / 11 `MyCall DxCall RRR | RR73 | 73`); the blind-CQ
//! pass 12 is the staircase's own, and a station that is *calling CQ* has no partner's frequency to aim at (`ApStage::Calling` gives no pass here: pass 5 on every candidate failed
//! by BP would cost 2-3x the round, for +0.3 dB measured, `docs/notes/PI3_JT9_GAP.md` section 22).
//!
//! **Divergences from WSJT-X** (the rung is off unless the application sets a context):
//! - it runs only on a candidate within [`ApContext::WINDOW_HZ`] of the partner's frequency **and** [`ApContext::WINDOW_DT_S`] of its DT, for every pass (upstream runs `iaptype` 2 on every
//!   candidate and only 3.. near `nfqso`, with no DT window): to bound its time on a board, and its false accepts (section 24);
//! - it locks the bits in the one-symbol LLRs `a` and `d` that the staircase has already made (`ft8b.f90` starts each pass from `llra` and `llrc`; `c` is the nsym = 3 metric that
//!   `LlrEffort::Minimal` does not compute);
//! - the passes of a stage are tried in `[9, 10, 11, 8]` order (the deepest lock first, as `msg::pipeline_ap::ap_passes` does), not upstream's `iaptype` 3, 4, 5, 6.
//!
//! The lock bits are *derived* from `msg::ap::ApHint::build_bits` by test, not shared with it: that function allocates (`Vec`), this one is `Copy` and fixed-size for a board.

use crate::msg::wsjt77::pack28;

/// Codeword length (`Ldpc174_91Params::N`).
const N: usize = 174;

/// Where the QSO stands, as far as the AP is concerned: the message the partner is expected to send (`ft8b.f90`'s `naptypes(nQSOProgress, ..)`, `decoder::frame::iaptypes`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum ApStage {
    /// The operator called CQ: a reply can come from anywhere, so there is no partner to aim at. No pass.
    Calling,
    /// The operator answered the partner (or the partner a CQ): `MyCall DxCall <report or grid>`. Passes 5 and 8.
    Replying,
    /// The operator sent a report and waits for the roger: `MyCall DxCall` and `RRR`, `RR73` or `73`. Passes 8, 9, 10, 11 (upstream's `iaptype` 3..6).
    Rogers,
    /// The QSO is closing. Passes 8 and 5.
    Signoff,
}

impl ApStage {
    /// The passes of this stage, in the order they are tried (the deepest lock first, as `msg::pipeline_ap::ap_passes` does).
    const fn passes(self) -> &'static [u8] {
        match self {
            ApStage::Calling => &[],
            ApStage::Replying => &[8, 5],
            ApStage::Rogers => &[9, 10, 11, 8],
            ApStage::Signoff => &[8, 5],
        }
    }
}

/// The two callsigns of a QSO, the stage it is in and where the partner is: the cells flow's AP rung runs the stage's passes on the candidates near that frequency.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ApContext {
    /// `pack28` of the operator's call.
    my: u32,
    /// `pack28` of the partner's call.
    his: u32,
    stage: ApStage,
    /// The partner's frequency, Hz, as `f32::to_bits` (so the context is `Eq`).
    freq_bits: u32,
    /// The partner's DT, s, as `f32::to_bits`: where its last message sat (a partner's clock does not move between two periods).
    dt_bits: u32,
}

/// The lock of one pass: which of the 174 codeword bits the hypothesis fixes, and to what.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct ApLock {
    pub mask: [bool; N],
    pub value: [bool; N],
    /// How many bits are locked (32, 61 or 77).
    pub locked: usize,
}

impl ApContext {
    /// A candidate is **near** when its frequency is within this of the partner's: `engine::pipeline::QSO_WINDOW_HZ`, `ft8b.f90`'s `napwid`.
    pub const WINDOW_HZ: f32 = 50.0;
    /// ... and its DT within this of the partner's, s. **Not upstream's**: a false accept of the AP rung (a lock of 32 or 61 bits and a CRC that happens to pass) does not depend on where a candidate
    /// is, so the window the rung is allowed to run in is what its false accepts scale with (`docs/notes/PI3_JT9_GAP.md` section 24).
    pub const WINDOW_DT_S: f32 = 0.4;

    /// The context of a QSO with `his_call` (the operator's own call `my_call`) in `stage`, the partner last heard at `freq_hz` and `dt_sec` (a decode's `freq_hz` and `dt_sec`). `None` if
    /// either call is not a standard one `pack28` can lock (a hashed or compound call has no fixed 28 bits) or is one of the tokens `CQ`, `DE`, `QRZ` (which `pack28` also packs, but which
    /// are no station's call). Mind the order: the **operator's call first, the partner's second**, then the frequency, then the DT.
    pub fn new(
        my_call: &str,
        his_call: &str,
        stage: ApStage,
        freq_hz: f32,
        dt_sec: f32,
    ) -> Option<Self> {
        let is_token = |c: &str| {
            ["CQ", "DE", "QRZ"]
                .iter()
                .any(|t| c.trim().eq_ignore_ascii_case(t))
        };
        if is_token(my_call) || is_token(his_call) {
            return None;
        }
        Some(Self {
            my: pack28(my_call)?,
            his: pack28(his_call)?,
            stage,
            freq_bits: freq_hz.to_bits(),
            dt_bits: dt_sec.to_bits(),
        })
    }

    /// The partner's frequency, Hz.
    pub fn freq_hz(&self) -> f32 {
        f32::from_bits(self.freq_bits)
    }

    /// The partner's DT, s.
    pub fn dt_sec(&self) -> f32 {
        f32::from_bits(self.dt_bits)
    }

    /// The stage of the QSO this context is for.
    pub fn stage(&self) -> ApStage {
        self.stage
    }

    /// This context with the partner at `freq_hz` and `dt_sec` (the partner moved, or was heard again).
    #[must_use]
    pub fn at(self, freq_hz: f32, dt_sec: f32) -> Self {
        Self {
            freq_bits: freq_hz.to_bits(),
            dt_bits: dt_sec.to_bits(),
            ..self
        }
    }

    /// Whether a candidate at `freq_hz` and `dt_sec` is within [`Self::WINDOW_HZ`] and [`Self::WINDOW_DT_S`] of the partner.
    pub fn is_near(&self, freq_hz: f32, dt_sec: f32) -> bool {
        (freq_hz - self.freq_hz()).abs() <= Self::WINDOW_HZ
            && (dt_sec - self.dt_sec()).abs() <= Self::WINDOW_DT_S
    }

    /// The pass ids of this context's stage, in the order they are tried.
    pub fn passes(&self) -> &'static [u8] {
        self.stage.passes()
    }

    /// The lock of `pass` (5, 8, 9, 10 or 11); `None` for any other id.
    pub fn lock(&self, pass: u8) -> Option<ApLock> {
        let mut mask = [false; N];
        let mut value = [false; N];
        let call = |mask: &mut [bool; N], value: &mut [bool; N], n28: u32, start: usize| {
            for i in 0..28 {
                mask[start + i] = true;
                value[start + i] = (n28 >> (27 - i)) & 1 == 1;
            }
            // the flag bit after a standard call (`ipa` / `ipb`) is 0
            mask[start + 28] = true;
        };
        match pass {
            5 => call(&mut mask, &mut value, self.my, 0),
            8..=11 => {
                call(&mut mask, &mut value, self.my, 0);
                call(&mut mask, &mut value, self.his, 29);
            }
            _ => return None,
        }
        if (9..=11).contains(&pass) {
            // RRR, RR73, 73 are `MAXGRID4 + 2`, `+ 3`, `+ 4`, with the `ir` flag (bit 58) 0
            let igrid: u32 = 32_400 + u32::from(pass - 7);
            mask[58] = true;
            for i in 0..15 {
                mask[59 + i] = true;
                value[59 + i] = (igrid >> (14 - i)) & 1 == 1;
            }
        }
        // i3 = 001, locked whenever a call is known
        mask[74] = true;
        mask[75] = true;
        mask[76] = true;
        value[76] = true;
        let locked = mask.iter().filter(|&&m| m).count();
        Some(ApLock {
            mask,
            value,
            locked,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::msg::ap::ApHint;

    /// **Fails if** a lock differs, in any of the 174 bits, from what `ApHint::build_bits` (the faithful pipeline's hypothesis for the same pass) makes: the bit positions, the `pack28` flag, the i3 lock, the
    /// RRR / RR73 / 73 constants, or the locked count (32, 61, 77).
    #[test]
    fn the_locks_are_the_faithful_hypotheses() {
        for (mine, his) in [("K1JT", "HA0DU"), ("JA1ZZZ", "VK3YYY"), ("W9XYZ", "3Y0Z")] {
            let ctx = ApContext::new(mine, his, ApStage::Rogers, 1500.0, 0.0).unwrap();
            let want = |hint: ApHint| {
                let (m, v) = hint.build_bits(N);
                (m, v)
            };
            for (pass, hint, locked) in [
                (5u8, ApHint::new().with_call1(mine), 32usize),
                (8, ApHint::new().with_call1(mine).with_call2(his), 61),
                (
                    9,
                    ApHint::new()
                        .with_call1(mine)
                        .with_call2(his)
                        .with_report("RRR"),
                    77,
                ),
                (
                    10,
                    ApHint::new()
                        .with_call1(mine)
                        .with_call2(his)
                        .with_report("RR73"),
                    77,
                ),
                (
                    11,
                    ApHint::new()
                        .with_call1(mine)
                        .with_call2(his)
                        .with_report("73"),
                    77,
                ),
            ] {
                let lock = ctx.lock(pass).unwrap();
                let (m, v) = want(hint);
                assert_eq!(lock.locked, locked, "{mine} {his} pass {pass}");
                for i in 0..N {
                    assert_eq!(lock.mask[i], m[i] != 0, "{mine} {his} pass {pass} mask {i}");
                    if lock.mask[i] {
                        assert_eq!(
                            lock.value[i],
                            v[i] != 0,
                            "{mine} {his} pass {pass} value {i}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn a_context_is_a_small_copy_value_and_knows_who_is_near() {
        assert!(core::mem::size_of::<ApContext>() <= 24);
        let ctx = ApContext::new("K1JT", "HA0DU", ApStage::Rogers, 1500.0, 0.1).unwrap();
        let moved = ctx.at(1620.0, 0.5);
        assert!(ctx.is_near(1549.0, 0.1) && !ctx.is_near(1551.0, 0.1));
        // the DT is part of "near": 0.4 s either side of the partner's
        assert!(ctx.is_near(1500.0, 0.49) && !ctx.is_near(1500.0, 0.51));
        assert!(ctx.is_near(1500.0, -0.29) && !ctx.is_near(1500.0, -0.31));
        assert!(moved.is_near(1600.0, 0.5) && !moved.is_near(1500.0, 0.5));
        assert_eq!(ctx.passes(), &[9, 10, 11, 8]);
        assert_eq!(
            ApContext::new("K1JT", "HA0DU", ApStage::Calling, 1500.0, 0.0)
                .unwrap()
                .passes(),
            &[] as &[u8]
        );
        // a call `pack28` cannot lock (a hash) makes no context
        assert!(ApContext::new("K1JT", "<...>", ApStage::Rogers, 1500.0, 0.0).is_none());
        // ... nor the tokens `pack28` also packs
        for token in ["CQ", "de", "QRZ"] {
            assert!(ApContext::new(token, "HA0DU", ApStage::Rogers, 1500.0, 0.0).is_none());
            assert!(ApContext::new("K1JT", token, ApStage::Rogers, 1500.0, 0.0).is_none());
        }
        assert!(ctx.lock(6).is_none() && ctx.lock(12).is_none());
    }
}
