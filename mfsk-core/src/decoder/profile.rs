// SPDX-License-Identifier: GPL-3.0-only
//! Named decode profiles: a hardware class's values on the axes `docs/notes/PROFILES.md` lists.
//!
//! A [`Profile`] is data, not a code path (`PROFILES.md` section 5-1, decision D2: the library holds the values so that the applications
//! and the bindings name the same ones): [`Profile::spec`] is the [`ProfileSpec`] it stands for, and [`Profile::apply`] puts its `depth`
//! and AP on a [`DecodeParams`] (AP raised from off, never lowered) and nothing else. **Only the faithful pipeline** is described here; the cells flow's presets are
//! `CellsOptions::p0_window()` / `p0_late()`.
//!
//! | profile | for | `depth` | `ap` | `lanes` | `slot_budget` |
//! |---|---|---|---|---|---|
//! | [`P1`](Profile::P1) | Cortex-A9 class, one or two cores | [`Depth::Normal`] | [`ApMode::CqOnly`] | 2 | 0.8 |
//! | [`P2`](Profile::P2) | Raspberry Pi 3B class, four cores | [`Depth::Deep`] | [`ApMode::CqOnly`] | 4 | 0.9 |
//!
//! **These values are an extrapolation, not a measurement on the hardware.** They follow from the host's time per slot (one thread, 60
//! slots: Normal 200 ms, Deep 353 ms) times the 25-40x of a Pi 3B core (`docs/notes/PI3_JT9_GAP.md` section 13: Normal 5-8 s, Deep 9-14 s
//! a slot; an A9: Normal 7-12 s, `PROFILES.md` section 8). The faithful pipeline does not get faster with threads (1.07x on four, measured),
//! so a profile's cores go to `lanes` (slots decoded at the same time), and `slot_budget` (a share of the period, as
//! [`PeriodBudget::share`](super::PeriodBudget::share)) stops a decode that does not fit and reports what it has. The shares are
//! guesses to adjust on the hardware. `Normal` and `Deep` keep FT8's early decode (the `SicEarly` checkpoints); `Fast` has none.
//!
//! **The word.** A profile is not the JTTY `profile` integer, nor `strictness`'s accept / reject profile.

use super::{ApMode, DecodeParams, Depth};

/// A hardware class's named set of values. See the module documentation for what each stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Profile {
    /// Cortex-A9 class (`PROFILES.md` P1): `Depth::Normal`, AP on, two lanes, a budget of 0.8 of the period.
    P1,
    /// Raspberry Pi 3B class (`PROFILES.md` P2): `Depth::Deep`, AP on, four lanes, a budget of 0.9 of the period.
    P2,
}

/// The values a [`Profile`] stands for.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct ProfileSpec {
    /// The search depth ([`DecodeParams::depth`]).
    pub depth: Depth,
    /// Blind AP, with no QSO context assumed ([`DecodeParams::ap`]).
    pub ap: ApMode,
    /// Periods decoded at the same time: a decode that outlasts its period is the case this is for.
    pub lanes: usize,
    /// The share of a period one decode may use before it stops and reports what it has (`PeriodBudget::share`'s fraction).
    pub slot_budget: f32,
}

impl Profile {
    /// Every profile, in order.
    pub const ALL: [Profile; 2] = [Profile::P1, Profile::P2];

    /// The values this profile stands for.
    pub const fn spec(self) -> ProfileSpec {
        match self {
            Profile::P1 => ProfileSpec {
                depth: Depth::Normal,
                ap: ApMode::CqOnly,
                lanes: 2,
                slot_budget: 0.8,
            },
            Profile::P2 => ProfileSpec {
                depth: Depth::Deep,
                ap: ApMode::CqOnly,
                lanes: 4,
                slot_budget: 0.9,
            },
        }
    }

    /// `"p1"`, `"p2"`.
    pub const fn name(self) -> &'static str {
        match self {
            Profile::P1 => "p1",
            Profile::P2 => "p2",
        }
    }

    /// The profile named `name`, in any case; `None` for any other.
    pub fn from_name(name: &str) -> Option<Profile> {
        Profile::ALL
            .into_iter()
            .find(|p| p.name().eq_ignore_ascii_case(name.trim()))
    }

    /// `params` with this profile's `depth` and AP on; every other field (the band, the station, the QSO context) is `params`'.
    ///
    /// AP is **raised, never lowered**: `params` that have it off (FT8's and JT65's default) get the profile's `ap`; a mode whose default already runs
    /// it (`ApMode::Full`, "whenever the QSO context allows") keeps its own, so a profile does not narrow what a mode does by itself.
    pub fn apply(self, params: DecodeParams) -> DecodeParams {
        let s = self.spec();
        let ap = if params.ap == ApMode::Off {
            s.ap
        } else {
            params.ap
        };
        params.depth(s.depth).ap(ap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_profile_is_the_chain_it_stands_for() {
        // AP off, as FT8's default: the profile raises it
        let p = DecodeParams::for_band((300.0, 2800.0)).ap(ApMode::Off);
        assert_eq!(
            Profile::P1.apply(p.clone()),
            p.clone().depth(Depth::Normal).ap(ApMode::CqOnly)
        );
        assert_eq!(
            Profile::P2.apply(p.clone()),
            p.clone().depth(Depth::Deep).ap(ApMode::CqOnly)
        );
        // the band is the caller's
        assert_eq!(Profile::P1.apply(p).band_hz, (300.0, 2800.0));
        assert_eq!(
            Profile::P1.spec(),
            ProfileSpec {
                depth: Depth::Normal,
                ap: ApMode::CqOnly,
                lanes: 2,
                slot_budget: 0.8
            }
        );
        assert_eq!(
            Profile::P2.spec(),
            ProfileSpec {
                depth: Depth::Deep,
                ap: ApMode::CqOnly,
                lanes: 4,
                slot_budget: 0.9
            }
        );
    }

    #[test]
    fn ap_is_raised_never_lowered() {
        let p = DecodeParams::for_band((200.0, 3000.0));
        for pr in Profile::ALL {
            assert_eq!(pr.apply(p.clone().ap(ApMode::Off)).ap, ApMode::CqOnly);
            // a mode that runs AP by default keeps its own
            assert_eq!(pr.apply(p.clone().ap(ApMode::Full)).ap, ApMode::Full);
            assert_eq!(pr.apply(p.clone().ap(ApMode::CqOnly)).ap, ApMode::CqOnly);
        }
    }

    #[test]
    fn names_round_trip_in_any_case() {
        for p in Profile::ALL {
            assert_eq!(Profile::from_name(p.name()), Some(p));
            assert_eq!(Profile::from_name(&p.name().to_uppercase()), Some(p));
        }
        assert_eq!(Profile::from_name(" P2 "), Some(Profile::P2));
        for bad in ["", "p0", "p3", "p", "1", "deep"] {
            assert_eq!(Profile::from_name(bad), None, "{bad:?}");
        }
    }
}
