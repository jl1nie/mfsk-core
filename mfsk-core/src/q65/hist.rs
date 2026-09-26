// SPDX-License-Identifier: GPL-3.0-or-later
//! The DX station from recent decodes — a port of `q65_hist` in
//! `lib/qra/q65/q65.f90`.
//!
//! WSJT-X keeps the 100 most recent Q65 decodes with their frequencies
//! (`call q65_hist(nint(f0dec),msg0=decoded)` after every decode,
//! `q65_decode.f90:341,454`). On a manual **Decode Again** with no DX call
//! entered (`if(lagain) call q65_hist(nfqso,dxcall=hiscall,dxgrid=hisgrid)`,
//! `q65_decode.f90:153-155`) it takes the DX call — and the grid, if the
//! message carries one — from the most recent decode near the Rx
//! frequency, so the full-AP list (`q65_set_list`, here
//! [`super::standard_qso_codewords`]) can be built without the operator
//! typing the call.
//!
//! The decoder here is stateless, so the history is the application's:
//! [`Q65History`] holds it, [`Q65History::record`] feeds it each decode,
//! and [`Q65History::lookup`] is the Decode Again step.

use alloc::collections::VecDeque;
use alloc::string::String;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std makes f32's own methods shadow it
use num_traits::Float;

/// `MAXHIST` in `q65_hist`.
pub const MAX_HIST: usize = 100;

/// `abs(nf0(i)-if0).gt.10`: how far, in Hz, a remembered decode may be
/// from the Rx frequency and still name the DX station.
pub const HIST_FREQ_TOL_HZ: i32 = 10;

/// What [`Q65History::lookup`] found: `q65_hist`'s `dxcall` and `dxgrid`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DxFromHistory {
    /// The message's second word (`msg(i1+1:i2-1)`), up to 12 characters.
    pub call: String,
    /// The third word's first four characters when they form a grid
    /// square other than `RR73`.
    pub grid: Option<String>,
}

/// The 100 most recent decodes and their frequencies, oldest first —
/// `q65_hist`'s `msg` / `nf0` arrays.
#[derive(Clone, Debug, Default)]
pub struct Q65History {
    entries: VecDeque<(i32, String)>,
}

impl Q65History {
    pub fn new() -> Self {
        Self::default()
    }

    /// Remember a decode at `freq_hz` (tone 0, as [`super::Q65Result::freq_hz`]),
    /// dropping the oldest once 100 are held.
    pub fn push(&mut self, freq_hz: f32, message: &str) {
        if self.entries.len() == MAX_HIST {
            self.entries.pop_front();
        }
        // `nf0(nhist)=if0`, `if0=nint(f0dec)`.
        self.entries
            .push_back((freq_hz.round() as i32, String::from(message)));
    }

    /// [`Self::push`] for a decode result.
    #[cfg(any(feature = "fft-rustfft", feature = "fft-extern"))]
    pub fn record(&mut self, r: &super::Q65Result) {
        self.push(r.freq_hz, &r.message);
    }

    /// The DX call, and grid if any, from the most recent decode within
    /// 10 Hz of `rx_freq_hz` (`nfqso`) whose first word is 3 to 12
    /// characters long — so a `CQ ...` decode is passed over for an
    /// older one. `None` when nothing qualifies, or when the most recent
    /// qualifying decode has no second word (upstream stops there with a
    /// blank call).
    ///
    /// WSJT-X asks only on Decode Again and only when no DX call is
    /// entered (`if(dxcall(1:3).ne.'   ') go to 900`); calling it then is
    /// the application's side of the port.
    pub fn lookup(&self, rx_freq_hz: f32) -> Option<DxFromHistory> {
        let if0 = rx_freq_hz.round() as i32;
        for (f, msg) in self.entries.iter().rev() {
            if (f - if0).abs() > HIST_FREQ_TOL_HZ {
                continue;
            }
            if let Some(dx) = parse(msg) {
                // A one-word message (`K1ABC`) qualifies and ends the
                // search with a blank `dxcall` (`exit`), as upstream.
                return (!dx.call.is_empty()).then_some(dx);
            }
        }
        None
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// `q65_hist`'s extraction from a 37-character, blank-padded message:
/// `i1=index(msg,' ')` must be 4..=13 (a first word of 3 to 12
/// characters); the call runs to the next blank; `g1` is the four
/// characters after that blank.
fn parse(msg: &str) -> Option<DxFromHistory> {
    // The fixed-width field upstream reads from.
    let mut m: [u8; 37] = [b' '; 37];
    for (d, s) in m.iter_mut().zip(msg.bytes()) {
        *d = s;
    }
    let i1 = m.iter().position(|&c| c == b' ')?; // 0-based: Fortran i1 - 1
    if !(3..=12).contains(&i1) {
        return None;
    }
    let rest = &m[i1 + 1..];
    let i2 = i1 + 1 + rest.iter().position(|&c| c == b' ').unwrap_or(rest.len());
    let call: String = m[i1 + 1..i2].iter().map(|&c| c as char).collect();
    // `dxcall` is `character(len=12)`.
    let call: String = call.chars().take(12).collect();
    let g1: [u8; 4] = core::array::from_fn(|k| m.get(i2 + 1 + k).copied().unwrap_or(b' '));
    let isgrid = (b'A'..=b'R').contains(&g1[0])
        && (b'A'..=b'R').contains(&g1[1])
        && g1[2].is_ascii_digit()
        && g1[3].is_ascii_digit()
        && &g1 != b"RR73";
    Some(DxFromHistory {
        call,
        grid: isgrid.then(|| g1.iter().map(|&c| c as char).collect()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dx(call: &str, grid: Option<&str>) -> Option<DxFromHistory> {
        Some(DxFromHistory {
            call: call.into(),
            grid: grid.map(Into::into),
        })
    }

    #[test]
    fn lookup_takes_the_most_recent_near_the_rx_frequency() {
        let mut h = Q65History::new();
        h.push(1500.0, "K1ABC W9XYZ EN37");
        h.push(1700.0, "K1ABC JA1ABC PM95");
        h.push(1508.0, "K1ABC W9XYZ -12");
        assert_eq!(h.lookup(1500.0), dx("W9XYZ", None));
        assert_eq!(h.lookup(1695.4), dx("JA1ABC", Some("PM95")));
        assert_eq!(h.lookup(1511.0), dx("W9XYZ", None));
        assert_eq!(h.lookup(1300.0), None);
        // 1500 vs 1511: 11 Hz, outside `abs(...).gt.10`.
        let mut h = Q65History::new();
        h.push(1500.0, "K1ABC W9XYZ EN37");
        assert_eq!(h.lookup(1511.0), None);
        assert_eq!(h.lookup(1510.0), dx("W9XYZ", Some("EN37")));
    }

    #[test]
    fn a_short_first_word_is_passed_over() {
        let mut h = Q65History::new();
        h.push(1500.0, "K1ABC W9XYZ EN37");
        h.push(1500.0, "CQ JA1ABC PM95");
        // "CQ" is 2 characters: `i1 = 3` fails `i1.ge.4`; the older one answers.
        assert_eq!(h.lookup(1500.0), dx("W9XYZ", Some("EN37")));
    }

    #[test]
    fn rr73_and_reports_are_not_grids() {
        let mut h = Q65History::new();
        h.push(1500.0, "K1ABC W9XYZ RR73");
        assert_eq!(h.lookup(1500.0), dx("W9XYZ", None));
        h.push(1500.0, "K1ABC W9XYZ R-15");
        assert_eq!(h.lookup(1500.0), dx("W9XYZ", None));
    }

    /// Each message alone in a history, looked up at its own frequency,
    /// against `q65_hist` from WSJT-X (`libwsjt_fort.a`, the routine is
    /// unchanged in v3.2.0-rc1) run on the same messages, 2026-09-26.
    #[test]
    fn matches_q65_hist() {
        let cases: &[(&str, &str, &str)] = &[
            ("K1ABC W9XYZ EN37", "W9XYZ", "EN37"),
            ("CQ JA1ABC PM95", "", ""),
            ("K1ABC W9XYZ RR73", "W9XYZ", ""),
            ("ABCDEFGHIJKL W9XYZ EN37", "W9XYZ", "EN37"),
            ("ABCDEFGHIJKLM W9XYZ EN37", "", ""),
            ("K1ABC <W9XYZ> -15", "<W9XYZ>", ""),
            ("K1ABC", "", ""),
            ("TNX 73 GL", "73", ""),
            ("K1ABC W9XYZ R-15", "W9XYZ", ""),
        ];
        for &(msg, call, grid) in cases {
            let mut h = Q65History::new();
            h.push(1500.0, msg);
            let got = h.lookup(1500.0);
            let want = (!call.is_empty()).then(|| DxFromHistory {
                call: call.into(),
                grid: (!grid.is_empty()).then(|| grid.into()),
            });
            assert_eq!(got, want, "{msg}");
        }
        // The one-word message stops the search rather than passing over.
        let mut h = Q65History::new();
        h.push(1500.0, "K1ABC W9XYZ EN37");
        h.push(1500.0, "K1ABC");
        assert_eq!(h.lookup(1500.0), None);
    }

    #[test]
    fn holds_the_last_hundred() {
        let mut h = Q65History::new();
        h.push(1000.0, "K1ABC W9XYZ EN37");
        for _ in 0..MAX_HIST {
            h.push(2000.0, "K1ABC JA1ABC PM95");
        }
        assert_eq!(h.len(), MAX_HIST);
        assert_eq!(h.lookup(1000.0), None);
    }
}
