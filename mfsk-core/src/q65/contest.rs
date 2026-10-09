// SPDX-License-Identifier: GPL-3.0-only
//! The NA VHF / WW Digi / ARRL Digi contest caller list — a port of
//! `q65_hist2` (`lib/qra/q65/q65.f90`) and `q65_set_list2`
//! (`lib/qra/q65/q65_set_list2.f90`).
//!
//! In contest mode (`ncontest.eq.1`) WSJT-X remembers up to 50 stations
//! that called with a grid, and builds its full-AP list from all of them
//! rather than from the one DX call: every `MyCall Caller Grid`,
//! `MyCall Caller R Grid`, `RRR`, `RR73` and `73`, each with the 78th bit
//! both clear and set. That list is what lets several callers be decoded
//! with q3 in one period (`super::DecodeRequest::ap_list` with an Rx
//! frequency).
//!
//! The list is the application's to keep, as `q65_decode.f90` keeps it
//! in `tsil.3q` between decodes: [`Q65Callers::record`] after each decode,
//! [`Q65Callers::expire`] before the next, [`contest_codewords`] to build
//! the list. Times are the caller's (Unix seconds, as `time()`), since
//! the crate reads no clock.

use alloc::string::String;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// needed with no std in the graph; a dep linking std makes f32's own methods shadow it
use num_traits::Float;

use crate::fec::qra::Q65Codec;
use crate::fec::qra15_65_64::QRA15_65_64_IRR_E23;
use crate::msg::q65::{pack77_q65, pack77_to_symbols_flagged};

/// `Q65_MAX_CALLERS` (`q65_limits.h` `Q65_CALLER_CAPACITY`, v3.3.0-beta1).
pub const MAX_CALLERS: usize = 50;
/// `Q65_AP_LIST_CAPACITY` in `q65_limits.h`: one empty codeword, then every
/// caller **and** the current DX station × 5 messages × 2 flags,
/// `1 + 10 * (50 + 1)`. rc1 had `MAX_NCW = 411` (41 callers) against a
/// 50-caller list, which overflowed; v3.3.0-beta1 sizes it for the full list.
pub const MAX_CONTEST_CODEWORDS: usize = 1 + 10 * (MAX_CALLERS + 1);
/// `hours.gt.24.0`: a caller not heard for a day is dropped.
pub const CALLER_TTL_SEC: u64 = 24 * 3600;

/// One remembered station — `q3list`'s `call`, `grid`, `nsec`, `nfreq`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Caller {
    /// Up to six characters (`character*6 c6`).
    pub call: String,
    /// The four-character grid it sent.
    pub grid: String,
    /// When it was last decoded, Unix seconds.
    pub last_heard: u64,
    /// Its audio frequency then, Hz (`nint(f0dec)`).
    pub freq_hz: i32,
}

/// `q65_hist2`'s `callers(1:nhist2)`, oldest first.
#[derive(Clone, Debug, Default)]
pub struct Q65Callers {
    callers: Vec<Caller>,
}

/// `isgrid`: two letters A..R, two digits, and not `RR73`.
fn isgrid(g: &[u8]) -> bool {
    g.len() >= 4
        && (b'A'..=b'R').contains(&g[0])
        && (b'A'..=b'R').contains(&g[1])
        && g[2].is_ascii_digit()
        && g[3].is_ascii_digit()
        && &g[..4] != b"RR73"
}

impl Q65Callers {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn callers(&self) -> &[Caller] {
        &self.callers
    }

    /// `q65_hist2(nfreq, msg0, ...)` for a decode at `freq_hz`, heard at
    /// `now` (Unix seconds).
    ///
    /// A message with a `/` is ignored; ` R ` is taken out; the second word
    /// (six characters) is the caller and the next four characters its
    /// grid. A caller already listed has its time and frequency refreshed;
    /// a new one is added only when it sent a grid. Once 50 are held the
    /// one **heard longest ago** makes room (`minloc(callers%nsec)`, the first
    /// on a tie): v3.3.0-beta1 `q65_record_caller`. rc1 dropped index 1, the
    /// oldest *inserted*, so a caller that kept calling was evicted anyway.
    /// Callers older than a day are expired first, as upstream does before
    /// recording.
    pub fn record(&mut self, freq_hz: f32, message: &str, now: u64) {
        self.expire(now);
        if message.contains('/') {
            return;
        }
        let mut m: Vec<u8> = message.bytes().collect();
        m.resize(37.max(m.len()), b' ');
        m.truncate(37);
        // `i0=index(msg,' R '); if(i0.ge.7) msg=msg(1:i0)//msg(i0+3:)`.
        if let Some(p) = m.windows(3).position(|w| w == b" R ")
            && p + 1 >= 7
        {
            m.drain(p + 1..p + 3);
            m.resize(37, b' ');
        }
        let (c6, g4) = match m.iter().position(|&c| c == b' ') {
            Some(i1) if (3..=12).contains(&i1) => {
                let rest = &m[i1 + 1..];
                let i2 = i1 + 1 + rest.iter().position(|&c| c == b' ').unwrap_or(rest.len());
                // `if(i2.le.i1+1 .or. i2+4.gt.len(text)) return` (1-based there):
                // an empty second word, or a grid that would run past the 37 characters.
                if i2 == i1 + 1 || i2 + 5 > 37 {
                    return;
                }
                let call: String = m[i1 + 1..i2].iter().take(6).map(|&c| c as char).collect();
                let g: Vec<u8> = (0..4)
                    .map(|k| m.get(i2 + 1 + k).copied().unwrap_or(b' '))
                    .collect();
                (call, g)
            }
            _ => (String::new(), Vec::from(*b"    ")),
        };
        let freq = freq_hz.round() as i32;
        if let Some(c) = self.callers.iter_mut().find(|c| c.call == c6) {
            c.last_heard = now;
            c.freq_hz = freq;
            return;
        }
        if isgrid(&g4) {
            if self.callers.len() == MAX_CALLERS
                && let Some(oldest) = self
                    .callers
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, c)| c.last_heard)
                    .map(|(i, _)| i)
            {
                // `callers(oldest:count-1)=callers(oldest+1:count)`: order kept.
                self.callers.remove(oldest);
            }
            self.callers.push(Caller {
                call: c6,
                grid: g4.iter().map(|&c| c as char).collect(),
                last_heard: now,
                freq_hz: freq,
            });
        }
    }

    /// Drop callers not heard for more than 24 hours, as `q65_decode`
    /// does when it reads the list back (`hours.gt.24.0`).
    ///
    /// Upstream's loop shifts the list down inside a `do i=1,nhist2` and
    /// so skips the entry that moves into place; that one goes on the
    /// next decode instead. Here every stale entry goes at once.
    pub fn expire(&mut self, now: u64) {
        self.callers
            .retain(|c| now.saturating_sub(c.last_heard) <= CALLER_TTL_SEC);
    }

    /// `rm_q3list`: forget one caller (worked, say).
    pub fn remove(&mut self, call: &str) {
        self.callers.retain(|c| c.call != call);
    }
}

/// `q65_set_list2(mycall, hiscall, hisgrid, callers, ...)`: the contest
/// full-AP list. The first codeword is all zeros (`codewords(:,1)=0`);
/// then, for every caller — and the current DX station too, when it has a
/// standard call and a grid and is not already listed — `MyCall Caller
/// Grid`, `… R Grid`, `… RRR`, `… RR73` and `… 73`, each encoded with the
/// 78th bit clear and set. A message that will not pack is skipped.
pub fn contest_codewords(
    my_call: &str,
    his_call: &str,
    his_grid: &str,
    callers: &Q65Callers,
) -> Vec<[i32; 63]> {
    let mut codec = Q65Codec::new(&QRA15_65_64_IRR_E23);
    let mut out: Vec<[i32; 63]> = Vec::with_capacity(MAX_CONTEST_CODEWORDS);
    out.push([0; 63]);
    let his6: String = his_call.trim().chars().take(6).collect();
    let add_his = crate::msg::wsjt77::pack77(my_call, his_call, "").is_some()
        && isgrid(his_grid.as_bytes())
        && !callers.callers.iter().any(|c| c.call == his6);
    let mut stations: Vec<(String, String)> = callers
        .callers
        .iter()
        .map(|c| (c.call.clone(), c.grid.clone()))
        .collect();
    // v3.3.0-beta1 `q65_set_list2`: `jmax=nhist2+1`, no longer clamped to 50,
    // so the DX station is the 51st when 50 callers are held (rc1 left it out).
    if add_his {
        stations.push((his6, his_grid.chars().take(4).collect()));
    }
    for (c6, g4) in &stations {
        let r_grid = alloc::format!("R {g4}");
        for tail in [g4.as_str(), r_grid.as_str(), "RRR", "RR73", "73"] {
            let Some(bits) = pack77_q65(my_call, c6, tail) else {
                continue;
            };
            for flag in [false, true] {
                let info = pack77_to_symbols_flagged(&bits, flag);
                let mut cw = [0_i32; 63];
                codec.encode(&info, &mut cw);
                out.push(cw);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_callers_with_grids() {
        let mut h = Q65Callers::new();
        h.record(1500.0, "K1ABC W9XYZ EN37", 100);
        h.record(1510.0, "K1ABC JA1ABC R PM95", 100);
        h.record(1520.0, "K1ABC VK3ABC -15", 100); // no grid: not added
        h.record(1530.0, "K1ABC W9XYZ/R EN37", 100); // compound: ignored
        let calls: Vec<_> = h
            .callers()
            .iter()
            .map(|c| (c.call.as_str(), c.grid.as_str()))
            .collect();
        assert_eq!(calls, [("W9XYZ", "EN37"), ("JA1ABC", "PM95")]);
        // A known caller is refreshed, grid or not.
        h.record(1600.0, "K1ABC W9XYZ RR73", 500);
        assert_eq!(h.callers()[0].last_heard, 500);
        assert_eq!(h.callers()[0].freq_hz, 1600);
    }

    #[test]
    fn six_character_calls_and_the_cap() {
        let mut h = Q65Callers::new();
        h.record(1500.0, "K1ABC JA1ABCD PM95", 0);
        assert_eq!(h.callers()[0].call, "JA1ABC");
        for k in 0..MAX_CALLERS {
            h.record(1500.0, &alloc::format!("K1ABC W{k}AB EN37"), 1);
        }
        assert_eq!(h.callers().len(), MAX_CALLERS);
        assert_ne!(h.callers()[0].call, "JA1ABC");
    }

    /// v3.3.0-beta1 evicts the caller heard longest ago, not the first inserted.
    #[test]
    fn full_list_evicts_the_least_recently_heard() {
        let mut h = Q65Callers::new();
        for k in 0..MAX_CALLERS {
            h.record(1500.0, &alloc::format!("K1ABC W{k}AB EN37"), 10 + k as u64);
        }
        // The first inserted calls again, so it is no longer the oldest.
        h.record(1500.0, "K1ABC W0AB RR73", 1000);
        h.record(1500.0, "K1ABC N0NEW EN37", 1001);
        let calls: Vec<_> = h.callers().iter().map(|c| c.call.as_str()).collect();
        assert_eq!(calls.len(), MAX_CALLERS);
        assert!(calls.contains(&"W0AB"), "refreshed caller must survive");
        assert!(!calls.contains(&"W1AB"), "least recently heard is evicted");
        assert_eq!(calls.last(), Some(&"N0NEW"));
        // Ties go to the first, as `minloc` does.
        let mut t = Q65Callers::new();
        for k in 0..MAX_CALLERS {
            t.record(1500.0, &alloc::format!("K1ABC W{k}AB EN37"), 5);
        }
        t.record(1500.0, "K1ABC N0NEW EN37", 6);
        assert!(!t.callers().iter().any(|c| c.call == "W0AB"));
    }

    /// `record` expires first, and an empty second word is ignored.
    #[test]
    fn record_expires_and_rejects_an_empty_call() {
        let mut h = Q65Callers::new();
        h.record(1500.0, "K1ABC W9XYZ EN37", 0);
        h.record(1500.0, "K1ABC  EN37", CALLER_TTL_SEC + 5);
        assert!(
            h.callers().is_empty(),
            "the stale caller goes on the next record"
        );
    }

    /// With 50 callers held the DX station is the 51st: `1 + 10 * 51` codewords.
    #[test]
    fn dx_station_is_listed_beside_fifty_callers() {
        let mut h = Q65Callers::new();
        for k in 0..MAX_CALLERS {
            h.record(1500.0, &alloc::format!("K1ABC W{k}AB EN37"), 1);
        }
        let cw = contest_codewords("K1ABC", "VK3ABC", "QF22", &h);
        assert_eq!(cw.len(), MAX_CONTEST_CODEWORDS);
        assert_eq!(MAX_CONTEST_CODEWORDS, 511);
    }

    #[test]
    fn expiry_after_a_day() {
        let mut h = Q65Callers::new();
        h.record(1500.0, "K1ABC W9XYZ EN37", 0);
        h.record(1500.0, "K1ABC JA1ABC PM95", 3600);
        h.expire(CALLER_TTL_SEC + 1);
        assert_eq!(h.callers().len(), 1);
        assert_eq!(h.callers()[0].call, "JA1ABC");
    }

    #[test]
    fn list_shape() {
        let mut h = Q65Callers::new();
        h.record(1500.0, "K1ABC W9XYZ EN37", 0);
        h.record(1500.0, "K1ABC JA1ABC PM95", 0);
        // Two callers plus the DX station: 1 + 3 × 5 × 2.
        let cw = contest_codewords("K1ABC", "VK3ABC", "QF22", &h);
        assert_eq!(cw.len(), 31);
        assert_eq!(cw[0], [0; 63]);
        // The DX station already listed is not added twice.
        assert_eq!(contest_codewords("K1ABC", "W9XYZ", "EN37", &h).len(), 21);
        // Flag clear and set differ.
        assert_ne!(cw[1], cw[2]);
        // Info symbols as WSJT-X v3.2.0-rc1's own `q65_set_list2` builds them
        // for these inputs (its `genq65` linked against `libwsjt_fort.a`,
        // 2026-09-26; all 31 codewords matched): `W9XYZ EN37`, `R EN37`,
        // `RR73` (the explicit acknowledgement), and `JA1ABC 73` flagged.
        assert_eq!(cw[1][..13], [2, 27, 55, 35, 20, 6, 5, 9, 55, 0, 33, 22, 18]);
        assert_eq!(cw[3][..13], [2, 27, 55, 35, 20, 6, 5, 9, 55, 2, 33, 22, 18]);
        assert_eq!(cw[7][..13], [2, 27, 55, 35, 20, 6, 5, 9, 55, 1, 62, 36, 50]);
        assert_eq!(
            cw[20][..13],
            [2, 27, 55, 35, 21, 7, 32, 34, 61, 9, 62, 37, 3]
        );
    }

    /// Prints the list for the fixed inputs `q65_set_list2`'s own driver
    /// uses, one codeword per line, for a diff against upstream.
    #[test]
    #[ignore]
    fn print_contest_list() {
        let mut h = Q65Callers::new();
        h.record(1500.0, "K1ABC W9XYZ EN37", 0);
        h.record(1500.0, "K1ABC JA1ABC PM95", 0);
        for cw in contest_codewords("K1ABC", "VK3ABC", "QF22", &h) {
            let line: alloc::string::String = cw.iter().map(|v| alloc::format!("{v:3}")).collect();
            std::println!("CW{line}");
        }
    }
}
