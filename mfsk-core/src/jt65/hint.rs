//! JT65 "deep search" correlation fallback: a bank of candidate messages
//! matched against the symbol spectra when Reed-Solomon fails.
//!
//! Ported from WSJT-X v3.3.0-beta1 `lib/hint65.f90` (`extract.f90:182-196`
//! calls it when `nft == 0` and `ndepth & 32`). Not v3.2.0-rc1's: beta1
//! replaced the `CALL3.TXT` read by a caller-supplied list (`set_calls`,
//! `jt65_decode.f90:107`), skips a codeword whose `ref` is not positive, and
//! sized the bank at `MAXMSG = 2*MAXCALLS + 3 + MAXRPT`.
//!
//! The bank holds, for each entry of the known-call list, `MYCALL CALL GRID`
//! and `CQ CALL GRID`; for the DX station, those two plus `MYCALL HISCALL rpt`
//! for the 63 reports `-01..-30`, `R-01..R-30`, `RO`, `RRR`, `73`. Every
//! message is packed, RS-encoded, interleaved and Gray-coded, then scored
//! against the 63 symbol spectra as `sum(s3[sent]) / sum(s3[best])` with the
//! best tone swapped for the runner-up wherever the codeword used it. The
//! best message is accepted when its margin over the best *different*
//! message is large enough (`qual >= 2.0 - 0.1*aggressiveness`).
//!
//! Where this differs from upstream, on purpose:
//! - **Standard messages only.** The bank was built when `jt72` packed only the
//!   standard form, so the `0123456789ABC` entry (`i == -1`) is not in it, and a
//!   call that does not pack as standard (compound prefix/suffix) is skipped, where
//!   upstream would pack it as free text. `msg::jt72::pack_message` now packs those
//!   (0.14); bringing the bank level with upstream is open, not done here.
//! - **`nflip` is 1.** HF JT65 has one sync pattern (see
//!   [`super::averaging`]), so the CQ-skip for `nflip < 0` never applies.
//! - **`mrs`/`mrs2` are recomputed** as the best and second-best tone of each
//!   position in [`super::rx::Jt65Demod::raw_pwr`], without upstream's
//!   `chkhist` birdie blanking of one tone across all positions.
//! - **The grid of a list entry is cut to four characters**, as `hisgrid(1:4)`
//!   is for the DX entry; upstream passes the stored six.

use alloc::string::String;
use alloc::vec::Vec;

use super::rx::Jt65Demod;
use super::tx::encode_channel_symbols;
use crate::msg::jt72;

/// `hint65.f90`'s report table, `-01..-30`, `R-01..R-30`, `RO`, `RRR`, `73`.
const REPORTS: [&str; 63] = [
    "-01", "-02", "-03", "-04", "-05", "-06", "-07", "-08", "-09", "-10", "-11", "-12", "-13",
    "-14", "-15", "-16", "-17", "-18", "-19", "-20", "-21", "-22", "-23", "-24", "-25", "-26",
    "-27", "-28", "-29", "-30", "R-01", "R-02", "R-03", "R-04", "R-05", "R-06", "R-07", "R-08",
    "R-09", "R-10", "R-11", "R-12", "R-13", "R-14", "R-15", "R-16", "R-17", "R-18", "R-19", "R-20",
    "R-21", "R-22", "R-23", "R-24", "R-25", "R-26", "R-27", "R-28", "R-29", "R-30", "RO", "RRR",
    "73",
];

/// `jt65_max_calls` (`jt65_mod.f90:18`); `set_calls` keeps the first 10 000.
pub const MAX_CALLS: usize = 10_000;

/// What the correlation search is told: the station, the DX station and the
/// list of known calls (`CALL3.TXT` before beta1).
#[derive(Clone, Debug, Default)]
pub struct HintSpec {
    pub mycall: String,
    pub hiscall: String,
    pub hisgrid: String,
    /// `(call, grid)` pairs. Only the first [`MAX_CALLS`] are used.
    pub calls: Vec<(String, String)>,
    /// `naggressive`, 0..=10: the acceptance margin is `2.0 - 0.1 * this`.
    pub aggressiveness: u8,
}

/// The packed-and-encoded bank for one [`HintSpec`].
pub struct HintBank {
    /// Channel data symbols in transmission order (Gray-coded, interleaved).
    sym: Vec<[u8; 63]>,
    words: Vec<[u8; 12]>,
    text: Vec<String>,
    qmin: f32,
}

impl core::fmt::Debug for HintBank {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("HintBank")
            .field("n_entries", &self.text.len())
            .field("qmin", &self.qmin)
            .finish_non_exhaustive()
    }
}

fn push(bank: &mut HintBank, c1: &str, c2: &str, g: &str) {
    let Some(words) = jt72::pack_standard(c1, c2, g) else {
        return;
    };
    let tones = encode_channel_symbols(&words);
    let mut s = [0u8; 63];
    let mut k = 0;
    for (i, &t) in tones.iter().enumerate() {
        if super::sync_pattern::JT65_NPRC[i] == 0 {
            s[k] = t - 2;
            k += 1;
        }
    }
    bank.sym.push(s);
    bank.words.push(words);
    bank.text
        .push(alloc::format!("{c1} {c2} {g}").trim_end().into());
}

fn up(s: &str, n: usize) -> String {
    s.trim().chars().take(n).collect::<String>().to_uppercase()
}

impl HintBank {
    pub fn new(spec: &HintSpec) -> Self {
        let mut bank = HintBank {
            sym: Vec::new(),
            words: Vec::new(),
            text: Vec::new(),
            qmin: 2.0 - 0.1 * f32::from(spec.aggressiveness.min(10)),
        };
        let (my, his, hg) = (
            up(&spec.mycall, 6),
            up(&spec.hiscall, 6),
            up(&spec.hisgrid, 4),
        );
        if !my.is_empty() {
            if !(his.is_empty() && hg.is_empty()) {
                push(&mut bank, &my, &his, &hg);
                push(&mut bank, "CQ", &his, &hg);
                for r in REPORTS {
                    push(&mut bank, &my, &his, r);
                }
            }
            for (call, grid) in spec.calls.iter().take(MAX_CALLS) {
                let (c, g) = (up(call, 6), up(grid, 4));
                push(&mut bank, &my, &c, &g);
                push(&mut bank, "CQ", &c, &g);
            }
        } else {
            // `CQ` entries need no `MYCALL`; the rest of the bank does.
            if !(his.is_empty() && hg.is_empty()) {
                push(&mut bank, "CQ", &his, &hg);
            }
            for (call, grid) in spec.calls.iter().take(MAX_CALLS) {
                push(&mut bank, "CQ", &up(call, 6), &up(grid, 4));
            }
        }
        bank
    }

    pub fn len(&self) -> usize {
        self.sym.len()
    }

    pub fn is_empty(&self) -> bool {
        self.sym.is_empty()
    }

    /// `hint65.f90`'s scoring and `extract.f90:186`'s acceptance, on one
    /// demodulated frame. `nadd` is the number of spectra summed (1 for a
    /// single period). Returns the packed words and the quality.
    pub fn best(&self, d: &Jt65Demod, nadd: u32) -> Option<([u8; 12], f32)> {
        // s3(i,j) = raw_pwr[j-1][i-1]; mrs/mrs2 the best and second tone.
        let mut mrs = [0usize; 63];
        let mut mrs2 = [0usize; 63];
        let mut ref0 = 0f32;
        for j in 0..63 {
            let row = &d.raw_pwr[j];
            let (mut b, mut b2) = (0usize, usize::MAX);
            for t in 1..64 {
                if row[t] > row[b] {
                    b2 = b;
                    b = t;
                } else if b2 == usize::MAX || row[t] > row[b2] {
                    b2 = t;
                }
            }
            mrs[j] = b;
            mrs2[j] = b2;
            ref0 += row[b];
        }
        let (mut u1, mut u2) = (-99.0f32, -99.0f32);
        let (mut ipk, mut msg00) = (0usize, usize::MAX);
        for k in 0..self.sym.len() {
            let mut psum = 0f32;
            let mut r = ref0;
            for j in 0..63 {
                let i = usize::from(self.sym[k][j]);
                let v = d.raw_pwr[j][i];
                psum += v;
                if i == mrs[j] {
                    r = r - v + d.raw_pwr[j][mrs2[j]];
                }
            }
            if r <= 0.0 {
                continue;
            }
            let p = psum / r;
            let same = |a: usize, b: usize| b != usize::MAX && self.text[a] == self.text[b];
            if p > u1 {
                if !same(k, msg00) {
                    u2 = u1;
                }
                u1 = p;
                ipk = k;
                msg00 = k;
            }
            if !same(k, msg00) && p > u2 {
                u2 = p;
            }
        }
        if u1 <= -99.0 {
            return None;
        }
        let bias = if nadd >= 8 {
            (1.04 * u2).max(0.60)
        } else if nadd >= 4 {
            (1.08 * u2).max(0.45)
        } else {
            (1.12 * u2).max(0.35)
        };
        let qual = 100.0 * (u1 - bias);
        // hint65 itself sets `decoded` at `qual >= 1.0`; `qmin >= 1.0` here.
        (qual >= self.qmin.max(1.0)).then(|| (self.words[ipk], qual))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn demod_of(words: &[u8; 12], tone_boost: f32) -> Jt65Demod {
        // A noiseless frame: the sent tone at every position.
        let tones = encode_channel_symbols(words);
        let mut raw = [[0.05f32; 64]; 63];
        let mut k = 0;
        for (i, &t) in tones.iter().enumerate() {
            if super::super::sync_pattern::JT65_NPRC[i] == 0 {
                raw[k][usize::from(t - 2)] = tone_boost;
                k += 1;
            }
        }
        Jt65Demod {
            symbols: [0; 63],
            conf: [0.0; 63],
            second_symbols: [0; 63],
            rel: [0.0; 63],
            raw_pwr: raw,
            snr_db: -20.0,
        }
    }

    #[test]
    fn finds_a_listed_call_and_ignores_an_unlisted_one() {
        let spec = HintSpec {
            mycall: "JL1NIE".into(),
            calls: alloc::vec![
                ("K1ABC".into(), "FN42".into()),
                ("DL1XYZ".into(), "JO31".into())
            ],
            ..Default::default()
        };
        let bank = HintBank::new(&spec);
        assert_eq!(bank.len(), 4);
        let w = jt72::pack_standard("CQ", "DL1XYZ", "JO31").unwrap();
        assert_eq!(bank.best(&demod_of(&w, 1.0), 1).map(|(x, _)| x), Some(w));
        let other = jt72::pack_standard("CQ", "W9ZZZ", "EN52").unwrap();
        assert!(bank.best(&demod_of(&other, 1.0), 1).is_none());
    }

    #[test]
    fn dx_station_gets_reports() {
        let spec = HintSpec {
            mycall: "JL1NIE".into(),
            hiscall: "K1ABC".into(),
            hisgrid: "FN42".into(),
            ..Default::default()
        };
        let bank = HintBank::new(&spec);
        assert_eq!(bank.len(), 2 + REPORTS.len());
        let w = jt72::pack_standard("JL1NIE", "K1ABC", "R-12").unwrap();
        assert_eq!(bank.best(&demod_of(&w, 1.0), 1).map(|(x, _)| x), Some(w));
    }

    #[test]
    fn empty_bank_decodes_nothing() {
        let bank = HintBank::new(&HintSpec::default());
        assert!(bank.is_empty());
        let w = jt72::pack_standard("CQ", "K1ABC", "FN42").unwrap();
        assert!(bank.best(&demod_of(&w, 1.0), 1).is_none());
    }
}
