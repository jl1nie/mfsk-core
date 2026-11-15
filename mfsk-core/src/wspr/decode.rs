//! Top-level WSPR decode entry point.
//!
//! Given aligned audio, a candidate base frequency, and a target start
//! sample, runs demod → deinterleave → Fano → message unpack. No coarse
//! search here; a later module will wrap this with a (freq × time) scan.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
#[cfg(not(feature = "std"))]
use alloc::string::String;
use alloc::vec::Vec;

#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
// `f32::round` and friends live in `std`; `num_traits::Float` brings them without it
use num_traits::Float;
#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::engine::pipeline::{ScanRow, push_unique};
use crate::msg::WsprMessage;

use super::search::SearchParams;
#[cfg(any())]
use super::search::coarse_search;

/// One successful WSPR decode.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct WsprResult {
    /// Recovered message payload.
    pub message: WsprMessage,
    /// Base frequency (tone 0) used for demodulation.
    pub freq_hz: f32,
    /// Sample index at which symbol 0 started, in the *caller's* audio
    /// buffer. **Clamped to 0** when the signal actually started before
    /// the buffer (negative-dt case); use [`Self::dt_sec`] for the
    /// signed offset that matches wsprd's reporting.
    pub start_sample: usize,
    /// wsprd-equivalent `dt`: signal-start offset in seconds, relative
    /// to the WSPR nominal anchor (slot start + 1 s). Positive values
    /// = signal arrived late, negative = arrived early. Range that
    /// `decode_scan` can express: about `−4.5 s .. +∞` (`wsprd`'s coarse search reaches
    /// `shift = 128 · (k0 + 1) = −1152` baseband samples at `k0 = −10`, `wsprd.c:1250`, and the refine
    /// stages move it a further −128 and −32: −1312 samples, dt −4.499 s).
    pub dt_sec: f32,
    /// Linear drift the successful demodulation ran at, in Hz across
    /// the 110.6 s frame — wsprd's `drift1`.
    ///
    /// Load-bearing for subtraction, not decoration: wsprd hands this
    /// straight to `subtract_signal2` (`wsprd.c:1446`) so the replica
    /// it removes drifts the same way the signal did. Reconstructing a
    /// drifting signal as a stationary one leaves most of it behind.
    pub drift_hz: f32,
    /// 50-bit FEC info payload returned by Fano. Used by
    /// `decode_scan_subtract` to reconstruct the 162-channel-symbol
    /// stream and subtract the signal from the audio for SIC.
    pub info_bits: [u8; 50],
    /// wsprd-calibrated SNR estimate (dB, WSPR→2500 Hz reference
    /// bandwidth), from the coarse-search candidate that produced this
    /// decode — same `10·log10(smspec) − 26.3` formula wsprd itself
    /// reports next to a spot (`wsprd.c:1093`, see
    /// [`super::coarse_baseband::BasebandCandidate::snr_db`]). Set by
    /// `super::DecodeRequest`, which always goes through the coarse
    /// search; [`super::SniperRequest`] has no coarse candidate to
    /// derive it from and leaves this `0.0`.
    pub snr_db: f32,
    /// A Type 3 message's call, when the decoder's [`WsprCallsignTable`]
    /// held one for its hash at the time of the decode, as `wsprd`'s
    /// `hashtab` does (`wsprd_utils.c:389-392`). `None` for Types 1 and 2
    /// and for a hash not heard, which prints `<...>`. Text:
    /// [`WsprMessage::render`].
    pub hashed_call: Option<String>,
    /// `wsprd`'s `f1`: the centre of the four tones as an offset from 1500 Hz, the frequency
    /// `wsprd` prints as `dialfreq + (1500 + f1) / 1e6` (`wsprd.c:1522-1525`). The exact value
    /// behind [`Self::freq_hz`] (tone 0, which is this less one and a half tone spacings,
    /// plus 1500 Hz, rounded in single precision).
    pub f1_hz: f32,
    /// `wsprd`'s `decodes[].sync` (`wsprd.c:1531`): the candidate's sync after the
    /// refine cascade, the figure `minsync2` is compared with and `ALL_WSPR.TXT` prints
    /// (`%5.2f`) and `wspr_spots.txt` prints as `(int)(10 * sync)`. `0.0` from
    /// [`super::SniperRequest`], which reports none.
    pub sync: f32,
    /// The decode pass that found it, `wsprd`'s `ipass`: 0 and 1 are the early passes,
    /// 2 the final one. `ALL_WSPR.TXT` prints it plus one. A skipped pass 1 (`wsprd.c:1065`)
    /// leaves a final-pass decode at 2.
    pub pass: u8,
    /// `wsprd`'s `decodes[].blocksize` (`blocksize + 3 * bitmetric`): the rung of the
    /// final pass's ladder that decoded, 1 to 3 coherent symbols, or 4 for the
    /// bit-by-bit metric (this crate's `nblock` 0).
    pub blocksize: u8,
    /// `wsprd`'s `decodes[].jitter` (`ii`): how far, in 375 Hz baseband samples, the
    /// successful demodulation was from the refined alignment, `iifac * k` with `k` in
    /// -8..=8 (`wsprd.c:1387-1393`). The reported `dt` is the refined alignment's, not
    /// this one's.
    pub jitter: i32,
    /// `wsprd`'s `decodes[].decodetype` (`osd_decode`): 0 for a Fano decode, 1 for one
    /// OSD found and the callsign table confirmed.
    pub decode_type: u8,
    /// `wsprd`'s `decodes[].nhardmin` (`wsprd.c:1393,1427,1500`): hard-decision errors
    /// of the accepted codeword against the soft symbols. OSD's own count for an OSD
    /// decode. For a Fano decode `wsprd` counts it only while it subtracts
    /// (`wsprd.c:1497-1503`), so `-s` leaves 0.
    pub nhardmin: i32,
    /// `wsprd`'s `decodes[].cycles`, `fano.c`'s `*cycles`: one more than the loop
    /// iterations of the search that accepted the codeword. `ALL_WSPR.TXT` prints
    /// `cycles / 81`.
    pub cycles: u32,
    /// `wsprd`'s `decodes[].metric`, `fano.c`'s final path metric (`%5d` in
    /// `ALL_WSPR.TXT`). On an OSD decode it is the metric of the `fano` run on the OSD
    /// codeword (`wsprd.c:1433`).
    pub metric: i32,
}

/// Scan-dedup tolerances: the same real signal decoded twice by nearby
/// candidates, or again in a later SIC pass. Shared by the single-scan
/// and SIC drivers, which each declared their own copy until #419.
///
/// **Deliberate divergence from `wsprd` on the `Depth` path** (CLAUDE.md §2.1): `wsprd` calls a
/// decode a duplicate when its callsign text equals an earlier one's and the two `f1` are within
/// 4 Hz, with no time test (`wsprd.c:1506-1511`), and lists at most 50 decodes (`:1513`). This
/// is "same message, 5 Hz, one symbol". With `WsprExtras::wsprd_gates` the scan uses `wsprd`'s
/// rule (`wsprd_duplicate`).
const FREQ_DEDUP_HZ: f32 = 5.0;
/// One WSPR symbol at 12 kHz.
const TIME_DEDUP_SAMPLES: i64 = 8192;

impl ScanRow for WsprResult {
    type Message = WsprMessage;
    fn scan_message(&self) -> &Self::Message {
        &self.message
    }
    fn scan_freq_hz(&self) -> f32 {
        self.freq_hz
    }
    fn scan_start_sample(&self) -> i64 {
        self.start_sample as i64
    }
}

/// Callsigns confirmed by a **Fano** decode earlier in the same scan.
///
/// WSPR carries no CRC, so nothing in the codeword itself distinguishes
/// a real decode from a plausible-looking one. That is survivable for
/// Fano — a sequential decoder simply fails to converge on noise — but
/// not for OSD, which by construction synthesises a valid codeword for
/// *any* input and will happily emit a well-formed callsign built from
/// noise.
///
/// `wsprd.c:1396` guards it structurally rather than by threshold:
///
/// ```c
/// ihash = nhash(callsign, strlen(callsign), 146);
/// if (strncmp(hashtab + ihash*13, callsign, 13) == 0) { … not_decoded = 0; }
/// ```
///
/// — an OSD result is accepted only if that callsign is *already* in
/// the table, i.e. was confirmed by an earlier successful decode. OSD
/// can therefore re-find a known station under conditions Fano can't
/// reach, but can never invent a new one.
///
/// The threshold this replaces (`nhardmin ≤ 44`) cannot work, and that
/// is measurable rather than theoretical: on `150426_0918.wav` the one
/// genuine OSD decode (W3BI, -25 dB) lands at `nhardmin = 39` while
/// the phantoms land at 40, 40, 40, 41, 41, 41, 42, 42 — a separation
/// of one hard error. That gate let 8 phantoms through against 8 real
/// decodes; real `wsprd` reports 9 real and 0 phantom on the same file.
///
/// The table is `wsprd`'s: 32768 buckets addressed by `nhash(call, len, 146) & 0x7fff`,
/// held sparsely, a later store into a bucket replacing the earlier one (two calls that
/// hash alike evict each other, as in the C; this crate used to keep both). Type 1
/// additionally requires the grid to match, mirroring wsprd's `loctab` check. It reads and
/// writes `wsprd`'s `hashtable.txt` ([`Self::from_hashtable_txt`],
/// [`Self::to_hashtable_txt`]).
#[derive(Debug, Clone, Default)]
pub struct WsprCallsignTable {
    /// `wsprd`'s `hashtab` / `loctab`, kept sparsely: bucket index (`nhash(call, len, 146) &
    /// 0x7fff`) to what the bucket holds. A later store into a bucket replaces the earlier
    /// one, as in the C.
    slots: BTreeMap<u16, Slot>,
}

/// One occupied bucket: `hashtab[i]` and `loctab[i]`.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Slot {
    call: String,
    /// Empty for a bucket only a Type 2 call has been stored in (`wsprd_utils.c:370` stores
    /// the call and leaves the locator slot as it was).
    grid: String,
}

/// `WSPRD_CALLSIGN_SIZE` (`wsprd_utils.h`): a call is at most 12 characters.
const CALL_MAX: usize = 12;
/// `WSPRD_GRID4_SIZE`: a locator is at most 4 characters.
const GRID_MAX: usize = 4;
/// `WSPRD_HASH_COUNT`.
const HASH_COUNT: u32 = 32_768;

/// `nhash(call, strlen(call), 146)`, the bucket a call lives in.
fn bucket(call: &str) -> u16 {
    (crate::msg::wspr::hash15(call.as_bytes()) & 0x7fff) as u16
}

impl WsprCallsignTable {
    /// Empty table: every OSD result will be rejected.
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a Fano-confirmed decode, as `unpk_` stores it (`wsprd_utils.c:353-356,
    /// 364-369`): a Type 1 call and locator, a Type 2 call alone. Type 3 is never recorded
    /// — it carries a hash rather than a callsign, so it can confirm nothing.
    pub fn record(&mut self, msg: &crate::msg::WsprMessage) {
        match msg {
            crate::msg::WsprMessage::Type1 { callsign, grid, .. } => {
                if callsign.len() <= CALL_MAX && grid.len() <= GRID_MAX {
                    self.slots.insert(
                        bucket(callsign),
                        Slot {
                            call: callsign.clone(),
                            grid: grid.clone(),
                        },
                    );
                }
            }
            crate::msg::WsprMessage::Type2 { callsign, .. } => {
                if callsign.len() <= CALL_MAX {
                    self.slots.entry(bucket(callsign)).or_default().call = callsign.clone();
                }
            }
            crate::msg::WsprMessage::Type3 { .. } => {}
        }
    }

    /// The call this table holds for a Type 3 message's 15-bit hash (`wsprd_utils.c:390`:
    /// `hashtab[ihash]`). `None` when no such call has been decoded.
    pub fn resolve(&self, hash: u32) -> Option<&str> {
        let slot = self.slots.get(&u16::try_from(hash).ok()?)?;
        (!slot.call.is_empty()).then_some(slot.call.as_str())
    }

    /// Whether an OSD-derived message may be accepted: the gate at `wsprd.c:1462-1468`. The
    /// call's bucket must hold that call and, for a Type 1, that locator as well.
    pub fn accepts(&self, msg: &crate::msg::WsprMessage) -> bool {
        match msg {
            crate::msg::WsprMessage::Type1 { callsign, grid, .. } => self
                .slots
                .get(&bucket(callsign))
                .is_some_and(|s| s.call == *callsign && s.grid == *grid),
            crate::msg::WsprMessage::Type2 { callsign, .. } => self
                .slots
                .get(&bucket(callsign))
                .is_some_and(|s| s.call == *callsign),
            // Type 3 is a hashed callsign with no table to resolve it
            // against — wsprd's own OSD path requires `itype` 1 or 2.
            crate::msg::WsprMessage::Type3 { .. } => false,
        }
    }

    /// Number of occupied buckets.
    pub fn len(&self) -> usize {
        self.slots.len()
    }

    /// Whether no call has been stored.
    pub fn is_empty(&self) -> bool {
        self.slots.is_empty()
    }

    /// The occupied buckets in index order, `(index, call, grid)`: the order and content
    /// of `wsprd`'s `hashtable.txt` (`wsprd.c:1637-1645`). `grid` is empty for a bucket
    /// only a Type 2 call was stored in.
    pub fn iter(&self) -> impl Iterator<Item = (u16, &str, &str)> {
        self.slots
            .iter()
            .map(|(&i, s)| (i, s.call.as_str(), s.grid.as_str()))
    }

    /// Load one `hashtable.txt` line, as `wsprd_load_hash_line` does
    /// (`wsprd_utils.c:97-118`, called at `wsprd.c:1055`): an index below 32768, a call of
    /// at most 12 characters and an optional locator of at most 4. A line without a locator
    /// keeps the bucket's existing one. Returns whether the line was accepted.
    pub fn load_hashtable_line(&mut self, line: &str) -> bool {
        let b = line.as_bytes();
        let is_space = |c: u8| matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r');
        let mut p = 0;
        // strtol(base 10): leading space, optional sign, digits.
        while p < b.len() && is_space(b[p]) {
            p += 1;
        }
        let mut neg = false;
        if p < b.len() && (b[p] == b'+' || b[p] == b'-') {
            neg = b[p] == b'-';
            p += 1;
        }
        let digits = p;
        let mut index: i64 = 0;
        while p < b.len() && b[p].is_ascii_digit() {
            index = index
                .saturating_mul(10)
                .saturating_add(i64::from(b[p] - b'0'));
            p += 1;
        }
        if p == digits {
            return false;
        }
        if neg {
            index = -index;
        }
        if !(0..i64::from(HASH_COUNT)).contains(&index) || p >= b.len() || !is_space(b[p]) {
            return false;
        }
        let mut token = |required: bool, max: usize| -> Option<&str> {
            while p < b.len() && is_space(b[p]) {
                p += 1;
            }
            if p >= b.len() {
                return if required { None } else { Some("") };
            }
            let start = p;
            while p < b.len() && !is_space(b[p]) {
                p += 1;
            }
            // `len >= dest_size` fails: room for the terminator.
            (p - start <= max).then(|| &line[start..p])
        };
        let Some(call) = token(true, CALL_MAX) else {
            return false;
        };
        let Some(grid) = token(false, GRID_MAX) else {
            return false;
        };
        let slot = self.slots.entry(index as u16).or_default();
        slot.call = call.into();
        if !grid.is_empty() {
            slot.grid = grid.into();
        }
        true
    }

    /// A table from the text of a `hashtable.txt`; lines `wsprd` would not load are skipped.
    pub fn from_hashtable_txt(text: &str) -> Self {
        let mut t = Self::new();
        for line in text.lines() {
            t.load_hashtable_line(line);
        }
        t
    }

    /// The text `wsprd` writes to `hashtable.txt` at the end of a run (`wsprd.c:1637-1645`,
    /// `"%5d %s %s\n"` per occupied bucket in index order). Empty for an empty table.
    pub fn to_hashtable_txt(&self) -> String {
        use core::fmt::Write;
        let mut out = String::new();
        for (i, call, grid) in self.iter() {
            let _ = writeln!(out, "{i:5} {call} {grid}");
        }
        out
    }
}

#[cfg(test)]
mod callsign_table_tests {
    use super::WsprCallsignTable;
    use crate::msg::WsprMessage;

    fn t1(callsign: &str, grid: &str) -> WsprMessage {
        WsprMessage::Type1 {
            callsign: callsign.into(),
            grid: grid.into(),
            power_dbm: 30,
        }
    }

    /// The OSD gate (`wsprd.c:1396`) admits a result only for a
    /// callsign an earlier Fano decode confirmed. Nothing else may
    /// open it — that is the entire reason OSD is safe to keep.
    #[test]
    fn accepts_only_recorded_callsigns() {
        let mut table = WsprCallsignTable::new();
        assert!(!table.accepts(&t1("W3BI", "FN20")), "empty table accepted");

        table.record(&t1("W3BI", "FN20"));
        assert!(table.accepts(&t1("W3BI", "FN20")));
        assert!(
            !table.accepts(&t1("ZZ9ZZZ", "AA00")),
            "unrecorded callsign accepted"
        );
    }

    /// Type 1 carries a grid, so wsprd requires both to match; Type 2
    /// carries a prefix/suffix instead and matches on callsign alone.
    #[test]
    fn type1_matches_on_grid_too_type2_does_not() {
        let mut table = WsprCallsignTable::new();
        table.record(&t1("W3BI", "FN20"));
        assert!(
            !table.accepts(&t1("W3BI", "AA00")),
            "Type 1 accepted a mismatched grid"
        );
        assert!(table.accepts(&WsprMessage::Type2 {
            callsign: "W3BI".into(),
            power_dbm: 30,
        }));
    }

    /// Type 3 is a hashed callsign with nothing to resolve it against,
    /// so wsprd's OSD path refuses it outright (`itype` must be 1 or 2).
    #[test]
    fn hashed_callsigns_are_never_accepted() {
        let mut table = WsprCallsignTable::new();
        table.record(&t1("W3BI", "FN20"));
        assert!(!table.accepts(&WsprMessage::Type3 {
            callsign_hash: 0x05c31,
            grid6: "FN20aa".into(),
            power_dbm: 30,
        }));
    }

    /// `K1ABC` and `RYEEUQHAFKK` share `nhash(call, len, 146) & 0x7fff` = 6521. `wsprd`'s
    /// table is those buckets, so the later store evicts the earlier call and the table
    /// stops accepting it (`wsprd_utils.c:353-356`); keeping both, as this crate did, is
    /// what the eviction test guards.
    #[test]
    fn a_colliding_call_evicts_the_earlier_one() {
        let mut table = WsprCallsignTable::new();
        table.record(&t1("K1ABC", "FN42"));
        assert!(table.accepts(&t1("K1ABC", "FN42")));
        table.record(&WsprMessage::Type1 {
            callsign: "RYEEUQHAFKK".into(),
            grid: "AA00".into(),
            power_dbm: 30,
        });
        assert_eq!(table.len(), 1, "one bucket");
        assert_eq!(table.resolve(6521), Some("RYEEUQHAFKK"));
        assert!(
            !table.accepts(&t1("K1ABC", "FN42")),
            "evicted call accepted"
        );
    }

    /// A Type 2 stores the call and leaves the bucket's locator as it was
    /// (`wsprd_utils.c:370`); into an empty bucket the locator is empty.
    #[test]
    fn type2_keeps_the_buckets_locator() {
        let mut table = WsprCallsignTable::new();
        table.record(&t1("K1ABC", "FN42"));
        table.record(&WsprMessage::Type2 {
            callsign: "RYEEUQHAFKK".into(),
            power_dbm: 30,
        });
        let rows: Vec<_> = table.iter().collect();
        assert_eq!(rows, [(6521, "RYEEUQHAFKK", "FN42")]);
        assert_eq!(table.to_hashtable_txt(), " 6521 RYEEUQHAFKK FN42\n");

        let mut fresh = WsprCallsignTable::new();
        fresh.record(&WsprMessage::Type2 {
            callsign: "PJ4/K1ABC".into(),
            power_dbm: 30,
        });
        let (_, call, grid) = fresh.iter().next().unwrap();
        assert_eq!((call, grid), ("PJ4/K1ABC", ""));
    }
}

/// `minsync1`, `wsprd.c:820`: the refined sync a candidate needs for the fine stages (and, in
/// `wsprd`, for its refined values to replace the coarse ones).
const MINSYNC1: f32 = 0.10;

/// wsprd's refine cascade (`wsprd.c:1221-1277`), in full. Each stage is
/// `sync_and_demodulate` in mode 0 (search lag, freq fixed) or mode 1
/// (search freq, lag fixed), keeping whichever cell scores highest on
/// the sync metric:
///
///   1. coarse lag   ±128 baseband samples, step 64
///   2. coarse freq  ±2 × 0.25 Hz
///   3. drift refine drift ± 0.5 (passes 0 and 1 only)
///   4. fine lag     ±32 baseband samples, step 16
///   5. fine freq    ±2 × 0.05 Hz
///
/// Stages 3-5 were missing here, and stage 2 ran at ±1.0 Hz in 0.5 Hz
/// steps rather than ±0.5 Hz in 0.25 Hz steps. That combination is why
/// handing this function the coarse stage's drift estimate used to
/// *hurt*: without stage 3 there is nothing to correct a drift the
/// ±4 Hz coarse grid got wrong, and without stages 4-5 the alignment
/// stays a coarse-grid cell away from the true one.
///
/// Extracted out of [`decode_at_baseband_inner`] so
/// [`debug_refined_sync`] can call it without duplicating the cascade
/// — the two need to agree on the exact value `minsync2` is compared
/// against, or a diagnostic built against a near-copy is answering a
/// slightly different question than the one being asked.
#[allow(clippy::type_complexity)]
fn refine_cascade(
    idat: &[f32],
    qdat: &[f32],
    f0_baseband_init: f32,
    lag_baseband_init: i32,
    drift_hz: f32,
    refine_drift: bool,
    // The champion buffer, caller-owned.
    //
    // Returned by value until 2026-08-24, which put a third 10 368 B
    // `IsQs` on the stack — the caller's return slot, on top of the two
    // this function ping-pongs. Writing into the caller's own buffer
    // makes the champion and the return the same storage. Fully
    // overwritten below before it is read, so the caller need not
    // initialise it to anything meaningful.
    out: &mut super::demod::IsQs,
) -> (f32, i32, f32, f32) {
    #[cfg(feature = "instrument")]
    super::instrument::bump(&super::instrument::CANDIDATES);

    let mut best_freq = f0_baseband_init;
    let mut best_lag = lag_baseband_init;
    let mut best_drift = drift_hz;

    // Ping-pong pair: `best_isqs` holds the current champion, `scratch`
    // is overwritten by every trial evaluation and swapped in when it
    // wins — no by-value `IsQs` return, no per-call-site local. Exactly
    // two 10 368 B buffers live for this whole 13-eval cascade, instead
    // of one fresh copy per call site (five distinct `let` bindings in
    // the old closure-based version) — see `tone_amplitudes_into`'s
    // doc comment for why that shape is what the embedded stack audit
    // (`docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`, "Stack")
    // found dominating `refine_cascade`'s 93-103 KB peak.
    let mut scratch = super::demod::IsQs::zeroed();
    super::demod::tone_amplitudes_into(idat, qdat, best_freq, best_lag, best_drift, out);
    let mut best_sync = super::demod::sync_score_isqs(out);

    // Trial `(f, lag, drift)` into `scratch`; if it beats the current
    // champion, adopt its parameters and swap `scratch` in as the new
    // `best_isqs` (pointer/tag swap, no 10 368 B copy).
    macro_rules! try_cell {
        ($f:expr, $lag:expr, $drift:expr) => {{
            super::demod::tone_amplitudes_into(idat, qdat, $f, $lag, $drift, &mut scratch);
            let sync = super::demod::sync_score_isqs(&scratch);
            if sync > best_sync {
                best_sync = sync;
                best_freq = $f;
                best_lag = $lag;
                best_drift = $drift;
                core::mem::swap(out, &mut scratch);
            }
        }};
    }

    // 1. Coarse lag.
    for &dlag in &[-128i32, -64, 64, 128] {
        try_cell!(best_freq, lag_baseband_init + dlag, best_drift);
    }

    // 2. Coarse freq — wsprd's `fstep = 0.25, ifmin = -2, ifmax = 2`.
    for i in -2i32..=2 {
        if i == 0 {
            continue;
        }
        try_cell!(f0_baseband_init + i as f32 * 0.25, best_lag, best_drift);
    }

    // 3. Drift refine — `wsprd.c:1236-1255`, passes 0 and 1 only. wsprd
    // tries `drift ± 0.5` and keeps the better, checking `+` first so a
    // tie goes to `+` exactly as the C's `else if` does.
    if refine_drift {
        for &dd in &[0.5f32, -0.5] {
            let before = best_sync;
            try_cell!(best_freq, best_lag, drift_hz + dd);
            if best_sync > before {
                break;
            }
        }
    }

    // Stages 4-5 run only above wsprd's `minsync1` (`wsprd.c:820,1325`);
    // below it the candidate is noise and the extra evals are wasted.
    //
    // `wsprd` writes the refined `freq` / `shift` / `drift` / `sync` back to the candidate only
    // inside this branch (`wsprd.c:1325-1343`), so a candidate that stays at or below `minsync1`
    // keeps its *coarse* values and is judged by `minsync2` on the coarse sync. The scan
    // (`refine_candidate`) does the same. The point decode (`decode_at_baseband_inner`) has no
    // coarse sync to fall back on and still drops such a candidate on its refined sync. The pass-2
    // ranking (`rank_pass2_candidates`, the embedded `wspr-pass2-topn` path) does hold the coarse
    // candidate and could do the same; it is left because it changes the CoreS3's runtime path,
    // which needs a flash and a captured log.
    if best_sync > MINSYNC1 {
        #[cfg(feature = "instrument")]
        super::instrument::bump(&super::instrument::MINSYNC1_PASS);
        // 4. Fine lag — ±32, step 16.
        let centre_lag = best_lag;
        for &dlag in &[-32i32, -16, 16, 32] {
            try_cell!(best_freq, centre_lag + dlag, best_drift);
        }
        // 5. Fine freq — `fstep = 0.05, ifmin = -2, ifmax = 2`.
        let centre_freq = best_freq;
        for i in -2i32..=2 {
            if i == 0 {
                continue;
            }
            try_cell!(centre_freq + i as f32 * 0.05, best_lag, best_drift);
        }
    }

    (best_freq, best_lag, best_drift, best_sync)
}

/// Point decode at a known alignment on a pre-decimated 375 Hz
/// baseband: wsprd's refine cascade, the `minsync2` gate, then Fano and
/// the OSD ladder. Public through [`super::SniperRequest`], which
/// documents each argument as a builder method.
///
/// `confirmed` is the set of callsigns an earlier Fano decode already
/// established (see [`WsprCallsignTable`]). `None` rejects every OSD
/// result outright, which is the only safe default for a caller with no
/// scan-level context: OSD synthesises a valid codeword for any input,
/// so ungated it is a phantom generator, not a sensitivity feature.
///
/// `refine_drift` mirrors `wsprd.c:1236`'s `if (ipass < 2)`: the first
/// two passes try `drift ± 0.5` around the coarse estimate and keep
/// whichever scores better on sync, while the final pass does not —
/// it runs with `maxdrift = 0` throughout, trading drift tracking for
/// a lower-variance frequency estimate on the weak signals that are
/// all that remain by then.
#[allow(clippy::too_many_arguments)]
pub(super) fn decode_at_baseband_inner(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    start_sample: usize,
    freq_hz: f32,
    drift_hz: f32,
    nblocks: &[usize],
    confirmed: Option<&WsprCallsignTable>,
    refine_drift: bool,
    ladder: Ladder,
) -> Option<WsprResult> {
    // `freq_hz` follows our tone-0 convention (matches `engine::tx::synthesize`
    // and `coarse_search.freq_hz`); wsprd's `noncoherent_sequence_detection`
    // takes the signal CENTER, so we add 1.5·tone_spacing here and sweep
    // around that point.
    let f0_center_init = freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
    let f0_baseband_init = f0_center_init - super::baseband::CENTER_HZ;
    let lag_baseband_init = start_sample as i32 / 32;

    let mut best_isqs = super::demod::IsQs::zeroed();
    let (best_freq, best_lag, best_drift, best_sync) = refine_cascade(
        idat,
        qdat,
        f0_baseband_init,
        lag_baseband_init,
        drift_hz,
        refine_drift,
        &mut best_isqs,
    );

    const MINSYNC2_EARLY: f32 = 0.12;
    const MINSYNC2_FINAL: f32 = 0.10;
    let minsync2 = if refine_drift {
        MINSYNC2_EARLY
    } else {
        MINSYNC2_FINAL
    };
    if best_sync <= minsync2 {
        #[cfg(feature = "instrument")]
        super::instrument::bump(&super::instrument::MINSYNC2_REJECTED);
        return None;
    }

    decode_from_refined(
        idat,
        qdat,
        sample_rate,
        best_freq,
        best_lag,
        best_drift,
        Some(&best_isqs),
        nblocks,
        confirmed,
        None,
        ladder,
    )
}

/// Mode 2 of the refine→demod cascade: bit metrics + Fano/OSD at an
/// *already*-refined alignment. Split out of
/// [`decode_at_baseband_inner`] so callers that need to
/// **rank candidates by refined sync before deciding which ones are
/// worth the ladder** — [`decode_pass2_top_n`]'s whole reason for
/// existing — can call [`refine_cascade`] once per candidate, sort,
/// and only pay this function's cost for the survivors, instead of
/// this always running unconditionally per candidate the way it did
/// when the two were one function.
///
/// `nblocks`/`confirmed` and the return value are unchanged from
/// [`decode_at_baseband_inner`]'s own contract; this is
/// a pure extraction, not a behaviour change — see that function's
/// remaining body for the minsync2 gate this is called after.
///
/// `budget`: checked before every (nblock, idt) position in the DT
/// peak-up ladder below; `Some(check)` returning `false` aborts the
/// ladder early, equivalent to this candidate simply failing to
/// decode rather than an error. Exists for the ladder's *failing*
/// path specifically — a candidate that never converges pays the
/// full sweep (up to 4 × 17 = 68 positions; ~48 s on CoreS3 for one
/// such candidate, see
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Dual-core,
/// take two") since neither loop's wsprd-faithful exit condition
/// (`while (... && not_decoded)`) has any notion of giving up early.
/// `None` runs the exhaustive, wsprd-faithful sweep unconditionally —
/// every existing caller of this function passes `None`; the check is
/// additive, not a default behaviour change.
///
/// `&(dyn Fn() -> bool + Sync)`, not `FnMut`: callers build the check
/// from a fixed captured deadline (embedded: `esp_timer_get_time() <
/// deadline`) rather than mutable state, so it can be shared as-is
/// across [`decode_pass2_top_n`]'s `par_iter()` batch instead of
/// needing one exclusive borrow per candidate.
#[allow(clippy::too_many_arguments)]
fn decode_from_refined(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    best_freq: f32,
    best_lag: i32,
    best_drift: f32,
    best_isqs: Option<&super::demod::IsQs>,
    nblocks: &[usize],
    confirmed: Option<&WsprCallsignTable>,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
    ladder: Ladder,
) -> Option<WsprResult> {
    match ladder_decode(
        idat,
        qdat,
        sample_rate,
        best_freq,
        best_lag,
        best_drift,
        best_isqs,
        nblocks,
        confirmed,
        budget,
        ladder,
    ) {
        LadderOutcome::Decoded(r, _) => Some(*r),
        LadderOutcome::Failed | LadderOutcome::NoPrint => None,
    }
}

/// How one candidate's decode ladder ended.
#[derive(Debug)]
enum LadderOutcome {
    /// No rung and no DT position produced an accepted codeword.
    Failed,
    /// A codeword was accepted that `unpk_` will not print (`noprint`): a reserved or
    /// implausible message. `wsprd` still stops the ladder there, counts it in
    /// `ndecodes_pass` and skips the subtraction (`wsprd.c:1481-1503`).
    NoPrint,
    /// The decode and its hard decisions, `symbols[i] > 127` of `wsprd` in deinterleaved
    /// order, which `count_hard_errors` (`wsprd.c:691`) compares with the replica.
    Decoded(Box<WsprResult>, [u8; 162]),
}

/// [`decode_from_refined`] with the ladder's outcome kept apart: `wsprd`'s
/// `while (ib <= nblocksize && not_decoded)` / `while (not_decoded && idt <= ...)` loops
/// stop on a decode whether or not it prints.
#[allow(clippy::too_many_arguments)]
fn ladder_decode(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    best_freq: f32,
    best_lag: i32,
    best_drift: f32,
    best_isqs: Option<&super::demod::IsQs>,
    nblocks: &[usize],
    confirmed: Option<&WsprCallsignTable>,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
    ladder: Ladder,
) -> LadderOutcome {
    use crate::engine::{FecCodec, MessageCodec};
    let codec = crate::fec::ConvFano;
    // Pooled across the whole `nblocks` sweep below (up to 3 values in
    // pass 2) instead of `fano_decode` allocating a fresh `Vec<Node>`
    // scratch per call — see `fano::fano_decode_with_scratch`'s doc
    // comment for why reuse is safe here.
    let mut fano_scratch = crate::fec::conv::fano::FanoScratch::new();

    // Mode 2: bit metrics + Fano. wsprd does *not* stop at the refined
    // alignment — `wsprd.c:1329-1423` wraps the demod-and-decode step in
    // a **DT peak-up loop**, retrying at `shift1 ± 8k` baseband samples
    // for `k = 0 .. 8` (`iifac = 8`, `idt <= 128 / iifac`), in the order
    // 0, +8, −8, +16, −16, … ±64. Seventeen positions, and it is the
    // inner loop: every `ib` rung gets the full sweep.
    //
    // This is not a refinement of the refinement — the refine cascade
    // maximises *sync*, and the alignment that maximises sync is not
    // always the one Fano converges from. On the WSJT-X golden, wsprd
    // wins `G8VDQ IO91 37` at `shift1 + 16`, the third position it
    // tries, having already scored position 0 higher on sync.
    //
    // Both loops exit on the first success, exactly as the C's
    // `while (ib <= nblocksize && not_decoded)` /
    // `while (not_decoded && idt <= ...)` do, so the cost below is
    // what a *failing* candidate pays, not a succeeding one.
    /// `iifac`, `wsprd.c:822` — step size in the final DT peak-up.
    const IIFAC: i32 = 8;
    /// `idt <= 128 / iifac`, `wsprd.c:1322`.
    const N_JITTER: i32 = 128 / IIFAC;
    // `nblock == 0` is this crate's spelling of wsprd's fourth ladder
    // rung (`ib == 4` → `blocksize = 1, bitmetric = 1`) — see
    // `demod::nblock1_bit_metrics_opt`.
    // One buffer for the whole ladder, not one per jitter position.
    //
    // `tone_amplitudes`'s by-value form put two 10 368 B `IsQs` on this
    // frame at once — the wrapper's own local and the caller's return
    // slot — inside a loop, where `opt-level=1` (which the embedded
    // crates pin for the Xtensa LLVM regression) has no obligation to
    // coalesce them. `tone_amplitudes_into` fully overwrites its `out`,
    // so hoisting one buffer here is behaviour-identical and costs a
    // single zeroing instead of one per position. Same reasoning that
    // took `refine_cascade` from a 93-103 KB peak to 61.5 KB; see
    // `tone_amplitudes_into`'s doc comment.
    let mut jitter_isqs = super::demod::IsQs::zeroed();
    'rungs: for &nblock in nblocks {
        'jitter: for idt in 0..=if ladder.jitter { N_JITTER } else { 0 } {
            if let Some(check) = budget
                && !check()
            {
                #[cfg(feature = "instrument")]
                super::instrument::bump(&super::instrument::LADDER_BUDGET_ABORTED);
                break 'rungs;
            }
            // wsprd's `ii = (idt+1)/2; if (idt%2 == 1) ii = -ii; ii *= iifac;`
            let mut ii = (idt + 1) / 2;
            if idt % 2 == 1 {
                ii = -ii;
            }
            let lag = best_lag + IIFAC * ii;
            // Position 0 is the refined alignment, whose `IsQs` the
            // cascade already built.
            let isqs = if let (0, Some(champion)) = (idt, best_isqs) {
                champion
            } else {
                super::demod::tone_amplitudes_into(
                    idat,
                    qdat,
                    best_freq,
                    lag,
                    best_drift,
                    &mut jitter_isqs,
                );
                &jitter_isqs
            };
            #[cfg(all(feature = "std", feature = "instrument"))]
            let t_bm = crate::clock::Tick::now();
            let bm = if nblock == 0 {
                super::demod::nblock1_bit_metrics_opt(isqs, true)
            } else {
                super::demod::nblock_bit_metrics(isqs, nblock)
            };
            #[cfg(all(feature = "std", feature = "instrument"))]
            super::instrument::add_us(
                &super::instrument::BIT_METRICS_US,
                t_bm.elapsed_micros() as u32,
            );
            // wsprd's `if (rms > minrms)` plausibility gate — skip the Fano attempt
            // outright when the normalised soft symbols are dominated by saturating
            // outliers. `wsprd` takes the RMS of the quantised `symbols[i] - 128` before it
            // deinterleaves (`wsprd.c:1400-1408`): with `wsprd_gates` so does this. **The
            // default path keeps this crate's gate on the float soft symbols** (a
            // deliberate divergence, CLAUDE.md §2.1; the two differ only when the RMS is
            // within a quantisation step of the threshold).
            let rms = if ladder.wsprd_gates {
                crate::fec::conv::fano::wsprd_quantised_rms(&bm)
            } else {
                let mut l = bm;
                deinterleave_llrs(&mut l);
                crate::fec::conv::fano::wsprd_soft_symbol_rms(&l)
            };
            let mut llrs = bm;
            deinterleave_llrs(&mut llrs);
            if rms <= crate::fec::conv::fano::WSPRD_MIN_RMS {
                continue;
            }
            // Fano first; if it fails to converge, fall back to OSD-1
            // (Ordered-Statistics Decoding, port of `osdwspr.f90`). OSD
            // can recover signals at -27 dB SNR (e.g. W3BI on the WSJT-X
            // golden) where Fano alone hits the convergence threshold.
            #[cfg(feature = "instrument")]
            super::instrument::bump(&super::instrument::FANO_ATTEMPTS);
            // Cycle-budget cap (issue #260): every real decode in the
            // WSJT-X golden converges inside 20 % of the uncapped
            // budget (measured worst case — G8VDQ, the -23 dB decode
            // pass 2 exists for — is 162 075 cycles = 2 001/bit,
            // 20.01 % of the 10 000/bit default), while a failing
            // candidate burns essentially the whole budget before
            // giving up (mean 99 %, `docs/notes/
            // WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Fano
            // convergence budget" section). Unlike pass 2's own
            // minsync2 + top-N + deadline-budget triage, passes 0 and
            // 1 had no cap at all: every minsync2 (0.12) survivor paid
            // the full, uncapped Fano+OSD ladder regardless of how
            // many candidates a busier band produced.
            //
            // **Host is wsprd's own number**: `wsprd.c:799`,
            // `unsigned int maxcycles=10000; //Decoder timeout limit`
            // (overridable there by a CLI flag, same as here by the
            // feature below). It is also what this path effectively ran
            // at before issue #260 wired the cap up, since
            // `ConvFano::DEFAULT_MAX_CYCLES` is the same 10 000.
            //
            // Raising it above the reference buys floor recall and then
            // starts manufacturing decodes. Swept over the AWGN corpus
            // (`wspr_awgn_snr_sweep`, 100 trials/cell at the floor,
            // 500 total — every file holds exactly one transmitted
            // message, so any other decode is a phantom by
            // construction):
            //
            // ```text
            //   cycles/bit   -32 dB  -31 dB  -30 dB   phantoms/500
            //        5 000     19 %    67 %    96 %          0
            //       10 000     22 %    70 %    96 %          0   <- wsprd, host
            //       20 000     24 %    73 %    96 %          0
            //       50 000     27 %    76 %    98 %          2
            //      100 000     31 %    75 %    98 %          7
            //      500 000     23 %    61 %    91 %   golden FAILS (4)
            // ```
            //
            // The recall column alone reads as "higher is better up to
            // ~200 000"; the phantom column says the usable range ends
            // an order of magnitude earlier, between 20 000 and 50 000.
            // Given long enough, Fano finds codewords that satisfy the
            // CRC but are not the transmitted message — and in a
            // multi-pass SIC decoder those do compounding damage, since
            // they enter the carried callsign table and get subtracted
            // from the residual. At 500 000 recall ends up *below* the
            // capped value for exactly that reason. 20 000 measured
            // clean too, but there is no principled reason to sit
            // between the reference decoder's timeout and the onset of
            // false decodes.
            //
            // Capping *below* the reference costs real recall, and
            // costs it outright: the only fallback below is
            // `osd_decode_packed`, gated on `confirmed` being `Some` —
            // a callsign table built by an **earlier** Fano decode — so
            // a lone weak signal that Fano gives up on has no rescue
            // path at all. Embedded accepts that (5 000 via
            // `wspr-fano-cap-fast`) because it is what keeps a CoreS3
            // `decode_scan` inside the 120 s slot (-12.2 % TOTAL, see
            // `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`); it
            // stays phantom-free, it just hears less. Same split shape
            // as `wspr-pass2-topn`.
            #[cfg(all(feature = "std", feature = "instrument"))]
            let t_fano = crate::clock::Tick::now();
            let fano_res = codec.decode_soft_detailed(
                &llrs,
                ladder.max_cycles_per_bit,
                ladder.bias,
                &mut fano_scratch,
            );
            #[cfg(all(feature = "std", feature = "instrument"))]
            super::instrument::add_us(&super::instrument::FANO_US, t_fano.elapsed_micros() as u32);
            // The accepted codeword: its 50 payload bits, what `nhardmin` will be, `fano`'s
            // cycles and metric, and whether OSD found it.
            let (info_bits, nhardmin, cycles, metric, decode_type) = if let Some(f) = fano_res {
                #[cfg(feature = "instrument")]
                super::instrument::bump(&super::instrument::FANO_OK);
                let mut info = [0u8; 50];
                info.copy_from_slice(&f.info);
                // `wsprd.c:1497-1503`: a Fano decode's hard-error count is taken only
                // while subtracting, against the re-encoded printed message.
                let nh = if ladder.subtraction {
                    f.hard_errors as i32
                } else {
                    0
                };
                (info, nh, f.cycles, f.metric, 0u8)
            } else {
                match (ladder.osd, confirmed) {
                    // This crate's own OSD (the `Depth` mapping): the final pass's, gated
                    // on the callsign table, see `WsprCallsignTable`.
                    (OsdSel::Legacy, Some(table)) => {
                        #[cfg(feature = "instrument")]
                        super::instrument::bump(&super::instrument::OSD_ATTEMPTS);
                        // `osd_decode_packed`: bit-packed GF(2) rows, verified
                        // bit-identical to `osd_decode` on 1626 real LLR
                        // vectors harvested from an AWGN sweep
                        // (`wspr_osd_packed_matches_unpacked`,
                        // `tests/wspr_sweep.rs`) — 8.5x faster on host, and
                        // the ~12.2 KB `osd_decode` stack frame this crate's
                        // embedded stack audit found (two-thirds of it one
                        // unpacked `[[u8;162];50]` matrix) drops to ~5 KB.
                        #[cfg(all(feature = "std", feature = "instrument"))]
                        let t_osd = crate::clock::Tick::now();
                        let osd_res = super::osd::osd_decode_packed(&llrs);
                        #[cfg(all(feature = "std", feature = "instrument"))]
                        super::instrument::add_us(
                            &super::instrument::OSD_US,
                            t_osd.elapsed_micros() as u32,
                        );
                        let Some((info, nhardmin)) = osd_res else {
                            continue;
                        };
                        // wsprd's structural gate (`wsprd.c:1462-1468`): an OSD result
                        // is accepted only for a callsign an earlier decode already
                        // confirmed. Replaces an `nhardmin <= 44` threshold that
                        // measurement showed cannot separate the two populations.
                        let Some(msg) = crate::msg::Wspr50Message
                            .unpack(&info, &crate::engine::DecodeContext::default())
                        else {
                            continue;
                        };
                        if !table.accepts(&msg) {
                            continue;
                        }
                        #[cfg(feature = "instrument")]
                        super::instrument::bump(&super::instrument::OSD_OK);
                        // What `fano` says of the codeword, for the result's fields.
                        let mut cw = [0u8; 162];
                        codec.encode(&info, &mut cw);
                        let f = codec.decode_codeword(
                            &cw,
                            ladder.max_cycles_per_bit,
                            ladder.bias,
                            &mut fano_scratch,
                        );
                        (info, nhardmin as i32, f.cycles, f.metric, 1u8)
                    }
                    // `wsprd -o n` (`wsprd.c:1422-1469`): `osdwspr` on the quantised soft
                    // symbols, `fano` on its codeword, then the table gate.
                    (OsdSel::Wsprd(ndepth), Some(table)) => {
                        #[cfg(feature = "instrument")]
                        super::instrument::bump(&super::instrument::OSD_ATTEMPTS);
                        let mut ss = [0f32; 162];
                        for (o, q) in ss
                            .iter_mut()
                            .zip(crate::fec::conv::fano::wsprd_quantised_symbols(&llrs))
                        {
                            *o = f32::from(q) - 128.0;
                        }
                        let osd = super::osdwspr::osdwspr(&ss, i32::from(ndepth));
                        let f = codec.decode_codeword(
                            &osd.cw,
                            ladder.max_cycles_per_bit,
                            ladder.bias,
                            &mut fano_scratch,
                        );
                        let mut info = [0u8; 50];
                        info.copy_from_slice(&f.info);
                        match crate::msg::wspr::osd_probe(&info) {
                            crate::msg::wspr::OsdProbe::Break => break 'jitter,
                            crate::msg::wspr::OsdProbe::Skip => continue,
                            crate::msg::wspr::OsdProbe::Candidate(msg) => {
                                if !table.accepts(&msg) {
                                    continue;
                                }
                            }
                        }
                        #[cfg(feature = "instrument")]
                        super::instrument::bump(&super::instrument::OSD_OK);
                        (info, osd.nhardmin, f.cycles, f.metric, 1u8)
                    }
                    _ => continue,
                }
            };
            let Some(message) = crate::msg::Wspr50Message
                .unpack(&info_bits, &crate::engine::DecodeContext::default())
            else {
                // `unpk_`'s `noprint`: accepted, then neither printed nor subtracted.
                return LadderOutcome::NoPrint;
            };
            let lag_audio = lag * 32;
            let dt_sec = lag_audio as f32 / sample_rate as f32 - 1.0;
            // wsprd stops at the first success: its `while` conditions
            // are `ib <= nblocksize && not_decoded` and
            // `not_decoded && idt <= ...`, so neither loop keeps going
            // once a codeword is accepted.
            let mut hard = [0u8; 162];
            for (h, q) in hard
                .iter_mut()
                .zip(crate::fec::conv::fano::wsprd_quantised_symbols(&llrs))
            {
                *h = u8::from(q > 127);
            }
            return LadderOutcome::Decoded(
                Box::new(WsprResult {
                    message,
                    freq_hz: best_freq + super::baseband::CENTER_HZ
                        - 1.5 * super::demod::TONE_SPACING_HZ,
                    start_sample: lag_audio.max(0) as usize,
                    dt_sec,
                    drift_hz: best_drift,
                    info_bits,
                    snr_db: 0.0,
                    hashed_call: None,
                    f1_hz: best_freq,
                    sync: 0.0,
                    pass: 0,
                    blocksize: if nblock == 0 { 4 } else { nblock as u8 },
                    jitter: IIFAC * ii,
                    decode_type,
                    nhardmin,
                    cycles,
                    metric,
                }),
                hard,
            );
        }
    }
    LadderOutcome::Failed
}

/// Fano's cycle budget per bit, `wsprd.c:819` `maxcycles = 10000` (the
/// `-C` default). Embedded, `wspr-fano-cap-fast` halves it; see the comment
/// at the Fano call below for what that costs.
#[cfg(not(feature = "wspr-fano-cap-fast"))]
pub(crate) const WSPR_FANO_CYCLE_BUDGET: u64 = 10_000;
#[cfg(feature = "wspr-fano-cap-fast")]
pub(crate) const WSPR_FANO_CYCLE_BUDGET: u64 = 5_000;

/// Which OSD a scan runs, `wsprd`'s `-o` (`wsprd.c:826,1422`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum OsdSel {
    /// This crate's own OSD, the one the [`Depth`](crate::decoder::Depth) mapping has always
    /// run: the final pass only, an order-2 sweep over the float soft symbols (`wspr::osd`),
    /// gated on the callsign table. **A deliberate divergence from `wsprd -o 4`** (CLAUDE.md
    /// §2.1), which runs `osdwspr` depth 4 on the quantised symbols at every rung of every pass;
    /// measured against `wsprd`, this OSD matches none of `-o 0..5` (`tests/wsprd_beta6_parity.rs`,
    /// `docs/notes/WSPRD_PARITY.md`). `OsdSel::Wsprd` is the faithful one.
    Legacy,
    /// `-o -1`, `wsprd`'s default: none.
    Off,
    /// `-o n`: `osdwspr` (`wspr::osdwspr`) at `ndepth = n` on every rung of every pass,
    /// gated on the callsign table.
    Wsprd(u8),
}

/// What a candidate's decode ladder tries: `wsprd`'s `-q` (no DT
/// peak-up jitter), `-C` (Fano cycles per bit), `-z` (Fano metric bias), `-o`
/// (OSD) and whether decodes are subtracted (`-s` turns it off).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Ladder {
    /// Retry the demodulation at `shift ± 8k` baseband samples
    /// (`wsprd.c:1366-1475`); `-q` (`quickmode`) tries position 0 only.
    pub jitter: bool,
    pub max_cycles_per_bit: u64,
    /// `bias` (`wsprd.c:830`, `-z`): subtracted from `metric_tables[2]` to build `mettab`
    /// (`wsprd.c:944-945`).
    pub bias: f32,
    pub osd: OsdSel,
    /// `subtraction` (`wsprd.c:824`): a decode is subtracted from the data, and only then
    /// does `wsprd` count its hard errors (`wsprd.c:1497-1503`).
    pub subtraction: bool,
    /// `wsprd`'s subtraction replica and `unpk_` limits (`wsprd.c:1497-1503`): see
    /// `WsprExtras::replica_from_text`.
    pub replica_from_text: bool,
    /// `wsprd`'s acceptance rules: the RMS gate on the quantised symbols (`wsprd.c:1400-1408`)
    /// and its duplicate rule (`:1506-1516`); see `WsprExtras::wsprd_gates`.
    pub wsprd_gates: bool,
}

impl Ladder {
    pub(crate) const DEFAULT: Ladder = Ladder {
        jitter: true,
        max_cycles_per_bit: WSPR_FANO_CYCLE_BUDGET,
        bias: crate::fec::ConvFano::METRIC_BIAS,
        osd: OsdSel::Legacy,
        subtraction: true,
        replica_from_text: false,
        wsprd_gates: false,
    };
}

/// What a scan does, after the arguments the WSJT-X GUI gives `wsprd` per
/// decoding depth (`mainwindow.cpp:2824-2826`): Fast `-qB`, Normal
/// `-C 500 -o 4`, Deep `-C 500 -o 4 -d`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ScanDepth {
    /// `npasses` (`wsprd.c:825`): 1 is `-s`, 2 is `-B` and stops after the two early
    /// passes, 3 adds the final one with the coherent-block ladder.
    pub passes: u8,
    pub ladder: Ladder,
    /// `-d`: also take every third spectrum bin above the SNR floor as a
    /// candidate (`wsprd.c:1161-1169`).
    pub more_candidates: bool,
    /// `wsprd`'s candidate selection: `Some((fmin, fmax))` keeps the candidates whose
    /// frequency offset from 1500 Hz is inside the window (`wsprd.c:781,1172-1183`), taking
    /// them in `wsprd`'s order (the first 200 local maxima, then `-d`'s). `None` is this
    /// crate's own (the strongest `max_candidates`, no window).
    pub freq_window: Option<(f32, f32)>,
    /// `-e` (`wsprd.c:1172-1176`): added to the window, once per pass the loop runs.
    pub dial_error_hz: f64,
}

impl ScanDepth {
    /// The scan every 0.12 request ran: `wsprd`'s defaults.
    pub(crate) const DEFAULT: ScanDepth = ScanDepth {
        passes: 3,
        ladder: Ladder::DEFAULT,
        more_candidates: false,
        freq_window: None,
        dial_error_hz: 0.0,
    };
}

// No front padding. `wsprd` addresses a signal that starts before the recording (`dt < -1.0`)
// directly: its coarse search reaches `shift = 128 * (k0 + 1)` down to -1152 baseband samples
// (`wsprd.c:1250`) and its demodulator skips the samples that are not there (`k > 0 && k < np`,
// `wsprd.c:278`). The scan did the same on a copy of the audio with 3.072 s of zeros in front
// through 0.16.1; that moved every spectrogram row by a pad, which changed the coarse sync of the
// same alignment (0.09791 -> 0.10070 on one -32 dB file) and so which candidates reached `minsync2`.

/// **Embedded-only** (feature `wspr-pass2-topn`; off by default, host
/// stays on [`decode_pass2_candidate`]'s wsprd-faithful full ladder).
/// How many pass-2 candidates, ranked by *refined* sync descending,
/// get the full Fano + DT-peak-up + OSD ladder. The rest are refined
/// (cheap — refine_cascade + minsync2 only) and then dropped.
///
/// Not a wsprd-faithful number — wsprd runs the ladder on every
/// `minsync2` survivor, no further cut. This is this crate's own
/// addition, and it costs real (if bounded) recall risk rather than
/// being provably safe the way `minsync2` is, so the number is
/// deliberately conservative and the evidence for it is recorded
/// here rather than assumed:
///
/// - the WSJT-X golden's own G8VDQ (rank 1 of 3 minsync2 survivors)
///   and W3BI (rank 2 of 4, in pass 1 — not gated by this constant,
///   which applies to pass 2 only, but the same population) show the
///   real decode is not always the single strongest candidate;
/// - a 260-trial AWGN sweep across 13 SNR levels
///   (`wspr_rank_sweep`, `tests/wspr_sweep.rs`) found the real
///   transmitted signal, whenever it decoded at all, landed at rank 0
///   or rank 1 in **every** trial — never rank 2 or beyond. That
///   sweep is single-signal (no SIC residual, no competing stations),
///   so it brackets pass 2's *isolated weak candidate* case rather
///   than reproducing pass 2 exactly, but it is the broadest evidence
///   available and it never once needed more than rank 1.
///
/// 2 is therefore the minimum this evidence supports, not a round
/// number. On the golden file this drops pass 2 wall-clock from
/// 107.5 s to roughly 72 s (measured 3→2 kept candidates), while
/// keeping the file's 9/9 recall (see the doc comment update in
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`).
///
/// **Also crucial, from the same sweep**: OSD is *not* dead weight
/// here the way it looked from the golden file alone (where its
/// success count was 0/0) — at -32/-31 dB, the marginal-detection
/// transition zone, OSD accounted for the *majority* of hits (7/8 and
/// 13/15). This constant trims *which candidates* reach the ladder;
/// it does not touch the ladder itself, and OSD stays in it.
#[cfg(feature = "wspr-pass2-topn")]
pub const PASS2_DEEP_LADDER_TOP_N: usize = 2;

/// `minsync2`'s final-pass threshold — matches
/// [`decode_at_baseband_inner`]'s own, `refine_drift =
/// false`. Shared by [`rank_pass2_candidates`] and
/// [`decode_at_baseband_inner`] rather than duplicated
/// as two magic-number `const`s that could drift apart.
#[cfg(feature = "wspr-pass2-topn")]
const MINSYNC2_FINAL: f32 = 0.10;

/// One coarse candidate after [`refine_cascade`] + the `minsync2`
/// gate, ready for [`deep_decode_pass2_candidate`]. `pub`: this is
/// [`rank_pass2_candidates`]'s return type, needed by any caller that
/// wants to split the ranked survivors across cores itself (issue
/// #260's dual-core pass-2 dispatch — see
/// `embedded_shared::apps::wspr_dual_core`) rather than always
/// running [`decode_pass2_top_n`]'s sequential Stage 2.
#[cfg(feature = "wspr-pass2-topn")]
pub struct WsprPass2Candidate<'c> {
    pub sync: f32,
    pub c: &'c super::coarse_baseband::BasebandCandidate,
    pub freq: f32,
    pub lag: i32,
    pub drift: f32,
    pub isqs: super::demod::IsQs,
}

/// Stage 1 of pass 2's candidate loop: refine + `minsync2`-filter
/// every coarse candidate (cheap — no Fano/OSD), then rank the
/// survivors by refined sync descending and keep the top
/// [`PASS2_DEEP_LADDER_TOP_N`] — see that constant's doc comment for
/// the AWGN-sweep evidence this cutoff is built on.
///
/// Split out of what used to be [`decode_pass2_top_n`]'s inline Stage
/// 1 so a dual-core dispatcher can rank once (cheap, parallel across
/// every coarse candidate) and then hand each of the ≤
/// `PASS2_DEEP_LADDER_TOP_N` survivors to a *different* core for
/// [`deep_decode_pass2_candidate`] — the expensive half — instead of
/// running that half sequentially on one core.
#[cfg(feature = "wspr-pass2-topn")]
pub fn rank_pass2_candidates<'c>(
    idat: &[f32],
    qdat: &[f32],
    cands: &'c [super::coarse_baseband::BasebandCandidate],
) -> Vec<WsprPass2Candidate<'c>> {
    // A plain `fn`, not a closure: closures infer one concrete lifetime
    // for their signature from first use, not the
    // `for<'a> Fn(&'a Candidate) -> Option<WsprPass2Candidate<'a>>`
    // this call site needs (`par_iter`/`iter` hand a fresh borrow per
    // item) — a `fn` item gets that generality for free via ordinary
    // lifetime elision.
    fn refine_one<'c>(
        idat: &[f32],
        qdat: &[f32],
        c: &'c super::coarse_baseband::BasebandCandidate,
    ) -> Option<WsprPass2Candidate<'c>> {
        let f0_center_init = c.freq_hz + 1.5 * super::demod::TONE_SPACING_HZ;
        let f0_baseband_init = f0_center_init - super::baseband::CENTER_HZ;
        let lag_baseband_init = c.lag_baseband;
        let mut isqs = super::demod::IsQs::zeroed();
        let (freq, lag, drift, sync) = refine_cascade(
            idat,
            qdat,
            f0_baseband_init,
            lag_baseband_init,
            c.drift_hz,
            false,
            &mut isqs,
        );
        if sync <= MINSYNC2_FINAL {
            #[cfg(feature = "instrument")]
            super::instrument::bump(&super::instrument::MINSYNC2_REJECTED);
            return None;
        }
        Some(WsprPass2Candidate {
            sync,
            c,
            freq,
            lag,
            drift,
            isqs,
        })
    }
    // Embarrassingly parallel exactly like pass 0/1's own per-candidate
    // decode already is — same rationale, same `par_iter()` pattern.
    // This matters here specifically: cutting stage 2 down to
    // `PASS2_DEEP_LADDER_TOP_N` candidates only pays off if stage 1
    // (now run over *every* coarse candidate, not just survivors)
    // doesn't itself become a sequential bottleneck on a many-core
    // host — an earlier version of this function ran stage 1 as a
    // plain sequential loop and measured *slower* wall-clock overall
    // despite doing less total work, purely from losing parallelism.
    #[cfg(feature = "parallel")]
    let mut survivors: Vec<WsprPass2Candidate<'_>> = cands
        .par_iter()
        .filter_map(|c| refine_one(idat, qdat, c))
        .collect();
    #[cfg(not(feature = "parallel"))]
    let mut survivors: Vec<WsprPass2Candidate<'_>> = cands
        .iter()
        .filter_map(|c| refine_one(idat, qdat, c))
        .collect();

    survivors.sort_by(|a, b| {
        b.sync
            .partial_cmp(&a.sync)
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    survivors.truncate(PASS2_DEEP_LADDER_TOP_N);
    survivors
}

/// Stage 2 of pass 2's candidate loop: the full Fano + DT-peak-up +
/// OSD ladder on one already-[`rank_pass2_candidates`]-ranked
/// survivor. `pub` for the same dual-core reason as
/// [`WsprPass2Candidate`] — lets a caller run this on a different
/// core per candidate instead of [`decode_pass2_top_n`]'s sequential
/// loop over all survivors.
///
/// `budget`: forwarded to [`decode_from_refined`] unchanged — see its
/// own doc comment for the contract. `None` runs the full wsprd-
/// faithful ladder, unconditionally; this is the only place a caller
/// can bound a single candidate's *failing*-path wall-clock, which is
/// exactly the case that dominates pass 2 (see
/// `docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s "Dual-core,
/// take two" — a non-converging candidate can cost 4-5x a converging
/// one).
#[cfg(feature = "wspr-pass2-topn")]
pub fn deep_decode_pass2_candidate(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    confirmed: &WsprCallsignTable,
    r: &WsprPass2Candidate<'_>,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
) -> Option<WsprResult> {
    let mut d = decode_from_refined(
        idat,
        qdat,
        sample_rate,
        r.freq,
        r.lag,
        r.drift,
        Some(&r.isqs),
        &[1, 2, 3, 0],
        Some(confirmed),
        budget,
        Ladder::DEFAULT,
    )?;
    let start_refined = d.start_sample;
    d.dt_sec = (start_refined as i64 - pad as i64) as f32 / sample_rate as f32
        - <super::Wspr as crate::engine::FrameLayout>::TX_START_OFFSET_S;
    d.start_sample = start_refined.saturating_sub(pad);
    d.snr_db = r.c.snr_db;
    // The embedded final pass is pass 2 and ranks by the refined sync it already has.
    d.sync = r.sync;
    d.pass = 2;
    Some(d)
}

/// Pass 2's own candidate loop, sequential: [`rank_pass2_candidates`]
/// then [`deep_decode_pass2_candidate`] on every survivor. This is
/// what [`decode_scan`]/[`decode_scan_inner`] use when the feature is
/// on, and what `embedded_shared::apps::wspr_bench` used to call
/// directly before the dual-core split above existed (that bench
/// predates this function and runs its own hand-inlined pass 0/1/2
/// loop — the "D pattern", avoiding a `tlsf_malloc` corruption bug
/// when `decode_block`-shaped helpers are called from a long bench
/// loop — rather than going through `decode_scan`).
///
/// `budget`: the same budget passed to every survivor's
/// [`deep_decode_pass2_candidate`] call — one shared check, not one
/// per candidate, since `&(dyn Fn() -> bool + Sync)` is a plain
/// shared reference and `par_iter()` needs `Sync` regardless. `None`
/// (host's own default, see [`decode_scan_inner`]) runs the full
/// wsprd-faithful ladder on every survivor.
#[cfg(feature = "wspr-pass2-topn")]
pub fn decode_pass2_top_n(
    idat: &[f32],
    qdat: &[f32],
    sample_rate: u32,
    pad: usize,
    cands: &[super::coarse_baseband::BasebandCandidate],
    confirmed: &WsprCallsignTable,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
) -> Vec<WsprResult> {
    let survivors = rank_pass2_candidates(idat, qdat, cands);

    // Stage 2 (the Fano/OSD ladder) is the expensive half — this is
    // exactly the work the old per-candidate `decode_pass2_candidate`
    // ran under `par_iter()`. Losing that here (a plain sequential
    // `for` over `PASS2_DEEP_LADDER_TOP_N` survivors) was the second
    // half of the same regression stage 1's fn-conversion fixed: two
    // candidates run one-after-another cost roughly 2x one candidate's
    // wall-clock even though the total work dropped; run them
    // concurrently instead now that there are only ever
    // `PASS2_DEEP_LADDER_TOP_N` of them. (Embedded targets go further
    // still and run them on *separate cores* — see
    // `embedded_shared::apps::wspr_dual_core` — this sequential/
    // `par_iter` version is host's own path.)
    #[cfg(feature = "parallel")]
    let out: Vec<WsprResult> = survivors
        .par_iter()
        .filter_map(|r| {
            deep_decode_pass2_candidate(idat, qdat, sample_rate, pad, confirmed, r, budget)
        })
        .collect();
    #[cfg(not(feature = "parallel"))]
    let out: Vec<WsprResult> = survivors
        .iter()
        .filter_map(|r| {
            deep_decode_pass2_candidate(idat, qdat, sample_rate, pad, confirmed, r, budget)
        })
        .collect();
    out
}

/// One pass's configuration, `wsprd.c:1066-1073`.
struct SicPass<'a> {
    /// Passes 0 and 1: drift refinement on, `minsync2 = 0.12`. The final pass:
    /// off, `0.10`.
    early: bool,
    /// The block sizes the pass's ladder tries, in order (`0` is the
    /// bit-by-bit rung).
    nblocks: &'a [usize],
    /// `wsprd`'s `ipass`.
    index: u8,
    /// Whether OSD may rescue a candidate Fano gave up on.
    use_osd: bool,
}

/// A candidate after `wsprd.c:1285-1343`'s refine cascade.
struct RefinedCandidate {
    snr_db: f32,
    /// Baseband frequency of the signal centre.
    freq: f32,
    /// Baseband lag.
    lag: i32,
    drift: f32,
    sync: f32,
}

fn refine_candidate(
    idat: &[f32],
    qdat: &[f32],
    c: &super::coarse_baseband::BasebandCandidate,
    refine_drift: bool,
) -> RefinedCandidate {
    let f0 = c.freq_hz + 1.5 * super::demod::TONE_SPACING_HZ - super::baseband::CENTER_HZ;
    let mut isqs = super::demod::IsQs::zeroed();
    let (freq, lag, drift, sync) = refine_cascade(
        idat,
        qdat,
        f0,
        c.lag_baseband,
        c.drift_hz,
        refine_drift,
        &mut isqs,
    );
    if sync <= MINSYNC1 {
        // `wsprd.c:1325-1343`: not above `minsync1`, so the candidate keeps what the coarse
        // search gave it, sync included, and `minsync2` is tested on that.
        return RefinedCandidate {
            snr_db: c.snr_db,
            freq: f0,
            lag: c.lag_baseband,
            drift: c.drift_hz,
            sync: c.sync,
        };
    }
    RefinedCandidate {
        snr_db: c.snr_db,
        freq,
        lag,
        drift,
        sync,
    }
}

/// `wsprd.c:1347-1362`: candidates that landed on the same frequency and lag
/// collapse to the stronger one, and those at or below `minsync2` are dropped.
/// (A duplicate of a dropped candidate is not remembered, as in the C.)
fn survivors(refined: Vec<RefinedCandidate>, minsync2: f32) -> Vec<RefinedCandidate> {
    let mut out: Vec<RefinedCandidate> = Vec::new();
    for r in refined {
        if let Some(k) = out
            .iter()
            .position(|o| (r.freq - o.freq).abs() < 0.05 && (r.lag - o.lag).abs() < 16)
        {
            if r.sync > out[k].sync {
                out[k] = r;
            }
        } else if r.sync > minsync2 {
            out.push(r);
        }
    }
    out
}

/// `decodes[50]` (`wsprd.c:797`): the most decodes `wsprd` lists.
const WSPRD_MAX_DECODES: usize = 50;
/// A hashed call that still prints: `<call>` in a 13-byte buffer (`wsprd_utils.c:392-393`).
const HASHED_CALL_MAX: usize = 10;

/// `wsprd`'s `callsign` for the duplicate test: the call of a Type 1 or 2, `<call>` or
/// `<...>` for a Type 3 (`unpk_`).
fn wsprd_callsign(r: &WsprResult) -> String {
    match &r.message {
        WsprMessage::Type1 { callsign, .. } | WsprMessage::Type2 { callsign, .. } => {
            callsign.clone()
        }
        WsprMessage::Type3 { .. } => match &r.hashed_call {
            Some(c) => alloc::format!("<{c}>"),
            None => String::from("<...>"),
        },
    }
}

/// `wsprd.c:1506-1511`: `!strcmp(callsign, allcalls[i]) && fabs(f1 - allfreqs[i]) < 4.0`, no
/// time test.
fn wsprd_duplicate(seen: &[WsprResult], d: &WsprResult) -> bool {
    let call = wsprd_callsign(d);
    seen.iter()
        .any(|s| (d.f1_hz - s.f1_hz).abs() < 4.0 && wsprd_callsign(s) == call)
}

/// The 50 bits `get_wspr_channel_symbols` (`wsprsim_utils.c:165-262`) would pack from the text
/// `wsprd` printed, or `None` where it returns 0. It classifies by the text: a Type 3 is a
/// leading `<`, and packs `nhash` of what is between the brackets, `...` when the call is not
/// in the table. Types 1 and 2 repack to the decoded bits, bar the two texts it refuses: a
/// Type 1 call that is not 3 to 6 characters (`i1 >= 3 && i1 < 7`) and a Type 2 that begins
/// with `/` (`i2 == 0`).
///
/// DIVERGENCE (known gap, `docs/notes/WSPRD_PARITY.md`): keeping the decoded bits assumes the
/// text re-packs to them. A field with an inner blank (`unpackcall` NULs blanks,
/// `wsprd_utils.c:191-195`) or a short Type 2 base call re-packs differently in the C; only
/// false decodes carry such fields. Faithful fix: port `get_wspr_channel_symbols` on the text.
fn replica_info_from_text(
    msg: &WsprMessage,
    hashed: Option<&str>,
    info: &[u8; 50],
) -> Option<[u8; 50]> {
    match msg {
        WsprMessage::Type1 { callsign, .. } => (3..7).contains(&callsign.len()).then_some(*info),
        WsprMessage::Type2 { callsign, .. } => (!callsign.starts_with('/')).then_some(*info),
        WsprMessage::Type3 { .. } => {
            let hash = u32::from(bucket(hashed.unwrap_or("...")));
            let mut data = [0u8; 7];
            for (i, b) in info.iter().enumerate() {
                if b & 1 != 0 {
                    data[i / 8] |= 1 << (7 - (i % 8));
                }
            }
            let (n1, n2) = crate::msg::wspr::unpack50(&data);
            // `m = 128 * ihash + ntype + 64`: the low seven bits of `n2` are `ntype + 64`.
            let packed = crate::msg::wspr::pack50(n1, (hash << 7) | (n2 & 127));
            let mut out = [0u8; 50];
            for (i, o) in out.iter_mut().enumerate() {
                *o = (packed[i / 8] >> (7 - (i % 8))) & 1;
            }
            Some(out)
        }
    }
}

/// One pass of wsprd's decode loop (`wsprd.c:1285-1480`), on the current data:
/// every candidate is refined first (on the unsubtracted data, in parallel
/// where available), then the survivors are decoded **one at a time**, each
/// accepted decode being subtracted from `idat`/`qdat` before the next
/// candidate is looked at. Returns how many candidates decoded (wsprd's
/// `ndecodes_pass`, duplicates included).
///
/// The subtraction uses the refined alignment `shift1`, not the jittered one
/// the decode succeeded at, and happens before the duplicate test, as in the C
/// (`wsprd.c:1450-1457`).
#[allow(clippy::too_many_arguments)]
fn sic_pass(
    idat: &mut [f32],
    qdat: &mut [f32],
    cands: &[super::coarse_baseband::BasebandCandidate],
    sample_rate: u32,
    pad: usize,
    pass: SicPass<'_>,
    confirmed: &mut WsprCallsignTable,
    ladder: Ladder,
    seen: &mut Vec<WsprResult>,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
    on_result: Option<&(dyn Fn(&WsprResult) + Sync)>,
) -> usize {
    const MINSYNC2_EARLY: f32 = 0.12;
    const MINSYNC2_FINAL: f32 = 0.10;
    let minsync2 = if pass.early {
        MINSYNC2_EARLY
    } else {
        MINSYNC2_FINAL
    };
    let refined: Vec<RefinedCandidate> = {
        let (i, q): (&[f32], &[f32]) = (idat, qdat);
        #[cfg(feature = "parallel")]
        {
            cands
                .par_iter()
                .map(|c| refine_candidate(i, q, c, pass.early))
                .collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            cands
                .iter()
                .map(|c| refine_candidate(i, q, c, pass.early))
                .collect()
        }
    };
    let mut ndecodes = 0usize;
    let surv = survivors(refined, minsync2);
    for r in surv {
        // The caller's budget, once per candidate before it is claimed (the
        // refine above is not cut); a candidate already running finishes, bar
        // the ladder's own poll between its positions.
        if let Some(check) = budget
            && !check()
        {
            break;
        }
        let osd = pass.use_osd.then_some(&*confirmed);
        let (mut d, hard) = match ladder_decode(
            idat,
            qdat,
            sample_rate,
            r.freq,
            r.lag,
            r.drift,
            None,
            pass.nblocks,
            osd,
            budget,
            ladder,
        ) {
            LadderOutcome::Decoded(d, hard) => (*d, hard),
            LadderOutcome::NoPrint => {
                // Counted, as `ndecodes_pass++` (`wsprd.c:1481`) is; not subtracted
                // (`subtraction && !noprint`), not listed.
                ndecodes += 1;
                continue;
            }
            LadderOutcome::Failed => continue,
        };
        ndecodes += 1;
        d.sync = r.sync;
        d.pass = pass.index;
        // wsprd prints `dt = shift1 * dt - 1.0` (`wsprd.c:1473`): the refined
        // alignment, not the jittered one the decode succeeded at (up to
        // +-64 baseband samples, 0.17 s, away).
        d.dt_sec = ((r.lag as i64 * 32 - pad as i64) as f32) / sample_rate as f32
            - <super::Wspr as crate::engine::FrameLayout>::TX_START_OFFSET_S;
        d.start_sample = d.start_sample.saturating_sub(pad);
        d.snr_db = r.snr_db;
        // `unpk_` looks a Type 3 hash up in the table as it stands, and
        // stores a Type 1 / 2 call (`wsprd_utils.c:353-356,364-369,390`): a Type 3
        // decoded before its sender's own message prints `<...>`.
        if let WsprMessage::Type3 { callsign_hash, .. } = &d.message {
            d.hashed_call = confirmed.resolve(*callsign_hash).map(String::from);
        }
        // `unpk_`'s `format_checked` into the 13-byte `callsign` (`wsprd_utils.c:392-393`):
        // `<call>` must fit, so a hashed call of 11 or 12 characters is `noprint`.
        if ladder.replica_from_text
            && d.hashed_call
                .as_ref()
                .is_some_and(|c| c.len() > HASHED_CALL_MAX)
        {
            continue;
        }
        confirmed.record(&d.message);
        if ladder.subtraction {
            // `wsprd` rebuilds the replica from the printed text
            // (`get_wspr_channel_symbols(call_loc_pow, ...)`, `wsprd.c:1498`), not from the
            // decoded bits, and counts `nhardmin` against it (`:1500`). A Type 3 whose call is
            // not in the table prints `<...>`, so the replica carries `nhash("...")` and is the
            // wrong codeword; a text the rebuild refuses ends the pass's candidate loop
            // (`break`, `:1501-1503`). **The `Depth` path subtracts the decoded codeword**, the
            // right signal, a deliberate divergence from `wsprd` (CLAUDE.md §2.1).
            let bits = if ladder.replica_from_text {
                match replica_info_from_text(&d.message, d.hashed_call.as_deref(), &d.info_bits) {
                    Some(b) => b,
                    None => break,
                }
            } else {
                d.info_bits
            };
            let symbols = super::encode_channel_symbols(&bits);
            super::subtract::subtract_signal_baseband_at(
                idat, qdat, r.freq, r.lag, d.drift_hz, &symbols,
            );
            if ladder.replica_from_text && d.decode_type == 0 {
                use crate::engine::FecCodec;
                let mut cw = [0u8; 162];
                crate::fec::ConvFano.encode(&bits, &mut cw);
                d.nhardmin = cw.iter().zip(hard.iter()).filter(|(c, h)| c != h).count() as i32;
            }
        }
        if ladder.wsprd_gates {
            // `wsprd.c:1506-1516`: same callsign text within 4 Hz of a listed decode is a
            // duplicate (still subtracted, above); at most 50 are listed.
            if seen.len() >= WSPRD_MAX_DECODES || wsprd_duplicate(seen, &d) {
                continue;
            }
            if let Some(cb) = on_result {
                cb(&d);
            }
            seen.push(d);
        } else {
            push_unique(seen, d, FREQ_DEDUP_HZ, TIME_DEDUP_SAMPLES, on_result);
        }
    }
    ndecodes
}

/// Scan an audio buffer for any number of WSPR frames, returning all
/// successful decodes: the wsprd-equivalent coarse search on the 375 Hz
/// baseband, then wsprd's three decode passes over its candidates.
/// Duplicate decodes (same message within ±5 Hz and ±1 symbol) collapse
/// to the earliest-candidate hit. Public through [`super::DecodeRequest`],
/// which documents `on_result`'s delivery contract and the cross-slot
/// table.
pub(super) fn decode_scan_inner(
    audio: &[f32],
    sample_rate: u32,
    nominal_start_sample: usize,
    params: &SearchParams,
    on_result: Option<&(dyn Fn(&WsprResult) + Sync)>,
    carried: Option<&mut WsprCallsignTable>,
    depth: ScanDepth,
    budget: Option<&(dyn Fn() -> bool + Sync)>,
) -> (Vec<WsprResult>, bool, Option<u32>) {
    let ladder = depth.ladder;
    // The caller's budget, remembering whether it ever said stop: what a
    // `BudgetReport::exhausted` is made of. Polled per survivor in every pass
    // and between a candidate's ladder positions; a spent budget skips the
    // passes after it.
    let hit = core::sync::atomic::AtomicBool::new(false);
    let hit_ref = &hit;
    let tracked = budget.map(|b| {
        move || {
            let ok = b();
            if !ok {
                hit_ref.store(true, core::sync::atomic::Ordering::Relaxed);
            }
            ok
        }
    });
    let budget: Option<&(dyn Fn() -> bool + Sync)> =
        tracked.as_ref().map(|c| c as &(dyn Fn() -> bool + Sync));
    let spent = || hit.load(core::sync::atomic::Ordering::Relaxed);
    // No front padding (see the note above `PASS2_DEEP_LADDER_TOP_N`): `wsprd` reaches a negative
    // `shift` directly, and reads only its `npoints` (114 s) of the recording. `pad` is 0 here; it
    // stays a parameter of `decode_pass2_top_n` and `deep_decode_pass2_candidate`, which the embedded
    // pipelines (`wspr_scan`) call with the pad of the buffer they build themselves.
    let pad = 0usize;
    let audio = &audio[..audio.len().min(super::baseband::WSPRD_NPOINTS)];
    let _ = nominal_start_sample;
    // Decimate ONCE up-front; the wsprd-equivalent coarse and the
    // demod both consume the same baseband buffer, so we save 32×
    // FFT work vs running each separately.
    // Channelizer selection. The reference (`decimate_to_baseband`,
    // wsprd's own) is exact and is what every number in this crate was
    // measured against; it is also unrunnable on an ESP32-S3, whose
    // 8 MB PSRAM cannot hold its 11.25 MiB `NFFT1` buffer and whose FFT
    // backend tops out at 8 192. Embedded turns on `wspr-ddc` (single-
    // stage streaming down-converter) or `wspr-ddc-cascade` (two-stage
    // cascade, ~4.5x cheaper and internal-DRAM-sized) instead — see
    // `wspr::ddc` for what each trades away.
    #[cfg(all(feature = "wspr-ddc", feature = "wspr-ddc-cascade"))]
    compile_error!("wspr-ddc and wspr-ddc-cascade select different channelizers — enable only one");
    #[cfg(not(any(feature = "wspr-ddc", feature = "wspr-ddc-cascade")))]
    let (mut idat, mut qdat) = super::baseband::decimate_to_baseband(audio);
    #[cfg(feature = "wspr-ddc")]
    let (mut idat, mut qdat) = super::ddc::ddc_to_baseband(audio);
    #[cfg(feature = "wspr-ddc-cascade")]
    let (mut idat, mut qdat) = super::ddc::ddc_to_baseband_cascade(audio);
    // wsprd-equivalent coarse: 512-pt windowed FFT on the 375 Hz
    // baseband, time-averaged spectrum + 30 th-percentile noise
    // floor, peak detection on smspec, 3-D (freq, time, drift)
    // refinement. Lifts the coarse score landscape to actually peak
    // at the right (freq, dt) for weak signals next to strong ones
    // (W5BIT, W3BI). See `coarse_baseband.rs`.
    let max_drift = 4i32;
    // `wsprd.c:1172-1176`: `fmin += dialfreq_error; fmax += dialfreq_error;` runs **once per
    // pass the loop executes**, inside it, so with `-e` the window drifts by `-e` more every
    // pass: ported as it is (a deliberate faithful port of what looks like an upstream
    // oversight). On the golden `-e 45` shows it: the final pass's window starts at +25 Hz
    // and so loses G8VDQ (-35 Hz), where shifting once would keep it (fixture `f_e45`).
    let mut window = depth.freq_window;
    let shift_window = |w: &mut Option<(f32, f32)>| {
        if let Some((lo, hi)) = w {
            *lo = (f64::from(*lo) + depth.dial_error_hz) as f32;
            *hi = (f64::from(*hi) + depth.dial_error_hz) as f32;
        }
    };
    shift_window(&mut window);
    let bb_cands = super::coarse_baseband::coarse_baseband_ext(
        &idat,
        &qdat,
        pad,
        params.max_candidates,
        max_drift,
        depth.more_candidates,
        window,
    );
    // Use the wsprd-equivalent coarse only. Legacy `coarse_search`
    // (12 kHz spectrogram) costs ~30 s of recall-test runtime on a
    // 122 s slot and brings nothing the new coarse doesn't already
    // find — every golden hit on the WSJT-X reference WAV (ND6P,
    // WD4LHT, NM7J, KI7CI, DJ6OL, W3HH, W5BIT) comes through the
    // baseband path. Kept the legacy module for synth round-trip
    // tests; not invoked here.
    // `coarse_baseband` already returns candidates sorted by sync score
    // descending — no remap/re-sort needed; keeping `BasebandCandidate`
    // itself (rather than the legacy `SyncCandidate`) preserves each
    // candidate's `snr_db` through to the decode below.
    let mut cands = bb_cands;
    cands.truncate(params.max_candidates);
    // The first coarse search of the call: pass 0's list after `find_peaks` and the cap
    // (`npk`, `wsprd.c:1148-1189`): narrowed to fmin..fmax, in `wsprd`'s order, only with a
    // `ScanDepth::freq_window`; otherwise the strongest `max_candidates` of the baseband. Passes
    // 1 and 2 search again on what is left and are not counted (`SlotResult::candidates`, #697).
    let first_search = Some(u32::try_from(cands.len()).unwrap_or(u32::MAX));
    let mut seen: Vec<WsprResult> = Vec::new();
    // 2-D refinement: WSPR's Fano (K=32 convolutional, no CRC) is
    // sensitive to *both* sub-bin freq and sub-t_step time mis-
    // alignment. Coarse-search rounds to 1.46 Hz / 170 ms; this
    // pass refines:
    //   time : ±170 ms / 43 ms step ⇒ 9 points
    //   freq : ±2 Hz   / 0.5 Hz step ⇒ 9 points
    // ≈ 81 sync_score evals × candidate.
    //
    // Going finer in time (e.g. 10 ms) actually *hurts* recall on
    // weak signals: WSPR has no CRC, so the highest-sync_score
    // alignment is often a noise-pattern Fano ghost rather than the
    // true signal. 43 ms preserves true peaks while the coarser
    // grid keeps us out of the ghost-attractor region. Better
    // long-term fix is a Fano-metric / callsign-sanity gate; until
    // then, 43 ms is the empirical optimum on the WSJT-X golden.
    // Tightened grid (3 × 3 = 9 evals/cand, was 9 × 9 = 81): wsprd's
    // entire 3-pass SIC runs in seconds; our refine cost dominated
    // total wall time. The small grid still recovers the same 5/8
    // goldens against the WSJT-X reference WAV — most of the recall
    // win came from sub-bin demod (slice 1 of issue #17), not from
    // the dense refine grid.
    const REFINE_FREQ_RADIUS_HZ: f32 = 1.0;
    const REFINE_FREQ_STEP_HZ: f32 = 1.0;
    let nsps = (sample_rate as f32 * <super::Wspr as crate::engine::ModulationParams>::SYMBOL_DT)
        .round() as i64;
    let refine_time_radius = nsps / 8; // ≈85 ms half-window
    let refine_time_step = nsps / 8; // 1 step at radius → 3 cells in time
    // (idat, qdat) computed above and shared with coarse_baseband.
    // Suppress the unused-warnings for refine_align knobs — they
    // belong to the legacy 12 kHz path that the new baseband
    // demod has retired (decode_at_baseband does its own ±1.5 Hz /
    // ±0.1 s sweep around the coarse pick).
    let _ = (
        REFINE_FREQ_RADIUS_HZ,
        REFINE_FREQ_STEP_HZ,
        refine_time_radius,
        refine_time_step,
    );
    // wsprd runs **three** passes (`npasses = 3`, `wsprd.c:805`), not
    // two, and they are not all configured alike (`wsprd.c:998-1010`):
    //
    //   pass 0, 1 : nblocksize = 1, maxdrift = 4, minsync2 = 0.12
    //   pass 2    : nblocksize = 4, maxdrift = 0, minsync2 = 0.10
    //
    // `minsync2` is applied inside
    // `decode_at_baseband_inner`, keyed off the same
    // `refine_drift` flag this file already threads through for
    // `maxdrift` — see that function's doc comment for why the two
    // reuse one boolean rather than taking a separate parameter.
    //
    // Each pass subtracts what it decoded before the next one re-runs
    // the coarse search, so pass 2 sees a residual with *two* rounds of
    // strong signals removed. This crate previously collapsed that into
    // two passes, which cost both of the WSJT-X golden's weakest
    // signals: W3BI (−27 dB) needs the extra subtraction round, and
    // G8VDQ (−23 dB) needs pass 2's zero-drift estimate.
    //
    // `wsprd.c:999` skips pass 1 entirely when pass 0 decoded nothing,
    // on the reasoning that with nothing subtracted the residual is the
    // input and re-searching it cannot find anything new.
    //
    // Each candidate's decode call is a pure function of
    // (idat, qdat, sample_rate, candidate) — no shared mutable state
    // during the loop body, so the decode step runs in parallel
    // (mirroring ft8::decode's own par_iter()/filter_map()/collect()
    // pattern, src/ft8/decode.rs:361-368). The dedup pass stays
    // strictly sequential afterward, in the original (order-preserving)
    // candidate order, so "first occurrence wins" semantics hold.
    //
    // Decodes carry their padded-buffer alignment alongside, so the
    // subtraction at the end of each pass knows where to aim.
    // Callsigns a decode in this slot has established, seeded from the
    // caller's cross-slot table. The `Depth` mapping's OSD (`OsdSel::Legacy`) runs in the
    // final pass only, so what passes 0 and 1 decode is what it may re-find; `wsprd` itself
    // runs OSD only with `-o n >= 0`, then at every rung of every pass (`OsdSel::Wsprd`).
    // As in `wsprd`, every decode updates the table at once (`unpk_`'s `store_hash_entry`),
    // the final pass's included.
    let mut confirmed = match &carried {
        Some(t) => (*t).clone(),
        None => WsprCallsignTable::new(),
    };

    // wsprd's passes 0 and 1: `maxdrift = 4`, `minsync2 = 0.12`, `nblocksize =
    // 1` (`wsprd.c:1066-1073`). Each pass searches the *current* data, so the
    // second sees the first's decodes subtracted; and inside a pass each
    // decode is subtracted at once (`wsprd.c:1450`), so a later candidate is
    // decoded from data with its stronger neighbours already removed. This
    // used to decode a pass's candidates in parallel and subtract at the end
    // of the pass, which gave later candidates the unsubtracted data: on the
    // WSJT-X golden the soft symbols of W5BIT, decoded before any subtraction
    // upstream, differed from `wsprd`'s by 2.6 levels of 256, and G8VDQ (which
    // `wsprd` decodes at 8080 Fano cycles/bit, this crate needed 13318) was
    // lost.
    // `wsprd.c:1064-1076`: `for (ipass = 0; ipass < npasses; ipass++)`, and pass 1 is
    // skipped (`ipass = 2`) only when pass 0 decoded nothing **and** `npasses > 2`: with `-B`
    // (`npasses = 2`) pass 1 always runs, with `-s` (1) it does not exist. A skipped pass 1
    // is not a pass 2 (`maxdrift = 0`, `minsync2 = 0.10`, the coherent-block ladder) that was
    // moved: the loop still ends at `npasses`.
    //
    // Passes 0 and 1: `nblocksize = 1`, `maxdrift = 4`, `minsync2 = 0.12`; pass 2: all four
    // rungs, `maxdrift = 0`, `minsync2 = 0.10` ("no drift for smaller frequency estimator
    // variance"). Each pass searches the data as the one before left it.
    let mut first_pass_cands = Some(core::mem::take(&mut cands));
    let mut ndecodes_pass = 0usize;
    let mut ipass = 0u8;
    while ipass < depth.passes {
        if ipass == 1 && ndecodes_pass == 0 && depth.passes > 2 {
            ipass = 2;
        }
        if spent() {
            break;
        }
        let early = ipass < 2;
        let pass_cands = match first_pass_cands.take() {
            Some(c) => c,
            None => {
                shift_window(&mut window);
                let mut c = super::coarse_baseband::coarse_baseband_ext(
                    &idat,
                    &qdat,
                    pad,
                    params.max_candidates,
                    if early { max_drift } else { 0 },
                    depth.more_candidates,
                    window,
                );
                c.truncate(params.max_candidates);
                c
            }
        };
        // Feature `wspr-pass2-topn` (embedded targets only — see `decode_pass2_top_n`)
        // swaps the final pass's sequential, wsprd-faithful loop for one that refines and
        // `minsync2`-filters every candidate first, then runs the expensive ladder only on
        // the top `PASS2_DEEP_LADDER_TOP_N` by refined sync, in parallel; it does not
        // subtract inside the pass.
        #[cfg(feature = "wspr-pass2-topn")]
        if !early {
            let raw2: Vec<WsprResult> = decode_pass2_top_n(
                &idat,
                &qdat,
                sample_rate,
                pad,
                &pass_cands,
                &confirmed,
                budget,
            );
            for d in raw2 {
                push_unique(&mut seen, d, FREQ_DEDUP_HZ, TIME_DEDUP_SAMPLES, on_result);
            }
            ipass += 1;
            continue;
        }
        ndecodes_pass = sic_pass(
            &mut idat,
            &mut qdat,
            &pass_cands,
            sample_rate,
            pad,
            SicPass {
                early,
                index: ipass,
                nblocks: if early { &[1] } else { &[1, 2, 3, 0] },
                use_osd: match ladder.osd {
                    OsdSel::Legacy => !early,
                    OsdSel::Off => false,
                    OsdSel::Wsprd(_) => true,
                },
            },
            &mut confirmed,
            ladder,
            &mut seen,
            budget,
            on_result,
        );
        ipass += 1;
    }

    // Hand this slot's table back to the caller's cross-slot one: what it held and
    // every decode of the slot stored, in `wsprd`'s order (`unpk_` stores at once,
    // duplicates included, `wsprd.c:1481`), which is what `hashtable.txt` is made of.
    // An OSD decode cannot widen it: `WsprCallsignTable::accepts` only admits a call
    // that is *already* an entry, so every entry still traces back to a Fano decode.
    if let Some(t) = carried {
        *t = confirmed;
        // The embedded final pass (`decode_pass2_top_n`) decodes without storing.
        #[cfg(feature = "wspr-pass2-topn")]
        for d in &seen {
            t.record(&d.message);
        }
    }

    (seen, spent(), first_search)
}

/// Deinterleave 162 LLRs in place (same permutation as [`deinterleave`]
/// but for `f32` values). Thin wrapper over
/// [`crate::engine::interleave::deinterleave_bitrev`] — was its own
/// third inline copy of the bit-reversal formula ("avoid exposing a
/// pub helper"), no longer needed now that the shared version is
/// itself the pub helper.
fn deinterleave_llrs(llrs: &mut [f32; 162]) {
    crate::engine::interleave::deinterleave_bitrev(llrs);
}

#[cfg(test)]
mod minsync1_tests {
    use super::super::coarse_baseband::BasebandCandidate;
    use super::*;

    /// A candidate whose refined sync is at or below `minsync1` keeps the coarse frequency, shift,
    /// drift **and sync** (`wsprd.c:1325-1343`), so `minsync2` is tested on the coarse sync. On pure
    /// noise the refined sync is far below `minsync1`; a coarse sync of 0.77 must come back as it is.
    /// This fails if the rule is dropped: the refined sync would come back instead.
    #[test]
    fn a_candidate_at_or_below_minsync1_keeps_its_coarse_values_and_sync() {
        let mut x = 0x2545_f491u32;
        let mut noise = || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x as f32 / u32::MAX as f32 - 0.5
        };
        let n = super::super::baseband::NFFT2;
        let idat: Vec<f32> = (0..n).map(|_| noise()).collect();
        let qdat: Vec<f32> = (0..n).map(|_| noise()).collect();
        let c = BasebandCandidate {
            start_sample: 3200,
            lag_baseband: 100,
            freq_hz: 1498.15,
            drift_hz: 2.0,
            sync: 0.77,
            snr_db: -20.0,
        };
        let f0 = c.freq_hz + 1.5 * super::super::demod::TONE_SPACING_HZ
            - super::super::baseband::CENTER_HZ;
        // The precondition, so the test cannot pass for the wrong reason.
        let mut isqs = super::super::demod::IsQs::zeroed();
        let refined_sync = refine_cascade(&idat, &qdat, f0, 100, 2.0, true, &mut isqs).3;
        assert!(
            refined_sync <= MINSYNC1,
            "refined sync {refined_sync} is above minsync1"
        );

        let r = refine_candidate(&idat, &qdat, &c, true);
        assert_eq!(r.sync, 0.77);
        assert_eq!(r.lag, 100);
        assert_eq!(r.drift, 2.0);
        assert_eq!(r.freq, f0);
        assert_eq!(r.snr_db, -20.0);
    }
}

#[cfg(test)]
mod ladder_tests {
    use super::*;
    use core::sync::atomic::{AtomicUsize, Ordering};

    /// The replica `wsprd` subtracts (`get_wspr_channel_symbols`, `wsprsim_utils.c:165-262`):
    /// a Type 3 packs `nhash` of the call it printed, `...` when it is not in the table, and two
    /// texts are refused (a Type 1 call of fewer than 3 characters, a Type 2 starting `/`).
    #[test]
    fn the_replica_is_rebuilt_from_the_printed_text() {
        use crate::msg::wspr::pack_message;
        let bits = pack_message("<K1ABC> FN42AB 37").unwrap().bits;
        let msg = crate::msg::wspr::unpack(&bits).unwrap();
        // The call is in the table: its hash is the transmitted one, the codeword unchanged.
        assert_eq!(
            replica_info_from_text(&msg, Some("K1ABC"), &bits),
            Some(bits)
        );
        // It is not: `<...>` hashes `...`, a different codeword, differing only in the hash.
        let wrong = replica_info_from_text(&msg, None, &bits).unwrap();
        assert_ne!(wrong, bits);
        assert_eq!(
            wrong[..28],
            bits[..28],
            "the call slot is the grid, unchanged"
        );
        let t1 = WsprMessage::Type1 {
            callsign: "K1".into(),
            grid: "FN42".into(),
            power_dbm: 30,
        };
        assert_eq!(replica_info_from_text(&t1, None, &bits), None);
        let t2 = WsprMessage::Type2 {
            callsign: "/K1ABC".into(),
            power_dbm: 30,
        };
        assert_eq!(replica_info_from_text(&t2, None, &bits), None);
    }

    /// `wsprd.c:1443-1445`: when the OSD codeword does not unpack (`unpackcall` /
    /// `unpackgrid` / `unpackpfx` fail) the `break` leaves the DT peak-up loop of that block
    /// size, so its remaining positions are never tried. On baseband noise the OSD codeword
    /// fails to unpack about one time in thirty; some seed must therefore show a ladder that
    /// stopped short of its `4 * 17` positions. A `continue` there (the mutation this guards)
    /// tries every position of every rung, whatever the seed.
    #[test]
    fn an_unpackable_osd_codeword_ends_the_dt_loop() {
        let positions = 4 * (1 + 128 / 8);
        let mut shortest = positions;
        for seed in 1..=40u32 {
            let mut x = seed.wrapping_mul(2_654_435_761) | 1;
            let mut noise = |n: usize| -> alloc::vec::Vec<f32> {
                (0..n)
                    .map(|_| {
                        x ^= x << 13;
                        x ^= x >> 17;
                        x ^= x << 5;
                        (x as f32 / u32::MAX as f32) - 0.5
                    })
                    .collect()
            };
            let (idat, qdat) = (noise(46_080), noise(46_080));
            let polls = AtomicUsize::new(0);
            let budget = || {
                polls.fetch_add(1, Ordering::Relaxed);
                true
            };
            let ladder = Ladder {
                max_cycles_per_bit: 2,
                osd: OsdSel::Wsprd(0),
                ..Ladder::DEFAULT
            };
            let table = WsprCallsignTable::new();
            let _ = ladder_decode(
                &idat,
                &qdat,
                12_000,
                0.0,
                1200,
                0.0,
                None,
                &[1, 2, 3, 0],
                Some(&table),
                Some(&budget),
                ladder,
            );
            shortest = shortest.min(polls.load(Ordering::Relaxed));
        }
        assert!(
            shortest < positions,
            "every ladder tried all {positions} positions: no OSD codeword failed to unpack, or the break is gone"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::super::search::SearchParams;
    use super::super::synthesize_type1;
    use crate::msg::WsprMessage;

    #[test]
    fn synth_decode_roundtrip_k1abc_fn42_37() {
        let freq = 1500.0;
        let audio =
            synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("valid message");
        let r = crate::wspr::DecodeRequest::sniper(&audio, 12_000, 0, freq)
            .decode()
            .expect("decode");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
    }

    #[test]
    fn scan_recovers_message_without_freq_hint() {
        let freq = 1500.0;
        let audio = synthesize_type1("K1ABC", "FN42", 37, 12_000, freq, 0.3).expect("synth");
        let decodes = crate::wspr::DecodeRequest::new(&audio, 12_000)
            .params(SearchParams {
                freq_min_hz: 1450.0,
                freq_max_hz: 1550.0,
                ..crate::wspr::search::default_search_params()
            })
            .decode();
        assert!(!decodes.is_empty(), "at least one decode");
        let d = decodes.into_iter().next().unwrap();
        assert_eq!(
            d.message,
            WsprMessage::Type1 {
                callsign: "K1ABC".into(),
                grid: "FN42".into(),
                power_dbm: 37,
            }
        );
        assert!((d.freq_hz - 1500.0).abs() <= 2.0);
    }

    #[test]
    fn survives_moderate_awgn() {
        use core::f32::consts::PI;

        let freq = 1500.0;
        let mut audio =
            synthesize_type1("K9AN", "EN50", 33, 12_000, freq, 0.5).expect("valid message");

        // Deterministic "noise": superposition of a handful of off-tone
        // sinusoids plus a pseudorandom dither. This is a cheap AWGN
        // stand-in that keeps the test free of rand dependencies.
        let mut seed: u32 = 0x1234_5678;
        for (i, s) in audio.iter_mut().enumerate() {
            // Linear congruential pseudorandom for reproducible noise.
            seed = seed.wrapping_mul(1_103_515_245).wrapping_add(12345);
            let rnd = ((seed >> 16) as f32 / 32768.0 - 1.0) * 0.10;
            let off = 0.05 * (2.0 * PI * 2345.7 * i as f32 / 12_000.0).sin();
            *s += rnd + off;
        }

        let r = crate::wspr::DecodeRequest::sniper(&audio, 12_000, 0, freq)
            .decode()
            .expect("decode under noise");
        assert_eq!(
            r.message,
            WsprMessage::Type1 {
                callsign: "K9AN".into(),
                grid: "EN50".into(),
                power_dbm: 33,
            }
        );
    }
}

#[cfg(any(test, feature = "internal-testing"))]
#[cfg_attr(not(feature = "internal-testing"), allow(dead_code))]
pub(crate) mod testing {
    use super::*;

    /// Refined `best_sync` for one candidate — the exact value `minsync2`
    /// gates on — without running Fano/OSD. Diagnostic-only: written to
    /// answer "where does a known weak decode rank among the candidates
    /// `minsync2` keeps", which needs the per-candidate refined sync, not
    /// just the aggregate pass/fail counters `wspr::instrument` tracks.
    ///
    /// `freq_hz`/`start_sample` follow `decode_at_baseband_inner`'s
    /// own conventions (tone-0 audio Hz; audio-rate sample index).
    pub fn debug_refined_sync(
        idat: &[f32],
        qdat: &[f32],
        start_sample: usize,
        freq_hz: f32,
        drift_hz: f32,
        refine_drift: bool,
    ) -> f32 {
        let f0_center_init = freq_hz + 1.5 * super::super::demod::TONE_SPACING_HZ;
        let f0_baseband_init = f0_center_init - super::super::baseband::CENTER_HZ;
        let lag_baseband_init = start_sample as i32 / 32;
        // This caller wants `best_sync` alone, but the cascade needs a
        // champion buffer to work in, so it supplies one and drops it.
        let mut isqs = super::super::demod::IsQs::zeroed();
        refine_cascade(
            idat,
            qdat,
            f0_baseband_init,
            lag_baseband_init,
            drift_hz,
            refine_drift,
            &mut isqs,
        )
        .3
    }

    /// WSPR subtract configuration (continuous-phase 4-FSK). Mirrors WSJT-X
    /// `subtract_signal2` in `wsprd.c`: tone spacing 1.4648 Hz, 8192
    /// samples/symbol at 12 kHz, no GFSK shaping (WSPR is plain CPFSK).
    const WSPR_SUBTRACT: crate::engine::dsp::subtract::SubtractCfg =
        crate::engine::dsp::subtract::SubtractCfg {
            sample_rate: 12_000.0,
            tone_spacing_hz: 1.4648,
            samples_per_symbol: 8192,
            // WSPR's nominal symbol-0 start is 1.0 s into the slot; our
            // `start_sample` is already absolute, so the subtract layer
            // sees `dt_sec` as `(start - 1.0*FS) / FS`. `base_offset_s = 1.0`
            // matches the convention used by `WsprResult::dt_sec`.
            base_offset_s: 1.0,
            gfsk: None,
        };

    /// LPF kernel half-width for the channel-aware subtract, in audio
    /// samples. Chosen by measuring residual suppression on the WSJT-X
    /// golden — see the call site in `decode_scan_subtract`.
    const WSPR_SUBTRACT_LPF_HALF: usize = 600;

    /// A second SIC layer wrapped around the scan, at 12 kHz.
    /// **`internal-testing` only since #403** — not part of the public
    /// decode API, and not part of the reference decoder.
    ///
    /// The doc comment here used to claim this function "mirrors WSJT-X
    /// `wsprd.c`'s `npasses=3` SIC loop (`wsprd.c:998-1438`)". That was
    /// true when it was written, and stopped being true in
    /// [#275](https://github.com/jl1nie/mfsk-core/issues/275): porting
    /// wsprd's three-pass structure faithfully put that same loop
    /// *inside* `decode_scan_inner`, citing the same `wsprd.c:805` /
    /// `wsprd.c:999` lines, down to the "skip pass 1 when pass 0 decoded
    /// nothing" rule. The wrapper was left in place and its comment was
    /// not revisited, so two nested layers ended up claiming the same
    /// construct. wsprd has one such loop; this ran three inner passes
    /// twice.
    ///
    /// What it costs, measured on the WSJT-X golden
    /// (`wspr_diag_pass_ablation`, Ryzen 9 9900X): the plain scan 0.71 s
    /// for 9/9 goldens, this 2.38 s for the same 9/9 — **3.3× for zero
    /// marginal recall**. `WSPR_BENCHMARK.md`'s "Option C" reached the
    /// same conclusion by ablation before #275 landed.
    ///
    /// It was `#[deprecated]` and public, kept only because removing it
    /// was a breaking change. #403 made that break; the function stays
    /// behind `internal-testing` because `wspr_diag_pass_ablation` and the
    /// timing probes in `tests/wspr_wsjtx_samples.rs` are the evidence for
    /// the 3.3× figure above, and evidence that no longer runs cannot be
    /// re-checked.
    ///
    /// **Delivery order/dedup contract** for `on_result`: `cb` fires once
    /// per accepted decode, at this function's own SIC-pass dedup-then-push
    /// point (`all.push(d)` below) — not inside the per-pass scan each SIC
    /// round makes internally. The outer SIC accept point is the
    /// final-acceptance point, so `cb` fires exactly once per result that
    /// ends up in the returned `Vec`, in the same order.
    ///
    /// Returns deduplicated decodes from all passes.
    pub fn decode_scan_subtract(
        audio: &[f32],
        sample_rate: u32,
        nominal_start_sample: usize,
        params: &SearchParams,
        on_result: Option<&(dyn Fn(&WsprResult) + Sync)>,
    ) -> Vec<WsprResult> {
        use crate::engine::dsp::subtract::subtract_tones_lpf;
        use crate::engine::pipeline::is_scan_dup;

        // The subtract helper takes `&mut [i16]`; convert once, mutate
        // across passes, work on `f32` for `decode_scan` per pass.
        let mut residual_i16: Vec<i16> = audio
            .iter()
            .map(|&x| (x * 32767.0).clamp(-32768.0, 32767.0) as i16)
            .collect();

        let mut all: Vec<WsprResult> = Vec::new();
        // wsprd uses 3 passes. Our `decode_scan` is expensive (~30 s on
        // a 120-s WSPR slot due to the 2-D refine grid), so we cap at 2
        // — empirically the bulk of the SIC benefit lands on pass 2 once
        // the strong KB0VHA-class signals have been removed.
        const NPASSES: usize = 2;

        for _pass in 0..NPASSES {
            // Re-convert residual back to f32 for decode_scan (it expects
            // unit-scale samples).
            let residual_f32: Vec<f32> =
                residual_i16.iter().map(|&s| s as f32 / 32_768.0).collect();
            let new_decodes = decode_scan_inner(
                &residual_f32,
                sample_rate,
                nominal_start_sample,
                params,
                None,
                None,
                ScanDepth::DEFAULT,
                None,
            )
            .0;
            if new_decodes.is_empty() {
                break;
            }
            let mut added = 0usize;
            for d in new_decodes {
                // Dedup only here: the subtraction below runs before the
                // row is reported and kept, so `push_unique` does not fit.
                if is_scan_dup(&all, &d, FREQ_DEDUP_HZ, TIME_DEDUP_SAMPLES) {
                    continue;
                }
                // Reconstruct the on-air channel symbols (162 4-FSK tones)
                // from the recovered 50-bit info, and subtract from the
                // residual at the decoded (freq, dt). Mirrors wsprd.c:1433-
                // 1437 `subtract_signal2(idat, qdat, …, channel_symbols)`.
                let symbols = super::super::encode_channel_symbols(&d.info_bits);
                // Channel-aware LPF subtract, matching wsprd's
                // `subtract_signal2` (`wsprd.c:549-602`): it estimates a
                // *time-varying* amplitude by low-pass filtering the
                // product of residual and reference, so it tracks QSB and
                // drift across the 110.6 s transmission.
                //
                // This replaces a constant-amplitude least-squares
                // subtract that was measurably not doing its job. On the
                // WSJT-X golden, subtracting W5BIT and measuring the
                // residual at its own four tone frequencies:
                //
                //   const-amplitude   +9.0  -7.7  -5.8  -4.2  dB
                //   LPF (this)       -25.4 -28.6 -30.0 -19.7  dB
                //
                // — the old path *amplified* the first tone by 9 dB and
                // removed only a few dB from the rest, leaving most of
                // the interferer in the residual it was supposed to
                // clean.
                //
                // The comment this replaces said the LPF path was avoided
                // because it "would cost a multi-second convolution on a
                // 1.4 M-sample WSPR slot". That stopped being true when
                // `subtract_tones_lpf` gained its FFT fast path for
                // FT8/FT4 (one forward + inverse FFT per call, O(N log N));
                // WSPR was simply never migrated. Measured cost of the
                // whole `decode_scan_subtract` on that slot is unchanged
                // at ~0.3-0.4 s.
                //
                // `lpf_half = 600` was chosen by measurement, not
                // inherited: 5760 (wsprd's `nfilt=360` scaled from its
                // 375 Hz baseband to 12 kHz) removes 10-15 dB less,
                // because our reference is built at audio rate where a
                // proportionally longer kernel over-smooths the amplitude
                // estimate.
                //
                // `endcorrection = false`: measured identical either way
                // here, and WSPR's transmission is long enough that edge
                // handling is negligible.
                subtract_tones_lpf(
                    &mut residual_i16,
                    &symbols,
                    d.freq_hz,
                    d.dt_sec,
                    &WSPR_SUBTRACT,
                    WSPR_SUBTRACT_LPF_HALF,
                    false,
                );
                if let Some(cb) = on_result {
                    cb(&d);
                }
                all.push(d);
                added += 1;
            }
            if added == 0 {
                break;
            }
        }
        all
    }
}
