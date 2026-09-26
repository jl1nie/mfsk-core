// SPDX-License-Identifier: GPL-3.0-or-later
//! FT8 callsign hash table for resolving `<...>` placeholders.
//!
//! Ported from WSJT-X `lib/77bit/packjt77.f90` (`ihashcall`,
//! `save_hash_call`, `hash10`, `hash12`, `hash22`).
//!
//! Three hash widths are used in FT8 messages:
//! - **22-bit** — packed inside a 28-bit callsign token (Type 1 messages)
//! - **12-bit** — Type 4 messages (one non-standard call)
//! - **10-bit** — DXpedition RR73 messages (Type 0, n3=1)
//!
//! The table is populated as callsigns are decoded and used to resolve
//! hashed callsigns in subsequent messages.

use alloc::vec::Vec;

/// Base-38 alphabet used for callsign hashing (matches WSJT-X).
const C38: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";

/// Magic constant for multiplicative hash (from WSJT-X).
const HASH_MAGIC: u64 = 47_055_833_459;

/// Compute the FT8 callsign hash at a given bit width.
///
/// The callsign is left-padded to 11 characters, converted to a base-38
/// number, multiplied by a magic constant, then the top `m` bits are
/// extracted.
///
/// # Arguments
/// * `call` — callsign (up to 11 chars, will be uppercased and padded)
/// * `m` — bit width: 10, 12, or 22
pub fn ihashcall(call: &str, m: u32) -> u32 {
    let call = call.to_ascii_uppercase();
    let bytes = call.as_bytes();

    let mut n64: u64 = 0;
    for i in 0..11 {
        let c = if i < bytes.len() { bytes[i] } else { b' ' };
        let j = C38.iter().position(|&x| x == c).unwrap_or(0);
        n64 = n64.wrapping_mul(38).wrapping_add(j as u64);
    }

    let hash64 = n64.wrapping_mul(HASH_MAGIC);
    (hash64 >> (64 - m)) as u32
}

/// One stored callsign, inline — WSJT-X's `character*13`.
///
/// `packjt77.f90:5-7` declares its three tables as
/// `character(len=13), dimension(...)`: the callsign text lives *in*
/// the array, not behind a pointer. This is the same thing, and the
/// reason it matters is allocation shape rather than size. The port
/// this replaces stored `String`s in two `BTreeMap`s and a `Vec`, so
/// every callsign learned cost three heap allocations plus map nodes
/// — about 130 B, measured on air 2026-09-20 — and every one of them
/// was small. On a target whose allocator sends sub-4 KB requests to
/// internal DRAM (the CoreS3's
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`), "small" means the
/// scarce pool: internal DRAM fell from 10.7 kB to 3.4 kB over 22
/// minutes of live reception until `esp-aes` could not allocate and
/// WiFi — the only console a USB-host-mode board has — went silent.
///
/// Inline entries make the whole table a few large allocations
/// instead of thousands of tiny ones — three by default, one under
/// `hash-table-small` — which on that board puts it in PSRAM where
/// there are megabytes spare.
///
/// Zero is the empty marker: every character `ihashcall` accepts is
/// printable, so a leading NUL cannot be a callsign.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Call13([u8; 13]);

impl Call13 {
    const EMPTY: Self = Self([0; 13]);

    fn from_str(s: &str) -> Self {
        let mut out = [0u8; 13];
        let b = s.as_bytes();
        let n = b.len().min(13);
        out[..n].copy_from_slice(&b[..n]);
        Self(out)
    }

    fn is_empty(&self) -> bool {
        self.0[0] == 0
    }

    fn as_str(&self) -> &str {
        let n = self.0.iter().position(|&c| c == 0).unwrap_or(13);
        // Written only from `&str`, so the bytes are valid UTF-8.
        core::str::from_utf8(&self.0[..n]).unwrap_or("")
    }
}

impl core::fmt::Debug for Call13 {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self.as_str())
    }
}

/// Maximum entries in the 22-bit LRU table (`packjt77.f90:4`,
/// `MAXHASH=1000`). Host only — under `hash-table-small` there is one
/// unified table instead and this does not exist.
#[cfg(not(feature = "hash-table-small"))]
const MAX_HASH22: usize = 1000;

/// Slots in the 10-bit table — the whole key space, as upstream
/// (`packjt77.f90:5`, `dimension(0:1023)`). 13 KB once allocated.
#[cfg(not(feature = "hash-table-small"))]
const N_HASH10: usize = 1024;

/// Slots in the 12-bit table (`packjt77.f90:6`, `dimension(0:4095)`).
/// 53 KB once allocated, and the reason allocation is lazy.
#[cfg(not(feature = "hash-table-small"))]
const N_HASH12: usize = 4096;

/// Entries in the unified table `hash-table-small` uses.
///
/// **256 and not 100, because of an allocator threshold.** One entry
/// is 28 B (a `Call13` padded to 16 by the three `u32` hashes beside
/// it), so 100 entries is 2.8 kB — under the CoreS3's
/// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`, which would put the
/// whole point of this rewrite back in internal DRAM. 256 entries is
/// 7.2 kB, over the line and into PSRAM, and still a quarter of
/// upstream's 1 000.
///
/// As history: a CoreS3 on 40 m logged 100 distinct callsigns in its
/// first 15 minutes (2026-09-20), so 256 is roughly 40 minutes of
/// memory.
#[cfg(feature = "hash-table-small")]
const N_ENTRIES: usize = 256;

/// One learned callsign and the three hashes it answers to.
///
/// `hash-table-small` only. Upstream keeps three separate tables
/// because on a PC it can: the 10- and 12-bit ones are arrays indexed
/// by the hash itself, 1 024 and 4 096 slots, needing no search and no
/// stored key. That is 66 KB to hold what a receiver actually sees —
/// 100 distinct callsigns in 15 minutes of a busy band — so 97 % of it
/// is empty. Storing the callsign once with its three hashes and
/// scanning for the one wanted is 7 KB for the same job.
///
/// The cost is eviction policy. A direct-indexed slot keeps its
/// callsign until a colliding hash overwrites it, which may be never;
/// here all three widths share one LRU, so a station heard long enough
/// ago renders `<...>` again. The 10- and 12-bit hashes appear in
/// DXpedition and Type-4 messages, both of which name the station
/// being worked *now*, so the depth that matters is recent — but the
/// rate at which this costs a resolution has not been measured on air.
/// Count `<...>` per decode before and after, the way
/// `decode_pipeline.rs` already does.
#[cfg(feature = "hash-table-small")]
#[derive(Clone, Copy, Debug)]
struct Entry {
    call: Call13,
    h10: u32,
    h12: u32,
    h22: u32,
}

/// Runtime callsign hash lookup table.
///
/// Populated during decoding; used to resolve `<...>` placeholders in
/// messages containing hashed callsigns.
///
/// **`new()` allocates nothing, and that is load-bearing.**
/// `wsjt77::is_plausible_payload` builds a fresh empty table on every
/// call, so an eager allocation here would land on every `unpack77`.
/// The storage appears on the first [`Self::insert`] and not before.
///
/// Two shapes, chosen at compile time. The default is upstream's —
/// three tables, the 10- and 12-bit ones direct-indexed over their key
/// spaces (83 KB). `hash-table-small` is one LRU of `N_ENTRIES`
/// holding each callsign once beside its three hashes (7 KB); see
/// `Entry` for what that trades. (Plain spans and not intra-doc
/// links: both items are `#[cfg]`-ed away in the build that renders
/// these docs.)
#[derive(Debug, Clone)]
pub struct CallsignHashTable {
    /// 10-bit hash → callsign, direct-indexed over the whole key
    /// space. `None` until the first insert.
    #[cfg(not(feature = "hash-table-small"))]
    calls10: Option<Vec<Call13>>,
    /// 12-bit hash → callsign, direct-indexed. `None` until the first
    /// insert.
    #[cfg(not(feature = "hash-table-small"))]
    calls12: Option<Vec<Call13>>,
    /// The 22-bit LRU: `hash22[i]` names `calls22[i]`, most recent
    /// first, exactly as `packjt77.f90:7,12`'s parallel arrays do.
    #[cfg(not(feature = "hash-table-small"))]
    calls22: Option<Vec<Call13>>,
    #[cfg(not(feature = "hash-table-small"))]
    hash22: Option<Vec<u32>>,
    /// The unified LRU, most recent first.
    #[cfg(feature = "hash-table-small")]
    entries: Option<Vec<Entry>>,
    /// Live entries — upstream's `nzhash`.
    n22: usize,
}

impl CallsignHashTable {
    /// Create an empty hash table. Allocates nothing.
    pub fn new() -> Self {
        Self {
            #[cfg(not(feature = "hash-table-small"))]
            calls10: None,
            #[cfg(not(feature = "hash-table-small"))]
            calls12: None,
            #[cfg(not(feature = "hash-table-small"))]
            calls22: None,
            #[cfg(not(feature = "hash-table-small"))]
            hash22: None,
            #[cfg(feature = "hash-table-small")]
            entries: None,
            n22: 0,
        }
    }

    /// Register a decoded callsign.
    ///
    /// Skips empty strings, `<...>` placeholders, and strings shorter
    /// than 2 characters. Strips `<>` brackets if present.
    ///
    /// (Upstream's `save_hash_call` requires 3, not 2
    /// — `packjt77.f90:91`, `if(len(trim(cw)) .lt. 3) return`. The
    /// port has always used 2; left as it was, because changing an
    /// acceptance rule is not what this rewrite is for.)
    pub fn insert(&mut self, call: &str) {
        let call = call.trim();
        // Strip angle brackets
        let call = call.strip_prefix('<').unwrap_or(call);
        let call = call.strip_suffix('>').unwrap_or(call);
        // Strip /R or /P suffix for hashing
        let base = if call.ends_with("/R") || call.ends_with("/P") {
            &call[..call.len() - 2]
        } else {
            call
        };

        if base.len() < 2 || base == "..." || base.starts_with("CQ") {
            return;
        }

        let n10 = ihashcall(base, 10);
        let n12 = ihashcall(base, 12);
        let n22 = ihashcall(base, 22);
        let entry = Call13::from_str(base);
        let _ = (n10, n12, entry);

        #[cfg(not(feature = "hash-table-small"))]
        {
            // `vec![Call13::EMPTY; N]` and not `Box::new([_; N])`: the
            // latter builds the array as a temporary first, and 53 KB
            // of temporary is an embedded task's whole stack. Two of
            // this project's "heap corruption" investigations turned
            // out to be stack overflows (`embedded-poc/CLAUDE.md`,
            // "Stacks, heaps, and the space between them").
            if self.calls10.is_none() {
                self.calls10 = Some(alloc::vec![Call13::EMPTY; N_HASH10]);
                self.calls12 = Some(alloc::vec![Call13::EMPTY; N_HASH12]);
                self.calls22 = Some(alloc::vec![Call13::EMPTY; MAX_HASH22]);
                self.hash22 = Some(alloc::vec![0u32; MAX_HASH22]);
            }
            // `ihashcall` returns the top `m` bits, so these are in
            // range by construction; the guard mirrors
            // `packjt77.f90:94,97` rather than trusting that across a
            // future change.
            if let Some(t) = self.calls10.as_mut()
                && let Some(slot) = t.get_mut(n10 as usize)
            {
                *slot = entry;
            }
            if let Some(t) = self.calls12.as_mut()
                && let Some(slot) = t.get_mut(n12 as usize)
            {
                *slot = entry;
            }
            // 22-bit LRU, `packjt77.f90:99-112`: refresh in place if
            // the hash is already known, else push everything down and
            // take the front.
            let (Some(hashes), Some(calls)) = (self.hash22.as_mut(), self.calls22.as_mut()) else {
                return;
            };
            if let Some(pos) = hashes[..self.n22].iter().position(|&h| h == n22) {
                calls[pos] = entry;
                hashes[..=pos].rotate_right(1);
                calls[..=pos].rotate_right(1);
                return;
            }
            if self.n22 < MAX_HASH22 {
                self.n22 += 1;
            }
            hashes[..self.n22].rotate_right(1);
            calls[..self.n22].rotate_right(1);
            hashes[0] = n22;
            calls[0] = entry;
        }

        #[cfg(feature = "hash-table-small")]
        {
            let t = self.entries.get_or_insert_with(|| {
                alloc::vec![
                    Entry {
                        call: Call13::EMPTY,
                        h10: 0,
                        h12: 0,
                        h22: 0,
                    };
                    N_ENTRIES
                ]
            });
            let new = Entry {
                call: entry,
                h10: n10,
                h12: n12,
                h22: n22,
            };
            // Same LRU discipline as the 22-bit table above, keyed on
            // the 22-bit hash: it is the widest, so two callsigns
            // sharing it is 1-in-4M rather than 1-in-1024.
            if let Some(pos) = t[..self.n22].iter().position(|e| e.h22 == n22) {
                t[pos] = new;
                t[..=pos].rotate_right(1);
                return;
            }
            if self.n22 < N_ENTRIES {
                self.n22 += 1;
            }
            t[..self.n22].rotate_right(1);
            t[0] = new;
        }
    }

    /// Look up a 10-bit hash. Returns the callsign if found.
    pub fn lookup10(&self, n10: u32) -> Option<&str> {
        #[cfg(not(feature = "hash-table-small"))]
        {
            let e = self.calls10.as_ref()?.get(n10 as usize)?;
            (!e.is_empty()).then(|| e.as_str())
        }
        #[cfg(feature = "hash-table-small")]
        {
            self.scan(|e| e.h10 == n10)
        }
    }

    /// Look up a 12-bit hash. Returns the callsign if found.
    pub fn lookup12(&self, n12: u32) -> Option<&str> {
        #[cfg(not(feature = "hash-table-small"))]
        {
            let e = self.calls12.as_ref()?.get(n12 as usize)?;
            (!e.is_empty()).then(|| e.as_str())
        }
        #[cfg(feature = "hash-table-small")]
        {
            self.scan(|e| e.h12 == n12)
        }
    }

    /// Look up a 22-bit hash. Returns the callsign, **unwrapped**.
    ///
    /// It used to return `<CALL>` while [`Self::lookup10`] and
    /// [`Self::lookup12`] returned the bare callsign — an asymmetry
    /// nothing announced, and one that cost a double-wrapped
    /// `<<PA3XYZ>>` during the issue #383 type-5 port before a test
    /// caught it. All three return the same shape now; the callers
    /// that want `<>` add it.
    pub fn lookup22(&self, n22: u32) -> Option<&str> {
        #[cfg(not(feature = "hash-table-small"))]
        {
            let hashes = self.hash22.as_ref()?;
            let pos = hashes[..self.n22].iter().position(|&h| h == n22)?;
            Some(self.calls22.as_ref()?[pos].as_str())
        }
        #[cfg(feature = "hash-table-small")]
        {
            self.scan(|e| e.h22 == n22)
        }
    }

    /// Most-recent-first scan of the unified table.
    ///
    /// Linear, and that is the whole point: at [`N_ENTRIES`] the scan
    /// is a few hundred `u32` compares against a decode that has just
    /// spent milliseconds in BP, and it buys back 76 KB.
    #[cfg(feature = "hash-table-small")]
    fn scan(&self, pred: impl Fn(&Entry) -> bool) -> Option<&str> {
        let t = self.entries.as_ref()?;
        t[..self.n22]
            .iter()
            .find(|e| !e.call.is_empty() && pred(e))
            .map(|e| e.call.as_str())
    }

    /// Clear all entries, keeping the blocks for reuse.
    pub fn clear(&mut self) {
        #[cfg(not(feature = "hash-table-small"))]
        for t in [
            self.calls10.as_mut(),
            self.calls12.as_mut(),
            self.calls22.as_mut(),
        ]
        .into_iter()
        .flatten()
        {
            t.fill(Call13::EMPTY);
        }
        #[cfg(feature = "hash-table-small")]
        if let Some(t) = self.entries.as_mut() {
            for e in t.iter_mut() {
                e.call = Call13::EMPTY;
            }
        }
        self.n22 = 0;
    }

    /// Distinct callsigns currently held, out of [`Self::capacity22`].
    ///
    /// The only count here that means what it says. The default
    /// build's 10- and 12-bit tables are indexed by the hash, so a
    /// collision overwrites and their length would count *occupied
    /// slots* rather than stations — 39 and 41 against 42 real
    /// callsigns, measured on air 2026-09-20. Neither is exposed.
    pub fn len22(&self) -> usize {
        self.n22
    }

    /// Callsigns held before the LRU starts evicting.
    ///
    /// Build-dependent — upstream's `MAXHASH` by default, smaller
    /// under `hash-table-small` — so a caller reporting occupancy
    /// asks rather than naming a number. `len22() == capacity22()`
    /// with unresolved hashes still appearing is the signal that the
    /// cap is too low for the traffic.
    pub fn capacity22(&self) -> usize {
        #[cfg(not(feature = "hash-table-small"))]
        {
            MAX_HASH22
        }
        #[cfg(feature = "hash-table-small")]
        {
            N_ENTRIES
        }
    }
}

impl Default for CallsignHashTable {
    fn default() -> Self {
        Self::new()
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// `ihashcall` against WSJT-X's own, value for value. `packjt77.f90` (3.2.0-rc1)
    /// replaced `ishft(47055833459_8*n8, m-64)` by `ihashcall_from_n8`, 16-bit limbs that
    /// return "the top `m` bits of the low 64 bits of `47055833459*n8`" without the signed
    /// 64-bit overflow the old form risked (#441). This crate's `wrapping_mul` is that same
    /// value, and the test is what says so: the vectors are `ihashcall(c, 10)`, `(c, 12)`
    /// and `(c, 22)` from that Fortran (gfortran, the two functions lifted out of
    /// `packjt77.f90`), for callsigns whose base-38 value times the constant overflows
    /// `i64` (nearly all of them: the product is up to about 7.7e27) and the extremes.
    #[test]
    fn ihashcall_matches_wsjtx_3_2() {
        const VECTORS: &[(&str, u32, u32, u32)] = &[
            ("K1ABC", 712, 2851, 2920267),
            ("W9XYZ", 972, 3889, 3982604),
            ("JA1ABC", 1, 6, 6274),
            ("PJ4/K1ABC", 346, 1387, 1420834),
            ("3DA0XYZ", 483, 1932, 1979256),
            ("VK3NV", 471, 1884, 1929750),
            ("JL1NIE", 17, 71, 73715),
            ("3Y0Z", 573, 2294, 2349372),
            ("K1JT", 511, 2047, 2096289),
            ("YC1MRF", 608, 2434, 2492464),
            ("DL8YHR", 965, 3861, 3953752),
            ("ZL4ZZZ", 371, 1485, 1521353),
            ("A61AB", 582, 2329, 2385011),
            ("9A1A/P", 914, 3656, 3744151),
            ("HB9CQK", 602, 2409, 2466836),
            ("EA6VQ", 802, 3209, 3286080),
            ("XE2X", 538, 2153, 2205628),
            ("TF/G4ABC", 817, 3269, 3347813),
            ("ZZZZZZZZ/ZZ", 902, 3609, 3695734),
            ("///////////", 671, 2685, 2749801),
            ("0", 179, 717, 734621),
            ("Z", 312, 1250, 1280558),
            ("ZZZZZZZZZZZ", 902, 3609, 3695718),
            ("9Z9Z9Z9Z9Z9", 950, 3800, 3891230),
        ];
        for &(call, h10, h12, h22) in VECTORS {
            assert_eq!(ihashcall(call, 10), h10, "{call} at 10 bits");
            assert_eq!(ihashcall(call, 12), h12, "{call} at 12 bits");
            assert_eq!(ihashcall(call, 22), h22, "{call} at 22 bits");
        }
    }

    /// How many callsigns the table holds before evicting. The two
    /// builds store them differently — see [`CallsignHashTable`] — so
    /// the shared tests below ask for the cap rather than naming one.
    #[cfg(not(feature = "hash-table-small"))]
    const CAP: usize = MAX_HASH22;
    #[cfg(feature = "hash-table-small")]
    const CAP: usize = N_ENTRIES;

    /// The `i`-th most recently learned callsign.
    fn nth(t: &CallsignHashTable, i: usize) -> &str {
        #[cfg(not(feature = "hash-table-small"))]
        {
            t.calls22.as_ref().unwrap()[i].as_str()
        }
        #[cfg(feature = "hash-table-small")]
        {
            t.entries.as_ref().unwrap()[i].call.as_str()
        }
    }

    /// Whether any storage has been allocated yet.
    fn allocated(t: &CallsignHashTable) -> bool {
        #[cfg(not(feature = "hash-table-small"))]
        {
            t.calls10.is_some()
        }
        #[cfg(feature = "hash-table-small")]
        {
            t.entries.is_some()
        }
    }

    #[test]
    fn hash_basic() {
        // Verify hash values are deterministic and non-zero
        let h22 = ihashcall("JA1ABC", 22);
        let h12 = ihashcall("JA1ABC", 12);
        let h10 = ihashcall("JA1ABC", 10);
        assert!(h22 < (1 << 22));
        assert!(h12 < (1 << 12));
        assert!(h10 < (1 << 10));
        // Same input → same output
        assert_eq!(h22, ihashcall("JA1ABC", 22));
    }

    /// `new()` must allocate nothing — `wsjt77::is_plausible_payload`
    /// builds one per `unpack77` call, and the storage is 83 KB by
    /// default.
    #[test]
    fn an_empty_table_holds_no_blocks_and_answers_nothing() {
        let t = CallsignHashTable::new();
        assert!(!allocated(&t), "new() must not allocate");
        assert_eq!(t.lookup10(0), None);
        assert_eq!(t.lookup12(0), None);
        assert_eq!(t.lookup22(0), None);
        assert_eq!(t.len22(), 0);
    }

    /// `packjt77.f90:99-112` — a repeat moves its entry to the front
    /// and does not grow the table; a new one pushes everything down.
    #[test]
    fn the_lru_orders_by_recency_like_upstream() {
        let mut t = CallsignHashTable::new();
        for c in ["JA1ABC", "3Y0Z", "W1AW"] {
            t.insert(c);
        }
        assert_eq!(t.len22(), 3);
        // Most recent first.
        assert_eq!(nth(&t, 0), "W1AW");
        assert_eq!(nth(&t, 2), "JA1ABC");

        // Re-inserting the oldest refreshes it in place, no growth.
        t.insert("JA1ABC");
        assert_eq!(t.len22(), 3);
        assert_eq!(nth(&t, 0), "JA1ABC");
        assert_eq!(nth(&t, 2), "3Y0Z");
        // And every one of them still resolves.
        for c in ["JA1ABC", "3Y0Z", "W1AW"] {
            assert_eq!(t.lookup22(ihashcall(c, 22)), Some(c), "{c}");
        }
    }

    /// The LRU is capped; by default the 10-/12-bit tables are bounded
    /// by their key spaces instead, a collision overwriting. Either
    /// way nothing grows without limit, which is the property that
    /// matters on a board with 10 kB of internal DRAM to spare.
    #[test]
    fn the_tables_stay_within_their_bounds() {
        let mut t = CallsignHashTable::new();
        for i in 0..(CAP + 50) {
            t.insert(&alloc::format!("A{i}BC"));
        }
        assert_eq!(t.len22(), CAP);
        #[cfg(not(feature = "hash-table-small"))]
        {
            assert_eq!(t.calls22.as_ref().unwrap().len(), MAX_HASH22);
            assert_eq!(t.calls10.as_ref().unwrap().len(), N_HASH10);
            assert_eq!(t.calls12.as_ref().unwrap().len(), N_HASH12);
        }
        #[cfg(feature = "hash-table-small")]
        assert_eq!(t.entries.as_ref().unwrap().len(), N_ENTRIES);
    }

    /// The whole point of `hash-table-small`: one entry answers all
    /// three widths, and eviction is therefore shared.
    ///
    /// The default build cannot assert the second half — its 10- and
    /// 12-bit tables are direct-indexed, so a callsign pushed out of
    /// the 22-bit LRU still resolves there until something collides
    /// with it. That difference is the trade this feature makes, and
    /// it is worth having written down as an assertion rather than as
    /// a sentence in a doc comment.
    #[cfg(feature = "hash-table-small")]
    #[test]
    fn the_unified_table_shares_one_entry_across_all_three_widths() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC");
        assert_eq!(t.entries.as_ref().unwrap().len(), N_ENTRIES);
        assert_eq!(t.lookup10(ihashcall("JA1ABC", 10)), Some("JA1ABC"));
        assert_eq!(t.lookup12(ihashcall("JA1ABC", 12)), Some("JA1ABC"));
        assert_eq!(t.lookup22(ihashcall("JA1ABC", 22)), Some("JA1ABC"));

        // Push it past the end of the LRU; all three widths forget it
        // together.
        for i in 0..N_ENTRIES {
            t.insert(&alloc::format!("A{i}BC"));
        }
        assert_eq!(t.lookup22(ihashcall("JA1ABC", 22)), None);
        assert_eq!(t.lookup12(ihashcall("JA1ABC", 12)), None);
        assert_eq!(t.lookup10(ihashcall("JA1ABC", 10)), None);
    }

    /// 256 entries of 28 B is 7.2 kB — deliberately over the CoreS3's
    /// `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`, so the one
    /// allocation lands in PSRAM instead of the internal DRAM this
    /// rewrite exists to stop consuming. Shrinking `N_ENTRIES` below
    /// ~147 would silently undo that, so the threshold is an
    /// assertion.
    #[cfg(feature = "hash-table-small")]
    #[test]
    fn the_unified_table_stays_above_the_psram_threshold() {
        let bytes = N_ENTRIES * core::mem::size_of::<Entry>();
        assert!(
            bytes > 4096,
            "{N_ENTRIES} x {} B = {bytes} B would land in internal DRAM",
            core::mem::size_of::<Entry>()
        );
    }

    /// A 13-character callsign is the longest `character*13` holds, and
    /// it has to come back whole.
    #[test]
    fn a_full_width_callsign_round_trips() {
        let long = "ABCDEFGHIJKLM";
        assert_eq!(long.len(), 13);
        let mut t = CallsignHashTable::new();
        t.insert(long);
        assert_eq!(t.lookup22(ihashcall(long, 22)), Some(long));
    }

    /// `clear` keeps the blocks — it is called between sessions, not
    /// between messages, and re-allocating 83 KB to answer `None` is
    /// not what it is for.
    #[test]
    fn clear_empties_without_releasing_the_blocks() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC");
        t.clear();
        assert_eq!(t.len22(), 0);
        assert_eq!(t.lookup22(ihashcall("JA1ABC", 22)), None);
        assert_eq!(t.lookup10(ihashcall("JA1ABC", 10)), None);
        assert!(allocated(&t), "blocks are kept for reuse");
    }

    #[test]
    fn insert_and_lookup() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC");
        t.insert("3Y0Z");

        let h22 = ihashcall("JA1ABC", 22);
        let h12 = ihashcall("JA1ABC", 12);
        let h10 = ihashcall("JA1ABC", 10);

        assert_eq!(t.lookup22(h22), Some("JA1ABC"));
        assert_eq!(t.lookup12(h12), Some("JA1ABC"));
        assert_eq!(t.lookup10(h10), Some("JA1ABC"));

        let h22z = ihashcall("3Y0Z", 22);
        assert_eq!(t.lookup22(h22z), Some("3Y0Z"));
    }

    #[test]
    fn lru_eviction() {
        let mut t = CallsignHashTable::new();
        // Fill beyond the cap
        for i in 0..CAP + 10 {
            t.insert(&format!("T{:04}X", i));
        }
        assert_eq!(t.len22(), CAP);
    }

    #[test]
    fn skip_special() {
        let mut t = CallsignHashTable::new();
        t.insert("<...>");
        t.insert("CQ");
        t.insert("CQ DX");
        t.insert("");
        t.insert("A"); // too short
        assert_eq!(t.len22(), 0);
    }

    #[test]
    fn strip_suffix() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC/P");
        let h22 = ihashcall("JA1ABC", 22);
        assert_eq!(t.lookup22(h22), Some("JA1ABC"));
    }
}
