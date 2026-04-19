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

use std::collections::HashMap;

/// Base-38 alphabet used for callsign hashing (matches WSJT-X).
const C38: &[u8] = b" 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ/";

/// Magic constant for multiplicative hash (from WSJT-X).
const HASH_MAGIC: u64 = 47_055_833_459;

/// Maximum entries in the 22-bit LRU table.
const MAX_HASH22: usize = 1000;

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

/// Runtime callsign hash lookup table.
///
/// Populated during decoding; used to resolve `<...>` placeholders in
/// messages containing hashed callsigns.
#[derive(Debug, Clone)]
pub struct CallsignHashTable {
    /// 10-bit hash → callsign (direct-indexed, 1024 slots)
    hash10: HashMap<u32, String>,
    /// 12-bit hash → callsign (direct-indexed, 4096 slots)
    hash12: HashMap<u32, String>,
    /// 22-bit hash → callsign (LRU, max 1000 entries)
    hash22: Vec<(u32, String)>,
}

impl CallsignHashTable {
    /// Create an empty hash table.
    pub fn new() -> Self {
        Self {
            hash10: HashMap::new(),
            hash12: HashMap::new(),
            hash22: Vec::new(),
        }
    }

    /// Register a decoded callsign, populating all three hash tables.
    ///
    /// Skips empty strings, `<...>` placeholders, and strings shorter than
    /// 3 characters. Strips `<>` brackets if present.
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

        self.hash10.insert(n10, base.to_string());
        self.hash12.insert(n12, base.to_string());

        // 22-bit: LRU update
        if let Some(pos) = self.hash22.iter().position(|(h, _)| *h == n22) {
            // Move to front
            let entry = self.hash22.remove(pos);
            self.hash22.insert(0, entry);
        } else {
            self.hash22.insert(0, (n22, base.to_string()));
            if self.hash22.len() > MAX_HASH22 {
                self.hash22.pop();
            }
        }
    }

    /// Look up a 10-bit hash. Returns the callsign if found.
    pub fn lookup10(&self, n10: u32) -> Option<&str> {
        self.hash10.get(&n10).map(|s| s.as_str())
    }

    /// Look up a 12-bit hash. Returns the callsign if found.
    pub fn lookup12(&self, n12: u32) -> Option<&str> {
        self.hash12.get(&n12).map(|s| s.as_str())
    }

    /// Look up a 22-bit hash. Returns the callsign wrapped in `<>` if found,
    /// matching WSJT-X convention (e.g. `<JA1ABC>`).
    pub fn lookup22(&self, n22: u32) -> Option<String> {
        self.hash22
            .iter()
            .find(|(h, _)| *h == n22)
            .map(|(_, call)| format!("<{}>", call))
    }

    /// Clear all entries.
    pub fn clear(&mut self) {
        self.hash10.clear();
        self.hash12.clear();
        self.hash22.clear();
    }

    /// Number of entries in the 22-bit table (for diagnostics).
    pub fn len22(&self) -> usize {
        self.hash22.len()
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

    #[test]
    fn insert_and_lookup() {
        let mut t = CallsignHashTable::new();
        t.insert("JA1ABC");
        t.insert("3Y0Z");

        let h22 = ihashcall("JA1ABC", 22);
        let h12 = ihashcall("JA1ABC", 12);
        let h10 = ihashcall("JA1ABC", 10);

        assert_eq!(t.lookup22(h22), Some("<JA1ABC>".to_string()));
        assert_eq!(t.lookup12(h12), Some("JA1ABC"));
        assert_eq!(t.lookup10(h10), Some("JA1ABC"));

        let h22z = ihashcall("3Y0Z", 22);
        assert_eq!(t.lookup22(h22z), Some("<3Y0Z>".to_string()));
    }

    #[test]
    fn lru_eviction() {
        let mut t = CallsignHashTable::new();
        // Fill beyond MAX_HASH22
        for i in 0..MAX_HASH22 + 10 {
            t.insert(&format!("T{:04}X", i));
        }
        assert_eq!(t.len22(), MAX_HASH22);
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
        assert_eq!(t.lookup22(h22), Some("<JA1ABC>".to_string()));
    }
}
