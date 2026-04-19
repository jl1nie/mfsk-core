//! A Priori (AP) hint for WSJT 77-bit message payloads.
//!
//! Known parts of the message (callsigns, grid, report) are converted to
//! their packed bit representation and marked as "locked" so a downstream
//! FEC decoder can clamp those LLRs to a high-confidence value. AP hints
//! typically drop the decode threshold by a few dB when the caller knows
//! the expected message format (e.g. "CQ from a specific DX call", or
//! "RRR/RR73/73 as part of a QSO exchange").
//!
//! The 77-bit bit layout is shared across FT8, FT4, FT2 and FST4 — all WSJT
//! Type-1 messages use the same `call1 / call2 / grid-or-report / i3` field
//! positions — so `ApHint` lives in the protocol-agnostic message layer.

use super::wsjt77::{pack_grid4, pack28};

/// A Priori information to bias decoding.
#[derive(Debug, Clone, Default)]
pub struct ApHint {
    /// Known first callsign (e.g. "CQ", "JA1ABC"). Locks message bits 0–28.
    pub call1: Option<String>,
    /// Known second callsign (e.g. "3Y0Z"). Locks message bits 29–57.
    pub call2: Option<String>,
    /// Known grid locator (e.g. "PM95"). Locks bits 58–73.
    pub grid: Option<String>,
    /// Known response token: "RRR", "RR73", or "73". Locks bits 58–73.
    pub report: Option<String>,
}

impl ApHint {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn with_call1(mut self, call: &str) -> Self {
        self.call1 = Some(call.to_string());
        self
    }
    pub fn with_call2(mut self, call: &str) -> Self {
        self.call2 = Some(call.to_string());
        self
    }
    pub fn with_grid(mut self, grid: &str) -> Self {
        self.grid = Some(grid.to_string());
        self
    }
    pub fn with_report(mut self, rpt: &str) -> Self {
        self.report = Some(rpt.to_string());
        self
    }

    /// True if any AP field is populated.
    pub fn has_info(&self) -> bool {
        self.call1.is_some() || self.call2.is_some()
    }

    /// Build the `(mask, bit_values)` bit vectors of length `n_codeword` for
    /// a downstream FEC codec. Bits 0–76 (the message payload) are populated
    /// from the hint fields; bits 77..N are left unmasked.
    ///
    /// `mask[i] == 1` means bit `i` is AP-locked; `values[i]` is the target
    /// bit value (0 or 1). The FEC codec clamps its LLR at these positions
    /// to `±apmag` accordingly.
    pub fn build_bits(&self, n_codeword: usize) -> (Vec<u8>, Vec<u8>) {
        let mut mask = vec![0u8; n_codeword];
        let mut values = vec![0u8; n_codeword];

        // Write 28-bit packed call + 1-bit flag (=0) starting at `start`.
        let mut set_call_bits = |call: &str, start: usize| {
            if let Some(n28) = pack28(call) {
                for i in 0..28 {
                    let bit = ((n28 >> (27 - i)) & 1) as u8;
                    mask[start + i] = 1;
                    values[start + i] = bit;
                }
                // Flag bit (ipa/ipb) = 0 for standard calls.
                mask[start + 28] = 1;
                values[start + 28] = 0;
            }
        };

        if let Some(ref c1) = self.call1 {
            set_call_bits(c1, 0);
        }
        if let Some(ref c2) = self.call2 {
            set_call_bits(c2, 29);
        }

        // Bits 58–73: grid or response field (15-bit value + 1-bit ir flag).
        if let Some(ref grid) = self.grid
            && let Some(igrid) = pack_grid4(grid)
        {
            mask[58] = 1;
            values[58] = 0; // ir=0
            for i in 0..15 {
                let bit = ((igrid >> (14 - i)) & 1) as u8;
                mask[59 + i] = 1;
                values[59 + i] = bit;
            }
        }
        if let Some(ref rpt) = self.report {
            let igrid_val: Option<u32> = match rpt.as_str() {
                "RRR" => Some(32_400 + 2),
                "RR73" => Some(32_400 + 3),
                "73" => Some(32_400 + 4),
                _ => None,
            };
            if let Some(igrid) = igrid_val {
                mask[58] = 1;
                values[58] = 0;
                for i in 0..15 {
                    let bit = ((igrid >> (14 - i)) & 1) as u8;
                    mask[59 + i] = 1;
                    values[59 + i] = bit;
                }
            }
        }

        // Lock message type i3 = 001 (Type 1 standard) when any call known.
        if self.has_info() {
            mask[74] = 1;
            values[74] = 0;
            mask[75] = 1;
            values[75] = 0;
            mask[76] = 1;
            values[76] = 1;
        }

        (mask, values)
    }

    /// Number of AP-locked message bits (informational; callers use it to
    /// scale per-pass confidence thresholds).
    pub fn locked_bits(&self, n_codeword: usize) -> usize {
        let (mask, _) = self.build_bits(n_codeword);
        mask.iter().filter(|&&m| m != 0).count()
    }
}
