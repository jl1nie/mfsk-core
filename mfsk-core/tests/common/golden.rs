// SPDX-License-Identifier: GPL-3.0-or-later
//! Tier-B golden assertions: recall **and** precision, together.
//!
//! ## Why these are one function and not two
//!
//! Nearly every golden test in this suite asserted only recall — "each
//! expected message was decoded" — and said nothing about what else
//! came out. That is half a test, and the missing half is the one that
//! catches the failure users actually notice.
//!
//! The case that proved it (2026-08-11): WSPR's
//! `wspr_wsjtx_sample_recall_vs_golden` reported **8/8** on
//! `150426_0918.wav` while `decode_scan` was simultaneously emitting
//! **8 phantom decodes** on the same audio — callsigns like
//! `UZC/7D0DKY` and `05S/C30EQG` invented out of noise, a 50 % false
//! decode rate. Real `wsprd` reports 9 real and 0 phantom on that
//! file. The test was green throughout, because nothing asked it the
//! other question.
//!
//! So [`assert_golden`] takes both bounds at once and there is no
//! recall-only entry point. Adding a golden test for a new protocol
//! forces a decision about its phantom budget.
//!
//! ## The three properties
//!
//! - **recall** — at least `min_hits` of `expected` must decode.
//!   A floor rather than equality, because some goldens carry entries
//!   this crate is known not to reach yet; state the number instead of
//!   quietly dropping the entry.
//! - **precision** — at most `max_extra` decodes outside `expected`.
//!   `0` is the target. A non-zero budget is a documented debt, not a
//!   default.
//! - **SNR accuracy** — for entries carrying `snr_db`, the reported
//!   value must be within `snr_tol_db` of the reference decoder's.
//!
//! ## What this does not cover
//!
//! Bit-exact parity (streaming == batch), roundtrips and TX waveform
//! properties are tier A — they need no corpus and live in their own
//! tests. Sensitivity curves are tier C and do not run in CI at all.
//! See `CONTRIBUTING.md`'s "Testing philosophy".

/// One expected decode from a reference decoder (`jt9`, `wsprd`,
/// WSJT-X itself) run over the same recording.
#[derive(Debug, Clone)]
pub struct GoldenEntry {
    /// Message text exactly as this crate renders it.
    pub msg: &'static str,
    /// Audio carrier in Hz, or `None` to skip the frequency check.
    pub freq_hz: Option<f32>,
    /// Reference `dt`, or `None` to skip the timing check.
    pub dt_sec: Option<f32>,
    /// Reference decoder's reported SNR, or `None` if this protocol
    /// has no verified SNR reference yet.
    pub snr_db: Option<f32>,
}

impl GoldenEntry {
    /// Message-only entry — no freq/dt/SNR checks.
    #[allow(dead_code)]
    pub const fn msg(msg: &'static str) -> Self {
        Self {
            msg,
            freq_hz: None,
            dt_sec: None,
            snr_db: None,
        }
    }

    #[allow(dead_code)]
    pub const fn at(mut self, freq_hz: f32, dt_sec: f32) -> Self {
        self.freq_hz = Some(freq_hz);
        self.dt_sec = Some(dt_sec);
        self
    }

    #[allow(dead_code)]
    pub const fn snr(mut self, snr_db: f32) -> Self {
        self.snr_db = Some(snr_db);
        self
    }
}

/// A protocol's expectation for one recording.
#[derive(Debug, Clone)]
pub struct GoldenSet {
    pub name: &'static str,
    pub expected: &'static [GoldenEntry],
    /// Minimum of `expected` that must decode. Use
    /// `expected.len()` unless a known gap is being tracked, and say
    /// which entry is missing and why in the caller's doc comment.
    pub min_hits: usize,
    /// Maximum decodes allowed **outside** `expected`. Target 0.
    pub max_extra: usize,
}

/// Tolerances for the per-entry checks.
#[derive(Debug, Clone, Copy)]
pub struct Tolerances {
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: f32,
}

impl Default for Tolerances {
    fn default() -> Self {
        Self {
            freq_hz: 4.0,
            dt_sec: 0.5,
            snr_db: 4.0,
        }
    }
}

/// A decode, flattened so `assert_golden` stays protocol-agnostic.
#[derive(Debug, Clone)]
pub struct DecodeView {
    pub msg: String,
    pub freq_hz: f32,
    pub dt_sec: f32,
    pub snr_db: Option<f32>,
}

/// Assert recall, precision and SNR accuracy for one recording.
///
/// `view` flattens the protocol's own result type; every protocol has
/// a different `DecodeResult`, and threading a trait through the test
/// suite would buy nothing over a closure.
///
/// Panics with a message that names the specific offenders — missing
/// entries by text, phantoms by text and frequency — because "recall
/// 7/8" alone has repeatedly cost debugging time.
#[allow(dead_code)]
pub fn assert_golden<T>(
    decodes: &[T],
    set: &GoldenSet,
    tol: Tolerances,
    view: impl Fn(&T) -> DecodeView,
) {
    let views: Vec<DecodeView> = decodes.iter().map(&view).collect();

    // ── recall ────────────────────────────────────────────────────
    let mut missing: Vec<&str> = Vec::new();
    let mut hits = 0usize;
    for g in set.expected {
        let hit = views.iter().any(|d| {
            d.msg == g.msg
                && g.freq_hz
                    .is_none_or(|f| (d.freq_hz - f).abs() <= tol.freq_hz)
                && g.dt_sec.is_none_or(|t| (d.dt_sec - t).abs() <= tol.dt_sec)
        });
        if hit {
            hits += 1;
        } else {
            missing.push(g.msg);
        }
    }

    // ── precision ─────────────────────────────────────────────────
    let phantoms: Vec<String> = views
        .iter()
        .filter(|d| !set.expected.iter().any(|g| g.msg == d.msg))
        .map(|d| format!("{:.0} Hz {:?}", d.freq_hz, d.msg))
        .collect();

    // ── SNR accuracy ──────────────────────────────────────────────
    let mut snr_errors: Vec<String> = Vec::new();
    for g in set.expected {
        let (Some(ref_snr), Some(d)) = (g.snr_db, views.iter().find(|d| d.msg == g.msg)) else {
            continue;
        };
        let Some(got) = d.snr_db else { continue };
        if (got - ref_snr).abs() > tol.snr_db {
            snr_errors.push(format!(
                "{:?}: reported {got:+.1} dB vs reference {ref_snr:+.1} dB \
                 (err {:+.1}, tol ±{})",
                g.msg,
                got - ref_snr,
                tol.snr_db
            ));
        }
    }

    eprintln!(
        "[{}] recall {hits}/{} (floor {})  extra {} (budget {})",
        set.name,
        set.expected.len(),
        set.min_hits,
        phantoms.len(),
        set.max_extra
    );

    assert!(
        hits >= set.min_hits,
        "[{}] recall regressed: {hits}/{} decoded, floor is {}. Missing: {missing:?}",
        set.name,
        set.expected.len(),
        set.min_hits
    );
    assert!(
        phantoms.len() <= set.max_extra,
        "[{}] emitted {} decode(s) outside the golden set, budget is {}. \
         WSPR shipped a 50 % false-decode rate behind a green recall test — \
         this is the assertion that catches that. Phantoms: {phantoms:#?}",
        set.name,
        phantoms.len(),
        set.max_extra
    );
    assert!(
        snr_errors.is_empty(),
        "[{}] reported SNR is out of tolerance:\n  {}",
        set.name,
        snr_errors.join("\n  ")
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    static EXPECTED: &[GoldenEntry] = &[
        GoldenEntry {
            msg: "CQ K1ABC FN42",
            freq_hz: Some(1000.0),
            dt_sec: Some(0.0),
            snr_db: Some(-10.0),
        },
        GoldenEntry {
            msg: "K1ABC W9XYZ EN37",
            freq_hz: None,
            dt_sec: None,
            snr_db: None,
        },
    ];

    fn set(min_hits: usize, max_extra: usize) -> GoldenSet {
        GoldenSet {
            name: "test",
            expected: EXPECTED,
            min_hits,
            max_extra,
        }
    }

    fn v(msg: &str, freq_hz: f32, snr_db: Option<f32>) -> DecodeView {
        DecodeView {
            msg: msg.to_string(),
            freq_hz,
            dt_sec: 0.0,
            snr_db,
        }
    }

    #[test]
    fn passes_when_recall_and_precision_both_hold() {
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(-11.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
        ];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    /// The regression that motivated this module: full recall, but the
    /// decoder also invented something.
    #[test]
    #[should_panic(expected = "outside the golden set")]
    fn full_recall_does_not_excuse_a_phantom() {
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(-10.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
            v("UZC/7D0DKY 17", 1501.0, None),
        ];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    #[test]
    #[should_panic(expected = "recall regressed")]
    fn missing_entry_fails_recall() {
        let d = [v("CQ K1ABC FN42", 1000.0, Some(-10.0))];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    #[test]
    #[should_panic(expected = "reported SNR is out of tolerance")]
    fn snr_outside_tolerance_fails() {
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(22.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
        ];
        assert_golden(&d, &set(2, 0), Tolerances::default(), |x| x.clone());
    }

    /// A frequency far from the golden entry must not count as a hit,
    /// or the freq column is decorative.
    #[test]
    #[should_panic(expected = "recall regressed")]
    fn wrong_frequency_is_not_a_hit() {
        let d = [
            v("CQ K1ABC FN42", 1400.0, Some(-10.0)),
            v("K1ABC W9XYZ EN37", 1500.0, None),
        ];
        assert_golden(&d, &set(2, 9), Tolerances::default(), |x| x.clone());
    }

    #[test]
    fn documented_budgets_are_honoured() {
        // A tracked gap (min_hits below len) and a tracked phantom
        // budget both pass without weakening the other assertion.
        let d = [
            v("CQ K1ABC FN42", 1000.0, Some(-10.0)),
            v("SOMETHING ELSE", 1600.0, None),
        ];
        assert_golden(&d, &set(1, 1), Tolerances::default(), |x| x.clone());
    }
}
