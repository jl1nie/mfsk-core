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
