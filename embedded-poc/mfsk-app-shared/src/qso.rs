//! QSO finite state machine (Phase 4 dry-run).
//!
//! Rust port of `rs-ft8n/docs/qso.js QsoManager`.
//! State transitions: Idle → Calling → Report → Final → Idle.
//!
//! - Calling: 自局 CQ または指定局呼出。retry 5 × 15 s
//! - Report:  受信報告返信 (rx_snr を tx_report に変換)
//! - Final:   73 送信。retry 3 回
//! - Idle:    初期/終了状態
//!
//! Phase 4 では UI 連携 (auto-CQ + 受信応答) のみ、実 audio TX (Phase 1
//! 後) は未配線。`process_message` の戻り値を decode_pipeline が
//! `UiState::tx_line` に push する。

use core::fmt::Write as _;
use heapless::String;

use crate::parity::Parity;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QsoState {
    Idle,
    Calling,
    Report,
    Final,
}

/// What the FSM wants to transmit next slot. The three fields map to
/// WSJT-X 77-bit `i3.n3 = 1` standard message slots: `call1 call2 report`.
#[derive(Debug, Clone)]
pub struct TxIntent {
    pub call1: String<13>,
    pub call2: String<13>,
    pub report: String<8>,
}

impl TxIntent {
    /// Whitespace-joined message body — what would go on air.
    pub fn formatted(&self) -> String<32> {
        let mut s: String<32> = String::new();
        let _ = push_clamped(&mut s, &self.call1);
        if !self.call2.is_empty() {
            let _ = s.push(' ');
            let _ = push_clamped(&mut s, &self.call2);
        }
        if !self.report.is_empty() {
            let _ = s.push(' ');
            let _ = push_clamped(&mut s, &self.report);
        }
        s
    }
}

const MAX_RETRIES: u8 = 5;
const FINAL_MAX_RETRIES: u8 = 3;

pub struct QsoManager {
    pub state: QsoState,
    pub my_call: String<11>,
    pub my_grid: String<6>,
    pub dx_call: String<11>,
    pub dx_grid: String<6>,
    pub tx_report: String<8>,
    pub rx_report: String<8>,
    pub rx_snr: i8,
    pub retry_count: u8,
    pub cq_suffix: String<8>,
    /// Auto-CQ gate (Phase 1.7-Stick, 2026-05-17). When `false`, the
    /// TX scheduler skips firing fresh CQ at slot boundary; existing
    /// in-progress QSOs (Calling/Report/Final states) still advance
    /// — only the Idle → Calling auto-transition is gated.
    /// Toggled from the menu UI; persisted in NVS as `"auto_cq"`.
    pub auto_cq_enabled: bool,
    /// Confirmed peer TX-slot parity. Locked on the first
    /// state-advancing decode for which the caller passes
    /// `parity_lock_ok = true` (= bootstrap_slot_shift has converged
    /// — see issue #110). Cleared by `reset()`; preserved across
    /// retries within the same QSO.
    pub peer_parity: Option<Parity>,
    /// Self TX parity. Locked on the first own TX completion
    /// (`record_self_tx`). Used as a fallback when `peer_parity` is
    /// still unknown so that successive own-CQ retries don't drift
    /// between even/odd slots.
    pub self_tx_parity: Option<Parity>,
}

impl QsoManager {
    pub fn new(my_call: &str, my_grid: &str) -> Self {
        let mut mc: String<11> = String::new();
        let _ = push_upper(&mut mc, my_call);
        let mut mg: String<6> = String::new();
        let _ = push_upper(&mut mg, my_grid);
        Self {
            state: QsoState::Idle,
            my_call: mc,
            my_grid: mg,
            dx_call: String::new(),
            dx_grid: String::new(),
            tx_report: String::new(),
            rx_report: String::new(),
            rx_snr: -10,
            retry_count: 0,
            cq_suffix: String::new(),
            auto_cq_enabled: false,
            peer_parity: None,
            self_tx_parity: None,
        }
    }

    /// Toggle the auto-CQ gate. Returns the new state.
    pub fn set_auto_cq(&mut self, on: bool) -> bool {
        self.auto_cq_enabled = on;
        on
    }

    pub fn set_rx_snr(&mut self, snr: i8) {
        self.rx_snr = snr;
    }

    pub fn reset(&mut self) {
        self.state = QsoState::Idle;
        self.dx_call.clear();
        self.dx_grid.clear();
        self.tx_report.clear();
        self.rx_report.clear();
        self.retry_count = 0;
        // Parity locks are per-QSO; the next QSO may be with a peer
        // on the opposite slot.
        self.peer_parity = None;
        self.self_tx_parity = None;
    }

    /// Preferred TX slot parity for the next own transmission.
    ///
    /// Precedence:
    ///   1. `opposite(peer_parity)` — primary case, set as soon as
    ///      we've cleanly decoded a peer message addressed to us.
    ///   2. `self_tx_parity` — same-QSO continuation when peer_parity
    ///      is still unlocked (e.g. peer transmitted but framing was
    ///      not yet settled when we processed the decode).
    ///   3. `None` — fresh cold-boot CQ. Caller may fire on any slot;
    ///      that slot's parity then becomes `self_tx_parity` via
    ///      `record_self_tx`.
    pub fn preferred_tx_parity(&self) -> Option<Parity> {
        if let Some(p) = self.peer_parity {
            return Some(p.opposite());
        }
        self.self_tx_parity
    }

    /// Audio-side callback after a TX cycle completes. Locks
    /// `self_tx_parity` if not already set, so that subsequent
    /// retransmissions in this QSO stick to the same slot parity even
    /// while peer_parity remains unknown.
    pub fn record_self_tx(&mut self, tx_slot: u32) {
        if self.self_tx_parity.is_none() {
            self.self_tx_parity = Some(Parity::from_slot_index(tx_slot));
        }
    }

    /// Atomic next-TX pick, parity- and auto-CQ-aware. Returns the
    /// `TxIntent` to fire iff one is pending (or `auto_cq_enabled` and
    /// we're Idle) AND the supplied `cur_par` matches our preferred TX
    /// parity. Returns `None` otherwise.
    ///
    /// Doing the parity check inside this method (rather than at each
    /// caller after `next_tx()`) lets the FSM evaluate
    /// `preferred_tx_parity()` against the **freshly transitioned**
    /// state if the auto-CQ branch fires — important because the
    /// `Idle → Calling` transition itself might affect
    /// `self_tx_parity` later. It also removes the duplicate
    /// orchestration block that was sitting in both
    /// `audio::tx_scheduler_pick_intent` and `tx_scheduler::scheduler_thread`.
    ///
    /// `cur_par` is the parity of the slot the caller intends to TX in;
    /// the caller is responsible for both bracketing this with the
    /// launch-deadline check (`parity::TX_LAUNCH_DEADLINE_US`) and
    /// pairing it with the matching `wav_idx` for later
    /// `record_self_tx`. `current_capture_info` is the right source
    /// for that pair.
    pub fn pick_tx_intent(&mut self, cur_par: Parity) -> Option<TxIntent> {
        let intent = self.next_tx().or_else(|| {
            if self.auto_cq_enabled && self.state == QsoState::Idle {
                Some(self.call_cq(None))
            } else {
                None
            }
        })?;
        // `None` from preferred_tx_parity = fresh cold-boot CQ — fire
        // in any slot; that slot becomes self_tx_parity via the
        // caller's subsequent `record_self_tx`.
        let our_par = self.preferred_tx_parity().unwrap_or(cur_par);
        if cur_par == our_par {
            Some(intent)
        } else {
            None
        }
    }

    /// Combined post-TX hook: lock `self_tx_parity` to the slot we
    /// just transmitted in (sticky for the rest of this QSO) AND
    /// advance the per-period FSM (retry counter / Idle reset).
    /// Single lock acquisition for callers that hold a
    /// `Mutex<QsoManager>`.
    pub fn finish_self_tx(&mut self, tx_slot: u32) {
        self.record_self_tx(tx_slot);
        let _ = self.on_period_end();
    }

    /// Begin calling CQ. Returns the TxIntent for the upcoming slot.
    pub fn call_cq(&mut self, suffix: Option<&str>) -> TxIntent {
        self.cq_suffix.clear();
        if let Some(s) = suffix {
            let _ = push_upper(&mut self.cq_suffix, s);
        }
        self.state = QsoState::Calling;
        self.dx_call.clear();
        self.dx_grid.clear();
        self.retry_count = 0;
        self.cq_intent()
    }

    /// Begin calling a specific station.
    pub fn call_station(&mut self, dx: &str) -> TxIntent {
        self.dx_call.clear();
        let _ = push_upper(&mut self.dx_call, dx);
        self.dx_grid.clear();
        self.state = QsoState::Calling;
        self.retry_count = 0;
        make_intent(&self.dx_call, &self.my_call, &self.my_grid)
    }

    /// Process a decoded message. Returns Some(TxIntent) if the FSM
    /// wants to transmit in response.
    ///
    /// `rx_slot` is the `wav_idx` of the audio slot the message was
    /// captured from (= `SpecBundle.wav_idx`). When the message drives
    /// a state transition AND `parity_lock_ok` is `true`, `peer_parity`
    /// is locked to `rx_slot & 1`. `parity_lock_ok` should be `false`
    /// during the bootstrap_slot_shift convergence window (see
    /// `parity::SAFE_DT_THRESHOLD_SEC`) — state transitions and retry
    /// counters still advance normally, but parity stays unlocked
    /// until a later clean decode.
    pub fn process_message(
        &mut self,
        text: &str,
        rx_slot: u32,
        parity_lock_ok: bool,
    ) -> Option<TxIntent> {
        if self.my_call.is_empty() {
            return None;
        }
        let mut words: heapless::Vec<&str, 8> = heapless::Vec::new();
        for w in text.split_whitespace() {
            if words.push(w).is_err() {
                break;
            }
        }
        if words.len() < 2 {
            return None;
        }
        let result = match self.state {
            QsoState::Idle => self.on_idle(&words),
            QsoState::Calling => self.on_calling(&words),
            QsoState::Report => self.on_report(&words),
            QsoState::Final => self.on_final(&words),
        };
        // Lock peer_parity only when (a) the decode actually drove a
        // state transition (= `result.is_some()` OR we landed in Idle
        // via Final's 73 handler) AND (b) the caller signalled that
        // slot framing is settled enough to trust `rx_slot`.
        // We use `result.is_some()` as the "addressed to us" proxy
        // for non-Final transitions; for Final → Idle (73 received),
        // the FSM has just completed and parity will be cleared on
        // next reset anyway, so no lock needed.
        if parity_lock_ok && self.peer_parity.is_none() && result.is_some() {
            self.peer_parity = Some(Parity::from_slot_index(rx_slot));
        }
        result
    }

    /// Slot end with no relevant response. Returns retry intent or None
    /// when retry budget is exhausted (FSM auto-resets).
    pub fn on_period_end(&mut self) -> Option<TxIntent> {
        if self.state == QsoState::Idle {
            return None;
        }
        let limit = if self.state == QsoState::Final {
            FINAL_MAX_RETRIES
        } else {
            MAX_RETRIES
        };
        if self.retry_count >= limit {
            // Final timeout: 73 already sent; treat as completed.
            if self.state == QsoState::Final {
                self.state = QsoState::Idle;
                return None;
            }
            self.reset();
            return None;
        }
        self.retry_count += 1;
        self.next_tx()
    }

    /// Current slot's TX intent (= what `on_period_end` retransmits).
    pub fn next_tx(&self) -> Option<TxIntent> {
        match self.state {
            QsoState::Calling => {
                if !self.dx_call.is_empty() {
                    Some(make_intent(&self.dx_call, &self.my_call, &self.my_grid))
                } else {
                    Some(self.cq_intent())
                }
            }
            QsoState::Report => Some(make_intent(&self.dx_call, &self.my_call, &self.tx_report)),
            QsoState::Final => Some(make_intent(&self.dx_call, &self.my_call, "73")),
            QsoState::Idle => None,
        }
    }

    /// `(retry_count, limit)` if mid-QSO with at least one retry, else None.
    pub fn retry_info(&self) -> Option<(u8, u8)> {
        if self.state == QsoState::Idle || self.retry_count == 0 {
            return None;
        }
        let limit = if self.state == QsoState::Final {
            FINAL_MAX_RETRIES
        } else {
            MAX_RETRIES
        };
        Some((self.retry_count, limit))
    }

    pub fn state_label(&self) -> &'static str {
        match self.state {
            QsoState::Idle => "IDLE",
            QsoState::Calling => "CALL",
            QsoState::Report => "RPT",
            QsoState::Final => "FIN",
        }
    }

    // ── Internal ──────────────────────────────────────────────────────

    fn cq_intent(&self) -> TxIntent {
        let mut call1: String<13> = String::new();
        let _ = call1.push_str("CQ");
        if !self.cq_suffix.is_empty() {
            let _ = call1.push(' ');
            let _ = push_clamped(&mut call1, &self.cq_suffix);
        }
        let mut call2: String<13> = String::new();
        let _ = push_clamped(&mut call2, &self.my_call);
        let mut report: String<8> = String::new();
        let _ = push_clamped(&mut report, &self.my_grid);
        TxIntent {
            call1,
            call2,
            report,
        }
    }

    fn auto_report(&self) -> String<8> {
        let snr = self.rx_snr.clamp(-50, 49);
        let mut s: String<8> = String::new();
        if snr >= 0 {
            let _ = write!(s, "+{:02}", snr);
        } else {
            let _ = write!(s, "-{:02}", -snr);
        }
        s
    }

    fn on_idle(&mut self, words: &[&str]) -> Option<TxIntent> {
        // qso.js _onIdle has a "CQ X CALL GRID" branch that fires only
        // when dx_call is preset (= UI button targeted a station). Phase
        // 4 dry-run has no targeting UI, so that branch is unreachable
        // and we omit it; Phase 6 button wiring re-introduces it.

        // "MYCALL THEIR_CALL GRID" — somebody answered our (auto-)CQ
        // even though we weren't actively calling. Accept it.
        if words.len() >= 3 && words[0].eq_ignore_ascii_case(&self.my_call) {
            self.dx_call.clear();
            let _ = push_upper(&mut self.dx_call, words[1]);
            self.dx_grid.clear();
            let _ = push_upper(&mut self.dx_grid, words[2]);
            self.tx_report = self.auto_report();
            self.state = QsoState::Report;
            self.retry_count = 0;
            return Some(make_intent(&self.dx_call, &self.my_call, &self.tx_report));
        }
        None
    }

    fn on_calling(&mut self, words: &[&str]) -> Option<TxIntent> {
        if words.len() < 3 || !words[0].eq_ignore_ascii_case(&self.my_call) {
            return None;
        }
        let responder = words[1];
        let field = words[2];
        // Directed call: only accept the target station.
        if !self.dx_call.is_empty() && !self.dx_call.as_str().eq_ignore_ascii_case(responder) {
            return None;
        }
        self.dx_call.clear();
        let _ = push_upper(&mut self.dx_call, responder);
        self.dx_grid.clear();
        let _ = push_upper(&mut self.dx_grid, field);

        if let Some(rpt) = parse_report(field) {
            // They sent a report directly — skip the grid exchange.
            self.rx_report.clear();
            let _ = push_clamped(&mut self.rx_report, field);
            let mut tx: String<8> = String::new();
            if !rpt.starts_with('R') {
                let _ = tx.push('R');
            }
            let _ = push_clamped(&mut tx, &rpt);
            self.tx_report = tx;
        } else {
            // Treated as grid; we generate our own report from rx_snr.
            self.tx_report = self.auto_report();
        }
        self.state = QsoState::Report;
        self.retry_count = 0;
        Some(make_intent(&self.dx_call, &self.my_call, &self.tx_report))
    }

    fn on_report(&mut self, words: &[&str]) -> Option<TxIntent> {
        if words.len() < 3
            || !words[0].eq_ignore_ascii_case(&self.my_call)
            || !words[1].eq_ignore_ascii_case(&self.dx_call)
        {
            return None;
        }
        let field = words[2];
        if field == "RRR" || field == "RR73" {
            self.state = QsoState::Final;
            self.retry_count = 0;
            return Some(make_intent(&self.dx_call, &self.my_call, "73"));
        }
        // R+NN — confirm + 73
        if field.starts_with('R') && parse_report(&field[1..]).is_some() {
            self.rx_report.clear();
            let _ = push_clamped(&mut self.rx_report, field);
            self.state = QsoState::Final;
            self.retry_count = 0;
            return Some(make_intent(&self.dx_call, &self.my_call, "RR73"));
        }
        // +NN (no R) — they sent a report; respond with R+NN
        if parse_report(field).is_some() {
            self.rx_report.clear();
            let _ = push_clamped(&mut self.rx_report, field);
            let mut tx: String<8> = String::new();
            let _ = tx.push('R');
            let _ = push_clamped(&mut tx, field);
            self.tx_report = tx;
            return Some(make_intent(&self.dx_call, &self.my_call, &self.tx_report));
        }
        None
    }

    fn on_final(&mut self, words: &[&str]) -> Option<TxIntent> {
        if words.len() >= 3
            && words[0].eq_ignore_ascii_case(&self.my_call)
            && words[1].eq_ignore_ascii_case(&self.dx_call)
            && (words[2] == "73" || words[2] == "RR73")
        {
            self.state = QsoState::Idle;
        }
        None
    }
}

/// Format `SS RR` or `SS RR LL` for the TX line, where SS is the FSM
/// state label and RR/LL are retry/limit. Returns "IDLE" when nothing
/// is queued.
pub fn format_tx_line(qso: &QsoManager, intent: Option<&TxIntent>) -> String<48> {
    let mut s: String<48> = String::new();
    let _ = write!(s, "{}: ", qso.state_label());
    match intent {
        Some(tx) => {
            let body = tx.formatted();
            let _ = push_clamped(&mut s, &body);
        }
        None => {
            let _ = s.push_str("---");
        }
    }
    if let Some((r, lim)) = qso.retry_info() {
        let _ = write!(s, " [{}/{}]", r, lim);
    }
    s
}

fn parse_report(s: &str) -> Option<String<8>> {
    let rest = s.strip_prefix('R').unwrap_or(s);
    let b = rest.as_bytes();
    if b.len() != 3 {
        return None;
    }
    if (b[0] != b'+' && b[0] != b'-') || !b[1].is_ascii_digit() || !b[2].is_ascii_digit() {
        return None;
    }
    let mut out: String<8> = String::new();
    let _ = push_clamped(&mut out, rest);
    Some(out)
}

fn make_intent(c1: &str, c2: &str, rpt: &str) -> TxIntent {
    let mut call1: String<13> = String::new();
    let _ = push_clamped(&mut call1, c1);
    let mut call2: String<13> = String::new();
    let _ = push_clamped(&mut call2, c2);
    let mut report: String<8> = String::new();
    let _ = push_clamped(&mut report, rpt);
    TxIntent {
        call1,
        call2,
        report,
    }
}

fn push_clamped<const N: usize>(dst: &mut String<N>, src: &str) -> Result<(), ()> {
    for ch in src.chars() {
        if dst.push(ch).is_err() {
            return Err(());
        }
    }
    Ok(())
}

fn push_upper<const N: usize>(dst: &mut String<N>, src: &str) -> Result<(), ()> {
    for ch in src.chars() {
        for u in ch.to_uppercase() {
            if dst.push(u).is_err() {
                return Err(());
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mgr() -> QsoManager {
        QsoManager::new("JL1NIE", "PM95")
    }

    #[test]
    fn cq_emits_cq_my_grid() {
        let mut m = mgr();
        let tx = m.call_cq(None);
        assert_eq!(tx.formatted().as_str(), "CQ JL1NIE PM95");
        assert_eq!(m.state, QsoState::Calling);
    }

    /// Helper: process_message with parity-lock disabled. Most legacy
    /// tests don't care about parity; they exercise the protocol state
    /// machine in isolation.
    fn pm(m: &mut QsoManager, text: &str) -> Option<TxIntent> {
        m.process_message(text, 0, false)
    }

    #[test]
    fn responder_grid_triggers_report() {
        let mut m = mgr();
        m.call_cq(None);
        m.set_rx_snr(-7);
        let tx = pm(&mut m, "JL1NIE W1AW FN31").unwrap();
        assert_eq!(m.state, QsoState::Report);
        assert_eq!(tx.formatted().as_str(), "W1AW JL1NIE -07");
    }

    #[test]
    fn report_to_rr73() {
        let mut m = mgr();
        m.call_cq(None);
        pm(&mut m, "JL1NIE W1AW FN31");
        // They send R-12 confirming our report
        let tx = pm(&mut m, "JL1NIE W1AW R-12").unwrap();
        assert_eq!(m.state, QsoState::Final);
        assert_eq!(tx.formatted().as_str(), "W1AW JL1NIE RR73");
    }

    #[test]
    fn final_to_idle_on_73() {
        let mut m = mgr();
        m.call_cq(None);
        pm(&mut m, "JL1NIE W1AW FN31");
        pm(&mut m, "JL1NIE W1AW R-12");
        let tx = pm(&mut m, "JL1NIE W1AW 73");
        assert!(tx.is_none());
        assert_eq!(m.state, QsoState::Idle);
    }

    #[test]
    fn unrelated_decode_ignored() {
        let mut m = mgr();
        m.call_cq(None);
        // Two random stations QSOing — must not affect us
        assert!(pm(&mut m, "AA1AA BB2BB FN42").is_none());
        assert_eq!(m.state, QsoState::Calling);
    }

    #[test]
    fn retry_increments_then_resets() {
        let mut m = mgr();
        m.call_cq(None);
        for i in 1..=MAX_RETRIES {
            let tx = m.on_period_end().unwrap();
            assert_eq!(m.retry_count, i);
            assert_eq!(tx.formatted().as_str(), "CQ JL1NIE PM95");
        }
        // 6th retry exceeds limit → reset
        assert!(m.on_period_end().is_none());
        assert_eq!(m.state, QsoState::Idle);
    }

    #[test]
    fn report_directly_skips_grid() {
        let mut m = mgr();
        m.call_cq(None);
        let tx = pm(&mut m, "JL1NIE W1AW -15").unwrap();
        assert_eq!(m.state, QsoState::Report);
        assert_eq!(tx.formatted().as_str(), "W1AW JL1NIE R-15");
    }

    #[test]
    fn idle_accepts_unsolicited_call() {
        let mut m = mgr();
        // Someone calls us while we're idle (e.g., we just booted)
        m.set_rx_snr(3);
        let tx = pm(&mut m, "JL1NIE W1AW FN31").unwrap();
        assert_eq!(m.state, QsoState::Report);
        assert_eq!(tx.formatted().as_str(), "W1AW JL1NIE +03");
    }

    #[test]
    fn format_tx_line_includes_retry() {
        let mut m = mgr();
        m.call_cq(None);
        m.on_period_end();
        let intent = m.next_tx();
        let line = format_tx_line(&m, intent.as_ref());
        assert_eq!(line.as_str(), "CALL: CQ JL1NIE PM95 [1/5]");
    }

    // ── Slot-parity tests (issue #110) ────────────────────────────

    #[test]
    fn parity_unlocked_at_boot() {
        let m = mgr();
        assert!(m.peer_parity.is_none());
        assert!(m.self_tx_parity.is_none());
        assert!(m.preferred_tx_parity().is_none());
    }

    #[test]
    fn peer_parity_locks_on_clean_decode_in_calling() {
        let mut m = mgr();
        m.call_cq(None);
        // Peer answers from an ODD slot (rx_slot = 7), framing settled.
        let tx = m
            .process_message("JL1NIE W1AW FN31", 7, true)
            .expect("state advances");
        assert_eq!(m.state, QsoState::Report);
        assert_eq!(m.peer_parity, Some(Parity::Odd));
        // Our TX should now be gated to EVEN.
        assert_eq!(m.preferred_tx_parity(), Some(Parity::Even));
        let _ = tx;
    }

    #[test]
    fn peer_parity_locks_on_unsolicited_call_in_idle() {
        let mut m = mgr();
        m.set_rx_snr(3);
        m.process_message("JL1NIE W1AW FN31", 4, true)
            .expect("idle accepts unsolicited call");
        assert_eq!(m.peer_parity, Some(Parity::Even));
        assert_eq!(m.preferred_tx_parity(), Some(Parity::Odd));
    }

    #[test]
    fn peer_parity_stays_locked_to_rx_slot_when_decode_spills() {
        // Issue #110's core invariant: even if `process_message` is
        // called LATE (during the slot after the one the audio came
        // from), `peer_parity` reflects the audio's slot, not "now".
        let mut m = mgr();
        m.call_cq(None);
        // Peer transmitted in slot 5 (ODD). Decode finished in slot 6
        // — but we pass rx_slot = 5 (the SpecBundle.wav_idx).
        m.process_message("JL1NIE W1AW FN31", 5, true)
            .expect("state advances");
        assert_eq!(m.peer_parity, Some(Parity::Odd));
        assert_eq!(m.preferred_tx_parity(), Some(Parity::Even));
    }

    #[test]
    fn peer_parity_does_not_lock_when_framing_unsettled() {
        let mut m = mgr();
        m.call_cq(None);
        // parity_lock_ok = false (bootstrap_slot_shift not yet converged).
        m.process_message("JL1NIE W1AW FN31", 5, false)
            .expect("state still advances");
        assert_eq!(m.state, QsoState::Report);
        assert!(m.peer_parity.is_none(), "must NOT lock during bootstrap");
        // Next clean decode (e.g. R-12 confirming our report) locks it.
        m.process_message("JL1NIE W1AW R-12", 7, true)
            .expect("report → final");
        assert_eq!(m.peer_parity, Some(Parity::Odd));
        // Our final 73 should go to Even.
        assert_eq!(m.preferred_tx_parity(), Some(Parity::Even));
    }

    #[test]
    fn peer_parity_does_not_lock_on_unrelated_decode() {
        let mut m = mgr();
        m.call_cq(None);
        // Random QSO between two other stations — must not lock parity.
        assert!(m.process_message("AA1AA BB2BB FN42", 3, true).is_none());
        assert!(m.peer_parity.is_none());
    }

    #[test]
    fn record_self_tx_locks_only_first_call() {
        let mut m = mgr();
        m.call_cq(None);
        m.record_self_tx(4);
        assert_eq!(m.self_tx_parity, Some(Parity::Even));
        // Second call must NOT flip parity — sticky for the QSO.
        m.record_self_tx(7);
        assert_eq!(m.self_tx_parity, Some(Parity::Even));
    }

    #[test]
    fn preferred_tx_parity_precedence() {
        let mut m = mgr();
        // No peer, no self → None (fresh cold CQ may use any slot).
        assert!(m.preferred_tx_parity().is_none());
        // Self TX locked first → that parity wins.
        m.record_self_tx(3);
        assert_eq!(m.preferred_tx_parity(), Some(Parity::Odd));
        // Once peer_parity lands, it OVERRIDES self_tx_parity — we
        // adapt to the peer's actual slot rather than perpetuating an
        // arbitrary cold-boot choice.
        m.peer_parity = Some(Parity::Odd);
        assert_eq!(m.preferred_tx_parity(), Some(Parity::Even));
    }

    #[test]
    fn reset_clears_both_parities() {
        let mut m = mgr();
        m.peer_parity = Some(Parity::Even);
        m.self_tx_parity = Some(Parity::Odd);
        m.reset();
        assert!(m.peer_parity.is_none());
        assert!(m.self_tx_parity.is_none());
    }

    // ── pick_tx_intent coverage (gemini PR #112 MEDIUM consolidation) ──

    #[test]
    fn pick_tx_intent_auto_cq_idle_fires_on_any_parity_when_unlocked() {
        let mut m = mgr();
        m.auto_cq_enabled = true;
        // No peer_parity, no self_tx_parity → preferred = None →
        // accept whichever slot we're polled in.
        let i = m.pick_tx_intent(Parity::Even);
        assert!(i.is_some());
        assert_eq!(m.state, QsoState::Calling);
    }

    #[test]
    fn pick_tx_intent_no_auto_cq_idle_returns_none() {
        let mut m = mgr();
        m.auto_cq_enabled = false;
        // No next_tx pending AND auto_cq disabled AND state == Idle.
        assert!(m.pick_tx_intent(Parity::Even).is_none());
        assert_eq!(m.state, QsoState::Idle);
    }

    #[test]
    fn pick_tx_intent_parity_mismatch_skips() {
        let mut m = mgr();
        m.auto_cq_enabled = true;
        m.peer_parity = Some(Parity::Even); // → prefer Odd
        // Polled on Even → parity mismatch, expect None and state
        // unchanged (auto-CQ does fire `call_cq()` which transitions
        // Idle → Calling regardless; the gate is on the parity match
        // for the *returned* intent only).
        assert!(m.pick_tx_intent(Parity::Even).is_none());
    }

    #[test]
    fn pick_tx_intent_parity_match_returns_intent() {
        let mut m = mgr();
        m.auto_cq_enabled = true;
        m.peer_parity = Some(Parity::Even); // → prefer Odd
        let i = m.pick_tx_intent(Parity::Odd);
        assert!(i.is_some());
    }

    #[test]
    fn finish_self_tx_records_parity_and_advances_period() {
        let mut m = mgr();
        m.call_cq(None); // Calling, retry_count = 0
        m.finish_self_tx(3); // Odd slot
        assert_eq!(m.self_tx_parity, Some(Parity::Odd));
        // on_period_end should have advanced retry counter.
        assert_eq!(m.retry_count, 1);
    }
}
