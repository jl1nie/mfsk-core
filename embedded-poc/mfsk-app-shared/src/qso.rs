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
        }
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
    pub fn process_message(&mut self, text: &str) -> Option<TxIntent> {
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
        match self.state {
            QsoState::Idle => self.on_idle(&words),
            QsoState::Calling => self.on_calling(&words),
            QsoState::Report => self.on_report(&words),
            QsoState::Final => self.on_final(&words),
        }
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

    #[test]
    fn responder_grid_triggers_report() {
        let mut m = mgr();
        m.call_cq(None);
        m.set_rx_snr(-7);
        let tx = m.process_message("JL1NIE W1AW FN31").unwrap();
        assert_eq!(m.state, QsoState::Report);
        assert_eq!(tx.formatted().as_str(), "W1AW JL1NIE -07");
    }

    #[test]
    fn report_to_rr73() {
        let mut m = mgr();
        m.call_cq(None);
        m.process_message("JL1NIE W1AW FN31");
        // They send R-12 confirming our report
        let tx = m.process_message("JL1NIE W1AW R-12").unwrap();
        assert_eq!(m.state, QsoState::Final);
        assert_eq!(tx.formatted().as_str(), "W1AW JL1NIE RR73");
    }

    #[test]
    fn final_to_idle_on_73() {
        let mut m = mgr();
        m.call_cq(None);
        m.process_message("JL1NIE W1AW FN31");
        m.process_message("JL1NIE W1AW R-12");
        let tx = m.process_message("JL1NIE W1AW 73");
        assert!(tx.is_none());
        assert_eq!(m.state, QsoState::Idle);
    }

    #[test]
    fn unrelated_decode_ignored() {
        let mut m = mgr();
        m.call_cq(None);
        // Two random stations QSOing — must not affect us
        assert!(m.process_message("AA1AA BB2BB FN42").is_none());
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
        let tx = m.process_message("JL1NIE W1AW -15").unwrap();
        assert_eq!(m.state, QsoState::Report);
        assert_eq!(tx.formatted().as_str(), "W1AW JL1NIE R-15");
    }

    #[test]
    fn idle_accepts_unsolicited_call() {
        let mut m = mgr();
        // Someone calls us while we're idle (e.g., we just booted)
        m.set_rx_snr(3);
        let tx = m.process_message("JL1NIE W1AW FN31").unwrap();
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
}
