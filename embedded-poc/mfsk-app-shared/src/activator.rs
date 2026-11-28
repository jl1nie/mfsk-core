//! The CQ-side QSO state machine for unattended portable operation
//! (SOTA / POTA activations).
//!
//! This station calls CQ on a frequency the operator picked, answers
//! whoever calls, logs each completed contact, and stops after a set
//! number of them. It is the *activator* half of a QSO only: it never
//! calls another station first. `qso.rs` is the older general-purpose
//! dry-run machine the StickS3 and Core2 still use; this one replaces it
//! on the CoreS3.
//!
//! **Pure.** No I/O, clock or thread: the board calls [`Activator::decide`]
//! once per own transmit period, at the reply deadline, with the
//! messages addressed to this station that have been decoded since the
//! last call, and gets back what to send and what to log. That is what
//! lets `hosttest/mfsk-app-shared` run every scenario below on the host.
//!
//! ## How it follows WSJT-X, and where it does not
//!
//! Checked against `widgets/mainwindow.cpp`:
//!
//! - **The reply is a function of the caller's latest message, not of a
//!   state counter** — WSJT-X's `processMessage` (5820-5990) sets the
//!   next Tx message from what was received, which is how it steps back
//!   when a caller repeats (a caller who did not copy RR73 sends R-12
//!   again and gets RR73 again). [`reply_to`] is that table.
//! - **Logged when RR73 is sent** (4951-4965: the first 73/RR73 sent
//!   logs the QSO). RR73 goes out once; it is repeated only if the
//!   caller asks for it again.
//! - **Callers with a later-stage message are picked first**, as WSJT-X
//!   processes R+/RR73 replies "normally" even while choosing among
//!   callers (4208-4209), so a tail-ender finishing an earlier exchange
//!   is not starved by new callers.
//! - **Deliberate divergence: a partner who goes quiet is dropped after
//!   [`Config::give_up_periods`] periods.** WSJT-X keeps sending until
//!   its Tx watchdog (6 min by default, reset whenever the message
//!   changes — 5025-5031). An activator's air time is the scarce thing,
//!   and a dropped caller is not lost: if they call again, the table
//!   answers from wherever their message says the exchange is.
//! - **Among several new callers, the strongest SNR** — WSJT-X offers
//!   "CQ: First" and "CQ: Max Dist" (4204-4206); on a portable station's
//!   power the strongest is the one most likely to complete.

use heapless::{String, Vec};

/// Callsign buffer: 11 characters and room for a `/P` or `/QRP`.
pub type Call = String<13>;

/// What follows `CQ` in the call.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CqModifier {
    /// `CQ MYCALL GRID`.
    None,
    /// `CQ SOTA`, `CQ POTA`, `CQ DX` … — one to four letters, a standard
    /// (i3=1) message (`wsjt77::pack28`'s directional CQ).
    Token(String<4>),
    /// `CQ 001` … `CQ 999`, also standard.
    Number(u16),
    /// Up to 13 characters of free text, sent instead of a structured CQ.
    FreeText(String<13>),
}

/// Operator settings for one activation.
#[derive(Debug, Clone)]
pub struct Config {
    pub my_call: Call,
    pub my_grid: String<6>,
    pub cq: CqModifier,
    /// Stop after this many logged contacts. 0 = never stop.
    pub target_qsos: u16,
    /// Own periods a partner may stay silent before being dropped: the
    /// reply goes out this many times in all. Default 2.
    pub give_up_periods: u8,
    /// After the target is reached, periods during which a logged
    /// partner who repeats R±NN (did not copy RR73) still gets RR73.
    pub grace_periods: u8,
    /// Stop calling after this many consecutive CQs with nobody calling.
    /// 0 = never.
    pub idle_stop_periods: u16,
}

impl Config {
    pub fn new(my_call: &str, my_grid: &str) -> Self {
        Self {
            my_call: upper(my_call),
            my_grid: upper(my_grid),
            cq: CqModifier::None,
            target_qsos: 0,
            give_up_periods: 2,
            grace_periods: 2,
            idle_stop_periods: 0,
        }
    }
}

/// The exchange field of a message addressed to this station.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Exchange {
    /// `MYCALL DX` with nothing after it.
    None,
    Grid(String<6>),
    Report(i8),
    RReport(i8),
    Rrr,
    Rr73,
    S73,
}

impl Exchange {
    /// Parse the third field of a standard message. `None` for anything
    /// an activator has no reply to.
    pub fn parse(field: &str) -> Option<Self> {
        let f = field.trim();
        match f {
            "" => return Some(Self::None),
            "RRR" => return Some(Self::Rrr),
            "RR73" => return Some(Self::Rr73),
            "73" => return Some(Self::S73),
            _ => {}
        }
        if let Some(r) = f.strip_prefix('R').and_then(parse_report) {
            return Some(Self::RReport(r));
        }
        if let Some(r) = parse_report(f) {
            return Some(Self::Report(r));
        }
        if is_grid4(f) {
            return Some(Self::Grid(String::try_from(f).ok()?));
        }
        None
    }

    /// How far along the exchange this message is — the order new
    /// callers are picked in.
    fn stage(&self) -> u8 {
        match self {
            Self::None | Self::Grid(_) => 0,
            Self::Report(_) => 1,
            Self::RReport(_) | Self::Rrr | Self::Rr73 => 2,
            Self::S73 => 3,
        }
    }
}

/// One decoded message addressed to this station.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Heard {
    pub from: Call,
    pub exchange: Exchange,
    pub snr: i8,
}

impl Heard {
    /// From a decoded standard or non-standard message's fields
    /// (`wsjt77::Wsjt77Fields::Standard { call1, call2, exchange }`).
    /// `None` unless it is addressed to `my_call`, comes from a resolved
    /// callsign, and carries an exchange an activator answers.
    ///
    /// Hashed calls arrive as `<CALL>` when resolved and `<...>` when
    /// not; the brackets are stripped and an unresolved one is refused,
    /// since it names nobody. Addressing compares base calls, so a
    /// caller who writes `JL1NIE` still reaches `JL1NIE/P` — what WSJT-X
    /// does with `m_baseCall`.
    pub fn parse(my_call: &str, call1: &str, call2: &str, exchange: &str, snr: i8) -> Option<Self> {
        let to = strip_hash(call1)?;
        let from = strip_hash(call2)?;
        if !same_base(to, my_call) || same_base(from, my_call) || from.starts_with("CQ") {
            return None;
        }
        Some(Self {
            from: upper(from),
            exchange: Exchange::parse(exchange)?,
            snr,
        })
    }
}

/// Where the exchange stands from the partner's side: the message they are expected to send next, after what this station last sent them. Mirrors
/// `mfsk_core::ft8::decode_block::ApStage` (the decoder's AP takes it from here).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartnerStage {
    /// This station called CQ: a reply can come from anywhere.
    Calling,
    /// This station answered the partner with a grid: they send a report (`MYCALL DX -15`).
    Replying,
    /// This station sent a report (`-15`) or a roger with a report (`R-15`): they send `R-15`, `RRR`, `RR73` or `73`.
    Rogers,
    /// This station sent `RRR`, `RR73` or `73`: they close with `73` or `RR73`.
    Signoff,
}

impl PartnerStage {
    /// The stage after this station sent `last_tx`.
    pub fn after(last_tx: &TxMsg) -> Self {
        let TxMsg::Std { exchange, .. } = last_tx else {
            return Self::Calling;
        };
        let x = exchange.as_str();
        let signed = |t: &str| {
            let mut c = t.chars();
            matches!(c.next(), Some('+' | '-')) && t.len() >= 2 && c.all(|d| d.is_ascii_digit())
        };
        match x {
            "RRR" | "RR73" | "73" => Self::Signoff,
            _ if signed(x) => Self::Rogers,
            _ if x.strip_prefix('R').is_some_and(signed) => Self::Rogers,
            _ => Self::Replying,
        }
    }
}

/// What to transmit.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TxMsg {
    Cq,
    /// `TO MYCALL EXCHANGE`.
    Std {
        to: Call,
        exchange: String<6>,
    },
}

impl TxMsg {
    /// The three standard-message fields (`call1`, `call2`, `exchange`)
    /// or, for a free-text CQ, `(text, "", "")`. What the board hands to
    /// `wsjt77::pack77` / `pack77_free_text`.
    pub fn fields(&self, cfg: &Config) -> (String<16>, Call, String<13>) {
        let mut c1: String<16> = String::new();
        let mut c3: String<13> = String::new();
        match self {
            TxMsg::Cq => match &cfg.cq {
                CqModifier::FreeText(t) => {
                    let _ = c1.push_str(t);
                    return (c1, Call::new(), c3);
                }
                CqModifier::None => {
                    let _ = c1.push_str("CQ");
                }
                CqModifier::Token(t) => {
                    let _ = c1.push_str("CQ ");
                    let _ = c1.push_str(t);
                }
                CqModifier::Number(n) => {
                    use core::fmt::Write as _;
                    let _ = write!(c1, "CQ {:03}", n % 1000);
                }
            },
            TxMsg::Std { to, exchange } => {
                let _ = c1.push_str(to);
                let _ = c3.push_str(exchange);
                return (c1, cfg.my_call.clone(), c3);
            }
        }
        let _ = c3.push_str(&cfg.my_grid);
        (c1, cfg.my_call.clone(), c3)
    }

    /// The message as it would read on air.
    pub fn text(&self, cfg: &Config) -> String<40> {
        let (a, b, c) = self.fields(cfg);
        let mut s: String<40> = String::new();
        for (i, part) in [a.as_str(), b.as_str(), c.as_str()].iter().enumerate() {
            if part.is_empty() {
                continue;
            }
            if i > 0 && !s.is_empty() {
                let _ = s.push(' ');
            }
            let _ = s.push_str(part);
        }
        s
    }
}

/// A completed contact, logged when RR73 was sent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QsoRecord {
    pub call: Call,
    /// Empty when the caller started with a report.
    pub grid: String<6>,
    pub rst_sent: i8,
    /// `None` when the caller's report never reached us: they sent RRR
    /// or RR73 to a contact this activation has no report for (the
    /// board restarted mid-contact, or they were dropped and their
    /// report went with them). Logged anyway — the plan's table says
    /// RRR/RR73 is logged if not already, and WSJT-X logs on the 73 it
    /// sends whatever `m_rptRcvd` then holds — and written to ADIF
    /// without `RST_RCVD`.
    pub rst_rcvd: Option<i8>,
    /// Own-period index the caller was first answered in, and the one
    /// RR73 was sent in. The board turns them into UTC.
    pub on_period: u32,
    pub off_period: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Not transmitting.
    Stopped,
    Cq,
    Working,
    /// Target reached; only answering logged partners who repeat.
    Done,
}

/// What one decision produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Decision {
    pub tx: Option<TxMsg>,
    pub logged: Option<QsoRecord>,
    pub phase: Phase,
    pub logged_total: u16,
}

#[derive(Debug, Clone)]
struct Partner {
    call: Call,
    grid: String<6>,
    rst_sent: i8,
    rst_rcvd: Option<i8>,
    first_period: u32,
    last_heard: u32,
    last_tx: TxMsg,
}

/// A partner logged recently: answered again (without a second log) if
/// they repeat R±NN, and counted anew if they start over.
#[derive(Debug, Clone)]
struct Recent {
    call: Call,
    period: u32,
}

/// Callers waiting while another contact is under way.
const PENDING_MAX: usize = 8;
/// Own periods a waiting caller stays eligible.
const PENDING_TTL: u32 = 2;
/// Own periods a logged partner stays in the no-second-log window.
const RECENT_TTL: u32 = 4;
/// Partners dropped for going quiet, remembered so that one who comes
/// back mid-exchange is logged with the grid and reports of the first
/// attempt rather than the ones we would have sent had we met now.
const DROPPED_MAX: usize = 4;

pub struct Activator {
    cfg: Config,
    phase: Phase,
    partner: Option<Partner>,
    pending: Vec<(Heard, u32), PENDING_MAX>,
    recent: Vec<Recent, 16>,
    dropped: Vec<Partner, DROPPED_MAX>,
    logged: u16,
    idle: u16,
    done_at: u32,
}

impl Activator {
    pub fn new(cfg: Config) -> Self {
        Self {
            cfg,
            phase: Phase::Stopped,
            partner: None,
            pending: Vec::new(),
            recent: Vec::new(),
            dropped: Vec::new(),
            logged: 0,
            idle: 0,
            done_at: 0,
        }
    }

    pub fn config(&self) -> &Config {
        &self.cfg
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn logged(&self) -> u16 {
        self.logged
    }

    /// The call being worked, if any.
    pub fn partner(&self) -> Option<&str> {
        self.partner.as_ref().map(|p| p.call.as_str())
    }

    /// The partner being worked and what they are expected to send next, from what this station last sent them: the decoder's AP aims at it
    /// (`mfsk_core::ft8::decode_block::ApContext`). `None` without a partner.
    pub fn partner_expectation(&self) -> Option<(&str, PartnerStage)> {
        let p = self.partner.as_ref()?;
        Some((p.call.as_str(), PartnerStage::after(&p.last_tx)))
    }

    /// Begin an activation: CQ from the next decision on. The logged
    /// count restarts.
    pub fn start(&mut self) {
        self.phase = Phase::Cq;
        self.partner = None;
        self.pending.clear();
        self.recent.clear();
        self.dropped.clear();
        self.logged = 0;
        self.idle = 0;
    }

    pub fn stop(&mut self) {
        self.phase = Phase::Stopped;
        self.partner = None;
        self.pending.clear();
    }

    /// Decide what to send in own period `period` (a counter that
    /// advances by one per own transmit period), given every message
    /// addressed to this station decoded since the last call — late
    /// ones included: a caller repeats, so a decode that missed its
    /// deadline is still worth answering from.
    pub fn decide(&mut self, period: u32, heard: &[Heard]) -> Decision {
        let mut logged = None;
        let tx = match self.phase {
            Phase::Stopped => None,
            Phase::Done => self.decide_done(period, heard),
            Phase::Cq | Phase::Working => self.decide_active(period, heard, &mut logged),
        };
        Decision {
            tx,
            logged,
            phase: self.phase,
            logged_total: self.logged,
        }
    }

    fn decide_active(
        &mut self,
        period: u32,
        heard: &[Heard],
        logged: &mut Option<QsoRecord>,
    ) -> Option<TxMsg> {
        self.pending
            .retain(|(_, p)| period.saturating_sub(*p) <= PENDING_TTL);
        self.recent
            .retain(|r| period.saturating_sub(r.period) <= RECENT_TTL);

        // The partner first: their latest, most advanced message.
        if let Some(call) = self.partner.as_ref().map(|p| p.call.clone()) {
            if let Some(h) = best_from(heard, &call) {
                let tx = self.reply(period, h, logged);
                self.queue_others(period, heard, &call);
                return tx.or_else(|| self.next_caller_or_cq(period, &[], logged));
            }
            let p = self.partner.as_ref().unwrap();
            if period.saturating_sub(p.last_heard) < self.cfg.give_up_periods as u32 {
                let tx = p.last_tx.clone();
                self.queue_others(period, heard, &call);
                return Some(tx);
            }
            // Gone quiet: drop them and move on. If they call again the
            // table picks up from their message.
            if let Some(p) = self.partner.take() {
                if self.dropped.is_full() {
                    self.dropped.remove(0);
                }
                let _ = self.dropped.push(p);
            }
        }
        self.next_caller_or_cq(period, heard, logged)
    }

    /// Answer the best of this period's callers and the waiting ones,
    /// or call CQ.
    fn next_caller_or_cq(
        &mut self,
        period: u32,
        heard: &[Heard],
        logged: &mut Option<QsoRecord>,
    ) -> Option<TxMsg> {
        let mut pool: Vec<Heard, 24> = Vec::new();
        for h in heard {
            let _ = pool.push(h.clone());
        }
        for (h, _) in &self.pending {
            if !pool.iter().any(|x| x.from == h.from) {
                let _ = pool.push(h.clone());
            }
        }
        // A 73 needs no answer; drop them before choosing.
        pool.retain(|h| h.exchange != Exchange::S73);
        let pick = pool
            .iter()
            .max_by_key(|h| (h.exchange.stage(), h.snr))
            .cloned();
        if let Some(h) = pick {
            self.pending.retain(|(x, _)| x.from != h.from);
            let from = h.from.clone();
            let tx = self.reply(period, &h, logged);
            self.queue_others(period, heard, &from);
            if tx.is_some() {
                self.idle = 0;
                return tx;
            }
        }
        self.cq(period)
    }

    fn cq(&mut self, _period: u32) -> Option<TxMsg> {
        self.phase = Phase::Cq;
        self.idle = self.idle.saturating_add(1);
        if self.cfg.idle_stop_periods > 0 && self.idle > self.cfg.idle_stop_periods {
            self.phase = Phase::Stopped;
            return None;
        }
        Some(TxMsg::Cq)
    }

    fn queue_others(&mut self, period: u32, heard: &[Heard], except: &str) {
        for h in heard {
            if h.from == except || h.exchange == Exchange::S73 {
                continue;
            }
            if let Some(slot) = self.pending.iter_mut().find(|(x, _)| x.from == h.from) {
                *slot = (h.clone(), period);
            } else if self.pending.push((h.clone(), period)).is_err() {
                // Full: replace the weakest if this one is stronger.
                let rank = |x: &Heard| (x.exchange.stage(), x.snr);
                if let Some(w) = self
                    .pending
                    .iter_mut()
                    .min_by_key(|(x, _)| rank(x))
                    .filter(|(x, _)| rank(h) > rank(x))
                {
                    *w = (h.clone(), period);
                }
            }
        }
    }

    /// The reply table (see the module doc) for `h`, updating the
    /// partner and logging on RR73.
    fn reply(&mut self, period: u32, h: &Heard, logged: &mut Option<QsoRecord>) -> Option<TxMsg> {
        let rst_sent = h.snr.clamp(-50, 49);
        let is_partner = self.partner.as_ref().is_some_and(|p| p.call == h.from);
        if !is_partner {
            let back = self
                .dropped
                .iter()
                .position(|p| p.call == h.from)
                .map(|i| self.dropped.remove(i));
            self.partner = Some(back.unwrap_or_else(|| Partner {
                call: h.from.clone(),
                grid: String::new(),
                rst_sent,
                rst_rcvd: None,
                first_period: period,
                last_heard: period,
                last_tx: TxMsg::Cq,
            }));
        }
        let p = self.partner.as_mut().unwrap();
        p.last_heard = period;
        let tx = match &h.exchange {
            Exchange::None | Exchange::Grid(_) => {
                if let Exchange::Grid(g) = &h.exchange {
                    p.grid = g.clone();
                }
                // Starting (or starting over): a new contact, counted
                // again if they were logged before.
                self.recent.retain(|r| r.call != h.from);
                p.rst_sent = rst_sent;
                std_msg(&h.from, &report(p.rst_sent, false))
            }
            Exchange::Report(r) => {
                self.recent.retain(|r2| r2.call != h.from);
                p.rst_rcvd = Some(*r);
                std_msg(&h.from, &report(p.rst_sent, true))
            }
            Exchange::RReport(r) => {
                p.rst_rcvd = Some(*r);
                let msg = std_msg(&h.from, "RR73");
                self.finish(period, logged);
                msg
            }
            Exchange::Rrr | Exchange::Rr73 => {
                let msg = std_msg(&h.from, "73");
                self.finish(period, logged);
                msg
            }
            Exchange::S73 => {
                self.partner = None;
                None
            }
        };
        if let (Some(t), Some(p)) = (&tx, self.partner.as_mut()) {
            p.last_tx = t.clone();
        }
        // `finish` has already moved the phase on (to Cq, or Done) and
        // retired the partner; only an exchange still under way is Working.
        if tx.is_some() && self.partner.is_some() && self.phase == Phase::Cq {
            self.phase = Phase::Working;
        }
        tx
    }

    /// RR73 (or 73) is going out: log once, retire the partner, and stop
    /// at the target.
    fn finish(&mut self, period: u32, logged: &mut Option<QsoRecord>) {
        let Some(p) = self.partner.take() else {
            return;
        };
        let already = self.recent.iter().any(|r| r.call == p.call);
        if !already {
            *logged = Some(QsoRecord {
                call: p.call.clone(),
                grid: p.grid.clone(),
                rst_sent: p.rst_sent,
                rst_rcvd: p.rst_rcvd,
                on_period: p.first_period,
                off_period: period,
            });
            self.logged = self.logged.saturating_add(1);
            if self.recent.is_full() {
                self.recent.remove(0);
            }
            let _ = self.recent.push(Recent {
                call: p.call,
                period,
            });
        } else if let Some(r) = self.recent.iter_mut().find(|r| r.call == p.call) {
            r.period = period;
        }
        self.phase = Phase::Cq;
        if self.cfg.target_qsos > 0 && self.logged >= self.cfg.target_qsos {
            self.phase = Phase::Done;
            self.done_at = period;
        }
    }

    /// Target reached: no new contacts, but a logged partner who repeats
    /// R±NN still gets RR73 for `grace_periods`.
    fn decide_done(&mut self, period: u32, heard: &[Heard]) -> Option<TxMsg> {
        if period.saturating_sub(self.done_at) > self.cfg.grace_periods as u32 {
            self.phase = Phase::Stopped;
            return None;
        }
        for h in heard {
            let is_recent = self.recent.iter().any(|r| r.call == h.from);
            if is_recent && matches!(h.exchange, Exchange::RReport(_)) {
                return std_msg(&h.from, "RR73");
            }
        }
        None
    }
}

/// The most advanced message from `call` in `heard`.
fn best_from<'a>(heard: &'a [Heard], call: &str) -> Option<&'a Heard> {
    heard
        .iter()
        .filter(|h| h.from == call)
        .max_by_key(|h| h.exchange.stage())
}

fn std_msg(to: &str, exchange: &str) -> Option<TxMsg> {
    Some(TxMsg::Std {
        to: String::try_from(to).ok()?,
        exchange: String::try_from(exchange).ok()?,
    })
}

/// `-07`, `+03`, or with `R` in front.
fn report(snr: i8, roger: bool) -> String<6> {
    use core::fmt::Write as _;
    let mut s: String<6> = String::new();
    if roger {
        let _ = s.push('R');
    }
    let v = snr.clamp(-50, 49);
    let _ = write!(
        s,
        "{}{:02}",
        if v >= 0 { '+' } else { '-' },
        v.unsigned_abs()
    );
    s
}

fn parse_report(s: &str) -> Option<i8> {
    let b = s.as_bytes();
    if b.len() != 3
        || !(b[0] == b'+' || b[0] == b'-')
        || !b[1].is_ascii_digit()
        || !b[2].is_ascii_digit()
    {
        return None;
    }
    let v = ((b[1] - b'0') * 10 + (b[2] - b'0')) as i8;
    Some(if b[0] == b'-' { -v } else { v })
}

fn is_grid4(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() == 4
        && (b'A'..=b'R').contains(&b[0])
        && (b'A'..=b'R').contains(&b[1])
        && b[2].is_ascii_digit()
        && b[3].is_ascii_digit()
        // `RR73` is also four characters matching this shape's letters
        // and digits; it is handled before this is reached.
        && s != "RR73"
}

/// `<JL1NIE/P>` → `JL1NIE/P`; `<...>` (unresolved) → `None`.
fn strip_hash(call: &str) -> Option<&str> {
    let c = call.trim();
    let c = c
        .strip_prefix('<')
        .and_then(|x| x.strip_suffix('>'))
        .unwrap_or(c);
    if c.is_empty() || c == "..." {
        None
    } else {
        Some(c)
    }
}

/// The base callsign: `JA1ABC/P` → `JA1ABC`, `KH6/JA1ABC` → `JA1ABC`
/// (the longest part, as WSJT-X's `Radio::base_callsign` takes it).
fn base(call: &str) -> &str {
    call.split('/').max_by_key(|p| p.len()).unwrap_or(call)
}

fn same_base(a: &str, b: &str) -> bool {
    base(a).eq_ignore_ascii_case(base(b))
}

fn upper<const N: usize>(s: &str) -> String<N> {
    let mut out: String<N> = String::new();
    for c in s.chars() {
        for u in c.to_uppercase() {
            if out.push(u).is_err() {
                return out;
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        let mut c = Config::new("JL1NIE/P", "PM95");
        c.cq = CqModifier::Token(String::try_from("SOTA").unwrap());
        c
    }

    fn h(from: &str, ex: &str, snr: i8) -> Heard {
        Heard::parse("JL1NIE/P", "JL1NIE/P", from, ex, snr).expect("addressed to us")
    }

    fn txt(a: &Activator, d: &Decision) -> std::string::String {
        d.tx.as_ref()
            .map(|t| t.text(a.config()).as_str().to_owned())
            .unwrap_or_default()
    }

    fn started(c: Config) -> Activator {
        let mut a = Activator::new(c);
        a.start();
        a
    }

    #[test]
    fn cq_texts() {
        let mut c = cfg();
        let a = Activator::new(c.clone());
        assert_eq!(TxMsg::Cq.text(a.config()).as_str(), "CQ SOTA JL1NIE/P PM95");
        c.cq = CqModifier::None;
        assert_eq!(TxMsg::Cq.text(&c).as_str(), "CQ JL1NIE/P PM95");
        c.cq = CqModifier::Number(7);
        assert_eq!(TxMsg::Cq.text(&c).as_str(), "CQ 007 JL1NIE/P PM95");
        c.cq = CqModifier::FreeText(String::try_from("QRV JA-1234").unwrap());
        assert_eq!(TxMsg::Cq.text(&c).as_str(), "QRV JA-1234");
    }

    #[test]
    fn full_exchange_logs_once_at_rr73() {
        let mut a = started(cfg());
        let d = a.decide(0, &[]);
        assert_eq!(txt(&a, &d), "CQ SOTA JL1NIE/P PM95");
        let d = a.decide(1, &[h("W1AW", "FN31", -7)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P -07");
        assert_eq!(a.phase(), Phase::Working);
        let d = a.decide(2, &[h("W1AW", "R-12", -8)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P RR73");
        let rec = d.logged.expect("logged at RR73");
        assert_eq!(rec.call.as_str(), "W1AW");
        assert_eq!(rec.grid.as_str(), "FN31");
        assert_eq!((rec.rst_sent, rec.rst_rcvd), (-7, Some(-12)));
        assert_eq!((rec.on_period, rec.off_period), (1, 2));
        // Their 73 needs nothing; back to CQ, nothing logged twice.
        let d = a.decide(3, &[h("W1AW", "73", -8)]);
        assert_eq!(txt(&a, &d), "CQ SOTA JL1NIE/P PM95");
        assert!(d.logged.is_none());
        assert_eq!(a.logged(), 1);
    }

    #[test]
    fn a_caller_who_starts_with_a_report_gets_our_report_rogered() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        // They skip the grid: our reply carries OUR report of THEM (-15
        // from their SNR), rogering theirs.
        let d = a.decide(1, &[h("W1AW", "-03", -15)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P R-15");
        let d = a.decide(2, &[h("W1AW", "RR73", -15)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P 73");
        let rec = d.logged.unwrap();
        assert_eq!((rec.rst_sent, rec.rst_rcvd), (-15, Some(-3)));
        assert!(rec.grid.is_empty());
    }

    #[test]
    fn the_partner_stage_follows_what_was_sent() {
        let std = |x: &str| TxMsg::Std {
            to: Call::try_from("W1AW").unwrap(),
            exchange: String::try_from(x).unwrap(),
        };
        assert_eq!(PartnerStage::after(&TxMsg::Cq), PartnerStage::Calling);
        assert_eq!(PartnerStage::after(&std("PM95")), PartnerStage::Replying);
        assert_eq!(PartnerStage::after(&std("-15")), PartnerStage::Rogers);
        assert_eq!(PartnerStage::after(&std("+05")), PartnerStage::Rogers);
        assert_eq!(PartnerStage::after(&std("R-15")), PartnerStage::Rogers);
        assert_eq!(PartnerStage::after(&std("RRR")), PartnerStage::Signoff);
        assert_eq!(PartnerStage::after(&std("RR73")), PartnerStage::Signoff);
        assert_eq!(PartnerStage::after(&std("73")), PartnerStage::Signoff);
    }

    #[test]
    fn a_quiet_partner_is_dropped_after_two_periods_and_answered_if_they_return() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        let d = a.decide(1, &[h("W1AW", "FN31", -7)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P -07");
        // Silent once: the report goes again.
        let d = a.decide(2, &[]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P -07");
        // Silent twice: dropped, CQ.
        let d = a.decide(3, &[]);
        assert_eq!(txt(&a, &d), "CQ SOTA JL1NIE/P PM95");
        assert!(a.partner().is_none());
        // They come back with R-12: finished from their message.
        let d = a.decide(4, &[h("W1AW", "R-12", -9)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P RR73");
        // Logged with what was actually exchanged on the first attempt:
        // the -07 we sent and their grid, not a -09 we never sent.
        let rec = d.logged.expect("logged on their return");
        assert_eq!(rec.grid.as_str(), "FN31");
        assert_eq!((rec.rst_sent, rec.rst_rcvd), (-7, Some(-12)));
        assert_eq!((rec.on_period, rec.off_period), (1, 4));
        assert_eq!(a.logged(), 1);
    }

    #[test]
    fn give_up_is_configurable() {
        let mut c = cfg();
        c.give_up_periods = 3;
        let mut a = started(c);
        a.decide(0, &[]);
        a.decide(1, &[h("W1AW", "FN31", -7)]);
        let d = a.decide(2, &[]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P -07");
        let d = a.decide(3, &[]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P -07");
        let d = a.decide(4, &[]);
        assert_eq!(txt(&a, &d), "CQ SOTA JL1NIE/P PM95");
    }

    #[test]
    fn a_repeated_r_report_gets_rr73_again_without_a_second_log() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        a.decide(1, &[h("W1AW", "FN31", -7)]);
        let d = a.decide(2, &[h("W1AW", "R-12", -8)]);
        assert!(d.logged.is_some());
        // They did not copy RR73.
        let d = a.decide(3, &[h("W1AW", "R-12", -8)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P RR73");
        assert!(d.logged.is_none());
        assert_eq!(a.logged(), 1);
    }

    #[test]
    fn the_strongest_new_caller_is_answered_and_the_rest_are_picked_up_after() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        let d = a.decide(
            1,
            &[
                h("K1ABC", "FN42", -18),
                h("W1AW", "FN31", -5),
                h("N2XY", "FN20", -12),
            ],
        );
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P -05");
        let d = a.decide(2, &[h("W1AW", "R-10", -5)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P RR73");
        // Next: the strongest waiting caller rather than a CQ.
        let d = a.decide(3, &[]);
        assert_eq!(txt(&a, &d), "N2XY JL1NIE/P -12");
    }

    #[test]
    fn a_caller_finishing_an_earlier_exchange_beats_new_callers() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        let d = a.decide(1, &[h("K1ABC", "FN42", -3), h("W1AW", "R-10", -15)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P RR73");
        // Logged although W1AW was never our partner in this activation
        // — the draft of this file dropped exactly this record.
        assert_eq!(
            d.logged.map(|r| r.call),
            Some(Call::try_from("W1AW").unwrap())
        );
        // The new caller waited, and is next.
        let d = a.decide(2, &[]);
        assert_eq!(txt(&a, &d), "K1ABC JL1NIE/P -03");
    }

    #[test]
    fn rr73_to_a_contact_we_hold_no_report_for_is_logged_without_one() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        let d = a.decide(1, &[h("W1AW", "RR73", -10)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P 73");
        let rec = d.logged.expect("logged");
        assert_eq!(rec.rst_rcvd, None);
        assert_eq!(a.logged(), 1);
    }

    #[test]
    fn the_phase_after_rr73_is_cq_not_working() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        a.decide(1, &[h("W1AW", "FN31", -7)]);
        let d = a.decide(2, &[h("W1AW", "R-12", -8)]);
        assert_eq!(d.phase, Phase::Cq);
        assert!(a.partner().is_none());
    }

    #[test]
    fn a_partner_is_not_interrupted_by_a_new_caller() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        a.decide(1, &[h("W1AW", "FN31", -20)]);
        // A much stronger station calls while W1AW is mid-exchange.
        let d = a.decide(2, &[h("K1ABC", "FN42", 5), h("W1AW", "R-12", -20)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P RR73");
        let d = a.decide(3, &[]);
        assert_eq!(txt(&a, &d), "K1ABC JL1NIE/P +05");
    }

    #[test]
    fn a_waiting_caller_expires() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        a.decide(1, &[h("W1AW", "FN31", -5), h("K1ABC", "FN42", -10)]);
        // W1AW stretches over three silent periods and a finish; K1ABC,
        // queued at 1, is past PENDING_TTL by the time we are free.
        a.decide(2, &[]);
        a.decide(3, &[h("W1AW", "R-10", -5)]);
        let d = a.decide(4, &[]);
        assert_eq!(txt(&a, &d), "CQ SOTA JL1NIE/P PM95");
    }

    #[test]
    fn stops_at_the_target_and_still_answers_a_repeat_in_grace() {
        let mut c = cfg();
        c.target_qsos = 1;
        let mut a = started(c);
        a.decide(0, &[]);
        a.decide(1, &[h("W1AW", "FN31", -7)]);
        let d = a.decide(2, &[h("W1AW", "R-12", -8)]);
        assert_eq!(d.phase, Phase::Done);
        // No CQ in Done.
        let d = a.decide(3, &[h("K1ABC", "FN42", -3)]);
        assert!(d.tx.is_none());
        // A logged partner who repeats still gets RR73 inside the grace.
        let d = a.decide(4, &[h("W1AW", "R-12", -8)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P RR73");
        // After the grace: stopped.
        let d = a.decide(6, &[]);
        assert_eq!(d.phase, Phase::Stopped);
        assert!(d.tx.is_none());
    }

    #[test]
    fn a_dupe_who_starts_over_is_counted_again() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        a.decide(1, &[h("W1AW", "FN31", -7)]);
        a.decide(2, &[h("W1AW", "R-12", -8)]);
        a.decide(3, &[]);
        a.decide(4, &[h("W1AW", "FN31", -6)]);
        let d = a.decide(5, &[h("W1AW", "R-11", -6)]);
        assert!(d.logged.is_some());
        assert_eq!(a.logged(), 2);
    }

    #[test]
    fn messages_not_for_us_and_unresolved_hashes_are_refused() {
        assert!(Heard::parse("JL1NIE/P", "K1ABC", "W1AW", "FN31", 0).is_none());
        assert!(Heard::parse("JL1NIE/P", "<...>", "W1AW", "FN31", 0).is_none());
        assert!(Heard::parse("JL1NIE/P", "JL1NIE/P", "<...>", "R-10", 0).is_none());
        // Resolved hash, and a caller who drops our /P.
        assert!(Heard::parse("JL1NIE/P", "<JL1NIE/P>", "W1AW", "FN31", 0).is_some());
        assert!(Heard::parse("JL1NIE/P", "JL1NIE", "W1AW", "FN31", 0).is_some());
        // Our own call as the sender (an echo) is not a caller.
        assert!(Heard::parse("JL1NIE/P", "JL1NIE/P", "JL1NIE", "FN31", 0).is_none());
    }

    #[test]
    fn exchanges_parse() {
        assert_eq!(Exchange::parse("RR73"), Some(Exchange::Rr73));
        assert_eq!(Exchange::parse("RRR"), Some(Exchange::Rrr));
        assert_eq!(Exchange::parse("73"), Some(Exchange::S73));
        assert_eq!(Exchange::parse("R-09"), Some(Exchange::RReport(-9)));
        assert_eq!(Exchange::parse("+05"), Some(Exchange::Report(5)));
        assert_eq!(
            Exchange::parse("FN31"),
            Some(Exchange::Grid(String::try_from("FN31").unwrap()))
        );
        assert_eq!(Exchange::parse(""), Some(Exchange::None));
        assert_eq!(Exchange::parse("TU"), None);
    }

    #[test]
    fn idle_cq_stops_when_configured() {
        let mut c = cfg();
        c.idle_stop_periods = 3;
        let mut a = started(c);
        for p in 0..3 {
            assert!(a.decide(p, &[]).tx.is_some());
        }
        assert_eq!(a.decide(3, &[]).phase, Phase::Stopped);
    }

    #[test]
    fn a_late_decode_is_answered_at_the_next_decision() {
        let mut a = started(cfg());
        a.decide(0, &[]);
        // Their call missed decision 1's deadline…
        let d = a.decide(1, &[]);
        assert_eq!(txt(&a, &d), "CQ SOTA JL1NIE/P PM95");
        // …and arrives with decision 2.
        let d = a.decide(2, &[h("W1AW", "FN31", -7)]);
        assert_eq!(txt(&a, &d), "W1AW JL1NIE/P -07");
    }
}
