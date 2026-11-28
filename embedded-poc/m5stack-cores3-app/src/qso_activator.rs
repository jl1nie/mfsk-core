// SPDX-License-Identifier: GPL-3.0-only
//! The `Activator` on the CoreS3 (`mfsk-app-shared/src/activator.rs`) behind the decoders' queue (`heard_queue.rs`, PROFILES D8): the cells pipeline [`push`](QsoStep::push)es every decode addressed to
//! this station, and **at the reply deadline** [`on_reply_deadline`](QsoStep::on_reply_deadline) empties the queue into `Activator::decide`, once per own transmit period.
//!
//! **What is and is not decided here.** Transmit is not wired (`plan_cores3_qso_tx`): the decision is shown on the panel's TX line and logged, nothing is sent, and the contact is logged to the
//! serial log only (no ADIF yet: it needs the dial frequency and the UTC of the period). **The clock**: D8 gives it to the QSO side; until a transmit scheduler exists the only clock is the
//! slot's end, so the receive path calls [`on_reply_deadline`](QsoStep::on_reply_deadline) from the one place that knows the window path has finished (the end of a slot's jobs, 0.38 s after
//! the slot's last audio), **every second slot** (an own period is two slots; which parity is ours is the transmit scheduler's choice, so here the even receive slots). The entry point is the
//! only thing that moves when the scheduler arrives.

use mfsk_app_shared::activator::{Activator, Config, PartnerStage, Phase};
use mfsk_app_shared::heard_queue::{heard_item_from_text, HeardCollector, HeardQueue};
use mfsk_app_shared::ui::state::UI;
use mfsk_core::ft8::decode_block::{ApContext, ApStage};

use crate::decode_pipeline::{MY_CALL as CFG_CALL, MY_GRID as CFG_GRID};

/// `MFSK_CORES3_TEST_CALL` / `MFSK_CORES3_TEST_GRID` (compile time): act as another station for a bench run on a baked or simulated band (`cfg.toml`'s call stays untouched).
const MY_CALL: &str = match option_env!("MFSK_CORES3_TEST_CALL") {
    Some(c) => c,
    None => CFG_CALL,
};
const MY_GRID: &str = match option_env!("MFSK_CORES3_TEST_GRID") {
    Some(g) => g,
    None => CFG_GRID,
};

/// The decoders' side of the queue: any task may push.
pub static HEARD: HeardQueue = HeardQueue::new();

/// Queue a decode from any task (the late path's): see [`QsoStep::push`].
pub fn push_heard(text: &str, bits: &[u8], snr_db: f32, period: u32) {
    if let Some(item) = heard_item_from_text(MY_CALL, text, bits, snr_db, period) {
        if !HEARD.push(item) {
            log::warn!("[QSO] heard queue full: the oldest decode was dropped");
        }
    }
}

/// How many stations' last places (frequency, DT) are kept.
const POSITIONS: usize = 8;

pub struct QsoStep {
    activator: Activator,
    collector: HeardCollector,
    /// Where each station that addressed this one was last decoded: (call, Hz, DT s). The AP aims at the partner's.
    places: Vec<(String, f32, f32)>,
}

impl QsoStep {
    /// An activator calling CQ from the first period (what `QsoManager::call_cq` did when idle).
    pub fn new() -> Self {
        let mut activator = Activator::new(Config::new(MY_CALL, MY_GRID));
        activator.start();
        Self {
            activator,
            collector: HeardCollector::new(),
            places: Vec::new(),
        }
    }

    /// The QSO context for the decoder's AP rung: the partner being worked, what they are expected to send next (from what this station last sent them) and where they were last
    /// decoded. `None` without a partner, before they have been decoded once (no place to aim at), or when this station called CQ (a reply can come from anywhere).
    pub fn ap_context(&self) -> Option<ApContext> {
        let (call, stage) = self.activator.partner_expectation()?;
        let stage = match stage {
            PartnerStage::Calling => return None,
            PartnerStage::Replying => ApStage::Replying,
            PartnerStage::Rogers => ApStage::Rogers,
            PartnerStage::Signoff => ApStage::Signoff,
        };
        let (_, f, d) = self.places.iter().find(|p| p.0 == call)?;
        ApContext::new(MY_CALL, call, stage, *f, *d)
    }

    /// Queue a decode (`text`: the rendered message, `bits`: its 77 bits) heard in own period `period`, if it is addressed to this station.
    pub fn push(&mut self, text: &str, bits: &[u8], snr_db: f32, period: u32, freq_hz: f32, dt_sec: f32) {
        if let Some(item) = heard_item_from_text(MY_CALL, text, bits, snr_db, period) {
            let from = item.heard.from.as_str();
            match self.places.iter_mut().find(|p| p.0 == from) {
                Some(p) => (p.1, p.2) = (freq_hz, dt_sec),
                None => {
                    if self.places.len() == POSITIONS {
                        self.places.remove(0);
                    }
                    self.places.push((from.to_string(), freq_hz, dt_sec));
                }
            }
            if !HEARD.push(item) {
                log::warn!("[QSO] heard queue full: the oldest decode was dropped");
            }
        }
    }

    /// The reply deadline of own period `period`: decide from everything queued since the last call, show the decision, log a finished contact.
    pub fn on_reply_deadline(&mut self, period: u32) {
        let heard = self.collector.collect(&HEARD, period);
        let n = heard.len();
        let d = self.activator.decide(period, heard);
        let tx = d.tx.as_ref().map(|t| t.text(self.activator.config()));
        let line = match (&tx, d.phase) {
            (Some(t), _) => format!("TX {t}"),
            (None, Phase::Stopped) => "TX off".to_string(),
            (None, _) => "TX -".to_string(),
        };
        log::info!("[QSO] period {period}: {n} heard -> {line}");
        if let Some(rec) = &d.logged {
            log::info!(
                "[QSO] LOGGED {} grid {} sent {:+} rcvd {:?} (periods {}..{}), {} in all",
                rec.call,
                rec.grid,
                rec.rst_sent,
                rec.rst_rcvd,
                rec.on_period,
                rec.off_period,
                d.logged_total
            );
        }
        if let Ok(mut ui) = UI.lock() {
            ui.set_tx_line(&line);
        }
    }
}
