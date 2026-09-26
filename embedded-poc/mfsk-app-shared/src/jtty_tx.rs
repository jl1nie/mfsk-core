//! The JTTY transmit sequencer, half-duplex.
//!
//! JTTY has no slot, so what starts a transmission is not a clock but the operator
//! and the state of the channel; and on a half-duplex radio (the IC-705) the receiver
//! is deaf while the transmitter runs. This module is the pure sequencing between
//! "a message of N samples is ready" and "the radio has gone back to receive":
//!
//! ```text
//!   Idle ──submit──▶ Waiting ──channel clear for `hold`──▶ Lead ──▶ Sending ──▶ Tail ──▶ Idle
//!                       │                                    PTT on                       PTT off
//!                       └─ `max_wait` ──▶ Cancelled (or send anyway)
//! ```
//!
//! **It is driven by the audio clock, not a wall clock.** The board calls
//! [`JttyTx::poll`] with the number of 12 kHz samples that have just passed (or are
//! about to be handed to the sound device) and gets back what those samples are:
//! [`Segment`]s that add up to exactly that many, the PTT edges among them, and any
//! events. The sample count is also the JTTY receiver's clock (BINDINGS §2.8.1,
//! "Feeding a live source"), which is why the two fit: a segment that is not
//! [`SegKind::Receive`] is a stretch during which the receiver must be fed **zeros**
//! instead of the radio's audio, so its timeline stays true to real time.
//!
//! It knows nothing of tones or waveforms. The board holds the message (an
//! `mfsk_core::jtty::tx::Synth`, say) and, for each [`SegKind::Message`] segment,
//! fills `len` samples from position `offset` of it. That keeps this module free of
//! `mfsk-core` and testable on the host (`hosttest/mfsk-app-shared`).
//!
//! What is *policy* and stays here, in the configuration: how long the channel must
//! have been quiet ([`TxConfig::hold`]), how long to keep asking
//! ([`TxConfig::max_wait`], then [`OnTimeout`]), the PTT-to-audio delay and the tail.
//! What is not decided here: whether the channel is busy (the caller derives it from
//! the receiver's open messages and passes it in), what to send, and queueing more
//! than one message (a second [`JttyTx::submit`] while busy is an error).

use heapless::Vec;

/// Samples per second of the audio clock this module counts in.
pub const FS: usize = 12_000;
/// One JTTY frame: 59 symbols of 384 samples.
pub const FRAME_SAMPLES: usize = 22_656;
/// The most frames a message has.
pub const MAX_FRAMES: usize = 16;
/// The longest transmission, in samples (about 30 s).
pub const MAX_SAMPLES: usize = FRAME_SAMPLES * MAX_FRAMES;

/// What happens when the channel stays busy for [`TxConfig::max_wait`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OnTimeout {
    /// Give up: the message is dropped and [`Event::Cancelled`] reported.
    Cancel,
    /// Transmit regardless.
    SendAnyway,
}

/// Timings, in 12 kHz samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxConfig {
    /// PTT on to the first audio sample: the radio's switching time. Default 0.15 s.
    pub lead: usize,
    /// Last audio sample to PTT off: lets the radio finish the tone and the ramp.
    /// Default 0.3 s.
    pub tail: usize,
    /// The channel must have been clear this long before PTT goes on. A station in
    /// the middle of a message sends its next frame straight after the last, so the
    /// receiver keeps it "busy" through frame gaps and this only has to cover the
    /// receiver's own delay. Default 1 s. `0` transmits at once.
    pub hold: usize,
    /// Longest to wait for a clear channel. Default 30 s (one longest message).
    pub max_wait: usize,
    /// What to do then.
    pub on_timeout: OnTimeout,
}

impl TxConfig {
    /// The defaults documented on the fields.
    pub const fn new() -> Self {
        Self {
            lead: FS * 15 / 100,
            tail: FS * 3 / 10,
            hold: FS,
            max_wait: FS * 30,
            on_timeout: OnTimeout::Cancel,
        }
    }
}

impl Default for TxConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Where the sequencer is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Nothing to send; the receiver has the audio.
    Idle,
    /// A message is queued and the channel is not yet clear enough.
    Waiting,
    /// PTT is on, the radio is switching; silence.
    Lead,
    /// The message is being played.
    Sending,
    /// The message is done or aborted; PTT still on for [`TxConfig::tail`].
    Tail,
}

/// What a stretch of audio-clock samples is for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SegKind {
    /// Receiving: feed the receiver the radio's audio; send nothing.
    Receive,
    /// PTT is on and the radio is switching: send silence; feed the receiver zeros.
    Lead,
    /// Send samples `offset .. offset + len` of the message; feed the receiver zeros.
    Message {
        /// Position in the message of the segment's first sample.
        offset: usize,
    },
    /// PTT is still on after the message: send silence; feed the receiver zeros.
    Tail,
}

/// A run of samples of one kind.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Segment {
    /// How many samples, at least 1.
    pub len: usize,
    /// What they are for.
    pub kind: SegKind,
}

impl Segment {
    /// Whether the receiver must be fed zeros for these samples.
    pub fn blanks_receiver(&self) -> bool {
        self.kind != SegKind::Receive
    }
}

/// Something the sequencer wants the application to know.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// The channel was clear; PTT goes on (see [`Poll::ptt`]).
    Started,
    /// The message and the tail are done; PTT goes off.
    Finished,
    /// The message was dropped without transmitting (timeout with
    /// [`OnTimeout::Cancel`], or [`JttyTx::abort`] while waiting).
    Cancelled,
    /// [`JttyTx::abort`] cut a transmission short.
    Aborted,
}

/// A PTT change, `offset` samples into the poll's chunk.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PttEdge {
    /// Samples from the start of the chunk.
    pub offset: usize,
    /// `true` for PTT on.
    pub on: bool,
}

/// What one [`JttyTx::poll`] found.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Poll {
    /// The chunk, in order; lengths add up to the samples asked for.
    pub segments: Vec<Segment, 6>,
    /// PTT changes within the chunk (at most one on and one off).
    pub ptt: Vec<PttEdge, 2>,
    /// Events, in order.
    pub events: Vec<Event, 4>,
}

impl Poll {
    fn push(&mut self, len: usize, kind: SegKind) {
        if len == 0 {
            return;
        }
        // extend the last segment when it continues (same kind, contiguous message)
        if let Some(last) = self.segments.last_mut() {
            let joins = match (last.kind, kind) {
                (SegKind::Receive, SegKind::Receive)
                | (SegKind::Lead, SegKind::Lead)
                | (SegKind::Tail, SegKind::Tail) => true,
                (SegKind::Message { offset: a }, SegKind::Message { offset: b }) => {
                    a + last.len == b
                }
                _ => false,
            };
            if joins {
                last.len += len;
                return;
            }
        }
        // six segments hold the worst case (receive, lead, message, tail, receive, and
        // a split): a full vector would be a bug in the sequencer
        let _ = self.segments.push(Segment { len, kind });
    }
}

/// Why a message was not accepted.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SubmitError {
    /// A message is already queued or being sent.
    Busy,
    /// Zero samples: there is nothing to send.
    Empty,
    /// More than [`MAX_SAMPLES`].
    TooLong,
}

/// The sequencer.
#[derive(Clone, Debug)]
pub struct JttyTx {
    cfg: TxConfig,
    state: State,
    /// Samples of the message.
    total: usize,
    /// Samples of the message sent so far.
    sent: usize,
    /// Samples the channel has been clear, while waiting.
    clear: usize,
    /// Samples spent waiting.
    waited: usize,
    /// Samples into the current lead or tail.
    stage: usize,
    /// Events raised between polls (an abort).
    pending: Vec<Event, 2>,
}

impl JttyTx {
    /// An idle sequencer.
    pub const fn new(cfg: TxConfig) -> Self {
        Self {
            cfg,
            state: State::Idle,
            total: 0,
            sent: 0,
            clear: 0,
            waited: 0,
            stage: 0,
            pending: Vec::new(),
        }
    }

    /// Where the sequencer is.
    pub fn state(&self) -> State {
        self.state
    }

    /// Whether PTT is (or, this instant, is about to be) on: the states in which the
    /// receiver is fed zeros.
    pub fn transmitting(&self) -> bool {
        matches!(self.state, State::Lead | State::Sending | State::Tail)
    }

    /// The timings in force.
    pub fn config(&self) -> &TxConfig {
        &self.cfg
    }

    /// Change the timings; they apply from the next message.
    pub fn set_config(&mut self, cfg: TxConfig) {
        self.cfg = cfg;
    }

    /// Queue a message of `total_samples` samples (at 12 kHz). It waits for a clear
    /// channel first; [`Self::force`] skips that.
    pub fn submit(&mut self, total_samples: usize) -> Result<(), SubmitError> {
        if self.state != State::Idle {
            return Err(SubmitError::Busy);
        }
        if total_samples == 0 {
            return Err(SubmitError::Empty);
        }
        if total_samples > MAX_SAMPLES {
            return Err(SubmitError::TooLong);
        }
        self.total = total_samples;
        self.sent = 0;
        self.clear = 0;
        self.waited = 0;
        self.stage = 0;
        self.state = State::Waiting;
        Ok(())
    }

    /// The operator overrides the wait: transmit at the next poll.
    pub fn force(&mut self) {
        if self.state == State::Waiting {
            self.clear = self.cfg.hold;
        }
    }

    /// Stop. Waiting: the message is dropped ([`Event::Cancelled`]). Lead or Sending:
    /// the audio stops at once and the tail runs ([`Event::Aborted`]) — a receiver of
    /// the message sees it stop without an end-of-message frame and reports it
    /// incomplete after three frame periods. Tail or Idle: nothing to do.
    pub fn abort(&mut self) {
        match self.state {
            State::Waiting => {
                self.state = State::Idle;
                let _ = self.pending.push(Event::Cancelled);
            }
            State::Lead | State::Sending => {
                self.state = State::Tail;
                self.stage = 0;
                let _ = self.pending.push(Event::Aborted);
            }
            State::Idle | State::Tail => {}
        }
    }

    fn start_tx(&mut self, out: &mut Poll, at: usize) {
        let _ = out.ptt.push(PttEdge {
            offset: at,
            on: true,
        });
        let _ = out.events.push(Event::Started);
        self.stage = 0;
        self.state = State::Lead;
    }

    /// Advance the audio clock by `samples` and say what they are.
    ///
    /// `channel_busy` is the caller's judgement for this chunk (typically "the JTTY
    /// receiver has an open message"); it only matters while waiting. Chunks of any
    /// size give the same sequence: the timeline of segments does not depend on how it
    /// is cut.
    pub fn poll(&mut self, samples: usize, channel_busy: bool) -> Poll {
        let mut out = Poll::default();
        for e in self.pending.iter() {
            let _ = out.events.push(*e);
        }
        self.pending.clear();
        let mut left = samples;
        while left > 0 {
            let at = samples - left;
            match self.state {
                State::Idle => {
                    out.push(left, SegKind::Receive);
                    left = 0;
                }
                State::Waiting => {
                    if self.clear >= self.cfg.hold {
                        self.start_tx(&mut out, at);
                    } else if self.waited >= self.cfg.max_wait {
                        match self.cfg.on_timeout {
                            OnTimeout::SendAnyway => self.start_tx(&mut out, at),
                            OnTimeout::Cancel => {
                                self.state = State::Idle;
                                let _ = out.events.push(Event::Cancelled);
                            }
                        }
                    } else {
                        let to_timeout = self.cfg.max_wait - self.waited;
                        let piece = if channel_busy {
                            left.min(to_timeout)
                        } else {
                            left.min(to_timeout).min(self.cfg.hold - self.clear)
                        };
                        out.push(piece, SegKind::Receive);
                        self.waited += piece;
                        self.clear = if channel_busy { 0 } else { self.clear + piece };
                        left -= piece;
                    }
                }
                State::Lead => {
                    let piece = left.min(self.cfg.lead - self.stage);
                    out.push(piece, SegKind::Lead);
                    self.stage += piece;
                    left -= piece;
                    if self.stage >= self.cfg.lead {
                        self.state = State::Sending;
                    }
                }
                State::Sending => {
                    let piece = left.min(self.total - self.sent);
                    out.push(piece, SegKind::Message { offset: self.sent });
                    self.sent += piece;
                    left -= piece;
                    if self.sent >= self.total {
                        self.stage = 0;
                        self.state = State::Tail;
                    }
                }
                State::Tail => {
                    let piece = left.min(self.cfg.tail - self.stage);
                    out.push(piece, SegKind::Tail);
                    self.stage += piece;
                    left -= piece;
                    if self.stage >= self.cfg.tail {
                        let _ = out.ptt.push(PttEdge {
                            offset: samples - left,
                            on: false,
                        });
                        let _ = out.events.push(Event::Finished);
                        self.state = State::Idle;
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CFG: TxConfig = TxConfig {
        lead: 3,
        tail: 4,
        hold: 5,
        max_wait: 20,
        on_timeout: OnTimeout::Cancel,
    };

    /// The kind of every sample over `n` samples, polled in chunks of `chunk`, plus
    /// every PTT edge (as an absolute sample index) and event.
    fn run(
        tx: &mut JttyTx,
        n: usize,
        chunk: usize,
        busy: impl Fn(usize) -> bool,
    ) -> (
        std::vec::Vec<char>,
        std::vec::Vec<(usize, bool)>,
        std::vec::Vec<Event>,
    ) {
        let (mut kinds, mut edges, mut events) = (std::vec![], std::vec![], std::vec![]);
        let mut t = 0;
        while t < n {
            let c = chunk.min(n - t);
            let poll = tx.poll(c, busy(t));
            assert_eq!(poll.segments.iter().map(|s| s.len).sum::<usize>(), c);
            for s in &poll.segments {
                let ch = match s.kind {
                    SegKind::Receive => 'r',
                    SegKind::Lead => 'l',
                    SegKind::Message { .. } => 'm',
                    SegKind::Tail => 't',
                };
                kinds.extend(std::iter::repeat_n(ch, s.len));
            }
            edges.extend(poll.ptt.iter().map(|e| (t + e.offset, e.on)));
            events.extend(poll.events.iter().copied());
            t += c;
        }
        (kinds, edges, events)
    }

    #[test]
    fn a_message_goes_idle_wait_lead_send_tail_idle() {
        let mut tx = JttyTx::new(CFG);
        assert_eq!(tx.submit(6), Ok(()));
        let (kinds, edges, events) = run(&mut tx, 30, 100, |_| false);
        // 5 clear samples, then lead 3, message 6, tail 4, then receive again
        let s: std::string::String = kinds.into_iter().collect();
        assert_eq!(s, "rrrrrlllmmmmmmttttrrrrrrrrrrrr");
        assert_eq!(edges, [(5, true), (18, false)]);
        assert_eq!(events, [Event::Started, Event::Finished]);
        assert_eq!(tx.state(), State::Idle);
    }

    #[test]
    fn the_timeline_does_not_depend_on_the_chunking() {
        // the busy flag is per chunk, so use edges (8 and 12) every chunk size divides
        let busy = |t: usize| (8..12).contains(&t);
        let reference = {
            let mut tx = JttyTx::new(CFG);
            tx.submit(9).unwrap();
            run(&mut tx, 60, 1, busy)
        };
        assert!(reference.0.contains(&'m') && reference.1.len() == 2);
        for chunk in [2, 4] {
            let mut tx = JttyTx::new(CFG);
            tx.submit(9).unwrap();
            assert_eq!(run(&mut tx, 60, chunk, busy), reference, "chunk {chunk}");
        }
    }

    #[test]
    fn a_busy_channel_restarts_the_hold() {
        let mut tx = JttyTx::new(CFG);
        tx.submit(2).unwrap();
        // clear for 4, busy for 3, then clear: PTT only after 5 clear samples again
        let (kinds, edges, _) = run(&mut tx, 30, 1, |t| (4..7).contains(&t));
        assert_eq!(
            edges[0],
            (12, true),
            "{:?}",
            kinds.iter().collect::<std::string::String>()
        );
    }

    #[test]
    fn waiting_too_long_cancels_or_sends_anyway() {
        let mut tx = JttyTx::new(CFG);
        tx.submit(2).unwrap();
        let (_, edges, events) = run(&mut tx, 40, 4, |_| true);
        assert!(edges.is_empty());
        assert_eq!(events, [Event::Cancelled]);
        assert_eq!(tx.state(), State::Idle);

        let mut tx = JttyTx::new(TxConfig {
            on_timeout: OnTimeout::SendAnyway,
            ..CFG
        });
        tx.submit(2).unwrap();
        let (_, edges, events) = run(&mut tx, 60, 4, |_| true);
        assert_eq!(edges, [(20, true), (29, false)]);
        assert_eq!(events, [Event::Started, Event::Finished]);
    }

    #[test]
    fn force_skips_the_wait() {
        let mut tx = JttyTx::new(CFG);
        tx.submit(2).unwrap();
        tx.force();
        let (_, edges, _) = run(&mut tx, 20, 5, |_| true);
        assert_eq!(edges[0], (0, true));
    }

    #[test]
    fn zero_hold_transmits_at_the_next_poll() {
        let mut tx = JttyTx::new(TxConfig { hold: 0, ..CFG });
        tx.submit(2).unwrap();
        let poll = tx.poll(4, true);
        assert_eq!(
            poll.ptt.as_slice(),
            [PttEdge {
                offset: 0,
                on: true
            }]
        );
    }

    #[test]
    fn abort_while_waiting_drops_the_message() {
        let mut tx = JttyTx::new(CFG);
        tx.submit(6).unwrap();
        tx.abort();
        assert_eq!(tx.state(), State::Idle);
        let poll = tx.poll(10, false);
        assert_eq!(poll.events.as_slice(), [Event::Cancelled]);
        assert!(poll.ptt.is_empty());
    }

    #[test]
    fn abort_while_sending_cuts_the_audio_and_runs_the_tail() {
        let mut tx = JttyTx::new(CFG);
        tx.submit(100).unwrap();
        let (_, _, _) = run(&mut tx, 5 + 3 + 10, 1, |_| false); // 10 samples into the message
        assert_eq!(tx.state(), State::Sending);
        tx.abort();
        let (kinds, edges, events) = run(&mut tx, 10, 3, |_| false);
        let s: std::string::String = kinds.into_iter().collect();
        assert_eq!(s, "ttttrrrrrr");
        assert_eq!(edges, [(4, false)]);
        assert_eq!(events, [Event::Aborted, Event::Finished]);
    }

    #[test]
    fn message_offsets_are_contiguous_across_chunks() {
        let mut tx = JttyTx::new(TxConfig {
            hold: 0,
            lead: 0,
            tail: 0,
            ..CFG
        });
        tx.submit(23).unwrap();
        let mut next = 0;
        for chunk in [7, 5, 9, 4, 6] {
            for s in tx.poll(chunk, false).segments {
                if let SegKind::Message { offset } = s.kind {
                    assert_eq!(offset, next);
                    next += s.len;
                }
            }
        }
        assert_eq!(next, 23);
    }

    #[test]
    fn submit_is_refused_while_busy_and_for_bad_lengths() {
        let mut tx = JttyTx::new(CFG);
        assert_eq!(tx.submit(0), Err(SubmitError::Empty));
        assert_eq!(tx.submit(MAX_SAMPLES + 1), Err(SubmitError::TooLong));
        assert_eq!(tx.submit(MAX_SAMPLES), Ok(()));
        assert_eq!(tx.submit(5), Err(SubmitError::Busy));
    }

    #[test]
    fn the_receiver_is_blanked_exactly_while_ptt_is_on() {
        let mut tx = JttyTx::new(CFG);
        tx.submit(6).unwrap();
        let (mut blank, mut ptt_on) = (0usize, false);
        let (mut edges, mut t) = (std::vec::Vec::new(), 0usize);
        for _ in 0..40 {
            let poll = tx.poll(1, false);
            // PTT-on is at the start of the chunk; PTT-off at its end (one sample long)
            for e in poll.ptt.iter().filter(|e| e.on) {
                ptt_on = true;
                edges.push((t, e.on));
            }
            let blanked = poll.segments.iter().any(|s| s.blanks_receiver());
            assert_eq!(blanked, ptt_on, "sample {t}");
            for e in poll.ptt.iter().filter(|e| !e.on) {
                ptt_on = false;
                edges.push((t + 1, e.on));
            }
            blank += usize::from(blanked);
            t += 1;
        }
        assert_eq!(blank, 3 + 6 + 4);
        assert_eq!(edges, [(5, true), (18, false)]);
    }
}
