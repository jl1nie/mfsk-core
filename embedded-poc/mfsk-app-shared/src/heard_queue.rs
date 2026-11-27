//! The queue between the decoders and the QSO machine (Issue #766, `docs/notes/PROFILES.md` section 10, D8).
//!
//! Two producers decode messages addressed to this station: the window path (core 0, before the reply deadline) and the late path (core 1, SIC after it). The **QSO task owns the clock**:
//! once per own transmit period, at the reply deadline, it empties the queue with [`HeardCollector::collect`] and hands the result to [`Activator::decide`](crate::activator::Activator::decide).
//!
//! - A producer's [`HeardQueue::push`] never blocks and never waits for the consumer: a full queue drops its **oldest** item (a late decode is the least valuable).
//! - The consumer drops an item older than [`HEARD_TTL`] own periods (the activator's own `PENDING_TTL`), and **a message already delivered for the same period** (the same 77 bits decoded in the same
//!   period: the window path's decode and the late path's decode of one signal) is not delivered again. **The same bits in a later period are a new message**: a partner who did not copy our
//!   reply sends it again, and the activator answers a repeat.
//!
//! Pure and `no_std` (a `heapless` MPMC queue: lock-free, safe for two producers on two cores); `hosttest/mfsk-app-shared` runs the tests.

use heapless::Vec;
use heapless::mpmc::MpMcQueue;

use crate::activator::Heard;

/// Items the queue holds (a power of two). A busy band has fewer messages addressed to one station in a period than this.
pub const QUEUE_CAP: usize = 16;
/// Own periods a decode stays deliverable, and a delivered message stays remembered: the activator's `PENDING_TTL`.
pub const HEARD_TTL: u32 = 2;
/// Delivered messages remembered (to refuse a second delivery of the same 77 bits).
const SEEN_CAP: usize = 32;

/// One decode addressed to this station.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HeardItem {
    /// The 77 message bits packed by [`pack_bits`]: what two decodes of one signal have in common.
    pub id: u128,
    /// The own transmit period the decode belongs to (the slot it was heard in, as the QSO task counts periods).
    pub period: u32,
    pub heard: Heard,
}

/// The 77 message bits (one per byte, 0 or 1) as a number. Bits beyond the 77th are ignored.
pub fn pack_bits(bits: &[u8]) -> u128 {
    bits.iter()
        .take(77)
        .fold(0u128, |acc, &b| (acc << 1) | (b & 1) as u128)
}

/// A [`HeardItem`] from a decoded message's text (`CALL1 CALL2 EXCHANGE`, the rendered standard message) and its 77 bits, if it is addressed to `my_call` and carries an exchange an
/// activator answers (`Heard::parse`'s rules). Free text, CQs and messages of more than three fields give `None`.
pub fn heard_item_from_text(
    my_call: &str,
    text: &str,
    bits: &[u8],
    snr_db: f32,
    period: u32,
) -> Option<HeardItem> {
    let mut f = text.split_whitespace();
    let (call1, call2) = (f.next()?, f.next()?);
    let exchange = f.next().unwrap_or("");
    if f.next().is_some() {
        return None;
    }
    let snr = snr_db.round().clamp(-128.0, 127.0) as i8;
    Some(HeardItem {
        id: pack_bits(bits),
        period,
        heard: Heard::parse(my_call, call1, call2, exchange, snr)?,
    })
}

/// The producers' end: `static HEARD: HeardQueue = HeardQueue::new();`.
pub struct HeardQueue(MpMcQueue<HeardItem, QUEUE_CAP>);

impl Default for HeardQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl HeardQueue {
    pub const fn new() -> Self {
        Self(MpMcQueue::new())
    }

    /// Queue `item`. Returns `false` if the queue was full and the **oldest** item was dropped to make room (the new one is always kept).
    pub fn push(&self, mut item: HeardItem) -> bool {
        let mut dropped = false;
        // two tries are enough: a competing producer can refill the slot we freed once; then the third attempt is the caller's problem (the item is lost, reported false)
        for _ in 0..3 {
            match self.0.enqueue(item) {
                Ok(()) => return !dropped,
                Err(back) => {
                    item = back;
                    let _ = self.0.dequeue();
                    dropped = true;
                }
            }
        }
        false
    }

    fn pop(&self) -> Option<HeardItem> {
        self.0.dequeue()
    }
}

/// The consumer's end, owned by the QSO task.
pub struct HeardCollector {
    /// `(id, the period of the item)`: what was delivered, until the item's period is older than the TTL.
    seen: Vec<(u128, u32), SEEN_CAP>,
    out: Vec<Heard, QUEUE_CAP>,
}

impl Default for HeardCollector {
    fn default() -> Self {
        Self::new()
    }
}

impl HeardCollector {
    pub const fn new() -> Self {
        Self {
            seen: Vec::new(),
            out: Vec::new(),
        }
    }

    /// Everything addressed to this station decoded since the last call, in arrival order, for `Activator::decide(period, ..)`: items older than [`HEARD_TTL`] periods and messages already
    /// delivered within it are left out. The slice is valid until the next call.
    pub fn collect(&mut self, q: &HeardQueue, period: u32) -> &[Heard] {
        self.seen
            .retain(|&(_, p)| period.saturating_sub(p) <= HEARD_TTL);
        self.out.clear();
        while let Some(item) = q.pop() {
            if period.saturating_sub(item.period) > HEARD_TTL {
                continue;
            }
            if self
                .seen
                .iter()
                .any(|&(id, p)| id == item.id && p == item.period)
            {
                continue;
            }
            if self.seen.is_full() {
                self.seen.remove(0);
            }
            let _ = self.seen.push((item.id, item.period));
            let _ = self.out.push(item.heard);
        }
        &self.out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(id: u128, period: u32, from: &str) -> HeardItem {
        HeardItem {
            id,
            period,
            heard: Heard::parse("JL1NIE", "JL1NIE", from, "PM95", -10).expect("addressed to us"),
        }
    }

    fn calls(v: &[Heard]) -> std::vec::Vec<&str> {
        v.iter().map(|h| h.from.as_str()).collect()
    }

    /// The same 77 bits from the window path and from the late path are one message. **Fails if** the consumer delivers a duplicate (remove the `seen` check).
    #[test]
    fn a_message_decoded_twice_is_delivered_once() {
        let (q, mut c) = (HeardQueue::new(), HeardCollector::new());
        q.push(item(7, 5, "JA1AAA"));
        q.push(item(7, 5, "JA1AAA"));
        q.push(item(8, 5, "JA1BBB"));
        assert_eq!(calls(c.collect(&q, 5)), ["JA1AAA", "JA1BBB"]);
    }

    /// A late decode of a message already delivered (the same bits, the same period) is not delivered again; **the same bits in a later period are a new message** (a partner repeating himself:
    /// the activator answers a repeat, also in the very next period). **Fails if** the late copy is delivered twice, or a repeat is swallowed by the memory of the first.
    #[test]
    fn a_repeat_in_a_later_period_is_a_new_message() {
        let (q, mut c) = (HeardQueue::new(), HeardCollector::new());
        q.push(item(7, 5, "JA1AAA"));
        assert_eq!(c.collect(&q, 5).len(), 1);
        q.push(item(7, 5, "JA1AAA")); // the late path's copy, one period on
        assert!(c.collect(&q, 6).is_empty());
        q.push(item(7, 6, "JA1AAA")); // the partner sent it again in the next period
        assert_eq!(c.collect(&q, 6).len(), 1);
        q.push(item(7, 6, "JA1AAA")); // and the late path's copy of that
        assert!(c.collect(&q, 7).is_empty());
    }

    /// A decode older than the TTL is not worth answering from. **Fails if** an item of period 5 reaches `decide` at period 8 (but one at period 6 does).
    #[test]
    fn a_stale_item_is_dropped() {
        let (q, mut c) = (HeardQueue::new(), HeardCollector::new());
        q.push(item(1, 5, "JA1OLD"));
        q.push(item(2, 6, "JA1NEW"));
        assert_eq!(calls(c.collect(&q, 8)), ["JA1NEW"]);
    }

    /// A full queue keeps the newest and drops the oldest, and says so. **Fails if** the new item is refused instead (a producer would lose the latest decode).
    #[test]
    fn a_full_queue_drops_its_oldest() {
        let (q, mut c) = (HeardQueue::new(), HeardCollector::new());
        for i in 0..QUEUE_CAP as u128 {
            assert!(q.push(item(i, 5, "JA1AAA")));
        }
        assert!(
            !q.push(item(1000, 5, "JA1ZZZ")),
            "full: the oldest was dropped"
        );
        let got = c.collect(&q, 5);
        assert_eq!(got.len(), QUEUE_CAP);
        assert_eq!(got.last().unwrap().from.as_str(), "JA1ZZZ");
    }

    /// Nothing is delivered twice by two calls in a row, and an empty queue gives an empty slice. **Fails if** `collect` returns the previous call's items again.
    #[test]
    fn a_second_call_with_nothing_new_is_empty() {
        let (q, mut c) = (HeardQueue::new(), HeardCollector::new());
        q.push(item(1, 5, "JA1AAA"));
        assert_eq!(c.collect(&q, 5).len(), 1);
        assert!(c.collect(&q, 5).is_empty());
    }

    /// Two producers on two threads (the window task and the late SIC) lose nothing and duplicate nothing while the queue has room for them. **Fails if** the queue is not safe
    /// for two producers (a lost or doubled item).
    #[test]
    fn two_producers_lose_nothing() {
        static Q: HeardQueue = HeardQueue::new();
        let a =
            std::thread::spawn(|| (0..6u128).for_each(|i| assert!(Q.push(item(i, 5, "JA1AAA")))));
        let b = std::thread::spawn(|| {
            (100..106u128).for_each(|i| assert!(Q.push(item(i, 5, "JA1BBB"))))
        });
        a.join().unwrap();
        b.join().unwrap();
        let mut c = HeardCollector::new();
        let got = c.collect(&Q, 5);
        assert_eq!(got.len(), 12);
        assert_eq!(
            got.iter().filter(|h| h.from.as_str() == "JA1AAA").count(),
            6
        );
    }

    /// What the decoders hand over is the rendered text and the bits: only a message to us with an exchange comes out, SNR rounded. **Fails if** a CQ, a message to someone else, free text
    /// or a four-field message is queued, or the report is lost.
    #[test]
    fn only_a_message_to_us_becomes_an_item() {
        let bits = [1u8; 77];
        let it = heard_item_from_text("JL1NIE", "JL1NIE JA1AAA -07", &bits, -6.6, 4).unwrap();
        assert_eq!(
            (it.period, it.heard.from.as_str(), it.heard.snr),
            (4, "JA1AAA", -7)
        );
        assert!(heard_item_from_text("JL1NIE", "JL1NIE JA1AAA PM95", &bits, 0.0, 4).is_some());
        assert!(heard_item_from_text("JL1NIE", "JL1NIE JA1AAA RR73", &bits, 0.0, 4).is_some());
        assert!(heard_item_from_text("JL1NIE", "CQ JA1AAA PM95", &bits, 0.0, 4).is_none());
        assert!(heard_item_from_text("JL1NIE", "JA1BBB JA1AAA -07", &bits, 0.0, 4).is_none());
        assert!(heard_item_from_text("JL1NIE", "JL1NIE JA1AAA R -07 X", &bits, 0.0, 4).is_none());
        assert!(heard_item_from_text("JL1NIE", "JL1NIE", &bits, 0.0, 4).is_none());
    }

    /// The queue and the activator over own periods in virtual time: a caller decoded by the window path is answered in that period; the same message decoded again by the late path (the same
    /// 77 bits) is **not** a second call; a reply that only the late path decodes (after its deadline) is answered one period later, and nobody is answered twice.
    /// **Fails if** a late decode is lost, the duplicate makes a second answer, or the late reply is answered in the period it was missed.
    #[test]
    fn a_late_decode_is_answered_one_period_on_and_nobody_twice() {
        use crate::activator::{Activator, Config, TxMsg};
        let cfg = Config::new("JL1NIE", "PM95");
        let mut act = Activator::new(cfg.clone());
        act.start();
        let (q, mut c) = (HeardQueue::new(), HeardCollector::new());
        let text = |d: &crate::activator::Decision| d.tx.as_ref().map(|t: &TxMsg| t.text(&cfg));
        let bits = |n: u8| {
            let mut b = [0u8; 77];
            b[0] = n;
            b[76] = 1;
            b
        };
        // period 1: nobody yet -> CQ
        let d = act.decide(1, c.collect(&q, 1));
        assert_eq!(text(&d).as_deref(), Some("CQ JL1NIE PM95"));
        // period 2: a caller, decoded by the window path before the deadline
        q.push(heard_item_from_text("JL1NIE", "JL1NIE JA1AAA PM95", &bits(1), -10.0, 2).unwrap());
        let d = act.decide(2, c.collect(&q, 2));
        let first = text(&d).expect("answers the caller");
        assert!(first.starts_with("JA1AAA JL1NIE"), "{first}");
        // the late path decodes the same call again after the deadline, and the caller's report, which only it hears
        q.push(heard_item_from_text("JL1NIE", "JL1NIE JA1AAA PM95", &bits(1), -10.0, 2).unwrap());
        q.push(heard_item_from_text("JL1NIE", "JL1NIE JA1AAA R-10", &bits(2), -10.0, 2).unwrap());
        // period 3: the duplicate is dropped, the report is the news: RR73 and the contact logged
        let got = c.collect(&q, 3).to_vec();
        assert_eq!(
            got.len(),
            1,
            "the duplicate of the first call is not delivered again"
        );
        let d = act.decide(3, &got);
        assert_eq!(text(&d).as_deref(), Some("JA1AAA JL1NIE RR73"));
        assert_eq!(d.logged_total, 1);
    }

    #[test]
    fn the_77_bits_pack_to_one_number() {
        let mut bits = [0u8; 77];
        bits[76] = 1;
        assert_eq!(pack_bits(&bits), 1);
        bits[0] = 1;
        assert_eq!(pack_bits(&bits), 1 | (1u128 << 76));
        assert_ne!(pack_bits(&bits), pack_bits(&[0u8; 77]));
    }
}
