//! Assembling decoded frames into messages.
//!
//! Ported from WSJT-X `lib/jtty/jtty_mdecode.f90` (`classify_active_candidate`,
//! `start_message`, `append_active_message`, `prune_receive_state`,
//! `queue_message_update`, `display_message_text`, and the merge at the end of
//! `decode_and_merge`), tag `v3.2.0-rc1`.
//!
//! A message is one or more frames on one frequency, a frame period (1.888 s)
//! apart. The hand-tuned constants are upstream's, carried verbatim (D5 of
//! `docs/notes/JTTY_UPSTREAM.md`):
//!
//! - a frame **continues** an active message if it starts 1‥3 frame periods after
//!   the message's last frame (±0.1 s) and within `10 + 3(n−1)` Hz of it — after a
//!   gap of `n > 1` periods, the missing text is marked `~~~~~`, shown as ` ... `;
//! - a frame that is the same transmission re-decoded by a neighbouring window
//!   (quarter-frame offsets within 10 Hz) is absorbed, as is one that matches a
//!   *recent* frame within 12 Hz and 50 ms;
//! - at most 30 messages are active; one that has had no continuation for more
//!   than 3 frame periods (measured from the oldest window a retro re-sweep can
//!   still reach) is reported incomplete and dropped;
//! - end of message (`eom`) completes a message; text is capped at 80 characters.
//!
//! Frame text follows upstream's convention so that lengths match: TEXT5 spaces
//! are `~`, a structured atom is followed by one implicit separating space that
//! upstream's character counter does not include.

use alloc::string::String;
use alloc::vec::Vec;
#[cfg(not(feature = "std"))]
#[allow(unused_imports)]
use num_traits::Float;

use super::dsp::NSS;
use super::rx::FrameDecode;
use super::source::Atom;
use super::{FRAME_SYMBOLS, MAX_FRAMES};

/// Frame period, seconds: 59 symbols of 192 samples at 6 kHz.
pub const FRAME_PERIOD_S: f32 = (FRAME_SYMBOLS * NSS) as f32 / 6_000.0;
/// Active messages kept at once.
pub const MAX_ACTIVE: usize = 30;
/// Frame periods a message may skip and still be continued.
pub const MAX_CONTINUATION_GAP: usize = 3;
/// Quarter-frame windows a retro re-sweep revisits.
pub const MAX_RETRO_STEPS: usize = 3;
/// Characters a message keeps.
const MAX_CHARS: usize = 80;
const FRAME_HISTORY_TIME_S: f32 = 0.05;
const NEAR_SIMULTANEOUS_HZ: f32 = 12.0;
const CONTINUATION_TIME_S: f32 = 0.1;
const _: () = assert!(MAX_MESSAGES_BOUND >= MAX_ACTIVE);
const MAX_MESSAGES_BOUND: usize = MAX_ACTIVE * MAX_FRAMES;

/// A message as far as it is known; one is emitted every time it grows, and once
/// more when it completes or is given up on.
#[derive(Clone, Debug, PartialEq)]
pub struct MessageUpdate {
    /// Stable for the life of the message.
    pub id: u64,
    /// Frequency of the latest frame, Hz.
    pub f1_hz: f32,
    /// Start of the first frame, seconds from the start of the audio.
    pub start_s: f32,
    /// The text so far, as upstream displays it (gaps as ` ... `).
    pub text: String,
    /// The end-of-message frame has arrived.
    pub complete: bool,
}

/// A frame as text: `decoded` (`~` for TEXT5 spaces), whether a structured atom's
/// implicit separator follows, and the EOM flag.
struct FrameText {
    decoded: String,
    trailing_sep: bool,
    last: bool,
}

fn frame_text(atom: &Atom, eom: bool) -> FrameText {
    match atom {
        Atom::Text5(t) => FrameText {
            decoded: t.chars().map(|c| if c == ' ' { '~' } else { c }).collect(),
            trailing_sep: false,
            last: eom,
        },
        other => FrameText {
            decoded: other.render(),
            trailing_sep: true,
            last: eom,
        },
    }
}

struct Active {
    id: u64,
    f1: f32,
    tsync: f32,
    start: f32,
    /// upstream's character counter `k` (it does not count implicit separators)
    k: usize,
    decoded: String,
    trailing_sep: bool,
}

struct Recent {
    f1: f32,
    tsync: f32,
}

/// How a candidate relates to an active message.
struct Classified {
    is_match: bool,
    window_dupe: bool,
    gap: usize,
}

fn classify(existing: &Active, f1: f32, tsync: f32) -> Classified {
    let (df1, dtsync) = (f1 - existing.f1, tsync - existing.tsync);
    let nfp = (dtsync / FRAME_PERIOD_S).round();
    let fp_resid = (dtsync - FRAME_PERIOD_S * nfp).abs();
    let mut c = Classified {
        is_match: false,
        window_dupe: false,
        gap: 1,
    };
    if nfp >= 1.0 && nfp <= MAX_CONTINUATION_GAP as f32 && fp_resid < CONTINUATION_TIME_S {
        let df_tol = 10.0 + 3.0 * (nfp - 1.0);
        if df1.abs() < df_tol {
            c.is_match = true;
            c.gap = nfp as usize;
        }
    }
    if !c.is_match {
        let qstep = FRAME_PERIOD_S / 4.0;
        let nstep = (dtsync / qstep).round();
        let resid = (dtsync - qstep * nstep).abs();
        if nstep.abs() <= MAX_RETRO_STEPS as f32
            && df1.abs() < 10.0
            && resid < 0.003
            && !((nstep.abs() as usize).is_multiple_of(4) && nstep != 0.0)
        {
            c.is_match = true;
            c.window_dupe = true;
        }
    }
    c
}

/// `display_message_text`: `~~~~~` (a gap) becomes ` ... `, every other `~` a
/// space, and one leading space is dropped.
fn display(decoded: &str) -> String {
    let s = decoded.replace("~~~~~", " ... ").replace('~', " ");
    let s = s.strip_prefix(' ').unwrap_or(&s);
    s.trim_end().into()
}

/// The receive-side message state: active messages and recently seen frames.
#[derive(Default)]
pub struct Assembler {
    active: Vec<Active>,
    recent: Vec<Recent>,
    next_id: u64,
    /// Frames decoded and subtracted in earlier windows that may still overlap a
    /// window to come (see [`super::rx::Params::carry`]).
    pub(super) carried: Vec<super::rx::Subtracted>,
}

impl Assembler {
    /// An empty assembler.
    pub fn new() -> Self {
        Self {
            next_id: 1,
            ..Self::default()
        }
    }

    /// `(frequency, start of last frame)` of every message that may still be
    /// continued — what a sticky-sync retry looks at.
    pub fn continuations(&self) -> impl Iterator<Item = (f32, f32)> + '_ {
        self.active.iter().map(|a| (a.f1, a.tsync))
    }

    /// Age out state before a new forward window starting `forward_tsync` seconds
    /// into the audio (`prune_receive_state`): frames older than the oldest window
    /// a retro re-sweep can revisit are forgotten, and messages with no
    /// continuation within three frame periods of that are reported incomplete.
    pub fn prune(&mut self, forward_tsync: f32, sink: &mut dyn FnMut(MessageUpdate)) {
        // a decoded frame can still lie in a later window while its end is after that window's start
        self.carried
            .retain(|x| x.tsync_s + FRAME_PERIOD_S > forward_tsync);
        let oldest_revisit = forward_tsync - MAX_RETRO_STEPS as f32 * FRAME_PERIOD_S / 4.0;
        self.recent
            .retain(|r| r.tsync >= oldest_revisit - FRAME_HISTORY_TIME_S);
        let limit = MAX_CONTINUATION_GAP as f32 * FRAME_PERIOD_S + CONTINUATION_TIME_S;
        let mut i = 0;
        while i < self.active.len() {
            if oldest_revisit - self.active[i].tsync > limit {
                sink(self.update(i, false));
                self.remove(i);
            } else {
                i += 1;
            }
        }
    }

    fn remove(&mut self, i: usize) {
        // upstream moves the last message into the freed slot
        self.active.swap_remove(i);
    }

    fn update(&self, i: usize, complete: bool) -> MessageUpdate {
        let a = &self.active[i];
        MessageUpdate {
            id: a.id,
            f1_hz: a.f1,
            start_s: a.start,
            text: display(&a.decoded),
            complete,
        }
    }

    fn is_recent(&self, f1: f32, tsync: f32) -> bool {
        self.recent.iter().any(|r| {
            (f1 - r.f1).abs() < NEAR_SIMULTANEOUS_HZ
                && (tsync - r.tsync).abs() < FRAME_HISTORY_TIME_S
        })
    }

    fn remember(&mut self, f1: f32, tsync: f32) {
        // upstream bounds the history; the oldest entry goes first
        if self.recent.len() >= MAX_MESSAGES_BOUND {
            self.recent.remove(0);
        }
        self.recent.push(Recent { f1, tsync });
    }

    /// Merge one decoded frame. Returns `false` if the frame was absorbed as a
    /// repeat of one already seen (or could not be started), `true` if it
    /// contributed to a message.
    pub fn push_frame(&mut self, f: &FrameDecode, sink: &mut dyn FnMut(MessageUpdate)) -> bool {
        let text = frame_text(&f.atom, f.eom);
        let (f1, tsync) = (f.f1_hz, f.tsync_s);
        let mut pure_dupe = self.is_recent(f1, tsync);
        let mut target: Option<(usize, usize)> = None; // (index, gap)

        if !pure_dupe && !self.active.is_empty() {
            let (mut have_window, mut best_df, mut best) = (false, f32::MAX, None);
            for (i, a) in self.active.iter().enumerate() {
                let c = classify(a, f1, tsync);
                if !c.is_match {
                    continue;
                }
                if c.window_dupe {
                    have_window = true;
                    continue;
                }
                let df = (f1 - a.f1).abs();
                if df < best_df {
                    best_df = df;
                    best = Some((i, c.gap));
                }
            }
            if have_window {
                pure_dupe = true;
            } else {
                target = best;
            }
        }
        if pure_dupe {
            return false;
        }
        match target {
            Some((i, gap)) => self.append(i, gap, f1, tsync, &text, sink),
            None => self.start(f1, tsync, &text, sink),
        }
    }

    fn start(
        &mut self,
        f1: f32,
        tsync: f32,
        t: &FrameText,
        sink: &mut dyn FnMut(MessageUpdate),
    ) -> bool {
        if !t.last && self.active.len() >= MAX_ACTIVE {
            return false;
        }
        self.remember(f1, tsync);
        let mut decoded = t.decoded.clone();
        if decoded.starts_with("599 ") {
            decoded.insert(0, '~');
        }
        let msg = Active {
            id: self.next_id,
            f1,
            tsync,
            start: tsync,
            k: decoded.chars().count(),
            decoded,
            trailing_sep: t.trailing_sep,
        };
        self.next_id += 1;
        self.active.push(msg);
        let i = self.active.len() - 1;
        sink(self.update(i, t.last));
        if t.last {
            self.remove(i);
        }
        true
    }

    fn append(
        &mut self,
        i: usize,
        gap: usize,
        f1: f32,
        tsync: f32,
        t: &FrameText,
        sink: &mut dyn FnMut(MessageUpdate),
    ) -> bool {
        self.remember(f1, tsync);
        let a = &mut self.active[i];
        let (k, n) = (a.k, t.decoded.chars().count());
        let chars: Vec<char> = t.decoded.chars().collect();
        if gap > 1 {
            let nstart = usize::from(chars.first() == Some(&'~'));
            let kz = (k + 5 + (n - nstart)).min(MAX_CHARS);
            let nchar = kz.saturating_sub(k + 5);
            a.decoded.push_str("~~~~~");
            a.decoded.extend(&chars[nstart..nstart + nchar]);
            a.k = k + 5 + nchar;
        } else {
            let kz = (k + n).min(MAX_CHARS);
            let take = kz.saturating_sub(k).min(n);
            if a.trailing_sep {
                a.decoded.push(' ');
            }
            a.decoded.extend(&chars[..take]);
            a.k = kz;
        }
        a.trailing_sep = t.trailing_sep;
        a.f1 = f1;
        a.tsync = tsync;
        sink(self.update(i, t.last));
        if t.last {
            self.remove(i);
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::jtty::source::{CallAction, NumberKind, Role};

    fn frame(atom: Atom, f1: f32, tsync: f32, eom: bool) -> FrameDecode {
        FrameDecode {
            channel: 0,
            f1_hz: f1,
            xdt_s: 0.0,
            tsync_s: tsync,
            snr_db: 10.0,
            nsync: 13,
            nsymerrs: 0,
            payload: [0; 34],
            atom,
            eom,
            rung: 1,
            rank: 1,
            pool: 1,
        }
    }

    fn run(frames: &[FrameDecode]) -> Vec<MessageUpdate> {
        let mut asm = Assembler::new();
        let mut out = Vec::new();
        for f in frames {
            asm.push_frame(f, &mut |u| out.push(u));
        }
        out
    }

    #[test]
    fn three_text_frames_make_one_message() {
        let p = FRAME_PERIOD_S;
        let u = run(&[
            frame(Atom::text5("HELLO"), 1500.0, 1.0, false),
            frame(Atom::text5(" WORL"), 1500.5, 1.0 + p, false),
            frame(Atom::text5("D 73 "), 1500.2, 1.0 + 2.0 * p, true),
        ]);
        assert_eq!(u.len(), 3);
        assert!(u.iter().all(|x| x.id == u[0].id));
        assert_eq!(u[0].text, "HELLO");
        assert_eq!(u[1].text, "HELLO WORL");
        assert_eq!(u[2].text, "HELLO WORLD 73");
        assert_eq!(
            u.iter().map(|x| x.complete).collect::<Vec<_>>(),
            [false, false, true]
        );
        assert_eq!(u[2].start_s, 1.0);
    }

    #[test]
    fn structured_atoms_are_separated_by_a_space() {
        let p = FRAME_PERIOD_S;
        let u = run(&[
            frame(Atom::call(CallAction::Call, "WB9XYZ"), 1500.0, 0.3, false),
            frame(
                Atom::Number {
                    role: Role::Full,
                    kind: NumberKind::Generic,
                    value: 123,
                },
                1500.0,
                0.3 + p,
                true,
            ),
        ]);
        assert_eq!(u.last().unwrap().text, "WB9XYZ 599 123");
    }

    #[test]
    fn a_missing_frame_shows_as_a_gap() {
        let p = FRAME_PERIOD_S;
        let u = run(&[
            frame(Atom::text5("RAN A"), 1500.0, 1.0, false),
            // one frame period skipped
            frame(Atom::text5("GHT O"), 1500.0, 1.0 + 2.0 * p, true),
        ]);
        assert_eq!(u.last().unwrap().text, "RAN A ... GHT O");
    }

    #[test]
    fn a_neighbouring_window_seeing_the_same_frame_is_absorbed() {
        let p = FRAME_PERIOD_S;
        let mut asm = Assembler::new();
        let mut n = 0;
        let mut sink = |_u: MessageUpdate| n += 1;
        assert!(asm.push_frame(&frame(Atom::text5("HELLO"), 1500.0, 1.0, false), &mut sink));
        // the same frame, a quarter-frame window later, 3 Hz off: absorbed
        assert!(!asm.push_frame(
            &frame(Atom::text5("HELLO"), 1503.0, 1.0 + p / 4.0, false),
            &mut sink
        ));
        // the same frame at nearly the same time: also absorbed (recent frame)
        assert!(!asm.push_frame(&frame(Atom::text5("HELLO"), 1505.0, 1.02, false), &mut sink));
        assert_eq!(n, 1);
    }

    #[test]
    fn two_stations_at_different_frequencies_are_two_messages() {
        let p = FRAME_PERIOD_S;
        let u = run(&[
            frame(Atom::text5("AAAAA"), 1500.0, 1.0, false),
            frame(Atom::text5("BBBBB"), 1700.0, 1.2, false),
            frame(Atom::text5("CCCCC"), 1701.0, 1.2 + p, true),
            frame(Atom::text5("DDDDD"), 1500.5, 1.0 + p, true),
        ]);
        let done: Vec<&MessageUpdate> = u.iter().filter(|x| x.complete).collect();
        assert_eq!(done.len(), 2);
        assert_eq!(done[0].text, "BBBBBCCCCC");
        assert_eq!(done[1].text, "AAAAADDDDD");
    }

    #[test]
    fn a_message_nobody_continues_is_reported_incomplete_and_dropped() {
        let p = FRAME_PERIOD_S;
        let mut asm = Assembler::new();
        let mut out = Vec::new();
        asm.push_frame(&frame(Atom::text5("LOST "), 1500.0, 1.0, false), &mut |u| {
            out.push(u)
        });
        asm.prune(1.0 + p, &mut |u| out.push(u));
        assert_eq!(out.len(), 1, "still active");
        asm.prune(1.0 + 4.5 * p, &mut |u| out.push(u));
        assert_eq!(out.len(), 2);
        assert!(!out[1].complete);
        assert_eq!(out[1].text, "LOST");
        assert_eq!(asm.continuations().count(), 0);
    }

    #[test]
    fn full_role_exchange_gets_a_leading_marker_that_display_strips() {
        let u = run(&[frame(
            Atom::Number {
                role: Role::Full,
                kind: NumberKind::Serial,
                value: 5,
            },
            1500.0,
            1.0,
            true,
        )]);
        assert_eq!(u[0].text, "599 005");
    }

    #[test]
    fn text_is_capped_at_eighty_characters() {
        let p = FRAME_PERIOD_S;
        let frames: Vec<FrameDecode> = (0..16)
            .map(|i| frame(Atom::text5("ABCDE"), 1500.0, 1.0 + i as f32 * p, i == 15))
            .collect();
        assert_eq!(run(&frames).last().unwrap().text.len(), MAX_CHARS);
    }
}
