//! The JTTY text packer: a typed message as the operator writes it → the fewest
//! frames' worth of [`Atom`]s.
//!
//! Ported from `pack_jtty` (with `normalize_jtty_message`, `normalize_serials`,
//! `try_compact` and the `consider` / `offer` helpers) in WSJT-X
//! `lib/jtty/jtty_mod.f90`, tag `v3.2.0-rc1`.
//!
//! What is here is the protocol half: **text and an [`ExchangeProfile`] in, atoms
//! out** (then [`tx::tones`] and the synthesiser take them on). What upstream's
//! GUI puts around it — the F-key templates, the N1MM `[[JTTY:<ACTION>]]` tags,
//! deciding the profile from the operating activity — is host policy and stays
//! out, as `QSO`-state code does for FT8 (#463).
//!
//! # What it does
//!
//! 1. **Normalise**: upper-case, NUL and `~` are spaces, runs of spaces collapse
//!    and the ends are trimmed, a character outside the [`ALPHABET`] becomes `#`.
//! 2. **RTTY Roundup only**: `599 <number>` is rewritten to the serial-number
//!    form (`599 5` → `599 005`).
//! 3. **Minimise frames** with a dynamic program over character offsets. At each
//!    offset the candidates are a TEXT5 atom (five characters, or what is left)
//!    and — at the start of a word — every structured atom that renders to
//!    exactly the text there and round-trips through encode → decode → render.
//!    Ties go to a structured atom, then to the longer span, then to the lower
//!    `100·kind + 2·subtype + role` key.
//! 4. The result must fit in [`MAX_FRAMES`] frames.
//!
//! # Upstream quirks kept
//!
//! A two-letter ARRL/RAC section (`DX`, `AB`, `MB`, `PE` …) after a class token is
//! never packed as a class/section atom, only as text: upstream's section lookup
//! rejects an argument shorter than three characters and is handed a trimmed word.
//!
//! # Where it differs from upstream
//!
//! Upstream's `character*80` argument silently truncates a longer message; here
//! more than [`MAX_MESSAGE_CHARS`] characters is [`PackError::TooLong`]. Upstream
//! works on bytes; here a character outside ASCII is one `#`, not one per byte.
//! Neither can matter to an ASCII message of at most 80 characters, which is what
//! the golden test (`tests/jtty_pack.rs`, against upstream's own `pack_jtty`) uses.

use alloc::string::String;
use alloc::vec::Vec;

use super::MAX_FRAMES;
use super::source::{
    ALPHABET, Atom, CONTROL_PHRASES, CallAction, LocationKind, MAX_MESSAGE_CHARS, MAX_NUMBER,
    NumberKind, Role, decode_payload, section_index,
};
#[cfg(doc)]
use super::tx;

/// What the operating activity says about the exchange (`JTTY_EXCHANGE_*`).
///
/// Only [`Self::RttyRoundup`] changes the packing: it adds serial-number and
/// state/province candidates, and rewrites `599 <number>` to the serial form.
/// The others are the same to the packer; they exist because a host tells the
/// three apart.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ExchangeProfile {
    /// No particular exchange (the default).
    #[default]
    Unknown = 0,
    /// ARRL Field Day: `<count><class> <section>` is a single atom in every profile.
    FieldDay = 1,
    /// RTTY Roundup: `599 <serial>` and `599 <state>`.
    RttyRoundup = 2,
}

/// Why a message could not be packed (upstream's `nframes = -1`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PackError {
    /// More than [`MAX_MESSAGE_CHARS`] characters as typed.
    TooLong,
    /// RTTY Roundup's serial rewrite made the message longer than
    /// [`MAX_MESSAGE_CHARS`], or produced a number that cannot be sent.
    Serial,
    /// It needs more than [`MAX_FRAMES`] frames.
    TooManyFrames,
}

impl core::fmt::Display for PackError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::TooLong => "message is longer than 80 characters",
            Self::Serial => "RTTY Roundup serial rewrite does not fit",
            Self::TooManyFrames => "message needs more than 16 frames",
        })
    }
}

#[cfg(feature = "std")]
impl std::error::Error for PackError {}

/// Fold operator text into the source alphabet (`normalize_jtty_message`).
pub fn normalize(text: &str) -> String {
    let mut out = String::new();
    let mut last_space = true;
    for c in text.chars() {
        let c = match c {
            '\0' | '~' => ' ',
            'a'..='z' => c.to_ascii_uppercase(),
            c if c.is_ascii() && ALPHABET.contains(&(c as u8)) => c,
            _ => '#',
        };
        if c == ' ' {
            if !last_space {
                out.push(' ');
            }
            last_space = true;
        } else {
            out.push(c);
            last_space = false;
        }
    }
    if out.ends_with(' ') {
        out.pop();
    }
    out
}

/// `decimal_value`: 1 to 6 digits, at most 131 071.
fn decimal_value(s: &[u8]) -> Option<u32> {
    if s.is_empty() || s.len() > 6 || !s.iter().all(u8::is_ascii_digit) {
        return None;
    }
    let v = s.iter().fold(0u32, |v, &d| 10 * v + u32::from(d - b'0'));
    (v <= MAX_NUMBER).then_some(v)
}

/// `RTTY Roundup`: `599 <decimal>` → the rendered serial atom (`599 005`).
fn normalize_serials(msg: &str) -> Result<String, PackError> {
    let words: Vec<&str> = msg.split(' ').collect();
    let mut out = String::new();
    let mut i = 0;
    while i < words.len() {
        let mut rendered = String::from(words[i]);
        let mut used = 1;
        if words[i] == "599"
            && i + 1 < words.len()
            && let Some(value) = decimal_value(words[i + 1].as_bytes())
        {
            let atom = Atom::Number {
                role: Role::Full,
                kind: NumberKind::Serial,
                value,
            };
            if atom.word().is_none() {
                return Err(PackError::Serial);
            }
            rendered = atom.render();
            used = 2;
        }
        if i > 0 {
            out.push(' ');
        }
        out.push_str(&rendered);
        if out.len() > MAX_MESSAGE_CHARS {
            return Err(PackError::Serial);
        }
        i += used;
    }
    Ok(out)
}

/// `100·kind + 2·subtype + role`, upstream's tie-break key (`JTTY_ATOM_*`).
fn key(atom: &Atom) -> u32 {
    let (kind, subtype, role) = match atom {
        Atom::Call { action, .. } => (0, *action as u32, 0),
        Atom::Number { role, kind, .. } => (1, *kind as u32, *role as u32),
        Atom::Location { role, kind, .. } => (2, *kind as u32, *role as u32),
        Atom::ZoneLoc3 { .. } => (3, 0, 0),
        Atom::ClassSection { .. } => (3, 1, 0),
        Atom::NumberTime { role, .. } => (4, 0, *role as u32),
        Atom::Control(id) => (5, u32::from(*id), 0),
        Atom::Grid4 { role, .. } => (6, 0, *role as u32),
        Atom::Text5(_) => (7, 0, 0),
    };
    100 * kind + 2 * subtype + role
}

const INF: usize = 999;

/// The dynamic program's state: `dp[i]` is the fewest frames for `msg[i..]`.
struct Plan<'a> {
    msg: &'a [u8],
    profile: ExchangeProfile,
    dp: Vec<usize>,
    successor: Vec<usize>,
    choice: Vec<Option<Atom>>,
}

impl Plan<'_> {
    /// `consider`: take `atom`, spanning up to `next`, as the choice at `ipos` if
    /// it is cheaper, or ties and wins on the upstream order.
    fn consider(&mut self, ipos: usize, atom: Atom, next: usize) {
        let cost = 1 + self.dp[next];
        if cost > MAX_FRAMES || cost > self.dp[ipos] {
            return;
        }
        if cost == self.dp[ipos] {
            let best = self.choice[ipos].as_ref().expect("a tie has a choice");
            let (rank, best_rank) = (atom.is_text5() as u8, best.is_text5() as u8);
            if rank > best_rank {
                return;
            }
            if rank == best_rank {
                if next < self.successor[ipos] {
                    return;
                }
                if next == self.successor[ipos] && key(&atom) >= key(best) {
                    return;
                }
            }
        }
        self.dp[ipos] = cost;
        self.successor[ipos] = next;
        self.choice[ipos] = Some(atom);
    }

    /// `offer`: a structured atom counts only if it survives encode → decode →
    /// render and reads exactly the text at `ipos`, followed by a space or the end.
    fn offer(&mut self, ipos: usize, atom: Atom) {
        let n = self.msg.len();
        let Some(payload) = atom.encode(false) else {
            return;
        };
        let Some((decoded, _)) = decode_payload(&payload) else {
            return;
        };
        let rendered = decoded.render();
        let rendered = rendered.trim_end().as_bytes();
        if rendered.is_empty() {
            return;
        }
        let last = ipos + rendered.len();
        if last > n || &self.msg[ipos..last] != rendered {
            return;
        }
        let mut next = n;
        if last < n {
            if self.msg[last] != b' ' {
                return;
            }
            // a structured frame supplies exactly one following separator column
            next = last + 1;
        }
        self.consider(ipos, atom, next);
    }

    /// `try_compact`: every structured atom that could start the word at `ipos`.
    fn try_compact(&mut self, ipos: usize) {
        let n = self.msg.len();
        let mut words: Vec<&[u8]> = Vec::with_capacity(3);
        let mut first = ipos;
        while words.len() < 3 && first < n {
            let end = self.msg[first..]
                .iter()
                .position(|&c| c == b' ')
                .map_or(n, |p| first + p);
            words.push(&self.msg[first..end]);
            first = end + 1;
        }
        let text = |w: &[u8]| String::from_utf8_lossy(w).into_owned();

        for w in &words {
            // `atom%text` is 13 characters
            if w.len() > 13 {
                continue;
            }
            for action in [
                CallAction::Cq,
                CallAction::Call,
                CallAction::TuCq,
                CallAction::CallTu,
                CallAction::CallAgn,
                CallAction::TuNow,
            ] {
                self.offer(
                    ipos,
                    Atom::Call {
                        action,
                        call: text(w),
                    },
                );
            }
        }
        for id in 0..CONTROL_PHRASES.len() {
            self.offer(ipos, Atom::Control(id as u8));
        }

        let rtty = self.profile == ExchangeProfile::RttyRoundup;
        for role in [Role::FieldOnly, Role::Full] {
            let field = if role == Role::Full {
                if words[0] != b"599" || words.len() < 2 {
                    continue;
                }
                words[1]
            } else {
                words[0]
            };
            let numeric = decimal_value(field);
            if let Some(value) = numeric {
                self.offer(
                    ipos,
                    Atom::Number {
                        role,
                        kind: NumberKind::Generic,
                        value,
                    },
                );
                if role == Role::Full && rtty {
                    self.offer(
                        ipos,
                        Atom::Number {
                            role,
                            kind: NumberKind::Serial,
                            value,
                        },
                    );
                }
            }
            if field.len() == 4 {
                self.offer(
                    ipos,
                    Atom::Grid4 {
                        role,
                        grid: text(field),
                    },
                );
            }
            if role != Role::Full || !(2..=3).contains(&field.len()) {
                continue;
            }
            if !field.iter().any(u8::is_ascii_uppercase) {
                continue;
            }
            self.offer(ipos, Atom::location(role, LocationKind::Qth, &text(field)));
            if rtty {
                self.offer(
                    ipos,
                    Atom::location(role, LocationKind::StateProvince, &text(field)),
                );
            }
        }

        let w1 = words[0];
        if words.len() < 2 || !(2..=3).contains(&w1.len()) {
            return;
        }
        let (count, class) = w1.split_at(w1.len() - 1);
        // Upstream's `pack77_arrl_section_index` returns -1 for an argument shorter than
        // three characters, and `pack_jtty` hands it `trim(word)`: a two-letter section
        // (`DX`, `AB`, `MB` …) is therefore never a class/section atom there, and it
        // matches on the first three characters of a longer word. Reproduced as it is
        // (the golden file has `1F DX`, which upstream sends as TEXT5).
        let name = words[1];
        let section = if name.len() >= 3 {
            section_index(&text(&name[..3]))
        } else {
            None
        };
        if let (Some(count), Some(section)) = (decimal_value(count), section) {
            // 1..=32 are the only counts an atom can carry
            if let Ok(count) = u8::try_from(count) {
                self.offer(
                    ipos,
                    Atom::ClassSection {
                        count,
                        class: class[0] as char,
                        section,
                    },
                );
            }
        }
    }
}

/// Pack `text` into the fewest atoms (`pack_jtty`). An empty message is `Ok` and
/// has no atoms.
///
/// The atoms are what [`tx::tones`] takes; the last carries the end-of-message flag
/// when it is encoded.
pub fn pack(text: &str, profile: ExchangeProfile) -> Result<Vec<Atom>, PackError> {
    if text.chars().count() > MAX_MESSAGE_CHARS {
        return Err(PackError::TooLong);
    }
    let mut msg = normalize(text);
    if profile == ExchangeProfile::RttyRoundup {
        msg = normalize_serials(&msg)?;
    }
    let n = msg.len();
    if n == 0 {
        return Ok(Vec::new());
    }
    let mut plan = Plan {
        msg: msg.as_bytes(),
        profile,
        dp: {
            let mut d = alloc::vec![INF; n + 1];
            d[n] = 0;
            d
        },
        successor: alloc::vec![0; n + 1],
        choice: alloc::vec![None; n + 1],
    };
    for ipos in (0..n).rev() {
        let inext = n.min(ipos + 5);
        plan.consider(ipos, Atom::Text5(msg[ipos..inext].into()), inext);
        if ipos > 0 && plan.msg[ipos - 1] != b' ' {
            continue;
        }
        plan.try_compact(ipos);
    }
    if plan.dp[0] > MAX_FRAMES {
        return Err(PackError::TooManyFrames);
    }
    let mut atoms = Vec::new();
    let mut ipos = 0;
    while ipos < n {
        atoms.push(plan.choice[ipos].take().expect("reachable"));
        ipos = plan.successor[ipos];
    }
    Ok(atoms)
}

/// [`pack`], then the channel tones of the message ([`tx::tones`]); `None` for an
/// empty message (there is nothing to send).
pub fn tones(text: &str, profile: ExchangeProfile) -> Result<Option<Vec<u8>>, PackError> {
    Ok(super::tx::tones(&pack(text, profile)?))
}
