//! JTTY source encoding: the 32-bit grammar word and its atoms.
//!
//! Ported from WSJT-X `lib/jtty/jtty_source_codec.f90` (`pack_jtty_atom`,
//! `unpack_jtty_atom`, `render_jtty_atom`) against the normative text in
//! `lib/jtty/jtty_source_encoding.txt`, tag `v3.2.0-rc1`.
//!
//! A frame carries one **atom** in a 32-bit word (MSB first; bits 31–32 are
//! `i2`):
//!
//! | `i2` | atom |
//! |---|---|
//! | 0 | call28 + action `n2` ∈ 0‥3: `CQ <call> CQ`, `<call>`, `TU <call> CQ`, `<call> TU` |
//! | 1 | call28 + `n2` ∈ 0‥1: `<call> AGN?`, `TU NOW <call>` |
//! | 2 | STRUCT30: a 27-bit body and a 3-bit family (exchange number / location / pair / number+time / control and grid) |
//! | 3 | TEXT5: five 6-bit characters |
//!
//! **Validity is part of decoding.** [`decode_payload`] returns `None` for a
//! word that upstream's receiver would discard before display, end-of-message
//! handling, message assembly *and signal subtraction*: the reserved bit set,
//! the all-zero word, an unassigned type / family / subtype / enum, an
//! out-of-range field, a non-zero reserved field, a non-canonical base-36
//! token, or a call that is not a standard call. A port that skipped any of
//! these would widen the false-decode surface, which is why they are all here
//! and all tested.
//!
//! Upstream's text packer (`pack_jtty`) is not ported (phase P5): this module
//! takes typed [`Atom`]s.

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::Write as _;

use super::{PAYLOAD_BITS, Payload};
use crate::msg::wsjt77;

/// The 64-character TEXT5 alphabet (`ALPHABET` in `jtty_source_codec.f90`);
/// a character's index is its 6-bit code.
pub const ALPHABET: &[u8; 64] =
    b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ +-./?!\"#$%,&*()_'=[]{}<>|:;";

/// The 18 registered control phrases (`CONTROL_TEXT`), by id.
pub const CONTROL_PHRASES: [&str; 18] = [
    "AGN?", "CALL?", "AGN CALL", "NR?", "AGN NR", "EXCH?", "STATE?", "SECTION?", "ZONE?", "GRID?",
    "RPRT?", "QSL TU", "TU", "QRZ?", "QSO B4", "WAIT", "NIL?", "OK?",
];

/// Largest value of a 17-bit exchange number.
pub const MAX_NUMBER: u32 = 131_071;
/// Number of Maidenhead 4-character grids, `18·18·10·10`.
pub const GRID4_COUNT: u32 = 32_400;
/// Most characters a rendered message keeps (`message` is `character*80`).
pub const MAX_MESSAGE_CHARS: usize = 80;

const BASE36: &[u8; 36] = b"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

/// The six call-word actions.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CallAction {
    /// `CQ <call> CQ`
    Cq = 0,
    /// `<call>`
    Call = 1,
    /// `TU <call> CQ`
    TuCq = 2,
    /// `<call> TU`
    CallTu = 3,
    /// `<call> AGN?`
    CallAgn = 4,
    /// `TU NOW <call>`
    TuNow = 5,
}

impl CallAction {
    fn from_index(i: u32) -> Option<Self> {
        Some(match i {
            0 => Self::Cq,
            1 => Self::Call,
            2 => Self::TuCq,
            3 => Self::CallTu,
            4 => Self::CallAgn,
            5 => Self::TuNow,
            _ => return None,
        })
    }
}

/// Whether an exchange atom is a whole utterance (`599 <field>`) or the bare
/// field. The `599` is never on the wire; only this bit is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Role {
    /// Render the field alone.
    FieldOnly = 0,
    /// Prefix the field with `599 `.
    Full = 1,
}

impl Role {
    fn from_bit(b: u32) -> Self {
        if b & 1 == 0 {
            Self::FieldOnly
        } else {
            Self::Full
        }
    }
}

/// What a 17-bit exchange number means (`EXCH_NUM`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NumberKind {
    /// Serial number, at least three digits when rendered.
    Serial = 0,
    /// CQ zone 1‥40, at least two digits.
    CqZone = 1,
    /// ITU zone 1‥90, at least two digits.
    ItuZone = 2,
    /// Age.
    Age = 3,
    /// Power.
    Power = 4,
    /// Check, at least two digits.
    Check = 5,
    /// Four-digit first-licence year, 0‥9999.
    FirstLicenseYear = 6,
    /// A number with no assigned meaning.
    Generic = 7,
}

impl NumberKind {
    fn from_index(i: u32) -> Option<Self> {
        Some(match i {
            0 => Self::Serial,
            1 => Self::CqZone,
            2 => Self::ItuZone,
            3 => Self::Age,
            4 => Self::Power,
            5 => Self::Check,
            6 => Self::FirstLicenseYear,
            7 => Self::Generic,
            _ => return None,
        })
    }
}

/// What a two- or three-character location token means (`EXCH_LOC`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LocationKind {
    /// State or province.
    StateProvince = 0,
    /// ARRL / RAC section.
    Section = 1,
    /// Country prefix.
    CountryPrefix = 2,
    /// A location with no assigned meaning.
    Qth = 3,
    /// Local administrative code.
    AdminCode = 4,
}

impl LocationKind {
    fn from_index(i: u32) -> Option<Self> {
        Some(match i {
            0 => Self::StateProvince,
            1 => Self::Section,
            2 => Self::CountryPrefix,
            3 => Self::Qth,
            4 => Self::AdminCode,
            _ => return None,
        })
    }
}

/// One frame's worth of message.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Atom {
    /// A standard callsign with one of the six actions.
    Call {
        /// What is said about the call.
        action: CallAction,
        /// Upper-case standard callsign (see [`is_standard_call`]).
        call: String,
    },
    /// A typed number.
    Number {
        /// Whole utterance or bare field.
        role: Role,
        /// Meaning of the number.
        kind: NumberKind,
        /// The value, at most [`MAX_NUMBER`] (and further limited by `kind`).
        value: u32,
    },
    /// A typed two- or three-character location token (base 36).
    Location {
        /// Whole utterance or bare field.
        role: Role,
        /// Meaning of the token.
        kind: LocationKind,
        /// Upper-case `[0-9A-Z]`, two or three characters; a three-character
        /// token may not have a leading `0`.
        token: String,
    },
    /// `599 <zone> <token>` — CQ zone 1‥40 and a location token.
    ZoneLoc3 {
        /// CQ zone, 1‥40.
        zone: u8,
        /// Location token, as for [`Atom::Location`].
        token: String,
    },
    /// Field Day class and section, `<count><class> <section>`.
    ClassSection {
        /// Transmitter count, 1‥32.
        count: u8,
        /// Class `'A'`‥`'F'`.
        class: char,
        /// 1-based index into the 86 ARRL/RAC sections (see [`section_index`]).
        section: u8,
    },
    /// Serial number and time of day, `599 <serial> <HHMM>`.
    NumberTime {
        /// Whole utterance or bare fields.
        role: Role,
        /// Serial, 0‥16383.
        serial: u16,
        /// Minute of the UTC day, 0‥1439.
        minute: u16,
    },
    /// One of the 18 registered control phrases (index into
    /// [`CONTROL_PHRASES`]).
    Control(u8),
    /// A four-character Maidenhead locator `AA00`‥`RR99`.
    Grid4 {
        /// Whole utterance or bare field.
        role: Role,
        /// The locator, upper case.
        grid: String,
    },
    /// Up to five characters of free text from [`ALPHABET`]; a decoded atom
    /// always holds exactly five (the sender pads with spaces).
    Text5(String),
}

impl Atom {
    /// A call atom; the callsign is upper-cased.
    pub fn call(action: CallAction, call: &str) -> Self {
        Self::Call {
            action,
            call: call.to_ascii_uppercase(),
        }
    }

    /// A location atom; the token is upper-cased.
    pub fn location(role: Role, kind: LocationKind, token: &str) -> Self {
        Self::Location {
            role,
            kind,
            token: token.to_ascii_uppercase(),
        }
    }

    /// A Field Day class/section atom from the section's name (`"EMA"`), or
    /// `None` when the name is not one of the 86.
    pub fn class_section(count: u8, class: char, section: &str) -> Option<Self> {
        Some(Self::ClassSection {
            count,
            class: class.to_ascii_uppercase(),
            section: section_index(section)?,
        })
    }

    /// A grid atom; the locator is upper-cased.
    pub fn grid4(role: Role, grid: &str) -> Self {
        Self::Grid4 {
            role,
            grid: grid.to_ascii_uppercase(),
        }
    }

    /// A TEXT5 atom (not upper-cased: the alphabet is upper case only).
    pub fn text5(text: &str) -> Self {
        Self::Text5(text.into())
    }

    /// The 32-bit grammar word, or `None` if the atom is not encodable (an
    /// unstandard call, an out-of-range field, a bad token, a character outside
    /// the alphabet …).
    pub fn word(&self) -> Option<u32> {
        match self {
            Self::Call { action, call } => {
                let n28 = call28(call)?;
                let a = *action as u32;
                let (i2, n2) = if a <= 3 { (0, a) } else { (1, a - 4) };
                Some((n28 << 4) | (n2 << 2) | i2)
            }
            Self::Number { role, kind, value } => valid_number(*kind, *value)
                .then(|| struct30(0, (*role as u32) << 26 | (*kind as u32) << 22 | value << 5)),
            Self::Location { role, kind, token } => {
                let (n, v) = pack_base36(token)?;
                Some(struct30(
                    1,
                    (*role as u32) << 26 | (*kind as u32) << 22 | (n - 2) << 21 | v << 5,
                ))
            }
            Self::ZoneLoc3 { zone, token } => {
                if !(1..=40).contains(zone) {
                    return None;
                }
                let (n, v) = pack_base36(token)?;
                let pair = u32::from(*zone) << 17 | (n - 2) << 16 | v;
                Some(struct30(2, pair << 1))
            }
            Self::ClassSection {
                count,
                class,
                section,
            } => {
                let class =
                    u32::from(u8::try_from(*class as u32).ok()?).checked_sub(u32::from(b'A'))?;
                if !(1..=32).contains(count) || class > 5 || !(1..=86).contains(section) {
                    return None;
                }
                let pair = u32::from(*count) << 17 | class << 14 | u32::from(*section) << 7;
                Some(struct30(2, 1 << 24 | pair << 1))
            }
            Self::NumberTime {
                role,
                serial,
                minute,
            } => (*serial <= 16_383 && *minute <= 1439).then(|| {
                struct30(
                    3,
                    (*role as u32) << 26 | u32::from(*serial) << 12 | u32::from(*minute) << 1,
                )
            }),
            Self::Control(id) => (usize::from(*id) < CONTROL_PHRASES.len())
                .then(|| struct30(4, u32::from(*id) << 16)),
            Self::Grid4 { role, grid } => {
                let idx = grid4_to_index(grid)?;
                Some(struct30(4, 1 << 23 | (*role as u32) << 22 | idx << 7))
            }
            Self::Text5(text) => {
                if text.chars().count() > 5 {
                    return None;
                }
                let top30 = text
                    .bytes()
                    .chain(core::iter::repeat(b' '))
                    .take(5)
                    .try_fold(0u32, |acc, c| Some(acc << 6 | alphabet_index(c)?))?;
                Some(top30 << 2 | 3)
            }
        }
    }

    /// The 34-bit payload: the word, the reserved bit (always 0) and `eom`.
    pub fn encode(&self, eom: bool) -> Option<Payload> {
        self.word().map(|w| payload_from_word(w, eom))
    }

    /// The text this atom renders as (`render_jtty_atom`); a structured atom's
    /// implicit trailing separator is *not* included (see [`render_message`]).
    pub fn render(&self) -> String {
        match self {
            Self::Call { action, call } => match action {
                CallAction::Cq => alloc::format!("CQ {call} CQ"),
                CallAction::Call => call.clone(),
                CallAction::TuCq => alloc::format!("TU {call} CQ"),
                CallAction::CallTu => alloc::format!("{call} TU"),
                CallAction::CallAgn => alloc::format!("{call} AGN?"),
                CallAction::TuNow => alloc::format!("TU NOW {call}"),
            },
            Self::Number { role, kind, value } => with_role(*role, &render_number(*kind, *value)),
            Self::Location { role, token, .. } => with_role(*role, token),
            Self::ZoneLoc3 { zone, token } => alloc::format!("599 {zone:02} {token}"),
            Self::ClassSection {
                count,
                class,
                section,
            } => alloc::format!(
                "{count}{class} {}",
                wsjt77::ARRL_SECTIONS
                    .get(usize::from(*section).wrapping_sub(1))
                    .copied()
                    .unwrap_or("?")
            ),
            Self::NumberTime {
                role,
                serial,
                minute,
            } => with_role(
                *role,
                &alloc::format!(
                    "{} {:02}{:02}",
                    render_number(NumberKind::Serial, u32::from(*serial)),
                    minute / 60,
                    minute % 60
                ),
            ),
            Self::Control(id) => CONTROL_PHRASES
                .get(usize::from(*id))
                .map_or_else(String::new, |s| (*s).into()),
            Self::Grid4 { role, grid } => with_role(*role, grid),
            Self::Text5(text) => text.clone(),
        }
    }

    fn is_text5(&self) -> bool {
        matches!(self, Self::Text5(_))
    }
}

fn with_role(role: Role, field: &str) -> String {
    match role {
        Role::Full => alloc::format!("599 {field}"),
        Role::FieldOnly => field.into(),
    }
}

/// Render a message: the atoms' text in order, a structured atom followed by one
/// implicit separating space, TEXT5 characters verbatim, the trailing space and
/// TEXT5 padding dropped, at most [`MAX_MESSAGE_CHARS`] characters
/// (`unpack_jtty`).
pub fn render_message(atoms: &[Atom]) -> String {
    let mut out = atoms.iter().fold(String::new(), |mut acc, a| {
        acc.push_str(&a.render());
        if !a.is_text5() {
            acc.push(' ');
        }
        acc
    });
    out.truncate(
        out.char_indices()
            .nth(MAX_MESSAGE_CHARS)
            .map_or(out.len(), |(i, _)| i),
    );
    out.truncate(out.trim_end().len());
    out
}

/// `body` (27 bits) and `family` (3 bits) as a STRUCT30 word (`i2 = 2`).
fn struct30(family: u32, body: u32) -> u32 {
    debug_assert!(body < 1 << 27);
    ((body << 3) | family) << 2 | 2
}

// ── payload bits ─────────────────────────────────────────────────────────────

/// The 34-bit payload for a grammar word: its 32 bits, MSB first, then the
/// reserved bit (0) and the end-of-message flag.
pub fn payload_from_word(word: u32, eom: bool) -> Payload {
    core::array::from_fn(|i| match i {
        0..=31 => ((word >> (31 - i)) & 1) as u8,
        32 => 0,
        _ => u8::from(eom),
    })
}

/// The grammar word and EOM flag of a payload, or `None` if a bit is not 0/1
/// or the reserved bit is set.
pub fn word_from_payload(p: &Payload) -> Option<(u32, bool)> {
    if p.iter().any(|&b| b > 1) || p[32] != 0 {
        return None;
    }
    let word = p[..32].iter().fold(0u32, |w, &b| (w << 1) | u32::from(b));
    Some((word, p[PAYLOAD_BITS - 1] == 1))
}

/// Decode a payload to its atom and EOM flag, applying every validity rule
/// (module docs). `None` means: discard.
pub fn decode_payload(p: &Payload) -> Option<(Atom, bool)> {
    let (word, eom) = word_from_payload(p)?;
    decode_word(word).map(|a| (a, eom))
}

/// Decode a 32-bit grammar word to an atom, `None` if it is invalid.
pub fn decode_word(word: u32) -> Option<Atom> {
    if word == 0 {
        return None; // the all-zero sentinel
    }
    let field = |v: u32, lo: u32, len: u32| (v >> lo) & ((1u32 << len) - 1);
    match word & 3 {
        i2 @ (0 | 1) => {
            let n2 = field(word, 2, 2);
            if i2 == 1 && n2 > 1 {
                return None;
            }
            let call = wsjt77::unpack28(word >> 4);
            if !is_standard_call(&call) {
                return None;
            }
            let action = CallAction::from_index(if i2 == 0 { n2 } else { n2 + 4 })?;
            Some(Atom::Call { action, call })
        }
        2 => {
            let body = word >> 5;
            match field(word, 2, 3) {
                0 => {
                    let kind = NumberKind::from_index(field(body, 22, 4))?;
                    let value = field(body, 5, 17);
                    (field(body, 0, 5) == 0 && valid_number(kind, value)).then_some(Atom::Number {
                        role: Role::from_bit(field(body, 26, 1)),
                        kind,
                        value,
                    })
                }
                1 => {
                    let kind = LocationKind::from_index(field(body, 22, 4))?;
                    let token = unpack_base36(field(body, 5, 16), field(body, 21, 1) + 2)?;
                    (field(body, 0, 5) == 0).then_some(Atom::Location {
                        role: Role::from_bit(field(body, 26, 1)),
                        kind,
                        token,
                    })
                }
                2 => {
                    let pair = field(body, 1, 23);
                    if field(body, 0, 1) != 0 {
                        return None;
                    }
                    match field(body, 24, 3) {
                        0 => {
                            let zone = field(pair, 17, 6);
                            let token = unpack_base36(field(pair, 0, 16), field(pair, 16, 1) + 2)?;
                            (1..=40).contains(&zone).then_some(Atom::ZoneLoc3 {
                                zone: zone as u8,
                                token,
                            })
                        }
                        1 => {
                            let (count, class, section) =
                                (field(pair, 17, 6), field(pair, 14, 3), field(pair, 7, 7));
                            ((1..=32).contains(&count)
                                && class <= 5
                                && (1..=86).contains(&section)
                                && field(pair, 0, 7) == 0)
                                .then(|| Atom::ClassSection {
                                    count: count as u8,
                                    class: char::from(b'A' + class as u8),
                                    section: section as u8,
                                })
                        }
                        _ => None,
                    }
                }
                3 => {
                    let (serial, minute) = (field(body, 12, 14), field(body, 1, 11));
                    (field(body, 0, 1) == 0 && minute <= 1439).then_some(Atom::NumberTime {
                        role: Role::from_bit(field(body, 26, 1)),
                        serial: serial as u16,
                        minute: minute as u16,
                    })
                }
                4 => {
                    let data = field(body, 0, 23);
                    match field(body, 23, 4) {
                        0 => {
                            let id = field(data, 16, 7);
                            (field(data, 0, 16) == 0 && (id as usize) < CONTROL_PHRASES.len())
                                .then_some(Atom::Control(id as u8))
                        }
                        1 => {
                            let idx = field(data, 7, 15);
                            (field(data, 0, 7) == 0 && idx < GRID4_COUNT).then(|| Atom::Grid4 {
                                role: Role::from_bit(field(data, 22, 1)),
                                grid: index_to_grid4(idx),
                            })
                        }
                        _ => None,
                    }
                }
                _ => None, // families 5, 6 (reserved) and 7 (guard)
            }
        }
        _ => {
            let top30 = word >> 2;
            Some(Atom::Text5(
                (0..5)
                    .map(|i| char::from(ALPHABET[((top30 >> (6 * (4 - i))) & 63) as usize]))
                    .collect(),
            ))
        }
    }
}

// ── calls ────────────────────────────────────────────────────────────────────

/// `true` when `call` can ride in a call word: upstream's `standard_call`.
///
/// A call must satisfy `chkcall` (a letter in one of the first two positions, a
/// digit in the second or third, then one to three letters), contain only
/// `A–Z0–9` (no `/`), not start with `Q`, and survive `pack28` → `unpack28`
/// unchanged. That last step is what keeps `CQ`, `DE`, hashed calls and every
/// other non-callsign token out.
pub fn is_standard_call(call: &str) -> bool {
    call28(call).is_some()
}

/// The 28-bit call field for a standard call, `None` otherwise.
fn call28(call: &str) -> Option<u32> {
    let w = call.as_bytes();
    if !chkcall(w)
        || !w
            .iter()
            .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit())
    {
        return None;
    }
    let n28 = wsjt77::pack28(call)?;
    // Only standard-call tokens; the specials and hashes sit below this.
    if n28 < wsjt77::NTOKENS + wsjt77::MAX22 {
        return None;
    }
    (wsjt77::unpack28(n28) == call).then_some(n28)
}

/// `chkcall.f90` for a call without `/`: 3–6 characters …
fn chkcall(w: &[u8]) -> bool {
    let n = w.len();
    if !(3..=6).contains(&n) || w[0] == b'Q' {
        return false;
    }
    let at = |i: usize| w.get(i).copied().unwrap_or(b' ');
    if !(at(0).is_ascii_uppercase() || at(1).is_ascii_uppercase()) {
        return false;
    }
    // 1-based position of the digit: the third if there is one, else the second.
    let i1 = if at(2).is_ascii_digit() {
        3
    } else if at(1).is_ascii_digit() {
        2
    } else {
        return false;
    };
    let suffix = &w[i1..];
    (1..=3).contains(&suffix.len()) && suffix.iter().all(u8::is_ascii_uppercase)
}

// ── numbers, tokens, grids, sections ────────────────────────────────────────

fn valid_number(kind: NumberKind, value: u32) -> bool {
    value <= MAX_NUMBER
        && match kind {
            NumberKind::CqZone => (1..=40).contains(&value),
            NumberKind::ItuZone => (1..=90).contains(&value),
            NumberKind::FirstLicenseYear => value <= 9999,
            _ => true,
        }
}

fn render_number(kind: NumberKind, value: u32) -> String {
    let mut s = String::new();
    let _ = match kind {
        NumberKind::Serial if value < 1000 => write!(s, "{value:03}"),
        NumberKind::CqZone | NumberKind::ItuZone | NumberKind::Check if value < 100 => {
            write!(s, "{value:02}")
        }
        NumberKind::FirstLicenseYear => write!(s, "{value:04}"),
        _ => write!(s, "{value}"),
    };
    s
}

/// The index of a character in [`ALPHABET`].
fn alphabet_index(c: u8) -> Option<u32> {
    ALPHABET.iter().position(|&a| a == c).map(|i| i as u32)
}

/// A two- or three-character base-36 token as `(length, value)`; a
/// three-character token whose value is below 36² (a leading `0`) is not
/// canonical and is refused.
fn pack_base36(token: &str) -> Option<(u32, u32)> {
    let n = token.len();
    if !(2..=3).contains(&n) {
        return None;
    }
    let value = token.bytes().try_fold(0u32, |acc, c| {
        BASE36
            .iter()
            .position(|&b| b == c)
            .map(|d| acc * 36 + d as u32)
    })?;
    (n == 2 || value >= 36 * 36).then_some((n as u32, value))
}

/// The inverse of [`pack_base36`] for a wire value, `None` if the value does not
/// fit `n` characters or is not canonical.
fn unpack_base36(value: u32, n: u32) -> Option<String> {
    if value >= 36u32.pow(n) || (n == 3 && value < 36 * 36) {
        return None;
    }
    Some(
        (0..n)
            .rev()
            .map(|i| char::from(BASE36[((value / 36u32.pow(i)) % 36) as usize]))
            .collect(),
    )
}

/// `((f₁·18 + f₂)·10 + d₁)·10 + d₂` for a locator `AA00`‥`RR99`.
///
/// This is JTTY's own grid index, **not** the pack77 `g15` mapping of
/// [`wsjt77::pack_grid4`].
pub fn grid4_to_index(grid: &str) -> Option<u32> {
    let g = grid.as_bytes();
    if g.len() != 4 {
        return None;
    }
    let field = |c: u8| c.checked_sub(b'A').filter(|&v| v < 18).map(u32::from);
    let digit = |c: u8| c.checked_sub(b'0').filter(|&v| v < 10).map(u32::from);
    Some(((field(g[0])? * 18 + field(g[1])?) * 10 + digit(g[2])?) * 10 + digit(g[3])?)
}

fn index_to_grid4(index: u32) -> String {
    let (d2, r) = (index % 10, index / 10);
    let (d1, r) = (r % 10, r / 10);
    let (f2, f1) = (r % 18, r / 18);
    [
        char::from(b'A' + f1 as u8),
        char::from(b'A' + f2 as u8),
        char::from(b'0' + d1 as u8),
        char::from(b'0' + d2 as u8),
    ]
    .iter()
    .collect()
}

/// The 1-based index of an ARRL/RAC section name (`"EMA"` → 11), `None` if it is
/// not one of the 86.
pub fn section_index(name: &str) -> Option<u8> {
    wsjt77::ARRL_SECTIONS
        .iter()
        .position(|&s| s == name)
        .map(|i| (i + 1) as u8)
}

/// All 86 section names in index order (`PACK77_ARRL_SECTIONS`).
pub fn sections() -> Vec<&'static str> {
    wsjt77::ARRL_SECTIONS.to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 34-bit vector from the spec as an integer, bit 1 most significant.
    fn bits34(v: u64) -> Payload {
        core::array::from_fn(|i| ((v >> (33 - i)) & 1) as u8)
    }

    fn atom_serial(v: u32) -> Atom {
        Atom::Number {
            role: Role::Full,
            kind: NumberKind::Serial,
            value: v,
        }
    }

    /// `jtty_source_encoding.txt`, "Representative 34-bit vectors".
    fn spec_vectors() -> Vec<(&'static str, Atom, bool, u64, &'static str)> {
        vec![
            (
                "CQ K1ABC CQ",
                Atom::call(CallAction::Cq, "K1ABC"),
                true,
                0x026F78D41,
                "CQ K1ABC CQ",
            ),
            (
                "SERIAL 123, more",
                atom_serial(123),
                false,
                0x20007B008,
                "599 123",
            ),
            (
                "SERIAL 123, EOM",
                atom_serial(123),
                true,
                0x20007B009,
                "599 123",
            ),
            (
                "STATE CA",
                Atom::location(Role::Full, LocationKind::StateProvince, "CA"),
                true,
                0x2001BA019,
                "599 CA",
            ),
            (
                "ZONE_LOC3 05 NWT",
                Atom::ZoneLoc3 {
                    zone: 5,
                    token: "NWT".into(),
                },
                true,
                0x00B790D29,
                "599 05 NWT",
            ),
            (
                "CLASS_SECTION 1D EMA",
                Atom::class_section(1, 'D', "EMA").unwrap(),
                true,
                0x082C58029,
                "1D EMA",
            ),
            (
                "NUM_TIME 156 1749",
                Atom::NumberTime {
                    role: Role::Full,
                    serial: 156,
                    minute: 17 * 60 + 49,
                },
                true,
                0x204E42D39,
                "599 156 1749",
            ),
            ("CONTROL AGN?", Atom::Control(0), true, 0x000000049, "AGN?"),
            (
                "GRID4 FN42",
                Atom::grid4(Role::FieldOnly, "FN42"),
                true,
                0x04A198049,
                "FN42",
            ),
            (
                "TEXT5 HELLO",
                Atom::text5("HELLO"),
                true,
                0x11395558D,
                "HELLO",
            ),
        ]
    }

    #[test]
    fn spec_vectors_encode_exactly() {
        for (name, atom, eom, hex, _) in spec_vectors() {
            assert_eq!(atom.encode(eom), Some(bits34(hex)), "{name}");
        }
    }

    #[test]
    fn spec_vectors_decode_and_render() {
        for (name, atom, eom, hex, text) in spec_vectors() {
            let (got, got_eom) = decode_payload(&bits34(hex)).unwrap_or_else(|| panic!("{name}"));
            assert_eq!(got, atom, "{name}");
            assert_eq!(got_eom, eom, "{name}");
            assert_eq!(got.render(), text, "{name}");
        }
    }

    #[test]
    fn alphabet_is_64_distinct_characters() {
        let mut s: Vec<u8> = ALPHABET.to_vec();
        s.sort_unstable();
        s.dedup();
        assert_eq!(s.len(), 64);
        assert_eq!(ALPHABET[36], b' ');
    }

    #[test]
    fn every_call_action_round_trips() {
        for (a, text) in [
            (CallAction::Cq, "CQ K1ABC CQ"),
            (CallAction::Call, "K1ABC"),
            (CallAction::TuCq, "TU K1ABC CQ"),
            (CallAction::CallTu, "K1ABC TU"),
            (CallAction::CallAgn, "K1ABC AGN?"),
            (CallAction::TuNow, "TU NOW K1ABC"),
        ] {
            let atom = Atom::call(a, "K1ABC");
            let p = atom.encode(false).unwrap();
            assert_eq!(decode_payload(&p), Some((atom.clone(), false)));
            assert_eq!(atom.render(), text);
        }
    }

    #[test]
    fn call_field_split_is_i2_and_n2() {
        // i2 = 1 carries actions 4 and 5 with n2 = 0, 1.
        let agn = Atom::call(CallAction::CallAgn, "W9XYZ").word().unwrap();
        let now = Atom::call(CallAction::TuNow, "W9XYZ").word().unwrap();
        assert_eq!((agn & 3, (agn >> 2) & 3), (1, 0));
        assert_eq!((now & 3, (now >> 2) & 3), (1, 1));
        assert_eq!(agn >> 4, now >> 4);
    }

    #[test]
    fn standard_calls() {
        for good in [
            "K1ABC", "W9XYZ", "JA6DEF", "3Y0Z", "VK3NV", "K1A", "W7UVW", "9A1AA",
        ] {
            assert!(is_standard_call(good), "{good}");
        }
        for bad in [
            "CQ", "DE", "QRZ", "K1ABC/P", "QU1RK", "Q1ABC", "K1ABCD", "K1", "ABC", "", "1234",
            "k1abc", "K1AB1", "<...>", "K1ABC ", "CQ 123",
        ] {
            assert!(!is_standard_call(bad), "{bad:?}");
        }
    }

    #[test]
    fn text5_padding_and_alphabet() {
        // Short text pads with spaces on the wire and decodes to five characters.
        let p = Atom::text5("/P").encode(true).unwrap();
        assert_eq!(
            decode_payload(&p).map(|(a, e)| (a.render(), e)),
            Some(("/P   ".into(), true))
        );
        assert!(Atom::text5("HELLO!").word().is_none(), "six characters");
        assert!(
            Atom::text5("hello").word().is_none(),
            "lower case is outside the alphabet"
        );
        assert!(Atom::text5("A~B").word().is_none());
        // every alphabet character round-trips
        for c in ALPHABET.iter() {
            let a = Atom::text5(&alloc::format!("{}", char::from(*c)));
            let (got, _) = decode_payload(&a.encode(false).unwrap()).unwrap();
            assert_eq!(got.render().as_bytes()[0], *c);
        }
    }

    #[test]
    fn number_kinds_render_canonically() {
        let n = |kind, value| {
            Atom::Number {
                role: Role::FieldOnly,
                kind,
                value,
            }
            .render()
        };
        assert_eq!(n(NumberKind::Serial, 5), "005");
        assert_eq!(n(NumberKind::Serial, 1234), "1234");
        assert_eq!(n(NumberKind::CqZone, 5), "05");
        assert_eq!(n(NumberKind::ItuZone, 90), "90");
        assert_eq!(n(NumberKind::Check, 7), "07");
        assert_eq!(n(NumberKind::Age, 7), "7");
        assert_eq!(n(NumberKind::Power, 100), "100");
        assert_eq!(n(NumberKind::FirstLicenseYear, 1999), "1999");
        assert_eq!(n(NumberKind::FirstLicenseYear, 5), "0005");
        assert_eq!(n(NumberKind::Generic, 0), "0");
    }

    #[test]
    fn number_ranges_are_enforced_both_ways() {
        let enc = |kind, value| {
            Atom::Number {
                role: Role::Full,
                kind,
                value,
            }
            .word()
        };
        assert!(enc(NumberKind::Serial, MAX_NUMBER).is_some());
        assert!(enc(NumberKind::Serial, MAX_NUMBER + 1).is_none());
        assert!(enc(NumberKind::CqZone, 0).is_none());
        assert!(enc(NumberKind::CqZone, 41).is_none());
        assert!(enc(NumberKind::CqZone, 40).is_some());
        assert!(enc(NumberKind::ItuZone, 91).is_none());
        assert!(enc(NumberKind::FirstLicenseYear, 10_000).is_none());
        // and on the way in: a valid word with an out-of-range value is refused
        let bad = struct30(0, 1 << 22 | 41 << 5); // CQ zone 41
        assert_eq!(decode_word(bad), None);
        let bad = struct30(0, 8 << 22); // number kind 8
        assert_eq!(decode_word(bad), None);
    }

    #[test]
    fn location_tokens_are_canonical_base36() {
        let enc = |t: &str| Atom::location(Role::Full, LocationKind::Qth, t).word();
        assert!(enc("MA").is_some());
        assert!(enc("0A").is_some(), "two characters may start with 0");
        assert!(enc("ABC").is_some());
        assert!(
            enc("0AB").is_none(),
            "three characters may not start with 0"
        );
        assert!(enc("A").is_none());
        assert!(enc("ABCD").is_none());
        // the constructor upper-cases; a lower-case token built by hand is refused
        let lower = Atom::Location {
            role: Role::Full,
            kind: LocationKind::Qth,
            token: "a1".into(),
        };
        assert!(lower.word().is_none());
        assert!(enc("A-").is_none());
        for t in ["ZZ", "00", "ZZZ", "100", "A0B"] {
            let a = Atom::location(Role::FieldOnly, LocationKind::AdminCode, t);
            assert_eq!(decode_word(a.word().unwrap()), Some(a), "{t}");
        }
        // length bit says 3 but the value fits two characters
        let bad = struct30(1, 1 << 21 | 100 << 5);
        assert_eq!(decode_word(bad), None);
        // value too large for two characters
        let bad = struct30(1, (36 * 36) << 5);
        assert_eq!(decode_word(bad), None);
    }

    #[test]
    fn grid4_index_and_domain() {
        assert_eq!(grid4_to_index("AA00"), Some(0));
        assert_eq!(grid4_to_index("RR99"), Some(GRID4_COUNT - 1));
        assert_eq!(grid4_to_index("SA00"), None);
        assert_eq!(grid4_to_index("AA0"), None);
        assert_eq!(grid4_to_index("aa00"), None);
        assert_eq!(index_to_grid4(grid4_to_index("FN42").unwrap()), "FN42");
        for g in ["AA00", "JN58", "PM95", "RR99"] {
            let a = Atom::grid4(Role::Full, g);
            assert_eq!(decode_word(a.word().unwrap()), Some(a), "{g}");
        }
        // 32400 is the first invalid index
        let bad = struct30(4, 1 << 23 | GRID4_COUNT << 7);
        assert_eq!(decode_word(bad), None);
        let ok = struct30(4, 1 << 23 | (GRID4_COUNT - 1) << 7);
        assert!(decode_word(ok).is_some());
    }

    #[test]
    fn sections_are_86_and_indexed_from_one() {
        assert_eq!(sections().len(), 86);
        assert_eq!(section_index("AB"), Some(1));
        assert_eq!(section_index("EMA"), Some(11));
        assert_eq!(section_index("NB"), Some(86));
        assert_eq!(section_index("ZZZ"), None);
        assert!(Atom::class_section(1, 'D', "ZZZ").is_none());
    }

    #[test]
    fn class_section_ranges() {
        let enc = |count, class, section| {
            Atom::ClassSection {
                count,
                class,
                section,
            }
            .word()
        };
        assert!(enc(1, 'A', 1).is_some());
        assert!(enc(32, 'F', 86).is_some());
        assert!(enc(0, 'A', 1).is_none());
        assert!(enc(33, 'A', 1).is_none());
        assert!(enc(1, 'G', 1).is_none());
        assert!(enc(1, '@', 1).is_none());
        assert!(enc(1, 'A', 0).is_none());
        assert!(enc(1, 'A', 87).is_none());
        // class 6 and 7, section 0 and 87 on the wire
        let pair = |count: u32, class: u32, section: u32| {
            struct30(2, 1 << 24 | (count << 17 | class << 14 | section << 7) << 1)
        };
        assert!(decode_word(pair(1, 5, 86)).is_some());
        assert_eq!(decode_word(pair(1, 6, 1)), None);
        assert_eq!(decode_word(pair(1, 7, 1)), None);
        assert_eq!(decode_word(pair(1, 0, 0)), None);
        assert_eq!(decode_word(pair(1, 0, 87)), None);
        assert_eq!(decode_word(pair(0, 0, 1)), None);
        assert_eq!(decode_word(pair(33, 0, 1)), None);
        assert_eq!(
            decode_word(pair(1, 0, 1) | 1 << 5),
            None,
            "zero bit 0 of the body"
        );
    }

    #[test]
    fn number_time_ranges_and_render() {
        let a = Atom::NumberTime {
            role: Role::FieldOnly,
            serial: 5,
            minute: 9 * 60 + 5,
        };
        assert_eq!(a.render(), "005 0905");
        assert!(
            Atom::NumberTime {
                role: Role::Full,
                serial: 16_384,
                minute: 0
            }
            .word()
            .is_none()
        );
        assert!(
            Atom::NumberTime {
                role: Role::Full,
                serial: 0,
                minute: 1440
            }
            .word()
            .is_none()
        );
        assert_eq!(decode_word(struct30(3, 1440 << 1)), None);
        assert_eq!(decode_word(struct30(3, 1439 << 1 | 1)), None, "zero bit");
        assert!(decode_word(struct30(3, 1439 << 1)).is_some());
    }

    #[test]
    fn control_phrases() {
        assert_eq!(CONTROL_PHRASES.len(), 18);
        for id in 0..18u8 {
            let a = Atom::Control(id);
            assert_eq!(decode_word(a.word().unwrap()), Some(a.clone()));
            assert_eq!(a.render(), CONTROL_PHRASES[usize::from(id)]);
        }
        assert!(Atom::Control(18).word().is_none());
        assert_eq!(decode_word(struct30(4, 18 << 16)), None);
        assert_eq!(
            decode_word(struct30(4, 1 << 16 | 1)),
            None,
            "reserved 16 bits"
        );
    }

    #[test]
    fn invalid_words_are_discarded() {
        // the all-zero sentinel
        assert_eq!(decode_word(0), None);
        // call word with i2 = 1, n2 = 2 or 3 (reserved)
        let call = Atom::call(CallAction::Call, "K1ABC").word().unwrap() >> 4 << 4;
        assert_eq!(decode_word(call | 2 << 2 | 1), None);
        assert_eq!(decode_word(call | 3 << 2 | 1), None);
        // call fields that are not standard calls: CQ, DE, a CQ-suffix token, a hash
        for n28 in [0u32, 1, 2, 3, 1003, 2_063_592 + 100, 2_063_592] {
            assert_eq!(decode_word(n28 << 4 | 1 << 2), None, "n28 {n28}");
        }
        // STRUCT30 families 5, 6 (reserved) and 7 (guard)
        for family in [5, 6, 7] {
            assert_eq!(
                decode_word(struct30(family, 0x123)),
                None,
                "family {family}"
            );
        }
        // EXCH_NUM with a non-zero trailing field
        assert_eq!(decode_word(struct30(0, 7 << 22 | 5 << 5 | 1)), None);
        // EXCH_LOC kinds 5‥15
        for kind in 5..16 {
            assert_eq!(decode_word(struct30(1, kind << 22 | 100 << 5)), None);
        }
        // EXCH_PAIR schemas 2‥7
        for schema in 2..8u32 {
            assert_eq!(decode_word(struct30(2, schema << 24)), None);
        }
        // MISC subtypes 2‥15
        for sub in 2..16u32 {
            assert_eq!(decode_word(struct30(4, sub << 23)), None);
        }
        // ZONE_LOC3 zones 0 and 41‥63
        let zl = |zone: u32| struct30(2, (zone << 17 | 100) << 1);
        assert_eq!(decode_word(zl(0)), None);
        assert_eq!(decode_word(zl(41)), None);
        assert_eq!(decode_word(zl(63)), None);
        assert!(decode_word(zl(1)).is_some());
        assert!(decode_word(zl(40)).is_some());
    }

    #[test]
    fn reserved_bit_and_non_binary_payloads_are_discarded() {
        let mut p = Atom::call(CallAction::Cq, "K1ABC").encode(true).unwrap();
        assert!(decode_payload(&p).is_some());
        p[32] = 1;
        assert!(decode_payload(&p).is_none());
        p[32] = 0;
        p[5] = 2;
        assert!(decode_payload(&p).is_none());
    }

    #[test]
    fn eom_is_only_the_last_bit() {
        let a = Atom::text5("ABCDE");
        let (w, more) = (a.encode(false).unwrap(), a.encode(true).unwrap());
        assert_eq!(&w[..33], &more[..33]);
        assert_eq!((w[33], more[33]), (0, 1));
    }

    #[test]
    fn every_encodable_word_bit_pattern_survives_the_round_trip() {
        // Encode a spread of atoms, decode, re-encode: the word is unchanged.
        let atoms = [
            Atom::call(CallAction::Cq, "JA6DEF"),
            atom_serial(0),
            atom_serial(1),
            atom_serial(MAX_NUMBER),
            Atom::location(Role::FieldOnly, LocationKind::Section, "EMA"),
            Atom::NumberTime {
                role: Role::FieldOnly,
                serial: 16_383,
                minute: 1439,
            },
            Atom::grid4(Role::Full, "JN58"),
            Atom::text5("     "),
            Atom::text5("~"), // not in the alphabet
        ];
        for a in atoms {
            match a.word() {
                Some(w) => {
                    let back = decode_word(w).unwrap_or_else(|| panic!("{a:?}"));
                    assert_eq!(back.word(), Some(w), "{a:?}");
                }
                None => assert_eq!(a, Atom::text5("~")),
            }
        }
    }

    #[test]
    fn message_rendering() {
        let cq = Atom::call(CallAction::Cq, "K1ABC");
        assert_eq!(render_message(std::slice::from_ref(&cq)), "CQ K1ABC CQ");
        // structured atoms are separated by one space
        assert_eq!(
            render_message(&[Atom::call(CallAction::Call, "WB9XYZ"), atom_serial(123)]),
            "WB9XYZ 599 123"
        );
        // TEXT5 is verbatim; the final padding is dropped
        assert_eq!(
            render_message(&[
                Atom::text5("HELLO"),
                Atom::text5(" WORL"),
                Atom::text5("D 73")
            ]),
            "HELLO WORLD 73"
        );
        // a structured atom after TEXT5 adds no separator of its own before it
        assert_eq!(
            render_message(&[Atom::text5("HI "), Atom::call(CallAction::Call, "K1ABC")]),
            "HI K1ABC"
        );
        // 80 characters at most
        let long: Vec<Atom> = (0..20).map(|_| Atom::text5("ABCDE")).collect();
        assert_eq!(render_message(&long).len(), MAX_MESSAGE_CHARS);
    }
}
