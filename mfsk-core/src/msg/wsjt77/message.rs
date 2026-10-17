// SPDX-License-Identifier: GPL-3.0-only
//! The whole-message 77-bit packer: text in, `i3.n3` and 77 bits out.
//!
//! A port of WSJT-X v3.2.0-rc1 `pack77_encode_core` and `pack77_encode_result`
//! (`lib/77bit/packjt77.f90:527-627`) with the status codes of `:66-71`, the
//! candidates they try in order (`pack77_01_candidate` `:2191`,
//! `pack77_03_candidate` `:2218`, `pack77_05` `:687`, `pack77_06_candidate`
//! `:2259`, `pack77_1_candidate` `:2305`, `pack77_3_candidate` `:2342`,
//! `pack77_4_candidate` `:2377`, `pack77_5_candidate` `:2406`,
//! `pack77_free_text_exact` `:719`), the source grammar they parse with
//! (`lib/77bit/packjt77_grammar.f90`) and the schemas they encode with
//! (`lib/77bit/packjt77_schema.f90`).
//!
//! Every candidate is accepted only if its bits unpack to the text it was given
//! (`take_candidate`, `packjt77.f90:200`, with `pack77_gate_message_checks`,
//! `packjt77_grammar.f90:641`), so a message this accepts is one the receiver
//! reads back as sent. The 0.6 candidate (the WSPR forms) is the existing
//! [`pack77_wspr`](super::pack77_wspr); the free-text fallback is the existing
//! [`pack77_free_text`](super::pack77_free_text).

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use super::{
    ARRL_SECTIONS, C1, C2, C3, C4, C38, MAX_GRID4, MAX22, NTOKENS, RTTY_STATES, c28_standard_shape,
    is_grid4, pack77_free_text, pack77_wspr, put_bits, read_bits, type12_standard_call, unpack77,
    valid_hash_call_token,
};

/// What [`pack77_message`] may be asked for beyond the text.
///
/// `packjt77.f90:152-157` `pack77_options`: only `prefer_wspr_50bit` is here.
/// `record_tx_hashes` is [`Packed77::hashed_calls`], for the caller to record;
/// `expand_dx_macro` and `dx_macro_base` (the `$DX` / `$DXCALL` macro, a GUI
/// convenience) are **not ported**.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Default)]
pub struct Pack77Options {
    /// Ask for the 50-bit WSPR payload (`i3=0, n3=6`) of a message of the WSPR
    /// forms, and refuse to fall through to another family or to free text when
    /// the message has such a form but is not a valid WSPR message
    /// ([`Pack77Error::PreferredFamilyRejected`]). FST4W wants this
    /// (`genfst4.f90:49-51`); with it unset `<CALL> GRID6` is a Type-4-or-free-text
    /// message, not a WSPR one (`packjt77.f90:2288`).
    pub prefer_wspr_50bit: bool,
}

impl Pack77Options {
    /// The default options: nothing preferred.
    pub const fn new() -> Self {
        Self {
            prefer_wspr_50bit: false,
        }
    }

    /// Set [`prefer_wspr_50bit`](Self::prefer_wspr_50bit).
    pub const fn prefer_wspr_50bit(mut self, on: bool) -> Self {
        self.prefer_wspr_50bit = on;
        self
    }
}

/// A packed message.
#[non_exhaustive]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packed77 {
    /// The 77 bits, one per byte (0 or 1), most significant first: the layout
    /// [`unpack77`] reads.
    pub bits: [u8; 77],
    /// Message type `i3`.
    pub i3: u8,
    /// Message subtype `n3` (0 unless `i3 == 0`).
    pub n3: u8,
    /// The calls this encode makes resolvable by a later hash-only decode: the
    /// calls in the call fields (standard, non-standard and `<CALL>`), and the
    /// call behind each hash of types 0.1, 0.6 (50-bit form), 4 and 5, without
    /// brackets, in the order the packer staged them, each once. Upstream's
    /// `pack77_hash_facts` (`packjt77.f90:81-86`), which `pack77` commits to the
    /// shared hash table unless `record_tx_hashes` is off
    /// (`commit_hash_facts`, `:448`). Empty for free text and telemetry.
    pub hashed_calls: Vec<String>,
}

/// Why [`pack77_message`] did not pack the text. The variants are
/// `PACK77_STATUS_*` of `packjt77.f90:66-71`.
///
/// Of the five, a message that reaches the free-text fallback can only end in
/// [`FreeTextTooLong`](Self::FreeTextTooLong),
/// [`FreeTextInvalid`](Self::FreeTextInvalid) or, with
/// [`Pack77Options::prefer_wspr_50bit`], [`PreferredFamilyRejected`](Self::PreferredFamilyRejected):
/// `pack77_encode_core` overwrites the result of every earlier step with that of
/// the free-text step (`:606`), so upstream never reports `NOT_ENCODED` or
/// `INTERNAL_ROUNDTRIP_REJECTED` from `pack77_encode_result`. They are here
/// because the codes are.
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pack77Error {
    /// `PACK77_STATUS_NOT_ENCODED`: no family took the text.
    NotEncoded,
    /// `PACK77_STATUS_FREE_TEXT_TOO_LONG`: more than 13 characters, and no
    /// structured family took it.
    FreeTextTooLong,
    /// `PACK77_STATUS_FREE_TEXT_INVALID`: a character outside the free-text
    /// alphabet (`0-9 A-Z + - . / ?` and space), and no structured family took it.
    FreeTextInvalid,
    /// `PACK77_STATUS_PREFERRED_FAMILY_REJECTED`: [`Pack77Options::prefer_wspr_50bit`]
    /// was set and the text has the shape of a WSPR message (two words, the
    /// first 5 to 12 characters, the second at most 6) but is not one.
    PreferredFamilyRejected,
    /// `PACK77_STATUS_INTERNAL_ROUNDTRIP_REJECTED`: a candidate's bits did not
    /// unpack to the text.
    RoundtripRejected,
}

impl core::fmt::Display for Pack77Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::NotEncoded => "not encoded",
            Self::FreeTextTooLong => "free text too long",
            Self::FreeTextInvalid => "invalid free text",
            Self::PreferredFamilyRejected => "preferred message family rejected",
            Self::RoundtripRejected => "internal round-trip check failed",
        })
    }
}

impl core::error::Error for Pack77Error {}

/// Pack a message the way WSJT-X v3.2.0-rc1 `pack77` does.
///
/// The text is upper-cased and split into words (at most 19, each at most 13
/// characters; beyond that only telemetry or free text can take it). Unless it
/// starts with `CQ `, `DE ` or `QRZ `, the candidates are tried in upstream's
/// order: 0.1 DXpedition (`CALL RR73; CALL <CALL> -08`), 0.3 / 0.4 ARRL Field Day
/// (`CALL CALL [R] 16A EMA`), and for one word 0.5 telemetry (up to 18 hex
/// digits); then 0.6 WSPR (`CALL GRID4 DBM`, `PFX/CALL DBM`, `CALL/SFX DBM`, and
/// with [`Pack77Options::prefer_wspr_50bit`] `<CALL> GRID6`), 1 and 2 standard
/// (with `/R`, `/P`, `<CALL>`, `CQ DX`, `CQ 123`), 3 ARRL RTTY Roundup, 4 one
/// non-standard call (`<CALL> PJ4/K1ABC RRR`, `CQ PJ4/K1ABC`) and 5 EU VHF
/// (`<CALL> <CALL> R 590003 IO91NP`); else exact free text (13 characters).
/// Each is taken only if its bits unpack to the text (`take_candidate`), so
/// `pack77_message` returns what a receiver will read back, or an error.
///
/// Only the first 37 bytes of `text` count, as in upstream's `character*37`.
/// Not ported: the `$DX` macro expansion (`expand_dx_macro`).
///
/// Ported from `lib/77bit/packjt77.f90:527-627` (`pack77_encode_core`,
/// `pack77_encode_result`).
///
/// # Examples
/// ```
/// # use mfsk_core::msg::wsjt77::{pack77_message, unpack77, Pack77Options};
/// let p = pack77_message("CQ POTA K1ABC FN42", &Pack77Options::new()).unwrap();
/// assert_eq!((p.i3, p.n3), (1, 0));
/// assert_eq!(unpack77(&p.bits).unwrap(), "CQ POTA K1ABC FN42");
/// assert_eq!(p.hashed_calls, ["K1ABC"]);
///
/// let p = pack77_message("<W9XYZ> PJ4/K1ABC RR73", &Pack77Options::new()).unwrap();
/// assert_eq!((p.i3, p.n3), (4, 0));
///
/// assert!(pack77_message("THIS IS FAR TOO LONG", &Pack77Options::new()).is_err());
/// ```
pub fn pack77_message(text: &str, options: &Pack77Options) -> Result<Packed77, Pack77Error> {
    // `character*37 msg0` (`:610`); a byte outside ASCII is in no alphabet.
    let msg: String = text
        .bytes()
        .take(37)
        .map(|b| if b < 0x80 { b as char } else { '\u{7f}' })
        .collect();
    match encode_core(&msg, options.prefer_wspr_50bit) {
        Ok(p) => Ok(p),
        // `pack77_encode_result` (`:610-627`): the WSPR shape wins over the
        // free-text verdict when the caller asked for the 50-bit family.
        Err(e) => {
            if e != Pack77Error::RoundtripRejected
                && failed_preferred_wspr_shape(&normalized_message(&msg), options.prefer_wspr_50bit)
            {
                Err(Pack77Error::PreferredFamilyRejected)
            } else {
                Err(e)
            }
        }
    }
}

// ── pack77_encode_core ──────────────────────────────────────────────────────

/// One candidate: bits, tag, and the calls it staged.
struct Candidate {
    bits: [u8; 77],
    i3: u8,
    n3: u8,
    facts: Vec<String>,
}

/// `pack77_encode_core` (`packjt77.f90:527-608`), without the DX macro.
fn encode_core(msg: &str, prefer_wspr_50bit: bool) -> Result<Packed77, Pack77Error> {
    // Convert msg to upper case; collapse multiple blanks; parse into words
    // (at most 19 of at most 13 characters).
    let words = split_words(msg, 19, 13);
    let special = raw_starts(msg, "CQ ") || raw_starts(msg, "DE ") || raw_starts(msg, "QRZ ");
    let Some(words) = words else {
        // word_overflow
        if !special && let Some(bits) = pack77_05(msg) {
            return Ok(plain(bits, 0, 5));
        }
        return free_text_exact(msg);
    };
    let nwords = words.len();
    if !special {
        // Check 0.1 (DXpedition mode)
        if let Some(p) = take_candidate(candidate_01(msg), msg) {
            return Ok(p);
        }
        // Check 0.2 (EU VHF contest exchange): commented out upstream (`:571`).
        // Check 0.3 and 0.4 (ARRL Field Day exchange)
        if let Some(p) = take_candidate(candidate_03(msg), msg) {
            return Ok(p);
        }
        if nwords < 2 {
            // Check 0.5 (telemetry)
            if let Some(bits) = pack77_05(msg) {
                return Ok(plain(bits, 0, 5));
            }
        }
    }
    for cand in [
        candidate_06(msg, prefer_wspr_50bit),
        candidate_1(msg),
        // Check Type 3 (ARRL RTTY contest exchange)
        candidate_3(msg),
        // Check Type 4 (One nonstandard call and one hashed call)
        candidate_4(msg),
        // Check Type 5 (EU VHF Contest with 2 hashed calls, report, serial, and grid6)
        candidate_5(msg),
    ] {
        if let Some(p) = take_candidate(cand, msg) {
            return Ok(p);
        }
    }
    // It defaults to exactly representable free text.
    free_text_exact(msg)
}

fn plain(bits: [u8; 77], i3: u8, n3: u8) -> Packed77 {
    Packed77 {
        bits,
        i3,
        n3,
        hashed_calls: Vec::new(),
    }
}

/// `take_candidate` (`packjt77.f90:200-225`): a candidate wins only if its bits
/// unpack to the message. One that does not is dropped (upstream records the
/// `INTERNAL_ROUNDTRIP_REJECTED` code, which the free-text step then overwrites).
fn take_candidate(candidate: Option<Candidate>, msg: &str) -> Option<Packed77> {
    let c = candidate?;
    if !gate(msg, &c) {
        return None;
    }
    let mut hashed_calls: Vec<String> = Vec::new();
    // Divergence from `pack77_hash_facts`, which lists a call once per use:
    // each call once, at its first place. `save_hash_call` keeps one entry per
    // 22-bit hash, in order of first commit, so the table is the same.
    for call in c.facts {
        if !hashed_calls.contains(&call) {
            hashed_calls.push(call);
        }
    }
    Some(Packed77 {
        bits: c.bits,
        i3: c.i3,
        n3: c.n3,
        hashed_calls,
    })
}

/// The part of `msg(1:pat.len())` test upstream does on the raw 37-character
/// message (`msg(1:3).eq.'CQ '`): the message padded with blanks, not folded.
fn raw_starts(msg: &str, pat: &str) -> bool {
    let b = msg.as_bytes();
    pat.bytes()
        .enumerate()
        .all(|(i, p)| b.get(i).copied().unwrap_or(b' ') == p)
}

/// `pack77_split_source_tokens` (`packjt77_grammar.f90:507-537`): upper-case, NUL
/// as blank, collapse blanks, split. `None` when there are more than `max_words`
/// words or a word longer than `max_len`.
fn split_words(msg: &str, max_words: usize, max_len: usize) -> Option<Vec<String>> {
    let mut words: Vec<String> = Vec::new();
    let mut prev_blank = true;
    for c in msg.trim_end_matches(' ').bytes() {
        let c = if c == 0 { b' ' } else { c.to_ascii_uppercase() };
        if c == b' ' {
            prev_blank = true;
            continue;
        }
        if prev_blank {
            if words.len() == max_words {
                return None;
            }
            words.push(String::new());
        }
        prev_blank = false;
        let w = words.last_mut()?;
        if w.len() == max_len {
            return None;
        }
        w.push(c as char);
    }
    Some(words)
}

/// The word at 1-based `k`, blank when there is none (upstream's arrays are
/// blank-filled).
fn word(w: &[String], k: usize) -> &str {
    w.get(k - 1).map_or("", String::as_str)
}

/// `pack77_normalized_message` (`packjt77.f90:785-803`), without the padding.
fn normalized_message(msg: &str) -> String {
    let mut out = String::new();
    let mut c0 = b' ';
    for c in msg.trim_end_matches(' ').bytes() {
        let mut c = if c == 0 { b' ' } else { c };
        if c == b' ' && c0 == b' ' {
            continue;
        }
        c = c.to_ascii_uppercase();
        out.push(c as char);
        c0 = c;
    }
    out.truncate(out.trim_end_matches(' ').len());
    out
}

/// `pack77_failed_preferred_wspr_shape` (`packjt77.f90:505-525`).
fn failed_preferred_wspr_shape(normalized: &str, prefer_wspr_50bit: bool) -> bool {
    prefer_wspr_50bit
        && split_words(normalized, 2, 13)
            .is_some_and(|w| w.len() == 2 && (5..=12).contains(&w[0].len()) && w[1].len() <= 6)
}

// ── free text and telemetry ─────────────────────────────────────────────────

/// `pack77_free_text_exact` (`packjt77.f90:719-729`) with
/// `pack77_free_text_reject_status` (`:750`) and `pack77_exact_free_text_ok`
/// (`:762`).
fn free_text_exact(msg: &str) -> Result<Packed77, Pack77Error> {
    let norm = normalized_message(msg);
    let alphabet_ok = norm
        .bytes()
        .all(|c| c == b' ' || super::FREE_TEXT.contains(&c));
    if norm.len() > 13 || !alphabet_ok {
        return Err(if !alphabet_ok || norm.len() <= 13 {
            Pack77Error::FreeTextInvalid
        } else {
            Pack77Error::FreeTextTooLong
        });
    }
    // An empty message is blank free text: 71 zero bits (`packtext77` of 13
    // blanks); `pack77_free_text` refuses the empty string.
    let bits = if norm.is_empty() {
        [0u8; 77]
    } else {
        pack77_free_text(&norm).ok_or(Pack77Error::FreeTextInvalid)?
    };
    Ok(plain(bits, 0, 0))
}

/// `pack77_05` (`packjt77.f90:687-717`) with `encode_pack77_telemetry`
/// (`packjt77_schema.f90:406`): 1 to 18 hex digits, `0.5`.
fn pack77_05(msg: &str) -> Option<[u8; 77]> {
    let text = msg.trim_start_matches(' ').trim_end_matches(' ');
    if text.is_empty() || text.len() > 18 {
        return None;
    }
    let mut hex = String::new();
    for ch in text.bytes() {
        let ch = ch.to_ascii_uppercase();
        if !ch.is_ascii_digit() && !(b'A'..=b'F').contains(&ch) {
            return None;
        }
        hex.push(ch as char);
    }
    // `read(c18,'(3z6)')` of the digits right-justified in 18 columns; blanks
    // read as zeros.
    let padded = format!("{hex:0>18}");
    let n1 = u64::from_str_radix(&padded[0..6], 16).ok()?;
    let n2 = u64::from_str_radix(&padded[6..12], 16).ok()?;
    let n3 = u64::from_str_radix(&padded[12..18], 16).ok()?;
    if n1 > 8_388_607 {
        return None; // ntel1 is 23 bits
    }
    let mut bits = [0u8; 77];
    put_bits(&mut bits, 1, 23, n1);
    put_bits(&mut bits, 24, 24, n2);
    put_bits(&mut bits, 48, 24, n3);
    put_bits(&mut bits, 72, 3, 5);
    Some(bits)
}

// ── hash facts and c28 lowering ─────────────────────────────────────────────

/// `ihashcall` with `ihashcall_from_n8` (`packjt77.f90:300-354`): the top `m` bits
/// of the low 64 bits of `47055833459 * n8`, by 16-bit limbs exactly as upstream
/// computes them (so a character outside the base-38 alphabet, index -1, gives
/// upstream's value, negative `n8` included). May be negative or out of range for
/// such input; the encoders refuse that.
fn ihashcall_limbs(c0: &str, m: u32) -> i64 {
    let b = c0.as_bytes();
    let mut n8: i64 = 0;
    for i in 0..11 {
        let c = b.get(i).copied().unwrap_or(b' ');
        let j = C38.iter().position(|&x| x == c).map_or(-1, |p| p as i64);
        n8 = n8.wrapping_mul(38).wrapping_add(j);
    }
    const HASH_FACTOR: [i64; 4] = [2419, 62_655, 10, 0];
    let mut limbs = [0i64; 4];
    let mut x = n8;
    for l in limbs.iter_mut() {
        *l = x % 65_536;
        x /= 65_536;
    }
    let mut p = [0i64; 4];
    let mut carry = 0i64;
    for i in 0..4 {
        let mut term = carry;
        for j in 0..=i {
            term += HASH_FACTOR[j] * limbs[i - j];
        }
        p[i] = term % 65_536;
        carry = term / 65_536;
    }
    if m <= 16 {
        p[3] / (1i64 << (16 - m))
    } else {
        p[3] * (1i64 << (m - 16)) + p[2] / (1i64 << (32 - m))
    }
}

/// `normalize_hash_call` (`packjt77.f90:392-412`): the call a staged token stands
/// for, without brackets. `None` for `<...>`, an invalid `<...>` token and a call
/// shorter than three characters.
fn normalize_hash_call(tok: &str) -> Option<String> {
    if tok.is_empty() || tok.starts_with("<...>") {
        return None;
    }
    let mut cw = tok;
    if cw.starts_with('<') {
        if !valid_hash_call_token(cw) {
            return None;
        }
        cw = &cw[1..];
        if let Some(i) = cw.find('>') {
            cw = &cw[..i];
        }
    }
    (cw.len() >= 3).then(|| cw.to_string())
}

/// `stage_hash_call` (`packjt77.f90:414-430`).
fn stage_hash_call(facts: &mut Vec<String>, tok: &str) {
    if let Some(cw) = normalize_hash_call(tok)
        && facts.len() < 32
    {
        facts.push(cw);
    }
}

/// `pack77_hash_index` (`packjt77.f90:432-446`): the `nbits`-bit hash of the call
/// in a `<CALL>` token (or a bare call), staging it.
fn hash_index(facts: &mut Vec<String>, tok: &str, nbits: u32) -> Option<u32> {
    let call = normalize_hash_call(tok)?;
    let n = ihashcall_limbs(&call, nbits);
    stage_hash_call(facts, &call);
    // `put_int` refuses a value outside the field.
    u32::try_from(n)
        .ok()
        .filter(|&n| u64::from(n) < 1u64 << nbits)
}

/// `pack77_valid_cq_modifier` (`packjt77_grammar.f90:324`): three digits, or one
/// to four letters.
fn valid_cq_modifier(tok: &str) -> bool {
    let b = tok.as_bytes();
    if b.is_empty() || b.len() > 4 {
        return false;
    }
    let letters = b.iter().filter(|c| c.is_ascii_uppercase()).count();
    let digits = b.iter().filter(|c| c.is_ascii_digit()).count();
    if letters + digits != b.len() {
        return false;
    }
    (digits == 3 && letters == 0) || ((1..=4).contains(&letters) && digits == 0)
}

/// `pack28_core` (`packjt77.f90:882-1001`), for the tokens the grammar hands it.
fn pack28_core(c13: &str, facts: &mut Vec<String>) -> u32 {
    let n28: i64 = 'n28: {
        // Check for special tokens first
        match c13 {
            "DE" => break 'n28 0,
            "QRZ" => break 'n28 1,
            "CQ" => break 'n28 2,
            _ => {}
        }
        if let Some(m) = c13.strip_prefix("CQ_")
            && !m.is_empty()
            && valid_cq_modifier(m)
        {
            if m.bytes().all(|c| c.is_ascii_digit()) {
                let nqsy = m.bytes().fold(0i64, |a, c| a * 10 + i64::from(c - b'0'));
                break 'n28 3 + nqsy;
            }
            // `c4=adjustr(c4)`: right-justified in four, blank as 0
            let mut mm = 0i64;
            for i in 0..4 {
                let k = (i + m.len()).wrapping_sub(4);
                let j = m.as_bytes().get(k).map_or(0, |&c| i64::from(c - b'A') + 1);
                mm = 27 * mm + j;
            }
            break 'n28 3 + 1000 + mm;
        }
        // Check for <...> callsign
        if c13.starts_with('<') {
            if !valid_hash_call_token(c13) {
                break 'n28 i64::from(NTOKENS);
            }
            stage_hash_call(facts, c13);
            let i2 = c13.find('>').unwrap_or(c13.len());
            let n22 = ihashcall_limbs(&c13[1..i2], 22);
            break 'n28 i64::from(NTOKENS) + n22;
        }
        // Check for standard callsign
        if !c28_standard_shape(c13) {
            // Treat this as a nonstandard callsign: compute its 22-bit hash
            stage_hash_call(facts, c13);
            break 'n28 i64::from(NTOKENS) + ihashcall_limbs(c13, 22);
        }
        let b = c13.as_bytes();
        let n = b.len();
        let iarea = (2..=n)
            .rev()
            .find(|&i| b[i - 1].is_ascii_digit())
            .unwrap_or(1);
        // This is a standard callsign
        stage_hash_call(facts, c13);
        let mut callsign = [b' '; 6];
        if iarea == 2 {
            callsign[1..6].copy_from_slice(&{
                let mut t = [b' '; 5];
                t[..n.min(5)].copy_from_slice(&b[..n.min(5)]);
                t
            });
        }
        if iarea == 3 {
            callsign[..n.min(6)].copy_from_slice(&b[..n.min(6)]);
        }
        let idx = |set: &[u8], c: u8| set.iter().position(|&x| x == c).map(|p| p as i64);
        let ix = [
            idx(C1, callsign[0]),
            idx(C2, callsign[1]),
            idx(C3, callsign[2]),
            idx(C4, callsign[3]),
            idx(C4, callsign[4]),
            idx(C4, callsign[5]),
        ];
        let [Some(i1), Some(i2), Some(i3), Some(i4), Some(i5), Some(i6)] = ix else {
            break 'n28 i64::from(NTOKENS) + ihashcall_limbs(c13, 22);
        };
        36 * 10 * 27 * 27 * 27 * i1
            + 10 * 27 * 27 * 27 * i2
            + 27 * 27 * 27 * i3
            + 27 * 27 * i4
            + 27 * i5
            + i6
            + i64::from(NTOKENS)
            + i64::from(MAX22)
    };
    (n28 & 0x0FFF_FFFF) as u32
}

/// `lower_c28_slot` (`packjt77.f90:2093-2128`): the 28-bit field for a call slot.
/// Some slots may auto-hash a non-standard call; stricter ones need an explicit
/// `<CALL>` token so the source text cannot choose a different family.
fn lower_c28_slot(token: &str, allow_auto_hash: bool, facts: &mut Vec<String>) -> Option<u32> {
    if token == "DE" || token == "QRZ" || token == "CQ" || token.starts_with("CQ_") {
        let mut trial = facts.clone();
        let n28 = pack28_core(token, &mut trial);
        if n28 < NTOKENS {
            return Some(n28);
        }
    }
    if valid_hash_call_token(token) || c28_standard_shape(token) {
        return Some(pack28_core(token, facts));
    }
    if !allow_auto_hash {
        return None;
    }
    Some(pack28_core(token, facts))
}

/// `pack77_c28_render_len` (`packjt77.f90:2131`): how long a call slot reads back,
/// `None` when it cannot.
fn c28_render_len(token: &str) -> Option<usize> {
    let m = token.len();
    if valid_hash_call_token(token) || c28_standard_shape(token) {
        return Some(m);
    }
    (m <= 11).then_some(m + 2)
}

/// `pack77_valid_c28_call_slot` (`packjt77_grammar.f90:898-916`).
fn valid_c28_call_slot(token: &str) -> bool {
    let n = token.len();
    if !(3..=13).contains(&n) || valid_hash_call_token(token) {
        return false;
    }
    if !token
        .bytes()
        .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == b'/')
    {
        return false;
    }
    if type12_standard_call(token) {
        return true;
    }
    let Some(slash) = token.find('/').map(|p| p + 1) else {
        return false;
    };
    if slash < 2 || slash >= n {
        return false;
    }
    !token[slash..].contains('/')
}

/// `pack77_parse_uint` (`packjt77_grammar.f90:379`).
fn parse_uint(tok: &str) -> Option<i64> {
    if tok.is_empty() || !tok.bytes().all(|c| c.is_ascii_digit()) {
        return None;
    }
    tok.parse().ok()
}

/// `pack77_parse_signed_report` (`packjt77_grammar.f90:393`): `+dd` or `-dd`.
fn parse_signed_report(tok: &str) -> Option<i32> {
    let b = tok.as_bytes();
    if b.len() != 3
        || (b[0] != b'+' && b[0] != b'-')
        || !b[1].is_ascii_digit()
        || !b[2].is_ascii_digit()
    {
        return None;
    }
    let v = i32::from(b[1] - b'0') * 10 + i32::from(b[2] - b'0');
    Some(if b[0] == b'-' { -v } else { v })
}

/// `pack77_report_index_from_snr` (`packjt77_grammar.f90:193`).
fn report_index_from_snr(isnr: i32) -> Option<u32> {
    if !(-50..=50).contains(&isnr) {
        return None;
    }
    let mut irpt = isnr;
    if (-50..=-31).contains(&irpt) {
        irpt += 101;
    }
    Some((irpt + 35) as u32)
}

// ── 0.1 DXpedition ──────────────────────────────────────────────────────────

/// `pack77_01_candidate` (`packjt77.f90:2191-2216`) with
/// `pack77_parse_dxpedition_source` (`packjt77_grammar.f90:918-944`),
/// `pack77_parse_dxpedition_report` (`:403`), `pack77_dxpedition_render_fits`
/// (`packjt77.f90:2148`) and `encode_pack77_dxpedition` (`packjt77_schema.f90:419`):
/// `K1ABC RR73; W9XYZ <KH1/KH7Z> -11`, 28 28 10 5.
fn candidate_01(msg: &str) -> Option<Candidate> {
    let w = split_words(msg, 5, 13)?;
    if w.len() != 5 || w[1] != "RR73;" {
        return None;
    }
    if !valid_c28_call_slot(&w[0]) || !valid_c28_call_slot(&w[2]) || !valid_hash_call_token(&w[3]) {
        return None;
    }
    let isnr = parse_signed_report(&w[4])?;
    if !(-30..=32).contains(&isnr) {
        return None;
    }
    let n5 = (isnr + 30) / 2;
    if 2 * n5 - 30 != isnr {
        return None;
    }
    // pack77_dxpedition_render_fits
    let n = c28_render_len(&w[0])? + 1 + 5 + 1 + c28_render_len(&w[2])? + 1 + w[3].len() + 1 + 3;
    if n > 37 {
        return None;
    }
    let mut facts = Vec::new();
    let n28a = lower_c28_slot(&w[0], true, &mut facts)?;
    let n28b = lower_c28_slot(&w[2], true, &mut facts)?;
    let n10 = hash_index(&mut facts, &w[3], 10)?;
    let mut bits = [0u8; 77];
    put_bits(&mut bits, 1, 28, u64::from(n28a));
    put_bits(&mut bits, 29, 28, u64::from(n28b));
    put_bits(&mut bits, 57, 10, u64::from(n10));
    put_bits(&mut bits, 67, 5, n5 as u64);
    put_bits(&mut bits, 72, 3, 1);
    Some(Candidate {
        bits,
        i3: 0,
        n3: 1,
        facts,
    })
}

// ── 0.3 / 0.4 ARRL Field Day ────────────────────────────────────────────────

/// `pack77_03_candidate` (`packjt77.f90:2218-2257`) with
/// `pack77_parse_field_day_source` (`packjt77_grammar.f90:946-977`),
/// `pack77_parse_field_day_exchange` (`:455`), `pack77_field_day_render_fits`
/// (`packjt77.f90:2162`) and `encode_pack77_field_day_{low,high}`
/// (`packjt77_schema.f90:450-466`): `WA9XYZ KA1ABC R 16A EMA`, 28 28 1 4 3 7.
fn candidate_03(msg: &str) -> Option<Candidate> {
    let w = split_words(msg, 5, 13)?;
    let nwords = w.len();
    if !(4..=5).contains(&nwords) {
        return None;
    }
    let section = &w[nwords - 1];
    let isec = ARRL_SECTIONS.iter().position(|s| *s == section.as_str())? + 1;
    if !valid_c28_call_slot(&w[0]) || !valid_c28_call_slot(&w[1]) {
        return None;
    }
    let mut ir = 0u64;
    if nwords == 5 {
        if w[2] != "R" {
            return None;
        }
        ir = 1;
    }
    // pack77_parse_field_day_exchange
    let ex = &w[nwords - 2];
    let n = ex.len();
    if n < 2 || (n > 2 && ex.starts_with('0')) {
        return None;
    }
    let ntx = parse_uint(&ex[..n - 1])?;
    if !(1..=32).contains(&ntx) {
        return None;
    }
    let nclass = i64::from(ex.as_bytes()[n - 1]) - i64::from(b'A');
    if !(0..=5).contains(&nclass) {
        return None;
    }
    // pack77_field_day_render_fits
    let nexch = if ntx >= 10 { 3 } else { 2 };
    let mut len =
        c28_render_len(&w[0])? + 1 + c28_render_len(&w[1])? + 1 + nexch + 1 + section.len();
    if ir == 1 {
        len += 2;
    }
    if len > 37 {
        return None;
    }
    let (n3, intx) = if ntx > 16 {
        (4, ntx - 17)
    } else {
        (3, ntx - 1)
    };
    let mut facts = Vec::new();
    let n28a = lower_c28_slot(&w[0], true, &mut facts)?;
    let n28b = lower_c28_slot(&w[1], true, &mut facts)?;
    let mut bits = [0u8; 77];
    put_bits(&mut bits, 1, 28, u64::from(n28a));
    put_bits(&mut bits, 29, 28, u64::from(n28b));
    put_bits(&mut bits, 57, 1, ir);
    put_bits(&mut bits, 58, 4, intx as u64);
    put_bits(&mut bits, 62, 3, nclass as u64);
    put_bits(&mut bits, 65, 7, isec as u64);
    put_bits(&mut bits, 72, 3, n3);
    Some(Candidate {
        bits,
        i3: 0,
        n3: n3 as u8,
        facts,
    })
}

// ── 0.6 WSPR ────────────────────────────────────────────────────────────────

/// `pack77_06_candidate` (`packjt77.f90:2259-2303`): the existing
/// [`pack77_wspr`], plus the calls it staged (`lower_c28_slot` /
/// `pack77_hash_index`): the call of the type-1 form, the base call of the
/// type-2 form, the call inside the `<CALL>` of the type-3 form.
fn candidate_06(msg: &str, prefer_wspr_50bit: bool) -> Option<Candidate> {
    let bits = pack77_wspr(msg, prefer_wspr_50bit)?;
    let w = split_words(msg, 3, 13)?;
    let first = word(&w, 1);
    let call = if bits[49] == 1 {
        // type 2: PFX/CALL or CALL/SFX, the base call is the part of 5+ characters
        // behind a prefix of at most 3 or before a suffix (`:2275`)
        let slash = first.find('/')? + 1;
        if slash <= 4 {
            &first[slash..]
        } else {
            &first[..slash - 1]
        }
        .to_string()
    } else if bits[48] == 0 {
        first.to_string()
    } else {
        normalize_hash_call(first)?
    };
    Some(Candidate {
        bits,
        i3: 0,
        n3: 6,
        facts: alloc::vec![call],
    })
}

// ── 1 / 2 standard ──────────────────────────────────────────────────────────

/// `pack77_type12_call_source` (`packjt77_grammar.f90:34`).
#[derive(Default)]
struct CallSource {
    c28_token: String,
    /// 0 none, 1 `/R`, 2 `/P`
    suffix: u8,
    is_hash: bool,
    is_cq: bool,
}

/// `pack77_parse_type12_call` (`packjt77_grammar.f90:759-815`).
fn parse_type12_call(token: &str, allow_special: bool) -> Option<CallSource> {
    let n = token.len();
    if !(1..=13).contains(&n) {
        return None;
    }
    if allow_special {
        if token == "CQ" {
            return Some(CallSource {
                c28_token: "CQ".into(),
                is_cq: true,
                ..Default::default()
            });
        }
        if token == "DE" || token == "QRZ" {
            return Some(CallSource {
                c28_token: token.into(),
                ..Default::default()
            });
        }
        if n >= 4 && token.starts_with("CQ_") {
            if !valid_cq_modifier(&token[3..]) {
                return None;
            }
            return Some(CallSource {
                c28_token: token.into(),
                is_cq: true,
                ..Default::default()
            });
        }
    }
    if token.starts_with('<') || token.contains('>') {
        if !valid_hash_call_token(token) {
            return None;
        }
        return Some(CallSource {
            c28_token: token.into(),
            is_hash: true,
            ..Default::default()
        });
    }
    let mut suffix = 0;
    let base = if let Some(slash) = token.find('/').map(|p| p + 1) {
        if slash < 2 || slash != n - 1 || token[slash..].contains('/') {
            return None;
        }
        suffix = match &token[slash - 1..] {
            "/R" => 1,
            "/P" => 2,
            _ => return None,
        };
        &token[..slash - 1]
    } else {
        token
    };
    if !type12_standard_call(base) {
        return None;
    }
    Some(CallSource {
        c28_token: base.into(),
        suffix,
        ..Default::default()
    })
}

/// `pack77_1_candidate` (`packjt77.f90:2305-2340`) with
/// `pack77_parse_type12_source` (`packjt77_grammar.f90:817-896`) and
/// `encode_pack77_type{1,2}` (`packjt77_schema.f90:529-545`): `CQ POTA K1ABC FN42`,
/// 28 1 28 1 1 15. The existing [`pack77`](super::pack77) takes the pieces
/// separately and has a narrower report range; this parses the text as upstream does.
fn candidate_1(msg: &str) -> Option<Candidate> {
    let w = split_words(msg, 5, 13)?;
    let nwords = w.len();
    if nwords < 2 {
        return None;
    }
    let mut call_1_token = w[0].clone();
    let mut call_2_index = 2;
    let mut tail_start = 3;
    if w[0] == "CQ" && nwords >= 3 && valid_cq_modifier(&w[1]) {
        call_1_token = format!("CQ_{}", w[1]);
        call_2_index = 3;
        tail_start = 4;
    }
    let call_1 = parse_type12_call(&call_1_token, true)?;
    let call_2 = parse_type12_call(word(&w, call_2_index), false)?;
    if call_1.is_cq && call_2.is_hash {
        return None;
    }
    if call_1.is_hash && (call_2.suffix != 0 || call_2.c28_token.contains('/')) {
        return None;
    }
    if call_2.is_hash && (call_1.suffix != 0 || call_1.c28_token.contains('/')) {
        return None;
    }
    if (call_1.suffix == 1 && call_2.suffix == 2) || (call_1.suffix == 2 && call_2.suffix == 1) {
        return None;
    }
    let i3: u8 = if call_1.suffix == 2 || call_2.suffix == 2 {
        2
    } else {
        1
    };

    let ntail = nwords + 1 - tail_start;
    if ntail > 2 {
        return None;
    }
    let (ir, igrid4): (u64, u32);
    if ntail == 0 {
        ir = 0;
        igrid4 = MAX_GRID4 + 1;
    } else if ntail == 2 {
        if word(&w, tail_start) != "R" || call_1.is_cq {
            return None;
        }
        let g = word(&w, tail_start + 1);
        if g.len() != 4 || !is_grid4(g) {
            return None;
        }
        ir = 1;
        igrid4 = grid4_index(g);
    } else {
        let tail = word(&w, tail_start);
        if tail.len() == 4 && is_grid4(tail) {
            ir = 0;
            igrid4 = grid4_index(tail);
        } else if call_1.is_cq {
            return None;
        } else if tail.starts_with('+') || tail.starts_with('-') {
            ir = 0;
            igrid4 = MAX_GRID4 + report_index_from_snr(parse_signed_report(tail)?)?;
        } else if tail.starts_with("R+") || tail.starts_with("R-") {
            ir = 1;
            igrid4 = MAX_GRID4 + report_index_from_snr(parse_signed_report(&tail[1..])?)?;
        } else {
            // pack77_qso_tail_index, pack77_type12_irpt_from_tail
            let itail = match tail {
                "RRR" => 1,
                "RR73" => 2,
                "73" => 3,
                _ => return None,
            };
            ir = 0;
            igrid4 = MAX_GRID4 + itail + 1;
        }
    }

    let mut facts = Vec::new();
    let n28a = lower_c28_slot(&call_1.c28_token, false, &mut facts)?;
    let n28b = lower_c28_slot(&call_2.c28_token, false, &mut facts)?;
    let mut bits = [0u8; 77];
    put_bits(&mut bits, 1, 28, u64::from(n28a));
    put_bits(&mut bits, 29, 1, u64::from(call_1.suffix != 0));
    put_bits(&mut bits, 30, 28, u64::from(n28b));
    put_bits(&mut bits, 58, 1, u64::from(call_2.suffix != 0));
    put_bits(&mut bits, 59, 1, ir);
    put_bits(&mut bits, 60, 15, u64::from(igrid4));
    put_bits(&mut bits, 75, 3, u64::from(i3));
    Some(Candidate {
        bits,
        i3,
        n3: 0,
        facts,
    })
}

/// `pack77_grid4_index` (`packjt77_grammar.f90:1465`), for a valid grid4.
fn grid4_index(g: &str) -> u32 {
    let b = g.as_bytes();
    u32::from(b[0] - b'A') * 1800
        + u32::from(b[1] - b'A') * 100
        + u32::from(b[2] - b'0') * 10
        + u32::from(b[3] - b'0')
}

// ── 3 ARRL RTTY Roundup ─────────────────────────────────────────────────────

/// `pack77_3_candidate` (`packjt77.f90:2342-2375`) with
/// `pack77_parse_type3_source` (`packjt77_grammar.f90:1002-1054`),
/// `pack77_parse_type3_serial` (`:417`), `pack77_type3_display_fits` (`:244`) and
/// `encode_pack77_type3` (`packjt77_schema.f90:547`):
/// `TU; W9XYZ K1ABC R 579 MA`, 1 28 28 1 3 13.
fn candidate_3(msg: &str) -> Option<Candidate> {
    let w = split_words(msg, 6, 13)?;
    let nwords = w.len();
    if !(4..=6).contains(&nwords) {
        return None;
    }
    let itu = usize::from(w[0] == "TU;");
    let ir = usize::from(word(&w, 3 + itu) == "R");
    let report_index = 3 + itu + ir;
    let call_1 = word(&w, 1 + itu);
    let call_2 = word(&w, 2 + itu);
    if !valid_c28_call_slot(call_1) || !valid_c28_call_slot(call_2) {
        return None;
    }
    let exchange_index = report_index + 1;
    if exchange_index != nwords {
        return None;
    }
    // pack77_rtty_report_index
    let rpt = word(&w, report_index).as_bytes();
    if rpt.len() != 3 || rpt[0] != b'5' || !(b'2'..=b'9').contains(&rpt[1]) || rpt[2] != b'9' {
        return None;
    }
    let irpt = u64::from(rpt[1] - b'2');
    let ex = word(&w, exchange_index);
    let nserial = if ex.len() == 4 { parse_uint(ex) } else { None };
    let nexch: u64 = if let Some(n) = nserial.filter(|n| (1..=7999).contains(n)) {
        n as u64
    } else {
        let imult = RTTY_STATES.iter().position(|m| *m == ex)? + 1;
        8000 + imult as u64
    };

    let mut facts = Vec::new();
    let n28a = lower_c28_slot(call_1, true, &mut facts)?;
    let n28b = lower_c28_slot(call_2, true, &mut facts)?;
    // pack77_type3_display_fits
    let exchange_len = if nexch < 8000 { 4 } else { ex.len() };
    let display_len = |n28: u32, call: &str| -> Option<usize> {
        let mut n = call.len();
        if (NTOKENS..NTOKENS + MAX22).contains(&n28) {
            if n > 11 {
                return None;
            }
            n += 2;
        }
        Some(n)
    };
    let (l1, l2) = (display_len(n28a, call_1)?, display_len(n28b, call_2)?);
    // pack77_type3_message_len
    let mut nmsg = l1 + 1 + l2 + 1 + 3 + 1 + exchange_len;
    if ir == 1 {
        nmsg = l1 + 1 + l2 + 3 + 3 + 1 + exchange_len;
    }
    if itu == 1 {
        nmsg += 4;
    }
    if nmsg > 37 {
        return None;
    }
    let mut bits = [0u8; 77];
    put_bits(&mut bits, 1, 1, itu as u64);
    put_bits(&mut bits, 2, 28, u64::from(n28a));
    put_bits(&mut bits, 30, 28, u64::from(n28b));
    put_bits(&mut bits, 58, 1, ir as u64);
    put_bits(&mut bits, 59, 3, irpt);
    put_bits(&mut bits, 62, 13, nexch);
    put_bits(&mut bits, 75, 3, 3);
    Some(Candidate {
        bits,
        i3: 3,
        n3: 0,
        facts,
    })
}

// ── 4 one non-standard call ─────────────────────────────────────────────────

/// `pack77_valid_type4_c11` (`packjt77_grammar.f90:1064`).
fn valid_type4_c11(tok: &str) -> bool {
    (1..=11).contains(&tok.len()) && tok.bytes().all(|c| C38.contains(&c))
}

/// `pack77_4_candidate` (`packjt77.f90:2377-2404`) with
/// `pack77_parse_type4_source` (`packjt77_grammar.f90:1056-1109`) and
/// `encode_pack77_type4` (`packjt77_schema.f90:562`):
/// `<WA9XYZ> PJ4/KA1ABC RR73`, 12 58 1 2 1.
fn candidate_4(msg: &str) -> Option<Candidate> {
    let w = split_words(msg, 3, 13)?;
    let nwords = w.len();
    if !(2..=3).contains(&nwords) {
        return None;
    }
    let hash_1 = valid_hash_call_token(&w[0]);
    let hash_2 = valid_hash_call_token(&w[1]);
    let (c11, icq, iflip, itail, hash_token): (&str, u8, u8, u64, &str);
    if w[0] == "CQ" {
        if nwords != 2 || hash_2 {
            return None;
        }
        let c = w[1].as_str();
        if c.len() < 5 || !valid_type4_c11(c) || type12_standard_call(c) {
            return None;
        }
        c11 = c;
        icq = 1;
        iflip = 0;
        itail = 0;
        hash_token = c; // `if(icq.eq.1 .and. len_trim(hash_token).eq.0) hash_token=c11`
    } else {
        itail = if nwords == 3 {
            match w[2].as_str() {
                "RRR" => 1,
                "RR73" => 2,
                "73" => 3,
                _ => return None,
            }
        } else {
            0
        };
        if hash_1 && !hash_2 {
            if !valid_type4_c11(&w[1]) || type12_standard_call(&w[1]) {
                return None;
            }
            hash_token = w[0].as_str();
            c11 = w[1].as_str();
            iflip = 0;
        } else if hash_2 && !hash_1 {
            if !valid_type4_c11(&w[0]) || type12_standard_call(&w[0]) {
                return None;
            }
            hash_token = w[1].as_str();
            c11 = w[0].as_str();
            iflip = 1;
        } else {
            return None;
        }
        icq = 0;
    }
    let mut facts = Vec::new();
    let n12 = hash_index(&mut facts, hash_token, 12)?;
    // pack77_type4_n58_from_call_text(adjustr(c11))
    let mut n58: u64 = 0;
    let lead = 11 - c11.len();
    for i in 0..11 {
        let c = if i < lead {
            b' '
        } else {
            c11.as_bytes()[i - lead]
        };
        n58 = n58 * 38 + C38.iter().position(|&x| x == c)? as u64;
    }
    let mut bits = [0u8; 77];
    put_bits(&mut bits, 1, 12, u64::from(n12));
    put_bits(&mut bits, 13, 58, n58);
    put_bits(&mut bits, 71, 1, u64::from(iflip));
    put_bits(&mut bits, 72, 2, itail);
    put_bits(&mut bits, 74, 1, u64::from(icq));
    put_bits(&mut bits, 75, 3, 4);
    Some(Candidate {
        bits,
        i3: 4,
        n3: 0,
        facts,
    })
}

// ── 5 EU VHF ────────────────────────────────────────────────────────────────

/// `pack77_5_candidate` (`packjt77.f90:2406-2436`) with
/// `pack77_parse_type5_source` (`packjt77_grammar.f90:1111-1155`),
/// `pack77_is_grid6` (`:1389`), `pack77_grid6_index` (`:1474`) and
/// `encode_pack77_type5` (`packjt77_schema.f90:577`):
/// `<PA3XYZ> <G4ABC/P> R 590003 IO91NP`, h12 h22 r1 s3 s11 g25.
fn candidate_5(msg: &str) -> Option<Candidate> {
    let w = split_words(msg, 5, 13)?;
    let nwords = w.len();
    if !(4..=5).contains(&nwords) {
        return None;
    }
    if !valid_hash_call_token(&w[0]) || !valid_hash_call_token(&w[1]) {
        return None;
    }
    let mut ir = 0u64;
    if nwords == 5 {
        if w[2] != "R" {
            return None;
        }
        ir = 1;
    }
    let exchange = &w[nwords - 2];
    let grid = &w[nwords - 1];
    let nx = if exchange.len() == 6 {
        parse_uint(exchange)?
    } else {
        return None;
    };
    if !(520_001..=592_047).contains(&nx) {
        return None;
    }
    let iserial = nx % 10_000;
    if !(1..=2047).contains(&iserial) {
        return None;
    }
    let g = grid.as_bytes();
    if g.len() != 6
        || !is_grid4(&grid[..4])
        || !(b'A'..=b'X').contains(&g[4])
        || !(b'A'..=b'X').contains(&g[5])
    {
        return None;
    }
    let irpt = nx / 10_000 - 52;
    let mut facts = Vec::new();
    let n12 = hash_index(&mut facts, &w[0], 12)?;
    let n22 = hash_index(&mut facts, &w[1], 22)?;
    let igrid6 = u64::from(grid4_index(&grid[..4])) * 24 * 24
        + u64::from(g[4] - b'A') * 24
        + u64::from(g[5] - b'A');
    let mut bits = [0u8; 77];
    put_bits(&mut bits, 1, 12, u64::from(n12));
    put_bits(&mut bits, 13, 22, u64::from(n22));
    put_bits(&mut bits, 35, 1, ir);
    put_bits(&mut bits, 36, 3, irpt as u64);
    put_bits(&mut bits, 39, 11, iserial as u64);
    put_bits(&mut bits, 50, 25, igrid6);
    put_bits(&mut bits, 75, 3, 5);
    Some(Candidate {
        bits,
        i3: 5,
        n3: 0,
        facts,
    })
}

// ── the round-trip gate ─────────────────────────────────────────────────────

/// The check `take_candidate` makes: the bits unpack, with an empty hash table, to
/// the same `i3.n3` and to the words that were asked for. A hashed call comes back
/// as `<...>`, which matches when the call asked for is one the candidate staged
/// and the message still fits in 37 characters with the call spelled out.
/// `pack77_gate_message_checks` (`packjt77_grammar.f90:641-705`).
fn gate(msg: &str, c: &Candidate) -> bool {
    let Some(decoded) = unpack77(&c.bits) else {
        return false;
    };
    let di3 = read_bits(&c.bits, 74, 3);
    let dn3 = if di3 == 0 {
        read_bits(&c.bits, 71, 3)
    } else {
        0
    };
    if di3 != u32::from(c.i3) || dn3 != u32::from(c.n3) {
        return false;
    }
    // The tokens of both sides: up to 19 words of up to 37 characters.
    let (Some(iw), Some(dw)) = (split_words(msg, 19, 37), split_words(&decoded, 19, 37)) else {
        return false;
    };
    if iw.len() != dw.len() {
        return false;
    }
    let mut nrender = 0usize;
    let mut messages_match = true;
    let mut resolved_render_fits = true;
    for (i, (a, d)) in iw.iter().zip(&dw).enumerate() {
        if i > 0 {
            nrender += 1;
        }
        let mut token_match = false;
        if a == d || *d == format!("<{a}>") {
            token_match = true;
            nrender += d.len();
        } else if d == "<...>" {
            let mut body = a.as_str();
            if body.len() >= 2 && body.starts_with('<') && body.ends_with('>') {
                body = if body.len() > 2 {
                    &body[1..body.len() - 1]
                } else {
                    ""
                };
            }
            token_match = c.facts.iter().any(|s| s == body);
            if body.len() > 11 {
                resolved_render_fits = false;
            }
            nrender += body.len() + 2;
        } else {
            nrender += d.len();
        }
        if !token_match {
            messages_match = false;
        }
        if d == "<...>" && !token_match {
            resolved_render_fits = false;
        }
        if nrender > 37 {
            resolved_render_fits = false;
        }
    }
    messages_match && resolved_render_fits
}
