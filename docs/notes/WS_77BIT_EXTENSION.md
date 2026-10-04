# WS (former WSJT-X Improved): the 77-bit message extension (issue #568)

Investigation of 2026-10-04. **Reference: WSJT-X `v3.2.0-rc1`.** WSJT-X's QSO sequencing (Auto Seq) is outside what this crate
ports, so what follows about sequencing is context for the packing results, not a requirement on the crate.

## 1. Summary

- **At the bit level the extension changes nothing.** WS 3.1.0 and 3.2.1 pack the same 77-bit layouts as WSJT-X; `packjt77.f90`
  differs from WSJT-X 3.0.2 only in declarations and in how the 22-bit hash multiplies (`mulmod64`). All 168 messages WS composes in the
  announced QSO patterns pack to **identical bits in WSJT-X 3.0.2, v3.2.0-rc1 and WS**, and decode identically in all three. It uses
  types 1 and 4 only. In the issue's terms: **case C** (existing formats, new operating combinations); no unused code point is taken
  (0/2, 0/7+, i3 6-7 are untouched) and no existing field is reinterpreted *in the bits*.
- **What did change is the text WS builds for each QSO step** (`MainWindow::genStdMsgs`, `widgets/mainwindow.cpp`). Where WSJT-X
  silently loses information, WS arranges the same fields so it survives:
  - Of the 72 messages in the announced patterns (RR73 variant), WSJT-X 3.0.2 packs 42 lossily when given the stock texts: all 24
    report messages (tx2, tx3) as type 4, which has no report field, so **the report is dropped**; 10 tx1 messages whose grid type 4
    cannot hold; and 8 RR73/73 messages between a non-standard call and a non-standard or compound one as **free text cut to 13
    characters** (`W250USA DG123`). v3.2.0-rc1 refuses exactly those 42 (the pack fails) instead of sending them.
  - WS packs the same steps as type 1 with hashed calls (`<W250USA> <DG123YCB> -05`) or type 4 with a hashed partner
    (`W250USA <DG123YCB> RR73`); nothing is lost in packing (0 of 72).
- **The price is paid in the identity fields.** In 14 of the 24 report messages WS sends the *base* call where the station's call has
  a suffix (`<W1XYZ/YOTA> DG2YCB -05` from `DG2YCB/QRP`). A faithful receiver shows `DG2YCB`. Hashed forms also need prior knowledge:
  a bystander, or a station that has not yet seen the full call, shows `<...>`.
- **Receivers are not harmed at the decode level.** WSJT-X 3.0.2 and rc1 decode every WS message as written, or as `<...>` when the hash
  is unknown; none decodes to a wrong call in the 168 cases. (A 22-bit or 12-bit hash can in principle collide with another stored
  call; see 5.3.)
- **This crate does not crash or mis-decode, but it differs from WSJT-X in one place**: calls carrying `/P` or `/R` are saved in the
  hash table without the suffix, so a `<W1XYZ/P>` hash stays `<...>` where WSJT-X resolves it. 32 of the 336 receiver outcomes (168 messages x
  2 table states) differ, all of that one kind. A patch is proposed (6); with it all 336 agree and the tier A+B gate passes (1072 passed,
  0 failed).
- **The announced "endless Auto Seq loop" is not a change in WS's receive logic**: `processMessage`'s decision ladder and
  `DecodedText` are identical to WSJT-X 3.0.2 apart from FT2 additions. A desk trace of the 3.0.2/WS code (not executed; rc1's
  `processMessage` was restructured and was not traced) shows how a stock station and a WS station stall (5.2).

## 2. Sources

| what | version | identifier |
|---|---|---|
| WS 3.2.1 | `ws-3.2.1_260926.tgz`, inner `ws.tgz` | sha256 `64733ed64eea189b109974e2cb93c6210b53b191a4c3588eebfa0cd30f73ccab`; inner md5 `c37a4c04f5ac9b12322e7fb2616c2d63` |
| WSJT-X Improved 3.1.0 | `wsjtx-3.1.0_improved_PLUS_260522.tgz`, inner `wsjtx.tgz` | sha256 `f159cde764ababb2c63ee8f307f18f29db0db322e152622cc5b4e2c7575fb5a0`; inner md5 `840a3ad23af07d292c8ed052d83c293e` |
| WSJT-X (reference) | `v3.2.0-rc1` | `567ad29ce` |
| WSJT-X (WS's base line) | `upstream/master`, 3.0.2 + 3 commits | `967c85a61` |
| mfsk-core | 0.13.0 plus later docs commits | `main` at `0a7fab35` |

Retrieved 2026-10-04 (`git fetch upstream`; SourceForge tarballs). The WS archives are wrappers; the sources are the inner archives.
Not available or not examined: the first WS 3.1.0 build with the extension (260226 / 260228; only the 260522 source remains on
SourceForge), WS 3.0.0 (251212), WS 3.2.0, WSJT-X 2.7.0, JTDX, MSHV. All WS sources are GPL v3 (their README, NEWS and `COPYING`; see #569).

## 3. WSJT-X 77-bit message types

From `lib/77bit/packjt77.f90` of WSJT-X 3.0.2 (`967c85a61`; the unpacker's dispatch is at lines 368-695). rc1 restructured the file
(`packjt77_schema.f90`, `packjt77_grammar.f90`); this table was **not** re-derived from rc1's source, but all 168 test messages
pack and unpack identically in rc1.

| i3.n3 | message | fields (bits) | notes |
|---|---|---|---|
| 0.0 | free text | 71 (13 characters) | `packjt77.f90:368` |
| 0.1 | DXpedition `K1ABC RR73; W9XYZ <KH1/KH7Z> -11` | 28 28 10 5 | hash10 |
| 0.2 | **unused** | | unpack fails (`:403`) |
| 0.3, 0.4 | ARRL Field Day | 28 28 1 4 3 7 | |
| 0.5 | telemetry | 71 | |
| 0.6 | WSPR types 1-3 | by bits 48-50 | |
| 0.7+ | **unused** | | unpack fails (`:530`) |
| 1, 2 | standard (`/R` for 1, `/P` for 2) | 28 1 28 1 1 15 | c28 may be a **22-bit hash** token; the 15-bit field is grid4 or report |
| 3 | ARRL RTTY Roundup | 1 28 28 1 3 13 | |
| 4 | one non-standard call | 12 58 1 2 1 | **hash12** of the other call, the 58-bit call, `iflip`, `nrpt` (none/RRR/RR73/73), `icq`; **no report and no grid** |
| 5 | EU VHF contest | 12 22 1 3 11 25 | |
| 6, 7 | **undefined** | | `:695` |

c28 space (`NTOKENS = 2063592`, `MAX22 = 4194304`): special tokens, then 22-bit hash tokens, then standard calls. The hash is
`ihashcall(call, m)`: the top m bits of the low 64 bits of `47055833459 * n8`, where `n8` is the call read in base 38. A call is saved
in the tables by `save_hash_call` (`:150-184`) **as written, including any `/P` or `/R`** (it removes only angle brackets).

## 4. What WS changed

### 4.1 `lib/77bit/packjt77.f90`: no layout change

`diff` of WSJT-X 3.0.2 against WS 3.1.0 (line endings ignored): `implicit none` and explicit integer declarations throughout,
`integer, parameter` instead of `parameter`, `npfx=0` initialisations, one `int(mod(n58, 38_8), kind=4)`, and `ihashcall` /
`ihashcallvar` computing the product with `mulmod64` (`lib/77bit/mulmod64.c`, the low 64 bits of a 128-bit product) instead of
`ihashcall_from_n8`. 295 diff lines, none in the pack/unpack branches. WS 3.1.0 and 3.2.1 have byte-identical `packjt77.f90`.
`packjt77_grammar.f90` in WS 3.2.1 is the 3.2 line's (it is in rc1 too) and is used by JTTY.

### 4.2 Message composition (`genStdMsgs`, `genCQMsg`)

WS adds four blocks to `genStdMsgs` (WS 3.1.0 `widgets/mainwindow.cpp`, around lines 9852-9965), all guarded by
`is77BitMode() && SpecOp::NONE == m_specOp` and `Radio::is_77bit_nonstandard_callsign(...)`
(`^[A-Z0-9/]{3,11}$` but not the strict standard form; `Radio.cpp:147`, unchanged from WSJT-X):
(a) tx1 without a grid for some non-standard cases; (b) tx2/tx3 prefixes built from `<his> <my>`, `his-base <my>` or
`<his> my-base`; (c) tx4 and (d) tx5 as `his <my>` forms. `genCQMsg`, `elide_tx1_not_allowed` and `Radio.cpp` are unchanged.

What each step sends, as composed by WS for the first station of each announced pattern (RR73 variant, report -05, grid JO31;
`i3` is the type it packs to; receivers are WSJT-X 3.0.2 and v3.2.0-rc1, which agree):

| pattern | sender | slot | WS sends | i3 | empty tables shows | after hearing the sender's CQ shows |
|---|---|---|---|---|---|---|
| nonstd/nonstd | DG123YCB | CQ | `CQ DG123YCB` | 4 | `CQ DG123YCB` | `CQ DG123YCB` |
| nonstd/nonstd | DG123YCB | tx1 | `<W250USA> DG123YCB` | 4 | `<W250USA> DG123YCB` | `<W250USA> DG123YCB` |
| nonstd/nonstd | DG123YCB | tx2 | `<W250USA> <DG123YCB> -05` | 1 | `<W250USA> <...> -05` | `<W250USA> <DG123YCB> -05` |
| nonstd/nonstd | DG123YCB | tx3 | `<W250USA> <DG123YCB> R-05` | 1 | `<W250USA> <...> R-05` | `<W250USA> <DG123YCB> R-05` |
| nonstd/nonstd | DG123YCB | tx4 | `W250USA <DG123YCB> RR73` | 4 | `W250USA <...> RR73` | `W250USA <DG123YCB> RR73` |
| nonstd/nonstd | DG123YCB | tx5 | `W250USA <DG123YCB> 73` | 4 | `W250USA <...> 73` | `W250USA <DG123YCB> 73` |
| compound/compound-1 | DG2YCB/QRP | CQ | `CQ DG2YCB/QRP` | 4 | `CQ DG2YCB/QRP` | `CQ DG2YCB/QRP` |
| compound/compound-1 | DG2YCB/QRP | tx1 | `<W1XYZ/YOTA> DG2YCB/QRP` | 4 | `<W1XYZ/YOTA> DG2YCB/QRP` | `<W1XYZ/YOTA> DG2YCB/QRP` |
| compound/compound-1 | DG2YCB/QRP | tx2 | `<W1XYZ/YOTA> DG2YCB -05` | 1 | `<W1XYZ/YOTA> DG2YCB -05` | `<W1XYZ/YOTA> DG2YCB -05` |
| compound/compound-1 | DG2YCB/QRP | tx3 | `<W1XYZ/YOTA> DG2YCB R-05` | 1 | `<W1XYZ/YOTA> DG2YCB R-05` | `<W1XYZ/YOTA> DG2YCB R-05` |
| compound/compound-1 | DG2YCB/QRP | tx4 | `W1XYZ/YOTA <DG2YCB/QRP> RR73` | 4 | `W1XYZ/YOTA <...> RR73` | `W1XYZ/YOTA <DG2YCB/QRP> RR73` |
| compound/compound-1 | DG2YCB/QRP | tx5 | `W1XYZ/YOTA <DG2YCB/QRP> 73` | 4 | `W1XYZ/YOTA <...> 73` | `W1XYZ/YOTA <DG2YCB/QRP> 73` |
| compound/compound-2 | HB9/DG2YCB | CQ | `CQ HB9/DG2YCB` | 4 | `CQ HB9/DG2YCB` | `CQ HB9/DG2YCB` |
| compound/compound-2 | HB9/DG2YCB | tx1 | `<KP5/W1XY> HB9/DG2YCB` | 4 | `<KP5/W1XY> HB9/DG2YCB` | `<KP5/W1XY> HB9/DG2YCB` |
| compound/compound-2 | HB9/DG2YCB | tx2 | `<KP5/W1XY> DG2YCB -05` | 1 | `<KP5/W1XY> DG2YCB -05` | `<KP5/W1XY> DG2YCB -05` |
| compound/compound-2 | HB9/DG2YCB | tx3 | `<KP5/W1XY> DG2YCB R-05` | 1 | `<KP5/W1XY> DG2YCB R-05` | `<KP5/W1XY> DG2YCB R-05` |
| compound/compound-2 | HB9/DG2YCB | tx4 | `KP5/W1XY <HB9/DG2YCB> RR73` | 4 | `KP5/W1XY <...> RR73` | `KP5/W1XY <HB9/DG2YCB> RR73` |
| compound/compound-2 | HB9/DG2YCB | tx5 | `KP5/W1XY <HB9/DG2YCB> 73` | 4 | `KP5/W1XY <...> 73` | `KP5/W1XY <HB9/DG2YCB> 73` |
| nonstd/compound | DG123YCB | CQ | `CQ DG123YCB` | 4 | `CQ DG123YCB` | `CQ DG123YCB` |
| nonstd/compound | DG123YCB | tx1 | `<W1XYZ/QRP> DG123YCB` | 4 | `<W1XYZ/QRP> DG123YCB` | `<W1XYZ/QRP> DG123YCB` |
| nonstd/compound | DG123YCB | tx2 | `W1XYZ <DG123YCB> -05` | 1 | `W1XYZ <...> -05` | `W1XYZ <DG123YCB> -05` |
| nonstd/compound | DG123YCB | tx3 | `W1XYZ <DG123YCB> R-05` | 1 | `W1XYZ <...> R-05` | `W1XYZ <DG123YCB> R-05` |
| nonstd/compound | DG123YCB | tx4 | `W1XYZ/QRP <DG123YCB> RR73` | 4 | `W1XYZ/QRP <...> RR73` | `W1XYZ/QRP <DG123YCB> RR73` |
| nonstd/compound | DG123YCB | tx5 | `W1XYZ/QRP <DG123YCB> 73` | 4 | `W1XYZ/QRP <...> 73` | `W1XYZ/QRP <DG123YCB> 73` |
| nonstd/P | DG123YCB | CQ | `CQ DG123YCB` | 4 | `CQ DG123YCB` | `CQ DG123YCB` |
| nonstd/P | DG123YCB | tx1 | `<W1XYZ/P> DG123YCB` | 4 | `<W1XYZ/P> DG123YCB` | `<W1XYZ/P> DG123YCB` |
| nonstd/P | DG123YCB | tx2 | `W1XYZ <DG123YCB> -05` | 1 | `W1XYZ <...> -05` | `W1XYZ <DG123YCB> -05` |
| nonstd/P | DG123YCB | tx3 | `W1XYZ <DG123YCB> R-05` | 1 | `W1XYZ <...> R-05` | `W1XYZ <DG123YCB> R-05` |
| nonstd/P | DG123YCB | tx4 | `<W1XYZ/P> DG123YCB RR73` | 4 | `<W1XYZ/P> DG123YCB RR73` | `<W1XYZ/P> DG123YCB RR73` |
| nonstd/P | DG123YCB | tx5 | `<W1XYZ/P> DG123YCB 73` | 4 | `<W1XYZ/P> DG123YCB 73` | `<W1XYZ/P> DG123YCB 73` |
| std+suffix/std+suffix | DG2YCB/MM | CQ | `CQ DG2YCB/MM` | 4 | `CQ DG2YCB/MM` | `CQ DG2YCB/MM` |
| std+suffix/std+suffix | DG2YCB/MM | tx1 | `<W1XYZ/P> DG2YCB/MM` | 4 | `<W1XYZ/P> DG2YCB/MM` | `<W1XYZ/P> DG2YCB/MM` |
| std+suffix/std+suffix | DG2YCB/MM | tx2 | `W1XYZ <DG2YCB/MM> -05` | 1 | `W1XYZ <...> -05` | `W1XYZ <DG2YCB/MM> -05` |
| std+suffix/std+suffix | DG2YCB/MM | tx3 | `W1XYZ <DG2YCB/MM> R-05` | 1 | `W1XYZ <...> R-05` | `W1XYZ <DG2YCB/MM> R-05` |
| std+suffix/std+suffix | DG2YCB/MM | tx4 | `<W1XYZ/P> DG2YCB/MM RR73` | 4 | `<W1XYZ/P> DG2YCB/MM RR73` | `<W1XYZ/P> DG2YCB/MM RR73` |
| std+suffix/std+suffix | DG2YCB/MM | tx5 | `<W1XYZ/P> DG2YCB/MM 73` | 4 | `<W1XYZ/P> DG2YCB/MM 73` | `<W1XYZ/P> DG2YCB/MM 73` |

Selected stock-versus-WS differences (WSJT-X 3.0.2 packing of the stock texts, `rt` = what its own unpacker returns):

| pattern | sender | step | stock text | stock `rt` | WS text | WS `rt` |
|---|---|---|---|---|---|---|
| nonstd/nonstd | DG123YCB | tx2 | `<W250USA> DG123YCB -05` | `<W250USA> DG123YCB` (report lost) | `<W250USA> <DG123YCB> -05` | same |
| nonstd/nonstd | DG123YCB | tx4 | `W250USA DG123YCB RR73` | `W250USA DG123` (free text, cut) | `W250USA <DG123YCB> RR73` | same |
| compound | DG2YCB/QRP | tx2 | `<W1XYZ/YOTA> DG2YCB/QRP -05` | `<W1XYZ/YOTA> DG2YCB/QRP` (report lost) | `<W1XYZ/YOTA> DG2YCB -05` | same (suffix not sent) |
| nonstd/P | DG123YCB | tx2 | `W1XYZ/P <DG123YCB> -05` | `W1XYZ/P <DG123YCB>` (report lost) | `W1XYZ <DG123YCB> -05` | same (suffix not sent) |

Counts over the 72 steps of the six announced patterns (the control pair of two standard calls is identical in both): 50 texts differ,
40 pack to different bits; stock texts lose something in packing in 42, WS texts in 0. A control pair of plain standard calls
(`DG2YCB` / `W1XYZ`) sends identical texts and bits in both.

### 4.3 The receive side is unchanged

`Decoder/decodedtext.cpp` is identical in WSJT-X 3.0.2 and WS 3.1.0 (255 lines, no difference). The decision ladder of
`processMessage` (from "no grid on end of msg" to "treat like a CQ/QRZ") differs only by the FT2 additions (`m_mode.startsWith("FT")`
and the like); `auto_sequence` and `processMessage` in WS 3.2.1 add a de-duplication argument for forwarded Q65/JT65 decodes.

### 4.4 Case per announced pattern

| pattern | bits | content |
|---|---|---|
| non-standard / non-standard | case C: types 1 and 4 | reports now sent (hashed both calls); RR73/73 as type 4 instead of cut free text |
| compound / compound (two forms) | case C | tx2/tx3 carry the **base** call of the sender, suffix dropped |
| non-standard / compound | case C | as above for the compound station |
| non-standard / `/P` | case C | as above |
| standard with `/MM`, `/P` etc. | case C | as above |

Nothing here is case A (no unused code point is used) or case B in the bits. The dropped suffix is a convention at the application
level: the field holds a valid base call and a receiver cannot tell it from the station that really uses that call.

### 4.5 Between versions

WS 3.1.0 (260522) and 3.2.1: `genStdMsgs` (261 lines), `genCQMsg` and `elide_tx1_not_allowed` have **no difference**; the `processMessage`
ladder is identical; `packjt77.f90` is identical; `Radio.cpp` is identical; `auto_sequence` differs by the de-dupe argument. The extension did
not change between these two. Earlier builds (260226 / 260228, WS 3.0.0, 3.2.0) were not available.

## 5. What receivers do

### 5.1 Decoding (executed)

A small Fortran driver around each tree's `packjt77` (`docs/notes/ws77_harness/`) packed and unpacked the messages. With the
receiver's own call set (as `jt9` does) and empty tables, or after unpacking the sender's CQ first:

| message | empty tables | after the sender's CQ |
|---|---|---|
| DG123YCB: `<W250USA> <DG123YCB> -05` | `<W250USA> <...> -05` | `<W250USA> <DG123YCB> -05` |
| DG2YCB/QRP: `<W1XYZ/YOTA> DG2YCB -05` | `<W1XYZ/YOTA> DG2YCB -05` | `<W1XYZ/YOTA> DG2YCB -05` |
| DG2YCB/QRP: `W1XYZ/YOTA <DG2YCB/QRP> RR73` | `W1XYZ/YOTA <...> RR73` | `W1XYZ/YOTA <DG2YCB/QRP> RR73` |
| DG123YCB: `W1XYZ <DG123YCB> -05` | `W1XYZ <...> -05` | `W1XYZ <DG123YCB> -05` |
| DG123YCB: `<W1XYZ/P> DG123YCB RR73` | `<W1XYZ/P> DG123YCB RR73` | `<W1XYZ/P> DG123YCB RR73` |

(Reading: a station that knows the sender's full call, learnt from the type-4 tx1 or CQ, shows every call; a bystander shows `<...>` for the
hashed field. A call carrying `/P` hashes with the suffix, so `<W1XYZ/P>` resolves for the station whose own call it is.)

In v3.2.0-rc1 the same 168 messages unpack identically in both table states (168/168).

### 5.2 Auto Seq (desk-checked, not executed)

This is from the WSJT-X 3.0.2 / WS code (WS's receive logic is the same). rc1's `processMessage` (line 9134) was restructured and was not traced.

- `DecodedText::messageWords` splits a standard message with `tokens_re`: element 2 is the first word (the addressee), 3 the second
  (the sender), 4 the third (grid, report, `RRR`, `RR73`, `73`), and **4 is empty when the message has no third word**
  (`Decoder/decodedtext.cpp`).
- `auto_sequence` (3.0.2 `mainwindow.cpp:7496`) passes a decode to `processMessage` only if it is a standard message or contains
  `73`/`RR73`, and the first word contains my base call and the second contains the partner's base call (or I am calling CQ with
  auto-reply).
- The ladder (`:9111-9300`): a grid answers with tx2; `RRR`/`RR73`/`73` with tx5/tx6; a third word starting with `R` with tx4 (`:9253`; from REPORT on, and from REPLYING on in FT8, FT4, MSK144 and Q65); otherwise, since every state is `>= CALLING`, the third word is read as a number (`:9262-9283`) and **an absent third word
  is `""`, which reads as `0` and counts as a report**: tx3 (`ROGER_REPORT`), or tx4 for `R-nn`.

Trace of a stock station B (`W250USA`, stock texts) and a WS station A (`DG123YCB`, calling CQ), both auto-sequencing:

| slot | sends | what the other side does |
|---|---|---|
| 1 | A: `CQ DG123YCB` (type 4) | |
| 2 | B: `<DG123YCB> W250USA` (tx1, type 4, no third word) | A: absent third word reads as report 0, goes to tx3 |
| 3 | A: `<W250USA> <DG123YCB> R-05` (type 1) | B: third word `R-05`, B is past REPLYING in FT8, goes to tx4 |
| 4 | B: stock tx4 `DG123YCB W250USA RR73` packs as free text, sent as `DG123YCB W250` | A: not a standard message and no `73` word, so `auto_sequence` ignores it |
| 5, 6, ... | A repeats tx3; B answers each tx3 with the same cut tx4 | no side leaves its state; it ends when the operator stops it or the Tx watchdog (not examined) does |

Two stock stations with non-standard calls stall the same way from slot 4: the stock text is not a standard message. The stall comes from the
stock side's unsendable RR73 (or, when rc1 refuses the pack, from the lack of any message), not from a change in WS's receiver. A WS-to-WS
sequence was not simulated.

### 5.3 Hash collisions (arithmetic, not measured)

A hashed call resolves to whatever stored call has the same hash. WSJT-X 3.0.2 keeps up to 1000 22-bit entries (`MAXHASH`) and 4096 12-bit slots
(`calls12`). For a hash the receiver has not learnt: about 1000 / 2^22 = 0.024% that a 22-bit hash hits a stored call; about 11% that a 12-bit
type-4 hash hits when 500 distinct calls are stored (1 - (1 - 1/4096)^500). This is how type 4 and the hashed type-1 field work in WSJT-X itself;
WS uses them in more of the steps of a QSO (the 12-bit form in tx4/tx5, the 22-bit form in tx2/tx3) where stock WSJT-X sent cut text or no report.

## 6. `mfsk-core`

### 6.1 Result

`unpack77_with_hash` was run on all 168 messages, with the receiver's own call inserted into the table (empty table, and after
`unpack77_learn` of the sender's CQ), and compared with WSJT-X 3.0.2. No panic, no `None`. 156 of 168 (empty table) and 148 of 168 (after
the CQ) agree; **all 32 differences are the same**: where WSJT-X prints `<W1XYZ/P>`, this crate prints `<...>`.

Cause: `CallsignHashTable::insert` strips a trailing `/R` or `/P` before hashing (`mfsk-core/src/msg/hash_table.rs`, "Strip /R or /P suffix
for hashing", present since the initial commit and pinned by the test `strip_suffix`), and `register_callsigns` (`wsjt77.rs`) reads the call of a
type 1/2 message without its `/R`//`/P` flag. Upstream's `save_hash_call` strips only angle brackets, and `unpack77` appends `/R` or `/P`
before saving (`packjt77.f90:548-559`). The failure is a missing resolution (`<...>`), never a wrong call, but it hits exactly the
traffic of this report (a station with a `/P` call addressed by a hashed call) and also stock WSJT-X traffic to such a station.

### 6.2 Proposed patch (not applied)

`docs/notes/ws77_hash_suffix.patch`: `insert` keeps the suffix, the learner appends the flag, the test is renamed `keeps_suffix`.
Checked in an isolated worktree: the 168 messages then agree in both table states (336 of 336), `msg::` unit tests pass, and
`MFSK_REQUIRE_CORPUS=1 cargo test -p mfsk-core --features full,internal-testing --release` gives 1072 passed, 0 failed, 211 ignored.
The patch applies cleanly to `main` at `0a7fab35`. Tracked in #570.

### 6.3 Defensive handling

- A hashed call that cannot be resolved is already shown as `<...>` and never as text; keep that.
- Messages in this report never reach a code point the crate refuses: types 1 and 4 only; types 0/2, 0/7+ and i3 6-7 still return `None`.
- Applications that match decodes to a QSO partner see the base call in tx2/tx3 for a suffixed partner (14 of 24 report messages). The sample
  FSMs under `embedded-poc/` are samples, not the reference, and were not evaluated for this report beyond a quick run, which showed that
  they compare the partner by the exact call and so log the base call when the suffix is dropped. Nothing is concluded from that.
- Fixtures: `mfsk-core/tests/fixtures/ws_77bit_extension.tsv` (168 rows, with columns `c77`, `rx_fresh`, `rx_after_cq`). No test loads it yet; a test comparing
  `unpack77_with_hash` with those columns fails on 32 rows until the patch is applied. This investigation changes no crate code.

## 7. Other implementations

JTDX and MSHV were not examined.

## 8. Open points

1. The message texts were produced by re-implementing the conditions of `genStdMsgs`/`genCQMsg` from the source, not by running WS. A mistake
   in that transcription would be in the fixtures. Running WS's GUI (or a harness linking its C++) on the five patterns would settle it.
2. The Auto Seq trace (5.2) is a reading of the 3.0.2 code, and rc1 was not traced. The stall was not reproduced by running either program.
3. The introduction build of the extension (WS 3.1.0 beta, 2026-02-26) was not available; 3.1.0 (260522) and 3.2.1 are identical in all
   the places examined. WS 3.0.0 and 3.2.0 were not fetched.
4. The type table (3) is from WSJT-X 3.0.2, not rc1's restructured source.
5. A WS-to-WS sequence and a WS-to-JTDX/MSHV sequence were not examined.

## 9. Reproduction

`docs/notes/ws77_harness/pack_cli.f90` and `unpack_cli.f90` are drivers around a tree's `packjt77` module (they contain no WS or WSJT-X code).
Per tree, with the tree's own `lib/` files (`packjt.f90`, `chkcall.f90`, `deg2grid.f90`, `grid2deg.f90`, `fmtmsg.f90`, `pfx.f90`, `77bit/packjt77.f90`;
for WS also `77bit/mulmod64.c` and `mulmod64_interface.f90`; for rc1 also `77bit/packjt77_schema.f90` and `packjt77_grammar.f90`):

```sh
gfortran -std=legacy -w -c packjt.f90 chkcall.f90 deg2grid.f90 grid2deg.f90 fmtmsg.f90 packjt77.f90   # rc1: schema and grammar first
gfortran -std=legacy -w -o pack_cli pack_cli.f90 *.o ; gfortran -std=legacy -w -o unpack_cli unpack_cli.f90 *.o
echo '<W250USA> <DG123YCB> -05' | ./pack_cli DG123YCB        # i3 n3 c77 ok text
printf 'L <c77 of the CQ>\nU <c77>\n' | ./unpack_cli W250USA   # learn, then unpack, as the receiver W250USA
```

GPL v3 applies to the WS and WSJT-X sources used here (#569, #568). Fixture rows are bit strings generated by running those packers.
