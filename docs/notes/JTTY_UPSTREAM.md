# JTTY — what WSJT-X 3.2.0-rc1 ships, and what porting it would take

JTTY is the one mode WSJT-X 3.2.0 adds that this crate has no counterpart
for. This note records how upstream implements it, so that the port (and the
decision of whether to make it) does not start from a cold read of ~4 000
lines of Fortran again. Tracking issue: #477. It is a reading of the source, **not** a measurement:
no JTTY signal has been generated or decoded with this crate, and none of the
sensitivity figures upstream might quote are reproduced here.

Read against: `wsjtx` tag `v3.2.0-rc1` (`567ad29ce6abf3d4a44f181cdbc7ceba0d73e5f4`,
2026-09-23), `lib/jtty/`, `Modulator/Jtty*`, `widgets/mainwindow_jtty.cpp`,
`widgets/Jtty*.hpp`. The upstream files are named per section; line numbers
are omitted because the mode is new and still moving between release
candidates — re-verify against the tag before quoting one.

Not read in full: `widgets/mainwindow_jtty.cpp` (GUI plumbing, ALL.TXT
logging), `TxAudioQueue` / TCI transport, `Audio/Fixture*`, the
`tests/unit/jtty/` bodies (only their names and `CMakeLists.txt`), and
`lib/jtty/wava/*` beyond its headers.

## What it is

A **non-slotted** mode: a transmission may start at any time and is one or
more self-contained 1.888 s frames; the receiver runs continuously. That is
the property that does not fit the crate's `Protocol` / `FrameLayout` model
(slot length, `TX_START_OFFSET_S`, fixed T/R period), for the same reason
MSK144 sits outside it. Everything else is conventional MFSK-with-FEC and
reuses existing parts (see "Reuse").

## Frame and waveform

| item | value | source |
|---|---|---|
| payload | 34 bit = 32 bit source word + bit 33 (reserved, always 0) + bit 34 (EOM) | `jtty_source_encoding.txt` |
| outer check | CRC-12, polynomial `0x80F` (leading term implicit), over the 34 bits → 46 bit | `jtty_tbcc_code_profile.f90` |
| FEC | tail-biting convolutional, R=1/2, K=10 (512 states), generators octal `1167` / `1545`, 46 info bit → 92 coded bit | same |
| symbol map | 2 coded bits → 1 tone, Gray: `00/01/11/10 → 0/1/2/3` (`tone = 2*b0 + (b0 xor b1)`) | `tbcc.f90`, `jtty_tbcc_list_decoder.f90` |
| sync | 13 symbols, `[0,2,2,3,0,0,3,2,1,3,1,2,0]` (peak ACF sidelobe 2/13) | `jtty_fec_mod.f90` |
| frame | 13 sync + 46 data = 59 symbols; `nsps=384` at 12 kHz → 31.25 baud → 1.888 s | `genjtty.f90` |
| modulation | 4-GFSK, tone spacing = baud (h=1), BT = 2.0, envelope ramp `nsps/8` | `gen_jttywave.f90`, `mainwindow_jtty.cpp` |
| bandwidth | ≈127 Hz (99 % power) — the older `jtty_design.txt` says 125 Hz half-power | `jtty_design.md` |

- The code is **tail-biting**, so there is no tail: the encoder's initial
  state is the message's own last 9 bits (`tbcc_encode`).
- `gen_jttywave` is the same construction as `gen_ft8wave`/`gen_fst4wave`
  (frequency-pulse superposition with dummy first/last symbols); only `bt`
  and `hmod=1.0` differ. `engine/dsp/gfsk.rs` is the natural host for it.
- A message is 1‥16 frames (`MAX_FRAMES=16`); EOM is set on the last atom
  only; a second queued native message adds no spacing frame.
- `rjtty_core` accepts `nsps` of 240 / 320 / 384 / 480 (50 / 37.5 / 31.25 /
  25 baud) but every GUI call site hard-codes 384. Treat 384 as the mode;
  the others are latent.

## Source grammar (the 32-bit word)

Normative text: `lib/jtty/jtty_source_encoding.txt` (kept upstream as the
spec, with 34-bit golden vectors — e.g. `CQ K1ABC CQ` = `0x026F78D41`,
`TEXT5 HELLO` = `0x11395558D`). The word is MSB-first; bits 31-32 are `i2`.

| `i2` | contents |
|---|---|
| 0 (n2 = 0..3) | call28 + action: `CQ <call> CQ`, `<call>`, `TU <call> CQ`, `<call> TU` |
| 1 (n2 = 0..1) | `<call> AGN?`, `TU NOW <call>`; n2 = 2,3 reserved/invalid |
| 2 | STRUCT30: 27-bit body + 3-bit family |
| 3 | TEXT5: five 6-bit characters, alphabet `0-9A-Z +-./?!"#$%,&*()_'=[]{}<>|:;` |

STRUCT30 families: `000` EXCH_NUM (role, kind, 17-bit value; 7 kinds:
serial, CQ/ITU zone, age, power, check, licence year, generic), `001`
EXCH_LOC (role, kind, 2–3 base-36 chars), `010` EXCH_PAIR (zone+loc3, Field
Day class/section), `011` EXCH_NUM_TIME, `100` MISC (18 control phrases,
GRID4), `101`/`110` reserved, `111` guard (always invalid).

- The call field is the **existing `pack28`/`unpack28`** with a round-trip
  requirement (`jtty_standard_call`): standard calls only, no `/`, no
  leading `Q`, alphanumerics only.
- Class/section indexes upstream's `PACK77_ARRL_SECTIONS` (86 entries) —
  the table already in `msg/wsjt77.rs` (`ARRL_SECTIONS`, NSEC=86).
- `599` is a role bit, never on the wire; digit widths, separators and
  "5NN" spelling are not encoded. Rendering is canonical (serial ≥ 3
  digits, zone/check ≥ 2, time exactly 4).
- **Validity is part of decoding**: after FEC + CRC the word must also
  satisfy bit 33 = 0, no unassigned type/family/subtype/enum, in-range
  fields, and not be all-zero. An invalid word is dropped before display,
  EOM handling, slot creation **and signal subtraction**. This is the
  phantom-decode guard (the crate's CRC-only false-accept history — #243,
  #253 — is the reason to keep it a hard requirement in a port).
- The former `i2=2` meaning (`599 ` + 5 chars) was replaced with no version
  discriminator; upstream calls JTTY "unreleased" so there is no legacy
  mode. Do not implement the old form.

### Transmit-side packing (`pack_jtty`, `jtty_mod.f90`)

Text is upper-cased, space-normalised, unsupported characters → `#`.
Then a dynamic program over character offsets picks the **minimum-frame**
sequence of candidate atoms and TEXT5 (an interior TEXT5 takes exactly 5
characters; ties prefer structured atom, longest span, kind/subtype/role
order). Every candidate must round-trip through pack → unpack → render and
match its source span exactly. Candidates: six call forms, control phrases,
GENERIC_NUMERIC, `599 <2–3 char with a letter>` as GENERIC_QTH, GRID4,
CLASS_SECTION. An **exchange profile** (Unknown / Field Day / RTTY Roundup)
is captured from the operating activity at submit time; only RTTY Roundup
adds SERIAL and STATE_PROVINCE candidates and canonicalises `599 05` →
`599 005`. Result ≤ 80 characters, else rejected, never truncated.

The eight F-key templates (F1 `CQ %M CQ` … F8 `%E`) compile to typed atoms
*before* placeholder expansion; a customised template falls back to text
packing; a recognised template with bad runtime data is rejected, never
demoted to literal. N1MM drives it with a leading `[[JTTY:<ACTION>]]` tag
(11 actions), spec in `jtty_n1mm_integration.md`. **None of the N1MM /
F-key / profile layer is protocol** — it is host UI policy — and a
library port can stop at "atoms in, tones out" plus the text packer.

## Receiver

Pipeline, in the order `rjtty_core → jtty_mdecode_step → jtty_mdecode`:

1. **Analytic signal** at 6 kHz (`ana64a`: real 12 kHz → FFT, zero negative
   frequencies, inverse half-size FFT). `engine/dsp/analytic.rs` exists.
2. **Windowing**: chunk = 1.25 frames (`nframe + nframe/4`); the start
   advances by a quarter frame (0.472 s) per step; a step is attempted once
   a whole chunk is buffered. Every window sees a fresh candidate search.
3. **Sync surface** `s0(freq, quarter-frame-step)`: correlate the 13-symbol
   sync waveform (`gen_syncwave`) with the analytic signal at 12-sample time
   steps, an 8192-point FFT per step, smoothed over frequency by a 1-2-3-2-1
   kernel. A Bluestein/chirp path replaces the full FFT when the searched
   band is narrow (`build_s0`); it is an optimisation, not a different
   algorithm.
4. **Candidates**: three channels — 0 = the operator's RX frequency ±`ftol`,
   1 and 2 = 1350 / 1650 Hz ±150 Hz (clamped to the waterfall). Channel 0
   takes up to `clamp(fwid/(nfz*df2), 2, 8)` peaks; 1 and 2 take 2 each.
   After each pick, a ±10 Hz × ±16 ms neighbourhood is masked. Channel 0 runs
   to completion (two passes if it subtracted something) before 1 and 2, and
   its successful neighbourhoods are erased from the shared surface so 1/2
   do not rediscover them. At most 100 candidates per call.
5. **Peak-up** (channel 0 only, `jtty_peakup`): re-search ±4 ms in time and
   ±2.5 Hz in 0.5 Hz steps by *moving* the 13-symbol correlation
   incrementally (4-sample hop), then fit a line to the unwrapped phase of
   the 13 per-symbol phasors to remove the sub-0.5 Hz residual — accepted
   only if the fit residual RMS < 1 rad and |Δf| ≤ 0.5 Hz.
6. **Gate**: shift the candidate to DC (`twkfreq`), take the 4 tone
   correlations per sync symbol, count hard-decision sync matches
   (`nsync`) and estimate S/N from the sync-tone power vs. the other three.
   Channel 0: `nsync > 6` and S/N ≥ `smin` (4.6 dB). Channels 1/2:
   `nsync > 8` and S/N ≥ 5.0 dB.
7. **Payload correlation** (`jtty_correlate_payload_symbols`): per data
   symbol, the complex correlation with each of the 4 tone references
   (full symbol → `zsym`) and the two half-symbol energies combined as
   `sqrt(E1+E2)` (→ `zhalf`).
8. **Decode ladder** (`jtty_tbcc_decode`): rungs L = 1, 2, 4 (coherent
   block length in symbols) on `zsym`; if all fail, L = 1 on `zhalf` (energy
   only, "half-symbol observation"). First rung with an accepted hypothesis
   wins. Details below.
9. **Validity + merge**: `unpack_jtty` (grammar check above), then
   duplicate suppression, subtraction, and assembly (below).
10. **Sticky retry**: if a call decoded nothing, and an active message's
    continuation is due almost exactly one frame period after its last
    frame (±0.1 s, within the channel's band), retry the FEC decode at that
    remembered sync point instead of searching — one attempt per channel.

### The list decoder (`jtty_tbcc_list_decoder.f90`, `jtty_tbcc_decoder.f90`)

- **Branch metric** for a block of L symbols and a hypothesised tone
  sequence `t_1..t_L`: `|Σ z[t_i, symbol_i]|² / L`. L=1 is per-symbol
  non-coherent energy; L=2/4 sum the complex correlations first, i.e. assume
  the carrier phase is constant over L symbols. It is an energy, not an LLR:
  no noise variance or amplitude estimate is needed. (Our inference: the
  coherent sum is valid because tone spacing = baud makes each tone's phase
  advance a multiple of 2π per symbol — `jtty_block_pow.f90` says exactly
  this about `ctones`. GFSK smoothing perturbs it slightly.) Blocks tile the
  46 bits from bit 1; L=4 gives 11 blocks of 4 + 1 of 2.
- **List WAVA**: all 512 states start alive at metric 0, origin = itself.
  Two circular passes (`wraps=2`); at the start of each pass every
  survivor's path bits and origin are reset to its current state, metrics
  carry over. Each state keeps the best **4** paths (`per_state_width`),
  ordered by metric then by 46-bit word. Predecessors of a destination state
  are enumerated structurally (the low L bits of the state *are* the input
  word), so no branch tables are needed for correctness — upstream builds
  them anyway for speed.
- **Reserved-bit pruning**: with `prune_reserved_zero`, branches that set bit
  33 are dropped inside the trellis, before any CRC.
- **Pool**: after the last pass, only survivors whose final state equals
  their origin (closed tail-biting paths) are kept; deduplicated by the full
  46-bit word through an open-addressing hash (capacity ≥ 2 × 512 × 4),
  keeping the best WAVA metric per word.
- **Re-scoring**: each pooled word is re-scored with a clean single-pass
  metric (start state = its own last 9 bits; must close), because the WAVA
  metric contains history from the earlier pass. Sorted by clean metric
  (ties: word, start state, WAVA metric); top **4** exported ("H4").
- **Admission** (caller): first exported hypothesis with a valid CRC-12; an
  all-zero payload stops the scan of that rung ("do not expand the fixed
  false-accept budget past the all-zero sentinel"). So at most 4 CRC trials
  per rung, ≤ 16 per attempt — roughly 16/4096 ≈ 0.4 % before the reserved
  bit / grammar checks (our estimate, not upstream's).
- The `_reference` routine is the plain implementation; the optimised one
  packs a path into an `int64` key (46 bit word | 11 bit origin | valid bit).
  Upstream keeps both and tests them against each other.
- Global `!$omp critical` around the whole ladder and `save`d plans:
  upstream decodes one candidate at a time. A port should not inherit that.

### Subtraction, deduplication, assembly (`subtract_jtty`, `decode_and_merge`)

- **Subtract**: re-encode the *decoded* payload to tones (sync + 46), build a
  unit-amplitude reference with `gen_jttywave` (BT 2.0) at the candidate's
  f/t, estimate the complex gain by low-pass filtering `c0·conj(ref)` with a
  `cos²` window of 2 symbols (FFT-domain), subtract `gain·ref`. Structurally
  `subtractft8` with the analytic buffer already complex, so no ×2 / real
  part step. Only after the source-grammar check.
- **Retro re-sweep** (`jtty_mdecode_step`): for each signal subtracted in a
  step, re-run the candidate search on the preceding 3 quarter-frame windows
  with that signal pre-subtracted (windows whose 1.25-frame span reaches
  into its energy but whose own search range did not include it). Not
  cascaded.
- **Duplicates**: same text within 32 ms; channel 1/2 candidates matching a
  channel-0 success within 3 Hz / 50 ms; frames seen recently (12 Hz /
  50 ms) are pure duplicates and are neither shown nor assembled.
- **Assembly**: ≤ 30 active messages. A new frame continues an active
  message if it lands `n` frame periods later (n = 1‥3, ±0.1 s) and within
  10 + 3(n−1) Hz; a gap (n > 1) inserts `~~~~~` (rendered ` ... `). Frames
  that are windows re-decoding the same transmission (quarter-frame offsets
  within 10 Hz) are absorbed. Messages with no continuation within 3 frame
  periods are flushed as incomplete. TEXT5 spaces are shown as `~`; structured
  atoms add an implicit separator. Display is capped at 80 characters.
  EOM (`is_last_frame && all_valid`) closes the message.

## Numbers that matter for a port

| quantity | value |
|---|---|
| frame | 59 symbols × 384 / 12 000 s = 1.888 s |
| search step | quarter frame = 0.472 s; window 1.25 frames = 2.36 s |
| sync FFT | 8192 point at 6 kHz (0.73 Hz bins) |
| trellis | 512 states × width 4 × 2^L branches × ⌈46/L⌉ blocks × 2 passes |
| one decode attempt | ≤ 4 rungs; each rung is a full list-WAVA over 46 symbols |

The trellis work per rung is of the order 10⁵–10⁶ metric additions (our
arithmetic, unmeasured). A host build is not the question; whether a
continuously-running receiver fits the CoreS3 budget (which is already
contested by FT8/FT4/FST4/WSPR) is, and it is **out of scope for the first
port**.

## Reuse in this crate

| need | exists | notes |
|---|---|---|
| call28 | `msg::wsjt77::pack28` / `unpack28` | needs the same round-trip filter as `jtty_standard_call` |
| ARRL sections (86) | `msg::wsjt77::ARRL_SECTIONS` | private const today |
| Maidenhead grid | `msg::wsjt77::pack_grid4` | **not reusable as is**: JTTY's GRID4 index is `((f1*18+f2)*10+d1)*10+d2`, domain 0‥32399 — a different mapping from the pack77 `g15` |
| GFSK synthesis | `engine::dsp::gfsk` | BT 2.0, h = 1, ramp `nsps/8` |
| analytic signal | `engine::dsp::analytic` | |
| subtraction | `engine::dsp::subtract` | FT8's; JTTY's works on a complex buffer at 6 kHz |
| tone-shift (`twkfreq`) | `engine::sync2d::freq_shift_cd0` or `engine::dsp::ddc` | not compared in detail |
| convolutional code | `fec::conv` (r=½ **K=32** Fano — WSPR) | different code and decoder; nothing shared |
| CRC-12 | `fec::qra::q65::crc12` | **same generator** x¹²+x¹¹+x³+x²+x+1 (= JTTY's `0x80F` with the leading term implicit), but that routine is LSB-first over 6-bit symbols; JTTY's is a bitwise MSB-first remainder over 46 bits (`jtty_tbcc_crc_valid`). Same polynomial, do not assume the same routine |

New: TBCC encoder, list-WAVA decoder + ladder, the source grammar and text
packer, the receive-state machine (candidates → assembly), continuous-TX
plumbing.

## Open questions for the port

1. **API shape.** MSK144 precedent says "outside `Protocol`, not in
   `PROTOCOLS`", but a continuous receiver needs a stateful streaming API
   (feed samples, poll updates — cf. `jtty_get_updates`), not
   `DecodeRequest::decode()`. `docs/reference/STREAMING.md` is the closest
   existing shape.
2. **Whose behaviour is the reference?** Upstream's assembly / dedup /
   retro-sweep constants are hand-tuned and undocumented beyond comments.
   A first target should be the *frame* decoder (sample → validated 32-bit
   word), for which the golden fixtures are unambiguous, and the assembly
   layer second.
3. **Fixtures.** Upstream ships one recording, `samples/JTTY/260807_134110.wav`
   (725 992 bytes), plus `sjtty` (simulator) and `tests/fixtures/jtty/`.
   The tier-B golden would be that recording; tier-C corpora need an
   `sjtty`-based generator (`sjtty` is built from `lib/jtty/sjtty.f90`).
   Decide `max_extra` for the golden up front, as `assert_golden` requires.
4. **Sensitivity.** Upstream's `lib/jtty/wava/` holds a Monte-Carlo TBCC
   simulator (AWGN / Rayleigh), an RCU bound and `tbcc_performance_curves.png`.
   None of it was re-run here. The release-notes claim ("far better weak-signal
   performance than RTTY") is unverified by this crate.
5. **Stability.** The mode is new in an rc; the grammar was already changed
   once without a discriminator. Pin the port to a tag and re-diff on the
   final 3.2.0 before publishing anything public.

## Suggested phasing

1. Wire-level: source codec (atoms ⇄ 32-bit word ⇄ text), 34-bit payload,
   CRC-12, TBCC encoder, waveform synth. Verify against the spec's golden
   vectors and by decoding our own synthesis (tier A).
2. Frame decoder: sync search, payload correlation, ladder + list-WAVA,
   grammar validity. Tier B on `260807_134110.wav`.
3. Multi-signal: subtraction, retro sweep, duplicate suppression, assembly.
   Its precision guard (false decodes under subtraction) ships in the same PR.
4. Host packaging: streaming API, FFI (`mfsk-ffi-abi` row types), bindings.
5. Text packer / macro layer, only if a consumer wants it in the library.
6. Embedded: separate decision after measuring phase 2 on a host.
