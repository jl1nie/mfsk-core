# JTTY — what WSJT-X 3.2.0-rc1 ships, and how it was ported

JTTY is the one mode WSJT-X 3.2.0 adds. This note records how upstream
implements it, the design decisions that were settled first, and the phased
plan, so that the port did not start from a cold read of ~4 000 lines of
Fortran. Tracking issue: #477 (closed; P6, embedded, is a separate decision).
The port exists on host, phases P0-P5 (`mfsk_core::jtty`, the C ABI, Kotlin and
Swift). The sections up to "Implementation plan" are written in the tense they
were planned in; what each phase actually measured is in the "P0 results" ...
"P5 results" sections at the end of the file.

The upstream half (the reading sections) is a reading of the source, **not** a
measurement. Measurements are in the "P* results" sections: "P0 results"
records what upstream's own `sjtty` / `rjtty` produced when built and run
(2026-09-26) — measurements of upstream, not of this crate, and only for the
settings stated there — and "P1 results" onward compare this crate's own
output against them.

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
  and `hmod=1.0` differ. (This crate's `engine/dsp/gfsk.rs` could not host it as
  is: see "Reuse" and #482.)
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
3. **Sync surface** `s0(freq, start-offset)`: correlate the 13-symbol
   sync waveform (`gen_syncwave`) with the analytic signal at 12-sample
   (2 ms) start offsets, an 8192-point FFT per offset, smoothed over
   frequency by a 1-2-3-2-1 kernel. The offsets span only the window's
   **first quarter frame** (237 columns: `ntstep/12 + 1` with `ntstep = 2832`
   at 6 kHz), so consecutive windows search disjoint start times; only the
   audio, not the search, overlaps. A Bluestein/chirp path replaces the full FFT when the searched
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

## Reuse in this crate

| need | exists | notes |
|---|---|---|
| call28 | `msg::wsjt77::pack28` / `unpack28` | used (P1); `unpack28` made `pub(crate)`; needs upstream's `standard_call` filter on top (`chkcall` subset, no `/`, no leading `Q`, round trip) — `jtty::source::is_standard_call` |
| ARRL sections (86) | `msg::wsjt77::ARRL_SECTIONS` | used (P1), made `pub(crate)` |
| Maidenhead grid | `msg::wsjt77::pack_grid4` | **not reusable as is**: JTTY's GRID4 index is `((f1*18+f2)*10+d1)*10+d2`, domain 0‥32399 — a different mapping from the pack77 `g15` |
| GFSK synthesis | `engine::dsp::gfsk` | was **not reusable as is** (it sampled the pulse one sample early vs every upstream `gen_*wave.f90`, #482, fixed in #490); JTTY keeps its own `tx::synth_f32` / `synth_complex` (parallel chunked scan, complex output, any symbol length; `gfsk_pulse` is shared) |
| analytic signal | `engine::dsp::analytic` | for the receiver (P2) |
| subtraction | `engine::dsp::subtract` | FT8's; JTTY's works on a complex buffer at 6 kHz |
| tone-shift (`twkfreq`) | `engine::sync2d::freq_shift_cd0` or `engine::dsp::ddc` | not compared in detail |
| convolutional code | `fec::conv` (r=½ **K=32** Fano — WSPR) | different code and decoder; nothing shared |
| CRC-12 | `fec::qra::q65::crc12` | **same generator** x¹²+x¹¹+x³+x²+x+1 (= JTTY's `0x80F` with the leading term implicit), but that routine is LSB-first over 6-bit symbols; JTTY's is a bitwise MSB-first remainder over 46 bits (`jtty_tbcc_crc_valid`). Same polynomial, do not assume the same routine — `jtty::crc` |

New: TBCC encoder (P1), list-WAVA decoder + ladder, the source grammar (P1) and
text packer, the receive-state machine (candidates → assembly), continuous-TX
plumbing.

## Numbers that matter for a port

| quantity | value |
|---|---|
| frame | 59 symbols × 384 / 12 000 s = 1.888 s |
| window / step | 1.25 frames = 2.36 s (28 320 samples at 12 kHz) / a quarter frame = 0.472 s |
| sync surface | 237 start offsets per window, one 8192-point FFT each (a narrower chirp/Bluestein path when the band is narrow) |
| analytic signal | one `ana64a` (FFT of the whole window) per window — 80 % of its audio was in the previous window |
| peak-up (channel 0) | 11 frequency trials per candidate, each shifting the buffer (`twkfreq`) |
| trellis | 512 states × width 4 × 2^L branches × ⌈46/L⌉ blocks × 2 passes, per rung, ≤ 4 rungs per candidate |
| retro re-sweep | 3 extra `jtty_mdecode` calls (each rebuilding its sync surface) per signal subtracted in a step |

None of this is measured. It is a list of where upstream spends work, to be
turned into a profile that tunes the parallel paths of D4 (chunk sizes, minimum
lengths) and says which of them dominates.

## Design decisions

**D1 — placement.** Outside `Protocol` / `PROTOCOLS`, as MSK144 is (it has no
T/R slot). Feature `jtty`, added to `full`; host first. As built it is `jtty = []` in
`Cargo.toml`: the wire level needs no FFT (and no `std`), and only the receiver
does — `jtty::{dsp, rx, assemble}` are compiled under `fft-rustfft` or
`fft-extern`, the same split `msk144`'s receive modules make. No `Protocol`
ZST, so nothing in `protocol_invariants.rs`.

**D2 — receive API: incremental in, synchronous out.** The input has to be
incremental, and that follows from the mode: there is no slot to hand over,
and the receiver carries state between calls (the search window, recently
seen frames, messages under assembly, the past audio the retro sweep reaches
back into). Precedents for the shape: `engine::ft4_coarse::Ft4SavgBuilder::push`
/ `push_with_rows(audio, &mut dyn FnMut(..))`, and the `decode_block`
streaming path.

The *output* does not need a poll. Decoding happens inside `push`, on the
caller's thread, so results can be delivered there:

```rust
receiver.push(&samples, &mut |u: &MessageUpdate| { /* id, f_hz, start, text, complete */ });
```

That is the callback idiom `STREAMING.md` §4 chose on purpose (runtime-free,
CPU-bound, nothing to `await`), and it composes the same way. A poll is only
needed when decoding runs on another thread, which the core does not do;
upstream's `jtty_get_updates` is a poll because its GUI decodes on a worker
thread. `STREAMING.md` itself is about *output* delivery for a decode whose
whole slot is already in hand; it is not a template for the input side.

`MessageUpdate` mirrors upstream's `message_update`: a stable message id,
the **cumulative** text so far, frequency, start time, and a `complete` flag
(EOM seen). A Rust closure does not cross the C ABI, and a C function-pointer
callback is awkward to bind from Kotlin and Swift, so `mfsk-ffi` owns a queue
inside the receiver handle and exposes a poll over it — the poll lives at the
FFI edge only.

**D3 — no global state.** Upstream keeps the trellis plans and workspaces in
`save`d module variables and serialises the decode ladder with
`!$omp critical`. That is an artefact of how they are held, not of the
algorithm. Here: an immutable plan (`Sync`, shared) and a per-call workspace;
the receiver owns every piece of mutable state. This is the precondition for
D4, so it is not deferred.

**D4 — parallelism and iterators, from the first version.** Decided by the
maintainer (2026-09-26), replacing an earlier "designed in, measured before it
is kept": independent work uses rayon from the start, and the code is written
in iterator style (`fold` / `scan` / `zip` / `array::from_fn`, not index
loops), so the parallel form is a substitution, not a rewrite. The earlier
`par_iter` reverts on JT65/Q65/JT9/uvpacket/MSK144 said nothing about this
workload — the loop was not the bottleneck there and the regions were tiny —
and upstream's serialisation (`!$omp critical` over `save`d statics) is an
artefact of how it holds state, not of the algorithm.

- *Freely parallel* (with D3): the 237 sync-surface columns per window, the
  analytic signal per window, the decode ladder for different candidates, the
  retro re-sweeps, per-frame encoding, waveform synthesis (phase rate per
  sample, then a chunked scan for the phase).
- *Genuinely sequential*: a decoded signal is subtracted from the window and
  later candidates in the same pass are demodulated from the *residual*
  (`c1 = twkfreq(c0, …)` from the current `c0`); peak selection with masking,
  duplicate suppression and assembly also depend on order. Deciding a whole
  pass against one residual and subtracting afterwards in a fixed order is
  legitimate — upstream already runs up to two passes over the residual — but
  it is a different schedule, so results can differ from upstream in edge
  cases. Bit-exact parity is therefore claimed only at the decoder boundary
  (P2), not for whole-window output.

Rules: behind the existing `parallel` feature (rayon, `std`); a sequential
path always exists (embedded has no rayon) and is the same code with the
iterator swapped; results are collected in order and reductions have a fixed
shape, so **the output is bit-identical for any thread count** (a test, for
every parallel path); no global mutable state. Profiling still has a job —
choosing chunk sizes and minimum lengths (P1's frame encoding is
microseconds, so it splits only from four frames up) and finding which of the
suspects in "Numbers that matter" dominates — but it tunes the parallel path,
it does not gate it.

**D5 — what to match upstream on.** The frame decoder (sample → validated
32-bit word) is unambiguous and testable against fixtures; the assembly
layer is hand-tuned constants. Port the former exactly, the latter
verbatim first and deviate only with a measurement.

**D6 — frequency drift (satellite / Doppler).** JTTY is expected to matter
for satellite work. In the code read for this note, sync and peak-up assume
a **constant** frequency across the frame: peak-up fits a straight line to the
unwrapped phase of the 13 sync phasors, which yields a frequency *offset*, and
no drift term appears anywhere in the receiver. Whether that suffices for LEO
Doppler is unchecked. `sjtty`'s channel model takes Doppler spread and delay
(`fdop`, `delay`); whether it can model a linear drift was not checked. Since
the transmit side is ours, drift can be synthesised in Rust and decoded by
both this crate and upstream's `rjtty` — which separates "our port is weaker"
from "the mode/upstream receiver is". That comparison is a P2 exit
criterion, not a later nice-to-have.

## Implementation plan

Each phase is its own PR. Exit criteria are stated so a phase can be judged
done without re-arguing scope.

### P0 — oracle and fixtures (no library code) — done, see "P0 results"
- `scripts/build_jttysim.sh` builds upstream `sjtty` and `rjtty` (plus our
  `jtty_ladder_oracle` driver) from a **clean export of the tag** — the local
  `WSJT-X/` tree carries uncommitted edits under `lib/`, so it is only used as
  a git repo to export from. Upstream's own top-level CMake was not usable
  here (it requires Qt5 incl. WebSockets, Hamlib and PortAudio to configure);
  `scripts/jttysim/CMakeLists.txt` replaces it in the export and reuses
  upstream's `CMake/Sources.cmake`, so the compile units are still upstream's.
- `scripts/gen_jtty_vectors.sh` vendors the sample recording under
  `embedded-poc/assets/golden/jtty/`, records what `rjtty` decodes from it,
  generates the `sjtty` vector set (tone sequences + a few WAVs) and the
  ladder cases.
- `scripts/gen_jtty_sweep_wavs.sh` generates the AWGN / fading sweep corpus
  (gitignored) and scores `rjtty` on it (`UPSTREAM_RECALL.tsv`, committed).
- **Exit (met):** the expected decodes are recorded and every fixture is
  reproducible byte for byte from the scripts.

### P1 — wire level (`mfsk-core/src/jtty/`) — done, see "P1 results"
`source.rs` (atoms ⇄ 32-bit word ⇄ text, validity), `crc.rs` (CRC-12, a
bitwise MSB-first remainder — not `fec::qra::q65::crc12`), `tbcc.rs` (encoder
and trellis step), `tx.rs` (frames → tones → GFSK, with its own synthesiser —
see "P1 results"). `pack28` and the ARRL section table are reused (made
`pub(crate)`).
- Tests (tier A/B): the spec's ten 34-bit vectors; every `sjtty` tone sequence
  in the manifest (15 messages); rendering against what `rjtty` prints;
  waveform against `sjtty`'s noiseless output; atom round trips for every kind;
  a rejection test per invalid class in the grammar; serial ⇄ parallel and
  any-thread-count identity.
- **Exit (met):** vectors, `sjtty` tones and `sjtty` waveform pass; no receiver
  yet.

### P2 — frame decoder, single signal — done, see "P2 results"
`correlate.rs` (symbol correlation), `trellis.rs` (list-WAVA, one `Plan` per
coherent length, immutable and shared), `ladder.rs` (the four rungs, evaluated
concurrently under `parallel`), `dsp.rs` (analytic signal, sync wave, frequency
shift), `rx.rs` (sync surface, candidates, peak-up, gate, decode, validate;
`Receiver::decode_window` and `scan`). No subtraction, retro sweep or assembly.
- Equivalence at the decoder boundary: `ladder_cases.txt`'s `ZS`/`ZH`
  (correlations generated by `jtty_ladder_oracle`, not extracted from signals)
  through the Rust list decoder and ladder; **the same top-4 words in the same
  order, CRC flags, start states, pool sizes, accepted rung and rank, metrics to
  1e-9**. Correlations extracted from the real recording's frames would add a
  check on the correlation stage; the frame-for-frame agreement below covers it
  in effect.
- Tier B: the sample recording frame for frame against `rjtty`; the seven
  `sjtty` vectors exactly and only them.
- Precision: Gaussian noise decodes nothing (60 files, 30 min, as `rjtty`);
  real JTTY-free audio decodes exactly the one false decode `rjtty` makes, and
  the sample recording's extra frame is the one `rjtty` makes — **`max_extra` is
  1, documented debt, not the target 0** (see "P2 results" for why it is not
  simply gated away).
- D6 drift comparison against `rjtty` (`scripts/jtty_drift_study.sh`).
- **Exit (met):** golden passes with the documented extra; decoder-boundary
  equivalence exact; recall identical to `rjtty`'s in all 18 cells of the sweep;
  the drift comparison is written up. An engineering checkpoint on whether the
  port is faithful — not a decision about whether JTTY is wanted.

### P3 — multi-signal — done, see "P3 results"
`subtract.rs` (a decoded frame off the 6 kHz analytic signal), `assemble.rs`
(`Assembler`, `MessageUpdate`), and in `rx.rs` the pass structure of
`jtty_mdecode`: channel 0 twice if it subtracted, channels 1 and 2 twice if
they did; the sticky-sync retry; the retro re-sweep of the three windows before
each subtracted signal; `Receiver::scan_messages`. Parallel as D4 says: a pass's
candidates are decoded at once and *settled in order*, failed ones re-decoded on
the residual while a round subtracts; a batch of windows has its analytic signal
and sync surface prepared ahead on the pool.
- Tests: the subtraction against a plain `cos²` convolution and against a direct
  port of upstream's FFT form; seven `sjtty` mixtures (one noise, calibrated
  component SNRs) against `rjtty`, a station in every channel among them; a scene
  in which a weak station needs the subtraction (off: lost); thread-count identity
  for frames and messages; the assembler's rules one by one.
- Two random two-station studies against `rjtty`
  (`scripts/jtty_multi_study.sh`, easy and hard).
- Wiring the JTTY sweeps into `run-sensitivity-sweeps.sh` and `sweep-baseline.json`
  was deferred from here and done in P4a (see "P4a results").
- **Exit (met):** the multi-station fixtures decode as `rjtty`'s; a station under
  a strong one is recovered; the differences from upstream's schedule are
  measured (2 of 200 in the hard set, none in the easy).

### P4 — host packaging
`JttyReceiver` (D2), `MessageUpdate`. FFI: a new receiver-handle function
family in `mfsk-ffi` with a queue and poll, a new `MfskMode` value (25 is the
next free one), `mfsk.h` regenerated (committed), the C++ smoke driver
extended, Kotlin and Swift bindings. This is more than MSK144's "one more
mode" because the ABI shape is a stateful handle, not a slot call.
- Docs: `docs/reference/{LIBRARY,BINDINGS}.md` and their `.ja.md` twins, the
  mode tables in README, CLAUDE.md's map; CHANGELOG. There is no separate
  benchmark note: the sweep is `mfsk-core/tests/jtty_sweep.rs` (`jtty_snr_sweep`,
  run by `scripts/run-sensitivity-sweeps.sh jtty`) and its baseline lives in
  `docs/notes/sweep-baseline.json` (see "P4a results").
- CI: a `jtty` row in the `feature-matrix` (`ci.yml`) and in
  `scripts/pre-push-check.sh`; path filters for `src/jtty/**` and `tests/jtty_*`
  beside the `msk144` ones.
- **Exit:** the C++ driver and both bindings receive a WAV fed in chunks and
  report the golden message.

### P5 — text packer (protocol half only)
`pack_jtty`'s dynamic program and its exchange profiles: text in, atoms out, and a
transmit path through the ABI and the bindings. **The F-key templates, N1MM
`[[JTTY:<ACTION>]]` tags and choosing the profile from the operating activity are
host UI policy and stay out of the library** — the line #463 draws for the
QSO-state FT8 decoders, agreed for JTTY on 2026-09-26. The profile is an argument.
- **Exit (met):** the same frames as upstream's `pack_jtty` on a few thousand messages
  under all three profiles, the same refusals, the ABI and both bindings sending text
  and receiving it back.

### P6 — embedded (separate decision)
After P2/P3 are measured on a host: whether a continuously running receiver
fits the CoreS3's budget, which FFT sizes, and a fixed-point path if so.

### Versioning and cadence
A new mode is patch-level by this crate's convention (MSK144 shipped as
`0.7.4`). Merge phases to `main` as they complete; do not tag for them —
releases stay on the biweekly cadence.

## Risks

- **Upstream is a release candidate.** The grammar already changed once with
  no discriminator. Pin to the tag; re-diff on the final 3.2.0 before
  publishing anything public.
- **Hand-tuned constants** (sync gate, dedup and continuation tolerances)
  have no derivation upstream; the fixtures are the only guard.
- **Single real recording.** Upstream ships one WAV; everything else is
  `sjtty` synthetic. Noiseless synthetic fixtures are not an instrument for
  sensitivity — the sweeps must use noise (a deterministic seed), as for the
  other modes.
- **f32 vs f64.** Upstream correlates in single precision and scores in
  double; tie-breaking depends on it. Keep the same split.
- **False decodes on real audio.** Upstream's receiver makes them (P2 results) and so does the faithful port; how to trade them against recall is open.
- **The oracle is upstream's rc.** Its behaviour, including the false decode below, is the reference only where it is right; P2 states where this crate deliberately differs.
- **Upstream's sensitivity claim is unverified here.** The release notes say
  "far better weak-signal performance" than RTTY; `lib/jtty/wava/` holds the
  simulator, an RCU bound and a curves image, none of which was re-run. P2/P3
  measure this crate against `rjtty` rather than against the claim.

## P0 results (2026-09-26)

Measured with `sjtty` / `rjtty` built by `scripts/build_jttysim.sh` from
`v3.2.0-rc1`. These describe upstream, not this crate.

- **Build.** ~10 s on 24 cores; the wrapper CMake needs `gfortran`, `cmake`,
  `libfftw3-dev` and `libboost-dev`, not Qt. Two upstream oddities had to be
  worked around: sources say `include '/lib/fftw3.f90'` (an absolute path; it
  resolves because gfortran also tries it against each `-I` directory, so the
  source root goes on the include path), and the Reed-Solomon C sources are
  left out as irrelevant to JTTY.
- **The sample recording.** `rjtty 4.6 1 384 1500 50` decodes
  `RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!` (10 frames, 1506–1508 Hz,
  dt 0.30 s), matching the user guide's screenshot — **and one extra line**,
  `4>-P'` at 1695.6 Hz on channel 2 with `nsync` 9 and 27 symbol errors in 59.
  That looks like a false accept, on a recording whose message is "no false
  decodes". Not investigated (whether the GUI filters it; whether it is the
  receiver or the `rjtty` front end). The golden test expects the message only.
- **Determinism.** `sjtty` gives identical bytes for identical arguments (its
  noise is seeded); `rjtty` is deterministic. `rjtty` truncates long path names
  (~80 characters): run it in the WAV's directory.
- **Sensitivity of upstream's receiver** (`CQ K1ABC CQ`, one frame, 20 files per
  cell, `sjtty` SNR convention, defaults above): AWGN 0/20 at −20 dB, 1/20 at
  −18, 6/20 at −17, **11/20 at −16**, 17/20 at −15, 20/20 from −14. ITU
  `mid_moderate` (0.5 Hz spread, 1 ms): 7/20 at −16, 11/20 at −15, and it
  **plateaus at 16/20 from −13 to −12** — 20 % lost even at high SNR, cause not
  investigated. No unexpected decodes in any of the 360 short files.
- **Ladder cases.** 33 generated cases cover every ladder outcome (L=1: 6,
  L=2: 8, L=4: 2, half-symbol rung: 10, all rungs fail: 7 — of which some at
  Es/N0 = 0 dB decode at L=4 and some at 10 dB with a half-symbol rotation
  decode at L=2, so the rungs are not simply ordered by SNR).

## P1 results (2026-09-26)

- **Encoder agrees with upstream.** All ten 34-bit vectors from
  `jtty_source_encoding.txt` encode and decode exactly, and the tone sequences
  of all 15 `sjtty` manifest messages are reproduced bit for bit — on the first
  run, including hand-chosen TEXT5 splits. The messages are turned into atoms
  by hand in the test; upstream's text packer is P5.
- **Rendering agrees** with what `rjtty` prints for the five vectors that have
  a WAV.
- **`sjtty`'s SNR is in a 2500 Hz reference bandwidth** (`sjtty.f90`:
  `sig = sqrt(2·2500/6000)·10^(snr/20)` against unit-variance noise); at
  SNR > 90 it writes the noiseless waveform, peak-normalised to 32766.9 — the
  synthesiser oracle used here. (This settles the "convention unchecked" in the
  P0 results: the sweep's dB values are 2500 Hz SNRs.)
- **A bug in the shared GFSK synthesiser (#482).** `engine::dsp::gfsk` samples
  the Gaussian pulse one sample early relative to *every* upstream
  `gen_*wave.f90` (they loop 1-based). Against `sjtty`'s noiseless waveform it
  showed as a uniform worst normalised sample error of 4.9e-2 — exactly
  2π·Δf/fs for JTTY's largest tone step — and one token (`i + 1`) brought it to
  1.1e-4. It is a shared-code change with reach into FT8/FT4/FST4 transmit and
  subtraction references, so it was its own issue, fixed since in #490; JTTY
  keeps its own synthesiser (`jtty::tx::synth_f32`), which is also the parallel
  chunked-scan design of D4 and produces the complex reference of P3. Against `sjtty` it is within 6e-4 (one frame) and
  1e-3 (two frames): that residual is upstream's single-precision phase
  accumulation, which grows with length; ours is `f64`.
- **`rjtty` quirk at dt = 0.** On the noiseless two-frame vector at dt 0.0 it
  emitted the call, then the whole message, a duplicate from the retro
  re-sweep, and finally `599 123` as a separate message — a transmission that
  starts at t = 0 is mis-assembled. The vectors use dt 0.3.
- **rayon from the start.** Frame encoding and waveform synthesis run on the
  pool under `parallel`; the output is bit-identical for 1, 2, 3 and 8 threads
  and to a plain sequential running sum (1e-6).

## P2 results (2026-09-26)

Measured with this crate's `Receiver` against WSJT-X `v3.2.0-rc1`'s `rjtty`
(defaults: 1500 Hz ± 50 Hz, `smin` 4.6, band 200–2800 Hz), on the fixtures and
corpora of P0.

- **The trellis and ladder agree with upstream exactly** on all 33 generated
  cases × four rungs (132 lists): words, order, CRC flags, start states, pool
  sizes, metrics (≤ 1e-9), and the accepted rung and rank in each case. It ran
  correct on its first run.
- **Frames agree.** On upstream's sample recording all ten frames of
  `RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!` come out at the same
  frequency (to 0.05 Hz), the same time (to 1 ms) and with the same cumulative
  text as `rjtty`'s ten lines; `nsync` and the symbol-error counts (0, 0, 0, 7,
  2, …) are equal too. The seven `sjtty` vectors decode to exactly their message
  and nothing else.
- **Recall is identical, cell for cell.** All 18 cells of the AWGN and ITU
  mid-moderate sweep (360 files) score the same as `rjtty`, with zero unexpected
  decodes: AWGN 50 % at −16 dB, fading plateauing at 16/20 (still unexplained).
  Of the 160 correct decodes, 115 are accepted at L=1, 28 at L=2 and 17 at L=4
  (none needed the half-symbol rung); 131 are rank 1, 13 rank 2, 14 rank 3, 2
  rank 4.
- **False decodes.** Gaussian noise: none, in 60 files (30 min), for either
  decoder. Real audio: the 25 vendored recordings with no JTTY in them (FT8, FT4,
  FST4, WSPR, JT9, JT65, MSK144, Q65; 1485 s) give **one** false decode, in a
  WSPR recording (`KD7NCT`, 1475.5 Hz, channel 0, 8 of 13 sync tones, 21 symbol
  errors, rung 2, rank 3) — and the sample recording's own extra (`4>-P'`,
  channel 2, 9 of 13, 27 errors, rung 2, rank 2). Both are made by `rjtty` too:
  this is upstream's false-accept behaviour, reproduced. Two events in ~26 min of
  real audio is far too few to state a rate; it is enough to say the rate is not
  zero. (An earlier draft added "and Gaussian noise is a poor proxy for it";
  that was not measured. What #487 measured is that the rate follows the number
  of candidates that pass the sync gate, times 16/4096 for the CRC and the
  fraction the grammar accepts, so a noise-only set bounds the rate but says
  nothing until its gated-candidate count is stated.)
- **Why a simple gate does not remove them.** Correct decodes near threshold
  carry up to 23–26 symbol errors in 59 (AWGN −16…−18 dB, fading −16 dB) against
  21 and 27 for the false ones, so an error-count limit cuts real decodes about
  as readily; and 29 of the 160 correct decodes are rank > 1, so accepting only
  the maximum-likelihood word (the false ones are ranks 2 and 3) would cost 18 %
  of the decodes at the edge. The budget is the CRC-12 itself: up to 16 CRC
  trials per candidate, ≈ 0.4 % before the grammar check, which passes any TEXT5
  word. The default stays faithful; what to trade is #487, not a guess
  made here.
- **D6: drift.** `scripts/jtty_drift_study.sh` — one frame, centred on 1500 Hz,
  linear drift, deterministic noise, 20 files per cell. **This crate and `rjtty`
  score the same in all 22 cells**, and both fall off a cliff:

  | drift (Hz/s) | 0 | 5 | 10 | 12 | 14 | 16 | 18 | 20+ |
  |---|---|---|---|---|---|---|---|---|
  | −8 dB | 20 | 20 | 20 | 20 | 19 | 11 | 6 | 0 |
  | −12 dB | 20 | 20 | 18 | 8 | 1 | 2 | 0 | 0 |

  Over one 1.9 s frame 16 Hz/s is 30 Hz, about one tone spacing (31.25 Hz).
  Upstream's receiver has no drift term, so this is a property of the mode's
  reference receiver, not of the port; a satellite operator needs the Doppler
  tracked out before the audio arrives (rig control), or a drift-aware receiver,
  which would be a deliberate extension beyond parity (#488).
- **Speed** (release, 30.2 s of audio, 60 windows, one window per quarter frame):
  11.4 ms per window on one thread (2.4 % of a core in real time), 6.2 ms on 2,
  3.0 ms on 8, 2.0 ms on the default 24-thread pool (247× real time). The output
  is bit-identical for 1, 2, 5 and 16 threads. Not profiled beyond that.

## P3 results (2026-09-26)

- **A bug found by comparing, not by a test.** The first version of
  `subtract_frame` used a single complex exponential for the `cos²` window's
  cosine part; `camp` is complex, so the `sin` part does not cancel unless the
  gain is constant, and the unit tests (constant gain) passed. Comparing the
  residual with a direct port of upstream's FFT form showed a 20 % difference;
  with both signs of the exponential the two agree to 2e-15. A test against the
  plain convolution on a varying signal now pins it.
- **Mixtures, identical to `rjtty`.** Seven recordings, several stations in one
  noise at calibrated SNRs (`sim/mix_*.wav`, `MIXES.tsv`): two stations 60 Hz
  apart, a weak one under a strong one, one in each of the three channels, four
  stations, a fading pair, back to back on one frequency, overlapping on one
  frequency. This crate and `rjtty` assemble the same messages in all seven.
  (`rjtty` at `ndebug 1` also prints frames it absorbed as repeats — counting
  those credited it with decoding a false frame that the GUI never shows; the
  comparisons use `ndebug 0`.)
- **A weak station needs the subtraction.** A strong station and one 20 dB down
  and 20 Hz away, starting 67 ms later: with `Params::subtract` off only the
  strong one is decoded, with it on both, and `rjtty` decodes both.
- **Random two-station recordings** (`scripts/jtty_multi_study.sh`; station A
  `CQ K1ABC CQ`, B `CQ W9XYZ CQ`, one noise, deterministic):
  - *easy* — B 25–120 Hz away, within 1.5 s, −14…−6 dB: **100 of 100 recordings
    identical**; A 100/100 for both, B 69/100 for both, no file differs.
  - *hard* — B 12–50 Hz away, within 0.3 s, −18…−10 dB, 200 recordings: B recovered
    **101 (this crate) against 103 (`rjtty`)**, A 200/200 for both; two files where
    `rjtty` decodes B and this crate does not (both marginal: 10 of 13 sync tones,
    19 symbol errors in 59 in the one examined), none the other way. Before the
    failed candidates were re-decoded on the residual it was 100 against 103.
  That is the cost of the schedule (D4): about 1 % of the weak stations in the
  hardest case, none where the stations are not almost on top of each other.
- **Speed** (release, the 30.2 s sample, 60 windows, retro re-sweeps and
  subtraction included): 11.7 ms per window on one thread (43× real time),
  6.2 ms on 8 threads (81×), 8.3 ms on the default 24-thread pool. The parallel
  gain is modest (1.4–1.9×) because windows are taken in order and the tasks
  inside one are small. By manual timers (`perf` does not run here): the ladder
  on candidates that pass the gate is about 45 % (~6 ms each on one thread; 50 of
  891 candidates reach it), sync-surface builds about 30 % (104 builds of 2.4 ms:
  one per window, one per subtraction, and one per retro window), the rest
  candidate selection and gating. Caching FFT plans per thread cut an analytic
  signal from 0.67 ms to 0.05 ms; building a planner per rayon split had been a
  large share of a window's cost. A retro window cannot use a prepared surface
  (the interferer changes the signal), which is where a signal-rich recording
  spends its time.
- **Not ported / different:** upstream coalesces pending updates between polls
  (`queue_message_update`); here every merge emits an update and the consumer
  coalesces — the streaming API is P4a. `Params::subtract` is new (off = a
  single-signal receiver).

## P4b results (2026-09-26)

The C ABI, the Kotlin and Swift bindings, the C++ driver.

- **ABI.** `MfskMode` 25 = JTTY (`MFSK_MODE_JTTY`), described as one frame
  (`t_slot_s` = the 1.888 s frame period, `slot_samples_12k` 22 656). New
  capability bit `MFSK_CAP_STREAM_RECEIVER` (1 << 15): the mode is received by a
  handle, not a slot decode, and `MFSK_CAP_DECODE_HANDLE` stays clear. Functions:
  `mfsk_jtty_{params_init, open, close, set_params, push_i16, push_f32, finish,
  reset, pending, poll}`; `MfskJttyParams` and `MfskJttyUpdate` are size-versioned.
  `mfsk_abi_version` stays 2: the change is additive.
- **Poll at the edge, callback inside** (D2). The Rust `Stream::push` takes a
  callback; the handle turns that into a queue. The queue **coalesces per message
  id** — a message that grew twice between polls comes out once with its latest
  text, which is what upstream's `jtty_get_updates` does and what bounds the queue
  (1024 messages; a caller that never polls loses the oldest). `poll` returns
  1 / 0 / negative `MfskStatus`, not an enum, because "nothing waiting" is not an error.
- **Feature.** `mfsk-ffi/jtty` (in `desktop` and `mobile`, implies `protocols` for
  the FFT). The entry points are always in the header (it is generated from one
  source, whatever the features); without the feature they answer
  `MFSK_STATUS_UNKNOWN_PROTOCOL`, as `mfsk_runtime_configure` does without `parallel`.
- **No transmit call.** The ABI has no JTTY synthesis, so every binding test feeds the
  vendored upstream recording (the transmit path and the packer came in P5).
- **Tests.** `tests/jtty_ffi.rs` (7: introspection, chunk-size independence, coalescing,
  finish/reset, 24 kHz + float, parameter validation, NULL handling), the C++ driver's
  `test_jtty`, the Kotlin JVM test and `JttyReceiverTests` (Swift, 5). The recording gives
  two messages, as in `tests/jtty_rx.rs`: the sentence, and upstream's known false
  decode `4>-P'` on channel 2 (the Rust and C++ tests see both; the Kotlin and Swift
  ones look only for the sentence).

## P4a results (2026-09-26)

The core API half of P4: the streaming receiver, its docs and the tier-C wiring.
The FFI and bindings (P4b) are separate.

- **`jtty::rx::Stream`** (D2 as designed): `push(&[i16], &mut dyn FnMut(MessageUpdate))`,
  `finish`, `reset`, `set_params`, `samples_seen`, `buffered_samples`. It decodes
  every window that a push completes, on the caller's thread and rayon's pool, and
  trims its buffer to what the retro re-sweep can still reach (three windows before
  the next one). `Receiver::run` was refactored into a `step` over an `Audio` view
  (`buf` plus the index of `buf[0]`), used by both the one-shot scan and the stream,
  so there is one schedule and parity is by construction. The test checks it anyway:
  chunk sizes 1, 333, 4096, 12000, 28320, 100000 and the whole recording give the
  updates `scan_messages` gives, and the buffer stays within `NCHUNK + 3 * STEP` plus
  the chunk.
- **Tier C.** `tests/jtty_sweep.rs::jtty_snr_sweep` scores the `jtty_sweep` corpus
  (awgn and mid_moderate, -20…-12 dB, 20 files a cell) through `Receiver::scan` into
  `channel,snr_db,trial,pass,extra`; `run-sensitivity-sweeps.sh jtty` runs it in 7 s.
  Baseline: 50 % crossing **-16.20 dB** (awgn) and **-15.25 dB** (mid_moderate),
  **0** unexpected decodes in 360 files. `sweep-baseline.json` stamps `jtty` on its own;
  `release-status.sh` watches `src/jtty`. The parity test against `UPSTREAM_RECALL.tsv`
  stays in `jtty_rx.rs`.
- **Docs.** `LIBRARY.md` §2.5 (with a doctest that synthesises, streams and decodes a
  message), the module map and the feature table, both languages; README mode table.

## P5 results (2026-09-26)

- **Packer.** `jtty::pack` (`normalize`, `pack`, `tones`, `ExchangeProfile`, `PackError`),
  a port of `pack_jtty`, `normalize_jtty_message`, `normalize_serials`, `try_compact`,
  `consider` and `offer`. No FFT, so it builds under `alloc,jtty`.
- **Oracle.** `scripts/jttysim/jtty_pack_oracle.f90` (ours, linked against upstream's
  `jtty_mod`) reads `profile<TAB>text` and prints `nframes` and the frames as hex;
  `scripts/gen_jtty_pack_cases.sh` builds 3 525 cases (curated ones, seeded random
  compositions of the tokens the packer treats specially, random text) and vendors
  the answers as `golden/jtty/pack_cases.tsv` (270 KB). `tests/jtty_pack.rs` requires the
  identical frames, or the identical refusal. Frame counts in the corpus run 0 to 16.
- **What the oracle found.** One upstream quirk, in the *first* run: `1F DX` came out as
  TEXT5 upstream and as a class/section atom here. `pack77_arrl_section_index` returns
  -1 for an argument shorter than three characters and `pack_jtty` hands it `trim(word)`,
  so a two-letter section (`DX`, `AB`, `MB`, `PE`, …) is never packed as a class/section
  atom; a longer word matches on its first three characters (which `offer`'s
  round-trip then rejects). Reproduced and documented in `pack.rs`. Whether it is
  intended is not knowable from the source; the receiver still decodes such an atom.
- **Deliberate differences.** More than 80 characters is `PackError::TooLong` (upstream's
  `character*80` truncates silently at the caller); a non-ASCII character is one `#`, not
  one per byte. Neither is reachable from an ASCII message of at most 80 characters.
- **ABI.** `mfsk_jtty_encode_tones(text, profile, tones, cap, out_len)`,
  `mfsk_jtty_synth_len`, `mfsk_jtty_tones_to_i16` / `_f32`; a NULL buffer with
  capacity 0 is a size query. Kotlin `MfskJtty`, Swift `Jtty`.

## E0 results: what a receive window costs on the CoreS3 (#499, 2026-09-26)

`embedded-poc/embedded-shared/src/apps/jtty_bench.rs`, `--bin jtty-bench --features jtty` in
`m5stack-cores3-app`; ESP32-S3 at 240 MHz, 80 MHz quad PSRAM, synthetic input, the esp-dsp backend
through `default_planner()`. The estimates in #499 were 2–3 ms for an 8192-point FFT, about 0.5 s for
a sync surface and 0.15–0.3 s for the ladder; the measurements are worse on the two that matter.

**FFT, complex f32, one transform (µs):**

| N | buffer in internal DRAM | buffer in PSRAM |
|---|---|---|
| 256 | 107 | 106 |
| 512 | 229 | 229 |
| 1024 | 494 | 494 |
| 2048 | 1 059 | 1 059 |
| 4096 | 2 264 | 2 379 |
| 8192 | **4 819** | **67 146** |

The PSRAM penalty is nil up to 4096 points (a 32 KB buffer, the size of the data cache) and **14×** at 8192
(64 KB). Unlike the FT4 `cd0` case (1.12× where 5–10× had been projected), the projection holds here. The
FFT belongs in internal DRAM, and the receiver's 8192-point transform needs a 64 KB block of it.

**One sync-surface column** (multiply the 2496-sample sync waveform in, zero-pad, FFT, power over the band,
1-2-3-2-1 smoothing), times 237 columns:

| | per column | per surface (237) |
|---|---|---|
| 8192-point, buffers in internal DRAM | 5.30 ms | **1 257 ms** |
| 8192-point, buffers in PSRAM | 73.1 ms | 17 330 ms |
| decimated by 16 (375 Hz), 156-sample waveform, 256-point, channel 0 (68 bins) | 0.149 ms | **35 ms** |
| the same, channels 1 and 2 (205 bins) | 0.182 ms | **43 ms** |

The receiver searches three bands per window. As it stands that is 3 × 1.26 s at best against a 0.472 s
window, 8× over with every buffer in internal DRAM and 110× over with the buffers where the allocator puts
64 KB. Decimated, the three bands are about 120 ms, a quarter of the window, and the placement no longer
matters (the working set is 2 KB). The same input on the host runs the whole window in 11.7 ms.

**The ladder** (`Ladder::decode`, synthetic correlations of a real frame, four runs each):

| input | mean | worst | outcome |
|---|---|---|---|
| +12 dB | 854 ms | 856 ms | rung 1 ×4 |
| +6 dB | 852 ms | 855 ms | rung 1 ×4 |
| +3 dB | 1 003 ms | 1 445 ms | rung 1 ×3, rung 2 |
| 0 dB | 2 315 ms | 2 892 ms | rung 3, none, rung 2, none |
| −3 dB | 2 889 ms | 2 893 ms | none ×4 |
| noise only | 2 665 ms | 2 884 ms | none ×3, one false accept (rung 3) |

A rung costs about 0.72–0.85 s, so a candidate that fails all four takes 2.9 s, and even the easiest success
takes 1.8 windows. `Ladder::new` takes 116 ms once and holds **98 KB of internal DRAM and 272 KB of PSRAM**.
This is the dominant cost and it is not the FFT. The trellis keeps every metric in `f64`
(`energies: Vec<f64>`, `wava`, `clean_metric`, the tie-break comparisons), which is software on the LX7. That
is the likely reason and is **not yet measured**: an `f32` (or integer) trellis is the experiment. The metrics
are `f64` because upstream's are and ties between equal-metric words are broken on the last bits, so an
embedded ladder would trade exact agreement with upstream on rare ties for speed, as the embedded FT8 path
already does; it has to be measured against the 33 ladder cases and the sweep, not assumed.

**Verdict.** Receive as the host does it does not fit, by a factor of 8 on the search and of 2–6 on the ladder.
The search is fixed by decimating (E1a: recall against the reference on `jtty_sweep`). The ladder needs
roughly a tenfold speed-up to leave room for the search inside 0.472 s, and nothing here shows that is
reachable: E1b (an `f32` trellis, on the host first for agreement, then on the device) decides it. Until then
receive is *not* shown to be feasible. Memory: the ladder's internal DRAM (98 KB) and a 64 KB FFT block
compete with what the WiFi and USB host leave (about 31 KB of largest block after they start), so both would
be allocated at boot in JTTY mode from the arena, and the ladder's small allocations would want to go to PSRAM
if the trellis tolerates it (measure).

Transmit is unaffected by any of this (E2, E3).

## E1b results: the trellis metrics in `f32` (#499, 2026-09-26)

E0 blamed the ladder's `f64` metrics (software on the LX7) "likely, not measured". Measured now, and the
hypothesis was mostly wrong.

**On the host, `f32` is the same decoder.** `Plan::decode_f32`, `Ladder::with_f32_metrics`,
`Receiver::with_f32_metrics` carry the path metrics in `f32` (keys and words stay integers).
- Against upstream's 33 ladder cases (132 lists): every list identical in words, order, CRC flag, start
  state and pool; the worst clean metric off by 3.4e-7 relative; the ladder accepts the same word at the
  same rung in 33 of 33 (`tests/jtty_ladder.rs`).
- Against `f64` on synthetic frames (300 per SNR from +9 to −4 dB and 2 000 of noise, 12 300 lists): one list
  differs (−2 dB, a near-tie), **0 of 4 100 frames accept a different word or rung**, false accepts equal.
- Receiver end to end: the same frames from every simulated vector, the mixtures and the sample recording
  (`tests/jtty_rx.rs`); the 360-file `jtty_sweep` corpus gives a **byte-identical per-trial CSV** with
  `MFSK_JTTY_SWEEP_F32=1`.

**On the CoreS3 it is 1.3×, not 10×.** One L=1 rung, ms per decode:

| trellis and survivor arrays in | `f64` metrics | `f32` metrics |
|---|---|---|
| PSRAM (allocations over 2 KB, the board's setting) | 845 | 635 |
| internal DRAM (`heap_caps_malloc_extmem_enable(256 KB)` for the run) | 611 | 414 |

The full ladder with the board's setting: 651 ms for a rung-1 success (was 856), 2.1 s for a candidate that
fails all four rungs (was 2.9 s). So `f32` is worth 1.3×, placement another 1.5×, together 2.0×; the
survivor arrays (two of 32 KB) and the larger plan tables are past the 32 KB data cache in PSRAM, the effect
E0 found for the FFT but smaller here. **414 ms for the fastest rung is still about 5× the budget** (a
candidate has well under 0.47 s of a window, and a weak one needs up to four rungs). Per path extension
that is about 1.1 µs, 260 cycles, for `377 000` of them (46 blocks × 2 wraps × 512 states × 2 predecessors ×
4 paths): the loop is compute-bound and inefficient on this core in ways `f32` does not touch. Suspects, not
measured: 64-bit keys (`u64` word|origin|valid, compared and OR-ed for every extension), 16-byte survivor
copies in `insert`, iterator-adapter bounds checks, unpredictable branches. Fixed point is not among them
(the LX7 has a single-precision FPU; i16 BP measured 0.85× `f32` on this board).

**Where this leaves receive.** Search: fixable, 35–43 ms per band decimated (E0). Ladder: `f32` and internal
placement give 2×; the remaining 5× has to come from the code (split the key, a specialised L=1 loop, no
per-decode allocation with a workspace reserved in internal DRAM at boot, fewer paths or one wrap where
recall allows) or from running candidates on the second core. None of that is measured, so receive is still
not shown feasible. The next step is profiling the trellis on the device (cycle counter around `advance`,
`insert`, the pool and the re-score) before changing anything: the last two guesses in this note (the FFT
estimate and `f64`) were both wrong by measuring.

