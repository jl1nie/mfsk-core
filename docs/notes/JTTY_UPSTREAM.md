# JTTY — what WSJT-X 3.2.0-rc1 ships, and what porting it would take

JTTY is the one mode WSJT-X 3.2.0 adds that this crate has no counterpart
for. This note records how upstream implements it, the design decisions to
settle first, and a phased plan, so that the port does not start from a cold
read of ~4 000 lines of Fortran. Tracking issue: #477.

The upstream half is a reading of the source, **not** a measurement: no JTTY
signal has been generated or decoded with this crate, and none of the
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
turned into a profile before anything is optimised or parallelised (see D4).

## Design decisions

**D1 — placement.** Outside `Protocol` / `PROTOCOLS`, as MSK144 is (it has no
T/R slot). Feature `jtty`, gated on a host FFT exactly as `msk144` is
(`Cargo.toml`), added to `full`; host first (`std`). No `Protocol` ZST, so
nothing in `protocol_invariants.rs`.

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

**D4 — parallelism: designed in, measured before it is kept.**
Two facts pull in opposite directions.

- *What is freely parallel.* With D3 the decode ladder for different
  candidates is independent work. The 237 columns of the sync surface are
  independent of each other. Neither is limited by the language or by
  upstream's locking.
- *What is genuinely sequential.* A decoded signal is subtracted from the
  window, and later candidates in the same pass are demodulated from the
  *residual* (`c1 = twkfreq(c0, …)` is taken from the current `c0`). Peak
  selection with masking, duplicate suppression and assembly also depend on
  order. Decoding a whole pass against one residual and applying subtractions
  afterwards in a fixed order is legitimate — upstream already runs up to two
  passes over the residual — but it is a different schedule, so results can
  differ from upstream in edge cases. Bit-exact parity is therefore claimed
  only at the decoder boundary (see P2), not for whole-window output.

Rules: the result must not depend on the thread count (with a test); a
single-threaded path always exists (embedded has no rayon); it sits behind
the existing `parallel` feature. Whether it *pays* is
an open measurement: the earlier candidate-loop `par_iter` experiments on
JT65/Q65/JT9/uvpacket/MSK144 gave nothing because that loop was not the
bottleneck there and the regions were tiny (≤ 7 items) — that says nothing
about JTTY, whose ladder is 4 list-WAVA runs per failing candidate and whose
sync surface is 237 FFTs per window. Profile first, in this order of suspicion
(all unmeasured): the sync surface, the analytic signal per window, peak-up,
the ladder on failing candidates, retro sweeps.

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

### P0 — oracle and fixtures (no library code)
- Build upstream `sjtty` and `rjtty`. Prefer the upstream CMake targets over a
  hand-written compile list like `scripts/build_ft4sim.sh`'s: these need
  FFTW and a much larger set of compile units than the FT4 simulator (not
  attempted yet). Wrap it in `scripts/build_jttysim.sh`, from a **clean
  checkout of the tag** — the local `WSJT-X/` tree carries uncommitted edits
  under `lib/`.
- Decode `samples/JTTY/260807_134110.wav` with `rjtty` (its defaults are
  `f0=1500`, `ftol=50`, `nsps=384`, band 200–2800 Hz, `smin=4.6`); cross-check
  the text against the user guide's `jtty.png`. Vendor the WAV under
  `embedded-poc/assets/golden/jtty/` and add its README row.
- Generate a small vector set: messages → `sjtty` WAV → `rjtty` output, plus
  the 34-bit vectors from the spec, and a small AWGN set at a few SNRs for
  the P2 sensitivity comparison. Check whether `sjtty`'s noise is seeded; if
  not, add deterministic noise so a run is reproducible.
- An **instrumented driver** for P2's decoder-boundary test: a small Fortran
  program linking upstream's `jtty_*` modules that writes out `zsym`/`zhalf`
  and the ladder's accepted word for each candidate. `rjtty` does not expose
  these, so this is new code on the oracle side (kept out of the library).
- **Exit:** the expected decodes are recorded, reproducible from a script.

### P1 — wire level (`mfsk-core/src/jtty/`)
`source.rs` (atoms ⇄ 32-bit word ⇄ text, validity), `crc.rs` (CRC-12, a
bitwise MSB-first remainder — not `fec::qra::q65::crc12`), `tbcc.rs` (encoder),
`tx.rs` (frames → tones → GFSK via `engine::dsp::gfsk`). `pack28` and the ARRL
section table are reused (`ARRL_SECTIONS` is private today).
- Tests (tier A): the spec's 34-bit vectors; encoder ⇄ `sjtty` tone/waveform
  agreement with noise off or at a very high SNR, if `sjtty` allows it
  (unchecked; this tests determinism, not sensitivity); round trip of
  every atom kind; rejection of every invalid class in the grammar.
- **Exit:** vectors and `sjtty` agreement pass; no receiver yet.

### P2 — frame decoder, single signal
`sync.rs` (sync surface, peak-up, gate), `correlate.rs`, `list_decoder.rs`
(a plain reference first, then the optimised one, kept and tested against
each other as upstream does), `ladder.rs`, validity.
- Equivalence at the decoder boundary: dump upstream's `zsym`/`zhalf` for
  the fixtures, feed them to the Rust ladder, require the **same top-4 words in the
  same order, with metrics equal to f64 tolerance** (f64 accumulation, as
  upstream). This isolates the trellis
  from DSP differences.
- Tier B: `assert_golden` on the sample WAV, `max_extra: 0` as the target.
- Precision guard: a noise-only recording set must decode nothing;
  false-accept counted against the ≈0.4 % CRC budget in this note.
- D6 drift comparison against `rjtty`.
- **Exit:** golden passes; decoder-boundary equivalence exact; sensitivity on
  a small `sjtty` AWGN set within noise of `rjtty`; the drift comparison is
  written up. This is an engineering checkpoint on whether the port is
  faithful — not a decision about whether JTTY is wanted.

### P3 — multi-signal
Subtraction (on the 6 kHz analytic buffer), retro sweep, duplicate
suppression, assembly, and the parallel paths of D4 if the profile justifies
them. Its precision guard ships in the same PR: both false-decode bugs this
suite has shipped were in subtraction paths (#243, #253).
- Tier C: `scripts/gen_jtty_sweep_wavs.sh` on `sjtty`, a sweep test, an entry
  in `sweep-baseline.json` including the unexpected-decode count; `run-sensitivity-sweeps.sh`
  wired.
- Thread-count independence test if D4's parallel path lands.
- **Exit:** multi-signal fixtures (two overlapping signals, one fading)
  decode both; the unexpected-decode count is recorded in the baseline, and a
  later rise of ≥ 3 and ≥ 1.5× is flagged as for ft8/ft4/fst4.

### P4 — host packaging
`JttyReceiver` (D2), `MessageUpdate`. FFI: a new receiver-handle function
family in `mfsk-ffi` with a queue and poll, a new `MfskMode` value (25 is the
next free one), `mfsk.h` regenerated (committed), the C++ smoke driver
extended, Kotlin and Swift bindings. This is more than MSK144's "one more
mode" because the ABI shape is a stateful handle, not a slot call.
- Docs: `docs/reference/{LIBRARY,BINDINGS}.md` and their `.ja.md` twins, the
  mode tables in README, CLAUDE.md's map; CHANGELOG. `docs/notes/JTTY_BENCHMARK.md`
  from the P3 sweeps.
- CI: a `jtty` row in the `feature-matrix` (`ci.yml`) and in
  `scripts/pre-push-check.sh`; path filters for `src/jtty/**` and `tests/jtty_*`
  beside the `msk144` ones.
- **Exit:** the C++ driver and both bindings receive a WAV fed in chunks and
  report the golden message.

### P5 — text packer and macro layer (only on request)
`pack_jtty`'s dynamic program, exchange profiles, F-key / N1MM compilation.
Host UI policy upstream, not protocol.

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
- **Build of the oracle** (P0) is untried and may be the first real cost.
- **Upstream's sensitivity claim is unverified here.** The release notes say
  "far better weak-signal performance" than RTTY; `lib/jtty/wava/` holds the
  simulator, an RCU bound and a curves image, none of which was re-run. P2/P3
  measure this crate against `rjtty` rather than against the claim.
