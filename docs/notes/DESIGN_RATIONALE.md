# Design rationale

Why `mfsk-core` is built the way it is. Part I is the design itself: what the crate is for, the constraints that
follow, and how each of the principles in `CLAUDE.md` §2 comes out of them. Part II keeps the few later decisions
that are still load-bearing and that someone would otherwise make again.

The reference manuals (`LIBRARY.md`, `EMBEDDED.md`, `BINDINGS.md`) say *what* is true; this says *why*. It is not a
history: what changed and when, with the measurements of the day, is in `CHANGELOG.md` and `docs/historical/`, and
this file links there rather than retelling it.

## Contents

**Part I — the design**

- [1. What the crate is for](#1-what-the-crate-is-for)
- [2. WSJT-X is the reference](#2-wsjt-x-is-the-reference)
- [3. One source, many targets: the `Protocol` trait](#3-one-source-many-targets-the-protocol-trait)
- [4. The decode API is `jt9`'s shape](#4-the-decode-api-is-jt9s-shape)
- [5. A wide public surface, in three tiers](#5-a-wide-public-surface-in-three-tiers)
- [6. Wide-band is the main path](#6-wide-band-is-the-main-path)
- [7. Measure, and never recall without precision](#7-measure-and-never-recall-without-precision)
- [8. Why Rust](#8-why-rust)

**Part II — decisions still load-bearing**

- [9. Why the embedded path doesn't widen PASS1 or enable OSD](#9-why-the-embedded-path-doesnt-widen-pass1-or-enable-osd)
- [10. What is deliberately not ported](#10-what-is-deliberately-not-ported)

---

# Part I — the design

## 1. What the crate is for

WSJT-X is the reference implementation of FT8, FT4, FST4, FST4W, WSPR, JT9, JT65, Q65 and MSK144, and the source of
every protocol constant here. It is also a Fortran / C / Qt desktop application, and that makes its decoders hard to
run anywhere else:

- in a browser (`wasm32`), where there is no Fortran runtime;
- on Android and iOS, where there is none to link either;
- inside a headless application — a skimmer, a monitoring station, a remote SDR front end, a `wsprd`-style daemon —
  that wants a library, not a process to drive;
- on a microcontroller (`no_std + alloc`: ESP32-S3, RP2350), with no OS, a few hundred kilobytes of RAM and, on the
  shipped boards, fixed-point arithmetic;
- as the core of a new protocol experiment that wants FT8's LDPC and sync machinery without forking a desktop app.

The crate exists to make the decoders and synthesisers available in all of those places **without changing what they
decode**. Two constraints follow, and the rest of Part I is how the design meets both at once:

1. **Results must be WSJT-X's.** A user moving from WSJT-X, or comparing against it, must get the same decodes, the
   same SNR and the same files. A decoder that is "about as good" is a different decoder (§2).
2. **One body of code must serve every target.** Five ports, one per platform, would drift apart within a release;
   the fidelity in (1) would then hold on at most one of them (§3).

## 2. WSJT-X is the reference

**The port is 1:1, not an improvement.** The value of the crate is that its output can be checked against WSJT-X and
found equal. An "improved" decoder cannot be checked: every difference would have to be argued as better or worse,
and a user could no longer tell a regression from a feature. So the algorithm is ported as it is, constants and quirks
included, and a test compares against the real program (`jt9`, `wsprd`) on the same input.

**Every algorithm file names what it ports.** A `Ported from lib/…f90:line` note sits at the top of each algorithm
file. Changing a decoder internal means reading the Fortran, making the matching change and keeping that pointer
valid; the pointer is what lets the next reader, or the next upstream release, be compared line for line.

**A divergence is allowed, and must be visible.** Some differences are deliberate: an option WSJT-X lacks, a bug that
upstream has and this crate does not copy, a platform limit. Each is marked in a comment at the code, and in the API
they are kept apart by type (everything upstream has is in `DecodeParams`, everything this crate adds is in a mode's
`Extras`; §4). Where both behaviours are useful, the faithful one is selectable: `WsprExtras::wsprd()` reproduces
`wsprd`'s output byte for byte, including the places where the crate's default path deliberately differs
(`WSPRD_PARITY.md`).

**The reference moves with upstream's development line.** The reference is v3.2.0-rc1 plus the results-changing parts
of v3.3.0-beta1 to beta6, tracked on `release/3.3`, never `master`. Upstream changes are classified A (alters
results), B (real bug fix) or C (refactor); A and B are ported, C is not (`CLAUDE.md` §5, `scripts/upstream-diff.sh`).
Following a moving reference is the price of being comparable to the WSJT-X people actually run.

## 3. One source, many targets: the `Protocol` trait

### What is shared and what is not

The protocols differ in their FEC (LDPC, convolutional + Fano, Reed-Solomon, QRA over GF(64)), their message width
(77, 72, 50 bits) and their frame (tones, symbol rate, sync layout, slot length). They share most of the receive and
transmit machinery around that: coarse and fine sync, symbol spectra and LLRs, equalisation, subtraction, GFSK
synthesis, resampling, the hash table, the 77-bit message codec. WSJT-X expresses that commonality by copying code
between per-mode source files. Here it is traits, split by what varies:

- **Per protocol**, as `const` associated items on a zero-sized type: tone count, symbol rate, Gray map, GFSK shaping
  (`ModulationParams`); total symbols, sync / data layout, slot length (`FrameLayout`); which FEC (`Protocol::Fec`)
  and which message codec (`Protocol::Msg`).
- **Shared**, generic over `P: Protocol`: `engine` (sync, LLR, equalisation, the pipeline driver, TX synthesis, DSP),
  `fec`, `msg`.

```text
     ┌──────────────────────────────────────────────────────────────┐
     │  ft8  ft4  fst4  fst4w  wspr  jt9  jt65  q65                 │  per-protocol ZSTs
     │       (each implements Protocol + FrameLayout)               │  (feature-gated)
     └────────────┬──────────────────────┬──────────────────────────┘
                  │                      │
         ┌────────▼─────────┐    ┌───────▼────────┐
         │       msg        │    │      fec       │  shared codecs
         │ Wsjt77 · Jt72    │    │ LDPC · RS      │  behind traits
         │ Wspr50 · Q65     │    │ ConvFano · QRA │
         │ · hash table     │    │                │
         └────────┬─────────┘    └───────┬────────┘
                  │                      │
              ┌───▼──────────────────────▼───┐
              │            engine            │  Protocol trait, DSP
              │  sync · llr · equalize ·     │  (resample / GFSK /
              │  pipeline · tx · dsp         │   downsample / subtract)
              └──────────────────────────────┘
```

Measured (`wc -l` over `mfsk-core/src`, test modules included, `uvpacket` excluded, at `f0e8dcc6`, 2026-11-18): of
108.0k lines, the shared modules `engine`, `fec` and `msg` are 47.2k (44 %), and the ten protocol directories 50.3k
(47 %); the rest is the public API (`decoder`, `encoder`, `iq`, `slotgrid`, `registry`). The per-protocol share is
protocol-specific by necessity (each mode's search, its subtraction, its own corners of upstream); the split of what
each protocol takes from the shared part is `LIBRARY.md` §3.1.

### Zero-cost: monomorphised, not dispatched

Because the per-protocol part is `const` items on zero-sized types, the generic receive path — `engine::sync` →
`engine::llr` → `engine::equalize` → `engine::pipeline`, free functions generic over `P` — is monomorphised per
protocol. The compiler sees a specialised function for each `P`, inlines the constants and autovectorises the hot
loops on native targets (SSE/AVX on x86_64, NEON on aarch64). There is no trait object and no vtable on the hot path,
so sharing the code costs nothing at run time; that is what lets the same code be the embedded path and the desktop
path.

The crate has no hand-written SIMD, by design: it keeps the code `no_std` and portable. On `wasm32-unknown-unknown`
the vectorisation therefore needs `-C target-feature=+simd128` in the consuming build (about 20 % on FT8,
`BENCHMARKS.md`, "WASM: `+simd128`").

### The same code on a microcontroller

The targets differ in what they can afford, not in the algorithm, so the differences are confined to a few seams
rather than a second implementation:

- **FFT** is a trait (`engine::fft::FftPlanner`): `rustfft` on a host, `esp-dsp` or CMSIS-DSP behind `fft-extern` on a
  board.
- **Parallelism** is `rayon` where `std` is, sequential where it is not, over the same loops.
- **Arithmetic**: the `fixed-point` feature swaps the scalar types of the hot path (`engine::scalar`) for Q-format
  integers, which is what the shipped boards run (`EMBEDDED.md`, "how f32 and fixed-point share one codebase").
  Embedded work is therefore measured on that path, not on `f32`.

### A static set of protocols

`PROTOCOLS` is a `const` slice fixed at compile time by Cargo features. There is no runtime `register_protocol()`:
every wired ZST is checked by `tests/protocol_invariants.rs` against the trait surface, and that guarantee cannot
extend to types unknown at compile time. The pay-off is in adding a protocol: FST4-60A joined as a trait impl on one
ZST plus a Costas table, with no change to shared pipeline code, and later sub-modes are one `fst4_submode!` /
`q65_submode!` line. A new protocol is a ZST, a `protocol_meta!` line and a rebuild (`CONTRIBUTING.md`, "Adding a new
protocol").

### What stays outside the trait

MSK144 and JTTY sit outside `Protocol` on purpose, and it is not a gap to close. MSK144 is continuous-phase MSK in
transient meteor bursts, scanned across the whole period; JTTY has no slot (frames start whenever the sender likes,
and a message is several of them). Neither fits the static-slot model the trait describes, and forcing them in would
make the trait describe nothing.

## 4. The decode API is `jt9`'s shape

### The decision

The public decode API copies how WSJT-X runs its decoders, one to one (since 0.13):

| WSJT-X | this crate |
|---|---|
| one decoder process per mode | `Decoder<P>` |
| the `params` block the GUI fills before each period (`jt9com`) | `DecodeParams` |
| SAVE and module variables | `P::State`, owned by the decoder |
| `ndepth` | `Depth`, deciding every search setting per mode, with the upstream lines cited |
| — | `P::Extras`: what this crate adds, typed per mode |

Transmit is the same idea in one call: `encoder::encode(mode, text)`.

Three things follow from that shape, and they are the reason for it:

1. **Comparing against upstream becomes a definition, not a translation.** A task in `scripts/upstream_tasks.json`
   is the same parameter block on both sides, which is what lets `UPSTREAM_EVALUATION.md` pair trials one to one
   and what §2's "same output" means in practice.
2. **State is correct by construction.** It lives where upstream keeps it — the hash table, FT8's previous cycle,
   WSPR's call table, Q65's average — one per decoder as one per process. A multi-channel caller cannot mix two
   channels' tables or forget to pass one. Before 0.13 the caller carried that state, and the crate's own
   `IqReceiver` got it wrong (0.12.0 was yanked for it: `docs/historical/CHANGELOG-0.13.md`).
3. **Divergence is visible in the types.** Everything upstream has is in `DecodeParams`; everything this crate adds
   is in `Extras`, so the rule of §2 that a deliberate divergence is called out holds by construction, and an option
   a mode lacks does not compile.

### What it costs, accepted

- **A field a mode does not read is ignored, not rejected**, as `jt9` ignores it (`tol_hz` on FT8). The "read by"
  column of `LIBRARY.md` §2.2 is the only guard.
- **The public block follows upstream's.** `averaging` and `deep_search` are `ndepth & 16` and `& 32`; a change to
  `jt9com` upstream is a reason to change this API.
- **No table shared between decoders.** Two channels of one mode learn callsigns separately, as two WSJT-X instances
  do.
- **`MessageFilter` takes a function pointer, not a closure**, so the extras stay `Clone + 'static`; a predicate that
  needs data reads a `static`.
- **Heavier defaults** (FT8 `Deep`: sync 1.3, 1000 candidates), because they are upstream's. Measured, this costs
  nothing that matters: at `Deep` the crate is 3.8–4.0× `jt9 -8 -d 3`'s speed on the same files, and decodes the same
  21 messages on `qso3_busy.wav` (`BENCHMARKS.md`).

### What it bought, measured

- QSO-context AP for FT8, FT4 and FST4 from upstream's `naptypes`: weak replies on FT8, 0 of 30 decoded with AP off
  and 20 of 30 on.
- Sensitivity "the same" as WSJT-X v3.2.0-rc1 on every mode but legacy JT65, on tasks defined identically on both
  sides (`BENCHMARKS.md`, 2026-10-04).
- One C handle for every mode (`mfsk_decoder_*`) and one shape for the Kotlin and Swift bindings, because the Rust
  side has one shape to expose.

The 0.12 API it replaced, and the migration, are in `LIBRARY.md` §1.2 and `docs/historical/CHANGELOG-0.13.md`.

## 5. A wide public surface, in three tiers

The crate serves two kinds of user: people building **applications** (receive, decode, transmit) and people building
**protocols** (a new mode on the existing traits, codecs and DSP — the fifth use in §1). Hiding the internals would
serve the first and lock out the second, and splitting them into two crates would make every shared fix a
cross-crate release. So the surface is wide on purpose, and three tiers say where to look (`LIBRARY.md` §4):

| tier | for | what |
|---|---|---|
| **A: application** | receive, decode, transmit | `prelude`, `decoder`, `encoder`, `iq`, `slotgrid`, `registry`, the message codecs, the protocol ZSTs |
| **P: protocol building** | a new mode | the root traits, `decoder::Decodable`, `engine::tx`, `fec::*` |
| **B: building blocks** | replace a stage, embedded | `engine::{dsp, sync, llr, …}`, protocol internals |

Two things are deliberately *not* public: the decode pipeline driver (`engine::pipeline`, `pub(crate)`), whose shape
changes whenever upstream's ladder does, and everything behind `internal-testing` (`mfsk_core::__internal`, no semver),
which is the tests' and the embedded crates' way in. No other public path depends on a feature.

## 6. Wide-band is the main path

WSJT-X decodes a whole passband for every mode. The crate's main path does the same, for every mode, and that path is
where fidelity is owed: **if a mode is not WSJT-X-faithful without a special mode, that is a bug in the wide-band
path**, not a reason to reach for the special mode. Three features that look like alternatives to it are, by this
rule, something narrower.

### Sniper is a roofing-filter mode, for FT8

The ±250 Hz window is hardware, not cleverness. Sniper mode is the software half of narrowing the *analogue* roofing
filter on a transceiver that has one at that width — the Yaesu FTDX101MP and FTDX10 are the usual examples. The
operator selects a ~500 Hz analogue filter and points it at a DX station whose carrier is already known; the audio
reaching the decoder is already band-limited to that slice, and searching ±250 Hz matches what the hardware left. It
is a special-purpose path for specific radios, not the natural shape for "any mode where you know who you are
working".

It is FT8-only, and that is the design: `SupportsSniper` is implemented for `Ft8` alone. FT4 is a contest protocol
whose premise is working a full band, so a roofing filter is against its purpose; FST4 narrows through its own DDC
channelizer (`FST4_DDC_DESIGN.md`), the same benefit without a second decode engine. The C ABI has no sniper call of
its own: the capability is `MFSK_CAP_SNIPER`, driven through `MfskDecodeParams::search_hz`.

### Equalisation is a property of the input audio

Local equalisation (`eq: EqMode::Local`) flattens the passband tilt that an analogue filter's skirt puts in. It
therefore describes the *audio*, not the search, and is available on the wide-band decode too: filtered audio can be
fed to a wide-band decode, and FT8's `eq_mode_recovers_bpf_edge_signal` pins a band-pass-edge signal that decodes with
`Local` and not with `Off` through the wide-band SIC engine.

On flat input, EQ can only cost: the `ft4sim`-generated FT4 golden has no receiver filter, and `Local` loses two
decodes of fourteen there. So the default is `Off`, right for recordings and simulations and wrong for a narrowed
receiver.

### A-priori decoding is a general option

AP locks high-confidence bits and lowers the threshold by 1–3 dB; nothing about it is narrow-band. It is the last rung
of the per-candidate ladder — `engine::pipeline::process_candidate_basic` for FT4 and every FST4 sub-mode, FT8's own
`ft8::decode_block::process_one_candidate_inner` for FT8 — so everything above it has already run and failed, and it
can only add decodes. `msg::pipeline_ap` is hypothesis generation (upstream's `iaptype`) with no engine of its own.

It was once coupled to the sniper by accident, through a separate engine whose candidate loop stopped on "a hint is
present"; on the WSJT-X golden that engine returned 4 decodes where the plain path returns 11. The account, and the
two AP bugs found while removing it (FT4 / FST4 locking scrambled bits; the missing blind-CQ hypothesis, FT4 AWGN
−16.89 → −18.00 dB), are in `docs/historical/CHANGELOG-0.11.md`; the move of the accept bound and the heavy
hypotheses to `ft8b.f90`'s rules is in `docs/historical/CHANGELOG-0.12.md` (#456).

AP's risk is manufactured decodes, so every AP path ships with precision guards next to its gain assertion (§7).

## 7. Measure, and never recall without precision

**Measure, don't assert.** Comments and docs carry what was measured, on what corpus, not what was intended. A
bottleneck is named only after it is measured; hypotheses are ranked by measurement. Embedded work is measured on the
shipped numeric path (`fixed-point`, §3), because that is what runs.

**Recall without precision is not a result.** A decoder can always find more by accepting more, and every decode it
manufactures is a station that was never there, sent on to a spot network or a log. So every decode path — not only
`decode()`, but each non-default strategy (SIC rounds, `sic_early`, AP, list decoders), where phantoms actually come
from — is guarded: golden tests go through `assert_golden`, recall floor and phantom ceiling together, with a
target of `max_extra: 0`, and a nonzero budget is documented debt. A new strategy ships with its precision guard in
the same change.

**Tests are organised by the property they assert**, because that is what says when to run them:

| tier | asserts | when |
|---|---|---|
| A | invariants: unit, round trip, bit-exact parity, TX waveform | every change |
| B | golden fidelity against a reference decoder: recall **and** phantoms **and** SNR, on vendored data (missing = failure) | every change |
| C | sensitivity: AWGN / fading threshold curves, unexpected decodes per SNR cell, ~17 GB generated | before a release, after a sensitivity-moving change |

A test must pass on a clean checkout, must not depend on wall clock or device speed, and must be able to fail; one
that cannot is deleted (`TESTING_GUIDE.md`).

## 8. Why Rust

- **Safety**: LDPC belief propagation, Berlekamp-Massey + Forney for RS and Fano sequential decoding are index-heavy
  code; safe Rust removes the memory-corruption class of bug from them.
- **Generics and trait bounds** describe a protocol family as data plus traits (§3); in C++ that is template
  metaprogramming, in Fortran it is not available.
- **Targets**: one source tree builds for `wasm32-unknown-unknown`, Android `arm64-v8a`, iOS, `no_std + alloc`
  microcontrollers and any host — the five places in §1.
- **Ecosystem**: `rustfft`, `num-complex`, `crc`, `rayon` keep the dependency graph small and reviewable.

---

# Part II — decisions still load-bearing

## 9. Why the embedded path doesn't widen PASS1 or enable OSD

Tested against the WSJT-X reference busy band on real S3 LX7 silicon
(`logs/s3_pass100_max30_2026-05-04.log`):

| config | qso3 post-SlotEnd | qso3 recall | total recall |
|---|---:|---:|---:|
| Bp/30/15 (ship) | **~1.2 s** | 7/18 | 14/22 (or 15 with phantom) |
| Bp/100/30 | **~1.6 s** | 7/18 (unchanged) | +1 (qso1 OH3NIV only) |
| `DecodeDepth::FULL`/200/100 (host estimate) | ~7 s | 7/18 (+1 on qso3 N1JFU) | 16/22 |

Two non-obvious findings drove the decision to stay at
`PASS1 = 30 / max_cand = 15`:

1. **qso3 busy-band recall is bounded by coarse_sync rank, not BP/OSD
   effort.** Widening PASS1 from 30 → 100 and max_cand 15 → 30 recovers
   no qso3 calls — the missed signals are below coarse_sync rank 100
   entirely. They need iterative subtraction (the WSJT-X wide-band
   path's hallmark), which `decode_block` doesn't implement.
2. **The FT8 QSO turnaround budget is ~2 s post-SlotEnd**, not the full
   15 s slot. After decode the UI has to draw the waterfall, update the
   callsign list, render RPRT, prep next-slot TX, and — on chips
   without an NTP-synced or GPS-disciplined RTC — re-estimate slot
   timing from the **median** `dt_sec` of decoded signals. (A plain
   mean is outlier-sensitive: one bogus-sync but CRC-valid decode skews
   the slot phase noticeably, and the ESP32's internal RTC drift is
   large enough that frame alignment has to be slaved to this
   decoder-derived estimate.) Bp/100/30 on qso3 leaves only ~0.4 s for
   all of that before the next TX must start — too tight. The +1
   qso1-only recall gain isn't worth the headroom loss.

So the embedded `decode_block` ships at the recall floor that fits the
2 s budget cleanly. Pushing further requires either porting iterative
subtraction to the embedded path (open question on cost — see
`ROADMAP.md`, "Embedded fine_refine attempt postmortem") or accepting
late-arrival "spotter mode" decodes that land too late for QSO
turnaround.

## 10. What is deliberately not ported

Each of these was evaluated and declined; read the record before re-deriving it.

| what | why not | record |
|---|---|---|
| SuperFox | its code table is licensed for WSJT-X only | #436 |
| FT2 | | #437 |
| WS's 77-bit extension | | #568 |

Something tempting from outside WSJT-X: `UPSTREAM_EVALUATION.md` and these records first, then ask.
