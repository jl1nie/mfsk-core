# mfsk-core — Library Architecture & API Reference

> **日本語版:** [LIBRARY.ja.md](LIBRARY.ja.md)

This document covers the mfsk-core library surface for embedders:
Rust crate consumers, C/C++ projects linking `libmfsk.so`, and
Kotlin/Android apps using the JNI scaffold.

For a quick-start overview (badges, dependency snippet, minimal
example) see [README.md](../../README.md). This document goes deeper
into *why* and *how*.

## 0. Introduction

**mfsk-core in one paragraph.** A pure-Rust reimplementation of the
WSJT-X weak-signal digital decoders (FT8, FT4, FST4, WSPR, JT9, JT65,
Q65, MSK144) behind a single generic core. The core
(`engine` / `fec` / `msg`) is protocol-agnostic; each protocol is a
small zero-sized type that plugs a FEC codec, a message codec and a
sync mode into it. One receive flow runs for every wired protocol —
`coarse-sync → refine → LLR → FEC decode → message unpack` (diagram in
§0.3) — with per-protocol strategy variations layered on top (§3). If you read nothing else, read **§0.3** (the layering), **§0.5**
(what each protocol reuses vs. brings itself), **§1** (module & crate
map) and **§3** (decode strategies).

### 0.1 Background

The weak-signal digital modes addressed by this library — FT8, FT4,
FST4, WSPR, JT9, JT65 and their siblings — were developed by Joe
Taylor K1JT and his collaborators as part of the WSJT-X project,
which is the reference implementation for the entire family. Every
algorithm in mfsk-core (sync correlation, LLR computation, LDPC
BP / OSD decoding, Fano sequential decoding of convolutional codes,
Reed-Solomon erasure decoding, per-protocol message encoding, …) is
derived from WSJT-X. Each source file's docstring cites the
corresponding file under `lib/ft8/`, `lib/ft4/`, `lib/fst4/`,
`lib/wsprd/`, `lib/jt9_*`, `lib/jt65_*`, etc.

WSJT-X evolved as a C++ + Fortran desktop application, and has been
refined in that form over many years. Deploying those same
algorithms outside the desktop — running them in a browser PWA,
embedding them in a standalone Android app, or calling them as a
library from another Rust or C++ project — requires a non-trivial
amount of per-platform work if one starts from the upstream source.

### 0.2 Goal

mfsk-core re-implements the WSJT-X algorithms in Rust and organises
them as a single crate that can be consumed identically from several
runtimes (native Rust, WebAssembly, Android JNI, C ABI). The aim is
to keep algorithmic equivalence with the upstream C++/Fortran code
while broadening the set of platforms that can host it.

### 0.3 The design in one picture: a generic core + per-protocol plug-ins

Read the crate bottom-up. There is a **generic core** that knows
nothing about any specific protocol, and each protocol is a thin
plug-in that selects which pieces of that core it uses.

1. **`engine/`** — protocol-agnostic DSP, sync, LLR, the equaliser,
   and the decode pipeline. Every function here is generic over
   `P: Protocol` and reads the protocol's constants; it contains no
   per-protocol branches.
2. **`fec/`** — the forward-error-correction codec families, each an
   `impl FecCodec`: LDPC (three sizes) + BP/OSD, convolutional + Fano,
   Reed-Solomon, and the Q-ary QRA codec.
3. **`msg/`** — the message codecs, each an `impl MessageCodec`, plus
   the generic `DecodeRequest`/`SniperRequest` builders (§4) that drive
   the whole pipeline.
4. **A protocol** is a zero-sized type (ZST) implementing three
   composable traits (`ModulationParams` + `FrameLayout` → `Protocol`,
   §2). It carries only constants and two associated-type choices —
   `type Fec` and `type Msg` — plus a `SYNC_MODE`. That is the entire
   act of "adding a protocol": pick a FEC, pick a message codec, pick a
   sync mode, declare the numbers.
5. **The FFI crates** stack on top of `mfsk-core`:
   `mfsk-ffi-abi` (shared `#[repr(C)]` status/result/options types) →
   `mfsk-ffi` (full C ABI, all protocols) and `mfsk-ffi-ft8` (FT8-only,
   embedded-friendly C ABI). Embedded targets under `embedded-poc/` are
   separate Cargo projects outside the workspace (§1, §8).

At decode time those layers execute as one receive flow, shared by
every wired protocol — free functions in `engine`, each generic over
`P: Protocol` (function-level notes in §4):

```text
┌─────────┐  coarse_sync   ┌──────────────┐  refine_candidate  ┌──────────┐
│ samples │ ─────────────▶ │  candidates  │ ─────────────────▶ │ candidate│
│ i16/f32 │  (FFT/Costas)  │ (f, dt, snr) │   (fine sync)      │ refined  │
└─────────┘                └──────────────┘                    └────┬─────┘
                                                                    │  symbol_spectra
                                                                    ▼
                  ┌─────────────┐  compute_llr  ┌──────────────┐  equalize_local
                  │   LLR vec   │ ◀───────────  │     cs[]     │ ◀──────────┐
                  │  (4 vars)   │   (per WSJT)  │   Complex    │ (per-tone  │
                  └──────┬──────┘               │  per-symbol  │  Wiener)   │
                         │                      └──────────────┘            │
                         │  P::Fec::decode_soft  (LDPC BP / Fano / RS /     │
                         │                        QRA-symbol-level)         │
                         ▼                                                  │
                  ┌─────────────┐                                           │
                  │ info bits   │                                           │
                  └──────┬──────┘                                           │
                         │  P::Msg::unpack                                  │
                         ▼                                                  │
                  ┌─────────────┐                                           │
                  │ message txt │ ──── (subtract for next iter) ────────────┘
                  └─────────────┘
```

Because `P: Protocol` is a **compile-time** type parameter,
monomorphisation emits one fully-specialised copy per protocol — the
abstraction has no runtime cost (§2, "Monomorphisation & zero cost").
Direct consequences: the same algorithm runs under native Rust, WASM,
Android and C/C++; an improvement to a shared path (e.g. LDPC BP)
benefits every protocol that uses it; adding a protocol keeps the diff
confined to that protocol's plug-in; and the C ABI branches once on
`protocol_id`, already-specialised past that point.

**§0.5 below is the payoff of this design** — a single table showing,
for every protocol, exactly which core pieces it reuses (generic) and
which it brings itself (bespoke).

### 0.4 Currently supported protocols

| Protocol     | Slot   | FEC                          | Message | Sync                 | Upstream source |
|--------------|--------|------------------------------|---------|----------------------|-----------------|
| FT8          | 15 s   | LDPC(174, 91) + CRC-14        | 77 bit | 3×Costas-7           | `lib/ft8/`      |
| FT4          | 7.5 s  | LDPC(174, 91) + CRC-14        | 77 bit | 4×Costas-4           | `lib/ft4/`      |
| FST4-60A     | 60 s   | LDPC(240, 101) + CRC-24       | 77 bit | 5×Costas-8           | `lib/fst4/`     |
| WSPR         | 120 s  | convolutional r=½ K=32 + Fano | 50 bit | per-symbol LSB       | `lib/wsprd/`    |
| JT9          | 60 s   | convolutional r=½ K=32 + Fano | 72 bit | 16 distributed slots | `lib/jt9_decode.f90`, `lib/conv232.f90` |
| JT65         | 60 s   | Reed-Solomon(63, 12) GF(2⁶)   | 72 bit | 63 distributed slots (pseudo-random) | `lib/jt65_decode.f90`, `lib/wrapkarn.c` |
| Q65-15A      | 15 s   | QRA(15, 65) GF(2⁶) + CRC-12   | 77 bit | 22 distributed slots | `lib/qra/q65/`  |
| Q65-30A      | 30 s   | (same QRA codec)              | 77 bit | (same sync layout)   | `lib/qra/q65/`  |
| Q65-60A‥E    | 60 s   | (same QRA codec)              | 77 bit | (same sync layout)   | `lib/qra/q65/`  |
| Q65-120D‥E   | 120 s  | (same QRA codec)              | 77 bit | (same sync layout)   | `lib/qra/q65/`  |
| Q65-300A     | 300 s  | (same QRA codec)              | 77 bit | (same sync layout)   | `lib/qra/q65/`  |

Q65 ships as ten wired sub-modes — two terrestrial modes (15-s and
30-s), five 60-s EME modes (Q65-60A through Q65-60E with
tone-spacing multipliers ×1, ×2, ×4, ×8, ×16), and three longer-period
scatter modes (Q65-120D 10 GHz rainscatter/troposcatter, Q65-120E
6 m ionoscatter, Q65-300A optical scatter — the deepest wired
sub-mode at ~-34 dB AWGN threshold). They share the FEC, message
codec, sync layout and a common impl block; only NSPS and tone
spacing differ.

FST4 similarly ships as five wired T/R-period sub-modes — FST4-15,
FST4-30, FST4-60A, FST4-120, FST4-300 — sharing LDPC(240, 101),
message codec, Costas-8 sync and GFSK shaping (BT=2.0); only `NSPS` /
`NDOWN` / `SYMBOL_DT` / `TONE_SPACING_HZ` differ (and `TX_START_OFFSET_S`
for FST4-15 alone). FST4-900 and FST4-1800 are deliberately not wired
(no user demand as of writing); FST4W, the WSPR-style one-way 50-bit
beacon variant (LDPC(240, 74), periods 120/300/900/1800 s), is a
separate message format not covered here — see issue #23.

**MSK144** is deliberately not in this table — no ZST implements
`Protocol` for it. See the MSK144 row and its footnote in §0.5.

### 0.5 Generic vs bespoke, per protocol

This is the map to read first. Each row is one protocol; each cell
says whether that layer is **shared** (reused verbatim from the
generic core in §0.3) or **own** (code that lives in the protocol's
own module). "Generic" and "bespoke" here mean exactly that — a shared
cell costs the protocol nothing to author; an own cell is the work
that protocol had to bring.

| Protocol | FEC codec | Message codec | Sync mode | Decode entry point |
|----------|-----------|---------------|-----------|--------------------|
| **FT8**  | shared `Ldpc174_91` | shared `Wsjt77Message` (77-bit) | `Block` — 3×Costas-7 | generic `DecodeRequest`, dispatching to FT8's own `ft8::decode_block` engine [^ft8] |
| **FT4**  | shared `Ldpc174_91` | shared `Wsjt77Message` (77-bit) | `Block` — 4×Costas-4 | generic `DecodeRequest` / `engine::pipeline` |
| **FST4** | shared `Ldpc240_101` | shared `Wsjt77Message` (77-bit) | `Block` — 5×Costas-8 | generic `DecodeRequest` / `engine::pipeline` |
| **WSPR** | own `ConvFano` (conv r=½ K=32 + Fano) | own `Wspr50Message` (50-bit) | own `Interleaved` [^wspr] | bespoke `wspr::decode` |
| **JT9**  | own `ConvFano232` (conv, 206-bit framing) | shared `Jt72Codec` (72-bit) | `Block` (length-1 slots) | bespoke `jt9` entry |
| **JT65** | own `Rs63_12` (RS GF(2⁶), erasure-aware) | shared `Jt72Codec` (72-bit) | `Block` (length-1 slots) | bespoke `jt65` entry |
| **Q65**  | own `Q65Fec` + QRA codec over GF(64) [^q65] | own `Q65Message` (77-bit) | `Block` | bespoke `q65::rx` + Q65-local `DecodeRequest` |
| **uvpacket** | shared `Ldpc240_101` (punctured) | own `UvPacketRawMessage` (byte-pipe) | `Block` — Costas-4 [^uv] | bespoke `uvpacket::rx` |
| **MSK144** | shared `Ldpc128_90` + CRC-13 | shared `msg::wsjt77` (77-bit) | **none — opts out of `Protocol`** [^msk] | bespoke `msk144::decode::decode_slot` |

The pattern the table makes visible:

- **FT8 / FT4 / FST4** are the "cheap" additions — LDPC + 77-bit
  message + block-Costas sync, so almost everything is shared. Their
  common code is a consequence of shared structure, not a test of the
  abstraction.
- **WSPR** swaps all three of *FEC family*, *message width* and *sync
  mode* independently — the proof that those axes are genuinely
  orthogonal.
- **Q65** adds a third FEC family (non-binary QRA over GF(64)), ten
  sub-modes from one macro, and five parallel decoder strategies (§3),
  all still inside the same `Protocol` super-trait.
- **uvpacket** is a non-WSJT applied example: it reuses only the FEC
  mother code and bypasses the generic TX/RX pipeline (§10.1).
- **MSK144** is the one protocol that opts out of the trait surface
  entirely, yet still reuses the FEC and message layers.

`§3` expands the Q65 decoder strategies; `§7` covers the `PROTOCOLS`
registry and the generic `tests/protocol_invariants.rs` checker that
enumerate and validate every wired ZST (24 in total: 20 WSJT-family
protocols/sub-modes plus 4 `uvpacket` sub-modes).

[^ft8]: FT8 uses the generic `DecodeRequest` builder like FT4/FST4, but
    internally routes through its own hand-tuned `ft8::decode_block`
    engine (host + embedded shared) rather than `engine::pipeline`;
    see §4 "FT8 block-decoder entry points" and §10.

[^wspr]: `SyncMode::Interleaved` — the lower bit of every channel
    symbol carries one bit of a fixed 162-bit sync vector, so sync is
    not a block of Costas arrays. WSPR is the only user of this variant.

[^q65]: `Q65Fec::decode_soft` returns `None` **by design** — the real
    decode runs over GF(64) probability vectors via the QRA codec
    (`fec::qra` + `fec::qra15_65_64`), not bit-LLRs. `NTONES = 65` with
    `BITS_PER_SYMBOL = 6` (tone 0 is a reserved sync tone) is the case
    that loosened the `GRAY_MAP` length contract to
    `[2^BITS_PER_SYMBOL, NTONES]`. See §1 "`FecCodec` is
    symbol-agnostic" and §3.

[^uv]: uvpacket bypasses the generic pipeline, so several of its
    `ModulationParams` constants are decorative — present only to
    satisfy the trait and the invariant test. See §10.1.

[^msk]: MSK144 (issue #25) is continuous-phase binary MSK sent as
    offset-QPSK, and repeats an 864-sample frame through the whole T/R
    period rather than sitting at a known offset in a fixed slot — so
    neither `ModulationParams`/`FrameLayout` nor `engine::pipeline`
    fit, and no ZST implements `Protocol` for it. Its own
    `msk144::decode::decode_slot` driver scans for pings via
    `msk144::spd`/`msk144::sync`. It still reuses the 77-bit
    `msg::wsjt77` codec and the generic LDPC BP/OSD engine
    (`fec::ldpc_128_90`, added the same way `Ldpc240_101` was for
    FST4). Golden-WAV recall vs WSJT-X `samples/MSK144/*.wav` is 3/3
    (`tests/msk144_wsjtx_samples.rs`).

## 1. Module layout

```text
mfsk_core
├── engine/           Protocol traits, DSP, sync, LLR, equaliser, pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract · msk · analytic
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (Wiener per-tone)
│   ├── spectrogram.rs  Spectrogram build/score kernel — JT9, JT65, Q65
│   │                   (WSPR keeps its own: fixed-point FFT backend +
│   │                   baseline-fit normalisation, a real difference)
│   ├── interleave.rs   bit-reversal interleave_bitrev/deinterleave_bitrev
│   │                   — WSPR, JT9 (JT65's is a different algorithm,
│   │                   a 7×9 matrix transpose, and stays in jt65/)
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
│                       (pub(crate) internals — see §4; call via
│                       msg::decode_request::DecodeRequest/SniperRequest)
├── fec/              FecCodec implementations
│   ├── ldpc/           LDPC(174, 91)  — FT8, FT4 (bp.rs / osd.rs / params.rs / tables.rs)
│   ├── ldpc240_101/    LDPC(240, 101) — FST4
│   ├── ldpc_128_90/    LDPC(128, 90)  — MSK144
│   ├── conv/           ConvFano r=½ K=32 — WSPR; ConvFano232 — JT9 (fano.rs)
│   ├── rs/             RS(63, 12) GF(2⁶) — JT65
│   └── qra/            Q-ary RA codec family — Q65
│       ├── code.rs       Generic QRA encoder + non-binary BP decoder
│       ├── q65.rs        Q65 application wrapper (CRC-12 + puncturing) +
│       │                 list-decoding primitives (check_codeword_llh,
│       │                 decode_with_codeword_list)
│       ├── fast_fading.rs Doppler-spread-aware intrinsic metric
│       ├── fading_tables.rs Gaussian / Lorentzian calibration tables
│       ├── npfwht.rs      Non-binary Walsh-Hadamard transform helpers
│       └── pdmath.rs      Probability-domain BP math helpers
├── msg/              Message codecs
│   ├── decode_request.rs DecodeRequest / SniperRequest — the public decode
│   │                     entry point for FT8/FT4/FST4 (§4, replaces the
│   │                     pre-0.8.0 decode_frame*/decode_sniper* families)
│   ├── wsjt77.rs       77-bit WSJT message (pack / unpack) — FT8, FT4, FST4, Q65, MSK144
│   ├── wspr.rs         50-bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72-bit JT message — JT9, JT65
│   ├── callsign28.rs   shared base-37/36/10/27³ callsign pack/unpack
│   │                   core — jt72 (JT9/JT65) and wspr both wrap it;
│   │                   traces to one upstream routine (`packjt.f90`'s
│   │                   packcall/unpackcall)
│   ├── q65.rs          77-bit <-> 13x GF(64)-symbol packing for the QRA codec
│   ├── ap.rs           ApHint — a-priori hint builder (with_call1/call2/grid/report)
│   ├── pipeline_ap.rs  AP-assisted multi-pass decode pipeline (77-bit-family protocols)
│   ├── packet_bytes.rs PacketBytesMessage — byte-payload example codec
│   └── hash_table.rs   Callsign hash table
├── registry.rs       PROTOCOLS static + ProtocolMeta + by_id / by_name
├── ft8/              FT8 ZST + decode + wave_gen
├── ft4/              FT4 ZST + decode
├── fst4/             FST4 family — 5 sub-mode ZSTs (15/30/60A/120/300) + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ erasure-aware RS)
├── q65/              Q65 family — 10 sub-mode ZSTs + decode + synth
│   ├── protocol.rs     q65_submode! macro emitting Q65a15 .. Q65a300 ZSTs
│   ├── rx.rs           5 decoder strategies (AWGN / AP-hint / fast-fading / AP-list / multi-period), see §3
│   ├── ap_list.rs      standard_qso_codewords — full AP-list candidate generator
│   ├── tx.rs           65-FSK synthesiser (sub-mode-aware)
│   ├── search.rs       coarse 22-symbol Costas-block search
│   └── sync_pattern.rs Q65 distributed sync layout
├── msk144/           MSK144 — no Protocol impl; own top-level driver (§0.5)
│   ├── tx.rs           codeword -> 864-sample complex OQPSK frame
│   ├── sync.rs         joint (CFO, timing) matched-filter search
│   ├── spd.rs          burst-candidate detector + short-ping decode loop
│   ├── frame_decode.rs sync gate -> LLR -> LDPC -> message
│   └── decode.rs       decode_slot(): sliding-window top-level driver
└── uvpacket/         Applied non-WSJT example — 4 sub-mode ZSTs, own tx/rx (§10.1)
    ├── protocol.rs     ModulationParams/FrameLayout impls (partly decorative, see §10.1)
    ├── framing.rs      variable-length burst framing
    ├── sync_pattern.rs 4-variant 127-chip BPSK m-sequence preamble
    ├── interleaver.rs  bit interleaver
    ├── puncture.rs     LDPC240_101 puncturing for the header block
    ├── message.rs      byte-pipe (app_type) message layer
    ├── tx.rs           π/4-DQPSK + RRC synthesiser
    └── rx.rs           LMS equaliser + differential demod + decode
```

Each protocol module is gated behind a feature flag (`ft8`, `ft4`,
`fst4`, `wspr`, `jt9`, `jt65`, `q65`, `msk144`, `packet-bytes`,
`uvpacket`). The `engine`, `fec`, `msg` and `registry` modules are
always available.

### Workspace crates

The Cargo workspace is four crates stacked on `mfsk-core`:

| Crate | Responsibility | Use when |
|-------|----------------|----------|
| `mfsk-core` | The library itself: every decoder / synthesiser plus the generic `Protocol` core (`engine` / `fec` / `msg`). Host (rustfft) or `no_std` + alloc via a pluggable FFT backend. | You consume it from Rust, WASM, or embedded — the base everything else builds on. |
| `mfsk-ffi-abi` | The shared `#[repr(C)]` status / result / options types (no logic). | Internal — depended on by both FFI crates so they share one ABI shape. |
| `mfsk-ffi` | The full C ABI: `libmfsk.{so,a,dylib}` + generated `mfsk.h`, **all** protocols (`features = ["full"]`). | C / C++ / Kotlin consumers needing any protocol (§8, §9). |
| `mfsk-ffi-ft8` | A smaller **FT8-only**, embedded-friendly C ABI: `libmfsk_ft8`, host or `no_std` fixed-point. | MCUs / size-constrained builds that only need FT8. |

Embedded application crates under `embedded-poc/` (ESP32-S3, RP2350,
Cortex-M) are separate Cargo projects **outside** the workspace and
depend on `mfsk-core` by path.

#### `FecCodec` is symbol-agnostic

The `FecCodec` trait surface (`engine/protocol.rs`) speaks in **bits**:
`&[u8]` info / codeword, `&[f32]` bit-LLRs, `K` and `N` counted in
bits. The four FEC families above include two non-binary codes —
Reed-Solomon over GF(2⁶) for JT65 and QRA over GF(2⁶) for Q65 — which
implement the bit-level trait by packing / unpacking bits ↔ symbols
inside their own `encode`. Their natural symbol-level decode lives
outside `decode_soft`: `Q65Fec::decode_soft` returns `None` by design,
and the real Q65 decode runs over GF(64) probability vectors via
`fec::qra::Q65Codec` from `q65::rx::decode_at_for`. Counting `K` /
`N` in bits keeps the cross-protocol invariant
`FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL` meaningful for both binary
and non-binary codes — see §7.2.

## 2. Protocol trait hierarchy

Every supported mode is described by a zero-sized type that
implements three composable traits:

<!-- Not compiled: re-declaring same-named traits here wouldn't
     actually check anything against the real definitions (unlike
     the worked examples below, which `impl` the real imported
     traits and so break if they drift). Kept in sync by hand against
     `engine/protocol.rs` when that file changes. -->

```rust,ignore
pub trait ModulationParams: Copy + Default + 'static {
    const NTONES: u32;
    const BITS_PER_SYMBOL: u32;
    const NSPS: u32;              // samples/symbol @ 12 kHz
    const SYMBOL_DT: f32;
    const TONE_SPACING_HZ: f32;
    const GRAY_MAP: &'static [u8];
    const GFSK_BT: f32;
    const GFSK_HMOD: f32;
    const NFFT_PER_SYMBOL_FACTOR: u32;
    const NSTEP_PER_SYMBOL: u32;
    const NDOWN: u32;
    const LLR_SCALE: f32 = 2.83;
}

pub trait FrameLayout: Copy + Default + 'static {
    const N_DATA: u32;
    const N_SYNC: u32;
    const N_SYMBOLS: u32;
    const N_RAMP: u32;
    const SYNC_MODE: SyncMode;  // Block(&[SyncBlock]) or Interleaved { .. }
    const T_SLOT_S: f32;
    const TX_START_OFFSET_S: f32;
}

pub enum SyncMode {
    /// Block-based Costas / pilot arrays at fixed symbol positions.
    /// Used by FT8 / FT4 / FST4.
    Block(&'static [SyncBlock]),
    /// Per-symbol bit-interleaved sync: one bit of a known sync vector
    /// is embedded at `sync_bit_pos` within every channel-symbol tone
    /// index. Used by WSPR (symbol = 2·data + sync_bit).
    Interleaved {
        sync_bit_pos: u8,
        vector: &'static [u8],
    },
}

pub trait Protocol: ModulationParams + FrameLayout + 'static {
    type Fec: FecCodec;
    type Msg: MessageCodec;
    const ID: ProtocolId;
}
```

### Worked examples — how the traits compose

Two concrete cases show how the three traits combine on a real ZST.

**FT4** — a standard block-Costas protocol that shares its FEC and
message codec with FT8:

```rust
use mfsk_core::engine::{
    FrameLayout, ModulationParams, Protocol, ProtocolId, SyncBlock, SyncMode,
};
use mfsk_core::fec::Ldpc174_91; // re-exported from fec::ldpc
use mfsk_core::msg::Wsjt77Message;

#[derive(Copy, Clone, Debug, Default)]
pub struct Ft4;

impl ModulationParams for Ft4 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 576;          // 48 ms @ 12 kHz
    const SYMBOL_DT: f32 = 0.048;
    const TONE_SPACING_HZ: f32 = 20.833;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 4;
    const NSTEP_PER_SYMBOL: u32 = 2;
    const NDOWN: u32 = 18;
    // (LLR_NSYM_MAX/INFO_SCRAMBLE_RVEC etc. are recall-tuning knobs
    // with defaults — see the real `ft4::Ft4` for FT4's overrides.)
}

impl FrameLayout for Ft4 {
    const N_DATA: u32 = 87;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 103;
    const N_RAMP: u32 = 2;
    const SYNC_MODE: SyncMode = SyncMode::Block(&FT4_SYNC_BLOCKS);
    const T_SLOT_S: f32 = 7.5;
    const TX_START_OFFSET_S: f32 = 0.5;
}

impl Protocol for Ft4 {
    type Fec = Ldpc174_91;          // shared with FT8
    type Msg = Wsjt77Message;       // shared with FT8
    const ID: ProtocolId = ProtocolId::Ft4;
}

const FT4_SYNC_BLOCKS: [SyncBlock; 4] = [
    SyncBlock { start_symbol:  0, pattern: &[0, 1, 3, 2] },
    SyncBlock { start_symbol: 33, pattern: &[1, 0, 2, 3] },
    SyncBlock { start_symbol: 66, pattern: &[2, 3, 1, 0] },
    SyncBlock { start_symbol: 99, pattern: &[3, 2, 0, 1] },
];
```

**WSPR** — structurally different on all three axes. The `Fec` and
`Msg` associated types switch to a new pair, and the sync is
expressed via `SyncMode::Interleaved`:

```rust
use mfsk_core::engine::{FrameLayout, ModulationParams, Protocol, ProtocolId, SyncMode};
use mfsk_core::fec::conv::ConvFano;
use mfsk_core::msg::wspr::Wspr50Message;

#[derive(Copy, Clone, Debug, Default)]
pub struct Wspr;

impl ModulationParams for Wspr {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 8192;                  // ~683 ms @ 12 kHz
    const SYMBOL_DT: f32 = 8192.0 / 12_000.0;
    const TONE_SPACING_HZ: f32 = 12_000.0 / 8192.0;  // ≈ 1.4648
    const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
    const GFSK_BT: f32 = 1.0;
    const GFSK_HMOD: f32 = 1.0;
    const NFFT_PER_SYMBOL_FACTOR: u32 = 1;
    const NSTEP_PER_SYMBOL: u32 = 16;
    const NDOWN: u32 = 32;
}

impl FrameLayout for Wspr {
    const N_DATA: u32 = 162;
    const N_SYNC: u32 = 0;                   // sync is embedded in data symbols
    const N_SYMBOLS: u32 = 162;
    const N_RAMP: u32 = 0;
    const SYNC_MODE: SyncMode = SyncMode::Interleaved {
        sync_bit_pos: 0,                     // LSB of the tone index
        vector: &WSPR_SYNC_VECTOR,           // 162-bit npr3
    };
    const T_SLOT_S: f32 = 120.0;
    const TX_START_OFFSET_S: f32 = 1.0;
}

impl Protocol for Wspr {
    type Fec = ConvFano;                     // convolutional + Fano
    type Msg = Wspr50Message;                // 50-bit message
    const ID: ProtocolId = ProtocolId::Wspr;
}

// Illustrative stand-in — the real 162-bit npr3 vector lives in
// `wspr::decode`'s private sync table.
const WSPR_SYNC_VECTOR: [u8; 162] = [0u8; 162];
```

Calling code just passes the ZST as a type argument —
`DecodeRequest::<Ft4>::new(...).decode()` (§4, §6.2) or the
WSPR-specific `wspr::decode::decode_scan_default(...)` — and the
trait composition pulls in the appropriate FEC, message codec, and
sync mode automatically.

### Monomorphisation & zero cost

All hot-path functions (`engine::sync::coarse_sync::<P>`,
`engine::llr::compute_llr::<P>`,
`engine::pipeline::process_candidate_basic::<P>`, …) take
`P: Protocol` as a **compile-time** type parameter. rustc
monomorphises one copy per concrete protocol; LLVM sees a
fully-specialised function and inlines the trait constants as
literals. The abstraction is free — the generated FT8 code is
byte-identical to the hand-written FT8-only path the library was
forked from, and FT4 benefits from every micro-optimisation applied
to the shared functions.

`dyn Trait` is reserved for cold paths only: the FFI boundary, the
protocol toggle in JS, and the `MessageCodec` that unpacks decoded
text (which runs once per successful decode, not once per candidate).

### Adding a new protocol

How much work a new protocol needs depends on how much of the
existing infrastructure it can reuse.

1. **Same FEC and same message as an existing mode** (e.g. FT2, or
   the other FST4 sub-modes). Define a new ZST and swap the numeric
   constants (`NTONES`, `NSPS`, `TONE_SPACING_HZ`, `SYNC_MODE`, and
   the sync pattern). `Fec` and `Msg` can be type aliases to the
   existing implementations, and the full `DecodeRequest::<P>`
   pipeline runs unchanged.

2. **New FEC but same message** (e.g. a different LDPC size). Add
   the codec as a new module under `fec/` and implement `FecCodec`
   for it. The BP / OSD / systematic-encode algorithms generalise
   naturally across LDPC sizes, so the only real changes are the
   parity-check and generator tables and the code dimensions (N, K).
   `fec::ldpc240_101` is the concrete example to follow.

3. **Both FEC and message are new** (e.g. WSPR). Add the FEC
   implementation, add the message codec, and — if the sync
   structure is fundamentally different — extend `SyncMode` with a
   new variant. WSPR was added via this route, introducing
   `ConvFano` + `Wspr50Message` + `SyncMode::Interleaved` while
   continuing to use the existing pipeline machinery (coarse
   search, spectrogram, candidate de-duplication, CRC check,
   message unpack).

4. **Sub-mode of an existing protocol** (e.g. Q65-60A through
   Q65-60E sharing everything with Q65-30A except NSPS / tone
   spacing). The `q65_submode!` macro takes the differing
   constants and emits the new ZST plus its three trait impls in
   one invocation; no new test or pipeline plumbing is needed —
   `tests/protocol_invariants.rs` mechanically picks up the new
   ZST after a one-line addition there.

## 3. Decoder strategies (Q65 case study)

**The shape of the decode across protocols.** Every protocol runs the
same underlying flow (§0.3), but the *strategy* wrapped around it varies.
Most protocols are a single pass; only Q65 exposes several parallel
receiver chains for one FEC frame, and MSK144 replaces the slot model
with a burst scan.

| Protocol | Generic (default) strategy | Special / optional strategies |
|----------|----------------------------|-------------------------------|
| **FT8**  | single-pass BP + OSD | AP iaptype loop (1–12); SIC 1–3 rounds (`.sic_rounds`/`.sic_early`) — §4 |
| **FT4**  | single-pass BP + OSD | SIC 1–3 rounds; full-slot coherent sync (`sync2d`) — §4 |
| **FST4** | single-pass BP + OSD | full-slot two-stage coherent sync search — §4 |
| **WSPR** | single bespoke pass (quarter-symbol spectrogram scan) | — |
| **JT9**  | single bespoke pass | — |
| **JT65** | single bespoke pass | RS erasure decode (`decode_at_with_erasures`); stochastic Chase decoder (`decode_at_with_chase`, #169) — §6.5 |
| **Q65**  | `(Δf,Δt,b90)` grid + Lorentzian fading BP (scan) | AP-hint, explicit fast-fading, AP-list, multi-period — **this section** |
| **MSK144** | burst-scan over the whole T/R period (not a static slot) | — |

The rest of this section details Q65, the richest case.

Most protocols in this library expose a single decoder entry point:
`DecodeRequest::<P>` for the FT family (§4, "The public decode entry
point"), `wspr::decode::decode_scan_default`
for WSPR, etc. Q65 is the first wired protocol where a single FEC
frame can be approached through several legitimately different
receiver chains, each trading runtime cost against a different kind
of channel pathology.

As of issue #204, these are exposed through three generic builders in
`mfsk_core::q65::decode_request` — `DecodeRequest<P>` (wide-band
scan), `SniperRequest<P>` (single known `(start_sample, base_freq_hz)`,
built via `DecodeRequest::sniper` or `SniperRequest::new` directly),
and `MultiPeriodRequest<P>` (averaged multi-slot decode) — mirroring
`msg::decode_request`'s FT8/FT4/FST4 shape (§4), generic over a
sealed `Q65SubMode` marker implemented for all ten sub-mode ZSTs. The
underlying `q65::rx` functions this section used to reference directly
(`decode_at_for`, `decode_scan_for`, …) are `pub(crate)` — the builders
are the public entry point. `.ap_hint()`, `.ap_list()`, `.fading()` are
plain inherent methods (not capability-gated marker traits like
`SupportsWideBandAp`) since every Q65 sub-mode supports every
capability uniformly.

A point worth flagging up front: `SniperRequest::decode()` with no
capability set (`decode_at_for` internally) really is the plain
Bessel-I0 metric — the point-decode-only baseline. But
`DecodeRequest::decode()` with no capability set (`decode_scan_for`
internally) — the shape nearly every real caller uses — routes each
coarse-search candidate through a WSJT-X-faithful `(Δf, Δt, b90)` grid
search using the fast-fading metric with `FadingModel::Lorentzian`,
ported from `q65_loops.f90` / `q65_dec_q012`. That mirrors WSJT-X's own
automatic decoder, which never actually runs a plain-AWGN-only Bessel
pass for its default scan either. So "AWGN" and "fast-fading" are best
read as two distinct *entry-point families* (sniper vs. scan), not two
cleanly separate front ends picked by channel type — the scan path
already assumes some amount of fading by default, and `.fading()`
exists for when the caller wants to pick a specific `(b90_ts, model)`
explicitly instead.

| When                                                   | Strategy                              | Builder call                                                          | Threshold gain |
|---------------------------------------------------------|----------------------------------------|------------------------------------------------------------------------|----------------|
| Single known candidate, unknown content                | AWGN Bessel + BP (point-decode only)   | `SniperRequest::<P>::new(...).decode()`                                | baseline       |
| Default scan — unknown channel, unknown content         | `(Δf,Δt,b90)` grid search + Lorentzian fading BP | `DecodeRequest::<P>::new(...).decode()`                        | WSJT-X-faithful default |
| Known callsign(s) or report, terrestrial channel        | AP-hint BP                             | `.ap_hint(&ap)` on either builder                                      | ~2 dB          |
| Doppler-spread channel, explicit model (microwave EME, ≥10 Hz spread) | Fast-fading metric + BP, caller-picked `(b90_ts, FadingModel)` | `.fading(model, b90_ts)` on either builder | 5–8 dB on spread channels |
| Known call pair, no QSO context, terrestrial            | AP-list template matching              | `.ap_list(&candidates)` on either builder                             | ~3 dB          |
| Weak/ionoscatter signal spanning several T/R periods    | Multi-period EMA averaging (3-stage cascade) | `MultiPeriodRequest::<P>::new(...).decode()`                     | recovers signals no single-period strategy can |

**AWGN Bessel + BP** (`SniperRequest` with no capability set) is the
textbook single-shot path: per-symbol FFT energies become probability
vectors via the Bessel-I0 metric, then non-binary belief propagation
runs on the QRA code. Falls back gracefully on any channel reasonably
close to additive Gaussian noise — but note it is the *point-decode*
shape only; see the note above for why the scan family doesn't stay on
this front end.

**AP-hint BP** (`.ap_hint(&ap)`) clamps the intrinsic probability
vectors at known information-bit positions before BP. A correct hint
shifts the BP fixed-point closer to the truth; a wrong hint typically
fails to converge rather than misdecoding (the CRC catches what's
left). Construct the hint via `mfsk_core::msg::ApHint` (`with_call1`,
`with_call2`, `with_grid`, `with_report`).

**Fast-fading metric** (`.fading(model, b90_ts)`) replaces the Bessel
front end with a spread-aware alternative, calibrated against a
caller-chosen `FadingModel::Gaussian` or `FadingModel::Lorentzian`
shape. Required for microwave EME where lunar libration spreads each
tone over 10–60 Hz: the 10 GHz EME reference recording in
`samples/Q65/60D_EME_10GHz/` decodes via this path but produces zero
hits with the plain Bessel front end. `b90_ts` is the spread
bandwidth × symbol period (typical: 0.05 = near-AWGN, 1.0 = moderate,
5.0+ = severe).

**AP-list template matching** (`.ap_list(&candidates)`) does *not* run
BP. Instead, the generator `q65::ap_list::standard_qso_codewords(my_call,
his_call, his_grid)` pre-encodes the WSJT-X "full AP list" — 206
standard exchanges that a known callsign pair can legally produce
(`MYCALL HISCALL`, `MYCALL HISCALL RRR/RR73/73`, `CQ HISCALL grid`,
plus the 200-entry SNR ladder). The decoder picks the candidate
whose log-likelihood under the soft observations exceeds a
size-adjusted threshold, or returns `None`. Useful when the
application has a known callsign pair but no QSO state — and at
SNR −25 dB (1 dB below the published Q65-30A threshold), the
test sweep shows AP-list decodes 6/6 frames where plain BP fails
0/6. `.ap_list()` and `.fading()` are mutually exclusive in the
underlying engine; `.decode()` resolves precedence as
ap_list > fading (+ ap_hint) > ap_hint > plain.

**Multi-period EMA averaging** (`MultiPeriodRequest`) mirrors
WSJT-X's `iavg=1`/`iavg=2` averaged decode from `q65_decode.f90` —
the strategy that lets ionoscatter and weak EME signals decode when
no single-period strategy above can. Takes `&[&[f32]]` (one buffer
per T/R slot) rather than a single audio buffer. It maintains an
exponential moving average of the per-slot spectrogram (time constant
`min(navg, 4)`) across consecutive T/R periods and, at each slot,
tries a 3-stage decode ladder against the averaged energies: (1)
AP-list, when `.ap_list()` was set; (2) fast-fading BP sweeping
`b90·Ts ∈ {3, 8, 15}` × `{Gaussian, Lorentzian}`; (3) plain Bessel BP
as a last-resort AWGN fallback (no separate `.fading()`/`.ap_hint()`
on this builder — the ladder always runs). Returns at most one decode
per slot, deduped by `(message, ±4 Hz freq)`. Not yet exposed via
`mfsk-ffi` — Rust API only as of writing.

The C ABI exposes four of the strategies above one-for-one as
`mfsk_q65_decode`, `mfsk_q65_decode_with_ap`, `mfsk_q65_decode_fading`
and `mfsk_q65_decode_with_ap_list`, each taking a `MfskQ65SubMode`
parameter so any of the ten sub-modes is reachable from C/C++/Kotlin;
`mfsk_q65_decode_fading` additionally takes an `MfskQ65FadingModel`
(`Gaussian` / `Lorentzian`) parameter (§8). Multi-period averaging is
not yet part of the C ABI.

Each of the four also takes an optional `hash_table` parameter
(issue #250) — an opaque `MfskCallsignHashTable*` handle
(`mfsk_callsign_hash_table_new`/`_insert`/`_free`) mirroring
`DecodeRequest::hash_table`'s `Arc<CallsignHashTable>`. NULL (the
pre-#250 default) leaves `<...>` Type-4 hashed-callsign placeholders
unresolved in the decoded message text; a populated table resolves
them to real callsigns registered via `_insert`. Purely a text-
rendering concern — it doesn't affect decode success or timing.

## 4. Shared primitives (`engine`)

### Receive pipeline — the engine functions

The receive flow diagrammed in **§0.3** — `coarse_sync` →
`refine_candidate` → `symbol_spectra` → `equalize_local` →
`compute_llr` → `P::Fec::decode_soft` → `P::Msg::unpack`, with a
`subtract` step feeding the next SIC iteration — is realised as a chain
of free functions in the `engine` submodules, each parameterised by
`P: Protocol`.

There is no `Demodulator` or `Receiver` trait. The receive path is
realised as free functions in `engine::sync`, `engine::llr`,
`engine::equalize`, `engine::pipeline`, each generic over `P: Protocol`.
Monomorphisation produces per-protocol code identical to a hand-
written decoder, without forcing every protocol to implement an
n-method receive interface. `engine::llr::compute_llr<P>` is the soft
demapper: it lives as a free fn rather than a `Protocol::demap()`
method because the spectral extraction (`symbol_spectra`), the four
WSJT-style LLR variants (a/b/c/d) and the equaliser feed into it as
data, not as trait composition. The same pattern applies to sync,
equalisation and the pipeline driver — all of them take the
protocol type as a parameter and read `P`'s associated constants
(`NTONES`, `NSPS`, `SYNC_MODE`, …) directly.

### The public decode entry point: `DecodeRequest` / `SniperRequest`

The engine functions in the §0.3 diagram (`coarse_sync`,
`decode_frame`, `process_candidate_basic`, …) are internal —
`pub(crate)` since issue #191/#203. Applications drive them through
two generic builders in `mfsk_core::msg::decode_request`, implemented
for `Ft8`, `Ft4`, and every FST4 sub-mode (the `FrameDecodable`
marker trait; Q65/WSPR/JT65/JT9/uvpacket keep their own bespoke entry
points, §6.3/§6.5):

* **`DecodeRequest<P>`** — wide-band search over `freq_min..freq_max`.
  `DecodeRequest::<P>::new(audio, freq_min, freq_max, sync_min,
  max_cand)`, then chain `.osd(bool)` (default `true`; toggles OSD
  fallback when the BP staircase fails — `LlrEffort` is always `Full`
  for host decodes, see the doc comment on `.osd` itself),
  `.strictness(...)`, `.eq_mode(...)`, `.known(...)` (skip/subtract
  already-decoded messages from an earlier pass), `.fft_cache(...)`
  (reuse a previous call's forward FFT), `.ap_hint(...)` where the
  protocol implements `SupportsWideBandAp` (FT8 only), and one of
  `.sic_rounds(n)` / `.sic_early()` to pick a
  successive-interference-cancellation strategy where the protocol
  supports it (`SupportsSicRounds`: FT8+FT4, `n` clamped 1..=3;
  `SupportsSicEarly`: FT8 only, fixed 3-checkpoint structure),
  `.on_result(cb)` for streaming delivery — see below; Q65/WSPR/
  JT65/JT9 get the same *pattern* through their own bespoke entry
  points (§10), not this trait — and `.budget(check)`, a
  caller-supplied wall-clock deadline predicate (see below). Call
  `.decode()` to get a
  `DecodeOutcome<P>` (`.results: Vec<P::DecodeResult>`, `.fft_cache`
  for a follow-up call, and `.budget: BudgetReport`).
* **`SniperRequest<P>`** — narrow-band, single-target search, gated on
  `SupportsSniper` and **implemented for FT8 alone**. **Read "Sniper mode
  is a roofing-filter mode" below before using it** — it is not a general
  "hunt one known station" convenience, and AP is not part of it.
  `DecodeRequest::<P>::sniper(audio, target_freq_hz, max_cand)` or
  `SniperRequest::<P>::new(...)` directly; `.search_hz(w)` to widen or
  narrow the default ±250 Hz window, `.osd(bool)`, `.strictness(...)`,
  `.eq_mode(...)`, `.ap_hint(...)` (no SIC variant — sniper mode is
  inherently single-candidate), and `.on_result(cb)` and
  `.budget(check)` (same as above). `.decode()` returns the same
  `DecodeOutcome<P>` shape.

This replaced FT8's `decode_frame*`/`decode_frame_subtract*`/
`decode_sniper*` family (15 public functions) and FT4/FST4's own
suffix-exploded equivalents — see §6.2/§6.4 for worked examples.

#### Sniper mode is a roofing-filter mode, and AP is not part of it

Three things about `SniperRequest` are repeatedly misread, including by
people working on this repository. Each has cost a real design decision,
so they are stated here rather than left in a doc comment.

**1. The ±250 Hz window is hardware, not cleverness.** Sniper mode is the
software half of narrowing the *analogue* roofing filter on a transceiver
that has one at that width — the Yaesu FTDX101MP and FTDX10 are the
usual examples. The operator selects a ~500 Hz analogue filter, points it
at a DX station whose carrier is already known, and the audio reaching
the decoder is already band-limited to that slice. Searching ±250 Hz is
the decoder matching what the hardware left in the passband. It is a
special-purpose path for specific radios, not the natural shape for
"any mode where you know who you are working".

`.eq_mode(EqMode::Local)` lives beside it for the same reason: an
analogue filter's skirt tilts the passband and local equalisation
flattens it again. EQ is therefore a property of the **input audio**,
which is why `DecodeRequest` carries it too — filtered audio can be fed
to a wide-band decode, and FT8's `eq_mode_recovers_bpf_edge_signal` pins
a band-pass-edge signal that decodes with `Local` and not with `Off`
through the wide-band SIC engine. On flat synthetic input EQ can only
cost: the `ft4sim`-generated FT4 golden has no receiver filter, and
`Local` loses two decodes of fourteen there. Default `Off` is right for
recordings and simulations and wrong for a narrowed receiver.

**2. It is FT8-only, and that is the design.** `SupportsSniper` is
implemented for `Ft8` and nothing else. The wide-band path is the main
path for every mode in this crate — if it is not WSJT-X-faithful without
a sniper, that is a bug in the wide-band path, not a reason to reach for
this one. FT4 is a contest protocol whose entire premise is working a
full band, so a roofing filter is against its purpose; FST4 narrows
through its own DDC channelizer (`docs/notes/FST4_DDC_DESIGN.md`), which
is the same benefit without a second decode engine. FT4 and FST4 sniper
entry points existed until 2026-09-13 and were removed; the C ABI
returns `MFSK_STATUS_UNKNOWN_PROTOCOL` for them.

**3. A-priori decoding is a general option that got coupled to it, and
no longer is.** AP locks high-confidence bits and lowers the threshold
by 1-3 dB. Nothing about it is narrow-band. But AP used to live in a
parallel engine (`msg::pipeline_ap::decode_sniper_ap`) that the sniper
was the only caller of, and that engine broke out of its candidate loop
on `if has_ap` — **the presence of a hint was what made the search
single-target**, not the width of the search. FT8 escaped only because
it has its own AP path and never entered that engine, which was the
whole content of `SupportsWideBandAp` being FT8-only.

Removing the coupling was not enough, and the measurement is worth
recording because "it is one line" was asserted twice here and was
wrong both times. Routing a wide-band FT4 decode through that engine
returned 4 decodes where the plain path returns 11, lost the hinted
station itself, and returned the *identical* set whether the hint named
a present or an absent station — so AP was changing nothing and the loss
was entirely the ladder: `process_candidate_ap` offered OSD at depth 2
with no depth-3/4 escalation and no Top-K rescue, and most of that
recording's decodes come from exactly those.

**The engine is now gone.** AP is a rung at the end of
`engine::pipeline::process_candidate_basic`'s own ladder — everything
above it has already run and failed, so it can only add decodes — and it
therefore reaches FT8, FT4 and every FST4 sub-mode. `msg::pipeline_ap`
is what remains: `ap_passes` (the hypothesis set, WSJT-X's `iaptype`
equivalents) and `ap_bits_for`, 96 lines with no engine of its own.

Two things were found while wiring it, both shipped:

* **FT4/FST4 AP was locking half its bits to the opposite of the
  truth.** Those protocols XOR the 77-bit message with their own RVEC
  before CRC and FEC (`ModulationParams::INFO_SCRAMBLE_RVEC`), so the
  info bits inside the codeword are the *scrambled* message, while an
  `ApHint` describes the message. `ap_bits_for` now scrambles. At
  −17/−18/−19 dB on 12 trials each: 3→12, 0→11, 0→8. FT8 has no RVEC
  and was never affected — which is why this survived, the protocol
  where AP is most used being the one where it worked.
  (`tests/ft4_ap_scramble.rs`.)
* **A blind-CQ hypothesis was missing** (upstream `iaptype 1`, locking
  the message type and the `CQ` prefix with no callsign knowledge). It
  is the pass that applies with no station knowledge at all, so it
  changes the *baseline* rather than a hinted case. FT4 AWGN threshold
  moved −16.89 → **−18.00 dB**, from 0.6 dB behind WSJT-X's published
  −17.5 dB to 0.5 dB ahead. The caveat is recorded in
  `docs/notes/FT4_BENCHMARK.md` §48: the sweep corpus is CQ-only, so
  this is the best case for that pass.

AP's risk is manufactured decodes, so wide-band AP ships with precision
guards in the same file as the gain assertion: a hint naming a station
that is not transmitting must not produce it, and a confident hint over
pure noise must decode nothing.

#### Compute budget: `.budget(check)`

`check: &(dyn Fn() -> bool + Sync)` returns `false` once the caller's
wall-clock allowance is spent, and the decode then starts no further
work; `DecodeOutcome::budget` reports what was left undone
(`exhausted`, `candidates_skipped`, `stages_run`, and the sync score of
the best candidate that was cut). Opt-in: without it, a decode behaves
exactly as it always has.

It is a closure, and `mfsk-core` holds no clock of its own, because
neither `wasm32-unknown-unknown` (`Instant::now` is unimplemented) nor
`no_std` can supply one — the caller passes host `Instant`, browser
`performance.now()`, or embedded `esp_timer_get_time`. `Sync`, not
`FnMut`, so one captured deadline can be shared across a `rayon` batch
rather than borrowed per candidate. This mirrors `wspr::decode`'s own
`budget` parameter.

**How the budget is spent, per strategy.** The default single-pass
strategy is *cheapest-first*: the cheap sync triage sweeps every
candidate (never budget-gated — it is what produces the ordering, and
it already rejects ~82 % of candidates before the expensive 58-symbol
DFT), the survivors are ordered by sync quality, and the ladder runs
down that order until the budget says stop. On `qso3_busy.wav` a budget
of 1/2/4/8 candidates therefore returns 1/2/4/8 *real* decodes rather
than whatever sits at the low end of the band, and 16 reaches the full
set. The SIC strategies poll only at candidate and round boundaries in
their existing order — each accepted decode is subtracted before the
next candidate is examined, so that order is the algorithm.

The triage sweep is a **floor**: never gated, so a budget shorter than
it returns nothing while still spending its time. Measured through
`bench/wasm` under Node on `qso3_busy.wav`, ~13 ms of a ~28 ms decode —
5 ms and 10 ms budgets return zero stations, 20 ms returns all 14.
`max_cand` is the knob for the floor; `.budget(..)` spends what is
above it.

Implemented for FT8, FT4 and every FST4 sub-mode, on every strategy.
The ranking key differs by protocol because each already computes a
different one for free: FT8 ranks by the Costas sync quality its triage
produces; FT4 needs no reordering (`ft4_coarse_sync` already returns
candidates ranked by score); FST4 ranks by the refined
`fst4_sync_search` score that `dedup_refined_candidates` computed for
every candidate. FT4's `.sic_rounds(n)` declines whole *rounds* — that
engine subtracts a round's decodes as one batch, so a round is the
granularity it can offer. The plain `decode_block` FT8 API (§4, what the
embedded boards call) is unaffected.

#### Streaming delivery: `.on_result(cb)`

Originated on FT8 (`DecodeRequest<Ft8>`/`SniperRequest<Ft8>`, plus the
`ft8::decode_block::decode_block_streaming` sibling to `decode_block` —
available under both the embedded and host `fft-rustfft` feature
gates since issue #243) — `cb: &(dyn Fn(&P::DecodeResult) + Sync)` fires once
per candidate as it's accepted, *alongside* (not instead of)
`decode()`'s own returned `DecodeOutcome`. Purely additive: the batch
API is unchanged, callers who don't call `.on_result()` see zero
behavioural difference.

The same conceptual API — a synchronous callback firing once per
accepted result, additive alongside the batch return — now also
covers Q65, WSPR, JT65, and JT9, each through its own bespoke entry
point rather than this trait (§0.5/§10 explain why those four don't
implement `FrameDecodable`): Q65's `DecodeRequest`/`SniperRequest`/
`MultiPeriodRequest` builders gained the same `.on_result(cb)` method;
WSPR/JT65/JT9 have no builder, so each gained a `decode_scan_streaming`
(and, for WSPR, `decode_scan_subtract_streaming`) sibling next to its
existing `decode_scan`, matching `decode_block_streaming`'s own
sibling-not-a-parameter precedent. No shared callback *type* exists
across protocols — `Q65Result`/`WsprResult`/`Jt65Result`/`Jt9Result`
are structurally distinct structs — so this is a consistent *pattern*
to code against (same method/function name and semantics per
protocol's own API family), not a single trait to abstract over. See
§10's per-protocol notes for exactly which function/builder to call.

**Delivery order and dedup contract differs by strategy** — see
`DecodeRequest::on_result`'s own doc comment for the authoritative
version, summarised here: on the sequential SIC strategies
(`.sic_rounds()`/`.sic_early()`) and `decode_block_streaming` (both
feature gates), `cb` fires exactly once per result that ends
up in the returned `Vec`, in the same order — zero divergence. On the
default single-pass strategy and `SniperRequest` (both parallelized
via `rayon` under `feature = "parallel"`), `cb` fires from whichever
thread decoded that candidate, in completion order, and *before* the
final cross-candidate dedup pass — on the rare occasion two different
sync candidates converge on the same message, `cb` may fire for both
even though only one survives into the returned `Vec`.

The same two contracts, not a third: Q65's scan builders, JT65's and
JT9's `decode_scan_streaming` are all sequential-exhaustive candidate
loops with no parallelism — exact-match, same as FT8's SIC strategies.
WSPR's `decode_scan_streaming` runs both its coarse-search passes
under `rayon::par_iter()` — same completion-order/possible-duplicate
caveat as FT8's default strategy; `decode_scan_subtract_streaming`
fires only at its own outer SIC-pass accept point (sequential,
exact-match), not inside the per-pass `decode_scan` calls it makes
internally. Q65's `MultiPeriodRequest::on_result` is a variant of the
sequential shape: it fires once per **slot** that yields an accepted
decode (its own natural streaming unit for multi-period EME/
ionoscatter averaging), rather than once per candidate.

**Does delivery order favour strong signals?** Tends to, on *both*
strategy families — `ft8::decode_block::coarse_sync` returns
candidates sorted by Costas sync score descending, and both the
sequential SIC loop and the parallel sweep process/dispatch that list
in-order, so higher-scored (typically stronger) candidates tend to
surface first either way (sync score correlates with SNR). But it's a
correlation, not a guarantee: sync score is a pre-demod
correlation-power measurement, not a direct predictor of post-demod
BP/OSD cost, so a highly-scored candidate can still need the full OSD
escalation while a lower-scored one converges in one BP pass. On the
**sequential** strategies specifically this has a real consequence the
parallel ones don't share — a candidate ahead in the list that needs
deep OSD blocks every candidate behind it (including ones that would
individually decode in microseconds), since there's only one thread;
the parallel strategies have no such blocking, each candidate's cost
is independent of every other's.

**Design decision: a synchronous callback, not `async`/a channel —
portability first.** §0.2's stated goal is a single crate "consumed
identically from several runtimes (native Rust, WebAssembly, Android
JNI, C ABI)," broadening the set of platforms that can host it — an
`async fn`/`Future`-based API (or anything requiring a channel/executor
by default) would need `std` and a runtime everywhere in the call
graph, which breaks the no_std embedded targets
(`embedded-poc/m5stack-*-app`) that are first-class consumers of this
exact decode path. `process_candidates_with_ap`'s existing fill-closure
parameter already established the plain-callback idiom for this
codebase (`F: FnMut(&mut [[Cmplx<f32>;8];79], &SyncCandidate,
SymMask)`) — `.on_result()` follows the same shape rather than
introducing a second, async-flavoured pattern next to it.

This also means `mfsk-core` never needs to know *how* a caller wants
results delivered across a thread — a host application wanting to feed
a GUI wraps the callback itself (e.g. `Sender::send` inside the
closure body) rather than the crate baking in a channel type. This
mirrors §0.3's "generic core + per-protocol plug-ins" framing: `engine`
and the protocol modules stay ignorant of their caller's runtime
environment, and cross-slot-boundary background continuation (the
WSJT-X "Fast"-mode shape — decoding keeps running after the next
slot's capture has already started) is achievable today purely at the
application layer (`std::thread::spawn` + call `.decode()`/
`decode_block_streaming` there) without any core-library support for
it.

> **Dedicated guide:** [STREAMING.md](STREAMING.md)
> ([日本語](STREAMING.ja.md)) collects the per-protocol streaming
> entry points, both delivery contracts, the full "why a synchronous
> callback, not `async`/Tokio/a channel" rationale, and a complete
> worked example of bridging `.on_result` into a Tokio async client
> (`spawn_blocking` + `mpsc` + optional `Stream` adaptor).

`DecodeDepth` (`llr_effort`/`osd`) itself is still a real type — it's
what `decode_block`/`decode_block_into` (the embedded/host-shared
plain-function FT8 API, §10) take positionally — but `DecodeRequest`/
`SniperRequest` no longer expose it directly: no host caller has ever
needed `LlrEffort::Minimal` (that variant exists solely for
`decode_block_into`'s ESP32 power budget), so the builders hardcode
`Full` and only surface the `osd` toggle.

**`WsjtxDepth`** (`mfsk_core::ft8::decode::WsjtxDepth`,
`DecodeRequest::<Ft8>::wsjtx_depth(...)`) bundles `.osd(...)` +
`.sic_rounds(n)`/`.sic_early()` + `.ap_hint()` into three named tiers
(`D1`/`D2`/`D3`) mirroring real WSJT-X's `jt9 -d 1/2/3` CLI flag, for
benchmarking against a real `jt9` build — see the type's own doc
comment for the exact tier→builder-method mapping and its known
limitations (OSD-strength doesn't exactly match jt9's).

### DSP (`mfsk_core::engine::dsp`)

| Module           | Purpose                                                     |
|------------------|-------------------------------------------------------------|
| `resample`       | linear resampler to 12 kHz                                  |
| `downsample`     | FFT-based complex decimation (`DownsampleCfg`)              |
| `gfsk`           | GFSK tone-to-PCM synthesiser (`GfskCfg`)                    |
| `subtract`       | phase-continuous least-squares SIC (`SubtractCfg`)          |

Each takes a runtime `*Cfg` struct (not `<P>`) because the tuning
parameters include composite-FFT sizes that are not trivially derived
from trait constants alone. Protocol modules expose module-level
constants for each — `ft8::downsample::FT8_CFG`,
`ft4::decode::FT4_DOWNSAMPLE`, etc.

### Sync (`mfsk_core::engine::sync`)

* `coarse_sync::<P>(audio, freq_min, freq_max, …)` — UTC-aligned 2D
  peak search over `P::SYNC_MODE.blocks()` for non-FT8 protocols.
* `refine_candidate::<P>(cd0, cand, search_steps)` — integer-sample
  scan + parabolic sub-sample interpolation.
* `make_costas_ref(pattern, ds_spb)` / `score_costas_block(...)` — raw
  correlation helpers exposed for diagnostics and custom pipelines.

> **FT8 routes through `decode_block::coarse_sync` exclusively.**
> As of 0.6.0 the FT8 host pipeline uses
> `mfsk_core::ft8::decode_block::coarse_sync` (graduated to public
> API alongside `compute_spectrogram`) — the older
> `ft8::sync::coarse_sync` thin wrapper has been removed. Calling
> `engine::sync::coarse_sync::<Ft8>` is still the right path for
> hand-rolled non-default usage but `DecodeRequest::<Ft8>`/
> `SniperRequest::<Ft8>` (§4) dispatch via `decode_block::coarse_sync`
> internally. See §10 for the FT8-specific notes.

### Sync2D — FT4 / FST4 full-slot coherent sync search (`mfsk_core::engine::sync2d`)

Two protocol-specific full-slot coherent searches live here, both
ported from WSJT-X and both scored via a **phase-continuous** Costas
reference (`make_costas_ref_continuous`, phase accumulating across the
whole 8-symbol block instead of resetting per symbol) with
`score_flat_coherent` (one coherent inner product, amplitude `|z|`) —
~3 dB better sync-score SNR discrimination than a non-coherent
`Σ|z_k|²` power-sum:

* `ft4_sync_search::<P>(cd0, candidate)` / the windowed variant
  `ft4_sync_search_window::<P>(cd0, candidate, ib_min, ib_max)` —
  **FT4 only**; a coherent full-slot Δt search (`ft4_decode.f90`'s
  `isync=1`/`isync=2` loop, `sync4d.f90` scorer) over the slot's
  downsampled-sample range rather than a local window around the
  coarse-sync candidate's own (frequently-wrong) Δt estimate.
* `fst4_sync_search::<P>(cd0, cand)` — FST4-specific two-stage
  full-slot search (`fst4_decode.f90:657-925`): a coarse pass over
  the entire T/R slot (±1.5 s, step 4, ±12 steps of 0.1·baud) then a
  fine pass (±7 steps of 0.02·baud × ±4 samples). Closed FST4's AWGN
  sensitivity gap vs WSJT-X's published thresholds to ~0.3 dB
  (issue #146).

Both superseded a shared local (Δf, Δt) refine (`sync2d_refine` /
`Sync2dConfig`) that has since been **removed** (2026-07-20, no call
sites left) once FT4 (issue #72) and FST4 (issue #146) each needed a
full-slot search instead — a local window anchored on the coarse-sync
candidate's position couldn't recover cases where that non-coherent Δt
estimate was wrong by more than the window's own radius.

`engine::sync::coarse_sync::<P>` also gained an FST4-only augmentation
in the same pass: a bin can enter the candidate list either via the
existing short-time Costas-grid threshold *or* by clearing a
full-slot non-coherent 4-tone power check modelled on WSJT-X's
`get_candidates_fst4` (baseline-normalised the same way as the
existing grid). Gated on `P::ID == ProtocolId::Fst4`, so FT8/FT4 are
byte-identical. It measured as a no-op on the narrow single-signal
AWGN sweep (the golden candidate was never actually at risk of being
dropped there) but is a real WSJT-X-faithful coverage improvement for
busy/wideband scans with many co-channel candidates competing for a
fixed-size list.

### LLR (`mfsk_core::engine::llr`)

* `symbol_spectra::<P>(cd0, i_start)` — per-symbol FFT bins (generic
  path; FT8 callers should prefer
  `ft8::decode_block::fill_symbol_spectra` which avoids the
  intermediate `cd0` allocation).
* `compute_llr::<P>(cs)` — four WSJT-style LLR variants (a/b/c/d),
  built from `nsym ∈ {1, 2, P::LLR_NSYM_MAX}` correlation-ladder
  hypotheses. `LLR_NSYM_MAX` defaults to 3 (FT8-calibrated); FT4
  overrides it to 4 and FST4 to 8 — both matching their own WSJT-X
  bit-metric code (`get_ft4_bitmetrics.f90` / `get_fst4_bitmetrics.f90`)
  rather than silently inheriting FT8's depth (FST4's override landed
  in 0.7.1, issue #146 — it had none before and was falling back to
  the FT8 default).
* `sync_quality::<P>(cs)` — hard-decision sync symbol count.

### Equalise (`mfsk_core::engine::equalize`)

* `equalize_local::<P>(cs)` — per-tone Wiener equaliser driven by
  `P::SYNC_MODE.blocks()` pilot observations; linearly extrapolates any tones
  that Costas doesn't visit.

### Pipeline (`mfsk_core::engine::pipeline`)

`decode_frame::<P>` (coarse sync → parallel process_candidate →
dedupe), `decode_frame_subtract::<P>` (3-pass SIC driver), and
`process_candidate_basic::<P>` (single-candidate BP+OSD) are the raw
engine functions underneath the pipeline — but as of issue #191/#203
they are **`pub(crate)`** (or `pub` only under the non-default
`internal-testing` feature, used by the crate's own test binaries).
Application code should not call them directly; use
`msg::decode_request::DecodeRequest`/`SniperRequest` instead (see
"The public decode entry point" above), which wrap these same
functions behind a builder. `decode_frame_subtract`
uses `subtract_signal_lpf` (WSJT-X-style channel-aware subtract) as of
0.6.2; the previous `subtract_signal_weighted` / `qsb_partial_gain`
path has been removed.

`DecodeStrictness` (`Strict`/`Normal`/`Deep`) exposes three methods
(a fourth, `osd_score_min()` — a pre-OSD coarse-sync-score gate — was
removed outright in issue #230: bypassed for both FST4 and FT4, it had
no live caller left on any protocol), with different live-per-protocol
scope — check which apply before assuming `.strictness(...)` does
anything for a given call:

* `osd_max_errors()` — post-OSD hard-error ceiling, `osd_depth`-tiered.
  **FT4-only in practice.** FST4-bypassed (`is_fst4` in
  `engine/pipeline.rs` — FST4 trusts CRC-24 alone, matching WSJT-X's
  own FST4 acceptance test `fst4_decode.f90:570`: `nharderrors >= 0 &&
  unpk77_success`, no such gate) but *live* for FT4, retuned against a
  real `ft4sim` AWGN/CCIR sweep (issue #72, 2026-07-18) — no longer a
  placeholder copy of an FT8 calibration. **Despite the name, FT8 has
  never actually called this method** — its own OSD dispatch used
  hardcoded constants instead (see `ft8_nharderrors_max` below); an
  earlier version of this doc incorrectly described it as
  FT8-calibrated.
* `ap_max_errors(locked_bits)` — AP-assisted decode hard-error
  ceiling, graded by locked-bit count. Live for FT8's per-candidate AP
  loop and the FT4/FST4 AP sniper (`msg::pipeline_ap`) alike —
  numerically unified across both call sites (issue #191).
* `ft8_nharderrors_max()` — FT8's own flat (not `osd_depth`-tiered)
  hard-error ceiling for the **non-AP** BP staircase and OSD fallback
  (`ft8::decode_block::process_candidates`/`osd_strategy`). Added
  issue #221: `.strictness(...)` was a documented no-op for FT8's
  non-AP path before this — hardcoded `36` (WSJT-X's own
  `ft8b.f90:422` ceiling) ran unconditionally, dead since issue #188
  removed the code that used to consume a strictness-tiered version.
  `Normal` still returns that same 36 (zero default-behavior change);
  `Strict`/`Deep` are new, live knobs — `Strict = 22` reuses real
  prior art from the issue #72 investigation, `Deep = 40` is
  exploratory and not yet swept against a fading corpus.

AP-aware variants live in `msg::pipeline_ap` because AP hint
construction is 77-bit specific.

### FT8 block-decoder entry points (`mfsk_core::ft8::decode_block`)

The FT8 module exposes a parallel set of entries on top of the
shared pipeline, sharing one `process_one_candidate_inner` body
between host and embedded callers (added in 0.6.1). All variants
operate on the same audio + spectrogram inputs and differ only in
which inner steps they enable:

* `decode_block` / `decode_block_tuned` — pass-1 BP only.
* `decode_block_with_ap` / `decode_block_with_ap_tuned` — pass-1 BP
  followed by the WSJT-X AP iaptype loop (1–12) for any candidate
  whose pass-1 step missed but whose sync quality crosses
  `q_thresh`. New in 0.6.1.
* `decode_block_into[_tuned]` — the embedded fixed-point entry point
  (`fixed-point` feature); same shape as `decode_block[_tuned]`, kept
  as a distinct name for API stability with `mfsk-ffi-ft8` and
  `embedded-shared::dual_core`. Prior to 0.8.0 this family also took
  caller-owned BASIS scratch buffers — removed (issue #162) once the
  Goertzel fill path made the scratch dead weight.
* `coarse_sync` / `coarse_sync_with_allsum` — the FT8 sync grid
  itself (graduated to public API in 0.6.0).
* `fill_symbol_spectra` / `fill_symbol_spectra_goertzel` — per-symbol
  FFT extraction directly from audio (replaces the cd0 +
  `engine::llr::symbol_spectra` two-step that older code used).

## 5. Feature flags

| Feature         | Default | Effect                                                        |
|-----------------|---------|---------------------------------------------------------------|
| `ft8`           | on      | FT8 ZST, decode, wave_gen                                    |
| `ft4`           | on      | FT4 ZST, decode                                              |
| `fst4`          | off     | FST4-15/30/60A/120/300 ZSTs, decode                           |
| `wspr`          | off     | WSPR ZST, decode, synth, spectrogram search                  |
| `jt9`           | off     | JT9 ZST, decode                                              |
| `jt65`          | off     | JT65 ZST, decode (+ erasure-aware RS)                        |
| `q65`           | off     | Q65-15A/30A + Q65-60A‥E + Q65-120D/E/300A ZSTs, five decode strategies (§3), synth |
| `msk144`        | off     | MSK144 — no `Protocol` ZST; own top-level driver (§0.5)       |
| `packet-bytes`  | off     | `PacketBytesMessage` — byte-payload example `MessageCodec`    |
| `uvpacket`      | off     | uvpacket — applied non-WSJT example, 4 sub-mode ZSTs (§10.1); pulls in `fst4` |
| `full`          | off     | Aggregate of all protocol features above                      |
| `parallel`      | on      | Enables rayon `par_iter` in pipeline (no-op under WASM)       |

## 6. Using from Rust

### 6.1 Dependencies

```toml
[dependencies]
mfsk-core = { version = "0.8", features = ["ft8", "ft4", "wspr"] }
```

Pull in only the protocol features you need; the examples below
enable several for illustration.

### 6.2 FT8 decode — minimal example

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
use mfsk_core::msg::decode_request::DecodeRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

// 1. Synthesise an FT8 frame and pad it into a 15-second slot.
let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1500.0, /* amp */ 20_000);

let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
for (i, &s) in frame.iter().enumerate() {
    if start + i < audio.len() { audio[start + i] = s; }
}

// 2. Decode it back. new(audio, freq_min, freq_max, sync_min, max_cand).
// OSD defaults to on; call `.osd(false)` for a cheaper BP-only decode.
let results = DecodeRequest::<Ft8>::new(&audio, 100.0, 3_000.0, 1.0, 50)
    .decode()
    .results;
for r in &results {
    if let Some(text) = unpack77(r.message77()) {
        println!("{:7.1} Hz  dt={:+.2} s  SNR={:+.0} dB  {}",
                 r.freq_hz, r.dt_sec, r.snr_db, text);
    }
}
```

### 6.3 WSPR — a separate demod path that still fits the abstraction

WSPR takes symbol-length FFTs directly at 12 kHz rather than
decimating to an FT-style baseband first, so its demodulation
pipeline is staged differently. The `wspr` module exposes its own
entry points. The FEC (`ConvFano`) and message codec
(`Wspr50Message`) are still declared as associated types on
`impl Protocol for Wspr`, so the trait surface remains consistent
— only the slot-level decoder differs.

```rust
# #[cfg(feature = "wspr")] {
use mfsk_core::wspr::decode::decode_scan_default;
use mfsk_core::wspr::tx::synthesize_type1;
use mfsk_core::msg::WsprMessage;

// Synthesise a WSPR Type 1 frame (120 s @ 12 kHz slot).
let samples_f32 = synthesize_type1("K1ABC", "FN42", 37, 12_000, 1500.0, 0.3)
    .expect("valid message");

let decodes = decode_scan_default(&samples_f32, /*sample_rate*/ 12_000);
assert!(!decodes.is_empty(), "roundtrip must decode");
for d in decodes {
    match d.message {
        WsprMessage::Type1 { callsign, grid, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  {} {} {}dBm", d.freq_hz, d.snr_db, callsign, grid, power_dbm);
        }
        WsprMessage::Type2 { callsign, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  {} {}dBm", d.freq_hz, d.snr_db, callsign, power_dbm);
        }
        WsprMessage::Type3 { callsign_hash, grid6, power_dbm } => {
            println!("{:7.2} Hz  {:+.0} dB  <#{:05x}> {} {}dBm",
                     d.freq_hz, d.snr_db, callsign_hash, grid6, power_dbm);
        }
    }
}
# }
```

`snr_db` is a wsprd-calibrated candidate SNR (dB, 2500 Hz reference)
carried through from the coarse search — the same figure wsprd
itself reports next to a spot.

`decode_scan_default` runs the (frequency × time) coarse search over
the whole slot internally. If the frequency and start sample are
already known, `wspr::decode::decode_at(samples, rate,
start_sample, freq_hz)` bypasses the scan.

### 6.4 Sniper mode + AP hint

Narrowing the search to ±250 Hz around a known target frequency and
supplying an a-priori hint lets the decoder recover weaker signals —
intended for use after a 500 Hz hardware BPF, or when hunting one
known station:

```rust
use mfsk_core::ft8::Ft8;
use mfsk_core::ft8::decode::{EqMode, ApHint};
use mfsk_core::ft8::wave_gen::{message_to_tones, tones_to_i16};
use mfsk_core::msg::decode_request::SniperRequest;
use mfsk_core::msg::wsjt77::{pack77, unpack77};

let msg77 = pack77("CQ", "JA1ABC", "PM95").unwrap();
let tones = message_to_tones(&msg77);
let frame = tones_to_i16(&tones, /* freq */ 1000.0, /* amp */ 20_000);
let mut audio = vec![0i16; 180_000]; // 15 s @ 12 kHz
let start = (0.5 * 12_000.0) as usize;
audio[start..start + frame.len()].copy_from_slice(&frame);

let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
let results = SniperRequest::<Ft8>::new(&audio, /*target_hz*/ 1000.0, /*max_cand*/ 15)
    .eq_mode(EqMode::Local)
    .ap_hint(&ap)
    .decode()
    .results;
assert!(!results.is_empty(), "roundtrip must decode");
for r in &results {
    let text = unpack77(r.message77()).unwrap();
    println!("{:7.1} Hz  {}", r.freq_hz, text);
}
```

`EqMode` has only `Off` / `Local` as of 0.7.0 — the earlier
`Adaptive` (try-EQ-then-non-EQ two-pass) variant was retired that
release once its measured payoff (~1/20 extra decodes at -18 dB)
stopped justifying the 2× per-candidate cost (issue #73). Callers
that want the old two-pass behaviour invoke the decoder twice
explicitly with `Local` then `Off`.

`SniperRequest::<Ft4>` works the same way (`FrameDecodable` is
implemented for both).

### 6.5 JT9 / JT65

Both JT9 and JT65 expose the same scan + point-decode pattern:

```rust
# #[cfg(feature = "jt65")] {
use mfsk_core::jt65::decode_scan_default;
use mfsk_core::jt65::tx::synthesize_standard;

let audio_f32 = synthesize_standard("CQ", "K1ABC", "FN42", 12_000, 1270.0, 0.3)
    .expect("pack + synth");
let decodes = decode_scan_default(&audio_f32, 12_000);
assert!(!decodes.is_empty(), "roundtrip must decode");
for d in decodes {
    println!("{:7.2} Hz  {:+.0} dB  {}", d.freq_hz, d.snr_db, d.message);
}
# }
```

JT65 additionally offers `decode_at_with_erasures` for low-SNR
signals where RS erasure decoding can recover frames that the
standard decoder misses, and — for deeper SNR still —
`decode_at_with_chase`/`decode_scan_chase*` (`jt65::chase`, issue
#169): a faithful port of WSJT-X's `ftrsdap` stochastic Chase decoder,
magic numbers included (erasure-probability table, `getpp`
spectral-power candidate ranking, literal acceptance-gate constants —
not just the algorithmic shape). Same call shape as the plain
`decode_scan` family, with an extra `&ChaseParams` argument. A second,
independent fix landed the same day: `search`/`rx` gained a fine
sub-bin frequency refinement + NCO correction that eliminates FFT
"scalloping loss" — this benefits *every* JT65 decode path, not just
`decode_at_with_chase` (`decode_at_with_erasures`'s own recall jumped
just as much, with zero code changes to that function). See `chase`'s
module doc for the chase algorithm and `docs/notes/BENCHMARKS.md`'s
JT65 section for the full measured story — together, the two fixes
closed the previously-documented ~7-8 dB gap essentially entirely on
this crate's AWGN corpus (with appropriate caveats on the WSJT-X
comparison methodology, spelled out there).

`Jt65Result::snr_db` and JT9's `Jt9Result::snr_db` are both
decode-side estimates from the per-symbol signal-tone vs. other-tones
power ratio; JT65's is converted to WSJT-X's 2500 Hz reference
bandwidth the same way Q65's is, but JT9's is **not** — its
multi-stage AGC/IFFT/coherent-sum pipeline doesn't reduce to a simple
bandwidth offset, so `Jt9Result::snr_db` is relative-only (compare JT9
decodes against each other, not against other protocols' `snr_db`).

## 7. Runtime registry & trait-surface verification

Two pieces of structural infrastructure make the library
self-describing and self-validating without any per-protocol
maintenance.

### 7.1 The `PROTOCOLS` registry

`mfsk_core::PROTOCOLS` is a `&'static [ProtocolMeta]` populated at
compile time from each `Protocol`-impl ZST's associated constants.
A consumer that wants to enumerate "what does this build support?"
no longer has to hardcode a list of its own:

```rust
use mfsk_core::PROTOCOLS;

for p in PROTOCOLS {
    println!(
        "{:10}  {:>3}-tone  {:>4} bits/sym  {:>5.1} s slot  ID={:?}",
        p.name, p.ntones, p.bits_per_symbol, p.t_slot_s, p.id,
    );
}
```

Each `ProtocolMeta` carries the protocol's `id` (`ProtocolId` enum,
family-level), display `name`, and every constant the trait surface
exposes — modulation (`ntones`, `bits_per_symbol`, `nsps`,
`symbol_dt`, `tone_spacing_hz`, `gfsk_bt`, `gfsk_hmod`), frame
(`n_data`, `n_sync`, `n_symbols`, `t_slot_s`), and codec
(`fec_k`, `fec_n`, `payload_bits`).

Lookup helpers:

* `mfsk_core::by_id(ProtocolId::Q65)` — yields *every* registry
  entry sharing the family-level id. Q65 yields ten (one per
  sub-mode); other protocols yield one.
* `mfsk_core::by_name("Q65-60D")` — exact-match name lookup.
* `mfsk_core::for_protocol_id(id)` — first entry sharing the id;
  convenient for the "single-mode-per-family" case.

Q65 is the case where the family / sub-mode distinction matters most:
all ten Q65 sub-modes share `ProtocolId::Q65` (the FFI tag is
family-level) but live as distinct registry entries because their
NSPS, tone spacing and slot length differ. FST4 is the same shape at
smaller scale — `by_id(ProtocolId::Fst4)` yields five (one per
T/R-period sub-mode).

The registry is built by an internal `protocol_meta!` macro in
`mfsk-core/src/registry.rs`; adding a new protocol is one line per
ZST plus its display name.

### 7.2 The generic trait-surface checker

`tests/protocol_invariants.rs` runs a single generic
`assert_protocol_invariants::<P: Protocol>(name)` against every
wired ZST. The body is the same for FT8, FT4, all five FST4
sub-modes, WSPR, JT9, JT65, all ten Q65 sub-modes, and all four
uvpacket sub-modes — 24 invocations, one implementation. Seventeen
invariants are pinned across three helper functions:

* **`assert_modulation_invariants<P: ModulationParams>`** —
  `2^BITS_PER_SYMBOL ≤ NTONES`; `SYMBOL_DT × 12000 == NSPS`;
  `TONE_SPACING_HZ`, `NDOWN`, `NSTEP_PER_SYMBOL`,
  `NFFT_PER_SYMBOL_FACTOR`, `GFSK_HMOD > 0`; `GFSK_BT ≥ 0`;
  `GRAY_MAP.len()` is in `[2^BITS_PER_SYMBOL, NTONES]`; map entries
  are unique and in range.
* **`assert_frame_layout_invariants<P>`** —
  `N_SYMBOLS == N_DATA + N_SYNC`; positive `T_SLOT_S`;
  non-negative `TX_START_OFFSET_S`. For `SyncMode::Block`, the
  sum of pattern lengths equals `N_SYNC` and every block fits
  inside the frame; for `SyncMode::Interleaved`, the sync vector
  length matches `N_SYMBOLS` and `sync_bit_pos < BITS_PER_SYMBOL`.
* **`assert_codec_consistency<P: Protocol>`** —
  `MessageCodec::PAYLOAD_BITS > 0`; `FecCodec::K > 0`;
  `FecCodec::N > K`; `FecCodec::K ≥ PAYLOAD_BITS` (the FEC
  budget holds the message); `FecCodec::N ≤ N_DATA × BITS_PER_SYMBOL`
  (the codeword fits in the channel symbols).

A second test cross-checks every registry entry against its ZST
through a *different* code path (lookup by name, then read trait
constants directly), so a typo inside the `protocol_meta!` macro
is caught even though it would pass `cargo build`.

This pinned the trait surface against silent drift while the Q65
work was landing — `GRAY_MAP`'s documented `len() == NTONES`
contract turned out not to hold for JT9 (which trims its map to
the eight data tones), and the test made the discrepancy visible
so the contract could be loosened to `[2^BITS_PER_SYMBOL, NTONES]`
without anyone having to remember to re-read the trait file.

Adding a new `Protocol` impl is now mechanical:

1. Implement the trait on a new ZST.
2. Add one line to `PROTOCOLS` in `registry.rs` via
   `protocol_meta!("Pretty-Name", MyProtocolZst)`.
3. Add a one-line `assert_protocol_invariants::<MyProtocolZst>(...)`
   to the corresponding test in `tests/protocol_invariants.rs`.

Any structural inconsistency surfaces in CI before the new
protocol's bespoke decode tests need to run.

## 8. C / C++ consumers via `mfsk-ffi`

### Artefacts

`cargo build -p mfsk-ffi --release` emits:

* `target/release/libmfsk.so`  (Linux / Android shared object)
* `target/release/libmfsk.a`   (static, for bundling)
* `mfsk-ffi/include/mfsk.h`    (cbindgen-generated, committed)

Every tagged release also attaches a prebuilt `linux-x86_64` tarball
of these artefacts to the GitHub Release — see `mfsk-ffi/README.md`.
Other platforms still need a local build; CI cross-compiles
Windows-GNU and Android arm64 on every source change, so they are
build-verified even though no binary is published for them yet.

### Linking, per platform

**`MFSK_API`** is emitted on every declaration. Define `MFSK_STATIC`
when linking `libmfsk.a`, `MFSK_BUILDING` when building the DLL itself,
and nothing when consuming the DLL. Without it a Windows DLL exports
nothing linkable, and a Unix shared object exports every non-static
symbol including Rust internals.

The calling convention is `extern "C"`'s own, which is `__cdecl` on
Windows for every signature here. It is documented rather than emitted:
cbindgen can place a prefix before the return type but not in the
`__cdecl` position between return type and name, and a macro that
cannot go where MSVC needs it would be worse than the default.

| platform | link line |
|---|---|
| Linux | `-lmfsk -lpthread -ldl -lm` |
| Android | `-lmfsk -llog -lm` (no `-ldl`/`-lpthread`; both are in Bionic's libc) |
| macOS | `-lmfsk -lpthread -lm` |
| Windows (MSVC) | `mfsk.dll.lib` plus `ws2_32.lib userenv.lib ntdll.lib bcrypt.lib` |
| Windows (GNU) | `-lmfsk -lws2_32 -luserenv -lntdll -lbcrypt` |

The repo's own single Unix `-ldl` line was wrong on all three of the
new targets.

**Android needs 16 KB page alignment.** Android 15 ships devices with a
16 KB kernel page size, and a `.so` linked for 4 KB does not load
there — surfacing as `UnsatisfiedLinkError` on exactly the newest
hardware. `.cargo/config.toml` sets
`-C link-arg=-Wl,-z,max-page-size=16384` for the three Android
triples, **and CI asserts the resulting `.so` carries it**, because
setting `RUSTFLAGS` in the environment *overrides* `target.*.rustflags`
rather than merging with it — so the flag is one workflow edit away
from being silently dropped.

### API

See `mfsk-ffi/include/mfsk.h` for the authoritative declarations.
Summary:

```c
enum MfskProtocol {
    MFSK_PROTOCOL_FT8     = 0,
    MFSK_PROTOCOL_FT4     = 1,
    MFSK_PROTOCOL_WSPR    = 2,
    MFSK_PROTOCOL_JT9     = 3,
    MFSK_PROTOCOL_JT65    = 4,
    MFSK_PROTOCOL_FST4S60 = 5,  // only FST4-60A is FFI-exposed; the
                                // other 4 wired FST4 sub-modes
                                // (15/30/120/300, §0.4) are Rust-API
                                // only as of writing
    MFSK_PROTOCOL_Q65A30  = 6,  // other Q65 sub-modes (60A..60E, 15A,
                                // 120D/E, 300A) and AP-hint / fading /
                                // AP-list strategies: dedicated
                                // mfsk_q65_* function family, see
                                // mfsk-ffi/include/mfsk.h
};

// Channel-spread fading model for mfsk_q65_decode_fading (§3).
enum MfskQ65FadingModel {
    MFSK_Q65_FADING_MODEL_GAUSSIAN   = 0,
    MFSK_Q65_FADING_MODEL_LORENTZIAN = 1,
};

uint32_t          mfsk_version(void);           // major<<16 | minor<<8 | patch
uint32_t          mfsk_abi_version(void);       // the boundary's own revision
MfskDecoder*      mfsk_decoder_new(MfskProtocol protocol);
void              mfsk_decoder_free(MfskDecoder* dec);

// `options` may be NULL to use this crate's per-protocol default
// search range / threshold / depth, or a handle from
// mfsk_decode_options_new(...) to override it uniformly.
MfskDecodeOptions* mfsk_decode_options_new(float freq_min_hz, float freq_max_hz,
                                  float sync_min, int max_cand,
                                  MfskDecodeDepth depth);

// Builder-parity setters (issue #162 follow-up, 0.9.0) — each mutates
// `opts` in place, one C call per DecodeRequest builder method.
// strictness/eq_mode/freq_hint apply to FT8/FT4/FST4-60A; sic_rounds
// applies to FT8/FT4; sic_early and ap_hint are FT8-only.
// sic_rounds/sic_early are mutually exclusive (last call wins).
MfskStatus mfsk_decode_options_set_strictness(MfskDecodeOptions* opts,
                                  MfskStrictness strictness);
MfskStatus mfsk_decode_options_set_eq_mode(MfskDecodeOptions* opts,
                                  MfskEqMode eq_mode);
MfskStatus mfsk_decode_options_set_freq_hint(MfskDecodeOptions* opts,
                                  float freq_hz);
MfskStatus mfsk_decode_options_set_sic_rounds(MfskDecodeOptions* opts,
                                  uint8_t rounds);       // clamped 1..=3
MfskStatus mfsk_decode_options_set_sic_early(MfskDecodeOptions* opts);
MfskStatus mfsk_decode_options_set_ap_hint(MfskDecodeOptions* opts,
                                  const char* call1, const char* call2,
                                  const char* grid, const char* report);

void              mfsk_decode_options_free(MfskDecodeOptions* opts);

MfskStatus        mfsk_decode_i16(MfskDecoder*, const int16_t* samples,
                                  size_t n, uint32_t sample_rate,
                                  const MfskDecodeOptions* options,
                                  MfskResultList* out);
MfskStatus        mfsk_decode_f32(MfskDecoder*, const float*,  size_t,
                                  uint32_t, const MfskDecodeOptions*,
                                  MfskResultList* out);

// Single-frequency-target decode (SniperRequest, §4 — issue #249).
// FT8 / FT4 / FST4-60A only; anything else returns
// MFSK_STATUS_UNKNOWN_PROTOCOL. `options` may be NULL.
//
// The reason to reach for this on FT4/FST4 is the AP hint:
// mfsk_decode_options_set_ap_hint reaches the *wide-band* decoder for
// FT8 only (SupportsWideBandAp is not implemented for the other two),
// while the sniper path takes a hint for all three. sync_min,
// max_cand, depth, strictness, eq_mode and ap_hint apply;
// freq_min_hz/freq_max_hz, freq_hint and sic_rounds/sic_early do not
// (the target frequency is the hint, and there is no SIC strategy) and
// are ignored rather than rejected.
MfskStatus        mfsk_decode_i16_sniper(MfskDecoder*, const int16_t* samples,
                                  size_t n, uint32_t sample_rate,
                                  float target_freq_hz,
                                  const MfskDecodeOptions* options,
                                  MfskResultList* out);
MfskStatus        mfsk_decode_f32_sniper(MfskDecoder*, const float*, size_t,
                                  uint32_t, float target_freq_hz,
                                  const MfskDecodeOptions*,
                                  MfskResultList* out);

MfskStatus        mfsk_encode_ft8(const char* call1, const char* call2,
                                  const char* report, float freq_hz,
                                  MfskSamples* out);
MfskStatus        mfsk_encode_ft4(...);      // same shape
MfskStatus        mfsk_encode_fst4s60(...);  // same shape
MfskStatus        mfsk_encode_wspr(const char* call, const char* grid,
                                   int32_t power_dbm, float freq_hz,
                                   MfskSamples* out);
MfskStatus        mfsk_encode_jt9(...);      // same shape as ft8
MfskStatus        mfsk_encode_jt65(...);     // same shape as ft8

void              mfsk_result_list_free(MfskResultList* list);
void              mfsk_samples_free(MfskSamples* s);
const char*       mfsk_last_error(void);
```

`MfskResultList` is caller-owned storage filled by the decode call.
Each `MfskResult::text` is a fixed inline buffer (not a heap
pointer) — the whole list is one allocation, freed in one call via
`mfsk_result_list_free`. (Before issue #205, `text` was a heap
`CString*` freed per message; `mfsk-ffi-ft8` always used the fixed
buffer, and `mfsk-ffi` adopted it too so both crates share one ABI
shape.)

`MfskSamples` is caller-owned storage filled by encode calls; it
holds 12 kHz f32 PCM and is freed by `mfsk_samples_free`.

See `mfsk-ffi/examples/cpp_smoke/` for a minimal end-to-end demo.

### Memory rules

1. **Handles**: allocate with `mfsk_decoder_new`, free with
   `mfsk_decoder_free`. One handle per thread. Free is idempotent on
   NULL.
2. **Result lists**: zero-initialise a `MfskResultList` on the
   stack, pass its address to the decode call, free with
   `mfsk_result_list_free` when done reading. `text` is a fixed
   inline buffer — no individual pointers to free.
3. **Sample buffers**: zero-initialise `MfskSamples`, pass to
   encode, free with `mfsk_samples_free`.
4. **Decode options**: an optional `MfskDecodeOptions*` handle from
   `mfsk_decode_options_new`, released with
   `mfsk_decode_options_free`. NULL is always valid (uses the
   protocol's built-in default). Since 0.9.0, the six
   `mfsk_decode_options_set_*` functions mutate the handle in place
   before it's passed to a decode call — the C-side mirror of
   `DecodeRequest`'s builder chain (§4). Not yet mirrored: `.known()`
   (#247), `SniperRequest` exposure (#249). Q65's `.hash_table()`
   shipped separately (#250, §3) — a different opaque handle
   (`MfskCallsignHashTable*`), since Q65's function family doesn't use
   `MfskDecodeOptions` at all.
5. **Errors**: on non-zero `MfskStatus`, call `mfsk_last_error` on the
   **same thread** to retrieve a human-readable diagnostic. The
   returned pointer is valid until the next fallible call on that
   thread.

### Thread safety

* An `MfskDecoder` is `!Sync`: one handle per concurrent thread.
* The decoder uses thread-local state for caching and error reporting,
  so spawning multiple threads each with its own handle is cheap.

### The host decides how threads are used

```c
MfskStatus mfsk_runtime_configure(const MfskRuntimeConfig* cfg);
uint32_t   mfsk_runtime_thread_count(void);
```

Even with `parallel` on, decoding used rayon's **global** pool:
`num_cpus` threads with 2 MiB stacks, spawned lazily on the first decode
and never joined. On Android those threads are not attached to ART, so a
callback from one cannot touch a JNIEnv; on iOS they sit outside GCD's
quality-of-service classes, competing with the audio render thread; on
both they keep running after the app is backgrounded. **There was no
hook to change any of that, at any layer.**

`on_thread_start` / `on_thread_stop` map straight onto rayon's
`start_handler` / `exit_handler`, which is what makes
`AttachCurrentThread` / `DetachCurrentThread` possible from JNI — and
therefore what makes a decode callback legal from a worker thread there.
`num_threads = 1` forces serial decoding, which is also what a build
without `parallel` does.

Call it once, before the first decode. A second call returns
`MFSK_STATUS_UNSUPPORTED` rather than being silently ignored: rayon
cannot rebuild a pool its threads may be parked in.

### Transmit is the same shape: nothing crosses as an allocation

Decode rows go into an array you own, and transmit is three stages —
`mfsk_pack77*` → `mfsk_message_to_tones` → `mfsk_tones_to_i16`/`_f32` —
each writing into a buffer you sized with `mfsk_symbol_count` and
`mfsk_synth_output_len`. **There is no pointer to free**, which deletes
the category that makes Kotlin and Swift wrappers leak when an exception
unwinds between the call and the free.

Before v2, seven `mfsk_encode_*` functions returned heap buffers and
accepted only the three-string `call1 call2 report` path, so a caller
with a type-4 or free-text message had no way in, and FST4 reached 60A
alone.

**Ask for the size rather than baking it.** The five FST4 sub-modes
differ by a factor of 30 in samples per symbol (720 → 21 504), so a
constant taken from 60A is silently wrong for the other four.

### Streaming ingestion: time is a parameter, never read

```c
MfskStream* mfsk_stream_open(uint32_t mode, uint32_t sample_rate, MfskStatus* out);
MfskStatus  mfsk_stream_push_i16(MfskStream*, const int16_t*, size_t);
void        mfsk_stream_set_epoch(MfskStream*, double utc_seconds_of_next_sample);
bool        mfsk_stream_slot_ready(const MfskStream*);
MfskStatus  mfsk_session_decode_stream(MfskDecodeSession*, MfskStream*, ...);
```

The ring is sized from `slot_samples_12k`, so FST4-300's 3.6 M-sample
slot works the same way FT4's 90 000-sample one does — `mfsk-ffi-ft8`'s
front end was FT8-sized and i16-only.

**No `Instant`, no `SystemTime`, no clock of any kind.** The host says
what UTC second the next sample belongs to and the grid does arithmetic.
That is what keeps this usable from wasm, from `no_std`, and from a
phone that was backgrounded for four minutes — the same choice
`BudgetCheck` makes for the decode deadline. Without an epoch the grid
free-runs from the first sample, which is exactly right for replaying a
recording.

`mfsk_session_decode_stream` is fused because taking FST4-300's slot out
and handing it back in moves 7 MB for nothing.

### Introspection: ask the library, don't hardcode a matrix

Until 2026-09-13 `mfsk_version()` was the entire introspection surface,
while `registry::PROTOCOLS` — which knows every wired mode with full
geometry — was not exposed at all. So a C consumer that needed to know
what a mode supports had one option: hardcode the matrix and let it rot.
The `MfskProtocol` enum above is that rot made visible — it has **one**
FST4 entry, so four of the five wired FST4 sub-modes were unreachable
from C for decode and encode alike.

```c
uint32_t    mfsk_mode_count(void);                    /* modes in THIS build */
MfskStatus  mfsk_mode_at(uint32_t index, MfskMode* out);
const char* mfsk_mode_name(MfskMode mode);            /* static, do not free */
MfskStatus  mfsk_mode_from_name(const char* name, MfskMode* out);
MfskStatus  mfsk_mode_info(MfskMode mode, MfskModeInfo* out);
uint64_t    mfsk_mode_caps(MfskMode mode);            /* MFSK_CAP_* bits */
MfskStatus  mfsk_mode_defaults(MfskMode mode, MfskDecodeDefaults* out);
```

**`MfskMode` addresses every mode, and its discriminants are ABI.** One
per registry entry plus MSK144, assigned once and never reordered — they
are deliberately *not* registry indices, because registry membership is
feature-gated and a build without `q65` would shift every index after
it. `mfsk_mode_count` / `mfsk_mode_at` say which of them this particular
build actually has. This ABI already learned the lesson once, in
`MfskQ65SubMode`; the cost of relearning it is silent misdispatch.

**Capabilities are published, not inferred.** `MFSK_CAP_DECODE_HANDLE`
is the load-bearing one: it says whether `mfsk_decode_i16` and friends
apply at all. Q65 takes a nominal start sample and a time tolerance and
reports `start_sample` rather than `dt`; WSPR/JT9/JT65 have no builder.
They are not lesser, they are shaped differently, and that is now a bit
a caller can read rather than a fact it has to know.

The bits mirror `mfsk_core::registry::caps`, which
`mfsk-core/tests/registry_caps.rs` ties to the trait impls in both
directions — naming a protocol that lacks a trait is a *compile* error
there, and implementing one without setting the bit is a runtime
failure. `mfsk-ffi/tests/mode_introspection.rs` closes the last link by
comparing each `MFSK_CAP_*` against the registry constant it mirrors.
That chain exists because a hand-written capability table lies within
two releases.

**`mfsk_mode_defaults` removes the ABI's worst trap.** Three different
per-protocol NULL-option defaults used to live inside one function, one
of them an FT8 `sync_min` of 2.0 that no test in the tree uses. Defaults
are data now — and `MfskDecodeDefaults::sync_scale` says whether two
modes' numbers are even comparable:

```c
MfskDecodeDefaults d = {0};
d.size = sizeof d;
mfsk_mode_defaults(MFSK_MODE_FT4, &d);
/* d.sync_min == 1.2, d.sync_scale == MFSK_SYNC_SCALE_BASELINE_NORMALISED */
```

FT4's spectrum is divided by a fitted baseline before scoring, so noise
sits at ~1.0 **by construction** and WSJT-X's own 1.2
(`ft4_decode.f90:195`) is a floor rather than a preference. FT8's and
FST4's are absolute Costas scores. Copying one across modes is wrong,
and before this field nothing said so.

**`MfskModeInfo::decode_fft1_size` is the field to read before
budgeting.** It is the forward FFT the decoder takes over the whole
slot: FT4 92 160 points, FST4-300 **4 194 304** — a factor of 45 that no
other field hints at, and the reason "one call shape for every mode" is
wrong as a memory story on a phone.

**Size versioning.** `MfskModeInfo` and `MfskDecodeDefaults` both lead
with `size`. Set it to your `sizeof` (or zero the struct and the library
fills it in); a library newer than your header writes only the prefix
you declared and rewrites `size` to what it actually wrote.
`MfskResult` grew `snr_db` in 0.8.1 with nothing marking it, and that
must not be repeatable.

`mfsk_abi_version()` is separate from `mfsk_version()` for the same
reason: the crate version moves for reasons that have nothing to do with
the boundary.

## 9. Kotlin / Android consumers

`bindings/kotlin/` is a maintained binding, built and run on a desktop
JVM by CI on every source change. It replaces the old
`mfsk-ffi/examples/kotlin_jni/` scaffold, which was written against the
pre-v2 ABI, marshalled results as pipe-separated strings, and was never
built by anything.

```kotlin
import io.github.mfskcore.*

// Ask the build what it has rather than hardcoding a list.
val ft8 = Mfsk.modes().first { Mfsk.modeName(it) == "FT8" }

// On Android, call this once before the first decode — see below.
Mfsk.configureRuntime(threads = 2)

MfskSession.open(ft8).use { s ->
    for (r in s.decode(pcm, sampleRate = 12_000)) {
        Log.i("ft8", "${r.freqHz} Hz  ${r.snrDb} dB  ${r.text}")
    }
}
```

**Shape.** `Mfsk` holds introspection and transmit; `MfskSession` is the
decode handle and is `AutoCloseable`, so `.use { }` releases it.
`MfskDecode` is a `data class` — a value, not a handle, because the ABI
writes rows into memory the caller owns. There is nothing to free and
nothing that can outlive a session.

**`Mfsk.configureRuntime` is the Android-specific part.** Without it the
decode runs on rayon's global pool, whose threads are plain pthreads the
VM has never attached — so nothing running on one can touch a JNIEnv,
and a decode callback from a worker thread is not merely discouraged but
illegal. The shim's thread hooks call `AttachCurrentThread` and
`DetachCurrentThread`, which is what makes that legal. It also takes the
pool off `num_cpus` × 2 MiB stacks that never join.

**A session is single-threaded.** It owns a callsign hash table it
mutates on every decode. One per thread; concurrent decodes on separate
sessions are supported.

**The shim is C, not Rust-with-`jni`, on purpose.** It `#include`s the
generated `mfsk.h`, so building it is another compiler reading that
header as a real translation unit. That has already earned its keep:
writing it found `MFSK_DECODE_FLAG_HASH_RESOLVED` absent from the
header, because the constant lived in a dependency cbindgen cannot emit
from. A Rust shim would link against the crate and see none of that.

Build and test: `bindings/kotlin/build.sh` (needs `JAVA_HOME` and
`kotlinc`). For Android, build `libmfsk.so` with
`cargo ndk -t arm64-v8a build -p mfsk-ffi --release --no-default-features
--features mobile` and compile the shim with the NDK's clang against the
same header; `.cargo/config.toml` already carries the 16 KB page-size
link flag every Android 15 device needs.

**Swift is not here yet.** It needs a macOS runner, which this repo only
has at tag time, so writing it now would put unverified code on `main`.

## 10. Protocol notes

| Protocol   | Slot   | Tones | Symbols | Tone Δf    | FEC              | Msg   | Sync       | Status |
|------------|--------|-------|---------|------------|------------------|-------|------------|--------|
| FT8        | 15 s   | 8     | 79      | 6.25 Hz    | LDPC(174, 91)    | 77 b  | 3×Costas-7 | implemented |
| FT4        | 7.5 s  | 4     | 103     | 20.833 Hz  | LDPC(174, 91)    | 77 b  | 4×Costas-4 | implemented |
| FST4-15    | 15 s   | 4     | 160     | 16.667 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | implemented (fastest FST4, ≈-20.7 dB threshold) |
| FST4-30    | 30 s   | 4     | 160     | 7.143 Hz   | LDPC(240, 101)   | 77 b  | 5×Costas-8 | implemented (≈-24.2 dB threshold) |
| FST4-60A   | 60 s   | 4     | 160     | 3.0864 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | implemented (dominant terrestrial sub-mode, ≈-28.1 dB threshold) |
| FST4-120   | 120 s  | 4     | 160     | 1.4634 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | implemented (≈-31.3 dB threshold) |
| FST4-300   | 300 s  | 4     | 160     | 0.5580 Hz  | LDPC(240, 101)   | 77 b  | 5×Costas-8 | implemented (≈-35.3 dB threshold, deepest wired FST4) |
| WSPR       | 120 s  | 4     | 162     | 1.465 Hz   | conv r=½ K=32 + Fano | 50 b | per-symbol LSB (npr3) | implemented |
| JT9        | 60 s   | 9     | 85      | 1.736 Hz   | conv r=½ K=32 + Fano | 72 b  | 16 distributed | implemented |
| JT65       | 60 s   | 65    | 126     | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 distributed | implemented |
| Q65-15A    | 15 s   | 65    | 85      | 6.667 Hz   | QRA(15, 65) GF(2⁶) + CRC-12 | 77 b | 22 distributed | implemented |
| Q65-30A    | 30 s   | 65    | 85      | 3.333 Hz   | (same QRA codec) | 77 b  | (same)     | implemented |
| Q65-60A    | 60 s   | 65    | 85      | 1.667 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (6 m EME) |
| Q65-60B    | 60 s   | 65    | 85      | 3.333 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (70 cm / 23 cm EME) |
| Q65-60C    | 60 s   | 65    | 85      | 6.667 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (~3 GHz EME) |
| Q65-60D    | 60 s   | 65    | 85      | 13.33 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (5.7 / 10 GHz EME) |
| Q65-60E    | 60 s   | 65    | 85      | 26.67 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (24 GHz+, extreme spread) |
| Q65-120D   | 120 s  | 65    | 85      | 6.0 Hz     | (same QRA codec) | 77 b  | (same)     | implemented (10 GHz rainscatter/troposcatter) |
| Q65-120E   | 120 s  | 65    | 85      | 12.0 Hz    | (same QRA codec) | 77 b  | (same)     | implemented (6 m ionoscatter) |
| Q65-300A   | 300 s  | 65    | 85      | 0.289 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (optical scatter, deepest AWGN) |

The shared-vs-bespoke split for every protocol is the §0.5 table; the
notes below add only the protocol-specific facts that table can't hold.

- **FST4** — LDPC(240, 101) + 24-bit CRC (`fec::ldpc240_101`); the
  BP/OSD code is the same across LDPC sizes, so the new material is
  just the parity-check/generator tables and code dimensions. The five
  wired sub-modes (FST4-15/30/60A/120/300) differ only in `NSPS` /
  `SYMBOL_DT` / `TONE_SPACING_HZ` — plus `TX_START_OFFSET_S` for
  FST4-15 alone (0.5 s rather than 1.0 s into the slot) — and are
  emitted by the `fst4_submode!` macro, the same way `q65_submode!`
  emits Q65's. FST4-900 / FST4-1800 remain unwired (no user demand).
  FST4W — the WSPR-style one-way 50-bit beacon variant, LDPC(240, 74),
  periods 120/300/900/1800 s — is a separate message format and out of
  scope; see issue #23.
- **WSPR** — `ConvFano` ported from WSJT-X `lib/wsprd/fano.c`;
  `Wspr50Message` covers Types 1 / 2 / 3. The `wspr` module adds a
  quarter-symbol spectrogram to keep the 120-s-slot coarse search
  within a reasonable time budget. Streaming delivery (§4):
  `wspr::decode::{decode_scan_streaming, decode_scan_subtract_streaming}`.
- **JT9 / JT65** — JT9's `ConvFano232` differs from WSPR's `ConvFano`
  only in its 206-bit codeword framing; both feed the 72-bit
  `Jt72Codec`. JT65's `Rs63_12` (re-exported `fec::Rs63_12`) does
  erasure-aware decoding via Karn's Berlekamp-Massey. Streaming
  delivery (§4): `jt9::decode_scan_streaming` /
  `jt65::decode_scan_streaming`.
- **Q65** — QRA over GF(64) (`fec::qra::QraCode` + the code instance
  `fec::qra15_65_64::QRA15_65_64_IRR_E23`); the application layer adds
  a CRC-12 over 13 information symbols and punctures the two CRC
  symbols out of the 65-symbol codeword, leaving the 63 channel symbols
  transmitted. Ten sub-modes differ only in `NSPS` and tone spacing
  (×1…×16); all five decoder strategies (§3) share the one QRA codec.
  Streaming delivery (§4): `.on_result(cb)` on `q65::{DecodeRequest,
  SniperRequest, MultiPeriodRequest}`.

### 10.1 Scope boundary: `uvpacket` as an applied example

`uvpacket` is in-tree but **outside** the WSJT family. It is an
applied example of how the FEC infrastructure (`Ldpc240_101`, BP,
OSD-2/3) can be reused for protocols that share none of the WSJT
modulation, sync, message-codec, or slot conventions. Specifically
it is a four-mode packet protocol for narrow-FM voice channels
(HT/mobile, ~3 kHz audio passband) using single-carrier
**π/4-DQPSK + LMS equaliser** + RRC pulse, four 127-chip BPSK
m-sequence preamble variants (mode-encoded), differential
demodulation (no carrier-phase tracker), and a byte-pipe API.

Sharing with the WSJT family stops at the FEC mother code:

| Layer | WSJT family | uvpacket |
|---|---|---|
| Modulation | M-ary tone FSK / GFSK | single-carrier π/4-DQPSK + RRC |
| Demod | non-coherent symbol-power detect | LMS equaliser + 1-symbol differential |
| Slot | fixed 7.5 / 15 / 60 / 120 s | variable-length burst |
| Sync | tone-index Costas blocks | 4-variant 127-chip BPSK m-sequence (mode-encoded) |
| Message | structured (callsign + grid) | byte-pipe (`app_type` tag) |
| Pipeline | generic `mfsk-core` TX/RX | bespoke `uvpacket::{tx,rx}` |
| FEC | (mode-specific) | `Ldpc240_101` (shared with FST4) + dedicated unpunctured header block |

Because uvpacket bypasses the generic TX/RX pipeline, several of
its `ModulationParams` trait constants (`NTONES = 4`, `GFSK_BT`,
`TONE_SPACING_HZ`, `GFSK_HMOD`) are decorative — they exist to
satisfy the trait signature and the `protocol_invariants` test
without being consulted by `tx::encode` or `rx::decode_known_layout`.
This trade-off is documented at
[`mfsk-core/src/uvpacket/protocol.rs`](../../mfsk-core/src/uvpacket/protocol.rs)
and is the natural consequence of keeping a non-WSJT protocol
in-tree rather than spinning it out as a sibling crate.

Despite bypassing the real receive pipeline, uvpacket's four
sub-mode ZSTs (`UvRobust`, `UvStandard`, `UvUltraRobust`,
`UvExpress`) *do* implement `Protocol` and are wired into both the
`PROTOCOLS` registry (§7.1) and `tests/protocol_invariants.rs` —
the trait surface is satisfied for enumeration/invariant purposes
even though `tx::encode`/`rx::decode_known_layout` never read most
of it.

Where Q65 (§0.5) shows the trait surface absorbing a third FEC family
and ten sub-modes *without* bending, uvpacket shows where it naturally
peels away: FEC + DSP + channel-test infrastructure carry over, but the
generic TX/RX pipeline and the message-codec / AP-compat traits do not.
That peeling is a sign the trait surface is **right-sized for the WSJT
family**, not a missing generalisation — abstracting `SYNC_MODE` beyond
"Costas blocks or interleaved" to cover m-sequences, equaliser state
and RRC shaping would make the WSJT paths pay indirection for no
in-family benefit. See [`docs/reference/UVPACKET.md`](UVPACKET.md) §0
for the same view from the applied-example side.

For the full uvpacket design narrative, AX.25 / M17 / D-STAR / DMR
/ VARA comparison, and characterisation curves, see
[`docs/reference/UVPACKET.md`](UVPACKET.md). Representative WAV samples are
at `audio_samples/uvpacket/`.

## License

Library code is GPL-3.0-or-later, derived from WSJT-X reference
algorithms.
