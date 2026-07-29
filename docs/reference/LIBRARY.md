# mfsk-core — Library Architecture & API Reference

> **日本語版:** [LIBRARY.ja.md](LIBRARY.ja.md)

This document covers the mfsk-core library surface for embedders:
Rust crate consumers, C/C++ projects linking `libmfsk.so`, and
Kotlin/Android apps using the JNI scaffold.

For a quick-start overview (badges, dependency snippet, minimal
example) see [README.md](../../README.md). This document goes deeper
into *why* and *how*.

## 0. Introduction

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

### 0.3 Design approach

Protocol-independent algorithms — DSP, sync, LLR, the equaliser,
LDPC BP / OSD, Fano convolutional decoding, Reed-Solomon erasure
decoding, and the shared parts of the message codec — live in the
`engine`, `fec`, and `msg` modules. Each protocol is a comparatively
small zero-sized type (ZST) that declares its own constants and the
specific FEC / message codec it uses. The pipeline is driven through
`DecodeRequest::<P>` (§4), taking `P: Protocol` as a compile-time type
parameter so that monomorphisation produces specialised code per
protocol. The abstraction does not add runtime cost.

Some direct consequences of this approach:

- The same algorithm implementation runs under native Rust, WASM,
  Android, and C / C++.
- Improvements to a shared path (e.g. LDPC BP) automatically benefit
  every protocol that uses it.
- Adding a new protocol tends to keep the diff confined to that
  protocol's own code (see §2 for the concrete steps).
- The C ABI in `mfsk-ffi` branches only once via `match protocol_id`;
  past that point, the code is already specialised.

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
`Protocol` for it. See §0.6.

### 0.5 Checking that the design actually works — WSPR and Q65 as stress tests

FT8, FT4 and FST4 share so much (LDPC FEC, 77-bit messages, block
Costas sync) that their common code is unavoidable rather than a
test of the abstraction. **WSPR** and **Q65** each push the trait
surface along independent axes, and were absorbed without touching
the FT-family code paths.

#### WSPR — three orthogonal differences from the FT family

1. **Different FEC family** — convolutional (r=½, K=32) with Fano
   sequential decoding instead of LDPC. Added as
   `mfsk_core::fec::conv::ConvFano`.
2. **Different message length** — 50 bits instead of 77. Types 1, 2
   and 3 are implemented in `mfsk_core::msg::wspr::Wspr50Message`.
3. **Different sync structure** — the lower bit of every channel
   symbol carries one bit of a fixed 162-bit sync vector, so sync is
   not a block of Costas arrays. Captured by adding an `Interleaved`
   variant to `FrameLayout::SYNC_MODE`.

#### Q65 — a third FEC family plus a five-way decoder-strategy axis

Q65 came later and stresses the abstraction along axes WSPR did not
exercise:

1. **Yet another FEC family** — Q-ary repeat-accumulate codes over
   GF(64), running belief propagation on probability vectors via
   non-binary Walsh-Hadamard messages. Added as
   `mfsk_core::fec::qra` (the QRA codec) plus
   `mfsk_core::fec::qra15_65_64` (Q65's specific code instance).
2. **65-tone FSK with one reserved sync tone** — `NTONES = 65` while
   `BITS_PER_SYMBOL = 6`. The data alphabet is GF(64); tone 0 is
   the dedicated sync tone. This is the case that surfaced (and
   fixed) the trait-doc inconsistency for `GRAY_MAP` length —
   neither `== NTONES` nor `== 2^BITS_PER_SYMBOL` holds for every
   protocol uniformly, so the contract was loosened to
   `[2^BITS_PER_SYMBOL, NTONES]`.
3. **Ten sub-modes sharing one impl block** — Q65-15A / Q65-30A for
   terrestrial work, Q65-60A‥E for EME at 6 m / 70 cm / 23 cm
   / 5.7 GHz / 10 GHz / 24 GHz+, and Q65-120D / Q65-120E / Q65-300A
   for longer-period scatter modes. They differ only in NSPS and tone
   spacing. The `q65_submode!` macro emits the per-sub-mode ZSTs
   and their trait impls in one line each — no per-mode code
   duplication.
4. **Five parallel decoder strategies** — Q65 is the first protocol
   in the library where the receiver chain has multiple legitimate
   paths through the same FEC. They are listed in §3:
   plain AWGN BP, AP-hint BP, fast-fading metric, AP-list
   template matching, and multi-period EMA averaging. Each is a
   builder method combination on `DecodeRequest`/`SniperRequest`/
   `MultiPeriodRequest`, generic over the sub-mode ZST; the
   underlying FEC and message codec are shared.

Where WSPR proved that *FEC family + message width + sync mode*
could each be swapped independently, Q65 proves that adding a third
FEC family, ten sub-modes and (now) five parallel decoder strategies
all still fit inside the same `Protocol` super-trait without bespoke
plumbing. §3 expands on the decoder strategies; §7 covers the
`PROTOCOLS` registry that lets consumers enumerate every wired
protocol (24 ZSTs in total: 20 WSJT-family protocols/sub-modes plus
4 `uvpacket` sub-modes, §10.1) and the
`tests/protocol_invariants.rs` generic checker that holds the trait
contract honest.

### 0.6 MSK144 — the protocol that doesn't use the abstraction

Every protocol in §0.4 shares one property underneath their
differences: a coarse-sync → LLR → FEC pipeline running over one
fixed-length slot, with 0..N frames sitting at (or near) a known
nominal offset. **MSK144** (issue #25) breaks both halves of that
assumption at once, and rather than force a fit, `msk144::decode`
is its own top-level driver — it never touches `engine::pipeline`,
`ModulationParams`, or `FrameLayout`, and no ZST implements
`Protocol` for it at all. This is a step further than WSPR or Q65:
those still adopt the trait surface (§0.5) even where their real
decode logic bypasses the shared pipeline; MSK144 opts out of the
trait surface itself.

1. **Not M-ary FSK.** MSK144 is continuous-phase binary MSK,
   transmitted as offset-QPSK: bits map onto I/Q rails, each shaped
   with a half-sine pulse. `ModulationParams`'s tone-index /
   Gray-map / GFSK-shaping model has no useful mapping onto that
   waveform. The modulator and matched-filter demodulator live in
   `engine::dsp::msk` as plain functions, not a trait impl.
2. **Not a static slot.** Every other protocol assumes a frame sits
   at a known nominal offset inside one fixed-length slot buffer.
   MSK144 instead repeats its 864-sample (72 ms) frame continuously
   through the whole 15 s (or 30 s) T/R period — real transmissions
   loop the same content back-to-back for the entire period — and a
   receiver has to scan for wherever an ionized-trail ping happens to
   land. `msk144::spd::detect_burst_candidates` (a squared-signal
   two-tone spectral scan) plus `msk144::sync::msk144_sync` (a joint
   CFO/symbol-timing matched-filter correlation) do that scanning,
   ported from WSJT-X's `msk144spd.f90`/`msk144sync.f90` — not from
   `engine::sync::coarse_sync`.

What it *does* share with the rest of the library: the 77-bit
`pack77`/`unpack77` message payload (`msg::wsjt77`, completely
unchanged — MSK144's real WSJT-X encoder calls the same function
FT8/FT4/FST4 do), and the generic LDPC belief-propagation/OSD engine
(`fec::ldpc::bp`/`osd`), parameterised by a fourth `LdpcParams` impl
for LDPC(128, 90) + CRC-13 (`fec::ldpc_128_90`) — added exactly the
way `Ldpc240_101Params` was added for FST4, per that trait's own
documented extension recipe. So the FEC and message layers extend
the same way every prior addition has; only the modulation and
frame-timing layers opt out, because there's nothing in
`ModulationParams`/`FrameLayout` for a transient-burst, non-FSK
protocol to plug into.

Golden-WAV recall against both WSJT-X `samples/MSK144/*.wav`
recordings is 3/3 (`tests/msk144_wsjtx_samples.rs`) — the divergent
architecture doesn't cost recall.

## 1. Module layout

```text
mfsk_core
├── engine/           Protocol traits, DSP, sync, LLR, equaliser, pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract · msk · analytic
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (Wiener per-tone)
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
├── msk144/           MSK144 — no Protocol impl; own top-level driver (§0.6)
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

The `mfsk-ffi` sibling crate in this workspace builds a C ABI
shared library (`libmfsk.{so,a,dylib}` + `mfsk.h`) on top of the
same crate.

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

## 4. Shared primitives (`engine`)

### Receive pipeline at a glance

The complete receive flow for any wired protocol — from raw audio
samples to decoded message text — is a chain of free functions in
the `engine` submodules below, parameterised by `P: Protocol`:

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

The engine functions in the diagram above (`coarse_sync`,
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
  `SupportsSicEarly`: FT8 only, fixed 3-checkpoint structure). Call
  `.decode()` to get a
  `DecodeOutcome<P>` (`.results: Vec<P::DecodeResult>`, plus
  `.fft_cache` for a follow-up call).
* **`SniperRequest<P>`** — narrow-band, single-target search.
  `DecodeRequest::<P>::sniper(audio, target_freq_hz, max_cand)` or
  `SniperRequest::<P>::new(...)` directly; `.osd(bool)`,
  `.strictness(...)`, `.eq_mode(...)`, and `.ap_hint(...)` where the
  protocol's message codec implements `WsjtApCompatible` (no SIC
  variant — sniper mode is inherently single-candidate). `.decode()`
  returns the same `DecodeOutcome<P>` shape.

This replaced FT8's `decode_frame*`/`decode_frame_subtract*`/
`decode_sniper*` family (15 public functions) and FT4/FST4's own
suffix-exploded equivalents — see §6.2/§6.4 for worked examples.

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

`DecodeStrictness` (`Strict`/`Normal`/`Deep`) exposes four methods,
with different live-per-protocol scope — check which apply before
assuming `.strictness(...)` does anything for a given call:

* `osd_score_min()` / `osd_max_errors()` — pre-OSD coarse-sync-score
  gate and post-OSD hard-error ceiling, `osd_depth`-tiered. **FT4-only
  in practice.** `osd_score_min` is bypassed entirely for both FST4
  and FT4 (`bypass_osd_score_min` in `engine/pipeline.rs` — FST4
  trusts CRC-24 alone, matching WSJT-X's own FST4 acceptance test
  `fst4_decode.f90:570`: `nharderrors >= 0 && unpk77_success`, no
  score pre-filter; FT4 hit the identical symptom independently and
  got the same bypass). `osd_max_errors` is FST4-bypassed too but
  *live* for FT4, retuned against a real `ft4sim` AWGN/CCIR sweep
  (issue #72, 2026-07-18) — no longer a placeholder copy of an FT8
  calibration. **Despite the name, FT8 has never actually called
  either method** — its own OSD dispatch used hardcoded constants
  instead (see `ft8_nharderrors_max` below); an earlier version of
  this doc incorrectly described these as FT8-calibrated.
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
| `msk144`        | off     | MSK144 — no `Protocol` ZST; own top-level driver (§0.6)       |
| `packet-bytes`  | off     | `PacketBytesMessage` — byte-payload example `MessageCodec`    |
| `uvpacket`      | off     | uvpacket — applied non-WSJT example, 4 sub-mode ZSTs (§10.1); pulls in `fst4` |
| `full`          | off     | Aggregate of all protocol features above                      |
| `parallel`      | on      | Enables rayon `par_iter` in pipeline (no-op under WASM)       |

## 6. Using from Rust

### 6.1 Dependencies

```toml
[dependencies]
mfsk-core = { version = "0.7", features = ["ft8", "ft4", "wspr"] }
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
            println!("{:7.2} Hz  {} {} {}dBm", d.freq_hz, callsign, grid, power_dbm);
        }
        WsprMessage::Type2 { callsign, power_dbm } => {
            println!("{:7.2} Hz  {} {}dBm", d.freq_hz, callsign, power_dbm);
        }
        WsprMessage::Type3 { callsign_hash, grid6, power_dbm } => {
            println!("{:7.2} Hz  <#{:05x}> {} {}dBm",
                     d.freq_hz, callsign_hash, grid6, power_dbm);
        }
    }
}
# }
```

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
    println!("{:7.2} Hz  {}", d.freq_hz, d.message);
}
# }
```

JT65 additionally offers `decode_at_with_erasures` for low-SNR
signals where RS erasure decoding can recover frames that the
standard decoder misses.

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
Other platforms/ABIs (including Android) still need a local build.

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
MfskDecoder*      mfsk_decoder_new(MfskProtocol protocol);
void              mfsk_decoder_free(MfskDecoder* dec);

// `options` may be NULL to use this crate's per-protocol default
// search range / threshold / depth, or a handle from
// mfsk_decode_options_new(...) to override it uniformly.
MfskDecodeOptions* mfsk_decode_options_new(float freq_min_hz, float freq_max_hz,
                                  float sync_min, int max_cand,
                                  MfskDecodeDepth depth);
void              mfsk_decode_options_free(MfskDecodeOptions* opts);

MfskStatus        mfsk_decode_i16(MfskDecoder*, const int16_t* samples,
                                  size_t n, uint32_t sample_rate,
                                  const MfskDecodeOptions* options,
                                  MfskResultList* out);
MfskStatus        mfsk_decode_f32(MfskDecoder*, const float*,  size_t,
                                  uint32_t, const MfskDecodeOptions*,
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
   protocol's built-in default).
5. **Errors**: on non-zero `MfskStatus`, call `mfsk_last_error` on the
   **same thread** to retrieve a human-readable diagnostic. The
   returned pointer is valid until the next fallible call on that
   thread.

### Thread safety

* An `MfskDecoder` is `!Sync`: one handle per concurrent thread.
* The decoder uses thread-local state for caching and error reporting,
  so spawning multiple threads each with its own handle is cheap.

## 9. Kotlin / Android consumers

`mfsk-ffi/examples/kotlin_jni/` ships a drop-in scaffold:

```kotlin
package io.github.mfskcore

Mfsk.open(Mfsk.Protocol.FT4).use { dec ->
    val pcm: ShortArray = /* captured audio */
    for (m in dec.decode(pcm, sampleRate = 12_000)) {
        Log.i("ft4", "${m.freqHz} Hz  ${m.snrDb} dB  ${m.text}")
    }
}
```

* `libmfsk.so` built via `cargo build --target aarch64-linux-android -p mfsk-ffi`.
* `libmfsk_jni.so` built from the ~115-line C shim, marshals
  `ShortArray` ↔ `MfskResultList`.
* `Mfsk.kt` exposes an `AutoCloseable` Kotlin class; use with
  `.use { }` to guarantee release.

Full build instructions in `mfsk-ffi/examples/kotlin_jni/README.md`.

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

FST4 does not share FT8's LDPC(174, 91); it uses a separate
LDPC(240, 101) + 24-bit CRC, implemented as `fec::ldpc240_101`.
The BP / OSD algorithms are structurally the same across LDPC
sizes, so the new material is essentially the parity-check and
generator tables together with the code dimensions. All five wired
sub-modes (FST4-15/30/60A/120/300) are complete end-to-end,
differing only in `NSPS` / `SYMBOL_DT` / `TONE_SPACING_HZ` (and
`TX_START_OFFSET_S` for FST4-15 alone, which starts 0.5 s rather
than 1.0 s into the slot); the `fst4_submode!` macro emits each ZST
the same way `q65_submode!` does for Q65. FST4-900 and FST4-1800
remain unwired (no user demand as of writing) but would follow the
same pattern. FST4W — the WSPR-style one-way 50-bit beacon variant,
LDPC(240, 74), periods 120/300/900/1800 s — is a separate message
format entirely and is out of scope here; see issue #23 for status.

WSPR is structurally different from the FT modes: it uses
convolutional coding (`fec::conv::ConvFano`, ported from WSJT-X
`lib/wsprd/fano.c`) rather than LDPC, a 50-bit message rather than
77-bit (`msg::wspr::Wspr50Message`, covering Types 1 / 2 / 3), and
a per-symbol interleaved sync (`SyncMode::Interleaved`) rather than
block Costas arrays. The `wspr` module contributes its own TX
synthesiser, RX demodulator, and a quarter-symbol spectrogram used
to keep the coarse search over a 120-s slot within a reasonable time
budget.

JT9 reuses the same convolutional FEC family as WSPR, but as its own
`ConvFano232` type (`fec::conv`) rather than literally sharing
`ConvFano` — JT9's 206-bit codeword framing differs from WSPR's —
plus a 72-bit JT message codec (`msg::jt72::Jt72Codec`). JT65 uses
Reed-Solomon `fec::rs::Rs63_12` (re-exported as `fec::Rs63_12`) with
erasure-aware decoding based on Karn's Berlekamp-Massey algorithm.

Q65 introduces a third FEC family: Q-ary repeat-accumulate codes
over GF(64), running non-binary belief propagation in the
probability domain via Walsh-Hadamard messages
(`fec::qra::QraCode` plus the concrete code instance
`fec::qra15_65_64::QRA15_65_64_IRR_E23`). The application layer
adds a CRC-12 over 13 user information symbols and punctures the
two CRC symbols out of the 65-symbol codeword, giving the 63
channel symbols actually transmitted. Ten wired sub-modes share
the same FEC + sync layout + 77-bit message; only `NSPS`
(15-s / 30-s / 60-s / 120-s / 300-s slot) and tone spacing
(×1, ×2, ×4, ×8, ×16) differ between them. The five parallel decoder
strategies introduced in
§3 (AWGN BP, AP-hint BP, fast-fading metric, AP-list template
matching, multi-period EMA averaging) all share the same QRA codec
under the hood.

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

#### Dual-probe view of the trait scope

The 0.4.0 release shipped two independent stress tests of the
trait abstractions, in opposite directions:

- **Q65 family expansion — *positive* probe.** Pushed the trait
  surface to a non-binary code (QRA over GF(2⁶)) and four parallel
  decoder strategies (AWGN / AP-hint / fast-fading / AP-list) without
  bending the trait shape. The `Protocol` / `ModulationParams` /
  `FrameLayout` / `FecCodec` / `MessageCodec` layers carried
  through unchanged for six sub-modes generated from one macro.
- **`uvpacket` — *negative* probe.** Stepped outside the WSJT
  family on every axis (modulation, sync, message format, slot
  policy) and observed where the abstraction naturally peels away.
  FEC + DSP + channel-test infrastructure carried over; the
  generic TX/RX pipeline and the message-codec / AP-compat traits
  did not.

The peeling is evidence that the trait surface is **right-sized
for the WSJT protocol family**, not evidence of a missing
generalisation. A general-purpose PHY framework would have to
abstract `SYNC_MODE` beyond "Costas blocks or interleaved" to
cover m-sequences, equaliser state, RRC pulse shaping, etc.; the
WSJT code paths would pay the indirection cost for no in-family
benefit. See [`docs/reference/UVPACKET.md`](UVPACKET.md) §0 for the same
view from the applied-example side.

For the full uvpacket design narrative, AX.25 / M17 / D-STAR / DMR
/ VARA comparison, and characterisation curves, see
[`docs/reference/UVPACKET.md`](UVPACKET.md). Representative WAV samples are
at `audio_samples/uvpacket/`.

## License

Library code is GPL-3.0-or-later, derived from WSJT-X reference
algorithms.
