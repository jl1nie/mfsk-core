# mfsk-core — Library Architecture & API Reference

> **日本語版:** [LIBRARY.ja.md](LIBRARY.ja.md)

This document covers the mfsk-core library surface for embedders:
Rust crate consumers, C/C++ projects linking `libmfsk.so`, and
Kotlin/Android apps using the JNI scaffold.

For a quick-start overview (badges, dependency snippet, minimal
example) see [README.md](../README.md). This document goes deeper
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
`core`, `fec`, and `msg` modules. Each protocol is a comparatively
small zero-sized type (ZST) that declares its own constants and the
specific FEC / message codec it uses. The pipeline is expressed as
`decode_frame::<P>()`, taking `P: Protocol` as a compile-time type
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
| Q65-30A      | 30 s   | QRA(15, 65) GF(2⁶) + CRC-12   | 77 bit | 22 distributed slots | `lib/qra/q65/`  |
| Q65-60A‥E    | 60 s   | (same QRA codec)              | 77 bit | (same sync layout)   | `lib/qra/q65/`  |

Q65 ships as six wired sub-modes — one terrestrial 30-s mode plus
five 60-s EME modes (Q65-60A through Q65-60E with tone-spacing
multipliers ×1, ×2, ×4, ×8, ×16). They share the FEC, message
codec, sync layout and a common impl block; only NSPS and tone
spacing differ.

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

#### Q65 — a third FEC family plus a four-way decoder-strategy axis

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
3. **Six sub-modes sharing one impl block** — Q65-30A for
   terrestrial work, plus Q65-60A‥E for EME at 6 m / 70 cm / 23 cm
   / 5.7 GHz / 10 GHz / 24 GHz+. They differ only in NSPS and tone
   spacing. The `q65_submode!` macro emits the per-sub-mode ZSTs
   and their trait impls in one line each — no per-mode code
   duplication.
4. **Four parallel decoder strategies** — Q65 is the first protocol
   in the library where the receiver chain has multiple legitimate
   paths through the same FEC. They are listed in §3:
   plain AWGN BP, AP-biased BP, fast-fading metric, and AP-list
   template matching. Each is a distinct entry-point function
   generic over the sub-mode ZST; the underlying FEC and message
   codec are shared.

Where WSPR proved that *FEC family + message width + sync mode*
could each be swapped independently, Q65 proves that adding a third
FEC family, six sub-modes and four parallel decoder strategies all
still fit inside the same `Protocol` super-trait without bespoke
plumbing. §3 expands on the four strategies; §7 covers the
`PROTOCOLS` registry that lets consumers enumerate every wired
protocol (eleven ZSTs in total) and the
`tests/protocol_invariants.rs` generic checker that holds the trait
contract honest.

## 1. Module layout

```
mfsk_core
├── core/             Protocol traits, DSP, sync, LLR, equaliser, pipeline
│   ├── protocol.rs     ModulationParams / FrameLayout / Protocol / FecCodec / MessageCodec
│   ├── dsp/            resample · downsample · gfsk · subtract
│   ├── sync.rs         coarse_sync / refine_candidate
│   ├── llr.rs          symbol_spectra / compute_llr / sync_quality
│   ├── equalize.rs     equalize_local (Wiener per-tone)
│   └── pipeline.rs     decode_frame / decode_frame_subtract / process_candidate_basic
├── fec/              FecCodec implementations
│   ├── ldpc174_91.rs   LDPC(174, 91)  — FT8, FT4
│   ├── ldpc240_101.rs  LDPC(240, 101) — FST4
│   ├── conv.rs         ConvFano r=½ K=32 — WSPR, JT9
│   ├── rs63_12.rs      RS(63, 12) GF(2⁶) — JT65
│   └── qra/            Q-ary RA codec family — Q65
│       ├── code.rs       Generic QRA encoder + non-binary BP decoder
│       ├── q65.rs        Q65 application wrapper (CRC-12 + puncturing) +
│       │                 list-decoding primitives (check_codeword_llh,
│       │                 decode_with_codeword_list)
│       ├── fast_fading.rs Doppler-spread-aware intrinsic metric
│       └── fading_tables.rs Gaussian / Lorentzian calibration tables
├── msg/              Message codecs
│   ├── wsjt77.rs       77-bit WSJT message (pack / unpack) — FT8, FT4, FST4, Q65
│   ├── wspr.rs         50-bit WSPR Types 1 / 2 / 3
│   ├── jt72.rs         72-bit JT message — JT9, JT65
│   └── hash.rs         Callsign hash table
├── registry.rs       PROTOCOLS static + ProtocolMeta + by_id / by_name
├── ft8/              FT8 ZST + decode + wave_gen
├── ft4/              FT4 ZST + decode
├── fst4/             FST4-60A ZST + decode
├── wspr/             WSPR ZST + decode + synth + spectrogram search
├── jt9/              JT9 ZST + decode
├── jt65/             JT65 ZST + decode (+ erasure-aware RS)
└── q65/              Q65 family — 6 sub-mode ZSTs + decode + synth
    ├── protocol.rs     q65_submode! macro emitting Q65a30 .. Q65e60 ZSTs
    ├── rx.rs           4 decoder strategies (AWGN / AP / fast-fading / AP-list)
    ├── ap_list.rs      standard_qso_codewords — full AP-list candidate generator
    ├── tx.rs           65-FSK synthesiser (sub-mode-aware)
    ├── search.rs       coarse 22-symbol Costas-block search
    └── sync_pattern.rs Q65 distributed sync layout
```

Each protocol module is gated behind a feature flag (`ft8`, `ft4`,
`fst4`, `wspr`, `jt9`, `jt65`, `q65`). The `core`, `fec`, `msg`
and `registry` modules are always available.

The `mfsk-ffi` sibling crate in this workspace builds a C ABI
shared library (`libmfsk.{so,a,dylib}` + `mfsk.h`) on top of the
same crate.

## 2. Protocol trait hierarchy

Every supported mode is described by a zero-sized type that
implements three composable traits:

```rust
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
use mfsk_core::core::protocol::*;
use mfsk_core::fec::ldpc174_91::Ldpc174_91;
use mfsk_core::msg::wsjt77::Wsjt77Message;

pub struct Ft4;

impl ModulationParams for Ft4 {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 576;          // 48 ms @ 12 kHz
    const TONE_SPACING_HZ: f32 = 20.833;
    const GRAY_MAP: &'static [u8] = &[0, 1, 3, 2];
    // … (GFSK_BT, NFFT_PER_SYMBOL_FACTOR, NDOWN, …)
}

impl FrameLayout for Ft4 {
    const N_DATA: u32 = 87;
    const N_SYNC: u32 = 16;
    const N_SYMBOLS: u32 = 103;
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
use mfsk_core::fec::conv::ConvFano;
use mfsk_core::msg::wspr::Wspr50Message;

pub struct Wspr;

impl ModulationParams for Wspr {
    const NTONES: u32 = 4;
    const BITS_PER_SYMBOL: u32 = 2;
    const NSPS: u32 = 8192;                  // ~683 ms @ 12 kHz
    const TONE_SPACING_HZ: f32 = 12_000.0 / 8192.0;  // ≈ 1.4648
    const GRAY_MAP: &'static [u8] = &[0, 1, 2, 3];
    // …
}

impl FrameLayout for Wspr {
    const N_DATA: u32 = 162;
    const N_SYNC: u32 = 0;                   // sync is embedded in data symbols
    const N_SYMBOLS: u32 = 162;
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
```

Calling code just passes the ZST as a type argument —
`decode_frame::<Ft4>(...)` or the WSPR-specific
`wspr::decode::decode_scan_default(...)` — and the trait composition
pulls in the appropriate FEC, message codec, and sync mode
automatically.

### Monomorphisation & zero cost

All hot-path functions (`core::sync::coarse_sync::<P>`,
`core::llr::compute_llr::<P>`,
`core::pipeline::process_candidate_basic::<P>`, …) take
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
   existing implementations, and the full `decode_frame::<P>()`
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
`decode_frame::<P>` for the FT family, `wspr::decode::decode_scan_default`
for WSPR, etc. Q65 is the first wired protocol where a single FEC
frame can be approached through **four** legitimately different
receiver chains, each trading runtime cost against a different kind
of channel pathology. They live side by side as parallel
function families in `mfsk_core::q65::rx`, each generic over the
sub-mode ZST.

| When                                               | Strategy                  | Entry point                                                   | Threshold gain |
|----------------------------------------------------|---------------------------|---------------------------------------------------------------|----------------|
| Default — unknown channel, unknown content         | AWGN Bessel + BP          | `decode_at_for<P>` / `decode_scan_for<P>`                     | baseline       |
| Known callsign(s) or report, terrestrial channel   | AP-biased BP              | `decode_at_with_ap_for<P>` / `decode_scan_with_ap_for<P>`     | ~2 dB          |
| Doppler-spread channel (microwave EME, ≥10 Hz spread) | Fast-fading metric + BP | `decode_at_fading_for<P>` / `decode_scan_fading_for<P>`       | 5–8 dB on spread channels |
| Known call pair, no QSO context, terrestrial       | AP-list template matching | `decode_at_with_ap_list_for<P>` / `decode_scan_with_ap_list_for<P>` | ~3 dB          |

**AWGN Bessel + BP** is the textbook path: per-symbol FFT energies
become probability vectors via the Bessel-I0 metric, then non-binary
belief propagation runs on the QRA code. Falls back gracefully on
any channel reasonably close to additive Gaussian noise.

**AP-biased BP** clamps the intrinsic probability vectors at known
information-bit positions before BP. A correct hint shifts the BP
fixed-point closer to the truth; a wrong hint typically fails to
converge rather than misdecoding (the CRC catches what's left).
Construct the hint via `mfsk_core::msg::ApHint` (`with_call1`,
`with_call2`, `with_grid`, `with_report`).

**Fast-fading metric** replaces the Bessel front end with a
spread-aware alternative calibrated against Gaussian or Lorentzian
fading shapes. Required for microwave EME where lunar libration
spreads each tone over 10–60 Hz: the 10 GHz EME reference recording
in `samples/Q65/60D_EME_10GHz/` decodes via this path but produces
zero hits with the AWGN front end. `b90_ts` is the spread bandwidth
× symbol period (typical: 0.05 = near-AWGN, 1.0 = moderate,
5.0+ = severe).

**AP-list template matching** does *not* run BP. Instead, the
generator `q65::ap_list::standard_qso_codewords(my_call, his_call,
his_grid)` pre-encodes the WSJT-X "full AP list" — 206 standard
exchanges that a known callsign pair can legally produce
(`MYCALL HISCALL`, `MYCALL HISCALL RRR/RR73/73`, `CQ HISCALL grid`,
plus the 200-entry SNR ladder). The decoder picks the candidate
whose log-likelihood under the soft observations exceeds a
size-adjusted threshold, or returns `None`. Useful when the
application has a known callsign pair but no QSO state — and at
SNR −25 dB (1 dB below the published Q65-30A threshold), the
test sweep shows AP-list decodes 6/6 frames where plain BP fails
0/6.

**Composing strategies.** Fast-fading and AP-list compose: the
fast-fading metric produces intrinsic vectors that
`Q65Codec::decode_with_codeword_list` accepts directly. Wiring a
combined `decode_at_fading_with_ap_list_for<P>` is a small additive
change; for now, callers do the composition explicitly via the
lower-level primitives (`intrinsics_fast_fading` + `Q65Codec`).

The C ABI exposes the same four strategies one-for-one as
`mfsk_q65_decode`, `mfsk_q65_decode_with_ap`, `mfsk_q65_decode_fading`
and `mfsk_q65_decode_with_ap_list`, each taking a `MfskQ65SubMode`
parameter so any of the six sub-modes is reachable from C/C++/Kotlin.

## 4. Shared primitives (`core`)

### DSP (`mfsk_core::core::dsp`)

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

### Sync (`mfsk_core::core::sync`)

* `coarse_sync::<P>(audio, freq_min, freq_max, …)` — UTC-aligned 2D
  peak search over `P::SYNC_MODE.blocks()`.
* `refine_candidate::<P>(cd0, cand, search_steps)` — integer-sample
  scan + parabolic sub-sample interpolation.
* `make_costas_ref(pattern, ds_spb)` / `score_costas_block(...)` — raw
  correlation helpers exposed for diagnostics and custom pipelines.

### LLR (`mfsk_core::core::llr`)

* `symbol_spectra::<P>(cd0, i_start)` — per-symbol FFT bins.
* `compute_llr::<P>(cs)` — four WSJT-style LLR variants (a/b/c/d).
* `sync_quality::<P>(cs)` — hard-decision sync symbol count.

### Equalise (`mfsk_core::core::equalize`)

* `equalize_local::<P>(cs)` — per-tone Wiener equaliser driven by
  `P::SYNC_MODE.blocks()` pilot observations; linearly extrapolates any tones
  that Costas doesn't visit.

### Pipeline (`mfsk_core::core::pipeline`)

* `decode_frame::<P>(...)` — coarse sync → parallel process_candidate → dedupe.
* `decode_frame_subtract::<P>(...)` — 3-pass SIC driver.
* `process_candidate_basic::<P>(...)` — single-candidate BP+OSD.

AP-aware variants live in `msg::pipeline_ap` because AP hint
construction is 77-bit specific.

## 5. Feature flags

| Feature       | Default | Effect                                                        |
|---------------|---------|---------------------------------------------------------------|
| `ft8`         | on      | FT8 ZST, decode, wave_gen                                    |
| `ft4`         | on      | FT4 ZST, decode                                              |
| `fst4`        | off     | FST4-60A ZST, decode                                         |
| `wspr`        | off     | WSPR ZST, decode, synth, spectrogram search                  |
| `jt9`         | off     | JT9 ZST, decode                                              |
| `jt65`        | off     | JT65 ZST, decode (+ erasure-aware RS)                        |
| `q65`         | off     | Q65-30A + Q65-60A‥E ZSTs, four decode strategies, synth      |
| `uvpacket`    | off     | UvPacket150 / 300 / 600 / 1200 ZSTs, byte-payload TX & RX    |
| `full`        | off     | Aggregate of all eight protocol families                      |
| `parallel`    | on      | Enables rayon `par_iter` in pipeline (no-op under WASM)       |
| `osd-deep`    | off     | Adds OSD-3 fallback to AP decodes under ≥55-bit lock          |
| `eq-fallback` | off     | Lets `EqMode::Adaptive` fall back to non-EQ when EQ fails     |

Both `osd-deep` and `eq-fallback` are heavy: they were measured to
boost FT4's −18 dB success rate by ~5/10 → 6/10 at the cost of ~10×
decode time. Left **off** by default so the stock build fits a 7.5 s
WASM slot comfortably; turn them on when running on a desktop where
CPU budget is abundant.

## 6. Using from Rust

### 6.1 Dependencies

```toml
[dependencies]
mfsk-core = { version = "0.1", features = ["ft8", "ft4", "wspr"] }
```

Pull in only the protocol features you need; the examples below
enable several for illustration.

### 6.2 FT8 decode — minimal example

```rust
use mfsk_core::ft8::{
    decode::{decode_frame, DecodeDepth},
    wave_gen::{message_to_tones, tones_to_i16},
};
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

// 2. Decode it back.
for r in decode_frame(&audio, 100.0, 3_000.0, 1.0, None,
                      DecodeDepth::BpAllOsd, 50) {
    if let Some(text) = unpack77(&r.message77) {
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
use mfsk_core::wspr::decode::decode_scan_default;
use mfsk_core::msg::WsprMessage;

let samples_f32: Vec<f32> = /* 120 s × 12 kHz of f32 samples */;

let decodes = decode_scan_default(&samples_f32, /*sample_rate*/ 12_000);
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
```

`decode_scan_default` runs the (frequency × time) coarse search over
the whole slot internally. If the frequency and start sample are
already known, `wspr::decode::decode_at(samples, rate,
start_sample, freq_hz)` bypasses the scan.

### 6.4 Sniper mode + AP hint

Narrowing the search to ±500 Hz and supplying an a-priori hint lets
the decoder recover weaker signals:

```rust
use mfsk_core::ft8::decode::{decode_sniper_ap, DecodeDepth, ApHint};
use mfsk_core::core::equalize::EqMode;

let ap = ApHint::new().with_call1("CQ").with_call2("JA1ABC");
for r in decode_sniper_ap(&audio, /*target_hz*/ 1000.0,
                          DecodeDepth::BpAllOsd, /*max_cand*/ 15,
                          EqMode::Adaptive, Some(&ap)) {
    // …
}
```

The FT4 equivalent is `ft4::decode::decode_sniper_ap` with a similar
signature.

### 6.5 JT9 / JT65

Both JT9 and JT65 expose the same scan + point-decode pattern:

```rust
use mfsk_core::jt65::decode_scan_default;

let audio_f32: Vec<f32> = /* 60 s × 12 kHz of f32 samples */;
for d in decode_scan_default(&audio_f32, 12_000) {
    println!("{:7.2} Hz  {}", d.freq_hz, d.message);
}
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
  entry sharing the family-level id. Q65 yields six (one per
  sub-mode); other protocols yield one.
* `mfsk_core::by_name("Q65-60D")` — exact-match name lookup.
* `mfsk_core::for_protocol_id(id)` — first entry sharing the id;
  convenient for the "single-mode-per-family" case.

Q65 is the case where the family / sub-mode distinction matters:
all six Q65 sub-modes share `ProtocolId::Q65` (the FFI tag is
family-level) but live as distinct registry entries because their
NSPS, tone spacing and slot length differ.

The registry is built by an internal `protocol_meta!` macro in
`mfsk-core/src/registry.rs`; adding a new protocol is one line per
ZST plus its display name.

### 7.2 The generic trait-surface checker

`tests/protocol_invariants.rs` runs a single generic
`assert_protocol_invariants::<P: Protocol>(name)` against every
wired ZST. The body is the same for FT8, FT4, FST4, WSPR, JT9,
JT65 and all six Q65 sub-modes — eleven invocations, one
implementation. Seventeen invariants are pinned across three
helper functions:

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
    MFSK_PROTOCOL_FST4S60 = 5,
};

uint32_t          mfsk_version(void);           // major<<16 | minor<<8 | patch
MfskDecoder*      mfsk_decoder_new(MfskProtocol protocol);
void              mfsk_decoder_free(MfskDecoder* dec);

MfskStatus        mfsk_decode_i16(MfskDecoder*, const int16_t* samples,
                                  size_t n, uint32_t sample_rate,
                                  MfskMessageList* out);
MfskStatus        mfsk_decode_f32(MfskDecoder*, const float*,  size_t,
                                  uint32_t, MfskMessageList* out);

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

void              mfsk_message_list_free(MfskMessageList* list);
void              mfsk_samples_free(MfskSamples* s);
const char*       mfsk_last_error(void);
```

`MfskMessageList` is caller-owned storage filled by the decode call;
text fields are `char*` UTF-8 NUL-terminated, owned by the list and
freed by `mfsk_message_list_free`.

`MfskSamples` is caller-owned storage filled by encode calls; it
holds 12 kHz f32 PCM and is freed by `mfsk_samples_free`.

See `mfsk-ffi/examples/cpp_smoke/` for a minimal end-to-end demo.

### Memory rules

1. **Handles**: allocate with `mfsk_decoder_new`, free with
   `mfsk_decoder_free`. One handle per thread. Free is idempotent on
   NULL.
2. **Message lists**: zero-initialise a `MfskMessageList` on the
   stack, pass its address to the decode call, free with
   `mfsk_message_list_free` when done reading. Do *not* free
   individual `text` pointers yourself.
3. **Sample buffers**: zero-initialise `MfskSamples`, pass to
   encode, free with `mfsk_samples_free`.
4. **Errors**: on non-zero `MfskStatus`, call `mfsk_last_error` on the
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
  `ShortArray` ↔ `MfskMessageList`.
* `Mfsk.kt` exposes an `AutoCloseable` Kotlin class; use with
  `.use { }` to guarantee release.

Full build instructions in `mfsk-ffi/examples/kotlin_jni/README.md`.

## 10. Protocol notes

| Protocol   | Slot   | Tones | Symbols | Tone Δf    | FEC              | Msg   | Sync       | Status |
|------------|--------|-------|---------|------------|------------------|-------|------------|--------|
| FT8        | 15 s   | 8     | 79      | 6.25 Hz    | LDPC(174, 91)    | 77 b  | 3×Costas-7 | implemented |
| FT4        | 7.5 s  | 4     | 103     | 20.833 Hz  | LDPC(174, 91)    | 77 b  | 4×Costas-4 | implemented |
| FST4-60A   | 60 s   | 4     | 160     | 3.125 Hz   | LDPC(240, 101)   | 77 b  | 5×Costas-8 | implemented |
| FST4 other | 15–1800 s | 4 | var     | var        | LDPC(240, 101)   | 77 b  | 5×Costas-8 | one more ZST per sub-mode |
| WSPR       | 120 s  | 4     | 162     | 1.465 Hz   | conv r=½ K=32 + Fano | 50 b | per-symbol LSB (npr3) | implemented |
| JT9        | 60 s   | 9     | 85      | 1.736 Hz   | conv r=½ K=32 + Fano | 72 b  | 16 distributed | implemented |
| JT65       | 60 s   | 65    | 126     | 2.69 Hz    | RS(63, 12) GF(2⁶)     | 72 b  | 63 distributed | implemented |
| Q65-30A    | 30 s   | 65    | 85      | 3.333 Hz   | QRA(15, 65) GF(2⁶) + CRC-12 | 77 b | 22 distributed | implemented |
| Q65-60A    | 60 s   | 65    | 85      | 1.667 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (6 m EME) |
| Q65-60B    | 60 s   | 65    | 85      | 3.333 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (70 cm / 23 cm EME) |
| Q65-60C    | 60 s   | 65    | 85      | 6.667 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (~3 GHz EME) |
| Q65-60D    | 60 s   | 65    | 85      | 13.33 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (5.7 / 10 GHz EME) |
| Q65-60E    | 60 s   | 65    | 85      | 26.67 Hz   | (same QRA codec) | 77 b  | (same)     | implemented (24 GHz+, extreme spread) |
| UvPacket150  | n/a (packet) | 4 | 99   | 150 Hz   | LDPC(174, 91) + CRC-7 | 1–10 B | 2×Costas-4 + interleave | implemented (mobile U/VHF SSB, weak) |
| UvPacket300  | n/a (packet) | 4 | 99   | 300 Hz   | (same)           | 1–10 B | (same)    | implemented (typical U/VHF SSB)      |
| UvPacket600  | n/a (packet) | 4 | 99   | 600 Hz   | (same)           | 1–10 B | (same)    | implemented (clean U/VHF SSB)        |
| UvPacket1200 | n/a (packet) | 4 | 99   | 1200 Hz  | (same)           | 1–10 B | (same)    | implemented (FM data channel, ≈1.2 kbps net) |

FST4 does not share FT8's LDPC(174, 91); it uses a separate
LDPC(240, 101) + 24-bit CRC, implemented as `fec::ldpc240_101`.
The BP / OSD algorithms are structurally the same across LDPC
sizes, so the new material is essentially the parity-check and
generator tables together with the code dimensions. FST4-60A is
complete end-to-end; the other FST4 sub-modes (-15/-30/-120/-300/
-900/-1800) differ only in `NSPS` / `SYMBOL_DT` /
`TONE_SPACING_HZ`, and each can be added as a short ZST reusing the
same FEC, sync and DSP.

WSPR is structurally different from the FT modes: it uses
convolutional coding (`fec::conv::ConvFano`, ported from WSJT-X
`lib/wsprd/fano.c`) rather than LDPC, a 50-bit message rather than
77-bit (`msg::wspr::Wspr50Message`, covering Types 1 / 2 / 3), and
a per-symbol interleaved sync (`SyncMode::Interleaved`) rather than
block Costas arrays. The `wspr` module contributes its own TX
synthesiser, RX demodulator, and a quarter-symbol spectrogram used
to keep the coarse search over a 120-s slot within a reasonable time
budget.

JT9 reuses the convolutional FEC (`ConvFano`) and a 72-bit JT
message codec (`msg::jt72::Jt72Codec`). JT65 uses Reed-Solomon
`fec::rs63_12::Rs63_12` with erasure-aware decoding based on Karn's
Berlekamp-Massey algorithm.

Q65 introduces a third FEC family: Q-ary repeat-accumulate codes
over GF(64), running non-binary belief propagation in the
probability domain via Walsh-Hadamard messages
(`fec::qra::QraCode` plus the concrete code instance
`fec::qra15_65_64::QRA15_65_64_IRR_E23`). The application layer
adds a CRC-12 over 13 user information symbols and punctures the
two CRC symbols out of the 65-symbol codeword, giving the 63
channel symbols actually transmitted. Six wired sub-modes share
the same FEC + sync layout + 77-bit message; only `NSPS`
(30-s vs 60-s slot) and tone spacing (×1, ×2, ×4, ×8, ×16) differ
between them. The four parallel decoder strategies introduced in
§3 (AWGN BP, AP-biased BP, fast-fading metric, AP-list template
matching) all share the same QRA codec under the hood.

## 11. uvpacket — motivating worked example

Most of this document treats `mfsk-core` as a Rust port of WSJT-X:
the protocols (FT8 / FT4 / FST4 / WSPR / JT9 / JT65 / Q65) are
fixed by the existing reference implementation, and the value-add
is making them run somewhere WSJT-X cannot. uvpacket is the
opposite use case — a brand-new protocol family designed inside
this codebase to demonstrate that the abstractions in §2 are not
just convenient repackaging of WSJT-X but a foundation that can
host new MFSK protocols. It also fills a real gap in amateur
data: short-payload packet transport over mobile / portable
U/VHF radios, where WSJT-X's slot-locked weak-signal modes are
the wrong shape and APRS / 9k6 packet has no built-in burst-error
tolerance.

### 11.1 Why uvpacket exists — the gap it fills

| Capability the operator wants            | FT8 / FT4 | APRS 1k2/9k6 | uvpacket |
|------------------------------------------|-----------|--------------|----------|
| Send arbitrary bytes (not pre-defined fields) | ✗  | ✓            | ✓        |
| Run any time, no UTC slot                | ✗         | ✓            | ✓        |
| Survive mobile multipath fading          | partial   | ✗            | ✓        |
| Multiple senders share one channel       | ✓         | partial      | ✓        |
| Soft-decision FEC on the receiver        | ✓         | ✗            | ✓        |
| ≥ 1 kbps net throughput                  | ✗         | ✓ (9k6)      | ✓ (1200) |

WSJT-family modes are tuned for the ionospheric weak-signal use
case: 15-second to 60-second slots, callsign / grid / report
fields, no per-frame integrity tag beyond the payload-level CRC.
APRS gives you bytes and runs continuously, but its AX.25 framing
plus FM emphasis lacks any form of bit-level FEC — a single deep
fade at U/VHF mobile speeds drops the whole frame. uvpacket sits
in the middle: WSJT-style soft-decision LDPC + interleaver, but
short-frame packet-shaped, no slot, byte payload, multi-signal
ready.

### 11.2 Channel assumptions

uvpacket is engineered for **U/VHF mobile / portable operation**.
The dominant impairments at those bands are:

- **Time-selective Rayleigh fading**: the multipath delay spread
  at U/VHF is small (μs-class) compared to a 3 kHz audio channel,
  so the channel is **frequency-flat** — the entire SSB / FM
  passband fades together as one complex gain `h(t)`. The
  envelope `|h(t)|` follows a Rayleigh distribution; the time
  scale of fades is set by Doppler. At 144 MHz / 50 km/h vehicle
  speed, Doppler is ~6.7 Hz so coherence time is ~150 ms — long
  enough to swallow several symbols at the slower sub-modes.
- **Burst-shaped errors**: a fade null does not corrupt random
  bits — it kills *consecutive* symbols. Forward-error correction
  designed for AWGN (e.g. plain LDPC) fails on this distribution
  because the errors cluster.
- **Multiple concurrent transmitters in the same band**: like
  FT8 channels, dozens of stations share a 3 kHz audio passband
  on different audio centre frequencies. The receiver must run
  a sliding-window coarse search and decode every distinct signal
  in parallel.

### 11.3 Design choices

Three mechanisms work together to make the LDPC + flat-fading
combination viable:

**Bit interleaver — `UVPACKET_INTERLEAVE`** &mdash; the 174 LDPC
codeword bits go through a stride-25 polynomial permutation
(`channel_bit[j] = codeword[(7·j) mod 174]`) before they hit the
4-FSK symbol mapper. Consecutive codeword bits land 25 channel
positions apart, so a burst of B consecutive corrupted channel
bits decoheres into B sparse codeword-bit losses at stride 7
(modulo 174). For a 50 ms fade null at UvPacket150 (≈ 7.5
symbols ≈ 14 channel bits) those 14 losses scatter across 14
distinct codeword positions out of 174 &mdash; sparse enough
for soft-decision LDPC to correct.

**Two Costas-4 sync blocks** &mdash; one at frame head (symbol 0),
one at mid-frame (symbol 47). A fade null at the start of the
frame still leaves the mid-frame block intact for
`coarse_sync` to lock onto, and vice versa.

**Sub-mode rate ladder** &mdash; four sub-modes share everything
(FEC, message codec, interleaver, sync layout) and differ only in
NSPS / tone spacing. The slowest variant trades throughput for
weak-signal resilience; the fastest drops below SSB-fit but
exploits the wider audio bandwidth available on FM data channels.

| ZST              | Baud  | Tone Δf | Audio BW | Net rate | Frame   | Channel        |
|------------------|-------|---------|----------|----------|---------|----------------|
| `UvPacket150`    |   150 |  150 Hz |  600 Hz  |  150 bps |  660 ms | weak SSB       |
| `UvPacket300`    |   300 |  300 Hz | 1200 Hz  |  300 bps |  330 ms | typical SSB    |
| `UvPacket600`    |   600 |  600 Hz | 2400 Hz  |  600 bps |  165 ms | clean SSB      |
| `UvPacket1200`   |  1200 | 1200 Hz | 4800 Hz  |  1.2 kbps|   83 ms | FM data        |

The 12 kHz audio pipeline caps practical baud at ~1200 before
Nyquist starts cutting tone separation. Higher rates would
require either a wider sample rate (24 / 48 kHz) or higher-order
modulation (PSK / QAM) — out of scope for this protocol.

### 11.4 What the implementation reuses

The whole protocol fits in `mfsk-core/src/uvpacket/`:

```text
src/uvpacket/
├── mod.rs           -- module docs + public re-exports
├── protocol.rs      -- ZSTs, uvpacket_submode! macro, INTERLEAVE table
├── sync_pattern.rs  -- 2× Costas-4 sync block layout
├── tx.rs            -- bytes → info → LDPC → tones (uses codeword_to_itone)
├── rx.rs            -- audio → pipeline::decode_frame → typed bytes
└── chain.rs         -- multi-frame chaining for >10-byte payloads
```

It pulls everything else from the shared infrastructure:

- **FEC**: `fec::Ldpc174_91` (the same FT8 / FT4 LDPC, no new FEC needed).
- **Message codec**: `msg::PacketBytesMessage` (4-bit length + 80-bit payload + 7-bit CRC-7); `verify_info` runs the CRC at the BP layer so spurious decodes are rejected mid-iteration.
- **DSP frontend**: `core::sync::coarse_sync<P>`, `core::dsp::downsample`, `core::llr::compute_llr<P>` — all generic over the protocol trait.
- **Decoder**: `pipeline::decode_frame::<P>` runs sync → downsample → LLR → BP/OSD with no uvpacket-specific code paths. The interleave is honoured by `core::tx::codeword_to_itone` (TX side) and `pipeline::deinterleave_llr_set` (RX side) via the `FrameLayout::CODEWORD_INTERLEAVE` constant.
- **Synthesis**: `core::dsp::gfsk::synth_f32` — same GFSK shaping FT4 uses.

The only genuinely new pieces are: the `INTERLEAVE` table (10 lines
of `const fn`), the two-Costas-block sync layout (5 lines), the
four `uvpacket_submode!` invocations (4 × 8 lines), the per-sub-mode
ZST + trait impls (handled by the macro), and the `chain` module
for multi-frame payloads. The TX path is 80 lines; the RX path
is 100 lines and is essentially "call `pipeline::decode_frame` then
unpack each result through `PacketBytesMessage::unpack`".

### 11.5 Headless quick start

```rust
# #[cfg(feature = "uvpacket")] {
use mfsk_core::uvpacket::{
    UvPacket600, decode_frame, synthesize_packet,
};

// TX: 5-byte payload at 1500 Hz, 600 baud → 12 kHz f32 PCM
let pcm_f32 = synthesize_packet::<UvPacket600>(b"hello", 1500.0, 0.5)
    .expect("payload fits in one frame");
let mut audio: Vec<i16> = pcm_f32
    .iter()
    .map(|&s| (s * 20_000.0) as i16)
    .collect();
audio.resize(12_000, 0); // 1 s buffer

// RX: scan 100..3000 Hz for any uvpacket frame
let frames = decode_frame::<UvPacket600>(
    &audio, 100.0, 3_000.0, /* sync_min */ 1.0, /* max_cand */ 50,
);
assert_eq!(frames[0].payload, b"hello");
# }
```

`tests/uvpacket_roundtrip.rs` exercises this end-to-end across
all four sub-modes including AWGN at moderate SNR, a 5 Hz Doppler
Rayleigh-fading channel, a 2 Hz slow-fade case for UvPacket150,
and a head-end burst-null scenario that confirms the mid-frame
sync + interleaver actually buy what the design claims.

### 11.6 Further reading

- [`docs/UVPACKET.md`](UVPACKET.md) — implementation deep dive:
  the interleaver math, Costas placement choice, channel-model
  derivation, fading test methodology, and the limits we hit
  trying to push net rate beyond 1.2 kbps in a 12 kHz audio
  pipeline.

## License

Library code is GPL-3.0-or-later, derived from WSJT-X reference
algorithms.
