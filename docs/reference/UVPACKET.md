# uvpacket — applied example: NFM voice-channel packet protocol

> **日本語版:** [UVPACKET.ja.md](UVPACKET.ja.md)

`uvpacket` is an in-tree **applied example** of how `mfsk-core`'s
FEC infrastructure (`Ldpc240_101`, belief propagation, OSD-2/3)
can be reused outside the WSJT-X family. It targets NFM voice
channels (HT/mobile, ~3 kHz audio passband) and — via the
AFC entry point — SSB carriers as well.

This document covers the modem itself. Application semantics
(signed QSL exchange, position beacons, short text) are
**not** part of `mfsk-core`; see §1.4 for the intended layering.

## 0. Why this applied example exists — the negative-probe pair to Q65

`uvpacket` is **not** a stand-alone applied example. It is a
deliberate **negative probe** of where `mfsk-core`'s trait
abstractions naturally stop holding, paired with the Q65 family
expansion shipped in 0.4.0 as the **positive probe** of how far
they reach inside the WSJT family. The two together — not either
alone — are what test the trait design's scope.

### What Q65 confirmed (positive)

The `Protocol` / `ModulationParams` / `FrameLayout` / `FecCodec` /
`MessageCodec` layers carried cleanly through, all the way to:

- a **non-binary** code (QRA over GF(2⁶)) plugged into `FecCodec`'s
  bit-level API by packing symbols inside its own `encode`;
- six sub-modes generated from a single `q65_submode!` macro, all
  passing `tests/protocol_invariants.rs`;
- four parallel decode strategies (AWGN / AP-hint / fast-fading /
  AP-list) sharing one generic `decode_at_for::<P>` body.

That's a positive signal: the trait surface stretches over the
"natural extensions" of the WSJT family without bending.

### What `uvpacket` was meant to probe (negative)

`uvpacket` deliberately walks outside the WSJT family:

- single-carrier **π/4-DQPSK + LMS equaliser** (not M-ary tone FSK);
- a **byte-pipe API** (not a structured callsign + grid message);
- its own dedicated header LDPC block + **variable-length bursts**
  (not fixed slots);
- four-variant **127-chip BPSK preambles** (not Costas tone-index
  blocks).

Hypothesis: *"if the trait abstractions break here it doesn't hurt
inside the WSJT family but it does mean the abstraction is missing
expressive power; if they peel away cleanly, the trait scope is
right-sized for WSJT."*

### The observed boundary

| Component | Outcome |
|---|---|
| `Ldpc240_101` mother code + BP / OSD-2/3 | **borrowed as-is** |
| Test-channel infrastructure (AWGN / Rayleigh / SSB realistic / FM realistic) | **borrowed as-is** |
| DSP primitives (RRC, correlators, LMS solve) | **borrowed as-is** |
| `MessageCodec` | **deliberately bypassed** (byte-pipe: no structured codec needed) |
| `WsjtApCompatible` | **deliberately bypassed** (uvpacket has no AP concept) |
| `Protocol::ID` / `ModulationParams` trait constants | **decorative** — not consulted by the bespoke TX/RX pipeline |
| Generic `engine::pipeline::decode_frame::<P>` | **not used** (bespoke `uvpacket::rx`) |

### Conclusion — the traits are intentionally WSJT-specific

`mfsk-core`'s trait abstractions are a **concrete abstraction
optimised for the WSJT protocol family**, not a general-purpose PHY
framework. That's a correct design judgement:

- *Because* the traits are WSJT-specific, a single `<P>` parameter
  threads through `coarse_sync::<P>` / `compute_llr::<P>` /
  `decode_frame::<P>` and monomorphises into per-protocol code that
  costs nothing — Q65's six sub-modes drop in for the price of one
  trait impl block.
- A general PHY framework would have to generalise `SYNC_MODE`
  beyond "Costas blocks or interleaved" to cover m-sequences, RRC
  pulse shaping, equaliser state, etc. The WSJT code paths would
  then read through additional indirection for no in-family benefit.

`uvpacket` demonstrates that outside that boundary the WSJT trait
abstractions peel away cleanly, which is *evidence the abstraction
is correctly scoped*, not evidence that it falls short. For
non-WSJT protocols the natural way to use `mfsk-core` is as a thin
library of FEC + DSP + channel-test infrastructure underneath your
own TX/RX pipeline — exactly the way `uvpacket` itself uses it.

`docs/reference/LIBRARY.md` §10.1 summarises the same dual-probe view from
the trait-design side, and is a one-paragraph companion to this
section.

The remainder of this document (§1 onward) treats `uvpacket` as a
modem on its own terms — design choices, characterisation,
implementation loss.

## 1. Scope

### 1.1 What this is — a modem, layered alongside the WSJT family

`mfsk-core`'s primary scope is the WSJT-X family of digital
modes (FT8/FT4/FST4/WSPR/JT9/JT65/Q65). uvpacket is **not** a
member of that family — it has its own modulation, sync, demod,
and message conventions, and it bypasses the generic mfsk-core
TX/RX pipeline. It lives in-tree because it reuses the FEC
layer (`Ldpc240_101` from FST4 + the BP/OSD machinery), and
spinning it out as a sibling crate would add maintenance cost
disproportionate to the deliverable.

The trade-off: the `Protocol::ID = ProtocolId::UvPacket` and
several `ModulationParams` constants (`NTONES = 4`, `GFSK_BT`,
`TONE_SPACING_HZ`, `GFSK_HMOD`) are decorative for uvpacket —
they exist only to satisfy the trait signature and the
`protocol_invariants` test, and are never consulted by
[`tx::encode`] or `rx::decode_*`. This is documented at
[`mfsk-core/src/uvpacket/protocol.rs`](../../mfsk-core/src/uvpacket/protocol.rs)
and [`docs/reference/LIBRARY.md`](LIBRARY.md) §10.1 as a scope-boundary
trade-off rather than disguised.

The crate ships **only the modem**:

- `tx::encode(header, payload, audio_centre_hz) -> Result<Vec<f32>, PackError>`
  — the convenience wrapper that allocates the output buffer.
- `tx::encode_into(out, header, payload, audio_centre_hz) -> Result<(), PackError>`
  + `tx::encode_output_len(mode, n_payload_blocks)` — caller-buffer
  TX (added in 0.4.1 alongside the embedded port) for I2S DMA-style
  use without per-burst `Vec` allocations.
- `rx::decode_known_layout(audio, sample_offset, audio_centre_hz, mode, &fec_opts)`
  — the basic decode entry; pass `default_fec_opts()` for OSD-2 /
  bp_max_iter = 50, or build a custom `FecOpts` for OSD-3 and
  caller-side AP masking.
- `rx::decode_known_layout_with_afc(.., &afc_opts)` — same but
  prepends an AFC sweep over `±afc_opts.search_hz` (default ±200 Hz).
- `rx::decode(audio, audio_centre_hz) -> Vec<DecodedFrame>` —
  auto-detect entry: scans the passband, identifies sync peaks for
  any of the four preamble variants, and decodes each at the mode
  the winning preamble selected.
- `rx::decode_multichannel(audio, &mc_opts, &fec_opts)` and
  `rx::measure_slot_energies(audio, &mc_opts, slot_spacing_hz)` —
  multi-channel passband scan + per-slot energy survey for LBT
  (see §3.10).

Frame composition, application-level dispatch, key management etc.
are the embedder's job.

### 1.2 What this is not

- Not a peer WSJT mode. The layering above is not coincidence —
  WSJT modes share callsign-message conventions, slot-aligned
  framing, structured message codecs; uvpacket shares none of
  that.
- Not an interoperable packet mode. No standardisation, no TNC
  support — designed for private groups running the same
  software on both ends.
- Not a voice mode. Data only.
- Not a wideband mode. Fits in a 3 kHz audio passband, nominal
  1008–1800 net bps. Different design point from M17 / D-STAR /
  DMR / VARA FM.
- Not a signed-QSL implementation. See §1.4.

### 1.3 Where it sits

A factual peer comparison for U/VHF private-group messaging at
~1 kbps in a 3 kHz audio passband:

| | Channel | FEC | Net bps | 90 % PER threshold (AWGN) | Open source |
|---|---|---|---:|---:|:-:|
| AX.25 / AFSK 1200 | NFM | none | 1200 | +10 dB SNR_3kHz | ✓ |
| PSK31 | SSB | none | ~50 | −10 dB SNR_2.5kHz | ✓ |
| Olivia 4/250 | SSB | conv + interleave | ~50 | −13 dB SNR_500Hz | ✓ |
| **uvpacket UltraRobust** | **NFM or SSB** | **LDPC + OSD, half-baud** | **504** | **−3.7 dB SNR_3kHz** | ✓ |
| uvpacket Robust | NFM or SSB | LDPC + OSD | 1008 | +1.3 dB SNR_3kHz | ✓ |
| uvpacket Standard | NFM or SSB | LDPC + OSD | 1200 | +4.0 dB SNR_3kHz | ✓ |
| uvpacket Express | NFM or SSB | LDPC + OSD | 1800 | +7.8 dB SNR_3kHz | ✓ |
| M17 4-FSK | 9 kHz | conv | 4800 | +5–7 dB SNR | ✓ |
| VARA FM | 12.5 kHz | proprietary | ~25000 | +10 dB SNR | ✗ |
| VARA HF | 2.4 kHz SSB | proprietary | 5000–25000 | varies | ✗ |
| D-STAR DV | 6.25 kHz | Golay | 4800 voice + 1200 data | ~+10 dB CNR | partial |
| DMR / NXDN | 6.25–12.5 kHz | BCH | 4-FSK voice + data | ~+7–8 dB CNR | (commercial) |

The closest peers in design space (open-source, FEC-coded,
audio-passband data on NFM/SSB) are AX.25 and PSK31; everything
to the right of the bold rows is in a different design space
(wider channels, voice-primary, or proprietary).

Versus AX.25 in NFM: 16 % less raw throughput, ~14 dB better
SNR threshold, real FEC and block-interleaver against fade
bursts. Versus PSK31 in SSB: 20 × more throughput at ~6 dB
worse threshold — different niche of the same passband.

uvpacket does not compete with VARA / M17 / D-STAR-class
protocols on speed; those use wider channels and / or carry
voice as well. It does not compete with FT8 / JS8 / Olivia on
extreme weak signals; those run at orders-of-magnitude lower
throughput.

The deliverable: an open-source, FEC-coded, modern packet modem
that fits the **3 kHz NFM / SSB voice passband** and works
across both, at sub-second burst duration with a four-step
opportunistic-throughput rate ladder.

### 1.4 Application architecture — modem here, apps elsewhere

The flagship application uvpacket was conceived for is **signed
QSL exchange between private groups**. The intended layering:

```text
┌─────────────────────────────────────────┐
│  Application (signed QSL, position,     │  app-layer repo,
│  short text, ARQ-ACK …)                 │  separate from
│  e.g. browser-WASM PWA via wasm-bindgen │  mfsk-core
├─────────────────────────────────────────┤
│  mfsk-core::uvpacket  (this crate)      │  modem only:
│  — tx::encode / rx::decode_*            │  bytes ↔ audio
│                                         │  with FEC
└─────────────────────────────────────────┘
              ↕  (audio over the air, via the radio)
```

The modem deliberately exposes a **byte-pipe API** with a 4-bit
`app_type` dispatch tag. Callers pick a meaning per `app_type`;
the modem is opaque to it. The suggested allocation in §2.5 is
a convention, not a contract.

For the signed-QSL flagship, the planned application crate is a
**browser-WASM PWA** (sibling repo, distinct from `mfsk-core`):
Web Audio for I/O, Web Crypto for signing, IndexedDB / OPFS for
key + log storage, `mfsk-core` linked via `wasm-bindgen` (with
`--features uvpacket`). Keeping the signed-QSL surface out of
this crate avoids coupling the modem's release cadence to UX
iteration and keeps the published artefact small.

Native (non-browser) embedders use the same byte-pipe API — the
application layer is whatever protocol you bolt on top.

## 2. Design

### 2.1 Modulation

Single-carrier **π/4-shifted DQPSK** with root-raised-cosine pulse
(α = 0.35, span 6 sym), audio centre 1500 Hz at 12 kHz sample rate.
Symbol rate is **1200 baud** (Robust / Standard / Express) or
**600 baud** (UltraRobust, see §2.3). Information bits are
differentially encoded onto the constellation rotation Δφ ∈
{±π/4, ±3π/4}; the RX recovers bits via 1-symbol conjugate product
followed by Gray-demapping after −π/4 derotation.

The π/4-DQPSK + differential demod combination removes the need
for an absolute carrier-phase reference — all phase impairments
that are slow relative to the symbol period (LO walk, clarifier
offset within the AFC range, group-delay tilt) cancel in the
1-symbol product. This is the key change from the 0.3 coherent-QPSK
pipeline that failed over-the-air despite passing AWGN bench.

### 2.2 Preamble — mode-encoded m-sequence + equaliser fit

Frame head is a **127-chip BPSK m-sequence** (Fibonacci LFSR,
length-7, one of four primitive polynomials selected by
[`Mode`]). 127 chips × 1 sym/chip = 106 ms at 1200 baud (212 ms at
600 baud for UltraRobust). The polynomial choice **encodes the
mode** in the preamble itself, replacing the 0.3-era brute-force
LDPC layout sweep — the RX runs four cross-correlations and picks
the winner, deterministic 1+n_blocks decode cost.

Cyclic autocorrelation sidelobes for length-127 m-sequences are
bounded by 1/127 ≈ −22 dB amplitude — clean enough to also fit a
**9-tap T-spaced LMS equaliser** in closed form against the known
preamble, recovered residual rotation, and pre-rotation timing
correction during the same preamble window. There are **no pilot
symbols** in the payload; the equaliser plus the 1-symbol
differential demod absorbs the residual channel for the entire
frame.

### 2.3 FEC

Reuses [`Ldpc240_101`] from FST4 as the rate-0.42 mother code (101
info bits → 240 channel bits per block). The four sub-modes apply
puncturing chosen by **kSR-greedy puncture-set selection**
(Ha–McLaughlin) to the 139 parity bits:

| Sub-mode | Baud | rate | Puncture | Net bps | Posture |
|---|---:|---:|---:|---:|---|
| UltraRobust | 600 | 0.42 | 0 % | 504 | weak-signal / marathon QSL (half-baud) |
| Robust | 1200 | 0.42 | 0 % | 1008 | maximum-margin at full baud |
| Standard | 1200 | 0.50 | 30 % | 1200 | typical NFM with fading |
| Express | 1200 | 0.75 | 76 % | 1800 | strong-signal headline (OSD-3 mandatory) |

UltraRobust shares the same FEC rate as Robust but doubles the
symbol period — every per-symbol energy doubles, fading averaging
improves, and tap-delay multipath becomes shorter relative to the
symbol period. The four-mode ladder spans a ~12 dB SNR_3kHz range
end-to-end (see §3).

kSR-greedy delivers ~1–3 dB Eb/N0 gain over uniform-spread
puncturing at the deeper rates, which is what makes Express viable
at all (uniform-spread fails to converge at 76 % parity puncture).

A **dedicated header LDPC block** (Ldpc240_101 unpunctured, equal
to Robust/UltraRobust rate) carries the 4-byte frame header
separately from the payload — header recovery is independent of
payload-block decode order and the puncturing depth.

### 2.4 Frame structure

- Variable length: 1–32 LDPC blocks per frame.
- **Header LDPC block** (Ldpc240_101 unpunctured) carries the
  4-byte frame header. Mode is **identified by the preamble
  polynomial** and is therefore not stored as a header field —
  the header carries block_count (5b) + app_type (4b) + sequence
  (5b) + reserved (2b) + CRC-16 (16b).
- Each payload LDPC block carries 96 info bits (12 byte) padded
  to the FEC's 101-bit input.
- **Block-interleaver** across all codewords in the frame spreads
  fade-burst erasures across every codeword.

### 2.5 Application API

Byte-pipe — bypasses `mfsk-core`'s `MessageCodec`. Callers deliver
raw bytes plus a 4-bit `app_type` tag. The modem does not interpret
the bytes. Suggested allocation:

| `app_type` | Use |
|---:|---|
| 0 | raw / experimentation |
| 1 | signed QSL exchange |
| 2 | position beacon |
| 3 | short text |
| 4 | ARQ ACK |
| 5–15 | user-defined |

## 3. Characterisation

### 3.1 Mode positioning summary

`tests/uvpacket_per_modes_sweep.rs` (`#[ignore]`, run with
`cargo test --release --test uvpacket_per_modes_sweep <name>
-- --ignored --nocapture`), 30 trials per cell, 4-block frame,
16-byte payload, π/4-DQPSK + LMS equaliser + OSD-2.

**90 % PER threshold (≥ 27/30 decoded), Eb/N0_info / SNR_3kHz dB:**

| Mode (net bps) | AWGN | Rayleigh fd=5 Hz | SSB realistic | FM realistic | Multipath 3-tap |
|---|---:|---:|---:|---:|---:|
| **UltraRobust** (504) | **+4 / −3.7** | **+8 / +0.3** | **+4 / −3.7** | **+6 / −1.7** | **+6 / −1.7** |
| Robust (1008) | +6 / +1.3 | +12 / +7.3 | +8 / +3.3 | +10 / +5.3 | +8 / +3.3 |
| Standard (1200) | +8 / +4.0 | +12 / +8.0 | +8 / +4.0 | +10 / +6.0 | +10 / +6.0 |
| Express (1800) | +10 / +7.8 | +20 / +17.8 | >+15 / >+12.8 | +20 / +17.8 | **fail** |

(SNR_3kHz = Eb/N0_info + 10·log₁₀(R_info / 3000); R_info per mode:
504 / 1008 / 1200 / 1800 bps → −7.74 / −4.74 / −3.98 / −2.22 dB.)

UltraRobust holds a uniform 4 dB margin over Robust on every
fading channel (Rayleigh, SSB, FM) and a 2 dB margin on AWGN /
multipath. Express collapses on multi-tap multipath even at +20 dB
Eb/N0 — the equaliser's 9 taps span ~7.5 ms at 1200 baud and
cannot resolve the 15 ms tap, while the 600-baud UltraRobust gives
the equaliser ~13 ms of T-spaced reach.

### 3.2 LDPC layer (modem-bypassed reference)

`tests/uvpacket_ldpc_direct.rs` feeds Gaussian-noise LLRs straight
into the LDPC decoder, calibrated for `Eb/N0_info` per channel bit.
This isolates the FEC from the modem and gives the **theoretical
ceiling** the modem end-to-end pipeline aspires to.

50 % PER thresholds: UltraRobust / Robust ≈ +0.5 dB (same FEC),
Standard ≈ +0.7 dB, Express ≈ +1.5 dB. The mother code's design
rate is 0.42 so Robust / UltraRobust hold a ~1 dB lead at the FEC
layer. The π/4-DQPSK end-to-end thresholds in §3.1 sit ~3 dB above
this LDPC-only ceiling — that 3 dB gap is the irreducible
**non-coherent-vs-coherent gap** of differential demodulation,
the price paid for surviving the over-the-air phase-impairment
stack (see §4).

### 3.3 AWGN sweep

```text
mode         Eb/N0 (dB)  -2   0   2   4   6   8  10
─────────────────────────────────────────────────────
UltraRobust               0   0   6  29  30  30  30
Robust                    0   0   0   1  25  30  30
Standard                  0   0   0   0  19  30  30
Express                   0   0   0   0   0  21  30
```

90 % PER thresholds: UltraRobust ≈ +4 dB, Robust ≈ +6 dB,
Standard ≈ +8 dB, Express ≈ +10 dB. The textbook rate ordering
(lower rate → lower threshold) is recovered, plus a 2 dB
half-baud bonus for UltraRobust.

### 3.4 Rayleigh flat fading

```text
mode         fd (Hz)  +4   +8  +12  +16  +20  +25   (Eb/N0_info dB)
────────────────────────────────────────────────────────
UltraRobust    1       4   22   29   30   30   30
UltraRobust    5       3   29   30   30   30   30
UltraRobust   10       4   29   30   30   30   30
Robust         1       0   10   26   30   30   30
Robust         5       0    7   29   30   30   30
Robust        10       0   10   30   30   30   30
Standard       1       1   11   24   30   30   30
Standard       5       0    6   26   30   30   30
Standard      10       0    7   30   30   30   30
Express        1       0    3   10   20   27   28
Express        5       0    0    3   18   29   30
Express       10       0    0    2   22   28   29
```

≥ 90 % PER thresholds: UltraRobust ≈ +8 dB across all Doppler;
Robust / Standard ≈ +12 dB; Express ≈ +20 dB. Doppler dependence
is mild for the lower three modes (differential demod absorbs
slow phase drift); Express is the only mode where 1 Hz vs 10 Hz
matters at threshold.

### 3.5 SSB realistic — clarifier offset + LO walk + light multipath

Channel: BPF (300, 2700) Hz with 100 Hz transition, clarifier
offset 100 Hz (within AFC range), LO phase walk 2 rad/√s,
single 5 ms multipath tap at −10 dB.

```text
mode         Eb/N0 (dB)  +4   +6   +8  +10  +12  +15
─────────────────────────────────────────────────────
UltraRobust              21   30   30   30   30   30
Robust                    0    7   28   30   30   30
Standard                  0    1   23   30   30   30
Express                   0    0    1    7   17   27
```

UltraRobust is **even with its AWGN threshold (+4 dB)** here —
the half-baud symbol period gives the equaliser enough time to
absorb the multipath tap and the differential demod is invariant
to the LO walk.

### 3.6 FM realistic — de-emphasis + discriminator drift + Rician

Channel: 75 µs de-emphasis, discriminator DC drift 50 Hz, LO walk
1 rad/√s, Rician K = 10 dB, single 5 ms multipath tap at −10 dB.

```text
mode         Eb/N0 (dB)  +6   +8  +10  +12  +15  +20
─────────────────────────────────────────────────────
UltraRobust              27   30   30   30   30   30
Robust                    0    9   28   30   30   30
Standard                  0    2   24   30   30   30
Express                   0    0    1    5   17   26
```

UltraRobust ~4 dB ahead of Robust again. Express is barely usable
on FM due to the de-emphasis tilt convolved with multipath.

### 3.7 Pure multi-tap multipath (3 + 8 + 15 ms)

Multipath stress test, no other impairments beyond AWGN.
Isolates the equaliser's reach.

```text
mode         Eb/N0 (dB)  +6   +8  +10  +12  +15  +20
─────────────────────────────────────────────────────
UltraRobust              30   30   30   30   30   30
Robust                    0   20   30   30   30   30
Standard                  0   19   28   30   30   30
Express                   0    0    0    0    6    8
```

UltraRobust is **floor-limited** even at +6 dB Eb/N0 — the 600-baud
symbol period (~1.67 ms) lets the 9-tap equaliser cover ~13 ms
T-spaced, comfortably wider than the 15 ms tail tap. Express
collapses entirely: 9 taps at 1200 baud cover ~7.5 ms and cannot
resolve the longer taps.

### 3.8 The FM-threshold floor — and why it makes the modem
###     implementation loss operationally invisible

The modem sits on top of FM detection. Below CNR ≈ +9–10 dB the
FM discriminator output is dominated by impulse noise and **any**
audio-domain modem fails catastrophically. The audio-domain Eb/N0
numbers above are meaningful only above the FM threshold.

**At the FM threshold**, post-detection audio SNR (in a 3 kHz
passband) is roughly `CNR_threshold + FM_SNR_improvement ≈ +9 +
10·log₁₀(B_IF/B_audio · 3) ≈ +9 + 11 ≈ +20 dB SNR_3kHz`.

Translating uvpacket UltraRobust's 90 % PER threshold (+4 dB
Eb/N0_info) to the same units:

```text
SNR_3kHz_UltraRobust = +4 + 10·log₁₀(504 / 3000) = −3.7 dB
```

Margin from the FM threshold floor down to the UltraRobust modem
threshold: **~+24 dB**. The residual modem implementation loss
is operationally invisible — it sits well below the channel's own
irreducible CNR floor, where no audio modem of any kind decodes.

The FM threshold is the binding constraint for NFM voice
channels.

### 3.9 SSB compatibility — and AFC

The modem is an audio-domain π/4-DQPSK + RRC processor (signal
occupies ~1600 Hz around the 1500 Hz centre at α = 0.35, well
inside a typical SSB passband). On SSB the FM-threshold floor
goes away and the modem operates at its true ~−3.7 dB SNR_3kHz
UltraRobust threshold — a useful weak-signal data envelope,
especially on HF.

**AFC entry point.** Use
[`decode_known_layout_with_afc(audio, .., &AfcOpts)`](https://docs.rs/mfsk-core/latest/mfsk_core/uvpacket/rx/fn.decode_known_layout_with_afc.html)
for SSB use; the default `decode_known_layout` assumes
`audio_centre_hz` is exact (right for NFM where TX/RX share the
same audio centre).

The AFC algorithm sweeps `audio_centre_hz + Δf_test` in 25 Hz
steps across `[−search_hz, +search_hz]` (default ±200 Hz),
runs the matched filter at each candidate, and picks the Δf
where the preamble-correlation magnitude peaks. Parabolic
refinement of the 3-point coarse-grid magnitudes drives the
final Δf to within a fraction of the grid spacing. Cost is
~17× single-decode cost (~50–100 ms in release mode); the
existing LMS phase fit downstream absorbs the sub-grid
residual without trouble.

The naive FFT-over-chip-rate-samples approach (cheap but wrong)
fails because the integer-sample preamble correlator that picks
`best_off` itself rolls off as `sinc(δ · 31 / 1200)`, landing on
noise samples for `|δ| ≳ 20 Hz`. The frequency-grid search
sidesteps this — the correlator magnitude itself peaks at the
correct Δf.

NFM users can keep using `decode_known_layout` (AFC is pure
overhead on a static-VFO channel).

### 3.10 Multi-channel SSB + slotted-ALOHA TX

A single uvpacket signal occupies `R_s · (1+α) = 1200 · 1.5 =
1800 Hz` end-to-end (RRC roll-off included; the −3 dB main
lobe is ~600 Hz). Practical adjacent-slot separation for
< −20 dB inter-slot interference is **1200 Hz**: at that
spacing one signal's spectrum lands within the next slot's
RRC roll-off zero. In a 2.4 kHz SSB passband that's room for
**two** simultaneous uvpacket frames (typically at 800 Hz
and 2000 Hz audio centres).

`mfsk-core::uvpacket::rx` ships two stateless primitives:

```rust
# #[cfg(feature = "uvpacket")] {
use mfsk_core::uvpacket::framing::FrameHeader;
use mfsk_core::uvpacket::rx::{
    MultiChannelOpts, decode_multichannel, default_fec_opts, measure_slot_energies,
};
use mfsk_core::uvpacket::{Mode, tx};

// RX: decode every frame in the passband, return each frame's
// detected audio centre.
let header = FrameHeader { mode: Mode::Robust, block_count: 4, app_type: 0, sequence: 1 };
let payload: Vec<u8> = (0..20).map(|i| (i ^ 0x5A) as u8).collect();
let audio = tx::encode(&header, &payload, /* audio_centre_hz */ 800.0).unwrap();

let mc_opts = MultiChannelOpts::default();
let fec_opts = default_fec_opts();
let frames = decode_multichannel(&audio, &mc_opts, &fec_opts);
assert!(!frames.is_empty(), "roundtrip must decode");
for (centre_hz, frame) in &frames {
    println!("{:7.1} Hz  {} byte payload", centre_hz, frame.payload.len());
}

// TX-side LBT: per-slot mean MF magnitude survey.
let energies = measure_slot_energies(&audio, &mc_opts, /*slot_spacing_hz*/ 1200.0);
for e in &energies {
    println!("{:7.1} Hz  mean|MF|^2={:.4}", e.audio_centre_hz, e.mean_mf_magnitude);
}
# }
```

`decode_multichannel` runs a coarse-grid frequency scan
(default 25 Hz step over 300–2700 Hz), takes per-grid-point
preamble correlation peaks, applies frequency-axis NMS at
`nms_radius_hz` (default 600 Hz, half slot spacing), and
decodes each survivor at its picked centre via
`decode_known_layout` (no inner AFC needed — the LMS phase
fit absorbs the ≤ 12.5 Hz coarse-grid residual).

`measure_slot_energies` reports the mean matched-filter
|output|² at each `slot_spacing_hz`-spaced slot centre.
Policy-free — callers filter by their own threshold (typical:
≤ 3 dB above band median = "free") and pick uniformly at
random among free slots for TX.

The TX side keeps the existing
`tx::encode(&header, &payload, audio_centre_hz)`. Operating
concept: each TX picks a random free slot via LBT, transmits
there, waits for an application-layer ARQ ACK. Collisions
retry with a fresh random pick — **slotted ALOHA on the
audio-frequency axis**.

This formalises the natural amateur-radio "watch the
frequency, find a clear spot, transmit" practice. CSMA/**CD**
proper isn't applicable to half-duplex SSB radio (can't
reliably listen while keying the transmitter); slotted ALOHA
+ LBT + ARQ at the application layer behaves equivalently
with much less mechanism.

mfsk-core ships no RNG dependency and no state machine — the
application supplies randomness (`rand::Rng`, browser
`crypto.getRandomValues`, …) and owns the ARQ + retry policy.

Empirical: two simultaneous frames at 800/2000 Hz centres
decode cleanly with detected centres within ±50 Hz of truth;
same setup at +8 dB Eb/N0_info AWGN: both decoded; slot
survey with one busy slot: busy mag > 5× free mag.

## 4. Modem implementation loss

The gap between the LDPC-only threshold (§3.2) and the π/4-DQPSK
end-to-end threshold (§3.3) is **~3 dB at AWGN** — this is the
irreducible non-coherent-vs-coherent gap of differential
demodulation, not engineering slack. The 0.4 redesign accepted
that gap as the price for surviving the over-the-air
phase-impairment stack that broke 0.3's coherent QPSK pipeline.

The current rx implements:

- **127-chip BPSK preamble cross-correlation × 4 polynomials** —
  one cross-correlation per Mode (UltraRobust / Robust / Standard
  / Express). The polynomial that wins identifies the mode without
  needing to try all LDPC layouts; decode cost becomes deterministic
  1+n_blocks regardless of mode confusion.
- **9-tap T-spaced LMS equaliser** fitted in closed form against
  the known preamble (least-squares solve, no iterative
  adaptation). Reach is ~7.5 ms at 1200 baud, ~13 ms at 600 baud
  — the latter resolves typical multi-tap multipath that 1200-baud
  modes cannot.
- **Residual rotation estimate** from the preamble's complex
  signed mean, applied before the −π/4 derotation. Absorbs
  clarifier offset within the AFC range without a per-symbol PLL.
- **1-symbol differential demodulation**: r_diff[n] = r[n] · r[n−1]*.
  Bits recovered by Gray-demapping after −π/4 derotation. No pilot
  insertion in the payload; no carrier-phase tracker.
- **σ-aware LLR scaling** from a magnitude-based σ²_n estimator on
  the differential samples.
- **OSD-2** by default (via `default_fec_opts()`);
  `decode_known_layout` takes `&FecOpts` for callers who want OSD-3
  (~30 × slower per decode, ~10–15 % better PER near threshold for
  the higher-rate modes) or caller-side AP masking.
- **Dedicated header LDPC block** (Ldpc240_101 unpunctured) so
  header recovery is independent of payload puncturing depth.

These choices trade ~3 dB AWGN headroom for the ability to
operate without an absolute phase reference. On every fading or
phase-walk channel measured in §3.4–3.7 that trade pays — the
0.3 coherent pipeline did not survive any of those channels at
field-realistic settings.

## 5. Modulation pivot history

- **0.3.0**: design abandoned after honest AFSK1200 / AX.25
  comparison showed it ~5–10 × faster on clean channels for the
  signed-QSL payload. For a 214-byte payload (JSON 126 B +
  Base64 sig 88 B):

  | Transport                  | Channel rate | Net rate  | 214 B airtime |
  |----------------------------|-------------:|----------:|--------------:|
  | AX.25 / AFSK 1200          |     1200 bps | ~1100 bps | ~2 s + TXDELAY 0.5 s |
  | UvPacket300 (LDPC + chain) |      600 bps |  ~200 bps | ~10 s |
  | UvPacket150 (LDPC + chain) |      300 bps |  ~100 bps | ~18 s |

  The order-of-magnitude airtime cost only pays off on channels
  AFSK 1200 cannot deliver (Rayleigh fade, mountain / portable
  weak SSB) — "drop-in AFSK replacement" was the wrong framing.
- **0.3.1 Phase 1**: 4-GFSK at h = 0.5. Phase 2 found
  `sinc(0.5) ≈ 0.637` left adjacent tones non-orthogonal under
  non-coherent detection (textbook condition is h ≥ 1).
- **0.3.1 Phase 2 → 0.3.3**: pivoted to coherent QPSK + RRC +
  LMS phase tracker. Bench-passed AWGN / Rayleigh sims but failed
  over-the-air on SSB and FM voice paths despite repeated
  patching (AFC, coherence-ratio gate, 1-shot AFC).
- **0.4.0**: replaced coherent QPSK with **π/4-DQPSK + LMS
  equaliser + 4-variant 127-chip preamble + dedicated header
  block + UltraRobust half-baud mode**. The 3 dB non-coherent gap
  is structural; the 5–10 dB phase-impairment loss it eliminates
  is much larger. See `docs/0.3.1_PLAN.md` and the pre-0.4 plan
  files for the chronology.

The σ formula in `tests/common/channel.rs` was also recalibrated
in Phase 2'a to take per-burst measured signal power, so stated
Eb/N0_info numbers are now cross-modulation comparable.

## 6. Audio samples

Representative WAV files for ear-level inspection live at
`audio_samples/uvpacket/` in this repository. All are 12 kHz mono
16-bit PCM, with 200 ms of leading/trailing silence:

| File | Mode | Channel | Decode |
|---|---|---|:-:|
| `uv_robust_clean.wav` | Robust, 4 blocks, 20 B | clean | ✓ |
| `uv_robust_awgn_+8db.wav` | Robust | AWGN +8 dB Eb/N0 | ✓ |
| `uv_robust_awgn_+4db.wav` | Robust | AWGN +4 dB Eb/N0 | ✓ (97 % per-frame after LMS) |
| `uv_robust_awgn_+2db.wav` | Robust | AWGN +2 dB Eb/N0 | ✓ (53 % per-frame statistic; this seed lands on the OK side after sub-sample timing) |
| `uv_robust_rayleigh_5hz_+15db.wav` | Robust | 5 Hz Rayleigh, +15 dB | ✓ |
| `uv_express_clean.wav` | Express, 4 blocks, 20 B | clean | ✓ |

To regenerate them:

```sh
cargo run --release --features uvpacket --example uvpacket_samples
```

The clean Robust burst is ~440 ms; Express is ~270 ms. The audible
character is "narrow-band data buzz" — the RRC pulse spreads each
QPSK symbol across multiple tones, giving an approximately flat
spectrum across `[1500 − 600, 1500 + 600] Hz` with raised-cosine
shoulders.

## 7. Implementation pointers

| Layer | File |
|---|---|
| Protocol ZSTs / sub-mode parameters | [`mfsk-core/src/uvpacket/protocol.rs`](../../mfsk-core/src/uvpacket/protocol.rs) |
| Frame header + CRC + bit packing | [`mfsk-core/src/uvpacket/framing.rs`](../../mfsk-core/src/uvpacket/framing.rs) |
| Puncture sets (kSR-greedy) | [`mfsk-core/src/uvpacket/puncture.rs`](../../mfsk-core/src/uvpacket/puncture.rs) |
| Block interleaver | [`mfsk-core/src/uvpacket/interleaver.rs`](../../mfsk-core/src/uvpacket/interleaver.rs) |
| Preamble polynomials (4 variants) | [`mfsk-core/src/uvpacket/sync_pattern.rs`](../../mfsk-core/src/uvpacket/sync_pattern.rs) |
| TX (bytes → audio) | [`mfsk-core/src/uvpacket/tx.rs`](../../mfsk-core/src/uvpacket/tx.rs) |
| RX (audio → bytes), equaliser | [`mfsk-core/src/uvpacket/rx.rs`](../../mfsk-core/src/uvpacket/rx.rs) |
| AWGN + Rayleigh harness | [`mfsk-core/tests/common/channel.rs`](../../mfsk-core/tests/common/channel.rs) |
| SSB / FM realistic channel sims | [`mfsk-core/tests/common/air_channel.rs`](../../mfsk-core/tests/common/air_channel.rs) |
| LDPC-only sweep (modem-bypassed) | [`mfsk-core/tests/uvpacket_ldpc_direct.rs`](../../mfsk-core/tests/uvpacket_ldpc_direct.rs) |
| Modem TX/RX diagnostics | [`mfsk-core/tests/uvpacket_modem_diag.rs`](../../mfsk-core/tests/uvpacket_modem_diag.rs) |
| 4 modes × 5 channels PER sweep | [`mfsk-core/tests/uvpacket_per_modes_sweep.rs`](../../mfsk-core/tests/uvpacket_per_modes_sweep.rs) |

## 8. License

GPL-3.0-or-later, matching the rest of `mfsk-core`. The LDPC mother
code is derived from WSJT-X (`lib/fst4/`).
