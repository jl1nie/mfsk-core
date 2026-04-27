# uvpacket — design and implementation deep dive

This document is the engineering reference for the `uvpacket`
protocol family. The high-level motivation lives in
[`LIBRARY.md` §11](LIBRARY.md#11-uvpacket--motivating-worked-example)
— here we cover the parts that are too detailed for the architecture
guide: the interleaver math, sync-layout derivation, channel-model
derivation, fading test methodology, and the rate-ladder ceiling.

## Channel model — flat-band time-selective Rayleigh fading

uvpacket targets U/VHF mobile / portable amateur operation. At 144
or 433 MHz with mobile-class velocities (10 – 150 km/h), the
channel impairments break down as follows:

- **Multipath delay spread**: typically 0.1 – 1 μs in suburban /
  open terrain, ≤ 5 μs in dense urban canyons. The coherence
  bandwidth is the inverse of the delay spread, so ~1 MHz at the
  worst end. A 3 kHz audio passband is **flat-fading** by three
  orders of magnitude — the entire band fades together as one
  complex gain `h(t)`.
- **Doppler spread**: vehicle velocity / wavelength, e.g. 50 km/h
  at 144 MHz gives 6.7 Hz Doppler. The coherence time is
  `9 / (16π fd)` ≈ 27 ms at that Doppler, but typical fade nulls
  span **multiple coherence times** because the Rayleigh envelope
  produces deep nulls only when both I and Q quadratures cross
  through small magnitudes. Practical fade-null durations of
  50 – 200 ms are routine.
- **AWGN floor**: receiver noise figure 1 – 5 dB, ambient EMI
  modest. Operating SNR in the 0 – 20 dB range covers the
  interesting cases.

Frequency-flat time-selective Rayleigh fading is well-modelled by
two independent low-pass-filtered Gaussian processes (in-phase
and quadrature), with cutoff equal to the Doppler frequency. The
test scaffolding in `tests/uvpacket_roundtrip.rs` implements
exactly this model — see `apply_rayleigh_fade` for the
first-order IIR construction with variance scaling that keeps
`E|h|² = 1` for unit-power channels.

## Interleaver — stride-25 polynomial over 174 bits

The TX side reads codeword bit `INTERLEAVE[j]` into channel-bit
position `j`; the RX side does the inverse. The table is
generated at compile time by:

```rust
const UVPACKET_INTERLEAVE: [u16; 174] = {
    let mut arr = [0u16; 174];
    let mut j = 0usize;
    while j < 174 {
        arr[j] = ((j * 7) % 174) as u16;
        j += 1;
    }
    arr
};
```

### Why stride 25 (with inverse 7)?

We want a permutation π : {0..174} → {0..174} satisfying:

1. **Surjective** — every codeword bit must appear once in the
   channel stream. Equivalently, gcd(stride, 174) = 1.
2. **Far apart** — consecutive codeword bits (i, i+1) must land
   far apart in the channel stream so a contiguous burst of B
   channel-bit losses produces sparse codeword-bit losses.
3. **Reversible cheaply** — both TX and RX should compute the
   permutation as `(stride · index) mod 174` rather than carry a
   precomputed inverse table. Equivalently, the multiplicative
   inverse of the stride mod 174 should also be a small integer.

Candidates (gcd = 1 with 174 = 2 · 3 · 29):

| Stride | Inverse mod 174 | Notes                                         |
|-------:|----------------:|-----------------------------------------------|
|     5  |             35  | Inverse too large, expensive on every iter.   |
|     7  |             25  | Symmetric pair — see stride 25.               |
|    11  |             95  | Inverse very large.                           |
|    25  |              7  | **Chosen** — large stride, tiny inverse.      |
|    31  |             129 | Inverse very large.                           |

Stride 25 puts consecutive codeword bits 25 channel positions
apart. With 4-FSK at 2 bits-per-symbol, that's ≈ 12 symbols of
separation. After deinterleaving on the RX side, a contiguous
burst of `B` channel-bit losses spreads into `B` codeword-bit
losses at stride 7 (since 25 × 7 ≡ 1 mod 174).

### Burst-loss tolerance, worked

For a fade null of duration `T` ms at sub-mode with NSPS samples
per symbol at 12 kHz audio, the channel-bit loss count is:

```
B = floor(T · 12000 / NSPS) · 2     // 2 bits per 4-FSK symbol
```

After deinterleave, those `B` losses occupy `B` codeword positions
spread across the 174-bit codeword. Sample numbers:

| Sub-mode      | Fade T | NSPS | Symbols lost | Channel bits | Codeword bits affected |
|---------------|-------:|-----:|-------------:|-------------:|-----------------------:|
| UvPacket150   |  50 ms |   80 |          7.5 |           14 |                     14 |
| UvPacket150   | 200 ms |   80 |           30 |           60 |                     60 |
| UvPacket300   |  50 ms |   40 |           15 |           30 |                     30 |
| UvPacket600   |  50 ms |   20 |           30 |           60 |                     60 |
| UvPacket1200  |  50 ms |   10 |           60 |          120 |                    120 |

LDPC(174, 91) under soft-decision BP routinely corrects 25–35
random bit erasures at threshold SNR. The 14-bit and 30-bit
cases sit comfortably inside that envelope; the 60-bit case is
near threshold; the 120-bit case (UvPacket1200, 50 ms fade)
exceeds it — but at UvPacket1200 a 50 ms fade is *most of the
frame* (the frame is 83 ms) and decode failure is the right
outcome.

## Sync layout — head + mid-frame Costas-4

The `SYNC_BLOCKS` constant places two sync blocks:

```rust
pub const UVPACKET_SYNC_BLOCKS: [SyncBlock; 2] = [
    SyncBlock { start_symbol: 0,  pattern: &UVPACKET_SYNC_PATTERN },
    SyncBlock { start_symbol: 47, pattern: &UVPACKET_SYNC_PATTERN },
];
```

### Why two and not three?

Three sync blocks (head + mid + tail) would give symmetric
coverage and slightly improve the worst case where the fade
straddles head + mid (the third block at the tail still
provides a sync vote). The cost is +4 channel symbols, which
at UvPacket150 is +27 ms of frame time (+4.2%).

The current design picks two blocks with the second at
mid-frame because the asymmetry **matters for short frames**:
at UvPacket1200 the entire 95-symbol frame is 79 ms, and adding
a third block raises that to 86 ms — comparable to one
coherence time. Empirically the two-block layout passed every
fading test in `tests/uvpacket_roundtrip.rs`; if a future
deployment shows the head + mid combination losing correlation
under harsher fading (high-speed mobile, heavily reflective
terrain), the third block at the tail is a one-line change to
`UVPACKET_SYNC_BLOCKS`.

### Why the `[0, 1, 3, 2]` Costas pattern?

This is the pattern FT4 uses for its first sync block
(`FT4_COSTAS_A`). Costas arrays of length 4 are well-studied —
their autocorrelation has a single peak with secondary lobes
≤ 1, which is the property `coarse_sync` exploits. Reusing a
proven pattern avoids re-deriving the autocorrelation property
for our protocol. Both blocks share the pattern; the
correlations sum coherently when both are clean and act
independently when one is faded.

## Rate ladder — why 1200 baud is the ceiling

In a 12 kHz audio pipeline, the practical limits on M-FSK are:

- **Symbol-rate ceiling** = `12000 / NSPS`. NSPS = 10 yields
  1200 baud; NSPS = 5 yields 2400 baud but the 4-FSK tones
  spread `4 × 2400 = 9600 Hz`, which with a 1500 Hz carrier
  centre puts the highest tone at 9000 Hz — beyond the 6 kHz
  Nyquist. Aliasing at NSPS = 5 makes UvPacket2400 unreliable.
- **Tone-spacing ceiling** = `Nyquist / NTONES`, with margin.
  For 4-FSK, the highest-tone frequency is `centre + 3 × Δf`.
  Keeping it ≤ 5500 Hz with a 1500 Hz centre gives Δf ≤ 1333 Hz
  — UvPacket1200's 1200 Hz spacing fits, UvPacket1500 wouldn't.
- **Spectral efficiency ceiling** = the M-FSK family. To break
  through the 1.2 kbps net ceiling, the modulation scheme has to
  change. PSK / QAM at the same baud rate gives 2 – 6× the bits
  per symbol but requires a coherent receiver and is a different
  protocol family. OFDM is another option for ≥ 5 kbps in 3 kHz
  (VARA-style).

Targeting "several kbps" net rate is a separate project that
needs either:

1. A wider audio pipeline (24 / 48 kHz) — touches every DSP
   module in `mfsk-core/src/core`. Not undertaken for uvpacket.
2. Higher-order modulation (8-PSK, 16-QAM) — requires a coherent
   demodulator and entirely new TX / RX paths. The trait
   surface in `core::protocol` would need extending.

uvpacket sits at the practical edge of what M-FSK in a 12 kHz
audio pipeline can deliver.

## Test methodology

`mfsk-core/tests/uvpacket_roundtrip.rs` runs four channel
scenarios for every sub-mode plus targeted burst tests:

1. **Clean baseband** — pad-to-buffer, no impairments. Sanity
   checks the encoder, interleaver, sync layout, RX pipeline and
   `PacketBytesMessage` CRC-7 line up byte-for-byte.
2. **AWGN at moderate SNR** — additive white Gaussian noise at
   +5 dB (UvPacket300) and +10 dB (UvPacket1200, wider band so
   noise integrates over more BW). Confirms LDPC corrects
   scattered errors.
3. **Time-selective Rayleigh fading + AWGN** — the headline
   scenario. 5 Hz Doppler at +12 dB SNR (UvPacket300) and 2 Hz
   Doppler at +10 dB SNR (UvPacket150) span the typical mobile
   conditions. The Rayleigh envelope is generated by two
   independent first-order IIR-filtered Gaussian processes with
   variance-scaled to keep mean channel power at unity (then
   compensated for the √2 envelope-magnitude factor with
   `mean_gain = 1.4`).
4. **Burst-null sanity** — hard-zero the leading 50 ms of audio
   to wipe out the head Costas block and ~11 leading data
   symbols. Verifies the mid-frame Costas + interleaver still
   recovers the frame.

The fading scenario uses a deterministic LCG-seeded RNG so the
test is reproducible across runs. PRs that touch the
interleaver, sync layout, or LLR-deinterleave path need to keep
all four scenarios passing.

## Where this leaves you

uvpacket as shipped is suitable for:

- **Data beacons** — periodic 1–10 byte payloads (telemetry,
  position, status) at the appropriate sub-mode.
- **Short application messages** — combined with `chain.rs` for
  payloads up to ~512 bytes (8 bytes app-payload × 64 frames).
- **Multi-station shared channels** — the receiver is built
  around `coarse_sync`'s sliding-window correlation, so multiple
  concurrent transmitters at different audio centre frequencies
  decode automatically (subject to the channel's aggregate SNR
  budget).

It's deliberately not suitable for:

- **Real-time voice / video** — 1.2 kbps net is far below
  conversational codecs.
- **Bulk file transfer** — at 1.2 kbps a 1 MB file would take
  ~111 minutes; use VARA HF or 9k6 packet for that.
- **Non-flat fading** — long-delay-spread channels (HF skywave
  with ionospheric multipath) are out of scope; FST4 / WSPR /
  Q65's longer sync structures are the right tools there.
