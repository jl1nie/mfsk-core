# IQ channelizer — detailed design (issue #534, last item)

Status: **the PFB is designed, not built.** The selectivity target (§1) and
the `Direct` rework that came out of this study (§7a) are done. Every number
below is either measured on this tree or computed by
`scripts/iq_pfb_design.py` / `scripts/iq_pfb_chain.py` (numpy + scipy, at the
same 120 dB + 3 dB design margin the code uses); rerun them rather than
trusting the prose.

## 1. Why, and what has to hold

`IqReceiver` today mixes and decimates every channel from the input rate
(`IqToAudio`, the `Direct` path). Its cost is linear in channels: measured on
one thread, 768 kS/s is 0.93 % of a core per channel, about 30 % for 32. A
skimmer wants tens of channels across a band.

Requirements for the replacement front end, all testable:

| # | requirement | target |
|---|---|---|
| R1 | selectivity: any interferer outside the channel's audio −200…6200 Hz, anywhere in ±Fs/2, including between FFT bins | ≤ −120 dB into the used window |
| R2 | passband: gain across audio 200…5800 Hz, for every dial position | flat to 0.01 dB |
| R3 | timing: audio index 0 is IQ sample 0 | ±1 sample at 12 kHz |
| R4 | any dial, any integer Fs the `Direct` path accepts | same placement errors |
| R5 | retune / gap / anchor semantics | identical to `Direct` |
| R6 | decode equivalence | the WAV path's sets on every recording `iq_receiver*.rs` uses |
| R7 | cost | fixed part + small per-channel part; beats `Direct` from a handful of channels |

**Why 120 dB.** The noise floor of an ideal ADC in 2500 Hz, relative to a
full-scale sine: 8 bits −72 dBFS at 768 kS/s, 12 bits −96, 14 bits −108,
16 bits −120 (−77 / −101 / −113 / −125 at 2.4 MS/s). 100 dB would let a
full-scale interferer leak above the floor of any 14- or 16-bit SDR; 120 dB
reaches the 16-bit floor. f32 holds it: Kaiser taps rounded to f32 keep
−119.7 dB, and a 237-tap FIR run in f32 on a full-scale tone has an
arithmetic floor of −136 dBFS. Every filter is designed at 123 dB, because
Kaiser's order estimate falls short at the stop edge for short filters
(designed at exactly 120, a 71-tap stage let an alias through at −117.9 dB).

`Direct` before this study used Blackman windows and measured **−73.5 dB**
at its window edge; it now meets R1 (§7a).

## 2. What was tried first, and why it is not the design

An overlap-save fast convolution (one FFT per frame at ~25 Hz bins, each
channel a run of bins times a raised-cosine mask, one small IFFT) was written
and measured at 768 kS/s before this study. With the interferer **on** a bin
centre it rejects −160…−218 dB; **between** bins it rejects only
**−71 dB** (512.3 Hz below the dial) and **−87 dB** (9012.7 Hz above). The mask
was defined on bins, not designed as a time-domain FIR that fits the overlap,
so it is a frequency-sampled filter: exact on the bins, leaking between them,
plus circular wrap-around. That is the spectral leakage a PFB's prototype
filter exists to prevent. It fails R1 and is not merged.

(A corrected overlap-save, mask = FFT of a Kaiser FIR no longer than the
overlap, would meet R1 at ~12 000 taps for a 400 Hz transition at 768 kS/s,
i.e. 64 k-point frames, 85 ms latency and 0.5 MB per stream. The PFB below
meets R1 with a 321-tap prototype. The PFB is the design.)

## 3. Architecture

```text
IQ @ Fs ──► PFB analysis, M sub-bands, 2x oversampled ──► sub-band b @ R = 2Fs/M  (shared, once per stream)
                                                            │
                         per channel ◄──────────────────────┘
   select b nearest the window centre ─► NCO by the residual r ─► ↓4 (or L/M) to 12 kHz complex
   ─► sharp low-pass ±2.8 / ±3.2 kHz ─► × e^{+jπn/2} (audio 3 kHz back up) ─► Re ─► audio @ 12 kHz
```

The front stage is the textbook polyphase filter bank: prototype low-pass →
polyphase decomposition into M branches → commutator feeding the branches →
M-point FFT across the branch outputs. It is oversampled by 2 (hop M/2) so a
channel's window never has to straddle two sub-bands: every window lies
inside the flat part of the sub-band nearest its centre. The fine, 400 Hz
transition is done per channel at 12 kHz, where it is 195 taps; putting it in
the prototype would need a prototype of ~12 000 taps.

## 4. PFB parameters

Rule: M even; sub-band spacing S = Fs/M in 20…32 kHz; 2Fs divisible by M, so
the sub-band rate R = 2S is an integer; prefer R a multiple of 12 kHz, then
S nearest 24 kHz. A window is ±3.2 kHz around its centre and the centre can
be up to S/2 from the nearest sub-band centre, so

- passband half-width Bp = S/2 + 3.2 kHz + 0.3 kHz margin,
- stopband Bs = R − Bp (aliases of anything beyond Bs fold to beyond −Bp, i.e.
  outside every window the sub-band serves).

| Fs | M | S (Hz) | R (Hz) | R/12 k | Bp | Bs |
|---:|---:|---:|---:|---:|---:|---:|
| 96 000 | 4 | 24 000 | 48 000 | 4 | 15 500 | 32 500 |
| 192 000 | 8 | 24 000 | 48 000 | 4 | 15 500 | 32 500 |
| 250 000 | 10 | 25 000 | 50 000 | 4.167 | 16 000 | 34 000 |
| 384 000 | 16 | 24 000 | 48 000 | 4 | 15 500 | 32 500 |
| 768 000 | 32 | 24 000 | 48 000 | 4 | 15 500 | 32 500 |
| 912 000 | 38 | 24 000 | 48 000 | 4 | 15 500 | 32 500 |
| 1 024 000 | 40 | 25 600 | 51 200 | 4.267 | 16 300 | 34 900 |
| 2 048 000 | 80 | 25 600 | 51 200 | 4.267 | 16 300 | 34 900 |
| 2 400 000 | 100 | 24 000 | 48 000 | 4 | 15 500 | 32 500 |
| 2 500 000 | 100 | 25 000 | 50 000 | 4.167 | 16 000 | 34 000 |
| 3 200 000 | 128 | 25 000 | 50 000 | 4.167 | 16 000 | 34 000 |
| 6 000 000 | 250 | 24 000 | 48 000 | 4 | 15 500 | 32 500 |
| 10 000 000 | 400 | 25 000 | 50 000 | 4.167 | 16 000 | 34 000 |

A rate with no such M (48 kS/s, or one whose factors do not allow it) keeps
the `Direct` path. M need not be a power of two; the FFT backend takes mixed
radices.

**Prototype.** Kaiser window, designed at 123 dB, β = 12.60, length
L′ = M(K−1)+1 (odd, so the group delay is a whole number of hops), stored in
an M·K table with a trailing zero. For S = 24 kHz, K = 13:

| Fs | M | K | L′ | passband ripple | worst stopband | group delay |
|---:|---:|---:|---:|---:|---:|---:|
| 192 000 | 8 | 13 | 97 | < 0.0001 dB | −124.6 dB | 48 in = 12 hops |
| 768 000 | 32 | 13 | 385 | < 0.0001 dB | −124.7 dB | 192 in = 12 hops |
| 2 400 000 | 100 | 13 | 1 201 | < 0.0001 dB | −125.2 dB | 600 in = 12 hops |

K depends only on (Bs − Bp)/Fs·M, i.e. on S, so every row with S = 24 kHz has
K = 13; the 25 and 25.6 kHz rows are computed the same way at build time.

## 5. The PFB, exactly

Every hop of M/2 input samples (hop index s, newest input at t = s·M/2):

1. **Commutator / polyphase filter**: for p = 0…M−1,
   `u[p] = Σ_{k=0}^{K-1} h[p + kM] · x[t − p − kM]` (K complex·real MACs each).
2. **FFT**: `Y[b] = Σ_p u[p] · e^{+j2π b p / M}` — an M-point inverse FFT
   (unscaled), done once for all sub-bands.
3. **Oversampling phase correction**: the sub-band b output is
   `y_b[s] = (−1)^{b·s} · Y[b]`. This is the `e^{−j2π b t/M}` of the
   down-conversion evaluated at t = s·M/2; without it every odd sub-band flips
   sign each hop.
4. **Delay**: y_b[s] is centred on input t − (L′−1)/2 = (s − (K−1))·M/2. The
   first K−1 outputs are dropped, so sub-band output 0 is IQ sample 0 (R3).

Sub-band b is centred at b·S (b > M/2 are the negative frequencies,
b ≡ b − M). Gain: the prototype has unit DC gain, so a tone of amplitude A in
the passband comes out at A.

Cost per input sample: 2K real multiply-adds per complex sample for step 1
plus an M-point FFT per M/2 samples. At 768 kS/s about 118 MFLOP/s, the price
of roughly two and a half `Direct` channels, independent of the channel count.

## 6. The per-channel back end

For a channel with dial d: window centre c = d + 3000 − center (Hz from DC),
b = round(c / S) mod M, residual r = c − b·S ∈ [−S/2, S/2].

| stage | rate | design (Kaiser, 123 dB) | verified |
|---|---|---|---|
| NCO by −r | R | f64 phasor, renormalised | — |
| ↓4 (R = 48 k) | 48 k → 12 k | 71 taps, pass ±3.2 k, stop ±8.8 k | stop −122.7 dB |
| or L/M (R = 50 / 51.2 k) | R → 12 k | `PolyphaseResampler::from_prototype`, same pass/stop, L/M = 6/25 or 15/64 | to be verified the same way |
| sharp | 12 k complex | 243 taps, pass ±2.8 k, stop ±3.2 k | stop −122.5 dB |
| × e^{+jπn/2}, Re | 12 k | exact (1, j, −1, −j) | — |

Aliases of the ↓4 land outside ±3.2 kHz by construction (stop at 12 k − 3.2 k),
where the sharp filter removes them. About 15.4 MFLOP/s per channel, against
`Direct`'s 49.6 at 768 kS/s, because nothing per channel runs at the input rate.

**Whole chain** (`scripts/iq_pfb_chain.py`, 768 kS/s): over every window
position across a sub-band, including the edge, in-window gain varies by
under 0.0001 dB (R2). The worst interferer anywhere in ±Fs/2 outside the
window, after every fold, lands in the window at **−123.8 dB** (window 2 kHz
off its sub-band centre, interferer 9.4 kHz from the window centre, landing
at −2.58 kHz): R1. Interferers 3.2–6 kHz off the window centre, which only the
sharp filter stops, come out at −122.4 dB.

## 7. Cost

MFLOP/s from the design, both paths at the same 120 dB. `Direct` is modelled
from its own filter rules (49.6 MFLOP/s per channel at 768 kS/s; measured
0.93 % of a core, which puts 1 % of a core at about 53 MFLOP/s):

| channels | 768 kS/s PFB | 768 kS/s Direct | ratio | 2.4 MS/s PFB | 2.4 MS/s Direct | ratio |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 134 | 50 | 0.37× | 504 | 120 | 0.24× |
| 2 | 149 | 99 | 0.67× | 520 | 240 | 0.46× |
| 3 | 164 | 149 | 0.91× | 535 | 360 | 0.67× |
| 4 | 180 | 199 | 1.10× | 550 | 480 | 0.87× |
| 8 | 241 | 397 | 1.65× | 612 | 961 | 1.57× |
| 32 | 611 | 1 588 | 2.60× | 981 | 3 843 | 3.92× |
| 128 | 2 087 | 6 353 | 3.04× | 2 458 | 15 372 | 6.25× |

Break-even is 4 channels at 768 kS/s and 6 at 2.4 MS/s. The implementation
measures these (§9) and this table is replaced by the measurement.

### 7a. `Direct`, reworked first

The study found `Direct` at −73.5 dB, below R1, with most of its cost in a
331-tap sharp filter at 24 kHz. It was reworked before the PFB (maintainer's
call): every filter a Kaiser design at 120 dB (+3 margin); the resampler moved
ahead of the sharp filter so the resampler only has to stop at 8.8 kHz and
the sharp filter runs at 12 kHz complex (243 taps). Measured, one thread,
release, eight channels:

| Fs | selectivity before → after | cost per channel before → after |
|---:|---|---|
| 192 000 | −73.5 → −124.0 dB | 0.56 % → 0.36 % |
| 768 000 | −73.5 → −121.0 dB | 1.04 % → 0.93 % |
| 2 400 000 | −73.5 → −122.0 dB | 2.33 % → 2.42 % |

Selectivity is the worst of a sweep of ~550 interferer positions per rate plus
~600 aimed at every stage's alias edges. At 2.4 MS/s the first stage
(÷10 at the input rate, 85 taps) is 68 % of the cost, which is why the rework
does not make it cheaper there; that is the part a PFB shares across channels.

## 8. Integration

- `mfsk_core::iq::Channelizer { Direct, Pfb }`; `IqReceiver::with_channelizer`.
  `Direct` stays the default and is unchanged. `Pfb` fails construction with
  `UnsupportedRate` for a rate with no M in §4.
- Already in `engine::dsp` from §7a: the Kaiser designer (`kaiser_order`,
  `design_lowpass_kaiser`, with a series `I0` for `no_std`),
  `FirStage::from_taps` and `PolyphaseResampler::from_prototype`. New:
  `iq::pfb::AnalysisBank` (§5), and the back end of §6 as its own type so the
  two paths share nothing but the filters.
- `retune`: the bank does not depend on the centre; each channel's (b, r) is
  recomputed and its back end reset; the bank's history is cleared, since it
  spans the change. `gap`: the bank's history is cleared and the clock
  advanced. Open slots are dropped either way, as now (R5).
- C ABI: `mfsk_iq_open` keeps `Direct`; `mfsk_iq_open_with(..., channelizer,
  &status)` selects. `MfskIqDecode` is unchanged.
- The overlap-save prototype of §2 is deleted, with its measurement kept here.

## 9. Verification plan

1. **Selectivity sweep** (R1): a unit-amplitude interferer across ±Fs/2
   outside the window, on and between bins and sub-band edges and aimed at
   every alias edge, for window positions across a sub-band; worst ≤ −120 dB,
   as `tests/iq_front_end.rs` already asserts for `Direct`.
2. **Flatness** (R2): tones at audio 200…5800 Hz for dials that put the window
   centre at −S/2…+S/2 of its sub-band; spread ≤ 0.01 dB.
3. **Timing** (R3): a burst starting at a known IQ sample starts at the same
   audio index as through `Direct`, ±1.
4. **Decode equivalence** (R6): every test in `iq_receiver.rs` and
   `iq_receiver_modes.rs` run with `Channelizer::Pfb` gives the WAV path's
   sets with no extra decode.
5. **Rates** (R4): each row of §4, plus a rate with no M falling back or
   refusing as specified.
6. **Cost** (R7): 1, 2, 4, 8, 32, 128 channels at 768 kS/s and 2.4 MS/s, both
   backends, one thread, release; the table in §7 is replaced by these numbers.

## 10. Decisions

1. Selectivity target **120 dB** (§1), designed at 123 — decided.
2. `Direct` reworked first (§7a) — done.
3. `Direct` stays the default; the caller picks `Pfb` (no automatic switch,
   since the channel count is not known at open) — proposed.
