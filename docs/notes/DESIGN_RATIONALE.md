# Design rationale

Why some of this crate's decisions went the way they did, with the
measurements that decided them.

This file exists so the reference manuals can state *what* is true
without also arguing *why*. `LIBRARY.md`, `EMBEDDED.md` and
`BINDINGS.md` carry conclusions; this carries the evidence behind the
ones that are repeatedly re-litigated, re-derived, or misread.

It is not a changelog. `CHANGELOG.md` and `docs/historical/` record
what changed and when; this records reasoning that is still load-bearing
for a decision someone might otherwise make again.

## Contents

- [1. Sniper mode is a roofing-filter mode](#1-sniper-mode-is-a-roofing-filter-mode)
- [2. Equalisation is a property of the input audio](#2-equalisation-is-a-property-of-the-input-audio)
- [3. A-priori decoding was coupled to the sniper by accident](#3-a-priori-decoding-was-coupled-to-the-sniper-by-accident)
- [4. Q65's decoder strategies, and what each is for](#4-q65s-decoder-strategies-and-what-each-is-for)
- [5. Why the embedded path doesn't widen PASS1 or enable OSD](#5-why-the-embedded-path-doesnt-widen-pass1-or-enable-osd)

---

## 1. Sniper mode is a roofing-filter mode

**The ±250 Hz window is hardware, not cleverness.** Sniper mode is the
software half of narrowing the *analogue* roofing filter on a
transceiver that has one at that width — the Yaesu FTDX101MP and FTDX10
are the usual examples. The operator selects a ~500 Hz analogue filter,
points it at a DX station whose carrier is already known, and the audio
reaching the decoder is already band-limited to that slice. Searching
±250 Hz is the decoder matching what the hardware left in the passband.
It is a special-purpose path for specific radios, **not** the natural
shape for "any mode where you know who you are working".

**It is FT8-only, and that is the design.** `SupportsSniper` is
implemented for `Ft8` and nothing else. The wide-band path is the main
path for every mode in this crate — if it is not WSJT-X-faithful
without a sniper, that is a bug in the wide-band path, not a reason to
reach for this one.

- **FT4** is a contest protocol whose entire premise is working a full
  band, so a roofing filter is against its purpose.
- **FST4** narrows through its own DDC channelizer
  (`docs/notes/FST4_DDC_DESIGN.md`), which is the same benefit without
  a second decode engine.

FT4 and FST4 sniper entry points existed until 2026-09-13 and were
removed. The C ABI has never exposed a sniper call of its own — the
capability is published as `MFSK_CAP_SNIPER` and driven through
`MfskDecodeParams::search_hz`.

## 2. Equalisation is a property of the input audio

`.eq_mode(EqMode::Local)` sits beside the sniper for the same reason an
analogue filter's skirt tilts the passband: local equalisation flattens
it again.

Because it describes the *audio* rather than the *search*,
`DecodeRequest` carries it too — filtered audio can be fed to a
wide-band decode, and FT8's `eq_mode_recovers_bpf_edge_signal` pins a
band-pass-edge signal that decodes with `Local` and not with `Off`
through the wide-band SIC engine.

**On flat synthetic input, EQ can only cost.** The `ft4sim`-generated
FT4 golden has no receiver filter, and `Local` loses two decodes of
fourteen there. Default `Off` is right for recordings and simulations,
and wrong for a narrowed receiver.

## 3. A-priori decoding was coupled to the sniper by accident

AP locks high-confidence bits and lowers the threshold by 1–3 dB.
Nothing about it is narrow-band.

But AP used to live in a parallel engine
(`msg::pipeline_ap::decode_sniper_ap`) that the sniper was the only
caller of, and that engine broke out of its candidate loop on
`if has_ap` — **the presence of a hint was what made the search
single-target**, not the width of the search. FT8 escaped only because
it has its own AP path and never entered that engine, which was the
whole content of `SupportsWideBandAp` being FT8-only.

**Removing the coupling was not enough, and the measurement is worth
recording because "it is one line" was asserted twice and was wrong
both times.** Routing a wide-band FT4 decode through that engine
returned **4 decodes where the plain path returns 11**, lost the hinted
station itself, and returned the *identical* set whether the hint named
a present or an absent station — so AP was changing nothing and the loss
was entirely the ladder: `process_candidate_ap` offered OSD at depth 2
with no depth-3/4 escalation and no Top-K rescue, and most of that
recording's decodes come from exactly those.

**The engine is now gone.** AP is a rung at the end of
`engine::pipeline::process_candidate_basic`'s own ladder — everything
above it has already run and failed, so it can only add decodes — and
it therefore reaches FT8, FT4 and every FST4 sub-mode.
`msg::pipeline_ap` is what remains: `ap_passes` (the hypothesis set,
WSJT-X's `iaptype` equivalents) and `ap_bits_for`, 96 lines with no
engine of its own.

Two bugs were found while wiring it, both shipped:

* **FT4/FST4 AP was locking half its bits to the opposite of the
  truth.** Those protocols XOR the 77-bit message with their own RVEC
  before CRC and FEC (`ModulationParams::INFO_SCRAMBLE_RVEC`), so the
  info bits inside the codeword are the *scrambled* message, while an
  `ApHint` describes the message. `ap_bits_for` now scrambles. At
  −17 / −18 / −19 dB on 12 trials each: **3→12, 0→11, 0→8**. FT8 has no
  RVEC and was never affected — which is why this survived: the protocol
  where AP is most used is the one where it worked.
  (`tests/ft4_ap_scramble.rs`.)
* **A blind-CQ hypothesis was missing** (upstream `iaptype 1`, locking
  the message type and the `CQ` prefix with no callsign knowledge). It
  is the pass that applies with no station knowledge at all, so it
  changes the *baseline* rather than a hinted case. FT4's AWGN
  threshold moved **−16.89 → −18.00 dB**, from 0.6 dB behind WSJT-X's
  published −17.5 dB to 0.5 dB ahead. Caveat recorded in
  `FT4_BENCHMARK.md` §48: the sweep corpus is CQ-only, so this is the
  best case for that pass.

AP's risk is manufactured decodes, so wide-band AP ships with precision
guards in the same file as the gain assertion: a hint naming a station
that is not transmitting must not produce it, and a confident hint over
pure noise must decode nothing.

## 4. Q65's decoder strategies, and what each is for

Q65 is the one wired protocol where a single FEC frame can be
approached through several legitimately different receiver chains, each
trading runtime cost against a different channel pathology. The
selection table lives in `LIBRARY.md`; this is what each front end
actually does.

**A point worth flagging first.** `SniperRequest::decode()` with no
capability set really is the plain Bessel-I0 metric — the
point-decode-only baseline. But `DecodeRequest::decode()` with no
capability set — the shape nearly every real caller uses — routes each
coarse-search candidate through a WSJT-X-faithful `(Δf, Δt, b90)` grid
search using the fast-fading metric with `FadingModel::Lorentzian`,
ported from `q65_loops.f90` / `q65_dec_q012`. That mirrors WSJT-X's own
automatic decoder, which never runs a plain-AWGN-only Bessel pass for
its default scan either.

So "AWGN" and "fast-fading" are best read as two distinct *entry-point
families* (sniper vs. scan), **not** two cleanly separate front ends
picked by channel type. The scan path already assumes some fading by
default, and `.fading()` exists for when the caller wants a specific
`(b90_ts, model)` explicitly instead.

**AWGN Bessel + BP** — the textbook single-shot path: per-symbol FFT
energies become probability vectors via the Bessel-I0 metric, then
non-binary belief propagation runs on the QRA code. Falls back
gracefully on any channel reasonably close to additive Gaussian noise.

**AP-hint BP** (`.ap_hint(&ap)`) clamps the intrinsic probability
vectors at known information-bit positions before BP. A correct hint
shifts the BP fixed-point closer to the truth; a wrong hint typically
fails to converge rather than misdecoding, and the CRC catches what is
left. ~2 dB.

**Fast-fading metric** (`.fading(model, b90_ts)`) replaces the Bessel
front end with a spread-aware alternative, calibrated against a
caller-chosen `Gaussian` or `Lorentzian` shape. Required for microwave
EME, where lunar libration spreads each tone over 10–60 Hz: the 10 GHz
EME reference recording in `samples/Q65/60D_EME_10GHz/` decodes via
this path and produces **zero** hits with the plain Bessel front end.
`b90_ts` is spread bandwidth × symbol period — typical values 0.05
(near-AWGN), 1.0 (moderate), 5.0+ (severe). 5–8 dB on spread channels.

**AP-list template matching** (`.ap_list(&candidates)`) does *not* run
BP. The generator `q65::ap_list::standard_qso_codewords(my_call,
his_call, his_grid)` pre-encodes the WSJT-X "full AP list" — 206
standard exchanges a known callsign pair can legally produce
(`MYCALL HISCALL`, `… RRR/RR73/73`, `CQ HISCALL grid`, plus the
200-entry SNR ladder). The decoder picks the candidate whose
log-likelihood under the soft observations exceeds a size-adjusted
threshold, or returns `None`. Useful when the application has a known
callsign pair but no QSO state: at −25 dB (1 dB below the published
Q65-30A threshold) the test sweep shows AP-list decoding **6/6** frames
where plain BP fails **0/6**. ~3 dB.

`.ap_list()` and `.fading()` are mutually exclusive in the underlying
engine; `.decode()` resolves precedence as
`ap_list > fading (+ ap_hint) > ap_hint > plain`.

**Multi-period EMA averaging** (`MultiPeriodRequest`) mirrors WSJT-X's
`iavg=1`/`iavg=2` averaged decode from `q65_decode.f90` — the strategy
that lets ionoscatter and weak EME signals decode when no
single-period strategy can. Takes `&[&[f32]]` (one buffer per T/R
slot). It maintains an exponential moving average of the per-slot
spectrogram (time constant `min(navg, 4)`) across consecutive T/R
periods and, at each slot, tries a 3-stage ladder against the averaged
energies: AP-list when `.ap_list()` was set, then fast-fading BP
sweeping `b90·Ts ∈ {3, 8, 15}` × `{Gaussian, Lorentzian}`, then plain
Bessel BP as an AWGN fallback. At most one decode per slot, deduped by
`(message, ±4 Hz)`.

## 5. Why the embedded path doesn't widen PASS1 or enable OSD

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
