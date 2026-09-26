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

**This section is about provenance, not availability.**
`SniperRequest::ap_hint()` works today and is passed through to the
decoder; so does `DecodeRequest::ap_hint()`, for FT8, FT4 and every FST4
sub-mode. What was removed was the *coupling* below, not AP's presence
on the sniper builder.

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

**The engine is now gone.** AP is a rung at the end of the
per-candidate ladder — everything above it has already run and failed,
so it can only add decodes. For FT4 and every FST4 sub-mode that ladder
is `engine::pipeline::process_candidate_basic`. FT8 does not implement
`GenericPipelineProtocol` and never reaches it: its AP rung sits at the
end of its own ladder, `ft8::decode_block::process_one_candidate_inner`,
which builds the same hypotheses inline rather than calling
`ap_passes` (#415; unifying the two ladders is #423).
`msg::pipeline_ap` is what remains: `ap_passes` (the hypothesis set,
WSJT-X's `iaptype` equivalents) and `ap_bits_for`, 103 lines with no
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

**Where the WSJT-X 3.2 port moved the AP rung (#456, 2026-09-26).** Two
things were taken from upstream, measured in `FT8_BENCHMARK.md` §15. The
accept bound is `ft8b.f90`'s: `DecodeStrictness::Normal`'s `ap_max_errors`
is 36, not 30 (25 from 55 locked bits) — right-hint FT8 8771 → 9315 hits,
at the price of a few more phantoms with a wrong hint, which carry an AP
pass id so a UI can mark them. And the heavy hypotheses (`iaptype >= 3`,
both callsigns locked) are tried only within `napwid` of the QSO frequency
(`freq_hint`) or, on FT8, of the transmit frequency (`tx_freq`, WSJT-X's
`nftx`): FT4 had been running them on every candidate, where
`ft4_decode.f90` and `ft8b.f90` do not, and without a `freq_hint` a heavy
hypothesis is now not tried at all, as upstream always has an `nfqso`.

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

That describes `.ap_list()` **without an Rx frequency**, which is the
old per-candidate match and is kept for callers that have none. With
`.rx_freq(hz)` (and `.ftol(hz)`, default 10 Hz) `.ap_list()` is WSJT-X's
**q3** decode instead, and it does run the fast-fading intrinsics: first
`q65_ccf_85` synchronises on all 85 symbols of every list message within
F Tol of the Rx frequency, and where one message leads the runner-up by
1.10 it list-decodes at that alignment over the `b90` sweep
(`q65_dec_q3` → `q65_dec1`); then the normal scan runs for the rest of
the band, as `q65_decode.f90` orders it (`q65/decode_request.rs`,
`q65/q3.rs`). A list decode is accepted only when `plog > PLOG_MIN`
(−242) and the message is not all zeros; the size-adjusted threshold
alone (about −256) let a wrong codeword through on the Q65-60B
troposcatter golden once the metric was corrected, so every list decode
in the crate (`.ap_list()` scans, sniper, multi-period) now applies both.
q3 at -24 / -26 / -28 / -30 dB on 20 `q65sim` files each decoded
20 / 20 / 7 / 2, the same files `jt9 -3` decodes.

The rest of WSJT-X 3.2's Q65 settings are options on the same request
(`CHANGELOG.md`, 0.12.0 has the measurements):

* **Max Drift** (`.max_drift(bins)`, 0..=50) searches a linear tone drift
  across the frame (`q65_ccf_22`'s `idrift` loop) and takes it out before
  the symbol spectra. The undrifted sweep runs first, as upstream; off by
  default. At 50, when nothing decoded near the Rx frequency, the q3 decode
  runs again on spectra shifted by the drift found (the "w3sz" stage 5).
* **EME delay and the ±1 s default.** `q65.f90` searches −1.0 to +1.0 s and
  reaches +5.5 s (+4.0 s for Q65-15) only when `emedelay > 0`, which the GUI
  sets for "Decode at 52 s". `default_search_params()` had been late by
  +5.5 s for every sub-mode, from measuring against the `jt9` CLI, which
  switches the delay on for TR 60 s itself. The default is now ±1 s and
  `.eme_delay(true)` restores the reach. Recorded, not closed: `jt9` with
  the delay off still decodes a Q65-120D frame 2.0 s late where this crate
  stops at +1.5 s.
* **Pileup** (`.pileup(true)`, with an `ap_hint` naming both callsigns)
  leaves the spare 78th bit free so a reply with the "copied last Tx"
  flag still matches; the row carries `copied_last_tx` (WSJT-X's `#`).
* **The contest list.** WSJT-X remembers up to 50 stations that called with
  a grid (`q65_hist2`) and builds its full-AP list from them
  (`q65_set_list2`). `q65::Q65Callers` is the list, held by the
  application because the library reads no clock, and
  `q65::contest_codewords` is the codeword list; it goes through
  `.ap_list()` like the standard one. `q65_hist` (the DX call and grid
  from earlier decodes) is likewise application-owned.
* **The decoder metric uses the punctured code rate 13/63**, not 15/65
  (`q65_init`'s `decoderEsNoMetric = nm * R * EbNoMetric`, with `R` the rate
  after puncturing). 15/65 made every Q65 intrinsic 12 % high; the
  winning codeword's log-likelihood was −242.78 against WSJT-X's −241.92 on
  identical spectra, 0.78 under `PLOG_MIN`, and is now equal to the
  hundredth.

**FST4's noise blanker** (`DecodeRequest::noise_blanker`, sub-modes
implementing `SupportsNoiseBlanker`) is not a Q65 matter but the same kind
of setting: WSJT-X runs the whole FST4 decode on samples passed through
`blanker.f90` at the GUI's **NB** level. `Percent(n)` blanks the loudest
`n` % of samples; `Sweep` decodes at 0, step, … 20 %. Off by default, as NB
0 % is. On 50 slots with 20 clicks a second: 0 decodes without it, 29 at
NB 2 % (`jt9 -7`: 27).

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
