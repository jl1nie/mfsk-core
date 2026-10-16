# Time base of an `rtl_tcp` stream (#685, #686)

> **Status (2026-10-12): this estimator is NOT used by the skimmer.** The conclusion of the whole investigation,
> with the cause (samples lost at the SDRplay API's callback stalls) and the fix (in the server), is
> `RTLTCP_RATE.md`. It is kept as the record of an approach that was tried
> and measured worse. The skimmer's rtl_tcp path is the one of commit `4d3f1ac4`: a 30 s pause-free least-squares measurement
> of the real rate at the start, the nearest rate the receiver takes as the nominal one, the library's slow rate tracker, and
> the minimum-delay anchor for the clock. On the same RSP1 recording it decoded 42-52 FT8 messages against 27-36 for this
> design (see "Open" below). The code is in the history: commits `a0d7d18c` .. `e4d12ed4`, `apps/skimmer/core/src/timebase.rs`.

What the skimmer knows about an `rtl_tcp` / `rsp_tcp` stream, and the one estimator that turns it into the
receiver's time axis. It replaces three pieces that were added one symptom at a time and interfered
(a blocking 30 s rate measurement, a 30 s minimum-delay anchor with a pause check, and the library's
`track_rate` fed with the anchor's output): see the history in #686.

## What is observable

Per message, `(A_i, S_i)`: the arrival time on the PC's clock (NTP-corrected, accurate to milliseconds)
and the samples received so far. Nothing else: the protocol has no timestamps, no rate, no sequence.

## The model

Within a stretch of the stream the device samples at one rate `R`, which is not the one asked for
(`rsp_tcp` asked for 2.048 MS/s gave ~1.99 MS/s, -2.8 %) and is not known:

    A_i = T0 + S_i / R + L_i,        L_i >= 0

`L_i` is delivery delay: the server's buffering, the network. It is heavy-tailed and only ever late. So
the *lower envelope* of `(S_i, A_i)` is the signal, and a line fitted to it gives both `R` (its slope) and
`T0` (its intercept), together. A minimum filter followed by a regression on its output (what this
replaced) fits twice to correlated errors: the filter's output moves in steps, so the second fit's own
standard error is optimistic and a short line swings by thousands of ppm.

Two things break the line:

* **a pause**: the server restarts its RSP (`RSP failure`); the samples of the pause never exist, the
  sample counter goes on, and the arrival times jump by `P`. Every point after lies `P` above the line.
* **a stall that is made up**: the network holds data and delivers the backlog at once. Points rise above
  the line and come back within seconds. This is *not* a pause: the samples all exist.

## The estimator (`skimmer_core::timebase::TimeBase`)

Pure: `push(samples, arrival_ns)` in, events out; no clock, no I/O, so every behaviour below is a test in
virtual time.

1. `x = samples / nominal` (s), `y = arrival - arrival0` (s). One point per 0.5 s is kept, the one lowest
   above the current line; the window is 240 s.
2. **The line**: the lower convex hull of the window, and of its edges the one with the least sum of
   residuals (all residuals of a hull edge are >= 0): the optimum of the linear program "line below every
   point, as high as possible". Needs 20 s of window; a line through hull vertices closer than 25 % of
   the span is not trusted for its slope (the previous slope is kept).
3. **Rate and anchor**: `b` = slope of `y` on `x`; the real rate is `nominal / b`, so the offset the
   receiver is told is `ppm = (1/b - 1) * 1e6`. The anchor is the line's intercept. Both change a little
   at a time (the line moves as the window moves), which the receiver's clock slews.
4. **A pause** is found when the last 6 s of points all lie more than 0.4 s above the line. Its length
   `P` is the least of those residuals; the samples lost are `P * R`. The caller tells the receiver
   (`IqReceiver::gap`), which moves its time on by that much, and the estimator shifts `x` of the points
   of the last 6 s by it: they are after the pause, so they lie on the same line again. The line itself
   does not change.
5. **A break the other way**: if the last 6 s of points all lie more than 0.2 s *below* the line (the
   rate is faster than the line says: a new regime after a restart), the points older than 30 s are
   dropped and the line is fitted again.

## What the receiver is told

* The nominal rate is the one asked of the server (a plan it takes already); no measurement, no
  `plannable_rate`. The offset is `ppm` of step 3, within +-5 % (`IqReceiver::MAX_RATE_OFFSET_PPM`), which
  moves the edges of the channel window by 160 Hz at most (FT8 is inside it; tested at +-2.5 %).
* `set_rate_offset_ppm(ppm)` when it moves by 50 ppm or more.
* `set_time(utc, samples)` with the line's UTC of the latest sample, on every message.
* `step_time(utc, samples)` (the clock set at once) for the first line, after a pause and after a
  break: the readings the clock had followed up to then were at another rate or before a hole.
* At the start, the offset the last session learned (kept by the process) until a line exists.

## What is not done

* The delay itself (`L_min`) is not estimated: it is the user's *Network delay*. DT is relative to the
  fastest packet seen.
* `SpyServer` keeps `anchor::AnchorEstimate`: its rate is exact and it numbers its messages, so a pause is
  a hole in the sequence.

## Open: what the first real-hardware replays showed (2026-10-12, RSP1 through `rsp_tcp`, 1.024 MS/s, FT8 7074 kHz)

**This design is not finished.** The estimator tracks the clock (DT) well, but the rate offset it hands the receiver
wanders, and that costs decodes. Recordings and the replay harness were made for this and are described below; the
recordings themselves (0.5-1.2 GB) are not in the repository.

* **Same recording, two clients** (replayed with the arrival times of the recording, the start aligned to its 15 s phase):
  the commit before the time base (`4d3f1ac4`: a 30 s rate measurement, then the nearest plannable rate, here 987 456 S/s,
  and the library tracker, which moves the offset only on a >=120 s window at >=3 sigma and >=100 ppm) decoded **52**
  FT8 messages in 245 s with the same station's frequency within a median 5 Hz from slot to slot; this design decoded
  **36** with a median 250 Hz (25 kHz from the IQ centre, a 1 % error in the offset is 250 Hz, and 126 ms of symbol timing over
  one FT8 transmission). The first implementation, with no rate handling, decoded 3: its clock stepped about every 90 s.
* **Why short windows cannot do it.** The server's own log (`rsp_clock.log`, rsp_tcp dbg3: samples against the server's
  UTC every ~50 ms) has the API delivering 990 720 S/s on average for a nominal 1 024 000 (-3.25 %), **varying +-2 % between
  10 s windows**, with dips at the five stream resets (and the same on the arrival side: 20 s least-squares windows gave
  -24 000..-42 000 ppm). An offset estimated from a window shorter than a couple of minutes carries that +-1 %.
* **Not settled:** whether the -3 % is a slow sample clock or samples lost before the TCP stage. Lines in the IQ (multiples of
  1 kHz + ~45 Hz, 161-419 kHz from the centre) stay within +-1 Hz of sample-space frequency for 10 minutes, across the resets,
  which looks like a constant clock; but they may be spurs locked to the ADC clock, which would say nothing about its rate.
  Decoded frequencies and the FT8 band edge on the spectrum (about +25.1 kHz for 7074.0 kHz) point the same way, not
  proof. Phase jumps of those lines are modulation, not holes (an attempt to read hole lengths from them gave 29 %
  of the time and is withdrawn).
* **First attempt at (1) below, same recording (2026-10-12 late):** the slope handed on moves only on strong evidence
  (>=120 s and >=300 ppm, or >=5000 ppm held 30 s) and the intercept follows the lowest points of the last 30 s against it:
  **27** decodes, station-frequency spread median 169 Hz (this design before: 36 / 250 Hz; `4d3f1ac4`: 52 / 5 Hz). Not enough. The
  first acceptance comes at ~40 s from a 30 s window, which carries +-1 % (the server's own rate is +-2 % between 10 s
  windows), and that value then stays for minutes. `4d3f1ac4` measured once over a pause-free 30 s and kept it.
* **Outcome:** the old mechanics, restored (1.024 MS/s default kept), gave 42 decodes and a 39 Hz spread on the same recording: the measured-once rate is what works; a continuously re-estimated one does not.
* **Was next, in this order:** (1) the offset moves only on strong evidence, as the old tracker did (window >=120 s, >=3 sigma and
  >=300 ppm, or a large step held 30 s), and starts from the last session's value; (2) replay the same recording again
  and compare decodes and the station-frequency spread; (3) only then the experiments with a fixed offset (0, -29000).
* **Defaults that follow from the measurements:** `DEFAULT_RATE` is 1 024 000 (at 2 048 000 the server's send queue sat at its
  ceiling, 93-95, and stayed 0 at 1 024 000, with the first client implementation too). DF and DT are separate: the offset
  decides frequency and symbol timing, the clock decides the slot position.
* **Tools** (kept outside the repository): a recorder and a replay server for rtl_tcp (python; the replay aligns its start to the
  recording's phase modulo 15 s and connects the client just before the first sample), `rsp_tcp` dbg3 with the server-side clock log.
