# Time base of an `rtl_tcp` stream (#685, #686)

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
