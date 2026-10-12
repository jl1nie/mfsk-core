# The sample rate of an `rsp_tcp` stream: what it is, and how to fix it (#685, #686)

Investigation of 2026-10-12/13 on an SDRplay RSP1 behind `rsp_tcp` (Windows, SDRplay API 3.x), read by the
skimmer's rtl_tcp client at 1.024 MS/s, FT8 on 7074 kHz. This note is the conclusion; the narrative of the
estimator that was tried and dropped on the way is `RTLTCP_TIMEBASE.md`.

## Conclusion

**The RSP1's sample clock is right; samples are lost on the way out of the SDRplay API.** The API's stream
callback stalls about every 1.85 s for ~93 ms (6 ticks of the 15.6 ms Windows scheduler), part of the samples
of the stall never reach `rsp_tcp`, and the stream that leaves the server is 3.3-3.7 % short, by an amount that
wanders +-2 % between 10 s windows because the stalls do. Nothing on the client side caused it: a plain python
reader sees the same shortfall.

**The fix belongs in the server**, where the holes can be seen exactly. In order of preference:

1. **Stop the losses.** The API's device parameter `mode` (`sdrplay_api_dev.h`: `sdrplay_api_ISOCH`, the
   default, or `sdrplay_api_BULK`) chooses the USB transfer: isochronous transfers drop what the host misses,
   bulk transfers are retried. `rsp_tcp` never sets it. With it: process priority / `timeBeginPeriod(1)` for
   the stalls themselves. Success is measured by the dbg3 clock log: the overall rate at the nominal one and no
   80 ms+ intervals without their samples.
2. **Fill what is still lost.** The callback's `params->firstSampleNum` (`sdrplay_api_callback.h`) numbers the
   samples; a jump larger than the previous `numSamples` is a hole of exactly that many samples. `rsp_tcp`
   writes that many zeros before the callback's samples. The stream then has the device's own time axis, holes
   are short silences at the right place, and no client needs a rate correction. (Whether the numbering counts
   the samples the API dropped is to be checked first: log `firstSampleNum` next to the clock log.)
3. **The client, with a fixed server:** the nominal rate, no rate offset; the clock from NTP-corrected arrival
   times minus a constant pipeline delay, calibrated from the median DT of decoded FT8 messages (target the
   median that SpyServer gives the same band, +0.09 s), not from a minimum-delay window. The pause detector
   (below) stays for servers that restart the device.

**For servers that cannot be changed** (stock `rtl_tcp`, stock `rsp_tcp`), the client keeps what measured best
(branch `iq-rate-offset-686` at `7bd0d0ec`): a 30 s pause-free least-squares measurement of the rate
at the start, the nearest plannable rate as the nominal one, the library's slow tracker (60 s window, 400 ppm a
fit, applied on the 15 s grid), the minimum-delay clock slewing up to 2 %, and a silence of more than 0.5 s told
to the receiver as lost samples. Its known limit: the 30 s measurement lands anywhere within +-1 % of the
recording's mean, and a measurement 1 % off loses most decodes until the tracker has walked it back (25 min at
400 ppm a minute). That limit is the reason the server fix is the answer, not more client-side estimation.

## Evidence

| what | how | result |
|---|---|---|
| server's own rate | `rsp_tcp` dbg3: UTC (`GetSystemTimePreciseAsFileTime`) and cumulative samples per ~50 ms of API callbacks, 600 s | 990 720 S/s (-3.25 %); 10 s windows 968k-1015k; dips at the 4 stream resets |
| where it is lost | same log, interval by interval | 8 725 normal intervals at a median 1 035 320 S/s (the catch-up after a stall); 324 intervals over 80 ms (one per 1.85 s, median 93 ms, max 1 432 ms at a reset) carrying 13.9 s less than their duration: 2.3 % of the time |
| sample clock | STFT of the recorded IQ, 64k-point, 1 s blocks | lines 161-419 kHz from the centre at exact multiples of 10 kHz (+45 Hz LO offset), steady within +-1 Hz for 10 min and across resets; FT8 band edges where the nominal scale puts them (+25.1 / +28.0 kHz for 7074.1 / 7077.0) |
| client shortfall | arrival times of a python reader that only reads | 3.95-3.98 MB/s for 4.096 (8-bit 2.048 MS/s): the same -3 % |
| 2.048 vs 1.024 MS/s | `rsp_tcp` console `queued` | 93-95 at 2.048 MS/s (from the first client implementation on, and with the python reader); 0 at 1.024 MS/s. `DEFAULT_RATE` is 1 024 000 |
| RSP restart | dbg3 log, arrivals | a silence of 0.9-1.5 s; the samples of it never exist; the sample counter goes on |
| burstiness | arrival times | 50-100 ms bursts; the longest silence outside a restart 100 ms |

## What was tried, on one recording (245 s, FT8 7074 kHz, 2026-10-12 21:35 JST), replayed to each client

Replays send the recorded bytes at the recorded offsets, aligned to the recording's phase modulo 15 s.
"Spread" is the median, over stations decoded at least three times, of the standard deviation of their frequency.

| client | decodes | spread | DT per slot |
|---|---|---|---|
| `4d3f1ac4`: 30 s measurement -> nominal 987 456, slow tracker | 52 | 5.3 Hz | |
| same mechanics with the fast tracker (30 s window at 5 000 ppm, 30 000 ppm a fit, clock stepped at 3 000 ppm) | 42 | 39 Hz | |
| restored slow tracker (`back2`) | 51 | 5.0 Hz | -0.2..+0.4, one +1.9 |
| + offset applied on the 15 s grid, clock slew 2 % | 49 | 5.1 Hz | -0.1..+0.35 |
| `TimeBase` (rate and anchor from one lower-envelope line, continuously) | 36 | 250 Hz | |
| `TimeBase` with a held slope | 27 | 169 Hz | |
| no rate offset (nominal 1 024 000), clock slew 2 % | 4 | | clock re-anchored every minute |
| no rate offset, clock slew 6 % (drift 0, no re-anchor) | 4 | | |
| first implementation (`267424d5`, no rate handling) | 3 | | |

What the table settles:

* **The audio's time scale must match the stream's** (with the stream short by 3.5 %, FT8 symbols are 3.5 %
  short: 450 ms over a transmission). With the clock right and no offset: 4 decodes. DT alone is not the problem.
* **The offset must not move inside a slot or wander.** A change inside a slot is a drifting tone (150 ppm is 4 Hz
  26 kHz from the IQ centre); an estimate that follows the +-2 % wander moves the frequency by 170-250 Hz from
  slot to slot.
* **Averaging longer does not find "the" rate**, because there is none: the shortfall is made of stalls, and each
  slot has its own. A uniform stretch is an approximation that only works because the stalls are short against
  a symbol (93 ms against 160 ms) and frequent enough to average within a slot.

Live, on the GUI (2026-10-12 23:20-23:45 JST): the 30 s measurement gave 1 007 325 (-1.6 %, against the
recording's -3.5 %), DT ran from 0 to -0.9 s in two minutes, few decodes with signals on the waterfall. Same
mechanism: the measurement is a draw from +-1 %.

## Measurements still to make

* Does `firstSampleNum` jump at the stalls, by the samples lost? (dbg4: log it.)
* Does `sdrplay_api_BULK` remove the stalls or the losses? Does a higher priority?
* SpyServer and RSP1 on the same antenna and band at once: the same stations' frequencies tell whether the
  decoded frequencies of the stretched audio are off by 3.6 % of their distance from the centre (they should be,
  if the clock is right and samples are missing).

## The server patch (dbg4)

`scripts/rtltcp/rsp_tcp_dbg3_to_dbg4.patch` (on top of dbg3; built 2026-10-13 with `cl /O2`, runs not yet made)
adds three switches and one log record to `rsp_tcp`:

* `-U` sets `devParams->mode = sdrplay_api_BULK` before `Init` (step 1);
* `-H` raises the process priority class and calls `timeBeginPeriod(1)` (step 1);
* `-Z` writes the hole between `firstSampleNum` and the previous callback's end as silence (uint8 128 / int16 0)
  ahead of the callback's samples, for holes up to 2.048 MS (step 2);
* an `F` record in `rsp_clock.log` for every callback that follows a hole: `F <UTC> <firstSampleNum> <hole> <filled>`
  (`clock_log.py` summarises it). No `F` records over a run that is 3 % short means the API's numbering does not
  count the lost samples, and `-Z` cannot help.

Runs to make, in this order, 10 min each (`start_dbg4_<name>.bat` in the pack; each start deletes `rsp_clock.log`,
so rename it after a run; `python3 -I scripts/rtltcp/clock_log.py rsp_clock.log 1024000` on each):

1. `plain`: are there holes in `firstSampleNum`? No `F` records means `fill` cannot help, and only 2 and 3 decide.
2. `bulk` (`-U`): does bulk transfer alone remove the losses?
3. `bulkhigh` (`-U -H`): does priority and the 1 ms timer add anything?
4. `fill` (`-Z`): only if 1 showed holes. `all` (`-U -H -Z`) is the confirmation of the final set. Success: overall rate at the nominal one and no 80 ms+ record gaps. Then the client with
`-Z` or `-U`, no rate offset, against the recording method above.

## Tools

`scripts/rtltcp/`: `record.py` (an rtl_tcp stream with the arrival time of every read), `replay.py` (a server
that replays it on the recording's 15 s phase; `run.sh` connects a client to it just before the first sample),
`clock_log.py` (rate per window and per interval from a dbg3 `rsp_clock.log`), `lines.py` (STFT line tracking in
1 s blocks), `compare.py` (decodes, spread and DT per slot from client logs). The recordings (0.5-1.2 GB) are not
in the repository.

`rsp_tcp` dbg3 is `rsp_tcp` with the recovery of dbg2 (Uninit/Init after `sdrplay_api_DeviceFailure` instead of
exit) and, in the stream callback, a record every 50 ms of `GetSystemTimePreciseAsFileTime`, cumulative samples
(64-bit), callbacks, `reset` and `fsChanged`, written to `rsp_clock.log` by the TCP worker thread. Built from
`rsp_tcp.c` with the SDRplay API 3 headers and pthreads-win32 (`cl /O2`).
