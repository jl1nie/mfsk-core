# mfsk-serve — manual

A resident process that decodes and encodes through `mfsk-core`. One JSON object per line in on stdin, one or more per request out on stdout.

> **A sample app.** It is `publish = false`, has no tag of its own, and was written to show how to hold decoders across slots from a program in
> another language. It reads and writes files and stdin/stdout only: no network, no authentication.

## 1. Why a resident process

A program that decodes one slot at a time can start a process per slot or keep one. A `Decoder` holds state between calls: the callsign hash
table, the two latest periods (FT8 a7), and a period's checkpoint state. A process per slot starts every slot with none of it, so a hashed
call heard in full a slot ago prints as `<...>`. This process keeps one decoder per mode for as long as it runs.

It is not, by itself, a speed-up. Measured on x86 (`qso3_busy.wav`, one thread): a warm process saves 10-21 ms of an 82-620 ms call
(`Depth::Fast` 82 ms, `Normal` 280 ms, `Deep` 620 ms). What the board costs is [#761](https://github.com/jl1nie/mfsk-core/issues/761), measured with
`bench` (section 5).

## 2. Install

```sh
cargo install --git https://github.com/jl1nie/mfsk-core --rev <commit> mfsk-serve
```

`--rev` pins the build; without it the tip of `main` is built, which can hold changes that are not in a release yet. On a Raspberry Pi build
on the board with a 64-bit OS: on 32-bit ARM, rustc 1.100.0 offers NEON only as an unstable target feature (it warns that this will become an error). `RUSTFLAGS="-C target-cpu=native"` is allowed there; what
it buys has not been measured.

## 3. Commands

```text
mfsk-serve                                    serve requests from stdin until it closes
mfsk-serve decode FILE [--mode M] [--depth D] [--band LO-HI]    one request, answers on stdout
mfsk-serve bench FILE [--mode M] [--iters N]                    time the decoder on FILE
```

`FILE` is a `.wav` (mono, 12 kHz, s16 or f32) or a `.raw` (s16le, mono, 12 kHz). `M` is a mode name as the library spells it (`FT8`, `FT4`,
`FST4-60`, `WSPR`, …), `FT8` by default.

## 4. The protocol

Every line is one JSON object. A request has `op` and may have `id`; every line answering it carries the same `id` (any JSON value, or none).
Requests are served in order, one at a time. A line that cannot be served gets one `error` line and the next request is served as usual.

### `decode`

| field | | |
|---|---|---|
| `mode` | required | mode name |
| `wav` | required | path; `.raw` is read as s16le, anything else as WAV |
| `depth` | | `fast`, `normal`, `deep` (the library's default, `deep`, when absent) |
| `band` | | `[lo, hi]` Hz (the mode's default band when absent) |
| `period` | | the period's index on the UTC grid; `prefix` needs it, and state that spans periods is used only when it is given |
| `budget_ms` | | stop taking candidates once this many milliseconds have passed |
| `learn` | | calls handed to `learn_callsign` before decoding |
| `prefix` | | `true`: the call is `Decoder::decode_prefix`, `wav` is the period so far; needs `period` |
| `samples` | | use only the first this many samples of the file, so a whole recording can be replayed at the checkpoints |

Depth, band and the other settings are those of this request; a request that leaves one out gets the default, not the previous request's.

Answer: one `row` per decoded message, then one `done`.

```json
{"id":7,"type":"row","mode":"FT8","text":"CQ EA2BFM IN83","freq_hz":2279.75,"dt_sec":0.105,"snr_db":-16.5}
{"id":7,"type":"done","rows":21,"candidates":161,"elapsed_ms":312.9,
 "budget":{"exhausted":false,"candidates_skipped":0,"stages_run":643,"cut_at_sync":null}}
```

`candidates` is how many candidates the call's first search kept (`SlotResult::candidates`; it is what was searched, not a recall figure). `budget`
is the library's `BudgetReport`. `elapsed_ms` is wall clock, for a person to read.

**Checkpoints.** With `prefix`, a call before a checkpoint returns no rows. FT8's are at 141 696, 162 432 and 180 000 samples; the decoder lists its
own (`AnyDecoder::prefix_points`). The call at the full length returns the period's complete set, the rows a whole-period call returns, in the
same order. Rows found by an earlier call of the period are in it again.

### `encode`

| field | | |
|---|---|---|
| `mode`, `text` | required | the message |
| `f0_hz` | required | the carrier: the frequency of tone 0 |
| `rate` | required | sample rate, 1 to 192 000 Hz |
| `out` | required | path; raw s16le samples are written there |
| `amplitude` | | peak of the s16 waveform (14 000 when absent) |

```json
{"id":"tx","type":"encoded","mode":"FT8","text":"CQ JA1ABC PM95","hashed_calls":["JA1ABC"],"samples":606720,"out":"/tmp/tx.raw"}
```

`text` is the message as the receiver will print it. The waveform is one frame from sample 0; placing it in the period (0.5 s after the boundary for
FT8) is the caller's. `hashed_calls` are what the message sends as hashes; the process hands them to its decoder (`learn_callsign`, WSJT-X's
`record_tx_hashes`), so a reply that refers to them resolves.

### `error`

```json
{"id":4,"type":"error","error":"unknown depth \"warp\" (fast, normal, deep)"}
```

A request line that is not JSON has no `id`. A request that panics inside the library is answered with an `error`, and that mode's decoder is dropped
(built again, empty, on its next request), because its state is unknown after a panic.

## 5. Measuring a board

```sh
mfsk-serve bench qso3_busy.wav --iters 6
```

For each depth: `whole #i` is a whole-period call, `#0` cold (the first call of a process, which is what a process per slot pays every slot), then
`prefix period i` is the same period fed in at the decoder's checkpoints, one time per checkpoint. The figures are wall clock on the machine that runs it.

What a reply needs is the time after the period ends: a reply is due when the transmit audio starts, 0.5 s after the boundary for FT8. For
`decode_prefix` that is the last figure of a line; for a whole-period call at `Fast`, all of it. Run it on the board and read those against 0.5 s. The
earlier checkpoints have the time to the next one (1.7 s from A to B, 1.5 s from B to the end for FT8).

## 6. Limits

- 12 kHz audio only; a file at another rate is refused rather than resampled.
- One request at a time. The decoders are not shared between processes; start one process per independent stream if you need two.
- `period` is what the caller says it is: the process has no clock.
