# rtl_tcp sample loss: how to experiment, judge results, and choose an implementation (#685, #686)

Written 2026-10-16 after three rounds (10-12, 10-14, 10-16) that each built client code, ran it, and
did not converge. This note fixes the method before any more client code is written. The synthetic
corpus it calls for is `apps/skimmer/core/examples/synth_rtltcp.rs`.

## 1. Why the previous rounds did not converge

| round | model of the fault | code built for it | what showed the model was wrong |
|---|---|---|---|
| 10-12 | the RSP1's ADC runs ~3 % slow (uniform rate error) | `RateStage`, `RateTracker`, 30 s `measure_rate`, `plannable_rate` | STFT lines steady to ±1 Hz: the clock is right (`RTLTCP_RATE.md`) |
| 10-14 | the API stalls ~93 ms every 1.85 s and loses part of each stall | 15 s grid for offset changes, pause inference, dbg3/dbg4 | dbg4 `F` records: ~3.5 small holes a second, not one large one per 1.85 s |
| 10-16 | numbered holes (2 107 in 600 s, mean ~10.5 ms, max 37 ms) plus three ~1.43 s resets | acquisition clock split from payload rate, `audio_rf_scale` | not yet tested against a known answer |

Four habits caused the churn, not one bad idea:

1. **Code before the fault model was confirmed.** Each round coded its hypothesis, and the code stayed as
   an option when the hypothesis fell. Five rtl_tcp knobs now combine freely; nobody can say which
   combination is right.
2. **A metric that could not answer the question.** Off-air decode counts were compared across
   different recordings (two different "245 s recordings"), commits, CLI vs GUI, NTP on/off and with a
   random 30 s calibration in each run. The spread between runs exceeded the effect being sought.
3. **No bounds.** Nothing measured what a perfect client would decode (holes zero-filled at their true
   positions) or what doing nothing decodes, so "27 rows" could not be called good or bad.
4. **Tests that encoded the hypothesis.** `FakeRtlServer::start_with_rate_error` and the periodic
   1.85 s holes of `iq_impairment_simulation.rs` reproduce rounds 1 and 2, not what dbg4 measured. They
   passed while the hardware failed.

## 2. Rules for every experiment from here

- **Fixed inputs.** Every comparison runs on the same files: the synthetic corpus (section 3), or one
  named real recording. A file is named by its SHA-256 in the result, not by its length.
- **Paired.** Candidates see byte-identical input. Signal, noise and loss are generated once per seed and
  shared by every variant (clean / delivered / oracle) and every client setting.
- **One variable at a time.** A comparison changes one client setting; everything else is written down
  (commit, flags, env) next to the result.
- **Known answer first.** Every synthetic transmission is in `truth.json`. A run is scored against it:
  no score, no conclusion. Off-air runs are exploratory and never decide between candidates.
- **Bounds in every table.** `clean` (no loss), `oracle` (loss zero-filled at the true position: the best
  any client can do without the server's help) and `delivered` with nominal handling (the floor).
- **Decide the acceptance rule before the run** (section 4), and keep runs that disprove a candidate.
- **Hardware last.** The RSP1 confirms a candidate the corpus already ranked; it does not choose one.

## 3. Data

### 3.1 Synthetic corpus (host, `synth_rtltcp`)

One run writes a directory:

| file | content |
|---|---|
| `clean.bin` | the source at 1 024 000 S/s, no loss |
| `delivered.bin` | what an unmodified `rsp_tcp` delivers: hole and reset samples removed, arrival times kept |
| `oracle.bin` | the same loss, zero-filled at the true positions (upper bound) |
| `truth.json` | every transmission (mode, dial, slot UTC, DT, audio Hz, SNR, text), every loss event, the parameters, file hashes |

All three `.bin` files are in `scripts/rtltcp/record.py` format (greeting included, a time per chunk),
so `recorded_iq::decode_recorded_iq`, `replay_rtltcp` and the GUI replay read them unchanged.

Signals: FT8 at 7 074 and 7 041 kHz, FT4 at 7 047.5 kHz, IQ centre 7 016 kHz (the RSP1 profile). Per
slot and channel, several transmissions at distinct audio frequencies, SNR cycling through a grid
(FT8 −24..0 dB, FT4 −20..0 dB) so every SNR gets the same count. White Gaussian noise, 8-bit
quantisation as `rsp_tcp` without `-E`.

Loss, two sources:

- **Measured statistics** (default): holes as a renewal process, 3.54 per second, mean 10.46 ms,
  longest 37 ms; resets of 1.43 s at 107, 335 and 440 s. These are the dbg4 plain-run numbers
  (`IQ_RSP1_PR688_2026-10-15.md`); the shape of the hole-length distribution is **assumed**
  (log-normal), since only the count, total and maximum were recorded.
- **Trace** (`--loss-trace`): the real hole sequence, from `rsp_clock.log` through
  `scripts/rtltcp/holes_from_clock_log.py`. Use it as soon as the 10-15 log is at hand; it removes the
  assumption above.

Delivery timing: samples leave the server in callbacks (50 ms by default), arrive after a fixed latency
plus jitter, and a hole or reset simply has no samples. A buffered stall (late but complete delivery) is
a separate option, for the pause-inference question.

### 3.2 Real recordings

The 600 s capture of 2026-10-15 has its dbg4 log, so its own oracle can be built the same way (zero-fill
at the logged positions). That gives a real-signal upper bound to set beside the synthetic one. Real
recordings rank nothing on their own (unknown transmitters, unknown DT).

## 4. How results are judged

Per mode, per candidate, scored against `truth.json`:

- **Decode probability against SNR**, with the threshold `SNR50` (50 % decoded) interpolated.
- **Loss to the oracle:** `SNR50(candidate) − SNR50(oracle)` in dB. The headline number.
- Slot-UTC correctness (the decode lands in the slot it was sent in), DT error, frequency error.
- False decodes (texts not in the truth), reanchors, gaps, dropped slots.

At least 3 seeds per setting; a difference counts only if it exceeds the spread between seeds.

Acceptance, decided now:

1. `clean` must decode at the expected SNR (reported SNR within ±2 dB of the truth) before anything else
   is read. Otherwise the generator or the harness is wrong.
2. A candidate is worth keeping when it closes at least half of the gap between the floor and the
   oracle for FT8 **and** does not make FT4 worse than the floor, with no false decodes added.
3. Between two candidates that both pass, the one with less code and fewer knobs wins unless the other is
   at least 1 dB better.

## 5. Implementation candidates (to be ranked, not chosen in advance)

| id | client behaviour | where it stands |
|---|---|---|
| A | nominal rate, nothing else | the floor |
| B | uniform stretch by measured payload rate, NCO at the nominal rate (`8caaf5a7` default) | audio pitch scaled ~4.4 % |
| C | uniform stretch, NCO at the measured rate (old ADC model) | known to displace channels far from the centre |
| D | nominal rate, zeros inserted evenly to make up the measured deficit | no pitch error; slips of < 1 ms; spurs to check |
| E | D, plus zero-fill of long pauses decided with a 1-2 s lookahead (late vs lost) | targets the 1.4 s resets |
| F | B or D with a calibration that ignores pauses (section 6) | fixes the 7 % bias of `measure_rate` |

Each is a setting of the same harness. Losers are deleted from the code, not kept as options.

## 6. Known defects to fix regardless of the ranking

- `measure_rate` fits one line through arrival times; a 1.43 s pause inside the 30 s window biases it by
  up to ~7 % and still passes the ±15 % gate (one connection in about six).
- `await_rate` cannot see the moment `rsp_tcp` reads its commands when the asked rate is its start-up
  rate (1 024 000, the default).
- The 15 s grid for applying a rate change falls inside 60 s / 120 s transmissions and JTTY.

## 7. Tools and first check (2026-10-16)

```sh
cargo run --release -p skimmer-core --example synth_rtltcp -- --out DIR --seconds 600 --seed 1
python3 -I scripts/rtltcp/holes_from_clock_log.py rsp_clock.log trace.csv     # then --loss trace:trace.csv
MFSK_RTL_RECORDING=DIR/delivered.bin MFSK_RTL_OUTPUT=rows.json MFSK_RTL_SKIP=0.001 MFSK_RTL_END=1000 \
  MFSK_RTL_EFFECTIVE_RATE=1024000 MFSK_RTL_DIAL=7074000 MFSK_RTL_CENTER=7016000 \
  cargo test -p skimmer-core --release --lib decode_recorded_iq -- --ignored --nocapture
python3 -I scripts/rtltcp/score_synth.py DIR/truth.json rows.json
```

The converter was checked on a hand-made log (two holes, a 1.43 s reset, a wrap of the 32-bit sample number):
it returned all three events at their exact positions.

First run, 120 s, seed 1, FT8 at 7 074 kHz only (91 transmissions, SNR −24..0 dB), measured-statistics loss
(4.2 % of samples, one reset):

| input | client | decoded | SNR50 | reported SNR error (median) |
|---|---|---:|---:|---:|
| clean | A (nominal) | 76/91 | −20.8 dB | −0.14 dB |
| oracle | A | 75/91 | −20.6 dB | −0.40 dB |
| delivered | A | 0/91 | — | — |
| delivered | B (stretch, the true mean payload rate, NCO nominal) | 31/91 | −11.0 dB | −3.0 dB |
| delivered | C (stretch, NCO at the payload rate) | 0/91 | — | — |

Rule 1 holds (the generator's SNR is the decoder's). The oracle loses 0.2 dB to clean: zero-filled holes in
the right place cost almost nothing. B, given the exact mean rate (no calibration error at all), is still
~10 dB behind the oracle and saturates near 60 % at strong signals; that is the ceiling of the current
default, and it matches the real captures' low counts. This is one seed and one channel: section 4's
3-seed rule applies before any ranking.

## 8. Order of work

1. Generate the corpus (3 seeds, 600 s) and check acceptance rule 1 on `clean`.
2. Extend `recorded_iq` to all three channels and add the scorer (truth vs rows, section 4).
3. Measure A, oracle and clean. If the oracle is not clearly above A, stop: the client cannot gain much.
4. Rank B to F.
5. Replace the assumed hole shape with the trace from the real log; repeat 3-4 on it.
6. Build the real recording's oracle; check the ranking holds on real signals.
7. Only then: implement the winner, delete the rest, confirm once on the RSP1.

## 9. Handoff to the host (2026-10-16)

The corpus was made in a cloud session and not kept (11 GB). Regenerate it on the host, in a directory
outside the repository:

```sh
cargo build --release -p skimmer-core --example synth_rtltcp
for s in 1 2 3; do
  target/release/examples/synth_rtltcp --out $CORPUS/seed$s --seconds 600 --seed $s
  (cd $CORPUS/seed$s && sha256sum *.bin truth.json > SHA256SUMS)
done
```

Reference checksums of the cloud run (x86_64 Linux, commit `233c8202`). `truth.json` contains no
floating-point arithmetic that depends on the CPU and must match exactly; if it does not, the generator or
its inputs changed. The `.bin` files go through rustfft and `libm`, whose results may differ in the last bit
on another CPU or OS; a mismatch there alone is not an error, but then record the host's own sums and use
only those from then on (rule: fixed inputs).

| seed | `truth.json` | `delivered.bin` |
|---|---|---|
| 1 | `df6e7ebff66ae2e080a6761ab2f9b727ffd3a58999e0a9ab03b76a717a60f43b` | `6ea1ab4eed76c73c3c59f45c15717f71ba329197052544521e1b2619ea53e888` |
| 2 | `3659574f4f02c0203e690bad690b6c27f489fb39433ed7b5a6d079ba6d8381b6` | `073d5b15a0ff452baec34a281e84890c2fb1ccac120f19e1e109ec6a2a9ff000` |
| 3 | `648fc08b01cc770c7cce53ae91ecb0f8000e7e73cc473211d250cf9e5432e33a` | `b1b01323fc552e7324d871deb85bbeb84170b109b8d651d7a9f1244eb8b77278` |

Per seed: 1 920 transmissions, 4.09 / 4.38 / 4.30 % of samples lost, 1 947 / 2 112 / 2 026 holes, 3 resets.

Next tasks, in order (section 8, steps 2-4):

1. **Harness, three channels.** `recorded_iq::decode_recorded_iq` takes one FT8 dial (`MFSK_RTL_DIAL`). Give it
   the corpus's three channels (FT8 7 074 / 7 041 kHz, FT4 7 047.5 kHz) and write `"mode"` and `"dial"` into
   each row; `score_synth.py` already filters by `--dial` and `--mode`.
2. **Bounds.** clean, oracle and delivered with A on the three seeds. Check rule 1 (clean's reported SNR
   within ±2 dB of the truth) for FT4 too, which the first check did not cover.
3. **Candidates.** B and C exist as `recorded_iq` settings (`MFSK_RTL_EFFECTIVE_RATE`,
   `MFSK_RTL_PIN_ACQUISITION`, `MFSK_RTL_TRACK`). Give B its rate two ways: the truth's mean payload rate
   (its ceiling) and the client's own 30 s `measure_rate` (what it gets in use). D and E need code in the
   harness first (zero insertion; lookahead pause fill). F is B or D with a pause-robust `measure_rate`.
4. **Trace.** Run `holes_from_clock_log.py` on the 2026-10-15 dbg4 `rsp_clock.log` (`--skip 30` drops the
   start-up) and regenerate with `--loss trace:FILE.csv`; repeat 2-3.
5. Record every table here with the commit, the flags and the `delivered.bin` checksum it ran on.

## 10. Results on the regenerated corpus (2026-10-17, steps 2-4 of section 9)

### 10.1 Inputs and method

- Corpus regenerated on the host at `233c8202`-era generator (repo HEAD `50e92010`), `synth_rtltcp --seconds 600 --seed 1..3`,
  outside the repository. **All three `truth.json` and all three `delivered.bin` SHA-256 match the section 9
  reference table exactly** (no mismatch to record); the host's `SHA256SUMS` is therefore the same table.
  `clean.bin` / `oracle.bin` were not compared with a reference (none was recorded); their sums are in each
  seed's `SHA256SUMS`.
- Harness: `f95116c1` (`recorded_iq`: three channels, `MFSK_RTL_MEASURE*`, D/E hooks; e387e0b5 is the same
  without D/E/the robust measure). Runner `scripts/rtltcp/eval_synth.py`, scorer `scripts/rtltcp/score_synth.py`
  (since e387e0b5 it scores only the rows of the channel in question; before that, rows of the other two
  channels were counted as false decodes). Bounds were run on e387e0b5 and re-run on f95116c1 with an identical
  output (diff empty); B/C rows run on e387e0b5, two of them (Bp_t1, Cm_t0) re-run on f95116c1, identical.
- Every run: `MFSK_RTL_SKIP=0.001 MFSK_RTL_END=1000 MFSK_RTL_CENTER=7016000`, channels FT8 7074 / FT8 7041 /
  FT4 7047.5 kHz in one pass, `release`, `rtl_minimum_arrival` on, `rtl_infer_gaps` off, `rtl_calibrate` zero
  in the harness (the calibration is replaced by the rate given, see below). The recording is the `.bin` of the
  named kind, same seed for every row. 507 FT8 and 869 FT4 transmissions per channel and seed (SNR grid of
  section 3.1), 3 seeds. In the tables "x / y / z" are seeds 1 / 2 / 3.
- `payload` rate = `delivered.bin` payload bytes / 2 / 600 s from `truth.json` = 982 068 / 979 114 / 979 973 S/s
  (includes the three 1.43 s resets). `measured` = what `RateMeter` returns on the recording for the client's
  window (5 s skip, then 30 s, the first 35 s of the file): 984 582 / 982 178 / 992 425 S/s. No reset falls in
  that window (they are at 107, 335, 440 s), so the 30 s measure is pause-free here by construction; the
  `at 100` rows below move the window over the 107 s reset.
- Flags of the settings: `T0`/`T1` = `MFSK_RTL_TRACK` 0/1 (T1 is the shipped client's setting); `pin` =
  `MFSK_RTL_PIN_ACQUISITION=1` (NCO at the nominal rate, B), no pin = C. B and C stretch the audio by the rate
  given (`MFSK_RTL_EFFECTIVE_RATE=payload` or `MFSK_RTL_MEASURE=1`); D/E keep the receiver at the nominal
  rate (`MFSK_RTL_ZERO_INSERT=1`, zeros inserted evenly, one run of zeros per 2048 input bytes, the deficit
  of the rate given); E adds `MFSK_RTL_PAUSE_FILL=1` (a gap over 0.5 s between records is zero-filled with
  `(gap - median interval) x 1 024 000` samples unless the next 1.5 s carry more than 1.1 x the rate, i.e. a
  catch-up burst; on this corpus every pause is lost, none delayed, so the late-or-lost test never takes the
  other branch and is not exercised); `all` = rate measured over the whole file, pauses removed from the time
  axis (a ceiling that uses the file's own mean, not a client-reachable number); F = `MFSK_RTL_MEASURE_ROBUST=1`.

### 10.2 Acceptance rule 1 (clean, reported SNR vs truth)

Median (reported - true) SNR, seeds 1 / 2 / 3: FT8 7074 -0.11 / -0.07 / -0.10 dB; FT8 7041 -0.07 / -0.12 /
-0.08 dB; **FT4 7047.5 -1.73 / -1.75 / -1.77 dB**. All within +-2 dB, so rule 1 holds, but FT4 is a steady
-1.75 dB from the truth even on clean input, and the oracle sits at -1.91 / -1.95 / -1.95 dB, 0.05 dB from the
limit. Not investigated (generator FT4 level or decoder SNR convention); no FT4 SNR-accuracy claim is made from
this corpus beyond the +-2 dB gate.

### 10.3 Bounds (step 2)

| job | channel | decoded (seeds 1/2/3) | SNR50 dB (1/2/3) | wrong slot | false |
|---|---|---|---|---|---|
| clean | FT8 7074 | 431 / 433 / 427 of 507 | -21.1 / -21.1 / -20.9 | 0 | 0 |
| clean | FT8 7041 | 431 / 427 / 432 of 507 | -21.1 / -20.9 / -21.1 | 0 | 0 |
| clean | FT4 7047.5 | 761 / 763 / 757 of 869 | -18.4 / -18.4 / -18.3 | 0 | 0 |
| oracle | FT8 7074 | 423 / 427 / 420 of 507 | -20.8 / -20.9 / -20.7 | 0 | 3 |
| oracle | FT8 7041 | 424 / 423 / 423 of 507 | -20.8 / -20.8 / -20.8 | 0 | 1 |
| oracle | FT4 7047.5 | 748 / 754 / 743 of 869 | -17.9 / -18.2 / -17.7 | 0 | 0 |
| A (delivered, nominal) | FT8 7074 | 0 / 0 / 0 of 507 | not reached | 0 | 1 |
| A | FT8 7041 | 0 / 0 / 0 of 507 | not reached | 0 | 0 |
| A | FT4 7047.5 | 0 / 0 / 1 of 869 | not reached | 0 | 0 |

(A shows 20-21 re-anchors per run; clean and oracle show none.) The oracle loses 0.1-0.4 dB (FT8) and 0.2-0.5
dB (FT4) to clean; its 4 false decodes over 9 channel-seeds (clean: 0) are texts that are not in the truth,
not looked at. **The oracle is clearly above A**: A decodes nothing, the oracle 83 % of FT8 and 86 % of FT4
transmissions. Section 8 step 3's stop condition does not apply, so B-F were run.

### 10.4 Candidates (steps 3, hole shape assumed: log-normal, measured statistics)

Gap closed = decoded count / the oracle's mean (423 for FT8 over 6 channel-seeds, 748 for FT4 over 3), because
A decodes nothing and has no SNR50. False = over all 9 channel-seeds. Per-seed numbers are in the next table.

| job | setting | FT8 decoded (of 507) | FT8 gap closed | FT4 decoded (of 869) | FT4 gap closed | false |
|---|---|---:|---:|---:|---:|---:|
| A | nominal | 0 | 0 % | 0 | 0 % | 1 |
| Bp_t0 | B, payload rate, pin, T0 | 296 | 70 % | 165 | 22 % | 5 |
| Bp_t1 | B, payload rate, pin, T1 | 289 | 68 % | 169 | 23 % | 3 |
| Bm_t0 | B, 30 s measure, pin, T0 | 300 | 71 % | 193 | 26 % | 2 |
| **Bm_t1** | **B, 30 s measure, pin, T1 (shipped)** | 297 | 70 % | 203 | 27 % | 0 |
| Cp_t0 / Cp_t1 | C, payload rate, no pin | 80 / 80 | 19 % | 65 / 65 | 9 % | 0 |
| Cm_t0 / Cm_t1 | C, 30 s measure, no pin | 105 / 106 | 25 % | 90 / 95 | 12-13 % | 1 / 0 |
| Dm_t0 / Dm_t1 | D, 30 s measure | 254 / 254 | 60 % | 145 / 153 | 19-20 % | 2 / 4 |
| Dall_t0 / Dall_t1 | D, whole-file robust rate | 266 / 266 | 63 % | 165 / 161 | 22 % | 2 / 2 |
| Em_t0 / Em_t1 | E, 30 s measure | 278 / 294 | 66 / 69 % | 157 / 184 | 21 / 25 % | 2 / 3 |
| Eall_t0 / Eall_t1 | E, whole-file robust rate | 310 / 309 | 73 % | 195 / 193 | 26 % | 4 / 3 |

Per seed, FT8 SNR50 in dB (1 / 2 / 3), and FT8 decoded (1 / 2 / 3). "n/r" = 50 % never reached. FT4 never reaches
50 % under any candidate except two seeds of E (see below), so only counts are given for it.

| job | FT8 7074 SNR50 | FT8 7074 decoded | FT8 7041 SNR50 | FT8 7041 decoded | FT4 decoded |
|---|---|---|---|---|---|
| oracle | -20.8 / -20.9 / -20.7 | 423 / 427 / 420 | -20.8 / -20.8 / -20.8 | 424 / 423 / 423 | 748 / 754 / 743 |
| Bp_t0 | -16.5 / -16.3 / -16.8 | 304 / 284 / 297 | -16.6 / -16.1 / -16.8 | 302 / 290 / 296 | 168 / 170 / 157 |
| Bp_t1 | -16.8 / -16.5 / -16.8 | 302 / 282 / 287 | -16.8 / -15.9 / -16.6 | 300 / 278 / 286 | 180 / 174 / 154 |
| Bm_t0 | -16.7 / -16.5 / -16.9 | 308 / 292 / 296 | -17.1 / -16.6 / -16.9 | 314 / 295 / 293 | 194 / 188 / 196 |
| Bm_t1 | -16.6 / -16.6 / -17.1 | 301 / 298 / 290 | -17.2 / -16.8 / -16.9 | 309 / 300 / 286 | 210 / 196 / 204 |
| Cp_t0 | n/r x3 | 0 / 0 / 0 | -12.3 / -2.5 / -4.4 | 172 / 152 / 158 | 68 / 63 / 64 |
| Cm_t0 | n/r x3 | 0 / 0 / 77 | -12.5 / -13.5 / -15.7 | 183 / 166 / 204 | 89 / 73 / 108 |
| Cm_t1 | n/r x3 | 13 / 0 / 64 | -12.8 / -14.5 / -16.1 | 191 / 172 / 196 | 98 / 81 / 105 |
| Dm_t0 | -13.0 / -11.6 / -15.9 | 228 / 204 / 276 | -15.6 / -14.7 / -15.1 | 288 / 262 / 264 | 136 / 115 / 183 |
| Dm_t1 | -12.9 / -11.4 / -16.2 | 228 / 207 / 277 | -15.6 / -14.7 / -15.2 | 281 / 266 / 267 | 152 / 124 / 182 |
| Dall_t0 | -15.6 / -12.9 / -14.8 | 280 / 216 / 248 | -16.4 / -15.4 / -16.3 | 293 / 268 / 288 | 193 / 155 / 146 |
| Dall_t1 | -15.5 / -12.8 / -14.8 | 280 / 218 / 251 | -16.5 / -15.1 / -16.4 | 295 / 265 / 289 | 192 / 152 / 140 |
| Em_t0 | -13.0 / -12.4 / -16.4 | 238 / 233 / 310 | -15.9 / -14.8 / -15.8 | 306 / 287 / 294 | 160 / 122 / 190 |
| Em_t1 | -13.3 / -12.4 / -16.5 | 261 / 241 / 325 | -16.3 / -14.9 / -16.2 | 325 / 298 / 315 | 182 / 141 / 230 |
| Eall_t0 | -16.1 / -13.4 / -15.2 | 319 / 255 / 291 | -16.8 / -15.9 / -16.8 | 342 / 314 / 336 | 232 / 172 / 181 |
| Eall_t1 | -15.8 / -13.4 / -15.2 | 315 / 261 / 293 | -16.8 / -15.9 / -16.8 | 342 / 303 / 338 | 234 / 170 / 174 |

Reading, within what the numbers show (no cause is named; none was measured):

- **B is far ahead of A and of C.** The shipped setting (Bm_t1) decodes 70 % of the oracle's FT8 count and
  is 3.7-4.4 dB short of the oracle in SNR50 (seed spread of Bm_t1: 0.5 dB at 7074, 0.4 dB at 7041). Giving B
  the truth's mean payload rate instead of its own 30 s measure changes it by 0.0-0.6 dB: not beyond the seed
  spread. T0 vs T1 is also inside the spread. Rule 2 holds for B: FT8 gap closed 70 % (> half), FT4 above
  the floor (A = 0), no false decodes added (Bm_t1: 0; oracle has 4).
- **C displaces the 7074 kHz channel** (58 kHz from the centre): 0 decodes in 5 of 6 seed-runs, the 7041 kHz
  channel (25 kHz) works partly; the seed spread is 10+ dB. C fails rule 2.
- **D and E do not beat B in FT8.** E with the 30 s measure (Em_t1) is 3-4 dB worse than B at 7074 kHz in
  seeds 1 and 2 and equal in seed 3, with a seed spread of 4 dB (-13.3 / -12.4 / -16.5), larger than any
  difference to B. Eall (a rate no client can have) is at best equal to B (7041: -16.8 / -15.9 / -16.8 vs
  Bm_t1 -17.2 / -16.8 / -16.9), still worse at 7074. Rule 3 (the one with less code wins unless the other is
  1 dB better) then favours B: E is nowhere 1 dB better than B.
- **E's pause fill helps D**: Em over Dm +24 / +40 FT8 decodes (means), the filled samples were 4.45-4.48 M per
  run (3 x 1.43 s x 1 024 000 = 4.39 M plus arrival jitter), so the three resets were each found and filled.
  The gain does not carry E past B.
- **FT4 is weak in every candidate**: B decodes about 44 % of FT4 transmissions even at 0 dB SNR
  (e.g. Bm_t1 seed 1: 35/79 at 0 dB, 34/79 at -4 dB), where the oracle decodes 79/79 from -14 dB. The only SNR50 values
  FT4 reaches are two E runs (Eall_t0 seeds 1, 2: -4.1 / -2.1 dB). It is not worse than the floor (A = 0).
  FT4 under E also shows frequency errors of order -180 Hz in the median (Eall_t1 seed 1), unexplained.
- The first-check B of section 7 (-11.0 dB, 120 s, one channel, seed 1) is not comparable with Bp_t0 here
  (-16.5 dB, 600 s): different length and number of resets, so no conclusion is drawn from the difference.

### 10.5 Candidate F: a calibration window with a pause in it

The 30 s window moved to `MFSK_RTL_MEASURE_AT=100` (window 105-135 s, over the 107 s reset), T0, pin for B:

| job | rate measured (1 / 2 / 3) | FT8 decoded, 7074 (1 / 2 / 3) | FT8 7074 SNR50 | FT4 decoded |
|---|---|---|---|---|
| Bm100_t0 (B, plain `measure_rate`) | 963 297 / 965 821 / 972 292 | 30 / 75 / 206 | n/r / n/r / -13.0 | 11 / 25 / 94 |
| **Bm100r_t0 (F on B)** | 983 616 / 986 078 / 992 862 | 320 / 289 / 298 | -16.9 / -16.5 / -16.9 | 204 / 211 / 200 |
| Dm100_t0 (D, plain) | 963 297 / 965 821 / 972 292 | 11 / 51 / 160 | n/r / n/r / -10.8 | 11 / 19 / 68 |
| **Dm100r_t0 (F on D)** | 983 616 / 986 078 / 992 862 | 228 / 214 / 271 | -12.8 / -12.6 / -15.5 | 143 / 153 / 187 |

A pause in the window moves the measured rate by 1.7-2.0 % (20 000 S/s) and takes the decode count to 10-70 %
of the pause-free run (seed 3 less than the others, 206 vs 296-308). The pause-robust rate returns the counts
and SNR50 to the pause-free values (Bm100r vs Bm_t0: 320 / 289 / 298 vs 308 / 292 / 296). This confirms the
section 6 defect (`measure_rate` sees a 1.43 s pause as a lower rate), with the rate error here ~2 % on a 30 s
window rather than the 7 % worst case quoted there (that figure was not reproduced). Only the window position was varied; a window with no pause in it (the default here) is
unaffected by F.

### 10.6 Ranking (rule 2, rule 3)

1. **B** (pin, stretch by the measured rate) passes rule 2 and is the simplest. C fails (channel
   displacement). D and E pass the FT8 half-gap test (60-73 %) but are not 1 dB better than B anywhere, so by
   rule 3 B wins on code size. F (pause-robust `measure_rate`) is not a rival to B: it removes a failure of
   the calibration that B (and D) depend on, and is worth doing whichever is chosen.
2. The gap that remains is the oracle's: B is 3.7-4.4 dB (FT8) behind it and decodes 44 % of FT4 at strong SNR.
   Closing that is not a matter of the candidates listed here.
3. These are results on a *simulated* hole shape (log-normal) with the generator's fixed 20 ms latency, 5 ms jitter and 50 ms callbacks; the trace run
   (section 9 step 4) is the check on the assumption and was **not possible**, see below.

### 10.7 Not done

- **Step 4 (trace): skipped at the time of section 10, done in section 12.** The 2026-10-15 dbg4 `rsp_clock.log` is not on this host (searched `/`, `/mnt/c/Users/minor`,
  `/home`, including the dbg4 folders on the Desktop and `build`; `IQ_RSP1_PR688_2026-10-15.md` records that
  the log is on the NAS and each batch deleted the previous). Nothing was fabricated; `holes_from_clock_log.py` was
  not run and sections 10.3-10.6 were not repeated with `--loss trace:`. Next: copy `rsp_clock.log` here, run
  `holes_from_clock_log.py rsp_clock.log trace.csv --skip 30`, regenerate with `--loss trace:trace.csv`, and run
  `eval_synth.py` with the jobs above (the runner reuses nothing across output directories).
- The late-vs-lost branch of E (delayed pauses with a catch-up burst) is not exercised: the corpus has no
  buffered stalls (`params.buffered` is empty).
- Section 8 steps 5-7 (real-recording oracle, implementing the winner, the RSP1) are untouched.
- D/E/F exist only in `recorded_iq.rs`; `rtltcp.rs` is unchanged.
- Run durations, CPU class and wall time were not recorded and do not enter any result.

## 11. Why FT4 decodes so little under B (2026-10-17)

Question: Bm_t1 (the shipped setting) decodes ~44 % of FT4 at 0 dB (35/79) where the oracle decodes 79/79 from -14 dB
and FT8 under B reaches 70 %. Nothing in the client or the generator was changed; harness `e9045a25` unchanged,
new files are `scripts/rtltcp/tx_features.py` (per-transmission join of truth, loss and outcome) and
`scripts/rtltcp/hole_patterns.py` (the control traces).

### 11.1 Candidates and the measurement that decides each (written before any run)

| id | candidate | confirmed if / excluded if |
|---|---|---|
| a | the failing set has structure (slot parity, start offset, DT, time since a reset, seed) | hit rate at strong SNR depends on it / does not |
| b | slot alignment / time base (DT of decoded vs missed, parity, clock offset) | failures follow the client's clock offset / do not |
| c | hole shape: B deletes the hole samples and stretches by one global rate, so the audio time of a frame is off by (loss in the frame - assumed loss); a 5 s FT4 frame (105 x 48 ms symbols) is shorter and tighter than a 12.6 s FT8 frame (79 x 160 ms) | hit rate falls with that mismatch, and a corpus with the same count/total/resets but no mismatch recovers / not |
| d | harness artefact of the FT4 channel | B on a lossless corpus reproduces clean for FT4 / not |
| e | the -1.75 dB FT4 SNR offset is related | it is absent without loss, or differs between decoded and missed / is present on clean and independent |
| f | resets (3 x 1.43 s) | misses cluster after resets / do not |

### 11.2 Inputs

Seeds 1-3, 600 s, `delivered.bin` of the section 9 corpus (SHA-256 `6ea1ab4e...` / `073d5b15...` / `b1b01323...`, truth
sums as in section 9), setting Bm_t1 = `MFSK_RTL_MEASURE=1 MFSK_RTL_TRACK=1 MFSK_RTL_PIN_ACQUISITION=1`, run through
`eval_synth.py` at `e9045a25`. It reproduces section 10: FT4 210 / 196 / 204 of 869, FT8 7074 301 / 298 / 290.
Controls are new corpora from the same generator, same seeds, `--seconds 600`; the `transmissions` array and
`clean.bin` are identical to the section 9 corpus (checked: `truth.json["transmissions"]` equal, `clean.bin` SHA-256
`1341719980e2578f...` for seed 1), only the loss differs. Per-seed truth / delivered prefixes:

| corpus | loss | delivered.bin seed 1 / 2 / 3 (first 16 hex) |
|---|---|---|
| shipped | log-normal lengths, Poisson spacing, 3 resets | 6ea1ab4eed76c73c / 073d5b15a0ff452b / b1b01323fc552e73 |
| none | `--loss none` | 1341719980e2578f / b64217633c6ad4c5 / e46bc9f890053558 |
| periodic | `hole_patterns.py periodic`: equal spacing, all 10.46 ms, same resets | 61a061f56e270f91 / e7958d767c1b336a / ba580a53bb1b1f96 |
| perlen | equal spacing, lengths drawn from the seed-1 holes | d324843058f06453 / df7d18ae6edf23d7 / ab638a190e5134ab |
| poisconst | Poisson spacing, all 10.46 ms | 1a05dc528dca5e4d / 56aec65f578b20cc / 1e96248caf58fa84 |

Lost fraction 4.39 % in periodic, 4.40 % in perlen; 4.1-4.4 % in the shipped corpus.

### 11.3 Measurements

**(a) The failing set.** At strong SNR (FT4 >= -6 dB, 960 transmissions over 3 seeds; the oracle decodes all of them)
B decodes 39-40 %. By slot parity 35 % / 44 %, by seed 41 / 38 / 40 %, by DT bin 35-42 %: no structure there. By slot
it is bimodal: of 240 FT4 slots with 4 strong transmissions each, 110 have 0 of 4 decoded and 59 have 4 of 4
(independent hits at p = 0.4 would give ~31 and ~6). By time since a reset: 0/24 (0-5 s), 1/72 (5-20 s), 11/84
(20-40 s), 47/108 (40-60 s), 244/468 (>60 s). FT8 at strong SNR (>= -12 dB) is 98 % outside the reset windows.

**(b) Time base.** The median DT error of FT8 hits wanders by +-100 ms in steady state and is -1.3..-1.5 s for ~30 s
after each reset (until the re-anchor). Binning FT4 slots by the FT8 clock offset of the same 15 s slot: offsets of
-500..+150 ms give 1.6-2.4 of 4 hits per slot with no trend (22 slots with offset < -500 ms give 0.05). So a clock
offset of +-0.1 s does not explain the misses; the ~1.4 s offset after a reset kills FT4 (see f).

**(c) Hole shape, correlational.** Per transmission (steady state: > 40 s from start and reset, no reset in the
frame, FT4 >= -6 dB, n = 672) the "mismatch" is the samples lost inside its 5.04 s frame minus what the client's
stretch assumes (`1 - rate/1 024 000` of the frame, per seed 3.85 / 4.08 / 3.08 %). Hit rate by mismatch octile:
0.18 (-125..-65 ms), 0.60, 0.60, 0.59, 0.68, 0.71 (+3..+27), 0.36 (+27..+58), 0.14 (+58..+144). Std of the mismatch
is 51 ms (1.1 FT4 symbols). The same table for FT8 (12.64 s frame, n = 588) is 0.99-1.00 everywhere except 0.97 /
0.89 in the two positive octiles (+43..+277 ms, i.e. < 2 FT8 symbols).

**(c) Hole shape, causal controls.** B (Bm_t1) on the same signals, only the loss arrangement changed:

| loss | mismatch std, FT4 (ms) | steady-state strong-SNR FT4 hit | FT4 decoded (seeds 1/2/3 of 869) | FT4 SNR50 dB | FT8 7074 decoded (of 507) |
|---|---:|---:|---|---|---|
| none (B = clean) | - | - | 762 / 762 / 756 (clean 761 / 763 / 757) | -18.5 / -18.4 / -18.2 | 431 / 431 / 428 |
| periodic | 3.8 | 0.98 | 565 / 570 / 573 | -16.1 / -16.3 / -16.4 | 325 / 322 / 327 |
| perlen (random lengths) | 23.7 | 0.87 | 404 / 409 / 407 | -12.8 / -11.9 / -12.3 | 306 / 297 / 311 |
| poisconst (random spacing) | 41.1 | 0.55 | 236 / 231 / 236 | -1.7 / -2.2 / -0.7 | 303 / 295 / 302 |
| shipped (both random) | 51.4 | 0.49 | 210 / 196 / 204 | n/r | 301 / 298 / 290 |
| oracle (shipped loss) | - | - | 748 / 754 / 743 | -17.9 / -18.2 / -17.7 | 423 / 427 / 420 |

With the same hole rate, mean length, total loss and resets, making the holes regular gives FT4 565-573 decodes
(2.7x the shipped corpus) and 98 % at strong SNR in steady state; FT4 hit rate and SNR50 fall in step with the
mismatch as spacing and then length are randomised. FT8 moves only 290-301 -> 325 (7074 kHz).

**(d) Harness.** B on `--loss none` reproduces clean for FT4 (762 / 762 / 756 vs 761 / 763 / 757, SNR50 within
0.1 dB), so the FT4 channel, its filter and its position (7047.5 kHz, 31.5 kHz from the centre) are not the cause.

**(e) -1.75 dB.** B on `--loss none` has the same FT4 SNR error as clean (-1.74 / -1.75 / -1.77 dB) and periodic /
oracle -1.95..-2.07; with the shipped holes -2.03. It is present without any loss and does not depend on the
loss, so it is independent of this question. Its cause is still not determined.

**(f) Resets.** In the periodic corpus (no mismatch) FT4 steady state is 0.98, but the 0-5 s / 5-20 s / 20-40 s windows
after a reset give 1/24, 2/72, 36/84 and the first 40 s of the file 60/72; overall strong-SNR hit rate is 0.82. FT8 in
the same corpus: 63/63 in 5-20 s, then 0/63 in 20-40 s (not understood; same in the shipped corpus: 56/63 and 0/63),
overall 0.82. So the resets cost both modes about the same share (~16-18 pp) and are not what separates FT4 from FT8.

### 11.4 Conclusion

- Determined by control: the FT4 deficit is caused by the irregularity of the holes under B's uniform stretch. Same
  signals, same loss totals, regular holes: FT4 565-573 decodes instead of 196-210, and in steady state 98 % instead
  of 49 % at strong SNR. Randomising spacing alone costs 0.98 -> 0.55, lengths alone 0.98 -> 0.87.
- Correlational support: FT4 hit rate drops from ~0.6-0.7 to 0.14-0.18 once the in-frame mismatch exceeds roughly
  +-40 ms (about one 48 ms symbol over 5 s); a hole-free time axis error of +-0.1 s (clock offset) does not matter.
- FT4 is hit harder than FT8 because the same irregularity is a bigger fraction of a symbol and FT4's tolerance
  window is narrower; the mismatch std of 51 ms is ~1 FT4 symbol, ~0.5 FT8 symbol (FT8's frame is 2.5x longer, so its
  absolute mismatch is larger, but FT8 still decodes 98 %). Why FT4 tolerates only ~40 ms was not measured (not tied to
  a decoder stage); that is the part stated by comparison of the two modes, not isolated.
- Independent and smaller: 3 resets cost ~16-18 pp of strong-SNR transmissions in both modes. Not the cause of the gap.
- Excluded: slot parity / offset / DT bins, a clock offset of +-0.1 s, the FT4 channel and its position, the -1.75 dB
  offset (present without loss).

### 11.5 Not verified

- The "mismatch" is a proxy: at |mismatch| < 25 ms the shipped corpus still decodes only 0.66 (periodic: 0.98), so
  the within-frame shape of the time error (not only its integral) matters. Not separated.
- Whether the harm is the time-scale error itself or something B does at each hole (the stretch resampler across a
  discontinuity) was not separated: periodic and Poisson corpora have the same number of discontinuities, which makes a
  pure discontinuity effect unlikely, but a fix was not tried (out of scope).
- Hole shape is the assumed log-normal (no trace; the real trace is checked in section 12); the real arrangement may be more or less irregular. The periodic
  corpus is a control, not a model of the RSP1.
- Real-signal check, other FT4 parameters (depth, AP), and the unexplained FT8 0/63 window 20-40 s after a reset
  were not examined. No candidate fix (e.g. a local rate estimate, D/E on this corpus) was evaluated here.

## 12. Repeat on the real hole sequence (2026-10-17, section 9 task 4)

Question: do the section 10 rankings and the section 11 FT4 diagnosis (irregular hole arrangement under one global
stretch rate) hold when the loss is the real dbg4 hole sequence instead of the assumed log-normal one? Only the loss
differs from sections 10/11; the client, the harness (`recorded_iq.rs`, last changed `f95116c1`) and the signals are
unchanged. Harness-only edits for this section: `score_synth.py --end` and `tx_features.py` `TXF_END` (both default to
the old behaviour; the section 10 tables re-score to the same numbers), and the new `scripts/rtltcp/trace_controls.py`.

### 12.1 Inputs

- **Trace.** `rsp_clock.log` (dbg4), copied from `\\TOMO-NAS\home\pc-desktop\rsp_clock.log`, mtime 2026-10-16 00:05,
  SHA-256 `c2dc520c4e8892c403c0240deae6e826c49bebd94630c9905baa7d589d263288`. Converted with
  `python3 -I scripts/rtltcp/holes_from_clock_log.py rsp_clock.log trace.csv --skip 30`: SHA-256 of `trace.csv`
  `73886797349c16c9475271441daa3afd365f408ddfff74e0facd5b944d53df3d`; 11 131 C/F pairs, 2 020 holes (21 627 122 samples)
  and 3 resets (estimated from the wall clock: 1.430 / 1.396 / 1.425 s, at 75.35 / 302.86 / 408.46 s of the stream),
  span 570.2 s.
- **Provenance** of that log is by matching numbers, not by a recorded run id: `IQ_RSP1_PR688_2026-10-15.md` says the NAS
  copy was dated Oct 12; this file is dated Oct 16 and agrees with the 10-15/16 dbg4 run (about 2 100 holes in 600 s,
  3 resets of ~1.4 s). Section 1's 2 107 holes in 600 s (3.51 / s) and this log's 2 020 in 570.2 s (3.54 / s) are about the same rate.
- **How `synth_rtltcp --loss trace:` handles the 570 s vs 600 s difference** (read from `plan_loss`, `synth_rtltcp.rs`): it
  reads `kind,start_sample,len` rows as they are (no scaling, no looping), sorts them, merges overlaps (a hole inside a
  reset becomes part of it; none did here), drops every event that starts at or after `seconds x 1 024 000` and clips
  one that crosses the end. Nothing wraps and nothing is added after the last row, so **the last 30 s of every 600 s
  corpus are loss-free** (last event ends at 570.005 s), and sample 0 is the first record after the log's 30 s skip, so
  the resets are at 75 / 303 / 408 s instead of 107 / 335 / 440 s. Consequence: any score over the whole file is
  flattered by a loss-free tail (A decodes 11-13 FT4 there, where it decodes none in the first 570 s). **All tables in
  this section therefore count only transmissions that end before 570 s** (`score_synth.py --end 570`; 481 FT8 and 825
  FT4 per channel and seed). The log-normal columns are re-scored the same way (first 570 s) so the two are comparable;
  the section 10 numbers (507 / 869) are unchanged.
- **Corpus.** `synth_rtltcp --seconds 600 --seed S --loss trace:trace.csv` for S = 1, 2, 3, release binary built at
  `219cd8d4` (SHA-256 `a5a24a78a9ca80dfad4c94a088d39581362bcf037a248d5eb7121e15e2cad409`), outside the repository.
  Checked against the section 9 log-normal corpus: `truth.json["transmissions"]` and `["stream"]` equal for all three
  seeds, and `clean.bin` SHA-256 identical for all three (so signals are the same); only `loss`, `stats`, `params.loss`,
  `delivered.bin` and `oracle.bin` differ. The trace is the same for all three seeds (total loss 25 979 990 samples =
  4.23 %, payload rate 980 700 S/s in each), so **the seed spread below is the spread of signals and noise only, not of
  the loss**; the log-normal corpus had a different loss realisation per seed.

  | seed | `truth.json` | `delivered.bin` | `oracle.bin` |
  |---|---|---|---|
  | 1 | `251ca8c14da3999158fcb986584794bed572d57241410adfd6c05b0be62615e7` | `b13ee40aa5e0c5ef1763586cdfacff5984159ed1d9004c38d8933294b33175c3` | `a388162d0566f33355345efa672f53b78fbd7d405e1c7a3dc783013489897825` |
  | 2 | `c6a14c77120509fe465cfbdab0e32f5d9596d7b0fbfc3498caa4a24b5f443b93` | `72865d6bf9aad0f75d114517e26f5ed1517cb6a19c2af9a75a7257f8430a191d` | `5f26892b53d9bf5701cae7ebc783bd8e7f29dea190483a0489f7b0bf43118aa0` |
  | 3 | `b93cc1e28a76a8201a760d4d046cf4e1fbd74105bdc45f8f72e0c5184f669d75` | `a81b59ba52b0bea0c1f3bdd391dc4042cb193d92e55d26ea051048784a692e03` | `7349770733ddc68be882e998cd4165254f334b95582088826cd211dfdd1d4cb9` |

  (`clean.bin`: `1341719980e2578f...` / `b64217633c6ad4c5...` / `e46bc9f890053558...`, the same as section 9/11.)
- **Flags.** As section 10.1 (`MFSK_RTL_SKIP=0.001 MFSK_RTL_END=1000 MFSK_RTL_CENTER=7016000`, three channels, release,
  `rtl_minimum_arrival` on, `rtl_infer_gaps` off), through `eval_synth.py`, 8 jobs in parallel. Job names and settings as
  in 10.1 (`p` = payload rate, `m` = 30 s measure, `all` = whole-file robust rate, `t0/t1` = `MFSK_RTL_TRACK`). The `all`
  rate here is `MFSK_RTL_MEASURE_WINDOW=600` with `MEASURE_ROBUST=1`; the section 10 `Dall/Eall` window setting was not
  recorded, and re-running the nearest settings on the section 9 corpus gave a rate 0.003 % different (988 812 vs
  988 786 S/s, seed 1) and a handful of decodes different (the other probed windows: 540 s 988 631, 570 s 988 719), so the
  `all` rows of the two corpora are the same setting only to that tolerance. The other jobs I re-ran this way
  (`Bm100_t0`, `Dm100r_t0`) reproduced section 10's rates and decoded sets exactly.
- **Measured rates** (what the client's 30 s `measure_rate` window, 5-35 s, returns): 986 864 / 986 903 / 986 875 S/s
  against the payload rate 980 700 (+0.63 %); on the log-normal corpus 984 582 / 982 178 / 992 425 against 982 068 /
  979 114 / 979 973.

### 12.2 Acceptance rule 1 and bounds

Clean's reported SNR error is the same as section 10.2 (FT8 -0.07..-0.12 dB, FT4 -1.74..-1.77 dB), so rule 1 holds.

| job | channel | decoded (seeds 1/2/3) | SNR50 dB (1/2/3) | wrong slot | false |
|---|---|---|---|---|---|
| clean | FT8 7074 | 408 / 411 / 405 of 481 | -21.0 / -21.1 / -20.9 | 0 | 0 |
| clean | FT8 7041 | 409 / 404 / 410 of 481 | -21.1 / -20.9 / -21.1 | 0 | 0 |
| clean | FT4 7047.5 | 723 / 725 / 719 of 825 | -18.4 / -18.5 / -18.3 | 0 | 0 |
| oracle | FT8 7074 | 399 / 402 / 400 of 481 | -20.7 / -20.9 / -20.7 | 0 | 1 |
| oracle | FT8 7041 | 398 / 400 / 406 of 481 | -20.7 / -20.7 / -21.0 | 0 | 1 |
| oracle | FT4 7047.5 | 714 / 707 / 705 of 825 | -18.2 / -17.8 / -17.9 | 0 | 0 |
| A (delivered, nominal) | FT8 7074 | 6 / 7 / 8 of 481 | not reached | 0 | 2 |
| A | FT8 7041 | 7 / 7 / 8 of 481 | not reached | 0 | 0 |
| A | FT4 7047.5 | 0 / 0 / 0 of 825 | not reached | 0 | 0 |

The oracle loses 0.2-0.5 dB to clean (FT8 0.3-0.4, FT4 0.2-0.6 dB), as in section 10.3, and is clearly above A
(A: 1-2 % of FT8, 0 FT4; log-normal A: 0). A's 6-8 FT8 decodes are in two slots (135 s and 195 s, seed 1; 13 rows),
which do not have unusually few holes (36-37 against a median of 46 per 13 s); why those two decode was not examined.
The stop condition of section 8 step 3 does not apply.

### 12.3 Candidates (same layout as 10.4; trace, first 570 s)

Gap closed = decoded count / the oracle's mean on the same corpus and span (FT8 400.8, FT4 708.7; log-normal first
570 s: 401.5 and 711.0). Counts are means over the channels (FT8: 2 channels x 3 seeds) and seeds; false over all 9
channel-seeds.

| job | setting | FT8 decoded (of 481) | FT8 / oracle (trace) | same, log-normal first 570 s | FT4 decoded (of 825) | FT4 / oracle (trace) | same, log-normal | false (trace) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| A | nominal | 7 | 2 % | 0 % | 0 | 0 % | 0 % | 2 |
| Bp_t0 | B, payload rate, pin, T0 | 246 | 61 % | 71 % | 121 | 17 % | 22 % | 3 |
| Bp_t1 | B, payload rate, pin, T1 | 233 | 58 % | 68 % | 117 | 16 % | 23 % | 0 |
| Bm_t0 | B, 30 s measure, pin, T0 | 250 | 62 % | 71 % | 161 | 23 % | 26 % | 2 |
| Bm_t1 | **B, 30 s measure, pin, T1 (shipped)** | 250 | 62 % | 70 % | 159 | 22 % | 28 % | 5 |
| Cp_t0 | C, payload rate, no pin, T0 | 65 | 16 % | 19 % | 46 | 6 % | 8 % | 2 |
| Cp_t1 | C, payload rate, no pin, T1 | 66 | 16 % | 19 % | 44 | 6 % | 9 % | 0 |
| Cm_t0 | C, 30 s measure, no pin, T0 | 89 | 22 % | 25 % | 71 | 10 % | 12 % | 0 |
| Cm_t1 | C, 30 s measure, no pin, T1 | 87 | 22 % | 25 % | 70 | 10 % | 13 % | 1 |
| Dm_t0 | D, 30 s measure, T0 | 220 | 55 % | 60 % | 122 | 17 % | 20 % | 5 |
| Dm_t1 | D, 30 s measure, T1 | 218 | 54 % | 60 % | 120 | 17 % | 21 % | 2 |
| Dall_t0 | D, whole-file robust rate, T0 | 206 | 51 % | 63 % | 108 | 15 % | 22 % | 1 |
| Dall_t1 | D, whole-file robust rate, T1 | 206 | 51 % | 64 % | 112 | 16 % | 22 % | 2 |
| Em_t0 | E, 30 s measure, T0 | 256 | 64 % | 66 % | 144 | 20 % | 21 % | 2 |
| Em_t1 | E, 30 s measure, T1 | 250 | 62 % | 70 % | 151 | 21 % | 25 % | 4 |
| Eall_t0 | E, whole-file robust rate, T0 | 242 | 60 % | 74 % | 135 | 19 % | 26 % | 4 |
| Eall_t1 | E, whole-file robust rate, T1 | 238 | 59 % | 74 % | 144 | 20 % | 26 % | 11 |

Per seed (1 / 2 / 3), as 10.4. "n/r" = 50 % never reached. FT4 never reaches 50 % in any candidate.

| job | FT8 7074 SNR50 | FT8 7074 decoded | FT8 7041 SNR50 | FT8 7041 decoded | FT4 decoded |
|---|---|---|---|---|---|
| oracle | -20.7 / -20.9 / -20.7 | 399 / 402 / 400 | -20.7 / -20.7 / -21.0 | 398 / 400 / 406 | 714 / 707 / 705 |
| Bp_t0 | -15.6 / -15.8 / -16.1 | 243 / 242 / 246 | -16.1 / -16.1 / -15.7 | 244 / 246 / 255 | 122 / 120 / 120 |
| Bp_t1 | -15.1 / -16.2 / -15.9 | 225 / 234 / 231 | -15.8 / -16.2 / -15.8 | 234 / 237 / 236 | 118 / 116 / 116 |
| Bm_t0 | -15.6 / -15.9 / -16.3 | 246 / 246 / 246 | -16.1 / -16.1 / -16.8 | 247 / 252 / 261 | 162 / 157 / 164 |
| Bm_t1 | -15.4 / -15.9 / -16.3 | 243 / 245 / 251 | -16.4 / -16.2 / -16.8 | 254 / 249 / 260 | 151 / 167 / 158 |
| Cp_t0 | n/r / n/r / n/r | 0 / 0 / 0 | n/r / n/r / -4.1 | 127 / 132 / 131 | 48 / 42 / 47 |
| Cp_t1 | n/r / n/r / n/r | 0 / 0 / 0 | n/r / n/r / -4.2 | 130 / 125 / 138 | 44 / 40 / 47 |
| Cm_t0 | n/r / n/r / n/r | 19 / 18 / 30 | -11.0 / -14.3 / -10.2 | 151 / 157 / 161 | 84 / 64 / 65 |
| Cm_t1 | n/r / n/r / n/r | 19 / 13 / 27 | -11.0 / -14.2 / -9.5 | 151 / 155 / 159 | 74 / 69 / 66 |
| Dm_t0 | -12.9 / -12.8 / -11.7 | 199 / 196 / 195 | -15.8 / -15.8 / -16.4 | 240 / 245 / 247 | 119 / 128 / 118 |
| Dm_t1 | -12.9 / -12.9 / -11.5 | 197 / 193 / 191 | -15.8 / -15.6 / -16.3 | 240 / 241 / 244 | 125 / 120 / 116 |
| Dall_t0 | -12.4 / -9.8 / -10.1 | 183 / 173 / 170 | -16.2 / -15.1 / -16.2 | 240 / 234 / 238 | 112 / 107 / 106 |
| Dall_t1 | -12.3 / -9.2 / -10.2 | 182 / 175 / 170 | -15.5 / -14.9 / -16.3 | 238 / 231 / 237 | 114 / 114 / 107 |
| Em_t0 | -13.0 / -13.8 / -13.4 | 230 / 233 / 228 | -16.3 / -16.1 / -16.5 | 281 / 278 / 288 | 148 / 152 / 133 |
| Em_t1 | -13.2 / -13.7 / -13.0 | 225 / 227 / 222 | -16.3 / -16.2 / -16.6 | 276 / 270 / 280 | 156 / 154 / 143 |
| Eall_t0 | -12.7 / -11.2 / -11.4 | 214 / 205 / 209 | -16.3 / -15.4 / -16.5 | 278 / 270 / 273 | 142 / 135 / 128 |
| Eall_t1 | -12.6 / -11.8 / -12.1 | 212 / 206 / 203 | -16.2 / -15.5 / -16.5 | 273 / 264 / 268 | 155 / 141 / 137 |

### 12.4 Comparison with the log-normal results

Rule 2 (candidate closes at least half of the floor-to-oracle gap for FT8, does not make FT4 worse than the floor, adds
no false decodes):

- **B.** Bm_t1, the shipped setting: FT8 62 % of the oracle's count on the trace, 70 % on log-normal; FT4 22 % against
  28 %. SNR50 7074 / 7041 kHz: -15.4 / -15.9 / -16.3 and -16.4 / -16.2 / -16.8 dB; on log-normal -16.6 / -16.6 / -17.1 and
  -17.2 / -16.8 / -16.9. So B is 0.1-1.2 dB (mean 0.7) worse on the trace (4.1-5.3 dB behind the oracle, against 3.7-4.4),
  with a seed spread of 0.9 dB (7074) and 0.6 dB (7041). The shift is about the size of the spread, in the same direction in
  both channels and all three seeds; it cannot be attributed more precisely, because the loss is one realisation here
  (see 12.1). Rule 2 holds for B (FT8 > half). Bm_t1 has 5 false texts over the 9 channel-seeds
  (log-normal: 0), not looked at; the other B variants have 0-3.
- **C fails** as before: FT8 16-22 % (7074 kHz: 0 decodes for `Cp`, 13-30 for `Cm`), 7041 kHz partly (127-161), FT4 6-10 %.
- **D and E do not beat B.** At 7074 kHz B (-15.4..-16.3 dB) is 2-3 dB ahead of E (-13.0..-13.8) and D (-11.7..-12.9) in
  all three seeds (log-normal: seed 3 equal, seeds 1 and 2 3-4 dB). At 7041 kHz E and B have the same SNR50
  (E_t1 -16.3 / -16.2 / -16.6, Bm_t1 -16.4 / -16.2 / -16.8) while E decodes ~10 % more (276 / 270 / 280 vs 254 / 249 / 260,
  as on log-normal: 325 / 298 / 315 vs 309 / 300 / 286 of 507). E is nowhere 1 dB better than B, so rule 3 still favours B. E
  over D (the lost pauses filled) is +30 FT8 and +30 FT4 decodes (means, t1), as on log-normal (+24 / +40 FT8). The
  whole-file rate rows (`Dall`, `Eall`) are worse than the 30 s-window rows here (D 51 % vs 54-55 %), the opposite of the
  log-normal corpus (63 % vs 60 %); these are not client-reachable rates and were not looked into further.
- **Ranking** B > E >= D >> C for FT8 on the real trace; the same order as section 10.6. T0 vs T1 and the payload rate vs the
  30 s measure change B by less than the seed spread, as before (B payload 58-61 %, measured 62 %).

**F, the calibration window with a pause in it.** In section 10.5 the 107 s reset sat 2 s after the start of the window,
which is a weak position for a straight-line fit; here the reset is at 75.35 s, so `MEASURE_AT=70` puts it 0.35 s after
the start, and `MEASURE_AT=55` (window 60-90 s) puts it in the middle. A mid-window row for the log-normal corpus was
added (`MEASURE_AT=87`, window 92-122 s, reset at 107 s, 15 s in). Rates in S/s (seeds 1 / 2 / 3), FT8 7074 kHz decodes:

| corpus | window | job | measured rate | FT8 7074 decoded | FT4 decoded |
|---|---|---|---|---|---|
| trace | 5-35 s (no pause) | Bm_t0 | 986 864 / 986 903 / 986 875 | 246 / 246 / 246 | 162 / 157 / 164 |
| trace | 75-105 s, pause at start | Bm70_t0 | 980 704 / 980 705 / 980 772 | 242 / 242 / 241 | 120 / 121 / 117 |
| trace | same, robust (F) | Bm70r_t0 | 985 027 / 985 018 / 985 091 | 238 / 251 / 252 | 146 / 139 / 145 |
| trace | 60-90 s, pause mid-window | Bm55_t0 | 929 247 / 929 260 / 929 235 | 0 / 0 / 0 | 0 / 0 / 0 |
| trace | same, robust (F) | Bm55r_t0 | 998 939 / 998 804 / 998 820 | 158 / 157 / 158 | 104 / 101 / 99 |
| trace | 75-105 s, D | Dm70_t0 / Dm70r_t0 | as above | 179 / 180 / 180 and 182 / 168 / 177 | 72 / 78 / 78 and 110 / 103 / 96 |
| trace | 60-90 s, D | Dm55_t0 / Dm55r_t0 | as above | 0 / 0 / 0 and 103 / 100 / 101 | 0 and 85 / 82 / 88 |
| log-normal | 92-122 s, pause mid-window | Bm87_t0 | 913 754 / 912 383 / 916 626 | 0 / 0 / 0 | 0 / 0 / 0 |
| log-normal | same, robust (F) | Bm87r_t0 | 982 405 / 981 190 / 984 832 | 301 / 309 / 305 | 178 / 185 / 178 |

A pause in the middle of the 30 s window takes the plain `measure_rate` 5.8 % (trace) / 7.0 % (log-normal) low, which the
+-15 % gate accepts, and every decode is lost: this reproduces the section 6 "up to ~7 %" figure, which section 10.5 did
not reach because its pause was near the window edge. F recovers the log-normal case (301-309 against 296-308 pause-free);
on the trace it returns 158 against 246 because its rate is 1.2 % above the pause-free one. Read off the truth, that is the
window, not F: the hole loss in source seconds 60-90 is 2.38 % (rate 999.6 kS/s with the hole loss alone), while the
5-35 s window has 3.82 %, and over the 90 reset-free 30 s windows at 5 s steps the hole-loss fraction has mean 3.81 %,
sd 0.51 %, range 2.70-5.36 % (log-normal seed 1: mean 3.37 %, sd 0.32 %, range 2.79-4.23 %). So the real trace is
burstier, and a single 30 s window estimates the stream's rate to about +-0.5 % (one sd), against +-0.3 % for the assumed
log-normal. The cost of that was not separated from other things; one data point is B at +0.6 % (Bm vs Bp) which changes
nothing.

### 12.5 The section 11 FT4 diagnosis on the real trace

What the real trace looks like (seed-independent): 2 020 holes, mean 10.46 ms, longest 37.1 ms, like the assumed
statistics, but the **length distribution is not log-normal**: 999 holes are about 4.3 ms and 967 about 16 ms, 25 are 21 ms, 23
are 32 ms, 3 are 37 ms and 3 others (0.7 and 11 ms); lag-1 autocorrelation of the length is -0.56. The **spacing is burstier than Poisson**: mean 282 ms,
median 129 ms, coefficient of variation 1.42 (assumed: 1.0), longest 3.73 s, shortest 16.6 ms.

Section 11's per-transmission measure on Bm_t1 (steady state, FT4 >= -6 dB / FT8 >= -12 dB, no reset in the frame, 40 s from
the start or a reset, transmissions ending before 570 s, `tx_features.py` with `TXF_END=570`):

| | FT4 hit | FT4 mismatch std | FT8 7074 hit | FT8 mismatch std |
|---|---|---|---|---|
| log-normal (section 11) | 0.487 (n 672) | 51 ms | 0.980 (n 588) | 91 ms |
| real trace | 0.405 (n 624) | 57 ms | 0.868 (n 546) | 109 ms |

FT4 hit rate by mismatch octile, trace: 0.28 (-142..-53 ms), 0.44, 0.57, 0.56, 0.58 (-0.2..+20), 0.40, 0.32, 0.12 (+61..+212);
log-normal: 0.18, 0.60, 0.60, 0.59, 0.68, 0.71, 0.36, 0.14. Same shape: flat at 0.5-0.7 for |mismatch| below ~40 ms, falling
beyond. FT8 on the trace: 0.99-1.00 for mismatch -113..+65 ms, 0.71 below -113, 0.88 at +65..+118, 0.39 above +118 ms (log-normal:
0.89 in the last octile, nothing below 0.97 otherwise); the FT8 failures also sit at the largest mismatches, and FT8 now has
mismatches of that size because the trace is burstier.

Causal controls on the real trace (`trace_controls.py`: the same 2 020 holes, the same 3 resets and the same signals;
`delivered.bin` first 16 hex seeds 1 / 2 / 3: periodic `23bd071b9d8c74ec` / `04abc0d2cc65f772` / `b1e728e00a2d4db5`, reallen
`a03c5a547dfc40d4` / `c59c4c31c3fc8ca9` / `9c9a59e252b0ff6e`, realspace `f7ec0ba043f7000a` / `dba64562dd60d4fd` /
`1f9a1ad04508ef67`), Bm_t1, first 570 s:

| loss | holes at | lengths | FT4 mismatch std | steady strong-SNR FT4 hit | FT4 decoded (1/2/3 of 825) | FT4 SNR50 dB | FT8 7074 decoded (of 481) |
|---|---|---|---:|---:|---|---|---|
| oracle (reference, trace loss) | - | - | - | - | 714 / 707 / 705 | -18.2 / -17.8 / -17.9 | 399 / 402 / 400 |
| periodic | equally spaced | all the mean 10.46 ms | 3.4 ms | 1.00 | 516 / 525 / 535 | -15.5 / -16.0 / -16.3 | 322 / 327 / 330 |
| reallen | equally spaced | the real lengths, in order | 12.3 ms | 0.99 | 487 / 485 / 491 | -14.8 / -14.6 / -14.8 | 313 / 318 / 316 |
| realspace | the real positions | all the mean | 55.1 ms | 0.41 | 169 / 168 / 158 | n/r | 250 / 247 / 255 |
| **real trace** | real | real | 57.3 ms | 0.405 | 151 / 167 / 158 | n/r | 243 / 245 / 251 |

With the same hole count, total loss and resets, equal spacing alone takes FT4 from 151-167 to 487-535 decodes (steady strong-
SNR hit 0.41 to 0.99-1.00); the real lengths on equal spacing cost little (about 7 % fewer than constant length); the real
positions with constant lengths are indistinguishable from the real trace. This is section 11's result (there:
spacing 0.98 to 0.55, lengths 0.98 to 0.87), with spacing the dominant factor and lengths weaker here. FT4 with the
real trace is not better or worse than with the assumed one in the way that matters: 22 % of the oracle against 28 %,
mismatch std 57 against 51 ms. FT8 7074 kHz recovers from 245 to 313-330 decodes on the equal-spacing controls.

E on the controls (Em_t1): FT4 517 / 513 / 519 (periodic) and 481 / 473 / 474 (reallen), i.e. as good as B; on realspace
142 / 133 / 135, somewhat below B (169 / 168 / 158). It loses to B in FT8 7074 kHz on the regular controls (periodic 299 / 296 / 296 against 322 /
327 / 330, SNR50 -15.4..-16.5 against -18.5..-19.1) and its reported FT4 SNR is about 1.5 dB lower than B's. Not looked into.

Resets: the FT8 hit rate at >= -12 dB is 63/63 (7074 kHz; 62/63 at 7041) in 5-20 s after a reset, **0/63 in 20-40 s**, and
447/504 (89 %) after 40 s; on the equal-spacing control it is 63/63, 0/63, 504/504. This is the unexplained FT8 0/63 window
of section 11.3 (f), present on the real trace too; not chased. FT4 on the trace: 0/14, 1/70, 25/96, 247/576 for 0-5, 5-20,
20-40, > 40 s; on the control 1/14, 1/70, 41/96, 576/576.

### 12.6 Reading

- The ranking of section 10 holds on the real trace: B is the best FT8 setting of B-E (clearly at 7074 kHz; at 7041 kHz equal in SNR50
  to E, which decodes ~10 % more), C fails, D and E are not 1 dB better than B anywhere. Its numbers are somewhat
  worse than on the assumed corpus (FT8 62 % against 70 % of the oracle, 4.1-5.3 against 3.7-4.4 dB behind it).
- The section 11 FT4 diagnosis holds: with a global stretch rate and the real hole positions FT4 decodes 22 % of the oracle's
  count; with the same holes equally spaced it decodes 69-75 %. What breaks FT4 is the irregular arrangement of the hole
  positions, not the number or total of the holes or the resets. "Lengths matter little, spacing matters most" is what these
  controls say here; section 11 had both.
- The real trace is not the assumed one: bimodal lengths (about 4.3 / 16 ms), burstier spacing (cv 1.42), a 30 s window that
  estimates the long-run rate to about +-0.5 %. A candidate that must work on the RSP1 should be judged on the trace, not
  on the log-normal corpus; neither changes the order of B-E.

### 12.7 Not verified / not done

- One loss realisation. The three seeds share the loss, so "spread between seeds" understates the loss-to-loss variation;
  the 0.7-1.3 dB difference to the log-normal results cannot be separated from that. A second real log would settle it; this
  is the only dbg4 log that was at hand.
- The log's identity is argued from numbers (12.1). Reset lengths are estimated from the wall clock (the converter says so).
  Whether the corpus's fixed 20 ms latency, 5 ms jitter and 50 ms callbacks match the RSP1 was not tested.
- The "global stretch rate" half of the diagnosis is shown only indirectly: D/E (zero insertion at an even rate, also a
  global rate) are no better for FT4 than B; no local-rate or per-hole correction was tried (a candidate, out of scope).
  The within-frame shape of the time error (not only its integral) still matters: at |mismatch| < 25 ms the real trace's
  FT4 hit is 0.54 against 1.00 on the periodic control (section 11.5 saw the same).
- Dall/Eall are not the same setting as section 10's to within 0.003 % in rate (12.1). Only three of the F positions were
  tried. The A decodes (12.2), E's SNR offset on the controls and the oracle's 4 false decodes are not examined.
- The FT8 0/63 window after a reset, the FT4 -1.75 dB offset on clean input and the 5 Bm_t1 false decodes were not chased.
- Step 6 (real-recording oracle) is deferred, decided 2026-10-17: the 600 s recording is gone (it lived in a cloud session's `/tmp`; the NAS `pc-desktop` holds only `rsp_clock.log` and the rsp_tcp builds, no IQ file). A re-record needs the RSP1 with the dbg4 build, recording and `rsp_clock.log` from the same run. The ranking therefore rests on synthetic signals (log-normal and real hole trace).
- Not done: step 7 (implementation), the RSP1. The corpora, rows and score tables are
  in `/home/minoru/tmp` (not kept in the repository); the commands above regenerate them.
