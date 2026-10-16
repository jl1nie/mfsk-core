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
- **D and E do not beat B in FT8.** E with the 30 s measure (Em_t1) is 2-4 dB worse than B at 7074 kHz in
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
window rather than the 7 % worst case quoted there (that case is a pause near the window's end on a shorter fit;
not reproduced). Only the window position was varied; a window with no pause in it (the default here) is
unaffected by F.

### 10.6 Ranking (rule 2, rule 3)

1. **B** (pin, stretch by the measured rate) passes rule 2 and is the simplest. C fails (channel
   displacement). D and E pass the FT8 half-gap test (60-73 %) but are not 1 dB better than B anywhere, so by
   rule 3 B wins on code size. F (pause-robust `measure_rate`) is not a rival to B: it removes a failure of
   the calibration that B (and D) depend on, and is worth doing whichever is chosen.
2. The gap that remains is the oracle's: B is 3.7-4.4 dB (FT8) behind it and decodes 44 % of FT4 at strong SNR.
   Closing that is not a matter of the candidates listed here.
3. These are results on a *simulated* hole shape (log-normal) and zero-delay-variation callbacks; the trace run
   (section 9 step 4) is the check on the assumption and was **not possible**, see below.

### 10.7 Not done

- **Step 4 (trace): skipped.** The 2026-10-15 dbg4 `rsp_clock.log` is not on this host (searched `/`, `/mnt/c/Users/minor`,
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
