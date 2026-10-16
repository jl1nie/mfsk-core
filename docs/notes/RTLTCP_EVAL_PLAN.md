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
