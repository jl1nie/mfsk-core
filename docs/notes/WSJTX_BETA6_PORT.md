# WSJT-X v3.3.0-beta4 → beta6: what was ported and what it measured (#726)

The audit of WSJT-X `release/3.3` from v3.3.0-beta4 to v3.3.0-beta6, and the changes it led to, released in 0.16.0. The
A/B/C classification of every changed upstream file is in Issue #726; this note keeps the measurements the CHANGELOG
entries summarise. Upstream line numbers are v3.3.0-beta6's unless stated. The reference `jt9` is beta6's, built by
`scripts/build_jt9_upstream.sh ../WSJT-X v3.3.0-beta6`.

## Transmit

### FT4: `gen_ft4wave` (#729)

Before: synthesised like `gen_ft8wave`, 103 symbols, an `nsps / 8` ramp, the phase starting at the first tone. WSJT-X frames
the 103 tones with a ramp symbol each side (105 symbols, 60 480 samples at 12 kHz) and ramps a whole symbol. On a fixed tone
list against WSJT-X's own generator, tones aligned: -21 dB over the frame before (-45 dB inside it, edges excluded); -103 dB
against beta4 and -82 dB against beta6 after (`tests/gen_wave_vs_wsjtx.rs`, fixtures from `scripts/wave/build_fixtures.sh`).

### FST4: `gen_fst4wave` (#736)

The first and last symbol were extended and ramped `nsps / 8`, as FT8's are; upstream does not extend them and ramps half a
symbol (`Waveform::Fst4`). FT8, FT4 and FST4 now keep phase and carrier in a `u64` turn accumulator (`engine/dsp/nco.rs`):
beta6 sums in `f64`, which the CoreS3's LX7 cannot afford per sample, and an `f32` sum drifted (FST4-60: -42 dB over 51 s).

### WSPR, JT9, JT65, Q65: `Modulator::modulate` (#737)

The loop took `cos` of an `f32` phase before advancing it, with `f0 + tone * spacing` in `f32`; WSJT-X advances a `double`
phase first and takes `sin`. Against that on a 110 s WSPR frame: -35 dB (an `f32` sum drifts 0.04 rad), and a different
phase. Now the same `u64` turn accumulator, with the per-symbol frequency formed in `f64` from the `f32` `f0` and spacing
(JT9's `12000/6912` is not exact in `f32`: 1e-4 rad over a frame): within 2e-4 of an `f64` reference of the Modulator's
loop (`cpfsk::tests`). `engine::dsp::subtract` rebuilds the same waveform. The JT65 averaging test tries up to 8 noise
draws, because the window it relied on moved.

The GUI's Q65 transmission is the Modulator's, not `genwave.f90`'s: `mainwindow.cpp:13518` starts Q65 with a positive
`toneSpacing`, and `Modulator` plays a precomputed waveform only for `toneSpacing < 0`.

### All tone modes, against WSJT-X's generators

On a fixed tone list (tier A, `tests/gen_wave_vs_wsjtx.rs`, beta6): FT8 -68 → -95 dB, FT4 -21 → -95 dB, FST4-15 -16 →
-88 dB (over the whole frame: the extra pulse tail moves the phase throughout), JTTY -99 dB (already, pinned, unchanged);
maximum sample error 3e-5 to 1.1e-4. The full-slot subtract pin `embedded-poc/assets/ft8_subtract_full_slot.bin` was
re-recorded on x86_64 (its input moved by up to 2 LSB); not re-measured on aarch64.

### Polynomial pulse and sine (#738)

The 3-symbol Gaussian pulse (`bt` 1 and 2) uses beta6's degree-7 tail polynomials (`waveform_kernels.f90`), built half
and mirrored; the NCO's `sin` is beta6's degree-9 polynomial on a folded quadrant. Against beta6's generators: FT8 -94 dB,
FT4 -96 dB, FST4-15 -88 dB, within about 1 dB of the `erf`/`sinf` form (FT8 1.1 dB worse, FT4 0.7 dB better); the pulse
within 3e-6 of the `erf` form. The envelope ramps still call libm `cos`. FT8 `synthesize` on an x86_64 host: 704 → 535 µs
(-24 %, with a fast libm). Not measured on the LX7 (the CoreS3 `tx-synth bench` times this path); the CoreS3's live
transmitter (`GfskStream`) gets only the polynomial pulse table and keeps its own form otherwise.

### Where the buffer goes: `tx_buffer_start` (#743)

FT4's buffer starts a ramp symbol (576 samples) before its tones. One value says where every mode's buffer goes so that
the frame lands at DT = 0: `TX_START_OFFSET_S` less that lead (FT4 5424, FT8 6000, the 1 s modes 12 000 samples at
12 kHz; samples, because 0.452 s has no exact `f32`). The every-mode encoder round trip asserts the decoded DT is within
`max(0.03 s, symbol / 16)` of 0; with FT4 placed by `tx_start_offset_s` it reads +0.0475 s and fails. WSJT-X's own
transmitter is not at DT = 0 for FT4: `Modulator::start` plays the 105-symbol buffer from `delay_ms = 300`
(`Modulator.cpp:167`), so the tones leave the PC at 0.348 s; FT8 (`delay_ms = 500`) and the 1 s modes agree with
`tx_buffer_start`.

## Receive

### FT4 subtraction reference (#739, #734)

It was `gen_ft8wave`'s form (103 symbols, an `nsps / 8` ramp), which left the two ramp symbols of a transmitted signal
behind: -20.9 dB through the constant-amplitude LS subtraction and about -20 dB through the decoder's LPF one; now nothing,
and -29 dB. `subtractft4.f90` builds `cref` with `gen_ft4wave` over `(103+2)*NSPS` from `nstart=dt*12000+1-NSPS`. The six
`ft4_*` and two `iq_*_ft4` snapshot fixtures moved; no message appeared, and three marginal ones left `ft4_eq_local`
(`AC6BW KR9A R 559 WI`, `CQ RU W0FRC DM79`, `W7BOB KJ7G RR73`: all in `jt9`'s list for the recording, all found only in
SIC round 1 at 17-18 hard errors, all still decoded by `ft4_default` and four other configurations). Now guarded by
`ft4_wsjtx_sample_eq_local_recall_floor` (12 of `jt9`'s 14).

### Q65: the clip base floored at int16's range (#740, #735; marked divergence)

`condition_symbol_spectra` clips the symbol energies at 20 × their 40th percentile (`q65_loops.f90`'s `s3lim`). On float
input with no noise that percentile is f32 rounding, 160 dB below the peak, so the clip saturated the tones' sidelobes too
(30-67 of 192 bins a symbol tied) and BP failed: an aligned noiseless Q65-15A frame with its start cut was lost with noise
at 1e-9 and decoded at 1e-7. WSJT-X cannot meet it (`int16` audio has 128 dB of range at Q65-15; `jt9` decodes the `int16`
form of the slot). `base` is floored at `peak / (3 * 32767^2 * nsps)`. The floor never engaged on the vendored recordings
(margin >= 114 dB) or the noisy synthetic tests; it can engage on a noiseless `int16` file (a tone's quantisation error
is not white, about -12 dBFS and louder), with the same decode. 4 of 12 new inputs fail without it.

### JTTY: misaligned views of a decoded frame (#728; upstream PR #702, `is_frame_alias`)

A strong signal left repeatable false frames (`IG4QXF TU`): a sync candidate a whole number of symbols inside a decoded
frame matched 7 of 13 sync tones by chance and still gave a TBCC codeword. It is dropped when, within 2.5 tone spacings and
at a near-whole-symbol offset, >= 80 % of >= 9 overlapping received tones equal that frame's, shifted alike. On upstream's
own case (`tests/jtty_alias.rs`, fox at -3 dB, 24 seeded trials): 4 false frames before, 0 after, 24/24 decoded in full,
and a second station 30 Hz away at 18.5 symbols still decodes.

### WSPR: `unpackpfx` (#727; upstream `ffa884606`)

`unpackpfx` kept `nc` in a `char`, so a type-2 suffix field past 60255 wrapped into a valid suffix; the port reproduced
that. Type 63 is reserved and refused. The 26 046-row `unpk_` fixture, rebuilt from beta6 (`wsprd_unpack_beta6.tsv`,
`scripts/wsprd/`): 1 488 rows change, none left unmatched.

### JT9 and JT65: DT from 1.0 s (#742)

Both had `TX_START_OFFSET_S = 0.0`. WSJT-X starts their audio 1.0 s into the minute (`Modulator.cpp:163` `delay_ms=1000`,
`jt9sim.f90:124` `k=12000`, `jt65sim.f90:217`) and reports DT from there (`jt65_decode.f90:425` `dtx-1.0`). So `Decoder`
reported every DT 1.0 s above `jt9`'s (JT65 golden: +1.02 against 0.0; JT9 sample: +0.90 to +1.04 against 0.0 to 0.1),
its search window sat 1 s early, and a host placing JT9/JT65 TX by `tx_start_offset_s` keyed up 1 s early. Found while
reviewing the TX placement design. Same messages after; the re-centred window picks other lags (JT65 golden SNR
-18.9 → -18.3 dB on four carriers, -18.6 → -20.1 dB on 1500 Hz; `jt9` -19). `tests/jt_dt_vs_jt9.rs` compares with `jt9`. The
JT9 golden test had absorbed the offset as a "0.91 s pre-roll" in the WAV; it was this reference.

### JT9: `jt9`'s candidate stage (#742)

The scan was this crate's own spectrogram search, tried in score order and cut to `max(max_candidates, 32)`. `jt9` scores
0.732 Hz bins with `sync9` over `decode_pcm`/`prepare_spectra`'s half-symbol spectra and walks them upwards, decoding at
the first bin that passes `ccflim` (3.0 / 2.7 / 2.5 by depth, the Rx window always 2.5) and `red2lim`, then marking
`i-1..=i+22` done (`jt9_decode.f90:59-284`). `peakdt9`'s DT depends on that bin: on `130418_1742.wav` the old scan
decoded `K1JT N5KDV EM41` at its peak bin (-0.10) where `jt9` uses the 1344.0 Hz bin (0.1), and `G7CNF N4HFA EL89` at 0.0
where `jt9` uses 1459.0 Hz (-0.2). The old 32-candidate cut is also why `CQ M0WAY IO82` was missed with default extras.

Ported as `jt9::sync9`. Against beta6's `jt9` on that file, at `-d 1` and `-d 3`, over 1050-1550 and 200-4000 Hz, with and
without an Rx window at 1119 Hz: the same rows (`max_extra: 0`), DT within `jt9`'s 0.1 s print, SNR within 0.5 dB
(`tests/jt9_wsjtx_samples.rs`). `CQ GM7GAX IO75` needs the Rx window over 1050-1550 Hz, in `jt9` too; the golden table had
it from the WSJT-X GUI, whose Rx frequency was there. v3.2.0-rc1 prints two numbers differently (`K1JT N5KDV EM41` -1 dB,
and over 200-4000 Hz at `-d 3` `G7CNF N4HFA EL89` at DT 0.0); beta1 and beta6 agree with the port. A short buffer is
scored over the half-symbols it has (`decode_pcm`: `min(181, (count-2048)/3456)`), not a fixed 181.

The crate's own `peakdt9` score gate (1.5, calibrated on this one file) is removed: with `sync9`'s gates in front it changed
no decode at any depth. A noise-only phantom test was written and deleted, because it could not fail: with every gate off,
Fano converged on nothing in six 60 s noise slots. `Jt9Extras::search` now supplies only the band.

Tier C: the JT9 sweep used `DecodeRequest`'s defaults (`Fast`, 200-4000 Hz), which matched `jt9 -d 3` only because the old
scan had no `ccflim`. With `jt9`'s gates `Fast` is `-d 1`, and read -26.30 dB (baseline -26.64). The sweep now decodes as its
upstream side runs (`-d 3`, 200-3000 Hz, `scripts/upstream_tasks.json`): -26.58 dB; against `jt9`, both 243 of 300, `jt9`
only 3, the crate only 0 (p 0.25), 0.14 dB behind at the crossing, a quarter of `jt9`'s time per file.

## Tier C before 0.16.0

Run on the release candidate (`feat/tx-buffer-start` at 188d8a22, 2026-11-14), against the 2026-10-31 baseline: no group
moved beyond threshold. FT8 ccir_good -0.38 dB and ccir_poor -0.32 dB (both better), awgn and ccir_moderate within 0.01; FT4,
FST4 (20 groups), Q65 (20), JTTY and JT65 +0.00; JT9 as above. Unexpected decodes: +1 in FT8 ccir_moderate (260 trials,
under the flag), every other group as before. Against upstream no group is behind; Q65's B-E sub-modes reach 0.5-2.3 dB
lower than `jt9` at the crossing.

## Not done

- LX7 timing of the polynomial pulse (#738), and the CoreS3's streaming `GfskStream`, which keeps its own form.
- The re-recorded subtract pin on aarch64.
- `ft4_eq_local`'s three marginal messages.
- Q65's `p40 > 30*p10` guard, which the floor may make removable.
- The JT9 sweep CSV has no unexpected-decode column.
