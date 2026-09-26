# JTTY oracle fixtures (WSJT-X 3.2.0-rc1)

Everything here comes from WSJT-X's own JTTY programs at tag `v3.2.0-rc1`
(`567ad29ce6abf3d4a44f181cdbc7ceba0d73e5f4`), built from a clean export of the
tag by `scripts/build_jttysim.sh`, and is regenerated, byte for byte, by
`scripts/gen_jtty_vectors.sh`. Tracking issue #477; plan in
`docs/notes/JTTY_UPSTREAM.md`.

| file | what it is |
|---|---|
| `260807_134110.wav` | upstream's own sample recording (`samples/JTTY/`), 12 kHz mono, 30.24 s. The only real recording upstream ships. |
| `260807_134110.expected.txt` | what `rjtty 4.6 1 384 1500 50` decodes from it (the WSJT-X defaults: `smin` `ndebug` `nsps` `f0` `ftol`), with its sha256. |
| `sim/*.wav` | `sjtty` recordings, `f0` 1500 Hz, `dt` 0.3 s, deterministic. A message per atom kind / frame count: AWGN at −8 dB SNR (2500 Hz bandwidth), plus two `clean_*` files at SNR > 90, where `sjtty` writes the noiseless waveform (peak-normalised) — the synthesiser oracle. |
| `sim/MANIFEST.tsv` | per vector: message, exchange profile, the **tone sequence** `sjtty` prints (the encoder oracle), and what `rjtty` decodes from the WAV. Vectors with `-` for the WAV are tones only. |
| `ladder_cases.txt` | inputs and outputs of the TBCC decode ladder, produced by our own driver (`scripts/jttysim/jtty_ladder_oracle.f90`, linked against upstream's `jtty_tbcc_*` modules). See below. |

## The real recording

`rjtty` decodes `RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!` at
1506–1508 Hz, `dt` 0.30 s, over 10 frames — the text the user guide's screenshot
shows (`doc/user_guide/en/images/jtty.png`, from a `v3.2.0-devel` build).

**It also decodes one thing the screenshot does not show:** a final line
`4>-P'` at 1695.6 Hz, channel 2 (the 1650 ± 150 Hz search), `nsync` 9, and 27
symbol errors out of 59. A CRC-valid frame with 27 wrong symbols is far more
consistent with a false accept than with a weak real signal, on a recording whose
message is literally "no false decodes". Not investigated: whether the GUI
filters it, and whether it is the `rc1` receiver or the `rjtty` front end.
This crate's receiver is the same algorithm and makes the same false decode
(`tests/jtty_rx.rs`): the golden test therefore has `max_extra: 1`, pinned to
exactly that frame, as documented debt rather than the target 0. A second one
exists: `rjtty` (and this crate) decode `KD7NCT` in the vendored WSPR recording
`golden/wspr/150426_0918.wav`, which has no JTTY in it. Why neither is simply
gated away: `docs/notes/JTTY_UPSTREAM.md`, "P2 results".

## The simulated vectors

`sjtty` seeds its own noise: the same arguments give the same bytes (checked),
and successive files in one run differ. `rjtty` is deterministic too (checked
twice). Like `jt65/jt65a_5sig_m18.wav`, these are simulator output, not
off-air recordings; unlike it they are noisy on purpose — a noiseless synthetic
fixture is not an instrument for anything but the encoder.

The tone sequences in `MANIFEST.tsv` are what `genjtty` produces: 13 sync
symbols (`0 2 2 3 0 0 3 2 1 3 1 2 0`) then 46 data symbols per frame.

## The ladder cases

Each case is a random 34-bit payload, encoded with upstream's `tbcc_encode`,
turned into the 4 × 46 complex correlations the receiver would hand to the
decoder (`ZS`, the full-symbol values) and the half-symbol energies (`ZH`), then
decoded by upstream's `jtty_tbcc_decode` (`DEC`) and by its list decoder at each
rung (`RUNG L1` / `L2` / `L4` on `ZS`, `H1` on `ZH`, up to four words each with
CRC flag, clean metric, WAVA metric and start state). The generative model is
documented at the top of the driver. Floats are printed with 9 significant
digits, which round-trips a float32 exactly.

The 33 cases cover every outcome of the ladder — accepted at L=1, L=2, L=4, by
the half-symbol rung, and total failure (`gen_jtty_vectors.sh` refuses to write
the file if one is missing). A Rust port is tested at exactly this boundary:
same `ZS`/`ZH` in, same words in the same order out, metrics equal to f64
tolerance. That isolates the trellis from the DSP in front of it.

The AWGN/fading sweep corpus is separate (`embedded-poc/assets/jtty_sweep/`,
`scripts/gen_jtty_sweep_wavs.sh`); it is gitignored, but the score `rjtty`
gets on it, `UPSTREAM_RECALL.tsv`, is committed.

The noise-only corpus (`embedded-poc/assets/jtty_noise/`, gitignored,
`scripts/gen_jtty_noise_wavs.sh`, 60 × 30 s at SNR −99 dB) is for the "Gaussian
noise decodes nothing" test; `rjtty` decodes nothing in it either.
