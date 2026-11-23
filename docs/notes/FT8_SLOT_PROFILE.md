# FT8 slot profile: how to measure it on a machine, and what was measured

Where the time of one FT8 decode goes, and the three commands that show it on any machine (a Raspberry Pi, a Mac, a PC). Written for
[#761](https://github.com/jl1nie/mfsk-core/issues/761) (FT8 on a Raspberry Pi 4 and 3); the subtract changes it led to are #761 and
[#762](https://github.com/jl1nie/mfsk-core/issues/762). **Every figure below is from one x86 machine. There is no Raspberry Pi measurement in this
repository yet.**

## 1. The three commands

All three read a 12 kHz mono recording; `embedded-poc/assets/qso3_busy.wav` (16 stations, one slot) is the one the figures use.

```sh
# 1. A whole-period call per depth (the first is cold), and the period fed in at the decoder's checkpoints.
cargo run --release -p mfsk-serve -- bench embedded-poc/assets/qso3_busy.wav --iters 8

# 2. The same, with each call's time split by stage, one core. Each TRACE_STAGE_FT8_ALL line (stderr) is the call
#    whose result line follows it on stdout.
MFSK_TRACE_STAGE_FT8=1 RAYON_NUM_THREADS=1 \
  cargo run --release -p mfsk-serve -- bench embedded-poc/assets/qso3_busy.wav --iters 8

# 3. What one `subtract` call is made of: the FFT pair, the reference waveform, the rest; and `refine_dt`.
cargo run --release -p mfsk-core --example subtract_profile -- embedded-poc/assets/qso3_busy.wav
```

Reading (2): `wall` is the call, `attributed` the time put to stages, and each stage is `time(count)`, largest first. A stage's time is
**exclusive** (OSD's own LLR and BP count as LLR and BP, not as OSD) and **summed over threads**, so only with `RAYON_NUM_THREADS=1` does it
add up to the wall time. The stages: `spectrogram`, `coarse_sync`, `fft_cache`, `downsample`, `fine_refine`, `symbol_spectra`, `llr_*`, `bp`,
`osd`, `subtract`, `baseline`, `xsnr`. What is in no stage (candidate triage, sorting, allocation) is `wall - attributed`; on the figures below
it is 0-8 %. The switch costs one atomic load per stage when unset (`ft8/stage_timing.rs`).

## 2. What was measured (x86, one thread, warm)

Ryzen 9 9900X, WSL2, rustc 1.100.0, `qso3_busy.wav`. "Before" is `main` at `a3310def`, before the subtract work; "then" is `6c4e6d0e` (after it);
"now" is `fad24caa` (the code of `886bcf29`: BP, real-input FFTs and `refine_dt`'s parallel trials added).

| depth (strategy) | whole call before | then | now | `subtract` before | then | now |
|---|---:|---:|---:|---:|---:|---:|
| `Fast` (`SicRounds(2)`, OSD off) | 79 ms | 61.5 ms | 58.9 ms | 35.6 ms | 18.8 ms | 25.8 ms |
| `Normal` (`SicEarly`) | 293 ms | 204.9 ms | 183.5 ms | 166 ms | 78.7 ms | 84.3 ms |
| `Deep` (`SicEarly`) | 594 ms | 502.6 ms | 392.0 ms | 198 ms | 96.9 ms | 95.7 ms |

Now, the stages after `subtract`: `Fast` fine refine 6.5, BP 6.3, coarse sync 5.0, spectrogram 2.5 ms; `Normal` OSD 21.6, coarse sync 15.4, fine
refine 13.8, BP 13.3, spectrogram 7.6 ms; `Deep` OSD 94.4, fine refine 48.5, BP 46.6, coarse sync 15.0 ms. BP fell from 15.1 / 32.6 / 121.5 ms (the
`ffcef959` table and `tanh` work). **OSD is what is left, with `subtract`.** OSD is GF(2) bit operations and BP a gather-heavy message passing, so a wider
SIMD is not expected to help either; that has not been tried.

**Explained, and fixed in `subtract` (not elsewhere):** `Fast`'s `subtract` was 25.8 ms in the trace (17 calls, 1.5 ms each) where it was 18.8 ms at
`6c4e6d0e`, while one `subtract` call timed alone is 1.05 ms. It is glibc's `malloc`: `subtract` allocated its two largest buffers on every call (`cfilt`,
1.44 MB, and rustfft's scratch, about as large), and an allocation above the `mmap` threshold takes fresh pages and page-faults on all of them. The
threshold rises by itself when a larger `mmap`ed block is freed; the complex-FFT downsample cache's 1.5 MB temporaries did that until `405f6e08` made
them 768 KB real-input ones, after which `cfilt` was above it. Bisected to that commit and, in it, to `downsample.rs`; with `MALLOC_MMAP_THRESHOLD_` swept
the step falls between 1.0 and 1.5 MB, the size of `cfilt`. Both buffers are kept per thread now (`with_work` in `engine/dsp/subtract.rs`), so the decode
no longer depends on the threshold: `Fast` 59.8 -> 51.0 ms with `subtract` 26.2 -> 18.2 ms, the same as with the threshold fixed at 1 GB; whole calls at
one thread now `Fast` 50.5, `Normal` 174.3, `Deep` 383.6 ms (four threads 51.0 / 155.7 / 361.5). **Other large buffers are not fixed**: the
reference waveform and `dphi` (about 0.6 MB each), `refine_dt`'s three 360 KB trial copies, and the spectrogram's arrays are above the default 128 KB
threshold. With the threshold set to 256 KB `Fast` is 58.0 ms against 50.8, so a process that has not yet freed anything large (a process per slot)
can still pay for them. They are to be fixed when they are found to matter, as these were. Linux/glibc only; not measured on Apple or on a Pi.

At the checkpoints of `decode_prefix` (`Normal`: 70 / 40 / 78 ms, `Deep`: 155 / 50 / 192 ms) the middle one only subtracts (`subtract` is the
whole of it). The checkpoints move work earlier; they do not remove it. What is left after the period ends is the last figure.

Threads, whole call at `RAYON_NUM_THREADS=1` and `4` (mean of 2 runs of 5 warm calls, `mfsk-serve bench`): `Fast` 58.9 -> 59.3 ms (unchanged), `Normal`
185.5 -> 166.3 ms, `Deep` 385.3 -> 367.1 ms. Only `refine_dt`'s three trials run in parallel (`da8950e0`); the SIC strategies otherwise subtract
candidate after candidate as `ft8b.f90` does, on one core. (`DecodeRequest::single_pass()` does use the others: 56 ms to 20 ms, 4 threads.) `mfsk-serve`
takes `--threads N` (or `MFSK_SERVE_THREADS=N`) to cap the workers, which leaves the other cores to whatever else runs on the machine.

## 3. What the first profile found, in `subtract` (#761, #762)

One `subtract` call was 2.41 ms (FFT pair 0.48, reference waveform 0.69, loops 0.35, `end_correction` 0.69). Widening the SIMD (AVX2 / AVX-512)
moved it 2 %. In order of what was done, all on `main`:

- `end_correction(lpf_half)` was rebuilt on every call, about two million dependent `f32` adds; it is built once, as `subtractft8.f90` does. Bit-identical. 2.41 -> 1.35 ms.
- `refine_dt` built the reference four times for its four applications; once now. Bit-identical. One `refine_dt` call 8.9 -> 6.55 ms.
- The reference's phase-rate array was built with a bound test on every element that was always true. Bit-identical. 1.35 -> 1.25 ms.
- The reference's `cos`/`sin` come from an `f64`-anchored phasor recurrence, not one libm call per sample (**a divergence from `gen_ft8wave.f90`**, #762). 1.25 -> 1.04 ms, and 44 times closer to the exact waveform.

Then, from the Apple arm64 work on the same issue (each measured by its commit's author on an Apple M-series core and **not repeated here**; the
x86 effect of the set is the table above): BP's index tables and a rational `tanh` (`ffcef959`, **a divergence from the WSJT-X BPs**, absolute error
4.95e-7, output clamped to [-1, 1] in `886bcf29`); real-input FFTs for the spectrogram, the downsample cache and the `refine_dt` score (`405f6e08`);
`refine_dt`'s three trials in parallel (`da8950e0`); and on aarch64 only, `subtract`'s FFT zero-padded to a length rustfft's NEON path is fast at
(`1f428a8b`, **a divergence from `subtractft8.f90`'s `NFFT = NMAX`**; the FFT pair was 97 % of a warm `subtract` there, 1.2-1.5 ms at 180 000 points against
0.47 ms at 196 608; other targets are bit-identical). The tier C sweeps over all of it moved no group beyond the threshold (2026-11-23, baseline re-stamped).

Not measured on a Raspberry Pi, or any Cortex-A core. A Cortex-A72's `sinf`/`cosf` and floating-point add latency are longer than a Zen 5's, so the
shares are probably larger there. That is a guess; command 3 is how to find out.

## 4. What to send back from a board

The output of the three commands, `uname -m` (it should say `aarch64`: on a 32-bit OS rustc 1.100.0 offers NEON only as an unstable target feature), the CPU (`grep -m1 'model name\|Model'
/proc/cpuinfo`), `rustc --version`, and what else was running (`top`). A reply is due when the transmit audio starts, 0.5 s after the period
ends; with `x` the factor by which one core of the board is slower than the one above, the last checkpoint fits when `x` is at most about 6 for `Normal`
(78 ms) and 2.6 for `Deep` (192 ms), and 8 for `Fast` (58.9 ms, all of it after the boundary). The factor is what the board's figures give; it is
not assumed here.
