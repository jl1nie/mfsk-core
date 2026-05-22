# Phase D — ESP32-S3 LX7 PIE SIMD acceleration

Plan for hand-rolling (and where free, just enabling) PIE 128-bit
vector kernels on the production S3 decode path. Drafted 2026-05-22.

Lives outside the `Phase B / Phase C` controller-line plan in
`docs/ROADMAP.md` because it is **pure decoder-side perf** and
benefits every S3 consumer (`m5stack-s3`, `m5stack-s3-app`, the
planned `m5stack-cores3-app`, and any `mfsk-ffi-ft8` user on S3).

## Why now

`embedded-poc/m5stack-s3/logs/s3_phaseC_q3i8_2026-05-04.log` —
representative post-Phase-C slot (qso3 reference WAV, dual-core,
`opt-level=1`, octal PSRAM):

| Stage | Wall (us) | Notes |
|---|---|---|
| stage1 inc FFT × 184 (during cap) | 59 800 | `dsps_fft2r_fc32_ae32_` |
| coarse_sync allsum + score (during cap) | 137 000 – 170 000 | f32 scalar Rust |
| stage 2 sync resolve (post-cap) | 188 000 | scalar |
| **pass 2 (Goertzel × ~30 cand, dual-core)** | **121 000 – 122 000** | **f32 scalar Rust** |
| stage 3 (BP/LDPC) | 100 000 – 625 000 | scalar; varies with `n_results` |
| post-SlotEnd total | 480 000 – 1 002 000 | target ≤ 300 000 (so the operator sees decodes well before the next 15 s slot boundary) |

The Phase-C streaming overlap (PR #123) parked stage1_inc +
coarse_sync inside capture, so the **critical path is pass 2 + stage
3**. Pass 2 is the largest fixed cost (Goertzel for every refined
candidate × 79 symbols × NSPS=1920 samples) and is **pure scalar
f32** today — initially the most attractive PIE target, but the
"D2 reconsidered" section below explains why this plan ends up
keeping that loop scalar.

## The discrepancy this plan also fixes

`embedded-poc/CLAUDE.md` and `docs/EMBEDDED.md:75` claim the
S3 build uses *"esp-dsp ASM + LX7 PIE SIMD (auto-picked by `esp-dsp`
at build)"*. The actual FFI bindings in
`embedded-poc/embedded-shared/src/esp_dsp_fft.rs` call
`dsps_fft2r_fc32_ae32_` (line 106), `dsps_fft2r_sc16_ae32_`
(line 118), and `dsps_dotprod_s16_ae32` (line 134). These are the
**`_ae32_` baseline asm**, not the LX7 PIE `_aes3_` variants.
Phase D1 below corrects this.

## Phase D — overview

| Sub-phase | Target | Effort | Expected gain | Risk |
|---|---|---|---|---|
| **D1** | Re-bind esp-dsp FFI to `_aes3_` (LX7 PIE) where esp-dsp 1.8.1 ships them | 0.5 day | stage1 inc FFT 60 → 30-40 ms; FFI dotprod path (currently unused, was BASIS) parity | Low — pure linkage change, fall back to `_ae32_` per-symbol if missing |
| **D2′** | Tighten the existing **f32 scalar** Goertzel (zero-pad scratch, kill per-sample bounds check, align state) | 0.5-1 day | pass 2 122 ms → ~100-110 ms (10-20 ms off) | Low |
| **D3** | PIE allsum + score for `coarse_sync` (and the incremental allsum builder in `stage1_inc`) | 2 days | coarse 137-170 ms → 60-80 ms (during cap; reduces stage1_inc slack pressure) | Med — coarse_sync is the recall floor, regression-test against existing sweeps |
| **D4** | PIE spectrum magnitude squared (`|re|² + |im|²`) post-FFT in stage1_inc | 1 day | stage1 inc FFT 60 ms → 50 ms | Low |

The original D2 (hand-rolled PIE Q15/Q31 Goertzel) is **deliberately
out** — see "D2 reconsidered" below for the cycle-budget evidence.
Goertzel is a tight IIR recurrence; the LX7 FPU + LLVM's existing
cross-tone unroll is already running at ≈ 1.6 cycles/tone/sample
(vs the 1.0 floor), so even the most optimistic PIE design only
buys ~1.6×, and only on the inner — easily eaten by Q31 setup,
saturation handling, and output extraction.

All remaining sub-phases are independent and individually
revertible. **Order D1 → D2′ first** (both low-risk, both cheap),
then D3 → D4 by measured residual.

## Reference budget

Goal: post-SlotEnd decode wall ≤ 300 ms on qso3 (current worst case
1.0 s, modal ~480 ms). D1+D2′+D3+D4 together should hit ~280-380 ms
modal; if that misses, stage 3 (BP/LDPC) tuning is the next lever,
not PIE Goertzel.

## D1 — switch to `_aes3_` (LX7 PIE esp-dsp kernels)

### Verification first

esp-dsp 1.8.1 is pinned in `embedded-poc/m5stack-s3-app/components_esp32s3.lock`.
Before changing any binding, **enumerate the actually-exported
LX7 PIE symbols** in the built `libesp-dsp.a` for the S3 target:

```sh
source ~/export-esp.sh
cd embedded-poc/m5stack-s3
cargo build --release
xtensa-esp32s3-elf-nm \
  .embuild/espressif/build/managed_components/espressif__esp-dsp/libesp-dsp.a \
  | grep -E '_aes3_|_ansi$|_ae32_' \
  | sort -u
```

Confirmed-present candidates to migrate to (subject to the above
listing):

- `dsps_fft2r_fc32_aes3_` — replaces `dsps_fft2r_fc32_ae32_`
  at three call sites: the `extern "C"` declaration (line 106),
  the 256-pt sub-FFT inside `MixedRadix3840Fft::process` (line
  284), AND the general-length path inside `EspDspFft::process`
  (line 324, which handles every non-3840 FFT including the
  stage1_inc-internal lengths). LX7 PIE radix-2 FFT, ~2× the
  AE32 version.
- `dsps_fft4r_fc32_aes3_` — **radix-4** alternative. If available,
  preferred for the 256-pt sub-kernel in `MixedRadix3840Fft`
  (line 281-294): 256 is `4^4`, so radix-4 is natively suited and
  is typically 1.4-1.7× faster than radix-2 on top of the PIE win.
- `dsps_dotprod_f32_aes3_` — currently not used (we hold the i16
  dotprod path), but worth wiring for future f32 callers.
- `dsps_fft2r_sc16_aes3_` if present — replaces
  `dsps_fft2r_sc16_ae32_` (line 118 + 445 + 499). If absent on
  1.8.1, keep `_ae32_` and revisit when esp-dsp ships sc16 PIE.

### Steps

1. **D1.0 — symbol audit** (commit message:
   `docs(embedded): record esp-dsp 1.8.1 _aes3_ symbol audit on S3`).
   Capture the `nm` output above into
   `docs/historical/ESP_DSP_1_8_1_SYMBOL_AUDIT.md` so future
   re-checks are diffable.
2. **D1.1 — fc32 radix-2 rebind**. In
   `embedded-poc/embedded-shared/src/esp_dsp_fft.rs:106,284,324`,
   add a second extern declaration for `dsps_fft2r_fc32_aes3_` and
   switch all three call sites (extern decl + `MixedRadix3840Fft`
   sub-kernel + generic `EspDspFft`) behind a
   `#[cfg(target_arch = "xtensa")]` + target-feature gate (LX7 =
   ESP32-S3 only; LX6 keeps `_ae32_`).
   Commit: `perf(embedded): bind LX7 fc32 FFT to esp-dsp PIE (_aes3_)`.
3. **D1.2 — fc32 radix-4 for the 256-pt sub-kernel** (only if
   D1.0 confirms `dsps_fft4r_fc32_aes3_`). Inside
   `MixedRadix3840Fft::process` (line 281-287), swap
   `dsps_fft2r_fc32_ae32_(..., 256, ...)` for
   `dsps_fft4r_fc32_aes3_(..., 256, ...)` + the matching init.
   Commit: `perf(embedded): use esp-dsp radix-4 PIE for the 3840-pt 256-row sub-FFT`.
4. **D1.3 — sc16 rebind if present**. Same shape as D1.1 against
   `dsps_fft2r_sc16_aes3_`.
5. **D1.4 — measurement**. Re-run the wav_sim slot loop:
   ```sh
   cd embedded-poc/m5stack-s3
   cargo build --release --bin mfsk-core-m5stack-s3
   ../scripts/flash-monitor.sh \
       target/xtensa-esp32s3-espidf/release/mfsk-core-m5stack-s3 \
       logs/s3_phaseD1_aes3_$(date +%Y-%m-%d).log \
       120
   ```
   Compare against `s3_phaseC_q3i8_2026-05-04.log`. Acceptance:
   stage1 inc < 45 ms AND no recall regression on qso3.

### Rollback

Per-binding revert is one-line; if D1.2 (radix-4) breaks shape
assumptions in `fft_mixed_3840`, keep D1.1 and skip D1.2.

## D2 reconsidered — why a PIE Goertzel kernel is *not* in this plan

### Target

`mfsk-core/src/ft8/decode_block/fill_symbol_spectra.rs:670-725`,
function `fill_symbol_spectra_goertzel`. The inner loop:

```rust
for n in 0..NSPS {                       // 1920 iterations
    let sample = audio[idx as usize].to_i16() as f32;
    for tone in 0..NTONES {              // 8 iterations, unrolled
        let s = sample + coeff[tone] * s_prev[tone] - s_prev2[tone];
        s_prev2[tone] = s_prev[tone];
        s_prev[tone] = s;
    }
}
```

### Cycle-budget evidence against PIE here

Measured on the qso3 sweep:

- pass 2 = 122 ms (dual-core wall) → 244 ms single-core work
- 30 cand × 79 sym × 1920 samp = 4.55 M sample-iterations
- 244 ms ÷ 4.55 M = 53.6 ns/sample-iter × 240 MHz =
  **12.9 cycles/sample for all 8 tones combined** =
  **1.6 cycles/tone/sample**

The LX7 single-issue f32 FPU's theoretical floor for this kernel is
~1 cycle/tone/sample (one fmadd issued per cycle, latency hidden by
the 8 independent cross-tone chains that LLVM unrolls — see the
inline comment at `fill_symbol_spectra.rs:708-716` for the existing
design intent). We are at 1.6, which is roughly 60 % of peak FPU
throughput. There is **at most a 1.6× headroom on the inner**, and
only at the inner — wrapper overhead is fixed.

Most optimistic PIE Q31 design (Option A from the earlier draft of
this doc: paired Q registers for i32 state, per-iteration saturating
`<< 1`):
- ~8 cycles/sample for 8 tones = 1 cycle/tone/sample
- → 244 ms × (1.0/1.6) ≈ 150 ms single-core = 75 ms dual-core
- → saves **≈ 47 ms** on the post-slot critical path

…before subtracting the cost of Q31 state setup, saturation
handling, mid-recursion re-normalisation, output extraction back to
f32, and the `extern "Rust"` shim hop. The realistic net is closer
to 20-30 ms, against 2-3 days of asm work and a real risk of
opening a numerics regression (PR #126 Gemini-bot review already
flagged three classes of footgun in the Q15 sketch — Q31 narrows
them but does not eliminate them).

### Decision

**Skip the hand-rolled PIE Goertzel kernel for Phase D.** Goertzel
is a tight IIR recurrence; the only parallelism available is the
8 independent tone-chains, which LLVM is *already* unrolling onto
the FPU pipeline at ~60 % of peak. PIE's strength is in
*dependency-free* vector reductions and gathers, which is exactly
what D3 (coarse_sync allsum) and D4 (|x|²) are.

The right Goertzel optimisations are scalar — D2′ below.

## D2′ — f32 scalar Goertzel micro-optimisations

Same target file, same inner loop. The wins here come from removing
per-iteration overhead that LLVM cannot eliminate by itself, not
from changing the arithmetic.

### Candidates (verify with `xtensa-esp32s3-elf-objdump -d` before
and after each change)

1. **Hoist the bounds check / zero-pad branch out of the sample
   loop.** Currently:
   ```rust
   let sample = if idx >= 0 && (idx as usize) < audio.len() {
       audio[idx as usize].to_i16() as f32
   } else {
       0.0
   };
   ```
   The branch executes 1920× per symbol × 79 symbols × 30 cand =
   4.55 M times. Replace with: build a per-symbol `[f32; NSPS]`
   scratch (or a wider per-call scratch) once outside the inner
   loop, with the head/tail zero-pad already applied; the inner
   loop then does a clean sequential f32 load. Wrapper cost is
   one `copy_from_slice` + two `fill(0.0)` per symbol — negligible
   vs the 1920-iter inner.
2. **Cast i16 → f32 once per symbol, not per sample.** Folds into
   step 1.
3. **Force 16-byte alignment** on the per-call `coeff`, `s_prev`,
   `s_prev2`, and the new sample scratch. LX7 FPU loads prefer
   aligned access; misalignment costs a cycle.
4. **`#[inline(always)]` on `to_i16`** for the embedded path so
   the per-sample fetch doesn't go through a thunk. Already true
   on host — verify on the `+esp` build.

### Approach

1. **D2′.0 — host measurement scaffold.** Add a host bench (criterion
   or hand-rolled) that times `fill_symbol_spectra_goertzel` against
   a fixed audio buffer. Lock the baseline ns/sample number; this
   is the gate for the embedded changes.
2. **D2′.1 — symbol-scratch hoist.** Implement #1+#2 above. Verify:
   - Host bench shows ≥ 10 % speedup, OR
   - The Xtensa disassembly shows the inner-loop branch and the
     i16→f32 conversion gone.
   If neither, revert.
   Commit: `perf(ft8): pre-padded per-symbol scratch for goertzel inner`.
3. **D2′.2 — alignment hints.** Force 16-byte alignment on the
   local state arrays in `fill_symbol_spectra_goertzel`. Rust does
   not allow `#[repr(align(N))]` on local variables, so the
   mechanical pattern is either (a) a tiny `#[repr(align(16))]`
   newtype wrapper struct around `[f32; NTONES]` (and the per-call
   scratch from D2′.1 around `[f32; NSPS]`), or (b) a fixed-size
   `Aligned16<T>(T)` helper if multiple call sites want it. Re-run
   host bench and embedded sweep.
   Commit: `perf(ft8): align goertzel state vectors for FPU load`.
4. **D2′.3 — measurement.** Re-run the wav_sim sweep on S3 and
   compare against the D1 log. Acceptance: pass 2 modal ≤ 110 ms
   (was 122 ms), identical decode list on qso3.

### Numerics

f32 throughout — no Q31, no shift, no saturation, no host A/B vs
fixed-point. The only correctness concern is that the
pre-pad/scratch path must produce the same f32 values as the
current branch for every `idx ∈ [0, audio.len())` and 0.0
otherwise. Host equivalence test against the existing implementation
on `qso3_busy.wav` is sufficient.

### Reference

esp-dsp's own `dsps_biquad_f32_ae32_.S` (LX6 baseline scalar IIR,
not a vector kernel) is informative on what GCC produces for a
well-tuned scalar f32 recurrence — the LLVM output should match
its instruction density modulo the difference between `s.fmadd`
and `s.fadd.s + s.fmul.s` issue patterns.

## D3 — PIE coarse_sync allsum + score

The first PIE-asm sub-phase in this plan (D1 is just a linkage
swap). Unlike Goertzel, the allsum precompute is a pure i16 gather
+ reduction with no inter-iteration dependency — exactly the shape
PIE is built for. Pursue after D1 + D2′ have landed and the build
infrastructure (Cargo `pie` feature, `cc::Build` for `*.S`, Rust
shim pattern) exists.

### Targets

1. `mfsk-core/src/ft8/decode_block/coarse_sync.rs:305-316` —
   `owned_allsum` precompute (sums 7 of 8 Costas-tone bins per
   `(fi, m)`). PIE 8-lane i16 gather + reduction.
2. `coarse_sync.rs:344-401` — score loop accumulators across 3
   Costas blocks × 7 tones × n_freq × n_lag. The per-cell f32
   `power_acc` sum can stay f32, but the **integer index gather**
   (8 bins, every-other) is a natural PIE `EE.VLD.128.IP`
   primitive.
3. `embedded-poc/embedded-shared/src/stage1_inc.rs` —
   the incremental allsum builder (the streaming counterpart to
   D3.1) needs the same treatment to stay in sync.

### Risk

`coarse_sync` is the recall floor. **Every change must pass the
host sweep + the 4-WAV embedded recall set** before merge. Use
the existing `coarse_sync_split_with_allsum_*` integration tests
as the gate.

## D4 — PIE `|x|²` for the FFT post-pass

Stage 1 incremental FFT (`embedded-shared/src/stage1_inc.rs`) and
the one-shot `compute_spectrogram`
(`mfsk-core/src/ft8/decode_block/spectrogram.rs:142-190`) both
end each row with `buf[i].norm_sqr()` over `n_freq ≈ 1043` bins.

PIE `EE.VMULAS.S16.QACC` does `re*re + im*im` for 4 complex
samples per instruction. Wins ~10 ms off stage1_inc; small but
free once the D3 wrapper machinery exists.

## Cross-cutting

### Build-system

Add a `pie` Cargo feature on `embedded-shared` that gates the
asm files and the wrapper module. Default-on for the
`xtensa-esp32s3-espidf` target via
`m5stack-s3/Cargo.toml` + `m5stack-s3-app/Cargo.toml` +
(future) `m5stack-cores3-app/Cargo.toml`. **Never** enable on
`xtensa-esp32-espidf` (LX6 has no PIE).

`build.rs` in `embedded-shared` picks up `src/pie/*.S` only when
the feature is on, using `cc::Build` with
`.compiler("xtensa-esp32s3-elf-gcc")` and `.flag("-mlongcalls")`.

### Regression gate

- Host: all existing `mfsk-core` decode tests pass (the f32
  baseline that D2′ must match cell-for-cell lives here).
- S3: `embedded-poc/m5stack-s3` wav_sim sweep on the 3-WAV
  qso3 set (qso3_busy, qso3_quiet, qso3_callcq) — same recall list
  as the baseline.
- Per-phase log captured under `embedded-poc/m5stack-s3/logs/`
  named `s3_phaseD<n>_<tag>_YYYY-MM-DD.log`. Compare against
  `s3_phaseC_q3i8_2026-05-04.log`.

### Doc fixups (Phase D0, ship as part of D1.0)

- `embedded-poc/CLAUDE.md` SIMD row — change LX7 cell to
  `"esp-dsp ASM (_ae32_); LX7 PIE _aes3_ being migrated in Phase D"`
  until D1 lands, then to `"esp-dsp ASM (_aes3_) + hand-rolled
  PIE kernels in embedded-shared/src/pie/"`.
- `docs/EMBEDDED.md:75-78` — same correction, both LX7 rows.
- `docs/ROADMAP.md` — add a `Phase D` section pointing at this
  doc.

## File-path index (for the implementer)

**Line numbers below were checked against the `476d2fa` parent
commit on `claude/esp32s3-simd-optimization-Kzo1G`.** They are
intentionally precise so the first implementer can jump straight
to the right call site, but they will rot as the code evolves —
re-grep the function/struct name (also given) before relying on
the number.

- esp-dsp FFI surface to rebind: `embedded-poc/embedded-shared/src/esp_dsp_fft.rs`
  (lines 86-141 = externs, 281-294 = `MixedRadix3840Fft::process`
  inner 256-pt fc32, 301-340 = generic `EspDspFft::process` fc32,
  411-481 = `MixedRadix3840Sc16Fft::process` + generic
  `EspDspFft16::process` sc16 — call sites 445 and 499).
- Goertzel target (D2′ scalar tweaks): `mfsk-core/src/ft8/decode_block/fill_symbol_spectra.rs:670-725`.
- coarse_sync target (D3 PIE allsum + score): `mfsk-core/src/ft8/decode_block/coarse_sync.rs:181-405`.
- stage1_inc allsum target (D3): `embedded-poc/embedded-shared/src/stage1_inc.rs`.
- Reference dotprod wrapper pattern (for D3/D4 Rust shim):
  `embedded-poc/embedded-shared/src/esp_dsp_fft.rs:167-183`
  (`mfsk_core_dot_q15_i32`).
- Reference esp-dsp PIE asm to crib from (D3/D4 — vector
  reductions, *not* IIR):
  `.embuild/espressif/build/managed_components/espressif__esp-dsp/modules/dotprod/`
  (after first build).
- Logs to compare against: `embedded-poc/m5stack-s3/logs/s3_phaseC_*`.
- Host bench scaffolding for D2′: `mfsk-core/tests/ft8_goertzel_vs_basis.rs`.

## Out of scope (deliberately)

- **PIE Goertzel kernel** — see "D2 reconsidered" above. Tight IIR,
  cross-tone parallelism already exploited by the FPU + LLVM unroll
  at ~60 % of peak, residual headroom too thin to justify Q31 asm.
- LX6 (Core2) — no PIE on classic ESP32, all D-phases are
  cfg-gated to S3. Core2 keeps the `_ae32_` scalar path.
- Stage-3 BP/LDPC inner — control-flow heavy, poor SIMD fit;
  separate effort if needed (track in `docs/ROADMAP.md` Open
  follow-ups).
