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
| post-SlotEnd total | 480 000 – 1 002 000 | budget = 13 800 ms (cap + late tail) |

The Phase-C streaming overlap (PR #123) parked stage1_inc +
coarse_sync inside capture, so the **critical path is pass 2 + stage
3**. Pass 2 is the largest fixed cost (Goertzel for every refined
candidate × 79 symbols × NSPS=1920 samples) and is **pure scalar
f32** today — the most leveraged single PIE target.

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
| **D2** | Hand-write PIE Q15 Goertzel for `fill_symbol_spectra_goertzel` | 2-3 days | pass 2 121 ms → 30-50 ms (3-4×) | Med — Q15 numerics need host A/B vs f32 |
| **D3** | PIE allsum + score for `coarse_sync` (and the incremental allsum builder in `stage1_inc`) | 2 days | coarse 137-170 ms → 60-80 ms (during cap; reduces stage1_inc slack pressure) | Med — coarse_sync is the recall floor, regression-test against existing sweeps |
| **D4** | PIE spectrum magnitude squared (`|re|² + |im|²`) post-FFT in stage1_inc | 1 day | stage1 inc FFT 60 ms → 50 ms | Low |

All sub-phases are independent and individually revertible. **Order
D1 → D2 first** (best ROI per day); D3 and D4 only if D1+D2 don't
already put us in the target budget.

## Reference budget

Goal: post-SlotEnd decode wall ≤ 300 ms on qso3 (current worst case
1.0 s, modal ~480 ms). D1+D2 alone should hit ~250-350 ms modal
based on the cost shares above.

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
  (line 106 + 284). LX7 PIE radix-2 FFT, ~2× the AE32 version.
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
   `embedded-poc/embedded-shared/src/esp_dsp_fft.rs:106,284`, add a
   second extern declaration for `dsps_fft2r_fc32_aes3_` and switch
   call sites behind a `#[cfg(target_arch = "xtensa")]` +
   target-feature gate (LX7 = ESP32-S3 only; LX6 keeps `_ae32_`).
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

## D2 — hand-rolled PIE Goertzel

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

8 tones × 1920 samples × per-symbol × 79 symbols × ~30 candidates =
~36 M FPU ops per slot, currently single-cycle-throughput f32 fmadd.
The 8 dependent chains are independent across `tone`, which is
exactly the PIE 8-lane Q15 vector layout (Q register = 8 × i16).

### Approach

1. **D2.0 — Q15 host A/B**. Land a **scalar Q15** version
   (`fill_symbol_spectra_goertzel_q15`) first, on host, with no PIE.
   Saturating Q15 arithmetic on f32-replica state. Compare against
   the f32 baseline using
   `tests/ft8_goertzel_vs_basis.rs` (existing) extended to cover
   q15-vs-f32 on `qso3_busy.wav` + the WSJT-X reference set:
   recall identical, per-station SNR within ±0.3 dB.
   Commit: `feat(ft8): scalar Q15 fill_symbol_spectra_goertzel + host A/B vs f32`.
2. **D2.1 — PIE kernel** in `embedded-shared/src/pie/goertzel_q15.S`
   (new file). Uses `EE.LD.QACC_H.L.128.IP`,
   `EE.VMULAS.S16.QACC.LDINCP`, `EE.VLDBC.16` style instructions to
   process 8 tones in one Q register per sample. Pseudocode:
   ```
   ; coeff vec (8 i16) in Q0
   ; s_prev vec in Q1, s_prev2 vec in Q2
   ; sample broadcast in Q3
   ; per n:
   ;   Q4 = Q0 * Q1 (Q15 saturating)
   ;   Q4 = Q4 + Q3 - Q2     ; new s
   ;   Q2 = Q1
   ;   Q1 = Q4
   ```
   Throughput target: ~1 cycle/sample (vs current ~8-10 cycles for
   the f32 unrolled chain), wall-clock 6-8× on the Goertzel inner.
3. **D2.2 — Rust wrapper** in
   `embedded-shared/src/pie/mod.rs`, gated by
   `#[cfg(all(target_arch = "xtensa", target_feature = "..."))]`.
   `fill_symbol_spectra_goertzel` in `mfsk-core` gains a
   `#[cfg(...)]` arm that calls the embedded symbol via the existing
   `extern "Rust"` shim pattern (see
   `mfsk_core_dot_q15_i32` in `esp_dsp_fft.rs:167-183`).
   Commit: `perf(embedded): PIE Q15 Goertzel kernel (m5stack-s3)`.
4. **D2.3 — on-device measurement**. Re-run wav_sim sweep; compare
   pass-2 wall against the D1 log. Acceptance: **pass 2 ≤ 50 ms
   modal AND identical decode list on qso3 + 1 other WAV**.

### Numerics gotchas

- The 8 tone frequencies span the slot's carrier window (200-2700
  Hz). `coeff = 2·cos(2π·f/Fs)` ∈ (-2, 2) — fits Q14.1 with one
  sign bit; pre-scale by `1/2` and shift compensate at the
  output.
- s_prev can overflow Q15 for high-energy slots. Adopt the same
  shift-then-renormalise strategy as `EspDspFft16` (per-symbol
  shift, recorded; output `re`/`im` re-scaled when materialised).
- Tail edge of `audio[]` zero-pads — the PIE kernel needs the
  same conditional or a pre-padded scratch (preferred: pad once
  per symbol in the wrapper, avoid PIE branch).

### Reference

esp-dsp's own `dsps_biquad_f32_aes3_.S` (3-tap IIR, structurally
identical to Goertzel) is the model to crib from. It ships under
`managed_components/espressif__esp-dsp/modules/iir/`.

## D3 — PIE coarse_sync allsum + score (optional)

Only pursue if D1+D2 don't hit the 300-ms post-slot target, or if
the in-capture share (currently 6% per Phase-E log line) starts
crowding stage1_inc as we add more candidates.

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
free once the D2 wrapper machinery exists.

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

- Host: all existing `mfsk-core` decode tests pass (the Q15
  scalar reference for D2 lives here).
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

- esp-dsp FFI surface to rebind: `embedded-poc/embedded-shared/src/esp_dsp_fft.rs`
  (lines 86-141 = externs, 281-294 = MixedRadix3840Fft, 445+499 = sc16).
- Goertzel target: `mfsk-core/src/ft8/decode_block/fill_symbol_spectra.rs:670-725`.
- coarse_sync target: `mfsk-core/src/ft8/decode_block/coarse_sync.rs:181-405`.
- stage1_inc allsum target: `embedded-poc/embedded-shared/src/stage1_inc.rs`.
- Reference dotprod wrapper pattern (for the PIE kernel Rust shim):
  `embedded-poc/embedded-shared/src/esp_dsp_fft.rs:167-183`
  (`mfsk_core_dot_q15_i32`).
- Reference esp-dsp PIE asm to crib from:
  `.embuild/espressif/build/managed_components/espressif__esp-dsp/modules/iir/dsps_biquad_f32_aes3_.S`
  (after first build).
- Logs to compare against: `embedded-poc/m5stack-s3/logs/s3_phaseC_*`.
- Host A/B test scaffolding: `mfsk-core/tests/ft8_goertzel_vs_basis.rs`.

## Out of scope (deliberately)

- LX6 (Core2) — no PIE on classic ESP32, all D-phases are
  cfg-gated to S3. Core2 keeps the `_ae32_` scalar path.
- Stage-3 BP/LDPC inner — control-flow heavy, poor SIMD fit;
  separate effort if needed (track in `docs/ROADMAP.md` Open
  follow-ups).
- esp-dsp upstream contribution of a generalised-Goertzel PIE
  kernel — would benefit the wider ecosystem, but out of scope
  for the ship-this-month decode-time target.
