//! FST4-60 decoder-only bench for ESP32-S3 (LX7) — issue #306/#307.
//!
//! Answers the question issue #306 actually asked: does the *decoder*
//! — LLR/BP/OSD over a real candidate population — fit inside
//! FST4-60's ~7 s post-slot margin on real CoreS3 hardware? Not "is
//! the whole embedded FST4 port done" (it isn't attempted here) and
//! not "is the WSPR result predictive of this" (issue #260's own
//! closing thread argued the opposite: WSPR's dominant cost was
//! Fano-sequential search burning a full node budget per failing
//! candidate, a failure mode FST4's bounded LDPC/BP + OSD-fallback
//! doesn't share).
//!
//! ## Why there is no FFT anywhere in this bench
//!
//! The first version of this bench baked only the wideband
//! `build_fft_cache` output and ran `coarse_sync` + `decode_frame` on
//! device — mirroring `wspr_bench`'s baked-baseband shape. On real
//! hardware it panicked on `coarse_sync`'s very first FFT call:
//! FST4-60A's spectrogram length (`NSPS × 2 = 7776 = 2⁵ × 3⁵`) isn't a
//! power of two, and the embedded `fft-extern`/ESP-DSP backend only
//! serves power-of-two lengths (plus one hand-rolled exception for
//! FT8's own `3840`). `downsample_cached`'s inverse FFT
//! (`fft2_size = 6912 = 2⁸ × 27`) is a second, independently-sized
//! non-power-of-two transform in the same path. Every one of FST4's 5
//! submodes hits this (filed as issue #307, with the full
//! factorization table — FST4-120's carries a bare factor of 41).
//!
//! Rather than wait on #307's FFT kernel to get a first wall-clock
//! number, this bench skips **both** FFT sites by baking each real
//! candidate already refined on a host: `mfsk-core/tests/
//! fst4_wsjtx_samples.rs::fst4_bake_golden_refined_candidates` runs
//! `coarse_sync` + `refine_candidate_position` +
//! `dedup_refined_candidates`'s own near-duplicate rule on the WSJT-X
//! golden, and bakes the survivors' `(cd0, freq_hz, i0, score)`
//! tuples — exactly what `process_candidate_basic_impl`'s
//! `precomputed_refine` parameter accepts. `decode_frame` doesn't
//! expose that parameter, so a new `engine::pipeline::
//! process_candidate_precomputed` (internal-testing-gated, same
//! visibility shape as `process_candidate_basic`) does. The result:
//! **the on-device path here never calls an FFT**, only LLR/BP/OSD —
//! exactly the "decoder" issue #306 asked about, isolated instead of
//! blocked.
//!
//! Two more FFT sites turned up past that first fix, both closed the
//! same day:
//!
//! - `GenericPipelineProtocol::snr_db`'s FST4 override
//!   (`fst4::baseline::fst4_snr_db`) calls `downsample_cached` a
//!   *second*, independent time from `fft_cache` — `fst4_raw_cs` needs
//!   the non-RMS-normalised spectrum, which the already-normalised
//!   `cd0` from `precomputed_refine` can't substitute for. Closed by a
//!   new `skip_snr` parameter on `process_candidate_basic_impl` /
//!   `process_candidate_precomputed`: `true` stores `NAN` in
//!   `DecodeResult::snr_db` instead of calling `P::snr_db` at all —
//!   this bench's SNR values are not real measurements, only its
//!   wall-clock is.
//! - `engine::llr::symbol_spectra` — the actual first step of LLR
//!   extraction, called for *every* candidate regardless of
//!   `precomputed_refine`/`skip_snr` — plans a `ds_spb =
//!   NSPS/NDOWN`-point FFT (36 for FST4-60A; 36-42 across all 5
//!   submodes), also not a power of two. Closed by
//!   `esp_dsp_fft::DirectDft`: a plain O(N²) textbook DFT (not an
//!   approximation — every FFT computes the identical sum by a faster
//!   route, so there is no algorithm-specific correctness risk the
//!   way a new fast-transform derivation would carry), wired into
//!   `EspDspPlanner::plan_forward` for any non-power-of-2 length up to
//!   `DIRECT_DFT_MAX_LEN` (64). Verified against `numpy.fft` on host
//!   at N=36/42 to ~1e-6 (f32-level precision) before flashing.
//!
//! With all three closed, the on-device path genuinely has zero FFT
//! calls left. One more thing turned up before a number came back —
//! `fft_cache` (5.7 MiB) was still being loaded and shipped even
//! though nothing on this path reads it once `precomputed_refine` +
//! `skip_snr = true` are both set; PSRAM math that had looked fine on
//! paper (893 KiB free post-load, per an earlier version of this
//! comment) turned out to leave the candidate loop's own transient
//! BP/OSD allocations no room at all — a 512 KiB request failed.
//! Dropped `fft_cache` from this bench entirely (`run_bench` no
//! longer takes it, `&[]` goes to `process_candidate_precomputed`
//! instead — safe exactly because nothing on this path dereferences
//! it); PSRAM free jumped from 87 KiB to 5.97 MiB post-load.
//!
//! **Measured, 2026-08-16, CoreS3 @ 240 MHz / opt-level 3, single
//! core:** the candidate loop — all 41 baked candidates, LLR/BP/OSD
//! only, zero FFT calls — took **89.691 s** and reached both real
//! decodes (`CQ N5TM EL29`, `CQ K9KFR EN71`), matching the host
//! golden exactly. Host `decode_loop` for the same 41 candidates is
//! 51.9 ms (`MFSK_TRACE_STAGE_FST4=1`, this box) — a **~1728×**
//! device/host ratio, roughly **13× over** FST4-60's ~7 s post-slot
//! margin.
//!
//! That ratio is the real finding here, not just the raw seconds:
//! it's close to WSPR's own *first*, wholly-unoptimized measurement
//! (issue #260: 1214.3 s against a 709.5 ms host baseline, ~1712× —
//! before `minsync2`, `opt-level=3`, the 160→240 MHz clock fix, or
//! any of that investigation's other four-and-a-half-times-total
//! speedup landed). This bench's build already has `opt-level = 3`
//! and the 240 MHz clock fix (both are `m5stack-cores3-app`-wide, not
//! per-bin) — so this *is* already past two of WSPR's early wins, and
//! the ratio still landed in WSPR's pre-optimization territory. The
//! premise this module's own earlier text repeated — that FST4's
//! bounded LDPC/BP/OSD "fails cheaply by construction" and so
//! shouldn't need WSPR's kind of optimization pass — was explicitly
//! flagged elsewhere as *"an untested claim, not a measured one"*
//! (`docs/reference/EMBEDDED.md`). It is now tested, and at this
//! unoptimized state it does not hold: something in FST4's LLR/BP/OSD
//! path costs about as much per real-world candidate as WSPR's
//! Fano-sequential search did before WSPR's own dedicated tuning
//! pass. Candidate suspects, unmeasured: the `LLR_NSYM_MAX = 8`
//! staircase rung (`4⁸ = 65536` tone-combination hypotheses per
//! group — the module doc for `fst4::decode`'s lazy-staircase code
//! calls this "128-256× FT8/FT4's own deepest rung"), and simply that
//! FST4 runs the generic f32 pipeline with no embedded-specific
//! optimisation pass at all, unlike FT8's dedicated fixed-point
//! `decode_block` or WSPR's now-four-rounds-tuned `decode_scan`.
//!
//! ## Follow-up, same day: per-candidate timing, and an OSD/BP split
//!
//! Added per-candidate wall-clock logging (collected into a `Vec`,
//! dumped *after* the timed loop so UART writes don't contaminate the
//! numbers) and a build-time `MFSK_FST4_BENCH_DEPTH=bp_only` switch
//! (`DecodeDepth::BP_ONLY` vs. the default `::FULL` — differ only in
//! `osd: bool`, since `LlrEffort` doesn't affect FST4 either way; see
//! that enum's own doc comment). Same shape as WSPR's own issue #260
//! controlled experiment (`confirmed = None`, short-circuiting before
//! `osd_decode`).
//!
//! **The 89.4 s is not spread across all 41 candidates — 94% of it is
//! 6 candidates, all failures:**
//!
//! | tier | count | each | total | decoded |
//! |---|---:|---:|---:|---|
//! | nsync-gate fails | 33 | 50-177 ms | ~6 s | no |
//! | **the tail** | **6** | **13.8-14.2 s** | **84.2 s (94%)** | **no** |
//! | real signals | 2 | 54-58 ms | 0.1 s | **yes** |
//!
//! Both real decodes are cheap — the entire cost is 6 candidates the
//! decoder ultimately rejects, matching WSPR's own shape (issue #260:
//! "OSD ran 896 times and succeeded 0 times... the cost is not where
//! any of us was looking").
//!
//! **OSD vs. plain-BP split on the same 6, `BP_ONLY` vs. `FULL`:**
//!
//! | | total | the 6-candidate tail (avg) |
//! |---|---:|---:|
//! | `FULL` (OSD on) | 89.411 s | 14.04 s |
//! | `BP_ONLY` (OSD off) | 51.755 s | 7.76 s |
//! | OSD's share | 37.66 s (42%) | 6.28 s |
//!
//! Both decodes still succeed with OSD off — on this file neither real
//! signal ever needed it. So OSD is a real cost (42% of the total) but
//! **not the majority one**: plain BP/LLR alone is 51.8 s, already
//! ~7.4× over the ~7 s budget by itself.
//!
//! ## Follow-up 2, same day: the `LLR_NSYM_MAX = 8` suspect confirmed
//!
//! `MFSK_FST4_BENCH_DEPTH=llr_probe` (see [`run_llr_stage_probe`])
//! calls `symbol_spectra` + each `compute_llr_*` stage directly, no
//! BP/OSD at all, timing every stage for every one of the 41
//! candidates (not just the tail — this mode skips the real
//! pipeline's early nsync-gate exit on purpose, to compare all four
//! stages against each other on equal footing).
//!
//! | stage | total (41 candidates) | share |
//! |---|---:|---:|
//! | `symbol_spectra` | 1.39 s | 0.5% |
//! | nsym=1 (`compute_llr_fast`) | 0.09 s | <0.1% |
//! | nsym=2 | 0.14 s | <0.1% |
//! | nsym=4 (`LLR_NSYM_MID`) | 1.10 s | 0.4% |
//! | **nsym=8 (`LLR_NSYM_MAX`)** | **301.2 s** | **99.1%** |
//!
//! `nsym=8` alone costs **~7.347 s per candidate**, uniform to within
//! 0.01% across all 41 — a pure function of the computation's size
//! (`4⁸ = 65536` tone-combination hypotheses per symbol group), not
//! data-dependent. That single stage accounts for essentially all of
//! the `BP_ONLY` run's ~7.76 s/candidate tail cost
//! (0.034+0.002+0.003+0.027+7.347 ≈ 7.41 s of it; the remaining
//! ~0.35 s is presumably `decode_soft_pooled`'s own BP iterations,
//! not separately measured here).
//!
//! One structural note this doesn't change: `SYNC_Q_MIN = 16` is
//! already FST4's own equivalent of WSPR's `minsync2` — WSJT-X's own
//! pre-ladder gate (`get_fst4_bitmetrics.f90`), faithfully ported
//! (issue #197), and it's what keeps 33 of 41 candidates from ever
//! reaching this stage in the real pipeline. The 8 that do clear it
//! are the same population a real `jt9` would also have to run this
//! computation on — there is no missing cheap-reject lever the way
//! WSPR's `minsync2` gap was; the cost is intrinsic to the candidates
//! that legitimately warrant deep decoding, not a filtering gap.
//!
//! So the remaining levers are in `compute_llr_partial` itself
//! (algorithmic restructuring, fixed-point, SIMD/PIE), accepting a
//! recall trade-off by skipping `nsym=8` on embedded the way FT8's
//! ship config skips OSD, or parallelism (dual-core).
//!
//! ## Follow-up 3, same day: one of those levers, taken — `~1.25×` overall
//!
//! Disassembling `fill_bmet_for_nsym`'s compiled Xtensa output
//! (`xtensa-esp32s3-elf-objdump` against this exact bench's ELF) found
//! that its hot loop's two `f32::max()` calls each compiled to a real
//! `callx8` to libm's `fmaxf` — Xtensa's FPU has no native float
//! max/min instruction, so LLVM can't lower `max`'s IEEE-754
//! NaN-propagation semantics to a single compare. At `nsym=8` that's
//! ~42 M subroutine calls per candidate (2 calls × 65536 elements × 16
//! bit positions), not 42 M single-cycle compares.
//!
//! `mfsk_core::engine::llr::fill_bmet_for_nsym`'s two operands here
//! (`v_for_one`/`v_for_zero`) are always either `sqrt(re²+im²)`
//! (finite, ≥ 0) or the literal `f32::NEG_INFINITY` — never NaN — so
//! `max`'s NaN handling is provably dead weight on this specific loop.
//! Replaced with a plain `>` comparison (bit-identical result for
//! non-NaN inputs, confirmed: every existing golden/AWGN
//! FT8/FT4/FST4 test still passes byte-for-byte). Re-disassembling
//! confirmed the fix: the hot loop now compiles to Xtensa's native
//! `ule.s` compare + `bt`/`bf` branch, zero `callx8` in that loop (one
//! `fmaxf` call remains elsewhere in the function, for the
//! once-per-bit-position `den = max_one.max(max_zero)` normalisation
//! step — negligible, 16 calls/group instead of 2×65536×16, left
//! alone).
//!
//! Measured effect, same CoreS3, same golden, same 41 candidates:
//!
//! | | before | after | speedup |
//! |---|---:|---:|---:|
//! | `nsym=8` (41-candidate `llr_probe` total) | 301.2 s | 178.1 s | 1.69× |
//! | `nsym=4` (same) | 1.10 s | 0.61 s | 1.80× |
//! | **`FULL` candidate loop (real 8-candidate population)** | **89.411 s** | **71.312 s** | **1.25×** |
//!
//! Still 2/2 real decodes, identical to every prior run. A genuine,
//! zero-risk, two-line win — but far short of the ~13× this session's
//! first measurement needed, and short of the ~1.7× its own `nsym=8`
//! isolation would suggest (the loop's other per-element overhead —
//! indexing, the branchless-select setup, memory traffic through
//! `s2` — was already there and untouched, so removing just the call
//! recovers less than the call's own share of a single iteration).
//! `compute_llr_partial`'s remaining levers (real algorithmic
//! restructuring, fixed-point, hand-written PIE vectorisation, a
//! recall trade-off, dual-core) are still open — this is one
//! confirmed data point on how much any single one of them is worth,
//! not a closed investigation.
//!
//! ## Follow-up 4, same day: OSD's own allocator traffic
//!
//! With `nsym=8` fixed, re-tallying the `FULL`/`BP_ONLY` split from
//! earlier put OSD at 53% of the total (up from 42% — the LLR fix
//! shrank the denominator, not OSD itself). `mfsk_core::fec::ldpc::
//! osd::osd_decode_generic::<Ldpc240_101Params>` (FST4's OSD entry
//! point, shared with FT8's `Ldpc174_91Params` and MSK144's
//! `Ldpc128_90Params` instantiations of the same generic function) is
//! an order-3 combinatorial search — `try_candidate` gets called
//! `C(101,3) ≈ 166 650` times per call for FST4 — and its closure
//! allocated two fresh heap `Vec<u8>` (240 B + 101 B) on *every* call,
//! immediately freed again on the (overwhelmingly common) path where
//! CRC verification fails. Disassembling confirmed it: 9
//! `__rust_alloc_zeroed` call sites and 38 `__rust_dealloc` sites in
//! one 4.2 KiB function.
//!
//! `P::N`/`P::K` are compile-time consts on `LdpcParams`, but `[u8;
//! P::N]` isn't expressible in a function generic over `P` on stable
//! Rust (`generic_const_exprs` remains nightly-only — confirmed by
//! trying it: `error: generic parameters may not be used in const
//! operations`). Used the same "fixed max bound + runtime-length
//! prefix slice" idiom `engine::llr`'s `MAX_NSYM`/`MAX_IBMAX_PLUS_1`
//! already established for the identical problem: a `[u8; 256]`
//! stack buffer (≥ 240/174/128, the three protocols' `N`), sliced to
//! `[..n]`. Collapsed the `try_candidate` + `update_best` closure
//! pair into one closure writing into reused scratch, copying into
//! `best_codeword`/`best_decoded` only on an actual improvement.
//! Bit-identical on host — confirmed against all three protocols'
//! goldens (FT8 full-parity 8/8, FT8 ship-config, MSK144, FST4-60),
//! not just FST4's. Re-disassembling: `__rust_alloc_zeroed` 9 → 1
//! call sites, `__rust_dealloc` 38 → 1 (the remaining single calls
//! are `osd_setup_generic_packed`'s one-time setup and the final
//! `OsdResult` construction, not per-candidate).
//!
//! | | before | after | speedup |
//! |---|---:|---:|---:|
//! | **`FULL` candidate loop** | **71.312 s** | **67.789 s** | **1.052×** |
//!
//! Real, but smaller than the alloc-call-count reduction alone would
//! suggest — same shape as the LLR fix: removing the allocator calls
//! recovers only their own share of a `try_candidate` iteration, and
//! the surrounding O(n) XOR/permute/weighted-distance work (n=240)
//! was already there. Cumulative at this point: 89.411 s → 67.789 s,
//! 1.319×, ≈9.7× over the ~7 s budget (down from ≈13×).
//!
//! ## Follow-up 5, same day: one lever on each side
//!
//! @VK3NV (issue #306) pointed out `fill_bmet_for_nsym`'s inner loop
//! still had a shift, a mask and two branchless selects per element
//! even after the `fmaxf` fix, and that for a fixed `bit_sel = b`,
//! `(i >> b) & 1` isn't data-dependent — it's the known periodic
//! pattern `2^b` zeros then `2^b` ones. `nt = ntones^nsym` is always a
//! power of 2, so `block = 2^bit_sel` always divides `nt` evenly — the
//! loop now walks `s2` in `2^bit_sel`-sized contiguous blocks,
//! alternating which accumulator each block feeds, with one compare
//! per element and no per-element predicate machinery. Bit-identical
//! across FT8/FT4/FST4/MSK144's shared code path.
//!
//! On the OSD side: `try_and_update`'s scatter
//! (`c[perm[col]]=cp[col]`, O(n)=240) ran on every one of the
//! ~166 650 calls before the `verify` (CRC-style) gate that rejects
//! nearly all of them — its only purpose was producing `decoded =
//! c[..k]` for `verify`. An inverse permutation
//! (`inv_perm[perm[col]]=col`, computed once) lets `decoded[i] =
//! cp[inv_perm[i]]` be gathered directly in O(k)=101, deferring the
//! full O(n) scatter to the rare candidate that actually passes
//! `verify`. Separately, the weighted-distance loop (reached only on
//! that same rare pass) was recomputing `llr[perm[col]] > 0.0` /
//! `.abs()` fresh on every call despite `perm`/`llr` never changing —
//! precomputed `hdec_perm`/`absrx_perm` once instead. Both
//! bit-identical on host (FT8 full-parity 8/8, ship-config, MSK144,
//! FST4-60, full 435-test lib suite). Re-disassembling:
//! `osd_decode_generic`'s closure shrank from 4.2 KiB to ~450 bytes,
//! and the scatter/weighted-distance code is now visibly gated behind
//! the `verify` call.
//!
//! | | before | after | speedup |
//! |---|---:|---:|---:|
//! | **`FULL` candidate loop** | **67.789 s** | **59.775 s** | **1.134×** |
//!
//! 2/2 real decodes unchanged. Cumulative at this point: 89.411 s →
//! 59.775 s, 1.496×, ≈8.5× over the ~7 s budget (down from ≈13×).
//!
//! ## Follow-up 6, same day: bit-packing OSD's XOR construction
//!
//! The one unconditional O(n) step Follow-up 5's `verify`-gate
//! reordering couldn't reach: codeword *construction*
//! (`c1[col]^=g[k1*n+col]`, and again for `c2`/`c3`/`c4`) can't be
//! deferred behind `verify` the way the scatter was — it's not
//! consuming a candidate, it *is* the candidate, run unconditionally
//! on every one of the ~166 650 order-3 combinations. Unlike the
//! scatter, no permutation is involved here — both `c*` and `g`'s
//! rows are already in the same permuted-column order — so it's a
//! plain elementwise XOR of two equal-length byte arrays, exactly the
//! shape `osd_setup_generic_packed` already bit-packs for its own
//! row-XOR during elimination. Repacked `g` into `OSD_WORDS=4` `u64`
//! words per row once per call (O(k·n), negligible), turning each
//! `c1`/`c2`/`c3`/`c4` construction from a 240-byte copy + 240-byte
//! XOR loop into 4 word-XORs with no copy at all (`child = parent ^
//! g_packed[row]` directly). `try_and_update` moved to the packed
//! representation too — the O(k) gather trades a byte load for a
//! bit-extract (same asymptotic cost), and the scatter +
//! weighted-distance loop collapsed from two passes into one.
//! Bit-identical on host (FT8 full-parity 8/8, ship-config, MSK144,
//! FST4-60, full 435-test lib suite, `fixed-point` spot-check).
//!
//! | | before | after | speedup |
//! |---|---:|---:|---:|
//! | **`FULL` candidate loop** | **59.775 s** | **54.087 s** | **1.105×** |
//!
//! 2/2 real decodes unchanged. Stack headroom dropped ~90 KB → ~83 KB
//! (new `g_packed` scratch ≈ 8 KB) — still wide margin.
//! **Cumulative from the original 89.411 s baseline: 1.653×, ≈7.7×
//! over the ~7 s budget (down from ≈13×).** `compute_llr_partial`'s
//! own recursive amplitude-table build (`build_group_amplitudes`) is
//! the last identified-but-untouched micro-lever, though at ~300
//! calls/candidate vs. OSD's ~166 650 its share is much smaller — see
//! `docs/reference/EMBEDDED.md` and issue #306 for where this leaves
//! the "does it fit" question.
//!
//! Also measured in passing: peak stack usage was tiny —
//! `BENCH_STACK`'s 96 KiB guess left 95 064 B of headroom untouched
//! (only ~3.2 KiB actually used), a wide margin unlike `wspr_bench`'s
//! own stack history. Worth right-sizing down for a future run, not
//! urgent for a single-shot bench.
//!
//! ## Follow-up 7, same day: `no8_osd` — the recall trade-off, verified
//! then measured
//!
//! With the small disassembly-led fixes exhausted (~7.7× over budget),
//! the user raised the recall-trade-off question again: skip
//! `LLR_NSYM_MAX=8` entirely (99% of the LLR/BP-side cost)? The one
//! observation available at the time — this file's own 2 real decodes
//! never reach `nsym=8` or OSD, converging in 54-58 ms — is exactly a
//! single-file result this project's own discipline says not to trust
//! without checking against real AWGN data. `tests/fst4_sweep.rs`'s
//! `fst4_60_diag_recall_tradeoff` (real FST4-60 AWGN corpus, 20
//! trials/SNR) found the hypothesis **not confirmed, in the opposite
//! direction from WSPR's `minsync2`**: near the -26..-28 dB crossing,
//! both `nsym=8` and OSD carry real recall — but critically, dropping
//! `nsym=8` while *keeping* OSD (`no8_osd`) recovers *more* recall than
//! dropping OSD while keeping `nsym=8` (`bp_only`, `DecodeDepth::
//! BP_ONLY`'s shape) at every SNR tested, despite `nsym=8` being the
//! far more expensive rung — OSD is a structurally different fallback
//! (bit-flip search over the LDPC systematic basis) that doesn't need
//! BP to converge at all, so it rescues candidates `nsym=8` also
//! misses. Per-*trial* (not just aggregate) monotonicity checks ruled
//! out a bookkeeping bug behind that result.
//!
//! `no8_osd` wasn't reachable via `DecodeDepth` alone (`LlrEffort` is
//! FT8-only in practice — see its own doc comment). Added
//! `skip_llr_nsym_max: bool` to `process_candidate_basic_impl` /
//! `process_candidate_precomputed` (same additive pattern as
//! `skip_snr`): when `true`, the `LLR_NSYM_MAX` staircase rung is
//! skipped entirely — both the BP attempt and its slot in OSD's
//! variant list — while OSD itself runs unmodified. `false` for every
//! existing caller, full merge gate + clippy green.
//!
//! Measured on real CoreS3 hardware, this bench's `MFSK_FST4_BENCH_
//! DEPTH=no8_osd`, same 41 baked candidates:
//!
//! | | `full` | `no8_osd` | speedup |
//! |---|---:|---:|---:|
//! | **candidate loop** | **54.087 s** | **25.710 s** | **2.10×** |
//!
//! 2/2 real decodes unchanged. **Cumulative from the original
//! 89.411 s baseline: 3.48×, ≈3.67× over the ~7 s budget** — a much
//! bigger single-step win than any of the six disassembly-led fixes,
//! but not free: this is the conscious recall trade-off `fst4_60_
//! diag_recall_tradeoff` measured, not a zero-cost speedup.
//!
//! Even generous dual-core assumptions don't close the remaining gap
//! alone: WSPR's own real dual-core yield was 1.35-1.47× (issue #260,
//! small-N straggler effects), and `no8_osd`'s 25.71 s ÷ 1.35-1.47×
//! is still 17.5-19.0 s (≈2.5-2.7× over budget); even the theoretical
//! 2.0× ceiling only reaches 12.9 s (≈1.84× over). Untested territory
//! past this point — recorded, not pursued further this round.
//!
//! ## What this does *not* attempt
//!
//! No `coarse_sync` on-device (can't yet — #307), no PSRAM-vs-SRAM
//! bandwidth arms, no dual-core split, no WiFi, no spot reporting, no
//! steady-state multi-slot pipeline. `wspr_bench` grew all of that
//! from real on-device findings over several rounds (issue #260);
//! building the same scaffolding here ahead of a first real
//! measurement would be guessing at what FST4 actually needs.
//!
//! The candidate population processed (41 survivors of 50 raw
//! `coarse_sync` candidates on the golden WAV, post-dedup) is the
//! same one `decode_frame_impl` would actually decode — including
//! every candidate that *fails*, not just the 2 real signals. WSPR's
//! own measurement (issue #260) found that most of a real candidate
//! loop's cost is exactly there; timing only the successes would
//! understate this the same way.

extern crate alloc;

use alloc::vec::Vec;

use num_complex::Complex32;

use mfsk_core::engine::ModulationParams;
use mfsk_core::engine::equalize::EqMode;
use mfsk_core::engine::llr::{compute_llr_fast, compute_llr_partial, symbol_spectra};
use mfsk_core::engine::pipeline::{DecodeDepth, DecodeStrictness, process_candidate_precomputed};
use mfsk_core::engine::sync::SyncCandidate;
use mfsk_core::fst4::Fst4s60;
use mfsk_core::fst4::decode::FST4_60A_DOWNSAMPLE;
use mfsk_core::msg::wsjt77::unpack77;

/// Matches `fst4::decode`'s own (private) `SYNC_Q_MIN` — WSJT-X's
/// `get_fst4_bitmetrics.f90` pre-ladder nsync gate (issue #197). Not
/// reachable from here (it isn't `pub`, deliberately — see that
/// const's own doc comment), so redeclared; if it ever moves this
/// bench silently stops matching the shipped gate rather than failing
/// to compile, which is the one place this file trades safety for
/// not needing a crate change just to flash a bench.
const SYNC_Q_MIN: u32 = 16;

const MALLOC_CAP_8BIT: u32 = 1 << 2;
const MALLOC_CAP_SPIRAM: u32 = 1 << 10;
const MALLOC_CAP_INTERNAL: u32 = 1 << 11;

fn now_us() -> i64 {
    unsafe { esp_idf_svc::sys::esp_timer_get_time() }
}

/// Bytes of the calling task's stack never touched so far — same
/// technique `wspr_scan::stack_headroom` uses, duplicated rather than
/// shared across a feature boundary this bench doesn't otherwise need.
fn stack_headroom() -> u32 {
    unsafe { esp_idf_svc::sys::uxTaskGetStackHighWaterMark(core::ptr::null_mut()) }
}

fn log_heap(tag: &str) {
    unsafe {
        log::info!(
            "[mem] {tag}: internal {} KB (largest contig {} KB), PSRAM {} KB (largest contig {} KB)",
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            ) / 1024,
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) / 1024,
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
                / 1024,
        );
    }
}

/// One baked, already-refined candidate — see the module doc and
/// `fst4_bake_golden_refined_candidates`'s own doc comment for the
/// exact byte layout this parses.
struct RefinedCandidate {
    cand: SyncCandidate,
    cd0: Vec<Complex32>,
    refined_freq_hz: f32,
    refined_i0: i32,
    refined_score: f32,
}

/// `bin` is the baked refined-candidates asset (`include_bytes!`).
/// Byte-wise, not a transmute: `include_bytes!` gives 1-byte alignment
/// and Xtensa faults on an unaligned `f32`/`i32` load.
fn load_refined_candidates(bin: &[u8]) -> Vec<RefinedCandidate> {
    let mut off = 0usize;
    let n = u32::from_le_bytes(bin[off..off + 4].try_into().unwrap()) as usize;
    off += 4;
    let mut out = Vec::with_capacity(n);
    for _ in 0..n {
        let freq_hz = f32::from_le_bytes(bin[off..off + 4].try_into().unwrap());
        let dt_sec = f32::from_le_bytes(bin[off + 4..off + 8].try_into().unwrap());
        let score = f32::from_le_bytes(bin[off + 8..off + 12].try_into().unwrap());
        let refined_freq_hz = f32::from_le_bytes(bin[off + 12..off + 16].try_into().unwrap());
        let refined_i0 = i32::from_le_bytes(bin[off + 16..off + 20].try_into().unwrap());
        let refined_score = f32::from_le_bytes(bin[off + 20..off + 24].try_into().unwrap());
        off += 24;
        let mut cd0 = Vec::with_capacity(FST4_60A_DOWNSAMPLE.fft2_size);
        for _ in 0..FST4_60A_DOWNSAMPLE.fft2_size {
            let re = f32::from_le_bytes(bin[off..off + 4].try_into().unwrap());
            let im = f32::from_le_bytes(bin[off + 4..off + 8].try_into().unwrap());
            cd0.push(Complex32::new(re, im));
            off += 8;
        }
        out.push(RefinedCandidate {
            cand: SyncCandidate {
                freq_hz,
                dt_sec,
                score,
            },
            cd0,
            refined_freq_hz,
            refined_i0,
            refined_score,
        });
    }
    assert_eq!(off, bin.len(), "refined-candidates asset byte accounting mismatch");
    out
}

/// Install the ESP-IDF logger, at most once per boot — same guard
/// `wspr_bench::init_logger_once` uses (`EspLogger::initialize_default`
/// aborts on a second call).
pub fn init_logger_once() {
    static LOGGER_READY: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if LOGGER_READY
        .compare_exchange(
            false,
            true,
            core::sync::atomic::Ordering::AcqRel,
            core::sync::atomic::Ordering::Acquire,
        )
        .is_ok()
    {
        esp_idf_svc::log::EspLogger::initialize_default();
    }
}

/// Runs `process_candidate_precomputed::<Fst4s60>` over every baked,
/// already-refined candidate — no FFT anywhere in this call chain
/// (see the module doc for why). `refined_bin` is the `include_bytes!`
/// blob produced by `fst4_bake_golden_refined_candidates`.
///
/// No `fft_cache` here, deliberately — the first real-device run
/// (2026-08-16) loaded it anyway (5.7 MiB) alongside the 2.16 MiB of
/// baked `cd0` buffers, leaving only ~87 KiB PSRAM free, and the
/// candidate loop's own transient allocations (BP/OSD scratch) then
/// failed a 512 KiB request. `fft_cache` was never actually *read* on
/// this path: `precomputed_refine` already skips the one call
/// (`downsample_cached`) that would use it, and `skip_snr = true`
/// skips the other (`fst4_raw_cs`, inside `P::snr_db`) — so it was
/// PSRAM spent on a buffer nothing touches. Dropping it is what turns
/// "peak usage was never actually the risk" (this module's earlier,
/// wrong conclusion — see below) into true.
pub fn run_bench(refined_bin: &[u8]) {
    log::info!("mfsk-core {}", mfsk_core::VERSION);
    log::info!(
        "fst4_bench: baked asset refined_candidates={} bytes",
        refined_bin.len(),
    );

    // Third mode, alongside FULL/BP_ONLY above: does the ~7.76 s the
    // `BP_ONLY` run still pays per hard candidate concentrate in the
    // `LLR_NSYM_MAX = 8` LLR-computation rung specifically, as the
    // module doc's leading suspect predicts? Bypasses
    // `process_candidate_precomputed` (and therefore BP/OSD) entirely
    // — times only `symbol_spectra` + each `compute_llr_*` stage
    // directly, the same building blocks `process_candidate_basic_
    // impl`'s lazy staircase calls internally (`engine::pipeline.rs`
    // lines ~926-967).
    if option_env!("MFSK_FST4_BENCH_DEPTH") == Some("llr_probe") {
        log::info!("fst4_bench: LLR-stage probe (MFSK_FST4_BENCH_DEPTH=llr_probe) — no BP/OSD");
        run_llr_stage_probe(refined_bin);
        return;
    }

    // Fifth mode: rung-major scheduling (issue #306 item 3, VK3NV) —
    // real-hardware confirmation of `tests/fst4_sweep.rs::fst4_60_diag_
    // rung_major_scheduling`'s host-timing projection. See
    // `run_rung_major`'s own doc comment.
    if option_env!("MFSK_FST4_BENCH_DEPTH") == Some("rung_major") {
        log::info!("fst4_bench: rung-major scheduling (MFSK_FST4_BENCH_DEPTH=rung_major)");
        run_rung_major(refined_bin);
        return;
    }

    // A/B switch for the issue #306 follow-up: is the per-candidate
    // cost tail (6 of 41 candidates, ~14 s each on the first
    // full-depth run) OSD, or the LLR staircase underneath it?
    // `DecodeDepth::BP_ONLY` differs from `::FULL` only in `osd:
    // false` — `llr_effort` doesn't matter for FST4 either way
    // (`LlrEffort` is FT8-only in practice; `process_candidate_
    // basic_impl` always computes every LLR variant regardless, see
    // that enum's own doc comment), so this isolates OSD's
    // contribution the same way WSPR's issue #260 controlled
    // experiment did (`confirmed = None`, short-circuiting before
    // `osd_decode`) — a build-time switch here rather than a runtime
    // one, same idiom as `wspr_bench`'s `MFSK_WSPR_BENCH_WIFI`.
    let depth = if option_env!("MFSK_FST4_BENCH_DEPTH") == Some("bp_only") {
        log::info!("fst4_bench: DecodeDepth::BP_ONLY (MFSK_FST4_BENCH_DEPTH=bp_only) — OSD off");
        DecodeDepth::BP_ONLY
    } else {
        log::info!("fst4_bench: DecodeDepth::FULL (default) — OSD on");
        DecodeDepth::FULL
    };
    // Fourth mode: recall-trade-off follow-up (issue #306). `tests/
    // fst4_sweep.rs`'s `fst4_60_diag_recall_tradeoff` found "drop
    // nsym=8, keep OSD" (`no8_osd`) recovers *more* real AWGN recall
    // than "drop OSD, keep nsym=8" (`bp_only` above) near the
    // crossing, despite `nsym=8` being the far more expensive rung —
    // measures its actual wall-clock here, the same way every other
    // lever this issue has considered was measured before being acted
    // on. `depth` stays `FULL` (OSD on) regardless of the `bp_only`
    // branch above; only `skip_llr_nsym_max` changes.
    let skip_llr_nsym_max = option_env!("MFSK_FST4_BENCH_DEPTH") == Some("no8_osd");
    if skip_llr_nsym_max {
        log::info!(
            "fst4_bench: skip_llr_nsym_max=true (MFSK_FST4_BENCH_DEPTH=no8_osd) — nsym=8 rung off, OSD on"
        );
    }
    log_heap("boot");

    let t_load = now_us();
    let candidates = load_refined_candidates(refined_bin);
    log::info!(
        "fst4_bench: loaded {} refined candidates in {} ms",
        candidates.len(),
        (now_us() - t_load) / 1000,
    );
    log_heap("post-load");

    // The candidate loop runs compute-bound for however long it takes
    // without yielding — same reasoning wspr_bench gives for
    // deinit-ing the watchdog rather than measuring its console
    // traffic.
    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    // `known: &[]` on every call, matching `decode_frame_impl`'s own
    // per-candidate loop exactly — it does not accumulate this pass's
    // own results into `known` as it goes (that parameter exists for
    // *cross-pass* dedup, e.g. SIC rounds feeding an earlier round's
    // decodes into a later one; `known` is threaded through
    // unchanged from the caller for the whole of a single pass).
    // Per-candidate timing, not just the loop total — issue #306
    // follow-up: is the ~90 s roughly uniform across all 41 (something
    // structurally expensive on every candidate, e.g. the
    // `LLR_NSYM_MAX = 8` staircase rung's 65536-hypothesis group
    // running further than it should) or concentrated in the few that
    // reach OSD (WSPR's own shape, issue #260: pass 2 was 76% of that
    // scan for one decode)? Collected into a `Vec` and dumped *after*
    // the timed loop, not logged inline — UART writes inside the timed
    // region would contaminate the very numbers this is trying to
    // measure. `now_us()` itself is two syscalls per candidate,
    // negligible next to what each candidate actually costs here.
    struct CandidateTiming {
        freq_hz: f32,
        us: i64,
        decoded: bool,
    }
    let mut timings: Vec<CandidateTiming> = Vec::with_capacity(41);

    let t0 = now_us();
    let mut results = Vec::new();
    for rc in candidates {
        let freq_hz = rc.cand.freq_hz;
        let precomputed_refine = (rc.cd0, rc.refined_freq_hz, rc.refined_i0, rc.refined_score);
        let t_cand = now_us();
        let decoded = process_candidate_precomputed::<Fst4s60>(
            &rc.cand,
            // Empty, not `fft_cache` — see `run_bench`'s doc comment.
            // Safe: `precomputed_refine` + `skip_snr = true` together
            // mean this call chain never dereferences `fft_cache`.
            &[],
            &FST4_60A_DOWNSAMPLE,
            depth,
            DecodeStrictness::Normal,
            &[],
            EqMode::Off,
            SYNC_Q_MIN,
            precomputed_refine,
            // `skip_snr = true`: FST4's real-SNR formula needs a
            // *second*, independent `downsample_cached` call
            // (`fst4_raw_cs`) that `precomputed_refine` above doesn't
            // cover — the module doc explains why this bench can't
            // serve that FFT either yet (issue #307). `snr_db` on
            // every `DecodeResult` below is `NAN`, not a real
            // measurement; this bench answers the wall-clock question
            // only.
            true,
            skip_llr_nsym_max,
        );
        timings.push(CandidateTiming {
            freq_hz,
            us: now_us() - t_cand,
            decoded: decoded.is_some(),
        });
        if let Some(res) = decoded {
            results.push(res);
        }
    }
    let total_us = now_us() - t0;

    // Sorted slowest-first — the shape (uniform vs. a long tail) is
    // the point, not any single candidate's identity.
    timings.sort_by(|a, b| b.us.cmp(&a.us));
    log::info!("fst4_bench: per-candidate timing, slowest first:");
    for (rank, t) in timings.iter().enumerate() {
        log::info!(
            "    #{rank:>2} {:8.1} Hz  {:>7} ms  decoded={}",
            t.freq_hz,
            t.us / 1000,
            t.decoded,
        );
    }
    let mean_us: i64 = timings.iter().map(|t| t.us).sum::<i64>() / timings.len() as i64;
    let median_us = timings[timings.len() / 2].us;
    log::info!(
        "fst4_bench: per-candidate mean {} ms, median {} ms, max {} ms, min {} ms",
        mean_us / 1000,
        median_us / 1000,
        timings.first().map_or(0, |t| t.us) / 1000,
        timings.last().map_or(0, |t| t.us) / 1000,
    );

    log::info!(
        "fst4_bench: candidate loop (LLR/BP/OSD only, no FFT) TOTAL = {} ms  ({} decodes)",
        total_us / 1000,
        results.len(),
    );
    for r in &results {
        let text = r
            .message77()
            .try_into()
            .ok()
            .and_then(|m77: &[u8; 77]| unpack77(m77));
        log::info!(
            // snr_db is NAN (skip_snr = true, see module doc) — not
            // logged as a number so it can't be mistaken for one.
            "    {:?} | {:.1} Hz | dt {:.2} s | SNR not measured (skip_snr)",
            text,
            r.freq_hz,
            r.dt_sec,
        );
    }
    log::info!("fst4_bench: stack headroom after candidate loop = {} B", stack_headroom());
    log_heap("post-decode");
    log::info!("=== fst4_bench complete ===");
}

/// `MFSK_FST4_BENCH_DEPTH=llr_probe` mode — times `symbol_spectra` and
/// each `compute_llr_*` stage directly, per candidate, with no BP/OSD
/// call at all. Isolates whether the `BP_ONLY` run's ~7.76 s/candidate
/// tail cost is the `LLR_NSYM_MAX = 8` rung specifically (65536
/// tone-combination hypotheses per symbol group) or spread across the
/// staircase — see the module doc's "Follow-up" section for why this
/// is the next thing worth separating out.
///
/// Mirrors `process_candidate_basic_impl`'s own lazy-staircase call
/// order exactly (`engine::pipeline.rs`, same file/line range as the
/// module doc above): `compute_llr_fast` (nsym=1, gives `llra`+`llrd`
/// together, ~5× faster than computing them separately per that
/// function's own doc comment), then `compute_llr_partial` at nsym=2,
/// `LLR_NSYM_MID` (4), and `LLR_NSYM_MAX` (8) in that order — but
/// unconditionally for every candidate here, not lazily stopping at
/// the first variant that would let BP converge, since this probe
/// never calls BP at all and the point is comparing all four stages'
/// costs against each other.
fn run_llr_stage_probe(refined_bin: &[u8]) {
    log_heap("boot");
    let candidates = load_refined_candidates(refined_bin);
    log::info!("fst4_bench: llr_probe over {} candidates", candidates.len());
    log_heap("post-load");

    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    struct StageTiming {
        freq_hz: f32,
        symspec_us: i64,
        fast_us: i64,
        nsym2_us: i64,
        nsym_mid_us: i64,
        nsym_max_us: i64,
    }
    let mut timings: Vec<StageTiming> = Vec::with_capacity(candidates.len());

    let nsym_mid = <Fst4s60 as ModulationParams>::LLR_NSYM_MID
        .expect("FST4-60A sets LLR_NSYM_MID") as usize;
    let nsym_max = <Fst4s60 as ModulationParams>::LLR_NSYM_MAX as usize;

    let t0 = now_us();
    for rc in &candidates {
        let t = now_us();
        let cs = symbol_spectra::<Fst4s60>(&rc.cd0, rc.refined_i0);
        let symspec_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_fast::<Fst4s60, f32>(&cs));
        let fast_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_partial::<Fst4s60, f32, f32>(&cs, 2));
        let nsym2_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_partial::<Fst4s60, f32, f32>(&cs, nsym_mid));
        let nsym_mid_us = now_us() - t;

        let t = now_us();
        core::hint::black_box(compute_llr_partial::<Fst4s60, f32, f32>(&cs, nsym_max));
        let nsym_max_us = now_us() - t;

        timings.push(StageTiming {
            freq_hz: rc.cand.freq_hz,
            symspec_us,
            fast_us,
            nsym2_us,
            nsym_mid_us,
            nsym_max_us,
        });
    }
    let total_us = now_us() - t0;

    timings.sort_by(|a, b| b.nsym_max_us.cmp(&a.nsym_max_us));
    log::info!("fst4_bench: llr_probe per-candidate stage timing, sorted by nsym=8 cost:");
    for t in &timings {
        log::info!(
            "    {:8.1} Hz  symspec {:>5} us  nsym1(fast) {:>6} us  nsym2 {:>6} us  nsym{} {:>6} us  nsym{} {:>7} us",
            t.freq_hz,
            t.symspec_us,
            t.fast_us,
            t.nsym2_us,
            nsym_mid,
            t.nsym_mid_us,
            nsym_max,
            t.nsym_max_us,
        );
    }
    let sum = |f: fn(&StageTiming) -> i64| -> i64 { timings.iter().map(f).sum() };
    let (s_symspec, s_fast, s_2, s_mid, s_max) = (
        sum(|t| t.symspec_us),
        sum(|t| t.fast_us),
        sum(|t| t.nsym2_us),
        sum(|t| t.nsym_mid_us),
        sum(|t| t.nsym_max_us),
    );
    log::info!(
        "fst4_bench: llr_probe totals over {} candidates — symspec {} ms, nsym1 {} ms, nsym2 {} ms, nsym{} {} ms, nsym{} {} ms, stage-sum {} ms, wall {} ms",
        timings.len(),
        s_symspec / 1000,
        s_fast / 1000,
        s_2 / 1000,
        nsym_mid,
        s_mid / 1000,
        nsym_max,
        s_max / 1000,
        (s_symspec + s_fast + s_2 + s_mid + s_max) / 1000,
        total_us / 1000,
    );
    log::info!("fst4_bench: stack headroom after llr_probe = {} B", stack_headroom());
    log_heap("post-probe");
    log::info!("=== fst4_bench (llr_probe) complete ===");
}

/// `MFSK_FST4_BENCH_DEPTH=rung_major` mode -- issue #306 item 3 (VK3NV),
/// step 3 of the two-step host-then-device plan
/// (`tests/fst4_sweep.rs::fst4_60_diag_rung_major_scheduling` did steps
/// 1-2 host-side). Calls the real, host-verified
/// `mfsk_core::fst4::rung_major::decode_rung_major` (correctness +
/// AWGN/CCIR-moderate recall checked against production in
/// `tests/fst4_sweep.rs::fst4_60_diag_decode_rung_major_correctness`)
/// instead of reimplementing the stage sequence in this bench a second
/// time -- the earlier hand-rolled version here (see git history) is
/// what that real implementation replaced, after review found its first
/// draft only approximated production's variant sequence closely enough
/// to demonstrate the *concept*, not closely enough for a comparable
/// total.
///
/// `decode_rung_major` drops `llrd` unconditionally (five stages:
/// llra/llrb/llre/llrc/OSD, not six) -- a targeted ablation found it
/// never contributes additional recall on real AWGN or CCIR-moderate
/// data (`fst4_60_diag_stage_ablation[_ccir_moderate]`), so it isn't
/// scheduled at all rather than being reordered. `skip_llrc` (the
/// `no8_osd` trade-off) is a real caller choice, not free -- this mode
/// runs both `false` and `true` so the two totals are directly
/// comparable to `full` (43.134 s) and `no8_osd` (17.147 s) above.
///
/// Also runs [`mfsk_core::fst4::rung_major::decode_phase_split_timed`]
/// (`Schedule::PhaseSplit`, `budget_ok = None`) over the same candidate
/// set immediately after -- issue #310's own implementation note
/// flagged this real-hardware comparison as not yet done, and #327
/// names it as the prerequisite for the dual-core dispatch design
/// (whether Phase A's cost is uniform enough on real hardware for plain
/// work-steal). Decode counts must match the `rung_major[label]` runs
/// above exactly; only per-stage order (and therefore the wall-clock
/// total under a hypothetical mid-run budget cutoff) differs.
fn run_rung_major(refined_bin: &[u8]) {
    use mfsk_core::fst4::Fst4s60;
    use mfsk_core::fst4::rung_major::RungMajorCandidate;

    log_heap("boot");
    let candidates = load_refined_candidates(refined_bin);
    log::info!("fst4_bench: rung_major over {} candidates", candidates.len());
    log_heap("post-load");

    let r = unsafe { esp_idf_svc::sys::esp_task_wdt_deinit() };
    log::info!("task watchdog deinit -> {r}");

    let inputs: Vec<RungMajorCandidate> = candidates
        .into_iter()
        .map(|rc| RungMajorCandidate {
            cand: rc.cand,
            cd0: rc.cd0,
            refined_freq_hz: rc.refined_freq_hz,
            i0: rc.refined_i0,
        })
        .collect();

    // Third combination (skip_llrc=true, skip_osd=true) answers VK3NV's
    // issue #306 item 1 follow-up directly: `no8_osd`'s own BP-only
    // total, not a subtraction from `full`'s split (dropping `llrc`
    // changes how often BP fails and OSD is even reached, so `full`'s
    // 71.9%/28.1% split isn't guaranteed to carry over).
    for (skip_llrc, skip_osd, label) in [
        (false, false, "full"),
        (true, false, "no8_osd"),
        (true, true, "no8_osd_bponly"),
    ] {
        let t0 = now_us();
        let results = mfsk_core::fst4::rung_major::decode_rung_major_timed::<Fst4s60>(
            &inputs, skip_llrc, skip_osd, &[0], None, 12_000.0,
        )
        .0;
        let total_us = now_us() - t0;
        let decoded_count = results.iter().flatten().count();
        log::info!(
            "fst4_bench: rung_major[{label}] TOTAL = {} ms ({} decodes)",
            total_us / 1000,
            decoded_count,
        );
        for r in results.iter().flatten() {
            let text = r
                .message77()
                .try_into()
                .ok()
                .and_then(|m77: &[u8; 77]| unpack77(m77));
            log::info!("    {:?} | {:.1} Hz | dt {:.2} s", text, r.freq_hz, r.dt_sec);
        }
        log_heap(&alloc::format!("post-rung-major-{label}"));
    }

    // Real-hardware `Schedule::PhaseSplit` comparison — the item #310's
    // own implementation comment flagged as queued ("Both schedules are
    // reachable from one build via fst4_bench, deliberately, so a CoreS3
    // session can measure them side by side") and #327 later named as
    // the direct prerequisite for the dual-core dispatch design (Phase
    // A's real-hardware uniformity determines whether simple work-steal
    // suffices there). `budget_ok = None` runs the whole ladder, so
    // decode counts must match `decode_rung_major_timed`'s above exactly
    // — only per-stage *order*, and therefore the wall-clock total under
    // a hypothetical mid-run cutoff, differs. Same `inputs`, same
    // (skip_llrc, skip_osd) grid, so each label is directly comparable
    // to its `rung_major[label]` counterpart above.
    for (skip_llrc, skip_osd, label) in [
        (false, false, "full"),
        (true, false, "no8_osd"),
        (true, true, "no8_osd_bponly"),
    ] {
        let t0 = now_us();
        let results = mfsk_core::fst4::rung_major::decode_phase_split_timed::<Fst4s60>(
            &inputs, skip_llrc, skip_osd, &[0], None, None, 12_000.0,
        )
        .0;
        let total_us = now_us() - t0;
        let decoded_count = results.iter().flatten().count();
        log::info!(
            "fst4_bench: phase_split[{label}] TOTAL = {} ms ({} decodes)",
            total_us / 1000,
            decoded_count,
        );
        for r in results.iter().flatten() {
            let text = r
                .message77()
                .try_into()
                .ok()
                .and_then(|m77: &[u8; 77]| unpack77(m77));
            log::info!("    {:?} | {:.1} Hz | dt {:.2} s", text, r.freq_hz, r.dt_sec);
        }
        log_heap(&alloc::format!("post-phase-split-{label}"));
    }

    log::info!(
        "fst4_bench: stack headroom after rung_major/phase_split = {} B",
        stack_headroom()
    );
    log::info!("=== fst4_bench (rung_major + phase_split) complete ===");
}

/// Stack for the bench task. Unmeasured starting point, not a
/// right-sized figure: `wspr_bench`'s own `SCAN_STACK` (72 KiB) was
/// only reached after finding a 128 KiB reservation and a stack
/// overflow inside `osd_decode_packed` on the way there. FST4 shares
/// the same generic LDPC/BP/OSD machinery FT8 already runs on-device
/// (`Ldpc240_101` vs FT8's `Ldpc174_91`, both `BpPooledFec`), so a
/// comparable order of magnitude is a reasonable starting guess — but
/// it is a guess. The first real run's `stack headroom after
/// candidate loop` log line is what turns this into a measured value;
/// resize once that number exists rather than trusting this one.
pub const BENCH_STACK: u32 = 96 * 1024;

extern "C" fn bench_task(arg: *mut core::ffi::c_void) {
    // SAFETY: `run` leaks a `&'static &'static [u8]` that outlives
    // this task, mirroring wspr_bench's own arg-passing shape for
    // `extern "C"` task entries.
    let refined_bin: &'static [u8] = unsafe { *(arg as *const &'static [u8]) };
    run_bench(refined_bin);
    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}

/// `refined_bin` is the baked golden asset (`include_bytes!`). Spawns
/// [`bench_task`] on a dedicated stack (see [`BENCH_STACK`]'s doc
/// comment on why that size is a starting guess) pinned to core 0,
/// then idles forever.
pub fn run(refined_bin: &'static [u8]) -> ! {
    esp_idf_svc::sys::link_patches();
    init_logger_once();

    let arg: &'static &'static [u8] = alloc::boxed::Box::leak(alloc::boxed::Box::new(refined_bin));
    let argp = arg as *const _ as *mut core::ffi::c_void;

    let created = unsafe {
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(bench_task),
            c"fst4_bench".as_ptr(),
            BENCH_STACK,
            argp,
            5,
            core::ptr::null_mut(),
            0,
        )
    };
    if created != 1 {
        log::error!("failed to create fst4_bench task ({BENCH_STACK} B stack)");
    }

    loop {
        unsafe { esp_idf_svc::sys::vTaskDelay(1000) };
    }
}
