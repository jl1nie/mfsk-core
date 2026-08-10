# Changelog

## 0.9.0 (unreleased) — streaming ergonomics for host UIs

Theme for this cycle: make the streaming decode surface easier to
build desktop/host UIs on. Redundant-processing cleanup from recent
PRs rides along as internal hygiene (it shrinks the surface these
additive APIs sit on); it is not the headline and has no behavioural
effect on any decode path.

### Added

- **Streaming decode delivery: `.on_result(cb)` / `*_streaming` siblings**
  (issue #204; PRs #237/#239/#240) — a synchronous callback fired once
  per accepted decode result, delivered as candidates resolve instead
  of only after the whole slot finishes. Purely additive: every
  existing batch `decode()`/`decode_scan(...)` call keeps returning its
  full `Vec`/`DecodeOutcome` unchanged, callers who don't opt in see
  zero difference. Deliberately a plain `Fn(&Result) + Sync` callback,
  not async/a channel/Tokio — `engine`/`protocol` stay executor-free so
  the `no_std` embedded targets that are first-class consumers of this
  same decode path keep working; a host wanting cross-thread delivery
  (e.g. into a GUI) wraps the callback itself. Design rationale and the
  two delivery contracts (sequential exact-match vs. parallel
  completion-order-with-possible-transient-duplicate) are written up in
  the new `docs/reference/STREAMING.md`/`.ja.md`, including a worked
  Tokio `spawn_blocking` + `mpsc` example.

  - **FT8**: `DecodeRequest`/`SniperRequest::on_result`, plus a new
    `decode_block_streaming` sibling to `decode_block` — originally
    embedded-only (`#[cfg(not(feature = "fft-rustfft"))]`), given a
    host `fft-rustfft` sibling with the same signature and contract
    later in this same cycle (issue #243's follow-up, see Fixed
    below). Threaded through all four
    decode strategies (`.decode()`/sniper/`.sic_rounds()`/
    `.sic_early()`). On the two `rayon`-parallel strategies the callback
    fires inside the per-candidate closure, before the later
    cross-candidate dedup pass — documented as a possible transient
    duplicate the returned `Vec` later excludes. On the three
    sequential/SIC strategies the fire point already is final
    acceptance, so delivery is an exact match with zero divergence.
    Candidates are visited in Costas-sync-score-descending order on
    every strategy (`coarse_sync` sorts before returning), so results
    also *tend* to favor strong signals first — a correlation, not a
    guarantee (sync score isn't a direct predictor of post-demod
    BP/OSD cost, and the sequential strategies still suffer
    head-of-line blocking behind a high-scored-but-OSD-needing
    candidate).
  - **Q65**: `.on_result()` on `DecodeRequest`, `SniperRequest`, and
    `MultiPeriodRequest` (`src/q65/decode_request.rs`) — the first two
    fire per accepted candidate (exact-match, sequential, no early
    exit); `MultiPeriodRequest` fires once per *slot* that yields an
    accepted decode, the natural streaming unit for its multi-period
    EME/ionoscatter averaging. `SniperRequest` fires 0-or-1 times
    (single-candidate decode, no loop to stream across) — kept for
    builder-API consistency, documented as such.
  - **WSPR/JT9/JT65**: none of the three have a builder API (plain `pub
    fn` families), so — matching FT8's own `decode_block`/
    `decode_block_streaming` precedent — each gains a non-breaking
    `decode_scan_streaming` sibling instead of a new parameter on the
    existing function. WSPR additionally gets
    `decode_scan_subtract_streaming` (fires at the outer SIC-pass
    accept point only, not inside each pass's internal `decode_scan`
    call). JT65/JT9's loops are sequential with no parallelism, so
    their streamed delivery is an exact match against the batch `Vec`,
    same order, with no divergence mechanism.
  - **`mfsk-ffi`: `mfsk_decode_i16_streaming`** — the streaming surface
    above was never exposed across the C ABI at all (found while
    auditing the FFI layer, issue #246 follow-up). FT8 only for now
    (other protocols return `MfskStatus::UnknownProtocol`); additive
    to `mfsk_decode_i16` in every sense `DecodeRequest::on_result`
    already establishes (`out` still gets the full batch list,
    `callback` is optional). First function in this crate to run
    caller-supplied code mid-decode, so it's also the first to need
    `catch_unwind` (a Rust panic must not unwind across an `extern
    "C"` boundary) and a `Sync`-asserted `user_data` wrapper (the
    parallel/rayon strategy invokes the callback from worker threads).
    Verified via `tests/streaming_ffi.rs` and a new `cpp_smoke`
    section exercising the real generated header from compiled C++.
  - **`mfsk-ffi`: `MfskDecodeOptions` builder parity** (issue #162
    follow-up) — `MfskDecodeOptions` hadn't grown a single setter
    since its creation (issue #205), despite its own doc comment
    anticipating exactly that; `mfsk_core`'s `DecodeRequest` builder
    grew `.strictness()`/`.eq_mode()`/`.freq_hint()`/`.ap_hint()`
    (FT8)/`.sic_rounds()`/`.sic_early()` over many sessions with zero
    FFI follow-through. New setters, each mutating an existing
    `MfskDecodeOptions` handle in place (one Rust builder-chain call ↔
    one C setter call before the handle is passed to a decode
    function): `mfsk_decode_options_set_strictness`/`_eq_mode`/
    `_freq_hint` (FT8/FT4/FST4-60A), `_sic_rounds`/`_sic_early`
    (FT8, `_sic_rounds` also FT4), `_ap_hint` (FT8 only, wide-band —
    reuses `mfsk_q65_decode_with_ap`'s existing 4-string convention via
    a new shared `build_ap_hint_from_cstrs` helper). Wired into every
    relevant `mfsk_decode_i16`/`_f32` branch and
    `mfsk_decode_i16_streaming`. New two new enums, `MfskStrictness`/
    `MfskEqMode` (`mfsk-ffi-abi`, alongside the existing
    `MfskDecodeDepth`). Verified with real recall-gain proofs on
    `qso3_busy.wav` for `sic_rounds`/`sic_early` (more stations than
    default) and `ap_hint` (reuses `mfsk-core`'s own already-proven
    AP-on gain, strict-superset + a known JTDX extra) — not just
    round-trip wiring checks. `.known()` (#247) and `SniperRequest`
    exposure (#249) are deliberately still deferred (each is a real
    design question of its own, not just a missing setter);
    `.fft_cache()` reuse (#248) was closed outright (no consumer
    materialized).
  - **`mfsk-ffi`: Q65 callsign hash-table exposure** (issue #250,
    the builder-parity pass's one deferred item that did ship). New
    opaque `MfskCallsignHashTable` handle
    (`mfsk_callsign_hash_table_new`/`_insert`/`_free`, same
    handle-pair shape as `MfskDecodeOptions`) mirroring
    `q65::DecodeRequest::hash_table`'s `Arc<CallsignHashTable>` —
    resolves `<...>` Type-4 hashed-callsign placeholders in decoded
    message text. Threaded through all four `mfsk_q65_decode_*`
    functions as a new optional trailing parameter (before `out`);
    NULL keeps the pre-#250 unresolved-placeholder behaviour, a
    breaking signature change for existing callers of those four
    functions (recompile + pass NULL to keep current behaviour).
    Deliberately not folded into `MfskDecodeOptions` — Q65's function
    family doesn't use that type at all. Verified with the same
    differential-test shape as `q65::rx`'s own Rust-side hash-table
    test: a Type-4 message built directly via `mfsk_core`'s Rust API
    (`mfsk_encode_q65` only packs standard messages) decodes to a
    literal `<...>` with `hash_table = NULL` and to the resolved
    `<JA1ABC>` once the same callsign is registered via
    `mfsk_callsign_hash_table_insert`.
  - **Bug, found and fixed same day (2026-08-08): `on_result` silently
    never fired for `Ft4`/FST4.** `on_result` is a field on the shared
    `DecodeRequest`/`SniperRequest<P>` structs (issue #191's generic
    builder, so it type-checked and built for any `P: FrameDecodable` —
    `Ft8` and every `Ft4`/FST4 sub-mode), but the doc comment on
    `DecodeRequest::on_result` promised delivery unconditionally, with
    no protocol scoping (unlike e.g. `ap_hint`, which does document
    itself as "FT8 only"). Only FT8's `FrameDecodable` impl actually
    read the field; `Ft4`/FST4's `__single_pass`/`__sniper`/
    `__flat_sic` never passed `req.on_result` down into
    `engine::pipeline::decode_frame`/`decode_frame_subtract`/
    `msg::pipeline_ap::decode_sniper_ap` — calling `.on_result(cb)` on
    an `Ft4`/FST4 `DecodeRequest` compiled cleanly and silently never
    invoked `cb`. No compile error (the field genuinely is read, by
    FT8's impl, so `dead_code` never fires for a struct-level field
    across every generic instantiation), and no test caught it: the
    entire test suite's `.on_result(` call sites were `ft8_streaming_
    decode.rs` and `q65_wsjtx_samples.rs` only — nothing exercised the
    `Ft4`/FST4 combination that silently broke the contract.

    Fixed by threading `on_result: Option<&(dyn Fn(&DecodeResult) +
    Sync)>` through all three call sites (`decode_frame_impl`'s two
    `filter_map` closures — same "before dedup, possible transient
    duplicate" contract as FT8's own parallel single-pass strategy;
    `decode_frame_subtract`'s per-pass `all_results.extend(deduped)`
    point — sequential exact-match, same as FT8's `.sic_rounds()`;
    `decode_sniper_ap`'s `results.push(r)` point — sequential,
    0-or-1+ depending on the AP early-exit) and wiring `req.on_result`
    through from `Ft4`'s and every FST4 sub-mode's `FrameDecodable`/
    `SupportsSicRounds` impls. New regression coverage:
    `tests/ft4_streaming_decode.rs` (all three FT4 strategies —
    single-pass, `.sic_rounds()`, sniper) and
    `tests/fst4_streaming_decode.rs` (FST4 has no SIC path, issue
    #193, so only single-pass applies); the sniper test is also the
    first in the suite to exercise `SniperRequest::on_result` for
    *any* protocol, FT8 included. Byte-identical recall verified via
    the existing golden-WAV suites (FT4 6/6, FST4-60A 1/1) plus the
    full `scripts/pre-push-check.sh` matrix.

  - **Same-day audit turned up a second, lower-severity instance of the
    same root cause: `DecodeRequest::fft_cache` was also a silent
    `Ft4`/FST4 no-op.** Unlike `on_result` this one didn't break
    correctness — the fix's own original 0.8.0-era CHANGELOG entry
    (`### Added`, `#191`, above) already called it out in passing
    ("`fft_cache` is reused where the underlying engine's buffer shape
    permits (FT8 single-pass/flat pass 0); elsewhere it's a documented
    no-op degrade… since `core::pipeline`'s generic engine has no
    cache injection point") — but that caveat was never propagated
    into `DecodeRequest::fft_cache`'s own doc comment (the surface a
    docs.rs/IDE-hover reader actually sees), unlike `ap_hint`'s
    equivalent "`Ft8` only" note on the same struct. Gave
    `engine::pipeline::decode_frame`/`decode_frame_subtract` the cache-
    injection point they'd never had (`decode_frame_subtract` only
    trusts it for pass 0 — later passes' residual has been mutated by
    subtraction, so their cache must always be rebuilt regardless) and
    wired `req.fft_cache` through the same `Ft4`/FST4 call sites.
    `SniperRequest` has no `fft_cache` field at all, so
    `decode_sniper_ap` is unaffected. New coverage:
    `tests/ft4_fft_cache_decode.rs` / `fst4_fft_cache_decode.rs` — a
    same-audio round-trip can't distinguish "reused" from "silently
    rebuilt" (both give the same answer when the audio hasn't
    changed), so these use a differential test instead: hand the
    decoder a real-but-*wrong* `FftCache` built from silence and
    assert the golden message stops decoding, proving the supplied
    cache is actually consumed rather than quietly discarded.

- **`msg::decoded::Decoded`** — a unified, owned, human-readable decode
  row for host UIs, plus a `to_decoded(..)` conversion on every
  protocol's native result type (`engine::pipeline::DecodeResult` for
  FT8/FT4/FST4, `WsprResult`, `Q65Result`, `Jt65Result`, `Jt9Result`).
  Each native result is structurally distinct with a different
  message-text path (FT8-family `unpack77`, `Display` for WSPR/JT65/JT9,
  an already-resolved `String` for Q65); `Decoded` resolves that once at
  the conversion boundary into the columns a decode list binds to for
  *every* mode — `text` / `freq_hz` / `dt_sec` / `snr_db` / `protocol`
  (`ProtocolId`). Owned (`Clone` + `Send`), so it drops straight into a
  channel from inside a streaming `.on_result` callback without the
  hand-rolled per-mode extraction the Tokio example in
  `docs/reference/STREAMING.md` previously spelled out.

  Additive, not a replacement: the native result types stay and keep
  their mode-specific diagnostics (`sync_score`, `hard_errors`,
  `iterations`, …). `Decoded` deliberately carries only the cross-mode
  intersection — those extras don't generalise into clean shared
  columns. Conversion signatures differ where the modes genuinely do:
  FT8/FT4/FST4 is fallible (`Option<Decoded>` — an unpack failure yields
  no row) and takes the caller's `ProtocolId` + an optional
  `CallsignHashTable`; WSPR is infallible; Q65/JT65/JT9 take
  `(sample_rate, nominal_start_sample)` to derive `dt_sec` from their
  sample-index `start_sample`. Design note: `docs/notes/DECODED_ROW.md`.

- **`serde` feature** (off by default) — derives `Serialize`/
  `Deserialize` on `Decoded` and `ProtocolId`, `no_std`-clean via
  serde's `alloc` feature, so a UI can emit decode rows as JSON for
  spotting / websocket / IPC. Folded into `full`. Purely additive.

- **JT65 stochastic Chase decoder** (`jt65::chase`, issue #169) — a
  faithful port of WSJT-X's `ftrsdap` stochastic Chase decoder,
  literal hand-tuned magic numbers included (not just the algorithmic
  shape — an initial same-day pass shipped a simplified approximation,
  then was rewritten for literal fidelity on request):
  `decode_at_with_chase`/`decode_scan_chase`/`_streaming`/`_default`,
  fully additive siblings of the existing `decode_at_with_erasures`/
  `decode_scan*` family (zero changes to any existing function or
  signature). Ported: WSJT-X's `perr[8][8]` erasure-probability table
  (`ftrsdap.c`, ×1.3 scale, keyed by confidence-ratio and reliability-
  rank buckets); the real `getpp` candidate-quality metric (re-encodes
  each successful trial's codeword, walks it through the same
  interleave+Gray-encode the transmitter uses, and averages the
  *original* raw FFT-bin power at the resulting positions — needed
  retaining the full 63×64 pre-decision spectrum through the demod
  step, `rx::demodulate_aligned_with_runnerup`, ~16 KB, not an
  embedded/no_std concern since JT65 already requires `std`); the
  literal `nhard`/`nsoft`/`ntotal` soft-distance formula using a
  newly-retained runner-up-tone identity; the literal acceptance gate
  (`ntotal ≤ nd0(81) && pp2/pp1 ≤ r0(0.87)`, tracking best/second-best
  candidate quality directly across all trials rather than a per-
  message tally — WSJT-X doesn't dedup by message at all); the literal
  `nhard ≤ 41 && ntotal ≤ 71` early exit; the exact LCG/`ir`-extraction
  RNG (a POSIX-style recurrence already present test-only in
  `fec/ldpc/bp.rs`, promoted to production use here); and WSJT-X's own
  `jt9 -6` trial count (`1000`, via `decoder.f90`'s
  `nranera=6 → ntrials=10**(6/2)=1000` formula — the initial pass had
  guessed `2000` with no real derivation). Not ported: AP-hint passes
  and the `hint65` correlation fallback (out of scope, no request for
  them). Covered by two always-run (not `#[ignore]`d) false-decode-rate
  tests, each asserting exactly zero false decodes (not a tolerance)
  across 20 seeds of pure noise and 20 seeds of a signal synthesized
  well below the measured sensitivity floor — re-verified against the
  ported acceptance gate.

  Measured on the existing `tests/jt65_sweep.rs` AWGN corpus (300
  files, 20 trials/SNR) at the faithful port's literal 1000-trial
  default, before the reliability-metric fix below: 50% recall
  crossing moved from −14 dB to ≈ −18.3 dB.

  **Follow-up fix, same day, user-prompted**: a challenge over the
  still-sizeable residual gap — reasonable to suspect something was
  being overlooked — turned up a real bug. The "faithful" pass above
  had reused this
  crate's pre-existing `conf` metric (`(best−second)/best`, a top-2
  -tone-only margin) everywhere WSJT-X's real `demod64a.f90` uses a
  materially different quantity: `mrprob = best_pwr/total_pwr` summed
  over *all 64* tones at a position, not just the top two — an
  SNR-like peakiness measure that reflects the local noise floor
  across the whole spectrum, which `conf` can't see (two positions
  with an identical top-2 margin can have very different `mrprob`
  depending on how much power sits in the other 62 tones). This fed
  the wrong quantity into both the erasure-priority ordering and the
  `nsoft` soft-distance weighting. Fixed by adding a `rel` field
  (`best_pwr/total_pwr`, cheap — `total_pwr` was already computed for
  the SNR estimate) to `rx::demodulate_aligned_with_runnerup` and using
  it everywhere WSJT-X uses `rxprob`/`mrprob`; `conf` was still correct
  and unchanged for the `PERR` table's ratio bucket, since
  `rxprob2/rxprob` algebraically reduces to `second_pwr/best_pwr`
  regardless of the `psum` normalization.

  50% recall crossing after this fix: −14 dB → ≈ −19.1 dB (~5.1 dB of
  the ~7-8 dB gap), ~2-3 dB still remaining at the deepest cells.

  **Second follow-up, same day, further user pushback**: still not
  satisfied with the residual gap, the user asked for a phase-by-phase
  comparison against WSJT-X's *real* pipeline, not just `ftrsdap.c` in
  isolation. That comparison (`decode65a.f90` → `decode65b.f90` →
  `extract.f90`) found the actual dominant cause, unrelated to
  `ftrsdap`: WSJT-X applies a continuous (non-quantized) frequency+
  drift correction to the *time-domain* signal (`afc65b.f90`'s
  chi-square fit + `twkfreq65.f90`'s phase-continuous correction)
  before any FFT. This crate's JT65 demod had no equivalent at all —
  every candidate frequency got rounded to the nearest FFT bin and
  stopped there. A rectangular window pays a well-known worst-case
  ≈3.9 dB "scalloping loss" for a tone sitting exactly half a bin off
  center — and this crate's own AWGN sweep golden frequency, 1500 Hz,
  sits at *exactly* bin 557.5 (NSPS=4460 @ 12 kHz ⇒ 2.6906 Hz/bin),
  the worst possible case, on every single trial in the corpus.
  Confirmed directly: a throwaway A/B probe (identical synth signal +
  identical noise, differing only in whether the frequency landed on a
  bin center) measured 100% vs. 43-50% recall at identical SNRs — the
  true scale of "what was being missed," bigger than anything found
  inside `ftrsdap` itself.

  Fixed with two additive pieces that apply to *every* JT65 decode
  path (not just chase, since they all share `search`/`rx`):
  `search::refine_freq_hz` (3-point log-power parabolic "Jacobsen"
  interpolation of the sync-tone power around the coarse bin, reusing
  the already-built spectrogram, no extra FFTs) gives candidate
  frequencies genuine sub-bin precision; a running-phase NCO in
  `rx::demodulate_aligned_with_confidence_inner` (same accumulator
  pattern as `engine::dsp::subtract`'s NCO loops) cancels the residual
  sub-bin offset on the audio before each symbol's FFT, phase-
  continuous across the whole 126-symbol frame — mirroring WSJT-X's
  `twkfreq65` (without the drift term; this crate's search doesn't
  estimate drift, and the AWGN corpus has none). Exactly a no-op for
  any caller already passing a bin-aligned frequency.

  **Final measured result**: plain `decode_at_with_erasures`'s 50%
  crossing moved −14 dB → ≈ −21.8 dB — with **zero code changes to
  that function itself**, purely by sharing the fixed `search`/`rx`.
  `decode_at_with_chase`'s moved to ≈ −23.8 dB. Both now at or beyond
  the previously-cited WSJT-X `jt9 -6` reference floor (~100% to
  −22 dB) — treated with real caution, not declared as "beats
  WSJT-X": that reference figure's own provenance wasn't independently
  re-verified against a real `jt9` binary this session, and the
  corpus's −25 dB floor is no longer deep enough to fully characterize
  either decoder (chase already shows partial recall there). The
  honest, load-bearing conclusion: **the originally-diagnosed ~7-8 dB
  gap is gone** on this crate's AWGN corpus — issue #169 can reasonably
  be considered closed as filed. Full before/after tables and the
  complete phase-by-phase writeup: `docs/notes/BENCHMARKS.md`'s JT65
  section.

- **JT9: configurable decode depth (`Jt9Depth`)** — found while
  profiling why JT9's candidate loop dominates real-recording decode
  time (`jt9::tests::candidate_loop_stage_diag`): `ConvFano232::
  decode_soft` is ~92% of it, almost entirely non-converging
  candidates burning the full `max_cycles_per_bit` budget before
  giving up (a real signal converges in microseconds regardless of the
  budget). WSJT-X's own `jt9_decode.f90` doesn't use one flat budget
  either — it escalates `limit=5000` (its own automatic-scan default,
  `-d1`) → `10000` (`-d2`) → `30000` (`-d3`) → `100000` ("Decode
  Again"). `Jt9Depth::{Fast,Normal,Deep,Max}` exposes the same four
  tiers; new `decode_scan_with_depth`/`decode_scan_streaming_with_depth`/
  `decode_at_baseband_with_fft_depth` siblings (existing `decode_scan`/
  `decode_scan_streaming` unchanged, `Jt9Depth::default()` = `Normal`
  = 10 000, this crate's pre-existing value, so default behavior is
  identical to before).

  Checked whether the default should move to `Fast` (WSJT-X's own
  `-d1`) by running a real `jt9 -9` build (defaults to `-d1`) against
  the same 20-file-per-SNR AWGN corpus `tests/jt9_sweep.rs` uses:
  real `jt9 -d1` scores 70%/10% (14/20, 2/20) at −26/−27 dB; this
  crate at `Fast` (5 000, matching WSJT-X's cycle budget exactly) gets
  60%/5%, at `Normal` (10 000) gets 65%/10% — so `Fast` would trail
  real `jt9` by *more* at both points, not close a gap to it, and
  `Normal` already falls ~5 points short of real `jt9 -d1` at −26 dB
  even at double its cycle budget (a small, separate, not-yet-
  investigated sensitivity gap). Kept `Normal` as the default.
  `docs/notes/BENCHMARKS.md`'s JT9 section has the full numbers and
  corrects a stale "80% at −26 dB" figure for real `jt9` to a freshly
  re-verified 70%.

- **JT9: fixed the small sensitivity gap above (task #24), and it
  wasn't cycle-budget-related at all.** Root-caused via two
  `jt9::decode::gate_diag` probes on the specific AWGN files this
  crate missed but real `jt9 -d1` decoded: `jt9::search::coarse_search`'s
  frequency grid is one bin wide (~1.736 Hz, exactly the tone spacing)
  with no sub-bin refinement, so candidates routinely landed 0.3-1 Hz
  off the true frequency — inside Fano's sharp non-convergence zone
  regardless of `Jt9Depth`. The same "coarse bin center isn't close
  enough, and nothing downstream recovers it" shape as JT65's own
  scalloping-loss fix (issue #169) — fixed with the identical
  technique, ported directly: `search::refine_freq_hz`, 3-point
  log-power parabolic interpolation of the sync-tone power across the
  already-built spectrogram (no extra FFTs, no new algorithm).

  Result: this crate now **exceeds** real `jt9 -d1` at both points
  that used to trail it — −26 dB 65%→**85%** (13/20→17/20), −27 dB
  10%→**25%** (2/20→5/20). Golden WAV recall unchanged (7/7). With the
  bug gone, `Jt9Depth::Fast` and `Normal` score *identically* on the
  full sweep — the extra cycle budget `Normal` was kept for above
  turned out to have been silently compensating for this bug, not
  buying real sensitivity — so `Jt9Depth::default()` moved to `Fast`
  (same result, half the non-converging-candidate cost). Golden-WAV
  `decode_scan` wall time: ~302 ms → **~159 ms**, roughly halved again
  on top of the `realfft` win earlier this file. Full writeup:
  `docs/notes/BENCHMARKS.md`'s JT9 section.

- **Re-verified FT8/FT9's sibling sensitivity claims against real
  binaries, not just published figures — found FT4 and FST4's
  documented "gaps" don't reproduce live, and found a genuine,
  root-caused gap in JT65.** Prompted by "does JT9's fix mean
  everything is now at parity?" — checked.

  - **FT4/FST4**: the previously-documented AWGN gaps (FT4 ~0.6 dB,
    FST4 0.10-0.60 dB across sub-modes) were measured against WSJT-X's
    *published* sensitivity figures, never against a live binary on
    the same corpus. Ran real `jt9 -5`/`jt9 -7` directly against
    `tests/ft4_sweep.rs`/`tests/fst4_sweep.rs`'s own AWGN corpora:
    FT4 this crate ≈−16.89 dB vs. real `jt9 -5` ≈−16.75 dB (crate
    slightly ahead, not 0.6 dB behind); FST4-60 **exact match** at
    both tested SNR points (75%/75%, 30%/30%); FST4-120 ≈−30.71 dB vs.
    ≈−30.67 dB (real, live) — a ~0.04 dB difference, noise-level at
    20 trials/point. Both FT4's and FST4's AP paths require `mycall`
    ≥3 chars (confirmed in `lib/ft4_decode.f90`/`lib/fst4_decode.f90`),
    so the bare-CLI real-binary runs are genuinely AP-free, an
    apples-to-apples comparison. Status upgraded to "at/above parity"
    for both in `docs/notes/BENCHMARKS.md`'s Summary table.
  - **JT65**: same check, opposite result — real `jt9 -6` scores
    50% (10/20) at −25 dB vs. this crate's chase decoder's 15%
    (3/20) on the identical `jt65_sweep` corpus, a real gap issue
    #169's closure didn't catch (that comparison was never checked
    against a live binary either, until now). **Initially misdiagnosed
    as a free CQ-AP-hypothesis asymmetry** (`lib/extract.f90`
    unconditionally *populates* a CQ AP entry) — corrected on closer
    reading: that entry is only ever *consulted* when `npass>1`, which
    itself requires non-empty `mycall` (`lib/extract.f90:142-150`) —
    JT65's AP path gates on `mycall` length exactly like FT4/FST4's,
    so the bare-CLI comparison above is genuinely AP-free on both
    sides. Also checked and ruled out: WSJT-X's real trial count at
    the CLI default depth is 100 (`jt65_decode.f90:106-119`), not the
    1000 this crate's `ChaseParams` assumed — yet real `jt9` still
    wins with 10× fewer trials, so trial count isn't it either. A
    direct probe (tried every `coarse_search` candidate, both
    `decode_at_with_erasures` and `decode_at_with_chase`, on the 7
    files real `jt9` decodes but this crate misses) found the correct
    candidate (exact frequency/timing) present in every file's
    candidate list — ruling out a repeat of JT9's coarse-frequency-grid
    issue — yet decode fails at every candidate regardless: the real
    gap is downstream, in demod confidence quality or `chase.rs`'s own
    `ftrsdap` calibration, not yet isolated. Needs the same
    phase-by-phase methodology that closed issue #169 originally;
    tracked as task #26, not attempted further this session. Full
    numbers and source citations: `docs/notes/BENCHMARKS.md`'s JT65
    section.

### Changed

- **wasm32 builds now enable rustfft's `wasm_simd` feature** — unlike
  `avx`/`sse`/`neon` on their native targets (auto-detected by rustfft
  at compile time, no flag needed), `wasm_simd` is a separate opt-in
  that the plain `rustfft` dependency did not previously enable, despite
  a stale comment in `Cargo.toml` claiming otherwise. Measured via a
  fresh A/B in `bench/wasm/` under `wasm32-unknown-unknown` + Node:
  ~15-17% faster on FT8's default decode path, ~23-30% faster on
  `.sic_early()` (the FFT-heavier multipass strategy — a `node --prof`
  breakdown showed ~66% of its wall-clock in FFT/subtract). Recall
  unchanged. No caller action needed — reaches every wasm32 consumer of
  mfsk-core via Cargo feature unification. Follow-up to issue #246.
- **`engine::sync::coarse_sync` no longer heap-allocates inside its
  hottest loop.** `fill_sync2d_row!`'s per-(freq-bin, lag)-cell
  accumulators (`t_blocks`/`t0_blocks`, `sync.rs`) were a fresh
  `vec![0.0f32; num_blocks]` pair allocated on every one of the
  `n_freq × (2·d.jz+1)` cells a candidate search visits — thousands of
  tiny allocations per call, shared by every protocol routing through
  `coarse_sync` (FT8/FT4/FST4). Replaced with fixed-size stack arrays
  (`[f32; 8]`, `num_blocks` ≤ 5 across every wired protocol) reused
  across the lag loop via `.fill(0.0)`. Behavior-preserving (same
  scores, byte-identical recall on FT8 `qso3_busy`/JTDX/full-parity/
  AP-on, FT4, FST4-60A; AWGN/CCIR 50%-crossings for FT8/FT4/FST4-30
  reproduced to the documented BENCHMARKS.md figures). No measurable
  wall-clock change on this box (same-session git-worktree A/B,
  `tests/coarse_sync_alloc_timing_probe.rs`, added alongside this fix)
  — consistent with `eb859cf`'s own finding that this class of loop's
  cost is dominated by arithmetic, not allocation churn; landed as a
  real elimination of allocator-call optimization barriers and
  needless churn regardless, not for a benchmark delta.
- **`engine::dsp::subtract::apply_at_offset`'s two per-sample loops**
  (the host `subtract_tones_lpf_fft` production path, shared by FT8's
  and FT4's SIC subtract) no longer branch on `j >= 0 && j <
  audio.len()` every iteration — the valid `i` range is computed once
  (`i_lo..i_hi`) and both loops run branch-free over it, since the
  out-of-range case has nothing to do in either loop (no fallback
  value, no `audio[j]` to touch). Byte-identical recall verified on
  the same FT8/FT4 golden suites.
- **JT65's and JT9's `Spectrogram`/`AudioFft::build` no longer sort
  the entire FFT-magnitude array just to read a bottom-95%
  trimmed-mean noise floor.** Found while auditing where a
  candidate-loop `rayon` parallelism pass (added, measured as zero
  benefit across 5 protocols, and reverted — see the entry below) would
  actually have paid off: profiling showed this `sort_unstable_by`
  step was itself *larger* than the FFT loop it was meant to be a
  minor adjunct to (JT65: 13-14.5ms sort vs. 7-7.7ms FFT). Only the
  *set* of bottom-95% values is needed, not their order, so
  `select_nth_unstable_by` (O(n) average partition) replaces the full
  O(n log n) sort — the same fix Q65's `Spectrogram::build_for` and
  FT8's `xsnr2_db_simple` noise median already had. `msk144::spd`'s
  noise-floor quantile had the identical pattern (a single order
  statistic, not even a range) and got the same fix. Measured:
  `jt65::search::Spectrogram::build` 22ms→10.4ms on the AWGN sweep
  corpus (release). No output change (order-independent sum / single
  index), all tests pass unmodified.
- **`jt9::softsym::AudioFft::build`'s big per-slot FFT now uses a
  real-input transform (`realfft`, new optional dependency gated
  behind the already-host-only `jt9` feature) instead of packing real
  audio into a full complex buffer.** Found via a phase-breakdown
  diagnostic (`jt9::tests::phase_breakdown_diag`) written to chase
  down a stale "unexplained ~61% of `decode_scan` time" note from the
  parallelism audit below — that figure predated the
  `select_nth_unstable_by` fix above and never accounted for this
  *second*, separate 653,184-point FFT (distinct from
  `jt9::search::AudioFft`). Real breakdown: `coarse_search` ~36%, this
  FFT ~57%, the actual per-candidate loop only ~7% — explaining why
  candidate-loop parallelism never helped JT9 either. The old code ran
  a full `rustfft` complex-to-complex transform and threw away the
  upper (Hermitian-redundant) half; `realfft` computes exactly the
  same `NFFT1/2+1` bins directly, in ~half the work — same numerics
  (it wraps `rustfft` itself), not a new algorithm. Measured:
  `AudioFft::build` ~9.4-10.0ms→~5.1ms, `decode_scan` total
  ~17-18ms→~12.8-13ms (release, real `jt9_sweep` AWGN WAVs).
  Byte-identical recall (`jt9_wsjtx_sample_recall_vs_golden`, both
  softsym golden-grid roundtrip tests) and unchanged AWGN sweep
  crossing point confirm correctness.
- **Investigated candidate-loop `rayon` parallelism across all 5
  protocols still missing it (JT65/Q65/JT9/uvpacket/MSK144) —
  measured zero benefit everywhere, including a deeper attempt at
  Q65's per-candidate `(Δf,Δt,b90)` grid search, and reverted all of
  it.** The first pass added `par_iter()` to each protocol's top-level
  candidate-decode loop on the (untested) assumption that independent
  candidates parallelize well; real timing showed no speedup on any of
  the five (MSK144's genuine `ScanState` cross-block dependency was
  also correctly identified and handled, for no eventual benefit). A
  second, better-informed pass profiled Q65's `decode_at_grid_for`
  specifically (its `Spectrogram::build_for` was already optimized,
  unlike JT65/JT9 above) and parallelized its `ibw`/`b90` sub-sweep —
  correct (`find_map_first` preserves the exact sequential
  first-success semantics; every real off-air recording still
  decoded identically) but, measured against real WSJT-X sample WAVs,
  also showed no speedup: `GridDepth::Fast` only ever has ≤7 candidate
  cells, called repeatedly inside a sequential outer loop, so
  `rayon`'s per-region dispatch overhead ate the gain — the same
  too-small-parallel-region failure mode as the first pass, one level
  deeper. Nothing from either pass shipped; the two real wins that did
  ship are the `select_nth_unstable_by` and `realfft` entries above,
  found *while* profiling for a parallelism target rather than by
  adding threads.
- Investigated, on host x86_64 (`objdump`, address-bounded to each
  symbol to avoid mis-attributing neighbouring/inlined code — an
  earlier unbounded capture falsely suggested AVX/FMA usage that
  turned out to be `rustfft`'s own kernels bleeding into the dump):
  `engine::sync::score_costas_block` already auto-vectorizes (packed
  SSE `mulps`/`addps`/`subps`/`shufps` complex multiply-conjugate, 2
  elements/iteration) with no code change needed.
  `engine::llr::fill_bmet_for_nsym`'s max-reduction — vectorized for
  `wasm32 +simd128` in PR #213 — does **not** carry the same win to
  host as currently built: its reduction runs scalar (`maxss`/`addss`/
  `subss`), not packed, under this crate's default `target-cpu=generic`
  (SSE2-baseline, no explicit vectorization hint). Left unchanged: the
  wasm measurement found this function is only ~2.9% of total decode
  time even *with* vectorization enabled there (CHANGELOG.md's #208
  entry), so a host fix's expected payoff is similarly small relative
  to the code complexity of forcing it — noted here so it isn't
  re-investigated from scratch, not acted on.

### Fixed

- **FT8 host `decode_block_multipass`'s `xsnr2` SNR-gate baseline was
  frozen at pass 1 instead of recomputed every pass (issue #243).**
  Found investigating why the host multipass path can't safely expose
  a streaming callback (`docs/reference/STREAMING.md` already
  documents the symptom). This crate captured the noise-floor
  baseline (`sbase`/`spec`) only once, at `ipass == 0`, then reused
  that single pre-subtraction snapshot for the final `xsnr2` gate
  applied to *every* candidate from *all 3* passes, in one batch,
  after the whole multipass loop finished. WSJT-X's own
  `ft8_decode.f90` calls `sync8` (which produces the baseline)
  *inside* the pass loop, once per pass, against that pass's
  then-current (already-subtracted-by-prior-passes) audio — pass-2/3
  candidates get judged against a cleaner baseline than pass-1's, and
  each candidate's decode→gate→return happens atomically inside one
  `ft8b` call, never revisited by a later pass.

  Fixed to match: `sbase`/`spec` are now recomputed at the top of
  *every* pass, and the `xsnr2` gate is applied *per pass* (right
  after that pass's own candidate loop, using that pass's own fresh
  baseline) instead of once, globally, after all 3 passes. This also
  means a candidate is fully finalised — accepted or dropped — before
  the *next* pass's subtraction and decoding begins, narrowing (though
  not on its own fully closing) the streaming-unsafe window
  `STREAMING.md` documents.

  Verified via git-worktree A/B on the `qso3_busy.wav` JTDX 18-entry
  golden: **recall byte-identical** (18/18 JTDX hit, 1 extra, both
  before and after), but several pass-2/3 candidates' reported
  `snr_db` changed to be visibly more accurate against the JTDX gold
  reference — most notably `WA2FZW DL5AXX RR73` (previously flagged
  as this suite's one lingering possible-false-positive suspect,
  `-6.4 dB → -16.0 dB`, now close to JTDX's own `-15.0 dB`). Full
  `qso3_apoff`/`qso3_apon`/`qso3_full_parity` golden suites and the
  AWGN/CCIR sweep (50% crossing unchanged at ≈−21.6 dB) all confirm no
  recall regression. `--features fixed-point` (embedded, which doesn't
  run this gate at all) unaffected.

- **FT8 host `decode_block_multipass`'s `xsnr2` gate is now atomic
  per candidate, not per pass — the streaming-unsafe window from the
  entry above is fully closed, not just narrowed.** Follow-up to
  issue #243: applying the gate per-pass (previous entry) still left
  a batch boundary *within* a pass — a candidate decoded early in a
  pass could still, in principle, be reported and then dropped by
  that same pass's own end-of-pass gate before the pass finished. The
  gate now runs inline, immediately after each candidate's signal is
  subtracted from the working buffer and before it is accepted, so a
  result is fully final — accepted or dropped — the instant it is
  processed, matching WSJT-X's own `ft8b.f90` one-candidate-per-call
  decode→gate→return atomicity exactly rather than approximating it
  at pass granularity.

  This removes the only reason `decode_block_streaming` wasn't
  available under `fft-rustfft`: `ft8::decode_block::decode_block_streaming`
  now has a host `#[cfg(feature = "fft-rustfft")]` implementation
  alongside the existing embedded `#[cfg(not(feature = "fft-rustfft"))]`
  one, same signature (`&mut dyn FnMut(&DecodeResult)`, both variants
  are always sequential — no rayon inside `decode_block_multipass`), same
  exact-match delivery contract. `docs/reference/STREAMING.md` updated;
  new `tests/ft8_decode_block_streaming_host.rs` verifies exact
  callback/batch equality (message order and `snr_db`) on
  `qso3_busy.wav`. `qso3_apoff`/`qso3_apon`/`qso3_full_parity`/JTDX
  golden suites and the full lib test suite confirm byte-identical
  output to the per-pass fix above — this is a pure restructuring of
  *when* a result becomes final, not a change to any computed value.

- **`DecodeRequest::sic_early().on_result(cb)` combined with
  `.known(...)` could deliver a callback for a result that then never
  appeared in the returned `Vec` — a second, distinct instance of the
  revoke-less-retract hazard issue #243 closed on the `decode_block`
  engine, this time in `SupportsSicEarly::__staged_sic`
  (`ft8/decode.rs`), a completely different code path from
  `decode_block_multipass`.** Found from a real user report against
  the #243 fix above, reproduced with the exact two messages
  reported (`K1BZM DK8NE -10`, `XE2X HA2NP RR73` — both real,
  marginal-SNR decodes at `hard_errors` 20/16 on `qso3_busy.wav`).

  `__staged_sic` subtracted `.known(...)` from the audio up front
  (correct), but never threaded `known` into any of the three
  checkpoints' own message77 dedup — instead relying on a *post-hoc*
  `results.retain(|r| !known.iter().any(...))` after
  `decode_frame_subtract_staged_with_ap_inner` had already fired
  `on_result` for every checkpoint's raw candidates. When subtraction
  of a `known` signal left enough residual for a checkpoint to
  independently re-decode that same message — routine for
  marginal/high-hard-error signals — the callback fired and the
  retain then silently dropped the result before it reached the
  returned `Vec`, violating `.on_result`'s own documented "exact
  match, zero divergence" contract for this sequential strategy.

  Fixed by threading `known` (renamed `outer_known` inside the inner
  function) into every checkpoint's own `sic_inner_passes` call —
  combined with each checkpoint's own already-decoded set at
  checkpoint C — so the message77 dedup that already ran atomically
  *before* the subtract/callback point (same shape the #243 fix
  established) now also covers caller-supplied `known`, not just
  same-call duplicates. The post-hoc `retain` is removed entirely
  rather than kept as a backstop — keeping it would have re-admitted
  exactly the hazard being closed.

  New tests `ft8_streaming_sic_early_matches_batch_exactly` and
  `ft8_streaming_sic_early_with_known_matches_batch_exactly`
  (`tests/ft8_streaming_decode.rs`) — the latter directly reproduces
  the reported bug (fails on the pre-fix code, passes after).
  `qso3_apoff`/`qso3_apon`/`qso3_full_parity` golden suites, the
  `.sic_early()`-specific probes (`ft8_qso3_dl8yhr_probe`,
  `ft8_qso3_dk8ne_probe`, `ft8_qso3_staged_sic_check`,
  `ft8_qso3_subtract_fix_check`, `ft8_wsjtx_depth_ladder`,
  `ft8_qso3_sync_cv_iteration_correlation`) and the full lib suite all
  confirm no recall change on the (much more common) `known = &[]`
  path — this only changes behaviour when `.known(...)` is combined
  with `.sic_early()`, and even then only removes phantom callback
  deliveries, never removes anything from the returned `Vec`.

- **The same revoke-less-retract pattern also existed on FT4's
  `.sic_rounds()`/default single-pass strategies and FST4's default
  single-pass strategy — found by grepping for the pattern rather than
  assuming the FT8 fix above was exhaustive, per the retrospective
  above.** `Ft4`/FST4's `dedup_known` post-filtered the *returned*
  `Vec` against `.known(...)` after `req.on_result` had already been
  threaded straight into the shared `engine::pipeline::decode_frame`/
  `decode_frame_subtract` and fired there — the generic pipeline has
  no `known` parameter of its own, so nothing gated the callback
  before it fired. Fixed with a new `pipeline::known_filtered_on_result`
  helper: wraps the caller's callback so a `known`-duplicate never
  reaches it, closing the gap without threading `known` through the
  protocol-agnostic engine itself. New regression test
  `ft4_streaming_sic_rounds_with_known_matches_batch_exactly`
  (`tests/ft4_streaming_decode.rs`); FST4 has no `.sic_rounds()` (no
  `SubtractCfg` exists for it) so its only affected strategy is the
  default single-pass one, already covered by the existing superset
  contract but tightened to exact anyway for consistency. Q65/WSPR/
  JT65/JT9 checked and confirmed unaffected — none of them has a
  `known`/cross-phase-dedup concept at all. No recall change on the
  `known = &[]` path (all existing FT4/FST4 recall + streaming tests
  pass unchanged).

- **FST4 could independently re-decode the same real signal up to 9x
  via distinct sync candidates that converge on the same true `(freq,
  dt)` only after per-candidate refinement — each redundant candidate
  paid the full LLR/BP/OSD staircase before a post-decode,
  message-based dedup threw all but one away.** Found auditing
  `on_result`'s §3b "possible transient duplicate" contract (issue
  #244) — the duplicates are legitimate per that contract (nothing
  incorrect ends up in the returned `Vec`), but the *compute cost* of
  redundantly decoding the same signal several times over is real and,
  compared to WSJT-X, avoidable.

  WSJT-X's own `fst4_decode.f90:310-353` never hits this: after a
  cheap per-candidate sync-refine (`fst4_sync_search`, no BP/OSD), an
  explicit "remove duplicate candidates" pass collapses any two
  candidates whose *refined* `(freq, isbest)` land within `0.10*baud`
  Hz / ±2 samples — only survivors ever reach the expensive decode
  loop. `engine::pipeline::decode_frame_impl` (FST4's shared engine)
  had no equivalent stage: coarse candidates went straight from
  `coarse_sync`'s own (coarser, pre-refine) NMS into full decode.
  Tightening that pre-refine NMS tolerance to match WSJT-X's ratio was
  tried first and measured *worse* (5→7 redundant firings on a test
  signal) — a tighter coarse filter lets more raw candidates survive
  into the expensive path, which `fst4_sync_search`'s own wide
  coherent search then independently pulls onto the same true position
  anyway. Ported WSJT-X's actual two-stage mechanism instead: a new
  `refine_candidate_position` (downsample + RMS-normalise +
  `fst4_sync_search` only) run for every coarse candidate, followed by
  `dedup_refined_candidates` — pre-decode NMS on the refined position,
  WSJT-X's exact tolerance, greedy by refined score. Scoped to the
  non-FT4 branch only; FT4's own `ft4_coarse_sync` measured zero
  redundant firings already (both a real WSJT-X sample and a clean
  synthetic signal) and wasn't touched.

  Measured on the real WSJT-X `FST4+FST4W/210115_0058.wav` golden
  sample: `on_result` firing count for the file's 2 real signals went
  from **9 → 2** — the redundancy is fully eliminated on this file, not
  just reduced (a separate clean-synthetic test went 5→2; the residual
  pair there sits outside even WSJT-X's own tolerance window, so
  WSJT-X's algorithm wouldn't catch it either — not a gap in the port).
  New regression test `fst4_60_wsjtx_sample_on_result_fires_once_per_decode`
  asserts this exactly (not just "fewer than 9") so a future regression
  is caught immediately. Every existing FST4 recall/golden test
  (`fst4_wsjtx_samples`, `fst4_sim_roundtrip` all 5 sub-modes,
  `fst4_streaming_decode`, `fst4_fft_cache_decode`) unchanged; full lib
  suite and `pre-push-check.sh`'s full feature matrix clean. Full AWGN
  sweep (`fst4_snr_sweep`, all 5 sub-modes) 50% crossing points land
  within 0.15 dB of the documented baseline table (FST4-300 essentially
  exact, -34.78 dB measured vs -34.78 dB documented) — normal
  sampling noise at n=20 trials/point, not a shift.

  **Follow-up, same day**: a user question ("BENCHMARKS.md should be
  updated — do we have the data?") prompted an actual before/after
  wall-clock measurement, which the `on_result` firing-count metric
  above never was. First result: the fix above was a **21% wall-clock
  regression** (369ms → 447ms, single-threaded, git-worktree A/B, 3
  runs each) on the same golden file — `refine_candidate_position`'s
  result was discarded after the dedup decision, so every surviving
  candidate paid the downsample+`fst4_sync_search` refine cost *twice*
  (once in the new dedup pass, once again inside
  `process_candidate_basic_impl`, unchanged). Fixed by threading the
  already-computed `cd0`/refined position through
  `process_candidate_basic_impl`'s new `precomputed_refine` parameter
  instead of discarding it — survivors now pay that cost exactly once,
  same as before this whole change. Re-measured with proper sample
  sizes (8-10 runs each, not 3): before ≈378.95ms, after ≈379.05ms —
  statistically indistinguishable, i.e. **wall-clock neutral on this
  file**, not a regression and not a measured speedup either. The
  earlier "worse" 3-sample measurements this correction itself relied
  on turned out to be as noisy as the original 3-sample "regression"
  reading — small sample counts on a ~370-450ms task aren't reliable
  here; 8+ samples were needed to separate signal from noise in both
  directions. No BENCHMARKS.md update: the honest conclusion is
  "eliminates wasted redundant compute (verified via the 9→2
  firing-count metric), wall-clock-neutral on the one file measured,
  no user-visible speedup claim to make." Golden-WAV output
  (message/freq/dt for both signals) verified byte-identical before
  and after this correction — pure performance refactor, no decode
  logic changed.

- **Q65 and MSK144 can now resolve `<...>` hashed-callsign
  placeholders — a gap this session first mis-scoped as "5 protocols
  wide" (Q65/WSPR/JT65/JT9/MSK144) before actually checking each
  protocol's own message format.** The hashed-callsign mechanism
  (WSJT-X's 77-bit message Type 4: one non-standard callsign + one
  12-bit-hashed standard callsign) is specific to
  `msg::wsjt77::Wsjt77Message`'s packing, not a cross-protocol concept
  — so the real scope, once checked, was narrower:
  - **MSK144** unpacks via the exact same `Wsjt77Message`/
    `unpack77_with_hash` dispatch FT8/FT4/FST4 use
    (`frame_decode::decode_frame`'s own doc comment already claimed
    this parity), but `decode_slot` — the crate's only public MSK144
    driver — had no parameter to supply a table, so every session
    permanently showed `<...>` for any hashed callsign heard. Fixed
    with an additive sibling, `decode_slot_with_hash_table` (same
    "`_streaming`-sibling-not-a-breaking-parameter" precedent used
    throughout this crate) — `decode_slot` now just calls it with
    `None`.
  - **Q65** packs its 77-bit payload through the same `Wsjt77Message`
    format internally, and `msg::q65::Q65Message::unpack` already
    honored `ctx.callsign_hash_table` when given one (`msg/q65.rs:150-153`)
    — but none of `q65::rx`'s nine decode functions, nor
    `q65::decode_request`'s three builders (`DecodeRequest`/
    `SniperRequest`/`MultiPeriodRequest`), had any way to build that
    `ctx` with a real table; every call site hardcoded
    `DecodeContext::default()`. Fixed by threading `ctx: &DecodeContext`
    through the full call chain (`decode_at_inner`/`decode_at_grid_for`/
    `decode_scan_inner`/the three `decode_averaged_*`/`decode_multi_period_for`
    helpers) and adding `.hash_table(Arc<CallsignHashTable>)` to all
    three builders — built once per `decode()` call from the caller's
    `Arc` (a refcount bump, not a table clone), not per grid cell,
    since `decode_at_grid_for`'s `(Δf,Δt,b90)` sweep can reach the
    unpack call site dozens of times in one decode attempt.
  - **WSPR** was *not* a gap: its own Type-3 hashed-callsign scheme
    (a distinct 15-bit `nhash`, not `Wsjt77Message`'s 12-bit hash) is
    deliberately exposed raw on the decoded message
    (`WsprMessage::Type3 { callsign_hash: u32, .. }`, `msg/wspr.rs:50-54`,
    "exposed raw so callers with a compatible WSPR hash table can
    resolve it") — already a complete design, caller-side resolution
    by intent, not an oversight.
  - **JT65/JT9** were *not* a gap either: `msg::jt72::Jt72Codec` (the
    72-bit `packjt.f90`-derived format both share) has no hashed-
    callsign concept in its wire format at all — an older WSJT
    message layout that predates the Type-4 compression scheme FT8
    introduced, not a crate limitation.

  New coverage: a differential test per fixed protocol (a same-audio
  round-trip can't distinguish "resolved" from "nothing to resolve"
  when the golden message has no hashed callsign, so both pin a
  message built via `pack77_type4`/`unpack77_with_hash`'s own recipe
  and assert the placeholder only resolves when a table is supplied) —
  `msk144::decode::tests::decode_slot_with_hash_table_resolves_hashed_callsign`
  and `q65::rx::tests::sniper_hash_table_resolves_hashed_callsign`.
  Byte-identical recall on every existing golden-WAV suite (MSK144 2/2,
  all 7 Q65 WSJT-X-golden-WAV tests), full `scripts/pre-push-check.sh`
  matrix clean.

### Docs

- **`docs/reference/STREAMING.md`/`.ja.md` §3 gained a "revoke-less
  retract" audit section** documenting the failure mode behind the two
  `on_result` fixes above (a callback fires for a candidate that a
  *separate*, later post-processing step then silently excludes from
  the returned `Vec`, with no revise/retract event) and a line-cited
  table confirming every other `on_result`/`cb` call site in the crate
  (WSPR ×2, Q65 ×5, JT65, JT9, plus FT8/FT4/FST4's own non-`known`
  paths) commits the callback-fired value/set to the returned
  collection with no filtering step in between — checked directly, not
  inferred from the fixes. Written up so a future `_streaming` sibling
  added to a protocol that also gains a `.known(...)`-style
  cross-phase-dedup parameter has a concrete pattern to check against,
  instead of every consumer needing their own reproduction experiment
  to confirm it.

### Changed

- **The remaining WSPR/JT9/MSK144 hot-loop findings from the same
  vectorization audit that produced `engine::sync::coarse_sync`'s and
  `engine::dsp::subtract::apply_at_offset`'s fixes above.** All six are
  the same two anti-patterns applied to different files — per-element
  index-only bounds branches (hoistable, same fix shape as
  `apply_at_offset`) and heap allocation inside a hot loop (same fix
  shape as `fill_sync2d_row!`) — not new vectorization work of their
  own. Byte-identical recall and AWGN sweep crossings on every affected
  protocol's golden/sweep suite (WSPR 8/8 golden + full 13-point AWGN
  sweep, MSK144 3/3 golden + full short/long-ping sweep, JT9 7/7 golden
  + full AWGN sweep) — all reproduced their documented BENCHMARKS.md
  percentages exactly.
  - **`wspr::subtract::subtract_signal_baseband`**: its two per-sample
    loops (`camp` build, final subtract) branched on `k > 0 && k < np`
    every iteration — the identical anti-pattern
    `apply_at_offset` had before today's earlier fix, just never
    applied here. Clamped the valid `i` range once per loop instead;
    the second loop's `n > 0.0` guard (a *value*, not a pure index
    boundary — `partial[]`'s startup-transient correction) stays a
    per-iteration check, not hoisted.
  - **`msk144::sync::msk144_freq_search`**'s per-CFO-trial loop
    allocated four fresh `Vec`s (`mixed`/`c`/`ct2`/`xcc`) every trial.
    Added `tweak1_into` (writes into a caller buffer instead of
    allocating) and hoisted all four buffers outside the trial loop,
    reused via `.fill`/`.copy_from_slice`; `best_frame`/`best_xcc` now
    copy from the scratch buffers only on an actual improvement
    (rarer than every trial) instead of moving a freshly-allocated
    buffer out of the loop unconditionally.
  - **`msk144::sync::rotate_to_shift`** used a per-element `%
    NSPM` index for what's structurally two contiguous copies (the
    same file's own `ct2` doubled-buffer build, a few lines above,
    already uses the correct two-`copy_from_slice` idiom for an
    equivalent wraparound — this function just didn't reuse it).
  - **`msk144::spd::detect_burst_candidates`**'s per-offset scan
    allocated two fresh buffers (`ctmp`/`tonespec`) every offset;
    `NFFT == NSPM` makes both fixed-size for the whole scan, so
    they're now allocated once and reused.
  - **`jt9::softsym::AudioFft::build`**'s envelope loop and
    **`AudioFft::downsam9`**'s FFT-shift bin remap both had an
    index-only `j < buf.len()` / `j >= 0 && j < c1_len` branch inside
    their hot inner loop, monotonic in the loop variable — same
    "boundary only, never mid-range" shape as `apply_at_offset`'s
    original bug. `downsam9`'s remap in particular is two
    separately-monotonic contiguous pieces (an FFT-shift split, `i in
    0..=nh2` vs. `i in (nh2, NFFT2)`) rather than one, so it's clamped
    as two ranges. Both now compute the valid sub-range once and loop
    branch-free over it.

  Not touched, per the same audit's own explicit "don't" list (see the
  `### Changed` entry above this one and the audit's own findings):
  `wspr::coarse_baseband`'s `refine_alignment_top_k` (gather-bound,
  matches `wsprd.c`'s own scattered access, no restructuring
  available without diverging from the reference algorithm), JT65's
  `score_candidate`/`decode_at_with_erasures`/demodulate argmax
  (gather-bound sync-position table / inherently sequential retry
  ladder / stateful reduction — none restructurable), and
  `msk144::sync::tweak1`'s NCO recurrence (loop-carried, single
  channel, no independent lane to interleave against — would need a
  materially bigger redesign, e.g. batching multiple CFO trials'
  rotors as parallel lanes, tracked as a future idea rather than done
  here).

- **Actually measured the above, not just asserted it — findings were
  mixed, and one initial reading was itself a measurement artifact.**
  Same-session git-worktree A/B, `msk144::decode::decode_slot` on both
  WSJT-X golden WAVs: **~13-15% faster** (843→730 ms, 804→680 ms) —
  `msk144_freq_search`'s CFO loop turned out to be a genuinely hot
  path (once per candidate × navmask × dither combination), so
  removing its four-alloc-per-trial cost was a real win, not just
  hygiene. `wspr::decode::decode_scan_subtract` on its golden WAV:
  flat (369.5 ms vs. the already-recorded 369.7 ms, within noise) —
  the LPF convolution step this doesn't touch already dominates.
  `jt9::decode_scan` on its golden WAV **initially measured ~5%
  slower** (311→325 ms) — investigated by isolating `AudioFft::build`
  and `downsam9` with a dedicated timing probe
  (`jt9::softsym::tests::probe_isolate_build_vs_downsam9`, kept as a
  standing diagnostic) and found **no regression in either function**,
  then re-ran the original end-to-end comparison *properly
  interleaved* (alternating worktrees every run instead of measuring
  one side's whole block, then the other's) and it also came back
  flat. The initial reading was this box's own run-to-run noise
  (already documented elsewhere in this file as ~15%) lining up
  against block-grouped measurement order, not a real effect from the
  code change — a live example of why every dated re-measurement
  entry in this file uses interleaved/isolated methodology rather
  than sequential before/after blocks.

- **`jt9::softsym::peakdt9`** — found while writing the isolation probe
  above, not part of the original audit, and a different fix shape
  from the rest of this cluster: its sliding-window coherent-sum loop
  recomputed the whole up-to-`NSPSD`-wide window from scratch at every
  one of `NFFT2` positions (O(NFFT2·NSPSD)); replaced with a running
  sum that adds the entering sample and subtracts the one that fell
  out of the window each step (O(NFFT2)), verified equivalent via the
  window's own `lo(i) = max(0, i-(NSPSD-1))` growth/slide structure.
  Also hoisted two more instances of the same index-only-branch
  pattern already fixed elsewhere in this file: the sync-score search
  loop's `idx >= p.len()` bound (monotonic in `sym`, only the upper
  edge ever triggers, and only for large `lag`) and the `c3` extraction
  loop's `j` bound (monotonic in `i`, a single fixed per-call offset).
  Properly interleaved measurement (git-worktree A/B, alternating every
  run — the methodology the entry above this one exists to justify)
  showed a modest but consistent ~1-2% `decode_scan` speedup across 4
  rounds, smaller than hoped for a supposedly-16x-fewer-ops change,
  but real and in the right direction (unlike the `AudioFft::build`/
  `downsam9` hoists, which measured as genuinely flat). Byte-identical
  recall (JT9 7/7 golden) and AWGN sweep (exact percentage match at
  every SNR point) confirm the running-sum rewrite is equivalent, not
  just faster.

## 0.8.1 — decode-side `snr_db` for WSPR/JT65/JT9/Q65 (#226)

### Added

- **`snr_db: f32` on `WsprResult`, `Jt65Result`, `Jt9Result`, and
  `Q65Result`** (issue #226, breaking — new required struct field on
  four public types). FT8/FT4/FST4 already share `engine::pipeline::
  DecodeResult`, which carries `snr_db`; MSK144 has its own
  (`SlotDecode::snr_db`, WSJT-X `mskrtd.f90` formula). The other four
  WSJT-family decoders had **no** decode-side signal-quality field at
  all — `mfsk-ffi`'s `push_simple` (used for WSPR, and the fixed-
  alignment JT9/JT65 FFI entry points) was hardcoding `snr_db: 0.0`
  for every one of them, which is why the gap wasn't visible from the
  FFI surface. Filed after WebFT8 tried to wire up Q65 QSO-exchange
  reports ("+NN"/"-NN") and found there was no SNR to report from —
  auditing the sibling protocols for the same gap (prompted by "is a
  missing field like this really just one mode's bug?") turned up all
  three others too.

  - **WSPR**: `wspr::coarse_baseband::BasebandCandidate` already
    computed a wsprd-calibrated candidate SNR (`10·log10(smspec) −
    26.3`, `wsprd.c:1093`) during coarse search — it was being
    discarded before reaching `WsprResult`. `decode_scan`/
    `decode_scan_subtract` now thread it through; direct
    `decode_at`/`decode_at_baseband`/`decode_at_baseband_nblocks`
    calls (no coarse candidate to derive it from) report `0.0`.
  - **JT65** and **Q65**: new decode-side estimate — signal = power at
    each data symbol's decoded tone, noise = mean power of the other
    63 tones in the same per-symbol FFT bin (same "opposite-tone"
    shape as `engine::llr::compute_snr_db_generic`, FT8/FT4/FST4),
    converted to WSJT-X's 2500 Hz reference bandwidth via `10·log10
    (2500/tone_spacing_hz)` — cross-checked against FT8's literal
    `-27 dB` @ 6.25 Hz and wsprd's literal `-26.3 dB` @ ~5.1 Hz, both
    within ~1 dB of that formula, so expect similar-order accuracy
    pending real calibration. Q65's four wide-energies decode paths
    (fast-fading metric, grid search) get a layout-aware variant of
    the same estimator; Q65's info-symbols are re-encoded back to the
    63-symbol channel codeword via `Q65Codec::encode` to know which
    tone was "decoded" at each slot (AP-list paths use the winning
    candidate codeword directly, no re-encode needed).
  - **JT9**: same signal/noise decomposition, but **not** WSJT-X
    2500 Hz-referenced — `jt9::softsym`'s `downsam9`→`peakdt9`→
    `symspec2` pipeline runs the per-symbol power through AGC scaling,
    an unnormalised IFFT, and a coherent sample sum before the tone
    comparison, so the tone-spacing bandwidth offset that works for
    JT65/Q65 doesn't apply; an initial attempt that borrowed it
    produced an implausible reading (clean noiseless synth ≈ −4 dB).
    `Jt9Result::snr_db` is documented as relative-only: useful to
    compare JT9 decodes against each other, not against the other
    protocols' dB2500 values.
  - Caught in testing: the shared floor/ratio formula returned the
    **-24 dB floor** for a perfectly clean noiseless synth signal
    across all three new estimators, because an exactly-orthogonal
    synthetic tone can leave literally zero measured power in the
    non-signal bins — "no measurable noise" (the *best* case), which
    the original floor-on-zero-noise branch (mirroring `engine::llr::
    compute_snr_db_generic`'s existing behavior) reported as the
    *worst* case instead. Fixed by returning a `+49 dB` ceiling
    (matching WSJT-X's own display-clamp convention) when noise is
    zero and signal isn't, in `q65::rx::snr_db_from_sig_noi`,
    `jt65::rx`, and `jt9::softsym::symspec2_from_ss2`.
  - `mfsk-ffi`'s `push_simple` now takes `snr_db` explicitly (WSPR and
    Q65 pass the real value; the JT9/JT65 fixed-alignment entry points
    still pass `0.0` — that path uses the bare `decode_at`, which has
    no SNR estimate available, not `decode_scan`).

## 0.8.0 — JT65 decode-chain bug fix (#24) + JT9 AWGN SNR sweep + Q65-15A/120D/120E/300A + fine-timing sensitivity fix + CQ-AP-hint parity note (#171) + BASIS removal (#162, breaking FFI change) + FT8 `DecodeDepth` redesign + auto-AP removal (issue #182 follow-up, breaking) + CCIR moderate/poor sweep gap closed (#190) + `DecodeRequest`/`SniperRequest` consolidation (#191, breaking) + `core::pipeline` dead-code cleanup (#192, breaking) + pre-#191 raw decode API demotion (#203, breaking) + `core` → `engine` module rename (#206, breaking) + FT8/FT4/FST4 `DecodeResult` unification (#194, breaking) + sealed `FecCodec` (#198) + Q65 `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` builder migration (#204, breaking) + unified `mfsk-ffi`/`mfsk-ffi-ft8` C-ABI conventions via new `mfsk-ffi-abi` shared crate (#205, breaking) + WSPR/JT9/JT65/Q65 decode-result naming convention (#206, breaking) + `downsample_cached` FFT-plan caching fix (#211) + wasm `+simd128` LLR vectorization (#208) + embedded-poc `+esp` compile fix (#215) + `DecodeRequest`/`SniperRequest::depth` → `.osd(bool)` (breaking) + FT8 `WsjtxDepth`/`wsjtx_depth` jt9-comparison preset + `DecodeRequest::flat()`/`.staged()` → `.sic_rounds(n)`/`.sic_early()` (#218, breaking) + FT8 `DecodeStrictness` wiring for the non-AP decode path (#221)

### Added

- **`DecodeRequest<P: FrameDecodable>` / `SniperRequest<P>`** (issue
  #191) — a single, ZST/trait-driven builder replacing the FT8
  `decode_frame*`/`decode_frame_subtract*`/`decode_sniper*` family (15
  public functions), FT4's `decode_frame`/`_with_options`/`_with_cache`/
  `_with_cache_and_options`/`decode_frame_subtract`/`_with_options`/
  `decode_sniper_ap`/`_with_options` (8 functions), and FST4's
  `decode_frame_for`/`_with_options_for`/`_with_cache_for`/
  `_with_cache_and_options_for` plus the FST4-60A convenience wrappers (8
  functions) — 31 functions total collapsed into two generic types plus
  a handful of builder methods. Lives in `msg::decode_request`,
  re-exported from each protocol's `decode` module.

  Suffix-exploded functions encoded three orthogonal axes (`ap_hint`,
  `precomputed_fft`/`known`, SIC strategy) as combinatorial function
  names, the same disease #188 fixed for `DecodeDepth` alone. The
  reported symptom: `decode_frame_subtract_with_known_and_ap` — the
  *only* function accepting `known`/`precomputed_fft`, and the one an
  external pipelined consumer (WebFT8's `decode_phase1`/`decode_phase2`)
  actually called — ran its own unfixed flat-3-pass engine, never
  receiving the staged-checkpoint SIC (#180) or sequential-subtract
  (#178/#179) fixes the "regular" subtract path got. Structurally
  guaranteed to recur with more suffixes.

  New design, capability-gated via marker traits so invalid
  protocol/feature combinations are compile errors, not silent no-ops or
  runtime panics:
  - `FrameDecodable: Protocol` — `type DecodeResult` + hidden dispatch;
    implemented for `Ft8`, `Ft4`, every FST4 sub-mode. Deliberately not
    implemented for Q65/WSPR/JT65/JT9/uvpacket, which keep their
    existing bespoke entry points.
  - `SupportsFlatSic` (`Ft8`, `Ft4`) gates `.flat()`; `SupportsStagedSic`
    (`Ft8` only) gates `.staged()`; `SupportsWideBandAp` (`Ft8` only)
    gates `DecodeRequest::ap_hint` (FT4/FST4's AP engine has an
    early-exit-after-first-hit optimization only valid for
    `SniperRequest`'s narrow-band single-target search — enabling
    wide-band AP for them would be new, unvalidated capability, kept
    out of scope). `SniperRequest::ap_hint` is gated by the existing
    `P::Msg: WsjtApCompatible` sealed trait instead, and is available
    for all three protocols.
  - `.known()`/`.fft_cache()` are universal (any `FrameDecodable`
    protocol): `known` is always honoured as a dedup filter, and as an
    upfront `subtract_signal_lpf_refine_dt` subtraction for FT8's
    `.flat()`/`.staged()` (the actual #191 fix — see below).
    `fft_cache` is reused where the underlying engine's buffer shape
    permits (FT8 single-pass/flat pass 0); elsewhere it's a documented
    no-op degrade (always-correct recompute, not a silent behavior
    change) since `core::pipeline`'s generic engine has no cache
    injection point.

  The actual bug fix, not just the API reshape: FT8's staged-checkpoint
  engine (`SupportsStagedSic::__staged_sic`) now subtracts `known` from
  the full audio buffer *before* checkpoint A runs (using
  `subtract_signal_lpf_refine_dt` — plain `subtract_signal_lpf`
  measurably lost `CQ DX DL8YHR JO41`, ~35 Hz from a `known` W1FC
  signal, in end-to-end testing; the dt-refined ±90-sample
  best-alignment search checkpoint B/C already used for their own
  carried-forward decodes turned out to matter for `known` too), so all
  three checkpoints see a residual with caller-supplied signals already
  removed. New regression test
  `ft8_qso3_staged_sic_check::staged_with_known_and_cache_finds_dl8yhr`
  reproduces the exact two-phase pipelined-caller shape end to end.

  Type unification required as a prerequisite for a genuinely generic
  builder (previously duplicated, non-interchangeable definitions):
  `ApHint` (canonical: `msg::ap::ApHint`; FT8's own copy was already
  byte-for-byte identical, reusing the same `pack28`/`pack_grid4`),
  `FftCache` (canonical: `core::pipeline::FftCache`, same underlying
  `Vec<Complex<f32>>`), `DecodeStrictness` (canonical:
  `core::pipeline::DecodeStrictness`; `ap_max_errors` — previously
  duplicated in `msg::pipeline_ap` too — moved onto the type, no numeric
  change), `DecodeDepth` (canonical: the #188-redesigned
  `{llr_effort, osd}` struct, replacing `core::pipeline`'s stale
  `BpAll`/`BpAllOsd` 2-variant enum FT4/FST4 were still on).

  Deleted outright (confirmed zero callers anywhere in the crate,
  including internal): `decode_frame_subtract_with_known`(`_and_ap`)
  (the buggy engine above), `decode_frame_with_cache` (FT8/FT4/FST4, all
  three), `decode_sniper_sic`. No deprecation shims — matches #188's
  precedent of a hard breaking rename with all callers migrated in the
  same change.

  Engine unification (porting FT8's `fine_refine_3stage`/nsync-gate/
  sync_cv into the generic `core::pipeline` engine FT4/FST4 share, so
  FT8 could stop having its own bespoke decode engine) was investigated
  and found to be organic drift rather than a necessary architectural
  boundary (git archaeology: both engines already existed side-by-side
  at the initial fork from `jl1nie/webft8`; FT8 pulled ahead via two
  unported investment commits) — but explicitly **not** bundled into
  this change; the trait boundary here doesn't block it (`decode()`'s
  internal engine dispatch is a private implementation detail), so it's
  tracked separately as issue #192. FST4 SIC support (issue #193, no
  `SubtractCfg` exists yet — new numerical work, not a refactor) and
  full `DecodeResult` unification (issue #194, FT8's `message77` strips
  CRC bits FT4/FST4's `info` retains — a real semantic difference, not
  just a naming one) are likewise deferred as separate issues.

- **`ft8::decode::WsjtxDepth` / `DecodeRequest::<Ft8>::wsjtx_depth`**
  — three named tiers (`D1`/`D2`/`D3`) mirroring real WSJT-X's `jt9 -d
  1/2/3` CLI flag, for apples-to-apples benchmarking against a real
  `jt9` build. `DecodeDepth` alone (LLR effort + OSD) has no SIC/AP
  dimension — jt9's real `ndepth` axis (`ft8_decode.f90:168-192`,
  `ft8b.f90:403-412`) simultaneously varies SIC pass count, `syncmin`,
  subtract dt-refine, and OSD strength — so this bundles `.osd(...)` +
  `.flat()`/`.staged()` + `.ap_hint()` into one preset per tier. Local
  `jt9 -8 -d1/-d2/-d3` measurements on `qso3_busy.wav` (this session):
  D1 14 decodes/370ms vs. mfsk-core 14/237ms; D2 19/1040ms vs.
  22/1078ms; D3 22/2110ms vs. 22/2991ms — see the type's own doc
  comment for the exact tier→builder-method mapping and known
  limitations (pass-count and OSD-strength don't exactly match jt9's;
  mfsk-core has no equivalent of jt9's lighter `maxosd=0` OSD branch at
  all, a separate follow-up). New durable regression test
  `tests/ft8_wsjtx_depth_ladder.rs` replaces the ad-hoc probes used to
  derive this.

- **`ft8::decode::decode_frame_subtract_staged` / `_with_ap`** (issue
  #180) — WSJT-X-faithful checkpoint SIC for FT8, ported from
  instrumented `jt9.f90`/`ft8_decode.f90` ground truth rather than the
  issue's own initial paraphrase. `jt9`'s disk-file FT8 decode is not a
  single pass over the 15 s slot: it decodes progressively larger audio
  *prefixes* at three checkpoints (0..141_696 / 0..162_432 / 0..172_800
  samples), carrying decoded signals forward and subtracting them from
  the residual *before* the final, hardest checkpoint's candidates are
  attempted — something the pre-existing flat 3-pass
  `decode_frame_subtract` structurally cannot reproduce regardless of
  threshold tuning, since nothing is ever subtracted before the
  residual a marginal candidate is found in gets assembled. Verified
  no regression vs the flat pass on all 3 real reference WAVs
  (`tests/ft8_qso3_staged_sic_check.rs`, `191111_110130.wav`,
  `191111_110200.wav`) — identical decode sets on each. The specific
  case that motivated #180 (`CQ DX DL8YHR JO41`, ~-17 dB on
  `qso3_busy.wav`) still does not decode even with the correct
  checkpoint architecture in place (checkpoint A finds 11 signals
  early, including its closest neighbour `W1FC` only ~35 Hz away,
  correctly subtracted before checkpoint C runs).

  **Root-caused, not just reproduced.** Re-ran real `jt9 -8 -d3`
  (rebuilt from the local `WSJT-X` checkout, `build_jt9/jt9`, on the
  byte-identical `samples/FT8/210703_133430.wav`) to get fresh ground
  truth instead of relying on the issue's prior transcript: at
  DL8YHR's true coordinates jt9's own per-Costas-block breakdown is
  `is1=2 is2=7 is3=6` → `nsync=15`. A new diagnostic
  (`ft8::decode::tests::issue_180_dl8yhr_staged_checkpoint_c_probe`,
  using `decode_frame_subtract_staged_with_ap_debug_residual` to
  reproduce the same per-block count against mfsk-core's *actual*
  production checkpoint-C residual) found:
  - **Raw, unsubtracted audio**: `is1=0 is2=3 is3=3` → `nsync=6`.
  - **Staged checkpoint-C residual** (all 11 early signals cleanly
    subtracted): `is1=1 is2=4 is3=4` → `nsync=9`.
  - A ±15 ms (±3-sample) `dt` sweep around the ground truth only ever
    moved the total between 8 and 10 — the off-by-one dt-convention
    hypothesis (#180 "Bug 1") explains at most ~1 of the 6-point
    shortfall, not the bulk of it.

  Two conclusions from the aggregate counts: (1) subtraction/SIC
  quality **is** doing real work (+3 points, raw→residual) — the
  staged architecture isn't wasted effort; (2) the *remaining* 6-point
  gap barely moves under a ±15 ms timing sweep (nsync stayed in
  8..10), ruling out the off-by-one dt-convention hypothesis (#180
  "Bug 1") as the dominant cause.

  **Symbol-level dump pinpoints the actual mechanism.** Re-instrumented
  `ft8b.f90` to also emit its raw 8-tone `s8` magnitudes for Costas
  blocks 2 and 3 (only block 1 was previously dumped), rebuilt `jt9`,
  and diffed symbol-by-symbol against mfsk-core's own tone magnitudes
  at the identical `(f1, dt)`, on both raw and staged-residual audio.
  Finding: at symbol positions **t=5 and t=6 of every 7-symbol Costas
  block**, mfsk-core computes a large spurious energy spike at **tone
  0** (DL8YHR's own base carrier frequency, 2606.25 Hz) that swamps
  the true, weak Costas tone — e.g. block 2 t=5: mfsk-core residual
  tone0=9.7 vs true tone6=0.2 (raw audio: tone0=42.0 vs tone6=2.7).
  Critically, **jt9's own real-signal data shows this same tone-0
  artifact only in block 1** (where it explains jt9's own weak
  `is1=2`) — its block-2/3 t=5,6 values have the *true* Costas tone
  clearly dominant (e.g. block 2 t=5: tone6=2641 vs tone0=1140; t=6:
  tone5=3570 vs tone0=1584). mfsk-core reproduces the artifact
  correctly where it's real (block 1) but *also* manufactures it where
  the real signal has none (blocks 2 and 3) — a structural,
  position-periodic pattern (always t=5,6 of a block, at three
  absolute times spread across the ~13 s message), not diffuse
  numeric imprecision.

  **Raw baseband (`cd0`) comparison localises the mechanism further —
  and revises the diagnosis.** Re-instrumented `ft8b.f90` a second
  time to dump its raw complex `cd0` (200 sps downsampled baseband)
  samples over the exact 64-sample window spanning block-1's t=5/t=6
  (Fortran index 268..331, i.e. `ibest+128 .. ibest+191`), rebuilt
  `jt9`, and diffed those against mfsk-core's own `cd0` at the
  identical physical samples (0-indexed `n` == Fortran's 1-indexed
  `n+1` — confirmed aligned, not an indexing bug: values track the
  same sign and shape throughout, ruling out a real off-by-one at the
  `cd0` level). The magnitudes are not just noisier — mfsk-core's
  `|cd0|` runs **2-4× (6-12 dB) larger than jt9's own post-subtraction
  `|cd0|` at the same samples**, most pronounced exactly over Fortran
  idx 280-327 (e.g. idx 285: jt9 `|cd0|`=205 vs mfsk-core 632; idx
  291: jt9 121 vs mfsk-core 597). That's not a tone-detection/FFT
  computation bug — it means **mfsk-core's residual genuinely has
  more uncancelled signal energy left in it than jt9's residual does**,
  at the same point in time and frequency, even though both have
  subtracted an overlapping set of interferers by this point.

  **Isolated (a) [subtraction quality] from (b) [a missing
  interferer], and ruled out (b).** Re-instrumented `ft8_decode.f90` a
  third time to dump `f1_save`/`xdt_save`/`allmessages(1:ndec_early)`
  — jt9's *actual* list of the 13 signals subtracted before the
  nzhsym=50 pass — rather than inferring it from stdout decode-print
  order. jt9's 13 match mfsk-core's 18-message final set on 12 calls,
  but include a 13th, `WA2FZW DL5AXX RR73` @ 2545.88 Hz (only 60 Hz
  from DL8YHR's own 2606.25 Hz carrier) that **mfsk-core never decodes
  anywhere** in this investigation (not in either checkpoint's results,
  not in the flat-pass baseline) — a real, plausible candidate for
  hypothesis (b): an unsubtracted nearby signal leaking into DL8YHR's
  band. Tested directly: manually built a `DecodeResult` from jt9's own
  reported `WA2FZW` coordinates and subtracted it from the staged
  residual with the same `subtract_signal_lpf` call production uses.
  Result: **no change** — `is1=1 is2=4 is3=4, nsync=9`, identical to
  before the subtract. Hypothesis (b) is ruled out for this specific
  signal (its own contribution isn't what's showing up as excess `cd0`
  energy near DL8YHR — either the reconstructed/refined waveform
  doesn't fit what's actually there, or the energy comes from
  elsewhere entirely). That left hypothesis (a) — `subtract_tones_lpf`
  vs `subtractft8.f90`'s actual per-signal cancellation depth on the
  *already-subtracted* candidates — as the standing lead.

  **Confirmed definitively: hypothesis (a), 100% subtraction quality,
  zero mfsk-core tone-detection bug.** The decisive test: swap the
  residual, keep mfsk-core's algorithm untouched. Re-instrumented
  `ft8_decode.f90` a fourth time to dump jt9's actual `dd` array —
  raw i16, taken right after jt9's real SIC (its 13 subtracted
  signals) and right before its own `nzhsym=50` `sync8`/`ft8b` search
  — to `/tmp/jt9_post_sic_dd.raw` (180,000 samples, sanity-checked:
  RMS 115, ±577 range, consistent with a heavily-subtracted residual).
  Loaded that exact buffer into mfsk-core and ran its *unmodified*
  `symbol_spectra_direct`/`sync_quality` at DL8YHR's coordinates:
  **`is1=2 is2=7 is3=6` → `nsync=15`** — bit-for-bit the same
  per-block breakdown jt9 itself reports on this buffer. Then ran the
  full BP/OSD chain (`compute_llr` → `bp_decode`/`osd_decode_*`) on
  the same residual: **decodes `"CQ DX DL8YHR JO41"` outright.**
  mfsk-core's downsample → per-symbol tone extraction → LLR → BP/OSD
  pipeline has no bug at all with respect to this signal — given the
  same clean residual WSJT-X itself decodes from, it decodes it too,
  byte-for-byte matching sync counts. The *entire* gap between
  mfsk-core's own decode (`nsync=9`, no decode) and jt9's (`nsync=15`,
  decodes) traces to one place: **mfsk-core's own SIC subtraction
  leaves more residual interference behind than `subtractft8.f90`
  does**, on the exact same set of already-decoded interfering
  signals. The next concrete, scoped step for #180 is a direct
  `subtract_tones_lpf` vs `subtractft8.f90` per-signal suppression-dB
  comparison (GFSK waveform reconstruction fidelity, LPF width/shape,
  QSB/fading-envelope tracking) — not sync-quality computation, not
  pipeline architecture, not a missing candidate. All diagnostics
  (per-block breakdown, `cd0` dump, WA2FZW confirmation, residual-swap
  confirmation) kept in
  `ft8::decode::tests::issue_180_dl8yhr_staged_checkpoint_c_probe`.
  As of the follow-up entry below, `decode_frame_subtract_with_ap`
  (and therefore `decode_frame_subtract`) delegates to this staged
  checkpoint SIC by default; the pre-#180 flat 3-pass behaviour is
  still available via `decode_frame_subtract_flat_with_ap` for callers
  that specifically want it.
  **Cost**: checkpoint A runs its own full 3-sub-pass search on top of
  checkpoint C's, so this is strictly more decode work than the flat
  pass. Measured (release, `--features full`, 10 reps, host, real
  reference WAVs): 1.31–1.87× the flat pass's wall-clock
  (`191111_110130.wav` 231ms→303ms, `191111_110200.wav`
  249ms→460ms, `qso3_busy.wav` 388ms→611ms).

- **`core::dsp::subtract::subtract_tones_lpf_refine_dt` / FT8's
  `subtract_signal_lpf_refine_dt`** (issue #180 follow-up) — closes
  part of the "next concrete step" left open above: a direct
  `subtract_tones_lpf` vs `subtractft8.f90` comparison found the two
  algorithms structurally identical (GFSK waveform synthesis, cos²
  LPF kernel, end-correction — all line-for-line matches), *except*
  for one missing feature: `subtractft8.f90` takes an `lrefinedt`
  argument, and `ft8_decode.f90`'s two early-decode-and-subtract
  checkpoints (lines 132, 162 — exactly the calls
  `decode_frame_subtract_staged` above is modelling) pass
  `lrefinedt=.true.`. That path (`subtractft8.f90`'s internal `sqf`/
  `peakup`) re-searches `dt` by ±90 samples (±7.5 ms @ 12 kHz) around
  the candidate's own value, scoring each trial by the residual power
  left in the signal's own tone band after subtraction, and commits
  the subtraction at the parabolic-fit minimum — rather than trusting
  the early candidate's own (still-coarse) `dt` outright. mfsk-core's
  staged-checkpoint SIC had no equivalent: every early subtract used
  the plain, non-refining `subtract_signal_lpf`. Ported `sqf`/`peakup`
  faithfully (`core::dsp::subtract::fft_lpf::subtract_tones_lpf_refine_dt_fft`)
  and wired it into `decode_frame_subtract_staged_with_ap_inner`'s
  checkpoint-B and checkpoint-C deferred subtracts (the flat 3-pass
  `sic_inner_passes`'s single confirmed-decode subtract is unchanged —
  matches `ft8b.f90:474`'s `lrefinedt=.false.`).
  **Effect on the DL8YHR case**: checkpoint-C's residual `sync_quality`
  at WSJT-X's own ground-truth coordinates improved from `nsync=9` to
  `nsync=10` (jt9 itself gets `nsync=15` on this signal) — a real,
  measured reduction in leftover interference from the same 13
  already-subtracted signals, though not yet enough on its own to
  flip this specific -17 dB decode (`any_hit` still `false` in
  `issue_180_dl8yhr_staged_checkpoint_c_probe`). No regression on any
  of the 18 golden `qso3_busy.wav` decodes or any other reference WAV
  (`ft8_qso3_staged_sic_check.rs`, `ft8_qso3_subtract_fix_check.rs`,
  `ft8_qso3_apoff_recall.rs`, `ft8_qso3_apon_recall.rs`, full
  workspace `cargo test --release --features full`: 100% pass). New
  regression test:
  `core::dsp::subtract::refine_dt_tests::refine_dt_beats_plain_subtract_when_dt_is_off`.
  Remaining gap (candidates per the prior comment: LPF width/shape,
  QSB/fading-envelope tracking, or f32 vs Fortran-single-precision
  accumulation differences over the 151_680-sample reference) is still
  open — logged here rather than pursued further in this pass.

- **`core::dsp::subtract::fft_lpf::normalized_kernel` LPF window shape
  bug — root cause of the DL8YHR gap, issue #180 closed.** Auditing
  the "LPF width/shape" item left open above (per the entry directly
  above this one) against `subtractft8.f90`/`subtractft4.f90` found the
  actual bug: `window(j) = cos(pi*j/NFILT)**2` in WSJT-X (`NFILT =
  2*lpf_half`) had been ported as `cos((j - lpf_half) * pi / lpf_half)²`
  — dividing by `lpf_half` instead of `NFILT` (`2*lpf_half`), doubling
  the cosine's argument range from `[-pi/2, pi/2]` to `[-pi, pi]`.
  `cos²` is monotonic (a proper single taper, 1 at the centre → 0 at
  the edges) only over the first range; over the doubled range it dips
  to 0 at the quarter points and rises back to **1 — full weight — at
  the true edges** (`lpf_half` samples / 166 ms away from the centre
  sample, for FT8's `lpf_half=2000`). Verified numerically: at
  offset=2000, shipped kernel=1.000 vs correct=0.000; at offset=1000,
  shipped=0.000 vs correct=0.500. So every `subtract_tones_lpf` /
  `subtract_signal_lpf` call since this became the canonical FT8/FT4
  SIC entry point (v0.6.2) had been running a badly-misshapen "lowpass"
  that gave 166-ms-stale channel samples as much weight as the current
  one — actively corrupting the complex-amplitude/QSB estimate instead
  of smoothing it. Root cause of the residual DL8YHR gap the two
  entries above this one were chasing (`refine_signal_freq`'s dt-search
  only got `nsync` from 9 to 10 on top of this broken window).
  One-line-formula fix, shared by both the FFT-cached path
  (`fft_lpf::normalized_kernel`, feeds both `cached_window_fft` and
  `end_correction`) and the `no_std` direct-convolution fallback
  (`subtract_tones_lpf_direct`) — fixing both fixes FT4 subtract too
  (`subtractft4.f90` uses the identical window formula, confirmed by
  inspection; FT4's own recall floor is unaffected — see regression
  results below). **Result: `CQ DX DL8YHR JO41` now decodes**
  (`decode_frame_subtract_staged`'s `qso3_busy.wav` golden-set test,
  `staged_sic_matches_flat_pass_golden_floor`: still 18/18 golden hits,
  0 phantoms in that set, unique-decode count 19→20, `has_dl8yhr` flips
  `false`→`true`) — issue #180 is closed. Full workspace `cargo test
  --release --features full` (all 70 test binaries): 100% pass, no regression on
  any golden decode set (FT8 or FT4).

- **`decode_frame_subtract_with_ap` now delegates to the staged
  checkpoint SIC by default** (issue #180 follow-up) — the flat 3-pass
  behaviour is a strict recall subset (verified: identical golden-set
  hits on every reference WAV, plus `CQ DX DL8YHR JO41` on
  `qso3_busy.wav`, which the flat loop structurally cannot reach) at
  1.3-1.9× the flat pass's wall-clock, still comfortably inside FT8's
  15 s slot budget on host. The old flat behaviour remains available
  as its own entry point, `decode_frame_subtract_flat_with_ap`, for
  callers that specifically want the cheaper non-staged path.
  **Fixed a stack-overflow regression this introduced**: the staged
  path's own two fallback branches (buffer shorter than checkpoint A;
  checkpoint A finds nothing) used to fall back to
  `decode_frame_subtract_with_ap` — which, after this change, calls
  right back into the staged path, recursing indefinitely. Both now
  fall back to `decode_frame_subtract_flat_with_ap` instead. Caught by
  the full-workspace regression run
  (`decode_frame_subtract_with_ap_silence_shape`, a short-buffer test)
  before release.

- **`decode_frame_subtract_with_auto_ap`** (`fft-rustfft` only) —
  `decode_frame_subtract_with_ap` plus a final rescue pass: harvest
  caller callsigns from the blind decodes and retry `coarse_sync`
  candidates that didn't decode blind, using each harvested callsign
  as an AP `mycall` hint. Recovers signals too weak for blind BP/OSD
  but not too weak to sync — e.g. `K1BZM DK8NE -10` (-19 dB) on
  `qso3_busy.wav`. **This is a genuine mfsk-core-original extension,
  not a port of any real jt9/WSJT-X mechanism** — checked directly
  against `ft8apset.f90`'s actual source: real WSJT-X only applies AP
  using the *operator-configured* `mycall`/`hiscall` and disables AP
  entirely the moment `mycall` isn't set (`if(len(trim(mycall12)).lt.3)
  return`, the subroutine's first line). A real `jt9 -8 -d 3` run with
  no `-c`/`-x` has AP fully disabled; its own `K1BZM DK8NE -10` decode
  is blind (`iaptype=0`) — a real, separate, still-open blind-decode
  sensitivity gap this function does not explain or close, it just
  reaches the same message through a different, AP-assisted route.
  Cost-optimized from a naive per-(candidate, callsign) sweep (~5.7s
  on `qso3_busy.wav`) via three measured steps: (1) a presync
  `sync_quality` gate before the callsign loop (66→8 candidates), (2)
  a blind-decode precheck sharing the same refined-candidate cache,
  (3) `DecodeDepth::BpAll` instead of `BpAllOsd` for the AP retries
  (OSD is redundant once AP narrows the search) — cut to ~1.25-1.3s
  single-threaded, ~0.9s on 24 cores via `rayon` (the per-callsign
  loop is embarrassingly parallel, no shared mutable state). The
  parallel speedup is real, independent task-level parallelism — not a
  substitute for closing the blind-decode gap above, and single-
  threaded this function alone still costs more wall-clock than real
  `jt9 -8 -d 3`'s entire file decode (~1.1s), since it does provably
  more search than jt9 needs for this signal (nothing — its own blind
  pass already finds it).

- **OSD fallback gate loosened from `q >= 12` to `q > 6`**
  (`decode_block/osd_strategy.rs`, issue #180 follow-up) — the old
  gate was a mfsk-core-specific deviation with no `ft8b.f90`
  counterpart, which only bails (`nsync <= 6`) before attempting any
  decode at all. Root-caused directly: `K1BZM DK8NE -10` sits at
  `q=11` on mfsk-core's own SIC residual — verified an *exact*
  per-block match to real jt9's own residual at the same coordinates
  (`is1=1 is2=7 is3=3`, confirmed via a locally-instrumented jt9
  rebuild) — yet never reached the OSD fallback at all under the old
  gate. Full regression suite showed zero change from loosening it —
  nothing was relying on the tighter gate to stay green.
  **Not sufficient on its own**: even past this gate, the real OSD
  dispatch for `q=11` (`osd_decode_npre1`, ndeep=2) still fails to
  decode this candidate on any LLR variant, where WSJT-X's real
  `osd174_91.f90` ndeep=2 succeeds (`hard_errors=18`). Verified this
  isn't a SIC/LLR-input problem: mfsk-core's own residual matches
  jt9's own residual not just on sync quality but on all 58
  data-symbol tone decisions (0/58 argmax disagreements) and on the
  actual LLR reliability ordering OSD consumes (top-91-most-reliable
  overlap 90-91/91 across all 4 LLR variants, zero hard-decision
  disagreements within the shared basis). Filed as a genuine OSD
  algorithm fidelity gap — issue #182, open.

- **FT8 OSD now seeds with BP-refined LLR, closing issue #182.**
  Root-caused the `osd_decode_npre1` fidelity gap above: WSJT-X's real
  `decode174_91.f90` driver never feeds `osd174_91` the raw channel LLR
  when `maxosd>0` — and FT8's blind `ndepth=3` dispatch always sets
  `maxosd=2` (`ft8b.f90:434-441`). It feeds `zsave(:,i)`, the running
  sum of the BP variable-node soft estimate `zn` across the first `i`
  BP iterations, trying `i=1` then `i=2` (`decode174_91.f90:52-64,
  137-148`). mfsk-core's OSD had only ever used the raw channel LLR
  (the 4 `a/b/c/d` variants) — confirmed exhaustively: `K1BZM DK8NE
  -10` never decodes via any channel-LLR variant at any OSD depth up
  to and including brute-force order-2 exhaustive search (no
  `ntheta` gate at all), so the true codeword simply isn't reachable
  from a channel-LLR-selected basis. The mechanism needed
  (`bp_llr_zsum`) already existed and is wired for FST4-120
  (`Ldpc240_101`, issue #146) but was never ported to FT8's
  `osd_strategy.rs` dispatch. Wired it in as a second-stage fallback
  (after the existing 4 channel-LLR attempts fail): feeding
  `bp_llr_zsum(llrd, 2)` into the *same*, unmodified `osd_decode_npre1`
  decodes DK8NE outright at `hard_errors=17` — better than jt9's own
  real result (`hard_errors=18`) on this exact candidate. (A parallel
  hypothesis — that WSJT-X's `osd174_91.f90:86-107` bounded-window
  `k+20` pivot search selects a different MRB basis than mfsk-core's
  unbounded one — was tested directly and disproven: the bases do
  differ, but `osd_decode_npre1` fails on *either* basis. Kept as a
  documented, `#[cfg(test)]`-only diagnostic so it isn't
  re-investigated from scratch.)
  **Effect**: `K1BZM DK8NE -10` now decodes through the real
  production dispatch (`decode_frame_subtract_with_ap`'s AP-on
  multipass path, `qso3_apon_subtract_jtdx_extras_diag`: 5/6 → 6/6
  JTDX extras). **Cost**: ~30-40% wall-clock increase on candidates
  that reach the OSD fallback at all (most candidates decode earlier
  in the BP/OSD staircase and never reach this code path) —
  `ft8_qso3_staged_sic_check.rs` ~1.0s → ~1.4s. **One new
  near-ceiling phantom observed** (`qso3_busy.wav`, freq 2570.25 Hz,
  `hard_errors=30`, malformed callsign token — ~1 Hz from the real,
  strong `W1FC F5BZB` at 2571.38 Hz, almost certainly spectral
  leakage): same class of tradeoff already accepted in this file's
  `OSD_HARDERRORS_MAX` 22→36 history, where tightening the ceiling to
  suppress phantoms was found to silently discard genuine weak
  decodes living in the same hard-error range. Full workspace `cargo
  test --release --features full`: 100% pass, no regressions.

- **FT8 OSD dispatch: dropped the direct-channel-LLR loop, since it
  was never what real WSJT-X does either** (issue #182 follow-up).
  Checking `ft8b.f90`'s actual `do ipass=1,4` / `decode174_91.f90`'s
  `maxosd` branches confirmed the raw-channel-LLR OSD path
  (`maxosd=0`) is a *different* WSJT-X depth setting FT8's blind
  `ndepth=3` dispatch never takes (that always sets `maxosd=2`, which
  only ever calls `osd174_91` on `zsave(:,i)`). So the pre-#182
  `osd_strategy.rs::try_fallback` loop that tried the 4 llr variants
  directly against `osd_decode_npre1`/`_npre2` — predating this
  session, from issue #63 — was itself not WSJT-X-faithful for this
  depth; the `bp_llr_zsum` fallback added above was layered *on top*
  of it rather than replacing it. Removed the direct-channel loop
  entirely and restructured the `zsave(:,1)`/`zsave(:,2)` loop to nest
  in WSJT-X's actual order (outer = llr variant / `ipass`, inner =
  `zsave` index — `ft8b.f90:294-298`, `decode174_91.f90:137-148`),
  rather than the previous variant-innermost ordering. Full workspace
  regression: 100% pass, zero decodes lost — every candidate that used
  to succeed via the direct-channel path also succeeds via
  `zsave`-seeded OSD. **Net effect vs the immediately-preceding
  zsum-as-addition state**: 12 OSD dispatches worst-case → 8 (33%
  fewer), single-threaded `qso3_busy.wav` blind decode
  1.37s → 1.27-1.30s, still 22/22 decodes.

- **`decode_frame_subtract_with_auto_ap` marked explicitly opt-in-only**
  (issue #182 follow-up) — with the `bp_llr_zsum` OSD fix above, this
  function's own motivating case (`K1BZM DK8NE -10`) now decodes via
  blind `decode_frame_subtract_with_ap` alone: measured on
  `qso3_busy.wav`, blind-only and blind-plus-auto-AP both return the
  same 22 decodes, so the auto-AP pass currently finds *zero*
  additional signals there while still paying its own full search cost
  (~0.3-0.5s single-threaded on top of blind). It was never wired into
  any default decode path (always a separate, explicitly-called
  function), so this is a documentation-only change making that
  explicit rather than a behavioural one — but the doc comment now
  says plainly not to reach for it by default until a concrete
  zsum-unreachable case justifies its cost again.

- **Reverted the now-pointless auto-AP `rayon` parallelism and
  `AudioSample: Sync` bound** (issue #182 follow-up) —
  `decode_block::auto_ap_strategy`'s per-callsign `par_iter` loop was
  parallelising a search that, per the entry above, now finds zero
  additional decodes on `qso3_busy.wav`: real multi-core speedup with
  no offsetting value, plus a second (parallel/sequential) code path
  to maintain for a function not wired into any default path. Reverted
  to one sequential path. With that gone, `AudioSample`'s `+ Sync`
  supertrait (added specifically to let `&[S]` cross that `rayon` task
  boundary) has no remaining caller that needs it — every other
  `rayon` usage in the codebase (`core/sync.rs`, `ft8/decode.rs`,
  `core/pipeline.rs`) operates on concrete `i16` or non-`AudioSample`-
  generic types where `Sync` was already automatic. Reverted to `Copy`
  only. Full workspace regression: 100% pass.

- **Bit-packed `osd_setup_ldpc174_91`'s Gaussian elimination — 6.8×
  faster setup, ~4× faster overall OSD dispatch** (issue #182
  perf follow-up). Profiling `try_fallback` found it was ~22-25% of
  total decode wall-clock (~281ms of ~1.27s on `qso3_busy.wav`, ~200
  candidates × up to 8 zsave-seeded dispatches each, ~1% success rate).
  Splitting a single `osd_decode_npre1` call's cost further (2000-rep
  synthetic-LLR microbenchmark) found **86% of it was
  `osd_setup_ldpc174_91`** (sort + Gaussian elimination to build the
  MRB systematic generator) — 113.5µs/call — versus only ~18µs/call
  for the actual `npre1` combinatorial search (the OSD algorithm
  itself). Root cause: `OsdSetup.g` stored the GF(2) generator matrix
  as **one byte per bit** (`Vec<u8>`, 91×174 bytes), so the
  elimination's row-XOR step did up to 174 individual byte XORs per
  row instead of ~3 packed 64-bit-word XORs — paid fresh on every one
  of the 8 zsave-seeded dispatches per candidate, since each has a
  different LLR reliability ordering and can't share a cached basis.
  Note this isn't a WSJT-X-fidelity gap: `osd174_91.f90`'s own
  `genmrb` is `integer*1` (byte-per-bit) too — bit-packing is a
  legitimate algorithmic improvement *beyond* what WSJT-X itself does,
  not a port of anything.

  Rewrote only `osd_setup_ldpc174_91`'s internals to build and
  eliminate on a packed `[u64; 3]`-per-row representation, unpacking
  to the existing `Vec<u8>` `g` format at the end — `OsdSetup`'s shape
  and every downstream consumer (`osd_npre1_pass`, `osd_npre2_pass`,
  `build_npre2_table`, `try_candidate_ldpc174_91`) are byte-for-byte
  unchanged, confining all risk to one function. Verified with a new
  differential test (`packed_elimination_matches_byte_reference`)
  asserting the packed rewrite produces identical `perm`/`g`/
  `hdec_perm`/`absrx_perm`/`c_perm` to a frozen byte-per-bit reference
  copy of the pre-rewrite code, across 5 seeded synthetic LLR vectors
  plus all-zero and exact-tie edge cases — not just "does the
  regression suite still pass."

  **Result**: `osd_setup_ldpc174_91` 113.5µs → 16.7µs/call (6.8×);
  total OSD dispatch (setup + `npre1_pass`) 131.6µs → 34.1µs/call
  (3.9×). End-to-end single-threaded blind decode of `qso3_busy.wav`:
  **1.27-1.30s → 1.09-1.12s** — landing right at real `jt9 -8 -d3`'s
  own measured ~1.1s total file decode time. Still 22/22 decodes, zero
  regressions (full workspace `cargo test --release --features full`,
  `qso3_apon_subtract_jtdx_extras_diag` still 6/6 JTDX extras).

- **`fine_refine_3stage` (`ft8/refine_fine.rs`) sped up ~45% by porting
  WSJT-X's real Stage-B/C algorithm** (issue #182 follow-up). Pulling
  real `jt9 -8 -d3`'s own `timer.out` breakdown found its BP+OSD stage
  (`dec174_9`) was 65% of jt9's *own* total runtime — while our own
  BP+OSD staircase (after the fixes above) was already ~2.5x faster
  than jt9's, our `process_candidate` "prelude" (dominated by
  `fine_refine_3stage`) still ran at ~2.2x jt9's own per-candidate rate
  for the equivalent step. Root-caused by reading `lib/ft8/sync8d.f90`
  and `lib/ft8/ft8b.f90:104-154` directly: our port's Stage-B/C
  frequency sweep shifted the *entire* ~3200-sample `cd0` baseband
  buffer per trial (11 trials/candidate, ~35,200 elements of work),
  while WSJT-X's real algorithm tweaks only the **32-sample Costas
  reference waveform** per trial (`ft8b.f90:133-140`'s `ctwk`,
  multiplied into `sync8d.f90`'s cached `csync` before conjugating) and
  never touches the data buffer at all — ~100x less arithmetic by
  construction, not a missing optimization. `sync8d.f90`'s `save
  first,twopi,csync` also confirmed WSJT-X caches its Costas reference
  table once for the *entire process*, not once per candidate.

  Two incremental fixes earlier in this same investigation (a
  once-per-candidate Costas-table lookup, then an NCO-rotation
  approximation for the data-shift with a tolerance-validated
  differential test) had already cut wall-clock from ~966ms to ~753ms
  single-threaded on `qso3_busy.wav` — real wins, but optimizing the
  *wrong* algorithm faster rather than replacing it. Porting WSJT-X's
  actual reference-tweak approach obsoletes both: the Costas table is
  now cached for the process lifetime (`std::sync::OnceLock`, `no_std`
  builds keep the once-per-candidate fallback), and the frequency sweep
  tweaks a 32-element reference (`build_tweak`, mirroring `ctwk` exactly
  — including the *non*-negated sign convention, since the tweak lands
  on the conjugated reference rather than the data) instead of shifting
  `cd0`. The old NCO-approximated `shift_freq` (and its `tmp` buffer
  allocation) is deleted entirely — nothing left to call it.

  Verified via a new differential test comparing the reference-tweak
  formulation against a frozen copy of the exact data-shift approach it
  replaces: measured max relative error `5.3e-6`, tighter than the
  `1e-5` bound (this is floating-point reassociation of two
  *exactly-equivalent* formulations, not an approximation — unlike the
  NCO fix it replaces). All 5 pre-existing `fine_refine_3stage` tests
  (including the sign-sensitive `freq_snap_positive_offset`/
  `freq_snap_negative_offset`) pass unchanged. Also hoisted the same
  "don't recompute a value that hasn't changed" fix into
  `core::sync::fine_sync_power_per_block` (used for `sync_cv`,
  shared across protocols) — FT8's 3 identical Costas sync blocks no
  longer rebuild the same reference waveform 3x per call; content-equal
  (not pointer-identity) caching keeps it correct for any `Protocol`.

  **Result**: single-threaded blind decode of `qso3_busy.wav`:
  **~753ms → ~682-719ms (avg ~700ms)** — ~39% faster than real
  `jt9 -8 -d3`'s own ~1.1s total file decode time. Full workspace
  regression (100% pass), `qso3_apon_subtract_jtdx_extras_diag` (still
  6/6 JTDX extras), and the no_std/fixed-point feature matrix across
  every protocol touched by the shared `core::sync.rs` change: all
  clean.

  Two smaller, related fixes landed in the same investigation: (1)
  `process_candidate`'s prelude reordered so `sync_quality`'s
  `nsync<=6` gate runs right after the cheap sync-symbol-only extraction
  and before the expensive 58-data-symbol FFT extraction — `sync_quality`
  never reads the data symbols, so the ~82% of candidates that fail this
  gate on `qso3_busy.wav` no longer pay for work whose result is never
  used. (2) `try_fallback`'s OSD dispatch now reuses the LLR variants
  `process_one_candidate_inner`'s own BP staircase already computed
  (`llra`/`llrd`/`llrb`/`llrc`) instead of a fresh `compute_llr`
  recompute — that recompute previously only got skipped when an AP
  hint was present (Gemini PR #81's original intent, avoiding a
  *second* recompute between OSD and the AP loop), so blind decode (the
  common case) always paid for nsym=3's full cost twice per
  OSD-reaching candidate. Also cached `bp_step_select`'s
  `MFSK_BP_KIND` env-var lookup (`std::sync::OnceLock`) instead of a
  syscall-and-allocate `std::env::var` on every one of the up-to-4
  BP calls per candidate — a debug-only A/B switch that never needs to
  change mid-process.

- **`core::dsp::subtract::refine_freq` sped up ~2.2x/call, fixing an FT4
  decode-speed regression** (issue #182 follow-up). While re-verifying
  every protocol's decode-speed benchmark row after the
  `fine_refine_3stage` fix above (`core::sync.rs`'s cache hoist touches
  every protocol), FT4's `decode_frame_subtract` golden-WAV wall-clock
  turned out to have silently regressed from 48.8ms to ~526-576ms —
  caused by an earlier, unrelated, already-merged commit (issues
  #178-#180, migrating FT4 onto FT8's WSJT-X-faithful channel-aware LPF
  subtract for recall-quality reasons) that was never re-benchmarked
  afterward. Root-caused (not just documented) via temporary `Instant`
  timers around the SIC subtract loop's two calls per accepted
  candidate: `subtract_tones_lpf` was fine (already FFT-cached from
  issue #180, <1ms/call); `refine_freq` cost ~35ms/call — essentially
  the entire regression (×14 real decodes ≈ 490ms).

  `refine_freq` grid-searches ±5Hz (FT4) / ±2.5Hz (FT8) at 0.1Hz
  resolution to compensate for coarse-sync's bin-quantized carrier
  estimate before subtracting. Every one of its ~50-100 evaluations
  called `generate_iq` → `synth_complex_f32_into` fresh, fully
  rebuilding the GFSK-shaped modulation (erf-based Gaussian pulse
  table, `O(nsym·pulse_len)` per-symbol convolution, full `O(nwave)`
  phase-integration + `sin`/`cos` loop) even though only the carrier
  frequency differs between evaluations of one call — the same
  "recompute something invariant across a search loop" bug pattern as
  the `fine_refine_3stage` fix above, this time in the protocol-agnostic
  subtract path shared by FT4 and FT8.

  `generate_iq`'s carrier term is added uniformly to every sample before
  phase integration, so `phi(k; f0) = phi_mod(k) + k·(2π·f0·dt)` — any
  carrier is a pure linear phase ramp on top of the carrier-free
  (tone-modulation-only) phase. Fixed by building the carrier-free
  phasor once per `refine_freq` call and deriving each grid point via a
  cheap per-sample NCO rotation + angle-addition instead of a full
  resynthesis (`ls_amp_mag_tweaked`), with periodic rotor
  renormalization to bound f32 drift over the ~59k (FT4) / ~334k (FT8)
  sample buffers. Verified against the frozen full-resynthesis path
  (`ls_amp_mag`, now `#[cfg(test)]`-only) with a differential test using
  a combined absolute/relative tolerance — the LS amplitude has a comb
  of deep correlation nulls a few tenths of a Hz apart where relative
  error alone isn't meaningful — plus an argmax-preservation test
  confirming the search still finds the true off-grid carrier.

  **Result**: `refine_freq` ~35ms/call → ~15.7ms/call
  (`RAYON_NUM_THREADS=1`, isolating per-call cost from thread count).
  `decode_frame_subtract` real production wall-clock: multi-threaded
  ~575.8ms → ~280ms, single-threaded ~893.6ms → ~602ms. Not a full
  return to 48.8ms — the LPF subtract + freq-refine step is a
  deliberate, permanent recall-quality cost (10/10 vs 0/10 on a
  Rayleigh-faded-interferer scenario, `docs/notes/FT4_BENCHMARK.md`
  section 13) that will always cost more than the pre-#178
  constant-amplitude path; this fix removes the *redundant* part of
  that cost, not the cost itself. Candidate count unchanged (31/31, no
  redundancy) and recall unaffected (`ft4_wsjtx_sample_recall_vs_golden`
  still 6/6). Also re-confirmed FT8's own SIC/JTDX golden suite
  byte-identical (`qso3_apoff` 7/8+7 phantom, `qso3_jtdx` 18/18,
  `qso3_apon` 6/6 JTDX extras) since `refine_freq` sits on FT8's subtract
  path too.

- **`refine_freq`'s search radius was 5x too wide, closing out the FT4
  decode-speed regression** (issue #182 follow-up). The NCO fix above
  cut `refine_freq` from ~35ms/call to ~15.7-16.2ms/call, but FT4's
  `decode_frame_subtract` golden-WAV wall-clock was still ~280ms — 5.7x
  the pre-#178 baseline of 48.8ms. Isolated `refine_freq` /
  `subtract_tones_lpf` with a standalone microbenchmark (14 calls each,
  matching the golden WAV's real accepted-decode count): `refine_freq`
  alone was 227ms of the 280ms total (81%), `subtract_tones_lpf` only
  13ms (already FFT-cached). The NCO fix reduced *per-evaluation* cost;
  the *evaluation count* (101/call, ±5Hz radius at 0.1Hz step) was
  untouched.

  Cross-checked the call site's "+/-5 Hz refine radius…matches WSJT-X"
  comment against `lib/ft4/subtractft4.f90` directly: WSJT-X's
  `subtractft4` has **no frequency-refine step at all** — it subtracts
  directly at the decoded `f0`, no grid search. The comment's "matches
  WSJT-X" claim was wrong; `refine_freq` compensates for mfsk-core's own
  coarse-frequency resolution, not anything WSJT-X does in its subtract
  path. The ±5Hz figure traced to `refine_freq`'s generic doc comment
  (written for `core::sync::coarse_sync`'s ~2.93Hz FFT-bin uncertainty)
  and was never re-derived after FT4 moved onto `ft4_coarse_sync` +
  `core::sync2d::ft4_sync_search` (issue #72). Reading `ft4_sync_search`'s
  df search line by line: both its coarse (`idf` step 3) and fine (`si`
  step 1) passes only ever produce integer-Hz `df` values, so the
  `freq_hz` `refine_freq` receives is always within ±0.5Hz of the true
  continuous optimum by construction — a much tighter bound than the
  ±2.5Hz the borrowed comment assumed.

  Fix: `ft4::decode::decode_frame_subtract_with_options`'s
  `refine_freq_radius_hz` `5.0 → 1.0` (0.1Hz step unchanged — the
  response's mainlobe is well under 1Hz wide for a ~7.5s tone train, so
  widening the step risks skipping it; only the radius was oversized).
  Cuts the grid 101 → 21 evaluations/call. **Result**: golden-WAV
  `decode_frame_subtract` wall-clock **280ms → 110ms** (~2.5x further,
  2.3x off the 48.8ms floor vs. 5.7x before this fix), recall
  byte-identical (6/6 golden, 14/14 total decodes). The Rayleigh-fading
  busy-band regression guard the LPF-subtract migration (#177-179) was
  built to fix (`ft4_busy_band_fading_probe.rs::busy_band_fading_baseline`)
  stayed 10/10. See `docs/notes/FT4_BENCHMARK.md` section 16.

- **`ft8::decode::DecodeDepth` redesigned from a flat, ad-hoc 4-variant
  enum into an orthogonal 2-field struct (breaking); the automatic
  auto-AP rescue it used to gate was found unconditionally costing
  ~1.2s for zero recall benefit and removed entirely** (issue #182
  follow-up). Investigating why FT8's WSJT-X AP-off golden floor stays
  at 7/8 (`K1BZM DK8NE -10` missing) surfaced that the ship-config
  benchmark call uses `DecodeDepth::BpVariantsAd`, which skips OSD
  entirely by design — not a fidelity bug, DK8NE genuinely decodes
  under `BpAllOsd` (confirmed directly: `decode_block(..., BpAllOsd,
  ..)` finds it at `freq=244.2 dt=0.510`). But the old
  `DecodeDepth` enum (`BpAll`/`BpAllOsd`/`BpAllNoNsym3`/`BpVariantsAd`)
  conflated two independent concerns — which LLR variants to try, and
  whether to escalate to OSD — into 4 named combinations that couldn't
  express "cheap variants + OSD" or any combination the original
  author hadn't happened to name. `BpAllNoNsym3` (a middle tier between
  the cheap and full variant sets) had zero real callers outside one
  dedicated sweep test.

  **New shape**:
  ```rust
  pub enum LlrEffort { Minimal, Full }
  pub struct DecodeDepth { pub llr_effort: LlrEffort, pub osd: bool }
  impl DecodeDepth {
      pub const EMBEDDED: Self; // Minimal, osd: false — was BpVariantsAd
      pub const BP_ONLY: Self;  // Full, osd: false — was BpAll
      pub const FULL: Self;     // Full, osd: true — was BpAllOsd
  }
  ```
  `BpAllNoNsym3` has no replacement — collapsed into the 2-tier scheme
  after both its own sweep test's history and a fresh host measurement
  (`qso3_busy.wav`) showed the LLR variants it dropped (`b`/`c`,
  WSJT-X `ft8b.f90`'s own `llrb`/`llrc` naming) contribute +2.5ms/+5.5ms
  wall-clock for **zero** extra decodes over the cheap `a`+`d` pair —
  not enough signal to justify a third named tier.
  `osd: true` is host-only by construction, not just convention: the
  OSD dispatch code (`decode_block::osd_strategy`, gated the same way
  `auto_ap_strategy` already was) is now `#[cfg(feature =
  "fft-rustfft")]`-excluded from non-host builds entirely, so it's
  impossible for an embedded build to accidentally link in OSD's
  Gaussian-elimination/combinatorial-search machinery, and `osd: true`
  is a silent no-op there rather than a footgun. OSD has never shipped
  on an ESP32 target and there is no plan to add it — this was always
  a permanent architectural boundary, now enforced structurally.

  **Auto-AP removed entirely, not just re-gated.** While migrating
  callers, `depth.osd`'s gate on `auto_ap_strategy`'s harvest-callsigns-
  and-retry rescue (issue #117) turned out to be doing double duty: it
  wasn't just gating the deliberately opt-in
  `decode_frame_subtract_with_auto_ap` research function, but was also
  the *only* thing preventing `auto_ap_strategy::run` (a separate,
  unbounded, 4x/200-candidate-widening variant) from firing
  **unconditionally** inside `decode_block_multipass` — the driver
  shared by *every* `decode_block*` entry point, `ap_hint` or not —
  whenever `depth.osd` was true. A plain `decode_block(audio, ...,
  DecodeDepth::FULL, ...)` call with no AP involvement at all was
  silently paying this cost. Measured directly on `qso3_busy.wav`
  (`RAYON_NUM_THREADS=1`): with it, `decode_block(FULL)` took
  ~1320-1450ms for 19 decodes at `max_cand` 60/200; without it, ~145-
  151ms for the *same* 19 decodes — 9x wall-clock for zero recall
  difference at any realistic `max_cand` (only `max_cand=15` lost 2
  decodes, a budget artifact of the unbounded variant's own internal
  200-candidate floor, not real AP value). This matches what this
  file's own CHANGELOG already found for the explicit
  `decode_frame_subtract_with_auto_ap` path — zero additional decodes
  once the OSD `bp_llr_zsum` fix (below) closed this mechanism's
  original motivating case (`K1BZM DK8NE -10`) through a different
  route. Also not a WSJT-X port: `ft8apset.f90`'s AP only ever uses the
  *operator-configured* `mycall`/`hiscall`, never same-slot decoded
  callsigns. With zero measured value, zero FFI/embedded/production
  consumers, and a real correctness surprise (`ap_hint`-independent
  cost), `auto_ap_strategy` (module, `run`/`run_bounded`,
  `decode_frame_subtract_with_auto_ap`) was deleted outright rather
  than re-gated — an app wanting this policy can rebuild it from the
  still-present, genuinely WSJT-X-faithful primitives (`ApHint`,
  `decode_block_with_ap`).

  Migration: `DecodeDepth::BpAll` → `DecodeDepth::BP_ONLY`,
  `::BpAllOsd` → `::FULL`, `::BpVariantsAd` → `::EMBEDDED`,
  `::BpAllNoNsym3` → no replacement (use `::BP_ONLY` or `::FULL`).
  `mfsk-ffi-ft8`'s C-facing `MfskFt8Depth` enum is unchanged (only its
  internal `map_depth()` target type changed shape) — no ABI break.
  `core::pipeline::DecodeDepth` (FT4/FST4's own, separate, already-
  clean 2-variant enum of the same name in a different module) is
  untouched — deliberately out of scope, since neither protocol runs
  on embedded today and it doesn't have the conflation problem this
  redesign targets.

  Verified: full workspace `cargo test --release --features full`
  (100% pass) and `-D clippy::perf -D warnings` clean; golden recall
  byte-identical everywhere it was checked (`DecodeDepth::EMBEDDED`
  still 7/8 WSJT-X AP-off golden / 14 total on `qso3_busy.wav`; JTDX
  AP-on extras still 6/6; staged-SIC `CQ DX DL8YHR JO41` still decodes;
  `LlrEffort::Minimal` vs `Full` still 32/40 vs 32/40 recall parity
  across the full in-repo corpus, `ft8_no_nsym3_sweep.rs`, adapted to
  the 2-tier scheme rather than deleted); all 3 embedded app crates
  (`m5stack-s3-app`, `m5stack-core2-app`, `m5stack-cores3-app`) plus the
  compute-bench crate build clean for their Xtensa targets; `mfsk-ffi`
  + C++ smoke driver green.

  **`fixed-point` (Q11i16, the numeric path embedded ships) separately
  verified — not part of the `full` feature set, so not covered by the
  checks above.** All FT8 tests pass under `--features
  fft-rustfft,ft8,uvpacket,parallel,fixed-point`, including both new
  tests (`ft8_qso3_full_parity_recall.rs` needed the same `SNR_TOL_DB`
  12→14 dB fixed-point widening `ft8_qso3_apoff_recall.rs` already
  uses — golden recall itself is 8/8 under fixed-point too, only the
  SNR-drift assertion needed the existing tolerance pattern). One
  unrelated pre-existing failure found and confirmed *not* caused by
  this change (reproduced identically on `main` before this PR, via a
  throwaway git-worktree check): `ft8_coarse_sync_bootstrap.rs`'s
  `bootstrap_dt_median_top5_matches_confirmed` under `fixed-point` —
  filed as [#189](https://github.com/jl1nie/mfsk-core/issues/189),
  left unfixed as out of scope here.

  **Also updated this pass** (issue #182 follow-up, same day):
  `docs/notes/BENCHMARKS.md` and `docs/notes/FT8_BENCHMARK.md`'s FT8
  AWGN/CCIR sweep tables re-measured (all 4 channels moved 0.6-1.0 dB
  more sensitive vs the last-tracked figures — confirmed by scope
  audit + a direct re-run that `decode_frame_inner`'s separate call
  graph never touched `auto_ap_strategy`, so this is accumulated prior
  work never rolled into the table, not an effect of this PR); a new
  permanent regression, `tests/ft8_qso3_full_parity_recall.rs`, tracks
  the **host full-parity** config (`DecodeDepth::FULL`, `sync_min=0.8`,
  `max_cand=60`) hitting the full WSJT-X 8-entry golden in ~139-148 ms
  (~7-8× faster than real `jt9 -8 -d3`'s own ~1.1 s); `README.md` /
  `docs/reference/LIBRARY.md` / `docs/reference/EMBEDDED.md` (+ `.ja.md`
  mirrors) updated for the renamed `DecodeDepth` API and to stop
  conflating ship-config's permanent 7/8 floor with the achievable 8/8
  host figure in top-level summary tables (they're different code
  paths by construction now, not a temporary gap — see `DecodeDepth`'s
  own doc comment).

### Changed

- **`DecodeRequest::flat()`/`.staged()` → `.sic_rounds(n)`/`.sic_early()`,
  `SupportsFlatSic`/`SupportsStagedSic` → `SupportsSicRounds`/
  `SupportsSicEarly`** (issue #218, breaking). The flat-SIC engine's
  round count was hardcoded to 3 (`for ipass in 0..3` in
  `sic_inner_passes_with_cache`, mirroring WSJT-X's `do ipass=1,npass`
  with `npass` fixed) with no caller-tunable upper bound — the exact gap
  `WsjtxDepth`'s own doc comment flagged: jt9 `-d1` runs SIC with
  `npass=2` (vs. 3 for `-d2`/`-d3`), but neither `.flat()` nor
  `.staged()` exposed a 2-vs-3-round knob, so `WsjtxDepth::D1` silently
  ran the full 3 rounds instead of matching jt9 `-d1` exactly. Round
  count is now a required argument on the strategy-selecting method
  itself (`.sic_rounds(n)`, clamped 1..=3 — WSJT-X's own `npass`/`nsp`
  never exceeds 3) rather than an independently-settable field, so
  `.sic_rounds(_).sic_early()` — where the round count would be
  silently ignored by the early-decode strategy — is structurally
  unwritable rather than a compiling no-op. `WsjtxDepth::D1` now uses
  `.sic_rounds(2)`, closing the gap; `D2`/`D3` use `.sic_early()`
  (checkpoint structure is fixed at 3, not exposed as a knob).

  Renamed rather than kept as an additive alias: `flat`/`staged`
  described mfsk-core's own internal buffer-structure axis (single
  full buffer vs. checkpoint-replayed growing prefixes), not
  self-descriptive at a bare call site and not WSJT-X's own vocabulary.
  `sic_early` borrows WSJT-X's actual term for the checkpoint mechanism
  (`ndec_early`/`MAX_EARLY` in `ft8_decode.f90`, checkpointed at
  `nzhsym` = 41/47/50 out of 79 symbols); `sic_rounds` avoids `pass`,
  which is overloaded in WSJT-X's own source — FT8's `ft8_decode.f90`
  `ipass`/`npass` *is* the subtraction loop, but FT4's `ft4_decode.f90`
  reserves `ipass`/`npasses` for an unrelated AP-hint-variant loop
  inside a single decode attempt, using `isp`/`nsp` for the actual
  subtraction loop instead. `SupportsFlatSic`/`SupportsStagedSic` were
  renamed alongside the methods they gate so a trait-bound compile
  error names something a reader can connect back to the method they
  called.

  FT4's flat-SIC engine (`engine::pipeline::decode_frame_subtract`)
  gained the same `.sic_rounds(n)` knob — its progressive-`sync_min`-
  relaxation pass array (`&[1.0, 0.75, 0.5]`, mfsk-core's own
  pre-migration design, not WSJT-X's) is now sliced to `n` rounds
  rather than always iterated in full. FT4's SIC still isn't
  WSJT-X-faithful in its own right (`ft4_decode.f90` uses a **fixed**
  `syncmin=1.18` across rounds, the same fixed-threshold design FT8
  already migrated to — see `flat_sic_inner`'s doc comment) — tracked
  as a separate follow-up, out of scope here.

  `.sic_rounds(3)` verified byte-identical to the pre-rename `.flat()`
  default on `qso3_busy.wav` (20/20 matching decodes, checked against a
  clean pre-#218 `main` worktree); new `tests/ft8_sic_rounds_recall.rs`
  locks in both that golden and a `sic_rounds(1) ⊆ sic_rounds(2) ⊆
  sic_rounds(3)` monotonicity invariant (measured 13/19/20 decodes).
  `&[1.0, 0.75, 0.5][..max_rounds]` is genuinely new slicing logic on
  FT4's side, not just a rename, and the FT8 golden above doesn't
  exercise it — a self-contained (no external sample-tree dependency)
  six-station synthetic scene in new `tests/ft4_sic_rounds_recall.rs`
  covers the same monotonicity invariant there (measured 4/6/6
  decodes; asserts `sic_rounds(1) < sic_rounds(3)` so the scenario
  can't pass vacuously).

- **`DecodeRequest`/`SniperRequest::depth(DecodeDepth)` → `.osd(bool)`
  (breaking).** `LlrEffort` was a dead lever on host — its own doc
  comment already says the 2-/3-symbol LLR estimates it toggles
  "empirically add zero extra decodes"; a full-repo grep of every
  `.depth(...)` builder call site (crate tests, `mfsk-ffi`,
  `bench/wasm`) found zero cases ever passing `LlrEffort::Minimal`
  through either builder — it exists solely for `decode_block_into`'s
  ESP32 power budget (`DecodeDepth::EMBEDDED`). Both builders now
  hardcode `LlrEffort::Full` internally and expose only the `osd`
  toggle that callers actually used (`DecodeDepth::FULL`/`BP_ONLY` →
  `.osd(true)`/`.osd(false)`, default unchanged at `true`).
  `DecodeDepth` itself is untouched and still required positionally by
  `decode_block`/`decode_block_into` (the embedded/host-shared
  function API) and by `mfsk-ffi-ft8`'s C ABI (`MfskDecodeDepth`,
  unaffected — it's a separate 2-variant `#[repr(C)]` mirror that never
  crossed the builder boundary). `mfsk-ffi`'s internal `map_depth` helper
  is renamed `map_osd` (returns `bool`) to match; its C ABI surface is
  unaffected.
- **Q65-60B/30A `decode_multi_period_for` sped up ~4×**
  (`q65/rx.rs`, `tests/q65_wsjtx_samples.rs`) — two stacked fixes: (1)
  the fast-fading `b90 × model` sweep (6 combinations) was redundantly
  re-extracting FFT-based energies from scratch for each combination
  instead of extracting once and reusing; (2) `max_candidates=32` hit
  its cap on every slot, paying the full 8-stage decode ladder for
  candidates far below the real signal's own score (confirmed #0-ranked
  in every slot of both golden recordings via score-distribution
  profiling) — cut in two verified steps (32→16, then — after
  re-profiling showed the fading-BP stage still ~79% of wall-clock even
  after fix (1) — 16→8, matching `SearchParams::default()`). Golden-WAV
  wall-clock: Q65-60B 1.89s → 0.49s, Q65-30A 2.55s → 0.64s,
  bit-identical recall at every step.
- **Q65-60A/`decode_scan_for` family rewritten as a faithful `(Δf, Δt,
  b90)` grid search** (`q65/rx.rs::decode_at_grid_for`) — WSJT-X's
  `q65_loops.f90` has no separate "plain BP" path at all, always
  sweeping the fast-fading metric over a submode-specific b90 range in
  an `ndepth`-gated grid; our previous port diverged into a
  narrow-window AWGN-only Bessel metric with a time-only retry. Landing
  the faithful port surfaced three further bugs found via a real-`jt9`
  cross-check: a missing full/unpruned `ibw` sweep at the origin cell
  (WSJT-X's primary `q65_dec_q012` decode stage, which the pruned
  `q65_loops` fallback alone doesn't cover — worst for wide-`ibwa`
  C/D/E sub-modes); coarse-sync time resolution 4× coarser than
  WSJT-X's own `NSTEP=8` (`q65/search.rs::Spectrogram`, fixed alongside
  a `q65_ccf_22`-style restructure — per-frequency time-collapse +
  local-max NMS + noise-adaptive percentile admission — needed to avoid
  regressing a real off-air multi-signal recording); and a wrong fading
  model — `q65_dec1`/`q65_dec2` hardcode Lorentzian, not Gaussian, but
  the port used Gaussian, invisible for narrow-`b90` A/B sub-modes and
  costing wide-`b90` C/D/E sub-modes ~2.5-3 dB. With all three fixed,
  all ten sub-modes sit within ~1 dB of real `jt9`'s own crossing. Real
  off-air golden-test recall also *improved* (6 m EME sample: 3 → 4
  messages recovered). A follow-up `max_candidates` calibration
  (`tests/q65_wsjtx_samples.rs`, score-distribution profiling — same
  methodology as the Q65-60B/30A cut above) found all 4 real signals in
  the 6 m EME golden recording ranked within the top 8 of 530 coarse
  candidates; cut the test's `max_candidates` 32→16 (2× headroom,
  deliberately not pushed to the edge given a thin ~0.002 score margin
  at the weakest signal's rank), dropping golden-WAV time 1.49s→0.69s
  (~2.2×) with bit-identical recall. The same calibration then extended
  to the four fading-metric golden tests (`decode_scan_fading_for`,
  which don't go through `decode_at_grid_for` but share the coarse-sync
  overhaul's `coarse_search_for` and had gotten slower from it): real
  signals ranked within the top 3 of 94-3094 candidates in all four
  recordings (Q65-300A's margin over the next candidate is thin,
  ~0.0004, so its cut kept more headroom than the others). Cutting
  each test's `max_candidates` (100/30/30/200 -> 8/8/8/20) dropped
  Q65-60D 0.39s->0.08s, Q65-120D 0.15s->0.12s, Q65-120E 0.32s->0.26s,
  Q65-300A 1.05s->0.34s, bit-identical recall throughout. See
  `docs/notes/Q65_BENCHMARK.md`.
- **FST4-60A OSD depth-escalation gate hand-calibrated for its own
  `N_SYNC=40`** (`core/pipeline.rs`) — the shared `osd_depth3_min=18`
  gate was calibrated against FT8's `N_SYNC=21`; FST4's larger sync
  sequence made 18 a far looser bar, escalating roughly half of all
  candidates into the expensive OSD depth-3/4 tier regardless of signal
  quality. Golden-WAV decode time dropped **2.60 s → 0.27 s (~8.4×)**,
  recall verified unchanged (4-channel AWGN/CCIR sweep matching the
  documented pre-fix baseline almost exactly, FST4-120/300 spot-checked).
  A first attempt (reusing FT4's exact `N_SYNC`-scaled formula) measured
  as a real ~0.5 dB AWGN sensitivity regression and was corrected before
  shipping — see `docs/notes/FST4_BENCHMARK.md` section 8.
- **FT4 coarse-candidate stage replaced with a faithful
  `getcandidates4.f90` port** (`core::ft4_coarse::ft4_coarse_sync`) —
  the generic Costas-lag coarse_sync FT4 previously shared with
  FT8/FST4 was a structurally different algorithm from what WSJT-X
  actually does for FT4 (a 2-D freq×lag correlation search vs. WSJT-X's
  frequency-only periodogram), producing ~4.5× redundant candidates
  per real signal on busy recordings. Golden-WAV decode time dropped
  **1.20 s → 0.049 s (~25×)** with byte-identical 6/6 recall; AWGN 50%
  sensitivity crossing moved −17.2 dB → −16.9 dB (structural algorithm
  swap reshaped rather than uniformly improved the SNR curve — 3 of 4
  channels held or improved). See `docs/notes/FT4_BENCHMARK.md` section
  13.
- **Q65-60B/30A `decode_multi_period_for` sped up further, ~2.3×/~1.3×**
  (`q65/search.rs::Spectrogram::build_for`) — follow-up to a separate
  FFT-cache wiring investigation on FT8's `decode_block` (same "does
  this pattern exist elsewhere" question, applied to Q65). That
  specific pattern wasn't present here: `decode_at_grid_for`'s
  `GridDepth::Fast` has exactly one `(Δf,Δt)` grid cell per candidate
  (confirmed via direct call-count instrumentation — 16 calls for
  Q65-60A's 16 candidates, not the ~333 an earlier estimate assumed),
  so no redundant FFT-planner construction existed to cache. Phase-by-
  phase profiling of `decode_multi_period_for` found a different bug
  instead: `Spectrogram::build_for`'s noise-floor estimate (run once per
  candidate-search call) computed a trimmed mean of the bottom 95% of
  FFT-magnitude bins via a full `sort_unstable_by` (O(n log n)) over the
  entire magnitude array (hundreds of thousands to millions of cells for
  a 60-120 s slot) when only the unordered *set* of bottom-95% values
  was needed. Same class of fix already applied to FT8's
  `xsnr2_db_simple` noise-floor median — swapped to
  `select_nth_unstable_by` (O(n) average partition), Q65 just hadn't had
  it applied. Measured as 64% of wall-clock on Q65-60B (short slot, low
  candidate count) and 12% on Q65-30A (longer multi-slot audio where the
  BP/fading-metric stage dominates instead). Golden-WAV decode time:
  Q65-60B **0.28 s → 0.12 s**, Q65-30A **0.72 s → 0.56 s**, byte-
  identical recall on both (VK7MO/VK7PD on 60B, K1JT/K9AN AP-list on
  30A).

### Removed

- **Legacy BASIS (Q15 sin/cos dot-product) per-symbol DFT fill path**
  (#162) — dead weight since the 0.6.4 Goertzel migration (Phase
  1.7.7-Stick) made Goertzel the sole production fill on every
  embedded target, but the old path's functions and scratch
  parameters were never actually deleted. Removed
  `fill_symbol_spectra_into`/`fill_symbol_spectra_into_generic`,
  `BASIS_SCRATCH_LEN`, `symbol_spectra_direct_into`, the
  `mfsk_core::core::dotprod` module (`dot_q15_i32` + the
  `mfsk_core_dot_q15_i32` extern symbol contract), and its esp-dsp
  ASM bridge in `embedded-shared::esp_dsp_fft`
  (`mfsk_core_dot_q15_i32`, `dsps_dotprod_s16_ae32`,
  `ESP_DSP_DOT_SHIFT`/`set_dot_shift`). `fill_symbol_spectra_generic`
  (the plain rotator-based DFT) is now unconditional — it no longer
  needs a separate `fixed-point`-gated BASIS-backed overload.
- **Breaking FFI change**: `mfsk_ft8_decode_i16`'s C ABI
  (`mfsk-ffi-ft8`) dropped its `basis_re`/`basis_im` `int16_t*`
  scratch parameters — they were dead weight once Goertzel took
  over, and the crate's `mfsk_ft8_basis_scratch_len()` sizing
  function is gone with them. **C callers must drop both arguments
  from their call site** when upgrading past 0.7.x. This forces the
  minor version bump for this release (0.7.4 → 0.8.0) ahead of the
  otherwise-patch-level content below.
- `basis_re`/`basis_im` parameters dropped from every Rust function
  in the call chain down to that FFI boundary:
  `decode_block_into[_tuned]`, `refine_candidates_into`,
  `process_candidates_into[_tuned]`,
  `process_candidates_into_with_cs_scratch[_tuned]` (all
  `mfsk-core`), and `dual_core::init`/`pass2_split`/`stage3_split`/
  `run_speculative_slot` (`embedded-shared`) — all embedded app
  crates (`m5stack-s3-app`, `m5stack-core2-app`, `m5stack-cores3-app`)
  updated to match; none of them were actually using the scratch for
  anything by this point (Goertzel needs none), so this is a pure
  signature simplification with no behavioral change on any target.
- `mfsk-core/tests/ft8_goertzel_vs_basis.rs` deleted — its entire
  purpose was A/B-validating BASIS against Goertzel during the
  Phase 1.7.7-Stick migration, which is moot once BASIS no longer
  exists to compare against.
- **`core::pipeline`'s unreachable generic-refine fallback + dead
  `refine_candidate_double`** (issue #192 investigation) — auditing
  whether FT8's decode engine should unify with `core::pipeline` (as
  #192 originally proposed) found the reverse problem first: the
  generic engine already carried speculative generality nothing used.
  An exhaustive call-graph audit (every call site in `src`, `tests`,
  and doc examples) confirmed two dead branches:
  - `process_candidate_basic`'s bare `refine_candidate::<P>` fallback
    (time-only refine, no frequency correction) — every real caller is
    `Ft4` or an FST4 sub-mode, both of which already take the
    frequency-aware `ft4_sync_search`/`fst4_sync_search` branch; FT8
    never calls into this pipeline at all (it has its own bespoke
    engine). Replaced the runtime branch with a new sealed-by-convention
    marker trait, `core::pipeline::GenericPipelineProtocol: Protocol`,
    implemented only for `Ft4` and the five FST4 sub-modes — calling
    `process_candidate_basic`/`decode_frame`/`decode_frame_subtract`
    with any other protocol is now a compile error instead of silently
    falling back to an unvalidated code path.
  - `core::sync::refine_candidate_double` and its `FineSyncDetail`
    result type — a generalized "double sync" (independent first-block/
    last-block refine + drift estimate) with zero callers anywhere in
    the crate; FT8's own copy of this idea was already removed in #49.
    Deleted both outright, along with the now-pointless `FineSyncDetail`
    re-export from `ft8::sync`.
  - The now-dead `refine_steps` parameter threaded through
    `process_candidate_basic`/`decode_frame`/`decode_frame_subtract`
    (only consumed by the removed fallback) was also dropped from all
    three signatures — `Ft4`/FST4 callers no longer pass it.
  - **Not done**: full FT8/`core::pipeline` engine unification, as #192
    originally scoped it. That would mean porting FT8-only machinery
    (blind-CQ AP pass, non-AP OSD fallback, lazy LLR-effort staircase,
    two-phase sync/data fill, frequency-aware 3-stage refine) into a
    layer with no second consumer — the opposite direction from this
    cleanup. Issue #192 closed with this narrower fix instead; FST4 SIC
    and `DecodeResult` semantic unification remain tracked separately
    in #193/#194.

    **Correction, next day (2026-07-27): #192 reopened.** A re-audit
    against the then-current code found 3 of the above close
    rationale's 4 "FT8-only, no second consumer" claims didn't hold
    up — the blind-CQ AP pass and non-AP OSD fallback both already had
    independent, parallel implementations serving FT4/FST4
    (`msg::pipeline_ap`'s pass 7, `core::pipeline::osd_escalation_gates`
    respectively) at the time this entry was written, and the lazy
    LLR-effort staircase was ported into `core::pipeline` the very
    next day (`4801722`, folded into this same 0.8.0 cycle) — only the
    frequency-aware 3-stage refine (`fine_refine_3stage`) is still
    genuinely FT8-only. Closing #192 made this duplication invisible
    instead of resolving it; it remains open as of this writing, not
    resolved by the narrower fix described above. As of 2026-08-08,
    still unstarted (`fine_refine_3stage` has had zero commits since
    the reopen) — see the issue thread for the full re-audit.
- **Breaking**: demoted the pre-#191 raw engine functions `DecodeRequest`/
  `SniperRequest` wrap to `pub(crate)` (issue #203, part of the pre-0.8.0
  public-API review tracked in #206) — `core::pipeline::{decode_frame,
  decode_frame_subtract, process_candidate_basic, osd_escalation_gates}`,
  the `GenericPipelineProtocol` trait, and `msg::pipeline_ap::{
  decode_sniper_ap, ap_bits_for, ap_passes, process_candidate_ap}` were
  still reachable at their pre-#191 legacy-shaped call sites even though
  #191 removed the protocol-module wrappers around them. `DecodeRequest`/
  `SniperRequest` are the only supported entry points now. Also
  `#[doc(hidden)]`'d `uvpacket::rx::{diag_sync_at, diag_estimate_freq_offset}`
  and `ft8::decode_block::process_candidates_into_with_cs_scratch{,_tuned,
  _tuned_with_fill}` (kept `pub`, not `pub(crate)`, since `embedded-poc`'s
  `embedded-shared::dual_core` depends on the `_tuned` variant as an
  external path dependency outside the workspace). A new non-`full`
  `internal-testing` feature keeps the demoted `core::pipeline` items
  `pub` for `mfsk-core/tests/{fst4_sweep,ft4_sweep,fst4_wsjtx_samples}.rs`,
  which call them directly as diagnostics — CI enables it alongside
  `full` for `cargo test`/`cargo clippy --all-targets` only; `cargo doc`/
  `cargo publish` stay on the `pub(crate)` shape downstream consumers
  actually see.
- **Breaking**: `core::pipeline::FftCache` (re-exported at
  `ft8::decode::FftCache`) is now an opaque newtype instead of `pub type
  FftCache = Vec<Complex<f32>>` (issue #206, pre-0.8.0 public-API
  review) — the old alias leaked `num_complex::Complex`, a dependency's
  type, into the public API. No public constructor and no way to
  inspect the contents; obtain one from a `decode_frame`-family return
  value / `DecodeOutcome::fft_cache` and pass it straight back into
  `DecodeRequest::fft_cache`. Added `FftCache::len`/`is_empty` for the
  one query downstream code plausibly needs.
- #203's `pub(crate)` demotion (above) surfaced `-D warnings`
  `dead_code` failures across the `feature-matrix` CI job's
  single-protocol builds (`fst4`, `jt9`, `jt65`, `q65`, `uvpacket`, the
  `alloc ft8 fft-extern[,fixed-point]` embedded presets) — several
  `core::pipeline`/`msg::pipeline_ap` items are only reachable via
  `ft4`/`fst4`'s `decode` modules, so any combination excluding both
  now correctly sees them as unreachable (previously masked because
  `pub` items are exempt from the lint regardless of in-crate callers).
  `#[allow(dead_code)]` added at each such item.
- **Breaking**: renamed the `mfsk_core::core` module to `mfsk_core::engine`
  (issue #206, pre-0.8.0 public-API review) — `pub mod core` shadowed
  Rust's own `core` crate at every scope where both names were
  simultaneously visible (the crate root, and anywhere writing a bare
  `core::` path meaning std rather than this module), which is exactly
  the kind of surprise a public API shouldn't hand downstream
  consumers. Mechanical rename: `core::pipeline`, `core::protocol`,
  `core::sync`/`sync2d`, `core::dsp::*`, `core::llr`, `core::equalize`,
  `core::scalar`, `core::fft`, `core::tx`, `core::ft4_coarse`,
  `core::baseline`, and the flattened re-exports (`DecodeContext`,
  `FecCodec`, `FecOpts`, `FecResult`, `FrameLayout`, `MessageCodec`,
  `MessageFields`, `ModulationParams`, `Protocol`, `ProtocolId`,
  `SyncBlock`, `SyncMode`) all move to `engine::*` with no behavior
  change. `embedded-poc` (path-dependency, outside the workspace)
  updated to match — not compile-verified here (no `+esp` toolchain in
  this environment); run `cargo check` there before flashing.
- **Breaking**: unified `ft8::decode::DecodeResult` with
  `engine::pipeline::DecodeResult` (issue #194, pre-0.8.0 public-API
  review) — FT8's own struct was byte-for-byte identical to the
  generic pipeline type (used by FT4/FST4) except for
  `message77: [u8; 77]` (CRC bits stripped) vs. the generic type's
  `info: Box<[u8]>` (full `K` FEC info bits, CRC retained) +
  `message77()` accessor slicing the leading 77. FT8's own BP/OSD
  engine already produced the full `info` at its one production
  construction site (`fec::ldpc::bp::BpResult::info`) — it was just
  being discarded in favor of the 77-bit-only field. Rather than just
  matching the shape, FT8 now literally re-exports
  `engine::pipeline::DecodeResult`, so a protocol-generic caller over
  `DecodeRequest<P>` can read every protocol's results the same way.
  `mfsk_core::msg::wsjt77::{unpack77, unpack77_with_hash}` and
  `mfsk_core::ft8::wave_gen::message_to_tones` relaxed from `&[u8; 77]`
  to `&[u8]` to match — a strict widening (any existing `&[u8; 77]`
  caller still compiles via unsized coercion) that also lets
  `result.message77()`'s `&[u8]` return flow in directly, without the
  copy-into-a-scratch-`[u8; 77]`-array dance FT4/FST4 callers
  previously needed. `WsprDecode`/`Jt9Decode`/`Jt65Decode`/`Q65Decode`
  are deliberately untouched — none of those protocols adopted
  `DecodeRequest`/`FrameDecodable` (#191), so unifying their naming
  with this family is deferred to #204's Q65 builder design pass
  rather than done piecemeal here.
- **Sealed `FecCodec`** (issue #198, pre-0.8.0 public-API review) — a
  private `sealed::Sealed` supertrait bound, implemented for all seven
  in-crate implementors (`Ldpc174_91`, `Ldpc240_101`, `Ldpc128_90`,
  `ConvFano`, `ConvFano232`, `Rs63_12`, `Q65Fec`), blocks downstream
  crates from implementing `FecCodec` themselves. `decode_soft` is
  still f32-hardcoded — genericizing it (`decode_soft<T: LlrScalar>`,
  which would unlock fixed-point BP for FT4/FST4/MSK144 through the
  generic pipeline, currently FT8-only via its own bespoke engine) is
  deliberately deferred: real numerical work with its own verification
  cost, decided (2026-07-27) as a stretch goal rather than a 0.8.0
  requirement. Sealing now means that redesign can land later as a
  signature change on existing implementors without breaking any
  downstream implementor, since none can exist.
- **Breaking**: migrated Q65's decode API to a `DecodeRequest`/
  `SniperRequest`/`MultiPeriodRequest` builder (issue #204, pre-0.8.0
  public-API review) — the largest single item in the review. Q65 still
  carried the full pre-#191 `_with_*`/`_for` suffix explosion (15 public
  entry points in `q65::rx`: `decode_at`, `decode_at_for`,
  `decode_at_fading_for`, `decode_at_with_ap`, `decode_at_with_ap_for`,
  `decode_at_with_ap_list_for`, `decode_multi_period`,
  `decode_multi_period_for`, `decode_scan`, `decode_scan_default`,
  `decode_scan_for`, `decode_scan_fading_for`, `decode_scan_with_ap`,
  `decode_scan_with_ap_for`, `decode_scan_with_ap_list_for`) — the exact
  combinatorial disease #191 collapsed into `DecodeRequest<P>` for
  FT8/FT4/FST4, shipping 0.8.0 with two contradictory decode-API
  philosophies otherwise.

  Q65 gets its **own** builder types (`q65::{DecodeRequest, SniperRequest,
  MultiPeriodRequest}`) rather than reusing
  `msg::decode_request::{DecodeRequest, SniperRequest}`: every `q65::rx`
  function (and the FFI layer wrapping it) operates on `&[f32]` audio,
  but the WSJT77-family builders hardcode `audio: &'a [i16]` — a real
  architectural difference (Q65's own decode chain works in float
  throughout, unlike the integer WSJT77-family path), not an oversight.
  `decode_multi_period*`'s `&[&[f32]]` (one buffer per T/R slot) is a
  further distinct shape from either wide-scan or single-target decode,
  hence the third builder type.

  - `DecodeRequest::<P>::new(audio, sample_rate, nominal_start_sample,
    params)` replaces `decode_scan*`; `.sniper(...)` (or
    `SniperRequest::<P>::new`) replaces `decode_at*`.
  - `.ap_hint(&ApHint)`, `.ap_list(&[[i32; 63]])`, and
    `.fading(model, b90_ts)` are plain inherent methods (not
    capability-gated marker traits like FT8/FT4/FST4's
    `SupportsWideBandAp`): every Q65 sub-mode supports every capability
    uniformly, so there is no invalid combination to guard against at
    the type level. A new `Q65SubMode: Protocol` marker (implemented for
    the 10 wired sub-mode ZSTs) stops the builders from compiling
    against a non-Q65 protocol and silently producing garbage.
  - `MultiPeriodRequest::<P>::new(audio_slots, sample_rate,
    nominal_start_sample, params)` + `.ap_list(...)` replaces
    `decode_multi_period*`.
  - The Q65-30A convenience wrappers (`decode_at`, `decode_at_with_ap`,
    `decode_scan`, `decode_scan_with_ap`, `decode_scan_default`,
    `decode_multi_period`) are gone entirely — `DecodeRequest::<Q65a30>`
    is not meaningfully more to type.

  Migration (issue #207 — call-site reshapes lose `rustc`'s "did you
  mean" hint that plain renames get, so worked examples save the
  source-reading detour):

  ```rust
  // before: wide-band scan
  decode_scan_for::<Q65a30>(&audio, sr, start, &params)
  // after
  DecodeRequest::<Q65a30>::new(&audio, sr, start, params).decode()

  // before: single-target sniper
  decode_at_for::<Q65a30>(&audio, sr, start, freq_hz)
  // after
  SniperRequest::<Q65a30>::new(&audio, sr, start, freq_hz).decode()

  // before: wide-band scan + fast-fading metric + AP hint
  decode_scan_fading_for::<Q65a60>(&audio, sr, start, &params, b90_ts, model, Some(&ap_hint))
  // after
  DecodeRequest::<Q65a60>::new(&audio, sr, start, params)
      .fading(model, b90_ts)
      .ap_hint(&ap_hint)
      .decode()
  ```

  All 15 functions demoted to `pub(crate)` (the underlying engine, used
  internally by the new builders). Updated every call site: 9
  `mfsk-core/tests/q65_*.rs` integration tests (~85 call sites) and
  `mfsk-ffi`'s Q65 FFI dispatch (~50 call sites across the 10-sub-mode ×
  4-capability match tables) — the C ABI itself is unchanged, only the
  internal Rust wiring moved to the new builders.

  Verified clean across all 14 feature-matrix combinations, clippy
  `--workspace --all-targets`, full test suite + doctests (including
  every Q65 WSJT-X real-off-air-sample recall gate: 10 GHz EME, 6 m
  ionoscatter/EME, 1296 MHz troposcatter, 120D rainscatter, optical
  scatter), `mfsk-ffi`/`mfsk-ffi-ft8` test suites, and `cargo doc`.
- **Breaking**: unified `mfsk-ffi` and `mfsk-ffi-ft8`'s C ABI
  conventions (issue #205, pre-0.8.0 public-API review) — the two FFI
  crates had independently evolved incompatible shapes for the same
  domain: clashing status vocabularies (`MfskStatus` vs
  `MfskFt8Status`, both using `-1..-4` for different meanings),
  duplicate result structs with different text-ownership models
  (`mfsk-ffi`'s `MfskMessage` held a heap `CString*` per message;
  `mfsk-ffi-ft8`'s `MfskFt8Result` always used a fixed inline buffer),
  and decode entry points that hardcoded every tuning knob
  positionally with no room to grow.

  New zero-dependency `no_std` crate, `mfsk-ffi-abi`, is now the
  single source of truth for the shared shape: `MfskStatus` (`Ok`,
  `NullPointer`, `InvalidArg`, `UnknownProtocol`, `DecodeFailed`,
  `Internal`), `MfskDecodeDepth` (`BpAll`/`BpAllOsd`, mirroring
  `engine::pipeline::DecodeDepth`), `MfskResult`/`MfskResultList`
  (fixed 40-byte inline `text` buffer, not a heap pointer — the whole
  list is one allocation freed in one call), and an opaque
  `MfskDecodeOptions` handle. Not published, not a C ABI on its own —
  each consuming crate `pub use`-re-exports these types so its own
  cbindgen-generated header carries identical definitions; not
  parsed together in the same C translation unit (only one of
  `mfsk.h`/`mfsk_ft8.h` is linked per target).

  - `mfsk-ffi-ft8`: dropped its local `MfskFt8Status`/`MfskFt8Depth`/
    `MfskFt8Result`/`MfskFt8ResultList` in favor of the shared types.
    `mfsk_ft8_decode_i16` (host feature) and the pre-existing
    `mfsk_ft8_decode_i16_alloc` are unified into one symbol name,
    `mfsk_ft8_decode_i16`, taking an `MfskDecodeOptions*` (NULL = this
    crate's built-in default) instead of five positional tuning
    arguments — **C callers must rename `mfsk_ft8_decode_i16_alloc`
    call sites and switch to an options handle.** Host error strings
    moved from `thread_local!` (unavailable in `no_std`) to a
    documented single-threaded `static mut` buffer written via raw
    pointer arithmetic (`&raw mut`/`copy_nonoverlapping`, satisfying
    the `dangerous_implicit_autorefs` lint).
  - `mfsk-ffi`: dropped its local `MfskStatus`/`MfskMessage`/
    `MfskMessageList` in favor of the shared types (renamed
    `MfskMessageList`→`MfskResultList`, `MfskMessage`→`MfskResult`,
    `mfsk_message_list_free`→`mfsk_result_list_free`). `MfskResult::text`
    is now a fixed inline buffer instead of a heap `CString*` — no
    more per-message `CString::from_raw` in the free path, matching
    `mfsk-ffi-ft8`'s existing model. **Breaking**: `mfsk_decode_f32`/
    `mfsk_decode_i16` gained a new `options: *const MfskDecodeOptions`
    parameter (construct with `mfsk_decode_options_new`, release with
    `mfsk_decode_options_free`) — **C callers must add a NULL (or
    real) argument at every call site.** NULL preserves each
    protocol's pre-0.8.0 hardcoded defaults exactly (FT8: 200-3000 Hz/
    sync_min 2.0/max_cand 50/`BpAllOsd`; FT4: sync_min 1.2; FST4-60A:
    100-3000 Hz/sync_min 0.8/max_cand 30); a non-null handle overrides
    freq range/sync_min/max_cand uniformly, and depth for FT8. Q65's
    generic-handle path (`decode_q65_default`) also honours the
    freq-range/max_cand override; `decode_wspr`/`decode_jt9_aligned`/
    `decode_jt65_aligned` and the dedicated `mfsk_q65_*` function
    family are untouched (no tunable search knobs to wire, or a
    separate ABI surface out of scope for this issue).

  Caught during verification, fixed as part of this same change:
  both crates' `cbindgen.toml` had `parse_deps = false`, which made
  cbindgen silently emit function signatures referencing the shared
  types **without ever `typedef`-ing them** in the generated header
  (a `pub use`-re-exported type isn't visible to cbindgen unless it
  parses the defining crate) — `MfskResult` in particular collapsed
  to a field-less opaque forward declaration because its `text` field
  used a `MFSK_TEXT_CAP + 1` compound array-length expression cbindgen
  can't evaluate across a crate boundary (a bare literal works fine,
  hence `mfsk-ffi-abi::MFSK_TEXT_BUF_LEN` — a derived constant used
  Rust-side, with a `const _` assertion keeping it in sync — while the
  struct field itself stays a literal `40`). Fixed by turning on
  `parse_deps = true` with `include = ["mfsk-ffi-abi"]` (not a bare
  `parse_deps = true`, which would additionally — and needlessly —
  parse all of `mfsk-core` through the `mfsk-ffi`/`mfsk-ffi-ft8`
  dependency graph). Verified both regenerated headers now carry
  full, byte-identical (differing only in indentation style per
  crate's `cbindgen.toml`) definitions for every shared type.

  Updated every call site: `mfsk-ffi/tests/{q65_ffi,wsjt_ffi}.rs`,
  `mfsk-ffi-ft8/tests/streaming.rs`, `mfsk-ffi/examples/cpp_smoke`
  (built + run, including the `RUN_FST4_ROUNDTRIP=1` gated path),
  `mfsk-ffi-ft8/tests/c_smoke/{smoke.c,tx_rx_round_trip.c}` (built +
  run against a real WAV and a synthesised round-trip),
  `mfsk-ffi-ft8/examples/streaming_recipe.c` (compile-checked; it's a
  documentation artefact with no `main`), `mfsk-ffi/examples/kotlin_jni/
  native/mfsk_jni.c` + its `README.md`, and `embedded-poc/embedded-shared/
  src/apps/compute_bench.rs`'s FFI smoke path (path-dependency outside
  the workspace, not compile-verified here — no `+esp` toolchain in
  this environment; run `cargo check` there before flashing).
  `docs/reference/{LIBRARY,EMBEDDED}.md` and their `.ja.md`
  counterparts updated to match, including a pre-existing stale
  `NULL, NULL, // Goertzel` leftover in `EMBEDDED.md`'s streaming
  recipe (from the #162 BASIS-scratch removal, unrelated to this
  issue but caught in the same doc pass).

  Verified clean across the `mfsk-core` feature matrix (unaffected by
  this issue, reconfirmed), clippy `--workspace --all-targets`, full
  test suite (`mfsk-ffi`/`mfsk-ffi-ft8`/`mfsk-ffi-abi`), `cargo doc`,
  and the C/C++ smoke drivers above.
- **Breaking**: unified the four legacy protocols' decode-result type
  names onto one naming convention (issue #206, pre-0.8.0 public-API
  review, decided 2026-07-27) — `WsprDecode`→`WsprResult`
  (`wspr::decode`), `Jt9Decode`→`Jt9Result` (`jt9`),
  `Jt65Decode`→`Jt65Result` (`jt65`), `Q65Decode`→`Q65Result`
  (`q65::rx`), matching `engine::pipeline::DecodeResult`'s `*Result`
  suffix instead of the WSJT77-family's leftover `*Decode`.
  Naming-convention-only: `decode_at`/`decode_scan`/`decode_slot`
  entry-point verbs are unchanged, and all four types keep their
  existing protocol-specific shapes — WSPR/JT9/JT65 carry an
  already-unpacked human message (`WsprMessage`/`Jt72Message`) plus
  mode-specific timing metadata via their own `decode_at`/`decode_scan`
  engines, which (unlike FT8/FT4/FST4) never route through
  `engine::pipeline`; `Q65Result` carries an unpacked `String` message
  rather than raw FEC info bits. A full structural merge onto
  `engine::pipeline::DecodeResult` would require first porting
  WSPR/JT9/JT65 onto the generic pipeline engine — the same scale of
  work as issue #192's FT8 proposal, times three protocols — and was
  explicitly scoped out of the 0.8.0 window as too large/risky for the
  time remaining before the release cut; tracked as future work rather
  than bundled here.

### Fixed

- **JT65 total decode failure, root-caused and fixed** (#24). Every
  `jt65sim`-generated (or otherwise WSJT-X-compatible) JT65 signal
  failed to decode regardless of SNR, even when `search::coarse_search`
  landed candidates within ~1 Hz / a few symbols of ground truth.
  Root cause: `jt65::interleave::interleave`/`deinterleave` (the 7×9
  transpose WSJT-X's `interleave63.f90` implements) had their
  permutations swapped relative to WSJT-X's TX/RX convention
  (`gen65.f90`/`jt65sim.f90` call `interleave63(sent, idir=1)` at TX;
  `extract.f90` calls `interleave63(mrsym, idir=-1)` at RX). The swap
  was internally self-consistent — TX and RX remained exact mutual
  inverses of each other — so every self-roundtrip test passed while
  real/independent-reference signals decoded to garbage RS codewords.
  Structurally identical to the JT9 encoder bug (#19): a decode path
  only ever cross-checked against this crate's own encoder, never an
  independent reference.
- New `tests/jt65_sweep.rs` (`#[ignore]`d, mirrors `ft8_sweep.rs`) +
  `scripts/gen_jt65_sweep_wavs.sh` build a `jt65sim`-generated AWGN
  SNR-sweep corpus and characterise the post-fix recall curve: 100%
  recall down to 0 dB, 50% around -14 dB, near-zero below -19 dB (all
  in `jt65sim`'s 2500 Hz reference bandwidth convention).
- Found and documented, but did **not** port in this fix: WSJT-X's
  own hard-decision-only path (`jt9 -6`, no `kvasd`) holds ~100%
  recall down to -22 dB on the same corpus — a further ~7-8 dB
  sensitivity gap. Root cause traced to `lib/ftrsd/ftrsdap.c`, a
  stochastic Chase decoder (randomized multi-trial soft-symbol
  erasure-pattern search around Berlekamp-Massey RS) that WSJT-X runs
  even without `kvasd` — a materially different (and more involved)
  algorithm than this crate's single-pass confidence-ordered erasure
  decode (`decode_at_with_erasures`). Tracked as a follow-up (#169);
  see the sweep test's doc comment for the full provenance.
- **Q65 AWGN sensitivity gap, root-caused and substantially
  narrowed** (#171). The initial sweep found a real, reproducible
  ~2-3 dB gap vs. WSJT-X's own plain-BP decode (`jt9 -3 -p 30 -b A`)
  for Q65-30A specifically (50% recall crossing ~-24 dB vs. WSJT-X's
  ~-26 to -27 dB), confirmed directly on a single failing file (not a
  batch-script artifact). The entire FEC/BP stack was verified
  byte-for-byte correct against `WSJT-X/lib/qra/q65/{qracodes.c,
  npfwht.c,pdmath.c}` first (all 10 code tables, the WHT, and every
  `pdmath` primitive — diffed programmatically, zero discrepancies),
  which pointed the search upstream: `coarse_search_for`'s reported
  best `(start_sample, freq_hz)` is measurably imprecise at low SNR
  — off by up to ~1/5 of a symbol period — and neither
  `decode_scan_for` nor `decode_at_for` ever refined that alignment
  before attempting decode. WSJT-X's `q65_loops` never trusts its own
  coarse alignment either — it always runs a local fine-timing
  (`idt`) retry loop before the real decode attempt. Added
  `decode_at_with_fine_timing_for` (tries the reported alignment
  first, then a symmetric ±3-step retry grid in units of `nsps/16`)
  to `decode_scan_inner`, the shared implementation behind every
  scan-level Q65 entry point. Full 990-file sweep re-run: all six
  wired sub-modes improved by roughly 1-2 dB at their threshold
  region (e.g. Q65-60A 20%→100% at -27 dB; Q65-30A 40%→93% at -24 dB,
  0%→33% at -25 dB).
- **Remaining residual gap traced to a comparison-methodology
  difference, not a further implementation bug.** Even after the
  fine-timing fix, a gap persisted at the deep end (e.g. Q65-30A ~0%
  at -26 dB vs. WSJT-X's ~40%). Traced this to WSJT-X's Q65 decode
  chain always having access to the free "CQ ??? ???" AP hypothesis
  (`aptype=1` in `extract.f90`'s AP table — needs no user-supplied
  callsign), so *every* real `jt9` decode attempt implicitly gets
  some AP-list benefit on CQ-calling signals, whether or not the user
  configured `-c`/`-x`. `decode_scan_for` (this crate's genuinely
  blind baseline) has no equivalent for-free hypothesis by design —
  the four decoder strategies stay deliberately separate
  (`docs/reference/LIBRARY.md` §3). Re-measured with
  `decode_scan_with_ap_for` + a `"CQ"` hint (now also reported by
  `tests/q65_sim_sweep.rs` as a second column) and it closes the gap
  almost exactly across all six sub-modes: Q65-30A -26 dB 0%→40%
  (matches WSJT-X's reported 40%), -25 dB 33%→93% (WSJT-X: 87%),
  -24 dB 93%→100% (WSJT-X: 100%); Q65-60A -29 dB 7%→53%, -28 dB
  47%→93%; Q65-60B/C/D/E show the same ~40-50 point jump at their
  respective thresholds. **Usage note, not a code change**:
  applications wanting WSJT-X-equivalent behavior for CQ traffic
  should call the AP-hinted path with at least a `"CQ"` hint rather
  than the plain one — matching what WSJT-X's own decoder always
  does internally. Issue #171 left open with this as the closing
  analysis (no further action expected; re-open if a real remaining
  gap is found with matched AP context on both sides).
- **`DecodeRequest<Ft8>::strictness()` was a documented no-op for
  FT8's non-AP decode path** (#221, found while benchmarking WebFT8's
  decode presets). `.strictness(Deep)` produced identical recall and
  wall-clock to `.strictness(Strict)` on every scenario tested — FT8's
  BP staircase and OSD fallback used hardcoded `36`
  (`OSD_HARDERRORS_MAX`/`WSJTX_NHARDERRORS_MAX`, WSJT-X's own
  `ft8b.f90:422` ceiling) unconditionally, dead since #188 removed the
  code that used to consume a strictness-tiered version;
  `DecodeStrictness::ap_max_errors` (the AP loop's own gate) was the
  only method FT8 ever actually called. New
  `DecodeStrictness::ft8_nharderrors_max()` wires all four BP-variant
  acceptance checks and the OSD dispatch to the per-request
  strictness: `Normal = 36` (unchanged default, zero regression —
  full FT8 golden/JTDX/full-parity/depth-ladder regression suite
  green), `Strict = 22` (real prior art reused from the issue #72
  investigation, not a fresh guess), `Deep = 40` (mfsk-core-original,
  exceeds WSJT-X's own ceiling, not yet swept against a fading
  corpus). FST4 remains unaffected by any `DecodeStrictness` method by
  design (#146).

### Added

- **Q65-15A**: a new sub-mode ZST (`Q65a15`, 15 s T/R period, ×1 tone
  spacing = 6.667 Hz), the fastest wired Q65 mode — one line via the
  existing `q65_submode!` macro, following the established
  15/30/60-s-period axis `q65params.f90` already defines. Wired
  through the FFI (`MfskQ65SubMode::A15`, appended after `E60` rather
  than inserted before `A30` to keep the `#[repr(C)]` enum's existing
  discriminant values stable) and the `PROTOCOLS` registry
  (`"Q65-15A"`). No real off-air recording exists for this period in
  WSJT-X's sample tree (same situation as Q65-60C/60E already
  documented), so no golden-WAV test is possible — covered instead by
  a dedicated `tests/q65_a15_roundtrip.rs` (synth + aligned/offset
  scan recovery) and folded into the `q65sim`-based AWGN sweep below
  (50% crossing ≈ -21 dB, matching `q65params.f90`'s analytical
  formula for the 15 s period).
- **Q65-120D, Q65-120E, Q65-300A**: three longer-period sub-modes
  (`Q65d120`/`Q65e120`/`Q65a300`), chosen because WSJT-X's own user
  guide (`doc/user_guide/en/protocols.adoc`) and sample tree document
  real, specific use cases for exactly these three: Q65-120D (10 GHz
  rainscatter/troposcatter, backed by 14 files in WSJT-X's own
  `UnitTests.txt` regression corpus), Q65-120E (6 m ionoscatter),
  and Q65-300A (optical/laser scatter — the deepest wired Q65
  sub-mode, ~-34 dB AWGN threshold, matching the published table
  value almost exactly). Unlike Q65-15A, these three **do** have
  golden-WAV tests: `tests/q65_wsjtx_samples.rs` gained
  `rainscatter_10ghz_120d_decodes_with_fading_metric`,
  `ionoscatter_6m_120e_decodes_with_fading_metric`, and
  `optical_scatter_300a_decodes_with_fading_metric` — golden messages
  ("VK3WE VK7MO QE37", "KB7IJ N0AN 73", "VK7MO VK7PD QE38")
  independently confirmed via `jt9 -3 -p {120,300} -b {D,E,A}` first.
  All three need the fast-fading metric to decode (plain BP fails,
  same shape as the existing Q65-60D EME test) — for Q65-300A this
  holds even though it's stable-path scatter rather than classic
  Doppler-spread EME, suggesting the fading metric's robustness helps
  generally at threshold-adjacent SNR, not only under true multipath.
  Now **10 wired Q65 sub-modes total**; docs (`LIBRARY.md`/`.ja.md`),
  `tests/protocol_invariants.rs`, FFI (`MfskQ65SubMode::{D120,E120,A300}`,
  discriminants 7-9), and the `q65sim` AWGN sweep all updated to match
  (the 120/300 s configs use 5 trials instead of 15 — their WAVs are
  proportionally larger and the #171 fine-timing retry multiplies
  decode cost further).
- **Direct WSJT-X cross-check for Q65-120D/120E/300A**: ran `jt9 -3
  -p {120,300} -b {D,E,A}` (no `-c`/`-x`) over the identical 165-file
  sweep corpus per sub-mode used above. Result: **no regression, and
  two sub-modes exceed WSJT-X's own plain decode**. Q65-300A's curve
  is statistically identical to `jt9`'s at every tested SNR point
  (both cross 50% at ≈-35 dB). Q65-120D and Q65-120E's `decode_scan_for`
  50% crossings (≈-30.7 dB, ≈-31.0 dB) are **2.5-3.4 dB better** than
  `jt9 -3`'s own plain-decode crossings (≈-28.2 dB, ≈-27.6 dB) —
  consistent across 4+ SNR points each, not sampling noise. Likely
  explanation (not fully confirmed): the #171 fine-timing retry tries
  a fixed ±3-step grid per coarse candidate regardless of T/R period,
  which may end up relatively more thorough than WSJT-X's own
  `q65_loops.f90` `idt`/`idf` retry granularity at these slower-baud,
  longer-period sub-modes specifically.
- New `scripts/gen_q65_sweep_wavs.sh` + `tests/q65_sim_sweep.rs`
  (`#[ignore]`d): a `q65sim`-generated AWGN SNR sweep covering every
  sub-mode ZST this crate actually wires (`Q65a15`, `Q65a30`,
  `Q65a60`, `Q65b60`, `Q65c60`, `Q65d60`, `Q65e60`, `Q65d120`,
  `Q65e120`, `Q65a300` — WSJT-X's Q65 also has other (period,
  sub-mode) combinations this crate doesn't implement, so the sweep
  intentionally covers only what's shipped). `q65sim` has a real
  CMakeLists.txt target (unlike `jt9sim`), so no new build script was
  needed — build via `cmake --build ~/wsjtx-build --target q65sim`.
  Below period=30 s, `q65sim` uses a completely different filename
  format (`000000_MMSS.wav`, not the sequential index used at ≥30 s)
  — `gen_q65_sweep_wavs.sh` special-cases this for Q65-15A.
- **New `tests/q65_wsjtx_samples.rs::tropo_1296_60b_decodes_via_averaging`**:
  Q65-60B was the only wired sub-mode with a real off-air recording
  already vendored (`WSJT-X/samples/Q65/60B_1296_Troposcatter/`) but
  no corresponding test. Single-slot decode fails on this dataset (as
  expected, same as the existing 10 GHz EME test's plain path); the
  multi-period EMA-on-spectrogram path (`decode_multi_period_for`,
  same mechanism the existing ionoscatter test uses) recovers the
  golden message "VK7MO VK7PD QE38" cleanly.
- Q65-60C and Q65-60E have no real off-air recording anywhere in
  WSJT-X's sample tree, so no golden-WAV test is possible for those
  two sub-modes (same situation JT65 was already in before this
  session — no real recording exists to validate against).
- New `scripts/build_jt9sim.sh` + `scripts/gen_jt9_sweep_wavs.sh` +
  `tests/jt9_sweep.rs` (`#[ignore]`d, mirrors `tests/jt65_sweep.rs`):
  a `jt9sim`-generated AWGN SNR sweep for JT9. Unlike `ft8sim`/
  `ft4sim`/`fst4sim`/`jt65sim`, `jt9sim` has no CMakeLists.txt target
  in WSJT-X at all — `build_jt9sim.sh` assembles its actual dependency
  closure (`gen9` → `packjt`/`entail`/`encode232`/`interleave9`/
  `graycode`, plus `jt9fano`/`fano232` for jt9sim's own internal
  self-verify step) as a standalone binary from source.
- Result: `decode_scan_default` holds **100% recall down to -24 dB**,
  crossing 50% around -26 dB — closely tracking WSJT-X's own `jt9 -9`
  on the identical 300-file corpus (100% to -25 dB, 80% at -26 dB; the
  per-cell differences are within 20-trial sampling noise at the
  steep part of the curve, not a systematic gap). Confirms JT9 has
  **no JT65-style hidden sensitivity gap** — the three remaining
  misses in the real-recording golden test
  (`tests/jt9_wsjtx_samples.rs`) are congestion/wrong-codeword-lock
  issues specific to that busy recording, not a general AWGN
  weakness.
- Also re-confirms the #19 encoder fix (`pack_grid4_plain`/
  `unpack_grid`) against a second, independently-built reference
  encoder: a fresh `jt9sim` signal ("CQ JL1NIE PM95" @ 1400 Hz)
  decodes cleanly, matching WSJT-X's own `jt9 -9` output exactly.

### Fixed

- **Q65 AWGN sensitivity gap, root-caused and substantially
  narrowed** (#171). The initial sweep found a real, reproducible
  ~2-3 dB gap vs. WSJT-X's own plain-BP decode (`jt9 -3 -p 30 -b A`)
  for Q65-30A specifically (50% recall crossing ~-24 dB vs. WSJT-X's
  ~-26 to -27 dB), confirmed directly on a single failing file (not a
  batch-script artifact). The entire FEC/BP stack was verified
  byte-for-byte correct against `WSJT-X/lib/qra/q65/{qracodes.c,
  npfwht.c,pdmath.c}` first (all 10 code tables, the WHT, and every
  `pdmath` primitive — diffed programmatically, zero discrepancies),
  which pointed the search upstream: `coarse_search_for`'s reported
  best `(start_sample, freq_hz)` is measurably imprecise at low SNR
  — off by up to ~1/5 of a symbol period — and neither
  `decode_scan_for` nor `decode_at_for` ever refined that alignment
  before attempting decode. WSJT-X's `q65_loops` never trusts its own
  coarse alignment either — it always runs a local fine-timing
  (`idt`) retry loop before the real decode attempt. Added
  `decode_at_with_fine_timing_for` (tries the reported alignment
  first, then a symmetric ±3-step retry grid in units of `nsps/16`)
  to `decode_scan_inner`, the shared implementation behind every
  scan-level Q65 entry point. Full 990-file sweep re-run: all six
  wired sub-modes improved by roughly 1-2 dB at their threshold
  region (e.g. Q65-60A 20%→100% at -27 dB; Q65-30A 40%→93% at -24 dB,
  0%→33% at -25 dB).
- **Remaining residual gap traced to a comparison-methodology
  difference, not a further implementation bug.** Even after the
  fine-timing fix, a gap persisted at the deep end (e.g. Q65-30A ~0%
  at -26 dB vs. WSJT-X's ~40%). Traced this to WSJT-X's Q65 decode
  chain always having access to the free "CQ ??? ???" AP hypothesis
  (`aptype=1` in `extract.f90`'s AP table — needs no user-supplied
  callsign), so *every* real `jt9` decode attempt implicitly gets
  some AP-list benefit on CQ-calling signals, whether or not the user
  configured `-c`/`-x`. `decode_scan_for` (this crate's genuinely
  blind baseline) has no equivalent for-free hypothesis by design —
  the four decoder strategies stay deliberately separate
  (`docs/reference/LIBRARY.md` §3). Re-measured with
  `decode_scan_with_ap_for` + a `"CQ"` hint (now also reported by
  `tests/q65_sim_sweep.rs` as a second column) and it closes the gap
  almost exactly across all six sub-modes: Q65-30A -26 dB 0%→40%
  (matches WSJT-X's reported 40%), -25 dB 33%→93% (WSJT-X: 87%),
  -24 dB 93%→100% (WSJT-X: 100%); Q65-60A -29 dB 7%→53%, -28 dB
  47%→93%; Q65-60B/C/D/E show the same ~40-50 point jump at their
  respective thresholds. **Usage note, not a code change**:
  applications wanting WSJT-X-equivalent behavior for CQ traffic
  should call the AP-hinted path with at least a `"CQ"` hint rather
  than the plain one — matching what WSJT-X's own decoder always
  does internally. Issue #171 left open with this as the closing
  analysis (no further action expected; re-open if a real remaining
  gap is found with matched AP context on both sides).
- **WSPR AWGN SNR sweep test infra** (`tests/wspr_sweep.rs`,
  `scripts/gen_wspr_sweep_wavs.sh`), closing the one gap left where
  WSJT-X ships a simulator but this crate had no simulator-driven
  objective benchmark — every other supported protocol
  (FT8/FT4/JT9/JT65/Q65/MSK144/FST4) already had a `*sim`-based AWGN
  sweep; WSPR's only prior validation was a single real-world WAV's
  fixed-SNR golden recall (`wspr_wsjtx_samples.rs`). WSJT-X's
  CMakeLists.txt only wires the C `wsprsim` (`lib/wsprd/wsprsim.c`),
  which writes `.c2` complex-baseband files for `wsprd`, not WAV audio
  this crate's decode path consumes — the WAV-capable simulator is a
  second, orphaned Fortran program (`lib/wsprd/wsprsimf.f90`, no
  CMake target, same situation as `jt9sim`) that this crate now
  builds standalone via `scripts/build_wsprsim.sh` (picking out its
  actual dependency closure, plus a local no-op `watterson` stub to
  avoid pulling in FFTW for a code path — the `.c2` branch — this
  build never exercises). 13-point AWGN sweep, 20 trials/point:
  100% recall from 0 dB down to -27 dB, 95% at -28/-29 dB, 40% at
  -30 dB, 0% at -31 dB and below — consistent with WSJT-X's published
  WSPR sensitivity floor.

### Tests

- **`mfsk-ffi` Rust-level ABI test coverage for the non-Q65
  protocols** (`mfsk-ffi/tests/wsjt_ffi.rs`). Previously only Q65 had
  a `cargo test`-driven FFI test (`q65_ffi.rs`); FT8/FT4/WSPR/JT9/
  JT65/FST4-60A were exercised solely by the C++ smoke driver
  (`examples/cpp_smoke`). Added encode/decode round-trips for all
  six (mirroring the C++ driver's known-good test vectors), an
  `mfsk_decode_i16` case (the f32 path was the only one covered
  before), and NULL/bad-input negative-path tests
  (`mfsk_decode_f32` with a null decoder/samples pointer,
  `mfsk_encode_ft8` with an unpackable callsign, freeing null
  pointers). Also fixed a CI gap found while adding this: the `ffi`
  job built `mfsk-ffi` and ran the C++ driver but never ran
  `cargo test -p mfsk-ffi` at all, so `q65_ffi.rs` itself was not
  actually executing in CI; `.github/workflows/ci.yml` now runs it
  (FST4's slow slot decode stays behind the same
  `RUN_FST4_ROUNDTRIP` gate the C++ driver already used).
- **`mfsk-ffi-ft8` wired into the same `ffi` CI job** — previously
  had zero CI coverage of any kind (not even a build step), despite
  shipping a `tests/streaming.rs` suite (7 tests, ring-buffer /
  resample / chunk-boundary coverage for the `mfsk_ft8_stream_*`
  API) that only ever ran locally. Added `cargo build`/`cargo test -p
  mfsk-ffi-ft8` steps using the crate's default `host` feature only
  — the `embedded-fixed-point` feature builds `no_std` and has no
  `cargo test` harness to run on a CI runner, so the
  xtensa-esp32/esp32s3-espidf cross-compiles stay build-only-verified
  in `release.yml` as before (they can't be executed without real
  hardware anyway).

### Changed

- **`mfsk-ffi`'s version unstuck from a stale `0.1.0`** — it had
  never been bumped since the crate's initial commit despite
  gaining Q65 sub-modes and other features release after release.
  Introduced `[workspace.package].version` in the root `Cargo.toml`
  as the single source of truth; `mfsk-core`, `mfsk-ffi`, and
  `mfsk-ffi-ft8` now all declare `version.workspace = true` instead
  of a hand-copied literal, so a release version bump is one edit
  instead of three manually-synced ones (the exact class of bug that
  let `mfsk-ffi` drift). `release.yml`'s tag-vs-Cargo.toml gate
  (`verify-tag` job) updated to read the workspace version from the
  root `Cargo.toml` instead of `mfsk-core/Cargo.toml` directly, since
  the latter no longer carries a literal version string.
- **`mfsk-ffi` release-build policy decided: desktop/mobile stays a
  single all-decoder library, and it's now distributed the same way
  as `mfsk-ffi-ft8`.** Confirmed design split — `mfsk-ffi-ft8` stays
  the embedded-only FT8 slice, `mfsk-ffi` stays the desktop/mobile
  superset covering all seven WSJT modes (Kotlin/Android's binary
  footprint isn't a concern here, unlike the embedded no_std targets;
  Android release binaries deliberately deferred — see below) — and
  closed the distribution gap this implied for the desktop target:
  `release.yml` gained `build-ffi-desktop-host` (linux-x86_64:
  `libmfsk.{so,a}` + `mfsk.h`), gating `publish` alongside the
  existing `mfsk-ffi-ft8` build jobs so a broken build blocks the
  release instead of leaving it half-published. `mfsk-ffi/README.md`,
  root `README.md`, and `docs/reference/LIBRARY.md` §8-9 updated to
  describe the new prebuilt-binary distribution instead of "clone and
  build". Android (arm64 via NDK cross-build) was drafted in the same
  pass — a working `build-ffi-desktop-android` job was written and
  locally verified for target/toolchain-name correctness — but
  deliberately left out of this release-build pass; Kotlin/Android
  consumers keep building `mfsk-ffi` locally with cargo-ndk per
  `mfsk-ffi/examples/kotlin_jni/README.md` for now.
- **`examples/kotlin_jni/` scaffold renamed to match the
  `wsjt-ffi` → `mfsk-ffi` crate rename it had missed** — found while
  drafting the Android release build above. `Wsjt.kt` /
  `wsjt_jni.c` / package `io.github.rsft8n` still referenced the
  pre-rename crate name (`cargo build -p wsjt-ffi`, `libwsjt.so`,
  `WsjtMessageList`, …), none of which exist anymore; the scaffold's
  own documented build commands would not have run. Renamed to
  `Mfsk.kt` / `mfsk_jni.c` / package `io.github.mfskcore` throughout,
  matching what `docs/reference/LIBRARY.md` §9 already (aspirationally)
  described. Also expanded `Mfsk.Protocol` from `FT8`/`FT4` only to
  all seven `MfskProtocol` values, matching `mfsk-ffi`'s actual
  all-decoder scope.

### Fixed

- **FT8 `decode_block` multi-pass decode ~10.5× faster on busy-band
  recordings, WSJT-X-faithful subtract algorithm.** A decode-speed
  investigation (prompted by `qso3_busy.wav` benchmarking 4.73 s —
  suspiciously slow for a 15 s slot on host hardware) profiled
  `decode_block`'s three-pass successive-interference-cancellation
  loop phase by phase. Coarse-sync, fine-refine, and per-candidate
  BP/OSD were all sub-5 ms each; the entire cost (~310 ms per
  accepted decode, ~13 calls on this recording) was
  `core::dsp::subtract::subtract_tones_lpf` — a naive O(candidates ×
  NFRAME × lpf_half) direct time-domain convolution for the
  channel-tracking LPF (≈608 M MACs/call at FT8's `lpf_half=2000`).
  Checked WSJT-X's own `lib/ft8/subtractft8.f90` /
  `lib/ft4/subtractft4.f90`: both use a *cached* filter-response FFT
  (built once, `first`/`save` in the Fortran) plus one forward +
  inverse FFT of the full slot per call — O(N log N), not O(N×M).
  Ported that algorithm exactly (including FT8's `endcorrection`
  edge-response boost, which `subtractft4.f90` omits and the port
  now matches per-protocol via a new `endcorrection: bool` parameter
  on `subtract_tones_lpf`) behind the existing `fft-rustfft` (host)
  feature; the direct-convolution version is kept as the `no_std`
  fallback, unchanged. No embedded target is affected either way —
  `decode_block`'s embedded (`not(fft-rustfft)`) variant of
  `decode_block_multipass` has no subtract loop at all (single-pass,
  no SIC), confirmed by grepping `embedded-shared`'s actual decode
  pipeline before assuming otherwise.
  `ft8_qso3_apoff_recall.rs`'s `qso3_apoff_meets_wsjtx_golden_floor`:
  **4.73 s → 0.45 s**, byte-identical recall (7/8 golden, 7 phantom,
  14 total). Also caught and fixed a second, independent staleness
  bug found during the same investigation: this test's comment
  claimed to match "the Core2 / S3 production path" while passing
  `DecodeDepth::BpAll`; the actual embedded ship config is
  `DecodeDepth::BpVariantsAd` (introduced later, skips the two most
  expensive BP LLR variants for failed candidates) — updated the
  test to match, verified identical recall either way. FT4's golden
  test is on a different (non-LPF, constant-amplitude) subtract path
  in `core::pipeline` and was unaffected by this change.
- **FT8 `decode_block` a further ~3.6× faster on busy-band recordings
  — two FFT-cache wiring gaps closed.** Follow-up profiling of
  `decode_block_multipass` (post the 10.5× subtract fix above) found
  its cost had moved, not disappeared: (1) `refine_candidates` and
  the per-candidate `process_candidates_tuned_with_ap[_ref]` calls
  were all passing `fft_cache: None`, so each of up to ~45
  candidates/slot re-ran the 192 k-point forward FFT from scratch
  even though the sibling `fine_refine_pass1` stage in the same pass
  already built and reused exactly this cache — a wiring gap, not a
  missing algorithm. Threaded the existing
  `build_fft_cache`/`downsample_cached` helpers through
  `refine_candidates`, `process_candidates_tuned_with_ap[_ref]`, and
  `auto_ap_strategy::run`, rebuilding the cache lazily only when the
  WSJT-X-mandated sequential subtract actually mutates the working
  buffer (not eagerly every pass) — alone dropped
  `qso3_apoff_meets_wsjtx_golden_floor` 0.43 s → ~0.12–0.22 s. (2)
  `subtract_tones_lpf_fft`'s filter-response FFT was already cached
  (`cached_window_fft`, from the fix above), but its forward/inverse
  `FftPlanner` *plans* for `nfft = 180 000` were rebuilt on every
  call — measured at ~2.8 ms/call for plan construction vs ~0.7 ms
  for the transform itself, i.e. plan-rebuild cost ~4× the FFT it
  was gating. Cached the plans in an `nfft`-keyed `OnceLock`, the
  same pattern `fill_symbol_spectra.rs`'s `SYMBOL_FFT_32` already
  used for the per-symbol 32-pt FFT. Combined: **0.43 s → 0.12 s
  (~3.6×)**, byte-identical recall (7/8 golden, 7 phantom, 14 total
  on `qso3_apoff`; 5/6 JTDX AP-on extras unchanged on `qso3_apon`).
  Both fixes are cache wiring, not new algorithms, so risk is low;
  `fft-rustfft` is `std`-gated but thread-free, so both caches also
  apply under `wasm32-unknown-unknown` (build-verified). No embedded
  impact either way, since `decode_block_multipass`'s
  `not(fft-rustfft)` variant has no subtract loop (single-pass, no
  SIC) and never calls these paths.
- **FT8 CCIR moderate/poor sweep sensitivity gap closed, root-caused
  as a comparison-methodology gap, not a numerical deficiency**
  (#190). The ~0.6-0.7 dB gap vs real `jt9 -8 -d3` on the
  `ft8sim`-driven CCIR moderate/poor 50%-crossing sweep traced (via a
  locally-instrumented `jt9` build, `ISSUE190_PROBE` prints following
  the `DL8YHR_PROBE` precedent from #180) to `jt9`'s CLI hardcoding
  `lft8apon=.true.` (`lib/jt9.f90:302`, independent of the GUI's own
  default-off `FT8AP` setting) — every `jt9 -8 -d3` invocation used
  throughout this project's WSJT-X-parity work has implicitly
  included a free "CQ ??? ???" AP hypothesis pass (WSJT-X iaptype-1,
  needs no operator-supplied mycall/hiscall), and the sweep corpus's
  message (`CQ JL1NIE PM95`) is exactly the class that pass targets.
  Traced nsync (14/16/18) matching jt9 almost exactly on the 3
  `ccir_moderate_m19_{01,05,14}.wav` trials jt9 wins and mfsk-core
  lost — ruling out the fine-sync/coherent-combining lead
  `FT8_BENCHMARK.md` section 10 had flagged — and found jt9's own
  blind `ipass=1..4` also failed identically on all 3, succeeding
  only via `ipass=5` (`iaptype=1`). mfsk-core's
  `process_one_candidate_inner` already had the matching "Pass 12:
  blind-CQ" logic, but it was gated behind `ap_hint.is_some()`, so
  plain `decode_frame` (no AP hint) never reached it. Not a revival of
  the auto-AP mechanism removed earlier in this release — that was
  iaptype-2 self-seeding from same-slot callsigns, an mfsk-core
  original with no WSJT-X counterpart; this is iaptype-1, a faithful
  port. Fix: pass 12 now runs whenever the blind BP/OSD staircase
  fails **and** `sync_quality (nsync) ≥ BLIND_CQ_MIN_NSYNC (12)`,
  independent of `ap_hint`. The nsync floor is a cost gate added after
  measuring an un-gated first version: on `qso3_busy.wav`'s multipass
  staged-SIC path it sent 188 candidates through this pass (140 at
  nsync 7-9, none producing a decode there — real recoveries needed
  nsync 14-18), pushing wall-clock 0.7 s → 1.5 s, slower than real
  jt9's own ~1.15 s on the same file. Gating at nsync≥12 cuts that to
  34 candidates and ~0.85 s (faster than jt9 again), zero recall
  change either way. Re-measured 50%-crossings (same 780-file corpus,
  gated version): AWGN -21.4→-21.6 dB, CCIR good -20.8→-21.1 dB, CCIR
  moderate -18.9→-20.0 dB, CCIR poor -19.0→-19.7 dB — moderate now
  ahead of real `jt9 -8 -d3`, poor at parity (was behind on both). No
  recall regression on any FT8 golden test (WSJT-X AP-off 7/8, JTDX
  18/18, full-parity 8/8, staged-SIC 18/18, AP-on JTDX-extras 6/6);
  `qso3_busy.wav` single-pass `DecodeDepth::FULL` wall-clock ~139-141
  ms → ~165-175 ms (~6-7× faster than real `jt9 -8 -d3`'s ~1.1-1.2 s).
  See `FT8_BENCHMARK.md` section 11 for the full trace and cost
  investigation.
- **`engine::dsp::downsample::downsample_cached` rebuilt its inverse-FFT
  plan (twiddle table) from scratch on every call instead of reusing one
  across a session** (#211), found while profiling the wasm `+simd128`
  benchmark harness (#208, Stage C — `node --prof` with real function
  symbols). Called once per FT8 candidate (~90×/slot), it constructed a
  fresh `default_planner()` and called `plan_inverse(cfg.fft2_size)`
  every time, even though `fft2_size` never varies within a decode
  session — discarding `rustfft::FftPlanner`'s own per-size cache each
  call and rebuilding the twiddle table via scalar `sin`/`cos`/`rem_pio2`.
  Same anti-pattern already fixed twice nearby (`SYMBOL_FFT_32` in
  `fill_symbol_spectra.rs`, `subtract_tones_lpf_fft`'s filter-response
  plan above) — `default_planner()`'s own doc comment already says to
  "reuse the same instance across all decodes in a session so rustfft's
  twiddle cache hits," this call site just didn't. Fixed with a
  `std`-gated `thread_local!` planner, reused across calls on the same
  thread; `no_std`/`fft-extern` (embedded) callers are unaffected — the
  only hot-loop caller (`fill_symbol_spectra_via_cd0`) is itself
  `fft-rustfft`-gated. Measured impact: ~12.5% of total wasm decode
  wall-clock (`node --prof` flat profile, `qso3_busy.wav`), bigger than
  any dense-kernel SIMD target found in the same profiling pass; not
  wasm-specific, the same redundant work happens on every target. Golden
  recall unchanged: FT8 full-parity 8/8, AP-off 7/8 (7 phantom, 14
  total), JTDX 18/18 (1 extra) — byte-identical to pre-fix.
- **`engine::llr::fill_bmet_for_nsym`'s max-reduction loop rewritten to
  actually vectorize under `wasm32 +simd128`** (#208 Stage D). The
  per-element `if (i >> bit_sel) & 1 == 1 { max_one[..] = v } else {
  max_zero[..] = v }` blocked LLVM's vectorizer two ways at once:
  branchy per-lane control flow, and an `i`-outer/`bit_sel`-inner loop
  nest that would need outer-loop vectorization LLVM doesn't attempt
  here. Making the branch branchless (feed `f32::NEG_INFINITY` to
  whichever accumulator the bit doesn't select, then an unconditional
  `max()`) alone changed nothing — confirmed empirically via
  `wasm-objdump`, the compiled region's `v128` count was unchanged.
  Swapping the loop order (`bit_sel` outer, `i` inner — a flat
  reduction over `s2` with a loop-invariant `bit_sel`) combined with
  the branchless rewrite is what actually unblocked it: 40 → 86 `v128`
  ops in the function's compiled region. This was the a-priori top
  Part-2 target from #208's dense-kernel survey, but profiling (Stage
  C) found its real ceiling is small (~2.9% of total decode time,
  `qso3_busy.wav`, wasm), well below what a `node --prof` flat profile
  found in an unrelated FFT-plan-caching bug (#211, fixed separately)
  — wall-clock impact here is within run-to-run measurement noise even
  though the vectorization itself is real and verified.
  Byte-identical recall on all three FT8 golden regressions
  (full-parity 8/8, AP-off 7/8/7-phantom/14-total, JTDX 18/18/1-extra)
  and the full `--features full` test suite.
- **`embedded-poc` didn't compile against the real `+esp` Xtensa
  toolchain** (#215) — #194's `DecodeResult.message77` field→method
  change wasn't propagated there, since `embedded-poc` is
  workspace-excluded and never built by host CI. 5 call sites still
  used `&r.message77` as a field: `embedded-shared/src/apps/{compute_bench,rx_wavsim}.rs`
  and each app's `decode_pipeline.rs`
  (`m5stack-s3-app`/`m5stack-core2-app`/`m5stack-cores3-app`). Found by
  installing `espup` fresh and running `cargo check` across all four
  `embedded-poc` crates — closes the "embedded-poc +esp check still
  owed" item from #206. All four now compile clean.

Older releases are archived out of this file to keep it skimmable:
0.6.0 – 0.7.4 in
[`docs/historical/CHANGELOG-0.6-0.7.md`](docs/historical/CHANGELOG-0.6-0.7.md),
0.1.0 – 0.5.12 in
[`docs/historical/CHANGELOG-0.x.md`](docs/historical/CHANGELOG-0.x.md).
