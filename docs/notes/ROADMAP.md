# Roadmap (post-0.9.0)

## Strategic state (2026-08-10)

Three tracks, at very different maturities:

1. **Host DSP / protocol — mature, effectively maintenance.** All eight
   protocol families sit at or near WSJT-X sensitivity parity. The one
   disclosed exception, JT65 vs. `ftrsdap`, was closed 2026-08-08 in
   three passes: a faithful port of WSJT-X's `ftrsdap` (#169,
   `jt65::chase::decode_at_with_chase`, literal magic numbers not just
   algorithmic shape); a same-day reliability-metric fix caught on user
   review; and — on further user pushback over the still-sizeable
   residual gap — a root-cause finding that the true bottleneck was FFT
   bin-quantization ("scalloping loss") in `rx`/`search`, unrelated to
   `ftrsdap` itself and affecting *every* JT65 decode path, not just
   chase. Fixing that (sub-bin frequency refinement + NCO correction)
   closed the diagnosed ~7-8 dB gap essentially entirely on this
   crate's AWGN corpus — see `docs/notes/BENCHMARKS.md`'s JT65 section
   for the full measured story and appropriate caveats on the WSJT-X
   comparison (not independently re-verified against a real `jt9`
   binary this session). GitHub issue #169 itself sat OPEN for two
   extra days after the fix landed (the closing commit's message
   didn't match GitHub's auto-close phrasing); closed manually
   2026-08-10 during a doc-staleness audit — a reminder to phrase fix
   commits as `Closes #NNN` literally, not just prose-mention the
   number. Golden lockdowns and the FST4 / FT4 / Q65
   sensitivity closes all landed across the 0.6.x–0.7.x line.
   A same-week hotspot survey (#246, real production-path stage
   timing across FT8/FT4/FST4/MSK144) found MSK144's OSD-tier
   fallback firing on ~68% doomed attempts and FST4's OSD escalation
   similarly wasteful (#245) — both faithful ports of WSJT-X's own
   shape, not bugs. A follow-up effect-vs-usage review (2026-08-10)
   closed both **without fixing them**: FST4's slot period (60s) and
   MSK144's T/R period (15-30s) both dwarf the ~370ms/~750ms decode
   cost the waste is a fraction of, and neither protocol runs on
   embedded or WASM — the two paths with real latency pressure. The
   same review tried two more speed avenues on FT8/WASM specifically
   (the protocol/platform pair that *does* have latency pressure) and
   found both negative: `rustfft`'s `wasm_simd` feature (already
   shipped, real +15-30% win, unrelated finding) and swapping the BP
   kernel from `SumProduct` to `NormalizedMinSum` to skip per-iteration
   `tanh`/`atanh` — re-verified with clean single-threaded native
   timing *and* a real `wasm32` build (`bench/wasm`), both showing no
   measurable difference and unchanged recall. BP's own compute is too
   small a fraction of total decode wall-clock for the kernel choice to
   matter on either platform. A related FST4 SIC feasibility experiment
   (#252) found no technical blocker, still open pending real demand.
   What's still open here (#143 / #193 FST4 AP+SIC, #192 FT8 engine
   unification, #224 JT4, #148 research, #252 SIC feasibility) is
   **tail work** — calibration, behaviour-preserving refactor, or
   low-demand modes — not a frontier. Advances here should be
   demand-driven (e.g. VK3NV's FST4-15/30 use case behind #143), not
   pursued for their own sake.

   A 2026-08-14 code-sharing audit (prompted by a user doubt over the
   README's unsourced "~80% shared" claim) confirmed the doubt but not
   the diagnosis: the true-generic fraction measures 31.8% (strict,
   protocol-bound code excluded) / 47.7% (directory), not 80% — fixed
   with a measured figure + methodology pointer to `LIBRARY.md` §0.5
   (PR #290), plus a permanent `sharing_ratchet_selftest` regression
   guard (PR #291). The root cause wasn't faithfulness-driven
   duplication in general (WSJT-X itself repeats per-protocol code,
   and QRA/Fano/RS/LDPC are genuinely different algorithms, not one
   thing written four times) but a narrower, real pattern: shared
   mechanisms built and adopted by 2-3 protocols, then left there with
   nothing flagging the rest. Concrete, verified duplication found and
   consolidated: `refine_freq_hz`/`dedup_known` (PR #292), 9 candidate-
   dedup call sites across JT9/JT65/WSPR/Q65 (PR #293), a 3x-duplicated
   `Spectrogram` build/score kernel across JT9/JT65/Q65 found via a
   bottom-up DSP-level sweep after top-down hit a genuine wall (PR
   #294), and a 5x-duplicated bit-reversal interleave permutation
   across WSPR/JT9 (PR #295) — plus, as adjacent cleanup, the dead
   `jt9::demod_bb` box-car path it should have been deleted alongside
   in 0.5.9 (PR #296). **Migrating JT9/JT65/WSPR/Q65 onto the generic
   `engine::pipeline` (the step that would have moved the needle most)
   is architecturally blocked**, not merely unstarted: the pipeline's
   `process_candidate_basic`/`decode_frame` require `P::Fec:
   BpPooledFec`, a soft-LLR belief-propagation scratch-reuse shape
   only `Ldpc174_91`/`Ldpc240_101` implement — Fano-sequential
   (JT9/WSPR), Reed-Solomon (JT65), and GF(64) QRA (Q65) can't satisfy
   it, an algorithmic mismatch rather than missing plumbing. Concluded
   by explicit user choice rather than a numeric target: no percentage
   was set as a stopping point, and the session's own retrospective
   note is that chasing a number here would have meant metric-gaming
   past this point, not more real consolidation.

2. **Host application / ergonomics — newly opened, consumer-driven.**
   0.9.0's theme ("make the streaming decode surface easy to build host
   UIs on") added `msg::decoded::Decoded`, per-protocol `to_decoded`, the
   `serde` feature, and completed streaming `.on_result` /
   `decode_scan_streaming` across all protocols, plus
   `mfsk-ffi`'s `mfsk_decode_i16_streaming` and (2026-08-09) six
   `mfsk_decode_options_set_*` builder-parity setters closing the
   `mfsk-ffi` vs. `DecodeRequest` gap for FT8/FT4/FST4-60A's scalar/
   strategy/AP knobs (issue #162 follow-up). Deliberately left for
   later, still open: `.known()` cross-phase dedup (#247),
   `SniperRequest` exposure (#249) — neither urgent, no consumer
   asking yet. Of the pass's other two deferred items: `.fft_cache()`
   reuse was closed outright 2026-08-10 (#248, no consumer
   materialized), and Q65's `.hash_table()` shipped 2026-08-10 (#250 —
   despite `mfsk-ffi`'s own near-zero download signal at review time,
   picked as the one cheap, self-contained, mechanically-scoped item
   worth doing regardless of demand; see *Open follow-ups* below and
   `mfsk-ffi/README.md` for the shipped shape).
   `session::SlotAssembler` (audio ingestion) is written
   and **parked pending a real consumer**. This track only advances
   when an actual host UI (WebFT8, a desktop app) needs the next
   piece — building further ahead of a consumer risks fixing an API
   shape nothing has validated.

3. **Embedded controller (Phase B-Core) — the real frontier, unblocked
   2026-08-23.** The main production target (M5Stack CoreS3 UAC FT8
   controller) had its crate skeleton and UAC host code shipped and
   compiling clean for months, with **#163 — live IC-705 hardware
   verification — never done**, because it is a human-at-the-bench task.
   It is done now: ten unbroken minutes of UAC capture, 125 MB, zero
   errors, WiFi associated throughout. The downstream sequence (shared
   UAC hoist → BLE CI-V → ADIF → touch UI → TX keying) is no longer
   sequenced behind a verification step, and the memory budget that
   made people doubt WiFi could coexist with the USB host turned out to
   be a symptom of two stack overflows rather than a real ceiling.

4. **WSPR embedded RX (Phase E, #260) — a separate frontier, and
   unstalled.** Not part of Phase B-Core: WSPR never goes through
   `decode_block`/`fixed-point`/the UAC controller stack, so this is a
   distinct decoder-level effort rather than the FT8-controller product
   line above. Opened 2026-08-11; a device that had no path from
   "audio arrives" to "WSPR decodes" at all now runs the whole thing —
   capture, down-conversion, decode — inside the 120 s slot with margin
   (steady state: 82.8–90.1 s against a 110 s deadline, 9/9 golden
   held). See
   **Phase E** below. Shared #163 as an unverified dependency for live
   audio (still WAV-fed/synthetic baseband here — the UAC path is proven
   on the FT8 controller but `wspr_app` has not been run against a radio
   yet) but is otherwise independent — most of what closed here (dual-core safety, task-stack
   placement, a streaming down-converter) generalizes to any future
   embedded decode-heavy protocol, not just WSPR.

The open strategic question this doc deliberately does **not** decide
(it's the maintainer's call, not to be inferred from momentum): whether
the next cycle's centre of gravity is **finishing the embedded product**
(build on the now-unblocked Phase B-Core) or **serving the library's
host-UI consumers** (extend track 2). The two aren't exclusive, but
attention is.

## Current line — 0.9.0 shipped

- **0.8.0** — legacy BASIS `fill_symbol_spectra_into` path removed
  (#162; a breaking FFI change to `mfsk_ft8_decode_i16`'s signature),
  bundled with the accumulated 0.7.5 content rather than cutting 0.7.5
  separately: JT65 interleave TX/RX-convention fix (#24), the JT9 AWGN
  sweep, and the new Q65 sub-modes.
- **0.8.1** — decode-side `snr_db` added to `WsprResult`, `Jt65Result`,
  `Jt9Result`, `Q65Result` (#226; breaking — a new required field on
  four public types). Closes the gap where only the FT8-family shared
  `DecodeResult` and MSK144 carried an SNR estimate; `mfsk-ffi`'s
  `push_simple` had been hardcoding `0.0` for the other four.
- **0.9.0 — streaming ergonomics for host UIs.**
  `msg::decoded::Decoded` (a unified, owned, `Send` decode row for host
  UIs) + a `to_decoded(..)` conversion on every protocol's native result
  type; the `serde` feature (off by default, `no_std`-clean) deriving
  Serialize/Deserialize on `Decoded` + `ProtocolId`; streaming
  `.on_result` / `decode_scan_streaming` now complete across all
  protocols, documented in `docs/reference/STREAMING.md` (+ `.ja`);
  `mfsk-ffi` builder parity for `MfskDecodeOptions` (#162 follow-up)
  plus Q65 callsign hash-table exposure (#250). Minor bump per this
  crate's "new cross-cutting public API surface = minor" convention
  (cf. 0.7.0's generic-API landing). Tagged 2026-08-10, 8 days after
  v0.8.1 — within the biweekly cadence window, not opportunistic.
  - **Parked, not in 0.9.0**: `session::SlotAssembler` (streaming audio
    ingestion — resample-to-12k + slot windowing + sample-counted slot
    timing). Code-complete and tested on branch
    `claude/streaming-interface-docs-vuet32`, held unmerged until a real
    consumer (a desktop UI, or a refactor of the embedded `audio.rs`
    slot statics onto it) validates the `push`/`poll`/`mark_slot_start`
    API shape. See track 2 above.

## Prior line (0.7.x) — shipped through 0.7.4 (2026-07-19)

Numeric recall/sensitivity figures for every protocol are **not**
duplicated here — `README.md`'s "What's solid" section is the
canonical, kept-current source (duplicating numbers in two docs is
exactly how this file's FT8 figures drifted out of sync with reality
for two months; see `CHANGELOG.md` for the full per-release detail).
This section is just a byline of what shipped since the last time
this doc was substantially updated (0.6.5, 2026-05-18):

- **0.7.0** — FST4 gained its remaining four sub-modes
  (FST4-15/30/120/300, via the `fst4_submode!` macro) and
  `coarse_sync` was parallelised under `--features parallel`.
- **0.7.1 / 0.7.2** — FST4 AWGN sensitivity gap vs. WSJT-X's
  published thresholds closed from ~2.4-3.1 dB down to a common
  ≈0.1-0.6 dB across all five sub-modes (issue #146 — coherent
  full-slot sync, an FST4-specific `LLR_NSYM_MAX` override, a
  zsum-OSD fallback). See `docs/notes/FST4_BENCHMARK.md` for how to
  reproduce the sweep.
- **0.7.3** — FT4 AWGN sensitivity gap narrowed ~1.8 dB → ~0.3 dB
  (coherent Costas-block scorer + an OSD-gate fix); FT8 CCIR
  moderate/poor fading recall gap closed by widening
  `OSD_HARDERRORS_MAX` back to WSJT-X's universal 36, which also
  resolved issue #150's JTDX-18 ground-truth question as a side
  effect.
- **0.7.4** — MSK144 meteor-scatter decode shipped (issue #25): full
  WSJT-X port (LDPC(128,90)+CRC-13, MSK/OQPSK DSP, burst-scan sync),
  3/3 golden-WAV recall. A systematic -1 dB SNR bias found
  post-ship was root-caused (missing fixed bandpass filter in the
  analytic-signal front end) and fixed, then cross-validated
  against a real WSJT-X `jt9` build (25/28 cells exact match — see
  `docs/notes/MSK144_BENCHMARK.md`). Also established this repo's
  release cadence policy: PRs land on `main` immediately, but
  crates.io tags are cut **biweekly** rather than per-change (escape
  hatch for security/correctness-critical fixes or explicit
  request) — see `CLAUDE.md`'s "Release cadence" section.

Embedded-line status (M5StickS3 / CoreS3 / Core2) has its own
detailed writeup under **Phase B** below — the short version: Phase
B-Core (CoreS3, the main production target) has its crate skeleton
and UAC host support shipped, and as of 2026-08-23 verified against
live hardware (issue #163, closed); Phase B-Stick
(M5StickS3, demo/fallback) and the Core2 sibling are both frozen,
unchanged since late May.

### What landed in 0.6.5 (2026-05-18)

Non-functional crates.io / docs.rs surface refresh — no source code
modifications, no new features, no bug fixes vs 0.6.4. The embedded
port is no longer aspirational, and the discoverable surface had to
catch up:

- **`mfsk-core/Cargo.toml` `description`** rewritten to lead with
  the working `embedded-poc/m5stack-s3-app` M5StickS3 FT8 controller
  (LCD UI, BLE CI-V to IC-705, acoustic mic, QSO FSM, ~1.2 s
  post-SlotEnd decode on Xtensa LX7).
- **`mfsk-ffi-ft8/Cargo.toml` `description`** notes M5StickS3
  end-to-end verification.
- **`mfsk-core/src/lib.rs` "Why this exists"** gains a fourth
  bullet for the handheld-controller use case, with cross-links to
  the M5StickS3 source crate and `docs/reference/MANUAL_M5STICKS3.md`.
- **`README.md`** opens with a hero photo of the device decoding
  five real on-air FT8 callsigns from a single 15 s slot
  (`docs/assets/m5sticks3-ft8-decode.jpg`).

### What landed in 0.6.4 (2026-05-18)

- **Phase 1.7.7-Stick Goertzel migration** (PR #104, commit `5501a2f`).
  The per-symbol DFT in `fill_symbol_spectra` switched from the
  precomputed Q15 `BASIS` table + esp-dsp ASM `dsps_dotprod_s16_ae32`
  dot product to a generalised Goertzel recursion
  (`fill_symbol_spectra_goertzel`) — 2-tap IIR per (sym, tone) with
  3 f32 state values on the stack, zero scratch, zero extern symbols.
  Sample-outer / tone-inner loop interleave lets LLVM unroll the
  constant `NTONES=8` inner loop so 8 independent dependent chains
  run through the FPU pipeline in parallel; result is stage-3
  wall-clock equivalent to BASIS (~1.4 s on S3 `qso3_busy.wav`) with
  **+0.16..+0.63 dB SNR improvement** (f32 Goertzel beats Q15 BASIS
  precision).
- **`fixed-point` ⇒ `nstep-half` coupling** (commit `dfc0cd5`).
  The two features were always co-enabled on every embedded target
  in practice; decoupling silently gave host fixed-point builds a
  different NSTEP grid than the embedded path, making host validation
  diverge from real-silicon behaviour. Now `fixed-point` implies
  `nstep-half`.
- **mag² saturation fix** in `compute_spectrogram` (same commit).
- **Internal-DRAM saving**: ~60 KB BASIS scratch × 2 (re/im) × 2
  cores = **120 KB freed** on M5StickS3, which is exactly what the
  Qso-mode bidirectional I2S DMA descriptor needs. Qso mode now
  boots cleanly on first try.

The legacy BASIS code path (`fill_symbol_spectra_into` and the
`mfsk_core_dot_q15_i32` extern symbol) and the `basis_re`/`basis_im`
scratch arguments on `mfsk_ft8_decode_i16` remain for back-compat
with 0.5.x callers. **Still present as of 0.7.4** — this was
originally slated for 0.7.0 removal, which never happened (0.7.0's
actual scope was FST4 sub-modes + `coarse_sync` parallelisation,
unrelated). Now tracked as [#162](https://github.com/jl1nie/mfsk-core/issues/162)
rather than re-promising a version number here.

### What landed in 0.6.3 (2026-05-17)

- **WSJT-X-faithful OSD precoding** (#63, PRs #86 + #87 + #88).
  `npre1` (ndeep=2) + `npre2` (ndeep=3) dispatch + 4 missing
  post-OSD validity gates from `ft8b.f90:422-459` + nsync/xsnr
  clamp-ordering fix, plus an empirically-tuned
  `OSD_HARDERRORS_MAX = 22` ceiling on the OSD path
  (mfsk-core-specific deviation from WSJT-X's universal 36) that
  eliminates the qso3_busy phantoms `npre1`/`npre2` alone did not
  filter. See `docs/historical/CHANGELOG-0.6-0.7.md`'s 0.6.3 entry
  for the WSJT-X-faithfulness / phantom-elimination trade-off
  rationale.
- **ε `decode_block.rs` restructure** — 6 stacked PRs (#77 types,
  #83 spectrogram, #79 coarse_sync, #80 fill_symbol_spectra, #81
  process_candidates, #82 osd_strategy). 3,517 lines → 423-line
  parent + 6 stage submodules. The OSD-strategy seam ε.6 carved
  out is load-bearing for #63's WSJT-X-faithful `npre1`/`npre2`
  dispatch.
- **#61 Core2 fold-in** (PR #76) — `mfsk-app-shared` carve-out +
  `m5stack-core2-app` sibling + retirement of the standalone Core2
  bench. Both production apps now share board-agnostic QSO FSM /
  UI / WiFi / NVS / log fanout.

### What landed across 0.6.0 → 0.6.2

- **0.6.0**: FT8 sync consolidation (`decode_block::coarse_sync` is
  canonical, `ft8::sync::coarse_sync` removed); `#40` host
  wide-band coarse-sync gap closed via `i_start as i32` + WSJT-X
  all-or-nothing boundary; `#46` / `#48 step B` / `#49 cat A-C`.
- **0.6.1**: per-candidate inner unification —
  `process_one_candidate_inner` is shared between the host
  `process_candidate` body and `decode_block::run`. New public
  `decode_block_with_ap[_tuned]` entries; AP iaptype loop reachable
  from embedded as Step 4 of the inner. Embedded OSD pass IDs
  migrated 4-7 → 14-17 to free 5..12 for AP.
- **0.6.2**: host cs-source unified onto `fill_symbol_spectra`
  (drops the cd0 + `ft8::llr::symbol_spectra` divergence); host SIC
  switched to `subtract_signal_lpf`; dead-code removal
  (`subtract_signal_weighted`, `subtract_signal`,
  `qsb_partial_gain`, `ft8::llr::symbol_spectra`); embedded
  `LlrT` Q3i8 → **Q11i16** (i8 → i16, ~16× LLR resolution; ship
  recall +1 entry, post-SlotEnd 1.341 s → 1.191 s); fixed-point
  build repair (broken since 0.6.1 by an internal call-site rename).

### Embedded fine_refine attempt postmortem (deferred)

Two paths were tried during 0.6.2 to lift embedded recall toward
the JTDX-extra band:

1. **Per-symbol DFT iteration** via `fill_symbol_spectra` per
   candidate — projected ~100 ms, actually >5 s blocked compute
   (200k+ × 1920-sample DFTs per slot), tripping the FreeRTOS Task
   Watchdog.
2. **`cd0` complex-baseband via cascaded esp-dsp FIR decimate**
   (3:1 → 4:1 → 5:1 = 60:1) — solved the compute side but hit a
   PSRAM bandwidth ceiling (~80 MB/s on S3 OCT mode means ~1 s
   just for memory traffic for 15 candidates; further alignment
   requirements, scratch fragmentation, lazy-alloc TLSF corruption
   were all incidental).

Honest read: embedded recall stays at **7/18** until either an
i16-throughout FIR refine path or a streaming chunk-processing
refactor lands. Both are out of scope for a 0.6.x patch — defer
to a 0.7.x design pass.

### Open follow-ups

Currently open GitHub issues (state:open as of 2026-08-19, verified
directly against the GitHub API — this is the live worklist; if you're
reading this file to decide what to work on next, trust this section
over any recall numbers or hardware status stated elsewhere in it).
Grouped by the three tracks in **Strategic state** above.

Re-read end to end 2026-08-19, comment threads included, which is what
this refresh is worth flagging: three entries below had drifted from
what their own threads concluded (#193's premise was invalidated by a
grep of WSJT-X, #310's title question was answered "no", #247 has a
real ABI decision in it that this section had recorded as "no open
design question"). Issue *titles* age badly here — several threads
resolve their nominal subject in the first few comments and then
spend thirty more on something else worth more. **Next actions —
the measurement queue** at the end of this section is the short
version if you're picking up work.

**Embedded (frontier):**

- **#163** — CoreS3 Phase 1-Verify: live UAC hardware RX confirmation.
  **Closed 2026-08-23.** An IC-705 enumerates through its internal hub,
  the audio interface opens, and the reader sustained 192,512 B/s for
  ten minutes — 125 MB, 30,520 packets, zero errors — with WiFi
  associated and the FT8 decode pipeline running slots off the live
  stream. Log kept at
  `embedded-poc/m5stack-cores3-app/logs/uac_stream_2026-08-23.log`.

  It took three findings on the enumeration path (VBUS needs three
  AW9523B bits, not two; a 2048 B control-transfer buffer; hub support)
  and then two stack overflows that presented as heap corruption and
  cost more time than the USB work did. `embedded-poc/CLAUDE.md` has
  both stories under "USB host VBUS on CoreS3" and "Stacks, heaps, and
  the space between them" — read them before the next bench session.

  Still open on this path: the transient-recovery code (a
  `USB_TRANSFER_STATUS_OVERFLOW` was seen once and stopped the stream
  dead; a stall watchdog plus re-open now handles it, but has not
  fired in the field since), and `wspr_app`/`fst4_app` have not been
  run against a radio at all.

- **#313** — CoreS3 WSPR standalone app, open items left after #260
  closed: no wall-clock slot alignment on the real-audio path (needs
  the NTP-fed `time_sync` hook; `uac.rs` still cites the stale `#32`/
  `#34` numbers for it), `SpotSink::Http` never run against a real
  wsprnet endpoint, and the two-stage DDC decimation that was deferred
  rather than rejected.

  Of those four, the third needed #163, which has now cleared for the
  shared `uac.rs` — though `wspr_app` itself still has to be run
  against a radio. Slot alignment is software-only and is a
  correctness bug, not a nicety: the slot boundary is bound to raw
  sample count from UAC stream start rather than UTC :00/:02, so DT
  reads against the wrong slot and every spot inherits that. It is
  worth fixing *before* #163 clears rather than after, because it is
  otherwise indistinguishable from a hardware problem the first time
  real audio flows. The wsprnet sink is blocked on a live third-party
  endpoint rather than on hardware, and carries a cost the others
  don't — a malformed spot is publicly visible on wsprnet.

- **#306** — FST4 on ESP32-S3: embedded feasibility. The umbrella, and
  the most active thread in the repo (VK3NV). Status as of 2026-08-17:
  the candidate loop measures **40.102 s** (`full`) / **13.643 s**
  (`no8_osd`) on real CoreS3 hardware over 41 candidates, against
  FST4-60's ~7 s margin — **not fitting yet, but quantified**, and down
  from ~13× over at the first measurement. Decoder-only figures: the
  bench routes around all three FFT sites and is fed host-baked
  baseband, so an end-to-end receiver still needs #307 or #309. The two
  live levers are breadth (#312) and depth (#310).

- **#310** — fold `rung_major`'s `offsets` into cost-ordered scheduling
  instead of an offset-major outer loop. Its real subject is the
  worst-case **time-to-first-decode** bound (~7.4 s for the first rung,
  ordering-independent), not the 1.26× total-time improvement. Embedded
  currently gets zero timing diversity — every caller passes `&[0]`.

  **The title's question has been answered "no", and the issue is now
  open for a different reason.** Folding offsets in would triple the
  first-rung sweep — the exact bound the module exists to provide —
  and the payoff can't be aimed: exhaustive `{0,+1,-1}` costs 1.6-2.7×
  decode units for +1 to +3 decodes per 20 trials, and ranking offsets
  cheaply by sync quality matched exhaustive in 1 of 8 runs and the
  single-offset baseline in the other seven. Worth reading for the
  retraction, which sharpened the result rather than reversing it: an
  initial "the ranking test is degenerate" reading was withdrawn after
  an explicit counter showed `argmax nsync == offset 0` in only 43-59%
  of candidates (vs 33% blind) — sync evidence *does* distinguish
  between offsets, its distinctions are just uninformative about
  decodability, agreeing with the offset that actually decoded 7/26 ≈
  27%, i.e. at chance.

  VK3NV's soft-Costas-margin proposal was implemented and measured
  rather than argued about, and **not adopted** — stratified AUC 0.717
  against `nsync`'s 0.920 unconditional, worth +0.014 AUC over the
  whole 9-feature set, for 320 B/candidate plus fitted weights. Two
  wrong readings were corrected on the way (a first +0.019…+0.099 AUC
  measured the wrong population, ~5× inflated; a follow-up "no
  information" verdict was also wrong). The budget curve is the
  durable part: a priority signal is worth 3.4× at a 10% budget, but
  FST4-60's real situation (~7 s against 13.6 s) is a ~50% budget where
  `nsync` alone already returns 380/386 = 98.4%.

  What landed instead is **`Schedule::PhaseSplit` (#317)**: phase A
  runs `llra` at `offsets[0]` breadth-first across every candidate and
  is never budget-gated (that *is* the latency invariant), phase B runs
  the rest of that offset's ladder depth-first ordered by the `nsync`
  phase A already computed, phase C handles remaining offsets under the
  budget gate. Simulated at matched budget it beats plain breadth-first
  166 vs 35 decodes at 10%, 288 vs 80 at 20%, converging by 100% —
  because below ~30-40% breadth-first has spent everything on cheap BP
  stages and OSD has not run for *anybody*, and OSD is where most
  decodes come from. `internal-testing`-gated, `decode_rung_major`
  unchanged, equivalence test asserts the two match. Also recorded
  in-issue as a **process error**: it was implemented before VK3NV had
  responded to the reasoning.

- **#312** — FST4 sniper: `max_cand = 50` binds before the search
  window width does, so narrowing the window changes nothing on the
  production path today. Sweep the cap as a *retained fraction* across
  ±25/50/100/250 Hz; leave `sync_min` alone first.

  Partly answered. At matched retention, narrowing the window does cut
  false survivors ~20× (`FST4_BENCHMARK.md` §10), and a preliminary
  near-threshold run suggests the recall cliff tracks the **absolute
  cap** rather than the retained fraction — which, if it survives the
  full grid (§11.2, queued), means "absolute floor for recall, fraction
  for cost" rather than either alone. A separate defect VK3NV found
  while auditing this — `coarse_sync` re-admitting candidates its own
  dedup had rejected — is fixed and shipped in 0.10.0 (#316), with a
  non-`#[ignore]`d regression test that fails against pre-fix code.
  Two details worth carrying forward from that fix: it was verified
  recall-neutral through the wideband production path (82/120
  identical in every cell) so it should be invisible to the tier-C
  sweeps, and both parties first reached for
  `score >= sync_min && stage1_pass(fi)`, which would also have
  dropped candidates that clear the score gate but fail stage 1 —
  exactly what #146's OR-gate exists to keep. Tracking suppression in
  its own vector was the fix; tightening the gate would have been a
  silent sensitivity regression.

  The incidental finding here is the one that matters to #310: on
  N5TM's ±100/±50 Hz rows every cap including 4 sits at ~2100 ms while
  ±250/±25 are 40-90 ms, i.e. **a single candidate costs ~2 s on its
  own** and ranks top-4 in the middle widths. Candidate-count
  reduction cannot bound per-candidate cost — which is why breadth
  (#312) and depth (#310) are complementary levers rather than
  substitutes.

- **#307** — `engine::fft` has no generic non-power-of-2 FFT.
  `coarse_sync`'s `nfft1` is non-power-of-two for all five FST4
  sub-modes and needs this regardless; `downsample_cached`'s
  `fft1_size` can be solved by this *or* by #309. VK3NV's 7776 → two
  exact 1944-point transforms derivation keeps FST4-15/30/60 under the
  configured 8192 ESP-DSP ceiling without a Kconfig change.

- **#309** — FST4-120/300 will need a streaming DDC: FST4-120's
  whole-slot FFT buffer (~11 MiB) is essentially the scale that blocked
  WSPR, and FST4-300 (~32 MiB) is worse. `wspr::ddc` is the template.
  Not on any current roadmap phase — filed so the comparison isn't
  rediscovered.

- **#311** — 3 of 5 CCIR-moderate trials that the unpruned OSD search
  decoded and the npre1/npre2 port does not remain unexplained after
  #308's timing fix recovered the other 2. A real `npre2`-pruning
  question, split out of #308 so it didn't close with it.

  An n=20 ablation answered the substitutable-vs-additive question as
  **neither: in 2 of 4 cells the two mechanisms interact**, rescuing
  trials neither rescues alone — so "add only the cheaper of the two"
  is the worst option, and which mechanism dominates flips between
  cells, meaning no universal escalation order falls out of this.
  Separately and more bluntly: **`npre` never decoded a trial the
  unpruned search missed**, in all four cells at both timing settings,
  so on that corpus the pruning is pure recall loss bought for speed
  and "selective fallback to unpruned" is behaviourally identical to
  always running unpruned.

  Blocked on an n=100 re-run, which needs `fst4sim` — not installed on
  the machine that produced the n=20 numbers. Runbook:
  `FST4_BENCHMARK.md` §11.3. Two traps recorded there and worth
  re-reading before regenerating anything: trial indices do not
  survive corpus regeneration (`gen_fst4_sweep_wavs.sh` regenerates
  rather than extends, so #306's `[2,15,33,45,67]` refer to a
  different corpus), and selecting trials through the production path
  is now circular, because `DecodeRequest`'s `osd` defaults true and
  #308's retry is gated on it.

**Host DSP / protocol (maturity — tail, not frontier):**

- **#143** — FST4 AP decode + SIC for FST4-15/30. Real user demand
  (VK3NV's real-time weak-signal messaging); the building blocks all
  exist (`msg/ap.rs`, `msg/pipeline_ap.rs`, `core/dsp/subtract.rs`),
  missing only the FST4 wiring + per-sub-mode `SubtractCfg` calibration.
  Low priority until crowded-band use materialises.

  **Read this issue for what it became, not its title** — 28 comments
  in, no AP/SIC code has been written, and its durable output is the
  WhisperQSO design thread plus three things now load-bearing
  elsewhere: the **guard-time correction** that produced #306's ~7 s
  budget (FST4-15 ≈4.9 s, -30 ≈6.6, -60 ≈7.2, -120 ≈9.7, -300 ≈12.3 —
  the full T/R period is *not* the decode budget); the first
  FST4-on-ESP32-S3 feasibility answer, "hard no across all five
  sub-modes", with the PSRAM capacity/bandwidth tables #309 later
  re-derives independently; and the `compute_spectra` frequency crop
  (11.4 MB → 5.53 MB at the crate default, → 0.536 MB at a 276 Hz
  segment, ~21×; wall-clock only ~10% because the crop touches the
  0.4% `extract` phase, not the 90.3% `fft` one). Nominally waiting on
  VK3NV's real Android/ARM timing numbers — every ARM figure in the
  thread today is extrapolated from a single Geekbench ratio against a
  76 ms FST4-60A host measurement.
- **#193** — FST4 has no SIC path (no `SubtractCfg`, no
  `decode_frame_subtract`). **The issue's own premise was invalidated
  in-thread and this entry was wrong until 2026-08-19**: there is no
  "WSJT-X FST4 subtract path" to calibrate against.
  `grep -n "subtract\|npass\|SIC" lib/fst4_decode.f90` against a
  checked-out WSJT-X tree returns zero genuine matches — WSJT-X never
  implemented SIC for FST4 either (control: the same grep on
  `lib/wsprd/wsprd.c` does find real `npasses`/`subtract` hits). So
  this is not a port lagging upstream; it would be new capability, and
  it is re-scoped from backlog item to future-feature idea. If ever
  pursued, FST4-15/30 are the plausible candidates (SNR regime near
  FT8's); the long sub-modes have a real tension between the long
  averaging window deep SNR needs and the short window real fading
  allows. `SupportsSicRounds`'s doc comment carries the correction.
- **#252** — FST4 SIC feasibility experiment (coherent full-slot sync
  from #146 + subtraction residue, 4 scenarios: moderate/tone-spacing
  +QSB/co-channel/3-station multi-subtract). No technical blocker
  found; one 3-station run hit a single hard_err=75 CRC-false-accept
  suspicion, not confirmed coherent-sync-specific. Feeds into #143/
  #193 (FST4 AP+SIC) if that work is picked up — whether it's worth
  picking up depends on real busy-FST4-band demand, still unverified.
- **#224** — JT4 not implemented (WSJT-X ships JT4A/JT4F golden WAVs).
  "Doable but demand unclear" — **and the demand question has since
  been answered, with dated sources: weak case.** The "432 and Above"
  EME newsletter shows zero JT4/JT4F/JT4G mentions in both its 2025-02
  and 2026-03 issues, with microwave-band QSO reports dominated by Q65
  and JT4 absent entirely; the one surviving use is beacons (GB3SCX
  10368.905 MHz, GB3SCK 24048.905 MHz, UK Microwave Group wiki last
  modified 2025-12-11), which reflects an operator not reconfiguring
  rather than decode demand. Deliberately left open as a decision
  record, not as work: **track, don't commit.** Closing it would lose
  the sourcing and invite the question being re-asked from scratch.
- **#148** — Research idea (not a commitment, from VK3NV): blind-paired
  FST4-120 with soft combining, as a Doppler-robust FST4-300 alternative.
  Q65's multi-period averaging (`q65/rx.rs`) is the architectural
  precedent, and the combining point is settled: **LLR-level**, because
  a bit LLR is phase-free by construction and FST4's 1/2/4/8-symbol
  coherent ladder is only ever coherent *within* a slot, so the gain is
  fully captured once each slot has its own LLR vector. That makes it
  standard Chase/HARQ combining — sum two 240-element vectors, retry
  BP/OSD, no change to `compute_llr_generic` or the nsym ladder.

  Blocked on test infrastructure, and the blocker is the honest kind:
  a valid measurement needs the *same continuous* fading process
  spanning both 120 s slots, and neither `fst4sim`'s Watterson
  generator (`watterson()` regenerates an independent process per
  call) nor `tests/common/air_channel.rs` provides it — two
  independent fading files would overstate the gain by removing
  exactly the correlation the poor-channel case depends on. Also
  worth carrying: an initial "AWGN should approach the ideal 3 dB"
  claim was walked back with numbers — at −30/−31 dB failures are
  almost entirely decode-limited (sync finds the candidate 39/40, the
  regime combining helps) but by −32 dB sync failure is ~28% of
  misses, so per-slot coarse sync becomes the second bottleneck before
  the ideal threshold is reached. "Generate a correlated two-period
  fading test channel" is the next piece of work and has no issue
  number yet.

**Host application / ergonomics (emerging, consumer-driven):**

- **#247** — expose `DecodeRequest::known()` for cross-phase dedup via
  `mfsk-ffi`. **This entry used to say "no open design question"; that
  was wrong.** `.known()` takes `&[P::DecodeResult]` — raw
  `message77`/`info` bits, `freq_hz`, `dt_sec` — and `MfskResult` is
  flattened display-only (`text: [c_char; 40]`, plus scalars), so a C
  caller holding an `MfskResultList` structurally cannot feed it back.
  Two options, neither chosen: an opaque "decode result usable as
  input" handle (new type + ownership story) or adding the raw
  `message77`/`info` bytes to `MfskResult` (additive and
  non-breaking, but leaks internal representation into the public
  ABI). Scope is FT8/FT4/FST4 only. The issue asks explicitly for its
  own pass rather than being bundled into the next unrelated FFI
  change.
- **#249** — expose `SniperRequest` (single-frequency-target decode)
  via `mfsk-ffi`. A wholly new function family, not a setter. The
  concrete cost of not having it: narrow-band `SniperRequest::ap_hint`
  is gated on `WsjtApCompatible` and so reaches **FT4 and FST4 too**,
  unlike `DecodeRequest`'s wide-band AP (`SupportsWideBandAp`, FT8
  only) — meaning **FT4/FST4 AP hinting is unreachable from the C ABI
  until this lands**. One decision embedded in it: `SniperRequest`'s
  knob set differs (`.sync_min()` as a setter; no `.freq_hint()`/
  `.known()`/`.fft_cache()`/`.sic_rounds()`/`.sic_early()`), so it
  needs either its own options type or a decision to reuse
  `MfskDecodeOptions` and ignore inapplicable fields — the latter
  matching crate convention.
- `session::SlotAssembler` is parked on branch
  `claude/streaming-interface-docs-vuet32` pending a real consumer
  (desktop UI, or the embedded `audio.rs` slot-statics replacement) to
  validate its shape before landing. No issue filed for it yet.

**Open as records, not as work:**

- **#284** — what FT8's OSD fallback actually costs, filed with "no
  action proposed" so future speed work starts from data rather than
  intuition: OSD buys 6 of 20 decodes for 47 ms (147.5 ms `FULL` vs
  103.6 ms `BP_ONLY`), and only 19 of 180 candidate-passes reach OSD at
  all. Its value is the two negative results. WSJT-X's cheap first pass
  (`ndeep=2`/`maxosd=0`) does not port — the nearest equivalent
  measured 148.1 ms (a no-op) and removing OSD from pass 0 entirely
  measured **175.0 ms, i.e. 27 ms worse**, because this crate subtracts
  each accepted decode immediately, so an early decode shrinks later
  candidate lists and pays for itself. Tightening the `q > 6` OSD gate
  is also out: it already mirrors `ft8b.f90`'s only bail, and the
  former mfsk-core-specific `q >= 12` is precisely what #180 traced
  `K1BZM DK8NE -10` (`q=11`) to. Two items remain unmeasured
  (`Q_NDEEP3_THRESHOLD = 18`, and the dispatch ladder order, which
  changes *which* decode is returned first); both are sensitivity-
  critical and need a measurement harness in front of them — the #282
  lesson, where source-diff reasoning predicted the wrong direction for
  2 of 5 protocols.
- **#252** — see the FST4 research entry above. Feasibility checked and
  closed as a question; open only to record that it was.

#### Next actions — the measurement queue

The single most useful thing to notice from the 2026-08-19 re-read:
**three separate work items are queued behind the same resource** — a
fast machine with the generated corpora and `fst4sim` present. They
are not competing priorities, they are one scheduling problem, and
two of them share a corpus.

1. **Tier-C sensitivity sweeps** — **done, 2026-08-19/20**, all seven
   protocols (see *Releases* in `CLAUDE.md` and `FST4_BENCHMARK.md`
   §11.1 for the full account). No regressions anywhere. FST4's five
   sub-modes all landed byte-identical to the pre-#316/#317
   measurement — both changes were built to be recall-neutral and
   this is the first end-to-end confirmation of that beyond their own
   targeted equivalence tests. The "8 h+ on an 8C/16T box" estimate
   below this entry (kept for the record) never matched reality: the
   real number, measured twice, is ~31 min on a 24-thread machine —
   the earlier partial run's extrapolation didn't account for the
   tier-C runner at the time also executing every accumulated
   `#[ignore]`d diagnostic in `fst4_sweep.rs` (47 of them alongside
   the one real gate), not just the sensitivity sweep itself; the
   runner now filters to the one real test
   (`scripts/run-sensitivity-sweeps.sh`'s `SUITES` comment has the
   detail). This also seeded `docs/notes/sweep-baseline.json`, a
   machine-readable baseline `scripts/sweep-regression-check.py` now
   diffs future sweeps against automatically instead of a human
   eyeballing printed tables.

   Original partial-run note, kept for provenance: FST4-15 passed all
   four channels 2026-08-19, AWGN 50% crossing −20.70 dB against
   `BENCHMARKS.md`'s −20.60 dB, inside 20-trial noise and matching
   WSJT-X's published −20.7 dB; the CCIR-good/moderate/poor crossings
   (−20.42 / −18.40 / −18.29 dB) were first-time records for FST4-15,
   since confirmed unchanged in the full re-run above.
2. **#311's n=100 ablation** — needs `fst4sim` specifically, and a
   regenerated corpus (see the two traps in that entry).
3. **#312's 96-configuration near-threshold cap grid** —
   `FST4_BENCHMARK.md` §11.2, ~2 h measured on an M5 laptop, narrowable
   via `MFSK_CAP_SWEEP_CELLS`/`_WIDTHS`/`_CAPS`.

Two of these bear on each other's reading and the order matters: if a
tier-C FST4 number moves, **#311 is the pre-existing explanation to
reach for first** — the npre1/npre2 OSD port shipped this cycle and is
measured to cost CCIR-moderate recall (m26 24→18 before #308's timing
fix brought it back to 20/100), and the residual 3/5 is exactly the
un-mechanized floor that would surface as an "unexplained" shift.
#316's dedup fix is the other candidate-gate change in this release,
but it was verified recall-neutral through the wideband production
path, so it should be invisible.

Independent of the queue: **#163 is closed (2026-08-23)** and is no
 longer the bottleneck for everything embedded — the UAC path is proven
 on the FT8 controller, leaving `wspr_app`/`fst4_app` to be run against
 a radio. **#313's slot-alignment fix** is the
highest-value software item in the embedded set, because it is a real
correctness bug that will otherwise be misread as a hardware fault the
first time real audio flows.

Recently closed since the 2026-07-19 snapshot (see the closed issue /
`git log` for fix commits): **#24** — JT65 interleave TX/RX-convention
bug, fixed, shipped in 0.8.0; **#162** — legacy BASIS
`fill_symbol_spectra_into` path removed, done, shipped in 0.8.0 (a
breaking `mfsk_ft8_decode_i16` FFI change); **#243** — FT8 host
multipass `xsnr2` SNR-gate frozen at pass 1, fixed to recompute every
pass, 2026-08-08; **#244** — redundant BP/OSD on near-duplicate
refined FST4 sync candidates (9→2 on the qso3 golden), 2026-08-08;
**#245** / **#246** / **#248** — closed 2026-08-10 without fixing,
per the effect-vs-usage review in *Strategic state* above (real
measured waste, but on protocols/paths without real latency or
consumer pressure); **#250** — Q65 callsign hash-table exposure via
`mfsk-ffi`, shipped 2026-08-10 (see *Strategic state* above and
`mfsk-ffi/README.md`); **#169** — JT65 `ftrsdap` sensitivity gap, fix
landed 2026-08-08 (see *Strategic state* above and
`docs/notes/BENCHMARKS.md`'s JT65 section), issue itself closed
2026-08-10 after a doc audit caught it
sitting open past the fix.

Closed since the 2026-08-10 snapshot (caught this section's own
staleness the same way #169 caught its own — #192 sat listed as open
above for several days after it actually closed): **#260** — WSPR RX
on ESP32-S3, closed 2026-08-14 once the whole receive path (capture,
down-conversion, decode) was measured running inside the slot with
margin on real device data, see **Phase E** below; **#192** — FT8/
`engine::pipeline` unification, closed 2026-08-14 narrowly (the
literal `fine_refine_3stage<P>` proposal — no second consumer, so
unifying it would have bought no duplication removal, only
WSJT-X-fidelity risk) but split into **#285** — FT8's OSD-escalation
gate and blind-CQ pass vs. their FT4/FST4 generic equivalents, itself
closed same-day after shipping an in-crate ratchet test
(`Q_NDEEP3_THRESHOLD == osd_escalation_gates::<Ft8>().1`) so a future
silent divergence between the two fails CI instead of relying on a
doc comment; **#287** — Q65 cross-candidate dedup window was a fixed
±4 Hz regardless of sub-mode, tighter than one tone spacing on the
wide sub-modes and letting a single Doppler-spread signal's two lobes
both survive as separate decodes, fixed same-day by scaling the
window to `(2 × TONE_SPACING_HZ).max(4.0)`. Also closed 2026-08-14,
not a GitHub issue: a code-sharing audit (#290-298) — see *Strategic
state*, track 1, for the full account.

Closed 2026-08-19: **#125** — License (GPLv3 vs. a permissive
license). Not a decision that was pending: it had been **answered in
full on 2026-06-05** and then sat open for ~2.5 months looking like an
unanswered outside question. GPL-3.0 is a requirement rather than a
preference — mfsk-core derives directly from WSJT-X (GPL-3.0-or-later)
in algorithms, DSP pipeline structure and FEC logic, so relicensing
would need permission from every upstream WSJT-X contributor. The
alternative offered was a clean-room reimplementation, which cuts
against source-faithfulness being a deliberate design goal for decode
parity. Closed as answered.

Closed since the 2026-08-15 snapshot: **#308** — FST4 had no
equivalent of WSJT-X's `i0±1` timing-jitter retry
(`fst4_decode.f90`'s `ijitter ∈ {0, +1, -1}`); ported to
`engine::pipeline` for FST4 at OSD depth, recovering 2 of the 5
CCIR-moderate trials the npre1/npre2 OSD port had lost, closed
2026-08-17. The other two halves of that issue were split out rather
than closed with it — the remaining 3/5 pruning question to **#311**,
and the embedded scheduling/cost half to **#310**, since the retry
triples the real-hardware candidate loop if ported unconditionally
and every embedded caller therefore still passes `&[0]`.

Closed since the 2026-05-18 snapshot (see the closed issue / `git
log` for the fix commit, not re-derived here):

- ~~**#25**~~ — MSK144 decode path (0.7.4).
- ~~**#58**~~ — coalesce redundant `compute_llr` (OSD/AP steps).
- ~~**#64**~~ — hoist `fft_cache` through host `decode_block_multipass`.
- ~~**#65**~~ — share `cd0` between SyncOnly + DataOnly `fill_symbol_spectra`.
- ~~**#73**~~ — `EqMode::Adaptive` collapsed into `EqMode::Local`.
- ~~**#74**~~ — `DecodeDepth::Bp` cheapest-rung caller check.
- ~~**#110**~~ — qso(FSM) slot parity tracking (TX/peer-decode collision).
- ~~**#113**~~ — CI: skip Build/Test/C++ matrix on docs-only PRs.
- ~~**#116**~~ — FT8: classify JTDX 5/18 misses on `qso3_busy.wav`.
- ~~**#117**~~ — FT8: auto-AP iaptype-2 from same-slot decoded callsigns.
- ~~**#146**~~ — FST4 AWGN sensitivity gap vs. WSJT-X's published table
  (0.7.1/0.7.2 close-out, see *Current line* above).
- ~~**#147**~~ — docs: `CHANGELOG.md` size + docs/ reader-facing vs.
  internal-notes split.
- ~~**#150**~~ — FT8 JTDX-only extra decodes ground-truth question
  (resolved as a side effect of 0.7.3's `OSD_HARDERRORS_MAX` widening).
- ~~**#156**~~ — MSK144 SNR sensitivity verification against real
  WSJT-X (0.7.4, see *Current line* above).
- ~~**#72**~~ — `DecodeStrictness` duplicate definition + uncalibrated
  copy for FT4/FST4. Turned out almost entirely resolved already
  (2026-07-18, before this closure): FT4's copy was retuned against
  a new `ft4sim`-based sweep (`core/pipeline.rs`'s `DecodeStrictness`,
  see `docs/notes/FT4_BENCHMARK.md` section 6), and FST4 bypasses
  these gates entirely per #146's fix — so the two enums now hold
  independently-calibrated, not-actually-duplicate values. Closed
  the residual "should the type itself be unified" question as no
  longer worth it (would need a protocol discriminator, net
  complexity increase).
- ~~**#171**~~ — Q65 AWGN sensitivity gap, root-caused and closed
  2026-07-19 (same day it was found). Two contributing causes, both
  resolved: (1) `coarse_search_for`'s reported best alignment is off
  by up to ~1/5 of a symbol period at low SNR with nothing refining it
  before decode — fixed with a local fine-timing retry
  (`decode_at_with_fine_timing_for`, mirrors WSJT-X's `q65_loops` `idt`
  loop); (2) the remaining gap after that fix was a comparison
  artifact, not a bug — WSJT-X's default `jt9` decode always has
  access to a free "CQ ??? ???" AP hypothesis that `decode_scan_for`
  (this crate's genuinely blind baseline) doesn't, by design.
  `decode_scan_with_ap_for` + a `"CQ"` hint (now a second column in
  `tests/q65_sim_sweep.rs`) matches WSJT-X's numbers almost exactly
  across all six wired sub-modes. FEC/BP stack was verified
  byte-for-byte correct against WSJT-X before either finding (10 code
  tables + WHT + every `pdmath` primitive, programmatic diff, zero
  discrepancies). Usage note for future sessions: applications wanting
  WSJT-X-equivalent behavior for CQ traffic should default to the
  AP-hinted path with at least a `"CQ"` hint, not the plain one.

Closed during the 0.6.x line (compact context table — for full
prose, see the closed issue / linked PR / `git log`):

| ref | summary | closed in |
|---|---|---|
| #40 | host wide-band coarse-sync candidate gap (AP-off recall 5/8 → 7/8) | v0.6.0 |
| #46 | sync consolidation — `decode_block::coarse_sync` + `compute_spectrogram` public | v0.6.0 |
| A0' | `decode_block_with_ap` as Step 4 of `process_one_candidate_inner` | v0.6.1 |
| #61 | fold `m5stack-core2` into S3 dual-core; retire Core2 bench (PR #76) | 0.6.3 |
| #63 | WSJT-X-faithful OSD `npre1`/`npre2` precoding (see "What landed in 0.6.3") | 0.6.3 |
| #105 | EMBEDDED.md rewrite (kept deep tech ref, refreshed for 0.6.4) | 0.6.4 |
| #23 | FST4-60A golden lockdown — wrong `NSPS`/`NDOWN`/`GFSK_BT` + missing `rvec` message scramble (PR #136) | 0.6.8 |
| #23 | FST4-15/30/120/300 sub-modes wired (`fst4_submode!` macro, WSJT-X-cross-checked; no golden WAV available — synth-roundtrip + source-verification only) | 0.7.0 |

(Note: `#23` appears twice — GitHub issue numbers aren't reused; this
issue was reopened/re-closed across two separate pieces of work,
FST4-60A golden lockdown then the remaining sub-modes.)

The JTDX AP-on-multipass recall is now 6/6 (was 5/6; see
`BENCHMARKS.md`'s FT8 section for the current number and mechanism).
The former remaining miss, `K1BZM DK8NE`, was closed by issue
[#182](https://github.com/jl1nie/mfsk-core/issues/182) — an
`osd_decode_npre1` LLR-fidelity gap (fed raw channel LLR instead of
BP-refined `bp_llr_zsum`), not the AP-list breadth this note used to
suspect.

Architectural notes that did not graduate to issues:

- **#48 option A** — `Protocol::Sync` associated type for
  type-system enforcement against future protocol-sync drift.
  Scoped out of v0.6.x (8 protocols + 2 macros + embedded feature
  matrix; benefit is speculative for non-FT8 protocols).
- **#49 cat D** — `engine::sync::coarse_sync<P>` `NotFt8` marker
  bound. Subsumed structurally by Phase 4 dispatch in v0.6.0; the
  doc note ("FT8 should not use this") is the practical backstop
  until #48 option A lands.
- **Embedded fine_refine** — design i16-throughout streaming FIR
  cd0 path so `fine_refine_pass1` can run on S3 within slot budget
  without a PSRAM round-trip per candidate. No issue filed yet;
  prerequisite is the ε `decode_block` restructure (see
  `docs/historical/CLEANUP_2026_05.md`) so the seam exists to hook a
  streaming alternative onto.

## Cleanup 2026-05

γ / β / δ / ε all landed in 0.6.3 (last stage PRs #77 + #83 + #79 +
#80 + #81 + #82 carved `decode_block.rs` from 3,517 lines into a
423-line parent + 6 stage submodules). See
[`docs/historical/CLEANUP_2026_05.md`](../historical/CLEANUP_2026_05.md) (HISTORICAL) for the
original four-stage plan; the per-stage acceptance-criteria template
documented there is reused for future cleanup waves.

# Roadmap (legacy, written for post-0.5.12)

Most of the post-0.5.12 plan landed in the 0.6.x bundle (PR #50,
shipped 2026-05-10) or is now tracked under a GitHub issue. The
sections below survive as historical context plus quick file-path
hints; the live worklist is the **Open follow-ups** section above.

## Phase A — Host protocol golden lockdowns

(A0 / A0' both closed in v0.6.x — see the closed-issues table above.)

- **A1** FST4-60A (`#23`) — closed in 0.6.8 (PR #136). Root cause was
  `Fst4s60`'s `NSPS`/`NDOWN`/`GFSK_BT` being never-revisited
  placeholders (wrong vs. `WSJT-X/lib/fst4_decode.f90` /
  `fst4sim.f90` / `gen_fst4wave.f90`) plus a missing `rvec` pre-LDPC
  message scramble; `tests/fst4_wsjtx_samples.rs` is un-ignored and
  recovers the golden decode.
- **A2** JT65 (`#24`) — **scope has moved on from the description
  below** (kept as a historical note; see "Open follow-ups" above
  for the current, accurate scope). Original framing: current
  implementation is JT65A; WSJT-X ships JT65B samples, so add a
  `Jt65b` ZST mirroring the Q65 sub-mode generic pattern. That's no
  longer the plan — JT65B/C are explicitly out of scope per the live
  issue, which is now about porting WSJT-X's `b65a`/`kvasd`
  soft-symbol erasure-metadata pipeline (or a lighter synth-only
  golden as a fallback).
- **A3** FST4-15/30/120/300 (`#23` stretch) — landed via the
  `fst4_submode!` macro (mirrors `q65_submode!`); all constants
  cross-checked against WSJT-X `fst4_decode.f90`/`fst4sim.f90` and
  pinned by an automated `DownsampleCfg`/`GfskCfg`-vs-trait-constant
  test. No golden WAV exists for these four periods (WSJT-X's sample
  tree only ships FST4-60A / FST4W-1800 recordings) — validated by
  synth-roundtrip self-consistency + source cross-checks only;
  requested by VK3NV (issue #23 comments) for a multi-period
  weak-signal messaging project. FST4-900 / FST4-1800 and FST4W
  remain deferred indefinitely (no user demand). **Update**: even
  without a real-audio golden, all five sub-modes now have a
  rigorous AWGN-sensitivity characterization vs. WSJT-X's published
  thresholds (issue #146, closed 0.7.1/0.7.2 — see *Current line*
  above and `docs/notes/FST4_BENCHMARK.md`), so "synth-only" no
  longer means "unquantified."

## Phase B — embedded controller line

**2026-05-17 pivot**: Phase 1 UAC hardware verification confirmed
**M5StickS3 cannot do USB host** (board lacks VBUS source circuit, ID
pin wiring, host power switch IC — silicon supports it, board doesn't
wire for it). Phase B splits in two:

- **Phase B-Stick** — `m5stack-s3-app`, demoted to **demo / acoustic
  fallback** path. Frozen after Phase 1.7.9 (2026-05-27 — see the
  detailed phase list below); Phases 1 (UAC), 2 (BLE CI-V), 5 (ADIF),
  6 (buttons), TX keying all roll forward to Phase B-Core.
- **Phase B-Core** — `m5stack-cores3-app` (NEW crate), the **main
  production controller**. M5Stack CoreS3 has AXP2101 PMIC + AW9523B
  I/O expander (USB host VBUS needs three AW9523B bits, not one —
  see `embedded-poc/CLAUDE.md` "USB host VBUS on CoreS3"),
  so UAC is viable.

Pattern: Phase B-Core reuses the `mfsk-app-shared` + `embedded-shared`
sibling-crate carve-out proven by `m5stack-core2-app` (Issue #61 Phase
2). See memory `project_m5stick_s3_no_usb_host` for the hardware
diagnosis, and `~/.claude/plans/happy-honking-globe.md` for the pivot
plan.

### Phase B-Stick — m5stack-s3-app (DEMO / FALLBACK)

Per `embedded-poc/m5stack-s3-app/CLAUDE.md` and the Phase 0..6 markers
in module doc-comments:

- **Phase 0 / 0.5 / 3** — Done. LCD bring-up, WAV-fed pipeline,
  4-region UI (status / waterfall / decoded list / TX strip).
- **Phase 4** — QSO FSM dry-run done (auto-CQ visible on LCD,
  `qso.rs` ~360 lines, 8 host-side unit tests). No TX audio
  synthesis on Stick (rolled forward to Phase B-Core).
- **Phase 0.6 / 0.7** — Done. WiFi UDP log streaming (UART /
  LCD / UDP fanout) + USB-CDC freeze fix (`println!` gated on
  `usb_serial_jtag_is_connected`) + runtime boot-mode selector
  (NVS + KEY2 long-press) for WiFi / decoder coexistence.
- **Phase 1 (UAC)** — ❌ **Not viable on Stick hardware**. The UAC
  code that shipped via PRs #29-#35 + #102 (`uac.rs` ~445 lines,
  managed-component bindings, board-agnostic) remains in-tree as
  canonical reference; lifted to `m5stack-cores3-app` in
  Phase 0-Core, then hoisted into `mfsk-app-shared` in
  Phase 1.5-Core after dual-board verification.
- **Phase 1.5 (Acoustic capture)** — ✅ Done (Phase 1.7.1-Stick).
  ES8311 ADC mic-mode → I2S RX → `LinearResamplerI16To12k` →
  `decode_pipeline::run_with_source`. `BootMode::Acoustic` added to
  the NVS cycle; live on `main`.
- **Phase 1.7 (QSO mode bidir I2S + demo toggle)** — ✅ Done. PR
  #121 adds a BtnA-long-press demo mode inside `BootMode::Qso`:
  audio source switches from mic to baked-in `qso3_busy.wav`,
  speaker plays back the WAV, TX scheduler is suppressed, LCD shows
  `DEMO MODE — TX OFF`. Deterministic demo safety net when ambient
  acoustic conditions kill the live WebFT8-to-mic path.
- **Phase 1.7.9 (cold-start auto-sync bootstrap)** — ✅ Done. PR
  #133 / v0.6.6. Auto-sync gains a fourth branch:
  `n_dec == 0 && best_n == 0 && bootstrap_dt_med.is_some()` →
  coarse_sync top-5 DT median drives a one-shot slot shift,
  `best_n` stays at 0 so the next confirmed-decode slot reclaims
  HWM. Removes the "BtnA required" cold-start dead-end for
  no-GPS / no-NTP operation. Helper
  (`mfsk_core::engine::sync::bootstrap_dt_median`) shared with
  WebFT8, gated by `tests/ft8_coarse_sync_bootstrap.rs`.
- **Phases 2 / 5 / 6 / TX keying** — Rolled forward to Phase B-Core.
  Stick frozen after Phase 1.7 demo toggle (1.7.9 is a
  controller-only auto-sync delta — no new HW surface).

### Phase B-Core — m5stack-cores3-app (MAIN TARGET, NEW)

Hardware: **M5Stack CoreS3** (full variant with 500 mAh battery +
GC0308 camera + LTR-553 proximity + BMI270 IMU). CoreS3 SE (cheaper
sensor-stripped variant) deferred until that hardware is procured;
when needed, gated behind `cfg(feature = "cores3_se")` in the same
crate.

**Status (corrected 2026-07-19 — this section previously said
"planned. Brings up after CoreS3 unit arrives", which was stale and
self-contradicted by this file's own Phase D section citing a
CoreS3 benchmark log)**: Phase 0-Core and Phase 1-Core both shipped
**2026-05-23** (PR #132, commit `1a93c92`) — CoreS3 hardware has
been on the bench and flashed since then. **Phase 1-Verify**, tracked
as [#163](https://github.com/jl1nie/mfsk-core/issues/163), closed
2026-08-23 — the UAC code now has ten unbroken minutes of live IC-705
capture behind it. Before that there had been no cores3-app feature
commit since 2026-06-07 (6+
weeks). The `(tasks #33/#34)` / `(task #48)` / `(task #49)` /
`(task #50)` / `(task #51)` annotations previously on the phases
below were leftover placeholder numbers that resolved to unrelated,
already-closed May items (CI hygiene, the v0.6.2 release, S3
boot-mode work) — removed below rather than left misleading.

- **Phase 0-Core** — DONE (2026-05-23). Crate skeleton: board.rs
  (ILI9342C SPI pins, I2C0 for AXP2101 + AW9523B + FT6336U, USB-OTG
  fixed GPIO 19/20, ES7210 audio pins), pmic.rs (AXP2101 + AW9523B
  init, distinct from S3-app's M5PM1 and Core2-app's AXP192),
  display.rs (ILI9342C via mipidsi — Core2-app pattern), main.rs
  orchestration, decode_pipeline.rs thin wrapper. First flash
  verified: I2C scan confirmed all peripherals, LCD up, 7/7 decodes
  on `qso3_busy.wav` via wav_sim, 137 ms post_slotend.
- **Phase 1-Core** — DONE (2026-05-23, same day). UAC: `uac.rs`
  cloned from s3-app verbatim; pmic.rs drives **AW9523B P1
  (`BOOST_EN` port1 bit7 + `USB_OTG_EN` port0 bit5 + `BUS_OUT_EN`
  port0 bit1) HIGH** before `usb_host_install()` (omission =
  floating VBUS = floating host capability). `BootMode::Uac`
  dispatch arm wired. `cargo check --release` clean on
  `xtensa-esp32s3-espidf` — not yet flashed/run against real
  hardware.
- **Phase 1-Verify** — **DONE 2026-08-23**
  ([#163](https://github.com/jl1nie/mfsk-core/issues/163)). An IC-705
  enumerates through its internal hub and the reader sustained
  192,512 B/s for ten minutes (125 MB, 30,520 packets, 0 errors) with
  WiFi associated and the decode pipeline running slots off the live
  stream. What remains on this checkpoint is a decode from a real
  on-air signal rather than a quiet band — the transport, not the
  sensitivity, is what was in question. This was the
  actual current bottleneck for the whole Phase B-Core line.
- **Phase 1.5-Core** — Hoist `uac.rs` into `mfsk-app-shared` (gated
  `cfg(feature = "uac")`); both s3-app and cores3-app consume via
  shared. Deferred until Phase 1-Verify passes (no premature
  abstraction) — unchanged blocking logic, just correctly blocked on
  a real open issue now instead of a placeholder task number.
- **Phase 2-Core** — BLE CI-V to IC-705. Same `civ.rs` work that
  was queued on s3-app; CoreS3 also has BLE.
- **Phase 5-Core** — ADIF (`flash_log.rs` + `adif.rs`). Crate-agnostic;
  could be hoisted to `mfsk-app-shared` from day 1.
- **Phase 6-Core** — FT6336U capacitive touch driver (CoreS3 base
  has no physical buttons beyond Power). Replaces the Stick
  `buttons.rs` paradigm with touch zones (menu / decoded-list
  tap-to-select / TX-strip tap-to-send).
- **Phase 7-Core** — TX keying (paired with Phase 2-Core
  `civ::set_ptt` + Phase 1-Core TX audio synth to UAC OUT endpoint).

`mfsk-ffi-ft8/src/stream.rs::mfsk_ft8_stream_*` and `embedded-shared`
resampler API are the seams shared between B-Stick and B-Core.

## Phase C — Quality / infra

- **C1** Embedded CI cross-build — still pending. `xtensa-esp32-espidf`
  / `xtensa-esp32s3-espidf` targets only build at release time today.
  The cleanup-2026-05 ε restructure (`decode_block` split) is a soft
  prerequisite so the embedded build does not have to re-validate a
  3500-line file on every PR.
- **C2** Reproducible release builds — `esp-rs/xtensa-toolchain@v1.5`
  is pinned, but the `+esp` Rust version inside the action still
  floats. Pin or capture in the artifact.
- **C3** m5stack-s3-app release artifact — `release.yml` builds
  `libmfsk_ft8.a` only; add a job emitting a flashable
  `espflash save-image` for the s3-app once Phase 1 UAC stabilises.

## Phase D — ESP32-S3 LX7 PIE SIMD acceleration

Pure decoder-side perf push for the S3 critical path (pass 2 +
stage1_inc). Detailed plan in [`docs/notes/PHASE_D_PIE_SIMD.md`](PHASE_D_PIE_SIMD.md).

| Sub-phase | Target | Status |
|---|---|---|
| **D1** | Re-bind esp-dsp FFI to `_aes3_` (LX7 PIE) | **Done** — commit `053bd67`; sc16 PIE 1.98→1.65s (−17%) on S3 sequential bench |
| **D2′** | f32 scalar Goertzel micro-optimisations (scratch hoist, alignment) | **Done** — commit `053bd67`; bounds-check hoisted out of inner loop |
| **D3** | PIE allsum + score for `coarse_sync` / `stage1_inc` | **Done** — commit `0b7978b`; sliding-window allsum |
| **D4** | PIE `\|x\|²` post-FFT in `stage1_inc` | **Done** — commit `a8235e7`; demux-mag² DC-hoist + 4× unroll |

CoreS3 dual-core wav_sim (qso3, 2026-06-05): **post_slotend 136〜138 ms, 7/7 decodes**.
Log: `embedded-poc/m5stack-cores3-app/logs/cores3_phaseD1_2026-06-05.log`.

## Phase E — WSPR embedded RX (#260)

Opened 2026-08-11, resolved 2026-08-14. Distinct from Phase B: WSPR
never goes through `decode_block`/`fixed-point`/the UAC controller
stack this phase's app crates are built around, so nothing here
required cores3-app or the UAC path to move — it's a
`wspr::decode`/`wspr::ddc` decoder-level effort, running the same host
f32 pipeline on-device via `fft-extern`.

**The blocker this phase closed**: no channelizer. `wsprd`'s own
whole-slot FFT channelizer (`wspr::baseband::decimate_to_baseband`)
needs an 11.25 MiB `Complex<f32>` buffer at a 1 474 560-point FFT —
neither a power of two nor within `esp-dsp`'s 8 192 ceiling — so
there was no path from "audio arrives" to "baseband exists" on any
target this crate ships for. Every WSPR device timing before this
phase was measured against a host-baked baseband as a stand-in.

| Item | Status |
|---|---|
| Streaming down-converter (`wspr::ddc`) | **Done** — single-stage FIR, ~25 KB state. Verified against the reference channelizer: golden 9/9, AWGN sweep within 1 trial/cell of 500, 0 phantoms either way. |
| Dual-core safety | **Done** — persistent worker + job queue (was spawn-per-pass, which silently fell back to sequential once WiFi held memory); worker stack reserved at boot before WiFi (was `.bss`, and before that a lazy heap allocation that silently lost the race — see `embedded-shared/src/worker_arena.rs`); scan-task stack placed before WiFi starts. |
| Fano cycle-budget split | **Done** — host now runs `wsprd`'s own 10 000 cycles/bit (`wsprd.c:799`); embedded keeps 5 000 via `wspr-fano-cap-fast`, paying floor recall for the slot deadline. Swept with a phantom-count column added to `wspr_awgn_snr_sweep` (a false-decode cliff exists above ~200 000; not visible from recall alone). |
| Coarse-stage perf | **Done** — loop interchange + a redundant-sqrt hoist in `refine_alignment_top_k` (97.5 % of the coarse stage was PSRAM re-reads, not FFTs); bit-exact, no new memory. |
| wsprnet spot reporting (`mfsk_app_shared::wsprnet`) | **Done**, off by default — ported from WSJT-X's own `Network/wsprnet.cpp`. `SpotSink::Http` upload path implemented but unverified against a real endpoint. |
| Steady-state pipeline measurement | **Done** — 4 consecutive slots, WiFi associated, front end running at its real duty cycle beside the decoder: decode 82.8–90.1 s against a 110 s deadline (120 s slot − 10 s spot-upload reserve), DDC 18.5–24.1 s under that load. 9/9 golden held every slot. |
| Live audio capture | **Not done here** — #163 closed 2026-08-23 and the shared `uac.rs` is proven on the FT8 controller (10 min, 0 errors), but `wspr_app` has not been run against a radio. Everything above is still measured against WAV-fed/synthetic baseband. |
| Two-stage DDC decimation (more margin) | **Deferred, not abandoned** — a first estimate (4× filter-cost reduction) didn't survive re-derivation by hand; steady-state margin measured at 19.9 s made it not worth chasing further this round. |

Full measurement account, including several attempts that measured
negative and were kept rather than deleted (an over-aggressive Fano
cap, a `.bss`-everything attempt that broke the heap, a DRAM-placement
experiment for the DDC that made things worse) so they aren't
re-attempted, is
[`docs/notes/WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`](WSPR_EMBEDDED_MEASUREMENT_RESULTS.md).
Branch `bench/wspr-s3-candidate-loop`, landed via PR
[#286](https://github.com/jl1nie/mfsk-core/pull/286). Narrative
writeup on the issue itself:
[#260](https://github.com/jl1nie/mfsk-core/issues/260).

## Quick file-path index

- Host FT8 reference (post-ε split, 0.6.3):
  `mfsk-core/src/ft8/decode_block.rs` (423-line parent / facade),
  `mfsk-core/src/ft8/decode_block/types.rs` (audio sample +
  tunables), `…/spectrogram.rs` (`Spectrogram` +
  `compute_spectrogram`), `…/coarse_sync.rs` (Costas search +
  allsum), `…/fill_symbol_spectra.rs` (per-symbol DFT family —
  **`fill_symbol_spectra_goertzel` is the current path; the
  BASIS `fill_symbol_spectra_into` is back-compat only, still
  present as of 0.7.4, tracked for removal under
  [#162](https://github.com/jl1nie/mfsk-core/issues/162)**),
  `…/process_candidates.rs` (engine +
  facade impls), `…/osd_strategy.rs` (OSD dispatch, #63 hook).
  Host `decode_frame*` family + `refine_fine` gate:
  `mfsk-core/src/ft8/decode.rs`.
- Probe templates: `mfsk-core/src/jt9/decode.rs::gate_diag::probe_missing_goldens`.
- Protocol-specific: `mfsk-core/src/fst4/decode.rs` ⇔
  `WSJT-X/lib/fst4_decode.f90`; `mfsk-core/src/jt65/{mod,rx,decode}.rs`
  ⇔ `WSJT-X/lib/jt65_decode.f90`.
- Embedded shared layers (board-agnostic):
  `embedded-poc/embedded-shared/src/{pipeline,dual_core,stage1_inc,esp_dsp_fft}.rs`
  (decoder layer), `embedded-poc/mfsk-app-shared/src/{qso,wifi,log_sink,boot_mode,ui/}`
  (controller layer).
- Embedded apps: `embedded-poc/m5stack-s3-app/src/{uac,civ,qso,audio,buttons,flash_log}.rs`
  (M5StickS3, demo / acoustic fallback per Phase B-Stick);
  `embedded-poc/m5stack-core2-app/src/` (Core2 sibling,
  wav_sim only).
- WSPR embedded (Phase E, board-agnostic): `mfsk-core/src/wspr/ddc.rs`
  (streaming down-converter), `embedded-poc/embedded-shared/src/wspr_dual_core.rs`
  (persistent worker), `embedded-poc/embedded-shared/src/apps/wspr_bench.rs`
  (the bench body — steady-state pipeline mode is `run_with_hooks`/
  `PIPELINE_SLOTS`), `embedded-poc/m5stack-cores3-app/src/bin/wspr_bench.rs`
  (the bin, incl. `MFSK_WSPR_SPOT`/`MFSK_WSPR_BENCH_WIFI` build-time
  switches), `embedded-poc/mfsk-app-shared/src/wsprnet.rs` (spot
  reporting).
- User manual: [`docs/reference/MANUAL_M5STICKS3.md`](../reference/MANUAL_M5STICKS3.md)
  ([JA](../reference/MANUAL_M5STICKS3.ja.md)).
- Infra: `.github/workflows/{ci,release}.yml`.
