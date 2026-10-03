# Changelog

## Unreleased (0.13.0) — one persistent `Decoder<P>` per mode on the WSJT-X parameter block (breaking), `Depth` as `ndepth`, QSO-context AP for FT8/FT4/FST4, a pull `IqReceiver` with a clock that follows the host, one C decoder handle for every mode (breaking), per-channel decoder threads in the skimmer

### 0.13.0 — decode API redesigned on the WSJT-X decoder model (breaking)

- **`Decoder<P>`: one persistent decoder per mode, as `jt9` runs.** It owns what upstream keeps between periods (the
  callsign hash table, FT8 a7 rows, Q65 period averages, WSPR's call table) and is driven per period by a
  `DecodeParams` that is `jt9com`'s block (`nfa/nfb`, `nfqso`, `nftx`, `ntol`, `ndepth`, `mycall/mygrid`,
  `hiscall/hisgrid`, `nQSOProgress`, `ncontest`, AP, `emedelay`). Library-only options are a typed `Extras` per mode, so an
  option a mode lacks does not compile (`AnyDecoder` returns `Unsupported`). Rows stream through `on_row` as found.
  Removed: every per-family `DecodeRequest`/`SniperRequest`/`MultiPeriodRequest` as public API (the wide-band ones are
  crate-private; `SniperRequest` stays for the WSPR boards), `known()`, `previous_cycle()`, `hash_table(Arc)`,
  WSPR `table()`, `wsjtx_depth`, `iq::IqMode` (now `registry::Mode`). Migration table: `docs/reference/LIBRARY.md`.
- **`Depth` decides the search, as `ndepth` does (breaking behaviour).** Default is Deep and the defaults follow the
  WSJT-X GUI (FT8/JT65 AP off, band 200–4000 Hz, FST4 600–1400 Hz). Ported per mode from v3.2.0-rc1: FT8 syncmin/passes/OSD and
  nfqso-first candidate order, FT4 passes and OSD, FST4 OSD without the i0±1 jitter at Fast, Q65 maxiters 40/60/100 and
  `q65_loops` windows, JT65 2/2/4 passes with `subtract65` and nvec 100/1000/1000, JT9 Fano limit and the nqd narrow
  pass, WSPR `-qB` / `-C 500 -o 4` / `-d` with `unpk_`'s Type-3 sanity checks. Tests: `tests/decoder_depth.rs`.
- **QSO-context AP for FT8, FT4 and FST4** from upstream's `naptypes` tables, derived from `station` + `qso` + `ap`
  (`ApHint` stays as the library's "hunt one DX"). A weak-reply sweep on FT8: 0 of 30 decoded with AP off, 20 of 30 on.
- **WSPR now reproduces `wsprd`, signal for signal, on the WSJT-X golden.** Compared against an instrumented
  `wsprd` v3.2.0-rc1: SNR within 0.02 dB (it was 0.05-0.26 dB, and one signal rounded to a different integer), DT
  equal to 3 decimals, frequency to 0.1 Hz, and G8VDQ (-23 dB), which this crate had needed a larger Fano budget for,
  decodes at the default one. Causes, all ported literally from `wsprd.c`: the noise floor is `tmpsort[122]` not 123;
  the sync window constant is 0.006147931; the front pad is a whole number of the 128-sample coarse strides, so every
  lag tried is `wsprd`'s; the whole recording is read; the coarse drift bin truncates `ifr + offset`, not the offset
  (every candidate came out with drift -1); candidates run strongest SNR first; and each decode is subtracted at once,
  with `wsprd`'s float-accumulated reference phase, instead of decoding a pass in parallel and subtracting at its end.
  Reported frequency is the centre of the four tones, and the power prints two wide (` 3`, not `3`). The AWGN sweep:
  50 % crossing -31.40 dB (was -31.33), 1 phantom in 260 trials as before; 20 ms/file at low SNR (was 52). Detail in
  `docs/notes/WSPR_UPSTREAM.md`.
- **JT65 message averaging (`ndepth & 16`), as `avg65`.** A candidate the single period fails on is kept (period, DT,
  frequency, 63 × 64 symbol powers; up to 64 periods per decoder) and summed with the saved periods of the same parity
  within 0.2 s of DT and `tol_hz` of frequency; two or more are decoded as one (the Chase search). Needs
  `SlotInput::period`. On noise where no single period decodes, the sixth summed one does. A JT65 frame also came back
  once per candidate around it (8 rows): the repeat test compared an unpadded start with a padded one since #283.
  Not ported: the JT65B/C smoothing loop; `deep_search` (`hint65`) is carried and not read.
- **JT9 and JT65 drop the all-zero codeword** (`000AAA 000AAA RA90`, `jt9fano.f90:88`, `decode65b.f90:27`): a noise-free JT9
  period returned the true message plus three of these.
- **`IqReceiver` is a pull model on a clock that follows the host.** `push_*` yields owned `CompletedSlot`s; the caller
  owns one decoder per channel, so per-channel options and hash tables follow. `SlotGrid`/`SampleClock`/`SlotCutter`
  (integer arithmetic, no atomics) slew a drifting clock at up to 400 ppm and step past 1 s, starting each slot on its own
  boundary with 0.2 s of overlap: a 24 h simulation at +13 ppm loses no slot (fixed-length slots slid off the UTC grid and lost one
  at about 21 h). `retune()` reports which channels paused or resumed.
- **C ABI 3: one `mfsk_decoder_*` handle for every mode (breaking).** `MfskParams`/`MfskExtras` are size-versioned and
  initialised by `mfsk_params_init`/`mfsk_extras_init`; rows stream through a callback; an option a mode lacks is
  `MFSK_UNSUPPORTED`; `MfskStream` cuts slots on the UTC grid. The session handle and every `mfsk_*_decode*` function are
  gone. cpp_smoke, Kotlin and the Swift package were ported and run; on macOS the Swift suite passes 98 of 98, after
  one test was updated for `mfsk_stream_clear` now dropping the slot being cut as well. Old→new table:
  `docs/reference/BINDINGS.md`.
- **Embedded:** the boards call the stage functions, which did not change; `cargo +esp check` passes on all four boards.
  New CI/pre-push rows build `Decoder`/`AnyDecoder` under `alloc` + `fft-extern` and the union of the board features.
  `Decoder::new` allocates nothing.
- **Skimmer sample app:** one decoder thread per channel behind a bounded queue (a slow decode no longer stalls the socket
  reader or the clock), per-channel band / DX call / depth (CLI `:band=LO-HI:dx=CALL:depth=..`, GUI ⚙ dialog, applied to a
  running skimmer at the channel's next slot), and a `STATUS.log` of push, decode, queue and drop figures for long runs.

- **0.12.0 is yanked (2026-10-03).** Its `iq::IqReceiver` lost every other slot with a real-clock time anchor and
  never resolved hashed (`<...>`) callsigns, so it was unusable for live reception; the fix is an API redesign, which
  ships as 0.13.0 rather than a 0.12.x patch. A `Cargo.lock` already holding 0.12.0 keeps working; new resolutions of
  `mfsk-core = "0.12"` fail until 0.13.0, and FT8/FT4/WAV users can stay on 0.11.x meanwhile.
- **`IqReceiver` decodes every slot when its time anchor is off the 12 kHz grid (#534).** A slot starts at its
  boundary rounded up to a sample, so the sample after a slot can lie just past the next boundary, and
  `next_boundary` skipped to the boundary after that. With an anchor off the grid, which a wall-clock timestamp
  nearly always is, the receiver decoded every other slot and could also lose the first one. A stream stamped from the
  wall clock decoded 2 slots of 4; it now decodes 4 of 4, 21 FT8 messages each. Every test had
  anchored on a whole second, which is on the grid. New tests cover arbitrary anchors for every slot period and two
  back-to-back slots anchored 40 µs off.
- **Sample app: a SpyServer skimmer, `apps/skimmer/`.** `skimmer-core` drives `IqReceiver` from a SpyServer
  (protocol as SDR++ speaks it) and reports connection, stream, decode and health as events; the `skimmer` CLI prints
  them and appends WSJT-X `ALL.TXT`-style lines; a Tauri + Svelte GUI (`apps/skimmer/gui`, built on Windows) shows
  channels with slot-progress pies and band presets from WSJT-X's frequency list, and decodes banded by slot. It shares the radio with SDR#: it never tunes the
  device unless `--tune`, sets only its own DDC centre, gives control back, and pauses channels outside the band. The
  socket is read on its own thread: the first decode (400-650 ms) had made the server drop IQ on Windows in 3 of 3
  runs, 0 after. Against an Airspy HF+ at 7041 kHz beside SDR#: 19-26 JA FT8 decodes per slot, WSL and Windows.
- **An XCFramework for iOS apps.** `bindings/swift/scripts/build-xcframework.sh` builds `libmfsk` for the iOS device
  (`aarch64-apple-ios`) and the Apple-silicon simulator (`aarch64-apple-ios-sim`) with the `mobile` features, and
  packages both as `target/xcframework/Mfsk.xcframework`. Each slice carries `mfsk.h` and a module map, so Swift code
  imports it as `CMfsk`, the same name the SwiftPM package uses. Before it reports success, the script compiles and
  links a small Swift program against each slice. It is not run in CI, and `Package.swift` still links by `-L` rather
  than through the XCFramework.
- **Skimmer installers for Windows and macOS, attached to each release.** `skimmer-installers.yml` builds the GUI into
  `mfsk-skimmer-vX.Y.Z-windows-x64-setup.exe` (NSIS) and `mfsk-skimmer-vX.Y.Z-macos-arm64.dmg` (Apple silicon). The
  release workflow attaches both after the crates.io publish without waiting on them, so a GUI build failure does not
  hold the library back. Pull requests that touch `apps/skimmer/` run the same build. The macOS app is signed ad hoc
  and not notarized. It carries `NSLocalNetworkUsageDescription`: without that entry, or with only the linker's
  signature, macOS refused the SpyServer connection with `No route to host` and never asked for permission.
- **The skimmer reconnects when the server goes quiet.** A SpyServer that dropped the client while a Mac slept left
  the socket ESTABLISHED, and the skimmer, which only writes when it changes a setting, sat "connected" with no data
  and never reconnected. `Conn::read` now takes 15 s with nothing from the server as a dropped connection, and the
  usual retry follows. With streaming off, while no channel fits the band, it pings every 5 s so the PONGs keep an idle
  connection from looking dead. The GUI also stops posting a notice for every lost IQ message: the health line counts
  them and the health log still records each one.
- **WSPR: `Decoded::freq_hz` and the text report now match `wsprd`'s own numbers.** `freq_hz` carried tone 0, 2.197 Hz
  below what `wsprd` reports — the centre of the four tones (`wsprd.c:1496`, `1500 + f1`) — so a caller comparing
  against `wsprd`'s output saw every WSPR frequency low by 1.5 tone spacings (ND6P: 1444.04 Hz here, 1446 Hz in
  `wsprd` on the WSJT-X golden). The power field also printed without `wsprd`'s `%2d` padding, dropping the leading
  space on single-digit dBm values (`wsprd_utils.c:353,364,398`). The Type 1/2/3 power arithmetic itself was already
  correct; only `freq_hz` and the two display fixes change. Snapshot fixtures move in their frequency bits only (36
  rows for 36).
- **Skimmer GUI: per-channel decode counts no longer disagree with the table, and the window keeps updating when
  covered or minimised.** Counts were incremented on arrival while the table itself lagged a `requestAnimationFrame`
  queue that macOS/browsers starve for a covered or minimised window — so a count could read higher than the rows the
  table ever showed, and follow-scroll (keyed on the row count, which stops changing once the 5000-row cap is hit)
  could stop following. Counts now come from the same batch the table applies, on a 100 ms timer instead of
  `requestAnimationFrame`, and the health line gains a `window got N` figure — the raw count of decodes the window
  received, to compare against what the table shows.
- **`scripts/release-status.sh` reports a yanked published version.** A tagged version is not necessarily a live one:
  this is exactly what happened to 0.12.0. The script now queries crates.io for the tag's published version and
  prints `YANKED`, its `yank_message` if one was set (else a pointer at this CHANGELOG's own "is yanked" entry), or a
  neutral "could not check" line when crates.io is unreachable — never a false `ok`.

## 0.12.0 — measured against WSJT-X 3.2.0-rc1 on defined tasks: behind on none but legacy JT65, faster on all but WSPR; FT8 and FT4 subtract by default (breaking); WSJT-X 3.2's JTTY and Q65 additions; wideband IQ input (#534); one decode entry shape for every mode and a transmit path generic over the protocol (breaking, #403 / #391), dt measured from the nominal start (breaking, #397), one `SyncCandidate` / `SearchParams` (breaking, #394); the CoreS3 receives FT8, FT4 and JTTY

- **Breaking (behaviour): FT8 and FT4 subtract by default, as WSJT-X does.** A plain `DecodeRequest::new(..).decode()`
  now runs `.sic_early()` on FT8 and `.sic_rounds(3)` on FT4. Before this it ran a single pass, which WSJT-X never does.
  `.single_pass()` asks for one pass, and the C ABI has `MfskDecodeParams::single_pass`, in what was padding, so the
  layout is unchanged; Kotlin `singlePass`, Swift `singlePass`. FT8 gains up to 21 points of recall on a crowded band
  (40 signals: 60 % to 81 %) and 0.3-1.5 dB on fading, at 2-3× the time. FT4 loses nothing and costs the same on noise.
  A caller that timed or budgeted the old default, such as WebFT8's phase-1 pass, should add `.single_pass()`. A budget
  on the new defaults is polled per stage, not per candidate.
- **FT4 `.sic_rounds()`: the blind CQ AP rung restored, and rounds as `ft4_decode.f90` runs them (#553).** The SIC path
  passed an empty AP list, so it lost 0.5-1.4 dB to FT4's own single pass. Against `jt9 -5 -d 3` it was 4 of 4
  channels behind. It also relaxed `sync_min` x0.75/x0.5 in later rounds and ran every round, where upstream keeps
  `syncmin` and stops when a pass adds nothing. That made it 4-7x slower than `jt9`. Both now match upstream. On the
  upstream baseline no channel is behind, and the 1040-file corpus decodes in 0.9 s instead of 10.9 s. A regression test
  pins the AP rung: at the threshold the old path decoded 2 of 24 seeds where the single pass decoded 9.
- **Builds for aarch64 on rustc 1.99.0 no longer hang.** At opt-level 3, 1.99.0's loop vectorizer never finishes
  on `ft8::refine_fine`'s Costas phasor table (cos/sin plus a ±π wrap), so a release build for Android, iOS or
  Apple-silicon macOS stopped in `mfsk-core` indefinitely. 1.98.1 and x86_64 were unaffected. A `black_box` on the
  phase step keeps that loop scalar; the arithmetic, and so the table, is unchanged. A 20-line standalone copy of the
  loop reproduces it: 36 ms on 1.98.1, still compiling after 60 s on 1.99.0. Found when CI's Android and Swift jobs
  ran for six hours on 2026-10-01; every CI job now has a `timeout-minutes`.
- **CoreS3: transmit bring-up probes, and PTT off on every CAT connect.** `MFSK_CORES3_TX_BRINGUP=a` opens the radio's
  audio OUT beside the IN stream and CI-V and streams one silent FT8 frame (never keys); `=b` keys PTT over CI-V with
  no audio for 15 s and 35 s, logging the `FB` and `1C 00` readback latency and the IN stream while keyed. Both are
  off by default and not yet run on hardware. In every build the CAT task now sends PTT off as its first frame, so a
  board that reset while transmitting unkeys the radio. Stage P1 of the CoreS3 QSO/TX plan; `src/tx_bringup.rs`.
- **CoreS3: three modes, FT8, FT4 and JTTY, in the default build.** The mode picker lists FT8, FT4 and JTTY;
  FST4 leaves the menu but keeps its receiver behind `--features fst4` (a board whose NVS says `fst4` still boots it,
  or FT8 with a logged reason). `ft4` and `jtty-rx` are now default features: a plain `cargo build --release` used to
  give an FT8-only image whose picker offered FT4 and JTTY and fell back to FT8 on either. The image grows from 2.36 to
  2.72 MB of a 9 MiB partition. JTTY gets FREQ presets, WSJT-X v3.2.0-rc1's own (`models/FrequencyList.cpp`). First
  step of the CoreS3 QSO/TX plan (automatic FT8/FT4 QSOs, JTTY templates, real transmission).
- **Q65 at 0.31x / 0.29x `jt9 -3`'s time, from 0.77x / 0.69x (#555).** `DecodeRequest::ap_hint()` now tries each
  candidate without AP first, as `q65_decode.f90`'s `ipass` loop does, so one hinted scan does what a plain scan plus
  a hinted one did; the new `Q65Result::ap` says which carried a decode. A candidate inside a decoded signal's band is
  skipped (`q65_decode.f90:375-377`): a strong signal cost 50 BP calls, now 1. The QRA BP kernels are specialised to
  `M = 64` (same arithmetic, noise files 119 -> 68 ms), and `maxiters` is upstream's 40 per depth instead of 50: 3 of
  2 640 sweep trials lost, none of which upstream decodes. No group behind upstream. Details: `BENCHMARKS.md`.
- **WSPR's Fano decoder resets its root node, as `fano.c:127` does (#557).** The node scratch is pooled across a
  candidate's 17 DT positions and its blocksizes (since `362b7690`), and the root's `encstate` was never reset, so
  from the second call on a search could start from the previous call's low bit. A test now holds every pooled call
  to a fresh one. The WSPR crossing does not move. Looked at while trying to halve WSPR's time; that was not found
  without changing decodes (`BENCHMARKS.md`): the crate runs at `wsprd`'s speed, Fano-bound like it.
- **Q65's sync smoothing covers only the bins the search reads (#556).** `nsmo` passes of `smo121` (E: 128) ran over
  the whole 0-6 kHz row; now over the search window ± Max Drift, widened by `nsmo`, which is bit-identical where it
  is read (unit test, and the sweep's 2 640 trials unchanged). Q65-120 noise files 116 -> 96 ms; the T1 task 0.29x /
  0.26x `jt9 -3`'s time.
- **Breaking (behaviour): FST4 finds candidates as `get_candidates_fst4` does, so its `sync_min` is upstream's
  `minsync` (#554).** FST4 used the generic 2-D Costas search. On noise it let `max_cand` candidates through on every
  file, and each one paid a full ±1.5 s `fst4_sync_search`; upstream refines 2-6. The search is now a port of
  `get_candidates_fst4` and `fst4_baseline` (`engine::fst4_coarse`): a four-tone CCF over the whole-slot FFT the
  pipeline already builds, divided by a fitted noise baseline, then peaks peeled off with a CLEAN loop until one falls
  below `minsync`. `sync_min` is therefore on FT4's baseline-normalised scale, where noise sits near 1.0: the registry
  and `mfsk_mode_defaults` publish 1.20 (FST4-15: 1.15) and 200 candidates, as `fst4_decode.f90:308-309` does, with
  `sync_scale` `BaselineNormalised`. A caller passing the old 0.8 / 50 now admits every peak above 0.8 and stops at
  50. On a busy recording that can drop a real signal: on the WSJT-X FST4-60 sample, CQ N5TM is the 145th candidate.
  Against `jt9 -7 -d 3` on the T1 task this takes FST4 from 3.1x/2.8x upstream's time to 0.42x/0.53x (noise /
  crossing), with no channel group behind upstream; unexpected decodes fall from 19 to 5 over the 20 groups (FST4-120
  AWGN: 9 to 0). The embedded wideband monitor keeps the generic search and its 0.8 / 50.
- **FST4's a-priori pass as `fst4_decode.f90` runs it: no longer 0.6-1.3 dB behind `jt9 -7` (#554).** Against
  `jt9 -7 -d 3` on the T1 task FST4 was behind in 15 of 20 channel groups. Every frame upstream decoded and this
  crate missed, in the sample traced, came from upstream's CQ AP pass, with 40-58 hard errors. The AP rung here
  rejected anything above 36, FT8's bound for a 174-bit code, which `fst4_decode.f90` does not have. The rung now
  follows upstream: no bound, OSD depth 3, and only the nsym=8 LLR set, rather than every variant of the ladder (with
  every variant it admitted 0-6 more false decodes a group). `Strict` keeps a bound. Now no group is behind upstream,
  and FST4-15 CCIR-moderate is 0.95 dB ahead. Unexpected decodes did not change. A hint of `CQ` alone no longer runs
  the CQ hypothesis twice (FT4 and FST4). FST4 takes 3.1x/2.8x upstream's time (noise/crossing), from 4.1x/3.9x.
- **Tier C's method and setup are one manual, `docs/notes/TIER_C_MANUAL.md`**: principles, which WSJT-X tree
  builds what, corpus generation, running and reading the output, and the traps hit so far. `BENCHMARKS.md` keeps
  the results. FT8, FT4 and FST4 now sweep as their upstream task T1, so one sweep serves both baselines. The
  separate `ft8_sic_early` pass is folded into FT8's T1, which runs `.sic_early()`. A full run takes 548 s, against 383 s.
- **Tier C compares against upstream on a defined task, not only against this crate's past.**
  `sweep-baseline.json` could not show a decoder that was behind WSJT-X from the start; Q65 ran 2.9–4.5× slower
  than `jt9` unnoticed (#552). A task (band, operator knowledge, depth) is now defined once in
  `scripts/upstream_tasks.json`, with the upstream command line and the crate request side by side.
  `scripts/upstream-baseline.py` commits upstream's per-trial outcome on the existing corpus
  (`docs/notes/upstream/`), then pairs every trial of a crate sweep against it (exact McNemar per group, plus
  unexpected decodes) and times both sides on a few files. `run-sensitivity-sweeps.sh` runs it for every task of a
  protocol it sweeps. No corpus grows and CI is unchanged. `scripts/build_jt9_upstream.sh` rebuilds the `v3.2.0-rc1`
  reference `jt9`/`wsprd` reproducibly. Eight tasks are defined: FT8, FT4, FST4, Q65, JT9, JT65, WSPR, and JTTY
  against `rjtty`. FT8, Q65, JT9 and WSPR are at or ahead of upstream and faster; JTTY agrees with `rjtty` on all 360
  trials at 0.11-0.14x its time. FT4 and FST4 were behind on accuracy (up to 1.4 dB)
  and speed (about 4–6×); both are now at or ahead on both (#553, #554). JT65's known gap is recorded and not chased. Details in
  `docs/notes/UPSTREAM_EVALUATION.md` (the evaluation sheet; `BENCHMARKS.md` leads with its verdicts and speeds).
- **Q65 decodes 3.4–11× faster: the coarse search admits candidates as `q65_ccf_22` does (#552).** Every Q65 scan
  decoded `max_candidates` = 8 candidates, even on noise alone. The search scored a bin
  `sync / (sync + noise floor)`, which is about 0.5 on noise, and a fixed floor of 0.1 admitted all of them. The
  relative test beside it (`q65_ccf_22`'s SNR ≥ 6) was ineffective too: on that ratio a steady carrier scores near
  1, so on the WSJT-X Q65-300A sample three carriers outranked the signal and nothing reached 6. The curve is now
  upstream's: the 22 sync symbols' power minus the bin's average over the whole spectrogram, so a carrier scores
  about zero. Admission is SNR ≥ 6 plus the curve's highest point, in place of upstream's best sync within
  `nfqso ± ntol`, which a plain scan here has no Rx frequency for. `SearchParams::score_threshold` is no longer
  read by the Q65 search. Timed one file at a time, single-threaded, on `jt9 -3 -d 1`'s own sequence (no-AP, then
  CQ-AP if nothing decoded): D-60 and E-120 now take 0.26–1.20× `jt9`'s decode time, from 2.8–4.5× (with #551's
  dual sync) and 1.5–2.3× (before it). Sensitivity is unchanged within 0.05 dB on every group of the 60-trial and
  release corpora, 21 trials lost against 8 gained over 15 840. There were no unexpected decodes, and tier A+B
  passes, including the real WSJT-X recordings.
- **Q65: a second coarse sync on smoothed spectra recovers up to 0.8 dB under wide Doppler spread (#551).** On a
  `q65sim` corpus with 20 Hz spread, Q65-120D/E trailed `jt9 -d 1` by 0.17–0.69 dB, the only condition measured where
  this crate was behind. The decoder was not at fault: the coarse sync lost the frame's (Δt, Δf) under spread.
  `q65_symspec` smooths each symbol spectrum `nsmo` times (D 32, E 128) before `q65_ccf_22` syncs on it, and this
  crate's sync did not. The scan now syncs twice, on the raw and on the smoothed spectra. Upstream syncs on the
  smoothed ones only, which costs narrow signals up to 0.38 dB. At 60 trials per cell it lost no trial on any corpus;
  it gained 0.40–0.79 dB on D/E-120 at 20 Hz, which now match or beat `jt9`, and up to 0.27 dB at 5 Hz. There were no
  unexpected decodes, including on 400 noise-only frames. Sub-mode A is unaffected (`nsmo = 0`). On its own it
  doubled decode time; the next entry takes that back and more. Details in `docs/notes/Q65_BENCHMARK.md`.
- **Tier-C corpora carry a provenance stamp, and the sweep runner refuses one without it.** A sweep baseline is
  only comparable with a corpus generated at `MFSK_SIM_SEED=1` (#390), and nothing checked that: a release sweep on
  2026-09-30 flagged JT65 +0.68 dB worse in unchanged code, because the local `jt65_sweep/` was a July corpus drawn
  from /dev/urandom before the seeded stub existed. The generators now write `.corpus-stamp` (seed, simulator and its
  sha256, generating commit) beside the WAVs, through `scripts/lib/corpus-stamp.sh`. They refuse a simulator not
  linked with `sim_sgran_stub.c`. The generators' default `target/ft8sim/ft8sim` and `target/ft4sim/ft4sim` on the
  machine that found this were still unseeded builds. They also refuse to add cells to a directory of unstamped or
  differently-seeded WAVs. `scripts/run-sensitivity-sweeps.sh` stops on a corpus with no stamp or the wrong seed
  (`MFSK_SWEEP_ALLOW_UNSTAMPED=1` overrides), and notes one whose generator changed since it was stamped. Every
  existing corpus was regenerated into an empty directory to adopt it: all ten, 13 220 WAVs, came out byte-identical,
  so no stored baseline moved (`docs/notes/BENCHMARKS.md`, "Generating the tier-C corpora").
- **CoreS3: `[boot-summary]` is printed on a boot with no WiFi too, when there is a console to print it to.**
  It went out on the first frame after the UDP log sink existed, so a boot that brought no WiFi (`WIFI: OFF`, or now
  `TIME: AIR DT`, #381) never printed it anywhere — not even on the USB-Serial-JTAG console a board plugged into a PC
  has from the first instruction — and the host/RTC/PMIC/I2C diagnostics it carries were lost. In peripheral mode it now
  also goes out when the boot will bring no WiFi, or 30 s in without the UDP sink (an AP that is not there); host mode
  with no WiFi still has no console to send it to. Found on the first `TIME: AIR DT` boot on a CoreS3 after #381. Verified
  on that board, both ways: `AIR DT` (NVS `grid_src=air`, WiFi driver never initialised) prints all four
  `[boot-summary]` lines on the serial console; `NTP` (WiFi up, UDP sink attempt 1) prints them once, after the sink.
  The same session is the first on-hardware confirmation of #381: `no WiFi — TIME: AIR DT (...)` and zero `wifi:` driver
  lines under AIR DT, association and NTP under NTP.

- **Fix: `alloc,fst4,fft-extern` did not build — `fst4::ddc::choose_k` called `f32::ceil`, which `no_std` lacks.**
  Every other no-`std` module carries `num_traits::Float` for exactly this; `fst4/ddc.rs` did not, and nothing built
  FST4 with `alloc` and without `std`: the pre-push feature matrix and CI's `feature-matrix` job had `alloc ft8`,
  `alloc ft4`, `alloc jt9`, `alloc jt65`, `alloc q65` and `alloc jtty` rows but no `alloc fst4`. The CoreS3 builds FST4 *with*
  `std`, so no board build showed it. The import is added and both matrix lists gain `alloc fst4 fft-extern`. Found by
  accident, while adding a trial row for another change. Swept at the same time, every other protocol as `alloc <mode>
  fft-extern`: `msk144`, `uvpacket`, `jtty` and `ft8 ft4 fst4` build; **`wspr` does not** (`wspr::osd` uses
  `std::sync::OnceLock` and `String`) and is left alone on purpose: it needs `std` by design, `embedded-shared`'s
  `wspr-bench` brings `mfsk-core/std`, and nothing asks for it without.

- **CoreS3: the WSPR receiver is removed (#313).**
  Gone from `m5stack-cores3-app`: the `wspr` and `wspr-golden` features, `apps/wspr.rs` (1 251 lines), the `wspr-bench`
  and `wifi-probe` bins (the latter needed `wspr`), the `boot_mode` dispatch arms and `MFSK_WSPR_SYNTH`. Gone from the
  shared crate `mfsk-app-shared`, because nothing else read them: `BootMode::Wspr` and its `MODE` picker row (four rows
  now, the widget centres on however many), `wsprnet.rs` (the spot uploader), `wspr_bands.rs`, the WSPR dial table of
  `CONFIG > FREQ`, and the WSPR half of `settings` / `http_config` (callsign, grid, TX power, band, wsprnet). What is
  left of the settings page is what every receiver uses — the NTP switch and server, and the log downloads — under their
  **unchanged `wspr_` NVS keys**, so an NTP server an operator set survives; the old keys stay in flash unread.
  A board that stored `boot_mode=wspr` (or was built with it in `cfg.toml`) would have fallen to `decode`, which replays a
  baked WAV; it now boots `uac`, the FT8 controller, and logs that `wspr` was removed. `m5stack-s3-app`'s `BootMode`
  matches follow. **Kept**: the library's WSPR (`mfsk-core`, `wspr::ddc` and the `wspr-ddc*` / `wspr-pass2-topn` /
  `wspr-fano-cap-fast` tuning features), `embedded-shared`'s `wspr_bench` / `wspr_scan` / `wspr_dual_core`, and the
  `m5stack-s3` bench crate that measures them. The receiver had reached `decoded 1 station(s)` on air once and never
  been run against a radio beyond that. Manual (`MANUAL_M5STACK_CORES3.md` / `.ja.md`), both `CLAUDE.md`s, `ROADMAP.md`
  and `EMBEDDED.md` / `.ja.md` say so and no longer point at the deleted files. The three app crates and `m5stack-s3`
  build for `xtensa-esp32s3-espidf` / `xtensa-esp32-espidf` (`cargo check --release`); host tests
  (`mfsk-app-shared-hosttest`) pass, 124 of them with #381's three (`wsprnet` and `wspr_bands` took 14 with them). **Not flashed.**

- **CoreS3: `TIME: AIR DT` turns WiFi off, whatever the `WIFI` row says (#381).**
  The rule is `mfsk_app_shared::wifi_policy::decide(wifi_on, air_dt)`, a truth table in its own module so the host
  compiles and tests it (`hosttest/mfsk-app-shared`, three tests); `net::bring_up`, the one place every receiver's WiFi
  decision goes through, asks it and logs why there is no radio (`TIME: AIR DT (...)` or `WIFI: OFF (CONFIG page)`, the
  grid source winning when both hold). The panel shows the running value (`effective_wifi_pref`), so it reads `WIFI: OFF`
  under AIR DT instead of claiming a radio that is not up; the stored `WIFI` choice is untouched and applies again under
  `TIME: NTP`. Why: an association campaign costs the decoder ~40 % (`fst4_sync_search` 711 → 1 395 ms per candidate,
  measured 2026-08-22) on the first slots, which are the ones an air-placed grid needs. This reverses `wifi_pref`'s own
  earlier decision (tying WiFi to the grid source was rejected so that a hilltop with a hotspot could have AIR DT and a
  log); the maintainer chose the tie on 2026-09-30 and `wifi_pref.rs` records both. The price, in the manual
  (`MANUAL_M5STACK_CORES3.md` / `.ja.md`): in FT8 (UAC) mode the USB host driver has taken USB-Serial-JTAG, so under AIR DT
  there is no console at all but the panel, and (until the WSPR receiver was removed, above) a WSPR board on AIR DT uploaded nothing. Built for `xtensa-esp32s3-espidf`
  (`cargo check --release`, default and `ft4,wspr,fst4,jtty-rx`); **not flashed**.

- **Docs: the PFB in the module maps and examples, and §6's DSP table corrected (#534).**
  `LIBRARY.md` / `.ja.md`: the `iq/` line of the module map names `PfbChannelizer` and `Channelizer::Direct | Pfb`;
  `engine/fft.rs` names `with_planner`; the `IqReceiver` example shows `with_channelizer`. `BINDINGS.md` / `.ja.md`: the C
  example shows `mfsk_iq_open_with`. §6's DSP table listed a module `fir` that does not exist; it now lists what does:
  `fir_decimate` (`FirStage`, `design_lowpass`, `kaiser_order`, `design_lowpass_kaiser`, `from_taps`) and `polyphase`
  (`PolyphaseResampler`, `from_prototype`), each with its `.ja.md` twin row.

- **IQ: a polyphase filter bank, `iq::PfbChannelizer`, as an option beside the `Direct` path (#534).**
  `IqReceiver::with_channelizer(stream, Channelizer::Pfb)` (C: `mfsk_iq_open_with(.., MFSK_IQ_CHANNELIZER_PFB, ..)`) shares
  one bank among all channels instead of mixing and decimating each from the input rate: 2x oversampled (hop M/2),
  sub-bands about 24 kHz apart at 48 kS/s (M = 32 at 768 kS/s, 100 at 2.4 MS/s), a Kaiser prototype of M(K−1)+1 taps
  (K = 13) so its delay is a whole number of hops, and each channel's back end an `IqToAudio` on the sub-band nearest its
  window. `Direct` (`IqReceiver::new`, `mfsk_iq_open`) stays the default and is unchanged: an amateur band's handful of
  channels is cheaper there. Measured, % of a core, one thread: 768 kS/s 1 / 4 / 8 / 32 / 128 channels 2.66 / 3.34 /
  4.32 / 10.22 / 34.94 on the bank against 0.92 / 3.68 / 7.40 / 29.89 direct; 2.4 MS/s 9.08 / 9.78 / 10.77 / 16.64 /
  41.15 against 2.38 / 9.51 / 19.13 / 76.51 — break-even about four channels. Selectivity on the bank −124.3…−124.4 dB at
  192 k, 250 k, 768 k, 2.048 M and 2.4 MS/s (window across a whole sub-band, ~1 000 interferer positions each). Every IQ
  decode test now runs through both paths with the same result, and the FFI test and C++ smoke driver drive both.
  `engine::fft::with_planner` (moved from `jtty::dsp`, which re-exports it) is how the bank plans its IFFT per push:
  holding a `Box<dyn Fft>` made `IqReceiver` `!Send`, and a test now pins `Send` on the receiver and both channelizers.
  `IqToAudio::new` reports a rate below 12 kHz before a placement error, as before the refactor. `mfsk.h` gains
  `mfsk_iq_open_with` and `MFSK_IQ_CHANNELIZER_DIRECT` / `_PFB` (105 exported functions). Design and measurements:
  `docs/notes/IQ_CHANNELIZER.md` §7b; `LIBRARY.md` §2.7 and `BINDINGS.md` §2.8.2 with their `.ja.md` twins.

- **IQ front end: 120 dB of selectivity, all filters Kaiser designs; the sharp filter moved to 12 kHz (#534).**
  The `Direct` path (`IqToAudio`, and `IqReceiver` on it) used Blackman windows, which stop at about 74 dB: measured, an
  interferer at the channel's window edge came through at −73.5 dB. Every filter is now a Kaiser design for
  `iq::REJECT_DB` = 120 dB, the noise floor of an ideal 16-bit ADC in 2500 Hz at 768 kS/s (−108 for 14 bits, −96 for 12,
  so 100 dB would let a full-scale interferer leak above a 14- or 16-bit SDR's floor), designed at 123 dB because
  Kaiser's order estimate falls short at the stop edge for short filters (at exactly 120 a 71-tap stage let an alias
  through at −117.9 dB). The resampler now comes before the sharp filter: it reaches 12 kHz *complex* and only has to stop
  at 8.8 kHz, and the 400 Hz-transition filter runs at 12 kHz (243 taps) instead of 24. New in `engine::dsp`:
  `kaiser_order`, `design_lowpass_kaiser` (series `I0`, `no_std`), `FirStage::from_taps`,
  `PolyphaseResampler::from_prototype`; the existing constructors are unchanged and now go through them.
  Measured, one thread, eight channels, worst over ~550 interferer positions per rate plus ~600 aimed at every stage's
  alias edges: selectivity −73.5 → −124.0 dB at 192 kS/s, → −121.0 at 768 k, → −122.0 at 2.4 M; cost per channel
  0.56 → 0.36 %, 1.04 → 0.93 %, 2.33 → 2.42 % of a core (at 2.4 MS/s the input-rate first stage is two-thirds of it).
  `tests/iq_front_end.rs` asserts ≤ −119 dB over 160 positions at 768 kS/s; every IQ decode test is unchanged.
  `docs/notes/IQ_CHANNELIZER.md` is the study this came out of: the design of a polyphase filter bank for many
  channels (not built), and the overlap-save channelizer tried first and dropped (−71 dB between FFT bins).

- **Docs for the IQ input: `LIBRARY.md` §2.7 and `BINDINGS.md` §2.8.2, with their `.ja.md` twins (#534, phase 5).**
  §2.7 covers `IqToAudio` and `IqReceiver`: the signal path, the audio-200 Hz floor and why (the real part folds the
  sideband below the dial onto the wanted one), placement errors, the time model and what drops an open slot, modes, the
  formats, the threading contract, the measured cost per channel and the evidence tests. §2.8.2 is the C face,
  `mfsk_iq_*`, with the loop a host writes and the same threading and time notes. The symbol index gains an IQ group and
  the exported-function count goes from 93 to 104 (counted with `nm -D` on `libmfsk.so`). The Rust examples are
  doctests (`LIBRARY.md` is included as one), so they compile and run in the docs job.

- **C ABI for the IQ receiver: `mfsk_iq_*` (#534, phase 3).**
  A handle of its own, as JTTY has: `mfsk_iq_open(sample_rate, center_hz, format, iq_swap, &status)` (`format` one of
  `MFSK_IQ_FORMAT_CF32/CS16/CS8/CU8/CS24`), `mfsk_iq_add_channel(rx, dial_hz, mode, &channel)` for FT8, FT4, the five
  FST4 periods, WSPR, JT9, JT65 and the ten Q65 sub-modes (a mode the receiver does not carry, or a dial that cannot be
  placed, is `INVALID_ARG`), `mfsk_iq_remove_channel`, `mfsk_iq_set_time_anchor(rx, utc_ns_at_sample_0)`,
  `mfsk_iq_retune`, `mfsk_iq_gap`, `mfsk_iq_push(rx, bytes, n)`, `mfsk_iq_samples_in`, `mfsk_iq_pending`, `mfsk_iq_close`.
  Decodes are collected by `mfsk_iq_poll(rx, &row)` into a size-versioned `MfskIqDecode` (channel, `MfskMode`, text, audio
  and absolute frequency, DT, SNR, the IQ sample index and UTC of the slot start, `has_utc`), returning 1 / 0 / negative
  as `mfsk_jtty_poll` does; a queue of 4096 drops its oldest. Decoding runs inside `mfsk_iq_push`, so a host that cannot
  block pushes from a worker thread; the handle is one-thread-at-a-time. Polling rather than a callback is the JTTY
  reason: no user-data contract to cross the boundary, and the Kotlin / Swift / C# wrappers (#533) are simpler.
  Evidence: `mfsk-ffi/tests/iq_ffi.rs` decodes a synthesised `CQ JA1ABC PM95` (FT8, audio 1500 Hz, in noise) placed as IQ at
  48 kS/s through each of the five formats, with the row's channel, mode, absolute frequency, UTC and slot sample checked,
  a free-running grid reporting `has_utc == 0`, a `retune` and a `gap` in the middle of the slot costing that slot, and
  the refusals (unknown format, rate under 12 kHz, non-finite centre, DC in the band, band edge, a mode the receiver does
  not carry, a retune that would put a channel out, NULL handles) as statuses; the C++ smoke driver does the same
  end-to-end through `mfsk.h`, which `header_compile.sh` compiles as C11 and C++17. `mfsk.h` is regenerated (+207 lines).

- **IQ sample formats `Cu8`, `Cs8` and `Cs24` (#534, phase 4).**
  `IqSampleFormat` gains `Cu8` (RTL-SDR, 128 = zero), `Cs8` (HackRF) and `Cs24` (24-bit IQ WAV), byte streams only through
  `push_bytes` on both `IqToAudio` and `IqReceiver`, which now share one converter (`IqSampleFormat::convert`); a sample
  split across calls is carried over as before. Full scales: 128, 128 and 8 388 608. Evidence: a unit test round-trips
  the five formats to their own precision; the byte path through the front end matches `Cf32` (RMS error over signal
  under 3 % for the 8-bit forms, under 0.01 % for `Cs24`); and `tests/iq_receiver.rs` quantises `qso3_busy.wav` as IQ at
  48 kS/s to each of the three with the peak at 0.7 of full scale, and the receiver decodes the WAV path's 14/14 with
  0 extra from each.

- **`IqReceiver` decodes WSPR, JT9, JT65 and every Q65 sub-mode too (#534, phase 2 completed).**
  `IqMode` gains `Wspr`, `Jt9`, `Jt65` and `Q65A15` … `Q65A300` (ten sub-modes), each through its own request type with
  its `default_search_params` and the nominal start the registry gives the mode, so `dt` reads as it does on the WAV path;
  FT8, FT4 and FST4 are unchanged. The module now builds with any one protocol feature. Slots are handed to the decoders
  as `f32` at the level the `i16` decoders take divided by 32768; the FT8 family converts back to `i16` itself.
  Evidence, `tests/iq_receiver_modes.rs`: each recording placed as IQ at 48 kS/s on a UTC grid anchored on a 600 s
  boundary (a multiple of every period from 7.5 s to 300 s), decoded through the receiver against the same request on
  the WAV: WSPR `150426_0918.wav` 9/9, JT9 `130418_1742.wav` 5/5, JT65 golden, Q65-120D rain-scatter golden and
  Q65-300A optical-scatter golden all identical, 0 extra, frequency within 2 Hz and DT within 0.1 s, `abs_freq_hz` =
  dial + audio. The other Q65 recordings vendored here only decode by averaging several periods and the receiver does
  not average, so they are not used; a unit test pins that every `IqMode` variant is a registry entry with the slot its
  period says. Two tests were tightened while doing this: a set of fewer than ten may lose no decode (the "one weak-edge
  decode may tip" allowance had let a Q65 recording lose its only message), and the first anchor used in the new test
  was a multiple of 300 s, not 120 s, which put WSPR and Q65-120 slots off their boundary and decoded nothing.

- **`iq::IqReceiver`: N channels of one IQ stream, slots cut on UTC from the sample count, absolute frequency in the row (#534, phase 2).**
  `add_channel(dial_hz, IqMode)` (FT8, FT4 and the five FST4 sub-modes, the ones sharing `DecodeRequest`; WSPR, JT9/JT65
  and Q65 have their own request types and follow), `set_time_anchor(utc_ns_at_sample_0)`, `on_decode(cb)`, then
  `push_cf32` / `push_cs16` / `push_bytes`. Slot `j` of a mode with period `T` covers UTC `[j·T, (j+1)·T)`, computed in
  integers from the sample count (each channel's audio index `k` is `k/12000` s after sample 0; the front end drops its own
  group delay so index 0 is IQ sample 0); with no anchor the grid free-runs from sample 0, right for a recording. A slot
  is decoded, with the registry's default search for its mode, once all of it has arrived; the partial slot the stream
  opened in the middle of is not. `retune(center_hz)` (all-or-nothing: `Err` and nothing changes if a channel no longer
  fits), `gap(lost_samples)` and re-anchoring drop the open slots and keep the sample clock going. Each row is the
  cross-mode `Decoded` plus `abs_freq_hz` (dial + audio frequency), the IQ sample index and the UTC of the slot start.
  Each slot is scaled to a fixed RMS before the `i16` the decoders take, and decoding runs inside `push`, so a caller that
  cannot block pushes from a worker thread. The dial is the caller's to choose: the decoders search the channel's audio
  200-3000 Hz themselves, and nothing here looks for signals.
  Evidence, `tests/iq_receiver.rs`: `qso3_busy.wav` (FT8) and the FT4 golden in **one** 192 kS/s stream on two dials, blocks
  of odd sizes, UTC-anchored: 14/14 FT8 and 11/11 FT4 messages of the WAV path, 0 extra, frequency within 2 Hz and DT
  within 0.05 s of it, `abs_freq_hz = dial + audio` exactly. The same recording after 3 s of nothing with the grid
  anchored 3 s early decodes on the next boundary; a `retune` or a `gap` in the middle of the first of two back-to-back
  slots costs that slot only and the second decodes as before; byte pushes split inside samples match typed ones. One
  decode in those runs is `K1BZM DK8NE -10`, the -17 dB entry of `common::ft8_qso3`'s known-real list that the WAV path's
  default search misses, so an extra that is in that list is allowed rather than counted as a phantom.
  Cost, one thread, release, `Cf32` in, per `IqToAudio` (so per channel; each mixes at the input rate): 768 kS/s one
  channel 1.1 % of a core (89x real time), 8 channels 11 %, 32 channels 44 %; 2.4 MS/s one channel 2.3 %, 8 channels
  18 %; 192 kS/s one channel 0.5 %. Linear in channels, which is the point where the FFT channelizer of #534's phase 4
  starts to earn its keep.

- **New `mfsk_core::iq`: one channel of a wideband IQ stream as 12 kHz USB audio (#534, phase 1).**
  `IqToAudio::new(IqStream { sample_rate, center_hz, format, iq_swap }, dial_hz)` then `push_cf32` / `push_cs16` /
  `push_bytes` (a sample split across calls is carried over) appends the audio a transceiver's USB output would have
  carried for that dial frequency, ready for any `DecodeRequest` after scaling to `i16`. Any integer rate from 12 kHz
  up: a cascade of short `FirStage`s down to 24-48 kS/s, one sharp filter there (2.8 kHz pass, 3.2 kHz stop around audio
  3 kHz, so the sideband below the dial that `Re()` would fold onto the wanted one is rejected from audio -200 Hz down
  and usable audio starts near 200 Hz), then a `PolyphaseResampler` `L/M`; rates that need `L > 2048` are refused
  (`UnsupportedRate`), as are a channel whose 0-6 kHz window is outside `±Fs/2` or that contains DC. The sample count is
  the clock (`samples_in()`); no time source is read. Formats `Cf32` and `Cs16`; `CU8` / `CS8` / `CS24`, the
  multi-channel `IqReceiver`, UTC slotting and the C ABI are the later phases of #534. Evidence: `tests/iq_front_end.rs`
  places `qso3_busy.wav` as double-sideband IQ (so the lower sideband must be rejected) at 48 k, 192 k, 250 k
  (`Cs16`), 768 k (channel 150 kHz below DC) and 2.4 M (I/Q swapped) with blocks of mixed sizes, and decodes it with the
  WAV path's request: 16/16 messages at every point, 0 extra, frequency within 1 Hz and DT within 0.05 s of the WAV
  decode. Unit tests pin tone frequency and amplitude across 48 k-2.4 M rates (including 2.048 M), the LSB rejection,
  `iq_swap`, the byte form against `Cf32`, and the placement errors.

- **Q65: the grid decode conditions the symbol spectra the way `q65_loops` does — Q65-120D now decodes to +2.0 s late like `jt9` (#521).**
  `q65_loops.f90:64-68` runs `spec64`'s passband equalisation (45th-percentile baseline per bin, smoothed), divides by
  `pctile(s3, 40)`, clips at `s3lim = 20` and zaps birdies (`q65_bzap`) before any `q65_dec2`; this crate fed raw `|FFT|²`
  to the fast-fading metric. With the window off the symbol boundary the neighbouring symbol's tone is the strong one, and
  unclipped it out-weighs the true tone without bound; clipped, both sit at 20 against a noise floor of 1 and the code
  sorts them out. #521's own analysis had ruled out the sync window and the `Δt` retry, correctly; this is the step it
  left unnamed. Six noise seeds per Δt at −12 dB (2500 Hz), the same synthetic frames through real `jt9 -3 -p 120 -b D
  -d 1` and this crate: the last Δt with every seed decoded is +2.0 for `jt9` (4/6 at +2.05, 0/6 at +2.15), +1.75 for
  this crate before and +1.95 after (3/6 at +2.05, 0/6 at +2.15). The conditioning is skipped when there is no noise
  floor (`pctile40 / pctile10 > 30`): a noise-free float synthesis leaves `pctile 40` at leakage level and the clip
  would flatten every tone (`dt_window`'s truncated-frame test caught it). That ratio is 4.5-8.2 on the 48 golden
  cells and 512 sweep cells measured, 4.9-5.9 on a +27 dB signal in noise, 67 and up on noise-free frames. Sensitivity
  is unchanged: `q65_sim_sweep` / `q65_snr_sweep` / `q65_ap_sweep` cross within 0.33 dB of the baseline on every group
  (the largest, `e120/cq`, is 55 trials), tier A+B green. `dt_window`'s Q65-120D reference moves from +1.5 to +2.0 s.

- **Q65: `MultiPeriodRequest` gets WSJT-X's `iavg=1` q3 decode — `.ap_list()` + `.rx_freq()` on the averaged spectra (#520).**
  Upstream's second pass (`q65_decode.f90:263-272`) runs the 85-symbol sync of every list message (`q65_ccf_85`) and the
  list decode (`q65_dec_q3`) on the running average `s1a` of the periods' symbol spectra (`u = 1/min(navg, 4)`), once
  two periods are in. `MultiPeriodRequest` had `.ap_list()` but no Rx frequency, so it always used the crate's own
  template match. `.rx_freq(hz)` / `.ftol(hz)` now turn the q3 on, from the second slot, ahead of the fading/plain
  ladder; one result a slot as before, so a q3 hit skips that slot's ladder (upstream goes on to its candidate loop).
  On the four-recording `30A_Ionoscatter_6m` golden, real `jt9 -3 -p 30 -b A -d 17 -c K1JT -x K9AN` decodes `K1JT K9AN
  R-16` at 1010 Hz, DT 0.3, −19 dB from the fourth file; this crate's q3 gives 1010.0 Hz, DT 0.32, `iterations=0`, −21.1 dB
  (the SNR comes from the averaged spectra, not `q65_snr`'s per-symbol alignment, hence 2 dB). The issue's oracle run had
  already shown the crate's fading/plain ladder recovering the same message without a list, so this is parity of
  mechanism, not a measured recall gain; no synthetic multi-period corpus exists to sweep it. Also corrects
  `tests/q65_wsjtx_samples.rs`, which claimed `jt9` cannot be driven through averaging — `-d 17` does it.

- **embedded: `jtty-bench`'s stack-placement run now checks that it got the configuration it asked for (#499, #528).**
  #516's "Back's stack in PSRAM" results were void: `bench_stack_place()` set `stack_alloc_caps` to
  `MallocCap::Spiram` alone, which `esp_pthread_set_cfg` refuses without `MALLOC_CAP_8BIT`
  (`components/pthread/pthread.c:159`, ESP-IDF v5.5.3), and the bench dropped the refusal (`let _ = cfg.set();`), so
  each case spawned on the previous default — "Back's stack in PSRAM" measured internal against internal, and "Front's
  stack in PSRAM" an unpinned default-priority Front. All ten `ThreadSpawnConfiguration::set()` calls in the bench now
  go through `apply_spawn_config`, which panics on a refusal instead of measuring something else; the PSRAM caps carry
  `Cap8bit`; and each thread logs where its stack actually is (`stack_place()`, a local's address against
  `SOC_EXTRAM_DATA_LOW`/`HIGH`), so the configuration under test is read off the log. The first run after the fix
  (`logs/jtty_bench_stackplace_fixed_2026-09-28.log`) put every stack where it was asked, but the two cases needing an
  internal 32 KB stack for Back could not spawn it (31 KB was the largest free internal block at that point of the
  bench), so Back now runs on a 20 KB stack — it peaks at 13 020 B in the E1 receiver, the same code. Of the cases that
  ran, only Back-in-PSRAM with Front internal was measured (Back 155 ms mean, 660 worst; Front 297 ms a window) — alone,
  not a comparison. **Not yet re-run on the board with the 20 KB stack, so #516's placement question is still open.**
  Bench-only: no library or receiver code changes.

- **JTTY: the trellis survivors take 12 bytes an entry, not 16 (#499).** `Surv` is `repr(C, packed(4))`: with an `f32`
  metric the default layout spent 4 of its 16 bytes on padding before the 8-aligned key, so `TrellisScratch`'s two arrays
  go from 32 to 24 KB each — 16 KB of internal DRAM on the CoreS3, where the JTTY receiver had run it down to 0–6 KB
  free (`docs/notes/JTTY_CORES3_APP.md` §14–§15). The key stays 4-aligned, which is all a 64-bit load needs on Xtensa,
  and fields are only ever read by value. Decodes do not move by a bit: `tests/jtty_ladder.rs`'s new
  `list_decodes_are_pinned` folds every `f64`, `f32` and reused-scratch list for 360 frames into a fingerprint taken
  from the unpacked layout, and the upstream ladder cases and `largest_first_build_decodes_bit_identically` pass
  unchanged. Not slower on the board either: JTTY's Back went from 203–247 to 184–228 ms a window on the golden
  recording.

- **JTTY: `Receiver::new_with_f32_metrics()`, the same receiver built largest allocation first (#499).**
  Decodes bit-identically to `Receiver::new().with_f32_metrics()` (`tests/jtty_rx.rs`, the streamed updates and the
  scan on WSJT-X's recording and on `testsig`'s six-station band); only the order of its allocations differs. On the
  ESP32-S3, where a receiver is built while allocations prefer internal DRAM and each one takes the first heap region
  with room, that order decides placement: the CoreS3 app has one internal region that can hold a 32 KB block, and the
  old order filled it with the smaller tables before the two 32 KB trellis survivor arrays, which landed in PSRAM (a
  rung 410 ms against 190). Built survivors first, then the transform buffer and the sync wave (made at its exact
  length, `dsp::sync_wave_exact`, so it never needs a second block while it grows), the four fit, and on the board Back
  went from 327–436 to 203–247 ms a window on the golden recording. `new()` and `with_f32_metrics()` are unchanged.

- **CoreS3: JTTY as a fifth receive mode, behind `--features jtty-rx` (#499, E1 of `docs/notes/JTTY_CORES3_APP.md`).**
  Picked from the touch panel like FT8/FT4/FST4/WSPR. JTTY has no slot, so the sample count is its clock: the audio sink
  stages samples with positioned gap markers for overflow, a clock reconciliation (`mfsk_app_shared::jtty_rx_clock`,
  hosttested) inserts zeros for a deficit above 20 ms against `esp_timer` and resets the stream above 1 s, and every
  reset starts a new generation of `Front`/`Back`. Front runs on core 1 (prio 4, 8 KB internal stack), Back on core 0
  (prio 5, 20 KB internal stack reserved from `worker_arena` at boot), queue 6, the panel above Back. **JTTY mode runs
  without WiFi**: beside the WiFi driver internal DRAM ran to 1–7 KB free and the receiver dropped dozens of windows
  (design note §12), so there is no UDP log, NTP or config page in this mode, and the ALL.TXT anchor comes from the
  BM8563 RTC (re-taken when the clock is set after the stream starts). A message is published once, on `complete`;
  ALL.TXT stamps it from that anchor plus its `start_s` and is flushed when no message is open (or after 10 min). The
  slot period is split three ways (`BootMode::fresh_row_ms` / `slot_rules_ms`), so nothing sees a period of 0. On the
  SIM feed it decodes the golden recording at the host's start times, but it is still slower than `jtty-bench`: both
  trellis survivor buffers land in PSRAM because `Receiver` allocates them after its smaller buffers have filled the
  one internal region that could hold them, and Front runs 255 ms a window while Back is idle against 430–462 ms while
  it decodes (§13). #516's PSRAM-stack bench never ran with a PSRAM stack (§12). FT8/FT4/FST4/WSPR are unchanged
  (FT8 SIM A/B against `main`: 60 decodes each over 9 slots and identical panel timing; re-run after the
  `worker_arena` change, 7 a slot on every slot after the one a boot-time NTP re-alignment cut).

- **docs: pin `lib/*.f90`/`lib/*.c` citations to the `v3.2.0-rc1` tag of `WSJTX/wsjtx`, and fix 7 that had drifted (#467).**
  `CONTRIBUTING.md` now states the reference tree explicitly — line numbers move between trees on any file that sees
  an edit, and nothing previously recorded which tree a citation's line number was checked against. Auditing this
  crate's own FT8/FT4/FST4 citations against the pinned tag found 7 real drifts, 2 introduced this session (#519,
  #465, both caught before merge would have been too late) and 5 pre-existing (`ft8b.f90:96→97`, `:154-161→155-162`,
  `:163-176→164-177`, `ft8_decode.f90:162→158`, `genft4.f90:64→67`) — comment-only, no behavior change.

- **FST4: the AP rung's OSD masks locked bits and runs on the AP-held BP sums, as `osd240_101.f90`/`decode240_101.f90` do (#465).**
  `osd_decode_npre_generic` takes an `ap_mask`: a test pattern that would flip a locked bit is skipped in both the
  `npre1` and `npre2` passes (`osd240_101.f90:192`/`:263`), mirroring `osd_decode_npre1_masked`'s FT8/FT4-pinned
  counterpart (#459). `Ldpc240_101`'s AP rung now also runs OSD on `bp_llr_zsum_ap_with_scratch`'s locked-bit-held sum
  after 1 and 2 BP iterations instead of the raw channel LLR — previously skipped entirely under an AP mask
  (`ap_slice.is_some()`), on the mistaken belief FST4 had no AP wiring yet (issue #143 is about SIC/list-decode, not
  this). Noise false-accept measured (300 000 draws, CQ-style lock): 32 after against 10 before, in 300 000 — not the
  order-of-magnitude improvement FT8/FT4 saw, because FST4's "before" was already the pruned `npre1` search rather
  than FT8's old combinatorial `osd_decode_deep` bug; both rates stay low in absolute terms
  (`tests/fst4_ap_osd_false_accept.rs`).
- **FT8: `DecodeRequest::eme_delay`, WSJT-X's EME-delay display shift (#519).**
  Ported from `ft8_decode.f90:234` (`if(emedelay.ne.0) xdt=xdt+2.0`): with it on, every delivered decode's `dt_sec` is
  offset by +2.0 s — a report-layer convention for the ~2.5 s Earth-Moon-Earth round trip, not a search change; off by
  default, as in WSJT-X's GUI. Applied at both of FT8's independent result-delivery points (the single-pass `accept`
  closure and the shared `.sic_rounds()`/`.sic_early()` engine), after the xsnr2 gate and after any waveform
  subtraction that reads the unshifted `dt_sec` — matching upstream's placement exactly. Unlike Q65's `.eme_delay()`,
  which widens the search window, this is display-only: FT8's coarse sync already covers a full slot regardless of the
  flag, and upstream applies no search change for FT8 either. Split from #501's audit of leftover WSJT-X 3.2.0-rc1
  fidelity gaps; FST4's `emedelay` and the other three items split into #520/#521/#522.

- **JTTY: `Receiver::new_f32_metrics`, the trellis survivors allocated first (#499).** Where a buffer lands is decided when
  it is allocated. Built as `new().with_f32_metrics()`, the two 32 KB survivors come last, and on the CoreS3 one of them
  found no internal block even on a fresh heap; a second receiver in the same process got neither. That was the rebuild
  slowdown. A ladder call costs 235 ms with both internal against 556 ms in PSRAM. `new_f32_metrics` decodes exactly as
  the builder does. The CoreS3 placement measurements are in `docs/notes/JTTY_CORES3_APP.md` §3.
- **JTTY: a receiver that drops windows instead of stalling, and its own ladder call for the band scan (#499).**
  `Front::push_or_drop` asks for room before each window and, when the decoder has fallen behind, drops the window
  unprepared; `Back` now takes the gap in window numbers (`Front::dropped`, `Back::skipped`), so the queue between the
  cores and the delay stay bounded instead of the audio input stalling. Only the frames that start in a dropped window are
  lost. `Params::side_ladder_budget` gives the side channels ladder calls of their own; `Params::embedded()` gives them
  one, which reads 38 of 60 messages with six stations sending at once against 20 when they shared channel 0's.
  `jtty::testsig::pileups` adds pileup and busy-band patterns, and `Params::retro_sweep` / `subtract_side_channels`
  make upstream's re-sweep and side-channel subtraction optional. On the CoreS3 with six stations, a queue of 3 windows
  dropped 7 in 10 trials (33 of 60 messages, delay at most 2.65 s), 5 dropped 2 (38 of 60, as the host), and 6 none (delay at most 3.11 s). Building a
  second `Receiver` in the same process put about 32 KB of it in PSRAM and slowed busy bands by half: build it once.
  Measurements in `docs/notes/JTTY_EMBEDDED_BUDGET.md` §14.
- **JTTY: a band scan beside channel 0, `Params::side_channels` (#499).** `SideChannels::Upstream`
  (the default: rjtty's 1350/1650 Hz ± 150 Hz) or `SideChannels::Band { lo_hz, hi_hz, width_hz,
  picks }`, upstream's earlier scan of the whole band in 200 Hz channels, each taking unrefined
  candidates through the side channels' stricter gate. With a decimated surface the side channels
  get their own, 8 ms by 1.46 Hz and kept compactly. `Params::embedded()` scans 200–2800 Hz with
  one candidate a channel: on the host single stations away from channel 0 are found 43, 55, 55
  and 54 times of 56 at −14/−10/−6/−2 dB, and on the CoreS3 the two-core receiver keeps up (front
  310 ms, back 205 ms against 472). Channel 0's decodes are unchanged.
- **JTTY: `Params::skip_decoded_hz` and a test-signal generator (#499).** With it set (20 Hz in
  `Params::embedded()`) the sync search does not look within that distance of a frame decoded in
  an earlier window, over that frame's own span, where it found partial matches of the frame's
  tail that the ladder then rejected: a quarter fewer ladder calls on a busy band and no frame
  lost on any corpus. The budgeted candidate path no longer refines a candidate twice. The
  hidden `jtty::testsig` module makes JTTY recordings from a seed (noise, offset, drift, Rayleigh
  fading, several stations); the CoreS3 bench and `tests/jtty_board_patterns.rs` run the same
  catalogue, and the board decoded all 210 cases as the host does.
- **JTTY: an embedded receiver that runs in real time on the CoreS3 (#499).** `Params::embedded()`
  now also sets `fir_analytic` (the analytic signal by a 97-tap complex FIR, computed once per
  sample as it arrives), `coarse_sync_grid` (sync on a 4 ms grid), `ladder_budget: Some(1)` (at
  most one ladder call a window, candidates ranked by the gate) and `ladder_rungs: Rungs::L1_L4`,
  and no subtraction; `rx::Front` and `rx::Back` split a `Stream` in two for two cores and together
  report exactly what it does. With those, on the CoreS3 (front on core 1, back on core 0, audio
  at its real rate) a window's front end takes 138 ms and its back end 178 ms on average against
  472 ms, and the sample recording's message is decoded 0.3 s after each window on average.
  Against `rjtty` on sjtty corpora it reads 163 of 360 frames at 1500 Hz (160), fewer under fast
  fading (ITU LD 67 against 78 of 120; a budget of two with the refined retry and
  `Rungs::FULL_SYMBOL` read 75), and no stations outside channel 0. Exact speed-ups on every
  configuration: the list trellis merges its predecessors' sorted lists instead of inserting
  each extension (output identical on 5 400 random decodes), its tables are shared per block
  length (380 to 22 KB), and with `f32` metrics its survivor arrays are made once. `jtty-stats`
  builds on targets without 64-bit atomics. `docs/notes/JTTY_EMBEDDED_BUDGET.md` sections 9 to 11
  have the measurements.
- **JTTY: three search options for a receiver that cannot afford the default (#499).**
  `jtty::rx::Params::ch0_only`, `decimate_sync` and `raw_first`, off by default so the receiver
  stays upstream's, and `Params::embedded()` sets the three together.
  - `ch0_only` skips the fixed side channels at 1350 and 1650 Hz; `decimate_sync` builds the sync
    surface with 512-point transforms instead of 8 192 (same 0.732 Hz bins; the product of window
    and sync wave is summed 16 samples at a time after mixing the band centre to DC) and falls back
    to the full one when the band exceeds 375 Hz; `raw_first` tries each channel-0 candidate
    unrefined and refines it only if that failed with 6 or more of 13 sync tones seen.
  - Measured on `jtty_sweep` (360 files, weak-signal passes): default 160, `embedded()` **164**,
    channel 0 alone with the default refine-first order 141; unexpected decodes 0 in each. The
    decimated surface made the same decision as the full one on every file; on the hard
    two-station set the weak station is found in 92 of 200 (77 without `raw_first`). In noise alone
    a window refines 1.8 candidates instead of 5.0 and the host's surface time falls 4x; ladder calls
    rise 34 % (0.073 to 0.098 a window). `docs/notes/JTTY_EMBEDDED_BUDGET.md` section 9 has the tables.

- **C ABI: WSJT-X 3.2's Q65 settings, `mfsk_q65_decode_ex` (#466, part 2).** Pileup, Max
  Drift, the EME delay and the q3 list decode were Rust-only. The four `mfsk_q65_decode*`
  functions pick a strategy by name and scan a fixed wide window, and these settings are
  combinations of them, so there is one call and a size-versioned `MfskQ65Params` instead of
  one more positional function each.
  - `mfsk_q65_params_init(mode, &p)` writes the library's defaults (±1 s, threshold 0.1, 8
    candidates) and `nominal_start_s`, the mode's `tx_start_offset_s`. Rows report `dt_sec`
    from that nominal start, as WSJT-X's DT column; the older functions report
    `start_sample / 12000`.
  - Settings: `pileup`, `eme_delay`, `max_drift` (0..=50), `rx_freq_hz` / `ftol_hz`, `ap_list`
    (0 none, 1 the standard QSO list, 2 the contest list), `fading_b90_ts` / `fading_model`,
    and the AP hint. With `rx_freq_hz` and `ap_list` it is the q3 decode.
  - Flags are `uint32_t` and absent floats are NaN, so no field can hold an invalid Rust `bool`
    or enum. A combination the engine would quietly not honour is refused with the reason:
    `ap_list` with fading, `rx_freq_hz` without a list, `pileup` without an AP hint,
    `max_drift` with fading or with a list decode that has no Rx frequency.
  - Rows carry `MFSK_DECODE_FLAG_COPIED_LAST_TX` (`flags` bit 1) on a Pileup reply, on the
    older Q65 calls too. `mfsk_encode_q65_flagged` sends one.
  - `MfskQ65History` (`q65_hist`: `_push`, `_record`, `_lookup`) and `MfskQ65Callers`
    (`q65_hist2`: `_record`, `_expire`, `_remove`, `_len`, `_get`) are handles the caller
    owns; times are the caller's Unix seconds.
  - Each setting is checked by a decode that only comes out right if it arrived (a 3 s late
    frame with the EME delay, a 60 Hz/min drifting one with Max Drift, a flagged reply under
    Pileup, a list message in a window holding nothing else with q3, a remembered caller with
    the contest list), and disabling the wiring fails five of them.
  - Kotlin: `Mfsk.decodeQ65(mode, samples, params, callers)` with `MfskQ65Params` (start from
    `Mfsk.q65DefaultParams(mode)`), `MfskQ65Fading`, `MfskQ65List.Standard` / `.Contest`,
    `MfskQ65ApHint`, the `AutoCloseable` handles `MfskQ65History` and `MfskQ65Callers`,
    `Mfsk.synthesizeQ65(..., copiedLastTx)` and `MfskDecode.copiedLastTx`. Kotlin could not
    decode Q65 at all before: it has no decode handle. 37 new JVM checks; swapping two slots in
    the shim fails five.
  - Swift: `Q65.decode(_:mode:params:callers:sampleRate:hashTable:)` with `Q65.Params`
    (`try Q65.Params(mode:)`), `Q65.Fading`, `Q65.List`, the classes `Q65History` and
    `Q65Callers`, `Q65.encode(..., copiedLastTx:)` and `Decode.copiedLastTx`, tested by
    `Q65ExtendedTests`. Written without a Swift toolchain on hand, so not yet built or run:
    `bindings/swift/scripts/test.sh` on a Mac.

- **C ABI: the transmit frequency and FST4's noise blanker, on `MfskDecodeParams` (#466,
  part 1).** Two knobs the Rust builder gained for the 3.2 port had no way in through
  `mfsk-ffi`, so a Kotlin or Swift caller could not reach them.
  - `tx_freq_hz` is WSJT-X's `nftx`: FT8 also tries the AP hypothesis that locks both
    callsigns within 50 Hz of it, not only of `freq_hint_hz`. `NaN` is unset, as for the
    hint. On a weak `K1JT HA0DU -12` at -22 dB with the QSO frequency 300 Hz off, 12
    fixed-seed slots decode 3 without it and 8 with it, and nothing else
    (`tx_freq_brings_the_two_callsign_ap_hypothesis_into_range`).
  - `nb_percent` and `nb_sweep_step` / `nb_ftol_hz` are the **NB** setting, every FST4
    sub-mode: a fixed level (`0..=25`), or a sweep over `0, step, .. 20` percent, whose
    levels above 0 need `freq_hint_hz`. The Rust `fst4_noise_blanker` slot, driven through
    the ABI, goes from buried under clicks to decoded at NB 2 %.
  - Two new capability bits say which mode has which: `MFSK_CAP_NOISE_BLANKER` (16, FST4
    only) and `MFSK_CAP_TX_FREQ` (17, FT8 only), mirrored in `registry::caps` and pinned
    in `tests/registry_caps.rs`. Bit 15 stays `MFSK_CAP_STREAM_RECEIVER`, which only
    `mfsk-ffi` defines.
  - As everywhere in this ABI, a knob the mode does not have is refused at
    `mfsk_session_open`, not dropped: `tx_freq_hz` on FT4 or FST4, a blanker on FT8 or
    FT4. Numbers the engine would quietly reinterpret are refused too — `nb_percent` past
    25 (the engine clamps it) and a `nb_sweep_step` other than 1, 2 or 5 (it reads the
    rest as 5).
  - The struct grew by appending after `search_hz`; the size-versioned contract means an
    older caller's shorter `size` leaves the new fields unset (`mfsk_abi_version` stays 2).
  - Swift: `DecodeParams.transmitFrequencyHz` and `.noiseBlanker` (`.percent(_)` /
    `.sweep(step:toleranceHz:)`), and `Capabilities.transmitFrequency` / `.noiseBlanker`.
    Written without a Swift toolchain on hand, so not yet built or run: verify with
    `bindings/swift/scripts/test.sh` on a Mac.
  - Kotlin: the binding could not set any `MfskDecodeParams` field (it opened every session
    with NULL params), so it reached neither `freq_hint_hz`, AP, SIC nor these two. It now
    has `MfskDecodeParams` (start from `Mfsk.defaultParams(mode)` and `copy`),
    `MfskApHint`, `MfskNoiseBlanker`, `MfskSession.open(mode, params)` and a per-call
    `decode(samples, params = …)`, plus `CAP_NOISE_BLANKER` / `CAP_TX_FREQ`. The JVM test
    pins each slot of the array marshalling by the refusal that names it, and runs the AP +
    transmit-frequency case end to end (4 of 12 without `txFreqHz`, 11 with). Run here on
    Temurin 17 and Kotlin 2.0.21, as CI does.
  - Still to expose from #466: the Q65 knobs (pileup, max drift, EME delay, q3 list
    decode, `Q65History`, `Q65Callers`, the flagged encode) and FT8's `previous_cycle`.

- **The GFSK synthesiser sampled its frequency pulse one sample early (#482).**
  `gen_ft8wave.f90` (and `gen_ft4wave`, `gen_fst4wave`) build the 3-symbol Gaussian pulse
  with a 1-based loop, `tt=(i-1.5*nsps)/nsps` for `i=1..3*nsps`. `engine::dsp::gfsk` ran
  the same formula from `i=0`, so every FT8, FT4 and FST4 waveform this crate made (the
  transmitter, the subtraction references, FT8's a8 correlation) had its whole pulse train
  shifted one sample. That is a phase error up to 2π·Δf/fs: 0.023 rad on FT8, 0.049 on
  JTTY, whose own port did it right and found this.
  - All three pulse tables are fixed: `synth_f32_into`, `synth_complex_f32_into` and
    `GfskStream`.
  - Against `ft8sim`'s noiseless output (v3.2.0-rc1, SNR 99), the worst normalised sample
    error went from 1.3e-2 to 1.2e-4 (`tests/gfsk_vs_wsjtx.rs`). The residue is
    `gen_ft8wave`'s phase table for its complex output.
  - `synth_matches_gen_ft8wave` compares the synthesiser with a 1-based f64
    transliteration of `gen_ft8wave.f90`.
  - Effect on decoding is small. The fixed-point FT8 qso3_busy recall (the embedded ship
    configuration, which subtracts) went from 11/20 to 12/20, still with 0 extra.
  - Tier C FT8 / FT4 / FST4 single-signal: every crossing and phantom count unchanged.
  - FT8 busy-band, 40 files per density: busy20 `.sic_early()` recall 81.0 → 81.1 % with
    extras 2 → 1, busy40 81.3 → 81.2 %, the rest identical.

- **Q65: the contest caller list, `q65_hist2` / `q65_set_list2`.** In NA VHF / WW Digi /
  ARRL Digi contest mode WSJT-X remembers up to 50 stations that called with a grid,
  and builds its full-AP list from all of them. This crate had neither half.
  - `q65::Q65Callers` is the list, held by the application. `record(freq, msg, now)`
    follows `q65_hist2`: it ignores compound calls, drops ` R `, takes a six-character
    call and a grid, refreshes a known caller, and evicts the oldest at 50. `expire(now)`
    drops callers not heard for 24 hours, and `remove(call)` is `rm_q3list`.
  - `q65::contest_codewords(my, his, his_grid, &callers)` follows `q65_set_list2`: an
    all-zero first codeword, then for every caller, and the DX station when it is
    standard, gridded and not yet listed, `MyCall Caller Grid` / `R Grid` / `RRR` /
    `RR73` / `73`, each with the 78th bit clear and set.
  - Checked against v3.2.0-rc1's own `q65_set_list2` on two callers plus a DX station:
    all 31 codewords identical.
  - Differences from upstream: times come from the caller, as the crate reads no clock.
    Expiry removes every stale entry at once, where upstream's shifting loop skips the
    entry that moves into place until the next decode.
  - Also `msg::wsjt77::pack77` now packs an `R <grid>` ending (`K1ABC W9XYZ R EN37`, the
    grid with `ir = 1`), as `pack77_1` does and as `unpack77` prints it. It fell through
    to the report parser and returned `None`.

- **Q65: WSJT-X's q3 list decode, and the Max Drift 50 stage 5.** With a codeword list,
  `q65_dec0` synchronises on all 85 symbols of every list message within F Tol of the Rx
  frequency (`q65_ccf_85`: symbol spectra at 8 steps per symbol, interpolated and
  smoothed as `q65_symspec`). Where one message leads the runner-up by 1.10, it list
  decodes at that alignment with the fast-fading metric over the `b90` sweep
  (`q65_dec_q3` → `q65_dec1`, accepted above `PLOG_MIN` = -242). This crate's
  `.ap_list()` instead matched templates with the AWGN metric at every 22-symbol sync
  candidate, and replaced the scan.
  - `q65::DecodeRequest` gains `.rx_freq(hz)` and `.ftol(hz)`, default 10 Hz as the
    `jt9` CLI. `.ap_list()` with an Rx frequency runs the ported q3 first, then the
    usual scan for the rest of the band, as `q65_decode.f90` does. Without one it keeps
    the old per-candidate match.
  - At `.max_drift(50)`, when nothing decoded near the Rx frequency, the q3 decode
    runs again on symbol spectra shifted by the drift found there (the "w3sz"
    stage 5, `q65.f90:211-250`).
  - `q65sim` Q65-30A, 20 WAVs per level at -24 / -26 / -28 / -30 dB: q3 here 20 / 20 /
    7 / 2, and `jt9 -3 -d 1` with the same list (`-c K1ABC -x JA1ABC -g PM95 -f 1500
    -F 10`) 20 / 20 / 7 / 2, the same files. It was 1 at -30 dB until the decoder metric
    used the punctured code rate (below).
  - Stage 5 on 10 WAVs at -24 dB drifting 60 / 120 Hz per minute: 0 / 0 → 7 / 1.
  - Noise-only, F Tol 100 Hz, 100 slots: 0 false decodes, with and without stage 5
    (`tests/q65_q3.rs`).
  - `MultiPeriodRequest`'s `.ap_list()` (the averaged q3) and `SniperRequest`'s are
    unchanged.

- **Q65 searches ±1 s by default, and `.eme_delay(true)` is WSJT-X's EME delay
  (breaking).** `q65.f90:127-130` searches `lag1 = -1.0 s` to `lag2 = +1.0 s`, and
  extends `lag2` to +5.5 s (`nsps >= 3600`) or +4.0 s (Q65-15) only when `emedelay > 0`.
  WSJT-X's GUI sets that only for "Decode at 52 s" (`mainwindow.cpp:5270-5271`).
  `default_search_params()` was +5.5 s late for every sub-mode. It had been measured
  against the `jt9` CLI, which turns the EME delay on for TR 60 s by itself
  (`jt9_params_init.f90`), so the source's `nsps >= 3600` could not explain why Q65-30A
  stopped at +1 s. The default is now -1.0 .. +1.0 s, and `.eme_delay(true)` on
  `q65::DecodeRequest` / `MultiPeriodRequest` restores the late reach
  (`q65::search::eme_delay_late_sec`). `tests/dt_window.rs` pins both.
  - Q65-60A at +1.0 s decodes by default.
  - Q65-60A at +5.0 s decodes only with the delay, as `jt9` with it on does.
  - Q65-120D at +1.5 s decodes at the window's edge, reported as dt +1.0.
  - Not closed: `jt9` (delay off, -12 dB) still decodes a Q65-120D frame 2.0 s late at
    that edge, and this crate reaches +1.5 s. The 0.5 s shortfall is recorded, not
    covered by widening the window past upstream's.
  - Also fixed: Q65 reported `dt_sec` from the start of the buffer, not from the
    nominal start (#397), whenever the nominal start was at least the 1 s early
    tolerance in. That covers every Q65-60 and longer request, and every
    `MultiPeriodRequest`. A Q65-60A frame 1.0 s late read +1.95.
  - Tier C (`q65_sim_sweep`, re-baselined): the sweep passed a nominal start of 0, so a
    TR 60 s frame, which `q65sim` places at 1.0 s, sat on the new window's edge and cost
    up to 0.5 dB. It now passes the sub-mode's `TX_START_OFFSET_S`, as upstream's `j0`.
    With that, the 60/120/300 s crossings are within ±0.33 dB of before. Q65-15A/30A
    are 0.2-0.3 dB later, because their window is now upstream's -0.5 .. +1.5 s of
    the slot, not the old -1.0 .. +5.5.

- **Q65's decoder metric used the unpunctured code rate, and its list decode had no
  `PLOG_MIN`.** `q65_init` sets `decoderEsNoMetric = nm * R * EbNoMetric` with
  `R = _q65_get_code_rate()`, which is message length over codeword length after
  puncturing. Q65's (15,65) code is `QRATYPE_CRCPUNCTURED2`, so that is 13/63. This crate
  used 15/65, a metric 12 % high, in every Q65 intrinsics computation (AWGN and
  fast-fading).
  - Found chasing a q3 decode that `jt9` made at -30 dB and this crate did not. The
    winning codeword's log-likelihood was -242.78 here against WSJT-X's -241.92 on
    identical symbol spectra, 0.78 under `PLOG_MIN`. With 13/63 it is -241.92 to the
    hundredth.
  - Correcting it let a wrong codeword through `MultiPeriodRequest`'s AP list on the
    Q65-60B troposcatter golden (`VK7MO VK7PD +44`). That path accepted anything above
    the list-size threshold (-260 + ln(ncw/3), about -256), where `q65_dec1` also
    requires `plog > PLOG_MIN` (-242) and a message that is not all zeros. The crate's
    list decodes (`.ap_list()` scans, sniper, multi-period) now apply both. Goldens back
    to 0 extra.
  - Tier C Q65 (plain and CQ-AP): every group within 0.15 dB.

- **JTTY: `Params::carry`, `Params::sequential`, an `f32` `subtract_frame`, and work counters (`jtty-stats`) (#499).**
  The counters showed that most candidates the ladder rejects are the tails of frames in the windows after the one that
  decoded them. `carry` (off by default) subtracts a decoded frame from the later windows it overlaps: ladder calls fall
  by 45 % and, against WSJT-X's simulator, weak stations recovered rise 69 → 95 of 100 (easy) and 101 → 108 of 200 (hard;
  `rjtty`: 69 and 103). `sequential` decodes a pass's candidates one after another as upstream does, which makes the hard
  set identical to `rjtty` (103, no message only one has); with both, 95 and 110. `subtract_frame` no longer uses `f64`
  (a reference from `tx::Synth::at` + `fill_complex`, the `cos²` filter as sliding sums over `f32` tables; 1e-3 of the frame
  from the `f64` form, kept as `subtract_frame_f64`); it was 1.8 s per call on the CoreS3. `jtty-stats` (host only) counts
  and times the stages; `docs/notes/JTTY_EMBEDDED_BUDGET.md` has the per-window model. `Params` gains two fields.

- **JTTY: trellis metrics in `f32` as an option (`Plan::decode_f32`, `Ladder::with_f32_metrics`, `Receiver::with_f32_metrics`, #499 E1b).**
  The list-WAVA trellis carries its path metrics in `f64` (as upstream); the option carries them in `f32`, which
  is hardware on an Xtensa LX7. Default unchanged. Agreement with `f64`: upstream's 33 ladder cases identical in
  all 132 lists (worst clean metric 3.4e-7 relative); 12 300 synthetic lists, one differs (a near-tie) and 0 of
  4 100 frames accept a different word or rung; the `jtty_sweep` corpus scores byte-identically. On the CoreS3 it is
  worth 1.3× (a rung-1 success 856 → 651 ms), not the 10× needed; the analysis is in `JTTY_UPSTREAM.md`, "E1b results".

- **JTTY transmit sequencer for the boards (`mfsk-app-shared::jtty_tx`, #499, E3).** The pure, half-duplex
  sequencing between "a message of N samples is ready" and "the radio is back on receive": Idle → Waiting
  (channel clear for `hold`, at most `max_wait`, then cancel or send anyway) → Lead (PTT on) → Sending → Tail
  (PTT off). It is driven by the audio clock, not a wall clock: `poll(samples, channel_busy)` returns segments
  that add up to exactly those samples, the PTT edges among them, and events, and the timeline does not depend
  on how the audio is chunked. A segment that is not `Receive` is a stretch during which the JTTY receiver must
  be fed zeros so its sample-count timeline stays true (BINDINGS §2.8.1). It knows nothing of tones; the board
  fills `Message` segments from `jtty::tx::Synth` at the offsets given, which `hosttest` checks against the
  whole-message synthesis at chunk sizes from 1 sample to a frame. Abort while waiting drops the message; abort
  while sending cuts the audio and runs the tail.

- **JTTY transmit: `jtty::tx::Synth`, the waveform as it is played (#499, E2).** `Synth::<F>::new(tones, f0, amplitude)`
  then `fill(&mut [f32])` produces the samples of `synth_f32` in pieces of any size with the phase carried
  between calls, so a transmitter feeding a sound device needs one buffer, not the whole 362 496-sample
  message, and any chunking gives identical bytes. `Synth<f64>` matches `synth_f32` to rounding.
  `Synth<f32>` is for the LX7, where `f64` is software: a single `f32` running phase drifts 17 mrad from the
  reference over 30 s (the carrier increment's representation error, systematic), so the carrier is a 64-bit
  integer oscillator and only the modulation phase is a wrapped `f32`; what is left is 4.4 mrad at the end of
  the longest message, at 300, 1500 and 2800 Hz alike, and a message synthesised that way in 480-sample pieces
  decodes on the receiver. No drift term (`synth_drifting_f32` stays the test-only whole-message form).

- **Docs: feeding JTTY from a live source.** `BINDINGS.md` / `.ja.md` and `Stream`'s docs say what the
  sample count means as time: a run of dropped samples must be replaced by zeros (or a `reset`), a
  slightly wrong source clock does not matter, and `push` belongs on a worker thread. MSK144's
  missing incremental receiver is #497.

- **JTTY text packer and transmit path (#477, phase P5).** `jtty::pack::pack(text, profile)` is
  upstream's `pack_jtty`: normalise the text, then pick the fewest frames with a dynamic program
  over character offsets (a callsign action, control phrase, number, grid, `599 <location>` or
  Field Day class/section is one frame, other text five characters a frame; a structured atom
  must round-trip through encode → decode → render and read exactly the text; ties go to the
  structured atom, the longer span, then the lower `100·kind + 2·subtype + role`).
  `ExchangeProfile::RttyRoundup` adds serial-number and state candidates and rewrites `599 5` to
  `599 005`. It refuses rather than truncates (`PackError`). Checked against upstream's own
  `pack_jtty` (a new oracle driver, `scripts/jttysim/jtty_pack_oracle.f90`) on 3 525 messages
  × profiles (`tests/jtty_pack.rs`, `embedded-poc/assets/golden/jtty/pack_cases.tsv`,
  `scripts/gen_jtty_pack_cases.sh`): the same frames every time. That found one upstream quirk,
  kept and documented: a two-letter section after a class token (`1F DX`) is sent as text,
  because upstream's section lookup rejects an argument shorter than three characters.
  What WSJT-X's GUI wraps around it (F-key templates, N1MM tags, choosing the profile) is host
  policy and not here, the line #463 draws for the QSO-state decoders.
  C ABI: `mfsk_jtty_encode_tones` (text → tones), `mfsk_jtty_synth_len`, `mfsk_jtty_tones_to_i16`,
  `mfsk_jtty_tones_to_f32`; the JTTY mode now also publishes `MFSK_CAP_ENCODE`. Kotlin `MfskJtty`
  (`tones`, `synthesize`, `encode`) and Swift `Jtty` / `JttyProfile`. Every binding test now also
  sends text and receives it back through its own receiver.

- **JTTY over the C ABI, Kotlin and Swift (#477, phase P4b).** `MfskMode` 25 is JTTY
  (`MFSK_MODE_JTTY`, `MFSK_CAP_STREAM_RECEIVER`), and a receiver handle of its own carries it:
  `mfsk_jtty_params_init` / `_open` / `_set_params` / `_push_i16` / `_push_f32` / `_finish` /
  `_reset` / `_pending` / `_poll` / `_close` (`MfskJttyParams`, `MfskJttyUpdate`, both
  size-versioned). Decoding runs inside `push`; the updates wait in a queue in the handle that
  coalesces per message (upstream's rule), capped at 1024 messages. A new `jtty` feature of
  `mfsk-ffi` (in `desktop` and `mobile`) gates it; without it the entry points stay in the
  header and answer `MFSK_STATUS_UNKNOWN_PROTOCOL`. `MfskJttyReceiver` (Kotlin, with the JNI
  shim) and `JttyReceiver` (Swift) wrap it. The C++ driver, the JVM test, the XCTest suite (73
  cases now) and `tests/jtty_ffi.rs` each feed upstream's sample recording in chunks and expect
  `RAN ALL NIGHT ON BAND NOISE - NO FALSE DECODES!`, complete, at about 1507 Hz. Transmit came with P5.

- **JTTY, the streaming receiver (#477, phase P4a).** `jtty::rx::Stream` takes 12 kHz audio in
  chunks of any size and passes each `MessageUpdate` to a callback from inside `push`;
  `finish` reports the messages still open when the audio ends, `reset` starts over. It keeps
  only the last `NCHUNK + 3 * STEP` samples (about 45 000), which is what the retro re-sweep can
  still reach, and one `Arc<Receiver>` serves any number of streams. The updates equal
  `Receiver::scan_messages`' for every chunk size tried (1 sample to the whole recording).
  `Receiver::scan_messages` and `Stream` now run the same window step, so there is one schedule.
  Tier C: `tests/jtty_sweep.rs` (`jtty_snr_sweep`) is wired into `run-sensitivity-sweeps.sh` as
  the `jtty` group and into `sweep-baseline.json`: 50 % crossing -16.20 dB on AWGN and -15.25 dB
  on the moderate fading channel, 0 unexpected decodes in 360 files. `release-status.sh` now
  watches `src/jtty`. `LIBRARY.md` (and its `.ja.md`) documents the receiver, with a doctest.

- **JTTY, several signals (#477, phase P3).** `Receiver::scan_messages` returns messages: a
  decoded frame is re-encoded and subtracted from the window (`jtty::subtract`) so a weaker
  station beneath it is found on the residual; the three windows before it are searched again
  with it gone (the retro re-sweep); an active message whose next frame is due gets a retry at
  the remembered sync point; and frames are joined into messages, repeats absorbed, with
  upstream's continuation, gap and completion rules (`jtty::assemble`, `MessageUpdate`).
  `Params::subtract` turns it off (a single-signal receiver). Against `rjtty` from `v3.2.0-rc1`:
  seven mixtures with a station in every channel assemble the same messages; on 100 random
  two-station recordings the two decode the identical messages; on 200 hard ones (12–50 Hz apart,
  almost simultaneous, the weak one −18…−10 dB) this crate recovers 101 of the weak stations to
  `rjtty`'s 103 — the price of deciding a pass's candidates at once on rayon's pool instead of one
  after another (D4). Comparing against `rjtty`'s own subtraction found and fixed a filter error
  of mine (a `cos²` window built from one complex exponential instead of two; invisible at
  constant gain). 11.7 ms a window on one thread, 6–8 ms on the pool.

- **JTTY, the frame decoder (#477, phase P2).** `jtty::rx::Receiver` (`decode_window`, `scan`)
  takes 12 kHz audio and returns the validated frames in it, and `jtty::{correlate, trellis,
  ladder}` are the pieces under it: the list-WAVA decoder of the tail-biting code, the four-rung
  decode ladder, the sync surface, candidate search and peak-up, and the gate. It is a faithful
  port of WSJT-X's `jtty_mdecode` **without** signal subtraction, retro re-sweep or message
  assembly (P3). Against `rjtty` from `v3.2.0-rc1`: the 132 candidate lists of 33 generated frames
  agree word for word; upstream's sample recording decodes the same ten frames at the same
  frequency and time; recall is identical in all 18 cells of an AWGN/fading sweep (AWGN 50 % at
  −16 dB) and in all 22 cells of a frequency-drift sweep; and Gaussian noise decodes nothing. Being
  the same algorithm it also makes upstream's false decodes — one in the vendored WSPR recording
  and one in upstream's own sample; #487 tracks what to do about them. A JTTY receiver, like
  `rjtty`, does not survive a frequency drift of more than about 12–16 Hz/s (#488). With `parallel`
  every independent step runs on rayon's pool (the 237 sync-surface columns of a window, its
  candidates, the ladder's rungs, the windows of a recording) and the output is bit-identical for
  any thread count: 11 ms a window on one thread, 2 ms on 24.

- **JTTY, the wire level (#477, phase P1).** WSJT-X 3.2.0 adds JTTY, a non-slotted 4-GFSK
  mode for RTTY-style contest exchanges; this crate had nothing for it. The new
  `jtty` feature (in `full`; no FFT, no `std` needed) adds `mfsk_core::jtty`: the 32-bit
  source grammar (`jtty::source`, with every validity rule a receiver applies before it
  displays, assembles or subtracts a frame), the CRC-12, the tail-biting K = 10
  convolutional encoder, and the transmit chain atoms → tones → audio (`jtty::tx`). Like
  MSK144 it has no `Protocol` type and is not in `PROTOCOLS`. **There is no receiver yet**
  (P2). Checked against WSJT-X `v3.2.0-rc1`'s own `sjtty`: the ten 34-bit vectors of its
  spec, the tone sequences of 15 messages bit for bit, and the noiseless waveform to 6e-4
  (one frame) / 1e-3 (two) of the peak — upstream's residual, from its single-precision
  phase sum. With `parallel`, frames are encoded and the waveform is synthesised on rayon's
  pool (phase as a chunked scan in `f64`); the output is bit-identical for any thread
  count. Not a use of `engine::dsp::gfsk`: that samples the Gaussian pulse one sample early
  relative to every upstream `gen_*wave.f90` (#482), which showed up here as a 4.9e-2
  waveform error. Plan, upstream reading and fixtures: `docs/notes/JTTY_UPSTREAM.md`,
  `embedded-poc/assets/golden/jtty/`.

- **FST4: WSJT-X's noise blanker (#469).** `fst4_decode.f90` runs its whole decode on
  samples passed through `blanker.f90`, at the level the GUI's **NB** setting names; this
  crate had no blanker. `DecodeRequest::noise_blanker(NoiseBlanker)` (FST4 sub-modes,
  `SupportsNoiseBlanker`) now does the same: `Percent(n)` zeroes the loudest `n` % of the
  samples and the one after each (`ndropmax = 1`), over upstream's `nfft1` samples;
  `Sweep { step, ftol_hz }` decodes at 0, step, … 20 % as NB -1 / -2 / -3 do, trying the
  levels above 0 only within `ftol_hz` of `freq_hint`, and keeps each message once.
  Off by default, as WSJT-X's NB 0 % is, so a request that does not ask decodes as before.
  Against `jt9 -7 -p 15 -X` (AP decodes excluded) on 50 FST4-15 slots with 20 full-scale
  clicks a second: 0 decodes each without NB, 29 (this crate) and 27 (`jt9`) at NB 2 %,
  differing only on four slots at the edge of decodability (`tests/fst4_noise_blanker.rs`).

- **Q65: WSJT-X's Max Drift (#471).** `q65_ccf_22` can search a linear tone drift
  across the frame (`do idrift=-max_drift,max_drift`, symbol `k` read at
  `i+nint(idrift*(k-43)/85)`), and `q65_loops` takes the drift it found out before the
  symbol spectra (`twkfreq`, `a(2)=-0.5*drift`, normalised over the whole T/R period).
  The GUI's Max Drift (0..50, default 0) turns it on; this crate had neither half.
  `q65::DecodeRequest::max_drift(bins)` now does both, for the plain and AP-hint scans.
  As upstream, the undrifted `(0,0)` sweep (`q65_dec_q012`) runs first and the drifted
  grid after it. Off by default, and the default path is unchanged. On `q65sim` WAVs
  (Q65-30A, -20 dB, 10 each) a drift of 30 / 60 / 120 Hz per minute decoded 5 / 0 / 0
  times plain, and 10 / 10 / 8 times with Max Drift (10 bins; 20 for the 120 Hz/min
  set). An undrifted set decoded 10 / 10, and 20 noise-only slots at Max Drift 50
  decoded nothing (`tests/q65_max_drift.rs`). Not ported: the drift-50 "w3sz" stage 5
  (`q65.f90:211-250`). When everything else has failed at Max Drift 50, it re-runs the
  q3 list decode on drift-corrected spectra. Here that decode is `.ap_list()`, which
  does not take Max Drift yet.

- **Q65: WSJT-X 3.2's Pileup "copied last Tx" flag (#470).** WSJT-X 3.2 sends Q65's
  spare 78th payload bit as a flag in Q65 Pileup mode (`genq65.f90`:
  `dgen(13)=2*dgen(13)+iflag`), reports it on decode (`q65_decode.f90:321`, shown as
  `#`), and under that mode leaves the bit free in the `MyCall DxCall ???` AP pattern
  (`q65_ap.f90`, iaptype 3). This crate always sent the bit clear, dropped it on decode,
  and always locked it in AP. Now:
  - `Q65Result::copied_last_tx` reports it (a new public field);
  - `q65::encode_channel_symbols_flagged` / `synthesize_standard_flagged_for` and
    `msg::q65::pack77_to_symbols_flagged` send it;
  - `.pileup(true)` on `q65::DecodeRequest` / `SniperRequest` frees it for a hint that
    names both callsigns and nothing else (`msg::q65::ap_hint_to_q65_mask_for`). Every
    other hint shape still locks it, as upstream's other iaptypes do.

  Default behaviour is unchanged. A flagged `K1ABC JA1ABC -15` in noise (Q65-30A,
  sniper, 20 seeds at σ 20) decodes 6 times blind, 0 times with that hint and the bit
  locked, and 20 times under `.pileup(true)`. The full-AP list (`q65::ap_list`) stays
  flag-clear, as `q65_set_list.f90` builds it. The doubled list is `q65_set_list2.f90`'s,
  the contest-caller list, which remains out of scope.

- **Q65: `q65_hist` — the DX station from recent decodes (#472).** WSJT-X keeps its last
  100 Q65 decodes with their frequencies. On a manual Decode Again with no DX call
  entered, it takes the DX call (the second word) and the grid (the third word, when it
  is a grid other than `RR73`) from the most recent decode within 10 Hz of the Rx
  frequency whose first word is 3 to 12 characters. That feeds the full-AP list
  (`q65_decode.f90:153-155`, `q65.f90` `q65_hist`). The decoder here is stateless, so
  `q65::Q65History` is a small type the application holds: `record(&Q65Result)` /
  `push(freq, msg)` and `lookup(rx_freq_hz) -> Option<DxFromHistory>`, following the
  same selection rules. Checked against upstream's routine on nine messages: CQ, RR73,
  reports, 12- and 13-character first words, hashed calls, a one-word message and free
  text. It is not wired into any request: when to ask (Decode Again, no call entered)
  is the application's decision, as it is WSJT-X's GUI's. The contest-caller variant
  `q65_hist2` stays out of scope with `q65_set_list2`.

- **FT8: WSJT-X's a7 and a8 list decoders (#464).** `ft8_decode.f90` runs two passes
  after the candidate loop that this crate did not have. Both choose among messages built
  from call signs already known instead of decoding the LDPC code
  (`ft8::list_decode`).
  - **a7** (`ft8_a7.f90`): for each message decoded in the same sequence one cycle earlier,
    the 206 messages the same pair could send next (RRR, RR73, 73, grid, CQ, reports) are
    scored against the four LLR variants at that decode's frequency and DT; the closest is
    accepted if its distance is at most 100 and the runner-up is 1.3 times further. The
    table follows `ft8_a7_save` (first two words and a grid; `/`, `<` and `CQ_` skipped; a
    station decoded again this slot within 3 Hz is not tried). New
    `DecodeRequest::<Ft8>::previous_cycle(&decodes)`: the application keeps the previous
    cycle's decodes and passes them, so the library stays stateless.
  - **a8** (`ft8_a8d.f90`, new in 3.x): with MyCall, HisCall and HisGrid (`ap_hint`) and the
    QSO frequency (`freq_hint`), the waveforms of that QSO's messages are correlated against
    the signal near the QSO frequency over +-1 s and +-5 Hz, and the best fit is kept if it
    passes `nhard <= 54`, `plog >= -159` and `sigobig >= 0.71`.
  - Pass ids 30 (a7) and 31 (a8). Both run at the end of every FT8 strategy; `.sic_early()`
    runs them on its residual, as upstream runs them on `dd` after subtraction.

  Measured against `jt9` 3.2.0-rc1 (built from the tag, the CLI's default calls cleared with
  `-c b -x b` for a7 since they are `K1ABC`/`W9XYZ`/`EN37`), `ft8sim` 3.2 corpora, 20 files a
  cell, same files for both (`tests/ft8_list_decode.rs`, `FT8_BENCHMARK.md` section 16).
  a7, `K1ABC W9XYZ RR73` after a strong `-10` one cycle earlier, decoded / of which by a7:
  jt9 -20 dB 20/1, -21 20/5, -22 20/18, -23 17/17, -24 8/8, -26 0; this crate 20/1, 20/6,
  20/18, 18/17, 8/8, 0. Without the previous cycle both decode 19, 15 (14), 2, 0 (1), 0.
  a8, `K1ABC W9XYZ R-12` at 1500 Hz with the hint: jt9 -20 20/1, -22 19/11, -24 14/14,
  -25 7/7, -26 1/1, -27 0; this crate 20/0, 20/13, 13/13, 8/8, 1/1, 0. Noise at -40 dB:
  nothing from either list decoder. Gates: `a7_finds_the_pairs_next_message`,
  `a8_finds_the_qso_message_at_the_qso_frequency`, `list_decoders_find_nothing_in_noise`
  (mutation-checked).

  Not like upstream: messages with a hashed call in a Type 1 field (a report or grid with
  one non-standard call) are not generated, since `pack28` has no hash form; the Type 4
  shapes are. a7's SNR is this crate's FT8 SNR. The FFI does not expose `previous_cycle`
  yet.

- **`pack77` sends RR73 and -35..-31 dB reports as WSJT-X does, and the RR73 AP
  hypothesis matches a real RR73 (#464).** `pack77_1` (`packjt77.f90`) tries the last
  word as a grid first, so `RR73`, which has a grid's shape, goes out as the grid RR73
  (15-bit field 32373); this crate packed it as `MAXGRID4 + 3` (32403). Both unpack as
  "RR73", which is why it went unnoticed, but they are different codewords: this crate's
  transmitted RR73 was not bit-identical to WSJT-X's, and `ApHint`'s RR73 pattern, which
  copied the same value, never matched a received RR73 (`ft8b.f90`'s `mrr73` is the
  grid). Reports -50..-31 dB wrap by 101 upstream (`irpt+101`, then `+35`); this crate
  wrapped only when `snr + 35` was negative, so -35..-31 packed as 0..4 and -34..-31
  collided with the bare / RRR / RR73 / 73 values (a -34 dB report read back as none).
  Checked by decoding what `ft8sim` (3.2.0-rc1) transmits for ten messages
  (RR73, RRR, 73, -50, -35, -33, R-33, -31, +49, a CQ with a grid): bit-identical now;
  `report_field_matches_wsjtx` pins the values. Found while porting a7, which builds its
  candidates with `pack77` and could not match an on-air RR73. Q65 is the exception and
  is unchanged on the air: `genq65.f90` rewrites the grid RR73 to 32403 before encoding,
  so the Q65 synthesiser (and `mfsk_encode_q65`), its full AP list and
  `Q65Message::pack` now go through `msg::q65::pack77_q65`, which does the same.

- **FT8's heavy AP hypotheses also run near the transmit frequency (#456).** `ft8b.f90`
  tries `iaptype >= 3` within `napwid` of `nfqso` or of `nftx`; this crate had the QSO
  frequency only. New `DecodeRequest::tx_freq(f)`; FT8 reads it with `freq_hint`
  (`pipeline::QsoFreqs`), FT4 and the other modes ignore it, as `ft4_decode.f90` has no
  `nftx`. No sweep passes either frequency, so no baseline moves.

  Tried and not taken: FST4's OSD on `zsave(:,1)` then `zsave(:,2)` with no raw-LLR attempt,
  as `decode240_101.f90` has it. On the FST4 sweep (the same 3520 files) 12 gained and 18
  lost (sign test p = 0.36), unexpected decodes 26 -> 33, so the #146 order (raw LLR, then
  the sum after 2 iterations) stays, with the measurement in its comment.

- **The AP passes take `ft8b.f90`'s bound and `ft4_decode.f90`'s frequency window
  (#456).** `DecodeStrictness::Normal`'s `ap_max_errors` is 36, what `ft8b.f90`
  accepts (`nharderrors.gt.36` is its only bound; `ft4_decode.f90` has none), where it
  was 30, or 25 from 55 locked bits, calibrated against the old AP rung's false accepts
  (gone since #459 and #460). Measured on the sweep corpus at 400 trials a cell (12 800
  files a condition, the same files each way; `docs/notes/FT8_BENCHMARK.md` section 15):
  FT8 with the right hint 8771 -> 9315 hits (+544, +6.2 %, none lost, crossings 0.26-0.39
  dB), with no hint +39; FT4 right hint 7422 -> 7780 (+358, +4.8 %), no hint +31. The
  price is AP hallucinations, a hypothesis locked onto a marginal signal (`K1ABC W9XYZ
  73`, 34 hard errors): FT8 +1 (right hint) or +4 (wrong hint) in 12 800 files. FT4's
  was 4 -> 64 with a wrong hint, and that is what the second half is for.

  `ft4_decode.f90` and `ft8b.f90` try the heavy AP hypotheses (`iaptype >= 3`: MyCall and
  DxCall locked, 58 bits or all 77) only within `napwid` (50 Hz) of `nfqso`; this
  crate tried them on every candidate in the band, which the tighter bound had been
  covering for. New `pipeline::QsoFreq` (`Unknown`, `Near`, `Far`, from
  `DecodeRequest::freq_hint`) and `HEAVY_AP_LOCKED_BITS = 55`: with a `freq_hint`, a heavy
  hypothesis is tried only for a candidate within 50 Hz of it, in FT4's ladder and in
  FT8's AP loop (`decode_sniper_inner` takes its target frequency as the hint). **Without a
  `freq_hint` it is not tried at all**: upstream always has an `nfqso`, so there is no
  case in which it tries one without a QSO frequency to be near. A caller that passes an
  `ap_hint` locking both callsigns now has to pass the QSO frequency too
  (`pipeline::ap_hypothesis_allowed`). FST4 is left alone (upstream's has no such test). FT4 with a wrong hint, hint at the signal: 64 -> 6 phantoms (14 with the hint
  away from it), hits 6232 -> 6274; with the right hint 7821 hits and 6 phantoms (7422 and 3
  before this change), and 6232 with the hint 600 Hz away, the heavy hypothesis being off
  there, as upstream's. FT8's phantoms sit on the true signal, inside the window, so the
  rule does not touch them (9 -> 9). FST4-15 and -30, all four channels: crossings 0.00 to -0.17 dB, unexpected
  decodes as in the baseline.

- **FT4's OSD runs the way `decode174_91` runs it (#456).** `ft4_decode.f90`
  decodes every pass, blind or a-priori, through `decode174_91(llr, Keff=91,
  maxosd=2, ndeep=2, apmask)`: BP, then OSD on the BP sum after 1 and after 2
  iterations, a test pattern that flips a locked bit skipped, the CRC checked
  once, on the winner. `Ldpc174_91::decode_soft` ran `osd_decode_deep` /
  `osd_decode_deep4` on the raw LLR instead, the CRC on every candidate; it now
  runs the search #459 gave FT8's AP rung (`osd_decode_npre1_masked` on
  `bp_llr_zsum_ap_with_scratch`), at every `osd_depth >= 1`, with or without an
  AP mask. Measured on iid Gaussian LLRs (sd 2.83): before, **22.6 %** of
  `osd_depth` 2 calls passed the CRC (6787 of 30 000) and every depth-3 and
  depth-4 call did, against `decode174_91`'s 5.8e-5; after, 29 in 300 000
  (9.7e-5). The pipeline's `hard_errors < osd_max_errors` gate was what kept
  them out (19 of 30 000 got through it before, 25 of 300 000 after).
  `tests/ft4_osd_false_accept.rs` holds the rate.

  On 3 000 slots of white noise through the whole pipeline: `(sync_min 1.2, 50)`
  0 phantoms before and after, `(0.05, 100)` **9 before, 2 after**. Tier C, FT4
  (same machine, before and after): crossings unchanged (AWGN, ccir_good,
  ccir_poor 0.00 dB, ccir_moderate -0.09 dB), unexpected decodes 0 in every group
  as before; the WSJT-X FT4 recording gives the same 11 decodes at both request
  shapes, at 2.5 ms a decode against 2.4 for `(1.2, 50)` and 4.8 against 6.2 for
  `(0.05, 100)` (before), and every FT4 test passes under `fixed-point`.

  Three more places where the FT4 ladder differed from `ft4_decode.f90`, each
  measured on its own. **The post-OSD `hard_errors < osd_max_errors` gate is gone**:
  upstream's FT4 has none (`nharderror.ge.0` and a message that unpacks), and switched
  off it moved none of the noise slots, the sweep or the recording (`osd_max_errors`
  stays as public API and says it is applied to no protocol). **The AP rung runs the
  OSD**: upstream's AP passes go through the same `decode174_91(maxosd=2, apmask)` as
  the blind ones, and this rung had `osd_depth: 0`, BP only. Crossings 0.34 / 0.39 /
  0.86 / 0.67 dB more sensitive (AWGN, ccir_good, ccir_moderate, ccir_poor), unexpected
  decodes still 0 in every group, noise slots `(0.05, 100)` 3 against 2, the recording
  the same 11 decodes at 2.28 ms against 2.24 for `(1.2, 50)` and 5.15 against 4.64 for
  `(0.05, 100)`. **`maxosd = 3` near the QSO frequency**: `ft4_decode.f90` decodes a
  candidate within `napwid` (50 Hz) of `nfqso` with the OSD on a third BP-sum snapshot
  (`zsave(:,3)`), blind and AP passes alike. New `FecOpts::osd_snapshots` (default 2;
  `Ldpc174_91` reads it, the other codecs ignore it) and `pipeline::QSO_WINDOW_HZ`; a
  candidate within 50 Hz of `DecodeRequest::freq_hint` gets 3. On identical BPSK/AWGN LLRs
  3 snapshots recover everything 2 do and more (257 against 226 of 4000 at amplitude 0.9,
  none by 2 alone; `tests/ft4_osd_false_accept.rs`). On the FT4 sweep with the hint on
  its signal (`MFSK_FT4_SWEEP_FREQ_HINT=1500`, 400 trials a cell, 20 800 files, the same
  files at `maxosd` 2 and 3): 10 972 -> 11 013 hits, **41 gained and none lost** (sign
  test p < 1e-4), all four channels; the crossings move by only 0.02-0.04 dB, because it
  acts on the cells at the threshold (37 % -> +1.1 points at -18 dB, 12 % -> +0.4 at
  -19 dB), and unexpected decodes stay at 3 in every condition. The hint alone changes
  no file. With `freq_hint 1500` the noise-slot phantoms stay at 0 and 3, and the
  recording gives the same 11 decodes wherever the hint points. `ap_max_errors` (30 / 25) still bounds AP decodes; upstream's FT4 has no such
  bound, and it is shared with FT8, so it is a separate measurement.

  The depth-4 rung of the generic ladder (`osd_decode_deep4`, pass id 13, at
  `nsync >= osd_depth3_min`) is removed: FST4's codec has always mapped depth 4
  to `min(3)` and FT4's now runs one search at every depth, so it repeated a
  failed decode with the same result (FST4-30, all four channels: crossings and
  unexpected decodes identical without it).

- **FST4's OSD searches the code WSJT-X searches, and checks the CRC on its
  winner (#456).** Two differences from `osd240_101.f90`, both in
  `osd_decode_npre_generic` (FST4's ndeep 2/3 path).

  *The winner.* As in #455, it verified every candidate and kept the closest
  CRC-valid one; `osd240_101.f90:285` runs `get_crc24` once, on the winner.

  *The code.* `fst4_decode.f90:478` calls `decode240_101(llr, Keff=91, ...)`:
  only the message and the first 14 CRC bits are free, the last 10 CRC bits are
  cascaded into the code, so the search runs over a (240,91) subcode. This
  crate searched all 101 bits (`P::K`). New `osd::PartialCrc` describes the
  subcode and `ldpc240_101::FST4_KEFF` is 91; the elimination is upstream's
  (column swaps, pivots in the first `keff` columns, window `keff+20`).
  Measured on upstream's own `decode240_101`, same BPSK/AWGN LLRs, ndeep 2,
  4000 draws a point, one `Keff` per process (`osd240_101` builds its
  generator once, so `Keff` cannot change inside one): recovered at amplitude
  1.0 / 0.9 / 0.8, `Keff=91` 3432 / 2071 / 593, `Keff=101` 2903 / 1231 / 211 --
  about 0.5 dB. The `osd-false-rate` measurements of #455/#457 ran
  `Keff=101` (`main240.f90`), which is not what `jt9 -7` runs.

  With 14 detecting CRC bits the wrong-codeword rate per OSD call is FT8's
  2^-14, not 2^-24, so `FrameDecodable::REQUIRES_UNPACK` (new, `false` by
  default, `true` for FST4) refuses a decode whose 77 bits do not unpack, as
  `fst4_decode.f90:570` (`unpk77_success`) does. That also does the job of the
  all-zero-codeword drop at `:484` (the raw all-zero word descrambles to
  `FST4_RVEC`, which does not unpack). Without it the vendored FST4-60
  recording produced a third, unpackable decode (`fst4_wsjtx_samples`).

  Tier C, FST4 (20 groups), against the previous baseline and against real
  `jt9 -7 -d3` (AP decodes excluded, since this crate has none here; `jt9`
  run over the same 5120 files with `scripts/score-jt9-sweep.py fst4`):
  - 50 % crossing, this crate minus `jt9`, mean over the 20 groups: +0.18 dB
    before #456, +0.44 with the winner rule alone, **-0.07 dB** now. Against
    the previous baseline: mean -0.25 dB (AWGN -0.08, fading -0.31; the worst
    group -0.71, FST4-60 `ccir_poor`; four groups reach 0.5 dB, all more
    sensitive).
  - Unexpected decodes: 99 before, **27** now (`jt9`: 7). They are 39 and 10
    independent events by (sub-mode, channel, trial): `fst4sim` draws one noise
    realisation per trial index and reuses it at every SNR, so one phantom
    shows up in every cell of its trial (`jt9`: 5).
  - Not measured: upstream at `norder=3` (over 30 s a draw in gfortran); the
    `Keff` comparison above is `norder=2`, BPSK, without fading. AP decodes
    are 8.9 % of `jt9`'s hits on this corpus (322 of 3608) and are where its
    apparent ~1 dB lead came from.

  `osd_decode_npre_generic` takes a `partial_crc: Option<PartialCrc>` (breaking;
  `None` searches all `P::K` bits as before). The same winner rule for FT8's AP
  path (`osd_decode_deep`), MSK144 and uvpacket is not done (#456); they share
  `osd_decode_generic`. `scripts/score-jt9-sweep.py` reads FST4 and can dump
  per-trial rows in the sweeps' CSV format for a paired comparison.

- **FT8's a-priori OSD runs the way `decode174_91` runs it (#456).** `ft8b.f90`
  sends its AP passes through the same `decode174_91` call as the blind ones,
  with an `apmask`: BP holds the locked bits, OSD runs on the BP sum after 1 and
  after 2 iterations (never the raw LLR), a test pattern that flips a locked bit
  is skipped (`osd174_91.f90`: `if(any(iand(apmaskr(1:k),mi).eq.1)) cycle`), and
  the CRC is checked on the winner. This crate's AP rung ran
  `osd_decode_deep(&llr_ap, 2, Some(check_crc14))` instead: the raw LLR, an
  order-2 search over every bit, the CRC on every candidate. Measured on iid
  Gaussian LLRs (sd 2.83) with the lock `ft8b.f90` sets up, counting decodes
  that pass the CRC: CQ lock (bits 1-29, 75-77) upstream 111 in 1 200 000
  (9.3e-5), this crate before **6676 in 30 000 (22 %)**, after 34 in 300 000
  (1.1e-4); 58-bit lock upstream 109 in 1 200 000, before 6730 in 30 000, after
  32 in 300 000. The order-2 search has thousands of candidates that flip the
  locked bits, each CRC-checked; `validate` and `ap_max_errors` were what kept
  those out of the decode list. New `osd::osd_decode_npre1_masked` and
  `bp::bp_llr_zsum_ap_with_scratch` (a locked bit keeps its value in `zn`, as
  `decode174_91.f90:52-58`); `tests/ft8_ap_osd_false_accept.rs` holds the rate.

  Tier C, FT8 (this machine reproduced the stored baseline exactly before the
  change): crossings 0.03-0.28 dB more sensitive on the CCIR/AWGN sweeps
  (`decode()` and `.sic_early()`), 0.12-1.00 dB on the eight ITU channels that
  cross (`itu_hm` -0.83, `itu_ld` -1.00, both on the noisy 25-55 % plateau);
  unexpected decodes unchanged at 0 on every sweep and 3 on the busy band; busy
  band recall -0.3 to +0.3 points (`.sic_early()` -0.1 to -0.3, at most one
  signal in 400). On `qso3_busy` the only difference is `CQ EA2BFM IN83`: it
  is now decoded in the `sync_min = 1.5` phase (blind-CQ AP, 18 hard errors)
  instead of the second one (29), so `ft8_qso3_staged_sic_check` looks for it
  in either. `ap_max_errors` (30 / 25) was calibrated against the old rung's
  false accepts and is left alone; loosening it towards upstream's 36 is a
  separate measurement. FT4's OSD (`Ldpc174_91::decode_soft`) has the same
  per-candidate CRC on the raw LLR and is not touched here.

- **FT8's OSD checks the CRC on its winner, as `osd174_91` does, not on
  every candidate (#452, #453).** `osd174_91.f90` takes the closest of all
  its candidate codewords by weighted distance and runs `get_crc14` once, on
  that one. This crate's OSD ran the CRC on every candidate and kept the
  closest one that passed: more sensitive on weak signals, and about forty
  times likelier to return a wrong codeword from noise. Measured on iid
  Gaussian LLRs (sd 2.83) through the ladder's `bp_llr_zsum` then
  `osd_decode_npre1`: upstream `decode174_91` (gfortran, `maxosd=2`,
  `norder=2`) 15 CRC-valid results in 260 000 draws (5.8e-5), this crate before
  452 in 200 000 (2.3e-3), after 180 in 2 000 000 (9.0e-5). New test
  `ft8_osd_false_accept` holds that down. The search got cheaper: no
  un-permute and CRC per candidate.

  Effect on the FT8 tier C (against the baseline of #451): unexpected
  decodes go to **0** on the AWGN/CCIR sweeps, the nine ITU channels and the
  200 noise-only files, and to 2 (`.sic_early()`, was 9; `jt9 -d3` 3.2: 4) and
  1 (`decode()`, was 33) over the 420 busy-band files. Recall on the busy band
  moves -0.7 / -0.8 / -0.1 points (`.sic_early()`, busy10/20/40) and -1.0 /
  -0.5 / -0.7 (`decode()`), still at or above `jt9`'s 81.5 / 79.9 / 81.0 %.
  The sweep crossings move by at most 0.17 dB (`.sic_early()` `ccir_poor`
  -19.71 to -19.55 dB, `decode()` `ccir_poor` -19.00 to -19.17 dB); on the
  ITU channels `itu_hm` improves by 0.83 dB and `itu_ld`, whose recall is a
  25-55 % plateau from -8 to +10 dB, crosses 50 % at 0 dB instead of -5 dB
  (6/20 rather than 10/20 at -5 dB): the CRC-aided search was helping there.

  Two things the previous entry left out are ported with it, because both
  turned out to be this defect: **OSD `ndeep=2` for every candidate**
  (upstream; the `q >= 18` `ndeep=3` split is removed, and `CQ EA2BFM IN83`
  on `qso3_busy`, which the split was needed for, now decodes at `ndeep=2`),
  and **no separate `sync_min` for the early checkpoint** (3.0 removed the
  2.0/1.3 scaling; without the OSD fix a zero-tailed buffer produced
  phantoms from noise, now none: 0 in 200 noise files, `.sic_early()` busy
  recall +0.6 points on busy20).

- **FT8 decodes as WSJT-X 3.x's `ft8_decode.f90` / `ft8b.f90` do: three
  passes with a squared metric, a fifth LLR, an nsync floor, and no `/R`
  or `TU; ` outside a contest (#439).** Six changes, each its own commit
  with the tier-C numbers it moved:
  - **Pass structure.** Pass 1 and 2 always run, pass 3 runs when there
    is any decode (an earlier stage's counted, as `ndecodes=ndec_early`);
    before 3.0 pass 2 needed a decode and pass 3 a new one.
  - **`imetric` 2.** Passes 2 and 3 build their bit metrics from `|cs|²`
    rather than `|cs|` (`s2=s2**2`), for the BP variants, the OSD input and
    the AP input alike. FT4, FST4 and every other caller keep `|cs|`.
  - **nsync floor.** A candidate must clear `nsync > 6`, `> 7` in the
    squared-metric passes, `> 8` for `WsjtxDepth::D1`/`D2` (`ndepth <= 2`).
    `wsjtx_depth()` carries the tier to the passes.
  - **Fifth metric `llre`.** Per bit, the raw nsym-1/2/3 metric of largest
    magnitude; BP (pass id 4) and OSD (ids 18, 23).
  - **`/R` and `TU; `.** With no contest active a standard or RTTY Roundup
    message carrying either is dropped after the CRC, as `ft8b.f90` drops
    it. `DecodeRequest::<Ft8>::contest(true)` (`ncontest != 0`) keeps them.
    `qso1`'s frozen list loses `7J0DNY/R PZ9BNR BM87`, which neither the
    2b9d654 nor the 3.2 `jt9 -d3` reports on that file.
  - **AP.** `apmag` comes from the variant in use, and the early
    checkpoint (`nzhsym < 50`) runs no AP pass.

  What it did, on the FT8 sweeps and the busy-band corpus (`max_cand` 600,
  `sync_min` 1.3; the baseline is main before this change): `.sic_early()`
  crossing -0.28 dB on `ccir_poor`, within 0.08 dB elsewhere; the single
  pass -0.10 dB on `ccir_poor` (`jt9`'s own change-by-change ablation found
  +0.33 dB on `ccir_moderate` for the pass rule and the squared metric
  together; here the gain lands on `ccir_poor` and `moderate` does not
  move). Unexpected
  decodes: the squared metric alone took the busy-band `.sic_early()` from
  6 to 18 and the nsync floor, the `/R` filter and the rest brought it to
  9 (`jt9 -d3` 3.2: 4); the sweeps' `.sic_early()` 3 to 5, the busy-band
  `decode()` 38 to 33; recall +0.5 / +1.1 / +0.3 points on busy10/20/40.
  The AP change measured as neutral.

  **`mlag` is 13, as upstream from 3.0.** On the fixed-point ship shape it
  loses one of the 20 known signals of `qso3_busy` at every `max_cand` from
  15 to 30 and ties from 40 up (table at `coarse_sync::MLAG`), so
  `ft8_qso3_apoff_recall`'s fixed-point floor is 11 (was 12); the sweeps are
  unchanged and the busy-band corpus is neutral to slightly better
  (`.sic_early()` extras 12 to 9; that is the 9 above).

  **Not ported in #451, and why (both since ported, see the entry above).** The `q >= 18` OSD `ndeep=3` split
  stays although upstream is `ndeep=2` throughout: forcing `ndeep=2` was better
  on the sweeps (`ccir_poor` -0.35 dB, busy-band extras 11 to 10 and 31 to 21),
  but the WebFT8-shaped phase-2 decode of `qso3_busy` then loses
  `CQ EA2BFM IN83`, which both `jt9` builds report. The early checkpoint's
  `sync_min` scaling (2.0/1.3) stays although 3.0 removed the `syncmin=2.0`
  line: without it a zero-tailed buffer produces phantoms even from
  noise-only files (10 in 200 with `.sic_early()`, 0 with the scaling) where
  `jt9` gives 1, and what in this port makes those buffers so much easier to
  fool is not yet found. `nQSOProgress`/`naptypes`, `napwid` and the contest
  AP types have no counterpart here.

- **FT4's published defaults follow WSJT-X 3.x: `sync_min` 1.18,
  `max_cand` 200 (#440).** `ft4_decode.f90` changed three constants
  between v2.7.0 and v3.0.0 (the tags were read; 3.0.2 and 3.2.0-rc1
  carry the same values): `syncmin` 1.2 → 1.18 (`:195`), `MAXCAND`
  100 → 200 (`:31`), and the AP window `napwid` 80 → 50 Hz (`:352`).
  The first two are `FT4_PROFILE`'s `DecodeDefaults`, so
  `mfsk_mode_defaults` / `registry::defaults_for` now return them.
  **`napwid` is not ported and nothing changes for it**: WSJT-X uses it
  to keep its AP passes near the QSO target frequency, and this API
  has no such aim point (see the note in `engine::pipeline`).

  Callers that pass their own `sync_min` / `max_cand` are unaffected.
  mfsk-core's own FT4 decode with the old pair (1.2, 100) and the new
  pair (1.18, 200) returns the same messages and frequencies on the real
  golden slot and 640 `ft4_sweep/` files (AWGN and three CCIR channels,
  −10 and −14…−20 dB; 366 decodes each, identical file by file). Real
  `jt9 -5` output is byte-identical between a build of WSJT-X
  `2b9d654` and one of `967c85a` over the whole 1040-file `ft4_sweep/`
  corpus at `-d1/-d2/-d3` and over the real `000000_000002.wav`
  (recorded in #444), so no reference moves. The
  candidate-count measurements quoted in `ft4_coarse_sync`'s docs were
  taken at 1.2 and are left as measured.

- **FT8 follows two WSJT-X 3.x constants — SNR floor and gate −25 dB,
  AP magnitude 1.1 — and does not follow a third, `mlag` 13 (#438).**
  Read at the `v2.7.0` and `v3.0.0` tags of `lib/ft8/`:

  | | 2.7 | 3.0 → 3.2.0-rc1 | here |
  |---|---|---|---|
  | `ft8b.f90` SNR clamp and `nsync <= 10 && xsnr <` bail-out | −24 dB | −25 dB | `FT8_SNR_FLOOR_DB` (the `xsnr2` paths only) |
  | `ft8b.f90` `apmag` scale | 1.01 | 1.1 | `Ft8::AP_MAG_SCALE` |
  | `sync8.f90` `mlag` | 10 | 13 | **kept at 10** |

  `Ft8` had used the trait default for `AP_MAG_SCALE`; it now sets its
  own, and the AP ladder reads it instead of a literal `1.01`. The
  default stays 1.01 for protocols that do not override it. The
  adjacent-tone SNR heuristic shared with FT4/FST4 keeps its −24 dB
  clamp: it is not `ft8b`'s formula.

  **`mlag` 13 was tried and reverted.** It changes nothing on the f32
  tier-C sweep (byte-identical, trial by trial), but on the fixed-point
  ship config `ft8_qso3_apoff_recall` drops from 12/20 to 11/20 against
  its floor of 12: a wider primary window reorders the 15 candidates
  that config keeps. Lowering the floor to fit a change that nothing here
  benefits from would be the wrong trade, so `MLAG` stays 10 with the
  reason at the constant. It is worth another look with a corpus that has
  off-time signals (the sweep is all DT 0).

  *Superseded by #439:* with the busy-band corpus (off-time signals) and the
  rest of the 3.x port in, `mlag` is now 13 and the floor is 11.

  **The other two have no measured effect on any local corpus.** After
  each, the FT8 sweep is byte-identical (800 trials, −25…−15 dB, AWGN
  and three CCIR channels) and the crossings stay at the
  `sweep-baseline.json` values (−21.62 / −21.45 / −19.75 / −18.90 dB);
  the FT8 test binaries and the pre-push fixed-point recall tests pass;
  `qso3_busy.wav` gives 14 / 22 / 22 decodes at D1/D2/D3, as before.
  That is expected rather than reassuring. The sweep does run the
  blind-CQ AP pass (`iaptype` 1, which `jt9` also always runs), and as a
  probe setting `AP_MAG_SCALE` to 3.0 left it byte-identical too: with
  this port's normalised min-sum BP a locked bit already dominates the
  channel LLRs, so the scale is insensitive over that range. The −25 dB
  gate matters for phantoms near the floor, which one clean signal at DT
  0 does not produce.

  **Not in this change** (#439): the `nsync` gate that upstream raised
  to 7 or 8 by pass and depth, the fifth bit metric, the AP passes at
  nsym 1 and 2, the `/R` and `TU; ` filter. Those need per-pass state
  and are where the sweep can move: real `jt9 -8 -d3` became 0.33 to
  0.84 dB more sensitive between 2.7 and 3.2.0-rc1 and now leads these
  crossings by 0.05 to 0.81 dB (`FT8_BENCHMARK.md` §13, in #444).
  `MAXCAND` 1000 and `MAX_EARLY` 200 have no counterpart: `max_cand` is
  the caller's, and the staged decode has no early-decode cap. `maxosd`
  for `-d2` does not apply: only the `maxosd > 0` OSD is ported.

- **Two new FT8 corpora that reach what the old ones cannot: ITU fast
  fading, and a crowded band with time offsets and empty files (#447).**
  The AWGN/CCIR corpora hold one signal at DT 0, at most 1 Hz of Doppler
  spread, and never an empty band.
  - `ft8_itu_sweep/` (`FT8_CHANNEL_SET=itu scripts/gen_ft8_sweep_wavs.sh`,
    9 ITU channels x 17 SNR points x 20 trials, needs a WSJT-X >= 3.0
    `ft8sim`). mfsk-core's FT8 crosses 50 % at -19.4 to -20.8 dB on the quiet
    and moderate ones, at -5.0 dB (`itu_ld`) and -1.7 dB (`itu_hm`) on the 10 Hz
    ones, and never on `itu_hd` (30 Hz; 0 % at +10 dB). Named `itu_*`, not
    `ccir_*`: WSJT-X corrected its Watterson simulator in February 2024, so the
    same fspread now gives a wider Doppler spectrum, and the existing `ccir_*`
    corpora (made from the `2b9d654` tree) are milder than the ITU channels
    of the same numbers. Regenerating `ccir_*` from a newer tree gives a
    different channel under the same name (AWGN files still match; verified
    byte for byte on FST4-60). `BENCHMARKS.md` now names the tree each corpus
    needs.
  - `ft8_busy_sweep/` (`scripts/gen_ft8_busy_wavs.py`): 100 single signals
    at -16 dB with DT scattered over -0.5..+1.5 s, 40 files each of 10, 20 and
    40 signals (200-2700 Hz, SNR -24..-6 dB), and 200 noise-only files, with a
    `truth.csv`. Built from noise-free ft8sim signals rescaled and summed, its
    SNR calibration checked against ft8sim itself (within 0.02 dB). New test
    `ft8_busy_sweep` writes an aggregate CSV (`set,strategy,trial,truth,hits,
    extra`); `sweep-regression-check.py` compares total recall (flagging a fall
    of a point) and total unexpected decodes per group, baseline in
    `_meta.aggregates`. Runner groups `ft8_itu` and `ft8_busy`.

  What they show on today's `main` (`max_cand` 600), with `jt9` on the same
  files: on `dt1` mfsk-core finds all 100 (time window: not a problem); on
  10/20/40 signals `.sic_early()` recalls 83.0/81.4/81.0 % against new
  `jt9 -d3`'s 81.5/79.9/81.0 %, and plain `decode()` 79.5/73.8/66.1 %; and
  noise alone gives no unexpected decodes. **The gap is precision**: `jt9`
  gives 0 (2b9d654) or 4 (3.2) unexpected decodes over all 420 files,
  mfsk-core 22 with `.sic_early()` and 48 with `decode()` at `sync_min` 0.8,
  all in files that contain signals. Most of that is `sync_min`: `jt9 -d3`
  runs 1.3, and at 1.3 `.sic_early()` gives 6 at the same recall (0.8 / 1.0 /
  1.3 / 1.6 / 2.1: 22 / 14 / 6 / 6 / 6 extras; recall 82.0 / 81.9 / 82.0 /
  81.3 / 78.5 %). The test's default is now 1.3 (`MFSK_FT8_BUSY_SYNC_MIN`);
  `decode()` keeps 38 there (`.sic_rounds(3)` gives 44). Not the cause:
  OSD ndeep 3 (22 -> 19 when forced to upstream's 2), upstream's post-CRC
  filters (they would drop a quarter to a third). Recorded in
  `sweep-baseline.json`; the decoder is not changed here.

- **Tier C counts unexpected decodes, not just recall, for FT8, FT4 and
  FST4 (#447).** The sweeps now write a trailing `extra` column, the
  number of distinct decoded messages that were not the injected one; every
  sweep WAV holds one transmission plus noise, so each is a CRC-valid
  payload out of noise. `sweep-regression-check.py` stores them per SNR
  cell in `sweep-baseline.json` (`_meta.precision`), compares over the
  cells a run shares with the baseline (so a narrowed re-run still
  compares), and flags a group that gains at least 3 and at least 1.5x
  (`--strict` exits 1). Old three-column CSVs still load.

  Why it was missing: nobody decided against it. #264 put precision in
  tier B after the WSPR case (recall 8/8 with 8 phantoms), and c757d32
  later trimmed the release runner to each file's "real recall gate" for
  speed, which dropped `ft8_strictness_probe`, the one probe that counted
  false accepts, from the run.

  What the first run showed, on today's `main` (recall unchanged: the
  FT8 and FT4 pass columns equal the previous run's row for row, and every
  FT8/FT4/FST4 crossing but `fst4/60` is +0.00 dB from its baseline;
  `fst4/60` moved 0.2-0.55 dB because its baseline was measured on a
  different corpus draw, see below):
  - **FT8 emits CRC-valid garbage next to a strong signal**: 5 unexpected
    decodes in 1040 trials, including at -10 and -15 dB, with
    `hard_errors` 28-36 (the injected message has 2-9), random call signs,
    at the signal's own frequency or off it; one carries `/R`, which
    WSJT-X 3.x drops. `jt9` 3.2 decodes only the injected message from
    the two strong-signal files checked (-15 and -10 dB). `.sic_early()` gives 0-2 per channel.
  - FT4 gives none in 1040 trials.
  - FST4, which accepts on CRC-24 alone, gives 0-25 per group (25 in
    FST4-15 AWGN over 180 trials).
  The baseline records this state; it does not fix it.

  Also: FT8 is swept through `.sic_early()` into its own CSV
  (`ft8_sic_early/...`), because the phantom-prone code lives in the
  non-default strategies; `MFSK_FT8_SWEEP_STRATEGY` and
  `MFSK_FT8_SWEEP_STRICTNESS` select the strategy and strictness by hand.
  FT8 and FT4 are no longer narrowed to a window around the crossing
  (the sweeps take seconds, and the phantoms sit outside that window).
  `ft8_strictness_probe` and `ft4_phantom_rate` are deleted: the `extra`
  column and those knobs cover them. `sweep-regression-check.py --keep
  GROUP` refreshes a baseline while leaving a group whose move is not yet
  explained alone, and records it under `kept_crossings`. It was used once
  here, for the four `fst4/60` groups, until the move was explained:
  running the baseline's own commit (`7561cd57`) on the local corpus gives
  today's values exactly, so it was the corpus, not the decoder. The FST4
  baseline (2026-09-21, another machine) predates the simulators' fixed
  seed (2026-09-23), so it came from a different noise draw; the four
  `fst4/60` entries are now this corpus's values (#448, closed).
  `CONTRIBUTING.md` and `CLAUDE.md` now describe tier C as sensitivity
  **and** precision.

- **FT8's host decode loops share one dedup, one sniper candidate
  closure, one AP hypothesis list and one policy check (#423, first
  slice).** The `message77` dedup was written five times in
  `ft8::decode` and `decode_block`; it is now
  `pipeline::has_message77` / `dedup_unique`. `decode_sniper_inner`
  wrote its per-candidate closure three times (budgeted, parallel,
  sequential) and now writes it once. The AP pass list in
  `process_one_candidate_inner` was `msg::pipeline_ap::ap_passes` line
  for line and now calls it, adding only FT8's own passes 5 and 12; the
  `policy.accepts(codec_is_plausible, FT8_FILTERS, ..)` pair became
  `policy_accepts`. No behaviour change: tier A+B is 815 passed before
  and after, `ft8_qso3_apoff_recall` passes under `fixed-point`, and
  the pre-push feature matrix is green. Still open under #423: the two
  SIC drivers, `fill_symbol_spectra_via_cd0`, the shift loop in
  `triage_candidate`, and `recompute_nsync`.

- **`src/`-side test WAV loaders have one implementation per protocol,
  not one per diagnostic probe (#421 continued).** JT9's own
  `#[ignore]`d probes (`rx.rs`, `decode.rs`, `mod.rs`, `search.rs`,
  `softsym.rs`) carried ~8 copies of a WAV reader that hardcoded
  `data` at byte offset 44 — fragile against any WAV with extra
  chunks (`JUNK`, `LIST`, `bext`, …) ahead of it. They now share
  `jt9::test_util::load_wav_f32`/`_opt`, a chunk-walking RIFF reader
  matching what `tests/common` already does. Two of `rx.rs`'s probes
  (`freq_sweep_1224hz`, `wide_freq_time_sweep`) were `#[test]`
  functions living outside any `#[cfg(test)]` boundary — an oversight
  that compiled them into every build, test or not; both now carry
  `#[cfg(test)]` like the rest of the file.

  FT8's `decode.rs` had the opposite problem: 8 byte-identical copies
  of an already-correct chunk-walking loader, each with a comment
  explaining `tests/common`'s isn't reachable from a `src/` unit test
  (true — it's a separate compiled crate). One copy now lives at the
  top of `decode.rs`'s own `mod tests`.

  MSK144's `spd.rs` and `decode.rs` each carried their own copy of
  `build_i4tone` and `msk144sim_reference_audio` (the independent
  WSJT-X-style oracle used to keep TX/RX test bugs from silently
  cancelling out) — consolidated into `msk144::test_util`, reused by
  both. `tests/msk144_snr_sweep.rs`'s own `build_i4tone` (a separate
  compiled crate, same boundary as FT8's case above) is left alone.

  No behavior change; every touched `#[ignore]`d probe was run
  directly (not just compiled) to confirm the new loaders produce the
  same audio the inline ones did.

- **The registry publishes the library's own search defaults for
  WSPR, JT9, JT65 and Q65 (#413).** `registry.rs` held hand-written
  `DecodeDefaults` for these four, and all four had drifted from the
  `default_search_params()` their `DecodeRequest` actually starts from:

  | mode | published before | now (= the library's default) |
  |---|---|---|
  | WSPR | no band, `max_cand` 0 | 1400–1600 Hz, 200 candidates, threshold 0.1 |
  | JT9 | no band, `max_cand` 0 | 200–4000 Hz, 8, 0.1 |
  | JT65 | no band, `max_cand` 0 | 1000–2000 Hz, 8, 0.1 |
  | Q65 (every sub-mode) | 200–3000 Hz, 32, 0.05 | 200–3000 Hz, 8, 0.1 |

  The profiles now read those functions (which became `const fn`)
  instead of copying them, and a test pins the equality. Through the C
  ABI, `mfsk_mode_defaults` therefore returns `MFSK_STATUS_OK` with a
  real band for WSPR, JT9 and JT65, where it used to return
  `MFSK_STATUS_UNSUPPORTED`. For Q65 it reports the library's 8 / 0.1.
  The old 32 / 0.05 values were the `mfsk_q65_*` family's own
  deliberately wide EME scan, which is unchanged. What those entry
  points decode with is unchanged too: only the published numbers moved.

  These four modes also get their own `SyncScale::SyncFraction`
  (`MFSK_SYNC_SCALE_SYNC_FRACTION = 2`, and `.syncFraction` in Swift).
  Their score is sync power as a fraction of sync plus noise, 0‥1, and
  Q65 had been labelled `CostasAbsolute` beside FT8's 0.8, inviting
  exactly the cross-mode copy the field exists to stop. ABI revision 2
  has not shipped yet, so the new value does not bump it.

- **`DecodeResult::sync_cv` means the same thing on every protocol
  (#414).** FT8 computed the per-Costas-block coefficient of variation
  as `sqrt(Σ(x−mean)²)/mean`, without dividing by the block count, so
  its value was √3 times what FT4 and FST4 report for the same channel.
  Both paths now call one `engine::sync::sync_power_cv`, the population
  CV. FT8's reported `sync_cv` drops by that factor. Nothing in the
  library, the FFI or the board crates thresholds on it. The old
  `sync_cv > 0.3` QSB gain gate was retired when FT8 moved to
  `subtract_tones_lpf`.

- **Docs: FT8's AP rung is on FT8's own ladder (#415).** `CLAUDE.md`,
  `DESIGN_RATIONALE.md` §3 and `LIBRARY.md` / `.ja.md` said AP reaches
  FT8 through `process_candidate_basic`. FT8 does not implement
  `GenericPipelineProtocol`: its AP rung ends
  `ft8::decode_block::process_one_candidate_inner`, which builds the
  same hypotheses inline. Unifying the two ladders is #423.
- **The generic pipeline's candidate ladder finishes a decode in one
  place (#416).** Every converging rung of
  `process_candidate_basic_impl` (BP, OSD depth 2/3, OSD depth 4, and
  AP) ended with the same re-encode → `snr_db` → descramble → message
  gate → `DecodeResult` block, written out four times. It is one
  `finish` closure now. FT4's single-pass strategy and the FST4 macro's
  were the same 60 lines, comment block included; both call
  `msg::decode_request::generic_single_pass` with their own downsample
  config and `nsync` floor. About 110 lines go. Decodes are
  bit-identical: every `DecodeResult` field, including `snr_db` and
  `pass`, on the FT4 and FST4 goldens under default, EQ, AP, OSD-off,
  SIC and loose-threshold requests, and on 108 synthetic weak-signal
  slots that reach the AP rung.
- **`tests/common` gains `parse_snr_tag` and `sweep_dir` (#421).**
  `parse_snr_tag` (a corpus filename's SNR tag, e.g. `m05` → `-5`) was a
  byte-identical copy in all nine sensitivity-sweep test files
  (`ft8_sweep.rs`, `ft4_sweep.rs`, `fst4_sweep.rs`, `wspr_sweep.rs`,
  `jt9_sweep.rs`, `jt65_sweep.rs`, `q65_sim_sweep.rs` and the two FST4
  DDC sniper probes); it moved once into `tests/common`. `sweep_dir`
  (the tier-C corpus directory: an env-var override, else
  `embedded-poc/assets/<sub>` next to the crate) followed the same
  shape in all sixteen call sites but hardcoded a different env var
  name and subdirectory each time — `tests/common::sweep_dir(env_var,
  sub)` now holds the one real implementation, and each file keeps a
  three-line wrapper naming its own env var and subdirectory. Net ~150
  lines removed, no behavior change (the fixed-point resolution order —
  env var first, then `CARGO_MANIFEST_DIR`-relative — and every
  existing `MFSK_*_SWEEP_DIR` name are unchanged).

  The rest of #421 (per-protocol `collect_wavs`/`make_slot`/
  `sample_path` "copies", which turned out to differ in filename
  parsing, synthesis parameters or output type per protocol rather
  than being genuine duplicates; the `src/`-side WAV parsers and
  MSK144 helper pairs; `tests/common/embedded_driver_harness.rs`'s own
  `load_wav_i16`, which is a deliberate copy — its module doc already
  explains it exists specifically to avoid pulling `fft-rustfft` in
  through `mod common`'s unconditional `air_channel`) is left for a
  follow-up; this entry covers the mechanically-safe slice only.

- **One `LlrT` for FT8's host path, not two (#420).** `ft8::decode::decode`
  and `ft8::decode_block::process_candidates` each declared their own
  `LlrT` (`Q11i16` under `fixed-point-llr`, `f32` otherwise) with the
  comment that the two "must track" each other. It is one type alias
  now, in `decode_block::types`, carrying the LLR-scalar history both
  copies used to repeat. `#420`'s other item — collapsing the six
  `pub`-under-`internal-testing` / `pub(crate)`-otherwise signature
  pairs in `engine::pipeline` — turned out not to be possible: `pub use`
  cannot widen an item's own declared visibility (confirmed against
  rustc), so avoiding the duplication would need a bespoke macro
  parsing each function's generics and `where` clause, which is worse
  for this file's readability than the duplication it would remove.
  That item is closed as won't-fix; the issue's cfg-hotspot audit
  stands as documentation of where the pipeline's `#[cfg]`s are, most
  of them justified embedded/host splits per its own findings.

- **One scan-dedup step and one SNR clamp across the scan modes
  (#419).** JT9, WSPR and Q65 each passed the same `|r| &r.message,
  |r| r.freq_hz, |r| r.start_sample as i64` closure triple to
  `scan_dedup_match`, then repeated the same "report to `on_result`,
  push" step. A `ScanRow` impl per result type now names those three
  fields once. `push_unique` covers the dedup-report-push step, and
  `is_scan_dup` covers the check alone for WSPR's SIC driver, which
  subtracts between the check and the push. JT65 keeps
  `scan_dedup_match_cross`, because its candidates are compared in
  unpadded samples against padded rows (documented, and left as is).
  WSPR's two copies of its dedup tolerances are now one pair of
  module constants. JT65 and Q65 shared the same ratio → dB → clamp
  code with different clamps. It is now
  `engine::llr::snr_db_from_sig_noi(…, floor, ceil)`, and each mode
  keeps its own constants (−30 / −1 and −24 / 49). Output is
  bit-identical: every row and every `on_result` call, in order, for
  the JT9 and JT65 recordings, the WSPR golden through both the scan
  and the SIC driver, and five Q65 sub-modes' goldens (74 lines).

- **Q65's receiver decodes through one tail and one scan loop (#418).**
  `q65/rx.rs` repeated the same blocks across its strategies:
  - the "BP, biased by the AP hint when it has one" match, 3 times
  - the "unpack → fallback SNR from narrow or wide energies → single-
    slot or averaged SNR → `Q65Result`" tail, 7 times
  - the coarse-search → decode → dedup → `on_result` loop, 3 times
  - two slot-averaging functions that differed only in the extractor
    they called

  These are now `bp_decode`, `finish` (with small `Energies` /
  `SnrAudio` enums saying which inputs it has), `scan_with`, and one
  `averaged_energies` taking the extractor. The multi-period path's own
  frequency-only dedup is unchanged; folding it into a shared dedup
  key is #419. Output is bit-identical: every `Q65Result` field on all
  11 vendored Q65 recordings across seven sub-modes (plain scan with
  default and wide parameters, AP hint, two fading models, AP list,
  sniper at three offsets with and without fading and AP list, and
  multi-period with and without AP list), and on 96 synthetic noisy
  Q65-30A / Q65-60B slots with a matching AP hint, where every strategy
  decodes (365 lines).

- **LDPC belief propagation has one body per kernel (breaking, #417).**
  `fec::ldpc::bp` carried two hand-kept copies of the sum-product /
  min-sum loop, `bp_decode_generic_kind` and its `_with_scratch` twin
  (about 270 lines each), and two of `bp_llr_zsum`. The unpooled
  entry points are now shims that build a fresh `BpScratch` and call
  the pooled body, as `bp_decode_generic_nms` already did. Likewise
  `Ldpc174_91::decode_soft` and `Ldpc240_101::decode_soft` call their
  own `decode_soft_pooled` instead of repeating it. FT8's host
  `bp_step_select` now decodes through the `BpScratch` it was always
  handed and ignored.

  Removed, with no caller in the crate, the FFI or the board crates:
  `fec::ldpc::bp::bp_decode_nms` (use `bp_decode_nms_with_scratch`, or
  `bp_decode_generic_nms::<Ldpc174_91Params, T>`),
  `bp_decode_nms_q11` (the same with `T = Q11i16`) and
  `llr_f32_to_q11` (use `Q11i16::from_f32(x).0`).

  Output is bit-identical. Compared with `to_bits()` before and after:
  every `DecodeResult` field on five FT8 recordings under default, AP,
  SIC, EQ and OSD-off requests; the FT4 and FST4 goldens; both MSK144
  goldens; and `decode_soft` on all three LDPC codecs over 360 noisy
  codewords each at OSD depths 0/2/3/4, with 120–184 BP convergences
  per codec (4 543 lines, identical). About 410 lines go.

- **One synthesis entry point, `engine::tx::synthesize::<P>` (breaking,
  #391).** Each mode had its own family: `tones_to_f32` / `_i16` /
  `_into` in `ft8::wave_gen` and `ft4::encode`, the same plus
  `_with_gfsk` and `synth_sample_count` in `fst4::encode`,
  `synthesize_audio` / `_into` / `_len` in `wspr::tx`,
  `synthesize_audio` in `jt9` / `jt65`, `synthesize_audio` / `_for` in
  `q65`. Which waveform a mode transmits is a property of the protocol,
  so it is one now: a new `engine::tx::FskWaveform` trait carries
  `Waveform::Gfsk(cfg)` (FT8, FT4, every FST4 sub-mode) or
  `Waveform::Cpfsk` (WSPR, JT9, JT65, every Q65 sub-mode), and five
  functions cover them all:

  ```rust
  ft8::wave_gen::tones_to_f32(&t, f, a)          →  engine::tx::synthesize::<Ft8>(&t, 12_000, f, a)
  ft4::encode::tones_to_i16_into(o, &t, f, a)    →  engine::tx::synthesize_i16_into::<Ft4>(o, &t, 12_000, f, a)
  fst4::encode::tones_to_f32_with_gfsk(&t, f, a, &FST4_120_GFSK)
                                                 →  engine::tx::synthesize::<Fst4s120>(&t, 12_000, f, a)
  fst4::encode::synth_sample_count(&cfg)         →  engine::tx::synth_len::<Fst4s…>(12_000)
  ft8::wave_gen::TONES_OUTPUT_LEN                →  engine::tx::synth_len::<Ft8>(12_000)
  wspr::tx::synthesize_audio(&s, sr, f, a)       →  engine::tx::synthesize::<Wspr>(&s, sr, f, a)
  q65::synthesize_audio_for::<P>(&t, sr, f, a)   →  engine::tx::synthesize::<P>(&t, sr, f, a)
  ```

  The message-level helpers stay: `wspr::synthesize_type1`, and
  `synthesize_standard` in `jt9` / `jt65` / `q65` (plus `q65`'s
  `synthesize_standard_for`), each now a pack-then-`synthesize` over the
  shared path. The `FT8_GFSK` / `FT4_GFSK` / `FST4_*_GFSK` constants stay
  as well, since each protocol's `FskWaveform` points at one and a
  streaming transmitter (`GfskStream`) names it.

  FST4's un-suffixed `tones_to_f32` silently meant FST4-60A, a trap its
  own doc comment warned about. `synthesize::<P>` cannot be called
  without naming a sub-mode, so the trap is gone rather than documented.
  A test helper that took a sub-mode type *and* a `GfskCfg` beside it
  now takes the type alone for the same reason.

  Two capabilities are new, not moved: the CPFSK modes gain `i16`
  output, with the conversion the GFSK path has always used, and the
  GFSK modes can synthesise at other sample rates. Their symbol length
  follows `SYMBOL_DT` exactly as CPFSK's does. At 48 kHz, FT8 lands
  within 0.016 of full scale of the 12 kHz waveform at every shared
  instant, and a test pins that. A second test checks each GFSK config
  against its protocol's own `NSPS`, `GFSK_BT` and `GFSK_HMOD`.

  Output is unchanged at every rate it was previously produced: the
  same 130 TX fingerprints match. The C ABI is unchanged too, and
  `mfsk.h` regenerates byte-identical. Internally, the FFI's
  `mfsk_tones_to_*` and `mfsk_synth_output_len` now dispatch through
  one `with_tone_mode!` table instead of a per-mode `GfskCfg` lookup.
  The three board crates are ported and `cargo check` clean on Xtensa.

- **One `message_to_tones`, generic over the protocol (breaking,
  #391).** `ft8::wave_gen`, `ft4::encode` and `fst4::encode` each had
  their own, differing only in data the trait already carries: whether
  to XOR the message with an RVEC first (`INFO_SCRAMBLE_RVEC`) and
  whether the FEC wants a CRC-14 or a CRC-24. They are
  `engine::tx::message_to_tones::<P>(&[u8; 77])` now, driven by a new
  `MessageCodec::append_crc` — the transmit-side twin of `verify_info`,
  with the same length dispatch in `Wsjt77Message`:

  ```rust
  ft8::wave_gen::message_to_tones(&m)  →  engine::tx::message_to_tones::<Ft8>(&m)
  ft4::encode::message_to_tones(&m)    →  engine::tx::message_to_tones::<Ft4>(&m)
  fst4::encode::message_to_tones(&m)   →  engine::tx::message_to_tones::<Fst4s60>(&m)
  ```

  Its tail, `engine::tx::info_to_tones::<P>`, replaces the pipeline's
  private `encode_tones_for_snr`, which was the same three lines.

  Two argument types move with it. FT8's `message_to_tones` took
  `&[u8]` and returned `[u8; 79]` where FT4 and FST4 took `&[u8; 77]`;
  all three take `&[u8; 77]` and return a `Vec<u8>` now, and FT8's
  `tones_to_*` accept `&[u8]` like the others (an existing `&[u8; 79]`
  still coerces). `DecodeResult::message77()` returns `&[u8; 77]`
  instead of `&[u8]`: it was always 77 bits, and 29 call sites across
  the library, its tests and the board crates were converting it back
  with `try_into()`. Comparing against an owned
  array needs a `*` now (`*r.message77() == m77`).

  Output is unchanged: the same 130 TX fingerprints match, which also
  confirms FT8's switch from its own `ldpc_encode` to the shared
  `Ldpc174_91::encode` changed nothing. The three board crates
  (`m5stack-cores3-app` with every feature, `-s3-app`, `-core2-app`)
  are ported and `cargo check` clean on Xtensa.

- **One Gray code, `engine::gray` (breaking, #391).** JT65 exposed
  `jt65::{gray6, inv_gray6}` and JT9 kept private `gray3` / `inv_gray3`
  copies, plus a third pair as closures in a test. They are one
  width-parameterised port of WSJT-X `igray.c` now:

  ```rust
  jt65::gray6(n)        →  engine::gray::gray(n, 6)
  jt65::inv_gray6(g)    →  engine::gray::inv_gray(g, 6)
  ```

  Writing it as a literal port of the C loop surfaced something the
  copies had avoided by accident: `igray.c` shifts an `int`, and the
  shift reaches 8 for any 4-bit-or-wider value, which overflows a `u8`
  — a panic in debug, and in release a shift by 0 that XORs the value
  with itself and returns 0. The shared version computes in `u32` for
  that reason, and its tests cover every width from 1 to 8 bits.

  `fst4::encode`'s private `append_crc24` moves to
  `fec::ldpc240_101::append_crc24`, next to `crc24` / `check_crc24` and
  the CRC-24 twin of #389's `fec::ldpc::append_crc14`.

  Output is unchanged: 130 TX fingerprints (every mode's Rust and C
  entry points, both sample rates, every FST4 and Q65 sub-mode) are
  identical before and after.

- **JT9 decodes through one builder, `jt9::DecodeRequest` (breaking,
  #403).** The six free functions it replaces — `decode_scan`,
  `decode_scan_default`, `decode_scan_with_depth`,
  `decode_scan_streaming`, `decode_scan_streaming_with_depth` and
  `decode_at` — had names that multiplied with every axis: adding
  `Jt9Depth` alone cost two new functions. Each axis is a method now:

  ```rust
  jt9::DecodeRequest::new(&audio, 12_000)
      .nominal_start(n).params(p).depth(Jt9Depth::Deep).on_result(&cb)
      .decode()                                   // Vec<Jt9Result>
  jt9::DecodeRequest::sniper(&audio, 12_000, start, freq_hz).decode()
                                                  // Option<Jt72Message>
  ```

  `decode_scan_default(a, r)` is `DecodeRequest::new(a, r).decode()`.
  Same shape as `q65::DecodeRequest`, and per-mode rather than the
  generic `msg::decode_request` for the same reason: JT9 decodes `f32`
  PCM, where the generic builder takes `i16`. Pure re-plumbing — the
  JT9 sweep's CSV is byte-identical before and after, and the golden
  test runs in the same 0.28 s. JT65 and WSPR follow in their own PRs.

- **JT65 decodes through `jt65::DecodeRequest` / `SniperRequest` too
  (breaking, #403).** Nine free functions go: `decode_scan`,
  `_default`, `_streaming`, `decode_scan_chase`, `_chase_default`,
  `_chase_streaming`, `decode_at`, `decode_at_with_erasures` and
  `chase::decode_at_with_chase`. The axis JT65 adds is how Reed-Solomon
  runs, and it is one method on either builder now:

  ```rust
  jt65::DecodeRequest::new(&audio, 12_000)
      .chase(ChaseParams::default())              // was decode_scan_chase*
      .decode()
  jt65::DecodeRequest::sniper(&audio, 12_000, start, freq_hz)
      .erasures(&[0, 8, 16, 24, 32])              // was decode_at_with_erasures
      .decode()
  ```

  The two scan loops underneath were the same 60 lines apart from the
  per-candidate decoder call, and are one function now. Both sweep CSVs
  (`jt65`, `jt65_chase`) are byte-identical before and after.

  `Jt65Result::dt_sec`'s doc still said "from the start of the audio
  buffer" and told callers to subtract their nominal start. That has
  been wrong since #397 made it run from the nominal start, and
  following it would have subtracted the nominal twice. The doc says
  what the field holds now.

- **WSPR decodes through `wspr::DecodeRequest` / `SniperRequest`
  (breaking, #403) — the last of the three.** Fourteen public
  functions go. The point decode was the worst of them: `decode_at` →
  `decode_at_with_drift` → `decode_at_baseband` → `_nblocks` →
  `_nblocks_gated` → `_nblocks_gated_drift`, each wrapper passing one
  more argument to the next. It is one builder now, from 12 kHz audio
  or from a baseband the caller already decimated:

  ```rust
  wspr::DecodeRequest::new(&audio, 12_000)
      .table(&mut table)              // was decode_scan_with_table
      .on_result(&cb)                 // was decode_scan_streaming
      .decode()
  wspr::SniperRequest::baseband(&idat, &qdat, 12_000, start, freq_hz)
      .drift(d).nblocks(&[1, 2, 3, 0]).confirmed(&table).refine_drift(false)
      .decode()                       // was decode_at_baseband_nblocks_gated_drift
  ```

  `decode` takes `&mut self` on the scan, because a table is written
  back to; it still chains on a temporary.

  **`decode_scan_subtract` and `_streaming` leave the public API.**
  Both were already `#[deprecated]`, and their own doc said they were
  kept only because removing them was a breaking change. They wrap a
  second SIC layer around the scan that wsprd has no counterpart for,
  measured at 3.3× the cost for zero extra recall on the WSJT-X golden.
  One `decode_scan_subtract(…, on_result: Option<_>)` remains behind
  `internal-testing`, because the ablation that produced the 3.3× figure
  runs through it.

  Still public, deliberately: `WsprCallsignTable`, and the pass-2 stages
  `rank_pass2_candidates` / `deep_decode_pass2_candidate` (feature
  `wspr-pass2-topn`) that the CoreS3 receiver composes its dual-core
  pipeline from. Those are pipeline stages, not a cross product of
  conveniences. The two `embedded-shared` modules that called
  `decode_at_baseband` are ported and `cargo check` clean on the Xtensa
  toolchain with the board's `wspr` feature on.

  The WSPR sweep CSV is byte-identical before and after.
  `STREAMING.md` (and `.ja.md`) now names the builders for all three
  modes; #404 and #405 had left its table on the old functions.

- **Q65's six skipped golden tests run on CI now.** #395 made them say
  that they skipped; this makes them stop. The 30A, 60B, 120D, 120E and
  300A sample directories are vendored under
  `embedded-poc/assets/golden/q65/` beside the 60A and 60D already there
  (11 WAVs, ~23 MB, byte-identical to WSJT-X's `samples/Q65/`), and the
  three Q65 test files resolve through `golden_subdir`, so a missing
  directory is a failure under `MFSK_REQUIRE_CORPUS` rather than an
  `ok`. The tests are the 30A/60B multi-period averaging gates, the
  120D/120E/300A fading-metric gates, and the 30A streaming-vs-batch
  parity check; `q65_snr_matches_jt9_ground_truth` gains its 120D, 120E
  and 300A cases.

  Size was why they were left out, and it measures as no reason: a
  fresh clone was 33 MB in ~2 s, CI checks out at depth 1, and the
  whole set adds ~5 s serial to the ~300 s tier A+B job.

  **A change to a recording now runs the tests that read it.** `ci.yml`
  triggered on `paths-ignore: embedded-poc/**`, which covered the
  goldens too, so a PR that replaced or broke one ran no CI at all —
  and `paths-ignore` has no way to re-include a subdirectory. The
  trigger is a `paths` list now, excluding `embedded-poc/**` and then
  re-including `assets/golden/**`, `assets/*.wav`, `assets/*.bin` and
  `mfsk-app-shared/src/**`. The recordings start tier A+B only, through
  a new `tier_ab` filter output, not `src`'s feature matrix, FFI,
  cross-compile and binding jobs, none of which opens a WAV. The
  `mfsk-app-shared` sources, and `hosttest/` itself, start the `ffi`
  job that runs `mfsk-app-shared-hosttest` — neither was in `src`, so a
  change to either ran that job only if something else did. Four
  filters also lose a `mfsk-core/src/core/**` line that matched nothing.

- **FST4's tier-C gate is 626 lines, not 7 791.** `fst4_sweep.rs` had
  accumulated 46 `#[ignore]`d diagnostic probes from investigations that
  are now closed — #146's AWGN gap, #198's f32-hardcoded `decode_soft`,
  #306's embedded feasibility, #308's `i0` jitter retry, #310's
  rung-major scheduling and soft Costas-margin proposal, #312's sniper
  cap. They sat in front of the four tests anyone actually runs before a
  release.

  They move to `tests/fst4_diagnostics.rs` intact. **Nothing is
  deleted**: several of these are the refutation that closed the issue —
  `soft_costas_margin_separation`, `_escalation_priority` and
  `_conditional_value` are the three steps that talked #310's proposal
  down, and their docs carry the numbers that did it. A design decision
  whose evidence is no longer executable is one nobody can re-check.
  50 tests before, 4 + 46 after.

- **An early signal's dt came back wrong, or as zero (#397).** Two
  defects, in the same handful of lines.

  `dt_sec` left `nominal_start_sample` out of its derivation in Q65 and
  JT65 — `(start_sample - pad) / sample_rate`, correct only when the
  nominal start happened to be 0. And `to_decoded` in Q65, JT65 and JT9
  re-derived dt from `start_sample` instead of reading the field, which
  is exactly what that field exists to avoid: `start_sample` is a
  `usize`, so a frame beginning before the nominal start saturates at 0
  and loses the sign. JT9 had no `dt_sec` at all and clamped inside
  `lag_to_audio_sample`.

  Measured on a Q65-30A and a JT65 frame placed 0.5 s early, where dt
  must be −0.5 s: Q65 reported **−0.013 s** and JT65 **+0.442 s** — the
  wrong sign. Both are in the searches' own reach (Q65's default window
  looks 1.0 s early), so this is not an unreachable corner.

  `dt_sec` is measured from the nominal start everywhere now, the four
  `to_decoded` impls read it rather than recompute, and they take no
  anchor arguments because they no longer need any. `Jt9Result` gains
  the field; WSPR's literal `-1.0` becomes `TX_START_OFFSET_S`, which is
  what it always was. `msg::decoded::dt_from_samples` is gone — nothing
  derives dt from a clamped index any more.

  **Breaking:** `Q65Result`/`Jt65Result`/`Jt9Result::to_decoded` lose
  their `(sample_rate, nominal_start_sample)` arguments, matching WSPR's
  signature. `Jt9Result` has a new public field.

  Sweep CSVs are byte-identical across all five suites, so nothing about
  which frames decode has changed — only what they report. The unit test
  that pinned the old behaviour (`to_decoded` deriving 0.5 s from
  `start_sample` while the struct said 1.0) asserted the bug, and now
  asserts the contract.

- **Q65's published `tx_start_offset_s` was 1.0 for every sub-mode; the
  15 s and 30 s periods are 0.5 (#399).** Upstream makes the nominal
  start depend on `nsps`, not on a single constant: `q65.f90:130-131`
  sets `j0 = 0.5/dtstep` and then `j0 = 1.0/dtstep` when
  `nsps >= 7200`, and `q65sim.f90:164-165` places its signal on the same
  rule. The `q65_submode!` macro applied 1.0 to all ten sub-mode ZSTs.

  **Nothing in the decode path changes.** Q65 centres its search on the
  caller's `nominal_start_sample` and measures `dt_sec` against the same
  anchor; the constant is not read there. The tier-C sweep is unchanged
  — Q65-15A and Q65-30A both 22/330 before and after, 60 s+ untouched.
  What was wrong is `registry`'s published `tx_start_offset_s`, whose own
  doc says a host synthesising a slot "has to know this and could not ask
  for it". That doc also enumerated "0.5 for FT8, FT4 and FST4-15" without
  mentioning Q65, which is how ten sub-modes came to publish the wrong
  number unnoticed.

- **Test assets resolve one way, not two (#395).** Ten sites reached for
  `../../WSJT-X/samples/...` relative to `CARGO_MANIFEST_DIR` instead of
  going through `tests/common::corpus`, which made the answer depend on
  where the working copy sat. The main clone found the tree; a `git
  worktree` under /tmp did not; CI did not either. Same code, different
  amount of work, no output saying so.

  It cost twice. **Six Q65 golden tests had been skipping on CI** —
  60B troposcatter, 120D rainscatter, 120E ionoscatter, 300A optical
  scatter and two more — for as long as the ad-hoc paths existed. And
  timing the #394 refactor compared a worktree against the working tree,
  where those same six skipped on one side only, so the arm doing less
  work read 25 % faster. Closure indirection, `#[inline]`, the shared
  search window, the ranking tail, machine drift, cargo overhead and
  codegen-unit partitioning were each measured and cleared before the
  cause turned out to be a directory nobody was told about. Measured
  properly, in one directory, the change was −5.9 %.

  `corpus` now names three classes instead of two. `golden_path` is a
  vendored asset that must be there and panics under
  `MFSK_REQUIRE_CORPUS`; `optional_corpus` is a tier-C sweep corpus that
  never runs in CI; and `upstream_sample_path`/`upstream_sample_dir` are
  the large upstream recordings that are **optional by design** —
  resolved through `$WSJTX_SAMPLES_DIR` only, never through a path
  relative to this checkout. Point that variable at a WSJT-X samples
  tree and the tests run; leave it unset and they skip, saying which
  variable would have run them. `grep -rn '\.\./\.\./WSJT-X' mfsk-core/tests`
  returns nothing now.

  The 77 bare `eprintln!("skipping…"); return;` sites route through
  `skip_or_fail` (fatal under `MFSK_REQUIRE_CORPUS`) or
  `missing_upstream` (never fatal) according to which class they are, so
  the distinction is visible at each call site rather than implied by
  the path it happened to build.
- **One candidate type, one parameter block, one search window for
  JT9/JT65/Q65/WSPR (#394).** Third step of the per-mode duplication
  cleanup, after #389's TX-side fold and #390's FFT entry point. The
  four modes each carried their own copy of the coarse-search
  scaffolding: `SyncCandidate` (four field-identical definitions),
  `SearchParams` (four, differing only in how the time tolerance was
  spelled), `DEFAULT_SCORE_THRESHOLD` (four, all `0.1`), the row/bin
  window arithmetic, and the sort-and-cap tail — that last one
  byte-for-byte identical in all four. They now live once in
  `engine::search`, along with the time collapse JT9 and Q65 share.
  Net −207 lines.

  **What did not get folded, and why.** The four differ on four
  independent axes, not one: time collapse (best-lag-per-bin vs every
  cell), admission (a fixed floor, or Q65's adaptive percentile gate
  from `q65.f90:553-574`), frequency local-max suppression (Q65 only,
  `q65.f90:563-566`), and frequency estimate (log-power refined vs bin
  centre). One function switching on all four would read worse than the
  four straight-line bodies it replaced, so each mode keeps its own
  policy next to the upstream line it ports. `score_candidate` is
  untouched for the same reason — three of them already delegate to
  `engine::spectrogram`, and WSPR's is a different estimator (per-bin
  `sbase_linear` normalisation), not a copy.

  `engine::sync::SyncCandidate` also stays separate: it carries
  `dt_sec` where these four carry `start_sample`, and merging them
  means converting FT8's candidate path to samples — 58 `.dt_sec` uses
  across 13 files, on the hot path, for no behavioural gain. Both types
  now document why the other exists.

  **Breaking, for four modes.** `jt9`/`jt65`/`q65`/`wspr`'s
  `search::SearchParams` and `search::SyncCandidate` are now
  re-exports of the `engine::search` types; `SearchParams::default()`
  is gone in favour of each mode's `search::default_search_params()`,
  since the defaults are mode-specific data. `time_tolerance_sec`
  becomes the `time_tolerance_early_sec`/`_late_sec` pair, and WSPR's
  `time_tolerance_symbols` becomes seconds — it was the last mode
  still counting symbols. The measurement rationale that lived in each
  `Default` impl moved onto the corresponding constructor.

  **Behaviour is unchanged, and this time that is measured exactly
  rather than argued.** Per-trial sweep CSVs against the same
  deterministic corpora are byte-identical before and after for JT9,
  JT65, JT65-chase, WSPR and Q65. WSPR is the one that could have
  moved — it is the mode whose tolerance changed units — and its
  window is still exactly 32 rows.

- **One CPFSK synthesiser, one CRC-14 append, one top-K pick.** Code that
  several modes had each copied now lives in one place. None of it changes
  output.
  - WSJT-X's plain continuous-phase FSK transmit loop (its
    positive-`toneSpacing` path) was repeated in `wspr::tx`, `jt9::tx`,
    `jt65::tx` and `q65::tx`. It is now `engine::dsp::cpfsk`. Each mode
    keeps its own tone-range assertion and its public signature.
    Fingerprinted before and after over all four modes at 12 and 48 kHz,
    Q65-60C and WSPR's no-allocation `synthesize_audio_into`: the output
    is bit-identical.
  - FT8 and FT4 each carried a private `append_crc14`. It is now
    `fec::ldpc::append_crc14`, beside `crc14` / `check_crc14`.
    `ft8::wave_gen::message_to_tones` still takes a slice and panics on
    one that is not 77 bits long, as it did before.
  - `engine::sync`'s three DT estimators (`bootstrap_dt_median` and the
    two circular ones) each partitioned out their top-K candidates with
    the same block. They now share one private helper.
  - Other candidates from the same survey were measured and left alone.
    The parabolic-peak and percentile sites across FT8, uvpacket, MSK144
    and Q65 are not copies of each other: the clamp ranges, the
    epsilons and the WSJT-X index-rounding rules all differ, so merging
    them would change decodes. A shared Gray-code helper is deferred to
    #391: it only pays off once the public `jt65::{gray6, inv_gray6}`
    can go, which is a breaking change.
- **`jt9`, `jt65` and `q65` are no longer host-only.** They used to imply
  `fft-rustfft`, and through it `std`. #392 took the backend out of the
  way by routing every FFT they run through `engine::fft`; what still
  pinned them was `std` inside the modules — prelude `Vec`/`String`/
  `format!` and bare `f32` methods, 64 / 44 / 117 errors under
  `alloc,<mode>,fft-extern` — and JT9's inverse FFT. Both are fixed, so
  the three features are now unqualified like `ft8`/`ft4`/`fst4`/`wspr`,
  and each has an `alloc,<mode>,fft-extern` row in CI's feature matrix
  and `scripts/pre-push-check.sh`. That row is the guard: it is the only
  build that fails when a `std::` path comes back.

  JT9's `downsam9` had assumed rustfft's convention, where forward and
  inverse are both unscaled. The embedded backend divides by `len` on
  its inverse arm, so the same code would have handed the rest of the
  pipeline numbers a factor of NFFT2 small — and that scale is not free
  there, since `downsam9` normalises against a measured noise floor.
  It now carries the same explicit `#[cfg]` `wspr::subtract` has at its
  own inverse; the `Fft` trait does not specify a convention, so this
  cannot be left to the type system.

  Dropping the implication also broke `--features jt9` on its own, which
  had been riding on `fft-rustfft` to pull in `engine`'s FFT-gated
  modules. Decode-side modules and entry points are now gated on the FFT
  meta-feature the way `wspr` gates its own, with TX and the const
  tables left unconditional.

  **Host behaviour is unchanged by construction** — every edit is inert
  under `std` + `fft-rustfft` — and measured: the tier A+B gate is 799
  passed / 0 failed. What did *not* change is that nothing embedded
  drives these modes yet: no fixed-point path, no board app, no
  on-device measurement. The docs say that rather than implying an
  embedded JT65 exists.

- **The tier-C corpora regenerate byte-identically, so a sweep baseline
  now travels between machines.** `gran_()` draws its noise from C
  `rand()`, and upstream's `sgran_()` seeds that from /dev/urandom — so
  `ft8sim`, `ft4sim`, `jt9sim`, `jt65sim` and `wsprsim` wrote a different
  realisation on every run, and a corpus generated on one machine was
  never the corpus generated on another. `fst4sim` and `q65sim` already
  did not: `fst4sim.f90:109` is `!   call sgran()`, commented out
  upstream, and `q65sim.f90` has no such call. The build scripts now link
  `scripts/sim_sgran_stub.c` in place of `lib/sgran.c`, seeding from
  `MFSK_SIM_SEED` (default 1); `jt65sim`, which comes from WSJT-X's CMake
  build and takes `sgran_` out of `libwsjt_fort.a`, gets the same
  treatment by re-linking through the new `scripts/build_jt65sim.sh`.
  Verified by generating each corpus twice and diffing, and end to end by
  rebuilding all 1 040 FT4 WAVs and finding them identical.

  This started as a 0.68 dB disagreement on FT4's `ccir_poor` between two
  machines running the same commit. It was not the decoder: re-run at the
  baseline's own commit, and at the commit that set its values, the
  per-trial results were byte-identical. It was the corpus, and the same
  effect made an FT8 `ccir_poor` figure move 1.32 dB and look like a
  regression — real `jt9 -8 -d3` over the same files returned −18.88 dB
  against this crate's −18.90 dB, and the eight-seed mean landed 0.01 dB
  from the figure the table had carried all along. `docs/notes/BENCHMARKS.md`
  gains a "Generating the tier-C corpora" section covering all seven
  (JT9/JT65/Q65/WSPR had no written procedure at all), the `-DWSJT_SKIP_MANPAGES=ON`
  that a WSJT-X configure hard-errors without, and the rule that a crossing
  is read by pairing it against `jt9` on the same corpus rather than as an
  absolute number.

  `docs/notes/sweep-baseline.json` is re-measured on the seed-1 corpora for
  FT8/FT4/JT9/JT65/Q65/WSPR. It stays one seed on purpose: against a fixed
  corpus a code change shows up exactly, which is what a release gate
  needs, and it costs the same 70 s it always did. Absolute figures belong
  in `BENCHMARKS.md`, which now carries multi-seed means — one draw is
  worth up to ~1.1 dB on a fading channel. FST4 is untouched: its generator
  was already deterministic, so its baseline still transfers.

  Four `gen_*_sweep_wavs.sh` also stopped breaking when handed a relative
  simulator path — they `cd` into a tmpdir before invoking it, which
  `gen_jt9`/`gen_wspr`/`gen_q65` had already worked around.

- **One spectrogram builder for JT9, JT65, Q65 and WSPR (#390).** The
  shared `engine::spectrogram` builder called `rustfft` directly, so
  WSPR, which has to build under `fft-extern` as well, carried its own
  copy of the same FFT loop and the same bottom-95 % noise estimate. The
  builder now goes through `engine::fft`, and `wspr::spectrogram`
  calls it and keeps only its per-bin baseline fit. rustfft is still
  the host backend (`engine::fft::default_planner()` returns it), so
  nothing changes on host. All four modes were fingerprinted at 12 and
  48 kHz: `mags_sqr`, `noise_per_bin`, `df` and WSPR's `sbase_linear`
  are bit-identical. Build time was measured A/B back to back and is
  within 1 % (JT65 ≈ 24.5 ms, Q65-30A ≈ 26.5 ms, WSPR ≈ 14.5 ms per
  slot on an Apple M5). `engine::spectrogram` is now compiled under
  either FFT backend, not only `fft-rustfft`.

- **JT9, JT65 and Q65 reach every FFT through `engine::fft` (#390).**
  Five demodulator sites planned their own `rustfft` instance for a
  per-symbol FFT: `jt9::rx`, `jt65::rx`, `q65::rx` twice, and
  `q65::snr`. They now share `engine::dsp::symbol_fft::SymbolFft`.
  Each mode still reads its own tone bins, because the reads differ
  (JT9 squares `norm()`, the others take `norm_sqr()`, and JT65 mixes
  its input through an NCO first). Q65's two energy extractors, narrow
  and wide, were the same loop with a different bin map and are now
  one. JT9's slot-length real FFT and its `downsam9` inverse FFT go
  through `engine::fft` as well. For the real FFT, `engine::fft`
  gains `FftPlanner::plan_real_forward` and a `RealFft` trait.
  - `plan_real_forward` is a provided method, so existing
    `FftPlanner` implementations, including the embedded ESP backend,
    compile unchanged: they get a complex-FFT fallback. The rustfft
    backend overrides it with `realfft`, which is now a dependency of
    `fft-rustfft` rather than of `jt9`.
  - rustfft is still the host backend, and the output is
    bit-identical. The fingerprints compared against `main` cover
    JT9's LLRs, slot FFT, envelope and `downsam9` output, JT65's full
    demodulation tuple at a fractional frequency (the NCO path),
    Q65-30A, -60B and -60E narrow and wide energies, and Q65's SNR
    composite spectrum. The JT9, JT65, Q65 and WSPR golden tests run
    in the same time, A/B, within run-to-run noise.
  - `jt9`, `jt65` and `q65` still imply `fft-rustfft`. The FFT
    backend no longer pins them; what does is `std` use inside the
    modules (64, 48 and 117 errors under `alloc,<mode>,fft-extern`)
    and JT9's `downsam9`, which relies on rustfft's unscaled inverse
    FFT. The ESP backend scales its inverse, which is why
    `wspr::subtract` carries an explicit `#[cfg]` for the same point.

- **Breaking: one JT65 demodulator entry point (#390).** Four public
  functions ran the same demodulation pass and each returned a
  different subset of its output. They are now one,
  `jt65::demodulate_aligned`, which returns a `jt65::Jt65Demod` struct
  with named fields. `jt65::decode_at` now reuses the SNR-carrying
  decode instead of repeating it. Output is unchanged: it is the same
  pass, and the JT65 golden and chase tests pass as before. Migration:

  | before | after |
  |---|---|
  | `demodulate_aligned(..)` → `[u8; 63]` | `demodulate_aligned(..)?.symbols` |
  | `demodulate_aligned_with_confidence(..)` → `(symbols, conf)` | `.symbols`, `.conf` |
  | `demodulate_aligned_with_confidence_and_snr(..)` → `(symbols, conf, snr_db)` | `.symbols`, `.conf`, `.snr_db` |
  | `demodulate_aligned_with_runnerup(..)` → 6-tuple | `.symbols`, `.conf`, `.second_symbols`, `.rel`, `.raw_pwr`, `.snr_db` |

- **`pack77` packs `/P` and `/R` callsigns.** It refused them: the
  suffixed call went straight to `pack28`, which takes six characters
  at most, and the whole message came back `None` — so a portable
  station could not produce `CQ SOTA JA1ABC/P PM95` or any reply after
  it, and the only `/P` path left was Type 4, which drops the grid.
  `pack77` now does what `pack77_1` does (`packjt77.f90:1176-1193`):
  the suffix becomes the `ipa`/`ipb` flag beside its 28-bit field, and
  a `/P` on either call makes the message Type 2 (i3=2), otherwise
  Type 1. `unpack77` already read both, so this completes a round trip
  that was half there. Two callers change with it: the FFI's FT8
  encoder (`mfsk_encode_ft8`) accepts suffixed calls instead of
  returning `InvalidArg`, and Q65's `standard_qso_codewords` builds its
  AP list for a `/P` station instead of returning it empty — both now
  as WSJT-X, whose `pack77` never had the gap. `pack77_type1` routes
  through `pack77`, so it gains the same.

- **CoreS3: CAT over the IC-705's USB, and a FREQ page to set the
  dial.** The IC-705 is one composite device (VID `0c26` PID `0036`)
  with two CDC-ACM functions beside its audio; CI-V is "USB (A)",
  probed on the radio 2026-09-23: `03` answered 7.041 MHz, `26 00`
  USB-D FIL1, and transceive `00` frames followed the dial. A CAT task
  (`civ_usb.rs`, over `espressif/usb_host_cdc_acm`) reads dial and mode
  on connect, keeps the status bar's `rig_freq_hz` — which `all.txt`
  and the ADIF log take their frequency from — on what transceive
  reports, and reopens after an unplug. It never touches DTR/RTS,
  which the radio can map to PTT.
  - **The port is opened by its data interface.** The S3's USB host has
    eight channels, one per pipe, and the IC-705 already holds seven.
    Opening the control interface added a notification pipe, the
    PCM2901 then failed to enumerate ("No more HCD channels
    available") and the board restarted every ~33 s. On the data
    interface the driver allocates only the two bulk pipes; audio held
    0 err and FT8 7 decodes/slot with it open.
  - **FREQ** on the menu's root lists the running receiver's presets
    (`mfsk_app_shared::freq_presets`): FT8's IARU channels plus the JA
    ones (1.908 / 3.531 / 7.041 / 144.460), FT4 as WSJT-X lists them,
    WSPR from `WSPR_BANDS`; FST4 has none yet. Three rows and NEXT per
    page; `*` marks the rig's dial. APPLY sends `05` and `26 00 01 01 01`
    (USB, data, FIL1) without a restart and saves the dial to NVS
    (`rig_hz`), which is sent again on the next connect.
  - The probe (reads only, `dc6ecc96`) was measured on the radio; the
    CAT task and the FREQ page have not been run against it yet.
- **CoreS3: NTP sets the clock once per boot, then stops.** SNTP was
  kept running and lwIP re-synced every hour
  (`CONFIG_LWIP_SNTP_UPDATE_DELAY`), *stepping* the clock by whatever
  it had drifted — a jump in the reference the slot grid holds to. The
  handle is now dropped after the first sync (or after a later retry
  lands), the RTC is written from that sync as before, and the crystal
  keeps time for the rest of the session. The crystal's own error over
  a long session is no longer corrected; it has not been measured on
  this board. `TIME: AIR DT` remains the explicit choice where there
  is no network.
- **CoreS3 FT8: an NTP sync inside a slot no longer hides the grid's
  error for two slots.** `GridPhase` estimates the grid against UTC as
  the minimum over a slot's blocks; when NTP landed mid-slot, blocks
  read on the RTC clock and on the NTP clock were minimised together.
  On an IC-705 (2026-09-23) the RTC blocks read −20 samples and won,
  and the real +972 (+81 ms) stood until the next slot measured it —
  the same shape as 2026-09-22's +61.6 / +94.8 ms. A new
  `time_sync::clock_epoch()` counts clock-source transitions, and a
  block on a new epoch restarts the window. Host-tested; not yet
  confirmed on the board.
- **CoreS3 FT8: the slot boundary falls on the sample, not on a
  100 ms chunk.** `Ft8ChunkSink` ended a slot only after a whole
  1 200-sample chunk, so every boundary was rounded up to the stream's
  next chunk edge — anywhere from 0 to 100 ms late, set by where the
  stream happened to start and then kept for the whole session, since
  every later slot is exactly 150 chunks and the NTP re-anchor leaves
  anything inside ±200 ms alone. Every station's DT read that much
  low, and on the SIM that was one weak station of seven missing the
  reply deadline on most boots. The sink now cuts the chunk at the
  boundary. Measured without the clock — the SIM logs where each slot
  boundary falls against its recording's own first sample: before,
  +100, +0/+46/+100 and +100 ms over three boots; after, −9 to +2 ms
  over five, with all seven stations decoded by key-up on four of
  them.

  **And then held to the sample against NTP.** The one-time anchor
  read the clock when a block arrived, so it carried that block's
  delivery delay — a scheduler tick on the SIM, the USB transfer on a
  radio — and the NTP re-anchor left anything inside ±200 ms alone.
  `time_sync::GridPhase` now takes, for every block, the grid's
  distance to its next boundary (from the block's last sample) against
  the clock's (read in microseconds, `utc_now_us`), and keeps the
  minimum over the slot: delivery delay only ever adds, so the minimum
  is the grid's error plus the path's smallest delay. With NTP owning
  the phase, the sink puts the grid on UTC once it is 0.5 ms or more
  out, and after that *holds* it, moving it again only past 10 ms —
  on air a correction every slot was moving the grid 3-6 ms at a time,
  and on this board that is enough to trade one weak station for
  another. Held, the grid drifts 3-4 samples a slot (an IC-705's
  48 kHz against the ESP32's crystal, ~20 ppm). The estimate agreed with the SIM's clock-free
  measurement to within 0.3 ms on every slot, and the grid settled at
  −0.3 to +0.2 ms within two slots of NTP on each of five boots; a
  five-minute run then held −3 samples with all seven stations by
  key-up for 16 slots running — the same seven on every boot, where
  the phase error had been choosing which seven. Without NTP the grid
  stays where the anchor put it (the air owns the phase then). The
  SIM feed was brought into line for the measurement: its recording
  starts at the boundary to the microsecond rather than wherever the
  tick woke it, and it hands a block over after the block's time has
  passed, as a radio does, instead of before.

- **CoreS3: no task-list walk while a radio's USB audio streams.**
  The panel printed a per-task CPU line every 10 s and a stack
  high-water table every 30 s, both through `uxTaskGetSystemState`,
  which on a dual-core IDF holds a critical section while it scans
  every task's stack byte by byte. For those milliseconds core 0's USB
  interrupt waited, the host controller's next isochronous buffer went
  in late, and its first 1-4 packets came back SKIPPED — about 3 ms of
  audio, uncounted, roughly every 10 s. Found by logging the class
  driver's own packet drops (`uac-host` at DEBUG) beside a clock-free
  measure of the slot grid on an IC-705: every drop followed a `[cpu]`
  line, and each cost the grid 36 samples. In USB host mode the panel
  now prints only each core's idle share (one counter read per core),
  and the stack table once at boot; after the change the grid moved
  3-4 samples a slot and nothing else, bar one drop during an
  `all.txt` flash write — the flash still stops everything while it
  writes, which is the next thing to schedule around. Each slot now
  logs one summary line (grid estimate, the next slot's length, the
  longest read wait and between-read gap and when they fell), and
  WiFi and lwIP moved to core 1 — whether that move helped on its own
  was not measured.

- **CoreS3: the USB audio path runs above the panel.** The panel went
  to priority 7 on 2026-09-21 so the screen would not stop during a
  decode, and that put it above the UAC class driver and the reader
  (6) on the same core. The check at the time ran on the SIM, where a
  starved feeder only lags; on a radio, a class driver that cannot
  resubmit its isochronous transfers within their 48 ms drops the
  frames, and nothing counts them. On an IC-705 the reader went up to
  102 ms between reads against an 85 ms ring, and the slot grid showed
  steps of 30 ms of audio that never arrived. The driver, the USB
  event task, the reader and the SIM feeder are at 8 now: the longest
  gap over the next 251 s was 14 ms.

- **`engine::baseline` no longer takes 4 KB of the caller's stack.**
  Its percentile and median sorts used the stable `sort_by`, whose
  driftsort scratch is a 4 KB array on the stack; neither needs
  stability (equal `f32`s are the same value), so both are
  `sort_unstable_by` and the output is unchanged. On the CoreS3 this
  was 3.6-3.9 KB of FT4's 8 KB slot task by itself, in the
  provisional coarse pass. The board's FT4 stacks were re-sized from
  a per-stage measurement at the same time: the slot task to 10 KB
  (1 136 B free of 8 KB even after the sort fix; 3 244 B free of
  10 KB now), the two capture-time workers to 4 KB (they used ~1.5 KB
  of 8), the candidate worker left at 8 KB (3.1 KB free) — 6 KB of
  internal DRAM back overall, and every FT4 task above the 2 KB the
  board's own stack report calls tight.

- **CoreS3: `qso.adi` and `all.txt` on flash.** The activator's
  contacts and every decode now have somewhere to go that survives a
  power cut. `littlefs` moves to the unused tail of the 16 MB flash
  (6.125 MiB at `0x9E0000`; the old 768 KB partition had no component
  mounting it, so nothing is lost) and is mounted through
  `joltwallet/littlefs`, formatted on first boot. `all.txt` is
  WSJT-X's `ALL.TXT` line for line (`MainWindow::write_all`), written
  for every mode through the one call that also fills the station list
  (`storage::publish_slot` — mode and period from the boot mode, dial
  frequency from the status bar's field, so a CAT link will reach every
  mode's log at once) and rotated to `all.1.txt` at 2 MiB; `qso.adi` follows `LogBook::QSOToADIF`
  field for field, plus `MY_SOTA_REF` / `MY_POTA_REF`, and is
  `fsync`ed per record. Both formatters are pure and tested against
  the WSJT-X layout in `hosttest/mfsk-app-shared`. `http_config` can
  serve the files (`FileSource`; verified, 9 KB in 85 ms), but no
  receiver keeps a server up for it — see below.

  **All flash I/O goes through one task with an internal-DRAM stack**
  (`storage.rs`): the HTTP server and the panel tasks run on PSRAM
  stacks, and a flash operation from one aborts the board
  (`cache_utils.c:127`). Three things about it were measured on the
  FT8 SIM build before it was right, 18 slots per run:

  - **It is spawned by the first request, not at boot.** Spawned at
    boot, its 5 KB stack came out of the block the decoder allocates
    next — largest free internal block before the decode loop 40 960 →
    32 768 B — and FT8 went from 0 to 9-14 slots of 18 finishing past
    key-up, losing the weakest station (K1JT, −15 dB) from the slot
    list. Lazily spawned: 0 of 18, 7 decodes every slot. A resident
    httpd costs the same way (~4.3 KB), which is why the FT8
    controller still has none; the logs are to be fetched from a
    server started on demand, or over Web Serial. **Neither the task
    nor its channel is made on the requester's stack**, which is a
    decoder's: made there, FT4's `ft4_slot` fell to 444-464 B free of
    8 KB. The channel is made at boot, the task by the panel loop.
    The task is spawned after the first request is sent, not when the
    channel exists (which is from boot).
  - **Lines and contacts are held in PSRAM and written only at chosen
    moments**, because a LittleFS write stops the cache on both cores
    and with it everything, whatever its priority: on an IC-705 a
    mid-slot `all.txt` flush cost a skipped USB audio packet. The
    flash's auto-suspend would avoid that, but IDF detects this
    board's chip as `generic`, outside what it supports. So the write
    points are: a **quiet window** the transmit side hands over with
    `storage::quiet_window` — the tail of our own transmit slot, after
    its audio (nothing calls it until transmission exists);
    **receive-only**, once 48 KB of `all.txt` is held (~45 min of a
    busy FT8 band), at the point in the slot after every station's
    transmission has ended and before the decode starts — 13.3 s for
    FT8, 5.8 s for FT4, 112 s for WSPR — 16 KB per slot; before a
    **download**; and before a **restart** from the panel. On air the
    receive-only writes started at +13 308-13 319 ms, took 48-83 ms
    per 4 KB and cost at most one packet, in audio no signal was in;
    a mode change wrote 586 B in 16 ms before restarting. `all.txt`
    is written a block (4 KB) per `sync`: LittleFS cannot program into
    a block a `sync` has committed, so smaller synced appends
    re-wrote the tail block every time (traced with
    `MFSK_STORAGE_TRACE=1`: one erase plus 512-4 096 B of programming
    per 448 B slot, and a 147 ms metadata compaction about one sync in
    ten). A power cut loses what is held — up to 48 KB of `all.txt`,
    and the contacts since the last window, normally none.

- **CoreS3: the CQ-side QSO state machine for portable activations
  (`mfsk-app-shared::activator`), host-tested, not yet wired.** SOTA /
  POTA operation wants the board to call CQ, answer whoever calls, log
  each contact and stop at a target count, unattended. The older
  `qso.rs` could not: fixed resend counts, no notion of a caller who
  went quiet, and a reply chosen by state rather than by what was
  received. The new machine is pure — the board calls `decide` once
  per own period at the reply deadline — and follows WSJT-X
  `mainwindow.cpp` where it matters: the reply is a function of the
  caller's latest message (5820-5990), so a caller who repeats `R-12`
  gets `RR73` again without a second log entry; the contact is logged
  when `RR73` is sent (4951-4965); a caller finishing an earlier
  exchange is served before new callers (4208-4209). Two deliberate
  divergences, both commented: a silent partner is dropped after two
  periods (configurable) rather than after WSJT-X's six-minute
  watchdog, and among new callers the strongest is answered. A partner
  dropped and heard again is logged with the grid and reports of the
  first attempt. 19 scenario tests run in `hosttest/mfsk-app-shared`,
  including every message the machine can emit going through
  `pack77`/`unpack77` and back for both `JL1NIE` and `JL1NIE/P` — the
  test that found the entry above. Logging to flash, the board wiring
  and actual transmission are the next stages.

- **CoreS3: the four receivers share one boot sequence
  (`boot::Receiver`).** This crate carried four `main`s: one per app,
  plus the FT8 controller's, inline in `main.rs`. Each opened with its
  own copy of the same seven steps in its own order, so the receiver
  with the most behaviour was the one with no name, and the copies had
  drifted — the WiFi decision existed four times over (#381), the
  sequence *after* the association existed twice (`net.rs` for three
  receivers, a hand-rolled thread in `main.rs` for the FT8 controller),
  and two receivers opened the `"mfsk"` NVS namespace a second time for
  a handle the first one already held.

  `boot::run` is now that sequence —
  `prepare → spawn_workers → attach_panel → net::bring_up → start →
  run_forever` — and a receiver implements only its differences. The
  order is the part worth writing once: arenas while the heap is whole
  (155 648 B largest free internal block before WiFi and the USB host
  take theirs, 31 744 B after), then the WiFi driver's synchronous
  internal-DRAM claim, then anything that takes a stack. `main.rs` goes
  from 546 lines to 166, and the FT8 controller becomes
  `apps::ft8::Ft8Controller` beside the other three.

  **One retry policy, `Once`, for all four.** The per-receiver
  `net::Policy` is gone: WSPR retried association forever (its test AP
  needed it), FT4 and FST4 tried four times and re-campaigned after
  three minutes of quiet, and the FT8 controller stopped after four.
  All four now stop after four, because the thing the three shapes
  traded against is the same for each of them — a retry loop costs the
  decoder ~40 % of its throughput while it runs (`fst4_sync_search`
  711 → 1 380 ms per candidate, the candidate loop 33 → 54 s, measured
  2026-08-22), since the driver's task sits at FreeRTOS priority 23. A
  board that failed four times has an AP problem, and finding that out
  again three minutes later buys nothing a reboot does not.

  **What that costs**: a receiver that never associates, or that loses
  its association mid-session, does not get the network back without a
  reboot — no wsprnet upload for WSPR, no NTP, no UDP log, no config
  page. That is the trade as chosen: the decode is what the board is
  for.

  **Behaviour preserved on purpose, where it differed for a reason.**
  The FT8 controller keeps its watchdog (its `task_wdt(IDLE0)` lines
  during a cold acquisition are a watched symptom), keeps
  `WIFI_PS_NONE` (`MIN_MODEM` has never been measured there — the A/B
  is on #381), keeps giving up after four association attempts rather
  than re-campaigning (`net::Policy::Once`), and keeps *not* serving
  the HTTP config page: unifying the sequence would have handed it a
  listening socket as a side effect, which is not a refactor.
  Waiting for the UDP log sink before `usb_host_install` stays FT8's
  alone, now as `Receiver::WAIT_FOR_LOG_SINK` rather than as a flag
  three receivers happened never to reach.

  **What did change**, all of it in the FT8 controller's favour and all
  of it wanting a board to confirm: its network task moves from a
  24 KiB pthread stack in internal DRAM to `net`'s PSRAM-backed task at
  priority 2 pinned to core 1 (never core 0, which carries capture); it
  gains the UDP-sink bind retry the shared path did not have and the
  30-attempt retry it did; its NTP server comes from the settings page
  rather than a constant (same default, `pool.ntp.org`); and
  `WIFI_ENABLED` is now false when there is no SSID to try, so UAC no
  longer waits 45 s for a sink that cannot arrive. The FT4, WSPR and
  FST4 panels also show the operator's real grid source instead of
  always `ntp`, because the setting is published before the dispatch
  now.


- **FT4 runs the message codec's plausibility verdict by default
  (#383).** `Ft4::MESSAGE_FILTER_DEFAULT` was `false` for one reason:
  nobody had measured what turning it on costs. FT8 runs it because a
  CRC-14 false positive that reaches `.sic_rounds()` / `.sic_early()`
  is *subtracted* from the audio, taking whatever real signal was
  underneath with it (`qso3_busy`: 18/18 becomes 17/18 with the verdict
  off). FT4 has the same CRC-14 and the same `SupportsSicRounds`, so
  the argument always transferred — the doubt was the cost, since the
  verdict's ITU-prefix allowlist was tuned on an FT8 recording and FT4
  is a contest mode full of DX prefixes.

  Measured on the `ft4sim` corpus, 720 slots across the threshold
  window (−21..−13 dB, four ITU-R channels, 20 trials a cell):

  | | verdict off | verdict on |
  |---|---|---|
  | golden rows | 353 | **354** |
  | phantom rows | 7 | **2** |

  35 of the 36 recall cells are identical and the one that moves goes
  **up** (`awgn −17 dB`, 16/20 → 17/20) — a rejection lets the
  candidate ladder keep going and reach a decode a phantom had taken
  the slot from. All four 50 %-crossing SNRs are unchanged to
  **0.00 dB** against `sweep-baseline.json`, so the baseline is not
  refreshed: the curve did not move.

  **What the corpus cannot say**: every slot in it carries the same
  callsign, so the allowlist's own risk is not exercised by it. The
  WSJT-X golden recording is (`ft4_message_policy`, where the verdict
  drops nothing, half of it ARRL RTTY Roundup), and a deployment that
  needs more widens it with `.also_accept()`.

  Two measurement harnesses land with it, both `#[ignore]`d:
  `MFSK_FT4_SWEEP_CODEC_FILTER=1` runs `ft4_snr_sweep` with the verdict
  on, and `ft4_phantom_rate` counts every row the corpus produces
  rather than only the golden one — which is what makes the phantom
  column above measurable at all.

  **FST4 stays off**, and not for want of measuring: CRC-24 puts its
  false-positive rate 512x below FT8's and FT4's.

- **CoreS3: `WIFI: ON` / `WIFI: OFF` on the CONFIG page, and every
  receiver honours it (#381).** `main.rs` decided from the boot mode
  alone, so a board set to `TIME: AIR DT` — which means "no
  infrastructure here, take the phase off the band" — still ran a full
  association campaign at boot. That campaign is four attempts three
  seconds apart, and while the driver hunts for an AP that is not there
  it runs at FreeRTOS priority 23, above anything this app creates:
  ~40 % of decoder throughput when it was measured (`fst4_sync_search`
  711 → 1 395 ms per candidate, 2026-08-22). It lands on the first
  slots after boot, which are exactly the ones a cold `AIR DT`
  acquisition needs.

  **Not derived from the grid source**, which was the obvious shape and
  the wrong one: the time source says where the *phase* comes from,
  this says whether there is a *console*. A hilltop with a phone
  hotspot wants `AIR DT` and a log both.

  The setting is read and published **before** `main` dispatches into
  WSPR, FST4 or FT4 — three of the four receivers never return from
  that dispatch and each brings up its own WiFi, so a setting read
  after it would have been a switch that does nothing in three of the
  four places the operator can see it. `wifi_pref` defaults to `On`, so
  a board that has never been set behaves exactly as before.

  What `WIFI: OFF` takes with it is said in the manual and in the
  module docs: in FT8 (UAC) mode WiFi is the only console, since the
  USB host driver has taken USB-Serial-JTAG; it also takes NTP, and in
  WSPR mode the wsprnet upload. The way back is the panel, which still
  works.

  Also folded: `main.rs` carried the same `WIFI_SSID empty` warning
  twice, so it printed twice.

- **The message-acceptance policy reaches `Ft4` and every FST4
  sub-mode, and five places said `Ft8` alone (#383).** No code changed:
  PR #386 gave `engine::pipeline` an `InfoAccept` seam and implemented
  `SupportsMessageFilter` for both, 14 commits before v0.11.0 was
  tagged. What did not change was the prose PR #385 had written while
  FT8 really was the only implementor — the trait's own doc comment,
  FT8's impl beside it, `LIBRARY.md`'s builder table and its `.ja.md`
  twin (which also still typed the closures `Fn(&str)`, omitted
  `.codec_filter()` and linked §2.6 as "§2.5"), and 0.11.0's own
  CHANGELOG entry, corrected in place with a note that the copy
  published to crates.io still carries the old text.

  `FrameDecodable::MESSAGE_FILTER_DEFAULT` is unchanged and still
  `true` for FT8 alone, so FT4 and FST4 decode bit-identically unless a
  caller asks for a policy. Whether FT4 *should* default to the codec
  verdict — it has FT8's CRC-14 and `SupportsSicRounds`, which is the
  reason FT8 does — is a measurement nobody has run.


- **Breaking, for code that implements `Protocol` or calls the Δt
  search directly: `Protocol::SyncPhasors`.** The Δt search's
  precomputed phasor tables used to reach it as
  `Option<&Ft4CoarsePhasors>` through a second entry point — one
  protocol's concrete type in a signature every protocol shares, and a
  fine pass that could not use the tables at all, since an argument
  cannot answer for a `df` the caller has not reached yet (it rebuilt
  nine `df` from `cos`/`sin` four times each). They are now an
  associated type: `Ft4` holds `Ft4CoarsePhasors`, every other protocol
  `()`, which answers every lookup with "build it" — what they all did
  before. **An out-of-tree `Protocol` impl needs
  `type SyncPhasors = ();`**, and `ft4_sync_search_window_with` takes
  `&P::SyncPhasors` where it took the `Option`. Decodes are unchanged
  (tier A+B).

- **FT4 per-candidate cost, found by instrumenting the board rather
  than the bench.** `ft4-bench`'s LLR/BP probe had attributed its whole
  "rest" to BP (`FT4_BENCHMARK.md` §38: "87 % is BP"). Timed in place on
  a CoreS3, more than half of it was `freq_shift_cd0` — 13.6 ms of a
  23.5 ms tail, evaluating `cos`/`sin` per sample and allocating its
  output every call. It now writes into a caller's buffer
  (`freq_shift_cd0_into`) through `engine::dsp::ddc::Mixer`, the NCO
  every other rotation in the crate already used. Beside it, all
  bit-identical and measured on the new FT4 host mirror
  (`tests/ft4_embedded_pipeline_mirror.rs`, which reproduces the board's
  12 candidates and 11 decodes call for call and counts allocations):
  `fine_sync_power_per_block` lends its Costas reference instead of
  cloning it on every cache hit (42 allocations a candidate → 0; the
  cache now also hits for FT4's four distinct patterns),
  `build_group_amplitudes` writes into reused storage
  (`compute_llr_fast` 94 allocations → 8), `SlotDecimator` owns its
  staging chunk (−317 allocations and −1.30 MB a slot on the capture
  side), and `ft4::ddc` sizes each FIR stage's history margin to stay
  under the CoreS3's 2 KB internal-DRAM threshold.

  Two cheaper front ends were built behind knobs and measured losing,
  and are kept for the record: `candidate_baseband_boxcar`
  (`MFSK_FT4_BOXCAR`; 4.4x faster DDC on the board, 0.29-0.50 dB of
  threshold and one of the golden's eleven decodes) and
  `ft4_sync_search_window_binned` (`MFSK_FT4_BINNED_SEARCH`; a quarter of
  the multiply-adds, no faster on the board, where the shipped dot
  product already runs on esp-dsp's PIE path).

- **FT4 on the CoreS3 answers by the reply deadline: 11 of 11 golden
  decodes inside 1 025 ms.** The deadline is the moment this station's
  audio must start — WSJT-X's FT4 modulator starts at the boundary +
  300 ms (`Modulator.cpp:74`), and the board keys the IC-705 by VOX on
  its USB audio, so the message has to exist then and not before
  (`EMBEDDED.md`, "FT4: the reply is due when the audio starts").
  `TX_TURNAROUND_BUDGET_MS` is now `REPLY_DEADLINE_MS`, 1 025 ms after
  capture close; it was 1 225, which put the audio at 8.0 s and the
  frame ~200 ms late at the other end. Decodes that land later are still
  kept for the screen.

  What made it fit is moving work into capture time, on both cores,
  with nothing about the decode changed. At Costas block D's end
  (5.444 s into the window) the receiver reads a provisional candidate
  list from a snapshot of the running periodogram
  (`Ft4SavgBuilder::snapshot`), snaps each carrier to its bin, and
  streams the half-rate audio into one `CandidateDdc` per carrier as it
  arrives — bit-identical to building it after the close, because every
  `FirStage` carries its own history. The coarse Δt/Δf sweep runs
  alongside it (`engine::sync2d::Ft4CoarseSweep` / `Ft4SweepScratch`):
  a cell's score is four Costas-block correlations, each readable the
  moment its 128 samples exist, and on a 5.04 s frame every cell of the
  ±1.0 s window is readable before the close. At the close only each
  baseband's last few percent and flush, the rest of the sweep and the
  fine pass remain. On the golden every candidate's search result is
  bit-identical to the one-shot search, score included, at any chunking.
  Buffers held across the capture are allocated just over the 2 KB
  threshold (`CandidateDdc::new_half_rate_with_min_alloc`,
  `FirStage::new_with_min_alloc`) so twelve of them do not sit in
  internal DRAM; the host mirror's small-allocation peak falls from
  31 536 B to 24 517 B.

  CoreS3 SIM, golden slot: the candidate loop ends at ~700-790 ms
  (from 1 300-1 600), with 11 of 11 decodes by 1 025 ms.

- **docs.rs builds `full`, and the release job waits for crates.io and
  docs.rs.** docs.rs built with `all-features`, which enables the
  mutually exclusive `wspr-ddc` / `wspr-ddc-cascade` pair and stops at
  their `compile_error!` — so 0.10.0, 0.10.1 and 0.11.0 published with
  no documentation, and nothing said so. `[package.metadata.docs.rs]`
  now lists `features = ["full"]`, and `release.yml` ends only once
  crates.io serves the new version and docs.rs reports its build — the
  first release to exercise that is this one.

- **CoreS3: one screen for every mode.** WSPR and FST4 had spot-list
  screens of their own; every mode now shows FT8's panel — status bar,
  waterfall, station list, link bar, menu — and five `mfsk-app-shared`
  spot-list modules and `spot_panel.rs` are gone. Where a mode's decode
  holds core 0 (WSPR, FST4) the panel runs on a core-1 PSRAM-stack task
  (`display::spawn_log_panel`), so its menu commits now go through the
  PSRAM-safe `commit_and_restart` helpers.

  The waterfall is drawn from the audio, the same way in every mode
  (`embedded_shared::waterfall` + `waterfall_feed`): the audio paths
  append to a ring without ever blocking, and the panel builds rows on
  esp-dsp — a real 2 048-point transform as 1 024 complex points through
  the radix-4 PIE kernel and `dsps_cplx2real_fc32` — 6 rows a second over
  200-3 000 Hz (FT8's decode band; its old rows stopped at 2 700). Slot
  rules come from the clock at the mode's period
  (`BootMode::slot_period_ms`). FT8's stage-1 row queue and its
  `wf_drain` task are gone, as are FT4's periodogram rows.

  The station list has one entry point, `UiState::publish_slot`, and
  the panel decides which rows are green: heard within one slot period,
  by the row's own time (`decoded_current_iter`, `render_in_flags`).
  Receivers no longer keep a watermark — FT4, WSPR and FST4 never moved
  it, so their new stations were never highlighted. The StickS3 and
  Core2 keep the watermark path unchanged.

- **CoreS3: the panel no longer stops for an FT8 decode.** It ran at
  `main`'s priority 1 and stood still 1.2-1.4 s every FT8 slot. Running
  it above the decode was unaffordable as it was — 16-19 % of core 0,
  measured with FreeRTOS run-time counters, which this adds
  (`CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS`, a `[cpu]` line every
  10 s). What made it cheap: SPI2 with DMA and interrupt-driven
  transfers (esp-idf-hal's defaults, no DMA and `polling: true`, spin
  the task through every byte), the waterfall sent as 4-row RGB565
  blocks instead of `fill_contiguous`'s per-pixel iterator (~375 SPI
  transactions a redraw), the status and link bars drawn only when
  their content changes (126-178 → 23-31 ms/s), and 6 frames a second.
  `display::PANEL_PRIORITY` (7) then applies in FT8 mode: 7 decodes, 6
  in time, 0 cut with the panel at 1 or at 7 on FT8 SIM, longest frame
  gap 1.2-1.4 s → 0.18-0.35 s. **FT4's panel stays at 1**: its reply
  deadline is the tight one, and above the decode the panel cost it
  ~200 ms of loop end.

- **CoreS3: clock and grid fixes found on the way.**
  - The RTC read and write-back ran in the panel at priority 1, below
    the decoders, and place a clock edge by polling; a preemption
    between the two halves put the edge late by its length. FT4 boots
    measured 0.9-3.1 s of NTP correction, FST4 ones none. Both now run
    at priority 7 for their duration, and the read sets the tick plus
    the time since the tick was seen. After the fix: tick seen 11-15 µs
    before the clock is set, STOP released 265-407 µs after the
    boundary, and the next boot within ±50 ms of NTP.
  - FT4 stopped steering its grid from decode DTs: the DT-median trim
    and the clock re-anchor pulled opposite ways and fired on every
    slot. `observe_slot_phase` stays as a readout.
  - The live grid phase is persisted every 20 slots while the air owns
    the grid, so FT4 inherits a fresh one from FT8; it used to be written
    only by a cold acquisition.
  - The esp-dsp FFT keeps one twiddle table per length, built once and
    never rewritten, so no transform holds a lock across its body.
  - The SIM harness feeds the real sink, aligns to the clock the
    receiver will use (not a stale one surviving the reset), and
    re-aligns at every pass so an NTP step no longer strands it.
    Measurement knobs: `MFSK_CORES3_FORCE_GRID`, `MFSK_SIM_CLOCK_STEP_MS`,
    `MFSK_WF_FEED_OFF`, `MFSK_FT4_SINGLE_CORE`.

## 0.11.0 — a new C ABI for every mode (breaking), FT4/FST4 a-priori decoding fixed (−1.1 dB AWGN), the sniper becomes FT8-only (breaking), a caller-supplied decode budget, `mfsk-ffi-ft8` retired, the CoreS3 FT8 receiver holds its slot grid and stops starving its own internal DRAM

**Why a minor bump.** Two independent reasons, either of which would be
enough by this crate's own convention — the precedent is `0.7.0` (the
generic `decode_frame_for::<P>` API) and `0.10.0` (three public
search-parameter type changes), both structural or breaking rather than
merely capable.

**1. The public API breaks**, and the two halves break very
differently:

*`mfsk-core` (the crates.io crate) breaks in two places.*

*The search API.* `SniperRequest` is gated on a new `SupportsSniper`,
implemented for `Ft8` alone, so `DecodeRequest::<Ft4>::sniper` and the
five FST4 equivalents no longer exist; `SniperRequest` itself is
`ft8`-gated. `ProtocolMeta` gained fields, which breaks a struct
literal but not a read.

*The message codecs* (#383). `Wsjt77Message`'s `Unpacked` is
`Wsjt77Fields` rather than `String`, `MessageCodec::is_plausible`
takes the unpacked value instead of the payload bytes,
`is_plausible_message` is gone, and `CallsignHashTable::lookup22`
returns `&str`. The full list is the table under **Added** below.

**A consumer that decodes and renders needs no edit** — `unpack77` and
`unpack77_with_hash` still return `Option<String>`, and the wide-band
decode API and every protocol's entry points are untouched. One that
asked the codec a *question*, or read a hash back out of the table,
does. (An earlier draft of this paragraph said the message codecs were
untouched and that a non-sniper consumer saw no change at all. Both
were wrong, and both contradicted this release's own #383 entry.)

*`mfsk-ffi` is rewritten.* Every pre-v2 decode symbol is gone, along
with `MfskProtocol`, `MfskResult`, `MfskResultList`, `MfskSamples` and
the opaque options handle with its eight setters. It is `publish =
false` and its only exercised consumer was the in-repo C++ driver, so
the blast radius is smaller than the diff suggests — but a C consumer
porting across will rewrite, not adjust. `docs/reference/LIBRARY.md` §8
is the map. `mfsk-ffi-ft8` is retired outright.

**2. Sensitivity moves, for the first time in several releases.** FT4
and FST4 a-priori decoding had been locking about half its bits to the
*opposite* of the truth since it was written, and neither ran the blind
CQ pass WSJT-X runs on every decode. FT4's AWGN threshold goes
**−16.89 → −18.00 dB**, from 0.6 dB behind WSJT-X's published figure to
0.5 dB ahead. FST4 is unchanged across all twenty sweep cells and FT8
across all four — both measured rather than assumed, with the reasons
recorded in `FST4_BENCHMARK.md` §16 and `FT4_BENCHMARK.md` §48-49.

**What is new rather than changed**, in one line each: a C ABI where
FT8, FT4 and all five FST4 sub-modes are addressed, configured and
reported identically, with capabilities published rather than guessed;
a caller-supplied wall-clock budget for FT8/FT4/FST4; maintained
Kotlin **and Swift** bindings; and Windows/Android cross-compilation
checked on every PR. The Swift package is verified by running its own
suite on a Mac rather than by CI, which still has Linux runners only.

**The CoreS3 receiver is the other half of this release, and it does not
touch the crates.io crate.** Every one of those changes is in
`embedded-poc/` and the two operator manuals; a `mfsk-core` consumer sees
none of it. What it amounts to is that the board now holds its slot grid
against a radio for half an hour without intervention — 102 slots over
29 minutes on 40 m, mean 8.2 decodes, no empty slot, no acquisition and
no trim — where before it walked off the air within minutes of a good NTP
sync. Three independent bugs were in the way, each of which presented as
the decoder's fault: 6.5 % of the USB audio was never delivered, the RTC
was a second slow on every write, and the phase correction clamped
itself in the one direction it could not be right in. The instrument
that should have caught the second read the board's own clock, so it
reported the grid healthy while the band said otherwise.

### Documentation

- **Tier C ran before this tag, and not one of the 28 groups moved.**
  FT8, FT4 and all twenty FST4 sub-mode/channel pairs reproduce their
  stored 50%-crossing SNR to the digit, so
  `docs/notes/sweep-baseline.json` changed only in provenance. That is
  the expected result rather than a lucky one: the sweeps use fixed
  seeds, and this release's decode-path changes are either
  default-identical by construction (`DefaultPolicy::CAN_REJECT` and
  `MESSAGE_FILTER_DEFAULT` are both `false` for FT4 and FST4, so
  `PolicyAccept::accept` folds to `return true` at compile time) or on
  paths these corpora never reach (FT8's `fine_sync_12k`,
  partial-Costas and chunked-GMFSK work is streaming and transmit).

  FST4 need not have been in the run. `release-status.sh` listed it
  because two commits touched `mfsk-core/src/engine`, and the warning
  it prints beside its own suggestion — "sharing a module name is not
  sharing a code path" — is the one that applied. Issue #280's lesson,
  relearned.

- **`BENCHMARKS.md`'s FT4 row was stale against this release.** It
  carried AWGN ≈ −16.9 dB, written 2026-07-30, while this release's own
  headline is the a-priori fix taking FT4 from −16.89 to −18.00 dB —
  the summary table pointed at the number the release exists to have
  moved. `FT8_BENCHMARK.md` §9's figures were stale by up to 1.1 dB in
  the more-sensitive direction (CCIR moderate −18.9 → −20.00 dB); §9 is
  correct as a record of 2026-07-26 and is left alone, with the new §12
  recording the pre-tag measurement and saying plainly that this pass
  did not establish which commits did it.

  `FT8_BENCHMARK.ja.md` stopped at §6, so a reader arriving in Japanese
  saw §4's figures — by then up to 1.8 dB stale — and none of the four
  investigations that moved them. §7 through §12 are translated; the
  section counts now match.

- **Fourteen sites still named `mfsk-ffi-ft8` in the present tense.** The
  crate was retired earlier in this release and the reference manuals
  were swept, but outside them four sites pointed a reader at files that
  are not there (`mfsk-ffi-ft8/src/stream.rs`,
  `mfsk-ffi-ft8/Cargo.toml`) and one at a type that no longer exists
  (`MfskFt8Stream`). `embedded-poc/CLAUDE.md` was wrong twice over,
  describing `idf-component/` as a shim "so C-only ESP-IDF projects can
  pull the FT8 decoder in without writing Rust glue" — that directory has
  been documentation-only since the crate went, and pure C cannot satisfy
  the `extern "Rust"` FFT-planner symbol, which is why the crate was
  retired in the first place. Also corrected: `engine/dsp/resample.rs`
  and `mfsk-ffi`'s module docs, an orphaned `embedded-shared/Cargo.toml`
  comment about a `wav_sim::spawn` signature that changed long ago, seven
  `ft8/decode_block*` caller lists (only the dead name dropped — the
  `pub` those lists justify is still reached by `embedded-shared` and the
  integration tests), both platform tables in the ハムフェア2026 slides,
  and `PHASE_D_PIE_SIMD.md`. `CHANGELOG.md`, `docs/historical/` and
  `ROADMAP.md`'s release-history sections are deliberately untouched,
  where the crate is a correct record of what a past release did.

- **"AP is not part of it" read as "AP does not work here", and three
  source doc comments still described the pre-refactor world.** Raised by
  a reader who could not work out why sniper mode had no AP. It does:
  `SniperRequest::ap_hint()` exists, is gated on `SupportsSniper` plus
  `P::Msg: WsjtApCompatible`, and `__sniper` passes `req.ap_hint` into
  `decode_sniper_inner`. `LIBRARY.md` §2.2's own method table listed it
  sixty lines above the sentence denying it.

  The sentence meant "AP is not part of what sniper *is*" — the ±250 Hz
  window is what makes it the sniper, and AP is an orthogonal option that
  also reaches the wide-band `DecodeRequest`. Reworded in `LIBRARY.md`,
  its twin and `CLAUDE.md` to say that, with the availability stated
  outright. `DESIGN_RATIONALE.md` §3 gains a line saying it is about
  where AP came from, not whether it is available.

  The 2026-09-13 refactor also left four doc comments in
  `msg/decode_request.rs` describing the world before it:
  `SupportsWideBandAp`'s own doc said "FT8 only" while the trait is
  implemented for `Ft8`, `Ft4` and every FST4 sub-mode; it and
  `DecodeRequest::ap_hint` both routed FT4/FST4 to AP "through
  `SniperRequest::ap_hint`", which `SupportsSniper` being FT8-only makes
  impossible; `SniperRequest::ap_hint` called its own location "an
  accident of implementation ... for FT4/FST4", protocols that cannot
  reach that builder. That first comment is what this crate's own
  documentation pass read when it wrote `.ap_hint()` as FT8-only in the
  method table — the stale comment propagated once before a review caught
  it, and would have again.

  The sniper-plus-AP path is pinned by `LIBRARY.md` §2.2's doctest, which
  builds a `SniperRequest`, sets `.ap_hint()` and asserts a decode.

- **The FFT extern symbol was documented under the wrong name, and the
  ESP-IDF template still described a retired crate.** Both halves of the
  embedded linking contract were wrong in ways a consumer only finds at
  link time.

  `EMBEDDED.md` and its twin spelled the i16 planner factory
  `mfsk_core_make_default_fft_planner16`; the symbol
  `engine::fft::default_planner_16` actually looks up — and that
  `embedded-shared/src/esp_dsp_fft.rs` defines — is
  `mfsk_core_make_default_fft_planner_16`, with an underscore. Copying
  the documented block gave an undefined-symbol error with nothing
  naming why. Four sites, two languages.

  `embedded-poc/idf-component/README.md` was written around
  `mfsk-ffi-ft8`, retired in 0.11.0, and described a file tree
  (`CMakeLists.txt`, `main/`, `components/`, `shim/`) that no longer
  exists — the directory is that README alone. It is rewritten against
  the current shape: why a Rust staticlib shim is required whatever you
  link (`extern "Rust"` and `Box<dyn Trait>` are not things C can
  define), the two linking options and when each is smaller, the shim's
  `Cargo.toml`, the CMake `IMPORTED` component with the
  `espressif__esp-dsp` requirement that makes the ASM kernels reachable,
  and the `-C panic=abort` the Xtensa build needs. It now says plainly
  that it is documentation rather than a buildable skeleton, so nobody
  goes looking for the files.

- **The reference manuals are split by audience, and the C ABI is
  rewritten from the header.** `LIBRARY.md` had grown 1667 → 2138 lines
  in a week across 16 commits that added 578 lines and removed 96;
  every PR appended its section and nothing was retired. With 0.11.0's
  breaking FFI rewrite that left §8 — the section this CHANGELOG points
  C consumers at as "the map" — wrong in both directions: ~20 documented
  symbols absent from `mfsk-ffi/include/mfsk.h` (`mfsk_decoder_new`,
  `mfsk_decode_options_new` and its eight setters,
  `mfsk_decode_{i16,f32}_sniper`, `mfsk_result_list_free`,
  `mfsk_samples_free`, `MfskProtocol`, `MfskResultList`, `MfskSamples`)
  and ~40 real ones undocumented, including 13 of the 14
  `mfsk_session_*` functions. `mfsk_session` appeared three times in
  2138 lines.

  New `docs/reference/BINDINGS.md` (+ `.ja.md`) is written from the
  header: session lifecycle, `MfskDecodeParams` / `MfskDecode` field by
  field, streaming capture, transmit, introspection with the capability
  bits, the bespoke per-mode entry points, a pre-v2 → v2 porting table,
  and a grouped index covering all 63 exported functions.

  `LIBRARY.md` is the Rust API reference again (2138 → 1081 lines),
  ordered for lookup: a table of contents, the runnable FT8 example
  moved from line 1249 to the top, and `DecodeRequest`/`SniperRequest`
  as a method table rather than one 30-line bullet.

  `EMBEDDED.md` drops two engineering diaries and a retired crate
  (2009 → 674). The 782-line FST4 attempt log and the 285-line FT4 one
  move to `FST4_BENCHMARK.md` §17 and `FT4_BENCHMARK.md` §50; 266 lines
  documenting `mfsk-ffi-ft8` — named 52 times per language though the
  crate was removed in 0.11.0 — become a short account of the Rust shim
  an ESP-IDF consumer actually needs. `## Where to go next` moves from
  line 1985 of 2009 to the top. A per-protocol embedded status table is
  new: FT8, FST4, FT4 and WSPR all decoding off the air.

  `docs/notes/DESIGN_RATIONALE.md` collects the argued passages with
  their measurements; `UAC_BRINGUP_CORES3.md` keeps #163's closed
  bring-up checklist.

  Both `.ja.md` twins follow, heading for heading. **`on_result`
  appeared ten times in `LIBRARY.md` and zero times in
  `LIBRARY.ja.md`** — streaming delivery had never been translated, so
  a Japanese reader could not learn from that document that it exists.

  Also corrected across the tree: dependency snippets saying 0.8;
  `SniperRequest::<Ft4>` still promised to work "the same way" when
  `SupportsSniper` is implemented for `Ft8` alone; the workspace table
  listing `mfsk-ffi-ft8` as live and omitting `hosttest/mfsk-app-shared`;
  a module tree missing six `engine/` files; feature tables omitting
  every no_std and FFT flag; and `CONTRIBUTING.md`'s layout block plus
  four `Cargo.toml` comments still naming a `core` module that does not
  exist. Section-number citations in `src/`, `tests/`, `release.yml`,
  `README.md` and `CLAUDE.md` are renumbered to match.

### Added

- **`hash-table-small`: the callsign hash table in 7 KB instead of
  83 KB.** Off by default — host keeps WSJT-X's shape — in the same
  "host stays faithful, embedded opts into the cheaper path" split as
  `wspr-pass2-topn` and `wspr-fano-cap-fast`.

  Upstream keeps three tables and can afford to: `packjt77.f90:5-6`
  indexes the 10- and 12-bit ones by the hash itself, 1 024 and 4 096
  slots, so a lookup is an array read and no key needs storing. The
  price is that they are sized by key space rather than by traffic. A
  receiver hears on the order of 100 distinct callsigns in 15 minutes
  — measured, CoreS3 on 40 m — so 97 % of those 66 KB hold nothing.
  With the feature on, each callsign is stored once beside its three
  hashes in one 256-entry table and looked up by scanning it.

  **What it trades is eviction policy, not capacity.** A
  direct-indexed slot keeps its callsign until a colliding hash
  overwrites it, which may be never; one shared LRU evicts by
  recency, so all three widths forget a station together and an old
  one renders `<...>` again. The 10- and 12-bit hashes appear in
  DXpedition and Type-4 messages, which name the station being worked
  *now*, so the depth that matters should be recent — but **how often
  this costs a resolution has not been measured on air.** The feature
  doc in `Cargo.toml` says so and names the instrument.

  256 entries and not 100 because of an allocator threshold: one
  entry is 28 B, so 100 would be 2.8 KB, under the CoreS3's
  `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096` and therefore back in
  the internal DRAM this exists to stop consuming. A test asserts the
  threshold rather than a comment mentioning it.

- **`unpack77` decodes to fields, and the message verdict reads them
  (#383, breaking).** `unpack77_fields` returns `Wsjt77Fields` — one
  variant per message type, carrying the decoded fields — and
  `Display` renders it, byte for byte as before.
  `Wsjt77Fields::callsigns()` yields exactly the callsign fields, and
  `Wsjt77Fields::is_plausible()` is the verdict.

  The old shape decoded a message into fields and then threw them
  away, so anything asking a question of it had to split the rendered
  string back into tokens and guess which ones were callsigns. Judging
  `JA1ABC 3Y0Z 6A EMA` that way tests `6A` and `EMA` against a
  callsign grammar, and judging `JA1ABC PM95 20` tests `PM95` and
  `20` — which is why the filter refused three whole message types.
  Fields have no such ambiguity, and everything a text rule might
  re-check (the ARRL section index, the grid bounds, the RTTY exchange
  range) was already enforced during unpacking.

  **Breaking surface**, all of it in `msg`:

  | before | after |
  |---|---|
  | `pub fn is_plausible_message(&str)` | removed — use `is_plausible_payload` or `Wsjt77Fields::is_plausible` |
  | `<Wsjt77Message as MessageCodec>::Unpacked = String` | `= Wsjt77Fields` (`Display` for the old rendering) |
  | `MessageCodec::is_plausible(&[u8])` | `is_plausible(&Self::Unpacked)` |
  | `.also_accept(f)` / `.message_filter(f)` taking `Fn(&str)` | taking `Fn(&Wsjt77Fields)` |
  | `CallsignHashTable::lookup22 -> Option<String>` | `-> Option<&str>`, and no longer `<>`-wrapped |

  `DecodeRequest` and `SniperRequest` gained a third type parameter,
  `Pol: MessagePolicy = DefaultPolicy`. It is defaulted, so
  `DecodeRequest<'_, Ft8>` still names the same type and no type
  position needs an edit.

  `unpack77` and `unpack77_with_hash` are unchanged and still return
  `Option<String>`, so a caller that only wants the rendering needs no
  edit.

  One incidental fix fell out: `packjt77.f90:616`'s "a CQ cannot name a
  hashed station" refusal was a string-prefix test, and the RTTY
  Roundup's optional `TU; ` prefix shifted the message far enough to
  defeat it. Asking the fields has no blind spot.

- **`DecodeRequest::also_accept(f)` / `.message_filter(f)` /
  `.codec_filter()` — a caller-supplied message-acceptance policy
  (#383).** `MessageCodec` gained `is_plausible`, the codec's own
  verdict on a decoded message, and a request now says what to do with
  that verdict: keep it, apply it explicitly with `codec_filter`,
  widen it with `also_accept`, or replace it with `message_filter`.
  The filter is mfsk-core's own — `ft8b.f90` gates on `nbadcrc` and
  `nharderrors` alone — so it is a judgement call, and the judgement
  belongs to whoever knows the band. `LIBRARY.md` §2.6 (and its
  `.ja.md` twin) is the writeup.

  **This API exists because of #373**, @madmedicnl's proposal to teach
  the decoder WSJT-CB's callsign grammar. A Cargo feature was the
  wrong shape for a per-operator policy, and #383 was written as the
  answer to that; wiring the resulting hook up is what then exposed
  the message-filter bug below. The CB grammar itself stays in its
  author's hands — a dialect is application policy — but the seam it
  asked for is here, and it is a better seam than the one first
  proposed to them.

  **A type parameter, not the `&'a dyn Fn` shape `.on_result()` and
  `.budget()` use.** Those fire once per decode; this fires once per
  candidate that reaches the text stage, so the default has to cost
  nothing: `DecodeRequest<'_, P>` still means
  `DecodeRequest<'_, P, DefaultPolicy>`, whose policy field is
  zero-sized (asserted at compile time) and whose verdict inlines to
  the bare codec call the ladder already made. `ft8_message_policy`
  decodes the reference recording with and without a no-op policy and
  compares whole result rows.

  **`Ft8`, `Ft4` and every FST4 sub-mode**, gated on
  `SupportsMessageFilter` — a compile error on a protocol that has no
  place to apply one, rather than a builder method that silently does
  nothing. The gate is not a statement about the codec (all three share
  `Wsjt77Message`, so the verdict means the same thing for each) but
  about the pipeline, and the two pipelines reach the message text from
  opposite sides of the `engine` / `msg` boundary. FT8's bespoke engine
  forms the string inside the per-candidate ladder, where a rejection
  lets the ladder keep going. The generic engine FT4 and FST4 share
  returns information bits and never forms a string — `engine` does not
  depend on `msg` — so it reaches the policy through a new `InfoAccept`
  seam, with `PolicyAccept` on the `msg` side doing the unpacking.

  **This entry said "FT8 only" at the moment 0.11.0 was tagged, and it
  was already wrong when it did**: the seam merged 14 commits before
  the tag and nothing came back to update the entry. Corrected
  2026-09-21; the copy published to crates.io with 0.11.0 still carries
  the old text. What *is* FT8-only is applying the codec's verdict with
  no caller involved — `FrameDecodable::MESSAGE_FILTER_DEFAULT` is
  `false` for FT4 and FST4, because turning it on there is a behaviour
  change nobody has measured against their sensitivity curves yet.

- **`unpack77` decodes every message type `packjt77.f90` defines
  (#383).** Three were missing and returned `None`, which is a dropped
  decode rather than a rejected one: telemetry (`i3=0, n3=5`), the
  WSPR types (`i3=0, n3=6`) and the EU VHF contest exchange (`i3=5`).
  ARRL Field Day (`i3=0, n3=3|4`) decoded only its two callsigns and
  rendered the exchange as a literal `[FD]`; it now reads
  `CALL CALL [R] <ntx><class> SEC` against the 86-entry ARRL section
  table. Only `0/2` and `i3 >= 6` are still refused, which is what
  upstream does with them.

  Type 5 needed a second grid unpacker. What this crate called
  `to_grid6` is a port of upstream's `to_grid` (base 25, with
  `j5 == j6 == 24` as the four-character sentinel); type 5 uses
  upstream's actual `to_grid6` (base 24, no sentinel, 18 662 400 valid
  codes in a 25-bit field). The old function is renamed `to_grid` and
  the real `to_grid6` added beside it.

- **The per-type validity checks `unpack77` was missing, and a
  measurement of what each stage actually removes (#383).** A CRC
  false positive's information bits are uniform, so `#[ignore]`d
  `phantom_survival_rates` runs 2 M uniform 77-bit payloads through
  `unpack77` and the codec verdict and prints the survival rate
  per `(i3, n3)` cell, plus what each mode's own pre-gate would remove
  on top — `msk144decodeframe.f90:103`, `ft8b.f90:510-511`, and the
  nothing that `ft4_decode.f90` and `fst4_decode.f90` apply.

  It showed that `ft8b.f90`'s gate removes **zero** survivors: all
  three of its conditions are already refused inside `unpack77`
  itself (`packjt77.f90:330`, `:457`, `:613`). MSK144's removes 28 %,
  and is already ported.

- **CoreS3 FT8: the coarse search window is now the emit point's
  coverage ceiling, and clamped to it.** `stage1_inc` emits at pair 87,
  filling rows 0..173; block 2's last Costas symbol is at row 162, so
  0.88 s is the widest lag that lands it inside real data. The window
  shipped at `1.0`, which `jz = round(lag / 0.08)` turned into 13 rows
  — ±1.04 s — and searching past the ceiling does not fail loudly:
  unfilled rows are present and zero, they add nothing to either sum,
  and a self-normalising ratio over fewer Costas symbols competes on
  equal terms with one over all of them. Swept on the host mirror over
  every distinct width the row grid allows (fixed-point, 51 capture
  phases per recording, `mirror_partial_block2_policies`) — decodes
  331/88/87 at ±0.80, **337/118/115 at ±0.88**, 335/116/112 at ±0.96,
  335/112/112 at ±1.04 on `qso3_busy`/`qso1`/`qso2`: a maximum on all
  three, no distinct station lost, and 23 lag steps instead of 27, i.e.
  15 % off coarse sync (100-180 ms a slot on this board). On a radio,
  30 slots at 9.97 ± 1.80 against 63 baseline slots at 8.54 ± 1.93 —
  but that is a before-and-after on an opening band, and a
  block-alternating harness (`MFSK_FT8_LAG_AB`) put the window's own
  share at +0.50 over 24 slots, agreeing with the host sweep rather
  than the +1.43. What is not a statistic: deferred candidates went
  from 1.27 a slot to 0, because the ones that needed the whole slot
  were the ones at the extremes of the old window.
  `docs/reference/EMBEDDED.md` "The coarse search window is a dependent
  variable" has the derivation; `decode_pipeline` clamps rather than
  trusting the two to agree, because the version of this invariant that
  was only written down is the one being fixed.

- **`ft8::decode_block::PartialBlock2`** names what a streaming caller
  does with a lag whose Costas block 2 is not fully covered — `Score`
  (WSJT-X's own behaviour, and what ships) or `Gate` (refuse the lag),
  with `coarse_sync_with_lag_and_partial` /
  `coarse_sync_with_allsum_lag_and_partial` to select it. It replaces
  an `Option<usize>` that said nothing about why, and exists so the two
  can be compared on one slot. Default behaviour is unchanged to the
  bit. A third variant that shrank a partial score toward the noise
  floor by `sqrt(N / N_full)` was written, measured and deleted: under
  `fixed-point` the widest lag a ±1.0 s search reaches still keeps 6 of
  7 symbols, so the correction is `sqrt(20/21)` = 2 %, and it changed
  no decode count on any fixture at any phase.

- **CoreS3 can transmit audio to the radio** (`MFSK_CORES3_TX_PROBE=1`,
  amplitude 0 by default — a probe, not yet a transmit path). A whole
  FT8 frame is synthesised a chunk at a time by
  `engine::dsp::gfsk::GfskStream` and written to the IC-705's USB audio
  OUT interface: 632 chunks of 20 ms in 12 603 ms against 12 640 ms of
  audio, twice, identically. See **Fixed** for the format constraint
  that had to be found first, and `docs/reference/EMBEDDED.md`
  "Transmitting over the same USB cable".


- **CoreS3: the time source is a setting, behind a two-level menu.** The
  overlay listed five receivers and nothing else, so the one thing a
  portable station has to decide before it can decode — where the slot
  phase comes from — was decided by the software, from whether WiFi
  happened to associate. The root is now `MODE` (which receiver boots) /
  `CONFIG` (`TIME: NTP` or `TIME: AIR DT`) / `DEMO`. A root row only
  navigates; a page selects-then-commits, because both commits restart
  the board. `grid_src::GridSource` persists beside `boot_mode` in the
  same NVS namespace and is published through `grid_source()` so the
  picker can mark what is running.

  **What `TIME: AIR DT` ended up being took three corrections**, all
  worth recording because each was a plausible reading of the mode:

  - It first *suppressed the system clock*, which was wrong twice. The
    clock is the log's — FT8 logging needs the minute right — so
    suppressing it throws away the timestamps this board exists to
    produce; and it disabled the RTC coarse anchor that the persisted
    grid fix is designed to complete, so every boot re-acquired. What
    drifts is the slot phase, not the minute: −3.3 ppm is 11.9 ms an
    hour, so days off the network make seconds of it against a ±1.0 s
    coarse search. `AIR DT` now simply does not start NTP
    (`clock_is_disciplined()` is false without it, so air-sync owns the
    phase by the existing rule).
  - The **RTC coarse anchor was then skipped** in that mode, on
    measurements of 0.68 s and 1.65 s of error — taken while the audio
    path was losing 6.5 % of its samples. The RTC was not wrong; the
    grid was drifting +1035 ms a slot after the anchor placed it.
    Skipping it cost what it was meant to save: an `AIR DT` start with a
    good RTC spent 2 min 9 s and two 25 s captures, the first decoding
    nothing, rediscovering a phase the clock already had. With the
    anchor it decoded 8 stations on the second slot.
  - FT8 now **reads** the `grid_fix` record it has been writing since
    #356b — sub-second phase, when it was taken, how confident — and
    `uac::seed_grid_fix_us` folds it into the sink's first coarse anchor
    the way `apps/ft4.rs` already did. Refused past 12 hours (≈0.14 s of
    holdover against a ±0.4 s plateau) or below 0.55 confidence. The
    stored phase is relative to the RTC's own grid, so a constant RTC
    error cancels and only between-session drift counts, which is what
    the age limit is for.

  `rtc` also stops discarding the sub-second: the BM8563 counts whole
  seconds, so a write landing mid-second used to cost up to a second of
  phase for nothing. The prescaler is reset through the `STOP` bit as
  the write goes in, and the read side waits for the seconds tick, so
  `tv_usec: 0` is true rather than assumed. On this receiver that second
  is the difference between a grid inside FT8's coarse search and one
  outside it.

- **CoreS3: the menu names choices rather than boards, and answers the
  finger.** "FT8 / UAC" sat beside "FT4" and "WSPR", which take their
  audio over USB from the same radio in the same way, so the suffix
  distinguished nothing; they are FT8, FT4, WSPR and FST4 now.
  "DECODE (wav)" was not a receiver but a recording on a loop and said
  so only in brackets — it moves to `DEMO` as `WAV REPLAY`. The widget's
  geometry comes from one `ROWS` constant with compile-time assertions
  against every page, so adding a row cannot leave it undrawable.

  Two touch bugs behind it. **Pressing the menu's own blank area
  dismissed it**: the widget is sized for its longest page, so shorter
  pages leave painted-out bands that look like part of the widget and
  fell through to "outside", which closes — aiming at the lower half of
  an open menu was indistinguishable from a panel ignoring touches.
  Those bands are `Target::Dead` now. And **the FT8 screen sampled touch
  once per frame, behind the render**; a tap is ~100 ms of contact, so a
  longer frame missed it outright. `pump_touch` spends the frame's idle
  50 ms polling at 12 ms instead of sleeping through it, so a tap lands
  on three or four samples however long the frame took. The bus is still
  only touched while the INT pin says a finger is down. (`spot_panel`
  had already worked this out for WSPR and FST4.)

  The press-and-hold acknowledgement is **a 2 px border on the whole
  screen rather than a progress bar**, drawn the moment a finger is seen
  and erased on release. A bar draws the eye to one place and animates a
  wait nobody asked to watch; what the operator needs is one bit — the
  panel felt that. Threshold 500 → 300 ms. Both operator manuals say
  what the border means and what its absence means.

- **CoreS3: each decode carries its own DT, and the waterfall shows
  where the slot starts.** The panel and log now use WSJT-X's column
  order — dB, DT, Freq, Message. The slot median was the only thing
  reported and it cannot answer the question it is asked: a grid half a
  second out puts every station at the same offset, while loose station
  clocks scatter around a correct one, and one number in the middle
  looks identical either way. Measured the same evening: `-1.02 -0.78
  -0.70 +0.31` against a grid 0.75 s out, against `-0.04 +0.36 +0.12
  -0.04 -0.04 +0.60` on one that is right. UTC is left out — it does not
  fit beside a 22-character message and the status bar has it.

  `stage1_inc` emits a `WfTick` per FFT pair, so the waterfall's
  vertical axis is time within the slot and the grid the decoder is
  using can be drawn on it (dashed, so the row stays readable) — which
  is how an operator tells "the signals do not start at the line" from
  "there are no signals".

  The grid trim is **one per placement rather than one per N slots**: on
  its first real run it walked the grid −0.844, −1.004, −0.764, −0.202,
  +0.671 on five consecutive slots, each correction decided from a slot
  captured before the previous one took effect.

- **Embedded: the priorities actually in effect are printed, and one of
  them was wrong.** `board::log_task_stacks` had `TaskStatus_t` in hand
  and printed only the stack; with `uxCurrentPriority`, `uxBasePriority`
  and `xTaskGetCoreID` beside it, every priority claim in this tree
  stops being a claim about a value *passed*. The `decode` task runs at
  **base priority 6**, level with `uac_reader` and `USB UAC Host` on
  core 0, while comment after comment here said "the decode thread's 5":
  `CONFIG_PTHREAD_TASK_PRIO_DEFAULT` is 5 and `spawn_named` does produce
  5, so `spawn_named_tuned`'s own default path is where 6 comes from.
  Left as it is for now — the audio loss it was suspected of was the
  isochronous URB budget — but the inconsistency is real and now
  visible. `uxCurrentPriority` alone cannot separate a configured 6 from
  an inherited one, which is why the base is printed next to it.

  `stage1_inc` reports the slot's gain the same way: the shift is locked
  from the slot's first second and applied to all of it, so the peak it
  was locked from belongs beside the peak the slot reached. A slot that
  decoded nothing while showing coarse scores of 20-125 against a 1.0
  noise floor is a saturated spectrogram, and that pair of numbers is
  what distinguishes it from a quiet band.

- **CoreS3: the acquisition trigger fires on measured signal, not on a
  count of empty slots.** `ACQUIRE_TRIGGER_SLOTS` stood in for "is the
  band quiet, or is the grid lost?" — worth asking, since a 25 s capture
  on a quiet band cannot succeed and both attempts measured on a radio
  returned "none of 5 candidate phases decoded". The direct measurement
  was already in hand: coarse sync's top score, at the noise floor on a
  quiet band and tens to low hundreds when a station is there the grid
  is missing. `observe_slot` takes `had_signal`, and a slot with nothing
  in it no longer counts toward the trigger at all.
  `REACQUIRE_TRIGGER_SLOTS` stays at six for a reason the signal test
  does not cover: after a lock, signal that will not decode is more
  often fading than a grid gone bad. `grid_src::grid_label` also names
  the state that had none — `TIME: AIR DT` holding the phase from the
  sink's one-shot RTC anchor reported plain `rtc`, indistinguishable on
  the panel from an NTP build that had not synced; it is `air:rtc` now.

- **CoreS3 FT8: fine sync moved to after key-up, where there is time
  for it.** Per-candidate fine sync (`ft8b.f90` Stages A/B/C) costs
  ~292 ms of a ~1.1 s pre-key-up budget, and inside that budget it is
  a net loss: 5.50 decodes a slot with it against 6.00 without, 9-10
  candidates cut by the deadline against 4 (2026-09-19, hardware). It
  was adopted on a host mirror that by its own header does not model
  the deadline, and on board runs whose slot boundary was sliding late.

  `DecodeConfig::fine_sync_late` (on by default on the CoreS3,
  `MFSK_FT8_FINE_SYNC_LATE=0` to put it back) runs the pre-key-up path
  on coarse positions alone, and `continue_leftovers` fine syncs what
  failed and retries on the idle tail — the existing coarse-fallback
  arrangement inverted: cheap first, refinement for what needs it, and
  on the whole slot rather than the prefix.

  On hardware the pre-key-up path drops from 292 ms of fine sync to
  none, candidates cut by the deadline go 9-10 → 3, and decodes before
  key-up go 5 → 6 with 2 more on the tail. Which is the point: the
  reply this period is decided from what decodes before key-up, and
  the marginal stations fine sync recovers are next period's contacts.

- **CoreS3 FT8: what the key-up bound cuts is finished on the idle
  time after it, instead of being thrown away.** The decode task
  blocks on the next SpecBundle for ~13 s of every 15, and the slot's
  audio stays valid until the next SlotEnd, so the candidates the
  bound stopped can run there at no cost to anything:
  `dual_core::continue_leftovers` takes them with their pass-2 spectra
  intact and yields the moment the next bundle lands — the rule the
  early path's retries already used for the Slot.

  On `qso3_busy` under `MFSK_CORES3_SIM` this turns 5 decodes a slot
  into 8 (5 before key-up, 3 after, no duplicates), with the per-slot
  timing unchanged: still nothing past key-up, still ~200 ms after
  slot end. Which matters because the bound exists for one thing —
  having the reply ready before this station keys up — and everything
  else it was cutting is the queue a CQ-first portable station picks
  its next contact from. One of the three recovered on every slot of
  that run is a `CQ`.

  The late rows reach the panel and the DT statistics but deliberately
  **not** `QsoManager`: the TX intent for that period is already out,
  and whether a caller decoded after key-up should enter the state
  machine a period late is a policy question rather than a side effect
  of where the decode happened to finish.

### Fixed

- **The callsign hash table starved the CoreS3's internal DRAM and
  took WiFi down with it.** Every callsign learned cost three heap
  allocations plus map nodes — two `BTreeMap<u32, String>` and a
  `Vec<String>` — about 130 B, all of it small. On a target whose
  allocator routes sub-4 KB requests to internal DRAM (the CoreS3's
  `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096`), "small" means the
  scarce pool: internal DRAM fell from 10.7 kB to 3.4 kB over 22
  minutes of live reception until `esp-aes` could not allocate and
  WiFi — the only console a USB-host-mode board has — went silent.
  The decoder itself kept running, which is why this presented as a
  network fault.

  The table now stores callsigns inline, as upstream does: WSJT-X
  declares all three of its tables `character(len=13)`
  (`packjt77.f90:5-7`), the text living *in* the array. The port had
  been keeping the sizes and losing the shape. Three large
  allocations replace thousands of tiny ones, which on that board
  puts them in PSRAM where there are megabytes spare, and `new()`
  still allocates nothing — `is_plausible_payload` builds a table per
  `unpack77` call, so an eager allocation there would be worse than
  the leak.

  `lookup22` also stops wrapping its result in `<>` while `lookup10`
  and `lookup12` return the bare callsign. Nothing announced that
  asymmetry and it cost a double-wrapped `<<PA3XYZ>>` during the
  #383 type-5 port before a test caught it; all three return the same
  shape now.

- **The CoreS3 ran with 352 bytes of contiguous internal DRAM at every
  slot boundary, and the margin was 188 bytes.** A second, independent
  cause of the same failure the hash table produced — that one ate the
  ceiling, this one ate the floor, and fixing either alone leaves the
  board one allocation away from the same silence.

  FT8's coarse-sync peak search holds four `n_freq`-long arrays at
  once (`coarse_sync.rs:793-796`, red/jpeak × primary/secondary). With
  the CoreS3's `freq_max: 3_000.0` and `NFFT_SPEC = 3840`, `n_freq` is
  977, so each array is 3 908 B — **188 bytes under the 4 096 B
  threshold that forces an allocation into internal DRAM**. All four
  land there, 15 632 B of them, for the two seconds straddling every
  slot boundary. At `freq_max: 3_200.0` they would be 4 164 B, clear
  the threshold, and none of this would ever have happened.

  Measured against an IC-705 on 40 m (2026-09-21): free internal sat
  at 17 527 B between slots and fell to 1 699 B across each boundary —
  a 15 828 B dip that those four arrays account for to 98.8 % — with
  the largest contiguous block down to 352 B. `CONFIG_SPIRAM_MALLOC_
  ALWAYSINTERNAL` drops 4096 → 2048, which moves them out:

  | | 4096 | 2048 |
  |---|---|---|
  | `coarse` | 87-95 ms | 87-92 ms |
  | free internal, between slots | 17 527 B | 20 051 B |
  | free internal, at the boundary | 1 699 B | **15 851 B** |
  | largest block at the boundary | 352 B | **7 680 B** |

  The cost this was expected to carry does not appear: the peak loop
  reads those arrays `n_freq × lags` times and PSRAM is slower, but
  against a ~1.1 s net decode budget the difference is below what the
  per-slot timer resolves. Note the direction — `embedded-poc/
  CLAUDE.md`'s board table records that *raising* this to 16384
  corrupts tlsf with `cs Box × 2` workers; only downward was
  unexplored.

  The `uac: rx tick` line gains `lrg=`, the largest contiguous internal
  block, because `int=` alone had been the misleading number: over 363
  samples the largest block is a median of 51 % of the free total, and
  at the low points it read 448-512 B while `int` still said 1.4 kB.
  Every headroom judgement made from `int` was about 2× optimistic.

  All three boards build with `hash-table-small` as well, since all
  three run `SPIRAM_MALLOC_ALWAYSINTERNAL = 4096` and the same
  reasoning applies.

- **`m5stack-s3-app` and `m5stack-core2-app` had not compiled since
  `bbbb3044`.** That commit added three fields to
  `dual_core::SpeculativeOut` and updated the CoreS3 and `rx_wavsim`,
  but not the other two boards. `ci.yml` `paths-ignore`s
  `embedded-poc/**`, so nothing said so for three weeks. Both now name
  and ignore the new fields rather than eliding them with `..` — the
  exhaustive pattern breaking is the only thing standing in for CI in
  that tree, and it is what surfaced this.

- **Three message types were surviving the phantom filter at 100 %
  because a marker string short-circuited it (#383).**
  The then-current `is_plausible_message` returned `true` on finding
  `[FD]`, `[RTTY]` or `RR73;` anywhere in the text, which skipped the
  ITU-prefix callsign check for the whole message. (That function is
  gone by the end of this release — see **Changed** below.) They were not surviving
  because they were plausible; nothing was looking at them. Field Day
  and DXpedition are now judged on the callsigns they carry, and
  `[RTTY]` — which upstream never produces, its 22 out-of-range
  exchange codes leaving the message blank — is refused.

  With the missing `isec` range check (`packjt77.f90:338`: a 1-based
  index into 86 sections carried in a 7-bit field, so 42 of 128 codes
  name nothing), the surviving phantom population over 2 M uniform
  payloads falls **374 277 → 318 572, −14.9 %**, and every structured
  type now survives at the ~52 % rate two callsigns and nothing else
  should give, instead of at 100 %.

  The EU VHF contest type is the exception that proves the rule: both
  its callsigns are hashes, so it carries no callsign to check and
  138 700 of the 2 M payloads reach it. It is accepted only when at
  least one hash *resolves* against the table — a real exchange has
  both callsigns registered, a CRC survivor hits an entry with
  probability ~n/2¹² and ~n/2²².

- **CoreS3: USB audio transmit works in one format only, and it is not
  the one the radio advertises first.** `uac_host_device_start` refused
  2 ch / 16 bit / 48 kHz with `ESP_ERR_NOT_SUPPORTED` — a format the
  IC-705's own descriptor lists. The refusal is not the UAC driver's:
  `hcd_pipe_alloc` rejects the pipe with `EP MPS (192) exceeds
  supported limit (128)`. An ESP32-S3 has no HS PHY, so
  `otg_dfifo_depth` is 256 lines, the default BALANCED bias gives
  `ptx_fifo_lines = 256/8 = 32`, and the periodic-OUT limit is
  `32 * 4` = 128 bytes; alt 1 carries 192. Mono 16-bit is 96 B/frame
  and is the only 16-bit OUT format that fits (alts 3-6 are 8-bit).
  `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT` would admit alt 1 and
  take the RX FIFO from 160 lines to 34 — on the board whose receive
  path already lost 2.6-6.5 % of its audio to an isochronous URB
  budget — so the probe walks a short format ladder instead and
  `write_ft8_frame` emits the channel count `device_start` accepted.
  Writing L = R into a stream the driver paces as mono hands the radio
  twice the audio it expects. The audio device is also not the radio:
  an IC-705 is a TI hub, an Icom CDC composite and a **PCM2901**
  codec, and the OUT interface belongs to the codec.

- **CoreS3: the transmit path overflowed its task stack the first time
  it ever ran.** Until `device_start` succeeded, `handle_tx_connected`'s
  body was code no measurement had covered: the silent-write buffer
  (1 920 B, live across the call) plus `write_ft8_frame`'s 3 840 B and
  480 B put 6 240 B on a 4 096 B stack. The first `start` that
  succeeded ran them and the board rebooted 30 s into the capture, with
  no panic on any console that still existed. Buffers moved to the
  heap, `APP_TASK_STACK` raised to 8 192, and the frame loop given a
  progress guard so it cannot spin forever inside an enumeration
  callback.

- **CoreS3: the TX probe's outcome survives the log path now.**
  `handle_tx_connected` runs during enumeration, tens of seconds before
  the network that carries its log, so its result was missing from
  every capture. `TX_STAGE` (0..6) rides the periodic
  `usb_host_lib_info` line and separates "never entered" from "open
  failed" from "still inside the 12.6 s frame" — three states one
  `esp_err_t` could not tell apart. The enumeration dump now also
  prints each interface's Type I format descriptor and `bInterval`,
  not just endpoint sizes.


- **CoreS3 FT8: the USB audio path was silently dropping 6.5 % of its
  samples, and that was the grid bug.**
  `CONFIG_UAC_NUM_ISOC_URBS=3` × `CONFIG_UAC_NUM_PACKETS_PER_URB=3` is
  one packet per 1 ms frame, so the USB layer held **9 ms** of audio.
  Each URB is resubmitted from inside `usb_host_uac`'s own transfer
  callback, in the class-driver task; when that task is late the
  controller has no descriptor queued and *skips* those frames. The
  samples never reach the ring, so nothing downstream can see that they
  are missing — and a single 10 ms FreeRTOS tick slice is already past
  the budget. Now 6 × 8 = 48 ms.

  Measured against an IC-705, ~220 s per configuration, each tick's own
  byte count over its own interval:

  | URBs × packets | delivered | slot grid |
  |---|---|---|
  | 3 × 3 (9 ms) | 179 609 B/s = 11 225 sa/s | **+1035 ms per 15 s slot** |
  | 6 × 8 (48 ms) | 192 062 B/s = 12 003.9 sa/s | −4.8 ms per slot |

  A sink that counts 180 000 samples as 15 s and is handed 11 225 sa/s
  walks off the air at a second every two and a half slots. That is the
  whole of "the grid drifts within minutes of an NTP sync that was
  itself good, and decodes fall to one station". After: 15 consecutive
  slots all decoding.

  Every other vantage point said "fine" and was checked first — the
  reader never blocks (`send_box` max 7 µs, back in
  `uac_host_device_read` within 4 ms, zero read timeouts) and the ring
  cannot overflow, since a reader that returns on 4 096 B keeps it at
  0-4 KB. What ruled all of that out was the one observation that a
  deficit second is never followed by a surplus one: anything merely
  delayed comes back.

- **CoreS3: the RTC was exactly one second slow on every write.**
  `write_from_system_clock` spun to just before a UTC second, took
  `as_secs() + 1`, held the chip with `STOP`, wrote eight registers over
  I²C — about 1 ms, which crosses that very boundary — and then waited
  for the *next* boundary to release. The chip resumed holding second
  S+1 at true time S+2. It now picks the release boundary first and
  writes the value belonging to it, which also gives the transaction a
  whole second instead of 800 µs.

  Invisible to anything that also runs NTP, since the system clock is
  disciplined within seconds of boot. `TIME: AIR DT` does not start NTP
  at all, so it inherited the whole second, and every measurement taken
  of that mode beforehand was of this bug:

  |  | `dt` | decodes | captures | trims |
  |---|---|---|---|---|
  | before | −0.7 .. −1.0 | 2-8 | 2 × 25 s | every slot |
  | after | +0.04 ± 0.07 | 4-9 | 0 | 0 |

  — grid locked on the second slot, 15 s from boot.

  **An instrument that shares the suspect's reference cannot convict
  it.** A probe added to decide whether the boundary was declared at the
  wrong *time* or over offset audio read `utc_now_ms()`, so it measured
  the grid against the board's own clock — the thing that was wrong —
  and reported "boundary on UTC to ±32 ms" while the band read
  `dt −1.0`. Hours went into treating that contradiction as a decoder
  problem. Both halves are now in `m5stack-cores3-app/CLAUDE.md`'s
  own list of what costs a session.

  Two loosenings added the same evening to make the grid self-correct
  faster are **reverted**, because with the clock right there is nothing
  to correct in a hurry: `ACQUIRE_TRIGGER_SLOTS` goes back to three, and
  the trim takes pooled cross-transmitter evidence only. The single-slot
  median admitted briefly was the 2026-09-05 per-slot servo under
  another name and behaved the same, walking the grid −0.844, −1.004,
  −0.764, −0.202, +0.671 on five consecutive slots.

- **CoreS3: the slot phase is corrected every slot, exactly, to the next
  UTC boundary.** It used to fire only past a re-anchor threshold and
  then take the whole error out of one slot. The error refills, so the
  correction was never rare; and a slot cut more than 12 000 samples
  short never reaches `SPEC_EMIT_PAIR` at 168 000, so it emits a partial
  `SpecBundle` and the decoder gets no tail window — alternating slots
  at `pair_done=85/92`, `tail_win=0`, `dec` 0-2 against 7 on the slots
  between them.

  A clamped correction was tried first and was wrong in the one
  direction it cannot be: `remain` is the distance to the *next*
  boundary, so a grid running early gives a small `remain`, and clamping
  it to a 174 000-sample floor shortens the slot where lengthening was
  wanted. The error then grew by exactly one clamp step per slot — on a
  radio it walked +503, +1027, +1494 … +7002 ms, and 26 of 123 slots
  over 30 minutes decoded nothing. Lengthening was never available
  either: `stage1_inc::NMAX` is 180 000 and that is the spectrogram's
  geometry (`N_TIME = NMAX / NSTEP - 3`), not a buffer that can grow.

  Ending the slot on the next UTC boundary is exact in one step
  whichever way the error points. A large error makes that one slot
  short enough to decode nothing, and that is the whole price — the
  right one, because spreading a 3 s error over six half-second steps
  gives six slots at a phase the band cannot be found at. A 200 ms dead
  zone keeps it from firing on jitter.

- **CoreS3: SNTP is still watched after the 20 s window closes.**
  `wait_synced`'s timeout ended a wait, not the sync, and the caller
  treated it as the answer for the session. Two consecutive host-mode
  boots timed out and ran as `grid=rtc` thereafter — which turns off the
  sink's UTC phase tracking entirely — from a board whose RTC had been
  set from NTP minutes earlier, and which synced in 2.4 s and 4.2 s on
  the boots either side. The network task now polls every 5 s at
  priority 2 until it lands, then promotes the clock source through the
  same `note_clock_from_ntp` the initial wait would have called.
  `wait_synced` counts its own delays rather than wall time, so this is
  not a starved poll given more room: the exchange had not arrived.

- **CoreS3: stage 3 is bounded by the slot boundary the audio sink
  publishes, and the carry-over is gone.** A slot's leftovers no longer
  travel into the next slot's decode; the deadline is computed per slot
  from the published boundary plus the ~0.5 s the late path needs, and
  `budget_ms` stays as a 13 s runaway cap and nothing else.

  Both halves were learned the hard way. Raising the cap to 13 s *as the
  only bound* let stage 3 run 1.0-2.0 s past slot end, and on a radio
  that cost the next slot's audio: every slot decoded 0 while the grid
  slipped a second at a time. And the hint needs a sanity check —
  a deliberately abnormal slot (cold acquisition stretches one to 25 s)
  published a boundary putting key-up 9.2 s in the past, and an already
  expired deadline claims no candidates at all (`hint_err=-10775ms`,
  `cut=15`, `dec=0`, on a slot whose audio was fine). More than half a
  slot out is a hint wrong about which slot it names; one or two seconds
  in the past is a backlogged pipeline, which is true and is meant to
  bite. `fine_sync_min_slack_ms` measures slack to that same deadline,
  so fine sync does not run in an aligned steady-state slot (~1.43 s
  against a 2 s threshold) — the measurement's answer, not an accident:
  at the honest ~1.1 s budget fine sync cost decodes rather than winning
  them, 10 against 5.5.

  An intermediate step bounded the carry-over by slack instead of
  removing it (three gates: no key-up overrun, `stage1_inc` within
  `CARRY_OVER_MAX_LAG_US` (300 ms) of the audio clock, 2 s slice cap). Recorded because it is
  right on its own terms and was **not** the cause: with the gate
  holding back 14-15 candidates a slot and the carry-over doing nothing,
  `hint_err` stayed at −0.5..−0.75 s and slots kept being skipped.

- **CoreS3: a wrecked slot is no longer evidence about the grid.** A
  cold acquisition holds the decode task for 25 s of capture plus
  10-15 s of compute, and a bundle queued meanwhile is decoded after its
  own slot has gone — no tail window, candidates cut wholesale. Counted,
  such a slot reads "signal present, nothing decoded", which is exactly
  what the acquisition trigger looks for, so the acquisition was asking
  for another acquisition on the strength of the slots it had just
  ruined. The test is `tail_win == 0 && post_slotend > 1 s`, and the
  pair matters: `q_wait > 7.5 s` was tried first and missed by a factor
  of two, because a *failed* acquisition holds the pipeline ~40 s and a
  *successful* one ~3 s. Against a 102-slot radio run `tail_win` was
  889-927 ms on every healthy slot and `post_slotend` had median
  336 ms / p90 454 ms, so neither half is close to normal.

  The acquisition trial loop also **reports every trial**, not only the
  ones that decoded. Two `continue`s sat above the log line, so "none of
  5 candidate phases decoded" covered two outcomes wanting opposite
  fixes — tried and failed, and never tried at all. The second is real:
  a trial cuts a whole slot starting at the candidate phase, so an
  offset past 10 s runs off the end of a 25 s capture and a third of the
  phase space is unreachable. Capturing two slots would fix it and is
  not available — `ACQUIRE_CAPTURE_SAMPLES = 2 * SLOT` takes the ring to
  720 KB and needs ~1.44 MB transiently while `take_acquisition_audio`
  hands one Vec out and `arm_acquisition` reserves the next; the board
  died of `rust_oom` in `stage1_inc` and rebooted into a loop. The
  reasoning is recorded beside the constant, so the next attempt starts
  from it rather than from the buffer.

  After all of it: 102 slots over 29 minutes on 40 m, mean 8.2 decodes,
  no empty slot, `cut > 0` on 2 slots, no acquisition and no trim.

- **A decoded row's message column went blank above 18 characters.**
  `ui::decoded_list` built each row in a `String<32>` while computing
  the message's room from `ROW_CHARS`, which is 40 — so with the
  14-character `dB DT Freq ` prefix it offered 25 characters into a
  buffer with 18 left. `heapless::String::push_str` is all-or-nothing,
  and the `Err` was discarded, so **every message of 19 characters or
  more drew its SNR, DT and frequency and then nothing at all**. On
  7041 kHz this morning that was 71 of 380 decodes (19 %), and always
  the portable stations: `JG3AGB/P JE1NGI PM95` is exactly 20. The
  formatter is now split out as `row_text` and tested in
  `hosttest/mfsk-app-shared`, which is what had been out of reach while
  it needed a `DrawTarget`. Confirmed fixed on the panel.

- **Callsign hashes never resolved, because nothing ever filled the
  table.** `unpack77_with_hash` has always been able to turn a hashed
  callsign field into a name; outside three unit tests there was no
  `CallsignHashTable::insert` call anywhere in the crate or its
  consumers, and the only plumbing that carries a table —
  `DecodeContext::callsign_hash_table` — is an
  `Arc<dyn Any + Send + Sync>`, which cannot be written through. So
  every receiver rendered `<...>` forever: 69 of 380 on-air decodes
  carried one.

  `msg::wsjt77::unpack77_learn` resolves first and registers second —
  a message must not resolve its own hash against a call it is itself
  introducing, which is the ordering WSJT-X's `save_hash_call` has.
  Registration walks the *fields*, never the rendered text: a token
  scan cannot separate `PM95` from a callsign, and `insert` rejects
  only `CQ…` and strings under two characters, so `RRR`, `DX` and `TU`
  would all be registered — and a polluted entry resolves a later hash
  to the **wrong** station, which is worse than a placeholder.
  `register_callsigns` is public for callers that cannot hold a `&mut`
  where they resolve: the FT4 receiver decodes candidates on two cores
  at once, so `Ft4Decode` now carries the raw payload and the app
  resolves and learns single-threaded after the workers join.

  Verified on the air, one exchange end to end: `CQ JA6GVF PM53` at
  12:09:46 on 891 Hz, then at 12:18:01 on 1194 Hz
  `JL1IBJ/3 <JA6GVF> 73` — the 12-bit hash resolved from a table
  entry learned eight and a half minutes and thirty-four slots
  earlier.

- **The reply deadline is the slot boundary, and `dec` was counting
  two things at once.** WSJT-X opens its transmit window *at* the
  boundary: the message is taken from the auto-sequencer and PTT
  asserted in the same pass (`mainwindow.cpp:4552-4711`), `txDelay`
  runs from the rig's PTT confirmation and is spent *inside* the
  0.5 s, and `Modulator::start` pads silence so audio still lands at
  `delay_ms` (500 for FT8, 300 for FT4). So a decode finishing inside
  that 0.5 s is already too late to answer, even though stage 3 is
  allowed to claim there. The whole schedule, with citations, is in
  `EMBEDDED.md` / `.ja.md`.

  The slot log now carries `intime=` — decodes completed before the
  boundary — measured exactly by running the early first-attempt batch
  as two `stage3_split` calls split there, which covers the same
  candidates in the same order for the same cost. On a radio,
  **8.4 % of decodes were arriving too late to be answered**, on 41 %
  of slots. `ref=` (candidates the early half handed to stage 3) joins
  it, so cost per candidate is derivable.

- **`MFSK_FT8_SPEC_EMIT_PAIR` makes the emit point a build knob**, and
  an A-B-A on the air says what moving it buys. At the shipped pair 87
  the early path wants 1 143 ms of stage 3 and only 818 ms lands before
  the boundary — about 4 of 15 candidates decided too late. Pair 85
  adds 320 ms:

  | arm | dec | intime | late | post_slotend |
  |---|---:|---:|---:|---:|
  | 87 (A, n=44) | 8.64 | 7.91 | 0.73 | 282 ms |
  | 85 (B, n=53) | 8.36 | **8.32** | **0.04** | **92 ms** |
  | 87 (C, n=50) | 8.54 | 7.86 | 0.68 | 261 ms |

  The two 87 arms agree to 0.1-0.2 σ on every figure, so the middle one
  is the flag and not the band. Late decodes essentially vanish
  (6.3 σ); the decode total moves −0.23 and `intime` +0.44, neither
  significant at this sample size. **Not adopted yet**: the −0.23 has
  not been separated into the lag ceiling and the shorter prefix
  (`defer` goes 1.4 → 3.9, and the late path has no budget without
  `share_cand_budget`). Default stays 87.

- **`stage1_inc::max_lag_s` is one authority for the emit/lag bound**,
  which three comments used to state independently and two of them a
  row apart. Emit pair `P` fills rows `0 ..= 2P-1`, block 2's last
  Costas sits at 162, so `lag ≤ (2P - 163) × 0.08` — 0.88 s at the
  shipped 87, against a configured 1.00 s. The boot line reports the
  pair and says `OVER` when it is.

  The explanation that bound used to carry was wrong and is corrected
  here: exceeding it does **not** make the search correlate against
  zeros in any way that matters. `tests/ft8_coarse_partial_blocks.rs`
  measures a zeroed tail scoring **bit-identically** to the same
  symbols skipped — the score's numerator and denominator are sums
  over the same symbols, and a zero adds nothing to either. What a lag
  past the bound really loses is *evidence*: the candidate is scored on
  two Costas blocks instead of three, with a ratio that is not
  penalised for it, so a two-block coincidence competes with a
  three-block station. That is the ±1.75 s widen's failure, and the
  shipped ±1.0 s is the same thing two rows over — two such candidates
  in the pass-1 list on `qso3_busy`, one at rank 8, inside the refined
  top-`max_cand`.

- **`MFSK_FT8_BLOCK2_GATE` refuses to score a lag whose block 2 is
  incomplete** — off by default, and a deliberate divergence from
  WSJT-X, which scores the truncated block and takes the result
  (`sync8.f90` guards the read and skips). Upstream never needs it:
  upstream's spectrogram ends where its slot ends, and this one is
  emitted early and declares the full `n_time` because that is what
  sets the allsum's stride. One rule covers both scores, since both
  contain block 2; negative lag moves block 2 *earlier* and is never
  the truncated one, so the asymmetry the measurement asked for — 0 %
  of 1 382 on-air decodes past +0.88 s against 0.26-1.35 % below
  −0.88 s — falls out with no special case. Opt-in through a
  `valid_rows` parameter, so the host path is bit-identical.

- **CoreS3: every acquisition trial now has a slot to cut.** The entry
  above reported the unreachable trials; this removes them, and the gap
  was wider than that report said. `acquire_slot_phases` returns centres
  in (−7.5, +7.5], `rem_euclid` turns a negative one into a capture
  offset in (7.5, 15] s, and a whole slot only fits behind an offset up
  to 10 s of a 25 s capture — so **every centre in (−5, 0)**, a third of
  the period, had nothing behind it. Measured at 30 capture starts per
  recording: 50 of 150 centres on each of `qso1`, `qso2` and
  `qso3_busy` — the geometry, not the band — and on `qso3_busy` six of
  thirty starts acquired *nothing at all*, the whole shortlist being
  centres with no slot behind them, for which the board's only answer is
  another 25 s capture.

  The room was in the trial's own window rather than in the buffer.
  `decode_block_tuned` searches ±2.5 s about wherever the slot is cut,
  and the reachable band's complement is 5 s wide and wraps at both
  ends, so no centre is further than 2.5 s from it: clamping the offset
  circularly into the band always lands inside the search. What makes it
  correct rather than merely closer is that the applied phase is now
  measured from the offset actually cut at instead of from the centre —
  the median DT that corrects it is relative to the cut, and the two
  agreed only while nothing moved. Decodes at the grid that results:
  `qso3_busy` 4.70 → 6.07, `qso1` 3.32 → 3.86, `qso2` 3.48 → 4.38; one
  start of thirty lost a decode, six gained a grid where there had been
  none. `mirror_acquisition_unreachable_phases` is the measurement.
  `ACQUIRE_CAPTURE_SAMPLES` is now load-bearing for the 2.5 s bound,
  which is `(SLOT − (CAPTURE − SLOT)) / 2`, and says so.

  **Confirmed on the board, against the same board without the fix.**
  `MFSK_CORES3_SIM` with the feed 12.5 s out, i.e. a true grid phase of
  −2.5 s, the middle of the band that had nothing behind it. Before:
  the two heaviest clusters (−0.377 s, −3.018 s) were both skipped,
  trials 3 and 4 decoded nothing, and trial 5 at −5.367 s decoded
  **one** station and set the grid 0.96 s out — which then ran at 2
  decodes a slot for six slots, one of them 0 with `cut=15`, before a
  **second** 25 s acquisition finally placed it. After: trial 2 cuts at
  10.10 s instead of 11.98 s, decodes 6, and the grid lands at −2.55 s.
  Steady 6 decodes a slot from the slot after the acquisition, `cut=0`,
  `hint_err` ±12 ms, one acquisition and no trim — seven slots, 105 s
  of band, earlier than the same board reached it without the clamp.
  Reproduced on a second flash (grid −2.46 s, same shape).

- **CoreS3: the panel is BGR, and every colour has been swapped since
  the first screen.** `CSS_ORANGE` (255, 165, 0) reached the panel as
  (0, 165, 255) — sky blue. mipidsi defaults to RGB; the CoreS3's
  ILI9342C is wired BGR. Green, white, grey and black are unaffected,
  which is most of what these screens draw and is why it survived: the
  menu's amber hold indicator was the first thing to name a colour the
  swap could ruin. The waterfall is the other casualty, its
  `black → blue → cyan → green → lime → red` palette displaying with the
  two ends reversed. All five CoreS3 display inits are set.
  `m5stack-core2-app` is deliberately untouched — same controller and
  the same M5GFX-derived `invert`, so very likely the same wiring, but
  that board's screen has not been looked at and a blind flip would be
  the same mistake in the other direction.

- **Core2's panel is BGR too.** The entry above deliberately left
  `m5stack-core2-app` on RGB, on the grounds that its screen had not
  been looked at and a blind flip would be the same mistake in the
  other direction. It is the same ILI9342C with the same M5GFX-derived
  `invert`, and the operator's reading of the pair settles it: same
  chip, same wiring. Set. **Not seen on that board** — the Core2 has
  not been flashed since, and if its colours ever read as swapped this
  is the line to try removing.
- **CoreS3: the audio task now outranks the decoder, and what that
  exposed.** Both were priority 5 — the decode pipeline and the
  UAC reader are `std::thread`s, so they took
  `CONFIG_PTHREAD_TASK_PRIO_DEFAULT` (5) and
  `CONFIG_PTHREAD_TASK_CORE_DEFAULT` (no affinity), while the raw
  FreeRTOS tasks around them were placed deliberately (`stage1_inc` at
  6 to preempt `dsp_worker` at 5; `net` pinned to core 1, "never core
  0, which carries capture"). Equal priority means FreeRTOS rotates
  them per 10 ms tick on whichever core they share, so a decoder
  running long took half the audio task's core: over one ~13 s cold
  acquisition the sink's slot-boundary publishes fell **4.06 s**
  behind, enough for the key-up floor to read the wrong slot's
  boundary. On the harness that is lag; on a radio it is loss — the
  reader drains a 16 KB (85 ms at 48 kHz stereo) ring and blocks
  rather than drops when the queue behind it is full. The audio task
  (reader and `MFSK_CORES3_SIM` feeder alike) is now priority 6, and
  the decode thread is pinned to core 0, where it already ran and
  where it must stay for the two-core split to be two cores.

  **The decode count on `qso3_busy` fell from 10 a slot to 5.5, and
  that number is the honest one.** With the feeder starved, the last
  12 000 samples of each slot took 1.52 s to arrive instead of 1.00 s,
  and the decoder spent that slack: `tail_win` was 1.56 s where the
  geometry says 1.00 s (the SpecBundle leaves stage1_inc at 168 000 of
  180 000 samples). It now measures 924-928 ms across every steady
  slot, 75 ms under nominal for stage1_inc's own lag, and `hint_err`
  is ±0 ms against stage1_inc's boundary. A station's key-up comes on
  UTC whatever our buffering does, so the slack was never ours: the
  real budget for answering in the next slot is ~1.1 s (0.93 s of tail
  + 0.5 s to key-up − the 320 ms guard), not the 1.8 s the board had
  been given. Per-slot `tail_win` in the 2026-08-23 IC-705 capture
  averaged 1.44 s with a 0.88-2.07 s spread — the same distortion,
  measured on a radio with the reader still at priority 5, so it does
  not settle what a radio gives at priority 6. That run is still to
  come.

  A consequence, measured the same day and not yet acted on:
  per-candidate fine sync costs 292 ms of that 1.1 s and now **loses**
  decodes — 5.50 a slot with it against 6.00 without, 9-10 candidates
  cut by the deadline against 4. It earned its place when the budget
  was 1.8 s (`MFSK_FT8_FINE_SYNC=0` A/Bs it), and `qso3_busy` is a
  crowded recording; on a band carrying six signals rather than
  eighteen the arithmetic may come back. Left on by default until
  that is measured rather than assumed.

- **CoreS3 FT8: a slot that cannot finish before key-up is dropped
  before it starts, and the boundary that decides it comes from the
  audio clock.** Cold acquisition occupies the decode core for ~10 s, so
  the slot behind it reaches the decoder with a fraction of its tail
  left. It then ran coarse and fine sync — which no deadline bounds; the
  key-up bound stops stage-3 *claims* — and finished up to 1.34 s past
  this station's key-up having decoded nothing.
  `dual_core::DecodeConfig::slot_floor_ms` (500 ms here,
  `MFSK_FT8_SLOT_FLOOR_MS=0` to disable) now drops such a slot up front,
  receiving its audio so the pipeline stays drained.

  Locating the boundary took two tries, both measured on the board. The
  first estimate was the SpecBundle's own — emit timestamp plus the
  audio it carried — but that sample count is what stage1_inc has
  *consumed*, and acquisition starves stage1_inc too: under exactly the
  condition the floor exists for, the estimate slid late with it and the
  floor never fired. The second read the boundary from the audio sink's
  clock, but split "the slot being decoded" from "the one before it" at
  the point the bundle is emitted (14.0 s of 15) — which is where the
  decoder reads it, so a few ms of jitter flipped the answer by a whole
  slot: every other slot was reported long over and thrown away, 8 of 18
  in one run, each with ~1.1 s left. The sink now publishes each slot's
  start **and its length**, the split sits at half a slot — seconds from
  either real reading — and the one slot a cold acquisition lengthens is
  measured as the longer slot it is rather than losing its extra 3 s of
  tail. `time_sync::decoded_slot_end_us` is that arithmetic, pinned by
  host tests; the per-slot log gained `hint_err` (the same boundary as
  stage1_inc later dates it, −36..−46 ms in steady state) and `q_wait`.

  Also: acquisition stops at the first trial reaching `LOCK_MIN_DECODES`
  (3). Scored on the host over 24 start phases per recording, the decode
  count is identical to running all five trials (qso3 8.00, qso1 3.88,
  qso2 4.74) for 5.0 → 2.3 trials, and on the board the stall it causes
  is 10.4 s where it was 15.2 s when trial 2 wins, 13.6 s when trial 3
  does. Steady state on `qso3_busy` under `MFSK_CORES3_SIM`, in each of
  two runs: eleven consecutive slots decoding 10, finishing ~256 ms
  after slot end against a key-up at +500 ms, none past it. The second
  run is also the first to drop a slot — the one acquisition pushed to
  280 ms of tail, where coarse and fine sync alone need ~370 ms, and
  which overran key-up by 738-1344 ms in every run before the floor.

  One caveat, from that same slot: while acquisition occupies the core
  the audio sink is starved with everything else, so its boundary
  publishes lag and the hint reads seconds out either way (−4.06 s on
  the slot before the shift, +4.44 s on the lengthened one) before
  settling to −35..−47 ms. Neither run lost a decode to it, and both
  readings agreed the dropped slot was under the floor, but an
  optimistic reading is one that fails to drop a slot that should be.
  Whether the sink lags the same way when the audio is a radio's rather
  than the harness's is not measured yet.

- **CoreS3 FT8 grid acquisition: the capture's own offset was dropped,
  one decode set the grid, and a partial slot could lock it.** Three
  ways the embedded controller's slot grid (#356) landed wrong, found on
  the board under `MFSK_CORES3_SIM`:

  *The capture offset.* The 25 s acquisition ring starts filling when
  `arm_acquisition` runs — part-way into a slot — and every phase is
  measured from the capture's first sample, but it was applied as a
  shift from a slot boundary. Clockless with the feed 3.000 s late, an
  acquisition armed ~0.98 s into its slot applied +1.90 s, left the grid
  1.10 s short (outside the ±1.0 s per-slot search), and needed a second
  acquisition: about three minutes with nothing decoded. `uac` now
  records where in its slot the capture began and the phase counts it;
  the same case then locked on one acquisition, 10 decodes a slot. What
  remains is the trial decodes' own median bias, ~0.2 s either way.

  *First decode wins.* Trial phases were accepted at the first that
  decoded anything, so one decode could set the grid — the thing
  `grid_state` refuses by name (`LOCK_MIN_DECODES`). Measured: accepted
  trials decoded 7, 2, 1 and 6, the 1 from trial 3 of 5 with two better
  phases untried. All trials are now ranked by decode count.

  *A partial slot voted.* The first slot after the clock anchor holds
  whatever audio remained, so its signals sit where no full slot on that
  grid puts them. Twice it decoded 8 and 5 and locked; every full slot
  on that grid then decoded 0 until the six-slot relock ran out. Partial
  slots are still decoded and shown but neither lock nor count as under
  par. **Not yet seen fixing that case**: the three runs since happened
  not to produce a partial slot on a wrong grid.

  Also: acquisition yields a tick between its pieces (the task watchdog
  fired seven times in one capture), and each trial's result is logged.

- **A grid correction the gap could not hold was applied twice** (#376,
  follow-up to #369). `SlotGrid::fill` cleared `clock_trim` at every
  window close, including the close that clamped `want_skip` and
  carried the remainder. The next window then measured the same
  uncorrected error — the grid really was still that far off — and
  `phase_error`, netting against a `clock_trim` of 0, queued it a
  second time on top of the carry. The grid overshot by exactly the
  carried amount and settled two windows later.

  Reachable from the case the re-anchor branch exists for: NTP stepping
  an RTC-seeded clock by seconds. On FT4's constants (`gap = 8 700`,
  `reanchor_thresh = 1 200`) any negative step past −0.825 s does it,
  and a −1 s step overshoots by **3 300 samples, 0.275 s**. Bounded and
  self-correcting, and inside both the 6.775 s capture window and the
  ±2.5 s coarse search, so the affected slot still decodes — which is
  why it was invisible from outside: one slightly-off slot, nothing
  naming why.

  The close now carries the unspent part as `clock_trim` instead of
  discarding it, so the next window nets to ~0 and queues nothing; with
  nothing clamped it is 0, exactly as before. The new test drives the
  grid in reader-sized blocks and fails on the old code with −200 where
  −100 was owed.

- **The two Q65 enums never reached the header, so C callers wrote
  `0`.** Every `mfsk_q65_*` function takes its sub-mode as `uint32_t`,
  and deliberately so: a `#[repr(C)]` fieldless enum is an `int` to C,
  so reading an out-of-range discriminant back as a Rust enum is
  undefined behaviour — the lesson `mfsk_mode_name((MfskMode)9999)`
  taught by segfaulting. But cbindgen emits a type only where a
  signature mentions it, so `MfskQ65SubMode` (0..=9, and *not* in
  slot-length order — `A15` is 6) and `MfskQ65FadingModel` were
  declared nowhere, and a C consumer had to hardcode the numbering. In
  a release whose headline is that the ABI can be asked what it
  supports, that is the wrong way round.

  `cbindgen.toml` lists both under `[export] include` now: the
  declarations appear, with their doc comments, and **no signature
  changes** — named in C, `uint32_t` on the wire. The C++ driver gained
  a Q65-30A round trip written by name (`MFSK_Q65_SUB_MODE_A30`,
  `MFSK_Q65_FADING_MODEL_GAUSSIAN`), which both proves the emission
  from a real translation unit and is the first time the driver
  exercised Q65 at all — it previously touched the family only to check
  that a bogus sub-mode is rejected.

- **A binding-only PR ran neither binding job.** `ci.yml`'s `changes`
  filter had no path list covering `bindings/**`, and both jobs were
  gated on `src`, which does not mention it — so a PR that changed only
  the Kotlin binding skipped the Kotlin job that exists to cover it.
  There is a `bindings` output beside `src` now (`bindings/**` plus
  `mfsk-ffi/**`, since both bindings compile against the generated
  header), and the two jobs are gated on either. A `bindings/**`-only
  change still skips the protocol tiers, the feature matrix and the
  cross-compiles, which is the point.

- **`MFSK_DECODE_FLAG_HASH_RESOLVED` was missing from the header.** The
  constant lived in `mfsk-ffi-abi`, and cbindgen cannot emit a
  dependency's constants — the same limitation that had already sent the
  capability bits and the array lengths into `mfsk-ffi` proper. A C
  consumer reading `MfskDecode::flags` could not name the bit.

  Found by the Kotlin JNI shim failing to compile, which is the first
  thing in this repo to read that field from C. The C++ driver did not
  catch it because it never touches `flags` — which is the argument for
  having more than one consumer compile against the header, and for the
  shim being C rather than Rust-with-`jni`: a Rust shim links against
  the crate and reads no header at all.

- **MSK144's `slot_samples_12k` was one sample short.** The row reported
  863 where the frame is 864 samples (144 symbols x 6 at 12 kHz):
  MSK144 has no registry entry, so its geometry is assembled by hand,
  and that one field was computed as `(0.072_f32 * 12_000.0) as u32` —
  0.072 is not exact in binary32 and `as u32` truncates rather than
  rounds. Every other mode copies a precomputed integer, so this was the
  only row that could be wrong, and it was. It is `NSPS * N_SYMBOLS`
  now, which also makes the invariant visible at the site.

  A caller sizing a buffer from that field got one sample less than a
  frame, and `mfsk_stream_open` sizes its capture ring from it.
  `mode_introspection.rs` now checks `slot_samples_12k == t_slot_s x
  12 kHz` for **every** mode in the build rather than the four spelled
  out by name, which is the assertion that would have caught it.
  Found by the new Swift binding's geometry test.

- **The local pre-push gate built the `mfsk-ffi` feature combinations
  without testing them**, and that cost a red CI run. `mfsk_runtime_-
  configure` correctly reports `UNSUPPORTED` on a build with no thread
  pool; the test asserting it covered only the `parallel` half, compiled
  fine under `mobile`, and failed at run time. CI runs the suite under
  both feature sets, so a local gate that only builds them reports green
  on exactly the combinations it exists to cover. It runs `cargo test`
  for both now.

  The test is split by feature, which is the better shape anyway: it
  pins both halves of what the library documents — with a pool, the
  configuration takes effect; without one, decoding is already
  single-threaded, which is a *stronger* contract than the pool
  provides rather than a missing one.

- **A macro-generated `extern "C"` function never reaches the header.**
  The packers and the two synthesis calls were written as
  `macro_rules!`, which cbindgen — parsing this crate syntactically —
  cannot expand, so six functions existed in the library and were absent
  from `mfsk.h`. A C consumer cannot call what is not declared.

  Found by the C++ driver failing to compile, which is the reason it is
  a real translation unit rather than a Rust test. They are written out
  explicitly now, with a comment at the site saying why.

- **FT4's slot grid was steered from the wrong reference frame, and now
  has tests (#354).** `apps/ft4.rs` read "samples to the next UTC
  boundary" from the clock and handed it straight to
  `SlotAccum::anchor_or_reanchor`, which counts from where the
  *accumulator* sits in the sample stream. The two are the staged-but-
  not-yet-fed backlog apart. During capture that is one UAC read
  (~21 ms) and would not matter; once a slot it is the decode — the
  ~1.4 s that piles up while `decode_slot` runs — which is past
  `REANCHOR_THRESH_SAMPLES` and comparable to the whole ±1.0 s Δt
  search. A grid that was not drifting therefore read as off by exactly
  the backlog, in the same direction, every slot. The caller now
  converts before it asks.

  Found by reading, not by running: this path is inside `if live`, so
  the baked-golden replay never reaches it and nothing had exercised it
  yet.

- **`embedded_shared::apps::ft4_grid::SlotGrid`** — the slot grid's
  arithmetic, split out of `SlotAccum` with no DSP and no ESP-IDF in
  it, so `hosttest/mfsk-app-shared` compiles it and its cases run in
  CI. `ft4_rx` pulls in `esp_idf_svc` and spawns a second core, so it
  only builds for Xtensa, and `embedded-poc` sits outside the host
  workspace with neither CI lint nor CI test reaching it — what was
  left checking this was a live run against a radio, which is the most
  expensive instrument available and the last one to be pointed at an
  off-by-one.

  It also corrects a claim in three places — `capture_window.rs`'s
  module doc, `ROADMAP.md`, and this file's own #313 entry below — that
  FT8 and FT4 both capture a *contiguous* grid and so have no
  inter-slot gap to manage. That is true of FT8 only. FT4 closes its
  capture at 6.775 s of a 7.5 s slot and discards the 8 700 samples
  between, which is the same skip `CaptureWindow` exists for; what is
  actually different is that FT4's grid carries a phase correction
  across window closes and `CaptureWindow` has no state of that kind.
  Both remain consolidation candidates rather than one covering the
  other, and neither is changed.

  The move was *nearly* behaviour-preserving, and the exception is the
  entry below: the extracted first-anchor branch zeroed a fill counter
  the old in-place version did not have, which desynchronised the grid
  from the audio the caller holds. Ten cases now, including the
  reference-frame bug above, the trim landing at the next window close
  rather than on a gap already set, the wrap that keeps a boundary just
  behind the grid reading as a small negative error, and a correction
  too large for one 0.725 s gap being carried rather than truncated.
  `SlotAccum::phase_error` exposes the same number for the receiver to
  log.

- **The AP magnitude is upstream's per-protocol value, not one constant.**
  `apmag = max(|llr|) * scale` decides how far above the strongest
  channel observation an AP-locked bit is clamped. Both LDPC codecs
  hardcoded `1.01` and the doc called it "matching WSJT-X convention" —
  true of FT8 (`ft8b.f90:303`) and wrong for FT4
  (`ft4_decode.f90:327`) and FST4 (`fst4_decode.f90:418`), both of
  which use `1.1`. FT8 and FT4 share `Ldpc174_91`, so the codec cannot
  tell which protocol it is serving; the value is now
  `Protocol::AP_MAG_SCALE`, defaulting to FT8's.

  **Measured neutral**: the FT4 sweep is unchanged on three channels and
  −0.07 dB on the fourth, within sampling noise at 180 trials per point.
  A faithfulness fix, not a sensitivity one — worth having because the
  next person reading that constant would have believed the comment.

- **FT4's residual sensitivity gap against WSJT-X was a missing
  a-priori pass: −16.89 dB → −18.00 dB AWGN.** WSJT-X runs AP passes on
  *every* FT4 and FST4 decode — `ft4_decode.f90:328`'s
  `npasses = 3 + nappasses(…)`, whose `iaptype = 1` locks the first 29
  bits to the CQ pattern using no knowledge of the station at all.
  mfsk-core ran AP only when a caller supplied a hint, so a blind decode
  attempted none.

  FT8 has had the equivalent since issue #190, and adding it there is
  what closed FT8's own gap against its published figure.
  `BLIND_CQ_MIN_NSYNC`'s doc claimed `ap_passes`' pass 7 was the
  FT4/FST4 analog; it is not — pass 7 needs the correspondent's
  callsign, making it upstream's iaptype 2/3. Nothing corresponded to
  iaptype 1.

  | channel | before | after |
  |---|---:|---:|
  | AWGN | −16.89 dB | **−18.00 dB** |
  | CCIR good | −17.46 dB | −17.62 dB |
  | CCIR moderate | −15.71 dB | −16.33 dB |
  | CCIR poor | −16.00 dB | −16.25 dB |

  AWGN now sits 0.5 dB **ahead** of WSJT-X's published −17.5 dB, where
  it was 0.6 dB behind. The gap history in `FT4_BENCHMARK.md` runs
  1.8 dB → 0.8 dB (§9, sync) → 0.6 dB → ahead (§48, this).

  **Read the number for what it is.** The sweep corpus transmits
  `CQ JL1NIE PM95`, so the blind CQ prior is hinting the exact message
  being sent — the best case, and the same best case upstream's
  published figure enjoys, since WSJT-X runs the same pass. On mixed
  traffic it changes nothing and invents nothing: the WSJT-X golden
  still decodes 11/14 single-pass and 14/14 with SIC, both with
  `extra 0`. The three it cannot reach are exchange frames, which a CQ
  prior structurally cannot help.

  This depended on the scramble fix below. Until that landed, an
  always-on AP pass would have made FT4 *worse*.

- **FT4 and FST4 a-priori decoding locked about half its bits to the
  wrong value, and had done so for as long as it existed.** An `ApHint`
  describes the *message*; FT4 and FST4 XOR that message with their own
  RVEC before CRC and FEC encode, so the info bits inside the codeword —
  what the decoder actually works on — are the *scrambled* message.
  `ap_bits_for` handed the decoder message-space values, so wherever the
  RVEC bit was 1 the hint pinned a bit, with high confidence, to the
  opposite of the truth.

  That is worse than not hinting at all, and it showed: AP-hinted FT4
  decoding measured **worse than plain decoding**. On an AWGN sweep at
  12 trials per point, hinting the CQ the decoder was looking for:

  | SNR | plain | AP, before | AP, after |
  |---:|---:|---:|---:|
  | −17 dB | 8/12 | 3/12 | 12/12 |
  | −18 dB | 2/12 | 0/12 | 11/12 |
  | −19 dB | 0/12 | 0/12 | 8/12 |

  About 3 dB at threshold, which is what this crate's own documentation
  has always claimed AP is worth ("1-3 dB when the hint matches a
  station actually on air"). The claim was right; the implementation
  was not.

  **FT8 was never affected** — it has no RVEC — and that is the other
  reason this survived: the protocol where AP is most used is the one
  where it happened to work. It also explains why the sniper, whose
  entire premise is trading bandwidth for AP gain, had been losing to a
  plain wide-band decode on the same audio.

  `tests/ft4_ap_scramble.rs` pins gain at threshold and, on the axis AP
  can actually hurt, that a hint naming a station which is not
  transmitting does not produce that station — checked at three SNRs and
  against pure noise.

- **An AP hint, not a narrow search, was what ended the search.** The
  shared AP engine broke out of its candidate loop on `if has_ap`, so
  supplying a hint made the search single-target regardless of how wide
  it was. That is why `SupportsWideBandAp` is FT8-only — FT8 has its own
  AP path and never enters that engine — and it is why a generally
  useful option was reachable for FT4 and FST4 only through the sniper,
  a path that exists for a specific piece of radio hardware.

  **Decoupling it does not deliver wide-band AP, and the measurement is
  why this entry is longer than "removed one line".** Routing FT4's
  wide-band decode through that engine returns 4 decodes where the plain
  path returns 11 on the WSJT-X golden — and loses the hinted station
  itself. It returns the *identical* set for a hint naming a present
  station and one naming an absent station, so AP is changing nothing;
  the loss is entirely the different ladder. It invents nothing, so this
  is recall, not false decodes: `process_candidate_ap` offered OSD at
  depth 2 only, with no depth-3/4 escalation and no Top-K rescue, and
  most of that recording's decodes come from exactly those.

  **So the engine is gone rather than fixed.** AP is a rung at the end
  of `engine::pipeline::process_candidate_basic`'s own ladder —
  everything above it has already run and failed, so it can only add
  decodes — and it therefore reaches FT8, FT4 and every FST4 sub-mode.
  `SupportsWideBandAp` is implemented for all of them, `caps::AP_WIDEBAND`
  says so, and `msg::pipeline_ap` is 96 lines of hypothesis generation
  with no engine of its own.

- **`SniperRequest::search_hz`** — the ±250 Hz window is a parameter
  rather than a literal at each dispatch site. It is the one
  candidate-population lever the FST4 embedded work has never been able
  to measure (issue #306: a real decode costs ~55 ms against ~14 s for a
  pathological false survivor, and every optimisation so far has reduced
  cost *per* survivor rather than their number). Default unchanged, and
  a test pins that the default reproduces the old literal exactly.

  Note the caveat raised on that issue and not yet answered: this path
  also runs a halved sync gate, so a narrower band does not automatically
  mean fewer expensive candidates.

- **`.eq_mode()` was silently dropped on FT4's SIC path**, and the docs
  around it, `SniperRequest` and AP said things that are not true.

  The generic SIC engine hardcoded `EqMode::Off` per candidate and had
  no `eq_mode` parameter at all, so a caller combining `.eq_mode(Local)`
  with `.sic_rounds(n)` on FT4 got the setting accepted and ignored.
  FT8's own SIC engine has always honoured it. Threaded through; the
  default is unchanged, so nothing moves unless a caller asked for it —
  the FT4 tier-C sweep is +0.00 dB on all four channels, which it must
  be, since the value passed for a caller that does not set it is the
  `EqMode::Off` that was hardcoded before.

  **What that surfaced matters more than the fix.** Local equalisation
  is a property of the *input audio*, not of the search: it flattens a
  passband that an **analogue** filter has tilted. `SniperRequest`'s
  ±250 Hz window exists because the operator narrowed the transceiver's
  analogue roofing filter — a capability of few radios (FTDX101MP,
  FTDX10) — and pointed it at a DX station whose carrier is known, so
  the arriving audio is already band-limited. Sniper mode is that
  hardware's software half, not a general "hunt one known station"
  convenience, and EQ belongs beside it for the filter, not for the
  narrow search. Which is also why `DecodeRequest` carries EQ: filtered
  audio can be handed to a wide-band decode, and FT8's
  `eq_mode_recovers_bpf_edge_signal` pins a band-pass-edge signal that
  only `Local` recovers, through the wide-band SIC engine. On flat
  synthetic input EQ can only cost — `Local` loses two decodes of
  fourteen on the `ft4sim`-generated FT4 golden.

  **And A-priori decoding was coupled to sniper by accident** — see the
  entry above, which is where that was chased down and undone. The doc
  claim corrected here was narrower and also wrong: that the coupling
  was one line (`decode_sniper_ap`'s `if has_ap`). It was two things,
  the second being the shallower ladder AP lived in, and measurement is
  what said so.

  All three are now written down where they will be found: `CLAUDE.md`,
  `docs/reference/LIBRARY.md` §4 (and its `.ja.md` twin), and the
  `SniperRequest` / `ap_hint` / `eq_mode` doc comments. The old
  `SniperRequest` comment offered "after a 500 Hz hardware BPF (or when
  hunting one known station)" — correct premise, escape hatch attached,
  and the escape hatch is what gets remembered.

- **The generated C headers did not compile as standard C, and nothing
  would have noticed.** Three opaque handle types (`MfskDecoder`,
  `MfskDecodeOptions`, `MfskCallsignHashTable`) were emitted as
  `struct X { uint8_t _priv[0]; }` — a zero-length array member, which is
  a GCC/Clang extension that ISO C rejects under `-pedantic` and MSVC
  accepts only with a warning. They are now true incomplete types
  (`typedef struct X X;`), the shape `MfskFt8Stream` already had and the
  one every consumer treats them as. A pointer to an incomplete type is
  exactly as opaque; nothing changes at the binary level, since the
  handles only ever cross as pointers.

  Found by the new check rather than by a consumer, which is the point:
  `mfsk-ffi/tests/header_compile.sh` now compiles each header as the only
  thing in a translation unit, as C11 and as C++17, with warnings fatal.
  Both `cbindgen.toml` files have set `cpp_compat = true` from the
  beginning and nothing verified it — the C++ smoke driver includes the
  header after its own, so a header that only worked in that position
  would have passed.

- **A cbindgen failure is now fatal, and `mfsk-ffi-abi` is a rerun
  trigger.** Header generation failing was a `cargo:warning`, so a build
  that could not regenerate the header succeeded anyway and shipped
  whatever stale copy was committed. Separately, `cbindgen.toml` sets
  `parse_deps = true` to pull the shared `#[repr(C)]` types across the
  crate boundary, but cargo was never told — so editing `MfskResult` left
  the committed header describing the previous ABI with nothing to catch
  it. CI now also fails on a dirty `include/` after a build, which is the
  gate that was missing: the C++ driver compiles against the runner's
  freshly-regenerated copy, never the committed one, so a stale header
  could merge green.

- **The flash-and-capture scripts only ran on WSL, and one of their
  diagnostics never ran anywhere.** `flash-monitor.sh` and `capture.sh`
  were written against GNU coreutils and usbipd passthrough:
  `timeout --foreground` (absent from the BSD userland),
  `script -qfc CMD FILE` (GNU's argument order — BSD takes the file
  first and the command as argv), `sed -i` without the backup suffix
  BSD requires, and `/dev/ttyACM0` hard-coded as the port. They now
  source `embedded-poc/scripts/lib-platform.sh`, which holds each
  divergence once: the serial node per platform (`/dev/cu.usbmodem*`
  on macOS, globbed — `cu.` not `tty.`, which would block on DCD), a
  pty runner that picks the right `script` form and falls back to a
  bash watchdog when neither `timeout` nor `gtimeout` is installed, and
  a CR strip that writes-and-moves instead of `sed -i`. `capture.sh`
  skips the usbipd attach step off `mfsk_is_wsl` — macOS addresses the
  board directly, the CoreS3's console being the S3's own
  USB-Serial-JTAG — while its other four guards are platform-independent
  and still run. `flash-monitor.sh` also now checks for `espflash` up
  front: without it, `script` reported the missing command *inside the
  transcript*, which then failed the "Flashing has completed" check and
  told the operator their capture window was too short.

  The same misdiagnosis had a second cause, found by installing espflash
  and watching it fail: the port-failure check grepped for `Serial port
  not found`, `Device or resource busy` and `connection_failed`, none of
  which is what espflash 4.x prints. It says **`Error while connecting
  to device`** — the string `embedded-poc/CLAUDE.md` already quoted, so
  only the script did not know it. The most ordinary failure there is
  (board not plugged in, or booted into USB host mode and holding the
  port) therefore reported "the write did not finish" instead of
  "NOTHING WAS WRITTEN", pointing the operator at the capture window
  rather than at the cable. Verified against espflash 4.6.0. Logs are
  also written with `NO_COLOR=1` now, since espflash wrapped those very
  strings in ANSI escapes.

  Found while doing this, unrelated to platform: `capture.sh` dispatched
  its usbipd diagnostics with `case $?` after `if ! attach_if_needed`,
  where `$?` is the *negated* status and so always 0. The two arms that
  name the physical step — power-cycle the board, or `usbipd bind` from
  an admin shell — had been unreachable for as long as the block
  existed, and every failure printed the generic "check the cable". The
  status is captured on the failing branch now.

  Two claims in `embedded-poc/CLAUDE.md` corrected while there: that
  `flash-monitor.sh` passes `--before no-reset` (it passes no
  `--before`/`--after` at all, and `no-reset` would stop it reaching the
  bootloader — the repo-root CLAUDE.md already said so), and that the
  CoreS3 enumerates "via CH9102 bridge" (it is native USB-Serial-JTAG:
  the crate enables the `usb-serial-jtag` feature for
  `usb_serial_jtag_is_connected`, and `usb_host_install()` detaches the
  console, which a bridge could not do). Everything above is verified
  on macOS without a board; nothing that needs the CoreS3 plugged in is.

- **Three defects in FT4's slot grid, all found by reviewing the
  extraction above rather than by running it.** None had reached a tag;
  all three were live on `main`.

  **The first anchor desynchronised the grid from the audio.**
  `SlotGrid` counts the window's fill while `SlotAccum` holds the
  samples, and `push` advances both by the same `take` — so they are
  one number kept in two places. The first anchor reset the grid's
  half and not the buffer, which the in-place version it replaced had
  no need to do (it measured the fill as `audio.len()` directly). The
  path that reaches it is ordinary: `time_sync` reports no phase until
  the RTC or NTP lands, while UAC audio arrives regardless, so a
  CoreS3 accumulates a part-filled window and *then* gets a clock. The
  window would close holding that pre-anchor audio plus a whole
  window, and `Ft4SavgBuilder` stops emitting rows at `nhsym` — so the
  periodogram the coarse search ran on averaged the stale span rather
  than the anchored one. `anchor_or_reanchor` now returns
  `Anchor::DiscardPartial` when it re-phases under a part-filled
  window, `SlotAccum` drops that window, and the close carries the
  FT8 line's cross-check with it: a window that closes holding
  anything other than `CAPTURE_CLOSE_SAMPLES` says so, the way
  `stage1_inc::finalize_slot` compares `audio_fill` against the
  reported `total_samples`.

  **A clock correction was queued once per audio block, not once per
  window.** `phase_error` is a function of the grid's position, and
  both sides of it advance by the same `take`, so the same real offset
  answers the same number on every call — while the receiver asks once
  per UAC read, ~21 ms, ~320 times across a 6.775 s window. The branch
  exists to absorb NTP stepping an RTC-seeded clock, which is seconds;
  multiplied by the block count, a 1 s step became ~3.8 M samples of
  `pending_shift`, and `fill` clamps a gap to one period and carries
  the rest, so the receiver would skip whole slots for dozens of slots
  afterwards. `phase_error` is now net of what this cycle has already
  queued: the first call reports the error and queues it, the rest of
  the window reports `None`, and the close spends it and re-arms so
  drift keeps being corrected. The same subtraction fixes the log —
  the "off the clock — trimmed" line was one per block, on the serial
  link that also carries the decodes.

  **The anchor log could print more than a slot.** `remain` has the
  staged backlog added to it before the log, and staging holds up to
  `STAGING_CAP`, so a 7.5 s grid could report ~11 500 ms to the next
  boundary. The grid takes it modulo the period internally; the line
  an operator reads to confirm the anchor now does too.

  Verified the way the rest of this module is: the three cases added
  for these fail against the code as it was, checked one at a time.
  Also here, from the same review: the piecewise-fill case asserted
  something that could not fail (`!closed || room == WINDOW` is true on
  every non-final push and, on the last, `fill` has already reset the
  counter), and now pins that the window closes on exactly the final
  piece; a bare intra-doc link that did not resolve; and four stale
  `6.25 s` / `1.25 s` figures left from before the capture close moved
  to 6.775 s. One of those, `RX_ONLY_BUDGET_MS`' "conservative by
  1.25 s ... `7.5 + 6.25`", could not be reconstructed as a claim at
  all, so it now states the relationship the constants support: the
  next window opens 0.725 s after this one closes, and a decode past
  that spends staging rather than the grid.

- **`wait-for-ci` spent 30 minutes to misdiagnose a commit that has no
  CI.** `ci.yml` `paths-ignore`s `embedded-poc/**` and `bench/wasm/**`,
  so a push touching only those trees starts no workflow run at all —
  and `release.yml`'s poll read "no run yet" as "the scheduler has not
  caught up", waited out its whole 30-minute cap, and failed with
  `Timed out waiting for CI`. That reads like a GitHub flake, and sends
  the reader to the Actions tab rather than to the tag. It is now a
  three-minute grace period, after which the failure says what is
  actually wrong: this commit starts no CI, nothing can gate the
  publish on it, and the tag belongs on a commit that has a run — which
  need not be the newest commit on `main`. Found by walking up to it:
  `main` sat at exactly such a commit (an `embedded-poc/CLAUDE.md` fix)
  while this release waited to be tagged.

  Refusing to publish was always the right outcome, and still is. Only
  the diagnosis changed.

  Also corrected here: the second gate's own comments, and `CLAUDE.md`'s
  description of it, still called the job it requires `Test (default)`.
  It has been `Test (tier A+B — invariants + golden)` since the tier
  rework, which is what the code matches on — the comment was the stale
  half, so the gate works and its documentation did not describe it.

- **The publish gate read a job's conclusion, which cannot tell "the
  goldens ran" from "the job was gated out" (#370).** `ci.yml`'s test
  job puts its suite behind `if: steps.gate.outputs.should_run ==
  'true'`; when that is false the conditioned steps conclude `skipped`
  and **the job still concludes `success`**. So the one check standing
  between a release and the vacuous-golden failure it was written for
  was satisfiable by a job that ran nothing.

  What kept it honest was a rule in a different file: `ci.yml`'s
  stage-2 gate short-circuits to `should_run=true` for every suite on
  `push` events, before consulting the paths filter, and `release.yml`
  only inspects push-event runs. True today, asserted nowhere, and a
  reasonable CI-minutes optimisation — "a docs-only push to main need
  not re-run the matrix" — would have flipped exactly that branch and
  quietly unhooked the gate, with the comment above it still claiming
  the opposite.

  The gate now reads the conclusion of the `Run suite` **step** inside
  that job, and fails closed when the step is absent. A rename on
  either side stops a release instead of passing it silently, and
  `ci.yml` says so at the step. Demonstrated against real API
  payloads, both directions: the main-push job for `b6f59181`
  (`Run suite: success`) is allowed, the `pull_request` job for the
  same tree (`Run suite: skipped`, job green in 5 s) is refused — and
  the pre-fix gate allowed that second one, which is the bug.

- **`docs/notes/WSPR_EMBEDDED_MEASUREMENT_PLAN.md` was cited from nine
  places and had never been committed.** The whole WSPR-on-embedded
  track points at it — `WSPR_EMBEDDED_MEASUREMENT_RESULTS.md`'s opening
  line links to it relatively (so, broken on GitHub), both
  `wspr_bench` binaries and `embedded-shared`'s copy name it in their
  module docs, `wspr::instrument`'s doc says it was written for its
  Phase 1, `wspr_wsjtx_samples.rs` marks the host side of that phase,
  and three `Cargo.toml`s justify a feature by it. The file itself
  only ever existed on the branch it was written on, which was never
  merged: the plan was written, the measurement was run from it, the
  results were committed, and the plan was left behind.

  Committed now as it was written, with one addition — a pointer
  forward to the results, since a reader arriving from any of those
  nine citations wants the answer and the plan predates it. Found while
  clearing out stale branches, which is the only reason the last copy
  was still reachable.

- **`timer.out` was a committed build artifact.** WSJT-X's Fortran
  `timer` module writes it into the working directory of whatever ran
  it, so benchmarking against the real `jt9` from a checkout beside
  this one drops a copy in the repo root. One was committed in
  `641b5ff3` (the JT65 SNR-clamp audit, #255) and had been on `main`
  ever since — 342 bytes reporting a 0.012 s `jt9` run against 85
  `read_wav` calls, which describes one invocation on one machine in
  2026 and nothing else. Removed, and `.gitignore` now covers the name
  so the next benchmark run does not re-add it.

### Added

- **Embedded FT8 (CoreS3): per-candidate fine sync in the per-slot
  pipeline, a coarse-position retry, one row per message, and a stage-3
  bound anchored to key-up.** `embedded-shared::dual_core` now runs
  `fine_sync_12k` on both cores between coarse sync and the prefix
  partition (`DecodeConfig::fine_sync`). A candidate that fails stage 3
  at its refined position is retried once at its coarse one — fine sync
  alone erased stations the coarse position decodes (`N1PJT HB9CQK`,
  `CQ LZ1JZ KN22`), and the retry is what makes it a strict addition.
  Retries are the lowest-value work (0.26-0.46 decodes a slot recovered
  from 7-13 retries on the host mirror), so the early path's run only in
  the tail window and yield the cores the moment the slot arrives; the
  rest follow every first attempt, deferred ones included.

  Results are deduplicated by message: fine sync pulls adjacent coarse
  cells onto one carrier (31 duplicate rows over 201 phases of `qso2`,
  against 7), and the slot count, the grid-lock policy and the QSO state
  machine all read the unfiltered list.

  `DecodeConfig::key_up_guard_ms` stops stage 3 claiming candidates that
  long before this station's key-up (slot end + 0.5 s), peeking the slot
  end from `slot_q` before the slot is received. `budget_ms` alone was
  derived assuming the SpecBundle arrives ≥1 336 ms before slot end;
  measured 1 027-1 936 ms depending on grid position, and below 1 336 ms
  its deadline fell after key-up — slots finished up to 472 ms past it.
  A candidate already in BP runs on after any deadline, by up to 313 ms
  here, hence a 320 ms guard. The slot-log warning now fires on key-up
  rather than on zero idle, which fine sync makes routine.

  On the board (`MFSK_CORES3_SIM`, `qso3_busy`), steady-state decodes
  went from 6 to 10 a slot, finishing 158-291 ms after slot end. Only
  the CoreS3 sets these; the S3 and Core2 apps keep `fine_sync: false`
  and `key_up_guard_ms: 0`, their slot budgets unmeasured with it.

- **`ft8::decode_block::fine_sync_12k` — WSJT-X's per-candidate fine
  sync, on 12 kHz audio.** `ft8b.f90`'s Stage A (DT ±50 ms in 5 ms),
  Stage B (frequency ±2.5 Hz in 0.5 Hz, via `ctwk`) and Stage C (DT
  ±20 ms at the new frequency), with upstream's step sizes and
  first-maximum tie rule. Upstream computes them on a 200 Hz baseband
  cut by a 192 000-point FFT the embedded planner does not carry, so the
  embedded path had skipped all three since 0.6.3 and decoded from
  coarse sync's 3.125 Hz / 40 ms grid. Here each Costas symbol is mixed
  once into 60-sample bins — one per cd0 sample — and every stage re-sums
  them, ~76 k two-multiply steps a candidate instead of ~1.65 M.

  Measured on `tests/ft8_embedded_pipeline_mirror.rs`, which reproduces
  the CoreS3 per-slot pipeline call for call (checked against the
  board's own `p1/ready/defer/dec` and message set), with the caller
  retrying a failed candidate once at its coarse position: `qso3_busy`
  goes from 6.35 to 8.53 decodes a slot across 201 grid phases, and from
  5.79 (none reaching 8) to 8.29 (96 % reaching 8) including acquisition
  from 30 starting misalignments. On the held-out recordings the same
  acquisition-inclusive score holds (`qso1` 3.88 → 3.88) or rises
  (`qso2` 4.43 → 4.96). The station it adds on `qso3_busy` is
  `K1JT EA3AGB`, whose decodable region sits 1.0-1.5 Hz below its coarse
  bin. No host decode path calls it, so no host sensitivity moves.

- **The tier-C sweeps every decode-path change in this section asked
  for, run before the tag.** `scripts/release-status.sh` named FT8, FT4
  and FST4; FST4 was already covered (its sweep ran after the AP-engine
  deletion), so FT8 and FT4 were the outstanding pair.

  Both were run against specific hypotheses rather than as a formality.
  **FT4**: `apmag` going per-protocol (1.01 → 1.1, touching both LDPC
  codecs) and the parallel AP engine being deleted in favour of a rung
  on the shared ladder — 89 lines out of `ft4/decode.rs`, a different
  code path reaching the same decodes. **FT8**: the budget scheduler's
  candidate reordering, which re-sorts to coarse order before the
  first-wins dedup and so can change a dedup outcome even with no budget
  set.

  Eight cells: seven bit-identical, one (FT4 CCIR-moderate) 0.07 dB
  better, which at 180 trials per cell is interpolation granularity.
  Both hypotheses negative. Written up in `FT4_BENCHMARK.md` §49,
  including the correction that §48's "`apmag` measured neutral" was a
  spot check and this is the sweep that agrees with it — the change is
  still right, because it is what upstream does, but it buys nothing
  measurable and saying so beats letting "we matched WSJT-X" imply a
  gain.

- **`bindings/kotlin/` — a maintained Kotlin binding, built and run on a
  desktop JVM by CI on every source change.** It replaces
  `mfsk-ffi/examples/kotlin_jni/`, which was written against the pre-v2
  ABI, marshalled results as pipe-separated strings, and was never built
  by anything — `LIBRARY.md` §9 described its own Android text as
  "aspirationally" written.

  `Mfsk` carries introspection and transmit; `MfskSession` is the decode
  handle and is `AutoCloseable`. `MfskDecode` is a `data class` — a
  value, not a handle, because the ABI writes rows into caller memory,
  so nothing has to be freed and nothing can outlive a session.

  **`Mfsk.configureRuntime` is the Android-specific part**, and the
  reason `mfsk_runtime_configure` exists. Rayon's global pool is plain
  pthreads the VM has never attached, so nothing running on one can
  touch a JNIEnv — a decode callback from a worker thread is not merely
  discouraged there, it is illegal. The shim's thread hooks call
  `AttachCurrentThread`/`DetachCurrentThread`, which is what makes it
  legal.

  The JVM test round-trips real audio and checks field values, because
  the risk in a thin binding is marshalling rather than decoding: a JNI
  signature that disagrees with the Kotlin declaration fails at run time
  with `UnsatisfiedLinkError`, not at compile time, and a field read in
  the wrong order produces plausible garbage.

  **A real bug the JVM test found on its first run**: the shim attached
  rayon's workers with `AttachCurrentThread`, which makes them
  *non-daemon* JVM threads — and the JVM will not exit while one is
  alive. Rayon's pool threads are never joined, so the process printed
  `ALL OK` a minute in and then hung until the run was cancelled
  seventy-two minutes later. `AttachCurrentThreadAsDaemon` is the fix.
  `build.sh` now wraps every stage in `timeout` and announces each one,
  so a regression here fails with a message naming the stage instead of
  stalling with no output to diagnose from.

  **It costs no wall clock**, which is worth stating because it was
  briefly moved to release time on the belief that it was slow. Measured
  on the runner: `kotlinc` install 1 s, and build + compile + test 55 s,
  against the ~300 s `Test (tier A+B)` job that gates every run anyway.
  The 72-minute first run was the hang below, not a cost. The same
  measurement applies to the two jobs beside it — `ffi` at ~120 s and
  `cross` at 90-140 s are both inside the same ceiling.

  Swift follows in the next entry, on the same argument minus the CI
  half: it was written where it could actually be run, so the objection
  that stood here — unverified code on `main` — no longer applies. What
  a macOS runner would buy is keeping it that way.

- **A `swift` CI job, and with it the iOS build the `cross` job could
  not do.** `macos-latest`, running `bindings/swift/scripts/test.sh`
  (the XCTest suite) and then `cargo build -p mfsk-ffi --release --target
  aarch64-apple-ios --no-default-features --features mobile`. One
  runner covers both because both halves need Xcode — XCTest ships with
  it rather than with the Command Line Tools, and the iOS SDK is its
  too.

  This is the repo's first macOS runner. `ci.yml` used to say iOS was
  "deliberately absent … the plan puts it at tag time", accepting that
  a break would land on `main` and be found at release; the platform
  the C ABI claims to support was built by nothing. Both halves are now
  covered per-PR by one job.

- **The four capability bits that no C entry point could reach.**
  `MFSK_CAP_BUDGET`, `MFSK_CAP_KNOWN_FILTER`, `MFSK_CAP_KNOWN_SUBTRACT`
  and `MFSK_CAP_FFT_CACHE` were published per mode by `mfsk_mode_caps`
  while `.budget()`, `.known()` and `.fft_cache()` existed only on the
  Rust `DecodeRequest`. A consumer read the bit, believed the mode did
  it, and found nothing to call — worse than an absent capability,
  because the ABI's whole introspection story is that the bits can be
  trusted. Three session-scoped entry points close it:

  - `mfsk_session_set_budget(dec, check, user)` polls a caller-supplied
    predicate during the search; returning `false` stops it and returns
    what was found. **The library still reads no clock** — the deadline
    is the caller's, as with the capture ring's slot grid.
    `mfsk_session_last_budget` reports what was left undone, including
    the sync quality of the best *skipped* candidate, which on FT8 is
    the scheduler's own ranking key and so says whether the cut took
    noise or a station. Absent is `-1` and NaN, not 0, for the same
    reason `freq_hint_hz` uses NaN.
  - `mfsk_session_keep_known(dec, keep)` carries a decode's results
    into the next one on that session as known signals — skipped, or
    subtracted where the mode publishes `_KNOWN_SUBTRACT`. The engine's
    results, not the C rows: reconstructing them from what this ABI
    hands back would lose what the subtraction needs, and the session
    already holds them. `mfsk_session_known_count` says how many.
  - `mfsk_session_keep_fft_cache(dec, keep)` reuses the slot transform
    for a second pass over the same audio — FST4-300's is 4 194 304
    points. **Reuse is checked, not trusted**: the cache is stored with
    an FNV fingerprint of the audio it was built from, and a decode of
    anything else transforms afresh. A cache reused against different
    audio is a confident wrong answer with nothing to signal it, and
    "the caller promised the buffers matched" is not a check.

  Both bindings expose all three (`setBudget` / `keepKnown` /
  `keepFFTCache` in Swift, the same in Kotlin with a `MfskBudgetCheck`
  `fun interface`), and the C++ driver exercises them against a
  two-station slot. With this, **every function in `mfsk.h` is reachable
  from both bindings, and every capability bit has an entry point.**

- **`mfsk-ffi/README.md` documented an ABI that no longer exists.** Its
  surface table, quick-start and ownership sections were all pre-v2:
  `mfsk_decoder_new`, `MfskResultList`, `mfsk_result_list_free`,
  `mfsk_samples_free`, `MFSK_PROTOCOL_*`. Every one of those was deleted
  in this same release, so the file's only runnable example could not
  compile and its memory rules described allocations that no longer
  cross the boundary. Rewritten against the current header: the table is
  grouped the way the header is, the quick start is the three-stage
  transmit plus a session decode with nothing to free, and "Mode
  selection" now shows the introspection loop rather than a hardcoded
  list of seven tags.

- **`mfsk_session_set_on_decode` in both bindings** — rows delivered as
  they are found, on top of the list the call returns, which stays
  authoritative. It exists for a UI with a long slot to fill:
  FST4-300's is five minutes.

  Swift: `session.onDecode { row in … }`, the closure boxed so it has a
  stable address, retained by the session for exactly as long as the C
  side holds a pointer to it, and cleared in `deinit` **before** the
  handle closes.

  Kotlin: `session.onDecode { row -> … }` over a `fun interface`, with
  two JNI hazards handled in the shim rather than left to the consumer.
  The callback can arrive on a thread the VM has never seen — only a
  *private* pool from `configureRuntime` gets the attach hooks, so a
  process that never called it has unattached rayon workers — and it
  attaches on demand, `AsDaemon` for the same reason the hooks use it.
  And the listener's method ID is taken from the **interface**, not
  from `GetObjectClass` on a lambda: a Kotlin `fun interface` is
  satisfied by an invokedynamic-spun class, and `FindClass` from an
  unattached worker would resolve through the system class loader,
  which cannot see application classes at all. The shim also captures
  the `JavaVM` in `JNI_OnLoad` now instead of only in
  `configureRuntime`, which removes the ordering requirement that
  created.

  An exception from a listener is printed and cleared: a rayon worker
  has nowhere to propagate one, and leaving it pending would make every
  later JNI call in that callback illegal. The decode is unaffected.

- **Q65 in the Swift binding**, now that its sub-mode numbering reaches
  the header: `Q65` carries all four strategies (plain, a-priori,
  fast-fading, AP-list), `Q65SubMode` and `Q65FadingModel` mirror the
  newly-emitted C enums, and `CallsignHashTable` is the one handle a
  caller owns rather than the session.

  `Q65SubMode` is a separate Swift type from `Mode` on purpose — Q65's
  discriminants are its own and not in slot-length order (`a15` is 6,
  appended so the earlier numbers stayed put) — with `.mode` bridging
  to the identity a decode row reports and `Q65SubMode(_:)` bridging
  back. `ABIContractTests` pins all ten against the header's constants.

  Writing the tests turned up a trap worth the documentation it now
  has: **an AP hint's fields are the message's fields in order, not
  roles.** `call1` is the first callsign field — `"CQ"` for a CQ, not
  the transmitting station — and a hint locks message bits rather than
  steering a search, so hinting the right callsigns in the wrong fields
  is not a weaker hint but a wrong one. On Q65 the clean signal that
  decodes four other ways then does not decode at all; on FT8 the AP
  rung is one of several, so the same mistake passes unnoticed, which
  is how it got written that way here first. Both directions are
  asserted in `Q65Tests`, and `DecodeParams.APHint`'s own doc comment
  was corrected with it.

- **Swift bindings — `bindings/swift`, a SwiftPM package over the C
  ABI.** `Mode` / `ModeInfo` / `Capabilities` for introspection,
  `DecodeParams` and `DecodeSession` for decoding (`[Int16]` and
  `[Float]`), `CaptureStream` plus the fused `session.decode(stream)`
  for live audio, `Message` and `Mode.synthesiseSlot` for the three
  transmit stages, `WSPR` / `JT9` / `JT65` for the modes with no decode
  handle, `Runtime` for the thread pool and the two version numbers.
  Errors throw as `MfskError`, carrying the status code and the reason
  string the call recorded.

  `Package.swift` carries no `unsafeFlags` — a package that has them
  cannot be used as a dependency, which for a binding is fatal — so the
  library search path comes from the caller;
  `bindings/swift/scripts/test.sh` supplies it, and on macOS also
  points `DEVELOPER_DIR` at Xcode, since XCTest ships with Xcode and not
  with the Command Line Tools. The module map includes
  `mfsk-ffi/include/mfsk.h` in place rather than copying it: CI already
  fails when that committed header drifts from the library, so there is
  nothing further to keep in step.

  43 tests, ~0.4 s, end-to-end in the same shape as the C++ driver —
  synthesise a known message, decode the PCM back, check the text
  survives — for FT8, FT4, WSPR, JT9, JT65 and the capture ring. Two of
  them exist because they are the only ABI numbers the binding restates:
  every `MfskMode` discriminant and every `MFSK_CAP_*` bit is checked
  against the header's own constant. Q65 is deliberately absent and
  `bindings/swift/README.md` says why: its sub-mode discriminants never
  reach `mfsk.h`, so a wrapper would have to hardcode them.

  Covered by CI on the same terms as the Kotlin binding — see the
  `swift` job below, which runs this suite on `macos-latest` and builds
  `aarch64-apple-ios` while it is there.

  Two library defects turned up while writing it, both fixed in this
  section: MSK144's `slot_samples_12k` (above), and
  `mfsk_session_copy_info` recording its failure only in the
  thread-local slot because it takes the handle as `const*` — so the
  binding reads the handle first and falls back to the global rather
  than reporting "(no detail)" for that one call.

- **`MFSK_API`, and a header that says how to link it.** Every
  declaration now carries an export/visibility macro: `__declspec`
  on Windows, `visibility("default")` elsewhere. The header had none,
  which means a Windows DLL exported nothing linkable and a Unix shared
  object exported every non-static symbol including Rust internals.
  Define `MFSK_STATIC` for the static library, `MFSK_BUILDING` when
  building the DLL.

  The calling convention is documented rather than emitted: cbindgen can
  place a prefix before the return type but not in the `__cdecl`
  position between return type and name, and a macro that cannot go
  where MSVC needs it would be worse than `extern "C"`'s default (which
  *is* `__cdecl` for every signature here). `LIBRARY.md` §8 gains a
  per-platform link table; the repo's single Unix `-ldl` line was wrong
  on all three new targets.

- **`mfsk_runtime_configure` — the host decides how threads are used.**
  Even with `parallel` on, decoding used rayon's **global** pool:
  `num_cpus` threads with 2 MiB stacks, spawned lazily on the first
  decode and never joined. On Android those threads are not attached to
  ART, so a callback from one cannot touch a JNIEnv; on iOS they sit
  outside GCD's quality-of-service classes, competing with the audio
  render thread; on both they keep running after the app is
  backgrounded. There was no hook to change any of that, at any layer.

  Decoding now runs inside a private pool when one is configured.
  `on_thread_start`/`on_thread_stop` map onto rayon's
  `start_handler`/`exit_handler`, which is what makes
  `AttachCurrentThread`/`DetachCurrentThread` possible from JNI and
  therefore what makes a decode callback legal from a worker thread
  there. `num_threads = 1` forces serial decoding.

  Configuring it once is enforced: a second call returns
  `MFSK_STATUS_UNSUPPORTED` rather than being silently ignored, because
  rayon cannot rebuild a pool its threads may be parked in.
  `mfsk_runtime_thread_count` is how a caller checks it took effect, and
  the test asserts that rather than that the call returned OK — a pool
  accepted and then not used would look identical from outside.

  No `unsafe` was needed to install it: every field of the session is
  already `Send`, the one raw pointer having carried that claim since
  the callback was added.

- **Cross-compilation is checked on every source change.** A `cross` CI
  job builds `x86_64-pc-windows-gnu` and `aarch64-linux-android`. The
  repo previously contained **zero** occurrences of any Windows, iOS or
  macOS target triple, while `LIBRARY.md` §9's Android section was
  described by the repo itself as "aspirationally" written.

  The job asserts rather than assumes: that the Windows DLL actually
  exports six named symbols (which is what `MFSK_API` is for), that the
  Android `.so` is 16 KB page-aligned, and that the `mobile` feature
  really has no rayon in its dependency tree.

  **Android 15 ships devices with a 16 KB kernel page size**, and a
  `.so` linked for 4 KB does not load there — surfacing as
  `UnsatisfiedLinkError` on exactly the newest hardware.
  `.cargo/config.toml` sets the link flag for the three Android
  triples, and the CI assertion exists because setting `RUSTFLAGS` in
  the environment **overrides** `target.*.rustflags` rather than merging
  with it. This workflow sets `RUSTFLAGS: -D warnings` globally, so the
  flag was one edit away from being silently dropped — the job passes it
  explicitly and then checks the ELF.

  iOS is deliberately still absent: it needs a macOS runner and the plan
  puts it at tag time. The risk is a break landing on main and being
  found at release, mitigated by iOS being a pure cross-compile of code
  Linux and Android exercise here.

- **Streaming capture, generalised off FT8.** `mfsk_stream_open` /
  `_push_i16` / `_push_f32` / `_buffered` / `_slot_ready` /
  `_take_slot_i16` / `_set_epoch` / `_clear` / `_close`, plus the fused
  `mfsk_session_decode_stream`.

  `mfsk-ffi-ft8` had the only streaming front end in this repo, sized
  for FT8 and taking i16 alone. The ring is sized from
  `slot_samples_12k` now, so FST4-300's 3.6 M-sample slot works the way
  FT4's 90 000-sample one does — a factor of 40 an FT8-sized ring got
  wrong in both directions.

  **Time is a parameter and is never read.** No `Instant`, no
  `SystemTime`, no clock of any kind: the host says what UTC second the
  next sample belongs to and the grid does arithmetic. That is what
  keeps this usable from wasm, from `no_std`, and from a phone that was
  backgrounded for four minutes — the same choice `BudgetCheck` makes
  for the decode deadline. Without an epoch the grid free-runs from the
  first sample, which is exactly right for replaying a recording. A
  test runs the same capture twice and asserts an identical reported
  time, which no wall-clock implementation could pass.

  `mfsk_session_decode_stream` is fused because taking FST4-300's slot
  out and handing it straight back in moves 7 MB for nothing. Polling it
  before a slot is ready returns `MFSK_STATUS_UNSUPPORTED` with
  `*out_len = 0` — a "not yet", not a failure to guard against.

- **Transmit is the same shape as receive: nothing crosses the boundary
  as an allocation.** `mfsk_pack77` / `_type1` / `_type4` /
  `_free_text` → `mfsk_message_to_tones` → `mfsk_tones_to_i16` /
  `_to_f32`, each stage writing into a buffer you sized with
  `mfsk_symbol_count` and `mfsk_synth_output_len`, plus `mfsk_unpack77`
  to read a packed message back.

  `mfsk-ffi-ft8` already had the better of this repo's two TX designs;
  `mfsk-ffi` had seven heap-allocating `mfsk_encode_*` functions that
  accepted only the three-string `call1 call2 report` path — so a caller
  with a type-4 or free-text message had no way in, and FST4 reached 60A
  alone. The pipeline now generalises over `MfskMode` and covers FT8,
  FT4 and all five FST4 sub-modes.

  **Ask for the size rather than baking it.** The FST4 sub-modes differ
  by a factor of 30 in samples per symbol (720 → 21 504), so a constant
  taken from 60A is silently wrong for the other four.

  The `mfsk_encode_*` convenience calls stay for the modes with no
  exposed tone stage (WSPR, JT9, JT65, Q65) and now write into a caller
  buffer too, which retires `MfskSamples` and `mfsk_samples_free`.
  `mfsk_symbol_count` returning 0 is how a caller asks which is which.

- **A decode session: one struct of parameters, rows into caller memory,
  and a callsign table that survives the slot.** `mfsk_session_open` /
  `_decode_i16` / `_decode_f32` / `_copy_info` / `_add_callsign` /
  `_last_error` / `_close`, plus `mfsk_decode_params_init`.

  Three defects it removes, in order of how much they mattered:

  **Hashed callsigns could never resolve over this ABI.** Every decode
  call built a fresh empty `CallsignHashTable` (`:927`, `:987`,
  `:1137`), so a `<...>` reference had nothing to look up — for any
  protocol, in any call, since the ABI was written. A table is worth
  something only if it outlives the slot that populated it, which is
  why the session owns one; it is fed from each decode and from
  `mfsk_session_add_callsign` for callsigns known from a band map or a
  log. `tests/v2_decode.rs` pins both halves against a type-4 message,
  whose text reads `<...> JA1ABC/QRP` until the table has seen the
  standard call.

  **The options handle was unsound.** `options_inner_mut` fabricated a
  `&'static mut` from a raw pointer with no synchronisation, which is UB
  under Stacked Borrows the moment two setters' borrows overlap —
  single-threaded, never mind concurrently. `MfskDecodeParams` is a
  plain `#[repr(C)]` struct the caller owns, which has no such question,
  and replaces eight fallible setter calls with one marshalling step for
  Kotlin and Swift.

  **Unsupported options were dropped in silence**, and one was silently
  *upgraded*. `ap_hint` reached FT8 only, `sic_rounds` FT8+FT4,
  `strictness` was a no-op on FST4 — and `MFSK_DECODE_DEPTH_BP_ALL` was
  quietly promoted to the full ladder, so a caller asking for the cheap
  path paid for the expensive one. On FST4-300 that ladder sits behind a
  4 194 304-point transform. Now `mfsk_session_open` returns
  `MFSK_STATUS_UNSUPPORTED` with a message naming the mode and the
  capability, and a mode with no decode handle at all is refused there
  rather than failing later with a confusing complaint about `max_cand`.

  Rows go into a caller-allocated `MfskDecode[]`, which deletes the
  "a Rust global-allocator pointer crosses the boundary and must come
  back to be freed" category for decode results. A short buffer returns
  `MFSK_STATUS_INVALID_ARG` with `*out_len` set to the count needed, so
  a caller can size and retry rather than get a truncation it cannot
  detect. The FEC information bits stay off the row — 91 or 101 bytes
  only a subtracting or persisting caller wants — and come back through
  `mfsk_session_copy_info`.

  `MfskDecode::mode` is the **concrete sub-mode**: `Decoded::protocol`
  collapses all five FST4 periods onto one id and the C row must not,
  because the addressing model does not.

- **Three things the type system and the new gates caught**, each of
  which would otherwise have shipped:

  **A `memset`-to-zero params struct was undefined, not merely
  invalid.** `MfskDecodeDepth` left discriminant 0 deliberately
  unassigned, so the commonest C idiom produced a value Rust has no
  variant for — and reading it as a Rust enum is UB, not a wrong
  answer. Found by `rustc` refusing to zero-initialise the type in a
  test. Fixed at both ends: `MFSK_DECODE_DEPTH_MODE_DEFAULT = 0` gives
  zero a meaning, and `read_params` validates every enum and bool field
  *as an integer* before the caller's bytes are ever read as Rust
  types. That is the same class of defect as the `&'static mut`
  accessor being replaced, so it would have been a poor thing to
  reintroduce in the replacement.

  **Two handle types were sharing one opaque C type.** The new session
  and the legacy `MfskDecoder` own different Rust values, and a pointer
  that wandered from one to the other's free function was UB no compiler
  could notice. `MfskDecodeSession` is a distinct incomplete type, so
  mixing them is a C type error. ("Session" is also the right word: it
  owns a hash table and the previous slot's rows, both of which only
  mean anything across more than one call.)

  **The generated header did not compile as C++.** `MFSK_AP_FIELD_LEN`
  and `MFSK_DECODE_TEXT_LEN` live in `mfsk-ffi-abi`, and cbindgen writes
  the *name* into the struct it generates while being unable to emit a
  `#define` for a dependency's constant — so `char text[…]` referenced
  an undeclared identifier. Caught by `tests/header_compile.sh`, added
  earlier in this same section; the fix is the same one the capability
  bits needed.

- **The float decode path no longer loses dynamic range at 12 kHz.**
  `resample_f32_to_12k` interpolates in f64 and peak-normalises to 0.8
  full-scale before quantising, but the pre-v2 path took that route only
  when a resample was needed and did a bare `(s * 32767.0) as i16` at
  12 kHz. So a quiet float buffer — a USB radio adapter at a low system
  volume, which is the case that normalisation exists for — was worse
  off at the one rate that needed no work. The session converts through
  the normalising path unconditionally.

- **The C ABI can be asked what it supports, instead of being guessed
  at.** `mfsk_version()` was the entire introspection surface, while
  `registry::PROTOCOLS` — which knows every wired mode with full
  geometry — was not exposed at all, so every C consumer that needed to
  know what a mode supports hardcoded a matrix. The existing
  `MfskProtocol` enum *is* that rot made visible: it has one FST4 entry,
  so four of the five wired FST4 sub-modes were unreachable from C for
  decode and encode alike.

  ```c
  uint32_t    mfsk_mode_count(void);                 /* modes in THIS build */
  MfskStatus  mfsk_mode_at(uint32_t i, MfskMode* out);
  const char* mfsk_mode_name(MfskMode);
  MfskStatus  mfsk_mode_from_name(const char*, MfskMode* out);
  MfskStatus  mfsk_mode_info(MfskMode, MfskModeInfo* out);
  uint64_t    mfsk_mode_caps(MfskMode);
  MfskStatus  mfsk_mode_defaults(MfskMode, MfskDecodeDefaults* out);
  uint32_t    mfsk_abi_version(void);
  ```

  `MfskMode` addresses all 25 modes — every registry entry plus MSK144 —
  with **discriminants that are ABI and are never reordered**. They are
  deliberately not registry indices: registry membership is
  feature-gated, so a build without `q65` would shift every index after
  it while these stay put. `mfsk_mode_count`/`mfsk_mode_at` report what
  a given build actually has (21 for the default `desktop` feature set).
  This ABI already learned that lesson in `MfskQ65SubMode`; relearning
  it costs silent misdispatch at a C boundary.

  Additive: no existing symbol changed, and `MfskStatus` gained
  `MFSK_STATUS_UNSUPPORTED = -6` without touching the values before it.

- **Capabilities cross the boundary as named bits, and cannot drift.**
  Fifteen `MFSK_CAP_*` `#define`s, of which `MFSK_CAP_DECODE_HANDLE` is
  load-bearing: it says whether `mfsk_decode_i16` applies at all. Q65
  takes a nominal start sample and a time tolerance and reports
  `start_sample` rather than `dt`; WSPR/JT9/JT65 have no builder. They
  are not lesser, they are shaped differently, and that is now a bit a
  caller reads rather than a fact it has to know.

  The drift chain is closed at both ends. `mfsk-core`'s
  `tests/registry_caps.rs` ties the registry bits to the trait impls in
  both directions — naming a protocol that lacks a trait is a *compile*
  error, implementing one without setting the bit fails at runtime — and
  `mfsk-ffi/tests/mode_introspection.rs` compares each `MFSK_CAP_*`
  against the registry constant it mirrors, stated one pair at a time so
  a loop built from the same source cannot make it vacuous. Verified by
  moving a bit and watching it fail.

  The constants are literals in `mfsk-ffi` rather than re-exported from
  the shared ABI crate because cbindgen emits a root-level literal
  `pub const` from the crate it generates from and **cannot evaluate one
  that references another crate's path** — measured, not assumed. Left
  in the dependency they reach C as an unnamed `uint64_t`, which is the
  failure this surface exists to end.

- **`mfsk_mode_defaults` removes the ABI's worst trap.** Three different
  per-protocol NULL-option defaults lived inside one function, one of
  them an FT8 `sync_min` of 2.0 that no test in the tree uses. Defaults
  are data now, published per mode — and `MfskDecodeDefaults::sync_scale`
  says whether two modes' numbers are comparable at all. FT4's spectrum
  is divided by a fitted baseline before scoring, so noise sits at ~1.0
  **by construction** and WSJT-X's own 1.2 (`ft4_decode.f90:195`) is a
  floor rather than a preference; FT8's and FST4's are absolute Costas
  scores. Copying one across modes is wrong and nothing said so.

- **`Protocol::DECODE_FFT1_SIZE`, published as
  `MfskModeInfo::decode_fft1_size`.** The forward FFT the decoder takes
  over the whole slot — FT4 92 160 points, FST4-300 **4 194 304**. A
  factor of 45 that no other registry field hints at, and the reason
  "one call shape for every mode" is wrong as a memory story on a phone.
  A host deciding which modes it can afford could not previously ask.

  Written as a literal on each `impl Protocol` rather than read from the
  `DownsampleCfg`, because those consts sit behind an FFT backend
  feature while the trait does not — a `--features fst4` build has the
  protocol and no downsampler. `tests/registry_fft_size.rs` is what
  stops the two drifting, and also pins that a mode with its own front
  end publishes 0 rather than a plausible-looking guess.

- **Size versioning, so the next field is not another `snr_db`.**
  `MfskModeInfo` and `MfskDecodeDefaults` lead with `size`: the caller
  sets its own `sizeof` (or zeroes the struct), and a newer library
  writes only the declared prefix and rewrites `size` to what it wrote.
  `MfskResult` grew `snr_db` in 0.8.1 with no marker at all. Exercised
  from both Rust and the C++ driver with a deliberately short struct and
  a 0xAA-filled tail.

  `mfsk_abi_version()` is separate from `mfsk_version()` for the same
  reason: the crate version moves for reasons that have nothing to do
  with the boundary.

- **The registry describes capability, not just geometry.**
  `ProtocolMeta` gains `profile: DecodeProfile` — a capability bitmask, a
  default search (band, `sync_min`, `max_cand`), the *scale* that
  `sync_min` is measured on, and the sniper candidate cap — plus
  `tx_start_offset_s` and `slot_samples_12k`, which a host needs in order
  to place a transmission or size a buffer and previously could not ask
  for.

  `PROTOCOLS` has always been able to say "this build has FST4-120". It
  could not say that FST4 has no SIC at all, that the sniper is FT8's
  alone, or that `.strictness()` is a no-op on FST4's non-AP path — so
  every consumer that needed to know hardcoded a matrix, and the C ABI
  that is about to publish one would have hardcoded it too.

  **The scale field exists because of a specific trap.** FT4's `sync_min`
  is divided by a fitted baseline, so noise sits at ~1.0 *by
  construction* and WSJT-X's own 1.2 (`ft4_decode.f90:195`) is a floor
  rather than a preference. FT8's and FST4's are absolute Costas scores.
  Three incomparable numbers currently sit in one C function with nothing
  saying so.

  `tests/registry_caps.rs` enforces the claims in both directions, and
  the type system does the work: `check_sic_rounds::<P>` is bounded on
  `SupportsSicRounds`, so naming a protocol that does not implement it is
  a compile error, while the body asserts the bit — implementing the
  trait and forgetting the bit fails at runtime. Verified by removing a
  bit and watching two tests fail.

- **`mfsk-ffi` has feature flags, and one of them drops rayon.** The crate
  had no `[features]` table at all and pinned `mfsk-core`'s `full`, so
  every consumer got std + rustfft + rayon + serde whether it wanted them
  or not, and `--no-default-features` changed nothing.

  rayon is the reason this matters. The first decode lazily spawns rayon's
  global pool — `num_cpus` threads, 2 MiB stacks each, never joined. On
  Android those threads are not attached to ART; on iOS they sit outside
  GCD's QoS, compete with the audio render thread, and keep running when
  the app is backgrounded. `--no-default-features --features mobile` is
  the same protocol coverage, single-threaded, without serde.

  Dropping `parallel` also *strengthens* the `.on_result` contract rather
  than weakening it: from "completion order, from a worker thread, with a
  possible transient duplicate" to "exactly once per returned result, in
  order" (`docs/reference/STREAMING.md` §3a vs §3b).

  `desktop` is the default and is byte-for-byte the previous build, down
  to an identical generated header. Individual protocol features are
  deliberately *not* offered yet — `src/lib.rs` imports every protocol
  module unconditionally, so `--features ft8` alone does not compile;
  splitting them belongs to the ABI rewrite that cfg's the entry points.
  Shipping a feature combination nobody can build would be worse than
  shipping one switch.

- **`DecodeRequest::budget` / `SniperRequest::budget` — FT8 decodes
  cheapest-first inside a caller's wall-clock allowance.** A decode can
  now be handed a caller-supplied deadline predicate
  (`BudgetCheck<'a> = &'a (dyn Fn() -> bool + Sync)`), and
  `DecodeOutcome` carries a `BudgetReport` saying what was left undone.
  Host and WASM callers have had no way to bound decode wall-clock at
  all: `max_cand`, `sync_min` and `.osd(bool)` are all chosen before the
  audio is seen, so a browser tab or a phone has to configure for its
  worst slot and pay that on every slot.

  **A closure rather than a `fn` pointer, and no clock in this crate.**
  `std::time::Instant::now` is unimplemented on
  `wasm32-unknown-unknown` and absent on `no_std`, so the caller
  supplies the clock — host `Instant`, browser `performance.now()`,
  embedded `esp_timer_get_time`. A `fn` pointer cannot capture that
  deadline, which is why `fst4::rung_major`'s existing `budget_ok:
  Option<fn() -> bool>` forced its embedded consumer to route the
  deadline through a per-core `UnsafeCell` global; `wspr::decode`'s
  `budget` parameter already uses the closure shape adopted here, and
  its `Sync` bound is what lets one captured deadline be shared across a
  `rayon` batch.

  **What "cheapest-first" buys, measured.** On `qso3_busy.wav` at the
  host research config, the budgeted single-pass decode returns the
  *strongest* n signals rather than the first n across the band: a
  budget of 1/2/4/8 candidates returns 1/2/4/8 real decodes, and 16
  reaches the full set of 14 that the unbudgeted decode finds. The
  scheduler gets that by sweeping the cheap sync triage across every
  candidate first — the gate that already rejects ~82 % of them before
  the 58-symbol DFT — then ordering the survivors by sync quality and
  spending the budget down that order. Phase one is never gated: it is
  what produces the ordering, and gating it would put the schedule back
  at the mercy of frequency order.

  **Deferring OSD to a second sweep was built and measured worse, so it
  is not here.** Running a cheap BP-only sweep across every survivor and
  offering OSD only to the failures is the obvious next move — OSD is
  ~30 % of an FT8 decode's wall-clock for ~30 % of its decodes. Same
  build, same WAV, budget in wall-clock ms through `bench/wasm`:
  interleaved returns 8/13/14 stations at 15/17/20 ms where the deferred
  schedule returns 7/11/11, and finishes the whole decode in 28 ms
  against 36 ms. The second sweep has to recompute each revisited
  candidate's LLR/BP ladder, and that costs more than the OSD it
  deferred — even with the 58 data-symbol DFTs retained across the two
  passes, which was the first thing tried. Issue #284 measured the same
  direction on the SIC engine for the same reason. The table is recorded
  at the scheduler so it is not re-attempted blind.

  **The triage sweep is a floor.** It is never gated — that is what
  makes the schedule independent of frequency order — so a budget
  shorter than the sweep returns nothing, having spent the sweep's time
  anyway. Measured through `bench/wasm` under Node on `qso3_busy.wav`:
  ~13 ms of a ~28 ms decode, so 5 ms and 10 ms budgets return zero
  stations in ~13 ms while a 20 ms budget returns all 14. `max_cand`
  remains the knob for the floor itself. `bench/wasm` gained
  `decode_wav_budget(audio, budget_ms)` and `bench.mjs` an
  `MFSK_BENCH_BUDGET_MS=5,10,20` sweep to reproduce that table.

  The SIC strategies (`.sic_rounds(n)`, `.sic_early()`) take the
  predicate too, but only poll at candidate and round boundaries, in
  their existing order. They cannot be reordered — each accepted decode
  is subtracted from the residual before the next candidate is looked
  at, so the order is the algorithm — and the poll deliberately sits
  *before* a candidate rather than between accepting a decode and
  subtracting it, since a cut in that window would leave an
  unsubtracted signal for the next round's `coarse_sync` to re-find as
  a duplicate. `tests/ft8_budget_scheduler.rs` asserts that directly.

  **FT4 and FST4 honour it too**, on every strategy, through the shared
  generic engine. Neither needed a new sweep: `ft4_coarse_sync` already
  returns candidates ranked by sync score, so FT4 declines the weakest
  by polling in the order it already had; and FST4's
  `dedup_refined_candidates` has already run `fst4_sync_search` over
  every candidate to suppress near-duplicates, so a *refined* score —
  sharper than the coarse one, and free — is in hand to rank by. FT4's
  `.sic_rounds(n)` declines whole rounds rather than candidates: that
  engine subtracts a round's decodes as one batch, so a round is the
  granularity it can honestly offer.

  FT8's own `decode_block` driver — what the ESP32 boards run — is
  untouched and keeps its app-level deadline.

  Because the API takes a closure and not a clock, the tests get a
  device-independent unit for free: a predicate backed by a counter
  cuts after exactly n candidates on every machine and thread count.
  A host is ~50× a CoreS3, so a wall-clock cap in a test would have
  asserted on the machine rather than on the scheduler.

  `DecodeOutcome` gaining a field is additive for anyone reading
  `.results` / `.fft_cache` (which is every known consumer, `mfsk-ffi`
  included) and source-breaking only for a downstream that constructs
  the struct itself.

- **`mfsk_decode_i16_sniper` / `mfsk_decode_f32_sniper` — single-target
  decode over the C ABI (#249).** Where `mfsk_decode_i16` searches a
  band, these aim at one `target_freq_hz`: the shape for a caller who
  already knows where the station is, from a sked, a spot, or the
  frequency being worked. `SniperRequest` derives a ±250 Hz window
  around the target and spends its candidate budget inside it.

  **The reason this matters beyond convenience is the AP hint.**
  `mfsk_decode_options_set_ap_hint` reaches the wide-band decoder for
  FT8 only — `SupportsWideBandAp` is not implemented for FT4 or FST4 —
  while `SniperRequest::ap_hint` is available to all three, since they
  share the 77-bit WSJT message. Until now, FT4 and FST4 a-priori
  hinting was **unreachable from C entirely**, and a hint that names a
  station actually on air is worth 1-3 dB.

  The existing `MfskDecodeOptions` handle is reused rather than a
  parallel sniper-specific one: `sync_min`, `max_cand`, `depth`,
  `strictness`, `eq_mode` and `ap_hint` apply; `freq_min_hz`/
  `freq_max_hz`, `freq_hint` and `sic_rounds`/`sic_early` do not (the
  target frequency is the hint, and a sniper request has no SIC
  strategy) and are ignored rather than rejected — the convention this
  crate already follows for `sic_early` on FT4. Protocols with no
  single-frequency mode return `MFSK_STATUS_UNKNOWN_PROTOCOL` instead
  of decoding something else.

  Covered by `mfsk-ffi/tests/sniper_ffi.rs` and a `test_sniper` case in
  the CI-run C++ driver, which decodes an FT4 signal at 1200 Hz through
  an AP hint from compiled C++ — the combination that was unreachable.

  **Superseded later in this same unreleased section** — see "The sniper
  is an FT8 mode" under *Removed*. The FT4/FST4 arms are gone and the
  AP hint they existed to carry now reaches those protocols through the
  ordinary wide-band `mfsk_decode_i16`, which is where it belonged.

### Removed

- **`mfsk-ffi-ft8` is retired.** Issue #251 decided not to invest
  further in it and wrote its own exit condition: "revisit only if …
  the crate becomes an actual maintenance drag on other refactors."
  This was that.

  The case for keeping it had already stopped holding.
  `embedded-poc/idf-component/README.md` states it itself — "pure-C code
  can't define `extern "Rust"` symbols" — so an ESP-IDF project must
  write a Rust staticlib shim to supply
  `mfsk_core_make_default_fft_planner()` regardless. Once a consumer is
  writing Rust, calling `mfsk_core::ft8::decode_block::*` directly is
  strictly simpler, which is exactly what all three of this repo's
  boards do. The C ABI never removed the work it existed to remove.

  Its residual value was catching embedded build breakage, and
  `scripts/pre-push-check.sh` already builds `alloc ft8 fft-extern` and
  `alloc ft8 fft-extern fixed-point` — the thing that actually matters,
  and it needs no C ABI.

  Removed with it: the crate and its header, the esp32/esp32s3 release
  tarballs (13 downloads across 30 releases), its CI job, and
  `ffi_smoke_one` in `embedded-shared/src/apps/compute_bench.rs` —
  ~40 lines whose timings the same bench already measured natively, so
  the C ABI there was adding a wrapper rather than a measurement. The
  four embedded app crates drop the dependency.

  **`embedded-poc/` is outside the workspace and CI never builds it**,
  so those edits are not compile-verified here. They are dependency
  removals and one deleted function with its only call site, but that is
  a statement about their shape, not a green build.

  This does not foreclose a `no_std` C ABI later — it would just be
  built on the redesigned surface rather than the legacy one, when a
  consumer exists. `mfsk-ffi-abi` stays split out for that reason: the
  types are plain data with no `std` requirement.

- **The pre-v2 decode surface is gone.** `mfsk_decoder_new`/`_free`, the
  `MfskDecodeOptions` handle and its eight setters, `mfsk_decode_i16`/
  `_f32`, both sniper entry points, `mfsk_decode_i16_streaming`,
  `MfskResultList`/`MfskResult`/`mfsk_result_list_free`, and the
  `MfskProtocol` enum. Everything they did is on the decode session or
  on a per-mode entry point, described in *Added* above.

  Deleted rather than deprecated because the two could not safely
  coexist: both handles crossed as the same opaque `MfskDecoder*`, so a
  pointer that wandered into the other's free function was undefined
  behaviour no compiler could notice.

  `MfskProtocol` was the shape of the problem this work exists to fix.
  One FST4 entry meant 15/30/120/300 were unreachable from C for decode
  and encode alike; `MfskMode` addresses all 25 modes.

- **WSPR, JT9 and JT65 have their own entry points instead of a generic
  dispatch that pretended they were the same shape.** `mfsk_wspr_decode`,
  `mfsk_jt9_decode_at`, `mfsk_jt65_decode_at`. JT9 and JT65 are point
  decodes at a known carrier rather than searches, and the pre-v2 path
  hardcoded 1500 Hz and 1270 Hz with no way to say otherwise — the
  frequency is an argument now. The Q65 family keeps its four functions
  and moves to the shared row type.

- **A session is single-threaded, where the old handle was not.** The
  pre-v2 handle carried one protocol tag, so sharing it across threads
  happened to work and the C++ driver tested that it did. A session
  caches a callsign hash table it mutates on every decode, so sharing
  one would be a data race. This module's own documentation already
  said that a change adding cached state must "tighten this documented
  contract back to strict one-per-thread"; this is that change. The
  driver now exercises one session per thread and concurrent mixed
  modes, which is the supported shape.

- **The sniper is an FT8 mode, and FT4/FST4 no longer offer one.**
  `SniperRequest` is now gated on its own trait, `SupportsSniper`,
  implemented for `Ft8` alone; `DecodeRequest::<Ft4>::sniper` and the
  FST4 equivalents no longer exist, and `mfsk_decode_*_sniper` returns
  `MFSK_STATUS_UNKNOWN_PROTOCOL` for them.

  The sniper's ±250 Hz window is the software half of narrowing a
  transceiver's *analogue* roofing filter — a handful of radios, pointed
  at a DX station whose carrier is already known. FT4 is a contest
  protocol whose premise is working a full band, so a roofing filter is
  against its purpose; FST4 narrows through its own DDC channelizer,
  which is the same benefit without a second decode engine. And the
  principle behind both: **the wide-band path is the main path for every
  mode here, so if it is not WSJT-X-faithful without a sniper, that is a
  bug in the wide-band path** — not a reason to ship a second one.

  What made the second engine look necessary was AP, and that coupling
  was an accident (see *Changed* below). With AP on the shared ladder
  there is nothing the FT4/FST4 sniper could do that the wide-band
  decode cannot, and it carried two latent defects of its own: it
  generated FT4's candidates with the generic 2-D Costas search rather
  than FT4's `getcandidates4.f90` port, so both the candidate set and
  the meaning of `sync_min` were wrong on that path.

  Deleted with it: `msg::pipeline_ap`'s entire engine — `decode_band_ap`,
  `decode_sniper_ap`, `process_candidate_ap`, `finalise_result` — leaving
  96 lines of hypothesis generation (`ap_passes`, `ap_bits_for`);
  `tests/ft4_sniper_aim_offset.rs`; and the FT4 sniper arms of
  `ft4_streaming_decode`, `ft4_snr_sweep` and `ft4_timing_budget`, whose
  AP cases moved to the wide-band path. `tests/registry_caps.rs` now
  pins `caps::SNIPER` to FT8 in both directions, with a
  `check_sniper::<P: SupportsSniper>` that will not compile for any
  other protocol.

### Fixed

- **An out-of-range enum argument from C was undefined behaviour, and it
  segfaulted.** `mfsk_mode_name((MfskMode)9999)` from the C++ driver
  crashed the process. A `#[repr(C)]` fieldless enum is an `int` to C,
  so a caller can pass a value from a config file, a newer header, or a
  plain mistake — and reading an out-of-range discriminant *as a Rust
  enum* lets the compiler assume it is one of the listed variants and
  optimise the match accordingly.

  Every entry point now takes the mode, the Q65 sub-mode and the fading
  model as `uint32_t` and validates it. C callers still write
  `MFSK_MODE_FT8`: an unscoped enum constant converts implicitly in both
  C and C++, so nothing changes on that side. The crash case is a
  permanent test.

  This is the third instance of one class of defect found in this
  redesign — the others being the `&'static mut` fabricated from a raw
  pointer, and a `memset`-to-zero options struct producing an invalid
  `MfskDecodeDepth`. All three are "C hands Rust a value Rust's type
  system says cannot exist", and all three are now checked at the
  boundary rather than assumed away.

- **The FFI crates' rustdoc was never checked, and a broken intra-doc
  link had already shipped through the gap.** Both the pre-commit hook
  and CI's `docs` job ran `cargo doc` for `mfsk-core` only, so
  `mfsk_decode_i16_sniper`'s doc comment could reference a private item
  and reach `main` unremarked. Both now cover `mfsk-ffi`,
  `mfsk-ffi-abi` and `mfsk-ffi-ft8`.

  This matters twice over in these crates specifically: their doc
  comments are also the text cbindgen copies into the committed C
  headers, so a bad link is wrong in the rustdoc *and* in `mfsk.h`.
  Found by running the gate that did not exist yet.

- **The FFI crates' hand-written version pins can no longer go stale.**
  `mfsk-ffi` and `mfsk-ffi-ft8` each required `mfsk-core` and
  `mfsk-ffi-abi` by `version` as well as by path, and those literals
  tracked nothing: `[workspace.package] version` is the single source of
  truth, and a caret requirement keeps matching across every patch bump.
  The mismatch surfaces only on a **minor** bump, as a resolve failure
  on release day — which is exactly when it happened cutting 0.10.0,
  with pins that had sat at `0.9` since 0.9.0 and survived 0.9.1
  untouched.

  All three FFI crates are `publish = false`, so cargo never needed the
  requirement. Removed, with the reasoning recorded where the pins were.
  Simulating a `0.11.0` bump now resolves cleanly where it previously
  failed. If one of these crates is ever published, cargo demands a
  version on each path dep and says so at that moment, rather than
  leaving a literal nobody re-reads.

### Changed

- **FST4's npre-vs-timing ablation re-measured at n=100, and two of its
  conclusions did not survive (#311).** The grid that decides whether
  the embedded escalation ladder (#310) needs both the `i0±1` timing
  retry and an unpruned-OSD fallback was measured on 20 trials per
  cell, where a rescue of 3-8 trials per 100 is frequently invisible —
  two cells reported "neither mechanism rescues anything alone", which
  was sample size rather than mechanism.

  At n=100: both mechanisms rescue on their own in every cell, 3 of 4
  cells are synergistic rather than 2, timing is the stronger single
  rung everywhere (+8/+4/+6/+13 trials against the fallback's
  +7/+3/+4/+5), and neither rescue set contains the other in any cell —
  so the escalation order the small sample suggested does not exist.
  `npre→unpruned` turns out to be exactly `unpruned` in all four cells
  at both timing settings, and `npre` never rescues a trial `unpruned`
  misses: on this corpus the pruned search is a strict recall subset,
  making npre1/npre2 a speed trade rather than the recall-neutral
  change it was recorded as. Tables in `docs/notes/FST4_BENCHMARK.md`.

  The corpora were extended 20 → 100 trials in place, which the same
  document said was impossible: it held that regenerating renames the
  signals behind existing trial indices. Measured otherwise — `fst4sim`
  is deterministic and its realisations do not depend on the requested
  trial count (four cells, 80 of 80 files byte-identical after
  regenerating at `TRIALS=100`). Corrected there and in
  `scripts/gen_fst4_sweep_wavs.sh`'s header.

### Changed

- **The crate declares a minimum supported Rust version, and the one
  the embedded crates already declared was wrong.** `mfsk-core` and
  the rest of the host workspace had no `rust-version` at all — a
  published crate with no floor, so cargo could not tell a downstream
  user why a build failed. The six `embedded-poc` manifests did have
  one, `1.82`, which had been unbuildable since `mfsk-core` moved to
  edition 2024 (that alone needs 1.85).

  The declared value is `1.93`, measured rather than inferred:
  `cargo +1.85 check -p mfsk-core --features full` fails on the
  let-chains in `engine/pipeline.rs` and `fec/ldpc/osd.rs` (stabilised
  for edition 2024 in 1.88), and `+1.88` then fails on
  `wspr::instrument`'s `const ALL: &[&AtomicU32]` with E0080 —
  `const` items could not hold references to mutable statics until
  1.93 ("Allow `const` items that contain mutable references to
  `static`", 1.93.0, 2026-01-22). `cargo +1.93 check --workspace
  --all-targets --features full,internal-testing` is clean.

  `[workspace.package] rust-version` is the single source, the way
  `version` already is, and `ci.yml` gains an `msrv` job pinned to
  1.93 (`cargo check --workspace --all-targets --features
  full,internal-testing`) so the number is falsifiable instead of
  resting on whatever `@stable` happens to be. Raising it stays a
  decision someone makes on purpose rather than something that drifts
  in with a new language feature.

### Changed

- **FST4 and WSPR ran the same display task twice (#353 item 2).** The
  two receivers' `display_loop`s were 330 lines each and differed in
  three places: the `BootMode` the picker highlights, which `ui::`
  render functions get called, and the log prefix. Everything around
  those — the AXP2101/AW9523B bring-up, the VBUS check that keeps the
  port flashable when the board is plugged into a PC, the SPI/mipidsi
  init with its 2026-08-15 orientation history, the boot summary, the
  RTC store, the dirty-seq redraw gating and the touch polling — was
  the same code written out twice, so a fix to one had to be
  remembered in the other. Each also carried its own `DisplayCtx`,
  task entry, spawn helper and `current_hhmmss`, and the ~25-line
  touch-poll block appeared four times across the two files.

  Now `crate::spot_panel`: a `SpotPanel` trait carrying exactly what
  differs (mode, tag, task name/stack/priority, the four render calls
  and the state lock) and one `run<P>` holding the loop — the same
  shape `crate::net` uses for network bring-up. `apps/fst4.rs` goes
  1 237 → 902 lines and `apps/wspr.rs` 1 798 → 1 413, against 543
  shared, and the touch block exists once.

  FT8 and FT4 stay on `ui::state::UI` + `display::run_log_panel`,
  which is a waterfall + decode ring + TX panel rather than a spot
  list; #353 is explicit that moving them onto this would be the wrong
  direction.

  Behaviour is meant to be identical, and the parts a compiler cannot
  check — that both screens still paint, and that the mode picker
  still gets a board out of a receiver — **have not been checked on
  hardware**. The issue asks for a flash and a look at each.

### Fixed

- **The CoreS3 WSPR receiver captured on its own grid, not UTC's, and
  stamped its spots from the wrong clock read (#313 item 1).** Three
  things, in one path, none of which shows up as a bad log line —
  the decoder still decodes, and the spots are simply filed against
  the wrong two minutes.

  `WsprDdcSink` opened its capture window wherever the USB stream
  happened to come up. A WSPR transmission starts 1 s into an even
  minute and runs 110.6 s, so an arbitrary phase cuts it. It then ran
  114 s captures back to back with no gap — a 114 s cadence against a
  120 s grid, sliding 6 s per slot even from an aligned start. (The
  synthetic producer never had that one: `ddc_loop` sleeps out the
  remainder of `SLOT_US`.) And the spot's timestamp was read from the
  clock at decode time, which is ~200 s after the window it describes
  opened — one or two boundaries later, on every spot.

  The window now opens on a UTC boundary and re-measures the gap from
  UTC at every boundary, so an NTP step or the RTC's drift is absorbed
  by one gap instead of accumulating; each slot carries its own start
  time to the reporting side. With no plausible clock it waits the
  fixed 6 s tail instead — cadence right, phase arbitrary — and the
  log says which of the two is in force.

  The batch splitting and gap arithmetic are
  `mfsk_app_shared::capture_window::CaptureWindow`, compiled and tested
  by `hosttest/mfsk-app-shared` rather than trusted: the app crate
  builds only for Xtensa, and this is the third slot grid in it. The
  other two — FT8's `Ft8ChunkSink` in `uac.rs`, FT4's
  `ft4_rx::SlotAccum` (#354, shipped in 0.10.1) — keep their own, and
  neither is changed here. (This entry said both capture a contiguous
  grid and so had no gap to manage. That is true of FT8 only; see the
  correction in this section's own FT4 entry above.) `civil_time
  ::slot_start_unix` rounds a clock read to the nearest slot boundary
  rather than flooring, since the read that opens a window lands a few
  milliseconds either side of the boundary it aimed at and flooring
  turns "1 ms early" into the previous slot.

  **Not verified against a radio.** `wspr_app` has never been run
  against one (#313 item 3); this is a host build plus tests, and the
  numbers that would confirm it are a log capture away.

### Changed

- **"M5StickS3 can't do USB host" was a claim about the silicon; the
  measurement was about the board (issue #360).** A reader searching
  for "M5StickS3 USB host vbus" found the sentence here and pointed out
  that ESP32-S3 can be a USB host in general — they run a USB keyboard
  off a StickS3 with VBUS supplied externally, `M5.Power
  .setUsbOutput(true)` returning without doing anything on that board.

  They are right, and the repository already knew it in the one place
  where the finding was written up in full: `ROADMAP.md`'s 2026-05-17
  pivot says "silicon supports it, board doesn't wire for it". What
  every shorter restatement of it dropped was that second clause —
  `README.md`, the root `CLAUDE.md`, `embedded-poc/CLAUDE.md`,
  `m5stack-s3-app/CLAUDE.md`, `m5stack-cores3-app/CLAUDE.md`,
  `EMBEDDED.md` / `.ja.md` and both StickS3 operator manuals each said
  some form of "cannot do USB host" flat. They now say what was
  actually verified: the board cannot **source VBUS**, so a bus-powered
  device attached to it gets no power.

  Nothing about the 2026-05-17 pivot changes — a controller carried to
  a hilltop with an IC-705 cannot bring a second 5 V supply, so the UAC
  line stays on CoreS3. `EMBEDDED.md`'s StickS3 row also still called
  the crate the "production controller", which the pivot demoted to
  demo / acoustic-fallback sixteen weeks ago and the `.ja.md` twin had
  already corrected.

- **A 13-day release gap is the cadence, not an overrun.**
  `release-status.sh` warned "past the 13-day maximum" at exactly
  `days >= 13` and `CLAUDE.md`'s "(max wait 13 days, average 7)" backed
  that reading — but the policy is "every 2 weeks", and the "average 7"
  came from averaging two escape-hatch same-week patches
  (v0.8.0→v0.8.1, v0.9.0→v0.9.1) in with the regular cuts. The regular
  cuts alone run 12, 8, 13, 13 days. The cadence check now has a
  separate "at the 2-week target" tier for days 13-14 and calls a
  release overdue only past 14.

Older releases are archived out of this file to keep it skimmable. The rule:
this file holds the unreleased section and the two latest minor series, and
each new minor's release PR moves the oldest series out.
0.8.0 – 0.10.1 in
[`docs/historical/CHANGELOG-0.8-0.10.md`](docs/historical/CHANGELOG-0.8-0.10.md),
0.6.0 – 0.7.4 in
[`docs/historical/CHANGELOG-0.6-0.7.md`](docs/historical/CHANGELOG-0.6-0.7.md),
0.1.0 – 0.5.12 in
[`docs/historical/CHANGELOG-0.x.md`](docs/historical/CHANGELOG-0.x.md).
