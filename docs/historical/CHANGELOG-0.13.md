# Changelog — 0.13.x history (0.13.0, 0.13.1)

0.13.0 and 0.13.1, split out from the top-level `CHANGELOG.md` when 0.15.0 was prepared. `CHANGELOG.md`
keeps the unreleased section and the two latest minor series; the release PR of each new minor
moves the oldest series here (`CLAUDE.md`, "Releases"). Full detail preserved verbatim, just
relocated. See [`../../CHANGELOG.md`](../../CHANGELOG.md) for 0.14.0 onward,
[`CHANGELOG-0.12.md`](CHANGELOG-0.12.md) for what came before.

## 0.13.1 — the LPF subtract no longer panics on a slot shorter than the frame (#567, #574), 48 kHz input is low-passed before 12 kHz (#576), hashed calls keep /P and /R (#570), the licence is GPL-3.0-only (#569), the skimmer installers say how to install them

- **48 kHz (and any rate above 12 kHz) is low-passed before going down to 12 kHz, as WSJT-X does (#576).**
  `engine::dsp::resample` had no anti-alias filter: 48 kHz → 12 kHz was a plain decimation by 4, so 6–24 kHz folded into
  the band. It runs in the C ABI's decode and stream entry points at any rate other than 12 kHz, on the boards' USB audio
  and microphone, and in wasm capture. On white noise over the full 48 kHz band FT8's 50 % crossing was -15.3 dB; with
  `fil4` (`lib/fil4.f90`, the 49-tap FIR `Detector.cpp` applies) it is -21.1 dB, against -21.0 dB for the same signal and
  noise generated at 12 kHz. `fil4` is now the filter at 48 kHz, in the batch functions and in `LinearResamplerI16To12k`
  (Q15, FPU-free); other rates get a Kaiser low-pass to its specification. Tests: a 9 kHz tone comes out ≥ 30 dB down
  (it was 0 dB), and FT8 at -19 dB in full-band 48 kHz noise decodes (0 of 4 before).

- **docs: the 0.13 decoder by use case, why it is shaped like WSJT-X's, and how to move to it.** `LIBRARY.md` §1.1 (and
  `.ja.md`) is new: a decoder is something you keep, shown on the common jobs (a live receiver numbering its periods, a
  QSO's context, one decoder per channel, a deadline, streaming rows, a non-GUI search, other sample rates), each with
  the reason for its shape and each a doctest. `LIBRARY.md` §1.2 (and
  `.ja.md`) gathers the migration from 0.12 that sat at the end of §6, adds what changes in results with the
  defaults (FT8 search, band, AP, JT9/JT65, WSPR, with the measurements behind them) and what replaces each removed option;
  §2 opens with the six rules the API follows. `DESIGN_RATIONALE.md` §6 records what went wrong in 0.12, the
  decision, what it cost and what it bought. `ft4::subtract_signal_lpf`'s doc no longer claims a short buffer is
  a no-op (#574).

- **`engine::dsp::subtract`: the LPF subtract no longer panics on a slot buffer shorter than the frame (#567).** `apply_at_offset` sized
  the FFT from `audio.len()`, where `subtractft8.f90` fixes `NFFT = NMAX = 15*12000` (v3.2.0-rc1, lines 10-11) — always at least
  `NFRAME = 1920*79`, so a short `dd` is simply zero-filled. On a shorter buffer `cfilt` was shorter than `nframe`, and both the
  camp build (`cfilt[i]`) and the end correction (`cfilt[nframe-1-d]`) indexed past its end. With end correction on — the FT8
  subtract's own setting — that panicked at *any* `dt`, not only a negative one, and on the `sic_early`/`__staged_sic` paths it
  takes the decode worker thread down with it. `nfft` is now `audio.len().max(nframe)`, which restores the reference's invariant;
  the existing clamps and the end correction are then correct as written. A full slot is unaffected (`audio.len() > nframe`
  already, so `nfft` does not move) and its output stays bit-identical. `residual_band_power` takes the same `nfft` so the
  `sqf` trial search scores on the grid the subtract actually used. New tests: `subtract_short_buffer`, and
  `decoder_input_length`, which runs every mode through `AnyDecoder` on a real frame cut to 0 … 1.5 periods from either end
  (on noise alone no decode happens and nothing after it runs, so it did not catch #567 until it carried a signal).
  **Affected:** every release since the FFT subtract (`28a38e7a`, 2026-07-20): 0.8.0 through 0.13.0. On an
  older version, pad the slot with zeros to its full length (FT8 180 000 samples, FT4 90 000) before decoding: the FFT is then
  at least a frame long and the panic cannot occur. Older versions are not yanked; 0.13.1 is the fixed one.

- **A hashed callsign with `/P` or `/R` now resolves, as in WSJT-X (#570).** `CallsignHashTable::insert` stripped the suffix and the
  type 1/2 learner never read the `/R` / `/P` flag, so `<W1XYZ/P>` stayed `<...>` where WSJT-X v3.2.0-rc1 prints it (32 of 336 outcomes on the
  WS fixtures of #568; no wrong call, a missing resolution). Both now follow `save_hash_call` and the unpacker (`packjt77.f90:356-390`,
  `:1755-1765`). New test `ws_77bit_extension` checks 168 messages in two table states against rc1: 336 of 336.

- **License identifier corrected to `GPL-3.0-only` (#569).** Up to 0.13.0 the crate, its README and the source headers said
  `GPL-3.0-or-later, matching upstream`. WSJT-X states "version 3" (README, About dialog, `COPYING`) and does not add "or later";
  this crate contains code derived from it. `license` in the `Cargo.toml` files, the `SPDX-License-Identifier` headers
  (Rust, Swift, Kotlin, C, C++), the READMEs, `CONTRIBUTING.md` and the reference docs now say GPL-3.0-only. Already published
  versions keep their crates.io metadata; this takes effect with the next release. Whether files that are original work
  (not derived from WSJT-X) should return to "or later" is left for a per-file decision.

- **skimmer: the install guide and the release page say how to install it.** `apps/skimmer/README.md` now lists what the
  app needs (a reachable SpyServer, a right clock), the Windows SmartScreen step, macOS 14 versus 15+ (*Open Anyway*,
  `xattr`), first run, what to check when it will not connect, and updating. The GitHub release body carries the file
  table with the release's version, the same install steps and a link to the README; `attach-skimmer` needs `verify-tag`
  for that version. Not run on a tag yet.

## 0.13.0 — one persistent `Decoder<P>` per mode on the WSJT-X parameter block (breaking), `Depth` as `ndepth`, QSO-context AP for FT8/FT4/FST4, a pull `IqReceiver` with a clock that follows the host, one C decoder handle for every mode (breaking), per-channel decoder threads in the skimmer

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
  running skimmer at the channel's next slot), and a `STATUS.log` of push, decode, queue and drop figures for long runs (rolled to `STATUS.log.1` past 5 MB; a server failing the same way repeatedly is one line and a count).

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
- **FT8's T1 sensitivity task now asks for what `jt9 -8 -d 3` does.** With no callsign known the AP tries only the CQ
  hypothesis; the 0.12 sweep asked for it with `ApHint::with_call1("CQ")`, which a `Decoder` still accepts and which
  reproduces the old numbers bit for bit. The CQ-only derivation is 0.2-0.5 dB behind that in fading channels
  (ccir_moderate -20.00 against -20.50 dB) and level with WSJT-X in paired counts (5 up-only against 3 crate-only, p =
  0.73); JT65, JT9 and Q65 crossings are unchanged and WSPR's moved -0.07 dB. Against WSJT-X no group is behind except
  JT65's known gap (15 upstream-only against 0, as in 0.12). AP is off by default.
- **The skimmer keeps every decode in SQLite and answers questions about them.** `skimmer.db` (WAL, indexed on time,
  band and call; the GUI no longer writes an ALL.TXT, the CLI's `--log` still does) records each decode with its sender, locator, CQ kind and the server that
  heard it. An Analysis view runs one query (UTC period, regexes on call, locator and text, band, mode, server, SNR,
  distance, bearing sector, CQ kind) over a great-circle or Mercator map that animates in a chosen window, the
  openings of the bands, a per-station hours-on-the-air map with a small map of the great circle, and a Database
  tab (size, clearing out old decodes, compacting, a CSV of the query, a copy of the file).
- **The skimmer runs up to four SpyServers, each in its own place, and can rotate bands.** A server has its own
  locator, network delay, gain and control; bearings are read from the server that heard the signal. A rotation gives
  each band of a server's channels a turn (at least five minutes, from the first band on Connect or on the UTC
  clock) and each band can be given the UTC hours it takes part in (day and night follow the server's local time);
  a band's decoder and waterfall history wait for its next turn. The rotation can be held on a band, or moved
  to the next or previous band at once (its whole turn starts then). Sixteen busy-band FT8 decoders ending together take
  about 3 s on 8 threads (`tests/load.rs`).
- **The skimmer's screen.** A server's dot switches it on or off; beside its channels the rotation shows the band and the
  time left, holds when pressed, and moves to the previous or next band at once (its whole turn starts then). Settings are
  in tabs. The map has a grey line (day, the band where the Sun is within 6 degrees of the horizon, night; sunrise and
  sunset per server are in its Settings), band-coloured paths when bands are chosen, and an Animate that can skip
  windows with no station. The installers can be built by hand (`workflow_dispatch`).
  A band's hours can be "day" or "night" following the calendar: from an hour before sunrise to an hour after sunset
  (or the night between), recomputed for each day at the server, with no moment of renewal.
- **The skimmer's clock can follow NTP, and the delay to the server can be entered.** The PC clock is not touched;
  its measured offset (best of five SNTP queries, every ten minutes) is added, and a fixed network delay is taken off
  the arrival times, which the minimum-delay anchor cannot remove.
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
