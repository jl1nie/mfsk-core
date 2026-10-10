# mfsk-core — C, Kotlin and Swift bindings

> **日本語版:** [BINDINGS.ja.md](BINDINGS.ja.md)

This document covers consuming mfsk-core from outside Rust. For the
Rust host API see [`LIBRARY.md`](LIBRARY.md); for `no_std` / embedded
targets see [`EMBEDDED.md`](EMBEDDED.md).

| binding | lives in | built and tested by |
|---|---|---|
| **C / C++** | `mfsk-ffi/`, header `mfsk-ffi/include/mfsk.h` | CI `ffi` job — Rust tests under both feature sets plus `examples/cpp_smoke/`, a real C++ driver including a multi-thread stress |
| **Kotlin / Android** | `bindings/kotlin/` (C shim + `Mfsk.kt`) | CI `kotlin` job, on a desktop JVM |
| **Swift / Apple** | `bindings/swift/` (SwiftPM package `MfskCore`) | CI `swift` job on `macos-latest` — 98 XCTest cases plus the `aarch64-apple-ios` cross-build; **not built since the single-decoder rewrite** |

All three sit on the same C ABI (`mfsk_abi_version()` is 3). `mfsk.h` is
cbindgen-generated and committed, and its doc comments are the
authoritative per-symbol reference — this document is the map, not a
replacement for it.

## Contents

- [1. Artefacts and linking](#1-artefacts-and-linking)
- [2. The C ABI](#2-the-c-abi)
  - [2.1 Shape: decoders, and memory you own](#21-shape-decoders-and-memory-you-own)
  - [2.2 Decoding a period](#22-decoding-a-period)
  - [2.3 `MfskParams` — the parameter block](#23-mfskparams--the-parameter-block)
    - [2.3.1 `MfskExtras` — the library's options](#231-mfskextras--the-librarys-options)
  - [2.4 `MfskDecode` — one result row](#24-mfskdecode--one-result-row)
  - [2.5 Streaming capture](#25-streaming-capture)
  - [2.6 Transmit](#26-transmit)
  - [2.7 Introspection](#27-introspection)
  - [2.8 Q65: the lists and the sub-mode numbering](#28-q65-the-lists-and-the-sub-mode-numbering)
    - [2.8.1 JTTY — a receiver handle instead of a slot call](#281-jtty--a-receiver-handle-instead-of-a-slot-call)
    - [2.8.2 Wideband IQ — a receiver handle for an SDR stream](#282-wideband-iq--a-receiver-handle-for-an-sdr-stream)
  - [2.9 Messages](#29-messages)
  - [2.10 Threads and the runtime](#210-threads-and-the-runtime)
  - [2.11 Errors and memory rules](#211-errors-and-memory-rules)
  - [2.12 Symbol index](#212-symbol-index)
- [3. Porting](#3-porting)
  - [3.1 From the 0.13 ABI](#31-from-the-013-abi)
  - [3.2 From the 0.12 ABI](#32-from-the-012-abi)
- [4. Kotlin / Android](#4-kotlin--android)
- [5. Swift / Apple](#5-swift--apple)

---

## 1. Artefacts and linking

`cargo build -p mfsk-ffi --release` emits:

* `target/release/libmfsk.so` (Linux / Android shared object)
* `target/release/libmfsk.a` (static, for bundling)
* `mfsk-ffi/include/mfsk.h` (cbindgen-generated, committed)

Every tagged release also attaches a prebuilt `linux-x86_64` tarball to
the GitHub Release — see `mfsk-ffi/README.md`. Other platforms need a
local build; CI cross-compiles Windows-GNU and Android arm64 on every
source change, so they are build-verified even though no binary is
published for them.

**`MFSK_API`** is emitted on every declaration. Define `MFSK_STATIC`
when linking `libmfsk.a`, `MFSK_BUILDING` when building the DLL itself,
and nothing when consuming the DLL. Without it a Windows DLL exports
nothing linkable, and a Unix shared object exports every non-static
symbol including Rust internals.

The calling convention is `extern "C"`'s own, which is `__cdecl` on
Windows for every signature here. It is documented rather than emitted:
cbindgen can place a prefix before the return type but not in the
`__cdecl` position between return type and name.

| platform | link line |
|---|---|
| Linux | `-lmfsk -lpthread -ldl -lm` |
| Android | `-lmfsk -llog -lm` (no `-ldl`/`-lpthread`; both are in Bionic's libc) |
| macOS | `-lmfsk -lpthread -lm` |
| Windows (MSVC) | `mfsk.dll.lib` plus `ws2_32.lib userenv.lib ntdll.lib bcrypt.lib` |
| Windows (GNU) | `-lmfsk -lws2_32 -luserenv -lntdll -lbcrypt` |

**Android needs 16 KB page alignment.** Android 15 ships devices with a
16 KB kernel page size, and a `.so` linked for 4 KB does not load there
— surfacing as `UnsatisfiedLinkError` on exactly the newest hardware.
`.cargo/config.toml` sets `-C link-arg=-Wl,-z,max-page-size=16384` for
the three Android triples, **and CI asserts the resulting `.so` carries
it**, because setting `RUSTFLAGS` in the environment *overrides*
`target.*.rustflags` rather than merging with it — so the flag is one
workflow edit away from being silently dropped.

---

## 2. The C ABI

### 2.1 Shape: decoders, and memory you own

Two rules cover most of the surface:

1. **A decoder is the decode handle, and it is WSJT-X's.** `jt9 -s` runs
   one persistent decoder per mode and drives it once per period with a
   parameter block; `mfsk_decoder_open` → decode one period per call →
   `mfsk_decoder_close` is that model. The handle owns what upstream
   keeps across periods and nothing else: the callsign hash table (never
   shared with another decoder, as upstream's is not), FT8's a7 list,
   Q65's and JT65's averages, WSPR's call table. `mfsk_decoder_clear` is
   WSJT-X's "Clear Avg". One handle serves all 20 slot modes — FT8, FT4,
   the five FST4 periods, WSPR, JT9, JT65 and the ten Q65 sub-modes; the
   per-family decode functions of the 0.12 ABI are gone. MSK144, JTTY and
   uvpacket have no decoder (`MFSK_STATUS_UNKNOWN_PROTOCOL` at open; JTTY
   has its own receiver, §2.8.1).
2. **Nothing crosses the boundary as an allocation.** Result rows,
   synthesised audio and unpacked text all go into buffers you sized
   and own. There is no pointer to free, which removes the category
   that makes wrappers leak when an exception unwinds between a call
   and its free.

The handles are `MfskDecoder*`, `MfskStream*`, `MfskJttyReceiver*`,
`MfskIqReceiver*`, `MfskQ65History*` and `MfskQ65Callers*`, each with
its own `_open`/`_new` and `_close`/`_free`. They are distinct
incomplete types, so passing one where another is expected is a C type
error rather than undefined behaviour.

### 2.2 Decoding a period

The minimal path — this is the shape `mfsk-ffi/examples/cpp_smoke/main.cpp`
runs in CI:

```c
#include "mfsk.h"

MfskParams p;
memset(&p, 0, sizeof p);
p.size = sizeof p;
mfsk_params_init(MFSK_MODE_FT8, &p);        /* the mode's defaults; zeroing is not the same */
p.band_hi_hz = 2600.0f;

MfskStatus st = MFSK_STATUS_INTERNAL;
MfskDecoder *d = mfsk_decoder_open(MFSK_MODE_FT8, &p, NULL, &st);   /* NULL extras: the depth's own */
if (d == NULL) { /* mfsk_last_error() says why */ }

MfskDecode rows[16];
memset(rows, 0, sizeof rows);
for (int i = 0; i < 16; ++i) rows[i].size = sizeof rows[i];
size_t n = 0;
if (mfsk_decoder_decode_i16(d, pcm, n_pcm, 12000, MFSK_PERIOD_NONE,
                            rows, 16, &n) == MFSK_STATUS_OK) {
    for (size_t i = 0; i < n; ++i) {
        printf("%.1f Hz  %.0f dB  %s\n",
               rows[i].freq_hz, rows[i].snr_db, rows[i].text);
    }
} else {
    fprintf(stderr, "%s\n", mfsk_decoder_last_error(d));
}
mfsk_decoder_close(d);
```

`NULL` for `params` or `extras` at open means the mode's own defaults.
`out_cap` bounds how many rows you are willing to receive and
`*out_len` always receives how many were found, so a short array returns
`MFSK_STATUS_INVALID_ARG` with the required count rather than a truncated
answer you cannot detect. The retry is the same call again with room
(for `mfsk_decoder_decode_stream`, the same stream): it is answered from the
rows already found, without decoding again. `sample_rate` other than 12 000 is
resampled.
`decode_f32` takes audio at any level: the engines that work in `float`
(WSPR, JT9, JT65, Q65) see it as is, and FT8, FT4 and FST4, which take
16-bit audio as WSJT-X does, get it scaled to a fixed level.

```c
MfskDecoder *mfsk_decoder_open(uint32_t mode, const MfskParams *params,
                               const MfskExtras *extras, MfskStatus *out_status);
void         mfsk_decoder_close(MfskDecoder *dec);

MfskStatus mfsk_decoder_decode_i16(MfskDecoder *dec, const int16_t *samples,
                                   size_t n_samples, uint32_t sample_rate, int64_t period,
                                   MfskDecode *out, size_t out_cap, size_t *out_len);
MfskStatus mfsk_decoder_decode_f32(MfskDecoder *dec, const float *samples, ...);
```

**`period`** is the period's index on the UTC grid (`utc_seconds / T`), or
`MFSK_PERIOD_NONE` (`INT64_MIN`) for a lone recording. The parts of the
state that need consecutive periods — FT8's a7, JT65's and Q65's averaging
— are used only when the index is given, and Q65's averaging also restarts
on a gap in it.

What else the handle does:

| call | effect |
|---|---|
| `mfsk_decoder_set_params(d, &p)` | change the parameter block between periods, as the GUI rewrites it before each one; the state is kept |
| `mfsk_decoder_set_extras(d, &e)` | replace the library's options. What the block leaves unset goes back to the depth's value; an option the mode lacks is `MFSK_STATUS_UNSUPPORTED` and nothing changes |
| `mfsk_decoder_set_q65_callers(d, callers)` | Q65 only: the contest callers (§2.8), copied; with `MFSK_CONTEST_GRID_EXCHANGE` they join the full-AP list. NULL removes them. Survives `set_extras` |
| `mfsk_decoder_set_on_decode(d, cb, user)` | deliver each row **as it is found**, on top of the array the call returns; NULL stops |
| `mfsk_decoder_set_budget(d, check, user)` | poll a caller-supplied predicate once per candidate; stop when it returns `false`. Every mode with a decoder publishes `MFSK_CAP_BUDGET` and takes one; a mode without the bit would get `MFSK_STATUS_UNSUPPORTED` |
| `mfsk_decoder_last_budget(d, &report)` | what the cut left undone — candidates skipped, stages run, how good the best skipped candidate was, and `rows_subtracted` (FT8 `SIC_EARLY`'s checkpoint-B and -C subtractions, appended to the size-versioned struct); zeroed when no budget was set. WSPR, JT9, JT65 and Q65 set `exhausted` only, so their counts stay 0 |
| `mfsk_decoder_decode_prefix_i16` / `_f32(d, pcm, n, rate, period, out, cap, &len)` | the early decode (#572): call as audio arrives, with **every sample of the period so far** and `period` set. FT8 returns checkpoint A's rows at 141 696 samples (~11.8 s, `stage == MFSK_STAGE_EARLY`), none at 162 432, and at the whole period (180 000) the complete set, the rows `decode_i16` gives for the same audio. Other calls, and every call of a mode with no early decode until the whole period, return none. The callback sees each row once across the period; `delivery` counts across its calls. A retry after a short-buffer refusal gets the held rows. `MFSK_PERIOD_NONE` makes it a plain decode |
| `mfsk_decoder_delivery_is_exact(d)` | whether the callback sees exactly the rows the call returns, once each and in order, under the current mode, depth and extras (`STREAMING.md` §3a). `false` for FT8's single pass and sniper, FT4 at `MFSK_DEPTH_FAST`, FST4 and WSPR: pair by `MfskDecode::delivery` there. Ask again after `set_params` / `set_extras` |
| `mfsk_decoder_add_callsign(d, "JL1NIE")` | seed the hash table so a later `<...>` resolves. `MFSK_STATUS_UNSUPPORTED` for a mode whose messages carry no hashed calls |
| `mfsk_decoder_copy_info(d, i, out, cap, &len)` | the FEC information bits behind row `i` of the last decode (`MfskDecode::info_bits` of them) |
| `mfsk_decoder_unpack77(d, msg, out, cap, &len)` | `mfsk_unpack77` with `<...>` resolved against this decoder's table |
| `mfsk_decoder_clear(d)` | forget everything carried between periods (WSJT-X's "Clear Avg", `ndepth & 128`) |
| `mfsk_decoder_last_error(d)` | this handle's error slot; survives a thread hop, unlike the global |

The budget predicate is polled per candidate and **the library reads no
clock of its own** — the deadline is whatever your predicate compares
against. That is what keeps it usable from wasm and from a phone that
was backgrounded mid-slot. The callback and the predicate may both be
called from rayon workers on a `desktop` build, possibly concurrently, in
completion order; a `mobile` build calls them on the calling thread in
candidate order. The returned array is authoritative either way.

### 2.3 `MfskParams` — the parameter block

`MfskParams` is WSJT-X's `params` common block (`lib/jt9com.f90`): what
the GUI fills before each period and the decoder reads. A mode reads what
its upstream decoder reads and ignores the rest, as `jt9` does. Let the
library write the mode's defaults before you override anything:

```c
MfskParams p;
memset(&p, 0, sizeof p);
p.size = sizeof p;
mfsk_params_init(MFSK_MODE_FT8, &p);
p.depth = MFSK_DEPTH_NORMAL;
p.rx_freq_hz = 1500.0f;
```

| field | meaning |
|---|---|
| `size` | `sizeof(MfskParams)` as the caller understands it |
| `depth` | `ndepth & 7`: `MFSK_DEPTH_FAST` 1, `_NORMAL` 2, `_DEEP` 3. 0 is Deep, the GUI's default. Decides every search setting the way `ndepth` does |
| `flags` | `MFSK_PARAM_AVERAGING` (bit 0, `ndepth & 16`: JT65, Q65), `MFSK_PARAM_DEEP_SEARCH` (bit 1, `ndepth & 32`: JT65), `MFSK_PARAM_EME_DELAY` (bit 2, `emedelay`) |
| `ap_mode` | `MFSK_AP_OFF` 0 (`lft8apon` off), `_CQ_ONLY` 1 (`lapcqonly`), `_FULL` 2 (every hypothesis the QSO context allows). `_init` writes the mode's own default: off for FT8 and JT65 (the GUI's "Enable AP" boxes), full for FT4 |
| `contest` | `ncontest`: `MFSK_CONTEST_NONE` 0, `_GRID_EXCHANGE` 1 (NA VHF, WW Digi, ARRL Digi, Q65 pileup), `_EU_VHF` 2, `_FIELD_DAY` 3, `_RTTY_ROUNDUP` 4, `_FOX` 6, `_HOUND` 7 |
| `qso_progress` | `nQSOProgress`: `MFSK_QSO_CALLING` 0, `_REPLYING` 1, `_REPORT` 2, `_ROGER_REPORT` 3, `_ROGERS` 4, `_SIGNOFF` 5 |
| `band_lo_hz`, `band_hi_hz` | the audio band searched (`nfa`, `nfb`). `_init` writes 200–4000 Hz for FT8 and FT4 (`jt9`'s command line) and 600–1400 Hz for FST4 (the GUI's F Low / F High); the other modes keep their registry band |
| `rx_freq_hz`, `tol_hz` | the Rx frequency and its tolerance (`nfqso`, `ntol`). NaN is unset — 0 Hz is a frequency |
| `tx_freq_hz` | the Tx frequency (`nftx`). NaN is unset. It steers FT8's a-priori search (`MFSK_CAP_TX_FREQ`) |
| `mycall`, `mygrid`, `hiscall`, `hisgrid` | the station and the QSO partner, NUL-terminated inline text (15 and 7 characters), empty when unknown. They build the AP hypotheses |

Every field is a plain integer or float — no `enum` or `bool` — so a value
from a config file or a newer header is a wrong answer the ABI can refuse,
never an invalid Rust value. **A bad block is refused, not clamped**:
`MFSK_STATUS_INVALID_ARG` for a depth, AP mode, contest or QSO progress
outside the listed values, and for a band that is NaN or has `band_hi_hz <=
band_lo_hz` (a caller that skipped `_init`).

**The QSO context is the message's context, not the transmitter's.** AP
hypotheses lock message bits rather than steer a search, so `hiscall` in
the wrong place is a wrong hint: its AP pass cannot decode. Every mode tries
a candidate without AP first (Q65 since #555), so what a wrong context
costs is the AP gain, on exactly the weak signals it was for.

### 2.3.1 `MfskExtras` — the library's options

What the library adds beyond upstream, per mode. Initialise with
`mfsk_extras_init`, which writes "unset" everywhere — NaN for a float, 0 for
a count, -1 for a choice that has a default — **a zeroed struct is not the
same** (0 `strictness` is Strict, 0 `osd` is off). `mfsk_decoder_open` and
`mfsk_decoder_set_extras` take it; passing NULL at open is `_init`'s value.

**An option the mode does not have is refused, never dropped:**
`MFSK_STATUS_UNSUPPORTED` at `mfsk_decoder_open` or `set_extras`, with the
option named in `mfsk_last_error()` (`mfsk_decoder_last_error` for
`set_extras`), so a caller finds out before the first slot. A value out of
range is `MFSK_STATUS_INVALID_ARG`: the caller's mistake rather than a
missing option.

| field | meaning | modes |
|---|---|---|
| `sync_min` | sync threshold over the depth's; NaN is the depth's. **Not comparable across modes** | FT8, FT4, FST4 |
| `max_cand` | candidate budget over the depth's; 0 is the depth's | all |
| `osd` | -1 the depth's, 0 off, 1 on | FT8, FT4, FST4 |
| `strictness` | accept/reject profile: -1 default, 0 strict, 1 normal, 2 deep | FT8, FT4, FST4 |
| `strategy`, `sic_rounds` | `MFSK_STRATEGY_DEFAULT` 0, `_SINGLE_PASS` 1 (one pass, no subtraction), `_SIC_ROUNDS` 2 (`sic_rounds` rounds), `_SIC_EARLY` 3 (checkpointed passes). FT8 and FT4 subtract by default, as WSJT-X does | FT8: 0–3; FT4: 0–2; FST4: 0–1 (upstream's `fst4_decode` has no subtraction) |
| `eq_mode` | 0 off, 1 local per-signal equalisation. A property of the *input audio* — it flattens a passband an analogue filter has tilted — not of the search | FT8, FT4, FST4 |
| `message_filter` | 0 the protocol's own message filter, 1 the codec's verdict alone | FT8, FT4, FST4 |
| `a7` | non-zero turns on FT8's a7 list decoder (`ft8_a7.f90`), fed by the decoder's own decodes two periods back. Needs a `period` | FT8 |
| `sniper_hz` | half-width of the roofing-filter search around `rx_freq_hz`, Hz; 0 is the wide-band search. **FT8 only, by design** — it matches an operator narrowing the transceiver's analogue filter | FT8 |
| `has_ap_hint`, `ap_call1`, `ap_call2`, `ap_grid`, `ap_report` | a free-form a-priori hint beside the QSO-context AP (it wins when given): the message's fields in order, `ap_call1` being `"CQ"` for a CQ, `ap_report` `"RRR"`, `"RR73"`, `"73"` or a report | FT8, FT4, FST4, Q65 |
| `nb_percent` | impulse-noise blanker (WSJT-X's **NB**): blank the loudest `n` percent of samples, 0..=25, before the slot transform; 0 blanks nothing | FST4 (`MFSK_CAP_NOISE_BLANKER`) |
| `nb_sweep_step`, `nb_ftol_hz` | non-zero (5, 2 or 1) decodes once per blanking level `0, step, 2*step, .. 20` percent; `nb_ftol_hz` (positive, required with it) is the blanked passes' half-width around `rx_freq_hz`, so **without an Rx frequency only the 0 % pass runs**. Costs up to 21 decodes | FST4 |
| `t_early_s`, `t_late_s`, `score_threshold` | how far before/after the nominal start a frame may begin, seconds, and the coarse-sync acceptance 0..1; NaN is the mode's own | WSPR, JT9, JT65, Q65 |
| `max_cycles_per_bit` | Fano cycles per bit (`wsprd -C`); 0 is the depth's | WSPR |
| `chase_trials` | Chase trials (`nvec`); 0 is the depth's | JT65 |
| `pileup` | **Q65 Pileup**: an AP hint naming both callsigns and nothing after them leaves the spare 78th bit free, so a reply carrying the "copied last Tx" flag still matches. Needs an AP hint | Q65 |
| `max_drift` | **Max Drift**, spectrum bins 0..=50: search a linear tone drift across the frame and take it out. Costs `2*bins+1` times the plain search; 0 is off | Q65 |
| `fading_b90_ts`, `fading_model` | the fast-fading metric: spread bandwidth times symbol period (NaN is plain AWGN); model 0 Gaussian, 1 Lorentzian, read only with `fading_b90_ts` | Q65 |

A Pileup reply comes back with `MFSK_DECODE_FLAG_COPIED_LAST_TX` in
`MfskDecode::flags`, which WSJT-X shows as `#`; to send one,
`mfsk_encode_q65_flagged` is `mfsk_encode_q65` with `copied_last_tx`.

`mfsk_decoder_set_extras` replaces the whole block: what you leave unset goes
back to the depth's value. The two structs have grown before and will again, so both are
size-versioned: a caller built against an older header passes the shorter
`size`, and the library reads only that prefix, leaving the rest at their
defaults.

### 2.4 `MfskDecode` — one result row

Flat, fixed-size, written into your array. `text` is an inline
`char[MFSK_DECODE_TEXT_LEN]`, NUL-terminated.

**Set `out[0].size = sizeof(MfskDecode)` before a decode.** The library steps
through your array by that stride, not by its own `sizeof`. A program built
against an older header (a shorter struct) gets each row where its array has
it, with the fields it knows, and nothing written past `out_cap` of its rows.
One built against a newer header keeps each row's tail as it was. Each row's
`size` comes back as your stride, so the array can go straight to
`mfsk_q65_history_record` or into another decode (before #635 it came back as
the bytes written, which for a newer header shrank it to the library's own
`sizeof`, and the history then read row 1 from inside row 0's tail). `0` means "this header's struct", which
is right only when the header and the library are the same version.
A `size` below 4 or not a multiple of 4 is `MFSK_STATUS_INVALID_ARG`, and
nothing is written. Before #607 the stride was the library's own, so an older
caller's rows after the first landed in the wrong place.
`mfsk_q65_history_record` reads an array by the same rule.

| field | meaning |
|---|---|
| `size` | `sizeof(MfskDecode)` as the caller understands it |
| `mode` | the **concrete sub-mode**, not the family — all five FST4 periods and all ten Q65 sub-modes report distinctly |
| `text` | decoded message, `<...>` resolved against the decoder's table where it can |
| `freq_hz`, `dt_sec`, `snr_db` | carrier, time offset from the slot's `dt = 0` reference, SNR in a 2500 Hz reference bandwidth |
| `sync_score` | sync score of this decode, on the scale of the mode's own search (not comparable between modes). `0.0` with `MFSK_DECODE_FLAG_HAS_SYNC_SCORE` clear for WSPR, JT9, JT65, Q65 and FT8's a7/a8 list decodes |
| `sync_cv` | coefficient of variation of the per-block sync powers — near 0 on a stable channel, elevated under QSB. The only fading indicator the row carries; `0.0` with `MFSK_DECODE_FLAG_HAS_SYNC_CV` clear where `sync_score` is absent |
| `hard_errors` | hard-decision errors the FEC corrected; `0` with `MFSK_DECODE_FLAG_HAS_HARD_ERRORS` clear for WSPR, JT9, JT65 and Q65, which report no count (a clean decode is `0` with the flag set) |
| `info_bits` | width of the information block `mfsk_decoder_copy_info` returns: 91 for FT8 and FT4, 101 for FST4, 50 for WSPR, 72 for JT9 and JT65, 77 for Q65 |
| `pass` | which decode pass produced the row. **Protocol-private** — diagnostics, not logic |
| `flags` | bit 0 = `MFSK_DECODE_FLAG_HASH_RESOLVED`, the text needed the hash table to resolve a `<...>` reference; bit 1 = `MFSK_DECODE_FLAG_COPIED_LAST_TX`, a Q65 Pileup reply (WSJT-X's `#`); bits 2-4 = `MFSK_DECODE_FLAG_HAS_SYNC_SCORE` / `_HAS_SYNC_CV` / `_HAS_HARD_ERRORS`, the three numbers above are the mode's own and not the `0` of a mode that reports none |
| `key_bits`, `key` | the message's identity key: `key_bits` bits (77 for FT8, FT4, FST4 and Q65; 72 for JT9 and JT65; 50 for WSPR; 0: none), packed most significant bit first into the 10 bytes of `key`, zero-padded. The same message in two decoders has one key even when its text differs (a `<...>` resolved in one only); one message at two frequencies has one key too. For the 77-bit modes it is the first 77 bits of `mfsk_decoder_copy_info`'s block |
| `delivery` | which delivery of the period this row is, or came from: a row given to the callback carries its position (0, 1, 2...), a returned row the position of the delivery it was, so the two pair exactly. `-1` for a returned row the callback never saw, and for every row of a call with no callback |
| `stage` | when a `mfsk_decoder_decode_prefix_*` sequence found the row: `MFSK_STAGE_EARLY` (a call before the period ended — FT8's checkpoint A, ~11.8 s, in time to answer in the next period), `MFSK_STAGE_FINAL` (the call whose audio was the whole period), `MFSK_STAGE_NONE` from a plain decode. Appended |

### 2.5 Streaming capture

A one-slot holder you push audio into, sized from the mode's own
`slot_samples_12k` — so FST4-300's 3.6 M-sample slot works the same way
FT4's 90 000-sample one does. It cuts slots on the mode's UTC grid from the
sample count, the way the IQ receiver does (§2.8.2): with no clock set the
grid free-runs from the first sample, which is exactly right for replaying a
recording; with one, a slot starts on its own boundary. At most one completed
slot waits, and a newer one replaces it (`mfsk_stream_dropped` counts them).

```c
MfskStream *mfsk_stream_open(uint32_t mode, uint32_t sample_rate, MfskStatus *out);
MfskStatus  mfsk_stream_push_i16(MfskStream *s, const int16_t *samples, size_t n);
MfskStatus  mfsk_stream_push_f32(MfskStream *s, const float *samples, size_t n);
uint64_t    mfsk_stream_position(const MfskStream *s);     /* 12 kHz samples taken in: its clock */
MfskStatus  mfsk_stream_set_time(MfskStream *s, int64_t utc_ns, uint64_t at_sample,
                                 int32_t *out_change);     /* MFSK_CLOCK_* */
bool        mfsk_stream_slot_ready(const MfskStream *s);
bool        mfsk_stream_slot_is_whole(const MfskStream *s);
MfskStatus  mfsk_stream_set_prefix_points(MfskStream *s, const size_t *points, size_t n);
uint64_t    mfsk_stream_dropped(const MfskStream *s);
size_t      mfsk_stream_take_slot_i16(MfskStream *s, int16_t *out, size_t cap,
                                      int64_t *out_period, int64_t *out_utc_ns);
void        mfsk_stream_clear(MfskStream *s);
void        mfsk_stream_close(MfskStream *s);

/* fused: decode straight out of the ring */
MfskStatus  mfsk_decoder_decode_stream(MfskDecoder *dec, MfskStream *stream,
                                       MfskDecode *out, size_t out_cap, size_t *out_len,
                                       int64_t *out_period, int64_t *out_slot_start_utc_ns);
MfskStatus  mfsk_decoder_prefix_points(const MfskDecoder *dec, size_t *out, size_t cap,
                                       size_t *out_len);
```

**No `Instant`, no `SystemTime`, no clock of any kind.** The host says
that sample `at_sample` (counted by `mfsk_stream_position`) was at UTC
`utc_ns`, as often as it has a reading; the stream follows the readings at
up to 400 ppm, so noisy readings and a drifting clock move slot boundaries by
milliseconds and lose no slot. `*out_change` receives `MFSK_CLOCK_FIRST` 0
(anchored), `MFSK_CLOCK_SLEWED` 1 (moved by at most the slew limit; open
slots are unaffected) or `MFSK_CLOCK_STEPPED` 2 (more than a second away: the
clock re-anchored and the slot that straddled the jump is dropped).

Prefer `mfsk_decoder_decode_stream` over take-then-decode: taking
FST4-300's slot out and handing it back in moves 7 MB for nothing. It uses
the slot's own index as the period (so a7 and averaging see consecutive
periods without the caller counting) and returns
`MFSK_STATUS_UNSUPPORTED` with `*out_len = 0` when no slot is ready yet, so
a caller can poll it instead of `mfsk_stream_slot_ready`. The stream and the
decoder must be of the same mode (`MFSK_STATUS_INVALID_ARG` otherwise). A
mode that is not cut into slots cannot open a stream (`UNSUPPORTED`).
`mfsk_stream_take_slot_i16` copies the slot out for a caller that wants the
samples: it returns the count written (0 if none is ready or `cap` is too
small) and the slot's period and, with a clock, its UTC start
(`*out_utc_ns` is 0 without one).

**Early decode on a stream is opt-in (#601).** `mfsk_decoder_prefix_points`
reports where a decoder's `decode_prefix` sequence does work before the whole
period: `141696, 162432` for FT8 at Normal or Deep depth (its `SicEarly`
strategy), none for every other mode and setting; ask again after changing
the params or extras. Hand them to `mfsk_stream_set_prefix_points`, and from
the next slot that opens the stream also makes **the slot so far** ready at
each point, then the whole slot. `mfsk_decoder_decode_stream` decodes a
ready prefix as `decode_prefix` would, so checkpoint A's rows come back, and
reach the `on_decode` callback, at ~11.8 s with `stage == MFSK_STAGE_EARLY`;
the whole slot's call returns the period's complete set, the early rows in it
still marked early. `mfsk_stream_slot_is_whole` tells a prefix from a whole
slot, and `mfsk_stream_take_slot_i16` returns a prefix's shorter length. A
newer delivery of the same period replaces an untaken prefix without counting
in `mfsk_stream_dropped`; a period the stream already delivered part of is
not delivered again after a clock steps back, so no decode splices two
recordings. It is opt-in because an existing caller of `take_slot_i16` would
otherwise start receiving short slots. `n == 0` turns it off.

### 2.6 Transmit

Three stages, each writing into a buffer you sized:

```text
mfsk_pack77*  →  mfsk_message_to_tones  →  mfsk_tones_to_i16 / _f32
```

Size the buffers with `mfsk_symbol_count(mode)` and
`mfsk_synth_output_len(mode)`. **Ask for the size rather than baking
it** — the five FST4 sub-modes differ by a factor of 30 in samples per
symbol (720 → 21 504), so a constant taken from 60A is silently wrong
for the other four.

The seven `mfsk_encode_*` helpers are the one-call shortcut for the
common `call1 / call2 / report` message:

```c
MfskStatus mfsk_encode_ft8(const char *call1, const char *call2, const char *report,
                           float freq_hz, float *out, size_t cap, size_t *out_len);
```

and likewise `mfsk_encode_ft4`, `_fst4s60`, `_wspr` (call, grid,
power_dbm), `_jt9`, `_jt65`, `_q65` (leading `submode`). For a type-1,
type-4 or free-text message, go through `mfsk_pack77_*` and the tone
pipeline instead.

### 2.7 Introspection

```c
uint32_t    mfsk_mode_count(void);                    /* modes in THIS build */
MfskStatus  mfsk_mode_at(uint32_t index, MfskMode *out);
const char *mfsk_mode_name(uint32_t mode);            /* static, do not free */
MfskStatus  mfsk_mode_from_name(const char *name, MfskMode *out);
MfskStatus  mfsk_mode_info(uint32_t mode, MfskModeInfo *out);
uint64_t    mfsk_mode_caps(uint32_t mode);            /* MFSK_CAP_* bits */
MfskStatus  mfsk_params_init(uint32_t mode, MfskParams *out);
MfskStatus  mfsk_extras_init(MfskExtras *out);
uint32_t    mfsk_abi_version(void);
uint32_t    mfsk_version(void);
```

**`MfskMode` addresses every mode, and its discriminants are ABI.** One
per registry entry plus MSK144 and JTTY, assigned once and never reordered —
deliberately *not* registry indices, because registry membership is
feature-gated and a build without `q65` would shift every index after
it. `mfsk_mode_count` / `mfsk_mode_at` say which of them this
particular build has.

**Capabilities are published, not inferred.** The word is
`mfsk_mode_caps(mode)`, or `MfskModeInfo::caps`. Since the decoder handle
serves all 20 slot modes, `MFSK_CAP_DECODE_HANDLE` no longer says whether a
decoder opens; it says the mode belongs to the **77-bit-message slot family**
(FT8, FT4, FST4), the one the QSO-context AP of `MfskParams`, a7 and the
sniper window apply to. WSPR, JT9, JT65 and Q65 decode through the same
handle with their own `MfskExtras` fields, and an option they lack is
`MFSK_STATUS_UNSUPPORTED`. The other bits say which options a mode honours,
so a caller can read them rather than know them:

| bit | constant | meaning |
|---|---|---|
| 0 | `MFSK_CAP_DECODE_HANDLE` | the 77-bit slot family: QSO-context AP, a7 and sniper apply |
| 1 | `MFSK_CAP_SNIPER` | narrow-band single-target search (`sniper_hz`). **FT8 only, by design** |
| 2 | `MFSK_CAP_AP_NARROW` | AP hint on a targeted search |
| 3 | `MFSK_CAP_AP_WIDEBAND` | AP hint on the wide-band search |
| 4 | `MFSK_CAP_SIC_ROUNDS` | flat successive-interference cancellation (`MFSK_STRATEGY_SIC_ROUNDS`) |
| 5 | `MFSK_CAP_SIC_EARLY` | checkpoint-emulation early decode (`MFSK_STRATEGY_SIC_EARLY`). FT8 only |
| 6 | `MFSK_CAP_OSD` | the OSD *switch* is honoured. Absent means "cannot be turned off", not "does not have it" |
| 7 | `MFSK_CAP_EQ_MODE` | equalisation reaches the decoder |
| 8 | `MFSK_CAP_STRICTNESS` | the strictness profile is honoured rather than accepted and dropped |
| 9 | `MFSK_CAP_BUDGET` | `mfsk_decoder_set_budget` is accepted: every mode with a decoder |
| 10–15 | `MFSK_CAP_KNOWN_FILTER` … `MFSK_CAP_STREAM_RECEIVER` | see `mfsk.h`. `_KNOWN_FILTER`, `_KNOWN_SUBTRACT` and `_FFT_CACHE` describe the Rust API; the C ABI has no call for them since the 0.12 `keep_known` / `keep_fft_cache` went |
| 16 | `MFSK_CAP_NOISE_BLANKER` | WSJT-X's impulse-noise blanker (`nb_percent`, `nb_sweep_step`). **Every FST4 sub-mode and no other** |
| 17 | `MFSK_CAP_TX_FREQ` | the transmit frequency (`tx_freq_hz`) steers the a-priori search. **FT8 only** |

The bits mirror `mfsk_core::registry::caps`, which
`mfsk-core/tests/registry_caps.rs` ties to the trait impls in both
directions — naming a protocol that lacks a trait is a *compile* error
there, and implementing one without setting the bit is a runtime
failure. `mfsk-ffi/tests/mode_introspection.rs` closes the last link by
comparing each `MFSK_CAP_*` against the registry constant it mirrors.
That chain exists because a hand-written capability table lies within
two releases.

**There is no `mfsk_mode_defaults` any more.** Defaults are data and
`mfsk_params_init` writes them; `MfskExtras` leaves the search settings
at "the depth's own", so there is no per-mode `sync_min` to read and
misapply across modes (FT4's baseline-normalised score, FT8's and FST4's
absolute Costas scores and the 0‥1 sync fraction of WSPR, JT9, JT65 and
Q65 are not comparable). `mfsk_params_init` is `MFSK_STATUS_UNKNOWN_PROTOCOL`
for a mode with no decoder (MSK144, JTTY, uvpacket).

**`MfskModeInfo::decode_fft1_size` is the field to read before
budgeting.** It is the forward FFT the decoder takes over the whole
slot: FT4 92 160 points, FST4-300 **4 194 304** — a factor of 45 that
no other field hints at, and the reason "one call shape for every mode"
is wrong as a memory story on a phone.

**Size versioning.** `MfskModeInfo`, `MfskParams`, `MfskExtras`,
`MfskDecode` and the other rows all lead with `size`. Set it to
your `sizeof` (or zero the struct and the library fills it in); a
library newer than your header writes only the prefix you declared and
rewrites `size` to what it actually wrote (for an array of `MfskDecode` rows, to
your stride, so the array stays readable as it is), and reads only the prefix of
an input struct you declared.

`mfsk_abi_version()` is separate from `mfsk_version()` on purpose: the
crate version moves for reasons that have nothing to do with the
boundary.

### 2.8 Q65: the lists and the sub-mode numbering

Q65 decodes through the ordinary handle (§2.2): Pileup, Max Drift and the
fast-fading metric are `MfskExtras` (§2.3.1), the EME delay and averaging
are `MfskParams::flags`, and the AP list is built from `mycall`, `hiscall`,
`hisgrid`, `qso_progress` and `ap_mode`, as upstream builds it. A mode opens as
a `MfskMode` (`MFSK_MODE_Q65A30`); `dt_sec` is measured from the mode's
nominal start as WSJT-X's DT column is. What stays outside the handle is the
two lists WSJT-X keeps, which are objects you own because the decoder is
stateless about them and the application's clock is the only clock, and the
sub-mode numbering that `mfsk_encode_q65*` still takes.

`MfskQ65History` (`q65_hist`, the 100 most recent decodes): `mfsk_q65_history_new` /
`_free` / `_len` / `_push(freq, text)`, `mfsk_q65_history_record(rows, n)` for a whole
row array, and `mfsk_q65_history_lookup(rx_freq, &dx)` for the DX call and grid a
"Decode Again" with none entered would read. `MfskQ65Callers` (`q65_hist2`, up to 50
stations that called with a grid): `mfsk_q65_callers_new` / `_free` / `_len` /
`_get`, `mfsk_q65_callers_record(freq, text, now)`, `mfsk_q65_callers_expire(now)`,
`mfsk_q65_callers_remove(call)`. Times are Unix seconds you pass, since the
library reads no clock. Neither is thread-safe. The contest list reaches a
decoder through `mfsk_decoder_set_q65_callers`, which copies it, so a list
changed afterwards has to be set again.

`MfskQ65SubMode` has **its own numbering**, where `a15` is 6. It is the
`submode` argument of `mfsk_encode_q65` and `mfsk_encode_q65_flagged`; bridge it
to `MfskMode` rather than assuming the two agree.

### 2.8.1 JTTY — a receiver handle instead of a slot call

JTTY (WSJT-X 3.2's keyboard mode) has no slot: frames start whenever the
sender likes and a message is several of them, so the receiver keeps state
and the output is message *updates*. The mode is `MFSK_MODE_JTTY`, it
publishes `MFSK_CAP_STREAM_RECEIVER` and `MFSK_CAP_ENCODE` (and not
`MFSK_CAP_DECODE_HANDLE`), and
`mfsk_mode_info` describes one frame — `t_slot_s` is the frame period
(1.888 s), `slot_samples_12k` 22 656.

```c
MfskJttyParams p;  mfsk_jtty_params_init(&p);        /* rjtty's defaults; NULL is the same */
MfskStatus st;
MfskJttyReceiver *rx = mfsk_jtty_open(48000, &p, &st); /* any rate; != 12000 is resampled */

for (each audio callback)  {                          /* chunks of any size */
    mfsk_jtty_push_i16(rx, pcm, n);                   /* decodes what it completes, then returns */
    MfskJttyUpdate u = {0};                           /* u.size = sizeof u, or 0 */
    while (mfsk_jtty_poll(rx, &u) == 1)               /* 1 = row written, 0 = none, <0 = MfskStatus */
        show(u.id, u.text, u.complete, u.f1_hz);      /* replace the row with this id */
}
mfsk_jtty_finish(rx);                                 /* a recording ended: last, incomplete rows */
mfsk_jtty_close(rx);
```

Decoding runs inside `push` on the calling thread (and on the pool
`mfsk_runtime_configure` installed): a few tens of milliseconds per 0.47 s of
audio completed, so call it from a worker, not the UI thread. The updates wait
in a queue in the handle; the queue **coalesces per message** — a message that
grew twice between polls is returned once with its latest text, which is
upstream's rule and what bounds the queue (at most 1024 distinct messages; a
caller that never polls loses the oldest). `id` is stable for a message's life.
The handle is not thread-safe: one thread at a time, like `MfskStream`. A build
without the `jtty` feature keeps the entry points and answers
`MFSK_STATUS_UNKNOWN_PROTOCOL`.

**SNR.** `MfskJttyUpdate::snr_db` (appended after `text`, so a caller that passed a smaller `size`
does not get it) is the first frame's SNR in 2 500 Hz, floored at -17 dB, as WSJT-X
v3.3.0-beta1 reports it: show it rounded. It is the simple estimate upstream settled on, so a
strong signal reads low (a true +30 dB reads about +11; at 0 and below it is within a dB).
Kotlin `MfskJttyUpdate.snrDb`, Swift `JttyUpdate.snrDb`.

**Why a row was emitted.** `MfskJttyUpdate::kind` (appended after `snr_db`) is one of
`MFSK_JTTY_UPDATE_GROWING` (0), `_COMPLETE` (1), `_EXPIRED` (2: no continuation within three frame
periods) and `_RECEPTION_ENDED` (3: `mfsk_jtty_finish` cut the message off), WSJT-X v3.3.0-beta1's
`UPDATE_*`. `complete` alone cannot tell the last two apart. Rows are coalesced per message, so it is
the reason for the latest update. Kotlin `MfskJttyUpdate.kind` (`MfskJttyUpdateKind`), Swift
`JttyUpdate.kind` (`JttyUpdate.Kind`).

**Feeding a live source.** There is no slot to align to and no clock in the library:
time is the number of samples pushed since `open` / `reset`, and `MfskJttyUpdate::start_s`
is that count over the sample rate. `start_s` is an `f32`, which keeps 8 ms at 24 hours and only a
tenth of a second after about 12 days, so read `start_time_s` (appended after `kind`, an `f64`,
the same number): Kotlin's and Swift's `startSeconds` are now `Double` from it. Map it to UTC yourself if you need to (note the UTC
of the first sample). The consequence for a live capture is that **the sample count must
follow real time**. A frame continues a message only if it starts one to three frame
periods (1.888 s, ±0.1 s) after the last, so:

- an audio callback that *drops* samples (an underrun, a USB glitch, a paused stream)
  shifts every later frame earlier and the message breaks. Push the same number of
  zeros as the audio you lost, or, after a long gap, `mfsk_jtty_reset`;
- a source that runs slightly fast or slow (a sound card whose clock is not the
  operator's) stretches the timing by its ppm error; a few hundred ppm is far inside the
  ±0.1 s over the 3 frame periods a continuation may span;
- chunk size does not matter, and there is no benefit to aligning chunks to anything.

Push from a worker, not from the audio callback or the UI thread: hand samples over
through a ring buffer, since one `push` can take tens of milliseconds. Results follow the
end of a frame by about half a second (a window is decoded when its last sample arrives).
MSK144 has the same problem in a sharper form and has no streaming receiver yet (#497).

**Transmit** is the same three stages as elsewhere, with text in front instead of
a 77-bit message:

```c
size_t n = 0;
mfsk_jtty_encode_tones("CQ K1ABC CQ", /*profile*/ 0, NULL, 0, &n);   /* size query: n = 59 */
uint8_t tones[16 * 59];
mfsk_jtty_encode_tones("CQ K1ABC CQ", 0, tones, sizeof tones, &n);   /* upstream's pack_jtty + genjtty */
int16_t pcm[16 * 59 * 384 + 4096];  size_t m;                        /* mfsk_jtty_synth_len(n) is the size */
mfsk_jtty_tones_to_i16(tones, n, 1500.0f, 8000.0f, pcm, sizeof pcm / 2, &m);
```

`mfsk_jtty_encode_tones_ex(text, profile, is_final, ...)` is the same with WSJT-X v3.3.0-beta1's
`is_final`: 0 leaves the end-of-message flag off the last frame, so a message typed in pieces can
go out as it is typed and be closed by a later one (`mfsk_jtty_encode_tones` is `is_final` = 1).

`profile` is 0 unknown, 1 Field Day, 2 RTTY Roundup; only RTTY Roundup changes the
packing (serial-number and state candidates, `599 5` → `599 005`). The packer picks
the fewest frames: a callsign, grid, report or control phrase is one frame, other text
five characters a frame. A message over 80 characters, over 16 frames, or an RTTY serial
that does not fit is `MFSK_STATUS_INVALID_ARG` with the reason in `mfsk_last_error`; an
empty message is `OK` with `*out_len = 0`. The F-key templates and N1MM tags WSJT-X wraps
around `pack_jtty` are host policy and are not in the library (see #463 for the line).

### 2.8.2 Wideband IQ — a receiver handle for an SDR stream

`mfsk_iq_*` is the C face of `mfsk_core::iq::IqReceiver` (LIBRARY.md §2.7): one
wideband complex-IQ stream in, N channels out, each carrying a mode on a dial
frequency, with the slots cut on UTC from the sample count. It is a handle of
its own, like JTTY's, because the input is IQ rather than audio and the
receiver carries state (per-channel filters, open slots, the sample clock). It
finds nothing: you say which dial carries which mode. **Each channel has a
decoder of its own** — the §2.2 handle, with its own hash table, a7 list and
averages — opened from the `MfskParams` and `MfskExtras` you give
`mfsk_iq_add_channel` (NULL for the mode's defaults).

```c
MfskStatus st;
/* 768 kS/s of CF32 centred on 14.200 MHz; iq_swap = 0 */
MfskIqReceiver *rx = mfsk_iq_open(768000, 14200000.0, MFSK_IQ_FORMAT_CF32, 0, &st);
/* Many channels? Share one polyphase filter bank instead (see "Selectivity" below):
   mfsk_iq_open_with(768000, 14200000.0, MFSK_IQ_FORMAT_CF32, 0, MFSK_IQ_CHANNELIZER_PFB, &st); */

MfskParams p;  memset(&p, 0, sizeof p);  p.size = sizeof p;
mfsk_params_init(MFSK_MODE_FT8, &p);                        /* per-channel options, as for mfsk_decoder_open */
strcpy(p.mycall, "JL1NIE");

uint32_t ft8, ft4;
mfsk_iq_add_channel(rx, 14074000.0, MFSK_MODE_FT8, &p, NULL, &ft8);  /* INVALID_ARG: DC in its window, or out of band */
mfsk_iq_add_channel(rx, 14080000.0, MFSK_MODE_FT4, NULL, NULL, &ft4);
mfsk_iq_set_time(rx, utc_ns_now, mfsk_iq_samples_in(rx), NULL);  /* no reading: the grid free-runs from sample 0 */

for (each block from the SDR) {
    mfsk_iq_push(rx, bytes, n_bytes);                       /* decodes every slot this completes, then returns */
    MfskIqDecode d = {0};                                   /* d.size = sizeof d, or 0 */
    while (mfsk_iq_poll(rx, &d) == 1)                       /* 1 = row written, 0 = none, <0 = MfskStatus */
        show(d.channel, d.text, d.abs_freq_hz, d.snr_db);
}
mfsk_iq_retune(rx, new_center_hz, &paused, &resumed);       /* the tuner moved */
mfsk_iq_gap(rx, lost_samples);                              /* samples never arrived */
mfsk_iq_close(rx);
```

`format` is one of `MFSK_IQ_FORMAT_CF32`, `_CS16`, `_CS8` (HackRF), `_CU8`
(RTL-SDR, 128 = zero) and `_CS24`, little-endian, I then Q, and `mfsk_iq_push`
takes bytes in that format; a sample split across two calls is carried over.
`iq_swap` non-zero exchanges I and Q, which sound-card IQ often needs. Any
integer rate of 12 000 or more whose ratio to 12 kHz is a small fraction is
accepted; `mfsk_iq_open` returns NULL with `INVALID_ARG` for one that is not.

A channel carries FT8, FT4, any of the five FST4 periods, WSPR, JT9, JT65 or a
Q65 sub-mode (`MfskMode`); MSK144, JTTY and uvpacket are `INVALID_ARG`, an
option the mode lacks is `UNSUPPORTED`, and a mode compiled out is
`UNKNOWN_PROTOCOL`. **Usable audio starts near 200 Hz** (the front end must
reject the sideband below the dial). Each slot is decoded with its period index,
so a7 and averaging, where the channel's params switch them on, see consecutive
periods; a slot lost to a gap or a retune breaks the run.

**The channel's decoder is borrowed.** `mfsk_iq_channel_decoder(rx, ch)` returns
it as an `MfskDecoder*` for the calls that configure one — `mfsk_decoder_set_params`,
`_set_extras`, `_add_callsign`, `_unpack77`, `_last_error`, `_set_q65_callers`,
`_clear` — which is how a skimmer changes the band, depth or DX call of a channel
between slots. **Do not close it**: it lives until the channel is removed or the
receiver closed, and the receiver decodes with it, so do not call its `decode_*`.
`mfsk_iq_channel_state(rx, ch)` says `MFSK_IQ_CHANNEL_ACTIVE` 0,
`MFSK_IQ_CHANNEL_PAUSED` 1 (see retune) or -1 for no such channel.

`MfskIqDecode` is size-versioned like the other rows. It carries `channel`
(what `add_channel` returned), the concrete `mode`, `text`, `freq_hz` (audio),
`abs_freq_hz` (the dial plus that, `double`), `dt_sec`, `snr_db`, `period` (the
slot's index on the mode's UTC grid, counted from sample 0 without a clock),
`slot_start_sample` (an index into the IQ stream) and `slot_start_utc_ns` with
`has_utc` saying whether a clock reading was set. Appended after `text`, the
row's detail as `MfskDecode` gives it (§2.4), same names and meanings:
`sync_score`, `sync_cv`, `hard_errors` (each valid when its
`MFSK_DECODE_FLAG_HAS_*` bit is set), `delivery`, `pass`, `flags`, `key_bits`
and `key`. Compare IQ rows by `key` and `freq_hz`, not by text.

**Time and discontinuities.** The sample count is the clock and the library
reads none. `mfsk_iq_set_time(rx, utc_ns, at_sample, &change)` says that complex
sample `at_sample` (the count `mfsk_iq_samples_in` returns) was at UTC `utc_ns`.
Call it as often as you have a reading: the receiver follows the readings at up
to 400 ppm, so a drifting crystal or host clock moves slot boundaries by
milliseconds and loses no slot, and only a jump of more than a second (`change`
is `MFSK_CLOCK_STEPPED`, as `MFSK_CLOCK_FIRST` / `_SLEWED` for the others) drops
the slots that straddle it. A slot is decoded when all of it has arrived; the
partial slot the stream opened in the middle of is not. `mfsk_iq_gap` drops
every open slot (audio across a hole is not a slot) and keeps the clock going.
`mfsk_iq_retune` drops them too; a channel whose audio window no longer fits the
new band is **paused** rather than failing the call — it keeps its dial and its
decoder and resumes when a later retune brings it back inside — and the call
reports how many channels paused and resumed. A recording needs a moment of
padding after its end, as a live stream has: a slot's last audio sample comes
out a few filter lengths after the last IQ sample that carries it.

**Early decode is on by default (#601).** A channel whose decoder has
checkpoints (FT8 at Normal or Deep depth) is also decoded at ~11.8 s inside
`mfsk_iq_push`: those rows reach the channel decoder's
`mfsk_decoder_set_on_decode` callback and `mfsk_iq_poll` before the slot is
whole, with `stage == MFSK_STAGE_EARLY` (`MfskIqDecode::stage`, appended).
The whole slot then queues the rest and not the early rows again; the
callback, too, sees each row once. The trade: an early row whose `<...>` the
period later resolves stays unresolved in the queue; pair by `delivery` for
the resolved text. The handle owns the decoder, so only *when* a row arrives
changes, as WSJT-X shows checkpoint A's rows. The points follow the channel
decoder's settings, read on every push, so `mfsk_decoder_set_params` on the
borrowed decoder takes effect from the next slot. `mfsk_iq_set_early(rx,
channel, false)` turns it off for a channel: whole slots only, rows with
`MFSK_STAGE_NONE`, as before.

**Threads.** Decoding runs inside `mfsk_iq_push` (hundreds of milliseconds for a
busy FT8 slot), on the calling thread and on the pool `mfsk_runtime_configure`
installed. Push from a worker, not the UI or the SDR's own callback thread. The
rows wait in a queue in the handle that `mfsk_iq_poll` drains (at most 4096; a
caller that never polls loses the oldest). It is a poll rather than a callback
for JTTY's reason: no user-data contract to cross the boundary, and a wrapper
in Kotlin, Swift or C# is simpler over a poll. The handle is not thread-safe:
one thread at a time.

Selectivity is 120 dB outside the channel's window. `mfsk_iq_open` gives the
direct path, where each channel mixes at the input rate and cost is linear in
channels (0.92 % of a core per channel at 768 kS/s). For many channels,
`mfsk_iq_open_with(..., MFSK_IQ_CHANNELIZER_PFB, &status)` shares one polyphase
filter bank among them: 2.7 % of a core for one channel at 768 kS/s, 10 % for
32, 35 % for 128, against 30 % for 32 direct. Break-even is about four
channels. The bank needs 40 kS/s or more (`INVALID_ARG` otherwise); rows and
every other call are the same on both.

### 2.9 Messages

```c
MfskStatus mfsk_pack77(const char *call1, const char *call2, const char *report,
                       uint8_t *out_message77);
MfskStatus mfsk_pack77_type1(const char *call1, const char *call2, const char *grid,
                             uint8_t *out_message77);
MfskStatus mfsk_pack77_type4(const char *nonstd_call, const char *std_call,
                             const char *report, bool is_cq, uint8_t *out_message77);
MfskStatus mfsk_pack77_free_text(const char *text, uint8_t *out_message77);
MfskStatus mfsk_unpack77(const uint8_t *message77, char *out, size_t cap, size_t *out_len);
MfskStatus mfsk_decoder_unpack77(const MfskDecoder *dec, const uint8_t *message77,
                                 char *out, size_t cap, size_t *out_len);
```

`out_message77` is a caller-owned 77-byte buffer in every case; none of
these allocate. `mfsk_pack77_free_text` packs **up to 13 characters of
free text** — despite the name it frees nothing. `mfsk_unpack77` leaves
`<...>` hash references unresolved; `mfsk_decoder_unpack77` resolves them
against that decoder's own table, which is the one its decodes populated.
Both report the size needed if `cap` is too small.

### 2.10 Threads and the runtime

```c
MfskStatus mfsk_runtime_configure(const MfskRuntimeConfig *cfg);
uint32_t   mfsk_runtime_thread_count(void);
```

* **A decoder is single-threaded.** It mutates its hash table and
  averages on every decode. One per concurrent thread; concurrent decodes on
  separate decoders are supported and cheap.
* Even with `parallel` on, decoding otherwise uses rayon's **global**
  pool: `num_cpus` threads with 2 MiB stacks, spawned lazily on the
  first decode and never joined. On Android those threads are not
  attached to ART, so a callback from one cannot touch a `JNIEnv`; on
  iOS they sit outside GCD's quality-of-service classes, competing with
  the audio render thread; on both they keep running after the app is
  backgrounded.
* `on_thread_start` / `on_thread_stop` map onto rayon's
  `start_handler` / `exit_handler`, which is what makes
  `AttachCurrentThread` / `DetachCurrentThread` possible from JNI — and
  therefore what makes a decode callback legal from a worker thread
  there. `num_threads = 1` forces serial decoding, which is also what a
  build without `parallel` does.
* **Call it once, before the first decode.** A second call returns
  `MFSK_STATUS_UNSUPPORTED` rather than being silently ignored: rayon
  cannot rebuild a pool its threads may be parked in.

### 2.11 Errors and memory rules

1. **Handles**: `mfsk_decoder_open` / `mfsk_decoder_close`,
   `mfsk_stream_open` / `mfsk_stream_close`,
   `mfsk_jtty_open` / `_close`, `mfsk_iq_open` / `_close`,
   `mfsk_q65_history_new` / `_free`, `mfsk_q65_callers_new` / `_free`.
   Close and free are idempotent on `NULL`. A channel's decoder
   (`mfsk_iq_channel_decoder`) is borrowed from its receiver and is not closed.
2. **Result rows, audio and text** go into caller-owned buffers.
   Nothing returned needs freeing. The two functions returning a
   `const char*` — `mfsk_last_error` and `mfsk_decoder_last_error` —
   hand back a borrowed pointer, not an allocation; so does
   `mfsk_mode_name`, whose string is static. The row a decode callback
   receives is valid for that call only; copy what you keep.
3. **Errors**: on a non-`MFSK_STATUS_OK` return, call
   `mfsk_decoder_last_error(d)` for a decoder call, or
   `mfsk_last_error()` for a free function (and for a failed `_open`, which
   has no handle yet), on the **same thread**. The global is thread-local:
   a Kotlin coroutine or Swift `async` caller that hops threads between the
   status and the message reads NULL from it, which is what the per-handle
   slot is for. The returned pointer is valid until the next fallible call on
   that thread, or on that handle.

`MfskStatus`: `OK = 0`, `NULL_POINTER = -1`, `INVALID_ARG = -2`,
`UNKNOWN_PROTOCOL = -3` (not in this build, or no decoder for that mode),
`DECODE_FAILED = -4`, `INTERNAL = -5` (always a bug), `UNSUPPORTED = -6` (the
mode is here but does not offer what was asked).

### 2.12 Symbol index

98 exported functions, grouped:

| group | symbols |
|---|---|
| decoder (22) | `mfsk_params_init` `mfsk_extras_init` `mfsk_decoder_open` `mfsk_decoder_close` `mfsk_decoder_last_error` `mfsk_decoder_set_params` `mfsk_decoder_set_extras` `mfsk_decoder_set_q65_callers` `mfsk_decoder_clear` `mfsk_decoder_add_callsign` `mfsk_decoder_set_on_decode` `mfsk_decoder_set_budget` `mfsk_decoder_last_budget` `mfsk_decoder_delivery_is_exact` `mfsk_decoder_decode_i16` `mfsk_decoder_decode_f32` `mfsk_decoder_decode_prefix_i16` `mfsk_decoder_decode_prefix_f32` `mfsk_decoder_copy_info` `mfsk_decoder_decode_stream` `mfsk_decoder_prefix_points` `mfsk_decoder_unpack77` |
| streaming (12) | `mfsk_stream_open` `mfsk_stream_close` `mfsk_stream_push_i16` `mfsk_stream_push_f32` `mfsk_stream_position` `mfsk_stream_set_time` `mfsk_stream_slot_ready` `mfsk_stream_slot_is_whole` `mfsk_stream_set_prefix_points` `mfsk_stream_dropped` `mfsk_stream_take_slot_i16` `mfsk_stream_clear` |
| introspection (8) | `mfsk_mode_count` `mfsk_mode_at` `mfsk_mode_name` `mfsk_mode_from_name` `mfsk_mode_info` `mfsk_mode_caps` `mfsk_abi_version` `mfsk_version` |
| transmit (13) | `mfsk_encode_ft8` `mfsk_encode_ft4` `mfsk_encode_fst4s60` `mfsk_encode_wspr` `mfsk_encode_jt9` `mfsk_encode_jt65` `mfsk_encode_q65` `mfsk_encode_q65_flagged` `mfsk_symbol_count` `mfsk_synth_output_len` `mfsk_message_to_tones` `mfsk_tones_to_i16` `mfsk_tones_to_f32` |
| Q65 lists (13) | `mfsk_q65_history_new` `mfsk_q65_history_free` `mfsk_q65_history_push` `mfsk_q65_history_record` `mfsk_q65_history_len` `mfsk_q65_history_lookup` `mfsk_q65_callers_new` `mfsk_q65_callers_free` `mfsk_q65_callers_record` `mfsk_q65_callers_expire` `mfsk_q65_callers_remove` `mfsk_q65_callers_len` `mfsk_q65_callers_get` |
| messages (5) | `mfsk_pack77` `mfsk_pack77_type1` `mfsk_pack77_free_text` `mfsk_pack77_type4` `mfsk_unpack77` |
| JTTY (15) | `mfsk_jtty_params_init` `mfsk_jtty_open` `mfsk_jtty_close` `mfsk_jtty_set_params` `mfsk_jtty_push_i16` `mfsk_jtty_push_f32` `mfsk_jtty_finish` `mfsk_jtty_reset` `mfsk_jtty_pending` `mfsk_jtty_poll` `mfsk_jtty_encode_tones` `mfsk_jtty_encode_tones_ex` `mfsk_jtty_synth_len` `mfsk_jtty_tones_to_i16` `mfsk_jtty_tones_to_f32` |
| IQ (15) | `mfsk_iq_open` `mfsk_iq_open_with` `mfsk_iq_close` `mfsk_iq_add_channel` `mfsk_iq_channel_decoder` `mfsk_iq_channel_state` `mfsk_iq_set_early` `mfsk_iq_remove_channel` `mfsk_iq_set_time` `mfsk_iq_retune` `mfsk_iq_gap` `mfsk_iq_push` `mfsk_iq_samples_in` `mfsk_iq_pending` `mfsk_iq_poll` |
| runtime (3) | `mfsk_last_error` `mfsk_runtime_configure` `mfsk_runtime_thread_count` |

---

## 3. Porting

### 3.1 From the 0.13 ABI

0.14.0 keeps ABI version 3: every struct grew by appended fields only, and no
function changed its signature, so a 0.13 program links and runs. What a C
caller has to look at is behaviour.

| area | 0.13 | 0.14 |
|---|---|---|
| a result array's stride | rows written `sizeof(MfskDecode)` apart, the library's own: a program built against an older, shorter header got row 1 onward in the wrong place and wrote past its buffer | rows are `out[0].size` apart, the caller's; set it to `sizeof(MfskDecode)` (or 0 for this header's) before the call, and a `size` that cannot be a struct size is `MFSK_STATUS_INVALID_ARG` with nothing written ([#607](https://github.com/jl1nie/mfsk-core/issues/607), §2.4) |
| each row's `size` after a decode | the bytes written, so a newer header's array came back saying the library's `sizeof` | your stride, so the array can go straight to `mfsk_q65_history_record` or the next decode; read what the library wrote from the header, not the row ([#635](https://github.com/jl1nie/mfsk-core/issues/635)) |
| retrying a call refused for a short `out` | `mfsk_decoder_decode_stream` had already taken the slot, so the retry got `MFSK_STATUS_UNSUPPORTED`; the other `decode_*` calls decoded again, firing the callback twice and stepping a Q65 average twice | the same call with room is answered from the rows already found, without decoding ([#633](https://github.com/jl1nie/mfsk-core/issues/633)) |
| `float` at a rate other than 12 kHz | resampled and peak-normalised to 16 bits per call | resampled keeping the level and handed over as at 12 kHz: the decoder sets the gain (RMS) once per period ([#634](https://github.com/jl1nie/mfsk-core/issues/634)) |
| `sync_score`, `sync_cv`, `hard_errors` | `0` from WSPR, JT9, JT65 and Q65 | still `0` there; `flags` bits 2–4 (`MFSK_DECODE_FLAG_HAS_*`) say which the row's mode measured ([#594](https://github.com/jl1nie/mfsk-core/issues/594)) |
| `info_bits` / `mfsk_decoder_copy_info` | FT8, FT4 and FST4 only | every mode: WSPR 50 bits, JT9 and JT65 72, Q65 77; rows also carry a `key` ([#592](https://github.com/jl1nie/mfsk-core/issues/592)) |
| `mfsk_decoder_set_budget` on WSPR, JT9, JT65, Q65 | `MFSK_STATUS_UNSUPPORTED` | accepted: they publish `MFSK_CAP_BUDGET` and poll once per candidate ([#593](https://github.com/jl1nie/mfsk-core/issues/593)) |
| `mfsk_iq_push`, an FT8 channel | whole slots only | **early by default**: checkpoint A's rows at ~11.8 s with `stage == MFSK_STAGE_EARLY`, the rest at the end without repeating them; `mfsk_iq_set_early(rx, channel, false)` restores whole slots ([#601](https://github.com/jl1nie/mfsk-core/issues/601), §2.8.2) |
| `MfskStream` | whole slots | unchanged unless you opt in with `mfsk_decoder_prefix_points` → `mfsk_stream_set_prefix_points`, then decode every delivery with `mfsk_decoder_decode_stream` (§2.5) |

New, all appended: `mfsk_decoder_decode_prefix_i16` / `_f32`,
`mfsk_decoder_prefix_points`, `mfsk_decoder_delivery_is_exact`,
`mfsk_stream_set_prefix_points`, `mfsk_stream_slot_is_whole`,
`mfsk_iq_set_early`; `MfskDecode` gains `key_bits`, `key`, `delivery`,
`stage`; `MfskIqDecode` gains `MfskDecode`'s detail (`sync_score`,
`sync_cv`, `hard_errors`, `pass`, `flags`, `key_bits`, `key`, `delivery`,
`stage`); `MfskBudgetReport` gains `rows_subtracted`. The decode results
move as `LIBRARY.md` §1.3 lists (callsign prefixes no longer checked, the OSD
ported line for line, JT9's sub-bin correction).

**Kotlin and Swift** follow the same ABI, and their row types are source
breaks: `syncScore`, `syncCv` / `syncCV` and `hardErrors` are nullable
(`null` / `nil` where the mode does not measure them), and rows gain `key`,
`keyBits`, `delivery` and `stage`. New: `decodePrefix`, `prefixPoints`,
`deliveryIsExact`, a stream's `setPrefixPoints` and `slotIsWhole` /
`isSlotWhole`, the IQ receiver's `setEarly`, and the budget report's
`rowsSubtracted`. Kotlin's lent channel decoder takes `onDecode` and
`setBudget`. As in C, an FT8 channel of the IQ receiver is early by default.

### 3.2 From the 0.12 ABI

0.13.0 replaced the C decode surface (ABI version 2 → 3). `mfsk-ffi` is
`publish = false`, and the in-repo C++ driver, the Kotlin binding and the Swift
package moved with it, so a C consumer porting across rewrites its decode calls
and keeps the rest (transmit, messages, streaming capture's push side, JTTY,
the Q65 lists, the runtime).

| 0.12 | 0.13 |
|---|---|
| `MfskDecodeSession`, `mfsk_session_open` / `_close` | `MfskDecoder`, `mfsk_decoder_open` / `_close` — a different type on purpose, since the two own different Rust values |
| `MfskDecodeParams` + `mfsk_decode_params_init` | two structs: `MfskParams` (WSJT-X's parameter block, `mfsk_params_init`) and `MfskExtras` (the library's options, `mfsk_extras_init`). `freq_min_hz` / `freq_max_hz` → `band_lo_hz` / `band_hi_hz`; `freq_hint_hz` → `rx_freq_hz` (+ `tol_hz`); `tx_freq_hz` stays; `depth` / `strictness` / `eq_mode` / `sync_min` / `max_cand` / `sic_*` / `single_pass` / `search_hz` → `depth` + the `MfskExtras` fields of the same names (`strategy`, `sniper_hz`); `has_ap_hint`, `ap_*` → `MfskExtras`; `nb_*` → `MfskExtras` |
| `mfsk_session_decode_i16` / `_f32` with a per-call `params` | `mfsk_decoder_decode_i16` / `_f32` with a `period` instead; change the block with `mfsk_decoder_set_params` / `_set_extras` |
| `mfsk_session_decode_stream(…, params, …, double *utc)` | `mfsk_decoder_decode_stream(…, int64_t *period, int64_t *utc_ns)` |
| `mfsk_session_set_on_decode` / `_set_budget` / `_last_budget` / `_add_callsign` / `_copy_info` / `_last_error` | same names with `mfsk_decoder_` |
| `mfsk_session_keep_known` / `_known_count` / `_keep_fft_cache` | gone; a decoder carries the state upstream carries. FT8's a7 (`MfskExtras::a7`, with a `period`) replaces carrying decodes forward |
| `mfsk_stream_set_epoch(s, double utc_s)`, `_buffered`, `_take_slot_i16(…, double *utc)` | `mfsk_stream_set_time(s, utc_ns, at_sample, &change)` with `mfsk_stream_position`, `_dropped`; `_take_slot_i16(…, int64_t *period, int64_t *utc_ns)`. The stream follows the readings (400 ppm) instead of taking an epoch |
| `mfsk_wspr_decode`, `mfsk_jt9_decode_at`, `mfsk_jt65_decode_at` | `mfsk_decoder_open(MFSK_MODE_WSPR / _JT9 / _JT65, …)` and `mfsk_decoder_decode_*`; the band is `band_lo_hz` / `band_hi_hz`, the frame window `t_early_s` / `t_late_s` |
| `mfsk_q65_decode`, `_with_ap`, `_fading`, `_with_ap_list`, `_decode_ex`, `mfsk_q65_params_init`, `MfskQ65Params` | one Q65 decoder: `pileup`, `max_drift`, `fading_*`, the AP hint in `MfskExtras`; `eme_delay` and averaging in `MfskParams::flags`; the AP list from the QSO context; contest callers via `mfsk_decoder_set_q65_callers` |
| `mfsk_callsign_hash_table_*`, the `MfskCallsignHashTable*` argument | gone; every decoder owns its table, `mfsk_decoder_add_callsign` seeds it |
| `mfsk_mode_defaults`, `MfskDecodeDefaults` | gone; `mfsk_params_init` writes the defaults |
| `mfsk_unpack77(session, …)` | `mfsk_unpack77(…)` and `mfsk_decoder_unpack77(dec, …)` |
| `mfsk_iq_add_channel(rx, dial, mode, &ch)` | `mfsk_iq_add_channel(rx, dial, mode, params, extras, &ch)` (NULL, NULL for the old behaviour); `mfsk_iq_channel_decoder`, `mfsk_iq_channel_state` are new |
| `mfsk_iq_set_time_anchor(rx, utc_ns_at_sample_0)` | `mfsk_iq_set_time(rx, utc_ns, at_sample, &change)`, repeatable |
| `mfsk_iq_retune(rx, hz)` failing with `INVALID_ARG` | `mfsk_iq_retune(rx, hz, &paused, &resumed)` pauses a channel that no longer fits |
| `MFSK_CAP_DECODE_HANDLE` as "a session opens" | "the 77-bit slot family"; every slot mode opens a decoder |
| FT8's `previous_cycle` "not exposed" (#496) | `MfskExtras::a7` with the `period` argument |

The checks that still matter: set `size` on every struct (or `_init` it), call
`mfsk_params_init` before touching a block, and read `mfsk_decoder_last_error`
rather than the global after a decoder call.

---

## 4. Kotlin / Android

`bindings/kotlin/` is a maintained binding, built and run on a desktop
JVM by CI on every source change. It replaces the old
`mfsk-ffi/examples/kotlin_jni/` scaffold, which was written against the
pre-v2 ABI, marshalled results as pipe-separated strings, and was never
built by anything.

```kotlin
import io.github.mfskcore.*

// Ask the build what it has rather than hardcoding a list.
val ft8 = Mfsk.modes().first { Mfsk.modeName(it) == "FT8" }

// On Android, call this once before the first decode — see below.
Mfsk.configureRuntime(threads = 2)

MfskDecoder.open(ft8).use { dec ->
    for (r in dec.decode(pcm, period = periodIndex)) {
        Log.i("ft8", "${r.freqHz} Hz  ${r.snrDb} dB  ${r.text}")
    }
}
```

**Shape.** `Mfsk` holds introspection and transmit; `MfskDecoder` is
the one decode handle for every slot mode (§2.1) and is `AutoCloseable`, so
`.use { }` releases it. `MfskDecode` is a `data class` — a value, not a
handle, because the ABI writes rows into memory the caller owns. There is
nothing to free and nothing that can outlive a decoder.

**`Mfsk.configureRuntime` is the Android-specific part.** Without it
the decode runs on rayon's global pool, whose threads are plain
pthreads the VM has never attached — so nothing running on one can
touch a `JNIEnv`, and a decode callback from a worker thread is not
merely discouraged but illegal. The shim's thread hooks call
`AttachCurrentThreadAsDaemon` and `DetachCurrentThread`, which is what
makes that legal. It also takes the pool off `num_cpus` × 2 MiB stacks
that never join.

**`AsDaemon` rather than the plain attach is load-bearing**, and it is
the one thing a consumer writing their own shim is most likely to get
wrong: a non-daemon attached thread keeps the JVM alive, and rayon's
workers are never joined, so the ordinary `AttachCurrentThread` hangs
the process on exit after everything has otherwise succeeded.

**A decoder is single-threaded.** It owns a callsign hash table it
mutates on every decode. One per thread; concurrent decodes on separate
decoders are supported.

**Parameters are two data classes.** `MfskParams` is the parameter block
(§2.3) and `MfskExtras` the library's options (§2.3.1). Start from
`Mfsk.defaultParams(mode)` and `copy` what differs — `MfskParams` has no
constructor default for the band, for the reason the ABI has an init call at
all (zeroing is not the mode's defaults; a zero band decodes nothing).
`MfskExtras()` is what `mfsk_extras_init` writes, every option unset:

```kotlin
val p = Mfsk.defaultParams(ft8).copy(
    rxFreqHz = 1500f,
    txFreqHz = 1500f,                       // FT8's nftx; needs CAP_TX_FREQ
    station = MfskStation("JL1NIE", "PM95"),
    qso = MfskQso("K1JT", "FN20", MfskQsoProgress.REPLYING),
)
val e = MfskExtras(
    a7 = true,                              // FT8's a7; needs a period on each decode
    apHint = MfskApHint("K1JT", "HA0DU"),   // the message's fields in order
)
MfskDecoder.open(ft8, p, e).use { dec -> dec.decode(pcm, period) }
```

`rxFreqHz`, `tolHz` and `txFreqHz` are nullable (NaN in C, since 0 Hz is a
frequency). `depth` is `MfskDepth`, `averaging` / `deepSearch` / `emeDelay`
the `flags`, `ap` an `MfskApMode`, `contest` an `MfskContest`. Among the
extras, `strategy` is `MfskStrategy.SinglePass`, `.SicRounds(n)` or
`.SicEarly`, `strictness` an `MfskStrictness`, `noiseBlanker` is
`MfskNoiseBlanker.Percent(n)` or `.Sweep(step, toleranceHz)` (FST4), and
`fading = MfskQ65Fading(b90Ts)` with `pileup` and `maxDrift` are Q65's. As at
the C level, an option the mode does not have **fails when the extras are
applied** — `MfskDecoder.open` or `setExtras` — with an
`MfskUnsupportedException` naming the field, rather than being dropped; a
value out of range is `MfskInvalidArgException`, a mode not in the build
`MfskUnknownModeException`. `dec.setParams(…)` and `dec.setExtras(…)` change
the block between periods, keeping the state; `dec.clear()` forgets it, and
`dec.addCallsign("JL1NIE")` seeds the hash table. The parameters cross JNI as
three flat arrays per struct whose layout is documented at `read_params` and
`read_extras` in `mfsk_jni.c`; the JVM test sees every slot by the refusal
that names it.

**Decoding.** `dec.decode(pcm, period = …)` takes `ShortArray` or `FloatArray`
(any level), `sampleRate` other than 12 000 is resampled, and `period` is the
UTC-grid index or null. `dec.copyInfo(i)` is the FEC bits of row `i`. A
stream cut on the UTC grid decodes without copying the slot out:

```kotlin
MfskStream.open(ft8).use { s ->
    s.push(chunk)                                   // any size
    s.setTime(utcNs, atSample = s.position)         // as often as a reading arrives
    dec.decodeStream(s)?.let { r -> show(r.period, r.slotStartUtcNs, r.rows) }  // null: no slot yet
}
```

`MfskStream.takeSlot()` copies the slot out instead, `slotReady` and `dropped`
mirror the C calls, and `setTime` returns an `MfskClockChange`
(`FIRST`, `SLEWED`, `STEPPED`).

**Q65 is the same decoder.** Open a Q65 mode and set `MfskParams.emeDelay` /
`averaging`, and `MfskExtras.pileup`, `maxDrift`, `fading` — there is no
`decodeQ65` any more. Rows report `copiedLastTx` (Pileup's `#`), and
`Mfsk.synthesizeQ65(…, copiedLastTx)` sends one. `MfskQ65History` (`q65_hist`:
`push`, `record(rows)`, `lookup(rxFreqHz)`) and `MfskQ65Callers` (`q65_hist2`:
`record(freqHz, text, now)`, `expire(now)`, `remove(call)`, `callers`) are
`AutoCloseable` handles you own; `dec.setQ65Callers(callers)` hands the
contest list to a decoder.

**`dec.setBudget { … }`, `dec.lastBudget`** are the budget (§2.2; every mode
with a decoder, `lastBudget.rowsSubtracted` included). The predicate crosses JNI once per candidate, so keep it to
a `System.nanoTime()` comparison against a captured deadline — anything heavier
belongs behind a boolean the JVM side already computed.

**`dec.decodePrefix(pcm, period)`** is the early decode (§2.2, #572): call it as
audio arrives with everything of the period so far. FT8 returns checkpoint A's
rows at 141 696 samples, with `stage == MfskStage.EARLY`, and the complete set
at the whole period. Over a stream: `stream.setPrefixPoints(dec.prefixPoints)`,
then `dec.decodeStream(stream)` on every ready slot (`stream.slotIsWhole`
tells them apart). Over IQ it is on by default: `MfskIqDecode.stage` says
which rows came early, and `rx.setEarly(ch, false)` turns it off (#601).

**Rows** are `MfskDecode`. `syncScore`, `syncCv` and `hardErrors` are
nullable, null where the mode reports none (the C row's clear
`MFSK_DECODE_FLAG_HAS_*` bit); `key` is the packed message key as hex, with
`keyBits`; `delivery` is the C row's, null for `-1`. `MfskIqDecode` carries
the same detail.

**`dec.onDecode { row -> … }`** delivers rows as they are found, on top of the
list `decode` returns — for a UI that wants something on screen before a long
slot finishes (`decode(…, onRow = …)` does it for one call). The listener is
called from rayon workers, so it must be safe concurrently, and an Android one
that touches views has to post to the main looper. It does **not** require
`configureRuntime` first: the shim attaches the worker itself (as a
daemon) if the VM has never seen it, and takes the listener's method ID
from the *interface* rather than from a lambda's spun class. An
exception it throws is printed and cleared — a rayon worker has nowhere
to propagate one — and the decode continues. `dec.deliveryIsExact` says
whether those rows are exactly the returned ones, in order; pair a streamed
row with its returned form by `delivery` either way.

**IQ** is `MfskIqReceiver` (§2.8.2): `MfskIqReceiver.open(sampleRate, centerHz,
format, iqSwap, channelizer)`, `addChannel(dialHz, mode, params, extras)`
returning the channel id, `push(bytes)`, `poll()` returning the
`MfskIqDecode`s, `setTime`, `retune` (returns the paused and resumed counts),
`gap`, and `channelDecoder(ch)` for a **borrowed** `MfskDecoder` whose
`setParams`, `setExtras` and `addCallsign` reconfigure a live channel, and
whose `onDecode` and `setBudget` see and bound the decodes `push` runs, early
rows included (the same object each time; the receiver takes the listener off
before it frees the channel). Call
`push` off the UI thread.

**JTTY** is `MfskJttyReceiver` (§2.8.1): `MfskJttyReceiver.open(sampleRate,
MfskJttyParams())` and then `for (u in rx.push(chunk)) …` — `push` returns the
updates it produced (one per message, latest text; `id` is stable), `finish()`
returns the last incomplete ones, `close()` releases the handle. Call `push`
off the UI thread. `Mfsk.CAP_STREAM_RECEIVER` is its capability bit; the JVM
test feeds the vendored upstream recording in 4096-sample chunks (and again
through the 24 kHz resampler) and expects the recording's message. Transmit is
`MfskJtty.tones(text, profile)`, `MfskJtty.synthesize(tones)` and `MfskJtty.encode(text)`.

**The shim is C, not Rust-with-`jni`, on purpose.** It `#include`s the
generated `mfsk.h`, so building it is another compiler reading that
header as a real translation unit. That has already earned its keep:
writing it found `MFSK_DECODE_FLAG_HASH_RESOLVED` absent from the
header, because the constant lived in a dependency cbindgen cannot emit
from. A Rust shim would link against the crate and see none of that.

Build and test: `bindings/kotlin/build.sh` (needs `JAVA_HOME` and
`kotlinc`). For Android, build `libmfsk.so` with
`cargo ndk -t arm64-v8a build -p mfsk-ffi --release --no-default-features
--features mobile` and compile the shim with the NDK's clang against
the same header; `.cargo/config.toml` already carries the 16 KB
page-size link flag every Android 15 device needs.

---

## 5. Swift / Apple

`bindings/swift/` is a SwiftPM package over the same ABI — not a
scaffold to copy, a package to depend on. Its module map includes
`mfsk-ffi/include/mfsk.h` **in place**, so it follows the header rather
than carrying a copy.

**Unbuilt since the single-decoder rewrite.** The package was moved onto
`mfsk_decoder_*` without a Swift toolchain on hand; nothing here has been
compiled or run on Apple hardware, and the 98 XCTest cases (counted with
`grep -rc 'func test' bindings/swift/Tests`) are written, not passing. Run
`bindings/swift/scripts/test.sh` on a Mac before relying on any line below.

```swift
import MfskCore

let slot = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JL1NIE", report: "PM95",
                                       frequencyHz: 1500)
let decoder = try Decoder(mode: .ft8)
for row in try decoder.decode(slot) {
    print(row.frequencyHz, row.snrDB, row.text)
}
```

* `Mode` / `ModeInfo` / `Capabilities` wrap the introspection family,
  so a picker is populated from the build rather than a hardcoded list.
* `Decoder` is the one decode handle (§2.1) for every slot mode, including
  WSPR, JT9, JT65 and the ten Q65 sub-modes; `Decoder(mode:params:extras:)`.
  `DecodeParams` is the parameter block (`try DecodeParams(mode: .ft8)` gives
  the mode's defaults, then `bandHz`, `rxFrequencyHz`, `depth`, `ap`, `station`,
  `qso`, …) and `Extras` the library's options (`strategy`, `apHint`, `a7`,
  `sniperHalfWidthHz`, `noiseBlanker`, `pileup`, `maxDrift`, `fading`, …).
  An option the mode lacks throws `MfskError` with code `.unsupported`
  at `init` / `setExtras`, naming it. `setParams`, `setExtras`, `clear()` and
  `addCallsign(_:)` act between periods; `decode(_:sampleRate:period:handler:)`
  takes `[Int16]` or `[Float]` and a `period` (nil for a lone recording).
* `CaptureStream` is the capture holder that cuts slots on the UTC grid
  (`setTime(utcNanoseconds:)`, `position`, `isSlotReady`, `droppedSlots`,
  `takeSlot()`), and `decoder.decode(stream)` is the fused decode that avoids
  copying a slot out and back in; it returns nil while no slot is ready.
* `decoder.setBudget { … }` bounds the search with a predicate the
  caller polls a clock in — the library reads none — and
  `decoder.lastBudget` says what the cut left undone, including how
  good the best skipped candidate was, and `rowsSubtracted`. Every mode with
  a decoder has `Capabilities.budget`.
* `Decode.syncScore`, `syncCV` and `hardErrors` are optionals, nil where the
  mode reports none; `key` / `keyBits` are the message key and `delivery`
  pairs a row given to `onDecode` with its returned form
  (`decoder.deliveryIsExact` says whether the two are the same list).
  `IQDecode` carries the same detail.
* `decoder.decodePrefix(pcm, period:)` is the early decode (§2.2, #572):
  every sample of the period so far. FT8 returns checkpoint A's rows at
  141 696 samples (`stage == .early`) and the complete set at the whole
  period. Over a `CaptureStream`: `try stream.setPrefixPoints(decoder.prefixPoints)`,
  then `decoder.decode(stream)` on every ready slot (`stream.isSlotWhole`).
  Over IQ it is on by default: `IQDecode.stage`, and
  `rx.setEarly(false, forChannel:)` turns it off (#601).
* `decoder.onDecode { row in … }` streams rows as they are found,
  alongside the array the call returns. On a `desktop` build the
  closure runs on rayon workers, possibly concurrently; on `mobile` it
  is one thread in candidate order. The closure is retained until
  replaced or the decoder is released, and cleared before the handle
  closes.
* Failures throw `MfskError`, which carries both the status code and
  the reason string — reading the handle's own error slot first and the
  thread-local global second.
* **Q65 is the same decoder.** Pileup, Max Drift and the fast-fading metric
  are `Extras` (`pileup`, `maxDrift`, `fading = Extras.Fading(…)`), the EME delay
  and averaging are `DecodeParams.emeDelay` / `averaging`, and
  `Decode.copiedLastTx` is Pileup's `#`. `Q65` is the transmit side:
  `Q65.encode(subMode:…)`, `Q65.encode(…, copiedLastTx:)`, with `Q65SubMode` (**its
  own numbering**, where `a15` is 6, bridged to `Mode` by `.mode`) and
  `Q65FadingModel`. `Q65History` (`q65_hist`) and `Q65Callers` (`q65_hist2`) are
  the two lists WSJT-X keeps, as classes you own; `decoder.setQ65Callers(_:)` hands
  the contest list over.
* An AP hint's fields (`Extras.APHint`) are the **message's fields in order** —
  `call1` is `"CQ"` for a CQ, not the transmitting station — and they lock
  message bits rather than steering a search, so a hint in the wrong
  order is a wrong hint. A decode still tries each candidate without AP
  first (since #555, as `jt9 -3` does), so a clean signal decodes either
  way; a weak one that needed the hint is lost.
* `Message.text(resolvedBy: decoder)` renders a packed message with the
  decoder's own hash table.
* **`IQReceiver`** (§2.8.2): `IQReceiver(sampleRate:centerHz:format:iqSwap:channelizer:)`,
  `addChannel(dialHz:mode:params:extras:)`, `push(_:)`, `poll()` / `drain()`
  returning `IQDecode`s, `setTime(utcNanoseconds:atSample:)`, `retune`, `gap`,
  `state(ofChannel:)`, and `decoder(forChannel:)` for a **borrowed** `Decoder`
  that reconfigures a live channel and must not be asked to decode.

**JTTY** is `JttyReceiver` (§2.8.1): `try JttyReceiver(sampleRate:params:)`,
then `try receiver.push(samples)` returns the `[JttyUpdate]` it produced (one per
message, latest text; `id` is stable, `isComplete`, `frequencyHz`,
`startSeconds`), `finish()` the last incomplete ones. `Mode.jtty` reports
`Capabilities.streamReceiver` and not `.decodeHandle`; `JttyParams` carries the
receive frequency, tolerance, sync floor, band and whether it subtracts. One
thread at a time, and off the main actor — `push` decodes before it returns.
`JttyReceiverTests` feeds it the vendored upstream recording (located through
`#filePath`) and a loopback of its own: `Jtty.tones(for:profile:)` (the text packer),
`Jtty.synthesise(_:)` and `Jtty.audio(for:)` turn text into audio.

`bindings/swift/scripts/test.sh` builds `libmfsk` and runs the tests;
`bindings/swift/README.md` covers linking from a real app, including why the
`mobile` feature set is the one an iOS build wants.
`bindings/swift/scripts/build-xcframework.sh` packages the iOS device and
simulator builds as `target/xcframework/Mfsk.xcframework` (module `CMfsk`),
link-checking both slices before it reports success.

CI runs that same script on `macos-latest` (`Swift binding (macOS) +
iOS build`), which is also where `aarch64-apple-ios` is cross-compiled:
XCTest ships with Xcode rather than with the Command Line Tools, and
the iOS SDK is Xcode's as well, so one runner covers both. Locally the
script points `DEVELOPER_DIR` at Xcode when `xcode-select` is on the
CLT.
