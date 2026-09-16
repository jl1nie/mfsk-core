# mfsk-core — C, Kotlin and Swift bindings

> **日本語版:** [BINDINGS.ja.md](BINDINGS.ja.md)

This document covers consuming mfsk-core from outside Rust. For the
Rust host API see [`LIBRARY.md`](LIBRARY.md); for `no_std` / embedded
targets see [`EMBEDDED.md`](EMBEDDED.md).

| binding | lives in | built and tested by |
|---|---|---|
| **C / C++** | `mfsk-ffi/`, header `mfsk-ffi/include/mfsk.h` | CI `ffi` job — Rust tests under both feature sets plus `examples/cpp_smoke/`, a real C++ driver including a multi-thread stress |
| **Kotlin / Android** | `bindings/kotlin/` (C shim + `Mfsk.kt`) | CI `kotlin` job, on a desktop JVM |
| **Swift / Apple** | `bindings/swift/` (SwiftPM package `MfskCore`) | CI `swift` job on `macos-latest` — 68 XCTest cases, plus the `aarch64-apple-ios` cross-build |

All three sit on the same C ABI. `mfsk.h` is cbindgen-generated and
committed, and its doc comments are the authoritative per-symbol
reference — this document is the map, not a replacement for it.

## Contents

- [1. Artefacts and linking](#1-artefacts-and-linking)
- [2. The C ABI](#2-the-c-abi)
  - [2.1 Shape: sessions, and memory you own](#21-shape-sessions-and-memory-you-own)
  - [2.2 Decoding a slot](#22-decoding-a-slot)
  - [2.3 `MfskDecodeParams` — the search](#23-mfskdecodeparams--the-search)
  - [2.4 `MfskDecode` — one result row](#24-mfskdecode--one-result-row)
  - [2.5 Streaming capture](#25-streaming-capture)
  - [2.6 Transmit](#26-transmit)
  - [2.7 Introspection](#27-introspection)
  - [2.8 Modes with their own entry point](#28-modes-with-their-own-entry-point)
  - [2.9 Messages](#29-messages)
  - [2.10 Threads and the runtime](#210-threads-and-the-runtime)
  - [2.11 Errors and memory rules](#211-errors-and-memory-rules)
  - [2.12 Symbol index](#212-symbol-index)
- [3. Porting from the pre-v2 ABI](#3-porting-from-the-pre-v2-abi)
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

### 2.1 Shape: sessions, and memory you own

Two rules cover most of the surface:

1. **A session is the decode handle.** `mfsk_session_open` →
   configure → decode one or more slots → `mfsk_session_close`. It owns
   a callsign hash table and, optionally, the previous slot's results
   and FFT — the things that only mean something across more than one
   call.
2. **Nothing crosses the boundary as an allocation.** Result rows,
   synthesised audio and unpacked text all go into buffers you sized
   and own. There is no pointer to free, which removes the category
   that makes wrappers leak when an exception unwinds between a call
   and its free.

The only handles are `MfskDecodeSession*`, `MfskStream*` and
`MfskCallsignHashTable*`, each with its own `_open`/`_new` and
`_close`/`_free`. They are distinct incomplete types, so passing one
where another is expected is a C type error rather than undefined
behaviour.

### 2.2 Decoding a slot

The minimal path — this is the shape `mfsk-ffi/examples/cpp_smoke/main.cpp`
runs in CI:

```c
#include "mfsk.h"

MfskStatus st = MFSK_STATUS_INTERNAL;
MfskDecodeSession* s = mfsk_session_open(MFSK_MODE_FT8, NULL, &st);
if (s == NULL) { /* mfsk_last_error() says why */ }

MfskDecode rows[16];
size_t n = 0;
if (mfsk_session_decode_i16(s, pcm, n_pcm, 12000, NULL,
                            rows, 16, &n) == MFSK_STATUS_OK) {
    for (size_t i = 0; i < n; ++i) {
        printf("%.1f Hz  %.0f dB  %s\n",
               rows[i].freq_hz, rows[i].snr_db, rows[i].text);
    }
}
mfsk_session_close(s);
```

`NULL` for the params means the mode's own defaults. `out_cap` bounds
how many rows you are willing to receive; `out_len` says how many were
written.

```c
MfskDecodeSession *mfsk_session_open(uint32_t mode, const MfskDecodeParams *params,
                                     MfskStatus *out_status);
void               mfsk_session_close(MfskDecodeSession *s);

MfskStatus mfsk_session_decode_i16(MfskDecodeSession *s, const int16_t *samples,
                                   size_t n_samples, uint32_t sample_rate,
                                   const MfskDecodeParams *params,
                                   MfskDecode *out, size_t out_cap, size_t *out_len);
MfskStatus mfsk_session_decode_f32(MfskDecodeSession *s, const float *samples, ...);
```

Per-session strategy, set once and applied to every later decode:

| call | effect |
|---|---|
| `mfsk_session_set_on_decode(s, cb, user)` | deliver rows as they are found, on top of the array the call returns |
| `mfsk_session_set_budget(s, check, user)` | poll a caller-supplied predicate; stop when it returns `false` |
| `mfsk_session_last_budget(s, &report)` | what the cut left undone — candidates skipped, stages run, and how good the best skipped candidate was |
| `mfsk_session_keep_known(s, true)` | carry this decode's results into the next as known signals |
| `mfsk_session_known_count(s)` | how many are currently carried |
| `mfsk_session_keep_fft_cache(s, true)` | reuse the slot transform for a second pass over the same audio |
| `mfsk_session_add_callsign(s, "JL1NIE")` | seed the hash table so `<...>` references resolve |
| `mfsk_session_copy_info(s, i, out, cap, &len)` | the FEC information bits behind row `i` |

The budget predicate is polled per candidate and **the library reads no
clock of its own** — the deadline is whatever your predicate compares
against. That is what keeps it usable from wasm and from a phone that
was backgrounded mid-slot.

### 2.3 `MfskDecodeParams` — the search

Zero the struct, set `size`, then let the library fill in the mode's
defaults before you override anything:

```c
MfskDecodeParams p;
memset(&p, 0, sizeof p);
p.size = sizeof p;
mfsk_decode_params_init(MFSK_MODE_FT8, &p);
p.freq_max_hz = 2600.0f;
```

| field | meaning |
|---|---|
| `freq_min_hz` / `freq_max_hz` | search band edges |
| `sync_min` | sync threshold — **not comparable across modes**, see `MfskDecodeDefaults::sync_scale` |
| `max_cand` | candidate budget |
| `depth` | `MfskDecodeDepth` — cost/recall rung |
| `strictness` | `MfskStrictness` — accept/reject threshold profile |
| `eq_mode` | `MfskEqMode`. A property of the *input audio* — it flattens a passband an analogue filter has tilted — not of the search |
| `freq_hint_hz` | prioritise candidates near this frequency; `NaN` (what `_init` writes) means unset |
| `sic_rounds` | successive-interference-cancellation rounds, 0 for none. Requires `MFSK_CAP_SIC_ROUNDS` |
| `sic_early` | checkpoint-emulation early decode. Requires `MFSK_CAP_SIC_EARLY` |
| `has_ap_hint`, `ap_call1`, `ap_call2`, `ap_grid` | a-priori hint. Requires `MFSK_CAP_AP_WIDEBAND` (or `_AP_NARROW` on a narrow-band call) |
| `search_hz` | half-width of a narrow-band search; 0 for the mode's default. Only meaningful with `MFSK_CAP_SNIPER` |

**The AP fields are the message's fields in order** — `ap_call1` is
`"CQ"` for a CQ, not the transmitting station. They lock message bits
rather than steering a search, so the wrong order removes the decode
instead of costing a fraction of a dB.

### 2.4 `MfskDecode` — one result row

Flat, fixed-size, written into your array. `text` is an inline
`char[MFSK_DECODE_TEXT_LEN]`, NUL-terminated.

| field | meaning |
|---|---|
| `size` | `sizeof(MfskDecode)` as the caller understands it |
| `mode` | the **concrete sub-mode**, not the family — all five FST4 periods report distinctly |
| `text` | decoded message |
| `freq_hz`, `dt_sec`, `snr_db` | carrier, time offset from the slot's `dt = 0`, SNR in a 2500 Hz reference bandwidth |
| `sync_score` | sync correlation for this decode |
| `sync_cv` | coefficient of variation of the per-block sync powers — near 0 on a stable channel, elevated under QSB. The only fading indicator the row carries |
| `hard_errors` | hard-decision errors the FEC corrected |
| `info_bits` | width of the FEC information block, 91 (CRC-14) or 101 (CRC-24) |
| `pass` | which decode pass produced the row. **Protocol-private** — diagnostics, not logic |
| `flags` | bit 0 = `MFSK_DECODE_FLAG_HASH_RESOLVED`, the text needed the hash table to resolve a `<...>` reference |

### 2.5 Streaming capture

A one-slot ring you push audio into, sized from the mode's own
`slot_samples_12k` — so FST4-300's 3.6 M-sample slot works the same way
FT4's 90 000-sample one does.

```c
MfskStream *mfsk_stream_open(uint32_t mode, uint32_t sample_rate, MfskStatus *out);
MfskStatus  mfsk_stream_push_i16(MfskStream *s, const int16_t *samples, size_t n);
MfskStatus  mfsk_stream_push_f32(MfskStream *s, const float *samples, size_t n);
void        mfsk_stream_set_epoch(MfskStream *s, double utc_seconds_of_next_sample);
bool        mfsk_stream_slot_ready(const MfskStream *s);
size_t      mfsk_stream_buffered(const MfskStream *s);
size_t      mfsk_stream_take_slot_i16(MfskStream *s, int16_t *out, size_t cap,
                                      double *out_slot_start_utc);
void        mfsk_stream_clear(MfskStream *s);
void        mfsk_stream_close(MfskStream *s);

/* fused: decode straight out of the ring */
MfskStatus  mfsk_session_decode_stream(MfskDecodeSession *s, MfskStream *stream,
                                       const MfskDecodeParams *params,
                                       MfskDecode *out, size_t out_cap, size_t *out_len,
                                       double *out_slot_start_utc);
```

**No `Instant`, no `SystemTime`, no clock of any kind.** The host says
what UTC second the next sample belongs to and the grid does
arithmetic. Without an epoch the grid free-runs from the first sample,
which is exactly right for replaying a recording.

Prefer `mfsk_session_decode_stream` over take-then-decode: taking
FST4-300's slot out and handing it back in moves 7 MB for nothing.

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
MfskStatus  mfsk_mode_defaults(uint32_t mode, MfskDecodeDefaults *out);
uint32_t    mfsk_abi_version(void);
uint32_t    mfsk_version(void);
```

**`MfskMode` addresses every mode, and its discriminants are ABI.** One
per registry entry plus MSK144, assigned once and never reordered —
deliberately *not* registry indices, because registry membership is
feature-gated and a build without `q65` would shift every index after
it. `mfsk_mode_count` / `mfsk_mode_at` say which of them this
particular build has.

**Capabilities are published, not inferred.**
`MFSK_CAP_DECODE_HANDLE` is the load-bearing one: it says whether
`mfsk_session_decode_i16` and friends apply at all. Q65 takes a nominal
start sample and a time tolerance; WSPR / JT9 / JT65 have no builder.
They are not lesser, they are shaped differently, and that is a bit a
caller can read rather than a fact it has to know.

| bit | constant | meaning |
|---|---|---|
| 0 | `MFSK_CAP_DECODE_HANDLE` | the session decode calls apply |
| 1 | `MFSK_CAP_SNIPER` | narrow-band single-target search. **FT8 only, by design** |
| 2 | `MFSK_CAP_AP_NARROW` | AP hint on a targeted search |
| 3 | `MFSK_CAP_AP_WIDEBAND` | AP hint on the wide-band search |
| 4 | `MFSK_CAP_SIC_ROUNDS` | flat successive-interference cancellation |
| 5 | `MFSK_CAP_SIC_EARLY` | checkpoint-emulation early decode. FT8 only |
| 6 | `MFSK_CAP_OSD` | the OSD *switch* is honoured. Absent means "cannot be turned off", not "does not have it" |
| 7 | `MFSK_CAP_EQ_MODE` | equalisation reaches the decoder |
| 8 | `MFSK_CAP_STRICTNESS` | the strictness profile is honoured rather than accepted and dropped |
| 9 | `MFSK_CAP_BUDGET` | a caller-supplied budget predicate is polled |

The bits mirror `mfsk_core::registry::caps`, which
`mfsk-core/tests/registry_caps.rs` ties to the trait impls in both
directions — naming a protocol that lacks a trait is a *compile* error
there, and implementing one without setting the bit is a runtime
failure. `mfsk-ffi/tests/mode_introspection.rs` closes the last link by
comparing each `MFSK_CAP_*` against the registry constant it mirrors.
That chain exists because a hand-written capability table lies within
two releases.

**`mfsk_mode_defaults` removes the ABI's worst trap.** Defaults are
data, and `MfskDecodeDefaults::sync_scale` says whether two modes'
numbers are even comparable:

```c
MfskDecodeDefaults d = {0};
d.size = sizeof d;
mfsk_mode_defaults(MFSK_MODE_FT4, &d);
/* d.sync_min == 1.2, d.sync_scale == MFSK_SYNC_SCALE_BASELINE_NORMALISED */
```

FT4's spectrum is divided by a fitted baseline before scoring, so noise
sits at ~1.0 **by construction** and WSJT-X's own 1.2
(`ft4_decode.f90:195`) is a floor rather than a preference. FT8's and
FST4's are absolute Costas scores. Copying one across modes is wrong,
and before this field nothing said so.

**`MfskModeInfo::decode_fft1_size` is the field to read before
budgeting.** It is the forward FFT the decoder takes over the whole
slot: FT4 92 160 points, FST4-300 **4 194 304** — a factor of 45 that
no other field hints at, and the reason "one call shape for every mode"
is wrong as a memory story on a phone.

**Size versioning.** `MfskModeInfo`, `MfskDecodeDefaults`,
`MfskDecodeParams` and `MfskDecode` all lead with `size`. Set it to
your `sizeof` (or zero the struct and the library fills it in); a
library newer than your header writes only the prefix you declared and
rewrites `size` to what it actually wrote.

`mfsk_abi_version()` is separate from `mfsk_version()` on purpose: the
crate version moves for reasons that have nothing to do with the
boundary.

### 2.8 Modes with their own entry point

Modes without `MFSK_CAP_DECODE_HANDLE` are addressed directly:

```c
MfskStatus mfsk_wspr_decode(const int16_t *samples, size_t n, uint32_t rate,
                            MfskDecode *out, size_t cap, size_t *out_len);
MfskStatus mfsk_jt9_decode_at (const int16_t *samples, size_t n, uint32_t rate,
                               float freq_hz, MfskDecode *out, size_t cap, size_t *out_len);
MfskStatus mfsk_jt65_decode_at(/* same shape as jt9 */);
```

Q65 carries a family of four, differing in what they are given to work
with — all take a `submode` and an optional `MfskCallsignHashTable*`:

| call | adds |
|---|---|
| `mfsk_q65_decode` | — |
| `mfsk_q65_decode_with_ap` | `ap_call1`, `ap_call2`, `ap_grid`, `ap_report` |
| `mfsk_q65_decode_fading` | `b90_ts`, `fading_model` (`MfskQ65FadingModel`) |
| `mfsk_q65_decode_with_ap_list` | `my_call`, `his_call`, `his_grid` — the QSO-state hypothesis list |

`MfskQ65SubMode` has **its own numbering**, where `a15` is 6; bridge to
`MfskMode` rather than assuming they agree.

The hash table is the one handle the caller owns rather than the
session: `mfsk_callsign_hash_table_new` / `_insert` / `_free`.

### 2.9 Messages

```c
MfskStatus mfsk_pack77(const char *call1, const char *call2, const char *report,
                       uint8_t *out_message77);
MfskStatus mfsk_pack77_type1(const char *call1, const char *call2, const char *grid,
                             uint8_t *out_message77);
MfskStatus mfsk_pack77_type4(const char *nonstd_call, const char *std_call,
                             const char *report, bool is_cq, uint8_t *out_message77);
MfskStatus mfsk_pack77_free_text(const char *text, uint8_t *out_message77);
MfskStatus mfsk_unpack77(const MfskDecodeSession *session, const uint8_t *message77,
                         char *out, size_t cap, size_t *out_len);
```

`out_message77` is a caller-owned 77-byte buffer in every case; none of
these allocate. `mfsk_pack77_free_text` packs **up to 13 characters of
free text** — despite the name it frees nothing. `mfsk_unpack77` takes
a session so `<...>` hash references resolve against its table; pass
`NULL` if you have none.

### 2.10 Threads and the runtime

```c
MfskStatus mfsk_runtime_configure(const MfskRuntimeConfig *cfg);
uint32_t   mfsk_runtime_thread_count(void);
```

* **A session is single-threaded.** It mutates its hash table on every
  decode. One per concurrent thread; concurrent decodes on separate
  sessions are supported and cheap.
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

1. **Handles**: `mfsk_session_open` / `mfsk_session_close`,
   `mfsk_stream_open` / `mfsk_stream_close`,
   `mfsk_callsign_hash_table_new` / `_free`. Close and free are
   idempotent on `NULL`.
2. **Result rows, audio and text** go into caller-owned buffers.
   Nothing returned needs freeing. The two functions returning a
   `const char*` — `mfsk_last_error` and `mfsk_session_last_error` —
   hand back a borrowed pointer, not an allocation; so does
   `mfsk_mode_name`, whose string is static.
3. **Errors**: on a non-`MFSK_STATUS_OK` return, call
   `mfsk_session_last_error(s)` for a session call, or
   `mfsk_last_error()` for a free function, on the **same thread**. The
   returned pointer is valid until the next fallible call on that
   thread.

`MfskStatus`: `OK = 0`, `NULL_POINTER = -1`, `INVALID_ARG = -2`,
`UNKNOWN_PROTOCOL = -3` (not in this build), `DECODE_FAILED = -4`,
`INTERNAL = -5` (always a bug), `UNSUPPORTED = -6` (the mode is here
but does not offer what was asked).

### 2.12 Symbol index

63 exported functions, grouped:

| group | symbols |
|---|---|
| session (14) | `mfsk_session_open` `mfsk_session_close` `mfsk_session_decode_i16` `mfsk_session_decode_f32` `mfsk_session_decode_stream` `mfsk_session_set_on_decode` `mfsk_session_set_budget` `mfsk_session_last_budget` `mfsk_session_keep_known` `mfsk_session_known_count` `mfsk_session_keep_fft_cache` `mfsk_session_add_callsign` `mfsk_session_copy_info` `mfsk_session_last_error` |
| streaming (9) | `mfsk_stream_open` `mfsk_stream_close` `mfsk_stream_push_i16` `mfsk_stream_push_f32` `mfsk_stream_buffered` `mfsk_stream_set_epoch` `mfsk_stream_slot_ready` `mfsk_stream_take_slot_i16` `mfsk_stream_clear` |
| introspection (10) | `mfsk_mode_count` `mfsk_mode_at` `mfsk_mode_name` `mfsk_mode_from_name` `mfsk_mode_info` `mfsk_mode_caps` `mfsk_mode_defaults` `mfsk_decode_params_init` `mfsk_abi_version` `mfsk_version` |
| bespoke decode (7) | `mfsk_wspr_decode` `mfsk_jt9_decode_at` `mfsk_jt65_decode_at` `mfsk_q65_decode` `mfsk_q65_decode_with_ap` `mfsk_q65_decode_fading` `mfsk_q65_decode_with_ap_list` |
| transmit (12) | `mfsk_encode_ft8` `mfsk_encode_ft4` `mfsk_encode_fst4s60` `mfsk_encode_wspr` `mfsk_encode_jt9` `mfsk_encode_jt65` `mfsk_encode_q65` `mfsk_message_to_tones` `mfsk_tones_to_i16` `mfsk_tones_to_f32` `mfsk_symbol_count` `mfsk_synth_output_len` |
| messages (5) | `mfsk_pack77` `mfsk_pack77_type1` `mfsk_pack77_type4` `mfsk_pack77_free_text` `mfsk_unpack77` |
| hash table (3) | `mfsk_callsign_hash_table_new` `mfsk_callsign_hash_table_insert` `mfsk_callsign_hash_table_free` |
| runtime (3) | `mfsk_runtime_configure` `mfsk_runtime_thread_count` `mfsk_last_error` |

---

## 3. Porting from the pre-v2 ABI

0.11.0 replaced the C decode surface outright. `mfsk-ffi` is
`publish = false` and its only exercised consumer was the in-repo C++
driver, so the blast radius is smaller than the diff suggests — but a C
consumer porting across will rewrite, not adjust.

| pre-v2 | v2 |
|---|---|
| `MfskProtocol` enum | `MfskMode` — one discriminant per registry entry, so all five FST4 sub-modes are addressable. Discover with `mfsk_mode_count` / `mfsk_mode_at` |
| `mfsk_decoder_new` / `_free` | `mfsk_session_open` / `mfsk_session_close` |
| `MfskDecodeOptions*` + eight `mfsk_decode_options_set_*` | `MfskDecodeParams`, a plain size-versioned struct. Fill via `mfsk_decode_params_init` |
| `MfskResultList` + `mfsk_result_list_free` | `MfskDecode out[]`, an array you own. Nothing to free |
| `MfskSamples` + `mfsk_samples_free` | caller buffers sized with `mfsk_symbol_count` / `mfsk_synth_output_len` |
| `mfsk_decode_{i16,f32}_sniper` | `MfskDecodeParams::search_hz` on the ordinary decode, where `MFSK_CAP_SNIPER` is set (FT8 only) |
| seven heap-returning `mfsk_encode_*` | same names, now writing into `out` / `cap` / `out_len` |
| `mfsk_last_error()` for everything | `mfsk_session_last_error(s)` for session calls; `mfsk_last_error()` still covers free functions |
| hardcoded per-mode geometry | `mfsk_mode_info` / `mfsk_mode_caps` / `mfsk_mode_defaults` |
| `mfsk-ffi-ft8` (FT8-only embedded crate) | retired. Use `mfsk-ffi`, or call `mfsk-core` from a Rust staticlib shim — see [`EMBEDDED.md`](EMBEDDED.md) |

`MfskDecodeSession` is deliberately **not** the same type as the pre-v2
`MfskDecoder`: the two own different Rust values, and a handle whose
meaning depends on which function you pass it to is the failure mode
this redesign exists to end.

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

MfskSession.open(ft8).use { s ->
    for (r in s.decode(pcm, sampleRate = 12_000)) {
        Log.i("ft8", "${r.freqHz} Hz  ${r.snrDb} dB  ${r.text}")
    }
}
```

**Shape.** `Mfsk` holds introspection and transmit; `MfskSession` is
the decode handle and is `AutoCloseable`, so `.use { }` releases it.
`MfskDecode` is a `data class` — a value, not a handle, because the ABI
writes rows into memory the caller owns. There is nothing to free and
nothing that can outlive a session.

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

**A session is single-threaded.** It owns a callsign hash table it
mutates on every decode. One per thread; concurrent decodes on separate
sessions are supported.

**`session.setBudget { … }`, `keepKnown`, `keepFftCache`** are the same
three strategies, per session. The budget predicate crosses JNI once
per candidate, so keep it to a `System.nanoTime()` comparison against a
captured deadline — anything heavier belongs behind a boolean the JVM
side already computed.

**`session.onDecode { row -> … }`** delivers rows as they are found, on
top of the list `decode` returns — for a UI that wants something on
screen before a long slot finishes. The listener is called from rayon
workers, so it must be safe concurrently, and an Android one that
touches views has to post to the main looper. It does **not** require
`configureRuntime` first: the shim attaches the worker itself (as a
daemon) if the VM has never seen it, and takes the listener's method ID
from the *interface* rather than from a lambda's spun class. An
exception it throws is printed and cleared — a rayon worker has nowhere
to propagate one — and the decode continues.

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

```swift
import MfskCore

let slot = try Mode.ft8.synthesiseSlot(call1: "CQ", call2: "JL1NIE", report: "PM95",
                                       frequencyHz: 1500)
let session = try DecodeSession(mode: .ft8)
for row in try session.decode(slot) {
    print(row.frequencyHz, row.snrDB, row.text)
}
```

* `Mode` / `ModeInfo` / `Capabilities` wrap the introspection family,
  so a picker is populated from the build rather than a hardcoded list.
* `DecodeSession` covers every mode with `MFSK_CAP_DECODE_HANDLE`;
  `WSPR`, `JT9` and `JT65` have their own entry points, as they do in C.
* `CaptureStream` is the one-slot capture ring, and
  `session.decode(stream)` is the fused decode that avoids copying a
  slot out and back in.
* `session.setBudget { … }` bounds the search with a predicate the
  caller polls a clock in — the library reads none — and
  `session.lastBudget` says what the cut left undone, including how
  good the best skipped candidate was. `keepKnown(_:)` carries a
  decode's results into the next as known signals; `keepFFTCache(_:)`
  reuses the slot transform for a second pass over the same audio.
* `session.onDecode { row in … }` streams rows as they are found,
  alongside the array the call returns. On a `desktop` build the
  closure runs on rayon workers, possibly concurrently; on `mobile` it
  is one thread in candidate order. The closure is retained until
  replaced or the session is released, and cleared before the handle
  closes.
* Failures throw `MfskError`, which carries both the status code and
  the reason string — reading the handle's own error slot first and the
  thread-local global second, because `mfsk_session_copy_info` takes
  the handle as `const*` and can only write the latter.
* `Q65` carries the whole family — all four decode strategies (plain,
  a-priori, fast-fading, AP-list), `Q65SubMode` (**its own numbering**,
  where `a15` is 6, bridged to `Mode` by `.mode`), `Q65FadingModel`,
  and `CallsignHashTable`, the one handle the caller owns rather than
  the session. The enums reach Swift because `cbindgen.toml` emits them
  into `mfsk.h`; before that a wrapper would have had to hardcode 0…9.
* An AP hint's fields are the **message's fields in order** — `call1`
  is `"CQ"` for a CQ, not the transmitting station — and they lock
  message bits rather than steering a search, so the wrong order
  removes the decode instead of costing a fraction of a dB. Both
  directions are pinned in `Q65Tests`.

`bindings/swift/scripts/test.sh` builds `libmfsk` and runs the 68
tests; `bindings/swift/README.md` covers linking from a real app,
including why the `mobile` feature set is the one an iOS build wants.

CI runs that same script on `macos-latest` (`Swift binding (macOS) +
iOS build`), which is also where `aarch64-apple-ios` is cross-compiled:
XCTest ships with Xcode rather than with the Command Line Tools, and
the iOS SDK is Xcode's as well, so one runner covers both. Locally the
script points `DEVELOPER_DIR` at Xcode when `xcode-select` is on the
CLT.
