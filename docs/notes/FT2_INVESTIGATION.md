# FT2: what the implementations actually transmit (2026-10-04)

Why this exists: issue #437 proposed porting WSJT-X's `lib/ft2/`. Before closing it
(not planned), the sources of every FT2 in circulation were read, because the two
projects that ship "FT2" disagree publicly about whether they interoperate. This
note records what the **source code** shows. It is not a benchmark, and it did not
run any on-air or WAV interoperability test.

Method: source code is the primary evidence. README text, comments, commit messages
and manuals were treated as data to be checked against the code, not as statements
of fact.

## Summary

- **The FT2 on the air today is FT4's transmit chain at half the symbol length, in
  both implementations** (WS, formerly WSJT-X Improved, and Decodium): 4-GFSK, 105
  symbols (87 data + 16 sync + 2 ramp), 288 samples per symbol at 12 kHz (24 ms),
  tone spacing 41.667 Hz, about 2.52 s of signal in a 3.75 s T/R period, four 4×4
  Costas arrays, the FT4 `rvec` scrambler, LDPC(174,91), CRC-14, 77-bit messages.
- **WSJT-X's own `lib/ft2/` is an unrelated 2019 experiment** (2-GFSK, 75 baud,
  144 symbols, LDPC(128,90), CRC-13). No release's mode list includes it, it ships
  unbuilt inside the WS tarballs, and it was deleted from WSJT-X's 3.2 line.
- IU8LMC's published table of "WS parameters" (2 tones, 13.3 ms, 1.92 s, LDPC(128,90),
  CRC 13) describes that unbuilt experiment, not what WS transmits.
- **Not established:** whether the two programs decode each other's signals. The
  source says the waveforms are the same; nothing was run.

## Sources

| what | version | where | identifier |
|---|---|---|---|
| WS 3.2.1 | `ws-3.2.1_260926.tgz` | SourceForge `wsjt-x-improved`, `WS_v3.2.1/Source code/` | sha256 `64733ed64eea189b109974e2cb93c6210b53b191a4c3588eebfa0cd30f73ccab`; inner `ws.tgz` md5 `c37a4c04f5ac9b12322e7fb2616c2d63` |
| WSJT-X Improved 3.1.0 | `wsjtx-3.1.0_improved_PLUS_260522.tgz` | same project, `WSJT-X_v3.1.0/Source code/` | sha256 `f159cde764ababb2c63ee8f307f18f29db0db322e152622cc5b4e2c7575fb5a0`; inner `wsjtx.tgz` md5 `840a3ad23af07d292c8ed052d83c293e` |
| Decodium 4.0 | `iu8lmc/Decodium-4.0-Core-Shannon` | GitHub | `5136ed301d813ea7e3460d7e703db618ea34dcdc` (2026-10-03), 1,460 commits |
| Decodium 3.0 | `iu8lmc/Decodium-3.0-Shannon` | GitHub | head `55aee72` (2026-03-30), 236 commits; first FT2 commit `641e7d6` (2026-02-21) |
| Decodium 3 (early) | `iu8lmc/decodium3` | GitHub | one commit, `0d238c3` (2026-02-21, "Redirect to build releases"): no source |
| WSJT-X | `v3.2.0-rc1` (`567ad29ce`) and `master` (`967c85a61`) | `WSJTX/wsjtx` | fetched 2026-10-04 |
| Decodium manual | PDF v1.0.262 (Appendix A); HTML rev. 2026-07-25 | ft2.it | PDF text extracted with `pypdf` |

The WS tarballs are wrappers: the real source is the inner `wsjtx.tgz` / `ws.tgz`.
SourceForge has no code repository for the project.

Not available: the first WS build with FT2 (260226 / 260228); SourceForge keeps only
the 260522 source of 3.1.0. MSHV and WSJT-Z were not examined.

## Parameters

Line numbers refer to the sources above. "WS" is 3.2.1; 3.1.0 and 3.2.1 show no
difference (`diff`) in `lib/ft4_decode.f90`, `lib/ft4/gen_ft4wave.f90`,
`lib/ft4/genft4.f90`, `lib/ft4/ft4_params.f90`, `Modulator/Modulator.cpp` and
`helper_functions.cpp`.

| | WSJT-X `lib/ft2/` (2019, unbuilt) | WS 3.1.0 / 3.2.1 (what it transmits) | Decodium 3.0, first FT2 commit | Decodium 4.0 |
|---|---|---|---|---|
| modulation | 2-GFSK, h = 0.8 (`ft2_iwave.f90`) | 4-GFSK, h = 1.0, BT 1.0, pulse 3 symbols (`lib/ft4/gen_ft4wave.f90`) | same (`gen_ft2wave.f90`) | same (`Modulator/FtxWaveformGenerator.cpp:1269-1318`) |
| samples per symbol (12 kHz) | 160 | 288 (`widgets/mainwindow.cpp:9055-9056`: `nsps = 4*288` at 48 kHz) | 288 (`lib/ft2/ft2_params.f90`) | 288 (`mainwindow.cpp:17355`, `Modulator/Modulator.cpp:145`) |
| symbol / baud | 13.3 ms / 75 | 24 ms / 41.667 | 24 ms | 24 ms |
| tones, spacing | 2 | 4, 41.667 Hz | 4 | 4 |
| symbols | 144 | 105 = 87 + 16 + 2 (`helper_functions.cpp:7`: `105*288`) | 105 (`NN2`) | 105 (`Modulator.cpp:144`, `src/core/helper_functions.cpp:275`) |
| signal length | 1.92 s | about 2.52 s | same | about 2.52 s |
| T/R period | n/a | 3.75 s (`mainwindow.cpp:12519`, `lib/decoder.f90:1218`) | `NMAX = 45000` (3.75 s) | 3.75 s (`mainwindow.cpp:23708, 25344`) |
| sync | fixed bit pattern `0000 11111111 0000` | four 4×4 Costas arrays (`lib/ft4/genft4.f90:29-32`) | "four 4x4 Costas arrays" | `icos4a..d` (`Modulator/FtxMessageEncoder.cpp:2674-2685`): `{0,1,3,2} {1,0,2,3} {2,3,1,0} {3,2,0,1}` |
| layout | n/a | `s4 d29 s4 d29 s4 d29 s4` (`genft4.f90:81-87`) | same | tones 0-3, 33-36, 66-69, 99-102 |
| bits to tone | n/a | Gray 00,01,11,10 to 0,1,2,3 | same | same (`FtxMessageEncoder.cpp:2664-2670`) |
| scrambler | n/a | `rvec` (77 bits) | same | `kFt2Ft4Rvec` (`:125-129`) |
| FEC / CRC | LDPC(128,90) / CRC-13 | LDPC(174,91) / CRC-14 (poly 0x2757) | same | same (`lib/crc14.cpp`) |
| start offset in slot | n/a | 150 ms (`Modulator/Modulator.cpp:80`) | 500 ms (`Modulator.cpp:80`) | 0 ms on Linux, 300 ms elsewhere (`Modulator.cpp:209-214`) |
| receive | n/a | FT4 decoder on audio stretched 2× by repeating each sample (`lib/decoder.f90:75, 1208-1218`); frequency ×2, DT `0.5*dt - 0.25` mapped back (`:1885-1886`) | own decoder | own decoder (`Detector/FT2DecodeWorker.cpp`, `FtxFt2Stage7.cpp`) |

Identical tables, checked mechanically:

- LDPC(174,91) generator (83 rows of 23 hex digits): SHA-256 (first 16 hex digits)
  `f05985e56cd2b176` in Decodium 4.0, WS 3.1.0, WS 3.2.1 and WSJT-X rc1.
- `rvec` (77 elements): element-for-element equal in Decodium 4.0, WS 3.2.1 and rc1.
- The GFSK construction (pulse `gfsk_pulse(1.0, t)` over 3 symbols, one-symbol
  raised-cosine ramps) is the same in `gen_ft4wave.f90` and Decodium's
  `generate_ft2_ft4_wave`.

WS's `genft4.f90` and `gen_ft4wave.f90` are not byte-identical to rc1's (183 differing
lines for `genft4.f90`); this was not pursued, since it does not bear on FT2.

## Call chains

**WS, transmit.** `widgets/mainwindow.cpp:9048-9060`: for `FT4` or `FT2`, `genft4_()`
produces the FT4 `itone`, then `gen_ft4wave_()` is called with `nsps = 4*576` (FT4)
or `4*288` (FT2) at 48 kHz. No FT2-specific encoder is called.

**WS, receive.** `lib/decoder.f90:1208`: `params%nmode.eq.52` repeats every sample
`FT2_STRETCH = 2` times into `id2_ft4` (`:1213`) and calls the FT4 decoder with
`tperiod = 3.75` (`:1216-1218`); `lib/ft4_decode.f90:208, 214, 246` treats 3.75 as FT2.

**The old `lib/ft2/` is not built.** `CMakeLists.txt` references one file from that
directory, `lib/ft2/gfsk_pulse.f90` (line 523 in 3.1.0, 560 in 3.2.1). `ft2.f90`,
`ft2_decode.f90`, `ft2_params.f90` and the rest are referenced nowhere. WSJT-X reached
the same conclusion and deleted them on the 3.2 line (`859f136b8`, 2026-07-21),
keeping `gfsk_pulse.f90` and `genft2.f90`, which moved to `lib/`.

**Decodium 4.0, transmit.** `mainwindow.cpp:17355-17358, 26053-26056`: `nsps = 4*288`,
`generateFt2Wave()` (`FtxWaveformGenerator.cpp:1758`, calling `generate_ft2_ft4_wave`).
Tones: `genft4_` (`FtxMessageEncoder.cpp:6807`), `encodeFt4`, `map_ft2_ft4_tones`
(`:2656-2688`).

## Decodium history (git log)

- 2026-02-21 `641e7d6` "Decodium 3 FT2 - rebrand, FT2 mode": the first FT2 commit. Its
  `lib/ft2/ft2_params.f90` already reads "LDPC(174,91) code, four 4x4 Costas arrays",
  `NSPS = 288`, `NN2 = 105`, `NMAX = 45000`; `helper_functions.cpp:6` has `105*288`.
  No 8-GFSK stage is visible in this history.
- 2026-02-23 `8b81f58`, `ac94ed6`: `lib/ft2/` deleted (a few leftovers such as
  `ft2.ini` remain in 4.0).
- 2026-03-12 `5dbfe19` TX timing fixes; 2026-03-19 `ee1bef6` a non-standard "Report + TU"
  payload convention (see below); 2026-03-20 `254c87b` "FT2 decoder v2"; 2026-03-23
  `2373fd8` logs FT2 as `MFSK` / `FT2` per ADIF 3.1.7.
- `git log -S` for `icos4a` and `FT2_STRETCH` found no commit that changed
  modulation, sync or FEC. Not every diff was read.

## Claims checked

| claim | verdict | basis |
|---|---|---|
| IU8LMC (2026-02-27): WS FT2 is 2 tones, 13.3 ms, 1.92 s, binary sync word, LDPC(128,90), CRC 13 | **refuted** as a description of what WS transmits | those numbers are the unbuilt `lib/ft2/ft2_params.f90` shipped in the tarball |
| IU8LMC: Decodium is 4 tones, 24 ms, 2.52 s, Costas arrays, LDPC(174,91), CRC 14 | **confirmed** | source |
| DG2YCB (2026-02-26): compatible; no IU8LMC code used | compatible: **supported by the source, untested on the air**. No code used: **undetermined** (the shared design follows from both being FT4-derived; code overlap was not checked) | the above |
| IU8LMC: "completely incompatible", "NO DECODE" | **not supported by the source**; no decode test was run | the above |
| Decodium Appendix A, PDF v1.0.262: 79 symbols, 1.896 s, three 7-symbol Costas arrays | **contradicted** by Decodium's own source; its arithmetic also fails (7+29+7+29+7 leaves 58 payload symbols = 116 bits, against a 174-bit codeword) | source |
| Decodium HTML manual, rev. 2026-07-25: 105 symbols (87+16+2), 24 ms, about 2.52 s | **confirmed** | source |
| ADIF "certified" FT2 | **partly**: ADIF 3.1.7 lists `MODE=MFSK, SUBMODE=FT2`, a logging name, not a modulation specification | ADIF |
| a mid-March change from 8-GFSK to 4-GFSK (third-party report) | **undetermined**; not visible in this history, which has 4 tones from 2026-02-21 | git log |

Decodium extends the payload: `ee1bef6` (2026-03-19) packs a report and "TU" into one
77-bit message using an unused `irpt` range (106-206) of the `igrid4` field, described
as decoding in WSJT-X as a normal report. Whether WS decodes it that way was not
checked.

## Open points

1. Real interoperability: WAVs generated by each side decoded by the other, with and
   without noise.
2. Sensitivity cost of WS's decoder-on-stretched-audio (sample repetition leaves
   images); unmeasured.
3. Whether the start-offset difference (150 ms against 0 or 300 ms) stays inside each
   decoder's DT search.
4. Equality of the 77-bit packing (WSJT-X `pack77` against Decodium's C++ `encodeFt4`)
   over all message types; only standard messages were looked at.
5. The first WS build with FT2 (260226 / 260228) is not available, so its behaviour
   cannot be compared with the current one.
6. The full Decodium diff history was not read.
7. MSHV and WSJT-Z were not examined.

## Decision

Issue #437 is closed as not planned: FT2 is not part of WSJT-X, there is no
specification of record (Decodium's own two revisions disagree), and the two
implementations dispute each other. `ProtocolId::Ft2 = 2` stays reserved in
`engine/protocol.rs`. If FT2 enters WSJT-X, the first step is to repeat this
comparison against that code; since both implementations are FT4 at 288 samples
per symbol, a port would start from the FT4 chain, and whether it can be
parameterised there was not examined.
