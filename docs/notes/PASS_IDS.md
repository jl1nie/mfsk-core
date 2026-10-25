# What `RowDetail::pass` means, per mode

`RowDetail::pass` (and `MfskDecode::pass`, `DecodeResult::pass`) is a mode's own identifier of the
decode pass that produced a row. It means something else in every mode, and FT4 and FST4 give some ids two
meanings, so **an application reads `RowDetail::method`** (`DecodeMethod`, #696). This table is where the numbers
come from, for diagnostics and for anyone matching on them today. Read from the producing code on 2026-10-25;
the code is the authority.

| mode | `pass` | meaning | `method` |
|---|---|---|---|
| FT8 | 0, 3, 1, 2, 4 (tried in this order) | BP on `llra`, `llrd`, `llrb` (nsym 2), `llrc` (nsym 3), `llre` | `Bp` |
| FT8 | 14..=18, 19..=23 | OSD on `zsave(:,1)`, `zsave(:,2)` (BP sum after 1, 2 iterations) for variants a..e (`osd_strategy.rs`: `PASS_ID_OSD_ZSAVE1_A` + variant, `..ZSAVE2_A` + variant) | `Osd` |
| FT8 | 5, 6..=11, 12 | a-priori: 5 the call alone, 6..=11 `msg::pipeline_ap::ap_passes` (DX call alone, `CQ` + DX call, `MyCall DxCall`, `... RRR`, `... RR73`, `... 73`), 12 the blind CQ | `Ap { iaptype }` (below) |
| FT8 | 30, 31 | a7, a8 list decode (`ft8::list_decode::PASS_ID_A7`, `PASS_ID_A8`); the sync fields are `None` | `ListA7`, `ListA8` |
| FT4, FST4 | 0, 1, 6, 2, 3 (tried in this order) | BP on `llra`, `llrb`, `llre` (nsym 4, FST4 only), `llrc` (nsym `LLR_NSYM_MAX`), `llrd` (not on FT4) | `Bp` |
| FT4, FST4 | 4, 5 | OSD at depth 2, depth 3 (`engine/pipeline.rs`) | `Osd` |
| FT4, FST4 | 12, 5, 8, 9, 11, 10 | QSO-context a-priori for `iaptype` 1..6 (`decoder/frame.rs::pass_of`); a free-form `ap_hint` uses 6..=11 (`ap_passes`) | `Ap { iaptype }` |
| FST4W | 0..=3 | the LLR variant (nsym 1, 2, 4, 8), **not a decode pass** | `None` |
| WSPR, JT9, JT65, Q65 | 0 | nothing recorded | `None` |

**The collisions.** On FT4 and FST4, **5** is both OSD depth 3 and the a-priori pass of `iaptype` 2, and **6** is
both the nsym-4 BP rung (FST4) and a free-form hint's pass. Across modes, **4** is BP on `llre` in FT8 and OSD depth 2
in FT4. `method` is set where the row is made, so none of these is a guess: the FT4 CCIR sweep at `Depth::Deep` with a
hint gives rows with `pass == 5` that are `Osd` and rows with `pass == 5` that are `Ap { iaptype: Some(2) }`
(`tests/decode_method.rs`).

**`iaptype`.** The inverse of `pass_of` for the a-priori ids: 12 → 1, 5 → 2, 8 → 3, 9 → 4, 11 → 5, 10 → 6 (WSJT-X's
`iaptype`: `CQ ...`, `MyCall ...`, `MyCall DxCall ...`, `... RRR`, `... 73`, `... RR73`). 6 and 7 (the DX call alone,
`CQ` + the DX call: an `ap_hint` with no upstream number) give `iaptype: None`.

**Not in `pass`, or in `method`.** A SIC round, an early window (`RowDetail::stage` has it), the noise-blanker level,
WSPR's `SicPass` and JT65's `ipass`. The native results carry more than `RowDetail` does: Q65's `ap` and `iterations`,
WSPR's `drift_hz`, FST4W's `keff`.
