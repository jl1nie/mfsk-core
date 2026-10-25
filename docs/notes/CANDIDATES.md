# What `SlotResult::candidates` counts, per mode

`SlotResult::candidates` (and `MfskBudgetReport::candidates`, Kotlin `candidates`, Swift `candidates`) is the
number of candidates the **first coarse search the call ran** kept, after the search's own sync threshold and
its cap, before the budget, the dedup or any decode. It is what was searched, not what was missed: it is not a
recall figure, and it counts noise candidates too (#696 for FT8 / FT4 / FST4, #697 for the others). Silence is
`Some(0)` where a search ran; `None` where the call made none (FT8's prefix at checkpoint B only subtracts; Q65's
averaged call when the q3 decode on the averaged spectra had already decoded) and for FST4W.

| mode | what is counted | cap | upstream name | the test that fails if this is wrong |
|---|---|---|---|---|
| FT8 | the first `coarse_sync` (+ `promote_near_rx`): checkpoint A's, on the first 11.8 s, in a call that includes it; checkpoint C's, on what A's decodes leave, in the final call of a prefix sequence. The sniper: its narrow window's | `max_cand` | `ncand` (`ft8_decode.f90`) | `decode_method.rs::candidates_is_what_the_coarse_search_kept_for_that_call` |
| FT4, FST4 | the list after `ft4_coarse_sync` / `fst4_coarse_sync` and, for FST4's blanker, the band restriction (`cand_band`); the first level only | `max_cand` | `ncand` (`ft4_decode.f90`, `fst4_decode.f90`) | the same test (FT4 leg) |
| Q65 | the first `scan_with` list: the unsmoothed coarse search (`coarse_search_drift_on_spec_for`) — a bin if it is the best bin or `(s-ave)/rms >= 6` and a local maximum. A call with `ap_list` and `rx_freq` runs the q3 decode first (not a search). Averaged: each period's search on the running-average spectrogram | `max_candidates`, default 8 | `ncand` (`q65_decode.f90`) | `decode_candidates_slow.rs::q65_*` |
| WSPR | pass 0's list after `find_peaks` and the cap. Passes 1 and 2 search the residual again and are not counted, so 2 passes and 3 passes give the same number; `Deep` (`-d`, `more_candidates`) adds peaks. **Not narrowed to fmin..fmax** as `wsprd`'s `npk` is (`wsprd.c:1178-1184`), so it can exceed upstream's for the same audio | `max_candidates`, default 200 | `npk` (`wsprd.c:1149-1189`) | `decode_candidates_slow.rs::wspr_*` |
| JT9 | the coarse list after `truncate(max(max_candidates, 32))` | the floor of 32 | none (upstream loops over bins) | `decode_candidates_slow.rs::jt9_*` |
| JT65 | pass 1's list (`ipass == 1`); later passes search the subtracted residual and are not counted | `min(max_candidates, 50 / ipass)`, 8 by default | `ncand` (`jt65_decode.f90:323`) | `decode_candidates_slow.rs::jt65_*` |

**Per call.** One period can report different numbers in different calls: with FT8's `SicEarly` a whole-period call
and a prefix sequence differ (on `qso3_busy`: 480 whole, 480 at A, `None` at B, 338 at the end). A caller that wants a
period's figure takes the call it means; summing is the caller's, next to `PeriodBudget`.

**How it travels.** Frame modes: a `pub(crate)` field of `BudgetReport` (`note_candidates`, first note wins), moved into
`DecodeOutcome::candidates` by `DecodeOutcome::new`. JT9, JT65 and Q65: `BudgetGate::note_candidates`, returned by
`decode_reported`. Q65 averaging: `MultiPeriod::last_candidates`. WSPR: the third value of `decode_scan_inner`.
