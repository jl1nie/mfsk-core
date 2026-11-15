# Agent log

One row per finished task, written by `scripts/agent-log.sh` (see `docs/notes/AGENT_WORKFLOW.md` §7). Tokens are measured
from the session transcript and its subagents: input + cache creation + output per model, cache reads in parentheses.
`?` means not recorded, never guessed. The whole transcript of a session is counted, so a session that did several
tasks charges each row with everything up to that point: compare rows from separate sessions, or read the difference
between rows of one. Reviewed at each release (`scripts/agent-log.sh --summary <last tag>`): at most one process change,
then compare.

| date | task | mode | commits | tokens by model | session | gate first time | review found | fixes after push | user corrections | note |
|---|---|---|---|---|---|---|---|---|---|---|
| 2026-10-25 | #692 #693 PeriodBudget #696 #697-item1 (one session, 2026-10-25) | port+design | 16 | claude-opus-5-5: 1835k (+36.5M cached); claude-sonnet-5-5: 2251k (+261.4M cached) | 53889a44 | ? | doc contradictions + weak test legs (#697) | ? | ? | seed row: whole session, several tasks, not separable |
| 2026-10-27 | #700 Encoded private fields | design | 3 | claude-opus-5-5: 1500k (+110.4M cached) | c87781aa | no | found: doc claim wider than measured + 32-bit ramp overflow; 5 minor | 0 | 0 |  |
| 2026-10-28 | #712 Row/SlotResult::new + #708 quick starts (part) | design | 5 | claude-opus-5-5: 477k (+16.1M cached) | 62f30928 | yes | found: false §1.4 banner (0.14 lacks Encoded::tones), CHANGELOG overstated scope, tautological asserts; 2 minor | 0 | 0 | #708 left open: recording example (#709), external-app check |
| 2026-10-28 | #711 part 1: band from mode defaults (docs + test) | design | 2 | claude-opus-5-5: 576k (+31.8M cached) | 62f30928 | yes | none | 0 | 0 | probe: inf band panics FT8/FST4/Q65, hangs JT9/JT65; FST4W NaN rx/tol drops decode |
| 2026-10-28 | #717 normalise DecodeParams at the Decoder | design | 3 | claude-opus-5-5: 930k (+43.0M cached) | 62f30928 | yes | none | 0 | 0 | found by #711 probe; CHANGELOG had two Skimmer headings, docs went in a 2nd commit |
| 2026-10-31 | release 0.15.0 (PR #719, tier-C, tag) | design | 3 | claude-sonnet-5-5: 148k (+3.3M cached) | e2eea6e0 | yes | n/a | 0 | ? |  |
| 2026-11-02 | gitignore jt9 root outputs (#720) | port | 1 | claude-opus-5-5: 384k (+9.6M cached) | d7707e0a | yes | n/a | 0 | 0 | root decoded.txt/jt9_wisdom.dat untracked; sweep copies kept |
| 2026-11-03 | #721 FT8 jt9 gap (dedupe DT, review fixes, tier C) + release 0.15.1 (PRs #723 #724) | port+design | 9 | claude-opus-5-5: 233k (+6.6M cached); claude-sonnet-5-5: 472k (+38.8M cached) | 3fe66e1c | no | found: multipass duplicate not subtracted + false budget reason, no noise-slot guard for blind-CQ, stale text; 4 minor | 0 | ? | gate failed once on ft8_decode_block_real_qso (dedupe displaced OH3NIV at cap 30) -> threshold 60; tier C needed a corpus stamp; whole session counted |
| 2026-11-14 | #749 FT4 tx_buffer_start = WSJT-X delay_ms | port+design | 3 | claude-opus-5-5: 328k (+15.7M cached) | 79fd0f6d | yes | found 3 low: dt0_buffer_start unasserted, -0.152 s claimed for WSJT-X unmeasured, stale comments | 1 | 1 | user corrected: patch releases are outside the cadence; do not re-raise |
| 2026-11-15 | wsprd parity (#753): merge main, gate, verify.sh, PR, CI, merge | port | 7 | claude-sonnet-5-5: 118k (+2.5M cached) | 6a7ae61d | yes | n/a | 0 | 0 | implementation (8 commits) came from an earlier session; this row is only merge+verify+PR. tier C WSPR sweep run afterwards on main 4b4d5ac0: -31.40 dB, +0.00 vs baseline, 1 phantom in 260 trials (not traced); wspr_scan.rs not flashed. (An earlier version of this note said the corpus was absent: wrong, it is on this machine.) |
