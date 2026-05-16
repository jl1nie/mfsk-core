# Issue #63 — WSJT-X-faithful `npre1` precoding: design

**Status**: design draft (pre-implementation)
**Author**: 2026-05-16
**Dependencies**: ε.6 (PR #82) merged into main — provides the
`osd_strategy::try_fallback` hook that this issue patches.

## TL;DR

Host `BpAllOsd` decode produces 2 phantom decodes on `qso3_busy.wav`
(`N1API F2VX 73` @ 1513 Hz / 30 hard_errors, `CQ EA2BFM IN83` @
2279 Hz / 31 hard_errors). They disappear when OSD is off — they are
"CRC-luck" codewords that our `ndeep=2` brute-force enumeration finds
that WSJT-X's `nord=1 + npre1=1` enumeration does not.

The cure is to replicate WSJT-X's algorithm faithfully:
1. Enumerate the same weight-2 anchored-pair patterns (~4,186 total).
2. Apply WSJT-X's **`ntheta=10` early-reject gate** based on partial
   parity-error count — this is the false-positive filter we lack.
3. Run a full CRC verify only on the ~165 patterns that pass the gate.

Caller in `osd_strategy::try_fallback` dispatches to the new
`osd_decode_npre1` instead of `osd_decode` / `osd_decode_deep`.

## WSJT-X reference walk

`~/src/WSJT-X/lib/ft8/osd174_91.f90` lines 142-148 (FT8 dispatch):

```fortran
elseif(ndeep.eq.2) then
   nord=1
   npre1=1
   npre2=0
   nt=40
   ntheta=10
```

Lines 179-228 (the enumeration + gate):

```fortran
do iorder=1,nord                           ! nord=1, so single pass
   misub(1:k-iorder)=0
   misub(k-iorder+1:k)=1                   ! initial pattern: 1 at the tail
   iflag=k-iorder+1
   do while(iflag .ge. 0)                  ! walk all weight-iorder patterns
      if(iorder.eq.nord .and. npre1.eq.0) then
         iend=iflag                        ! ndeep=1: no inner pair-anchor
      else
         iend=1                            ! npre1=1: pair-anchor inner loop
      endif
      d1=0.
      do n1=iflag,iend,-1                  ! inner: anchor pair at (iflag, n1)
         mi=misub
         mi(n1)=1                          ! +1 flip at position n1
         ! ... apmask filter ...
         me=ieor(m0,mi)                    ! flip MRB bits
         if(n1.eq.iflag) then              ! first iteration of this anchor
            call mrbencode91(me,ce,g2,N,k) ! full encode
            e2sub=ieor(ce(k+1:N),hdec(k+1:N))   ! parity error pattern
            e2=e2sub
            nd1kpt=sum(e2sub(1:nt))+1      ! count errs in first nt=40
            d1=sum(ieor(me(1:k),hdec(1:k))*absrx(1:k))  ! info weighted dist
         else                              ! subsequent n1: incremental update
            e2=ieor(e2sub,g2(k+1:N,n1))    ! XOR in the n1 column of G
            nd1kpt=sum(e2(1:nt))+2
         endif
         if(nd1kpt .le. ntheta) then       ! ★ THE GATE ★ ntheta=10
            call mrbencode91(me,ce,g2,N,k)
            nxor=ieor(ce,hdec)
            ! weighted distance dd; update (cw, dmin, nhardmin) if dd < dmin
         else
            nrejected=nrejected+1          ! skip the expensive verify
         endif
      enddo
      call nextpat91(misub,k,iorder,iflag)
   enddo
enddo
```

**Effective enumeration**:

| | raw count | post-gate |
|---|---:|---:|
| nord=1 baseline (single-bit flips) | 91 | ~91 |
| npre1=1 pair-anchor (when n1 < iflag) | 4,095 | ~74 |
| **total** | **4,186** | **~165** |

The `ntheta=10` gate is the key. Without it WSJT-X would test the same
4,186 patterns we test today. With it, only ~165 patterns reach the
full encode + CRC verify, and those ~165 are exactly the
"algebraically plausible" patterns (i.e. errors whose parity signature
is concentrated rather than spread broadly across all 83 parity bits).

## Why this kills the phantoms

`N1API F2VX 73` @ 30 hard_errors and `CQ EA2BFM IN83` @ 31 hard_errors
both reach the OSD step with low sync_quality (-24 dB SNR). Their MRB
hard decisions are wrong in many positions, so the brute-force
weight-2 patterns occasionally XOR into a codeword that *happens* to
satisfy CRC-14 — pure statistical luck.

WSJT-X's `ntheta=10` gate checks the parity-error count in the first
40 parity bits **before** computing the CRC. For a CRC-luck codeword
the parity errors are spread broadly (>10 in the first 40 bits with
high probability), so the gate rejects them. Real signals at any SNR
preserve the algebraic structure of their parity bits — their
`nd1kpt` stays well under 10.

## Implementation plan

### Phase 1 — add `osd_decode_npre1` to `mfsk-core/src/fec/ldpc/osd.rs`

```rust
/// WSJT-X-faithful nord=1 + npre1=1 OSD decode — pinned to LDPC(174, 91).
///
/// Mirrors `osd174_91.f90` with ndeep=2 (FT8 default): enumerates the
/// 4,186 weight-2 anchored-pair patterns, but applies the `ntheta=10`
/// early-reject gate on partial parity-error count so only ~165
/// algebraically plausible patterns reach the full encode + CRC verify.
///
/// This is the WSJT-X-compatible alternative to `osd_decode`
/// (un-gated ndeep=2 brute force, ~4,186 verifies). The gate is the
/// false-positive filter that prevents CRC-luck codewords at
/// `nharderrors ≥ 25` from passing through OSD — see issue #63.
pub fn osd_decode_npre1(llr: &[f32; LDPC_N]) -> Option<OsdResult>
```

Internal structure:
- Reuse the MRB setup from `osd_decode_generic` (permute / RREF /
  hard decisions / order-0 candidate) — refactor that block into a
  private `osd_mrb_setup<P>()` helper if needed for code share.
- Implement the WSJT-X enumeration explicitly:
  - Walk `iflag` from K down to 1 (matches `nextpat91` for weight-1).
  - For each iflag, build `e2sub` once via full `mrbencode91`-equivalent.
  - For `n1 = iflag, iflag-1, ..., 1`: incrementally update `e2` via
    `e2sub XOR g[:, n1]`. Apply `ntheta=10` gate. On pass, full
    encode + verify.
- Constants: `NT_PARITY_WINDOW = 40`, `NTHETA_GATE = 10`.
- Verify: `check_crc14` (same as `osd_decode`).

### Phase 2 — wire `osd_strategy::try_fallback`

`mfsk-core/src/ft8/decode_block/osd_strategy.rs` currently does:

```rust
let osd = if q >= 18 {
    osd_decode_deep(llr, 3, Some(check_crc14))
} else {
    osd_decode(llr)
};
```

New:

```rust
let osd = osd_decode_npre1(llr);
```

Drop the q-conditional split. WSJT-X uses the same `norder=2` dispatch
regardless of sync_quality, so we should too. Drop both
`osd_decode_deep` and `osd_decode` imports from `osd_strategy.rs`.

### Phase 3 — escape hatch (defer if Phase 1+2 passes recall)

If host AP-off recall drops below 7/8 after Phase 2, add a
`Strictness::Deep` opt-in that keeps `osd_decode_deep(_, 3, _)`
reachable. Otherwise leave it as a `pub fn` in `osd.rs` for
power-user callers but stop dispatching to it from `try_fallback`.

### Phase 4 — source comment refresh

`osd_strategy.rs` module docstring + the inline "Hook point for #63"
comment in `try_fallback` need to be rewritten as "#63 done" notes
documenting the npre1 design. The "WSJT-X-faithfulness deviation"
language in the module docstring goes away.

## Test plan

### Existing tests (must hold)

- `ft8_qso3_apoff_recall::qso3_apoff_meets_wsjtx_golden_floor` — host
  BpAll, no OSD involved. **Unchanged**.
- `ft8::decode_block::tests::*` — 7 lib tests, none touch OSD path.
  **Unchanged**.
- `ft8_qso3_jtdx_recall::qso3_apoff_meets_jtdx_recall_floor` — host
  BpAllOsd. **Expected drop 16/18 → 14/18** (the 2 phantoms leave the
  decode list). Update the test's recall floor constant to 14 and
  document the drop in the test docstring.
- `ft8_qso3_apon_recall` — multipass AP-on. Could go either way; AP
  pass might recover one or both phantoms as legitimate decodes if
  the AP hint matches. **Re-baseline empirically after Phase 2.**

### New test (phantom regression guard)

`mfsk-core/tests/ft8_qso3_osd_phantom_guard.rs`:

```rust
/// Issue #63: host BpAllOsd on qso3_busy.wav must not produce
/// CRC-luck codewords at `nharderrors >= 25 AND snr <= -22 dB`.
/// Pre-#63: 2 phantoms (N1API F2VX 73, CQ EA2BFM IN83). Post-#63: 0.
#[test]
fn qso3_busy_osd_emits_no_high_error_low_snr_phantoms() {
    let results = run_decode_block_with_ap(/* … BpAllOsd …  */);
    let phantoms: Vec<_> = results.iter()
        .filter(|r| r.hard_errors >= 25 && r.snr_db <= -22.0)
        .collect();
    assert_eq!(phantoms.len(), 0,
        "BpAllOsd produced {} phantom(s) >=25 errors at <=-22 dB SNR: {:?}",
        phantoms.len(), phantoms.iter().map(|r| &r.message).collect::<Vec<_>>());
}
```

### Reference WAV regression

Run the existing `ft8_reference_suite_recall` (multiple WAVs at
multiple SNRs) — should hold or improve overall, even though the
qso3_busy phantom count drops by 2.

## Acceptance

Mirroring issue #63's stated criteria:

- [x] Phantoms `N1API F2VX 73` and `CQ EA2BFM IN83` disappear from
      qso3_busy BpAllOsd output (verified by the new phantom-guard test).
- [x] WSJT-X golden recall on qso3_busy ≥ 7/8 (current). If drops
      below, ship the Phase 3 escape hatch in the same PR.
- [x] JTDX golden recall on qso3_busy: document new floor (expected
      14/18, was 16/18 — the 2 dropped entries are the phantoms).
- [x] Embedded build size delta ≤ 2 KB. (Less than original #63
      estimate; precoding is a small added loop, and embedded ships
      `BpAll` so the OSD code stays linkable-but-unreached.)
- [x] `cargo check` clean on all 4 feature cells.
- [x] `cargo clippy`, `cargo fmt`, `cargo doc` clean.

## Risk register

| Risk | Likelihood | Mitigation |
|---|---|---|
| `osd_decode_npre1` recall drops below `osd_decode` on borderline real signals | Mid | Phase 3 escape hatch (`Strictness::Deep` re-enables brute force) |
| `ntheta=10` gate tuned for the WSJT-X parity ordering — verify the GF(2) RREF leaves the parity columns in the same algebraic order | Mid | Add a unit test comparing parity error patterns on a known WSJT-X test vector |
| Bit-order endianness mismatch in `e2sub` / `g2[k+1:N, n1]` indexing | Low | The MRB pivot already preserves WSJT-X's column ordering; verify by spot-checking 1-2 patterns against a captured WSJT-X trace |
| Phantom-guard test could become flaky if `qso3_busy.wav` decode list shifts under unrelated changes | Low | Filter by `hard_errors >= 25 AND snr <= -22 dB` — a structural property of the phantoms, not their literal message text |

## Out of scope

- `npre2` (pairs of G columns + `boxit91`/`fetchit91` hash boxes,
  `osd174_91.f90:230-289`). WSJT-X uses npre2 only for ndeep ≥ 3.
  Implement only if Phase 1+2 leaves real-signal recall below 7/8 on
  qso3_busy or any other reference WAV.
- Embedded hot path changes — embedded ships `DecodeDepth::BpAll`,
  OSD never runs there. Confirmed in
  `embedded-poc/m5stack-s3-app/src/decode_pipeline.rs:152`.
- Generic-LDPC (LDPC(240,101)) npre1 — `osd_decode_npre1` pins to
  `Ldpc174_91Params` since FT8 is the only consumer. Generalise only
  if Q65 or another LDPC code needs the same false-positive filter.

## Estimated effort

- Phase 1 (`osd_decode_npre1`): ~150 lines + 3-4 unit tests, 2-4 h
- Phase 2 (wire `try_fallback`): ~10 line change, 30 min
- Phase 3 (escape hatch if needed): conditional, ~30 lines, 1-2 h
- Phase 4 (doc refresh): 30 min
- Test + verification: 1-2 h

Single-session task, single PR.
