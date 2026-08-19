#!/usr/bin/env bash
# Tier C — sensitivity sweeps. Run this before cutting a release.
#
# WHY THIS IS A SCRIPT AND NOT A CI JOB
#
# These sweeps need the generated corpora under
# `embedded-poc/assets/*_sweep/` — ~17 GB locally, gitignored, and
# produced by WSJT-X's own Fortran simulators (`jt65sim`, `fst4sim`,
# `ft4sim`, …) which have to be built first. CI cannot hold or
# rebuild that, which is why every one of these tests silently skips
# there.
#
# A nightly workflow was considered and rejected: on a solo, bursty
# repo most nights would re-measure unchanged code, and this project
# has already deleted one scheduled tier for exactly that reason —
# see ci.yml's note that it "only ever cost wall-clock for output
# nobody was routinely reading". Releases are throttled to roughly
# biweekly, which is a real interval with a real decision attached:
# shipping a sensitivity regression to crates.io is the outcome worth
# spending minutes to avoid.
#
# WHAT IT DOES NOT DO
#
# It does not assert. These tests print tables; sensitivity is a
# curve, and a threshold that moved 0.3 dB is a judgement call, not a
# boolean. Read the output against `docs/notes/*BENCHMARK.md` and
# against the previous release's numbers.
#
# Usage:
#   scripts/run-sensitivity-sweeps.sh              # everything present
#   scripts/run-sensitivity-sweeps.sh ft4 fst4     # only these
#   MFSK_SWEEP_LOG=out.txt scripts/run-sensitivity-sweeps.sh
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

LOG="${MFSK_SWEEP_LOG:-}"
FEATURES="full,internal-testing"

# Tier-C binaries, grouped so a caller can ask for one protocol.
#
# Entries as "bin" run every #[ignore]d fn in that file. Entries as
# "bin:fn" run only that one fn (cargo test's positional name filter).
# The `:fn` form exists because ft8_sweep.rs / ft4_sweep.rs /
# fst4_sweep.rs each accumulate one-off #[ignore]d diagnostic probes
# from past closed investigations (issue #146/#148/#244/#245/#306/#308/
# #310/#311/#312 for fst4 alone — 47 of them as of 2026-08-19, growing
# with every new investigation) alongside the one fn that's an actual
# recall-vs-SNR regression gate. Without the filter this script
# re-runs all of that dead-investigation code on every release sweep
# — fst4_sweep.rs's ignored-fn pile (1 real sweep + N diagnostics) was
# most of why a full fst4 sweep took ~40 minutes on 2026-08-14, and
# will keep growing as more diagnostics land — this filter is meant to
# stay correct without hand-updating a count. The diagnostic fns
# aren't deleted — still runnable by name for whoever revisits that
# issue — just not part of the automated release gate.
declare -A SUITES=(
  [ft8]="ft8_sweep:ft8_snr_sweep ft8_no_nsym3_sweep"
  [ft4]="ft4_sweep:ft4_snr_sweep ft4_snr_sweep ft4_diag_low_snr ft4_timing_budget"
  [fst4]="fst4_sweep:fst4_snr_sweep fst4_sim_roundtrip"
  [wspr]="wspr_sweep"
  [jt65]="jt65_sweep"
  [jt9]="jt9_sweep"
  [q65]="q65_sim_sweep q65_snr_sweep q65_ap_sweep"
  [msk144]="msk144_snr_sweep"
  [uvpacket]="uvpacket_per_modes_sweep uvpacket_snr_calibration"
  [fec]="ldpc_min_sum"
  [bench]="bench_qso3_busy_timing"
)

want=("$@")
[ ${#want[@]} -eq 0 ] && want=("${!SUITES[@]}")

# Corpora these need, so a missing one is reported up front rather
# than as a wall of silent skips.
declare -A CORPUS=(
  [ft8]=ft8_sweep [ft4]=ft4_sweep [fst4]=fst4_sweep [wspr]=wspr_sweep
  [jt65]=jt65_sweep [jt9]=jt9_sweep [q65]=q65_sweep [bench]=fst4_sweep
)

echo "== tier C sensitivity sweeps =="
missing=()
for k in "${want[@]}"; do
  c="${CORPUS[$k]:-}"
  [ -z "$c" ] && continue
  if ! compgen -G "embedded-poc/assets/$c/*.wav" >/dev/null 2>&1; then
    missing+=("$k (embedded-poc/assets/$c — scripts/gen_${c%_sweep}_sweep_wavs.sh)")
  fi
done
if [ ${#missing[@]} -gt 0 ]; then
  echo "corpora absent — these groups will report nothing:"
  printf '  - %s\n' "${missing[@]}"
  echo "generate with scripts/gen_*_sweep_wavs.sh (build the simulators first:"
  echo "scripts/build_*sim.sh), or pass only the groups you have."
  echo
fi

run() {
  if [ -n "$LOG" ]; then "$@" 2>&1 | tee -a "$LOG"; else "$@"; fi
}

# Every recall-vs-SNR sweep this script runs can also dump a per-trial
# CSV (`MFSK_*_SWEEP_CSV` for the ft8/ft4/fst4 sweeps that already had
# it for bootstrap-CI purposes, `MFSK_*_SWEEP_SUMMARY_CSV` added
# alongside for the rest). Point them all at $CSV_DIR so
# sweep-regression-check.py can diff this run against the last
# baseline afterward — replaces manually eyeballing the printed tables
# against docs/notes/*BENCHMARK.md.
CSV_DIR="$REPO_ROOT/target/sweep-csv"
rm -rf "$CSV_DIR"
mkdir -p "$CSV_DIR"

# Narrow re-run: once docs/notes/sweep-baseline.json has a crossing
# for a group, there's no need to re-sweep its already-known 0%/100%
# plateaus every release — scripts/sweep-narrow-plan.py turns the
# baseline into per-mode SNR windows (±3 dB by default) for FST4/FT8/
# FT4. This is the automated version of the manual "re-verifying a
# fix" narrowing docs/notes/FST4_BENCHMARK.md section 7 already
# recommends — FST4-300 alone is why this matters (it's the slowest
# sub-mode by far, and until now every full sweep re-swept it end to
# end regardless of how little the last several releases moved).
# A mode absent from the baseline (new sub-mode, first-ever run) gets
# no line here, so its cargo test invocation below falls through to
# the full generated grid — narrowing never silently drops coverage,
# only redundant SNR points once a baseline exists.
# Set MFSK_SWEEP_FULL=1 to always use the full grid regardless.
NARROW_PLAN=""
if [ -z "${MFSK_SWEEP_FULL:-}" ] && [ -f "$REPO_ROOT/docs/notes/sweep-baseline.json" ] \
   && command -v python3 >/dev/null 2>&1; then
  NARROW_PLAN="$(python3 "$REPO_ROOT/scripts/sweep-narrow-plan.py" 2>/dev/null || true)"
fi

narrow_window() {
  # $1 = proto (fst4|ft8|ft4), $2 = mode (fst4 only, else empty)
  # prints "lo\thi" on stdout, nothing (and returns 1) if no plan line.
  [ -z "$NARROW_PLAN" ] && return 1
  printf '%s\n' "$NARROW_PLAN" | awk -F'\t' -v p="$1" -v m="$2" \
    '$1==p && $2==m {print $3"\t"$4; found=1} END{exit !found}'
}

[ -n "$LOG" ] && : > "$LOG"
fail=0
for k in "${want[@]}"; do
  entries="${SUITES[$k]:-}"
  if [ -z "$entries" ]; then echo "unknown group: $k" >&2; fail=1; continue; fi
  for entry in $entries; do
    b="${entry%%:*}"
    filt=""
    [ "$entry" != "$b" ] && filt="${entry#*:}"
    [ -f "mfsk-core/tests/$b.rs" ] || continue

    unset MFSK_FT8_SWEEP_CSV MFSK_FT4_SWEEP_CSV MFSK_FST4_SWEEP_CSV \
          MFSK_WSPR_SWEEP_SUMMARY_CSV MFSK_JT65_SWEEP_SUMMARY_CSV \
          MFSK_JT65_CHASE_SWEEP_SUMMARY_CSV MFSK_JT9_SWEEP_SUMMARY_CSV \
          MFSK_Q65_SWEEP_SUMMARY_CSV MFSK_MSK144_SWEEP_SUMMARY_CSV
    case "$b:$filt" in
      ft8_sweep:ft8_snr_sweep) export MFSK_FT8_SWEEP_CSV="$CSV_DIR/ft8.csv" ;;
      ft4_sweep:ft4_snr_sweep) export MFSK_FT4_SWEEP_CSV="$CSV_DIR/ft4.csv" ;;
      fst4_sweep:fst4_snr_sweep) export MFSK_FST4_SWEEP_CSV="$CSV_DIR/fst4.csv" ;;
      wspr_sweep:) export MFSK_WSPR_SWEEP_SUMMARY_CSV="$CSV_DIR/wspr.csv" ;;
      jt65_sweep:)
        export MFSK_JT65_SWEEP_SUMMARY_CSV="$CSV_DIR/jt65.csv"
        export MFSK_JT65_CHASE_SWEEP_SUMMARY_CSV="$CSV_DIR/jt65_chase.csv"
        ;;
      jt9_sweep:) export MFSK_JT9_SWEEP_SUMMARY_CSV="$CSV_DIR/jt9.csv" ;;
      q65_sim_sweep:) export MFSK_Q65_SWEEP_SUMMARY_CSV="$CSV_DIR/q65.csv" ;;
      msk144_snr_sweep:) export MFSK_MSK144_SWEEP_SUMMARY_CSV="$CSV_DIR/msk144.csv" ;;
    esac

    echo

    # fst4_sweep: one narrowed invocation per mode present in the
    # baseline, falling back to the full grid for any mode that isn't.
    if [ "$b:$filt" = "fst4_sweep:fst4_snr_sweep" ] && [ -n "$NARROW_PLAN" ]; then
      any_narrowed=0
      for mode in 15 30 60 120 300; do
        win="$(narrow_window fst4 "$mode")" || continue
        any_narrowed=1
        lo="${win%%$'\t'*}"; hi="${win#*$'\t'}"
        echo "───── $k / $b (mode=$mode, narrowed ${lo}..${hi} dB) ─────"
        MFSK_FST4_SWEEP_MODES="$mode" MFSK_FST4_SWEEP_SNR_MIN="$lo" MFSK_FST4_SWEEP_SNR_MAX="$hi" \
          run cargo test --release -p mfsk-core --features "$FEATURES" \
              --test "$b" "$filt" -- --ignored --nocapture || fail=1
      done
      # Any mode the baseline doesn't know yet still needs a full pass.
      known_modes="$(printf '%s\n' "$NARROW_PLAN" | awk -F'\t' '$1=="fst4"{print $2}')"
      for mode in 15 30 60 120 300; do
        echo "$known_modes" | grep -qx "$mode" && continue
        echo "───── $k / $b (mode=$mode, no baseline yet — full grid) ─────"
        MFSK_FST4_SWEEP_MODES="$mode" \
          run cargo test --release -p mfsk-core --features "$FEATURES" \
              --test "$b" "$filt" -- --ignored --nocapture || fail=1
      done
      continue
    fi
    if { [ "$b:$filt" = "ft8_sweep:ft8_snr_sweep" ] || [ "$b:$filt" = "ft4_sweep:ft4_snr_sweep" ]; } \
       && win="$(narrow_window "${b%_sweep}" '')"; then
      lo="${win%%$'\t'*}"; hi="${win#*$'\t'}"
      proto_upper="$(printf '%s' "${b%_sweep}" | tr '[:lower:]' '[:upper:]')"
      echo "───── $k / $b (narrowed ${lo}..${hi} dB) ─────"
      run env "MFSK_${proto_upper}_SWEEP_SNR_MIN=$lo" "MFSK_${proto_upper}_SWEEP_SNR_MAX=$hi" \
          cargo test --release -p mfsk-core --features "$FEATURES" \
              --test "$b" "$filt" -- --ignored --nocapture || fail=1
      continue
    fi

    if [ -n "$filt" ]; then
      echo "───── $k / $b ─────"
      # `--ignored` because every tier-C test is #[ignore]d by design.
      # Positional filter skips this binary's other #[ignore]d fns —
      # see the SUITES comment above for why.
      run cargo test --release -p mfsk-core --features "$FEATURES" \
          --test "$b" "$filt" -- --ignored --nocapture || fail=1
    else
      echo "───── $k / $b ─────"
      run cargo test --release -p mfsk-core --features "$FEATURES" \
          --test "$b" -- --ignored --nocapture || fail=1
    fi
  done
done

echo
if [ "$fail" -ne 0 ]; then
  echo "one or more sweeps failed to run (see above)."
  exit 1
fi

shopt -s nullglob
csvs=("$CSV_DIR"/*.csv)
shopt -u nullglob
if [ "${#csvs[@]}" -gt 0 ] && command -v python3 >/dev/null 2>&1; then
  echo
  echo "== regression check vs docs/notes/sweep-baseline.json =="
  run python3 "$REPO_ROOT/scripts/sweep-regression-check.py" "${csvs[@]}"
fi

cat <<'MSG'

== done ==

These print tables and assert nothing. Before tagging:

  1. Read the regression-check table above (or re-run it: `python3
     scripts/sweep-regression-check.py target/sweep-csv/*.csv`). A
     group flagged "!!" moved >=0.5 dB from the stored baseline and is
     worth explaining before shipping; groups marked NEW have no
     baseline entry yet.
  2. Also sanity-check against docs/notes/*BENCHMARK.md prose and the
     previous release's numbers — the JSON baseline only covers
     50%-crossing SNR, not e.g. WSPR's phantom-decode count.
  3. Once you understand why the numbers are what they are, refresh
     the baseline: `python3 scripts/sweep-regression-check.py
     --update-baseline target/sweep-csv/*.csv`, and update
     docs/notes/BENCHMARKS.md if it moved for a reason you understand
     (new hardware counts as a reason — the table records the
     machine).
  4. Then follow the tag sequence in CLAUDE.md.
MSG
