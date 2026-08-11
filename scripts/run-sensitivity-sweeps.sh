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
declare -A SUITES=(
  [ft8]="ft8_sweep ft8_no_nsym3_sweep"
  [ft4]="ft4_sweep ft4_snr_sweep ft4_diag_low_snr ft4_timing_budget"
  [fst4]="fst4_sweep"
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

[ -n "$LOG" ] && : > "$LOG"
fail=0
for k in "${want[@]}"; do
  bins="${SUITES[$k]:-}"
  if [ -z "$bins" ]; then echo "unknown group: $k" >&2; fail=1; continue; fi
  for b in $bins; do
    [ -f "mfsk-core/tests/$b.rs" ] || continue
    echo
    echo "───── $k / $b ─────"
    # `--ignored` because every tier-C test is #[ignore]d by design.
    run cargo test --release -p mfsk-core --features "$FEATURES" \
        --test "$b" -- --ignored --nocapture || fail=1
  done
done

echo
if [ "$fail" -ne 0 ]; then
  echo "one or more sweeps failed to run (see above)."
  exit 1
fi
cat <<'MSG'
== done ==

These print tables and assert nothing. Before tagging:

  1. Compare against docs/notes/*BENCHMARK.md and the previous
     release's numbers. A threshold that moved by more than ~0.5 dB
     in the wrong direction is worth explaining before shipping.
  2. Update docs/notes/BENCHMARKS.md if the numbers moved for a
     reason you understand (new hardware counts as a reason — the
     table records the machine).
  3. Then follow the tag sequence in CLAUDE.md.
MSG
